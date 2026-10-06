"""
状态估计与扰动观测模块
======================
KalmanFilter2D : 4 状态 [x, y, vx, vy] 常速度模型卡尔曼滤波，
                 输出滤波位置/速度，替代直接差分 + EMA（EMA 保留为 fallback）。
FirstOrderESO : 一阶过阻尼对象 x_dot = b0*(u+d)，b0=1/c。
ESO1D         : 原三状态二阶对象 observer，原样保留用于 legacy A/B。

位置 mm，时间 s，力 µN；c 的单位 µN*s/mm，b0 为 mm/(µN*s)。
过阻尼代数式 c*v=F+d 只产生一个位置积分器，不能推得 x_ddot=(F+d)/c。
ESO1D 的历史阶次不匹配由专项回归保留；新控制默认使用 FirstOrderESO。
"""

import math

import numpy as np


class KalmanFilter2D:
    """常速度模型卡尔曼滤波：状态 [x, y, vx, vy]，观测 [x, y]"""

    def __init__(self, q_pos=1e-4, q_vel=5e-3, r_meas=0.005):
        self.q_pos = float(q_pos)  # 过程噪声位置方差 (mm²)
        self.q_vel = float(q_vel)  # 过程噪声速度方差 (mm²/s²)
        self.r_meas = float(r_meas)  # 观测噪声方差 (mm²)
        self.x = np.zeros(4)  # [x, y, vx, vy] (mm, mm/s)
        self.P = np.eye(4) * 1.0
        self.initialized = False

    def reset(self, x_mm=0.0, y_mm=0.0):
        self.x = np.array([x_mm, y_mm, 0.0, 0.0])
        self.P = np.eye(4) * 1.0
        self.initialized = True

    def step(self, dt, z=None):
        """dt: 实测周期(s)。z: 观测 (x_mm, y_mm) 或 None（丢失时仅预测）。
        返回 (pos(2,), vel(2,))"""
        dt = max(float(dt), 1e-4)
        if not self.initialized:
            if z is not None:
                self.reset(z[0], z[1])
            return self.x[:2].copy(), self.x[2:].copy()
        # 预测
        F = np.array(
            [[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float
        )
        Q = np.diag(
            [self.q_pos * dt, self.q_pos * dt, self.q_vel * dt, self.q_vel * dt]
        )
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        # 更新
        if z is not None:
            H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=float)
            R = np.eye(2) * self.r_meas
            y = np.asarray(z, float) - H @ self.x
            S = H @ self.P @ H.T + R
            K = self.P @ H.T @ np.linalg.inv(S)
            self.x = self.x + K @ y
            self.P = (np.eye(4) - K @ H) @ self.P
        return self.x[:2].copy(), self.x[2:].copy()


def fal(e, alpha, delta):
    """ESO fal 函数：小误差线性段 + 大误差幂次，避免微分奇异"""
    e = float(e)
    if abs(e) <= delta:
        return e / (delta ** (1.0 - alpha))
    return math.copysign(abs(e) ** alpha, e)


class ESO1D:
    """单轴三阶 ESO。
    动力学假设：ẍ = b0·(u + d)，b0 = 1/c（c 为斯托克斯阻力系数，
    见模块 docstring）。z3 为扰动力的估计（力单位）。
    离散实现，使用真实 dt。"""

    def __init__(self, b0, omega0, fal_delta, dist_limit, beta=None):
        self.b0 = float(b0)
        if beta is None:
            w = float(omega0)
            self.beta1, self.beta2, self.beta3 = 3.0 * w, 3.0 * w * w, w**3
        else:
            self.beta1, self.beta2, self.beta3 = beta
        self.delta = float(fal_delta)
        self.dist_limit = float(dist_limit)
        self.z1 = 0.0  # 位置估计
        self.z2 = 0.0  # 速度估计
        self.z3 = 0.0  # 扰动力估计（力单位）
        self.initialized = False

    def reset(self, x0=0.0):
        self.z1, self.z2, self.z3 = float(x0), 0.0, 0.0
        self.initialized = True

    def step(self, dt, x_meas, u):
        """dt(s)，x_meas 位置量测，u 本帧实际施加力（力单位）。
        返回 z3（扰动力估计，已限幅）。"""
        dt = max(float(dt), 1e-4)
        if not self.initialized:
            self.reset(x_meas)
            return self.z3
        e1 = self.z1 - float(x_meas)
        self.z1 += dt * (self.z2 - self.beta1 * e1)
        self.z2 += dt * (
            self.b0 * (float(u) + self.z3) - self.beta2 * fal(e1, 0.5, self.delta)
        )
        self.z3 -= dt * self.beta3 * fal(e1, 0.25, self.delta)
        self.z3 = float(np.clip(self.z3, -self.dist_limit, self.dist_limit))
        return self.z3


class FirstOrderESO:
    """Predict/correct observer for x_dot=b0*(u+d), with d in force units.

    z1 is position [mm]; z2 is disturbance [µN], not velocity. Gains are
    l1=2*omega [1/s], l2=omega**2/b0 [µN/(mm*s)]. Input u is the previous
    execution interval's model force [µN]. No target force is consumed.

    One correction is Schur stable for 0<T*omega<sqrt(8)-2. Subintervals
    bound T*omega to 0.5, with linearly interpolated measurements and held u.
    A missing vision sample or dt beyond max_dt reanchors position and clears
    disturbance on recovery; unknown gap dynamics cannot become an innovation.
    omega is a software setting, not a measured hardware bandwidth.
    """

    def __init__(
        self,
        b0: float,
        omega0: float,
        dist_limit: float,
        *,
        max_dt: float = 0.15,
    ) -> None:
        values = (b0, omega0, dist_limit, max_dt)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("FirstOrderESO parameters must be finite")
        if b0 <= 0 or omega0 <= 0 or dist_limit < 0 or max_dt <= 0:
            raise ValueError("b0, omega0, max_dt must be positive; limit nonnegative")
        self.b0 = float(b0)
        self.omega0 = float(omega0)
        self.l1 = 2.0 * self.omega0
        self.l2 = self.omega0**2 / self.b0
        self.dist_limit = float(dist_limit)
        self.max_dt = float(max_dt)
        self.z1 = 0.0
        self.z2 = 0.0
        self.initialized = False
        self._last_y = 0.0
        self._gap = False

    def reset(self, x0: float = 0.0) -> None:
        """Anchor measured position and clear disturbance at start/recovery."""
        if not math.isfinite(x0):
            raise ValueError("position must be finite")
        self.z1 = self._last_y = float(x0)
        self.z2 = 0.0
        self.initialized = True
        self._gap = False

    def mark_gap(self) -> None:
        """Require reanchoring on the next valid vision sample."""
        self._gap = True

    def step(self, dt: float, x_meas: float, u: float) -> float:
        """Return disturbance [µN] from endpoint position and prior force."""
        dt, y, u = float(dt), float(x_meas), float(u)
        if not all(math.isfinite(value) for value in (dt, y, u)) or dt <= 0:
            raise ValueError("dt must be positive; dt, measurement and u finite")
        if not self.initialized or self._gap or dt > self.max_dt:
            self.reset(y)
            return self.z2
        steps = max(1, math.ceil(dt * self.omega0 / 0.5))
        interval = dt / steps
        start_y = self._last_y
        for index in range(1, steps + 1):
            # Exact endpoint for steps=1 retains the stated predict/correct law.
            sample = y if index == steps else start_y + (y - start_y) * index / steps
            predicted = self.z1 + interval * self.b0 * (u + self.z2)
            error = sample - predicted
            self.z1 = predicted + interval * self.l1 * error
            self.z2 = float(
                np.clip(
                    self.z2 + interval * self.l2 * error,
                    -self.dist_limit,
                    self.dist_limit,
                )
            )
        self._last_y = y
        return self.z2
