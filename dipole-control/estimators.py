# -*- coding: utf-8 -*-
"""
状态估计与扰动观测模块
======================
KalmanFilter2D : 4 状态 [x, y, vx, vy] 常速度模型卡尔曼滤波，
                 输出滤波位置/速度，替代直接差分 + EMA（EMA 保留为 fallback）。
ESO1D         : 单轴三阶 ESO（扩张状态观测器）。

ESO 动力学假设（重要，与低雷诺数过阻尼系统一致，见需求 九）：
    磁珠准静态动力学：c·v = F + d   （c = 斯托克斯阻力系数，d = 广义扰动力）
    →  ẍ = (F + d)/c ≡ b0·u + f，  b0 = 1/c，f = d/c
    即从磁力 u 到位置 x 等效"双积分器 + 输入增益 b0 = 1/c"，
    因此采用标准三阶 ESO 结构（β1=3ω0, β2=3ω0², β3=ω0³），
    但 b0 取 1/c 而非 1/m。z3 以**力为单位**（d 而非 f）估计，补偿时直接减。
    全部离散实现，使用真实 dt。
"""

import math

import numpy as np


class KalmanFilter2D:
    """常速度模型卡尔曼滤波：状态 [x, y, vx, vy]，观测 [x, y]"""

    def __init__(self, q_pos=1e-4, q_vel=5e-3, r_meas=0.005):
        self.q_pos = float(q_pos)       # 过程噪声位置方差 (mm²)
        self.q_vel = float(q_vel)       # 过程噪声速度方差 (mm²/s²)
        self.r_meas = float(r_meas)     # 观测噪声方差 (mm²)
        self.x = np.zeros(4)            # [x, y, vx, vy] (mm, mm/s)
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
        F = np.array([[1, 0, dt, 0],
                      [0, 1, 0, dt],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]], dtype=float)
        Q = np.diag([self.q_pos * dt, self.q_pos * dt,
                     self.q_vel * dt, self.q_vel * dt])
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
            self.beta1, self.beta2, self.beta3 = 3.0 * w, 3.0 * w * w, w ** 3
        else:
            self.beta1, self.beta2, self.beta3 = beta
        self.delta = float(fal_delta)
        self.dist_limit = float(dist_limit)
        self.z1 = 0.0       # 位置估计
        self.z2 = 0.0       # 速度估计
        self.z3 = 0.0       # 扰动力估计（力单位）
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
        self.z2 += dt * (self.b0 * (float(u) + self.z3)
                         - self.beta2 * fal(e1, 0.5, self.delta))
        self.z3 -= dt * self.beta3 * fal(e1, 0.25, self.delta)
        self.z3 = float(np.clip(self.z3, -self.dist_limit, self.dist_limit))
        return self.z3
