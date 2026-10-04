"""
1 mm N38 自由对齐磁珠磁驱运动仿真器（物理模型 / 运动积分 / 参数扫描 / 实验拟合）
================================================================================
物理约定（全部 SI，注释标注单位）：
- 磁珠：球形永磁体，磁矩快速趋向当地磁场方向（自由对齐），
      m_b = (Br/μ0)·V，  F = m_b·∇|B|，  ∇|B| = BᵀG/√(BᵀB+B_eps²)
  磁场/梯度/磁力全部来自现有 DipoleSolver（解析梯度，180 偶极子标定模型），
  **不实现第二套磁场模型**。为匹配仿真磁珠参数（直径/Br 可调），
  仿真器构造独立的 DipoleSolver 实例（同一 JSON、同一类、同一接口）。
- 动力学：低雷诺数过阻尼（F_total = 0，忽略惯性/附加质量）：
      F_m = c_v·v + F_friction,   c_v = 6π η r
  库仑摩擦：N = m g（z=0 水平面），F_f,max = μ N；
  静止/滑动按 |F_m| 与 μ N 比较，切换点做数值保护（见 step()）。
- 电流：I_A = command × (2/99)，|I| ≤ 2 A。
- 单位：内部 m / s / N / T / A；对外遥测 mm / mm/s / µN / mT。

GUI 见 bead_sim_gui.py；基础验证 Test 1~8 见 tests/test_bead_sim.py。
"""

import csv
import math
from typing import ClassVar

import numpy as np
from dipole_solver import DEFAULT_MODEL_PATH, DipoleSolver

G_ACC = 9.81  # 重力加速度 (m/s²)
MU0 = 4.0 * math.pi * 1e-7  # 真空磁导率 (T·m/A)
V_EPS = 1e-12  # 速度零阈值 (m/s)
F_EPS = 1e-15  # 力零阈值 (N)
V_MAX = 1.0  # 速度安全上限 (m/s)，正常工作 <0.02 m/s


# ================= 物理模型 =================
class MagnetModel:
    """球形永磁磁珠（单位：mm 输入 / SI 内部）"""

    def __init__(self, diameter_mm=1.0, Br_T=1.20, density_kg_m3=7500.0):
        if not (0.05 <= diameter_mm <= 5.0):
            raise ValueError(f"直径超出合理范围 0.05~5 mm: {diameter_mm}")
        if not (0.1 <= Br_T <= 1.6):
            raise ValueError(f"Br 超出合理范围 0.1~1.6 T: {Br_T}")
        self.diameter_mm = float(diameter_mm)
        self.Br_T = float(Br_T)
        self.density_kg_m3 = float(density_kg_m3)
        self.radius_m = self.diameter_mm * 0.5e-3  # m
        self.volume_m3 = (4.0 / 3.0) * math.pi * self.radius_m**3  # m³
        self.mass_kg = self.density_kg_m3 * self.volume_m3  # kg
        # 球形永磁体等效磁矩 m_b = (Br/μ0)·V (A·m²)
        self.moment_A_m2 = (self.Br_T / MU0) * self.volume_m3

    @property
    def weight_N(self):
        """重力 (N)"""
        return self.mass_kg * G_ACC


class FluidModel:
    """液体（黏度）"""

    def __init__(self, viscosity_mPa_s=1000.0):
        if viscosity_mPa_s < 0:
            raise ValueError("黏度不能为负")
        self.viscosity_mPa_s = float(viscosity_mPa_s)

    @property
    def viscosity_Pa_s(self):
        return self.viscosity_mPa_s * 1e-3  # mPa·s → Pa·s

    def drag_coefficient(self, radius_m):
        """斯托克斯黏性阻力系数 c_v = 6π η r (N·s/m)。
        η→0 时数值下限保护（避免 v=F/c 除零），下限极小不影响物理。"""
        c = 6.0 * math.pi * self.viscosity_Pa_s * radius_m
        return max(c, 1e-12)


class FrictionModel:
    """底面库仑摩擦（z=0 水平面）"""

    def __init__(self, mu=0.10):
        if not (0.0 <= mu <= 2.0):
            raise ValueError(f"摩擦系数超出合理范围 0~2: {mu}")
        self.mu = float(mu)

    def normal_force_N(self, magnet: MagnetModel):
        """法向力 N = m g (N)（水平面，无外加法向场时）"""
        return magnet.weight_N

    def start_force_N(self, magnet: MagnetModel):
        """启动磁力阈值 F_start = μ m g (N)"""
        return self.mu * self.normal_force_N(magnet)


# ================= 运动仿真器 =================
class BeadSimulator:
    """过阻尼磁珠运动仿真器（Mode A 核心：电流 → 场 → 力 → 运动）。

    磁场/梯度/磁力来自 DipoleSolver（解析梯度，基场缓存）。
    每步重新计算 B/G/∇|B|/F_m/阻力/摩擦/速度/位置；全 float64；
    B≈0、v≈0、F≈0、摩擦切换点均有数值保护（见 step()）。"""

    def __init__(
        self,
        magnet=None,
        fluid=None,
        friction=None,
        model_json=DEFAULT_MODEL_PATH,
        pos0_mm=(0.0, 0.0),
    ):
        self.magnet = magnet or MagnetModel()
        self.fluid = fluid or FluidModel()
        self.friction = friction or FrictionModel()
        # 与仿真磁珠参数一致的求解器实例（同一 JSON/同一模型，非第二套磁场）
        self.solver = DipoleSolver.from_json(
            model_json,
            bead_diameter_mm=self.magnet.diameter_mm,
            bead_Br=self.magnet.Br_T,
        )
        self.c_v = self.fluid.drag_coefficient(self.magnet.radius_m)  # N·s/m
        self.N_normal = self.friction.normal_force_N(self.magnet)  # N
        self.F_start = self.friction.start_force_N(self.magnet)  # N
        self.reset(pos0_mm)

    # ---- 状态 ----
    def reset(self, pos_mm=(0.0, 0.0)):
        self.pos = np.array([pos_mm[0] * 1e-3, pos_mm[1] * 1e-3])  # m
        self.vel = np.zeros(2)  # m/s
        self.t = 0.0  # s
        self.history = []  # 遥测行

    def pos_m3(self):
        return np.array([self.pos[0], self.pos[1], 0.0])  # m（z=0 平面）

    # ---- 单步（Mode A） ----
    def step(self, currents_A, dt=1.0 / 300.0):
        """给定六路实际电流 (A)，推进一个仿真时间步 dt (s)。
        返回遥测 dict（量纲：mT / T/m / µN / mm / mm/s）。"""
        I = np.asarray(currents_A, dtype=np.float64)
        fm = self.solver.forward_model(self.pos_m3(), I)
        B = fm["B"]  # (3,) T
        G = fm["G"]  # (3,3) T/m
        F_m = np.atleast_1d(fm["F"]).astype(np.float64)  # (3,) N
        F_m2 = F_m[:2]  # 平面内磁力
        muN = self.friction.mu * self.N_normal  # μ N (N)
        speed = float(np.linalg.norm(self.vel))

        # ---- 库仑摩擦 + 过阻尼速度解（静止/滑动双状态，切换点保护）----
        if speed > V_EPS:
            # 滑动中：动摩擦沿 −v̂
            v_hat = self.vel / speed
            F_fric = -muN * v_hat
            v_new = (F_m2 + F_fric) / self.c_v
            if (
                float(np.dot(v_new, self.vel)) < 0.0
                and np.linalg.norm(F_m2) <= muN + F_EPS
            ):
                # 摩擦足以使运动反转且驱动力低于启动阈值 → 停住（静摩擦保持）
                v_new = np.zeros(2)
                F_fric = -F_m2
                state = "STATIC"
            else:
                state = "SLIDING"
        else:
            # 静止：|F_m| ≤ μN → 静摩擦恰好抵消磁力（F_fric = −F_m），保持静止
            if np.linalg.norm(F_m2) <= muN + F_EPS:
                v_new = np.zeros(2)
                F_fric = -F_m2.copy()
                state = "STATIC"
            else:
                # 启动：动摩擦沿 −F̂_m
                F_fric = -muN * F_m2 / max(float(np.linalg.norm(F_m2)), F_EPS)
                v_new = (F_m2 + F_fric) / self.c_v
                state = "SLIDING"
        # 数值安全：速度上限（正常工作远低于此值）
        sp_new = float(np.linalg.norm(v_new))
        if sp_new > V_MAX:
            v_new = v_new * (V_MAX / sp_new)
        self.vel = v_new
        self.pos = self.pos + v_new * dt
        self.t += dt

        F_drag = -self.c_v * v_new  # N（黏性阻力）
        F_fric_uN = np.array(F_fric[:2]) * 1e6
        x_mm, y_mm = self.pos * 1e3
        row = {
            "t": self.t,
            "x": x_mm,
            "y": y_mm,
            "vx": self.vel[0] * 1e3,
            "vy": self.vel[1] * 1e3,
            "speed": float(np.linalg.norm(self.vel)) * 1e3,
            "Bx_mT": B[0] * 1e3,
            "By_mT": B[1] * 1e3,
            "Bz_mT": B[2] * 1e3,
            "Bmag_mT": float(np.linalg.norm(B)) * 1e3,
            "G": G.copy(),  # T/m (3,3)
            "Fx_uN": F_m[0] * 1e6,
            "Fy_uN": F_m[1] * 1e6,
            "Fz_uN": F_m[2] * 1e6,
            "Fmag_uN": float(np.linalg.norm(F_m)) * 1e6,
            "Fdrag_x_uN": F_drag[0] * 1e6,
            "Fdrag_y_uN": F_drag[1] * 1e6,
            "Ffric_x_uN": F_fric_uN[0],
            "Ffric_y_uN": F_fric_uN[1],
            "I_A": I.copy(),
            "state": state,
        }
        self.history.append(row)
        return row

    # ---- 理论性能（对照用） ----
    def theory(self, F_m_N=None):
        """黏性阻力系数 (µN/(mm/s))、启动磁力 (µN)、给定磁力下理论稳态速度 (mm/s)"""
        c_v_uN = self.c_v * 1e3 / 1e-3  # N·s/m → µN/(mm/s)
        f_start_uN = self.F_start * 1e6
        v_ss = 0.0
        if F_m_N is None:
            F_m_N = self.F_start * 2.0  # 未给定时用 2×阈值示例
        drive = float(np.linalg.norm(F_m_N)) - self.F_start
        if drive > 0:
            v_ss = drive / self.c_v * 1e3  # m/s → mm/s
        return {"c_v_uN_per_mm_s": c_v_uN, "F_start_uN": f_start_uN, "v_ss_mm_s": v_ss}


# ================= 轨迹（Mode B 参考） =================
class TrajectoryModel:
    """世界系 mm 轨迹；按弧长-时间生成参考点与参考速度（切向）"""

    def __init__(self, points_mm, closed=False):
        self.pts = np.asarray(points_mm, dtype=np.float64)
        if self.pts.ndim != 2 or self.pts.shape[0] < 2:
            raise ValueError("轨迹至少需要 2 个点")
        self.closed = bool(closed)
        seg = np.linalg.norm(np.diff(self.pts, axis=0), axis=1)
        self.s = np.concatenate([[0.0], np.cumsum(seg)])
        self.total = float(self.s[-1])

    @classmethod
    def circle(cls, cx=0.0, cy=0.0, r_mm=3.0, n=180):
        pts = [
            (
                cx + r_mm * math.cos(2 * math.pi * i / n),
                cy + r_mm * math.sin(2 * math.pi * i / n),
            )
            for i in range(n)
        ]
        pts.append(pts[0])
        return cls(pts, closed=True)

    @classmethod
    def rectangle(cls, cx=0.0, cy=0.0, w_mm=6.0, h_mm=4.0):
        x0, y0, x1, y1 = cx - w_mm / 2, cy - h_mm / 2, cx + w_mm / 2, cy + h_mm / 2
        pts = [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)]
        return cls(pts, closed=True)

    @classmethod
    def triangle(cls, cx=0.0, cy=0.0, side_mm=5.0):
        R = side_mm / math.sqrt(3)
        v = [
            (
                cx + R * math.cos(math.pi / 2 + 2 * math.pi * k / 3),
                cy + R * math.sin(math.pi / 2 + 2 * math.pi * k / 3),
            )
            for k in range(3)
        ]
        v.append(v[0])
        return cls(v, closed=True)

    @classmethod
    def from_points(cls, pts_mm):
        return cls(pts_mm, closed=False)

    def point_at(self, arc_mm):
        """弧长位置 → 点 (mm)；闭合轨迹取模，开放轨迹钳到末端"""
        s = arc_mm
        if self.closed:
            s = s % self.total
        else:
            s = min(max(s, 0.0), self.total)
        i = int(np.searchsorted(self.s, s, side="left"))
        i = min(max(i, 1), len(self.s) - 1)
        t = (s - self.s[i - 1]) / max(self.s[i] - self.s[i - 1], 1e-12)
        p = self.pts[i - 1] + t * (self.pts[i] - self.pts[i - 1])
        d = self.pts[i] - self.pts[i - 1]
        n = np.linalg.norm(d)
        return p, (d / n if n > 1e-12 else np.zeros(2))


# ================= 控制器（Mode B） =================
class PositionController:
    """位置控制器：X/Y 独立 PID + 速度前馈（阻力补偿）→ 目标磁力 (N)。
    输出经 F_lim (N) 限幅。逆解由 solve_commands 完成（两层结构）。"""

    def __init__(
        self,
        c_v_N,
        kp=(20.0, 20.0),
        ki=(2.0, 2.0),
        kd=(5.0, 5.0),
        f_lim_N=100e-6,
        int_lim=100.0,
    ):
        self.kp = np.array(kp, float)  # µN/mm
        self.ki = np.array(ki, float)  # µN/(mm·s)
        self.kd = np.array(kd, float)  # µN/(mm/s)
        self.f_lim_uN = f_lim_N * 1e6
        self.int_lim = float(int_lim)
        self.c_v_uN = c_v_N * 1e6 * 1e3  # µN/(mm/s)
        self.integral = np.zeros(2)
        self.vel_est = np.zeros(2)  # mm/s

    def reset(self):
        self.integral = np.zeros(2)
        self.vel_est = np.zeros(2)

    def compute(self, pos_mm, vel_mm, ref_mm, v_ref_mm=0.0, dt=1 / 30):
        """返回 F_des (N,3)，z 分量为 0。"""
        e = np.asarray(ref_mm, float) - np.asarray(pos_mm, float)
        # 速度 EMA（与主程序一致的微分滤波）
        v_meas = np.asarray(vel_mm, float)
        self.vel_est = 0.6 * v_meas + 0.4 * self.vel_est
        self.integral = np.clip(self.integral + e * dt, -self.int_lim, self.int_lim)
        vr = np.broadcast_to(np.asarray(v_ref_mm, float), (2,))
        F_uN = np.array(
            [
                self.kp[0] * e[0]
                + self.ki[0] * self.integral[0]
                + self.kd[0] * (vr[0] - self.vel_est[0]),
                self.kp[1] * e[1]
                + self.ki[1] * self.integral[1]
                + self.kd[1] * (vr[1] - self.vel_est[1]),
            ]
        )
        # 速度前馈/阻力补偿：F += c_v·v_ref（与主程序合并式一致，不重复补偿）
        F_uN = F_uN + self.c_v_uN * vr
        fn = float(np.linalg.norm(F_uN))
        if fn > self.f_lim_uN:
            F_uN *= self.f_lim_uN / fn
        return np.array([F_uN[0], F_uN[1], 0.0]) * 1e-6, e


# ================= 闭环仿真（Mode B） =================
class ClosedLoopRunner:
    """轨迹 → 控制器 → solve_commands → 电流 → BeadSimulator 闭环。
    视觉反馈 = 仿真真实位置 + 高斯测量噪声（可调 σ，mm）。"""

    def __init__(
        self,
        sim: BeadSimulator,
        traj: TrajectoryModel,
        controller: PositionController,
        speed_mm_s=1.0,
        vision_noise_mm=0.0,
        seed=12345,
    ):
        self.sim = sim
        self.traj = traj
        self.ctrl = controller
        self.speed = float(speed_mm_s)
        self.noise = float(vision_noise_mm)
        self.rng = np.random.default_rng(seed)
        self.arc = 0.0
        self.cmd_prev = np.zeros(6, dtype=int)
        self.metrics = {
            "max_speed": 0.0,
            "max_I": 0.0,
            "sum_I": 0.0,
            "n_I": 0,
            "err_sq_sum": 0.0,
            "err_n": 0,
            "done_t": None,
        }
        self.t = 0.0

    def reset(self):
        self.arc = 0.0
        self.cmd_prev = np.zeros(6, dtype=int)
        self.ctrl.reset()
        self.metrics = {
            "max_speed": 0.0,
            "max_I": 0.0,
            "sum_I": 0.0,
            "n_I": 0,
            "err_sq_sum": 0.0,
            "err_n": 0,
            "done_t": None,
        }
        self.t = 0.0

    def step_ctrl(self, dt_ctrl=1.0 / 30.0):
        """控制器周期：参考点 → F_des → 逆解 → I_target（30Hz）。"""
        self.arc += self.speed * dt_ctrl
        p_ref, d_hat = self.traj.point_at(self.arc)
        pos_meas = self.sim.pos * 1e3
        if self.noise > 0:
            pos_meas = pos_meas + self.rng.normal(0, self.noise, 2)
        F_des, _e = self.ctrl.compute(
            pos_meas, self.sim.vel * 1e3, p_ref, self.speed * d_hat, dt_ctrl
        )
        rec = self.sim.solver.solve_commands(
            np.array([self.sim.pos[0], self.sim.pos[1], 0.0]),
            F_des,
            cmd_prev=self.cmd_prev,
        )
        self.cmd_prev = np.asarray(rec["commands"], dtype=int)
        err = float(np.linalg.norm(p_ref - self.sim.pos * 1e3))
        m = self.metrics
        m["err_sq_sum"] += err**2
        m["err_n"] += 1
        return rec, p_ref, err

    def step_motion(self, dt_sim=1.0 / 300.0):
        """运动子步：以当前 I_target 积分（每控制周期调用 10 次）。"""
        I_A = self.cmd_prev * self.sim.solver.current_gain
        row = self.sim.step(I_A, dt_sim)
        m = self.metrics
        m["max_speed"] = max(m["max_speed"], row["speed"])
        m["max_I"] = max(m["max_I"], float(np.max(np.abs(row["I_A"]))))
        m["sum_I"] += float(np.abs(row["I_A"]).sum())
        m["n_I"] += 1
        self.t += dt_sim
        return row


# ================= 参数扫描 =================
class ParameterSweep:
    """OFAT 参数扫描：每参数沿默认值列表扫一遍，闭环跑固定航点任务。
    输出指标：启动磁力 / 理论稳态速度 / 到达时间 / 最大速度 / 最大电流 /
    平均电流 / 轨迹误差（RMS）。"""

    DIAMETERS: ClassVar[list[float]] = [0.5, 0.75, 1.0, 1.25, 1.5, 2.0]
    BRS: ClassVar[list[float]] = [0.8, 1.0, 1.2, 1.3]
    VISCOSITIES: ClassVar[list[float]] = [100.0, 500.0, 1000.0, 2000.0, 5000.0]
    MUS: ClassVar[list[float]] = [0.0, 0.05, 0.10, 0.20, 0.30]
    TARGET_MM = (3.0, 0.0)
    TOL_MM = 0.2
    T_MAX = 12.0

    @classmethod
    def run_point(
        cls,
        diameter_mm,
        Br_T,
        visc_mPas,
        mu,
        kp=(20.0, 20.0),
        ki=(2.0, 2.0),
        kd=(5.0, 5.0),
        f_lim_N=100e-6,
        speed=1.0,
    ):
        """单配置闭环航点任务，返回指标 dict"""
        mag = MagnetModel(diameter_mm, Br_T)
        fluid = FluidModel(visc_mPas)
        fric = FrictionModel(mu)
        sim = BeadSimulator(mag, fluid, fric)
        ctrl = PositionController(sim.c_v, kp, ki, kd, f_lim_N)
        runner = ClosedLoopRunner(
            sim, TrajectoryModel([(0.0, 0.0), cls.TARGET_MM]), ctrl, speed
        )
        dt_c, dt_s = 1.0 / 30.0, 1.0 / 300.0
        while runner.t < cls.T_MAX:
            _rec, _p_ref, _err = runner.step_ctrl(dt_c)
            for _ in range(10):
                runner.step_motion(dt_s)
            if (
                math.hypot(
                    cls.TARGET_MM[0] - sim.pos[0] * 1e3,
                    cls.TARGET_MM[1] - sim.pos[1] * 1e3,
                )
                < cls.TOL_MM
                and runner.metrics["done_t"] is None
            ):
                runner.metrics["done_t"] = runner.t
        th = sim.theory(np.array([30e-6, 0.0, 0.0]))  # 30µN 参考力
        m = runner.metrics
        return {
            "diameter_mm": diameter_mm,
            "Br_T": Br_T,
            "viscosity_mPas": visc_mPas,
            "mu": mu,
            "F_start_uN": fric.start_force_N(mag) * 1e6,
            "c_v_uN_per_mm_s": sim.c_v * 1e6 * 1e3,
            "v_ss_30uN_mm_s": th["v_ss_mm_s"],
            "time_to_target_s": m["done_t"] if m["done_t"] is not None else -1.0,
            "max_speed_mm_s": m["max_speed"],
            "max_I_A": m["max_I"],
            "mean_I_A": m["sum_I"] / max(m["n_I"] * 6, 1),
            "track_rms_mm": math.sqrt(m["err_sq_sum"] / max(m["err_n"], 1)),
        }

    @classmethod
    def run_ofat(cls, base=None):
        """逐参数扫描（其余参数取默认），返回行列表"""
        base = base or {
            "diameter_mm": 1.0,
            "Br_T": 1.2,
            "viscosity_mPas": 1000.0,
            "mu": 0.10,
        }
        rows = []
        for d in cls.DIAMETERS:
            rows.append(
                cls.run_point(d, base["Br_T"], base["viscosity_mPas"], base["mu"])
            )
        for br in cls.BRS:
            rows.append(
                cls.run_point(
                    base["diameter_mm"], br, base["viscosity_mPas"], base["mu"]
                )
            )
        for eta in cls.VISCOSITIES:
            rows.append(
                cls.run_point(base["diameter_mm"], base["Br_T"], eta, base["mu"])
            )
        for mu in cls.MUS:
            rows.append(
                cls.run_point(
                    base["diameter_mm"], base["Br_T"], base["viscosity_mPas"], mu
                )
            )
        return rows

    @staticmethod
    def save_csv(rows, path):
        if not rows:
            return
        keys = list(rows[0].keys())
        with open(path, "w", encoding="utf-8", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(rows)

    @classmethod
    def plot_results(cls, rows, path_png):
        """参数-性能关系图（4 子图）"""
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(2, 2, figsize=(11, 7))
        groups = [
            ("diameter_mm", "Diameter (mm)"),
            ("Br_T", "Br (T)"),
            ("viscosity_mPas", "Viscosity (mPa*s)"),
            ("mu", "Friction mu"),
        ]
        for ax, (key, label) in zip(axes.ravel(), groups):
            _sub = [r for r in rows if abs(r[key] - r[key]) < 1]  # 全部
            xs = [r[key] for r in rows]
            order = np.argsort(xs)
            xs = np.array(xs)[order]
            ax.plot(
                xs,
                [rows[i]["v_ss_30uN_mm_s"] for i in order],
                "o-",
                label="v_ss@30µN (mm/s)",
            )
            ax.plot(
                xs, [rows[i]["F_start_uN"] for i in order], "s--", label="F_start (µN)"
            )
            ax.set_xlabel(label)
            ax.set_ylabel("µN / mm·s⁻¹")
            ax.grid(True, alpha=0.3)
            ax.legend(fontsize=8)
        fig.suptitle("Parameter sweep: F_start / v_ss@30uN")
        fig.tight_layout()
        fig.savefig(path_png, dpi=120)
        plt.close(fig)


# ================= 实验参数反求 =================
class ExperimentFitter:
    """轨迹反推等效黏度 η_eff 与等效摩擦系数 μ_eff：
        min_{η,μ} Σ ‖p_sim(t_k) − p_exp(t_k)‖²
    网格搜索（η 对数格 + μ 线性格）+ 一次局部细化。"""

    def __init__(
        self,
        diameter_mm=1.0,
        Br_T=1.20,
        density=7500.0,
        model_json=DEFAULT_MODEL_PATH,
        dt=1.0 / 300.0,
    ):
        self.magnet = MagnetModel(diameter_mm, Br_T, density)
        self.model_json = model_json
        self.dt = float(dt)

    def _simulate(self, eta_mPas, mu, t_exp, I_of_t):
        """按实验时间轴仿真，返回仿真位置采样 (N,2) mm"""
        sim = BeadSimulator(
            self.magnet, FluidModel(eta_mPas), FrictionModel(mu), self.model_json
        )
        t_end = float(t_exp[-1])
        n = max(2, int(t_end / self.dt))
        out_t = np.linspace(0.0, t_end, n)
        pos = np.zeros((n, 2))
        k_exp = 0
        for i, t in enumerate(out_t):
            while k_exp < len(t_exp) - 2 and t_exp[k_exp + 1] <= t:
                k_exp += 1
            I = I_of_t(t)
            sim.step(I, self.dt)
            pos[i] = sim.pos * 1e3
        return out_t, pos

    @staticmethod
    def _rms(pos_sim, t_exp, xy_exp):
        """仿真位置在实验时刻插值后的 RMS 误差 (mm)"""
        xs = np.interp(t_exp, [0, t_exp[-1] + 1e-9], [pos_sim[0, 0], pos_sim[-1, 0]])
        # 通用：仿真输出本身均匀，直接按时间线性插值
        n = len(pos_sim)
        t_sim = np.linspace(0.0, t_exp[-1], n)
        xs = np.interp(t_exp, t_sim, pos_sim[:, 0])
        ys = np.interp(t_exp, t_sim, pos_sim[:, 1])
        d = np.hypot(xs - xy_exp[:, 0], ys - xy_exp[:, 1])
        return float(np.sqrt(np.mean(d**2))), xs, ys

    def fit(self, t_exp, xy_exp, I_of_t, eta_grid=None, mu_grid=None, verbose=False):
        """网格搜索 + 局部细化。返回 dict(eta_eff_mPas, mu_eff, rms_mm,
        xs_fit, ys_fit, history)。"""
        t_exp = np.asarray(t_exp, float)
        xy_exp = np.asarray(xy_exp, float)
        eta_grid = eta_grid or np.geomspace(10.0, 10000.0, 20)
        mu_grid = mu_grid or np.linspace(0.0, 0.5, 21)
        history = []
        best = None
        for eta in eta_grid:
            for mu in mu_grid:
                _, pos = self._simulate(eta, mu, t_exp, I_of_t)
                rms, _, _ = self._rms(pos, t_exp, xy_exp)
                history.append((float(eta), float(mu), rms))
                if best is None or rms < best[2]:
                    best = (float(eta), float(mu), rms)
        # 局部细化：±20% 对数/线性细格
        eta0, mu0, _ = best
        eta_fine = np.geomspace(max(eta0 * 0.8, 1.0), eta0 * 1.25, 9)
        mu_fine = np.linspace(max(mu0 - 0.05, 0.0), mu0 + 0.05, 11)
        for eta in eta_fine:
            for mu in mu_fine:
                _, pos = self._simulate(eta, mu, t_exp, I_of_t)
                rms, _, _ = self._rms(pos, t_exp, xy_exp)
                history.append((float(eta), float(mu), rms))
                if rms < best[2]:
                    best = (float(eta), float(mu), rms)
        eta_e, mu_e, rms_e = best
        _, pos = self._simulate(eta_e, mu_e, t_exp, I_of_t)
        _, xs, ys = self._rms(pos, t_exp, xy_exp)
        result = {
            "eta_eff_mPas": eta_e,
            "mu_eff": mu_e,
            "rms_mm": rms_e,
            "xs_fit": xs,
            "ys_fit": ys,
            "history": history,
        }
        if verbose:
            print(
                f"拟合: η_eff={eta_e:.1f} mPa·s, μ_eff={mu_e:.3f}, "
                f"RMS={rms_e:.3f} mm"
            )
        return result

    @staticmethod
    def load_experiment_csv(path):
        """实验 CSV：须含列 t,x,y（可选 I0..I5）。
        返回 (t, xy(N,2), I_of_t 或 None)。"""
        with open(path, "r", encoding="utf-8") as f:
            reader = csv.reader(f)
            header = next(reader)
            rows = [r for r in reader if r]
        idx = {name.strip().lower(): k for k, name in enumerate(header)}
        for need in ("t", "x", "y"):
            if need not in idx:
                raise ValueError(f"实验 CSV 缺少列 {need}")
        t = np.array([float(r[idx["t"]]) for r in rows])
        xy = np.array([[float(r[idx["x"]]), float(r[idx["y"]])] for r in rows])
        i_cols = [idx[f"i{k}"] for k in range(6) if f"i{k}" in idx]
        I_of_t = None
        if len(i_cols) == 6:
            I_mat = np.array([[float(r[c]) for c in i_cols] for r in rows])

            def I_of_t(tq, _t=t, _I=I_mat):
                return [float(np.interp(tq, _t, _I[:, j])) for j in range(6)]

        return t, xy, I_of_t

    @staticmethod
    def save_comparison_plot(t_exp, xy_exp, xs_fit, ys_fit, path_png):
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(6, 6))
        ax.plot(xy_exp[:, 0], xy_exp[:, 1], "r.-", ms=3, lw=1, label="实验")
        ax.plot(xs_fit, ys_fit, "b-", lw=1.5, label="仿真(拟合)")
        ax.set_xlabel("x (mm)")
        ax.set_ylabel("y (mm)")
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)
        ax.legend()
        fig.tight_layout()
        fig.savefig(path_png, dpi=120)
        plt.close(fig)


# ================= CSV 导出 =================
CSV_HEADER = [
    "time",
    "x",
    "y",
    "vx",
    "vy",
    "speed",
    "Bx_mT",
    "By_mT",
    "Bz_mT",
    "Bmag_mT",
    "Gxx",
    "Gxy",
    "Gxz",
    "Gyx",
    "Gyy",
    "Gyz",
    "Gzx",
    "Gzy",
    "Gzz",
    "Fx_uN",
    "Fy_uN",
    "Fz_uN",
    "Fmag_uN",
    "Fdrag_x_uN",
    "Fdrag_y_uN",
    "Ffric_x_uN",
    "Ffric_y_uN",
    "I0_A",
    "I1_A",
    "I2_A",
    "I3_A",
    "I4_A",
    "I5_A",
    "state",
]


def save_history_csv(history, path):
    with open(path, "w", encoding="utf-8", newline="") as f:
        f.write(",".join(CSV_HEADER) + "\n")
        for r in history:
            g = r["G"].ravel()
            f.write(
                ",".join(
                    str(x)
                    for x in [
                        round(r["t"], 6),
                        round(r["x"], 6),
                        round(r["y"], 6),
                        round(r["vx"], 6),
                        round(r["vy"], 6),
                        round(r["speed"], 6),
                        round(r["Bx_mT"], 6),
                        round(r["By_mT"], 6),
                        round(r["Bz_mT"], 6),
                        round(r["Bmag_mT"], 6),
                        *[round(v, 8) for v in g],
                        round(r["Fx_uN"], 5),
                        round(r["Fy_uN"], 5),
                        round(r["Fz_uN"], 5),
                        round(r["Fmag_uN"], 5),
                        round(r["Fdrag_x_uN"], 5),
                        round(r["Fdrag_y_uN"], 5),
                        round(r["Ffric_x_uN"], 5),
                        round(r["Ffric_y_uN"], 5),
                        *[round(a, 5) for a in r["I_A"]],
                        r["state"],
                    ]
                )
                + "\n"
            )
