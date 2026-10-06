"""
10 Hz 线性 MPC 控制器（输出目标磁力，不直接输出电流）
======================================================
动力学模型（准静态低雷诺数，与 ESO/摩擦模块一致）：
    c·v = F + d        （c = 斯托克斯阻力系数，d = 广义扰动力，代数关系）
    ẋ = v              →  离散（步长 dt = 1/MPC_HZ）：
    x_{k+1} = x_k + dt·(F_k + d)/c

代价（每轴独立，x/y 解耦）：
    J = Σ_{k=1..N} wp·(x_k − x_ref_k)²
      + Σ_{k=0..N−1} wv·((F_k + d)/c − v_ref_k)²
      + Σ_{k=0..N−1} wu·(F_k − F_center_k)²
      + Σ_{k=0..N−1} wΔ·(F_k − F_{k−1})²

    - 第一项：位置轨迹跟踪（参考点由路径弧长 + 设定速度生成）；
    - 第二项：速度跟踪（准静态下 v=(F+d)/c，等价于把 F 拉向 c·v_ref−d，
      天然包含速度前馈与扰动补偿，且与 30Hz 层的阻力补偿不重复——MPC 层
      不再单独加 c·(v_des−v)）；
    - 第三项：absolute 时 F_center=0（legacy）；steady_state 时
      F_center=c*v_ref-d，避免把维持速度的必要力压向零。
      第四项：相对实际已发送电流对应磁力的变化率。

求解：动力学对 F 线性 → 二次代价收缩为小规模线性最小二乘（N×N），
使用活动集精确求解 |F_k| ≤ F_max 的小规模箱约束二次规划。
输出 F_target 序列的第一项（滚动时域）。

单位：位置 mm，速度 mm/s，力 µN，c 单位 µN/(mm/s)。
"""

import numpy as np


class ForceMPC:
    """单轴（x 或 y）线性 MPC；x/y 各建一个实例（模型解耦）。"""

    def __init__(
        self,
        dt,
        horizon=3,
        c_drag=9.42,
        w_pos=1.0,
        w_vel=2.0,
        w_u=0.005,
        fmax=40.0,
        w_delta=0.01,
        effort_mode="absolute",
    ):
        if effort_mode not in ("absolute", "steady_state"):
            raise ValueError(f"Unknown MPC effort mode: {effort_mode}")
        self.effort_mode = effort_mode
        self.dt = float(dt)
        self.N = int(horizon)
        self.c = float(c_drag)
        self.wp = float(w_pos)
        self.wv = float(w_vel)
        self.wu = float(w_u)
        self.wd = float(w_delta)
        self.fmax = float(fmax)
        self.a = self.dt / self.c  # mm/µN：力→每步位移增益

    @staticmethod
    def _bounded_qp(H, g, limit):
        """求 min x'Hx+2g'x, s.t. |x_i|<=limit；N=3 时仅 27 个活动集。"""
        import itertools

        n = len(g)
        best_x, best_obj = None, float("inf")
        tol = 1e-10
        for state_tuple in itertools.product((-1, 0, 1), repeat=n):
            state = np.asarray(state_tuple, dtype=np.int8)
            free = state == 0
            fixed = ~free
            x = np.zeros(n, dtype=float)
            x[state < 0] = -limit
            x[state > 0] = limit
            if np.any(free):
                rhs = -g[free]
                if np.any(fixed):
                    rhs -= H[np.ix_(free, fixed)] @ x[fixed]
                try:
                    x[free] = np.linalg.solve(H[np.ix_(free, free)], rhs)
                except np.linalg.LinAlgError:
                    x[free] = np.linalg.pinv(H[np.ix_(free, free)]) @ rhs
                if np.any(np.abs(x[free]) > limit + tol):
                    continue
            obj = float(x @ H @ x + 2.0 * g @ x)
            if obj < best_obj:
                best_x, best_obj = x.copy(), obj
        return np.zeros(n) if best_x is None else best_x

    @staticmethod
    def _pad_reference(values, n):
        """取未来前 n 项；不足时保持最后一项，而不是发生广播错误。"""
        src = np.asarray(values, dtype=float).ravel()
        out = np.zeros(n, dtype=float)
        if src.size:
            m = min(n, src.size)
            out[:m] = src[:m]
            out[m:] = src[m - 1]
        return out

    def compute(self, x0, d, ref_seq, v_ref_seq=None, f_prev=0.0):
        """单轴 MPC 求解。
        x0      : 当前位置 (mm)
        d       : 扰动力估计 (µN)（如 ESO z3；无则 0）
        ref_seq : (N,) 参考位置序列 [mm]（k=1..N 的参考，或 k=0..N−1，长度补齐为 N）
        v_ref_seq: (N,) 参考速度序列 [mm/s]（可 None → 视为 0）
        返回 dict: F0（本周期输出力 µN，已饱和）、F_seq、x_pred、cost、fmax
        """
        N = self.N
        a = self.a
        d = float(d)
        # 参考序列对齐到长度 N（k = 0..N−1 对应施加 F_k 后的时刻 k+1）
        r = self._pad_reference(ref_seq, N)
        vr = self._pad_reference(v_ref_seq, N) if v_ref_seq is not None else np.zeros(N)

        # 预测：x_{k+1} = x0 + a·Σ_{i≤k}(F_i + d)，k = 0..N−1（x_{k+1} 对应 F_0..F_k）
        # 灵敏度矩阵 L[k, i] = a·(i ≤ k)
        L = a * np.tril(np.ones((N, N)))  # (N,N)
        k_vec = np.arange(N) + 1.0  # x_{k+1} 的扰动行程 = a·(k+1)·d
        x_pred_const = x0 + a * k_vec * d  # F=0 时的预测轨迹（含扰动）

        # 代价二次型（变量 F_0..F_{N−1}）：
        #   J = Σ wp·(x0 + L F + a·k·d − r)² + Σ wv/c²·(F + d − c·vr)²
        #       + Σ wu·(F-F_center)²; L already contains a=dt/c.
        # D·F-b = [F0-F_prev, F1-F0, ...]。
        D = np.eye(N)
        if N > 1:
            D[np.arange(1, N), np.arange(N - 1)] = -1.0
        b_delta = np.zeros(N)
        b_delta[0] = float(f_prev)
        H = (
            self.wp * (L.T @ L)
            + np.eye(N) * (self.wu + self.wv / self.c**2)
            + self.wd * (D.T @ D)
        )
        gv = np.zeros(N)
        for k in range(N):
            # 位置项梯度：wp·L_kᵀ·(x0 + a·k·d − r_k)
            gv += self.wp * L[k, :] * (x_pred_const[k] - r[k])
            # 速度项梯度：wv/c²·(F_k + d − c·vr_k) → 线性项 −wv/c²·(c·vr_k − d)
            gv[k] += -(self.wv / self.c**2) * (self.c * vr[k] - d)
        gv -= self.wd * (D.T @ b_delta)
        # J=F'HF+2g'F+constant: centering changes only g, never H or bounds.
        # Keep the absolute branch's arithmetic/order identical to its parent.
        F_ss = self.c * vr - d
        if self.effort_mode == "steady_state":
            gv -= self.wu * F_ss
        # 直接求箱约束 QP；不能把耦合的无约束序列逐元素 clip。
        F_seq = self._bounded_qp(H, gv, self.fmax)

        x_pred = x_pred_const + L @ F_seq
        v_pred = (F_seq + d) / self.c
        cost = float(
            self.wp * np.sum((x_pred - r) ** 2)
            + self.wv * np.sum((v_pred - vr) ** 2)
            + self.wu
            * np.sum(
                (F_seq - F_ss) ** 2 if self.effort_mode == "steady_state" else F_seq**2
            )
            + self.wd * np.sum((D @ F_seq - b_delta) ** 2)
        )
        return {
            "F0": float(F_seq[0]),
            "F_seq": F_seq.copy(),
            "x_pred": x_pred.copy(),
            "v_pred": v_pred.copy(),
            "delta_F": (D @ F_seq - b_delta).copy(),
            "cost": cost,
            "fmax": self.fmax,
        }
