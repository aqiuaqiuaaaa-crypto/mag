"""
180 磁偶极子磁场/磁力解算与电流逆解模块（纯 NumPy，可独立自测）
================================================================
物理模型
--------
- 6 路电磁铁，线圈顺序与串口通道固定对应：a0~a5 = +X, +Y, +Z, −X, −Y, −Z。
- 每路 30 个磁偶极子，共 180 个。偶极子位置（mm）与磁矩（mT·mm³，1A 电流响应
  矢量）由标定模型 JSON 加载，加载时严格校验（见 validate_model_data）。
- 单位换算（内部统一 SI, float64）：位置 mm→m（×1e-3）；磁矩 mT·mm³→A·m²（×1e-5）。
  标定拟合约定（与 COMSOL 原始场数据一致，经 magnetic_field_validation /
  MPC统一框架 / 运动demo 三处独立程序交叉验证）：B[mT] = Σ[3(K·R)R/|R|⁵ − K/R³]，
  R 用 mm、不含 μ0/(4π)，由此 K = m_SI×1e5，即 m_SI = mom×1e-5。
  （若误按 mom=μ0·m 约定 ×1e-12/μ0，B 与 F 全部偏小 4π 倍——已修正的 bug。）
  实际线圈电流 I_A = 指令 × (2/99)，即 ±99 指令 ↔ ±2A。
- 单偶极子磁场（R = 场点 − 偶极子位置）：
      B_i = μ0/(4π) · [ 3 R (m_i·R)/|R|⁵ − m_i/|R|³ ]
  总场 B = Σ B_i（NumPy 向量化，180 偶极子按线圈分块预求和，不逐个建对象）。
- 磁力：磁珠为硬磁球形永磁体（1mm N38），磁矩在液体中自由转动、沿当地磁场排列：
      F = m_b ∇|B|,   m_b = (Br/μ0)·(4/3)πr³
  ∇|B| 由**解析磁场梯度张量** G = ∂B/∂x 计算：∇|B| = (BᵀG)/max(|B|, B_eps)。
  |B|→0 处 ∇|B| 病态，用 B_eps 下限保护避免 NaN（另保留数值差分版本仅供 DEBUG
  验证，见 grad_absB_numeric）。

电流逆解（L1 + Tikhonov L2 正则化，带箱约束的高斯-牛顿）
--------------------------------------------------------
    min_I  0.5‖F(I) − F_des‖² + λ1·Σ|I_i| + 0.5·λ2·Σ I_i²
    s.t.   −2 ≤ I_i ≤ 2
           I_prev_i − Δ ≤ I_i ≤ I_prev_i + Δ      (Δ = 每帧变化率上限, 0.1818A)

- 注意：这不是单纯的"Tikhonov 正则化"——λ1 为 L1 项（最小电流绝对值之和），
  λ2 为 Tikhonov L2 项，两个独立参数。
- 求解：IRLS 把 L1 项二次近似（w_i = 1/(|I_i|+ε)），每步解正则化高斯-牛顿
  最小二乘并用箱约束投影 + 回溯线搜索；候选解始终位于箱约束内（先约束后搜索，
  不做"解完再截断"）。
- F = m·∇|B| 是电流的偶函数（J(I=0)≡0），必须热启动（上一帧电流）+ 多冷启动。
- 收敛判据统一：force_relative_error = ‖F_act−F_des‖/max(‖F_des‖,eps) ≤ 2%。
- F_des 超出约束箱所能达到的范围时 converged=False 且
  current_constraint_active=True（磁力不可达/受电流约束）。
- solve_commands() 返回**最终真正发送电流**对应的 F_actual（发送电流为整数指令，
  由 I* 四舍五入得到，F_actual 按发送电流重新调用力模型计算）。

坐标约定：世界系（与模型 JSON 一致），磁珠工作平面 z=0，内部单位米。
"""

import itertools
import json
import math
import os
import time

import numpy as np

MU0_OVER_4PI = 1e-7  # μ0/4π  (T·m/A)
MU0 = 4.0 * math.pi * 1e-7

# 电流/指令映射（与 config.py 保持一致；模块独立可用）
MAX_CURRENT_A = 2.0
CMD_MAX = 99
CMD_TO_A = MAX_CURRENT_A / CMD_MAX
DIRECTED_CUBE_OFFSETS = np.asarray(
    list(itertools.product((-1, 0, 1), repeat=6)), dtype=int
)
MAX_DELTA_CMD = 9  # 9 指令 = 0.1818A，严格 ≤ 0.2A/帧
MAX_DELTA_A = MAX_DELTA_CMD * CMD_TO_A

CONV_TOL = 0.02  # 收敛判据：力相对误差 ≤ 2%
LINEAR_COND_MAX = 1e10  # 线性解条件数上限（超过则回退约束 GN）
B_EPS = 1e-15  # |B| 下限保护
SOLVER_N_ITER = 8
SOLVER_N_STARTS = 3
LAMBDA1_TAU = 1.0
LAMBDA2_TAU = 1.0

DEFAULT_MODEL_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "models",
    "N30LM_six_coils_180dipoles.json",
)
COIL_ORDER = ["+X", "+Y", "+Z", "-X", "-Y", "-Z"]
DIPOLES_PER_COIL = 30
N_COILS = 6
TOTAL_DIPOLES = N_COILS * DIPOLES_PER_COIL


class ModelValidationError(ValueError):
    """标定模型不符合 6×30=180 规格时抛出"""


def validate_model_data(data):
    """校验原始模型 JSON 字典。任何不符合项抛 ModelValidationError。
    校验项：n_coils=6 / dipoles_per_coil=30 / 总数=180 / coil_order 顺序 /
    六个线圈齐全 / pos、mom 维度 (30,3)。"""
    if not isinstance(data, dict):
        raise ModelValidationError("模型根节点必须是 JSON 对象")
    if int(data.get("n_coils", -1)) != N_COILS:
        raise ModelValidationError(
            f"n_coils 必须为 {N_COILS}，实际 {data.get('n_coils')}"
        )
    if int(data.get("dipoles_per_coil", -1)) != DIPOLES_PER_COIL:
        raise ModelValidationError(
            f"dipoles_per_coil 必须为 {DIPOLES_PER_COIL}，实际 {data.get('dipoles_per_coil')}"
        )
    if "n_dipoles_total" not in data:
        raise ModelValidationError("缺少 n_dipoles_total")
    if int(data["n_dipoles_total"]) != TOTAL_DIPOLES:
        raise ModelValidationError(
            f"n_dipoles_total 必须为 {TOTAL_DIPOLES}，实际 {data['n_dipoles_total']}"
        )
    order = list(data.get("coil_order", []))
    if order != COIL_ORDER:
        raise ModelValidationError(
            f"coil_order 必须为 {COIL_ORDER}（a0~a5 = +X,+Y,+Z,−X,−Y,−Z），实际 {order}"
        )
    coils = data.get("coils", {})
    for name in COIL_ORDER:
        if name not in coils:
            raise ModelValidationError(f"缺少线圈 {name}")
        c = coils[name]
        pos = np.asarray(c.get("pos"), dtype=float)
        mom = np.asarray(c.get("mom"), dtype=float)
        if pos.shape != (DIPOLES_PER_COIL, 3):
            raise ModelValidationError(
                f"线圈 {name} pos 维度必须为 ({DIPOLES_PER_COIL},3)，实际 {pos.shape}"
            )
        if mom.shape != (DIPOLES_PER_COIL, 3):
            raise ModelValidationError(
                f"线圈 {name} mom 维度必须为 ({DIPOLES_PER_COIL},3)，实际 {mom.shape}"
            )
        if not (np.all(np.isfinite(pos)) and np.all(np.isfinite(mom))):
            raise ModelValidationError(f"线圈 {name} 存在非有限数值")


class DipoleSolver:
    """6 路电磁铁 → 180 磁偶极子的磁场 / 磁力解算与约束电流逆解"""

    def __init__(
        self,
        seg_pos,
        seg_mom_unit,
        seg_coil_idx,
        n_coils=N_COILS,
        coil_names=None,
        model_info=None,
        bead_diameter_mm=1.0,
        bead_Br=1.20,
        current_gain=CMD_TO_A,
        b_eps=B_EPS,
    ):
        seg_pos = np.asarray(seg_pos, dtype=np.float64)
        seg_mom = np.asarray(seg_mom_unit, dtype=np.float64)
        seg_idx = np.asarray(seg_coil_idx, dtype=int)
        if seg_pos.ndim != 2 or seg_pos.shape[1] != 3:
            raise ModelValidationError(
                f"seg_pos 维度必须为 (N,3)，实际 {seg_pos.shape}"
            )
        if seg_mom.shape != seg_pos.shape:
            raise ModelValidationError(
                f"seg_mom 维度必须与 seg_pos 一致，实际 {seg_mom.shape}"
            )
        if seg_idx.shape != (seg_pos.shape[0],):
            raise ModelValidationError("seg_coil_idx 维度必须为 (N,)")
        if seg_idx.min() < 0 or seg_idx.max() >= n_coils:
            raise ModelValidationError("seg_coil_idx 必须在 0~5 范围内")
        # 每个线圈必须恰好 DIPOLES_PER_COIL 个偶极子（防止 JSON/数组异常静默错位）
        counts = np.bincount(seg_idx, minlength=n_coils)
        if not np.all(counts == DIPOLES_PER_COIL):
            raise ModelValidationError(
                f"每线圈偶极子数必须为 {DIPOLES_PER_COIL}，实际 {counts.tolist()}"
            )
        if seg_pos.shape[0] != TOTAL_DIPOLES:
            raise ModelValidationError(
                f"偶极子总数必须为 {TOTAL_DIPOLES}（6×30），实际 {seg_pos.shape[0]}"
            )
        # 按线圈重排为 (6,30,3)，便于向量化按线圈预求和
        order = np.lexsort((np.arange(seg_idx.size), seg_idx))
        self.seg_pos = seg_pos[order].reshape(n_coils, DIPOLES_PER_COIL, 3)
        self.seg_mom = seg_mom[order].reshape(n_coils, DIPOLES_PER_COIL, 3)
        self.seg_coil_idx = seg_idx[order]
        self.n_coils = int(n_coils)
        self.coil_names = list(coil_names) if coil_names else list(COIL_ORDER)
        if self.coil_names != COIL_ORDER:
            raise ModelValidationError(
                f"coil_names 必须为 {COIL_ORDER}，实际 {self.coil_names}"
            )
        self.model_info = dict(model_info or {})
        self.current_gain = float(current_gain)  # I_A = 指令 × gain
        self.bead_radius = bead_diameter_mm * 0.5e-3
        self.bead_moment = (bead_Br / MU0) * (4.0 / 3.0) * math.pi * self.bead_radius**3
        self.b_eps = float(b_eps)
        self._rng = np.random.default_rng(20260927)  # 实例级 rng：可设种子，逐帧推进
        self._seg_pos_flat = self.seg_pos.reshape(-1, 3)  # (180,3)
        self._seg_mom_flat = self.seg_mom.reshape(-1, 3)  # (180,3)
        # 基场缓存：B、G 对电流线性，同一位姿只需一次偶极子遍历，
        # 之后任意电流的力 = 纯矩阵运算（雅可比/线搜索/目标函数加速 ~100×）
        self._cache_pos = None
        self._cache_Bc = None
        self._cache_Gc = None

    # ---------------- 模型构建与校验 ----------------
    @classmethod
    def from_json(
        cls,
        path=DEFAULT_MODEL_PATH,
        bead_diameter_mm=1.0,
        bead_Br=1.20,
        current_gain=CMD_TO_A,
    ):
        """加载标定模型 JSON 并严格校验（6×30=180、coil_order、维度）。
        校验失败抛 ModelValidationError——不允许静默退化到错误模型。"""
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        validate_model_data(data)
        pos_all, mom_all, idx_all = [], [], []
        for j, name in enumerate(COIL_ORDER):
            c = data["coils"][name]
            pos_all.append(np.asarray(c["pos"], dtype=float) * 1e-3)  # mm → m
            # mT·mm³ → A·m²：标定拟合约定为 B[mT] = Σ[3(K·R)R/|R|⁵ − K/R³]
            # （R 用 mm，不含 μ0/(4π)）。与 SI 偶极子公式逐项对齐可推出
            #   K[mT·mm³] = m_SI[A·m²] × 1e5，即 m_SI = mom × 1e-5。
            # 推导：B_SI = μ0/4π·1e9·Σ[...]（R 换 mm 的尺度因子），
            #       B_mT = 1e3·B_SI = Σ[...] ⇒ 系数 1e3·(μ0/4π)·1e9 = 1e5。
            # 若按 mom=μ0·m 约定用 ×1e-12/μ0，会偏小 4π 倍（2026-09-27 已修正）。
            # 【COMSOL 实测确认】单线圈 1A 在原点 |B|=11.6mT，模型 11.563mT（差 0.3%），
            # 见 models/comsol_reference.json 与 validate_against_comsol_reference()。
            # 绝对标定请用 validate_against_comsol_reference()。
            mom_all.append(np.asarray(c["mom"], dtype=float) * 1e-5)
            idx_all.append(np.full(DIPOLES_PER_COIL, j, dtype=int))
        info = {
            "model_name": data.get("model_name", os.path.basename(path)),
            "path": path,
            "n_coils": N_COILS,
            "dipoles_per_coil": DIPOLES_PER_COIL,
            "coil_order": list(COIL_ORDER),
        }
        solver = cls(
            np.vstack(pos_all),
            np.vstack(mom_all),
            np.concatenate(idx_all),
            n_coils=N_COILS,
            coil_names=list(COIL_ORDER),
            model_info=info,
            bead_diameter_mm=bead_diameter_mm,
            bead_Br=bead_Br,
            current_gain=current_gain,
        )
        solver.validate_model()
        return solver

    @classmethod
    def synthetic_ring(
        cls,
        ring_radius_mm=60.0,
        coil_height_mm=30.0,
        coil_length_mm=30.0,
        base_moment_1a=0.10,
        bead_diameter_mm=1.0,
        bead_Br=1.20,
        current_gain=CMD_TO_A,
    ):
        """DEBUG/SIMULATION 专用理想圆环模型（6×30=180）。
        禁止作为正式实验 fallback。每线圈偶极子数固定为 DIPOLES_PER_COIL=30
        （与构造函数校验一致，不提供可变参数）。"""
        n_seg_per_coil = DIPOLES_PER_COIL
        R, zc, L = ring_radius_mm * 1e-3, coil_height_mm * 1e-3, coil_length_mm * 1e-3
        mpa = base_moment_1a / n_seg_per_coil
        pos, mom, idx = [], [], []
        for j in range(N_COILS):
            th = 2.0 * math.pi * j / N_COILS
            c = np.array([R * math.cos(th), R * math.sin(th), -zc])
            u = np.array([-R * math.cos(th), -R * math.sin(th), zc])
            u = u / np.linalg.norm(u)
            for i in range(n_seg_per_coil):
                s = -L / 2.0 + L * (i + 0.5) / n_seg_per_coil
                pos.append(c + s * u)
                mom.append(mpa * u)
                idx.append(j)
        info = {
            "model_name": "synthetic_ring(DEBUG)",
            "n_coils": N_COILS,
            "dipoles_per_coil": n_seg_per_coil,
            "coil_order": list(COIL_ORDER),
        }
        solver = cls(
            np.array(pos),
            np.array(mom),
            np.array(idx, dtype=int),
            coil_names=list(COIL_ORDER),
            model_info=info,
            bead_diameter_mm=bead_diameter_mm,
            bead_Br=bead_Br,
            current_gain=current_gain,
        )
        solver.validate_model()
        return solver

    def validate_model(self):
        """实例级模型校验：线圈数 / 每线圈偶极子数 / 维度 / 索引范围 / 总数 / 顺序"""
        if self.n_coils != N_COILS:
            raise ModelValidationError(f"n_coils 必须为 {N_COILS}")
        if self.seg_pos.shape != (N_COILS, DIPOLES_PER_COIL, 3):
            raise ModelValidationError(
                f"偶极子排布必须为 ({N_COILS},{DIPOLES_PER_COIL},3)，实际 {self.seg_pos.shape}"
            )
        if self.seg_mom.shape != self.seg_pos.shape:
            raise ModelValidationError("磁矩维度与位置维度不一致")
        if not np.all(np.isfinite(self.seg_pos)) or not np.all(
            np.isfinite(self.seg_mom)
        ):
            raise ModelValidationError("模型存在非有限数值")
        if self.seg_pos.shape[0] * self.seg_pos.shape[1] != TOTAL_DIPOLES:
            raise ModelValidationError(f"偶极子总数必须为 {TOTAL_DIPOLES}")
        if self.coil_names != COIL_ORDER:
            raise ModelValidationError(f"线圈顺序必须为 {COIL_ORDER}")
        return True

    # ---------------- 磁场与解析梯度（向量化） ----------------
    def _field_grad_coils(self, pos):
        """在 pos (3,) 处按线圈预求和的单位电流场与场梯度。
        B 对电流线性：B(I) = Σ_j I_j Bc[j]；梯度张量同理。
        返回 Bc (6,3) [T/A]，Gc (6,3,3) [T/(m·A)]，Gc[j][k,l] = ∂B_k/∂x_l。"""
        d = pos[None, :] - self._seg_pos_flat  # (180,3)
        m = self._seg_mom_flat  # (180,3)
        # 距离正则化：r2 下限 δ²（δ=1µm），同时保护 r、r³、r⁵ 全链路，
        # 偶极子位于场点上时不会产生 NaN/Inf（1µm ≪ 工作距离 60mm，物理无影响）
        d2_min = 1e-12
        r2 = np.maximum(np.einsum("ij,ij->i", d, d), d2_min)
        r = np.sqrt(r2)
        r3 = r**3
        r5 = r3 * r2
        f = np.einsum("ij,ij->i", m, d)  # m·R
        # B = C [3 d (m·d)/r⁵ − m/r³]
        B = MU0_OVER_4PI * (3.0 * f[:, None] * d / r5[:, None] - m / r3[:, None])
        # G_kl = C [3δ_kl f/r⁵ + 3(R_k m_l + m_k R_l)/r⁵ − 15 R_k R_l f/(r⁵ r²)]
        coef1 = 3.0 * f / r5  # (180,)
        coef3 = -15.0 * f / (r5 * r2)  # (180,)
        G = MU0_OVER_4PI * (
            coef1[:, None, None] * np.eye(3)[None, :, :]
            + 3.0
            * (d[:, :, None] * m[:, None, :] + m[:, :, None] * d[:, None, :])
            / r5[:, None, None]
            + coef3[:, None, None] * (d[:, :, None] * d[:, None, :])
        )
        Bc = B.reshape(N_COILS, DIPOLES_PER_COIL, 3).sum(axis=1)  # (6,3)
        Gc = G.reshape(N_COILS, DIPOLES_PER_COIL, 3, 3).sum(axis=1)  # (6,3,3)
        return Bc, Gc

    def field_at(self, pos, currents):
        """合成磁场 B (T)。currents: (6,) 实际电流 (A)"""
        Bc, _ = self._field_grad_coils(np.asarray(pos, dtype=np.float64))
        return Bc.T @ np.asarray(currents, dtype=np.float64)

    def _bg_batch(self, pos, currents):
        """pos (3,) 固定，多组电流 (N,6) 共享基场缓存。
        返回 B (N,3)，G (N,3,3)，|B| (N,)。"""
        I = np.atleast_2d(np.asarray(currents, dtype=np.float64))
        pos = np.asarray(pos, dtype=np.float64)
        if self._cache_pos is None or not np.array_equal(pos, self._cache_pos):
            self._cache_Bc, self._cache_Gc = self._field_grad_coils(pos)
            self._cache_pos = pos.copy()
        Bc, Gc = self._cache_Bc, self._cache_Gc
        B = I @ Bc  # (N,3)
        G = np.einsum("nj,jkl->nkl", I, Gc)  # (N,3,3)
        nb = np.linalg.norm(B, axis=1)
        return B, G, nb

    def force_batch(self, pos, currents):
        """多组电流下磁珠受力 F = m_b·∇|B| = m_b·(BᵀG)/q，(N,3) 单位 N。
        解析梯度，无差分。数值稳定项与梯度/雅可比严格一致：
        q = sqrt(BᵀB + ε²)（ε=b_eps 极小，正常工作区域 |B|≫ε 不改变物理结果）。"""
        B, G, nb = self._bg_batch(pos, currents)
        q = np.sqrt(nb**2 + self.b_eps**2)[:, None]
        return self.bead_moment * np.einsum("nk,nkl->nl", B, G) / q

    def force_at(self, pos, currents):
        """磁珠在 pos 处、电流 currents 下受到的磁力 (N,3)"""
        return self.force_batch(pos, np.asarray(currents, dtype=np.float64)[None, :])[0]

    def forward_model(self, pos, currents):
        """MDM 正向模型：给定电流 → B(mT)、G(mT/m)、∇|B|(mT/m)、F(N)。
        供 30Hz 电流执行层用 I_est（估计电流）评估实际磁场/磁力。"""
        B, G, _ = self._bg_batch(pos, np.asarray(currents, dtype=np.float64)[None, :])
        B0, G0 = B[0], G[0]
        q = math.sqrt(float(B0 @ B0) + self.b_eps**2)
        g_absB = (B0 @ G0) / q
        F = self.bead_moment * (B0 @ G0) / q
        return {
            "B": B0.copy(),
            "B_mT": B0 * 1e3,
            "B_magnitude_mT": float(np.linalg.norm(B0)) * 1e3,
            "G": G0.copy(),
            "grad_absB": g_absB,
            "F": np.array([F] if np.isscalar(F) else F),
        }

    # ------- DEBUG：数值差分梯度（仅供验证解析梯度，不用于正式控制） -------
    def grad_absB_numeric(self, pos, currents, delta=1e-5):
        """二阶两点中心差分计算 ∇|B|，仅用于 DEBUG 验证"""
        pos = np.asarray(pos, dtype=np.float64)
        I = np.asarray(currents, dtype=np.float64)
        g = np.zeros(3)
        for k in range(3):
            p1 = pos.copy()
            p1[k] += delta
            p2 = pos.copy()
            p2[k] -= delta
            b1 = np.linalg.norm(self.field_at(p1, I))
            b2 = np.linalg.norm(self.field_at(p2, I))
            g[k] = (b1 - b2) / (2.0 * delta)
        return g

    def grad_absB(self, pos, currents):
        """解析 ∇|B| (T/m)：(BᵀG)/q，q = sqrt(BᵀB + ε²)（与 force_batch 一致）"""
        B, G, nb = self._bg_batch(pos, np.asarray(currents, dtype=np.float64)[None, :])
        q = math.sqrt(nb[0] ** 2 + self.b_eps**2)
        return (B[0] @ G[0]) / q

    # ---------------- 电流逆解 ----------------
    def _objective(self, pos, F_des, currents, lambda1, lambda2):
        """0.5‖F−F_des‖² + λ1·Σ|I| + 0.5·λ2·ΣI²"""
        F = self.force_batch(pos, currents)
        d = F - F_des[None, :]
        res = 0.5 * np.einsum("ij,ij->i", d, d)
        return (
            res
            + lambda1 * np.abs(currents).sum(axis=1)
            + 0.5 * lambda2 * np.einsum("ij,ij->i", currents, currents)
        )

    def current_jacobian(self, pos, I):
        """解析电流雅可比 J_FI = ∂F/∂I，(3, n) 或 (3,6)。

        由 B(I) = Σ_j I_j·Bc[j]、G(I) = Σ_j I_j·Gc[j]（对电流线性，基场缓存），
        F = m_b·(BᵀG)/q，q = sqrt(BᵀB + ε²)：

            ∂F_a/∂I_j = m_b·[ (B_j·G[:,a] + B·G_j[:,a]) / q
                               − (BᵀG)_a·(B·B_j) / q³ ]

        完全消除有限差分（原 h=0.05·span≈0.2A 差分对 GN 偏大且引入截断误差）。
        力为电流的偶函数：I≈0 处 J≡0（由热启动/非零种子处理）。"""
        I = np.atleast_2d(np.asarray(I, dtype=np.float64))  # (n,6)
        B, G, nb = self._bg_batch(pos, I)  # B(n,3) G(n,3,3)
        Bc, Gc = self._cache_Bc, self._cache_Gc  # (6,3) (6,3,3)
        q = np.sqrt(nb**2 + self.b_eps**2)  # (n,)
        num = np.einsum("nk,nkl->nl", B, G)  # (n,3) BᵀG
        # (B_jᵀG)[n,j,a] = Σ_k Bc[j,k]·G[n,k,a]； (BᵀG_j)[n,j,a] = Σ_k B[n,k]·Gc[j,k,a]
        t1 = np.einsum("jk,nka->nja", Bc, G)
        t2 = np.einsum("nk,jka->nja", B, Gc)
        BtBj = B @ Bc.T  # (n,6) BᵀB_j
        J = self.bead_moment * (
            (t1 + t2) / q[:, None, None] - (num / q**3)[:, None, :] * BtBj[:, :, None]
        )
        return J.transpose(0, 2, 1)  # (n,3,6) [分量,j]

    def _jacobian(self, pos, I, h=None):
        """兼容接口：单电流 → ∂F/∂I (3,6)，解析（h 参数保留但不再使用）"""
        return self.current_jacobian(pos, I)[0]

    def current_jacobian_numeric(self, pos, I, h=None, max_current=MAX_CURRENT_A):
        """数值差分雅可比（仅供验证解析雅可比，不用于正式控制）。
        步长按需求取 h = max(1e-4, 0.02·span)，span=2·max_current。"""
        span = 2.0 * float(max_current)
        if h is None:
            h = max(1e-4, 0.02 * span)
        I = np.asarray(I, dtype=np.float64)
        Jn = np.zeros((3, self.n_coils))
        for j in range(self.n_coils):
            Ip = I.copy()
            Ip[j] += h
            Im = I.copy()
            Im[j] -= h
            Fp = self.force_at(pos, Ip)
            Fm = self.force_at(pos, Im)
            Jn[:, j] = (Fp - Fm) / (2.0 * h)
        return Jn

    def _solve_from(
        self, pos, F_des, I0, lower, upper, lambda1, lambda2, n_iter, conv_tol
    ):
        """单起点：IRLS 加权 + 箱约束投影高斯-牛顿 + 回溯线搜索。
        所有候选解都被投影到 [lower, upper]（约束进入优化，而非解后截断）。
        返回 (I_final, objective, force_error)

        IRLS 系数对应关系（目标函数 → 近似 → 增广LS 严格一致）：
        真实目标      J(u) = 0.5‖res−Jδ‖² + λ1Σ|u_i| + 0.5λ2Σu_i²
        L1 的 IRLS    λ1|u_i| ≤ 0.5·λ1·w_i·u_i² + λ1|Ī_i|/2，w_i = 1/(|Ī_i|+ε)
        二次代理      Q(δ) = 0.5‖res−Jδ‖² + 0.5Σ(λ1·w_i + λ2)·u_i²
        增广LS        lstsq 求 ‖[J;diag(s)]δ − [res;−s·Ī]‖²，s_i = sqrt(λ1·w_i+λ2)，
                      其目标 = 2·Q(δ)（整体乘 2 不改变 argmin）
        → 代码中 s = sqrt(λ1·w + λ2) 与四层系数严格一致（不需要 2·λ1·w）。"""
        I = np.clip(np.asarray(I0, dtype=np.float64), lower, upper)
        eps_w = 0.05 * (upper - lower).max()
        span = (upper - lower).max()
        alphas = np.array([1.0, 0.5, 0.25, 0.1])
        F0 = self.force_batch(pos, I[None, :])[0]
        best = (
            I.copy(),
            float(self._objective(pos, F_des, I[None, :], lambda1, lambda2)[0]),
            float(np.linalg.norm(F0 - F_des)),
        )
        for _ in range(n_iter):
            res = F_des - self.force_batch(pos, I[None, :])[0]
            if np.linalg.norm(res) <= conv_tol * max(np.linalg.norm(F_des), 1e-30):
                break
            J = self._jacobian(pos, I, h=0.05 * span)
            w = 1.0 / (np.abs(I) + eps_w)  # IRLS L1 权重
            s = np.sqrt(lambda1 * w + lambda2)  # L1(二次近似)+L2 合并
            A = np.vstack([J, np.diag(s)])
            y = np.concatenate([res, -s * I])
            try:
                step, *_ = np.linalg.lstsq(A, y, rcond=None)
            except np.linalg.LinAlgError:
                break
            nrm = float(np.max(np.abs(step)))
            if nrm > span:
                step = step * (span / nrm)
            # 回溯线搜索：候选全部投影到箱约束内
            cands = np.clip(I[None, :] + alphas[:, None] * step[None, :], lower, upper)
            objs = self._objective(pos, F_des, cands, lambda1, lambda2)
            k = int(np.argmin(objs))
            if objs[k] >= best[1] * (1 - 1e-12):
                break
            I = cands[k]
            best = (
                I.copy(),
                float(objs[k]),
                float(np.linalg.norm(self.force_batch(pos, I[None, :])[0] - F_des)),
            )
            if best[2] <= conv_tol * max(np.linalg.norm(F_des), 1e-30):
                break
        return best

    def solve_currents(
        self,
        pos,
        F_des,
        I_prev=None,
        max_current=MAX_CURRENT_A,
        max_delta=MAX_DELTA_A,
        lambda1=None,
        lambda2=None,
        n_iter=SOLVER_N_ITER,
        conv_tol=CONV_TOL,
    ):
        """约束正则化电流逆解（多起点）。
        目标：min 0.5‖F(I)−F_des‖² + λ1‖I‖₁ + 0.5λ2‖I‖₂²
        s.t. −max_current ≤ I ≤ max_current 且 |I − I_prev| ≤ max_delta（箱约束）。
        λ1/λ2 缺省按力标度自适应（L1_TAU/L2_TAU=1.0），可显式覆盖。
        返回记录 dict：
          currents(6,) final_current(6,) initial_current(6,) objective
          force_error force_error_percent converged current_constraint_active
          requested_force(3,) achieved_force(3,) n_starts elapsed_ms
        F_actual 按最终电流重新计算。"""
        t0 = time.perf_counter()
        pos = np.asarray(pos, dtype=np.float64)
        F_des = np.asarray(F_des, dtype=np.float64)
        F_norm = float(np.linalg.norm(F_des))
        epsF = 1e-30
        record = {
            "currents": np.zeros(self.n_coils),
            "final_current": np.zeros(self.n_coils),
            "initial_current": np.zeros(self.n_coils),
            "objective": 0.0,
            "force_error": F_norm,
            "force_error_percent": 100.0 * F_norm / max(F_norm, epsF),
            "converged": F_norm < 1e-12,
            "current_constraint_active": False,
            "requested_force": F_des.copy(),
            "achieved_force": np.zeros(3),
            "n_starts": 0,
            "elapsed_ms": 0.0,
        }
        if F_norm < 1e-12:
            # 零力目标：返回 slew 约束允许的最接近零的电流（安全归零路径）。
            # 不得直接返回全零——若上一帧非零，跳变会违反 |ΔI| ≤ max_delta。
            # 状态语义：initial_current = I_prev（真实起点）；converged 仅在
            # 归零后力确实为零（≤ZERO_FORCE_TOL）时为 True——归零中途
            # （电流仍在斜坡上）不算收敛。
            ZERO_FORCE_TOL = 1e-9  # N，可按实验要求调整
            if I_prev is not None:
                I_prev = np.clip(
                    np.asarray(I_prev, dtype=np.float64), -max_current, max_current
                )
                lo0 = np.maximum(-max_current, I_prev - max_delta)
                hi0 = np.minimum(max_current, I_prev + max_delta)
                I_zero = np.clip(np.zeros(self.n_coils), lo0, hi0)
                I_initial = I_prev.copy()
            else:
                I_zero = np.zeros(self.n_coils)
                I_initial = np.zeros(self.n_coils)
            F_act = self.force_batch(pos, I_zero[None, :])[0]
            ferr = float(np.linalg.norm(F_act))
            record.update(
                {
                    "currents": I_zero.copy(),
                    "final_current": I_zero.copy(),
                    "initial_current": I_initial,
                    "force_error": ferr,
                    "force_error_percent": (0.0 if ferr <= ZERO_FORCE_TOL else np.inf),
                    "converged": bool(ferr <= ZERO_FORCE_TOL),
                    "current_constraint_active": bool(np.linalg.norm(I_zero) > 1e-12),
                    "achieved_force": F_act.copy(),
                    "n_starts": 1,
                    "elapsed_ms": (time.perf_counter() - t0) * 1e3,
                }
            )
            return record

        # 自适应正则化系数（λ1: N²/A，λ2: N²/A²）。
        # 标度原则：力匹配优先——正则化项在收敛判据量级 (conv_tol·F)² 上仅作为
        # 力等价解之间的仲裁，默认 lambda1 = tau1·conv_tol²·F²/Imax，
        # lambda2 = tau2·conv_tol²·(F/Imax)²，可用参数显式覆盖。
        if lambda1 is None:
            lambda1 = LAMBDA1_TAU * conv_tol**2 * F_norm**2 / max_current
        if lambda2 is None:
            lambda2 = LAMBDA2_TAU * conv_tol**2 * (F_norm / max_current) ** 2

        # 箱约束：±max_current 与 slew-rate |I−I_prev|≤max_delta 的交集
        if I_prev is not None:
            I_prev = np.clip(
                np.asarray(I_prev, dtype=np.float64), -max_current, max_current
            )
            lower = np.maximum(-max_current, I_prev - max_delta)
            upper = np.minimum(max_current, I_prev + max_delta)
        else:
            lower = np.full(self.n_coils, -max_current)
            upper = np.full(self.n_coils, max_current)

        # 多起点：热启动优先；冷启动用单线圈满幅（避免对称抵消锥）投影到箱内
        starts = []
        if I_prev is not None and np.linalg.norm(I_prev) > 1e-9:
            starts.append(I_prev.copy())
        seed_bank = max_current * np.eye(self.n_coils)
        F_bank = self.force_batch(pos, seed_bank)
        align = F_bank @ (F_des / F_norm)
        for j in np.argsort(-align)[: SOLVER_N_STARTS - len(starts)]:
            starts.append(np.clip(seed_bank[j], lower, upper))
        starts.append(0.5 * (lower + upper))  # 箱中心（保留，不截断）
        starts = starts[: max(SOLVER_N_STARTS, 1) + 1]  # +1 为箱中心预留名额

        results = []
        for I0 in starts:
            I_f, obj, ferr = self._solve_from(
                pos, F_des, I0, lower, upper, lambda1, lambda2, n_iter, conv_tol
            )
            results.append((I0.copy(), I_f, obj, ferr))

        # 选择：先取收敛者中目标函数最小者；都未收敛则取力误差最小者
        conv = [r for r in results if r[3] <= conv_tol * max(F_norm, epsF)]
        pool = conv if conv else results
        key = (lambda r: r[2]) if conv else (lambda r: r[3])
        I0_sel, I_sel, obj_sel, _ferr_sel = min(pool, key=key)

        F_act = self.force_batch(pos, I_sel[None, :])[0]
        ferr = float(np.linalg.norm(F_act - F_des))
        ferr_pct = 100.0 * ferr / max(F_norm, epsF)
        at_bound = np.any(np.isclose(I_sel, lower, atol=1e-9)) or np.any(
            np.isclose(I_sel, upper, atol=1e-9)
        )
        record.update(
            {
                "currents": I_sel.copy(),
                "final_current": I_sel.copy(),
                "initial_current": I0_sel.copy(),
                "objective": float(obj_sel),
                "force_error": ferr,
                "force_error_percent": ferr_pct,
                "converged": bool(ferr_pct <= 100.0 * conv_tol),
                "current_constraint_active": bool(
                    ferr_pct > 100.0 * conv_tol and at_bound
                ),
                "achieved_force": F_act.copy(),
                "n_starts": len(results),
                "elapsed_ms": (time.perf_counter() - t0) * 1e3,
            }
        )
        return record

    def solve_commands(
        self,
        pos,
        F_des,
        cmd_prev=None,
        max_cmd=CMD_MAX,
        max_delta_cmd=MAX_DELTA_CMD,
        lambda1=None,
        lambda2=None,
        n_iter=SOLVER_N_ITER,
        conv_tol=CONV_TOL,
        max_active=N_COILS,
    ):
        """以串口指令为单位的逆解。实际电流 = 指令 × gain；箱约束按指令域等价
        （I_prev = cmd_prev×gain，Δ = max_delta_cmd×gain）。返回的 commands 为整数，
        已在 [cmd_prev−Δcmd, cmd_prev+Δcmd]∩[−max_cmd, max_cmd] 内；
        F_actual / force_error / objective 按**最终发送电流**（指令×gain）重新计算，
        不显示优化器内部未发送的候选结果。

        max_active：同时工作（非零电流）电磁铁数量上限。
        - max_active >= 6：无稀疏约束（默认，多路正则化逆解）。
        - max_active == 1：单极模式，指令域整数穷举精确求解。
        - 2 <= max_active < 6：连续解 → 稀疏投影（保留 |cmd| 最大的前 max_active 路，
          斜率箱内无法归零的通道强制占用名额）→ 保持约束的贪心精修。
        切换/停用线圈受斜率限制，须经零衰减（箱约束自然实现）；
        若强制占用名额数 > max_active，本帧无严格可行解，返回箱内上一帧电流并
        标记 sparse_infeasible=True（调用方应先过渡归零）。"""
        if not (1 <= max_active <= self.n_coils):
            raise ValueError(f"max_active 必须在 1~{self.n_coils}")
        if not (1 <= max_cmd <= CMD_MAX):
            raise ValueError(f"max_cmd 必须在 1~{CMD_MAX}，实际 {max_cmd}")
        if not (0 <= max_delta_cmd <= MAX_DELTA_CMD):
            raise ValueError(
                f"max_delta_cmd 必须在 0~{MAX_DELTA_CMD}，" f"实际 {max_delta_cmd}"
            )
        g = self.current_gain
        F_des_v = np.asarray(F_des, dtype=np.float64)  # 提前（零力委托需要）
        F_norm = float(np.linalg.norm(F_des_v))
        g1 = (
            lambda1
            if lambda1 is not None
            else LAMBDA1_TAU * conv_tol**2 * F_norm**2 / (max_cmd * g)
        )
        g2 = (
            lambda2
            if lambda2 is not None
            else LAMBDA2_TAU * conv_tol**2 * (F_norm / (max_cmd * g)) ** 2
        )
        # 逐通道整数箱约束（cmd_prev 可为标量或数组）
        cp = (
            np.full(self.n_coils, cmd_prev, dtype=float)
            if cmd_prev is not None and np.isscalar(cmd_prev)
            else (
                np.zeros(self.n_coils)
                if cmd_prev is None
                else np.asarray(cmd_prev, float)
            )
        )
        # 严格 slew 保证：整数指令 c 须满足 |c − cp| ≤ max_delta_cmd，
        # 即 c ∈ [ceil(cp−Δ), floor(cp+Δ)]（cp 非整数时比 round±Δ 收紧 1 个指令）
        if cmd_prev is None:
            # 首帧：无 slew 约束，只受幅值 ±max_cmd
            lo = np.full(self.n_coils, -max_cmd)
            hi = np.full(self.n_coils, max_cmd)
        else:
            lo = np.maximum(-max_cmd, np.ceil(cp - max_delta_cmd))
            hi = np.minimum(max_cmd, np.floor(cp + max_delta_cmd))
        t0 = time.perf_counter()

        # 零力目标：委托 solve_currents 的安全归零逻辑（slew 箱内最近零点，
        # 不跳变；归零中途 converged=False），再走同一整数量化/回代路径。
        # 注意必须在 cp/lo/hi/t0 就绪之后。
        if float(np.linalg.norm(F_des_v)) < 1e-12:
            I_prev0 = None if cmd_prev is None else cp * g
            rec0 = self.solve_currents(
                pos,
                np.zeros(3),
                I_prev=I_prev0,
                max_current=max_cmd * g,
                max_delta=max_delta_cmd * g,
                lambda1=lambda1,
                lambda2=lambda2,
                n_iter=n_iter,
                conv_tol=conv_tol,
            )
            cmd0 = np.clip(
                np.round(rec0["final_current"] / g),
                np.full(self.n_coils, -max_cmd),
                np.full(self.n_coils, max_cmd),
            ).astype(int)
            I_sent0 = cmd0 * g
            F_act0 = self.force_batch(pos, I_sent0[None, :])[0]
            ferr0 = float(np.linalg.norm(F_act0))
            slew0 = (
                True
                if cmd_prev is None
                else bool(np.max(np.abs(cmd0 - np.round(cp))) <= max_delta_cmd + 1e-9)
            )
            return {
                "commands": cmd0,
                "currents": I_sent0.copy(),
                "final_current": I_sent0.copy(),
                "initial_current": rec0["initial_current"].copy(),
                "objective": float(rec0["objective"]),
                "force_error": ferr0,
                "force_error_percent": (0.0 if ferr0 <= 1e-9 else np.inf),
                "converged": bool(rec0["converged"] and slew0),
                "current_constraint_active": bool(np.linalg.norm(I_sent0) > 1e-12),
                "achieved_force": F_act0.copy(),
                "requested_force": F_des_v.copy(),
                "max_active": int(max_active),
                "active_coils": [int(i) for i in range(self.n_coils) if cmd0[i] != 0],
                "sparse_infeasible": False,
                "elapsed_ms": (time.perf_counter() - t0) * 1e3,
            }

        sparse_infeasible = False
        active_coils = []
        if max_active == 1:
            cmd, active_coils, sparse_infeasible = self._unipolar_best_cmd(
                pos, F_des_v, cp, lo, hi, g1, g2
            )
            initial_current = cp * g
        else:
            I_prev = None if cmd_prev is None else cp * g
            rec = self.solve_currents(
                pos,
                F_des_v,
                I_prev=I_prev,
                max_current=max_cmd * g,
                max_delta=max_delta_cmd * g,
                lambda1=lambda1,
                lambda2=lambda2,
                n_iter=n_iter,
                conv_tol=conv_tol,
            )
            cmd = np.clip(np.round(rec["currents"] / g), lo, hi).astype(int)
            if max_active >= self.n_coils:
                # ±1 立方体穷举精修（连续最优取整点的邻域，消除量化主要损失），
                # 再接贪心 ±1/±2（覆盖立方体外的整数组合）；
                # 取整后已满足收敛判据则跳过（快速路径）
                err0 = float(
                    np.linalg.norm(
                        self.force_batch(pos, (cmd * g)[None, :])[0] - F_des_v
                    )
                )
                if err0 > conv_tol * F_norm:
                    cmd = self._polish_cube(pos, F_des_v, cmd, lo, hi, g1, g2)
                    cmd = self._refine_commands(pos, F_des_v, cmd, lo, hi, g1, g2)
            if max_active < self.n_coils:
                # 稀疏投影：最多 max_active 路非零（箱内无法归零的通道强制保留）
                cmd, keep, sparse_infeasible = self._project_sparse(
                    cmd,
                    lo,
                    hi,
                    max_active,
                    cmd_prev=(cp if cmd_prev is not None else None),
                )
                if not sparse_infeasible:
                    # 候选活动子集：S1=投影保留集（按无约束解 |cmd|）；
                    # S2=按单线圈力与 F_des 对齐度选 top-K（强制通道必含）。
                    # 对每个子集做子空间连续重解（非活动通道箱收缩为 [0,0] 后
                    # 复用带箱约束的正则化高斯-牛顿），取目标函数最小者。
                    forced = [
                        i for i in range(self.n_coils) if not (lo[i] <= 0 <= hi[i])
                    ]
                    free = [i for i in range(self.n_coils) if i not in forced]
                    F_bank = self.force_batch(pos, max_cmd * g * np.eye(self.n_coils))
                    align = F_bank @ (F_des_v / max(F_norm, 1e-30))
                    S_align = (
                        forced
                        + sorted(free, key=lambda i: -align[i])[
                            : max_active - len(forced)
                        ]
                    )
                    subsets = [sorted(set(keep)), sorted(set(S_align))]
                    best_cmd, best_obj, best_fe = None, np.inf, False
                    for S in subsets:
                        lo2, hi2 = lo.copy(), hi.copy()
                        mask = np.zeros(self.n_coils, dtype=bool)
                        mask[S] = True
                        lo2[~mask] = 0
                        hi2[~mask] = 0
                        I0 = np.clip(cmd * g, lo2, hi2)
                        I_sub, _obj_sub, _ = self._solve_from(
                            pos,
                            F_des_v,
                            I0,
                            lo2,
                            hi2,
                            g1,
                            g2,
                            min(n_iter, 10),
                            conv_tol,
                        )
                        cmd_sub = np.clip(np.round(I_sub / g), lo2, hi2).astype(int)
                        k_sel, objs_sel, _ferrs_sel, feas_sel = (
                            self._select_force_candidate(
                                pos, F_des_v, [cmd_sub], g1, g2, conv_tol
                            )
                        )
                        o = float(objs_sel[k_sel])
                        fe = bool(feas_sel[k_sel])
                        # 可行优先：可行子集解 > objective 最小
                        if (
                            best_cmd is None
                            or (fe and not best_fe)
                            or (fe == best_fe and o < best_obj)
                        ):
                            best_cmd, best_obj, best_fe = cmd_sub, o, fe
                    cmd = best_cmd if best_cmd is not None else cmd
                    # 保持稀疏约束的整数贪心精修（活动通道 ±1/±2）
                    mask = np.zeros(self.n_coils, dtype=bool)
                    mask[[i for i in range(self.n_coils) if cmd[i] != 0]] = True
                    lo2, hi2 = lo.copy(), hi.copy()
                    lo2[~mask] = 0
                    hi2[~mask] = 0
                    cmd = self._refine_sparse(
                        pos, F_des_v, cmd, lo2, hi2, g1, g2, max_active
                    )
                    active_coils = [i for i in range(self.n_coils) if cmd[i] != 0]
            else:
                # 整数量化细化：贪心逐通道 ±1/±2 与成对 ±1，指令域精修
                cmd = self._refine_commands(pos, F_des_v, cmd, lo, hi, g1, g2)
            initial_current = rec["initial_current"]

        # active_coils 一律按最终整数命令重算，保证与实际输出六路命令一致
        active_coils = [i for i in range(self.n_coils) if cmd[i] != 0]

        I_sent = cmd * g
        F_act = self.force_batch(pos, I_sent[None, :])[0]
        ferr = float(np.linalg.norm(F_act - F_des_v))
        ferr_pct = 100.0 * ferr / max(F_norm, 1e-30)
        at_bound = np.any(np.isclose(cmd, lo, atol=1e-9)) or np.any(
            np.isclose(cmd, hi, atol=1e-9)
        )
        record = {
            "commands": cmd,
            "currents": I_sent.copy(),
            "final_current": I_sent.copy(),
            "initial_current": initial_current,
            "objective": float(
                self._objective(pos, F_des_v, I_sent[None, :], g1, g2)[0]
            ),
            "force_error": ferr,
            "force_error_percent": ferr_pct,
            "converged": bool(ferr_pct <= 100.0 * conv_tol and not sparse_infeasible),
            "current_constraint_active": bool(
                (ferr_pct > 100.0 * conv_tol or sparse_infeasible) and at_bound
            ),
            "achieved_force": F_act.copy(),
            "requested_force": F_des_v.copy(),
            "max_active": int(max_active),
            "active_coils": [int(i) for i in active_coils],
            "sparse_infeasible": bool(sparse_infeasible),
            "elapsed_ms": (time.perf_counter() - t0) * 1e3,
        }
        return record

    def _project_sparse(self, cmd, lo, hi, max_active, cmd_prev=None):
        """稀疏投影：最多 max_active 路非零。
        - 强制通道：斜率箱内无法取 0 的通道（|prev| > Δ）必须保留且占名额；
        - 其余（可归零）通道按 |cmd| 从大到小保留至名额用满，其余置 0；
        - 强制通道数 > max_active 时本帧无严格可行解，返回箱内原值并标记。
        返回 (cmd, keep_list(保留通道，含名额内取 0 者), infeasible)。"""
        cmd = np.clip(np.asarray(cmd, dtype=int), lo.astype(int), hi.astype(int))
        forced = [i for i in range(self.n_coils) if not (lo[i] <= 0 <= hi[i])]
        if len(forced) > max_active:
            # 无严格可行解（强制通道数超名额）：返回上一帧命令（箱内裁剪）
            # 或当前候选，keep 返回全部通道仅作诊断。调用方必须检查
            # sparse_infeasible 并走过渡归零，不得当作稀疏解使用。
            if cmd_prev is not None:
                fallback = np.clip(
                    np.asarray(cmd_prev, dtype=int), lo.astype(int), hi.astype(int)
                )
            else:
                fallback = cmd.copy()
            return fallback, list(range(self.n_coils)), True
        zeroable = [i for i in range(self.n_coils) if i not in forced]
        budget = max_active - len(forced)
        order = sorted(zeroable, key=lambda i: -abs(int(cmd[i])))
        keep = sorted(set(forced) | set(order[:budget]))
        out = cmd.copy()
        for i in zeroable:
            if i not in keep:
                out[i] = 0
        return out, keep, False

    # ================= 定向磁场 + 目标力 联合非分离逆解 =================
    # 单一六维电流向量 I 同时优化：目标磁力 + 磁场方向 + 磁场强度 8~12 mT
    # （区间软约束 + 中心偏好）+ L1/L2 正则 + 电流箱约束 + 斜率箱约束。
    # 不做 I_align + I_force 的分离求解。物理模型（180 偶极子、解析梯度、
    # F = m_b∇|B| 自由对齐）与基场缓存全部复用，优化迭代只做矩阵/向量运算。

    def _directed_terms(
        self, pos, I, F_des, F_scale, b_dir_hat, B_min_mT, B_target_mT, B_max_mT
    ):
        """联合目标的各项残差与解析导数（基场缓存，无偶极子遍历）。
        所有磁场量在 mT 域计算（B_internal × 1e3），力残差按 F_scale 归一化。
        返回 dict：r_F(3,) dr_F(3,6) r_dir dr_dir(6,) r_range dr_range(6,)
                   r_center dr_center(6,) B(mT) b_hat Bmag_mT(优化用,含ε) F(N)"""
        I = np.asarray(I, dtype=np.float64)
        B, G, _nb = self._bg_batch(pos, I[None, :])
        B0, G0 = B[0], G[0]
        q = math.sqrt(float(B0 @ B0) + self.b_eps**2)  # T（含 ε 保护的模）
        Bmag_mT = q * 1e3  # 优化用 |B|（与梯度/雅可比同一 q）
        b_hat = B0 / q
        F = self.bead_moment * (B0 @ G0) / q  # N
        Bc = self._cache_Bc
        # 力残差（按 F_scale 归一化，N² 与无量纲磁场项不混用）
        r_F = (F - F_des) / F_scale
        dr_F = self.current_jacobian(pos, I)[0] / F_scale  # (3,6)
        # 方向残差 r_dir = 1 − b̂·b̂_d
        # ∂r_dir/∂I_j = −(b̂_d·B_j)/q + (b̂_d·B)(BᵀB_j)/q³
        r_dir = 1.0 - float(b_hat @ b_dir_hat)
        dr_dir = (
            -(Bc @ b_dir_hat) / q + float(b_dir_hat @ B0) * (B0 @ Bc.T) / q**3
        )  # (6,)
        # 磁场强度区间残差（仅在 8~12 mT 外非零）
        if Bmag_mT < B_min_mT:
            r_range = (B_min_mT - Bmag_mT) / 2.0
            dr_range = -0.5 * 1e3 * (B0 @ Bc.T) / q  # ∂B_mT/∂I_j = (BᵀB_j)/q
        elif Bmag_mT > B_max_mT:
            r_range = (Bmag_mT - B_max_mT) / 2.0
            dr_range = +0.5 * 1e3 * (B0 @ Bc.T) / q
        else:
            r_range, dr_range = 0.0, np.zeros(self.n_coils)
        # 中心偏好残差
        r_center = (Bmag_mT - B_target_mT) / 2.0
        dr_center = 0.5 * 1e3 * (B0 @ Bc.T) / q
        return {
            "r_F": r_F,
            "dr_F": dr_F,
            "r_dir": r_dir,
            "dr_dir": dr_dir,
            "r_range": r_range,
            "dr_range": dr_range,
            "r_center": r_center,
            "dr_center": dr_center,
            "B_mT": B0 * 1e3,
            "b_hat": b_hat,
            "Bmag_mT": float(q * 1e3),
            "F": F,
        }

    def _directed_terms_batch(
        self, pos, I_arr, F_des, F_scale, b_dir_hat, B_min_mT, B_target_mT, B_max_mT
    ):
        """_directed_terms 的批量版本：I_arr (N,6) → 各候选的残差/判据（全向量化，
        供整数 polish 的 729 候选单批评估）。"""
        I = np.atleast_2d(np.asarray(I_arr, dtype=np.float64))
        B, G, nb = self._bg_batch(pos, I)
        Bc = self._cache_Bc
        q = np.sqrt(nb**2 + self.b_eps**2)  # (N,)
        b_hat = B / q[:, None]
        F = self.bead_moment * np.einsum("nk,nkl->nl", B, G) / q[:, None]
        r_F = (F - F_des[None, :]) / F_scale  # (N,3)
        Bmag_mT = q * 1e3
        r_dir = 1.0 - b_hat @ b_dir_hat  # (N,)
        # dr_dir 行（N,6）与 dr_range/dr_center 行（N,6）
        lin_term = (Bc @ b_dir_hat)[None, :] / q[:, None]
        BtBj = B @ Bc.T  # (N,6) BᵀB_j
        quad_term = (b_dir_hat @ B.T)[:, None] * BtBj / q[:, None] ** 3
        dr_dir_rows = -lin_term + quad_term
        r_range = np.where(
            Bmag_mT < B_min_mT,
            (B_min_mT - Bmag_mT) / 2.0,
            np.where(Bmag_mT > B_max_mT, (Bmag_mT - B_max_mT) / 2.0, 0.0),
        )
        dr_sign = np.where(
            Bmag_mT < B_min_mT, -0.5, np.where(Bmag_mT > B_max_mT, 0.5, 0.0)
        )
        dr_range_rows = dr_sign[:, None] * 1e3 * BtBj / q[:, None]
        r_center = (Bmag_mT - B_target_mT) / 2.0
        dr_center_rows = 0.5 * 1e3 * BtBj / q[:, None]
        return {
            "r_F": r_F,
            "r_dir": r_dir,
            "r_range": r_range,
            "r_center": r_center,
            "B_mT": B * 1e3,
            "b_hat": b_hat,
            "Bmag_mT": Bmag_mT,
            "F": F,
            "dir_err_deg_arr": np.degrees(np.arccos(np.clip(b_hat @ b_dir_hat, -1, 1))),
            "range_ok_arr": (Bmag_mT >= B_min_mT) & (Bmag_mT <= B_max_mT),
            "dr_dir_rows": dr_dir_rows,
            "dr_range_rows": dr_range_rows,
            "dr_center_rows": dr_center_rows,
        }

    def _directed_objective(
        self,
        pos,
        I,
        F_des,
        F_scale,
        b_dir_hat,
        B_min_mT,
        B_target_mT,
        B_max_mT,
        lambda_dir,
        lambda_range,
        lambda_center,
        lambda1,
        lambda2,
    ):
        """联合目标（线搜索与选解用，与 GN 增广残差严格一致——全部平方形式）：
        J = 0.5‖r_F‖² + λ_dir·r_dir² + λ_range·r_range² + λ_center·r_center²
            + λ1Σ|I| + 0.5λ2ΣI²
        （GN 增广残差为 √λ·r，其平方 = λ·r²。）"""
        t = self._directed_terms(
            pos, I, F_des, F_scale, b_dir_hat, B_min_mT, B_target_mT, B_max_mT
        )
        I = np.asarray(I, dtype=np.float64)
        return (
            0.5 * float(t["r_F"] @ t["r_F"])
            + lambda_dir * t["r_dir"] ** 2
            + lambda_range * t["r_range"] ** 2
            + lambda_center * t["r_center"] ** 2
            + lambda1 * float(np.abs(I).sum())
            + 0.5 * lambda2 * float(I @ I)
        )

    def _solve_directed_from(
        self,
        pos,
        I0,
        lower,
        upper,
        F_des,
        F_scale,
        b_dir_hat,
        B_min_mT,
        B_target_mT,
        B_max_mT,
        lambda_dir,
        lambda_range,
        lambda_center,
        lambda1,
        lambda2,
        n_iter,
        conv_tol,
        dir_tol_deg,
        eps_w,
        field_only=False,
        force_zero_tol_N=1e-6,
    ):
        """单起点联合 GN：增广残差 [r_F; √λ_dir·r_dir; √λ_range·r_range;
        √λ_center·r_center] + IRLS L1/L2 行，回溯线搜索在联合目标上。
        所有候选解投影到 [lower, upper]。返回 (I, J_obj, terms, 迭代数)。"""
        I = np.clip(np.asarray(I0, dtype=np.float64), lower, upper)
        alphas = np.array([1.0, 0.5, 0.25, 0.1, 0.05])
        args = (
            F_des,
            F_scale,
            b_dir_hat,
            B_min_mT,
            B_target_mT,
            B_max_mT,
            lambda_dir,
            lambda_range,
            lambda_center,
            lambda1,
            lambda2,
        )

        def _feasible(iv, terms_v):
            """可行 = 力 + 磁场区间 + 方向 全部满足（与收敛判据同一优先级）。
            field_only：|F| ≤ force_zero_tol_N（绝对零力约束）；
            正常模式：‖F−F_des‖ ≤ conv_tol·F_scale。"""
            if field_only:
                force_ok = np.linalg.norm(terms_v["F"]) <= force_zero_tol_N
            else:
                force_ok = np.linalg.norm(terms_v["F"] - F_des) <= conv_tol * F_scale
            dir_ok = (
                math.degrees(
                    math.acos(np.clip(float(terms_v["b_hat"] @ b_dir_hat), -1, 1))
                )
                <= dir_tol_deg
            )
            range_ok = B_min_mT <= terms_v["Bmag_mT"] <= B_max_mT
            return bool(force_ok and dir_ok and range_ok)

        best = (
            I.copy(),
            self._directed_objective(pos, I, *args),
            self._directed_terms(
                pos, I, F_des, F_scale, b_dir_hat, B_min_mT, B_target_mT, B_max_mT
            ),
        )
        best_feas = _feasible(best[0], best[2])
        n_iter_used = 0
        for _ in range(n_iter):
            t = self._directed_terms(
                pos, I, F_des, F_scale, b_dir_hat, B_min_mT, B_target_mT, B_max_mT
            )
            n_iter_used += 1
            # 收敛：力 + 磁场区间 + 方向 全部满足（中心偏好不是硬条件）
            if field_only:
                force_ok_iter = np.linalg.norm(t["F"]) <= force_zero_tol_N
            else:
                force_ok_iter = np.linalg.norm(t["F"] - F_des) <= conv_tol * F_scale
            in_range = B_min_mT <= t["Bmag_mT"] <= B_max_mT
            dir_ok = (
                math.degrees(math.acos(np.clip(t["b_hat"] @ b_dir_hat, -1, 1)))
                <= dir_tol_deg
            )
            if force_ok_iter and in_range and dir_ok:
                break
            # 增广残差与雅可比（GN：min ‖res + J_res·δ‖² + IRLS 行）
            A = np.vstack(
                [
                    t["dr_F"],
                    math.sqrt(lambda_dir) * t["dr_dir"][None, :],
                    math.sqrt(lambda_range) * t["dr_range"][None, :],
                    math.sqrt(lambda_center) * t["dr_center"][None, :],
                ]
            )
            r_vec = np.concatenate(
                [
                    t["r_F"],
                    [math.sqrt(lambda_dir) * t["r_dir"]],
                    [math.sqrt(lambda_range) * t["r_range"]],
                    [math.sqrt(lambda_center) * t["r_center"]],
                ]
            )
            # IRLS L1/L2 行（eps_w = max(1e-4, 0.5·current_gain)，需求 十）
            w = 1.0 / (np.abs(I) + eps_w)
            s = np.sqrt(lambda1 * w + lambda2)
            A = np.vstack([A, np.diag(s)])
            y = np.concatenate([-r_vec, -s * I])
            try:
                step, *_ = np.linalg.lstsq(A, y, rcond=None)
            except np.linalg.LinAlgError:
                break
            nrm = float(np.max(np.abs(step)))
            span = (upper - lower).max()
            if nrm > span:
                step = step * (span / nrm)
            cands = np.clip(I[None, :] + alphas[:, None] * step[None, :], lower, upper)
            objs = np.array([self._directed_objective(pos, c, *args) for c in cands])
            feas = [
                bool(
                    _feasible(
                        c,
                        self._directed_terms(
                            pos,
                            c,
                            F_des,
                            F_scale,
                            b_dir_hat,
                            B_min_mT,
                            B_target_mT,
                            B_max_mT,
                        ),
                    )
                )
                for c in cands
            ]
            # 可行优先：可行候选中取目标最小；无可行时取目标最小
            k = int(np.argmin(np.where(feas, objs, objs + 1e18)))
            k_feas = bool(feas[k])
            if best_feas and not k_feas:
                break  # 已有可行解，不退回不可行候选
            if (k_feas and not best_feas) or (
                k_feas == best_feas and objs[k] >= best[1] * (1 - 1e-12)
            ):
                break
            I = cands[k]
            best_feas = k_feas
            best = (
                I.copy(),
                float(objs[k]),
                self._directed_terms(
                    pos, I, F_des, F_scale, b_dir_hat, B_min_mT, B_target_mT, B_max_mT
                ),
            )
        return best[0], best[1], best[2], n_iter_used

    # ================= 文献法：[B; F] = A_BF I + Moore-Penrose 伪逆 =================
    def field_force_actuation_matrix(self, pos, B_direction):
        """构造位置相关的 6×6 field-force 整体驱动矩阵。

        A_B = Bc.T；给定期望场方向 b_hat 后，期望磁矩 M=m*b_hat，
        A_F[:,j] = Gc[j].T @ M。返回 (A_BF, b_hat)。
        """
        direction = np.asarray(B_direction, dtype=np.float64)
        if direction.shape != (3,) or np.linalg.norm(direction) < 1e-12:
            raise ValueError("B_direction 必须是非零三维向量")
        b_hat = direction / np.linalg.norm(direction)
        self._bg_batch(pos, np.zeros((1, self.n_coils)))
        Bc, Gc = self._cache_Bc, self._cache_Gc
        A_B = Bc.T
        moment = self.bead_moment * b_hat
        A_F = np.column_stack([Gc[j].T @ moment for j in range(self.n_coils)])
        return np.vstack([A_B, A_F]), b_hat

    @staticmethod
    def _bounded_pseudoinverse(A, y, lower, upper, rcond=1e-10, l2_weight=1e-8):
        """精确求解小规模箱约束线性最小二乘。

        六通道系统只有 ``3**6=729`` 种自由/下界/上界活动集，因此可穷举
        所有活动集并求每个自由子问题，避免“逐路钳位”漏掉更优的约束解。
        极小的二范数项只在输出残差相同的解之间偏向较小电流。
        """
        A = np.asarray(A, dtype=np.float64)
        y = np.asarray(y, dtype=np.float64)
        lower = np.asarray(lower, dtype=np.float64)
        upper = np.asarray(upper, dtype=np.float64)
        n = A.shape[1]
        if lower.shape != (n,) or upper.shape != (n,):
            raise ValueError("lower/upper 维数必须等于电流通道数")
        if np.any(lower > upper):
            raise ValueError("电流约束下界不能大于上界")

        # 目标函数 ||A x-y||² + lambda||x||² 的正规方程。
        H = A.T @ A + max(float(l2_weight), 0.0) * np.eye(n)
        g = A.T @ y
        best_x = None
        best_free = None
        best_obj = float("inf")
        tol = 1e-11
        for state_tuple in itertools.product((-1, 0, 1), repeat=n):
            # -1=下界，0=自由，1=上界。
            state = np.asarray(state_tuple, dtype=np.int8)
            free = state == 0
            fixed = ~free
            x = np.empty(n, dtype=np.float64)
            x[state < 0] = lower[state < 0]
            x[state > 0] = upper[state > 0]
            if np.any(free):
                rhs = g[free]
                if np.any(fixed):
                    rhs = rhs - H[np.ix_(free, fixed)] @ x[fixed]
                Hff = H[np.ix_(free, free)]
                try:
                    x[free] = np.linalg.solve(Hff, rhs)
                except np.linalg.LinAlgError:
                    x[free] = np.linalg.pinv(Hff, rcond=rcond) @ rhs
                if np.any(x[free] < lower[free] - tol) or np.any(
                    x[free] > upper[free] + tol
                ):
                    continue
            residual = A @ x - y
            obj = float(residual @ residual + max(float(l2_weight), 0.0) * (x @ x))
            if obj < best_obj:
                best_obj, best_x, best_free = obj, x.copy(), free.copy()

        # 全部变量固定的活动集始终可行，正常情况下不可能到这里。
        if best_x is None:
            best_x = np.clip(np.zeros(n), lower, upper)
            best_free = np.zeros(n, dtype=bool)
        return best_x, best_free

    def solve_field_force_pseudoinverse(
        self,
        pos,
        F_des,
        B_direction,
        B_magnitude_mT=10.0,
        cmd_prev=None,
        max_cmd=CMD_MAX,
        max_delta_cmd=MAX_DELTA_CMD,
        field_scale_mT=None,
        force_scale_N=None,
        rcond=1e-10,
        bounded_l2_weight=1e-8,
    ):
        """按文献的整体驱动矩阵，用 Moore–Penrose 伪逆求六路电流。

        对无量纲矩阵 W A_BF 求 ``pinv(W A_BF) @ W y``。无安全约束激活时，
        这是达到 [B_des;F_des] 的最小二范数电流；约束激活时，精确求箱约束
        线性最小二乘。最后在整数指令邻域选择输出残差最小、电流二范数次小的
        实际发送指令。
        """
        t0 = time.perf_counter()
        pos = np.asarray(pos, dtype=np.float64)
        F_des = np.asarray(F_des, dtype=np.float64)
        if pos.shape != (3,) or F_des.shape != (3,):
            raise ValueError("pos 与 F_des 必须是三维向量")
        if B_magnitude_mT <= 0:
            raise ValueError("B_magnitude_mT 必须大于 0")
        if not (1 <= max_cmd <= CMD_MAX):
            raise ValueError(f"max_cmd 必须在 1~{CMD_MAX}")
        if not (0 <= max_delta_cmd <= MAX_DELTA_CMD):
            raise ValueError(f"max_delta_cmd 必须在 0~{MAX_DELTA_CMD}")

        A, b_hat = self.field_force_actuation_matrix(pos, B_direction)
        B_des = B_magnitude_mT * 1e-3 * b_hat
        y = np.concatenate([B_des, F_des])
        B_scale = max((field_scale_mT or B_magnitude_mT) * 1e-3, 1e-9)
        F_scale = max(float(force_scale_N or np.linalg.norm(F_des)), 1e-6)
        scales = np.array([B_scale] * 3 + [F_scale] * 3)
        A_w = A / scales[:, None]
        y_w = y / scales

        cp = (
            np.full(self.n_coils, cmd_prev, dtype=float)
            if cmd_prev is not None and np.isscalar(cmd_prev)
            else (
                np.zeros(self.n_coils)
                if cmd_prev is None
                else np.asarray(cmd_prev, dtype=float)
            )
        )
        if cp.shape != (self.n_coils,):
            raise ValueError(f"cmd_prev 必须是标量或 {self.n_coils} 维向量")
        if cmd_prev is None:
            lo = np.full(self.n_coils, -max_cmd, dtype=float)
            hi = np.full(self.n_coils, max_cmd, dtype=float)
        else:
            lo = np.maximum(-max_cmd, np.ceil(cp - max_delta_cmd))
            hi = np.minimum(max_cmd, np.floor(cp + max_delta_cmd))
        lower, upper = lo * self.current_gain, hi * self.current_gain

        A_pinv = np.linalg.pinv(A_w, rcond=rcond)
        I_pinv = A_pinv @ y_w
        unconstrained = bool(
            np.all(I_pinv >= lower - 1e-12) and np.all(I_pinv <= upper + 1e-12)
        )
        if unconstrained:
            I_cont = I_pinv.copy()
            free_mask = np.ones(self.n_coils, dtype=bool)
        else:
            I_cont, free_mask = self._bounded_pseudoinverse(
                A_w, y_w, lower, upper, rcond=rcond, l2_weight=bounded_l2_weight
            )

        # 整数指令：在连续有界解周围的 3^6 邻域中，先最小化归一化输出残差，
        # 相同残差下选择 ||I||2 更小者，落实文献的最小电流范数原则。
        c0 = np.clip(np.round(I_cont / self.current_gain), lo, hi).astype(int)
        cube = c0[None, :] + DIRECTED_CUBE_OFFSETS
        valid = np.all((cube >= lo) & (cube <= hi), axis=1)
        candidates = cube[valid]
        I_candidates = candidates * self.current_gain
        residuals = I_candidates @ A_w.T - y_w[None, :]
        residual_norm = np.linalg.norm(residuals, axis=1)
        current_norm = np.linalg.norm(I_candidates, axis=1)
        # lexsort 最后一项为第一关键字：残差优先，电流范数次之。
        k = int(np.lexsort((current_norm, residual_norm))[0])
        cmd = candidates[k].astype(int)
        I_sent = I_candidates[k]

        achieved = A @ I_sent
        F_linear = achieved[3:]
        # 最终显示和闭环诊断必须基于真正发出的量化电流重新正向计算。
        # 当量化/约束使磁场偏离目标方向时，实际自由对齐磁力会不同于线性化值。
        forward = self.forward_model(pos, I_sent)
        B_act = np.asarray(forward["B"], dtype=np.float64)
        F_actual = np.atleast_1d(np.asarray(forward["F"], dtype=np.float64))
        Bmag_mT = float(np.linalg.norm(B_act) * 1e3)
        b_hat_act = B_act / max(np.linalg.norm(B_act), 1e-15)
        direction_error = math.degrees(
            math.acos(np.clip(float(b_hat_act @ b_hat), -1.0, 1.0))
        )
        B_err = float(np.linalg.norm(B_act - B_des))
        F_err = float(np.linalg.norm(F_actual - F_des))
        F_linear_err = float(np.linalg.norm(F_linear - F_des))
        F_norm = float(np.linalg.norm(F_des))
        B_err_pct = 100.0 * B_err / max(B_magnitude_mT * 1e-3, 1e-15)
        F_err_pct = 100.0 * F_err / max(F_norm, 1e-15)
        current_ok = bool(np.max(np.abs(cmd)) <= max_cmd)
        slew_ok = (
            True
            if cmd_prev is None
            else bool(np.max(np.abs(cmd - np.round(cp))) <= max_delta_cmd + 1e-9)
        )
        field_ok = bool(B_err_pct <= 5.0)
        force_ok = bool(F_err <= CONV_TOL * max(F_norm, 1e-15))
        rank = int(np.linalg.matrix_rank(A_w))
        singular_values = np.linalg.svd(A_w, compute_uv=False)
        cond = (
            float(singular_values[0] / singular_values[-1])
            if singular_values[-1] > 0
            else float("inf")
        )
        at_bound = bool(
            np.any(np.isclose(I_sent, lower, atol=0.5 * self.current_gain))
            or np.any(np.isclose(I_sent, upper, atol=0.5 * self.current_gain))
        )
        return {
            "commands": cmd,
            "currents": I_sent.copy(),
            "pseudoinverse_current": I_pinv.copy(),
            "bounded_continuous_current": I_cont.copy(),
            "minimum_current_norm_A": float(np.linalg.norm(I_pinv)),
            "achieved_force": F_actual.copy(),
            "achieved_force_nonlinear": F_actual.copy(),
            "achieved_force_linear": F_linear.copy(),
            "requested_force": F_des.copy(),
            "force_error": F_err,
            "force_error_percent": F_err_pct,
            "linear_force_error": F_linear_err,
            "linear_force_error_percent": 100.0 * F_linear_err / max(F_norm, 1e-15),
            "force_ok": force_ok,
            "B": B_act.copy(),
            "B_magnitude_mT": Bmag_mT,
            "B_direction": b_hat_act.copy(),
            "B_direction_error_deg": direction_error,
            "field_vector_error_T": B_err,
            "field_vector_error_percent": B_err_pct,
            "field_magnitude_error_mT": Bmag_mT - B_magnitude_mT,
            "field_magnitude_error_percent": 100.0
            * abs(Bmag_mT - B_magnitude_mT)
            / B_magnitude_mT,
            "field_in_range": field_ok,
            "field_range_ok": field_ok,
            "direction_ok": bool(direction_error <= 5.0),
            "requested_B_direction": b_hat.copy(),
            "requested_B_magnitude_mT": float(B_magnitude_mT),
            "requested_B": B_des.copy(),
            "current_bound_ok": current_ok,
            "slew_ok": slew_ok,
            "converged": bool(field_ok and force_ok and current_ok and slew_ok),
            "current_constraint_active": bool(not unconstrained or at_bound),
            "field_constraint_active": bool(not field_ok),
            "direction_constraint_active": bool(direction_error > 5.0),
            "solver_mode": "field-force-moore-penrose",
            "actuation_matrix": A.copy(),
            "normalized_actuation_matrix": A_w.copy(),
            "actuation_rank": rank,
            "actuation_condition": cond,
            "singular_values": singular_values.copy(),
            "pseudoinverse": A_pinv.copy(),
            "unconstrained_pseudoinverse": unconstrained,
            "free_channels_after_bounds": [int(i) for i in np.flatnonzero(free_mask)],
            "objective": float(residual_norm[k] ** 2),
            "elapsed_ms": (time.perf_counter() - t0) * 1e3,
            "iterations": 1,
            "gn_iterations": 0,
            "polish_iterations": 1,
            "active_coils": [int(i) for i in range(self.n_coils) if cmd[i] != 0],
            "sparse_infeasible": False,
            "log_lines": [
                f"[B;F]=A I, rank={rank}, cond={cond:.3g}",
                f"B_des={np.round(B_des*1e3, 4)} mT, F_des={np.round(F_des*1e6, 3)} µN",
                f"I=A^+y={np.round(I_pinv, 5)} A, ||I||2={np.linalg.norm(I_pinv):.5f} A",
                (
                    f"command={cmd.tolist()}, B={np.round(B_act*1e3, 4)} mT, "
                    f"F_actual={np.round(F_actual*1e6, 3)} µN, "
                    f"F_linear={np.round(F_linear*1e6, 3)} µN"
                ),
            ],
        }

    # ================= 自由方向磁场幅值 + 目标力 联合逆解 =================
    def _magnitude_terms(self, pos, I, F_des, F_scale, B_min_mT, B_target_mT, B_max_mT):
        """自由场方向联合目标的残差和解析导数，不包含任何方向残差。"""
        I = np.asarray(I, dtype=np.float64)
        B, G, _ = self._bg_batch(pos, I[None, :])
        B0, G0 = B[0], G[0]
        q = math.sqrt(float(B0 @ B0) + self.b_eps**2)
        Bmag_mT = q * 1e3
        F = self.bead_moment * (B0 @ G0) / q
        Bc = self._cache_Bc
        r_F = (F - F_des) / F_scale
        dr_F = self.current_jacobian(pos, I)[0] / F_scale
        dBmag = 1e3 * (B0 @ Bc.T) / q
        if Bmag_mT < B_min_mT:
            r_range = (B_min_mT - Bmag_mT) / 2.0
            dr_range = -0.5 * dBmag
        elif Bmag_mT > B_max_mT:
            r_range = (Bmag_mT - B_max_mT) / 2.0
            dr_range = 0.5 * dBmag
        else:
            r_range, dr_range = 0.0, np.zeros(self.n_coils)
        r_center = (Bmag_mT - B_target_mT) / 2.0
        dr_center = 0.5 * dBmag
        return {
            "r_F": r_F,
            "dr_F": dr_F,
            "r_range": r_range,
            "dr_range": dr_range,
            "r_center": r_center,
            "dr_center": dr_center,
            "B": B0,
            "B_mT": B0 * 1e3,
            "Bmag_mT": float(Bmag_mT),
            "F": F,
        }

    def _magnitude_terms_batch(
        self, pos, I_arr, F_des, F_scale, B_min_mT, B_target_mT, B_max_mT
    ):
        """自由场方向联合目标的批量版本，供整数指令精修。"""
        I = np.atleast_2d(np.asarray(I_arr, dtype=np.float64))
        B, G, nb = self._bg_batch(pos, I)
        q = np.sqrt(nb**2 + self.b_eps**2)
        F = self.bead_moment * np.einsum("nk,nkl->nl", B, G) / q[:, None]
        Bmag_mT = q * 1e3
        r_F = (F - F_des[None, :]) / F_scale
        r_range = np.where(
            Bmag_mT < B_min_mT,
            (B_min_mT - Bmag_mT) / 2.0,
            np.where(Bmag_mT > B_max_mT, (Bmag_mT - B_max_mT) / 2.0, 0.0),
        )
        r_center = (Bmag_mT - B_target_mT) / 2.0
        return {
            "r_F": r_F,
            "r_range": r_range,
            "r_center": r_center,
            "B": B,
            "B_mT": B * 1e3,
            "Bmag_mT": Bmag_mT,
            "F": F,
            "range_ok_arr": ((Bmag_mT >= B_min_mT) & (Bmag_mT <= B_max_mT)),
        }

    @staticmethod
    def _magnitude_objective_from_terms(
        terms, I, lambda_range, lambda_center, lambda1, lambda2
    ):
        I = np.asarray(I, dtype=np.float64)
        return (
            0.5 * float(terms["r_F"] @ terms["r_F"])
            + lambda_range * terms["r_range"] ** 2
            + lambda_center * terms["r_center"] ** 2
            + lambda1 * float(np.abs(I).sum())
            + 0.5 * lambda2 * float(I @ I)
        )

    def _solve_magnitude_from(
        self,
        pos,
        I0,
        lower,
        upper,
        F_des,
        F_scale,
        B_min_mT,
        B_target_mT,
        B_max_mT,
        lambda_range,
        lambda_center,
        lambda1,
        lambda2,
        n_iter,
        conv_tol,
        eps_w,
    ):
        """单起点箱约束 Gauss–Newton；磁场方向在迭代中完全自由。"""
        I = np.clip(np.asarray(I0, dtype=np.float64), lower, upper)
        alphas = np.array([1.0, 0.5, 0.25, 0.1, 0.05])

        def evaluate(iv):
            t = self._magnitude_terms(
                pos, iv, F_des, F_scale, B_min_mT, B_target_mT, B_max_mT
            )
            obj = self._magnitude_objective_from_terms(
                t, iv, lambda_range, lambda_center, lambda1, lambda2
            )
            feasible = bool(
                np.linalg.norm(t["F"] - F_des) <= conv_tol * F_scale
                and B_min_mT <= t["Bmag_mT"] <= B_max_mT
            )
            return obj, t, feasible

        best_obj, best_terms, best_feasible = evaluate(I)
        best_I = I.copy()
        used = 0
        for _ in range(n_iter):
            used += 1
            if best_feasible:
                break
            t = self._magnitude_terms(
                pos, I, F_des, F_scale, B_min_mT, B_target_mT, B_max_mT
            )
            A = np.vstack(
                [
                    t["dr_F"],
                    math.sqrt(lambda_range) * t["dr_range"][None, :],
                    math.sqrt(lambda_center) * t["dr_center"][None, :],
                ]
            )
            r_vec = np.concatenate(
                [
                    t["r_F"],
                    [math.sqrt(lambda_range) * t["r_range"]],
                    [math.sqrt(lambda_center) * t["r_center"]],
                ]
            )
            w = 1.0 / (np.abs(I) + eps_w)
            reg = np.sqrt(lambda1 * w + lambda2)
            A = np.vstack([A, np.diag(reg)])
            y = np.concatenate([-r_vec, -reg * I])
            try:
                step, *_ = np.linalg.lstsq(A, y, rcond=None)
            except np.linalg.LinAlgError:
                break
            span = float(np.max(upper - lower))
            nrm = float(np.max(np.abs(step)))
            if nrm > span > 0:
                step *= span / nrm
            candidates = np.clip(
                I[None, :] + alphas[:, None] * step[None, :], lower, upper
            )
            evaluated = [evaluate(c) for c in candidates]
            feasible_mask = np.array([e[2] for e in evaluated], bool)
            objs = np.array([e[0] for e in evaluated])
            k = int(np.argmin(np.where(feasible_mask, objs, objs + 1e18)))
            obj_k, terms_k, feasible_k = evaluated[k]
            if best_feasible and not feasible_k:
                break
            if (feasible_k and not best_feasible) or (
                feasible_k == best_feasible and obj_k < best_obj * (1.0 - 1e-12)
            ):
                I = candidates[k]
                best_I, best_obj = I.copy(), float(obj_k)
                best_terms, best_feasible = terms_k, bool(feasible_k)
            else:
                break
        return best_I, best_obj, best_terms, used

    def solve_force_with_field_magnitude(
        self,
        pos,
        F_des,
        B_min_mT=8.0,
        B_target_mT=10.0,
        B_max_mT=12.0,
        cmd_prev=None,
        max_cmd=CMD_MAX,
        max_delta_cmd=MAX_DELTA_CMD,
        lambda_range=10.0,
        lambda_center=0.1,
        lambda1=1e-4,
        lambda2=1e-4,
        n_iter=12,
        conv_tol=CONV_TOL,
        realtime=False,
    ):
        """目标力 + 磁场幅值联合逆解，磁场方向完全自由。

        最小化力误差、8~12mT 区间误差、对 10mT 的弱中心偏好及电流代价；
        使用解析 dF/dI 和 d|B|/dI，并在整数指令域做最终精修。原有
        ``solve_force_and_field`` 保留给需要姿态控制的定向场任务。
        """
        t0 = time.perf_counter()
        if not (1 <= max_cmd <= CMD_MAX):
            raise ValueError(f"max_cmd 必须在 1~{CMD_MAX}，实际 {max_cmd}")
        if not (0 <= max_delta_cmd <= MAX_DELTA_CMD):
            raise ValueError(f"max_delta_cmd 必须在 0~{MAX_DELTA_CMD}")
        if not (B_min_mT <= B_target_mT <= B_max_mT):
            raise ValueError("必须满足 B_min_mT ≤ B_target_mT ≤ B_max_mT")
        F_des = np.asarray(F_des, dtype=np.float64)
        if F_des.shape != (3,):
            raise ValueError("F_des 必须是三维向量")
        F_scale = max(float(np.linalg.norm(F_des)), 1e-6)
        cp = (
            np.full(self.n_coils, cmd_prev, dtype=float)
            if cmd_prev is not None and np.isscalar(cmd_prev)
            else (
                np.zeros(self.n_coils)
                if cmd_prev is None
                else np.asarray(cmd_prev, dtype=float)
            )
        )
        if cmd_prev is None:
            lo = np.full(self.n_coils, -max_cmd)
            hi = np.full(self.n_coils, max_cmd)
        else:
            lo = np.maximum(-max_cmd, np.ceil(cp - max_delta_cmd))
            hi = np.minimum(max_cmd, np.floor(cp + max_delta_cmd))
        lower, upper = lo * self.current_gain, hi * self.current_gain

        # 自由方向问题在 I=0 处不可微；用多个候选场方向的线性解只作为 GN 初值。
        self._bg_batch(pos, np.zeros((1, self.n_coils)))
        Bc, Gc = self._cache_Bc, self._cache_Gc
        dirs = [
            np.array(v, float)
            for v in (
                (1, 0, 0),
                (-1, 0, 0),
                (0, 1, 0),
                (0, -1, 0),
                (0, 0, 1),
                (0, 0, -1),
                (1, 1, 1),
                (1, 1, -1),
                (1, -1, 1),
                (1, -1, -1),
                (-1, 1, 1),
                (-1, 1, -1),
                (-1, -1, 1),
                (-1, -1, -1),
            )
        ]
        starts = []
        prev_I = np.clip(cp * self.current_gain, lower, upper)
        if np.linalg.norm(prev_I) > 1e-12:
            starts.append(prev_I)
        grad_target = F_des / self.bead_moment
        for direction in dirs:
            bh = direction / np.linalg.norm(direction)
            M = np.zeros((6, self.n_coils))
            M[:3, :] = Bc.T
            rhs = np.zeros(6)
            rhs[:3] = B_target_mT * 1e-3 * bh
            for axis in range(3):
                M[3 + axis, :] = Gc[:, :, axis] @ bh
                rhs[3 + axis] = grad_target[axis]
            try:
                seed, *_ = np.linalg.lstsq(M, rhs, rcond=None)
                starts.append(np.clip(seed, lower, upper))
            except np.linalg.LinAlgError:
                pass
        if not starts:
            starts = [np.clip(np.full(self.n_coils, self.current_gain), lower, upper)]

        # 先廉价排序，再只对最佳若干初值运行非线性迭代，满足实时控制预算。
        scored = []
        for seed in starts:
            terms = self._magnitude_terms(
                pos, seed, F_des, F_scale, B_min_mT, B_target_mT, B_max_mT
            )
            obj = self._magnitude_objective_from_terms(
                terms, seed, lambda_range, lambda_center, lambda1, lambda2
            )
            scored.append((obj, seed))
        n_starts = 4 if realtime else min(10, len(scored))
        selected = [x[1] for x in sorted(scored, key=lambda x: x[0])[:n_starts]]
        eps_w = max(1e-4, 0.5 * self.current_gain)
        solved = []
        total_gn = 0
        for seed in selected:
            result = self._solve_magnitude_from(
                pos,
                seed,
                lower,
                upper,
                F_des,
                F_scale,
                B_min_mT,
                B_target_mT,
                B_max_mT,
                lambda_range,
                lambda_center,
                lambda1,
                lambda2,
                min(n_iter, 6) if realtime else n_iter,
                conv_tol,
                eps_w,
            )
            total_gn += result[3]
            terms = result[2]
            feasible = bool(
                np.linalg.norm(terms["F"] - F_des) <= conv_tol * F_scale
                and B_min_mT <= terms["Bmag_mT"] <= B_max_mT
            )
            solved.append((*result[:3], feasible))
        feasible_solved = [s for s in solved if s[3]]
        pool = feasible_solved if feasible_solved else solved
        I_best, _, _, _ = min(pool, key=lambda x: x[1])

        def batch_eval(commands):
            C = np.atleast_2d(np.asarray(commands, dtype=float))
            I = C * self.current_gain
            terms = self._magnitude_terms_batch(
                pos, I, F_des, F_scale, B_min_mT, B_target_mT, B_max_mT
            )
            objs = (
                0.5 * np.einsum("ij,ij->i", terms["r_F"], terms["r_F"])
                + lambda_range * terms["r_range"] ** 2
                + lambda_center * terms["r_center"] ** 2
                + lambda1 * np.abs(I).sum(axis=1)
                + 0.5 * lambda2 * np.einsum("ij,ij->i", I, I)
            )
            force_ok = (
                np.linalg.norm(terms["F"] - F_des[None, :], axis=1)
                <= conv_tol * F_scale
            )
            return objs, force_ok & terms["range_ok_arr"]

        cmd0 = np.clip(np.round(I_best / self.current_gain), lo, hi).astype(int)
        cube = cmd0[None, :] + DIRECTED_CUBE_OFFSETS
        cube = cube[np.all((cube >= lo) & (cube <= hi), axis=1)]
        objs, ok = batch_eval(cube)
        k = int(np.argmin(np.where(ok, objs, objs + 1e18)))
        cmd, best_obj, best_ok = cube[k].copy(), float(objs[k]), bool(ok[k])
        polish_iters = 1
        for _ in range(2):
            neighbours = []
            for j in range(self.n_coils):
                for delta in (1, -1, 2, -2):
                    v = cmd.copy()
                    v[j] = int(np.clip(v[j] + delta, lo[j], hi[j]))
                    if v[j] != cmd[j]:
                        neighbours.append(v)
            if not neighbours:
                break
            objs, ok = batch_eval(neighbours)
            k = int(np.argmin(np.where(ok, objs, objs + 1e18)))
            candidate_ok, candidate_obj = bool(ok[k]), float(objs[k])
            if (candidate_ok and not best_ok) or (
                candidate_ok == best_ok and candidate_obj < best_obj
            ):
                cmd = np.asarray(neighbours[k], int)
                best_obj, best_ok = candidate_obj, candidate_ok
                polish_iters += 1
            else:
                break

        I_sent = cmd * self.current_gain
        out = self.forward_model(pos, I_sent)
        F_act = np.atleast_1d(out["F"]).astype(float)
        B_act = np.asarray(out["B"], float)
        Bmag_mT = float(out["B_magnitude_mT"])
        F_err = float(np.linalg.norm(F_act - F_des))
        F_err_pct = 100.0 * F_err / max(float(np.linalg.norm(F_des)), 1e-30)
        force_ok = bool(F_err <= conv_tol * F_scale)
        field_ok = bool(B_min_mT <= Bmag_mT <= B_max_mT)
        current_ok = bool(np.max(np.abs(cmd)) <= max_cmd)
        slew_ok = (
            True
            if cmd_prev is None
            else bool(np.max(np.abs(cmd - np.round(cp))) <= max_delta_cmd + 1e-9)
        )
        at_bound = bool(np.any(np.isclose(cmd, lo)) or np.any(np.isclose(cmd, hi)))
        field_error_mT = Bmag_mT - B_target_mT
        converged = bool(force_ok and field_ok and current_ok and slew_ok)
        return {
            "commands": cmd,
            "currents": I_sent,
            "achieved_force": F_act,
            "requested_force": F_des.copy(),
            "force_error": F_err,
            "force_error_percent": F_err_pct,
            "force_ok": force_ok,
            "B": B_act,
            "B_magnitude_mT": Bmag_mT,
            "B_direction": B_act / max(np.linalg.norm(B_act), 1e-12),
            "field_magnitude_error_mT": field_error_mT,
            "field_magnitude_error_percent": 100.0
            * abs(field_error_mT)
            / max(B_target_mT, 1e-12),
            "field_in_range": field_ok,
            "field_range_ok": field_ok,
            "direction_ok": True,
            "B_direction_error_deg": None,
            "requested_B_direction": None,
            "requested_B_magnitude_mT": float(B_target_mT),
            "requested_B_range_mT": (float(B_min_mT), float(B_max_mT)),
            "current_bound_ok": current_ok,
            "slew_ok": slew_ok,
            "converged": converged,
            "current_constraint_active": bool(
                (not force_ok or not field_ok) and at_bound
            ),
            "field_constraint_active": bool(not field_ok),
            "direction_constraint_active": False,
            "solver_mode": "free-field-magnitude",
            "realtime": bool(realtime),
            "iterations": int(total_gn + polish_iters),
            "gn_iterations": int(total_gn),
            "polish_iterations": int(polish_iters),
            "elapsed_ms": (time.perf_counter() - t0) * 1e3,
            "objective": best_obj,
            "active_coils": [int(i) for i in range(self.n_coils) if cmd[i] != 0],
            "sparse_infeasible": False,
            "log_lines": [
                f"Target force: {np.round(F_des * 1e6, 3)} µN",
                f"Target |B|: {B_target_mT:.3f} mT, direction free",
                f"Achieved force: {np.round(F_act * 1e6, 3)} µN, error {F_err_pct:.2f}%",
                f"Achieved B: {np.round(B_act * 1e3, 4)} mT, |B|={Bmag_mT:.3f} mT",
                f"Command: {cmd.tolist()}, converged={converged}",
            ],
        }

    def solve_force_and_field(
        self,
        pos,
        F_des,
        B_direction,
        B_min_mT=8.0,
        B_target_mT=10.0,
        B_max_mT=12.0,
        cmd_prev=None,
        max_cmd=CMD_MAX,
        max_delta_cmd=MAX_DELTA_CMD,
        lambda_dir=25.0,
        lambda_range=10.0,
        lambda_center=0.1,
        lambda1=1e-4,
        lambda2=1e-4,
        n_iter=25,
        conv_tol=CONV_TOL,
        dir_tol_deg=5.0,
        force_zero_tol_N=1e-6,
        realtime=False,
    ):
        """定向磁场 + 目标力 联合非分离逆解（单一六维电流向量，不分离求解）。

        核心洞察：B(I) 与 G(I) 均对电流线性，因此同时要求
            B = B_target·b̂_d（3 个线性方程）
            F = m_b·b̂_dᵀG(I) = F_des（3 个线性方程）
        构成 [B;F]=A I 的 6×6 线性方程组。B、F 行先按各自典型尺度
        无量纲化，再计算 rank、condition number 和显式 Moore–Penrose 伪逆。
        若其解满足 |I| ≤ 2A（与 slew 箱），则该解
        **精确**同时满足目标力、磁场方向与强度（非线性分母 |B| 被 B=B_target
        的约束严格抵消），无需 GN。
        若线性解越界（±2A / slew 箱）或方程组奇异，回退到约束 GN 多起点
        （联合目标线搜索）求箱内最接近解，并如实报告各项可行性状态。

        联合目标（量化 polish 与 GN 回退共用，与 GN 增广残差一致——全部平方形式）：
            J = 0.5‖(F−F_des)/F_scale‖² + λ_dir·r_dir²
              + λ_range·r_range² + λ_center·r_center² + λ1‖I‖₁ + 0.5λ2‖I‖₂²

        收敛判据（同时满足）：
        - 普通模式（F_des 给定）：力相对误差 ≤2%；
        - 纯磁场模式（F_des=None）：残余磁力 |F| ≤ force_zero_tol_N；
        - 两种模式均需满足 B_min ≤ |B| ≤ B_max 与方向误差 ≤ dir_tol_deg。
        中心偏好（接近 B_target）不是硬条件。
        纯磁场模式物理定义：低梯度定向磁场——B 指向指定方向、幅值达标，
        同时 ∇|B|（即磁珠受力 F=m_b∇|B|）压到零力容差以内，用于长条微磁铁
        磁矩/长轴与外场对齐实验（力矩 τ=m×B 主导，平移受力被抑制）。
        B_direction 为世界系三维向量（自动归一化，零向量拒绝）。

        返回记录 dict：commands/currents/achieved_force/B/B_magnitude_mT/
        B_direction/B_direction_error_deg/field_in_range/force_error/
        force_error_percent/force_ok/field_range_ok/direction_ok/
        current_bound_ok/slew_ok/converged/*_constraint_active/
        iterations/elapsed_ms/log_lines/objective。"""
        t0 = time.perf_counter()
        if not (1 <= max_cmd <= CMD_MAX):
            raise ValueError(f"max_cmd 必须在 1~{CMD_MAX}，实际 {max_cmd}")
        if not (0 <= max_delta_cmd <= MAX_DELTA_CMD):
            raise ValueError(
                f"max_delta_cmd 必须在 0~{MAX_DELTA_CMD}，" f"实际 {max_delta_cmd}"
            )
        b_dir = np.asarray(B_direction, dtype=np.float64)
        if np.linalg.norm(b_dir) < 1e-12:
            raise ValueError("B_direction 不能为零向量")
        b_dir_hat = b_dir / np.linalg.norm(b_dir)  # 世界系归一化
        if not (B_min_mT <= B_target_mT <= B_max_mT):
            raise ValueError(
                f"要求 B_min_mT ≤ B_target_mT ≤ B_max_mT，"
                f"实际 {B_min_mT} / {B_target_mT} / {B_max_mT}"
            )
        field_only = F_des is None
        if field_only:
            # 纯磁场模式（长条微磁铁磁矩对齐实验）：F_des=0 仍是**必要约束**——
            # F_scale 取零力容差，使 µN 级残余力在目标函数中保持可感知的量级
            # （F=1µN → r_F=1），优化器不会对它视而不见
            F_des_v = np.zeros(3)
            F_scale = max(float(force_zero_tol_N), 1e-12)
        else:
            F_des_v = np.asarray(F_des, dtype=np.float64)
            F_scale = max(float(np.linalg.norm(F_des_v)), 1e-12)

        # 整数指令箱约束（与 solve_commands 完全一致的 ceil/floor 严格 slew）
        cp = (
            np.full(self.n_coils, cmd_prev, dtype=float)
            if cmd_prev is not None and np.isscalar(cmd_prev)
            else (
                np.zeros(self.n_coils)
                if cmd_prev is None
                else np.asarray(cmd_prev, dtype=float)
            )
        )
        if cmd_prev is None:
            # 首帧：无 slew 约束，只受幅值 ±max_cmd（±99 ↔ ±2A）
            lo = np.full(self.n_coils, -max_cmd)
            hi = np.full(self.n_coils, max_cmd)
        else:
            lo = np.maximum(-max_cmd, np.ceil(cp - max_delta_cmd))
            hi = np.minimum(max_cmd, np.floor(cp + max_delta_cmd))
        max_cur = max_cmd * self.current_gain  # ±2A
        if cmd_prev is None:
            # 首帧（cmd_prev=None）：只受 |I| ≤ 2A，无 slew 约束（需求 十一）
            lower = np.full(self.n_coils, -max_cur)
            upper = np.full(self.n_coils, max_cur)
        else:
            lower = np.maximum(-max_cur, (cp - max_delta_cmd) * self.current_gain)
            upper = np.minimum(max_cur, (cp + max_delta_cmd) * self.current_gain)

        # ---- 联合线性精确解 ----
        # 使用与新求解器相同的 [B;F]=A I 结构；纯磁场模式也保留后三行，
        # 以 [B_des;0] 的 6×6 伪逆同时满足目标场并尽量消除平移力。
        self._bg_batch(pos, np.zeros(6)[None, :])  # 预热基场缓存
        Bc, Gc = self._cache_Bc, self._cache_Gc  # (6,3), (6,3,3)
        M = np.zeros((6, 6))
        M[:3, :] = Bc.T  # B 分量行
        rhs = np.zeros(6)
        rhs[:3] = B_target_mT * 1e-3 * b_dir_hat  # T
        m_b = self.bead_moment
        for l in range(3):
            M[3 + l, :] = m_b * (Gc[:, :, l] @ b_dir_hat)  # F_l 对电流的系数
            rhs[3 + l] = 0.0 if field_only else F_des_v[l]
        # T 与 N 不可直接比较；秩、条件数和伪逆都只在无量纲矩阵上计算。
        B_row_scale = max(B_target_mT * 1e-3, 1e-9)
        F_row_scale = max(F_scale, force_zero_tol_N, 1e-12)
        row_scales = np.array([B_row_scale] * 3 + [F_row_scale] * 3)
        M_use = M / row_scales[:, None]
        rhs_use = rhs / row_scales
        lin_exact = False
        I_lin = None
        M_rank = int(np.linalg.matrix_rank(M_use))
        M_cond = float(np.linalg.cond(M_use))
        try:
            I_lin = np.linalg.pinv(M_use, rcond=1e-10) @ rhs_use
            resid_ok = bool(np.linalg.norm(M_use @ I_lin - rhs_use) < 1e-9)
            # 秩满 + 条件数可接受才认作线性精确解，否则回退约束 GN。
            lin_exact = bool(
                resid_ok and M_rank >= M_use.shape[0] and M_cond < LINEAR_COND_MAX
            )
        except np.linalg.LinAlgError:
            M_rank, M_cond = -1, float("inf")

        eps_w = max(1e-4, 0.5 * self.current_gain)  # 需求 十
        iters_used = 0  # 真实 GN 迭代计数（各起点累计）
        polish_iters = 0  # 整数 polish 迭代计数
        _args = (
            F_des_v,
            F_scale,
            b_dir_hat,
            B_min_mT,
            B_target_mT,
            B_max_mT,
            lambda_dir,
            lambda_range,
            lambda_center,
            lambda1,
            lambda2,
        )

        # ---- 候选生成 ----
        candidates = []
        if realtime:
            # 实时控制路径：固定目标场方向后，B 与 ∇|B| 对 I 的联合方程是线性的。
            # 直接把 6×6 线性解投影进幅值/slew 箱，再在整数域批量精修；避免多起点
            # Python GN 长时间持有 GIL、拖慢 30Hz 视觉线程。后续帧的移动箱会逐步
            # 到达未受 slew 限制的线性目标。
            if I_lin is not None:
                candidates.append(np.clip(np.asarray(I_lin, float), lower, upper))
            else:
                candidates.append(np.clip(cp * self.current_gain, lower, upper))
        elif lin_exact and np.all(I_lin >= lower) and np.all(I_lin <= upper):
            candidates.append(I_lin.copy())  # 线性精确解（箱内）
        else:
            # 回退：约束 GN 多起点（箱内最接近解）
            starts = []
            if lin_exact or I_lin is not None:
                starts.append(np.clip(np.asarray(I_lin, float), lower, upper))
            if cmd_prev is not None and np.linalg.norm(cp) > 1e-9:
                starts.append(cp * self.current_gain)
            if not realtime:
                rec_force = self.solve_currents(
                    pos, F_des_v, I_prev=None, n_iter=min(n_iter, 12)
                )
                starts.append(np.clip(rec_force["final_current"], lower, upper))
                for _ in range(3):
                    starts.append(
                        np.clip(
                            self._rng.uniform(-1.0, 1.0, self.n_coils), lower, upper
                        )
                    )
            if not starts:
                starts.append(np.clip(np.zeros(self.n_coils), lower, upper))
            best = None
            best_feasible = False
            for I0 in starts:
                I_f, obj, terms, _ = self._solve_directed_from(
                    pos,
                    I0,
                    lower,
                    upper,
                    F_des_v,
                    F_scale,
                    b_dir_hat,
                    B_min_mT,
                    B_target_mT,
                    B_max_mT,
                    lambda_dir,
                    lambda_range,
                    lambda_center,
                    lambda1,
                    lambda2,
                    min(n_iter, 8) if realtime else n_iter,
                    conv_tol,
                    dir_tol_deg,
                    eps_w,
                    field_only=field_only,
                    force_zero_tol_N=force_zero_tol_N,
                )
                # 跨起点可行优先：可行解 > objective 最小（不覆盖可行解）
                if field_only:
                    force_ok = np.linalg.norm(terms["F"]) <= force_zero_tol_N
                else:
                    force_ok = (
                        np.linalg.norm(terms["F"] - F_des_v) <= conv_tol * F_scale
                    )
                direction_ok = (
                    math.degrees(
                        math.acos(np.clip(float(terms["b_hat"] @ b_dir_hat), -1.0, 1.0))
                    )
                    <= dir_tol_deg
                )
                field_ok = B_min_mT <= terms["Bmag_mT"] <= B_max_mT
                feasible = bool(force_ok and direction_ok and field_ok)
                if (
                    best is None
                    or (feasible and not best_feasible)
                    or (feasible == best_feasible and obj < best[1])
                ):
                    best = (I_f.copy(), obj, terms)
                    best_feasible = feasible
            if best is not None:
                candidates.append(best[0])

        # ---- 整数指令（箱内取整 → 联合目标 ±1 立方体穷举 → 贪心 ±1/±2） ----
        # 选择策略：可行优先——邻域内存在同时满足力/区间/方向判据的整数点时
        # 优先选其目标函数最小者；不存在时才退回联合目标最小者。
        # 全部候选批量评估（_directed_terms_batch 向量化）。
        def batch_eval(cmds_int):
            C = np.asarray(cmds_int, dtype=float)
            I_cand = C * self.current_gain
            t = self._directed_terms_batch(
                pos,
                I_cand,
                F_des_v,
                F_scale,
                b_dir_hat,
                B_min_mT,
                B_target_mT,
                B_max_mT,
            )
            objs = (
                0.5 * np.einsum("ij,ij->i", t["r_F"], t["r_F"])
                + lambda_dir * t["r_dir"] ** 2
                + lambda_range * t["r_range"] ** 2
                + lambda_center * t["r_center"] ** 2
                + lambda1 * np.abs(I_cand).sum(axis=1)
                + 0.5 * lambda2 * np.einsum("ij,ij->i", I_cand, I_cand)
            )
            if field_only:
                # 纯磁场模式：绝对零力判据（整数控锂同样必须满足，因为
                # 最终发送的就是整数指令）
                force_ok_arr = np.linalg.norm(t["F"], axis=1) <= force_zero_tol_N
            else:
                force_ok_arr = np.linalg.norm(t["r_F"], axis=1) <= conv_tol
            ok_arr = (
                force_ok_arr & t["range_ok_arr"] & (t["dir_err_deg_arr"] <= dir_tol_deg)
            )
            return objs, ok_arr

        polish_iters = 0
        best_cmd, best_obj, best_ok = None, np.inf, False

        def consider(cmds_int, objs, ok_arr):
            """可行优先：可行候选中取目标函数最小者；无可行时取目标最小者"""
            nonlocal best_cmd, best_obj, best_ok
            k = int(np.argmin(np.where(ok_arr, objs, objs + 1e18)))
            if (
                best_cmd is None
                or (ok_arr[k] and not best_ok)
                or (ok_arr[k] == best_ok and objs[k] < best_obj * (1 - 1e-12))
            ):
                best_cmd, best_obj, best_ok = (
                    np.asarray(cmds_int[k], dtype=int).copy(),
                    float(objs[k]),
                    bool(ok_arr[k]),
                )

        for I_cand in candidates:
            c0 = np.clip(np.round(I_cand / self.current_gain), lo, hi).astype(int)
            cube = c0[None, :] + DIRECTED_CUBE_OFFSETS
            valid = np.all((cube >= lo) & (cube <= hi), axis=1)
            cands = cube[valid]
            polish_iters += 1
            objs, ok_arr = batch_eval(cands)
            consider(cands, objs, ok_arr)
        if best_cmd is None:
            c0 = np.zeros(self.n_coils, dtype=int)
            objs, ok_arr = batch_eval([c0])
            polish_iters += 1
            consider([c0], objs, ok_arr)
        cmd = best_cmd
        for _ in range(2):  # 贪心 ±1/±2（批量）
            cands = []
            for j in range(self.n_coils):
                for dd in (1, -1, 2, -2):
                    v = cmd.copy()
                    v[j] = min(int(hi[j]), max(int(lo[j]), v[j] + dd))
                    if v[j] != cmd[j]:
                        cands.append(v)
            if not cands:
                break
            polish_iters += 1
            objs, ok_arr = batch_eval(cands)
            _n0 = len(cands)
            k = int(np.argmin(np.where(ok_arr, objs, objs + 1e18)))
            o = objs[k]
            ok = bool(ok_arr[k])
            if (ok and not best_ok) or (ok == best_ok and o < best_obj * (1 - 1e-12)):
                cmd, best_obj, best_ok = cands[k], float(o), ok
                best_ok = ok

        # ---- 最终回代：从实际发送电流重算 B / 方向 / 力 ----
        I_sent = cmd * self.current_gain
        fm_out = self.forward_model(pos, I_sent)
        F_act = np.atleast_1d(fm_out["F"]).astype(float)
        B_act = fm_out["B"]
        Bmag_mT = float(fm_out["B_magnitude_mT"])
        b_hat_act = B_act / max(np.linalg.norm(B_act), 1e-12)
        dir_err_deg = math.degrees(
            math.acos(np.clip(float(b_hat_act @ b_dir_hat), -1, 1))
        )
        F_err = float(np.linalg.norm(F_act - F_des_v))
        if field_only:
            # 纯磁场模式：percent 相对零力容差（1µN 容差下 F=0.5µN → 50%），
            # 直观反映残余力与容差的关系
            F_err_pct = (
                100.0 * float(np.linalg.norm(F_act)) / max(force_zero_tol_N, 1e-30)
            )
        else:
            F_err_pct = 100.0 * F_err / max(float(np.linalg.norm(F_des_v)), 1e-30)
        if field_only:
            # 纯磁场模式：最终回代的零力判据 |F| ≤ force_zero_tol_N（绝对值），
            # 不得无条件 True——否则 0.1µN 与 1mN 都会被误报为可行
            force_ok = bool(np.linalg.norm(F_act) <= force_zero_tol_N)
        else:
            force_ok = bool(
                F_err <= conv_tol * max(float(np.linalg.norm(F_des_v)), 1e-30)
            )
        field_range_ok = bool(B_min_mT <= Bmag_mT <= B_max_mT)
        direction_ok = bool(dir_err_deg <= dir_tol_deg)
        current_limit = max_cur  # = max_cmd × current_gain（实际允许幅值）
        current_bound_ok = bool(
            np.max(np.abs(cmd)) <= max_cmd
            and np.max(np.abs(I_sent)) <= current_limit + 1e-9
        )
        slew_ok = (
            True
            if cmd_prev is None
            else bool(np.max(np.abs(cmd - np.round(cp))) <= max_delta_cmd + 1e-9)
        )
        # converged 需同时满足：力（或零力）+ 磁场区间 + 方向 + 电流幅值 + slew
        # （定向模式无稀疏约束；solve_commands 的稀疏不可行已在其自身 converged
        #  逻辑中强制 False）
        converged = bool(
            force_ok
            and field_range_ok
            and direction_ok
            and current_bound_ok
            and slew_ok
        )
        at_bound = bool(
            np.any(np.isclose(cmd, lo, atol=1e-9))
            or np.any(np.isclose(cmd, hi, atol=1e-9))
        )
        f_tgt_txt = (
            "N/A (field-only)"
            if field_only
            else f"[{F_des_v[0]*1e6:+.2f}, {F_des_v[1]*1e6:+.2f}, {F_des_v[2]*1e6:+.2f}] µN"
        )
        log_lines = [
            f"Target force: {f_tgt_txt}",
            (
                f"Target B direction: {np.round(b_dir_hat, 4)}, "
                f"range {B_min_mT}~{B_max_mT} mT (target {B_target_mT})"
            ),
            f"Achieved force: [{F_act[0]*1e6:+.2f}, {F_act[1]*1e6:+.2f}, "
            f"{F_act[2]*1e6:+.2f}] µN"
            + ("" if field_only else f", error {F_err_pct:.2f}%"),
            f"Achieved B: {np.round(B_act*1e3, 4)} mT, |B| = {Bmag_mT:.3f} mT",
            (
                f"Achieved B direction: {np.round(b_hat_act, 4)}, "
                f"direction error {dir_err_deg:.2f} deg"
            ),
            f"Current [6]: [{', '.join(f'{x:+.4f}' for x in I_sent)}] A",
            f"Command [6]: [{', '.join(str(int(c)) for c in cmd)}]",
            (
                f"Field range status: {'OK' if field_range_ok else 'OUT OF RANGE'} | "
                f"Direction status: {'OK' if direction_ok else 'EXCEEDS'} | "
                f"Force status: {(f'OK (tol {force_zero_tol_N*1e6:.2f} µN)' if force_ok else f'EXCEEDS {force_zero_tol_N*1e6:.2f} µN') if field_only else ('OK' if force_ok else 'EXCEEDS 2%')} | "
                f"Current bound: {'OK' if current_bound_ok else 'VIOLATED'} | "
                f"Slew: {'OK' if slew_ok else 'VIOLATED'}"
            ),
            (
                f"Solver: {'linear-exact' if lin_exact else 'GN-fallback'} "
                f"(rank {M_rank}, cond {M_cond:.3g}) | "
                f"iterations {iters_used + polish_iters} "
                f"(GN {iters_used} + polish {polish_iters}), "
                f"time {(time.perf_counter()-t0)*1e3:.1f} ms"
            ),
            f"Converged: {converged}",
        ]
        return {
            "commands": cmd,
            "currents": I_sent,
            "achieved_force": F_act,
            "force_error": F_err,
            "force_error_percent": F_err_pct,
            "force_ok": force_ok,
            "B": B_act,
            "B_magnitude_mT": Bmag_mT,
            "B_direction": b_hat_act,
            "B_direction_error_deg": dir_err_deg,
            "field_in_range": field_range_ok,
            "field_range_ok": field_range_ok,
            "direction_ok": direction_ok,
            "current_bound_ok": current_bound_ok,
            "slew_ok": slew_ok,
            "converged": converged,
            "current_constraint_active": bool(
                (not force_ok or not field_range_ok) and at_bound
            ),
            "field_constraint_active": bool(not field_range_ok),
            "direction_constraint_active": bool(not direction_ok),
            "requested_force": F_des_v.copy(),
            "force_zero_tol_N": float(force_zero_tol_N),
            "requested_B_direction": b_dir_hat.copy(),
            "requested_B_magnitude_mT": float(B_target_mT),
            "requested_B_range_mT": (float(B_min_mT), float(B_max_mT)),
            "realtime": bool(realtime),
            "iterations": int(iters_used + polish_iters),
            "gn_iterations": int(iters_used),
            "polish_iterations": int(polish_iters),
            "linear_rank": M_rank,
            "linear_cond": M_cond,
            "elapsed_ms": (time.perf_counter() - t0) * 1e3,
            "log_lines": log_lines,
            "objective": best_obj,
        }

    def _select_force_candidate(
        self, pos, F_des, cmds, lambda1, lambda2, conv_tol=CONV_TOL
    ):
        """普通力模式的候选选择（可行优先，供三个 polish 函数统一使用）：
        第一优先 force_error ≤ conv_tol·‖F_des‖；第二优先 objective 最小；
        无可行候选时取 force_error 最小。返回 (k, obj, ferr, feasible)。"""
        C = np.asarray(cmds, dtype=np.float64)
        F = self.force_batch(pos, C * self.current_gain)
        ferr = np.linalg.norm(F - F_des[None, :], axis=1)
        F_norm = float(np.linalg.norm(F_des))
        feasible = ferr <= conv_tol * max(F_norm, 1e-30)
        obj = self._objective(pos, F_des, C * self.current_gain, lambda1, lambda2)
        if np.any(feasible):
            k = int(np.argmin(np.where(feasible, obj, np.inf)))
        else:
            k = int(np.argmin(ferr))
        return k, obj, ferr, feasible

    def _polish_cube(self, pos, F_des, cmd, lo, hi, lambda1, lambda2):
        """连续解取整后，在 ±1 指令立方体（3^6=729 候选，分块单批评估）内
        穷举精修——消除整数量化的主要损失。"""
        base = np.clip(
            np.asarray(cmd, dtype=int),
            np.asarray(lo, dtype=int),
            np.asarray(hi, dtype=int),
        )
        lo_i = np.asarray(lo, dtype=int)
        hi_i = np.asarray(hi, dtype=int)
        cands = []
        for d in itertools.product((-1, 0, 1), repeat=self.n_coils):
            v = base + np.array(d, dtype=int)
            if np.all(v >= lo_i) and np.all(v <= hi_i):
                cands.append(v)
        if not cands:
            return base
        # base 自身状态由 _select_force_candidate 统一计算（best_feas 不得硬编码
        # False——base 可能已满足力判据，否则后续比较会错误拒绝可行候选）
        _, base_obj_arr, base_ferr_arr, base_feas_arr = self._select_force_candidate(
            pos, F_des, [base], lambda1, lambda2
        )
        best = base
        best_obj = float(base_obj_arr[0])
        best_ferr = float(base_ferr_arr[0])
        best_feas = bool(base_feas_arr[0])
        C_all = np.array(cands, dtype=float)
        for s0 in range(0, len(C_all), 250):  # 分块控制内存
            C = C_all[s0 : s0 + 250]
            k, objs, ferrs, feasible = self._select_force_candidate(
                pos, F_des, C, lambda1, lambda2
            )
            # 仅当候选更优（可行优先已在 _select_force_candidate 内处理）才更新
            k_obj = float(objs[k])
            k_ferr = float(ferrs[k])
            k_feas = bool(feasible[k])
            if k_feas and not best_feas:
                accept = True
            elif not k_feas and best_feas:
                accept = False
            elif k_feas and best_feas:
                accept = k_obj < best_obj * (1 - 1e-12)
            else:
                accept = k_ferr < best_ferr * (1 - 1e-12)
            if accept:
                best, best_obj, best_ferr, best_feas = (
                    C[k].astype(int),
                    k_obj,
                    k_ferr,
                    k_feas,
                )
        return best

    def _refine_sparse(
        self, pos, F_des, cmd, lo, hi, lambda1, lambda2, max_active, sweeps=2
    ):
        """保持"最多 max_active 路非零"约束的指令域贪心精修。
        移动类型：活动通道 ±1/±2（含降到 0 的停用）；可归零的零通道 ±1 激活
        （名额满时与一个 |v|<=Δ 的活动通道归零交换）。"""
        best = np.asarray(cmd, dtype=int).copy()
        best_obj = float(
            self._objective(
                pos, F_des, (best * self.current_gain)[None, :], lambda1, lambda2
            )[0]
        )
        _, _, best_ferr, best_feas = self._select_force_candidate(
            pos, F_des, [best], lambda1, lambda2
        )

        def count_nonzero(v):
            return int(np.sum(np.asarray(v) != 0))

        def try_candidates(cand_list):
            nonlocal best, best_obj, best_ferr, best_feas
            if not cand_list:
                return False
            # 稀疏约束属于硬约束：先滤除非零路数超限的候选
            _C_all = np.array(cand_list, dtype=float)
            keep_mask = np.array([count_nonzero(c) <= max_active for c in cand_list])
            if not np.any(keep_mask):
                return False
            cands = [c for c, m in zip(cand_list, keep_mask) if m]
            k, objs, ferrs, feasible = self._select_force_candidate(
                pos, F_des, cands, lambda1, lambda2
            )
            k_obj = float(objs[k])
            k_ferr = float(ferrs[k])
            k_feas = bool(feasible[k])
            if k_feas and not best_feas:
                accept = True  # 可行 > 不可行
            elif not k_feas and best_feas:
                accept = False
            elif k_feas and best_feas:
                accept = k_obj < best_obj * (1 - 1e-12)
            else:
                accept = k_ferr < best_ferr * (1 - 1e-12)  # 均不可行比力误差
            if accept:
                best, best_obj = cands[k], k_obj
                best_ferr, best_feas = k_ferr, k_feas
                return True
            return False

        for _ in range(sweeps):
            improved = False
            actives = [i for i in range(self.n_coils) if best[i] != 0]
            zeros_zeroable = [
                i for i in range(self.n_coils) if best[i] == 0 and lo[i] <= 0 <= hi[i]
            ]
            # 1) 活动通道 ±1 / ±2（允许降到 0 停用）
            cands = []
            for j in actives:
                for d in (1, -1, 2, -2):
                    v = best.copy()
                    v[j] = int(np.clip(v[j] + d, lo[j], hi[j]))
                    if v[j] != best[j] and count_nonzero(v) <= max_active:
                        cands.append(v)
            if try_candidates(cands):
                improved = True
            # 2) 零通道激活（名额未满），或与活动通道交换（激活 + 停用）
            for j in zeros_zeroable:
                for d in (1, -1):
                    v = best.copy()
                    v[j] = int(np.clip(best[j] + d, lo[j], hi[j]))
                    if v[j] == 0:
                        continue
                    if count_nonzero(v) <= max_active:
                        if try_candidates([v]):
                            improved = True
                            break
                    else:
                        # 名额满：与任一可一帧归零（0 ∈ [lo, hi]）的活动通道交换
                        for k2 in actives:
                            if best[k2] != 0 and lo[k2] <= 0 <= hi[k2]:
                                w = v.copy()
                                w[k2] = 0
                                if count_nonzero(w) <= max_active and try_candidates(
                                    [w]
                                ):
                                    improved = True
                                    break
                        if improved:
                            break
            if not improved:
                break
        return best

    def _unipolar_best_cmd(self, pos, F_des, cp, lo, hi, lambda1, lambda2):
        """单极约束的指令域精确求解：候选集 = {单线圈 j 在其整数箱内取值，其余
        通道为 0（0 必须也在其余通道箱内）} ∪ {全零}。全部候选一次批量评估，
        选择用 _select_force_candidate（可行优先：force_error ≤2% 优先，
        其次 objective——与普通/polish 模式统一）。
        切换线圈时旧通道须经零衰减（箱约束自然实现）。
        返回 (commands(6,), active_coil_list, infeasible:bool)。"""
        best_cmd = np.zeros(self.n_coils, dtype=int)
        best_j = -1
        cands, cand_j = [], []

        # 全零候选（需所有通道箱含 0）
        if np.all(lo <= 0) and np.all(hi >= 0):
            cands.append(np.zeros(self.n_coils, dtype=float))
            cand_j.append(-1)

        # 单线圈候选
        for j in range(self.n_coils):
            # 其余通道必须能取 0，否则该候选本帧不可行
            if any(lo[i] > 0 or hi[i] < 0 for i in range(self.n_coils) if i != j):
                continue
            for c in range(int(np.ceil(lo[j])), int(np.floor(hi[j])) + 1):
                if c == 0:
                    continue  # 全零候选已含
                v = np.zeros(self.n_coils, dtype=float)
                v[j] = c
                cands.append(v)
                cand_j.append(j)

        if not cands:
            # 上一帧多路非零且都远离 0：无严格单极可行解（GUI 会先过渡归零）。
            # 防御性回退：取箱内可行的上一帧电流，标记不可行
            return (
                np.clip(np.round(cp), lo, hi).astype(int),
                list(range(self.n_coils)),
                True,
            )

        C = np.array(cands, dtype=float)
        k_sel, _, _, _ = self._select_force_candidate(pos, F_des, C, lambda1, lambda2)
        best_cmd = C[k_sel].astype(int)
        best_j = cand_j[k_sel]
        return best_cmd, ([best_j] if best_j >= 0 else []), False

    def _refine_commands(self, pos, F_des, cmd, lo, hi, lambda1, lambda2, sweeps=3):
        """指令域贪心细化：先逐路试 ±1 指令，再试成对通道 ±1 组合（跳出单路
        局部最优）。候选选择用 _select_force_candidate（可行优先：
        force_error ≤2% 优先，其次 objective 最小——防止 1.8%→2.1% 反向）。"""
        best = np.asarray(cmd, dtype=int).copy()
        _, _, best_ferr, best_feas = self._select_force_candidate(
            pos, F_des, [best], lambda1, lambda2
        )
        best_obj = float(
            self._objective(
                pos, F_des, (best * self.current_gain)[None, :], lambda1, lambda2
            )[0]
        )

        def try_candidates(cand_list):
            nonlocal best, best_obj, best_ferr, best_feas
            if not cand_list:
                return False
            k, objs, ferrs, feasible = self._select_force_candidate(
                pos, F_des, cand_list, lambda1, lambda2
            )
            k_obj = float(objs[k])
            k_ferr = float(ferrs[k])
            k_feas = bool(feasible[k])
            if k_feas and not best_feas:
                accept = True  # 可行 > 不可行
            elif not k_feas and best_feas:
                accept = False
            elif k_feas and best_feas:
                accept = k_obj < best_obj * (1 - 1e-12)
            else:
                accept = k_ferr < best_ferr * (1 - 1e-12)  # 均不可行比力误差
            if accept:
                best, best_obj = cand_list[k], k_obj
                best_ferr, best_feas = k_ferr, k_feas
                return True
            return False

        for _ in range(sweeps):
            improved = False
            # 单路 ±1 / ±2（±2 可跳出仅 ±1 到达不了的组合）
            for j in range(self.n_coils):
                cands = []
                for d in (+1, -1, +2, -2):
                    c = best.copy()
                    c[j] = int(np.clip(c[j] + d, lo[j], hi[j]))
                    if c[j] != best[j]:
                        cands.append(c)
                if try_candidates(cands):
                    improved = True
            # 成对 ±1（含一升一降，保持和电流近似不变的方向组合）
            for j in range(self.n_coils):
                for k2 in range(j + 1, self.n_coils):
                    cands = []
                    for dj in (+1, -1):
                        for dk in (+1, -1):
                            c = best.copy()
                            c[j] = int(np.clip(c[j] + dj, lo[j], hi[j]))
                            c[k2] = int(np.clip(c[k2] + dk, lo[k2], hi[k2]))
                            if np.any(c != best):
                                cands.append(c)
                    if try_candidates(cands):
                        improved = True
            if not improved:
                break
        return best

    # ---------------- 其他 ----------------
    def drag_coeff(self, viscosity_pa_s):
        """斯托克斯阻力系数 6πμr (N·s/m)"""
        return 6.0 * math.pi * viscosity_pa_s * self.bead_radius

    def drag_uN_per_mm_s(self, viscosity_mPa_s):
        """µN/(mm/s) 形式的阻力系数，便于 GUI 显示"""
        return self.drag_coeff(viscosity_mPa_s * 1e-3) * 1e3


# ================= 自测 =================


# ================= 自检（Test 1~13） =================
DEFAULT_COMSOL_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "models", "comsol_reference.json"
)


def validate_against_comsol_reference(
    solver, ref_path=DEFAULT_COMSOL_PATH, rel_tol=0.05
):
    """绝对磁场标定：与 COMSOL 参考数据逐点比较 B 矢量。

    参考 JSON 格式（缺失时本测试跳过，绝不伪造参考值）：
    {
      "cases": [
        {"coil": "+X", "current_A": 1.0,
         "point_mm": [0.0, 0.0, 0.0], "B_mT": [Bx, By, Bz]},
        ...
      ]
    }
    返回 (results, all_ok)：results 每项含 coil/point_mm/B_model_mT/
    B_comsol_mT/relative_error/ok；all_ok 为全部相对误差 ≤ rel_tol。
    注意：该测试校验磁矩整体缩放（约定/单位），解析梯度一致性测试无法替代。
    （mT·mm³ 存在相差 4π 的两种约定，见 from_json 内注释。）"""
    import json

    if not os.path.exists(ref_path):
        return [], False
    with open(ref_path, "r", encoding="utf-8") as f:
        ref = json.load(f)
    results, all_ok = [], True
    for case in ref.get("cases", []):
        j = solver.coil_names.index(case["coil"])
        I = np.zeros(solver.n_coils)
        I[j] = float(case.get("current_A", 1.0))
        p_mm = np.asarray(case["point_mm"], dtype=float)
        B_model = solver.field_at(p_mm * 1e-3, I) * 1e3  # T → mT
        ref_val = case["B_mT"]
        if np.isscalar(ref_val):
            # 幅值比较：参考只给 |B| 时使用
            B_ref_mag = float(ref_val)
            rel = float(abs(np.linalg.norm(B_model) - B_ref_mag) / B_ref_mag)
            results.append(
                {
                    "coil": case["coil"],
                    "point_mm": p_mm.tolist(),
                    "B_model_mT": float(np.linalg.norm(B_model)),
                    "B_comsol_mT": B_ref_mag,
                    "relative_error": rel,
                    "ok": rel <= rel_tol,
                }
            )
        else:
            B_ref = np.asarray(ref_val, dtype=float)
            rel = float(
                np.linalg.norm(B_model - B_ref) / max(np.linalg.norm(B_ref), 1e-30)
            )
            results.append(
                {
                    "coil": case["coil"],
                    "point_mm": p_mm.tolist(),
                    "B_model_mT": B_model.tolist(),
                    "B_comsol_mT": B_ref.tolist(),
                    "relative_error": rel,
                    "ok": rel <= rel_tol,
                }
            )
        all_ok = all_ok and results[-1]["ok"]
    return results, all_ok


if __name__ == "__main__":
    import sys
    from collections.abc import Callable

    TESTS: list[tuple[str, Callable[[], None]]] = []

    def test(name):
        def deco(fn):
            TESTS.append((name, fn))
            return fn

        return deco

    s = DipoleSolver.from_json()
    info = s.model_info

    # ---- Test 1: 模型加载 ----
    @test("Model loading")
    def t1():
        s.validate_model()
        assert s.seg_pos.shape == (N_COILS, DIPOLES_PER_COIL, 3)
        assert s.coil_names == ["+X", "+Y", "+Z", "-X", "-Y", "-Z"]
        assert abs(s.bead_moment - 5.0e-4) < 0.01e-4

    # ---- Test 2: 单线圈 1A 磁场（量级 + 诊断表） ----
    @test("Field calculation")
    def t2():
        print()
        print("  coil |   |B| (mT) | Fx/Fy/Fz @原点 (µN)")
        for j in range(N_COILS):
            Ij = np.zeros(N_COILS)
            Ij[j] = 1.0
            Bj = s.field_at(np.zeros(3), Ij)
            Fj = s.force_at(np.zeros(3), Ij)
            print(
                f"  {s.coil_names[j]:>4s} | {np.linalg.norm(Bj)*1e3:10.4f} | "
                f"{Fj[0]*1e6:+8.3f} {Fj[1]*1e6:+8.3f} {Fj[2]*1e6:+8.3f}"
            )
            assert 1e-5 < np.linalg.norm(Bj) < 1e-1  # 0.01 ~ 100 mT 量级
        I = np.zeros(N_COILS)
        I[0] = 1.0
        # 绝对换算守门：×1e-5 约定下 |B(0)| = 11.5635 mT/A（magnetic_field_validation、
        # MPC统一框架、运动demo 三处独立程序与 60 偶极子模型交叉验证一致）。
        # 若磁矩换算再被改错（如 ×1e-12/μ0，偏小 4π），此处立即失败。
        b0 = np.linalg.norm(s.field_at(np.zeros(3), I))
        assert (
            abs(b0 - 11.5635e-3) < 0.06e-3
        ), f"|B(0)|={b0*1e3:.4f} mT 偏离 11.5635 mT/A——磁矩换算约定可能错误"
        assert np.linalg.norm(s.field_at(np.zeros(3), I)) > 5e-4  # > 0.5 mT

    # ---- Test 3: 解析梯度 vs 数值梯度 ----
    @test("Gradient check")
    def t3():
        I = np.array([0.8, -1.2, 0.5, 1.6, -0.4, 1.1])
        for p in [np.zeros(3), np.array([3e-3, -2e-3, 0.0])]:
            ga, gn = s.grad_absB(p, I), s.grad_absB_numeric(p, I)
            rel = np.linalg.norm(ga - gn) / np.linalg.norm(gn)
            assert rel < 1e-5, f"梯度相对误差 {rel:.2e}"

    # ---- Test 4: 磁场线性叠加（模型假设检验，非铁磁饱和验证） ----
    @test("Superposition")
    def t4():
        rng = np.random.default_rng(7)
        for _ in range(5):
            I1 = rng.uniform(-2, 2, 6)
            I2 = rng.uniform(-2, 2, 6)
            p = np.array([rng.uniform(-5e-3, 5e-3), rng.uniform(-5e-3, 5e-3), 0.0])
            B12 = s.field_at(p, I1 + I2)
            assert np.allclose(
                B12, s.field_at(p, I1) + s.field_at(p, I2), rtol=1e-10, atol=1e-21
            )
            G12 = s._bg_batch(p, (I1 + I2)[None, :])[1][0]
            Gs = s._bg_batch(p, I1[None, :])[1][0] + s._bg_batch(p, I2[None, :])[1][0]
            assert np.allclose(G12, Gs, rtol=1e-10, atol=1e-21)

    # ---- Test 5: F(I) ≈ F(−I)（自由对齐磁矩模型的符号对称性） ----
    @test("Force symmetry")
    def t5():
        rng = np.random.default_rng(3)
        for _ in range(5):
            I = rng.uniform(-2, 2, 6)
            p = np.array([2e-3, -1e-3, 0.0])
            Fp, Fm = s.force_at(p, I), s.force_at(p, -I)
            rel = np.linalg.norm(Fp - Fm) / max(np.linalg.norm(Fp), 1e-30)
            assert rel < 1e-12, f"F(I)≠F(-I): {rel:.2e}"

    # ---- Test 6: 解析电流雅可比 vs 小步长有限差分 ----
    @test("Current Jacobian")
    def t6():
        worst = 0.0
        for I in [
            np.array([0.8, -1.2, 0.5, 1.6, -0.4, 1.1]),
            np.array([0.2, 0.1, -0.15, 0.3, -0.1, 0.05]),
        ]:
            J_a = s.current_jacobian(np.zeros(3), I)[0]
            J_n = s.current_jacobian_numeric(np.zeros(3), I, h=0.002)
            rel = np.linalg.norm(J_a - J_n) / np.linalg.norm(J_n)
            worst = max(worst, rel)
            assert rel < 1e-3, f"雅可比相对误差 {rel:.2e}"
        print(f"  (解析 vs 差分 h=0.002A 最差相对误差 {worst:.2e})")

    # ---- Test 7: 目标力逆解 ----
    # 注：整数指令量化（0.0202A/格）与首帧 ±9 指令斜率箱共同决定小目标/大目标的
    # 可达性，部分方向（如纯 y）客观无法达到 2%——此时如实 converged=False。
    # 本测试选用实测整数误差 <1.8% 的目标（探针实测）校验逆解精度与 F_actual 一致性。
    @test("Force inverse")
    def t7():
        for F_des in [
            np.array([40e-6, 0.0, 0.0]),
            np.array([60e-6, 0.0, 0.0]),
            np.array([50e-6, 30e-6, 0.0]),
        ]:
            rec = s.solve_commands(np.zeros(3), F_des, cmd_prev=0)
            err = rec["force_error_percent"]
            assert err <= 2.0, f"F_des={F_des*1e6}µN 误差 {err:.2f}%"
            F_ref = s.force_at(np.zeros(3), rec["currents"])
            assert np.linalg.norm(rec["achieved_force"] - F_ref) < 1e-18

    # ---- Test 8: ±2A 电流约束 ----
    @test("Current bound")
    def t8():
        # 10 mN 远超 ±2A 能力（实测最大 ~1.3 mN）：电流必须限幅且如实报告不收敛
        rec = s.solve_commands(np.zeros(3), np.array([1e-2, 0, 0]), cmd_prev=None)
        assert np.max(np.abs(rec["currents"])) <= MAX_CURRENT_A + 1e-9
        assert not rec["converged"]  # 远超能力，须如实报告

    # ---- Test 9: 30Hz 下 ΔI/frame 约束 ----
    @test("Slew-rate bound")
    def t9():
        I_prev = np.array([1.0, -0.5, 0.3, 0.8, -1.0, 0.2])
        rec = s.solve_commands(
            np.zeros(3), np.array([20e-6, 0, 0]), cmd_prev=I_prev / s.current_gain
        )
        d = np.abs(rec["currents"] - I_prev)
        assert (
            np.max(d) <= MAX_DELTA_A + 1e-9
        ), f"ΔI 超限: {np.max(d):.4f} > {MAX_DELTA_A:.4f} A"

    # ---- Test 10: command 整数范围与映射 ----
    @test("Command interface")
    def t10():
        for cmd, amp in [(0, 0.0), (99, 2.0), (-99, -2.0)]:
            assert abs(cmd * s.current_gain - amp) < 1e-12
        # 越界指令必须在箱约束/安全层被限幅到 ±99（±2A）
        rec = s.solve_commands(
            np.zeros(3), np.array([2e-6, 0, 0]), cmd_prev=0, max_cmd=99
        )
        assert np.max(np.abs(rec["commands"])) <= 99
        assert np.max(np.abs(rec["currents"])) <= MAX_CURRENT_A + 1e-9

    # ---- Test 11: max_active = 1（单极） ----
    @test("Sparse mode (max_active=1)")
    def t11():
        rec = s.solve_commands(
            np.zeros(3), np.array([2e-6, 0, 0]), cmd_prev=0, max_active=1
        )
        assert int(np.sum(rec["commands"] != 0)) <= 1
        assert rec["active_coils"] == [i for i in range(6) if rec["commands"][i] != 0]

    # ---- Test 12: max_active = 3（三路） ----
    @test("Sparse mode (max_active=3)")
    def t12():
        rng = np.random.default_rng(11)
        prev = np.zeros(6, dtype=int)
        for _ in range(5):
            F_des = rng.uniform(-3e-6, 3e-6, 3)
            rec = s.solve_commands(np.zeros(3), F_des, cmd_prev=prev, max_active=3)
            nz = int(np.sum(rec["commands"] != 0))
            assert nz <= 3, f"三路约束被违反: {rec['commands']}"
            assert rec["active_coils"] == [
                i for i in range(6) if rec["commands"][i] != 0
            ]
            prev = rec["commands"]

    # ---- Test 13: COMSOL 绝对磁场标定 ----
    @test("COMSOL calibration")
    def t13():
        results, _all_ok = validate_against_comsol_reference(s)
        if not results:
            print(
                f"  SKIPPED: COMSOL reference data unavailable "
                f"({DEFAULT_COMSOL_PATH})"
            )
            return
        for r in results:
            bm = r["B_model_mT"]
            bm_s = f"|B|={bm:.3f}" if np.isscalar(bm) else f"B={np.round(bm, 4)}"
            print(
                f"  {r['coil']} @{r['point_mm']}: model {bm_s} mT, "
                f"comsol={r['B_comsol_mT']} mT, rel={r['relative_error']:.3%}"
            )
            assert r["ok"], f"绝对标定超差: {r}"

    # ---- Test 14 (spec 18): 定向磁场基础求解（含零力条件） ----
    @test("Directed field basics")
    def t_directed_basic():
        rec = s.solve_force_and_field(np.zeros(3), None, B_direction=[0, 0, -1])
        assert (
            8.0 <= rec["B_magnitude_mT"] <= 12.0
        ), f"|B|={rec['B_magnitude_mT']:.2f} mT 超出 8~12"
        assert (
            rec["B_direction_error_deg"] <= 5.0
        ), f"方向误差 {rec['B_direction_error_deg']:.2f}°"
        assert rec["force_ok"], (
            f"纯磁场模式残余力过大: {rec['force_error']*1e6:.3f} µN > "
            f"{rec['force_zero_tol_N']*1e6:.3f} µN"
        )
        assert rec["converged"]

    # ---- Test 15 (spec 19): 目标力 + 定向磁场联合 ----
    @test("Joint force+field")
    def t_joint():
        # −100µN + −z 场：线性可行，应完全收敛
        rec = s.solve_force_and_field(
            np.zeros(3), np.array([0, -100e-6, 0]), B_direction=[0, 0, -1]
        )
        assert (
            rec["force_error_percent"] <= 2.0
        ), f"力误差 {rec['force_error_percent']:.2f}%"
        assert rec["field_range_ok"] and rec["direction_ok"] and rec["converged"]
        # −50µN：量化地板约 2.8%（±2 立方体穷举验证），如实报告不假装收敛
        rec2 = s.solve_force_and_field(
            np.zeros(3), np.array([0, -50e-6, 0]), B_direction=[0, 0, -1]
        )
        assert (
            rec2["force_error_percent"] <= 5.0
        ), f"力误差 {rec2['force_error_percent']:.2f}% 超出量化可达范围"
        assert rec2["field_range_ok"] and rec2["direction_ok"]

    # ---- Test 16 (spec 20): 方向翻转（四个方向，纯定向模式，含零力条件） ----
    @test("Direction flips")
    def t_dirflip():
        for bd in [[0, 0, 1], [0, 0, -1], [1, 0, 0], [0, 1, 0]]:
            rec = s.solve_force_and_field(np.zeros(3), None, B_direction=bd)
            assert (
                8.0 <= rec["B_magnitude_mT"] <= 12.0
            ), f"b̂={bd}: |B|={rec['B_magnitude_mT']:.2f}"
            assert (
                rec["B_direction_error_deg"] <= 5.0
            ), f"b̂={bd}: 方向误差 {rec['B_direction_error_deg']:.2f}°"
            assert rec["force_ok"], (
                f"b̂={bd}: 残余力 {rec['force_error']*1e6:.3f} µN > "
                f"{rec['force_zero_tol_N']*1e6:.3f} µN"
            )
            assert rec["converged"]

    # ---- Test 17 (spec 21): 整数 command 回代一致性 ----
    @test("Integer cmd recompute")
    def t_int_recompute():
        rec = s.solve_force_and_field(
            np.zeros(3), np.array([0, -100e-6, 0]), B_direction=[0, 0, -1]
        )
        fm_out = s.forward_model(np.zeros(3), rec["currents"])
        assert np.allclose(fm_out["B"], rec["B"], atol=1e-18), "B 未按发送电流重算"
        assert np.allclose(
            np.atleast_1d(fm_out["F"]), np.atleast_1d(rec["achieved_force"]), atol=1e-18
        ), "F 未按发送电流重算"
        assert abs(rec["B_magnitude_mT"] - np.linalg.norm(fm_out["B"]) * 1e3) < 1e-12

    # ---- Test 18 (spec 22): 电流边界 ----
    @test("Command bounds")
    def t_cmd_bounds():
        for F in [np.array([0, -100e-6, 0]), np.array([30e-6, -30e-6, 5e-6]), None]:
            rec = s.solve_force_and_field(np.zeros(3), F, B_direction=[0, 0, -1])
            assert np.max(np.abs(rec["commands"])) <= 99
            assert np.max(np.abs(rec["currents"])) <= MAX_CURRENT_A + 1e-9

    # ---- Test 19 (spec 23): 后续帧 slew ----
    @test("Slew (directed)")
    def t_slew_directed():
        rec0 = s.solve_force_and_field(
            np.zeros(3), np.array([0, -80e-6, 0]), B_direction=[0, 0, -1], cmd_prev=0
        )
        prev = rec0["commands"]
        rec1 = (
            s.solve_force_and_field(
                np.zeros(3),
                np.array([0, -40e-6, 0]),
                cmd_prev=prev,
                B_direction=[0, 0, -1],
                max_active=None,
            )
            if False
            else s.solve_force_and_field(
                np.zeros(3),
                np.array([0, -40e-6, 0]),
                cmd_prev=prev,
                B_direction=[0, 0, -1],
            )
        )
        d = np.abs(rec1["commands"] - prev)
        assert (
            np.max(d) <= MAX_DELTA_CMD + 1e-9
        ), f"slew 超限: {np.max(d)} > {MAX_DELTA_CMD}"

    # ---- Test 20 (spec 24): 不可行状态如实报告 ----
    @test("Infeasible honest")
    def t_infeasible():
        rec = s.solve_force_and_field(
            np.zeros(3), np.array([0, -5e-3, 0]), B_direction=[0, 0, -1]
        )
        assert not rec["converged"], "5mN 不可达却报告收敛"
        assert not rec["force_ok"]
        assert np.max(np.abs(rec["currents"])) <= MAX_CURRENT_A + 1e-9

    # ---- Test 21 (需求 13): 纯磁场零力模式 —— 防止 F 条件被绕掉的守门测试 ----
    @test("Directed field zero-force")
    def t_directed_zero_force():
        rec = s.solve_force_and_field(
            np.zeros(3), None, B_direction=[0, 0, -1], force_zero_tol_N=1e-6
        )
        assert rec["field_range_ok"]
        assert rec["direction_ok"]
        assert (
            rec["force_error"] <= 1e-6
        ), f"残余力 {rec['force_error']*1e6:.3f} µN > 1 µN"
        assert rec["force_ok"]
        assert rec["converged"]
        # force_error_percent 语义：相对零力容差（100% = 恰好在容差边界）
        assert rec["force_error_percent"] <= 100.0

    # ---- 运行与报告 ----
    print("=" * 64)
    print("180 磁偶极子解算器自检（Test 1~21）")
    print("=" * 64)
    print(f"模型名称: {info['model_name']}")
    print(f"线圈顺序: {info['coil_order']}")
    print(f"偶极子数量: {info['n_coils']} × {info['dipoles_per_coil']} = 180")
    print(f"磁珠磁矩: {s.bead_moment:.4e} A·m²")
    print(f"阻力系数 (1000 mPa·s): {s.drag_uN_per_mm_s(1000.0):.2f} µN/(mm/s)")
    print(
        f"最大允许电流: ±{MAX_CURRENT_A} A (指令 ±{CMD_MAX}, "
        f"gain={s.current_gain:.6f})"
    )
    print(f"单帧最大电流变化: ±{MAX_DELTA_A:.4f} A ({MAX_DELTA_CMD} 指令)")
    print(
        f"正则化: L1 λ1 + Tikhonov λ2（自适应 tau1={LAMBDA1_TAU}, "
        f"tau2={LAMBDA2_TAU}）"
    )
    print("-" * 64)

    n_fail = 0
    for name, fn in TESTS:
        try:
            fn()
            print(f"Test {name:<28s} PASS")
        except AssertionError as e:
            n_fail += 1
            print(f"Test {name:<28s} FAIL: {e}")
        except Exception as e:  # noqa
            n_fail += 1
            print(f"Test {name:<28s} FAIL: {type(e).__name__}: {e}")
    print("-" * 64)
    print(
        f"通过 {len(TESTS) - n_fail}/{len(TESTS)}"
        + ("  ✓" if n_fail == 0 else "  ✗ 存在失败项")
    )
    sys.exit(1 if n_fail else 0)
