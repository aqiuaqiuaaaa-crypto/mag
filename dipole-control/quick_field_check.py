"""
快速验证：输入电磁铁电流矩阵 + 位置 → 输出磁场 B 与梯度
========================================================
复用 dipole_solver.DipoleSolver（180 偶极子标定模型）。

- 位置输入单位 mm（与模型 JSON 一致），内部转 m。
- 电流输入 (6,) A，线圈顺序 a0~a5 = +X, +Y, +Z, -X, -Y, -Z。
- 输出：
    B        (3,)   合成磁场 [T]，打印为 mT
    G        (3,3)  磁场梯度张量 ∂B_k/∂x_l [T/m]
    grad|B|  (3,)   ∇|B| = BᵀG/|B| [T/m]（磁珠受力 F = m_b·∇|B| 的来源）
- 验证：解析 G 与 field_at 的中心差分逐元素对比（应 <1e-6 相对误差）；
  电流线性 B(αI) = α·B(I)。

用法：
    py quick_field_check.py                     # 运行内置测试用例
    或 import quick_field_check; quick_field_check.evaluate(pos_mm, currents_A)
"""

import numpy as np
from dipole_solver import DipoleSolver

EPS_POS_M = 1e-9  # 位置扰动步长 [m]


def evaluate(solver, pos_mm, currents_A, delta=EPS_POS_M):
    """pos_mm: (3,) 场点 [mm]；currents_A: (6,) 各线圈电流 [A]。
    返回 dict：B (T)、G (T/m)、grad_absB (T/m)、absB (T)、rel_err_diff（差分验证）。"""
    pos = np.asarray(pos_mm, dtype=np.float64) * 1e-3  # mm → m
    I = np.asarray(currents_A, dtype=np.float64)
    if I.shape != (solver.n_coils,):
        raise ValueError(f"currents 必须为 ({solver.n_coils},)，实际 {I.shape}")

    Bc, Gc = solver._field_grad_coils(pos)  # (6,3), (6,3,3) 单位电流
    B = Bc.T @ I  # (3,)
    G = np.einsum("j,jkl->kl", I, Gc)  # G[k,l] = ∂B_k/∂x_l
    absB = np.linalg.norm(B)
    grad_absB = (B @ G) / max(absB, solver.b_eps)

    # 中心差分交叉验证解析梯度张量
    G_num = np.zeros((3, 3))
    for k in range(3):
        p1, p2 = pos.copy(), pos.copy()
        p1[k] += delta
        p2[k] -= delta
        G_num[:, k] = (solver.field_at(p1, I) - solver.field_at(p2, I)) / (2 * delta)
    denom = max(np.linalg.norm(G), 1e-30)
    rel_err = np.linalg.norm(G - G_num) / denom

    G_fro = np.linalg.norm(G)  # Frobenius 范数
    G_eig = np.linalg.eigvalsh(G)  # 实对称，迹恒为 0
    grad_absB_norm = np.linalg.norm(grad_absB)
    eB = B / max(absB, solver.b_eps)  # B 方向单位矢量
    grad_along_B = float(eB @ grad_absB)  # ∂|B|/∂ê_B
    return {
        "B": B,
        "G": G,
        "grad_absB": grad_absB,
        "absB": absB,
        "G_fro": G_fro,
        "G_eig": G_eig,
        "grad_absB_norm": grad_absB_norm,
        "grad_along_B": grad_along_B,
        "G_num": G_num,
        "rel_err_diff": rel_err,
    }


def print_result(name, r):
    print(f"\n--- {name} ---")
    print(
        f"B  = [{', '.join(f'{v*1e3:+9.4f}' for v in r['B'])}] mT,  |B| = {r['absB']*1e3:.4f} mT"
    )
    print("G = ∂B/∂x [mT/m]（行=B 分量 k，列=空间方向 l=x,y,z）:")
    for row in r["G"]:
        print("    [" + ", ".join(f"{v*1e3:+10.3f}" for v in row) + "]")
    print(
        f"|G|_F = {r['G_fro']*1e3:.4f} mT/m,  "
        f"特征值 λ = [{', '.join(f'{v*1e3:+.3f}' for v in r['G_eig'])}] mT/m"
    )
    print(
        f"∇|B| = [{', '.join(f'{v:+.4f}' for v in r['grad_absB']*1e3)}] mT/m,  "
        f"|∇|B|| = {r['grad_absB_norm']*1e3:.4f} mT/m"
    )
    print(f"沿 B 方向主梯度 ∂|B|/∂ê_B = {r['grad_along_B']*1e3:+.4f} mT/m")
    print(
        f"解析 G vs 中心差分相对误差: {r['rel_err_diff']:.2e}"
        f"  {'✓' if r['rel_err_diff'] < 1e-5 else '✗ 超差'}"
    )


def _grad_absB_numeric(solver, pos_mm, currents_A, delta_mm=1e-3):
    """∇|B| 的 mm 级差分（粗粒度物理合理性对照，不要求与解析一致到 1e-6）"""
    pos = np.asarray(pos_mm, float)
    I = np.asarray(currents_A, float)
    g = np.zeros(3)
    for k in range(3):
        p1, p2 = pos.copy(), pos.copy()
        p1[k] += delta_mm
        p2[k] -= delta_mm
        g[k] = (
            np.linalg.norm(evaluate(solver, p1, I)["B"])
            - np.linalg.norm(evaluate(solver, p2, I)["B"])
        ) / (2 * delta_mm * 1e-3)
    return g


def main():
    s = DipoleSolver.from_json()
    print(f"模型: {s.model_info['model_name']}  " f"线圈顺序: {s.coil_names}")

    # 用例 1：单线圈 +X 1A @ 原点（与自测/标定值 ~11.6 mT 对照）
    r = evaluate(s, [0, 0, 0], [1, 0, 0, 0, 0, 0])
    print_result("+X 1A @ (0,0,0) mm", r)

    # 用例 2：一般电流组合 @ 偏置位置
    I2 = [0.8, -1.2, 0.5, 1.6, -0.4, 1.1]
    r2 = evaluate(s, [3.0, -2.0, 0.0], I2)
    print_result(f"I={I2} @ (3,-2,0) mm", r2)

    # 用例 3：∇|B| 与 mm 级差分对照（量级/符号合理性）
    g_num = _grad_absB_numeric(s, [3.0, -2.0, 0.0], I2)
    rel = np.linalg.norm(r2["grad_absB"] - g_num) / np.linalg.norm(g_num)
    print(
        f"\n∇|B| 解析 vs 1mm 差分: 解析"
        f"[{', '.join(f'{v:+.2f}' for v in r2['grad_absB']*1e3)}] mT/m, "
        f"差分[{', '.join(f'{v:+.2f}' for v in g_num*1e3)}] mT/m, "
        f"相对偏差 {rel:.2e}（mm 级差分含曲率，10% 内合理）"
    )

    # 用例 4：电流线性 B(αI) = α·B(I)
    a = 2.7
    B1 = evaluate(s, [3.0, -2.0, 0.0], I2)["B"]
    B2 = evaluate(s, [3.0, -2.0, 0.0], (a * np.array(I2)))["B"]
    lin_err = np.linalg.norm(B2 - a * B1) / max(np.linalg.norm(B1), 1e-30)
    print(
        f"电流线性校验 B({a}·I) = {a}·B(I): 相对误差 {lin_err:.2e}"
        f"  {'✓' if lin_err < 1e-12 else '✗'}"
    )

    # 用例 5：对称抵消陷阱提示——等电流对称组合 |B| 应接近 0
    r5 = evaluate(s, [0, 0, 0], [1, 1, 1, 1, 1, 1])
    print(
        f"对称等电流 (1,1,1,1,1,1) @ 原点: |B| = {r5['absB']*1e6:.3f} µT"
        f"（场抵消，∇|B| 病态——逆解需非抵消种子）"
    )

    print("\n快速验证完成 ✓")


if __name__ == "__main__":
    main()
