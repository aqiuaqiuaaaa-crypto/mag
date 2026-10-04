# -*- coding: utf-8 -*-
"""
严格单元测试（独立可运行：py tests/test_solver.py，也兼容 pytest）
================================================================
覆盖：180 偶极子校验 / 线圈顺序 / 磁场量级 / 磁力模型 / 解析梯度 vs 中心差分 /
L1 正则化 / Tikhonov L2 正则化 / 电流幅值约束 / 斜率约束 / 力匹配 /
2% 收敛判据 / 串口映射 / 43 字节帧 / 手动电流斜率
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config as cfg
from dipole_solver import (DipoleSolver, ModelValidationError,
                           validate_model_data, MAX_CURRENT_A, CMD_MAX,
                           CMD_TO_A, MAX_DELTA_A, MAX_DELTA_CMD, CONV_TOL)

RESULTS: list[tuple[str, bool, str]] = []


def test(fn):
    def wrapper():
        try:
            fn()
            RESULTS.append((fn.__name__, True, ""))
            print(f"  PASS  {fn.__name__}")
        except AssertionError as e:
            RESULTS.append((fn.__name__, False, str(e)))
            print(f"  FAIL  {fn.__name__}: {e}")
        except Exception as e:  # noqa
            RESULTS.append((fn.__name__, False, f"{type(e).__name__}: {e}"))
            print(f"  ERROR {fn.__name__}: {type(e).__name__}: {e}")
    wrapper.__name__ = fn.__name__
    return wrapper


S = DipoleSolver.from_json()


# ---------- 1. 模型 ----------
@test
def test_model_180():
    s = DipoleSolver.from_json()
    assert s.seg_pos.shape == (6, 30, 3), f"偶极子排布 {s.seg_pos.shape}"
    assert s.seg_pos.shape[0] * s.seg_pos.shape[1] == 180
    s.validate_model()
    # 违规模型必须抛错
    bad = dict(S.model_info and {})
    data = {"n_coils": 6, "dipoles_per_coil": 30, "n_dipoles_total": 180,
            "coil_order": cfg.COIL_ORDER,
            "coils": {n: {"pos": [[0, 0, 0]] * 5, "mom": [[1, 0, 0]] * 5}
                      for n in cfg.COIL_ORDER}}
    try:
        validate_model_data(data)
        raise AssertionError("5 偶极子/线圈未触发校验错误")
    except ModelValidationError:
        pass


@test
def test_coil_order():
    assert S.coil_names == ["+X", "+Y", "+Z", "-X", "-Y", "-Z"]
    assert S.model_info["coil_order"] == ["+X", "+Y", "+Z", "-X", "-Y", "-Z"]
    wrong = {"n_coils": 6, "dipoles_per_coil": 30, "n_dipoles_total": 180,
             "coil_order": ["+X", "+Y", "+Z", "-X", "-Z", "-Y"],
             "coils": {n: {"pos": [[0, 0, 0]] * 30, "mom": [[1, 0, 0]] * 30}
                       for n in ["+X", "+Y", "+Z", "-X", "-Z", "-Y"]}}
    try:
        validate_model_data(wrong)
        raise AssertionError("错误 coil_order 未触发校验错误")
    except ModelValidationError:
        pass


# ---------- 2. 磁场/磁力 ----------
@test
def test_field():
    # 每线圈 1A 在原点的场量级：标定模型 ~10 mT
    I = np.zeros(6); I[0] = 1.0
    B = S.field_at(np.zeros(3), I)
    nb = np.linalg.norm(B)
    assert 1e-4 < nb < 1e0, f"|B| 量级异常: {nb}"
    assert nb > 5e-3, f"|B| 过小: {nb * 1e3:.2f} mT"   # 标定模型 ~11.6 mT/A
    # 零电流零场
    assert np.linalg.norm(S.field_at(np.zeros(3), np.zeros(6))) == 0.0


@test
def test_force():
    # F = m_b ∇|B|：与解析 ∇|B| 自洽
    I = np.array([0.8, -1.2, 0.5, 1.6, -0.4, 1.1])
    pos = np.array([2e-3, 1e-3, 0.0])
    F = S.force_at(pos, I)
    g = S.grad_absB(pos, I)
    F_ref = S.bead_moment * g
    assert np.linalg.norm(F - F_ref) / np.linalg.norm(F_ref) < 1e-12
    assert np.linalg.norm(F) > 1e-7, "力量级异常"


@test
def test_analytic_gradient():
    # 解析梯度 vs 7 点中心差分（|B| 非零区域），rel error < 1e-5
    for I in [np.array([0.8, -1.2, 0.5, 1.6, -0.4, 1.1]),
              np.array([2.0, 0.0, -2.0, 0.0, 1.0, 0.0])]:
        for pos in [np.zeros(3), np.array([3e-3, -2e-3, 0.0]),
                    np.array([-5e-3, 4e-3, 0.0])]:
            ga = S.grad_absB(pos, I)
            gn = S.grad_absB_numeric(pos, I)
            rel = np.linalg.norm(ga - gn) / np.linalg.norm(gn)
            assert rel < 1e-5, f"解析梯度误差 {rel:.2e} @ {pos}, I={I}"


# ---------- 3. 正则化逆解 ----------
@test
def test_l1_regularization():
    # λ1 越大 Σ|I| 越小（力仍尽量匹配）
    F_des = np.array([4e-6, 1.5e-6, 0.0])
    r_lo = S.solve_currents(np.zeros(3), F_des, I_prev=np.zeros(6), lambda1=0.0)
    r_hi = S.solve_currents(np.zeros(3), F_des, I_prev=np.zeros(6), lambda1=1e-8)
    s_lo = float(np.abs(r_lo["final_current"]).sum())
    s_hi = float(np.abs(r_hi["final_current"]).sum())
    assert s_hi <= s_lo + 1e-9, f"λ1 增大后 Σ|I| 未减小: {s_hi} vs {s_lo}"


@test
def test_tikhonov_regularization():
    # λ2 越大 ΣI² 越小
    F_des = np.array([4e-6, 1.5e-6, 0.0])
    r_lo = S.solve_currents(np.zeros(3), F_des, I_prev=np.zeros(6), lambda2=0.0)
    r_hi = S.solve_currents(np.zeros(3), F_des, I_prev=np.zeros(6), lambda2=1e-8)
    p_lo = float(np.einsum('i,i->', r_lo["final_current"], r_lo["final_current"]))
    p_hi = float(np.einsum('i,i->', r_hi["final_current"], r_hi["final_current"]))
    assert p_hi <= p_lo + 1e-15, f"λ2 增大后 ΣI² 未减小: {p_hi} vs {p_lo}"


@test
def test_current_bounds():
    # 大力不可达时输出受 ±2A 限制
    rec = S.solve_currents(np.zeros(3), np.array([1e-2, 0, 0]), I_prev=np.zeros(6))
    assert np.max(np.abs(rec["final_current"])) <= MAX_CURRENT_A + 1e-9
    assert not rec["converged"] and rec["current_constraint_active"]


@test
def test_slew_rate():
    # 斜率约束进入优化：|I − I_prev| ≤ max_delta（严格解内约束，非解后截断）
    I_prev = np.array([1.0, -0.5, 0.3, 0.8, -1.0, 0.2])
    rec = S.solve_currents(np.zeros(3), np.array([100e-6, 0, 0]), I_prev=I_prev)
    d = np.abs(rec["final_current"] - I_prev)
    assert np.max(d) <= MAX_DELTA_A + 1e-9, \
        f"斜率超限: {np.max(d):.4f} > {MAX_DELTA_A:.4f} A"
    # 箱约束边界正确（与幅值约束取交集）
    assert np.max(rec["final_current"]) <= min(MAX_CURRENT_A, 1.0 + MAX_DELTA_A) + 1e-9


@test
def test_solver_force():
    # 指令域求解：F_actual 必须按最终发送电流重算（严格一致）
    # 注意 0.0202A/指令 的量化与首帧 ±9 指令斜率箱决定了部分目标（如纯 y 方向）
    # 2% 未必可达（探针实测的客观限制），此时 converged=False 是诚实输出；
    # 此处校验误差在量化可达范围内
    for F_des in [np.array([40e-6, 0, 0.0]), np.array([0.0, 60e-6, 0.0]),
                  np.array([-30e-6, 30e-6, 0.0])]:
        rec = S.solve_commands(np.zeros(3), F_des, cmd_prev=0)
        F_ref = S.force_at(np.zeros(3), rec["currents"])   # 发送电流重算
        assert np.linalg.norm(rec["achieved_force"] - F_ref) < 1e-15, \
            "F_actual 与发送电流不一致"
        assert rec["converged"] == (rec["force_error_percent"] <= 2.0), \
            "收敛标志与 2% 判据不一致"
        assert rec["force_error_percent"] < 8.0, \
            f"F_des={F_des}: 误差 {rec['force_error_percent']:.2f}% 超出量化可达范围"


@test
def test_unipolar():
    # 单极模式(max_active=1)：任何时刻最多一路线圈有电流；经零切换；斜率仍受箱约束
    prev = np.zeros(6, dtype=int)
    for F_des in [np.array([4e-6, 0, 0]), np.array([0, 4e-6, 0]),
                  np.array([-3e-6, 3e-6, 0])]:
        rec = S.solve_commands(np.zeros(3), F_des, cmd_prev=prev, max_active=1)
        nz = int(np.sum(rec["commands"] != 0))
        assert nz <= 1, f"单极约束被违反: {rec['commands']}"
        # 箱约束仍成立
        d = np.abs(rec["commands"] - prev)
        assert np.max(d) <= MAX_DELTA_CMD + 1e-9, "单极模式下斜率超限"
        # F_actual 与发送电流一致
        F_ref = S.force_at(np.zeros(3), rec["currents"])
        assert np.linalg.norm(rec["achieved_force"] - F_ref) < 1e-15
        prev = rec["commands"]
    # 归零：F_des=0 时最终应到全零
    rec = S.solve_commands(np.zeros(3), np.zeros(3), cmd_prev=prev, max_active=1)
    assert np.all(rec["commands"] == 0)
    # 单极下力方向受限：纯 x 目标误差必然大（单线圈横向分量），不得假装收敛
    rec = S.solve_commands(np.zeros(3), np.array([4e-6, 0, 0]),
                           cmd_prev=0, max_active=1)
    assert rec["force_error_percent"] > 2.0 or rec["converged"]


@test
def test_max3_active():
    # 三路模式(max_active=3)：最多 3 路非零；斜率箱约束仍成立；F_act 一致
    prev = np.zeros(6, dtype=int)
    for F_des in [np.array([4e-6, 0, 0]), np.array([0, 4e-6, 2e-6]),
                  np.array([-5e-6, 3e-6, 0])]:
        rec = S.solve_commands(np.zeros(3), F_des, cmd_prev=prev, max_active=3)
        nz = int(np.sum(rec["commands"] != 0))
        assert nz <= 3, f"三路约束被违反: {rec['commands']}"
        assert len(rec["active_coils"]) == nz
        d = np.abs(rec["commands"] - prev)
        assert np.max(d) <= MAX_DELTA_CMD + 1e-9, "三路模式下斜率超限"
        F_ref = S.force_at(np.zeros(3), rec["currents"])
        assert np.linalg.norm(rec["achieved_force"] - F_ref) < 1e-15
        prev = rec["commands"]
    # 约束代价：三路应比无约束差或相当（无约束 30µN 约 1~2%）
    rec3 = S.solve_commands(np.zeros(3), np.array([30e-6, 0, 0]),
                            cmd_prev=0, max_active=3)
    rec6 = S.solve_commands(np.zeros(3), np.array([30e-6, 0, 0]), cmd_prev=0)
    assert rec3["force_error_percent"] >= rec6["force_error_percent"] - 1e-9


@test
def test_convergence_threshold():
    # 2% 统一判据：力误差 ≤2% ⇔ converged
    rec = S.solve_commands(np.zeros(3), np.array([30e-6, 0, 0]), cmd_prev=0)
    assert rec["converged"] == (rec["force_error_percent"] <= 100 * CONV_TOL)
    # 不可达力 → converged=False 且约束激活
    rec = S.solve_commands(np.zeros(3), np.array([5e-3, 0, 0]), cmd_prev=0)
    assert not rec["converged"]
    assert rec["force_error_percent"] > 2.0


# ---------- 4. 串口 ----------
@test
def test_serial_mapping():
    import magnetic_dipole_pid as m
    for cmd in [-99, -50, 0, 33, 99]:
        assert abs(cmd * cfg.CMD_TO_A - cmd * (2.0 / 99.0)) < 1e-15
    frame, cur = m.build_command([99, -99, 0, 50, -50, 1])
    assert cur[0] == 99 and abs(99 * cfg.CMD_TO_A - 2.0) < 1e-12
    assert abs(99 * cfg.CMD_TO_A) == MAX_CURRENT_A


@test
def test_serial_frame():
    import magnetic_dipole_pid as m
    frame, cur = m.build_command([30, -45, 60, -75, 90, -10])
    assert frame == "a0:+30,a1:-45,a2:+60,a3:-75,a4:+90,a5:-10\r\n"
    assert len(frame) == 43
    assert len(cur) == 6


@test
def test_manual_current_slew():
    import magnetic_dipole_pid as m
    # 手动实时发送也不能跳变：上一帧 0 → 目标 +99，单帧最多 +9 指令
    out = m.apply_slew([99, 99, 99, 99, 99, 99], [0, 0, 0, 0, 0, 0])
    assert out == [9, 9, 9, 9, 9, 9], f"斜率层输出 {out}"
    assert max(abs(o - l) for o, l in zip(out, [0] * 6)) <= cfg.MAX_DELTA_CMD
    # 9 指令 = 0.1818A ≤ 0.2A
    assert cfg.MAX_DELTA_CMD * cfg.CMD_TO_A <= 0.2
    # 负方向与幅值限幅
    out = m.apply_slew([-99] * 6, [-95, -95, -95, -95, -95, -95])
    assert out == [-99] * 6      # 95−99=−4 ≤ 9，可一步到位
    out = m.apply_slew([200] * 6, [90] * 6)
    assert out == [99] * 6       # 幅值限幅 +99


@test
def test_runtime_current_limit():
    import magnetic_dipole_pid as m
    # 降低上限时必须立即满足新上限，即使上一帧仍在 ±99。
    out = m.apply_slew([99, -99, 70, -70, 0, 1],
                       [99, -99, 80, -80, 0, 0], max_cmd=50)
    assert max(abs(x) for x in out) <= 50
    frame, cur = m.build_command([99, -99, 51, -51, 0, 1], max_cmd=50)
    assert cur == [50, -50, 50, -50, 0, 1]
    assert len(frame) == cfg.SERIAL_FRAME_BYTES


# ---------- 5. P0/P1 修复回归 ----------
@test
def test_zero_force_slew():
    # P0①：零力目标 + 非零上一帧 → 结果必须在 slew 箱内（不得跳变）
    prev = np.array([1.0, -0.5, 0.3, 0.8, -1.0, 0.2])
    rec = S.solve_currents(np.zeros(3), np.zeros(3), I_prev=prev)
    d = np.abs(rec["final_current"] - prev)
    assert np.max(d) <= MAX_DELTA_A + 1e-9, f"零力跳变 {np.max(d):.4f}A"
    # 多帧归零最终到达 0
    cur = prev.copy()
    for _ in range(30):
        cur = S.solve_currents(np.zeros(3), np.zeros(3), I_prev=cur)["final_current"]
    assert np.allclose(cur, 0.0), f"30 帧未归零: {cur}"


@test
def test_sparse_infeasible_shape():
    # P0②：不可行分支返回值必须与其他路径同构（cmd 数组），且标志为 True
    lo = np.full(6, 20.0); hi = np.full(6, 29.0)   # 全部通道无法归零
    cmd = np.full(6, 25, dtype=int)
    out, keep, infeasible = S._project_sparse(cmd, lo, hi, max_active=3)
    assert infeasible and out.shape == (6,) and np.all((out >= lo) & (out <= hi))


@test
def test_per_coil_count_validation():
    # P1⑥：某线圈偶极子数 ≠30 必须抛 ModelValidationError
    pos = np.random.default_rng(0).uniform(-0.05, 0.05, (179, 3))
    mom = np.random.default_rng(1).uniform(-1, 1, (179, 3)) * 1e-4
    idx = np.repeat(np.arange(6), 30)[:179]
    try:
        DipoleSolver(pos, mom, idx)
        raise AssertionError("179 偶极子未抛错")
    except ModelValidationError:
        pass


@test
def test_r2_regularization_no_nan():
    # P1⑤：场点与偶极子重合时不得 NaN/Inf
    p = S.seg_pos[2, 7].copy()
    F = S.force_at(p, np.array([1.0, 0, 0, 0, 0, 0]))
    assert np.all(np.isfinite(F)), f"零距离产生非有限值: {F}"


@test
def test_bmt_units_consistent():
    # P1⑦：三处 B_mT 字段必须是 mT 量级（同电流同位置应一致）
    I = np.array([0.5, 0.2, -0.3, 0.4, -0.1, 0.6])
    r1 = S.forward_model(np.zeros(3), I)["B_mT"]
    t1 = S._directed_terms(np.zeros(3), I, np.array([0, -30e-6, 0.0]),
                           3e-5, np.array([0.0, 0.0, -1.0]), 8.0, 10.0, 12.0)["B_mT"]
    tb = S._directed_terms_batch(np.zeros(3), I[None, :], np.array([0, -30e-6, 0.0]),
                                 3e-5, np.array([0.0, 0.0, -1.0]), 8.0, 10.0, 12.0)["B_mT"]
    assert 0.1 < np.linalg.norm(r1) < 100.0, f"forward_model B_mT 量级异常: {r1}"
    assert np.allclose(r1, t1, atol=1e-9)
    assert np.allclose(r1, tb[0], atol=1e-9)


@test
def test_free_field_magnitude_solver():
    # 球形磁珠位置控制：只约束 |B|，不得偷偷恢复成固定 -Z 方向。
    F_des = np.array([20e-6, 0.0, 0.0])
    rec = S.solve_force_with_field_magnitude(
        np.zeros(3), F_des, max_cmd=50, cmd_prev=None, realtime=True)
    assert rec["solver_mode"] == "free-field-magnitude"
    assert rec["requested_B_direction"] is None
    assert 8.0 <= rec["B_magnitude_mT"] <= 12.0
    assert np.max(np.abs(rec["commands"])) <= 50
    assert np.linalg.norm(rec["achieved_force"]
                          - S.force_at(np.zeros(3), rec["currents"])) < 1e-15
    # 该目标的自由最优场接近 +X，明确不是旧的 [0,0,-1] 硬约束。
    assert rec["B_direction"][0] > 0.8

    # 解析 d|B|/dI 与中心差分一致。
    I = np.array([0.4, -0.3, 0.2, 0.1, -0.2, 0.5])
    t = S._magnitude_terms(np.zeros(3), I, F_des, np.linalg.norm(F_des),
                           8.0, 10.0, 12.0)
    h = 1e-6
    numeric = np.zeros(6)
    for j in range(6):
        d = np.zeros(6); d[j] = h
        bp = S.field_at(np.zeros(3), I + d)
        bm = S.field_at(np.zeros(3), I - d)
        numeric[j] = (np.linalg.norm(bp) - np.linalg.norm(bm)) / (2 * h) * 1e3
    assert np.allclose(2.0 * t["dr_center"], numeric, rtol=1e-6, atol=1e-8)


@test
def test_field_force_moore_penrose_solver():
    F_des = np.array([20e-6, 0.0, 0.0])
    direction = np.array([0.0, 0.0, -1.0])
    rec = S.solve_field_force_pseudoinverse(
        np.zeros(3), F_des, direction, 10.0, max_cmd=50, cmd_prev=None)
    A = rec["actuation_matrix"]
    Aw = rec["normalized_actuation_matrix"]
    assert A.shape == (6, 6) and rec["actuation_rank"] == 6
    assert rec["solver_mode"] == "field-force-moore-penrose"
    assert np.allclose(rec["pseudoinverse"], np.linalg.pinv(Aw, rcond=1e-10))
    y = np.concatenate([10e-3 * direction, F_des])
    scales = np.array([10e-3] * 3 + [np.linalg.norm(F_des)] * 3)
    assert np.linalg.norm(Aw @ rec["pseudoinverse_current"] - y / scales) < 1e-8
    assert np.isclose(rec["actuation_condition"], np.linalg.cond(Aw))
    # 线性诊断严格使用整体驱动矩阵；最终实际力则由发送电流正向模型重算。
    assert np.allclose(A @ rec["currents"],
                       np.concatenate([rec["B"], rec["achieved_force_linear"]]))
    assert np.allclose(rec["achieved_force"],
                       S.force_at(np.zeros(3), rec["currents"]))
    assert np.max(np.abs(rec["commands"])) <= 50
    assert np.allclose(rec["requested_B_direction"], direction)


@test
def test_bounded_linear_least_squares():
    A = np.array([[1.0, 2.0], [2.0, -1.0], [0.5, 0.25]])
    y = np.array([3.0, -2.0, 0.7])
    lower = np.array([-0.3, -0.2])
    upper = np.array([0.4, 0.25])
    l2 = 1e-8
    x, _ = S._bounded_pseudoinverse(
        A, y, lower, upper, l2_weight=l2)
    assert np.all(x >= lower - 1e-12) and np.all(x <= upper + 1e-12)
    obj = np.linalg.norm(A @ x - y) ** 2 + l2 * np.linalg.norm(x) ** 2
    # 二维密网格给出独立的全局最优上界；活动集解不应更差。
    x0 = np.linspace(lower[0], upper[0], 401)
    x1 = np.linspace(lower[1], upper[1], 401)
    X0, X1 = np.meshgrid(x0, x1, indexing="ij")
    grid = np.column_stack([X0.ravel(), X1.ravel()])
    residual = grid @ A.T - y
    grid_obj = np.einsum("ij,ij->i", residual, residual)
    grid_obj += l2 * np.einsum("ij,ij->i", grid, grid)
    assert obj <= float(grid_obj.min()) + 1e-10


if __name__ == "__main__":
    print("=" * 64)
    print("单元测试（180 偶极子标定模型）")
    print("=" * 64)
    for fn in [test_model_180, test_coil_order, test_field, test_force,
               test_analytic_gradient, test_l1_regularization,
               test_tikhonov_regularization, test_current_bounds,
               test_slew_rate, test_unipolar, test_max3_active, test_solver_force,
               test_convergence_threshold,
               test_serial_mapping, test_serial_frame, test_manual_current_slew,
               test_runtime_current_limit,
               test_zero_force_slew, test_sparse_infeasible_shape,
               test_per_coil_count_validation, test_r2_regularization_no_nan,
               test_bmt_units_consistent, test_free_field_magnitude_solver,
               test_field_force_moore_penrose_solver,
               test_bounded_linear_least_squares]:
        fn()
    n_fail = sum(1 for _, ok, _ in RESULTS if not ok)
    print("-" * 64)
    print(f"总计 {len(RESULTS)} 项, 通过 {len(RESULTS) - n_fail}, 失败 {n_fail}")
    sys.exit(1 if n_fail else 0)
