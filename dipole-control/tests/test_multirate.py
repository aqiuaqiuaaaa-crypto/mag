# -*- coding: utf-8 -*-
"""
多速率控制架构单元测试（独立运行：py tests/test_multirate.py）
========================================================================
覆盖：视觉/Kalman/MPC/MDM/电流执行频率、PWM 常量、电流幅值、指令斜率、
RL 线圈模型方程与稳态、COMSOL 标定保持、力回代、视觉丢帧、求解慢不阻塞视觉。
"""
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config as cfg
from dipole_solver import DipoleSolver
from estimators import KalmanFilter2D
from mpc import ForceMPC
from multirate import SharedState, ControlWorker, CurrentExecutor

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


# ---- Test 1: 视觉模块 ----
@test
def test_vision():
    import magnetic_dipole_pid as m
    frame = np.full((1080, 1920, 3), 200, dtype=np.uint8)
    cv2 = __import__("cv2")
    frame = cv2.circle(frame, (1200, 400), 12, (10, 10, 10), -1)   # 暗磁珠
    vp = {"mode": "GRAY", "thresh": 100, "invert": True, "h_lo": 0, "h_hi": 179,
          "s_lo": 0, "v_lo": 0, "morph_k": 3, "morph_open": True,
          "morph_close": False, "a_min": 50, "a_max": 50000, "circ_min": 0.0,
          "r_min": 2, "r_max": 100}
    cx, cy, area, _ = m.detect_bead(frame, vp)
    assert cx is not None and abs(cx - 1200) < 3 and abs(cy - 400) < 3
    assert 300 < area < 700    # r=12 → 面积 ~452


# ---- Test 2: Kalman 30Hz ----
@test
def test_kalman_30hz():
    kf = KalmanFilter2D(cfg.KALMAN_Q_POS, cfg.KALMAN_Q_VEL, cfg.KALMAN_R)
    dt = 1.0 / cfg.KALMAN_HZ
    ts = []
    for k in range(60):
        t0 = time.perf_counter()
        p, v = kf.step(dt, (k * 0.03, 0.0))
        ts.append(time.perf_counter() - t0)
    assert len(ts) == 60
    assert max(ts) < dt, "Kalman 单步耗时超过 30Hz 周期"
    # 跟踪匀速运动
    assert abs(kf.x[2] - 0.03 / dt) < 0.2 * 0.03 / dt


# ---- Test 3/4: MPC 与 MDM 10Hz（工作线程实测频率） ----
@test
def test_mpc_mdm_10hz():
    shared = SharedState()
    worker = ControlWorker(shared, S)
    shared.set_kalman(np.zeros(2), np.zeros(2), time.time())
    shared.set_reference([np.zeros(2), np.array([1.0, 0.0]),
                          np.array([2.0, 0.0]), np.array([3.0, 0.0])], 1.0,
                         time.time())
    shared.set_params({"fmax": cfg.MPC_FMAX_UN, "max_active": 6,
                       "mpc_on": True, "fz_lift": 0.0,
                       "eso_d": np.zeros(2), "viscosity_mPas": 1000.0,
                       "bead_radius_m": 0.5e-3}, time.time())
    worker.start()
    time.sleep(1.2)
    shared.stop()
    worker.join(timeout=2.0)
    seq = shared._I_target["seq"]
    hz = seq / 1.2
    assert 7.0 <= hz <= 13.0, f"MPC/MDM 实测 {hz:.1f} Hz（期望 ~10）"
    # 跨线程目标的单位必须是 A，而不是整数串口指令。
    target_A = shared._I_target["currents"]
    assert np.max(np.abs(target_A)) <= cfg.MAX_CURRENT_A + 1e-9
    assert np.allclose(shared._I_target["rec"]["requested_B_direction"],
                       cfg.CONTROL_FIELD_DIRECTION)
    assert shared._I_target["rec"]["solver_mode"] == "field-force-moore-penrose"
    assert all(len(row) == 24 for row in shared.drain_mpc_log())
    # 线程退出安全
    assert not worker.is_alive()


@test
def test_mpc_path_tangent_reference():
    worker = ControlWorker(SharedState(), S)
    path = [np.array([0.0, 0.0]), np.array([3.0, 0.0])]
    ref, vref = worker._reference_window(path, 1.0, np.array([0.0, 0.0]))
    assert len(ref) == cfg.MPC_HORIZON and len(vref) == cfg.MPC_HORIZON
    assert all(abs(v[1]) < 1e-12 and v[0] >= 0.0 for v in vref)
    assert np.allclose(ref[0], [0.1, 0.0])
    # 同一实际位置重复求参考不能像旧 _ref_arc 那样随时间持续向前漂移。
    ref_again, _ = worker._reference_window(path, 1.0, np.array([0.0, 0.0]))
    assert np.allclose(ref_again[0], ref[0])
    moved_ref, _ = worker._reference_window(path, 1.0, np.array([1.4, 0.2]))
    assert np.allclose(moved_ref[0], [1.5, 0.0])
    # 水平路径不得像旧实现一样把 y 轴速度也错误设成 +speed。
    my = ForceMPC(1.0 / cfg.MPC_HZ, c_drag=9.42)
    out_y = my.compute(0.0, 0.0, [p[1] for p in ref],
                       [v[1] for v in vref])
    assert abs(out_y["F0"]) < 1e-12


@test
def test_mpc_bounded_qp_and_actual_force_smoothing():
    hard = ForceMPC(0.1, horizon=3, c_drag=9.42, fmax=5.0,
                    w_delta=0.0)
    smooth = ForceMPC(0.1, horizon=3, c_drag=9.42, fmax=5.0,
                      w_delta=10.0)
    # 不足 horizon 的参考必须保持末项补齐，且箱约束严格成立。
    a = hard.compute(0.0, 0.0, [10.0], [0.0], f_prev=-4.0)
    b = smooth.compute(0.0, 0.0, [10.0], [0.0], f_prev=-4.0)
    assert np.max(np.abs(a["F_seq"])) <= 5.0 + 1e-12
    assert np.max(np.abs(b["F_seq"])) <= 5.0 + 1e-12
    assert abs(b["F0"] + 4.0) < abs(a["F0"] + 4.0)


@test
def test_mpc_tuned_force_scale():
    """1 mm/s 时黏性物理标尺为 9.42µN；默认权重不得再把力压到 1~2µN。"""
    tuned = ForceMPC(0.1, horizon=cfg.MPC_HORIZON, c_drag=9.42,
                     w_pos=cfg.MPC_W_POS, w_vel=cfg.MPC_W_VEL,
                     w_u=cfg.MPC_W_U, fmax=cfg.MPC_FMAX_UN,
                     w_delta=cfg.MPC_W_DU)
    legacy = ForceMPC(0.1, horizon=3, c_drag=9.42,
                      w_pos=1.0, w_vel=2.0, w_u=0.05,
                      fmax=40.0, w_delta=0.10)
    refs = [0.1, 0.2, 0.3]
    speeds = [1.0, 1.0, 1.0]
    f_new = tuned.compute(0.0, 0.0, refs, speeds)["F0"]
    f_old = legacy.compute(0.0, 0.0, refs, speeds)["F0"]
    assert f_new >= 5.0, f"调参后 1mm/s 首步力仍过小: {f_new:.3f}µN"
    assert f_new >= 3.0 * f_old, (f"控制惩罚降低 10 倍后提升不足: "
                                 f"old={f_old:.3f}, new={f_new:.3f}µN")


@test
def test_worker_solver_cache_isolation():
    worker = ControlWorker(SharedState(), S)
    assert worker.solver is not S
    before = None if S._cache_pos is None else S._cache_pos.copy()
    worker.solver.field_at(np.array([1e-3, 0.0, 0.0]), np.zeros(6))
    if before is None:
        assert S._cache_pos is None
    else:
        assert np.array_equal(S._cache_pos, before)


@test
def test_mpc_live_parameter_refresh():
    shared = SharedState()
    worker = ControlWorker(shared, S)
    shared.set_kalman(np.zeros(2), np.zeros(2), time.time())
    shared.set_reference([np.zeros(2), np.array([2.0, 0.0])], 1.0,
                         time.time())
    params = {"fmax": 25.0, "mpc_on": True, "mpc_horizon": 4,
              "mpc_w_pos": 2.5, "mpc_w_vel": 1.5,
              "mpc_w_u": 0.004, "mpc_w_du": 0.008,
              "fz_lift": 0.0, "eso_d": np.zeros(2),
              "viscosity_mPas": 1000.0, "bead_radius_m": 0.5e-3}
    shared.set_params(params, time.time())
    worker._step()
    assert worker.mpc_x.N == 4
    assert worker.mpc_x.wp == 2.5 and worker.mpc_x.wv == 1.5
    assert worker.mpc_x.wu == 0.004 and worker.mpc_x.wd == 0.008
    assert worker.mpc_x.fmax == 25.0
    snap = shared._I_target
    assert np.allclose(snap["ref_target"], [0.1, 0.0])
    # 运行中改值，下一工作周期必须重建而无需停止线程。
    params.update({"mpc_horizon": 2, "mpc_w_pos": 4.0})
    shared.set_params(params, time.time())
    worker._step()
    assert worker.mpc_x.N == 2 and worker.mpc_x.wp == 4.0


# ---- Test 5: 电流执行 30Hz ----
@test
def test_current_30hz():
    ex = CurrentExecutor()
    shared = SharedState()
    shared.set_I_target(np.full(6, 0.5), None, np.zeros(2), 0, 0, 0)
    n = 30
    t0 = time.perf_counter()
    first = None
    for k in range(n):
        out = ex.step(shared, 1.0 / cfg.CURRENT_HZ, np.zeros(3), S, [0] * 6)
        if k == 0:
            first = out["cmd"]
    dt_meas = time.perf_counter() - t0
    hz = n / dt_meas
    assert hz > 300, f"电流执行实测 {hz:.0f}Hz（应远超 30Hz 下限）"
    # 0→0.5A 的首帧插值为约 0.167A，即约 8 个指令；若安培/指令
    # 被重复换算，这里会错误饱和到 9。
    assert first == [8] * 6, f"安培/指令单位换算异常: {first}"


# ---- Test 6: PWM 20kHz（STM32 侧常量） ----
@test
def test_pwm_constant():
    assert cfg.PWM_HZ == 20000.0
    # Python 侧不生成 PWM：电流执行周期独立于 PWM
    assert cfg.CURRENT_HZ == 30.0


# ---- Test 7: 六路电流幅值 ----
@test
def test_current_bounds():
    rng = np.random.default_rng(1)
    for _ in range(10):
        F = rng.uniform(-50e-6, 50e-6, 3)
        rec = S.solve_commands(np.zeros(3), F, cmd_prev=0)
        assert np.max(np.abs(rec["currents"])) <= cfg.MAX_CURRENT_A + 1e-9
        assert np.max(np.abs(rec["commands"])) <= cfg.CMD_MAX


# ---- Test 8: 指令斜率 ----
@test
def test_command_slew():
    ex = CurrentExecutor()
    shared = SharedState()
    last = [0] * 6
    prev = [0] * 6
    # 连续大目标跳变：每帧实际指令变化都必须 ≤9
    shared.set_I_target(np.full(6, 2.0), None, np.zeros(2), 0, 0, 0)
    for frame in range(20):
        d = ex.step(shared, 1.0 / 30, np.zeros(3), S, prev)
        last = d["cmd"]
        assert max(abs(a - b) for a, b in zip(last, prev)) <= cfg.MAX_DELTA_CMD, \
            f"指令斜率超限: {last} vs {prev}"
        prev = last
        shared.set_last_sent(prev)
    # 运行时上限也必须在执行层成立；降低上限时幅值安全优先。
    d = ex.step(shared, 1.0 / 30, np.zeros(3), S, [99] * 6, max_cmd=50)
    assert max(abs(c) for c in d["cmd"]) <= 50


# ---- Test 9: RL 模型方程 L dI/dt + R I = V ----
@test
def test_rl_equation():
    ex = CurrentExecutor()
    cmd = [99, 0, 0, 0, 0, 0]          # V = 24V → I∞ = 2A
    dt = 1.0 / 30
    Is = []
    for _ in range(90):                 # 3 秒
        ex.update_est(dt, np.zeros(3), S, cmd)
        Is.append(ex.I_est[0])
    # 与解析解 I(t) = I∞(1−e^{−t/τ}) 比较
    tau = cfg.L_COIL_H / cfg.R_COIL_OHM
    for k, t in enumerate([(k + 1) * dt for k in range(90)]):
        if k % 30 == 0 and t > 0.1:
            I_analytic = 2.0 * (1 - math.exp(-t / tau))
            assert abs(Is[k] - I_analytic) < 0.05, \
                f"t={t:.2f}s: I_est={Is[k]:.3f} vs 解析 {I_analytic:.3f}"


# ---- Test 10: 恒定目标 → I_est 收敛到 V/R ----
@test
def test_rl_steady_state():
    ex = CurrentExecutor()
    cmd = [50, -50, 25, -25, 0, 0]
    for _ in range(300):                # 10 秒 ≫ τ=22.75ms
        ex.update_est(1.0 / 30, np.zeros(3), S, cmd)
    for j, c in enumerate(cmd):
        I_inf = (c / 99.0) * cfg.V_SUPPLY / cfg.R_COIL_OHM
        assert abs(ex.I_est[j] - I_inf) < 0.01, \
            f"通道 {j}: I_est={ex.I_est[j]:.4f} ≠ V/R={I_inf:.4f}"
        # 稳态与指令-电流映射一致
        assert abs(I_inf - c * cfg.CMD_TO_A) < 1e-9


# ---- Test 11: COMSOL 标定保持 ----
@test
def test_comsol_calibration():
    from dipole_solver import validate_against_comsol_reference
    results, ok = validate_against_comsol_reference(S)
    assert len(results) > 0, "参考文件存在但无结果"
    assert ok, f"COMSOL 标定失败: {[(r['coil'], round(r['relative_error'],4)) for r in results]}"
    for r in results:
        assert r["relative_error"] < 0.05      # 单线圈 1A @原点 |B|≈11.6mT


# ---- Test 12: 力回代 ----
@test
def test_force_roundtrip():
    for F_des in [np.array([10e-6, 0, 0]), np.array([0, -20e-6, 5e-6])]:
        rec = S.solve_commands(np.zeros(3), F_des, cmd_prev=0)
        F_re = S.force_at(np.zeros(3), rec["currents"])
        rel = np.linalg.norm(F_re - F_des) / np.linalg.norm(F_des)
        assert rel <= max(rec["force_error_percent"] / 100, 0.02) + 1e-9


# ---- Test 13: 视觉丢帧不崩溃 ----
@test
def test_vision_loss():
    kf = KalmanFilter2D()
    ex = CurrentExecutor()
    shared = SharedState()
    shared.set_I_target(np.full(6, 0.3), None, np.zeros(2), 0, 0, 0)
    dt = 1.0 / 30
    for k in range(150):                # 5 秒无观测
        p, v = kf.step(dt, None)        # 仅预测
        d = ex.step(shared, dt, np.append(p, 0.0) * 1e-3, S, [0] * 6)
        assert np.all(np.isfinite(p)) and np.all(np.isfinite(d["I_est"]))
        assert np.all(np.isfinite(d["B"]))
    assert not d["low_field"] or True   # 低场告警可出现但不得 NaN


# ---- Test 14: MDM 慢求解不阻塞视觉循环 ----
@test
def test_solver_no_block():
    shared = SharedState()
    worker = ControlWorker(shared, S)
    shared.set_kalman(np.zeros(2), np.zeros(2), time.time())
    shared.set_reference([np.zeros(2), np.array([1.0, 0.0])], 1.0, time.time())
    shared.set_params({"fmax": 40.0, "max_active": 6, "mpc_on": True,
                       "fz_lift": 0.0, "eso_d": np.zeros(2),
                       "viscosity_mPas": 1000.0, "bead_radius_m": 0.5e-3},
                      time.time())
    # 注入慢求解（模拟 50ms 的 MDM）
    orig_solve = worker.solver.solve_field_force_pseudoinverse
    def slow_solve(*a, **kw):
        time.sleep(0.05)
        return orig_solve(*a, **kw)
    worker.solver.solve_field_force_pseudoinverse = slow_solve
    worker.start()
    # 模拟 30Hz 视觉主循环：测最大帧间隔
    intervals = []
    t_prev = time.perf_counter()
    t_end = t_prev + 1.0
    while time.perf_counter() < t_end:
        time.sleep(1.0 / 30)                    # 主循环节拍（视觉）
        now = time.perf_counter()
        intervals.append(now - t_prev)
        t_prev = now
        shared.set_kalman(np.zeros(2), np.zeros(2), time.time())
    shared.stop()
    worker.join(timeout=2.0)
    max_iv = max(intervals)
    assert max_iv < 0.06, \
        f"30Hz 视觉循环受求解阻塞: 最大帧间隔 {max_iv*1e3:.1f} ms"
    # 慢求解确实发生过
    assert shared._I_target["seq"] > 0


if __name__ == "__main__":
    print("=" * 64)
    print("多速率控制架构单元测试")
    print("=" * 64)
    for fn in [test_vision, test_kalman_30hz, test_mpc_mdm_10hz,
               test_mpc_path_tangent_reference,
               test_mpc_bounded_qp_and_actual_force_smoothing,
               test_mpc_tuned_force_scale,
               test_worker_solver_cache_isolation,
               test_mpc_live_parameter_refresh,
               test_current_30hz, test_pwm_constant, test_current_bounds,
               test_command_slew, test_rl_equation, test_rl_steady_state,
               test_comsol_calibration, test_force_roundtrip,
               test_vision_loss, test_solver_no_block]:
        fn()
    n_fail = sum(1 for _, ok, _ in RESULTS if not ok)
    print("-" * 64)
    print(f"总计 {len(RESULTS)} 项, 通过 {len(RESULTS) - n_fail}, 失败 {n_fail}")
    sys.exit(1 if n_fail else 0)
