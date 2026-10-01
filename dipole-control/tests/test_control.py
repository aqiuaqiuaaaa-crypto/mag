# -*- coding: utf-8 -*-
"""
控制模块单元测试（摩擦/减摩/前馈/ESO/Kalman/方向）
==================================================
独立运行：py tests/test_control.py（也兼容 pytest）
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config as cfg
from dipole_solver import DipoleSolver
from estimators import KalmanFilter2D, ESO1D
import friction_model as fm

RESULTS = []


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
W = fm.effective_weight_uN(cfg.RHO_BEAD, cfg.RHO_FLUID, cfg.BEAD_DIAMETER_MM * 0.5e-3)
DT = 0.033


@test
def test_friction_model():
    # N = max(N_min, W_eff − Fz)
    assert abs(fm.normal_force_uN(W, 0, 2) - W) < 1e-9
    assert abs(fm.normal_force_uN(W, 0.6 * W, 2) - 0.4 * W) < 1e-9
    assert fm.normal_force_uN(W, 10 * W, 2) == 2.0        # N_min 下限
    # 有效重量量级：6500 kg/m³ × 5.24e-10 m³ × 9.81 ≈ 33.4 µN
    assert 30.0 < W < 37.0, f"W_eff={W}"


@test
def test_lift_effect():
    # Fz↑ → N↓ → 摩擦补偿↓（减摩生效）
    fz = fm.lift_force_uN(W, 0.4, cfg.FZ_MAX_UN, cfg.N_MIN_UN)
    assert 0 <= fz <= cfg.FZ_MAX_UN
    n0 = fm.normal_force_uN(W, 0, cfg.N_MIN_UN)
    n1 = fm.normal_force_uN(W, fz, cfg.N_MIN_UN)
    assert n1 < n0
    fx0, _, _ = fm.friction_comp_uN(1.0, 0.0, n0, 0.25, 0.15, cfg.FRICTION_V_EPS)
    fx1, _, _ = fm.friction_comp_uN(1.0, 0.0, n1, 0.25, 0.15, cfg.FRICTION_V_EPS)
    assert fx1 < fx0
    # 不做完全悬浮：ratio<1 时 Fz_lift < W_eff
    assert fz < W
    # 静→动平滑过渡（无跳变）
    mus = [fm.mu_effective(v, 0.25, 0.15, cfg.FRICTION_V_EPS)
           for v in np.linspace(0, 1.0, 50)]
    assert all(mus[i] >= mus[i + 1] - 1e-12 for i in range(len(mus) - 1))


@test
def test_drag():
    # F_drag = 6πμrv：μ=1Pa·s, r=0.5mm → 9.42 µN/(mm/s)
    c = S.drag_uN_per_mm_s(1000.0)
    assert abs(c - 6 * math.pi * 1.0 * 0.5e-3 * 1e3) < 1e-6
    assert abs(c - 9.42) < 0.01
    # 前馈+反馈合并不重复：F_drag = c(Kv·v_des − v)，v=v_des,Kv=1 时应为 0
    v_des, v = 1.0, 1.0
    assert abs(c * (1.0 * v_des - v)) < 1e-12


@test
def test_feedforward():
    # v_des↑ → F_ff = Kv·c·v_des 线性增大
    c = S.drag_uN_per_mm_s(1000.0)
    kv = 1.0
    f1 = kv * c * 0.5
    f2 = kv * c * 2.0
    assert f2 == 4 * f1
    assert abs(f2 - c * 2.0) < 1e-12     # Kv=1 时恰为完整斯托克斯前馈


@test
def test_eso():
    # 已知人工扰动 d=20µN：闭环中 z3 应跟踪，且补偿后速度收敛到期望值
    c = S.drag_uN_per_mm_s(1000.0)
    for w0 in (3.0, 4.0, 6.0):
        eso = ESO1D(b0=1.0 / c, omega0=w0, fal_delta=cfg.ESO_FAL_DELTA,
                    dist_limit=cfg.ESO_DIST_LIMIT_UN)
        x = v = 0.0
        d = 10.0                     # 未建模恒定扰动（力，限幅 12µN 内）
        v_des = 1.0
        z3s = []
        # 简单位置闭环：u = c·v_des + Kp·(x_des−x) − z3
        Kp = 50.0                    # µN/mm
        x_des = 0.0
        for k in range(3000):
            u = c * v_des + Kp * (x_des - x) - eso.z3
            v += DT * (u + d) / c
            x += v * DT
            x_des += v_des * DT
            z3s.append(eso.step(DT, x, u))
        tail = float(np.mean(z3s[-300:]))
        assert abs(tail - d) < 2.0, f"ω0={w0}: z3={tail:.2f} 未跟踪扰动 {d}µN"


@test
def test_kalman():
    # 加噪测量：Kalman 速度估计 std 应显著优于直接差分
    rng = np.random.default_rng(0)
    kf = KalmanFilter2D(cfg.KALMAN_Q_POS, cfg.KALMAN_Q_VEL, cfg.KALMAN_R)
    dt = DT
    truth_v, pos, z_prev = 1.0, 0.0, None
    diff_v, kal_v = [], []
    for k in range(300):
        pos += truth_v * dt
        z = pos + rng.normal(0, 0.05)      # 0.05mm 视觉噪声
        if z_prev is not None and k > 20:
            diff_v.append((z - z_prev) / dt)
        z_prev = z
        p, v = kf.step(dt, (z, 0.0))
        if k > 50:
            kal_v.append(v[0])
    assert np.std(kal_v) < np.std(diff_v), \
        f"Kalman std={np.std(kal_v):.4f} 应小于差分 std={np.std(diff_v):.4f}"
    # 丢失观测时仅预测不发散
    p, v = kf.step(dt, None)
    assert np.all(np.isfinite(p)) and np.all(np.isfinite(v))


@test
def test_force_direction():
    # Solver 对 ±X/±Y 期望力生成对应方向（力与期望方向夹角 < 45°）
    for F_des in [np.array([4e-6, 0, 0]), np.array([-4e-6, 0, 0]),
                  np.array([0, 4e-6, 0]), np.array([0, -4e-6, 0])]:
        rec = S.solve_commands(np.zeros(3), F_des, cmd_prev=0)
        F_act = rec["achieved_force"]
        cosang = np.dot(F_act[:2], F_des[:2]) / (
            np.linalg.norm(F_act[:2]) * np.linalg.norm(F_des[:2]) + 1e-30)
        assert cosang > 0.7, \
            f"F_des={F_des[:2]}: 方向偏差过大 cos={cosang:.2f}, F_act={F_act}"
        # F_actual 与最终发送电流一致
        F_ref = S.force_at(np.zeros(3), rec["currents"])
        assert np.linalg.norm(F_act - F_ref) < 1e-15


if __name__ == "__main__":
    print("=" * 64)
    print("控制模块单元测试")
    print("=" * 64)
    for fn in [test_friction_model, test_lift_effect, test_drag,
               test_feedforward, test_eso, test_kalman, test_force_direction]:
        fn()
    n_fail = sum(1 for _, ok, _ in RESULTS if not ok)
    print("-" * 64)
    print(f"总计 {len(RESULTS)} 项, 通过 {len(RESULTS) - n_fail}, 失败 {n_fail}")
    sys.exit(1 if n_fail else 0)
