# -*- coding: utf-8 -*-
"""
磁驱运动仿真器基础验证（Test 1~8，独立运行：py tests/test_bead_sim.py）
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from bead_sim import (MagnetModel, FluidModel, FrictionModel, BeadSimulator,
                      G_ACC, MU0, V_EPS)

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


@test
def test1_moment_value():
    """Test 1: 1 mm N38 → m_b ≈ 5.0×10⁻⁴ A·m²"""
    mag = MagnetModel(1.0, 1.20, 7500.0)
    assert abs(mag.moment_A_m2 - 5.0e-4) < 0.02e-4, \
        f"m_b = {mag.moment_A_m2:.4e} A·m²"
    s = BeadSimulator(mag)
    assert abs(s.solver.bead_moment - mag.moment_A_m2) < 1e-12


@test
def test2_zero_field_zero_force():
    """Test 2: 零电流 → B=0 → F=0"""
    sim = BeadSimulator()
    fm = sim.solver.forward_model(np.zeros(3), np.zeros(6))
    assert np.linalg.norm(fm["B"]) == 0.0
    assert np.linalg.norm(np.atleast_1d(fm["F"])) == 0.0


@test
def test3_zero_friction():
    """Test 3: μ=0 → 无底面摩擦，速度精确 = F_m_actual/c_v（实际回代力）"""
    sim = BeadSimulator(MagnetModel(), FluidModel(1000.0), FrictionModel(0.0))
    F_des_N = 30e-6                       # 目标 30 µN +x
    rec = sim.solver.solve_commands(np.zeros(3), np.array([F_des_N, 0, 0]),
                                    cmd_prev=None)
    I = rec["currents"]
    r = sim.step(I, 1.0 / 300.0)
    # 整数量化使实际力 ≠ 目标力，以实际回代力为基准
    F_act_N = r["Fx_uN"] * 1e-6
    v_expected = F_act_N / sim.c_v * 1e3                  # mm/s
    assert abs(r["speed"] - v_expected) / v_expected < 1e-6, \
        f"v={r['speed']:.4f} vs 期望 {v_expected:.4f} mm/s"
    assert r["state"] == "SLIDING"
    assert abs(r["Ffric_x_uN"]) < 1e-9 and abs(r["Ffric_y_uN"]) < 1e-9


@test
def test4_zero_viscosity_guard():
    """Test 4: η→0 数值保护，不允许除零"""
    fluid = FluidModel(0.0)               # 0 黏度
    c = fluid.drag_coefficient(0.5e-3)
    assert c >= 1e-12, "阻力系数下限保护失效"
    sim = BeadSimulator(MagnetModel(), fluid, FrictionModel(0.0))
    rec = sim.solver.solve_commands(np.zeros(3), np.array([5e-6, 0, 0]),
                                    cmd_prev=None)
    r = sim.step(rec["currents"], 1.0 / 300.0)
    assert np.isfinite(r["speed"]) and np.isfinite(r["x"]) and np.isfinite(r["y"])


@test
def test5_static_below_threshold():
    """Test 5: |F_m| < μmg → STATIC（磁珠不动）。
    取 0.5×阈值目标——整数量化后仍远低于 μN。"""
    sim = BeadSimulator(MagnetModel(), FluidModel(1000.0), FrictionModel(0.10))
    F_start = sim.F_start                  # N
    rec = sim.solver.solve_commands(
        np.zeros(3), np.array([0.5 * F_start, 0, 0]), cmd_prev=None)
    x0, y0 = sim.pos.copy()
    for _ in range(60):
        r = sim.step(rec["currents"], 1.0 / 300.0)
    assert r["state"] == "STATIC", f"应静止，实际 {r['state']}"
    assert abs(r["speed"]) < 1e-12
    assert abs(sim.pos[0] - x0) < 1e-12 and abs(sim.pos[1] - y0) < 1e-12
    # 静摩擦恰好抵消磁力（F_fric = −F_m）
    assert abs(r["Ffric_x_uN"] + r["Fx_uN"]) < 1e-6
    assert abs(r["Ffric_y_uN"] + r["Fy_uN"]) < 1e-6


@test
def test6_sliding_above_threshold():
    """Test 6: |F_m| > μmg → SLIDING"""
    sim = BeadSimulator(MagnetModel(), FluidModel(1000.0), FrictionModel(0.10))
    F_start = sim.F_start
    rec = sim.solver.solve_commands(
        np.zeros(3), np.array([3.0 * F_start, 0, 0]), cmd_prev=None)
    r = sim.step(rec["currents"], 1.0 / 300.0)
    assert r["state"] == "SLIDING"
    assert r["speed"] > 0
    # 动摩擦 = −μN 沿运动方向
    assert abs(r["Ffric_x_uN"] + sim.friction.mu * sim.N_normal * 1e6) < 1e-6


@test
def test7_steady_state_velocity():
    """Test 7: 逐点稳态校验 v = (‖F_m(pos)‖ − μmg)/c_v。
    力随位置变化是真实物理（场空间分布），校验必须逐点进行。"""
    sim = BeadSimulator(MagnetModel(), FluidModel(1000.0), FrictionModel(0.10))
    F_des_N = 30e-6
    rec = sim.solver.solve_commands(np.zeros(3), np.array([F_des_N, 0, 0]),
                                    cmd_prev=None)
    dt = 1.0 / 300.0
    for k in range(600):               # 2 s
        r = sim.step(rec["currents"], dt)
        if k % 100 == 0 or k == 599:
            p3 = np.array([sim.pos[0], sim.pos[1], 0.0])
            F_here = float(np.linalg.norm(
                sim.solver.force_at(p3, rec["currents"])[:2]))
            v_expected = (F_here - sim.F_start) / sim.c_v * 1e3
            assert abs(r["speed"] - v_expected) / max(v_expected, 1e-9) < 1e-3, \
                f"k={k}: v={r['speed']:.6f} vs 逐点 v_ss={v_expected:.6f} mm/s"


@test
def test8_force_direction_reversal():
    """Test 8: 磁力反向 → 运动方向同步反转"""
    sim = BeadSimulator(MagnetModel(), FluidModel(1000.0), FrictionModel(0.10))
    rec_p = sim.solver.solve_commands(np.zeros(3), np.array([30e-6, 0, 0]),
                                      cmd_prev=None)
    for _ in range(30):
        sim.step(rec_p["currents"], 1.0 / 300.0)
    assert sim.vel[0] > 0
    rec_n = sim.solver.solve_commands(np.zeros(3), np.array([-30e-6, 0, 0]),
                                      cmd_prev=rec_p["commands"])
    # 允许 slew 过渡：跑足够帧，直到收敛到新稳态
    for _ in range(120):
        sim.step(rec_n["currents"], 1.0 / 300.0)
    assert sim.vel[0] < 0, f"反向后速度仍为正: {sim.vel}"
    r = sim.step(rec_n["currents"], 1.0 / 300.0)
    # 逐点稳态：当前位置的实际力决定稳态速度（方向 −x）
    p3 = np.array([sim.pos[0], sim.pos[1], 0.0])
    F_here = float(np.linalg.norm(
        sim.solver.force_at(p3, rec_n["currents"])[:2]))
    v_neg = (F_here - sim.F_start) / sim.c_v * 1e3
    assert abs(r["vx"] - (-v_neg)) / v_neg < 1e-3


if __name__ == "__main__":
    print("=" * 64)
    print("磁驱运动仿真器基础验证（Test 1~8）")
    print("=" * 64)
    for fn in [test1_moment_value, test2_zero_field_zero_force,
               test3_zero_friction, test4_zero_viscosity_guard,
               test5_static_below_threshold, test6_sliding_above_threshold,
               test7_steady_state_velocity, test8_force_direction_reversal]:
        fn()
    n_fail = sum(1 for _, ok, _ in RESULTS if not ok)
    print("-" * 64)
    print(f"总计 {len(RESULTS)} 项, 通过 {len(RESULTS) - n_fail}, 失败 {n_fail}")
    sys.exit(1 if n_fail else 0)
