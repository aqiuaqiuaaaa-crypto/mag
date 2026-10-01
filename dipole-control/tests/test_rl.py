# -*- coding: utf-8 -*-
"""RL 模块单元测试（py tests/test_rl.py，兼容 pytest）"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config as cfg
from dipole_solver import DipoleSolver
from rl_policy import (MLPPolicy, PathEnv, make_state, action_to_cmd,
                       make_circle_waypoints_mm, STATE_SCALE_E, STATE_SCALE_V)

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


@test
def test_policy_shapes():
    pol = MLPPolicy()
    a = pol.act(make_state(1.0, -0.5, 2.0, 0.0, [0] * 6))
    assert a.shape == (6,)
    assert np.all(np.abs(a) <= 1.0)
    states = np.array([make_state(1, 0, 0, 0, [0] * 6) for _ in range(5)])
    A = pol.act_batch(states)
    assert A.shape == (5, 6)


@test
def test_action_mapping_constraints():
    # 斜率约束结构性成立：任意动作 |Δcmd| ≤ 9；幅值 |cmd| ≤ 99；整数
    rng = np.random.default_rng(0)
    cmd = np.zeros(6, dtype=int)
    for _ in range(300):
        a = rng.uniform(-1, 1, 6) * 3.0     # 故意超界
        new = action_to_cmd(a, cmd)
        assert np.max(np.abs(new - cmd)) <= cfg.MAX_DELTA_CMD + 1e-9
        assert np.all(np.abs(new) <= cfg.CMD_MAX)
        assert np.all(new == np.round(new))
        cmd = new


@test
def test_env_constraints():
    env = PathEnv(S, make_circle_waypoints_mm())
    env.reset()
    rng = np.random.default_rng(1)
    prev = env.cmd.copy()
    for i in range(300):
        st, r, done, info = env.step(rng.uniform(-1, 1, 6))
        assert np.all(np.abs(env.cmd) <= cfg.CMD_MAX)
        assert np.max(np.abs(env.cmd - prev)) <= cfg.MAX_DELTA_CMD + 1e-9
        assert np.isfinite(r) and np.all(np.isfinite(st))
        prev = env.cmd.copy()
        if done:
            env.reset()
            prev = env.cmd.copy()   # reset 后指令清零，重置基准


@test
def test_env_dynamics():
    # 斯托克斯准静态：稳态速度 v = F/c（无噪声下逐步验证）
    env = PathEnv(S, make_circle_waypoints_mm(), force_noise=0.0)
    env.reset(start_mm=np.zeros(2))
    env.cmd = np.array([50, 0, 0, 0, 0, 0])
    I = env.cmd * cfg.CMD_TO_A
    F = S.force_at(np.zeros(3), I)[:2] * 1e6
    v_expected = F / env.c
    st, r, done, info = env.step(np.zeros(6))   # action=0 → cmd 不变(0)... 注意cmd被覆盖
    # cmd 在 step 内由 action 重建：action=0 → Δ=0 → cmd 保持 step 前值? 
    # action_to_cmd(0, cmd_prev=env.cmd) → env.cmd 不变 ✓
    assert np.allclose(env.vel_mm, v_expected, rtol=1e-9), \
        f"动力学不一致: {env.vel_mm} vs {v_expected}"


@test
def test_state_scale():
    s = make_state(4.0, -4.0, 15.0, -15.0, [99, -99, 0, 0, 99, -99])
    assert np.all(np.abs(s) <= 1.0 + 1e-9), f"状态归一化越界: {s}"


@test
def test_save_load_roundtrip(tmp_path=None):
    import tempfile
    pol = MLPPolicy()
    p = os.path.join(tempfile.gettempdir(), "_rl_test_policy.json")
    pol.save(p, {"note": "test"})
    pol2 = MLPPolicy.load(p)
    assert np.array_equal(pol.get_weights(), pol2.get_weights())
    assert pol2.state_dim == 10 and pol2.act_dim == 6
    os.remove(p)


@test
def test_es_runs():
    # 小规模 ES 训练冒烟：结构完整、返回有限、历史记录正确
    from rl_policy import train_es
    pol, hist = train_es(S, make_circle_waypoints_mm(n_wp=4), iters=2,
                         pop=6, episode_steps=40, seed=0, log=lambda s: None)
    assert len(hist) == 2
    assert all(np.isfinite(h["mean"]) for h in hist)
    assert pol.n_params == 1606


@test
def test_trained_policy_file():
    # 训练产物（train_rl.py 生成）可加载且推理正常
    p = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                     "models", "rl_policy.json")
    if not os.path.exists(p):
        print("    (跳过: models/rl_policy.json 不存在，先运行 py train_rl.py)")
        return
    pol = MLPPolicy.load(p)
    a = pol.act(make_state(1.0, 1.0, 0, 0, [0] * 6))
    assert a.shape == (6,) and np.all(np.isfinite(a))


@test
def test_deploy_pipeline():
    # 部署链路冒烟：状态→策略→action_to_cmd→安全层，端到端约束成立
    import magnetic_dipole_pid as m
    import tempfile
    m.SETTINGS_FILE = os.path.join(tempfile.gettempdir(), "_rl_deploy_settings.json")
    from PySide6.QtWidgets import QApplication
    app = QApplication.instance() or QApplication([])
    win = m.RLPathControl() if hasattr(m, "RLPathControl") else None
    if win is None:
        import rl_path_control as r
        win = r.RLPathControl()
    assert win.policy is not None, "默认策略应已加载"
    win.bead = (960, 540, 300)
    win.frame_size = (1920, 1080)
    win.video.frame_size = win.frame_size
    win.make_circle(400)
    win.start_tracking()
    assert not win.chk_cur_live.isChecked()
    prev = list(win.last_sent_cmd)
    for _ in range(30):
        win.tick()
        d = np.max(np.abs(np.array(win.last_sent_cmd) - np.array(prev)))
        assert d <= cfg.MAX_DELTA_CMD + 1e-9, "部署帧斜率超限"
        assert np.max(np.abs(win.last_sent_cmd)) <= cfg.CMD_MAX
        prev = list(win.last_sent_cmd)
    print(f"    部署 30 帧: cmd={win.last_sent_cmd}, 策略 {win.last_rl_ms:.2f}ms")


if __name__ == "__main__":
    print("=" * 64)
    print("RL 模块单元测试")
    print("=" * 64)
    for fn in [test_policy_shapes, test_action_mapping_constraints,
               test_env_constraints, test_env_dynamics, test_state_scale,
               test_save_load_roundtrip, test_es_runs, test_trained_policy_file,
               test_deploy_pipeline]:
        fn()
    n_fail = sum(1 for _, ok, _ in RESULTS if not ok)
    print("-" * 64)
    print(f"总计 {len(RESULTS)} 项, 通过 {len(RESULTS) - n_fail}, 失败 {n_fail}")
    sys.exit(1 if n_fail else 0)
