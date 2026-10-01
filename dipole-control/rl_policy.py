# -*- coding: utf-8 -*-
"""
强化学习路径控制核心模块（纯 numpy 自包含：MLP 策略 + 进化策略 ES 训练）
==========================================================================
设计
----
- 策略：MLP float64，输入归一化状态（目标相对位置/速度/当前指令），
  输出 6 维动作 tanh([-1,1])，动作解释为**每帧指令增量**：
      Δcmd = clip(action × MAX_DELTA_CMD, ±9)
      cmd  = clip(round(cmd_prev + Δcmd), ±99)     （整数指令）
  斜率约束 |Δcmd| ≤ 9（=0.1818A ≤ 0.2A/帧）与幅值约束 ±99（↔±2A）由该映射
  **结构性保证**，训练环境与实机部署使用同一映射。
- 训练环境 PathEnv：以 180 偶极子标定模型（DipoleSolver）为被控对象，
  准静态低雷诺数动力学 v = F(I)/c（c = 6πμr，斯托克斯），位置积分 30Hz，
  力加 5% 乘性噪声提升鲁棒性；奖励 = 前进进度 + 到点奖励 − 动作平滑罚
  − 电流幅值罚（L1 风格）；越界/超时终止。
- 训练：进化策略 ES（镜像采样 + 排名基线），无反向传播依赖；
  权重存 JSON（与标定模型同样的工程习惯）。

状态向量（10 维，归一化）
--------------------------
[ex/s_e, ey/s_e, vx/s_v, vy/s_v, cmd0/99, ..., cmd5/99]
s_e = 4mm，s_v = 15mm/s
"""

import json
import math
import os
import time

import numpy as np

import config as cfg

MAX_DELTA_CMD = cfg.MAX_DELTA_CMD     # 9 指令/帧 = 0.1818A，严格 ≤ 0.2A
CMD_MAX = cfg.CMD_MAX                 # ±99 ↔ ±2A
CMD_TO_A = cfg.CMD_TO_A
CONTROL_DT = 1.0 / cfg.CONTROL_HZ     # 1/30 s
WAYPOINT_TOL_MM = 0.5
BOUNDS_MM = 20.0                      # 磁珠超出 ±20mm 视野外终止

STATE_SCALE_E = 4.0                   # 位置误差归一化尺度 (mm)
STATE_SCALE_V = 15.0                  # 速度归一化尺度 (mm/s)


# ================= 策略网络 =================
class MLPPolicy:
    """10 → 32(tanh) → 32(tanh) → 6(tanh)，参数一维向量化的 numpy MLP"""

    def __init__(self, state_dim=10, hidden=32, act_dim=6, weights=None):
        self.state_dim, self.hidden, self.act_dim = state_dim, hidden, act_dim
        self.shapes = [(hidden, state_dim), (hidden,),
                       (hidden, hidden), (hidden,),
                       (act_dim, hidden), (act_dim,)]
        self.sizes = [int(np.prod(s)) for s in self.shapes]
        self.n_params = sum(self.sizes)
        if weights is None:
            self.theta = self._init_theta()
        else:
            self.theta = np.asarray(weights, dtype=np.float64)
            assert self.theta.size == self.n_params, \
                f"策略参数维度不符: {self.theta.size} != {self.n_params}"

    def _init_theta(self):
        rng = np.random.default_rng(0)
        dims = [self.state_dim, self.hidden, self.hidden, self.act_dim]
        params = []
        for i in range(0, 6, 2):
            W = rng.normal(0, math.sqrt(2.0 / dims[i // 2]),
                           size=self.shapes[i])
            params.append(W.ravel())
            params.append(np.zeros(self.shapes[i + 1]))
        return np.concatenate(params)

    def get_weights(self):
        return self.theta.copy()

    def set_weights(self, theta):
        self.theta = np.asarray(theta, dtype=np.float64).copy()

    def _forward(self, x):
        p = self.theta
        o = 0
        W1 = p[o:o + self.sizes[0]].reshape(self.shapes[0]); o += self.sizes[0]
        b1 = p[o:o + self.sizes[1]]; o += self.sizes[1]
        W2 = p[o:o + self.sizes[2]].reshape(self.shapes[2]); o += self.sizes[2]
        b2 = p[o:o + self.sizes[3]]; o += self.sizes[3]
        W3 = p[o:o + self.sizes[4]].reshape(self.shapes[4]); o += self.sizes[4]
        b3 = p[o:o + self.sizes[5]]
        h1 = np.tanh(W1 @ x + b1)
        h2 = np.tanh(W2 @ h1 + b2)
        return np.tanh(W3 @ h2 + b3)

    def act(self, state):
        """单状态 → 动作 [-1,1]^6"""
        return self._forward(np.asarray(state, dtype=np.float64))

    def act_batch(self, states):
        return np.array([self._forward(s) for s in states])

    def save(self, path, meta=None):
        data = {"algo": "ES-MLP", "state_dim": self.state_dim,
                "hidden": self.hidden, "act_dim": self.act_dim,
                "weights": [float(x) for x in self.theta],
                "meta": meta or {}}
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f)
        return path

    @classmethod
    def load(cls, path):
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        if data.get("algo") != "ES-MLP":
            raise ValueError(f"非 ES-MLP 策略文件: {data.get('algo')}")
        pol = cls(state_dim=int(data["state_dim"]), hidden=int(data["hidden"]),
                  act_dim=int(data["act_dim"]), weights=data["weights"])
        pol.meta = data.get("meta", {})
        return pol


# ================= 状态构造（训练/部署共用） =================
def make_state(ex_mm, ey_mm, vx_mm, vy_mm, cmd):
    """误差(mm)/速度(mm/s)/当前整数指令 → 归一化状态 (10,)"""
    return np.array([ex_mm / STATE_SCALE_E, ey_mm / STATE_SCALE_E,
                     vx_mm / STATE_SCALE_V, vy_mm / STATE_SCALE_V,
                     cmd[0] / CMD_MAX, cmd[1] / CMD_MAX, cmd[2] / CMD_MAX,
                     cmd[3] / CMD_MAX, cmd[4] / CMD_MAX, cmd[5] / CMD_MAX],
                    dtype=np.float64)


def action_to_cmd(action, cmd_prev, max_delta_cmd=MAX_DELTA_CMD, cmd_max=CMD_MAX):
    """动作 → 新整数指令（训练与部署共用；约束结构性保证）"""
    d = np.clip(np.asarray(action, dtype=np.float64) * max_delta_cmd,
                -max_delta_cmd, max_delta_cmd)
    return np.clip(np.round(np.asarray(cmd_prev) + d), -cmd_max, cmd_max).astype(int)


# ================= 训练环境 =================
class PathEnv:
    """180 偶极子仿真环境：准静态斯托克斯动力学 + 与实机一致的电流约束"""

    def __init__(self, solver, waypoints_mm, viscosity_mPa_s=cfg.VISCOSITY_MPA_S,
                 episode_steps=400, force_noise=0.05, seed=None):
        self.solver = solver
        self.waypoints = np.asarray(waypoints_mm, dtype=np.float64)  # (M,2)
        self.c = solver.drag_uN_per_mm_s(viscosity_mPa_s)            # µN/(mm/s)
        self.episode_steps = episode_steps
        self.force_noise = force_noise
        self.rng = np.random.default_rng(seed)

    def reset(self, start_mm=None):
        self.wp_idx = 0
        self.step_count = 0
        self.cmd = np.zeros(6, dtype=int)
        self.vel_mm = np.zeros(2)
        if start_mm is None:
            start_mm = self.waypoints[0] + \
                self.rng.uniform(-1.5, 1.5, size=2)
        self.pos_mm = np.asarray(start_mm, dtype=np.float64).copy()
        return self._state()

    def _state(self):
        tgt = self.waypoints[min(self.wp_idx, len(self.waypoints) - 1)]
        e = tgt - self.pos_mm
        return make_state(e[0], e[1], self.vel_mm[0], self.vel_mm[1], self.cmd)

    def step(self, action):
        new_cmd = action_to_cmd(action, self.cmd)
        dcmd = new_cmd - self.cmd
        self.cmd = new_cmd
        I_A = self.cmd * CMD_TO_A
        F_uN = self.solver.force_at(
            np.array([self.pos_mm[0], self.pos_mm[1], 0.0]) * 1e-3, I_A)[:2] * 1e6
        if self.force_noise > 0:
            F_uN = F_uN * (1.0 + self.rng.normal(0, self.force_noise, size=2))
        self.vel_mm = F_uN / self.c                       # 斯托克斯准静态
        self.pos_mm = self.pos_mm + self.vel_mm * CONTROL_DT
        self.step_count += 1

        tgt = self.waypoints[min(self.wp_idx, len(self.waypoints) - 1)]
        d_prev = getattr(self, "_d_prev", None)
        d_now = float(np.linalg.norm(tgt - self.pos_mm))
        reward = 0.0
        if d_prev is not None:
            reward += 30.0 * (d_prev - d_now)             # 前进进度
        self._d_prev = d_now
        reward -= 0.001 * float(np.sum(dcmd ** 2))        # 动作平滑罚
        reward -= 0.0002 * float(np.sum(np.abs(self.cmd)))  # 电流幅值罚(L1)

        done = False
        if d_now < WAYPOINT_TOL_MM:
            reward += 5.0
            self.wp_idx += 1
            self._d_prev = None
            if self.wp_idx >= len(self.waypoints):
                reward += 20.0                             # 全程完成
                done = True
        if np.any(np.abs(self.pos_mm) > BOUNDS_MM):
            reward -= 10.0
            done = True
        if self.step_count >= self.episode_steps:
            done = True
        return self._state(), reward, done, {"dist": d_now}

    def rollout(self, policy, start_mm=None, noise_in_env=True):
        """完整回合，返回 (总奖励, 完成路点数, 步数)"""
        s = self.reset(start_mm)
        total, done = 0.0, False
        while not done:
            a = policy.act(s)
            s, r, done, info = self.step(a)
            total += r
        return total, self.wp_idx, self.step_count


# ================= ES 训练 =================
def train_es(solver, waypoints_mm, iters=80, pop=16, sigma=0.1, lr=0.03,
             episodes_per=1, episode_steps=400, seed=0, log=print,
             policy=None):
    """OpenAI-ES 风格进化策略训练（镜像采样 + 排名基线）。
    返回 (policy, history)"""
    rng = np.random.default_rng(seed)
    policy = policy or MLPPolicy()
    env = PathEnv(solver, waypoints_mm, episode_steps=episode_steps, seed=seed)
    n = policy.n_params
    theta = policy.get_weights()
    half = pop // 2
    history = []

    def evaluate(th, n_ep):
        policy.set_weights(th)
        rets = []
        for _ in range(n_ep):
            r, _, _ = env.rollout(policy)
            rets.append(r)
        return float(np.mean(rets))

    for it in range(iters):
        eps = [rng.normal(size=n) for _ in range(half)]
        returns = []
        for e in eps:
            returns.append(evaluate(theta + sigma * e, episodes_per))
            returns.append(evaluate(theta - sigma * e, episodes_per))
        returns = np.array(returns)
        # 排名基线（centered ranks → [-0.5, 0.5]）
        order = np.argsort(np.argsort(returns))
        ranks = (order / (len(returns) - 1) - 0.5)
        grad = np.zeros(n)
        for i, e in enumerate(eps):
            g = ranks[2 * i] - ranks[2 * i + 1]      # +ε 与 −ε 的镜像差
            grad += g * e
        theta = theta + (lr / (half * sigma)) * grad
        policy.set_weights(theta)
        history.append({"iter": it, "mean": float(returns.mean()),
                        "max": float(returns.max()),
                        "min": float(returns.min())})
        log(f"  ES iter {it + 1}/{iters}: return mean={returns.mean():8.2f} "
            f"max={returns.max():8.2f} min={returns.min():8.2f}")
    return policy, history


def make_circle_waypoints_mm(radius_mm=3.0, n_wp=8, center_mm=(0.0, 0.0)):
    """训练用圆形路径路点（世界 mm）"""
    return np.array([[center_mm[0] + radius_mm * math.cos(2 * math.pi * i / n_wp),
                      center_mm[1] + radius_mm * math.sin(2 * math.pi * i / n_wp)]
                     for i in range(n_wp)])


def evaluate_policy(solver, policy, waypoints_mm, n_ep=5, episode_steps=400,
                    seed=123):
    """确定性评估：完成率 / 平均终点距 / 平均回报"""
    env = PathEnv(solver, waypoints_mm, episode_steps=episode_steps, seed=seed)
    stats = []
    for _ in range(n_ep):
        s = env.reset()
        done = False
        while not done:
            s, r, done, info = env.step(policy.act(s))
        stats.append((env.wp_idx / len(waypoints_mm),
                      float(np.linalg.norm(
                          waypoints_mm[min(env.wp_idx, len(waypoints_mm) - 1)]
                          - env.pos_mm)),
                      env.wp_idx))
    comp = float(np.mean([s[0] for s in stats]))
    dist = float(np.mean([s[1] for s in stats]))
    return comp, dist, stats


if __name__ == "__main__":
    print("=" * 64)
    print("RL 模块自测（不训练，仅结构与环境约束验证）")
    print("=" * 64)
    from dipole_solver import DipoleSolver
    s = DipoleSolver.from_json()
    pol = MLPPolicy()
    print(f"策略参数量: {pol.n_params}, 状态 10 维, 动作 6 维")

    wps = make_circle_waypoints_mm()
    env = PathEnv(s, wps)
    st = env.reset()
    print(f"状态维度: {st.shape}, 初值范数: {np.linalg.norm(st):.3f}")

    # 约束验证：随机动作 200 步
    rng = np.random.default_rng(1)
    for i in range(200):
        a = rng.uniform(-1, 1, 6)
        st, r, done, info = env.step(a)
        assert np.all(np.abs(env.cmd) <= CMD_MAX), "幅值约束被违反"
        if i > 0:
            assert np.max(np.abs(env.cmd - prev_cmd)) <= MAX_DELTA_CMD, \
                "斜率约束被违反"
        prev_cmd = env.cmd.copy()
        assert np.all(np.isfinite(st)) and np.isfinite(r)
    print(f"200 步随机动作: |cmd|≤{CMD_MAX}, |Δcmd|≤{MAX_DELTA_CMD}, "
          f"奖励有限 ✓ (完成路点 {env.wp_idx}/{len(wps)})")

    # 保存/载入往返
    p = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "models", "_rl_roundtrip.json")
    pol.save(p, {"note": "roundtrip"})
    pol2 = MLPPolicy.load(p)
    assert np.array_equal(pol.get_weights(), pol2.get_weights())
    os.remove(p)
    print("策略 JSON 保存/载入往返 ✓")
    print("自测全部通过 ✓")
