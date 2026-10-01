# -*- coding: utf-8 -*-
"""
强化学习策略训练脚本（ES，180 偶极子仿真环境）
================================================
用法：
    py train_rl.py                 # 默认 80 轮（约 3~5 分钟）
    py train_rl.py --quick         # 快速 20 轮（冒烟/演示）
    py train_rl.py --iters 150 --pop 20 --sigma 0.12 --lr 0.04
输出：models/rl_policy.json（含训练元数据），并打印确定性评估结果
（完成率 / 平均终点距）。训练与部署使用同一电流约束（±99 ↔ ±2A，
9 指令/帧斜率，整数指令量化）。
"""

import argparse
import os
import sys
import time

import numpy as np

import config as cfg
from dipole_solver import DipoleSolver
from rl_policy import (MLPPolicy, PathEnv, make_circle_waypoints_mm,
                       train_es, evaluate_policy)

DEFAULT_OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "models", "rl_policy.json")


def main():
    ap = argparse.ArgumentParser(description="RL(ES) 路径控制策略训练")
    ap.add_argument("--iters", type=int, default=80)
    ap.add_argument("--pop", type=int, default=16)
    ap.add_argument("--sigma", type=float, default=0.1)
    ap.add_argument("--lr", type=float, default=0.03)
    ap.add_argument("--episodes", type=int, default=1, help="每候选评估回合数")
    ap.add_argument("--steps", type=int, default=400, help="每回合最大步数")
    ap.add_argument("--radius", type=float, default=3.0, help="训练圆路径半径 mm")
    ap.add_argument("--nwp", type=int, default=8, help="训练路点数")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", type=str, default=DEFAULT_OUT)
    ap.add_argument("--quick", action="store_true", help="快速模式(20轮)")
    args = ap.parse_args()
    if args.quick:
        args.iters = min(args.iters, 20)
        args.steps = min(args.steps, 250)

    solver = DipoleSolver.from_json()
    wps = make_circle_waypoints_mm(radius_mm=args.radius, n_wp=args.nwp)
    print("=" * 64)
    print(f"RL(ES) 训练: {args.iters} 轮, 种群 {args.pop} (镜像), σ={args.sigma}, "
          f"lr={args.lr}")
    print(f"环境: 180 偶极子模型 + 斯托克斯准静态, {args.steps} 步/回合, "
          f"路点 {len(wps)} 个 (半径 {args.radius}mm)")
    print(f"电流约束与实机一致: ±{cfg.CMD_MAX} 指令 ↔ ±{cfg.MAX_CURRENT_A}A, "
          f"斜率 ≤{cfg.MAX_DELTA_CMD} 指令/帧 ({cfg.MAX_DELTA_A:.3f}A)")
    print("=" * 64)

    t0 = time.time()
    policy, history = train_es(
        solver, wps, iters=args.iters, pop=args.pop, sigma=args.sigma,
        lr=args.lr, episodes_per=args.episodes, episode_steps=args.steps,
        seed=args.seed,
        log=lambda s: print(f"  [{time.time() - t0:6.0f}s] {s}"))

    comp, dist, stats = evaluate_policy(solver, policy, wps, n_ep=6,
                                        episode_steps=args.steps)
    print("-" * 64)
    print(f"确定性评估 (6 回合): 路点完成率 {comp * 100:.1f}%, "
          f"平均终点距 {dist:.2f} mm")
    path = policy.save(args.out, meta={
        "iters": args.iters, "pop": args.pop, "sigma": args.sigma,
        "lr": args.lr, "seed": args.seed,
        "steps": args.steps, "waypoints": len(wps),
        "radius_mm": args.radius,
        "eval_completion": comp, "eval_final_dist_mm": dist,
        "history_tail": history[-5:],
        "current_limits": {"cmd_max": cfg.CMD_MAX,
                           "max_delta_cmd": cfg.MAX_DELTA_CMD},
    })
    print(f"策略已保存: {path}")
    print("训练完成 ✓")


if __name__ == "__main__":
    main()
