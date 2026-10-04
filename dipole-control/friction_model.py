"""
底面接触摩擦 / 法向力模型（贴底磁珠减摩控制）
==============================================
法向力：  N = max(N_min, W_eff − Fz_lift)，Fz_lift > 0 为向上磁力
有效重力： W_eff = (ρ_bead − ρ_fluid)·V·g   （浮力已扣除）
库仑摩擦（平滑）： F_fric = −μ·N·tanh(v / v_eps)   （对 x/y 分别计算）
静/动摩擦平滑切换： μ_eff(v) = μ_d + (μ_s − μ_d)·exp(−(|v|/v_eps)²)
    |v|→0 时趋近静摩擦 μ_s，|v|≫v_eps 时趋近动摩擦 μ_d，无数值跳变。

减摩控制器（lift）：
    N_target = normal_force_ratio · W_eff
    Fz_lift = clip(W_eff − N_target, 0, Fz_max)   且 N_target ≥ N_min
    默认 ratio ∈ [0.3, 0.5]：适度减小法向压力，**不做完全悬浮**。

单位约定：力 µN，速度 mm/s，长度 mm。
"""

import math

import numpy as np


def effective_weight_uN(rho_bead, rho_fluid, radius_m, g=9.81):
    """有效重力（扣除浮力），单位 µN"""
    v = 4.0 / 3.0 * math.pi * radius_m**3
    return (rho_bead - rho_fluid) * v * g * 1e6


def normal_force_uN(w_eff, fz_lift, n_min):
    """法向接触力：N = max(N_min, W_eff − Fz_lift)"""
    return max(float(n_min), float(w_eff) - max(float(fz_lift), 0.0))


def lift_force_uN(w_eff, normal_ratio, fz_max, n_min):
    """Fz 减摩控制器输出：Fz_lift = clip(W_eff(1−ratio), 0, Fz_max)，
    且保证 N_target = W_eff − Fz_lift ≥ N_min。不做完全悬浮。"""
    n_target = max(float(n_min), float(w_eff) * float(normal_ratio))
    fz = float(w_eff) - n_target
    return float(np.clip(fz, 0.0, float(fz_max)))


def mu_effective(v, mu_static, mu_dynamic, v_eps):
    """静→动摩擦系数平滑过渡：μ_eff(v) = μ_d + (μ_s−μ_d)·exp(−(|v|/v_eps)²)"""
    v = float(v)
    return float(mu_dynamic) + (float(mu_static) - float(mu_dynamic)) * math.exp(
        -((v / float(v_eps)) ** 2)
    )


def friction_comp_uN(v_des_x, v_des_y, normal_force, mu_static, mu_dynamic, v_eps):
    """摩擦补偿（加在磁力指令上，方向沿期望速度，幅值 μ_eff·N）。
    摩擦力本身为 −μ·N·tanh(v/v_eps)（阻碍运动），补偿取正号。
    返回 (fx_comp, fy_comp, mu_eff)。"""
    if normal_force <= 0:
        return 0.0, 0.0, float(mu_dynamic)
    fx = (
        mu_effective(v_des_x, mu_static, mu_dynamic, v_eps)
        * normal_force
        * math.tanh(v_des_x / float(v_eps))
    )
    fy = (
        mu_effective(v_des_y, mu_static, mu_dynamic, v_eps)
        * normal_force
        * math.tanh(v_des_y / float(v_eps))
    )
    return (
        float(fx),
        float(fy),
        mu_effective(math.hypot(v_des_x, v_des_y), mu_static, mu_dynamic, v_eps),
    )
