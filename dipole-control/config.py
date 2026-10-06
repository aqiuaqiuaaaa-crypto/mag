import math
import os

"""
集中物理与控制参数配置（所有可调常数唯一来源，float64）
======================================================
"""

# ---------- 磁珠 ----------
BEAD_DIAMETER_MM = 1.0  # N38 球形 NdFeB 直径
BEAD_BR_T = 1.20  # 剩磁 (T)

# ---------- 液体环境 ----------
VISCOSITY_MPA_S = 1000.0  # 1000 mPa·s

# ---------- 相机 / 视觉 ----------
CAM_INDEX = 1
FRAME_W = 1920  # 1080p
FRAME_H = 1080
VIEW_WIDTH_MM = 20.0  # 画面实际宽度 2cm
DISP_W, DISP_H = 960, 540  # GUI 显示分辨率
FRAME_OFFSET_MM = (0.0, 0.0)  # 模型原点在相机世界 XY 中的位置；待实机标定，默认 0

# ---------- 控制 ----------
CONTROL_HZ = 30  # 目标控制频率
TICK_MS = 33  # 1000/30 ≈ 33.3 → 33 ms（约 30 Hz，dt 用实测值）

# ---------- 电流 ----------
MAX_CURRENT_A = 2.0  # 电流幅值 ±2 A
CMD_MAX = 99  # STM32 指令范围 ±99
DEFAULT_CMD_LIMIT = 50  # GUI 默认安全上限；运行时可在 1~99 内调整
CMD_TO_A = MAX_CURRENT_A / CMD_MAX  # I_A = command × (2/99)
# 每帧电流变化率：满量程 2A 的 10% = 0.2A。
# 指令为整数，10 指令 ≈ 0.2020 A 略超 0.2A，故取 9 指令 = 0.1818 A，严格 ≤ 0.2 A/帧。
MAX_DELTA_CMD = 9
# 10 Hz worker target half-width about last actually sent command.
# Nominal 3 x 30 Hz frames per period; final per-frame safety stays at 9.
WORKER_TARGET_DELTA_MAX = 27  # select 9 for legacy A/B and rollback
MAX_DELTA_A = MAX_DELTA_CMD * CMD_TO_A  # ≈ 0.1818 A

# ---------- 串口 ----------
BAUDRATE = 115200
SERIAL_FRAME_BYTES = 43  # "a0:+03d,...,a5:+03d\r\n" 固定 43 字节

# ---------- 解算器 ----------
CONV_TOL = 0.02  # 收敛判据：force_relative_error ≤ 2%（统一）
B_EPS = 1e-15  # |B| 下限保护（避免 ∇|B| 在 B→0 处 NaN/inf）
SOLVER_N_ITER = 8  # 每个起点的高斯-牛顿最大迭代
SOLVER_N_STARTS = 3  # 多起点数（热启动 + 2 个冷启动）
LAMBDA1_TAU = 1.0  # L1 正则化自适应系数: λ1 = tau1·(‖F‖/Imax)²·Imax
LAMBDA2_TAU = 1.0  # Tikhonov L2 自适应系数: λ2 = tau2·(‖F‖/Imax)²
SOLVER_WARN_MS = 33.0  # 单次解算超过 33ms 在 GUI 告警

# ---------- 模型 ----------
MODEL_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "models",
    "N30LM_six_coils_180dipoles.json",
)
COIL_ORDER = ["+X", "+Y", "+Z", "-X", "-Y", "-Z"]  # a0~a5，绝不可改变
DIPOLES_PER_COIL = 30
N_COILS = 6
TOTAL_DIPOLES = N_COILS * DIPOLES_PER_COIL  # 180

# ---------- 仿真器保留的历史 PID 默认值（实机 GUI 不再使用） ----------
PID = {
    "Kpx": 20.0,
    "Kix": 2.0,
    "Kdx": 5.0,
    "Kpy": 20.0,
    "Kiy": 2.0,
    "Kdy": 5.0,
}
PID_FMAX_UN = 100.0  # 期望磁力范数限幅 (µN)
VEL_LPF_ALPHA = 0.6  # RAW/EMA 状态估计的速度滤波系数（新值权重）

# ---------- 摩擦 / 法向减摩 ----------
RHO_BEAD = 7500.0  # 磁珠密度 (kg/m³)，N38 NdFeB
RHO_FLUID = 1000.0  # 液体密度 (kg/m³)
G_ACC = 9.81  # 重力加速度 (m/s²)
N_MIN_UN = 2.0  # 最小安全法向力 (µN)
FZ_MAX_UN = 40.0  # 最大向上减摩力 (µN)，不做完全悬浮
NORMAL_RATIO_DEFAULT = 0.4  # 目标法向力比例 N_target/W_eff
FRICTION_V_EPS = 0.05  # 摩擦平滑速度阈值 (mm/s)

# ---------- 路径 ----------
PATH_SPEED_MM_S = 1.0  # 路径跟踪期望速度 (mm/s)
PATH_DS_MM = 0.15  # 路径弧长重采样间距 (mm)
# 路径连续投影的保守软件默认；不是实机标定的最终参数。
PATH_BACKTRACK_MM = 0.3
PATH_PROJ_FWD_MIN_MM = 1.0
PATH_PROJ_FWD_FACTOR = 3.0  # forward = max(min_mm, factor * (Fmax/c) * T_mpc)
PATH_LOCAL_PROJ_MAX_DIST_MM = 1.5
PATH_LOST_DIST_MM = 3.0
PATH_FINISH_TOL_MM = 0.5  # 保留 GUI spin_tol_default 原默认语义

# ---------- ESO ----------
ESO_MODE = "first_order"  # L3 default; "legacy" for L0; GUI checkbox supplies off
ESO_OMEGA0 = 4.0  # 软件默认带宽 (rad/s)，沿用历史值，尚未实机标定
ESO_FAL_DELTA = 0.05  # fal 函数线性段宽度 (mm)
ESO_DIST_LIMIT_UN = 80.0  # 扰动估计限幅 (µN)

# ---------- Kalman ----------
KALMAN_Q_POS = 1e-4  # 过程噪声位置方差 (mm²)
KALMAN_Q_VEL = 5e-3  # 过程噪声速度方差 (mm²/s²)
KALMAN_R = 0.005  # 观测噪声方差 (mm²)，对应 ~0.07mm 视觉噪声

# ---------- 执行器动态（仅估计，无电流传感器） ----------
L_COIL_H = 0.273  # 线圈电感 (H)
R_COIL_OHM = 12.0  # 线圈电阻 (Ω)

# ---------- 多速率控制架构 ----------
VISION_HZ = 30.0  # 视觉检测
KALMAN_HZ = 30.0  # Kalman 状态估计
MPC_HZ = 10.0  # MPC 控制器
SOLVER_HZ = 10.0  # MDM 磁力逆解
CURRENT_HZ = 30.0  # 电流执行层（插值+斜率+量化+串口）
PWM_HZ = 20000.0  # PWM 频率（STM32/H 桥侧，Python 仅记录与配置）
KALMAN_STALE_S = 0.15  # 保守软件门槛；age > 此值暂停 worker，非实机标定
TARGET_STALE_HOLD_S = 0.30  # age > 此值冻结目标接受/插值，保持最后成功命令
TARGET_STALE_STOP_S = 1.00  # age > 此值由 GUI normal_stop 按原斜率归零

# MPC（准静态模型：x' = x + dt·(F+d)/c，c = 斯托克斯阻力系数）
MPC_HORIZON = 3  # 预测步数（10Hz × 3 = 300ms 视界）
MPC_EFFORT_MODE = "steady_state"  # L3 default; "absolute" for L0/L1
MPC_W_POS = 1.0  # 位置误差权重 (1/mm²)
MPC_W_VEL = 2.0  # 速度误差权重（速度=(F+d)/c，代数关系）
MPC_W_U = 0.005  # 控制量权重 (1/µN²)：原 0.05 会把 µN 力过度压小
MPC_W_DU = 0.01  # 磁力变化权重 (1/µN²)：保留平滑但不阻碍起步
MPC_FMAX_UN = 40.0  # MPC 每步水平力限幅 (µN)

# 线圈电气模型（RL，一阶；无电流传感器，I_est 为估计值）
V_SUPPLY = 24.0  # 母线电压 (V)；D=cmd/99 → I∞ = D·V/R = cmd×2/99 ✓

# 磁矩对齐 / 低场诊断
ZETA_R = (
    8.0 * math.pi * (BEAD_DIAMETER_MM * 0.5e-3) ** 3
)  # 旋转阻尼 ζ_r=8πηa³ (N·m·s)，η=1 Pa·s
B_MAG_THRESHOLD_T = 1e-4  # 低场阈值 (T)：低于此值磁矩方向归一化不可靠
ALIGNMENT_RATIO_MIN = 5.0  # ratio = T_current/τ_r ≥ 5 视为准静态

# 文献法 field-force 整体驱动目标默认值；GUI 可实时调整方向和模长。
CONTROL_FIELD_DIRECTION = (0.0, 0.0, -1.0)
CONTROL_FIELD_TARGET_MT = 10.0
CONTROL_FIELD_MIN_MT = 8.0
CONTROL_FIELD_MAX_MT = 12.0

# 原定向场求解器仍保留给未来的姿态/杆状机器人控制及其独立单元测试。
ALIGNMENT_FIELD_DIRECTION = (0.0, 0.0, -1.0)
ALIGNMENT_FIELD_TARGET_MT = CONTROL_FIELD_TARGET_MT
ALIGNMENT_FIELD_MIN_MT = CONTROL_FIELD_MIN_MT
ALIGNMENT_FIELD_MAX_MT = CONTROL_FIELD_MAX_MT

# ---------- 定向磁场联合逆解 ----------
B_DIRECTION_TOL_DEG = 5.0  # 方向收敛容差 (deg)
LINEAR_COND_MAX = 1e10  # 6×6/3×6 线性解条件数上限（超过则回退 GN）
