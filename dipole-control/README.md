# 180 磁偶极子 MPC 实机磁控程序

主 GUI 的唯一自动轨迹控制器是 **MPC**。视觉状态与路径参考进入 10Hz 工作线程，
由 **180 磁偶极子标定模型**（6 路电磁铁 × 每路 30 个偶极子）计算六路电流目标，
30Hz 执行层经统一安全限幅和串口封帧发送到 STM32。

```text
Camera → Vision → Kalman/EMA/RAW + ESO → Path Reference → SharedState
  → ControlWorker / ForceMPC (10Hz) → DipoleSolver [B;F]=A·I 伪逆
  → CurrentExecutor (30Hz) → send_commands() → Serial → STM32
```

保留 GUI 目标磁场方向/模长、模型到相机坐标标定、Fz 减摩、手动六路电流、
手动磁力和诊断/标定。2026-10-04 移除旧 PD/PID 实机自动控制链；入口文件名仍为
`magnetic_dipole_pid.py`。强化学习 RL/ES-MLP 已于 2026-10-04 从正式工程移除；
独立仿真器保留。2026-10-28 已移除 CURT/ADC 测量链，主 GUI 只负责现有视觉与 MPC 控制。

## 运行方法

以下命令在 `dipole-control/` 目录执行；入口文件名保持不变。

```bash
pip install -r requirements.txt

python -m py_compile magnetic_dipole_pid.py
python -m py_compile dipole_solver.py

python dipole_solver.py          # 解算器自测（模型/梯度/逆解输出）
python tests/test_solver.py      # 14 项严格单元测试
python magnetic_dipole_pid.py    # 启动 GUI
```

> 本机为 Windows + Python 3.14，若 `python` 不可用请用 `py` 启动器。

## 文件

| 文件 | 说明 |
|---|---|
| `config.py` | 所有物理/控制参数集中配置（唯一来源） |
| `dipole_solver.py` | 180 偶极子磁场/解析梯度/约束正则化电流逆解（纯 numpy） |
| `mpc.py` | 10Hz 线性 MPC（准静态模型，输出目标磁力） |
| `multirate.py` | 多速率调度：10Hz 工作线程 + 30Hz 电流执行层 + R-L 线圈估计 |
| `magnetic_dipole_pid.py` | MPC-only PySide6 实机 GUI（30Hz 主循环 + 10Hz 工作线程） |
| `tests/test_solver.py` | 25 项严格单元测试 |
| `smoke_test.py` | 无头冒烟测试（不需相机/串口） |
| `models/N30LM_six_coils_180dipoles.json` | 标定偶极子模型 |

## 系统规格（严格固定）

- 磁珠：1mm N38 球形 NdFeB（m_b = (Br/μ0)·V ≈ 5.0×10⁻⁴ A·m²）；运动平面 z=0
- 液体 1000 mPa·s；相机 1920×1080；画面宽度 20mm；控制频率 ≈30Hz（TICK_MS=33，dt 实测）
- **模型通道顺序（任何模块不可改变）：a0~a5 = +X, +Y, +Z, −X, −Y, −Z**；
  这些是模型通道名，不再假定等于相机坐标方向，实机方向由六路线圈扫描标定。
- 协议量程 ±2A ↔ 串口 ±99（I_A = command × 2/99），GUI 默认运行上限为 ±50、
  可在顶部设为 1~99；协议固定 43 字节：
  `a0:+30,a1:-45,a2:+60,a3:-75,a4:+90,a5:-10\r\n`

## 物理模型

- 每路电磁铁 30 个磁偶极子（位置 mm、磁矩 mT·mm³@1A，由标定 JSON 加载并**严格校验**）。
  单位换算：mm→m ×1e-3；mT·mm³→A·m² ×1e-5（标定拟合约定 B[mT]=Σ[3(K·R)R/|R|⁵−K/R³]，
  R 用 mm、不含 μ0/4π；与 COMSOL 原始数据经 magnetic_field_validation / MPC统一框架 /
  运动demo 三处独立程序交叉验证；**勿改成 ×1e-12/μ0——那会使 B、F 全部偏小 4π 倍**）。
  绝对标定接口：`validate_against_comsol_reference()`（models/comsol_reference.json）。
- 单偶极子磁场 B_i = μ0/4π·[3R(m_i·R)/|R|⁵ − m_i/|R|³]，总场 ΣB_i（NumPy 向量化，
  按线圈预求和，无逐偶极子 Python 对象）。
- 磁珠为硬磁球形永磁体、磁矩沿当地磁场排列：**F = m_b·∇|B|**。
  **∇|B| 用解析磁场梯度张量** G = ∂B/∂x（3×3）计算：∇|B| = (BᵀG)/max(|B|, B_eps)；
  |B|→0 病态点由 B_eps 保护。7 点中心差分版本保留为 `grad_absB_numeric`（仅 DEBUG
  验证，单元测试验证两者相对误差 < 1e-5，实测 ~1e-7）。
- 模型校验失败（非 6×30=180 / coil_order 错误 / 维度错误）时抛 `ModelValidationError`：
  GUI 显示明确错误、**禁用自动控制与手动磁力**、仅保留手动电流；**不回退**到错误模型。
  `synthetic_ring()` 仅限 DEBUG/SIMULATION（同样 6×30=180）。

## 逆电流求解

### GUI 主控制链：field-force Moore–Penrose 伪逆

主 GUI 的 MPC 工作线程、手动磁力与方向/摩擦诊断调用 `solve_field_force_pseudoinverse()`：
由 GUI 给定 `B_des=B0·b_hat`，令 `M_des=m0·b_hat`，再由 180-MDM 的单位电流
场 `Bc` 和梯度 `Gc` 实时构造 `A_B=Bc.T`、`A_F[:,j]=Gc[j].T@M_des`，得到
`[B;F]=A_BF·I`。B、F 行量纲归一化后计算 Moore–Penrose 伪逆；无约束时
`I=A_BF⁺[B_des;F_des]` 是最小二范数电流。触及幅值/斜率限制时，穷举六通道
活动集精确求箱约束线性最小二乘，最后在整数指令邻域按输出残差、再按电流范数
选解。GUI 中的实际磁力、误差和闭环记录均用最终发送电流重新调用非线性正向模型
计算；线性矩阵回代值单独保留为诊断量。

### 保留的力-only 求解器：L1 + Tikhonov 箱约束高斯-牛顿

    min_I  0.5‖F(I) − F_des‖² + λ1·Σ|I_i| + 0.5·λ2·Σ I_i²
    s.t.   −2 ≤ I_i ≤ +2      （幅值约束）
           |I_i − I_prev_i| ≤ Δcmd×2/99 = 9×2/99 ≈ 0.1818A/帧 （斜率约束）

- **λ1（L1，最小电流绝对值之和）与 λ2（Tikhonov L2）是两个独立参数**，缺省按力标度
  自适应（conv_tol² 量级，保证力匹配优先），可显式覆盖。注意这不是单纯"Tikhonov
  正则化"。
- 求解：IRLS（w_i = 1/(|I_i|+ε)）二次近似 L1 + 高斯-牛顿局部线性化
  F(I+ΔI) ≈ F(I) + JΔI（J 为 3×6 数值雅可比，差分步长取幅值 5%）；
  **幅值与斜率约束以箱约束形式进入优化**（候选解投影到箱内 + 回溯线搜索），
  不是解后截断。
- F = m∇|B| 为电流偶函数（J(0)≡0）：热启动（上一帧实际发送电流）优先 + 多冷启动
  （单线圈满幅模式按力方向对齐度预排序），每个起点同一优化流程，按目标函数选优。
- 记录字段：initial_current / final_current / objective / force_error / converged /
  current_constraint_active / elapsed_ms。
- **收敛判据统一 2%**：force_relative_error = ‖F_act−F_des‖/max(‖F_des‖,eps) ≤ 2%。
  F_des 超出约束可达范围 → converged=False 且 current_constraint_active=True，
  GUI 显示"磁力不可达/受电流约束"；整数指令量化（0.0202A/格）导致的小力未收敛
  属正常现象（闭环可校正），不做掩饰。
- `solve_commands()` 返回的 F_actual / force_error / objective 均按**最终发送电流**
  （整数指令 × gain）重新调用力模型计算；指令域再做贪心细化（单路 ±1/±2、
  成对 ±1）以减小量化损失。

## 实机首次使用

1. 顶部先把“电流上限”设为 30~50，打开相机并稳定识别磁珠。
2. 在“诊断”页点击“扫描六路并自动标定”。程序逐路静置、缓升并仅统计稳定段速度，
   用模型单线圈力方向和视觉速度方向拟合二维正交坐标映射；该映射会持久化。
3. 在“路径与追踪”页设置路径、速度、采样间距和 MPC 参数，确认目标磁场方向/模长，
   点击“开始追踪”。自动控制唯一为 MPC；Kalman/ESO/Fz 设置在“高级控制”页，
   10Hz 工作线程生成磁力目标和六路电流，30Hz 执行层插值并发送。
4. 先用小路径、低速度验证。若扫描 RMS 超过 30°，程序拒绝应用标定并提示检查
   模型通道顺序与 STM32 a0~a5 接线，避免用错误映射闭环。

## 电流安全层（统一）

主 GUI 的所有模式（MPC/手动磁力/手动电流/诊断）发送前必经：
用户运行时幅值上限（默认 ±50，绝不超过协议 ±99）+ 斜率限幅 ≤ **9 指令/帧**
（= 0.1818A，严格 ≤ 0.2A/帧；取 9 而非 10
是因为 10×2/99=0.2020A 略超 0.2A）。
`normal_stop()` 按斜率限制逐帧归零；`emergency_stop()`（GUI 急停按钮）立即发送
全零帧 `a0:+00,...,a5:+00`，不受斜率限制。

## 磁珠运动仿真器（bead_sim）

`py bead_sim_gui.py` —— 1mm N38 自由对齐磁珠的磁驱运动仿真器，用于回答：
理论速度多少 / 摩擦启动阈值多少 / 磁力能否克服摩擦 / 参数对性能的影响 /
实验与仿真不一致时如何反求参数。

- **物理模型**（`bead_sim.py`）：磁矩 m_b=(Br/μ0)V 自由对齐，F=m_b∇|B|
  **全部来自现有 DipoleSolver**（解析梯度，为匹配仿真磁珠参数构造独立实例，
  不实现第二套磁场模型）；低雷诺数过阻尼动力学 F_m = c_v·v + F_friction，
  c_v=6πηr；库仑摩擦静/动切换（静止：|F_m|≤μN 保持，动摩擦 = −μN·v̂，
  反转保护）；float64，B_eps/v_eps/F_eps 数值保护，η→0 有下限保护。
- **模式**：Mode A 给定六路电流开环；Mode B 轨迹（圆/矩形/三角/直线）
  → 位置 PID（X/Y 独立+速度前馈）→ solve_commands 逆解 → 运动闭环，
  视觉反馈=真实位置+可调高斯噪声。
- **GUI**：磁珠/液体/摩擦参数全可调，实时显示 B(mT)/G(T/m)/F(µN)/阻力/摩擦/
  位置速度(mm, mm/s)/STATIC-SLIDING 状态/理论性能（c_v、F_start=μmg、
  v_ss 对照实测速度）/磁矩-磁场对齐角；画布显示工作区、轨迹、磁珠（含磁矩
  箭头）、力矢量。
- **参数扫描**（OFAT：直径/Br/黏度/μ）→ CSV + PNG；**实验拟合**：
  载入 t,x,y,I0..I5 实验轨迹，网格+细化搜索 η_eff/μ_eff，
  输出拟合值与前后轨迹对比图（合成数据验证：η 785/800，μ 0.165/0.15）。
- **CSV**：34 列逐仿真步（t/x/y/v/B(3)+幅值/G 九元素/F(3)+幅值/阻力/摩擦/
  I(6)/状态）。
- **验证**：`py tests/test_bead_sim.py`（Test 1~8：磁矩值/零场零力/零摩擦/
  零黏度保护/静摩擦保持/滑动启动/稳态速度逐点校验/力反向）。

## 多速率控制架构（新增）

这里的 **R-L** 指电阻—电感一阶电流模型，不是 Reinforcement Learning；
**ESO** 是扩张状态观测器，仍为 MPC 提供扰动估计。

```
视觉 30Hz → Kalman 30Hz → MPC 10Hz → MDM 逆解 10Hz → 电流执行 30Hz → PWM 20kHz(STM32)
```

- **10Hz 工作线程**（`multirate.ControlWorker`）：MPC（`mpc.py`，准静态线性模型
  x'=x+dt·(F+d)/c，位置+路径切向速度+控制量+ΔF 代价）用活动集精确求小规模
  箱约束 QP，输出 F_target → 180 偶极子构造 `[B;F]=A·I` 并伪逆求 I_target，
  写入线程安全 `SharedState`；工作线程使用独立的求解器缓存，**绝不阻塞或污染
  30Hz GUI 正向诊断**。
- **30Hz 电流执行层**（`multirate.CurrentExecutor`）：100ms 窗口内目标插值
  (α=1/3,2/3,1) → 斜率限幅 ≤9 指令/帧 → 整数量化 → 串口；R-L 线圈模型
  L·dI/dt=V−R·I（V=(cmd/99)·24V，稳态 I∞=cmd×2/99 与映射自洽，τ=22.75ms）
  维护 **I_est（估计电流，无电流传感器）**；用 I_est 走 MDM 正向模型得 B/G/∇|B|/F_est。
- **状态闭环**：GUI 每个 30Hz 周期向 MPC 发布最新位置、速度和实际已发送整数指令；
  MPC 的 ΔF 从这些实际指令正向计算出的磁力开始，而不是假定上一理想目标已经实现。
  每个 MPC 周期先把磁珠实际位置投影到路径，再从投影点向前一个周期生成预测视界，
  不再使用随时间开环漂移的 `_ref_arc`。路径速度参考按局部切线分解到 x/y；停止后
  再次启动会重建 SharedState/worker。
- **默认 MPC 权重**：`W_POS=1, W_VEL=2, W_U=0.005, W_DU=0.01`。后两项按 µN
  量纲设定，避免旧值 `0.05/0.10` 让 10µN 控制力的代价远高于 1mm 位置误差；
  `FMAX=40µN` 保持为安全上限，并不会主动放大未饱和的输出。
  这是 config 默认；现有 `gui_settings.json` 会覆盖成 `40/0.8/0.02/0.001`、
  `FMAX=50µN`。本次保留保存设置和所有权重、物理值。
- **用力中心**：应用默认 `steady_state`，用 `wu*(F−(c*v_ref−d))²`，H 不变、
  g 减去 `wu*(c*v_ref−d)`。路径页可选回 `absolute`（原 `wu*F²`）；独立
  `ForceMPC` 构造器保留 absolute 默认，应用通过配置明确选择新模式。
- **MPC 实时参数**：路径页可在线修改预测步数 `N`（1~6）、`Wpos/Wvel/Wu/Wdu`
  和独立水平力上限；GUI 每30Hz发布参数，工作线程在下一10Hz周期自动重建 MPC。
  画面黄色圆为 GUI 路点，品红圆为 MPC 首参考点，橙色箭头为 `F_target`，红色箭头
  为实际发送电流对应的 `F_actual`，可直接区分参考错误与执行换向滞后；MPC CSV
  同时记录首参考位置/速度和每周期实际使用的全部 MPC 参数。
- **磁矩对齐诊断**：τ_r = ζ_r/(m_b·|B|)，ζ_r=8πηa³=3.14e-9 N·m·s（不写死 15.4ms——
  它随 |B| 变化，B≈0.4mT 时 τ_r≈15.7ms）；ratio=T_current/τ_r <5 时告警
  "磁矩对齐非准静态"但不停止。低场检测 |B|<B_MAG_THRESHOLD_T 独立告警。
- **唯一自动控制器**：路径页固定显示 MPC，不再提供控制器选择；每次开始追踪
  都创建新的 SharedState、CurrentExecutor 和 10Hz ControlWorker。
- **诊断页多速率状态**：各层标称与实测频率、F_target、Bmag、τ_align、T/τ、I_est、
  MDM 耗时；“保存实验CSV”直接保存 24 列 10Hz MPC/MDM 日志，默认名
  `experiment_mpc10hz.csv`，不再生成旧实机控制日志。

## 高级控制与诊断（状态估计 / ESO / 减摩）

- **Fz 减摩**：N = max(N_min, W_eff − Fz_lift)，W_eff=(ρ珠−ρ液)·V·g≈33.4µN；
  Fz_lift = clip(W_eff·(1−ratio), 0, Fz_max)，默认 ratio=0.4（N 降至 40%，
  **适度减压、不做完全悬浮**）。
- **ESO**：默认 `FirstOrderESO`，与 c·v=F+d / x_dot=(u+d)/c 的单积分器一致。
  z1 为 mm、z2 为扰动力 µN；l1=2ω、l2=cω²，omega=4 沿用软件默认、未实机标定。
  GUI 先用上一执行帧的 R-L/MDM `F_est` 更新 observer，再发布给 MPC；统一 `z3`
  数组是兼容扰动力字段，新 observer 的内部 z2 通过它发布。
  PC observer 遇到 measurement gap（视觉缺帧）或 dt > 0.15 s 时，下一次有效量测
  只重锚 z1/上次量测，保留 z2 / d_hat；当帧不预测/校正未知缺口。
  observer lifecycle reset（FirstOrderESO OFF → ON、新 tracking、observer mode switch、
  新实例或显式 reset）才清零 z2 / d_hat；其余有效更新用 Tω≤0.5 子步稳定校正。
  原 `ESO1D` 类及行为不变，保留为 legacy；它的二阶假设不符合一阶对象，仅用于 A/B。
- **A/B 模式**：L0=legacy+absolute，L1=ESO off+absolute，L2=ESO off+steady_state，
  L3=first_order+steady_state（新默认）。高级控制页选择 observer / ESO 开关，路径页
  选择 effort；选择可持久化，旧设置没有新模式键时使用 L3。统一日志记录实际模式、
  d_hat/u_eso/F_ss，字段与时序见 [CONTROL_LOG.md](CONTROL_LOG.md)。
- **状态估计**：KALMAN（默认，[x,y,vx,vy] 常速度 KF，Q/R 可调）/ EMA / RAW；
  视觉丢失 >1s 自动正常停止。
- **方向测试模式**：恒定三维 F_test，实时显示 F_des/F_act/幅值误差/目标方向/
  实际方向/方向误差——区分自动控制问题、逆解问题与摩擦问题。
- **摩擦标定模式**：+X 方向逐级 Fz（默认 0~40µN）下 Fx 斜坡，运动判定后记录
  F_start(Fz) → CSV（Fz, N_est, F_start, velocity），拟合 μ_s·N。
- **线圈方向诊断表**：6 线圈各 ~1A 顺序扫描，模型 (Fx,Fy,Fz) vs 实验视觉平均
  速度，快速核对线圈编号/坐标/电流方向一致性。
- **执行器动态**：MPC 的 CurrentExecutor 保持原有一阶 I_est 电流估计及 B/F 诊断，
  I_est 是模型估计值；原实机链的独立执行器估计开关已移除。
- **性能**：状态栏保留各阶段耗时和超时提示；连续 5 帧超 33.3ms 显示“控制周期超时”。
- **MPC CSV（24 列）**：timestamp、mpc_ms、solver_ms、mpc_cost、ref_x/ref_y、
  vref_x/vref_y、Fx_target/Fy_target、force_error、converged、horizon、W_pos/W_vel/
  W_u/W_du、Fmax、a0_cmd..a5_cmd。字段顺序、工作线程生产逻辑和数值语义不变。

## 通用逆解与视觉

- 实机运动控制始终使用六路 field-force 伪逆，以同时满足目标力和磁场矢量；
  路径页的约束模式控件保留，开始追踪时固定为六路自由模式。
- DipoleSolver 的通用力-only、单极/三路稀疏逆解接口及其测试保持原样。
- 视觉：GRAY（默认）/HSV 两模式 + 形态学开/闭运算 + 面积/圆度/半径过滤。
- 路径：圆、矩形、三角形、鼠标绘制、路径速度和弧长采样间距均保留。
- 历史公共 `CSV_HEADER` 仍供独立工具使用；主 GUI 不再生产或导出该旧格式。

## GUI 功能

手动电流（滑条/数值 ±99，标注线圈方向）/ 手动磁力 / 识别参数 / 路径与追踪
（圆、矩形、三角形、右键鼠标绘制，自动移动到起点→跟踪→完成→正常停止）/ 物理参数
（模型 JSON 路径、放大倍数、磁珠、粘度）。顶栏：串口连接、停止（斜率归零）、**急停**。
“识别参数”页可保存带绿色路径和全部画面标注的截图，并可开始/结束 MP4 或 AVI 录像；
关闭相机或退出程序时，正在录制的视频也会自动完成封装并保存。

## 历史路线（historical / removed）

RL/ES-MLP 曾作为独立实验路线存在；其 GUI、策略部署、训练代码、专用测试、
策略权重和独立设置现已删除。当前正式实机自动轨迹控制仅使用 MPC。
历史分析报告和验证日志保留作记录，不代表当前可运行功能。

## 假设与注意事项

- 磁珠限制在 z=0 平面（Fz 期望为 0，但底层模型保留完整 Fx/Fy/Fz 与 G 张量）。
- 准静态低雷诺数假设；模型 JSON 坐标系需与相机世界系一致（x 右、y 上、z=0）。
- 解算可输出负电流（协议 ±99 双向）。
- 指令量化 0.0202A/格：小力目标下 2% 收敛判据可能因量化不可达（单元测试与 CSV
  如实记录），闭环控制可校正。

## 当前固件与控制日志（2026-10-28）

CURT/ADC 页面、接收器、专属 CSV 及 STM32 测量链已移除；不解析或等待测量遥测。
六路命令依旧为 43 字节有符号格式，通道对应物理 Pole `[1,3,5,4,6,2]`。
STM32 保留 12 字节幅值回传；当前 GUI 不消费该回传，控制不依赖回传确认。
MPC 10Hz / 执行 30Hz、三帧插值、限幅/斜率、FirstOrderESO、freshness、路径进度、
串口故障锁存、IDLE heartbeat、急停及 STM32 300ms watchdog 均保留。

“控制日志”页继续工作，100ms 状态定时器保留为 `control_log_status_timer`。
新日志 schema_version=2：控制流 95 列，worker 流 56 列，删除 11 个测量字段；
具体字段与历史兼容性见 [CONTROL_LOG.md](CONTROL_LOG.md)。用户设置文件不因清理重置。

正式 canonical HEX：`pwm_double20260926DC6output/MDK-ARM/pwm_02/pwm_02.hex`，
36,321 bytes，SHA-256：`b3dd7100d8e1cc43e50c61b5761b73fe17f73cb529482e9cb445edce5268879e`。
完整 Keil Rebuild、无测量链验收和冻结 SHA 比对通过；尚未烧录或实机验证。
构建入口及输入一致性规则见 [FIRMWARE_BUILD_SOP.md](../FIRMWARE_BUILD_SOP.md)。
历史 artifacts/ 和 csvvvv/ 保留，不能将旧测量版 HEX 作为当前控制版镜像。
