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
独立仿真器保留；CURT 遥测已集成主 GUI 的“CURT / ADC”页，独立 CLI 继续保留。

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
| `curt_telemetry.py` | 共享 ADCStreamDecoder / ADCSnapshot / CSV 格式，以及独立接收 CLI |
| `adc_csv.py` | GUI ADC CSV 的有界队列与独立写盘线程，不访问串口或控制状态 |
| `tests/test_gui_curt_telemetry.py` | 主 GUI 接收、生命周期、监视隔离与 CSV 回归测试 |
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
  视觉 gap 或 dt>0.15s 后重置位置、清零扰动；其余 dt 用 Tω≤0.5 子步稳定校正。
  原 `ESO1D` 类原文保留为 legacy，它的二阶假设不符合一阶对象，仅用于 A/B。
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

## 主 GUI CURT / ADC 监视（2026-10-04）

启动 `magnetic_dipole_pid.py`，使用顶部原有串口连接，再进入“CURT / ADC”页。
主 GUI 是该 COM 的唯一所有者：同一个 `self.ser` 发送原 a0...a5 signed 指令并接收
`@ADC`。不要同时运行独立 `curt_telemetry.py` 或其他程序打开同一端口。

- 接收不依赖相机、MPC 是否开始或当前 Tab；独立 15 ms RX QTimer 只读当前可用字节，
  每次最多 1024 bytes，串口 `timeout=0`，不使用阻塞 readline。共享 ADCStreamDecoder
  处理半帧、粘包、旧 12-byte echo、损坏帧和重新同步；没有第二个 RX reader。
- 六行固定为 `a0/Pole1, a1/Pole3, a2/Pole5, a3/Pole4, a4/Pole6, a5/Pole2`，
  每行显示当前 GUI command 和未标定 ADC raw，内部数组始终按 a0...a5，不按 Pole 重排。
  GUI command 与 MCU 快照不严格同步；raw 不是 Ampere，也不是电气模型的 I_est。
- 显示 frame_count、MCU timestamp_ms、UTC PC receive time、data age、valid/running、
  error_flags、overrun_count、dma_error_count、dma_late_count，以及拒绝帧和 RX/parser 异常数。
- LIVE 表示新鲜且 valid/running 为 1、error_flags 为 0；超过 500 ms 没有完整 ADC 帧为
  STALE（连接后尚未收到帧也显示 STALE）；故障快照或 RX/parser 异常为 INVALID；
  未连接为 DISCONNECTED。100 ms 显示定时器持续更新数据年龄，所有状态只影响监视。
- “选择 CSV 路径 / 开始 ADC 记录 / 停止 ADC 记录”记录未来收到的完整快照，使用与 CLI
  相同的 15 列 CSV（timestamp_pc、frame_count、timestamp_mcu、raw0...raw5、错误计数、
  valid/running）。CSV 与 MPC 实验 CSV 分开，文件已存在时拒绝覆盖。
- CSV 写盘使用独立线程和 256 行有界队列，RX 回调只做不等待的入队；写盘故障或队列满
  会明确显示并停止记录，不影响 MPC 或串口发送。停止记录异步完成已接收行并关闭文件。
- 断开停止 RX/ADC 记录并清除旧快照与半帧；重连开始新解码会话，不自动续记旧 CSV。
  普通停止/急停只停止控制，已连接的 ADC 观察继续。退出先按原流程急停和关闭串口，
  再最多等待 1 秒收尾 CSV；若存储设备长期挂起，该等待上限不代表文件已成功保存。

MPC、Kalman/ESO、Fz 减摩、六路逆解、R-L 电流模型、10/30 Hz 调度和发送函数保持。
ADC raw 仅显示、监测和记录，尚未标定为 Ampere，不进入 MPC/PWM 反馈，不实现电流 PI。
固件 ADC 500 Hz / telemetry 约 10 Hz 不变。GUI 软件已使用模拟串口验证；新 telemetry HEX
仍待烧录后的实板 GUI 验证，本轮没有自动连接、驱动或烧录硬件。

```powershell
C:\Python314\python.exe -B -m pytest -q -p no:cacheprovider tests/test_gui_curt_telemetry.py tests/test_curt_telemetry.py
```

## 独立 CURT raw 串口工具（保留，2026-10-03）

独立工具 `curt_telemetry.py` 接收新版固件的 10 Hz 遥测。ADC 内部仍为 500 Hz，
工具只显示/记录未标定 raw，不做安培换算，也不参与 PI 或控制反馈。

本项目当前解释器是 `C:\Python314\python.exe`，已补齐 PySide6 6.11.2 和
pyserial 3.5。numpy、OpenCV、pip 版本保持原样。

先烧录本轮 telemetry HEX（根目录 `artifacts/curt-telemetry-20261003/pwm_02.hex`），
关闭 GUI/串口助手对同一 COM 口的连接。在 `dipole-control` 目录运行：

```powershell
C:\Python314\python.exe -B -m serial.tools.list_ports
C:\Python314\python.exe -B curt_telemetry.py --port COM4 --duration 30 --csv curt_raw.csv
```

COM4 是示例，请替换为当前实际端口；本轮开发环境未检测到 COM 口。默认模式只读，
进入/退出均不发送控制命令。CSV 使用 UTC ISO 时间，逐帧刷新，拒绝覆盖已有文件。
终端打印 raw、有效/运行状态、错误计数、估计采样频率、拒绝帧数；5 秒没有完整遥测时提示等待。

固件帧格式（14 个十进制无符号字段，一行 CRLF）：

```text
@ADC,<frame_count>,<timestamp_ms>,<raw0>,<raw1>,<raw2>,<raw3>,<raw4>,<raw5>,<error_flags>,<overrun_count>,<dma_error_count>,<dma_late_count>,<valid>,<running>\r\n
```

`timestamp_ms` 是完整快照发布时间。`valid`/`running` 为 0 或 1；采样失败时仍发故障状态，
raw 可保留最后一帧。采样频率用相邻快照的 frame_count 和 MCU 时间增量计算，不把
10 Hz 的串口帧率当作 ADC 采样率。正常相邻遥测约增加 50 个 frame_count，总体约 500/s。
解码器处理拆包、粘包、截断和前面无换行的旧 12 字节 echo；控制帧/未知前缀不会被解析为 ADC。

如需同一端口上同时驱动和记录，可以显式指定原 43 字节 signed 命令。下例先发全零并记录
2 秒基线，随后以 30 Hz 重发仅 a0 为 +05 的固定命令；10 秒到期或 Ctrl+C 时尝试发全零：

```powershell
C:\Python314\python.exe -B curt_telemetry.py --port COM4 --duration 10 --csv curt_a0.csv --command 'a0:+05,a1:+00,a2:+00,a3:+00,a4:+00,a5:+00'
```

`--command` 是显式测试选项，数值单位是已有 PWM 命令；与 GUI 共用原协议，但由独立进程
持有串口。不要同时用两个程序打开同一 COM 口。串口断开时全零发送可能失败，原固件没有新增
通信超时停机机制。依次只改变 a1..a5 的非零槽位，确认主变化分别对应 raw[1]..raw[5]。
逻辑到物理顺序仍为 `[Pole1, Pole3, Pole5, Pole4, Pole6, Pole2]`。

独立 parser/记录器测试：

```powershell
C:\Python314\python.exe -B -m pytest -q -p no:cacheprovider tests/test_curt_telemetry.py
```

测试使用内存串口，不连接或驱动实际硬件。以下是 2026-10-03 的历史软件验证记录，
当前主 GUI 集成验证以 PROJECT_MEMORY 最新阶段为准：原上位机测试的缺依赖项已全部解决；默认环境
完整运行是 66/67，一个线程时序测试越限。仅在诊断子进程设
`OPENBLAS_NUM_THREADS=1`、`OMP_NUM_THREADS=1` 后该组 18/18 通过，原配置和控制代码未改。
