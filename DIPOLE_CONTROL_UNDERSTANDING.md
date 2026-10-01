# dipole-control 只读静态代码理解报告

> 分析日期：2026-09-30  
> 分析范围：`~/Desktop/mag/dipole-control`，并只读对照 `260426-1.py` 与 `pwm_double20230915DC6output`。  
> 方法：只读静态分析；未运行任何项目程序、测试、仿真或控制循环，未连接相机/串口/硬件，未安装依赖，未烧录 STM32，未发送任何命令。  
> 证据标签：**[CONFIRMED]** 源码直接确认；**[INFERRED]** 多处代码支持的高可信推断；**[UNKNOWN]** 当前源码无法确认。

## 1. Executive summary

**[CONFIRMED] 这个项目中的 MPC 不是“六电流 MPC”。** 它是一个二维、线性、准静态、以水平磁力为控制量的滚动时域控制器：状态只有磁珠平面位置 `[x,y]`；优化变量是未来 N 步 `[Fx,Fy]`；预测模型为 `p_{k+1}=p_k+dt(F_k+d)/c`；输出第一步期望水平力。代码位置：`dipole-control/mpc.py:32-140`，实际调用：`dipole-control/multirate.py:257-317`。

**[CONFIRMED] 项目实际控制层级属于方案 B：**

```text
MPC -> desired horizontal force [Fx,Fy]
    -> append independently computed Fz_lift
    -> prescribed desired B + desired 3-D force
    -> position-dependent [B;F] = A I allocation
    -> six target currents [A]
    -> 30 Hz interpolation / slew limit / integer quantization
    -> six signed serial commands
```

它不是 `MPC -> six currents`，也不是 `MPC -> desired velocity -> lower controller`。MPC 内部使用参考速度，但速度由准静态关系 `v=(F+d)/c` 代数计算；实测速度不是 MPC 状态。

**[CONFIRMED] “多磁偶极子”指每个电磁铁由多个等效磁偶极子表示。** 六个电磁铁各 30 个，共 180 个。每个偶极子的 3-D 位置和 1 A 响应磁矩向量直接存储在 `models/N30LM_six_coils_180dipoles.json`。所以属于题目中的 **B：每个电磁铁 ≈ 多个磁偶极子**，不是机器人由多个偶极子离散化。证据：`dipole-control/models/N30LM_six_coils_180dipoles.json:1-24`、`dipole-control/dipole_solver.py:86-206`。

**[CONFIRMED] 磁模型完整计算 `B=[Bx,By,Bz]`、3×3 空间梯度和 `F=[Fx,Fy,Fz]`，但机器人运动与 MPC 被二维化。** 所有实时位置传给磁模型时都使用 `[x,y,0]`；MPC 不含 z、vz 或接触状态；Fz 不是 MPC 优化量，而是可选“减摩抬升力”。因此代码是“3-D 磁场/磁力 + 2-D 运动/MPC”。

**[CONFIRMED] 机器人使用硬磁永磁球模型，而不是软磁球模型。** 磁矩大小由 `m_b=(Br/mu0)V` 给出，并假设磁矩瞬时沿局部磁场排列，因此使用 `F=m_b grad|B|`。没有实现软磁常见的 `F proportional to grad|B|^2`，也没有在控制中计算 `tau=m cross B`；仅诊断磁矩对齐时间。证据：`dipole-control/dipole_solver.py:160-163,304-325`。

**[CONFIRMED] COMSOL 不直接进入 MPC。** 当前项目没有 COMSOL 场图、查表或插值。COMSOL 相关代码只用一个 JSON 中的六个标量锚点，检查每个单线圈 1 A 在原点的 `|B|=11.6 mT`。180 个偶极子参数如何得到，源码没有拟合/导出程序，也没有 provenance 字段，因此其来源是 **[UNKNOWN]**。

**[CONFIRMED] Python 侧已有相机、MPC、分配器和串口发送接口，但与目录中旧 STM32 固件存在明确协议断点。** 新代码发送 43 字节带符号帧，例如 `a0:+30,...`；旧固件只按固定偏移解析 37 字节两位无符号帧 `a0:30,...`。因此旧固件无法正确解析新帧，负电流也无法实现。证据：`dipole-control/magnetic_dipole_pid.py:72-83` 对比 `pwm_double20230915DC6output/Core/Src/main.c:43-46,139-145`。

**[CONFIRMED] 项目包含 learning，但不是 MPC residual learning。** `rl_policy.py` 是一套与 MPC 并行的 ES-MLP 直接六命令控制器；没有 GP、神经网络残差动力学、能力图或 execution-risk 模型进入 MPC。

## 2. Project structure

### 2.1 完整目录枚举

```text
dipole-control/
├── README.md
├── requirements.txt
├── config.py
├── magnetic_dipole_pid.py
├── mpc.py
├── multirate.py
├── dipole_solver.py
├── estimators.py
├── friction_model.py
├── bead_sim.py
├── bead_sim_gui.py
├── quick_field_check.py
├── smoke_test.py
├── rl_policy.py
├── train_rl.py
├── rl_path_control.py
├── gui_settings.json
├── rl_gui_settings.json
├── models/
│   ├── N30LM_six_coils_180dipoles.json
│   ├── comsol_reference.json
│   └── rl_policy.json
├── tests/
│   ├── test_solver.py
│   ├── test_control.py
│   ├── test_multirate.py
│   ├── test_bead_sim.py
│   ├── test_rl.py
│   └── __pycache__/                 # 5 个对应测试的 .pyc
├── tmp/pdfs/octomag/
│   ├── page-03.png
│   ├── page-04.png
│   ├── page-05.png
│   ├── page-06.png
│   └── page-07.png
├── __pycache__/                     # 14 个源码模块的 .pyc
├── MPC圆400pix.csv
├── MPC圆400pix_mpc10hz.csv
├── experiment.csv
├── pid圆400pix.csv
├── pid圆400pix2.csv
└── pid矩形700-450.csv
```

### 2.2 重要文件分类

| 文件/目录 | 分类 | 是否进入当前 MPC 实机主链 | 作用 |
|---|---|---:|---|
| `magnetic_dipole_pid.py` | entry point / vision / reference / serial / visualization / glue | 是 | 主 GUI、相机检测、状态发布、MPC 模式选择、串口发送 |
| `multirate.py` | MPC orchestration / actuator execution | 是 | 10 Hz `ControlWorker` 与 30 Hz `CurrentExecutor` |
| `mpc.py` | MPC / optimizer / dynamics | 是 | 两个单轴 `ForceMPC`，输出 Fx、Fy |
| `dipole_solver.py` | magnetic model / dipole model / actuator allocation / optimizer | 是 | 180 偶极子正向模型与 `[B;F]->I` 分配 |
| `config.py` | configuration | 是 | 默认物理量、频率、约束、MPC 权重 |
| `gui_settings.json` | persisted configuration | 是 | 启动时覆盖多项默认值 |
| `estimators.py` | state input / estimator | 是 | `[x,y,vx,vy]` Kalman 与 ESO |
| `friction_model.py` | dynamics/compensation | 条件进入 | Fz lift、法向力、摩擦前馈；MPC 只接收 Fz 与 ESO d |
| `models/N30LM...json` | magnetic calibration/model data | 是 | 六线圈 × 30 偶极子参数 |
| `models/comsol_reference.json` | calibration/validation data | 否 | 六个原点单线圈场强锚点 |
| `bead_sim.py` | simulation / dynamics / calibration | 否 | 独立二维贴底仿真、参数扫描、eta/mu 拟合 |
| `bead_sim_gui.py` | simulation entry / visualization | 否 | 仿真 GUI |
| `quick_field_check.py` | utility / validation | 否 | 手动检查 B/G/梯度 |
| `smoke_test.py` | offline integration check | 否 | 无头 GUI/控制链脚本；本次未运行 |
| `rl_policy.py` | learning / simulation | 否，平行控制器 | ES-MLP 策略和训练环境 |
| `train_rl.py` | training entry | 否 | 离线训练策略 |
| `rl_path_control.py` | alternate entry / vision / serial | 否，平行控制器 | RL 直接六命令实机 GUI |
| `tests/` | static test definitions | 否 | 方程、约束、接口测试；本次未运行 |
| CSV 文件 | data/logs | 否 | 以前运行产生的数据；不参与控制 |
| `tmp/pdfs/octomag/*.png` | literature/reference assets | 否 | 源码没有引用 |

依赖只有 NumPy、OpenCV、pyserial 和 PySide6；没有 CasADi、IPOPT、OSQP、SciPy、cvxpy 或 acados（`dipole-control/requirements.txt:1-4`）。

## 3. Main entry point and call graph

### 3.1 主入口

**[CONFIRMED] 实机 PID/MPC 主入口：** `dipole-control/magnetic_dipole_pid.py:2824-2828`。

启动过程：

```text
QApplication
  -> MagneticDipoleControl.__init__
     -> build GUI
     -> load gui_settings.json
     -> DipoleSolver.from_json(...)
     -> start 33 ms QTimer
```

构造函数不会自动打开相机或串口；必须由用户操作 GUI。模型加载失败会禁用自动控制和手动力控制，但保留手动电流界面（`magnetic_dipole_pid.py:325-359`）。

### 3.2 MPC 实际调用链

```text
MagneticDipoleControl.tick()                         30 Hz
├── cap.read()
├── detect_bead()
├── estimate_state() -> pos[2], vel[2]
└── control_step()
    └── mpc_track_step()
        ├── optional ESO update -> z3[2]
        ├── SharedState.set_kalman/set_last_sent/set_params
        ├── CurrentExecutor.step()
        │   ├── read six I_target [A]
        │   ├── 3-frame interpolation
        │   ├── command slew/amplitude/quantization
        │   └── estimated coil-current + magnetic forward diagnostics
        └── send_commands() -> optional serial.write()

ControlWorker._step()                               10 Hz worker
├── read pos/path/params/last six commands
├── _reference_window()
├── ForceMPC.compute() for x
├── ForceMPC.compute() for y
├── append Fz_lift and rotate XY into model frame
├── DipoleSolver.solve_field_force_pseudoinverse()
└── publish six I_target [A]
```

关键位置：`magnetic_dipole_pid.py:1985-2077,2220-2223,2427-2506`；`multirate.py:156-359,368-450`。

### 3.3 核心 class/function

- `MagneticDipoleControl`：实机 GUI 和总调度。
- `detect_bead`：灰度/HSV、形态学、轮廓质心视觉检测。
- `KalmanFilter2D`：四状态视觉滤波器，但不是 MPC 模型。
- `ESO1D`：扰动力估计；可把 z3 传入 MPC。
- `SharedState`：30 Hz 主线程与 10 Hz worker 的锁保护快照。
- `ControlWorker._step`：MPC 到磁力分配器的主调用链。
- `ForceMPC.compute`：单轴 QP。
- `DipoleSolver._field_grad_coils`：180 偶极子单位电流 B/G。
- `DipoleSolver.forward_model/force_at`：电流到 B/G/F。
- `DipoleSolver.field_force_actuation_matrix`：构造 6×6 `[B;F]` 矩阵。
- `DipoleSolver.solve_field_force_pseudoinverse`：当前 GUI/MPC 使用的六电流分配器。
- `CurrentExecutor.step/update_est`：目标电流执行与开环电流估计。
- `build_command/send_commands`：最终串口协议。

## 4. Overall control architecture

**[CONFIRMED] 当前 MPC 主架构：**

```text
camera image
  -> 2-D bead centroid [px]
  -> camera world position [x,y] mm
  -> Kalman/EMA/RAW state estimate
  -> path projection + future 2-D position/tangent-speed reference
  -> 2-D force MPC (Fx,Fy)
  -> optional independent Fz_lift
  -> coordinate rotation/reflection R^T
  -> desired B vector + desired 3-D force
  -> 180-dipole field-force allocator
  -> six target currents [A]
  -> 30 Hz interpolation + slew + integer quantization
  -> signed a0..a5 serial frame
  -> [BREAK with supplied old STM32 parser]
  -> PWM/H bridge/six electromagnets
  -> robot motion
  -> camera
```

控制层级最接近题目中的 **方案 B**，但在分配器中附加了期望磁场：

```text
MPC -> desired force -> joint desired-field/force current allocation -> six currents
```

## 5. MPC formulation

### 5.1 状态、输入和预测模型

**[CONFIRMED] MPC state：**

```math
x_k = [p_{x,k}, p_{y,k}]^T
```

实现上 x/y 完全解耦，各使用一个 `ForceMPC`。内部单轴状态只是当前位置标量 `x0`。`z`、速度、加速度、磁矩方向、B、梯度、六电流和线圈动态均不是 MPC 状态。

**[CONFIRMED] MPC control input：**

```math
u_k = [F_{x,k}, F_{y,k}]^T \quad [\mu N]
```

**[CONFIRMED] 单轴预测模型：**

```math
c v_k = F_k + d
```

```math
p_{k+1} = p_k + \Delta t\,v_k
          = p_k + \frac{\Delta t}{c}(F_k+d)
```

其中 `c=6*pi*eta*r`，单位换成 `uN/(mm/s)`；`d` 是可选 ESO 广义扰动力，预测视界内视为常量。源码：`dipole-control/mpc.py:3-26,88-140`。

对 N 步力序列，可写为：

```math
P = p_0 1 + (\Delta t/c) L(F+d1)
```

其中 `L` 是下三角全 1 累加矩阵。代码在 `mpc.py:104-108` 构造这一关系。

### 5.2 Horizon、dt 和滚动方式

- **[CONFIRMED]** 标称 MPC 频率：10 Hz，`dt=0.1 s`（`config.py:101-106`；worker 默认周期 `multirate.py:161-167`）。
- **[CONFIRMED]** 源码默认 N=3，即 300 ms；GUI允许 1..6（`config.py:109-115`、`magnetic_dipole_pid.py:948-972`）。
- **[CONFIRMED]** 交付态 `gui_settings.json` 也是 N=3。
- **[CONFIRMED]** 每个 10 Hz 周期重新读取位置、重新把位置投影到路径、生成参考、求完整序列但只执行第一项 `F0`，所以是 receding horizon（`multirate.py:196-238,257-317`）。

### 5.3 参考生成

当前实际位置投影到最近路径线段弧长 `s_near`，首参考点从 `s_near+v_ref*dt` 开始，后续按 `v_ref*dt` 前推；每一点附局部切向速度。到开放路径末端后参考速度变为零。源码：`multirate.py:196-238`。

## 6. State definition

| 层 | 状态/量 | 是否进入 MPC 动态状态 |
|---|---|---:|
| 相机测量 | `[x,y]` | 是，作为位置初值 |
| Kalman | `[x,y,vx,vy]` | 只有 `[x,y]` 进入 |
| ESO | 每轴 `[z1,z2,z3]` | 只有 `z3=d` 作为外部参数 |
| 机器人 z | 固定 0 | 否 |
| 六线圈电流估计 | `I_est[6]` | 否 |
| 磁矩方向 | 由 B 瞬时决定 | 否 |

**[CONFIRMED]** `ControlWorker._step` 读取了 Kalman `vel`，但未把它传给 `ForceMPC`，也没有在该函数的 MPC 计算中使用（`multirate.py:268-291`）。因此不能把“估计器是四状态”误写成“MPC 是四状态”。

## 7. Control-input definition

MPC 变量是未来各步水平磁力序列：

```math
U_x=[F_{x,0},...,F_{x,N-1}]^T,
U_y=[F_{y,0},...,F_{y,N-1}]^T.
```

Fz 不参与优化。worker 在 MPC 之后构造：

```math
F_{target}=[F_{x,0},F_{y,0},F_{z,lift}]^T.
```

代码：`multirate.py:281-300`。

## 8. Cost function

每轴独立代价为：

```math
J = \sum_{k=0}^{N-1}
    w_p(p_{k+1}-p^r_{k+1})^2
  + w_v((F_k+d)/c-v^r_k)^2
  + w_u F_k^2
  + w_{\Delta}(F_k-F_{k-1})^2.
```

第一步变化使用 `F_{-1}=F_prev`。`F_prev` 由当前位置和上一帧实际发送的六个整数命令经磁模型计算，而不是上一 MPC 理想目标（`multirate.py:273-279`）。

各项含义：

| 项 | 状态 | 说明 |
|---|---|---|
| position tracking | [CONFIRMED] | 未来位置对路径位置参考 |
| velocity tracking | [CONFIRMED] | 代数速度 `(F+d)/c` 对路径切向速度 |
| control effort | [CONFIRMED] | 力平方，不是电流平方 |
| force-rate smoothing | [CONFIRMED] | `Delta F` 平方；是软代价，不是硬 slew constraint |
| terminal cost | [CONFIRMED] 无独立项 | 最后位置仅使用同一 stage weight |
| current magnitude/rate | [CONFIRMED] 不在 MPC | 下游分配/执行约束 |
| obstacle/collision | [CONFIRMED] 无 | 未实现 |
| Fz penalty | [CONFIRMED] 无 | Fz 不在 MPC |

权重有两套需要区分：

| 来源 | N | Wpos | Wvel | Wu | Wdu | Fxy max |
|---|---:|---:|---:|---:|---:|---:|
| `config.py` 默认 | 3 | 1.0 | 2.0 | 0.005 | 0.01 | 40 uN |
| 交付态 `gui_settings.json` | 3 | 40.0 | 0.8 | 0.02 | 0.001 | 50 uN |

GUI 启动时读取持久化设置，因此第二行是当前交付文件存在时的有效值。位置：`config.py:109-115`；`gui_settings.json:41-48`；发布：`magnetic_dipole_pid.py:2481-2506`。

## 9. Constraints

### 9.1 MPC 内部

- **[CONFIRMED]** 单轴箱约束：`|Fx_k|<=Fmax`、`|Fy_k|<=Fmax`。
- **[CONFIRMED]** 取第一步后再施加圆形水平力约束 `sqrt(Fx^2+Fy^2)<=Fmax`（`multirate.py:295-298`）。
- **[CONFIRMED]** 无速度、加速度、workspace、z、Fz、B、gradient、current、collision 或 terminal hard constraint。
- **[CONFIRMED]** `Delta F` 只有代价，没有硬力变化率约束。

### 9.2 MPC 之外的执行约束

- 六电流/命令幅值：运行时 `|cmd_i|<=max_cmd`。
- 每通道命令变化：`|cmd_i-cmd_prev_i|<=9`。
- 连续电流与命令关系：默认 `I=cmd*(2/99)`；交付设置为 `I=cmd*0.02`。
- 整数命令量化。
- Fz lift 由 `[0,Fz_max]` 静态裁剪；不构成 z 动力学约束。
- 期望 B 和 F 由加权最小二乘同时逼近；最终 5% B 向量误差、2% F 误差用于 `converged` 判定，不是 MPC 可行域。

## 10. Solver

**[CONFIRMED] MPC 核心是线性模型下的凸箱约束 QP。** 代码自行枚举每个预测步的三种活动状态 `{-Fmax, free, +Fmax}`，即每轴 `3^N` 个活动集；自由变量用 `numpy.linalg.solve`，奇异时用 `pinv`。N 默认 3 时每轴 27 组；GUI限制 N<=6。源码：`mpc.py:48-75,118-140`。

所以它不是 CasADi/IPOPT/OSQP/cvxpy/acados/scipy 优化器，也不是 NLP。整个闭环包含非线性磁力正向模型，但 **MPC 子问题本身是线性 MPC/QP**。

下游 `[B;F]` 分配也是自写 NumPy 求解：无约束时 Moore-Penrose 伪逆，有约束时枚举六电流 `3^6=729` 个活动集求箱约束线性最小二乘，再搜索整数命令邻域（`dipole_solver.py:991-1121`）。

## 11. Multi-dipole magnetic model

### 11.1 “多磁偶极子”的含义

**[CONFIRMED] 每个真实/模型电磁铁用 30 个等效偶极子表示，总数 180。**

| 问题 | 结论 |
|---|---|
| 一共有多少 dipole | [CONFIRMED] 180 = 6×30 |
| 每个位置在哪里 | [CONFIRMED] JSON 中各 coil 的 `pos[30][3]`，单位 mm，workspace 坐标 |
| 每个方向在哪里 | [CONFIRMED] `mom[30][3]` 向量方向；每个偶极子可不同 |
| moment 大小 | [CONFIRMED] `mom` 向量模，JSON 单位 `mT*mm^3 @ 1A`；加载乘 `1e-5` 成 `A*m^2` |
| 与 current 是否正比 | [CONFIRMED] 是，模型直接用 `I_j*m_unit` |
| saturation/nonlinearity | [CONFIRMED] 没有 moment-current 饱和；力因 B 方向归一化而对 I 非线性/偶对称 |
| 是否考虑铁芯 | [UNKNOWN] 无显式铁芯材料/磁导/饱和；可能隐含于等效拟合，但源码不能证明 |
| 参数来源 | [UNKNOWN] JSON 无 provenance，拟合脚本/原始数据不在项目中 |
| 六线圈叠加 | [CONFIRMED] B 和 G 简单线性叠加 |
| magnetic coupling | [CONFIRMED] 无显式线圈间耦合/互感/共同铁路模型 |

JSON 明确注明每个 `mom` 是 1 A 线圈响应、位置与磁矩在 workspace 坐标中（`models/N30LM_six_coils_180dipoles.json:1-24`）。实际数组从第 25 行开始。

### 11.2 通道顺序

**[CONFIRMED] 模型和新串口顺序固定为：**

```text
a0..a5 = [+X, +Y, +Z, -X, -Y, -Z]
```

对置索引是 `a0<->a3`、`a1<->a4`、`a2<->a5`。若外部继续采用旧映射 `[1,3,5,4,6,2]`，这些索引对恰好对应物理 `1<->4`、`3<->6`、`5<->2`；这是 **[INFERRED] 条件一致性**，新源码本身没有 pole 1..6 映射，不能据此确认实际接线。

## 12. Magnetic field equations

对第 j 个线圈的第 q 个偶极子：

```math
R_{jq}=r-p_{jq}, \quad m_{jq}(I_j)=I_j m^{1A}_{jq}.
```

单偶极子磁场：

```math
B_{jq}(r)=\frac{\mu_0}{4\pi}
\left[
\frac{3R_{jq}(m_{jq}\cdot R_{jq})}{|R_{jq}|^5}
-\frac{m_{jq}}{|R_{jq}|^3}
\right].
```

总场：

```math
B(r,I)=\sum_{j=1}^{6}\sum_{q=1}^{30} B_{jq}(r,I_j)
      =\sum_{j=1}^{6} I_j B^{1A}_j(r).
```

解析梯度张量 `G_{kl}=partial B_k/partial x_l` 使用：

```math
G_{kl}=\frac{\mu_0}{4\pi}\left[
\frac{3\delta_{kl}(m\cdot R)}{r^5}
+\frac{3(R_km_l+m_kR_l)}{r^5}
-\frac{15R_kR_l(m\cdot R)}{r^7}
\right].
```

代码完整实现并对 30 个偶极子按线圈求和：`dipole_solver.py:256-302`。场点与偶极子重合时用 1 um 距离下限作数值保护。

## 13. Magnetic force/torque model

磁珠被建模为硬磁球形永磁体：

```math
m_b=\frac{B_r}{\mu_0}\frac{4}{3}\pi r_b^3.
```

假设磁矩瞬时沿局部场：

```math
m_b vector = m_b\frac{B}{|B|}.
```

于是：

```math
F=\nabla(m_b vector\cdot B)=m_b\nabla|B|
 =m_b\frac{B^T G}{\sqrt{B^TB+epsilon^2}}.
```

源码：`dipole_solver.py:160-163,304-325,341-345`。

- **[CONFIRMED]** 不是软磁 `F proportional to grad|B|^2` 模型。
- **[CONFIRMED]** 控制中没有计算 `tau=m cross B`。
- **[INFERRED]** 在“完全对齐”的名义假设下 `m cross B=0`；瞬态转矩过程被省略。
- **[CONFIRMED]** `CurrentExecutor` 只计算 `tau_align=zeta_r/(m_b|B|)` 作为诊断，不把姿态加入状态或动力学（`multirate.py:419-435`）。

## 14. Robot dynamics model

### 14.1 MPC 中的真实模型

**[CONFIRMED]** 只有过阻尼 Stokes 准静态关系：

```math
c=6\pi\eta r,
\quad v=(F+d)/c,
\quad p_{k+1}=p_k+dt\,v.
```

MPC 中没有：

- mass 或 inertia；
- acceleration state；
- gravity/buoyancy；
- bottom-contact state；
- Coulomb friction；
- z motion；
- Fz 对 xy 摩擦的显式耦合。

可选 ESO 的 d 可以吸收某些未建模效应，但它只是二维常量扰动力输入，并不等同于显式接触模型。

### 14.2 其他模块中的动力学

- `friction_model.py` 假设贴底，计算扣除浮力的有效重量、Fz 减压后的法向力和经验摩擦前馈；只有部分输出进入 MPC。
- `bead_sim.py` 是独立仿真器：固定 z=0，使用 Stokes drag + 静/动 Coulomb 摩擦，忽略惯性；运动只使用 Fx/Fy（`bead_sim.py:96-192`）。
- 仿真器法向力使用 `N=mg`，没有扣浮力且不使用磁 Fz；实时 friction 模块则使用浮力和 Fz。两处假设不一致。

## 15. Treatment of z/Fz

| 问题 | 结论 |
|---|---|
| B 是否完整 3-D | [CONFIRMED] 是，Bx/By/Bz |
| gradient 是否含 z | [CONFIRMED] 是，完整 3×3 G |
| force 是否含 Fz | [CONFIRMED] 是，Fx/Fy/Fz |
| robot state 是否含 z | [CONFIRMED] 否 |
| MPC 是否控制 z | [CONFIRMED] 否 |
| z 是否固定 | [CONFIRMED] 所有实时磁模型查询使用 z=0 |
| 系统是否二维 | [CONFIRMED] 视觉、路径、运动预测均二维 |
| Fz 是否完全忽略 | [CONFIRMED] 否；可作为外部 lift 命令进入电流分配，但不反馈 z |

二维化的关键位置：

- `multirate.py:273`：`pos_m=[x,y,0]*1e-3`。
- `multirate.py:288-294`：只对 x/y 求 MPC，再追加 Fz。
- `magnetic_dipole_pid.py:2481-2506`：Fz 由法向力比例公式发布。
- `bead_sim.py:119-137`：仿真位置只有二维，查询场时加 z=0。

**[CONFIRMED]** 没有自由悬浮模型，也没有显式“始终贴底”状态机进入 MPC。更准确的描述是：运动学被固定在 z=0 平面，同时可命令一个旨在减小接触压力的 Fz。

## 16. COMSOL relationship

当前代码关系是：

```text
unknown external model/calibration process
  -> 180-dipole JSON
  -> analytic dipole B/G/F model
  -> MPC downstream allocation

COMSOL six scalar origin values
  -> optional validation function only
```

`models/comsol_reference.json` 只有六个 case：每个线圈 1 A、点 `(0,0,0)`、场强标量 11.6 mT（`comsol_reference.json:1-10`）。`validate_against_comsol_reference` 逐点比较模型输出（`dipole_solver.py:2133-2183`）。

- **[CONFIRMED]** COMSOL 不在实时调用链。
- **[CONFIRMED]** 无 field map、lookup table、interpolation 或 polynomial evaluator。
- **[UNKNOWN]** 180 偶极子是否来自 COMSOL 拟合、手工设置或实验拟合。README/注释称其为“标定模型”，但生成程序和原始数据缺失。
- **[UNKNOWN]** 铁芯效应是否已经被等效偶极参数隐式吸收。

## 17. Six-coil mapping and units

### 17.1 输出量与单位链

```text
ForceMPC output: Fx,Fy                    [uN]
worker F_target: Fx,Fy,Fz                 [uN]
allocator input: F_des                    [N]
allocator continuous output: I[6]         [A]
quantized output: command[6]              signed integer
serial output: a0..a5                     protocol units
```

模型 JSON 单位：position mm，moment `mT*mm^3 @ 1 A`，current A。内部场模型使用 SI。

### 17.2 电流范围

- 源码默认：±99 ↔ ±2 A，`2/99=0.020202... A/command`。
- GUI默认安全上限：±50 command。
- 交付 `gui_settings.json` 保存 gain=0.02 A/command，因此按该设置 ±99 ↔ ±1.98 A。
- 电流允许正负；负号用于反转等效偶极矩/场方向。

### 17.3 Pole mapping

**[UNKNOWN]** 新代码没有 pole 1..6 标识，因此无法从新源码直接确认是否仍是 `[1,3,5,4,6,2]`。它只强制 `[+X,+Y,+Z,-X,-Y,-Z]`。

**[INFERRED]** 若旧数组顺序仍用于实际接线，则对置关系在索引上相容；但必须由接线表、师兄说明或逐路线圈实验确认。

## 18. Hardware / serial interface

### 18.1 Python 侧

**[CONFIRMED] 已实现并接线：**

- pyserial 端口选择和 115200 baud；
- 连接时发送零帧；
- 所有模式统一经过幅值和斜率安全层；
- 固定 43 字节 ASCII 帧；
- MPC 30 Hz 执行层最终调用 `send_commands`。

格式：

```text
a0:+30,a1:-45,a2:+60,a3:-75,a4:+90,a5:-10\r\n
```

### 18.2 与旧 STM32 的断点

**[CONFIRMED] 当前新 Python 与提供的旧固件不兼容。**

旧固件：

```c
#define FRAME_LEN 37
#define PARSE(buf, idx) (((buf)[idx]-'0')*10 + ((buf)[idx+1]-'0'))
```

并固定从 3、9、15、21、27、33 取两位数字。新帧在索引 3 是 `'+'` 或 `'-'`，且每个字段多一字符，导致首通道解析错误、后续偏移全部错位。旧固件还把值 clamp 到 0..100，无法表达负命令（`pwm_double20230915DC6output/Core/Src/main.c:43-46,139-145`）。

因此当前真实链路是：

```text
new MPC software -> signed 43-byte serial frame -> X old 37-byte unsigned parser
```

**[UNKNOWN]** 实验板是否已烧录另一版支持带符号 43 字节协议的固件；本目录没有这版源码，本次也未连接硬件验证。

### 18.3 电流反馈

**[CONFIRMED]** Python 中的 `I_est` 来自一阶 RL 线圈模型，不是 ADC 测量：

```math
I_{est,k+1}=e^{-Rdt/L}I_{est,k}+(1-e^{-Rdt/L})I_{steady,k}.
```

没有 CURT、六路 current PI 或实测电流闭环。PWM 20 kHz 只是 Python 配置/记录常数，实际 PWM 在 STM32。

## 19. Comparison with old PID system

| 层 | 旧 `260426-1.py` | 新 MPC 主链 |
|---|---|---|
| 测量 | 像素 x,y | 像素 -> mm；Kalman/EMA/RAW |
| 控制状态 | 像素位置误差、差分速度 | MPC 位置 `[x,y]`；ESO d 可选 |
| 控制器 | PID 输出 `ux,uy` | 线性 force MPC 输出 Fx,Fy |
| 执行器选择 | 与固定方向投影，argmax | 位置相关 180-dipole `[B;F]` 分配 |
| 同时工作通道 | 主要一个非负通道 | 六路可同时、可正负 |
| 物理量 | 0..90 无量纲命令 | 名义上 uN -> A -> signed command |
| 磁场模型 | 无 | 180 偶极子 3-D B/G/F |
| 动力学 | 无显式模型 | `v=(F+d)/c` 准静态模型 |
| 约束 | 命令幅值/变化 | MPC 力界 + 下游电流/命令界与 slew |
| 串口 | 37 字节无符号 | 43 字节带符号 |

旧代码证据：`260426-1.py:273-368`。

## 20. Current implementation completeness

### 20.1 真正实现并进入主链

- 相机二维检测与世界尺度转换；
- Kalman/EMA/RAW 状态估计；
- 路径生成、重采样、实际位置投影和切向参考；
- 10 Hz 线性 force MPC；
- 180 偶极子 B/G/F 正向模型；
- 指定 B + 目标 F 的六电流分配；
- 30 Hz 目标电流插值、幅值/斜率/量化安全层；
- Python 串口写出；
- 开环线圈电流动态估计和模型诊断；
- GUI 和 CSV 日志。

### 20.2 条件启用

- ESO 扰动估计；
- Fz lift；
- 单线圈视觉方向标定；
- PID 模式的摩擦/阻力/速度前馈；
- 相机、串口、保存日志。

### 20.3 Simulation-only / offline

- `bead_sim.py` / `bead_sim_gui.py`；
- 参数扫描和 eta/mu 轨迹拟合；
- `quick_field_check.py`；
- `smoke_test.py`；
- `train_rl.py` 和 RL 训练环境；
- 单元测试。

### 20.4 保留但不在当前 GUI/MPC 主链

- nonlinear force-only `solve_currents/solve_commands`；
- `solve_force_with_field_magnitude`；
- `solve_force_and_field`；
- sparse 1/3-coil 求解路径；自动追踪会强制六路自由；
- 数值梯度和数值电流 Jacobian；
- `synthetic_ring`。

### 20.5 数据能证明什么

`MPC圆400pix_mpc10hz.csv` 有 516 条 10 Hz 记录，说明 MPC/allocator 软件路径以前运行过。配套 `MPC圆400pix.csv` 只有表头，因为当前 MPC 分支没有追加 PID 使用的 30 Hz完整记录。CSV 不能证明使用了哪版固件、串口成功、真实电流或真实 F/B；这些值主要来自命令与模型。

## 21. Interfaces relevant to future learning

### 21.1 当前纯 nominal physics 部分

- 180 偶极子解析 B/G；
- 硬磁球 `m_b=(Br/mu0)V`；
- 瞬时对齐下 `F=m_b grad|B|`；
- Stokes drag `c=6*pi*eta*r`；
- 一阶线圈 RL 估计；
- `[B;F]=A I` 固定期望磁矩方向分配。

### 21.2 明显经验/配置部分

- 180 个偶极子的所有位置和磁矩；
- current_gain；
- 交付设置中的 Br=0.35 T；
- viscosity、mu_s、mu_d；
- 2-D `force_model_to_camera`；
- ESO 带宽/限幅；
- MPC 权重；
- Fz normal-ratio 与上下限。

### 21.3 可能的 model mismatch 来源

- 等效偶极参数来源不可追溯；
- 线性 moment-current 假设，无铁芯饱和/磁滞/耦合；
- 命令到实际电流完全开环；
- 瞬时磁矩对齐；
- 固定 z=0，无显式接触切换；
- MPC 无摩擦，只有 Stokes drag + 可选 ESO；
- Fz 对接触/xy 的作用不进入 MPC；
- 分配器使用目标 B 方向固定磁矩的线性力矩阵，最终实际力才用非线性自由对齐模型重算；
- camera/model 只做一个全局 2-D 正交变换；
- 实际交付 Br、gain、B方向与 README/config 默认不一致。

### 21.4 已建模与可作为 residual 的量

已完整给出 nominal 接口：

```text
(position_3D, six currents) -> B, G, grad|B|, force_3D
```

最自然的 residual 插入点之一是 `DipoleSolver.forward_model` 之后：

```math
F_real = F_nominal(p,I) + Delta F_learned(p,I,I_history,...).
```

另一自然位置是 MPC 动力学：

```math
v_real = (F+d)/c + Delta v_learned(p,v,F,...).
```

前者修正磁/执行器模型，后者可吸收摩擦、近壁效应和接触等执行动力学。当前代码均未实现。

### 21.5 intrinsic actuation capability 接口

`DipoleSolver.field_force_actuation_matrix(pos,B_direction)` 已给出位置和期望方向相关的 `A_BF`；分配器还返回 rank、condition、singular values、约束是否激活、实际力误差。它是研究位置/方向能力图的直接 nominal interface。当前没有学习或持久化 capability map。

### 21.6 execution-aware planning 接口

最自然接口在：

1. `ControlWorker._reference_window` 生成未来参考之前/之后；
2. `ForceMPC.compute` 的位置/速度参考与 cost；
3. `solve_field_force_pseudoinverse` 返回的可达性、condition 和实际残差。

当前 MPC 两轴独立且只支持二次 stage cost。如果 execution risk 依赖二维位置、方向、历史或约束可达域，现有接口可提供数据，但当前自写单轴 QP 不能直接表达一般空间耦合风险项。

### 21.7 当前 learning 内容

**[CONFIRMED]** 已有 ES-MLP：10 维状态到 6 维命令增量，训练环境使用 nominal dipole force + Stokes dynamics + 5% 力噪声（`rl_policy.py:46-204`）。它是平行的直接策略，不是 residual、GP 或 execution-aware MPC。

## 22. Assumptions and unresolved questions

### Potential issues / questions to verify

1. **[CONFIRMED issue] 串口协议不兼容。** 实验板是否已经更换为支持 43 字节 signed frame 的固件？若没有，新控制链到 STM32 处中断。
2. **[UNKNOWN] 新模型通道与物理 pole 1..6 的精确接线表是什么？** 新源码没有 `[1,3,5,4,6,2]`。
3. **[UNKNOWN] 180 偶极子 JSON 如何产生？** 需要 COMSOL 拟合脚本、目标函数、训练点、误差地图和版本信息。
4. **[CONFIRMED limitation] COMSOL 参考仅有原点六个标量 `|B|`。** 无法由此验证方向、梯度、工作区空间误差或力。
5. **[UNKNOWN] 铁芯饱和、磁滞和线圈耦合是否已隐含在拟合中？** 源码没有显式模型。
6. **[CONFIRMED setting mismatch] 为什么交付 `Br=0.35 T`，而 README/config 的 N38 为 1.20 T？** 这会把假设磁矩缩小到约 29.2%。
7. **[CONFIRMED setting mismatch] 为什么交付 current gain=0.02，而源码默认 2/99？** 需要确认实际安培标定和正负方向。
8. **[CONFIRMED setting mismatch] 交付目标 B 是 +Z，源码默认是 -Z。** 需要确认物理 z 正方向和期望姿态。
9. **[CONFIRMED] `force_model_to_camera` 当前是 identity 且未标定。** 逐路实验是否做过、物理接线是否可能需要 permutation，而不是单一旋转/镜像？
10. **[UNKNOWN] 机器人通电运动时是否始终贴底。** 当前 MPC 无 z/contact 状态，Fz 仅作为静态减摩命令。
11. **[CONFIRMED simplification] MPC 不显式考虑摩擦、重力、浮力或 Fz-xy 耦合。** ESO 是否足够吸收这些效应需要实验验证。
12. **[CONFIRMED equation mismatch] ESO 注释/实现基于 `xddot=(F+d)/c`，MPC 基于 `xdot=(F+d)/c`。** 需要师兄确认希望使用的一致模型。
13. **[CONFIRMED] Kalman velocity未进入 MPC 状态。** 这是设计选择还是遗漏需确认。
14. **[CONFIRMED] `Delta F` 的起点由 last command 的静态磁模型计算，不使用 RL `I_est`。** 线圈动态显著时应验证该差异。
15. **[CONFIRMED] 磁矩瞬时对齐只做诊断。** 需要比较 `tau_align` 与 30/10 Hz 周期及真实旋转阻力。
16. **[UNKNOWN] 硅油黏度、底面摩擦和近壁修正是否实验标定。** 当前多个值是配置/占位。
17. **[CONFIRMED] 没有真实电流反馈。** 日志中的 A、B、F 主要为命令换算或模型估计。
18. **[CONFIRMED] 自动 MPC/PID 跟踪强制六路自由，稀疏 1/3 路代码不在当前主链。** README相关描述应按调用链理解。
19. **[CONFIRMED] MPC 30 Hz主 CSV当前不写数据，只有10 Hz专用日志。** 若用于 residual learning，需要补齐同步测量，但本轮未修改。
20. **[UNKNOWN] 随附 CSV 是仿真、无串口 dry-run 还是真实硬件实验。** 文件自身不足以确认。
21. **[PENDING VERIFICATION]** 静态测试、smoke test、模型数值、性能频率和求解收敛本轮均未执行，遵守只读/不运行约束。

## 23. Short final summary: “这个 MPC 到底是什么”

这个 MPC 是一个 **10 Hz、二维位置状态、二维磁力输入、Stokes 过阻尼预测模型、N 步滚动时域、箱约束 QP**。它预测未来 `[x,y]`，优化 `[Fx,Fy]` 的位置跟踪、切向速度跟踪、力幅值和力变化；默认 N=3，但当前交付设置把权重改为 `40/0.8/0.02/0.001`、水平力上限 50 uN。

MPC 本身不懂六线圈，也不优化电流。第一步目标力与独立 Fz lift 组合成 3-D 力，再与 GUI 指定的 3-D 目标磁场一起进入 180 偶极子模型构造的 6×6 `[B;F]=A I` 分配器，得到六路电流，最终量化成带符号的 a0..a5 命令。

磁模型是“每个电磁铁 30 个等效偶极子”的解析叠加模型；它完整计算 3-D B/G/F，使用硬磁 N38 球、磁矩瞬时随 B 对齐和 `F=m_b grad|B|`。运动/MPC仍固定 z=0，Fz 只用于减摩，没有 z 状态或接触动力学。

软件内部从相机到六电流已经接通，但和当前目录中的旧 STM32 固件没有接通：新 Python 是 43 字节 signed 协议，旧固件是 37 字节 unsigned 固定偏移解析。在确认/更换固件协议与物理通道映射之前，不能把当前代码视为已经完成真实六磁极端到端闭环。

```text
top camera
  -> bead centroid (px)
  -> [x,y] mm + Kalman
  -> path projection / future reference
  -> ForceMPC: predict [x,y], optimize [Fx,Fy]
  -> + optional Fz_lift
  -> 180-dipole 3-D B/G/F model
  -> [B_des;F_des] = A(position,B_direction) * I
  -> six target currents [A]
  -> interpolation / limit / quantization
  -> a0..a5 signed 43-byte serial frame
  -> X supplied legacy STM32 expects unsigned 37-byte frame
  -> PWM / H bridges / six coils               [currently unresolved]
  -> magnetic robot
  -> camera feedback
```
