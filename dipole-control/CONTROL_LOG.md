# 统一控制日志

GUI 的“控制日志”页 → 选择**新的** CSV 名称 → “开始记录” → 按原流程实验 → “停止记录”。默认不记录，不改变自动追踪、手动控制或停止流程。停止后后台排空队列，状态变成“已停止”后可开始下一次记录；必须选择新文件名。

选 `run.csv` 时，一个会话产生：

- `run.csv`：每个完成的主循环 tick 一行，约 30 Hz，104 列。
- `run.worker.csv`：每次完成的 MPC/solver 诊断或 STALE_INPUT / PATH_DEVIATION 跳过事件一行，约 10 Hz，55 列；原有手动导出 MPC CSV 的格式和内容保留。
- `run.metadata.json`：开始记录时 GUI/config、MPC 权重、物理参数、R、B 目标、路径、模式、command/current 限制；Git HEAD 和 tracked dirty 标志由写盘线程读取。`R_force_model_to_camera` 是 model XY → camera world XY 的正交映射；`offset.position_mm` 保存当前 `frame_offset_mm` 的 XY 与固定 z=0，来源为 `config.FRAME_OFFSET_MM`，含义是模型原点在相机世界坐标中的位置（默认 `[0,0]`，尚未实机标定）。ADC offset 仍为 `0/default`，原有像素中心也保留；freshness 门槛由现有 config 快照记录。
- `run.summary.json`：正常结束时两个流分别的 attempted / accepted / written / dropped / rejected 和日志错误。

四个文件均用 `x` 模式创建；任何一个已经存在都会报日志错误，不覆盖。CSV/JSON 写入、flush、Git 查询只在独立 daemon 线程执行。默认队列 2048 行，主线程和 worker 只提交标量快照，使用 `put_nowait`；队列满只增加对应流 dropped，之后可以恢复。目标关联缓存另有 128 项上限。

## 30 Hz 字段表

| 分组 | 字段 | 类型、单位与含义 |
|---|---|---|
| 时间 | `t_mono`, `t_wall`, `tick` | float 秒（monotonic 主时间轴）、float Unix 秒（人工换算 wall-clock）、int GUI 生命周期 tick 序号 |
| 时间/性能 | `dt_ms`, `jitter_ms`, `total_ms`, `vision_ms`, `estimator_ms`, `controller_ms`, `serial_ms`, `dropped` | float ms；dropped 为 int、两个流累计溢出数。dt 沿用原控制的 wall-clock/clamped dt；jitter 沿用原有绝对偏差；total 沿用 render/status/log capture **之前**的计时；serial_ms 是最近一次 send_commands 耗时，无发送时保持 |
| 状态 | `mode`, `tracking` | string 模式、bool 追踪状态 |
| Freshness | `kalman_age_ms`, `target_age_ms`, `stale_status` | float ms / string；全部 age 基于内部 monotonic。输入尚未发布时 kalman age 留空；target age 是本 tick executor 检查的目标年龄，executor 未运行时取最新发布目标。发送/heartbeat 不刷新 age；seq 仍表示实际采用的目标 |
| 视觉 | `frame_ok`, `detected`, `px_x`, `px_y`, `area_px` | bool 帧可用/检出，float 像素/像素面积；直接使用当前检测结果 |
| 测量 | `meas_x_mm`, `meas_y_mm` | float mm；直接复制 estimate_state 已转换的 z，失检留空，不再转换坐标 |
| 估计 | `kf_x`, `kf_y`, `kf_vx`, `kf_vy`, `estimator_mode` | float mm / mm/s，string RAW/EMA/KALMAN；kf 字段仅 KALMAN 模式填入 |
| 当前估计器输出 | `state_x_mm`, `state_y_mm`, `state_vx_mm_s`, `state_vy_mm_s` | float mm / mm/s；记录 RAW/EMA/KALMAN 实际供控制使用的输出 |
| ESO | `eso_mode`, `eso_updated`, `d_hat_x`, `d_hat_y`, `u_eso_x`, `u_eso_y` | string first_order/legacy/off、bool 本 tick 是否 step、float µN；u 是上一执行帧的 camera-world R-L/MDM 估计力（last_F_actual），不是未来 F_target，也不是 CURT 测量。未 step 留空；d_hat 是统一扰动力向量，新 ESO 来自 z2，legacy 来自 z3。恢复帧 step 可能只重置位置并清零扰动 |
| MPC 用力中心 | `effort_mode`, `F_ss_x`, `F_ss_y` | string absolute/steady_state、float µN。F_ss=c*v_ref−d_used 是未截断的必要稳态力；按执行器采用的 run/seq 与 F/I/ref 关联。没有可用目标时 F_ss 留空，effort_mode 为当前 GUI 选择；实际 target 已存在时优先记录该目标使用的模式 |
| 路径 | `target_idx`, `s_progress`, `s_total`, `finished`, `path_deviation`, `ref_x`, `ref_y`, `vref_x`, `vref_y` | target_idx 为统一弧长派生的显示索引；s_progress/s_total 为 float mm，finished/path_deviation 为 bool。progress 字段是当前最新已提交 worker 进度，参考字段仍按执行器采用的 seq 关联；worker 在本 tick 中发布时，两者可能属于相邻 seq |
| 目标关联 | `run`, `seq`, `alpha`, `frames_since`, `F_target_x`, `F_target_y`, `F_target_z`, `I_target_0`…`I_target_5` | int 追踪运行编号/执行器使用的 seq、float 插值 alpha、int 帧计数、float µN / A；目标/参考按 `(run,seq)` 从有界快照关联，避免混入 worker 后发的目标 |
| 执行 | `executor_updated`, `cmd_exec_a0`…`cmd_exec_a5`, `cmd_sent_a0`…`cmd_sent_a5`, `max_cmd`, `serial_connected` | bool 本 tick executor 是否执行、int CurrentExecutor 输出、int 最终安全层状态、int GUI 限幅、bool 串口连接；cmd_sent **不是**硬件收到命令的确认 |
| 模型 | `I_est_0`…`I_est_5`, `F_est_x`, `F_est_y`, `F_est_z`, `B_x`, `B_y`, `B_z`, `B_mag_mT`, `align_ratio`, `low_field` | float A / µN / T / mT / 无量纲，bool 低场；来自已有执行器/R-L 诊断，不重新 forward_model；executor 未更新时保持上次模型值，并由 executor_updated 标明 |
| ADC | `adc_frame_count`, `adc_age_ms`, `raw0`…`raw5`, `adc_valid`, `adc_running`, `adc_error_flags` | int MCU frame_count、float 距 PC 最近收到快照的 ms、int raw/状态/flags；仅监测，不转换成 A、不进入控制 |

空单元格表示不可用或本 tick 没有执行，**不代填 0**。CSV bool 为 `True`/`False`；除 mode/estimator_mode/eso_mode/effort_mode/stale_status 外，非空单元格可按表转成 int/float/bool。模型诊断在首次 executor 执行之前不可用；手动电流模式不经过 CurrentExecutor，cmd_exec 留空，但每个 tick 仍记录最终 cmd_sent。退出、连接、急停等发生在 tick 之外的 UART 事件不单独增加控制行。

`I_target` 是 solver 发布的目标电流；`I_est` 是 R-L 估计；`raw` 是未标定 ADC。三者用途和单位不同。通道顺序保持 a0…a5 / `+X,+Y,+Z,-X,-Y,-Z` / Pole `1,3,5,4,6,2`。

## worker 字段表

| 分组 | 字段 | 含义 |
|---|---|---|
| 时间/关联 | `t_mono`, `t_wall`, `run`, `seq`, `dropped`, `kalman_age_ms`, `target_age_ms`, `stale_status` | monotonic / wall-clock，按 run+seq 联表；kalman_age_ms 为本周期实际消费输入的 monotonic age（即使已有更新输入也不冒充新输入），target_age_ms 为观察时最新发布目标的 monotonic age |
| MPC 输入 | `x0_x`, `x0_y`, `d_used_x`, `d_used_y`, `F_prev_x`, `F_prev_y`, `F_prev_z` | 当前准静态 MPC 的位置 x0（mm）、已有扰动力输入和 F_prev（µN）；不虚构四状态预测模型 |
| 控制结构 | `eso_mode`, `effort_mode`, `F_ss_x`, `F_ss_y` | 本次 worker 消费参数中的 first_order/legacy/off，实际 MPC 实例的 absolute/steady_state，以及 c*vref−d_used（µN）。跳过事件保持这些未计算列为空。F_prev 仍是发送整数命令的静态 forward 力，和含 R-L lag 的 ESO 输入 u_eso 不同 |
| 目标/路径 | `F_target_x`, `F_target_y`, `F_target_z`, `ref_x`, `ref_y`, `vref_x`, `vref_y`, `s_progress`, `s_total`, `finished`, `path_deviation`, `I_target_0`…`I_target_5` | 直接复制已有 MPC、参考窗口、solver 输出 |
| 参数/性能 | `mpc_ms`, `solver_ms`, `mpc_cost`, `horizon`, `w_pos`, `w_vel`, `w_u`, `w_du`, `fmax`, `max_cmd`, `force_error_uN` | 当前实际 MPC 实例权重/限制与已有求解计时/误差 |
| 可行性 | `converged`, `current_constraint_active`, `field_constraint_active`, `direction_constraint_active`, `sparse_infeasible`, `current_bound_ok`, `slew_ok`, `actuation_condition` | 复制 solver 已有 bool 标记和数值条件数；没有该字段时留空 |

当前 metadata 是**开始时**快照；实验途中 GUI 参数变更不生成新的 metadata。worker 行包含实际使用的 MPC 权重/限制，30 Hz 行包含当前模式/max_cmd。为完整保留不同实验配置，应分别开始新记录会话。

## ESO / effort A/B（2026-10-06）

新默认为 L3：FirstOrderESO + steady_state effort；omega=4 沿用软件默认，尚未实机标定。高级控制选择 observer，路径页选择 effort；ESO checkbox 可以关闭。L0=legacy+absolute，L1=off+absolute，L2=off+steady_state，L3=first_order+steady_state；first_order+absolute 也可用于诊断。两项选择可保存/载入，旧设置缺少新键时使用新默认。

新 ESO 用 x_dot=(u+d)/c，z1 为 mm、z2 为 µN。缺失视觉帧标记 gap；下一次有效输入只重置位置并清零扰动。dt>0.15 s（复用当前 input freshness 门槛）同样重置；未改变 freshness 的判定或停止流程。在允许的 omega/dt 范围内按 T*omega≤0.5 子步处理，位置量测在线性插值后校正；不补算未知缺口力。legacy ESO 不经过这些新分支。

steady_state 将 wu*F² 改成 wu*(F−F_ss)²，H 不变、g 减去 wu*F_ss，箱约束与变化率项不变。日志中的 F_ss 未 clip；实际 F_target 仍受原 Fmax 箱约束和总水平幅值约束、solver/执行器约束影响。

## Freshness 语义（2026-10-06）

SharedState 自行记录输入/参数/目标的 monotonic 时间，旧 wall-clock `t` 仅保留兼容显示/日志。SharedState、worker、executor 可注入同一 clock；GUI 的控制日志时间轴也使用该 clock。worker 在输入读取时和目标正式发布时两次检查输入年龄；过期或 seq 被另一发布抢先更新时，不增加 seq、不覆盖新目标。参数快照记录 monotonic 时间，但参数本身为持久配置，本轮不另加参数过期策略。

| 门槛 | 边界行为 |
|---|---|
| `KALMAN_STALE_S=0.15` | age ≤0.15 s 可求解；>0.15 s 或尚无输入时 STALE_INPUT，不逆解、不发布、不记 solver_error；fresh 输入后自动恢复 |
| `TARGET_STALE_HOLD_S=0.30` | target age ≤0.30 s 保留原执行数学；>0.30 至 ≤1.00 s 为 STALE_TARGET，不采用过期 seq、不推进插值，保持最后成功命令；既有 R-L/正向诊断按保持命令更新 |
| `TARGET_STALE_STOP_S=1.00` | target age >1.00 s，在 GUI 下一次检查时请求现有 normal_stop；随后按原 slew 归零，状态 STALE_TARGET_TIMEOUT，不急停 |

三个数值仅为当前保守软件门槛，尚未实机标定。视觉丢失时不会因 KF 预测而续期 SharedState 输入；原视觉丢失 >1 s 正常停止保留，计时改用 monotonic。过期暂停期间重复最后成功帧，不刷新目标年龄、不改变 serial/heartbeat 实现。超时状态在正常归零后保留供诊断，下一次 tracking 启动复位；freshness 不阻止其他既有手动模式入口。

STALE_INPUT 的 worker 事件沿用当前 run/seq，MPC/solver/target 列留空，表示本周期未计算、未发布；不能把它统计为一个新目标。正常 worker 行为 FRESH，只有成功目标发布才增加 seq。30 Hz 同时存在 input/target 失效时，显示优先级为 target timeout → stale target → stale input → fresh。FRESH 的非 tracking 行表示当前模式不使用该共享控制目标，不应把其 age 当作正在执行的 AUTO_TRACK 数据。

## 统一路径进度（2026-10-06）

worker 是唯一 progress 计算者；SharedState 保存冻结的 PathProgress。`_reference_window()` 只计算候选，`set_I_target(...progress=...)` 在同一锁内通过原 freshness/stop/seq 检查及新增 reference generation 检查后，原子提交 target 与进度。正常 worker 行中的 s_progress/s_total/finished 与该次提交一致；GUI 结束判断与黄色目标点只读取 SharedState，target_idx 不再独立推进。

每次 tracking 创建新 SharedState/worker；set_reference 也重置 s_progress=0、finished=False、deviation=False，并递增 reference generation。原 lead、生成器和 resample 不改；s_total 包含 lead，闭合图形也沿用原始首尾点，不自动增加闭合边。

局部窗口为 `[max(0,s_prev-0.3), min(s_total,s_prev+max(1.0,3*(Fmax/c)*T_mpc))]` mm，逐线段按窗口裁剪投影。最近距离 >1.5 mm 才向前扩大至路径尾部；扩大后距离 >3 mm 时发布 PATH_DEVIATION 状态，保留原 arc/target/seq/时间戳，由 GUI 调用原 normal_stop。恰好 1.5/3 mm 不升级。finish 同时要求 s_progress ≥ s_total-finish_tol、到实际终点的距离 ≤ finish_tol；finish_tol 复用原 GUI 的0.5 mm默认。六个默认集中于 config，均非实机最终标定参数。

STALE_INPUT/慢求解被拒绝时不推进进度。PATH_DEVIATION 的 worker 行沿用旧 seq，未计算的 target/solver 列为空；其 path_deviation 为True。30 Hz progress 取最新 SharedState，故即使已结束但执行器尚未采用最终 seq，finished 仍正确记录True。参考/F/I字段仍表示实际执行器采用的目标，不能误认为它们永远和最新 progress 属于同一 seq。

## 计数与故障

正常停止排空后，每个流 `attempted = written + dropped + rejected`；无错误、无停止后额外生产时 rejected=0。CSV 中 dropped 是**写入该行之前**的累计溢出数；末尾溢出由 summary 补齐，不能只依赖最后一行。两个流共享队列，但 summary 各自计数。

写盘失败后记录结束，GUI 显示“日志错误”、未写入和拒收计数，控制、追踪、worker 和串口保持运行。失败会话的 metadata/CSV/summary 可能未完成，不能承诺磁盘故障时 summary 一定落盘；GUI 内存统计仍可辨认 accepted-written 未落盘量。正常开始/停止均不等待磁盘；退出时先按原流程急停/关闭控制资源，再有限时 join 日志线程。

## 软件验证（2026-10-05）

基线 `19c1017792a3a79a200965424d2455da91391fa5`。新增 25 项测试覆盖真实 QTimer + 后台 worker、合成相机、双向假串口、ADC raw、CSV 类型/列数/单调时间、tick 数、队列溢出恢复、文件不覆盖、打开/中途写盘错误、run 重置及 seq 发布竞争。

原 checkpoint / 当前日志关闭 / 当前日志开启的固定输入、dt、worker 时序三方回放：150 tick、144 个实际发送的 43-byte 帧逐字节一致，状态/ESO 输出/R-L 数组和 worker 目标逐位一致。source/AST 审查：CurrentExecutor（含 R-L）、send_commands/protocol/safety、急停、路径函数原文一致；既有计算移除纯日志观察语句后 AST 一致；MPC/solver/estimator/friction/config/现有测试及 146 个固件文件原字节一致。ADC 仍只监测。

交替运行顺序性能测量（offscreen、合成相机、假串口、测试子进程 BLAS/OMP 单线程，仅软件结果）：

- AUTO_TRACK 60 tick：快照入队中位 **0.0723 ms**，完整 tick 配对中位增量 **0.0143 ms**。
- 手动电流 240 tick：快照入队中位 **0.0531 ms**、最大 **0.1082 ms**；完整 tick 关闭/开启中位 **1.5218 / 1.6461 ms**，配对中位增量 **0.1059 ms**。
- 两项中位增量均小于 1 ms；配对中位增量与“两个中位数之差”是不同统计量，计时包含调度噪声。未由软件测试替代真实相机/串口/磁盘的实验测量。

Ruff 0、Black 23 files、mypy 0/23 files；新模块 mypy --strict 通过。GUI smoke、MPC 6/6、multirate 18/18、solver/protocol/safety 25/25、CURT 132/132、shared control 6/6、bead simulator 8/8、既有错误路径 46/46 均通过。原有 multirate 时序测试前两次分别出现 67.1/62.1 ms 越过原 60 ms 阈值，保留失败日志；之后原 checkpoint 与当前代码单独运行均 18/18，未修改测试、调度或阈值。新模块标准库线程行跟踪为 294/296（99.32%，包含写盘线程）。

完整软件证据在工作区 `artifacts/unified-control-log-20261005/`，不纳入 checkpoint。
