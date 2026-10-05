# 统一控制日志

GUI 的“控制日志”页 → 选择**新的** CSV 名称 → “开始记录” → 按原流程实验 → “停止记录”。默认不记录，不改变自动追踪、手动控制或停止流程。停止后后台排空队列，状态变成“已停止”后可开始下一次记录；必须选择新文件名。

选 `run.csv` 时，一个会话产生：

- `run.csv`：每个完成的主循环 tick 一行，约 30 Hz，95 列。
- `run.worker.csv`：每次完成的 MPC/solver 诊断一行，约 10 Hz，46 列；原有手动导出 MPC CSV 的格式和内容保留。
- `run.metadata.json`：开始记录时 GUI/config、MPC 权重、物理参数、R、B 目标、路径、模式、command/current 限制；Git HEAD 和 tracked dirty 标志由写盘线程读取。`R_force_model_to_camera` 是 model XY → camera world XY 的正交映射；`offset.position_mm` 保存当前 `frame_offset_mm` 的 XY 与固定 z=0，来源为 `config.FRAME_OFFSET_MM`，含义是模型原点在相机世界坐标中的位置（默认 `[0,0]`，尚未实机标定）。ADC offset 仍为 `0/default`，原有像素中心也保留；未增加 CSV 字段。
- `run.summary.json`：正常结束时两个流分别的 attempted / accepted / written / dropped / rejected 和日志错误。

四个文件均用 `x` 模式创建；任何一个已经存在都会报日志错误，不覆盖。CSV/JSON 写入、flush、Git 查询只在独立 daemon 线程执行。默认队列 2048 行，主线程和 worker 只提交标量快照，使用 `put_nowait`；队列满只增加对应流 dropped，之后可以恢复。目标关联缓存另有 128 项上限。

## 30 Hz 字段表

| 分组 | 字段 | 类型、单位与含义 |
|---|---|---|
| 时间 | `t_mono`, `t_wall`, `tick` | float 秒（monotonic 主时间轴）、float Unix 秒（人工换算 wall-clock）、int GUI 生命周期 tick 序号 |
| 时间/性能 | `dt_ms`, `jitter_ms`, `total_ms`, `vision_ms`, `estimator_ms`, `controller_ms`, `serial_ms`, `dropped` | float ms；dropped 为 int、两个流累计溢出数。dt 沿用原控制的 wall-clock/clamped dt；jitter 沿用原有绝对偏差；total 沿用 render/status/log capture **之前**的计时；serial_ms 是最近一次 send_commands 耗时，无发送时保持 |
| 状态 | `mode`, `tracking` | string 模式、bool 追踪状态 |
| 视觉 | `frame_ok`, `detected`, `px_x`, `px_y`, `area_px` | bool 帧可用/检出，float 像素/像素面积；直接使用当前检测结果 |
| 测量 | `meas_x_mm`, `meas_y_mm` | float mm；直接复制 estimate_state 已转换的 z，失检留空，不再转换坐标 |
| 估计 | `kf_x`, `kf_y`, `kf_vx`, `kf_vy`, `estimator_mode` | float mm / mm/s，string RAW/EMA/KALMAN；kf 字段仅 KALMAN 模式填入 |
| 当前估计器输出 | `state_x_mm`, `state_y_mm`, `state_vx_mm_s`, `state_vy_mm_s` | float mm / mm/s；记录 RAW/EMA/KALMAN 实际供控制使用的输出 |
| ESO | `eso_mode`, `eso_updated`, `d_hat_x`, `d_hat_y`, `u_eso_x`, `u_eso_y` | string ON/OFF、bool 本 tick 是否 step、float µN；u 是已有的实际 ESO 输入，未 step 留空；d_hat 是当前 z3，可能保持 |
| 路径 | `target_idx`, `s_progress`, `ref_x`, `ref_y`, `vref_x`, `vref_y` | int 索引，float mm / mm/s；s_progress 仅来自已有 worker `_ref_arc` 最近投影弧长，未实现新的路径推进算法 |
| 目标关联 | `run`, `seq`, `alpha`, `frames_since`, `F_target_x`, `F_target_y`, `F_target_z`, `I_target_0`…`I_target_5` | int 追踪运行编号/执行器使用的 seq、float 插值 alpha、int 帧计数、float µN / A；目标/参考按 `(run,seq)` 从有界快照关联，避免混入 worker 后发的目标 |
| 执行 | `executor_updated`, `cmd_exec_a0`…`cmd_exec_a5`, `cmd_sent_a0`…`cmd_sent_a5`, `max_cmd`, `serial_connected` | bool 本 tick executor 是否执行、int CurrentExecutor 输出、int 最终安全层状态、int GUI 限幅、bool 串口连接；cmd_sent **不是**硬件收到命令的确认 |
| 模型 | `I_est_0`…`I_est_5`, `F_est_x`, `F_est_y`, `F_est_z`, `B_x`, `B_y`, `B_z`, `B_mag_mT`, `align_ratio`, `low_field` | float A / µN / T / mT / 无量纲，bool 低场；来自已有执行器/R-L 诊断，不重新 forward_model；executor 未更新时保持上次模型值，并由 executor_updated 标明 |
| ADC | `adc_frame_count`, `adc_age_ms`, `raw0`…`raw5`, `adc_valid`, `adc_running`, `adc_error_flags` | int MCU frame_count、float 距 PC 最近收到快照的 ms、int raw/状态/flags；仅监测，不转换成 A、不进入控制 |

空单元格表示不可用或本 tick 没有执行，**不代填 0**。CSV bool 为 `True`/`False`；除 mode/estimator_mode/eso_mode 外，非空单元格可按表转成 int/float/bool。模型诊断在首次 executor 执行之前不可用；手动电流模式不经过 CurrentExecutor，cmd_exec 留空，但每个 tick 仍记录最终 cmd_sent。退出、连接、急停等发生在 tick 之外的 UART 事件不单独增加控制行。

`I_target` 是 solver 发布的目标电流；`I_est` 是 R-L 估计；`raw` 是未标定 ADC。三者用途和单位不同。通道顺序保持 a0…a5 / `+X,+Y,+Z,-X,-Y,-Z` / Pole `1,3,5,4,6,2`。

## worker 字段表

| 分组 | 字段 | 含义 |
|---|---|---|
| 时间/关联 | `t_mono`, `t_wall`, `run`, `seq`, `dropped`, `kalman_age_ms` | monotonic / wall-clock，按 run+seq 联表；kalman_age_ms 使用已有 wall-clock 状态时间戳，在 worker 完成时计算，仅用于诊断 |
| MPC 输入 | `x0_x`, `x0_y`, `d_used_x`, `d_used_y`, `F_prev_x`, `F_prev_y`, `F_prev_z` | 当前准静态 MPC 的位置 x0（mm）、已有扰动力输入和 F_prev（µN）；不虚构四状态预测模型 |
| 目标/路径 | `F_target_x`, `F_target_y`, `F_target_z`, `ref_x`, `ref_y`, `vref_x`, `vref_y`, `s_progress`, `I_target_0`…`I_target_5` | 直接复制已有 MPC、参考窗口、solver 输出 |
| 参数/性能 | `mpc_ms`, `solver_ms`, `mpc_cost`, `horizon`, `w_pos`, `w_vel`, `w_u`, `w_du`, `fmax`, `max_cmd`, `force_error_uN` | 当前实际 MPC 实例权重/限制与已有求解计时/误差 |
| 可行性 | `converged`, `current_constraint_active`, `field_constraint_active`, `direction_constraint_active`, `sparse_infeasible`, `current_bound_ok`, `slew_ok`, `actuation_condition` | 复制 solver 已有 bool 标记和数值条件数；没有该字段时留空 |

当前 metadata 是**开始时**快照；实验途中 GUI 参数变更不生成新的 metadata。worker 行包含实际使用的 MPC 权重/限制，30 Hz 行包含当前模式/max_cmd。为完整保留不同实验配置，应分别开始新记录会话。

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
