# PROJECT_MEMORY

项目长期进度记忆源。以后处理本项目时，优先读取并在每阶段结束后更新本文件。

## 最新权威摘要（2026-10-05：统一控制日志，独立任务）

- 基于 `19c1017792a3a79a200965424d2455da91391fa5`（`refactor: harden Python error handling and lint clean`），本轮仅增加统一日志。GUI 新增“控制日志”页、“开始记录 / 停止记录”，默认关闭，不改变原实验启动流程。
- 新增 `dipole-control/control_log.py`：2048 行有界队列、put_nowait、独立写盘线程；队列满只累计 dropped 并继续，写盘失败只结束日志并在 GUI 显示错误，不停止追踪/worker/串口。会话 CSV、worker CSV、metadata、summary 全部独占创建、不覆盖。SharedState 仅增加日志观察接口及 128 项有界目标关联缓存。
- 控制 CSV 95 列、每 tick 一行，time.monotonic 为主时间轴，保留 wall-clock；直接快照视觉、测量/估计、ESO、路径参考、目标、执行命令、R-L/磁场磁力模型及 raw ADC。worker CSV 46 列，通过执行器实际使用的 `(run,seq)` 关联；s_progress 只读取已有 `_ref_arc`，kalman_age 只读取已有 wall-clock 时间信息，没有实施路径修复或 freshness。
- metadata 保存开始时 GUI/config、实际 MPC 权重、物理参数、电阻/电感、R、0/default offset、B 目标、路径、限制和模式，以及 Git commit / tracked dirty。字段、单位、持有值/空值语义、正常结束计数及失败边界见 `dipole-control/CONTROL_LOG.md`。
- 源码保护与固定输入/dt/worker 时序三方回放证明：原 checkpoint / 当前日志 OFF / 当前日志 ON，150 tick、144 条实际发送的 43-byte 帧及控制状态/ESO/R-L/worker 目标逐位相同。CurrentExecutor（含 R-L）、send_commands/protocol/safety、急停、路径函数原文保持；现有计算移除纯日志观察语句后 AST 保持；MPC、solver、estimators、friction、config、原有测试和 146 个固件文件字节保持。ADC 仅 raw 监测，未接入电流 PI。
- 新增日志测试 25/25。性能：AUTO_TRACK 60 tick 快照入队中位 0.0723 ms、完整 tick 配对中位增量 0.0143 ms；手动电流 240 tick 对应 0.0531 / 0.1059 ms，快照最大 0.1082 ms；均达到中位额外开销 <1 ms。结果来自合成相机/假串口，未操作实板。
- Ruff 0、Black 23 files、mypy 0/23 files，新模块 --strict 通过；GUI smoke、MPC 6/6、multirate 18/18、solver/protocol/safety 25/25、CURT 132/132、shared control 6/6、bead simulator 8/8、原错误路径 46/46 通过。multirate 原时序测试前两次最大帧间隔 67.1/62.1 ms 超过原 60 ms 阈值，失败日志保留；随后原 checkpoint 和当前代码单独测试均 18/18，未改测试/阈值/控制节拍。新日志模块标准库线程行跟踪 294/296（99.32%）。
- 软件验证、原 checkpoint 回放、源码保护、统计与 diff/status 证据保存在 `artifacts/unified-control-log-20261005/`；artifacts 不纳入提交。本轮用户明确授权验证通过后建立唯一独立 checkpoint：`feat: add unified control diagnostics logging`（本条所在提交），不 push。

> 早期摘要和计划保留为历史；本轮未处理 ESO、MPC、坐标系、路径、solver、watchdog、freshness、STM32 或六路电流 PI。当前控制链及实板事实承接上一 checkpoint。

## 最新权威摘要（2026-10-04：中断恢复后 Ruff 清零，独立代码质量 checkpoint）

- 恢复时 HEAD 仍为类型 checkpoint `65fad950a092299af602e69f1e092a32795fb030`；暂存区为空。已有 16 个 tracked 文件的未提交修改和 untracked artifacts 原样保留，并在 `artifacts/code-quality-20261004/ruff-zero/recovered/` 保存恢复快照、完整 diff 与 SHA-256。
- 重新运行 Ruff 的真实起点为 **24 项**：BLE001 14、S110 3、F821 1、RUF046 4、FURB122 2。中断前已改完 heightForWidth、coil_test_step、矢量箭头两个坐标的 4 处 RUF046；逐处验证等价后保留，没有重复修改；CSV 两处尚未改。
- 本轮新增 4 处舍入冗余转换清理及 2 处惰性生成器 writelines；全部 8 个舍入诊断及两个 CSV 路径与类型 checkpoint 对照，1,122 组验证 PASS，包含整数类型、协议/限幅、半整数舍入、CSV 字节/顺序、转换/编码异常及部分输出。未使用 unsafe-fixes、noqa、新增 ignore 或修改 Ruff 规则。
- F821 真 bugfix：bead_sim_gui 导入 os，参数扫描走真实 CSV 和 PNG 导出不再 NameError；保留失败传播及 finally 恢复鼠标。另修复设置文件 JSON 根节点为数组/null 时未被校验的问题，提示并保留默认设置。
- 异常捕获按串口、模型、文件/编码、OpenCV、CSV、控件转换分别收窄。安全边界保留宽捕获并用 logger.exception 留完整 traceback：TX 失败断开、急停写入失败继续停本地线程、手动力解算失败安全归零、诊断失败显示值归零；closeEvent 的设置序列化 bug 也不能阻断后续急停。串口关闭预期失败显示/记录，finally 始终清理引用；模型/串口连接的真正程序错误在禁用模型或清理连接后重新抛出。
- 最终 Ruff **0**、Black **PASS / 20 files**、mypy **0 error / 20 files**；GUI smoke PASS；MPC 6/6、multirate 18/18、solver/protocol/safety 25/25、CURT parser/GUI 132/132、shared control 6/6、bead simulator 8/8；新增异常路径/参数扫描 46/46。多速率时序组本轮单独执行一次即通过，沿用历史仅测试子进程的 BLAS/OMP 单线程设置；未改变控制节拍/阈值。新增仿真 GUI 测试有 10 条已有空图例 warning，无失败。
- 本轮与恢复快照比较，仅新增修改三个生产文件和本记忆，并新增 tests/test_error_paths.py。ForceMPC、SharedState、ControlWorker、CurrentExecutor、DipoleSolver、KalmanFilter2D、ESO1D 类源码保持；send_commands/emergency_stop 正常执行主体及实际发送字节保持。146 个 tracked 固件文件和 4 个 CURT 专属文件逐字节保持；MPC 数学/权重、R-L 模型、10/30 Hz、a0...a5、CURT 控制反馈边界不变。
- 完整逐项决策、最终 diff/stat/status、全部检查日志与源码保护证据：`artifacts/code-quality-20261004/ruff-zero/final-review.md`、`final-results.json`、`equivalence-results.json`、`protection-results.json`。旧目录中的 28 项报告是历史状态，不代表本轮结果。
- 用户明确授权将已验证的代码质量清理建立独立 checkpoint：本条所在提交 `refactor: harden Python error handling and lint clean`，共 17 个文件（15 个既有 Python 源码/测试文件、本记忆及新增 tests/test_error_paths.py），父提交为上述类型 checkpoint。提交前核对完整工作区/暂存区 diff 与精确路径，显式 stage；artifacts、日志、CSV、cache 与其他无关文件均未纳入，未 push。早期“未提交/等待审查”保留为历史记录；本轮仅整理提交和更新记忆，功能代码未继续修改。
- 该 checkpoint 的提交前后 Git 记录与路径验证位于 `artifacts/code-quality-20261004/ruff-zero/checkpoint/`；完成后 tracked 工作区与暂存区无差异，仅 artifacts/ 未跟踪。没有打开真实 COM/相机、操作实机或修改/构建/烧录固件；真实硬件验证、CURT 标定与 PI 状态不变。

## 历史摘要（2026-10-04：类型 checkpoint 完成，Ruff / Black 初轮清理待审查）

- 已按用户授权建立独立类型修复 checkpoint：`65fad950a092299af602e69f1e092a32795fb030`，`chore: clean up Python typing`。仅提交 7 个类型修复/依赖声明/记忆文件；未提交 artifacts，未 push。
- Ruff 全上位机新基线为 104 项：未用导入/变量 37、导入/顺序/风格 34、简化 9、异常处理 17、潜在 bug 1、其他 6。45 项仅用安全 fix 自动处理；31 项逐项人工确认后处理；保留 28 项。未使用 unsafe-fixes、未新增 noqa 或全局 ignore。
- 人工修改仅为 21 个未读取绑定重命名（所有计算、解包和分配保留）、4 个共享列表 ClassVar 注解、5 处有意字符串拼接的括号、1 处同顺序短路条件合并。保留 BLE001 14、S110 3、RUF046 8、FURB122 2、F821 1；未改变故障回退、整数转换、CSV write 调用顺序，未修复仿真 GUI 参数扫描原有的 os 未定义错误路径。
- Black 仅格式化当时 check 指出的 15 个文件；全上位机最终 black --check 为 PASS（19 文件），mypy 为 0 error（19 文件）。requirements-dev.txt、运行依赖及 pyserial 版本本轮均未改。
- Ruff 后、Black 后均执行 GUI smoke、MPC 6、multirate 18、solver/protocol/safety 25、CURT parser/GUI 132、shared control 6、bead simulator 基础 8，最终均通过。Black 后 multirate 并行首轮有一次最大帧间隔 119.6 ms 的耗时断言失败；其他检查结束后单独重跑 18/18 通过，原失败与重跑日志均保留；未调整 60 ms 阈值或控制节拍。
- 源码保护审查通过：19 个上位机 Python 文件在明确允许的导入清理、未读绑定名、ClassVar、等价短路条件和 docstring 缩进规范化后，可执行 AST 与类型 checkpoint 等价；send_commands() AST 完全一致。STM32 固件、CURT parser/CSV/parser tests/GUI telemetry tests 均逐字节保持；MPC 数学、Kalman/ESO、R-L、电流执行及仿真物理表达式保持。
- 完整逐项 Ruff 决策与全部日志：`artifacts/code-quality-20261004/ruff-item-review.md`、`ruff-stage-results.json`、`black-stage-results.json`、`black-stage-protection.json`。日志/缓存/CSV 均在未跟踪 artifacts 内，不提交。
- 本轮代码质量修改 **尚未提交**，等待用户审查；HEAD 保持上述类型 checkpoint。未 push，未操作真实串口、相机或硬件；CURT 电流标定和 PI 状态不变。

## 历史摘要（2026-10-04：上位机类型检查清零；当时未提交，现已 checkpoint）

- 本轮仅修复类型信息：原 7 个代码诊断与 2 个 pyserial 类型桩缺失诊断均已解决。全上位机 mypy 默认参数及 `--follow-imports=silent` 均为 19 source files / 0 error；没有全局 ignore_missing_imports 或新增 type: ignore。
- 四处空列表注解与两个可选模块注解；原失败计数表达式、全部既有函数/类定义和控制行为保持。安装 `types-pyserial==3.5.0.20260712`，pyserial 仍为 3.5；新增 requirements-dev.txt，运行 requirements.txt 不变。
- GUI smoke、MPC 专项 6、multirate 18、solver/protocol/safety 25、共享控制 6、CURT parser/GUI 132、仿真基础 8 均通过。ruff 仍有原有 104 个诊断；black 仍有原有 15 个需格式化文件，均无新增。
- 当前 HEAD 仍为 `1e9303721c47530f6c9995859ac283f96f1cff5e`。遵守本轮要求，不自动 commit、不 push；下方“7 个类型问题和 2 个类型桩问题待处理”属于历史记录。
- 完整检查与源码保护证据位于 `artifacts/type-check-20261004/`。真实硬件烧录、实测、CURT 电流标定及 PI 状态不变。

## 最新权威摘要（2026-10-04：CURT 已集成 MPC-only 主 GUI）

- 正式实机自动轨迹控制仍为 **MPC-only**；RL / ES-MLP 已删除；独立仿真器及其 PID 保留。
- 用户明确授权的 Phase A checkpoint 已提交：`10f6c164608aa8af83872cbe3420666e3ef79879`，`refactor: keep mpc-only control and remove rl stack`。只含已完成的 MPC-only / RL 清理 13 个路径，未 push；固件、独立 telemetry 源码与 artifacts 未纳入。
- **CURT telemetry 已完成主 GUI 软件集成和验证**：同一个 `MagneticDipoleControl.self.ser` 负责原有 TX 与唯一 RX reader；独立 15 ms RX QTimer + 现有 ADCStreamDecoder，新增“CURT / ADC”Tab。
- `@ADC` 只用于 ADC raw 显示、健康监测与独立 CSV 记录；raw 尚未标定为 Ampere，CURT 未进入 MPC/PWM 控制反馈，电流 PI 尚未实现。
- MPC 核心、Kalman/ESO、Fz 减摩、R-L 电流模型、六路逆解、signed 协议、a0...a5 顺序及 ADC 500 Hz / telemetry 约 10 Hz 设计均保持。原有 send_commands() 和其他控制函数源码不变。
- 软件回归通过：GUI 集成 27 项、原 CURT 105 项、GUI smoke、shared control 6/6、multirate 18/18、solver/protocol/safety 25/25、MPC 专项 6 项；兼容适配器下全 pytest 208 passed / 288 subtests passed。原生 pytest 仍有 4 个既有装饰器收集错误，静态检查既有问题另见文末。
- 本轮未打开真实 COM、未运行真实硬件 MPC、未发送硬件命令、未烧录或实测。主 GUI telemetry 实板验证仍待烧录新 HEX 后由用户执行；电流标定与 PI 仍属后续阶段。
- **CURT 两个 Git checkpoint 已获用户明确授权**：底层 telemetry 已提交 `7e624ccfc9e1e54b3b2feb9a317d36213e2c25b2`（`feat: add CURT telemetry pipeline`）；主 GUI 集成由本条记录所在提交（`feat: integrate CURT telemetry into MPC GUI`）固化。旧“不自动提交”边界对后续工作继续有效；本轮不 push。

完整架构、生命周期、测试、Git 边界与实板操作步骤见文末“CURT 主 GUI 集成完成（2026-10-04，Phase A / B）”。

> 下方早期“CURT 为独立工具、尚未集成主 GUI”和旧下一步计划保留为 historical / superseded；软件当前状态以本摘要及文末最新阶段为准。硬件烧录/测量事实没有由软件测试代替。


## 最新权威摘要（2026-10-04：MPC-only 与 RL 清理）

- 最后更新：2026-10-04。正式实机自动轨迹控制已收敛为 **MPC-only**；原 PD/PID 实机分支属于 historical / superseded，入口仍为 `dipole-control/magnetic_dipole_pid.py`。
- 强化学习 RL / ES-MLP 的独立 GUI、策略部署、训练代码、专用测试、策略 JSON 与设置 JSON 已从当前正式工程移除，属于历史实验路线。
- Kalman、ESO、Fz 减摩与六路磁场/磁力逆解保留。**ESO 是 Extended State Observer，不是强化学习；R-L 是电阻—电感电流动态模型，不是 Reinforcement Learning。**
- 保留 `SharedState → ControlWorker / ForceMPC（约 10 Hz）→ DipoleSolver → CurrentExecutor（约 30 Hz）→ send_commands() → Serial → STM32`，手动控制及诊断保持。
- 独立仿真器及其 PID、ParameterSweep、ExperimentFitter 本轮未处理。CURT telemetry 与电流标定工作继续保留；`curt_telemetry.py` 仍为独立工具，**尚未集成主 GUI**。
- 本轮仅做软件清理和无硬件验证，未操作串口、相机、固件或实板；下文 2026-10-03 的烧录/实测结论是当时记录，本轮没有更新其硬件事实。

完整清单、保护证明和验证结果见文末“强化学习 RL / ES-MLP 从正式工程移除（2026-10-04）”。

> 以下旧页首、旧“当前权威摘要”和早期控制路线保留为 **historical / superseded**。当前软件功能边界以上述 2026-10-04 摘要及文末最新章节为准；历史硬件事实仅由后续实际验证更新。

最后更新：2026-10-03；当前阶段：CURT telemetry 软件完成；新 telemetry HEX 尚未烧录到 STM32，实板 telemetry/raw 验证尚未完成；电流标定与六路 PI 留后续阶段。

历史状态说明：下文 2026-09-30 / 2026-10-01 的“CURT 未接线、PA4/JP5 未确认、采样版未烧录、未添加遥测、Keil 未执行、Git 不可用”等表述保留为历史记录，已被末尾 2026-10-03 最新状态取代。软件基线仍承接“第一阶段 Windows 软件验证完成（2026-10-01）”。

> 原页首历史状态（2026-09-30，已被后续状态取代）：六路 CURT 只采样软件基础设施已完成；Keil 最终构建与实板验证待执行。

## 当前权威摘要（2026-10-03）

- 六路 CURT 实物接线已完成，使用驱动板 CN4 接口；PA4/JP5 功能选择已确认在正确 ADC 档。
- GND 使用原有系统接线，已连接，不是当前 blocker。
- ADC1 + DMA2 Stream0 + TIM3 TRGO + current_sense 第一阶段软件已完成，六通道采样设计保持 500 frame/s；上一版采样固件已烧录（用户确认）。
- `C:\Python314\python.exe` 已安装 PySide6 6.11.2、pyserial 3.5，两个 import 均 PASS；Python 依赖缺失状态已解除。
- `uart_telemetry.c/.h` 和 `dipole-control/curt_telemetry.py` 已实现；约 10 Hz 前台快照遥测、终端 raw/状态显示和 CSV 记录已完成软件验证。
- 当前源码对应的新 HEX：`C:/Users/Administrator/Desktop/mag/artifacts/curt-telemetry-20261003/pwm_02.hex`。
- **该 telemetry 新 HEX 目前尚未烧录到 STM32，也尚未完成实板 telemetry 验证。** 当前下一步是烧录、运行接收器并验证真实 raw；外部电流表对照、电流标定和六路 PI 随后进行。
- 默认环境原上位机测试为 66/67，仍有一项线程时序越限；线程限制诊断 18/18 PASS。该性能事项与已解除的接线、跳帽和依赖问题分别记录。

最新阶段的完整状态、验证结果和九步实板计划见文末“CURT telemetry 软件完成（2026-10-03）”。

# Project baseline

- 当前上位机：`dipole-control/`。
- 当前固件：`pwm_double20260926DC6output/`，STM32F407IGTx，Keil MDK 工程 `MDK-ARM/pwm_02.uvprojx`。
- CubeMX 6.6.1；STM32CubeF4 V1.27.1；现有 HAL V1.8.1。只补同版本 ADC 文件，不升级现有 HAL。
- **历史状态/已被后续更新取代（Linux 阶段）**：当前可见 `.git/` 没有有效元数据，不能提供分支、HEAD 或 clean 状态；禁止重建 Git 或自动提交。后续 Windows 阶段已确认有效 Git baseline；禁止自动提交的边界继续有效。
- signed protocol：43 字节 ASCII，`a0:+30,a1:-45,a2:+60,a3:-75,a4:+90,a5:-10\r\n`；范围 -99…+99。
- 固定顺序：`[a0,a1,a2,a3,a4,a5] = [+X,+Y,+Z,-X,-Y,-Z]`。
- PWM 计算保持 `CCR = 4200 + signed_command * 42`；TIM2 保持现有更新链和运行期 ARR=41999。
- TIM1/TIM8 PWM 参数、signed parser、USART1 115200 8N1、UART DMA、PF 输出行为不得在本阶段改变。

| Logical | PWM / GPIO | Complementary GPIO | Physical pole (user supplied) |
|---|---|---|---|
| a0 | TIM1_CH1 / PA8 | TIM1_CH1N / PB13 | 1 |
| a1 | TIM1_CH2 / PA9 | TIM1_CH2N / PB14 | 3 |
| a2 | TIM1_CH3 / PA10 | TIM1_CH3N / PB15 | 5 |
| a3 | TIM8_CH1 / PI5 | TIM8_CH1N / PH13 | 4 |
| a4 | TIM8_CH2 / PI6 | TIM8_CH2N / PH14 | 6 |
| a5 | TIM8_CH3 / PI7 | TIM8_CH3N / PH15 | 2 |

物理编号仅作记录，不用于重排命令或采样数组。现存 `.hex/.axf` 不证明实机固件与当前源码一致。

# Confirmed custom wiring

- PF10 → poles 1/4/5/6 CTRL_SD。
- PF2 → pole 2 CTRL_SD。
- PF7 → pole 3 CTRL_SD。
- PF7 是用户已确认的定制普通 GPIO 输出，不是待解决风险；不得恢复为原厂 PM1_BEMFW。
- 使能为 4+1+1 分组，不是六路独立 enable。现有 Start_PWM/Stop_PWM 保持不变。
- 旧只读报告关于 PF7 未确认的表述已由本文件中的实物事实取代；旧报告不作为本项的最新结论。

# Current feedback mapping

| Logical | GPIO | ADC1 regular rank | Channel |
|---|---|---|---|
| a0 | PB0 | 1 | IN8 |
| a1 | PA6 | 2 | IN6 |
| a2 | PA3 | 3 | IN3 |
| a3 | PC2 | 4 | IN12 |
| a4 | PC3 | 5 | IN13 |
| a5 | PA4 | 6 | IN4 |

- 数组必须保持 `[a0,a1,a2,a3,a4,a5]`；rank 通道严格 `[8,6,3,12,13,4]`。
- 已实现：ADC1 → DMA2 Stream0 / Channel0 circular；独立 TIM3 TRGO 定时触发。
- 采用 500 个完整六路帧/秒：TIM3 84 MHz / (839+1) / (199+1)。TIM3 不启用 update IRQ，也不配置定时器输出 GPIO。
- ADC 12 bit、右对齐、PCLK2/4=21 MHz、84 cycles 采样时间，六路扫描约 27.4 μs；不是六路同瞬间采样。
- 两个六路帧的 halfword DMA 缓冲区；HT/TC 交替发布整帧，合计约 500 个正常 DMA 完成中断/秒。
- **历史阶段边界（2026-09-30，串口遥测限制已被后续更新取代）**：原始 ADC 为权威数据，不实现安培换算、PI、串口遥测或任何 ADC→PWM 路径。2026-10-03 已新增前台串口遥测；安培换算、PI 和 ADC→PWM 仍未实现。
- **历史状态/已被后续更新取代**：PA4/JP5 最终仍需实板确认。当前 DAC 未启用；本轮可配置 PA4 analog ADC，不以硬件尚未接线阻塞软件。最新用户确认：PA4/JP5 已在正确 ADC 档，不再是 blocker。

# Completed

**历史阶段记录（2026-09-30）**：以下为当时的完成情况；后续 Keil 构建、实物接线和遥测软件进度以 Windows 阶段及文末最新阶段为准。

- 上一阶段：只读基线审查完成；报告 `六路CURT反馈准备_只读代码审查报告_20260930.txt`。
- 本轮：确认 PROJECT_MEMORY 原先不存在；新增本文件作为长期计划/发现/进度记录。
- 修改前完整工作目录快照：`/tmp/mag-adc-baseline.f7bxHV/workspace`。这是临时差异审查依据，不是 Git 提交。
- 已补同版本 ADC HAL source/header 和必需的 LL ADC header；来源及哈希在固件 `tests/vendor_adc_sources.json`，与 ST 官方 v1.27.1 内容逐项比对通过（只规范 LF/末尾换行）。
- 已添加独立 `adc.c/.h`、`current_sense.c/.h`，固定逻辑 rank、analog GPIO、12 halfword 双帧 DMA、完整 raw 快照和状态计数。
- 已增加独立 TIM3、DMA2 Stream0 / ADC IRQ；优先级 3，原 UART=0、TIM2=2 不变。
- 已小范围同步 `.ioc` 和 Keil 文件清单；main.c 只加 include 和一次启动调用，原非 UTF-8 字节/CRLF 保持。
- 一次启动/故障锁存至 MCU reset，无自动重试；失败不调用全局 Error_Handler，不触碰 PWM/UART/CTRL_SD。
- 完整工作目录差异审查已完成：9 个已有文件修改、18 个新增文件；上位机、历史资产、已有固件二进制未改，没有提交/自动格式化/CubeMX 整体再生成。
- 独立代码审查及修复复审完成：没有未解决的 Critical/Important 软件项；修复了 DMA 延迟源帧一致性和累计错误位重复计数，并补充真实 C 回归测试。
- 本阶段无已知新增软件 blocker。仍不声称 Keil 最终构建、可烧录 image 或实板采样已验证；现有历史控制/通信问题没有借本轮扩大修改。
- 本轮没有实现电流 PI，ADC 测量不会改变现有 PWM 输出；这指代码控制路径/配置保持，不代替实板负载与示波器验证。

# Verification results

**历史阶段记录（2026-09-30）**：保留 Linux 阶段的验证结果和工具限制；“Keil 未执行、CURT 未连接、Git 不可用”等状态已被后续更新取代。

- 初始无可用 Keil、Wine、GCC、Clang、arm-none-eabi-gcc、make。后已下载并解包 TCC 和 ARM GCC 13.2.1 到 `/tmp/mag-adc-baseline.f7bxHV/toolchain`，不安装、不改系统。
- 静态集成最终 **16/16 PASS**。首次 RED 为 13 项中 10 项因新增设施缺失失败、3 项原基线保护通过；后续加入厂商来源哈希、完整配置和稳定源帧检查。
- 真实 C 模块最终 **17/17 场景 PASS**。初版 8 场景 PASS；独立审查后旧逻辑 6 场景 RED，修复后 GREEN。覆盖完整 HT/TC、PRIMASK、所有启动失败、OVR/DMA累计/重复mask、half0/half1 积压、NDTR 边界、拷贝期间 wrap、NDTR 相同但 opposite sticky flag 已置位。HAL 为 test stub，不是真实 ADC 测量。
- 全工程 C 编译验证：Keil 工程列出的 33/33 个 C translation units 均经 ARM GCC 13.2.1 编译 PASS，全部 C object relocatable link PASS。7 条厂商 unused-parameter 警告，新编写 ADC/current_sense 模块 `-Werror` 下通过。
- **历史状态/已被后续更新取代：Keil 最终构建未执行**：缺少 MDK/ARMCC；未组装 Keil ARMASM startup、未做最终 startup/library/image link、未产生新可烧录固件。不将静态检查/ARM C 编译写成 Keil 编译通过。后续 Windows Keil 最终 Rebuild 已完成。
- **历史状态/接线部分已被后续更新取代**：实板采样、时序/精度/原 PWM 示波器对照未执行；尚未连接 CURT。最新六路 CURT 已接线；真实 raw、时序和测量质量仍待实板验证。
- 已核对初始代码：无 ADC 驱动/初始化；候选 GPIO 未占用；UART RX DMA2 Stream2/Channel4、TX Stream7/Channel4。
- `main.c` 含非 UTF-8 中文注释，现有换行为 CRLF；必须保留原字节和编码，只做可验证的 ASCII 插入。
- Git 命令失败原因：可见环境没有有效 Git 元数据；不把该失败视为代码差异或 clean 证明。
- 字节级保护 PASS：signed parser、Check_Frame、SystemClock_Config、Start/Stop_PWM、原 TIM/UART callbacks、TIM1/TIM2/TIM8 初始化与 PWM GPIO、旧 DMA/UART IRQ、原 GPIO/UART/HAL/CMSIS/BSP/startup。main 移除两处允许插入后整文件 SHA-256 等于改前基线，CCR 六处公式和 PF2/PF7/PF10 行为未改。
- 资源 sweep PASS：ADC1 独立模式、ADC2/ADC3/DAC 未启用；六个 analog GPIO 无复用冲突；DMA2 Stream0 不占 UART Stream2/7；TIM3 不占 TIM1/2/8；新增 IRQ priority=3，原优先级不变；先原 GPIO/DMA/timer 初始化，再 ADC/TIM3 init、DMA arm、TIM3 start。

实际复现命令（临时编译器路径失效后可用已安装的同类工具替代）：

```sh
cd pwm_double20260926DC6output
python3 -B tests/test_adc_infrastructure.py
python3 -B tests/run_current_sense_tests.py --cc /tmp/mag-adc-baseline.f7bxHV/toolchain/usr/bin/tcc --compiler-lib /tmp/mag-adc-baseline.f7bxHV/toolchain/usr/lib/x86_64-linux-gnu/tcc
python3 -B tests/compile_firmware_sources.py --cc /tmp/mag-adc-baseline.f7bxHV/toolchain/usr/bin/arm-none-eabi-gcc --bin-prefix /tmp/mag-adc-baseline.f7bxHV/toolchain/usr/bin --newlib-include /tmp/mag-adc-baseline.f7bxHV/toolchain/usr/include/newlib
```

# Open hardware questions

**历史问题清单（2026-09-30）**：接线和 ADC 档位两项已解决并在下方标记；有符号电流特性与标定仍属于后续测量问题。

- OPEN A：CURT 在正向、反向、续流、换向情况下能否可靠恢复有符号线圈电流。
- **历史状态/已被后续更新取代；已解决**：OPEN B：六路实物 CURT 接线尚未完成。最新状态：六路 CURT 实物接线已完成。
- **历史状态/已被后续更新取代；已解决**：PA4/JP5 实板 ADC/PM2_AMPW 路径确认。最新状态：已确认正确 ADC 档；GND 使用原有系统接线，不是 blocker。
- 六路 offset/gain、参考电压和方向标定。
- CURT RC 实测延迟及噪声、通道串扰。
- 不声称 ADC 已测得真实有符号线圈电流；1.27 V、0.12 V/A 不作为闭环常数。

# Next step

**历史计划（2026-09-30，顺序已被后续更新取代）**：保留下述原计划。当前应先烧录新 telemetry HEX、接收并验证 raw，再进行电流表对照和标定，详见文末九步计划。

先进行单路 CURT 实物测量和外部电流表对照，验证后再考虑六路 PI 电流闭环。

测量前需在真实 MDK 环境重建/最终链接并确认刷入新源码对应固件；当前旧 hex/axf 未更新。

# Implementation plan / progress

本轮使用本文件统一保存计划、发现和进度，不用临时报告替代长期记忆。

- [x] 读取权威基线、用户边界，确认 PF7 定制接线；保存改前快照。
- [x] 先添加并运行失败测试：13 项测试中 10 项因 ADC 缺失预期失败，3 项原基线保护通过；尚未实现时未将测试写为通过。
- [x] 从 ST 官方 CubeF4 v1.27.1 补齐 ADC HAL，并记录来源和哈希；现有 HAL 不改。
- [x] 添加 `Core/Inc/adc.h`、`Core/Src/adc.c`，固定 rank、analog GPIO、DMA2 Stream0。
- [x] 在 `tim.c/.h` 仅增加 TIM3 初始化；在 `dma.c`、`stm32f4xx_it.c/.h` 仅增加新采样 IRQ。
- [x] 添加 `current_sense.c/.h`：raw、完整帧计数、时间、valid/running、错误计数；PRIMASK 保存/恢复保护一致快照。
- [x] `main.c` 仅加头文件和启动调用；ADC 采样失败只停止采样，不关 PWM、不改命令、不调用全局 Error_Handler。
- [x] 小范围同步 `.ioc` 和 Keil 工程清单；不执行 CubeMX 整体再生成。
- [x] 运行全部验证与完整快照差异审查；复核采样路径没有 CCR/TIM1/2/8/PF/UART 写入。
- [x] 独立代码审查、记录真实构建状态、更新 Completed / Verification / 软件 blocker。

验证复现命令及边界见固件 `tests/README.md`。主进度文件保持本文件；测试脚本/fixture 是验证资产，不是另一个项目记忆源。

# Findings / encountered issues

- ST 官方 GitHub API 在 web 工具不可访问；shell curl 沙箱内连本地代理失败。已通过标准网络授权方式读取官方 v1.27.1 目录，后续只取同版本 ADC 文件。
- 候选 rank/GPIO/DMA/TIM 资源无硬冲突。保留 UART DMA 和中断优先级；新增采样 IRQ 优先级为 3，低于 UART=0 和 TIM2=2。
- DMA 与 UART 共用 DMA2 总线，但 Stream0/Channel0 不占 RX Stream2/Channel4 或 TX Stream7/Channel4；ADC DMA hardware priority=LOW，现有 UART hardware priority=LOW 不变。不是相互独立的物理总线。
- CubeMX 未启动/再生成。新增非致命 MX_ADC1_Init/MX_TIM3_Init 返回 HAL status，由模块启动；不要盲目再生成导致其被改成 void/Error_Handler 或重复 TIM3 用户区符号、main 双重初始化。再生成必须做同样的完整基线保护检查。
- snapshot valid 表示已发布且无锁存故障，不是电流标定有效或自动 freshness 判据；timestamp 为发布 tick。调试暂停不等于 ADC 停止，仍需实板压力测试。
- 独立审查发现并修复：DMA completion IRQ 积压/拷贝被抢占会让原 HT/TC 直接复制方案发布旧帧或混合帧，而 ADC OVR 不能检测 CPU 消费超时。现使用 NDTR + sticky opposite completion flag，在六路本地拷贝前后验证半区稳定，失败锁存 DMA_LATE/dma_late_count、仅停 TIM3。不修改旧 HAL/IRQ/PWM。
- 复制前后还检查 HAL DMA error/ADC error/硬件 OVR，避免 HAL 在 completion callback 后才处理错误造成误发布；累计 HAL error mask 仅对新出现类别计数，防止重复 OVR/DMA 统计。one-shot 错误计数定义为每次启动中新锁存类别，当前至 reset 不重启。
- 同版本 HAL_ADC_Start_DMA 未向调用方返回 HAL_DMA_Start_IT 的失败状态；新增模块需检查 DMA 实际 BUSY/EN 状态后才启动 TIM3，不修改厂商 HAL。
- 原报告的 UART 帧处理、快照竞态、TX buffer、无通信超时/真正 SD 急停等历史问题不在本轮修改范围；不以 ADC 数据驱动任何安全/PWM 控制。

# Change inventory / code evidence

**历史阶段代码证据（2026-09-30）**：以下行号、修改清单和“未添加消费者/遥测”描述对应当时的只采样版本；2026-10-03 已增加独立前台遥测消费者，原 ADC 发布逻辑不变。

相对本轮改前工作目录快照；路径以当前固件根目录为基准：

- 已修改（9）：`Core/Src/main.c`、`Core/Src/tim.c`、`Core/Inc/tim.h`、`Core/Src/dma.c`、`Core/Src/stm32f4xx_it.c`、`Core/Inc/stm32f4xx_it.h`、`Core/Inc/stm32f4xx_hal_conf.h`、`pwm_02.ioc`、`MDK-ARM/pwm_02.uvprojx`。
- 新增模块（4）：`Core/Src/adc.c`、`Core/Inc/adc.h`、`Core/Src/current_sense.c`、`Core/Inc/current_sense.h`。
- 新增同版本厂商依赖（5）：`Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c`、`Src/stm32f4xx_hal_adc_ex.c`；同一 driver 目录 `Inc/stm32f4xx_hal_adc.h`、`Inc/stm32f4xx_hal_adc_ex.h`、`Inc/stm32f4xx_ll_adc.h`。
- 新增验证资产（8）：`tests/test_adc_infrastructure.py`、`tests/signed_pwm_baseline.json`、`tests/test_current_sense.c`、`tests/stubs/stm32f4xx_hal.h`、`tests/run_current_sense_tests.py`、`tests/compile_firmware_sources.py`、`tests/vendor_adc_sources.json`、`tests/README.md`。
- 新增长期记忆（1）：工作目录根 `PROJECT_MEMORY.md`。

主要代码事实：

- `adc.c:9 regular_channels` 固定 `[8,6,3,12,13,4]`；`:14 MX_ADC1_Init` ADC scan/21MHz/84cycles；`:54 HAL_ADC_MspInit` analog pins/DMA2 Stream0。
- `tim.c:386 MX_TIM3_Init` 独立 500Hz TRGO；`dma.c:46` 与 `adc.c:88` 新 IRQ priority3。
- `current_sense.c:20 CurrentSense_Start` 非致命独立启动；`:77 FrameIsStable` / `:90 PublishFrame` 完整源帧验证；`:136 CurrentSense_GetSnapshot` 一致快照；`:53 LatchSamplingFault` 只停 TIM3。
- `current_sense.h:15 CurrentSense_Snapshot` raw/status schema；`:33 CurrentSense_GetSnapshot` 前台接口。未添加任何采样值消费/PWM/串口遥测路径。
- `main.c:333` 唯一采样启动调用；`:853 HAL_TIM_PeriodElapsedCallback` / `:885` 起六路 CCR 公式保留。

**历史状态/接线与跳帽部分已被后续更新取代**：硬件事实：PF7/CTRL_SD 为用户已确认定制接线；其余 CURT 特性/接线/PA4/JP5/标定保留 OPEN。500Hz/27.4μs 是根据现有时钟配置计算的设计值，尚无实板时序证据。最新接线与 ADC 档已确认；CURT 特性、标定和实板时序证据仍待验证。

# Windows Keil Build Baseline

**历史构建基线（2026-10-01）**：以下构建参数、产物及 Program Size 对应第一阶段采样版；当前 telemetry 版本的构建结果和 HEX 路径见 2026-10-03 阶段。

- 时间：2026-10-01。
- 环境迁移：Linux 开发环境 → Windows10 开发环境。
- 本章记录最新编译状态。页首及既有章节中“Keil 最终构建待执行/未执行”等表述保留为 Linux 阶段历史记录；本次 Windows Rebuild 已完成，实板验证仍待执行。

## Keil 环境与工程

| 项目 | 已验证配置 |
|---|---|
| Keil MDK-ARM | 5.43a |
| µVision | 5.43.1.0 |
| Device | STM32F407IGTx |
| Device Pack | Keil::STM32F4xx_DFP 3.1.1 |
| ARM Compiler | ARM Compiler 5.06 update 7 (build 960) |
| 实际使用的 CMSIS Pack | ARM::CMSIS 6.3.0，CORE 6.2.0 |
| 工程 | pwm_double20260926DC6output |
| Target | pwm_02 |
| 工程文件 | MDK-ARM/pwm_02.uvprojx |

- UV4.exe：`C:\Users\Administrator\AppData\Local\Keil_v5\UV4\UV4.exe`。
- ARM Compiler 路径：`C:\Users\Administrator\AppData\Local\Keil_v5\ARM\ARMCC\Bin`。
- 原工程 `MDK-ARM/pwm_02.uvprojx` SHA-256：`7F60EC238678F82662420C2B752852EA82F72C5659ED8F828D1576EC76B4F43C`。

## 验证方式与保护范围

- 使用 Keil 命令行 Rebuild，选择 Target `pwm_02`。
- 在临时副本中执行，未对原工程目录执行 Rebuild。
- 原固件目录 242 个文件的 SHA-256 在验证前后全部一致，未新增、删除或修改原工程文件。
- 临时副本中核对的 113 个输入文件（Core、Drivers、启动汇编、.uvprojx、.ioc）也保持不变。
- 未修改源码、.uvprojx、.ioc、Keil 配置或 HAL 文件；未自动修复错误或警告。
- 未重新生成 CubeMX 工程，未烧录硬件。
- 本次项目记忆更新只记录已完成的验证，不重新编译。

## 编译结果

- Error：0。
- Warning：1。
- 编译范围：33 个 C 文件、1 个启动汇编文件；最终链接成功。
- Program Size：Code=15072，RO-data=480，RW-data=52，ZI-data=2556（字节）。
- Keil 进程退出码：1，表示仅有警告，不表示编译失败。

唯一 Warning：

```text
../Core/Src/main.c(942):
warning: #1-D: last line of file ends without a newline
```

该 warning 为文件末尾缺少换行符，不影响程序功能，因此未修复。

## 生成产物与完整输出

- 新生成产物：`pwm_02.axf`、`pwm_02.hex`，位于临时副本的 `MDK-ARM/pwm_02/`。
- 临时验证根目录：`C:\Users\Administrator\AppData\Local\Temp\mag-keil-verify-20261001-36e027aa`。
- AXF：`C:\Users\Administrator\AppData\Local\Temp\mag-keil-verify-20261001-36e027aa\pwm_double20260926DC6output\MDK-ARM\pwm_02\pwm_02.axf`。
- HEX：`C:\Users\Administrator\AppData\Local\Temp\mag-keil-verify-20261001-36e027aa\pwm_double20260926DC6output\MDK-ARM\pwm_02\pwm_02.hex`。
- 完整编译输出：临时验证根目录下 `keil-rebuild.log`。
- 完整工具链与 Pack 构建记录：临时副本 `MDK-ARM/pwm_02/pwm_02.build_log.htm`。
- 原工程目录中的旧 .axf/.hex 保持原样；本次成功结论对应上述临时副本的新产物。
- 日志和产物保存在临时目录，后续若目录被清理，应依据本章环境配置重新建立验证副本，不能将旧产物视为当前源码的构建结果。

实际执行的命令：

```bat
"C:\Users\Administrator\AppData\Local\Keil_v5\UV4\UV4.exe" -r "C:\Users\Administrator\AppData\Local\Temp\mag-keil-verify-20261001-36e027aa\pwm_double20260926DC6output\MDK-ARM\pwm_02.uvprojx" -t "pwm_02" -j0 -sg -o "C:\Users\Administrator\AppData\Local\Temp\mag-keil-verify-20261001-36e027aa\keil-rebuild.log"
```

## 状态与后续开发

Windows Keil 开发链已经恢复，可以作为后续 STM32 固件开发基线。该结论覆盖编译、汇编和最终链接，不代替实板采样、时序、精度或电流标定验证。

后续开发基于该 baseline 继续：

- ADC/current_sense 验证。
- 六路电流反馈接入。
- 电流闭环设计。

# 第一阶段 Windows 软件验证完成（2026-10-01）

**历史阶段状态（截至 2026-10-01，接线/档位/烧录/无遥测状态已被后续更新取代）**。原阶段说明保留如下：

> 本章为最新阶段状态，承接 Windows Keil Build Baseline。之前 Linux 阶段的“Keil 待执行”等描述和全部历史记录保留。当前第一阶段的只采样软件链路及软件回归验证已完成；实板接线、实际 ADC 采样和测量准确性仍未验证。未进入第二阶段电流闭环设计。

## 只读扫描结论

- 检查了 `current_sense.c/.h`、`adc.c/.h`、`dma.c/.h`、`gpio.c/.h`、`main.c`、`stm32f4xx_it.c`、`tim.c/.h`、相关 HAL/CMSIS 实现及 Keil 工程。
- `main.c:333` 一次调用 `CurrentSense_Start()`；模块按 ADC1 初始化、TIM3 初始化、启动 ADC DMA、启动 TIM3 的顺序建立采样链路。初始化失败返回状态并记录故障，不调用全局 Error_Handler，不改变 PWM。
- ADC1 独立模式，12 bit、右对齐、扫描 6 个 rank；连续转换关闭、不连续模式关闭、TIM3 TRGO 上升沿触发、EOC 为整个序列、DMA 连续请求开启。
- ADC 时钟 PCLK2/4 = 21 MHz，每通道采样时间 84 cycles。GPIOA/B/C 时钟及六个模拟输入均在 ADC MSP 中配置，模拟模式、无上下拉。

| 顺序 / raw 下标 | 反馈 | 引脚 | ADC1 通道 | 当前状态 |
|---|---|---|---|---|
| rank 1 / raw[0] | a0 | PB0 | IN8 | 已实现 |
| rank 2 / raw[1] | a1 | PA6 | IN6 | 已实现 |
| rank 3 / raw[2] | a2 | PA3 | IN3 | 已实现 |
| rank 4 / raw[3] | a3 | PC2 | IN12 | 已实现 |
| rank 5 / raw[4] | a4 | PC3 | IN13 | 已实现 |
| rank 6 / raw[5] | a5 | PA4 | IN4 | 已实现；PA4/JP5 实板路径待确认（历史状态/已被后续更新取代：已确认正确 ADC 档） |

- DMA2 Stream0 / Channel0，外设到内存、内存地址递增、两端 halfword、循环模式、FIFO 关闭、硬件优先级 LOW；12 个 uint16_t 构成两个六通道半区。
- DMA2 Stream0 IRQ 经 HAL DMA 中断处理调用 ADC 半完成/完成回调，再进入 `PublishFrame()`；ADC IRQ 经 HAL ADC 中断处理进入错误回调。新增两个 IRQ 优先级均为 3；现有 UART=0、TIM2=2 保持。
- TIM3 时钟 84 MHz，PSC=839、ARR=199，TRGO=UPDATE，设计频率 500 Hz，每 2 ms 一组六路数据；HT/TC 交替发布完整帧。六路顺序转换总计约 27.4 µs，属于配置计算值，尚无实板测量证据，六路并非同时采样。
- `published` 缓存六路 raw、frame_count、timestamp_ms、error_flags、overrun_count、dma_error_count、dma_late_count、valid、running。时间戳为发布时 HAL tick，valid 表示完整帧且未锁存错误，不代表校准有效或自动判定数据新鲜度。
- **历史状态/消费者与链接状态已被后续更新取代**：`CurrentSense_GetSnapshot()` 是保留的前台一致性读取 API，当前 main 没有调用它，也没有串口遥测或 ADC→PWM 消费路径；本轮不添加消费者。最终链接会移除这个未引用函数，内部采样/发布缓存仍保留并运行。该 API 的实际 C 行为已通过现有测试，ARMCC 目标文件中也有正确指令；尚未在实板运行调用该 API。最新 telemetry 版本已在前台调用该 API，最终链接保留它；实板 telemetry 验证仍未完成。

## ARMCC 与运行风险复核

- 当前工程仍使用 MDK 5.43a / µVision 5.43.1.0 / ARM Compiler 5.06 update 7 (build 960) / Keil::STM32F4xx_DFP 3.1.1，未修改工具链或工程配置。
- 本轮 ARMCC 目标文件反汇编确认：快照先 MRS 保存 PRIMASK，再 CPSID i，复制完成后 MSR 恢复原 PRIMASK；DMA 数据以 halfword 读取，发布路径含 DMB。CMSIS 已分别提供 ARMCC/GCC 的对齐和屏障实现，未发现本模块必须修复的编译器行为差异。
- 本轮 MAP 确认 `dma_buffer` 位于 0x2000016C、24 bytes、4-byte 对齐；`published` 位于 0x20000184、40 bytes，均在普通 SRAM，DMA 缓冲区未放到 CCM。
- 现有保护包含 DMA 实际 BUSY/EN 检查（该 HAL 版本不会传播 HAL_DMA_Start_IT 失败返回值）、复制前后 NDTR 与对侧 sticky completion flag 检查、待处理 ADC OVR/DMA error 检查及故障锁存；故障停止 TIM3 触发并使快照失效，直到 MCU reset，不自动重启。
- 尚待实板确认 UART/其他中断压力下的延迟和 DMA 仲裁。调试暂停时 ADC/DMA 不保证同步停止，可能触发 DMA_LATE；恢复采样验证前应复位。以上软件检查和模拟 HAL 测试不证明实际外设运行、接线、采样准确度或时序。

## 本轮验证结果与保护范围

- 静态集成/原 PWM 基线保护：16/16 PASS，使用 `C:\Python314\python.exe -B tests/test_adc_infrastructure.py`。
- 实际 `current_sense.c` 的模拟 HAL C 回归：17/17 场景 PASS；使用临时解压的 Windows x64 TCC 0.9.27 和现有 `tests/run_current_sense_tests.py`，未安装系统编译器，未修改测试脚本。该测试不等同于在 MCU 上运行 ARMCC 固件。
- 17 场景覆盖正常 HT/TC 完整帧、空指针、外来 ADC handle、PRIMASK 恢复、一次启动、各启动失败、OVR/DMA 错误、重复错误类别计数、半区积压、NDTR 边界、复制期间 wrap/full wrap、待处理错误和对侧 sticky flag。
- Keil 命令行 Rebuild 在新临时副本中执行，33 个 C 文件 + 1 个启动汇编，最终链接和 HEX 生成成功：**0 Error，1 Warning**，进程退出码 1 表示有警告。
- 唯一 warning：`../Core/Src/main.c(942): warning: #1-D: last line of file ends without a newline`；保持 baseline，未修复文件末尾换行。
- Program Size：Code=15072、RO-data=480、RW-data=52、ZI-data=2556 bytes，与既有 Windows baseline 一致。
- 记录本章前，工作目录全部 275 个文件 SHA-256 均未改变；临时副本中 113 个 Core/Drivers/启动文件/uvprojx/ioc 输入的 SHA-256 也未改变。工程 SHA-256 仍为 `7F60EC238678F82662420C2B752852EA82F72C5659ED8F828D1576EC76B4F43C`。
- 本轮未发现第一阶段必须补齐的固件代码项。只向 `PROJECT_MEMORY.md` 追加本章；源码、PWM/控制算法、API、.uvprojx、.ioc、HAL、Keil 配置及原工程产物均未改，未运行 CubeMX 再生成或烧录。

## 本轮日志与产物

- 验证目录：`C:\Users\Administrator\AppData\Local\Temp\mag-phase1-verify-20261001-027b045f`；日志和产物保留于此，临时目录被清理后需重建。
- 静态检查完整输出：`adc-infrastructure-tests.log`；C 回归完整输出：`current-sense-c-tests.log`。
- Keil 完整编译输出：`keil-rebuild.log`；命令：`keil-command.txt`；退出码和执行时间：`execution-result.json`。
- 工具链与 Pack 构建记录：副本 `pwm_double20260926DC6output/MDK-ARM/pwm_02/pwm_02.build_log.htm`；MAP、AXF、HEX 位于同目录。
- 新产物：上述副本目录中的 `pwm_02.axf`（947260 bytes）、`pwm_02.hex`（43947 bytes）。原工程中的旧产物未更新，也未刷入 MCU。
- ARMCC 检查证据：`current-sense-armcc-disassembly.txt`、`snapshot-armcc-disassembly.txt`；哈希检查：`workspace-before.json`、`hash-verification-before-documentation.json`。

实际 Rebuild 命令：

```bat
"C:\Users\Administrator\AppData\Local\Keil_v5\UV4\UV4.exe" -r "C:\Users\Administrator\AppData\Local\Temp\mag-phase1-verify-20261001-027b045f\pwm_double20260926DC6output\MDK-ARM\pwm_02.uvprojx" -t "pwm_02" -j0 -sg -o "C:\Users\Administrator\AppData\Local\Temp\mag-phase1-verify-20261001-027b045f\keil-rebuild.log"
```

## 第一阶段当前状态 / 下一步

**历史状态/已被后续更新取代（截至 2026-10-01）**：下述“当前”指第一阶段验证时点；当前权威状态与下一步见文末 2026-10-03 新阶段。

- 软件：已完成只采样接入、软件回归和 Windows Keil 最终构建；本轮无待修复的软件 blocker。
- **历史状态/已被后续更新取代**：硬件接线：等待六路 CURT 接线及 PA4/JP5 路径确认。最新两项均已完成，不再是 blocker。
- **历史状态/烧录版本已被后续更新取代**：ADC 实采：等待烧录本轮对应固件，验证 raw[0..5] 引脚顺序、已知电压响应、frame_count 约 500 次/秒递增、运行状态/错误计数，以及 UART/PWM 工作时的采样连续性。用户已确认第一阶段采样版烧录；当前待烧录的是新 telemetry HEX，raw 验证仍待执行。
- 测量质量：六路 offset/gain、参考电压、方向/续流/换向特性、RC 延迟、噪声、串扰及原 PWM 对照仍需硬件证据。
- 本阶段不添加电流 PI，不改变原 PWM 逻辑；先完成接线和 ADC 实采验证，再决定后续阶段。

# UART telemetry 与 Windows 依赖恢复（2026-10-03，最新状态）

**同日软件实现与构建的详细记录**：本章内容保留；文末新增“CURT telemetry 软件完成（2026-10-03）”作为收口后的当前权威状态和实板下一步计划。

本章承接“第一阶段 Windows 软件验证完成（2026-10-01）”。历史记录不删除；本章取代旧接线、跳帽、烧录、无遥测和 Git 不可用状态。只采样基础设施保持原样，新增前台观察/记录能力；未进入电流标定或 PI。

## 最新实物状态与版本边界

以下为用户本轮明确确认的实物事实，不把软件测试当作硬件测量证据：

- 六路 CURT 已通过驱动板正式 CN4 接口完成接线。
- GND 已由师兄接好，本轮不新增 GND 线。
- PA4/PA5 功能选择跳帽已在 ADC 档；原 PA4/JP5 路径确认问题已解除。
- 含 ADC1 + DMA2 Stream0 + TIM3 + current_sense 的上一版采样固件已经构建并烧录。
- 当前实板目标是 PC 可见 raw[0..5]，验证连续性、逻辑顺序和 CURT 响应。
- **本轮生成的是新的 telemetry 固件，尚未由本轮烧录。** 用户确认已烧录的是上一版采样固件；需要再烧录本章指定 HEX 后才会出现 @ADC。
- 电流 offset/gain、符号/续流/换向特性、噪声/延迟/串扰仍未标定；六路 PI 尚未开始。
- 逻辑/物理顺序仍为 a0..a5 → `[Pole1,Pole3,Pole5,Pole4,Pole6,Pole2]`，ADC 引脚/rank 和全部 PWM/CTRL_SD 映射不变。

## 完整诊断与实现选择

- 修改前完整读取本文件，并审查 USART1 TX/RX、main 前台、RX idle callback、UART/ADC/DMA IRQ、current_sense、HAL DMA/UART 锁和完成路径、Keil 文件列表、两套上位机串口调用。
- USART1：115200、8N1，PB6/PB7；RX = DMA2 Stream2/Channel4，normal + ReceiveToIdle；TX = DMA2 Stream7/Channel4，normal；ADC = DMA2 Stream0/Channel0，circular。IRQ、DMA 优先级和所有初始化配置均未改。
- 原应用唯一 TX 在 main 主循环，使用 `tx_buf[16]` 发送 12 个无符号 echo 字节，无 CRLF。填充发生在 busy 判断之前，存在下一条命令重写 DMA 源缓冲区的问题。原应用没有统一调度器，没有 TX IT/blocking/printf 调用，也没有自定义 TX-complete callback。
- Drivers/SYSTEM/usart 中有历史 printf/直接寄存器/另一 UART handle 实现，但不在实际 Keil 文件列表，本轮不启用、不修改。
- Python 两套 GUI 持有各自单个串口对象，只发送 signed 帧，当前没有串口读取路径。Windows 同一 COM 口由一个进程持有，独立接收器与 GUI 不同时打开该口。
- 新增 `uart_telemetry.c/.h` 作为唯一前台 TX 所有者，复用现有 TX DMA 和 HAL 完成链；main 只加 include、替换原发送调用为 QueueEcho、添加一次前台 Poll。
- 保留旧 12 字节 echo 的内容。独立 12-byte latest-echo 槽保存待发内容，DMA 使用独占、4-byte 对齐的 128-byte SRAM buffer。busy 时不改 DMA buffer；UART gState、TX DMA state 和 stream EN 均空闲才复用，DMA完成但 UART TC 未完成时仍不复用。
- 每 100 ms 通过 `CurrentSense_GetSnapshot()` 获取最近完整快照；到期遥测优先于 echo，忙时下一轮前台重试，不积压旧快照、不追赶突发发送。ADC 仍为 500 Hz，初始化失败/故障快照也发送，便于看到 valid/running/error。
- 本 HAL 的 TX setup 与 RX ReceiveToIdle rearm 共用 `huart1.Lock`。仅对非阻塞 HAL TX DMA 启动过程短暂保存/屏蔽/恢复 PRIMASK，避免 RX ISR 重启时遇到锁占用；已核对 HAL_DMA_Start_IT 无等待循环。格式化、快照读取、完整 UART wire transfer 均不放进该 TX 临界区。无 RX abort/restart、无 ISR 新工作，IRQ 优先级不改。实际临界区时长仍待实板测量。
- 本 HAL 也不传播 TX 的 HAL_DMA_Start_IT 失败。模块识别 silent failure、HAL error 和 TX DMA error，锁存 TX-only 故障至 reset；不调用 Error_Handler，不停止或改变 RX/PWM/ADC。DMA完成与状态检查交错不会误报，已有回归覆盖。
- 帧采用完整建议字段，十进制无符号数，14 个字段，CRLF 结尾：

```text
@ADC,<frame_count>,<timestamp_ms>,<raw0>,<raw1>,<raw2>,<raw3>,<raw4>,<raw5>,<error_flags>,<overrun_count>,<dma_error_count>,<dma_late_count>,<valid>,<running>\r\n
```

- 12-bit raw 与 uint32 极值组合最长 106 bytes（含 CRLF），约 10 Hz；128-byte buffer 连 uint16 极值测试的 112 bytes 也容纳。正常 UART 负载与原 echo 相加有充足带宽，但中断/总线压力效果仍需硬件验证。
- 无 raw→Ampere、无闭环常数、无 PI、无 ADC→PWM；signed parser、Check_Frame、CCR公式、Start/Stop_PWM、TIM1/2/8、PF2/7/10、ADC1 rank、TIM3 500 Hz、current_sense 内部发布逻辑、HAL/ISR 均保持原字节。

## PC 接收工具与依赖

- 解释器由 README、历史验证、`python` 和 `py -0p` 确认为 `C:\Python314\python.exe`，Python 3.14.4，prefix/base_prefix 相同；没有新建 venv。
- 原环境补齐 pyserial 3.5、PySide6 6.11.2 及必需同版本 shiboken6 / Essentials / Addons。`import serial`、`import PySide6` PASS。
- numpy 2.4.4、opencv-python 4.13.0.92、pip 26.1.1 保持不变；requirements 未改，没有无关运行依赖升级。
- 新增 `dipole-control/curt_telemetry.py`：默认只读，终端逐帧打印，可保存 CSV；异步循环读取非阻塞串口，无 GUI/相机依赖。CSV 包含 timestamp_pc（UTC ISO）、frame_count、timestamp_mcu、raw0..5、error_flags、overrun_count、dma_error_count、dma_late_count、valid、running。
- parser 严格校验 CRLF、前缀、14字段、ASCII十进制、raw 0..4095、uint32/flag 范围；stream decoder 支持拆包/粘包、旧 echo 无换行、截断和超长恢复，不误解析控制帧/未知前缀。
- 终端用快照 frame_count/MCU timestamp 增量估算采样率（包含 uint32 wrap），保留故障/冻结状态，5 秒无完整帧提示等待。
- 可显式指定 `--command` 做同一端口的固定 signed 命令测试：先发全零、2 秒基线，随后按 30 Hz 目标节拍重发用户给出的原协议帧，结束时尝试全零。默认只读模式从不发送，测试代码不打开真实端口。
- mypy/black/ruff/coverage 仅放到临时 QA 目录，不安装到运行环境，不重构既有 Python 模块。

## 本轮验证结果

| 验证 | 结果 / 边界 |
|---|---|
| 原 ADC infrastructure | 16/16 PASS |
| 原 current_sense 真 C + 模拟 HAL | 17/17 PASS |
| 新 UART telemetry 真 C + 模拟 HAL | 18/18 PASS，`-Wall -Werror` |
| 新 integration / phase-1 字节保护 | 3/3 PASS |
| 新 PC parser / stream / CSV / CLI / 模拟端口 | 105/105 PASS；无 skipped |
| 新 Python 覆盖率 | 99%，未覆盖仅 __main__ 入口一行 |
| 新 Python 静态检查 | mypy --strict / black --check / ruff 全 PASS |
| 原上位机 67 项，默认环境 | 66/67；原 6 个缺 PySide6 项全部解决，无依赖导致 failed/skipped |
| 原 GUI smoke | SMOKE OK；相机/串口不连接，设置隔离 |
| 多速率时序诊断 | 默认两次最大帧间隔 60.3 / 66.6 ms，超过原 60 ms 判据；诊断子进程 OPENBLAS_NUM_THREADS=1、OMP_NUM_THREADS=1 后 18/18 PASS；项目/系统配置未改，不声称默认环境 67/67 |
| 原 signed PWM fixture | signed_pwm_baseline.json 未改、未重捕获；保护检查通过 |
| Keil 最终 Rebuild | 34 C + 1 startup；最终链接/HEX生成成功，0 Error、1 Warning |

原 test_adc_infrastructure 仅扩展 main 的三项遥测允许编辑的精确反向恢复，再验证原始 SHA；原 parser/RX/PWM/clock/IRQ/HAL/ioc 保护继续执行。另保存 21 个 phase-1 Core/ioc/project 输入哈希，防止原采样内部变化。没有用重捕获 baseline 掩盖修改。

唯一 Keil warning 仍为 main.c 末尾没有换行，本轮位置为 line 938；保留原非 UTF-8 字节和 CRLF。
最终 Program Size：Code=15700、RO-data=480、RW-data=64、ZI-data=2696 bytes。
相对 phase-1：Code +628 bytes，RW +12 bytes，ZI +140 bytes；最终链接包含原快照 API。
MAP：dma_buffer 0x20000178 /24 bytes、published 0x20000190 /40 bytes、UART tx_buffer 0x200001B8 /128 bytes、echo_buffer 0x20000238 /12 bytes，均普通 SRAM。

## 文件与构建产物

- 修改既有文件（6）：本 PROJECT_MEMORY.md；dipole-control/README.md；固件 Core/Src/main.c、MDK-ARM/pwm_02.uvprojx、tests/test_adc_infrastructure.py、tests/README.md。
- 新增源码/测试（11）：dipole-control/curt_telemetry.py、tests/test_curt_telemetry.py；固件 Core/Inc/uart_telemetry.h、Core/Src/uart_telemetry.c；tests/telemetry_main_patch.py、telemetry_phase1_baseline.json、test_uart_telemetry.c、run_uart_telemetry_tests.py、test_uart_telemetry_integration.py、telemetry_stubs/stm32f4xx_hal.h、telemetry_stubs/usart.h。
- 修改前完整快照：`C:\Users\ADMINI~1\AppData\Local\Temp\mag-telemetry-before-20261003-lvo0nr4t/workspace`，SHA 清单 before-sha256.json；初始 Git clean，现有 Git baseline 为 f178822（2026-10-02）。没有提交或改 Git 配置。
- 构建在临时副本 `C:\Users\ADMINI~1\AppData\Local\Temp\mag-telemetry-keil-20261003-jqx69yjz` 完成，没有在原工程 Rebuild 或 CubeMX 再生成。
- **本轮当前源码对应 HEX：`C:\Users\Administrator\Desktop\mag\artifacts\curt-telemetry-20261003\pwm_02.hex`**，45747 bytes。
- HEX SHA-256：`ad2ba25e68f57ea1e396d9950316dec15ab322c298859866a16e9ee90ddd75e3`。
- 同目录保存 AXF、MAP、Keil build_log.htm、keil-rebuild.log、keil-result.json、全部测试/QA日志及 source-and-output-sha256.json。
- 112 个 Core/Drivers/RTE头文件/启动文件/ioc/uvprojx 输入与最终 Keil 副本逐项哈希一致。原工程输出目录的旧 HEX/AXF 未替换；使用上述 artifacts HEX，不沿用历史旧文件。

## 仍待执行 / 下一步实板验证

- 当前开发环境 `serial.tools.list_ports` 未检测到 COM 口，本轮没有连接板卡、发送实际控制命令或烧录；实际 raw/时序/质量尚无本轮测量证据。
- 先烧录上述 **telemetry 新 HEX**，退出 Keil 调试/长断点并复位，关闭占用 COM 的 GUI/串口助手，确认实际端口。
- 在 dipole-control 中运行 `C:\Python314\python.exe -B curt_telemetry.py --port COM4 --duration 30 --csv curt_raw.csv`（COM4 替换为实际端口）。检查 10 Hz @ADC、raw[0..5] 持续输出、valid=1、running=1、error_flags 和三个错误计数为0、frame_count 总体约500/s增长。
- 显式 `--command` 小指令逐路测试，建议先仅 a0:+05，其他 +00，记录2秒零基线和随后响应；确认主变化 raw[0]。依次 a1..a5 对 raw[1]..raw[5]，物理顺序不重排。
- 用该单端口测试同时重发控制帧和接收遥测，检查 PWM/UART 工作时 ADC 连续性；保存CSV并核对计数/错误/冻结状态。测量新增短临界区耗时和优先级/总线压力实际影响。
- 默认 Python BLAS 线程时序越限保留为独立性能事项；本轮未修改 MPC、GUI、Kalman、模型或运行参数来消除它。
- 完成真实 raw 验证后，再做单路 CURT 与外部电流表对照；校准和六路 PI 留后续阶段。

# CURT telemetry 软件完成（2026-10-03）

本章为当前权威阶段记录，汇总同日已完成的软件工作。前述历史阶段全部保留；接线待完成、PA4/JP5 待确认、Python 依赖缺失和 telemetry 尚未实现等旧状态均为**历史状态/已被后续更新取代**。

## 最新权威状态与完成边界

- 六路 CURT 实物接线已完成，使用正式 CN4 控制/采集接口。
- PA4/JP5 功能选择已确认在正确 ADC 档；GND 使用原有系统接线，已连接，不是当前 blocker。
- ADC1 + DMA2 Stream0 + TIM3 TRGO + current_sense 第一阶段软件已完成，六通道采样保持 500 frame/s 的设计配置；上一版采样固件已烧录（用户确认）。
- Python 环境 `C:\Python314\python.exe` 已安装 PySide6 6.11.2、pyserial 3.5；`import PySide6`、`import serial` 均 PASS。
- STM32 已新增 `Core/Src/uart_telemetry.c`、`Core/Inc/uart_telemetry.h`，约 10 Hz 发送最近完整快照；底层 ADC 六通道采样仍保持 500 frame/s。
- 遥测从前台读取 `CurrentSense_GetSnapshot()`；ADC/DMA ISR、ADC callback、HT/TC callback 中不做 UART 发送。
- USART TX 使用单一前台调度器及独占 DMA buffer，发送忙时不覆盖 DMA 源数据；保留原 UART echo 和 ReceiveToIdle RX 控制链。
- `dipole-control/curt_telemetry.py` 默认只读接收 telemetry，终端显示 raw 和状态，支持 CSV 记录；显式 `--command` 是可选的同端口逐路测试功能，默认只读模式不发送命令。
- 本阶段完成范围为软件实现、回归验证及 Keil 最终构建；raw→Ampere、电流标定、ADC→PWM 和六路 PI 均未实现。

telemetry 最终帧格式（14 个十进制字段，CRLF 结尾）：

```text
@ADC,frame_count,timestamp_ms,raw0,raw1,raw2,raw3,raw4,raw5,error_flags,overrun_count,dma_error_count,dma_late_count,valid,running\r\n
```

## 验证与当前源码产物

以下为本轮已完成验证的记录，本次文档更新未重跑测试或构建：

| 验证项 | 结果 |
|---|---|
| ADC 静态检查 | 16/16 PASS |
| current_sense C 回归 | 17/17 PASS |
| telemetry C 回归 | 18/18 PASS |
| 集成保护 | 3/3 PASS |
| PC parser / recorder / CLI | 105/105 PASS |
| mypy / black / ruff / GUI smoke | PASS |
| 原上位机测试，默认环境 | 66/67；原缺依赖问题已解决，仍有一项线程时序越限 |
| 线程限制诊断，多速率组 | 18/18 PASS；只在诊断子进程限制 BLAS/OMP 线程，项目配置未改 |
| Keil Rebuild | 0 Error、1 个已有 newline warning；最终链接及 HEX 生成成功 |

Program Size：`Code=15700, RO=480, RW=64, ZI=2696` bytes。

当前源码对应的新 HEX：

```text
C:/Users/Administrator/Desktop/mag/artifacts/curt-telemetry-20261003/pwm_02.hex
```

**这个 telemetry 新 HEX 目前尚未烧录到 STM32，尚未完成实板 telemetry/raw 验证。** 不将软件测试通过或上一版采样固件已烧录记作本版实板验证通过。构建输入、产物哈希与完整日志位于同目录，验证汇总为 `verification-summary.json`。

## 当前下一步：实板 raw 验证的九步顺序

1. 烧录上述 telemetry 新 HEX，复位 STM32，释放 GUI/串口助手对同一 COM 口的占用。
2. 在 `dipole-control` 目录运行 `curt_telemetry.py`，使用实际 COM 口；例如 `C:\Python314\python.exe -B curt_telemetry.py --port COM4 --duration 30 --csv curt_raw.csv`，COM4 替换为实机端口。
3. 验证 `raw[0..5]` 连续输出并保存记录，不做安培换算。
4. 验证 `frame_count` 总体约 500/s 增长；约 10 Hz 遥测下，相邻正常帧通常约增加 50。
5. 验证 `valid=1`、`running=1`。
6. 验证 `error_flags / overrun / dma_error / dma_late` 正常，正常运行时错误位和对应三个计数均为 0；同时观察 UART 控制/PWM 工作时是否连续、是否冻结或丢失。
7. 逐路驱动并验证以下对应关系，保持原逻辑顺序和物理映射：

   | 指令通道 | 对应 raw | 物理磁极 |
   |---|---|---|
   | a0 | raw[0] | Pole1 |
   | a1 | raw[1] | Pole3 |
   | a2 | raw[2] | Pole5 |
   | a3 | raw[3] | Pole4 |
   | a4 | raw[4] | Pole6 |
   | a5 | raw[5] | Pole2 |

8. 实板 raw 验证通过后，再做外部电流表对照和电流标定，包括正反向、续流/换向、offset/gain、噪声与延迟。
9. 六路 PI 电流闭环仍属于后续阶段；本阶段到真实 raw 采样验证，不用 ADC 数据改变 PWM。

本次仅更新 PROJECT_MEMORY.md 的摘要、历史状态标识和阶段记录；未修改任何源码、测试、工程配置或构建产物。


# 强化学习 RL / ES-MLP 从正式工程移除（2026-10-04）

## 范围与历史关系

原项目曾存在 PD / PID / MPC 实机自动控制及独立 RL / ES-MLP 实验路线。2026-10-04 开始收敛为正式 MPC-only baseline：上一阶段完成主 GUI 实机 PD/PID 分支清理，本阶段只删除强化学习路线。PD/PID/RL 属于 historical / superseded，不再属于当前生产实机自动轨迹控制路径。独立仿真器的 PID 不受此结论影响，本轮未改。

本轮修改前记录了 Git status、HEAD 和 worktree，HEAD 为 `f178822199efe485e845018ecaa62a946e46fee3`，分支为 `main`。工作区已有前期 MPC、telemetry 和固件改动；本轮逐文件快照/哈希以修改前工作区为基准，没有覆盖这些改动。所有删除都是 tracked 文件的普通工作区删除，可由 Git 恢复；未 commit、push、git clean 或 reset。

## 依赖确认与删除清单

依赖方向为 `rl_path_control → rl_policy / 主 GUI 公共工具 / DipoleSolver / config`、`train_rl → rl_policy`、`tests/test_rl → rl_policy / train_es / rl_path_control`。主 GUI、mpc、multirate、dipole_solver、estimators、friction_model 均未导入强化学习模块。被 RL 借用的 `VideoLabel`、`detect_bead`、`build_command`、`apply_slew` 等公共代码仍留在原模块，不随 RL 删除。

删除六个 RL 专用文件：

- `dipole-control/rl_path_control.py`：独立强化学习实机 GUI / 策略部署入口。
- `dipole-control/rl_policy.py`：MLPPolicy、状态/动作映射、PathEnv、ES 训练与评估。
- `dipole-control/train_rl.py`：ES-MLP 训练命令入口。
- `dipole-control/tests/test_rl.py`：9 项强化学习专用测试及其辅助代码。
- `dipole-control/models/rl_policy.json`：明确标记 ES-MLP 的策略权重。
- `dipole-control/rl_gui_settings.json`：独立 RL GUI 设置。

正式工程未发现其他 RL-only checkpoint、训练输出、CSV 或启动脚本；历史审查报告、Git 历史及 `artifacts/` 验证日志保留为 historical。`requirements.txt` 无 RL-only 依赖，未改；`config.py` 和主 GUI 设置未改。

README 移除当前 RL GUI、训练、策略加载、运行命令和文件清单，保留简短 historical / removed 说明，并解释 ESO 与 R-L 术语。主 GUI 只替换一行过时 RL 文档引用注释；没有修改函数、控件或控制行为。旧公共 CSV 表头属于历史数据格式说明，本轮不再扩大 PD/PID 清理。

## 保持的正式主链及辅助功能

```text
Camera → Vision → Kalman / ESO → Path Reference → SharedState
  → ControlWorker / ForceMPC（约 10 Hz）→ DipoleSolver
  → CurrentExecutor（约 30 Hz）→ send_commands() → Serial → STM32
```

`KalmanFilter2D`、`ESO1D`、`fal()`、Fz 减摩、模型 `N30LM_six_coils_180dipoles.json`、target B/F、六路联合逆解、signed UART 协议与 a0...a5 顺序均保持。`CurrentExecutor.I_est` 和 `L dI/dt = V - R I` 的精确离散更新保留；它是估计电流，不是 CURT 实测反馈。`test_rl_equation` / `test_rl_steady_state` 检查该电阻—电感模型，保留。

手动六路电流、手动磁力、诊断、路径、视觉和标定保留。`bead_sim.py` / `bead_sim_gui.py`、ParameterSweep、ExperimentFitter、仿真 PID 未改。`curt_telemetry.py`、ADC、固件、电流标定与 PI 均未改；ADC 底层 500 Hz、telemetry 约 10 Hz 的既有设计不变，主 GUI telemetry 集成仍属后续阶段。

保护证明：主 GUI 完整 AST 与本轮修改前相同；其他 MPC 核心、状态估计、摩擦、模型、设置、仿真、共享测试、telemetry 和全部固件文件字节哈希与本轮基准相同。全局剩余匹配逐行分类为电气 R-L（B）、历史文档（C）、普通英文/第三方（D）或共享 IRLS 求解代码（E），未发现当前活动强化学习残留（A）。

## 本轮验证与既有 blocker

- 真实 `__main__` GUI 入口、模型加载及已保存 MPC 参数 PASS；使用 offscreen，不打开硬件、不改设置。
- GUI smoke PASS：合成视觉、路径、MPC 开始/停止/重启、真实工作线程、CSV、手动控制和诊断；使用内存串口。
- shared control 6/6 PASS；multirate 18/18 PASS；solver / protocol / safety 25/25 PASS；MPC 专项 6 PASS；CURT telemetry 105 PASS。
- 原生全项目 pytest：181 passed、4 errors、288 subtests passed。4 个原有装饰器辅助函数 `test(fn)` 被 pytest 误收集，分别位于 test_bead_sim、test_control、test_multirate、test_solver；本轮未改这些共享测试。
- 复用上一阶段仅放于 artifacts 的 pytest 兼容适配器（跳过上述辅助函数并把旧装饰器吞掉的断言失败重新上报）：181 passed、288 subtests passed。旧 RL 默认策略路径导致的 test_deploy_pipeline 失败随 RL-only 测试删除，未声称修复 RL 策略部署。
- mypy 主 GUI 使用 `--ignore-missing-imports --follow-imports=silent` PASS；全上位机相同参数仍有 7 个既有类型诊断（solver 及旧共享测试）。默认参数另有 2 个 pyserial 类型桩缺失诊断，修改前 GUI 快照在同一环境下也出现相同 2 项。未安装依赖或类型桩。
- ruff：147 → 104 个诊断，减少项均随 RL 文件删除，新增 0。
- black check：19 → 15 个未格式化文件，减少项均随 RL 文件删除，新增 0；未整仓格式化。

既有 pytest 收集、类型和格式问题没有为本轮“全绿”而扩大修改。以上均为软件/模拟端口验证，不代表真实硬件控制或 telemetry 实测完成。

完整依赖扫描、逐行残留分类、Git 状态、测试日志和保护证明位于 `artifacts/rl-cleanup-20261004/`；此前 `artifacts/mpc-only-20261004/` 保留为历史阶段记录。下一步可单独规划主 GUI 的 CURT 监视集成，或另开维护阶段处理旧测试/静态检查；本轮没有实施这些后续工作。


# CURT 主 GUI 集成完成（2026-10-04，Phase A / B）

## Phase A：已授权 checkpoint

重新记录 git status、diff/stat、cached diff、log、HEAD、worktree；改前 HEAD 为 `f178822199efe485e845018ecaa62a946e46fee3`，暂存区为空。显式 stage 7 个已修改文件（PROJECT_MEMORY、README、config、gui_settings、主 GUI、smoke_test、test_control）和 6 个 RL tracked deletion，不使用 git add . / 未核对的 add -A。

核对完整 cached diff 后，按用户明确授权创建一次 commit：`10f6c164608aa8af83872cbe3420666e3ef79879`，消息 `refactor: keep mpc-only control and remove rl stack`，13 文件、563 insertions / 2032 deletions，未 push。确认提交路径精确匹配允许清单后才开始 Phase B。

checkpoint 后仍有 5 个此前固件修改（main.c、uvoptx、uvprojx、固件 tests README、test_adc_infrastructure），以及既有 untracked 的独立 curt_telemetry.py/测试、UART telemetry 固件资产和 artifacts。这些均不属于此次 checkpoint 边界，原样保留。PROJECT_MEMORY 里的早期 telemetry 记录仅为历史进度，不表示这些源码已纳入 checkpoint。

## Phase B：软件架构与文件边界

- `dipole-control/magnetic_dipole_pid.py`：新增 ADC monitor 状态、两个 Qt timer、“CURT / ADC”Tab、RX/健康/CSV 槽函数；局部扩展初始化、UI、toggle_serial 和 closeEvent。
- 新增 `dipole-control/adc_csv.py`：ADCLogger，仅把不可变 CSV 行交给有界队列和独立磁盘线程；不访问串口、Qt 或控制状态。
- 新增 `dipole-control/tests/test_gui_curt_telemetry.py`：27 项集成/故障隔离/生命周期/CSV 测试。
- README 增加 GUI 使用与健康/CSV说明，独立 CLI 保留；本记忆只新增最新摘要和阶段，旧文字完整保留。
- curt_telemetry.py 及原 105 项测试字节不变，CLI 行为不变；requirements/config/gui_settings、仿真器及所有固件文件没有因 Phase B 改动。

```text
Camera → Vision → Kalman / ESO → Path Reference → SharedState
  → ControlWorker / ForceMPC（约 10 Hz）→ DipoleSolver
  → CurrentExecutor（约 30 Hz）→ send_commands() → self.ser TX → STM32
STM32 ADC/current_sense → @ADC → 同一个 self.ser RX
  → ADCStreamDecoder / ADCSnapshot → CURT Tab / health
  → bounded ADC CSV queue → disk writer
```

主 GUI 只保留原有一次 Serial 构造与一个 RX reader。RX 的 15 ms QTimer 属于 GUI 主线程，不依赖相机、tracking 或 Tab；检查 in_waiting，只读当前可用数据，每次最多 1024 bytes，timeout 从 0.1 改为 0（仅接收等待语义改变，发送函数不变）。不用 readline，不在 control_step/mpc_track_step/ControlWorker 中读取串口，不调用独立 CLI main()。

复用既有 ADCStreamDecoder/ADCSnapshot 及 CSV_HEADER/csv_row，无第二套 parser；拆包、粘包、未知前缀、旧 12-byte echo、损坏帧及重同步均验证。RX/decoder 异常显示错误与计数，不能改变控制状态或发送。原 TX 错误处理仍按原函数关闭串口并置 None，下一次 RX tick 停止 reader；未修改 send_commands()。

连接成功后清空上一会话 decoder/快照/计数并启动 RX；断开先停止 RX/ADC 记录、清半帧和旧快照，再按原流程急停并关闭串口。普通停止/急停保持观察，重连不自动续记旧 CSV。closeEvent 停止接收/显示/控制定时器，执行原有急停和资源释放，之后才进行最多 1 秒的 CSV 退出等待，不让写盘延迟急停；存储设备挂起时不保证已完成全部保存。

## Tab、健康和 CSV

六路顺序固定 `[a0,a1,a2,a3,a4,a5]`；UI 额外标记 Pole `[1,3,5,4,6,2]`，每路显示当前 GUI command 和 **ADC raw（未标定）**，不按物理极号重排，不显示为真实 Ampere。command 与 MCU ADC 快照不严格同步，I_target/I_est 没有混入 raw。

显示 frame_count、MCU timestamp_ms、UTC PC receive time、基于 monotonic 的 data age、valid/running、十六进制 error_flags、overrun/dma_error/dma_late_count，附接收帧/字节、拒绝帧和 RX/parser 错误计数。100 ms 独立显示 timer 更新年龄与 CSV 状态，不读取串口。

LIVE = 新鲜且 valid/running=1、error_flags=0；超过 500 ms 没有完整 ADC 帧或连接后仍未收帧为 STALE；故障快照或 RX/parser 异常为 INVALID；未连接为 DISCONNECTED。stale 保留最后 raw 并标年龄，断开清除旧 raw。所有健康状态只用于观察，不改变 MPC/PWM。

选择新路径、开始/停止 ADC 记录；使用现有 15 列 CSV（UTC timestamp_pc、frame_count、timestamp_mcu、raw0...raw5、错误计数、valid/running），与 MPC CSV 完全分开，拒绝覆盖已有文件。256 行有界 Queue，GUI 用 put_nowait，不等待磁盘；文件由专用线程创建/逐行写入/刷新/关闭。队列满或写盘失败会明确显示并停止记录，控制继续。断开结束当前日志，重连需重新开始新文件。

## 软件验证与保护证明

- 新 GUI 集成 27 PASS；原 CURT parser/CLI 105 PASS；真实 GUI __main__、已保存 MPC 参数、模型、事件循环和 timer 退出清理 PASS。
- GUI smoke PASS：视觉/路径、MPC 开始/停止/重启、手动六路电流、手动磁力和诊断；shared 6/6、multirate 18/18、solver/protocol/safety 25/25、MPC/worker 专项 6 PASS。
- 原生根目录 pytest：208 passed、4 个既有 test(fn) 装饰器收集错误、288 subtests passed；仅复用 artifacts 兼容适配器后：208 passed、288 subtests passed。共享测试没有为修复历史错误而改动。
- 三个 Python 改动/新增文件的 mypy（ignore-missing-imports + follow-imports=silent）PASS；新 ADCLogger strict PASS。全上位机仍有原有 7 个类型诊断；默认修改文件检查仍有原有 2 个 pyserial 类型桩缺失问题。
- ruff 原有 104 项 → 104 项，新增 0；black 原有 15 个需格式化文件 → 15 个，新增 0，两个新文件 PASS；只格式化新 ADC 方法及新文件，未整仓格式化。
- 新监视/CSV函数 stdlib trace + dis 行覆盖 97.51%，不声称整个历史 GUI 的覆盖率；未安装依赖或 coverage 包。
- 合成串口 300 次测量：单帧 RX 平均约 0.12 ms、14 帧突发约 0.40 ms，最大分别约 0.77/0.67 ms；这是软件测量，不代替真实相机/串口调度验证。
- CURT Tab 已用模拟端口做实际布局截图。offscreen 默认字体缺字，仅预览显式加载既有系统雅黑字体；生产代码未改字体。

271 个受保护基准文件字节一致，包括 MPC/multirate/solver/estimators/friction、模型、配置、设置、旧测试、CLI、仿真和全部固件。主 GUI 只有 __init__、_build_ui、toggle_serial、closeEvent 四个既有函数改变；其他既有函数逐个比较完整源码完全相同，包含 send_commands、视觉/状态估计、路径、MPC、手动控制和诊断。没有恢复 PD/PID/RL，没有标定 raw→A、ADC反馈或电流 PI。

审查证据在 `artifacts/curt-gui-20261004/`：phase-a.json、checkpoint-status、cached diff、protection-check.json、test-results、质量日志、new-code-coverage、rx-benchmark 与 GUI 预览。临时 QA 工具副本/缓存已清理，不进入 Git。

## 后续实板操作（本轮未执行）

1. 由用户确认并烧录 `artifacts/curt-telemetry-20261003/pwm_02.hex`，复位；本轮未重新构建或烧录固件。
2. 关闭独立 curt_telemetry.py 与串口助手对同一 COM 的占用，在 dipole-control 目录启动主 GUI，选实际 COM 后连接。连接按原流程发送全零帧；先保持控制停止，可先不打开相机。
3. 在 CURT / ADC 页观察 LIVE、raw0...raw5、valid=running=1、error_flags 与错误计数，以及 timestamp/data age 是否持续更新；正常约 10 Hz telemetry 对应 frame_count 每帧约 +50、总体约 +500/s。
4. 选择新的 ADC CSV，开始记录后停止，检查独立 15 列和逻辑 raw 顺序；断开/重连验证旧半帧/快照不延续，普通停止/急停后接收仍工作。
5. 用户需要时在原安全流程下逐路低幅手动驱动，确认 raw0...raw5 对应 Pole1/3/5/4/6/2；再开相机/小路径验证 MPC 与观察共存，不能把 raw 当成真实电流或反馈。
6. 保存实板结果后，再单独规划外部电流表标定及后续 PI；Phase B 是否建立第二个 checkpoint 由用户审查决定，本轮没有第二次 commit 或 push。


# CURT 分层 Git checkpoint（2026-10-04）

用户本轮明确要求将现有 CURT 工作整理成底层 telemetry、GUI 集成两个独立提交。本文早期“Phase B 尚未提交 / 等待审查决定”是历史记录，当前提交边界以本节与页首摘要为准。本轮只整理提交并更新记忆，没有继续修改功能。

- 前置 MPC-only checkpoint：`10f6c164608aa8af83872cbe3420666e3ef79879`。
- 底层 telemetry：`7e624ccfc9e1e54b3b2feb9a317d36213e2c25b2`，`feat: add CURT telemetry pipeline`，16 个文件。包含 STM32 `main.c` 的既有 telemetry 接入、`uart_telemetry.h/c`、两个 Keil 工程配置、固件测试/桩/保护 fixture 与 tests README，以及独立 `curt_telemetry.py` 和 parser/CLI 测试；不包含 GUI 集成。
- GUI 集成：本节所在提交，`feat: integrate CURT telemetry into MPC GUI`，仅 5 个文件：`dipole-control/magnetic_dipole_pid.py`、`dipole-control/adc_csv.py`、`dipole-control/tests/test_gui_curt_telemetry.py`、`dipole-control/README.md`、`PROJECT_MEMORY.md`。SHA 可由 `git log -1` 获取。
- 每次提交前均核对完整 `git diff --cached`、`git diff --cached --stat` 和精确文件清单；CRLF 文件使用 `core.whitespace=cr-at-eol` 进行 whitespace 检查，保留原编码与换行。artifacts、临时日志、CSV、缓存均未纳入提交，未 push。
- 本轮复核：parser/CLI 105 项与 GUI 27 项，共 132 PASS；ADC 基础设施 16 PASS；固件 telemetry 集成保护 3 PASS。首次 pytest 因系统临时目录权限产生 fixture 错误，指定工作区 artifacts 内的独立临时目录后通过；没有因此修改源码或测试。
- 主 GUI 与前置 MPC-only checkpoint 对比：105 个既有函数源码一致，仅初始化、UI、串口生命周期与退出清理四处包含此前完成的 CURT 接入。MPC、solver、multirate、Kalman、ESO、R-L current model 及发送函数保持。
- 7 个既有 mypy 问题和 2 个 pyserial 类型桩问题留待后续，未修复、未安装类型桩。实板烧录、CURT 标定及电流 PI 仍未执行。


# 上位机类型维护（2026-10-04）

修改范围：`dipole_solver.py` 仅在 __main__ 自测区导入 Callable 并声明 TESTS 的 name/callback 二元组列表；`tests/test_solver.py`、`tests/test_multirate.py`、`tests/test_bead_sim.py` 仅声明 RESULTS 的 name/pass/error 三元组列表。三个 sum(1 for ... if not ok) 原文未改，列表元素类型明确后其重载推断错误同步消失。

安装 typeshed 项目的 pyserial 类型桩后，原 try/except ImportError 的 _serial/list_ports = None 分支暴露两个 Module/None 赋值诊断。`magnetic_dipole_pid.py` 仅新增 ModuleType 导入与两个 ModuleType | None 模块级声明；保留完整导入分支、串口处理及所有 GUI 函数原文，没有宽泛 Any、ignore_missing_imports 或 type: ignore。

使用 --no-deps 安装纯 .pyi 类型包 `types-pyserial==3.5.0.20260712`；其元数据明确针对 pyserial==3.5.*，无运行 .py 模块。pyserial 3.5 的 28 个源码文件 SHA-256 与安装前一致；numpy、OpenCV、PySide6 版本不变。mypy 2.4.0 / ruff 0.16.10 / black 26.5.1 与历史 QA 版本一致，仅安装到 artifacts 的隔离目录；新增 `dipole-control/requirements-dev.txt` 固定以上检查工具与类型桩，运行 requirements.txt 原字节保持。

验证：全上位机 mypy 默认跟随导入检查和原 follow-imports=silent 检查均 0 error（19 文件）；GUI smoke PASS；MPC 专项 6/6（复用既有 artifacts pytest 兼容适配器以捕获旧装饰器吞掉的断言）；multirate 18/18；solver/protocol/safety 25/25；共享控制 6/6；CURT parser/GUI 132/132；仿真基础 8/8。测试进程沿用单 BLAS 线程诊断设置与工作区临时目录，未改控制线程或 MPC 10/30 Hz 配置。

ruff 104 → 104 个原有诊断，按文件/规则/信息对比新增 0；black 15 → 15 个需格式化文件，文件集合一致。没有扩大到 lint 清理或整仓格式化。源码保护检查确认五个修改 Python 文件去掉新增类型信息后 AST 一致，所有既有函数和类定义原文一致，包含 DipoleSolver 类与 send_commands()；其余控制、MPC、CURT、模型和固件保持字节一致。本轮未 commit 或 push。
