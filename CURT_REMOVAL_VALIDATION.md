# CURT/ADC 清理与控制修复保留验证（2026-10-28）

当前仓库已移除测量链，保留修复后的 MPC-only 控制系统。正式原工程构建和镜像验收完成；未创建提交、未 push、未烧录、未连接真实硬件。

## 基线与保护

- 分支 main，HEAD `0beec9bd8fed75b505dfb3de41eab9c28257609e`，upstream origin/main；开始和结束的暂存区均为空，HEAD 未变。
- 开始时已有 `dipole-control/gui_settings.json` 修改，以及未跟踪的 artifacts/、csvvvv/。用户设置文件与开始时逐字节相同，COM5、相机0及用户参数均保留。
- 11份历史 CSV（3263行）逐文件 SHA 不变。历史 artifacts 未清理。旧 canonical HEX/AXF/MAP 保存于本轮 old-images/，原构建输出由正式构建入口归档到 release/old-output/。
- 修改前220个 tracked 文件已复制并记录 SHA；staged/unstaged diff、untracked清单、分支及提交记录均保存于 `artifacts/curt-removal-20261028/`。
- 正式构建保留默认 dirty 拒绝。为遵守不提交的约束，显式使用审查后的 InputManifest，绑定HEAD、完整固件status、154项输入/删除路径和构建脚本/验收器SHA；构建后再核对。没有忽略dirty或将HEAD冒充实际源码版本。

## 三方差异分类

师兄原始控制文件的依据是未修改的 `tests/signed_pwm_baseline.json`，完整 SHA 为
`fa5d005b9dede4168a274a3b53b8451e56605eea97418851d5f902022dad9894`。
仓库最初 Git 提交已经包含早期 ADC 增量，因此没有把最初 Git 版本直接当作这个原始文件。
当前 main 精确逆转历史遥测、ADC和watchdog补丁后，才能得到上述 SHA。

计划文件先生成到 artifacts/planned/ 并与原始基线、当前HEAD比较；实际修改从当前代码精确移除CURT增量。没有用旧文件覆盖控制链。

| 差异类别 | 当前与最终版本的处理及依据 |
|---|---|
| 原有控制功能 | 43字节有符号协议、六路索引、CCR=`4200+42*command`、PWM通道/极性、UART/GPIO/时钟、物理Pole `[1,3,5,4,6,2]` 保留；历史函数/文件/Cube参数夹具验证 |
| 既有Bug修复 | 下表列出的提交对应当前控制模块保持原字节；main的12个控制/安全函数保持原字节，106个GUI方法连同方法源码保持相同 |
| CURT专属增量 | 删除ADC采样、TIM3触发、Stream0采样IRQ、快照/遥测、GUI页面/接收/CSV及专属测试；同步工程与构建验收规则 |
| 回传生命周期适配 | 恢复12字节HAL_UART_Transmit_DMA；READY检查提前包住TX缓冲写入；保留TX/RX共享HAL锁的短IRQ临界区及PRIMASK恢复。不会覆盖在途DMA数据，忙碌回传不阻塞控制 |
| 共享资源适配 | 15个测试消费者迁至gui_test_support；100ms日志状态定时器保留并更名；控制日志schema升为2 |
| 历史测试适配 | 历史GUI从固定Git对象临时加载自身已移除的依赖，临时sys.modules映射结束即恢复；不将旧测量模块恢复到生产目录 |
| 旧设置测试问题 | 清理前已有测试假定用户设置没有新mode键；现在只在测试副本中pop对应键，构造真实缺省场景，用户设置原字节不变 |

最终GUI仅8个既有方法改变：`__init__`、`_build_ui`、`_tab_control_log`、`_control_log_metadata`、`_finish_control_log_tick`、`toggle_serial`、`_latch_serial_fault`、`closeEvent`。改变分别是移除测量状态/页面/列和接线、保留日志刷新、删除串口/关闭流程中的ADC清理。连接首零帧、故障锁存、worker停止、急停和日志收尾顺序保留。

最终main相对原始文件，仅保留既有watchdog/RX快照/非法数字检查补丁，加上述echo缓冲/锁保护。`test_only_watchdog_and_echo_buffer_guard_differ_from_original_main` 精确逆转这些允许差异后核对原始SHA，未重新采集原始夹具。

## 既有修复逐项保留证据

| 修复 | 提交 | 源码保留证据 | 针对性回归 |
|---|---|---|---|
| FirstOrderESO缺帧语义 | `8ad47f5` | estimators.py原字节；GUI `_restart_first_order_eso` / `mpc_track_step` 原源码。gap或dt>0.15只重锚z1并保留z2；生命周期reset才清零 | test_eso_gap、test_eso_gap_simulation，含gap恢复、dt边界、reset与OFF/ON |
| ESO/MPC一致性 | `9d0479e` | estimators.py、mpc.py、multirate.py原字节；FirstOrderESO+steady_state默认保留 | test_eso_mpc、test_gui_eso_mpc、test_control_mode_simulation |
| 视觉丢帧与新鲜度 | `93c2e46` | SharedState/worker/executor原模块字节；GUI tick和控制路径原源码，monotonic与stale门槛保留 | test_freshness、test_gui_freshness、缺帧闭环 |
| 路径进度 | `8549352` | path_progress.py、multirate.py原字节，GUI路径方法原源码 | test_path_progress、test_gui_path_progress、历史回放 |
| 串口故障锁存/heartbeat | `4f18f88` | `_write_serial_frame`、`_serial_control_allowed`、tick原源码；`_latch_serial_fault`只删除ADC清理调用，原锁存/关串口/停worker保留 | test_serial_safety、test_error_paths，含短写/异常/首零帧/重连/IDLE heartbeat |
| 急停与线圈扫描重启 | `c0bd306` | `_clear_active_modes`、normal_stop、emergency_stop、扫描控制方法原源码 | test_stop_modes，包含真实模式入口及停止后继续tick不重新通电 |
| 300ms watchdog/CTRL_SD/零输出恢复 | `45f18be` | 6个watchdog函数及RX/CCR回调逐字节不变；GPIO三组与TIM1/TIM8配置夹具不变 | 30个生产C场景、7项集成测试；299/300/301ms、回绕、归零、自然update证明、恢复过期、快照竞争 |
| 相机/模型坐标变换 | `93add86` | frames.py原字节；GUI坐标转换与求解调用方法原源码 | test_frames、test_gui_frames，含非单位变换 |
| 10Hz目标箱/30Hz slew | `2a1be54` | multirate/config原字节，box27及每帧9限幅保留 | test_target_box、test_target_box_simulation、原父box9回放 |
| 三帧插值与执行层 | `75e02cc` | CurrentExecutor所在multirate.py原字节；legacy_three_frame默认保留，R-L估计未改 | test_interpolation、test_interpolation_simulation、父版本回放 |

MPC、solver、multirate、estimators、frames、path_progress、friction、config、模型及运行/开发依赖文件均与清理前SHA一致。C函数及106个GUI方法的完整名称/SHA证据见 `final-protection.json`。

## 移除内容与一致性

- 27个经过确认的tracked专属文件删除：两个PC模块、两个PC测试、六个Core源/头、五个ADC HAL/LL文件，以及12个采样/遥测测试与夹具。通用C提取器先迁至 `tests/c_source.py`。
- main取消CurrentSense启动及UARTTelemetry调用；保留直接DMA幅值回传。GPIO/UART/时钟文件不改。
- 清除TIM3 TRGO、DMA2 Stream0采样初始化与强ADC IRQ；HAL_ADC_MODULE_ENABLED及专属include关闭。通用CMSIS、HAL legacy宏、startup弱向量保留。
- .ioc清除六个ADC引脚和专属关联；IP=8、引脚=25、DMA请求=2。原TIM1/2/8、USART和CTRL_SD参数按夹具核对。
- uvprojx与uvoptx各精确删除5个源文件条目；两表顺序、路径和文件编号连续性有自动测试。
- 没有CURT专属第三方运行依赖；pyserial等仍服务控制，因此requirements原字节保留。

## 控制日志与串口生命周期

- schema_version=2；控制CSV由106列减为95列，worker仍56列。
- 删除11列：adc_frame_count、adc_age_ms、raw0..raw5、adc_valid、adc_running、adc_error_flags；同时删除offset.adc_raw及semantics.ADC。
- 其余字段顺序/类型/语义不变；测试核对表头与行、类型、run/seq、异步写盘、错误隔离、日志开启/关闭控制输出一致。
- control_log_status_timer继续每100ms刷新日志；关闭路径停止该timer。CURT页面和RX timer已不存在，GUI不解析/等待测量数据，也不消费12字节echo。
- 没有统一控制日志的生产读取器；仿真实验CSV入口按t/x/y列名校验，缺列报错。旧schema 1文件不改写，外部分析须先检查schema并按列名读取，禁止套用新位置索引。详见 CONTROL_LOG.md。

## 验证结果与清理前对照

| 检查 | 结果 |
|---|---|
| 清理前完整上位机pytest | 721通过、1个既有设置假设失败；完整失败日志保留 |
| 清理后完整上位机pytest | 592通过，10条既有绘图warning；新增生命周期测试包含在内 |
| 四个既有脚本式测试独立入口 | 清理前后均为shared6、solver25、multirate18、bead8，全部通过；未修改其旧装饰器或降低时序阈值 |
| 上位机合计 | 649项通过，另GUI smoke通过；串口/停止/日志113项专项通过（是上述测试子集，不重复计数） |
| 生产固件C | 清理前29/29，清理后30/30；新增DMA busy echo，TX调用同时检查IRQ临界区 |
| 1194帧对照 | 原始与最终trace逐字节相同；清理前HEAD的所有已有trace字段也相同。每帧新增记录的12个echo字节全部核对 |
| 固件/镜像单元测试 | 17项、127个subtests通过：控制基础设施4、watchdog集成7、验收规则6 |
| 正式构建流程逻辑测试 | 21/21；包含默认dirty拒绝、显式清单漂移拒绝、路径/clean/fresh/SHA保护 |
| 整镜像故障注入 | 10/10：正确镜像允许、旧@ADC镜像拒绝、强采样IRQ/残留模块/状态/初始化/协议缺失/watchdog破坏拒绝 |
| 上位机与构建工具质量 | Ruff通过、Black 45文件通过；mypy 45文件无错误；日志/共享fixture/新GUI生命周期/验收器等5文件strict通过 |
| 保护核对 | HEAD/index、用户设置、11份CSV、12个C函数、106个GUI方法、最终构建输入与镜像SHA均通过 |

旧脚本式测试模块的test(fn)辅助装饰器与pytest原生收集不兼容，因此完整pytest命令明确排除这4个模块，再逐个执行其正式独立入口；57项断言均已执行，不是删除或跳过控制测试。
初轮清理后回归发现6个历史GUI回放缺少已删除模块的导入，修复的是历史依赖隔离加载；完整复跑592项全部通过。所有初轮失败日志均保留。

## 正式固件结果

正式链路：原工程 → clean Rebuild → canonical静态验收 → artifacts冻结 → SHA三方一致。

- 工程：`pwm_double20260926DC6output/MDK-ARM/pwm_02.uvprojx`，STM32F407IGT6，target=pwm_02。
- 0 Error / 1 Warning；唯一warning为原main.c缺末尾换行，未为消警告改写其旧编码。
- Code/RO/RW/ZI = 12378/458/60/2252 bytes；ROM=12,896 bytes，静态RAM=2,312 bytes。
- canonical HEX：`pwm_double20260926DC6output/MDK-ARM/pwm_02/pwm_02.hex`，36,321 bytes。
- SHA-256：`b3dd7100d8e1cc43e50c61b5761b73fe17f73cb529482e9cb445edce5268879e`。
- HEX/AXF/fromelf BIN加载字节一致；镜像不含@ADC或采样模块/状态，两个通用IRQ均来自startup默认别名；协议/PWM/回传/watchdog符号与调用、300ms及CTRL_SD恢复反汇编证据通过。
- canonical/frozen/verified HEX、AXF、MAP SHA分别相同。最终清单与manifest在 `artifacts/curt-removal-20261028/release-final/`。先前本轮release/保留为过程证据；最终对测试文件换行恢复后再次执行正式构建，生产C和HEX未变。

## 证据入口与限制

- `artifacts/curt-removal-20261028/final-protection.json`：三方差异分类、逐项修复、保护SHA、方法清单与最终manifest。
- `final-fixed-full-tests.log`、各独立测试log、`image-fault-tests.json`、`build-logic-tests-final.log`：完整验证结果。
- `original-c-trace.txt` / `after-c-trace.txt`：1194帧原始/最终对照；golden夹具为 `tests/signed_control_trace.json`，LF规范化SHA=`29deb45e27bb479b501c7ebbce1e6c0ef43af18ad1226e4f647c5d24c2decc8d`。
- `release-final/manifest.json` / `static-verification.json` / `build.log` / `SHA256.txt`：正式产物与构建链。

软件回归、C外设模型和镜像证据不能证明实板PWM极性/电压、驱动电流、CTRL_SD电平或实际300ms关断/恢复时序。新HEX尚未烧录，板上版本未在本轮读取或改变。GUI无MCU watchdog执行确认；ESO/R-L在真实硬件停机后的连续性限制仍保留，未借清理改变控制语义。
