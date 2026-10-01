# PROJECT_MEMORY

项目长期进度记忆源。以后处理本项目时，优先读取并在每阶段结束后更新本文件。
最后更新：2026-09-30；当前阶段：六路 CURT 只采样软件基础设施已完成；Keil 最终构建与实板验证待执行。

# Project baseline

- 当前上位机：`dipole-control/`。
- 当前固件：`pwm_double20260926DC6output/`，STM32F407IGTx，Keil MDK 工程 `MDK-ARM/pwm_02.uvprojx`。
- CubeMX 6.6.1；STM32CubeF4 V1.27.1；现有 HAL V1.8.1。只补同版本 ADC 文件，不升级现有 HAL。
- 当前可见 `.git/` 没有有效元数据，不能提供分支、HEAD 或 clean 状态；禁止重建 Git 或自动提交。
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
- 原始 ADC 为权威数据，不实现安培换算、PI、串口遥测或任何 ADC→PWM 路径。
- PA4/JP5 最终仍需实板确认。当前 DAC 未启用；本轮可配置 PA4 analog ADC，不以硬件尚未接线阻塞软件。

# Completed

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

- 初始无可用 Keil、Wine、GCC、Clang、arm-none-eabi-gcc、make。后已下载并解包 TCC 和 ARM GCC 13.2.1 到 `/tmp/mag-adc-baseline.f7bxHV/toolchain`，不安装、不改系统。
- 静态集成最终 **16/16 PASS**。首次 RED 为 13 项中 10 项因新增设施缺失失败、3 项原基线保护通过；后续加入厂商来源哈希、完整配置和稳定源帧检查。
- 真实 C 模块最终 **17/17 场景 PASS**。初版 8 场景 PASS；独立审查后旧逻辑 6 场景 RED，修复后 GREEN。覆盖完整 HT/TC、PRIMASK、所有启动失败、OVR/DMA累计/重复mask、half0/half1 积压、NDTR 边界、拷贝期间 wrap、NDTR 相同但 opposite sticky flag 已置位。HAL 为 test stub，不是真实 ADC 测量。
- 全工程 C 编译验证：Keil 工程列出的 33/33 个 C translation units 均经 ARM GCC 13.2.1 编译 PASS，全部 C object relocatable link PASS。7 条厂商 unused-parameter 警告，新编写 ADC/current_sense 模块 `-Werror` 下通过。
- **Keil 最终构建未执行**：缺少 MDK/ARMCC；未组装 Keil ARMASM startup、未做最终 startup/library/image link、未产生新可烧录固件。不将静态检查/ARM C 编译写成 Keil 编译通过。
- 实板采样、时序/精度/原 PWM 示波器对照未执行；尚未连接 CURT。
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

- OPEN A：CURT 在正向、反向、续流、换向情况下能否可靠恢复有符号线圈电流。
- OPEN B：六路实物 CURT 接线尚未完成。
- PA4/JP5 实板 ADC/PM2_AMPW 路径确认。
- 六路 offset/gain、参考电压和方向标定。
- CURT RC 实测延迟及噪声、通道串扰。
- 不声称 ADC 已测得真实有符号线圈电流；1.27 V、0.12 V/A 不作为闭环常数。

# Next step

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

硬件事实：PF7/CTRL_SD 为用户已确认定制接线；其余 CURT 特性/接线/PA4/JP5/标定保留 OPEN。500Hz/27.4μs 是根据现有时钟配置计算的设计值，尚无实板时序证据。

# Windows Keil Build Baseline

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

本章为最新阶段状态，承接 Windows Keil Build Baseline。之前 Linux 阶段的“Keil 待执行”等描述和全部历史记录保留。当前第一阶段的只采样软件链路及软件回归验证已完成；实板接线、实际 ADC 采样和测量准确性仍未验证。未进入第二阶段电流闭环设计。

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
| rank 6 / raw[5] | a5 | PA4 | IN4 | 已实现；PA4/JP5 实板路径待确认 |

- DMA2 Stream0 / Channel0，外设到内存、内存地址递增、两端 halfword、循环模式、FIFO 关闭、硬件优先级 LOW；12 个 uint16_t 构成两个六通道半区。
- DMA2 Stream0 IRQ 经 HAL DMA 中断处理调用 ADC 半完成/完成回调，再进入 `PublishFrame()`；ADC IRQ 经 HAL ADC 中断处理进入错误回调。新增两个 IRQ 优先级均为 3；现有 UART=0、TIM2=2 保持。
- TIM3 时钟 84 MHz，PSC=839、ARR=199，TRGO=UPDATE，设计频率 500 Hz，每 2 ms 一组六路数据；HT/TC 交替发布完整帧。六路顺序转换总计约 27.4 µs，属于配置计算值，尚无实板测量证据，六路并非同时采样。
- `published` 缓存六路 raw、frame_count、timestamp_ms、error_flags、overrun_count、dma_error_count、dma_late_count、valid、running。时间戳为发布时 HAL tick，valid 表示完整帧且未锁存错误，不代表校准有效或自动判定数据新鲜度。
- `CurrentSense_GetSnapshot()` 是保留的前台一致性读取 API，当前 main 没有调用它，也没有串口遥测或 ADC→PWM 消费路径；本轮不添加消费者。最终链接会移除这个未引用函数，内部采样/发布缓存仍保留并运行。该 API 的实际 C 行为已通过现有测试，ARMCC 目标文件中也有正确指令；尚未在实板运行调用该 API。

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

- 软件：已完成只采样接入、软件回归和 Windows Keil 最终构建；本轮无待修复的软件 blocker。
- 硬件接线：等待六路 CURT 接线及 PA4/JP5 路径确认。
- ADC 实采：等待烧录本轮对应固件，验证 raw[0..5] 引脚顺序、已知电压响应、frame_count 约 500 次/秒递增、运行状态/错误计数，以及 UART/PWM 工作时的采样连续性。
- 测量质量：六路 offset/gain、参考电压、方向/续流/换向特性、RC 延迟、噪声、串扰及原 PWM 对照仍需硬件证据。
- 本阶段不添加电流 PI，不改变原 PWM 逻辑；先完成接线和 ADC 实采验证，再决定后续阶段。
