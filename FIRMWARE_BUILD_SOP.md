# 正式 firmware canonical build / release SOP

本规范是以后选择正式可烧录固件的入口；历史日期目录中的 HARDWARE_SOP / README
所列 artifacts HEX 只描述当时验证，不替代本规范。此流程只构建、核验与冻结，**不下载、不烧录**。

## 两类 build

- **Isolated validation**：tracked source → staging → Keil Rebuild → validation artifact。
  继续保留原 prepare_build.py / verify_protection.py，用于隔离构建与保护测试。
  只证明该副本能构建；不会更新 canonical，不能自动称为 official/canonical release。
- **Official release**：**ORIGINAL project → clean Rebuild → canonical verification →
  artifacts freeze → SHA equality**。正式入口为 `tools/firmware/clean-build.ps1`。
  它由 2026-10-06 一次性 clean-build.ps1 整理而来；历史证据脚本保持原样，不能当成可重跑入口。

历史 watchdog 构建成功，但只从 staging 把新版复制到 artifacts；缺少正式 canonical
发布/一致性检查，原工程仍留旧 HEX。根因是 **canonical artifact consistency gap**，
不是 uvprojx、Generate HEX、source list、编译失败或 restore 旧 HEX 的问题。
禁止用 staging → artifacts → canonical 的拷贝链代替原工程 Rebuild。

## 固定工程与输出

以下路径均相对 Git workspace 根目录；脚本从自己的位置定位 Git 根，不依赖当前目录。

| 项目 | canonical 路径 |
|---|---|
| Keil project | `pwm_double20260926DC6output/MDK-ARM/pwm_02.uvprojx` |
| target / MCU | `pwm_02` / `STM32F407IGT6`（XML device family 为 `STM32F407IGTx`） |
| HEX | `pwm_double20260926DC6output/MDK-ARM/pwm_02/pwm_02.hex` |
| AXF | `pwm_double20260926DC6output/MDK-ARM/pwm_02/pwm_02.axf` |
| MAP | `pwm_double20260926DC6output/MDK-ARM/pwm_02/pwm_02.map` |

staging/delivery 副本、目标/output 配置改变、custom build hook 开启均拒绝作为正式入口。
需要重新审查这些变化，不能通过替换输入参数绕过 canonical 路径。

## 操作入口

在项目根目录运行；DryRun 不清理、不调用 Keil、不创建冻结目录，也不更新 Git。

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools/firmware/clean-build.ps1 -DryRun
powershell -NoProfile -ExecutionPolicy Bypass -File tools/firmware/test_clean_build.ps1
```

正式构建由操作者另行执行：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools/firmware/clean-build.ps1
```

默认在已安装的 LOCALAPPDATA/Keil_v5 中发现 UV4，选用 ARM/ARMCC 下匹配的 fromelf。
其他安装位置使用已确认的绝对路径传 `-UV4Path` / `-FromElfPath`，不能用 GCC 替代正式镜像。
`-Python` 可选择现有 Python；静态核验仅用标准库。工具缺失在清理前停止。
`-ArtifactDirectory` 可指定 workspace/artifacts 下的新绝对路径；已存在目录一律拒绝覆盖。

## 顺序与硬门槛

1. **Preflight**：记录 Git HEAD、tracked 状态、原 uvprojx 路径/SHA、target、MCU、
   时间及全部 tracked firmware 输入 SHA。任何 staged/unstaged tracked firmware 修改都停止；
   不提交、不丢弃修改。其他 tracked 文档/PC 修改只记录状态，不改变该 firmware 门槛。
2. **审计与 clean**：记录旧 HEX/AXF/MAP 的 size/mtime/SHA，归档将清理的旧 generated
   output。只清理 canonical output、MDK Objects/Listings 与 MDK listings 中已识别、
   未跟踪的生成文件；保留 tracked scatter/input。越界、junction/symlink、未知 output 文件停止。
   完成后必须确认 canonical **HEX、AXF、MAP 全部不存在**。
3. **原工程 Rebuild**：只使用 `UV4.exe -r <original project> -t pwm_02 -j0 -sg -o <log>`。
   没有 Download/debugger 参数。完整日志先写 canonical output 的
   `pwm_02.release-build.tmp`；必须有 Rebuild/错误警告总结，0 errors。
   退出码0/1可接受，warnings完整记录；不为消警告修改控制逻辑。
4. **Freshness**：三份 canonical 产物必须非空，mtime 在本次 start/finish UTC 窗口内。
   同时要求清理后“不存在”证据；mtime 新或 SHA 与过去相同都不能单独证明 fresh。
5. **Canonical static verification**：复用 `verify_image.py` 的 2026-10-06 逻辑：Intel HEX
   结构/校验、AXF PT_LOAD/HEX/fromelf BIN 加载字节一致、入口/vector、`@ADC`、
   current_sense/uart_telemetry/ADC/TIM3/DMA IRQ/watchdog live symbols、main cross-references、
   300ms比较及CTRL_SD shutdown/recovery代码。核验 canonical 文件，不先复制 staging 文件。
   fromelf BIN/反汇编仅是分析证据；单独运行 verifier PASS **不等于 release PASS**。
6. **Freeze**：static PASS 后，再检查 HEAD/firmware 输入 SHA 未变。
   从 **canonical → 新 artifacts 目录**复制 HEX/AXF/MAP 和完整 build.log，保留预检、
   static-verification、工具/命令/时间元数据。绝不反向覆盖 canonical。
7. **SHA equality**：复制后重新计算 canonical 与 frozen HEX SHA；两者及 static 验证时
   记录的 SHA 必须相等。AXF/MAP 同样执行此检查，防止核验后替换文件。
8. **Manifest**：只有上述全部 PASS 才写 `manifest.json.status=PASS`；记录 HEAD、project
   SHA、target/MCU、工具/命令、build时间/errors/warnings、absence/freshness/verification
   状态，以及 canonical/frozen 三份文件的绝对路径、size、mtime、SHA；另存 SHA256.txt。
   失败目录保留诊断、manifest为FAIL，缺失/PASS之前退出的 manifest 都不能当成正式发布。

**Authoritative / burnable** 必须同时满足：原工程 clean Rebuild PASS、canonical fresh、
静态核验PASS、SHA已记录、canonical SHA == frozen SHA == verified SHA。
烧录前重新核对所选 HEX 与记录的 SHA；不同日期的 validation artifact 不能混用。
这里的 burnable 表示构建来源和镜像一致性，不代表硬件功能已验证，也不授权自动烧录。

## 产物、测试与历史边界

默认新目录为 `artifacts/firmware-release-<timestamp>/`，保存 HEX/AXF/MAP、build.log、
preflight.json、static-verification.json/fromelf分析数据、manifest.json、SHA256.txt和old-output。
artifacts 保持 untracked；脚本没有 git add/commit/push，也不修改 .gitignore。

轻量测试在独立 artifacts fixture 中验证正确路径、staging拒绝、旧产物清理/拒绝、
缺失/空/过期产物、absence证明、SHA不等FAIL/相等PASS、核验后篡改、tracked scatter保留、
未知文件保护、warning/error判定，以及真实workspace DryRun保持 canonical 与Git暂存区不变。
不需要为这些门槛重复真实Keil构建。

2026-10-06 已完成的原工程 clean Rebuild/静态核验证据继续保留在
`artifacts/firmware-clean-build-20261006/`。本次仅固化流程，未重新构建：canonical HEX仍为
46,798 bytes，mtime `2026-10-06 14:50:45.781467+08:00`，SHA-256
`88eccb1b3b96725f67b356748942b8552fbb9abff984bbeba82ec0e64c18ef8a`。

仅上述 SHA 的当前 HEX 属于 **grandfathered / pre-SOP authoritative build**：它在本 SOP
正式建立前，于 2026-10-06 手工按等价门槛完成 canonical clean Rebuild、静态核验、
canonical/verified/frozen SHA equality 和 artifact freeze 后获认可。
后续任何新的正式 firmware build 必须走本 SOP 的全部流程并取得 PASS manifest；
此特例只适用于上述已验证 HEX，不允许未来绕过 SOP。

首次实际执行新正式入口时仍必须完成全部门槛，不能把本次DryRun/旧核验报告伪装成新build。
