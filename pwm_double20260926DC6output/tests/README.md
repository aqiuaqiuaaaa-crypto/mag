# Measurement-only ADC verification

Run from the firmware directory. Tests never open UART, change host control settings,
flash hardware or generate files in the project. Python is used with `-B`.

```sh
python3 -B tests/test_adc_infrastructure.py
python3 -B tests/run_current_sense_tests.py --cc cc
python3 -B tests/compile_firmware_sources.py --cc arm-none-eabi-gcc
```

- `signed_pwm_baseline.json` is a SHA-256 fixture captured **before implementation**.
  Do not recapture it to hide regressions. It protects original HAL/CMSIS/BSP,
  GPIO/UART/startup, parser/PWM/timer callbacks, clock configuration and existing
  CubeMX resources. `main.c` must reduce to identical original bytes after removing
  its two explicitly allowed ASCII additions, preserving its legacy encoding/CRLF.
- `test_current_sense.c` includes the **real** acquisition module with only HAL
  replaced by `stubs/stm32f4xx_hal.h`. Each scenario is a fresh executable process.
  It checks six-channel HT/TC ordering, complete-frame counters, timestamps,
  PRIMASK restoration, invalid/NULL/foreign-ADC handling, one-shot startup,
  ADC/TIM init failure, DMA failure (including HAL's silent failure), TIM start
  failure, ADC overrun, DMA error and cumulative/repeated error masks. DMA latency
  regression cases cover opposite flags still pending, invalid NDTR, both halves,
  wrap during copying and a whole wrap returning to the same NDTR range.
  This is a software test, not ADC accuracy proof.
- `compile_firmware_sources.py` compiles **every C translation unit listed in the
  actual Keil project**, then makes a relocatable link to detect duplicate symbols.
  Newly authored `adc.c`/`current_sense.c` use `-Werror`. Other pre-existing/vendor
  warnings are reported. It does **not** assemble the Keil ARMASM startup or perform
  the final startup/library/image link; it is not a successful Keil build claim.
- Vendor source provenance and normalized-file SHA-256 are in
  `vendor_adc_sources.json`; existing vendor files are never upgraded.

Temporary toolchains can be passed using `--compiler-lib` (TCC) or
`--bin-prefix`/`--newlib-include` (relocated ARM GCC). No compiler installation is
needed. The actual environment and results are recorded in root PROJECT_MEMORY.md.

The module is boot-started once. Use `CurrentSense_GetSnapshot()` from foreground
code/debugger when adding a diagnostic consumer later; no consumer or telemetry
protocol is added this stage. Inspect `raw`, `frame_count`, `timestamp_ms`, `valid`,
`running`, `error_flags`, `overrun_count`, `dma_error_count`, `dma_late_count`.
Always check freshness; breakpoints/long interrupt masking can stall publication
while DMA keeps running. Uncertain/overwritten DMA halves are rejected and latch
`CURRENT_SENSE_ERROR_DMA_LATE`, stopping only TIM3 until MCU reset. Repeated
accumulated HAL error bits are counted once per newly latched category.
Hardware wiring/calibration and PA4/JP5 remain OPEN. Do not interpret raw values as
calibrated signed coil current or feed them to PWM.

## Foreground UART telemetry (2026-10-03)

The old hardware-open statements above are historical: the user confirms six CURT
wires via CN4, existing GND, PA4/PA5 ADC jumpers, and a flashed phase-1 acquisition
image. This update produces a **new telemetry image**, still requiring a new flash
before PC raw validation. Calibration and PI remain unimplemented.

`uart_telemetry.c` is the sole Core USART1 TX caller. It arbitrates the unchanged
12-byte command echo and 100 ms latest-snapshot telemetry, using one persistent
128-byte DMA buffer and a separate 12-byte latest-echo slot. A due telemetry frame
has priority; busy DMA/UART slots defer to a later main-loop iteration. No UART
transmit, formatting or new work is added to ADC/DMA callbacks or IRQ handlers.

The short DMA-start critical section saves/restores PRIMASK: this HAL's TX and
ReceiveToIdle restart share the UART lock, so the RX ISR must not observe it locked
by foreground TX setup. No waiting, formatting or wire-transfer polling occurs in
that section. UART gState, TX DMA state and stream EN gate buffer reuse; UART TC,
not merely DMA completion, releases ownership. HAL errors, silent DMA-start failure
and TX DMA errors latch TX-only failure until reset, without aborting RX/PWM/ADC.

```powershell
python -B tests/test_adc_infrastructure.py
python -B tests/run_current_sense_tests.py --cc <host-C-compiler>
python -B tests/test_uart_telemetry_integration.py
python -B tests/run_uart_telemetry_tests.py --cc <host-C-compiler>
```

`telemetry_main_patch.py` reverses exactly three authorized foreground edits before
the original pre-ADC main hash check. `signed_pwm_baseline.json` was **not** changed
or recaptured. `telemetry_phase1_baseline.json` also protects complete phase-1
Core/ioc bytes and restores just the one new Keil file-list entry for hashing.
Parser, RX callback, PWM updates, clock, timers, ADC acquisition and IRQ bytes
remain protected; the test extension does not replace those original checks.

The telemetry C suite executes the production module with fake HAL/snapshots.
18 scenarios cover exact fields/order, 100 ms cadence, invalid/faulted snapshots,
uint32/uint16 maximum formatting, busy/latest behavior, DMA buffer lifetime,
echo fairness/latest queue, tick wrap, ISR/masked rejection, DMA versus UART-TC
ownership, HAL busy/error, silent DMA-start failure, asynchronous DMA error,
completion during status inspection and pending RX with the shared HAL lock.

Historical telemetry logs, input hashes, HEX/AXF/MAP and Keil output are preserved
in root `artifacts/curt-telemetry-20261003/` as historical validation evidence,
not the current official flashing entry point.

For current official flashing, use the canonical HEX (relative to repository root):
`pwm_double20260926DC6output/MDK-ARM/pwm_02/pwm_02.hex` (46,798 bytes). Verify
authoritative SHA-256 `88eccb1b3b96725f67b356748942b8552fbb9abff984bbeba82ec0e64c18ef8a`
before flashing; do not choose by filename or use the historical telemetry HEX.
Official build / release rules: [FIRMWARE_BUILD_SOP.md](../../FIRMWARE_BUILD_SOP.md).
The current authoritative HEX has not been flashed.

## Command timeout and hardware shutdown (2026-10-05)

```powershell
python -B tests/test_command_watchdog_integration.py
python -B tests/run_command_watchdog_tests.py --cc <host-C-compiler>
```

`main.c` checks command age in its nonblocking foreground loop and again at the
actual command commit. Only a validated immutable RX snapshot commits and feeds
`last_valid_cmd_tick`. The signed 43-byte format, channel order, numeric parser,
TIM2 callback and its `4200 + command * 42` formula remain unchanged. The format
checker now rejects nondecimal digits; formerly the numeric parser silently
returned zero for them. The original acceptance of a valid 43-byte prefix in a
longer RX event is retained. Short, malformed and invalid frames never feed.

At unsigned HAL tick age **>=300 ms**, the common safety function lowers PF10,
PF2 and PF7, clears all commands, calls the original TIM2 CCR update path, and
latches timeout. The PWM timers continue running under driver shutdown. No
forced update event, polarity/topology/frequency change, ADC feedback, hardware
IWDG or telemetry error bit is introduced. This is a cooperative command timeout;
it does not protect against a halted CPU, frozen SysTick or interrupts disabled
indefinitely. Actual shutdown time also includes foreground scheduling latency.

Recovery is scheme A: the first valid frame commits zero and starts recovery.
All frames processed during the settling phase also commit zero. After the zero
writes, only the unused TIM1/TIM8 update flags are cleared (no timer update IRQ
or DMA is enabled). Polling waits for two HAL tick increments AND a new natural
update event from EACH timer. These events directly prove that the PWM **active**
compare values became zero, independently of SysTick interrupt latency. Only then
does polling clear the latch and raise all three CTRL_SD pins. A subsequent
valid frame may command nonzero; no buffered old target is automatically applied.
Recovery expiry is checked before enable. Boot with no command retains the
existing GPIO LOW/PWM stopped state; the original first-command startup remains.

The host C suite compiles production definitions/functions and the actual main
loop body; generated clock/peripheral initialization is protected by byte checks
and compiled in Keil separately. Fake HAL models 500 us TIM2 updates and separate
50 us natural CCR preload transfers, including a still-stale active compare at
timeout. 29 scenarios cover heartbeat, 299/300/301 boundaries, rejected frames,
uint32 wrap, startup, recovery ordering/quantization/expiry/early frames, repeated
timeouts, missing preload update on one timer, no stale command revival, RX
snapshot races, PRIMASK and GPIO groups.
Its healthy trace exhausts -99..99 on each of six channels (1194 frames) and can
be compared byte-for-byte with a saved checkpoint using `--baseline-main` and
`--trace-output`; `--baseline-timeout-probe` reproduces the old held output.

`watchdog_main_patch.py` reverses only the explicit new main edits before the
historical ADC/telemetry hash checks. Both old hash fixtures remain unchanged;
edits outside the allowlist remain detectable. PC, sampling and telemetry source
files are unchanged. New build/evidence and the pending hardware SOP are in
`artifacts/command-watchdog-20261005/`; bench tests are **not yet performed**.
