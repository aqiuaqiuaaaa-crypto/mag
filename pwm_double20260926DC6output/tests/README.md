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
