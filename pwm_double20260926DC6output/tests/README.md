# Signed six-channel control verification

Run from the firmware directory. These checks never open a real port or flash hardware.

```powershell
python -B tests/test_control_infrastructure.py
python -B tests/test_command_watchdog_integration.py
python -B tests/run_command_watchdog_tests.py --cc <host-C-compiler>
```

`signed_pwm_baseline.json` is the original, unchanged SHA-256 fixture. It protects
GPIO/UART/HAL/CMSIS/BSP/startup, parser/PWM callbacks, clocks, CubeMX resources and
the original `main.c` hash `fa5d005b9dede4168a274a3b53b8451e56605eea97418851d5f902022dad9894`.
The current main reduces to that original only after reversing the exact watchdog
allowlist and the documented DMA echo buffer/lock guards. Never recapture this
fixture to hide a control change. `c_source.py` extracts production function bytes.

The 43-byte signed command frame and logical a0..a5 order remain unchanged.
PWM uses the original `4200 + command * 42` formula, timers, polarity and channels.
Physical poles remain `[1,3,5,4,6,2]`. Invalid digits are rejected before commit;
an accepted longer RX event retains the original valid 43-byte prefix semantics.
The foreground validates an immutable RX snapshot, not a mutable mailbox.

At unsigned tick age >=300 ms, watchdog polling shuts down PF2/PF7/PF10, zeros
all six commands through the original CCR callback and latches timeout. Only a
validated commit feeds it. Recovery commits zero, waits two HAL tick increments
AND fresh natural TIM1/TIM8 update flags, then enables CTRL_SD. Only a subsequent
frame can apply nonzero commands. No forced timer event, polarity or timing change.
This cooperative timeout cannot protect a halted CPU or frozen tick; bench timing
and physical shutdown/recovery still require hardware verification.

The host C harness executes actual production functions and the main loop with
fake peripherals, separate CCR preloads/active values and natural update events.
Its 30 scenarios cover heartbeat, 299/300/301 ms, rejected frames, tick wrap,
startup, recovery ordering/expiry, stale targets, RX races, PRIMASK and DMA-busy echo.
The 12-byte magnitude echo uses HAL_UART_Transmit_DMA. The persistent TX buffer is
written only when UART TX is READY. Submission preserves PRIMASK and masks IRQs
briefly because foreground TX and RX rearm share the same HAL UART lock. Busy echo
is dropped without delaying control or overwriting in-flight bytes.

`signed_control_trace.json` records a golden trace obtained from the original
main with the same harness: all -99..99 values on each of six channels, 1194 frames,
including signed commands, preload/active CCRs, CTRL_SD, PWM masks and 12 echo bytes.
The runner checks its normalized-LF SHA on every healthy trace. Optional
`--baseline-main <saved-original>` and `--trace-output <new-file>` allow direct
comparison; `--baseline-timeout-probe` demonstrates the old held-output behavior.

ADC sampling, TIM3 TRGO, DMA2 Stream0 sampling, CURT modules and their dedicated
tests were removed on 2026-10-28. Generic CMSIS definitions and startup weak vector
aliases remain. No feedback or current PI was introduced. Historical experiments
and build images in root artifacts/ and csvvvv/ remain preserved.

Optional `compile_firmware_sources.py` compiles every actual uvprojx C source and
performs a relocatable GCC link; that does not replace a Keil image build.
Official original-project Rebuild and acceptance:
[FIRMWARE_BUILD_SOP.md](../../FIRMWARE_BUILD_SOP.md).
