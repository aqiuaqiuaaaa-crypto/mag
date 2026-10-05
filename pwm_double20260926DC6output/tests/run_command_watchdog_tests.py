"""Compile actual main.c control functions/loop against a small peripheral model.

Clock configuration and generated peripheral initializers are checked separately
and compiled by Keil. No alternate parser, watchdog or PWM formula is substituted.
The model distinguishes readable CCR preloads from active PWM compare values.
"""

import argparse
import subprocess
import tempfile
from pathlib import Path

from test_adc_infrastructure import c_function

ROOT = Path(__file__).resolve().parents[1]
FUNCTIONS = [
    "PARSE_SIGNED",
    "Check_Frame",
    "Start_PWM",
    "Stop_PWM",
    "HAL_UARTEx_RxEventCallback",
    "HAL_TIM_PeriodElapsedCallback",
]
WATCHDOG_FUNCTIONS = [
    "driver_shutdown_all",
    "driver_enable_all",
    "clear_command_outputs",
    "enter_command_timeout_safe_state",
    "command_watchdog_poll",
    "commit_command",
]
CASES = [
    "heartbeat",
    "boundary-299",
    "boundary-300",
    "boundary-301",
    "zero-equivalence",
    "gpio-groups",
    "short",
    "malformed",
    "invalid-digits",
    "out-of-range",
    "overflow",
    "parser-reject",
    "wrap-heartbeat",
    "wrap-timeout",
    "startup",
    "recovery",
    "recovery-wrap",
    "stale",
    "recovery-quantization",
    "recovery-expiry",
    "recovery-preload-proof",
    "recovery-early-frames",
    "late-frame",
    "commit-boundary",
    "snapshot-race",
    "primask",
    "foreign-callbacks",
    "trailing-data",
    "trace",
]


def region(data: bytes, name: str) -> bytes:
    start = f"/* USER CODE BEGIN {name} */".encode("ascii")
    end = f"/* USER CODE END {name} */".encode("ascii")
    assert data.count(start) == data.count(end) == 1
    return data.split(start, 1)[1].split(end, 1)[0]


def control_source(data: bytes, baseline: bool = False) -> bytes:
    """Extract production bytes; only add declarations and finite test entrypoints."""
    parts = [b'#include "stm32f4xx_hal.h"\n#include <string.h>\n']
    parts.extend(region(data, name) for name in ("PD", "PV", "PFP"))
    names = FUNCTIONS + ([] if baseline else WATCHDOG_FUNCTIONS)
    bodies = []
    for name in names:
        body = c_function(data, name)
        # Legacy callback return types are on the preceding line, outside this
        # byte extractor's function-name span. Preserve their declared void.
        if body.startswith(name.encode("ascii")):
            body = b"void " + body
        signature = body.split(b"{", 1)[0].strip()
        parts.append(signature + b";\n")
        bodies.append(body)
    parts.extend(bodies)
    loop = region(data, "3").rstrip()
    assert loop.endswith(b"}")  # closing brace of the original while (1)
    parts.extend(
        [
            b"\nstatic void firmware_boot(void) {\n",
            region(data, "2"),
            b"}\n",
            b"\nstatic void firmware_step(void) {\n",
            loop[:-1],
            b"}\n",
        ]
    )
    return b"\n".join(parts)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cc", default="cc")
    parser.add_argument("--compiler-lib")
    parser.add_argument(
        "--baseline-main",
        type=Path,
        help="run only the identical healthy trace against a saved checkpoint",
    )
    parser.add_argument("--trace-output", type=Path)
    parser.add_argument("--baseline-timeout-probe", action="store_true")
    args = parser.parse_args()
    baseline = args.baseline_main is not None
    source = args.baseline_main or ROOT / "Core/Src/main.c"
    assert not args.baseline_timeout_probe or baseline
    cases = (
        (["baseline-timeout"] if args.baseline_timeout_probe else ["trace"])
        if baseline
        else CASES
    )
    with tempfile.TemporaryDirectory(prefix="command-watchdog-test-") as directory:
        folder = Path(directory)
        (folder / "main_under_test.inc").write_bytes(
            control_source(source.read_bytes(), baseline)
        )
        executable = str(folder / "test_command_watchdog.exe")
        command = [args.cc, "-std=c99", "-Wall", "-Werror"]
        if args.compiler_lib:
            command += ["-B" + args.compiler_lib]
        if baseline:
            command += ["-DBASELINE"]
        command += [
            "-I" + str(ROOT / "tests/watchdog_stubs"),
            "-I" + directory,
            str(ROOT / "tests/test_command_watchdog.c"),
            "-o",
            executable,
        ]
        subprocess.run(command, check=True, cwd=directory)
        for case in cases:
            result = subprocess.run(
                [executable, case],
                cwd=directory,
                check=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                timeout=30,
            )
            if case == "trace" and args.trace_output:
                args.trace_output.write_bytes(result.stdout)
            elif case != "trace":
                print(result.stdout.decode("ascii"), end="", flush=True)
            print("PASS: " + case, flush=True)
    print(f"{len(cases)}/{len(cases)} production control scenarios PASS")


if __name__ == "__main__":
    main()
