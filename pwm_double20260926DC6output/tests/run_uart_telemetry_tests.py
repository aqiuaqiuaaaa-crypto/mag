"""Build/run real uart_telemetry.c with fake peripherals, never open real UART."""
import argparse
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument("--cc", default="cc")
parser.add_argument("--compiler-lib")
args = parser.parse_args()
cases = ["normal", "invalid", "maximum", "busy-latest", "echo-buffer", "echo-priority",
         "tick-wrap", "isr-masked", "dma-ownership", "hal-busy", "hal-error", "dma-silent",
         "dma-error", "fast-complete", "bad-input", "echo-latest", "completion-race", "rx-lock"]
with tempfile.TemporaryDirectory(prefix="uart-telemetry-test-") as directory:
    executable = str(Path(directory) / "test_uart_telemetry.exe")
    command = [args.cc, "-std=c99", "-Wall", "-Werror"]
    if args.compiler_lib:
        command += ["-B" + args.compiler_lib]
    command += ["-I" + str(ROOT / "tests/telemetry_stubs"),
                "-I" + str(ROOT / "Core/Inc"), str(ROOT / "tests/test_uart_telemetry.c"),
                "-o", executable]
    subprocess.run(command, check=True, cwd=directory)
    failures = []
    for case in cases:
        result = subprocess.run([executable, case], cwd=directory)
        print("FAIL:" if result.returncode else "PASS:", case, flush=True)
        if result.returncode:
            failures.append(case)
    if failures:
        raise SystemExit("Failed scenarios: " + ", ".join(failures))
    print(f"{len(cases)}/{len(cases)} telemetry scenarios PASS")
