"""Build/run actual current_sense.c with fake HAL in a temporary directory."""
import argparse
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument("--cc", default="cc")
parser.add_argument("--compiler-lib", help="optional TCC -B directory")
args = parser.parse_args()
if not (ROOT / "Core/Src/current_sense.c").is_file():
    raise SystemExit("RED: production current_sense.c does not exist")
with tempfile.TemporaryDirectory(prefix="current-sense-test-") as directory:
    executable = str(Path(directory) / "test_current_sense")
    command = [args.cc, "-std=c99", "-Wall", "-Werror"]
    if args.compiler_lib:
        command += ["-B" + args.compiler_lib]
    command += ["-I" + str(ROOT / "tests/stubs"), "-I" + str(ROOT / "Core/Inc"),
                str(ROOT / "tests/test_current_sense.c"), "-o", executable]
    subprocess.run(command, check=True, cwd=directory)
    failures = []
    for case in ["normal", "adc-init", "timer-init", "dma-start", "dma-silent",
                 "timer-start", "overrun", "dma-error", "late-half", "copy-wrap",
                 "pending-dma-error", "pending-overrun", "copy-full-wrap",
                 "backlog-same-half", "ndtr-invalid", "late-tc", "late-tc-same-half"]:
        result = subprocess.run([executable, case], cwd=directory)
        if result.returncode:
            failures.append(case)
        print("FAIL:" if result.returncode else "PASS:", case, flush=True)
    if failures:
        raise SystemExit("Failed scenarios: " + ", ".join(failures))
