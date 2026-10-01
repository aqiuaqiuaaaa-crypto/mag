"""ARM compile + relocatable C link, NOT a Keil/final-image build or flashing.

Uses every C source from the real uvprojx and writes artifacts only to /tmp.
The existing ARMASM startup is deliberately not converted or substituted.
"""
import argparse
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "MDK-ARM/pwm_02.uvprojx"
parser = argparse.ArgumentParser()
parser.add_argument("--cc", default="arm-none-eabi-gcc")
parser.add_argument("--bin-prefix", help="relocated toolchain bin directory")
parser.add_argument("--newlib-include", help="relocated newlib header directory")
args = parser.parse_args()
tree = ET.parse(PROJECT)
settings = tree.find(".//TargetArmAds/Cads/VariousControls")
if settings is None:
    raise SystemExit("Keil target compile settings not found")
flags = ["-mcpu=cortex-m4", "-mthumb", "-mfpu=fpv4-sp-d16", "-mfloat-abi=hard",
         "-std=c99", "-O2", "-ffunction-sections", "-fdata-sections", "-Wall", "-Wextra"]
if args.bin_prefix:
    flags += ["-B" + args.bin_prefix.rstrip("/") + "/"]
if args.newlib_include:
    flags += ["-isystem", args.newlib_include]
flags += ["-D" + d.strip() for d in settings.findtext("Define").split(",") if d.strip()]
flags += ["-I" + str((PROJECT.parent / d.replace("\\", "/")).resolve())
          for d in settings.findtext("IncludePath").split(";") if d]
files = [entry for entry in tree.findall(".//Files/File") if entry.findtext("FileType") == "1"]
with tempfile.TemporaryDirectory(prefix="curt-arm-objects-") as directory:
    objects = []
    failures = []
    warnings = 0
    for index, entry in enumerate(files):
        relative = entry.findtext("FilePath").replace("\\", "/")
        source = (PROJECT.parent / relative).resolve()
        target = str(Path(directory) / (str(index) + ".o"))
        extra = ["-Werror"] if source.name in ["adc.c", "current_sense.c"] else []
        result = subprocess.run([args.cc, *flags, *extra, "-c", str(source), "-o", target],
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        output = result.stdout.decode("utf-8", errors="replace")
        warnings += output.count("warning:")
        if output:
            print(output, end="", flush=True)
        print(("PASS: " if result.returncode == 0 else "FAIL: ") + relative, flush=True)
        if result.returncode:
            failures.append(relative)
        else:
            objects.append(target)
    print(f"C translation units: {len(files) - len(failures)}/{len(files)}; warnings: {warnings}")
    if failures:
        raise SystemExit("Failed: " + ", ".join(failures))
    # Combines the entire C source set, detecting duplicate strong definitions.
    subprocess.run([args.cc, *flags, "-nostdlib", "-Wl,-r", *objects,
                    "-o", str(Path(directory) / "firmware-c-relocatable.o")], check=True)
    print("PASS: all C objects relocatable link; final startup/library/image link NOT performed")
