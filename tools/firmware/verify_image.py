"""Static checks reused from the 2026-10-06 canonical build evidence.

This verifier alone never labels an image an official release. clean-build.ps1
adds original-project, clean/freshness, source protection and freeze/hash gates.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import struct
import subprocess
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any

TZ = timezone(timedelta(hours=8), "Asia/Shanghai")


def metadata(path: Path) -> dict[str, object]:
    stat = path.stat()
    return {
        "path": str(path),
        "size": stat.st_size,
        "mtime_shanghai": datetime.fromtimestamp(stat.st_mtime, TZ).isoformat(),
        "mtime_ns": stat.st_mtime_ns,
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
    }


def intel_hex(path: Path) -> tuple[dict[int, int], dict[str, object]]:
    memory: dict[int, int] = {}
    base = 0
    eof = False
    count = 0
    entry = None
    for line in path.read_text(encoding="ascii").splitlines():
        if eof:
            raise ValueError("Unexpected record after EOF")
        if not line.startswith(":"):
            raise ValueError(line)
        record = bytes.fromhex(line[1:])
        if len(record) < 5 or len(record) != record[0] + 5:
            raise ValueError("Bad HEX length")
        if sum(record) % 256 != 0:
            raise ValueError("Bad HEX checksum")
        address = int.from_bytes(record[1:3], "big")
        kind = record[3]
        payload = record[4:-1]
        if kind != 0 and (
            kind not in {1, 2, 3, 4, 5}
            or address != 0
            or len(payload) != {1: 0, 2: 2, 3: 4, 4: 2, 5: 4}[kind]
        ):
            raise ValueError("Bad HEX control record")
        if kind == 0:
            for offset, value in enumerate(payload):
                absolute = base + address + offset
                if absolute in memory and memory[absolute] != value:
                    raise ValueError("Invalid firmware image")
                memory[absolute] = value
        elif kind == 1:
            if payload:
                raise ValueError("Invalid firmware image")
            eof = True
        elif kind == 2:
            base = int.from_bytes(payload, "big") << 4
        elif kind == 4:
            base = int.from_bytes(payload, "big") << 16
        elif kind == 5:
            entry = int.from_bytes(payload, "big")
        elif kind == 3:
            entry = (int.from_bytes(payload[:2], "big") << 4) + int.from_bytes(
                payload[2:], "big"
            )
        else:
            raise ValueError(f"Unknown HEX record {kind}")
        count += 1
    if not (eof and memory):
        raise ValueError("Invalid firmware image")
    lo, hi = (min(memory), max(memory))
    if len(memory) != hi - lo + 1:
        raise ValueError("Unexpected holes in load image")
    binary = bytes(memory[address] for address in range(lo, hi + 1))
    marker_addresses = [
        f"0x{lo + match.start():08x}" for match in re.finditer(b"@ADC", binary)
    ]
    return (
        memory,
        {
            **metadata(path),
            "records": count,
            "checksums_valid": True,
            "base": f"0x{lo:08x}",
            "end_inclusive": f"0x{hi:08x}",
            "load_bytes": len(binary),
            "load_sha256": hashlib.sha256(binary).hexdigest(),
            "entry": f"0x{entry:08x}" if entry is not None else None,
            "adc_marker_addresses": marker_addresses,
        },
    )


def elf_load(path: Path) -> tuple[dict[int, int], dict[str, object]]:
    blob = path.read_bytes()
    header = struct.unpack_from("<16sHHIIIIIHHHHHH", blob)
    if not header[0][:6] == b"\x7fELF\x01\x01":
        raise ValueError("Invalid firmware image")
    if not (header[1] == 2 and header[2] == 40):
        raise ValueError("AXF is not ARM ELF executable")
    memory: dict[int, int] = {}
    segments = []
    for index in range(header[10]):
        kind, offset, vaddr, paddr, filesz, memsz, flags, align = struct.unpack_from(
            "<IIIIIIII", blob, header[5] + index * header[9]
        )
        if kind != 1 or not filesz:
            continue
        if not 134217728 <= paddr < 135266304:
            raise ValueError("Invalid firmware image")
        data = blob[offset : offset + filesz]
        if not len(data) == filesz:
            raise ValueError("Invalid firmware image")
        memory.update({paddr + i: byte for i, byte in enumerate(data)})
        segments.append(
            {
                "physical_address": f"0x{paddr:08x}",
                "virtual_address": f"0x{vaddr:08x}",
                "file_bytes": filesz,
                "memory_bytes": memsz,
                "flags": flags,
                "alignment": align,
            }
        )
    if not memory:
        raise ValueError("Invalid firmware image")
    return (
        memory,
        {
            **metadata(path),
            "machine": "EM_ARM",
            "class": "ELF32",
            "type": "ET_EXEC",
            "entry": f"0x{header[4]:08x}",
            "load_segments": segments,
        },
    )


def symbols(map_text: str) -> dict[str, dict[str, Any]]:
    result = {}
    for match in re.finditer(
        "^\\s*(\\S+)\\s+(0x[0-9a-fA-F]+)\\s+Thumb Code\\s+(\\d+)\\s+(.+)$",
        map_text,
        re.MULTILINE,
    ):
        name, address, size, obj = match.groups()
        result[name] = {
            "thumb_address": address,
            "code_address": f"0x{int(address, 16) & ~1:08x}",
            "bytes": int(size),
            "object": obj.strip(),
            "map_line": match.group(0).strip(),
        }
    return result


def disassembly_function(text: str, name: str) -> str:
    match = re.search(
        "^    " + re.escape(name) + "\\s*\\n(.*?)(?=^    i\\.|\\Z)",
        text,
        re.MULTILINE | re.DOTALL,
    )
    if not match is not None:
        raise ValueError(name)
    return name + "\n" + match.group(1)


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def verify_no_measurement(
    map_text: str, linked: dict[str, dict[str, Any]]
) -> dict[str, str]:
    """Reject real sampling code; allow only startup-owned default IRQ aliases.

    The MAP includes object provenance and live cross references. A generic
    vector-table name alone does not establish an active peripheral handler.
    """
    require(
        re.search(
            r"\b(?:adc|current_sense|uart_telemetry|stm32f4xx_hal_adc(?:_ex)?)\.o\b",
            map_text,
            re.IGNORECASE,
        )
        is None,
        "CURT/ADC object remains in MAP",
    )
    for name in linked:
        require(
            re.match(
                r"(?:CurrentSense_|UARTTelemetry_|HAL_ADC|MX_ADC|MX_TIM3_Init$|FormatSnapshot$)",
                name,
            )
            is None,
            f"Measurement implementation remains: {name}",
        )
    require(
        re.search(
            r"^\s*(?:hadc1|hdma_adc1|htim3|latest_snapshot|pending_echo)\s+0x",
            map_text,
            re.MULTILINE,
        )
        is None,
        "Measurement state remains in MAP",
    )
    handlers = {}
    for name in ("ADC_IRQHandler", "DMA2_Stream0_IRQHandler"):
        item = linked.get(name)
        require(item is not None, f"Missing startup vector alias: {name}")
        assert item is not None
        owner = str(item["object"])
        require(
            owner.startswith("startup_stm32f407xx.o("),
            f"Active sampling IRQ remains: {name}",
        )
        handlers[name] = owner
    return handlers


def verify(output: Path, binary_path: Path, disassembly_path: Path) -> dict[str, Any]:
    memory, hex_info = intel_hex(output / "pwm_02.hex")
    load, axf_info = elf_load(output / "pwm_02.axf")
    require(memory == load, "HEX/AXF load bytes differ")
    binary = binary_path.read_bytes()
    require(
        binary == bytes(memory[address] for address in sorted(memory)), "HEX/BIN differ"
    )
    require(not hex_info["adc_marker_addresses"], "Unexpected @ADC in final image")
    stack, reset = struct.unpack_from("<II", binary)
    require(0x20000000 <= stack <= 0x20020000, "Invalid vector stack")
    require(bool(reset & 1) and (reset & ~1) in memory, "Invalid reset vector")
    require(hex_info["entry"] == axf_info["entry"], "HEX/AXF entry differs")
    map_path = output / "pwm_02.map"
    map_text = map_path.read_text(encoding="utf8", errors="replace")
    linked = symbols(map_text)
    required = (
        "main",
        "command_watchdog_poll",
        "commit_command",
        "clear_command_outputs",
        "driver_shutdown_all",
        "driver_enable_all",
        "Check_Frame",
        "PARSE_SIGNED",
        "Start_PWM",
        "Stop_PWM",
        "HAL_TIM_PeriodElapsedCallback",
        "HAL_UARTEx_RxEventCallback",
        "HAL_UART_Transmit_DMA",
        "MX_TIM1_Init",
        "MX_TIM2_Init",
        "MX_TIM8_Init",
        "MX_USART1_UART_Init",
    )
    for name in required:
        require(name in linked, f"Missing linked symbol: {name}")
        item = linked[name]
        address, size = int(item["code_address"], 16), int(item["bytes"])
        require(
            size > 0 and all(address + i in memory for i in range(size)),
            f"Symbol not in image: {name}",
        )
        owner = str(item["object"]).split("(", 1)[0]
        require(
            not re.search(
                r"Removing " + re.escape(owner) + r"\(i\." + re.escape(name) + r"\)",
                map_text,
            ),
            f"Dead-stripped symbol: {name}",
        )
    default_irqs = verify_no_measurement(map_text, linked)
    for name in (
        "last_valid_cmd_tick",
        "watchdog_timeout_latched",
        "recovery_pending",
        "recovery_zero_tick",
    ):
        require(
            bool(
                re.search(
                    r"^\s*" + name + r"\s+0x200[0-9a-f]+\s+Data", map_text, re.MULTILINE
                )
            ),
            f"Missing watchdog state: {name}",
        )
    for reference in (
        "main.o(i.main) refers to main.o(i.command_watchdog_poll)",
        "main.o(i.main) refers to stm32f4xx_hal_uart.o(i.HAL_UART_Transmit_DMA)",
        "main.o(i.main) refers to main.o(i.Check_Frame)",
        "main.o(i.main) refers to main.o(i.PARSE_SIGNED)",
    ):
        require(reference in map_text, f"Missing main cross-reference: {reference}")
    disasm = disassembly_path.read_text(encoding="utf8", errors="replace")
    poll = disassembly_function(disasm, "command_watchdog_poll")
    commit = disassembly_function(disasm, "commit_command")
    shutdown = disassembly_function(disasm, "driver_shutdown_all")
    enable = disassembly_function(disasm, "driver_enable_all")
    require(bool(re.search(r"CMP\s+\w+,#0x12c", poll)), "Missing 300ms comparison")
    require(
        "BL       driver_shutdown_all" in poll
        and "BL       clear_command_outputs" in poll,
        "Missing watchdog shutdown/zero",
    )
    require(
        poll.index("BL       driver_shutdown_all")
        < poll.index("BL       clear_command_outputs"),
        "Incorrect shutdown order",
    )
    require(
        "BL       driver_enable_all" in poll and bool(re.search(r"CMP\s+\w+,#2", poll)),
        "Missing recovery",
    )
    require(
        "BL       command_watchdog_poll" in commit
        and "BL       clear_command_outputs" in commit,
        "Missing commit protection",
    )
    require(
        all("#0x484" in text and "0x40021400" in text for text in (shutdown, enable)),
        "Missing CTRL_SD GPIO",
    )
    require(
        "MOVS     r2,#0" in shutdown and "MOVS     r2,#1" in enable,
        "Missing GPIO levels",
    )
    return {
        "status": "PASS",
        "scope": "static_only_not_release",
        "hex": hex_info,
        "images": [
            metadata(output / name)
            for name in ("pwm_02.hex", "pwm_02.axf", "pwm_02.map")
        ],
        "hex_axf_bin_equal": True,
        "linked_symbols": {name: linked[name] for name in required},
        "watchdog_300ms_shutdown_recovery": True,
        "ctrl_sd": True,
        "measurement_absent": True,
        "default_sampling_irq_aliases": default_irqs,
        "protocol_pwm_echo_linked": True,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--fromelf", required=True, type=Path)
    parser.add_argument("--report", required=True, type=Path)
    args = parser.parse_args()
    # These are analysis outputs derived from the canonical AXF, not release copies.
    binary = args.report.with_suffix(".bin")
    disassembly = args.report.with_suffix(".fromelf.txt")
    axf = args.output / "pwm_02.axf"
    subprocess.run(
        [str(args.fromelf), "--bin", "--output", str(binary), str(axf)],
        check=True,
        capture_output=True,
    )
    result = subprocess.run(
        [str(args.fromelf), "--text", "-c", "-s", str(axf)],
        check=True,
        capture_output=True,
    )
    disassembly.write_bytes(result.stdout)
    report = verify(args.output, binary, disassembly)
    args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf8")
    print("Canonical image static checks PASS (not independently a release)")


if __name__ == "__main__":
    main()
