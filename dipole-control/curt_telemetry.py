"""Receive raw CURT telemetry; optional explicit signed commands for bench tests.

The default mode only reads. It never interprets raw ADC counts as amperes.
Use one process per COM port; the GUI must release the port before this recorder.
"""

from __future__ import annotations

import argparse
import asyncio
import csv
import importlib
import re
import sys
import time
from collections.abc import Callable
from contextlib import ExitStack, closing
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Protocol, TextIO, cast

PREFIX = b"@ADC,"
MAX_LINE_BYTES = 128
UINT32_MAX = (1 << 32) - 1
ZERO_COMMAND = b"a0:+00,a1:+00,a2:+00,a3:+00,a4:+00,a5:+00\r\n"
CSV_HEADER = (
    "timestamp_pc",
    "frame_count",
    "timestamp_mcu",
    "raw0",
    "raw1",
    "raw2",
    "raw3",
    "raw4",
    "raw5",
    "error_flags",
    "overrun_count",
    "dma_error_count",
    "dma_late_count",
    "valid",
    "running",
)


@dataclass(frozen=True, slots=True)
class ADCSnapshot:
    """One complete uncalibrated MCU snapshot, in logical a0 through a5 order."""

    frame_count: int
    timestamp_mcu: int
    raw: tuple[int, int, int, int, int, int]
    error_flags: int
    overrun_count: int
    dma_error_count: int
    dma_late_count: int
    valid: int
    running: int

    def csv_row(self, timestamp_pc: str) -> tuple[str | int, ...]:
        """Return fields in the documented CSV order."""
        return (
            timestamp_pc,
            self.frame_count,
            self.timestamp_mcu,
            *self.raw,
            self.error_flags,
            self.overrun_count,
            self.dma_error_count,
            self.dma_late_count,
            self.valid,
            self.running,
        )


def parse_adc_line(line: bytes) -> ADCSnapshot | None:
    """Parse a strict @ADC CRLF line; ignore other prefixes, reject malformed ADC.

    Raises:
        ValueError: An ADC record has invalid framing, field count or values.
    """
    if not line.startswith(PREFIX):
        return None
    if len(line) > MAX_LINE_BYTES or not line.endswith(b"\r\n"):
        raise ValueError("ADC line must be bounded and terminated by CRLF")
    fields = line[len(PREFIX) : -2].split(b",")
    if len(fields) != 14 or any(
        re.fullmatch(rb"[0-9]{1,10}", f) is None for f in fields
    ):
        raise ValueError("ADC line requires 14 unsigned decimal fields")
    values = [int(f) for f in fields]
    if any(v > UINT32_MAX for v in values):
        raise ValueError("ADC uint32 overflow")
    if any(v > 4095 for v in values[2:8]) or any(v > 1 for v in values[12:14]):
        raise ValueError("ADC raw or status flag out of range")
    return ADCSnapshot(
        values[0],
        values[1],
        cast(tuple[int, int, int, int, int, int], tuple(values[2:8])),
        values[8],
        values[9],
        values[10],
        values[11],
        values[12],
        values[13],
    )


class ADCStreamDecoder:
    """Bounded framing, including legacy 12-byte echo without a newline."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.rejected = 0

    def feed(self, chunk: bytes) -> list[ADCSnapshot]:
        """Recover complete ADC records from arbitrary split/coalesced bytes."""
        self.buffer.extend(chunk)
        snapshots: list[ADCSnapshot] = []
        while self.buffer:
            start = self.buffer.find(PREFIX)
            if start < 0:
                self.buffer[:] = self.buffer[-(len(PREFIX) - 1) :]
                break
            del self.buffer[:start]
            end = self.buffer.find(b"\n")
            restart = self.buffer.find(PREFIX, len(PREFIX))
            if restart >= 0 and (end < 0 or restart < end):
                self.rejected += 1
                del self.buffer[:restart]
                continue
            if end < 0:
                if len(self.buffer) > MAX_LINE_BYTES:
                    self.rejected += 1
                    self.buffer[:] = self.buffer[-(len(PREFIX) - 1) :]
                break
            line = bytes(self.buffer[: end + 1])
            del self.buffer[: end + 1]
            try:
                snapshot = parse_adc_line(line)
            except ValueError:
                self.rejected += 1
                continue
            if snapshot is not None:
                snapshots.append(snapshot)
        return snapshots


def command_frame(value: str) -> bytes:
    """Validate an explicitly supplied existing signed control frame."""
    body = value.removesuffix("\r\n")
    pattern = ",".join(rf"a{i}:[+-][0-9]{{2}}" for i in range(6))
    if re.fullmatch(pattern, body, flags=re.ASCII) is None:
        raise argparse.ArgumentTypeError(
            "command must contain a0..a5 signed two-digit values"
        )
    return body.encode("ascii") + b"\r\n"


def sample_rate(previous: ADCSnapshot | None, current: ADCSnapshot) -> float | None:
    """Estimate published frames/s with uint32 wrap handling, excluding resets."""
    if previous is None:
        return None
    dt = (current.timestamp_mcu - previous.timestamp_mcu) & UINT32_MAX
    df = (current.frame_count - previous.frame_count) & UINT32_MAX
    if dt == 0:
        return 0.0 if df == 0 else None
    if dt > 10000 or df > 10000:
        return None
    return df * 1000.0 / dt


class SerialPort(Protocol):
    """Minimal serial interface, also usable by an in-memory test port."""

    @property
    def in_waiting(self) -> int: ...
    def read(self, size: int = 1) -> bytes: ...
    def write(self, data: bytes) -> int: ...
    def close(self) -> None: ...


async def receive(
    port: SerialPort,
    csv_output: TextIO | None = None,
    duration_s: float | None = None,
    command: bytes | None = None,
    emit: Callable[[str], None] = print,
    command_delay_s: float = 2.0,
) -> int:
    """Read a nonblocking port, print states and optionally log CSV.

    An explicit command enables a 30 Hz fixed-command bench test, with a zero
    frame at entry/exit and two seconds of zero-output baseline by default.
    Read-only mode never writes to the MCU, including on exit.
    """
    decoder = ADCStreamDecoder()
    writer = csv.writer(csv_output) if csv_output is not None else None
    if writer is not None:
        writer.writerow(CSV_HEADER)
        assert csv_output is not None
        csv_output.flush()
    previous: ADCSnapshot | None = None
    received = 0
    started = time.monotonic()
    next_command = started + command_delay_s
    next_wait_notice = started + 5.0
    try:
        if command is not None:
            port.write(ZERO_COMMAND)
        while duration_s is None or time.monotonic() - started < duration_s:
            now = time.monotonic()
            if command is not None and now >= next_command:
                port.write(command)
                next_command = now + 1.0 / 30.0
            chunk = port.read(min(max(port.in_waiting, 1), 256))
            for snapshot in decoder.feed(chunk):
                timestamp = datetime.now(timezone.utc).isoformat(
                    timespec="milliseconds"
                )
                rate = sample_rate(previous, snapshot)
                rate_text = "--" if rate is None else f"{rate:.1f}"
                emit(
                    f"{timestamp} fc={snapshot.frame_count} mcu_ms={snapshot.timestamp_mcu} "
                    f"raw={list(snapshot.raw)} flags=0x{snapshot.error_flags:08X} "
                    f"ovr={snapshot.overrun_count} dma={snapshot.dma_error_count} "
                    f"late={snapshot.dma_late_count} valid={snapshot.valid} "
                    f"running={snapshot.running} sample_hz={rate_text} "
                    f"rejected={decoder.rejected}"
                )
                if writer is not None:
                    writer.writerow(snapshot.csv_row(timestamp))
                    assert csv_output is not None
                    csv_output.flush()
                previous = snapshot
                received += 1
                next_wait_notice = now + 5.0
            if now >= next_wait_notice:
                emit(
                    f"No complete @ADC frame for 5 seconds; rejected={decoder.rejected}"
                )
                next_wait_notice = now + 5.0
            await asyncio.sleep(0.005)
    finally:
        if command is not None:
            port.write(ZERO_COMMAND)
    return received


def main(argv: list[str] | None = None) -> int:
    """CLI entry point; no hardware is opened when importing this module."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="Existing COM port, e.g. COM4")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--csv", type=Path, help="New CSV file; existing files are preserved"
    )
    parser.add_argument("--duration", type=float, help="Stop after this many seconds")
    parser.add_argument(
        "--command",
        type=command_frame,
        help="Explicit fixed signed frame for bench test",
    )
    args = parser.parse_args(argv)
    if args.baud <= 0 or (args.duration is not None and args.duration <= 0):
        parser.error("baud and duration must be positive")
    if args.command is not None and args.duration is not None and args.duration <= 2.0:
        parser.error("command mode needs more than 2 seconds for the zero baseline")
    try:
        serial_module = importlib.import_module("serial")
    except ModuleNotFoundError:
        print(
            "pyserial is missing; install it in this Python interpreter",
            file=sys.stderr,
        )
        return 2
    try:
        with ExitStack() as stack:
            # Open CSV first so a file error cannot initiate a control session.
            output = (
                stack.enter_context(args.csv.open("x", newline="", encoding="utf-8"))
                if args.csv
                else None
            )
            port = cast(
                SerialPort,
                serial_module.serial_for_url(
                    args.port,
                    baudrate=args.baud,
                    timeout=0,
                    write_timeout=0.5,
                    rtscts=False,
                    dsrdtr=False,
                ),
            )
            stack.enter_context(closing(port))
            print(
                f"{args.port}: {'explicit command test' if args.command else 'read only'}; Ctrl+C to stop"
            )
            count = asyncio.run(receive(port, output, args.duration, args.command))
            print(f"Received {count} ADC frames")
    except KeyboardInterrupt:
        return 0
    except (OSError, ValueError, serial_module.SerialException) as error:
        print(f"Telemetry error: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
