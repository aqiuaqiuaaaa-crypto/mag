"""Parser/framing/recorder regression tests; use in-memory ports only."""

from __future__ import annotations

import asyncio
import csv
import importlib.util
import io
import sys
from pathlib import Path
from typing import Any

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import curt_telemetry as telemetry

LINE = b"@ADC,12345,456789,1572,1576,1568,1581,1570,1575,0,0,0,0,1,1\r\n"


def test_normal_and_csv_order() -> None:
    snapshot = telemetry.parse_adc_line(LINE)
    assert snapshot is not None
    assert snapshot.raw == (1572, 1576, 1568, 1581, 1570, 1575)
    assert snapshot.frame_count == 12345
    row = snapshot.csv_row("pc")
    assert len(row) == len(telemetry.CSV_HEADER) == 15
    assert row[0:3] == ("pc", 12345, 456789)
    assert row[-6:] == (0, 0, 0, 0, 1, 1)


@pytest.mark.parametrize(
    "line",
    [
        b"a0:+30,a1:-45,a2:+60,a3:-75,a4:+90,a5:-10\r\n",
        b"123456789012",
        b"@DEBUG,1,2\r\n",
        b"",
        b"echo" + LINE,
    ],
)
def test_other_prefix_is_not_adc(line: bytes) -> None:
    assert telemetry.parse_adc_line(line) is None


@pytest.mark.parametrize(
    "line",
    [
        LINE[:-1],
        LINE[:-2] + b"\n",
        LINE.replace(b"12345", b"-12345"),
        LINE.replace(b"12345", b"+12345"),
        LINE.replace(b"1572", b"4096"),
        LINE.replace(b"1572", b" 1572"),
        LINE.replace(b"12345", b"4294967296"),
        LINE.replace(b"12345", b"9999999999999999999999999999"),
        LINE.replace(b"1572", b"x"),
        LINE.replace(b",1,1\r\n", b",2,1\r\n"),
        LINE.replace(b",1,1\r\n", b",1,2\r\n"),
        LINE.replace(b",1,1\r\n", b",1\r\n"),
        LINE.replace(b",1,1\r\n", b",1,1,0\r\n"),
        LINE.replace(b"1572", b"\xff"),
        b"@ADC," + b"1" * 200 + b"\r\n",
        LINE.replace(b"1572", b"1.0"),
    ],
)
def test_malformed_adc_is_rejected(line: bytes) -> None:
    with pytest.raises(ValueError):
        telemetry.parse_adc_line(line)


def test_fault_and_boundary_values() -> None:
    snapshot = telemetry.parse_adc_line(
        b"@ADC,4294967295,4294967295,0,4095,0,4095,0,4095,524288,1,2,3,0,0\r\n"
    )
    assert snapshot is not None and snapshot.valid == 0 and snapshot.running == 0
    assert snapshot.error_flags == 524288 and snapshot.dma_late_count == 3


@pytest.mark.parametrize("split", range(len(LINE) + 1))
def test_every_two_chunk_split(split: int) -> None:
    decoder = telemetry.ADCStreamDecoder()
    assert len(decoder.feed(LINE[:split]) + decoder.feed(LINE[split:])) == 1


def test_echo_coalescing_and_truncation_recovery() -> None:
    decoder = telemetry.ADCStreamDecoder()
    records = []
    stream = b"123456789012" + LINE + b"@ADC,1,2,3" + LINE + b"000000000000" + LINE
    for byte in stream:
        records.extend(decoder.feed(bytes([byte])))
    assert len(records) == 3 and decoder.rejected == 1


def test_malformed_and_oversize_recovery() -> None:
    decoder = telemetry.ADCStreamDecoder()
    assert len(decoder.feed(b"@ADC,nope\r\n" + LINE)) == 1
    assert decoder.rejected == 1
    assert decoder.feed(b"@ADC," + b"1" * 10000) == []
    assert len(decoder.buffer) <= 4 and decoder.rejected == 2
    assert len(decoder.feed(LINE)) == 1
    assert decoder.feed(b"unknown" * 10000) == [] and len(decoder.buffer) <= 4


def test_partial_marker_and_new_marker_before_newline() -> None:
    decoder = telemetry.ADCStreamDecoder()
    assert decoder.feed(b"echo@AD") == []
    assert len(decoder.feed(b"C," + LINE[5:])) == 1
    assert len(decoder.feed(b"@ADC,broken" + LINE)) == 1
    assert decoder.rejected == 1


def test_sample_rate_wrap_stall_and_reset() -> None:
    from dataclasses import replace

    snapshot = telemetry.parse_adc_line(LINE)
    assert snapshot is not None
    assert telemetry.sample_rate(None, snapshot) is None
    assert telemetry.sample_rate(snapshot, snapshot) == 0.0
    assert (
        telemetry.sample_rate(
            snapshot, replace(snapshot, frame_count=12395, timestamp_mcu=456889)
        )
        == 500.0
    )
    previous = replace(
        snapshot,
        frame_count=telemetry.UINT32_MAX - 24,
        timestamp_mcu=telemetry.UINT32_MAX - 49,
    )
    current = replace(snapshot, frame_count=25, timestamp_mcu=50)
    assert telemetry.sample_rate(previous, current) == 500.0
    assert (
        telemetry.sample_rate(
            snapshot, replace(snapshot, frame_count=0, timestamp_mcu=0)
        )
        is None
    )
    assert telemetry.sample_rate(snapshot, replace(snapshot, frame_count=12346)) is None


@pytest.mark.parametrize(
    "value",
    [
        "a0:10",
        "",
        "a0:+100",
        "@ADC,1",
        telemetry.ZERO_COMMAND.decode().replace("a0", "a1"),
    ],
)
def test_control_frame_rejects_invalid(value: str) -> None:
    import argparse

    with pytest.raises(argparse.ArgumentTypeError):
        telemetry.command_frame(value)


def test_control_frame_keeps_signed_protocol() -> None:
    command = "a0:+30,a1:-45,a2:+60,a3:-75,a4:+90,a5:-10"
    assert telemetry.command_frame(command) == command.encode() + b"\r\n"
    assert len(telemetry.command_frame(command)) == 43
    assert telemetry.command_frame(command + "\r\n") == telemetry.command_frame(command)


class MemoryPort:
    def __init__(self, data: bytes = LINE) -> None:
        self.data = bytearray(data)
        self.writes: list[bytes] = []
        self.closed = False

    @property
    def in_waiting(self) -> int:
        return len(self.data)

    def read(self, size: int = 1) -> bytes:
        result = bytes(self.data[:size])
        del self.data[:size]
        return result

    def write(self, data: bytes) -> int:
        self.writes.append(data)
        return len(data)

    def close(self) -> None:
        self.closed = True


def test_read_only_recorder_and_csv() -> None:
    port = MemoryPort(b"echo12345678" + LINE + LINE)
    output = io.StringIO()
    messages: list[str] = []
    count = asyncio.run(telemetry.receive(port, output, 0.02, emit=messages.append))
    assert count == 2 and port.writes == []
    rows = list(csv.reader(io.StringIO(output.getvalue())))
    assert rows[0] == list(telemetry.CSV_HEADER) and len(rows) == 3
    assert rows[1][3:9] == ["1572", "1576", "1568", "1581", "1570", "1575"]
    assert "sample_hz=0.0" in messages[-1]


def test_optional_command_and_exit_zero() -> None:
    port = MemoryPort()
    command = telemetry.command_frame("a0:+10,a1:+00,a2:+00,a3:+00,a4:+00,a5:+00")
    asyncio.run(
        telemetry.receive(
            port,
            duration_s=0.02,
            command=command,
            command_delay_s=0,
            emit=lambda _: None,
        )
    )
    assert port.writes == [telemetry.ZERO_COMMAND, command, telemetry.ZERO_COMMAND]


def test_command_exception_still_attempts_zero() -> None:
    class BrokenPort(MemoryPort):
        def read(self, size: int = 1) -> bytes:
            raise OSError("disconnected")

    port = BrokenPort()
    with pytest.raises(OSError, match="disconnected"):
        asyncio.run(telemetry.receive(port, command=telemetry.ZERO_COMMAND))
    assert port.writes == [telemetry.ZERO_COMMAND, telemetry.ZERO_COMMAND]


def test_idle_notice(monkeypatch: pytest.MonkeyPatch) -> None:
    from types import SimpleNamespace

    counter = iter([0.0, 6.0, 6.0, 7.0])
    monkeypatch.setattr(
        telemetry, "time", SimpleNamespace(monotonic=lambda: next(counter, 7.0))
    )
    messages: list[str] = []
    asyncio.run(telemetry.receive(MemoryPort(b""), duration_s=7, emit=messages.append))
    assert "No complete @ADC frame" in messages[0]


def test_cli_read_only_and_existing_csv(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    port = MemoryPort()
    module = importlib.import_module("serial")
    monkeypatch.setattr(module, "serial_for_url", lambda *a, **kw: port)
    path = tmp_path / "raw.csv"
    assert (
        telemetry.main(["--port", "memory", "--csv", str(path), "--duration", "0.02"])
        == 0
    )
    assert port.closed and port.writes == []
    assert telemetry.main(["--port", "memory", "--csv", str(path)]) == 2


@pytest.mark.parametrize(
    "args",
    [
        ["--baud", "0"],
        ["--duration", "0"],
        ["--duration", "1", "--command", telemetry.ZERO_COMMAND.decode()],
    ],
)
def test_cli_invalid_arguments(args: list[str]) -> None:
    with pytest.raises(SystemExit) as error:
        telemetry.main(["--port", "memory", *args])
    assert error.value.code == 2


def test_cli_missing_dependency(monkeypatch: pytest.MonkeyPatch) -> None:
    def missing(name: str) -> Any:
        raise ModuleNotFoundError(name)

    monkeypatch.setattr(importlib, "import_module", missing)
    assert telemetry.main(["--port", "memory"]) == 2


def test_cli_port_error_and_keyboard_interrupt(monkeypatch: pytest.MonkeyPatch) -> None:
    module = importlib.import_module("serial")

    def broken(*args: Any, **kwargs: Any) -> Any:
        raise module.SerialException("port busy")

    monkeypatch.setattr(module, "serial_for_url", broken)
    assert telemetry.main(["--port", "memory"]) == 2

    def interrupted(*args: Any, **kwargs: Any) -> Any:
        raise KeyboardInterrupt

    monkeypatch.setattr(module, "serial_for_url", interrupted)
    assert telemetry.main(["--port", "memory"]) == 0
