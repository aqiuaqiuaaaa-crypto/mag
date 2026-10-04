"""CURT GUI integration tests using synthetic frames and one in-memory port."""

from __future__ import annotations

import csv
import os
import sys
import time
from collections.abc import Callable, Iterator
from datetime import datetime
from pathlib import Path
from threading import Event, get_ident
from types import SimpleNamespace

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import magnetic_dipole_pid as gui
from adc_csv import ADCLogger
from curt_telemetry import CSV_HEADER, ADCStreamDecoder, parse_adc_line
from PySide6.QtWidgets import QApplication


def adc_frame(
    count: int = 50,
    *,
    valid: int = 1,
    running: int = 1,
    flags: int = 0,
) -> bytes:
    """Construct distinguishable raw channels and MCU counters for tests."""
    return (
        f"@ADC,{count},{count * 2},101,202,303,404,505,606,"
        f"{flags},2,3,4,{valid},{running}\r\n"
    ).encode("ascii")


class MemorySerial:
    """One duplex in-memory port; record thread ownership and bounded reads."""

    def __init__(self) -> None:
        self.incoming = bytearray()
        self.frames: list[bytes] = []
        self.read_sizes: list[int] = []
        self.thread_ids: list[int] = []
        self.closed = False
        self.fail_rx = False
        self.fail_tx = False
        self.on_close: Callable[[], None] | None = None

    @property
    def in_waiting(self) -> int:
        if self.fail_rx:
            raise OSError("simulated RX failure")
        return len(self.incoming)

    def read(self, size: int = 1) -> bytes:
        assert not self.closed
        self.thread_ids.append(get_ident())
        self.read_sizes.append(size)
        chunk = bytes(self.incoming[:size])
        del self.incoming[:size]
        return chunk

    def write(self, data: bytes) -> int:
        if self.fail_tx:
            raise OSError("simulated TX failure")
        assert not self.closed and len(data) == gui.cfg.SERIAL_FRAME_BYTES
        self.thread_ids.append(get_ident())
        self.frames.append(data)
        return len(data)

    def close(self) -> None:
        if self.on_close is not None:
            self.on_close()
        self.closed = True


@pytest.fixture(scope="session")
def app() -> QApplication:
    """Reuse Qt's single application without requiring pytest-qt."""
    instance = QApplication.instance()
    if instance is None:
        instance = QApplication([])
    assert isinstance(instance, QApplication)
    return instance


@pytest.fixture
def ports(monkeypatch: pytest.MonkeyPatch) -> list[MemorySerial]:
    """Replace the actual COM opener and enumeration before constructing a GUI."""
    opened: list[MemorySerial] = []

    def open_port(port: str, baud: int, *, timeout: int) -> MemorySerial:
        assert port == "MOCK-COM" and baud == gui.cfg.BAUDRATE and timeout == 0
        serial = MemorySerial()
        opened.append(serial)
        return serial

    monkeypatch.setattr(gui, "_serial", SimpleNamespace(Serial=open_port))
    monkeypatch.setattr(gui, "list_ports", SimpleNamespace(comports=list))
    return opened


@pytest.fixture
def window(
    app: QApplication,
    ports: list[MemorySerial],
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> Iterator[gui.MagneticDipoleControl]:
    """Isolate settings and prohibit any camera opening in all GUI tests."""
    monkeypatch.setattr(gui, "SETTINGS_FILE", str(tmp_path / "settings.json"))

    def no_camera(*args: object, **kwargs: object) -> None:
        raise AssertionError("GUI telemetry tests must not open hardware")

    monkeypatch.setattr(gui.cv2, "VideoCapture", no_camera)
    widget = gui.MagneticDipoleControl()
    widget.timer.stop()  # Advance existing control manually when needed.
    widget.combo_port.addItem("MOCK-COM")
    assert widget.ser is None and widget.cap is None and widget.model_ok
    yield widget
    widget.close()
    if widget.adc_logger is not None:
        assert widget.adc_logger.wait_closed(2.0)
    app.processEvents()


def connect(
    window: gui.MagneticDipoleControl, ports: list[MemorySerial]
) -> MemorySerial:
    """Use the real connect slot, with the fixture's single mocked COM opener."""
    window.toggle_serial()
    assert window.ser is ports[-1] and window.adc_rx_timer.isActive()
    return ports[-1]


def pump_until(app: QApplication, ready: Callable[[], bool]) -> None:
    """Drive real QTimers with a short deadline, without real device activity."""
    deadline = time.monotonic() + 2.0
    while not ready() and time.monotonic() < deadline:
        app.processEvents()
        time.sleep(0.005)
    assert ready()


def test_decoder_created_and_independent_timers(
    window: gui.MagneticDipoleControl,
) -> None:
    assert isinstance(window.adc_decoder, ADCStreamDecoder)
    assert window.adc_snapshot is None and not window.adc_rx_timer.isActive()
    assert window.adc_status_timer.isActive() and not window.timer.isActive()
    assert window.adc_rx_timer.parent() is window
    assert window.tabs.tabText(window.tabs.count() - 1) == "CURT / ADC"
    assert window.adc_health() == "DISCONNECTED"


def test_single_serial_connect_and_disconnect_lifecycle(
    window: gui.MagneticDipoleControl, ports: list[MemorySerial]
) -> None:
    serial = connect(window, ports)
    assert len(ports) == 1
    assert serial.frames == [gui.build_command([0] * 6)[0].encode("ascii")]
    assert window.adc_health() == "STALE"

    def check_stopped_before_close() -> None:
        assert not window.adc_rx_timer.isActive()

    serial.on_close = check_stopped_before_close
    window.toggle_serial()
    assert serial.closed and window.ser is None
    assert not window.adc_rx_timer.isActive()
    assert window.adc_health() == "DISCONNECTED"
    assert serial.frames[-1] == gui.build_command([0] * 6)[0].encode("ascii")


def test_actual_rx_timer_without_camera_tracking_or_adc_tab(
    app: QApplication,
    window: gui.MagneticDipoleControl,
    ports: list[MemorySerial],
) -> None:
    serial = connect(window, ports)
    window.tabs.setCurrentIndex(0)
    serial.incoming.extend(adc_frame())
    pump_until(app, lambda: window.adc_snapshot is not None)
    assert not window.tracking and window.cap is None
    assert window.adc_health() == "LIVE"
    assert set(serial.thread_ids) == {get_ident()}


def test_reconnect_discards_snapshot_partial_frame_and_counters(
    window: gui.MagneticDipoleControl, ports: list[MemorySerial]
) -> None:
    serial = connect(window, ports)
    serial.incoming.extend(adc_frame() + adc_frame(100)[:20])
    window.poll_serial_rx()
    assert window.adc_snapshot is not None and window.adc_decoder.buffer
    old_decoder = window.adc_decoder
    window.toggle_serial()
    assert window.adc_snapshot is None and not window.adc_decoder.buffer
    serial = connect(window, ports)
    assert window.adc_decoder is not old_decoder
    assert window.adc_rx_frames == window.adc_rx_bytes == 0
    assert window.adc_received_at is None and not window.adc_receive_time
    serial.incoming.extend(adc_frame(100)[20:])
    window.poll_serial_rx()
    assert window.adc_snapshot is None
    serial.incoming.extend(adc_frame(150))
    window.poll_serial_rx()
    assert window.adc_snapshot is not None
    assert window.adc_snapshot.frame_count == 150 and window.adc_rx_frames == 1


def test_raw_pole_order_commands_and_all_health_fields(
    window: gui.MagneticDipoleControl, ports: list[MemorySerial]
) -> None:
    serial = connect(window, ports)
    window.send_commands([7, -6, 5, -4, 3, -2])
    serial.incoming.extend(adc_frame())
    window.poll_serial_rx()
    assert window.adc_snapshot is not None
    assert window.adc_snapshot.raw == (101, 202, 303, 404, 505, 606)
    for index, pole in enumerate((1, 3, 5, 4, 6, 2)):
        item = window.tbl_adc.item(index, 0)
        assert item is not None and item.text() == f"a{index} / Pole{pole}"
        assert window.adc_raw_cells[index].text() == str((index + 1) * 101)
        assert (
            window.adc_command_cells[index].text()
            == f"{window.last_sent_cmd[index]:+03d}"
        )
    expected = {
        "frame_count": "50",
        "timestamp_mcu": "100",
        "valid": "1",
        "running": "1",
        "error_flags": "0x00000000",
        "overrun_count": "2",
        "dma_error_count": "3",
        "dma_late_count": "4",
    }
    for key, value in expected.items():
        assert window.adc_info_labels[key].text() == value
    assert datetime.fromisoformat(window.adc_receive_time).utcoffset() is not None
    assert window.adc_info_labels["age"].text().endswith(" ms")
    assert window.adc_snapshot.csv_row("pc")[3:9] == (101, 202, 303, 404, 505, 606)


@pytest.mark.parametrize("split", [3, 21, -1])
def test_gui_partial_frames(
    window: gui.MagneticDipoleControl, ports: list[MemorySerial], split: int
) -> None:
    serial = connect(window, ports)
    frame = adc_frame()
    serial.incoming.extend(frame[:split])
    window.poll_serial_rx()
    assert window.adc_snapshot is None
    serial.incoming.extend(frame[split:])
    window.poll_serial_rx()
    assert window.adc_snapshot is not None and window.adc_rx_frames == 1


def test_gui_echo_coalescing_unknown_prefix_and_resynchronization(
    window: gui.MagneticDipoleControl, ports: list[MemorySerial]
) -> None:
    serial = connect(window, ports)
    serial.incoming.extend(
        b"123456789012"
        + adc_frame(50)
        + adc_frame(100)
        + b"@OTHER,noise\r\n@ADC,broken"
        + adc_frame(150)
    )
    window.poll_serial_rx()
    assert window.adc_snapshot is not None
    assert window.adc_snapshot.frame_count == 150 and window.adc_rx_frames == 3
    assert window.adc_decoder.rejected == 1 and window.adc_health() == "LIVE"


def test_gui_malformed_frame_does_not_replace_last_snapshot(
    window: gui.MagneticDipoleControl, ports: list[MemorySerial]
) -> None:
    serial = connect(window, ports)
    serial.incoming.extend(adc_frame())
    window.poll_serial_rx()
    old = window.adc_snapshot
    received = window.adc_received_at
    serial.incoming.extend(b"@ADC,invalid\r\n@OTHER,1\r\n123456789012")
    window.poll_serial_rx()
    assert window.adc_snapshot is old and window.adc_received_at == received
    assert window.adc_decoder.rejected == 1 and window.adc_rx_frames == 1


@pytest.mark.parametrize("valid,running,flags", [(0, 1, 0), (1, 0, 0), (1, 1, 8)])
def test_gui_invalid_status_only_changes_monitor(
    window: gui.MagneticDipoleControl,
    ports: list[MemorySerial],
    valid: int,
    running: int,
    flags: int,
) -> None:
    serial = connect(window, ports)
    window.send_commands([5, -5, 5, -5, 5, -5])
    commands = list(window.last_sent_cmd)
    serial.incoming.extend(adc_frame(valid=valid, running=running, flags=flags))
    window.poll_serial_rx()
    assert window.adc_health() == "INVALID" and window.last_sent_cmd == commands
    assert not serial.closed and window.adc_snapshot is not None
    assert window.adc_snapshot.raw == (101, 202, 303, 404, 505, 606)


def test_gui_stale_age_and_disconnect_clear_raw(
    window: gui.MagneticDipoleControl, ports: list[MemorySerial]
) -> None:
    serial = connect(window, ports)
    serial.incoming.extend(adc_frame())
    window.poll_serial_rx()
    assert window.adc_received_at is not None
    assert window.adc_health(window.adc_received_at + gui.ADC_STALE_S + 0.01) == "STALE"
    window.adc_received_at = time.monotonic() - gui.ADC_STALE_S - 0.1
    window._refresh_adc_display()
    assert window.lbl_adc_health.text() == "STALE"
    assert window.adc_raw_cells[0].text() == "101"
    window.toggle_serial()
    assert window.lbl_adc_health.text() == "DISCONNECTED"
    assert all(cell.text() == "—" for cell in window.adc_raw_cells)
    assert window.adc_info_labels["age"].text() == "—"


def test_gui_rx_reads_only_available_bytes_with_single_poll_budget(
    window: gui.MagneticDipoleControl, ports: list[MemorySerial]
) -> None:
    serial = connect(window, ports)
    window.poll_serial_rx()
    assert serial.read_sizes == []
    serial.incoming.extend(b"x" * (gui.ADC_RX_MAX_BYTES * 3) + adc_frame())
    window.poll_serial_rx()
    assert serial.read_sizes == [gui.ADC_RX_MAX_BYTES]
    assert window.adc_rx_bytes == gui.ADC_RX_MAX_BYTES
    while serial.in_waiting:
        window.poll_serial_rx()
    assert max(serial.read_sizes) <= gui.ADC_RX_MAX_BYTES
    assert window.adc_snapshot is not None and window.adc_rx_frames == 1
    assert len(window.adc_decoder.buffer) <= 4


def test_rx_failure_isolated_from_signed_tx_and_port_ownership(
    window: gui.MagneticDipoleControl, ports: list[MemorySerial]
) -> None:
    serial = connect(window, ports)
    serial.fail_rx = True
    window.poll_serial_rx()
    assert window.adc_rx_errors == 1 and window.adc_health() == "INVALID"
    window.send_commands([9, -9, 3, -3, 0, 0])
    assert serial.frames[-1] == gui.build_command(window.last_sent_cmd)[0].encode(
        "ascii"
    )
    assert window.ser is serial and not serial.closed and len(ports) == 1
    serial.fail_rx = False
    serial.incoming.extend(adc_frame())
    window.poll_serial_rx()
    assert window.adc_health() == "LIVE" and window.adc_rx_errors == 1


def test_unexpected_parser_exception_isolated_from_control(
    window: gui.MagneticDipoleControl,
    ports: list[MemorySerial],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    serial = connect(window, ports)
    original = window.adc_decoder.feed

    def fail_parser(chunk: bytes) -> list[object]:
        raise RuntimeError("unexpected decoder error")

    monkeypatch.setattr(window.adc_decoder, "feed", fail_parser)
    serial.incoming.extend(adc_frame())
    window.poll_serial_rx()
    assert window.adc_parser_errors == 1 and window.adc_health() == "INVALID"
    window.send_commands([9] * 6)
    assert window.last_sent_cmd == [9] * 6 and not serial.closed
    monkeypatch.setattr(window.adc_decoder, "feed", original)
    serial.incoming.extend(adc_frame(100))
    window.poll_serial_rx()
    assert window.adc_health() == "LIVE"


def test_adc_csv_buttons_record_each_snapshot_and_stop_without_stopping_rx(
    window: gui.MagneticDipoleControl,
    ports: list[MemorySerial],
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    serial = connect(window, ports)
    path = tmp_path / "adc.csv"
    monkeypatch.setattr(
        gui.QFileDialog, "getSaveFileName", lambda *a: (str(path), "CSV")
    )
    window.btn_adc_path.click()
    window.btn_adc_log_start.click()
    logger = window.adc_logger
    assert logger is not None and logger.opened.wait(2.0)
    serial.incoming.extend(adc_frame(50) + adc_frame(100) + adc_frame(150))
    window.poll_serial_rx()
    window.btn_adc_log_stop.click()
    assert logger.wait_closed(2.0) and not logger.error
    assert window.adc_rx_timer.isActive()
    with path.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.reader(stream))
    assert rows[0] == list(CSV_HEADER) and len(rows) == 4
    assert [row[1:3] for row in rows[1:]] == [
        ["50", "100"],
        ["100", "200"],
        ["150", "300"],
    ]
    assert all(
        row[3:9] == ["101", "202", "303", "404", "505", "606"] for row in rows[1:]
    )
    assert all(
        datetime.fromisoformat(row[0]).utcoffset() is not None for row in rows[1:]
    )
    serial.incoming.extend(adc_frame(200))
    window.poll_serial_rx()
    assert window.adc_snapshot is not None and window.adc_snapshot.frame_count == 200
    assert logger.rows_written == 3


def test_csv_failure_preserves_existing_file_and_does_not_affect_tx(
    window: gui.MagneticDipoleControl, ports: list[MemorySerial], tmp_path: Path
) -> None:
    serial = connect(window, ports)
    path = tmp_path / "keep.csv"
    path.write_text("KEEP", encoding="utf-8")
    window.edit_adc_csv.setText(str(path))
    window.start_adc_logging()
    logger = window.adc_logger
    assert logger is not None and logger.wait_closed(2.0) and logger.error
    serial.incoming.extend(adc_frame())
    window.poll_serial_rx()
    window.send_commands([5] * 6)
    assert path.read_text(encoding="utf-8") == "KEEP"
    assert window.adc_health() == "LIVE" and window.last_sent_cmd == [5] * 6


def test_bounded_csv_queue_overflow_stops_logging_without_waiting(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    release = Event()
    original = ADCLogger._run

    def delayed(self: ADCLogger) -> None:
        assert release.wait(2.0)
        original(self)

    monkeypatch.setattr(ADCLogger, "_run", delayed)
    logger = ADCLogger(tmp_path / "bounded.csv", capacity=1)
    snapshot = parse_adc_line(adc_frame())
    assert snapshot is not None
    try:
        assert logger.enqueue(snapshot, "pc")
        assert not logger.enqueue(snapshot, "pc") and "queue full" in logger.error
        assert not logger.enqueue(snapshot, "pc")
    finally:
        release.set()
        assert logger.wait_closed(2.0)
    assert logger.rows_written == 1


def test_adc_csv_rejects_invalid_capacity(tmp_path: Path) -> None:
    with pytest.raises(ValueError):
        ADCLogger(tmp_path / "unused.csv", capacity=0)
    assert not (tmp_path / "unused.csv").exists()


def test_slow_csv_disk_work_does_not_wait_in_gui_rx_or_tx(
    window: gui.MagneticDipoleControl,
    ports: list[MemorySerial],
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    serial = connect(window, ports)
    entered = Event()
    release = Event()
    original = ADCLogger._run

    def stalled_disk(self: ADCLogger) -> None:
        entered.set()
        assert release.wait(2.0)
        original(self)

    monkeypatch.setattr(ADCLogger, "_run", stalled_disk)
    window.edit_adc_csv.setText(str(tmp_path / "slow_disk.csv"))
    window.start_adc_logging()
    logger = window.adc_logger
    assert logger is not None and entered.wait(2.0)
    try:
        serial.incoming.extend(adc_frame() + adc_frame(100))
        window.poll_serial_rx()
        window.send_commands([5] * 6)
        assert not logger.opened.is_set()
        assert window.adc_rx_frames == 2 and window.last_sent_cmd == [5] * 6
        assert window.adc_health() == "LIVE"
    finally:
        logger.request_stop()
        release.set()
        assert logger.wait_closed(2.0)
    assert logger.rows_written == 2


def test_mpc_active_telemetry_never_changes_shared_control_state_and_stop_keeps_rx(
    app: QApplication, window: gui.MagneticDipoleControl, ports: list[MemorySerial]
) -> None:
    serial = connect(window, ports)
    window.bead = (800, 540, 300)
    window.estimate_state(1.0 / 30.0, True)
    window.make_rect()
    window.start_tracking()
    worker = window.worker
    shared = window.shared
    assert worker is not None and worker.is_alive()
    pump_until(app, lambda: shared.get_I_target()[1] > 0)
    before_pos = window.state_pos_mm.copy()
    before_vel = window.state_vel_mm.copy()
    before_z3 = window.z3.copy()
    commands = list(window.last_sent_cmd)
    serial.incoming.extend(adc_frame() + b"@ADC,invalid\r\n")
    window.poll_serial_rx()
    assert window.tracking and window.worker is worker and window.shared is shared
    assert np.array_equal(window.state_pos_mm, before_pos)
    assert np.array_equal(window.state_vel_mm, before_vel)
    assert np.array_equal(window.z3, before_z3) and window.last_sent_cmd == commands
    window.mpc_track_step(1.0 / 30.0)
    assert window.tracking and serial.frames[-1].startswith(b"a0:")
    window.normal_stop()
    assert window.adc_rx_timer.isActive() and not serial.closed
    serial.incoming.extend(adc_frame(100))
    window.poll_serial_rx()
    assert window.adc_snapshot is not None and window.adc_snapshot.frame_count == 100
    window.emergency_stop()
    assert window.adc_rx_timer.isActive()
    assert serial.frames[-1] == gui.build_command([0] * 6)[0].encode("ascii")


def test_tx_disconnect_cleanup_on_next_rx_tick(
    window: gui.MagneticDipoleControl, ports: list[MemorySerial]
) -> None:
    serial = connect(window, ports)
    serial.fail_tx = True
    window.send_commands([5] * 6)
    assert window.ser is None and serial.closed
    window.poll_serial_rx()
    assert not window.adc_rx_timer.isActive() and window.adc_health() == "DISCONNECTED"


def test_connection_zero_write_failure_releases_port(
    window: gui.MagneticDipoleControl,
    ports: list[MemorySerial],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    serial = MemorySerial()
    serial.fail_tx = True
    monkeypatch.setattr(gui._serial, "Serial", lambda *a, **kw: serial)
    monkeypatch.setattr(gui.QMessageBox, "warning", lambda *a: None)
    window.toggle_serial()
    assert window.ser is None and serial.closed
    assert not window.adc_rx_timer.isActive() and window.adc_health() == "DISCONNECTED"


def test_close_stops_timers_reader_and_flushes_csv(
    window: gui.MagneticDipoleControl,
    ports: list[MemorySerial],
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    serial = connect(window, ports)
    path = tmp_path / "on_close.csv"
    window.edit_adc_csv.setText(str(path))
    window.start_adc_logging()
    logger = window.adc_logger
    assert logger is not None and logger.opened.wait(2.0)
    window.send_commands([9] * 6)
    original_wait = logger.wait_closed

    def verify_safe_before_disk_wait(timeout: float = 1.0) -> bool:
        assert serial.closed
        assert serial.frames[-1] == gui.build_command([0] * 6)[0].encode("ascii")
        return original_wait(timeout)

    monkeypatch.setattr(logger, "wait_closed", verify_safe_before_disk_wait)
    serial.incoming.extend(adc_frame())
    window.poll_serial_rx()
    reads = len(serial.read_sizes)
    window.close()
    assert logger.finished.is_set() and logger.wait_closed(0)
    assert serial.closed and len(serial.read_sizes) == reads
    assert not window.adc_rx_timer.isActive()
    assert not window.adc_status_timer.isActive() and not window.timer.isActive()
    with path.open(encoding="utf-8", newline="") as stream:
        assert len(list(csv.reader(stream))) == 2


def test_disconnect_finishes_csv_and_reconnect_does_not_resume_it(
    window: gui.MagneticDipoleControl, ports: list[MemorySerial], tmp_path: Path
) -> None:
    serial = connect(window, ports)
    window.edit_adc_csv.setText(str(tmp_path / "session.csv"))
    window.start_adc_logging()
    logger = window.adc_logger
    assert logger is not None and logger.opened.wait(2.0)
    serial.incoming.extend(adc_frame())
    window.poll_serial_rx()
    window.toggle_serial()
    assert logger.wait_closed(2.0) and logger.rows_written == 1
    serial = connect(window, ports)
    serial.incoming.extend(adc_frame(100))
    window.poll_serial_rx()
    assert window.adc_rx_frames == 1 and logger.rows_written == 1
