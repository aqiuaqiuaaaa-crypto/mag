"""Stop regressions using real mode entries, synthetic vision and captured UART."""

from __future__ import annotations

import csv
import sys
from itertools import pairwise
from pathlib import Path
from unittest.mock import Mock

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import magnetic_dipole_pid as gui
import test_gui_curt_telemetry as telemetry_tests
from test_gui_control_log import configure_camera
from test_gui_curt_telemetry import MemorySerial, adc_frame, connect

app = telemetry_tests.app
ports = telemetry_tests.ports
window = telemetry_tests.window

ACTIVE_MODES = (
    "tracking",
    "dir_test",
    "friction_calibration",
    "coil_scan",
    "manual_current",
    "manual_force",
)
ZERO_FRAME = b"a0:+00,a1:+00,a2:+00,a3:+00,a4:+00,a5:+00\r\n"


@pytest.fixture
def clock(monkeypatch: pytest.MonkeyPatch) -> list[float]:
    """Use identical control dt; run the actual MPC worker at explicit deadlines."""
    value = [1000.0]
    monkeypatch.setattr(gui.time, "time", lambda: value[0])
    monkeypatch.setattr(gui.ControlWorker, "start", lambda self: None)
    return value


def commands(frame: bytes) -> list[int]:
    assert len(frame) == 43 and frame.endswith(b"\r\n")
    return [int(pair.split(b":")[1]) for pair in frame.strip().split(b",")]


def advance(
    widget: gui.MagneticDipoleControl, clock: list[float], count: int = 1
) -> None:
    for _ in range(count):
        clock[0] += 1 / gui.cfg.CONTROL_HZ
        if (
            widget.tracking
            and widget.worker is not None
            and widget._control_tick_count % 3 == 0
        ):
            widget.worker._step()
        widget.tick()


def enter_mode(
    widget: gui.MagneticDipoleControl, mode: str, clock: list[float]
) -> MemorySerial:
    """Activate each real GUI mode and wait for actual nonzero serial output."""
    port = MemorySerial()
    widget.ser = port
    configure_camera(widget)
    widget.last_time = clock[0]
    advance(widget, clock)
    if mode == "tracking":
        widget.path_px = [(160, 90), (240, 90), (280, 90)]
        widget.start_tracking()
    elif mode == "dir_test":
        widget.spin_ftx.setValue(10)
        widget.btn_dir_test.setChecked(True)
    elif mode == "friction_calibration":
        widget.btn_calib.setChecked(True)
    elif mode == "coil_scan":
        widget.start_coil_scan()
    elif mode == "manual_current":
        for spin, value in zip(widget.cur_spins, (40, -30, 20, -10, 9, -8)):
            spin.setValue(value)
        widget.chk_cur_live.setChecked(True)
    elif mode == "manual_force":
        widget.spin_fx.setValue(10)
        widget.chk_force_live.setChecked(True)
    else:
        raise AssertionError(mode)
    for _ in range(80):
        advance(widget, clock)
        if max(map(abs, widget.last_sent_cmd)) >= 18:
            break
    assert max(map(abs, widget.last_sent_cmd)) >= 18, mode
    assert any(frame != ZERO_FRAME for frame in port.frames), mode
    return port


def assert_modes_cleared(widget: gui.MagneticDipoleControl) -> None:
    assert not widget.tracking and not widget.dir_test_on and not widget.calib_active
    assert widget.coil_test_idx is None and widget.coil_test_phase == "zero"
    assert widget.coil_test_frames == widget.coil_test_samples == 0
    assert np.array_equal(widget.coil_test_vsum, np.zeros(2))
    assert widget.coil_test_start is None
    assert not widget.btn_dir_test.isChecked() and not widget.btn_calib.isChecked()
    assert not widget.chk_cur_live.isChecked() and not widget.chk_force_live.isChecked()
    assert widget.combo_constraint.isEnabled()
    assert widget.shared.stopped() and widget.worker is None


def watch_active_branches(
    widget: gui.MagneticDipoleControl, monkeypatch: pytest.MonkeyPatch
) -> list[Mock]:
    spies = []
    for name in (
        "control_step",
        "direction_test_step",
        "calib_step",
        "coil_test_step",
        "_solve_and_send",
    ):
        spy = Mock(wraps=getattr(widget, name))
        monkeypatch.setattr(widget, name, spy)
        spies.append(spy)
    return spies


def test_coil_drive_emergency_cannot_restart(
    window: gui.MagneticDipoleControl,
    clock: list[float],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    port = enter_mode(window, "coil_scan", clock)
    # Accumulate real samples, rather than just fabricating stale counters.
    for _ in range(40):
        if window.coil_test_samples >= 3:
            break
        advance(window, clock)
    assert window.coil_test_idx == 0 and window.coil_test_phase == "drive"
    assert window.coil_test_samples >= 3 and window.coil_test_frames > 10
    assert window.coil_test_start is not None and any(window.last_sent_cmd)
    scan = Mock(wraps=window.coil_test_step)
    monkeypatch.setattr(window, "coil_test_step", scan)
    first_stop_frame = len(port.frames)
    window.emergency_stop()
    assert_modes_cleared(window)
    assert window.mode == "IDLE" and not window.stopping
    advance(window, clock, 30)
    # Immediate zero followed by one unchanged IDLE heartbeat per tick.
    assert port.frames[first_stop_frame:] == [ZERO_FRAME] * 31
    assert window.last_sent_cmd == [0] * 6
    scan.assert_not_called()


@pytest.mark.parametrize("mode", ACTIVE_MODES)
def test_emergency_disarms_every_active_mode(
    window: gui.MagneticDipoleControl,
    clock: list[float],
    monkeypatch: pytest.MonkeyPatch,
    mode: str,
) -> None:
    port = enter_mode(window, mode, clock)
    spies = watch_active_branches(window, monkeypatch)
    stop_index = len(port.frames)
    window.emergency_stop()
    assert_modes_cleared(window)
    assert window.last_sent_cmd == [0] * 6 and not window.stopping
    advance(window, clock, 30)
    assert port.frames[stop_index:] == [ZERO_FRAME] * 31
    assert window.mode == "IDLE" and window.last_sent_cmd == [0] * 6
    for spy in spies:
        spy.assert_not_called()


@pytest.mark.parametrize("mode", ACTIVE_MODES)
def test_normal_stop_keeps_existing_slew_and_stays_zero(
    window: gui.MagneticDipoleControl,
    clock: list[float],
    monkeypatch: pytest.MonkeyPatch,
    mode: str,
) -> None:
    port = enter_mode(window, mode, clock)
    spies = watch_active_branches(window, monkeypatch)
    before = list(window.last_sent_cmd)
    frame_index = len(port.frames)
    window.normal_stop()
    assert_modes_cleared(window)
    assert window.stopping and window.mode == "STOPPING"
    assert window.last_sent_cmd == before and len(port.frames) == frame_index
    # This is the original strategy: only tick sends slew-limited zero targets.
    expected = []
    previous = before
    for _ in range(30):
        if any(previous):
            previous = gui.apply_slew(
                [0] * 6,
                previous,
                gui.cfg.MAX_DELTA_CMD,
                max_cmd=window._current_cmd_limit(),
            )
            expected.append(
                gui.build_command(previous, max_cmd=window._current_cmd_limit())[
                    0
                ].encode("ascii")
            )
        else:
            expected.append(ZERO_FRAME)  # New IDLE heartbeat after original ramp.
        advance(window, clock)
        assert window.last_sent_cmd == previous
    assert port.frames[frame_index:] == expected and expected[-1] == ZERO_FRAME
    actual = [before, *(commands(frame) for frame in expected)]
    for old, new in pairwise(actual):
        assert all(
            abs(b) <= abs(a) and abs(b - a) <= gui.cfg.MAX_DELTA_CMD
            for a, b in zip(old, new)
        )
    assert not window.stopping and window.mode == "IDLE"
    for spy in spies:
        spy.assert_not_called()


@pytest.mark.parametrize("stop_name", ["normal_stop", "emergency_stop"])
def test_both_stops_use_the_same_cleanup(
    window: gui.MagneticDipoleControl,
    monkeypatch: pytest.MonkeyPatch,
    stop_name: str,
) -> None:
    clear = Mock(wraps=window._clear_active_modes)
    monkeypatch.setattr(window, "_clear_active_modes", clear)
    getattr(window, stop_name)()
    clear.assert_called_once()
    assert_modes_cleared(window)


def test_emergency_zero_precedes_worker_join(
    window: gui.MagneticDipoleControl,
    clock: list[float],
) -> None:
    port = enter_mode(window, "tracking", clock)
    events: list[str] = []

    class ObservedSerial(MemorySerial):
        def write(self, data: bytes) -> int:
            assert data == ZERO_FRAME
            assert not window.tracking and window.coil_test_idx is None
            assert window.shared.stopped()
            events.append("zero")
            return super().write(data)

    class JoiningWorker:
        def is_alive(self) -> bool:
            return True

        def join(self, timeout: float) -> None:
            assert timeout == 1.0 and events == ["zero"]
            assert window.last_sent_cmd == [0] * 6
            events.append("join")

    window.ser = ObservedSerial()
    window.worker = JoiningWorker()
    window.emergency_stop()
    assert events == ["zero", "join"] and window.worker is None
    assert not port.closed


@pytest.mark.parametrize("stop_name", ["normal_stop", "emergency_stop"])
def test_stopping_twice_preserves_records_and_manual_setpoints(
    window: gui.MagneticDipoleControl,
    clock: list[float],
    stop_name: str,
) -> None:
    enter_mode(window, "coil_scan", clock)
    record = {"coil": 0, "velocity": np.array([1, 2]), "F_model": np.zeros(3)}
    window.coil_test_records.append(record)
    window.calib_records.append({"Fz": 0, "N": 1, "F_start": 2, "v": 3})
    window.man_currents = [10] * 6
    for _ in range(2):
        getattr(window, stop_name)()
        assert_modes_cleared(window)
    assert window.coil_test_records == [record] and len(window.calib_records) == 1
    assert window.man_currents == [10] * 6
    advance(window, clock, 30)
    assert window.last_sent_cmd == [0] * 6 and window.mode == "IDLE"


def test_stop_keeps_existing_adc_monitor_and_log_fields(
    window: gui.MagneticDipoleControl,
    ports: list[MemorySerial],
    clock: list[float],
    tmp_path: Path,
) -> None:
    configure_camera(window)
    window.last_time = clock[0]
    advance(window, clock)
    port = connect(window, ports)
    window.start_coil_scan()
    advance(window, clock, 15)
    assert any(window.last_sent_cmd)
    window.edit_control_log.setText(str(tmp_path / "stop.csv"))
    window.start_control_log()
    sink = window.control_logger
    assert sink is not None
    window.emergency_stop()
    port.incoming.extend(adc_frame(88))
    window.poll_serial_rx()
    advance(window, clock, 30)
    assert window.adc_rx_timer.isActive() and window.adc_status_timer.isActive()
    assert window.ser is port and window.adc_snapshot is not None
    assert window.adc_snapshot.frame_count == 88
    window.stop_control_log()
    assert sink.wait_closed(3) and sink.error is None
    with sink.path.open(newline="", encoding="utf8") as file:
        records = list(csv.DictReader(file))
    assert len(records) == 30
    assert all(row["mode"] == "IDLE" and row["tracking"] == "False" for row in records)
    assert all(row[f"cmd_sent_a{i}"] == "0" for row in records for i in range(6))
    assert all(row["raw5"] == "606" for row in records)


@pytest.mark.parametrize("stop_name", ["normal_stop", "emergency_stop"])
def test_stop_joins_actual_tracking_worker(
    window: gui.MagneticDipoleControl,
    stop_name: str,
) -> None:
    configure_camera(window)
    window.ser = MemorySerial()
    window.tick()
    window.path_px = [(160, 90), (260, 90)]
    window.start_tracking()
    worker = window.worker
    assert worker is not None and worker.is_alive()
    getattr(window, stop_name)()
    assert not worker.is_alive()
    assert_modes_cleared(window)
