"""Serial safety using real GUI modes, synthetic vision and captured writes."""

from __future__ import annotations

import sys
from pathlib import Path
from unittest.mock import Mock

import numpy as np
import pytest
from serial import SerialException, SerialTimeoutException

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import config as cfg
import magnetic_dipole_pid as gui
import test_gui_curt_telemetry as telemetry_tests
import test_stop_modes as stop_tests
from multirate import ControlWorker
from test_gui_control_log import configure_camera
from test_gui_curt_telemetry import MemorySerial
from test_stop_modes import (
    ACTIVE_MODES,
    ZERO_FRAME,
    advance,
    assert_modes_cleared,
    enter_mode,
)

app = telemetry_tests.app
ports = telemetry_tests.ports
window = telemetry_tests.window
clock = stop_tests.clock
FAULT_MESSAGE = "串口故障，STM32 可能仍保持最后成功命令"
IO_ERRORS = (SerialTimeoutException("USB write timeout"), OSError("USB unplugged"))


class FaultSerial(MemorySerial):
    """Record attempts separately from complete successful frames."""

    def __init__(self) -> None:
        super().__init__()
        self.error: Exception | None = None
        self.short_write: int | None = None
        self.attempts: list[bytes] = []

    def write(self, data: bytes) -> int:
        self.attempts.append(data)
        if self.error is not None:
            raise self.error
        if self.short_write is not None:
            return self.short_write
        return super().write(data)


def request_mode(widget: gui.MagneticDipoleControl, mode: str) -> None:
    """Exercise the actual entry callbacks, without manufacturing active flags."""
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
        widget.cur_spins[0].setValue(40)
        widget.chk_cur_live.setChecked(True)
    elif mode == "manual_force":
        widget.spin_fx.setValue(10)
        widget.chk_force_live.setChecked(True)
    else:
        raise AssertionError(mode)


def establish_fault(widget: gui.MagneticDipoleControl) -> tuple[FaultSerial, list[int]]:
    """First succeed, then fail; the fault must retain genuine send history."""
    port = FaultSerial()
    widget.ser = port
    widget.send_commands([9, -9, 8, -8, 7, -7])
    previous = list(widget.last_sent_cmd)
    assert any(previous)
    port.error = OSError("USB unplugged")
    widget.send_commands([30] * 6)
    assert widget.serial_fault and widget.last_sent_cmd == previous
    return port, previous


def recovery_port(
    widget: gui.MagneticDipoleControl,
    monkeypatch: pytest.MonkeyPatch,
    port: FaultSerial,
) -> Mock:
    opener = Mock(return_value=port)
    assert gui._serial is not None
    monkeypatch.setattr(gui._serial, "Serial", opener)
    widget.combo_port.setCurrentText("MOCK-COM")
    widget.toggle_serial()
    opener.assert_called_once_with(
        "MOCK-COM", cfg.BAUDRATE, timeout=0, write_timeout=0.05
    )
    return opener


@pytest.mark.parametrize("mode", ACTIVE_MODES)
@pytest.mark.parametrize("error", IO_ERRORS)
def test_write_failure_latches_and_disarms_every_mode(
    window: gui.MagneticDipoleControl,
    clock: list[float],
    monkeypatch: pytest.MonkeyPatch,
    mode: str,
    error: Exception,
) -> None:
    port = enter_mode(window, mode, clock)
    previous = list(window.last_sent_cmd)
    frame_index = len(port.frames)
    failed_write = Mock(side_effect=error)
    monkeypatch.setattr(port, "write", failed_write)
    advance(window, clock)
    assert window.serial_fault and window.mode == "SERIAL_FAULT"
    assert not window.stopping and port.closed and window.ser is None
    assert_modes_cleared(window)
    assert window.last_sent_cmd == previous and len(port.frames) == frame_index
    assert FAULT_MESSAGE in window.lbl_serial.text()
    assert FAULT_MESSAGE in window.status_lbl.text()
    spies = stop_tests.watch_active_branches(window, monkeypatch)
    executor = Mock(wraps=window.executor.step)
    monkeypatch.setattr(window.executor, "step", executor)
    advance(window, clock, 30)
    failed_write.assert_called_once()
    executor.assert_not_called()
    for spy in spies:
        spy.assert_not_called()
    assert window.mode == "SERIAL_FAULT" and window.last_sent_cmd == previous


@pytest.mark.parametrize("mode", ACTIVE_MODES)
def test_fault_rejects_all_mode_entries(
    window: gui.MagneticDipoleControl,
    clock: list[float],
    monkeypatch: pytest.MonkeyPatch,
    mode: str,
) -> None:
    configure_camera(window)
    advance(window, clock)
    _, previous = establish_fault(window)
    start = Mock()
    monkeypatch.setattr(ControlWorker, "start", start)
    solve = Mock(wraps=window._solve_motion_target)
    monkeypatch.setattr(window, "_solve_motion_target", solve)
    request_mode(window, mode)
    assert_modes_cleared(window)
    advance(window, clock, 30)
    start.assert_not_called()
    solve.assert_not_called()
    assert window.mode == "SERIAL_FAULT" and window.last_sent_cmd == previous


@pytest.mark.parametrize("mode", ACTIVE_MODES)
def test_reconnect_zero_success_unlocks_mode_entries(
    window: gui.MagneticDipoleControl,
    clock: list[float],
    monkeypatch: pytest.MonkeyPatch,
    mode: str,
) -> None:
    configure_camera(window)
    advance(window, clock)
    _, previous = establish_fault(window)

    class RecoverySerial(FaultSerial):
        def write(self, data: bytes) -> int:
            if not self.attempts:
                assert data == ZERO_FRAME
                assert window.serial_fault and window.mode == "SERIAL_FAULT"
                assert window.last_sent_cmd == previous
            return super().write(data)

    port = RecoverySerial()
    recovery_port(window, monkeypatch, port)
    assert port.frames == [ZERO_FRAME] and port.attempts == [ZERO_FRAME]
    assert not window.serial_fault and window.mode == "IDLE"
    assert window.last_sent_cmd == [0] * 6 and window.ser is port
    request_mode(window, mode)
    for _ in range(80):
        advance(window, clock)
        if any(window.last_sent_cmd):
            break
    assert any(window.last_sent_cmd), mode
    assert any(frame != ZERO_FRAME for frame in port.frames)
    if mode == "tracking":
        assert window.tracking and window.worker is not None


@pytest.mark.parametrize("error", IO_ERRORS)
def test_reconnect_zero_failure_keeps_fault_and_old_command(
    window: gui.MagneticDipoleControl,
    monkeypatch: pytest.MonkeyPatch,
    error: Exception,
) -> None:
    _, previous = establish_fault(window)
    port = FaultSerial()
    port.error = error
    recovery_port(window, monkeypatch, port)
    assert port.attempts == [ZERO_FRAME] and not port.frames and port.closed
    assert window.serial_fault and window.mode == "SERIAL_FAULT" and window.ser is None
    assert window.last_sent_cmd == previous
    assert not window.adc_rx_timer.isActive()
    assert_modes_cleared(window)


def test_initial_open_has_timeout_and_zero_first(
    window: gui.MagneticDipoleControl,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    assert not window.serial_fault
    port = FaultSerial()
    recovery_port(window, monkeypatch, port)
    assert port.frames == [ZERO_FRAME] and not window.serial_fault
    assert window.last_sent_cmd == [0] * 6 and window.mode == "IDLE"


def test_last_sent_updates_only_after_complete_success(
    window: gui.MagneticDipoleControl,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    port = MemorySerial()
    window.ser = port
    old = list(window.last_sent_cmd)

    def write(data: bytes) -> int:
        assert window.last_sent_cmd == old
        assert data == b"a0:+09,a1:-09,a2:+08,a3:-08,a4:+07,a5:-07\r\n"
        return port.write(data)

    proxy = Mock(wraps=port)
    proxy.write = write
    window.ser = proxy
    window.send_commands([9, -9, 8, -8, 7, -7])
    assert window.last_sent_cmd == [9, -9, 8, -8, 7, -7]
    assert len(port.frames) == 1 and not window.serial_fault


@pytest.mark.parametrize("count", [0, 42])
@pytest.mark.parametrize("reconnecting", [False, True])
def test_short_write_is_not_success(
    window: gui.MagneticDipoleControl,
    monkeypatch: pytest.MonkeyPatch,
    count: int,
    reconnecting: bool,
) -> None:
    _, previous = establish_fault(window)
    port = FaultSerial()
    port.short_write = count
    if reconnecting:
        recovery_port(window, monkeypatch, port)
    else:
        window.serial_fault = False
        window.ser = port
        window.send_commands([30] * 6)
    assert len(port.attempts) == 1 and not port.frames and port.closed
    assert window.serial_fault and window.mode == "SERIAL_FAULT"
    assert window.last_sent_cmd == previous and window.ser is None


@pytest.mark.parametrize(
    "operation",
    [
        "send_commands",
        "emergency_stop",
        "normal_stop",
        "apply_manual_force",
        "send_manual_currents",
    ],
)
def test_no_operation_clears_latched_fault(
    window: gui.MagneticDipoleControl,
    clock: list[float],
    operation: str,
) -> None:
    port, previous = establish_fault(window)
    attempts = list(port.attempts)
    if operation == "send_commands":
        window.send_commands([0] * 6)
    else:
        getattr(window, operation)()
    advance(window, clock, 30)
    assert window.serial_fault and window.mode == "SERIAL_FAULT"
    assert window.last_sent_cmd == previous and port.attempts == attempts
    assert_modes_cleared(window)


def test_disconnected_send_and_stop_do_not_invent_success(
    window: gui.MagneticDipoleControl,
) -> None:
    port = MemorySerial()
    window.ser = port
    window.send_commands([9] * 6)
    window.ser = None
    window.send_commands([30] * 6)
    assert not window._write_serial_frame(ZERO_FRAME, [0] * 6)
    window.emergency_stop()
    assert window.last_sent_cmd == [9] * 6 and not window.serial_fault
    assert_modes_cleared(window)


def test_idle_heartbeat_repeats_manual_hold_without_control(
    window: gui.MagneticDipoleControl,
    clock: list[float],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    configure_camera(window)
    advance(window, clock)
    port = FaultSerial()
    window.ser = port
    window.send_commands([9, -9, 8, -8, 7, -7])
    previous = list(window.last_sent_cmd)
    frame = port.frames[-1]
    start_index = len(port.frames)
    before_current = window.executor.I_est.copy()
    before_eso = window.z3.copy()
    spies = [
        Mock(wraps=window._solve_motion_target),
        Mock(wraps=window.executor.step),
        Mock(),
    ]
    monkeypatch.setattr(window, "_solve_motion_target", spies[0])
    monkeypatch.setattr(window.executor, "step", spies[1])
    monkeypatch.setattr(ControlWorker, "start", spies[2])
    slew = Mock(wraps=gui.apply_slew)
    monkeypatch.setattr(gui, "apply_slew", slew)
    advance(window, clock, 30)
    assert window.mode == "IDLE" and not window.serial_fault
    assert port.frames[start_index:] == [frame] * 30
    assert window.last_sent_cmd == previous and window.worker is None
    assert np.array_equal(window.executor.I_est, before_current)
    assert np.array_equal(window.z3, before_eso)
    for spy in spies:
        spy.assert_not_called()
    slew.assert_not_called()


@pytest.mark.parametrize("error", IO_ERRORS)
@pytest.mark.parametrize("emergency", [False, True])
def test_heartbeat_or_emergency_failure_latches(
    window: gui.MagneticDipoleControl,
    clock: list[float],
    error: Exception,
    emergency: bool,
) -> None:
    port = FaultSerial()
    window.ser = port
    window.send_commands([9] * 6)
    port.error = error
    if emergency:
        window.emergency_stop()
    else:
        advance(window, clock)
    assert window.serial_fault and window.mode == "SERIAL_FAULT"
    assert window.last_sent_cmd == [9] * 6 and port.closed
    advance(window, clock, 30)
    assert len(port.attempts) == 2 and window.last_sent_cmd == [9] * 6


def test_disconnect_failed_zero_preserves_fault(
    window: gui.MagneticDipoleControl,
) -> None:
    port = FaultSerial()
    window.ser = port
    window.send_commands([9] * 6)
    port.error = SerialTimeoutException("disconnect zero timed out")
    window.toggle_serial()
    assert window.serial_fault and window.mode == "SERIAL_FAULT"
    assert window.ser is None and port.closed and window.last_sent_cmd == [9] * 6
    assert FAULT_MESSAGE in window.lbl_serial.text()


def test_write_failure_stops_real_worker(
    window: gui.MagneticDipoleControl,
) -> None:
    configure_camera(window)
    port = FaultSerial()
    window.ser = port
    window.tick()
    request_mode(window, "tracking")
    worker = window.worker
    assert worker is not None and worker.is_alive()
    port.error = OSError("unplugged during tracking")
    window.send_commands([30] * 6)
    assert not worker.is_alive()
    assert_modes_cleared(window)
    assert window.serial_fault and window.mode == "SERIAL_FAULT"


def test_generic_serial_exception_latches(
    window: gui.MagneticDipoleControl,
) -> None:
    port = FaultSerial()
    window.ser = port
    window.send_commands([9] * 6)
    port.error = SerialException("device unavailable")
    window.send_commands([30] * 6)
    assert window.serial_fault and window.mode == "SERIAL_FAULT"
    assert window.last_sent_cmd == [9] * 6 and window.ser is None and port.closed
