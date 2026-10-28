"""Headless control lifecycle and diagnostics after measurement removal."""

from pathlib import Path

import gui_test_support as support
import magnetic_dipole_pid as gui
from gui_test_support import MemorySerial, connect, pump_until
from PySide6.QtWidgets import QApplication

app = support.app
ports = support.ports
window = support.window


def test_control_log_timer_remains_without_measurement_page(
    window: gui.MagneticDipoleControl, app: QApplication, tmp_path: Path
) -> None:
    assert all("CURT" not in window.tabs.tabText(i) for i in range(window.tabs.count()))
    assert not any(name.startswith("adc_") for name in vars(window))
    timer = window.control_log_status_timer
    assert timer.isActive() and timer.interval() == 100
    window.edit_control_log.setText(str(tmp_path / "timer.csv"))
    window.start_control_log()
    sink = window.control_logger
    assert sink is not None and sink.wait_closed(3)
    pump_until(app, lambda: "已停止" in window.lbl_control_log.text())
    assert window.btn_control_log_start.isEnabled()


def test_idle_echo_is_not_parsed_and_close_stops_control_first(
    window: gui.MagneticDipoleControl, ports: list[MemorySerial]
) -> None:
    serial = connect(window, ports)
    serial.incoming.extend(b"304560759010")
    for _ in range(5):
        window.tick()
    assert not serial.read_sizes
    assert all(len(frame) == 43 for frame in serial.frames)
    window.close()
    assert serial.closed and window.worker is None
    assert (
        not window.timer.isActive() and not window.control_log_status_timer.isActive()
    )
    assert serial.frames[-1] == b"a0:+00,a1:+00,a2:+00,a3:+00,a4:+00,a5:+00\r\n"
