"""Exercise GUI failures with isolated files and synthetic hardware only."""

from __future__ import annotations

import csv
import json
import os
import sys
from collections.abc import Iterator
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import bead_sim_gui as sim_gui
import magnetic_dipole_pid as gui
from PySide6.QtWidgets import QApplication


@pytest.fixture(scope="session")
def app() -> QApplication:
    """Reuse Qt's single application for both GUIs."""
    instance = QApplication.instance()
    if instance is None:
        instance = QApplication([])
    assert isinstance(instance, QApplication)
    return instance


@pytest.fixture
def messages(monkeypatch: pytest.MonkeyPatch) -> list[str]:
    """Record modal messages without blocking the offscreen event loop."""
    found: list[str] = []

    def record(parent: object, title: str, message: str) -> None:
        found.append(message)

    monkeypatch.setattr(gui.QMessageBox, "warning", record)
    monkeypatch.setattr(gui.QMessageBox, "information", record)
    return found


@pytest.fixture
def window(
    app: QApplication, monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> Iterator[gui.MagneticDipoleControl]:
    """Never open a real port, camera, or the user's settings file."""
    monkeypatch.setattr(gui, "SETTINGS_FILE", str(tmp_path / "settings.json"))
    monkeypatch.setattr(gui, "list_ports", SimpleNamespace(comports=list))

    def forbid_hardware(*args: object, **kwargs: object) -> None:
        raise AssertionError("Unexpected hardware access")

    monkeypatch.setattr(gui, "_serial", SimpleNamespace(Serial=forbid_hardware))
    monkeypatch.setattr(gui.cv2, "VideoCapture", forbid_hardware)
    widget = gui.MagneticDipoleControl()
    widget.timer.stop()
    yield widget
    widget.close()
    app.processEvents()


@pytest.fixture
def simulator(app: QApplication) -> Iterator[sim_gui.BeadSimGUI]:
    """Construct the real simulator GUI with its existing physical model."""
    widget = sim_gui.BeadSimGUI()
    widget.timer.stop()
    yield widget
    widget.close()
    app.processEvents()


class FailingSerial:
    """An in-memory port with independent write and close failures."""

    def __init__(
        self, write_error: Exception, close_error: Exception | None = None
    ) -> None:
        self.write_error = write_error
        self.close_error = close_error
        self.close_attempted = False

    def write(self, data: bytes) -> int:
        assert len(data) == gui.cfg.SERIAL_FRAME_BYTES
        raise self.write_error

    def close(self) -> None:
        self.close_attempted = True
        if self.close_error is not None:
            raise self.close_error


@pytest.mark.parametrize("close_error", [None, OSError("close failed")])
@pytest.mark.parametrize("write_error", [OSError("TX failed"), RuntimeError("TX bug")])
def test_send_failure_disconnects_and_logs(
    window: gui.MagneticDipoleControl,
    write_error: Exception,
    close_error: Exception | None,
    caplog: pytest.LogCaptureFixture,
) -> None:
    """TX errors retain the existing clamp/slew fallback and release the port."""
    port = FailingSerial(write_error, close_error)
    window.ser = port
    window.send_commands([99, -99, 0, 0, 0, 0])
    assert port.close_attempted and window.ser is None
    assert window.last_sent_cmd == [9, -9, 0, 0, 0, 0]
    assert window.btn_serial.text() == "连接"
    assert "串口错误" in window.lbl_serial.text()
    assert any(record.exc_info for record in caplog.records)
    if close_error is not None:
        assert "关闭失败" in window.lbl_serial.text()


def test_unexpected_close_error_still_clears_port(
    window: gui.MagneticDipoleControl,
) -> None:
    """A close implementation bug propagates after the connection is cleared."""
    port = FailingSerial(OSError("TX failed"), RuntimeError("close bug"))
    window.ser = port
    with pytest.raises(RuntimeError, match="close bug"):
        window.send_commands([0] * 6)
    assert window.ser is None and window.btn_serial.text() == "连接"


@pytest.mark.parametrize("error", [OSError("unplugged"), RuntimeError("write bug")])
def test_emergency_failure_stops_real_worker(
    window: gui.MagneticDipoleControl,
    error: Exception,
    caplog: pytest.LogCaptureFixture,
) -> None:
    """A failed zero frame cannot prevent the existing local emergency stop."""
    assert not window.shared.stopped()
    worker = gui.ControlWorker(window.shared, window.solver)
    window.worker = worker
    worker.start()
    window.tracking = True
    window.stopping = True
    window.last_sent_cmd = [50] * 6
    window.ser = FailingSerial(error)
    window.emergency_stop()
    assert not worker.is_alive() and window.worker is None
    assert window.shared.stopped()
    assert not window.tracking and not window.stopping
    assert window.last_sent_cmd == [0] * 6 and window.mode == "IDLE"
    assert any(record.exc_info for record in caplog.records)
    window.ser = None


@pytest.mark.parametrize("error", [OSError("busy"), ValueError("bad port")])
def test_connect_expected_failure_is_reported(
    window: gui.MagneticDipoleControl,
    monkeypatch: pytest.MonkeyPatch,
    messages: list[str],
    error: Exception,
) -> None:
    """Expected port-opening errors leave reception stopped and no live port."""

    def fail(*args: object, **kwargs: object) -> None:
        raise error

    window.combo_port.addItem("MOCK")
    monkeypatch.setattr(gui, "_serial", SimpleNamespace(Serial=fail))
    window.toggle_serial()
    assert window.ser is None and not window.adc_rx_timer.isActive()
    assert messages and "打开失败" in messages[-1]


@pytest.mark.parametrize("error", [OSError("first write failed"), RuntimeError("bug")])
def test_connect_initial_write_failure_cleans_up(
    window: gui.MagneticDipoleControl,
    monkeypatch: pytest.MonkeyPatch,
    messages: list[str],
    error: Exception,
) -> None:
    """The initial zero frame must succeed before a connection becomes active."""
    port = FailingSerial(error)
    window.combo_port.addItem("MOCK")
    monkeypatch.setattr(gui, "_serial", SimpleNamespace(Serial=lambda *a, **k: port))
    if isinstance(error, OSError):
        window.toggle_serial()
    else:
        with pytest.raises(RuntimeError, match="bug"):
            window.toggle_serial()
    assert port.close_attempted and window.ser is None
    assert not window.adc_rx_timer.isActive()


@pytest.mark.parametrize("payload", [b"{", b"[]", b"\xff", b'{"n_coils":null}'])
def test_invalid_model_disables_control(
    window: gui.MagneticDipoleControl, tmp_path: Path, payload: bytes
) -> None:
    """Unreadable or invalid model data cannot leave the old model enabled."""
    model_path = tmp_path / "model.json"
    model_path.write_bytes(payload)
    window._phys_params["MODEL"] = str(model_path)
    window._reload_solver()
    assert not window.model_ok and window.solver is None
    assert not window.btn_track.isEnabled() and not window.btn_force_once.isEnabled()


def test_model_programming_error_is_visible_and_disables_control(
    window: gui.MagneticDipoleControl,
    monkeypatch: pytest.MonkeyPatch,
    caplog: pytest.LogCaptureFixture,
) -> None:
    """Unexpected loader bugs propagate after disabling model-dependent actions."""

    def fail(*args: object, **kwargs: object) -> None:
        raise RuntimeError("loader bug")

    monkeypatch.setattr(gui.DipoleSolver, "from_json", fail)
    with pytest.raises(RuntimeError, match="loader bug"):
        window._reload_solver()
    assert not window.model_ok and window.solver is None
    assert not window.btn_track.isEnabled()
    assert any(record.exc_info for record in caplog.records)


@pytest.mark.parametrize(
    "error", [ValueError("invalid target"), RuntimeError("solver bug")]
)
def test_manual_solver_failure_enters_safe_stop(
    window: gui.MagneticDipoleControl,
    monkeypatch: pytest.MonkeyPatch,
    error: Exception,
    caplog: pytest.LogCaptureFixture,
) -> None:
    """All manual solve failures stop live output and preserve traceback evidence."""

    def fail(*args: object) -> None:
        raise error

    monkeypatch.setattr(window, "_solve_motion_target", fail)
    window.chk_force_live.setChecked(True)
    window._solve_and_send(np.ones(3))
    assert window.mode == "STOPPING" and window.stopping
    assert not window.chk_force_live.isChecked() and window.shared.stopped()
    assert "解算失败" in window.lbl_force_out.text()
    assert any(record.exc_info for record in caplog.records)


def test_diagnostic_bug_is_logged_without_blocking_send(
    window: gui.MagneticDipoleControl,
    monkeypatch: pytest.MonkeyPatch,
    caplog: pytest.LogCaptureFixture,
) -> None:
    """Visualization errors cannot interrupt the independent safety send path."""

    def fail(*args: object) -> None:
        raise KeyError("unexpected model output")

    monkeypatch.setattr(window.solver, "forward_model", fail)
    window.send_commands([0] * 6)
    assert np.array_equal(window.current_B_T, np.zeros(3))
    assert np.array_equal(window.current_F_N, np.zeros(3))
    assert any(record.exc_info for record in caplog.records)


@pytest.mark.parametrize("method", ["save_path_snapshot", "start_video_recording"])
@pytest.mark.parametrize(
    "error", [OSError("disk failed"), gui.cv2.error("codec failed")]
)
def test_media_expected_errors_show_message(
    window: gui.MagneticDipoleControl,
    monkeypatch: pytest.MonkeyPatch,
    messages: list[str],
    method: str,
    error: Exception,
) -> None:
    """File and codec failures keep their GUI warning fallback."""
    window.frame = window.last_render_bgr = np.zeros((16, 16, 3), dtype=np.uint8)
    monkeypatch.setattr(gui.QFileDialog, "getSaveFileName", lambda *a: ("mock.png", ""))

    def fail(*args: object) -> None:
        raise error

    helper = (
        "_save_path_snapshot_to"
        if method == "save_path_snapshot"
        else "_start_video_recording_to"
    )
    monkeypatch.setattr(window, helper, fail)
    getattr(window, method)()
    assert messages and "失败" in messages[-1]
    assert window.video_writer is None


@pytest.mark.parametrize("method", ["save_path_snapshot", "start_video_recording"])
def test_media_programming_errors_propagate(
    window: gui.MagneticDipoleControl,
    monkeypatch: pytest.MonkeyPatch,
    messages: list[str],
    method: str,
) -> None:
    """Unexpected media helper bugs are no longer mislabeled as save failures."""
    window.frame = window.last_render_bgr = np.zeros((16, 16, 3), dtype=np.uint8)
    monkeypatch.setattr(gui.QFileDialog, "getSaveFileName", lambda *a: ("mock.png", ""))

    def fail(*args: object) -> None:
        raise RuntimeError("media bug")

    helper = (
        "_save_path_snapshot_to"
        if method == "save_path_snapshot"
        else "_start_video_recording_to"
    )
    monkeypatch.setattr(window, helper, fail)
    with pytest.raises(RuntimeError, match="media bug"):
        getattr(window, method)()


@pytest.mark.parametrize("payload", [b"{", b"[]", b"null", b"\xff"])
def test_invalid_settings_keep_existing_defaults(
    window: gui.MagneticDipoleControl,
    payload: bytes,
    capsys: pytest.CaptureFixture[str],
) -> None:
    """Malformed syntax, encoding, and root types all use the default settings."""
    before = window.spin_cmd_max.value()
    Path(gui.SETTINGS_FILE).write_bytes(payload)
    window.load_settings()
    assert window.spin_cmd_max.value() == before
    assert "使用默认值" in capsys.readouterr().out


@pytest.mark.parametrize("value", ["bad", None, [], float("inf")])
def test_invalid_widget_setting_logs_and_keeps_value(
    window: gui.MagneticDipoleControl, value: object, caplog: pytest.LogCaptureFixture
) -> None:
    """Bad external values do not erase the old integer widget setting."""
    before = window.spin_cmd_max.value()
    window._widget_set(window.spin_cmd_max, value)
    assert window.spin_cmd_max.value() == before
    assert "无效控件设置" in caplog.text


def test_invalid_calibration_settings_reset_transform(
    window: gui.MagneticDipoleControl, caplog: pytest.LogCaptureFixture
) -> None:
    """Invalid numeric calibration data restores the safe identity transform."""
    Path(gui.SETTINGS_FILE).write_text(
        json.dumps({"force_model_to_camera": [["bad", 0], [0, 1]]}), encoding="utf-8"
    )
    window.load_settings()
    assert np.array_equal(window.force_model_to_camera, np.eye(2))
    assert not window.force_frame_calibrated and "方向标定设置无效" in caplog.text


def test_settings_save_io_failure_does_not_prevent_stop(
    window: gui.MagneticDipoleControl,
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    """A save failure during close cannot suppress the mandatory emergency stop."""
    monkeypatch.setattr(gui, "SETTINGS_FILE", str(tmp_path))
    window.tracking = True
    window.last_sent_cmd = [50] * 6
    window.close()
    assert window.last_sent_cmd == [0] * 6 and window.shared.stopped()
    assert "参数保存失败" in capsys.readouterr().out


def test_settings_serialization_bug_does_not_prevent_stop(
    window: gui.MagneticDipoleControl,
    monkeypatch: pytest.MonkeyPatch,
    caplog: pytest.LogCaptureFixture,
) -> None:
    """Unexpected serialization bugs are logged before close proceeds to safety."""

    def fail(*args: object, **kwargs: object) -> None:
        raise RuntimeError("serialization bug")

    monkeypatch.setattr(gui.json, "dump", fail)
    window.last_sent_cmd = [50] * 6
    window.close()
    assert window.last_sent_cmd == [0] * 6 and window.shared.stopped()
    assert any(record.exc_info for record in caplog.records)


@pytest.mark.parametrize(
    "payload",
    [None, b"", b"\xff", b"x,y\n0,0\n", b"t,x,y\n0,1\n", b"t,x,y\nwrong,0,0\n"],
)
def test_fit_csv_expected_failure_is_reported(
    simulator: sim_gui.BeadSimGUI,
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    messages: list[str],
    payload: bytes | None,
) -> None:
    """Missing files and malformed CSV input retain the fitter's warning fallback."""
    path = tmp_path / "experiment.csv"
    if payload is not None:
        path.write_bytes(payload)
    monkeypatch.setattr(
        sim_gui.QFileDialog, "getOpenFileName", lambda *a: (str(path), "")
    )
    simulator.run_fit()
    assert messages and "读取失败" in messages[-1]


@pytest.mark.parametrize("error", [csv.Error("bad CSV"), RuntimeError("parser bug")])
def test_fit_csv_distinguishes_parser_failure_from_bug(
    simulator: sim_gui.BeadSimGUI,
    monkeypatch: pytest.MonkeyPatch,
    messages: list[str],
    error: Exception,
) -> None:
    """CSV format errors are expected, while unrelated parser bugs propagate."""
    monkeypatch.setattr(
        sim_gui.QFileDialog, "getOpenFileName", lambda *a: ("mock.csv", "")
    )

    def fail(*args: object) -> None:
        raise error

    monkeypatch.setattr(sim_gui.ExperimentFitter, "load_experiment_csv", fail)
    if isinstance(error, csv.Error):
        simulator.run_fit()
        assert "读取失败" in messages[-1]
    else:
        with pytest.raises(RuntimeError, match="parser bug"):
            simulator.run_fit()


def test_parameter_sweep_exports_csv_and_png_without_name_error(
    simulator: sim_gui.BeadSimGUI,
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    messages: list[str],
) -> None:
    """Exercise the real export/plot path while replacing only the long simulation."""
    rows = [
        {
            "diameter_mm": 1.0,
            "Br_T": 1.2,
            "viscosity_mPas": 1000.0,
            "mu": 0.1,
            "v_ss_30uN_mm_s": 1.5,
            "F_start_uN": 2.0,
        }
    ]
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(sim_gui.ParameterSweep, "run_ofat", lambda: rows)
    simulator.run_sweep()
    with (tmp_path / "bead_sweep.csv").open(encoding="utf-8", newline="") as stream:
        saved = list(csv.DictReader(stream))
    assert saved == [{key: str(value) for key, value in rows[0].items()}]
    assert (tmp_path / "bead_sweep.png").read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    assert QApplication.overrideCursor() is None
    assert messages and "完成 1 组" in messages[-1]


def test_parameter_sweep_failure_restores_cursor(
    simulator: sim_gui.BeadSimGUI,
    monkeypatch: pytest.MonkeyPatch,
    messages: list[str],
) -> None:
    """An export failure keeps its original propagation and cursor cleanup."""
    monkeypatch.setattr(sim_gui.ParameterSweep, "run_ofat", list)

    def fail(*args: object) -> None:
        raise OSError("sweep disk failure")

    monkeypatch.setattr(sim_gui.ParameterSweep, "save_csv", fail)
    with pytest.raises(OSError, match="sweep disk failure"):
        simulator.run_sweep()
    assert QApplication.overrideCursor() is None and not messages
