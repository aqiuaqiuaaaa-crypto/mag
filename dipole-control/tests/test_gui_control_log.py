"""Synthetic camera/serial replay: logging must observe, never drive control."""

from __future__ import annotations

import csv
import json
import os
import sys
import time
from itertools import pairwise
from pathlib import Path
from statistics import median
from typing import Any

import cv2
import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import magnetic_dipole_pid as gui
import test_gui_curt_telemetry as telemetry_tests
from control_log import CONTROL_FIELDS, WORKER_FIELDS
from curt_telemetry import parse_adc_line
from PySide6.QtWidgets import QApplication
from test_gui_curt_telemetry import (
    MemorySerial,
    adc_frame,
    connect,
    pump_until,
)

# Reuse hardware-isolating fixtures, without changing the existing test module.
app = telemetry_tests.app
ports = telemetry_tests.ports
window = telemetry_tests.window


class SyntheticCamera:
    def __init__(self) -> None:
        self.index = 0
        self.fail = False
        self.released = False

    def read(self) -> tuple[bool, Any]:
        self.index += 1
        if self.fail:
            return False, None
        frame = np.zeros((180, 320, 3), np.uint8)
        cv2.circle(frame, (160 + min(self.index // 4, 30), 90), 8, (255, 255, 255), -1)
        return True, frame

    def release(self) -> None:
        self.released = True


def configure_camera(widget: gui.MagneticDipoleControl) -> SyntheticCamera:
    camera = SyntheticCamera()
    widget.cap = camera
    widget.frame_size = (320, 180)
    widget.video.frame_size = (320, 180)
    widget.chk_invert.setChecked(False)
    widget.spin_amin.setValue(20)
    widget.spin_rmin.setValue(1)
    return camera


def start_log(widget: gui.MagneticDipoleControl, path: Path) -> None:
    widget.edit_control_log.setText(str(path))
    widget.btn_control_log_start.click()
    assert widget.control_logger is not None


def rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf8") as file:
        return list(csv.DictReader(file))


def validate_types(records: list[dict[str, str]]) -> None:
    strings = {"mode", "estimator_mode", "eso_mode", "stale_status"}
    booleans = {
        "tracking",
        "frame_ok",
        "detected",
        "eso_updated",
        "executor_updated",
        "serial_connected",
        "low_field",
        "converged",
        "current_constraint_active",
        "field_constraint_active",
        "direction_constraint_active",
        "sparse_infeasible",
        "current_bound_ok",
        "slew_ok",
    }
    integers = {
        "tick",
        "dropped",
        "target_idx",
        "run",
        "seq",
        "frames_since",
        "max_cmd",
        "adc_frame_count",
        "adc_valid",
        "adc_running",
        "adc_error_flags",
        "horizon",
    }
    for row in records:
        for field, value in row.items():
            if not value or field in strings:
                continue
            if field in booleans:
                assert value in {"True", "False"}, (field, value)
            elif field in integers or field.startswith(("cmd_", "raw")):
                int(value)
            else:
                float(value)


def finish_log(widget: gui.MagneticDipoleControl) -> None:
    widget.btn_control_log_stop.click()
    assert widget.control_logger is not None and widget.control_logger.wait_closed(3)
    widget._refresh_control_log_status()


def test_real_qtimer_worker_adc_and_metadata(
    window: gui.MagneticDipoleControl,
    app: QApplication,
    ports: list[MemorySerial],
    tmp_path: Path,
) -> None:
    assert window.control_logger is None and not window.btn_control_log_stop.isEnabled()
    port = connect(window, ports)
    configure_camera(window)
    window.tick()
    window.path_px = [(160, 90), (240, 90), (280, 90)]
    window.start_tracking()
    start_tick = window._control_tick_count
    start_log(window, tmp_path / "live.csv")
    sink = window.control_logger
    assert sink is not None
    port.incoming.extend(adc_frame(77))
    window.timer.start(gui.cfg.TICK_MS)
    pump_until(app, lambda: window._control_tick_count >= start_tick + 16)
    window.timer.stop()
    stop_tick = window._control_tick_count
    finish_log(window)
    control = rows(sink.path)
    worker = rows(sink.worker_path)
    assert len(control) == stop_tick - start_tick
    assert [int(row["tick"]) for row in control] == list(
        range(start_tick + 1, stop_tick + 1)
    )
    assert sink.dropped == 0 and sink.error is None
    timestamps = [float(row["t_mono"]) for row in control]
    assert all(b > a for a, b in pairwise(timestamps))
    assert (
        tuple(control[0]) == CONTROL_FIELDS
        and worker
        and tuple(worker[0]) == WORKER_FIELDS
    )
    assert all(None not in row for row in control + worker)
    validate_types(control + worker)
    assert any(
        row["adc_frame_count"] == "77" and row["raw5"] == "606" for row in control
    )
    assert any(
        row["executor_updated"] == "True" and row["cmd_exec_a0"] for row in control
    )
    worker_targets = {(row["run"], row["seq"]): row for row in worker}
    for row in control:
        key = row["run"], row["seq"]
        if key in worker_targets:
            for field in (
                "F_target_x",
                "F_target_y",
                "F_target_z",
                "ref_x",
                "I_target_0",
            ):
                assert row[field] == worker_targets[key][field]
    assert all(len(frame) == 43 for frame in port.frames)
    metadata = json.loads(sink.metadata_path.read_text(encoding="utf8"))
    for field in (
        "git_commit",
        "gui",
        "config",
        "physics",
        "R_force_model_to_camera",
        "offset",
        "B_target",
        "path",
        "limits",
        "modes",
        "mpc_weights",
    ):
        assert field in metadata
    assert metadata["offset"]["adc_raw"] == [0] * 6
    assert "raw counts" in metadata["semantics"]["ADC"]
    assert metadata["gui"]["mpc_w_du"] == window.spin_mpc_w_du.value()
    written = len(control)
    window.tick()
    assert len(rows(sink.path)) == written
    assert "已停止" in window.lbl_control_log.text()


def test_output_bits_identical_with_logging_and_adc(
    window: gui.MagneticDipoleControl,
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    # Drive the actual worker synchronously every third tick for reproducible
    # inputs/timing. A separate test above exercises both real QTimers/threads.
    monkeypatch.setattr(gui.ControlWorker, "start", lambda self: None)
    plain = gui.MagneticDipoleControl()
    plain.timer.stop()
    clock = [1000.0]
    monkeypatch.setattr(gui.time, "time", lambda: clock[0])
    ports_pair = [MemorySerial(), MemorySerial()]
    cameras = []
    try:
        for widget, port in zip((plain, window), ports_pair):
            widget.ser = port
            cameras.append(configure_camera(widget))
            widget.last_time = clock[0]
            widget.tick()
            widget.path_px = [(160, 90), (240, 90), (280, 90)]
            widget.start_tracking()
        start_log(window, tmp_path / "equivalence.csv")
        sink = window.control_logger
        assert sink is not None
        before = window._control_tick_count
        auto_extra: list[float] = []
        auto_capture: list[float] = []
        for tick in range(60):
            clock[0] += 1 / gui.cfg.CONTROL_HZ
            if tick % 3 == 0:
                plain.worker._step()
                window.worker._step()
            for camera in cameras:
                camera.fail = 20 <= tick < 23
            # Deliberately different ADC data: it must have no control effect.
            window.adc_snapshot = parse_adc_line(adc_frame(tick + 1))
            window.adc_received_at = time.monotonic()
            elapsed = {}
            for widget in ((plain, window) if tick % 2 == 0 else (window, plain)):
                start = time.perf_counter()
                widget.tick()
                elapsed[widget] = (time.perf_counter() - start) * 1e3
            auto_extra.append(elapsed[window] - elapsed[plain])
            auto_capture.append(window.control_log_capture_ms)
            assert ports_pair[0].frames == ports_pair[1].frames
            for attr in ("state_pos_mm", "state_vel_mm", "z3", "last_F_actual"):
                assert np.array_equal(getattr(plain, attr), getattr(window, attr))
            assert np.array_equal(plain.executor.I_est, window.executor.I_est)
            assert plain.executor.seq == window.executor.seq
            assert plain.executor.frames_since == window.executor.frames_since
            assert np.array_equal(
                plain.shared.get_I_target()[0], window.shared.get_I_target()[0]
            )
        finish_log(window)
        control = rows(sink.path)
        assert len(control) == window._control_tick_count - before == 60
        assert any(
            row["frame_ok"] == "False" and row["meas_x_mm"] == "" for row in control
        )
        assert any(row["eso_updated"] == "True" and row["u_eso_x"] for row in control)
        assert all(row["raw0"] == "101" for row in control)
        assert len(rows(sink.worker_path)) == 20
        assert median(auto_extra) < 1.0 and median(auto_capture) < 1.0
        print(
            f"AUTO_TRACK: capture_median={median(auto_capture):.4f} ms paired_extra={median(auto_extra):.4f} ms n=60"
        )
    finally:
        plain.close()


@pytest.mark.parametrize("estimator", ["RAW", "EMA", "KALMAN"])
def test_manual_idle_and_estimator_fields(
    window: gui.MagneticDipoleControl,
    tmp_path: Path,
    estimator: str,
) -> None:
    configure_camera(window)
    window.combo_estimator.setCurrentText(estimator)
    window.ser = MemorySerial()
    start_log(window, tmp_path / f"{estimator}.csv")
    window.tick()
    window.man_currents = [99, -99, 0, 18, 30, -30]
    window.chk_cur_live.setChecked(True)
    for _ in range(6):
        window.tick()
    finish_log(window)
    sink = window.control_logger
    assert sink is not None
    control = rows(sink.path)
    assert len(control) == 7 and control[0]["mode"] == "IDLE"
    assert all(
        row["executor_updated"] == "False" and row["cmd_exec_a0"] == ""
        for row in control
    )
    assert all(row["estimator_mode"] == estimator for row in control)
    assert bool(control[-1]["kf_x"]) == (estimator == "KALMAN")
    assert int(control[-1]["cmd_sent_a0"]) == window._current_cmd_limit()
    assert len(window.ser.frames) == 7  # Initial IDLE heartbeat + six active ticks.


def test_open_failure_leaves_control_running(
    window: gui.MagneticDipoleControl,
    app: QApplication,
    tmp_path: Path,
) -> None:
    configure_camera(window)
    port = MemorySerial()
    window.ser = port
    window.man_currents = [40, -40, 30, -30, 20, -20]
    window.chk_cur_live.setChecked(True)
    start_log(window, tmp_path / "missing" / "failed.csv")
    sink = window.control_logger
    assert sink is not None
    pump_until(app, lambda: sink.finished.is_set())
    window._refresh_control_log_status()
    for _ in range(10):
        window.tick()
    assert len(port.frames) == 10 and window.ser is port
    assert (
        window.mode == "MANUAL_CURRENT" and window.timer.interval() == gui.cfg.TICK_MS
    )
    assert (
        "日志错误" in window.lbl_control_log.text()
        and window.btn_control_log_start.isEnabled()
    )


def test_mid_stream_failure_leaves_tracking_and_worker_running(
    window: gui.MagneticDipoleControl,
    app: QApplication,
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    import control_log

    real_writer = control_log.csv.writer

    class BrokenWriter:
        def __init__(self, file: Any) -> None:
            self.writer = real_writer(file)
            self.count = 0

        def writerow(self, row: Any) -> None:
            self.count += 1
            if self.count > 2:
                raise OSError("disk full during experiment")
            self.writer.writerow(row)

    monkeypatch.setattr(control_log.csv, "writer", BrokenWriter)
    configure_camera(window)
    window.ser = MemorySerial()
    window.tick()
    window.path_px = [(160, 90), (260, 90)]
    window.start_tracking()
    start_log(window, tmp_path / "full.csv")
    sink = window.control_logger
    assert sink is not None
    window.timer.start(gui.cfg.TICK_MS)
    pump_until(app, lambda: sink.finished.is_set())
    count = window._control_tick_count
    pump_until(app, lambda: window._control_tick_count >= count + 6)
    window.timer.stop()
    window._refresh_control_log_status()
    assert window.tracking and window.worker.is_alive()
    assert window.shared.get_solver_error() is None and window.ser is not None
    assert "disk full" in window.lbl_control_log.text()


def test_restart_and_new_tracking_run(
    window: gui.MagneticDipoleControl,
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    monkeypatch.setattr(gui.ControlWorker, "start", lambda self: None)
    configure_camera(window)
    start_log(window, tmp_path / "first.csv")
    window.tick()
    window.path_px = [(160, 90), (250, 90)]
    window.start_tracking()
    window.worker._step()
    window.tick()
    first_run = window._control_log_run
    window.normal_stop()
    window.start_tracking()
    window.worker._step()
    window.tick()
    assert window._control_log_run == first_run + 1
    finish_log(window)
    assert window.control_logger is not None
    control = rows(window.control_logger.path)
    assert control[1]["seq"] == control[2]["seq"] == "1"
    assert control[1]["run"] != control[2]["run"]
    start_log(window, tmp_path / "second.csv")
    window.tick()
    finish_log(window)
    assert (
        window.control_logger is not None and len(rows(window.control_logger.path)) == 1
    )


def test_logging_capture_overhead(
    window: gui.MagneticDipoleControl,
    tmp_path: Path,
) -> None:
    plain = gui.MagneticDipoleControl()
    plain.timer.stop()
    try:
        for widget in (plain, window):
            configure_camera(widget)
            widget.ser = MemorySerial()
            widget.chk_cur_live.setChecked(True)
            widget.man_currents = [12, -12, 24, -24, 36, -36]
            for _ in range(20):
                widget.tick()  # Warm up OpenCV, Qt and model caches.
        start_log(window, tmp_path / "performance.csv")
        overhead: list[float] = []
        paired_extra: list[float] = []
        elapsed_off: list[float] = []
        elapsed_on: list[float] = []
        for index in range(240):
            times = {}
            for widget in ((plain, window) if index % 2 == 0 else (window, plain)):
                start = time.perf_counter()
                widget.tick()
                times[widget] = (time.perf_counter() - start) * 1e3
            overhead.append(window.control_log_capture_ms)
            elapsed_off.append(times[plain])
            elapsed_on.append(times[window])
            paired_extra.append(times[window] - times[plain])
        finish_log(window)
        measured = median(overhead)
        extra = median(paired_extra)
        assert measured < 1.0, f"median capture/queue overhead {measured:.3f} ms"
        assert extra < 1.0, f"paired full-tick median overhead {extra:.3f} ms"
        print(
            f"capture overhead: median={measured:.4f} ms max={max(overhead):.4f} ms n=240"
        )
        print(
            f"full tick: off={median(elapsed_off):.4f} ms on={median(elapsed_on):.4f} ms paired_extra={extra:.4f} ms"
        )
        assert plain.ser.frames == window.ser.frames
        assert window.control_logger is not None and window.control_logger.dropped == 0
    finally:
        plain.close()


def test_gui_snapshot_error_is_diagnostics_only(
    window: gui.MagneticDipoleControl,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    start_log(window, tmp_path / "snapshot-error.csv")
    assert window.control_logger is not None

    def bad_enqueue(*args: object) -> None:
        raise RuntimeError("snapshot fault")

    monkeypatch.setattr(window.control_logger, "enqueue", bad_enqueue)
    window.tick()
    assert window.control_logger is not None and window.control_logger.error
    assert window.mode == "IDLE" and window.shared.get_solver_error() is None
    finish_log(window)


def test_executed_sequence_keeps_its_target_when_worker_publishes_again(
    window: gui.MagneticDipoleControl,
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    monkeypatch.setattr(gui.ControlWorker, "start", lambda self: None)
    configure_camera(window)
    window.tick()
    window.path_px = [(160, 90), (250, 90)]
    window.start_tracking()
    start_log(window, tmp_path / "seq-race.csv")
    window.worker._step()
    original_target = window.shared.get_I_target()[3]["F_target"].copy()
    original_step = window.executor.step

    def publish_after_step(*args: Any, **kwargs: Any) -> Any:
        diag = original_step(*args, **kwargs)
        window.shared.set_I_target(np.zeros(6), None, [999, -999], 0, 0, 0)
        return diag

    monkeypatch.setattr(window.executor, "step", publish_after_step)
    window.tick()
    finish_log(window)
    assert window.shared.get_I_target()[1] == 2 and window.executor.seq == 1
    assert window.control_logger is not None
    row = rows(window.control_logger.path)[0]
    assert row["seq"] == "1"
    assert float(row["F_target_x"]) == original_target[0]
    assert float(row["F_target_y"]) == original_target[1]
