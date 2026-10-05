"""Actual GUI pause/normal-stop and CSV evidence, without real hardware."""

from __future__ import annotations

import sys
from pathlib import Path
from unittest.mock import Mock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import magnetic_dipole_pid as gui
import test_gui_curt_telemetry as telemetry_tests
from multirate import ControlWorker, CurrentExecutor, SharedState
from test_freshness import FakeClock
from test_gui_control_log import configure_camera, finish_log, rows, start_log
from test_gui_curt_telemetry import MemorySerial

app = telemetry_tests.app
ports = telemetry_tests.ports
window = telemetry_tests.window


def start_tracking(
    widget: gui.MagneticDipoleControl, monkeypatch: pytest.MonkeyPatch
) -> tuple[FakeClock, MemorySerial]:
    clock = FakeClock()
    widget.control_clock = clock
    widget.shared = SharedState(clock=clock)
    widget.executor = CurrentExecutor(clock=clock)
    monkeypatch.setattr(ControlWorker, "start", lambda self: None)
    port = MemorySerial()
    widget.ser = port
    configure_camera(widget)
    widget.tick()
    for _ in range(4):
        widget.send_commands([36] * 6)
    widget.path_px = [(160, 90), (240, 90), (280, 90)]
    widget.start_tracking()
    assert widget.worker is not None
    widget.worker._step()
    widget.tick()
    assert any(widget.last_sent_cmd) and widget.tracking
    return clock, port


def test_stalled_worker_holds_then_normal_stop_and_logs(
    window: gui.MagneticDipoleControl, monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    clock, port = start_tracking(window, monkeypatch)
    start_log(window, tmp_path / "stall.csv")
    normal = Mock(wraps=window.normal_stop)
    emergency = Mock(wraps=window.emergency_stop)
    monkeypatch.setattr(window, "normal_stop", normal)
    monkeypatch.setattr(window, "emergency_stop", emergency)
    held = list(window.last_sent_cmd)
    assert max(map(abs, held)) > 9
    seq, frames = window.executor.seq, window.executor.frames_since
    for age in (0.35, 0.60, 1.00):
        clock.now = age
        window.tick()
        assert window.last_sent_cmd == held and window.tracking and not window.stopping
        assert window.executor.seq == seq and window.executor.frames_since == frames
        assert window.stale_status == "STALE_TARGET"
        assert "STALE_TARGET" in window.status_lbl.text()
        assert port.frames[-1] == gui.build_command(held)[0].encode("ascii")
    clock.now = 1.10
    window.tick()
    normal.assert_called_once()
    emergency.assert_not_called()
    assert window.stopping and not window.tracking and window.worker is None
    assert (
        window.last_sent_cmd == held and window.stale_status == "STALE_TARGET_TIMEOUT"
    )
    for _ in range(30):
        previous = list(window.last_sent_cmd)
        clock.now += 1 / 30
        window.tick()
        assert all(
            abs(b) <= abs(a) and abs(b - a) <= 9
            for a, b in zip(previous, window.last_sent_cmd)
        )
    assert window.last_sent_cmd == [0] * 6 and window.mode == "IDLE"
    finish_log(window)
    assert window.control_logger is not None
    records = rows(window.control_logger.path)
    assert [r["stale_status"] for r in records[:4]] == ["STALE_TARGET"] * 3 + [
        "STALE_TARGET_TIMEOUT"
    ]
    assert float(records[0]["target_age_ms"]) == 350
    assert float(records[3]["target_age_ms"]) == 1100


def test_vision_loss_stops_publication_and_can_recover(
    window: gui.MagneticDipoleControl, monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    clock, _port = start_tracking(window, monkeypatch)
    assert window.worker is not None
    worker = window.worker
    solve = Mock(wraps=worker.solver.solve_field_force_pseudoinverse)
    monkeypatch.setattr(worker.solver, "solve_field_force_pseudoinverse", solve)
    start_log(window, tmp_path / "vision.csv")
    camera = window.cap
    camera.fail = True
    seq = window.shared.get_I_target()[1]
    for age in (0.10, 0.20, 0.25):
        clock.now = age
        window.tick()
        worker._step()
    # A fresh 0.10s input may publish; older input may never do so.
    assert solve.call_count == 1 and window.shared.get_I_target()[1] == seq + 1
    assert window.shared.get_freshness()["worker_status"] == "STALE_INPUT"
    assert window.stale_status == "STALE_INPUT" and window.tracking
    assert window.shared.get_solver_error() is None
    camera.fail = False
    clock.now = 0.26
    window.tick()  # new vision updates shared input without stopping
    worker._step()
    clock.now = 0.27
    window.tick()
    assert window.shared.get_I_target()[1] == seq + 2
    assert window.stale_status == "FRESH"
    assert window.tracking and not window.stopping
    finish_log(window)
    assert window.control_logger is not None
    assert "STALE_INPUT" in {
        r["stale_status"] for r in rows(window.control_logger.path)
    }
    worker_rows = rows(window.control_logger.worker_path)
    stale_rows = [r for r in worker_rows if r["stale_status"] == "STALE_INPUT"]
    assert len(stale_rows) == 2 and all(r["I_target_0"] == "" for r in stale_rows)
    assert all(int(r["seq"]) == seq + 1 for r in stale_rows)


def test_vision_loss_persistent_timeout_and_monotonic_stop(
    window: gui.MagneticDipoleControl, monkeypatch: pytest.MonkeyPatch
) -> None:
    clock, _port = start_tracking(window, monkeypatch)
    window.cap.fail = True
    clock.now = 0.01
    window.tick()
    clock.now = 0.20
    assert window.worker is not None
    window.worker._step()
    window.tick()
    assert window.tracking
    clock.now = 0.35
    window.tick()
    held = window.last_sent_cmd.copy()
    assert window.stale_status == "STALE_TARGET" and any(held)
    clock.now = 1.0
    window.tick()
    assert window.tracking  # target timeout uses strictly > 1.0
    clock.now = 1.10
    window.tick()
    assert window.stopping and not window.tracking
    assert window.stale_status == "STALE_TARGET_TIMEOUT"
    assert window.last_sent_cmd == held
    assert window.shared.get_solver_error() is None


def test_fresh_csv_ages_use_injected_clock(
    window: gui.MagneticDipoleControl, monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    clock, _port = start_tracking(window, monkeypatch)
    start_log(window, tmp_path / "fresh.csv")
    for _ in range(6):
        clock.now += 1 / 30
        window.tick()
        assert window.worker is not None
        window.worker._step()
    finish_log(window)
    assert window.control_logger is not None
    for row in rows(window.control_logger.path):
        assert row["stale_status"] == "FRESH"
        assert float(row["kalman_age_ms"]) == 0.0
        assert 0 <= float(row["target_age_ms"]) <= 34


def test_existing_one_second_vision_stop_is_preserved(
    window: gui.MagneticDipoleControl, monkeypatch: pytest.MonkeyPatch
) -> None:
    clock, _port = start_tracking(window, monkeypatch)
    window.cap.fail = True
    window.tick()  # lost_since = monotonic zero
    clock.now = 0.10
    assert window.worker is not None
    window.worker._step()  # last permissible publication at age 0.10
    clock.now = 1.00
    window.tick()
    assert window.tracking and window.stale_status == "STALE_TARGET"
    clock.now = 1.01
    window.tick()
    assert window.stopping and not window.tracking
    # Input loss exceeded 1s first; target itself is still only 0.91s old.
    assert window.stale_status == "STALE_TARGET"
