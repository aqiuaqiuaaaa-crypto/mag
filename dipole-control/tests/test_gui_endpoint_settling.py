"""Actual GUI lifecycle at the endpoint with fake camera/serial and clock."""

from __future__ import annotations

import sys
from pathlib import Path
from unittest.mock import Mock

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import config as cfg
import gui_test_support as gui_tests
from control_mode_simulation import run_simulation
from multirate import ControlWorker
from test_freshness import FakeSolver
from test_gui_freshness import start_tracking

app = gui_tests.app
ports = gui_tests.ports
window = gui_tests.window


def endpoint_tracking(window, monkeypatch):
    clock, port = start_tracking(window, monkeypatch)
    worker = window.worker
    worker.solver = FakeSolver()
    # Minimal solver double has no timing/diagnostic fields for status text.
    monkeypatch.setattr(window, "update_status", lambda _dt: None)
    end = np.array(window.px_to_world_mm(window.full_path[-1]))
    window.shared.set_reference([end - [1, 0], end], 1, clock.now)
    window.state_pos_mm = end.copy()
    window.state_vel_mm = np.array([1.5, 0])
    window.bead = (*window.world_mm_to_px(end), 200)
    window.shared.set_kalman(end, window.state_vel_mm, clock.now)
    worker._step()
    clock.now += 0.001
    gui_tests.publish_camera_observation(window)
    return clock, port, worker, end


def test_fast_endpoint_keeps_worker_executor_and_nonzero_control(window, monkeypatch):
    clock, port, worker, end = endpoint_tracking(window, monkeypatch)
    normal = Mock(wraps=window.normal_stop)
    monkeypatch.setattr(window, "normal_stop", normal)
    for k in range(1, 10):
        clock.now += 0.05
        gui_tests.publish_camera_observation(window)
        window.mpc_track_step(0.05)
        if k % 2 == 0:
            worker._step()
        assert window.tracking and not window.stopping
        assert window.worker is worker and not window.shared.stopped()
        assert not window.shared.get_progress().finished
        assert any(window.last_sent_cmd)
        snap = window.shared.get_I_target()[3]
        np.testing.assert_allclose(snap["ref_target"], end)
        np.testing.assert_array_equal(snap["ref_velocity"], [0, 0])
    normal.assert_not_called()
    assert len(port.frames) >= 9 and window.executor.seq > 1


@pytest.mark.parametrize(
    "action", ["normal_stop", "emergency_stop", "serial_fault", "target_timeout"]
)
def test_safety_stop_bypasses_terminal_dwell(window, monkeypatch, action):
    clock, _port, _worker, _ = endpoint_tracking(window, monkeypatch)
    window.mpc_track_step(0.05)
    held = window.last_sent_cmd.copy()
    assert any(held)
    if action == "serial_fault":
        window._latch_serial_fault(OSError("simulated"))
    elif action == "target_timeout":
        clock.now += cfg.TARGET_STALE_STOP_S + 0.01
        window.mpc_track_step(0.05)
    else:
        getattr(window, action)()
    assert not window.tracking and window.worker is None
    assert window.shared.stopped()
    if action == "emergency_stop":
        assert window.last_sent_cmd == [0] * 6
    elif action in ("normal_stop", "target_timeout"):
        assert window.stopping and window.last_sent_cmd == held
        for _ in range(8):
            clock.now += 0.05
            window.tick()
        assert window.last_sent_cmd == [0] * 6


def test_visual_loss_revokes_ready_finish_without_waiting_for_worker(
    window, monkeypatch
):
    clock, _port, worker, end = endpoint_tracking(window, monkeypatch)
    for _ in range(6):
        clock.now += 0.125
        window.shared.set_kalman(end, [0, 0], clock.now)
        worker._step()
    assert window.shared.get_progress().finished
    window.bead = None
    normal = Mock(wraps=window.normal_stop)
    monkeypatch.setattr(window, "normal_stop", normal)
    window.mpc_track_step(0.05)
    assert not window.shared.get_progress().finished
    assert window.tracking and window.worker is worker
    normal.assert_not_called()


def test_restart_clears_partial_dwell_and_terminal_phase(window, monkeypatch):
    clock, _port, worker, end = endpoint_tracking(window, monkeypatch)
    for _ in range(3):
        clock.now += 0.125
        window.shared.set_kalman(end, [0, 0], clock.now)
        worker._step()
    old = window.shared
    window.normal_stop()
    window.start_tracking()
    assert old.stopped() and window.shared is not old
    p = window.shared.get_progress()
    assert not p.finished and not p.holding_endpoint
    assert p.approach_mono is None
    window.worker._step()
    assert not window.shared.get_progress().finished


@pytest.mark.parametrize("scenario", ["nominal", "friction_drag"])
def test_existing_synthetic_plant_endpoint_smoke(
    window, monkeypatch, tmp_path, scenario
):
    import json

    monkeypatch.setattr(ControlWorker, "start", lambda self: None)
    result = run_simulation(
        window,
        "L3",
        scenario,
        ticks=600,
        noise=False,
        profile="saved",
        target_box_delta=27,
        path_mm=[(-4, 0), (0, 0)],
        allow_finish=True,
    )
    evidence = {
        "scenario": scenario,
        "completion_time_s": result["completion_time_s"],
        "last_state_mm": window.state_pos_mm.tolist(),
        "last_speed_mm_s": float(np.linalg.norm(window.state_vel_mm)),
        "max_delta_cmd": result["max_delta_cmd"],
        "diverged": result["diverged"],
        "note": "Existing synthetic first-order plant, not physical bead validation.",
    }
    (tmp_path / "endpoint-simulation.json").write_text(
        json.dumps(evidence, indent=2), encoding="utf-8"
    )
    assert not result["diverged"] and result["max_delta_cmd"] <= 9
    assert result["completion_time_s"] is not None
    assert np.linalg.norm(window.state_pos_mm) <= cfg.PATH_FINISH_TOL_MM
    assert np.linalg.norm(window.state_vel_mm) <= cfg.PATH_FINISH_SPEED_MM_S
