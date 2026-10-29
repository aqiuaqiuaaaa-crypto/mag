"""N5: monotonic hold budget, unsuccessful latch and existing stop/safety paths."""

from __future__ import annotations

import json
import sys
from pathlib import Path
from unittest.mock import Mock

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import config as cfg
import gui_test_support as gui_tests
from multirate import ControlWorker, SharedState
from test_endpoint_settling import sample, setup_endpoint
from test_freshness import FakeClock, FakeSolver
from test_gui_control_log import finish_log, rows, start_log
from test_gui_endpoint_settling import endpoint_tracking

app = gui_tests.app
ports = gui_tests.ports
window = gui_tests.window


def test_default_budget_allows_closed_loop_until_exact_deadline():
    rig = setup_endpoint()
    clock, shared, worker = rig
    assert cfg.ENDPOINT_SETTLING_TIMEOUT_S == 10.0
    for t in np.arange(0.125, 10, 0.125):
        before = shared.get_I_target()[1]
        assert not sample(rig, float(t), vel=(1, 0)).finished
        assert not shared.settling_timed_out()
        assert shared.get_I_target()[1] == before + 1
        assert np.any(shared.get_I_target()[0])
    before = shared.get_I_target()[1]
    assert not sample(rig, 10, vel=(1, 0)).finished
    assert shared.settling_timed_out()
    assert shared.get_I_target()[1] == before  # no new target after expiry
    clock.now = 10.125
    shared.set_kalman([1, 0], [0, 0], 0)
    worker._step()
    assert shared.settling_timed_out() and not shared.get_progress().finished


def test_deceleration_does_not_start_timeout_and_gap_does_not_extend_it():
    clock = FakeClock()
    shared = SharedState(clock=clock)
    shared.set_reference([[0, 0], [10, 0]], 1, 0)
    shared.set_params({"mpc_on": True, "settling_timeout_s": 0.5}, 0)
    worker = ControlWorker(shared, FakeSolver(), clock=clock)
    # Spend > timeout inside 2 mm zone, but reference has not reached endpoint.
    for i in range(11):
        clock.now = i * 0.1
        shared.set_kalman([8.1, 0], [0, 0], 0)
        worker._step()
        assert not shared.get_progress().holding_endpoint
        assert not shared.settling_timed_out()
    clock.now = 1.1
    shared.set_kalman([10, 0], [1, 0], 0)
    worker._step()
    assert not shared.get_progress().holding_endpoint  # bounded arc projection
    clock.now = 1.2
    shared.set_kalman([10, 0], [1, 0], 0)
    worker._step()
    assert shared.get_progress().holding_endpoint
    shared.reset_settling()  # vision loss cancels success dwell, not hold budget
    clock.now = 1.699
    assert not shared.settling_timed_out()
    clock.now = 1.701
    assert shared.settling_timed_out() and not shared.get_progress().finished


@pytest.mark.parametrize("change", ["stale", "nan_position", "speed", "missing"])
def test_invalid_or_missing_state_never_turns_timeout_into_completion(change):
    rig = setup_endpoint(settling_timeout_s=0.5)
    for t in [0.125, 0.25, 0.375]:
        assert not sample(rig, t).finished
    clock, shared, worker = rig
    clock.now = 0.5
    if change == "nan_position":
        shared.set_kalman([np.nan, 0], [0, 0], 0)
    elif change == "speed":
        shared.set_kalman([1, 0], [1, 0], 0)
    elif change == "missing":
        shared.reset_settling()
    else:
        clock.now = 0.6
    # No solve with NaN state is needed to reject a false completion.
    assert shared.settling_timed_out() and not shared.get_progress().finished
    assert not worker.finished


@pytest.mark.parametrize("bad_time", [-0.001, float("nan"), float("inf")])
def test_monotonic_failure_latches_unsuccessful_exit(bad_time):
    rig = setup_endpoint()
    sample(rig, 0.125, vel=(1, 0))
    rig[0].now = bad_time
    assert rig[1].settling_timed_out()
    assert not rig[1].get_progress().finished
    rig[0].now = 0.25
    assert rig[1].settling_timed_out()  # clock recovery cannot restart budget


@pytest.mark.parametrize("limit", [0, -1, float("nan"), float("inf")])
def test_invalid_budget_cannot_create_unbounded_hold(limit):
    clock = FakeClock()
    shared = SharedState(clock=clock)
    shared.set_reference([[0, 0], [1, 0]], 1, 0)
    shared.set_params({"mpc_on": True, "settling_timeout_s": limit}, 0)
    shared.set_kalman([1, 0], [1, 0], 0)
    ControlWorker(shared, FakeSolver(), clock=clock)._step()
    assert shared.settling_timed_out() and not shared.get_progress().finished
    assert shared.get_I_target()[1] == 0


def test_solve_crossing_deadline_cannot_publish_success(monkeypatch):
    rig = setup_endpoint(settling_timeout_s=0.7)
    for t in [0.125, 0.25, 0.375, 0.5]:
        sample(rig, t)
    clock, shared, worker = rig
    previous = shared.get_I_target()[1]
    solve = worker.solver.solve_field_force_pseudoinverse

    def slow(*args, **kwargs):
        result = solve(*args, **kwargs)
        clock.now += 0.08  # input still fresh, but hold budget has expired
        return result

    monkeypatch.setattr(worker.solver, "solve_field_force_pseudoinverse", slow)
    assert not sample(rig, 0.625).finished
    assert shared.settling_timed_out() and shared.get_I_target()[1] == previous


def test_success_before_deadline_and_stopped_diagnostic_are_distinct():
    rig = setup_endpoint(settling_timeout_s=0.75)
    for t in [0.125, 0.25, 0.375, 0.5, 0.625]:
        sample(rig, t)
    clock, shared, _ = rig
    assert shared.get_progress().finished and not shared.settling_timed_out()
    shared.stop()  # GUI normal completion acknowledges the result
    clock.now = 100
    assert shared.get_progress().finished and not shared.settling_timed_out()


def test_new_reference_resets_timeout_latch_and_hold_start():
    rig = setup_endpoint(settling_timeout_s=0.5)
    assert not sample(rig, 0.5, vel=(1, 0)).finished
    assert rig[1].settling_timed_out()
    rig[1].set_reference([[0, 0], [1, 0]], 1, 0)
    assert not rig[1].settling_timed_out()
    assert not sample(rig, 0.625, vel=(1, 0)).finished
    rig[0].now = 1.124
    assert not rig[1].settling_timed_out()
    rig[0].now = 1.125
    assert rig[1].settling_timed_out()


def timeout_tracking(window, monkeypatch):
    monkeypatch.setattr(cfg, "ENDPOINT_SETTLING_TIMEOUT_S", 0.5)
    rig = endpoint_tracking(window, monkeypatch)
    clock, _port, worker, _end = rig
    for t in [0.125, 0.25, 0.375]:
        clock.now = t
        gui_tests.publish_camera_observation(window)
        window.mpc_track_step(0.125)
        worker._step()
        assert window.tracking and not window.shared.get_progress().finished
    return rig


def test_gui_timeout_logs_failure_ramps_and_never_restarts(
    window, monkeypatch, tmp_path
):
    clock, port, _worker, _end = timeout_tracking(window, monkeypatch)
    start_log(window, tmp_path / "timeout.csv")
    held = window.last_sent_cmd.copy()
    assert max(map(abs, held)) > 9
    count = len(port.frames)
    normal, emergency = Mock(wraps=window.normal_stop), Mock(
        wraps=window.emergency_stop
    )
    monkeypatch.setattr(window, "normal_stop", normal)
    monkeypatch.setattr(window, "emergency_stop", emergency)
    clock.now = 0.5
    window._begin_control_log_tick(-10000, 0.125)  # wall clock is irrelevant
    gui_tests.publish_camera_observation(window)
    window.mpc_track_step(0.125)
    window._finish_control_log_tick(True)
    normal.assert_called_once()
    emergency.assert_not_called()
    assert window.stale_status == "SETTLING_TIMEOUT"
    assert "未完成" in window.lbl_dir.text()
    assert not window.shared.get_progress().finished
    assert window.stopping and not window.tracking and window.worker is None
    assert window.shared.stopped() and window.shared.settling_timed_out()
    assert window.last_sent_cmd == held and len(port.frames) == count
    for _ in range(15):
        before = window.last_sent_cmd.copy()
        clock.now += 1 / 30
        window.tick()
        assert all(
            abs(b) <= abs(a) and abs(b - a) <= 9
            for a, b in zip(before, window.last_sent_cmd)
        )
        assert not window.tracking and window.worker is None
    assert window.mode == "IDLE" and window.last_sent_cmd == [0] * 6
    finish_log(window)
    records = rows(window.control_logger.path)
    assert all(r["stale_status"] == "SETTLING_TIMEOUT" for r in records)
    assert all(r["finished"] == "False" for r in records)
    metadata = json.loads(window.control_logger.metadata_path.read_text("utf-8"))
    assert "SETTLING_TIMEOUT" in json.dumps(metadata)
    old = window.shared
    window.start_tracking()
    assert window.shared is not old and not window.shared.settling_timed_out()
    assert window.stale_status == "FRESH" and window.tracking


@pytest.mark.parametrize("action", ["normal_stop", "emergency_stop", "serial_fault"])
@pytest.mark.parametrize("at_deadline", [False, True])
def test_safety_actions_bypass_hold_budget(window, monkeypatch, action, at_deadline):
    clock, port, _worker, _end = timeout_tracking(window, monkeypatch)
    held = window.last_sent_cmd.copy()
    clock.now = 0.5 if at_deadline else 0.4
    if action == "serial_fault":
        port.fail_tx = True
        window.send_commands(held)  # real write-fault path, in-memory port
    else:
        getattr(window, action)()
    assert not window.tracking and window.worker is None and window.shared.stopped()
    if action == "emergency_stop":
        assert window.last_sent_cmd == [0] * 6 and not window.stopping
    elif action == "serial_fault":
        assert window.serial_fault and window.mode == "SERIAL_FAULT"
    else:
        assert window.stopping and window.last_sent_cmd == held
