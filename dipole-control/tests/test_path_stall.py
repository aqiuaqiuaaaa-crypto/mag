"""Measured path progress watchdog through worker, GUI and normal-stop UART."""

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
import magnetic_dipole_pid as gui
from multirate import ControlWorker, SharedState
from path_progress import PathProgress, PathStallWatchdog
from test_freshness import FakeClock, FakeSolver
from test_gui_control_log import finish_log, rows, start_log
from test_gui_freshness import start_tracking

app = gui_tests.app
ports = gui_tests.ports
window = gui_tests.window


def rig(path=None, speed=1.0):
    clock = FakeClock()
    shared = SharedState(clock=clock)
    shared.set_reference([[0, 0], [40, 0]] if path is None else path, speed, 0)
    shared.set_params({"mpc_on": True, "path_stall_active": True}, 0)
    worker = ControlWorker(shared, FakeSolver(), clock=clock)
    return clock, shared, worker


def observe(setup, t, pos=(0, 0)):
    clock, shared, worker = setup
    clock.now = t
    shared.set_kalman(pos, [0, 0], -10000 - t)  # wall clock intentionally reverses
    worker._step()
    return shared.path_stall_timed_out()


def test_default_grace_and_deadline_use_observed_time_without_early_stop():
    setup = rig()
    _, shared, _ = setup
    assert cfg.PATH_STALL_STARTUP_GRACE_S == 2
    assert cfg.PATH_STALL_TIMEOUT_S == 10
    for t in np.arange(0, 12, 0.125):
        seq = shared.get_I_target()[1]
        assert not observe(setup, float(t))
        assert shared.get_I_target()[1] == seq + 1
        assert np.any(shared.get_I_target()[0])  # still running the original loop
    seq = shared.get_I_target()[1]
    assert observe(setup, 12)
    assert not shared.get_progress().finished and not shared.settling_timed_out()
    assert shared.get_I_target()[1] == seq
    assert observe(setup, 12.125, (1, 0))  # recovery cannot clear a latched failure
    assert shared.get_I_target()[1] == seq


@pytest.mark.parametrize(
    "path",
    [
        [[0, 0], [40, 0]],
        [[0, 0], [4, 0], [4, 4], [0, 4], [0, 8], [30, 8]],
    ],
)
def test_straight_turns_and_short_pause_do_not_trip(path):
    setup = rig(path)
    points = np.asarray(path, float)
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    arc = np.r_[0, np.cumsum(lengths)]
    for t in np.arange(0, 24, 0.125):
        # A 1.5 s turn pause; thereafter normal 1 mm/s motion resumes.
        s = min(t, 4) if t < 5.5 else t - 1.5
        pos = [np.interp(s, arc, points[:, axis]) for axis in (0, 1)]
        assert not observe(setup, float(t), pos)
    assert setup[1].get_progress().s_progress > 20


@pytest.mark.parametrize("speed", [0, 0.01, 0.1, 0.199])
def test_low_reference_speed_is_not_stall_evidence(speed):
    setup = rig(speed=speed)
    for t in np.arange(0, 15, 0.125):
        assert not observe(setup, float(t))
    assert setup[1].get_I_target()[1] == 120


def test_motion_at_minimum_monitored_speed_does_not_trip():
    setup = rig(speed=cfg.PATH_STALL_MIN_REF_SPEED_MM_S)
    for t in np.arange(0, 20, 0.125):
        assert not observe(setup, float(t), (0.2 * t, 0))


def test_deceleration_and_endpoint_hold_remain_owned_by_endpoint_timeout():
    setup = rig([[0, 0], [10, 0]])
    for t in np.arange(0, 10, 0.125):
        assert not observe(setup, float(t), (min(t, 8.2), 0))
    assert setup[1].get_progress().approach_s is not None
    for t in np.arange(10, 30, 0.125):
        assert not observe(setup, float(t), (8.2, 0))
    assert setup[1].settling_timed_out()
    assert not setup[1].get_progress().finished


def test_progress_resets_budget_without_renewing_startup_grace():
    setup = rig()
    for t in np.arange(0, 11.125, 0.125):
        assert not observe(setup, float(t), (0.4 if t >= 11 else 0, 0))
    for t in np.arange(11.125, 21, 0.125):
        assert not observe(setup, float(t), (0.4, 0))
    assert observe(setup, 21, (0.4, 0))  # 10 s after progress, not 12 s


def test_bounded_noise_and_backtracking_are_not_accumulated_as_progress():
    setup = rig()
    for i in range(96):
        # Net range < threshold, but total travelled distance greatly exceeds it.
        assert not observe(setup, i / 8, (0.1 + 0.08 * (-1) ** i, 0))
    assert observe(setup, 12, (0.1, 0))


def test_projection_crawling_cannot_renew_budget_without_position_movement():
    watchdog = PathStallWatchdog()
    for i in range(97):
        watchdog.observe(i / 8, PathProgress(i * 0.4, 100), [40, 0], eligible=True)
        assert watchdog.timed_out == (i == 96)


@pytest.mark.parametrize(
    "event", ["missing", "stale_input", "stale_target", "inactive"]
)
def test_gaps_pause_evidence_but_inactive_mode_starts_a_new_window(event):
    setup = rig()
    clock, shared, worker = setup
    for t in np.arange(0, 11.875, 0.125):
        assert not observe(setup, float(t))
    clock.now = 12
    if event == "missing":
        shared.reset_settling()  # same entry as invalid GUI observation
    elif event == "stale_input":
        worker._step()
        assert shared.get_freshness()["worker_status"] == "STALE_INPUT"
    elif event == "stale_target":
        clock.now = 12.2
        shared.set_kalman([0, 0], [0, 0], 0)
        assert shared.get_freshness()["target_status"] == "STALE_TARGET"
    else:
        shared.set_params({"mpc_on": True, "path_stall_active": False}, 0)
    assert not shared.path_stall_timed_out()
    shared.set_params({"mpc_on": True, "path_stall_active": True}, 0)
    deadline = 24.5 if event == "inactive" else 12.75
    for t in np.arange(12.5, deadline, 0.125):
        assert not observe(setup, float(t))
    assert observe(setup, deadline)


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -1.0])
def test_invalid_or_backwards_time_cannot_supply_stationary_duration(bad):
    watchdog = PathStallWatchdog()
    for t in np.arange(0, 11.875, 0.125):
        watchdog.observe(float(t), PathProgress(), [0, 0], eligible=True)
    watchdog.observe(bad, PathProgress(), [0, 0], eligible=True)
    watchdog.observe(12, PathProgress(), [0, 0], eligible=True)
    assert not watchdog.timed_out


def test_repeated_observation_and_invalid_position_cannot_earn_time():
    watchdog = PathStallWatchdog()
    for _ in range(200):
        watchdog.observe(0, PathProgress(), [0, 0], eligible=True)
    assert not watchdog.timed_out
    watchdog.observe(0.125, PathProgress(), [np.nan, 0], eligible=True)
    np.testing.assert_array_equal(watchdog.anchor_pos, [0, 0])
    watchdog.observe(20, PathProgress(), [0, 0], eligible=True)
    assert not watchdog.timed_out


@pytest.mark.parametrize("race", ["stop", "new_reference", "slow_solve"])
def test_rejected_worker_publication_cannot_latch_stall(race, monkeypatch):
    setup = rig()
    clock, shared, worker = setup
    for t in np.arange(0, 12, 0.125):
        assert not observe(setup, float(t))
    solve = worker.solver.solve_field_force_pseudoinverse

    def changed(*args, **kwargs):
        result = solve(*args, **kwargs)
        if race == "stop":
            shared.stop()
        elif race == "new_reference":
            shared.set_reference([[0, 0], [30, 0]], 1, 0)
        else:
            clock.now += 0.2
        return result

    monkeypatch.setattr(worker.solver, "solve_field_force_pseudoinverse", changed)
    seq = shared.get_I_target()[1]
    assert not observe(setup, 12)
    assert shared.get_I_target()[1] == seq


def test_new_reference_resets_latched_failure():
    setup = rig()
    for i in range(97):
        observe(setup, i / 8)
    assert setup[1].path_stall_timed_out()
    setup[1].set_reference([[0, 0], [30, 0]], 1, 0)
    assert not observe(setup, 12.125)


def stalled_tracking(window, monkeypatch):
    monkeypatch.setattr(cfg, "PATH_STALL_STARTUP_GRACE_S", 0.25)
    monkeypatch.setattr(cfg, "PATH_STALL_TIMEOUT_S", 0.5)
    clock, port = start_tracking(window, monkeypatch)
    monkeypatch.setattr(gui, "detect_bead", lambda *_: (160, 90, 200, None))
    window.shared.set_reference([[0, 0], [20, 0]], 1, 0)
    worker = window.worker
    for i in range(1, 7):
        clock.now = i / 8
        window.tick()  # fresh actual production observation/ESO/Kalman path
        worker._step()
        assert window.tracking and not window.shared.path_stall_timed_out()
    return clock, port, worker


def test_gui_stall_logs_unfinished_ramps_and_requires_explicit_restart(
    window, monkeypatch, tmp_path
):
    clock, port, worker = stalled_tracking(window, monkeypatch)
    start_log(window, tmp_path / "path-stall.csv")
    clock.now = 0.875
    window.tick()
    worker._step()
    assert window.shared.path_stall_timed_out()
    held, count = window.last_sent_cmd.copy(), len(port.frames)
    assert max(map(abs, held)) > 9
    normal, emergency = Mock(wraps=window.normal_stop), Mock(
        wraps=window.emergency_stop
    )
    monkeypatch.setattr(window, "normal_stop", normal)
    monkeypatch.setattr(window, "emergency_stop", emergency)
    clock.now = 0.9
    window.tick()
    normal.assert_called_once()
    emergency.assert_not_called()
    assert (
        window.stale_status == "PATH_STALL_TIMEOUT"
        and "未完成" in window.lbl_dir.text()
    )
    assert window.stopping and not window.tracking and window.worker is None
    assert not window.shared.get_progress().finished
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
    assert all(r["finished"] == "False" for r in records)
    assert all(r["stale_status"] == "PATH_STALL_TIMEOUT" for r in records[1:])
    metadata = json.loads(window.control_logger.metadata_path.read_text("utf-8"))
    assert "PATH_STALL_TIMEOUT" in json.dumps(metadata)
    assert metadata["config"]["PATH_STALL_MIN_PROGRESS_MM"] == 0.3
    old = window.shared
    window.start_tracking()
    assert window.shared is not old and not window.shared.path_stall_timed_out()
    assert window.stale_status == "FRESH" and window.tracking


@pytest.mark.parametrize("action", ["normal_stop", "emergency_stop", "serial_fault"])
@pytest.mark.parametrize("latched", [False, True])
def test_safety_paths_never_wait_for_path_stall(window, monkeypatch, action, latched):
    clock, port, worker = stalled_tracking(window, monkeypatch)
    if latched:
        clock.now = 0.875
        window.tick()
        worker._step()
        assert window.shared.path_stall_timed_out()
    held = window.last_sent_cmd.copy()
    if action == "serial_fault":
        port.fail_tx = True
        window.send_commands(held)
    else:
        getattr(window, action)()
    assert not window.tracking and window.worker is None and window.shared.stopped()
    if action == "emergency_stop":
        assert window.last_sent_cmd == [0] * 6 and not window.stopping
    elif action == "serial_fault":
        assert window.serial_fault and window.mode == "SERIAL_FAULT"
    else:
        assert window.stopping and window.last_sent_cmd == held
