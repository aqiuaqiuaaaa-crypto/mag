"""Endpoint planning and dwell with controlled monotonic time; no hardware."""

from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import config as cfg
from multirate import ControlWorker, SharedState
from path_progress import EndpointSettling, PathGeometry, PathProgress
from test_freshness import FakeClock, FakeSolver


def setup_endpoint(**params):
    clock = FakeClock()
    shared = SharedState(clock=clock)
    shared.set_reference([[0, 0], [1, 0]], 1, 0)
    shared.set_params({"mpc_on": True, **params}, 0)
    worker = ControlWorker(shared, FakeSolver(), clock=clock)
    shared.set_kalman([1, 0], [0, 0], 0)
    worker._step()
    assert shared.get_progress().holding_endpoint
    assert not shared.get_progress().finished
    return clock, shared, worker


def sample(rig, time, pos=(1, 0), vel=(0, 0)):
    clock, shared, worker = rig
    clock.now = time
    shared.set_kalman(pos, vel, -1000)  # deliberately unrelated wall clock
    worker._step()
    return shared.get_progress()


def test_reference_speed_tapers_continuously_and_reaches_zero():
    geo = PathGeometry.from_path([[0, 0], [10, 0]])
    d = cfg.PATH_END_DECEL_MM
    arcs = np.linspace(10 - d, 10, 101)
    speeds = [geo.advance_reference(s, 1, 0, d)[1] for s in arcs]
    assert speeds[0] == 1 and speeds[-1] == 0
    assert np.all(np.diff(speeds) <= 0)
    for boundary in [10 - d, 10]:
        left = geo.advance_reference(boundary - 1e-9, 1, 0, d)
        right = geo.advance_reference(boundary + 1e-9, 1, 0, d)
        assert abs(left[1] - right[1]) < 3e-5
    # Analytic reference finishes in 2D/v, without an asymptotic crawl.
    assert geo.advance_reference(10 - d, 1, 2 * d, d) == (10, 0)


def test_nonterminal_reference_and_geometry_are_unchanged():
    geo = PathGeometry.from_path([[0, 0], [10, 0]])
    ref, vel = geo.reference(PathProgress(2), 1, 0.1, 3)
    np.testing.assert_allclose(ref, [[2.1, 0], [2.2, 0], [2.3, 0]])
    np.testing.assert_allclose(vel, [[1, 0]] * 3)


@pytest.mark.parametrize("distance", [0, -1, float("nan"), float("inf")])
def test_invalid_deceleration_distance_fails_before_generating_reference(distance):
    geo = PathGeometry.from_path([[0, 0], [10, 0]])
    with pytest.raises(ValueError, match="deceleration distance"):
        geo.reference(PathProgress(), 1, 0.1, 3, decel_distance=distance)


def test_terminal_cursor_finishes_even_if_measured_position_stalls_short():
    clock = FakeClock()
    shared = SharedState(clock=clock)
    shared.set_reference([[0, 0], [1, 0]], 1, 0)
    shared.set_params({"mpc_on": True}, 0)
    worker = ControlWorker(shared, FakeSolver(), clock=clock)
    # 0.4 mm short: holding must continue, not falsely finish or freeze ref short.
    for i in range(60):
        clock.now = i * 0.1
        shared.set_kalman([0.6, 0], [0, 0], 0)
        worker._step()
    target = shared.get_I_target()[3]
    np.testing.assert_array_equal(target["ref_target"], [1, 0])
    np.testing.assert_array_equal(target["ref_velocity"], [0, 0])
    assert shared.get_progress().holding_endpoint
    assert not shared.get_progress().finished
    assert worker.solver.solve_calls == 60
    assert np.any(shared.get_I_target()[0])  # hold is still solved, not forced zero


@pytest.mark.parametrize(
    "path",
    [
        [[2, 3]],
        [[2, 3], [2, 3]],
        [[0, 0], [0, 0], [0.1, 0], [0.1, 0]],
    ],
)
def test_short_single_and_duplicate_paths_hold_actual_endpoint(path):
    geo = PathGeometry.from_path(path)
    ref, vel = geo.reference(PathProgress(geo.total), 1, 0.1, 3)
    np.testing.assert_allclose(ref, [path[-1]] * 3)
    np.testing.assert_allclose(vel, np.zeros((3, 2)))


@pytest.mark.parametrize("speed", [0.3, 1.412277676, 2.0])
def test_fast_arrival_never_finishes_and_still_solves(speed):
    rig = setup_endpoint()
    for t in np.arange(0.125, 1.25, 0.125):
        p = sample(rig, t, vel=(speed, 0))
        assert not p.finished
        snapshot = rig[1].get_I_target()[3]
        np.testing.assert_array_equal(snapshot["ref_target"], [1, 0])
        np.testing.assert_array_equal(snapshot["ref_velocity"], [0, 0])
    assert rig[2].solver.solve_calls > 8
    assert np.any(rig[1].get_I_target()[0])


def test_exact_position_speed_and_dwell_boundaries():
    rig = setup_endpoint()
    for t in [0.125, 0.25, 0.375, 0.5]:
        assert not sample(rig, t, pos=(1, 0.25), vel=(0.2, 0)).finished
    assert sample(rig, 0.625, pos=(1, 0.25), vel=(0.2, 0)).finished
    assert rig[2].finished


@pytest.mark.parametrize("kind", ["position", "speed", "vision"])
def test_excursion_between_worker_solves_restarts_entire_dwell(kind):
    rig = setup_endpoint()
    clock, shared, _worker = rig
    for t in [0.125, 0.25, 0.375, 0.5]:
        assert not sample(rig, t).finished
    # Worker never consumes this excursion; SharedState must still observe it.
    clock.now = 0.55
    if kind == "position":
        shared.set_kalman([1, np.nextafter(0.25, np.inf)], [0, 0], 0)
    elif kind == "speed":
        shared.set_kalman([1, 0], [np.nextafter(0.2, np.inf), 0], 0)
    else:
        shared.reset_settling()
    for t in [0.625, 0.75, 0.875, 1.0]:
        assert not sample(rig, t).finished
    assert sample(rig, 1.125).finished


def test_new_invalid_state_revokes_committed_finish_before_gui_can_stop():
    rig = setup_endpoint()
    for t in [0.125, 0.25, 0.375, 0.5, 0.625]:
        sample(rig, t)
    assert rig[1].get_progress().finished
    rig[0].now = 0.65
    rig[1].set_kalman([1, 0], [1, 0], 0)
    assert not rig[1].get_progress().finished


@pytest.mark.parametrize("gap", [0.151, 1.0, 10.0])
def test_long_observation_gap_cannot_count_as_settled(gap):
    rig = setup_endpoint()
    for t in [0.125, 0.25, 0.375, 0.5]:
        sample(rig, t)
    assert not sample(rig, 0.5 + gap).finished
    assert not sample(rig, 0.5 + gap + 0.125).finished


def test_repeated_snapshot_worker_calls_cannot_earn_dwell():
    rig = setup_endpoint()
    sample(rig, 0.125)
    clock, shared, worker = rig
    for t in [0.15, 0.2, 0.25]:
        clock.now = t
        worker._step()
        assert not shared.get_progress().finished
    clock.now = 1
    worker._step()
    assert shared.get_freshness()["worker_status"] == "STALE_INPUT"
    assert not shared.get_progress().finished


def test_target_stale_resets_dwell_even_with_fresh_positions():
    rig = setup_endpoint()
    clock, shared, worker = rig
    for t in [0.125, 0.25, 0.375, 0.5]:
        sample(rig, t)
    # Worker stalls while camera continues, then target freshness is lost.
    for t in [0.625, 0.75, 0.875]:
        clock.now = t
        shared.set_kalman([1, 0], [0, 0], 0)
    assert shared.get_freshness()["target_status"] == "STALE_TARGET"
    worker._step()
    for t in [1.0, 1.125, 1.25, 1.375]:
        assert not sample(rig, t).finished
    assert sample(rig, 1.5).finished


def test_slow_solve_does_not_commit_completion(monkeypatch):
    rig = setup_endpoint()
    for t in [0.125, 0.25, 0.375, 0.5]:
        sample(rig, t)
    clock, shared, worker = rig
    old_seq = shared.get_I_target()[1]
    original = worker.solver.solve_field_force_pseudoinverse

    def slow(*args, **kwargs):
        result = original(*args, **kwargs)
        clock.now += 0.16
        return result

    monkeypatch.setattr(worker.solver, "solve_field_force_pseudoinverse", slow)
    assert not sample(rig, 0.625).finished
    assert shared.get_I_target()[1] == old_seq


@pytest.mark.parametrize("interruption", ["vision", "excursion", "new_path", "stop"])
def test_commit_cannot_resurrect_dwell_invalidated_during_solve(
    monkeypatch, interruption
):
    rig = setup_endpoint()
    for t in [0.125, 0.25, 0.375, 0.5]:
        sample(rig, t)
    clock, shared, worker = rig
    original = worker.solver.solve_field_force_pseudoinverse

    def interfere(*args, **kwargs):
        result = original(*args, **kwargs)
        if interruption == "vision":
            shared.reset_settling()
        elif interruption == "excursion":
            clock.now += 0.01
            shared.set_kalman([1, 0], [1, 0], 0)
            clock.now += 0.01
            shared.set_kalman([1, 0], [0, 0], 0)
        elif interruption == "new_path":
            shared.set_reference([[0, 0], [0, 1]], 1, 0)
        else:
            shared.stop()
        return result

    monkeypatch.setattr(worker.solver, "solve_field_force_pseudoinverse", interfere)
    assert not sample(rig, 0.625).finished


def test_new_reference_does_not_inherit_timer_or_terminal_cursor():
    rig = setup_endpoint()
    for t in [0.125, 0.25, 0.375, 0.5, 0.625]:
        sample(rig, t)
    _, shared, _ = rig
    shared.set_reference([[0, 0], [1, 0]], 1, 0)
    assert not shared.get_progress().finished
    assert shared.get_progress().approach_s is None
    assert not sample(rig, 0.75).finished
    assert not sample(rig, 0.875).finished


def test_irregular_monotonic_samples_and_backward_time():
    timer = EndpointSettling()
    for t in [0.0, 0.13, 0.24, 0.38, 0.49]:
        timer.observe(t, True, 0.15)
        assert not timer.ready(t, 0.5)
    timer.observe(0.5, True, 0.15)
    assert timer.ready(0.5, 0.5)
    timer.observe(0.4, True, 0.15)  # fail closed on a broken monotonic source
    assert not timer.ready(0.4, 0.5)
    timer.observe(float("nan"), True, 0.15)
    assert timer.since is None


def test_dwell_uses_consumed_input_not_solve_completion_time():
    timer = EndpointSettling()
    for t in [0.0, 0.125, 0.25, 0.375]:
        timer.observe(t, True, 0.15)
    assert not timer.ready(0.5, 0.5)  # no observed sample at .5
    assert not timer.ready(0.375, 0.5)


def test_terminal_plan_pauses_across_gap_and_keeps_endpoint_after_backtrack():
    geo = PathGeometry.from_path([[0, 0], [10, 0]])
    prev = PathProgress(9, 10, approach_s=9, approach_mono=0)
    projected = PathProgress(8.8, 10)
    resumed = geo.plan_approach(projected, prev, 1, 5, 2)
    assert resumed.approach_s == 9
    held = geo.plan_approach(projected, replace(prev, approach_s=10), 1, 0.1, 2)
    ref, vel = geo.reference(held, 1, 0.1, 3)
    np.testing.assert_allclose(ref, [[10, 0]] * 3)
    np.testing.assert_allclose(vel, np.zeros((3, 2)))


def test_real_log_completion_snapshot_is_rejected_even_at_old_position_tolerance():
    clock = FakeClock()
    shared = SharedState(clock=clock)
    endpoint = [3.75, -0.7395833333333335]
    shared.set_reference([[3, endpoint[1]], endpoint], 1, 0)
    shared.set_params({"mpc_on": True, "finish_tol_mm": 0.5}, 0)
    worker = ControlWorker(shared, FakeSolver(), clock=clock)
    # Actual worker seq101 input, matched to control CSV row328.
    for t in np.arange(0, 1.25, 0.125):
        clock.now = float(t)
        shared.set_kalman(
            [4.204107417455867, -0.7582464874529283],
            [0.246559934874187, -1.3905885202740962],
            0,
        )
        worker._step()
        assert shared.get_progress().at_endpoint
        assert not shared.get_progress().finished
        np.testing.assert_array_equal(shared.get_I_target()[3]["ref_velocity"], [0, 0])
