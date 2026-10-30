"""F-1: missing observations pause evidence instead of renewing the budget."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from path_progress import PathProgress, PathStallWatchdog
from test_path_stall import observe, rig

RATES = [0.0, 0.005, 0.01, 0.02, 0.05]


def periodic_missing(tick: int, rate: float, phase: int = 0) -> bool:
    if not rate:
        return False
    period = round(1 / rate)
    return tick % period == period // 2 + phase


@pytest.mark.parametrize("rate", RATES)
@pytest.mark.parametrize("phase", [0, 1, 2])
def test_periodic_gui_gaps_cannot_renew_worker_stall_budget(rate, phase):
    clock, shared, worker = rig()
    tripped = None
    for tick in range(600):  # 20 s, exact rates over this complete schedule
        clock.now = tick / 30
        if periodic_missing(tick, rate, phase):
            shared.reset_settling()  # production invalid-observation entry
        else:
            shared.set_kalman([0, 0], [0, 0], clock.now)
        if tick % 3 == 0:
            worker._step()
        if shared.path_stall_timed_out():
            tripped = clock.now
            break
    assert tripped is not None
    assert 12 <= tripped <= 15  # original 2 + 10 s plus excluded gap intervals
    assert not shared.get_progress().finished
    assert not shared.settling_timed_out()


@pytest.mark.parametrize("movement", [(0, 0), (0.4, 0)])
def test_recovery_compares_real_progress_without_renewing_grace(movement):
    setup = rig()
    clock, shared, _ = setup
    for t in np.arange(0, 11.875, 0.125):
        assert not observe(setup, float(t))
    clock.now = 11.875
    shared.reset_settling()
    assert not observe(setup, 12, movement)  # gap itself earns no time
    deadline = 22 if movement[0] else 12.25
    for t in np.arange(12.125, deadline, 0.125):
        assert not observe(setup, float(t), movement)
    assert observe(setup, deadline, movement)


def test_old_worker_snapshot_after_gap_cannot_resume_or_supply_progress():
    setup = rig()
    clock, shared, _ = setup
    for t in np.arange(0, 11.875, 0.125):
        assert not observe(setup, float(t))
    # This snapshot was published before the missing GUI frame, but has not
    # yet been consumed. A delayed solver may still try to commit it.
    clock.now = 11.8
    shared.set_kalman([1, 0], [0, 0], 0)
    clock.now = 11.85
    shared.reset_settling()
    watchdog = shared._path_stall
    for _ in range(5):
        watchdog.observe(11.8, PathProgress(1, 40), [1, 0], eligible=True)
    assert watchdog.anchor_s == 0
    np.testing.assert_array_equal(watchdog.anchor_pos, [0, 0])
    assert not observe(setup, 11.9)
    assert not observe(setup, 12.025)
    assert observe(setup, 12.15)


@pytest.mark.parametrize("rate", RATES)
def test_moving_turning_decelerating_worker_with_gaps_is_not_stalled(rate):
    points = np.array([[0, 0], [4, 0], [4, 4], [0, 4], [0, 8], [10, 8]])
    arc = np.r_[0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    clock, shared, worker = rig(points)
    approached = False
    for tick in range(900):
        clock.now = tick / 30
        s = min(clock.now, arc[-1])
        pos = [np.interp(s, arc, points[:, axis]) for axis in (0, 1)]
        if periodic_missing(tick, rate):
            shared.reset_settling()
        else:
            shared.set_kalman(pos, [0, 0], clock.now)
        if tick % 3 == 0:
            worker._step()
        approached |= shared.get_progress().approach_s is not None
        assert not shared.path_stall_timed_out()
    assert approached and shared.get_progress().s_progress > 25


def test_implicit_long_sample_gap_pauses_instead_of_erasing_evidence():
    watchdog = PathStallWatchdog()
    for t in np.arange(0, 11.875, 0.125):
        watchdog.observe(float(t), PathProgress(), [0, 0], eligible=True)
    watchdog.observe(20, PathProgress(), [0, 0], eligible=True)
    assert not watchdog.timed_out  # unknown interval is not stationary evidence
    watchdog.observe(20.125, PathProgress(), [0, 0], eligible=True)
    assert not watchdog.timed_out
    watchdog.observe(20.25, PathProgress(), [0, 0], eligible=True)
    assert watchdog.timed_out


@pytest.mark.parametrize("rate", RATES)
@pytest.mark.parametrize("seed", [20261030, 20261031, 20261101])
def test_random_exact_rate_gaps_still_have_a_bounded_stationary_deadline(rate, seed):
    clock, shared, worker = rig()
    missing = set(
        np.random.default_rng(seed).choice(600, round(600 * rate), replace=False)
    )
    assert len(missing) / 600 == rate
    tripped = None
    for tick in range(600):
        clock.now = tick / 30
        if tick in missing:
            shared.reset_settling()
        else:
            shared.set_kalman([0, 0], [0, 0], clock.now)
        if tick % 3 == 0:
            worker._step()
        if shared.path_stall_timed_out():
            tripped = clock.now
            break
    assert tripped is not None and 12 <= tripped < 19


def test_pause_partway_through_startup_grace_does_not_grant_new_grace():
    setup = rig()
    clock, shared, _ = setup
    for t in np.arange(0, 1.625, 0.125):
        assert not observe(setup, float(t))
    clock.now = 1.625
    shared.reset_settling()
    for t in np.arange(1.75, 12.25, 0.125):
        assert not observe(setup, float(t))
    assert observe(setup, 12.25)


def test_gap_during_solver_commit_cannot_count_its_old_snapshot(monkeypatch):
    setup = rig()
    _, shared, worker = setup
    for t in np.arange(0, 12, 0.125):
        assert not observe(setup, float(t))
    solve = worker.solver.solve_field_force_pseudoinverse

    def lose_observation(*args, **kwargs):
        result = solve(*args, **kwargs)
        shared.reset_settling()  # missing GUI frame while the solver was busy
        return result

    with monkeypatch.context() as patch:
        patch.setattr(
            worker.solver, "solve_field_force_pseudoinverse", lose_observation
        )
        assert not observe(setup, 12)
    assert not observe(setup, 12.125)  # real recovery establishes continuity
    assert observe(setup, 12.25)
