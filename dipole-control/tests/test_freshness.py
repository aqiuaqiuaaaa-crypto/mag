"""Deterministic monotonic freshness, publication races and jitter tests."""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from unittest.mock import Mock

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import config as cfg
import multirate
from multirate import ControlWorker, CurrentExecutor, SharedState


@dataclass
class FakeClock:
    now: float = 0.0

    def __call__(self) -> float:
        return self.now


class FakeSolver:
    current_gain = cfg.CMD_TO_A
    n_coils = 6
    bead_moment = 5e-4

    def __init__(self) -> None:
        self.solve_calls = 0
        self.force_calls = 0

    def force_at(self, pos: Any, currents: Any) -> Any:
        self.force_calls += 1
        return np.asarray(currents)[:3] * 1e-6

    def solve_field_force_pseudoinverse(
        self, pos: Any, force: Any, **kwargs: Any
    ) -> dict[str, Any]:
        self.solve_calls += 1
        return {
            "currents": np.full(6, 0.5),
            "commands": np.full(6, 25),
            "achieved_force": np.asarray(force).copy(),
            "B": np.array([0.001, 0.002, -0.003]),
            "requested_B_direction": np.asarray(kwargs["B_direction"]).copy(),
            "converged": True,
        }

    def forward_model(self, pos: Any, currents: Any) -> dict[str, Any]:
        return {
            "B": np.array([0.001, 0.002, -0.003]),
            "B_magnitude_mT": 3.7416573867739413,
            "G": np.zeros((3, 3)),
            "grad_absB": np.zeros(3),
            "F": np.asarray(currents)[:3] * 1e-6,
        }


def setup_worker() -> tuple[FakeClock, SharedState, ControlWorker]:
    clock = FakeClock()
    shared = SharedState(clock=clock)
    shared.set_kalman([3, -4], [0, 0], 9999.0)
    shared.set_params({"mpc_on": True, "eso_d": np.zeros(2)}, 9999.0)
    shared.set_reference([[3, -4], [5, -3]], 1.0, 9999.0)
    return clock, shared, ControlWorker(shared, FakeSolver(), clock=clock)


def test_internal_timestamps_are_separate_from_caller_wall_time() -> None:
    clock = FakeClock(10.0)
    shared = SharedState(clock=clock)
    assert shared.get_kalman_snapshot()[3] is None
    assert shared.get_I_target()[3]["t_mono"] == 10.0
    shared.set_kalman([3, -4], [0, 0], 9999.0)
    assert shared.get_kalman_snapshot()[2:] == (9999.0, 10.0)
    assert shared.get_kalman()[2] == 9999.0
    clock.now = 11.0
    shared.set_params({"mpc_on": True}, -9999.0)
    assert shared.get_params()["t"] == -9999.0
    assert shared.get_params()["t_mono"] == 11.0
    assert shared.get_kalman_snapshot()[3] == 10.0
    clock.now = 12.0
    shared.set_I_target(np.zeros(6), None, [0, 0], 0, 0, 0)
    assert shared.get_I_target()[3]["t_mono"] == 12.0


@pytest.mark.parametrize(
    "age,should_publish",
    [(0.10, True), (0.15, True), (np.nextafter(0.15, np.inf), False), (0.20, False)],
)
def test_worker_input_boundaries(age: float, should_publish: bool) -> None:
    clock, shared, worker = setup_worker()
    clock.now = age
    worker._step()
    assert shared.get_I_target()[1] == int(should_publish)
    assert worker.solver.solve_calls == worker.solver.force_calls == int(should_publish)
    assert shared.get_freshness()["worker_status"] == (
        "FRESH" if should_publish else "STALE_INPUT"
    )
    assert shared.get_solver_error() is None


def test_stale_input_recovery_and_missing_input() -> None:
    clock, shared, worker = setup_worker()
    clock.now = 0.20
    worker._step()
    assert shared.get_I_target()[1] == 0
    shared.set_kalman([3.1, -4], [0, 0], -1000.0)
    worker._step()
    assert shared.get_I_target()[1] == 1
    assert shared.get_freshness()["worker_status"] == "FRESH"
    missing = SharedState(clock=clock)
    missing.set_params({"mpc_on": True}, 0)
    missing_worker = ControlWorker(missing, FakeSolver(), clock=clock)
    missing_worker._step()
    assert missing_worker.solver.solve_calls == 0
    assert missing.get_freshness()["worker_status"] == "STALE_INPUT"


@pytest.mark.parametrize("interruption", ["slow_solve", "newer_target", "stop"])
def test_commit_rechecks_age_seq_and_stop(
    monkeypatch: pytest.MonkeyPatch, interruption: str
) -> None:
    clock, shared, worker = setup_worker()
    original = worker.solver.solve_field_force_pseudoinverse

    def interfere(*args: Any, **kwargs: Any) -> Any:
        rec = original(*args, **kwargs)
        if interruption == "slow_solve":
            clock.now = 0.20
        elif interruption == "newer_target":
            shared.set_I_target(np.ones(6), {"newer": True}, [99, 99], 0, 0, 0)
        else:
            shared.stop()
        return rec

    monkeypatch.setattr(worker.solver, "solve_field_force_pseudoinverse", interfere)
    worker._step()
    target, seq, _, snapshot = shared.get_I_target()
    if interruption == "newer_target":
        assert seq == 1 and snapshot["rec"] == {"newer": True}
        np.testing.assert_array_equal(target, np.ones(6))
    else:
        assert seq == 0 and snapshot["t_mono"] == 0.0
        assert snapshot["rec"] is None
    assert shared.get_solver_error() is None


@pytest.mark.parametrize(
    "age,status",
    [
        (0.10, "FRESH"),
        (0.30, "FRESH"),
        (np.nextafter(0.30, np.inf), "STALE_TARGET"),
        (0.35, "STALE_TARGET"),
        (1.00, "STALE_TARGET"),
        (np.nextafter(1.00, np.inf), "STALE_TARGET_TIMEOUT"),
        (1.10, "STALE_TARGET_TIMEOUT"),
    ],
)
def test_target_boundaries_and_no_expired_seq_acceptance(
    age: float, status: str
) -> None:
    clock = FakeClock()
    shared = SharedState(clock=clock)
    executor = CurrentExecutor(clock=clock)
    solver = FakeSolver()
    shared.set_I_target(np.full(6, 0.5), None, [0, 0], 0, 0, 0)
    first = executor.step(shared, 1 / 30, np.zeros(3), solver, [0] * 6)
    last_sent = first["cmd"]
    before = (
        executor.seq,
        executor.frames_since,
        executor.I_from.copy(),
        executor.I_to.copy(),
    )
    shared.set_I_target(np.full(6, -1.0), None, [0, 0], 0, 0, 0)
    clock.now = age
    diag = executor.step(shared, 1 / 30, np.zeros(3), solver, last_sent)
    assert executor.stale_status == status
    if status == "FRESH":
        assert executor.seq == 2 and diag["cmd"] != last_sent
    else:
        assert executor.seq == before[0] and executor.frames_since == before[1]
        np.testing.assert_array_equal(executor.I_from, before[2])
        np.testing.assert_array_equal(executor.I_to, before[3])
        assert diag["cmd"] == last_sent
        assert diag["stop_requested"] == (status == "STALE_TARGET_TIMEOUT")
    assert solver.solve_calls == 0


def test_hold_then_new_fresh_target_resumes() -> None:
    clock = FakeClock()
    shared = SharedState(clock=clock)
    executor = CurrentExecutor(clock=clock)
    solver = FakeSolver()
    shared.set_I_target(np.full(6, 0.5), None, [0, 0], 0, 0, 0)
    cmd = executor.step(shared, 1 / 30, np.zeros(3), solver, [0] * 6)["cmd"]
    clock.now = 0.35
    hold = executor.step(shared, 1 / 30, np.zeros(3), solver, cmd)
    assert hold["cmd"] == cmd and executor.frames_since == 0
    shared.set_I_target(np.full(6, -0.5), None, [0, 0], 0, 0, 0)
    resumed = executor.step(shared, 1 / 30, np.zeros(3), solver, cmd)
    assert executor.seq == 2 and resumed["interp_alpha"] == 1 / 3
    assert executor.stale_status == "FRESH"


def test_publication_during_executor_read_is_not_overwritten(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    clock = FakeClock()
    shared = SharedState(clock=clock)
    executor = CurrentExecutor(clock=clock)
    solver = FakeSolver()
    shared.set_I_target(np.full(6, 0.5), None, [0, 0], 0, 0, 0)
    cmd = executor.step(shared, 1 / 30, np.zeros(3), solver, [0] * 6)["cmd"]
    clock.now = 0.35
    original = shared.get_I_target

    def concurrent_publication() -> Any:
        captured = original()
        shared.set_I_target(np.full(6, -0.5), None, [0, 0], 0, 0, 0)
        return captured

    monkeypatch.setattr(shared, "get_I_target", concurrent_publication)
    diag = executor.step(shared, 1 / 30, np.zeros(3), solver, cmd)
    assert diag["cmd"] == cmd and executor.seq == 1
    assert original()[1] == 2 and original()[3]["t_mono"] == 0.35
    monkeypatch.setattr(shared, "get_I_target", original)
    executor.step(shared, 1 / 30, np.zeros(3), solver, cmd)
    assert executor.seq == 2 and executor.stale_status == "FRESH"


def test_wall_clock_jump_has_no_effect(monkeypatch: pytest.MonkeyPatch) -> None:
    clock, shared, worker = setup_worker()
    wall = Mock(return_value=1e12)
    monkeypatch.setattr(multirate.time, "time", wall)
    clock.now = 0.10
    worker._step()
    assert shared.get_I_target()[2] == 1e12
    wall.return_value = -1e12
    clock.now = 0.20
    worker._step()
    assert shared.get_I_target()[1] == 1
    assert shared.get_freshness()["target_age_s"] == 0.10
    shared.set_kalman([3, -4], [0, 0], -1e12)
    worker._step()
    assert shared.get_I_target()[1] == 2 and shared.get_I_target()[2] == -1e12


def test_10000_tick_jitter_without_false_fault() -> None:
    clock, shared, worker = setup_worker()
    executor = CurrentExecutor(clock=clock)
    solver = FakeSolver()
    rng = np.random.default_rng(20261006)
    next_worker = 0.0
    last_sent = [0] * 6
    max_kalman_age = max_target_age = 0.0
    for _ in range(10000):
        camera_interval = float(rng.uniform(0.025, 0.070))
        camera_time = clock.now + camera_interval
        while next_worker <= camera_time:
            clock.now = next_worker
            max_kalman_age = max(max_kalman_age, shared.get_freshness()["kalman_age_s"])
            worker._step()
            next_worker += rng.uniform(0.070, 0.130)
        clock.now = camera_time
        shared.set_kalman([3, -4], [0, 0], -9999.0)
        diag = executor.step(shared, camera_interval, np.zeros(3), solver, last_sent)
        last_sent = diag["cmd"]
        max_target_age = max(max_target_age, executor.target_age_s)
        assert executor.stale_status == "FRESH" and not diag.get(
            "stop_requested", False
        )
        assert shared.get_freshness()["worker_status"] == "FRESH"
        assert shared.get_solver_error() is None
    assert max_kalman_age <= 0.070 and max_target_age <= 0.130
    print(
        f"JITTER ticks=10000 seq={shared.get_I_target()[1]} max_kalman_ms={max_kalman_age*1000:.6f} max_target_ms={max_target_age*1000:.6f} false_faults=0"
    )
