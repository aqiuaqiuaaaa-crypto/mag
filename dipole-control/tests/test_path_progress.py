"""Path topology, local/fallback projection and transactional publication."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

import numpy as np
import pytest
from numpy.typing import NDArray

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import config as cfg
from multirate import ControlWorker, SharedState
from path_progress import PathGeometry, PathProgress
from test_freshness import FakeClock, FakeSolver

Matrix = NDArray[np.float64]


def circle() -> Matrix:
    theta = np.linspace(0, 2 * np.pi, 241)
    return 2 * np.column_stack((np.cos(theta), np.sin(theta)))


def eight() -> Matrix:
    theta = np.linspace(-np.pi / 2, 3 * np.pi / 2, 801)
    return np.column_stack((4 * np.sin(theta), 3 * np.sin(2 * theta)))


def project(geometry: PathGeometry, pos: Any, previous: PathProgress) -> PathProgress:
    return geometry.project(
        pos,
        previous,
        fmax=cfg.MPC_FMAX_UN,
        c_drag=9.42,
        period=0.1,
        finish_tol=cfg.PATH_FINISH_TOL_MM,
    )


def old_projection(path: Matrix, pos: Matrix) -> float:
    """Exact parent projection equations, retained only as bug/comparison oracle."""
    vecs = np.diff(path, axis=0)
    lengths = np.linalg.norm(vecs, axis=1)
    arc = np.concatenate([[0.0], np.cumsum(lengths)])
    best, near = float("inf"), 0.0
    for i, vec in enumerate(vecs):
        length2 = float(vec @ vec)
        if length2 <= 1e-12:
            continue
        t = float(np.clip(((pos - path[i]) @ vec) / length2, 0, 1))
        d2 = float(np.sum((pos - (path[i] + t * vec)) ** 2))
        if d2 < best:
            best, near = d2, float(arc[i] + t * lengths[i])
    return near


@pytest.mark.parametrize("offset", [0.06, 0.08])
def test_circle_start_parent_bug(offset: float) -> None:
    path = circle()
    geometry = PathGeometry.from_path(path)
    pos = np.array([2.0, -offset])
    old_s = old_projection(path, pos)
    old_ref, old_vref = geometry.reference(PathProgress(old_s), 1.0, 0.1, 3)
    candidate = project(geometry, pos, PathProgress())
    ref, vref = geometry.reference(candidate, 1.0, 0.1, 3)
    assert old_s >= geometry.total - 0.1
    assert np.linalg.norm(old_vref[0]) == 0
    assert np.allclose(old_ref[0], path[-1])
    assert candidate.s_progress < 0.1 and not candidate.finished
    assert np.linalg.norm(vref[0]) > 0.99
    assert np.linalg.norm(ref[0] - path[-1]) > 0.09
    print(
        f"CIRCLE offset={offset} old_s={old_s:.12f} new_s={candidate.s_progress:.12f} total={geometry.total:.12f}"
    )


def test_circle_true_end_does_not_jump_back() -> None:
    geometry = PathGeometry.from_path(circle())
    previous = PathProgress(geometry.total - 0.2)
    end = project(geometry, geometry.points[-1], previous)
    assert end.s_progress > geometry.total - 1e-9
    assert end.finished and end.target_idx == len(geometry.points) - 1


@pytest.mark.parametrize(
    "path",
    [
        np.array([[0, 0], [4, 0], [4, 3], [0, 3], [0, 0]], dtype=float),
        np.array([[0, 0], [4, 0], [2, 3], [0, 0]], dtype=float),
    ],
)
def test_closed_polygon_start_not_finished(path: Matrix) -> None:
    geometry = PathGeometry.from_path(path)
    candidate = project(geometry, [0, 0.06], PathProgress())
    assert candidate.s_progress < 0.1 and not candidate.finished
    assert project(geometry, path[-1], PathProgress(geometry.total - 0.2)).finished


def test_eight_crossings_keep_branch_and_reference_continuity() -> None:
    path = eight()
    geometry = PathGeometry.from_path(path)
    previous = PathProgress()
    last_ref = None
    max_jump = 0.0
    for point, actual_s in zip(path, geometry.arc):
        candidate = project(geometry, point, previous)
        assert abs(candidate.s_progress - actual_s) < 1e-9
        assert not candidate.expanded_search and not candidate.path_deviation
        ref, _ = geometry.reference(candidate, 1.0, 0.1, 3)
        if last_ref is not None:
            max_jump = max(max_jump, float(np.linalg.norm(ref[0] - last_ref)))
        last_ref = ref[0]
        previous = candidate
    assert previous.finished and max_jump < 0.1
    print(
        f"EIGHT steps={len(path)} max_ref_jump_mm={max_jump:.12f} total={geometry.total:.12f}"
    )


@pytest.mark.parametrize(
    "path",
    [
        np.array([[0, 0], [8, 0]], dtype=float),
        np.array([[0, 0], [4, 0], [4, 4], [8, 4]], dtype=float),
    ],
)
def test_open_reference_matches_parent(path: Matrix) -> None:
    geometry = PathGeometry.from_path(path)
    previous = PathProgress()
    for actual_s in np.linspace(0, geometry.total, 121):
        pos = geometry.reference(PathProgress(float(actual_s)), 0, 0.1, 1)[0][0]
        candidate = project(geometry, pos, previous)
        old_s = old_projection(path, pos)
        assert abs(candidate.s_progress - old_s) < 1e-9
        old_ref, old_vref = geometry.reference(PathProgress(old_s), 1, 0.1, 3)
        ref, vref = geometry.reference(candidate, 1, 0.1, 3)
        np.testing.assert_allclose(ref, old_ref, atol=1e-9, rtol=0)
        np.testing.assert_allclose(vref, old_vref, atol=1e-9, rtol=0)
        previous = candidate


def test_1000_noisy_eight_steps_have_bounded_backtrack() -> None:
    geometry = PathGeometry.from_path(eight())
    previous = PathProgress()
    rng = np.random.default_rng(20261006)
    max_backtrack = max_forward = max_arc_error = 0.0
    for actual_s in np.linspace(0, geometry.total, 1000):
        pos = geometry.reference(PathProgress(float(actual_s)), 0, 0.1, 1)[0][0]
        noisy = pos + rng.normal(0, 0.07, 2)
        candidate = project(geometry, noisy, previous)
        delta = candidate.s_progress - previous.s_progress
        max_backtrack = max(max_backtrack, -delta)
        max_forward = max(max_forward, delta)
        max_arc_error = max(max_arc_error, abs(candidate.s_progress - actual_s))
        assert delta >= -cfg.PATH_BACKTRACK_MM - 1e-9
        assert abs(candidate.s_progress - actual_s) < 0.4
        assert not candidate.path_deviation and not candidate.expanded_search
        previous = candidate
    print(
        f"NOISE steps=1000 sigma_mm=0.07 max_backtrack_mm={max_backtrack:.12f} max_forward_mm={max_forward:.12f} max_arc_error_mm={max_arc_error:.12f}"
    )


@pytest.mark.parametrize(
    "offset,lost",
    [(2.0, False), (3.0, False), (np.nextafter(3.0, np.inf), True), (5.0, True)],
)
def test_fallback_and_loss_boundaries(offset: float, lost: bool) -> None:
    geometry = PathGeometry.from_path([[0, 0], [10, 0]])
    candidate = project(geometry, [5, offset], PathProgress())
    assert candidate.expanded_search and candidate.path_deviation == lost
    assert candidate.s_progress == 5 and candidate.distance_mm == offset
    assert not candidate.finished


@pytest.mark.parametrize(
    "offset,expanded", [(1.5, False), (np.nextafter(1.5, np.inf), True)]
)
def test_local_distance_boundary(offset: float, expanded: bool) -> None:
    geometry = PathGeometry.from_path([[0, 0], [10, 0]])
    candidate = project(geometry, [0.5, offset], PathProgress())
    assert candidate.expanded_search == expanded


def test_finish_needs_both_arc_and_endpoint() -> None:
    geometry = PathGeometry.from_path([[0, 0], [10, 0]])
    assert not project(geometry, [10, 0.6], PathProgress(9.8)).finished
    assert project(geometry, [9.5, 0], PathProgress(9.0)).finished
    assert not project(
        PathGeometry.from_path(circle()), [2, 0], PathProgress()
    ).finished


@pytest.mark.parametrize(
    "path", [[], [[2, 3]], [[2, 3], [2, 3]], [[0, 0], [0, 0], [2, 0]]]
)
def test_empty_single_duplicate_geometry(path: Any) -> None:
    geometry = PathGeometry.from_path(path)
    candidate = project(geometry, [2, 3], PathProgress())
    ref, vref = geometry.reference(candidate, 1, 0.1, 3)
    assert len(ref) == len(vref) == 3
    assert np.isfinite(ref).all() and np.isfinite(vref).all()


def test_partial_segment_window_and_dynamic_forward_range() -> None:
    geometry = PathGeometry.from_path([[0, 0], [10, 0]])
    backward = project(geometry, [4, 0], PathProgress(5))
    assert backward.s_progress == 5 - cfg.PATH_BACKTRACK_MM
    assert not backward.expanded_search
    forward = project(geometry, [2, 0], PathProgress())
    assert forward.s_progress == cfg.PATH_PROJ_FWD_FACTOR * cfg.MPC_FMAX_UN / 9.42 * 0.1
    assert not forward.expanded_search
    wider = geometry.project(
        [2.5, 0], PathProgress(), fmax=100, c_drag=10, period=0.1, finish_tol=0.5
    )
    assert wider.s_progress == 2.5 and not wider.expanded_search


def make_worker() -> tuple[FakeClock, SharedState, ControlWorker]:
    clock = FakeClock()
    shared = SharedState(clock=clock)
    shared.set_reference([[0, 0], [10, 0]], 1, 0)
    shared.set_params({"mpc_on": True}, 0)
    shared.set_kalman([0.8, 0], [0, 0], 0)
    return clock, shared, ControlWorker(shared, FakeSolver(), clock=clock)


@pytest.mark.parametrize(
    "interruption", ["stale_before", "slow_solve", "stop", "new_target", "new_path"]
)
def test_progress_cannot_commit_rejected_target(
    monkeypatch: pytest.MonkeyPatch, interruption: str
) -> None:
    clock, shared, worker = make_worker()
    worker._step()
    before = shared.get_progress()
    shared.set_kalman([1.2, 0], [0, 0], 0)
    original = worker.solver.solve_field_force_pseudoinverse

    def interrupt(*args: Any, **kwargs: Any) -> Any:
        rec = original(*args, **kwargs)
        if interruption == "slow_solve":
            clock.now = 0.2
        elif interruption == "stop":
            shared.stop()
        elif interruption == "new_target":
            shared.set_I_target(np.ones(6), None, [0, 0], 0, 0, 0)
        elif interruption == "new_path":
            shared.set_reference([[0, 0], [0, 8]], 1, 0)
        return rec

    monkeypatch.setattr(worker.solver, "solve_field_force_pseudoinverse", interrupt)
    if interruption == "stale_before":
        clock.now = 0.2
    worker._step()
    if interruption == "new_path":
        assert shared.get_progress().s_progress == 0
        assert shared.get_progress().s_total == 8
    else:
        assert (
            shared.get_progress() == before and worker.s_progress == before.s_progress
        )


def test_reference_reset_and_deviation_do_not_advance_arc() -> None:
    clock, shared, worker = make_worker()
    worker._step()
    assert shared.get_progress().s_progress == worker.s_progress == 0.8
    seq = shared.get_I_target()[1]
    shared.set_kalman([0.8, 5], [0, 0], 0)
    worker._step()
    assert shared.get_progress().path_deviation
    assert shared.get_progress().s_progress == 0.8 and shared.get_I_target()[1] == seq
    assert shared.get_solver_error() is None and worker.solver.solve_calls == 1
    shared.set_reference([[0, 0], [0, 8]], 1, 0)
    assert shared.get_progress() == PathProgress(s_total=8)
    shared.set_kalman([0, 0], [0, 0], 0)
    clock.now += 0.01
    worker._step()
    assert worker.s_progress == 0 and worker.s_total == 8 and not worker.finished
    assert not shared.get_progress().path_deviation


def test_deviation_report_rejected_when_stale_stopped_or_newer() -> None:
    clock, shared, _worker = make_worker()
    before = shared.get_progress()
    version = shared.get_reference_snapshot()[2]
    for expected_seq, expected_reference, age in [
        (1, version, 0),
        (0, version + 1, 0),
        (0, version, 0.2),
    ]:
        clock.now = age
        assert not shared.report_path_deviation(
            5,
            input_mono=0,
            expected_seq=expected_seq,
            expected_reference=expected_reference,
        )
        assert shared.get_progress() == before
    clock.now = 0
    shared.stop()
    assert not shared.report_path_deviation(
        5, input_mono=0, expected_seq=0, expected_reference=version
    )
    assert shared.get_progress() == before
