"""Frame geometry, units, old bug evidence and real solver identity regression."""

from __future__ import annotations

import copy
import sys
from pathlib import Path

import numpy as np
import pytest
from numpy.typing import NDArray

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import config as cfg
from dipole_solver import DipoleSolver
from frames import cam_to_model_pos, cam_to_model_vec, model_to_cam_vec

Matrix = NDArray[np.float64]
TRANSFORMS = (
    (np.eye(2), np.zeros(2)),
    (np.array([[0.0, -1.0], [1.0, 0.0]]), np.zeros(2)),
    (np.diag([-1.0, 1.0]), np.zeros(2)),
    (np.array([[0.0, -1.0], [1.0, 0.0]]), np.array([1.2, -0.8])),
    (np.diag([-1.0, 1.0]), np.array([1.2, -0.8])),
)


@pytest.mark.parametrize("pos", [(0.0, 0.0), (3.0, -4.0), (-2.75, 1.25), (-0.0, 0.0)])
def test_identity_position_bitwise(pos: tuple[float, float]) -> None:
    old = np.array([pos[0], pos[1], 0.0]) * 1e-3
    new = cam_to_model_pos(pos, np.eye(2), cfg.FRAME_OFFSET_MM)
    assert new.tobytes() == old.tobytes()
    assert cfg.FRAME_OFFSET_MM == (0.0, 0.0)


@pytest.mark.parametrize("R,offset", TRANSFORMS)
def test_noncentral_position_and_target_recovery(R: Matrix, offset: Matrix) -> None:
    p_mm = np.array([3.0, -4.0])
    p_cam = R @ p_mm + offset
    recovered = cam_to_model_pos(p_cam, R, offset)
    np.testing.assert_allclose(
        recovered, np.array([3e-3, -4e-3, 0.0]), rtol=0, atol=1e-18
    )
    for vec in (np.array([20e-6, -7e-6, 2e-6]), np.array([0.003, -0.004, -0.008])):
        cam = model_to_cam_vec(vec, R)
        assert cam_to_model_vec(cam, R).tobytes() == vec.tobytes()


def test_position_units_and_plane() -> None:
    p = cam_to_model_pos([3.0, -4.0, 77.0], np.eye(2), [0.0, 0.0])
    assert p.tolist() == [0.003, -0.004, 0.0]


def test_random_vector_round_trip_and_purity() -> None:
    rng = np.random.default_rng(20261005)
    for angle in rng.uniform(-np.pi, np.pi, 30):
        c, s = np.cos(angle), np.sin(angle)
        for reflection in (1.0, -1.0):
            R = np.array([[c, -s], [s, c]]) @ np.diag([reflection, 1.0])
            for v in rng.normal(size=(20, 3)):
                original = v.copy()
                model = cam_to_model_vec(v, R)
                camera = model_to_cam_vec(model, R)
                assert np.max(np.abs(camera - v)) < 1e-12
                assert model[2] == camera[2] == v[2]
                assert v.tobytes() == original.tobytes()
    pos = np.array([3.0, -4.0])
    offset = np.array([1.2, -0.8])
    originals = (pos.copy(), offset.copy())
    cam_to_model_pos(pos, R, offset)
    assert pos.tobytes() == originals[0].tobytes()
    assert offset.tobytes() == originals[1].tobytes()


@pytest.mark.parametrize("R,offset", TRANSFORMS[1:])
def test_old_bug_position_untransformed(R: Matrix, offset: Matrix) -> None:
    p_model_mm = np.array([3.0, -4.0])
    p_cam_mm = R @ p_model_mm + offset
    old_pos = np.array([*p_cam_mm, 0.0]) * 1e-3
    correct_pos = np.array([*p_model_mm, 0.0]) * 1e-3
    F_model = np.array([20e-6, -7e-6, 2e-6])
    F_cam = model_to_cam_vec(F_model, R)
    # Old F/B transform already worked, but position remained in camera XY.
    old_F = F_cam.copy()
    old_F[:2] = R.T @ F_cam[:2]
    assert old_F.tobytes() == F_model.tobytes()
    assert not np.array_equal(old_pos, correct_pos)
    np.testing.assert_allclose(
        cam_to_model_pos(p_cam_mm, R, offset), correct_pos, rtol=0, atol=1e-18
    )


@pytest.mark.parametrize("pos", [(0.0, 0.0), (3.0, -4.0), (-2.75, 1.25)])
@pytest.mark.parametrize("F", [(12e-6, -5e-6, 2e-6), (-8e-6, 3e-6, 0.0)])
def test_identity_real_solver_and_diagnostics_bitwise(
    pos: tuple[float, float], F: tuple[float, float, float]
) -> None:
    old_solver = DipoleSolver.from_json()
    new_solver = copy.deepcopy(old_solver)
    R = np.eye(2)
    old_pos = np.array([pos[0], pos[1], 0.0]) * 1e-3
    new_pos = cam_to_model_pos(pos, R, cfg.FRAME_OFFSET_MM)
    F_old = np.array(F)
    F_old[:2] = R.T @ F_old[:2]
    B_cam = np.array([0.3, -0.4, -0.8])
    B_old = B_cam.copy()
    B_old[:2] = R.T @ B_old[:2]
    F_new = cam_to_model_vec(F, R)
    B_new = cam_to_model_vec(B_cam, R)
    assert old_pos.tobytes() == new_pos.tobytes()
    assert F_old.tobytes() == F_new.tobytes()
    assert B_old.tobytes() == B_new.tobytes()
    old = old_solver.solve_field_force_pseudoinverse(
        old_pos, F_old, B_direction=B_old, cmd_prev=[0] * 6, max_cmd=50
    )
    new = new_solver.solve_field_force_pseudoinverse(
        new_pos, F_new, B_direction=B_new, cmd_prev=[0] * 6, max_cmd=50
    )
    assert old.keys() == new.keys()
    for key in old:
        if key == "elapsed_ms":
            continue
        if isinstance(old[key], np.ndarray):
            assert old[key].tobytes() == new[key].tobytes(), key
        else:
            assert old[key] == new[key], key
    for key in (
        "achieved_force",
        "achieved_force_linear",
        "B",
        "requested_B_direction",
    ):
        old_camera = old[key].copy()
        old_camera[:2] = R @ old[key][:2]
        assert model_to_cam_vec(new[key], R).tobytes() == old_camera.tobytes()
