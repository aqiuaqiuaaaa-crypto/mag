"""Capture actual solver/forward inputs through every camera-side caller."""

from __future__ import annotations

import copy
import sys
from pathlib import Path
from unittest.mock import Mock

import numpy as np
import pytest
from numpy.typing import NDArray

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import config as cfg
import magnetic_dipole_pid as gui
import test_gui_curt_telemetry as telemetry_tests
from frames import cam_to_model_pos, cam_to_model_vec, model_to_cam_vec
from multirate import ControlWorker
from test_frames import TRANSFORMS
from test_gui_control_log import configure_camera
from test_gui_curt_telemetry import MemorySerial

app = telemetry_tests.app
ports = telemetry_tests.ports
window = telemetry_tests.window
Matrix = NDArray[np.float64]


def set_frames(widget: gui.MagneticDipoleControl, R: Matrix, offset: Matrix) -> None:
    configure_camera(widget)
    widget.ser = MemorySerial()
    widget.force_model_to_camera = R.copy()
    widget.frame_offset_mm = offset.copy()
    widget.state_pos_mm = R @ np.array([3.0, -4.0]) + offset
    widget.state_vel_mm = np.zeros(2)
    widget.bead = (*widget.world_mm_to_px(widget.state_pos_mm), 200.0)
    for spin, value in zip(
        (widget.spin_bdir_x, widget.spin_bdir_y, widget.spin_bdir_z), (0.3, -0.4, -0.8)
    ):
        spin.setValue(value)


def assert_model_positions(spy: Mock, expected: Matrix) -> None:
    assert spy.call_count > 0
    for call in spy.call_args_list:
        assert call.args[0].tobytes() == expected.tobytes()


@pytest.mark.parametrize("R,offset", TRANSFORMS)
@pytest.mark.parametrize(
    "caller", ["manual_once", "manual_live", "dir_test", "calibration"]
)
def test_gui_solver_callers(
    window: gui.MagneticDipoleControl,
    monkeypatch: pytest.MonkeyPatch,
    R: Matrix,
    offset: Matrix,
    caller: str,
) -> None:
    set_frames(window, R, offset)
    solve = Mock(wraps=window.solver.solve_field_force_pseudoinverse)
    forward = Mock(wraps=window.solver.forward_model)
    monkeypatch.setattr(window.solver, "solve_field_force_pseudoinverse", solve)
    monkeypatch.setattr(window.solver, "forward_model", forward)
    if caller.startswith("manual"):
        window.spin_fx.setValue(12)
        window.spin_fy.setValue(-5)
        window.spin_fz.setValue(2)
        if caller == "manual_live":
            window.chk_force_live.setChecked(True)
            window.tick()
        else:
            window.apply_manual_force()
        F_camera_uN = np.array([12.0, -5.0, 2.0])
    elif caller == "dir_test":
        window.spin_ftx.setValue(12)
        window.spin_fty.setValue(-5)
        window.spin_ftz.setValue(2)
        window.btn_dir_test.setChecked(True)
        window.direction_test_step()
        F_camera_uN = np.array([12.0, -5.0, 2.0])
    else:
        window.btn_calib.setChecked(True)
        window.calib_state = "ramp"
        window.calib_step(1 / 30)
        F_camera_uN = np.array([window.calib_Fx, 0.0, window.calib_fz_levels[0]])
    pos_model = cam_to_model_pos(window.state_pos_mm, R, offset)
    assert_model_positions(solve, pos_model)
    assert_model_positions(forward, pos_model)
    received = solve.call_args
    assert (
        received.args[1].tobytes() == cam_to_model_vec(F_camera_uN * 1e-6, R).tobytes()
    )
    field_cam, magnitude = window._field_target_camera()
    assert (
        received.kwargs["B_direction"].tobytes()
        == cam_to_model_vec(field_cam, R).tobytes()
    )
    assert received.kwargs["B_magnitude_mT"] == magnitude
    rec = window.last_solver_rec
    for key in (
        "achieved_force",
        "achieved_force_nonlinear",
        "achieved_force_linear",
        "B",
        "requested_B_direction",
    ):
        assert rec[key].tobytes() == model_to_cam_vec(rec[key + "_model"], R).tobytes()


@pytest.mark.parametrize("R,offset", TRANSFORMS)
def test_coil_scan_and_manual_current_diagnostics(
    window: gui.MagneticDipoleControl,
    monkeypatch: pytest.MonkeyPatch,
    R: Matrix,
    offset: Matrix,
) -> None:
    set_frames(window, R, offset)
    force = Mock(wraps=window.solver.force_at)
    forward = Mock(wraps=window.solver.forward_model)
    monkeypatch.setattr(window.solver, "force_at", force)
    monkeypatch.setattr(window.solver, "forward_model", forward)
    window.cur_spins[0].setValue(9)
    window.send_manual_currents()
    p = cam_to_model_pos(window.state_pos_mm, R, offset)
    assert_model_positions(forward, p)
    model_diag = window.solver.forward_model(
        p, np.array(window.last_sent_cmd) * window.solver.current_gain
    )
    assert (
        window.current_B_T.tobytes() == model_to_cam_vec(model_diag["B"], R).tobytes()
    )
    assert (
        window.current_F_N.tobytes()
        == model_to_cam_vec(np.atleast_1d(model_diag["F"]), R).tobytes()
    )
    forward.reset_mock()
    window.start_coil_scan()
    window.coil_test_phase = "drive"
    window.coil_test_frames = 10
    window.coil_test_samples = 19
    window.last_sent_cmd = [
        min(window._current_cmd_limit(), round(1.0 / window.solver.current_gain)),
        0,
        0,
        0,
        0,
        0,
    ]
    window.coil_test_step(1 / 30)
    assert len(window.coil_test_records) == 1
    assert_model_positions(force, p)
    assert_model_positions(forward, p)


@pytest.mark.parametrize("R,offset", TRANSFORMS)
def test_worker_and_executor_share_position_transform(
    window: gui.MagneticDipoleControl,
    monkeypatch: pytest.MonkeyPatch,
    R: Matrix,
    offset: Matrix,
) -> None:
    set_frames(window, R, offset)
    monkeypatch.setattr(gui.ControlWorker, "start", lambda self: None)
    window.path_px = [
        window.world_mm_to_px(window.state_pos_mm + [2, 1]),
        window.world_mm_to_px(window.state_pos_mm + [4, 2]),
    ]
    window.start_tracking()
    worker = window.worker
    assert isinstance(worker, ControlWorker)
    solve = Mock(wraps=worker.solver.solve_field_force_pseudoinverse)
    force = Mock(wraps=worker.solver.force_at)
    forward = Mock(wraps=window.solver.forward_model)
    monkeypatch.setattr(worker.solver, "solve_field_force_pseudoinverse", solve)
    monkeypatch.setattr(worker.solver, "force_at", force)
    monkeypatch.setattr(window.solver, "forward_model", forward)
    worker._step()
    p = cam_to_model_pos(window.state_pos_mm, R, offset)
    assert_model_positions(solve, p)
    assert_model_positions(force, p)
    snap = window.shared.get_I_target()[3]
    F_camera = np.array([*snap["F_target"], window.shared.get_params()["fz_lift"]])
    assert (
        solve.call_args.args[1].tobytes()
        == (cam_to_model_vec(F_camera, R) * 1e-6).tobytes()
    )
    B_camera = window._field_target_camera()[0]
    assert (
        solve.call_args.kwargs["B_direction"].tobytes()
        == cam_to_model_vec(B_camera, R).tobytes()
    )
    for key in (
        "achieved_force",
        "achieved_force_nonlinear",
        "achieved_force_linear",
        "B",
        "requested_B_direction",
    ):
        rec = snap["rec"]
        assert rec[key].tobytes() == model_to_cam_vec(rec[key + "_model"], R).tobytes()
    window.mpc_track_step(1 / 30)
    assert_model_positions(forward, p)
    diag = window.last_diag
    assert (
        diag["F_est"].tobytes()
        == model_to_cam_vec(window.executor.last_F_est, R).tobytes()
    )
    assert diag["B"].tobytes() == model_to_cam_vec(window.executor.last_B, R).tobytes()
    native_diag = window.solver.forward_model(p, window.executor.I_est)
    assert (
        diag["grad_absB"].tobytes()
        == model_to_cam_vec(native_diag["grad_absB"], R).tobytes()
    )


def test_offset_snapshot_and_metadata(window: gui.MagneticDipoleControl) -> None:
    R, offset = TRANSFORMS[-1]
    set_frames(window, R, offset)
    window._publish_mpc_params(1000.0)
    snapshot = window.shared.get_params()
    np.testing.assert_array_equal(snapshot["frame_offset_mm"], offset)
    snapshot["frame_offset_mm"][0] = 999
    assert window.shared.get_params()["frame_offset_mm"][0] == offset[0]
    metadata = window._control_log_metadata()
    assert metadata["R_force_model_to_camera"] == R.tolist()
    metadata_offset = metadata["offset"]
    assert isinstance(metadata_offset, dict)
    assert metadata_offset["position_mm"] == [*offset.tolist(), 0.0]
    assert "config.FRAME_OFFSET_MM" in metadata_offset["source"]
    assert cfg.FRAME_OFFSET_MM == (0.0, 0.0)


@pytest.mark.parametrize("R,offset", TRANSFORMS[1:])
def test_nonidentity_gui_matches_model_native_solver(
    window: gui.MagneticDipoleControl, R: Matrix, offset: Matrix
) -> None:
    set_frames(window, R, offset)
    native_solver = copy.deepcopy(window.solver)
    F_model = np.array([12e-6, -5e-6, 2e-6])
    F_camera = model_to_cam_vec(F_model, R)
    rec = window._solve_motion_target(window._bead_pos_m(), F_camera)
    B_model = cam_to_model_vec(window._field_target_camera()[0], R)
    native = native_solver.solve_field_force_pseudoinverse(
        np.array([0.003, -0.004, 0.0]),
        F_model,
        B_direction=B_model,
        B_magnitude_mT=window._field_target_camera()[1],
        cmd_prev=window.last_sent_cmd,
        max_cmd=window._current_cmd_limit(),
    )
    for key in ("currents", "commands", "pseudoinverse_current", "actuation_matrix"):
        assert rec[key].tobytes() == native[key].tobytes(), key
    for key in ("achieved_force", "achieved_force_linear", "B"):
        assert rec[key + "_model"].tobytes() == native[key].tobytes(), key


def test_model_origin_table_stays_model_native(
    window: gui.MagneticDipoleControl, monkeypatch: pytest.MonkeyPatch
) -> None:
    set_frames(window, *TRANSFORMS[-1])
    force = Mock(wraps=window.solver.force_at)
    monkeypatch.setattr(window.solver, "force_at", force)
    window._fill_coil_table_model()
    assert force.call_count == 6
    assert_model_positions(force, np.zeros(3))
