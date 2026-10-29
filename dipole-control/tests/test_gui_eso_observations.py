"""Raw ESO observations stay distinct from MPC state and are consumed once."""

from __future__ import annotations

import sys
from pathlib import Path
from unittest.mock import Mock

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import gui_test_support as gui_tests
import magnetic_dipole_pid as gui
from test_gui_freshness import start_tracking

app = gui_tests.app
ports = gui_tests.ports
window = gui_tests.window


def observe(window, clock, position=(0.3, -0.2), *, stamp=None):
    """Publish a new synthetic detection through the production conversion."""
    window.bead = (*window.world_mm_to_px(position), 200)
    window.estimate_state(
        1 / 30, True, observation_mono=clock.now if stamp is None else stamp
    )


def test_raw_camera_mm_to_eso_but_kalman_state_and_compensation_to_mpc(
    window, monkeypatch
):
    clock, _ = start_tracking(window, monkeypatch)
    clock.now += 1 / 30
    # Nonidentity force calibration must not rotate the camera-world ESO position.
    window.force_model_to_camera = np.array([[0.0, -1.0], [1.0, 0.0]])
    px = window.world_mm_to_px((0.3, -0.2))
    monkeypatch.setattr(gui, "detect_bead", lambda *_: (*px, 200, None))
    sx, sy = Mock(return_value=2.0), Mock(return_value=-3.0)
    monkeypatch.setattr(window.eso_x, "step", sx)
    monkeypatch.setattr(window.eso_y, "step", sy)
    window.last_F_actual = np.array([7e-6, -4e-6, 0])
    window.tick()
    np.testing.assert_allclose(
        [sx.call_args.args[1], sy.call_args.args[1]], [0.3, -0.2]
    )
    np.testing.assert_allclose([sx.call_args.args[2], sy.call_args.args[2]], [7, -4])
    assert not np.allclose(window.state_pos_mm, [0.3, -0.2])
    pos, vel, _ = window.shared.get_kalman()
    np.testing.assert_array_equal(pos, window.state_pos_mm)
    np.testing.assert_array_equal(vel, window.state_vel_mm)
    worker = window.worker
    compute = Mock(wraps=worker.mpc_x.compute)
    monkeypatch.setattr(worker.mpc_x, "compute", compute)
    worker._step()
    assert compute.call_args.args[0] == window.state_pos_mm[0]
    assert compute.call_args.args[1] == 2
    np.testing.assert_array_equal(window.shared.get_params()["eso_d"], [2, -3])


@pytest.mark.parametrize("kind", ["consumed", "duplicate", "stale", "future", "nan"])
def test_unusable_observation_cannot_update_eso_or_refresh_shared_state(
    window, monkeypatch, kind
):
    clock, _ = start_tracking(window, monkeypatch)
    clock.now += 1 / 30
    observe(window, clock)
    window.mpc_track_step(1 / 30)
    state = (window.eso_x.z1, window.eso_x.z2, window.z3.copy())
    before = window.shared.get_kalman_snapshot()[3]
    step = Mock(wraps=window.eso_x.step)
    monkeypatch.setattr(window.eso_x, "step", step)
    if kind == "duplicate":
        observe(window, clock)
    elif kind == "stale":
        observe(window, clock)
        clock.now += gui.cfg.KALMAN_STALE_S + 0.001
    elif kind == "future":
        observe(window, clock, stamp=clock.now + 1)
    elif kind == "nan":
        observe(window, clock, stamp=float("nan"))
    window.mpc_track_step(1 / 30)
    step.assert_not_called()
    assert (window.eso_x.z1, window.eso_x.z2) == state[:2]
    np.testing.assert_array_equal(window.z3, state[2])
    assert window.eso_x._gap
    assert window.shared.get_kalman_snapshot()[3] == before


@pytest.mark.parametrize("bad", [(None, 4), (4, None), (np.nan, 4), (4, np.inf)])
def test_invalid_detection_does_not_become_zero_or_poison_kalman(
    window, monkeypatch, bad
):
    clock, _ = start_tracking(window, monkeypatch)
    monkeypatch.setattr(gui, "detect_bead", lambda *_: (*bad, 200, None))
    step = Mock(wraps=window.eso_x.step)
    monkeypatch.setattr(window.eso_x, "step", step)
    clock.now += 1 / 30
    window.tick()
    step.assert_not_called()
    assert window.bead is None and window._vision_observation is None
    assert np.all(np.isfinite(window.kf.x))
    assert window.eso_x._gap


def test_cached_image_without_new_read_is_not_a_new_detection(window, monkeypatch):
    clock, _ = start_tracking(window, monkeypatch)
    window.cap = None  # A previously displayed frame remains in window.frame.
    assert window.frame is not None
    detect = Mock()
    monkeypatch.setattr(gui, "detect_bead", detect)
    clock.now += 1 / 30
    window.tick()
    detect.assert_not_called()
    assert window.bead is None and window.eso_x._gap


@pytest.mark.parametrize("missing", [False, True])
def test_observation_interval_and_gap_reanchor_preserve_disturbance(
    window, monkeypatch, missing
):
    clock, _ = start_tracking(window, monkeypatch)
    window.eso_x.z2 = 8
    window.z3[0] = 8
    if missing:
        window.bead = None
        window.mpc_track_step(0.005)
    clock.now += 0.16 if not missing else 1 / 30
    observe(window, clock, (1.0, 0))
    step = Mock(wraps=window.eso_x.step)
    monkeypatch.setattr(window.eso_x, "step", step)
    window.mpc_track_step(0.005)  # ESO must use observation times, not this clamp.
    assert step.call_args.args[0] == pytest.approx(0.16 if not missing else 1 / 30)
    assert window.z3[0] == 8 and window.eso_x.z1 == pytest.approx(1)


@pytest.mark.parametrize("restart", ["tracking", "toggle", "structure"])
def test_lifecycle_discards_pending_measurement_and_disturbance(
    window, monkeypatch, restart
):
    clock, _ = start_tracking(window, monkeypatch)
    clock.now += 1 / 30
    observe(window, clock)
    window.eso_x.z2 = 8
    if restart == "tracking":
        window.start_tracking()
    elif restart == "toggle":
        window.chk_eso.setChecked(False)
        window.chk_eso.setChecked(True)
    else:
        window.combo_eso.setCurrentText("legacy")
        window.combo_eso.setCurrentText("first_order")
    window.mpc_track_step(1 / 30)
    assert not window.eso_x.initialized and window.eso_x.z2 == 0
    clock.now += 1 / 30
    observe(window, clock, (0.6, 0.4))
    window.mpc_track_step(1 / 30)
    assert window.eso_x.initialized and window.eso_x.z2 == 0
    assert window.eso_x.z1 == pytest.approx(0.6)


def test_timestamp_is_read_completion_not_detection_completion(window, monkeypatch):
    clock, _ = start_tracking(window, monkeypatch)
    read = window.cap.read
    detect = gui.detect_bead

    def delayed_read():
        clock.now += 0.04
        return read()

    def delayed_detection(*args):
        clock.now += 0.02
        return detect(*args)

    monkeypatch.setattr(window.cap, "read", delayed_read)
    monkeypatch.setattr(gui, "detect_bead", delayed_detection)
    step = Mock(wraps=window.eso_x.step)
    monkeypatch.setattr(window.eso_x, "step", step)
    window.tick()
    assert step.call_args.args[0] == pytest.approx(0.04)
    assert window._eso_observation_mono == pytest.approx(0.04)
