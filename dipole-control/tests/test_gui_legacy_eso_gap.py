"""B1: exercise tick -> detection/KF -> production legacy ESO, without adapters."""

from __future__ import annotations

import copy
import sys
from pathlib import Path
from unittest.mock import Mock

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import gui_test_support as gui_tests
import magnetic_dipole_pid as gui
from estimators import ESO1D, FirstOrderESO
from test_gui_freshness import start_tracking

app = gui_tests.app
ports = gui_tests.ports
window = gui_tests.window


def legacy_tracking(window, monkeypatch):
    clock, port = start_tracking(window, monkeypatch)
    window.combo_eso.setCurrentText("legacy")
    assert isinstance(window.eso_x, ESO1D)
    monkeypatch.setattr(gui.time, "time", lambda: 1000 + clock.now)
    window.last_time = 1000 + clock.now
    px = window.world_mm_to_px((0.3, -0.2))
    monkeypatch.setattr(gui, "detect_bead", lambda *_: (*px, 200, None))
    clock.now += 1 / 30
    window.tick()
    assert window.eso_x.initialized
    return clock, port


def check_historical_tick(window, clock, monkeypatch, interval):
    old = copy.deepcopy(window.eso_x)
    force = float(window.last_F_actual[0] * 1e6)
    step = Mock(wraps=window.eso_x.step)
    with monkeypatch.context() as patch:
        patch.setattr(window.eso_x, "step", step)
        previous = window.last_time
        clock.now += interval
        window.tick()
    dt = min(0.2, max(0.005, (1000 + clock.now) - previous))
    step.assert_called_once_with(dt, window.state_pos_mm[0], force)
    old.step(dt, window.state_pos_mm[0], force)
    assert {k: v for k, v in vars(window.eso_x).items() if k != "step"} == {
        k: v for k, v in vars(old).items() if k != "step"
    }
    np.testing.assert_array_equal(window.shared.get_kalman()[0], window.state_pos_mm)
    np.testing.assert_array_equal(window.shared.get_params()["eso_d"], window.z3)
    return dt


@pytest.mark.parametrize("gap", [0.3, 0.6, 0.93])
def test_vision_gap_recovery_uses_only_latest_tick_dt(window, monkeypatch, gap):
    clock, _ = legacy_tracking(window, monkeypatch)
    prior_observation = window._eso_observation_mono
    interval = gap / int(np.ceil(gap * 30))
    window.cap.fail = True
    frozen = vars(window.eso_x).copy()
    for _ in range(int(np.ceil(gap * 30)) - 1):
        clock.now += interval
        window.tick()
        assert vars(window.eso_x) == frozen and window.tracking
    window.cap.fail = False
    used = check_historical_tick(window, clock, monkeypatch, interval)
    assert clock.now - prior_observation == pytest.approx(gap)
    assert used == pytest.approx(interval) and used < 0.04
    assert not np.isclose(window.state_pos_mm[0], 0.3)
    assert window.tracking and not window.stopping


@pytest.mark.parametrize("interval", [0.001, 1 / 30, 0.06, 0.3, 0.6, 0.93])
def test_continuous_or_stalled_tick_retains_historical_clamp(
    window, monkeypatch, interval
):
    clock, _ = legacy_tracking(window, monkeypatch)
    used = check_historical_tick(window, clock, monkeypatch, interval)
    assert used == pytest.approx(min(0.2, max(0.005, interval)))


@pytest.mark.parametrize("gap", [1.2, 5.0])
def test_long_loss_stops_and_recovery_cannot_restart_legacy(window, monkeypatch, gap):
    clock, _ = legacy_tracking(window, monkeypatch)
    window.cap.fail = True
    frozen = vars(window.eso_x).copy()
    for _ in range(round(gap * 30)):
        clock.now += 1 / 30
        window.tick()
    assert not window.tracking and window.shared.stopped()
    assert vars(window.eso_x) == frozen
    window.cap.fail = False
    clock.now += 1 / 30
    window.tick()
    assert not window.tracking and vars(window.eso_x) == frozen
    window.start_tracking()
    assert not window.eso_x.initialized
    clock.now += 1 / 30
    window.tick()
    assert window.tracking and window.eso_x.initialized and window.z3[0] == 0


@pytest.mark.parametrize("gap", [0.3, 0.6, 0.93])
def test_first_order_keeps_raw_observation_interval_and_reanchor(
    window, monkeypatch, gap
):
    clock, _ = start_tracking(window, monkeypatch)
    assert isinstance(window.eso_x, FirstOrderESO)
    window.eso_x.z2 = window.z3[0] = 8
    window.bead = None
    window.mpc_track_step(1 / 30)
    clock.now += gap
    window.bead = (*window.world_mm_to_px((1, 0)), 200)
    window.estimate_state(1 / 30, True, observation_mono=clock.now)
    step = Mock(wraps=window.eso_x.step)
    monkeypatch.setattr(window.eso_x, "step", step)
    window.mpc_track_step(1 / 30)
    assert step.call_args.args[:2] == pytest.approx((gap, 1))
    assert window.z3[0] == 8 and window.eso_x.z1 == pytest.approx(1)
