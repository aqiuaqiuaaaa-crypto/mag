"""Mode switches, persistence, prior-force timing and observation-only log fields."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import magnetic_dipole_pid as gui
import test_gui_curt_telemetry as telemetry_tests
from control_mode_simulation import MODES, configure_mode
from estimators import ESO1D, FirstOrderESO
from test_gui_control_log import finish_log, rows, start_log
from test_gui_freshness import start_tracking

app = telemetry_tests.app
ports = telemetry_tests.ports
window = telemetry_tests.window


def test_new_default_and_persistence(window: gui.MagneticDipoleControl) -> None:
    assert window._eso_mode() == gui.cfg.ESO_MODE == "first_order"
    assert (
        window.combo_effort.currentText() == gui.cfg.MPC_EFFORT_MODE == "steady_state"
    )
    assert isinstance(window.eso_x, FirstOrderESO)
    configure_mode(window, "L0")
    window.save_settings()
    configure_mode(window, "L3")
    window.load_settings()
    assert window._eso_mode() == "legacy" and isinstance(window.eso_x, ESO1D)
    assert window.combo_effort.currentText() == "absolute"


def test_existing_saved_settings_use_new_default_when_modes_absent(
    window: gui.MagneticDipoleControl,
) -> None:
    settings = json.loads(
        (Path(gui.__file__).parent / "gui_settings.json").read_text(encoding="utf8")
    )
    assert "eso_mode" not in settings and "mpc_effort_mode" not in settings
    Path(gui.SETTINGS_FILE).write_text(json.dumps(settings), encoding="utf8")
    window.load_settings()
    window._sync_physics_from_ui()
    window._rebuild_eso()
    assert window._eso_mode() == "first_order" and isinstance(
        window.eso_x, FirstOrderESO
    )
    assert window.combo_effort.currentText() == "steady_state"
    assert window.spin_mpc_w_pos.value() == 40
    assert window.spin_mpc_w_u.value() == 0.02


@pytest.mark.parametrize("mode", MODES)
def test_actual_modes_worker_and_csv(
    window: gui.MagneticDipoleControl,
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    mode: str,
) -> None:
    configure_mode(window, mode)
    clock, _ = start_tracking(window, monkeypatch)
    start_log(window, tmp_path / "mode.csv")
    assert window.worker is not None
    for _ in range(4):
        clock.now += 1 / 30
        window.tick()
        window.worker._step()
    eso, effort, enabled = MODES[mode]
    assert window.worker.mpc_x.effort_mode == effort
    finish_log(window)
    assert window.control_logger is not None
    for stream in (window.control_logger.path, window.control_logger.worker_path):
        records = rows(stream)
        assert records
        assert all(row["eso_mode"] == (eso if enabled else "off") for row in records)
        assert all(row["effort_mode"] == effort for row in records)
        assert all(row["F_ss_x"] and row["F_ss_y"] for row in records)
    metadata = json.loads(
        window.control_logger.metadata_path.read_text(encoding="utf8")
    )
    assert metadata["modes"]["eso_mode"] == (eso if enabled else "off")
    assert metadata["modes"]["effort_mode"] == effort


def test_previous_interval_force_not_future_target(
    window: gui.MagneticDipoleControl, monkeypatch: pytest.MonkeyPatch
) -> None:
    clock, _ = start_tracking(window, monkeypatch)
    window.last_F_actual = np.array([7e-6, -4e-6, 0])
    used = []

    def step(dt: float, x: float, u: float) -> float:
        used.append(u)
        return 2

    monkeypatch.setattr(window.eso_x, "step", step)
    monkeypatch.setattr(window.eso_y, "step", step)
    window.mpc_track_step(1 / 30)
    assert used == [7, -4]
    previous_estimate = window.last_F_actual[:2].copy() * 1e6
    clock.now += 1 / 30
    window.mpc_track_step(1 / 30)
    np.testing.assert_array_equal(used[2:], previous_estimate)
    np.testing.assert_array_equal(window.shared.get_params()["eso_d"], [2, 2])


def test_gui_gap_preserves_force_and_off_reenable_resets(
    window: gui.MagneticDipoleControl, monkeypatch: pytest.MonkeyPatch
) -> None:
    clock, _ = start_tracking(window, monkeypatch)
    assert isinstance(window.eso_x, FirstOrderESO)
    window.eso_x.reset(0)
    window.eso_x.z2 = 8
    bead = window.bead
    window.bead = None
    window.mpc_track_step(1 / 30)
    window.bead = bead
    window.state_pos_mm = np.array([100, 0])
    clock.now += 1 / 30
    window.mpc_track_step(1 / 30)
    assert window.z3.tolist() == [8, 0]
    window.chk_eso.setChecked(False)
    window.mpc_track_step(1 / 30)
    window.chk_eso.setChecked(True)
    window.state_pos_mm = np.array([200, 0])
    window.mpc_track_step(1 / 30)
    assert window.z3.tolist() == [0, 0]
    # Runtime structure switch resets ESO alone and preserves the active Kalman.
    state = window.kf.x.tobytes()
    window.combo_eso.setCurrentText("legacy")
    assert isinstance(window.eso_x, ESO1D)
    assert window.kf.x.tobytes() == state


@pytest.mark.parametrize("disabled_tick", [False, True])
def test_first_order_toggle_is_full_reset_even_between_ticks(
    window: gui.MagneticDipoleControl,
    monkeypatch: pytest.MonkeyPatch,
    disabled_tick: bool,
) -> None:
    start_tracking(window, monkeypatch)
    window.eso_x.reset(0)
    window.eso_x.z2 = 8
    window.chk_eso.setChecked(False)
    if disabled_tick:
        window.mpc_track_step(1 / 30)
    window.chk_eso.setChecked(True)
    assert isinstance(window.eso_x, FirstOrderESO)
    assert window.eso_x.z2 == 0 and not window.eso_x.initialized
    window.state_pos_mm = np.array([200, 0])
    window.mpc_track_step(1 / 30)
    assert window.z3.tolist() == [0, 0] and window.eso_x.z1 == 200


def test_new_tracking_and_mode_switch_discard_disturbance(
    window: gui.MagneticDipoleControl, monkeypatch: pytest.MonkeyPatch
) -> None:
    start_tracking(window, monkeypatch)
    previous = window.eso_x
    previous.reset(0)
    previous.z2 = 8
    window.start_tracking()
    assert window.eso_x is not previous and window.eso_x.z2 == 0
    window.eso_x.z2 = 8
    state = window.kf.x.tobytes()
    window.combo_eso.setCurrentText("legacy")
    window.combo_eso.setCurrentText("first_order")
    assert isinstance(window.eso_x, FirstOrderESO) and window.eso_x.z2 == 0
    assert not window.eso_x.initialized and window.kf.x.tobytes() == state


def test_legacy_toggle_keeps_historical_observer_state(
    window: gui.MagneticDipoleControl, monkeypatch: pytest.MonkeyPatch
) -> None:
    configure_mode(window, "L0")
    start_tracking(window, monkeypatch)
    previous = window.eso_x
    assert isinstance(previous, ESO1D)
    previous.reset(1)
    previous.z3 = 8
    window.chk_eso.setChecked(False)
    window.mpc_track_step(1 / 30)
    window.chk_eso.setChecked(True)
    assert window.eso_x is previous and previous.z3 == 8


def test_existing_log_fields_show_gap_and_lifecycle_reset(
    window: gui.MagneticDipoleControl,
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    clock, _ = start_tracking(window, monkeypatch)
    start_log(window, tmp_path / "gap.csv")
    window.eso_x.reset(0)
    window.eso_x.z2 = 8
    window.z3[:] = [8, 0]
    bead = window.bead

    def record(dt: float) -> None:
        clock.now += dt
        window._begin_control_log_tick(clock.now, dt)
        window.mpc_track_step(dt)
        window._finish_control_log_tick(window.bead is not None)

    window.bead = None
    record(1 / 30)
    window.bead = bead
    window.state_pos_mm = np.array([100, 0])
    record(1 / 30)
    window.state_pos_mm = np.array([200, 0])
    record(0.16)
    window.chk_eso.setChecked(False)
    record(1 / 30)
    window.chk_eso.setChecked(True)
    record(1 / 30)
    finish_log(window)
    assert window.control_logger is not None
    records = rows(window.control_logger.path)
    assert records[0]["detected"] == "False"
    assert records[0]["eso_updated"] == "False"
    assert records[0]["u_eso_x"] == "" and float(records[0]["d_hat_x"]) == 8
    assert records[1]["eso_updated"] == "True"
    assert float(records[1]["d_hat_x"]) == float(records[2]["d_hat_x"]) == 8
    assert float(records[2]["dt_ms"]) == 160
    assert records[3]["eso_mode"] == "off" and records[3]["eso_updated"] == "False"
    assert records[4]["eso_mode"] == "first_order"
    assert float(records[4]["d_hat_x"]) == 0


@pytest.mark.parametrize("pause", [0.16, 0.25, 0.30, 0.300001, 0.5])
def test_gui_tick_dt_clamp_preserves_force_but_is_not_watchdog_confirmation(
    window: gui.MagneticDipoleControl,
    monkeypatch: pytest.MonkeyPatch,
    pause: float,
) -> None:
    clock, _ = start_tracking(window, monkeypatch)
    assert window.worker is not None
    window.eso_x.reset(0)
    window.eso_x.z2 = 8
    before = 1000.0
    window.last_time = before
    monkeypatch.setattr(gui.time, "time", lambda: before + pause)
    clock.now += 1 / 30
    window.worker._step()
    window.tick()
    assert window.z3[0] == 8
    assert window.eso_x.z1 == window.state_pos_mm[0]


def test_live_effort_mode_refresh(
    window: gui.MagneticDipoleControl, monkeypatch: pytest.MonkeyPatch
) -> None:
    clock, _ = start_tracking(window, monkeypatch)
    assert window.worker is not None
    worker = window.worker
    previous = worker.mpc_x
    window.combo_effort.setCurrentText("absolute")
    clock.now += 0.01
    window._publish_mpc_params(1)
    worker._step()
    assert worker.mpc_x is not previous and worker.mpc_x.effort_mode == "absolute"
    assert worker.mpc_x.wu == previous.wu and worker.mpc_x.wd == previous.wd
