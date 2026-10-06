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


def test_gui_gap_and_off_reenable_clear_innovation(
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
    assert window.z3.tolist() == [0, 0]
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
