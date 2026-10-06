"""L0-L3 closed loop with production Kalman, path, freshness, solver and executor."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import magnetic_dipole_pid as gui
import test_gui_curt_telemetry as telemetry_tests
from control_mode_simulation import MODES, SCENARIOS, run_simulation
from multirate import ControlWorker

app = telemetry_tests.app
ports = telemetry_tests.ports
window = telemetry_tests.window


@pytest.mark.parametrize("scenario", SCENARIOS)
@pytest.mark.parametrize("mode", MODES)
@pytest.mark.parametrize("profile", ["defaults", "saved"])
def test_full_chain_matrix(
    window: gui.MagneticDipoleControl,
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    mode: str,
    scenario: str,
    profile: str,
) -> None:
    monkeypatch.setattr(ControlWorker, "start", lambda self: None)
    result = run_simulation(window, mode, scenario, profile=profile)
    (tmp_path / f"{profile}-{scenario}-{mode}.json").write_text(
        json.dumps(result, indent=2), encoding="utf8"
    )
    assert not result["diverged"]
    assert result["max_delta_cmd"] <= gui.cfg.MAX_DELTA_CMD
    assert result["command_frames"] == 240
    assert result["worker_publishes"] == 80
    assert result["tracking_rms_mm"] < 5
    assert result["cross_track_rms_mm"] < 1
    assert result["speed_std_mm_s"] < 0.8
    if mode == "L3" and profile == "defaults":
        assert 0.8 < result["mean_path_speed_mm_s"] < 1.2
        assert abs(result["d_hat_x_uN"] - result["d_true_x_uN"]) < 2
    elif mode in ("L1", "L2"):
        assert result["d_hat_x_uN"] == 0


def test_full_chain_vision_gap_recovery(
    window: gui.MagneticDipoleControl, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(ControlWorker, "start", lambda self: None)
    result = run_simulation(window, "L3", "friction_drag", vision_gap=True)
    trace = result["trace"]
    assert not result["diverged"] and result["max_delta_cmd"] <= 9
    assert any(r["worker_status"] == "STALE_INPUT" for r in trace)
    assert trace[69]["d_hat_uN"] == [0, 0]
    assert np.linalg.norm(trace[70]["d_hat_uN"]) < 5
    assert 0.8 < result["mean_path_speed_mm_s"] < 1.2
