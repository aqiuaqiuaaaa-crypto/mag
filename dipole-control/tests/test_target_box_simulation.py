"""Only the worker target half-width changes in the prior L3 plant framework."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import magnetic_dipole_pid as gui
import test_gui_curt_telemetry as telemetry_tests
from control_mode_simulation import SCENARIOS, run_simulation
from multirate import ControlWorker

app = telemetry_tests.app
ports = telemetry_tests.ports
window = telemetry_tests.window


@pytest.mark.parametrize("box", [9, 27])
@pytest.mark.parametrize("scenario", SCENARIOS)
@pytest.mark.parametrize("profile", ["defaults", "saved"])
def test_l3_box_ab(
    window: gui.MagneticDipoleControl,
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    box: int,
    scenario: str,
    profile: str,
) -> None:
    monkeypatch.setattr(ControlWorker, "start", lambda self: None)
    result = run_simulation(
        window, "L3", scenario, profile=profile, target_box_delta=box
    )
    (tmp_path / "box-simulation.json").write_text(
        json.dumps(result, indent=2), encoding="utf8"
    )
    print(
        "AB_EVIDENCE " + json.dumps({k: v for k, v in result.items() if k != "trace"})
    )
    assert not result["diverged"]
    assert result["max_delta_cmd"] <= 9
    assert result["tracking_rms_mm"] < 5
    assert result["cross_track_rms_mm"] < 1
    assert result["speed_std_mm_s"] < 0.8
    assert result["worker_publishes"] == 80 and result["command_frames"] == 240
    assert 0.8 < result["mean_path_speed_mm_s"] < 1.2
