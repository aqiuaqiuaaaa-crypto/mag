"""Paired seeds, unchanged L3/box27 plant and numerical refinement subset."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import magnetic_dipole_pid as gui
import test_gui_curt_telemetry as telemetry_tests
from control_mode_simulation import SCENARIOS, run_simulation
from interpolation_analysis import MODES, evidence, simulation_metrics
from multirate import ControlWorker

app = telemetry_tests.app
ports = telemetry_tests.ports
window = telemetry_tests.window
CASES = [*SCENARIOS, "nominal_no_noise", "vision_gap", "corner_reversal_finish"]


@pytest.mark.parametrize("mode", MODES)
@pytest.mark.parametrize("profile", ["defaults", "saved"])
@pytest.mark.parametrize("seed", [20261006, 20261007, 20261008])
@pytest.mark.parametrize("case", CASES)
def test_paired_interpolation_ab(
    window: gui.MagneticDipoleControl,
    monkeypatch: pytest.MonkeyPatch,
    mode: str,
    profile: str,
    seed: int,
    case: str,
) -> None:
    monkeypatch.setattr(ControlWorker, "start", lambda self: None)
    corner = case == "corner_reversal_finish"
    result = run_simulation(
        window,
        "L3",
        case if case in SCENARIOS else "nominal",
        ticks=600 if corner else 240,
        noise=case != "nominal_no_noise",
        vision_gap=case == "vision_gap",
        profile=profile,
        target_box_delta=27,
        interpolation_mode=mode,
        seed=seed,
        path_mm=[(-4, 0), (-2, 0), (-2, 2), (-4, 2)] if corner else None,
        allow_finish=corner,
    )
    metrics = simulation_metrics(result)
    metrics["case"] = case
    evidence(
        f"simulation-{case}-{profile}-{seed}-{mode}",
        {**metrics, "trace": result["trace"]},
    )
    assert not result["diverged"]
    assert result["max_delta_cmd"] <= 9
    assert metrics["max_delta"] <= 9
    assert result["tracking_rms_mm"] < 5
    if corner:
        assert metrics["completed"]
    else:
        assert result["speed_std_mm_s"] < 0.8
        assert 0.8 < result["mean_path_speed_mm_s"] < 1.2


@pytest.mark.parametrize("mode", MODES)
@pytest.mark.parametrize("profile", ["defaults", "saved"])
def test_physical_integration_refinement(
    window: gui.MagneticDipoleControl,
    monkeypatch: pytest.MonkeyPatch,
    mode: str,
    profile: str,
) -> None:
    monkeypatch.setattr(ControlWorker, "start", lambda self: None)
    result = run_simulation(
        window,
        "L3",
        "nominal",
        profile=profile,
        target_box_delta=27,
        interpolation_mode=mode,
        plant_substeps=10,
    )
    metrics = simulation_metrics(result)
    evidence(
        f"refinement-nominal-{profile}-{mode}", {**metrics, "trace": result["trace"]}
    )
    assert not result["diverged"] and metrics["max_delta"] <= 9
