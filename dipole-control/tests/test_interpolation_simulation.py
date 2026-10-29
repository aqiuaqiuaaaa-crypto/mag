"""Paired seeds, unchanged L3/box27 plant and numerical refinement subset."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import gui_test_support as gui_tests
import magnetic_dipole_pid as gui
from control_mode_simulation import SCENARIOS, run_simulation
from interpolation_analysis import MODES, evidence, simulation_metrics
from multirate import ControlWorker

app = gui_tests.app
ports = gui_tests.ports
window = gui_tests.window
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
        # Completion now means sustained settling, not just passing within
        # 0.5 mm. Noisy corner cases may remain in closed loop for all 20 s.
        endpoint = np.asarray(result["path_mm"][-1])

        def eligible(row):
            return (
                row["holding_endpoint"]
                and np.linalg.norm(np.asarray(row["state_mm"]) - endpoint)
                <= gui.cfg.PATH_FINISH_TOL_MM
                and np.linalg.norm(row["state_velocity_mm_s"])
                <= gui.cfg.PATH_FINISH_SPEED_MM_S
            )

        if metrics["completed"]:
            assert (
                np.linalg.norm(np.asarray(result["final_state_mm"]) - endpoint)
                <= gui.cfg.PATH_FINISH_TOL_MM
            )
            assert (
                np.linalg.norm(result["final_state_velocity_mm_s"])
                <= gui.cfg.PATH_FINISH_SPEED_MM_S
            )
            # Every recorded observation over the final dwell must qualify.
            tail = [
                r
                for r in result["trace"]
                if r["time_s"]
                > result["completion_time_s"] - gui.cfg.PATH_FINISH_DWELL_S
            ]
            assert tail and all(eligible(r) for r in tail)
        else:
            assert window.tracking and not window.stopping
            assert window.shared.get_progress().holding_endpoint
            assert result["executed_ticks"] == 600
            # An uncompleted run must not hide a full settled interval at
            # the end (allow two worker periods for asynchronous observation).
            tail_s = gui.cfg.PATH_FINISH_DWELL_S + 2 / gui.cfg.MPC_HZ
            tail = result["trace"][-int(tail_s * gui.cfg.CONTROL_HZ) :]
            assert not all(eligible(r) for r in tail)
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
