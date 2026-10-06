"""Pinned-parent no-gap golden and fixed-seed dropout through production GUI."""

from __future__ import annotations

import hashlib
import json
import os
import sys
from itertools import pairwise
from pathlib import Path
from typing import Any

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import magnetic_dipole_pid as gui
import test_gui_curt_telemetry as telemetry_tests
from control_mode_simulation import SCENARIOS, run_simulation
from multirate import ControlWorker

app = telemetry_tests.app
ports = telemetry_tests.ports
window = telemetry_tests.window
BASELINE = json.loads(
    (Path(__file__).parent / "fixtures/eso_gap_a2716e1.json").read_text(encoding="utf8")
)


def trace_digest(result: dict[str, Any]) -> str:
    trace = [
        {k: v for k, v in row.items() if k != "solver_ms"} for row in result["trace"]
    ]
    return hashlib.sha256(
        json.dumps(
            trace, sort_keys=True, separators=(",", ":"), allow_nan=False
        ).encode()
    ).hexdigest()


def save_evidence(name: str, result: dict[str, Any]) -> None:
    directory = os.environ.get("MAG_ESO_GAP_EVIDENCE_DIR")
    if directory:
        path = Path(directory)
        path.mkdir(parents=True, exist_ok=True)
        (path / f"{name}.json").write_text(
            json.dumps(result, indent=2), encoding="utf8"
        )


@pytest.mark.parametrize("profile", ["defaults", "saved"])
@pytest.mark.parametrize("scenario", SCENARIOS)
def test_no_gap_l3_parent_trace_is_bit_identical(
    window: gui.MagneticDipoleControl,
    monkeypatch: pytest.MonkeyPatch,
    profile: str,
    scenario: str,
) -> None:
    monkeypatch.setattr(ControlWorker, "start", lambda self: None)
    result = run_simulation(
        window, "L3", scenario, profile=profile, target_box_delta=27
    )
    key = f"{profile}-{scenario}"
    assert trace_digest(result) == BASELINE["no_gap"][key]["trace_sha256"]
    save_evidence(f"no-gap-{key}", result)


@pytest.mark.parametrize("rate", [0.01, 0.03, 0.10])
def test_random_dropout_keeps_compensation_and_recovers_speed(
    window: gui.MagneticDipoleControl,
    monkeypatch: pytest.MonkeyPatch,
    rate: float,
) -> None:
    monkeypatch.setattr(ControlWorker, "start", lambda self: None)
    result = run_simulation(
        window,
        "L3",
        "friction_drag",
        profile="saved",
        target_box_delta=27,
        ticks=360,
        dropout_fraction=rate,
        seed=BASELINE["seed"],
    )
    old = BASELINE["gap"][str(rate)]
    speed = result["mean_path_speed_mm_s"]
    # Registered before the fix, relative to the reproduced production plant.
    assert 0.95 <= speed <= 1.05
    assert abs(speed - 1) <= 0.4 * abs(old["mean_path_speed_mm_s"] - 1)
    assert result["tracking_rms_mm"] <= 0.75 * old["tracking_rms_mm"]
    assert not result["diverged"] and result["max_delta_cmd"] <= 9
    trace = result["trace"]
    recoveries = 0
    for previous, current in pairwise(trace):
        if previous["missing"]:
            np.testing.assert_array_equal(current["d_hat_uN"], previous["d_hat_uN"])
            recoveries += int(not current["missing"])
    assert recoveries > 0
    result["recoveries"] = recoveries
    save_evidence(f"random-{rate}", result)


def test_nine_frame_gap_keeps_nonzero_compensation(
    window: gui.MagneticDipoleControl, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(ControlWorker, "start", lambda self: None)
    result = run_simulation(
        window, "L3", "friction_drag", vision_gap=True, target_box_delta=27
    )
    trace = result["trace"]
    assert any(row["worker_status"] == "STALE_INPUT" for row in trace)
    assert np.linalg.norm(trace[59]["d_hat_uN"]) > 1
    for row in trace[60:70]:
        np.testing.assert_array_equal(row["d_hat_uN"], trace[59]["d_hat_uN"])
    assert np.linalg.norm(np.subtract(trace[70]["d_hat_uN"], trace[69]["d_hat_uN"])) < 1
    assert not result["diverged"] and result["max_delta_cmd"] <= 9
    save_evidence("nine-frames", result)
