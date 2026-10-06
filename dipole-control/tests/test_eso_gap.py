"""Measurement continuity must not discard a learned disturbance force."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from estimators import FirstOrderESO
from test_eso_mpc import DT, C, observer


def converged() -> tuple[FirstOrderESO, float]:
    eso = observer()
    x = 0.0
    eso.reset(x)
    for _ in range(300):
        x += DT * (8 - 3) / C
        eso.step(DT, x, 8)
    assert eso.z2 == pytest.approx(-3, abs=1e-9)
    return eso, x


@pytest.mark.parametrize("frames", [1, 3, 15, 29])
def test_single_and_repeated_missing_frames_preserve_disturbance(frames: int) -> None:
    eso, x = converged()
    learned = eso.z2
    for _ in range(frames):
        eso.mark_gap()
        x += DT * (8 - 3) / C
        assert eso.z2 == learned
    x += DT * (8 - 3) / C
    assert eso.step(DT, x, 8) == learned
    assert eso.z1 == x
    estimates = []
    for _ in range(60):
        x += DT * (8 - 3) / C
        estimates.append(eso.step(DT, x, 8))
    assert max(abs(d + 3) for d in estimates) < 1e-9


@pytest.mark.parametrize("dt", [0.16, 0.25, 0.30, 0.300001, 0.5, 0.99])
def test_dt_discontinuity_preserves_force_without_integrating_gap(dt: float) -> None:
    eso, x = converged()
    learned = eso.z2
    x += dt * (8 - 3) / C
    assert eso.step(dt, x, 8) == learned
    assert eso.z1 == x
    x += DT * (8 - 3) / C
    assert eso.step(DT, x, 8) == pytest.approx(-3, abs=1e-9)


def test_max_dt_boundary_is_strict() -> None:
    eso = observer()
    eso.reset()
    assert eso.step(0.15, 1, 0) != 0  # correction, not re-anchor
    learned = eso.z2
    assert eso.step(float(np.nextafter(0.15, math.inf)), 10000, 0) == learned
    assert eso.z1 == 10000


def test_full_reset_and_new_instance_discard_disturbance() -> None:
    eso, _ = converged()
    eso.mark_gap()
    eso.reset(123)
    assert eso.z1 == 123 and eso.z2 == 0
    assert eso.step(DT, 123, 0) == 0
    fresh = observer()
    assert fresh.z2 == 0 and not fresh.initialized
    assert fresh.step(DT, 456, 40) == 0 and fresh.z1 == 456


def test_gap_at_motion_and_friction_reversal_reconverges() -> None:
    eso, x = converged()
    learned = eso.z2
    eso.mark_gap()
    x += 0.1 * (-8 + 3) / C
    assert eso.step(DT, x, -8) == learned
    values = []
    for _ in range(300):
        x += DT * (-8 + 3) / C
        values.append(eso.step(DT, x, -8))
        assert math.isfinite(eso.z1) and math.isfinite(eso.z2)
    assert max(abs(d) for d in values) < 4
    assert eso.z2 == pytest.approx(3, abs=1e-9)
