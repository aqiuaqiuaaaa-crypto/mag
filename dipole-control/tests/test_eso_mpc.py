"""Order/units, disturbance recovery, QP derivation and parent binary golden data."""

from __future__ import annotations

import json
import math
import struct
import sys
from functools import partial
from pathlib import Path
from typing import Any

import numpy as np
import pytest
from numpy.typing import NDArray

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import config as cfg
from dipole_solver import DipoleSolver
from estimators import ESO1D, FirstOrderESO, KalmanFilter2D
from mpc import ForceMPC

C = DipoleSolver.from_json().drag_uN_per_mm_s(cfg.VISCOSITY_MPA_S)
DT = 1 / cfg.CONTROL_HZ


def observer(omega: float = cfg.ESO_OMEGA0, limit: float = 80) -> FirstOrderESO:
    return FirstOrderESO(1 / C, omega, limit, max_dt=cfg.KALMAN_STALE_S)


@pytest.mark.parametrize("u,d", [(8, 3), (-8, 3), (8, -3), (-8, -3), (0, 6), (6, 0)])
def test_constant_input_disturbance_and_legacy_counterexample(
    u: float, d: float
) -> None:
    new = observer()
    old = ESO1D(1 / C, cfg.ESO_OMEGA0, cfg.ESO_FAL_DELTA, cfg.ESO_DIST_LIMIT_UN)
    x = 0.0
    new.reset(x)
    old.reset(x)
    for _ in range(6000):
        x += DT * (u + d) / C
        new.step(DT, x, u)
        old.step(DT, x, u)
    assert abs(new.z2 - d) < 1e-9
    assert abs(old.z3 + u) < 1e-8


def test_predict_correct_units_and_no_future_target() -> None:
    eso = observer()
    eso.reset(1)
    # b0 mm/(µN*s), l1 1/s, l2 µN/(mm*s).
    assert C == pytest.approx(6 * math.pi * 1 * 0.5e-3 * 1e3)
    assert eso.l1 == 2 * cfg.ESO_OMEGA0
    assert eso.l2 == C * cfg.ESO_OMEGA0**2
    u, d = 7.0, -2.0
    predicted = 1 + DT * u / C
    measured = 1 + DT * (u + d) / C
    error = measured - predicted
    result = eso.step(DT, measured, u)
    assert eso.z1 == predicted + DT * eso.l1 * error
    assert result == DT * eso.l2 * error
    # Independently rescale mm -> m and µN -> N. Physical output is identical.
    si = FirstOrderESO(1 / (C * 1e-3), cfg.ESO_OMEGA0, 80e-6)
    si.reset(1e-3)
    assert si.step(DT, measured * 1e-3, u * 1e-6) == pytest.approx(result * 1e-6)
    assert si.z1 == pytest.approx(eso.z1 * 1e-3)


def test_disturbance_steps_and_limit() -> None:
    eso = observer(limit=10)
    eso.reset()
    x = 0.0
    for d in (5.0, -7.0, 0.0, 30.0, -30.0):
        tail = []
        for _ in range(300):
            x += DT * (4 + d) / C
            tail.append(eso.step(DT, x, 4))
            assert math.isfinite(eso.z1) and abs(eso.z2) <= 10
        assert np.mean(tail[-30:]) == pytest.approx(np.clip(d, -10, 10), abs=1e-5)


def test_kalman_noise_path() -> None:
    rng = np.random.default_rng(20261006)
    kf = KalmanFilter2D(cfg.KALMAN_Q_POS, cfg.KALMAN_Q_VEL, cfg.KALMAN_R)
    eso = observer()
    estimates = []
    x = 0.0
    for _ in range(1800):
        x += DT * (8 - 3) / C
        measurement = np.array([x, 0]) + rng.normal(0, math.sqrt(cfg.KALMAN_R), 2)
        pos, _ = kf.step(DT, measurement)
        estimates.append(eso.step(DT, float(pos[0]), 8))
    tail = np.asarray(estimates[-900:])
    assert abs(float(tail.mean()) + 3) < 0.2
    assert float(tail.std()) < 1.5
    assert np.max(np.abs(tail)) < 5


@pytest.mark.parametrize("gap", ["missing", "large_dt", "initial"])
def test_gap_reanchors_without_force_impulse(gap: str) -> None:
    eso = observer()
    eso.reset()
    eso.z2 = 7
    if gap == "missing":
        eso.mark_gap()
    elif gap == "initial":
        eso.initialized = False
    dt = np.nextafter(cfg.KALMAN_STALE_S, math.inf) if gap == "large_dt" else DT
    assert eso.step(float(dt), 10000, -40) == 0
    assert eso.z1 == 10000
    assert eso.step(DT, 10000 + DT * (-40 + 3) / C, -40) > 0


def test_allowed_discrete_grid_and_schur_boundary() -> None:
    bound = math.sqrt(8) - 2
    for omega in np.linspace(0.5, 20, 40):
        for dt in np.linspace(0.005, 0.2, 40):
            if dt > cfg.KALMAN_STALE_S:
                eso = observer(float(omega))
                eso.reset()
                eso.z2 = 7
                assert eso.step(float(dt), 10000, -40) == 0
                assert eso.z1 == 10000
                continue
            steps = max(1, math.ceil(dt * omega / 0.5))
            t = dt / steps
            h = t * omega
            assert 0 < h <= 0.5 < bound
            matrix = np.array(
                [[1 - 2 * h, (1 - 2 * h) * t / C], [-t * C * omega**2, 1 - h**2]]
            )
            assert max(abs(np.linalg.eigvals(matrix))) < 1
            eso = observer(float(omega))
            eso.reset()
            x = 0.0
            # Run enough physical time to cover the slowest allowed bandwidth.
            for _ in range(math.ceil(20 / dt)):
                x += dt * (8 - 3) / C
                eso.step(float(dt), x, 8)
            assert eso.z2 == pytest.approx(-3, abs=0.003)

    def radius(h: float) -> float:
        m = np.array([[1 - 2 * h, (1 - 2 * h) * h], [-h, 1 - h * h]])
        return float(max(abs(np.linalg.eigvals(m))))

    assert radius(bound - 1e-6) < 1 < radius(bound + 1e-6)


@pytest.mark.parametrize(
    "values",
    [
        (0, 4, 80, 0.15),
        (1, 0, 80, 0.15),
        (1, 4, -1, 0.15),
        (1, 4, 80, 0),
        (math.nan, 4, 80, 0.15),
    ],
)
def test_invalid_observer_parameters(values: tuple[float, float, float, float]) -> None:
    b0, omega, limit, max_dt = values
    with pytest.raises(ValueError):
        FirstOrderESO(b0, omega, limit, max_dt=max_dt)


@pytest.mark.parametrize(
    "dt,y,u",
    [(0, 0, 0), (-1, 0, 0), (math.inf, 0, 0), (DT, math.nan, 0), (DT, 0, math.inf)],
)
def test_invalid_observer_sample(dt: float, y: float, u: float) -> None:
    with pytest.raises(ValueError):
        observer().step(dt, y, u)
    with pytest.raises(ValueError):
        observer().reset(math.nan)


class CaptureMPC(ForceMPC):
    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self.H = np.empty((0, 0))
        self.g = np.empty(0)

        def capture(
            H: NDArray[np.float64], g: NDArray[np.float64], limit: float
        ) -> Any:
            self.H, self.g = H.copy(), g.copy()
            return ForceMPC._bounded_qp(H, g, limit)

        object.__setattr__(self, "_bounded_qp", capture)


def mpc(mode: str, **kwargs: Any) -> CaptureMPC:
    return CaptureMPC(1 / cfg.MPC_HZ, c_drag=C, effort_mode=mode, **kwargs)


def centered_objective(
    force: NDArray[np.float64],
    controller: CaptureMPC,
    x0: float,
    d: float,
    r: NDArray[np.float64],
    vr: NDArray[np.float64],
    previous: float,
) -> float:
    L = controller.a * np.tril(np.ones((3, 3)))
    D = np.eye(3) - np.eye(3, k=-1)
    b = np.array([previous, 0, 0])
    position = x0 + controller.a * np.arange(1, 4) * d + L @ force
    return float(
        controller.wp * np.sum((position - r) ** 2)
        + controller.wv * np.sum(((force + d) / C - vr) ** 2)
        + controller.wu * np.sum((force - (C * vr - d)) ** 2)
        + controller.wd * np.sum((D @ force - b) ** 2)
    )


def test_absolute_parent_golden_binary() -> None:
    data = json.loads(
        (Path(__file__).parent / "fixtures/mpc_absolute_8549352.json").read_text(
            encoding="utf8"
        )
    )
    assert data["checkpoint"] == "8549352e492e4d9cde2cc4142001fcada7ff994a"
    for case in data["cases"]:
        controller = CaptureMPC(**case["params"], effort_mode="absolute")
        result = controller.compute(**case["input"])
        for key, expected in case["output_hex"].items():
            value = result[key]
            raw = (
                value.tobytes()
                if isinstance(value, np.ndarray)
                else struct.pack("d", value)
            )
            assert raw.hex() == expected, (case["input"], key)
        assert controller.H.tobytes().hex() == case["H_hex"]
        assert controller.g.tobytes().hex() == case["g_hex"]


def test_centered_quadratic_derivation_and_finite_difference() -> None:
    rng = np.random.default_rng(61006)
    for _ in range(20):
        r, vr = rng.normal(size=3), rng.normal(size=3)
        x0, d, previous = map(float, rng.normal(size=3))
        absolute, centered = mpc("absolute"), mpc("steady_state")
        absolute.compute(x0, d, r, vr, previous)
        out = centered.compute(x0, d, r, vr, previous)
        ss = C * vr - d
        assert absolute.H.tobytes() == centered.H.tobytes()
        np.testing.assert_allclose(
            centered.g, absolute.g - centered.wu * ss, rtol=0, atol=0
        )
        objective = partial(
            centered_objective,
            controller=centered,
            x0=x0,
            d=d,
            r=r,
            vr=vr,
            previous=previous,
        )

        force = rng.normal(size=3)
        gradient = 2 * (centered.H @ force + centered.g)
        eps = 1e-5
        finite = np.array(
            [
                (objective(force + eps * e) - objective(force - eps * e)) / (2 * eps)
                for e in np.eye(3)
            ]
        )
        np.testing.assert_allclose(gradient, finite, atol=1e-9)
        assert out["cost"] == pytest.approx(objective(out["F_seq"]), abs=1e-12)


@pytest.mark.parametrize("velocity,d", [(1, 0), (-1, 3), (1, -4), (0, 7)])
def test_correct_disturbance_force_fixed_point(velocity: float, d: float) -> None:
    controller = mpc("steady_state")
    expected = C * velocity - d
    out = controller.compute(
        2, d, 2 + controller.dt * velocity * np.arange(1, 4), [velocity] * 3, expected
    )
    np.testing.assert_allclose(out["F_seq"], expected, rtol=0, atol=1e-13)


@pytest.mark.parametrize("velocity,d", [(100, -80), (-100, 80)])
def test_center_outside_force_box(velocity: float, d: float) -> None:
    controller = mpc("steady_state")
    out = controller.compute(
        0, d, controller.dt * velocity * np.arange(1, 4), [velocity] * 3
    )
    assert np.all(np.isfinite(out["F_seq"])) and math.isfinite(out["cost"])
    assert np.max(abs(out["F_seq"])) <= cfg.MPC_FMAX_UN
    assert out["F0"] == math.copysign(cfg.MPC_FMAX_UN, velocity)


@pytest.mark.parametrize(
    "profile,kappa", [("defaults", 0.8224296694545196), ("saved", 0.6633151421575442)]
)
def test_current_kappa_and_centered_no_steady_bias(profile: str, kappa: float) -> None:
    params = {}
    if profile == "saved":
        settings = json.loads(
            (Path(__file__).resolve().parents[1] / "gui_settings.json").read_text(
                encoding="utf8"
            )
        )
        params = {
            "w_pos": settings["mpc_w_pos"],
            "w_vel": settings["mpc_w_vel"],
            "w_u": settings["mpc_w_u"],
            "w_delta": settings["mpc_w_du"],
            "fmax": settings["mpc_fmax"],
            "horizon": settings["mpc_horizon"],
        }
    for mode, expected in (("absolute", kappa), ("steady_state", 1.0)):
        controller = mpc(mode, **params)
        refs = controller.dt * np.arange(1, 4)
        force = 0.0
        for _ in range(80):
            force = controller.compute(0, 0, refs, np.ones(3), force)["F0"]
        assert force / C == pytest.approx(expected, abs=1e-13)
    with pytest.raises(ValueError):
        mpc("unknown")
