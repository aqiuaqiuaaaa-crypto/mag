"""Deterministic hardware-free first-order plant through the actual GUI chain.

No alternate controller/solver/executor is implemented. Only physical time,
camera position and UART are supplied by the harness. The independent plant uses
BeadSimulator's existing Coulomb law and DipoleSolver, with exact configured R-L
lag at 150 Hz. Drag changes are declared synthetic uncertainty cases.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import config as cfg
import magnetic_dipole_pid as gui
import numpy as np
from bead_sim import BeadSimulator, FluidModel, FrictionModel, MagnetModel
from multirate import CurrentExecutor, SharedState

MODES = {
    "L0": ("legacy", "absolute", True),
    "L1": ("legacy", "absolute", False),
    "L2": ("first_order", "steady_state", False),
    "L3": ("first_order", "steady_state", True),
}
SCENARIOS = {
    "nominal": (1.0, 0.0),
    "friction": (1.0, 0.10),  # existing BeadSimulator default, not a fitted coefficient
    "drag": (1.5, 0.0),
    "friction_drag": (1.5, 0.10),
}


@dataclass
class SimulationClock:
    now: float = 1000.0

    def __call__(self) -> float:
        return self.now


class SimulationSerial:
    def __init__(self) -> None:
        self.frames: list[bytes] = []

    def write(self, frame: bytes) -> int:
        self.frames.append(frame)
        return len(frame)

    def close(self) -> None:
        pass


def configure_mode(widget: gui.MagneticDipoleControl, mode: str) -> None:
    eso, effort, enabled = MODES[mode]
    widget.combo_eso.setCurrentText(eso)
    widget.combo_effort.setCurrentText(effort)
    widget.chk_eso.setChecked(enabled)
    widget._reset_controllers()


def run_simulation(
    widget: gui.MagneticDipoleControl,
    mode: str,
    scenario: str,
    *,
    ticks: int = 240,
    noise: bool = True,
    vision_gap: bool = False,
    profile: str = "defaults",
) -> dict[str, Any]:
    """Return metrics and per-tick trace; caller disables real worker scheduling."""
    factor, friction = SCENARIOS[scenario]
    if profile == "saved":
        # Reproduce tracked GUI startup overrides, without writing real settings.
        settings = json.loads(
            (Path(gui.__file__).parent / "gui_settings.json").read_text(encoding="utf8")
        )
        items = widget._settings_items()
        for key, value in settings.items():
            item = items.get(key)
            if item is not None and not isinstance(item, list):
                if key == "model_path":
                    value = str(Path(gui.__file__).parent / value)
                widget._widget_set(item, value)
        widget._sync_physics_from_ui()
        widget._reload_solver()
        assert widget.model_ok
    plant = BeadSimulator(
        magnet=MagnetModel(
            widget.spin_beadD.value(), widget.spin_br.value(), widget.spin_rho_b.value()
        ),
        fluid=FluidModel(widget.viscosity_mPas * factor),
        friction=FrictionModel(friction),
        pos0_mm=(-4, 0),
    )
    clock = SimulationClock()
    port = SimulationSerial()
    configure_mode(widget, mode)
    widget.control_clock = clock
    widget.shared = SharedState(clock=clock)
    widget.executor = CurrentExecutor(clock=clock)
    widget.ser = port
    widget.frame_size = (1920, 1080)
    widget.spin_vieww.setValue(cfg.VIEW_WIDTH_MM)
    widget.combo_estimator.setCurrentText("KALMAN")
    widget.last_sent_cmd = [0] * 6
    widget.last_F_actual = np.zeros(3)
    widget.kf.reset(-4, 0)
    widget.state_pos_mm = np.array([-4.0, 0.0])
    widget.state_vel_mm = np.zeros(2)
    start_px = widget.world_mm_to_px((-4, 0))
    widget.bead = (*start_px, 100)
    # Actual start_tracking supplies lead, reference, params, shared and worker.
    widget.path_px = [
        start_px,
        widget.world_mm_to_px((9, 0)),
        widget.world_mm_to_px((9, 4)),
    ]
    widget.start_tracking()
    assert widget.worker is not None
    worker = widget.worker
    # No floating wall-clock is used by the harness; production stop/freshness is.
    rng = np.random.default_rng(20261006)
    physical_current = np.zeros(6)
    dt = 1 / cfg.CONTROL_HZ
    trace = []
    for tick in range(ticks):
        clock.now = 1000 + tick * dt
        position = plant.pos * 1e3
        missing = vision_gap and 60 <= tick < 69
        if missing:
            widget.bead = None
        else:
            measured = position + (
                rng.normal(0, math.sqrt(cfg.KALMAN_R), 2) if noise else 0
            )
            widget.bead = (*widget.world_mm_to_px(measured), 100)
        widget.estimate_state(dt, not missing)
        # Same asynchronous ordering every cycle: worker consumes prior snapshot.
        if tick % 3 == 0:
            worker._step()
        previous_force = widget.last_F_actual.copy() * 1e6
        widget.mpc_track_step(dt)
        assert widget.tracking and not widget.stopping
        snapshot = widget.shared.get_I_target()[3]
        target = snapshot["F_target"].copy()
        diagnostic_force = widget.last_F_actual.copy() * 1e6
        disturbances = widget.z3.copy()
        cmd = np.asarray(widget.last_sent_cmd)
        assert np.max(abs(cmd)) <= cfg.DEFAULT_CMD_LIMIT
        # Independent R-L integration and nonlinear field-force at actual plant position.
        sub_dt = dt / 5
        a = math.exp(-cfg.R_COIL_OHM * sub_dt / cfg.L_COIL_H)
        for _ in range(5):
            physical_current = (
                a * physical_current + (1 - a) * cmd * widget.solver.current_gain
            )
            plant.step(physical_current, sub_dt)
        actual_velocity = plant.vel * 1e3
        # Lumped force relative to the nominal model; includes synthetic excess drag.
        fm = plant.solver.forward_model(plant.pos_m3(), physical_current)
        force_xy = np.atleast_1d(fm["F"])[:2] * 1e6
        true_d = widget._drag_c_uN() * actual_velocity - force_xy
        trace.append(
            {
                "tick": tick,
                "time_s": (tick + 1) * dt,
                "pos_mm": (plant.pos * 1e3).tolist(),
                "velocity_mm_s": actual_velocity.tolist(),
                "d_hat_uN": disturbances.tolist(),
                "d_true_uN": true_d.tolist(),
                "F_target_uN": target.tolist(),
                "F_est_uN": diagnostic_force[:2].tolist(),
                "u_eso_uN": previous_force[:2].tolist(),
                "cmd": cmd.tolist(),
                "seq": snapshot["seq"],
                "alpha": (widget.last_diag or {}).get("interp_alpha"),
                "worker_status": widget.shared.get_freshness()["worker_status"],
                "target_status": widget.shared.get_freshness()["target_status"],
                "missing": missing,
                "solver_converged": bool(
                    (snapshot["rec"] or {}).get("converged", False)
                ),
            }
        )
    tail = trace[ticks // 2 :]
    positions = np.array([r["pos_mm"] for r in trace])
    speeds = np.array([r["velocity_mm_s"] for r in tail])
    commands = np.array([r["cmd"] for r in trace])
    errors = positions - np.column_stack(
        [-4 + dt * np.arange(1, ticks + 1), np.zeros(ticks)]
    )
    changes = np.diff(commands, axis=0)
    reversals = (changes[1:] * changes[:-1]) < 0
    result = {
        "mode": mode,
        "scenario": scenario,
        "profile": profile,
        "ticks": ticks,
        "noise": noise,
        "vision_gap": vision_gap,
        "mean_path_speed_mm_s": float(speeds[:, 0].mean()),
        "tracking_rms_mm": float(np.sqrt(np.mean(np.sum(errors**2, axis=1)))),
        "cross_track_rms_mm": float(np.sqrt(np.mean(positions[:, 1] ** 2))),
        "d_hat_x_uN": float(np.mean([r["d_hat_uN"][0] for r in tail])),
        "d_true_x_uN": float(np.mean([r["d_true_uN"][0] for r in tail])),
        "F_target_x_uN": float(np.mean([r["F_target_uN"][0] for r in tail])),
        "F_est_x_uN": float(np.mean([r["F_est_uN"][0] for r in tail])),
        "force_saturation_ratio": float(
            np.mean(
                [
                    np.linalg.norm(r["F_target_uN"])
                    >= widget.spin_mpc_fmax.value() - 1e-9
                    for r in trace
                ]
            )
        ),
        "command_limit_ratio": float(
            np.mean(np.any(abs(commands) >= cfg.DEFAULT_CMD_LIMIT, axis=1))
        ),
        "speed_std_mm_s": float(speeds[:, 0].std()),
        "max_delta_cmd": int(np.max(abs(changes))),
        "command_reversal_ratio": float(np.mean(reversals)),
        "command_frames": len(port.frames),
        "worker_publishes": int(trace[-1]["seq"]),
        "diverged": not bool(
            np.all(np.isfinite(positions)) and np.max(abs(positions)) < 20
        ),
        "trace": trace,
    }
    return result
