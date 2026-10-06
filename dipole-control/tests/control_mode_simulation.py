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
    dropout_fraction: float = 0.0,
    profile: str = "defaults",
    target_box_delta: int = 9,
    interpolation_mode: str = "legacy_three_frame",
    seed: int = 20261006,
    path_mm: list[tuple[float, float]] | None = None,
    allow_finish: bool = False,
    plant_substeps: int = 5,
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
    widget.combo_target_box.setCurrentText(str(target_box_delta))
    widget.combo_interpolation.setCurrentText(interpolation_mode)
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
    widget.path_px = (
        [widget.world_mm_to_px(p) for p in path_mm]
        if path_mm is not None
        else [start_px, widget.world_mm_to_px((9, 0)), widget.world_mm_to_px((9, 4))]
    )
    widget.start_tracking()
    assert widget.worker is not None
    worker = widget.worker
    # No floating wall-clock is used by the harness; production stop/freshness is.
    rng = np.random.default_rng(seed)
    dropout_rng = np.random.default_rng(seed + 1)
    physical_current = np.zeros(6)
    dt = 1 / cfg.CONTROL_HZ
    trace = []
    completion_time: float | None = None
    for tick in range(ticks):
        clock.now = 1000 + tick * dt
        position = plant.pos * 1e3
        missing = (vision_gap and 60 <= tick < 69) or (
            dropout_fraction > 0 and dropout_rng.random() < dropout_fraction
        )
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
        if not (widget.tracking and not widget.stopping):
            assert allow_finish and widget.shared.get_progress().finished
            completion_time = tick * dt
            break  # This tick requested normal_stop and sent no tracking frame.
        snapshot = widget.shared.get_I_target()[3]
        target = snapshot["F_target"].copy()
        diagnostic_force = widget.last_F_actual.copy() * 1e6
        disturbances = widget.z3.copy()
        cmd = np.asarray(widget.last_sent_cmd)
        assert np.max(abs(cmd)) <= cfg.DEFAULT_CMD_LIMIT
        # Independent R-L integration and nonlinear field-force at actual plant position.
        sub_dt = dt / plant_substeps
        a = math.exp(-cfg.R_COIL_OHM * sub_dt / cfg.L_COIL_H)
        for _ in range(plant_substeps):
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
                "I_est_A": widget.executor.I_est.tolist(),
                "I_physical_A": physical_current.tolist(),
                "F_physical_uN": (np.atleast_1d(fm["F"]) * 1e6).tolist(),
                "current_gain": widget.solver.current_gain,
                "tracking": widget.tracking,
                "target_cmd": (
                    snapshot["currents"] / widget.solver.current_gain
                ).tolist(),
                "solver_ms": float(snapshot["solver_time_ms"]),
                "s_progress": float(widget.shared.get_progress().s_progress),
                "target_box_delta": target_box_delta,
                "current_constraint_active": bool(
                    (snapshot["rec"] or {}).get("current_constraint_active", False)
                ),
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
        if completion_time is not None:
            break
    count = len(trace)
    tail = trace[count // 2 :]
    positions = np.array([r["pos_mm"] for r in trace])
    speeds = np.array([r["velocity_mm_s"] for r in tail])
    commands = np.array([r["cmd"] for r in trace])
    errors = positions - np.column_stack(
        [-4 + dt * np.arange(1, count + 1), np.zeros(count)]
    )
    if path_mm is not None:
        nodes = np.asarray(path_mm, float)
        lengths = np.linalg.norm(np.diff(nodes, axis=0), axis=1)
        arcs = np.r_[0, np.cumsum(lengths)]
        reference = np.column_stack(
            [
                np.interp(dt * np.arange(1, count + 1), arcs, nodes[:, axis])
                for axis in (0, 1)
            ]
        )
        errors = positions - reference
    changes = np.diff(commands, axis=0)
    reversals = (changes[1:] * changes[:-1]) < 0
    published = [
        r for i, r in enumerate(trace) if i == 0 or r["seq"] != trace[i - 1]["seq"]
    ]
    target_changes = np.diff(np.array([r["target_cmd"] for r in published]), axis=0)
    solver_times = np.array([r["solver_ms"] for r in published])
    path_speed = float(speeds[:, 0].mean())
    if path_mm is not None:
        directions = np.diff(nodes, axis=0) / lengths[:, None]
        segments = np.clip(
            np.searchsorted(arcs, [r["s_progress"] for r in tail], side="right") - 1,
            0,
            len(directions) - 1,
        )
        path_speed = float(np.mean(np.sum(speeds * directions[segments], axis=1)))
    result = {
        "target_box_delta": target_box_delta,
        "interpolation_mode": interpolation_mode,
        "seed": seed,
        "plant_substeps": plant_substeps,
        "completion_time_s": completion_time,
        "executed_ticks": count,
        "path_mm": path_mm,
        "max_target_delta_cmd": float(np.max(abs(target_changes))),
        "solver_mean_ms": float(solver_times.mean()),
        "solver_p95_ms": float(np.percentile(solver_times, 95)),
        "final_s_progress_mm": float(trace[-1]["s_progress"]),
        "constraint_active_ratio": float(
            np.mean([r["current_constraint_active"] for r in published])
        ),
        "max_tracking_error_mm": float(np.max(np.linalg.norm(errors, axis=1))),
        "max_ahead_reference_mm": float(np.max(errors[:, 0])),
        "mode": mode,
        "scenario": scenario,
        "profile": profile,
        "ticks": ticks,
        "noise": noise,
        "vision_gap": vision_gap,
        "mean_path_speed_mm_s": path_speed,
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
