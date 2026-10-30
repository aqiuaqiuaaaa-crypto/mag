"""Real AUTO_TRACK ticks, independent plant, dropped camera frames, memory UART."""

from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import config as cfg
import gui_test_support as gui_tests
import magnetic_dipole_pid as gui
from bead_sim import BeadSimulator, FluidModel, FrictionModel, MagnetModel
from control_mode_simulation import SimulationClock, configure_mode
from multirate import ControlWorker
from test_gui_control_log import configure_camera
from test_path_stall_gaps import RATES, periodic_missing

app = gui_tests.app
ports = gui_tests.ports
window = gui_tests.window


def run_closed_loop(widget, monkeypatch, rate, *, blocked=False, long_loss=False):
    # Use the saved profile verbatim; no production control parameters are tuned.
    settings = json.loads(
        (Path(gui.__file__).parent / "gui_settings.json").read_text(encoding="utf8")
    )
    for key, value in settings.items():
        item = widget._settings_items().get(key)
        if item is not None and not isinstance(item, list):
            if key == "model_path":
                value = str(Path(gui.__file__).parent / value)
            widget._widget_set(item, value)
    widget._sync_physics_from_ui()
    widget._reload_solver()
    assert widget.model_ok
    configure_mode(widget, "L3")
    plant = BeadSimulator(
        magnet=MagnetModel(
            widget.spin_beadD.value(), widget.spin_br.value(), widget.spin_rho_b.value()
        ),
        fluid=FluidModel(widget.viscosity_mPas),
        friction=FrictionModel(0.0),
        pos0_mm=(-4, 0),
    )
    clock = SimulationClock()
    widget.control_clock = clock
    monkeypatch.setattr(gui.time, "time", clock)
    monkeypatch.setattr(ControlWorker, "start", lambda self: None)
    widget.ser = gui_tests.MemorySerial()
    port = widget.ser
    camera = configure_camera(widget)
    widget.combo_estimator.setCurrentText("KALMAN")
    widget.kf.reset(-4, 0)
    widget.state_pos_mm = np.array([-4.0, 0.0])
    widget.state_vel_mm = np.zeros(2)
    widget.last_sent_cmd = [0] * 6
    widget.last_F_actual = np.zeros(3)
    widget.bead = (*widget.world_mm_to_px((-4, 0)), 100)
    path = (
        [(-4, 0), (5, 0), (5, 4)] if blocked else [(-4, 0), (-2, 0), (-2, 2), (-4, 2)]
    )
    widget.path_px = [widget.world_mm_to_px(p) for p in path]
    widget.start_tracking()
    worker = widget.worker
    assert worker is not None
    measured = np.array([-4.0, 0.0])

    def detect(*_):
        return (*widget.world_mm_to_px(measured), 100, None)

    monkeypatch.setattr(gui, "detect_bead", detect)
    dt = 1 / cfg.CONTROL_HZ
    widget.last_time = clock.now - dt
    rng = np.random.default_rng(20261006)
    physical_current = np.zeros(6)
    trace = []
    stopped_at = None
    stall_at = None
    armed_samples = 0
    approached = False
    for tick in range(900):
        clock.now = 1000 + tick * dt
        missing = tick >= 90 if long_loss else periodic_missing(tick, rate)
        camera.fail = missing
        measured = plant.pos * 1e3 + rng.normal(0, math.sqrt(cfg.KALMAN_R), 2)
        previous = np.asarray(widget.last_sent_cmd)
        widget.tick()  # mode, observations, ESO, Kalman, safety and executor
        if widget.tracking:
            assert widget.mode == "AUTO_TRACK"
            params = widget.shared.get_params()
            assert params["mpc_on"] and params["path_stall_active"]
            if tick % 3 == 0:
                worker._step()  # actual MPC and inverse, only scheduling is synthetic
            watchdog = widget.shared._path_stall
            if watchdog.anchor_pos is not None:
                armed_samples += 1
        elif stopped_at is None:
            stopped_at = tick * dt
        if widget.shared.path_stall_timed_out() and stall_at is None:
            stall_at = tick * dt
        progress = widget.shared.get_progress()
        approached |= progress.approach_s is not None
        cmd = np.asarray(widget.last_sent_cmd)
        assert np.max(np.abs(cmd - previous)) <= 9
        assert np.max(np.abs(cmd)) <= settings["current_max"]
        # Existing independent R-L/field/plant model at 150 Hz. A blocked bead
        # is an explicitly imposed mechanical clamp, never a fake controller.
        sub_dt = dt / 5
        a = math.exp(-cfg.R_COIL_OHM * sub_dt / cfg.L_COIL_H)
        for _ in range(5):
            physical_current = (
                a * physical_current + (1 - a) * cmd * widget.solver.current_gain
            )
            plant.step(physical_current, sub_dt)
            if blocked:
                plant.pos[:] = [-0.004, 0]
                plant.vel[:] = 0
        trace.append(
            {
                "time_s": tick * dt,
                "missing": missing,
                "mode": widget.mode,
                "watchdog_enabled": bool(
                    widget.shared.get_params().get("path_stall_active")
                ),
                "elapsed_s": widget.shared._path_stall.elapsed_s,
                "s_progress": progress.s_progress,
                "holding_endpoint": progress.holding_endpoint,
                "finished": progress.finished,
                "stale_status": widget.stale_status,
                "stall": widget.shared.path_stall_timed_out(),
                "position_mm": (plant.pos * 1e3).tolist(),
                "velocity_mm_s": (plant.vel * 1e3).tolist(),
                "command": cmd.tolist(),
            }
        )
        if stopped_at is not None and tick * dt >= stopped_at + 1:
            break
    result = {
        "rate": rate,
        "blocked": blocked,
        "long_loss": long_loss,
        "stall_at": stall_at,
        "stopped_at": stopped_at,
        "armed_samples": armed_samples,
        "approached": approached,
        "finished": widget.shared.get_progress().finished,
        "final_status": widget.stale_status,
        "final_mode": widget.mode,
        "final_command": widget.last_sent_cmd,
        "actual_missing": sum(row["missing"] for row in trace),
        "trace": trace,
    }
    evidence = os.environ.get("MAG_F1_EVIDENCE_DIR")
    if evidence:
        directory = Path(evidence)
        directory.mkdir(parents=True, exist_ok=True)
        name = f"{'loss' if long_loss else 'blocked' if blocked else 'moving'}-{rate}"
        (directory / f"{name}.json").write_text(
            json.dumps(result, indent=2), encoding="utf8"
        )
        (directory / f"{name}.uart").write_bytes(b"".join(port.frames))
    assert armed_samples > 10  # prevent a vacuous IDLE/disabled-watchdog pass
    return result


@pytest.mark.parametrize("rate", RATES)
def test_blocked_closed_loop_with_periodic_dropouts_stops_in_bounded_time(
    window, monkeypatch, rate
):
    result = run_closed_loop(window, monkeypatch, rate, blocked=True)
    assert result["stall_at"] is not None
    # Each isolated lost GUI frame excludes at most two 10 Hz intervals.
    # Include three worker periods for schedule endpoints/quantization. This
    # bounds wall time without pretending missing time is stationary evidence.
    period = 1 / cfg.MPC_HZ
    bound = (12 + 3 * period) / (1 - 2 * period * cfg.CONTROL_HZ * rate)
    assert 12 <= result["stall_at"] <= bound < 18
    assert not result["finished"]
    assert result["final_status"] == "PATH_STALL_TIMEOUT"
    assert result["final_command"] == [0] * 6 and result["final_mode"] == "IDLE"
    assert rate == 0 or result["actual_missing"] > 0


@pytest.mark.parametrize("rate", RATES)
def test_moving_closed_loop_turns_and_decelerates_without_false_stall(
    window, monkeypatch, rate
):
    result = run_closed_loop(window, monkeypatch, rate)
    assert result["stall_at"] is None
    assert result["approached"] and result["finished"]
    assert result["final_command"] == [0] * 6 and result["final_mode"] == "IDLE"
    assert rate == 0 or result["actual_missing"] > 0


def test_long_camera_loss_uses_existing_freshness_stop(window, monkeypatch):
    result = run_closed_loop(window, monkeypatch, 0, blocked=True, long_loss=True)
    assert result["stall_at"] is None and not result["finished"]
    assert result["stopped_at"] is not None and 4 <= result["stopped_at"] <= 4.3
    assert any(row["stale_status"] == "STALE_INPUT" for row in result["trace"])
    assert result["final_command"] == [0] * 6 and result["final_mode"] == "IDLE"
