"""ON/OFF actual UART, acceptance races, stale/stop and identical R-L checks."""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import config as cfg
import magnetic_dipole_pid as gui
import multirate
import test_gui_curt_telemetry as telemetry_tests
import test_gui_freshness as freshness_tests
import test_gui_path_progress as progress_tests
from dipole_solver import DipoleSolver
from interpolation_analysis import MODES, PARENT, command_metrics, evidence
from multirate import CurrentExecutor, SharedState
from test_freshness import FakeClock, FakeSolver, setup_worker
from test_gui_control_log import finish_log, rows, start_log
from test_l0_checkpoint_replay import exact, replay_checkpoint
from test_target_box import frame_commands

app = telemetry_tests.app
ports = telemetry_tests.ports
window = telemetry_tests.window


def prepare(
    widget: gui.MagneticDipoleControl, mode: str, limit: int = 99
) -> tuple[FakeClock, SharedState, CurrentExecutor, telemetry_tests.MemorySerial]:
    clock = FakeClock()
    shared = SharedState(clock=clock)
    executor = CurrentExecutor(clock=clock, interpolation_mode=mode)
    port = telemetry_tests.MemorySerial()
    widget.ser = port
    widget.last_sent_cmd = [0] * 6
    widget.spin_cmd_max.setValue(limit)
    # Property/step tests isolate the true executor/sender, not the magnetic plant.
    widget.solver = FakeSolver()
    return clock, shared, executor, port


def frame(
    widget: gui.MagneticDipoleControl,
    shared: SharedState,
    executor: CurrentExecutor,
    port: telemetry_tests.MemorySerial,
    limit: int = 99,
) -> dict[str, Any]:
    previous = list(widget.last_sent_cmd)
    diag = executor.step(
        shared, 1 / 30, np.zeros(3), widget.solver, previous, max_cmd=limit
    )
    widget.send_commands(diag["cmd"])
    actual = frame_commands(port.frames[-1])
    assert actual == widget.last_sent_cmd == diag["cmd"]
    assert max(abs(b - a) for a, b in zip(previous, actual)) <= 9
    assert max(map(abs, actual)) <= limit
    assert np.all(np.isfinite(diag["I_est"]))
    return diag


def publish(shared: SharedState, target: Any) -> bool:
    return shared.set_I_target(
        np.asarray(target) * cfg.CMD_TO_A,
        None,
        [0, 0],
        0,
        0,
        0,
        expected_seq=shared.get_I_target()[1],
        input_mono=shared.clock(),
    )


@pytest.mark.parametrize("mode", MODES)
@pytest.mark.parametrize(
    "step", [sign * size for sign in (1, -1) for size in (1, 3, 6, 9, 12, 18, 27)]
)
def test_step_actual_uart(
    window: gui.MagneticDipoleControl,
    monkeypatch: pytest.MonkeyPatch,
    mode: str,
    step: int,
) -> None:
    clock, shared, executor, port = prepare(window, mode)
    captured: list[Any] = []
    original = multirate.apply_slew_cmd

    def capture(value: Any, *args: Any, **kwargs: Any) -> Any:
        captured.append(np.asarray(value).copy() / cfg.CMD_TO_A)
        return original(value, *args, **kwargs)

    monkeypatch.setattr(multirate, "apply_slew_cmd", capture)
    assert publish(shared, [step] * 6)
    trace = []
    for index in range(6):
        clock.now = index / 30
        diag = frame(window, shared, executor, port)
        trace.append(
            {
                "frame": index + 1,
                "scheduled_send_time_ms": clock.now * 1000,
                "target": [step] * 6,
                "candidate": captured[-1].tolist(),
                "post_slew": diag["cmd"],
                "sent": frame_commands(port.frames[-1]),
                "alpha": diag["interp_alpha"],
            }
        )
    sent = np.array([r["sent"][0] for r in trace])
    legacy = {
        1: [0, 1, 1],
        3: [1, 2, 3],
        6: [2, 4, 6],
        9: [3, 6, 9],
        12: [4, 8, 12],
        18: [6, 12, 18],
        27: [9, 18, 27],
    }
    expected = (
        legacy[abs(step)]
        if mode == MODES[0]
        else [min(abs(step), 9 * n) for n in (1, 2, 3)]
    )
    assert sent[:3].tolist() == [int(math.copysign(n, step)) for n in expected]
    result: dict[str, Any] = {
        "mode": mode,
        "step": step,
        "sent": sent.tolist(),
        "trace": trace,
        "max_delta": int(max(abs(np.diff(np.r_[0, sent])))),
    }
    for name, fraction in (
        ("first_ms", 1 / abs(step)),
        ("t50_ms", 0.5),
        ("t90_ms", 0.9),
        ("t100_ms", 1.0),
    ):
        result[name] = float(
            (np.flatnonzero(abs(sent) >= abs(step) * fraction)[0] + 1) / 30 * 1000
        )
        result[name.replace("_ms", "_send_tick_ms")] = result[name] - 1000 / 30
    result["timing_convention"] = (
        "Frame-end budget; send_tick subtracts one dt, acceptance and first write share injected tick zero. Real scheduling/UART latency is not measured."
    )
    evidence(f"step-{mode}-{step}", result)


SEQUENCES = {
    "A": [[n] * 6 for n in (0, 6, 12, 18, 24, 27)],
    "B": [[n] * 6 for n in (0, 27, 54, 81)],
    "C": [[n] * 6 for n in (0, 18, -18, 18)],
    "D": [[0] * 6] + [[n] * 6 for n in (3, -3)] * 50,
    "E": [
        [0] * 6,
        [1, -3, 6, -9, 12, -27],
        [-3, 6, -9, 18, -12, 27],
        [6, -9, 18, -27, 3, -1],
    ],
}


@pytest.mark.parametrize("mode", MODES)
@pytest.mark.parametrize("sequence", SEQUENCES)
def test_continuous_targets(
    window: gui.MagneticDipoleControl, mode: str, sequence: str
) -> None:
    clock, shared, executor, port = prepare(window, mode)
    trace = []
    for target in SEQUENCES[sequence]:
        assert publish(shared, target)
        anchor = np.asarray(window.last_sent_cmd) * cfg.CMD_TO_A
        for phase in range(3):
            clock.now += 1 / 30
            diag = frame(window, shared, executor, port)
            np.testing.assert_array_equal(executor.I_from, anchor)
            trace.append(
                {
                    "target": target,
                    "sent": diag["cmd"],
                    "seq": executor.seq,
                    "phase": executor.frames_since,
                    "alpha": diag["interp_alpha"],
                }
            )
    evidence(
        f"continuous-{sequence}-{mode}",
        {
            "mode": mode,
            "sequence": sequence,
            "frames": len(trace),
            **command_metrics([r["sent"] for r in trace], [r["target"] for r in trace]),
            "sent": [r["sent"] for r in trace] if sequence != "D" else [],
            "trace": trace,
        },
    )


@pytest.mark.parametrize("mode", MODES)
def test_early_seq_reanchors_actual_no_rebound(
    window: gui.MagneticDipoleControl, mode: str
) -> None:
    clock, shared, executor, port = prepare(window, mode)
    for target in ([27] * 6, [-18] * 6, [6] * 6, [-3] * 6):
        previous = np.asarray(window.last_sent_cmd)
        assert publish(shared, target)
        clock.now += 1 / 30
        frame(window, shared, executor, port)
        np.testing.assert_array_equal(executor.I_from, previous * cfg.CMD_TO_A)
        assert executor.frames_since == 0
        assert all(
            min(a, b) <= c <= max(a, b)
            for a, b, c in zip(previous, target, window.last_sent_cmd)
        )
    for _ in range(4):
        clock.now += 1 / 30
        frame(window, shared, executor, port)
    assert window.last_sent_cmd == [-3] * 6


@pytest.mark.parametrize("mode", MODES)
@pytest.mark.parametrize("limit", [50, 99])
def test_20000_random_targets_uart_and_guards(
    window: gui.MagneticDipoleControl, mode: str, limit: int
) -> None:
    clock, shared, executor, port = prepare(window, mode, limit)
    rng = np.random.default_rng(20261006 + limit)
    frames = stale_checks = race_checks = stops = 0
    for group in range(20000):
        clock.now += 1 / 30
        previous_seq = shared.get_I_target()[1]
        # Same seed/mode, targets center on the actual sent command per production box.
        target = np.clip(
            np.asarray(window.last_sent_cmd) + rng.integers(-27, 28, 6), -limit, limit
        )
        assert publish(shared, target)
        assert not shared.set_I_target(
            np.full(6, 99.0),
            None,
            [0, 0],
            0,
            0,
            0,
            expected_seq=previous_seq,
            input_mono=clock.now,
        )
        race_checks += 1
        last_seq = executor.seq
        for _ in range(int(rng.integers(1, 4))):
            frame(window, shared, executor, port, limit)
            assert executor.seq >= last_seq
            last_seq = executor.seq
            frames += 1
            clock.now += 1 / 30
        if group % 211 == 0:
            held = list(window.last_sent_cmd)
            phase = executor.frames_since
            seq = executor.seq
            clock.now += 0.35
            assert publish(shared, -target)
            clock.now += 0.35
            diag = frame(window, shared, executor, port, limit)
            frames += 1
            assert (
                diag["cmd"] == held
                and executor.seq == seq
                and executor.frames_since == phase
            )
            clock.now += 0.8
            diag = frame(window, shared, executor, port, limit)
            frames += 1
            assert diag["stop_requested"] and diag["cmd"] == held
            stale_checks += 2
        if group % 997 == 0:
            shared.stop()
            assert not publish(shared, [27] * 6)
            for _ in range((limit + 8) // 9):
                previous = list(window.last_sent_cmd)
                window.send_commands([0] * 6)
                assert (
                    max(abs(b - a) for a, b in zip(previous, window.last_sent_cmd)) <= 9
                )
                assert frame_commands(port.frames[-1]) == window.last_sent_cmd
                frames += 1
            assert window.last_sent_cmd == [0] * 6
            # Production tracking restart creates both objects afresh.
            shared = SharedState(clock=clock)
            executor = CurrentExecutor(clock=clock, interpolation_mode=mode)
            assert (
                executor.seq == -1
                and executor.frames_since == 0
                and not np.any(executor.I_est)
            )
            stops += 1
    evidence(
        f"random-{mode}-{limit}",
        {
            "mode": mode,
            "limit": limit,
            "groups": 20000,
            "frames": frames,
            "stale_checks": stale_checks,
            "overwrite_races_rejected": race_checks,
            "stop_restart_checks": stops,
            "violations": {
                "slew": 0,
                "amplitude": 0,
                "nan_inf": 0,
                "stale": 0,
                "seq_regression": 0,
                "post_stop_nonzero_restart": 0,
                "overwrite": 0,
            },
        },
    )


@pytest.mark.parametrize("mode", MODES)
@pytest.mark.parametrize("interruption", ["slow_solve", "newer_target", "stop"])
def test_worker_commit_guards_both_modes(
    monkeypatch: pytest.MonkeyPatch, mode: str, interruption: str
) -> None:
    clock, shared, worker = setup_worker()
    original = worker.solver.solve_field_force_pseudoinverse

    def interfere(*args: Any, **kwargs: Any) -> Any:
        rec = original(*args, **kwargs)
        if interruption == "slow_solve":
            clock.now = 0.20
        elif interruption == "newer_target":
            assert publish(shared, [-9] * 6)
        else:
            shared.stop()
        return rec

    monkeypatch.setattr(worker.solver, "solve_field_force_pseudoinverse", interfere)
    worker._step()
    assert shared.get_I_target()[1] == int(interruption == "newer_target")
    executor = CurrentExecutor(clock=clock, interpolation_mode=mode)
    diag = executor.step(shared, 1 / 30, np.zeros(3), worker.solver, [0] * 6)
    expected = (
        ([-3] * 6 if mode == MODES[0] else [-9] * 6)
        if interruption == "newer_target"
        else [0] * 6
    )
    assert diag["cmd"] == expected


@pytest.mark.parametrize("mode", MODES)
def test_executor_read_race_preserves_new_publication(
    monkeypatch: pytest.MonkeyPatch, mode: str
) -> None:
    clock = FakeClock()
    shared = SharedState(clock=clock)
    executor = CurrentExecutor(clock=clock, interpolation_mode=mode)
    publish(shared, [6] * 6)
    original = shared.get_I_target

    def concurrent_read() -> Any:
        captured = original()
        shared.set_I_target(np.full(6, -6 * cfg.CMD_TO_A), None, [0, 0], 0, 0, 0)
        return captured

    monkeypatch.setattr(shared, "get_I_target", concurrent_read)
    first = executor.step(shared, 1 / 30, np.zeros(3), FakeSolver(), [0] * 6)
    assert executor.seq == 1 and original()[1] == 2
    monkeypatch.setattr(shared, "get_I_target", original)
    executor.step(shared, 1 / 30, np.zeros(3), FakeSolver(), first["cmd"])
    assert executor.seq == 2 and original()[1] == 2


@pytest.mark.parametrize("mode", MODES)
@pytest.mark.parametrize(
    "case",
    [
        "stall",
        "vision",
        "vision_timeout",
        "stop",
        "emergency",
        "deviation",
        "restart",
        "mode_log",
    ],
)
def test_gui_compatibility(
    window: gui.MagneticDipoleControl,
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    mode: str,
    case: str,
) -> None:
    window.combo_interpolation.setCurrentText(mode)
    if case == "stall":
        freshness_tests.test_stalled_worker_holds_then_normal_stop_and_logs(
            window, monkeypatch, tmp_path
        )
    elif case == "vision":
        freshness_tests.test_vision_loss_stops_publication_and_can_recover(
            window, monkeypatch, tmp_path
        )
    elif case == "vision_timeout":
        freshness_tests.test_vision_loss_persistent_timeout_and_monotonic_stop(
            window, monkeypatch
        )
    elif case == "deviation":
        progress_tests.test_gui_fallback_or_normal_stop_for_path_deviation(
            window, monkeypatch, tmp_path, 3.5, True
        )
    elif case == "restart":
        progress_tests.test_tracking_restart_resets_progress_and_keeps_lead(
            window, monkeypatch
        )
    else:
        clock, port = freshness_tests.start_tracking(window, monkeypatch)
        executor = window.executor
        assert executor.interpolation_mode == mode
        if case == "mode_log":
            start_log(window, tmp_path / "mode.csv")
            # Setting changes apply to the next run; logged value is the actual mode.
            window.combo_interpolation.setCurrentText(
                MODES[1] if mode == MODES[0] else MODES[0]
            )
            for _ in range(3):
                clock.now += 1 / 30
                window.tick()
            assert window.executor is executor and executor.interpolation_mode == mode
            finish_log(window)
            assert window.control_logger is not None
            assert {
                r["interpolation_mode"] for r in rows(window.control_logger.path)
            } == {mode}
            metadata = json.loads(
                window.control_logger.metadata_path.read_text(encoding="utf8")
            )
            assert metadata["modes"]["interpolation_mode"] == mode
            window.normal_stop()
            window.start_tracking()
            assert window.executor is not executor
            assert window.executor.interpolation_mode != mode
        else:
            if case == "emergency":
                window.emergency_stop()
                assert (
                    window.last_sent_cmd == [0] * 6
                    and frame_commands(port.frames[-1]) == [0] * 6
                )
            else:
                window.normal_stop()
                for _ in range(30):
                    previous = list(window.last_sent_cmd)
                    clock.now += 1 / 30
                    window.tick()
                    assert all(
                        abs(b) <= abs(a) and abs(b - a) <= 9
                        for a, b in zip(previous, window.last_sent_cmd)
                    )
                assert window.last_sent_cmd == [0] * 6
            window.start_tracking()
            assert window.executor is not executor
            assert window.executor.seq == -1 and window.executor.frames_since == 0
            assert not np.any(window.executor.I_est)


@pytest.mark.parametrize("profile", ["defaults", "saved"])
def test_parent_2a_legacy_golden(
    app: Any, monkeypatch: pytest.MonkeyPatch, tmp_path: Path, profile: str
) -> None:
    replay_checkpoint(
        app,
        monkeypatch,
        tmp_path,
        profile,
        base=PARENT,
        controller_mode="L3",
        target_box_delta=27,
    )


def test_same_commands_rl_bit_identity() -> None:
    solver = DipoleSolver.from_json(cfg.MODEL_PATH)
    executors = [CurrentExecutor(interpolation_mode=mode) for mode in MODES]
    rng = np.random.default_rng(20261006)
    commands = np.zeros(6, dtype=int)
    for _ in range(2000):
        commands = np.clip(commands + rng.integers(-9, 10, 6), -99, 99)
        outputs = [
            executor.update_est(1 / 30, np.zeros(3), solver, commands)
            for executor in executors
        ]
        exact(outputs[0], outputs[1], "identical_command.RL")
    evidence(
        "rl-identity",
        {
            "frames": 2000,
            "I_est_F_est_bit_identical": True,
            "tau_ms": cfg.L_COIL_H / cfg.R_COIL_OHM * 1000,
            "cutoff_Hz": cfg.R_COIL_OHM / (2 * math.pi * cfg.L_COIL_H),
            "a_30Hz": math.exp(-cfg.R_COIL_OHM / (30 * cfg.L_COIL_H)),
        },
    )


@pytest.mark.parametrize("mode", MODES)
@pytest.mark.parametrize("case", ["step6", "step18", "step27", "alternating3", "mixed"])
def test_rl_actual_mode_response(mode: str, case: str) -> None:
    solver = DipoleSolver.from_json(cfg.MODEL_PATH)
    clock = FakeClock()
    shared = SharedState(clock=clock)
    executor = CurrentExecutor(clock=clock, interpolation_mode=mode)
    targets = (
        [[int(case[4:]) * sign for sign in (1, 0, -1, 1, -1, 0)]]
        if case.startswith("step")
        else SEQUENCES["D" if case == "alternating3" else "E"]
    )
    last = [0] * 6
    trace = []
    for target in targets:
        assert publish(shared, target)
        for _ in range(12 if case.startswith("step") else 3):
            previous_current = executor.I_est.copy()
            clock.now += 1 / 30
            diag = executor.step(
                shared, 1 / 30, np.array([0.001, -0.002, 0]), solver, last
            )
            last = diag["cmd"]
            instant = (
                (np.asarray(last) * solver.current_gain - previous_current)
                * cfg.R_COIL_OHM
                / cfg.L_COIL_H
            )
            trace.append(
                {
                    "cmd": last,
                    "target": target,
                    "I_est_A": diag["I_est"].tolist(),
                    "sampled_dI_A_s": (
                        (diag["I_est"] - previous_current) * 30
                    ).tolist(),
                    "instant_dI_A_s": instant.tolist(),
                    "F_est_uN": (diag["F_est"] * 1e6).tolist(),
                }
            )

    def rms(key: str) -> float:
        return float(np.sqrt(np.mean(np.asarray([r[key] for r in trace]) ** 2)))

    evidence(
        f"rl-{case}-{mode}",
        {
            "mode": mode,
            "case": case,
            "I_rms_A": rms("I_est_A"),
            "sampled_dI_rms_A_s": rms("sampled_dI_A_s"),
            "instant_dI_peak_A_s": float(
                np.max(abs(np.asarray([r["instant_dI_A_s"] for r in trace])))
            ),
            "F_rms_uN": rms("F_est_uN"),
            "trace": trace,
        },
    )


@pytest.mark.parametrize("invalid", ["", "off", "linear", None])
def test_invalid_mode_rejected(invalid: Any) -> None:
    with pytest.raises(ValueError, match="interpolation_mode"):
        CurrentExecutor(interpolation_mode=invalid)


@pytest.mark.parametrize("mode", MODES)
def test_gui_mode_persistence_and_default(
    window: gui.MagneticDipoleControl, mode: str
) -> None:
    assert window.combo_interpolation.currentText() == cfg.EXECUTOR_INTERPOLATION_MODE
    window.combo_interpolation.setCurrentText(mode)
    window.save_settings()
    settings = json.loads(Path(gui.SETTINGS_FILE).read_text(encoding="utf8"))
    assert settings["interpolation_mode"] == mode
    window.combo_interpolation.setCurrentText(
        MODES[1] if mode == MODES[0] else MODES[0]
    )
    window.load_settings()
    assert window.combo_interpolation.currentText() == mode
