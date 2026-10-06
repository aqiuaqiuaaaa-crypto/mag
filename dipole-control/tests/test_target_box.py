"""Period target bounds, actual UART safety and unchanged interpolation semantics."""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any
from unittest.mock import Mock

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import config as cfg
import magnetic_dipole_pid as gui
import test_gui_curt_telemetry as telemetry_tests
from dipole_solver import DipoleSolver
from multirate import CurrentExecutor, SharedState
from test_freshness import FakeClock, FakeSolver, setup_worker
from test_gui_control_log import finish_log, rows, start_log
from test_gui_freshness import start_tracking
from test_l0_checkpoint_replay import exact, replay_checkpoint

app = telemetry_tests.app
ports = telemetry_tests.ports
window = telemetry_tests.window
BASE = "9d0479e8655a1602bc45dd961f066be59dd94ad6"


def orthogonal_solve(
    monkeypatch: pytest.MonkeyPatch,
    desired: list[int],
    previous: list[int],
    box: int,
    limit: int = 99,
) -> dict[str, Any]:
    """Production bounded/quantized solver with an independently diagonal test matrix.

    Synthetic matrix only isolates clipping; physical JSON is used by the replay,
    closed-loop and performance tests. Its artificial B/F units are not plant data.
    """
    solver = DipoleSolver.from_json(cfg.MODEL_PATH)
    y = np.asarray(desired, float) * solver.current_gain
    direction = y[:3] / np.linalg.norm(y[:3])
    monkeypatch.setattr(
        solver, "field_force_actuation_matrix", lambda *args: (np.eye(6), direction)
    )
    return solver.solve_field_force_pseudoinverse(
        np.zeros(3),
        y[3:],
        direction,
        np.linalg.norm(y[:3]) * 1000,
        cmd_prev=previous,
        max_cmd=limit,
        target_box_delta=box,
    )


@pytest.mark.parametrize("box", [9, 27])
@pytest.mark.parametrize(
    "desired,previous,limit",
    [
        ([30, -30, 20, -20, 27, -27], [0] * 6, 99),
        ([30] * 6, [0] * 6, 99),
        ([-30] * 6, [0] * 6, 99),
        ([30, 0, 0, 0, 0, 0], [0] * 6, 99),
        ([80, -80, 80, -80, 80, -80], [45, -45] * 3, 50),
        ([-80, 80, -80, 80, -80, 80], [-45, 45] * 3, 50),
    ],
)
def test_basic_signed_and_saturated_box(
    monkeypatch: pytest.MonkeyPatch,
    box: int,
    desired: list[int],
    previous: list[int],
    limit: int,
) -> None:
    rec = orthogonal_solve(monkeypatch, desired, previous, box, limit)
    cp = np.asarray(previous)
    expected = np.clip(
        desired, np.maximum(-limit, cp - box), np.minimum(limit, cp + box)
    )
    np.testing.assert_array_equal(rec["commands"], expected)
    assert np.max(abs(rec["commands"] - cp)) <= box
    assert np.max(abs(rec["commands"])) <= limit
    assert rec["slew_ok"] and rec["current_bound_ok"]


def test_inactive_box_binary_equivalence(monkeypatch: pytest.MonkeyPatch) -> None:
    old = orthogonal_solve(monkeypatch, [3, -3, 2, -2, 7, -7], [0] * 6, 9)
    new = orthogonal_solve(monkeypatch, [3, -3, 2, -2, 7, -7], [0] * 6, 27)
    assert old["unconstrained_pseudoinverse"] and new["unconstrained_pseudoinverse"]
    for key in old:
        if key != "elapsed_ms":
            exact(old[key], new[key], key)


def test_real_json_inactive_binary_equivalence() -> None:
    solver = DipoleSolver.from_json(cfg.MODEL_PATH)
    rng = np.random.default_rng(20261006)
    for _ in range(20):
        pos = np.r_[rng.uniform(-5, 5, 2) * 1e-3, 0.0]
        current = rng.integers(-15, 16, 6) * solver.current_gain
        output = solver.forward_model(pos, current)
        direction = output["B"] / np.linalg.norm(output["B"])
        args = (pos, output["F"], direction, output["B_magnitude_mT"])
        settled = solver.solve_field_force_pseudoinverse(*args, max_cmd=50)
        pair = [
            solver.solve_field_force_pseudoinverse(
                *args, cmd_prev=settled["commands"], max_cmd=50, target_box_delta=box
            )
            for box in (9, 27)
        ]
        assert all(rec["unconstrained_pseudoinverse"] for rec in pair)
        for key in pair[0]:
            if key != "elapsed_ms":
                exact(pair[0][key], pair[1][key], key)


@pytest.mark.parametrize("invalid", [-1, 0, 10, 28, float("nan"), float("inf")])
def test_target_box_rejects_unsupported_values(invalid: float) -> None:
    solver = DipoleSolver.from_json(cfg.MODEL_PATH)
    with pytest.raises(ValueError, match="target_box_delta"):
        solver.solve_field_force_pseudoinverse(
            np.zeros(3), np.zeros(3), [0, 0, 1], target_box_delta=invalid
        )
    with pytest.raises(ValueError, match="max_delta_cmd"):
        solver.solve_field_force_pseudoinverse(
            np.zeros(3), np.zeros(3), [0, 0, 1], max_delta_cmd=27
        )


def frame_commands(frame: bytes) -> list[int]:
    assert len(frame) == cfg.SERIAL_FRAME_BYTES
    return [
        int(channel.split(":")[1])
        for channel in frame.decode("ascii").strip().split(",")
    ]


def execute_frame(
    widget: gui.MagneticDipoleControl,
    shared: SharedState,
    executor: CurrentExecutor,
    port: telemetry_tests.MemorySerial,
    limit: int,
) -> dict[str, Any]:
    previous = np.array(widget.last_sent_cmd)
    diag = executor.step(
        shared, 1 / 30, np.zeros(3), widget.solver, previous, max_cmd=limit
    )
    widget.send_commands(diag["cmd"])
    actual = np.asarray(frame_commands(port.frames[-1]))
    np.testing.assert_array_equal(actual, widget.last_sent_cmd)
    assert np.max(abs(actual - previous)) <= 9
    assert np.max(abs(actual)) <= limit
    return diag


@pytest.mark.parametrize("limit", [50, 99])
def test_10000_random_targets_actual_uart(
    window: gui.MagneticDipoleControl,
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    limit: int,
) -> None:
    # Only forward-model diagnostics are stubbed; executor, both slew layers,
    # quantization, command frame builder and GUI serial write are production code.
    monkeypatch.setattr(window, "solver", FakeSolver())
    port = telemetry_tests.MemorySerial()
    window.ser = port
    window.spin_cmd_max.setValue(limit)
    clock = FakeClock()
    shared = SharedState(clock=clock)
    executor = CurrentExecutor(clock=clock)
    rng = np.random.default_rng(20261006 + limit)
    alphas: list[float] = []
    max_slew = 0
    for _ in range(10000):
        cp = np.asarray(window.last_sent_cmd)
        target = np.clip(cp + rng.integers(-27, 28, 6), -limit, limit)
        shared.set_I_target(target * window.solver.current_gain, None, [0, 0], 0, 0, 0)
        frames = int(rng.integers(1, 4))  # early arrivals included; nominal cycle is 3
        for index in range(frames):
            clock.now += 1 / 30
            before = np.asarray(window.last_sent_cmd)
            diag = execute_frame(window, shared, executor, port, limit)
            assert diag["interp_alpha"] == (index + 1) / 3
            alphas.append(diag["interp_alpha"])
            actual = np.asarray(window.last_sent_cmd)
            max_slew = max(max_slew, int(np.max(abs(actual - before))))
            assert np.all(actual >= np.minimum(cp, target))
            assert np.all(actual <= np.maximum(cp, target))
        if frames == 3:
            np.testing.assert_array_equal(window.last_sent_cmd, target)
    evidence = {
        "groups": 10000,
        "frames": len(port.frames),
        "max_delta": max_slew,
        "limit": limit,
        "alphas": sorted(set(alphas)),
        "seed": 20261006 + limit,
        "status": "PASS",
    }
    (tmp_path / "random-results.json").write_text(json.dumps(evidence), encoding="utf8")
    print("RANDOM_EVIDENCE " + json.dumps(evidence))


@pytest.mark.parametrize("sign", [1, -1])
def test_t90_step_actual_uart(
    window: gui.MagneticDipoleControl,
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    sign: int,
) -> None:
    monkeypatch.setattr(window, "solver", FakeSolver())
    window.spin_cmd_max.setValue(50)
    evidence: dict[str, dict[str, Any]] = {}
    for box in (9, 27):
        window.last_sent_cmd = [0] * 6
        port = telemetry_tests.MemorySerial()
        window.ser = port
        clock = FakeClock()
        shared = SharedState(clock=clock)
        executor = CurrentExecutor(clock=clock)
        response = []
        for frame in range(12):
            if frame % 3 == 0:
                cp = np.asarray(window.last_sent_cmd)
                target = orthogonal_solve(
                    monkeypatch, [sign * 27] * 6, cp.tolist(), box, 50
                )["commands"]
                shared.set_I_target(
                    target * window.solver.current_gain, None, [0, 0], 0, 0, 0
                )
            clock.now += 1 / 30
            execute_frame(window, shared, executor, port, 50)
            response.append(sign * window.last_sent_cmd[0])
        crossing = next(i for i, value in enumerate(response) if value >= 0.9 * 27)
        evidence[str(box)] = {
            "t90_ms": (crossing + 1) * 1000 / 30,
            "response": response,
        }
        assert response == sorted(response) and max(response) == 27
    assert evidence["27"]["t90_ms"] < evidence["9"]["t90_ms"]
    (tmp_path / "t90-results.json").write_text(json.dumps(evidence), encoding="utf8")
    print("T90_EVIDENCE " + json.dumps({"sign": sign, "results": evidence}))


def test_repeated_and_early_updates_no_old_rebound(
    window: gui.MagneticDipoleControl, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(window, "solver", FakeSolver())
    window.ser = port = telemetry_tests.MemorySerial()
    clock = FakeClock()
    shared = SharedState(clock=clock)
    executor = CurrentExecutor(clock=clock)
    window.spin_cmd_max.setValue(50)
    for desired, frames in [(27, 3), (0, 1), (36, 1), (-9, 3), (18, 3), (-9, 3)]:
        cp = np.asarray(window.last_sent_cmd)
        target = np.clip(
            [desired, -desired, desired, -desired, 0, desired], cp - 27, cp + 27
        )
        shared.set_I_target(target * window.solver.current_gain, None, [0, 0], 0, 0, 0)
        for index in range(frames):
            clock.now += 1 / 30
            before = np.asarray(window.last_sent_cmd)
            diag = execute_frame(window, shared, executor, port, 50)
            assert diag["interp_alpha"] == (index + 1) / 3
            np.testing.assert_array_equal(
                executor.I_from, cp * window.solver.current_gain
            )
            actual = np.asarray(window.last_sent_cmd)
            assert np.all(abs(target - actual) <= abs(target - before))


@pytest.mark.parametrize("box", [9, 27])
def test_worker_bounds_center_on_actual_and_guard_publication(
    monkeypatch: pytest.MonkeyPatch, box: int
) -> None:
    clock, shared, worker = setup_worker()
    params = shared.get_params()
    params["target_box_delta"] = box
    shared.set_params(params, 0)
    shared.set_last_sent([12, -12] * 3)
    solve = Mock(wraps=worker.solver.solve_field_force_pseudoinverse)
    monkeypatch.setattr(worker.solver, "solve_field_force_pseudoinverse", solve)
    worker._step()
    np.testing.assert_array_equal(solve.call_args.kwargs["cmd_prev"], [12, -12] * 3)
    assert solve.call_args.kwargs["target_box_delta"] == box
    seq, progress = shared.get_I_target()[1], shared.get_progress()
    clock.now = 0.2
    worker._step()
    assert solve.call_count == 1
    assert shared.get_I_target()[1] == seq and shared.get_progress() == progress
    shared.set_kalman([3.1, -4], [0, 0], 0)

    def slow(*args: Any, **kwargs: Any) -> Any:
        rec = solve(*args, **kwargs)
        clock.now += 0.2
        return rec

    monkeypatch.setattr(worker.solver, "solve_field_force_pseudoinverse", slow)
    worker._step()
    assert shared.get_I_target()[1] == seq and shared.get_progress() == progress


def test_unseen_stale_27_target_cannot_activate() -> None:
    clock = FakeClock()
    shared = SharedState(clock=clock)
    executor = CurrentExecutor(clock=clock)
    solver = FakeSolver()
    shared.set_I_target(np.full(6, 27 * solver.current_gain), None, [0, 0], 0, 0, 0)
    clock.now = 0.31
    diag = executor.step(shared, 1 / 30, np.zeros(3), solver, [0] * 6)
    assert diag["cmd"] == [0] * 6 and executor.seq == -1 and executor.frames_since == 0
    assert diag["stale_status"] == "STALE_TARGET"
    clock.now = 1.01
    diag = executor.step(shared, 1 / 30, np.zeros(3), solver, [0] * 6)
    assert diag["stop_requested"] and executor.seq == -1


@pytest.mark.parametrize("stop", ["normal_stop", "emergency_stop"])
def test_real_27_tracking_stop(
    window: gui.MagneticDipoleControl, monkeypatch: pytest.MonkeyPatch, stop: str
) -> None:
    clock, port = start_tracking(window, monkeypatch)
    assert window.shared.get_I_target()[3]["rec"]["target_box_delta"] == 27
    previous = window.last_sent_cmd.copy()
    getattr(window, stop)()
    if stop == "emergency_stop":
        # Existing explicit hard-zero exception is preserved, including bypass slew.
        assert frame_commands(port.frames[-1]) == [0] * 6
    for _ in range(20):
        clock.now += 1 / 30
        window.tick()
        if stop == "normal_stop":
            assert max(abs(a - b) for a, b in zip(window.last_sent_cmd, previous)) <= 9
        previous = window.last_sent_cmd.copy()
    assert not window.tracking and window.last_sent_cmd == [0] * 6


def test_gui_default_persistence_manual_and_seq_log(
    window: gui.MagneticDipoleControl, monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    assert window.combo_target_box.currentText() == "27"
    start_log(window, tmp_path / "box.csv")
    clock, _port = start_tracking(window, monkeypatch)
    old_seq = window.executor.seq
    window.combo_target_box.setCurrentText("9")
    window.save_settings()
    settings = json.loads(Path(gui.SETTINGS_FILE).read_text(encoding="utf8"))
    assert settings["target_box_delta"] == "9"
    window.combo_target_box.setCurrentText("27")
    window.load_settings()
    assert window.combo_target_box.currentText() == "9"
    window.tick()  # UI changed; executor still uses the older 27 target
    assert window.executor.seq == old_seq
    assert window.worker is not None
    window.worker._step()
    clock.now += 1 / 30
    window.tick()
    finish_log(window)
    assert window.control_logger is not None
    control = rows(window.control_logger.path)
    worker = rows(window.control_logger.worker_path)
    assert any(r["target_box_delta"] == "27" for r in control)
    assert control[-1]["target_box_delta"] == "9"
    assert {r["target_box_delta"] for r in worker} == {"9", "27"}
    solve = Mock(wraps=window.solver.solve_field_force_pseudoinverse)
    monkeypatch.setattr(window.solver, "solve_field_force_pseudoinverse", solve)
    window.combo_target_box.setCurrentText("27")
    window._solve_motion_target(np.zeros(3), np.array([5e-6, 0, 0]))
    assert "target_box_delta" not in solve.call_args.kwargs


@pytest.mark.parametrize("profile", ["defaults", "saved"])
def test_9d0479e_legacy_box_binary_replay(
    app: Any, monkeypatch: pytest.MonkeyPatch, tmp_path: Path, profile: str
) -> None:
    replay_checkpoint(
        app, monkeypatch, tmp_path, profile, base=BASE, controller_mode="L3"
    )
