"""Binary replay of real parent GUI, ESO, MPC and worker against selected L0.

Parent Python modules are read from the pinned local Git object, never from an
unversioned artifact. Time measurements/callback identities and the added mode
diagnostics are excluded; every parent control quantity and UART byte is checked.
"""

from __future__ import annotations

import json
import struct
import subprocess
import sys
import time
from dataclasses import asdict
from pathlib import Path
from types import ModuleType, SimpleNamespace
from typing import Any
from unittest.mock import patch

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import magnetic_dipole_pid as current
import multirate as current_multi
import test_gui_curt_telemetry as telemetry_tests
from control_mode_simulation import configure_mode
from PySide6.QtWidgets import QApplication, QMessageBox
from test_gui_control_log import configure_camera
from test_gui_curt_telemetry import MemorySerial
from test_serial_safety import request_mode
from test_stop_modes import ACTIVE_MODES

BASE = "8549352e492e4d9cde2cc4142001fcada7ff994a"
ROOT = Path(__file__).resolve().parents[2]
app = telemetry_tests.app


def exact(left: Any, right: Any, path: str) -> None:
    if isinstance(left, np.ndarray):
        assert left.dtype == right.dtype and left.shape == right.shape
        assert left.tobytes() == right.tobytes(), path
    elif isinstance(left, (float, np.floating)):
        assert struct.pack("d", left) == struct.pack("d", right), path
    elif isinstance(left, dict):
        assert left.keys() == right.keys(), path
        for key in left:
            exact(left[key], right[key], f"{path}.{key}")
    elif isinstance(left, (list, tuple)):
        assert len(left) == len(right), path
        for index, (a, b) in enumerate(zip(left, right)):
            exact(a, b, f"{path}.{index}")
    else:
        assert left == right, path


def load_parent(
    monkeypatch: pytest.MonkeyPatch, base: str = BASE
) -> dict[str, ModuleType]:
    loaded: dict[str, ModuleType] = {}
    for name in (
        "config",
        "dipole_solver",
        "estimators",
        "mpc",
        "multirate",
        "magnetic_dipole_pid",
    ):
        source = subprocess.check_output(
            ["git", "show", f"{base}:dipole-control/{name}.py"], cwd=ROOT
        )
        module = ModuleType(f"_l0_parent_{name}")
        module.__file__ = str(ROOT / "dipole-control" / f"{name}.py")
        monkeypatch.setitem(sys.modules, module.__name__, module)
        with patch.dict(sys.modules, loaded):
            exec(  # noqa: S102 -- pinned local checkpoint; no hardware
                compile(source, module.__file__, "exec"), module.__dict__
            )
        loaded[name] = module
    return loaded


def states(left: Any, right: Any, mode: str) -> None:
    for name in (
        "last_sent_cmd",
        "mode",
        "stopping",
        "tracking",
        "dir_test_on",
        "calib_active",
        "coil_test_idx",
        "coil_test_phase",
        "coil_test_frames",
        "coil_test_samples",
        "coil_test_vsum",
        "state_pos_mm",
        "state_vel_mm",
        "z3",
        "last_F_actual",
        "current_B_T",
        "current_F_N",
        "stale_status",
        "target_idx",
    ):
        exact(getattr(left, name), getattr(right, name), f"{mode}.{name}")
    for name in ("kf", "eso_x", "eso_y", "executor"):
        original = {
            key: value
            for key, value in vars(getattr(left, name)).items()
            if key != "clock"
        }
        values = {key: vars(getattr(right, name))[key] for key in original}
        exact(original, values, f"{mode}.{name}")
    a, b = left.shared.get_I_target(), right.shared.get_I_target()
    exact(a[:3], b[:3], mode + ".target")
    for key in ("F_target", "ref_target", "ref_velocity", "mpc_cost"):
        exact(a[3][key], b[3][key], f"{mode}.target.{key}")
    exact(left.shared.get_kalman(), right.shared.get_kalman(), mode + ".kalman")
    exact(
        asdict(left.shared.get_progress()),
        asdict(right.shared.get_progress()),
        mode + ".progress",
    )
    if left.worker is not None:
        assert right.worker is not None
        for name in ("_ref_arc", "_c_drag"):
            exact(
                getattr(left.worker, name),
                getattr(right.worker, name),
                f"{mode}.worker.{name}",
            )
        if left.worker._mpc_signature is not None:
            exact(
                left.worker._mpc_signature,
                right.worker._mpc_signature[: len(left.worker._mpc_signature)],
                mode + ".signature",
            )
        for name in ("mpc_x", "mpc_y"):
            old, new = getattr(left.worker, name), getattr(right.worker, name)
            if old is not None:
                exact(vars(old), {k: vars(new)[k] for k in vars(old)}, f"{mode}.{name}")
    if a[3]["rec"] is not None:
        for key in a[3]["rec"]:
            if key != "elapsed_ms":
                exact(a[3]["rec"][key], b[3]["rec"][key], f"{mode}.solver.{key}")
    assert not right.serial_fault


class ReplayCamera:
    def __init__(self, mode: str) -> None:
        self.index = 0
        self.mode = mode

    def read(self) -> tuple[bool, Any]:
        import cv2

        self.index += 1
        if self.mode == "vision_gap" and 50 <= self.index < 57:
            return False, None
        step = min(max(0, (self.index - 8) // 2), 60)
        x, y = (
            (160 + step, 90)
            if self.mode != "open_polyline" or step <= 40
            else (200, 90 - (step - 40))
        )
        frame = np.zeros((180, 320, 3), np.uint8)
        cv2.circle(frame, (x, y), 8, (255, 255, 255), -1)
        return True, frame

    def release(self) -> None:
        pass


def replay_checkpoint(
    app: QApplication,
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    profile: str,
    *,
    base: str = BASE,
    controller_mode: str = "L0",
    target_box_delta: int = 9,
    include_vision_gap: bool = True,
) -> None:
    parent = load_parent(monkeypatch, base)
    old_gui, old_multi = parent["magnetic_dipole_pid"], parent["multirate"]
    settings_path = tmp_path / "settings.json"
    if profile == "saved":
        settings = json.loads(
            (ROOT / "dipole-control/gui_settings.json").read_text(encoding="utf8")
        )
        settings["model_path"] = str(ROOT / "dipole-control" / settings["model_path"])
        settings_path.write_text(json.dumps(settings), encoding="utf8")
    for module in (old_gui, current):
        monkeypatch.setattr(module, "SETTINGS_FILE", str(settings_path))
        monkeypatch.setattr(module, "list_ports", SimpleNamespace(comports=list))

        def no_camera(*args: Any, **kwargs: Any) -> None:
            raise AssertionError("parent replay must not access a camera")

        monkeypatch.setattr(module.cv2, "VideoCapture", no_camera)
    monkeypatch.setattr(old_gui.ControlWorker, "start", lambda self: None)
    monkeypatch.setattr(current.ControlWorker, "start", lambda self: None)
    cases: list[dict[str, Any]] = []
    with patch.object(QMessageBox, "information", lambda *args: None):
        for mode in (
            "open_line",
            "open_polyline",
            "vision_gap",
            *[m for m in ACTIVE_MODES if m != "tracking"],
        ):
            if mode == "vision_gap" and not include_vision_gap:
                continue  # L3 gap semantics have dedicated continuity regressions.
            for stop in ("normal_stop", "emergency_stop"):
                clock = [1000.0]
                windows = [
                    old_gui.MagneticDipoleControl(),
                    current.MagneticDipoleControl(),
                ]
                configure_mode(windows[1], controller_mode)
                windows[1].combo_target_box.setCurrentText(str(target_box_delta))
                windows[1].combo_interpolation.setCurrentText("legacy_three_frame")
                ports = [MemorySerial(), MemorySerial()]
                count = 0
                try:
                    with patch.object(time, "time", lambda clock=clock: clock[0]):
                        for i, (window, port) in enumerate(zip(windows, ports)):
                            window.timer.stop()
                            window.adc_status_timer.stop()
                            window.control_clock = lambda clock=clock: clock[0]
                            multi = old_multi if i == 0 else current
                            window.shared = multi.SharedState(
                                clock=window.control_clock
                            )
                            window.executor = multi.CurrentExecutor(
                                clock=window.control_clock
                            )
                            window.ser = port
                            window.last_time = clock[0]
                            configure_camera(window)
                            if mode.startswith("open_") or mode == "vision_gap":
                                window.cap = ReplayCamera(mode)

                        def compare(
                            operation: Any,
                            windows: list[Any] = windows,
                            ports: list[MemorySerial] = ports,
                            mode: str = mode,
                            stop: str = stop,
                        ) -> None:
                            nonlocal count
                            start = [len(port.frames) for port in ports]
                            candidates: list[list[Any]] = [[], []]
                            for index, window in enumerate(windows):
                                multi = old_multi if index == 0 else current_multi
                                original_slew = multi.apply_slew_cmd

                                def capture(
                                    value: Any,
                                    *args: Any,
                                    original: Any = original_slew,
                                    bucket: list[Any] = candidates[index],
                                    **kwargs: Any,
                                ) -> Any:
                                    bucket.append(np.asarray(value).copy())
                                    return original(value, *args, **kwargs)

                                with patch.object(multi, "apply_slew_cmd", capture):
                                    operation(window)
                            exact(candidates[0], candidates[1], mode + ".candidate")
                            frames = [port.frames[i:] for port, i in zip(ports, start)]
                            assert frames[0] == frames[1], (mode, stop)
                            assert all(len(frame) == 43 for frame in frames[0])
                            count += len(frames[0])
                            states(windows[0], windows[1], mode)

                        def tick(window: Any, windows: list[Any] = windows) -> None:
                            if (
                                window.tracking
                                and window.worker is not None
                                and window._control_tick_count % 3 == 0
                            ):
                                if window is windows[0]:
                                    with patch.dict(
                                        sys.modules, {"mpc": parent["mpc"]}
                                    ):
                                        window.worker._step()
                                else:
                                    window.worker._step()
                            window.tick()

                        clock[0] += 1 / 30
                        compare(tick)

                        def enter(window: Any, mode: str = mode) -> None:
                            if mode.startswith("open_") or mode == "vision_gap":
                                window.path_px = (
                                    [(160, 90), (200, 90), (200, 50), (250, 50)]
                                    if mode == "open_polyline"
                                    else [(160, 90), (280, 90)]
                                )
                                window.start_tracking()
                            else:
                                request_mode(window, mode)

                        compare(enter)
                        for _ in range(120):
                            clock[0] += 1 / 30
                            compare(tick)
                        assert any(
                            frame != current.build_command([0] * 6)[0].encode("ascii")
                            for frame in ports[0].frames
                        )
                        compare(lambda window, stop=stop: getattr(window, stop)())
                        for _ in range(30):
                            clock[0] += 1 / 30
                            compare(tick)
                        assert all(
                            w.mode == "IDLE" and w.last_sent_cmd == [0] * 6
                            for w in windows
                        )
                        cases.append(
                            {
                                "mode": mode,
                                "stop": stop,
                                "frames": count,
                                "active_ticks": 120,
                                "stop_ticks": 30,
                            }
                        )
                finally:
                    for window in windows:
                        window.save_settings = lambda: None
                        window.close()
    evidence = {
        "parent": base,
        "controller_mode": controller_mode,
        "target_box_delta": target_box_delta,
        "profile": profile,
        "include_vision_gap": include_vision_gap,
        "status": "PASS",
        "cases": cases,
        "frames": sum(c["frames"] for c in cases),
        "active_ticks": sum(c["active_ticks"] for c in cases),
        "stop_ticks": sum(c["stop_ticks"] for c in cases),
        "exclusions": [
            "performance timing",
            "clock callback identity",
            "added mode/F_ss/target_box diagnostics",
        ],
    }
    (tmp_path / "replay-results.json").write_text(
        json.dumps(evidence, indent=2), encoding="utf8"
    )
    print("REPLAY_EVIDENCE " + json.dumps(evidence))


@pytest.mark.parametrize("profile", ["defaults", "saved"])
def test_l0_parent_full_control_and_stop_replay(
    app: QApplication, monkeypatch: pytest.MonkeyPatch, tmp_path: Path, profile: str
) -> None:
    replay_checkpoint(app, monkeypatch, tmp_path, profile)
