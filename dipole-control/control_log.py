"""Opt-in control diagnostics: immutable rows, bounded queue, one disk thread.

Blank CSV cells mean unavailable/not executed this tick. ADC values are raw counts.
All force columns use micro-newtons, B columns tesla, current columns amperes,
positions millimetres and velocities millimetres/second. No control calculation
or feedback lives here. A session creates four NEW files and never overwrites.
"""

from __future__ import annotations

import csv
import json
import subprocess
import time
from collections import OrderedDict
from collections.abc import Mapping
from contextlib import ExitStack
from pathlib import Path
from queue import Empty, Full, Queue
from threading import Event, Lock, Thread
from typing import Any, Literal

Scalar = str | int | float | bool | None
Stream = Literal["control", "worker"]

CONTROL_FIELDS = (
    "t_mono",
    "t_wall",
    "tick",
    "dt_ms",
    "jitter_ms",
    "total_ms",
    "vision_ms",
    "estimator_ms",
    "controller_ms",
    "serial_ms",
    "dropped",
    "mode",
    "tracking",
    "kalman_age_ms",
    "target_age_ms",
    "stale_status",
    "frame_ok",
    "detected",
    "px_x",
    "px_y",
    "area_px",
    "meas_x_mm",
    "meas_y_mm",
    "kf_x",
    "kf_y",
    "kf_vx",
    "kf_vy",
    "estimator_mode",
    "state_x_mm",
    "state_y_mm",
    "state_vx_mm_s",
    "state_vy_mm_s",
    "eso_mode",
    "effort_mode",
    "F_ss_x",
    "F_ss_y",
    "eso_updated",
    "d_hat_x",
    "d_hat_y",
    "u_eso_x",
    "u_eso_y",
    "target_idx",
    "s_progress",
    "s_total",
    "finished",
    "path_deviation",
    "ref_x",
    "ref_y",
    "vref_x",
    "vref_y",
    "run",
    "seq",
    "alpha",
    "frames_since",
    "F_target_x",
    "F_target_y",
    "F_target_z",
    *(f"I_target_{i}" for i in range(6)),
    "executor_updated",
    *(f"cmd_exec_a{i}" for i in range(6)),
    *(f"cmd_sent_a{i}" for i in range(6)),
    "max_cmd",
    "serial_connected",
    *(f"I_est_{i}" for i in range(6)),
    "F_est_x",
    "F_est_y",
    "F_est_z",
    "B_x",
    "B_y",
    "B_z",
    "B_mag_mT",
    "align_ratio",
    "low_field",
    "adc_frame_count",
    "adc_age_ms",
    *(f"raw{i}" for i in range(6)),
    "adc_valid",
    "adc_running",
    "adc_error_flags",
)
WORKER_FIELDS = (
    "t_mono",
    "t_wall",
    "run",
    "seq",
    "dropped",
    "kalman_age_ms",
    "target_age_ms",
    "stale_status",
    "x0_x",
    "x0_y",
    "d_used_x",
    "d_used_y",
    "eso_mode",
    "effort_mode",
    "F_ss_x",
    "F_ss_y",
    "F_prev_x",
    "F_prev_y",
    "F_prev_z",
    "F_target_x",
    "F_target_y",
    "F_target_z",
    "ref_x",
    "ref_y",
    "vref_x",
    "vref_y",
    "s_progress",
    "s_total",
    "finished",
    "path_deviation",
    "mpc_ms",
    "solver_ms",
    "mpc_cost",
    "horizon",
    "w_pos",
    "w_vel",
    "w_u",
    "w_du",
    "fmax",
    "max_cmd",
    "force_error_uN",
    "converged",
    "current_constraint_active",
    "field_constraint_active",
    "direction_constraint_active",
    "sparse_infeasible",
    "current_bound_ok",
    "slew_ok",
    "actuation_condition",
    *(f"I_target_{i}" for i in range(6)),
)


class ControlLogger:
    """Nonblocking producer API. Errors belong to diagnostics, never to control."""

    def __init__(
        self, path: Path, metadata: Mapping[str, object], *, capacity: int = 2048
    ) -> None:
        if capacity < 1:
            raise ValueError("capacity must be positive")
        self.path = path
        self.worker_path = path.with_suffix(".worker.csv")
        self.metadata_path = path.with_suffix(".metadata.json")
        self.summary_path = path.with_suffix(".summary.json")
        self.error: str | None = None
        self.opened = Event()
        self.finished = Event()
        self._stopping = Event()
        self._lock = Lock()
        self._queue: Queue[tuple[Stream, tuple[Scalar, ...]]] = Queue(capacity)
        self._metadata = dict(metadata)
        self._stats = {
            name: {
                "attempted": 0,
                "accepted": 0,
                "written": 0,
                "dropped": 0,
                "rejected": 0,
            }
            for name in ("control", "worker")
        }
        self._targets: OrderedDict[tuple[int, int], dict[str, Scalar]] = OrderedDict()
        self._thread = Thread(target=self._run, name="control CSV", daemon=True)
        self._thread.start()

    @property
    def recording(self) -> bool:
        return not (self._stopping.is_set() or self.finished.is_set() or self.error)

    @property
    def dropped(self) -> int:
        with self._lock:
            return sum(stats["dropped"] for stats in self._stats.values())

    def stats(self) -> dict[str, dict[str, int]]:
        with self._lock:
            return {name: dict(stats) for name, stats in self._stats.items()}

    def fail(self, exc: Exception) -> None:
        """Contain even a diagnostics programming error at the producer boundary."""
        self.error = f"{type(exc).__name__}: {exc}"
        self._stopping.set()

    def enqueue(self, stream: Stream, values: Mapping[str, Scalar]) -> bool:
        """Freeze existing scalar values; full queue only increments dropped."""
        try:
            fields = CONTROL_FIELDS if stream == "control" else WORKER_FIELDS
            with self._lock:
                stats = self._stats[stream]
                stats["attempted"] += 1
                if self._stopping.is_set() or self.finished.is_set():
                    stats["rejected"] += 1
                    return False
                dropped = sum(s["dropped"] for s in self._stats.values())
                row = tuple(
                    dropped if key == "dropped" else values.get(key) for key in fields
                )
                try:
                    self._queue.put_nowait((stream, row))
                except Full:
                    stats["dropped"] += 1
                    return False
                stats["accepted"] += 1
                return True
        except Exception as exc:  # noqa: BLE001 -- isolate diagnostics from control
            with self._lock:
                self._stats[stream]["rejected"] += 1
            self.fail(exc)
            return False

    def remember_target(self, run: int, seq: int, target: Mapping[str, Any]) -> None:
        """Observer for the existing SharedState snapshot, before it is unlocked.

        The untyped legacy dictionary is converted to scalar diagnostics only;
        the controller snapshot is never modified. Keep at most 128 targets.
        """
        if not self.recording:
            return
        try:
            row: dict[str, Scalar] = {"run": run, "seq": seq}
            for prefix, key in (("ref", "ref_target"), ("vref", "ref_velocity")):
                for axis, value in zip(("x", "y"), target[key]):
                    row[f"{prefix}_{axis}"] = float(value)
            row.update(
                F_target_x=float(target["F_target"][0]),
                F_target_y=float(target["F_target"][1]),
            )
            rec = target.get("rec") or {}
            if "mpc_effort_mode" in rec:
                row["effort_mode"] = str(rec["mpc_effort_mode"])
                for axis, value in zip(("x", "y"), rec["F_ss_camera_uN"]):
                    row[f"F_ss_{axis}"] = float(value)
            force = rec.get("requested_force_camera")
            if force is not None:
                row["F_target_z"] = float(force[2]) * 1e6
            for i, value in enumerate(target["currents"]):
                row[f"I_target_{i}"] = float(value)
            with self._lock:
                self._targets[(run, seq)] = row
                if len(self._targets) > 128:
                    self._targets.popitem(last=False)
        except Exception as exc:  # noqa: BLE001 -- isolate diagnostics from control
            self.fail(exc)

    def target(self, run: int, seq: int) -> dict[str, Scalar]:
        with self._lock:
            return dict(self._targets.get((run, seq), {}))

    def remember_progress(self, run: int, seq: int, progress: float) -> None:
        with self._lock:
            if (run, seq) in self._targets:
                self._targets[(run, seq)]["s_progress"] = progress

    def request_stop(self) -> None:
        self._stopping.set()

    def wait_closed(self, timeout: float = 1.0) -> bool:
        self.request_stop()
        self._thread.join(timeout)
        return self.finished.is_set()

    def _git_metadata(self) -> dict[str, object]:
        try:
            result = subprocess.run(
                ["git", "rev-parse", "HEAD"],
                cwd=Path(__file__).resolve().parents[1],
                capture_output=True,
                text=True,
                timeout=2.0,
                check=True,
                creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
            )
            dirty = subprocess.run(
                ["git", "diff", "--quiet", "HEAD"],
                cwd=Path(__file__).resolve().parents[1],
                capture_output=True,
                check=False,
                timeout=2.0,
                creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
            )
            return {
                "git_commit": result.stdout.strip(),
                "git_tracked_dirty": dirty.returncode != 0,
            }
        except (OSError, subprocess.SubprocessError) as exc:
            return {"git_commit": None, "git_error": str(exc)}

    def _run(self) -> None:
        try:
            paths = (self.path, self.worker_path, self.metadata_path, self.summary_path)
            if any(path.exists() for path in paths):
                raise FileExistsError("日志会话文件已存在，请选择新文件名")
            with ExitStack() as stack:
                control = stack.enter_context(
                    self.path.open("x", newline="", encoding="utf-8")
                )
                worker = stack.enter_context(
                    self.worker_path.open("x", newline="", encoding="utf-8")
                )
                meta = stack.enter_context(
                    self.metadata_path.open("x", encoding="utf-8")
                )
                summary = stack.enter_context(
                    self.summary_path.open("x", encoding="utf-8")
                )
                metadata = {
                    **self._metadata,
                    **self._git_metadata(),
                    "schema_version": 1,
                    "queue_capacity": self._queue.maxsize,
                    "control_fields": CONTROL_FIELDS,
                    "worker_fields": WORKER_FIELDS,
                    "writer_started_mono": time.monotonic(),
                }
                json.dump(metadata, meta, ensure_ascii=False, indent=2, allow_nan=False)
                meta.flush()
                writers = {"control": csv.writer(control), "worker": csv.writer(worker)}
                writers["control"].writerow(CONTROL_FIELDS)
                writers["worker"].writerow(WORKER_FIELDS)
                control.flush()
                worker.flush()
                self.opened.set()
                while not self._stopping.is_set() or not self._queue.empty():
                    try:
                        stream, row = self._queue.get(timeout=0.05)
                    except Empty:
                        continue
                    writers[stream].writerow(row)
                    # Flush on the disk thread, so a read-back sees completed rows.
                    (control if stream == "control" else worker).flush()
                    with self._lock:
                        self._stats[stream]["written"] += 1
                json.dump(
                    {"stats": self.stats(), "error": self.error}, summary, indent=2
                )
                summary.flush()
        except Exception as exc:  # noqa: BLE001 -- writer failure cannot stop control
            # Includes serialization and unexpected writer faults; never signal a
            # solver error, stop tracking or touch the serial connection.
            self.fail(exc)
        finally:
            self.finished.set()
