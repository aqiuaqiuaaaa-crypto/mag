"""Declared metrics for executor comparisons; no alternate control algorithm."""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any

import numpy as np

MODES = ("legacy_three_frame", "direct")
PARENT = "2a1be54b5e1e668963f4ec756aa44b6084008a26"


def evidence(name: str, record: dict[str, Any]) -> None:
    directory = os.environ.get("MAG_INTERPOLATION_EVIDENCE_DIR")
    if directory:
        destination = Path(directory)
        destination.mkdir(parents=True, exist_ok=True)
        (destination / (name + ".json")).write_text(
            json.dumps(record, indent=2, allow_nan=False), encoding="utf8"
        )
    print(
        "INTERPOLATION_EVIDENCE "
        + json.dumps({k: v for k, v in record.items() if k != "trace"})
    )


def command_metrics(commands: Any, targets: Any) -> dict[str, float]:
    """RMS over all samples/channels; lag is six-channel L2 command units.

    HF ratio uses detrended Hann-window command power, bins >=8 Hz at fs=30 Hz.
    It is a sampled metric, not a statement about PWM or continuous-time harmonics.
    """
    cmds = np.asarray(commands, float)
    target = np.asarray(targets, float)
    delta = np.diff(cmds, axis=0, prepend=np.zeros((1, 6)))
    jerk = np.diff(delta, axis=0, prepend=np.zeros((1, 6)))
    centered = cmds - cmds.mean(axis=0)
    spectrum = abs(np.fft.rfft(centered * np.hanning(len(cmds))[:, None], axis=0)) ** 2
    frequency = np.fft.rfftfreq(len(cmds), 1 / 30)
    lag = np.linalg.norm(target - cmds, axis=1)
    return {
        "command_rms": float(np.sqrt(np.mean(cmds**2))),
        "delta_rms": float(np.sqrt(np.mean(delta**2))),
        "delta2_rms": float(np.sqrt(np.mean(jerk**2))),
        "max_delta": float(np.max(abs(delta))),
        "hf_power_ratio": float(
            spectrum[frequency >= 8].sum() / max(spectrum.sum(), 1e-30)
        ),
        "lag_mean": float(lag.mean()),
        "lag_median": float(np.median(lag)),
        "lag_p95": float(np.percentile(lag, 95)),
        "lag_max": float(lag.max()),
    }


def simulation_metrics(result: dict[str, Any]) -> dict[str, Any]:
    trace = result["trace"]
    metrics: dict[str, Any] = command_metrics(
        [r["cmd"] for r in trace], [r["target_cmd"] for r in trace]
    )
    groups: list[list[dict[str, Any]]] = []
    for row in trace:
        if not groups or row["seq"] != groups[-1][0]["seq"]:
            groups.append([])
        groups[-1].append(row)
    delays = []
    for group in groups:
        # Publication is immediately before the first frame at this 10/30 Hz ordering.
        # Replaced/unreached targets are censored; they never get a fabricated delay.
        for index, row in enumerate(group):
            if max(abs(np.asarray(row["target_cmd"]) - row["cmd"])) <= 1:
                delays.append((index + 1) / 30 * 1000)
                break
    current = np.asarray([r["I_est_A"] for r in trace])
    physical = np.asarray([r["I_physical_A"] for r in trace])
    gain = float(trace[0]["current_gain"])
    previous = np.vstack([np.zeros((1, 6)), current[:-1]])
    cmds = np.asarray([r["cmd"] for r in trace])
    import config as cfg

    instant_derivative = (cmds * gain - previous) * cfg.R_COIL_OHM / cfg.L_COIL_H
    metrics.update(
        reached_targets=len(delays),
        censored_targets=len(groups) - len(delays),
        target_delay_mean_ms=float(np.mean(delays)) if delays else None,
        target_delay_p95_ms=float(np.percentile(delays, 95)) if delays else None,
        target_delay_send_tick_mean_ms=(
            float(np.mean(delays) - 1000 / 30) if delays else None
        ),
        I_est_rms_A=float(np.sqrt(np.mean(current**2))),
        I_physical_rms_A=float(np.sqrt(np.mean(physical**2))),
        sampled_dI_rms_A_s=float(np.sqrt(np.mean(((current - previous) * 30) ** 2))),
        instant_dI_peak_A_s=float(np.max(abs(instant_derivative))),
        F_est_rms_uN=float(
            np.sqrt(np.mean(np.asarray([r["F_est_uN"] for r in trace]) ** 2))
        ),
        velocity_variation_rms_mm_s=float(
            np.sqrt(
                np.mean(
                    np.diff(np.asarray([r["velocity_mm_s"] for r in trace]), axis=0)
                    ** 2
                )
            )
        ),
        completed=result["completion_time_s"] is not None,
    )
    executed = [r for r in trace if not r["missing"]]
    executor_lag = np.linalg.norm(
        np.asarray([r["target_cmd"] for r in executed])
        - np.asarray([r["cmd"] for r in executed]),
        axis=1,
    )
    metrics.update(
        executor_lag_mean=float(executor_lag.mean()),
        executor_lag_median=float(np.median(executor_lag)),
        executor_lag_p95=float(np.percentile(executor_lag, 95)),
        executor_lag_max=float(executor_lag.max()),
    )
    if result["path_mm"] is not None:
        nodes = np.asarray(result["path_mm"])
        end_direction = nodes[-1] - nodes[-2]
        end_direction = end_direction / np.linalg.norm(end_direction)
        positions = np.asarray([r["pos_mm"] for r in trace])
        metrics["endpoint_overshoot_mm"] = float(
            max(0, np.max((positions - nodes[-1]) @ end_direction))
        )
        metrics["endpoint_error_mm"] = float(np.linalg.norm(positions[-1] - nodes[-1]))
        # Euclidean speed is reported separately from the inherited straight-path x speed.
        metrics["mean_motion_speed_mm_s"] = float(
            np.mean(
                np.linalg.norm(np.asarray([r["velocity_mm_s"] for r in trace]), axis=1)
            )
        )
    metrics.update({k: v for k, v in result.items() if k != "trace"})
    return metrics
