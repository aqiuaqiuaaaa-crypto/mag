"""Continuous arc-length projection; no controller or actuator math lives here."""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

import config as cfg
import numpy as np
from numpy.typing import ArrayLike, NDArray


@dataclass(frozen=True, slots=True)
class PathProgress:
    """One committed worker progress snapshot, in camera-world millimetres."""

    s_progress: float = 0.0
    s_total: float = 0.0
    finished: bool = False
    target_idx: int = 0  # display only: endpoint of the projected segment
    path_deviation: bool = False
    distance_mm: float = 0.0
    expanded_search: bool = False
    at_endpoint: bool = False  # geometry only; never sufficient for completion
    approach_s: float | None = None  # terminal reference arc, not measured progress
    approach_mono: float | None = None

    @property
    def holding_endpoint(self) -> bool:
        return self.approach_s is not None and self.approach_s >= self.s_total


@dataclass
class EndpointSettling:
    """Dwell in fresh observation time, not wall time or number of worker solves."""

    since: float | None = None
    last_sample: float | None = None

    def reset(self) -> None:
        self.since = self.last_sample = None

    def observe(self, sample_mono: float, eligible: bool, max_gap: float) -> None:
        if not eligible or not math.isfinite(sample_mono):
            self.reset()
            return
        if self.last_sample is not None:
            gap = sample_mono - self.last_sample
            if gap < 0 or gap > max_gap:
                self.reset()
            elif gap == 0:
                return  # solving an old snapshot again cannot earn dwell
        if self.since is None:
            self.since = sample_mono
        self.last_sample = sample_mono

    def ready(self, input_mono: float, dwell: float) -> bool:
        return bool(
            math.isfinite(dwell)
            and dwell > 0
            and self.since is not None
            and self.last_sample is not None
            and self.since <= input_mono <= self.last_sample
            and input_mono - self.since >= dwell
        )


@dataclass(frozen=True, slots=True)
class PathGeometry:
    """Existing polyline and cumulative lengths, including its lead segment."""

    points: NDArray[np.float64]
    vectors: NDArray[np.float64]
    lengths: NDArray[np.float64]
    arc: NDArray[np.float64]

    @classmethod
    def from_path(cls, path: ArrayLike) -> PathGeometry:
        """Copy a path without resampling, closing or otherwise changing it."""
        points = np.asarray(path, dtype=np.float64).reshape(-1, 2).copy()
        vectors = np.diff(points, axis=0)
        lengths = np.linalg.norm(vectors, axis=1)
        arc = np.concatenate([[0.0], np.cumsum(lengths)])
        return cls(points, vectors, lengths, arc)

    @property
    def total(self) -> float:
        return float(self.arc[-1])

    def _nearest(
        self, pos: NDArray[np.float64], lower: float, upper: float
    ) -> tuple[float, float]:
        """Project only onto the permitted arc, clipping partial segments."""
        best_d2, nearest = float("inf"), lower
        for i, vec in enumerate(self.vectors):
            if self.arc[i + 1] < lower or self.arc[i] > upper:
                continue
            length2 = float(vec @ vec)
            if length2 <= 1e-12:
                continue
            t = float(np.clip(((pos - self.points[i]) @ vec) / length2, 0.0, 1.0))
            t_min = max(0.0, (lower - self.arc[i]) / self.lengths[i])
            t_max = min(1.0, (upper - self.arc[i]) / self.lengths[i])
            if t < t_min:
                t = float(t_min)
            elif t > t_max:
                t = float(t_max)
            projection = self.points[i] + t * vec
            d2 = float(np.sum((pos - projection) ** 2))
            if d2 < best_d2:
                best_d2 = d2
                nearest = float(self.arc[i] + t * self.lengths[i])
        return nearest, math.sqrt(best_d2)

    def project(
        self,
        pos: ArrayLike,
        previous: PathProgress,
        *,
        fmax: float,
        c_drag: float,
        period: float,
        finish_tol: float,
    ) -> PathProgress:
        """Compute a candidate; the caller commits it only with a valid target."""
        xy = np.asarray(pos, dtype=np.float64)
        if len(self.points) == 0:
            return PathProgress()
        end_distance = float(np.linalg.norm(xy - self.points[-1]))
        if self.total <= 1e-12:
            nearest, distance, expanded = 0.0, end_distance, False
        else:
            lower = max(0.0, previous.s_progress - cfg.PATH_BACKTRACK_MM)
            forward = max(
                cfg.PATH_PROJ_FWD_MIN_MM,
                cfg.PATH_PROJ_FWD_FACTOR * max(fmax, 0.0) / max(c_drag, 1e-9) * period,
            )
            upper = min(previous.s_progress + forward, self.total)
            nearest, distance = self._nearest(xy, lower, upper)
            expanded = distance > cfg.PATH_LOCAL_PROJ_MAX_DIST_MM
            if expanded:
                nearest, distance = self._nearest(xy, lower, self.total)
        deviation = distance > cfg.PATH_LOST_DIST_MM
        at_endpoint = (
            not deviation
            and nearest >= self.total - finish_tol
            and end_distance <= finish_tol
        )
        index = min(
            max(int(np.searchsorted(self.arc, nearest, side="right")), 0),
            len(self.points) - 1,
        )
        return PathProgress(
            s_progress=nearest,
            s_total=self.total,
            target_idx=index,
            path_deviation=deviation,
            distance_mm=distance,
            expanded_search=expanded,
            at_endpoint=at_endpoint,
        )

    def advance_reference(
        self, arc: float, speed: float, dt: float, decel_distance: float
    ) -> tuple[float, float]:
        """Integrate v(s)=v_nom*min(1,sqrt(remaining/D)) exactly.

        This is a finite-duration reference profile, not an identified plant
        acceleration. Speed is continuous at both ends of the deceleration zone.
        """
        if not math.isfinite(decel_distance) or decel_distance <= 0:
            raise ValueError("terminal deceleration distance must be positive")
        speed, dt = max(float(speed), 0.0), max(float(dt), 0.0)
        remaining = max(self.total - float(arc), 0.0)
        if speed == 0 or remaining == 0:
            return self.total - remaining, 0.0
        cruise_time = min(dt, max(remaining - decel_distance, 0.0) / speed)
        remaining -= speed * cruise_time
        dt -= cruise_time
        if remaining <= decel_distance:
            root = max(
                math.sqrt(remaining) - speed * dt / (2 * math.sqrt(decel_distance)), 0.0
            )
            remaining = root * root
            velocity = speed * root / math.sqrt(decel_distance)
        else:
            velocity = speed
        arc = self.total - remaining
        return arc, 0.0 if arc >= self.total else velocity

    def plan_approach(
        self,
        progress: PathProgress,
        previous: PathProgress,
        speed: float,
        input_mono: float,
        decel_distance: float,
    ) -> PathProgress:
        """Latch a terminal reference cursor; commit it with the worker target.

        Once slowing, advance on successive fresh input times instead of
        reanchoring at the measured position forever. Gaps pause the cursor;
        overshooting or backtracking cannot restart the terminal approach.
        """
        if not len(self.points):
            return progress
        if previous.approach_s is None:
            if self.total - progress.s_progress > decel_distance:
                return progress
            arc = progress.s_progress
        else:
            dt = input_mono - (
                previous.approach_mono
                if previous.approach_mono is not None
                else input_mono
            )
            if not 0 <= dt <= cfg.KALMAN_STALE_S:
                dt = 0.0
            arc, _ = self.advance_reference(
                previous.approach_s, speed, dt, decel_distance
            )
        # A measured arrival/overshoot already projects onto the endpoint.
        if progress.s_progress >= self.total:
            arc = self.total
        return replace(progress, approach_s=arc, approach_mono=input_mono)

    def reference(
        self,
        progress: PathProgress,
        speed: float,
        period: float,
        horizon: int,
        *,
        decel_distance: float = cfg.PATH_END_DECEL_MM,
    ) -> tuple[list[NDArray[np.float64]], list[NDArray[np.float64]]]:
        """Keep polyline interpolation; taper velocity before the endpoint."""
        if not math.isfinite(decel_distance) or decel_distance <= 0:
            raise ValueError("terminal deceleration distance must be positive")
        if len(self.points) == 0:
            zeros = [np.zeros(2) for _ in range(horizon)]
            return zeros, [p.copy() for p in zeros]
        if self.total <= 1e-12:
            p = self.points[-1].copy()
            return [p.copy() for _ in range(horizon)], [
                np.zeros(2) for _ in range(horizon)
            ]
        base = (
            progress.s_progress if progress.approach_s is None else progress.approach_s
        )
        pts, velocities = [], []
        for k in range(horizon):
            # Preserve the original arithmetic outside the terminal zone.
            cruise_s = min(base + max(float(speed), 0.0) * period, self.total)
            cruise_s = min(cruise_s + max(float(speed), 0.0) * period * k, self.total)
            if cruise_s < self.total - decel_distance:
                sk, planned_speed = cruise_s, max(float(speed), 0.0)
            else:
                sk, planned_speed = self.advance_reference(
                    base, speed, period * (k + 1), decel_distance
                )
            i = int(np.searchsorted(self.arc, sk, side="right") - 1)
            i = min(max(i, 0), len(self.lengths) - 1)
            t = (sk - self.arc[i]) / max(self.lengths[i], 1e-9)
            pts.append(self.points[i] + t * self.vectors[i])
            if sk >= self.total - 1e-9 or self.lengths[i] <= 1e-9:
                velocities.append(np.zeros(2))
            else:
                velocities.append(planned_speed * self.vectors[i] / self.lengths[i])
        return pts, velocities
