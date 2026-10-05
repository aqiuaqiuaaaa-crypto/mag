"""Continuous arc-length projection; no controller or actuator math lives here."""

from __future__ import annotations

import math
from dataclasses import dataclass

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
        finished = (
            not deviation
            and nearest >= self.total - finish_tol
            and end_distance <= finish_tol
        )
        index = min(
            max(int(np.searchsorted(self.arc, nearest, side="right")), 0),
            len(self.points) - 1,
        )
        return PathProgress(
            nearest, self.total, finished, index, deviation, distance, expanded
        )

    def reference(
        self, progress: PathProgress, speed: float, period: float, horizon: int
    ) -> tuple[list[NDArray[np.float64]], list[NDArray[np.float64]]]:
        """Use the existing lookahead/interpolation/tangent reference equations."""
        if len(self.points) < 2:
            zeros = [np.zeros(2) for _ in range(horizon)]
            return zeros, [p.copy() for p in zeros]
        if self.total <= 1e-12:
            p = self.points[-1].copy()
            return [p.copy() for _ in range(horizon)], [
                np.zeros(2) for _ in range(horizon)
            ]
        s0 = min(progress.s_progress + max(float(speed), 0.0) * period, self.total)
        pts, velocities = [], []
        for k in range(horizon):
            sk = min(s0 + max(float(speed), 0.0) * period * k, self.total)
            i = int(np.searchsorted(self.arc, sk, side="right") - 1)
            i = min(max(i, 0), len(self.lengths) - 1)
            t = (sk - self.arc[i]) / max(self.lengths[i], 1e-9)
            pts.append(self.points[i] + t * self.vectors[i])
            if sk >= self.total - 1e-9 or self.lengths[i] <= 1e-9:
                velocities.append(np.zeros(2))
            else:
                velocities.append(float(speed) * self.vectors[i] / self.lengths[i])
        return pts, velocities
