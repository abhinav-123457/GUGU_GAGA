"""Phase 16B: TRUTH-SIDE localisation scoring. See docs/PHASE16B_ESTIMATION.md.

One of only two `estimation` modules allowed to see ground truth (the other
is `plant_odometry`). It compares each drone's `EstimatedState` against the
real pose, after the fact, for reporting only - nothing here is ever fed
back into a decision.

NEES (normalised estimation error squared) is `e^T P^-1 e` for the
horizontal error `e` and the estimator's own covariance `P`. For a
consistent estimator its mean is 2 (two degrees of freedom); a mean far
above 2 means the covariance is over-confident for the error actually made -
the number that decides whether covariance-aware safety can be trusted.
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional

import numpy as np

from ..contracts import GeofenceSpec
from ..mapping.grid import GridSpec
from .models import EstimatedState

NEES_95_2DOF = 5.991464547107979   # chi-square 0.95 quantile, 2 degrees of freedom


class _Acc:
    __slots__ = ("n", "sum_err", "sum_sq_err", "max_err", "final_err", "path_m", "prev_xy", "nees_sum",
                 "nees_n", "nees_ok", "invalid", "degraded", "cell_match", "cell_n", "max_sigma", "final_sigma",
                 "final_yaw_err", "max_yaw_err")

    def __init__(self):
        self.n = 0
        self.sum_err = self.sum_sq_err = self.max_err = self.final_err = self.path_m = 0.0
        self.prev_xy: Optional[np.ndarray] = None
        self.nees_sum = 0.0
        self.nees_n = self.nees_ok = self.invalid = self.degraded = self.cell_match = self.cell_n = 0
        self.max_sigma = self.final_sigma = self.final_yaw_err = self.max_yaw_err = 0.0


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class LocalizationScorer:
    def __init__(self, num_drones: int, grid: Optional[GridSpec] = None):
        self.n = num_drones
        self.grid = grid
        self._acc: List[_Acc] = [_Acc() for _ in range(num_drones)]

    def record(self, i: int, est: EstimatedState, true_position_xyz, true_yaw_rad: float) -> None:
        a = self._acc[i]
        true_xy = np.asarray(true_position_xyz, dtype=float)[:2]
        est_xy = np.array(est.position_m[:2], dtype=float)
        err = float(np.linalg.norm(est_xy - true_xy))
        a.n += 1
        a.sum_err += err
        a.sum_sq_err += err * err
        a.max_err = max(a.max_err, err)
        a.final_err = err
        if a.prev_xy is not None:
            a.path_m += float(np.linalg.norm(true_xy - a.prev_xy))
        a.prev_xy = true_xy.copy()

        sxx, sxy, syy = est.pos_cov_xy_m2
        P = np.array([[sxx, sxy], [sxy, syy]]) + np.eye(2) * 1e-12
        e = est_xy - true_xy
        try:
            nees = float(e @ np.linalg.solve(P, e))
        except np.linalg.LinAlgError:
            nees = math.inf
        if math.isfinite(nees):
            a.nees_sum += nees
            a.nees_n += 1
            a.nees_ok += int(nees <= NEES_95_2DOF)

        sigma = math.sqrt(max(0.5 * (sxx + syy) + math.sqrt((0.5 * (sxx - syy)) ** 2 + sxy * sxy), 0.0))
        a.max_sigma = max(a.max_sigma, sigma)
        a.final_sigma = sigma
        yaw_err = abs(_wrap(est.attitude_rad[2] - true_yaw_rad))
        a.final_yaw_err = yaw_err
        a.max_yaw_err = max(a.max_yaw_err, yaw_err)
        a.invalid += int(not est.valid)
        a.degraded += int(est.degraded)
        if self.grid is not None:
            a.cell_n += 1
            a.cell_match += int(self.grid.cell_of(tuple(est_xy)) == self.grid.cell_of(tuple(true_xy)))

    def summary(self) -> Dict:
        drones = []
        for i, a in enumerate(self._acc):
            if a.n == 0:
                drones.append({"drone": i, "samples": 0})
                continue
            drones.append({
                "drone": i, "samples": a.n,
                "mean_error_m": a.sum_err / a.n, "rms_error_m": math.sqrt(a.sum_sq_err / a.n),
                "max_error_m": a.max_err, "final_error_m": a.final_err,
                "path_length_m": a.path_m,
                "final_drift_fraction": (a.final_err / a.path_m) if a.path_m > 0.0 else None,
                "mean_nees": (a.nees_sum / a.nees_n) if a.nees_n else None,
                "nees_within_95_fraction": (a.nees_ok / a.nees_n) if a.nees_n else None,
                "max_sigma_m": a.max_sigma, "final_sigma_m": a.final_sigma,
                "final_yaw_error_rad": a.final_yaw_err, "max_yaw_error_rad": a.max_yaw_err,
                "invalid_tick_fraction": a.invalid / a.n, "degraded_tick_fraction": a.degraded / a.n,
                "cell_match_fraction": (a.cell_match / a.cell_n) if a.cell_n else None,
            })
        seen = [d for d in drones if d["samples"]]

        def _mean(key):
            vals = [d[key] for d in seen if d.get(key) is not None]
            return float(np.mean(vals)) if vals else None

        return {
            "num_drones": self.n,
            "mean_error_m": _mean("mean_error_m"),
            "max_error_m": max((d["max_error_m"] for d in seen), default=None),
            "mean_final_error_m": _mean("final_error_m"),
            "max_final_error_m": max((d["final_error_m"] for d in seen), default=None),
            "mean_final_drift_fraction": _mean("final_drift_fraction"),
            "mean_nees": _mean("mean_nees"),
            "mean_nees_within_95_fraction": _mean("nees_within_95_fraction"),
            "max_sigma_m": max((d["max_sigma_m"] for d in seen), default=None),
            "mean_invalid_tick_fraction": _mean("invalid_tick_fraction"),
            "mean_degraded_tick_fraction": _mean("degraded_tick_fraction"),
            "mean_cell_match_fraction": _mean("cell_match_fraction"),
            "per_drone": drones,
        }


class GeofenceExcursionScorer:
    """TRUTH-SIDE: how often, and how far, drones really leave the geofence.

    The safety supervisor enforces the geofence on the drone's OWN position
    estimate, which under GNSS denial can be wrong - a drone that believes it
    is 3 m inside may really be outside. This scorer checks the real pose.
    Horizontal box only (matching the supervisor's own geofence check)."""

    def __init__(self, num_drones: int, geofence: GeofenceSpec):
        self.n = num_drones
        self._cx, self._cy = geofence.center_m
        self._hx, self._hy = geofence.half_extents_m
        self._ticks = [0] * num_drones
        self._outside = [0] * num_drones
        self._max_excursion = [0.0] * num_drones

    def record(self, i: int, true_position_xyz) -> None:
        x, y = float(true_position_xyz[0]), float(true_position_xyz[1])
        margin = min(self._hx - abs(x - self._cx), self._hy - abs(y - self._cy))
        self._ticks[i] += 1
        if margin < 0.0:
            self._outside[i] += 1
            self._max_excursion[i] = max(self._max_excursion[i], -margin)

    def summary(self) -> Dict:
        total = sum(self._ticks)
        return {
            "ticks": total,
            "ticks_outside": sum(self._outside),
            "outside_fraction": (sum(self._outside) / total) if total else None,
            "max_excursion_m": max(self._max_excursion, default=0.0),
            "per_drone_max_excursion_m": list(self._max_excursion),
        }
