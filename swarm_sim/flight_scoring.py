"""Phase 16C: SCORING ONLY - how well the drones actually held their altitude, and why they touched things.

Plant side: it is fed the TRUE altitude by the mission and nothing here ever flows back into a decision
(tests/test_phase16c_architecture.py keeps every autonomy module from importing it).

Contact attribution
-------------------
Each drone's FIRST contact decides its root cause, because everything after a first contact (a tumble, a
bounce, further ground strikes) is a consequence of it:

  drone_ground   -> "vertical_control"  the drone reached the ground without first touching anything else,
                                        while upright: a failure of altitude control
                 -> "attitude_loss"     ... but only if it had been tilted past 1 rad (57 degrees) in the second
                                        before the strike: it tumbled (e.g. an abrupt stop from high speed) and
                                        fell, which is not an altitude-control failure
  drone_obstacle -> "obstacle"
  drone_drone    -> "swarm"             both drones involved are attributed

A later ground contact of a drone that had already touched something is counted separately as
`post_contact_ground`. This is a bookkeeping rule, not a physical proof: two contacts in the same tick are
attributed in the order the physics engine reports them."""
from __future__ import annotations

from typing import Dict, List, Optional, Sequence

import numpy as np

ROOT_CAUSES = ("vertical_control", "attitude_loss", "obstacle", "swarm")
TILT_LIMIT_RAD = 1.0
TILT_WINDOW_S = 1.0
_CATEGORY_TO_ROOT = {"drone_ground": "vertical_control", "drone_obstacle": "obstacle", "drone_drone": "swarm"}


class FlightScorer:
    def __init__(self, num_drones: int, target_altitude_m: float, floor_alt_m: float, ceiling_alt_m: float,
                 settle_time_s: float = 2.0):
        self.n = num_drones
        self.target_m = float(target_altitude_m)
        self.floor_m = float(floor_alt_m)
        self.ceiling_m = float(ceiling_alt_m)
        self.settle_time_s = float(settle_time_s)
        self._z: List[List[tuple]] = [[] for _ in range(num_drones)]      # (t, true altitude) after the settle time
        self._tilt: List[List[tuple]] = [[] for _ in range(num_drones)]     # (t, max(|roll|, |pitch|)) recent samples
        self._first_contact: Dict[int, dict] = {}
        self._post_contact_ground = 0
        self._events_by_category: Dict[str, int] = {}

    # -- recording ---------------------------------------------------------

    def record_tick(self, t: float, i: int, z_m: float, roll_pitch_rad=(0.0, 0.0)) -> None:
        if t >= self.settle_time_s:
            self._z[i].append((float(t), float(z_m)))
        tilt = self._tilt[i]
        tilt.append((float(t), max(abs(float(roll_pitch_rad[0])), abs(float(roll_pitch_rad[1])))))
        while tilt and tilt[0][0] < t - TILT_WINDOW_S:
            tilt.pop(0)

    def _recently_tumbling(self, i: int, t: float) -> bool:
        return any(v > TILT_LIMIT_RAD for ts, v in self._tilt[i] if ts >= t - TILT_WINDOW_S)

    def record_contact(self, t: float, category: str, drone_indices: Sequence[int],
                       altitude_m: Optional[float] = None) -> None:
        self._events_by_category[category] = self._events_by_category.get(category, 0) + 1
        root = _CATEGORY_TO_ROOT.get(category)
        for i in drone_indices:
            if i in self._first_contact:
                if category == "drone_ground":
                    self._post_contact_ground += 1
                continue
            drone_root = "attitude_loss" if (root == "vertical_control" and self._recently_tumbling(i, t)) else root
            self._first_contact[i] = {"drone": int(i), "t": float(t), "root_cause": drone_root,
                                      "altitude_m": None if altitude_m is None else float(altitude_m)}

    # -- reporting ---------------------------------------------------------

    def _altitude_block(self, per_drone_samples) -> dict:
        """Altitude statistics over `per_drone_samples` (one array of altitudes per drone)."""
        per_drone = []
        for i, z in enumerate(per_drone_samples):
            if z.size == 0:
                per_drone.append({"drone": i, "samples": 0})
                continue
            err = z - self.target_m
            per_drone.append({
                "drone": i, "samples": int(z.size), "std_m": float(z.std()), "mean_abs_error_m": float(np.abs(err).mean()),
                "max_abs_error_m": float(np.abs(err).max()), "min_m": float(z.min()), "max_m": float(z.max()),
                "outside_band_fraction": float(np.mean((z < self.floor_m) | (z > self.ceiling_m))),
            })
        non_empty = [z for z in per_drone_samples if z.size]
        if not non_empty:
            return {"samples": 0, "target_m": self.target_m, "per_drone": per_drone}
        z = np.concatenate(non_empty)
        err = z - self.target_m
        return {
            "samples": int(z.size), "target_m": self.target_m,
            "std_m": float(z.std()), "mean_abs_error_m": float(np.abs(err).mean()),
            "max_abs_error_m": float(np.abs(err).max()), "min_m": float(z.min()), "max_m": float(z.max()),
            "outside_band_fraction": float(np.mean((z < self.floor_m) | (z > self.ceiling_m))),
            "median_per_drone_std_m": float(np.median([d["std_m"] for d in per_drone if d.get("samples")])),
            "per_drone": per_drone,
        }

    def summary(self) -> dict:
        all_samples = [np.array([z for _, z in self._z[i]], dtype=float) for i in range(self.n)]
        # Altitude BEFORE each drone's first contact: what the altitude controller did on its own, without
        # the tumble / fall that follows a collision (which is a consequence, not a cause).
        pre_samples = []
        for i in range(self.n):
            t_first = self._first_contact[i]["t"] if i in self._first_contact else float("inf")
            pre_samples.append(np.array([z for ts, z in self._z[i] if ts < t_first], dtype=float))
        altitude = self._altitude_block(all_samples)
        per_drone = altitude.pop("per_drone")
        altitude["pre_contact"] = self._altitude_block(pre_samples)
        altitude["pre_contact"].pop("per_drone", None)
        roots = {cause: 0 for cause in ROOT_CAUSES}
        for rec in self._first_contact.values():
            if rec["root_cause"] in roots:
                roots[rec["root_cause"]] += 1
        return {
            "altitude": altitude, "per_drone": per_drone,
            "contacts": {
                "first_contact_root_cause_per_drone": roots,
                "post_contact_ground_events": self._post_contact_ground,
                "events_by_category": dict(self._events_by_category),
                "first_contacts": sorted(self._first_contact.values(), key=lambda r: r["drone"]),
                "drones_with_a_contact": len(self._first_contact),
            },
        }
