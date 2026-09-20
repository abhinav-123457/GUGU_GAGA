"""Phase 14: hidden-ground-truth SAR world + sensor observation generation
for a single vehicle. See docs/PHASE14_SAR_WEBOTS.md.

Mirrors swarm_sim/sensors.py's own isolation rule exactly, just for one
vehicle instead of a swarm:

    Hidden victim ground truth (THIS MODULE's _victims_gt - never public)
                    |
                    v
          VictimSensorModel.sense()          (swarm_sim.sensors, reused unmodified)
                    |
                    v
          SensorObservation / DetectionCandidate   (swarm_sim.contracts)
                    |
                    v
          SAR mission / search planner              (never sees ground truth)

`SARWorld._victims_gt` is the ONLY place a real victim position exists in
this module's public surface - every test in
tests/test_phase14_sar_mission.py::TestHiddenTruthIsolation and every AST
check in tests/test_phase14_sar_architecture.py exist specifically to prove
sar_search.py/sar_mission.py's planner-facing code never reads it.
"""
from __future__ import annotations

import dataclasses
import math
from typing import Iterable, List, Optional, Tuple

import numpy as np

from .contracts import DetectionCandidate, Frame, GeofenceSpec, SensorObservation, VictimGroundTruth
from .sensors import VictimSensorModel

Vec2 = Tuple[float, float]
Vec3 = Tuple[float, float, float]


@dataclasses.dataclass(frozen=True)
class RectangularSearchArea:
    """Axis-aligned bounded rectangle - the only search-area shape this
    phase supports (no polygon, no obstacles - see module docstring of
    sar_search.py)."""
    min_x_m: float
    min_y_m: float
    max_x_m: float
    max_y_m: float

    def __post_init__(self):
        if not (self.max_x_m > self.min_x_m):
            raise ValueError(f"max_x_m ({self.max_x_m}) must be > min_x_m ({self.min_x_m})")
        if not (self.max_y_m > self.min_y_m):
            raise ValueError(f"max_y_m ({self.max_y_m}) must be > min_y_m ({self.min_y_m})")

    @property
    def width_m(self) -> float:
        return self.max_x_m - self.min_x_m

    @property
    def height_m(self) -> float:
        return self.max_y_m - self.min_y_m

    @property
    def center_m(self) -> Vec2:
        return ((self.min_x_m + self.max_x_m) / 2.0, (self.min_y_m + self.max_y_m) / 2.0)

    def contains(self, xy: Vec2, margin_m: float = 0.0) -> bool:
        x, y = xy
        return (self.min_x_m - margin_m <= x <= self.max_x_m + margin_m
                and self.min_y_m - margin_m <= y <= self.max_y_m + margin_m)

    def to_geofence(self, floor_alt_m: float, ceiling_alt_m: float, margin_m: float = 0.0) -> GeofenceSpec:
        """A geofence exactly matching this search area (plus an optional
        symmetric margin) - the search waypoints (sar_search.py) are always
        generated INSIDE this same area, so they can never lie outside the
        geofence built from it."""
        cx, cy = self.center_m
        return GeofenceSpec(
            frame=Frame.LOCAL_ENU, center_m=(cx, cy),
            half_extents_m=(self.width_m / 2.0 + margin_m, self.height_m / 2.0 + margin_m),
            floor_alt_m=floor_alt_m, ceiling_alt_m=ceiling_alt_m,
        )


@dataclasses.dataclass
class SensorModelConfig:
    """Only the fields swarm_sim.sensors.VictimSensorModel actually reads
    (it duck-types its `config` argument) - deliberately NOT
    swarm_sim.config.MissionConfig, which is a much larger 6-drone,
    obstacle-avoidance-laden config this one-vehicle phase has no use for.
    Defaults mirror MissionConfig's own victim_sensor_* defaults so the
    detection model behaves the same way it already does in the tested
    6-drone mission."""
    victim_sensor_range_m: float = 8.0
    victim_sensor_hfov_deg: float = 90.0
    victim_sensor_vfov_deg: float = 140.0
    victim_sensor_noise_std_m: float = 0.4
    victim_sensor_false_negative_prob: float = 0.08
    victim_sensor_false_positive_rate: float = 0.01
    victim_sensor_latency_steps: int = 1
    victim_sensor_dropout_prob: float = 0.03
    victim_sensor_confidence: float = 0.85
    victim_sensor_false_positive_confidence: float = 0.45
    victim_sensor_localization_uncertainty_m: float = 0.4
    obstacle_radius: float = 0.5   # unused this phase (no obstacles) - VictimSensorModel still reads this attribute


class SARWorld:
    """Owns hidden victim ground truth and produces this tick's
    SensorObservation for one vehicle, via an unmodified
    swarm_sim.sensors.VictimSensorModel (constructed with num_drones=1, no
    obstacles). See module docstring for the isolation guarantee."""

    def __init__(self, search_area: RectangularSearchArea, victim_positions_m: Iterable[Vec2], rng,
                 sensor_config: Optional[SensorModelConfig] = None, vehicle_id: str = "drone0"):
        self.search_area = search_area
        self.vehicle_id = vehicle_id
        self._sensor_cfg = sensor_config or SensorModelConfig()
        self._victims_gt: List[VictimGroundTruth] = [
            VictimGroundTruth(victim_id=f"victim-{i}", position_m=(float(x), float(y)), found=False)
            for i, (x, y) in enumerate(victim_positions_m)
        ]
        victims_xy = np.array([v.position_m for v in self._victims_gt], dtype=float).reshape(-1, 2)
        self._sensor = VictimSensorModel(self._sensor_cfg, num_drones=1, victims_xy=victims_xy,
                                          obstacles_xy=np.zeros((0, 2)), rng=rng)

    # -- scoring-only aggregates (counts/ids only, never a position) --------

    @property
    def victim_ground_truth_count(self) -> int:
        return len(self._victims_gt)

    @property
    def confirmed_victim_ids(self) -> Tuple[str, ...]:
        return tuple(v.victim_id for v in self._victims_gt if v.found)

    @property
    def missed_victim_count(self) -> int:
        return sum(1 for v in self._victims_gt if not v.found)

    # -- planner-facing: observations only -----------------------------------

    def observe(self, drone_xyz: Vec3, heading_xy: Vec2, t_s: float) -> SensorObservation:
        """The only method the mission/search layer calls each tick -
        returns a contracts.SensorObservation, never a raw position."""
        already_found = [v.found for v in self._victims_gt]
        detections, dropped = self._sensor.sense(0, drone_xyz, heading_xy, t_s, already_found)
        return SensorObservation(
            vehicle_id=self.vehicle_id, sensor_timestamp_s=t_s, sensor_latency_s=0.0,
            fov_deg=self._sensor_cfg.victim_sensor_hfov_deg,
            range_returns_m=(), occluded=(), dropout=dropped,
            pose_uncertainty_m=0.0, detections=detections,
        )

    # -- scoring only: called by the MISSION layer, never the planner -------

    def mark_found_near(self, xy: Vec2, radius_m: float) -> Optional[str]:
        """Scoring/bookkeeping only - called by sar_mission.py once a
        cluster of detections is CONFIRMED, using the confirmed cluster's
        own BELIEVED position (never hidden truth) as `xy`. Marks the
        nearest still-hidden victim within `radius_m` as found (so
        VictimSensorModel stops generating further reports for it, exactly
        like FloodSearchMission's own already_found_mask) and returns its
        victim_id - or None if no hidden victim was actually there, which
        is this world's own honest way of flagging a false confirmation to
        the mission layer's scoring, without ever handing a position back."""
        best_idx, best_d = None, radius_m
        for idx, v in enumerate(self._victims_gt):
            if v.found:
                continue
            d = math.dist(v.position_m, xy)
            if d <= best_d:
                best_idx, best_d = idx, d
        if best_idx is None:
            return None
        found = self._victims_gt[best_idx]
        self._victims_gt[best_idx] = dataclasses.replace(found, found=True)
        return found.victim_id
