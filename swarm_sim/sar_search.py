"""Phase 14: deterministic boustrophedon/lawnmower search pattern for a
single vehicle. See docs/PHASE14_SAR_WEBOTS.md.

No obstacle avoidance, no swarm coordination, no optimization claim, and
no guarantee of victim detection - this is a fixed, deterministic coverage
path over a bounded rectangle, nothing more. This module never imports
swarm_sim.sar_world's hidden-truth-holding attributes and never receives a
victim position - it only ever produces plain (x, y, z) waypoints from the
search area's own public bounds.
"""
from __future__ import annotations

import dataclasses
import math
from typing import Optional, Tuple

from .sar_world import RectangularSearchArea

Vec3 = Tuple[float, float, float]


def generate_lawnmower_waypoints(area: RectangularSearchArea, altitude_m: float, lane_spacing_m: float,
                                  margin_m: float = 0.0) -> Tuple[Vec3, ...]:
    """Boustrophedon coverage: parallel lanes along x, alternating
    direction each lane, spaced `lane_spacing_m` apart in y. Every waypoint
    satisfies `area.contains(xy, margin_m=0.0)` by construction (each
    coordinate is clamped into [min+margin, max-margin], itself a subset of
    `area`) - deterministic: the same (area, altitude_m, lane_spacing_m,
    margin_m) always produces the same sequence, no RNG involved.
    """
    if lane_spacing_m <= 0.0:
        raise ValueError(f"lane_spacing_m must be > 0, got {lane_spacing_m!r}")
    if margin_m < 0.0:
        raise ValueError(f"margin_m must be >= 0, got {margin_m!r}")

    x_min, x_max = area.min_x_m + margin_m, area.max_x_m - margin_m
    y_min, y_max = area.min_y_m + margin_m, area.max_y_m - margin_m
    if x_min > x_max or y_min > y_max:
        raise ValueError(f"margin_m={margin_m!r} leaves no room inside the search area")

    waypoints = []
    y = y_min
    lane_index = 0
    while y <= y_max + 1e-9:
        clamped_y = min(y, y_max)
        if lane_index % 2 == 0:
            waypoints.append((x_min, clamped_y, altitude_m))
            waypoints.append((x_max, clamped_y, altitude_m))
        else:
            waypoints.append((x_max, clamped_y, altitude_m))
            waypoints.append((x_min, clamped_y, altitude_m))
        lane_index += 1
        y += lane_spacing_m
    return tuple(waypoints)


def velocity_toward(current_xyz: Vec3, target_xyz: Vec3, max_speed_mps: float) -> Vec3:
    """A plain proportional velocity vector from `current_xyz` straight at
    `target_xyz`, capped at `max_speed_mps` - the only "guidance law" this
    phase uses. Returns (0, 0, 0) if already effectively at the target."""
    dx = target_xyz[0] - current_xyz[0]
    dy = target_xyz[1] - current_xyz[1]
    dz = target_xyz[2] - current_xyz[2]
    dist = math.sqrt(dx * dx + dy * dy + dz * dz)
    if dist < 1e-6:
        return (0.0, 0.0, 0.0)
    scale = min(max_speed_mps, dist) / dist
    return (dx * scale, dy * scale, dz * scale)


@dataclasses.dataclass
class LawnmowerSearchPattern:
    """Stateful waypoint follower over a fixed, precomputed waypoint
    sequence - index-only state, trivially deterministic/reproducible."""
    waypoints: Tuple[Vec3, ...]
    reach_tolerance_m: float = 0.5
    _index: int = 0

    @property
    def current_waypoint(self) -> Optional[Vec3]:
        if self.is_complete():
            return None
        return self.waypoints[self._index]

    def is_complete(self) -> bool:
        return self._index >= len(self.waypoints)

    def advance_if_reached(self, position_xyz: Vec3) -> bool:
        """If the current waypoint is within `reach_tolerance_m` of
        `position_xyz`, advances to the next one and returns True.
        Otherwise returns False without changing state."""
        if self.is_complete():
            return False
        if math.dist(position_xyz, self.waypoints[self._index]) <= self.reach_tolerance_m:
            self._index += 1
            return True
        return False

    @property
    def waypoints_completed(self) -> int:
        return min(self._index, len(self.waypoints))

    @property
    def waypoint_count(self) -> int:
        return len(self.waypoints)
