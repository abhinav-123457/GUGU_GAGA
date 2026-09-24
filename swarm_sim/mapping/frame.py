"""Phase 16A: mission area, launch/landing zone and the map that ties them to
the 1x1 m grid. See docs/PHASE16A_MISSION_FRAME_GRID.md.

Pure geometry - no randomness, no clock, no simulator, no sensors.

FRAME. Everything here is expressed in `contracts.Frame.LOCAL_ENU`, used as a
*mission-local* right-handed z-up frame whose origin is the launch-zone
centre. It is NOT geographic east/north: with GNSS denied there is no
absolute reference, so "x" and "y" are just the two axes the team fixed at
the launch zone. (`Frame.LOCAL_ENU` is reused only so the existing
`GeofenceSpec` / `VehicleState` contracts accept these numbers unchanged.)

Assumptions inferred from the competition rules (not stated by them) -
recorded in docs/PHASE16_GNSS_DENIED_ROADMAP.md's assumptions register:
  * the launch zone lies inside the search area,
  * the grid is axis-aligned with the mission frame,
  * the frame origin is the launch-zone centre,
  * the search area is an axis-aligned rectangle.
"""
from __future__ import annotations

import dataclasses
import hashlib
import json
import math
from typing import Optional, Tuple

from ..contracts import Frame, GeofenceSpec
from .grid import GridSpec

Vec2 = Tuple[float, float]

FT_TO_M = 0.3048
LAUNCH_ZONE_SIDE_M = 12 * FT_TO_M   # 12 ft x 12 ft = 3.6576 m, per the competition rules

_MAP_ID_HEX_CHARS = 12


def _finite(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


@dataclasses.dataclass(frozen=True)
class MissionArea:
    """Axis-aligned rectangular search area. Sign convention for `inset_m`
    (here and in `LaunchZone`): positive shrinks the region, so a point is
    "inside with margin" - the opposite sign to
    `sar_world.RectangularSearchArea.contains(margin_m=...)`."""
    min_x_m: float
    min_y_m: float
    max_x_m: float
    max_y_m: float

    def __post_init__(self):
        for name in ("min_x_m", "min_y_m", "max_x_m", "max_y_m"):
            if not _finite(getattr(self, name)):
                raise ValueError(f"{name} must be a finite number, got {getattr(self, name)!r}")
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

    def contains(self, xy: Vec2, inset_m: float = 0.0) -> bool:
        x, y = xy
        return (self.min_x_m + inset_m <= x <= self.max_x_m - inset_m
                and self.min_y_m + inset_m <= y <= self.max_y_m - inset_m)

    def inset(self, inset_m: float) -> "MissionArea":
        """The area shrunk by `inset_m` on every side (raises if it collapses)."""
        return MissionArea(self.min_x_m + inset_m, self.min_y_m + inset_m,
                           self.max_x_m - inset_m, self.max_y_m - inset_m)

    def to_geofence(self, floor_alt_m: float, ceiling_alt_m: float, inset_m: float = 0.0) -> GeofenceSpec:
        """A `contracts.GeofenceSpec` matching this area shrunk by `inset_m`."""
        area = self.inset(inset_m) if inset_m else self
        cx, cy = area.center_m
        return GeofenceSpec(
            frame=Frame.LOCAL_ENU, center_m=(cx, cy),
            half_extents_m=(area.width_m / 2.0, area.height_m / 2.0),
            floor_alt_m=floor_alt_m, ceiling_alt_m=ceiling_alt_m,
        )

    def to_dict(self) -> dict:
        return {"min_x_m": self.min_x_m, "min_y_m": self.min_y_m,
                "max_x_m": self.max_x_m, "max_y_m": self.max_y_m}

    @classmethod
    def from_dict(cls, data: dict) -> "MissionArea":
        return cls(float(data["min_x_m"]), float(data["min_y_m"]), float(data["max_x_m"]), float(data["max_y_m"]))


@dataclasses.dataclass(frozen=True)
class LaunchZone:
    """The square take-off / landing area. `yaw_rad` rotates it about its
    centre (counter-clockwise, mission frame); 0 = sides parallel to the axes."""
    center_m: Vec2 = (0.0, 0.0)
    side_m: float = LAUNCH_ZONE_SIDE_M
    yaw_rad: float = 0.0

    def __post_init__(self):
        if not (isinstance(self.center_m, tuple) and len(self.center_m) == 2
                and all(_finite(v) for v in self.center_m)):
            raise ValueError(f"center_m must be a 2-tuple of finite numbers, got {self.center_m!r}")
        if not (_finite(self.side_m) and self.side_m > 0.0):
            raise ValueError(f"side_m must be a finite number > 0, got {self.side_m!r}")
        if not _finite(self.yaw_rad):
            raise ValueError(f"yaw_rad must be a finite number, got {self.yaw_rad!r}")

    def _to_zone_frame(self, xy: Vec2) -> Vec2:
        dx, dy = xy[0] - self.center_m[0], xy[1] - self.center_m[1]
        c, s = math.cos(self.yaw_rad), math.sin(self.yaw_rad)
        return (c * dx + s * dy, -s * dx + c * dy)

    def _from_zone_frame(self, local: Vec2) -> Vec2:
        c, s = math.cos(self.yaw_rad), math.sin(self.yaw_rad)
        return (self.center_m[0] + c * local[0] - s * local[1],
                self.center_m[1] + s * local[0] + c * local[1])

    def contains(self, xy: Vec2, inset_m: float = 0.0) -> bool:
        """True if `xy` is inside the zone by at least `inset_m`."""
        lx, ly = self._to_zone_frame(xy)
        limit = self.side_m / 2.0 - inset_m
        return abs(lx) <= limit and abs(ly) <= limit

    def corners(self) -> Tuple[Vec2, Vec2, Vec2, Vec2]:
        h = self.side_m / 2.0
        return tuple(self._from_zone_frame(p) for p in ((-h, -h), (h, -h), (h, h), (-h, h)))

    def slots(self, n: int, min_spacing_m: float, edge_margin_m: float = 0.0) -> Tuple[Vec2, ...]:
        """`n` deterministic take-off / landing slots inside the zone.

        The slots are the first `n` points (row-major) of a `cols x rows`
        lattice spanning the zone shrunk by `edge_margin_m`; among lattices
        with `cols * rows >= n` the one with the largest achieved spacing
        wins (ties go to fewer columns). Raises ValueError if no lattice
        reaches `min_spacing_m` - the rules force every drone into this one
        small zone, so infeasible fleets must fail loudly, not silently
        overlap."""
        if not (isinstance(n, int) and not isinstance(n, bool) and n >= 1):
            raise ValueError(f"n must be an int >= 1, got {n!r}")
        if not (_finite(min_spacing_m) and min_spacing_m >= 0.0):
            raise ValueError(f"min_spacing_m must be a finite number >= 0, got {min_spacing_m!r}")
        if not (_finite(edge_margin_m) and edge_margin_m >= 0.0):
            raise ValueError(f"edge_margin_m must be a finite number >= 0, got {edge_margin_m!r}")
        usable = self.side_m - 2.0 * edge_margin_m
        if usable < 0.0 or (n > 1 and usable == 0.0):
            raise ValueError(f"edge_margin_m={edge_margin_m!r} leaves no usable zone (side {self.side_m!r} m)")
        if n == 1:
            return (self._from_zone_frame((0.0, 0.0)),)

        best = None   # (achieved_spacing, -cols, cols, rows)
        for cols in range(1, n + 1):
            rows = math.ceil(n / cols)
            spacings = [usable / (k - 1) for k in (cols, rows) if k > 1]
            achieved = min(spacings)
            key = (achieved, -cols)
            if best is None or key > best[0]:
                best = (key, cols, rows)
        (achieved, _), cols, rows = best
        if achieved < min_spacing_m:
            raise ValueError(
                f"cannot place {n} slots at >= {min_spacing_m!r} m spacing in a {self.side_m!r} m zone with "
                f"{edge_margin_m!r} m edge margin (best achievable spacing {achieved!r} m)"
            )
        sx = usable / (cols - 1) if cols > 1 else 0.0
        sy = usable / (rows - 1) if rows > 1 else 0.0
        x0 = -usable / 2.0 if cols > 1 else 0.0
        y0 = -usable / 2.0 if rows > 1 else 0.0
        slots = []
        for k in range(n):
            r, c = divmod(k, cols)
            slots.append(self._from_zone_frame((x0 + c * sx, y0 + r * sy)))
        return tuple(slots)

    def to_dict(self) -> dict:
        return {"center_m": list(self.center_m), "side_m": self.side_m, "yaw_rad": self.yaw_rad}

    @classmethod
    def from_dict(cls, data: dict) -> "LaunchZone":
        return cls(center_m=(float(data["center_m"][0]), float(data["center_m"][1])),
                   side_m=float(data["side_m"]), yaw_rad=float(data["yaw_rad"]))


@dataclasses.dataclass(frozen=True)
class MissionMap:
    """Search area + launch zone + 1x1 m grid, in one mission frame."""
    area: MissionArea
    launch_zone: LaunchZone
    grid: GridSpec
    allow_zone_outside_area: bool = False

    def __post_init__(self):
        if not self.allow_zone_outside_area:
            outside = [c for c in self.launch_zone.corners() if not self.area.contains(c)]
            if outside:
                raise ValueError(
                    f"launch zone corner(s) {outside!r} lie outside the search area; "
                    f"pass allow_zone_outside_area=True if that is intended"
                )
        gx1, gy1 = self.grid.max_xy_m
        tol = 1e-9 * max(1.0, self.grid.cell_size_m)
        if (self.grid.origin_xy_m[0] > self.area.min_x_m + tol or self.grid.origin_xy_m[1] > self.area.min_y_m + tol
                or gx1 < self.area.max_x_m - tol or gy1 < self.area.max_y_m - tol):
            raise ValueError("grid does not cover the whole search area")

    @classmethod
    def centered_square(cls, arena_size_m: float, cell_m: float = 1.0,
                        launch_zone: Optional[LaunchZone] = None) -> "MissionMap":
        """Square area centred on the origin - the legacy `FloodSearchMission`
        arena (`arena_size / 2` half extent about (0, 0)) - with the launch
        zone at the origin unless given."""
        half = arena_size_m / 2.0
        area = MissionArea(-half, -half, half, half)
        return cls(area=area, launch_zone=launch_zone or LaunchZone(),
                   grid=GridSpec.from_area(area, cell_m))

    @classmethod
    def zone_at_corner(cls, width_m: float, height_m: float, corner: str = "sw", cell_m: float = 1.0,
                       side_m: float = LAUNCH_ZONE_SIDE_M) -> "MissionMap":
        """Rectangular `width_m` x `height_m` area with the launch zone (centred
        on the frame origin) flush against one corner: corner is one of
        "sw", "se", "nw", "ne". The area extends away from the zone's outer edges."""
        if corner not in ("sw", "se", "nw", "ne"):
            raise ValueError(f"corner must be one of 'sw', 'se', 'nw', 'ne', got {corner!r}")
        h = side_m / 2.0
        if corner[1] == "w":
            min_x, max_x = -h, -h + width_m
        else:
            min_x, max_x = h - width_m, h
        if corner[0] == "s":
            min_y, max_y = -h, -h + height_m
        else:
            min_y, max_y = h - height_m, h
        area = MissionArea(min_x, min_y, max_x, max_y)
        return cls(area=area, launch_zone=LaunchZone(side_m=side_m), grid=GridSpec.from_area(area, cell_m))

    def to_geofence(self, floor_alt_m: float, ceiling_alt_m: float, inset_m: float = 0.0) -> GeofenceSpec:
        return self.area.to_geofence(floor_alt_m, ceiling_alt_m, inset_m)

    def to_dict(self) -> dict:
        return {"area": self.area.to_dict(), "launch_zone": self.launch_zone.to_dict(),
                "grid": self.grid.to_dict(), "allow_zone_outside_area": self.allow_zone_outside_area}

    @classmethod
    def from_dict(cls, data: dict) -> "MissionMap":
        return cls(area=MissionArea.from_dict(data["area"]), launch_zone=LaunchZone.from_dict(data["launch_zone"]),
                   grid=GridSpec.from_dict(data["grid"]),
                   allow_zone_outside_area=bool(data["allow_zone_outside_area"]))

    @property
    def map_id(self) -> str:
        """12-hex-char content hash of the map (canonical JSON), for
        `build_run_manifest(map_id=...)` - same map, same id, on any machine."""
        canonical = json.dumps(self.to_dict(), sort_keys=True, separators=(",", ":"))
        return hashlib.sha256(canonical.encode("utf-8")).hexdigest()[:_MAP_ID_HEX_CHARS]
