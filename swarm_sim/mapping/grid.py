"""Phase 16A: the 1x1 m mission grid. See docs/PHASE16A_MISSION_FRAME_GRID.md.

Pure geometry - no randomness, no clock, no simulator, no sensors. A survivor
is reported as the grid cell it was found in, so the rules below (which cell
owns a boundary point, how a cell is named) are part of the mission's
externally visible behaviour and are pinned by tests/test_phase16a_*.py.

Conventions (inferred from the competition rules, not stated by them):
  * The grid is axis-aligned with the mission frame; cell (0, 0) is the
    cell at the frame's minimum-x / minimum-y corner of the search area.
  * Cells are half-open: a cell owns its lower x / y edge and not its upper
    one, EXCEPT that the far edge of the whole grid belongs to the last
    cell so every point of a closed search area is in exactly one cell.
  * A point within `_SNAP_CELLS` (1e-9 of a cell) of a grid line is snapped
    onto that line before the rule above is applied, so a coordinate
    computed as `origin + k * cell_size` always lands in cell k regardless
    of floating-point rounding.
"""
from __future__ import annotations

import dataclasses
import math
import re
from typing import TYPE_CHECKING, List, NamedTuple, Optional, Tuple

if TYPE_CHECKING:  # avoid a runtime import cycle - frame.py imports this module
    from .frame import MissionArea

Vec2 = Tuple[float, float]

_SNAP_CELLS = 1e-9
_CELL_ID_RE = re.compile(r"^X(\d+)Y(\d+)$")


class CellIndex(NamedTuple):
    ix: int   # column, increasing with +x
    iy: int   # row, increasing with +y


@dataclasses.dataclass(frozen=True)
class GridSpec:
    origin_xy_m: Vec2       # minimum-x / minimum-y corner of cell (0, 0)
    cell_size_m: float
    n_cols: int
    n_rows: int

    def __post_init__(self):
        if not (isinstance(self.origin_xy_m, tuple) and len(self.origin_xy_m) == 2
                and all(isinstance(v, (int, float)) and math.isfinite(v) for v in self.origin_xy_m)):
            raise ValueError(f"origin_xy_m must be a 2-tuple of finite numbers, got {self.origin_xy_m!r}")
        if not (isinstance(self.cell_size_m, (int, float)) and math.isfinite(self.cell_size_m)
                and self.cell_size_m > 0.0):
            raise ValueError(f"cell_size_m must be a finite number > 0, got {self.cell_size_m!r}")
        for name, value in (("n_cols", self.n_cols), ("n_rows", self.n_rows)):
            if not (isinstance(value, int) and not isinstance(value, bool) and value >= 1):
                raise ValueError(f"{name} must be an int >= 1, got {value!r}")

    # -- construction -------------------------------------------------------

    @classmethod
    def from_area(cls, area: "MissionArea", cell_size_m: float = 1.0, allow_partial: bool = False) -> "GridSpec":
        """Grid whose cell (0, 0) starts at the area's minimum corner. Unless
        `allow_partial`, the area's sides must be whole multiples of the
        cell size (so no cell hangs outside the search area); with it, the
        last column / row is the one that reaches or passes the far edge."""
        if not (isinstance(cell_size_m, (int, float)) and math.isfinite(cell_size_m) and cell_size_m > 0.0):
            raise ValueError(f"cell_size_m must be a finite number > 0, got {cell_size_m!r}")
        counts = []
        for side_m, label in ((area.max_x_m - area.min_x_m, "width"), (area.max_y_m - area.min_y_m, "height")):
            q = side_m / cell_size_m
            nearest = round(q)
            if nearest >= 1 and abs(q - nearest) <= _SNAP_CELLS:
                counts.append(int(nearest))
            elif allow_partial:
                counts.append(max(1, math.ceil(q)))
            else:
                raise ValueError(
                    f"area {label} {side_m!r} m is not a whole multiple of cell_size_m={cell_size_m!r}; "
                    f"pass allow_partial=True to let the last cell overhang"
                )
        return cls(origin_xy_m=(float(area.min_x_m), float(area.min_y_m)),
                   cell_size_m=float(cell_size_m), n_cols=counts[0], n_rows=counts[1])

    # -- extent -------------------------------------------------------------

    @property
    def n_cells(self) -> int:
        return self.n_cols * self.n_rows

    @property
    def max_xy_m(self) -> Vec2:
        return (self.origin_xy_m[0] + self.n_cols * self.cell_size_m,
                self.origin_xy_m[1] + self.n_rows * self.cell_size_m)

    def contains_cell(self, cell: CellIndex) -> bool:
        return 0 <= cell.ix < self.n_cols and 0 <= cell.iy < self.n_rows

    def _require_cell(self, cell: CellIndex) -> None:
        if not self.contains_cell(cell):
            raise ValueError(f"cell {tuple(cell)!r} is outside the {self.n_cols}x{self.n_rows} grid")

    # -- point <-> cell -----------------------------------------------------

    def _axis_index(self, value: float, origin: float, n: int) -> Optional[int]:
        if not math.isfinite(value):
            return None
        q = (value - origin) / self.cell_size_m
        if not math.isfinite(q):
            return None
        nearest = round(q)
        if abs(q - nearest) <= _SNAP_CELLS:
            k = nearest
            if k == n:        # a point ON the grid's far edge belongs to the last cell...
                k = n - 1
        else:
            k = math.floor(q)  # ...but one beyond it (k >= n here) is outside the grid
        return int(k) if 0 <= k < n else None

    def cell_of(self, xy: Vec2) -> Optional[CellIndex]:
        """The cell owning `xy`, or None if `xy` is outside the grid (or not
        finite). See the module docstring for the boundary rule."""
        ix = self._axis_index(float(xy[0]), self.origin_xy_m[0], self.n_cols)
        iy = self._axis_index(float(xy[1]), self.origin_xy_m[1], self.n_rows)
        if ix is None or iy is None:
            return None
        return CellIndex(ix, iy)

    def cell_bounds(self, cell: CellIndex) -> Tuple[float, float, float, float]:
        """(min_x, min_y, max_x, max_y) of the cell."""
        self._require_cell(cell)
        x0 = self.origin_xy_m[0] + cell.ix * self.cell_size_m
        y0 = self.origin_xy_m[1] + cell.iy * self.cell_size_m
        return (x0, y0, x0 + self.cell_size_m, y0 + self.cell_size_m)

    def cell_center(self, cell: CellIndex) -> Vec2:
        x0, y0, x1, y1 = self.cell_bounds(cell)
        return ((x0 + x1) / 2.0, (y0 + y1) / 2.0)

    # -- naming (what the GCS shows) ---------------------------------------

    def cell_id(self, cell: CellIndex) -> str:
        """Stable human-readable name, e.g. cell (7, 12) -> "X07Y12". Index
        digits widen past 99 rather than truncate."""
        self._require_cell(cell)
        return f"X{cell.ix:02d}Y{cell.iy:02d}"

    def parse_cell_id(self, cell_id: str) -> CellIndex:
        match = _CELL_ID_RE.match(cell_id) if isinstance(cell_id, str) else None
        if match is None:
            raise ValueError(f"not a cell id (expected like 'X07Y12'): {cell_id!r}")
        cell = CellIndex(int(match.group(1)), int(match.group(2)))
        self._require_cell(cell)
        return cell

    # -- iteration / neighbourhoods ----------------------------------------

    def iter_cells(self):
        """Row-major (iy outer, ix inner) - the same order as flat_index."""
        for iy in range(self.n_rows):
            for ix in range(self.n_cols):
                yield CellIndex(ix, iy)

    def flat_index(self, cell: CellIndex) -> int:
        self._require_cell(cell)
        return cell.iy * self.n_cols + cell.ix

    def from_flat(self, k: int) -> CellIndex:
        if not (isinstance(k, int) and not isinstance(k, bool) and 0 <= k < self.n_cells):
            raise ValueError(f"flat index must be an int in [0, {self.n_cells}), got {k!r}")
        return CellIndex(k % self.n_cols, k // self.n_cols)

    def neighbors(self, cell: CellIndex, connectivity: int = 4) -> Tuple[CellIndex, ...]:
        """In-grid neighbours in row-major order. connectivity is 4 (edge
        neighbours) or 8 (edge + diagonal)."""
        if connectivity not in (4, 8):
            raise ValueError(f"connectivity must be 4 or 8, got {connectivity!r}")
        self._require_cell(cell)
        found: List[CellIndex] = []
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if (dx, dy) == (0, 0) or (connectivity == 4 and dx != 0 and dy != 0):
                    continue
                candidate = CellIndex(cell.ix + dx, cell.iy + dy)
                if self.contains_cell(candidate):
                    found.append(candidate)
        return tuple(found)

    def cells_overlapping_disc(self, xy: Vec2, radius_m: float) -> Tuple[CellIndex, ...]:
        """Every in-grid cell whose rectangle intersects (or touches) the
        disc of radius `radius_m` about `xy`, row-major. `xy` may lie outside
        the grid - the disc can still overlap edge cells."""
        if not (isinstance(radius_m, (int, float)) and math.isfinite(radius_m) and radius_m >= 0.0):
            raise ValueError(f"radius_m must be a finite number >= 0, got {radius_m!r}")
        if not (math.isfinite(xy[0]) and math.isfinite(xy[1])):
            raise ValueError(f"xy must be finite, got {xy!r}")
        cs = self.cell_size_m
        ox, oy = self.origin_xy_m
        # Candidate range widened by one cell each side so floating-point
        # rounding at a grid line can never drop a touching cell - the exact
        # nearest-point test below is the real filter.
        ix_lo = max(0, math.floor((xy[0] - radius_m - ox) / cs) - 1)
        ix_hi = min(self.n_cols - 1, math.floor((xy[0] + radius_m - ox) / cs) + 1)
        iy_lo = max(0, math.floor((xy[1] - radius_m - oy) / cs) - 1)
        iy_hi = min(self.n_rows - 1, math.floor((xy[1] + radius_m - oy) / cs) + 1)
        found: List[CellIndex] = []
        for iy in range(iy_lo, iy_hi + 1):
            for ix in range(ix_lo, ix_hi + 1):
                x0, y0, x1, y1 = self.cell_bounds(CellIndex(ix, iy))
                nearest_x = min(max(xy[0], x0), x1)
                nearest_y = min(max(xy[1], y0), y1)
                if math.hypot(xy[0] - nearest_x, xy[1] - nearest_y) <= radius_m:
                    found.append(CellIndex(ix, iy))
        return tuple(found)

    # -- serialisation ------------------------------------------------------

    def to_dict(self) -> dict:
        return {"origin_xy_m": list(self.origin_xy_m), "cell_size_m": self.cell_size_m,
                "n_cols": self.n_cols, "n_rows": self.n_rows}

    @classmethod
    def from_dict(cls, data: dict) -> "GridSpec":
        return cls(origin_xy_m=(float(data["origin_xy_m"][0]), float(data["origin_xy_m"][1])),
                   cell_size_m=float(data["cell_size_m"]),
                   n_cols=int(data["n_cols"]), n_rows=int(data["n_rows"]))
