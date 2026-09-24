# Phase 16A: mission frame, launch zone and 1x1 m grid

**Status: built and tested (142 tests). Pure geometry - no existing module
was changed, and nothing in the running simulation uses it yet.** Phase 16B
starts wiring it into `FloodSearchMission`. See
[PHASE16_GNSS_DENIED_ROADMAP.md](PHASE16_GNSS_DENIED_ROADMAP.md) for why and
for the assumptions register.

## What was built

`swarm_sim/mapping/` (standard library + `contracts` only):

- **`frame.py`**
  - `MissionArea` - axis-aligned rectangle; `contains`, `inset`,
    `to_geofence(floor, ceiling, inset_m)` returning the existing
    `contracts.GeofenceSpec`.
  - `LaunchZone` - 12 ft x 12 ft (3.6576 m) square by default, optionally
    offset and rotated. `contains`, `corners`, and `slots(n, min_spacing_m,
    edge_margin_m)`: a deterministic lattice of take-off/landing positions
    that **raises** when the fleet cannot fit at the requested spacing.
  - `MissionMap(area, launch_zone, grid)` - validates that the zone is inside
    the area and the grid covers it; `centered_square` reproduces the legacy
    `FloodSearchMission` arena exactly; `zone_at_corner` puts the zone flush
    against a corner; `to_dict`/`from_dict`; `map_id` (12-hex content hash for
    `build_run_manifest(map_id=...)`).
- **`grid.py`** - `CellIndex`, `GridSpec`: `from_area`, `cell_of`,
  `cell_bounds`, `cell_center`, `cell_id`/`parse_cell_id` (`"X07Y12"`),
  `iter_cells`, `flat_index`/`from_flat`, `neighbors` (4 or 8),
  `cells_overlapping_disc`.
- **`swarm_sim/mission_rules.py`** - `MIN_DRONES`, `MAX_SURVIVORS`,
  `MAX_TOTAL_MASS_KG`, `KIT_MASS_KG`, `KIT_DIMS_M`, `check_fleet_mass`.

### Frame

Everything uses `Frame.LOCAL_ENU` as a **mission-local right-handed z-up
frame with its origin at the launch-zone centre**. It is not geographic
east/north - with GNSS denied there is no absolute reference. `LOCAL_ENU` is
reused only so `GeofenceSpec`/`VehicleState` accept the numbers unchanged.

### Cell rule

Cells are half-open: a cell owns its lower x/y edges. The far edge of the
grid belongs to the last cell, so every point of a closed search area is in
exactly one cell; anything beyond it has no cell. A point within 1e-9 of a
cell size from a grid line is snapped onto that line first, so a coordinate
computed as `origin + k * cell_size` always lands in cell `k` despite
floating-point rounding.

### Sign convention for margins

`inset_m` is positive-inwards everywhere in this package (a point is "inside
with margin"). This is the **opposite** sign to
`sar_world.RectangularSearchArea.contains(margin_m=...)`, whose positive
margin expands the area; the parity test converts explicitly.

## Verified (by `tests/test_phase16a_*.py`)

- `cell_of` <-> `cell_center`/lower-corner round trip for every cell,
  including a 0.1 m grid with an inexact origin (the case where naive
  `floor` lands one cell low).
- Boundary rules: interior lines go to the higher cell, far edge to the last
  cell, negative coordinates, outside / NaN / inf / 1e308 give `None`.
- `cells_overlapping_disc` matches a brute-force nearest-point check.
- Zone side is 3.6576 m; rotated-zone containment (a diamond at 45 degrees);
  corners on the boundary at any yaw/offset.
- Slots: inside the zone with margin, pairwise spacing >= requested,
  deterministic, unchanged by rotation/offset except by the rigid motion;
  infeasible fleets raise (10 drones at 1 m spacing with 0.5 m margin does
  not fit; 0.8 m spacing does).
- `MissionMap.centered_square(arena).to_geofence(...)` equals the
  `GeofenceSpec` `FloodSearchMission` builds today for arenas of 40, 20 and
  15 m; `MissionArea.to_geofence` matches `RectangularSearchArea.to_geofence`
  (used only as a test oracle - production `mapping/` does not import
  `sar_world`).
- JSON round-trip preserves equality and `map_id`; `map_id` changes with any
  geometry change.
- AST architecture tests: `mapping/` and `mission_rules.py` import only the
  standard library, numpy and their siblings/`contracts`; no randomness,
  clock, PyBullet, sensors, mission, or ground-truth identifiers; no
  module-level mutable state.

One real bug was found by the tests while building this phase: the first
`cell_of` mapped every point just past the far edge (for example x = 10.001 on
a 10-wide grid) to the last cell instead of rejecting it. Fixed so the
far-edge rule applies only to points on the edge itself.

## Inferred (not stated by the competition rules)

See assumptions A1-A7 in the roadmap doc: zone inside the area, grid
axis-aligned, origin at the zone centre, rectangular area, the cell-edge
tie rule, `X07Y12` naming, and that the zone does not align to cell edges.

## Limitations

- No simulator code uses any of this yet; it becomes live in 16B (geofence
  built from `MissionMap`) and 16C (launch-slot spawn).
- Only axis-aligned rectangular search areas are supported (no polygons).
- `check_fleet_mass` checks declared masses; the simulated drone mass is
  not representative of the real fleet.
