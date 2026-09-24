"""Phase 16A: mission frame, launch zone and 1x1 m grid - see
docs/PHASE16A_MISSION_FRAME_GRID.md. Pure geometry, no simulator."""
import dataclasses
import hashlib
import itertools
import json
import math

import pytest

from swarm_sim import mission_rules
from swarm_sim.contracts import Frame, GeofenceSpec
from swarm_sim.mapping import (
    FT_TO_M, LAUNCH_ZONE_SIDE_M, CellIndex, GridSpec, LaunchZone, MissionArea, MissionMap,
)
from swarm_sim.sar_world import RectangularSearchArea   # test-only parity oracle


def _grid(cols=10, rows=10, origin=(0.0, 0.0), cell=1.0) -> GridSpec:
    return GridSpec(origin_xy_m=origin, cell_size_m=cell, n_cols=cols, n_rows=rows)


# ==========================================================================
# GridSpec: point <-> cell
# ==========================================================================

class TestCellOf:
    def test_round_trip_center_and_lower_corner_for_every_cell(self):
        grid = _grid(cols=7, rows=5, origin=(-3.5, -2.5))
        for cell in grid.iter_cells():
            assert grid.cell_of(grid.cell_center(cell)) == cell
            x0, y0, _, _ = grid.cell_bounds(cell)
            assert grid.cell_of((x0, y0)) == cell

    def test_lower_corner_snaps_to_its_own_cell_despite_float_rounding(self):
        # 0.1-sized cells and an origin that is not representable exactly:
        # origin + k*cell is the classic place plain floor() lands one cell low.
        grid = _grid(cols=26, rows=14, origin=(-1.3, -0.7), cell=0.1)
        for cell in grid.iter_cells():
            x0, y0, _, _ = grid.cell_bounds(cell)
            assert grid.cell_of((x0, y0)) == cell, cell

    def test_interior_boundary_belongs_to_the_higher_cell(self):
        grid = _grid()
        assert grid.cell_of((3.0, 4.0)) == CellIndex(3, 4)
        assert grid.cell_of((2.999, 3.999)) == CellIndex(2, 3)

    def test_far_edge_of_the_grid_belongs_to_the_last_cell(self):
        grid = _grid(cols=4, rows=6)
        assert grid.cell_of((4.0, 6.0)) == CellIndex(3, 5)
        assert grid.cell_of((4.0, 0.0)) == CellIndex(3, 0)

    def test_origin_corner_is_cell_zero_zero(self):
        assert _grid(origin=(-2.0, -2.0)).cell_of((-2.0, -2.0)) == CellIndex(0, 0)

    def test_negative_coordinates(self):
        grid = _grid(cols=4, rows=4, origin=(-2.0, -2.0))
        assert grid.cell_of((-0.0001, -0.0001)) == CellIndex(1, 1)
        assert grid.cell_of((0.0, 0.0)) == CellIndex(2, 2)
        assert grid.cell_of((-1.0, -1.0)) == CellIndex(1, 1)     # lower edge owned by the cell above/right

    @pytest.mark.parametrize("xy", [(-0.001, 5.0), (5.0, -0.001), (10.001, 5.0), (5.0, 10.001),
                                     (float("nan"), 1.0), (1.0, float("nan")), (float("inf"), 1.0),
                                     (1.0, -float("inf")), (1e308, 1.0)])
    def test_points_outside_or_non_finite_have_no_cell(self, xy):
        assert _grid().cell_of(xy) is None

    def test_every_point_of_a_closed_area_is_in_exactly_one_cell(self):
        grid = _grid(cols=3, rows=3)
        for x, y in itertools.product([0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0], repeat=2):
            cell = grid.cell_of((x, y))
            assert cell is not None and grid.contains_cell(cell)


class TestGridSpecValidation:
    @pytest.mark.parametrize("kwargs", [
        dict(cell_size_m=0.0), dict(cell_size_m=-1.0), dict(cell_size_m=float("nan")),
        dict(n_cols=0), dict(n_rows=0), dict(n_cols=1.5), dict(n_cols=True),
        dict(origin_xy_m=(float("nan"), 0.0)), dict(origin_xy_m=(0.0,)), dict(origin_xy_m=[0.0, 0.0]),
    ])
    def test_rejects_bad_arguments(self, kwargs):
        base = dict(origin_xy_m=(0.0, 0.0), cell_size_m=1.0, n_cols=3, n_rows=3)
        base.update(kwargs)
        with pytest.raises(ValueError):
            GridSpec(**base)

    def test_is_frozen(self):
        with pytest.raises(dataclasses.FrozenInstanceError):
            _grid().n_cols = 5


class TestFromArea:
    def test_whole_multiple_area(self):
        grid = GridSpec.from_area(MissionArea(-20.0, -20.0, 20.0, 20.0))
        assert (grid.n_cols, grid.n_rows, grid.origin_xy_m, grid.cell_size_m) == (40, 40, (-20.0, -20.0), 1.0)
        assert grid.n_cells == 1600

    def test_rectangular_area(self):
        grid = GridSpec.from_area(MissionArea(0.0, 0.0, 12.0, 5.0))
        assert (grid.n_cols, grid.n_rows) == (12, 5)

    def test_partial_cells_rejected_unless_allowed(self):
        area = MissionArea(0.0, 0.0, 10.5, 4.0)
        with pytest.raises(ValueError, match="whole multiple"):
            GridSpec.from_area(area)
        grid = GridSpec.from_area(area, allow_partial=True)
        assert (grid.n_cols, grid.n_rows) == (11, 4)
        assert grid.max_xy_m[0] >= 10.5

    def test_area_smaller_than_one_cell_needs_allow_partial(self):
        area = MissionArea(0.0, 0.0, 0.5, 3.0)
        with pytest.raises(ValueError):
            GridSpec.from_area(area)
        assert GridSpec.from_area(area, allow_partial=True).n_cols == 1

    def test_non_unit_cell_size(self):
        grid = GridSpec.from_area(MissionArea(0.0, 0.0, 6.0, 3.0), cell_size_m=0.5)
        assert (grid.n_cols, grid.n_rows) == (12, 6)

    def test_float_noise_in_width_is_tolerated(self):
        area = MissionArea(-1.8288, -1.8288, -1.8288 + 20.0, -1.8288 + 10.0)
        grid = GridSpec.from_area(area)
        assert (grid.n_cols, grid.n_rows) == (20, 10)

    def test_bad_cell_size(self):
        with pytest.raises(ValueError):
            GridSpec.from_area(MissionArea(0.0, 0.0, 4.0, 4.0), cell_size_m=0.0)


class TestCellIds:
    def test_format_matches_the_documented_example(self):
        grid = _grid(cols=20, rows=20)
        assert grid.cell_id(CellIndex(7, 12)) == "X07Y12"
        assert grid.cell_id(CellIndex(0, 0)) == "X00Y00"

    def test_ids_are_unique_and_round_trip(self):
        grid = _grid(cols=13, rows=9)
        ids = [grid.cell_id(c) for c in grid.iter_cells()]
        assert len(set(ids)) == grid.n_cells
        assert all(grid.parse_cell_id(grid.cell_id(c)) == c for c in grid.iter_cells())

    def test_ids_widen_past_two_digits_instead_of_truncating(self):
        grid = _grid(cols=120, rows=3)
        assert grid.cell_id(CellIndex(105, 2)) == "X105Y02"
        assert grid.parse_cell_id("X105Y02") == CellIndex(105, 2)

    @pytest.mark.parametrize("bad", ["x07y12", "X7", "X07Y", "07Y12", "X07-Y12", "", "X07Y12 ", None, 42])
    def test_parse_rejects_malformed(self, bad):
        with pytest.raises(ValueError):
            _grid(cols=20, rows=20).parse_cell_id(bad)

    def test_parse_rejects_out_of_grid(self):
        with pytest.raises(ValueError):
            _grid(cols=5, rows=5).parse_cell_id("X05Y00")

    def test_cell_id_rejects_out_of_grid(self):
        with pytest.raises(ValueError):
            _grid(cols=5, rows=5).cell_id(CellIndex(0, 5))


class TestIterationAndNeighbours:
    def test_iter_cells_is_row_major_and_matches_flat_index(self):
        grid = _grid(cols=4, rows=3)
        cells = list(grid.iter_cells())
        assert len(cells) == 12
        assert cells[:5] == [CellIndex(0, 0), CellIndex(1, 0), CellIndex(2, 0), CellIndex(3, 0), CellIndex(0, 1)]
        assert [grid.flat_index(c) for c in cells] == list(range(12))
        assert [grid.from_flat(k) for k in range(12)] == cells

    @pytest.mark.parametrize("k", [-1, 12, 1.0, True, None])
    def test_from_flat_rejects_bad_index(self, k):
        with pytest.raises(ValueError):
            _grid(cols=4, rows=3).from_flat(k)

    def test_corner_and_interior_neighbour_counts(self):
        grid = _grid(cols=5, rows=5)
        assert len(grid.neighbors(CellIndex(0, 0), 4)) == 2
        assert len(grid.neighbors(CellIndex(0, 0), 8)) == 3
        assert len(grid.neighbors(CellIndex(2, 2), 4)) == 4
        assert len(grid.neighbors(CellIndex(2, 2), 8)) == 8
        assert len(grid.neighbors(CellIndex(0, 2), 8)) == 5

    def test_neighbour_order_is_row_major(self):
        assert _grid(cols=5, rows=5).neighbors(CellIndex(2, 2), 8) == (
            CellIndex(1, 1), CellIndex(2, 1), CellIndex(3, 1), CellIndex(1, 2),
            CellIndex(3, 2), CellIndex(1, 3), CellIndex(2, 3), CellIndex(3, 3))

    def test_neighbours_are_always_in_grid_and_exclude_self(self):
        grid = _grid(cols=4, rows=3)
        for cell in grid.iter_cells():
            for connectivity in (4, 8):
                found = grid.neighbors(cell, connectivity)
                assert cell not in found and all(grid.contains_cell(n) for n in found)

    def test_bad_connectivity_and_cell(self):
        grid = _grid()
        with pytest.raises(ValueError):
            grid.neighbors(CellIndex(1, 1), 6)
        with pytest.raises(ValueError):
            grid.neighbors(CellIndex(10, 1), 4)


class TestCellsOverlappingDisc:
    def test_disc_inside_one_cell(self):
        assert _grid().cells_overlapping_disc((5.5, 5.5), 0.4) == (CellIndex(5, 5),)

    def test_zero_radius_is_the_owning_cell(self):
        assert _grid().cells_overlapping_disc((2.5, 7.5), 0.0) == (CellIndex(2, 7),)

    def test_disc_on_a_grid_corner_touches_four_cells(self):
        assert set(_grid().cells_overlapping_disc((5.0, 5.0), 0.4)) == {
            CellIndex(4, 4), CellIndex(5, 4), CellIndex(4, 5), CellIndex(5, 5)}

    def test_disc_touching_an_edge_counts_but_diagonals_only_if_within_radius(self):
        found = set(_grid().cells_overlapping_disc((5.5, 5.5), 0.5))
        assert found == {CellIndex(5, 5), CellIndex(4, 5), CellIndex(6, 5), CellIndex(5, 4), CellIndex(5, 6)}

    def test_disc_centred_outside_grid_still_overlaps_edge_cells(self):
        assert _grid().cells_overlapping_disc((-0.3, 5.5), 0.5) == (CellIndex(0, 5),)

    def test_disc_fully_outside_overlaps_nothing(self):
        assert _grid().cells_overlapping_disc((-5.0, 5.0), 1.0) == ()

    def test_huge_disc_covers_the_whole_grid_row_major(self):
        grid = _grid(cols=3, rows=3)
        assert grid.cells_overlapping_disc((1.5, 1.5), 100.0) == tuple(grid.iter_cells())

    def test_result_matches_brute_force(self):
        grid = _grid(cols=8, rows=6, origin=(-4.0, -3.0))
        for centre, radius in [((0.3, -0.2), 1.7), ((-3.9, 2.9), 0.9), ((3.99, 2.99), 2.2)]:
            expected = []
            for cell in grid.iter_cells():
                x0, y0, x1, y1 = grid.cell_bounds(cell)
                dx = centre[0] - min(max(centre[0], x0), x1)
                dy = centre[1] - min(max(centre[1], y0), y1)
                if math.hypot(dx, dy) <= radius:
                    expected.append(cell)
            assert grid.cells_overlapping_disc(centre, radius) == tuple(expected)

    @pytest.mark.parametrize("radius", [-0.1, float("nan"), float("inf"), "1"])
    def test_bad_radius(self, radius):
        with pytest.raises(ValueError):
            _grid().cells_overlapping_disc((1.0, 1.0), radius)

    def test_non_finite_centre(self):
        with pytest.raises(ValueError):
            _grid().cells_overlapping_disc((float("nan"), 1.0), 1.0)


# ==========================================================================
# MissionArea
# ==========================================================================

class TestMissionArea:
    def test_validation(self):
        with pytest.raises(ValueError):
            MissionArea(1.0, 0.0, 1.0, 5.0)
        with pytest.raises(ValueError):
            MissionArea(0.0, 5.0, 1.0, 5.0)
        with pytest.raises(ValueError):
            MissionArea(float("nan"), 0.0, 1.0, 1.0)

    def test_contains_and_inset_sign(self):
        area = MissionArea(-10.0, -5.0, 10.0, 5.0)
        assert area.contains((10.0, 5.0)) and area.contains((-10.0, -5.0))
        assert not area.contains((10.01, 0.0))
        assert not area.contains((9.5, 0.0), inset_m=1.0)      # positive inset = margin inside
        assert area.contains((8.9, 0.0), inset_m=1.0)

    def test_inset_shrinks_and_raises_when_it_collapses(self):
        area = MissionArea(0.0, 0.0, 10.0, 4.0)
        assert area.inset(1.0) == MissionArea(1.0, 1.0, 9.0, 3.0)
        with pytest.raises(ValueError):
            area.inset(2.0)

    def test_geofence_parity_with_rectangular_search_area(self):
        for (min_x, min_y, max_x, max_y) in [(-20.0, -20.0, 20.0, 20.0), (0.0, 0.0, 12.0, 8.0),
                                              (-1.8288, -1.8288, 18.1712, 8.1712)]:
            for inset in (0.0, 0.5, 1.25):
                mine = MissionArea(min_x, min_y, max_x, max_y).to_geofence(0.5, 8.0, inset_m=inset)
                theirs = RectangularSearchArea(min_x, min_y, max_x, max_y).to_geofence(0.5, 8.0, margin_m=-inset)
                assert mine.frame is theirs.frame is Frame.LOCAL_ENU
                assert mine.center_m == pytest.approx(theirs.center_m)
                assert mine.half_extents_m == pytest.approx(theirs.half_extents_m)
                assert (mine.floor_alt_m, mine.ceiling_alt_m) == (0.5, 8.0)

    def test_dict_round_trip(self):
        area = MissionArea(-3.5, -2.25, 9.0, 4.125)
        assert MissionArea.from_dict(json.loads(json.dumps(area.to_dict()))) == area


# ==========================================================================
# LaunchZone
# ==========================================================================

class TestLaunchZone:
    def test_default_side_is_twelve_feet(self):
        assert FT_TO_M == 0.3048
        assert LAUNCH_ZONE_SIDE_M == pytest.approx(3.6576)
        assert LaunchZone().side_m == LAUNCH_ZONE_SIDE_M

    def test_contains_is_inclusive_and_inset_shrinks(self):
        zone = LaunchZone()
        h = zone.side_m / 2.0
        assert zone.contains((0.0, 0.0)) and zone.contains((h, h)) and zone.contains((-h, h))
        assert not zone.contains((h + 0.001, 0.0))
        assert not zone.contains((h - 0.1, 0.0), inset_m=0.5)
        assert zone.contains((h - 0.6, 0.0), inset_m=0.5)

    def test_offset_centre(self):
        zone = LaunchZone(center_m=(10.0, -4.0))
        assert zone.contains((10.0, -4.0)) and not zone.contains((0.0, 0.0))

    def test_rotated_zone_is_a_diamond_at_45_degrees(self):
        zone = LaunchZone(yaw_rad=math.pi / 4)
        vertex = zone.side_m / math.sqrt(2.0)          # 2.586 m for the 12 ft zone
        assert zone.contains((vertex - 0.01, 0.0)) and not zone.contains((vertex + 0.01, 0.0))
        assert not zone.contains((zone.side_m / 2.0 + 0.5, zone.side_m / 2.0 + 0.5))

    def test_corners_are_on_the_boundary_at_any_yaw_and_offset(self):
        for yaw in (0.0, 0.3, math.pi / 4, 2.0):
            zone = LaunchZone(center_m=(1.5, -2.0), yaw_rad=yaw)
            corners = zone.corners()
            assert len(corners) == 4
            for corner in corners:
                assert math.hypot(corner[0] - 1.5, corner[1] + 2.0) == pytest.approx(zone.side_m / math.sqrt(2.0))
                assert zone.contains(corner, inset_m=-1e-9)
            for a, b in zip(corners, corners[1:] + corners[:1]):
                assert math.hypot(a[0] - b[0], a[1] - b[1]) == pytest.approx(zone.side_m)

    @pytest.mark.parametrize("kwargs", [dict(side_m=0.0), dict(side_m=-1.0), dict(side_m=float("nan")),
                                         dict(center_m=(0.0,)), dict(center_m=(float("inf"), 0.0)),
                                         dict(yaw_rad=float("nan"))])
    def test_validation(self, kwargs):
        with pytest.raises(ValueError):
            LaunchZone(**kwargs)

    def test_dict_round_trip(self):
        zone = LaunchZone(center_m=(1.0, 2.0), side_m=4.0, yaw_rad=0.25)
        assert LaunchZone.from_dict(json.loads(json.dumps(zone.to_dict()))) == zone


class TestLaunchZoneSlots:
    def test_single_slot_is_the_centre(self):
        slots = LaunchZone(center_m=(3.0, 4.0)).slots(1, 1.0, 0.5)
        assert len(slots) == 1 and slots[0] == pytest.approx((3.0, 4.0))

    @pytest.mark.parametrize("n", [2, 3, 4, 5, 6, 7, 8, 9])
    def test_feasible_fleets_are_inside_the_zone_and_separated(self, n):
        zone = LaunchZone()
        slots = zone.slots(n, min_spacing_m=1.0, edge_margin_m=0.5)
        assert len(slots) == n
        assert all(zone.contains(s, inset_m=0.5 - 1e-9) for s in slots)
        for a, b in itertools.combinations(slots, 2):
            assert math.hypot(a[0] - b[0], a[1] - b[1]) >= 1.0 - 1e-9

    def test_deterministic_and_prefix_stable_within_a_lattice(self):
        zone = LaunchZone()
        assert zone.slots(6, 1.0, 0.5) == zone.slots(6, 1.0, 0.5)
        assert zone.slots(9, 1.0, 0.5)[:8] == zone.slots(8, 1.0, 0.5)   # both use the 3x3 lattice

    def test_slots_are_distinct(self):
        slots = LaunchZone().slots(9, 1.0, 0.5)
        assert len(set(slots)) == 9

    def test_infeasible_fleets_raise_rather_than_overlap(self):
        zone = LaunchZone()
        with pytest.raises(ValueError, match="cannot place"):
            zone.slots(20, 1.0, 0.5)
        with pytest.raises(ValueError, match="cannot place"):
            zone.slots(10, 1.0, 0.5)
        assert len(zone.slots(10, 0.8, 0.5)) == 10

    def test_edge_margin_that_consumes_the_zone_raises(self):
        with pytest.raises(ValueError):
            LaunchZone().slots(4, 0.0, 2.0)
        with pytest.raises(ValueError):
            LaunchZone().slots(2, 0.0, LAUNCH_ZONE_SIDE_M / 2.0)

    @pytest.mark.parametrize("args", [(0, 1.0, 0.5), (-1, 1.0, 0.5), (2.5, 1.0, 0.5), (True, 1.0, 0.5),
                                       (3, -1.0, 0.5), (3, float("nan"), 0.5), (3, 1.0, -0.1)])
    def test_bad_arguments(self, args):
        with pytest.raises(ValueError):
            LaunchZone().slots(*args)

    def test_rotation_and_offset_preserve_spacing_and_containment(self):
        plain = LaunchZone().slots(6, 1.0, 0.5)
        zone = LaunchZone(center_m=(7.0, -3.0), yaw_rad=0.6)
        moved = zone.slots(6, 1.0, 0.5)
        assert all(zone.contains(s, inset_m=0.5 - 1e-9) for s in moved)
        for (a, b), (c, d) in zip(itertools.combinations(plain, 2), itertools.combinations(moved, 2)):
            assert math.hypot(a[0] - b[0], a[1] - b[1]) == pytest.approx(math.hypot(c[0] - d[0], c[1] - d[1]))

    def test_no_edge_margin_puts_slots_on_the_zone_boundary(self):
        zone = LaunchZone()
        slots = zone.slots(4, 1.0, 0.0)
        assert all(zone.contains(s, inset_m=-1e-9) for s in slots)
        assert max(abs(s[0]) for s in slots) == pytest.approx(zone.side_m / 2.0)


# ==========================================================================
# MissionMap
# ==========================================================================

class TestMissionMap:
    def test_centered_square_matches_the_legacy_flood_search_geofence(self):
        # mission.py builds: GeofenceSpec(LOCAL_ENU, center (0,0), half (arena/2, arena/2), floor, ceiling)
        for arena in (40.0, 20.0, 15.0):
            half = arena / 2
            legacy = GeofenceSpec(frame=Frame.LOCAL_ENU, center_m=(0.0, 0.0), half_extents_m=(half, half),
                                  floor_alt_m=0.5, ceiling_alt_m=8.0)
            assert MissionMap.centered_square(arena).to_geofence(0.5, 8.0) == legacy

    def test_centered_square_grid_and_zone(self):
        m = MissionMap.centered_square(40.0)
        assert (m.grid.n_cols, m.grid.n_rows, m.grid.origin_xy_m) == (40, 40, (-20.0, -20.0))
        assert m.launch_zone == LaunchZone()
        assert m.grid.cell_of((0.0, 0.0)) == CellIndex(20, 20)

    def test_centered_square_relaxation_flags_accept_what_the_legacy_mission_accepts(self):
        # FloodSearchMission takes any arena_size; the strict checks stay on by default.
        with pytest.raises(ValueError, match="whole multiple"):
            MissionMap.centered_square(15.5)
        odd = MissionMap.centered_square(15.5, allow_partial_cells=True)
        assert (odd.grid.n_cols, odd.grid.n_rows) == (16, 16)
        assert odd.to_geofence(0.5, 8.0).half_extents_m == (7.75, 7.75)
        with pytest.raises(ValueError, match="outside the search area"):
            MissionMap.centered_square(2.0)
        tiny = MissionMap.centered_square(2.0, allow_zone_outside_area=True)
        assert tiny.allow_zone_outside_area is True and tiny.grid.n_cells == 4

    def test_zone_larger_than_area_is_rejected_unless_allowed(self):
        with pytest.raises(ValueError, match="outside the search area"):
            MissionMap.centered_square(2.0)
        assert MissionMap.centered_square(2.0, launch_zone=LaunchZone(side_m=1.0)).launch_zone.side_m == 1.0
        area = MissionArea(-1.0, -1.0, 1.0, 1.0)
        assert MissionMap(area, LaunchZone(), GridSpec.from_area(area), allow_zone_outside_area=True)

    def test_zone_partly_outside_is_rejected(self):
        area = MissionArea(-10.0, -10.0, 10.0, 10.0)
        with pytest.raises(ValueError, match="outside the search area"):
            MissionMap(area, LaunchZone(center_m=(9.0, 0.0)), GridSpec.from_area(area))

    def test_grid_must_cover_the_area(self):
        area = MissionArea(-10.0, -10.0, 10.0, 10.0)
        with pytest.raises(ValueError, match="cover"):
            MissionMap(area, LaunchZone(), _grid(cols=10, rows=10, origin=(-10.0, -10.0)))
        with pytest.raises(ValueError, match="cover"):
            MissionMap(area, LaunchZone(), _grid(cols=20, rows=20, origin=(-9.0, -10.0)))

    @pytest.mark.parametrize("corner", ["sw", "se", "nw", "ne"])
    def test_zone_at_corner_is_flush_with_that_corner(self, corner):
        m = MissionMap.zone_at_corner(20.0, 10.0, corner)
        assert (m.area.width_m, m.area.height_m) == pytest.approx((20.0, 10.0))
        assert (m.grid.n_cols, m.grid.n_rows) == (20, 10)
        h = LAUNCH_ZONE_SIDE_M / 2.0
        xs = sorted(c[0] for c in m.launch_zone.corners())
        ys = sorted(c[1] for c in m.launch_zone.corners())
        if corner[1] == "w":
            assert m.area.min_x_m == pytest.approx(xs[0]) and m.area.max_x_m == pytest.approx(-h + 20.0)
        else:
            assert m.area.max_x_m == pytest.approx(xs[-1]) and m.area.min_x_m == pytest.approx(h - 20.0)
        if corner[0] == "s":
            assert m.area.min_y_m == pytest.approx(ys[0]) and m.area.max_y_m == pytest.approx(-h + 10.0)
        else:
            assert m.area.max_y_m == pytest.approx(ys[-1]) and m.area.min_y_m == pytest.approx(h - 10.0)
        assert m.area.contains((0.0, 0.0))                     # the frame origin (zone centre) is inside the area

    def test_zone_at_corner_rejects_bad_corner_and_too_small_area(self):
        with pytest.raises(ValueError):
            MissionMap.zone_at_corner(20.0, 10.0, "center")
        with pytest.raises(ValueError):
            MissionMap.zone_at_corner(2.0, 10.0, "sw")

    def test_json_round_trip_preserves_equality_and_id(self):
        for m in (MissionMap.centered_square(40.0), MissionMap.zone_at_corner(20.0, 10.0, "ne"),
                  MissionMap(MissionArea(-1.0, -1.0, 1.0, 1.0), LaunchZone(), GridSpec.from_area(MissionArea(-1.0, -1.0, 1.0, 1.0)),
                             allow_zone_outside_area=True)):
            restored = MissionMap.from_dict(json.loads(json.dumps(m.to_dict())))
            assert restored == m and restored.map_id == m.map_id

    def test_map_id_is_a_stable_12_hex_content_hash(self):
        m = MissionMap.centered_square(40.0)
        canonical = json.dumps(m.to_dict(), sort_keys=True, separators=(",", ":"))
        assert m.map_id == hashlib.sha256(canonical.encode("utf-8")).hexdigest()[:12]
        assert len(m.map_id) == 12 and all(ch in "0123456789abcdef" for ch in m.map_id)
        assert MissionMap.centered_square(40.0).map_id == m.map_id

    def test_map_id_changes_with_any_geometry_change(self):
        ids = {
            MissionMap.centered_square(40.0).map_id,
            MissionMap.centered_square(30.0).map_id,
            MissionMap.centered_square(40.0, cell_m=2.0).map_id,
            MissionMap.centered_square(40.0, launch_zone=LaunchZone(yaw_rad=0.1)).map_id,
            MissionMap.centered_square(40.0, launch_zone=LaunchZone(center_m=(1.0, 0.0))).map_id,
            MissionMap.zone_at_corner(40.0, 40.0, "sw").map_id,
        }
        assert len(ids) == 6

    def test_is_frozen(self):
        with pytest.raises(dataclasses.FrozenInstanceError):
            MissionMap.centered_square(40.0).area = MissionArea(0.0, 0.0, 1.0, 1.0)


# ==========================================================================
# mission_rules
# ==========================================================================

class TestMissionRules:
    def test_constants_match_the_competition_rules(self):
        assert mission_rules.MIN_DRONES == 2
        assert mission_rules.MAX_SURVIVORS == 10
        assert mission_rules.MAX_TOTAL_MASS_KG == 25.0
        assert mission_rules.KIT_MASS_KG == pytest.approx(0.2)
        assert mission_rules.KIT_DIMS_M == pytest.approx((0.20, 0.10, 0.05))

    def test_fleet_within_limit(self):
        result = mission_rules.check_fleet_mass([4.0, 4.0, 4.0], payload_kg=0.6)
        assert result.ok and result.total_kg == pytest.approx(12.6)
        assert result.margin_kg == pytest.approx(12.4) and result.limit_kg == 25.0

    def test_limit_is_inclusive_and_over_limit_is_a_result_not_an_exception(self):
        assert mission_rules.check_fleet_mass([12.5, 12.0], payload_kg=0.5).ok
        over = mission_rules.check_fleet_mass([12.5, 12.0], payload_kg=0.6)
        assert not over.ok and over.margin_kg == pytest.approx(-0.1)

    def test_kits_count_toward_the_budget(self):
        drones = [12.4, 12.4]
        assert mission_rules.check_fleet_mass(drones, payload_kg=0.0).ok
        assert not mission_rules.check_fleet_mass(drones, payload_kg=5 * mission_rules.KIT_MASS_KG).ok

    @pytest.mark.parametrize("args", [([], 0.0), ([-1.0], 0.0), ([1.0], -0.1), ([float("nan")], 0.0),
                                       ([1.0], float("inf")), (["1"], 0.0), ([True], 0.0)])
    def test_malformed_inputs_raise(self, args):
        with pytest.raises(ValueError):
            mission_rules.check_fleet_mass(*args)

    def test_bad_limit(self):
        with pytest.raises(ValueError):
            mission_rules.check_fleet_mass([1.0], 0.0, limit_kg=0.0)
