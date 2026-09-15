"""Phase 3 Couzin tests - see docs/PHASE3_BEHAVIORS.md. Required items
covered: Couzin zone transitions, Couzin field-of-view filtering, Couzin
coincident-neighbor handling (plus bounded speed/turn-rate)."""
import math

import numpy as np

from swarm_sim.behaviors.couzin import couzin_candidate_command, couzin_direction


R_REP, R_ORI, R_ATT = 2.0, 4.0, 6.0


def test_couzin_no_neighbors_keeps_current_heading():
    positions = np.array([[0.0, 0.0, 3.0]])
    headings = np.array([[0.0, 1.0, 0.0]])
    out = couzin_direction(positions, headings, 0, [], R_REP, R_ORI, R_ATT)
    assert np.allclose(out, [0.0, 1.0, 0.0])


def test_couzin_zone_transition_repulsion_dominates():
    # Neighbor inside repulsion zone.
    positions = np.array([[0.0, 0.0, 3.0], [1.0, 0.0, 3.0]])
    headings = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    out = couzin_direction(positions, headings, 0, [1], R_REP, R_ORI, R_ATT)
    # Repulsion pushes agent 0 away from +x neighbor -> -x direction.
    assert out[0] < 0


def test_couzin_zone_transition_orientation_only():
    # Neighbor inside orientation zone (between r_repulsion and r_orientation).
    positions = np.array([[0.0, 0.0, 3.0], [3.0, 0.0, 3.0]])
    headings = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    out = couzin_direction(positions, headings, 0, [1], R_REP, R_ORI, R_ATT)
    # Should align toward neighbor's heading (+y), not attract toward its position (+x).
    assert np.allclose(out, [0.0, 1.0, 0.0], atol=1e-9)


def test_couzin_zone_transition_attraction_only():
    # Neighbor inside attraction zone (between r_orientation and r_attraction).
    positions = np.array([[0.0, 0.0, 3.0], [5.0, 0.0, 3.0]])
    headings = np.array([[0.0, 1.0, 0.0], [0.0, -1.0, 0.0]])
    out = couzin_direction(positions, headings, 0, [1], R_REP, R_ORI, R_ATT)
    # Attraction pulls agent 0 toward the +x neighbor's position.
    assert out[0] > 0.9


def test_couzin_zone_transition_beyond_attraction_ignored():
    positions = np.array([[0.0, 0.0, 3.0], [100.0, 0.0, 3.0]])
    headings = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])
    out = couzin_direction(positions, headings, 0, [1], R_REP, R_ORI, R_ATT)
    assert np.allclose(out, [0.0, 1.0, 0.0])  # too far -> own heading unchanged


def test_couzin_fov_filtering_excludes_behind():
    # Neighbor directly behind agent 0 (facing +x, neighbor at -x), inside
    # the orientation zone, narrow FOV -> must be excluded, so agent 0
    # keeps its own heading rather than aligning with a neighbor it can't see.
    positions = np.array([[0.0, 0.0, 3.0], [-3.0, 0.0, 3.0]])
    headings = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    out = couzin_direction(positions, headings, 0, [1], R_REP, R_ORI, R_ATT, fov_deg=270.0)
    assert np.allclose(out, [1.0, 0.0, 0.0])


def test_couzin_fov_360_sees_everything():
    positions = np.array([[0.0, 0.0, 3.0], [-3.0, 0.0, 3.0]])
    headings = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    out = couzin_direction(positions, headings, 0, [1], R_REP, R_ORI, R_ATT, fov_deg=360.0)
    assert np.allclose(out, [0.0, 1.0, 0.0], atol=1e-9)


def test_couzin_repulsion_ignores_fov():
    # Couzin's repulsion zone is not gated by FOV in this implementation -
    # crowding/collision risk applies even from directly behind.
    positions = np.array([[0.0, 0.0, 3.0], [-1.0, 0.0, 3.0]])
    headings = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    out = couzin_direction(positions, headings, 0, [1], R_REP, R_ORI, R_ATT, fov_deg=270.0)
    assert out[0] > 0  # repulsion from behind (-x neighbor) pushes agent 0 toward +x


def test_couzin_coincident_neighbor_excluded_deterministically():
    positions = np.array([[0.0, 0.0, 3.0], [0.0, 0.0, 3.0]])
    headings = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    out1 = couzin_direction(positions, headings, 0, [1], R_REP, R_ORI, R_ATT)
    out2 = couzin_direction(positions, headings, 0, [1], R_REP, R_ORI, R_ATT)
    assert np.all(np.isfinite(out1))
    assert np.allclose(out1, out2)              # deterministic
    assert np.allclose(out1, [1.0, 0.0, 0.0])   # treated as "not visible" -> own heading kept


def test_couzin_mixed_coincident_and_valid_neighbor():
    # A coincident neighbor must not corrupt the result when a valid
    # repulsion-zone neighbor is also present.
    positions = np.array([[0.0, 0.0, 3.0], [0.0, 0.0, 3.0], [1.0, 0.0, 3.0]])
    headings = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, -1.0, 0.0]])
    out = couzin_direction(positions, headings, 0, [1, 2], R_REP, R_ORI, R_ATT)
    assert np.all(np.isfinite(out))
    assert out[0] < 0  # repulsion from the valid neighbor at +x


def test_couzin_candidate_command_bounds_turn_rate():
    positions = np.array([[0.0, 0.0, 3.0], [5.0, 0.0, 3.0]])
    headings = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    prev_heading = np.array([1.0, 0.0, 0.0])
    out = couzin_candidate_command(
        positions, headings, 0, [1], R_REP, R_ORI, R_ATT,
        prev_heading=prev_heading, speed_mps=2.0, max_speed_mps=5.0,
        max_turn_rate_radps=0.5, dt_s=1 / 24,
    )
    assert np.all(np.isfinite(out))
    heading_out = out / np.linalg.norm(out)
    prev_angle = math.atan2(prev_heading[1], prev_heading[0])
    new_angle = math.atan2(heading_out[1], heading_out[0])
    angle_change = abs(math.atan2(math.sin(new_angle - prev_angle), math.cos(new_angle - prev_angle)))
    assert angle_change <= 0.5 * (1 / 24) + 1e-6
    assert np.linalg.norm(out) <= 5.0 + 1e-9


def test_couzin_candidate_command_bounds_speed():
    positions = np.array([[0.0, 0.0, 3.0]])
    headings = np.array([[1.0, 0.0, 0.0]])
    out = couzin_candidate_command(
        positions, headings, 0, [], R_REP, R_ORI, R_ATT,
        prev_heading=np.array([1.0, 0.0, 0.0]), speed_mps=100.0, max_speed_mps=3.0,
        max_turn_rate_radps=5.0, dt_s=1 / 24,
    )
    assert np.linalg.norm(out) <= 3.0 + 1e-9
