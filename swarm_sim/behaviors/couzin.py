"""Couzin, I. D. et al. (2002) - Collective memory and spatial sorting in
animal groups.

Zonal model with strict priority: a neighbor inside the zone of repulsion
overrides everything else (collision/crowding avoidance always wins); absent
that, headings blend the zone of orientation (align) and zone of attraction
(cohere). A field-of-view angle models the blind zone behind each agent.

Phase 3 corrections (see docs/PHASE3_BEHAVIORS.md):
- `couzin_candidate_command` adds the "bounded speed and turn rate" half of
  the common contract that was previously enforced only far downstream, in
  SwarmController - turn rate is bounded here via
  swarm_sim/behaviors/common.py's clamp_turn_rate, speed via clamp_speed,
  so this module's output is independently contract-compliant.
- Explicit, deterministic no-neighbor and coincident-neighbor behavior
  (see couzin_direction's docstring) - unchanged in substance from before,
  now documented and covered by tests/test_behaviors_couzin.py rather than
  being an incidental consequence of the dist<1e-9 guard.
"""
import numpy as np

from .common import bounded_heading, clamp_speed, clamp_turn_rate


def _in_fov(rel_vec, heading, fov_deg):
    if fov_deg >= 360:
        return True
    n1 = np.linalg.norm(rel_vec)
    if n1 < 1e-9:
        return True
    cos_angle = np.dot(rel_vec, heading) / (n1 * np.linalg.norm(heading) + 1e-9)
    cos_angle = np.clip(cos_angle, -1.0, 1.0)
    return np.degrees(np.arccos(cos_angle)) <= fov_deg / 2


def couzin_direction(positions, headings, i, neighbor_ids, r_repulsion, r_orientation, r_attraction, fov_deg=270.0):
    """Raw desired heading (unit vector, NOT yet turn-rate-limited - see
    couzin_candidate_command for the contract-compliant entry point).

    Zone priority, strict: any neighbor within r_repulsion makes repulsion
    ALONE the output, regardless of how many other neighbors are in the
    orientation/attraction zones - Couzin's model gives crowding/collision
    avoidance absolute priority, never blended away by cohesion.

    No-neighbor / coincident-neighbor behavior, explicit and deterministic:
    a neighbor at (near-)exactly this agent's own position (dist < 1e-9)
    has no well-defined bearing and is excluded from every zone - same
    treatment as "not visible" (out of r_attraction or outside the FOV),
    not a special case. When no neighbor ends up in any zone at all, the
    output is this agent's own current heading unchanged - never zero,
    never NaN, never a coin-flip.
    """
    pos_i, head_i = positions[i], headings[i]
    repulsion = np.zeros(3)
    orientation = np.zeros(3)
    attraction = np.zeros(3)
    n_rep = n_ori = n_att = 0

    for j in neighbor_ids:
        rel = positions[j] - pos_i
        dist = np.linalg.norm(rel)
        if dist < 1e-9 or dist > r_attraction:
            continue
        if dist >= r_repulsion and not _in_fov(rel, head_i, fov_deg):
            continue
        if dist < r_repulsion:
            repulsion -= rel / dist
            n_rep += 1
        elif dist < r_orientation:
            orientation += headings[j]
            n_ori += 1
        else:
            attraction += rel / dist
            n_att += 1

    if n_rep > 0:
        desired = repulsion / n_rep
    else:
        desired = np.zeros(3)
        if n_ori > 0:
            desired += orientation / n_ori
        if n_att > 0:
            desired += attraction / n_att
        if n_ori == 0 and n_att == 0:
            desired = head_i.copy()

    return bounded_heading(desired, head_i)


def couzin_candidate_command(positions, headings, i, neighbor_ids, r_repulsion, r_orientation, r_attraction,
                              prev_heading, speed_mps, max_speed_mps, max_turn_rate_radps, dt_s, fov_deg=270.0):
    """Contract-compliant candidate command: the raw Couzin heading, turn-
    rate-limited from `prev_heading` and scaled to a speed that never
    exceeds max_speed_mps. Returns a finite 3-vector velocity, LOCAL_ENU,
    m/s (z=0) - see swarm_sim/behaviors/common.py."""
    raw_heading = couzin_direction(positions, headings, i, neighbor_ids,
                                    r_repulsion, r_orientation, r_attraction, fov_deg)
    limited_heading = clamp_turn_rate(raw_heading, prev_heading, max_turn_rate_radps, dt_s)
    return clamp_speed(limited_heading * min(speed_mps, max_speed_mps), max_speed_mps)
