"""Reynolds, C. W. (1987) - Flocks, herds and schools: a distributed behavioral model.

Three local steering rules: separation (avoid crowding), alignment (match
neighbors' velocity), cohesion (move toward the local center of mass).
Corrected in Phase 3 (see docs/PHASE3_BEHAVIORS.md) to add:

- an explicit PRIORITY rule, not just a weighted blend: any neighbor inside
  `priority_radius` (default half of `radius`) makes separation alone the
  steering output for this step - cohesion/alignment never get a chance to
  cancel out an active too-close encounter. Outside that, the three rules
  blend by explicit weights (w_sep > w_align == w_coh by default).
- bounded acceleration: `boids_candidate_command` clips the raw steering
  force to `max_accel_mps2` (Reynolds' model is naturally acceleration-
  shaped, unlike Vicsek/Couzin's heading-shaped output) and the resulting
  velocity to `max_speed_mps` - see swarm_sim/behaviors/common.py.
- deterministic, NaN-free handling of a coincident neighbor: any neighbor
  within `coincidence_eps` of this agent's own position contributes
  nothing to separation (rather than 1/0 or a clipped-but-enormous spurious
  force) - a coincident position carries no meaningful separation
  direction, so it is excluded exactly like "no neighbor there" rather
  than approximated.
- safe zero-neighbor behavior: every one of separation/alignment/cohesion
  is exactly the zero vector with no neighbors, and
  `boids_candidate_command` falls back to continuing along the caller's
  supplied `prev_velocity` heading (bounded_heading's fallback) rather than
  producing a zero or NaN command.

Output contract (frame/units/bounds): see swarm_sim/behaviors/common.py's
module docstring.
"""
import numpy as np

from .common import bounded_heading, clamp_acceleration, clamp_speed

COINCIDENCE_EPS_M = 1e-6


def _visible(positions, i, neighbor_ids, radius):
    """Neighbor ids within `radius` of agent i, excluding exactly-coincident
    positions (see module docstring) - shared by separation/alignment/
    cohesion so all three agree on who counts as "close enough to matter"."""
    if len(neighbor_ids) == 0:
        return np.array([], dtype=int), np.zeros((0, 3)), np.zeros(0)
    ids = np.asarray(neighbor_ids, dtype=int)
    delta = positions[i] - positions[ids]
    dist = np.linalg.norm(delta, axis=1)
    keep = (dist > COINCIDENCE_EPS_M) & (dist < radius)
    return ids[keep], delta[keep], dist[keep]


def separation(positions, i, neighbor_ids, radius):
    ids, delta, dist = _visible(positions, i, neighbor_ids, radius)
    if len(ids) == 0:
        return np.zeros(3)
    weighted = delta / (dist[:, None] ** 2)
    return weighted.sum(axis=0)


def alignment(positions, velocities, i, neighbor_ids, radius):
    ids, _, _ = _visible(positions, i, neighbor_ids, radius)
    if len(ids) == 0:
        return np.zeros(3)
    return velocities[ids].mean(axis=0) - velocities[i]


def cohesion(positions, i, neighbor_ids, radius):
    ids, _, _ = _visible(positions, i, neighbor_ids, radius)
    if len(ids) == 0:
        return np.zeros(3)
    center = positions[ids].mean(axis=0)
    return center - positions[i]


def boids_steer(positions, velocities, i, neighbor_ids, radius,
                 w_sep=1.5, w_align=1.0, w_coh=1.0, priority_radius=None):
    """Raw steering force (NOT yet bounded - see boids_candidate_command for
    the contract-compliant entry point). Priority rule: if any visible
    neighbor is within `priority_radius` (default radius/2), returns
    separation alone; otherwise a weighted blend of all three rules."""
    if priority_radius is None:
        priority_radius = radius / 2.0
    _, _, prio_dist = _visible(positions, i, neighbor_ids, priority_radius)
    sep = separation(positions, i, neighbor_ids, radius)
    if len(prio_dist) > 0:
        return sep
    ali = alignment(positions, velocities, i, neighbor_ids, radius)
    coh = cohesion(positions, i, neighbor_ids, radius)
    return w_sep * sep + w_align * ali + w_coh * coh


def boids_candidate_command(positions, velocities, i, neighbor_ids, radius, prev_velocity,
                             max_accel_mps2, max_speed_mps, dt_s,
                             w_sep=1.5, w_align=1.0, w_coh=1.0, priority_radius=None):
    """Contract-compliant candidate command: integrates the (acceleration-
    bounded) steering force for one control step and clips the result to
    max_speed_mps. Returns a finite 3-vector velocity, LOCAL_ENU, m/s (z=0)
    - see swarm_sim/behaviors/common.py."""
    steer = boids_steer(positions, velocities, i, neighbor_ids, radius,
                         w_sep, w_align, w_coh, priority_radius)
    accel = clamp_acceleration(steer, max_accel_mps2)
    if not np.all(np.isfinite(accel)):
        accel = np.zeros(3)
    prev_velocity = np.asarray(prev_velocity, dtype=float)
    if not np.all(np.isfinite(prev_velocity)):
        prev_velocity = np.zeros(3)
    candidate = prev_velocity + accel * dt_s
    candidate[2] = 0.0
    if not np.all(np.isfinite(candidate)) or np.linalg.norm(candidate) < 1e-9:
        # Degenerate: no usable steering and no usable previous velocity to
        # coast on - hold position rather than emit a zero-length/NaN
        # command (still a valid, finite, bounded candidate).
        candidate = bounded_heading(prev_velocity, np.array([1.0, 0.0, 0.0])) * min(
            max_speed_mps, float(np.linalg.norm(prev_velocity)),
        )
    return clamp_speed(candidate, max_speed_mps)
