"""Vicsek, T. et al. (1995) - Novel type of phase transition in a system of
self-driven particles.

This IS the canonical Vicsek baseline, not an approximation of it - see
docs/PHASE3_BEHAVIORS.md. Phase 3 corrected two bugs in the previous
version: it averaged 3D velocity vectors and added 3D Gaussian noise
directly to the (already near-unit) vector, rather than the paper's
construction of an atan2 CIRCULAR mean heading perturbed by angular noise.
A linear mean of vectors happens to be a reasonable approximation of a
circular mean for headings that are already close together, which is why
it didn't look obviously wrong - but it silently breaks down whenever
headings are broadly spread (e.g. exactly opposed neighbors) and doesn't
match the paper's noise model at all.

Two neighbor-selection modes:
- `vicsek_heading` (SAR mode, non-periodic): used by SwarmController.
  Neighbors are whoever the caller already selected via this simulator's
  bounded, non-periodic search arena - no wraparound.
- `vicsek_periodic_step` (periodic regression mode): reproduces the
  original paper's toroidal boundary conditions over the whole population
  at once. Not used by SwarmController - this simulator's arena is
  bounded, not periodic - kept only so
  tests/test_behaviors_vicsek.py can check this package's Vicsek math
  reproduces the textbook order-parameter-vs-noise phase transition.

Output contract (frame/units/bounds) for vicsek_heading: see
swarm_sim/behaviors/common.py's module docstring.
"""
import math

import numpy as np

from .common import bounded_heading, clamp_speed, clamp_turn_rate


def mean_heading_angle(headings_xy, ids):
    """atan2-based circular mean of the 2D headings at `ids` (index array,
    may include self): sums the unit vectors first, then takes one atan2 -
    the correct way to average angles (a plain mean of atan2(y, x) values
    is wrong across the +-pi wraparound). Returns None when the summed
    vector is (near) zero - the headings exactly cancel and there is no
    well-defined mean direction (e.g. two neighbors facing exactly
    opposite ways)."""
    ids = np.asarray(ids, dtype=int)
    if len(ids) == 0:
        return None
    total = headings_xy[ids].sum(axis=0)
    if np.linalg.norm(total) < 1e-9:
        return None
    return math.atan2(total[1], total[0])


def vicsek_heading(headings, i, neighbor_ids, noise_strength, rng):
    """SAR-mode (non-periodic) canonical Vicsek step for one agent: the
    atan2 circular mean heading of self + neighbors (2D/horizontal only -
    z is ignored on input and always 0 on output), perturbed by uniform
    angular noise in [-noise_strength, +noise_strength] radians
    (`noise_strength` is the paper's noise amplitude, in radians here -
    configurable per docs/PHASE3_BEHAVIORS.md). Falls back to this agent's
    own current heading when neighbors' directions exactly cancel or none
    are visible - always a finite unit heading, never NaN."""
    headings = np.asarray(headings, dtype=float)
    own_xy = headings[i, :2]
    ids = np.append(np.asarray(neighbor_ids, dtype=int), i)
    angle = mean_heading_angle(headings[:, :2], ids)
    if angle is None:
        angle = math.atan2(own_xy[1], own_xy[0]) if np.linalg.norm(own_xy) > 1e-9 else 0.0
    angle += rng.uniform(-noise_strength, noise_strength)
    result = np.array([math.cos(angle), math.sin(angle), 0.0])
    return bounded_heading(result, headings[i])


def vicsek_candidate_command(headings, i, neighbor_ids, noise_strength, rng,
                              prev_heading, speed_mps, max_speed_mps, max_turn_rate_radps, dt_s):
    """Contract-compliant candidate command: the raw (SAR-mode) Vicsek
    heading, turn-rate-limited from `prev_heading` and scaled to a speed
    that never exceeds max_speed_mps. Returns a finite 3-vector velocity,
    LOCAL_ENU, m/s (z=0) - see swarm_sim/behaviors/common.py. Added in
    Phase 3 alongside boids_candidate_command/couzin_candidate_command/
    olfati_saber_candidate_command so all four models expose the same
    bounded-output entry point, not just vicsek_heading's raw direction."""
    raw_heading = vicsek_heading(headings, i, neighbor_ids, noise_strength, rng)
    limited_heading = clamp_turn_rate(raw_heading, prev_heading, max_turn_rate_radps, dt_s)
    return clamp_speed(limited_heading * min(speed_mps, max_speed_mps), max_speed_mps)


# --- Periodic-boundary regression mode (Vicsek et al. 1995's original
# toroidal-arena model) - reference/test-only, see module docstring. ------

def _wrapped_delta(a, b, box_size):
    d = a - b
    return d - box_size * np.round(d / box_size)


def vicsek_periodic_step(positions, angles, box_size, radius, noise_strength, rng):
    """Regression-only: one synchronous update of the WHOLE population
    under periodic (toroidal) boundary conditions on a box_size x box_size
    square, exactly matching the original paper's model. positions: (N, 2).
    angles: (N,) radians. Returns (N,) new angles."""
    n = positions.shape[0]
    headings = np.stack([np.cos(angles), np.sin(angles)], axis=1)
    new_angles = np.empty(n)
    for i in range(n):
        delta = _wrapped_delta(positions, positions[i], box_size)
        dist = np.linalg.norm(delta, axis=1)
        neighbor_mask = dist < radius
        total = headings[neighbor_mask].sum(axis=0)
        base_angle = angles[i] if np.linalg.norm(total) < 1e-9 else math.atan2(total[1], total[0])
        new_angles[i] = base_angle + rng.uniform(-noise_strength, noise_strength)
    return new_angles


def order_parameter(angles):
    """Vicsek's v_a: the mean normalized velocity magnitude
    |mean(unit_heading_i)|, in [0, 1] - the standard order parameter used
    to plot the model's phase transition (v_a -> 1 as noise -> 0 at
    sufficient density; v_a -> 0 as noise grows)."""
    headings = np.stack([np.cos(angles), np.sin(angles)], axis=1)
    return float(np.linalg.norm(headings.mean(axis=0)))
