"""Shared contract helpers for every behavior model in this package
(Reynolds Boids, Vicsek, Couzin, Olfati-Saber-inspired) - see
docs/PHASE3_BEHAVIORS.md's "common controller contract" section.

Every public entry point in boids.py/vicsek.py/couzin.py/olfati_saber.py
returns a CANDIDATE command only: a bounded, always-finite 3-vector in the
same LOCAL_ENU frame as swarm_sim.contracts.Frame.LOCAL_ENU (z held at 0 -
this simulator turns only in the horizontal plane, altitude is a separate
position-hold loop), in units of meters/second. None of these functions
apply anything to a vehicle, construct a Command, or are called from
anywhere but SwarmController.step() - they generate candidates that
SwarmController (and, from Phase 4 on, the safety supervisor sitting in
front of it) may still override. See docs/PHASE3_BEHAVIORS.md.

Native output categories, i.e. which raw quantity each model computes
BEFORE this module's bounding is applied (this determines which helper
below a given model's *_candidate_command wrapper uses):

- ACCELERATION-shaped: Boids (boids.py) and the Olfati-Saber-inspired
  controller (olfati_saber.py). Both compute a steering FORCE - directly
  interpretable as the `u_i` input to a double-integrator (q_dot=p,
  p_dot=u), matching Reynolds' and Olfati-Saber's own formulations. Their
  *_candidate_command wrappers use clamp_acceleration on the raw force,
  integrate one Euler step, then clamp_speed the result.
- HEADING-shaped: Vicsek (vicsek.py) and Couzin (couzin.py). Both compute
  a desired DIRECTION, not a force. Their *_candidate_command wrappers use
  clamp_turn_rate (bounding how fast that heading may change from the
  previous step, not the resulting acceleration) then scale to a speed
  and clamp_speed it.

This is why clamp_acceleration and clamp_turn_rate are two separate
helpers rather than one: they bound two different native quantities, and
a turn-rate bound does not imply a bounded acceleration (for circular
motion, acceleration = speed * turn_rate) or vice versa - see the
"remaining risks" note in docs/PHASE3_BEHAVIORS.md's benchmark results.
"""
import numpy as np


def sanitize_vector(vec, fallback):
    """Replace a NaN/Inf-containing vector with `fallback`. Every behavior
    function's return path goes through this (directly or via
    bounded_heading/clamp_speed below) - this is what "no NaN output for
    coincident or missing neighbors" means concretely."""
    vec = np.asarray(vec, dtype=float)
    if vec.shape != (3,) or not np.all(np.isfinite(vec)):
        return np.array(fallback, dtype=float)
    return vec


def bounded_heading(vec, fallback):
    """Normalize `vec` to a finite unit heading. Falls back to `fallback`
    (itself sanitized) whenever `vec` is degenerate: zero-length, NaN, or
    Inf - covers "safe behavior for zero neighbors" (callers pass a
    zero vector when no neighbor contributed anything) and "no NaN output
    for coincident neighbors" (a repulsion term from an exactly-coincident
    neighbor is never computed in the first place - see couzin.py - but if
    it ever were, this is the backstop)."""
    fallback = sanitize_vector(fallback, np.array([1.0, 0.0, 0.0]))
    vec = sanitize_vector(vec, fallback)
    n = np.linalg.norm(vec)
    if not np.isfinite(n) or n < 1e-9:
        return fallback
    return vec / n


def clamp_speed(vec, max_speed_mps):
    """Bounds a velocity-shaped vector to at most `max_speed_mps`, direction
    preserved. Finite by construction given a finite input (NaN/Inf inputs
    should already have been caught by sanitize_vector/bounded_heading
    upstream of this)."""
    vec = np.asarray(vec, dtype=float)
    speed = np.linalg.norm(vec)
    if speed <= max_speed_mps or speed < 1e-9:
        return vec
    return vec * (max_speed_mps / speed)


def clamp_acceleration(steer_vec, max_accel_mps2):
    """Bounds a steering-force-shaped vector (Reynolds' model works in
    acceleration, not heading) to at most `max_accel_mps2` in magnitude."""
    return clamp_speed(steer_vec, max_accel_mps2)


def clamp_turn_rate(new_heading, prev_heading, max_turn_rate_radps, dt_s):
    """Limits the angular change (about the vertical/yaw axis only - this
    simulator holds altitude via a separate loop, see the module
    docstring) from `prev_heading` to `new_heading` to at most
    `max_turn_rate_radps * dt_s`. Both headings are unit (or near-unit) XY
    vectors; z is ignored on input and always 0 on output. This is the
    "bounded turn rate" half of the common contract, used by the
    heading-based models (Vicsek, Couzin, Olfati-Saber) - Boids instead
    bounds acceleration directly (clamp_acceleration above), matching each
    model's own native quantity."""
    prev = bounded_heading(prev_heading, np.array([1.0, 0.0, 0.0]))
    new = bounded_heading(new_heading, prev)
    prev_angle = float(np.arctan2(prev[1], prev[0]))
    new_angle = float(np.arctan2(new[1], new[0]))
    delta = float(np.arctan2(np.sin(new_angle - prev_angle), np.cos(new_angle - prev_angle)))
    max_delta = abs(max_turn_rate_radps) * dt_s
    delta = float(np.clip(delta, -max_delta, max_delta))
    angle = prev_angle + delta
    return np.array([np.cos(angle), np.sin(angle), 0.0])


def all_finite(*vecs) -> bool:
    return all(np.all(np.isfinite(np.asarray(v, dtype=float))) for v in vecs)
