"""Viswanathan, G. M. et al. (1999) - Optimizing the success of random searches.

Step lengths drawn from a heavy-tailed Levy-stable distribution (via
Mantegna's algorithm) rather than a Gaussian: mostly short local moves
punctuated by occasional long straight runs. Proven near-optimal for
searching sparse, randomly-located targets with no prior information -
exactly the "search a flooded area for scattered survivors" case.

A "flight leg" is a single straight run: pick a direction and a step length,
hold that heading until the leg's distance is covered, then resample. This
persistence is essential - resampling direction every control tick collapses
the heavy-tailed long-run behavior into unflyable noise.
"""
from math import gamma, sin, pi
import numpy as np


def levy_step_length(alpha, rng):
    sigma_u = (
        gamma(1 + alpha) * sin(pi * alpha / 2)
        / (gamma((1 + alpha) / 2) * alpha * 2 ** ((alpha - 1) / 2))
    ) ** (1 / alpha)
    u = rng.normal(0.0, sigma_u)
    v = rng.normal(0.0, 1.0)
    return u / (abs(v) ** (1 / alpha))


def _sample_direction(rng):
    d = rng.normal(size=3)
    d[2] = 0.0  # aerial search: hold altitude, turn only in the horizontal plane
    n = np.linalg.norm(d)
    return d / n if n > 1e-9 else np.array([1.0, 0.0, 0.0])


def levy_flight_leg(rng, alpha, prev_direction, remaining_m, dt, speed_mps, min_leg_m=2.0, max_leg_m=15.0):
    """Advance one flight leg. Returns (direction, remaining_distance_m).

    Call every control step with the previous call's outputs; when
    remaining_m drops to zero a new direction and leg length are drawn.
    """
    if prev_direction is None or remaining_m <= 0.0:
        direction = _sample_direction(rng)
        leg_length = float(np.clip(abs(levy_step_length(alpha, rng)), min_leg_m, max_leg_m))
    else:
        direction = prev_direction
        leg_length = remaining_m
    return direction, leg_length - speed_mps * dt
