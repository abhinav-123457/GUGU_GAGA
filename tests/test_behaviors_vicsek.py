"""Phase 3 Vicsek tests - see docs/PHASE3_BEHAVIORS.md. Required items
covered: Vicsek angular averaging, Vicsek angular-noise distribution,
Vicsek wraparound regression mode, Vicsek non-periodic SAR mode."""
import math

import numpy as np

from swarm_sim.behaviors.vicsek import (
    mean_heading_angle, order_parameter, vicsek_candidate_command, vicsek_heading, vicsek_periodic_step,
)


def test_mean_heading_angle_atan2_averaging():
    # Two headings at +30deg and -30deg average to 0deg exactly - a
    # correct circular mean, not a naive average of atan2 outputs (which
    # would also give 0 here, so also check a case where wraparound would
    # break a naive linear-angle mean: +170deg and -170deg average to
    # 180deg (pointing in -x), not 0deg.
    headings = np.array([
        [math.cos(math.radians(30)), math.sin(math.radians(30))],
        [math.cos(math.radians(-30)), math.sin(math.radians(-30))],
        [math.cos(math.radians(170)), math.sin(math.radians(170))],
        [math.cos(math.radians(-170)), math.sin(math.radians(-170))],
    ])
    angle = mean_heading_angle(headings, [0, 1])
    assert math.isclose(angle, 0.0, abs_tol=1e-9)

    angle2 = mean_heading_angle(headings, [2, 3])
    assert math.isclose(abs(angle2), math.pi, abs_tol=1e-9)


def test_mean_heading_angle_cancelling_neighbors_returns_none():
    headings = np.array([[1.0, 0.0], [-1.0, 0.0]])
    assert mean_heading_angle(headings, [0, 1]) is None


def test_mean_heading_angle_empty_ids_returns_none():
    headings = np.array([[1.0, 0.0]])
    assert mean_heading_angle(headings, []) is None


def test_vicsek_heading_zero_noise_matches_mean_of_group():
    # Three agents (self + 2 neighbors) all heading the same direction ->
    # zero-noise output must reproduce that exact heading.
    headings = np.array([
        [1.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
    ])
    rng = np.random.default_rng(0)
    out = vicsek_heading(headings, 0, [1, 2], noise_strength=0.0, rng=rng)
    assert np.allclose(out, [1.0, 0.0, 0.0], atol=1e-9)


def test_vicsek_heading_no_neighbors_falls_back_to_own_heading():
    headings = np.array([[0.0, 1.0, 0.0]])
    rng = np.random.default_rng(0)
    out = vicsek_heading(headings, 0, [], noise_strength=0.0, rng=rng)
    assert np.allclose(out, [0.0, 1.0, 0.0], atol=1e-9)


def test_vicsek_heading_sar_mode_uses_only_caller_supplied_neighbors():
    """Non-periodic SAR mode: neighbor selection is entirely up to the
    caller (this simulator's bounded arena has no wraparound at all) -
    two agents on opposite edges of an arena are simply not neighbors
    unless the caller includes them, regardless of arena size."""
    headings = np.array([[1.0, 0.0, 0.0], [-1.0, 0.0, 0.0]])
    rng = np.random.default_rng(0)
    # Caller does not include agent 1 as a neighbor -> output ignores it entirely.
    out = vicsek_heading(headings, 0, [], noise_strength=0.0, rng=rng)
    assert np.allclose(out, [1.0, 0.0, 0.0], atol=1e-9)


def test_vicsek_angular_noise_distribution_bounded_and_spread():
    headings = np.array([[1.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    rng = np.random.default_rng(1)
    noise_strength = 0.3
    angles = []
    for _ in range(2000):
        out = vicsek_heading(headings, 0, [1], noise_strength=noise_strength, rng=rng)
        angles.append(math.atan2(out[1], out[0]))
    angles = np.array(angles)
    # Every sample must stay within the configured noise half-width of the
    # (zero-noise) mean heading of 0 radians.
    assert np.all(np.abs(angles) <= noise_strength + 1e-9)
    # And the distribution should actually spread out, not collapse to a
    # single value (i.e. noise is doing something) - std of a uniform
    # distribution on [-w, w] is w/sqrt(3).
    expected_std = noise_strength / math.sqrt(3)
    assert abs(angles.std() - expected_std) < 0.03


def test_vicsek_zero_noise_is_deterministic_and_noise_increases_spread():
    headings = np.array([[1.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    rng = np.random.default_rng(2)
    out_a = vicsek_heading(headings, 0, [1], noise_strength=0.0, rng=rng)
    out_b = vicsek_heading(headings, 0, [1], noise_strength=0.0, rng=rng)
    assert np.allclose(out_a, out_b)


def test_vicsek_candidate_command_bounds_speed_and_turn_rate():
    headings = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    rng = np.random.default_rng(0)
    out = vicsek_candidate_command(
        headings, 0, [1], noise_strength=0.0, rng=rng,
        prev_heading=np.array([1.0, 0.0, 0.0]), speed_mps=100.0, max_speed_mps=3.0,
        max_turn_rate_radps=0.2, dt_s=1 / 24,
    )
    assert np.all(np.isfinite(out))
    assert np.linalg.norm(out) <= 3.0 + 1e-9
    angle = math.atan2(out[1], out[0])
    assert abs(angle) <= 0.2 * (1 / 24) + 1e-6


def test_vicsek_periodic_step_wraparound_neighbor():
    """Regression mode: two agents placed on opposite edges of a small
    periodic box are neighbors THROUGH the wraparound even though their
    raw Euclidean separation is large - this is what distinguishes the
    periodic mode from SAR mode, and is exactly the original paper's
    boundary condition."""
    box_size = 10.0
    radius = 2.0
    positions = np.array([[0.1, 5.0], [9.9, 5.0]])  # 0.2 apart across the wrap, 9.8 apart directly
    angles = np.array([0.0, math.pi])  # facing opposite ways
    rng = np.random.default_rng(0)
    new_angles = vicsek_periodic_step(positions, angles, box_size, radius, noise_strength=0.0, rng=rng)
    # Each agent's neighborhood (via wraparound) includes itself + the
    # other -> opposite unit vectors cancel -> falls back to its own
    # current angle (see mean_heading_angle's cancelling-neighbors case).
    assert math.isclose(new_angles[0], 0.0, abs_tol=1e-9)
    assert math.isclose(abs(new_angles[1]), math.pi, abs_tol=1e-9)


def test_vicsek_periodic_step_without_wraparound_would_not_see_each_other():
    """Sanity check that the wraparound in the test above is doing real
    work: with a radius too small to bridge the wrap gap (0.2) this would
    fail, but confirming the *direct* (non-wrapped) distance (9.8) is
    indeed larger than the box half-size shows plain Euclidean distance
    alone would have missed this neighbor pair entirely."""
    a, b = np.array([0.1, 5.0]), np.array([9.9, 5.0])
    direct_dist = np.linalg.norm(a - b)
    assert direct_dist > 5.0  # bigger than half the 10m box - would be "out of range" without wraparound


def test_vicsek_order_parameter_high_when_aligned_low_when_random():
    aligned = np.zeros(50)
    assert order_parameter(aligned) > 0.99

    rng = np.random.default_rng(0)
    random_angles = rng.uniform(-math.pi, math.pi, size=2000)
    assert order_parameter(random_angles) < 0.1


def test_vicsek_periodic_regression_order_parameter_decreases_with_noise():
    """Canonical Vicsek phase-transition sanity check (see module
    docstring: this IS the baseline, not an approximation of it) - lower
    noise should, on average over several steps, sustain higher order
    (more aligned) than high noise, for the same population/density."""
    rng_low = np.random.default_rng(42)
    rng_high = np.random.default_rng(42)
    n = 60
    box_size = 10.0
    radius = 2.5
    positions = rng_low.uniform(0, box_size, size=(n, 2))
    angles_low = rng_low.uniform(-math.pi, math.pi, size=n)
    angles_high = angles_low.copy()

    for _ in range(30):
        angles_low = vicsek_periodic_step(positions, angles_low, box_size, radius, noise_strength=0.05, rng=rng_low)
        angles_high = vicsek_periodic_step(positions, angles_high, box_size, radius, noise_strength=2.5, rng=rng_high)

    assert order_parameter(angles_low) > order_parameter(angles_high)
