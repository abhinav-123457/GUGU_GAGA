"""Phase 3 common-contract helper tests (swarm_sim/behaviors/common.py) -
see docs/PHASE3_BEHAVIORS.md."""
import numpy as np

from swarm_sim.behaviors.common import (
    all_finite, bounded_heading, clamp_acceleration, clamp_speed, clamp_turn_rate, sanitize_vector,
)


def test_sanitize_vector_replaces_nan():
    out = sanitize_vector(np.array([np.nan, 1.0, 0.0]), np.array([1.0, 0.0, 0.0]))
    assert np.array_equal(out, np.array([1.0, 0.0, 0.0]))


def test_bounded_heading_normalizes():
    out = bounded_heading(np.array([3.0, 4.0, 0.0]), np.array([1.0, 0.0, 0.0]))
    assert np.isclose(np.linalg.norm(out), 1.0)


def test_bounded_heading_falls_back_on_zero_vector():
    out = bounded_heading(np.zeros(3), np.array([0.0, 1.0, 0.0]))
    assert np.allclose(out, [0.0, 1.0, 0.0])


def test_bounded_heading_falls_back_on_nan():
    out = bounded_heading(np.array([np.nan, np.nan, 0.0]), np.array([0.0, 1.0, 0.0]))
    assert np.allclose(out, [0.0, 1.0, 0.0])
    assert np.all(np.isfinite(out))


def test_clamp_speed_bounds_magnitude_preserves_direction():
    v = np.array([10.0, 0.0, 0.0])
    out = clamp_speed(v, 2.0)
    assert np.isclose(np.linalg.norm(out), 2.0)
    assert np.allclose(out / np.linalg.norm(out), v / np.linalg.norm(v))


def test_clamp_speed_leaves_slow_vectors_unchanged():
    v = np.array([0.5, 0.0, 0.0])
    assert np.allclose(clamp_speed(v, 2.0), v)


def test_clamp_acceleration_is_same_bound_as_clamp_speed():
    v = np.array([0.0, 5.0, 0.0])
    out = clamp_acceleration(v, 1.5)
    assert np.isclose(np.linalg.norm(out), 1.5)


def test_clamp_turn_rate_limits_large_heading_change():
    prev = np.array([1.0, 0.0, 0.0])
    new = np.array([0.0, 1.0, 0.0])  # 90 degree change
    out = clamp_turn_rate(new, prev, max_turn_rate_radps=1.0, dt_s=0.1)  # allows only 0.1 rad
    angle = np.arctan2(out[1], out[0])
    assert np.isclose(angle, 0.1, atol=1e-6)
    assert np.isclose(np.linalg.norm(out), 1.0)


def test_clamp_turn_rate_passes_through_small_change():
    prev = np.array([1.0, 0.0, 0.0])
    new_angle = 0.01
    new = np.array([np.cos(new_angle), np.sin(new_angle), 0.0])
    out = clamp_turn_rate(new, prev, max_turn_rate_radps=5.0, dt_s=0.1)
    assert np.isclose(np.arctan2(out[1], out[0]), new_angle, atol=1e-6)


def test_all_finite_true_and_false():
    assert all_finite(np.array([1.0, 2.0]), np.array([0.0]))
    assert not all_finite(np.array([1.0, np.nan]))
