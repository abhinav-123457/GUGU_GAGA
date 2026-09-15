"""Phase 3 Reynolds Boids tests - see docs/PHASE3_BEHAVIORS.md.
Required items covered: Boids empty-neighbor behavior, Boids
coincident-neighbor behavior, separation priority, speed and acceleration
bounds."""
import numpy as np

from swarm_sim.behaviors.boids import (
    alignment, boids_candidate_command, boids_steer, cohesion, separation,
)


def test_boids_empty_neighbors_all_terms_zero():
    positions = np.array([[0.0, 0.0, 3.0]])
    velocities = np.array([[1.0, 0.0, 0.0]])
    assert np.allclose(separation(positions, 0, [], 5.0), 0.0)
    assert np.allclose(alignment(positions, velocities, 0, [], 5.0), 0.0)
    assert np.allclose(cohesion(positions, 0, [], 5.0), 0.0)
    assert np.allclose(boids_steer(positions, velocities, 0, [], 5.0), 0.0)


def test_boids_candidate_command_safe_with_no_neighbors():
    positions = np.array([[0.0, 0.0, 3.0]])
    velocities = np.array([[1.0, 0.0, 0.0]])
    out = boids_candidate_command(
        positions, velocities, 0, [], radius=5.0, prev_velocity=np.array([1.0, 0.0, 0.0]),
        max_accel_mps2=2.0, max_speed_mps=5.0, dt_s=1 / 24,
    )
    assert np.all(np.isfinite(out))
    assert np.linalg.norm(out) > 0.0
    assert out[2] == 0.0


def test_boids_coincident_neighbor_no_nan():
    # Neighbor at the exact same position as agent 0.
    positions = np.array([[0.0, 0.0, 3.0], [0.0, 0.0, 3.0]])
    velocities = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    sep = separation(positions, 0, [1], radius=5.0)
    assert np.all(np.isfinite(sep))
    assert np.allclose(sep, 0.0)  # coincident neighbor excluded, not a huge spurious force
    steer = boids_steer(positions, velocities, 0, [1], radius=5.0)
    assert np.all(np.isfinite(steer))
    out = boids_candidate_command(
        positions, velocities, 0, [1], radius=5.0, prev_velocity=np.array([1.0, 0.0, 0.0]),
        max_accel_mps2=2.0, max_speed_mps=5.0, dt_s=1 / 24,
    )
    assert np.all(np.isfinite(out))


def test_boids_separation_priority_overrides_cohesion_alignment():
    # Neighbor very close (inside priority radius) but with a velocity/
    # position that would otherwise pull cohesion/alignment the opposite
    # way from separation - separation alone must win.
    positions = np.array([[0.0, 0.0, 3.0], [0.2, 0.0, 3.0]])
    velocities = np.array([[0.0, 0.0, 0.0], [-5.0, 0.0, 0.0]])
    steer = boids_steer(positions, velocities, 0, [1], radius=5.0, priority_radius=2.5)
    sep = separation(positions, 0, [1], radius=5.0)
    assert np.allclose(steer, sep)
    # The neighbor sits at +x, so separation alone must push agent 0 in -x -
    # exactly opposite of what alignment (neighbor moving at -5 in x) would
    # otherwise blend in were priority not enforced.
    assert steer[0] < 0


def test_boids_far_neighbor_blends_all_three_terms():
    positions = np.array([[0.0, 0.0, 3.0], [4.0, 0.0, 3.0]])
    velocities = np.array([[0.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    steer = boids_steer(positions, velocities, 0, [1], radius=5.0, priority_radius=1.0)
    sep = separation(positions, 0, [1], radius=5.0)
    assert not np.allclose(steer, sep)  # blended, not separation-only


def test_boids_candidate_command_bounds_acceleration_and_speed():
    positions = np.array([[0.0, 0.0, 3.0], [0.05, 0.0, 3.0]])  # extremely close -> huge raw separation
    velocities = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    prev_v = np.array([0.0, 0.0, 0.0])
    out = boids_candidate_command(
        positions, velocities, 0, [1], radius=5.0, prev_velocity=prev_v,
        max_accel_mps2=2.0, max_speed_mps=5.0, dt_s=1 / 24,
    )
    assert np.all(np.isfinite(out))
    assert np.linalg.norm(out) <= 5.0 + 1e-9
    # One step from rest, bounded acceleration over dt cannot exceed accel*dt.
    assert np.linalg.norm(out - prev_v) <= 2.0 * (1 / 24) + 1e-9


def test_boids_candidate_command_speed_never_exceeds_max_over_many_steps():
    rng = np.random.default_rng(0)
    positions = np.array([[0.0, 0.0, 3.0], [1.0, 0.5, 3.0]])
    velocities = np.array([[0.0, 0.0, 0.0], [3.0, -2.0, 0.0]])
    v = np.array([0.0, 0.0, 0.0])
    for _ in range(200):
        v = boids_candidate_command(
            positions, velocities, 0, [1], radius=5.0, prev_velocity=v,
            max_accel_mps2=3.0, max_speed_mps=4.0, dt_s=1 / 24,
        )
        assert np.all(np.isfinite(v))
        assert np.linalg.norm(v) <= 4.0 + 1e-6
