"""Phase 3 Olfati-Saber-inspired controller tests - see
docs/PHASE3_BEHAVIORS.md. Required items covered: Olfati-Saber spacing
behavior, graph fragmentation detection (+ connectivity metrics,
candidate-command bounds)."""
import numpy as np

from swarm_sim.behaviors.olfati_saber import (
    algebraic_connectivity, bump, build_adjacency, connected_components,
    is_fragmented, olfati_saber_candidate_command, phi_action, sigma_gradient,
    sigma_norm, spacing_term,
)

D_ALPHA = 4.0
R_ALPHA = 6.0


def test_sigma_norm_zero_at_origin_and_finite_everywhere():
    assert sigma_norm(np.array([0.0, 0.0])) == 0.0
    assert np.isfinite(sigma_norm(np.array([1e6, 1e6])))


def test_sigma_gradient_finite_at_zero_separation():
    grad = sigma_gradient(np.array([[0.0, 0.0]]))
    assert np.all(np.isfinite(grad))
    assert np.allclose(grad, 0.0)


def test_bump_function_shape():
    z = np.array([0.0, 0.1, 0.5, 1.0, 1.5])
    out = bump(z)
    assert out[0] == 1.0            # flat region
    assert 0.0 < out[2] < 1.0        # taper region
    assert out[4] == 0.0             # beyond cutoff


def test_phi_action_sign_change_at_desired_distance():
    # phi_action(z, d_alpha) scales n_ij, which points FROM this agent
    # TOWARD the neighbor - so it must be negative (force away from the
    # neighbor - repulsion) when closer than d_alpha, positive (force
    # toward the neighbor - attraction) when farther, ~zero at d_alpha.
    assert phi_action(np.array(1.0), D_ALPHA) < 0
    assert phi_action(np.array(8.0), D_ALPHA) > 0
    assert abs(phi_action(np.array(D_ALPHA), D_ALPHA)) < 1e-9


def test_spacing_term_repels_when_too_close():
    own_pos = np.array([0.0, 0.0, 3.0])
    neighbor = np.array([[1.0, 0.0, 3.0]])  # much closer than D_ALPHA
    force = spacing_term(own_pos, neighbor, D_ALPHA, R_ALPHA)
    # Repulsion should push agent 0 away from the neighbor (-x direction).
    assert force[0] < 0


def test_spacing_term_attracts_when_too_far():
    own_pos = np.array([0.0, 0.0, 3.0])
    neighbor = np.array([[5.5, 0.0, 3.0]])  # farther than D_ALPHA, within R_ALPHA
    force = spacing_term(own_pos, neighbor, D_ALPHA, R_ALPHA)
    assert force[0] > 0


def test_spacing_term_zero_with_no_neighbors():
    own_pos = np.array([0.0, 0.0, 3.0])
    force = spacing_term(own_pos, np.empty((0, 3)), D_ALPHA, R_ALPHA)
    assert np.allclose(force, 0.0)


def test_spacing_term_finite_for_coincident_neighbor():
    own_pos = np.array([0.0, 0.0, 3.0])
    neighbor = np.array([[0.0, 0.0, 3.0]])  # exactly coincident
    force = spacing_term(own_pos, neighbor, D_ALPHA, R_ALPHA)
    assert np.all(np.isfinite(force))


def test_candidate_command_bounded_and_finite_no_neighbors():
    out = olfati_saber_candidate_command(
        own_pos=np.array([0.0, 0.0, 3.0]), own_vel=np.array([1.0, 0.0, 0.0]),
        neighbor_positions=np.empty((0, 3)), neighbor_velocities=np.empty((0, 3)),
        target_pos=np.array([0.0, 0.0, 3.0]), target_vel=np.array([1.0, 0.0, 0.0]),
        d_alpha=D_ALPHA, r_alpha=R_ALPHA, max_accel_mps2=2.0, max_speed_mps=5.0, dt_s=1 / 24,
    )
    assert np.all(np.isfinite(out))
    assert np.linalg.norm(out) <= 5.0 + 1e-9
    assert out[2] == 0.0


def test_candidate_command_bounded_for_extremely_close_neighbor():
    out = olfati_saber_candidate_command(
        own_pos=np.array([0.0, 0.0, 3.0]), own_vel=np.zeros(3),
        neighbor_positions=np.array([[0.01, 0.0, 3.0]]), neighbor_velocities=np.array([[0.0, 0.0, 0.0]]),
        target_pos=np.array([0.0, 0.0, 3.0]), target_vel=np.zeros(3),
        d_alpha=D_ALPHA, r_alpha=R_ALPHA, max_accel_mps2=2.0, max_speed_mps=5.0, dt_s=1 / 24,
    )
    assert np.all(np.isfinite(out))
    assert np.linalg.norm(out) <= 5.0 + 1e-9


# --- connectivity / fragmentation ----------------------------------------

def test_build_adjacency_symmetric_no_self_loops():
    positions = np.array([[0.0, 0.0], [1.0, 0.0], [10.0, 10.0]])
    adj = build_adjacency(positions, radius=2.0)
    assert np.array_equal(adj, adj.T)
    assert np.all(np.diag(adj) == 0)
    assert adj[0, 1] == 1
    assert adj[0, 2] == 0


def test_connected_components_single_component():
    positions = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]])
    adj = build_adjacency(positions, radius=1.5)
    comps = connected_components(adj)
    assert len(comps) == 1
    assert not is_fragmented(adj)


def test_fragmentation_detection_two_components():
    positions = np.array([[0.0, 0.0], [1.0, 0.0], [50.0, 50.0], [51.0, 50.0]])
    adj = build_adjacency(positions, radius=2.0)
    comps = connected_components(adj)
    assert len(comps) == 2
    assert is_fragmented(adj)
    assert sorted(comps) == [[0, 1], [2, 3]]


def test_algebraic_connectivity_zero_when_fragmented_positive_when_connected():
    frag_positions = np.array([[0.0, 0.0], [1.0, 0.0], [50.0, 50.0], [51.0, 50.0]])
    connected_positions = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [3.0, 0.0]])
    frag_adj = build_adjacency(frag_positions, radius=2.0)
    connected_adj = build_adjacency(connected_positions, radius=1.5)
    assert algebraic_connectivity(frag_adj) == 0.0
    assert algebraic_connectivity(connected_adj) > 0.0


def test_algebraic_connectivity_single_node():
    assert algebraic_connectivity(np.zeros((1, 1), dtype=int)) == 0.0
