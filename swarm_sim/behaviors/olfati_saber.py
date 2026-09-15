"""Olfati-Saber, R. (2006) - Flocking for multi-agent dynamic systems:
algorithms and theory. IEEE Trans. Automatic Control 51(3).

Phase 3 "review" deliverable (docs/PHASE3_BEHAVIORS.md): this module did
not exist before Phase 3. It is explicitly split into two tiers so it is
never mistaken for a from-the-paper guarantee this codebase does not
actually provide:

FROM THE PAPER, implemented per its published formulas:
  - the sigma-norm `sigma_norm` (eq. 3.3) and its gradient direction
    `sigma_gradient` (eq. 3.4) - a smooth stand-in for Euclidean distance
    with no singularity at zero separation;
  - the bump function `bump` (eq. 3.6) and pairwise action function
    `phi_alpha` (eq. 3.9-3.11), which together produce the SPACING term
    (a smooth, bounded, distance-based repulsion-then-attraction around a
    desired inter-agent distance `d_alpha`);
  - the velocity-consensus ALIGNMENT term (eq. 3.13's a_ij(q) weighting
    applied to (p_j - p_i)).

ENGINEERING APPROXIMATIONS, not from the paper's formal guarantees:
  - NAVIGATION: the paper tracks a dedicated virtual "gamma-agent" with
    its own dynamics; this module instead takes whatever
    (target_position_m, target_velocity_mps) SwarmController already has
    on hand (a recruitment beacon, or the current search heading held at
    cruise speed) and feeds those into the same navigational-feedback
    functional form (eq. 3.20). There is no virtual-leader subsystem here.
  - CONNECTIVITY / FRAGMENTATION: `connected_components`/
    `algebraic_connectivity` MEASURE the current snapshot of a
    locally-sensed interaction graph (built by the caller from real
    neighbor observations, never ground truth). The paper's Theorem 2.2/
    3.1-class results (the flocking potential's minimum preserves initial
    connectivity under continuous-time gradient dynamics with no sensing
    noise/dropout/latency) are NOT reproduced or claimed here - this
    simulator's sensing is noisy, latent, and dropout-prone (Phase 2), and
    control runs in discrete steps. These functions are a monitoring/
    diagnostic tool only (see docs/PHASE3_BEHAVIORS.md's fragmentation-
    detection benchmark), not a connectivity guarantee.

Output contract (frame/units/bounds) for olfati_saber_candidate_command:
see swarm_sim/behaviors/common.py's module docstring - like Boids, this
model's native output is an acceleration (the paper's u_i drives a
double-integrator q_dot=p, p_dot=u), so bounding follows Boids' pattern:
clamp acceleration, integrate one step, clamp speed.
"""
import math

import numpy as np

from .common import bounded_heading, clamp_acceleration, clamp_speed

# Bump function / action function shape parameters - Olfati-Saber (2006)'s
# own worked example values (section VII), not tuned for this simulator.
BUMP_H = 0.2
PHI_A = 5.0
PHI_B = 5.0
PHI_C = abs(PHI_A - PHI_B) / np.sqrt(4 * PHI_A * PHI_B) if PHI_A != PHI_B else 0.0
SIGMA_EPS = 0.1


def sigma_norm(z, eps=SIGMA_EPS):
    """eq. 3.3: a smooth norm with no gradient singularity at z=0, unlike
    Euclidean norm. z: (..., d) array of relative-position vectors."""
    z = np.asarray(z, dtype=float)
    sq = np.sum(z * z, axis=-1)
    return (np.sqrt(1.0 + eps * sq) - 1.0) / eps


def sigma_gradient(z, eps=SIGMA_EPS):
    """eq. 3.4: the sigma-norm's gradient direction, n_ij = z / sqrt(1 +
    eps*||z||^2) - finite everywhere, including z=0 (returns the zero
    vector there rather than dividing by zero)."""
    z = np.asarray(z, dtype=float)
    sq = np.sum(z * z, axis=-1, keepdims=True)
    return z / np.sqrt(1.0 + eps * sq)


def bump(z, h=BUMP_H):
    """eq. 3.6: rho_h(z) - 1 for z in [0, h), smooth cosine taper to 0 over
    [h, 1], 0 for z > 1 (and z < 0, symmetric use only needs z >= 0 here)."""
    z = np.asarray(z, dtype=float)
    out = np.zeros_like(z)
    in_flat = z < h
    in_taper = (z >= h) & (z <= 1.0)
    out[in_flat] = 1.0
    out[in_taper] = 0.5 * (1.0 + np.cos(np.pi * (z[in_taper] - h) / (1.0 - h)))
    return out


def _sigma_1(z):
    return z / np.sqrt(1.0 + z ** 2)


def phi_action(z, d_alpha, a=PHI_A, b=PHI_B, c=PHI_C):
    """eq. 3.10: phi(z) = 0.5*[(a+b)*sigma_1(z+c) + (a-b)] evaluated at
    (z - d_alpha) - negative (attractive) for z > d_alpha, positive
    (repulsive) for z < d_alpha, zero at z == d_alpha."""
    return 0.5 * ((a + b) * _sigma_1(z - d_alpha + c) + (a - b))


def _sigma_norm_scalar(d, eps=SIGMA_EPS):
    """sigma_norm of a plain scalar real-world distance `d` (no direction
    needed - sigma_norm only depends on magnitude). Used to convert the
    real-meter d_alpha_m/r_alpha_m parameters callers pass (matching this
    codebase's other distance config fields, all in meters) into the
    sigma-norm units phi_action/bump actually operate in - see the module
    docstring's sigma_norm paragraph. Without this conversion, comparing a
    sigma-norm distance z against a raw-meter r_alpha silently cuts every
    real neighbor off far too early (sigma_norm(d) > d for d > 0)."""
    return (math.sqrt(1.0 + eps * d * d) - 1.0) / eps


def phi_alpha(z, d_alpha_m, r_alpha_m, eps=SIGMA_EPS, h=BUMP_H):
    """eq. 3.11: the bump-windowed action function - phi_action smoothly
    cut off to zero beyond the sensing range. `z` is a sigma-norm distance
    (from sigma_norm); `d_alpha_m`/`r_alpha_m` are plain real-meter
    distances, converted to the same sigma-norm scale internally."""
    d_alpha = _sigma_norm_scalar(d_alpha_m, eps)
    r_alpha = _sigma_norm_scalar(r_alpha_m, eps)
    return bump(z / r_alpha, h) * phi_action(z, d_alpha)


def spacing_term(own_pos, neighbor_positions, d_alpha_m, r_alpha_m, eps=SIGMA_EPS, h=BUMP_H):
    """Gradient-based spacing force, summed over neighbors (eq. 3.9's
    u_i^alpha spacing half): each neighbor contributes phi_alpha(sigma-
    distance) along the sigma-gradient direction toward/away from it.
    `d_alpha_m`/`r_alpha_m` are real-meter distances (see phi_alpha).
    Returns the zero vector for zero neighbors (safe, not NaN)."""
    if len(neighbor_positions) == 0:
        return np.zeros(own_pos.shape[-1])
    rel = np.asarray(neighbor_positions) - own_pos
    z = sigma_norm(rel, eps)
    n_ij = sigma_gradient(rel, eps)
    weights = phi_alpha(z, d_alpha_m, r_alpha_m, eps, h)
    return (weights[:, None] * n_ij).sum(axis=0)


def alignment_term(own_pos, own_vel, neighbor_positions, neighbor_velocities, r_alpha_m, eps=SIGMA_EPS, h=BUMP_H):
    """Velocity-consensus force (eq. 3.13's a_ij(q) weighting): each
    neighbor pulls this agent's velocity toward its own, weighted by the
    same bump function of sigma-distance used for spacing (so a neighbor
    at the edge of sensing range contributes smoothly less). `r_alpha_m`
    is a real-meter distance (see phi_alpha). Zero vector for zero
    neighbors."""
    if len(neighbor_positions) == 0:
        return np.zeros(own_vel.shape[-1])
    rel = np.asarray(neighbor_positions) - own_pos
    z = sigma_norm(rel, eps)
    r_alpha = _sigma_norm_scalar(r_alpha_m, eps)
    a_ij = bump(z / r_alpha, h)
    vel_diff = np.asarray(neighbor_velocities) - own_vel
    return (a_ij[:, None] * vel_diff).sum(axis=0)


def navigation_term(own_pos, own_vel, target_pos, target_vel, c1=1.0, c2=1.0):
    """eq. 3.20-style navigational feedback toward a group objective -
    ENGINEERING APPROXIMATION (see module docstring): target_pos/target_vel
    stand in for the paper's dedicated virtual leader / gamma-agent."""
    pos_err = np.asarray(own_pos) - np.asarray(target_pos)
    vel_err = np.asarray(own_vel) - np.asarray(target_vel)
    return -c1 * _sigma_1(pos_err) - c2 * vel_err


def olfati_saber_steer(own_pos, own_vel, neighbor_positions, neighbor_velocities,
                        target_pos, target_vel, d_alpha, r_alpha,
                        c_spacing=1.0, c_align=1.0, c_nav=1.0, eps=SIGMA_EPS, h=BUMP_H):
    """Raw steering acceleration (NOT yet bounded - see
    olfati_saber_candidate_command for the contract-compliant entry
    point): weighted sum of the spacing, alignment, and navigation terms
    above. `d_alpha`/`r_alpha` here are real-meter distances (converted to
    sigma-norm units internally by spacing_term/alignment_term - see
    phi_alpha's docstring)."""
    own_pos = np.asarray(own_pos, dtype=float)
    own_vel = np.asarray(own_vel, dtype=float)
    spacing = spacing_term(own_pos, neighbor_positions, d_alpha, r_alpha, eps, h)
    align = alignment_term(own_pos, own_vel, neighbor_positions, neighbor_velocities, r_alpha, eps, h)
    nav = navigation_term(own_pos, own_vel, target_pos, target_vel)
    return c_spacing * spacing + c_align * align + c_nav * nav


def olfati_saber_candidate_command(own_pos, own_vel, neighbor_positions, neighbor_velocities,
                                    target_pos, target_vel, d_alpha, r_alpha,
                                    max_accel_mps2, max_speed_mps, dt_s,
                                    c_spacing=1.0, c_align=1.0, c_nav=1.0):
    """Contract-compliant candidate command: integrates the (acceleration-
    bounded) steering force for one control step and clips the result to
    max_speed_mps. `d_alpha`/`r_alpha` are real-meter distances (see
    olfati_saber_steer). Returns a finite 3-vector velocity, LOCAL_ENU,
    m/s (z=0) - see swarm_sim/behaviors/common.py."""
    steer = olfati_saber_steer(own_pos, own_vel, neighbor_positions, neighbor_velocities,
                                target_pos, target_vel, d_alpha, r_alpha, c_spacing, c_align, c_nav)
    accel = clamp_acceleration(steer, max_accel_mps2)
    if not np.all(np.isfinite(accel)):
        accel = np.zeros_like(accel)
    own_vel = np.asarray(own_vel, dtype=float)
    if not np.all(np.isfinite(own_vel)):
        own_vel = np.zeros_like(accel)
    candidate = own_vel + accel * dt_s
    if candidate.shape[-1] >= 3:
        candidate[..., 2] = 0.0
    if not np.all(np.isfinite(candidate)) or np.linalg.norm(candidate) < 1e-9:
        candidate = bounded_heading(own_vel, np.array([1.0, 0.0, 0.0])) * min(
            max_speed_mps, float(np.linalg.norm(own_vel)),
        )
    return clamp_speed(candidate, max_speed_mps)


# --- Connectivity / fragmentation monitoring (evaluation-only) -----------
# See module docstring's "ENGINEERING APPROXIMATIONS" note: these measure
# a locally-sensed graph snapshot, they do not prove or enforce anything.

def build_adjacency(positions, radius):
    """Symmetric 0/1 adjacency: nodes i != j are connected iff their
    distance is < radius. `positions`: (N, d) array (any d)."""
    positions = np.asarray(positions, dtype=float)
    n = positions.shape[0]
    diff = positions[:, None, :] - positions[None, :, :]
    dist = np.linalg.norm(diff, axis=-1)
    adjacency = (dist < radius).astype(int)
    np.fill_diagonal(adjacency, 0)
    return adjacency


def connected_components(adjacency):
    """List of components (each a sorted list of node indices) via BFS
    over the adjacency matrix."""
    n = adjacency.shape[0]
    visited = [False] * n
    components = []
    for start in range(n):
        if visited[start]:
            continue
        stack, comp = [start], []
        visited[start] = True
        while stack:
            node = stack.pop()
            comp.append(node)
            for nb in np.nonzero(adjacency[node])[0]:
                if not visited[nb]:
                    visited[nb] = True
                    stack.append(int(nb))
        components.append(sorted(comp))
    return components


def is_fragmented(adjacency) -> bool:
    return len(connected_components(adjacency)) > 1


def algebraic_connectivity(adjacency) -> float:
    """Fiedler value: the second-smallest eigenvalue of the graph
    Laplacian (degree matrix minus adjacency). Exactly 0 iff the graph has
    more than one connected component; larger values indicate a more
    robustly connected (harder to fragment) topology. Returns 0.0 for
    fewer than 2 nodes (undefined/trivially connected)."""
    n = adjacency.shape[0]
    if n < 2:
        return 0.0
    degree = np.diag(adjacency.sum(axis=1))
    laplacian = degree - adjacency
    eigenvalues = np.sort(np.linalg.eigvalsh(laplacian.astype(float)))
    return float(eigenvalues[1])
