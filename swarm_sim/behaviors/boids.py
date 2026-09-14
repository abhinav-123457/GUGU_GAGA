"""Reynolds, C. W. (1987) - Flocks, herds and schools: a distributed behavioral model.

Three local steering rules combined into one heading: separation (avoid
crowding), alignment (match neighbors' heading), cohesion (move toward the
local center of mass).
"""
import numpy as np


def separation(positions, i, neighbor_ids, radius):
    if len(neighbor_ids) == 0:
        return np.zeros(3)
    delta = positions[i] - positions[neighbor_ids]
    dist = np.linalg.norm(delta, axis=1)
    close = dist < radius
    if not np.any(close):
        return np.zeros(3)
    weighted = delta[close] / np.clip(dist[close, None], 1e-6, None) ** 2
    return weighted.sum(axis=0)


def alignment(velocities, i, neighbor_ids):
    if len(neighbor_ids) == 0:
        return np.zeros(3)
    return velocities[neighbor_ids].mean(axis=0) - velocities[i]


def cohesion(positions, i, neighbor_ids):
    if len(neighbor_ids) == 0:
        return np.zeros(3)
    center = positions[neighbor_ids].mean(axis=0)
    return center - positions[i]


def boids_steer(positions, velocities, i, neighbor_ids, radius, w_sep=1.5, w_align=1.0, w_coh=1.0):
    sep = separation(positions, i, neighbor_ids, radius)
    ali = alignment(velocities, i, neighbor_ids)
    coh = cohesion(positions, i, neighbor_ids)
    return w_sep * sep + w_align * ali + w_coh * coh
