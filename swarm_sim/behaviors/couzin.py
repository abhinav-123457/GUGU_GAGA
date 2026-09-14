"""Couzin, I. D. et al. (2002) - Collective memory and spatial sorting in
animal groups.

Zonal model with strict priority: a neighbor inside the zone of repulsion
overrides everything else (collision/crowding avoidance always wins); absent
that, headings blend the zone of orientation (align) and zone of attraction
(cohere). A field-of-view angle models the blind zone behind each agent.
"""
import numpy as np


def _in_fov(rel_vec, heading, fov_deg):
    if fov_deg >= 360:
        return True
    n1 = np.linalg.norm(rel_vec)
    if n1 < 1e-9:
        return True
    cos_angle = np.dot(rel_vec, heading) / (n1 * np.linalg.norm(heading) + 1e-9)
    cos_angle = np.clip(cos_angle, -1.0, 1.0)
    return np.degrees(np.arccos(cos_angle)) <= fov_deg / 2


def couzin_direction(positions, headings, i, neighbor_ids, r_repulsion, r_orientation, r_attraction, fov_deg=270.0):
    pos_i, head_i = positions[i], headings[i]
    repulsion = np.zeros(3)
    orientation = np.zeros(3)
    attraction = np.zeros(3)
    n_rep = n_ori = n_att = 0

    for j in neighbor_ids:
        rel = positions[j] - pos_i
        dist = np.linalg.norm(rel)
        if dist < 1e-9 or dist > r_attraction:
            continue
        if dist >= r_repulsion and not _in_fov(rel, head_i, fov_deg):
            continue
        if dist < r_repulsion:
            repulsion -= rel / dist
            n_rep += 1
        elif dist < r_orientation:
            orientation += headings[j]
            n_ori += 1
        else:
            attraction += rel / dist
            n_att += 1

    if n_rep > 0:
        desired = repulsion / n_rep
    else:
        desired = np.zeros(3)
        if n_ori > 0:
            desired += orientation / n_ori
        if n_att > 0:
            desired += attraction / n_att
        if n_ori == 0 and n_att == 0:
            desired = head_i.copy()

    norm = np.linalg.norm(desired)
    return desired / norm if norm > 1e-9 else head_i
