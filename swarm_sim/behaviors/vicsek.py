"""Vicsek, T. et al. (1995) - Novel type of phase transition in a system of
self-driven particles.

Each agent adopts the average heading of its neighbors (itself included),
perturbed by noise. Used here as a lightweight alternative flocking model to
Couzin's zonal one (selectable via MissionConfig.flock_model).
"""
import numpy as np


def vicsek_heading(headings, i, neighbor_ids, noise_strength, rng):
    ids = np.append(neighbor_ids, i).astype(int)
    mean_dir = headings[ids].mean(axis=0)
    norm = np.linalg.norm(mean_dir)
    base = mean_dir / norm if norm > 1e-9 else headings[i]
    noisy = base + rng.normal(scale=noise_strength, size=3)
    n = np.linalg.norm(noisy)
    return noisy / n if n > 1e-9 else base
