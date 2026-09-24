"""Phase 16B: the one carrier of simulator ground truth.

`PlantTruth` wraps what PyBullet reports about every drone this tick. It
exists so the *type name itself* is the boundary: the identifier
`PlantTruth` (and this module, and `estimation.plant_odometry` /
`estimation.metrics`) is forbidden by tests/test_phase16b_architecture.py in
every autonomy module - controller, safety supervisor, recruitment,
consensus, network, behaviours, mapping and the truth-free half of
`estimation`. Code that holds a `PlantTruth` is plant-side (sensor models,
scoring, the physics loop) by definition. See
docs/PHASE16B_ESTIMATION.md.
"""
from __future__ import annotations

import dataclasses

import numpy as np


@dataclasses.dataclass(frozen=True, eq=False)
class PlantTruth:
    t: float
    positions: np.ndarray    # (N, 3) world-frame metres, PyBullet truth
    velocities: np.ndarray   # (N, 3) world-frame m/s
    rpys: np.ndarray         # (N, 3) roll, pitch, yaw in radians
