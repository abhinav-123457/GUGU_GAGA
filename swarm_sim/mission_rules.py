"""Phase 16A: the competition's hard numeric limits, in one place, so later
phases (fleet sizing, payload, scoring) reference a single source instead of
scattering magic numbers. See docs/PHASE16_GNSS_DENIED_ROADMAP.md.

Sim mass caveat: the PyBullet CF2X drone is ~27 g, so the 25 kg fleet budget
is only checkable against a *declared* real-airframe mass, never against the
simulated body.
"""
from __future__ import annotations

import dataclasses
import math
from typing import Sequence, Tuple

MIN_DRONES = 2                                   # "at least two or more drones"
MAX_SURVIVORS = 10                               # "up to ten survivors"
MAX_TOTAL_MASS_KG = 25.0                         # combined all-up weight, all drones
KIT_MASS_KG = 0.2                                # one survival kit: 200 g
KIT_DIMS_M: Tuple[float, float, float] = (0.20, 0.10, 0.05)   # 20 x 10 x 5 cm


@dataclasses.dataclass(frozen=True)
class FleetMassCheck:
    total_kg: float
    limit_kg: float
    margin_kg: float      # limit - total; negative = over the limit
    ok: bool


def check_fleet_mass(per_drone_kg: Sequence[float], payload_kg: float = 0.0,
                     limit_kg: float = MAX_TOTAL_MASS_KG) -> FleetMassCheck:
    """Combined all-up weight check. `per_drone_kg` are the drones' own
    masses (batteries included); `payload_kg` is the total mass of every kit
    carried at take-off. Exceeding the limit is a *result* (`ok=False`), not
    an exception; only malformed inputs raise."""
    if len(per_drone_kg) < 1:
        raise ValueError("per_drone_kg must list at least one drone")
    values = list(per_drone_kg) + [payload_kg, limit_kg]
    if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in values):
        raise ValueError("masses and limit must be finite numbers")
    if any(v < 0.0 for v in list(per_drone_kg) + [payload_kg]) or limit_kg <= 0.0:
        raise ValueError("masses must be >= 0 and limit_kg > 0")
    total = float(sum(per_drone_kg)) + float(payload_kg)
    return FleetMassCheck(total_kg=total, limit_kg=float(limit_kg), margin_kg=float(limit_kg) - total,
                          ok=total <= limit_kg)
