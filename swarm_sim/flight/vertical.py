"""Phase 16C: the outer altitude loop.

`vz* = clip(kp * (z_target - z_estimate))`, slew-limited against the controller's OWN previous output.
The measured vertical speed is deliberately not an input: the mission's inner loop (the plant's velocity
tracking) already damps, and feeding the noisy estimated vertical speed back here is what made Phase 16B's
drones random-walk in altitude (docs/PHASE16B_ESTIMATION.md, finding 7). Gains are illustrative.

Pure numpy-free arithmetic on floats; it sees only the drone's own estimate."""
from __future__ import annotations

import dataclasses
import math
from typing import Dict


def _positive(name: str, value) -> None:
    if not (isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) and value > 0.0):
        raise ValueError(f"{name} must be a finite number > 0, got {value!r}")


@dataclasses.dataclass(frozen=True)
class VerticalConfig:
    kp_per_s: float = 1.0
    max_climb_mps: float = 1.5
    max_descent_mps: float = 1.0
    slew_mps2: float = 1.0

    def __post_init__(self):
        for name in ("kp_per_s", "max_climb_mps", "max_descent_mps", "slew_mps2"):
            _positive(name, getattr(self, name))

    @classmethod
    def from_mission_config(cls, cfg) -> "VerticalConfig":
        return cls(kp_per_s=cfg.altitude_kp_per_s, max_climb_mps=cfg.altitude_max_climb_mps,
                   max_descent_mps=cfg.altitude_max_descent_mps, slew_mps2=cfg.altitude_slew_mps2)


class VerticalController:
    """One altitude loop per drone."""

    def __init__(self, num_drones: int, config: VerticalConfig):
        if not (isinstance(num_drones, int) and not isinstance(num_drones, bool) and num_drones >= 1):
            raise ValueError(f"num_drones must be an int >= 1, got {num_drones!r}")
        self.cfg = config
        self.num_drones = num_drones
        self._prev: Dict[int, float] = {i: 0.0 for i in range(num_drones)}

    def command(self, i: int, z_target_m: float, z_estimate_m: float, dt_s: float) -> float:
        """The vertical velocity setpoint (m/s, up positive) for drone `i` this control tick."""
        if not (isinstance(dt_s, (int, float)) and math.isfinite(dt_s) and dt_s > 0.0):
            raise ValueError(f"dt_s must be a finite number > 0, got {dt_s!r}")
        cfg = self.cfg
        if math.isfinite(z_estimate_m) and math.isfinite(z_target_m):
            raw = cfg.kp_per_s * (z_target_m - z_estimate_m)
            raw = max(-cfg.max_descent_mps, min(cfg.max_climb_mps, raw))
        else:
            raw = 0.0                      # no usable estimate: stop climbing/descending, do not guess
        prev = self._prev[i]
        step = cfg.slew_mps2 * dt_s
        out = prev + max(-step, min(step, raw - prev))
        self._prev[i] = out
        return out

    def previous(self, i: int) -> float:
        return self._prev[i]

    def reset(self, i: int) -> None:
        self._prev[i] = 0.0
