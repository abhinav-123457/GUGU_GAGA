"""Phase 16B: EstimatedState -> the existing contract types the autonomy
stack already consumes. Truth-free. See docs/PHASE16B_ESTIMATION.md.

Health mapping (deliberate): the estimator alone never reports
`HealthState.FAILED`. FAILED makes SafetySupervisor abort the vehicle
(priority 1); a lost or untrustworthy estimate must instead reach the
supervisor's own, gentler estimator-invalid path (SAFE_HOLD), which is what
`VehicleState.estimator_valid=False` triggers. So: valid and not degraded
-> OK; anything else -> DEGRADED, with `estimator_valid` carrying validity.
"""
from __future__ import annotations

import math
from typing import Optional

from ..contracts import Frame, HealthState, VehicleState
from .models import EstimatedState


def pose_uncertainty_m(est: EstimatedState) -> float:
    """1-sigma horizontal position uncertainty along the worst axis:
    sqrt of the larger eigenvalue of the position covariance. Conservative
    (never below the true 1-sigma on any axis)."""
    sxx, sxy, syy = est.pos_cov_xy_m2
    half_trace, half_diff = 0.5 * (sxx + syy), 0.5 * (sxx - syy)
    lam_max = half_trace + math.sqrt(half_diff * half_diff + sxy * sxy)
    return math.sqrt(max(lam_max, 0.0))


def health_state_of(est: EstimatedState) -> HealthState:
    return HealthState.OK if (est.valid and not est.degraded) else HealthState.DEGRADED


def to_vehicle_state(est: EstimatedState, vehicle_id: str, sim_time_s: float, battery_fraction: float = 1.0,
                     last_valid_command_time_s: Optional[float] = None) -> VehicleState:
    """The drone's own state as the rest of the stack sees it: position,
    velocity and attitude all come from the estimate. Acceleration and
    angular velocity are not estimated in 16B and stay zero, as in the
    perfect-state path."""
    return VehicleState(
        vehicle_id=vehicle_id, sim_time_s=sim_time_s, frame=Frame.LOCAL_ENU,
        position_m=est.position_m, velocity_mps=est.velocity_mps,
        acceleration_mps2=(0.0, 0.0, 0.0), attitude_rad=est.attitude_rad,
        angular_velocity_radps=(0.0, 0.0, 0.0), battery_fraction=battery_fraction,
        health_state=health_state_of(est), estimator_valid=est.valid,
        last_valid_command_time_s=last_valid_command_time_s if last_valid_command_time_s is not None else sim_time_s,
    )
