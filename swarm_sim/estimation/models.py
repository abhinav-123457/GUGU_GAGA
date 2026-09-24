"""Phase 16B: the data types that cross the odometry -> estimator -> autonomy
boundary. Truth-free: nothing here carries ground truth, so autonomy code
may import this module. See docs/PHASE16B_ESTIMATION.md.

    plant side                         estimator side              autonomy side
    OdometrySuite.measure()  --->  OdometryBundle  --->  StateEstimator  --->  EstimatedState
"""
from __future__ import annotations

import dataclasses
from typing import Optional, Tuple

Vec2 = Tuple[float, float]
Vec3 = Tuple[float, float, float]


@dataclasses.dataclass(frozen=True)
class OdometryMeasurement:
    """What one odometry source reports for one tick, in the drone's BODY
    frame.

    `valid=False` means the TRANSLATIONAL odometry was lost this tick (visual
    tracking loss, a dropped LiDAR scan match): `dp_body_xy_m`, `v_body_xy_mps`
    and `vz_mps` are then None. The heading-rate channel `dyaw_rad` is the
    IMU gyro's, which keeps integrating when tracking is lost, so it is still
    reported (a source that has no gyro reading either leaves it None)."""
    sensor_id: str
    valid: bool
    dp_body_xy_m: Optional[Vec2] = None     # displacement over this tick
    v_body_xy_mps: Optional[Vec2] = None    # instantaneous velocity
    vz_mps: Optional[float] = None
    dyaw_rad: Optional[float] = None        # heading change over this tick


@dataclasses.dataclass(frozen=True)
class OdometryBundle:
    """Everything the estimator gets in one tick."""
    t: float
    dt: float
    measurements: Tuple[OdometryMeasurement, ...]
    range_z_m: float                        # downward rangefinder: height above ground
    roll_pitch_rad: Tuple[float, float]     # IMU attitude estimate


@dataclasses.dataclass(frozen=True)
class EstimatedState:
    """A drone's own belief about its state in the mission frame - the ONLY
    pose the autonomy stack sees in `localization_mode="estimated"`.

    `pos_cov_xy_m2` is the horizontal position covariance (sxx, sxy, syy).
    `valid` / `degraded` are derived from that covariance and odometry
    freshness (see StateEstimator); `reason` says why when either is off.
    """
    t: float
    position_m: Vec3
    velocity_mps: Vec3
    attitude_rad: Vec3                      # roll, pitch, yaw (yaw is the estimated yaw)
    pos_cov_xy_m2: Tuple[float, float, float]
    sigma_z_m: float
    sigma_yaw_rad: float
    valid: bool
    degraded: bool
    reason: str
    last_odometry_time_s: Optional[float]
    last_aiding_time_s: Optional[float]     # last external position fix; None while dead-reckoning
