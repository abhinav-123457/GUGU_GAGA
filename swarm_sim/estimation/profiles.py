"""Phase 16B: GNSS-denied odometry drift profiles. See docs/PHASE16B_ESTIMATION.md.

EVERY NUMBER IN THIS FILE IS ILLUSTRATIVE, NOT HARDWARE-VERIFIED. They are
chosen to give plausible orders of magnitude for consumer-grade
visual-inertial / optical-flow / LiDAR odometry, not measured from any
specific sensor. Measured on this simulator (90 s at 2.5 m/s, 225 m path,
24 Hz, 12 seeds): mean final position error about 0.8 % of path for vio /
lidar / fused, 2.6 % for flow_rf, 4.2 % for stress. Running the swarm
against these profiles shows how the swarm copes with *an assumed* error
model; it does not validate real sensor accuracy. Real numbers need bench /
hardware-in-the-loop / tethered data.

The profile is used twice, on opposite sides of the truth boundary:
  * plant side (`plant_odometry.OdometrySuite`) - generates the errors:
    hidden per-sensor scale-factor error, velocity bias and yaw-rate bias
    (all random walks), white noise, dropouts and degenerate ticks;
  * estimator side (`estimator.StateEstimator`) - knows only the sensors'
    nominal noise densities (what a datasheet would say), scaled by
    `assumed_noise_scale`. It does NOT know the realised drift, the
    dropout schedule, or which ticks were degenerate.
`assumed_noise_scale` < 1 therefore models an over-confident estimator
(its covariance is too small for the error it actually makes) and > 1 a
conservative one.
"""
from __future__ import annotations

import dataclasses
import math
from typing import Dict, Tuple


def _finite_nonneg(name: str, value) -> None:
    if not (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value) and value >= 0.0):
        raise ValueError(f"{name} must be a finite number >= 0, got {value!r}")


@dataclasses.dataclass(frozen=True)
class SensorDriftSpec:
    """Error model of ONE odometry source. All noise/random-walk terms are
    1-sigma. Displacement error over a tick of length dt is modelled as
    (1 + scale) * true + bias * dt + white_noise * dt."""
    sensor_id: str
    vel_noise_std_mps: float             # white noise on each velocity sample (m/s)
    init_scale_error_std: float          # 1-sigma of the initial multiplicative scale error
    scale_error_rw_per_sqrt_s: float     # scale-factor random walk (1/sqrt(s))
    vel_bias_rw_mps_per_sqrt_s: float    # body-frame velocity-bias random walk (m/s per sqrt(s))
    yaw_noise_std_radps: float           # white noise on each yaw-rate sample (rad/s)
    yaw_bias_rw_radps_per_sqrt_s: float  # yaw-rate-bias random walk (rad/s per sqrt(s))
    dropout_prob: float                  # per-tick probability of losing the translational measurement (gyro heading survives)
    degeneracy_prob: float               # per-tick probability of a degenerate tick (low texture / featureless)
    degeneracy_drift_multiplier: float   # noise and drift-step multiplier during a degenerate tick (>= 1)
    # White noise on the reported VERTICAL speed (m/s), deliberately separate from the horizontal
    # `vel_noise_std_mps`. In a real flight stack the vertical rate is a heavily filtered IMU /
    # barometer / rangefinder estimate, not the raw visual-odometry sample. The mission is
    # sensitive to it: the supervisor's accel limiter returns a vertical command proportional to the
    # drone's OWN vertical speed, and the mission replaces the altitude setpoint by z + vz_cmd*dt
    # whenever that command is non-zero (~97 % of ticks, in the legacy truth pipeline too), so the
    # altitude is damped, not held, and noise on the vertical-speed reading is integrated into an
    # altitude random walk. Reusing the horizontal 0.05-0.15 m/s density here made drones sink,
    # overshoot the 8 m ceiling and crash (Phase 16B finding 7). Default 0.0 keeps positional
    # construction of the older 10-field spec valid.
    vz_noise_std_mps: float = 0.0

    def __post_init__(self):
        if not (isinstance(self.sensor_id, str) and self.sensor_id):
            raise ValueError("sensor_id must be a non-empty string")
        for name in ("vel_noise_std_mps", "init_scale_error_std", "scale_error_rw_per_sqrt_s",
                     "vel_bias_rw_mps_per_sqrt_s", "yaw_noise_std_radps", "yaw_bias_rw_radps_per_sqrt_s",
                     "vz_noise_std_mps"):
            _finite_nonneg(name, getattr(self, name))
        for name in ("dropout_prob", "degeneracy_prob"):
            _finite_nonneg(name, getattr(self, name))
            if getattr(self, name) > 1.0:
                raise ValueError(f"{name} must be in [0, 1], got {getattr(self, name)!r}")
        _finite_nonneg("degeneracy_drift_multiplier", self.degeneracy_drift_multiplier)
        if self.degeneracy_drift_multiplier < 1.0:
            raise ValueError("degeneracy_drift_multiplier must be >= 1")


@dataclasses.dataclass(frozen=True)
class DriftProfile:
    name: str
    sensors: Tuple[SensorDriftSpec, ...]
    rangefinder_sigma_m: float           # downward rangefinder noise (height above ground)
    attitude_sigma_rad: float            # IMU roll/pitch estimate noise
    init_pos_sigma_m: float              # how well a drone knows its launch-slot position in the mission frame
    init_yaw_sigma_rad: float            # ... and its initial heading
    assumed_noise_scale: float = 1.0     # estimator's Q/R multiplier; < 1 = over-confident
    sigma_degraded_m: float = 1.0        # worst-axis 1-sigma position uncertainty that flags DEGRADED
    sigma_invalid_m: float = 5.0         # ... and that invalidates the estimate
    odometry_timeout_s: float = 1.0      # no valid odometry for this long invalidates the estimate

    def __post_init__(self):
        if not (isinstance(self.name, str) and self.name):
            raise ValueError("name must be a non-empty string")
        if not (isinstance(self.sensors, tuple) and self.sensors
                and all(isinstance(s, SensorDriftSpec) for s in self.sensors)):
            raise ValueError("sensors must be a non-empty tuple of SensorDriftSpec")
        ids = [s.sensor_id for s in self.sensors]
        if len(set(ids)) != len(ids):
            raise ValueError(f"duplicate sensor ids in profile: {ids}")
        for name in ("rangefinder_sigma_m", "attitude_sigma_rad", "init_pos_sigma_m", "init_yaw_sigma_rad",
                     "assumed_noise_scale", "sigma_degraded_m", "sigma_invalid_m", "odometry_timeout_s"):
            _finite_nonneg(name, getattr(self, name))
        if not self.assumed_noise_scale > 0.0:
            raise ValueError("assumed_noise_scale must be > 0")
        if not self.sigma_invalid_m > self.sigma_degraded_m:
            raise ValueError("sigma_invalid_m must exceed sigma_degraded_m")
        if not self.odometry_timeout_s > 0.0:
            raise ValueError("odometry_timeout_s must be > 0")

    def with_assumed_noise_scale(self, scale: float) -> "DriftProfile":
        return dataclasses.replace(self, assumed_noise_scale=scale)

    def with_init_pos_sigma(self, sigma_m: float) -> "DriftProfile":
        return dataclasses.replace(self, init_pos_sigma_m=sigma_m)

    def to_dict(self) -> dict:
        return dataclasses.asdict(self)


# --- illustrative sensor specs -------------------------------------------------

_IDEAL = SensorDriftSpec("ideal", 0, 0, 0, 0, 0, 0, 0, 0, 1.0)

_VIO = SensorDriftSpec(
    "vio", vel_noise_std_mps=0.05, init_scale_error_std=0.005, scale_error_rw_per_sqrt_s=0.001,
    vel_bias_rw_mps_per_sqrt_s=0.001, yaw_noise_std_radps=0.005, yaw_bias_rw_radps_per_sqrt_s=1.5e-5,
    dropout_prob=0.01, degeneracy_prob=0.02, degeneracy_drift_multiplier=3.0, vz_noise_std_mps=0.02)

_FLOW_RF = SensorDriftSpec(
    "flow_rf", vel_noise_std_mps=0.10, init_scale_error_std=0.02, scale_error_rw_per_sqrt_s=0.003,
    vel_bias_rw_mps_per_sqrt_s=0.003, yaw_noise_std_radps=0.01, yaw_bias_rw_radps_per_sqrt_s=3.0e-5,
    dropout_prob=0.02, degeneracy_prob=0.05, degeneracy_drift_multiplier=3.0, vz_noise_std_mps=0.03)

_LIDAR = SensorDriftSpec(
    "lidar", vel_noise_std_mps=0.03, init_scale_error_std=0.002, scale_error_rw_per_sqrt_s=0.0005,
    vel_bias_rw_mps_per_sqrt_s=0.0005, yaw_noise_std_radps=0.003, yaw_bias_rw_radps_per_sqrt_s=1.0e-5,
    dropout_prob=0.005, degeneracy_prob=0.03, degeneracy_drift_multiplier=5.0, vz_noise_std_mps=0.02)

_STRESS = SensorDriftSpec(
    "stress", vel_noise_std_mps=0.15, init_scale_error_std=0.03, scale_error_rw_per_sqrt_s=0.004,
    vel_bias_rw_mps_per_sqrt_s=0.006, yaw_noise_std_radps=0.02, yaw_bias_rw_radps_per_sqrt_s=6.0e-5,
    dropout_prob=0.10, degeneracy_prob=0.10, degeneracy_drift_multiplier=4.0, vz_noise_std_mps=0.05)


def _profile(name, sensors, rangefinder_sigma_m=0.02, attitude_sigma_rad=0.005, init_pos_sigma_m=0.05,
             init_yaw_sigma_rad=0.01, **kwargs) -> DriftProfile:
    return DriftProfile(name=name, sensors=tuple(sensors), rangefinder_sigma_m=rangefinder_sigma_m,
                        attitude_sigma_rad=attitude_sigma_rad, init_pos_sigma_m=init_pos_sigma_m,
                        init_yaw_sigma_rad=init_yaw_sigma_rad, **kwargs)


PROFILES: Dict[str, DriftProfile] = {
    # Perfect odometry, perfect initial knowledge: the estimated state equals truth (to float
    # rounding). The reference that the drifting profiles are compared against.
    "ideal": _profile("ideal", [_IDEAL], rangefinder_sigma_m=0.0, attitude_sigma_rad=0.0,
                      init_pos_sigma_m=0.0, init_yaw_sigma_rad=0.0),
    "vio": _profile("vio", [_VIO]),
    "flow_rf": _profile("flow_rf", [_FLOW_RF], rangefinder_sigma_m=0.03),
    "lidar": _profile("lidar", [_LIDAR]),
    # All three sources fused by inverse variance - independent errors partly cancel.
    "fused": _profile("fused", [_VIO, _FLOW_RF, _LIDAR]),
    # A deliberately poor single source, for failure-mode testing (validity gate, DEGRADED, SAFE_HOLD).
    "stress": _profile("stress", [_STRESS], rangefinder_sigma_m=0.05, attitude_sigma_rad=0.01,
                       init_pos_sigma_m=0.2, init_yaw_sigma_rad=0.03),
}


def get_profile(name: str) -> DriftProfile:
    try:
        return PROFILES[name]
    except KeyError:
        raise ValueError(f"unknown estimator profile {name!r}; known: {sorted(PROFILES)}") from None
