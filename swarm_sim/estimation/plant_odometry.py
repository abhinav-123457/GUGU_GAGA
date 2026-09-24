"""Phase 16B: PLANT-SIDE odometry error generator. See docs/PHASE16B_ESTIMATION.md.

This is one of only two modules in `swarm_sim.estimation` allowed to touch
ground truth (the other is `metrics`) - it is the simulated hardware: it
looks at what the drone REALLY did and produces what a VIO / optical-flow /
LiDAR-odometry stack would REPORT, corrupted by hidden, slowly-varying
errors. The autonomy stack sees only the resulting `OdometryBundle`s,
through `StateEstimator`, never the truth and never these hidden states.

Hidden per drone and sensor: a scale-factor error, a body-frame velocity
bias and a yaw-rate bias (each a random walk), plus per-tick white noise,
dropouts and degenerate ticks (noise and drift steps multiplied). A dropout
loses the TRANSLATIONAL measurement only; the heading-rate (gyro) channel is
still reported. The reported displacement over a tick is

    dp_meas = (1 + scale) * dp_true_body + bias * dt + white_noise * dt

with `dp_true_body` the exact displacement rotated into the previous-tick
body frame, so the `ideal` profile reproduces truth to floating-point
rounding. Every random draw comes from a per-(sensor, drone) stream named
`odometry/<sensor>/drone<i>` on the mission's SeedManager, and a fixed
number of draws is made per tick regardless of dropouts, so enabling
estimated mode never perturbs any other subsystem's random stream.
"""
from __future__ import annotations

import math
from typing import Dict, Optional, Tuple

import numpy as np

from ..plant_truth import PlantTruth
from ..seeding import SeedManager
from .models import OdometryBundle, OdometryMeasurement
from .profiles import DriftProfile, SensorDriftSpec

_UNIFORM_DRAWS = 2     # degenerate?, dropped?
_NORMAL_DRAWS = 10     # scale step, bias step (2), yaw-bias step, dp noise (2), v noise (2), vz noise, yaw noise


def _wrap_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _to_body(yaw: float, world_xy: np.ndarray) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([c * world_xy[0] + s * world_xy[1], -s * world_xy[0] + c * world_xy[1]])


class _Hidden:
    __slots__ = ("scale", "bias", "yaw_bias")

    def __init__(self, scale: float):
        self.scale = scale
        self.bias = np.zeros(2)
        self.yaw_bias = 0.0


class OdometrySuite:
    def __init__(self, profile: DriftProfile, num_drones: int, seed_manager: SeedManager):
        self.profile = profile
        self.n = num_drones
        self._sensor_rng: Dict[Tuple[int, str], np.random.Generator] = {}
        self._hidden: Dict[Tuple[int, str], _Hidden] = {}
        for i in range(num_drones):
            for spec in profile.sensors:
                rng = seed_manager.rng(f"odometry/{spec.sensor_id}/drone{i}")
                self._sensor_rng[(i, spec.sensor_id)] = rng
                self._hidden[(i, spec.sensor_id)] = _Hidden(float(rng.normal(scale=spec.init_scale_error_std)))
        self._imu_rng = [seed_manager.rng(f"odometry/imu_range/drone{i}") for i in range(num_drones)]
        self._init_rng = [seed_manager.rng(f"odometry/init/drone{i}") for i in range(num_drones)]
        self._prev: Dict[int, Tuple[np.ndarray, float]] = {}

    def initial_offsets(self, i: int) -> Tuple[float, float, float]:
        """(dx, dy, dyaw): how wrong drone `i`'s own initial pose in the
        mission frame is (launch-slot placement / heading accuracy). Draw once per drone."""
        rng = self._init_rng[i]
        dxy = rng.normal(scale=self.profile.init_pos_sigma_m, size=2)
        dyaw = float(rng.normal(scale=self.profile.init_yaw_sigma_rad))
        return float(dxy[0]), float(dxy[1]), dyaw

    def start(self, i: int, position_xyz, yaw_rad: float) -> None:
        """Record the true spawn pose so the first `measure` reports the real
        displacement since spawn; without it the first tick reports zero motion."""
        self._prev[i] = (np.asarray(position_xyz, dtype=float)[:2].copy(), float(yaw_rad))

    def hidden_state(self, i: int, sensor_id: str) -> Tuple[float, Tuple[float, float], float]:
        """(scale error, velocity bias, yaw-rate bias) - plant-side introspection for tests/metrics only."""
        h = self._hidden[(i, sensor_id)]
        return h.scale, (float(h.bias[0]), float(h.bias[1])), h.yaw_bias

    def measure(self, i: int, truth: PlantTruth, dt: float) -> OdometryBundle:
        p = np.asarray(truth.positions[i], dtype=float)
        v = np.asarray(truth.velocities[i], dtype=float)
        rpy = np.asarray(truth.rpys[i], dtype=float)
        yaw = float(rpy[2])

        prev = self._prev.get(i)
        if prev is None:
            dp_true_body, dpsi_true = np.zeros(2), 0.0
        else:
            prev_xy, prev_yaw = prev
            dp_true_body = _to_body(prev_yaw, p[:2] - prev_xy)
            dpsi_true = _wrap_angle(yaw - prev_yaw)
        self._prev[i] = (p[:2].copy(), yaw)
        v_body = _to_body(yaw, v[:2])
        sqrt_dt = math.sqrt(dt)

        measurements = []
        for spec in self.profile.sensors:
            measurements.append(self._measure_one(i, spec, dp_true_body, dpsi_true, v_body, float(v[2]), dt, sqrt_dt))

        imu = self._imu_rng[i].standard_normal(3)
        prof = self.profile
        return OdometryBundle(
            t=float(truth.t), dt=float(dt), measurements=tuple(measurements),
            range_z_m=float(p[2] + prof.rangefinder_sigma_m * imu[0]),
            roll_pitch_rad=(float(rpy[0] + prof.attitude_sigma_rad * imu[1]),
                            float(rpy[1] + prof.attitude_sigma_rad * imu[2])),
        )

    def _measure_one(self, i: int, spec: SensorDriftSpec, dp_true_body, dpsi_true, v_body, vz_true,
                     dt: float, sqrt_dt: float) -> OdometryMeasurement:
        rng = self._sensor_rng[(i, spec.sensor_id)]
        u = rng.random(_UNIFORM_DRAWS)
        z = rng.standard_normal(_NORMAL_DRAWS)
        m = spec.degeneracy_drift_multiplier if u[0] < spec.degeneracy_prob else 1.0
        h = self._hidden[(i, spec.sensor_id)]

        h.scale += spec.scale_error_rw_per_sqrt_s * sqrt_dt * m * z[0]
        h.bias = h.bias + spec.vel_bias_rw_mps_per_sqrt_s * sqrt_dt * m * z[1:3]
        h.yaw_bias += spec.yaw_bias_rw_radps_per_sqrt_s * sqrt_dt * m * z[3]

        # The heading rate is the IMU gyro's, so it survives a loss of translational odometry.
        dyaw = dpsi_true + h.yaw_bias * dt + spec.yaw_noise_std_radps * m * dt * z[9]
        if u[1] < spec.dropout_prob:
            return OdometryMeasurement(spec.sensor_id, False, dyaw_rad=float(dyaw))

        sigma_v = spec.vel_noise_std_mps * m
        dp = (1.0 + h.scale) * dp_true_body + h.bias * dt + sigma_v * dt * z[4:6]
        vel = (1.0 + h.scale) * v_body + h.bias + sigma_v * z[6:8]
        vz = vz_true + spec.vz_noise_std_mps * z[8]
        return OdometryMeasurement(
            spec.sensor_id, True, dp_body_xy_m=(float(dp[0]), float(dp[1])),
            v_body_xy_mps=(float(vel[0]), float(vel[1])), vz_mps=float(vz), dyaw_rad=float(dyaw),
        )
