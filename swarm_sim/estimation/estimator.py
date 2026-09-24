"""Phase 16B: truth-free dead-reckoning state estimator with covariance.
See docs/PHASE16B_ESTIMATION.md for the equations and the verified/inferred
split. This module never sees ground truth: its only inputs are
`OdometryBundle`s (what the odometry sensors reported) and, optionally,
external position fixes; its only output is `EstimatedState`.

Horizontal state (7-state EKF)
    x = [px, py, psi, bx, by, s, wb]
        px, py   position in the mission frame (m)
        psi      heading (rad)
        bx, by   body-frame velocity bias (m/s)
        s        scale-factor error (dimensionless)
        wb       yaw-rate bias (rad/s)
    Per tick, with fused body-frame displacement dp and yaw change dpsi:
        d      = dp - b*dt
        p'     = p + (1 - s) * R(psi) d
        psi'   = psi + dpsi - wb*dt
        b, s, wb   random walks (mean stays 0 until an external fix arrives)
    The bias, scale and yaw-bias states are unobservable from odometry
    alone, so their covariance only grows: position variance grows faster
    than linearly with time, which is the correct signature of dead
    reckoning (and what makes covariance-aware safety worth having).
Vertical state (scalar KF): height from the downward rangefinder, propagated
    with the fused vertical velocity.

The estimator knows the sensors' NOMINAL noise densities (times
`profile.assumed_noise_scale`), never the realised drift, the dropout
schedule, or which ticks were degenerate - so it can be, and under
`stress` / degeneracy is, over-confident. tests/test_phase16b_estimation.py
measures exactly that with NEES.
"""
from __future__ import annotations

import dataclasses
import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .models import EstimatedState, OdometryBundle, OdometryMeasurement
from .profiles import DriftProfile, SensorDriftSpec

_MIN_VAR = 1e-12
# Odometry dropout: extrapolate the last velocity and let its uncertainty grow
# at this (illustrative) unmodelled-acceleration rate, so a long gap costs
# quadratically growing position variance.
_GAP_ACCEL_STD_MPS2 = 2.0
_GAP_YAW_RATE_STD_RADPS = 0.02
_Z_PROCESS_VAR_PER_S = 1e-6

_PX, _PY, _PSI, _BX, _BY, _SCALE, _WB = range(7)


@dataclasses.dataclass(frozen=True)
class FixResult:
    accepted: bool
    nis: float          # normalised innovation squared (2 dof); large = the fix disagrees with the belief


def _var(sigma: float) -> float:
    return max(sigma * sigma, _MIN_VAR)


class _Fused:
    """Inverse-variance fusion of one tick's measurements, plus the covariance
    terms the fused 'virtual sensor' implies. Translation (displacement,
    velocity, scale and bias random walks) is fused over the sources whose
    translational odometry is valid; heading (yaw change, yaw-bias random
    walk) over every source that still reports its gyro channel, dropped or
    not. Weights depend only on nominal white-noise variances, so per-sensor
    random-walk variances combine as sum(w^2 q)."""
    __slots__ = ("has_translation", "has_heading", "dp", "v", "vz", "var_v", "q_bias", "q_scale",
                 "dpsi", "var_yaw_noise", "q_wb")

    def __init__(self, measurements: Sequence[OdometryMeasurement], specs: Dict[str, SensorDriftSpec], a: float):
        trans = [(m, specs[m.sensor_id]) for m in measurements if m.valid]
        head = [(m, specs[m.sensor_id]) for m in measurements if m.dyaw_rad is not None]
        self.has_translation, self.has_heading = bool(trans), bool(head)
        if trans:
            w = _weights([_var(a * s.vel_noise_std_mps) for _, s in trans])
            self.dp = sum(wi * np.asarray(m.dp_body_xy_m, dtype=float) for wi, (m, _) in zip(w, trans))
            self.v = sum(wi * np.asarray(m.v_body_xy_mps, dtype=float) for wi, (m, _) in zip(w, trans))
            self.vz = float(sum(wi * m.vz_mps for wi, (m, _) in zip(w, trans)))
            self.var_v = float(sum(wi ** 2 * _var(a * s.vel_noise_std_mps) for wi, (_, s) in zip(w, trans)))
            self.q_bias = float(sum(wi ** 2 * (a * s.vel_bias_rw_mps_per_sqrt_s) ** 2 for wi, (_, s) in zip(w, trans)))
            self.q_scale = float(sum(wi ** 2 * (a * s.scale_error_rw_per_sqrt_s) ** 2 for wi, (_, s) in zip(w, trans)))
        if head:
            w = _weights([_var(a * s.yaw_noise_std_radps) for _, s in head])
            self.dpsi = float(sum(wi * m.dyaw_rad for wi, (m, _) in zip(w, head)))
            self.var_yaw_noise = float(sum(wi ** 2 * (a * s.yaw_noise_std_radps) ** 2 for wi, (_, s) in zip(w, head)))
            self.q_wb = float(sum(wi ** 2 * (a * s.yaw_bias_rw_radps_per_sqrt_s) ** 2 for wi, (_, s) in zip(w, head)))


def _weights(variances) -> np.ndarray:
    inv = 1.0 / np.asarray(variances, dtype=float)
    return inv / inv.sum()


class StateEstimator:
    def __init__(self, vehicle_id: str, profile: DriftProfile, initial_position_m: Sequence[float],
                 initial_yaw_rad: float = 0.0, t0: float = 0.0):
        self.vehicle_id = vehicle_id
        self.profile = profile
        self._a = profile.assumed_noise_scale
        self._specs: Dict[str, SensorDriftSpec] = {s.sensor_id: s for s in profile.sensors}

        self._x = np.zeros(7)
        self._x[_PX], self._x[_PY] = float(initial_position_m[0]), float(initial_position_m[1])
        self._x[_PSI] = float(initial_yaw_rad)

        # Initial scale-error variance: what the fused sensor set implies at t0 (all sensors assumed up).
        w = np.array([1.0 / _var(self._a * s.vel_noise_std_mps) for s in profile.sensors])
        w = w / w.sum()
        scale_var = float(sum(wi ** 2 * (self._a * s.init_scale_error_std) ** 2 for wi, s in zip(w, profile.sensors)))

        self._P = np.zeros((7, 7))
        self._P[_PX, _PX] = self._P[_PY, _PY] = _var(self._a * profile.init_pos_sigma_m)
        self._P[_PSI, _PSI] = _var(self._a * profile.init_yaw_sigma_rad)
        self._P[_SCALE, _SCALE] = scale_var

        self._z = float(initial_position_m[2]) if len(initial_position_m) > 2 else 0.0
        self._pz = max((self._a * profile.rangefinder_sigma_m) ** 2, 1e-12)

        self._t = float(t0)
        self._last_odometry_t: Optional[float] = float(t0)
        self._last_aiding_t: Optional[float] = None
        self._gap_s = 0.0
        self._last_v_body = np.zeros(2)
        self._last_vz = 0.0
        self._roll_pitch = (0.0, 0.0)

    # -- accessors for tests / diagnostics (never used by autonomy code) ----------

    @property
    def covariance(self) -> np.ndarray:
        return self._P.copy()

    @property
    def state_vector(self) -> np.ndarray:
        return self._x.copy()

    # -- one control tick ----------------------------------------------------------

    def step(self, bundle: OdometryBundle) -> EstimatedState:
        dt = bundle.dt
        if not (math.isfinite(dt) and dt > 0.0):
            raise ValueError(f"bundle.dt must be a finite number > 0, got {dt!r}")
        self._t = bundle.t
        self._roll_pitch = (float(bundle.roll_pitch_rad[0]), float(bundle.roll_pitch_rad[1]))

        for m in bundle.measurements:
            if m.sensor_id not in self._specs:
                raise ValueError(f"unknown sensor id {m.sensor_id!r} for profile {self.profile.name!r}")
        fused = _Fused(bundle.measurements, self._specs, self._a)

        if fused.has_translation:
            self._gap_s = 0.0
            self._last_odometry_t = bundle.t
            self._last_v_body = np.asarray(fused.v, dtype=float)
            self._last_vz = fused.vz
            dp, vz, var_v = fused.dp, fused.vz, fused.var_v
            q_bias, q_scale = fused.q_bias, fused.q_scale
        else:
            # Translational odometry lost: extrapolate the last velocity and let its uncertainty grow.
            self._gap_s += dt
            dp, vz = self._last_v_body * dt, self._last_vz
            var_v = (_GAP_ACCEL_STD_MPS2 * self._gap_s) ** 2
            q_bias = q_scale = 0.0
        if fused.has_heading:
            dpsi, var_yaw, q_wb = fused.dpsi, fused.var_yaw_noise, fused.q_wb
        else:
            dpsi, var_yaw, q_wb = 0.0, _GAP_YAW_RATE_STD_RADPS ** 2, 0.0

        self._propagate_xy(dp, dpsi, var_v, var_yaw, q_bias, q_scale, q_wb, dt)
        self._propagate_z(vz, var_v, dt)

        self._update_z(float(bundle.range_z_m))
        return self.state()

    # -- EKF pieces ----------------------------------------------------------------

    def _propagate_xy(self, dp_body, dpsi, var_v, var_yaw_noise, q_bias, q_scale, q_wb, dt: float) -> None:
        x, P = self._x, self._P
        psi, b, s, wb = x[_PSI], x[_BX:_BY + 1], x[_SCALE], x[_WB]
        c, sn = math.cos(psi), math.sin(psi)
        R = np.array([[c, -sn], [sn, c]])
        dR = np.array([[-sn, -c], [c, -sn]])
        d = np.asarray(dp_body, dtype=float) - b * dt
        m = 1.0 - s
        u = R @ d

        F = np.eye(7)
        F[_PX:_PY + 1, _PSI] = m * (dR @ d)
        F[_PX:_PY + 1, _BX:_BY + 1] = -m * R * dt
        F[_PX:_PY + 1, _SCALE] = -u
        F[_PSI, _WB] = -dt

        x[_PX:_PY + 1] += m * u
        x[_PSI] += dpsi - wb * dt

        # The hidden bias / scale / yaw-bias random walks step BEFORE this tick's measurement is
        # formed (that is how the plant generates it), so their process noise enters ahead of F;
        # the white measurement noise enters after it.
        Q_walk = np.zeros((7, 7))
        Q_walk[_BX, _BX] = Q_walk[_BY, _BY] = q_bias * dt
        Q_walk[_SCALE, _SCALE] = q_scale * dt
        Q_walk[_WB, _WB] = q_wb * dt
        Q_white = np.zeros((7, 7))
        Q_white[_PX, _PX] = Q_white[_PY, _PY] = var_v * dt * dt * m * m
        Q_white[_PSI, _PSI] = var_yaw_noise * dt * dt

        P = F @ (P + Q_walk) @ F.T + Q_white
        self._P = 0.5 * (P + P.T)

    def _propagate_z(self, vz: float, var_v: float, dt: float) -> None:
        self._z += vz * dt
        self._pz += var_v * dt * dt + _Z_PROCESS_VAR_PER_S * dt

    def _update_z(self, range_z_m: float) -> None:
        # The floor is far below _MIN_VAR so a noise-free rangefinder (the `ideal` profile) is trusted
        # essentially completely rather than blended 50/50 with the propagated height.
        r = max((self._a * self.profile.rangefinder_sigma_m) ** 2, 1e-18)
        k = self._pz / (self._pz + r)
        self._z += k * (range_z_m - self._z)
        self._pz = (1.0 - k) * self._pz

    # -- external aiding (unit-tested in 16B, wired into the mission in 16F) --------

    def apply_position_fix(self, xy_m: Sequence[float], noise, t: float, source: str = "fix",
                           gate_nis: Optional[float] = None) -> FixResult:
        """Fuse an absolute horizontal position measurement in the mission
        frame (e.g. seeing the launch-zone fiducial). `noise` is a 1-sigma
        in metres (scalar) or a 2x2 covariance; it is used as given (the
        estimator's `assumed_noise_scale` applies to odometry only).
        `gate_nis`, if given, rejects a fix whose normalised innovation
        squared exceeds it. Cross-covariance means an accepted fix also
        corrects heading, bias and scale to the extent they are correlated
        with position."""
        z = np.asarray(xy_m, dtype=float)
        if z.shape != (2,) or not np.all(np.isfinite(z)):
            raise ValueError(f"xy_m must be two finite numbers, got {xy_m!r}")
        if np.ndim(noise) == 0:
            if not (math.isfinite(noise) and noise >= 0.0):
                raise ValueError(f"noise sigma must be finite and >= 0, got {noise!r}")
            Rm = np.eye(2) * max(float(noise) ** 2, _MIN_VAR)
        else:
            Rm = np.asarray(noise, dtype=float)
            if Rm.shape != (2, 2) or not np.all(np.isfinite(Rm)):
                raise ValueError("noise covariance must be a finite 2x2 matrix")

        x, P = self._x, self._P
        innovation = z - x[_PX:_PY + 1]
        S = P[_PX:_PY + 1, _PX:_PY + 1] + Rm
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return FixResult(False, math.inf)
        nis = float(innovation @ S_inv @ innovation)
        if gate_nis is not None and nis > gate_nis:
            return FixResult(False, nis)

        K = P[:, _PX:_PY + 1] @ S_inv
        H = np.zeros((2, 7))
        H[0, _PX] = H[1, _PY] = 1.0
        self._x = x + K @ innovation
        A = np.eye(7) - K @ H
        P = A @ P @ A.T + K @ Rm @ K.T          # Joseph form: stays symmetric positive semi-definite
        self._P = 0.5 * (P + P.T)
        self._last_aiding_t = float(t)
        return FixResult(True, nis)

    # -- output --------------------------------------------------------------------

    def state(self) -> EstimatedState:
        x, P = self._x, self._P
        sxx, sxy, syy = float(P[_PX, _PX]), float(P[_PX, _PY]), float(P[_PY, _PY])
        half_trace, half_diff = 0.5 * (sxx + syy), 0.5 * (sxx - syy)
        radius = math.sqrt(half_diff * half_diff + sxy * sxy)
        lam_max, lam_min = half_trace + radius, half_trace - radius
        sigma_h = math.sqrt(max(lam_max, 0.0))

        c, sn = math.cos(x[_PSI]), math.sin(x[_PSI])
        v_body = self._last_v_body - x[_BX:_BY + 1]   # during an odometry gap this is the last good velocity
        v_xy = (1.0 - x[_SCALE]) * (np.array([[c, -sn], [sn, c]]) @ v_body)

        finite = bool(np.all(np.isfinite(x)) and np.all(np.isfinite(P)) and math.isfinite(self._z)
                      and math.isfinite(self._pz))
        psd = lam_min >= -1e-9 * max(1.0, lam_max) and self._pz >= 0.0 and P[_PSI, _PSI] >= 0.0
        since_odometry = (self._t - self._last_odometry_t) if self._last_odometry_t is not None else math.inf
        prof = self.profile

        reason = ""
        if not finite:
            reason = "non-finite state or covariance"
        elif not psd:
            reason = "covariance not positive semi-definite"
        elif sigma_h > prof.sigma_invalid_m:
            reason = f"position uncertainty {sigma_h:.2f} m exceeds invalid threshold {prof.sigma_invalid_m:.2f} m"
        elif since_odometry > prof.odometry_timeout_s:
            reason = f"no valid odometry for {since_odometry:.2f} s (timeout {prof.odometry_timeout_s:.2f} s)"
        valid = reason == ""
        degraded = valid and sigma_h >= prof.sigma_degraded_m
        if degraded:
            reason = f"position uncertainty {sigma_h:.2f} m at or above degraded threshold {prof.sigma_degraded_m:.2f} m"

        return EstimatedState(
            t=self._t,
            position_m=(float(x[_PX]), float(x[_PY]), float(self._z)),
            velocity_mps=(float(v_xy[0]), float(v_xy[1]), float(self._last_vz)),
            attitude_rad=(self._roll_pitch[0], self._roll_pitch[1], float(x[_PSI])),
            pos_cov_xy_m2=(sxx, sxy, syy),
            sigma_z_m=math.sqrt(max(self._pz, 0.0)),
            sigma_yaw_rad=math.sqrt(max(float(P[_PSI, _PSI]), 0.0)),
            valid=valid, degraded=degraded, reason=reason,
            last_odometry_time_s=self._last_odometry_t, last_aiding_time_s=self._last_aiding_t,
        )
