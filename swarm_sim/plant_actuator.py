"""Phase 16C: the plant-side actuator boundary.

Everything in here is the SIMULATED AUTOPILOT / physics side and may read the true state; nothing the
autonomy stack decides passes through it except the final, safety-filtered velocity setpoint. Keeping
the PID (which is fed true position and true state) behind one explicit object stops estimated mode from
silently benefiting from simulator truth in its last control loop, and is the single place a real
PX4/ArduPilot autopilot would replace.

Modes
-----
"legacy"    the pre-16C behaviour, operation for operation: the position target is [true x, true y, z*],
            where z* is the fixed flight altitude unless the safety layer asked for vertical motion, in
            which case z* is re-anchored to true z + vz*dt.
"velocity"  pure velocity tracking in all three axes: the position target is the current true position,
            so the only thing driving the airframe is the commanded velocity (x, y and z). The velocity
            setpoint is first shaped by the autopilot's own acceleration limit (`max_accel_mps2`, horizontal
            and vertical bounded separately), as every real autopilot does: the mission maps HOLD / LAND /
            ABORT and rejected commands to an instantaneous zero-velocity step, and a 27 g airframe braked
            from ~5 m/s in one tick flips (roll past 90 degrees) and falls (found by the 16C gate).

This module is deliberately independent of `plant_truth`: callers pass plain arrays."""
from __future__ import annotations

from typing import Optional

import numpy as np

from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel

MODES = ("legacy", "velocity")


class SimulatedAutopilot:
    def __init__(self, num_drones: int, mode: str, flight_altitude_m: float, safety_enabled: bool,
                 floor_alt_m: float, ceiling_alt_m: float, max_accel_mps2: Optional[float] = None):
        if mode not in MODES:
            raise ValueError(f"mode must be one of {MODES}, got {mode!r}")
        if max_accel_mps2 is not None and not (max_accel_mps2 > 0.0 and np.isfinite(max_accel_mps2)):
            raise ValueError(f"max_accel_mps2 must be a finite number > 0 or None, got {max_accel_mps2!r}")
        self.mode = mode
        self.max_accel_mps2 = max_accel_mps2         # velocity mode only; None = no shaping
        self._applied_vel = np.zeros((num_drones, 3))
        self.flight_altitude_m = flight_altitude_m
        self.safety_enabled = safety_enabled
        self.floor_alt_m = floor_alt_m
        self.ceiling_alt_m = ceiling_alt_m
        self.pid = [DSLPIDControl(drone_model=DroneModel.CF2X) for _ in range(num_drones)]

    def shaped_velocity(self, i: int, target_vel, dt: float):
        """The velocity setpoint actually handed to the PID: `target_vel` moved from the previously applied
        setpoint by at most `max_accel_mps2 * dt` (horizontal magnitude and vertical component separately)."""
        target = np.asarray(target_vel, dtype=float)
        prev = self._applied_vel[i]
        step = self.max_accel_mps2 * dt
        dh = target[:2] - prev[:2]
        norm = float(np.hypot(dh[0], dh[1]))
        if norm > step:
            dh = dh * (step / norm)
        dz = float(np.clip(target[2] - prev[2], -step, step))
        out = np.array([prev[0] + dh[0], prev[1] + dh[1], prev[2] + dz])
        self._applied_vel[i] = out
        return out

    def rpm(self, i: int, state, position, target_vel, safe_vel_z, dt: float, control_timestep: float):
        """Motor speeds for drone `i`. `state` is the raw 20-vector the physics returned, `position` its
        true (x, y, z), `target_vel` the velocity setpoint in the TRUE frame, `safe_vel_z` the vertical
        component of the safety-filtered command (used only by the legacy altitude re-anchoring)."""
        if self.mode == "legacy":
            target_z = self.flight_altitude_m
            if self.safety_enabled and abs(safe_vel_z) > 1e-9:
                target_z = float(np.clip(position[2] + safe_vel_z * dt, self.floor_alt_m, self.ceiling_alt_m))
            target_pos = np.array([position[0], position[1], target_z])
        else:
            target_pos = np.array([position[0], position[1], position[2]])
            if self.max_accel_mps2 is not None:
                target_vel = self.shaped_velocity(i, target_vel, dt)
        rpm, _, _ = self.pid[i].computeControlFromState(
            control_timestep=control_timestep,
            state=state,
            target_pos=target_pos,
            target_rpy=np.array([0.0, 0.0, 0.0]),
            target_vel=target_vel,
        )
        return rpm
