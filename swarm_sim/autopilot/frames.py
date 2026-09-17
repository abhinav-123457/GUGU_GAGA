"""Phase 6: explicit frame conversion for adapter commands - see
docs/PHASE6_AUTOPILOT_ADAPTERS.md's "Frame conventions" section.

Nothing upstream of this module (SwarmController, SafetySupervisor,
mission.py's own PyBullet handling) ever converts frames - see
contracts.py's module docstring, which reserves frame conversion for
exactly this module. Every conversion here is an explicit function call;
there is no implicit conversion path anywhere in this package.

Axis/sign conventions (the only three frames this project defines -
`swarm_sim.contracts.Frame`):

- **LOCAL_ENU** - this project's own convention for what PyBullet's world
  frame means (see `contracts.PYBULLET_LOCAL_ENU_AXIS_CONVENTION`):
  x = East-like, y = North-like, z = Up. Yaw is measured the standard math
  way: 0 rad points along +x (East-like), increasing counter-clockwise
  toward +y (North-like) - this matches PyBullet's own Euler-angle yaw
  convention, which is what `mission.py` already feeds into
  `VehicleState.attitude_rad`.
- **LOCAL_NED** - MAVLink/ArduCopter's `SET_POSITION_TARGET_LOCAL_NED`
  convention (a real protocol convention, not this project's choice):
  x = North, y = East, z = Down. Yaw is the standard compass heading: 0
  rad points along +x (North), increasing clockwise toward +y (East).
- **BODY** (this project's convention for MAVLink's BODY_FRD): x = Forward,
  y = Right, z = Down, relative to the vehicle's CURRENT heading - not a
  fixed world axis relabeling. Converting to/from BODY requires the
  vehicle's yaw (`attitude_rad[2]`, following this project's existing
  roll/pitch/yaw ordering - see `contracts.VehicleState.attitude_rad` and
  `mission.py`'s `rpys[i]`); pitch/roll are deliberately ignored (a
  documented simplification, not an oversight - see below), so this
  conversion is only exact for a vehicle in level flight. If no attitude
  is available, BODY conversion is refused outright rather than
  approximated with a static axis swap - a static swap silently ignores
  the vehicle's actual heading, which is exactly the "convert BODY using
  only a static axis swap" mistake this phase was told to avoid.

**ENU<->NED is a fixed relabeling** (always valid, no attitude needed):
NED north = ENU y, NED east = ENU x, NED down = -(ENU z). Applying it
twice returns the original vector exactly (it is an involution).

**ENU<->BODY (yaw-only)**: with Forward = (cos(yaw), sin(yaw), 0) and
Right = Forward rotated -90 degrees about the ENU z-axis = (sin(yaw),
-cos(yaw), 0) (verified: Forward x Right = Down = (0, 0, -1), the
right-handed FRD condition), a vector's BODY components are its dot
products with Forward/Right/(0,0,-1); the inverse is the same rotation
run backward. This is a **yaw-only** simplification: it ignores roll and
pitch's effect on what "forward"/"right"/"down" mean, which is the
standard simplification used for horizontal setpoint conversion (the
same one `SwarmController`'s own heading vectors already use elsewhere in
this codebase) and is exact only when the vehicle is level. Documented
here, not hidden.

**NED<->BODY** is not given its own formula - it always pivots through
LOCAL_ENU (NED -> ENU -> BODY or BODY -> ENU -> NED), so there is exactly
one place the yaw-rotation math lives.

Yaw angle conversion (`yaw_rad`/`yaw_rate_radps` fields, when present):
ENU yaw (from East, counter-clockwise) and NED yaw (from North, clockwise)
are related by `yaw_enu = pi/2 - yaw_ned` (and its own inverse - applying
it twice returns the original value modulo the wraparound). `yaw_rate`
flips sign between ENU and NED because their z-axes point opposite ways
(a positive turn rate about "up" is a negative turn rate about "down" for
the same physical rotation). Converting yaw to/from BODY is refused - a
body-relative absolute heading is not a well-defined quantity, and none
of this project's `CommandType`s (see `contracts.py`) actually require
one in BODY frame.
"""
from __future__ import annotations

import dataclasses
import math
from typing import Optional, Tuple

from ..contracts import Command, Frame

_VALID_FRAMES = (Frame.LOCAL_ENU, Frame.LOCAL_NED, Frame.BODY)


def _is_valid_frame(frame) -> bool:
    return isinstance(frame, Frame) and frame in _VALID_FRAMES


def _enu_to_ned(v: Tuple[float, float, float]) -> Tuple[float, float, float]:
    x, y, z = v
    return (y, x, -z)


def _ned_to_enu(v: Tuple[float, float, float]) -> Tuple[float, float, float]:
    x, y, z = v
    return (y, x, -z)  # ENU<->NED is its own inverse (a pure relabeling)


def _enu_to_body(v: Tuple[float, float, float], yaw_rad: float) -> Tuple[float, float, float]:
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    vx, vy, vz = v
    forward = vx * c + vy * s
    right = vx * s - vy * c
    down = -vz
    return (forward, right, down)


def _body_to_enu(v: Tuple[float, float, float], yaw_rad: float) -> Tuple[float, float, float]:
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    forward, right, down = v
    x = forward * c + right * s
    y = forward * s - right * c
    z = -down
    return (x, y, z)


def _yaw_enu_to_ned(yaw_rad: float) -> float:
    return (math.pi / 2.0) - yaw_rad


def _yaw_ned_to_enu(yaw_rad: float) -> float:
    return (math.pi / 2.0) - yaw_rad  # involution, same formula


def convert_vector_frame(vector: Tuple[float, float, float], source_frame: Frame, target_frame: Frame,
                          vehicle_yaw_rad: Optional[float] = None) -> Tuple[float, float, float]:
    """Explicit vector-only conversion (position or velocity), used by
    `convert_command_frame` below and reusable directly by tests. Never
    mutates `vector`; always returns a new tuple. Raises ValueError if
    either frame is not one of `contracts.Frame`'s three members, or if a
    BODY conversion is requested without `vehicle_yaw_rad`."""
    if not _is_valid_frame(source_frame):
        raise ValueError(f"unknown source frame: {source_frame!r}")
    if not _is_valid_frame(target_frame):
        raise ValueError(f"unknown target frame: {target_frame!r}")
    if source_frame == target_frame:
        return tuple(float(c) for c in vector)

    if Frame.BODY in (source_frame, target_frame) and vehicle_yaw_rad is None:
        raise ValueError(
            "converting to/from BODY (FRD) requires the vehicle's current yaw - "
            "refusing to approximate with a static axis swap (see frames.py module docstring)"
        )

    # Pivot through LOCAL_ENU so there is exactly one conversion path.
    if source_frame == Frame.LOCAL_NED:
        enu = _ned_to_enu(vector)
    elif source_frame == Frame.BODY:
        enu = _body_to_enu(vector, vehicle_yaw_rad)
    else:
        enu = tuple(float(c) for c in vector)

    if target_frame == Frame.LOCAL_NED:
        return _enu_to_ned(enu)
    if target_frame == Frame.BODY:
        return _enu_to_body(enu, vehicle_yaw_rad)
    return enu


def convert_command_frame(command: Command, target_frame: Frame,
                           vehicle_attitude_rad: Optional[Tuple[float, float, float]] = None) -> Command:
    """Explicit, tested frame conversion for a validated `contracts.Command`.
    Returns a NEW `Command` in `target_frame` - never mutates `command`,
    never converts anything implicitly. See module docstring for the
    axis/sign conventions and why BODY conversion is refused without
    attitude.

    `vehicle_attitude_rad` is the vehicle's own (roll, pitch, yaw) tuple
    (this project's existing convention - see `contracts.VehicleState`);
    only `[2]` (yaw) is used, per the yaw-only BODY simplification
    documented above.
    """
    if not _is_valid_frame(target_frame):
        raise ValueError(f"unknown target frame: {target_frame!r}")
    if not _is_valid_frame(command.frame):
        raise ValueError(f"unknown source frame on command: {command.frame!r}")

    if command.frame == target_frame:
        return command

    yaw_rad = None if vehicle_attitude_rad is None else float(vehicle_attitude_rad[2])
    involves_body = Frame.BODY in (command.frame, target_frame)
    if involves_body and yaw_rad is None:
        raise ValueError(
            "converting Command to/from BODY (FRD) requires vehicle_attitude_rad "
            "(yaw) - refusing to approximate with a static axis swap"
        )
    if involves_body and (command.yaw_rad is not None or command.yaw_rate_radps is not None):
        raise ValueError(
            "cannot convert yaw_rad/yaw_rate_radps to/from BODY - a body-relative "
            "absolute heading is not well-defined (see frames.py module docstring)"
        )

    new_position = (
        convert_vector_frame(command.desired_position_m, command.frame, target_frame, yaw_rad)
        if command.desired_position_m is not None else None
    )
    new_velocity = (
        convert_vector_frame(command.desired_velocity_mps, command.frame, target_frame, yaw_rad)
        if command.desired_velocity_mps is not None else None
    )

    new_yaw_rad = command.yaw_rad
    new_yaw_rate = command.yaw_rate_radps
    if not involves_body:
        if {command.frame, target_frame} == {Frame.LOCAL_ENU, Frame.LOCAL_NED}:
            if command.frame == Frame.LOCAL_ENU and new_yaw_rad is not None:
                new_yaw_rad = _yaw_enu_to_ned(new_yaw_rad)
            elif command.frame == Frame.LOCAL_NED and new_yaw_rad is not None:
                new_yaw_rad = _yaw_ned_to_enu(new_yaw_rad)
            if new_yaw_rate is not None:
                new_yaw_rate = -new_yaw_rate

    return dataclasses.replace(
        command, frame=target_frame, desired_position_m=new_position, desired_velocity_mps=new_velocity,
        yaw_rad=new_yaw_rad, yaw_rate_radps=new_yaw_rate,
    )
