"""Phase 6: adapter-layer types - see docs/PHASE6_AUTOPILOT_ADAPTERS.md.

These are the ONLY types that cross the SafetySupervisor -> AutopilotAdapter
boundary. `AdapterCommand` always wraps an already-validated
`swarm_sim.contracts.Command` - there is no constructor path from a raw
SwarmController candidate (a bare numpy velocity array) to an
`AdapterCommand` without first building a real `Command`, which itself
requires passing through `SafetySupervisor.evaluate()` in the mission
pipeline (see mission.py). This is what "a raw candidate can never reach
an adapter" means structurally, not just by convention.

Ground-truth isolation: nothing in this module reads WorldState, victim
ground truth, obstacle geometry, or the mission's global drone-position
array - it only defines types.
"""
from __future__ import annotations

import dataclasses
import math
from enum import Enum
from typing import Optional

from ..contracts import Command, Frame


# --------------------------------------------------------------------------
# Enums
# --------------------------------------------------------------------------

class ConnectionState(Enum):
    """Adapter <-> vehicle link state. Distinct from AutopilotMode: this is
    about whether the transport/link exists and is trustworthy at all, not
    what the vehicle is doing."""
    DISCONNECTED = "DISCONNECTED"
    CONNECTING = "CONNECTING"
    CONNECTED = "CONNECTED"
    DEGRADED = "DEGRADED"
    FAILSAFE = "FAILSAFE"
    DISARMED = "DISARMED"
    ARMED = "ARMED"
    LANDING = "LANDING"
    UNKNOWN = "UNKNOWN"


class AutopilotMode(Enum):
    """What the vehicle's own flight-mode state machine is doing. HOLD/
    OFFBOARD/GUIDED/RTL/LAND/ABORT/DISARMED name the modes this project's
    adapters can request; OFFBOARD is used as this project's generic
    "actively tracking external setpoints" mode (PX4's own term - an
    ArduPilot transport would map the same request to ArduPilot's GUIDED,
    see docs/PHASE6_AUTOPILOT_ADAPTERS.md)."""
    HOLD = "HOLD"
    OFFBOARD = "OFFBOARD"
    GUIDED = "GUIDED"
    RTL = "RTL"
    LAND = "LAND"
    ABORT = "ABORT"
    DISARMED = "DISARMED"
    UNKNOWN = "UNKNOWN"


# --------------------------------------------------------------------------
# Validation helpers (same pattern as contracts.py)
# --------------------------------------------------------------------------

def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def _validate_finite(name: str, value) -> None:
    _require(isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value),
              f"{name} must be a finite number, got {value!r}")


def _validate_vec3_or_none(name: str, value) -> None:
    if value is None:
        return
    _require(isinstance(value, tuple) and len(value) == 3, f"{name} must be a 3-tuple or None, got {value!r}")
    for i, component in enumerate(value):
        _validate_finite(f"{name}[{i}]", component)


def _validate_nonneg_int(name: str, value) -> None:
    _require(isinstance(value, int) and not isinstance(value, bool) and value >= 0,
              f"{name} must be a non-negative int, got {value!r}")


# --------------------------------------------------------------------------
# VehicleTelemetry
# --------------------------------------------------------------------------

@dataclasses.dataclass(frozen=True)
class VehicleTelemetry:
    vehicle_id: str
    timestamp_s: float
    frame: Frame
    position_m: Optional[tuple]
    velocity_mps: Optional[tuple]
    acceleration_mps2: Optional[tuple]
    attitude_rad: Optional[tuple]
    angular_velocity_radps: Optional[tuple]
    battery_fraction: Optional[float]
    estimator_valid: bool
    connection_state: ConnectionState
    autopilot_mode: AutopilotMode
    armed: bool
    failsafe: bool
    sequence: int

    def __post_init__(self):
        _require(isinstance(self.vehicle_id, str) and self.vehicle_id, "vehicle_id must be a non-empty str")
        _validate_finite("timestamp_s", self.timestamp_s)
        _require(isinstance(self.frame, Frame), "frame must be a contracts.Frame")
        _validate_vec3_or_none("position_m", self.position_m)
        _validate_vec3_or_none("velocity_mps", self.velocity_mps)
        _validate_vec3_or_none("acceleration_mps2", self.acceleration_mps2)
        _validate_vec3_or_none("attitude_rad", self.attitude_rad)
        _validate_vec3_or_none("angular_velocity_radps", self.angular_velocity_radps)
        if self.battery_fraction is not None:
            _require(0.0 <= self.battery_fraction <= 1.0,
                      f"battery_fraction must be in [0, 1] or None, got {self.battery_fraction!r}")
        _require(isinstance(self.estimator_valid, bool), "estimator_valid must be a bool")
        _require(isinstance(self.connection_state, ConnectionState), "connection_state must be a ConnectionState")
        _require(isinstance(self.autopilot_mode, AutopilotMode), "autopilot_mode must be an AutopilotMode")
        _require(isinstance(self.armed, bool), "armed must be a bool")
        _require(isinstance(self.failsafe, bool), "failsafe must be a bool")
        _validate_nonneg_int("sequence", self.sequence)


# --------------------------------------------------------------------------
# AdapterCommand / AdapterResult
# --------------------------------------------------------------------------

@dataclasses.dataclass(frozen=True)
class AdapterCommand:
    """Wraps an already-validated `contracts.Command` with the adapter-
    level monotonic sequence number the interface requires. This is the
    ONLY type `AutopilotAdapter.send_command` accepts - see module
    docstring for why a raw candidate cannot reach it."""
    command: Command
    sequence: int

    def __post_init__(self):
        _require(isinstance(self.command, Command),
                  f"command must be a validated swarm_sim.contracts.Command, got {type(self.command)!r}")
        _validate_nonneg_int("sequence", self.sequence)

    @property
    def vehicle_id(self) -> str:
        return self.command.vehicle_id

    @property
    def frame(self) -> Frame:
        return self.command.frame

    @property
    def timestamp_s(self) -> float:
        return self.command.timestamp_s

    @property
    def expiration_time_s(self) -> float:
        return self.command.expiration_time_s

    @property
    def source(self) -> str:
        return self.command.source


@dataclasses.dataclass(frozen=True)
class AdapterResult:
    accepted: bool
    reason: str
    timestamp_s: float
    command_sequence: Optional[int]
    adapter_state: ConnectionState
    transformed_command: Optional[AdapterCommand] = None

    def __post_init__(self):
        _require(isinstance(self.accepted, bool), "accepted must be a bool")
        _require(isinstance(self.reason, str) and self.reason, "reason must be a non-empty str")
        _validate_finite("timestamp_s", self.timestamp_s)
        if self.command_sequence is not None:
            _validate_nonneg_int("command_sequence", self.command_sequence)
        _require(isinstance(self.adapter_state, ConnectionState), "adapter_state must be a ConnectionState")
        if self.transformed_command is not None:
            _require(isinstance(self.transformed_command, AdapterCommand),
                      "transformed_command must be an AdapterCommand or None")
            _require(self.accepted, "a rejected AdapterResult must not carry transformed_command")
        # `transformed_command` is only meaningful for send_command()
        # results (an accepted command's validated/frame-checked form) -
        # connect()/disconnect()/set_mode()/request_land()/etc. reuse this
        # same result type for a non-command acceptance and leave it None
        # even when accepted=True. AutopilotAdapter.send_command
        # implementations are responsible for always setting it on
        # acceptance - see mock.py.
