"""Phase 7: SITL wire-schema types (telemetry, command acknowledgement,
transport-level results) and offline MAVLink status-mapping tables - see
docs/PHASE7_SITL_INTEGRATION.md.

These types model what a real local SITL bridge would eventually carry
over its own transport (a future ArduPilot/PX4 SITL process) - richer than
swarm_sim.autopilot.types.VehicleTelemetry, because namespace, heartbeat
status, and command-acknowledgement status are SITL-transport-specific
concepts, not adapter-interface concepts - but built with the same
frozen-dataclass/__post_init__ validation pattern used throughout this
project. swarm_sim.autopilot.sitl.SITLAdapter is the only place these types
are translated to/from the Phase 6 AutopilotAdapter types
(VehicleTelemetry/AdapterResult) that mission.py and the rest of the swarm
actually consume.

No pymavlink/MAVSDK/serial/socket import anywhere in this module - see
tests/test_sitl_architecture.py. The mapping tables at the bottom are
specifications only (plain dicts naming a real protocol's equivalent
concept) - nothing here parses or emits an actual MAVLink message.
"""
from __future__ import annotations

import dataclasses
import math
from enum import Enum
from typing import Optional, Tuple

from ..autopilot.types import AutopilotMode


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


def _validate_nonempty_str(name: str, value) -> None:
    _require(isinstance(value, str) and len(value) > 0, f"{name} must be a non-empty str, got {value!r}")


class FailureType(Enum):
    """Discrete failures `SITLTransport.inject_failure` can toggle on a
    single vehicle's namespace - continuous channel effects (latency,
    packet-loss probability) are constructor-time configuration on
    FakeSITLTransport instead, since they are rates, not events."""
    COMMAND_PACKET_LOSS = "COMMAND_PACKET_LOSS"
    TELEMETRY_PACKET_LOSS = "TELEMETRY_PACKET_LOSS"
    HEARTBEAT_LOSS = "HEARTBEAT_LOSS"
    ESTIMATOR_INVALID = "ESTIMATOR_INVALID"
    BATTERY_CRITICAL = "BATTERY_CRITICAL"
    VEHICLE_DISCONNECT = "VEHICLE_DISCONNECT"


@dataclasses.dataclass(frozen=True)
class TransportResult:
    """Outcome of a transport-level call (start/stop, or a command's own
    hand-off onto the transport) - distinct from CommandAck, which is the
    vehicle's own acceptance/rejection of a command's content (frame,
    sequence, failsafe state). A TransportResult can be `success=False`
    (e.g. "transport not started") even before a command ever reaches a
    vehicle's own validation logic."""
    success: bool
    reason: str
    timestamp_s: float

    def __post_init__(self):
        _require(isinstance(self.success, bool), "success must be a bool")
        _require(isinstance(self.reason, str) and self.reason, "reason must be a non-empty str")
        _validate_finite("timestamp_s", self.timestamp_s)


@dataclasses.dataclass(frozen=True)
class CommandAck:
    vehicle_id: str
    namespace: str
    command_sequence: Optional[int]
    accepted: bool
    reason: str
    timestamp_s: float
    autopilot_mode: AutopilotMode
    failsafe: bool

    def __post_init__(self):
        _validate_nonempty_str("vehicle_id", self.vehicle_id)
        _validate_nonempty_str("namespace", self.namespace)
        if self.command_sequence is not None:
            _validate_nonneg_int("command_sequence", self.command_sequence)
        _require(isinstance(self.accepted, bool), "accepted must be a bool")
        _require(isinstance(self.reason, str) and self.reason, "reason must be a non-empty str")
        _validate_finite("timestamp_s", self.timestamp_s)
        _require(isinstance(self.autopilot_mode, AutopilotMode), "autopilot_mode must be an AutopilotMode")
        _require(isinstance(self.failsafe, bool), "failsafe must be a bool")


@dataclasses.dataclass(frozen=True)
class SITLTelemetry:
    vehicle_id: str
    namespace: str
    timestamp_s: float
    sequence: int
    position_m: Optional[Tuple[float, float, float]]
    velocity_mps: Optional[Tuple[float, float, float]]
    attitude_rad: Optional[Tuple[float, float, float]]
    angular_velocity_radps: Optional[Tuple[float, float, float]]
    battery_fraction: Optional[float]
    estimator_valid: bool
    armed: bool
    flight_mode: AutopilotMode
    failsafe: bool
    heartbeat_ok: bool
    last_command_ack_status: Optional[str]

    def __post_init__(self):
        _validate_nonempty_str("vehicle_id", self.vehicle_id)
        _validate_nonempty_str("namespace", self.namespace)
        _validate_finite("timestamp_s", self.timestamp_s)
        _validate_nonneg_int("sequence", self.sequence)
        _validate_vec3_or_none("position_m", self.position_m)
        _validate_vec3_or_none("velocity_mps", self.velocity_mps)
        _validate_vec3_or_none("attitude_rad", self.attitude_rad)
        _validate_vec3_or_none("angular_velocity_radps", self.angular_velocity_radps)
        if self.battery_fraction is not None:
            _require(0.0 <= self.battery_fraction <= 1.0,
                      f"battery_fraction must be in [0, 1] or None, got {self.battery_fraction!r}")
        _require(isinstance(self.estimator_valid, bool), "estimator_valid must be a bool")
        _require(isinstance(self.armed, bool), "armed must be a bool")
        _require(isinstance(self.flight_mode, AutopilotMode), "flight_mode must be an AutopilotMode")
        _require(isinstance(self.failsafe, bool), "failsafe must be a bool")
        _require(isinstance(self.heartbeat_ok, bool), "heartbeat_ok must be a bool")
        if self.last_command_ack_status is not None:
            _validate_nonempty_str("last_command_ack_status", self.last_command_ack_status)


# --------------------------------------------------------------------------
# Offline MAVLink status-mapping tables (specification only - see module
# docstring and docs/PHASE7_SITL_INTEGRATION.md's "future ArduPilot/PX4 SITL
# mapping" sections).
# --------------------------------------------------------------------------

MAVLINK_HEARTBEAT_MAPPING_SPEC = {
    "heartbeat_ok=True": "a HEARTBEAT message was received within the link's expected period",
    "heartbeat_ok=False": "no HEARTBEAT message received within the timeout - link presumed lost",
}

MAVLINK_COMMAND_ACK_MAPPING_SPEC = {
    "accepted=True": "COMMAND_ACK.result == MAV_RESULT_ACCEPTED",
    "accepted=False:sequence_replayed_stale": "COMMAND_ACK.result == MAV_RESULT_TEMPORARILY_REJECTED (stale seq)",
    "accepted=False:sequence_replayed_conflicting": "COMMAND_ACK.result == MAV_RESULT_DENIED (seq conflict)",
    "accepted=False:frame_mismatch": "COMMAND_ACK.result == MAV_RESULT_UNSUPPORTED (coordinate_frame)",
    "accepted=False:expired": "COMMAND_ACK.result == MAV_RESULT_TEMPORARILY_REJECTED (stale timestamp)",
    "accepted=False:namespace_mismatch": "no MAVLink analogue - this project's own multi-vehicle routing guard",
}

MAVLINK_FAILSAFE_MAPPING_SPEC = {
    "HEARTBEAT_LOSS": "SYS_STATUS/RADIO_STATUS link-loss failsafe (ArduPilot FS_THR / PX4 COM_DL_LOSS_T)",
    "TELEMETRY_PACKET_LOSS": "elevated packet loss inferred from missed EKF/GLOBAL_POSITION_INT stream, no single flag",
    "ESTIMATOR_INVALID": "EKF_STATUS_REPORT variance flags / PX4 ekf2 'not ready' status",
    "BATTERY_CRITICAL": "BATTERY_STATUS below failsafe threshold (ArduPilot FS_BATT / PX4 COM_LOW_BAT_ACT)",
    "VEHICLE_DISCONNECT": "loss of the MAVLink connection itself (heartbeat timeout at the GCS/companion level)",
}
