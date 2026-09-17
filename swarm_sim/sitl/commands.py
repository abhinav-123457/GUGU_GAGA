"""Phase 7: SITL command schema and offline MAVLink command/mode-name
mapping tables - see docs/PHASE7_SITL_INTEGRATION.md.

SITLCommand wraps an already-validated swarm_sim.autopilot.types.AdapterCommand
(which itself only ever wraps an already-validated swarm_sim.contracts.Command)
with the two extra fields a local SITL transport layer needs:

- `namespace` - this vehicle's isolated channel (see vehicle_namespace.py).
- `safety_decision_id` - a deterministic, derived identifier for the
  SafetyDecision that produced this command's filtered_command (see
  `derive_safety_decision_id` below). Phase 7 does not add a stored id
  field to contracts.SafetyDecision itself - that dataclass, and
  safety_supervisor.py generally, are out of scope for this phase per the
  reviewer's explicit instruction carried over from Phase 5. The id is
  computed purely from already-public SafetyDecision/Command fields.

There is still no constructor path from a raw SwarmController candidate to
a SITLCommand: building one requires an AdapterCommand, which requires a
validated Command - see autopilot/types.py's own docstring.
"""
from __future__ import annotations

import dataclasses
import hashlib

from ..autopilot.types import AdapterCommand
from ..contracts import CommandType, Frame


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def derive_safety_decision_id(vehicle_id: str, sim_time_s: float, filtered_command) -> str:
    """A deterministic identifier for the SafetyDecision that produced
    `filtered_command`, derived purely from already-public fields (never
    stored on contracts.SafetyDecision itself - see module docstring). Two
    calls with the same vehicle_id/sim_time_s/filtered_command content
    always produce the same id (needed for exact-replay determinism tests);
    any difference in content changes it."""
    parts = [
        vehicle_id, f"{sim_time_s:.9f}", filtered_command.command_type.value,
        filtered_command.frame.value, repr(filtered_command.desired_position_m),
        repr(filtered_command.desired_velocity_mps), repr(filtered_command.yaw_rad),
        repr(filtered_command.yaw_rate_radps), f"{filtered_command.timestamp_s:.9f}",
        f"{filtered_command.expiration_time_s:.9f}", filtered_command.source,
    ]
    digest = hashlib.sha256("|".join(parts).encode("utf-8")).hexdigest()[:16]
    return f"sd-{vehicle_id}-{digest}"


@dataclasses.dataclass(frozen=True)
class SITLCommand:
    adapter_command: AdapterCommand
    namespace: str
    safety_decision_id: str

    def __post_init__(self):
        _require(isinstance(self.adapter_command, AdapterCommand),
                  f"adapter_command must be an AdapterCommand, got {type(self.adapter_command)!r}")
        _require(isinstance(self.namespace, str) and self.namespace, "namespace must be a non-empty str")
        _require(isinstance(self.safety_decision_id, str) and self.safety_decision_id,
                  "safety_decision_id must be a non-empty str")

    @property
    def vehicle_id(self) -> str:
        return self.adapter_command.vehicle_id

    @property
    def sequence(self) -> int:
        return self.adapter_command.sequence

    @property
    def command_type(self) -> CommandType:
        return self.adapter_command.command.command_type

    @property
    def frame(self) -> Frame:
        return self.adapter_command.frame

    @property
    def desired_position_m(self):
        return self.adapter_command.command.desired_position_m

    @property
    def desired_velocity_mps(self):
        return self.adapter_command.command.desired_velocity_mps

    @property
    def yaw_rad(self):
        return self.adapter_command.command.yaw_rad

    @property
    def yaw_rate_radps(self):
        return self.adapter_command.command.yaw_rate_radps

    @property
    def timestamp_s(self) -> float:
        return self.adapter_command.timestamp_s

    @property
    def expiration_time_s(self) -> float:
        return self.adapter_command.expiration_time_s

    @property
    def source(self) -> str:
        return self.adapter_command.source


# --------------------------------------------------------------------------
# Offline MAVLink command/mode-name mapping tables (specification only -
# see module docstring). No pymavlink/MAVSDK import; these are plain dicts
# and naming only, not an implementation of any MAVLink message.
# --------------------------------------------------------------------------

MAVLINK_FRAME_MAPPING_SPEC = {
    Frame.LOCAL_ENU.value: "no direct MAVLink frame - this project's own PyBullet-world convention "
                            "(see contracts.PYBULLET_LOCAL_ENU_AXIS_CONVENTION); converted to LOCAL_NED before "
                            "an eventual MAV_FRAME_LOCAL_NED setpoint",
    Frame.LOCAL_NED.value: "MAV_FRAME_LOCAL_NED (SET_POSITION_TARGET_LOCAL_NED position/velocity fields)",
    Frame.BODY.value: "MAV_FRAME_BODY_FRD (SET_POSITION_TARGET_LOCAL_NED with coordinate_frame=BODY_FRD)",
}

MAVLINK_COMMAND_TYPE_MAPPING_SPEC = {
    CommandType.POSITION_SETPOINT.value: "SET_POSITION_TARGET_LOCAL_NED with only position type_mask bits cleared",
    CommandType.VELOCITY_SETPOINT.value: "SET_POSITION_TARGET_LOCAL_NED with only velocity type_mask bits cleared",
    CommandType.YAW_SETPOINT.value: "SET_POSITION_TARGET_LOCAL_NED yaw field (type_mask yaw bit cleared)",
    CommandType.YAW_RATE_SETPOINT.value: "SET_POSITION_TARGET_LOCAL_NED yaw_rate field (type_mask yaw_rate bit cleared)",
    CommandType.HOLD.value: "MAV_CMD_DO_SET_MODE -> ArduPilot LOITER / PX4 HOLD (no setpoint stream)",
    CommandType.LAND.value: "MAV_CMD_NAV_LAND",
    CommandType.ABORT.value: "MAV_CMD_DO_FLIGHTTERMINATION (ArduPilot) / PX4 kill-switch equivalent - simulated only",
}

MAVLINK_MODE_NAME_MAPPING_SPEC = {
    "HOLD": {"ardupilot": "LOITER", "px4": "HOLD"},
    "OFFBOARD": {"ardupilot": "GUIDED", "px4": "OFFBOARD"},
    "GUIDED": {"ardupilot": "GUIDED", "px4": "OFFBOARD"},
    "RTL": {"ardupilot": "RTL", "px4": "AUTO.RTL"},
    "LAND": {"ardupilot": "LAND", "px4": "AUTO.LAND"},
    "ABORT": {"ardupilot": "n/a - simulated termination only", "px4": "n/a - simulated termination only"},
    "DISARMED": {"ardupilot": "n/a (arming state, not a flight mode)", "px4": "n/a (arming state, not a flight mode)"},
}
