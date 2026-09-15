"""Phase 1 simulator contract: versioned, validated data interfaces shared by
every future stage (sensor realism, safety supervisor, consensus, autopilot
adapters). Nothing in this module changes existing mission/controller
behavior - it only defines the vocabulary that later phases will migrate
onto and populate.

Design choices, and why:

- Every dataclass is frozen (immutable after construction) and every
  vector field is a plain tuple of floats, never a NumPy array. A tuple
  cannot be mutated in place and cannot be accidentally shared/aliased the
  way a NumPy array reference can - this is what satisfies the "don't let
  mutable arrays leak between objects" requirement structurally, rather
  than by remembering to call .copy() at every call site. `to_numpy()` on
  each vector-shaped field's owner (or the module-level `as_array()` below)
  is the explicit, opt-in way to get a NumPy view for the existing
  PyBullet/MAVLink code, and it always returns a fresh array.
- Every dataclass validates itself in __post_init__: shape, finiteness,
  ranges, and cross-field consistency (e.g. expiration after timestamp).
  A malformed contract object cannot be constructed at all - callers find
  out immediately, not three steps later when a NaN reaches PyBullet.
- Frame is explicit on every field that carries a position/velocity below
  contract level, rather than assumed. Nothing here performs a frame
  conversion; Phase 6 (autopilot adapter) is where ENU<->NED conversion
  happens, deliberately, and only there.
"""
from __future__ import annotations

import dataclasses
import math
from enum import Enum
from typing import Optional, Tuple

CONTRACT_VERSION = "0.1.0"

Vec2 = Tuple[float, float]
Vec3 = Tuple[float, float, float]


# --------------------------------------------------------------------------
# Shared enums (vocabulary only - no behavior attached in Phase 1)
# --------------------------------------------------------------------------

class Frame(Enum):
    """Coordinate frame a position/velocity field is expressed in. Never
    converted implicitly anywhere in this module - see module docstring.

    LOCAL_ENU and LOCAL_NED name conventional axis meanings; PyBullet
    itself has no notion of either. PyBullet is a generic Z-up rigid-body
    world (gravity along -Z is a real PyBullet convention), but what its
    X/Y axes *mean* is purely an application-level choice. `mission.py`
    today chooses to treat PyBullet's world frame as LOCAL_ENU - that is
    this project's convention, not a property PyBullet enforces. The exact
    mapping is written down once, below, as
    `PYBULLET_LOCAL_ENU_AXIS_CONVENTION`, so Phase 6's adapter tests have a
    single documented source to convert against instead of each call site
    guessing. MAVLink/ArduCopter's SET_POSITION_TARGET_LOCAL_NED, by
    contrast, genuinely is NED by protocol definition - that one is not
    this project's choice.
    """
    LOCAL_ENU = "LOCAL_ENU"
    LOCAL_NED = "LOCAL_NED"
    BODY = "BODY"              # vehicle body frame


# This project's current application-level choice of what LOCAL_ENU's axes
# mean when the underlying simulator is PyBullet (see Frame docstring
# above). Not enforced anywhere in Phase 1 - documented so a later phase
# has one fixture to test the exact PyBullet<->ENU<->NED mapping against,
# instead of re-deriving it from scattered call sites.
PYBULLET_LOCAL_ENU_AXIS_CONVENTION = {
    "x": "application-chosen East-like axis (PyBullet world +X)",
    "y": "application-chosen North-like axis (PyBullet world +Y)",
    "z": "Up (PyBullet world +Z) - this one is an actual PyBullet convention, "
         "since gravity is applied along -Z",
}


class HealthState(Enum):
    OK = "OK"
    DEGRADED = "DEGRADED"
    FAILED = "FAILED"


class CommandType(Enum):
    POSITION_SETPOINT = "POSITION_SETPOINT"
    VELOCITY_SETPOINT = "VELOCITY_SETPOINT"
    YAW_SETPOINT = "YAW_SETPOINT"
    YAW_RATE_SETPOINT = "YAW_RATE_SETPOINT"
    HOLD = "HOLD"
    LAND = "LAND"
    ABORT = "ABORT"


class SafetyState(Enum):
    """The 11-state safety FSM vocabulary. Defined here as a type only -
    no supervisor exists yet to produce these; that's a later phase."""
    NORMAL = "NORMAL"
    DEGRADED_SENSOR = "DEGRADED_SENSOR"
    DEGRADED_LINK = "DEGRADED_LINK"
    LOW_BATTERY = "LOW_BATTERY"
    LOST_AGENT = "LOST_AGENT"
    GEOFENCE_RISK = "GEOFENCE_RISK"
    SEPARATION_RISK = "SEPARATION_RISK"
    ABORT = "ABORT"
    SAFE_HOLD = "SAFE_HOLD"
    RETURN_TO_SAFE_POINT = "RETURN_TO_SAFE_POINT"
    LAND_REQUESTED = "LAND_REQUESTED"


# --------------------------------------------------------------------------
# Validation helpers
# --------------------------------------------------------------------------

def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def _validate_finite(name: str, value: float) -> None:
    _require(isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value),
              f"{name} must be a finite number, got {value!r}")


def _validate_nonneg_int(name: str, value: int) -> None:
    _require(isinstance(value, int) and not isinstance(value, bool) and value >= 0,
              f"{name} must be a non-negative int, got {value!r}")


def _validate_vec(name: str, value, length: int) -> None:
    _require(isinstance(value, tuple) and len(value) == length,
              f"{name} must be a {length}-tuple of floats, got {value!r}")
    for i, component in enumerate(value):
        _validate_finite(f"{name}[{i}]", component)


def _validate_unit_range(name: str, value: float) -> None:
    _validate_finite(name, value)
    _require(0.0 <= value <= 1.0, f"{name} must be in [0, 1], got {value!r}")


def _validate_nonnegative(name: str, value: float) -> None:
    _validate_finite(name, value)
    _require(value >= 0.0, f"{name} must be >= 0, got {value!r}")


def _validate_nonempty_str(name: str, value: str) -> None:
    _require(isinstance(value, str) and len(value) > 0, f"{name} must be a non-empty string")


def _validate_enum(name: str, value, enum_type) -> None:
    _require(isinstance(value, enum_type), f"{name} must be a {enum_type.__name__}, got {value!r}")


# --------------------------------------------------------------------------
# WorldState
# --------------------------------------------------------------------------

@dataclasses.dataclass(frozen=True)
class Obstacle:
    obstacle_id: str
    position_m: Vec2          # x, y - obstacles are vertical cylinders today
    radius_m: float

    def __post_init__(self):
        _validate_nonempty_str("obstacle_id", self.obstacle_id)
        _validate_vec("position_m", self.position_m, 2)
        _validate_finite("radius_m", self.radius_m)
        _require(self.radius_m > 0.0, f"radius_m must be > 0, got {self.radius_m!r}")


@dataclasses.dataclass(frozen=True)
class GeofenceSpec:
    frame: Frame
    center_m: Vec2
    half_extents_m: Vec2      # axis-aligned box half-extents, meters
    floor_alt_m: float
    ceiling_alt_m: float

    def __post_init__(self):
        _validate_enum("frame", self.frame, Frame)
        _validate_vec("center_m", self.center_m, 2)
        _validate_vec("half_extents_m", self.half_extents_m, 2)
        _require(all(h > 0.0 for h in self.half_extents_m),
                  f"half_extents_m must be > 0, got {self.half_extents_m!r}")
        _validate_finite("floor_alt_m", self.floor_alt_m)
        _validate_finite("ceiling_alt_m", self.ceiling_alt_m)
        _require(self.floor_alt_m < self.ceiling_alt_m,
                  "floor_alt_m must be < ceiling_alt_m")


@dataclasses.dataclass(frozen=True)
class VictimGroundTruth:
    """Ground truth. Used only for mission scoring - never a valid input to
    any controller, sensor model, or command path."""
    victim_id: str
    position_m: Vec2
    found: bool

    def __post_init__(self):
        _validate_nonempty_str("victim_id", self.victim_id)
        _validate_vec("position_m", self.position_m, 2)
        _require(isinstance(self.found, bool), "found must be a bool")


@dataclasses.dataclass(frozen=True)
class WorldState:
    sim_time_s: float
    step_index: int
    dt_s: float
    frame: Frame
    obstacles: Tuple[Obstacle, ...]
    geofence: GeofenceSpec
    victims_ground_truth: Tuple[VictimGroundTruth, ...]
    contract_version: str = CONTRACT_VERSION

    def __post_init__(self):
        check_contract_version_compatible(self.contract_version, context="WorldState")
        _validate_nonnegative("sim_time_s", self.sim_time_s)
        _validate_nonneg_int("step_index", self.step_index)
        _validate_finite("dt_s", self.dt_s)
        _require(self.dt_s > 0.0, f"dt_s must be > 0, got {self.dt_s!r}")
        _validate_enum("frame", self.frame, Frame)
        _require(isinstance(self.obstacles, tuple) and
                  all(isinstance(o, Obstacle) for o in self.obstacles),
                  "obstacles must be a tuple of Obstacle")
        _require(isinstance(self.geofence, GeofenceSpec), "geofence must be a GeofenceSpec")
        _require(isinstance(self.victims_ground_truth, tuple) and
                  all(isinstance(v, VictimGroundTruth) for v in self.victims_ground_truth),
                  "victims_ground_truth must be a tuple of VictimGroundTruth")


# --------------------------------------------------------------------------
# VehicleState
# --------------------------------------------------------------------------

@dataclasses.dataclass(frozen=True)
class VehicleState:
    vehicle_id: str
    sim_time_s: float
    frame: Frame
    position_m: Vec3
    velocity_mps: Vec3
    acceleration_mps2: Vec3
    attitude_rad: Vec3               # roll, pitch, yaw
    angular_velocity_radps: Vec3
    battery_fraction: float          # 0..1, 1 = full
    health_state: HealthState
    estimator_valid: bool
    last_valid_command_time_s: Optional[float]
    contract_version: str = CONTRACT_VERSION

    def __post_init__(self):
        check_contract_version_compatible(self.contract_version, context="VehicleState")
        _validate_nonempty_str("vehicle_id", self.vehicle_id)
        _validate_nonnegative("sim_time_s", self.sim_time_s)
        _validate_enum("frame", self.frame, Frame)
        _validate_vec("position_m", self.position_m, 3)
        _validate_vec("velocity_mps", self.velocity_mps, 3)
        _validate_vec("acceleration_mps2", self.acceleration_mps2, 3)
        _validate_vec("attitude_rad", self.attitude_rad, 3)
        _validate_vec("angular_velocity_radps", self.angular_velocity_radps, 3)
        _validate_unit_range("battery_fraction", self.battery_fraction)
        _validate_enum("health_state", self.health_state, HealthState)
        _require(isinstance(self.estimator_valid, bool), "estimator_valid must be a bool")
        if self.last_valid_command_time_s is not None:
            _validate_nonnegative("last_valid_command_time_s", self.last_valid_command_time_s)


# --------------------------------------------------------------------------
# SensorObservation
# --------------------------------------------------------------------------

@dataclasses.dataclass(frozen=True)
class DetectionCandidate:
    candidate_id: str
    position_m: Vec2
    frame: Frame
    confidence: float
    localization_uncertainty_m: float   # 1-sigma radius

    def __post_init__(self):
        _validate_nonempty_str("candidate_id", self.candidate_id)
        _validate_vec("position_m", self.position_m, 2)
        _validate_enum("frame", self.frame, Frame)
        _validate_unit_range("confidence", self.confidence)
        _validate_nonnegative("localization_uncertainty_m", self.localization_uncertainty_m)


@dataclasses.dataclass(frozen=True)
class SensorObservation:
    vehicle_id: str
    sensor_timestamp_s: float      # when the sensor sampled the world
    sensor_latency_s: float        # delay before this observation became available
    fov_deg: float
    range_returns_m: Tuple[float, ...]   # math.inf = no return within max range
    occluded: Tuple[bool, ...]           # same length as range_returns_m
    dropout: bool                        # whole-observation dropout flag
    pose_uncertainty_m: float            # 1-sigma
    detections: Tuple[DetectionCandidate, ...]
    contract_version: str = CONTRACT_VERSION

    def __post_init__(self):
        check_contract_version_compatible(self.contract_version, context="SensorObservation")
        _validate_nonempty_str("vehicle_id", self.vehicle_id)
        _validate_nonnegative("sensor_timestamp_s", self.sensor_timestamp_s)
        _validate_nonnegative("sensor_latency_s", self.sensor_latency_s)
        _require(0.0 < self.fov_deg <= 360.0, f"fov_deg must be in (0, 360], got {self.fov_deg!r}")
        _require(isinstance(self.range_returns_m, tuple), "range_returns_m must be a tuple")
        for i, r in enumerate(self.range_returns_m):
            _require(isinstance(r, (int, float)) and (math.isfinite(r) or math.isinf(r)) and r >= 0.0,
                      f"range_returns_m[{i}] must be a finite non-negative number or +inf, got {r!r}")
            _require(not (isinstance(r, float) and math.isnan(r)), f"range_returns_m[{i}] must not be NaN")
        _require(isinstance(self.occluded, tuple) and len(self.occluded) == len(self.range_returns_m),
                  "occluded must be a tuple the same length as range_returns_m")
        _require(all(isinstance(o, bool) for o in self.occluded),
                  "occluded must contain only bool values")
        _require(isinstance(self.dropout, bool), "dropout must be a bool")
        _validate_nonnegative("pose_uncertainty_m", self.pose_uncertainty_m)
        _require(isinstance(self.detections, tuple) and
                  all(isinstance(d, DetectionCandidate) for d in self.detections),
                  "detections must be a tuple of DetectionCandidate")


# --------------------------------------------------------------------------
# NeighborObservation
# --------------------------------------------------------------------------

@dataclasses.dataclass(frozen=True)
class NeighborObservation:
    receiver_id: str
    sender_id: str
    frame: Frame
    measured_position_m: Vec3
    measured_velocity_mps: Vec3
    sample_timestamp_s: float       # when sender sampled its own state
    delivery_timestamp_s: float     # when receiver got it
    packet_age_s: float             # delivery_timestamp_s - sample_timestamp_s
    communication_confidence: float
    stale: bool
    contract_version: str = CONTRACT_VERSION

    def __post_init__(self):
        check_contract_version_compatible(self.contract_version, context="NeighborObservation")
        _validate_nonempty_str("receiver_id", self.receiver_id)
        _validate_nonempty_str("sender_id", self.sender_id)
        _require(self.receiver_id != self.sender_id, "receiver_id and sender_id must differ")
        _validate_enum("frame", self.frame, Frame)
        _validate_vec("measured_position_m", self.measured_position_m, 3)
        _validate_vec("measured_velocity_mps", self.measured_velocity_mps, 3)
        _validate_nonnegative("sample_timestamp_s", self.sample_timestamp_s)
        _validate_nonnegative("delivery_timestamp_s", self.delivery_timestamp_s)
        _require(self.delivery_timestamp_s >= self.sample_timestamp_s,
                  "delivery_timestamp_s must be >= sample_timestamp_s")
        _validate_nonnegative("packet_age_s", self.packet_age_s)
        expected_age = self.delivery_timestamp_s - self.sample_timestamp_s
        _require(math.isclose(self.packet_age_s, expected_age, abs_tol=1e-6),
                  f"packet_age_s ({self.packet_age_s!r}) must equal "
                  f"delivery_timestamp_s - sample_timestamp_s ({expected_age!r})")
        _validate_unit_range("communication_confidence", self.communication_confidence)
        _require(isinstance(self.stale, bool), "stale must be a bool")


# --------------------------------------------------------------------------
# Command
# --------------------------------------------------------------------------

_SETPOINT_FIELDS = ("desired_position_m", "desired_velocity_mps", "yaw_rad", "yaw_rate_radps")

# Each Command is single-purpose: exactly the fields its command_type
# implies are required (must be set), everything else must be None. A
# POSITION_SETPOINT and a VELOCITY_SETPOINT are mutually exclusive on one
# Command (mixing them across axes in one setpoint is exactly the
# ArduCopter type_mask bug found in the WSL bridge last session); yaw and
# yaw-rate are likewise separate, single-purpose command types; HOLD/LAND/
# ABORT are mode commands and carry no setpoint fields at all.
_COMMAND_TYPE_RULES = {
    CommandType.POSITION_SETPOINT: (
        frozenset({"desired_position_m"}),
        frozenset({"desired_velocity_mps", "yaw_rad", "yaw_rate_radps"}),
    ),
    CommandType.VELOCITY_SETPOINT: (
        frozenset({"desired_velocity_mps"}),
        frozenset({"desired_position_m", "yaw_rad", "yaw_rate_radps"}),
    ),
    CommandType.YAW_SETPOINT: (
        frozenset({"yaw_rad"}),
        frozenset({"desired_position_m", "desired_velocity_mps", "yaw_rate_radps"}),
    ),
    CommandType.YAW_RATE_SETPOINT: (
        frozenset({"yaw_rate_radps"}),
        frozenset({"desired_position_m", "desired_velocity_mps", "yaw_rad"}),
    ),
    CommandType.HOLD: (frozenset(), frozenset(_SETPOINT_FIELDS)),
    CommandType.LAND: (frozenset(), frozenset(_SETPOINT_FIELDS)),
    CommandType.ABORT: (frozenset(), frozenset(_SETPOINT_FIELDS)),
}


@dataclasses.dataclass(frozen=True)
class Command:
    vehicle_id: str
    command_type: CommandType
    frame: Frame
    desired_position_m: Optional[Vec3]
    desired_velocity_mps: Optional[Vec3]
    yaw_rad: Optional[float]
    yaw_rate_radps: Optional[float]
    timestamp_s: float
    expiration_time_s: float
    source: str
    confidence: float
    contract_version: str = CONTRACT_VERSION

    def __post_init__(self):
        check_contract_version_compatible(self.contract_version, context="Command")
        _validate_nonempty_str("vehicle_id", self.vehicle_id)
        _validate_enum("command_type", self.command_type, CommandType)
        _validate_enum("frame", self.frame, Frame)
        if self.desired_position_m is not None:
            _validate_vec("desired_position_m", self.desired_position_m, 3)
        if self.desired_velocity_mps is not None:
            _validate_vec("desired_velocity_mps", self.desired_velocity_mps, 3)
        if self.yaw_rad is not None:
            _validate_finite("yaw_rad", self.yaw_rad)
        if self.yaw_rate_radps is not None:
            _validate_finite("yaw_rate_radps", self.yaw_rate_radps)
        _validate_nonnegative("timestamp_s", self.timestamp_s)
        _validate_nonnegative("expiration_time_s", self.expiration_time_s)
        _require(self.expiration_time_s > self.timestamp_s,
                  "expiration_time_s must be > timestamp_s")
        _validate_nonempty_str("source", self.source)
        _validate_unit_range("confidence", self.confidence)

        required, forbidden = _COMMAND_TYPE_RULES[self.command_type]
        for name in required:
            _require(getattr(self, name) is not None,
                      f"{self.command_type.value} requires {name} to be set")
        for name in forbidden:
            _require(getattr(self, name) is None,
                      f"{self.command_type.value} must not set {name}")

    def is_expired(self, now_s: float) -> bool:
        """A command is expired at, and after, its expiration timestamp -
        the boundary itself is not a grace period."""
        return now_s >= self.expiration_time_s


# --------------------------------------------------------------------------
# SafetyDecision
# --------------------------------------------------------------------------

@dataclasses.dataclass(frozen=True)
class SafetyDecision:
    vehicle_id: str
    sim_time_s: float
    accepted: bool
    filtered_command: Optional[Command]
    active_constraints: Tuple[str, ...]
    reason: str
    emergency_state: SafetyState
    min_predicted_clearance_m: Optional[float]
    time_to_collision_s: Optional[float]
    contract_version: str = CONTRACT_VERSION

    def __post_init__(self):
        check_contract_version_compatible(self.contract_version, context="SafetyDecision")
        _validate_nonempty_str("vehicle_id", self.vehicle_id)
        _validate_nonnegative("sim_time_s", self.sim_time_s)
        _require(isinstance(self.accepted, bool), "accepted must be a bool")
        # Policy: an accepted decision is the one that says what actually
        # got sent on, so it must carry that command; a rejected decision
        # sent nothing on, so it must not carry one.
        if self.accepted:
            _require(self.filtered_command is not None,
                      "an accepted SafetyDecision must include filtered_command")
        else:
            _require(self.filtered_command is None,
                      "a rejected SafetyDecision must not include filtered_command")
        if self.filtered_command is not None:
            _require(isinstance(self.filtered_command, Command),
                      "filtered_command must be a Command or None")
            _require(self.filtered_command.vehicle_id == self.vehicle_id,
                      "filtered_command.vehicle_id must match SafetyDecision.vehicle_id")
        _require(isinstance(self.active_constraints, tuple) and
                  all(isinstance(c, str) and len(c) > 0 for c in self.active_constraints),
                  "active_constraints must be a tuple of non-empty str")
        _require(isinstance(self.reason, str), "reason must be a str")
        _validate_enum("emergency_state", self.emergency_state, SafetyState)
        if self.min_predicted_clearance_m is not None:
            _validate_nonnegative("min_predicted_clearance_m", self.min_predicted_clearance_m)
        if self.time_to_collision_s is not None:
            _validate_nonnegative("time_to_collision_s", self.time_to_collision_s)


# --------------------------------------------------------------------------
# Generic JSON-compatible serialization for every contract type above
# --------------------------------------------------------------------------

def _parse_version(version_str: str) -> Tuple[int, int, int]:
    parts = version_str.split(".")
    _require(len(parts) == 3 and all(p.isdigit() for p in parts),
              f"invalid contract version string: {version_str!r}")
    return int(parts[0]), int(parts[1]), int(parts[2])


def check_contract_version_compatible(found_version, *, context: str) -> None:
    """Compatibility policy: this module's own additive-only guarantee
    (new optional fields, new enum members) only holds within one MAJOR
    version. A different major version means the shape may have changed
    in a way `from_dict` cannot safely paper over, so it's rejected
    outright rather than risk silently misreading old/new data. Any
    minor/patch difference within the same major version is accepted -
    that's exactly what "additive, non-breaking" is for."""
    _require(isinstance(found_version, str),
              f"{context}: contract_version must be a str, got {found_version!r}")
    found_major, _, _ = _parse_version(found_version)
    current_major, _, _ = _parse_version(CONTRACT_VERSION)
    _require(found_major == current_major,
              f"{context}: incompatible contract_version {found_version!r} "
              f"(major {found_major}) - this code understands major version "
              f"{current_major} ({CONTRACT_VERSION!r})")


def to_dict(value):
    """Recursively convert any contract dataclass (or plain value) into a
    JSON-compatible structure: Enum -> .value, dataclass -> dict, tuple/list
    -> list, everything else passed through unchanged."""
    if isinstance(value, Enum):
        return value.value
    if dataclasses.is_dataclass(value) and not isinstance(value, type):
        return {f.name: to_dict(getattr(value, f.name)) for f in dataclasses.fields(value)}
    if isinstance(value, (tuple, list)):
        return [to_dict(v) for v in value]
    return value


_ENUM_TYPES = {
    "frame": Frame, "health_state": HealthState, "command_type": CommandType,
    "emergency_state": SafetyState,
}
_NESTED_TYPES = {
    "geofence": GeofenceSpec, "filtered_command": Command,
}
_NESTED_TUPLE_TYPES = {
    "obstacles": Obstacle, "victims_ground_truth": VictimGroundTruth,
    "detections": DetectionCandidate,
}
_TUPLE_FIELDS = {
    "position_m", "velocity_mps", "acceleration_mps2", "attitude_rad",
    "angular_velocity_radps", "measured_position_m", "measured_velocity_mps",
    "desired_position_m", "desired_velocity_mps", "center_m", "half_extents_m",
    "active_constraints", "range_returns_m", "occluded",
}


def from_dict(cls, data: dict):
    """Reconstruct a contract dataclass from the JSON-compatible structure
    `to_dict` produces. Field-name-driven (not full typing introspection) -
    deliberately simple, matching this module's small fixed set of types.

    Two things are enforced strictly, not left to fall through to a
    generic TypeError from the constructor: every field is required
    (present in `data`) except `contract_version` itself, whose absence/
    presence is governed by the version check below instead; and, for any
    type that carries a `contract_version` field, that field must be
    present in `data` and its major version must match this module's
    current CONTRACT_VERSION."""
    if data is None:
        return None
    _require(isinstance(data, dict), f"{cls.__name__}.from_dict requires a dict, got {type(data)!r}")

    field_names = [f.name for f in dataclasses.fields(cls)]
    if "contract_version" in field_names:
        _require("contract_version" in data,
                  f"{cls.__name__}.from_dict requires 'contract_version' in serialized data")
        check_contract_version_compatible(data["contract_version"], context=cls.__name__)

    required = [n for n in field_names if n != "contract_version"]
    missing = [n for n in required if n not in data]
    _require(not missing, f"{cls.__name__}.from_dict missing required fields: {sorted(missing)}")

    kwargs = {}
    for name in field_names:
        if name == "contract_version":
            kwargs[name] = data.get("contract_version", CONTRACT_VERSION)
            continue
        raw = data[name]
        if raw is None:
            kwargs[name] = None
        elif name in _ENUM_TYPES:
            kwargs[name] = _ENUM_TYPES[name](raw)
        elif name in _NESTED_TYPES:
            kwargs[name] = from_dict(_NESTED_TYPES[name], raw)
        elif name in _NESTED_TUPLE_TYPES:
            kwargs[name] = tuple(from_dict(_NESTED_TUPLE_TYPES[name], item) for item in raw)
        elif name in _TUPLE_FIELDS:
            kwargs[name] = tuple(raw)
        else:
            kwargs[name] = raw
    return cls(**kwargs)
