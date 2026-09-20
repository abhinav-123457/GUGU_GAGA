"""Phase 15C: offline reference fixtures for concepts useful to this
project from Langostino (swarm-subnet/Langostino, commit 035b334, MIT) -
a single-aircraft ROS2 (Humble)/INAV reference platform verified from its
own README/repo metadata (Raspberry Pi companion computer, INAV flight
controller, MSP serial protocol, GPS/compass, I2C LiDAR, ROS2
observations/actions - see docs/PHASE15_SWARM124_LANGOSTINO_INTEGRATION.md).

This module NEVER imports `serial`/`pyserial`, NEVER opens a socket or
serial port, and NEVER constructs an MSP/RC/PWM command - see
tests/test_phase15_external_architecture.py's
`test_langostino_reference_never_touches_serial_or_msp`. It only maps
plain, offline Python dataclasses (representing what a real Langostino
vehicle's sensors/companion-computer link WOULD report) into this
project's own existing typed contracts (`swarm_sim.contracts.SensorObservation`,
`VehicleState`, `HealthState`), so this project's own SAR mission/
SafetySupervisor code can be reviewed against a second, independently-
designed sensor/fault-handling model without ever running Langostino's
own code or touching real hardware.

Every "Do not" from Phase 15's own spec is structural here, not just
documented: there is no code path in this module that could send a
receiver-channel override of any kind, because this module never
constructs, imports, or references a serial/companion-link command type
at all - it only takes and returns this project's own dataclasses.
"""
from __future__ import annotations

import dataclasses
import math
from typing import Optional

from ..contracts import Frame, HealthState, SensorObservation, Vec3, VehicleState


@dataclasses.dataclass(frozen=True)
class LangostinoLidarFixture:
    """Offline stand-in for a Langostino-style I2C LiDAR single-beam
    distance reading. `valid=False` represents the "missing LiDAR"
    scenario (I2C read failure/timeout) - a real, expected fault mode,
    not something this fixture pretends can't happen."""
    vehicle_id: str
    timestamp_s: float
    distance_m: Optional[float]   # None when valid is False
    valid: bool


@dataclasses.dataclass(frozen=True)
class LangostinoTelemetryFixture:
    """Offline stand-in for Langostino's GPS/attitude/altitude telemetry -
    ROS2-style, timestamped, and deliberately allowed to hold an invalid
    altitude (e.g. a barometer glitch) since this fixture exists
    specifically to exercise that fault path, not to assume it away."""
    vehicle_id: str
    timestamp_s: float
    position_m: Vec3
    attitude_rad: Vec3
    altitude_m: float
    gps_fix: bool


@dataclasses.dataclass(frozen=True)
class LangostinoLinkFixture:
    """Offline stand-in for the companion-computer <-> flight-controller
    link Langostino's own architecture depends on (Raspberry Pi speaking
    MSP to INAV) - this project only ever sees the OUTCOME of that link
    (connected/heartbeat-age), never the MSP bytes themselves."""
    vehicle_id: str
    timestamp_s: float
    companion_computer_connected: bool
    last_heartbeat_s: float
    emergency_landing_requested: bool = False


# Altitude bounds a real vehicle could plausibly report - anything outside
# this range is treated as a sensor fault (e.g. an uninitialized/garbage
# barometer reading), not a real altitude. Deliberately generous (this
# project's own live hard ceiling is 2.0m - see docs/PHASE14_SAR_WEBOTS.md)
# since this is a sanity bound on the SENSOR, not a mission limit.
_MIN_PLAUSIBLE_ALTITUDE_M = -50.0
_MAX_PLAUSIBLE_ALTITUDE_M = 500.0


def is_altitude_valid(altitude_m: float) -> bool:
    return (isinstance(altitude_m, (int, float)) and not isinstance(altitude_m, bool)
            and math.isfinite(altitude_m) and _MIN_PLAUSIBLE_ALTITUDE_M <= altitude_m <= _MAX_PLAUSIBLE_ALTITUDE_M)


def is_observation_stale(timestamp_s: float, now_s: float, max_age_s: float) -> bool:
    """Shared staleness check for any Langostino-style fixture - mirrors
    `SARMission`'s own `now_s - telem.timestamp_s > stale_telemetry_max_age_s`
    pattern (swarm_sim/sar_mission.py) rather than inventing a new one."""
    return (now_s - timestamp_s) > max_age_s


def map_lidar_to_sensor_observation(fixture: LangostinoLidarFixture, *, fov_deg: float = 5.0,
                                     sensor_latency_s: float = 0.0, pose_uncertainty_m: float = 0.1):
    """Maps a single-beam LiDAR reading onto this project's existing
    `SensorObservation` (a 1-element `range_returns_m`). The "missing
    LiDAR" fixture (`valid=False`) maps to `dropout=True` with an empty
    range-returns tuple, exactly the whole-observation-dropout case
    `SensorObservation` already models - not a new, Langostino-specific
    concept."""
    if not fixture.valid or fixture.distance_m is None:
        return SensorObservation(
            vehicle_id=fixture.vehicle_id, sensor_timestamp_s=fixture.timestamp_s,
            sensor_latency_s=sensor_latency_s, fov_deg=fov_deg, range_returns_m=(), occluded=(),
            dropout=True, pose_uncertainty_m=pose_uncertainty_m, detections=(),
        )
    return SensorObservation(
        vehicle_id=fixture.vehicle_id, sensor_timestamp_s=fixture.timestamp_s, sensor_latency_s=sensor_latency_s,
        fov_deg=fov_deg, range_returns_m=(float(fixture.distance_m),), occluded=(False,), dropout=False,
        pose_uncertainty_m=pose_uncertainty_m, detections=(),
    )


def map_telemetry_to_vehicle_state(fixture: LangostinoTelemetryFixture, *, sim_time_s: float,
                                    velocity_mps: Vec3 = (0.0, 0.0, 0.0),
                                    acceleration_mps2: Vec3 = (0.0, 0.0, 0.0),
                                    angular_velocity_radps: Vec3 = (0.0, 0.0, 0.0),
                                    battery_fraction: float = 1.0) -> VehicleState:
    """Maps Langostino-style GPS/attitude/altitude telemetry onto this
    project's existing `VehicleState`. An invalid altitude
    (`is_altitude_valid()` False) or a missing GPS fix maps to
    `estimator_valid=False` and `health_state=DEGRADED`.

    Unlike `CandidateCommand` (this project's deliberately-permissive
    untrusted type), `VehicleState.__post_init__` requires every
    position/velocity component to be finite - it has no "NaN allowed,
    caller must check a flag" mode. So an invalid altitude is substituted
    with a safe `0.0` sentinel here rather than passed through as NaN
    (which would raise inside `VehicleState.__init__` and crash the
    caller instead of degrading it - a real bug this module's own smoke
    test caught). The sentinel is never meant to be trusted on its own -
    `estimator_valid=False`/`health_state=DEGRADED` are what tell the
    caller to disregard `position_m`, exactly the same pattern this
    project already uses for a real ArduPilot EKF's invalid-estimate case
    (`VehicleTelemetry.estimator_valid`, swarm_sim/autopilot/types.py)."""
    altitude_ok = is_altitude_valid(fixture.altitude_m)
    x, y, _z = fixture.position_m
    position_m = (x, y, fixture.altitude_m if altitude_ok else 0.0)
    healthy = altitude_ok and fixture.gps_fix
    return VehicleState(
        vehicle_id=fixture.vehicle_id, sim_time_s=sim_time_s, frame=Frame.LOCAL_ENU, position_m=position_m,
        velocity_mps=velocity_mps, acceleration_mps2=acceleration_mps2, attitude_rad=fixture.attitude_rad,
        angular_velocity_radps=angular_velocity_radps, battery_fraction=battery_fraction,
        health_state=HealthState.OK if healthy else HealthState.DEGRADED,
        estimator_valid=healthy, last_valid_command_time_s=None,
    )


def map_link_status_to_health_state(fixture: LangostinoLinkFixture, *, now_s: float,
                                     max_heartbeat_age_s: float = 2.0) -> HealthState:
    """Maps companion-computer link status onto this project's own
    `HealthState`. Deliberately DEGRADED, not FAILED, for a stale/lost
    companion-computer link on its own: in Langostino's real architecture
    the flight controller (INAV) keeps its own onboard failsafes even if
    the Raspberry Pi companion is unreachable, so losing that link alone
    is a real but recoverable degradation, not a full vehicle failure.
    An explicit `emergency_landing_requested=True` - a deliberate operator/
    companion-computer decision, not a mere link timeout - maps to FAILED,
    matching how this project already treats an operator-abort-class event
    elsewhere (`MissionContext.operator_abort`, safety_supervisor.py)."""
    if fixture.emergency_landing_requested:
        return HealthState.FAILED
    link_lost = (not fixture.companion_computer_connected
                 or (now_s - fixture.last_heartbeat_s) > max_heartbeat_age_s)
    return HealthState.DEGRADED if link_lost else HealthState.OK


# --------------------------------------------------------------------------
# Named fixture builders for the exact scenarios Phase 15C's spec requires -
# one function per bullet, so a test can name the scenario it is exercising
# instead of hand-building a fixture inline every time.
# --------------------------------------------------------------------------

def lidar_distance_observation(vehicle_id: str = "drone0", timestamp_s: float = 0.0,
                                distance_m: float = 3.5) -> LangostinoLidarFixture:
    return LangostinoLidarFixture(vehicle_id=vehicle_id, timestamp_s=timestamp_s, distance_m=distance_m, valid=True)


def missing_lidar(vehicle_id: str = "drone0", timestamp_s: float = 0.0) -> LangostinoLidarFixture:
    return LangostinoLidarFixture(vehicle_id=vehicle_id, timestamp_s=timestamp_s, distance_m=None, valid=False)


def gps_attitude_altitude_telemetry(vehicle_id: str = "drone0", timestamp_s: float = 0.0,
                                     position_m: Vec3 = (0.0, 0.0, 1.0), altitude_m: float = 1.0,
                                     gps_fix: bool = True) -> LangostinoTelemetryFixture:
    return LangostinoTelemetryFixture(vehicle_id=vehicle_id, timestamp_s=timestamp_s, position_m=position_m,
                                       attitude_rad=(0.0, 0.0, 0.0), altitude_m=altitude_m, gps_fix=gps_fix)


def stale_observation(vehicle_id: str = "drone0", stale_timestamp_s: float = 0.0) -> LangostinoLidarFixture:
    return lidar_distance_observation(vehicle_id=vehicle_id, timestamp_s=stale_timestamp_s)


def invalid_altitude_telemetry(vehicle_id: str = "drone0", timestamp_s: float = 0.0) -> LangostinoTelemetryFixture:
    return gps_attitude_altitude_telemetry(vehicle_id=vehicle_id, timestamp_s=timestamp_s,
                                            altitude_m=float("nan"), gps_fix=True)


def emergency_landing_request(vehicle_id: str = "drone0", timestamp_s: float = 0.0) -> LangostinoLinkFixture:
    return LangostinoLinkFixture(vehicle_id=vehicle_id, timestamp_s=timestamp_s,
                                  companion_computer_connected=True, last_heartbeat_s=timestamp_s,
                                  emergency_landing_requested=True)


def companion_computer_link_loss(vehicle_id: str = "drone0", timestamp_s: float = 0.0,
                                  last_heartbeat_s: float = -100.0) -> LangostinoLinkFixture:
    return LangostinoLinkFixture(vehicle_id=vehicle_id, timestamp_s=timestamp_s,
                                  companion_computer_connected=False, last_heartbeat_s=last_heartbeat_s)
