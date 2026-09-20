"""Phase 14: one-vehicle simulated SAR mission - offline state machine and
runner. See docs/PHASE14_SAR_WEBOTS.md.

    This is a one-drone simulated SAR mission prototype - not an
    operational search-and-rescue system.

Command path (never bypassed - see tests/test_phase14_sar_architecture.py):

    SARMission (this module) builds a raw CandidateCommand
        -> SafetySupervisor.evaluate()                (swarm_sim.safety_supervisor, reused unmodified)
        -> SafetyDecision.filtered_command
        -> AdapterCommand
        -> SITLAdapter.send_command()                  (offline: swarm_sim.autopilot.sitl + FakeSITLTransport;
                                                          live: scripts/run_phase14_sar_mission.py's own
                                                          ArduPilotSITLTransport wiring, reusing Phase 12/13
                                                          gate helpers - never this module)

`SARMission.tick()` never calls `adapter.send_command()` itself - it
returns a `TickResult` and the caller (`run_sar_mission_offline` below, or
the live runner in scripts/run_phase14_sar_mission.py) is responsible for
actually sending `tick_result.adapter_command` and reporting the result
back via `record_command_result()`. This keeps the state machine testable
without any transport at all (see tests/test_phase14_sar_mission.py's
direct `SARMission.tick()` tests) and keeps offline/live as two thin
callers of the same core logic.
"""
from __future__ import annotations

import csv
import dataclasses
import enum
import json
import math
import os
import random
import time
from typing import Dict, List, Optional, Tuple

from .autopilot.sitl import SITLAdapter
from .autopilot.types import AdapterCommand, AdapterResult, VehicleTelemetry
from .contracts import (
    Frame, GeofenceSpec, HealthState, SafetyDecision, SafetyState, SensorObservation, VehicleState,
)
from .manifest import build_run_manifest
from .safety_supervisor import CandidateCommand, MissionContext, SafetySupervisor, SafetySupervisorConfig
from .sar_search import LawnmowerSearchPattern, generate_lawnmower_waypoints, velocity_toward
from .sar_world import RectangularSearchArea, SARWorld, SensorModelConfig
from .seeding import SeedManager
from .sitl.fake_transport import FakeSITLTransport

Vec2 = Tuple[float, float]
Vec3 = Tuple[float, float, float]


# --------------------------------------------------------------------------
# Mission states
# --------------------------------------------------------------------------

class MissionState(enum.Enum):
    INIT = "INIT"
    PREFLIGHT_CHECK = "PREFLIGHT_CHECK"
    TAKEOFF_REQUESTED = "TAKEOFF_REQUESTED"
    TRANSIT = "TRANSIT"
    SEARCHING = "SEARCHING"
    DETECTION_CANDIDATE = "DETECTION_CANDIDATE"
    DETECTION_CONFIRMED = "DETECTION_CONFIRMED"
    RETURN_HOME = "RETURN_HOME"
    LAND_REQUESTED = "LAND_REQUESTED"
    LANDED = "LANDED"
    ABORTED = "ABORTED"
    FAILED = "FAILED"


TERMINAL_STATES = frozenset({MissionState.LANDED, MissionState.ABORTED, MissionState.FAILED})

# Every transition this module will ever perform - _transition() raises if
# asked for anything not listed here, so "document every state transition"
# is a structural guarantee, not just a doc comment (see
# tests/test_phase14_sar_mission.py::test_illegal_transition_raises).
ALLOWED_TRANSITIONS: Dict[MissionState, frozenset] = {
    MissionState.INIT: frozenset({MissionState.PREFLIGHT_CHECK, MissionState.FAILED, MissionState.ABORTED}),
    MissionState.PREFLIGHT_CHECK: frozenset({MissionState.TAKEOFF_REQUESTED, MissionState.FAILED,
                                              MissionState.ABORTED}),
    MissionState.TAKEOFF_REQUESTED: frozenset({MissionState.TRANSIT, MissionState.RETURN_HOME,
                                                MissionState.LAND_REQUESTED, MissionState.FAILED,
                                                MissionState.ABORTED}),
    MissionState.TRANSIT: frozenset({MissionState.SEARCHING, MissionState.RETURN_HOME,
                                      MissionState.LAND_REQUESTED, MissionState.FAILED, MissionState.ABORTED}),
    MissionState.SEARCHING: frozenset({MissionState.DETECTION_CANDIDATE, MissionState.DETECTION_CONFIRMED,
                                        MissionState.RETURN_HOME, MissionState.LAND_REQUESTED,
                                        MissionState.FAILED, MissionState.ABORTED}),
    MissionState.DETECTION_CANDIDATE: frozenset({MissionState.SEARCHING, MissionState.DETECTION_CONFIRMED,
                                                  MissionState.RETURN_HOME, MissionState.LAND_REQUESTED,
                                                  MissionState.FAILED, MissionState.ABORTED}),
    MissionState.DETECTION_CONFIRMED: frozenset({MissionState.RETURN_HOME, MissionState.LAND_REQUESTED,
                                                  MissionState.FAILED, MissionState.ABORTED}),
    MissionState.RETURN_HOME: frozenset({MissionState.LAND_REQUESTED, MissionState.FAILED,
                                          MissionState.ABORTED}),
    MissionState.LAND_REQUESTED: frozenset({MissionState.LANDED, MissionState.FAILED, MissionState.ABORTED}),
    MissionState.LANDED: frozenset(),
    MissionState.ABORTED: frozenset(),
    MissionState.FAILED: frozenset(),
}

_ACTIVE_SEARCH_STATES = frozenset({MissionState.TRANSIT, MissionState.SEARCHING,
                                    MissionState.DETECTION_CANDIDATE, MissionState.DETECTION_CONFIRMED})


# --------------------------------------------------------------------------
# Config
# --------------------------------------------------------------------------

@dataclasses.dataclass
class SARMissionConfig:
    vehicle_id: str = "drone0"
    namespace: str = "sitl/drone0"
    system_id: int = 1
    component_id: int = 1
    seed: int = 0

    home_m: Vec3 = (0.0, 0.0, 0.0)
    search_area: RectangularSearchArea = dataclasses.field(
        default_factory=lambda: RectangularSearchArea(0.0, 0.0, 20.0, 20.0))
    search_altitude_m: float = 1.0
    lane_spacing_m: float = 3.0
    geofence_margin_m: float = 1.0
    ground_altitude_m: float = 0.0
    # Deliberately BELOW ground_altitude_m, not equal to it: the
    # SafetySupervisor treats its geofence floor as a no-go boundary to
    # stay away from, not a landing target - if the floor sat exactly at
    # ground level, the supervisor's own altitude-floor protection would
    # prevent the vehicle from ever fully descending to land (found via
    # this phase's own offline smoke test - see docs/PHASE14_SAR_WEBOTS.md).
    # Mirrors Phase 10/13's own FENCE_ALT_MAX being set well above (not at)
    # the operating altitude, just for the floor instead of the ceiling.
    geofence_floor_alt_m: float = -2.0
    geofence_ceiling_alt_m: float = 2.0

    # Optional cap on how many search waypoints to follow before forcing
    # RETURN_HOME even if the pattern isn't complete and nothing was
    # confirmed - None (default) runs the full pattern, exactly as every
    # offline test above exercises. Added for Phase 14B's own "validate
    # the command/telemetry path on a short bounded segment before running
    # the full pattern live" requirement (see docs/PHASE14_SAR_WEBOTS.md) -
    # never changes offline behavior, arm/takeoff/land limits, or any
    # SafetySupervisor gate.
    max_search_waypoints: Optional[int] = None

    victim_positions_m: Tuple[Vec2, ...] = ((10.0, 10.0),)
    sensor_config: Optional[SensorModelConfig] = None
    confirmation_min_detections: int = 2
    confirmation_cluster_radius_m: float = 1.5
    confirmation_min_confidence: float = 0.5
    stale_observation_max_age_s: float = 3.0

    max_speed_mps: float = 0.25
    waypoint_reach_tolerance_m: float = 0.5
    dt_s: float = 0.5
    command_expiration_horizon_s: float = 2.0

    mission_timeout_s: float = 90.0
    command_timeout_s: float = 5.0
    landing_timeout_s: float = 30.0
    estimator_invalid_grace_s: float = 2.0
    heartbeat_loss_grace_s: float = 2.0
    stale_telemetry_max_age_s: float = 2.0
    land_altitude_tolerance_m: float = 0.15
    land_velocity_tolerance_mps: float = 0.05

    safety_config: Optional[SafetySupervisorConfig] = None

    def build_safety_config(self) -> SafetySupervisorConfig:
        if self.safety_config is not None:
            return self.safety_config
        # fallback_dt_s (used only for a vehicle's very first evaluate()
        # call, before any real inter-tick interval is known) defaults to
        # 1/24s in SafetySupervisorConfig - a frame-rate assumption tuned
        # for a much faster swarm-sim control loop elsewhere in this
        # codebase, not this mission's own slower control period. Found
        # via this phase's own live
        # testing: with that mismatched default, the very first safety
        # evaluation after handoff from ArduPilot's own takeoff (while the
        # vehicle still had real residual climb momentum) computed an
        # acceleration budget ~24x too small to meaningfully correct that
        # momentum, letting the vehicle coast well past its intended
        # altitude before a realistic dt took over on the next tick. Using
        # this mission's own `dt_s` instead gives that first correction a
        # realistic budget - see docs/PHASE14_SAR_WEBOTS.md.
        return SafetySupervisorConfig(max_speed_mps=self.max_speed_mps, geofence_margin_m=self.geofence_margin_m,
                                       fallback_dt_s=self.dt_s)

    def build_geofence(self) -> GeofenceSpec:
        """A geofence that safely encloses BOTH the search area and
        `home_m`, each with buffer - never just the search area alone.
        `home_m` sitting on or near a search-area edge (e.g. the common
        case of launching from a corner of the search rectangle) would
        otherwise place the vehicle right on the fence boundary at
        takeoff, causing a GEOFENCE_RISK override before the mission ever
        starts moving (found via this phase's own offline smoke test -
        see docs/PHASE14_SAR_WEBOTS.md).

        The buffer here is `2 * geofence_margin_m`, not `geofence_margin_m`
        - `SafetySupervisor` itself flags GEOFENCE_RISK whenever the
        vehicle's horizontal distance to this boundary drops below
        `geofence_margin_m` (see geofence_horizontal_margin_m() in
        safety_supervisor.py). A one-margin buffer places `home_m` exactly
        ON that risk threshold rather than strictly inside it - offline,
        FakeSITLTransport's noiseless kinematics never crosses that exact
        boundary, but a real live flight's GPS/accel noise and PID
        hunting does, almost every tick, permanently overriding the
        candidate and preventing the mission from ever leaving
        TAKEOFF_REQUESTED (found via this phase's first live Webots run -
        see docs/PHASE14_SAR_WEBOTS.md). Doubling the buffer gives `home_m`
        a full `geofence_margin_m` of real headroom against that noise."""
        buffer_m = 2.0 * self.geofence_margin_m
        min_x = min(self.search_area.min_x_m, self.home_m[0]) - buffer_m
        max_x = max(self.search_area.max_x_m, self.home_m[0]) + buffer_m
        min_y = min(self.search_area.min_y_m, self.home_m[1]) - buffer_m
        max_y = max(self.search_area.max_y_m, self.home_m[1]) + buffer_m
        return GeofenceSpec(
            frame=Frame.LOCAL_ENU, center_m=((min_x + max_x) / 2.0, (min_y + max_y) / 2.0),
            half_extents_m=((max_x - min_x) / 2.0, (max_y - min_y) / 2.0),
            floor_alt_m=self.geofence_floor_alt_m, ceiling_alt_m=self.geofence_ceiling_alt_m,
        )


# --------------------------------------------------------------------------
# Detection clustering (confirmation rule)
# --------------------------------------------------------------------------

@dataclasses.dataclass
class _DetectionCluster:
    centroid_xy: Vec2
    member_ids: List[str] = dataclasses.field(default_factory=list)

    def add(self, candidate_id: str, position_m: Vec2) -> None:
        self.member_ids.append(candidate_id)
        n = len(self.member_ids)
        self.centroid_xy = (
            (self.centroid_xy[0] * (n - 1) + position_m[0]) / n,
            (self.centroid_xy[1] * (n - 1) + position_m[1]) / n,
        )


# --------------------------------------------------------------------------
# Tick result - see module docstring for why send_command lives outside tick()
# --------------------------------------------------------------------------

@dataclasses.dataclass
class TickResult:
    adapter_command: Optional[AdapterCommand] = None
    candidate: Optional[CandidateCommand] = None
    decision: Optional[SafetyDecision] = None


def _empty_observation(vehicle_id: str, now_s: float) -> SensorObservation:
    return SensorObservation(vehicle_id=vehicle_id, sensor_timestamp_s=now_s, sensor_latency_s=0.0,
                              fov_deg=360.0, range_returns_m=(), occluded=(), dropout=True,
                              pose_uncertainty_m=0.0, detections=())


# --------------------------------------------------------------------------
# The mission
# --------------------------------------------------------------------------

class SARMission:
    """One-vehicle SAR mission state machine. Vehicle-agnostic to its
    telemetry source: `tick()` only ever reads a `VehicleTelemetry` (the
    same type both `SITLAdapter(FakeSITLTransport)` [offline] and
    `build_ardupilot_sitl_adapter(ArduPilotSITLTransport)` [live] produce),
    so the identical state-machine/search/confirmation logic drives both
    modes - only transport/adapter construction differs between
    `run_sar_mission_offline` (below) and the live runner in
    scripts/run_phase14_sar_mission.py."""

    def __init__(self, config: SARMissionConfig, world: SARWorld,
                 supervisor: Optional[SafetySupervisor] = None):
        self.config = config
        self.world = world
        self.supervisor = supervisor or SafetySupervisor(config.build_safety_config())
        self.geofence = config.build_geofence()
        waypoints = generate_lawnmower_waypoints(config.search_area, config.search_altitude_m,
                                                   config.lane_spacing_m, margin_m=config.geofence_margin_m)
        self.search_pattern = LawnmowerSearchPattern(waypoints, reach_tolerance_m=config.waypoint_reach_tolerance_m)

        self.state = MissionState.INIT
        self.events: List[dict] = []
        self.telemetry_rows: List[dict] = []
        self.command_rows: List[dict] = []
        self.safety_decision_rows: List[dict] = []
        self.detection_rows: List[dict] = []

        self.failure_reason: Optional[str] = None
        self.confirmed_victim_id: Optional[str] = None
        self.false_confirmation = False

        self._sequence = 0
        self._mission_start_s: Optional[float] = None
        self._last_now_s = 0.0
        self._last_heading_xy: Vec2 = (1.0, 0.0)
        self._last_valid_command_time_s: Optional[float] = None
        self._estimator_invalid_since_s: Optional[float] = None
        self._heartbeat_lost_since_s: Optional[float] = None
        self._stale_telemetry_since_s: Optional[float] = None
        self._land_requested_at_s: Optional[float] = None
        self._return_home_started_at_s: Optional[float] = None
        self._first_rejection_at_s: Optional[float] = None

        self._clusters: List[_DetectionCluster] = []
        self._seen_candidate_ids: set = set()
        self._confirmed_cluster: Optional[_DetectionCluster] = None

        self.observations_generated = 0
        self.detections_used_for_confirmation = 0
        self.duplicate_observations = 0
        self.malformed_observations = 0
        self.stale_observations_rejected = 0
        self.geofence_violation_ticks = 0
        self.max_altitude_m = 0.0
        self.max_speed_mps = 0.0
        self.min_battery_fraction = 1.0
        self.hold_band_altitudes_m: List[float] = []
        self.safety_state_histogram: Dict[str, int] = {}

    # -- state machine ---------------------------------------------------

    def _transition(self, new_state: MissionState, reason: str, **details) -> None:
        if new_state == self.state:
            return
        allowed = ALLOWED_TRANSITIONS.get(self.state, frozenset())
        if new_state not in allowed:
            raise ValueError(f"illegal mission-state transition {self.state.value} -> {new_state.value} "
                              f"(reason={reason!r})")
        self.events.append({
            "wall_time_s": time.time(), "sim_time_s": self._last_now_s, "event": "state_transition",
            "from_state": self.state.value, "to_state": new_state.value, "reason": reason, **details,
        })
        self.state = new_state
        if new_state in (MissionState.FAILED, MissionState.ABORTED):
            self.failure_reason = reason

    def _log_event(self, event: str, **details) -> None:
        self.events.append({"wall_time_s": time.time(), "sim_time_s": self._last_now_s, "event": event,
                             "mission_state": self.state.value, **details})

    # -- helpers -----------------------------------------------------------

    def _current_heading_xy(self, telem: VehicleTelemetry) -> Vec2:
        if telem.velocity_mps is not None:
            vx, vy = telem.velocity_mps[0], telem.velocity_mps[1]
            if math.hypot(vx, vy) > 1e-3:
                self._last_heading_xy = (vx, vy)
        return self._last_heading_xy

    def _vehicle_state_from_telemetry(self, telem: VehicleTelemetry, now_s: float) -> VehicleState:
        return VehicleState(
            vehicle_id=self.config.vehicle_id, sim_time_s=now_s, frame=Frame.LOCAL_ENU,
            position_m=telem.position_m, velocity_mps=telem.velocity_mps or (0.0, 0.0, 0.0),
            acceleration_mps2=(0.0, 0.0, 0.0), attitude_rad=telem.attitude_rad or (0.0, 0.0, 0.0),
            angular_velocity_radps=telem.angular_velocity_radps or (0.0, 0.0, 0.0),
            battery_fraction=telem.battery_fraction if telem.battery_fraction is not None else 1.0,
            health_state=HealthState.OK if telem.estimator_valid else HealthState.DEGRADED,
            estimator_valid=telem.estimator_valid, last_valid_command_time_s=self._last_valid_command_time_s,
        )

    def _find_or_create_cluster(self, candidate_id: str, position_m: Vec2) -> _DetectionCluster:
        for cluster in self._clusters:
            if math.dist(cluster.centroid_xy, position_m) <= self.config.confirmation_cluster_radius_m:
                return cluster
        cluster = _DetectionCluster(centroid_xy=position_m)
        self._clusters.append(cluster)
        return cluster

    def ingest_observation(self, observation: SensorObservation, now_s: float) -> Optional[_DetectionCluster]:
        """Public (not just internal) so tests can feed a hand-built,
        deliberately malformed SensorObservation directly - see
        tests/test_phase14_sar_mission.py::TestSensorAndDetection. Returns
        the cluster that just reached confirmation this call, or None."""
        if observation.vehicle_id != self.config.vehicle_id:
            self.malformed_observations += 1
            self._log_event("observation_rejected_malformed", reason="vehicle_id_mismatch")
            return None
        if observation.dropout and observation.detections:
            self.malformed_observations += 1
            self._log_event("observation_rejected_malformed", reason="dropout_with_detections")
            return None
        age_s = now_s - observation.sensor_timestamp_s
        if age_s > self.config.stale_observation_max_age_s:
            self.stale_observations_rejected += 1
            self._log_event("observation_rejected_stale", age_s=age_s)
            return None
        if observation.dropout:
            return None   # legitimate "sensor saw nothing this tick" - not an error

        newly_confirmed = None
        for d in observation.detections:
            is_duplicate = d.candidate_id in self._seen_candidate_ids
            used = (not is_duplicate) and d.confidence >= self.config.confirmation_min_confidence
            self.detection_rows.append({
                "sim_time_s": now_s, "candidate_id": d.candidate_id,
                "believed_x_m": d.position_m[0], "believed_y_m": d.position_m[1],
                "confidence": d.confidence, "localization_uncertainty_m": d.localization_uncertainty_m,
                "duplicate": is_duplicate, "used_for_confirmation": used,
            })
            if is_duplicate:
                self.duplicate_observations += 1
                self._log_event("duplicate_detection_ignored", candidate_id=d.candidate_id)
                continue
            self._seen_candidate_ids.add(d.candidate_id)
            if not used:
                continue
            self.detections_used_for_confirmation += 1
            cluster = self._find_or_create_cluster(d.candidate_id, d.position_m)
            cluster.add(d.candidate_id, d.position_m)
            if newly_confirmed is None and len(cluster.member_ids) >= self.config.confirmation_min_detections:
                newly_confirmed = cluster
        return newly_confirmed

    # -- the tick ------------------------------------------------------------

    def tick(self, now_s: float, telem: VehicleTelemetry) -> TickResult:
        self._last_now_s = now_s
        if self.state in TERMINAL_STATES:
            return TickResult()

        if self.state == MissionState.INIT:
            self._transition(MissionState.PREFLIGHT_CHECK, "mission_started")

        if self.state == MissionState.PREFLIGHT_CHECK:
            if telem.position_m is None or not telem.estimator_valid:
                self._transition(MissionState.FAILED, "preflight_check_failed")
                return TickResult()
            self._mission_start_s = now_s
            self._transition(MissionState.TAKEOFF_REQUESTED, "preflight_check_passed")

        # ---- mission-level fault checks (independent of, and in addition
        # to, SafetySupervisor's own tiered checks below) - "never silently
        # continue" after any of these (see module docstring / PHASE14 doc).
        if telem.position_m is None:
            self._transition(MissionState.FAILED, "no_position_telemetry")
            return TickResult()

        if not telem.estimator_valid:
            self._estimator_invalid_since_s = self._estimator_invalid_since_s or now_s
            if now_s - self._estimator_invalid_since_s > self.config.estimator_invalid_grace_s:
                self._transition(MissionState.FAILED, "estimator_invalid_persisted")
                return TickResult()
        else:
            self._estimator_invalid_since_s = None

        if telem.failsafe:
            self._heartbeat_lost_since_s = self._heartbeat_lost_since_s or now_s
            if now_s - self._heartbeat_lost_since_s > self.config.heartbeat_loss_grace_s:
                self._transition(MissionState.FAILED, "heartbeat_or_link_loss_persisted")
                return TickResult()
        else:
            self._heartbeat_lost_since_s = None

        if now_s - telem.timestamp_s > self.config.stale_telemetry_max_age_s:
            self._stale_telemetry_since_s = self._stale_telemetry_since_s or now_s
            if now_s - self._stale_telemetry_since_s > self.config.stale_telemetry_max_age_s:
                self._transition(MissionState.FAILED, "stale_telemetry_persisted")
                return TickResult()
        else:
            self._stale_telemetry_since_s = None

        if (self.state not in (MissionState.RETURN_HOME, MissionState.LAND_REQUESTED)
                and self._mission_start_s is not None
                and now_s - self._mission_start_s > self.config.mission_timeout_s):
            self._log_event("mission_timeout", elapsed_s=now_s - self._mission_start_s)
            self._transition(MissionState.LAND_REQUESTED, "mission_timeout_emergency_land")

        if self.state == MissionState.LAND_REQUESTED:
            self._land_requested_at_s = self._land_requested_at_s or now_s
            if now_s - self._land_requested_at_s > self.config.landing_timeout_s:
                self._transition(MissionState.FAILED, "landing_timeout_exceeded")
                return TickResult()

        # ---- bookkeeping ----
        self.max_altitude_m = max(self.max_altitude_m, telem.position_m[2])
        if telem.velocity_mps is not None:
            self.max_speed_mps = max(self.max_speed_mps, math.dist(telem.velocity_mps, (0.0, 0.0, 0.0)))
        if telem.battery_fraction is not None:
            self.min_battery_fraction = min(self.min_battery_fraction, telem.battery_fraction)
        if self.state == MissionState.SEARCHING:
            self.hold_band_altitudes_m.append(telem.position_m[2])
        self.telemetry_rows.append({
            "sim_time_s": now_s, "mission_state": self.state.value,
            "x_m": telem.position_m[0], "y_m": telem.position_m[1], "z_m": telem.position_m[2],
            "vx_mps": telem.velocity_mps[0] if telem.velocity_mps else None,
            "vy_mps": telem.velocity_mps[1] if telem.velocity_mps else None,
            "vz_mps": telem.velocity_mps[2] if telem.velocity_mps else None,
            "battery_fraction": telem.battery_fraction, "estimator_valid": telem.estimator_valid,
            "armed": telem.armed, "mode": telem.autopilot_mode.value, "failsafe": telem.failsafe,
        })

        # ---- SAR-specific: sensor observation + detection/confirmation ----
        observation = None
        if self.state in _ACTIVE_SEARCH_STATES:
            heading_xy = self._current_heading_xy(telem)
            observation = self.world.observe(telem.position_m, heading_xy, now_s)
            self.observations_generated += 1
            newly_confirmed = self.ingest_observation(observation, now_s)
            if newly_confirmed is not None and self.state != MissionState.DETECTION_CONFIRMED:
                self._confirmed_cluster = newly_confirmed
                self._transition(MissionState.DETECTION_CONFIRMED, "victim_confirmed",
                                  cluster_size=len(newly_confirmed.member_ids))
            elif observation.detections and self.state == MissionState.SEARCHING:
                self._transition(MissionState.DETECTION_CANDIDATE, "candidate_detection_observed")
            elif not observation.detections and self.state == MissionState.DETECTION_CANDIDATE:
                self._transition(MissionState.SEARCHING, "candidate_detection_not_repeated")

        if self.state == MissionState.DETECTION_CONFIRMED and self._confirmed_cluster is not None:
            matched_id = self.world.mark_found_near(self._confirmed_cluster.centroid_xy,
                                                      self.config.confirmation_cluster_radius_m)
            self.confirmed_victim_id = matched_id
            self.false_confirmation = matched_id is None
            self._log_event("victim_confirmation_scored", matched_victim_id=matched_id,
                             false_confirmation=self.false_confirmation)
            self._transition(MissionState.RETURN_HOME, "victim_confirmed_returning_home")

        # ---- state-specific waypoint target ----
        candidate_target: Vec3 = self.config.home_m
        if self.state == MissionState.TAKEOFF_REQUESTED:
            candidate_target = (self.config.home_m[0], self.config.home_m[1], self.config.search_altitude_m)
            if math.dist(telem.position_m, candidate_target) <= self.config.waypoint_reach_tolerance_m:
                self._transition(MissionState.TRANSIT, "search_altitude_reached")
                candidate_target = self.search_pattern.current_waypoint or candidate_target
        elif self.state == MissionState.TRANSIT:
            first_wp = self.search_pattern.current_waypoint
            candidate_target = first_wp if first_wp is not None else (
                self.config.home_m[0], self.config.home_m[1], self.config.search_altitude_m)
            if first_wp is not None and math.dist(telem.position_m, first_wp) <= self.config.waypoint_reach_tolerance_m:
                self._transition(MissionState.SEARCHING, "search_pattern_started")
        elif self.state in (MissionState.SEARCHING, MissionState.DETECTION_CANDIDATE):
            self.search_pattern.advance_if_reached(telem.position_m)
            segment_limit_reached = (self.config.max_search_waypoints is not None
                                      and self.search_pattern.waypoints_completed
                                      >= self.config.max_search_waypoints)
            if self.search_pattern.is_complete():
                candidate_target = (self.config.home_m[0], self.config.home_m[1], self.config.search_altitude_m)
                self._transition(MissionState.RETURN_HOME, "search_pattern_complete_no_confirmation")
            elif segment_limit_reached:
                candidate_target = (self.config.home_m[0], self.config.home_m[1], self.config.search_altitude_m)
                self._transition(MissionState.RETURN_HOME, "search_segment_limit_reached")
            else:
                candidate_target = self.search_pattern.current_waypoint

        if self.state == MissionState.RETURN_HOME:
            self._return_home_started_at_s = self._return_home_started_at_s or now_s
            candidate_target = (self.config.home_m[0], self.config.home_m[1], self.config.search_altitude_m)
            if math.dist(telem.position_m, candidate_target) <= self.config.waypoint_reach_tolerance_m:
                self._transition(MissionState.LAND_REQUESTED, "home_reached_requesting_land")

        if self.state == MissionState.LAND_REQUESTED:
            candidate_target = (self.config.home_m[0], self.config.home_m[1], self.config.ground_altitude_m)
            landed = (abs(telem.position_m[2] - self.config.ground_altitude_m) <= self.config.land_altitude_tolerance_m
                      and math.dist(telem.velocity_mps or (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
                      <= self.config.land_velocity_tolerance_mps)
            if landed:
                self._transition(MissionState.LANDED, "landing_confirmed_by_telemetry")
                return TickResult()

        if self.state in TERMINAL_STATES:
            return TickResult()

        # ---- build candidate, run it through the SafetySupervisor ----
        desired_velocity = velocity_toward(telem.position_m, candidate_target, self.config.max_speed_mps)
        candidate = CandidateCommand(
            vehicle_id=self.config.vehicle_id, desired_velocity_mps=desired_velocity, frame=Frame.LOCAL_ENU,
            timestamp_s=now_s, expiration_time_s=now_s + self.config.command_expiration_horizon_s,
            source="SARMissionLawnmowerPlanner",
        )
        own_state = self._vehicle_state_from_telemetry(telem, now_s)
        sensor_observation = observation if observation is not None else _empty_observation(self.config.vehicle_id, now_s)
        mission_context = MissionContext(
            geofence=self.geofence, operator_abort=False, contact_detected=False,
            safe_point_m=(self.config.home_m[0], self.config.home_m[1], self.config.search_altitude_m),
            own_agent_isolated=False,
        )
        decision = self.supervisor.evaluate(candidate, own_state, sensor_observation, (), mission_context, now_s)

        state_name = decision.emergency_state.value
        self.safety_state_histogram[state_name] = self.safety_state_histogram.get(state_name, 0) + 1
        if any(c.split(":")[0] == "geofence_boundary" for c in decision.active_constraints):
            self.geofence_violation_ticks += 1

        if not decision.accepted:
            self._first_rejection_at_s = self._first_rejection_at_s or now_s
            self._log_event("safety_supervisor_rejected_candidate", reason=decision.reason)
            if now_s - self._first_rejection_at_s > self.config.command_timeout_s:
                self._transition(MissionState.FAILED, "command_timeout_supervisor_rejected")
            return TickResult(candidate=candidate, decision=decision)
        self._first_rejection_at_s = None

        self._apply_safety_emergency_state(decision.emergency_state)

        self._sequence += 1
        adapter_command = AdapterCommand(command=decision.filtered_command, sequence=self._sequence)
        return TickResult(adapter_command=adapter_command, candidate=candidate, decision=decision)

    def _apply_safety_emergency_state(self, emergency_state: SafetyState) -> None:
        """Maps the SafetySupervisor's OWN tiered emergency state (which
        already produced a safe filtered_command regardless) onto a
        mission-level state transition - the supervisor remains the final
        command authority; this only decides what the MISSION does next."""
        if emergency_state in (SafetyState.LOW_BATTERY, SafetyState.RETURN_TO_SAFE_POINT,
                                SafetyState.GEOFENCE_RISK):
            if self.state in (MissionState.TRANSIT, MissionState.SEARCHING, MissionState.DETECTION_CANDIDATE):
                self._transition(MissionState.RETURN_HOME, f"safety_supervisor_{emergency_state.value.lower()}")
        elif emergency_state == SafetyState.LAND_REQUESTED and self.state != MissionState.LAND_REQUESTED:
            self._transition(MissionState.LAND_REQUESTED, "safety_supervisor_land_requested")
        elif emergency_state == SafetyState.ABORT and self.state not in TERMINAL_STATES:
            self._transition(MissionState.ABORTED, "safety_supervisor_abort")

    # -- feedback from the caller after actually sending a command ----------

    def record_command_result(self, tick_result: TickResult, adapter_result: Optional[AdapterResult],
                               now_s: float) -> None:
        """Called by the runner AFTER it (not this class) has actually
        called `adapter.send_command(tick_result.adapter_command)` - see
        module docstring for why sending lives outside tick()."""
        if tick_result.candidate is None or tick_result.decision is None:
            return
        c, d = tick_result.candidate, tick_result.decision
        self.command_rows.append({
            "sim_time_s": now_s, "mission_state": self.state.value,
            "candidate_vx": c.desired_velocity_mps[0], "candidate_vy": c.desired_velocity_mps[1],
            "candidate_vz": c.desired_velocity_mps[2],
            "filtered_command_type": d.filtered_command.command_type.value if d.filtered_command else None,
            "filtered_vx": (d.filtered_command.desired_velocity_mps[0]
                             if d.filtered_command and d.filtered_command.desired_velocity_mps else None),
            "filtered_vy": (d.filtered_command.desired_velocity_mps[1]
                             if d.filtered_command and d.filtered_command.desired_velocity_mps else None),
            "filtered_vz": (d.filtered_command.desired_velocity_mps[2]
                             if d.filtered_command and d.filtered_command.desired_velocity_mps else None),
            "adapter_accepted": adapter_result.accepted if adapter_result is not None else None,
            "adapter_reason": adapter_result.reason if adapter_result is not None else None,
            "sequence": tick_result.adapter_command.sequence if tick_result.adapter_command is not None else None,
            "command_age_s": 0.0,
        })
        self.safety_decision_rows.append({
            "sim_time_s": now_s, "mission_state": self.state.value, "accepted": d.accepted, "reason": d.reason,
            "emergency_state": d.emergency_state.value, "active_constraints": ";".join(d.active_constraints),
            "geofence_distance_m": d.geofence_distance_m,
        })
        if adapter_result is not None and adapter_result.accepted:
            self._last_valid_command_time_s = now_s

    # -- summary -------------------------------------------------------------

    def build_summary(self, *, mode: str, extra: Optional[dict] = None) -> dict:
        summary = {
            "mission_id": f"phase14-sar-{self.config.vehicle_id}-seed{self.config.seed}",
            "mode": mode,
            "seed": self.config.seed,
            "vehicle_id": self.config.vehicle_id,
            "system_id": self.config.system_id,
            "component_id": self.config.component_id,
            "final_state": self.state.value,
            "failure_reason": self.failure_reason,
            "search_area": {
                "min_x_m": self.config.search_area.min_x_m, "min_y_m": self.config.search_area.min_y_m,
                "max_x_m": self.config.search_area.max_x_m, "max_y_m": self.config.search_area.max_y_m,
            },
            "waypoint_count": self.search_pattern.waypoint_count,
            "waypoints_completed": self.search_pattern.waypoints_completed,
            "victim_ground_truth_count": self.world.victim_ground_truth_count,
            "observations_generated": self.observations_generated,
            "detections_used_for_confirmation": self.detections_used_for_confirmation,
            "duplicate_observations_ignored": self.duplicate_observations,
            "stale_observations_rejected": self.stale_observations_rejected,
            "malformed_observations_rejected": self.malformed_observations,
            "confirmed_victim_ids": list(self.world.confirmed_victim_ids),
            "confirmed_victims": len(self.world.confirmed_victim_ids),
            "false_positive_confirmations": 1 if self.false_confirmation else 0,
            "missed_victims": self.world.missed_victim_count,
            "maximum_altitude_m": self.max_altitude_m,
            "target_altitude_m": self.config.search_altitude_m,
            "altitude_overshoot_m": self.max_altitude_m - self.config.search_altitude_m,
            "hold_band_m": {
                "min_m": min(self.hold_band_altitudes_m) if self.hold_band_altitudes_m else None,
                "max_m": max(self.hold_band_altitudes_m) if self.hold_band_altitudes_m else None,
                "sample_count": len(self.hold_band_altitudes_m),
            },
            "maximum_speed_mps": self.max_speed_mps,
            "battery_minimum_fraction": self.min_battery_fraction,
            "safety_state_histogram": dict(self.safety_state_histogram),
            "geofence_violation_ticks": self.geofence_violation_ticks,
            "collisions_or_contacts": (
                "not modeled offline (FakeSITLTransport has no contact physics)" if mode == "offline" else None
            ),
            "command_rejections": sum(1 for r in self.safety_decision_rows if not r["accepted"]),
            "landing_result": self.state == MissionState.LANDED,
            "disarm_result": (
                "not_modeled_offline (FakeSITLTransport never arms - see module docstring)"
                if mode == "offline" else None
            ),
            "realtime_factor": None if mode == "offline" else None,
        }
        if extra:
            summary.update(extra)
        return summary


# --------------------------------------------------------------------------
# Offline runner
# --------------------------------------------------------------------------

_TELEMETRY_CSV_FIELDS = ("sim_time_s", "mission_state", "x_m", "y_m", "z_m", "vx_mps", "vy_mps", "vz_mps",
                          "battery_fraction", "estimator_valid", "armed", "mode", "failsafe")
_COMMAND_CSV_FIELDS = ("sim_time_s", "mission_state", "candidate_vx", "candidate_vy", "candidate_vz",
                        "filtered_command_type", "filtered_vx", "filtered_vy", "filtered_vz",
                        "adapter_accepted", "adapter_reason", "sequence", "command_age_s")
_SAFETY_CSV_FIELDS = ("sim_time_s", "mission_state", "accepted", "reason", "emergency_state",
                       "active_constraints", "geofence_distance_m")
_DETECTION_CSV_FIELDS = ("sim_time_s", "candidate_id", "believed_x_m", "believed_y_m", "confidence",
                          "localization_uncertainty_m", "duplicate", "used_for_confirmation")
_ACTUATOR_CSV_FIELDS = ("sim_time_s", "note")


def _write_csv(path: str, fieldnames: Tuple[str, ...], rows: List[dict]) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row.get(k) for k in fieldnames})


def run_sar_mission_offline(config: SARMissionConfig, out_dir: Optional[str] = None,
                             max_ticks: Optional[int] = None) -> dict:
    """Self-contained offline run: builds its own SeedManager, SARWorld,
    FakeSITLTransport + SITLAdapter, drives SARMission.tick() to
    completion (or `max_ticks`), optionally writes the full required log
    set to `out_dir`, and returns the summary dict. Never opens a socket,
    never spawns a process, never touches Webots/ArduPilot - see
    docs/PHASE14_SAR_WEBOTS.md."""
    seed_manager = SeedManager(config.seed)
    world = SARWorld(config.search_area, config.victim_positions_m, rng=seed_manager.rng("victim_sensor"),
                      sensor_config=config.sensor_config, vehicle_id=config.vehicle_id)
    supervisor = SafetySupervisor(config.build_safety_config())
    mission = SARMission(config, world, supervisor)

    transport = FakeSITLTransport(vehicle_ids=(config.vehicle_id,),
                                   rng=random.Random(seed_manager.seed_for("transport")),
                                   initial_positions={config.vehicle_id: config.home_m})
    transport.start()
    adapter = SITLAdapter(config.vehicle_id, transport.registry.namespace_of(config.vehicle_id), transport)
    adapter.connect()
    transport.step(1e-6)   # seed an initial telemetry frame at home_m before the loop reads it (SimClock.advance requires dt_s > 0)

    if max_ticks is None:
        max_ticks = int(math.ceil((config.mission_timeout_s + config.landing_timeout_s + 60.0) / config.dt_s))

    hit_max_ticks = True
    for _ in range(max_ticks):
        telem = adapter.read_telemetry()
        now_s = telem.timestamp_s
        tick_result = mission.tick(now_s, telem)
        adapter_result = None
        if tick_result.adapter_command is not None:
            adapter_result = adapter.send_command(tick_result.adapter_command)
        mission.record_command_result(tick_result, adapter_result, now_s)
        if mission.state in TERMINAL_STATES:
            hit_max_ticks = False
            break
        transport.step(config.dt_s)

    if hit_max_ticks and mission.state not in TERMINAL_STATES:
        mission._transition(MissionState.FAILED, "max_ticks_exceeded_without_terminal_state")

    adapter.disconnect()

    manifest = build_run_manifest(
        config=dataclasses.asdict(config) if dataclasses.is_dataclass(config) else config,
        master_seed=config.seed, seed_manager=seed_manager, map_id="phase14_sar_rectangular_area",
        model_id="offline_fake_sitl_kinematic", event_log=mission.events,
    )
    summary = mission.build_summary(mode="offline")

    if out_dir is not None:
        os.makedirs(out_dir, exist_ok=True)
        manifest.save_json(os.path.join(out_dir, "manifest.json"))
        with open(os.path.join(out_dir, "mission_events.jsonl"), "w") as f:
            for event in mission.events:
                f.write(json.dumps(event, default=str) + "\n")
        _write_csv(os.path.join(out_dir, "telemetry.csv"), _TELEMETRY_CSV_FIELDS, mission.telemetry_rows)
        _write_csv(os.path.join(out_dir, "safety_decisions.csv"), _SAFETY_CSV_FIELDS, mission.safety_decision_rows)
        _write_csv(os.path.join(out_dir, "commands.csv"), _COMMAND_CSV_FIELDS, mission.command_rows)
        _write_csv(os.path.join(out_dir, "detections.csv"), _DETECTION_CSV_FIELDS, mission.detection_rows)
        _write_csv(os.path.join(out_dir, "actuator_log.csv"), _ACTUATOR_CSV_FIELDS,
                   [{"sim_time_s": 0.0, "note": "not applicable offline - FakeSITLTransport has no actuator/PWM "
                                                 "model; see live mode's actuator_log.csv for real SERVO_OUTPUT_RAW"}])
        with open(os.path.join(out_dir, "summary.json"), "w") as f:
            json.dump(summary, f, indent=2, default=str)

    summary["out_dir"] = out_dir
    return summary
