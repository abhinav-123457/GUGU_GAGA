"""Phase 15D: drives ONE vehicle from an external (Swarm124-style)
scripted policy, through the SAME SafetySupervisor authority as every
other command path in this project - see
docs/PHASE15_SWARM124_LANGOSTINO_INTEGRATION.md.

Command path (never bypassed - see
tests/test_phase15d_external_policy_architecture.py):

    ScriptedSwarm124Policy.act()                  (swarm_sim.external.swarm124_policy_adapter)
        -> Swarm124Action
        -> convert_swarm124_action_to_candidate()  (swarm_sim.external.swarm124_policy_adapter -
                                                      validates + builds an untrusted CandidateCommand,
                                                      rejecting malformed/stale/expired/wrong-vehicle/
                                                      too-fast actions before the supervisor ever sees them)
        -> SafetySupervisor.evaluate()              (swarm_sim.safety_supervisor, reused unmodified)
        -> SafetyDecision.filtered_command
        -> AdapterCommand
        -> SITLAdapter.send_command()               (offline: this module's own run_external_policy_mission_offline,
                                                       against FakeSITLTransport; live:
                                                       scripts/run_phase15d_external_policy_mission.py's own
                                                       ArduPilotSITLTransport wiring, reusing Phase 13/14's
                                                       gate/arm/takeoff/land helpers - never this module)

`ExternalPolicyMission.tick()` never calls `adapter.send_command()`
itself - exactly like `SARMission.tick()` (swarm_sim/sar_mission.py), it
returns a `TickResult` and the caller sends `tick_result.adapter_command`
and reports the result back via `record_command_result()`.

This module never arms or takes off a vehicle - `tick()` assumes the
vehicle is already armed and airborne (a real, gated arm+takeoff already
happened in the caller - see the live script) and only ever produces
velocity setpoints. It has no arm/takeoff/land MAVLink logic of its own.
"""
from __future__ import annotations

import csv
import dataclasses
import enum
import json
import math
import os
import random
from typing import Dict, List, Optional, Tuple

from .autopilot.sitl import SITLAdapter
from .autopilot.types import AdapterCommand, AdapterResult, VehicleTelemetry
from .contracts import Frame, GeofenceSpec, HealthState, SafetyDecision, SafetyState, SensorObservation, VehicleState
from .external.swarm124_policy_adapter import (
    DEPTH_BEAM_COUNT, ScriptedSwarm124Policy, Swarm124Observation, convert_swarm124_action_to_candidate,
    validate_observation,
)
from .manifest import build_run_manifest
from .safety_supervisor import CandidateCommand, MissionContext, SafetySupervisor, SafetySupervisorConfig
from .sar_search import velocity_toward
from .seeding import SeedManager
from .sitl.fake_transport import FakeSITLTransport

Vec2 = Tuple[float, float]
Vec3 = Tuple[float, float, float]


class ExternalPolicyMissionState(enum.Enum):
    INIT = "INIT"
    RUNNING = "RUNNING"
    LAND_REQUESTED = "LAND_REQUESTED"
    LANDED = "LANDED"
    ABORTED = "ABORTED"
    FAILED = "FAILED"


TERMINAL_STATES = frozenset({ExternalPolicyMissionState.LANDED, ExternalPolicyMissionState.ABORTED,
                              ExternalPolicyMissionState.FAILED})

ALLOWED_TRANSITIONS: Dict[ExternalPolicyMissionState, frozenset] = {
    ExternalPolicyMissionState.INIT: frozenset({ExternalPolicyMissionState.RUNNING,
                                                 ExternalPolicyMissionState.FAILED,
                                                 ExternalPolicyMissionState.ABORTED}),
    ExternalPolicyMissionState.RUNNING: frozenset({ExternalPolicyMissionState.LAND_REQUESTED,
                                                    ExternalPolicyMissionState.FAILED,
                                                    ExternalPolicyMissionState.ABORTED}),
    ExternalPolicyMissionState.LAND_REQUESTED: frozenset({ExternalPolicyMissionState.LANDED,
                                                           ExternalPolicyMissionState.FAILED,
                                                           ExternalPolicyMissionState.ABORTED}),
    ExternalPolicyMissionState.LANDED: frozenset(),
    ExternalPolicyMissionState.ABORTED: frozenset(),
    ExternalPolicyMissionState.FAILED: frozenset(),
}


@dataclasses.dataclass
class ExternalPolicyMissionConfig:
    vehicle_id: str = "drone0"
    namespace: str = "sitl/drone0"
    system_id: int = 1
    component_id: int = 1
    seed: int = 0
    run_id: str = "phase15d-run0"

    home_m: Vec3 = (0.0, 0.0, 0.0)
    search_altitude_m: float = 1.0
    target_xy: Vec2 = (5.0, 5.0)
    goal_reach_tolerance_m: float = 0.5

    max_speed_mps: float = 0.25
    max_accel_mps2: float = 0.5
    action_expiry_horizon_s: float = 2.0
    max_action_staleness_s: float = 2.0

    dt_s: float = 1.0
    mission_timeout_s: float = 60.0
    command_timeout_s: float = 5.0
    landing_timeout_s: float = 30.0
    estimator_invalid_grace_s: float = 2.0
    heartbeat_loss_grace_s: float = 2.0
    stale_telemetry_max_age_s: float = 2.0
    land_altitude_tolerance_m: float = 0.15
    land_velocity_tolerance_mps: float = 0.05
    ground_altitude_m: float = 0.0

    geofence_margin_m: float = 1.0
    geofence_floor_alt_m: float = -2.0
    geofence_ceiling_alt_m: float = 2.0

    safety_config: Optional[SafetySupervisorConfig] = None

    def build_safety_config(self) -> SafetySupervisorConfig:
        if self.safety_config is not None:
            return self.safety_config
        # Same two live-testing-driven fixes as SARMissionConfig
        # (swarm_sim/sar_mission.py) - fallback_dt_s and link_timeout_s
        # both default to values tuned for a much faster swarm-sim loop,
        # not this mission's own ~1Hz live tick rate. Scaled the same way
        # here rather than re-deriving (and possibly forgetting) either
        # fix for this second live mission type - see docs/PHASE14_SAR_WEBOTS.md.
        return SafetySupervisorConfig(max_speed_mps=self.max_speed_mps, geofence_margin_m=self.geofence_margin_m,
                                       fallback_dt_s=self.dt_s, link_timeout_s=2.0 * self.dt_s)

    def build_geofence(self) -> GeofenceSpec:
        """Encloses both home_m and target_xy, each with a doubled buffer -
        see SARMissionConfig.build_geofence()'s own docstring
        (swarm_sim/sar_mission.py) for why a single-margin buffer places
        home exactly on SafetySupervisor's own GEOFENCE_RISK threshold
        instead of strictly inside it."""
        buffer_m = 2.0 * self.geofence_margin_m
        min_x = min(self.home_m[0], self.target_xy[0]) - buffer_m
        max_x = max(self.home_m[0], self.target_xy[0]) + buffer_m
        min_y = min(self.home_m[1], self.target_xy[1]) - buffer_m
        max_y = max(self.home_m[1], self.target_xy[1]) + buffer_m
        return GeofenceSpec(
            frame=Frame.LOCAL_ENU, center_m=((min_x + max_x) / 2.0, (min_y + max_y) / 2.0),
            half_extents_m=((max_x - min_x) / 2.0, (max_y - min_y) / 2.0),
            floor_alt_m=self.geofence_floor_alt_m, ceiling_alt_m=self.geofence_ceiling_alt_m,
        )


@dataclasses.dataclass
class TickResult:
    adapter_command: Optional[AdapterCommand] = None
    candidate: Optional[CandidateCommand] = None
    decision: Optional[SafetyDecision] = None
    external_rejected: bool = False


def _empty_observation(vehicle_id: str, now_s: float) -> SensorObservation:
    return SensorObservation(vehicle_id=vehicle_id, sensor_timestamp_s=now_s, sensor_latency_s=0.0,
                              fov_deg=360.0, range_returns_m=(), occluded=(), dropout=True,
                              pose_uncertainty_m=0.0, detections=())


class ExternalPolicyMission:
    """One-vehicle mission driven by an external policy instead of this
    project's own SAR search pattern - see module docstring for the full
    command path. Vehicle-agnostic to its telemetry source exactly like
    `SARMission` (swarm_sim/sar_mission.py): `tick()` only ever reads a
    `VehicleTelemetry`, so the identical logic drives both
    `run_external_policy_mission_offline` (below) and the live runner in
    scripts/run_phase15d_external_policy_mission.py."""

    def __init__(self, config: ExternalPolicyMissionConfig, policy: ScriptedSwarm124Policy,
                 supervisor: Optional[SafetySupervisor] = None):
        self.config = config
        self.policy = policy
        self.supervisor = supervisor or SafetySupervisor(config.build_safety_config())
        self.geofence = config.build_geofence()

        self.state = ExternalPolicyMissionState.INIT
        self.events: List[dict] = []
        self.telemetry_rows: List[dict] = []
        self.command_rows: List[dict] = []
        self.safety_decision_rows: List[dict] = []

        self.failure_reason: Optional[str] = None
        self.goal_reached = False

        self._sequence = 0
        self._mission_start_s: Optional[float] = None
        self._last_now_s = 0.0
        self._last_valid_command_time_s: Optional[float] = None
        self._estimator_invalid_since_s: Optional[float] = None
        self._heartbeat_lost_since_s: Optional[float] = None
        self._stale_telemetry_since_s: Optional[float] = None
        self._land_requested_at_s: Optional[float] = None
        self._first_rejection_at_s: Optional[float] = None
        self._last_policy_velocity_mps: Optional[Vec3] = None
        self._last_policy_timestamp_s: Optional[float] = None

        self.policy_actions_generated = 0
        self.external_rejections = 0
        self.malformed_observations_rejected = 0
        self.max_altitude_m = 0.0
        self.max_speed_mps = 0.0
        self.min_battery_fraction = 1.0
        self.safety_state_histogram: Dict[str, int] = {}
        self.geofence_violation_ticks = 0

    # -- helpers -------------------------------------------------------------

    def _transition(self, new_state: ExternalPolicyMissionState, reason: str, **details) -> None:
        allowed = ALLOWED_TRANSITIONS.get(self.state, frozenset())
        if new_state not in allowed:
            raise ValueError(f"illegal mission-state transition {self.state.value} -> {new_state.value} "
                              f"(reason={reason!r})")
        self.events.append({
            "wall_time_s": self._last_now_s, "sim_time_s": self._last_now_s, "event": "state_transition",
            "from_state": self.state.value, "to_state": new_state.value, "reason": reason, **details,
        })
        self.state = new_state
        if new_state in (ExternalPolicyMissionState.FAILED, ExternalPolicyMissionState.ABORTED):
            self.failure_reason = reason

    def _log_event(self, event: str, **details) -> None:
        self.events.append({"sim_time_s": self._last_now_s, "event": event,
                             "mission_state": self.state.value, **details})

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

    def _apply_safety_emergency_state(self, emergency_state: SafetyState) -> None:
        """Mirrors SARMission's own `_apply_safety_emergency_state` - the
        supervisor's own filtered_command is already safe regardless; this
        only decides what the MISSION does next. Deliberately landS IN
        PLACE (LAND_REQUESTED) rather than a separate return-home leg for
        every emergency tier, not just LOW_BATTERY - Phase 15D's own scope
        is validating the external-policy command path, not a full SAR
        return journey, so the simplest safe response to "something is
        wrong" is to stop advancing toward the target and land."""
        if emergency_state in (SafetyState.LOW_BATTERY, SafetyState.RETURN_TO_SAFE_POINT,
                                SafetyState.GEOFENCE_RISK, SafetyState.DEGRADED_LINK):
            if self.state == ExternalPolicyMissionState.RUNNING:
                self._transition(ExternalPolicyMissionState.LAND_REQUESTED,
                                  f"safety_supervisor_{emergency_state.value.lower()}")
        elif emergency_state == SafetyState.LAND_REQUESTED and self.state != ExternalPolicyMissionState.LAND_REQUESTED:
            self._transition(ExternalPolicyMissionState.LAND_REQUESTED, "safety_supervisor_land_requested")
        elif emergency_state == SafetyState.ABORT and self.state not in TERMINAL_STATES:
            self._transition(ExternalPolicyMissionState.ABORTED, "safety_supervisor_abort")

    # -- the tick --------------------------------------------------------------

    def tick(self, now_s: float, telem: VehicleTelemetry) -> TickResult:
        self._last_now_s = now_s
        if self.state in TERMINAL_STATES:
            return TickResult()

        if self.state == ExternalPolicyMissionState.INIT:
            if telem.position_m is None or not telem.estimator_valid:
                self._transition(ExternalPolicyMissionState.FAILED, "preflight_check_failed")
                return TickResult()
            self._mission_start_s = now_s
            self._transition(ExternalPolicyMissionState.RUNNING, "policy_mission_started")

        # ---- mission-level fault checks - same "never silently continue"
        # pattern as SARMission (swarm_sim/sar_mission.py) ----
        if telem.position_m is None:
            self._transition(ExternalPolicyMissionState.FAILED, "no_position_telemetry")
            return TickResult()

        if not telem.estimator_valid:
            # `is None`, never `or` - now_s can legitimately be exactly
            # 0.0 (the live script starts its sim clock at 0.0), and 0.0
            # is falsy in Python, so `self._X or now_s` would silently
            # reset an already-set "since" timestamp to the CURRENT now_s
            # every tick instead of preserving it, permanently defeating
            # this grace-period check (found while writing this mission's
            # own tests - see docs/PHASE15_SWARM124_LANGOSTINO_INTEGRATION.md;
            # the identical latent bug was also fixed in SARMission,
            # swarm_sim/sar_mission.py). Applies to every "_since_s"/
            # "_at_s" timer in this class.
            if self._estimator_invalid_since_s is None:
                self._estimator_invalid_since_s = now_s
            if now_s - self._estimator_invalid_since_s > self.config.estimator_invalid_grace_s:
                self._transition(ExternalPolicyMissionState.FAILED, "estimator_invalid_persisted")
                return TickResult()
        else:
            self._estimator_invalid_since_s = None

        if telem.failsafe:
            if self._heartbeat_lost_since_s is None:
                self._heartbeat_lost_since_s = now_s
            if now_s - self._heartbeat_lost_since_s > self.config.heartbeat_loss_grace_s:
                self._transition(ExternalPolicyMissionState.FAILED, "heartbeat_or_link_loss_persisted")
                return TickResult()
        else:
            self._heartbeat_lost_since_s = None

        if now_s - telem.timestamp_s > self.config.stale_telemetry_max_age_s:
            if self._stale_telemetry_since_s is None:
                self._stale_telemetry_since_s = now_s
            if now_s - self._stale_telemetry_since_s > self.config.stale_telemetry_max_age_s:
                self._transition(ExternalPolicyMissionState.FAILED, "stale_telemetry_persisted")
                return TickResult()
        else:
            self._stale_telemetry_since_s = None

        if (self.state == ExternalPolicyMissionState.RUNNING and self._mission_start_s is not None
                and now_s - self._mission_start_s > self.config.mission_timeout_s):
            self._log_event("mission_timeout", elapsed_s=now_s - self._mission_start_s)
            self._transition(ExternalPolicyMissionState.LAND_REQUESTED, "mission_timeout_emergency_land")

        if self.state == ExternalPolicyMissionState.LAND_REQUESTED:
            if self._land_requested_at_s is None:
                self._land_requested_at_s = now_s
            if now_s - self._land_requested_at_s > self.config.landing_timeout_s:
                self._transition(ExternalPolicyMissionState.FAILED, "landing_timeout_exceeded")
                return TickResult()

        # ---- bookkeeping ----
        self.max_altitude_m = max(self.max_altitude_m, telem.position_m[2])
        if telem.velocity_mps is not None:
            self.max_speed_mps = max(self.max_speed_mps, math.dist(telem.velocity_mps, (0.0, 0.0, 0.0)))
        if telem.battery_fraction is not None:
            self.min_battery_fraction = min(self.min_battery_fraction, telem.battery_fraction)
        self.telemetry_rows.append({
            "sim_time_s": now_s, "mission_state": self.state.value,
            "x_m": telem.position_m[0], "y_m": telem.position_m[1], "z_m": telem.position_m[2],
            "vx_mps": telem.velocity_mps[0] if telem.velocity_mps else None,
            "vy_mps": telem.velocity_mps[1] if telem.velocity_mps else None,
            "vz_mps": telem.velocity_mps[2] if telem.velocity_mps else None,
            "battery_fraction": telem.battery_fraction, "estimator_valid": telem.estimator_valid,
            "armed": telem.armed, "mode": telem.autopilot_mode.value, "failsafe": telem.failsafe,
        })

        if self.state == ExternalPolicyMissionState.RUNNING:
            dist_to_target = math.dist((telem.position_m[0], telem.position_m[1]), self.config.target_xy)
            if dist_to_target <= self.config.goal_reach_tolerance_m:
                self.goal_reached = True
                self._transition(ExternalPolicyMissionState.LAND_REQUESTED, "policy_target_reached")

        if self.state == ExternalPolicyMissionState.LAND_REQUESTED:
            candidate_target = (self.config.home_m[0], self.config.home_m[1], self.config.ground_altitude_m)
            landed = (abs(telem.position_m[2] - self.config.ground_altitude_m) <= self.config.land_altitude_tolerance_m
                      and math.dist(telem.velocity_mps or (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
                      <= self.config.land_velocity_tolerance_mps)
            if landed:
                self._transition(ExternalPolicyMissionState.LANDED, "landing_confirmed_by_telemetry")
                return TickResult()

        if self.state in TERMINAL_STATES:
            return TickResult()

        # ---- build a candidate: from the external policy while RUNNING,
        # or a plain hover-descend-home target once LAND_REQUESTED (the
        # external policy is never consulted again once something has
        # asked this mission to land - see _apply_safety_emergency_state) ----
        external_rejected = False
        policy_row_fields = {"policy_direction_x": None, "policy_direction_y": None, "policy_speed_mps": None,
                              "external_accepted": None, "external_rejection_reason": None,
                              "policy_version": None, "model_checksum": None}
        if self.state == ExternalPolicyMissionState.RUNNING:
            observation = Swarm124Observation(
                vehicle_id=self.config.vehicle_id, timestamp_s=now_s, position_m=telem.position_m,
                velocity_mps=telem.velocity_mps or (0.0, 0.0, 0.0),
                depth_returns_m=tuple(50.0 for _ in range(DEPTH_BEAM_COUNT)),  # no obstacle sensing this phase
                teammate_relative_m=(), seed=self.config.seed, run_id=self.config.run_id,
            )
            obs_error = validate_observation(observation)
            if obs_error is not None:
                self.malformed_observations_rejected += 1
                self._log_event("malformed_observation_rejected", detail=obs_error)
                return TickResult()

            action = self.policy.act(observation, action_expiry_s=now_s + self.config.action_expiry_horizon_s)
            self.policy_actions_generated += 1
            policy_row_fields.update({
                "policy_direction_x": action.direction_xy[0], "policy_direction_y": action.direction_xy[1],
                "policy_speed_mps": action.speed_mps, "policy_version": action.policy_version,
                "model_checksum": action.model_checksum,
            })
            external_result = convert_swarm124_action_to_candidate(
                action, known_vehicle_ids=(self.config.vehicle_id,), now_s=now_s,
                max_speed_mps=self.config.max_speed_mps, max_accel_mps2=self.config.max_accel_mps2,
                max_staleness_s=self.config.max_action_staleness_s,
                previous_velocity_mps=self._last_policy_velocity_mps,
                previous_timestamp_s=self._last_policy_timestamp_s,
            )
            self._last_policy_timestamp_s = now_s
            policy_row_fields["external_accepted"] = external_result.accepted
            if not external_result.accepted:
                external_rejected = True
                self.external_rejections += 1
                if self._first_rejection_at_s is None:
                    self._first_rejection_at_s = now_s
                policy_row_fields["external_rejection_reason"] = (
                    external_result.rejection.value if external_result.rejection else None)
                self._log_event("external_policy_action_rejected",
                                 reason=policy_row_fields["external_rejection_reason"], detail=external_result.detail)
                self.command_rows.append({"sim_time_s": now_s, "mission_state": self.state.value,
                                           **policy_row_fields, "candidate_vx": None, "candidate_vy": None,
                                           "candidate_vz": None, "filtered_vx": None, "filtered_vy": None,
                                           "filtered_vz": None, "safety_accepted": None, "safety_reason": None,
                                           "adapter_accepted": None, "sequence": None})
                if now_s - self._first_rejection_at_s > self.config.command_timeout_s:
                    self._transition(ExternalPolicyMissionState.FAILED, "command_timeout_external_rejected")
                return TickResult(external_rejected=True)
            self._last_policy_velocity_mps = external_result.candidate.desired_velocity_mps
            candidate = external_result.candidate
        else:
            # Only reachable when self.state == LAND_REQUESTED (RUNNING is
            # handled above, and TERMINAL_STATES already returned) - the
            # LAND_REQUESTED block above always sets candidate_target
            # before this point is ever reached in that state.
            desired_velocity = velocity_toward(telem.position_m, candidate_target, self.config.max_speed_mps)
            candidate = CandidateCommand(
                vehicle_id=self.config.vehicle_id, desired_velocity_mps=desired_velocity, frame=Frame.LOCAL_ENU,
                timestamp_s=now_s, expiration_time_s=now_s + 2.0, source="ExternalPolicyMissionLandFallback",
            )

        own_state = self._vehicle_state_from_telemetry(telem, now_s)
        sensor_observation = _empty_observation(self.config.vehicle_id, now_s)
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

        self.command_rows.append({
            "sim_time_s": now_s, "mission_state": self.state.value, **policy_row_fields,
            "candidate_vx": candidate.desired_velocity_mps[0], "candidate_vy": candidate.desired_velocity_mps[1],
            "candidate_vz": candidate.desired_velocity_mps[2],
            "filtered_vx": (decision.filtered_command.desired_velocity_mps[0]
                             if decision.filtered_command and decision.filtered_command.desired_velocity_mps
                             else None),
            "filtered_vy": (decision.filtered_command.desired_velocity_mps[1]
                             if decision.filtered_command and decision.filtered_command.desired_velocity_mps
                             else None),
            "filtered_vz": (decision.filtered_command.desired_velocity_mps[2]
                             if decision.filtered_command and decision.filtered_command.desired_velocity_mps
                             else None),
            "safety_accepted": decision.accepted, "safety_reason": decision.reason,
            "adapter_accepted": None, "sequence": None,
        })
        self.safety_decision_rows.append({
            "sim_time_s": now_s, "mission_state": self.state.value, "accepted": decision.accepted,
            "reason": decision.reason, "emergency_state": decision.emergency_state.value,
            "active_constraints": ";".join(decision.active_constraints),
            "geofence_distance_m": decision.geofence_distance_m,
        })

        if not decision.accepted:
            if self._first_rejection_at_s is None:
                self._first_rejection_at_s = now_s
            self._log_event("safety_supervisor_rejected_candidate", reason=decision.reason)
            if now_s - self._first_rejection_at_s > self.config.command_timeout_s:
                self._transition(ExternalPolicyMissionState.FAILED, "command_timeout_supervisor_rejected")
            return TickResult(candidate=candidate, decision=decision, external_rejected=external_rejected)
        self._first_rejection_at_s = None

        self._apply_safety_emergency_state(decision.emergency_state)

        self._sequence += 1
        adapter_command = AdapterCommand(command=decision.filtered_command, sequence=self._sequence)
        self.command_rows[-1]["sequence"] = self._sequence
        return TickResult(adapter_command=adapter_command, candidate=candidate, decision=decision,
                           external_rejected=external_rejected)

    def record_command_result(self, tick_result: TickResult, adapter_result: Optional[AdapterResult],
                               now_s: float) -> None:
        """Called by the runner AFTER it (not this class) has actually
        called `adapter.send_command(tick_result.adapter_command)` -
        mirrors SARMission.record_command_result()'s own contract exactly."""
        if tick_result.adapter_command is None or not self.command_rows:
            return
        self.command_rows[-1]["adapter_accepted"] = adapter_result.accepted if adapter_result is not None else None
        if adapter_result is not None and adapter_result.accepted:
            self._last_valid_command_time_s = now_s

    def build_summary(self, *, mode: str, extra: Optional[dict] = None) -> dict:
        summary = {
            "mission_id": f"phase15d-external-policy-{self.config.vehicle_id}-seed{self.config.seed}",
            "mode": mode,
            "seed": self.config.seed,
            "run_id": self.config.run_id,
            "vehicle_id": self.config.vehicle_id,
            "system_id": self.config.system_id,
            "component_id": self.config.component_id,
            "final_state": self.state.value,
            "failure_reason": self.failure_reason,
            "target_xy": list(self.config.target_xy),
            "goal_reached": self.goal_reached,
            "policy_version": self.policy.policy_version,
            "model_checksum": self.policy.model_checksum,
            "policy_actions_generated": self.policy_actions_generated,
            "external_rejections": self.external_rejections,
            "malformed_observations_rejected": self.malformed_observations_rejected,
            "command_rejections": sum(1 for r in self.safety_decision_rows if not r["accepted"]),
            "maximum_altitude_m": self.max_altitude_m,
            "target_altitude_m": self.config.search_altitude_m,
            "maximum_speed_mps": self.max_speed_mps,
            "battery_minimum_fraction": self.min_battery_fraction,
            "safety_state_histogram": dict(self.safety_state_histogram),
            "geofence_violation_ticks": self.geofence_violation_ticks,
            "collisions_or_contacts": (
                "not modeled offline (FakeSITLTransport has no contact physics)" if mode == "offline" else None
            ),
            "landing_result": self.state == ExternalPolicyMissionState.LANDED,
            "disarm_result": (
                "not_modeled_offline (FakeSITLTransport never arms - see module docstring)"
                if mode == "offline" else None
            ),
            "realtime_factor": None,
        }
        if extra:
            summary.update(extra)
        return summary


# --------------------------------------------------------------------------
# Offline runner
# --------------------------------------------------------------------------

_TELEMETRY_CSV_FIELDS = ("sim_time_s", "mission_state", "x_m", "y_m", "z_m", "vx_mps", "vy_mps", "vz_mps",
                          "battery_fraction", "estimator_valid", "armed", "mode", "failsafe")
_COMMAND_CSV_FIELDS = ("sim_time_s", "mission_state", "policy_direction_x", "policy_direction_y",
                        "policy_speed_mps", "external_accepted", "external_rejection_reason", "policy_version",
                        "model_checksum", "candidate_vx", "candidate_vy", "candidate_vz", "filtered_vx",
                        "filtered_vy", "filtered_vz", "safety_accepted", "safety_reason", "adapter_accepted",
                        "sequence")
_SAFETY_CSV_FIELDS = ("sim_time_s", "mission_state", "accepted", "reason", "emergency_state",
                       "active_constraints", "geofence_distance_m")


def _write_csv(path: str, fieldnames: Tuple[str, ...], rows: List[dict]) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row.get(k) for k in fieldnames})


def run_external_policy_mission_offline(config: ExternalPolicyMissionConfig, out_dir: Optional[str] = None,
                                         max_ticks: Optional[int] = None) -> dict:
    """Self-contained offline run: builds its own SeedManager,
    FakeSITLTransport + SITLAdapter, and ScriptedSwarm124Policy, drives
    ExternalPolicyMission.tick() to completion (or max_ticks), optionally
    writes the full log set to out_dir, and returns the summary dict.
    Never opens a socket, never spawns a process, never touches
    Webots/ArduPilot - mirrors run_sar_mission_offline() exactly
    (swarm_sim/sar_mission.py)."""
    seed_manager = SeedManager(config.seed)
    supervisor = SafetySupervisor(config.build_safety_config())
    policy = ScriptedSwarm124Policy(target_xy=config.target_xy, max_speed_mps=config.max_speed_mps,
                                     seed=config.seed, run_id=config.run_id)
    mission = ExternalPolicyMission(config, policy, supervisor)

    # Starts already at search_altitude_m, not home_m's ground level - this
    # mission (live and offline alike) always assumes a real arm+takeoff
    # already happened before its own tick loop starts (see module
    # docstring: "This module never arms or takes off a vehicle"), so the
    # offline stand-in should start from the same already-airborne
    # precondition rather than silently exercising a ground-level (never
    # actually flying) command path.
    initial_position = (config.home_m[0], config.home_m[1], config.search_altitude_m)
    transport = FakeSITLTransport(vehicle_ids=(config.vehicle_id,),
                                   rng=random.Random(seed_manager.seed_for("transport")),
                                   initial_positions={config.vehicle_id: initial_position})
    transport.start()
    adapter = SITLAdapter(config.vehicle_id, transport.registry.namespace_of(config.vehicle_id), transport)
    adapter.connect()
    transport.step(1e-6)

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
        mission._transition(ExternalPolicyMissionState.FAILED, "max_ticks_exceeded_without_terminal_state")

    adapter.disconnect()

    manifest = build_run_manifest(
        config=dataclasses.asdict(config) if dataclasses.is_dataclass(config) else config,
        master_seed=config.seed, seed_manager=seed_manager, map_id="phase15d_external_policy_point_to_point",
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
        with open(os.path.join(out_dir, "summary.json"), "w") as f:
            json.dump(summary, f, indent=2, default=str)

    return summary
