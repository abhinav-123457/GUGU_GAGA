"""Phase 4: an independent safety supervisor sitting between candidate-
command generation (swarm_sim.controller.SwarmController, and eventually
any other planner) and the vehicle/PyBullet command interface. See
docs/PHASE4_SAFETY.md for the full state-transition diagram, priority
policy, and documented assumptions.

    Sensor observations
          |
          v
    SwarmController candidate command   <- CandidateCommand (untrusted, unvalidated)
          |
          v
    SafetySupervisor.evaluate()         <- THIS MODULE
          |
          v
    SafetyDecision (accepted Command / HOLD / RETURN_TO_SAFE_POINT / LAND / ABORT / rejected)
          |
          v
    PyBullet or future autopilot adapter

THIS IS NOT A CERTIFIED COLLISION-AVOIDANCE SYSTEM. It is a simulation-
scoped safety layer: conservative heuristics over noisy, latent, dropout-
prone sensor data (Phase 2), evaluated once per control step. It has not
been verified against any airworthiness standard, has no formal proof of
correctness, and every numeric threshold in SafetySupervisorConfig is a
documented engineering choice, not a certified limit. See "Assumptions"
in docs/PHASE4_SAFETY.md before reading anything here as a safety
guarantee.

Independence from the behavior planner (swarm_sim/behaviors/*.py,
swarm_sim/controller.py): this module imports nothing from either, is
called as a separate step after SwarmController.step() returns (never
from inside it), and its own output type (a validated
swarm_sim.contracts.Command, or a REJECTED SafetyDecision with no
command) is the only thing that ever reaches PyBullet. It never sees
victim ground truth, and it never uses the past to relax evaluation -
ConsensusBoard's confirmations reach this module only as they always
did, through RecruitmentBoard-mediated committed beacons, and cannot
change what evaluate() decides about a candidate command's own physical
safety.

Why CandidateCommand is a separate, unvalidated type from
swarm_sim.contracts.Command: Command.__post_init__ already validates
finiteness at construction time - a NaN velocity would crash the mission
before ever reaching a safety check. CandidateCommand carries the same
information without that validation, specifically so this module's own
"reject malformed/non-finite" check has something real to catch. Every
Command this module CONSTRUCTS (as SafetyDecision.filtered_command) does
go through swarm_sim.contracts.Command's full validation - nothing
invalid can leave this module, whatever came in.
"""
from __future__ import annotations

import dataclasses
import math
from collections import deque
from typing import Dict, List, Optional, Tuple

from .contracts import (
    Command, CommandType, Frame, GeofenceSpec, HealthState, NeighborObservation,
    SafetyDecision, SafetyState, SensorObservation, Vec3, VehicleState,
)

# --------------------------------------------------------------------------
# Priority tier numbers (see docs/PHASE4_SAFETY.md's priority policy) and
# a pure function to recover the tier that produced a given SafetyDecision
# from its active_constraints - used both internally (hysteresis, Phase
# 4.1) and externally (mission.py's per-tick safety-transition telemetry,
# also Phase 4.1: "add per-tick safety-transition telemetry... override
# tier"). Kept as a derivation rather than a new SafetyDecision field so
# the contract wasn't extended again just for this.
# --------------------------------------------------------------------------

TIER_OPERATOR_ABORT = 1
TIER_GEOFENCE = 2
TIER_ALTITUDE_CRITICAL = 2.5
TIER_SEPARATION = 3
TIER_OBSTACLE = 4
TIER_ALTITUDE = 5
TIER_BATTERY = 6
TIER_LINK = 7
TIER_ESTIMATOR = 8
TIER_MISSION_CANDIDATE = 9

_CONSTRAINT_TO_TIER = {
    "operator_abort": TIER_OPERATOR_ABORT,
    "geofence_boundary": TIER_GEOFENCE,
    "altitude_floor_critical": TIER_ALTITUDE_CRITICAL,
    "separation_risk": TIER_SEPARATION,
    "obstacle_clearance": TIER_OBSTACLE,
    "obstacle_stuck": TIER_OBSTACLE,
    "altitude_floor": TIER_ALTITUDE,
    "altitude_ceiling": TIER_ALTITUDE,
    "low_battery": TIER_BATTERY,
    "battery_critical": TIER_BATTERY,
    "degraded_link": TIER_LINK,
    "lost_agent": TIER_LINK,
    "estimator_invalid": TIER_ESTIMATOR,
}


def tier_of(active_constraints: Tuple[str, ...]) -> Optional[int]:
    """The lowest (highest-priority) tier number any of `active_constraints`
    maps to, or TIER_MISSION_CANDIDATE if none match a known override
    constraint (the nominal/no-override case), or None for an empty tuple
    with nothing else to go on (e.g. a rejected decision)."""
    if not active_constraints:
        return None
    tiers = [TIER_MISSION_CANDIDATE]
    for c in active_constraints:
        key = c.split(":")[0]
        if key in _CONSTRAINT_TO_TIER:
            tiers.append(_CONSTRAINT_TO_TIER[key])
    return min(tiers)


# --------------------------------------------------------------------------
# Untrusted input types
# --------------------------------------------------------------------------


@dataclasses.dataclass(frozen=True)
class CandidateCommand:
    """The untrusted, unvalidated output of a behavior planner (today,
    swarm_sim.controller.SwarmController.step()). Deliberately NOT
    swarm_sim.contracts.Command - see the module docstring. May legally
    contain NaN/Inf components, an already-expired timestamp, or a
    wrong-shaped velocity; SafetySupervisor.evaluate() is exactly what is
    responsible for catching that, not this constructor."""
    vehicle_id: str
    desired_velocity_mps: Tuple[float, float, float]
    frame: Frame
    timestamp_s: float
    expiration_time_s: float
    source: str = "SwarmController"


@dataclasses.dataclass(frozen=True)
class MissionContext:
    """Everything evaluate() needs that isn't the candidate command, this
    vehicle's own state, or its sensor readings - all evaluation/operator
    inputs, never victim ground truth. `contact_detected` is an actual
    PyBullet contact-physics result (see mission.py's existing
    _classify_and_log_contacts) fed in as a real physical event a real
    drone's IMU/collision sensor could plausibly detect - this is NOT the
    same category as victim ground truth (which this module never
    receives at all, by construction: no field here or anywhere in this
    module's signature carries victim positions)."""
    geofence: GeofenceSpec
    operator_abort: bool = False
    contact_detected: bool = False
    safe_point_m: Optional[Vec3] = None
    # Whether THIS vehicle is currently isolated from the rest of the
    # swarm's communication graph (network.connected_component_sizes()
    # says its own component has size 1) - mission-computed from the same
    # already-comms-realistic CommsNetwork data used for
    # swarm_connectivity_fraction, NOT the short-range onboard proximity
    # sensor in neighbor_observations. Using the onboard sensor for this
    # was tried and rejected: its range (a few meters, by design, for
    # collision avoidance only) makes "no proximity-sensor neighbor right
    # now" completely routine during normal search dispersal, which
    # produced a large false-positive LOST_AGENT rate - see
    # docs/PHASE4_SAFETY.md's "Assumptions"/lessons-learned note.
    own_agent_isolated: bool = False


# --------------------------------------------------------------------------
# Safety event log schema
# --------------------------------------------------------------------------


@dataclasses.dataclass(frozen=True)
class SafetyEvent:
    """One row of the safety event log - every command rejection and every
    state-changing override is logged as one of these (docs/PHASE4_SAFETY.md
    calls this out as a required deliverable: "a safety event log
    schema"). `details` is a small, JSON-serializable dict of whatever
    numbers motivated this event (e.g. clearance figures) - kept generic
    rather than one field per possible cause, since different event types
    carry different relevant numbers."""
    sim_time_s: float
    vehicle_id: str
    event_type: str          # "command_rejected" | "command_overridden" | "state_transition" | "stuck_detected" | "recovery"
    from_state: str
    to_state: str
    reason: str
    details: Dict[str, float] = dataclasses.field(default_factory=dict)

    def to_dict(self) -> dict:
        return dataclasses.asdict(self)


# --------------------------------------------------------------------------
# Configuration - every threshold below is an engineering choice, not a
# certified limit. See docs/PHASE4_SAFETY.md's "Assumptions" section.
# --------------------------------------------------------------------------


@dataclasses.dataclass
class SafetySupervisorConfig:
    # Independent bounds - deliberately this module's OWN config, not a
    # read of MissionConfig/SwarmController's tuning, so a planner change
    # can never implicitly loosen what the supervisor enforces.
    max_speed_mps: float = 5.0
    max_accel_mps2: float = 4.0
    max_turn_rate_radps: float = 3.0

    # Command handling.
    command_latency_s: float = 0.15   # assumed delay before a command takes effect - lookahead uses this
    assumed_brake_decel_mps2: float = 3.0   # assumed max deceleration capability for stopping-distance predictions

    # Vehicle geometry (CF2X body radius - see mission.py's DRONE_BODY_RADIUS_M;
    # duplicated here rather than imported so this module has zero import-time
    # coupling to mission.py, per the "independent" requirement).
    vehicle_body_radius_m: float = 0.0397 + 0.023135

    # Separation (inter-drone).
    separation_hard_margin_m: float = 0.3      # body clearance below this -> SEPARATION_RISK override
    separation_uncertainty_inflation_m: float = 0.5
    separation_stale_inflation_multiplier: float = 2.0   # extra caution for a stale neighbor reading
    neighbor_watch_window_s: float = 1.0        # how long a vanished neighbor is still treated as "might be close"

    # Obstacles. obstacle_uncertainty_inflation_m is deliberately larger
    # than separation's own margin - see docs/PHASE4_SAFETY.md's
    # "Assumptions": this simulator's DSLPIDControl has its own velocity-
    # tracking lag on top of this module's own acceleration-bounded
    # command ramp, which the predicted_travel_distance_m lookahead does
    # not model (that would require system-identifying the PID's closed-
    # loop response, out of scope) - the extra inflation is an empirical,
    # documented buffer for that unmodeled lag, not a derived quantity.
    obstacle_hard_margin_m: float = 0.3
    obstacle_uncertainty_inflation_m: float = 1.0

    # Stuck/wedging detection (Phase 2 obstacle-wedging response).
    stuck_window_s: float = 2.0
    stuck_min_progress_m: float = 0.3
    stuck_near_obstacle_m: float = 1.0          # only counts as "stuck near an obstacle" inside this range
    stuck_violation_streak: int = 3             # consecutive bad ticks (at ~stuck_window_s spacing) before triggering
    escape_duration_s: float = 3.0
    escape_speed_mps: float = 2.0

    # Geofence / altitude.
    geofence_margin_m: float = 2.0
    altitude_margin_m: float = 0.5
    # Phase 4.1: a tighter, separate threshold that ALWAYS preempts
    # separation/obstacle/soft-altitude (checked right after geofence,
    # before tier 3) - "altitude floor protection must have priority over
    # horizontal escape when necessary". Deliberately smaller than
    # altitude_margin_m: the margin-based GEOFENCE_RISK response is the
    # normal, hysteresis-eligible altitude tier (5); this is the narrow,
    # non-negotiable emergency case - see docs/PHASE4_SAFETY.md.
    altitude_critical_margin_m: float = 0.15

    # Phase 4.1 anti-oscillation: once tiers 3 (separation), 4 (obstacle),
    # or 5-soft (altitude margin) becomes the active override for a
    # vehicle, it stays the active one for at least this long even if a
    # DIFFERENT one of those three would also fire on a later tick -
    # prevents rapid alternation between them, which was found to sustain
    # roll/pitch demand on DSLPIDControl (see docs/PHASE4_SAFETY.md's
    # Phase 4.1 section). Never applied to tier 1 (abort), tier 2
    # (geofence-outside), or the critical-altitude check above - those
    # always preempt immediately regardless of any held tier.
    min_override_hold_s: float = 0.5

    # Battery.
    battery_reserve_fraction: float = 0.2       # below this -> LOW_BATTERY / RETURN_TO_SAFE_POINT
    battery_critical_fraction: float = 0.08     # below this -> LAND_REQUESTED

    # Link / agent tracking.
    link_timeout_s: float = 1.0                 # own last-valid-command age beyond this -> DEGRADED_LINK
    lost_agent_timeout_s: float = 5.0           # a previously-seen neighbor silent this long -> LOST_AGENT

    # Command validity.
    command_expiry_grace_s: float = 0.0         # 0 = Command.is_expired's own boundary rule, no extra grace

    # Fallback control-step estimate used only for a vehicle's very first
    # evaluate() call (no previous timestamp yet to measure a real dt
    # from) - see evaluate()'s dt_s computation.
    fallback_dt_s: float = 1.0 / 24.0


# --------------------------------------------------------------------------
# Geometry helpers - pure functions, unit-tested directly.
# --------------------------------------------------------------------------


def braking_distance_m(speed_mps: float, brake_decel_mps2: float) -> float:
    """v^2 / (2a) - how far a vehicle travels while decelerating from
    speed_mps to a stop at a constant brake_decel_mps2."""
    if brake_decel_mps2 <= 0.0:
        return math.inf
    return (speed_mps ** 2) / (2.0 * brake_decel_mps2)


def predicted_travel_distance_m(speed_mps: float, command_latency_s: float, brake_decel_mps2: float) -> float:
    """Conservative lookahead distance: how far the vehicle could still
    travel in the current direction accounting for (a) the assumed
    command_latency_s before a new command takes effect at all (it keeps
    its current velocity that whole time) and (b) then braking to a stop.
    Used to check clearance not just "now" but "by the time anything
    could actually change.\""""
    return speed_mps * command_latency_s + braking_distance_m(speed_mps, brake_decel_mps2)


def _xy(vec3) -> Tuple[float, float]:
    return (vec3[0], vec3[1])


def _horizontal_distance(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def geofence_horizontal_margin_m(position_m, geofence: GeofenceSpec) -> float:
    """Signed distance from the nearest horizontal (box) boundary face -
    positive = still inside by that much, negative = already outside.
    Keep-out zones beyond this single flight-area box are not modeled -
    see docs/PHASE4_SAFETY.md's "Assumptions"."""
    cx, cy = geofence.center_m
    hx, hy = geofence.half_extents_m
    x, y = position_m[0], position_m[1]
    dx = hx - abs(x - cx)
    dy = hy - abs(y - cy)
    return min(dx, dy)


def vehicle_body_clearance_m(center_to_center_m: float, body_radius_m: float) -> float:
    """Edge-to-edge clearance between two equal-radius spherical/cylindrical
    vehicle bodies - may be negative if the bodies already overlap."""
    return center_to_center_m - 2.0 * body_radius_m


def inflate_for_uncertainty(clearance_m: float, uncertainty_margin_m: float) -> float:
    """A conservative (never optimistic) clearance estimate: subtract the
    uncertainty margin rather than trust the point estimate exactly."""
    return clearance_m - uncertainty_margin_m


def bound_velocity_step(own_vel, target_vel, dt_s: float, max_accel_mps2: float,
                          max_turn_rate_radps: float, max_speed_mps: float,
                          max_vertical_accel_mps2: Optional[float] = None):
    """Limits how far `target_vel` may differ from `own_vel` in ONE
    control step of length dt_s - acceleration and turn-rate bounded,
    then speed-capped. This is applied to EVERY command this module
    constructs (not only the mission-candidate tier) - an earlier version
    only bounded the mission-candidate path, which let the geofence/
    separation/obstacle override tiers command an instantaneous, abrupt
    velocity reversal every time they fired. That is exactly what
    destabilized DSLPIDControl's attitude loop badly enough to crash a
    vehicle in one of this phase's own scenario reruns - see
    docs/PHASE4_SAFETY.md's "Assumptions"/lessons-learned note. Pure
    tuple/float math, no numpy - see the module docstring on why this
    module doesn't import swarm_sim/behaviors/common.py's equivalent
    helpers (independence from the behavior-planner package).

    Phase 4.1 fix: horizontal and vertical acceleration are bounded
    INDEPENDENTLY, not as one combined 3D magnitude. The original version
    computed a single accel_mag over all 3 axes and applied one shared
    scale factor - which meant a large horizontal delta (typical for an
    escape/repulsion command) consumed nearly the entire accel budget,
    leaving almost none to correct a simultaneous vertical velocity error.
    That silent starvation was the confirmed root cause (see
    docs/PHASE4_SAFETY.md's Phase 4.1 diagnosis) of vertical velocity
    persisting uncorrected during horizontal avoidance, contributing to
    the drone_ground_contact_count regression this sub-phase fixes."""
    dt_s = max(dt_s, 1e-3)
    max_vertical_accel_mps2 = max_accel_mps2 if max_vertical_accel_mps2 is None else max_vertical_accel_mps2

    delta_h = (target_vel[0] - own_vel[0], target_vel[1] - own_vel[1])
    accel_h = math.hypot(*delta_h) / dt_s
    bounded_x, bounded_y = target_vel[0], target_vel[1]
    if accel_h > max_accel_mps2:
        scale = max_accel_mps2 / max(accel_h, 1e-9)
        bounded_x = own_vel[0] + delta_h[0] * scale
        bounded_y = own_vel[1] + delta_h[1] * scale

    delta_v = target_vel[2] - own_vel[2]
    accel_v = abs(delta_v) / dt_s
    bounded_z = target_vel[2]
    if accel_v > max_vertical_accel_mps2:
        bounded_z = own_vel[2] + math.copysign(max_vertical_accel_mps2 * dt_s, delta_v)

    bounded = (bounded_x, bounded_y, bounded_z)

    own_speed = math.hypot(own_vel[0], own_vel[1])
    new_speed = math.hypot(bounded[0], bounded[1])
    if own_speed > 1e-6 and new_speed > 1e-6:
        prev_angle = math.atan2(own_vel[1], own_vel[0])
        new_angle = math.atan2(bounded[1], bounded[0])
        d_angle = math.atan2(math.sin(new_angle - prev_angle), math.cos(new_angle - prev_angle))
        max_delta_angle = max_turn_rate_radps * dt_s
        if abs(d_angle) > max_delta_angle:
            clipped_angle = prev_angle + max(-max_delta_angle, min(max_delta_angle, d_angle))
            bounded = (math.cos(clipped_angle) * new_speed, math.sin(clipped_angle) * new_speed, bounded[2])

    return _clip_speed(bounded, max_speed_mps)


# --------------------------------------------------------------------------
# The supervisor
# --------------------------------------------------------------------------


class SafetySupervisor:
    """One instance covers the whole swarm (mirrors SwarmController's own
    one-instance-per-mission, many-drones-by-index pattern) - internal
    per-vehicle state (position history, last-seen neighbors, escape-mode
    timers) is keyed by vehicle_id so each drone's history stays separate."""

    def __init__(self, config: Optional[SafetySupervisorConfig] = None):
        self.cfg = config or SafetySupervisorConfig()
        self.event_log: List[SafetyEvent] = []
        self._last_state: Dict[str, SafetyState] = {}
        self._position_history: Dict[str, deque] = {}
        self._neighbor_last_seen: Dict[str, Dict[str, float]] = {}
        self._escape_until_s: Dict[str, float] = {}
        self._stuck_streak: Dict[str, int] = {}
        self._stuck_trigger_reported: Dict[str, bool] = {}
        self._isolated_since: Dict[str, float] = {}
        self._last_eval_time: Dict[str, float] = {}
        # Phase 4.1 anti-oscillation (see SafetySupervisorConfig.min_override_hold_s).
        self._active_soft_tier: Dict[str, int] = {}
        self._soft_tier_since: Dict[str, float] = {}
        # Transient, set at the start of each evaluate() call and read by
        # _velocity_command/_return_to_safe_point_command below - safe
        # because evaluate() runs to completion for one vehicle before
        # being called again (no reentrancy), and it avoids threading
        # own_vel/dt_s through every one of this class's small command-
        # construction helpers individually.
        self._current_own_vel = (0.0, 0.0, 0.0)
        self._current_dt_s = 1.0 / 24.0

    # -- logging ------------------------------------------------------

    def _log(self, vehicle_id: str, sim_time_s: float, event_type: str, to_state: SafetyState,
              reason: str, details: Optional[Dict[str, float]] = None) -> None:
        from_state = self._last_state.get(vehicle_id, SafetyState.NORMAL)
        self.event_log.append(SafetyEvent(
            sim_time_s=sim_time_s, vehicle_id=vehicle_id, event_type=event_type,
            from_state=from_state.value, to_state=to_state.value, reason=reason,
            details=details or {},
        ))
        self._last_state[vehicle_id] = to_state

    # -- command construction helpers ----------------------------------

    def _hold_command(self, vehicle_id: str, frame: Frame, now_s: float) -> Command:
        return Command(
            vehicle_id=vehicle_id, command_type=CommandType.HOLD, frame=frame,
            desired_position_m=None, desired_velocity_mps=None, yaw_rad=None, yaw_rate_radps=None,
            timestamp_s=now_s, expiration_time_s=now_s + 1.0, source="SafetySupervisor", confidence=1.0,
        )

    def _land_command(self, vehicle_id: str, frame: Frame, now_s: float) -> Command:
        return Command(
            vehicle_id=vehicle_id, command_type=CommandType.LAND, frame=frame,
            desired_position_m=None, desired_velocity_mps=None, yaw_rad=None, yaw_rate_radps=None,
            timestamp_s=now_s, expiration_time_s=now_s + 1.0, source="SafetySupervisor", confidence=1.0,
        )

    def _abort_command(self, vehicle_id: str, frame: Frame, now_s: float) -> Command:
        return Command(
            vehicle_id=vehicle_id, command_type=CommandType.ABORT, frame=frame,
            desired_position_m=None, desired_velocity_mps=None, yaw_rad=None, yaw_rate_radps=None,
            timestamp_s=now_s, expiration_time_s=now_s + 1.0, source="SafetySupervisor", confidence=1.0,
        )

    def _velocity_command(self, vehicle_id: str, frame: Frame, velocity_mps, now_s: float) -> Command:
        cfg = self.cfg
        bounded = bound_velocity_step(self._current_own_vel, tuple(velocity_mps), self._current_dt_s,
                                        cfg.max_accel_mps2, cfg.max_turn_rate_radps, cfg.max_speed_mps)
        return Command(
            vehicle_id=vehicle_id, command_type=CommandType.VELOCITY_SETPOINT, frame=frame,
            desired_position_m=None, desired_velocity_mps=tuple(float(v) for v in bounded),
            yaw_rad=None, yaw_rate_radps=None,
            timestamp_s=now_s, expiration_time_s=now_s + max(cfg.command_latency_s * 4, 0.5),
            source="SafetySupervisor", confidence=1.0,
        )

    def _return_to_safe_point_command(self, vehicle_id, frame, own_pos, safe_point, now_s) -> Command:
        cfg = self.cfg
        if safe_point is None:
            return self._hold_command(vehicle_id, frame, now_s)
        delta = (safe_point[0] - own_pos[0], safe_point[1] - own_pos[1], 0.0)
        dist = math.hypot(delta[0], delta[1])
        if dist < 1e-6:
            return self._hold_command(vehicle_id, frame, now_s)
        speed = min(cfg.max_speed_mps, dist)
        velocity = (delta[0] / dist * speed, delta[1] / dist * speed, 0.0)
        return self._velocity_command(vehicle_id, frame, velocity, now_s)

    # -- top-level entry point ------------------------------------------

    def evaluate(self, candidate_command: CandidateCommand, own_state: VehicleState,
                 sensor_observation: SensorObservation, neighbor_observations: Tuple[NeighborObservation, ...],
                 mission_context: MissionContext, now_s: float) -> SafetyDecision:
        vid = own_state.vehicle_id
        cfg = self.cfg
        frame = own_state.frame
        own_pos = own_state.position_m
        own_vel = own_state.velocity_mps
        own_speed = math.hypot(own_vel[0], own_vel[1])

        # Real measured control-step interval (falls back to
        # cfg.fallback_dt_s for this vehicle's very first evaluate() call)
        # - used to acceleration/turn-rate-bound EVERY command this module
        # constructs this tick, not just the mission-candidate path. See
        # bound_velocity_step's docstring for why this matters.
        dt_s = now_s - self._last_eval_time[vid] if vid in self._last_eval_time else cfg.fallback_dt_s
        dt_s = max(dt_s, 1e-3)
        self._last_eval_time[vid] = now_s
        self._current_own_vel = own_vel
        self._current_dt_s = dt_s

        self._update_position_history(vid, now_s, own_pos)
        self._update_neighbor_last_seen(vid, now_s, neighbor_observations)

        brake_dist = braking_distance_m(own_speed, cfg.assumed_brake_decel_mps2)
        # "Account for command latency"/"account for braking distance":
        # how far this vehicle could still travel, continuing at its
        # current speed, before a new command even takes effect
        # (command_latency_s) plus the distance needed to then stop
        # (brake_dist) - subtracted from sensed clearance below so the
        # obstacle/separation checks react to where the vehicle WILL BE
        # by the time anything can change, not just where it is now. This
        # was the actual root cause of a real Phase 4 finding: without
        # this lookahead, a vehicle already moving at cruise speed toward
        # an obstacle could close the last ~1m within the sensor's own
        # latency window before the instantaneous-clearance check ever
        # fired - see docs/PHASE4_SAFETY.md's "Assumptions"/lessons-
        # learned note.
        predicted_travel = predicted_travel_distance_m(own_speed, cfg.command_latency_s, cfg.assumed_brake_decel_mps2)

        def decide(state: SafetyState, command: Optional[Command], accepted: bool, constraints, reason,
                   min_clearance=None, ttc=None, c2c=None, body=None, obstacle=None, geofence_dist=None,
                   uncertainty=None) -> SafetyDecision:
            self._last_state[vid] = state
            return SafetyDecision(
                vehicle_id=vid, sim_time_s=now_s, accepted=accepted, filtered_command=command,
                active_constraints=tuple(constraints), reason=reason, emergency_state=state,
                min_predicted_clearance_m=min_clearance, time_to_collision_s=ttc,
                center_to_center_clearance_m=c2c, vehicle_body_clearance_m=body,
                obstacle_surface_clearance_m=obstacle, geofence_distance_m=geofence_dist,
                uncertainty_margin_m=uncertainty, braking_distance_m=brake_dist,
            )

        # --- Priority 1: operator abort / emergency state ---------------
        if mission_context.operator_abort or own_state.health_state == HealthState.FAILED:
            reason = "operator abort requested" if mission_context.operator_abort else \
                "vehicle health_state is FAILED (emergency)"
            self._log(vid, now_s, "state_transition", SafetyState.ABORT, reason)
            return decide(SafetyState.ABORT, self._abort_command(vid, frame, now_s), True,
                          ["operator_abort"], reason)

        # --- Priority 2: geofence and keep-out zones (horizontal) -------
        geofence_margin = geofence_horizontal_margin_m(own_pos, mission_context.geofence)
        if geofence_margin < 0.0:
            reason = f"outside geofence by {-geofence_margin:.2f}m - holding rather than autonomously re-entering"
            self._log(vid, now_s, "command_overridden", SafetyState.GEOFENCE_RISK, reason,
                       {"geofence_distance_m": geofence_margin})
            return decide(SafetyState.GEOFENCE_RISK, self._hold_command(vid, frame, now_s), True,
                          ["geofence_boundary"], reason, geofence_dist=geofence_margin)
        if geofence_margin < cfg.geofence_margin_m:
            inward = _inward_velocity(own_pos, mission_context.geofence, cfg.max_speed_mps)
            reason = f"within {geofence_margin:.2f}m of the geofence boundary (margin {cfg.geofence_margin_m}m)"
            self._log(vid, now_s, "command_overridden", SafetyState.GEOFENCE_RISK, reason,
                       {"geofence_distance_m": geofence_margin})
            return decide(SafetyState.GEOFENCE_RISK, self._velocity_command(vid, frame, inward, now_s), True,
                          ["geofence_boundary"], reason, geofence_dist=geofence_margin)

        # --- Priority 2.5 (Phase 4.1): critical altitude floor ------------
        # Always preempts separation/obstacle/soft-altitude, regardless of
        # any currently-held override tier - "altitude floor protection
        # must have priority over horizontal escape when necessary". Pure
        # vertical recovery, no horizontal component, so it never fights
        # whatever horizontal avoidance was doing.
        critical_decision = self._evaluate_altitude_critical(vid, frame, own_pos, mission_context, now_s, decide)
        if critical_decision is not None:
            self._active_soft_tier.pop(vid, None)
            return critical_decision

        # --- Priorities 3/4/5(soft): separation, obstacle, altitude margin
        # Phase 4.1 anti-oscillation: these three are evaluated together
        # and selected via hysteresis (SafetySupervisorConfig.min_override_hold_s)
        # rather than a plain first-match-wins short circuit - see that
        # config field's docstring and docs/PHASE4_SAFETY.md's Phase 4.1
        # section for why (rapid alternation among these three was found
        # to sustain roll/pitch demand on DSLPIDControl).
        sep_decision = self._evaluate_separation(vid, frame, own_pos, own_vel, predicted_travel,
                                                    neighbor_observations, now_s, decide)
        obs_decision = self._evaluate_obstacles(vid, frame, own_pos, sensor_observation,
                                                  mission_context, predicted_travel, now_s, decide)
        alt_decision = self._evaluate_altitude_soft(vid, frame, own_pos, mission_context, now_s, decide)
        soft_decision = self._select_with_hysteresis(
            vid, now_s, [(TIER_SEPARATION, sep_decision), (TIER_OBSTACLE, obs_decision), (TIER_ALTITUDE, alt_decision)],
        )
        if soft_decision is not None:
            return soft_decision

        # --- Priority 6: battery reserve ----------------------------------
        if own_state.battery_fraction < cfg.battery_critical_fraction:
            reason = f"battery {own_state.battery_fraction:.2%} below critical {cfg.battery_critical_fraction:.2%}"
            self._log(vid, now_s, "state_transition", SafetyState.LAND_REQUESTED, reason)
            return decide(SafetyState.LAND_REQUESTED, self._land_command(vid, frame, now_s), True,
                          ["battery_critical"], reason)
        if own_state.battery_fraction < cfg.battery_reserve_fraction:
            reason = f"battery {own_state.battery_fraction:.2%} below reserve {cfg.battery_reserve_fraction:.2%}"
            self._log(vid, now_s, "state_transition", SafetyState.LOW_BATTERY, reason)
            cmd = self._return_to_safe_point_command(vid, frame, own_pos, mission_context.safe_point_m, now_s)
            state = SafetyState.RETURN_TO_SAFE_POINT if mission_context.safe_point_m is not None else SafetyState.LOW_BATTERY
            return decide(state, cmd, True, ["low_battery"], reason)

        # --- Priority 7: lost-link behavior -------------------------------
        link_decision = self._evaluate_link(vid, frame, own_state, neighbor_observations, mission_context,
                                              now_s, decide)
        if link_decision is not None:
            return link_decision

        # --- Priority 8: estimator validity -------------------------------
        if not own_state.estimator_valid:
            reason = "own-state estimator_valid is False - cannot trust position/velocity for autonomous flight"
            self._log(vid, now_s, "command_overridden", SafetyState.SAFE_HOLD, reason)
            return decide(SafetyState.SAFE_HOLD, self._hold_command(vid, frame, now_s), True,
                          ["estimator_invalid"], reason)

        # --- Priority 9: mission candidate command ------------------------
        return self._evaluate_candidate(vid, frame, own_state, sensor_observation, candidate_command, now_s, decide)

    # -- per-tier helpers -------------------------------------------------

    def _update_position_history(self, vid, now_s, pos):
        hist = self._position_history.setdefault(vid, deque())
        hist.append((now_s, pos))
        window = self.cfg.stuck_window_s * 3
        while hist and now_s - hist[0][0] > window:
            hist.popleft()

    def _update_neighbor_last_seen(self, vid, now_s, neighbor_observations):
        seen = self._neighbor_last_seen.setdefault(vid, {})
        for n in neighbor_observations:
            seen[n.sender_id] = now_s

    def _evaluate_separation(self, vid, frame, own_pos, own_vel, predicted_travel, neighbor_observations, now_s, decide):
        cfg = self.cfg
        worst_body_clearance = math.inf
        worst_c2c = math.inf
        worst_direction = None
        for n in neighbor_observations:
            dx0, dy0 = own_pos[0] - n.measured_position_m[0], own_pos[1] - n.measured_position_m[1]
            c2c = math.hypot(dx0, dy0)
            uncertainty = cfg.separation_uncertainty_inflation_m
            if n.stale:
                uncertainty *= cfg.separation_stale_inflation_multiplier
            # Lookahead only applies when actually closing on this neighbor
            # (own velocity has a component pointing toward it) - a vehicle
            # already moving away needs no predictive penalty. See
            # evaluate()'s predicted_travel comment.
            closing = c2c > 1e-9 and (own_vel[0] * -dx0 + own_vel[1] * -dy0) > 0.0
            lookahead = predicted_travel if closing else 0.0
            body_clearance = inflate_for_uncertainty(
                vehicle_body_clearance_m(c2c, cfg.vehicle_body_radius_m), uncertainty + lookahead,
            )
            if body_clearance < worst_body_clearance:
                worst_body_clearance = body_clearance
                worst_c2c = c2c
                dx, dy = own_pos[0] - n.measured_position_m[0], own_pos[1] - n.measured_position_m[1]
                worst_direction = (dx, dy)

        seen = self._neighbor_last_seen.get(vid, {})
        recently_watched = any(now_s - t <= cfg.neighbor_watch_window_s for t in seen.values())
        if not neighbor_observations and recently_watched:
            # Dropped observation, conservative handling: a neighbor was
            # close recently and we have no fresh reading to say it isn't
            # anymore - keep the previous separation-risk posture instead
            # of silently relaxing to "all clear".
            if self._last_state.get(vid) == SafetyState.SEPARATION_RISK:
                reason = "neighbor observation dropped shortly after a separation risk - holding conservatively"
                self._log(vid, now_s, "command_overridden", SafetyState.SEPARATION_RISK, reason)
                return decide(SafetyState.SEPARATION_RISK, self._hold_command(vid, frame, now_s), True,
                              ["separation_risk", "sensor_dropout"], reason)

        if worst_body_clearance < cfg.separation_hard_margin_m:
            if worst_direction is not None:
                n = math.hypot(*worst_direction)
                away = (worst_direction[0] / n, worst_direction[1] / n, 0.0) if n > 1e-9 else (1.0, 0.0, 0.0)
            else:
                away = (1.0, 0.0, 0.0)
            reason = f"nearest-neighbor body clearance {worst_body_clearance:.2f}m below margin {cfg.separation_hard_margin_m}m"
            self._log(vid, now_s, "command_overridden", SafetyState.SEPARATION_RISK, reason,
                       {"vehicle_body_clearance_m": worst_body_clearance})
            velocity = tuple(a * cfg.max_speed_mps for a in away)
            return decide(SafetyState.SEPARATION_RISK, self._velocity_command(vid, frame, velocity, now_s), True,
                          ["separation_risk"], reason, c2c=worst_c2c, body=worst_body_clearance,
                          uncertainty=cfg.separation_uncertainty_inflation_m)
        return None

    def _nearest_obstacle_clearance(self, sensor_observation: SensorObservation, cfg: SafetySupervisorConfig,
                                      predicted_travel: float):
        finite = [r for r in sensor_observation.range_returns_m if math.isfinite(r)]
        if not finite:
            return None, None
        idx = min(range(len(sensor_observation.range_returns_m)),
                   key=lambda i: sensor_observation.range_returns_m[i]
                   if math.isfinite(sensor_observation.range_returns_m[i]) else math.inf)
        raw = min(finite)
        # predicted_travel (latency + braking lookahead, see evaluate())
        # applied unconditionally here, unlike separation's closing-velocity
        # gate - the range scan's bins don't individually carry enough
        # heading information to cheaply tell "closing on THIS bin" apart
        # from "passing near it", so this is deliberately the more
        # conservative of the two (documented simplification, see
        # docs/PHASE4_SAFETY.md's "Assumptions").
        clearance = inflate_for_uncertainty(raw - cfg.vehicle_body_radius_m,
                                             cfg.obstacle_uncertainty_inflation_m + predicted_travel)
        return clearance, idx

    def _evaluate_obstacles(self, vid, frame, own_pos, sensor_observation, mission_context, predicted_travel,
                              now_s, decide):
        cfg = self.cfg
        obstacle_clearance, bin_idx = self._nearest_obstacle_clearance(sensor_observation, cfg, predicted_travel)

        # Actual physical contact (evaluation input, see MissionContext's
        # docstring) always wins over whatever the sensor currently
        # reports - this is exactly the Phase 2 gap: a stale/optimistic
        # obstacle reading must never be allowed to mask a real contact.
        contact_now = mission_context.contact_detected
        near_obstacle = obstacle_clearance is not None and obstacle_clearance < cfg.stuck_near_obstacle_m
        history = self._position_history.get(vid, deque())
        progress = self._progress_over_window(history, now_s, cfg.stuck_window_s)
        stuck_condition = contact_now or (near_obstacle and progress is not None and progress < cfg.stuck_min_progress_m)

        streak = self._stuck_streak.get(vid, 0)
        if stuck_condition:
            streak += 1
        else:
            streak = 0
        self._stuck_streak[vid] = streak

        escape_until = self._escape_until_s.get(vid)
        in_escape = escape_until is not None and now_s < escape_until

        if contact_now or streak >= cfg.stuck_violation_streak or in_escape:
            if not in_escape:
                self._escape_until_s[vid] = now_s + cfg.escape_duration_s
                self._stuck_trigger_reported[vid] = True
                reason = ("actual PyBullet contact detected" if contact_now else
                          f"near-zero progress ({progress}) within {cfg.stuck_near_obstacle_m}m of an obstacle "
                          f"for {streak} consecutive checks")
                self._log(vid, now_s, "stuck_detected", SafetyState.SAFE_HOLD, reason,
                           {"progress_m": progress or 0.0, "obstacle_surface_clearance_m": obstacle_clearance or -1.0})
            escape_velocity = self._escape_velocity(own_pos, sensor_observation, bin_idx, cfg)
            return decide(SafetyState.SAFE_HOLD, self._velocity_command(vid, frame, escape_velocity, now_s), True,
                          ["obstacle_stuck"], "bounded escape: retreating from sensed obstacle direction",
                          obstacle=obstacle_clearance, uncertainty=cfg.obstacle_uncertainty_inflation_m)

        if self._stuck_trigger_reported.get(vid) and escape_until is not None and now_s >= escape_until:
            recovered = obstacle_clearance is None or obstacle_clearance >= cfg.stuck_near_obstacle_m
            self._log(vid, now_s, "recovery", SafetyState.NORMAL if recovered else SafetyState.SAFE_HOLD,
                       "escape window ended" + (", clearance recovered" if recovered else ", still near obstacle"),
                       {"obstacle_surface_clearance_m": obstacle_clearance or -1.0})
            self._stuck_trigger_reported[vid] = False
            self._escape_until_s[vid] = None

        if obstacle_clearance is not None and obstacle_clearance < cfg.obstacle_hard_margin_m:
            escape_velocity = self._escape_velocity(own_pos, sensor_observation, bin_idx, cfg)
            reason = f"sensed obstacle clearance {obstacle_clearance:.2f}m below margin {cfg.obstacle_hard_margin_m}m"
            self._log(vid, now_s, "command_overridden", SafetyState.SAFE_HOLD, reason,
                       {"obstacle_surface_clearance_m": obstacle_clearance})
            return decide(SafetyState.SAFE_HOLD, self._velocity_command(vid, frame, escape_velocity, now_s), True,
                          ["obstacle_clearance"], reason, obstacle=obstacle_clearance,
                          uncertainty=cfg.obstacle_uncertainty_inflation_m)
        return None

    def _progress_over_window(self, history: deque, now_s: float, window_s: float) -> Optional[float]:
        in_window = [p for (t, p) in history if now_s - t <= window_s]
        if len(in_window) < 2:
            return None
        first, last = in_window[0], in_window[-1]
        return math.hypot(last[0] - first[0], last[1] - first[1])

    def _escape_velocity(self, own_pos, sensor_observation, bin_idx, cfg):
        """Bounded escape direction: away from the nearest sensed obstacle
        bearing (range-scan bin index), sensor-derived only - never
        ground-truth obstacle geometry. Falls back to backing away along
        -x (arbitrary but deterministic) if no bin data is available.

        The z-component is always exactly 0.0 - an explicit "hold zero
        vertical velocity" target, not "don't care about vertical". Since
        Phase 4.1's bound_velocity_step decouples horizontal and vertical
        acceleration budgets, this z=0 target is no longer starved by a
        large horizontal escape delta (the Phase 4 root cause - see
        docs/PHASE4_SAFETY.md). This is still a simplified stand-in for a
        real altitude-hold controller: it cancels vertical VELOCITY, it
        does not correct any vertical POSITION drift that already
        happened - genuine floor protection is
        _evaluate_altitude_critical's job, not this method's."""
        if bin_idx is None or not sensor_observation.range_returns_m:
            return (-cfg.escape_speed_mps, 0.0, 0.0)
        n_bins = len(sensor_observation.range_returns_m)
        bin_angle = 2 * math.pi * bin_idx / n_bins
        away = (-math.cos(bin_angle) * cfg.escape_speed_mps, -math.sin(bin_angle) * cfg.escape_speed_mps, 0.0)
        return away

    def _evaluate_altitude_critical(self, vid, frame, own_pos, mission_context, now_s, decide):
        """Phase 4.1: a narrow, non-negotiable exception to the nominal
        priority order - see SafetySupervisorConfig.altitude_critical_margin_m's
        docstring. Pure vertical recovery (no horizontal component at
        all), so it can never be fighting a simultaneous horizontal
        avoidance command for control authority."""
        cfg = self.cfg
        z = own_pos[2]
        floor = mission_context.geofence.floor_alt_m
        if z < floor + cfg.altitude_critical_margin_m:
            reason = f"altitude {z:.2f}m within critical margin {cfg.altitude_critical_margin_m}m of floor {floor}m"
            self._log(vid, now_s, "command_overridden", SafetyState.GEOFENCE_RISK, reason)
            velocity = (0.0, 0.0, cfg.max_speed_mps)
            return decide(SafetyState.GEOFENCE_RISK, self._velocity_command(vid, frame, velocity, now_s), True,
                          ["altitude_floor_critical"], reason)
        return None

    def _evaluate_altitude_soft(self, vid, frame, own_pos, mission_context, now_s, decide):
        cfg = self.cfg
        z = own_pos[2]
        floor, ceiling = mission_context.geofence.floor_alt_m, mission_context.geofence.ceiling_alt_m
        # Altitude floor/ceiling is treated as part of GEOFENCE_RISK - the
        # required 11-state vocabulary has no separate altitude state, and
        # GeofenceSpec already carries floor/ceiling as one spec with the
        # horizontal box - see docs/PHASE4_SAFETY.md. This is the soft,
        # hysteresis-eligible margin tier (5) - see _evaluate_altitude_critical
        # above for the narrower, always-preempting emergency case.
        if z < floor + cfg.altitude_margin_m:
            reason = f"altitude {z:.2f}m within {cfg.altitude_margin_m}m of floor {floor}m"
            self._log(vid, now_s, "command_overridden", SafetyState.GEOFENCE_RISK, reason)
            velocity = (0.0, 0.0, cfg.max_speed_mps * 0.5)
            return decide(SafetyState.GEOFENCE_RISK, self._velocity_command(vid, frame, velocity, now_s), True,
                          ["altitude_floor"], reason)
        if z > ceiling - cfg.altitude_margin_m:
            reason = f"altitude {z:.2f}m within {cfg.altitude_margin_m}m of ceiling {ceiling}m"
            self._log(vid, now_s, "command_overridden", SafetyState.GEOFENCE_RISK, reason)
            velocity = (0.0, 0.0, -cfg.max_speed_mps * 0.5)
            return decide(SafetyState.GEOFENCE_RISK, self._velocity_command(vid, frame, velocity, now_s), True,
                          ["altitude_ceiling"], reason)
        return None

    def _select_with_hysteresis(self, vid, now_s, candidates):
        """candidates: [(tier_int, decision_or_None), ...] already in
        priority order. Returns the decision to actually use, applying
        SafetySupervisorConfig.min_override_hold_s so a previously-active
        tier among these candidates is preferred over switching to a
        different one that also fired, as long as the held tier is STILL
        genuinely firing (never forces a cleared/inactive tier's stale
        action). Logs every override-tier transition (Phase 4.1 required
        item: "safety event logging of override transitions") and every
        time hysteresis actually suppresses a switch that would otherwise
        have happened."""
        cfg = self.cfg
        held = self._active_soft_tier.get(vid)
        fired = [(t, d) for t, d in candidates if d is not None]
        if not fired:
            if held is not None:
                self._log(vid, now_s, "state_transition", SafetyState.NORMAL,
                           f"override tier {held} cleared - all soft-tier conditions resolved")
            self._active_soft_tier.pop(vid, None)
            return None

        held_entry = next((f for f in fired if f[0] == held), None)
        if held_entry is not None and (now_s - self._soft_tier_since.get(vid, now_s) < cfg.min_override_hold_s):
            chosen_tier, chosen_decision = held_entry
            if fired[0][0] != held:
                self._log(vid, now_s, "command_overridden", chosen_decision.emergency_state,
                           f"hysteresis: holding tier {held} (suppressing switch to tier {fired[0][0]}) - "
                           f"held for {now_s - self._soft_tier_since.get(vid, now_s):.2f}s of "
                           f"required {cfg.min_override_hold_s}s")
        else:
            chosen_tier, chosen_decision = fired[0]

        if chosen_tier != held:
            self._log(vid, now_s, "state_transition", chosen_decision.emergency_state,
                       f"override tier changed: {held} -> {chosen_tier}")
            self._active_soft_tier[vid] = chosen_tier
            self._soft_tier_since[vid] = now_s
        return chosen_decision

    def _evaluate_link(self, vid, frame, own_state, neighbor_observations, mission_context, now_s, decide):
        cfg = self.cfg
        last_valid = own_state.last_valid_command_time_s
        if last_valid is not None and now_s - last_valid > cfg.link_timeout_s:
            reason = f"last valid command was {now_s - last_valid:.2f}s ago (timeout {cfg.link_timeout_s}s)"
            self._log(vid, now_s, "state_transition", SafetyState.DEGRADED_LINK, reason)
            cmd = self._return_to_safe_point_command(vid, frame, own_state.position_m,
                                                        mission_context.safe_point_m, now_s)
            state = SafetyState.RETURN_TO_SAFE_POINT if mission_context.safe_point_m is not None else SafetyState.DEGRADED_LINK
            return decide(state, cmd, True, ["degraded_link"], reason)

        # LOST_AGENT: this vehicle has been isolated from the rest of the
        # swarm's communication graph continuously for lost_agent_timeout_s
        # - a sustained condition, not a single-tick blip, and sourced from
        # mission-level network connectivity (see MissionContext.own_agent_isolated's
        # docstring for why the onboard proximity sensor is deliberately NOT
        # used for this).
        if mission_context.own_agent_isolated:
            since = self._isolated_since.setdefault(vid, now_s)
            if now_s - since >= cfg.lost_agent_timeout_s:
                reason = f"isolated from the swarm's communication graph for {now_s - since:.2f}s"
                self._log(vid, now_s, "state_transition", SafetyState.LOST_AGENT, reason)
                return decide(SafetyState.LOST_AGENT, self._hold_command(vid, frame, now_s), True,
                              ["lost_agent"], reason)
        else:
            self._isolated_since.pop(vid, None)
        return None

    def _evaluate_candidate(self, vid, frame, own_state, sensor_observation, candidate_command, now_s, decide):
        cfg = self.cfg

        if candidate_command is None:
            reason = "no candidate command supplied"
            self._log(vid, now_s, "command_rejected", SafetyState.NORMAL, reason)
            return decide(SafetyState.NORMAL, None, False, ["command_missing"], reason)
        if candidate_command.vehicle_id != vid:
            reason = f"candidate_command.vehicle_id {candidate_command.vehicle_id!r} does not match {vid!r}"
            self._log(vid, now_s, "command_rejected", SafetyState.NORMAL, reason)
            return decide(SafetyState.NORMAL, None, False, ["command_mismatched_vehicle"], reason)
        if candidate_command.frame != own_state.frame:
            reason = (f"candidate_command.frame {candidate_command.frame!r} does not match "
                      f"own_state.frame {own_state.frame!r} - frames are never silently converted")
            self._log(vid, now_s, "command_rejected", SafetyState.NORMAL, reason)
            return decide(SafetyState.NORMAL, None, False, ["command_frame_mismatch"], reason)

        vel = candidate_command.desired_velocity_mps
        malformed = (not isinstance(vel, tuple) or len(vel) != 3 or
                     not all(isinstance(x, (int, float)) and not isinstance(x, bool) for x in vel) or
                     not all(math.isfinite(x) for x in vel))
        if malformed:
            reason = f"candidate desired_velocity_mps is malformed or non-finite: {vel!r}"
            self._log(vid, now_s, "command_rejected", SafetyState.NORMAL, reason)
            return decide(SafetyState.NORMAL, None, False, ["command_malformed"], reason)

        if now_s >= candidate_command.expiration_time_s + cfg.command_expiry_grace_s:
            reason = (f"candidate command expired at {candidate_command.expiration_time_s:.3f}s, "
                      f"now is {now_s:.3f}s")
            self._log(vid, now_s, "command_rejected", SafetyState.NORMAL, reason)
            return decide(SafetyState.NORMAL, None, False, ["command_expired"], reason)

        # Bound speed, acceleration, and turn rate independently of
        # whatever the planner already tried to enforce - see
        # swarm_sim/behaviors/common.py for the same distinction between
        # acceleration-shaped and heading-shaped native outputs; here we
        # only ever have a velocity, so acceleration is derived from the
        # change versus this vehicle's own current velocity, over the
        # REAL measured control-step interval (self._current_dt_s) - not
        # command_latency_s, which is a "how far might this travel before
        # anything changes" lookahead constant, a different quantity (see
        # bound_velocity_step's docstring for why conflating the two was
        # itself a bug this phase found and fixed).
        own_vel = own_state.velocity_mps
        dt_s = self._current_dt_s
        bounded_vel = _clip_speed(vel, cfg.max_speed_mps)
        delta = tuple(b - o for b, o in zip(bounded_vel, own_vel))
        accel_mag = math.sqrt(sum(d * d for d in delta)) / dt_s
        constraints = []
        if math.hypot(*vel[:2]) > cfg.max_speed_mps + 1e-9:
            constraints.append("speed_limit")
        if accel_mag > cfg.max_accel_mps2:
            scale = cfg.max_accel_mps2 / max(accel_mag, 1e-9)
            bounded_vel = tuple(o + d * scale for o, d in zip(own_vel, delta))
            constraints.append("accel_limit")
        own_speed = math.hypot(own_vel[0], own_vel[1])
        new_speed = math.hypot(bounded_vel[0], bounded_vel[1])
        if own_speed > 1e-6 and new_speed > 1e-6:
            prev_angle = math.atan2(own_vel[1], own_vel[0])
            new_angle = math.atan2(bounded_vel[1], bounded_vel[0])
            d_angle = math.atan2(math.sin(new_angle - prev_angle), math.cos(new_angle - prev_angle))
            max_delta = cfg.max_turn_rate_radps * dt_s
            if abs(d_angle) > max_delta:
                constraints.append("turn_rate_limit")
                clipped_angle = prev_angle + max(-max_delta, min(max_delta, d_angle))
                bounded_vel = (math.cos(clipped_angle) * new_speed, math.sin(clipped_angle) * new_speed, bounded_vel[2])

        state = SafetyState.DEGRADED_SENSOR if sensor_observation.dropout else SafetyState.NORMAL
        reason = "candidate accepted" if not constraints else f"candidate bounded: {', '.join(constraints)}"
        if sensor_observation.dropout:
            constraints.append("sensor_dropout")
            reason += " (sensor dropout this tick - degraded, not blocked)"
        command = self._velocity_command(vid, frame, bounded_vel, now_s)
        if not constraints or constraints == ["sensor_dropout"]:
            self._last_state[vid] = state
        else:
            self._log(vid, now_s, "command_overridden", state, reason)
        return decide(state, command, True, constraints, reason)


def _clip_speed(vec3, max_speed_mps: float):
    speed = math.hypot(vec3[0], vec3[1])
    if speed <= max_speed_mps or speed < 1e-9:
        return tuple(vec3)
    scale = max_speed_mps / speed
    return (vec3[0] * scale, vec3[1] * scale, vec3[2])


def _inward_velocity(position_m, geofence: GeofenceSpec, max_speed_mps: float):
    cx, cy = geofence.center_m
    dx, dy = cx - position_m[0], cy - position_m[1]
    n = math.hypot(dx, dy)
    if n < 1e-9:
        return (0.0, 0.0, 0.0)
    return (dx / n * max_speed_mps, dy / n * max_speed_mps, 0.0)
