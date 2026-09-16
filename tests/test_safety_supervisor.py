"""Phase 4 safety supervisor tests - see docs/PHASE4_SAFETY.md. Covers
required items 1-20, 23-24, 27-28 (items 21/22/25/26 are AST-based
architecture checks, in tests/test_safety_supervisor_architecture.py).
"""
import math

import pytest

from swarm_sim.contracts import (
    CommandType, Frame, GeofenceSpec, HealthState, NeighborObservation, SafetyState,
    SensorObservation, VehicleState, from_dict, to_dict,
)
from swarm_sim.safety_supervisor import (
    CandidateCommand, MissionContext, SafetySupervisor, SafetySupervisorConfig,
    braking_distance_m, inflate_for_uncertainty, vehicle_body_clearance_m,
)

GEOFENCE = GeofenceSpec(frame=Frame.LOCAL_ENU, center_m=(0.0, 0.0), half_extents_m=(20.0, 20.0),
                          floor_alt_m=0.5, ceiling_alt_m=10.0)


def own_state(pos=(0.0, 0.0, 3.0), vel=(1.0, 0.0, 0.0), batt=0.9, health=HealthState.OK,
              est=True, last_cmd=1.0, t=1.0):
    return VehicleState(
        vehicle_id="drone0", sim_time_s=t, frame=Frame.LOCAL_ENU, position_m=pos, velocity_mps=vel,
        acceleration_mps2=(0.0, 0.0, 0.0), attitude_rad=(0.0, 0.0, 0.0), angular_velocity_radps=(0.0, 0.0, 0.0),
        battery_fraction=batt, health_state=health, estimator_valid=est, last_valid_command_time_s=last_cmd,
    )


def sensor_obs(dropout=False, ranges=None, n_bins=16):
    r = ranges if ranges is not None else (math.inf,) * n_bins
    return SensorObservation(
        vehicle_id="drone0", sensor_timestamp_s=1.0, sensor_latency_s=0.0, fov_deg=270.0,
        range_returns_m=r, occluded=(False,) * len(r), dropout=dropout, pose_uncertainty_m=0.0, detections=(),
    )


def neighbor(pos=(0.2, 0.0, 3.0), stale=False, sender="drone1"):
    return NeighborObservation(
        receiver_id="drone0", sender_id=sender, frame=Frame.LOCAL_ENU, measured_position_m=pos,
        measured_velocity_mps=(0.0, 0.0, 0.0), sample_timestamp_s=0.9, delivery_timestamp_s=1.0,
        packet_age_s=0.1, communication_confidence=0.9, stale=stale,
    )


def candidate(vel=(1.0, 0.0, 0.0), t=1.0, ttl=1.0, vid="drone0", frame=Frame.LOCAL_ENU):
    return CandidateCommand(vehicle_id=vid, desired_velocity_mps=vel, frame=frame,
                              timestamp_s=t, expiration_time_s=t + ttl)


def ctx(**kwargs):
    kwargs.setdefault("geofence", GEOFENCE)
    return MissionContext(**kwargs)


def sup(**overrides):
    return SafetySupervisor(SafetySupervisorConfig(**overrides))


# 1. expired command rejection ------------------------------------------

def test_expired_command_rejected():
    cand = candidate(t=0.0, ttl=0.5)
    d = sup().evaluate(cand, own_state(), sensor_obs(), (), ctx(), now_s=1.0)
    assert d.accepted is False
    assert d.filtered_command is None
    assert "command_expired" in d.active_constraints


def test_command_not_yet_expired_is_not_rejected_for_expiry():
    cand = candidate(t=0.9, ttl=1.0)
    d = sup().evaluate(cand, own_state(), sensor_obs(), (), ctx(), now_s=1.0)
    assert "command_expired" not in d.active_constraints


# 2. NaN/Inf command rejection --------------------------------------------

@pytest.mark.parametrize("bad_component", [float("nan"), float("inf"), float("-inf")])
def test_non_finite_command_rejected(bad_component):
    cand = candidate(vel=(bad_component, 0.0, 0.0))
    d = sup().evaluate(cand, own_state(), sensor_obs(), (), ctx(), now_s=1.0)
    assert d.accepted is False
    assert d.filtered_command is None
    assert "command_malformed" in d.active_constraints


def test_malformed_shape_command_rejected():
    cand = candidate(vel=(1.0, 0.0))  # wrong length
    d = sup().evaluate(cand, own_state(), sensor_obs(), (), ctx(), now_s=1.0)
    assert d.accepted is False
    assert "command_malformed" in d.active_constraints


# 3. speed-limit enforcement ----------------------------------------------

def test_speed_limit_enforced():
    s = sup(max_speed_mps=2.0)
    cand = candidate(vel=(10.0, 0.0, 0.0))
    d = s.evaluate(cand, own_state(vel=(1.0, 0.0, 0.0)), sensor_obs(), (), ctx(), now_s=1.0)
    assert d.accepted
    speed = math.hypot(*d.filtered_command.desired_velocity_mps[:2])
    assert speed <= 2.0 + 1e-6
    assert "speed_limit" in d.active_constraints


# 4. acceleration-limit enforcement ---------------------------------------

def test_acceleration_limit_enforced():
    # fallback_dt_s (not command_latency_s - a different quantity, see
    # bound_velocity_step's docstring) is the control-step interval used
    # for this vehicle's first evaluate() call, since there is no prior
    # timestamp yet to measure a real one from.
    s = sup(max_accel_mps2=1.0, fallback_dt_s=0.1, max_speed_mps=10.0)
    cand = candidate(vel=(5.0, 0.0, 0.0))
    d = s.evaluate(cand, own_state(vel=(0.0, 0.0, 0.0)), sensor_obs(), (), ctx(), now_s=1.0)
    assert "accel_limit" in d.active_constraints
    delta_speed = math.hypot(*d.filtered_command.desired_velocity_mps[:2])
    assert delta_speed <= 1.0 * 0.1 + 1e-6


# 5. turn-rate enforcement -------------------------------------------------

def test_turn_rate_enforced():
    s = sup(max_turn_rate_radps=0.5, fallback_dt_s=0.1, max_accel_mps2=1000.0, max_speed_mps=1000.0)
    cand = candidate(vel=(0.0, 3.0, 0.0))   # same speed, 90 degree turn
    d = s.evaluate(cand, own_state(vel=(3.0, 0.0, 0.0)), sensor_obs(), (), ctx(), now_s=1.0)
    assert "turn_rate_limit" in d.active_constraints
    out = d.filtered_command.desired_velocity_mps
    angle = math.atan2(out[1], out[0])
    assert angle <= 0.5 * 0.1 + 1e-6


# 6. geofence boundary rejection -------------------------------------------

def test_geofence_outside_boundary_holds():
    d = sup().evaluate(candidate(), own_state(pos=(25.0, 0.0, 3.0)), sensor_obs(), (), ctx(), now_s=1.0)
    assert d.emergency_state == SafetyState.GEOFENCE_RISK
    assert d.filtered_command.command_type == CommandType.HOLD
    assert d.geofence_distance_m < 0.0


def test_geofence_within_margin_pushes_inward():
    # Stationary own_state: isolates direction selection from the (separately
    # tested) acceleration-bounded ramp toward that direction.
    d = sup(geofence_margin_m=2.0).evaluate(candidate(), own_state(pos=(19.0, 0.0, 3.0), vel=(0.0, 0.0, 0.0)),
                                              sensor_obs(), (), ctx(), now_s=1.0)
    assert d.emergency_state == SafetyState.GEOFENCE_RISK
    assert d.filtered_command.command_type == CommandType.VELOCITY_SETPOINT
    assert d.filtered_command.desired_velocity_mps[0] < 0.0   # pushed back toward center (-x)


def test_geofence_well_inside_does_not_trigger():
    d = sup().evaluate(candidate(), own_state(pos=(0.0, 0.0, 3.0)), sensor_obs(), (), ctx(), now_s=1.0)
    assert d.emergency_state != SafetyState.GEOFENCE_RISK


# 7/8. altitude floor and ceiling -------------------------------------------

def test_altitude_floor_enforced():
    # floor=0.5, altitude_margin_m=0.5 (soft threshold 1.0m), but
    # altitude_critical_margin_m=0.15 (critical threshold 0.65m) - z=0.8
    # is inside the soft margin but outside the critical one, isolating
    # the soft (hysteresis-eligible) tier from the Phase 4.1 critical tier
    # (see test_altitude_floor_critical_preempts_everything below).
    d = sup().evaluate(candidate(), own_state(pos=(0.0, 0.0, 0.8)), sensor_obs(), (), ctx(), now_s=1.0)
    assert d.emergency_state == SafetyState.GEOFENCE_RISK
    assert "altitude_floor" in d.active_constraints
    assert d.filtered_command.desired_velocity_mps[2] > 0.0


def test_altitude_floor_critical_preempts_everything():
    """Phase 4.1: within altitude_critical_margin_m of the floor, the
    critical tier fires instead of the soft one, and preempts even an
    active separation risk - "altitude floor protection must have
    priority over horizontal escape when necessary"."""
    d = sup().evaluate(candidate(vel=(0.0, 0.0, 0.0)), own_state(pos=(0.0, 0.0, 0.6), vel=(0.0, 0.0, 0.0)),
                        sensor_obs(), (neighbor(pos=(0.1, 0.0, 0.6)),), ctx(), now_s=1.0)
    assert d.emergency_state == SafetyState.GEOFENCE_RISK
    assert "altitude_floor_critical" in d.active_constraints
    out = d.filtered_command.desired_velocity_mps
    assert out[2] > 0.0
    assert out[0] == 0.0 and out[1] == 0.0   # pure vertical recovery, no horizontal component


def test_altitude_ceiling_enforced():
    d = sup().evaluate(candidate(), own_state(pos=(0.0, 0.0, 9.8)), sensor_obs(), (), ctx(), now_s=1.0)
    assert d.emergency_state == SafetyState.GEOFENCE_RISK
    assert "altitude_ceiling" in d.active_constraints
    assert d.filtered_command.desired_velocity_mps[2] < 0.0


# 9. obstacle-clearance override -------------------------------------------

def test_obstacle_clearance_override():
    d = sup().evaluate(candidate(), own_state(), sensor_obs(ranges=(0.4,) + (math.inf,) * 15), (),
                        ctx(), now_s=1.0)
    assert d.emergency_state == SafetyState.SAFE_HOLD
    assert "obstacle_clearance" in d.active_constraints
    assert d.obstacle_surface_clearance_m < sup().cfg.obstacle_hard_margin_m


def test_obstacle_far_away_does_not_trigger():
    d = sup().evaluate(candidate(), own_state(), sensor_obs(ranges=(5.0,) + (math.inf,) * 15), (),
                        ctx(), now_s=1.0)
    assert d.emergency_state != SafetyState.SAFE_HOLD


# 10. inter-drone separation override ---------------------------------------

def test_separation_override():
    # Stationary own_state: isolates direction selection from the ramp.
    d = sup().evaluate(candidate(vel=(0.0, 0.0, 0.0)), own_state(vel=(0.0, 0.0, 0.0)), sensor_obs(),
                        (neighbor(pos=(0.2, 0.0, 3.0)),), ctx(), now_s=1.0)
    assert d.emergency_state == SafetyState.SEPARATION_RISK
    assert "separation_risk" in d.active_constraints
    # Repulsion pushes away from the +x neighbor.
    assert d.filtered_command.desired_velocity_mps[0] < 0.0


def test_separation_far_neighbor_does_not_trigger():
    d = sup().evaluate(candidate(), own_state(), sensor_obs(), (neighbor(pos=(10.0, 0.0, 3.0)),),
                        ctx(), now_s=1.0)
    assert d.emergency_state != SafetyState.SEPARATION_RISK


def test_override_commands_are_acceleration_bounded_not_instantaneous():
    """A real bug this phase found and fixed: override tiers (geofence/
    separation/obstacle) used to only speed-cap their output, letting them
    command an instantaneous velocity reversal every time they fired -
    exactly the kind of abrupt command that destabilized DSLPIDControl in
    one of this phase's own scenario reruns (see docs/PHASE4_SAFETY.md).
    Every command this module constructs must now be acceleration-bounded
    relative to the vehicle's own current velocity, the same as the
    mission-candidate path always was."""
    s = sup(max_accel_mps2=1.0, fallback_dt_s=0.1, max_speed_mps=10.0)
    fast_forward = own_state(vel=(5.0, 0.0, 0.0))
    d = s.evaluate(candidate(vel=(5.0, 0.0, 0.0)), fast_forward, sensor_obs(),
                    (neighbor(pos=(0.2, 0.0, 3.0)),), ctx(), now_s=1.0)
    assert d.emergency_state == SafetyState.SEPARATION_RISK
    out = d.filtered_command.desired_velocity_mps
    delta_speed = math.hypot(out[0] - 5.0, out[1] - 0.0)
    assert delta_speed <= 1.0 * 0.1 + 1e-6   # max_accel_mps2 * fallback_dt_s


# 11. stale neighbor observation handling (conservative) --------------------

def test_stale_neighbor_treated_more_conservatively_than_fresh():
    # c2c=0.8, body=0.8-0.12=0.68. Fresh: 0.68-0.2=0.48 >= 0.3 margin (no
    # risk). Stale: 0.68-0.6=0.08 < 0.3 margin (risk) - same physical
    # reading, different conservatism purely from staleness.
    s = sup(separation_hard_margin_m=0.3, separation_uncertainty_inflation_m=0.2,
            separation_stale_inflation_multiplier=3.0, vehicle_body_radius_m=0.06)
    pos = (0.8, 0.0, 3.0)
    # Zero velocity so the closing-velocity lookahead (a separate, real
    # requirement - "account for... braking distance") contributes
    # nothing here; this test isolates staleness alone.
    hover = own_state(vel=(0.0, 0.0, 0.0))
    fresh = s.evaluate(candidate(vel=(0.0, 0.0, 0.0)), hover, sensor_obs(), (neighbor(pos=pos, stale=False),),
                        ctx(), now_s=1.0)
    assert fresh.emergency_state != SafetyState.SEPARATION_RISK

    stale = s.evaluate(candidate(vel=(0.0, 0.0, 0.0)), hover, sensor_obs(), (neighbor(pos=pos, stale=True),),
                        ctx(), now_s=1.0)
    assert stale.emergency_state == SafetyState.SEPARATION_RISK


# latency/braking lookahead ("account for command latency"/"account for
# braking distance") -----------------------------------------------------

def test_closing_velocity_triggers_earlier_than_hovering_at_same_clearance():
    """Same instantaneous clearance, same uncertainty - but a vehicle
    already closing fast on a neighbor must be flagged where a hovering
    one at the identical distance is not, because it will have covered
    real ground before any new command can take effect."""
    s = sup(separation_hard_margin_m=0.3, separation_uncertainty_inflation_m=0.1,
            vehicle_body_radius_m=0.06, command_latency_s=0.2, assumed_brake_decel_mps2=2.0)
    pos = (0.8, 0.0, 3.0)   # body clearance = 0.8-0.12=0.68, well clear of a 0.3+0.1 margin at rest
    hovering = s.evaluate(candidate(vel=(0.0, 0.0, 0.0)), own_state(vel=(0.0, 0.0, 0.0)), sensor_obs(),
                           (neighbor(pos=pos),), ctx(), now_s=1.0)
    assert hovering.emergency_state != SafetyState.SEPARATION_RISK

    closing_fast = s.evaluate(candidate(vel=(4.0, 0.0, 0.0)), own_state(vel=(4.0, 0.0, 0.0)), sensor_obs(),
                               (neighbor(pos=pos),), ctx(), now_s=1.0)
    assert closing_fast.emergency_state == SafetyState.SEPARATION_RISK


def test_receding_velocity_does_not_get_penalized_by_lookahead():
    """Moving AWAY from a neighbor at the same distance/speed must not be
    treated as more dangerous than hovering - the lookahead only applies
    when actually closing."""
    s = sup(separation_hard_margin_m=0.3, separation_uncertainty_inflation_m=0.1,
            vehicle_body_radius_m=0.06, command_latency_s=0.2, assumed_brake_decel_mps2=2.0)
    pos = (0.8, 0.0, 3.0)
    receding = s.evaluate(candidate(vel=(-4.0, 0.0, 0.0)), own_state(vel=(-4.0, 0.0, 0.0)), sensor_obs(),
                           (neighbor(pos=pos),), ctx(), now_s=1.0)
    assert receding.emergency_state != SafetyState.SEPARATION_RISK


def test_obstacle_lookahead_triggers_earlier_for_fast_approach():
    s = sup(obstacle_hard_margin_m=0.3, obstacle_uncertainty_inflation_m=0.1,
            vehicle_body_radius_m=0.06, command_latency_s=0.2, assumed_brake_decel_mps2=2.0)
    ranges = (0.9,) + (math.inf,) * 15   # raw-body clearance ~0.84, clear at rest
    slow = s.evaluate(candidate(vel=(0.1, 0.0, 0.0)), own_state(vel=(0.1, 0.0, 0.0)), sensor_obs(ranges=ranges),
                       (), ctx(), now_s=1.0)
    assert slow.emergency_state != SafetyState.SAFE_HOLD

    fast = s.evaluate(candidate(vel=(4.0, 0.0, 0.0)), own_state(vel=(4.0, 0.0, 0.0)), sensor_obs(ranges=ranges),
                       (), ctx(), now_s=1.0)
    assert fast.emergency_state == SafetyState.SAFE_HOLD


# 12. stale/invalid obstacle observation handling (conservative) ------------

def test_contact_detected_overrides_optimistic_obstacle_sensor():
    """The exact Phase 2 gap: sensor says "all clear" (dropout, all-inf
    ranges) but a real PyBullet contact happened - contact must win."""
    d = sup().evaluate(candidate(), own_state(), sensor_obs(dropout=True), (),
                        ctx(contact_detected=True), now_s=1.0)
    assert d.emergency_state == SafetyState.SAFE_HOLD
    assert "obstacle_stuck" in d.active_constraints


# 13. dropped sensor handling (conservative) ---------------------------------

def test_dropped_neighbor_observation_after_separation_risk_holds_conservatively():
    s = sup()
    d1 = s.evaluate(candidate(), own_state(), sensor_obs(), (neighbor(pos=(0.2, 0.0, 3.0)),), ctx(), now_s=1.0)
    assert d1.emergency_state == SafetyState.SEPARATION_RISK
    d2 = s.evaluate(candidate(), own_state(t=1.05), sensor_obs(), (), ctx(), now_s=1.05)
    assert d2.emergency_state == SafetyState.SEPARATION_RISK
    assert d2.filtered_command.command_type == CommandType.HOLD
    assert "sensor_dropout" in d2.active_constraints


# 14. invalid own-state handling ---------------------------------------------

def test_invalid_estimator_holds():
    d = sup().evaluate(candidate(), own_state(est=False), sensor_obs(), (), ctx(), now_s=1.0)
    assert d.emergency_state == SafetyState.SAFE_HOLD
    assert d.filtered_command.command_type == CommandType.HOLD
    assert "estimator_invalid" in d.active_constraints


# 15. low-battery transition --------------------------------------------------

def test_low_battery_transitions_to_return_to_safe_point():
    d = sup().evaluate(candidate(), own_state(batt=0.15), sensor_obs(), (),
                        ctx(safe_point_m=(0.0, 0.0, 3.0)), now_s=1.0)
    assert d.emergency_state == SafetyState.RETURN_TO_SAFE_POINT
    assert "low_battery" in d.active_constraints


def test_low_battery_without_safe_point_stays_low_battery_state():
    d = sup().evaluate(candidate(), own_state(batt=0.15), sensor_obs(), (), ctx(), now_s=1.0)
    assert d.emergency_state == SafetyState.LOW_BATTERY


def test_critical_battery_requests_land():
    d = sup().evaluate(candidate(), own_state(batt=0.03), sensor_obs(), (), ctx(), now_s=1.0)
    assert d.emergency_state == SafetyState.LAND_REQUESTED
    assert d.filtered_command.command_type == CommandType.LAND


# 16. lost-link transition -----------------------------------------------------

def test_degraded_link_from_stale_command_age():
    d = sup(link_timeout_s=1.0).evaluate(candidate(), own_state(last_cmd=0.0), sensor_obs(), (),
                                           ctx(), now_s=2.0)
    assert d.emergency_state in (SafetyState.DEGRADED_LINK, SafetyState.RETURN_TO_SAFE_POINT)
    assert "degraded_link" in d.active_constraints


def test_lost_agent_from_sustained_swarm_isolation():
    """LOST_AGENT is sourced from mission-level swarm connectivity
    (MissionContext.own_agent_isolated), not the short-range onboard
    proximity sensor - see that field's docstring for why (an earlier
    version used neighbor_observations directly and produced a large
    false-positive rate during normal search dispersal)."""
    s = sup(lost_agent_timeout_s=1.0, link_timeout_s=100.0)
    d1 = s.evaluate(candidate(), own_state(t=1.0, last_cmd=1.0), sensor_obs(), (),
                     ctx(own_agent_isolated=True), now_s=1.0)
    assert d1.emergency_state != SafetyState.LOST_AGENT   # not yet sustained long enough
    d2 = s.evaluate(candidate(), own_state(t=2.5, last_cmd=2.4), sensor_obs(), (),
                     ctx(own_agent_isolated=True), now_s=2.5)
    assert d2.emergency_state == SafetyState.LOST_AGENT
    assert "lost_agent" in d2.active_constraints


def test_brief_isolation_blip_does_not_trigger_lost_agent():
    s = sup(lost_agent_timeout_s=2.0, link_timeout_s=100.0)
    s.evaluate(candidate(), own_state(t=1.0, last_cmd=1.0), sensor_obs(), (),
               ctx(own_agent_isolated=True), now_s=1.0)
    # Reconnects before the sustained timeout - isolation clock resets.
    s.evaluate(candidate(), own_state(t=1.5, last_cmd=1.5), sensor_obs(), (),
               ctx(own_agent_isolated=False), now_s=1.5)
    d = s.evaluate(candidate(), own_state(t=2.5, last_cmd=2.4), sensor_obs(), (),
                    ctx(own_agent_isolated=True), now_s=2.5)
    assert d.emergency_state != SafetyState.LOST_AGENT


# 17/18. operator-abort / emergency-state precedence ---------------------------

def test_operator_abort_takes_precedence_over_geofence_and_separation():
    d = sup().evaluate(candidate(), own_state(pos=(25.0, 0.0, 3.0)), sensor_obs(),
                        (neighbor(pos=(0.1, 0.0, 3.0)),), ctx(operator_abort=True), now_s=1.0)
    assert d.emergency_state == SafetyState.ABORT
    assert d.filtered_command.command_type == CommandType.ABORT


def test_health_failed_takes_precedence_over_geofence():
    d = sup().evaluate(candidate(), own_state(pos=(25.0, 0.0, 3.0), health=HealthState.FAILED), sensor_obs(),
                        (), ctx(), now_s=1.0)
    assert d.emergency_state == SafetyState.ABORT


# 19/20. persistent obstacle-contact/stuck detection + bounded escape ----------

def test_persistent_near_zero_progress_near_obstacle_triggers_stuck_and_escape():
    s = sup(stuck_violation_streak=3, stuck_window_s=1.0, escape_speed_mps=1.0)
    states = []
    for k, t in enumerate([1.0, 1.5, 2.0, 2.5, 3.0]):
        d = s.evaluate(candidate(), own_state(pos=(0.01 * k, 0.0, 3.0), t=t),
                        sensor_obs(ranges=(0.4,) + (math.inf,) * 15), (), ctx(), now_s=t)
        states.append(d)
    triggered = [d for d in states if "obstacle_stuck" in d.active_constraints]
    assert triggered, "expected a persistent-stuck escape to eventually trigger"
    escape = triggered[0]
    assert escape.emergency_state == SafetyState.SAFE_HOLD
    speed = math.hypot(*escape.filtered_command.desired_velocity_mps[:2])
    assert math.isclose(speed, s.cfg.escape_speed_mps, abs_tol=1e-6)
    assert any(e.event_type == "stuck_detected" for e in s.event_log)


def test_bounded_escape_direction_points_away_from_sensed_obstacle_bin():
    s = sup()
    n_bins = 16
    ranges = [math.inf] * n_bins
    ranges[0] = 0.2   # nearest obstacle sensed dead ahead (bin 0, angle 0)
    # Stationary own_state: isolates direction selection from the ramp.
    d = s.evaluate(candidate(vel=(0.0, 0.0, 0.0)), own_state(vel=(0.0, 0.0, 0.0)),
                    sensor_obs(ranges=tuple(ranges), n_bins=n_bins), (),
                    ctx(contact_detected=True), now_s=1.0)
    vx, vy, _ = d.filtered_command.desired_velocity_mps
    assert vx < 0.0   # escape direction is opposite the obstacle bearing (angle 0 -> +x)
    speed = math.hypot(vx, vy)
    assert speed <= s.cfg.escape_speed_mps + 1e-6


# 23. safety decision serialization --------------------------------------------

def test_safety_decision_round_trips_through_to_dict_from_dict():
    d = sup().evaluate(candidate(), own_state(), sensor_obs(ranges=(0.4,) + (math.inf,) * 15), (),
                        ctx(), now_s=1.0)
    data = to_dict(d)
    restored = from_dict(type(d), data)
    assert restored == d


def test_safety_event_serializes_to_plain_dict():
    s = sup()
    s.evaluate(candidate(), own_state(), sensor_obs(ranges=(0.4,) + (math.inf,) * 15), (), ctx(), now_s=1.0)
    assert s.event_log
    event_dict = s.event_log[0].to_dict()
    assert isinstance(event_dict, dict)
    assert event_dict["vehicle_id"] == "drone0"


# 24. deterministic replay -----------------------------------------------------

def test_deterministic_replay_identical_call_sequence():
    def run():
        s = sup()
        outputs = []
        for t in [1.0, 1.5, 2.0]:
            d = s.evaluate(candidate(t=t), own_state(pos=(0.02 * t, 0.0, 3.0), t=t), sensor_obs(), (),
                            ctx(), now_s=t)
            outputs.append((d.emergency_state, d.accepted,
                             d.filtered_command.desired_velocity_mps if d.filtered_command else None))
        return outputs, [e.to_dict() for e in s.event_log]

    out_a, log_a = run()
    out_b, log_b = run()
    assert out_a == out_b
    assert log_a == log_b


# 27. actual vehicle-body clearance calculation --------------------------------

def test_vehicle_body_clearance_calculation():
    assert math.isclose(vehicle_body_clearance_m(1.0, 0.06), 1.0 - 0.12)
    assert vehicle_body_clearance_m(0.05, 0.06) < 0.0   # overlapping bodies -> negative, not clamped


# 28. uncertainty-inflated clearance calculation --------------------------------

def test_uncertainty_inflated_clearance_calculation():
    assert math.isclose(inflate_for_uncertainty(1.0, 0.3), 0.7)
    assert inflate_for_uncertainty(0.2, 0.3) < 0.0


def test_braking_distance_increases_with_speed_and_decreases_with_decel():
    assert braking_distance_m(0.0, 3.0) == 0.0
    assert braking_distance_m(4.0, 3.0) > braking_distance_m(2.0, 3.0)
    assert braking_distance_m(4.0, 6.0) < braking_distance_m(4.0, 3.0)
    assert braking_distance_m(4.0, 0.0) == math.inf
