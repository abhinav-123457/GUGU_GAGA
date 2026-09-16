"""Phase 4.1 regression tests - diagnosing and fixing the
drone_ground_contact_count regression from Phase 4 (0 -> 4179 in the
delayed-observation obstacle-wedging scenario). See
docs/PHASE4_SAFETY.md's "Phase 4.1" section for the full diagnosis:

- bound_velocity_step originally bounded horizontal+vertical acceleration
  as ONE combined 3D magnitude, so a large horizontal escape/repulsion
  delta consumed nearly the whole accel budget, silently starving a
  simultaneous vertical correction - confirmed root cause, fixed by
  decoupling the two axes.
- separation/obstacle/soft-altitude tiers could rapidly alternate tick to
  tick, each restarting the acceleration-bounded ramp toward a different
  direction - fixed with tier-hold hysteresis
  (SafetySupervisorConfig.min_override_hold_s).
- a new, tighter altitude_critical_margin_m tier always preempts
  separation/obstacle/soft-altitude for genuine ground-strike prevention.

Required regression items covered: repeated alternating override
suppression, horizontal escape preserving altitude, altitude-floor
precedence, vertical command bounds, no ground contact in a deterministic
simplified test, safety event logging of override transitions, recovery
after obstacle escape.
"""
import math

import pytest

from swarm_sim.contracts import (
    Frame, GeofenceSpec, HealthState, NeighborObservation, SafetyState, SensorObservation, VehicleState,
)
from swarm_sim.safety_supervisor import (
    CandidateCommand, MissionContext, SafetySupervisor, SafetySupervisorConfig, bound_velocity_step,
)

GEOFENCE = GeofenceSpec(frame=Frame.LOCAL_ENU, center_m=(0.0, 0.0), half_extents_m=(20.0, 20.0),
                          floor_alt_m=0.5, ceiling_alt_m=10.0)


def own_state(pos=(0.0, 0.0, 3.0), vel=(0.0, 0.0, 0.0), t=0.0):
    return VehicleState(
        vehicle_id="drone0", sim_time_s=t, frame=Frame.LOCAL_ENU, position_m=pos, velocity_mps=vel,
        acceleration_mps2=(0.0, 0.0, 0.0), attitude_rad=(0.0, 0.0, 0.0), angular_velocity_radps=(0.0, 0.0, 0.0),
        battery_fraction=0.9, health_state=HealthState.OK, estimator_valid=True, last_valid_command_time_s=t,
    )


def sensor_obs(ranges=None, n_bins=16, dropout=False):
    r = ranges if ranges is not None else (math.inf,) * n_bins
    return SensorObservation(
        vehicle_id="drone0", sensor_timestamp_s=0.0, sensor_latency_s=0.0, fov_deg=270.0,
        range_returns_m=r, occluded=(False,) * len(r), dropout=dropout, pose_uncertainty_m=0.0, detections=(),
    )


def neighbor(pos, sender="drone1"):
    return NeighborObservation(
        receiver_id="drone0", sender_id=sender, frame=Frame.LOCAL_ENU, measured_position_m=pos,
        measured_velocity_mps=(0.0, 0.0, 0.0), sample_timestamp_s=0.0, delivery_timestamp_s=0.0,
        packet_age_s=0.0, communication_confidence=0.9, stale=False,
    )


def candidate(vel=(0.0, 0.0, 0.0), t=0.0, ttl=1.0):
    return CandidateCommand(vehicle_id="drone0", desired_velocity_mps=vel, frame=Frame.LOCAL_ENU,
                              timestamp_s=t, expiration_time_s=t + ttl)


def ctx(**kwargs):
    kwargs.setdefault("geofence", GEOFENCE)
    return MissionContext(**kwargs)


# --- vertical command bounds / decoupled acceleration ---------------------

def test_bound_velocity_step_vertical_not_starved_by_large_horizontal_delta():
    """The confirmed Phase 4 root cause: a large horizontal target delta
    must not reduce the vertical correction below its own independent
    accel*dt budget."""
    own_vel = (0.0, 0.0, -3.0)          # falling
    target = (10.0, 10.0, 0.0)          # large horizontal escape + "stop falling"
    dt_s, max_accel = 0.1, 2.0
    out = bound_velocity_step(own_vel, target, dt_s, max_accel, max_turn_rate_radps=10.0, max_speed_mps=20.0)
    vertical_change = out[2] - own_vel[2]
    assert vertical_change >= max_accel * dt_s - 1e-6   # full vertical budget was used, not a starved fraction
    assert vertical_change <= max_accel * dt_s + 1e-6


def test_bound_velocity_step_vertical_bounded_independently():
    own_vel = (0.0, 0.0, 0.0)
    target = (0.0, 0.0, 100.0)
    out = bound_velocity_step(own_vel, target, dt_s=0.1, max_accel_mps2=2.0,
                                max_turn_rate_radps=10.0, max_speed_mps=50.0)
    assert out[2] <= 2.0 * 0.1 + 1e-6


# --- anti-oscillation / hysteresis ------------------------------------------

def test_repeated_alternating_override_suppressed_by_hysteresis():
    """Both a separation risk AND an obstacle risk are true every tick;
    without hysteresis the natural per-tick winner could still change if
    the two clearances see-saw around each other. With hysteresis, once
    one of them becomes the active tier it stays active for
    min_override_hold_s even while both remain individually true."""
    s = SafetySupervisor(SafetySupervisorConfig(min_override_hold_s=0.5, separation_hard_margin_m=0.3,
                                                    obstacle_hard_margin_m=0.3, vehicle_body_radius_m=0.06))
    tiers_seen = []
    for k in range(10):
        t = k * (1.0 / 24.0)
        d = s.evaluate(candidate(t=t), own_state(t=t), sensor_obs(ranges=(0.5,) + (math.inf,) * 15),
                        (neighbor((0.5, 0.0, 3.0)),), ctx(), now_s=t)
        tiers_seen.append(d.emergency_state)
    # Whichever fired first (SEPARATION_RISK or SAFE_HOLD) must be the
    # only one seen for the whole 10-tick (~0.4s) window - both conditions
    # are held true the entire time, so with hysteresis nothing should
    # flip mid-window (10 ticks at 1/24s ~= 0.42s < min_override_hold_s=0.5s).
    assert len(set(tiers_seen)) == 1


def test_hysteresis_does_not_block_escalation_to_operator_abort():
    """Hysteresis only applies among tiers 3/4/5(soft) - it must never
    delay operator abort or geofence, which always preempt immediately."""
    s = SafetySupervisor(SafetySupervisorConfig(min_override_hold_s=5.0))
    d1 = s.evaluate(candidate(t=0.0), own_state(t=0.0), sensor_obs(ranges=(0.5,) + (math.inf,) * 15),
                     (), ctx(), now_s=0.0)
    assert d1.emergency_state == SafetyState.SAFE_HOLD
    d2 = s.evaluate(candidate(t=0.01), own_state(t=0.01), sensor_obs(), (), ctx(operator_abort=True), now_s=0.01)
    assert d2.emergency_state == SafetyState.ABORT


def test_safety_event_logs_override_tier_transition():
    """Required item: safety event logging of override transitions."""
    s = SafetySupervisor(SafetySupervisorConfig(min_override_hold_s=0.1))
    s.evaluate(candidate(t=0.0), own_state(t=0.0), sensor_obs(ranges=(0.5,) + (math.inf,) * 15), (),
                ctx(), now_s=0.0)
    transition_events = [e for e in s.event_log if "override tier changed" in e.reason]
    assert transition_events
    assert "None -> 4" in transition_events[0].reason or "-> 4" in transition_events[0].reason


def test_hysteresis_suppression_is_logged():
    """Obstacle (tier 4) becomes active first, then separation (tier 3 -
    naturally HIGHER priority) also starts firing. Without hysteresis the
    natural first-priority-wins order would immediately swap the active
    tier from 4 to 3 the moment separation trips; hysteresis instead keeps
    tier 4 active for min_override_hold_s, logging the suppression."""
    s = SafetySupervisor(SafetySupervisorConfig(min_override_hold_s=1.0, separation_hard_margin_m=0.3,
                                                    obstacle_hard_margin_m=0.3, vehicle_body_radius_m=0.06))
    t0 = 0.0
    d0 = s.evaluate(candidate(t=t0), own_state(t=t0), sensor_obs(ranges=(0.5,) + (math.inf,) * 15),
                     (), ctx(), now_s=t0)
    assert d0.emergency_state == SafetyState.SAFE_HOLD   # tier 4 engaged first

    chosen_states = [d0.emergency_state]
    for k in range(1, 5):
        t = k * (1.0 / 24.0)
        d = s.evaluate(candidate(t=t), own_state(t=t), sensor_obs(ranges=(0.5,) + (math.inf,) * 15),
                        (neighbor((0.5, 0.0, 3.0)),), ctx(), now_s=t)   # separation now ALSO fires
        chosen_states.append(d.emergency_state)
    assert len(set(chosen_states)) == 1   # stayed on tier 4 throughout the hold window
    suppression_events = [e for e in s.event_log if "hysteresis: holding" in e.reason]
    assert suppression_events


# --- altitude-floor precedence ----------------------------------------------

def test_altitude_floor_precedes_active_obstacle_escape():
    s = SafetySupervisor(SafetySupervisorConfig())
    d = s.evaluate(candidate(vel=(0, 0, 0), t=0.0), own_state(pos=(0.0, 0.0, 0.55), vel=(0, 0, 0), t=0.0),
                    sensor_obs(ranges=(0.2,) + (math.inf,) * 15), (), ctx(contact_detected=True), now_s=0.0)
    assert "altitude_floor_critical" in d.active_constraints
    out = d.filtered_command.desired_velocity_mps
    assert out[0] == 0.0 and out[1] == 0.0 and out[2] > 0.0


# --- no ground contact: deterministic simplified kinematic test ------------

def test_no_ground_contact_in_deterministic_simplified_simulation():
    """Runs the REAL SafetySupervisor + bound_velocity_step over many
    ticks in a small, deterministic, PyBullet-free kinematic loop: a
    drone starting with a strong downward velocity while simultaneously
    needing a large horizontal separation escape. Before the Phase 4.1
    fixes (combined 3D accel bound, no hysteresis, no critical-altitude
    tier) this exact shape of scenario is what produced sustained descent
    into the ground plane; after the fixes, altitude must never reach 0."""
    cfg = SafetySupervisorConfig(min_override_hold_s=0.3, separation_hard_margin_m=0.5,
                                    vehicle_body_radius_m=0.06, max_accel_mps2=2.0)
    sup = SafetySupervisor(cfg)
    dt = 1.0 / 24.0
    z = 3.0
    vz = -2.0   # already descending hard at the start
    vx, vy = 0.0, 0.0
    for k in range(120):   # 5 seconds
        t = k * dt
        own = own_state(pos=(0.0, 0.0, z), vel=(vx, vy, vz), t=t)
        d = sup.evaluate(candidate(vel=(3.0, 0.0, 0.0), t=t), own, sensor_obs(),
                          (neighbor((0.4, 0.0, z)),), ctx(), now_s=t)
        assert z > 0.05, f"ground contact at t={t:.3f}s (z={z:.4f})"
        cmd = d.filtered_command
        if cmd is not None and cmd.desired_velocity_mps is not None:
            vx, vy, vz = cmd.desired_velocity_mps
        else:
            vx, vy, vz = 0.0, 0.0, 0.0
        z += vz * dt
    assert z > 0.05


# --- recovery after obstacle escape -----------------------------------------

def test_recovery_event_logged_after_obstacle_escape_clears():
    s = SafetySupervisor(SafetySupervisorConfig(stuck_violation_streak=1, escape_duration_s=0.05,
                                                    obstacle_hard_margin_m=0.3, vehicle_body_radius_m=0.06))
    t = 0.0
    # Trigger via direct contact - immediate stuck_detected/escape.
    s.evaluate(candidate(t=t), own_state(t=t), sensor_obs(), (), ctx(contact_detected=True), now_s=t)
    assert any(e.event_type == "stuck_detected" for e in s.event_log)
    # Advance past escape_duration_s with clearance now recovered (far obstacle).
    t = 0.2
    s.evaluate(candidate(t=t), own_state(t=t), sensor_obs(ranges=(5.0,) + (math.inf,) * 15), (),
                ctx(), now_s=t)
    recovery_events = [e for e in s.event_log if e.event_type == "recovery"]
    assert recovery_events
    assert "recovered" in recovery_events[0].reason
