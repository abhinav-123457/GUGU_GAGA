"""Phase 16C: the supervisor's vertical axis - see docs/PHASE16C_FLIGHT_LIFECYCLE.md.

Legacy bounds the combined 3D acceleration on the mission-candidate path, so on almost every tick the
vertical command is a fraction of the drone's OWN measured vertical speed (the coupling that turned noisy
estimated vertical speed into an altitude random walk in Phase 16B). `independent_vertical_axis` bounds the
vertical axis against the previous COMMAND instead; `preserve_candidate_vertical` stops horizontal overrides
from cancelling an altitude hold. Both default to the legacy behaviour."""
import math

import pytest

from swarm_sim.contracts import (
    CommandType, Frame, GeofenceSpec, HealthState, NeighborObservation, SafetyState, SensorObservation,
    VehicleState,
)
from swarm_sim.safety_supervisor import CandidateCommand, MissionContext, SafetySupervisor, SafetySupervisorConfig

GEOFENCE = GeofenceSpec(frame=Frame.LOCAL_ENU, center_m=(0.0, 0.0), half_extents_m=(20.0, 20.0),
                        floor_alt_m=0.5, ceiling_alt_m=8.0)
DT = 1.0 / 24.0
STEP = 4.0 * DT            # default max_accel_mps2 * one control tick


def own_state(pos=(0.0, 0.0, 3.0), vel=(1.0, 0.0, 0.0), t=1.0, batt=0.9):
    return VehicleState(
        vehicle_id="drone0", sim_time_s=t, frame=Frame.LOCAL_ENU, position_m=pos, velocity_mps=vel,
        acceleration_mps2=(0.0, 0.0, 0.0), attitude_rad=(0.0, 0.0, 0.0), angular_velocity_radps=(0.0, 0.0, 0.0),
        battery_fraction=batt, health_state=HealthState.OK, estimator_valid=True, last_valid_command_time_s=t,
    )


def sensor_obs(t=1.0):
    r = (math.inf,) * 16
    return SensorObservation(
        vehicle_id="drone0", sensor_timestamp_s=t, sensor_latency_s=0.0, fov_deg=270.0, range_returns_m=r,
        occluded=(False,) * 16, dropout=False, pose_uncertainty_m=0.0, detections=(),
    )


def neighbor(pos, t=1.0):
    return NeighborObservation(
        receiver_id="drone0", sender_id="drone1", frame=Frame.LOCAL_ENU, measured_position_m=pos,
        measured_velocity_mps=(0.0, 0.0, 0.0), sample_timestamp_s=t - 0.1, delivery_timestamp_s=t,
        packet_age_s=0.1, communication_confidence=0.9, stale=False,
    )


def candidate(vel, t=1.0):
    return CandidateCommand(vehicle_id="drone0", desired_velocity_mps=vel, frame=Frame.LOCAL_ENU,
                            timestamp_s=t, expiration_time_s=t + 1.0)


def ctx(**kw):
    return MissionContext(geofence=kw.pop("geofence", GEOFENCE), **kw)


def sup(independent=True, preserve=True, **kw):
    return SafetySupervisor(SafetySupervisorConfig(independent_vertical_axis=independent,
                                                   preserve_candidate_vertical=preserve, **kw))


def step(supervisor, cand_vel, own_pos=(0.0, 0.0, 3.0), own_vel=(1.0, 0.0, 0.0), t=1.0, neighbors=(), batt=0.9,
         **ctx_kw):
    return supervisor.evaluate(candidate(cand_vel, t), own_state(own_pos, own_vel, t, batt), sensor_obs(t), tuple(neighbors),
                               ctx(**ctx_kw), now_s=t)


def vz(decision):
    assert decision.accepted and decision.filtered_command.command_type == CommandType.VELOCITY_SETPOINT
    return decision.filtered_command.desired_velocity_mps[2]


class TestDefaults:
    def test_both_switches_default_to_legacy(self):
        cfg = SafetySupervisorConfig()
        assert cfg.independent_vertical_axis is False and cfg.preserve_candidate_vertical is False

    def test_legacy_vertical_command_follows_the_drones_own_vertical_speed(self):
        """Documents the coupling this phase removes: candidate vz = 0, own vz = -0.3, one horizontal accel
        limit firing -> the filtered vertical command is a fraction of the measured vertical speed."""
        d = step(sup(independent=False, preserve=False), (3.0, 0.0, 0.0), own_vel=(0.0, 0.0, -0.3))
        assert "accel_limit" in d.active_constraints
        assert -0.3 < vz(d) < -1e-3


class TestIndependentVerticalAxis:
    @pytest.mark.parametrize("own_vz", [-0.5, -0.1, 0.0, 0.4])
    def test_vertical_command_ignores_the_measured_vertical_speed(self, own_vz):
        d = step(sup(), (3.0, 0.0, 0.0), own_vel=(0.0, 0.0, own_vz))
        assert vz(d) == 0.0

    def test_a_noisy_vertical_speed_reading_never_moves_the_command(self):
        s = sup()
        out = [vz(step(s, (1.0, 0.0, 0.0), own_vel=(1.0, 0.0, noise), t=1.0 + k * DT))
               for k, noise in enumerate([0.3, -0.25, 0.4, -0.35, 0.2, -0.3])]
        assert out == [0.0] * 6

    def test_vertical_speed_ramps_at_the_acceleration_limit_from_the_previous_command(self):
        s = sup()
        first = vz(step(s, (0.0, 0.0, 1.0), own_vel=(0.0, 0.0, 0.0), t=1.0))
        second = vz(step(s, (0.0, 0.0, 1.0), own_vel=(0.0, 0.0, 0.0), t=1.0 + DT))
        assert first == pytest.approx(STEP) and second == pytest.approx(2 * STEP)

    def test_a_large_horizontal_delta_does_not_starve_the_vertical_axis(self):
        d = step(sup(), (5.0, 0.0, 0.1), own_vel=(0.0, 0.0, 0.0))
        assert "accel_limit" in d.active_constraints                 # horizontal limited ...
        assert vz(d) == pytest.approx(0.1)                            # ... vertical delivered in full

    def test_the_horizontal_limit_is_unchanged_by_the_vertical_axis(self):
        legacy = step(sup(independent=False, preserve=False), (5.0, 0.0, 0.0), own_vel=(0.0, 0.0, 0.0))
        new = step(sup(), (5.0, 0.0, 0.0), own_vel=(0.0, 0.0, 0.0))
        assert new.filtered_command.desired_velocity_mps[:2] == pytest.approx(
            legacy.filtered_command.desired_velocity_mps[:2])

    def test_a_hold_resets_the_reference_so_the_next_ramp_starts_from_zero(self):
        s = sup()
        for k in range(4):
            step(s, (0.0, 0.0, 1.0), own_vel=(0.0, 0.0, 0.0), t=1.0 + k * DT)
        hold = step(s, (1.0, 0.0, 1.0), own_pos=(25.0, 0.0, 3.0), t=1.0 + 4 * DT)     # outside the geofence
        assert hold.filtered_command.command_type == CommandType.HOLD
        after = vz(step(s, (0.0, 0.0, 1.0), own_vel=(0.0, 0.0, 0.0), t=1.0 + 5 * DT))
        assert after == pytest.approx(STEP)


class TestOverridesKeepTheAltitudeCommand:
    CLOSE = (0.25, 0.0, 3.0)

    def test_a_separation_override_keeps_the_candidate_vertical_speed(self):
        s = sup()
        d = step(s, (1.0, 0.0, 0.1), own_vel=(1.0, 0.0, 0.0), neighbors=[neighbor(self.CLOSE)])
        assert d.emergency_state == SafetyState.SEPARATION_RISK
        assert vz(d) == pytest.approx(0.1)

    def test_legacy_separation_override_commands_zero_vertical_speed(self):
        d = step(sup(independent=False, preserve=False), (1.0, 0.0, 0.1), own_vel=(1.0, 0.0, 0.0),
                 neighbors=[neighbor(self.CLOSE)])
        assert d.emergency_state == SafetyState.SEPARATION_RISK
        assert vz(d) == pytest.approx(0.0, abs=1e-9)

    def test_the_horizontal_geofence_approach_response_keeps_the_candidate_vertical_speed(self):
        d = step(sup(), (2.0, 0.0, 0.1), own_pos=(18.5, 0.0, 3.0), own_vel=(2.0, 0.0, 0.0))
        assert d.emergency_state == SafetyState.GEOFENCE_RISK and "geofence_boundary" in d.active_constraints
        assert vz(d) == pytest.approx(0.1)

    def test_safe_point_return_keeps_the_candidate_vertical_speed(self):
        d = step(sup(), (1.0, 0.0, 0.1), own_pos=(5.0, 0.0, 3.0), own_vel=(1.0, 0.0, 0.0), batt=0.15,
                 safe_point_m=(0.0, 0.0, 0.0))
        assert d.emergency_state == SafetyState.RETURN_TO_SAFE_POINT
        assert d.filtered_command.desired_velocity_mps[0] < 1.0        # already braking towards the safe point (rate-limited)
        assert vz(d) == pytest.approx(0.1)

    def test_legacy_safe_point_return_commands_zero_vertical_speed(self):
        d = step(sup(independent=False, preserve=False), (1.0, 0.0, 0.1), own_pos=(5.0, 0.0, 3.0),
                 own_vel=(1.0, 0.0, 0.0), batt=0.15, safe_point_m=(0.0, 0.0, 0.0))
        assert d.emergency_state == SafetyState.RETURN_TO_SAFE_POINT
        assert vz(d) == pytest.approx(0.0, abs=1e-9)

    def test_the_candidate_vertical_speed_is_clamped_to_the_speed_limit(self):
        s = sup(max_speed_mps=1.0)
        d = step(s, (1.0, 0.0, 50.0), own_vel=(1.0, 0.0, 0.0), neighbors=[neighbor(self.CLOSE)])
        assert abs(vz(d)) <= 1.0 + 1e-9


class TestAltitudeProtectionStillOwnsTheVerticalAxis:
    @pytest.mark.parametrize("independent, preserve", [(False, False), (True, True)])
    def test_floor_recovery_climbs_even_when_the_candidate_descends(self, independent, preserve):
        d = step(sup(independent, preserve), (1.0, 0.0, -0.5), own_pos=(0.0, 0.0, 0.7), own_vel=(1.0, 0.0, 0.0))
        assert "altitude_floor" in d.active_constraints or "altitude_floor_critical" in d.active_constraints
        assert vz(d) > 0.0

    @pytest.mark.parametrize("independent, preserve", [(False, False), (True, True)])
    def test_ceiling_recovery_descends_even_when_the_candidate_climbs(self, independent, preserve):
        d = step(sup(independent, preserve), (1.0, 0.0, 0.5), own_pos=(0.0, 0.0, 7.7), own_vel=(1.0, 0.0, 0.0))
        assert "altitude_ceiling" in d.active_constraints
        assert vz(d) < 0.0


class TestRobustness:
    def test_a_malformed_candidate_is_still_rejected(self):
        d = step(sup(), (float("nan"), 0.0, 0.0))
        assert d.accepted is False and "command_malformed" in d.active_constraints

    def test_a_non_finite_vertical_component_is_rejected_not_propagated(self):
        s = sup()
        d = step(s, (1.0, 0.0, float("inf")), neighbors=[neighbor((0.25, 0.0, 3.0))])
        assert d.accepted is False or math.isfinite(vz(d))
