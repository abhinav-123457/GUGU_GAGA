"""Phase 15A/15B functional tests for the Swarm124 policy adapter. See
docs/PHASE15_SWARM124_LANGOSTINO_INTEGRATION.md for what is verified from
swarm-subnet/swarm's own README versus what this module infers.
"""
from __future__ import annotations

import dataclasses
import math

from swarm_sim.contracts import Frame
from swarm_sim.external.external_contracts import ExternalActionRejection
from swarm_sim.external.swarm124_policy_adapter import (
    DEPTH_BEAM_COUNT, MAX_TEAMMATES, ScriptedSwarm124Policy, Swarm124Observation, convert_swarm124_action_to_candidate,
    validate_observation,
)
from swarm_sim.safety_supervisor import MissionContext, SafetySupervisor

_KNOWN = ("drone0",)


def _observation(**overrides) -> Swarm124Observation:
    defaults = dict(
        vehicle_id="drone0", timestamp_s=0.0, position_m=(0.0, 0.0, 1.0), velocity_mps=(0.0, 0.0, 0.0),
        depth_returns_m=tuple([5.0] * DEPTH_BEAM_COUNT), teammate_relative_m=(), seed=1, run_id="run-1",
    )
    defaults.update(overrides)
    return Swarm124Observation(**defaults)


def _policy(target_xy=(5.0, 5.0), max_speed_mps=0.25, seed=1, run_id="run-1") -> ScriptedSwarm124Policy:
    return ScriptedSwarm124Policy(target_xy=target_xy, max_speed_mps=max_speed_mps, seed=seed, run_id=run_id)


class TestScriptedPolicyIsDeterministic:
    def test_same_observation_and_config_produce_the_same_action(self):
        obs = _observation()
        a1 = _policy().act(obs, action_expiry_s=2.0)
        a2 = _policy().act(obs, action_expiry_s=2.0)
        assert a1 == a2

    def test_speed_never_exceeds_configured_max(self):
        obs = _observation(position_m=(0.0, 0.0, 1.0))
        action = _policy(target_xy=(1000.0, 1000.0), max_speed_mps=0.25).act(obs, action_expiry_s=2.0)
        assert action.speed_mps <= 0.25 + 1e-9

    def test_at_target_produces_zero_speed(self):
        obs = _observation(position_m=(5.0, 5.0, 1.0))
        action = _policy(target_xy=(5.0, 5.0)).act(obs, action_expiry_s=2.0)
        assert action.speed_mps == 0.0

    def test_scripted_policy_has_no_model_checksum(self):
        # honest, not fabricated - there is no model file for a scripted policy
        assert ScriptedSwarm124Policy.model_checksum is None


class TestObservationValidation:
    def test_correct_shape_observation_is_valid(self):
        assert validate_observation(_observation()) is None

    def test_wrong_depth_length_is_rejected(self):
        obs = _observation(depth_returns_m=(1.0, 2.0))
        assert validate_observation(obs) is not None

    def test_too_many_teammates_is_rejected(self):
        obs = _observation(teammate_relative_m=tuple((1.0, 1.0, 0.0) for _ in range(MAX_TEAMMATES + 1)))
        assert validate_observation(obs) is not None

    def test_malformed_teammate_entry_is_rejected(self):
        obs = _observation(teammate_relative_m=((1.0, 1.0),))  # 2-tuple, not 3
        assert validate_observation(obs) is not None

    def test_up_to_max_teammates_is_valid(self):
        obs = _observation(teammate_relative_m=tuple((float(i), 0.0, 0.0) for i in range(MAX_TEAMMATES)))
        assert validate_observation(obs) is None


class TestActionConversion:
    def test_normal_action_converts_to_an_accepted_candidate(self):
        action = _policy().act(_observation(), action_expiry_s=2.0)
        result = convert_swarm124_action_to_candidate(
            action, known_vehicle_ids=_KNOWN, now_s=0.0, max_speed_mps=0.25, max_accel_mps2=1.0, max_staleness_s=1.0,
        )
        assert result.accepted is True
        assert result.candidate.frame == Frame.LOCAL_ENU

    def test_direction_is_normalized_before_scaling_by_speed(self):
        action = dataclasses.replace(_policy().act(_observation(), action_expiry_s=2.0),
                                      direction_xy=(3.0, 4.0), speed_mps=0.25)  # not a unit vector
        result = convert_swarm124_action_to_candidate(
            action, known_vehicle_ids=_KNOWN, now_s=0.0, max_speed_mps=0.25, max_accel_mps2=1.0, max_staleness_s=1.0,
        )
        vx, vy, vz = result.candidate.desired_velocity_mps
        assert math.isclose(math.hypot(vx, vy), 0.25, abs_tol=1e-9)
        assert vz == 0.0

    def test_zero_direction_vector_produces_zero_velocity_not_nan(self):
        action = dataclasses.replace(_policy().act(_observation(), action_expiry_s=2.0), direction_xy=(0.0, 0.0))
        result = convert_swarm124_action_to_candidate(
            action, known_vehicle_ids=_KNOWN, now_s=0.0, max_speed_mps=0.25, max_accel_mps2=1.0, max_staleness_s=1.0,
        )
        assert result.accepted is True
        assert result.candidate.desired_velocity_mps == (0.0, 0.0, 0.0)

    def test_nan_direction_component_is_rejected_not_silently_zeroed(self):
        """Regression: an earlier version of this converter computed
        direction/norm without checking finiteness first, so a NaN
        component's norm was also NaN, which compared False against the
        near-zero threshold and fell through to the "no direction"
        default - silently turning a malformed action into an innocuous
        zero-velocity command instead of rejecting it."""
        action = dataclasses.replace(_policy().act(_observation(), action_expiry_s=2.0),
                                      direction_xy=(float("nan"), 0.0))
        result = convert_swarm124_action_to_candidate(
            action, known_vehicle_ids=_KNOWN, now_s=0.0, max_speed_mps=0.25, max_accel_mps2=1.0, max_staleness_s=1.0,
        )
        assert result.accepted is False
        assert result.rejection == ExternalActionRejection.NAN_OR_INF_VALUE

    def test_infinite_speed_is_rejected(self):
        action = dataclasses.replace(_policy().act(_observation(), action_expiry_s=2.0), speed_mps=float("inf"))
        result = convert_swarm124_action_to_candidate(
            action, known_vehicle_ids=_KNOWN, now_s=0.0, max_speed_mps=0.25, max_accel_mps2=1.0, max_staleness_s=1.0,
        )
        assert result.accepted is False

    def test_malformed_direction_shape_is_rejected_as_unexpected_dimensions(self):
        action = dataclasses.replace(_policy().act(_observation(), action_expiry_s=2.0),
                                      direction_xy=(1.0, 0.0, 0.0))
        result = convert_swarm124_action_to_candidate(
            action, known_vehicle_ids=_KNOWN, now_s=0.0, max_speed_mps=0.25, max_accel_mps2=1.0, max_staleness_s=1.0,
        )
        assert result.accepted is False
        assert result.rejection == ExternalActionRejection.UNEXPECTED_ACTION_DIMENSIONS

    def test_turn_rate_is_never_forwarded_into_the_candidate(self):
        """CandidateCommand has no yaw/turn field - see the converter's own
        docstring for why turn_rate_radps is deliberately dropped, not
        silently lost by accident."""
        action = dataclasses.replace(_policy().act(_observation(), action_expiry_s=2.0), turn_rate_radps=99.0)
        result = convert_swarm124_action_to_candidate(
            action, known_vehicle_ids=_KNOWN, now_s=0.0, max_speed_mps=0.25, max_accel_mps2=1.0, max_staleness_s=1.0,
        )
        assert not hasattr(result.candidate, "turn_rate_radps")
        assert not hasattr(result.candidate, "yaw_rate_radps")


class TestNeverBypassesSafetySupervisor:
    def test_accepted_candidate_still_needs_a_safety_decision_to_become_sendable(self):
        """The adapter's OWN output (an ExternalAdapterResult with a
        CandidateCommand) is not itself sendable - it must still go
        through SafetySupervisor.evaluate() exactly like every other
        candidate in the project, producing a SafetyDecision before any
        AdapterCommand could ever be built. This test drives that real
        path end to end, the same way SARMission.tick() does."""
        action = _policy(target_xy=(5.0, 5.0)).act(_observation(), action_expiry_s=2.0)
        result = convert_swarm124_action_to_candidate(
            action, known_vehicle_ids=_KNOWN, now_s=0.0, max_speed_mps=0.25, max_accel_mps2=1.0, max_staleness_s=1.0,
        )
        assert result.accepted is True

        supervisor = SafetySupervisor()
        own_state = _vehicle_state()
        from swarm_sim.contracts import GeofenceSpec
        geofence = GeofenceSpec(frame=Frame.LOCAL_ENU, center_m=(5.0, 5.0), half_extents_m=(20.0, 20.0),
                                 floor_alt_m=-2.0, ceiling_alt_m=5.0)
        decision = supervisor.evaluate(result.candidate, own_state, _empty_observation(), (),
                                        MissionContext(geofence=geofence), now_s=0.0)
        assert decision.accepted is True
        assert decision.filtered_command is not None

    def test_a_rejected_external_action_never_reaches_the_supervisor_with_a_candidate(self):
        action = dataclasses.replace(_policy().act(_observation(), action_expiry_s=2.0),
                                      direction_xy=(float("nan"), 0.0))
        result = convert_swarm124_action_to_candidate(
            action, known_vehicle_ids=_KNOWN, now_s=0.0, max_speed_mps=0.25, max_accel_mps2=1.0, max_staleness_s=1.0,
        )
        assert result.accepted is False
        assert result.candidate is None


def _vehicle_state():
    from swarm_sim.contracts import HealthState, VehicleState
    return VehicleState(
        vehicle_id="drone0", sim_time_s=0.0, frame=Frame.LOCAL_ENU, position_m=(0.0, 0.0, 1.0),
        velocity_mps=(0.0, 0.0, 0.0), acceleration_mps2=(0.0, 0.0, 0.0), attitude_rad=(0.0, 0.0, 0.0),
        angular_velocity_radps=(0.0, 0.0, 0.0), battery_fraction=1.0, health_state=HealthState.OK,
        estimator_valid=True, last_valid_command_time_s=None,
    )


def _empty_observation():
    from swarm_sim.contracts import SensorObservation
    return SensorObservation(vehicle_id="drone0", sensor_timestamp_s=0.0, sensor_latency_s=0.0, fov_deg=90.0,
                              range_returns_m=(), occluded=(), dropout=False, pose_uncertainty_m=0.0, detections=())
