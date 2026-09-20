"""Phase 15A functional tests for swarm_sim/external/external_contracts.py's
shared validation pipeline. See docs/PHASE15_SWARM124_LANGOSTINO_INTEGRATION.md.
"""
from __future__ import annotations

from swarm_sim.contracts import Frame
from swarm_sim.external.external_contracts import (
    ExternalActionRejection, ExternalPolicyAction, validate_and_build_candidate, validate_observation_shape,
)

_KNOWN = ("drone0",)


def _action(**overrides) -> ExternalPolicyAction:
    defaults = dict(
        vehicle_id="drone0", desired_velocity_mps=(0.1, 0.1, 0.0), frame=Frame.LOCAL_ENU, timestamp_s=0.0,
        action_expiry_s=2.0, policy_version="v1", seed=1, run_id="run-1", model_checksum="abc123",
    )
    defaults.update(overrides)
    return ExternalPolicyAction(**defaults)


def _validate(action, **overrides):
    defaults = dict(known_vehicle_ids=_KNOWN, now_s=0.0, expected_frame=Frame.LOCAL_ENU,
                     max_speed_mps=0.25, max_accel_mps2=1.0, max_staleness_s=1.0)
    defaults.update(overrides)
    return validate_and_build_candidate(action, **defaults)


class TestAcceptedPath:
    def test_well_formed_action_is_accepted_and_produces_a_candidate(self):
        result = _validate(_action())
        assert result.accepted is True
        assert result.candidate is not None
        assert result.candidate.vehicle_id == "drone0"
        assert result.candidate.frame == Frame.LOCAL_ENU

    def test_policy_version_and_model_checksum_are_recorded_on_acceptance(self):
        result = _validate(_action(policy_version="scripted-v2", model_checksum="deadbeef"))
        assert result.policy_version == "scripted-v2"
        assert result.model_checksum == "deadbeef"

    def test_seed_and_run_id_are_recorded_on_acceptance(self):
        result = _validate(_action(seed=42, run_id="epoch-7"))
        assert result.seed == 42
        assert result.run_id == "epoch-7"


class TestRejections:
    def test_nan_velocity_component_is_rejected(self):
        result = _validate(_action(desired_velocity_mps=(float("nan"), 0.0, 0.0)))
        assert result.accepted is False
        assert result.rejection == ExternalActionRejection.NAN_OR_INF_VALUE

    def test_infinite_velocity_component_is_rejected(self):
        result = _validate(_action(desired_velocity_mps=(float("inf"), 0.0, 0.0)))
        assert result.accepted is False
        assert result.rejection == ExternalActionRejection.NAN_OR_INF_VALUE

    def test_wrong_action_dimensions_are_rejected(self):
        result = _validate(_action(desired_velocity_mps=(0.1, 0.1)))
        assert result.accepted is False
        assert result.rejection == ExternalActionRejection.UNEXPECTED_ACTION_DIMENSIONS

    def test_non_frame_value_is_rejected_as_wrong_units(self):
        result = _validate(_action(frame="LOCAL_ENU"))  # a bare string, not a Frame member
        assert result.accepted is False
        assert result.rejection == ExternalActionRejection.WRONG_UNITS

    def test_wrong_frame_is_rejected(self):
        result = _validate(_action(frame=Frame.LOCAL_NED))
        assert result.accepted is False
        assert result.rejection == ExternalActionRejection.WRONG_FRAME

    def test_stale_action_is_rejected(self):
        result = _validate(_action(timestamp_s=-10.0), now_s=0.0, max_staleness_s=1.0)
        assert result.accepted is False
        assert result.rejection == ExternalActionRejection.STALE_OUTPUT

    def test_future_timestamp_is_rejected_as_stale(self):
        result = _validate(_action(timestamp_s=100.0), now_s=0.0)
        assert result.accepted is False
        assert result.rejection == ExternalActionRejection.STALE_OUTPUT

    def test_expired_action_is_rejected(self):
        result = _validate(_action(action_expiry_s=-1.0), now_s=0.0)
        assert result.accepted is False
        assert result.rejection == ExternalActionRejection.EXPIRED_OUTPUT

    def test_unknown_vehicle_id_is_rejected(self):
        result = _validate(_action(vehicle_id="drone9"))
        assert result.accepted is False
        assert result.rejection == ExternalActionRejection.UNKNOWN_VEHICLE_ID

    def test_duplicate_vehicle_id_in_known_list_is_rejected(self):
        result = _validate(_action(), known_vehicle_ids=("drone0", "drone0"))
        assert result.accepted is False
        assert result.rejection == ExternalActionRejection.DUPLICATE_VEHICLE_ID

    def test_speed_beyond_configured_limit_is_rejected(self):
        result = _validate(_action(desired_velocity_mps=(99.0, 0.0, 0.0)), max_speed_mps=0.25)
        assert result.accepted is False
        assert result.rejection == ExternalActionRejection.SPEED_LIMIT_EXCEEDED

    def test_acceleration_beyond_configured_limit_is_rejected(self):
        result = _validate(
            _action(timestamp_s=1.0, desired_velocity_mps=(1.0, 0.0, 0.0)),
            now_s=1.0, max_speed_mps=1.0, max_accel_mps2=0.01, max_staleness_s=5.0,
            previous_velocity_mps=(0.0, 0.0, 0.0), previous_timestamp_s=0.0,
        )
        assert result.accepted is False
        assert result.rejection == ExternalActionRejection.ACCEL_LIMIT_EXCEEDED

    def test_missing_policy_version_is_rejected(self):
        result = _validate(_action(policy_version=None))
        assert result.accepted is False
        assert result.rejection == ExternalActionRejection.MISSING_POLICY_VERSION

    def test_empty_policy_version_is_rejected(self):
        result = _validate(_action(policy_version=""))
        assert result.accepted is False
        assert result.rejection == ExternalActionRejection.MISSING_POLICY_VERSION

    def test_missing_seed_is_rejected(self):
        result = _validate(_action(seed=None))
        assert result.accepted is False
        assert result.rejection == ExternalActionRejection.MISSING_SEED_OR_RUN_ID

    def test_missing_run_id_is_rejected(self):
        result = _validate(_action(run_id=None))
        assert result.accepted is False
        assert result.rejection == ExternalActionRejection.MISSING_SEED_OR_RUN_ID

    def test_rejected_action_never_produces_a_candidate(self):
        result = _validate(_action(desired_velocity_mps=(float("nan"), 0.0, 0.0)))
        assert result.candidate is None


class TestObservationShapeValidation:
    def test_valid_observation_returns_none(self):
        assert validate_observation_shape("depth", (1.0, 2.0, 3.0), 3) is None

    def test_wrong_length_is_rejected(self):
        assert validate_observation_shape("depth", (1.0, 2.0), 3) is not None

    def test_non_tuple_is_rejected(self):
        assert validate_observation_shape("depth", [1.0, 2.0, 3.0], 3) is not None

    def test_nan_element_is_rejected(self):
        assert validate_observation_shape("depth", (1.0, float("nan"), 3.0), 3) is not None
