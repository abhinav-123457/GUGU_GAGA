import math

import pytest

from swarm_sim import contracts as c


def _obstacle():
    return c.Obstacle(obstacle_id="obs0", position_m=(1.0, 2.0), radius_m=1.0)


def _geofence():
    return c.GeofenceSpec(
        frame=c.Frame.LOCAL_ENU, center_m=(0.0, 0.0), half_extents_m=(20.0, 20.0),
        floor_alt_m=0.5, ceiling_alt_m=10.0,
    )


def _victim():
    return c.VictimGroundTruth(victim_id="v0", position_m=(5.0, 5.0), found=False)


def _detection():
    return c.DetectionCandidate(
        candidate_id="d0", position_m=(3.0, 4.0), frame=c.Frame.LOCAL_ENU,
        confidence=0.8, localization_uncertainty_m=0.5,
    )


def _vehicle_state(vehicle_id="drone0"):
    return c.VehicleState(
        vehicle_id=vehicle_id, sim_time_s=1.0, frame=c.Frame.LOCAL_ENU,
        position_m=(0.0, 0.0, 3.0), velocity_mps=(1.0, 0.0, 0.0),
        acceleration_mps2=(0.0, 0.0, 0.0), attitude_rad=(0.0, 0.0, 0.0),
        angular_velocity_radps=(0.0, 0.0, 0.0), battery_fraction=0.9,
        health_state=c.HealthState.OK, estimator_valid=True,
        last_valid_command_time_s=0.5,
    )


def _command(vehicle_id="drone0", timestamp_s=1.0, expiration_time_s=2.0):
    return c.Command(
        vehicle_id=vehicle_id, command_type=c.CommandType.VELOCITY_SETPOINT,
        frame=c.Frame.LOCAL_ENU, desired_position_m=None,
        desired_velocity_mps=(1.0, 0.0, 0.0), yaw_rad=None, yaw_rate_radps=None,
        timestamp_s=timestamp_s, expiration_time_s=expiration_time_s,
        source="swarm_controller", confidence=0.9,
    )


def _safety_decision(filtered=None, accepted=None, vehicle_id="drone0"):
    # Policy: accepted <=> a filtered_command is present. Default `accepted`
    # to match whatever `filtered` was given, so existing call sites that
    # only vary `filtered` still build a valid, consistent SafetyDecision.
    if accepted is None:
        accepted = filtered is not None
    return c.SafetyDecision(
        vehicle_id=vehicle_id, sim_time_s=1.0, accepted=accepted, filtered_command=filtered,
        active_constraints=("geofence",), reason="nominal", emergency_state=c.SafetyState.NORMAL,
        min_predicted_clearance_m=2.0, time_to_collision_s=None,
    )


# ---------------------------------------------------------------------------
# Construction of every dataclass
# ---------------------------------------------------------------------------

def test_construct_world_state():
    ws = c.WorldState(
        sim_time_s=0.0, step_index=0, dt_s=1.0 / 24.0, frame=c.Frame.LOCAL_ENU,
        obstacles=(_obstacle(),), geofence=_geofence(), victims_ground_truth=(_victim(),),
    )
    assert ws.contract_version == c.CONTRACT_VERSION
    assert len(ws.obstacles) == 1


def test_construct_vehicle_state():
    vs = _vehicle_state()
    assert vs.battery_fraction == pytest.approx(0.9)


def test_construct_sensor_observation():
    obs = c.SensorObservation(
        vehicle_id="drone0", sensor_timestamp_s=1.0, sensor_latency_s=0.1, fov_deg=120.0,
        range_returns_m=(1.0, 2.5, math.inf), occluded=(False, False, False),
        dropout=False, pose_uncertainty_m=0.2, detections=(_detection(),),
    )
    assert len(obs.range_returns_m) == 3
    assert math.isinf(obs.range_returns_m[2])


def test_construct_neighbor_observation():
    n = c.NeighborObservation(
        receiver_id="drone0", sender_id="drone1", frame=c.Frame.LOCAL_ENU,
        measured_position_m=(1.0, 1.0, 3.0), measured_velocity_mps=(0.0, 0.0, 0.0),
        sample_timestamp_s=1.0, delivery_timestamp_s=1.2, packet_age_s=0.2,
        communication_confidence=0.7, stale=False,
    )
    assert n.packet_age_s == pytest.approx(0.2)


def test_construct_command():
    cmd = _command()
    assert cmd.command_type is c.CommandType.VELOCITY_SETPOINT
    assert not cmd.is_expired(1.5)
    assert cmd.is_expired(2.5)


def test_construct_safety_decision():
    sd = _safety_decision(filtered=_command())
    assert sd.emergency_state is c.SafetyState.NORMAL
    assert sd.filtered_command.vehicle_id == "drone0"


# ---------------------------------------------------------------------------
# Invalid-shape rejection
# ---------------------------------------------------------------------------

def test_vec3_wrong_length_rejected():
    with pytest.raises(ValueError):
        c.VehicleState(
            vehicle_id="d0", sim_time_s=0.0, frame=c.Frame.LOCAL_ENU,
            position_m=(0.0, 0.0),  # wrong length
            velocity_mps=(0.0, 0.0, 0.0), acceleration_mps2=(0.0, 0.0, 0.0),
            attitude_rad=(0.0, 0.0, 0.0), angular_velocity_radps=(0.0, 0.0, 0.0),
            battery_fraction=1.0, health_state=c.HealthState.OK,
            estimator_valid=True, last_valid_command_time_s=None,
        )


def test_vec2_wrong_length_rejected():
    with pytest.raises(ValueError):
        c.Obstacle(obstacle_id="o0", position_m=(1.0, 2.0, 3.0), radius_m=1.0)


def test_occluded_length_mismatch_rejected():
    with pytest.raises(ValueError):
        c.SensorObservation(
            vehicle_id="d0", sensor_timestamp_s=0.0, sensor_latency_s=0.0, fov_deg=90.0,
            range_returns_m=(1.0, 2.0), occluded=(False,),  # length mismatch
            dropout=False, pose_uncertainty_m=0.0, detections=(),
        )


# ---------------------------------------------------------------------------
# NaN / infinity rejection (and the one place +inf is legitimately allowed)
# ---------------------------------------------------------------------------

def test_nan_position_component_rejected():
    with pytest.raises(ValueError):
        c.Obstacle(obstacle_id="o0", position_m=(float("nan"), 0.0), radius_m=1.0)


def test_infinite_position_component_rejected():
    with pytest.raises(ValueError):
        c.Obstacle(obstacle_id="o0", position_m=(float("inf"), 0.0), radius_m=1.0)


def test_nan_range_return_rejected():
    with pytest.raises(ValueError):
        c.SensorObservation(
            vehicle_id="d0", sensor_timestamp_s=0.0, sensor_latency_s=0.0, fov_deg=90.0,
            range_returns_m=(float("nan"),), occluded=(False,),
            dropout=False, pose_uncertainty_m=0.0, detections=(),
        )


def test_infinite_range_return_allowed():
    obs = c.SensorObservation(
        vehicle_id="d0", sensor_timestamp_s=0.0, sensor_latency_s=0.0, fov_deg=90.0,
        range_returns_m=(math.inf,), occluded=(False,),
        dropout=False, pose_uncertainty_m=0.0, detections=(),
    )
    assert math.isinf(obs.range_returns_m[0])


# ---------------------------------------------------------------------------
# Timestamp / expiry validation
# ---------------------------------------------------------------------------

def test_command_expiry_before_timestamp_rejected():
    with pytest.raises(ValueError):
        _command(timestamp_s=2.0, expiration_time_s=1.0)


def test_command_expiry_equal_timestamp_rejected():
    with pytest.raises(ValueError):
        _command(timestamp_s=1.0, expiration_time_s=1.0)


def test_negative_timestamp_rejected():
    with pytest.raises(ValueError):
        _command(timestamp_s=-1.0, expiration_time_s=1.0)


def test_neighbor_observation_packet_age_mismatch_rejected():
    with pytest.raises(ValueError):
        c.NeighborObservation(
            receiver_id="drone0", sender_id="drone1", frame=c.Frame.LOCAL_ENU,
            measured_position_m=(0.0, 0.0, 0.0), measured_velocity_mps=(0.0, 0.0, 0.0),
            sample_timestamp_s=1.0, delivery_timestamp_s=1.2,
            packet_age_s=0.5,  # should be 0.2
            communication_confidence=0.5, stale=False,
        )


def test_neighbor_observation_delivery_before_sample_rejected():
    with pytest.raises(ValueError):
        c.NeighborObservation(
            receiver_id="drone0", sender_id="drone1", frame=c.Frame.LOCAL_ENU,
            measured_position_m=(0.0, 0.0, 0.0), measured_velocity_mps=(0.0, 0.0, 0.0),
            sample_timestamp_s=2.0, delivery_timestamp_s=1.0, packet_age_s=-1.0,
            communication_confidence=0.5, stale=False,
        )


# ---------------------------------------------------------------------------
# Coordinate-frame / enum validation
# ---------------------------------------------------------------------------

def test_invalid_frame_type_rejected():
    with pytest.raises(ValueError):
        c.DetectionCandidate(
            candidate_id="d0", position_m=(0.0, 0.0), frame="LOCAL_ENU",  # string, not Frame
            confidence=0.5, localization_uncertainty_m=0.1,
        )


def test_invalid_command_type_rejected():
    with pytest.raises(ValueError):
        c.Command(
            vehicle_id="d0", command_type="VELOCITY_SETPOINT", frame=c.Frame.LOCAL_ENU,
            desired_position_m=None, desired_velocity_mps=(0.0, 0.0, 0.0),
            yaw_rad=None, yaw_rate_radps=None, timestamp_s=0.0, expiration_time_s=1.0,
            source="x", confidence=0.5,
        )


def test_confidence_out_of_range_rejected():
    with pytest.raises(ValueError):
        c.DetectionCandidate(
            candidate_id="d0", position_m=(0.0, 0.0), frame=c.Frame.LOCAL_ENU,
            confidence=1.5, localization_uncertainty_m=0.1,
        )


# ---------------------------------------------------------------------------
# Serialization round trips
# ---------------------------------------------------------------------------

def test_command_to_dict_from_dict_round_trip():
    cmd = _command()
    d = c.to_dict(cmd)
    assert d["command_type"] == "VELOCITY_SETPOINT"
    restored = c.from_dict(c.Command, d)
    assert restored == cmd


def test_safety_decision_round_trip_with_nested_command():
    sd = _safety_decision(filtered=_command())
    d = c.to_dict(sd)
    assert d["emergency_state"] == "NORMAL"
    assert d["filtered_command"]["vehicle_id"] == "drone0"
    restored = c.from_dict(c.SafetyDecision, d)
    assert restored == sd


def test_safety_decision_round_trip_when_rejected():
    sd = _safety_decision(filtered=None)  # accepted=False, per the helper's policy
    assert sd.accepted is False
    d = c.to_dict(sd)
    assert d["filtered_command"] is None
    restored = c.from_dict(c.SafetyDecision, d)
    assert restored == sd


def test_world_state_round_trip_with_nested_tuples():
    ws = c.WorldState(
        sim_time_s=1.0, step_index=24, dt_s=1.0 / 24.0, frame=c.Frame.LOCAL_ENU,
        obstacles=(_obstacle(), _obstacle()), geofence=_geofence(),
        victims_ground_truth=(_victim(),),
    )
    d = c.to_dict(ws)
    assert len(d["obstacles"]) == 2
    restored = c.from_dict(c.WorldState, d)
    assert restored == ws


# ---------------------------------------------------------------------------
# is_expired boundary: expired AT the expiration timestamp, not just after
# ---------------------------------------------------------------------------

def test_command_is_expired_exactly_at_expiration_timestamp():
    cmd = _command(timestamp_s=1.0, expiration_time_s=2.0)
    assert cmd.is_expired(2.0) is True
    assert cmd.is_expired(1.999999) is False
    assert cmd.is_expired(2.000001) is True


# ---------------------------------------------------------------------------
# Booleans rejected wherever a genuine number/int is required
# ---------------------------------------------------------------------------

def test_bool_rejected_as_battery_fraction():
    with pytest.raises(ValueError):
        c.VehicleState(
            vehicle_id="d0", sim_time_s=0.0, frame=c.Frame.LOCAL_ENU,
            position_m=(0.0, 0.0, 0.0), velocity_mps=(0.0, 0.0, 0.0),
            acceleration_mps2=(0.0, 0.0, 0.0), attitude_rad=(0.0, 0.0, 0.0),
            angular_velocity_radps=(0.0, 0.0, 0.0), battery_fraction=True,
            health_state=c.HealthState.OK, estimator_valid=True,
            last_valid_command_time_s=None,
        )


def test_bool_rejected_as_step_index():
    with pytest.raises(ValueError):
        c.WorldState(
            sim_time_s=0.0, step_index=True, dt_s=1.0 / 24.0, frame=c.Frame.LOCAL_ENU,
            obstacles=(), geofence=_geofence(), victims_ground_truth=(),
        )


def test_bool_rejected_as_radius():
    with pytest.raises(ValueError):
        c.Obstacle(obstacle_id="o0", position_m=(0.0, 0.0), radius_m=True)


# ---------------------------------------------------------------------------
# Finite-value validation gaps closed: dt_s and radius_m must reject +inf
# ---------------------------------------------------------------------------

def test_infinite_dt_s_rejected():
    with pytest.raises(ValueError):
        c.WorldState(
            sim_time_s=0.0, step_index=0, dt_s=math.inf, frame=c.Frame.LOCAL_ENU,
            obstacles=(), geofence=_geofence(), victims_ground_truth=(),
        )


def test_infinite_radius_rejected():
    with pytest.raises(ValueError):
        c.Obstacle(obstacle_id="o0", position_m=(0.0, 0.0), radius_m=math.inf)


# ---------------------------------------------------------------------------
# Command semantic validation: each command_type's field combination
# ---------------------------------------------------------------------------

def test_velocity_setpoint_with_no_velocity_rejected():
    with pytest.raises(ValueError):
        c.Command(
            vehicle_id="d0", command_type=c.CommandType.VELOCITY_SETPOINT, frame=c.Frame.LOCAL_ENU,
            desired_position_m=None, desired_velocity_mps=None, yaw_rad=None, yaw_rate_radps=None,
            timestamp_s=0.0, expiration_time_s=1.0, source="x", confidence=0.5,
        )


def test_position_setpoint_with_both_position_and_velocity_rejected():
    with pytest.raises(ValueError):
        c.Command(
            vehicle_id="d0", command_type=c.CommandType.POSITION_SETPOINT, frame=c.Frame.LOCAL_ENU,
            desired_position_m=(1.0, 1.0, 1.0), desired_velocity_mps=(1.0, 0.0, 0.0),
            yaw_rad=None, yaw_rate_radps=None,
            timestamp_s=0.0, expiration_time_s=1.0, source="x", confidence=0.5,
        )


def test_yaw_setpoint_with_no_yaw_rejected():
    with pytest.raises(ValueError):
        c.Command(
            vehicle_id="d0", command_type=c.CommandType.YAW_SETPOINT, frame=c.Frame.LOCAL_ENU,
            desired_position_m=None, desired_velocity_mps=None, yaw_rad=None, yaw_rate_radps=None,
            timestamp_s=0.0, expiration_time_s=1.0, source="x", confidence=0.5,
        )


def test_yaw_rate_setpoint_with_no_yaw_rate_rejected():
    with pytest.raises(ValueError):
        c.Command(
            vehicle_id="d0", command_type=c.CommandType.YAW_RATE_SETPOINT, frame=c.Frame.LOCAL_ENU,
            desired_position_m=None, desired_velocity_mps=None, yaw_rad=None, yaw_rate_radps=None,
            timestamp_s=0.0, expiration_time_s=1.0, source="x", confidence=0.5,
        )


def test_hold_command_with_setpoint_rejected():
    with pytest.raises(ValueError):
        c.Command(
            vehicle_id="d0", command_type=c.CommandType.HOLD, frame=c.Frame.LOCAL_ENU,
            desired_position_m=(0.0, 0.0, 0.0), desired_velocity_mps=None,
            yaw_rad=None, yaw_rate_radps=None,
            timestamp_s=0.0, expiration_time_s=1.0, source="x", confidence=0.5,
        )


def test_land_command_with_setpoint_rejected():
    with pytest.raises(ValueError):
        c.Command(
            vehicle_id="d0", command_type=c.CommandType.LAND, frame=c.Frame.LOCAL_ENU,
            desired_position_m=None, desired_velocity_mps=(1.0, 0.0, 0.0),
            yaw_rad=None, yaw_rate_radps=None,
            timestamp_s=0.0, expiration_time_s=1.0, source="x", confidence=0.5,
        )


def test_abort_command_with_setpoint_rejected():
    with pytest.raises(ValueError):
        c.Command(
            vehicle_id="d0", command_type=c.CommandType.ABORT, frame=c.Frame.LOCAL_ENU,
            desired_position_m=None, desired_velocity_mps=None,
            yaw_rad=0.1, yaw_rate_radps=None,
            timestamp_s=0.0, expiration_time_s=1.0, source="x", confidence=0.5,
        )


def test_abort_command_with_no_setpoints_accepted():
    cmd = c.Command(
        vehicle_id="d0", command_type=c.CommandType.ABORT, frame=c.Frame.LOCAL_ENU,
        desired_position_m=None, desired_velocity_mps=None, yaw_rad=None, yaw_rate_radps=None,
        timestamp_s=0.0, expiration_time_s=1.0, source="x", confidence=0.5,
    )
    assert cmd.command_type is c.CommandType.ABORT


def test_position_setpoint_valid_form_accepted():
    cmd = c.Command(
        vehicle_id="d0", command_type=c.CommandType.POSITION_SETPOINT, frame=c.Frame.LOCAL_ENU,
        desired_position_m=(1.0, 2.0, 3.0), desired_velocity_mps=None,
        yaw_rad=None, yaw_rate_radps=None,
        timestamp_s=0.0, expiration_time_s=1.0, source="x", confidence=0.5,
    )
    assert cmd.desired_position_m == (1.0, 2.0, 3.0)


# ---------------------------------------------------------------------------
# SafetyDecision semantic validation
# ---------------------------------------------------------------------------

def test_accepted_decision_without_filtered_command_rejected():
    with pytest.raises(ValueError):
        c.SafetyDecision(
            vehicle_id="d0", sim_time_s=0.0, accepted=True, filtered_command=None,
            active_constraints=(), reason="x", emergency_state=c.SafetyState.NORMAL,
            min_predicted_clearance_m=None, time_to_collision_s=None,
        )


def test_rejected_decision_with_filtered_command_rejected():
    with pytest.raises(ValueError):
        c.SafetyDecision(
            vehicle_id="d0", sim_time_s=0.0, accepted=False, filtered_command=_command(),
            active_constraints=(), reason="x", emergency_state=c.SafetyState.ABORT,
            min_predicted_clearance_m=None, time_to_collision_s=None,
        )


def test_filtered_command_vehicle_id_mismatch_rejected():
    mismatched = _command(vehicle_id="drone1")
    with pytest.raises(ValueError):
        c.SafetyDecision(
            vehicle_id="drone0", sim_time_s=0.0, accepted=True, filtered_command=mismatched,
            active_constraints=(), reason="x", emergency_state=c.SafetyState.NORMAL,
            min_predicted_clearance_m=None, time_to_collision_s=None,
        )


def test_empty_active_constraint_string_rejected():
    with pytest.raises(ValueError):
        c.SafetyDecision(
            vehicle_id="d0", sim_time_s=0.0, accepted=False, filtered_command=None,
            active_constraints=("",), reason="x", emergency_state=c.SafetyState.SAFE_HOLD,
            min_predicted_clearance_m=None, time_to_collision_s=None,
        )


# ---------------------------------------------------------------------------
# occluded must contain only bool elements
# ---------------------------------------------------------------------------

def test_occluded_non_bool_element_rejected():
    with pytest.raises(ValueError):
        c.SensorObservation(
            vehicle_id="d0", sensor_timestamp_s=0.0, sensor_latency_s=0.0, fov_deg=90.0,
            range_returns_m=(1.0,), occluded=(1,),  # int, not bool
            dropout=False, pose_uncertainty_m=0.0, detections=(),
        )


# ---------------------------------------------------------------------------
# Contract-version compatibility policy in from_dict
# ---------------------------------------------------------------------------

def test_from_dict_requires_contract_version_present():
    d = c.to_dict(_command())
    del d["contract_version"]
    with pytest.raises(ValueError, match="contract_version"):
        c.from_dict(c.Command, d)


def test_from_dict_rejects_incompatible_major_version():
    d = c.to_dict(_command())
    d["contract_version"] = "99.0.0"
    with pytest.raises(ValueError, match="incompatible contract_version"):
        c.from_dict(c.Command, d)


def test_from_dict_accepts_same_major_different_minor_patch():
    d = c.to_dict(_command())
    current_major = c.CONTRACT_VERSION.split(".")[0]
    d["contract_version"] = f"{current_major}.99.99"
    restored = c.from_dict(c.Command, d)
    assert restored.vehicle_id == "drone0"


def test_from_dict_missing_required_field_raises_clear_value_error():
    d = c.to_dict(_command())
    del d["vehicle_id"]
    with pytest.raises(ValueError, match="vehicle_id"):
        c.from_dict(c.Command, d)


def test_from_dict_missing_field_on_type_without_contract_version():
    d = c.to_dict(_obstacle())
    del d["radius_m"]
    with pytest.raises(ValueError, match="radius_m"):
        c.from_dict(c.Obstacle, d)


# ---------------------------------------------------------------------------
# contract_version is validated on direct construction too, not only via
# from_dict - a caller building one of these six by hand with an
# incompatible or malformed version string must fail immediately.
# ---------------------------------------------------------------------------

def test_direct_construction_rejects_incompatible_major_version_world_state():
    with pytest.raises(ValueError, match="incompatible contract_version"):
        c.WorldState(
            sim_time_s=0.0, step_index=0, dt_s=1.0 / 24.0, frame=c.Frame.LOCAL_ENU,
            obstacles=(), geofence=_geofence(), victims_ground_truth=(),
            contract_version="99.0.0",
        )


def test_direct_construction_rejects_incompatible_major_version_vehicle_state():
    with pytest.raises(ValueError, match="incompatible contract_version"):
        c.VehicleState(
            vehicle_id="d0", sim_time_s=0.0, frame=c.Frame.LOCAL_ENU,
            position_m=(0.0, 0.0, 0.0), velocity_mps=(0.0, 0.0, 0.0),
            acceleration_mps2=(0.0, 0.0, 0.0), attitude_rad=(0.0, 0.0, 0.0),
            angular_velocity_radps=(0.0, 0.0, 0.0), battery_fraction=1.0,
            health_state=c.HealthState.OK, estimator_valid=True,
            last_valid_command_time_s=None, contract_version="99.0.0",
        )


def test_direct_construction_rejects_incompatible_major_version_sensor_observation():
    with pytest.raises(ValueError, match="incompatible contract_version"):
        c.SensorObservation(
            vehicle_id="d0", sensor_timestamp_s=0.0, sensor_latency_s=0.0, fov_deg=90.0,
            range_returns_m=(), occluded=(), dropout=False, pose_uncertainty_m=0.0,
            detections=(), contract_version="99.0.0",
        )


def test_direct_construction_rejects_incompatible_major_version_neighbor_observation():
    with pytest.raises(ValueError, match="incompatible contract_version"):
        c.NeighborObservation(
            receiver_id="drone0", sender_id="drone1", frame=c.Frame.LOCAL_ENU,
            measured_position_m=(0.0, 0.0, 0.0), measured_velocity_mps=(0.0, 0.0, 0.0),
            sample_timestamp_s=0.0, delivery_timestamp_s=0.0, packet_age_s=0.0,
            communication_confidence=0.5, stale=False, contract_version="99.0.0",
        )


def test_direct_construction_rejects_incompatible_major_version_command():
    with pytest.raises(ValueError, match="incompatible contract_version"):
        c.Command(
            vehicle_id="d0", command_type=c.CommandType.VELOCITY_SETPOINT, frame=c.Frame.LOCAL_ENU,
            desired_position_m=None, desired_velocity_mps=(0.0, 0.0, 0.0),
            yaw_rad=None, yaw_rate_radps=None, timestamp_s=0.0, expiration_time_s=1.0,
            source="x", confidence=0.5, contract_version="99.0.0",
        )


def test_direct_construction_rejects_incompatible_major_version_safety_decision():
    with pytest.raises(ValueError, match="incompatible contract_version"):
        c.SafetyDecision(
            vehicle_id="d0", sim_time_s=0.0, accepted=False, filtered_command=None,
            active_constraints=(), reason="x", emergency_state=c.SafetyState.NORMAL,
            min_predicted_clearance_m=None, time_to_collision_s=None,
            contract_version="99.0.0",
        )


def test_direct_construction_rejects_malformed_version_string():
    with pytest.raises(ValueError, match="invalid contract version string"):
        c.Command(
            vehicle_id="d0", command_type=c.CommandType.VELOCITY_SETPOINT, frame=c.Frame.LOCAL_ENU,
            desired_position_m=None, desired_velocity_mps=(0.0, 0.0, 0.0),
            yaw_rad=None, yaw_rate_radps=None, timestamp_s=0.0, expiration_time_s=1.0,
            source="x", confidence=0.5, contract_version="not-a-version",
        )


def test_direct_construction_accepts_same_major_different_minor_patch():
    current_major = c.CONTRACT_VERSION.split(".")[0]
    cmd = c.Command(
        vehicle_id="d0", command_type=c.CommandType.VELOCITY_SETPOINT, frame=c.Frame.LOCAL_ENU,
        desired_position_m=None, desired_velocity_mps=(0.0, 0.0, 0.0),
        yaw_rad=None, yaw_rate_radps=None, timestamp_s=0.0, expiration_time_s=1.0,
        source="x", confidence=0.5, contract_version=f"{current_major}.99.99",
    )
    assert cmd.contract_version == f"{current_major}.99.99"
