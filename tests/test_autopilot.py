"""Phase 6: autopilot adapter tests - see docs/PHASE6_AUTOPILOT_ADAPTERS.md
and swarm_sim/autopilot/*.py. Architecture/AST checks live in
tests/test_autopilot_architecture.py.
"""
import random

import pytest

from swarm_sim.autopilot import (
    ArduPilotAdapterSkeleton, AutopilotAdapter, AutopilotMode, ConnectionState, MockAdapter,
    PX4AdapterSkeleton,
)
from swarm_sim.autopilot.frames import convert_command_frame, convert_vector_frame
from swarm_sim.autopilot.types import AdapterCommand, VehicleTelemetry
from swarm_sim.contracts import Command, CommandType, Frame

GEO_FRAME = Frame.LOCAL_ENU


def _command(vid="drone0", ctype=CommandType.VELOCITY_SETPOINT, frame=Frame.LOCAL_ENU,
             vel=(1.0, 0.0, 0.0), pos=None, t=0.0, ttl=1.0, yaw=None, yaw_rate=None, source="test"):
    kwargs = dict(vehicle_id=vid, command_type=ctype, frame=frame, desired_position_m=pos,
                  desired_velocity_mps=vel, yaw_rad=yaw, yaw_rate_radps=yaw_rate,
                  timestamp_s=t, expiration_time_s=t + ttl, source=source, confidence=0.9)
    if ctype in (CommandType.HOLD, CommandType.ABORT):
        kwargs["desired_velocity_mps"] = None
    if ctype == CommandType.LAND:
        kwargs["desired_velocity_mps"] = None
    return Command(**kwargs)


def _adapter_command(seq=0, **kwargs):
    return AdapterCommand(command=_command(**kwargs), sequence=seq)


def _telemetry(t=0.0, vid="drone0", frame=Frame.LOCAL_ENU, connection_state=ConnectionState.CONNECTED,
               mode=AutopilotMode.OFFBOARD):
    return VehicleTelemetry(
        vehicle_id=vid, timestamp_s=t, frame=frame, position_m=(0.0, 0.0, 3.0), velocity_mps=(0.0, 0.0, 0.0),
        acceleration_mps2=None, attitude_rad=(0.0, 0.0, 0.0), angular_velocity_radps=None, battery_fraction=1.0,
        estimator_valid=True, connection_state=connection_state, autopilot_mode=mode, armed=False,
        failsafe=False, sequence=0,
    )


def _connected_adapter(**kwargs):
    a = MockAdapter("drone0", **kwargs)
    a.connect()
    a.push_ground_truth_state(_telemetry(t=0.0))
    return a


# --- 1. interface implementation test --------------------------------------

def test_mock_adapter_implements_autopilot_adapter_protocol():
    assert isinstance(MockAdapter("drone0"), AutopilotAdapter)


def test_skeletons_implement_autopilot_adapter_protocol():
    assert isinstance(ArduPilotAdapterSkeleton("drone0"), AutopilotAdapter)
    assert isinstance(PX4AdapterSkeleton("drone0"), AutopilotAdapter)


# --- 2. mock connect/disconnect ---------------------------------------------

def test_mock_connect_transitions_to_connected():
    a = MockAdapter("drone0")
    assert a.state == ConnectionState.DISCONNECTED
    result = a.connect()
    assert result.accepted
    assert a.state == ConnectionState.CONNECTED


def test_mock_disconnect_transitions_back():
    a = _connected_adapter()
    result = a.disconnect()
    assert result.accepted
    assert a.state == ConnectionState.DISCONNECTED


# --- 3. telemetry read -------------------------------------------------------

def test_read_telemetry_before_any_push_is_safe_and_disconnected():
    a = MockAdapter("drone0")
    tel = a.read_telemetry()
    assert tel.estimator_valid is False
    assert tel.connection_state == ConnectionState.DISCONNECTED


def test_read_telemetry_returns_pushed_state():
    a = _connected_adapter()
    tel = a.read_telemetry()
    assert tel.position_m == (0.0, 0.0, 3.0)
    assert tel.estimator_valid is True


# --- 4. command acceptance ---------------------------------------------------

def test_nominal_command_is_accepted():
    a = _connected_adapter()
    result = a.send_command(_adapter_command(seq=0, t=0.0, ttl=1.0))
    assert result.accepted
    assert result.transformed_command is not None
    assert a.mode == AutopilotMode.OFFBOARD


# --- 5. expired command rejection -------------------------------------------

def test_expired_command_is_rejected():
    # send_command derives "now" from the command's own timestamp plus the
    # adapter's configured command_latency_s (there is no separate now_s
    # argument in the required interface - see mock.py's module
    # docstring) - a tiny nonzero latency is what makes an
    # already-inconsistent-by-the-time-it-arrives command observable here.
    a = _connected_adapter(command_latency_s=0.01)
    result = a.send_command(_adapter_command(seq=0, t=0.0, ttl=0.001))
    assert not result.accepted
    assert result.reason == "expired"


def test_is_command_fresh_false_once_now_passes_expiration():
    a = _connected_adapter()
    cmd = _adapter_command(seq=0, t=0.0, ttl=1.0)
    assert a.is_command_fresh(cmd, now_s=0.5) is True
    assert a.is_command_fresh(cmd, now_s=1.5) is False


# --- 6. NaN/Inf rejection (structural: contracts.Command itself refuses) ---

def test_command_with_nan_cannot_be_constructed():
    with pytest.raises(ValueError):
        _command(vel=(float("nan"), 0.0, 0.0))


def test_command_with_inf_cannot_be_constructed():
    with pytest.raises(ValueError):
        _command(vel=(float("inf"), 0.0, 0.0))


# --- 7. frame mismatch rejection ---------------------------------------------

def test_frame_mismatch_is_rejected():
    a = _connected_adapter(operating_frame=Frame.LOCAL_ENU)
    result = a.send_command(_adapter_command(seq=0, frame=Frame.LOCAL_NED))
    assert not result.accepted
    assert result.reason == "frame_mismatch"


# --- 8. explicit ENU/NED conversion -----------------------------------------

def test_enu_to_ned_conversion_is_exact():
    # East 2, North 3, Up 4 (ENU) -> North 3, East 2, Down -4 (NED)
    assert convert_vector_frame((2.0, 3.0, 4.0), Frame.LOCAL_ENU, Frame.LOCAL_NED) == (3.0, 2.0, -4.0)


def test_ned_to_enu_conversion_is_exact():
    assert convert_vector_frame((3.0, 2.0, -4.0), Frame.LOCAL_NED, Frame.LOCAL_ENU) == (2.0, 3.0, 4.0)


def test_enu_ned_roundtrip_is_identity():
    v = (1.5, -2.5, 3.5)
    assert convert_vector_frame(convert_vector_frame(v, Frame.LOCAL_ENU, Frame.LOCAL_NED),
                                  Frame.LOCAL_NED, Frame.LOCAL_ENU) == v


def test_convert_command_frame_enu_to_ned_converts_velocity_and_frame():
    cmd = _command(vel=(1.0, 0.0, 0.0), frame=Frame.LOCAL_ENU)
    converted = convert_command_frame(cmd, Frame.LOCAL_NED)
    assert converted.frame == Frame.LOCAL_NED
    assert converted.desired_velocity_mps == (0.0, 1.0, -0.0)
    assert cmd.frame == Frame.LOCAL_ENU  # original untouched


def test_convert_command_frame_rejects_unknown_target():
    cmd = _command()
    with pytest.raises(ValueError):
        convert_command_frame(cmd, "not_a_frame")


# --- 9. BODY frame rejection without attitude -------------------------------

def test_body_conversion_without_attitude_is_rejected():
    cmd = _command(frame=Frame.LOCAL_ENU)
    with pytest.raises(ValueError):
        convert_command_frame(cmd, Frame.BODY)


def test_body_to_enu_conversion_without_attitude_is_rejected():
    cmd = _command(frame=Frame.BODY, vel=(1.0, 0.0, 0.0))
    with pytest.raises(ValueError):
        convert_command_frame(cmd, Frame.LOCAL_ENU)


# --- 10. BODY frame conversion with valid attitude --------------------------

def test_body_conversion_with_yaw_zero_matches_enu():
    cmd = _command(vel=(1.0, 0.0, 0.0), frame=Frame.LOCAL_ENU)
    converted = convert_command_frame(cmd, Frame.BODY, vehicle_attitude_rad=(0.0, 0.0, 0.0))
    assert converted.desired_velocity_mps == pytest.approx((1.0, 0.0, 0.0), abs=1e-9)


def test_body_conversion_with_yaw_90_degrees_rotates_correctly():
    import math
    # Facing due "north-like" (+y ENU), forward ENU velocity of 1 m/s along +y
    # should read as pure BODY-forward (1, 0, 0) once yaw accounts for heading.
    cmd = _command(vel=(0.0, 1.0, 0.0), frame=Frame.LOCAL_ENU)
    converted = convert_command_frame(cmd, Frame.BODY, vehicle_attitude_rad=(0.0, 0.0, math.pi / 2))
    assert converted.desired_velocity_mps == pytest.approx((1.0, 0.0, 0.0), abs=1e-9)


def test_body_conversion_roundtrip_with_attitude_is_identity():
    cmd = _command(vel=(2.0, -1.0, 0.5), frame=Frame.LOCAL_ENU)
    attitude = (0.0, 0.0, 1.234)
    body = convert_command_frame(cmd, Frame.BODY, vehicle_attitude_rad=attitude)
    back = convert_command_frame(body, Frame.LOCAL_ENU, vehicle_attitude_rad=attitude)
    assert back.desired_velocity_mps == pytest.approx(cmd.desired_velocity_mps, abs=1e-9)


def test_body_conversion_rejects_yaw_field_present():
    cmd = _command(ctype=CommandType.YAW_SETPOINT, vel=None, yaw=0.3, frame=Frame.LOCAL_ENU)
    with pytest.raises(ValueError):
        convert_command_frame(cmd, Frame.BODY, vehicle_attitude_rad=(0.0, 0.0, 0.0))


# --- 11. wrong vehicle ID rejection ------------------------------------------

def test_wrong_vehicle_id_is_rejected():
    a = _connected_adapter()
    result = a.send_command(_adapter_command(seq=0, vid="drone99"))
    assert not result.accepted
    assert result.reason == "wrong_vehicle_id"


# --- 12. replayed sequence rejection ----------------------------------------

def test_conflicting_replay_of_same_sequence_is_rejected():
    a = _connected_adapter()
    first = a.send_command(_adapter_command(seq=0, t=0.0, vel=(1.0, 0.0, 0.0)))
    assert first.accepted
    second = a.send_command(_adapter_command(seq=0, t=0.0, vel=(2.0, 0.0, 0.0)))
    assert not second.accepted
    assert second.reason == "sequence_replayed_conflicting"


def test_stale_sequence_number_is_rejected():
    a = _connected_adapter()
    a.send_command(_adapter_command(seq=5, t=0.0))
    result = a.send_command(_adapter_command(seq=3, t=0.1))
    assert not result.accepted
    assert result.reason == "sequence_replayed_stale"


def test_identical_replay_of_last_accepted_sequence_is_idempotent():
    a = _connected_adapter()
    first = a.send_command(_adapter_command(seq=0, t=0.0, vel=(1.0, 0.0, 0.0)))
    second = a.send_command(_adapter_command(seq=0, t=0.0, vel=(1.0, 0.0, 0.0)))
    assert first.accepted and second.accepted
    assert first.reason == second.reason == "accepted"


def test_monotonically_increasing_sequence_is_accepted():
    a = _connected_adapter()
    r0 = a.send_command(_adapter_command(seq=0, t=0.0))
    r1 = a.send_command(_adapter_command(seq=1, t=0.1))
    assert r0.accepted and r1.accepted


# --- 13. stale telemetry failsafe -------------------------------------------

def test_stale_telemetry_rejects_navigation_command_and_requests_hold():
    a = MockAdapter("drone0", telemetry_stale_timeout_s=0.5)
    a.connect()
    a.push_ground_truth_state(_telemetry(t=0.0))
    result = a.send_command(_adapter_command(seq=0, t=10.0, ttl=5.0))  # far past staleness window
    assert not result.accepted
    assert result.reason == "stale_telemetry"
    assert a.mode == AutopilotMode.HOLD


def test_non_navigation_command_is_not_blocked_by_stale_telemetry():
    a = MockAdapter("drone0", telemetry_stale_timeout_s=0.5)
    a.connect()
    a.push_ground_truth_state(_telemetry(t=0.0))
    result = a.send_command(_adapter_command(seq=0, ctype=CommandType.HOLD, t=10.0, ttl=5.0))
    assert result.accepted


# --- 14. disconnected-command rejection -------------------------------------

def test_command_while_disconnected_is_rejected():
    a = MockAdapter("drone0")
    result = a.send_command(_adapter_command(seq=0))
    assert not result.accepted
    assert result.reason == "disconnected"


# --- 15. failsafe command rejection -----------------------------------------

def test_command_during_failsafe_is_rejected():
    a = _connected_adapter()
    a.inject_failsafe(True)
    result = a.send_command(_adapter_command(seq=0))
    assert not result.accepted
    assert result.reason == "failsafe_active"


# --- 16-19. HOLD / LAND / RTL / ABORT modes ---------------------------------

def test_hold_command_sets_hold_mode():
    a = _connected_adapter()
    result = a.send_command(_adapter_command(seq=0, ctype=CommandType.HOLD))
    assert result.accepted
    assert a.mode == AutopilotMode.HOLD


def test_land_command_sets_land_mode_and_landing_state():
    a = _connected_adapter()
    result = a.send_command(_adapter_command(seq=0, ctype=CommandType.LAND))
    assert result.accepted
    assert a.mode == AutopilotMode.LAND
    assert a.state == ConnectionState.LANDING


def test_request_return_to_launch_sets_rtl_mode():
    a = _connected_adapter()
    result = a.request_return_to_launch()
    assert result.accepted
    assert a.mode == AutopilotMode.RTL


def test_abort_command_takes_precedence_over_current_mode():
    a = _connected_adapter()
    a.send_command(_adapter_command(seq=0, ctype=CommandType.VELOCITY_SETPOINT, t=0.0))
    assert a.mode == AutopilotMode.OFFBOARD
    result = a.send_command(_adapter_command(seq=1, ctype=CommandType.ABORT, t=0.1))
    assert result.accepted
    assert a.mode == AutopilotMode.ABORT


def test_abort_method_sets_abort_mode():
    a = _connected_adapter()
    result = a.abort()
    assert result.accepted
    assert a.mode == AutopilotMode.ABORT


def test_mode_request_while_disconnected_is_rejected():
    a = MockAdapter("drone0")
    result = a.request_land()
    assert not result.accepted


# --- 20. adapter history correctness ----------------------------------------

def test_command_history_records_accept_and_reject_in_order():
    a = _connected_adapter()
    a.send_command(_adapter_command(seq=0, t=0.0))
    a.send_command(_adapter_command(seq=0, t=0.0, vel=(9.0, 0.0, 0.0)))  # conflicting replay -> rejected
    a.send_command(_adapter_command(seq=1, t=0.1))
    assert len(a.command_history) == 3
    assert a.command_history[0][1].accepted
    assert not a.command_history[1][1].accepted
    assert a.command_history[1][1].reason == "sequence_replayed_conflicting"
    assert a.command_history[2][1].accepted


# --- 21. deterministic replay -----------------------------------------------

def test_deterministic_replay_same_seed_same_history():
    def run():
        a = MockAdapter("drone0", command_packet_loss_prob=0.4, rng=random.Random(42))
        a.connect()
        a.push_ground_truth_state(_telemetry(t=0.0))
        results = [a.send_command(_adapter_command(seq=i, t=i * 0.1)).reason for i in range(20)]
        return results

    assert run() == run()


# --- 22. configurable latency ------------------------------------------------

def test_command_latency_can_push_command_past_expiry():
    a = _connected_adapter(command_latency_s=1.0)
    result = a.send_command(_adapter_command(seq=0, t=0.0, ttl=0.5))
    assert not result.accepted
    assert result.reason == "expired"


def test_zero_latency_does_not_reject_a_normally_valid_command():
    a = _connected_adapter(command_latency_s=0.0)
    result = a.send_command(_adapter_command(seq=0, t=0.0, ttl=0.5))
    assert result.accepted


def test_telemetry_latency_delays_visibility_of_a_new_push():
    a = MockAdapter("drone0", telemetry_latency_s=1.0)
    a.connect()
    a.push_ground_truth_state(_telemetry(t=0.0, connection_state=ConnectionState.CONNECTED))
    a.push_ground_truth_state(_telemetry(t=0.2, connection_state=ConnectionState.CONNECTED))  # too soon - dropped
    tel = a.read_telemetry()
    assert tel.timestamp_s == 0.0
    a.push_ground_truth_state(_telemetry(t=1.5, connection_state=ConnectionState.CONNECTED))  # now delivered
    tel2 = a.read_telemetry()
    assert tel2.timestamp_s == 1.5


# --- 23. configurable packet loss -------------------------------------------

def test_command_packet_loss_probability_one_always_drops():
    a = _connected_adapter(command_packet_loss_prob=1.0)
    result = a.send_command(_adapter_command(seq=0))
    assert not result.accepted
    assert result.reason == "packet_loss"


def test_command_packet_loss_probability_zero_never_drops():
    a = _connected_adapter(command_packet_loss_prob=0.0)
    result = a.send_command(_adapter_command(seq=0))
    assert result.accepted


def test_telemetry_packet_loss_probability_one_never_updates():
    a = MockAdapter("drone0", telemetry_packet_loss_prob=1.0)
    a.connect()
    a.push_ground_truth_state(_telemetry(t=0.0))
    tel = a.read_telemetry()
    assert tel.estimator_valid is False  # still the pre-push default - every push was dropped


# --- offline skeletons never connect/act ------------------------------------

def test_ardupilot_skeleton_connect_always_fails():
    a = ArduPilotAdapterSkeleton("drone0")
    result = a.connect()
    assert not result.accepted
    assert a.state == ConnectionState.DISCONNECTED


def test_px4_skeleton_connect_always_fails():
    a = PX4AdapterSkeleton("drone0")
    result = a.connect()
    assert not result.accepted
    assert a.state == ConnectionState.DISCONNECTED


def test_ardupilot_skeleton_send_command_always_rejected():
    a = ArduPilotAdapterSkeleton("drone0")
    result = a.send_command(_adapter_command(seq=0))
    assert not result.accepted


def test_px4_skeleton_mode_requests_always_rejected():
    a = PX4AdapterSkeleton("drone0")
    assert not a.request_land().accepted
    assert not a.request_return_to_launch().accepted
    assert not a.abort().accepted
