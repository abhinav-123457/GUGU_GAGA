"""Phase 7: local, offline SITL integration tests - see
docs/PHASE7_SITL_INTEGRATION.md and swarm_sim/sitl/*.py,
swarm_sim/autopilot/sitl.py. Architecture/AST checks live in
tests/test_sitl_architecture.py.
"""
import random

import pytest

from swarm_sim.autopilot import AdapterCommand, AutopilotAdapter, AutopilotMode, ConnectionState
from swarm_sim.autopilot.ardupilot_sitl import ArduPilotSITLAdapterSkeleton
from swarm_sim.autopilot.px4_sitl import PX4SITLAdapterSkeleton
from swarm_sim.autopilot.sitl import SITLAdapter
from swarm_sim.contracts import Command, CommandType, Frame
from swarm_sim.sitl.clock import SimClock
from swarm_sim.sitl.commands import (
    MAVLINK_COMMAND_TYPE_MAPPING_SPEC, MAVLINK_FRAME_MAPPING_SPEC, MAVLINK_MODE_NAME_MAPPING_SPEC,
    SITLCommand, derive_safety_decision_id,
)
from swarm_sim.sitl.fake_transport import FakeSITLTransport
from swarm_sim.sitl.telemetry import CommandAck, FailureType, TransportResult
from swarm_sim.sitl.vehicle_namespace import NamespaceError, VehicleNamespaceRegistry, namespace_for

VIDS = ("drone0", "drone1")


def _command(vid="drone0", ctype=CommandType.VELOCITY_SETPOINT, frame=Frame.LOCAL_ENU,
             vel=(1.0, 0.0, 0.0), pos=None, t=0.0, ttl=1.0, yaw=None, yaw_rate=None, source="test"):
    kwargs = dict(vehicle_id=vid, command_type=ctype, frame=frame, desired_position_m=pos,
                  desired_velocity_mps=vel, yaw_rad=yaw, yaw_rate_radps=yaw_rate,
                  timestamp_s=t, expiration_time_s=t + ttl, source=source, confidence=0.9)
    if ctype in (CommandType.HOLD, CommandType.ABORT, CommandType.LAND):
        kwargs["desired_velocity_mps"] = None
    return Command(**kwargs)


def _adapter_command(seq=0, **kwargs):
    return AdapterCommand(command=_command(**kwargs), sequence=seq)


def _sitl_command(transport, seq=0, vid="drone0", **kwargs):
    ac = _adapter_command(seq=seq, vid=vid, **kwargs)
    return SITLCommand(adapter_command=ac, namespace=transport.registry.namespace_of(vid),
                        safety_decision_id=derive_safety_decision_id(vid, ac.timestamp_s, ac.command))


def _transport(vehicle_ids=VIDS, seed_telemetry=True, **kwargs):
    t = FakeSITLTransport(vehicle_ids=vehicle_ids, rng=random.Random(0), **kwargs)
    t.start()
    if seed_telemetry:
        # Matches Phase 6's own MockAdapter test convention
        # (_connected_adapter pushes an initial ground-truth state before
        # any send_command call) - "no telemetry ever received" is itself
        # the stale condition (see fake_transport.py's own
        # _last_state_update_at_s), so most tests establish a fresh
        # baseline first and test staleness explicitly where it matters.
        for vid in vehicle_ids:
            t.push_vehicle_state(vid, (0.0, 0.0, 3.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                                  1.0, True, now_s=0.0)
    return t


def _connected_adapter(transport, vid="drone0"):
    a = SITLAdapter(vid, transport.registry.namespace_of(vid), transport)
    a.connect()
    return a


# ==========================================================================
# 1. SimClock
# ==========================================================================

def test_clock_advance_moves_forward():
    c = SimClock()
    assert c.advance(0.5) == 0.5
    assert c.now_s == 0.5


def test_clock_advance_rejects_nonpositive_dt():
    c = SimClock()
    with pytest.raises(ValueError):
        c.advance(0.0)
    with pytest.raises(ValueError):
        c.advance(-1.0)


def test_clock_set_manual_jump():
    c = SimClock(initial_time_s=1.0)
    assert c.set(5.0) == 5.0


def test_clock_set_rejects_going_backwards():
    c = SimClock(initial_time_s=5.0)
    with pytest.raises(ValueError):
        c.set(4.999)


def test_clock_is_from_the_future_respects_tolerance():
    c = SimClock(initial_time_s=10.0, future_tolerance_s=0.05)
    assert not c.is_from_the_future(10.05)
    assert c.is_from_the_future(10.06)


def test_clock_is_stale():
    c = SimClock(initial_time_s=10.0)
    assert not c.is_stale(9.5, timeout_s=1.0)
    assert c.is_stale(8.9, timeout_s=1.0)


def test_clock_is_expired_exact_boundary():
    c = SimClock(initial_time_s=10.0)
    assert c.is_expired(10.0)     # expired AT, not just after
    assert not c.is_expired(10.0001)


def test_clock_never_uses_wall_clock_module_level():
    import swarm_sim.sitl.clock as clock_mod
    assert "time" not in dir(clock_mod) or not hasattr(clock_mod.time, "time")


# ==========================================================================
# 2. VehicleNamespaceRegistry
# ==========================================================================

def test_namespace_for_is_deterministic_and_distinct():
    assert namespace_for("drone0") == namespace_for("drone0")
    assert namespace_for("drone0") != namespace_for("drone1")


def test_registry_register_and_lookup_roundtrip():
    reg = VehicleNamespaceRegistry()
    ns = reg.register("drone0")
    assert reg.namespace_of("drone0") == ns
    assert reg.vehicle_of(ns) == "drone0"


def test_registry_rejects_duplicate_vehicle_id():
    reg = VehicleNamespaceRegistry()
    reg.register("drone0")
    with pytest.raises(NamespaceError):
        reg.register("drone0")


def test_registry_require_match_raises_on_mismatch():
    reg = VehicleNamespaceRegistry()
    reg.register("drone0")
    reg.register("drone1")
    with pytest.raises(NamespaceError):
        reg.require_match("drone0", reg.namespace_of("drone1"))


def test_registry_unknown_vehicle_raises():
    reg = VehicleNamespaceRegistry()
    with pytest.raises(NamespaceError):
        reg.namespace_of("ghost")
    with pytest.raises(NamespaceError):
        reg.vehicle_of("sitl/ghost")


def test_fake_sitl_transport_rejects_namespace_collision_at_construction():
    """Two channels attempting to claim the same vehicle identity -
    Phase 7's "namespace collision attempt" scenario."""
    with pytest.raises(NamespaceError):
        FakeSITLTransport(vehicle_ids=["drone0", "drone0"])


# ==========================================================================
# 3. FakeSITLTransport lifecycle / determinism
# ==========================================================================

def test_transport_start_stop():
    t = FakeSITLTransport(vehicle_ids=VIDS)
    assert not t.is_running
    result = t.start()
    assert result.success and t.is_running
    result = t.stop()
    assert result.success and not t.is_running


def test_send_command_rejected_before_start():
    t = FakeSITLTransport(vehicle_ids=VIDS)
    cmd = _sitl_command(t)
    result = t.send_command(cmd)
    assert not result.success
    assert result.reason == "transport_not_started"


def test_send_command_rejected_after_stop():
    t = _transport()
    t.stop()
    result = t.send_command(_sitl_command(t))
    assert not result.success
    assert result.reason == "transport_stopped"


def test_unknown_vehicle_id_rejected_at_transport_level():
    t = _transport()
    ac = _adapter_command(vid="ghost")
    bad = SITLCommand(adapter_command=ac, namespace="sitl/ghost", safety_decision_id="x")
    result = t.send_command(bad)
    assert not result.success and result.reason == "unknown_vehicle_id"


def test_namespace_mismatch_rejected_at_transport_level():
    t = _transport()
    ac = _adapter_command(vid="drone0")
    forged = SITLCommand(adapter_command=ac, namespace=t.registry.namespace_of("drone1"),
                          safety_decision_id="x")
    result = t.send_command(forged)
    assert not result.success and result.reason == "namespace_mismatch"


def test_step_is_deterministic_fixed_step_execution():
    t1 = FakeSITLTransport(vehicle_ids=("d",), rng=random.Random(123))
    t1.start()
    t1.send_command(_sitl_command(t1, vid="d"))
    t1.step(1.0)
    telem1 = t1.receive_telemetry("d")

    t2 = FakeSITLTransport(vehicle_ids=("d",), rng=random.Random(123))
    t2.start()
    t2.send_command(_sitl_command(t2, vid="d"))
    t2.step(1.0)
    telem2 = t2.receive_telemetry("d")

    assert telem1.position_m == telem2.position_m
    assert telem1.velocity_mps == telem2.velocity_mps
    assert telem1.sequence == telem2.sequence


def test_step_integrates_velocity_setpoint_kinematics():
    t = _transport(vehicle_ids=("d",))   # seeded at (0, 0, 3.0)
    ac = _adapter_command(vid="d", vel=(2.0, 0.0, 0.0))
    cmd = SITLCommand(adapter_command=ac, namespace=t.registry.namespace_of("d"),
                       safety_decision_id="x")
    t.send_command(cmd)
    t.step(1.0)
    telem = t.receive_telemetry("d")
    assert telem.position_m == (2.0, 0.0, 3.0)


def test_set_sim_time_does_not_run_kinematics_integration():
    t = _transport(vehicle_ids=("d",), seed_telemetry=False)
    ac = _adapter_command(vid="d", vel=(2.0, 0.0, 0.0))
    cmd = SITLCommand(adapter_command=ac, namespace=t.registry.namespace_of("d"), safety_decision_id="x")
    t.send_command(cmd)
    t.set_sim_time(5.0)   # manual jump - no step() called
    telem = t.receive_telemetry("d")
    # No telemetry emitted at all yet (only push_vehicle_state/step emit) -
    # the placeholder has no position.
    assert telem.position_m is None


# ==========================================================================
# 4. Command validation chain (frame / expiry / future / sequence / loss)
# ==========================================================================

def test_frame_mismatch_rejected():
    t = _transport()
    ac = _adapter_command(vid="drone0", frame=Frame.LOCAL_NED)
    cmd = SITLCommand(adapter_command=ac, namespace=t.registry.namespace_of("drone0"), safety_decision_id="x")
    t.send_command(cmd)
    ack = t.receive_ack("drone0")
    assert not ack.accepted and ack.reason == "frame_mismatch"


def test_command_expiry_exact_boundary_rejected():
    t = _transport(command_latency_s=0.01)
    ac = _adapter_command(vid="drone0", t=0.0, ttl=0.005)   # expires before latency-delayed arrival
    cmd = SITLCommand(adapter_command=ac, namespace=t.registry.namespace_of("drone0"), safety_decision_id="x")
    t.send_command(cmd)
    ack = t.receive_ack("drone0")
    assert not ack.accepted and ack.reason == "expired"


def test_command_from_the_future_rejected():
    t = _transport(future_tolerance_s=0.01)
    t.set_sim_time(0.0)
    ac = _adapter_command(vid="drone0", t=5.0, ttl=10.0)
    cmd = SITLCommand(adapter_command=ac, namespace=t.registry.namespace_of("drone0"), safety_decision_id="x")
    t.send_command(cmd)
    ack = t.receive_ack("drone0")
    assert not ack.accepted and ack.reason == "command_from_the_future"


def test_command_packet_loss_probability_can_reject():
    t = _transport(vehicle_ids=("d",), command_packet_loss_prob=1.0)
    t.send_command(_sitl_command(t, vid="d"))
    ack = t.receive_ack("d")
    assert not ack.accepted and ack.reason == "command_packet_loss"


def test_command_packet_loss_failure_injection_forces_loss():
    t = _transport(vehicle_ids=("d",), command_packet_loss_prob=0.0)
    t.inject_failure("d", FailureType.COMMAND_PACKET_LOSS)
    t.send_command(_sitl_command(t, vid="d"))
    ack = t.receive_ack("d")
    assert not ack.accepted and ack.reason == "command_packet_loss"


def test_sequence_replayed_stale_rejected():
    t = _transport(vehicle_ids=("d",))
    t.send_command(_sitl_command(t, vid="d", seq=5))
    assert t.receive_ack("d").accepted
    t.send_command(_sitl_command(t, vid="d", seq=3))
    ack = t.receive_ack("d")
    assert not ack.accepted and ack.reason == "sequence_replayed_stale"


def test_sequence_idempotent_replay_returns_prior_ack():
    t = _transport(vehicle_ids=("d",))
    cmd = _sitl_command(t, vid="d", seq=5, vel=(1.0, 0.0, 0.0))
    t.send_command(cmd)
    first_ack = t.receive_ack("d")
    identical = SITLCommand(adapter_command=cmd.adapter_command, namespace=cmd.namespace,
                             safety_decision_id="different-id-does-not-matter")
    t.send_command(identical)
    second_ack = t.receive_ack("d")
    assert first_ack.accepted and second_ack.accepted
    assert first_ack.timestamp_s == second_ack.timestamp_s


def test_sequence_conflicting_replay_rejected():
    """Duplicate command handling: same sequence, DIFFERENT content ->
    rejected, never silently accepted as if idempotent."""
    t = _transport(vehicle_ids=("d",))
    t.send_command(_sitl_command(t, vid="d", seq=5, vel=(1.0, 0.0, 0.0)))
    assert t.receive_ack("d").accepted
    t.send_command(_sitl_command(t, vid="d", seq=5, vel=(9.0, 0.0, 0.0)))
    ack = t.receive_ack("d")
    assert not ack.accepted and ack.reason == "sequence_replayed_conflicting"


# ==========================================================================
# 5. Telemetry (latency / packet loss / push_vehicle_state)
# ==========================================================================

def test_telemetry_packet_loss_probability_drops_delivery():
    t = _transport(vehicle_ids=("d",), telemetry_packet_loss_prob=1.0)
    t.push_vehicle_state("d", (1, 2, 3), (0, 0, 0), (0, 0, 0), (0, 0, 0), 1.0, True, now_s=1.0)
    telem = t.receive_telemetry("d")
    assert telem.position_m is None    # nothing ever delivered


def test_telemetry_latency_delays_delivery():
    t = _transport(vehicle_ids=("d",), telemetry_latency_s=2.0)
    t.push_vehicle_state("d", (9, 9, 9), (0, 0, 0), (0, 0, 0), (0, 0, 0), 1.0, True, now_s=0.0)
    assert t.receive_telemetry("d").position_m is None    # not yet arrived
    t.set_sim_time(2.0)
    assert t.receive_telemetry("d").position_m == (9, 9, 9)


def test_push_vehicle_state_sets_position_directly_not_integrated():
    """push_vehicle_state (the PyBullet-integrated path's own hook) must
    set the pushed position exactly, never adding its own velocity*dt on
    top of it - `step`'s integrator is a separate mode used only by the
    standalone scenarios that have no real physics behind them (see
    fake_transport.py's module docstring); the two are never combined for
    one vehicle in a real run."""
    t = _transport(vehicle_ids=("d",))
    t.push_vehicle_state("d", (100.0, 0.0, 0.0), (5.0, 0.0, 0.0), (0, 0, 0), (0, 0, 0), 1.0, True, now_s=1.0)
    assert t.receive_telemetry("d").position_m == (100.0, 0.0, 0.0)
    t.push_vehicle_state("d", (100.2, 0.0, 0.0), (5.0, 0.0, 0.0), (0, 0, 0), (0, 0, 0), 1.0, True, now_s=1.04)
    telem = t.receive_telemetry("d")
    assert telem.position_m[0] == pytest.approx(100.2)


# ==========================================================================
# 6. Failsafes (heartbeat / estimator / battery / disconnect)
# ==========================================================================

def test_heartbeat_loss_blocks_navigation_and_forces_hold():
    t = _transport(vehicle_ids=("d",))
    t.inject_failure("d", FailureType.HEARTBEAT_LOSS)
    t.send_command(_sitl_command(t, vid="d", ctype=CommandType.VELOCITY_SETPOINT))
    ack = t.receive_ack("d")
    assert not ack.accepted and ack.reason == "heartbeat_lost"
    assert ack.autopilot_mode == AutopilotMode.HOLD


def test_heartbeat_loss_does_not_block_hold_command():
    t = _transport(vehicle_ids=("d",))
    t.inject_failure("d", FailureType.HEARTBEAT_LOSS)
    t.send_command(_sitl_command(t, vid="d", ctype=CommandType.HOLD, vel=None))
    assert t.receive_ack("d").accepted


def test_estimator_invalid_blocks_navigation():
    t = _transport(vehicle_ids=("d",))
    t.inject_failure("d", FailureType.ESTIMATOR_INVALID)
    t.send_command(_sitl_command(t, vid="d"))
    ack = t.receive_ack("d")
    assert not ack.accepted and ack.reason == "estimator_invalid"


def test_battery_critical_blocks_navigation():
    t = _transport(vehicle_ids=("d",))
    t.inject_failure("d", FailureType.BATTERY_CRITICAL)
    t.send_command(_sitl_command(t, vid="d"))
    ack = t.receive_ack("d")
    assert not ack.accepted and ack.reason == "battery_critical"


def test_battery_drain_auto_triggers_critical_failure():
    t = _transport(vehicle_ids=("d",), battery_drain_per_s=0.5, battery_critical_threshold=0.6)
    t.step(1.0)   # drains to 0.5, below the 0.6 threshold
    assert FailureType.BATTERY_CRITICAL in t._channel("d").active_failures


def test_stale_telemetry_blocks_navigation_and_forces_hold():
    """A vehicle whose real state hasn't been refreshed (via push_vehicle_state
    or step()) within telemetry_stale_timeout_s must reject a navigation
    setpoint and force HOLD - mirrors Phase 6 MockAdapter's own
    stale_telemetry policy. "Never received at all" also counts as stale
    (see fake_transport.py's _last_state_update_at_s)."""
    t = _transport(vehicle_ids=("d",), seed_telemetry=False, telemetry_stale_timeout_s=1.0)
    t.send_command(_sitl_command(t, vid="d", t=0.0))
    ack = t.receive_ack("d")
    assert not ack.accepted and ack.reason == "stale_telemetry"
    assert ack.autopilot_mode == AutopilotMode.HOLD


def test_stale_telemetry_does_not_block_non_navigation_command():
    t = _transport(vehicle_ids=("d",), seed_telemetry=False)
    t.send_command(_sitl_command(t, vid="d", ctype=CommandType.HOLD, vel=None))
    assert t.receive_ack("d").accepted


def test_fresh_telemetry_within_timeout_allows_navigation_command():
    t = _transport(vehicle_ids=("d",), seed_telemetry=False, telemetry_stale_timeout_s=1.0)
    t.push_vehicle_state("d", (0, 0, 3), (0, 0, 0), (0, 0, 0), (0, 0, 0), 1.0, True, now_s=0.5)
    t.send_command(_sitl_command(t, vid="d", t=0.5))
    assert t.receive_ack("d").accepted


def test_vehicle_disconnect_rejects_all_commands():
    t = _transport(vehicle_ids=("d",))
    t.inject_failure("d", FailureType.VEHICLE_DISCONNECT)
    t.send_command(_sitl_command(t, vid="d", ctype=CommandType.HOLD, vel=None))
    ack = t.receive_ack("d")
    assert not ack.accepted and ack.reason == "disconnected"


def test_failure_injection_on_one_vehicle_does_not_affect_another():
    t = _transport()
    t.inject_failure("drone0", FailureType.HEARTBEAT_LOSS)
    t.push_vehicle_state("drone0", (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), 1.0, True, now_s=1.0)
    t.push_vehicle_state("drone1", (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), 1.0, True, now_s=1.0)
    assert t.receive_telemetry("drone0").heartbeat_ok is False
    assert t.receive_telemetry("drone1").heartbeat_ok is True
    t.send_command(_sitl_command(t, vid="drone1"))
    assert t.receive_ack("drone1").accepted   # drone1 fully unaffected


# ==========================================================================
# 7. Command acknowledgement retrieval
# ==========================================================================

def test_receive_ack_placeholder_when_nothing_sent_yet():
    t = _transport(vehicle_ids=("d",))
    ack = t.receive_ack("d")
    assert not ack.accepted and ack.reason == "no_ack_available"


def test_receive_ack_timeout():
    t = _transport(vehicle_ids=("d",), ack_timeout_s=1.0)
    t.send_command(_sitl_command(t, vid="d", ctype=CommandType.HOLD, vel=None))
    assert t.receive_ack("d").accepted
    t.set_sim_time(10.0)
    ack = t.receive_ack("d")
    assert not ack.accepted and ack.reason == "acknowledgement_timeout"


# ==========================================================================
# 8. Namespace isolation (Phase 7 requirement 3's explicit test list)
# ==========================================================================

def test_drone0_commands_cannot_affect_drone1_state():
    t = _transport()
    for seq in range(5):
        t.send_command(_sitl_command(t, vid="drone0", seq=seq, vel=(float(seq), 0.0, 0.0)))
    d1 = t._channel("drone1")
    assert d1.command_history == []
    assert d1._last_accepted_sequence is None


def test_telemetry_from_drone1_cannot_satisfy_drone0_ack():
    t = _transport()
    t.push_vehicle_state("drone1", (7, 7, 7), (0, 0, 0), (0, 0, 0), (0, 0, 0), 1.0, True, now_s=1.0)
    t.send_command(_sitl_command(t, vid="drone0", ctype=CommandType.HOLD, vel=None))
    ack = t.receive_ack("drone0")
    assert ack.vehicle_id == "drone0" and ack.namespace == t.registry.namespace_of("drone0")
    telem0 = t.receive_telemetry("drone0")
    assert telem0.vehicle_id == "drone0"
    assert telem0.position_m != (7, 7, 7)    # never received drone1's pushed state


def test_sequence_numbers_tracked_per_vehicle_independently():
    t = _transport()
    t.send_command(_sitl_command(t, vid="drone0", seq=10))
    t.send_command(_sitl_command(t, vid="drone1", seq=0))
    assert t.receive_ack("drone0").accepted
    assert t.receive_ack("drone1").accepted
    assert t._channel("drone0")._last_accepted_sequence == 10
    assert t._channel("drone1")._last_accepted_sequence == 0


def test_distributed_consensus_messages_cannot_be_confused_with_autopilot_messages():
    """Cross-check against swarm_sim.network.CommsNetwork /
    swarm_sim.distributed_consensus: exercising both a CommsNetwork message
    channel and a FakeSITLTransport for the same vehicle ids leaves each
    system's own histories completely disjoint - there is no shared queue
    or storage a message could leak through."""
    from swarm_sim.distributed_consensus import ConsensusMessage

    t = _transport()
    t.send_command(_sitl_command(t, vid="drone0"))
    ack = t.receive_ack("drone0")
    telem = t.receive_telemetry("drone0")

    # Structurally distinct types - no shared base class, no overlapping
    # field name that a careless isinstance/getattr check could confuse.
    assert not isinstance(ack, (ConsensusMessage,))
    assert not isinstance(telem, (ConsensusMessage,))
    assert type(ack).__name__ != "ConsensusMessage"
    assert type(telem).__name__ != "ConsensusMessage"


# ==========================================================================
# 9. SITLAdapter (AutopilotAdapter over SITLTransport)
# ==========================================================================

def test_sitl_adapter_implements_autopilot_adapter_protocol():
    t = _transport(vehicle_ids=("d",))
    assert isinstance(SITLAdapter("d", t.registry.namespace_of("d"), t), AutopilotAdapter)


def test_sitl_adapter_connect_disconnect():
    t = FakeSITLTransport(vehicle_ids=("d",))
    a = SITLAdapter("d", t.registry.namespace_of("d"), t)
    assert a.state == ConnectionState.DISCONNECTED
    result = a.connect()
    assert result.accepted and a.state == ConnectionState.CONNECTED
    a.disconnect()
    assert a.state == ConnectionState.DISCONNECTED


def test_sitl_adapter_send_command_accepted():
    t = _transport(vehicle_ids=("d",))
    a = _connected_adapter(t, "d")
    result = a.send_command(_adapter_command(vid="d", seq=0))
    assert result.accepted
    assert result.transformed_command.command.desired_velocity_mps == (1.0, 0.0, 0.0)


def test_sitl_adapter_wrong_vehicle_id_rejected():
    t = _transport(vehicle_ids=("d",))
    a = _connected_adapter(t, "d")
    result = a.send_command(_adapter_command(vid="other", seq=0))
    assert not result.accepted and result.reason == "wrong_vehicle_id"


def test_sitl_adapter_rejects_command_while_transport_stopped():
    t = FakeSITLTransport(vehicle_ids=("d",))   # never started
    a = SITLAdapter("d", t.registry.namespace_of("d"), t)
    result = a.send_command(_adapter_command(vid="d"))
    assert not result.accepted


def test_sitl_adapter_expired_command_rejected():
    t = _transport(vehicle_ids=("d",), command_latency_s=0.02)
    a = _connected_adapter(t, "d")
    result = a.send_command(_adapter_command(vid="d", t=0.0, ttl=0.01))
    assert not result.accepted and result.reason == "expired"


def test_sitl_adapter_is_command_fresh():
    t = _transport(vehicle_ids=("d",))
    a = _connected_adapter(t, "d")
    cmd = _adapter_command(vid="d", t=1.0, ttl=1.0)
    assert a.is_command_fresh(cmd, now_s=1.5)
    assert not a.is_command_fresh(cmd, now_s=3.0)


def test_sitl_adapter_hold_land_rtl_abort_modes():
    t = _transport(vehicle_ids=("d",))
    a = _connected_adapter(t, "d")
    assert a.request_land(now_s=1.0).accepted and a.mode == AutopilotMode.LAND
    assert a.state == ConnectionState.LANDING
    assert a.request_return_to_launch(now_s=2.0).accepted and a.mode == AutopilotMode.RTL
    assert a.abort(now_s=3.0).accepted and a.mode == AutopilotMode.ABORT


def test_sitl_adapter_replayed_sequence_rejected():
    t = _transport(vehicle_ids=("d",))
    a = _connected_adapter(t, "d")
    a.send_command(_adapter_command(vid="d", seq=5, t=0.0))
    result = a.send_command(_adapter_command(vid="d", seq=2, t=0.0))
    assert not result.accepted and result.reason == "sequence_replayed_stale"


def test_sitl_adapter_push_ground_truth_state_reflected_in_telemetry():
    t = _transport(vehicle_ids=("d",))
    a = _connected_adapter(t, "d")
    from swarm_sim.autopilot.types import VehicleTelemetry as AutopilotVehicleTelemetry
    a.push_ground_truth_state(AutopilotVehicleTelemetry(
        vehicle_id="d", timestamp_s=1.0, frame=Frame.LOCAL_ENU, position_m=(3.0, 4.0, 5.0),
        velocity_mps=(0.0, 0.0, 0.0), acceleration_mps2=None, attitude_rad=(0.0, 0.0, 0.0),
        angular_velocity_radps=None, battery_fraction=0.9, estimator_valid=True,
        connection_state=ConnectionState.CONNECTED, autopilot_mode=AutopilotMode.HOLD, armed=False,
        failsafe=False, sequence=0,
    ))
    telem = a.read_telemetry()
    assert telem.position_m == (3.0, 4.0, 5.0)
    assert telem.battery_fraction == 0.9


def test_sitl_adapter_never_reports_armed_true():
    """No implicit arm, ever - Phase 7 requirement."""
    t = _transport(vehicle_ids=("d",))
    a = _connected_adapter(t, "d")
    for seq in range(3):
        a.send_command(_adapter_command(vid="d", seq=seq, t=float(seq)))
    assert a.armed is False
    assert a.read_telemetry().armed is False


def test_sitl_adapter_deterministic_replay_reject_reason_sequence():
    def run_once():
        t = _transport(vehicle_ids=("d",))
        a = _connected_adapter(t, "d")
        reasons = []
        for seq, ttl in [(0, 1.0), (1, 1.0), (0, 1.0), (5, 1.0)]:
            result = a.send_command(_adapter_command(vid="d", seq=seq, t=0.0, ttl=ttl))
            reasons.append((result.accepted, result.reason))
        return reasons
    assert run_once() == run_once()


# ==========================================================================
# 10. ArduPilot/PX4 SITL adapter skeletons (offline placeholders only)
# ==========================================================================

def test_ardupilot_sitl_skeleton_connect_always_fails():
    a = ArduPilotSITLAdapterSkeleton("d")
    assert isinstance(a, AutopilotAdapter)
    result = a.connect()
    assert not result.accepted
    assert not a.send_command(_adapter_command(vid="d")).accepted


def test_px4_sitl_skeleton_connect_always_fails():
    a = PX4SITLAdapterSkeleton("d")
    assert isinstance(a, AutopilotAdapter)
    result = a.connect()
    assert not result.accepted
    assert not a.send_command(_adapter_command(vid="d")).accepted


# ==========================================================================
# 11. Command schema / mapping specs / safety-decision id
# ==========================================================================

def test_derive_safety_decision_id_deterministic_and_content_sensitive():
    cmd_a = _command(vid="d", vel=(1.0, 0.0, 0.0))
    cmd_b = _command(vid="d", vel=(2.0, 0.0, 0.0))
    id_a1 = derive_safety_decision_id("d", 0.0, cmd_a)
    id_a2 = derive_safety_decision_id("d", 0.0, cmd_a)
    id_b = derive_safety_decision_id("d", 0.0, cmd_b)
    assert id_a1 == id_a2
    assert id_a1 != id_b


def test_sitl_command_carries_namespace_and_safety_decision_id():
    t = _transport(vehicle_ids=("d",))
    cmd = _sitl_command(t, vid="d")
    assert cmd.namespace == t.registry.namespace_of("d")
    assert isinstance(cmd.safety_decision_id, str) and cmd.safety_decision_id


def test_mavlink_mapping_specs_cover_every_frame_and_command_type():
    for frame in Frame:
        assert frame.value in MAVLINK_FRAME_MAPPING_SPEC
    for ctype in CommandType:
        assert ctype.value in MAVLINK_COMMAND_TYPE_MAPPING_SPEC


def test_mavlink_mode_name_mapping_spec_covers_required_modes():
    for mode_name in ("HOLD", "OFFBOARD", "RTL", "LAND", "ABORT"):
        assert mode_name in MAVLINK_MODE_NAME_MAPPING_SPEC
        assert "ardupilot" in MAVLINK_MODE_NAME_MAPPING_SPEC[mode_name]
        assert "px4" in MAVLINK_MODE_NAME_MAPPING_SPEC[mode_name]


def test_transport_result_and_command_ack_and_telemetry_reject_bad_values():
    with pytest.raises(ValueError):
        TransportResult(success="yes", reason="ok", timestamp_s=0.0)
    with pytest.raises(ValueError):
        CommandAck(vehicle_id="d", namespace="sitl/d", command_sequence=0, accepted=True, reason="",
                   timestamp_s=0.0, autopilot_mode=AutopilotMode.HOLD, failsafe=False)
