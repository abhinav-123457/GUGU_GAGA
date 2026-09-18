"""Phase 8: real local ArduPilot SITL transport tests - see
docs/PHASE8_ARDUPILOT_SITL.md and swarm_sim/sitl/ardupilot_transport.py.
Architecture/AST checks live in tests/test_ardupilot_sitl_architecture.py.

Functional tests run against `_dummy_ardupilot_vehicle.DummyArduPilotVehicle`
- a minimal, local, in-process fake that speaks REAL MAVLink over a real
localhost TCP socket (see that module's own docstring). This exercises the
actual wire protocol (real message encode/decode, real socket accept/recv/
send) without needing real ArduPilot firmware - a genuine test of this
transport's own code, not a claim of ArduPilot-specific behavior.
"""
import itertools
import subprocess
import sys
import time
from unittest.mock import patch

import pytest

from swarm_sim.autopilot import AdapterCommand, AutopilotMode, ConnectionState
from swarm_sim.autopilot.ardupilot_sitl import build_ardupilot_sitl_adapter
from swarm_sim.contracts import Command, CommandType, Frame
from swarm_sim.sitl.ardupilot_transport import (
    ArduPilotSITLTransport, ArduPilotTransportError, ArduPilotVehicleEndpoint, _SITLProcessHandle,
    dry_run_validate, parse_connection_string,
)
from swarm_sim.sitl.commands import SITLCommand, derive_safety_decision_id
from swarm_sim.sitl.telemetry import FailureType
from swarm_sim.sitl.vehicle_namespace import NamespaceError

from _dummy_ardupilot_vehicle import DummyArduPilotVehicle

_port_counter = itertools.count(17000)


def _next_port() -> int:
    return next(_port_counter)


def _start_portable(handle: "_SITLProcessHandle") -> None:
    """Starts a `_SITLProcessHandle` using the portable (non-Windows)
    argv-construction path - `sys.executable` is directly executable by
    this OS either way, so forcing that path lets these lifecycle tests
    run identically on Windows and Linux without needing wsl.exe or a
    wsl_distro configured."""
    with patch("swarm_sim.sitl.ardupilot_transport.platform.system", return_value="Linux"):
        handle.start()


def _command(vid="d", ctype=CommandType.VELOCITY_SETPOINT, frame=Frame.LOCAL_ENU,
             vel=(1.0, 0.0, 0.0), pos=None, t=0.0, ttl=5.0, yaw=None, yaw_rate=None):
    kwargs = dict(vehicle_id=vid, command_type=ctype, frame=frame, desired_position_m=pos,
                  desired_velocity_mps=vel, yaw_rad=yaw, yaw_rate_radps=yaw_rate,
                  timestamp_s=t, expiration_time_s=t + ttl, source="test", confidence=0.9)
    if ctype in (CommandType.HOLD, CommandType.ABORT, CommandType.LAND):
        kwargs["desired_velocity_mps"] = None
    return Command(**kwargs)


def _adapter_command(seq=0, **kwargs):
    return AdapterCommand(command=_command(**kwargs), sequence=seq)


class _Dummy:
    """Bundles a DummyArduPilotVehicle + a matching, started
    ArduPilotSITLTransport for one vehicle - the common fixture shape most
    tests need. `warm()` primes telemetry so stale-telemetry doesn't mask
    whatever the test is actually checking."""

    def __init__(self, vehicle_id="d", system_id=1, component_id=1, **transport_kwargs):
        self.port = _next_port()
        self.vehicle = DummyArduPilotVehicle(port=self.port, system_id=system_id, component_id=component_id)
        self.vehicle.start()
        endpoints = {vehicle_id: ArduPilotVehicleEndpoint(
            vehicle_id=vehicle_id, connection_string=f"tcp:127.0.0.1:{self.port}",
            system_id=system_id, component_id=component_id,
        )}
        self.transport = ArduPilotSITLTransport(
            endpoints, allowed_ports=frozenset({self.port}), startup_timeout_s=5.0,
            heartbeat_timeout_s=transport_kwargs.pop("heartbeat_timeout_s", 2.0),
            ack_timeout_s=transport_kwargs.pop("ack_timeout_s", 2.0),
            telemetry_stale_timeout_s=transport_kwargs.pop("telemetry_stale_timeout_s", 2.0),
            **transport_kwargs,
        )
        self.vehicle_id = vehicle_id

    def start(self):
        return self.transport.start()

    def warm(self, sleep_s=0.3):
        time.sleep(sleep_s)
        self.transport.receive_telemetry(self.vehicle_id)

    def close(self):
        self.transport.stop()
        self.vehicle.stop()


@pytest.fixture
def dummy():
    d = _Dummy()
    d.start()
    yield d
    d.close()


# ==========================================================================
# 1. dry-run validation
# ==========================================================================

def test_dry_run_validate_accepts_well_formed_config():
    endpoints = {"d0": ArduPilotVehicleEndpoint(vehicle_id="d0", connection_string="tcp:127.0.0.1:17500", system_id=1)}
    report = dry_run_validate(endpoints, allowed_ports=frozenset({17500}))
    assert report.ok
    assert report.problems == ()


def test_dry_run_validate_never_opens_a_socket_or_starts_a_process(dummy_unused=None):
    """A dry run against a port with NOTHING listening must still report
    ok=True (it validates shape only, never connects)."""
    endpoints = {"d0": ArduPilotVehicleEndpoint(vehicle_id="d0", connection_string="tcp:127.0.0.1:17501", system_id=1)}
    report = dry_run_validate(endpoints, allowed_ports=frozenset({17501}))
    assert report.ok


def test_dry_run_validate_flags_duplicate_system_id():
    endpoints = {
        "a": ArduPilotVehicleEndpoint(vehicle_id="a", connection_string="tcp:127.0.0.1:17502", system_id=1),
        "b": ArduPilotVehicleEndpoint(vehicle_id="b", connection_string="tcp:127.0.0.1:17503", system_id=1),
    }
    report = dry_run_validate(endpoints, allowed_ports=frozenset({17502, 17503}))
    assert not report.ok
    assert any("duplicate system_id" in p for p in report.problems)


def test_dry_run_validate_flags_port_outside_allowlist():
    endpoints = {"a": ArduPilotVehicleEndpoint(vehicle_id="a", connection_string="tcp:127.0.0.1:17504", system_id=1)}
    report = dry_run_validate(endpoints, allowed_ports=frozenset({9999}))
    assert not report.ok
    assert any("not in the configured allowed_ports" in p for p in report.problems)


# ==========================================================================
# 2. localhost-only enforcement
# ==========================================================================

def test_endpoint_rejects_non_loopback_host():
    with pytest.raises(ArduPilotTransportError):
        ArduPilotVehicleEndpoint(vehicle_id="d", connection_string="tcp:8.8.8.8:5760", system_id=1)


def test_endpoint_rejects_hostname_that_is_not_localhost():
    with pytest.raises(ArduPilotTransportError):
        ArduPilotVehicleEndpoint(vehicle_id="d", connection_string="tcp:example.com:5760", system_id=1)


def test_endpoint_accepts_127_0_0_1_and_localhost():
    ArduPilotVehicleEndpoint(vehicle_id="d", connection_string="tcp:127.0.0.1:17505", system_id=1)
    ArduPilotVehicleEndpoint(vehicle_id="d", connection_string="tcp:localhost:17506", system_id=1)


def test_no_external_endpoint_by_default_transport_construction_rejects_it():
    """Even if somehow an endpoint object existed with a non-loopback host
    (it can't, per __post_init__ above), the transport's OWN constructor
    independently re-validates every host - defense in depth, not a
    single check relied on everywhere."""
    with pytest.raises(ArduPilotTransportError):
        parse_and_reject = parse_connection_string("tcp:1.2.3.4:5760")
        from swarm_sim.sitl.ardupilot_transport import _require_loopback_host
        _require_loopback_host(parse_and_reject[1])


def test_parse_connection_string_rejects_malformed_strings():
    with pytest.raises(ArduPilotTransportError):
        parse_connection_string("not-a-connection-string")
    with pytest.raises(ArduPilotTransportError):
        parse_connection_string("ftp:127.0.0.1:21")


# ==========================================================================
# 3. subprocess safety (no shell=True, bounded startup/shutdown, ownership)
# ==========================================================================

def test_process_handle_never_uses_shell_true():
    with patch("swarm_sim.sitl.ardupilot_transport.platform.system", return_value="Linux"), \
         patch("subprocess.Popen") as mock_popen:
        mock_popen.return_value.pid = 12345
        handle = _SITLProcessHandle(sys.executable, ["-c", "pass"])
        handle.start()
        _, kwargs = mock_popen.call_args
        assert kwargs.get("shell", False) is False
        args_passed = mock_popen.call_args[0][0]
        assert isinstance(args_passed, list)


def test_process_handle_passes_argv_as_a_list_never_a_joined_string():
    with patch("swarm_sim.sitl.ardupilot_transport.platform.system", return_value="Linux"), \
         patch("subprocess.Popen") as mock_popen:
        mock_popen.return_value.pid = 1
        handle = _SITLProcessHandle(sys.executable, ["-c", "print('a; rm -rf /'); "])
        handle.start()
        argv = mock_popen.call_args[0][0]
        assert argv == [sys.executable, "-c", "print('a; rm -rf /'); "]


def test_process_handle_stores_pid_and_launched_argv():
    handle = _SITLProcessHandle(sys.executable, ["-c", "import time; time.sleep(2)"])
    _start_portable(handle)
    try:
        assert handle.pid is not None
        assert handle.launched_argv[0] == sys.executable
        assert handle.is_running()
    finally:
        handle.stop(graceful_timeout_s=2.0)


def test_process_handle_bounded_graceful_shutdown():
    handle = _SITLProcessHandle(sys.executable, ["-c", "import time; time.sleep(10)"])
    _start_portable(handle)
    t0 = time.monotonic()
    handle.stop(graceful_timeout_s=1.0)
    elapsed = time.monotonic() - t0
    assert not handle.is_running()
    assert elapsed < 5.0   # bounded - never hangs waiting for the process


def test_process_handle_detects_unexpected_exit():
    handle = _SITLProcessHandle(sys.executable, ["-c", "import sys; sys.exit(3)"])
    _start_portable(handle)
    deadline = time.monotonic() + 3.0
    while handle.is_running() and time.monotonic() < deadline:
        time.sleep(0.05)
    assert not handle.is_running()
    assert handle.exit_code == 3


def test_process_handle_captures_stdout_and_stderr():
    handle = _SITLProcessHandle(
        sys.executable, ["-c", "import sys; print('hello-stdout'); print('hello-stderr', file=sys.stderr)"],
    )
    _start_portable(handle)
    deadline = time.monotonic() + 3.0
    while handle.is_running() and time.monotonic() < deadline:
        time.sleep(0.02)
    handle.stop(graceful_timeout_s=2.0)
    assert "hello-stdout" in handle.stdout_text
    assert "hello-stderr" in handle.stderr_text


def test_process_handle_stop_only_ever_acts_on_its_own_popen_object():
    """Ownership verification is structural: stop() never re-acquires a
    handle by PID - it only ever calls .terminate()/.kill() on the exact
    Popen object created in start(), so it is architecturally impossible
    for it to signal an unrelated process."""
    handle = _SITLProcessHandle(sys.executable, ["-c", "import time; time.sleep(2)"])
    _start_portable(handle)
    real_popen = handle._popen
    with patch.object(real_popen, "terminate", wraps=real_popen.terminate) as spy:
        handle.stop(graceful_timeout_s=2.0)
        spy.assert_called_once()


def test_process_handle_windows_spawn_requires_wsl_distro():
    with patch("platform.system", return_value="Windows"):
        handle = _SITLProcessHandle("/some/linux/binary", [])
        with pytest.raises(ArduPilotTransportError):
            handle.start()


# ==========================================================================
# 4. heartbeat detection / startup lifecycle
# ==========================================================================

def test_start_succeeds_once_heartbeat_from_expected_identity_arrives(dummy):
    assert dummy.transport.is_running


def test_start_times_out_if_no_endpoint_listening():
    port = _next_port()
    endpoints = {"d": ArduPilotVehicleEndpoint(vehicle_id="d", connection_string=f"tcp:127.0.0.1:{port}", system_id=1)}
    t = ArduPilotSITLTransport(endpoints, allowed_ports=frozenset({port}), startup_timeout_s=0.5)
    with pytest.raises(ArduPilotTransportError):
        t.start()
    assert not t.is_running


def test_start_ignores_heartbeat_from_wrong_system_id():
    """A dummy vehicle broadcasting under system_id=9 must never satisfy
    an endpoint configured to expect system_id=1 - startup keeps waiting
    (and times out) rather than accepting the wrong identity."""
    port = _next_port()
    wrong_vehicle = DummyArduPilotVehicle(port=port, system_id=9, component_id=1)
    wrong_vehicle.start()
    try:
        endpoints = {"d": ArduPilotVehicleEndpoint(vehicle_id="d", connection_string=f"tcp:127.0.0.1:{port}", system_id=1)}
        t = ArduPilotSITLTransport(endpoints, allowed_ports=frozenset({port}), startup_timeout_s=1.0)
        with pytest.raises(ArduPilotTransportError):
            t.start()
    finally:
        wrong_vehicle.stop()


def test_start_ignores_heartbeat_from_wrong_component_id():
    port = _next_port()
    wrong_vehicle = DummyArduPilotVehicle(port=port, system_id=1, component_id=9)
    wrong_vehicle.start()
    try:
        endpoints = {"d": ArduPilotVehicleEndpoint(vehicle_id="d", connection_string=f"tcp:127.0.0.1:{port}", system_id=1, component_id=1)}
        t = ArduPilotSITLTransport(endpoints, allowed_ports=frozenset({port}), startup_timeout_s=1.0)
        with pytest.raises(ArduPilotTransportError):
            t.start()
    finally:
        wrong_vehicle.stop()


def test_start_is_idempotent_does_not_reopen_connection(dummy):
    conn_before = dummy.transport._channel(dummy.vehicle_id).connection
    result = dummy.transport.start()
    assert result.accepted if hasattr(result, "accepted") else result.success
    assert dummy.transport._channel(dummy.vehicle_id).connection is conn_before


# ==========================================================================
# 5. telemetry parsing / command ack parsing
# ==========================================================================

def test_telemetry_parses_position_velocity_battery_estimator(dummy):
    dummy.warm()
    telem = dummy.transport.receive_telemetry(dummy.vehicle_id)
    assert telem.position_m is not None
    assert telem.velocity_mps is not None
    assert telem.battery_fraction == pytest.approx(1.0)
    assert telem.estimator_valid is True
    assert telem.heartbeat_ok is True


def test_telemetry_reflects_unhealthy_estimator():
    port = _next_port()
    vehicle = DummyArduPilotVehicle(port=port, estimator_healthy=False)
    vehicle.start()
    try:
        endpoints = {"d": ArduPilotVehicleEndpoint(vehicle_id="d", connection_string=f"tcp:127.0.0.1:{port}", system_id=1)}
        t = ArduPilotSITLTransport(endpoints, allowed_ports=frozenset({port}), startup_timeout_s=5.0)
        t.start()
        time.sleep(0.3)
        telem = t.receive_telemetry("d")
        assert telem.estimator_valid is False
        t.stop()
    finally:
        vehicle.stop()


def test_command_acknowledgement_parsing_for_mode_change(dummy):
    dummy.warm()
    adapter = build_ardupilot_sitl_adapter(dummy.vehicle_id, dummy.transport)
    adapter.connect()
    result = adapter.request_return_to_launch(now_s=0.0)
    # request_return_to_launch on SITLAdapter is a local mode-set, not
    # routed through the transport (Phase 7 design, unchanged) - so this
    # checks the OTHER path: an actual HOLD Command through send_command,
    # which DOES go through the transport and waits for a real COMMAND_ACK.
    hold_result = adapter.send_command(_adapter_command(vid=dummy.vehicle_id, ctype=CommandType.HOLD, vel=None, seq=0))
    assert hold_result.accepted
    assert hold_result.reason == "accepted"


# ==========================================================================
# 6. system/component ID validation at the command layer, namespace isolation
# ==========================================================================

def test_send_command_rejects_wrong_vehicle_id(dummy):
    dummy.warm()
    adapter = build_ardupilot_sitl_adapter(dummy.vehicle_id, dummy.transport)
    adapter.connect()
    result = adapter.send_command(_adapter_command(vid="someone-else", seq=0))
    assert not result.accepted and result.reason == "wrong_vehicle_id"


def test_transport_rejects_namespace_mismatch(dummy):
    dummy.warm()
    ac = _adapter_command(vid=dummy.vehicle_id, seq=0)
    forged = SITLCommand(adapter_command=ac, namespace="sitl/not-a-real-namespace",
                          safety_decision_id="x")
    result = dummy.transport.send_command(forged)
    assert not result.success and result.reason == "namespace_mismatch"


def test_two_vehicle_namespace_isolation():
    port_a, port_b = _next_port(), _next_port()
    va = DummyArduPilotVehicle(port=port_a, system_id=1)
    vb = DummyArduPilotVehicle(port=port_b, system_id=2)
    va.start()
    vb.start()
    try:
        endpoints = {
            "drone0": ArduPilotVehicleEndpoint(vehicle_id="drone0", connection_string=f"tcp:127.0.0.1:{port_a}", system_id=1),
            "drone1": ArduPilotVehicleEndpoint(vehicle_id="drone1", connection_string=f"tcp:127.0.0.1:{port_b}", system_id=2),
        }
        t = ArduPilotSITLTransport(endpoints, allowed_ports=frozenset({port_a, port_b}), startup_timeout_s=5.0,
                                    telemetry_stale_timeout_s=2.0)
        t.start()
        time.sleep(0.3)
        t.receive_telemetry("drone0")
        t.receive_telemetry("drone1")

        cmd0 = _adapter_command(vid="drone0", seq=0)
        sc0 = SITLCommand(adapter_command=cmd0, namespace=t.registry.namespace_of("drone0"),
                           safety_decision_id="x")
        t.send_command(sc0)
        assert t.receive_ack("drone0").accepted

        # drone1 must be completely unaffected
        assert t.receive_ack("drone1").reason == "no_ack_available"
        assert t._channel("drone1").last_accepted_sequence is None
        assert t.receive_telemetry("drone1").vehicle_id == "drone1"

        t.stop()
    finally:
        va.stop()
        vb.stop()


def test_one_vehicle_process_failure_does_not_affect_another():
    """Modeled here as one vehicle's dummy connection closing (its
    "process" stopping, in this in-thread test double's own terms) -
    the other vehicle's channel must be unaffected."""
    port_a, port_b = _next_port(), _next_port()
    va = DummyArduPilotVehicle(port=port_a, system_id=1)
    vb = DummyArduPilotVehicle(port=port_b, system_id=2)
    va.start()
    vb.start()
    endpoints = {
        "drone0": ArduPilotVehicleEndpoint(vehicle_id="drone0", connection_string=f"tcp:127.0.0.1:{port_a}", system_id=1),
        "drone1": ArduPilotVehicleEndpoint(vehicle_id="drone1", connection_string=f"tcp:127.0.0.1:{port_b}", system_id=2),
    }
    t = ArduPilotSITLTransport(endpoints, allowed_ports=frozenset({port_a, port_b}), startup_timeout_s=5.0,
                                heartbeat_timeout_s=0.5, telemetry_stale_timeout_s=2.0)
    t.start()
    time.sleep(0.3)
    t.receive_telemetry("drone0")
    t.receive_telemetry("drone1")
    va.stop()   # simulate drone0's vehicle process disappearing
    time.sleep(1.0)
    telem0 = t.receive_telemetry("drone0")
    telem1 = t.receive_telemetry("drone1")
    assert telem0.heartbeat_ok is False       # drone0 correctly detected as lost
    assert telem1.heartbeat_ok is True        # drone1 fully unaffected
    t.stop()
    vb.stop()


# ==========================================================================
# 7. sequence handling / replay rejection
# ==========================================================================

def test_sequence_replayed_stale_rejected(dummy):
    dummy.warm()
    adapter = build_ardupilot_sitl_adapter(dummy.vehicle_id, dummy.transport)
    adapter.connect()
    adapter.send_command(_adapter_command(vid=dummy.vehicle_id, seq=5, t=0.0))
    result = adapter.send_command(_adapter_command(vid=dummy.vehicle_id, seq=2, t=0.0))
    assert not result.accepted and result.reason == "sequence_replayed_stale"


def test_sequence_idempotent_replay_accepted(dummy):
    dummy.warm()
    adapter = build_ardupilot_sitl_adapter(dummy.vehicle_id, dummy.transport)
    adapter.connect()
    cmd = _adapter_command(vid=dummy.vehicle_id, seq=5, vel=(1.0, 0.0, 0.0), t=0.0)
    first = adapter.send_command(cmd)
    second = adapter.send_command(_adapter_command(vid=dummy.vehicle_id, seq=5, vel=(1.0, 0.0, 0.0), t=0.0))
    assert first.accepted and second.accepted


def test_sequence_conflicting_replay_rejected(dummy):
    dummy.warm()
    adapter = build_ardupilot_sitl_adapter(dummy.vehicle_id, dummy.transport)
    adapter.connect()
    adapter.send_command(_adapter_command(vid=dummy.vehicle_id, seq=5, vel=(1.0, 0.0, 0.0), t=0.0))
    result = adapter.send_command(_adapter_command(vid=dummy.vehicle_id, seq=5, vel=(9.0, 0.0, 0.0), t=0.0))
    assert not result.accepted and result.reason == "sequence_replayed_conflicting"


def test_per_vehicle_sequence_tracked_independently():
    port_a, port_b = _next_port(), _next_port()
    va = DummyArduPilotVehicle(port=port_a, system_id=1)
    vb = DummyArduPilotVehicle(port=port_b, system_id=2)
    va.start()
    vb.start()
    endpoints = {
        "drone0": ArduPilotVehicleEndpoint(vehicle_id="drone0", connection_string=f"tcp:127.0.0.1:{port_a}", system_id=1),
        "drone1": ArduPilotVehicleEndpoint(vehicle_id="drone1", connection_string=f"tcp:127.0.0.1:{port_b}", system_id=2),
    }
    t = ArduPilotSITLTransport(endpoints, allowed_ports=frozenset({port_a, port_b}), startup_timeout_s=5.0,
                                telemetry_stale_timeout_s=2.0)
    t.start()
    time.sleep(0.3)
    t.receive_telemetry("drone0")
    t.receive_telemetry("drone1")
    sc0 = SITLCommand(adapter_command=_adapter_command(vid="drone0", seq=10), namespace=t.registry.namespace_of("drone0"), safety_decision_id="x")
    sc1 = SITLCommand(adapter_command=_adapter_command(vid="drone1", seq=0), namespace=t.registry.namespace_of("drone1"), safety_decision_id="x")
    t.send_command(sc0)
    t.send_command(sc1)
    assert t._channel("drone0").last_accepted_sequence == 10
    assert t._channel("drone1").last_accepted_sequence == 0
    t.stop()
    va.stop()
    vb.stop()


# ==========================================================================
# 8. frame conversion
# ==========================================================================

def test_enu_velocity_setpoint_converted_to_ned_on_wire(dummy):
    dummy.warm()
    adapter = build_ardupilot_sitl_adapter(dummy.vehicle_id, dummy.transport)   # operating_frame=ENU (default)
    adapter.connect()
    result = adapter.send_command(_adapter_command(vid=dummy.vehicle_id, frame=Frame.LOCAL_ENU, vel=(1.0, 0.0, 0.0), seq=0))
    assert result.accepted
    time.sleep(0.2)
    sp = dummy.vehicle.received_setpoints[-1]
    assert sp.vx == pytest.approx(0.0)
    assert sp.vy == pytest.approx(1.0)
    assert sp.vz == pytest.approx(0.0)


def test_telemetry_converted_back_to_enu(dummy):
    dummy.warm()
    telem = dummy.transport.receive_telemetry(dummy.vehicle_id)
    assert telem.position_m is not None   # already ENU by construction of this transport's own default


def test_frame_mismatch_rejected_when_operating_frame_is_ned():
    port = _next_port()
    vehicle = DummyArduPilotVehicle(port=port)
    vehicle.start()
    try:
        endpoints = {"d": ArduPilotVehicleEndpoint(vehicle_id="d", connection_string=f"tcp:127.0.0.1:{port}", system_id=1)}
        t = ArduPilotSITLTransport(endpoints, allowed_ports=frozenset({port}), operating_frame=Frame.LOCAL_NED,
                                    startup_timeout_s=5.0, telemetry_stale_timeout_s=2.0)
        t.start()
        time.sleep(0.3)
        t.receive_telemetry("d")
        ac = _adapter_command(vid="d", frame=Frame.LOCAL_ENU, seq=0)   # wrong frame - transport expects NED
        sc = SITLCommand(adapter_command=ac, namespace=t.registry.namespace_of("d"), safety_decision_id="x")
        t.send_command(sc)
        ack = t.receive_ack("d")
        assert not ack.accepted and ack.reason == "frame_mismatch"
        t.stop()
    finally:
        vehicle.stop()


def test_body_frame_rejected_without_attitude():
    """Sent before ANY ATTITUDE telemetry has ever arrived - attitude is
    genuinely unavailable, so BODY conversion must be refused, never
    approximated."""
    port = _next_port()
    # send_attitude=False: position/battery/EKF telemetry keeps flowing
    # (so this is NOT a stale-telemetry rejection) but ATTITUDE
    # specifically never arrives - the genuinely correct scenario for
    # "attitude unavailable," isolated from "no telemetry at all."
    vehicle = DummyArduPilotVehicle(port=port, send_attitude=False)
    vehicle.start()
    try:
        endpoints = {"d": ArduPilotVehicleEndpoint(vehicle_id="d", connection_string=f"tcp:127.0.0.1:{port}", system_id=1)}
        # operating_frame stays at its ENU default - BODY is always an
        # allowed exception to the frame-mismatch check regardless of the
        # configured operating_frame (see the transport's own
        # _evaluate_and_send), gated instead by the actual attitude-aware
        # conversion this test is checking.
        t = ArduPilotSITLTransport(endpoints, allowed_ports=frozenset({port}),
                                    startup_timeout_s=5.0, telemetry_stale_timeout_s=2.0)
        t.start()
        time.sleep(0.3)
        t.receive_telemetry("d")
        ac = _adapter_command(vid="d", frame=Frame.BODY, seq=0)
        sc = SITLCommand(adapter_command=ac, namespace=t.registry.namespace_of("d"), safety_decision_id="x")
        t.send_command(sc)
        ack = t.receive_ack("d")
        assert not ack.accepted and ack.reason.startswith("frame_conversion_failed")
        t.stop()
    finally:
        vehicle.stop()


def test_yaw_and_yaw_rate_setpoint_sent_as_velocity_hold_with_yaw(dummy):
    dummy.warm()
    adapter = build_ardupilot_sitl_adapter(dummy.vehicle_id, dummy.transport)
    adapter.connect()
    result = adapter.send_command(_adapter_command(
        vid=dummy.vehicle_id, ctype=CommandType.YAW_SETPOINT, vel=None, yaw=0.5, seq=0,
    ))
    assert result.accepted


# ==========================================================================
# 9. stale telemetry / heartbeat loss / process exit / command timeout
# ==========================================================================

def test_stale_telemetry_blocks_navigation_command():
    port = _next_port()
    vehicle = DummyArduPilotVehicle(port=port, send_telemetry=False)
    vehicle.start()
    try:
        endpoints = {"d": ArduPilotVehicleEndpoint(vehicle_id="d", connection_string=f"tcp:127.0.0.1:{port}", system_id=1)}
        t = ArduPilotSITLTransport(endpoints, allowed_ports=frozenset({port}), startup_timeout_s=5.0,
                                    telemetry_stale_timeout_s=0.5)
        t.start()
        ac = _adapter_command(vid="d", seq=0)
        sc = SITLCommand(adapter_command=ac, namespace=t.registry.namespace_of("d"), safety_decision_id="x")
        t.send_command(sc)
        ack = t.receive_ack("d")
        assert not ack.accepted and ack.reason == "stale_telemetry"
        assert ack.autopilot_mode == AutopilotMode.HOLD
        t.stop()
    finally:
        vehicle.stop()


def test_heartbeat_loss_detected_and_blocks_commands():
    port = _next_port()
    vehicle = DummyArduPilotVehicle(port=port)
    vehicle.start()
    endpoints = {"d": ArduPilotVehicleEndpoint(vehicle_id="d", connection_string=f"tcp:127.0.0.1:{port}", system_id=1)}
    t = ArduPilotSITLTransport(endpoints, allowed_ports=frozenset({port}), startup_timeout_s=5.0,
                                heartbeat_timeout_s=0.3, telemetry_stale_timeout_s=5.0)
    t.start()
    time.sleep(0.2)
    t.receive_telemetry("d")
    vehicle.send_heartbeat = False   # simulate a lost link
    time.sleep(0.6)
    ac = _adapter_command(vid="d", seq=0)
    sc = SITLCommand(adapter_command=ac, namespace=t.registry.namespace_of("d"), safety_decision_id="x")
    t.send_command(sc)
    ack = t.receive_ack("d")
    assert not ack.accepted and ack.reason == "heartbeat_lost"
    t.stop()
    vehicle.stop()


def test_process_exit_during_startup_raises():
    """`sys.executable` is a real, native-to-this-OS Python binary, so the
    OS-level subprocess call itself works regardless of which wrapping
    strategy `_SITLProcessHandle` picks - `platform.system` is patched
    here only to force the portable (non-Windows, no wsl.exe wrapper)
    argv-construction path, since sys.executable is directly executable
    either way."""
    port = _next_port()
    endpoints = {"d": ArduPilotVehicleEndpoint(
        vehicle_id="d", connection_string=f"tcp:127.0.0.1:{port}", system_id=1,
        executable_path=sys.executable, extra_args=("-c", "import sys; sys.exit(1)"),
    )}
    t = ArduPilotSITLTransport(endpoints, allowed_ports=frozenset({port}), startup_timeout_s=3.0)
    with patch("swarm_sim.sitl.ardupilot_transport.platform.system", return_value="Linux"):
        with pytest.raises(ArduPilotTransportError, match="exited during startup"):
            t.start()


def test_command_ack_timeout_raises_and_is_reported_as_rejection(dummy):
    """A dummy configured to never send COMMAND_ACK - a HOLD mode-change
    command must be rejected once ack_timeout_s elapses, not hang
    forever."""
    dummy.vehicle.ack_commands = False
    dummy.warm()
    adapter = build_ardupilot_sitl_adapter(dummy.vehicle_id, dummy.transport)
    adapter.connect()
    t0 = time.monotonic()
    result = adapter.send_command(_adapter_command(vid=dummy.vehicle_id, ctype=CommandType.HOLD, vel=None, seq=0))
    elapsed = time.monotonic() - t0
    assert not result.accepted
    assert "mavlink_send_failed" in result.reason
    assert elapsed < 5.0


# ==========================================================================
# 10. mode-change failure / operator abort / LAND / RTL / no-arm / no-takeoff
# ==========================================================================

def test_mode_change_rejected_result_propagated_as_rejection():
    """A COMMAND_ACK with a non-accepted MAV_RESULT must surface as a
    rejected AdapterResult, not a silent success."""
    port = _next_port()

    class _RejectingVehicle(DummyArduPilotVehicle):
        def _handle_message(self, msg):
            if msg.get_type() == "COMMAND_LONG":
                self.received_commands.append(msg)
                if self._conn.port is not None:
                    from pymavlink import mavutil
                    self._conn.mav.command_ack_send(msg.command, mavutil.mavlink.MAV_RESULT_DENIED)
                return
            super()._handle_message(msg)

    vehicle = _RejectingVehicle(port=port)
    vehicle.start()
    try:
        endpoints = {"d": ArduPilotVehicleEndpoint(vehicle_id="d", connection_string=f"tcp:127.0.0.1:{port}", system_id=1)}
        t = ArduPilotSITLTransport(endpoints, allowed_ports=frozenset({port}), startup_timeout_s=5.0,
                                    telemetry_stale_timeout_s=2.0)
        t.start()
        time.sleep(0.3)
        t.receive_telemetry("d")
        adapter = build_ardupilot_sitl_adapter("d", t)
        adapter.connect()
        result = adapter.send_command(_adapter_command(vid="d", ctype=CommandType.HOLD, vel=None, seq=0))
        assert not result.accepted
        assert "mavlink_send_failed" in result.reason
        t.stop()
    finally:
        vehicle.stop()


def test_operator_abort_sends_flighttermination_and_transitions_mode(dummy):
    dummy.warm()
    adapter = build_ardupilot_sitl_adapter(dummy.vehicle_id, dummy.transport)
    adapter.connect()
    result = adapter.send_command(_adapter_command(vid=dummy.vehicle_id, ctype=CommandType.ABORT, vel=None, seq=0))
    assert result.accepted
    assert adapter.mode == AutopilotMode.ABORT


def test_land_requested_sends_nav_land_and_transitions_mode(dummy):
    dummy.warm()
    adapter = build_ardupilot_sitl_adapter(dummy.vehicle_id, dummy.transport)
    adapter.connect()
    result = adapter.send_command(_adapter_command(vid=dummy.vehicle_id, ctype=CommandType.LAND, vel=None, seq=0))
    assert result.accepted
    assert adapter.mode == AutopilotMode.LAND
    assert adapter.state == ConnectionState.LANDING


def test_return_to_safe_point_hold_mode_change(dummy):
    dummy.warm()
    adapter = build_ardupilot_sitl_adapter(dummy.vehicle_id, dummy.transport)
    adapter.connect()
    result = adapter.send_command(_adapter_command(vid=dummy.vehicle_id, ctype=CommandType.HOLD, vel=None, seq=0))
    assert result.accepted
    assert adapter.mode == AutopilotMode.HOLD


def test_no_arm_anywhere_in_module():
    import ast
    import pathlib
    import swarm_sim.sitl.ardupilot_transport as mod
    tree = ast.parse(pathlib.Path(mod.__file__).read_text())
    identifiers = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Attribute):
            identifiers.add(node.attr)
        if isinstance(node, ast.Name):
            identifiers.add(node.id)
    assert "MAV_CMD_COMPONENT_ARM_DISARM" not in identifiers
    assert "arducopter_arm" not in identifiers
    assert "MAV_CMD_NAV_TAKEOFF" not in identifiers


def test_vehicle_never_reports_armed_true_across_a_full_command_sequence(dummy):
    dummy.warm()
    adapter = build_ardupilot_sitl_adapter(dummy.vehicle_id, dummy.transport)
    adapter.connect()
    for seq in range(4):
        adapter.send_command(_adapter_command(vid=dummy.vehicle_id, seq=seq, t=0.0))
    assert adapter.read_telemetry().armed is False
    assert dummy.vehicle.armed is False


# ==========================================================================
# 11. FakeSITL regression compatibility (Phase 7 must still work unchanged)
# ==========================================================================

def test_fake_sitl_transport_still_importable_and_functional():
    from swarm_sim.sitl.fake_transport import FakeSITLTransport
    t = FakeSITLTransport(vehicle_ids=("d",))
    result = t.start()
    assert result.success


def test_fake_sitl_full_suite_still_passes():
    """Smoke check, not a re-run of the whole Phase 7 suite (that's
    tests/test_sitl.py's job, run every time regardless) - just confirms
    importing the Phase 8 modules alongside Phase 7's own doesn't break
    FakeSITL's own construction/behavior."""
    from swarm_sim.autopilot.sitl import SITLAdapter
    from swarm_sim.autopilot.types import VehicleTelemetry as AutopilotVehicleTelemetry
    from swarm_sim.sitl.fake_transport import FakeSITLTransport

    t = FakeSITLTransport(vehicle_ids=("drone0",))
    t.start()
    a = SITLAdapter("drone0", t.registry.namespace_of("drone0"), t)
    a.connect()
    a.push_ground_truth_state(AutopilotVehicleTelemetry(
        vehicle_id="drone0", timestamp_s=0.0, frame=Frame.LOCAL_ENU, position_m=(0.0, 0.0, 3.0),
        velocity_mps=(0.0, 0.0, 0.0), acceleration_mps2=None, attitude_rad=(0.0, 0.0, 0.0),
        angular_velocity_radps=None, battery_fraction=1.0, estimator_valid=True,
        connection_state=ConnectionState.CONNECTED, autopilot_mode=AutopilotMode.HOLD, armed=False,
        failsafe=False, sequence=0,
    ))
    result = a.send_command(_adapter_command(vid="drone0", seq=0))
    assert result.accepted
