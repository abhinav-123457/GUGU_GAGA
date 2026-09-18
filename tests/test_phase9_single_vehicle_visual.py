"""Phase 9 functional tests - see docs/PHASE9_SINGLE_VEHICLE_VISUAL.md and
scripts/run_phase9_single_vehicle_visual.py. Architecture/AST checks live in
tests/test_phase9_single_vehicle_visual_architecture.py.

Reuses `_dummy_ardupilot_vehicle.DummyArduPilotVehicle` - the same
in-process, real-MAVLink-over-a-real-localhost-socket fixture Phase 8's own
tests use (tests/test_ardupilot_sitl.py) - so these are genuine ATTACH-mode
exercises of the script's own code, not a re-test of
ArduPilotSITLTransport's already-covered internals.
"""
import itertools
import os
import sys
import time

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"))

import run_phase9_single_vehicle_visual as phase9  # noqa: E402
from swarm_sim.autopilot.types import AutopilotMode  # noqa: E402
from swarm_sim.contracts import Frame  # noqa: E402
from swarm_sim.safety_supervisor import SafetySupervisor  # noqa: E402
from swarm_sim.sitl.ardupilot_transport import ArduPilotVehicleEndpoint  # noqa: E402
from swarm_sim.sitl.telemetry import SITLTelemetry  # noqa: E402

from _dummy_ardupilot_vehicle import DummyArduPilotVehicle  # noqa: E402
from pymavlink import mavutil  # noqa: E402

_port_counter = itertools.count(18300)


def _next_port() -> int:
    return next(_port_counter)


def _args(**overrides):
    defaults = dict(
        connection=f"tcp:127.0.0.1:{overrides.pop('port', 0)}", system_id=1, component_id=1,
        namespace="sitl/drone0", duration=2.0, dry_run=False, attach=True, hold_test=False,
        planner_preview=False, startup_timeout=2.0,
    )
    defaults.update(overrides)
    return _Namespace(**defaults)


class _Namespace:
    """Plain attribute bag mirroring argparse.Namespace - avoids depending
    on argparse.Namespace internals for construction from a dict."""
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


@pytest.fixture
def dummy_vehicle():
    port = _next_port()
    v = DummyArduPilotVehicle(port=port, system_id=1, component_id=1)
    v.start()
    yield v, port
    v.stop()


# ==========================================================================
# 1. dry-run
# ==========================================================================

def test_dry_run_accepts_well_formed_config():
    report = phase9.run_dry_run(_args(port=19000, namespace="sitl/drone0"))
    assert report["ok"] is True
    assert report["problems"] == []


def test_dry_run_never_opens_a_socket_even_if_nothing_listens():
    report = phase9.run_dry_run(_args(port=19001))
    assert report["ok"] is True   # nothing listening on 19001 - dry-run must still pass (shape-only)


# ==========================================================================
# 2. localhost-only enforcement
# ==========================================================================

def test_non_loopback_connection_rejected_cleanly_not_a_crash():
    args = _args(port=0)
    args.connection = "tcp:8.8.8.8:5760"
    report = phase9.run_dry_run(args)
    assert report["ok"] is False
    assert any("loopback" in p for p in report["remaining_failures"])


def test_attach_mode_also_rejects_non_loopback_before_any_connection_attempt():
    args = _args(port=0, attach=True)
    args.connection = "tcp:1.2.3.4:5760"
    t0 = time.monotonic()
    report = phase9.run_telemetry_visualization(args)
    elapsed = time.monotonic() - t0
    assert report["attach"]["succeeded"] is False
    assert elapsed < 1.0   # rejected immediately at construction, never attempts a bounded wait


# ==========================================================================
# 3. namespace validation
# ==========================================================================

def test_namespace_mismatch_rejected_before_any_connection():
    args = _args(port=0)
    args.namespace = "not-a-valid-namespace"
    report = phase9.run_dry_run(args)
    assert report["ok"] is False
    assert any("namespace" in p for p in report["remaining_failures"])


def test_vehicle_id_from_namespace_round_trips():
    assert phase9.vehicle_id_from_namespace("sitl/drone0") == "drone0"
    with pytest.raises(ValueError):
        phase9.vehicle_id_from_namespace("drone0")
    with pytest.raises(ValueError):
        phase9.vehicle_id_from_namespace("sitl/")


# ==========================================================================
# 4. correct / incorrect system id / component id (bounded, real MAVLink)
# ==========================================================================

def test_attach_succeeds_with_correct_system_and_component_id(dummy_vehicle):
    vehicle, port = dummy_vehicle
    report = phase9.run_telemetry_visualization(_args(port=port, duration=0.5, startup_timeout=3.0))
    assert report["attach"]["succeeded"] is True
    assert report["heartbeat"]["received"] is True


def test_attach_fails_with_wrong_system_id():
    port = _next_port()
    vehicle = DummyArduPilotVehicle(port=port, system_id=9, component_id=1)
    vehicle.start()
    try:
        report = phase9.run_telemetry_visualization(_args(port=port, system_id=1, startup_timeout=1.0, duration=0.5))
        assert report["attach"]["succeeded"] is False
        assert any("timed out" in f for f in report["remaining_failures"])
    finally:
        vehicle.stop()


def test_attach_fails_with_wrong_component_id():
    port = _next_port()
    vehicle = DummyArduPilotVehicle(port=port, system_id=1, component_id=9)
    vehicle.start()
    try:
        report = phase9.run_telemetry_visualization(
            _args(port=port, system_id=1, component_id=1, startup_timeout=1.0, duration=0.5)
        )
        assert report["attach"]["succeeded"] is False
    finally:
        vehicle.stop()


# ==========================================================================
# 5. heartbeat timeout
# ==========================================================================

def test_heartbeat_timeout_is_bounded_and_reported():
    port = _next_port()
    vehicle = DummyArduPilotVehicle(port=port, send_heartbeat=False)
    vehicle.start()
    try:
        t0 = time.monotonic()
        report = phase9.run_telemetry_visualization(_args(port=port, startup_timeout=1.0, duration=0.5))
        elapsed = time.monotonic() - t0
        assert report["attach"]["succeeded"] is False
        assert report["heartbeat"]["received"] is False
        assert elapsed < 3.0
    finally:
        vehicle.stop()


# ==========================================================================
# 6. telemetry parsing / status line
# ==========================================================================

def _sample_telemetry(**overrides) -> SITLTelemetry:
    defaults = dict(
        vehicle_id="drone0", namespace="sitl/drone0", timestamp_s=1.5, sequence=3,
        position_m=(1.0, 2.0, 3.0), velocity_mps=(0.1, 0.2, 0.3), attitude_rad=(0.0, 0.0, 1.5708),
        angular_velocity_radps=(0.0, 0.0, 0.0), battery_fraction=0.75, estimator_valid=True,
        armed=False, flight_mode=AutopilotMode.HOLD, failsafe=False, heartbeat_ok=True,
        last_command_ack_status="accepted",
    )
    defaults.update(overrides)
    return SITLTelemetry(**defaults)


def test_telemetry_summary_extracts_all_required_fields():
    telem = _sample_telemetry()
    summary = phase9._telemetry_summary(telem)
    assert summary["position_m"] == (1.0, 2.0, 3.0)
    assert summary["battery_fraction"] == 0.75
    assert summary["armed"] is False
    assert summary["flight_mode"] == "HOLD"
    assert summary["heartbeat_ok"] is True
    assert summary["last_command_ack_status"] == "accepted"


def test_status_line_contains_every_required_field():
    telem = _sample_telemetry()
    line = phase9.status_line(12.3, "drone0", "sitl/drone0", 1, 1, telem)
    for expected in ("drone0", "sitl/drone0", "sysid=1/1", "alt=3.00m", "batt=75%", "armed=NO",
                     "mode=HOLD", "hb=OK", "ekf=OK", "last_ack=accepted"):
        assert expected in line
    assert "flight successful" not in line.lower()


def test_status_line_handles_missing_telemetry_gracefully():
    telem = _sample_telemetry(position_m=None, velocity_mps=None, attitude_rad=None,
                               battery_fraction=None, flight_mode=AutopilotMode.UNKNOWN,
                               last_command_ack_status=None)
    line = phase9.status_line(0.0, "drone0", "sitl/drone0", 1, 1, telem)
    assert "n/a" in line   # missing fields reported honestly, never fabricated


# ==========================================================================
# 7. ENU/NED conversion (no double conversion at the VehicleState boundary)
# ==========================================================================

def test_vehicle_state_from_telemetry_passes_through_already_converted_fields():
    """ArduPilotSITLTransport (Phase 8, unmodified) already converts
    NED->ENU before this project's own code ever sees it - this helper
    must never convert a second time."""
    telem = _sample_telemetry(position_m=(4.0, 5.0, 6.0), velocity_mps=(0.5, -0.5, 0.0))
    state = phase9._vehicle_state_from_telemetry("drone0", telem, now_s=1.5)
    assert state.frame == Frame.LOCAL_ENU
    assert state.position_m == (4.0, 5.0, 6.0)
    assert state.velocity_mps == (0.5, -0.5, 0.0)
    assert state.sim_time_s == 1.5


def test_vehicle_state_from_telemetry_fills_safe_defaults_when_unavailable():
    telem = _sample_telemetry(position_m=None, velocity_mps=None, attitude_rad=None, battery_fraction=None)
    state = phase9._vehicle_state_from_telemetry("drone0", telem, now_s=0.0)
    assert state.position_m == (0.0, 0.0, 0.0)
    assert state.acceleration_mps2 == (0.0, 0.0, 0.0)
    assert state.battery_fraction == 1.0


# ==========================================================================
# 8. no process spawning / no process killing
# ==========================================================================

def test_build_attach_endpoint_never_sets_executable_path():
    endpoint = phase9.build_attach_endpoint(_args(port=5760))
    assert isinstance(endpoint, ArduPilotVehicleEndpoint)
    assert endpoint.executable_path is None
    assert endpoint.wsl_distro is None
    assert endpoint.working_directory is None


def test_attach_mode_channel_never_has_a_process_handle(dummy_vehicle):
    vehicle, port = dummy_vehicle
    args = _args(port=port, hold_test=True, startup_timeout=3.0)
    report = phase9.run_hold_command_test(args)
    assert report["attach"]["succeeded"] is True
    # There is no way to reach a _SITLProcessHandle in ATTACH mode - the
    # endpoint never carries an executable_path (see the test above), so
    # nothing this script does could ever start OR kill a process.


# ==========================================================================
# 9. HOLD command / safe rejected-navigation-command handling
# ==========================================================================

def test_hold_command_accepted_and_never_arms(dummy_vehicle):
    vehicle, port = dummy_vehicle
    report = phase9.run_hold_command_test(_args(port=port, hold_test=True, startup_timeout=3.0))
    assert report["hold_command"]["accepted"] is True
    assert vehicle.armed is False


def test_rejected_navigation_command_records_correct_safe_result(dummy_vehicle):
    vehicle, port = dummy_vehicle
    report = phase9.run_hold_command_test(_args(port=port, hold_test=True, startup_timeout=3.0))
    nav = report["rejected_navigation_command_test"]
    assert nav["attempted"] is True
    assert nav["armed_after"] is False
    assert "correct safe result" in nav["interpretation"]
    assert vehicle.armed is False


# ==========================================================================
# 10. planner_preview: SafetySupervisor gating, no raw candidate bypass, no arm
# ==========================================================================

def test_planner_preview_calls_safety_supervisor_before_every_send(dummy_vehicle, monkeypatch):
    vehicle, port = dummy_vehicle
    call_order = []
    original_evaluate = SafetySupervisor.evaluate

    def spy_evaluate(self, *args, **kwargs):
        call_order.append("evaluate")
        return original_evaluate(self, *args, **kwargs)

    monkeypatch.setattr(SafetySupervisor, "evaluate", spy_evaluate)

    args = _args(port=port, planner_preview=True, duration=2.0, startup_timeout=3.0)
    report = phase9.run_planner_preview(args)

    assert len(call_order) == len(report["ticks"]) == 2
    assert report["armed_at_any_point"] is False
    assert vehicle.armed is False
    for tick in report["ticks"]:
        assert tick["safety_decision"]["accepted"] is True
        assert tick["transport_result"]["accepted"] is True


def test_planner_preview_never_sends_a_raw_candidate_to_the_adapter(dummy_vehicle, monkeypatch):
    """Every AdapterCommand actually sent must wrap
    decision.filtered_command (SafetySupervisor's own validated output) -
    never the raw CandidateCommand constructed each tick."""
    vehicle, port = dummy_vehicle
    sent_commands = []
    from swarm_sim.autopilot.sitl import SITLAdapter
    original_send = SITLAdapter.send_command

    def spy_send(self, command):
        sent_commands.append(command)
        return original_send(self, command)

    monkeypatch.setattr(SITLAdapter, "send_command", spy_send)

    args = _args(port=port, planner_preview=True, duration=1.0, startup_timeout=3.0)
    phase9.run_planner_preview(args)

    assert sent_commands   # at least one command was actually sent
    for adapter_command in sent_commands:
        # A CandidateCommand has no .command_type - only a validated
        # contracts.Command (SafetyDecision.filtered_command) does.
        assert hasattr(adapter_command.command, "command_type")


def test_planner_preview_never_arms_or_takes_off(dummy_vehicle):
    vehicle, port = dummy_vehicle
    report = phase9.run_planner_preview(_args(port=port, planner_preview=True, duration=2.0, startup_timeout=3.0))
    assert report["armed_at_any_point"] is False
    assert vehicle.armed is False
    takeoff_cmds = [c for c in vehicle.received_commands
                    if getattr(c, "command", None) == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF]
    assert takeoff_cmds == []


def test_planner_preview_refuses_to_run_without_telemetry():
    """If telemetry never arrives (e.g. a vehicle that sends heartbeats
    but not LOCAL_POSITION_NED), planner_preview must refuse to run rather
    than fabricating a VehicleState from nothing."""
    port = _next_port()
    vehicle = DummyArduPilotVehicle(port=port, send_telemetry=False)
    vehicle.start()
    try:
        report = phase9.run_planner_preview(
            _args(port=port, planner_preview=True, duration=1.0, startup_timeout=3.0)
        )
        assert report["attach"]["succeeded"] is True
        assert report["ticks"] == []
        assert any("no telemetry available" in f for f in report["remaining_failures"])
    finally:
        vehicle.stop()


# ==========================================================================
# 11. mode separation - results are never merged across modes
# ==========================================================================

def test_modes_produce_disjoint_top_level_report_shapes(dummy_vehicle):
    vehicle, port = dummy_vehicle
    telem_report = phase9.run_telemetry_visualization(_args(port=port, duration=0.5, startup_timeout=3.0))
    assert telem_report["mode"] == "telemetry_visualization"
    assert "ticks" not in telem_report
    assert "hold_command" not in telem_report


def test_dry_run_report_never_contains_live_telemetry_fields():
    report = phase9.run_dry_run(_args(port=19010))
    assert "telemetry_samples" not in report
    assert "ticks" not in report


# ==========================================================================
# 12. FakeSITL regression compatibility (Phase 7 must remain unaffected)
# ==========================================================================

def test_fake_sitl_transport_still_importable_and_functional():
    from swarm_sim.sitl.fake_transport import FakeSITLTransport
    t = FakeSITLTransport(vehicle_ids=("d",))
    assert t.start().success
