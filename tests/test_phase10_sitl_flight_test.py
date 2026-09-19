"""Phase 10 functional tests - see docs/PHASE10_SITL_FLIGHT_TEST.md and
scripts/run_phase10_sitl_flight_test.py. Architecture/AST checks live in
tests/test_phase10_sitl_flight_test_architecture.py.

`_FlightSimVehicle` extends `_dummy_ardupilot_vehicle.DummyArduPilotVehicle`
(the same real-MAVLink-over-a-real-localhost-socket fixture Phase 8/9's own
tests use) with just enough arm/takeoff/land/param/version simulation to
exercise Phase 10's own gating and sequencing logic - it lives ENTIRELY in
this file and never modifies the shared fixture, so Phase 7/8/9's own tests
(which rely on DummyArduPilotVehicle.armed staying False forever) are
completely unaffected.
"""
import itertools
import os
import sys
import time

import pytest
from pymavlink import mavutil

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"))

import run_phase10_sitl_flight_test as phase10  # noqa: E402

from _dummy_ardupilot_vehicle import DummyArduPilotVehicle  # noqa: E402

_port_counter = itertools.count(18900)


def _next_port() -> int:
    return next(_port_counter)


class _Namespace:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


def _args(**overrides):
    port = overrides.pop("port", 0)
    defaults = dict(
        connection=f"tcp:127.0.0.1:{port}", system_id=1, component_id=1, namespace="sitl/drone0",
        startup_timeout=3.0, duration=2.0, dry_run=False, attach=False, diagnose_prearm=False,
        sitl_only=True, allow_sitl_arm_test=True, confirm_sitl_flight_test=True,
        max_altitude=2.0, max_speed=0.5,
    )
    defaults.update(overrides)
    return _Namespace(**defaults)


class _FlightSimVehicle(DummyArduPilotVehicle):
    """See module docstring. `params` seeds PARAM_REQUEST_READ responses;
    `reject_arm`/`reject_takeoff` simulate ArduPilot's own real rejection
    (never bypassed by the code under test); altitude climbs/descends over
    real wall-clock time so Phase 10's own bounded-polling loops have
    something real to observe."""

    def __init__(self, *args, params=None, reject_arm=False, reject_takeoff=False, **kwargs):
        super().__init__(*args, **kwargs)
        self.params = dict(params or {})
        self.reject_arm = reject_arm
        self.reject_takeoff = reject_takeoff
        self._takeoff_target_alt = None
        self.arm_attempts = 0
        self.takeoff_attempts = 0
        self.land_attempts = 0
        self.received_param_sets = []

    def _handle_message(self, msg):
        msg_type = msg.get_type()
        if msg_type == "PARAM_SET":
            # Recorder only - deliberately never applies the write, so a
            # test can prove "no parameter write occurs in the project
            # code" (requirement 9) by asserting this list stays empty,
            # independent of the AST-based structural check in
            # test_phase10_sitl_flight_test_architecture.py.
            self.received_param_sets.append(msg)
            return
        if msg_type == "PARAM_REQUEST_READ":
            pid = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode()
            pid = pid.rstrip("\x00")
            if pid in self.params and self._conn.port is not None:
                self._conn.mav.param_value_send(
                    pid.encode(), float(self.params[pid]), mavutil.mavlink.MAV_PARAM_TYPE_REAL32, 1, 0,
                )
            return
        if msg_type == "COMMAND_LONG" and msg.command == mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
            self.received_commands.append(msg)
            if self._conn.port is not None:
                self._conn.mav.autopilot_version_send(0, 0x04060300, 0, 0, 0, (0,) * 8, (0,) * 8, (0,) * 18, 0, 0, 0)
                self._conn.mav.command_ack_send(msg.command, mavutil.mavlink.MAV_RESULT_ACCEPTED)
            return
        if msg_type == "COMMAND_LONG" and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            self.received_commands.append(msg)
            arm_requested = msg.param1 > 0.5
            if arm_requested:
                self.arm_attempts += 1
                if self.reject_arm:
                    if self._conn.port is not None:
                        self._conn.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_CRITICAL,
                                                          b"PreArm: Simulated rejection")
                        self._conn.mav.command_ack_send(msg.command, mavutil.mavlink.MAV_RESULT_FAILED)
                    return
            self.armed = arm_requested
            if self._conn.port is not None:
                self._conn.mav.command_ack_send(msg.command, mavutil.mavlink.MAV_RESULT_ACCEPTED)
            return
        if msg_type == "COMMAND_LONG" and msg.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
            self.received_commands.append(msg)
            self.takeoff_attempts += 1
            if self.armed and not self.reject_takeoff:
                self._takeoff_target_alt = msg.param7
                if self._conn.port is not None:
                    self._conn.mav.command_ack_send(msg.command, mavutil.mavlink.MAV_RESULT_ACCEPTED)
            elif self._conn.port is not None:
                self._conn.mav.command_ack_send(msg.command, mavutil.mavlink.MAV_RESULT_FAILED)
            return
        if msg_type == "COMMAND_LONG" and msg.command == mavutil.mavlink.MAV_CMD_NAV_LAND:
            self.received_commands.append(msg)
            self.land_attempts += 1
            self._takeoff_target_alt = 0.0
            if self._conn.port is not None:
                self._conn.mav.command_ack_send(msg.command, mavutil.mavlink.MAV_RESULT_ACCEPTED)
            return
        super()._handle_message(msg)

    def _send_telemetry(self):
        if self._takeoff_target_alt is not None:
            current = -self.position[2]
            target = self._takeoff_target_alt
            step = 0.6
            if current < target - 0.05:
                current = min(target, current + step)
            elif target == 0.0 and current > 0.0:
                current = max(0.0, current - step)
                if current <= 0.01:
                    self.armed = False
            self.position[2] = -current
        super()._send_telemetry()


@pytest.fixture
def flight_sim():
    port = _next_port()
    v = _FlightSimVehicle(port=port, system_id=1, component_id=1,
                          params={"BATT_MONITOR": 4.0, "FENCE_ENABLE": 1.0, "ARMING_CHECK": 1.0})
    v.start()
    yield v, port
    v.stop()


# ==========================================================================
# 1. dry-run
# ==========================================================================

def test_dry_run_reports_ok_and_shows_missing_gate_flags():
    report = phase10.run_dry_run(_args(port=19100, allow_sitl_arm_test=False, confirm_sitl_flight_test=False))
    assert report["ok"] is True   # endpoint itself is fine even though gate flags are missing
    assert "allow_sitl_arm_test_flag" in report["gate_reasons_failed"]


def test_dry_run_never_opens_a_socket():
    t0 = time.monotonic()
    report = phase10.run_dry_run(_args(port=19101))
    assert time.monotonic() - t0 < 1.0
    assert report["mode"] == "dry_run"


def test_dry_run_rejects_non_loopback_connection():
    args = _args(port=0)
    args.connection = "tcp:8.8.8.8:5760"
    report = phase10.run_dry_run(args)
    assert report["ok"] is False


# ==========================================================================
# 2. gate checks (Step 2's required list)
# ==========================================================================

@pytest.mark.parametrize("field", ["sitl_only", "allow_sitl_arm_test"])
def test_flight_test_refuses_without_each_required_gate_flag(flight_sim, field):
    vehicle, port = flight_sim
    report = phase10.run_sitl_flight_test(_args(port=port, **{field: False}))
    assert report["gate"]["passed"] is False
    assert report["arm"]["attempted"] is False
    assert vehicle.armed is False




def test_flight_test_refuses_wrong_namespace(flight_sim):
    vehicle, port = flight_sim
    report = phase10.run_sitl_flight_test(_args(port=port, namespace="sitl/some-other-drone"))
    assert report["gate"]["passed"] is False
    assert "namespace_is_sitl_drone0" in report["gate"]["reasons_failed"]
    assert vehicle.armed is False


@pytest.mark.parametrize("field,value", [("max_altitude", 50.0), ("max_speed", 5.0)])
def test_flight_test_refuses_limits_above_hard_cap(flight_sim, field, value):
    vehicle, port = flight_sim
    report = phase10.run_sitl_flight_test(_args(port=port, **{field: value}))
    assert report["gate"]["passed"] is False
    assert vehicle.armed is False


def test_flight_test_refuses_non_loopback_connection():
    args = _args(port=0)
    args.connection = "tcp:1.2.3.4:5760"
    report = phase10.run_sitl_flight_test(args)
    assert report["gate"]["passed"] is False
    assert "localhost_connection" in report["gate"]["reasons_failed"]


def test_flight_test_requires_confirmation_even_with_all_flags(flight_sim, monkeypatch):
    vehicle, port = flight_sim
    monkeypatch.setattr(sys.stdin, "isatty", lambda: False)
    report = phase10.run_sitl_flight_test(_args(port=port, confirm_sitl_flight_test=False))
    assert report["gate"]["passed"] is True   # gate itself passed...
    assert report["operator_confirmed"] is False   # ...but confirmation was never supplied
    assert report["attach"]["succeeded"] is False   # never even attached
    assert vehicle.armed is False


# ==========================================================================
# 3. wrong system/component id, geofence-disabled, battery gates
# ==========================================================================

def test_flight_test_stops_if_geofence_disabled():
    port = _next_port()
    v = _FlightSimVehicle(port=port, system_id=1, component_id=1,
                          params={"BATT_MONITOR": 4.0, "FENCE_ENABLE": 0.0, "ARMING_CHECK": 1.0})
    v.start()
    try:
        report = phase10.run_sitl_flight_test(_args(port=port))
        assert report["geofence_enabled"] is False
        assert report["geofence_gate"]["passed"] is False
        assert "fence_enabled" in report["geofence_gate"]["reasons_failed"]
        assert report["arm"]["attempted"] is False
        assert v.armed is False
    finally:
        v.stop()


def test_flight_test_geofence_gate_passes_with_bounded_fence_params(flight_sim):
    """flight_sim seeds FENCE_ENABLE=1.0 alongside a realistic bounded local
    fence (FENCE_TYPE/FENCE_ALT_MAX/FENCE_RADIUS matching
    docs/phase10_sitl_geofence.parm) - the gate only requires FENCE_ENABLE
    to be non-zero, and passing it must let the sequence proceed to arm."""
    vehicle, port = flight_sim
    args = _args(port=port, duration=1.0)
    # Simulate the operator having applied the documented bounded-fence
    # override file (FENCE_TYPE=7, FENCE_ALT_MAX=10, FENCE_RADIUS=10,
    # FENCE_MARGIN=2) in addition to FENCE_ENABLE=1 already seeded above.
    vehicle.params.update({"FENCE_TYPE": 7.0, "FENCE_ALT_MAX": 10.0, "FENCE_RADIUS": 10.0, "FENCE_MARGIN": 2.0})
    report = phase10.run_sitl_flight_test(args)
    assert report["geofence_gate"]["passed"] is True
    assert report["geofence_gate"]["checks"]["fence_enabled"]["passed"] is True
    assert report["arm"]["accepted"] is True
    assert report["remaining_failures"] == []


def test_flight_test_never_sends_param_set(flight_sim):
    """No parameter write occurs in the project code (requirement 9): a
    live, wire-level check (the vehicle records any PARAM_SET it receives -
    see _FlightSimVehicle._handle_message) that a full flight-test sequence
    never sends one, including ARMING_CHECK. Complements the AST-based
    structural check in test_phase10_sitl_flight_test_architecture.py
    (no `param_set_send` call exists anywhere in the module)."""
    vehicle, port = flight_sim
    report = phase10.run_sitl_flight_test(_args(port=port, duration=1.0))
    assert report["remaining_failures"] == []
    assert vehicle.received_param_sets == []
    assert vehicle.params["ARMING_CHECK"] == 1.0   # remains enabled - never touched


def test_prearm_diagnostics_never_sends_param_set():
    port = _next_port()
    v = _FlightSimVehicle(port=port, system_id=1, component_id=1,
                          params={"ARMING_CHECK": 1.0, "BATT_MONITOR": 4.0, "FENCE_ENABLE": 0.0})
    v.start()
    try:
        report = phase10.run_prearm_diagnostics(_args(port=port, startup_timeout=3.0))
        assert report["attach"]["succeeded"] is True
        assert v.received_param_sets == []
    finally:
        v.stop()


def test_flight_test_attach_fails_with_wrong_system_id():
    port = _next_port()
    v = _FlightSimVehicle(port=port, system_id=9, component_id=1)
    v.start()
    try:
        report = phase10.run_sitl_flight_test(_args(port=port, system_id=1, startup_timeout=1.0))
        assert report["attach"]["succeeded"] is False
        assert v.armed is False
    finally:
        v.stop()


# ==========================================================================
# 4. prearm rejection - never bypassed
# ==========================================================================

def test_flight_test_stops_on_rejected_arm_never_bypasses():
    port = _next_port()
    v = _FlightSimVehicle(port=port, system_id=1, component_id=1, reject_arm=True,
                          params={"BATT_MONITOR": 4.0, "FENCE_ENABLE": 1.0, "ARMING_CHECK": 1.0})
    v.start()
    try:
        report = phase10.run_sitl_flight_test(_args(port=port))
        assert report["arm"]["attempted"] is True
        assert report["arm"]["accepted"] is False
        assert v.armed is False
        assert v.arm_attempts == 1   # never retried
        assert "PreArm" in report["arm"]["statustexts"][0]
        assert report["flight_actually_happened"] is False
    finally:
        v.stop()


# ==========================================================================
# 5. happy path: arm -> takeoff -> hold -> land -> disarm
# ==========================================================================

def test_full_happy_path_flight_sequence(flight_sim):
    vehicle, port = flight_sim
    report = phase10.run_sitl_flight_test(_args(port=port, max_altitude=2.0, duration=1.0))
    assert report["remaining_failures"] == []
    assert report["arm"]["accepted"] is True
    assert report["arm"]["confirmed_by_heartbeat"] is True
    assert report["takeoff"]["accepted"] is True
    assert report["flight_actually_happened"] is True
    assert report["takeoff"]["max_altitude_observed_m"] > 0.3
    assert report["land"]["accepted"] is True
    assert report["disarm_confirmed"] is True
    assert vehicle.armed is False   # ends disarmed
    assert report["emergency_action"] is None
    assert report["version"]["confirmed"] is True


def test_happy_path_never_exceeds_configured_altitude_by_much(flight_sim):
    vehicle, port = flight_sim
    report = phase10.run_sitl_flight_test(_args(port=port, max_altitude=1.0, duration=1.0))
    assert report["takeoff"]["max_altitude_observed_m"] <= 1.5   # generous margin over the 1.0m target


# ==========================================================================
# 6. rejected takeoff after a successful arm -> emergency cleanup
# ==========================================================================

def test_rejected_takeoff_after_arm_triggers_emergency_disarm():
    port = _next_port()
    v = _FlightSimVehicle(port=port, system_id=1, component_id=1, reject_takeoff=True,
                          params={"BATT_MONITOR": 4.0, "FENCE_ENABLE": 1.0, "ARMING_CHECK": 1.0})
    v.start()
    try:
        report = phase10.run_sitl_flight_test(_args(port=port, duration=1.0))
        assert report["arm"]["accepted"] is True
        assert report["takeoff"]["accepted"] is False
        assert report["emergency_action"] is not None
        # emergency cleanup sends LAND then, if still armed, a plain disarm -
        # confirm it actually reached the vehicle.
        assert v.land_attempts >= 1
    finally:
        v.stop()


# ==========================================================================
# 7. prearm_diagnostics - read-only, never arms, never writes a parameter
# ==========================================================================

def test_prearm_diagnostics_reads_params_and_never_arms():
    port = _next_port()
    v = _FlightSimVehicle(port=port, system_id=1, component_id=1,
                          params={"ARMING_CHECK": 1.0, "INS_ACCOFFS_X": 0.0, "INS_ACCOFFS_Y": 0.0,
                                  "INS_ACCOFFS_Z": 0.0, "BATT_MONITOR": 0.0})
    v.start()
    try:
        report = phase10.run_prearm_diagnostics(_args(port=port, startup_timeout=3.0))
        assert report["attach"]["succeeded"] is True
        assert report["parameters"]["ARMING_CHECK"] == 1.0
        assert report["diagnosis"]["accel_offsets_are_zero"] is True
        assert report["diagnosis"]["arming_check_stayed_enabled"] is True
        assert report["battery"]["monitor_configured"] is False
        assert v.armed is False
        assert v.arm_attempts == 0
    finally:
        v.stop()


def test_prearm_diagnostics_reports_geofence_values_when_available():
    port = _next_port()
    v = _FlightSimVehicle(port=port, system_id=1, component_id=1,
                          params={"ARMING_CHECK": 1.0, "BATT_MONITOR": 4.0, "FENCE_ENABLE": 1.0,
                                  "FENCE_TYPE": 7.0, "FENCE_ALT_MAX": 10.0, "FENCE_RADIUS": 10.0,
                                  "FENCE_MARGIN": 2.0, "FENCE_ACTION": 1.0})
    v.start()
    try:
        report = phase10.run_prearm_diagnostics(_args(port=port, startup_timeout=3.0))
        assert report["geofence"] == {
            "FENCE_ENABLE": 1.0, "FENCE_TYPE": 7.0, "FENCE_ALT_MAX": 10.0,
            "FENCE_RADIUS": 10.0, "FENCE_MARGIN": 2.0, "FENCE_ACTION": 1.0,
        }
        assert v.armed is False
        assert v.received_param_sets == []   # reporting-only - still never writes
    finally:
        v.stop()


def test_prearm_diagnostics_reports_geofence_none_when_unavailable():
    """"when available" (requirement wording) - a param the vehicle never
    answers must show up as None, never fabricated or omitted."""
    port = _next_port()
    v = _FlightSimVehicle(port=port, system_id=1, component_id=1,
                          params={"ARMING_CHECK": 1.0, "FENCE_ENABLE": 0.0})
    v.start()
    try:
        report = phase10.run_prearm_diagnostics(_args(port=port, startup_timeout=3.0))
        assert report["geofence"]["FENCE_ENABLE"] == 0.0
        assert report["geofence"]["FENCE_TYPE"] is None
        assert report["geofence"]["FENCE_ALT_MAX"] is None
        assert report["geofence"]["FENCE_RADIUS"] is None
        assert report["geofence"]["FENCE_MARGIN"] is None
        assert report["geofence"]["FENCE_ACTION"] is None
    finally:
        v.stop()


def test_prearm_diagnostics_detects_calibrated_accel_offsets():
    port = _next_port()
    v = _FlightSimVehicle(port=port, system_id=1, component_id=1,
                          params={"INS_ACCOFFS_X": 0.001, "INS_ACCOFFS_Y": 0.001, "INS_ACCOFFS_Z": 0.001})
    v.start()
    try:
        report = phase10.run_prearm_diagnostics(_args(port=port, startup_timeout=3.0))
        assert report["diagnosis"]["accel_offsets_are_zero"] is False
    finally:
        v.stop()


# ==========================================================================
# 8. mode separation - telemetry_only delegates to Phase 9, unmodified
# ==========================================================================

def test_attach_mode_delegates_to_phase9_telemetry_visualization(flight_sim):
    vehicle, port = flight_sim
    args = _args(port=port, attach=True, duration=0.5, sitl_only=False, allow_sitl_arm_test=False,
                 confirm_sitl_flight_test=False)
    import run_phase9_single_vehicle_visual as phase9
    report = phase9.run_telemetry_visualization(args)
    assert report["mode"] == "telemetry_visualization"
    assert vehicle.armed is False


def test_modes_produce_disjoint_report_shapes(flight_sim):
    vehicle, port = flight_sim
    dry = phase10.run_dry_run(_args(port=port))
    diag = phase10.run_prearm_diagnostics(_args(port=port, startup_timeout=3.0))
    assert "arm" not in dry
    assert "arm" not in diag
    assert dry["mode"] != diag["mode"]


# ==========================================================================
# 9. FakeSITL regression compatibility
# ==========================================================================

def test_fake_sitl_transport_still_importable_and_functional():
    from swarm_sim.sitl.fake_transport import FakeSITLTransport
    t = FakeSITLTransport(vehicle_ids=("d",))
    assert t.start().success
