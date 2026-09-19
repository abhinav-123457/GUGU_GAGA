"""Phase 13 functional tests - see docs/PHASE13_WEBOTS_FLIGHT_TEST.md and
scripts/run_phase13_webots_flight_test.py. Architecture/AST checks live in
tests/test_phase13_webots_architecture.py.

Reuses `_FlightSimVehicle` from tests/test_phase10_sitl_flight_test.py
directly (same real-MAVLink-over-a-real-localhost-socket arm/takeoff/land
simulation Phase 10's own tests use) - never redefined here, so Phase 10's
own tests and this file always exercise identical vehicle behavior. The
three Webots-specific, local-system checks (`_detect_webots_installation`,
`_official_example_paths`, `_port_appears_bound`) are monkeypatched to
report "everything present" for these tests, since no real Webots process
is running in the test environment - this is exactly the same pattern
tests/test_phase12_webots_config.py already uses for the same functions.
"""
import itertools
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"))

import run_phase13_webots_flight_test as phase13  # noqa: E402

from test_phase10_sitl_flight_test import _FlightSimVehicle  # noqa: E402

_port_counter = itertools.count(19900)


def _next_port() -> int:
    return next(_port_counter)


class _Namespace:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


def _args(**overrides):
    port = overrides.pop("port", 0)
    defaults = dict(
        connection=f"tcp:127.0.0.1:{port}", system_id=1, component_id=1, namespace="sitl/drone0",
        startup_timeout=3.0, instance=0, ardupilot_root="/fake/ardupilot", duration=2.0,
        dry_run=False, attach=False, diagnose_prearm=False,
        webots_only=True, allow_webots_arm_test=True, confirm_webots_flight_test=True,
        max_altitude=1.0, max_speed=0.25,
    )
    defaults.update(overrides)
    return _Namespace(**defaults)


@pytest.fixture(autouse=True)
def _webots_present(monkeypatch):
    """All Phase 13 tests run as if Webots + the official example + a bound
    controller port are present - see module docstring."""
    monkeypatch.setattr(phase13, "_detect_webots_installation",
                         lambda: {"found": True, "path": "/usr/local/bin/webots", "method": "PATH"})
    monkeypatch.setattr(phase13, "_official_example_paths",
                         lambda root, world, vehicle: {
                             "ardupilot_root": root, "world_path": f"{root}/worlds/{world}",
                             "params_path": f"{root}/params/{vehicle}.parm",
                             "controller_path": f"{root}/controllers/ardupilot_vehicle_controller.py",
                             "world_exists": True, "params_exists": True, "controller_exists": True,
                         })
    monkeypatch.setattr(phase13, "_port_appears_bound", lambda host, port: True)


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
    report = phase13.run_dry_run(_args(port=19100, webots_only=False, allow_webots_arm_test=False,
                                        confirm_webots_flight_test=False))
    assert report["ok"] is True
    assert "webots_only_flag" in report["gate_reasons_failed"]
    assert "allow_webots_arm_test_flag" in report["gate_reasons_failed"]


def test_dry_run_never_opens_a_socket():
    import time
    t0 = time.monotonic()
    report = phase13.run_dry_run(_args(port=19101))
    assert time.monotonic() - t0 < 1.0
    assert report["mode"] == "dry_run"


def test_dry_run_rejects_non_loopback_connection():
    args = _args(port=0)
    args.connection = "tcp:8.8.8.8:5760"
    report = phase13.run_dry_run(args)
    assert report["ok"] is False


def test_dry_run_flags_missing_webots_and_ardupilot_root(monkeypatch):
    monkeypatch.setattr(phase13, "_detect_webots_installation",
                         lambda: {"found": False, "path": None, "method": "not_found"})
    report = phase13.run_dry_run(_args(port=19102, ardupilot_root=None))
    assert "webots_installed" in report["gate_reasons_failed"]
    assert "ardupilot_root_given" in report["gate_reasons_failed"]
    assert "official_world_exists" in report["gate_reasons_failed"]


# ==========================================================================
# 2. gate checks
# ==========================================================================

@pytest.mark.parametrize("field", ["webots_only", "allow_webots_arm_test"])
def test_flight_test_refuses_without_each_required_gate_flag(flight_sim, field):
    vehicle, port = flight_sim
    report = phase13.run_webots_flight_test(_args(port=port, **{field: False}))
    assert report["gate"]["passed"] is False
    assert report["arm"]["attempted"] is False
    assert vehicle.armed is False


def test_flight_test_refuses_wrong_namespace(flight_sim):
    vehicle, port = flight_sim
    report = phase13.run_webots_flight_test(_args(port=port, namespace="sitl/some-other-drone"))
    assert report["gate"]["passed"] is False
    assert "namespace_is_sitl_drone0" in report["gate"]["reasons_failed"]
    assert vehicle.armed is False


@pytest.mark.parametrize("field,value", [("max_altitude", 5.0), ("max_speed", 3.0), ("duration", 60.0)])
def test_flight_test_refuses_limits_above_hard_cap(flight_sim, field, value):
    vehicle, port = flight_sim
    report = phase13.run_webots_flight_test(_args(port=port, **{field: value}))
    assert report["gate"]["passed"] is False
    assert vehicle.armed is False


def test_flight_test_refuses_non_loopback_connection():
    args = _args(port=0)
    args.connection = "tcp:1.2.3.4:5760"
    report = phase13.run_webots_flight_test(args)
    assert report["gate"]["passed"] is False
    assert "localhost_connection" in report["gate"]["reasons_failed"]


def test_flight_test_refuses_without_webots_installed(flight_sim, monkeypatch):
    vehicle, port = flight_sim
    monkeypatch.setattr(phase13, "_detect_webots_installation",
                         lambda: {"found": False, "path": None, "method": "not_found"})
    report = phase13.run_webots_flight_test(_args(port=port))
    assert report["gate"]["passed"] is False
    assert "webots_installed" in report["gate"]["reasons_failed"]
    assert vehicle.armed is False


def test_flight_test_refuses_without_ardupilot_root(flight_sim):
    vehicle, port = flight_sim
    report = phase13.run_webots_flight_test(_args(port=port, ardupilot_root=None))
    assert report["gate"]["passed"] is False
    assert "ardupilot_root_given" in report["gate"]["reasons_failed"]
    assert vehicle.armed is False


def test_flight_test_refuses_without_controller_port_bound(flight_sim, monkeypatch):
    vehicle, port = flight_sim
    monkeypatch.setattr(phase13, "_port_appears_bound", lambda host, port: False)
    report = phase13.run_webots_flight_test(_args(port=port))
    assert report["gate"]["passed"] is False
    assert "webots_controller_port_bound" in report["gate"]["reasons_failed"]
    assert vehicle.armed is False


def test_flight_test_requires_confirmation_even_with_all_flags(flight_sim, monkeypatch):
    vehicle, port = flight_sim
    monkeypatch.setattr(sys.stdin, "isatty", lambda: False)
    report = phase13.run_webots_flight_test(_args(port=port, confirm_webots_flight_test=False))
    assert report["gate"]["passed"] is True
    assert report["operator_confirmed"] is False
    assert report["attach"]["succeeded"] is False
    assert vehicle.armed is False


# ==========================================================================
# 3. ARMING_CHECK / geofence live gates
# ==========================================================================

def test_flight_test_stops_if_arming_check_disabled():
    port = _next_port()
    v = _FlightSimVehicle(port=port, system_id=1, component_id=1,
                          params={"BATT_MONITOR": 4.0, "FENCE_ENABLE": 1.0, "ARMING_CHECK": 0.0})
    v.start()
    try:
        report = phase13.run_webots_flight_test(_args(port=port))
        assert report["arming_check_gate"]["passed"] is False
        assert report["arm"]["attempted"] is False
        assert v.armed is False
    finally:
        v.stop()


def test_flight_test_stops_if_geofence_disabled():
    port = _next_port()
    v = _FlightSimVehicle(port=port, system_id=1, component_id=1,
                          params={"BATT_MONITOR": 4.0, "FENCE_ENABLE": 0.0, "ARMING_CHECK": 1.0})
    v.start()
    try:
        report = phase13.run_webots_flight_test(_args(port=port))
        assert report["geofence_enabled"] is False
        assert report["arm"]["attempted"] is False
        assert v.armed is False
    finally:
        v.stop()


def test_flight_test_never_sends_param_set(flight_sim):
    vehicle, port = flight_sim
    report = phase13.run_webots_flight_test(_args(port=port, duration=1.0))
    assert report["remaining_failures"] == []
    assert vehicle.received_param_sets == []
    assert vehicle.params["ARMING_CHECK"] == 1.0


# ==========================================================================
# 4. rejected arm / rejected takeoff -> emergency cleanup, never bypassed
# ==========================================================================

def test_flight_test_stops_on_rejected_arm_never_bypasses():
    port = _next_port()
    v = _FlightSimVehicle(port=port, system_id=1, component_id=1, reject_arm=True,
                          params={"BATT_MONITOR": 4.0, "FENCE_ENABLE": 1.0, "ARMING_CHECK": 1.0})
    v.start()
    try:
        report = phase13.run_webots_flight_test(_args(port=port))
        assert report["arm"]["attempted"] is True
        assert report["arm"]["accepted"] is False
        assert v.armed is False
        assert v.arm_attempts == 1
        assert report["flight_actually_happened"] is False
    finally:
        v.stop()


def test_rejected_takeoff_after_arm_triggers_emergency_land():
    port = _next_port()
    v = _FlightSimVehicle(port=port, system_id=1, component_id=1, reject_takeoff=True,
                          params={"BATT_MONITOR": 4.0, "FENCE_ENABLE": 1.0, "ARMING_CHECK": 1.0})
    v.start()
    try:
        report = phase13.run_webots_flight_test(_args(port=port, duration=1.0))
        assert report["arm"]["accepted"] is True
        assert report["takeoff"]["accepted"] is False
        assert report["emergency_action"] is not None
        assert v.land_attempts >= 1
    finally:
        v.stop()


# ==========================================================================
# 5. happy path: arm -> takeoff -> hold -> land -> disarm
# ==========================================================================

def test_full_happy_path_flight_sequence(flight_sim):
    vehicle, port = flight_sim
    report = phase13.run_webots_flight_test(_args(port=port, max_altitude=1.0, duration=1.0))
    assert report["remaining_failures"] == []
    assert report["arm"]["accepted"] is True
    assert report["arm"]["confirmed_by_heartbeat"] is True
    assert report["takeoff"]["accepted"] is True
    assert report["flight_actually_happened"] is True
    assert report["takeoff"]["max_altitude_observed_m"] > 0.3
    assert report["land"]["accepted"] is True
    assert report["disarm_confirmed"] is True
    assert vehicle.armed is False
    assert report["emergency_action"] is None
    assert report["version"]["confirmed"] is True
    assert report["disarm_time_s"] is not None


def test_happy_path_never_exceeds_configured_altitude_by_much(flight_sim):
    vehicle, port = flight_sim
    report = phase13.run_webots_flight_test(_args(port=port, max_altitude=1.0, duration=1.0))
    assert report["takeoff"]["max_altitude_observed_m"] <= 1.5


def test_happy_path_writes_csv_log(flight_sim, tmp_path, monkeypatch):
    vehicle, port = flight_sim
    csv_path = str(tmp_path / "flight_log.csv")
    monkeypatch.setattr(phase13, "CSV_PATH", csv_path)
    report = phase13.run_webots_flight_test(_args(port=port, duration=1.0))
    assert report["remaining_failures"] == []
    assert os.path.isfile(csv_path)
    import csv as csv_module
    with open(csv_path, newline="") as f:
        rows = list(csv_module.DictReader(f))
    assert len(rows) > 5
    assert any(row["event"] == "hold" for row in rows)
    assert any(row["armed"] == "True" for row in rows)


def test_happy_path_reports_collision_and_realtime_factor_honestly(flight_sim):
    vehicle, port = flight_sim
    report = phase13.run_webots_flight_test(_args(port=port, duration=1.0))
    assert "not directly observable" in report["collision_or_contact"]
    assert report["realtime_factor"] is None


# ==========================================================================
# 6. reliable actuator-output (SERVO_OUTPUT_RAW) logging
# ==========================================================================
#
# `_FlightSimVehicle` extends `DummyArduPilotVehicle` (see
# tests/_dummy_ardupilot_vehicle.py), which only sends SERVO_OUTPUT_RAW when
# constructed with `send_servo_output=True` (opt-in, default False - every
# other test above/below, and every Phase 7-12 test, is unaffected). This
# directly exercises the real fix: ArduPilotSITLTransport's own
# `_poll_incoming` now caches SERVO_OUTPUT_RAW the same way it already
# caches HEARTBEAT/ATTITUDE/etc, instead of this script racing a second,
# competing `recv_match()` call against it (the bug that caused the real
# live run's own actuator log to come back empty - see
# docs/PHASE13_WEBOTS_FLIGHT_TEST.md).

@pytest.fixture
def flight_sim_with_servo():
    port = _next_port()
    v = _FlightSimVehicle(port=port, system_id=1, component_id=1,
                          params={"BATT_MONITOR": 4.0, "FENCE_ENABLE": 1.0, "ARMING_CHECK": 1.0},
                          send_servo_output=True)
    v.servo_outputs = [1200, 1200, 1200, 1200]
    v.start()
    yield v, port
    v.stop()


def test_actuator_evidence_captured_during_real_flight(flight_sim_with_servo, tmp_path, monkeypatch):
    vehicle, port = flight_sim_with_servo
    actuator_path = str(tmp_path / "actuator_log.csv")
    monkeypatch.setattr(phase13, "ACTUATOR_CSV_PATH", actuator_path)
    report = phase13.run_webots_flight_test(_args(port=port, max_altitude=1.0, duration=1.0))
    assert report["flight_actually_happened"] is True
    evidence = report["actuator_evidence"]
    assert evidence["messages_received"] > 0
    assert evidence["rows_written"] > 0
    assert evidence["last_values"] == {
        "servo1_raw": 1200, "servo2_raw": 1200, "servo3_raw": 1200, "servo4_raw": 1200,
    }
    assert "received and" in evidence["note"]
    import csv as csv_module
    with open(actuator_path, newline="") as f:
        rows = list(csv_module.DictReader(f))
    assert len(rows) == evidence["rows_written"]
    assert all(row["servo1_raw"] == "1200" for row in rows)


def test_actuator_evidence_reports_gap_honestly_when_no_servo_messages(flight_sim):
    """`flight_sim` (no `send_servo_output`) never sends SERVO_OUTPUT_RAW -
    this must be reported as a real, honest gap, never fabricated or
    silently left implying success."""
    vehicle, port = flight_sim
    report = phase13.run_webots_flight_test(_args(port=port, max_altitude=1.0, duration=1.0))
    assert report["flight_actually_happened"] is True
    evidence = report["actuator_evidence"]
    assert evidence["messages_received"] == 0
    assert evidence["rows_written"] == 0
    assert evidence["last_values"] is None
    assert "no SERVO_OUTPUT_RAW messages were received" in evidence["note"]


def test_actuator_evidence_present_even_on_early_gate_failure():
    """Honest reporting should not depend on how far the sequence got -
    even a flight that stops at a gate before arming still reports
    (zeroed) actuator evidence, not a missing/None field."""
    report = phase13.run_webots_flight_test(_args(port=0, webots_only=False))
    assert report["actuator_evidence"] is None  # never even attached - nothing to report yet


def test_actuator_capture_never_blocks_the_control_loop(flight_sim):
    """`flight_sim` never sends SERVO_OUTPUT_RAW - proves the flight
    sequence completes in roughly the expected wall-clock time (hold
    duration plus a small overhead) instead of hanging while waiting for
    an actuator message that will never arrive."""
    import time
    vehicle, port = flight_sim
    t0 = time.monotonic()
    report = phase13.run_webots_flight_test(_args(port=port, max_altitude=1.0, duration=1.0))
    elapsed = time.monotonic() - t0
    assert report["flight_actually_happened"] is True
    assert elapsed < 20.0  # generous bound - a real block/hang would run into a multi-second timeout instead


def test_actuator_logger_writes_row_only_on_new_message(tmp_path):
    path = str(tmp_path / "actuator.csv")
    logger = phase13._ActuatorLogger(path)
    try:
        channel = _Namespace(servo_output_messages_received=0, last_servo_output_raw=None)
        telem = _Namespace(timestamp_s=1.0, armed=False, flight_mode=None, position_m=(0.0, 0.0, 0.5))

        assert logger.poll(channel, "tick", telem) is False  # no message yet
        assert logger.rows_written == 0

        channel.servo_output_messages_received = 1
        channel.last_servo_output_raw = (1100, 1100, 1100, 1100)
        assert logger.poll(channel, "tick", telem) is True  # new message
        assert logger.rows_written == 1

        assert logger.poll(channel, "tick", telem) is False  # same count - no new message
        assert logger.rows_written == 1

        channel.servo_output_messages_received = 2
        channel.last_servo_output_raw = (1300, 1300, 1300, 1300)
        assert logger.poll(channel, "tick", telem) is True
        assert logger.rows_written == 2
        assert logger.polls == 4
    finally:
        logger.close()


def test_actuator_logger_handles_zero_messages_honestly(tmp_path):
    path = str(tmp_path / "actuator.csv")
    logger = phase13._ActuatorLogger(path)
    try:
        channel = _Namespace(servo_output_messages_received=0, last_servo_output_raw=None)
        telem = _Namespace(timestamp_s=1.0, armed=False, flight_mode=None, position_m=None)
        for _ in range(5):
            assert logger.poll(channel, "tick", telem) is False
        assert logger.rows_written == 0
        assert logger.polls == 5
    finally:
        logger.close()


def test_actuator_logger_never_blocks_with_no_channel(tmp_path):
    """`channel=None` (the state before attach succeeds) must be handled
    without raising or blocking - not a state this class should ever
    need to wait on."""
    path = str(tmp_path / "actuator.csv")
    logger = phase13._ActuatorLogger(path)
    try:
        assert logger.poll(None, "tick", None) is False
    finally:
        logger.close()


# ==========================================================================
# 7. explicit altitude-result classification
# ==========================================================================

def test_classify_altitude_result_flags_real_overshoot_not_exact():
    """The exact real-run numbers this feature was built to report
    honestly - never claim the 2.0m target was hit exactly when the
    measured peak was 2.11m."""
    result = phase13._classify_altitude_result(
        target_altitude_m=2.0, measured_peak_altitude_m=2.11,
        hold_altitudes_m=[2.05, 2.04, 2.05], hard_ceiling_m=2.0,
    )
    assert result["target_altitude_m"] == 2.0
    assert result["measured_peak_altitude_m"] == 2.11
    assert result["overshoot_m"] == pytest.approx(0.11)
    assert result["overshoot_pct"] == pytest.approx(5.5, abs=0.01)
    assert result["within_hard_ceiling"] is False
    assert result["pass_fail"] == "fail"
    assert "exact" not in result["reason"]
    assert "2.110" in result["reason"] or "2.11" in result["reason"]
    assert "2.000" in result["reason"] or "2.0" in result["reason"]


def test_classify_altitude_result_hold_band_from_hold_samples_only():
    result = phase13._classify_altitude_result(
        target_altitude_m=1.0, measured_peak_altitude_m=1.05,
        hold_altitudes_m=[0.98, 1.02, None, 1.05, 0.99], hard_ceiling_m=2.0,
    )
    assert result["hold_band_m"] == {"min_m": 0.98, "max_m": 1.05, "sample_count": 4}


def test_classify_altitude_result_passes_when_within_tolerance_and_ceiling():
    result = phase13._classify_altitude_result(
        target_altitude_m=1.0, measured_peak_altitude_m=1.05,
        hold_altitudes_m=[1.0, 1.02], hard_ceiling_m=2.0,
    )
    assert result["pass_fail"] == "pass"
    assert result["within_hard_ceiling"] is True
    assert result["classification"] == "within_tolerance"


def test_classify_altitude_result_flags_undershoot():
    result = phase13._classify_altitude_result(
        target_altitude_m=1.0, measured_peak_altitude_m=0.2,
        hold_altitudes_m=[0.15, 0.2], hard_ceiling_m=2.0,
    )
    assert result["overshoot_m"] == pytest.approx(-0.8)
    assert result["classification"] == "undershoot_exceeds_tolerance"
    assert result["pass_fail"] == "fail"


def test_classify_altitude_result_flags_exceeding_hard_ceiling_even_if_close_to_target():
    result = phase13._classify_altitude_result(
        target_altitude_m=2.0, measured_peak_altitude_m=2.05,
        hold_altitudes_m=[2.0, 2.02], hard_ceiling_m=2.0,
    )
    assert result["within_hard_ceiling"] is False
    assert result["pass_fail"] == "fail"
    assert "hard safety ceiling" in result["reason"]


def test_full_happy_path_report_includes_altitude_result(flight_sim):
    vehicle, port = flight_sim
    report = phase13.run_webots_flight_test(_args(port=port, max_altitude=1.0, duration=1.0))
    altitude_result = report["altitude_result"]
    assert altitude_result["target_altitude_m"] == 1.0
    assert altitude_result["measured_peak_altitude_m"] == report["takeoff"]["max_altitude_observed_m"]
    assert altitude_result["hold_band_m"]["sample_count"] > 0
    assert altitude_result["pass_fail"] in ("pass", "fail")


def test_already_armed_refuses_to_proceed():
    port = _next_port()
    v = _FlightSimVehicle(port=port, system_id=1, component_id=1,
                          params={"BATT_MONITOR": 4.0, "FENCE_ENABLE": 1.0, "ARMING_CHECK": 1.0})
    v.start()
    v.armed = True
    try:
        report = phase13.run_webots_flight_test(_args(port=port))
        assert report["initially_disarmed"] is False
        assert report["arm"]["attempted"] is False
    finally:
        v.stop()


# ==========================================================================
# 6. prearm_diagnostics - read-only, never arms
# ==========================================================================

def test_prearm_diagnostics_reads_params_and_never_arms():
    port = _next_port()
    v = _FlightSimVehicle(port=port, system_id=1, component_id=1,
                          params={"ARMING_CHECK": 1.0, "BATT_MONITOR": 4.0, "FENCE_ENABLE": 1.0})
    v.start()
    try:
        report = phase13.run_prearm_diagnostics(_args(port=port, startup_timeout=3.0))
        assert report["attach"]["succeeded"] is True
        assert report["parameters"]["ARMING_CHECK"] == 1.0
        assert report["webots_installation"]["found"] is True
        assert report["webots_controller_port_bound"] is True
        assert v.armed is False
        assert v.arm_attempts == 0
    finally:
        v.stop()
