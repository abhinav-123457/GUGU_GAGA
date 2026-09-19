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
