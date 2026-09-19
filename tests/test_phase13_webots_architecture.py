"""Architecture checks for scripts/run_phase13_webots_flight_test.py - see
docs/PHASE13_WEBOTS_FLIGHT_TEST.md. AST-based structural proof mirroring
Phase 10/11/12's own patterns.
"""
import ast
import os
import pathlib
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"))

import run_phase13_webots_flight_test as phase13  # noqa: E402

FORBIDDEN_HARDWARE_IMPORTS = {
    "serial", "pyserial", "RPi", "gpiozero", "smbus", "smbus2", "spidev", "dronekit",
}
FORBIDDEN_PROCESS_IDENTIFIERS = {
    "Popen", "check_call", "check_output", "terminate", "SIGKILL", "SIGTERM", "_SITLProcessHandle",
}


def _tree() -> ast.AST:
    return ast.parse(pathlib.Path(phase13.__file__).read_text())


def _collect_identifiers(tree_or_node):
    names = set()
    for node in ast.walk(tree_or_node):
        if isinstance(node, ast.Name):
            names.add(node.id)
        elif isinstance(node, ast.Attribute):
            names.add(node.attr)
        elif isinstance(node, ast.arg):
            names.add(node.arg)
    return names


def _collect_imports(tree_or_node):
    imported = set()
    for node in ast.walk(tree_or_node):
        if isinstance(node, ast.Import):
            imported.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module.split(".")[0])
    return imported


def _fn(name: str):
    return next(n for n in ast.walk(_tree()) if isinstance(n, ast.FunctionDef) and n.name == name)


# --- no hardware / process spawning / process killing -----------------------

def test_no_hardware_specific_imports():
    imported = _collect_imports(_tree())
    assert not (imported & FORBIDDEN_HARDWARE_IMPORTS)


def test_no_subprocess_module_imported():
    assert "subprocess" not in _collect_imports(_tree())


def test_no_process_lifecycle_identifiers_referenced():
    identifiers = _collect_identifiers(_tree())
    leaked = identifiers & FORBIDDEN_PROCESS_IDENTIFIERS
    assert not leaked, f"references forbidden process-lifecycle identifiers: {leaked}"


def test_never_kills_webots_or_ardupilot():
    source = pathlib.Path(phase13.__file__).read_text()
    for forbidden in ("os.system(", "os.exec", ".kill(", "os.kill("):
        assert forbidden not in source


# --- no parameter writes -----------------------------------------------------

def test_never_calls_param_set_anywhere():
    identifiers = _collect_identifiers(_tree())
    assert "param_set_send" not in identifiers
    assert "mav_param_set_send" not in identifiers


def test_never_forces_arm_with_the_force_magic_value():
    for node in ast.walk(_tree()):
        if isinstance(node, ast.Constant) and node.value == 21196:
            raise AssertionError("force-arm/disarm magic value (21196) must never appear")


# --- default mode never arms; three explicit flags required -----------------

def test_default_cli_args_never_select_a_live_flight_test():
    args = phase13.build_parser().parse_args([])
    assert args.webots_only is False
    assert args.allow_webots_arm_test is False
    assert args.confirm_webots_flight_test is False
    assert args.dry_run is False


def test_main_dispatch_falls_back_to_dry_run():
    source = pathlib.Path(phase13.__file__).read_text()
    main_fn = _fn("main")
    main_source = ast.get_source_segment(source, main_fn)
    assert "run_dry_run(args)" in main_source


def test_main_requires_all_three_flags_together():
    source = pathlib.Path(phase13.__file__).read_text()
    main_fn = _fn("main")
    main_source = ast.get_source_segment(source, main_fn)
    assert "args.webots_only and args.allow_webots_arm_test and args.confirm_webots_flight_test" in main_source


def test_flight_test_gate_checks_all_three_explicit_flags():
    source = pathlib.Path(phase13.__file__).read_text()
    gate_fn = _fn("_check_flight_test_gate")
    gate_source = ast.get_source_segment(source, gate_fn)
    assert "webots_only" in gate_source
    assert "allow_webots_arm_test" in gate_source


# --- one vehicle only / no six-vehicle configuration -------------------------

def test_no_multi_vehicle_endpoint_dict_construction():
    for node in ast.walk(_tree()):
        if isinstance(node, ast.Call):
            fn = node.func
            if isinstance(fn, ast.Name) and fn.id == "ArduPilotSITLTransport" and node.args:
                first_arg = node.args[0]
                assert isinstance(first_arg, ast.Dict) and len(first_arg.keys) == 1


def test_no_num_drones_or_six_vehicle_configuration():
    identifiers = _collect_identifiers(_tree())
    assert "num_drones" not in identifiers
    assert "VEHICLE_IDS" not in identifiers


# --- no swarm planner / consensus / FlightGear / PyBullet / raw candidate ---

def test_no_swarm_or_consensus_or_planner_references():
    tree = _tree()
    identifiers = _collect_identifiers(tree)
    imports = _collect_imports(tree)
    assert "distributed_consensus" not in imports
    assert "pybullet" not in imports
    assert not (identifiers & {
        "FloodSearchMission", "DistributedConsensus", "DroneConsensusNode",
        "CandidateCommand", "SafetySupervisor",
    })


def test_no_flightgear_import():
    source = pathlib.Path(phase13.__file__).read_text()
    assert "flightgear_bridge" not in source
    assert "telemetry_mapping" not in source
    assert "FlightGearBridge" not in source


# --- no hardcoded non-local default / hardware endpoint ----------------------

def test_default_connection_is_loopback():
    args = phase13.build_parser().parse_args([])
    assert args.connection.split(":")[1] in ("127.0.0.1", "localhost")


def test_no_hardcoded_non_loopback_ip_literal():
    """"0.0.0.0" is expected - it's the local bind-test wildcard address
    _port_appears_bound uses (reused from Phase 12), never a remote target."""
    import ipaddress
    ip_pattern = re.compile(r"^\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}$")
    for node in ast.walk(_tree()):
        if isinstance(node, ast.Constant) and isinstance(node.value, str) and ip_pattern.match(node.value):
            ip = ipaddress.ip_address(node.value)
            assert ip.is_loopback or ip.is_private or ip.is_unspecified, (
                f"unexpected non-local hardcoded IP literal: {node.value!r}"
            )


def test_no_serial_or_radio_endpoint_default():
    args = phase13.build_parser().parse_args([])
    assert "COM" not in args.connection  # no Windows serial-port-style default
    assert "/dev/" not in args.connection


# --- conservative hard caps ---------------------------------------------------

def test_hard_caps_are_conservative():
    assert phase13.HARD_MAX_ALTITUDE_M <= 2.0
    assert phase13.HARD_MAX_SPEED_MPS <= 0.25
    assert phase13.HARD_MAX_DURATION_S <= 30.0


def test_default_altitude_and_duration_within_conservative_targets():
    args = phase13.build_parser().parse_args([])
    assert args.max_altitude <= 1.0
    assert args.duration <= 10.0


# --- timeout -> emergency LAND; landing/disarm confirmation present ----------

def test_climb_timeout_triggers_emergency_cleanup():
    source = pathlib.Path(phase13.__file__).read_text()
    flight_fn = _fn("run_webots_flight_test")
    flight_source = ast.get_source_segment(source, flight_fn)
    assert "emergency_cleanup(\"no observed climb after takeoff accepted\")" in flight_source


def test_emergency_cleanup_sends_land_then_disarm():
    source = pathlib.Path(phase13.__file__).read_text()
    assert "MAV_CMD_NAV_LAND" in source
    assert "MAV_CMD_COMPONENT_ARM_DISARM" in source


def test_report_includes_disarm_and_touchdown_confirmation_fields():
    source = pathlib.Path(phase13.__file__).read_text()
    assert '"disarm_confirmed"' in source
    assert '"touchdown_time_s"' in source
    assert '"disarm_time_s"' in source


# --- preserves Phase 10 / 11 / 12 ----------------------------------------------

def test_phase10_still_never_arms_in_dry_run_or_diagnostics():
    import run_phase10_sitl_flight_test as phase10_module
    tree = ast.parse(pathlib.Path(phase10_module.__file__).read_text())
    forbidden = {"MAV_CMD_COMPONENT_ARM_DISARM", "MAV_CMD_NAV_TAKEOFF"}
    for fn_name in ("run_dry_run", "run_prearm_diagnostics"):
        fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == fn_name)
        assert not (_collect_identifiers(fn) & forbidden)


def test_phase11_flightgear_viewer_unaffected():
    import run_phase11_flightgear_viewer as phase11_module
    tree = ast.parse(pathlib.Path(phase11_module.__file__).read_text())
    forbidden = {"MAV_CMD_COMPONENT_ARM_DISARM", "MAV_CMD_NAV_TAKEOFF", "MAV_CMD_NAV_LAND"}
    assert not (_collect_identifiers(tree) & forbidden)


def test_phase12_smoke_test_unaffected():
    import run_phase12_webots_smoke_test as phase12_module
    tree = ast.parse(pathlib.Path(phase12_module.__file__).read_text())
    forbidden = {"MAV_CMD_COMPONENT_ARM_DISARM", "MAV_CMD_NAV_TAKEOFF", "MAV_CMD_NAV_LAND", "command_long_send"}
    assert not (_collect_identifiers(tree) & forbidden)


def test_fake_sitl_transport_still_importable_and_functional():
    from swarm_sim.sitl.fake_transport import FakeSITLTransport
    t = FakeSITLTransport(vehicle_ids=("d",))
    assert t.start().success


def test_ardupilot_transport_module_still_never_references_arm_or_takeoff():
    import swarm_sim.sitl.ardupilot_transport as transport_module
    tree = ast.parse(pathlib.Path(transport_module.__file__).read_text())
    identifiers = _collect_identifiers(tree)
    assert "MAV_CMD_COMPONENT_ARM_DISARM" not in identifiers
    assert "MAV_CMD_NAV_TAKEOFF" not in identifiers


def test_reuses_phase10_and_phase12_helpers_instead_of_duplicating():
    source = pathlib.Path(phase13.__file__).read_text()
    assert "from run_phase10_sitl_flight_test import" in source
    assert "from run_phase12_webots_smoke_test import" in source
