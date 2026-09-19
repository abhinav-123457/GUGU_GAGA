"""Architecture checks for scripts/run_phase12_webots_smoke_test.py - see
docs/PHASE12_WEBOTS_SITL.md. AST-based structural proof that the Phase 12
smoke test never arms/takes off/lands/writes a parameter, never spawns or
kills Webots/ArduPilot, never touches the six-drone swarm/consensus/
planner internals, and preserves every earlier phase's own guarantees.
"""
import ast
import os
import pathlib
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"))

import run_phase12_webots_smoke_test as phase12_module  # noqa: E402

FORBIDDEN_HARDWARE_IMPORTS = {
    "serial", "pyserial", "RPi", "gpiozero", "smbus", "smbus2", "spidev", "dronekit",
}
FORBIDDEN_PROCESS_IDENTIFIERS = {
    "Popen", "check_call", "check_output", "terminate", "SIGKILL", "SIGTERM", "_SITLProcessHandle",
}
FORBIDDEN_FLIGHT_COMMAND_IDENTIFIERS = {
    "MAV_CMD_COMPONENT_ARM_DISARM", "MAV_CMD_NAV_TAKEOFF", "MAV_CMD_NAV_LAND", "MAV_CMD_DO_SET_MODE",
    "SET_POSITION_TARGET_LOCAL_NED", "set_position_target_local_ned_send", "command_long_send",
    "param_set_send", "mav_param_set_send",
}


def _tree() -> ast.AST:
    return ast.parse(pathlib.Path(phase12_module.__file__).read_text())


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


# --- no hardware / process spawning / process killing -----------------------

def test_no_hardware_specific_imports():
    imported = _collect_imports(_tree())
    leaked = imported & FORBIDDEN_HARDWARE_IMPORTS
    assert not leaked, f"imports a forbidden hardware module: {leaked}"


def test_no_subprocess_module_imported():
    imported = _collect_imports(_tree())
    assert "subprocess" not in imported


def test_no_process_lifecycle_identifiers_referenced():
    identifiers = _collect_identifiers(_tree())
    leaked = identifiers & FORBIDDEN_PROCESS_IDENTIFIERS
    assert not leaked, f"references forbidden process-lifecycle identifiers: {leaked}"


def test_never_spawns_or_kills_webots_or_ardupilot():
    source = pathlib.Path(phase12_module.__file__).read_text()
    for forbidden in ("os.system(", "os.exec", ".kill(", "os.kill("):
        assert forbidden not in source, f"unexpected process-control call: {forbidden}"


# --- never arm/takeoff/land/mode/setpoint/param-write ------------------------

def test_no_flight_command_identifiers_anywhere():
    identifiers = _collect_identifiers(_tree())
    leaked = identifiers & FORBIDDEN_FLIGHT_COMMAND_IDENTIFIERS
    assert not leaked, f"references forbidden flight-command identifiers: {leaked}"


def test_never_forces_arm_with_the_force_magic_value():
    for node in ast.walk(_tree()):
        if isinstance(node, ast.Constant) and node.value == 21196:
            raise AssertionError("force-arm/disarm magic value (21196) must never appear")


# --- profile handling ---------------------------------------------------------

def test_only_smoke_test_profile_is_implemented():
    assert phase12_module.PROFILE_SMOKE_TEST == "webots_offline_smoke_test"
    assert phase12_module.PROFILE_FLIGHT_TEST == "webots_sitl_flight_test"
    args = phase12_module.build_parser().parse_args([])
    assert args.profile == phase12_module.PROFILE_SMOKE_TEST


def test_flight_test_profile_never_silently_runs():
    source = pathlib.Path(phase12_module.__file__).read_text()
    assert "not implemented in this phase" in source


# --- no hardcoded non-loopback/non-private IP; explicit-only addresses ------

def test_no_hardcoded_public_ip_literal():
    ip_pattern = re.compile(r"^\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}$")
    import ipaddress
    for node in ast.walk(_tree()):
        if isinstance(node, ast.Constant) and isinstance(node.value, str) and ip_pattern.match(node.value):
            ip = ipaddress.ip_address(node.value)
            assert ip.is_loopback or ip.is_private, f"unexpected non-local hardcoded IP literal: {node.value!r}"


def test_sitl_address_and_sim_address_default_to_none_never_guessed():
    args = phase12_module.build_parser().parse_args([])
    assert args.sitl_address is None
    assert args.sim_address is None


def test_default_connection_is_loopback():
    args = phase12_module.build_parser().parse_args([])
    assert args.connection.split(":")[1] in ("127.0.0.1", "localhost")


# --- no six-drone swarm / consensus / planner / SafetySupervisor bypass -----

def test_no_swarm_or_consensus_or_planner_references():
    tree = _tree()
    identifiers = _collect_identifiers(tree)
    imports = _collect_imports(tree)
    assert "distributed_consensus" not in imports
    assert "pybullet" not in imports
    assert not (identifiers & {
        "FloodSearchMission", "DistributedConsensus", "DroneConsensusNode",
        "CandidateCommand", "SafetySupervisor", "num_drones", "VEHICLE_IDS",
    })


def test_no_flightgear_or_pybullet_fallback():
    tree = _tree()
    imports = _collect_imports(tree)
    assert "pybullet" not in imports
    assert "flightgear_bridge" not in " ".join(imports)
    source = pathlib.Path(phase12_module.__file__).read_text()
    assert "telemetry_mapping" not in source
    assert "FlightGearBridge" not in source


def test_no_multi_vehicle_endpoint_construction():
    tree = _tree()
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            fn = node.func
            is_transport_ctor = isinstance(fn, ast.Name) and fn.id == "ArduPilotSITLTransport"
            if is_transport_ctor and node.args:
                first_arg = node.args[0]
                assert isinstance(first_arg, ast.Dict) and len(first_arg.keys) == 1, (
                    "ArduPilotSITLTransport must be constructed with exactly one vehicle"
                )


# --- preserves earlier phases --------------------------------------------------

def test_phase10_still_never_arms_in_dry_run_or_diagnostics():
    import run_phase10_sitl_flight_test as phase10_module
    tree = ast.parse(pathlib.Path(phase10_module.__file__).read_text())
    for fn_name in ("run_dry_run", "run_prearm_diagnostics"):
        fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == fn_name)
        assert not (_collect_identifiers(fn) & FORBIDDEN_FLIGHT_COMMAND_IDENTIFIERS)


def test_phase11_still_never_sends_flight_commands():
    import run_phase11_flightgear_viewer as phase11_module
    tree = ast.parse(pathlib.Path(phase11_module.__file__).read_text())
    assert not (_collect_identifiers(tree) & (FORBIDDEN_FLIGHT_COMMAND_IDENTIFIERS - {"command_long_send"}))


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


# --- default CLI never selects a live run --------------------------------------

def test_default_cli_args_select_dry_run():
    args = phase12_module.build_parser().parse_args([])
    assert args.dry_run is False
    assert args.run is False


def test_main_dispatch_falls_back_to_dry_run():
    source = pathlib.Path(phase12_module.__file__).read_text()
    tree = ast.parse(source)
    main_fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "main")
    main_source = ast.get_source_segment(source, main_fn)
    assert "run_dry_run(args)" in main_source
