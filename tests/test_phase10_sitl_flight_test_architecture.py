"""Phase 10 architecture checks - see docs/PHASE10_SITL_FLIGHT_TEST.md.
Required items covered (Step 5): default profile never arms/takes off,
required opt-in flags, one-vehicle-only, localhost-only, no ARMING_CHECK=0
write, no raw planner candidate/distributed-consensus reaching the
transport, no PyBullet mission construction, no six-vehicle configuration.

Behavioral items (wrong system/component ID rejected, emergency LAND
attempted on timeout, failures stop rather than continue) are functional,
not structural, and are covered by tests/test_phase10_sitl_flight_test.py
instead - mirroring Phase 8/9's own split between architecture (AST) and
functional test files.
"""
import ast
import os
import pathlib
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"))

import run_phase10_sitl_flight_test as phase10_module  # noqa: E402

FORBIDDEN_HARDWARE_IMPORTS = {
    "serial", "pyserial", "RPi", "gpiozero", "smbus", "smbus2", "spidev", "dronekit",
}
FORBIDDEN_PROCESS_IDENTIFIERS = {
    "Popen", "check_call", "check_output", "terminate", "SIGKILL", "SIGTERM", "_SITLProcessHandle",
}


def _tree() -> ast.AST:
    return ast.parse(pathlib.Path(phase10_module.__file__).read_text())


def _collect_identifiers(tree: ast.AST):
    names = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Name):
            names.add(node.id)
        elif isinstance(node, ast.Attribute):
            names.add(node.attr)
        elif isinstance(node, ast.arg):
            names.add(node.arg)
    return names


def _collect_imports(tree: ast.AST):
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module.split(".")[0])
    return imported


def _call_name(call: ast.Call):
    f = call.func
    if isinstance(f, ast.Attribute):
        return f.attr
    if isinstance(f, ast.Name):
        return f.id
    return None


def _fn(name: str):
    tree = _tree()
    return next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)


# --- no hardware / process spawning / process killing -----------------------

def test_no_hardware_specific_imports():
    imported = _collect_imports(_tree())
    leaked = imported & FORBIDDEN_HARDWARE_IMPORTS
    assert not leaked, f"Phase 10 script imports a forbidden hardware module: {leaked}"


def test_no_subprocess_module_imported_at_all():
    imported = _collect_imports(_tree())
    assert "subprocess" not in imported


def test_no_process_lifecycle_identifiers_referenced():
    identifiers = _collect_identifiers(_tree())
    leaked = identifiers & FORBIDDEN_PROCESS_IDENTIFIERS
    assert not leaked, f"Phase 10 script references forbidden process-lifecycle identifiers: {leaked}"


# --- localhost-only / no non-loopback endpoint ------------------------------

def test_default_connection_is_loopback():
    args = phase10_module.build_parser().parse_args([])
    assert args.connection.split(":")[1] in ("127.0.0.1", "localhost")


def test_no_hardcoded_non_loopback_ip_literal_anywhere_in_module():
    tree = _tree()
    ip_pattern = re.compile(r"^\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}$")
    for node in ast.walk(tree):
        if isinstance(node, ast.Constant) and isinstance(node.value, str) and ip_pattern.match(node.value):
            assert node.value == "127.0.0.1", f"unexpected hardcoded IP literal: {node.value!r}"


def test_gate_checks_localhost_connection():
    tree = _tree()
    gate_fn = _fn("_check_flight_test_gate")
    source = ast.get_source_segment(pathlib.Path(phase10_module.__file__).read_text(), gate_fn)
    assert "localhost_connection" in source
    assert "127.0.0.1" in source


# --- default profile never arms / never takes off ---------------------------

def test_default_cli_args_never_select_flight_test():
    args = phase10_module.build_parser().parse_args([])
    assert args.sitl_only is False
    assert args.allow_sitl_arm_test is False
    assert args.confirm_sitl_flight_test is False
    assert args.dry_run is False
    assert args.attach is False
    assert args.diagnose_prearm is False


def test_no_arm_or_takeoff_identifiers_in_dry_run_or_diagnostics_functions():
    forbidden = {"MAV_CMD_COMPONENT_ARM_DISARM", "MAV_CMD_NAV_TAKEOFF"}
    for fn_name in ("run_dry_run", "run_prearm_diagnostics"):
        identifiers = _collect_identifiers(_fn(fn_name))
        leaked = identifiers & forbidden
        assert not leaked, f"{fn_name} references forbidden arm/takeoff identifiers: {leaked}"


def test_prearm_diagnostics_never_calls_param_set():
    identifiers = _collect_identifiers(_fn("run_prearm_diagnostics"))
    assert "param_set_send" not in identifiers


# --- required opt-in flags are actually required ----------------------------

def test_gate_requires_sitl_only_and_allow_sitl_arm_test():
    gate_fn = _fn("_check_flight_test_gate")
    source = ast.get_source_segment(pathlib.Path(phase10_module.__file__).read_text(), gate_fn)
    assert "args.sitl_only" in source
    assert "args.allow_sitl_arm_test" in source


def test_confirmation_is_checked_before_any_attach_in_flight_test():
    fn = _fn("run_sitl_flight_test")
    calls = [n for n in ast.walk(fn) if isinstance(n, ast.Call)]
    confirm_calls = [c for c in calls if _call_name(c) == "_operator_confirmed"]
    endpoint_calls = [c for c in calls if _call_name(c) == "_try_build_endpoint"]
    start_calls = [c for c in calls if _call_name(c) == "start"]
    assert confirm_calls and endpoint_calls and start_calls
    assert min(c.lineno for c in confirm_calls) < min(c.lineno for c in endpoint_calls) < min(c.lineno for c in start_calls)


# --- one vehicle only / no six-vehicle configuration ------------------------

def test_no_multi_vehicle_endpoint_dict_construction():
    """Every ArduPilotSITLTransport(...) call site in this module must be
    constructed from a single-entry dict ({vehicle_id: endpoint}) - never a
    loop or comprehension building multiple endpoints."""
    tree = _tree()
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            fn = node.func
            is_transport_ctor = (isinstance(fn, ast.Name) and fn.id == "ArduPilotSITLTransport")
            if is_transport_ctor and node.args:
                first_arg = node.args[0]
                assert isinstance(first_arg, ast.Dict) and len(first_arg.keys) == 1, (
                    "ArduPilotSITLTransport must be constructed with exactly one vehicle"
                )


def test_no_num_drones_or_six_vehicle_configuration():
    identifiers = _collect_identifiers(_tree())
    assert "num_drones" not in identifiers
    assert "VEHICLE_IDS" not in identifiers


# --- geofence gate requires FENCE_ENABLE, never writes a parameter ---------

def test_geofence_gate_checks_fence_enabled():
    gate_fn = _fn("_check_geofence_gate")
    source = ast.get_source_segment(pathlib.Path(phase10_module.__file__).read_text(), gate_fn)
    assert "fence_enabled" in source


def test_geofence_gate_never_calls_param_set():
    identifiers = _collect_identifiers(_fn("_check_geofence_gate"))
    assert "param_set_send" not in identifiers
    assert "mav_param_set_send" not in identifiers


def test_flight_test_calls_geofence_gate_before_setting_mode_or_arming():
    """The geofence gate must be evaluated (and, if failed, must stop)
    before GUIDED mode is set or an arm command is sent."""
    fn = _fn("run_sitl_flight_test")
    calls = [n for n in ast.walk(fn) if isinstance(n, ast.Call)]
    gate_calls = [c for c in calls if _call_name(c) == "_check_geofence_gate"]
    mode_calls = [c for c in calls if _call_name(c) == "_set_mode_guided"]
    assert gate_calls and mode_calls
    assert min(c.lineno for c in gate_calls) < min(c.lineno for c in mode_calls)


# --- no ARMING_CHECK=0 written, no parameter writes at all ------------------

def test_never_calls_param_set_anywhere_in_module():
    identifiers = _collect_identifiers(_tree())
    assert "param_set_send" not in identifiers
    assert "mav_param_set_send" not in identifiers


def test_never_forces_arm_with_the_force_magic_value():
    """MAV_CMD_COMPONENT_ARM_DISARM's param2 (force) must never be the
    documented force-arm/disarm magic value (21196) anywhere in this
    module - every arm/disarm call in this file passes p2=0."""
    tree = _tree()
    for node in ast.walk(tree):
        if isinstance(node, ast.Constant) and node.value == 21196:
            raise AssertionError("force-arm/disarm magic value (21196) must never appear in this module")


# --- no raw planner candidate / distributed-consensus reaching the transport

def test_no_candidate_command_or_safety_supervisor_import_in_manual_flight_path():
    """The manual arm/takeoff/land path must stay a clean, separate code
    path from the supervised planner_preview path (Phase 9, reused
    unmodified) - see module docstring's "Command authority" section."""
    identifiers = _collect_identifiers(_fn("run_sitl_flight_test"))
    assert "CandidateCommand" not in identifiers
    assert "SafetySupervisor" not in identifiers
    assert "evaluate" not in identifiers


def test_no_distributed_consensus_references():
    tree = _tree()
    identifiers = _collect_identifiers(tree)
    imports = _collect_imports(tree)
    forbidden_names = {"DistributedConsensus", "DroneConsensusNode", "distributed_consensus"}
    leaked = identifiers & forbidden_names
    assert not leaked, f"Phase 10 script references distributed-consensus identifiers: {leaked}"
    assert "distributed_consensus" not in imports


# --- no PyBullet mission construction ---------------------------------------

def test_no_pybullet_or_flood_search_mission_reference():
    tree = _tree()
    identifiers = _collect_identifiers(tree)
    imports = _collect_imports(tree)
    assert "pybullet" not in imports
    assert "FloodSearchMission" not in identifiers


# --- manual command path never routes through ArduPilotSITLTransport.send_command

def test_manual_flight_commands_never_call_transport_send_command():
    fn = _fn("run_sitl_flight_test")
    calls = [n for n in ast.walk(fn) if isinstance(n, ast.Call)]
    assert not any(_call_name(c) == "send_command" for c in calls), (
        "the manual arm/takeoff/land path must never call ArduPilotSITLTransport.send_command()"
    )


def test_ardupilot_transport_module_still_never_references_arm_or_takeoff():
    """Regression re-check of Phase 8's own guarantee - Phase 10 does not
    touch ardupilot_transport.py, so this must still hold unmodified."""
    import swarm_sim.sitl.ardupilot_transport as transport_module
    tree = ast.parse(pathlib.Path(transport_module.__file__).read_text())
    identifiers = _collect_identifiers(tree)
    assert "MAV_CMD_COMPONENT_ARM_DISARM" not in identifiers
    assert "MAV_CMD_NAV_TAKEOFF" not in identifiers


# --- Phase 9 remains untouched -----------------------------------------------

def test_phase9_module_is_imported_not_duplicated():
    tree = _tree()
    imports = [n for n in ast.walk(tree) if isinstance(n, ast.Import)]
    assert any(
        any(alias.name == "run_phase9_single_vehicle_visual" for alias in node.names)
        for node in imports
    ), "Phase 10 must import Phase 9's module, not reimplement its telemetry/hold/planner modes"
