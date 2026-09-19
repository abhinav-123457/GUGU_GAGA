"""Phase 11 architecture checks - see docs/PHASE11_FLIGHTGEAR_VIEWER.md.
AST inspection of scripts/run_phase11_flightgear_viewer.py and
swarm_sim/visualization/*.py, mirroring Phase 8/9/10's own architecture-
test pattern (structural proof, not just runtime behavior).
"""
import ast
import os
import pathlib
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"))

import run_phase11_flightgear_viewer as phase11_module  # noqa: E402

from swarm_sim.visualization import flightgear_bridge as fg_bridge_module  # noqa: E402
from swarm_sim.visualization import telemetry_mapping as tmap_module  # noqa: E402

FORBIDDEN_HARDWARE_IMPORTS = {
    "serial", "pyserial", "RPi", "gpiozero", "smbus", "smbus2", "spidev", "dronekit",
}
FORBIDDEN_PROCESS_IDENTIFIERS = {
    "Popen", "check_call", "check_output", "terminate", "SIGKILL", "SIGTERM", "_SITLProcessHandle",
}
FORBIDDEN_FLIGHT_COMMAND_IDENTIFIERS = {
    "MAV_CMD_COMPONENT_ARM_DISARM", "MAV_CMD_NAV_TAKEOFF", "MAV_CMD_NAV_LAND", "MAV_CMD_DO_SET_MODE",
    "SET_POSITION_TARGET_LOCAL_NED", "set_position_target_local_ned_send",
}


def _tree(module) -> ast.AST:
    return ast.parse(pathlib.Path(module.__file__).read_text())


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


def _call_name(call: ast.Call):
    f = call.func
    if isinstance(f, ast.Attribute):
        return f.attr
    if isinstance(f, ast.Name):
        return f.id
    return None


def _fn(module, name: str):
    tree = _tree(module)
    return next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)


# --- no hardware / process spawning / process killing -----------------------

def test_no_hardware_specific_imports():
    for module in (phase11_module, fg_bridge_module, tmap_module):
        imported = _collect_imports(_tree(module))
        leaked = imported & FORBIDDEN_HARDWARE_IMPORTS
        assert not leaked, f"{module.__name__} imports a forbidden hardware module: {leaked}"


def test_no_subprocess_module_imported_anywhere():
    for module in (phase11_module, fg_bridge_module, tmap_module):
        imported = _collect_imports(_tree(module))
        assert "subprocess" not in imported


def test_no_process_lifecycle_identifiers_referenced():
    for module in (phase11_module, fg_bridge_module, tmap_module):
        identifiers = _collect_identifiers(_tree(module))
        leaked = identifiers & FORBIDDEN_PROCESS_IDENTIFIERS
        assert not leaked, f"{module.__name__} references forbidden process-lifecycle identifiers: {leaked}"


# --- never arm/takeoff/land/mode/setpoint ------------------------------------

def test_no_flight_command_identifiers_anywhere_in_script():
    identifiers = _collect_identifiers(_tree(phase11_module))
    leaked = identifiers & FORBIDDEN_FLIGHT_COMMAND_IDENTIFIERS
    assert not leaked, f"script references forbidden flight-command identifiers: {leaked}"


def test_no_flight_command_identifiers_in_bridge_module():
    identifiers = _collect_identifiers(_tree(fg_bridge_module))
    leaked = identifiers & FORBIDDEN_FLIGHT_COMMAND_IDENTIFIERS
    assert not leaked


def test_never_forces_arm_with_the_force_magic_value():
    for module in (phase11_module, fg_bridge_module, tmap_module):
        tree = _tree(module)
        for node in ast.walk(tree):
            if isinstance(node, ast.Constant) and node.value == 21196:
                raise AssertionError(f"force-arm/disarm magic value (21196) must never appear in {module.__name__}")


# --- the ONLY command_long_send target is MAV_CMD_SET_MESSAGE_INTERVAL -----

def test_only_command_sent_is_set_message_interval():
    tree = _tree(phase11_module)
    calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call) and _call_name(n) == "command_long_send"]
    assert calls, "expected at least one command_long_send call (the GLOBAL_POSITION_INT stream request)"
    for call in calls:
        source = ast.get_source_segment(pathlib.Path(phase11_module.__file__).read_text(), call)
        assert "MAV_CMD_SET_MESSAGE_INTERVAL" in source, (
            f"command_long_send call does not target MAV_CMD_SET_MESSAGE_INTERVAL: {source}"
        )


def test_bridge_module_never_calls_command_long_send():
    identifiers = _collect_identifiers(_tree(fg_bridge_module))
    assert "command_long_send" not in identifiers


# --- no parameter writes -----------------------------------------------------

def test_never_calls_param_set_anywhere():
    for module in (phase11_module, fg_bridge_module, tmap_module):
        identifiers = _collect_identifiers(_tree(module))
        assert "param_set_send" not in identifiers
        assert "mav_param_set_send" not in identifiers


# --- localhost-only, both endpoints -------------------------------------------

def test_default_connection_is_loopback():
    args = phase11_module.build_parser().parse_args([])
    assert args.connection.split(":")[1] in ("127.0.0.1", "localhost")


def test_no_hardcoded_non_loopback_ip_literal_anywhere():
    ip_pattern = re.compile(r"^\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}$")
    for module in (phase11_module, fg_bridge_module, tmap_module):
        for node in ast.walk(_tree(module)):
            if isinstance(node, ast.Constant) and isinstance(node.value, str) and ip_pattern.match(node.value):
                assert node.value == "127.0.0.1", f"unexpected hardcoded IP literal in {module.__name__}: {node.value!r}"


def test_flightgear_endpoint_parser_checks_loopback():
    fn = _fn(fg_bridge_module, "parse_flightgear_endpoint")
    source = ast.get_source_segment(pathlib.Path(fg_bridge_module.__file__).read_text(), fn)
    assert "_LOOPBACK_HOSTS" in source


def test_bridge_never_calls_socket_recv():
    """Structural proof that FlightGear can never send anything this
    project would act on - the bridge module never calls .recv()/
    .recvfrom() on its own socket at all."""
    identifiers = _collect_identifiers(_tree(fg_bridge_module))
    assert "recv" not in identifiers
    assert "recvfrom" not in identifiers


# --- one vehicle only / no six-drone swarm ------------------------------------

def test_no_multi_vehicle_endpoint_dict_construction():
    tree = _tree(phase11_module)
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            fn = node.func
            is_transport_ctor = isinstance(fn, ast.Name) and fn.id == "ArduPilotSITLTransport"
            if is_transport_ctor and node.args:
                first_arg = node.args[0]
                assert isinstance(first_arg, ast.Dict) and len(first_arg.keys) == 1, (
                    "ArduPilotSITLTransport must be constructed with exactly one vehicle"
                )


def test_no_num_drones_or_six_vehicle_configuration():
    identifiers = _collect_identifiers(_tree(phase11_module))
    assert "num_drones" not in identifiers
    assert "VEHICLE_IDS" not in identifiers


# --- no distributed-consensus / no PyBullet / no raw planner candidate -------

def test_no_distributed_consensus_or_pybullet_references():
    for module in (phase11_module, fg_bridge_module, tmap_module):
        tree = _tree(module)
        identifiers = _collect_identifiers(tree)
        imports = _collect_imports(tree)
        assert "distributed_consensus" not in imports
        assert "pybullet" not in imports
        assert "FloodSearchMission" not in identifiers
        assert not (identifiers & {"DistributedConsensus", "DroneConsensusNode"})


def test_no_raw_candidate_command_access():
    identifiers = _collect_identifiers(_tree(phase11_module))
    assert "CandidateCommand" not in identifiers
    assert "SafetySupervisor" not in identifiers


# --- default CLI never selects a live mode ------------------------------------

def test_default_cli_args_select_dry_run():
    args = phase11_module.build_parser().parse_args([])
    assert args.dry_run is False   # explicit --dry-run flag defaults False...
    assert args.diagnose is False
    assert args.attach is False
    assert args.flightgear_view is False
    assert args.replay is None
    # ...but main()'s own dispatch (see module source) falls through to
    # run_dry_run when none of the above are set, which is exercised by
    # tests/test_phase11_flightgear_viewer.py's own dry-run tests.


def test_main_dispatch_falls_back_to_dry_run():
    source = pathlib.Path(phase11_module.__file__).read_text()
    tree = ast.parse(source)
    main_fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "main")
    main_source = ast.get_source_segment(source, main_fn)
    assert "run_dry_run(args)" in main_source


# --- FakeSITL / Phase 10 remain unaffected ------------------------------------

def test_fake_sitl_transport_still_importable_and_functional():
    from swarm_sim.sitl.fake_transport import FakeSITLTransport
    t = FakeSITLTransport(vehicle_ids=("d",))
    assert t.start().success


def test_phase10_module_still_never_references_arm_or_takeoff_in_dry_run():
    import run_phase10_sitl_flight_test as phase10_module
    tree = ast.parse(pathlib.Path(phase10_module.__file__).read_text())
    for fn_name in ("run_dry_run", "run_prearm_diagnostics"):
        fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == fn_name)
        identifiers = _collect_identifiers(fn)
        assert not (identifiers & FORBIDDEN_FLIGHT_COMMAND_IDENTIFIERS)


def test_ardupilot_transport_module_still_never_references_arm_or_takeoff():
    """Regression re-check of Phase 8's own guarantee - Phase 11 does not
    touch ardupilot_transport.py, so this must still hold unmodified."""
    import swarm_sim.sitl.ardupilot_transport as transport_module
    tree = ast.parse(pathlib.Path(transport_module.__file__).read_text())
    identifiers = _collect_identifiers(tree)
    assert "MAV_CMD_COMPONENT_ARM_DISARM" not in identifiers
    assert "MAV_CMD_NAV_TAKEOFF" not in identifiers


def test_phase9_module_is_imported_not_duplicated():
    tree = _tree(phase11_module)
    imports = [n for n in ast.walk(tree) if isinstance(n, ast.Import)]
    assert any(
        any(alias.name == "run_phase9_single_vehicle_visual" for alias in node.names)
        for node in imports
    ), "Phase 11 must import Phase 9's module for --attach, not reimplement it"


# --- telemetry_mapping.py is pure - no I/O ------------------------------------

def test_telemetry_mapping_module_never_imports_sockets_or_mavlink():
    imports = _collect_imports(_tree(tmap_module))
    assert "socket" not in imports
    assert "pymavlink" not in imports
