"""Phase 9 architecture checks - see docs/PHASE9_SINGLE_VEHICLE_VISUAL.md.
Required items covered: no process spawn/kill, no motor/PWM/serial import,
localhost-only, no arm/takeoff, no SafetySupervisor bypass in
planner_preview, no distributed-consensus access, no raw candidate/
desired_vel reaching ArduPilotSITLTransport, FakeSITL remains available.

Mirrors the AST-check style of tests/test_ardupilot_sitl_architecture.py.
"""
import ast
import os
import pathlib
import re
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"))

import run_phase9_single_vehicle_visual as phase9_module  # noqa: E402

FORBIDDEN_HARDWARE_IMPORTS = {
    "serial", "pyserial", "RPi", "gpiozero", "smbus", "smbus2", "spidev", "dronekit",
}
FORBIDDEN_ACTUATION_IDENTIFIERS = {
    "rpm", "pwm", "motor", "arm", "disarm", "arducopter_arm", "arducopter_disarm",
    "MAV_CMD_COMPONENT_ARM_DISARM", "MAV_CMD_NAV_TAKEOFF", "motors_armed", "throttle",
}
FORBIDDEN_PROCESS_IDENTIFIERS = {
    "Popen", "check_call", "check_output", "terminate", "kill", "SIGKILL", "SIGTERM",
    "_SITLProcessHandle",
}


def _tree() -> ast.AST:
    return ast.parse(pathlib.Path(phase9_module.__file__).read_text())


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


# --- no hardware imports / no motor-PWM / no arm / no takeoff --------------

def test_no_hardware_specific_imports():
    imported = _collect_imports(_tree())
    leaked = imported & FORBIDDEN_HARDWARE_IMPORTS
    assert not leaked, f"Phase 9 script imports a forbidden hardware module: {leaked}"


def test_no_actuation_identifiers_referenced():
    identifiers = _collect_identifiers(_tree())
    leaked = identifiers & FORBIDDEN_ACTUATION_IDENTIFIERS
    assert not leaked, f"Phase 9 script references forbidden actuation identifiers: {leaked}"


# --- no process spawning / no process killing -------------------------------

def test_no_subprocess_module_imported_at_all():
    """ATTACH mode only: this script has no legitimate reason to ever
    import subprocess - the operator starts ArduCopter SITL manually."""
    imported = _collect_imports(_tree())
    assert "subprocess" not in imported


def test_no_process_lifecycle_identifiers_referenced():
    identifiers = _collect_identifiers(_tree())
    leaked = identifiers & FORBIDDEN_PROCESS_IDENTIFIERS
    assert not leaked, f"Phase 9 script references forbidden process-lifecycle identifiers: {leaked}"


def test_every_ardupilot_vehicle_endpoint_construction_leaves_executable_path_unset():
    """AST-level guarantee behind the functional
    test_build_attach_endpoint_never_sets_executable_path: no call site in
    this module may pass executable_path/wsl_distro/working_directory at
    all - ATTACH mode is the only mode this phase implements."""
    tree = _tree()
    forbidden_kwargs = {"executable_path", "wsl_distro", "working_directory"}
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            fn = node.func
            is_endpoint_ctor = (isinstance(fn, ast.Name) and fn.id == "ArduPilotVehicleEndpoint") or \
                                (isinstance(fn, ast.Attribute) and fn.attr == "ArduPilotVehicleEndpoint")
            if is_endpoint_ctor:
                used_kwargs = {kw.arg for kw in node.keywords if kw.arg is not None}
                leaked = used_kwargs & forbidden_kwargs
                assert not leaked, f"ArduPilotVehicleEndpoint constructed with spawn-mode kwargs: {leaked}"


# --- localhost-only enforcement ---------------------------------------------

def test_default_connection_is_loopback():
    parser = phase9_module.build_parser()
    args = parser.parse_args([])
    assert args.connection.split(":")[1] in ("127.0.0.1", "localhost")


def test_no_hardcoded_non_loopback_ip_literal_anywhere_in_module():
    tree = _tree()
    ip_pattern = re.compile(r"^\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}$")
    for node in ast.walk(tree):
        if isinstance(node, ast.Constant) and isinstance(node.value, str) and ip_pattern.match(node.value):
            assert node.value == "127.0.0.1", (
                f"unexpected hardcoded IP literal in Phase 9 script: {node.value!r}"
            )


# --- SafetySupervisor is the final command authority in planner_preview ----

def _fn(name: str):
    tree = _tree()
    return next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)


def _call_name(call: ast.Call):
    f = call.func
    if isinstance(f, ast.Attribute):
        return f.attr
    if isinstance(f, ast.Name):
        return f.id
    return None


def test_planner_preview_calls_evaluate_before_send_command():
    fn = _fn("run_planner_preview")
    calls = [n for n in ast.walk(fn) if isinstance(n, ast.Call)]
    evaluate_calls = [c for c in calls if _call_name(c) == "evaluate"]
    send_calls = [c for c in calls if _call_name(c) == "send_command"]
    assert evaluate_calls and send_calls
    assert min(c.lineno for c in evaluate_calls) < min(c.lineno for c in send_calls)


def test_planner_preview_send_command_argument_derives_from_filtered_command():
    """The AdapterCommand actually sent must be built from
    decision.filtered_command - never the raw CandidateCommand
    constructed each tick. Checked structurally: the AdapterCommand(...)
    call site inside run_planner_preview must reference `filtered_command`
    as an attribute access somewhere in its own construction."""
    fn = _fn("run_planner_preview")
    adapter_command_calls = [
        n for n in ast.walk(fn)
        if isinstance(n, ast.Call) and (
            (isinstance(n.func, ast.Name) and n.func.id == "AdapterCommand")
            or (isinstance(n.func, ast.Attribute) and n.func.attr == "AdapterCommand")
        )
    ]
    assert adapter_command_calls
    for call in adapter_command_calls:
        referenced = {a.attr for a in ast.walk(call) if isinstance(a, ast.Attribute)}
        assert "filtered_command" in referenced, (
            "AdapterCommand sent in planner_preview must be built from decision.filtered_command"
        )


def test_no_bare_desired_vel_name_reaches_send_command():
    """Mirrors mission.py's own architecture guarantee
    (test_send_to_autopilot_adapter_never_called_with_raw_controller_candidate):
    no call to send_command anywhere in this module may pass a bare name
    called `desired_vel`."""
    tree = _tree()
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and _call_name(node) == "send_command":
            for arg in list(node.args) + [kw.value for kw in node.keywords]:
                for sub in ast.walk(arg):
                    if isinstance(sub, ast.Name) and sub.id == "desired_vel":
                        raise AssertionError("send_command() must never receive a bare desired_vel")


# --- no distributed-consensus access ----------------------------------------

def test_no_distributed_consensus_references():
    tree = _tree()
    identifiers = _collect_identifiers(tree)
    imports = _collect_imports(tree)
    forbidden_names = {"DistributedConsensus", "DroneConsensusNode", "distributed_consensus"}
    leaked = identifiers & forbidden_names
    assert not leaked, f"Phase 9 script references distributed-consensus identifiers: {leaked}"
    assert "distributed_consensus" not in imports


# --- no PyBullet / no full mission construction -----------------------------

def test_no_pybullet_or_flood_search_mission_reference():
    tree = _tree()
    identifiers = _collect_identifiers(tree)
    imports = _collect_imports(tree)
    assert "pybullet" not in imports
    assert "FloodSearchMission" not in identifiers


# --- FakeSITL remains available ---------------------------------------------

def test_fake_sitl_transport_still_importable_and_selectable():
    from swarm_sim.config import MissionConfig
    from swarm_sim.sitl.fake_transport import FakeSITLTransport
    cfg = MissionConfig(autopilot_path="fake_sitl")
    assert cfg.autopilot_path == "fake_sitl"
    t = FakeSITLTransport(vehicle_ids=("d",))
    assert t.start().success


# --- never arms / never takes off (module-level, re-affirmed) --------------

def test_no_arm_or_takeoff_identifiers_anywhere_in_module():
    identifiers = _collect_identifiers(_tree())
    assert "MAV_CMD_COMPONENT_ARM_DISARM" not in identifiers
    assert "MAV_CMD_NAV_TAKEOFF" not in identifiers
    assert "arm" not in identifiers
    assert "disarm" not in identifiers


# --- default mode is dry-run / never spawns ---------------------------------

def test_default_cli_args_select_dry_run_not_attach():
    parser = phase9_module.build_parser()
    args = parser.parse_args([])
    assert args.attach is False
    assert args.hold_test is False
    assert args.planner_preview is False
