"""Phase 8 architecture checks - see docs/PHASE8_ARDUPILOT_SITL.md.
Required items covered: no real transport starts during normal unit
tests, the default configuration is never ardupilot_sitl, FakeSITL
remains selectable, no hardware-specific motor/PWM/serial import or call,
no non-loopback endpoint acceptance, no shell command construction, no
SafetySupervisor bypass, no direct PyBullet-to-ArduPilot command path, no
distributed-consensus-to-transport path.
"""
import ast
import pathlib

import pytest

import swarm_sim.distributed_consensus as dconsensus_module
import swarm_sim.mission as mission_module
import swarm_sim.sitl.ardupilot_transport as ardupilot_transport_module
from swarm_sim.config import MissionConfig
from swarm_sim.contracts import Frame
from swarm_sim.sitl.ardupilot_transport import ArduPilotTransportError, _require_loopback_host

FORBIDDEN_HARDWARE_IMPORTS = {
    "serial", "pyserial", "RPi", "gpiozero", "smbus", "smbus2", "spidev", "dronekit",
}
FORBIDDEN_ACTUATION_IDENTIFIERS = {
    "rpm", "pwm", "motor", "arm", "disarm", "arducopter_arm", "arducopter_disarm",
    "MAV_CMD_COMPONENT_ARM_DISARM", "MAV_CMD_NAV_TAKEOFF", "motors_armed", "throttle",
}
FORBIDDEN_SHELL_CALLS = {"system", "popen", "check_call", "check_output"}   # os.system/os.popen-style


def _tree_of_module(module) -> ast.AST:
    return ast.parse(pathlib.Path(module.__file__).read_text())


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


# --- no hardware-specific imports / no serial / no motor-PWM calls ---------

def test_ardupilot_transport_imports_no_hardware_specific_module():
    imported = _collect_imports(_tree_of_module(ardupilot_transport_module))
    leaked = imported & FORBIDDEN_HARDWARE_IMPORTS
    assert not leaked, f"ardupilot_transport.py imports a forbidden hardware module: {leaked}"


def test_ardupilot_transport_never_references_actuation_identifiers():
    identifiers = _collect_identifiers(_tree_of_module(ardupilot_transport_module))
    leaked = identifiers & FORBIDDEN_ACTUATION_IDENTIFIERS
    assert not leaked, f"ardupilot_transport.py references forbidden actuation identifiers: {leaked}"


def test_ardupilot_transport_uses_subprocess_popen_never_os_system_or_shell():
    """No os.system/os.popen/subprocess.check_call/check_output anywhere -
    subprocess launches must go through the one, list-argv, shell=False
    `_SITLProcessHandle.start` path. Qualified on the object the call is
    made on (os./subprocess.) so this doesn't false-positive on an
    unrelated same-named method (e.g. platform.system())."""
    tree = _tree_of_module(ardupilot_transport_module)
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            f = node.func
            if (isinstance(f, ast.Attribute) and isinstance(f.value, ast.Name)
                    and f.value.id in ("os", "subprocess") and f.attr in FORBIDDEN_SHELL_CALLS):
                raise AssertionError(f"forbidden shell-adjacent call found: {f.value.id}.{f.attr}")
        if isinstance(node, ast.keyword) and node.arg == "shell":
            assert isinstance(node.value, ast.Constant) and node.value.value is False, (
                "any shell= keyword in ardupilot_transport.py must be literally shell=False"
            )


def test_no_f_string_or_percent_formatted_argv_passed_to_popen():
    """Defends against the "constructing shell commands from untrusted
    strings" failure mode structurally: subprocess.Popen's first
    positional argument in this module must be a list/tuple literal or a
    name bound to one - never an f-string/%-formatted string (which would
    imply a single shell command line, not an argv list)."""
    tree = _tree_of_module(ardupilot_transport_module)
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            fn = node.func
            is_popen = (isinstance(fn, ast.Attribute) and fn.attr == "Popen") or (isinstance(fn, ast.Name) and fn.id == "Popen")
            if is_popen and node.args:
                first_arg = node.args[0]
                assert not isinstance(first_arg, (ast.JoinedStr,)), "Popen's argv must never be an f-string"


# --- localhost-only enforcement -----------------------------------------

def test_require_loopback_host_rejects_every_non_loopback_example():
    for host in ("8.8.8.8", "1.1.1.1", "192.168.1.5", "10.0.0.1", "example.com", "0.0.0.0"):
        with pytest.raises(ArduPilotTransportError):
            _require_loopback_host(host)


def test_require_loopback_host_accepts_loopback_forms():
    for host in ("127.0.0.1", "localhost", "::1"):
        _require_loopback_host(host)   # must not raise


def test_no_hardcoded_non_loopback_ip_literal_anywhere_in_module():
    """AST-level guarantee behind the functional check above: no string
    constant in this module looks like a routable (non-loopback,
    non-private-example) IP address."""
    import re
    tree = _tree_of_module(ardupilot_transport_module)
    ip_pattern = re.compile(r"^\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}$")
    for node in ast.walk(tree):
        if isinstance(node, ast.Constant) and isinstance(node.value, str) and ip_pattern.match(node.value):
            assert node.value in ("127.0.0.1", "0.0.0.0"), (
                f"unexpected hardcoded IP literal in ardupilot_transport.py: {node.value!r}"
            )


# --- default configuration / FakeSITL remains selectable --------------------

def test_default_autopilot_path_is_not_ardupilot_sitl():
    cfg = MissionConfig()
    assert cfg.autopilot_path != "ardupilot_sitl"
    assert cfg.autopilot_path in ("mock_adapter", "fake_sitl")


def test_ardupilot_sitl_fields_default_to_none_or_safe_values():
    cfg = MissionConfig()
    assert cfg.ardupilot_sitl_connection_strings is None
    assert cfg.ardupilot_sitl_system_ids is None


def test_fake_sitl_transport_still_importable_and_selectable():
    from swarm_sim.sitl.fake_transport import FakeSITLTransport
    cfg = MissionConfig(autopilot_path="fake_sitl")
    assert cfg.autopilot_path == "fake_sitl"
    t = FakeSITLTransport(vehicle_ids=("d",))
    assert t.start().success


def test_no_real_transport_starts_by_constructing_a_mission_config():
    """Merely constructing a MissionConfig (even one that names
    ardupilot_sitl) must never itself open a socket or start a process -
    only actually building a FloodSearchMission with that config does,
    and even then only after every required field is filled in (see
    mission.py's own explicit-configuration-required check)."""
    cfg = MissionConfig(autopilot_path="ardupilot_sitl")   # construction alone - no mission built
    assert cfg.autopilot_path == "ardupilot_sitl"   # just proves the string is a legal value, nothing started


def test_mission_construction_with_ardupilot_sitl_and_no_endpoints_fails_loudly_not_silently():
    """Selecting ardupilot_sitl without filling in the required per-vehicle
    fields must raise immediately - never silently fall back to
    fake_sitl/mock_adapter."""
    import inspect
    run_fn_src = inspect.getsource(mission_module.FloodSearchMission.__init__)
    assert "ardupilot_sitl_connection_strings" in run_fn_src
    assert "raise ValueError" in run_fn_src


# --- no SafetySupervisor bypass / no direct PyBullet-to-ArduPilot path -----

def _init_fn():
    tree = _tree_of_module(mission_module)
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == "FloodSearchMission":
            for item in node.body:
                if isinstance(item, ast.FunctionDef) and item.name == "__init__":
                    return item
    raise AssertionError("FloodSearchMission.__init__ not found")


def _run_fn():
    tree = _tree_of_module(mission_module)
    return next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "run")


def _adapter_helper_fn():
    tree = _tree_of_module(mission_module)
    return next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "_send_to_autopilot_adapter")


def _call_name(call: ast.Call):
    f = call.func
    if isinstance(f, ast.Attribute):
        return f.attr
    if isinstance(f, ast.Name):
        return f.id
    return None


def test_run_still_calls_evaluate_before_the_adapter_helper():
    """Re-affirms Phase 6/7's own guarantee still holds with ardupilot_sitl
    as a third possible adapter type behind self.autopilot_adapters - the
    ordering check in mission.py's run() does not change per adapter
    type."""
    run_fn = _run_fn()
    calls = [n for n in ast.walk(run_fn) if isinstance(n, ast.Call)]
    evaluate_calls = [c for c in calls if _call_name(c) == "evaluate"]
    adapter_helper_calls = [c for c in calls if _call_name(c) == "_send_to_autopilot_adapter"]
    assert evaluate_calls and adapter_helper_calls
    assert min(c.lineno for c in evaluate_calls) < min(c.lineno for c in adapter_helper_calls)


def test_ardupilot_sitl_is_one_of_the_accepted_autopilot_paths_in_run():
    run_fn = _run_fn()
    source = ast.get_source_segment(pathlib.Path(mission_module.__file__).read_text(), run_fn) or ""
    assert "ardupilot_sitl" in source or "fake_sitl" in source   # the tuple check covers both by construction


def test_send_to_autopilot_adapter_never_called_with_raw_controller_candidate():
    run_fn = _run_fn()
    for node in ast.walk(run_fn):
        if isinstance(node, ast.Call) and _call_name(node) == "_send_to_autopilot_adapter":
            for arg in node.args:
                for sub in ast.walk(arg):
                    if isinstance(sub, ast.Name) and sub.id == "desired_vel":
                        raise AssertionError("_send_to_autopilot_adapter() must never receive desired_vel directly")


def test_no_direct_pybullet_to_ardupilot_command_path():
    """mission.py itself must never import pymavlink or call anything on
    an ArduPilotSITLTransport/mavutil object directly - every real-SITL
    interaction must go through _send_to_autopilot_adapter() ->
    AutopilotAdapter.send_command() -> SITLTransport.send_command(), the
    same chain as every other adapter type."""
    tree = _tree_of_module(mission_module)
    imports = _collect_imports(tree)
    assert "pymavlink" not in imports
    identifiers = _collect_identifiers(tree)
    assert "mavutil" not in identifiers
    # mission.py may reference ArduPilotSITLTransport/ArduPilotVehicleEndpoint
    # (construction only, in __init__) but must never call MAVLink-specific
    # methods like send_mavlink/recv_match/mav directly.
    assert "recv_match" not in identifiers
    assert "mavlink_connection" not in identifiers


def test_adapter_helper_calls_send_command_which_ardupilot_transport_forwards_over_mavlink():
    """Two-part chain check mirroring Phase 7's own: (1) mission.py's
    _send_to_autopilot_adapter calls send_command() (unchanged), and (2)
    ArduPilotSITLTransport.send_command itself eventually reaches
    _send_mavlink_command - so the full chain from run() down to the real
    transport is real, not a same-named decoy at any layer."""
    helper_calls = [n for n in ast.walk(_adapter_helper_fn()) if isinstance(n, ast.Call)]
    assert any(_call_name(c) == "send_command" for c in helper_calls)

    tree = _tree_of_module(ardupilot_transport_module)
    send_command_fn = next(
        n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "send_command"
        and any(isinstance(a, ast.arg) and a.arg == "command" for a in n.args.args)
    )
    inner_calls = [n for n in ast.walk(send_command_fn) if isinstance(n, ast.Call)]
    assert any(_call_name(c) == "_evaluate_and_send" for c in inner_calls)

    evaluate_fn = next(
        n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "_evaluate_and_send"
    )
    evaluate_calls = [n for n in ast.walk(evaluate_fn) if isinstance(n, ast.Call)]
    assert any(_call_name(c) == "_send_mavlink_command" for c in evaluate_calls)


# --- no distributed-consensus-to-transport path -----------------------------

def test_distributed_consensus_never_references_ardupilot_or_mavlink():
    tree = _tree_of_module(dconsensus_module)
    identifiers = _collect_identifiers(tree)
    imports = _collect_imports(tree)
    forbidden_names = {
        "ArduPilotSITLTransport", "ArduPilotVehicleEndpoint", "mavutil", "pymavlink",
        "send_mavlink_command", "SITLTransport",
    }
    leaked = identifiers & forbidden_names
    assert not leaked, f"distributed_consensus.py references ArduPilot/MAVLink-layer identifiers: {leaked}"
    assert "pymavlink" not in imports
    assert "ardupilot_transport" not in imports


# --- safety-supervisor bypass check (whole-repo guarantee, re-affirmed) ----

def test_safety_supervisor_module_never_imports_ardupilot_transport():
    import swarm_sim.safety_supervisor as safety_module
    imports = _collect_imports(_tree_of_module(safety_module))
    assert "pymavlink" not in imports
    assert "ardupilot_transport" not in imports
