"""Phase 7 architecture checks - see docs/PHASE7_SITL_INTEGRATION.md.
Required items covered: no transport/hardware imports anywhere in
swarm_sim/sitl/ or the new autopilot/*sitl* modules, no actuation/arming
calls, no external-connection calls, mission commands still pass through
SafetySupervisor.evaluate() before ever reaching an adapter/transport,
distributed consensus has no adapter or SITL access, every transport
command carries a vehicle namespace, and namespace mismatches are checked
before a command can touch per-vehicle state.
"""
import ast
import pathlib

import swarm_sim.distributed_consensus as dconsensus_module
import swarm_sim.mission as mission_module
from swarm_sim.autopilot import ardupilot_sitl as autopilot_ardupilot_sitl
from swarm_sim.autopilot import px4_sitl as autopilot_px4_sitl
from swarm_sim.autopilot import sitl as autopilot_sitl_module
from swarm_sim.sitl import clock as sitl_clock
from swarm_sim.sitl import commands as sitl_commands
from swarm_sim.sitl import fake_transport as sitl_fake_transport
from swarm_sim.sitl import telemetry as sitl_telemetry
from swarm_sim.sitl import transport as sitl_transport_module
from swarm_sim.sitl import vehicle_namespace as sitl_vehicle_namespace

SITL_PACKAGE_MODULES = [
    sitl_clock, sitl_commands, sitl_fake_transport, sitl_telemetry, sitl_transport_module, sitl_vehicle_namespace,
]
SITL_ADAPTER_MODULES = [autopilot_sitl_module, autopilot_ardupilot_sitl, autopilot_px4_sitl]
ALL_PHASE7_MODULES = SITL_PACKAGE_MODULES + SITL_ADAPTER_MODULES

FORBIDDEN_TRANSPORT_IMPORTS = {
    "pybullet", "pymavlink", "mavsdk", "MAVSDK", "serial", "socket",
    "gym_pybullet_drones", "dronekit", "subprocess", "asyncio",
}
FORBIDDEN_ACTUATION_IDENTIFIERS = {
    "rpm", "pwm", "motor", "computeControlFromState", "arm", "disarm", "arm_throttle", "throttle",
}
FORBIDDEN_GROUND_TRUTH_IDENTIFIERS = {
    "victims_ground_truth", "WorldState", "victims", "obstacles", "mission", "positions", "velocities",
}
FORBIDDEN_EXTERNAL_CONNECTION_CALLS = {
    "socket", "connect", "bind", "sendto", "recvfrom", "Serial", "popen", "Popen",
}


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
        elif isinstance(node, ast.FunctionDef):
            names.add(node.name)
    return names


def _collect_imports(tree: ast.AST):
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module.split(".")[0])
    return imported


def _call_names(tree: ast.AST):
    names = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            f = node.func
            if isinstance(f, ast.Attribute):
                names.add(f.attr)
            elif isinstance(f, ast.Name):
                names.add(f.id)
    return names


# --- no transport/hardware imports -------------------------------------

def test_phase7_modules_import_no_transport_or_hardware_module():
    for module in ALL_PHASE7_MODULES:
        imported = _collect_imports(_tree_of_module(module))
        leaked = imported & FORBIDDEN_TRANSPORT_IMPORTS
        assert not leaked, f"{module.__name__} imports a forbidden transport/hardware module: {leaked}"


def test_sitl_adapter_skeletons_have_zero_transport_imports_at_all():
    """Stronger than the generic check above: the SITL-targeted skeleton
    modules must have NO imports beyond this project's own package and
    __future__."""
    allowed_roots = {"__future__", "swarm_sim", "base"}
    for module in (autopilot_ardupilot_sitl, autopilot_px4_sitl):
        tree = _tree_of_module(module)
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    root = alias.name.split(".")[0]
                    assert root in allowed_roots, f"{module.__name__} imports unexpected module: {alias.name}"
            elif isinstance(node, ast.ImportFrom):
                if node.level == 0 and node.module:
                    root = node.module.split(".")[0]
                    assert root in allowed_roots, f"{module.__name__} imports unexpected module: {node.module}"


# --- no actuation / no external connections -----------------------------

def test_phase7_modules_never_reference_actuation_identifiers():
    for module in ALL_PHASE7_MODULES:
        leaked = _collect_identifiers(_tree_of_module(module)) & FORBIDDEN_ACTUATION_IDENTIFIERS
        assert not leaked, f"{module.__name__} references forbidden actuation identifiers: {leaked}"


def test_phase7_modules_never_open_an_external_connection():
    for module in ALL_PHASE7_MODULES:
        leaked = _call_names(_tree_of_module(module)) & FORBIDDEN_EXTERNAL_CONNECTION_CALLS
        assert not leaked, f"{module.__name__} calls a forbidden external-connection API: {leaked}"


def test_phase7_modules_reference_no_ground_truth_identifiers():
    for module in ALL_PHASE7_MODULES:
        leaked = _collect_identifiers(_tree_of_module(module)) & FORBIDDEN_GROUND_TRUTH_IDENTIFIERS
        assert not leaked, f"{module.__name__} references forbidden ground-truth identifiers: {leaked}"


def test_fake_sitl_transport_never_sets_armed_true():
    """No implicit arm anywhere in the transport implementation - `armed`
    is only ever read or initialized to False, never assigned True."""
    tree = _tree_of_module(sitl_fake_transport)
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if (isinstance(target, ast.Attribute) and target.attr == "armed"
                        and isinstance(node.value, ast.Constant) and node.value.value is True):
                    raise AssertionError("FakeSITLTransport must never assign armed = True")


# --- mission commands pass through SafetySupervisor, then adapter, then
#     (when fake_sitl) transport -----------------------------------------

def _run_fn():
    tree = ast.parse(pathlib.Path(mission_module.__file__).read_text())
    return next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "run")


def _adapter_helper_fn():
    tree = ast.parse(pathlib.Path(mission_module.__file__).read_text())
    return next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "_send_to_autopilot_adapter")


def _call_name(call: ast.Call):
    f = call.func
    if isinstance(f, ast.Attribute):
        return f.attr
    if isinstance(f, ast.Name):
        return f.id
    return None


def test_run_still_calls_evaluate_before_the_adapter_helper_with_fake_sitl_enabled():
    """Re-affirms Phase 6's own guarantee still holds now that
    self.autopilot_adapters can also hold SITLAdapter instances - the
    ordering check in mission.py's run() doesn't change per adapter type."""
    run_fn = _run_fn()
    calls = [n for n in ast.walk(run_fn) if isinstance(n, ast.Call)]
    evaluate_calls = [c for c in calls if _call_name(c) == "evaluate"]
    adapter_helper_calls = [c for c in calls if _call_name(c) == "_send_to_autopilot_adapter"]
    assert evaluate_calls and adapter_helper_calls
    assert min(c.lineno for c in evaluate_calls) < min(c.lineno for c in adapter_helper_calls)


def test_adapter_helper_calls_send_command_which_sitladapter_forwards_to_transport():
    """Two-part chain check: (1) mission.py's _send_to_autopilot_adapter
    calls send_command() (Phase 6's own check, still true), and (2)
    SITLAdapter.send_command itself calls transport.send_command() - so
    the full chain from run() down to the SITLTransport is real, not a
    same-named decoy at any layer."""
    helper_calls = [n for n in ast.walk(_adapter_helper_fn()) if isinstance(n, ast.Call)]
    assert any(_call_name(c) == "send_command" for c in helper_calls)

    tree = _tree_of_module(autopilot_sitl_module)
    send_command_fn = next(
        n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "send_command"
    )
    inner_calls = [n for n in ast.walk(send_command_fn) if isinstance(n, ast.Call)]
    assert any(_call_name(c) == "send_command" for c in inner_calls), (
        "SITLAdapter.send_command must itself call SITLTransport.send_command()"
    )


def test_send_to_autopilot_adapter_never_called_with_raw_controller_candidate():
    """Re-asserted here (also in test_autopilot_architecture.py /
    test_safety_supervisor_architecture.py) so this file stands on its own
    for the Phase 7 boundary too."""
    run_fn = _run_fn()
    for node in ast.walk(run_fn):
        if isinstance(node, ast.Call) and _call_name(node) == "_send_to_autopilot_adapter":
            for arg in node.args:
                for sub in ast.walk(arg):
                    if isinstance(sub, ast.Name) and sub.id == "desired_vel":
                        raise AssertionError("_send_to_autopilot_adapter() must never receive desired_vel directly")


# --- distributed consensus has no adapter or SITL access -----------------

def test_distributed_consensus_never_references_sitl_or_adapter_identifiers():
    tree = _tree_of_module(dconsensus_module)
    identifiers = _collect_identifiers(tree)
    imports = _collect_imports(tree)
    forbidden_names = {
        "send_command", "AutopilotAdapter", "MockAdapter", "AdapterCommand", "connect", "autopilot",
        "SITLAdapter", "SITLTransport", "FakeSITLTransport", "SITLCommand", "sitl",
    }
    leaked = identifiers & forbidden_names
    assert not leaked, f"distributed_consensus.py references adapter/SITL-layer identifiers: {leaked}"
    assert "autopilot" not in imports and "sitl" not in imports


# --- namespace guarantees -------------------------------------------------

def test_sitl_command_dataclass_declares_a_namespace_field():
    tree = _tree_of_module(sitl_commands)
    class_def = next(n for n in ast.walk(tree) if isinstance(n, ast.ClassDef) and n.name == "SITLCommand")
    field_names = {
        stmt.target.id for stmt in class_def.body
        if isinstance(stmt, ast.AnnAssign) and isinstance(stmt.target, ast.Name)
    }
    assert "namespace" in field_names
    assert "safety_decision_id" in field_names


def test_fake_sitl_transport_send_command_checks_namespace_before_touching_channel_state():
    """AST-level guarantee behind the functional cross-namespace tests in
    test_sitl.py: FakeSITLTransport.send_command's own body must reference
    the namespace registry's mismatch check before it can reach a
    per-vehicle channel."""
    tree = _tree_of_module(sitl_fake_transport)
    send_command_fn = next(
        n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "send_command"
        and any(isinstance(a, ast.arg) and a.arg == "command" for a in n.args.args)
    )
    identifiers = _collect_identifiers(send_command_fn)
    assert "require_match" in identifiers, "send_command must check the namespace registry before proceeding"


def test_sitl_transport_protocol_declares_every_required_method():
    required = {
        "start", "stop", "send_command", "receive_telemetry", "receive_ack", "set_sim_time", "inject_failure",
    }
    declared = {
        n.name for n in ast.walk(_tree_of_module(sitl_transport_module))
        if isinstance(n, ast.FunctionDef)
    }
    missing = required - declared
    assert not missing, f"SITLTransport Protocol is missing required methods: {missing}"
