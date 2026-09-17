"""Phase 6 architecture checks - see docs/PHASE6_AUTOPILOT_ADAPTERS.md.
Required items covered: no transport/hardware imports, no direct
actuation calls, mission commands pass through SafetySupervisor then
AutopilotAdapter, distributed consensus has no direct adapter access, no
ground-truth access in the adapter layer.
"""
import ast
import pathlib

import swarm_sim.distributed_consensus as dconsensus_module
import swarm_sim.mission as mission_module
from swarm_sim import autopilot as autopilot_pkg
from swarm_sim.autopilot import base as autopilot_base
from swarm_sim.autopilot import frames as autopilot_frames
from swarm_sim.autopilot import mock as autopilot_mock
from swarm_sim.autopilot import ardupilot as autopilot_ardupilot
from swarm_sim.autopilot import px4 as autopilot_px4
from swarm_sim.autopilot import types as autopilot_types

AUTOPILOT_MODULES = [autopilot_base, autopilot_frames, autopilot_mock, autopilot_ardupilot, autopilot_px4, autopilot_types]

FORBIDDEN_TRANSPORT_IMPORTS = {
    "pybullet", "pymavlink", "mavsdk", "MAVSDK", "serial", "socket",
    "gym_pybullet_drones", "dronekit",
}
FORBIDDEN_ACTUATION_IDENTIFIERS = {
    "rpm", "pwm", "motor", "computeControlFromState", "arm", "disarm", "arm_throttle",
}
FORBIDDEN_GROUND_TRUTH_IDENTIFIERS = {
    "victims_ground_truth", "WorldState", "victims", "obstacles", "mission", "positions", "velocities",
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
    return names


def _collect_imports(tree: ast.AST):
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module.split(".")[0])
    return imported


# --- no transport/hardware imports ------------------------------------------

def test_autopilot_package_imports_no_transport_or_hardware_module():
    for module in AUTOPILOT_MODULES:
        imported = _collect_imports(_tree_of_module(module))
        leaked = imported & FORBIDDEN_TRANSPORT_IMPORTS
        assert not leaked, f"{module.__name__} imports a forbidden transport/hardware module: {leaked}"


def test_ardupilot_and_px4_skeletons_have_zero_transport_imports_at_all():
    """Stronger than the generic check above: the two named skeleton
    modules must have NO imports beyond this project's own package and
    __future__ - not even something transport-adjacent this project
    hasn't thought to forbid by name."""
    allowed_roots = {"__future__", "swarm_sim", "base", "types"}  # relative imports resolve with a leading dot
    for module in (autopilot_ardupilot, autopilot_px4):
        tree = _tree_of_module(module)
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    root = alias.name.split(".")[0]
                    assert root in allowed_roots, f"{module.__name__} imports unexpected module: {alias.name}"
            elif isinstance(node, ast.ImportFrom):
                # relative imports (level > 0, e.g. "from .base import ...")
                # are always same-package and therefore fine.
                if node.level == 0 and node.module:
                    root = node.module.split(".")[0]
                    assert root in allowed_roots, f"{module.__name__} imports unexpected module: {node.module}"


# --- no direct actuation calls ----------------------------------------------

def test_autopilot_package_never_references_actuation_identifiers():
    for module in AUTOPILOT_MODULES:
        leaked = _collect_identifiers(_tree_of_module(module)) & FORBIDDEN_ACTUATION_IDENTIFIERS
        assert not leaked, f"{module.__name__} references forbidden actuation identifiers: {leaked}"


# --- ground-truth isolation --------------------------------------------------

def test_autopilot_package_references_no_ground_truth_identifiers():
    for module in AUTOPILOT_MODULES:
        leaked = _collect_identifiers(_tree_of_module(module)) & FORBIDDEN_GROUND_TRUTH_IDENTIFIERS
        assert not leaked, f"{module.__name__} references forbidden ground-truth identifiers: {leaked}"


# --- mission commands pass through SafetySupervisor then AutopilotAdapter --

def _run_fn():
    tree = _tree_of_module(mission_module)
    return next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "run")


def _call_name(call: ast.Call):
    f = call.func
    if isinstance(f, ast.Attribute):
        return f.attr
    if isinstance(f, ast.Name):
        return f.id
    return None


def _adapter_helper_fn():
    tree = _tree_of_module(mission_module)
    return next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "_send_to_autopilot_adapter")


def test_run_calls_evaluate_then_the_adapter_helper_which_itself_calls_send_command():
    """send_command() is called inside mission.py's _send_to_autopilot_adapter
    helper, not inline in run() - so the ordering guarantee is checked in
    two parts: (1) run() calls evaluate() before it calls
    _send_to_autopilot_adapter(), and (2) _send_to_autopilot_adapter()
    itself really does call send_command() (so the chain from run() is
    real, not just a same-named decoy)."""
    run_fn = _run_fn()
    calls = [n for n in ast.walk(run_fn) if isinstance(n, ast.Call)]
    evaluate_calls = [c for c in calls if _call_name(c) == "evaluate"]
    adapter_helper_calls = [c for c in calls if _call_name(c) == "_send_to_autopilot_adapter"]
    assert evaluate_calls, "run() must call SafetySupervisor.evaluate()"
    assert adapter_helper_calls, "run() must call _send_to_autopilot_adapter() (mock-adapter path)"
    assert min(c.lineno for c in evaluate_calls) < min(c.lineno for c in adapter_helper_calls)

    helper_calls = [n for n in ast.walk(_adapter_helper_fn()) if isinstance(n, ast.Call)]
    assert any(_call_name(c) == "send_command" for c in helper_calls), (
        "_send_to_autopilot_adapter() must itself call AutopilotAdapter.send_command()"
    )


def test_send_to_autopilot_adapter_is_never_called_with_the_raw_controller_candidate():
    """run()'s call(s) to _send_to_autopilot_adapter() must never pass
    desired_vel (the raw SwarmController candidate) as an argument - only
    decision.filtered_command (an ACCEPTED SafetyDecision's own output).
    Re-asserted here (also covered in
    test_safety_supervisor_architecture.py) so this file stands on its
    own as the Phase 6 adapter-boundary check."""
    run_fn = _run_fn()
    for node in ast.walk(run_fn):
        if isinstance(node, ast.Call) and _call_name(node) == "_send_to_autopilot_adapter":
            for arg in node.args:
                for sub in ast.walk(arg):
                    if isinstance(sub, ast.Name) and sub.id == "desired_vel":
                        raise AssertionError("_send_to_autopilot_adapter() must never receive desired_vel directly")


def test_to_velocity_command_still_never_receives_desired_vel_directly():
    """Re-affirms the Phase 4 guarantee still holds after Phase 6's
    adapter boundary was added."""
    run_fn = _run_fn()
    for node in ast.walk(run_fn):
        if isinstance(node, ast.Call) and _call_name(node) == "to_velocity_command":
            arg_names = [a.id for a in node.args if isinstance(a, ast.Name)]
            assert "desired_vel" not in arg_names


# --- distributed consensus has no direct adapter access ---------------------

def test_distributed_consensus_never_references_autopilot_adapter():
    tree = _tree_of_module(dconsensus_module)
    identifiers = _collect_identifiers(tree)
    imports = _collect_imports(tree)
    forbidden_names = {"send_command", "AutopilotAdapter", "MockAdapter", "AdapterCommand", "connect", "autopilot"}
    leaked = identifiers & forbidden_names
    assert not leaked, f"distributed_consensus.py references adapter-layer identifiers: {leaked}"
    assert "autopilot" not in imports, "distributed_consensus.py must not import the autopilot package"


# --- interface shape ---------------------------------------------------------

def test_autopilot_adapter_protocol_declares_every_required_method():
    required = {
        "connect", "disconnect", "read_telemetry", "send_command", "set_mode",
        "request_land", "request_return_to_launch", "abort", "is_command_fresh",
    }
    declared = {
        n.name for n in ast.walk(_tree_of_module(autopilot_base))
        if isinstance(n, ast.FunctionDef)
    }
    missing = required - declared
    assert not missing, f"AutopilotAdapter Protocol is missing required methods: {missing}"
