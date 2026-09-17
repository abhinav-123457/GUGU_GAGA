"""Phase 5 architecture checks - see docs/PHASE5_DISTRIBUTED_CONSENSUS.md.
Required items covered: no direct peer-state access (2), ground-truth
isolation (11/20), no direct actuation path (12/21), all commands still
pass through SafetySupervisor (12/22).
"""
import ast
import pathlib

import swarm_sim.distributed_consensus as dconsensus_module
import swarm_sim.mission as mission_module

FORBIDDEN_GROUND_TRUTH_IDENTIFIERS = {
    "victims_ground_truth", "WorldState", "victims", "obstacles", "mission",
    "positions", "velocities",
}
FORBIDDEN_ACTUATION_IDENTIFIERS = {"rpm", "pwm", "motor", "computeControlFromState"}
FORBIDDEN_ACTUATION_IMPORTS = {"pybullet", "p", "mission", "run_mission", "DSLPIDControl", "gym_pybullet_drones"}
PRIVATE_NODE_ATTRS = {"_evidence", "_confirmed_ids", "_relayed_ids", "_own_seq"}


def _source_path(module) -> pathlib.Path:
    return pathlib.Path(module.__file__)


def _tree(module) -> ast.AST:
    return ast.parse(_source_path(module).read_text())


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


# --- 11/20. ground-truth isolation ------------------------------------------

def test_distributed_consensus_references_no_ground_truth_identifiers():
    tree = _tree(dconsensus_module)
    leaked = _collect_identifiers(tree) & FORBIDDEN_GROUND_TRUTH_IDENTIFIERS
    assert not leaked, f"distributed_consensus.py references forbidden ground-truth identifiers: {leaked}"


def test_distributed_consensus_node_methods_take_no_ground_truth_parameter():
    tree = _tree(dconsensus_module)
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef):
            param_names = {a.arg for a in node.args.args}
            leaked = param_names & FORBIDDEN_GROUND_TRUTH_IDENTIFIERS
            assert not leaked, f"{node.name}() takes a forbidden ground-truth parameter: {leaked}"


# --- 12/21. no direct actuation path -----------------------------------------

def test_distributed_consensus_imports_no_pybullet_or_actuation_module():
    tree = _tree(dconsensus_module)
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module.split(".")[0])
    leaked = imported & FORBIDDEN_ACTUATION_IMPORTS
    assert not leaked, f"distributed_consensus.py imports a forbidden actuation-path module: {leaked}"


def test_distributed_consensus_never_references_motor_pwm_rpm_identifiers():
    tree = _tree(dconsensus_module)
    leaked = _collect_identifiers(tree) & FORBIDDEN_ACTUATION_IDENTIFIERS
    assert not leaked, f"distributed_consensus.py references forbidden actuation identifiers: {leaked}"


def _method_source(module, method_name):
    tree = _tree(module)
    return next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == method_name)


def test_distributed_submit_and_resolve_methods_never_touch_actuation():
    """mission.py's two Phase 5 entry points (_submit_detections_distributed,
    _resolve_confirmed_detections_distributed) must only ever call
    network/board/diagnostics/consensus-adjacent methods - never
    to_velocity_command, computeControlFromState, or reach the PID/motor
    layer directly."""
    forbidden_calls = {"to_velocity_command", "computeControlFromState"}
    for method_name in ("_submit_detections_distributed", "_resolve_confirmed_detections_distributed"):
        fn = _method_source(mission_module, method_name)
        for node in ast.walk(fn):
            if isinstance(node, ast.Call):
                f = node.func
                name = f.attr if isinstance(f, ast.Attribute) else (f.id if isinstance(f, ast.Name) else None)
                assert name not in forbidden_calls, f"{method_name}() must not call {name}()"


# --- 2. no direct peer-state access -----------------------------------------

def test_no_cross_node_access_to_another_drones_private_state():
    """The only attribute access to a DroneConsensusNode's private storage
    (_evidence/_confirmed_ids/_relayed_ids/_own_seq) allowed anywhere in
    distributed_consensus.py is `self.<attr>` from inside the node's own
    methods - never `some_other_object.<attr>` (which is exactly what
    "one drone reading another drone's private consensus state" would
    look like in code)."""
    tree = _tree(dconsensus_module)
    violations = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Attribute) and node.attr in PRIVATE_NODE_ATTRS:
            is_self_access = isinstance(node.value, ast.Name) and node.value.id == "self"
            if not is_self_access:
                violations.append((node.attr, ast.dump(node.value)))
    assert not violations, f"cross-object access to private node state found: {violations}"


def test_distributed_consensus_orchestrator_only_calls_public_node_methods():
    """DistributedConsensus (the one object that spans multiple drones)
    must reach each DroneConsensusNode only through its public API - i.e.
    no attribute access on a `self.nodes[...]` / node lookup result
    targets a name in PRIVATE_NODE_ATTRS. Covered by the whole-module scan
    above; this test additionally pins that DistributedConsensus itself
    contains zero such accesses, so a future edit inside just that class
    cannot quietly reintroduce one."""
    tree = _tree(dconsensus_module)
    class_node = next(n for n in ast.walk(tree) if isinstance(n, ast.ClassDef) and n.name == "DistributedConsensus")
    for node in ast.walk(class_node):
        if isinstance(node, ast.Attribute):
            assert node.attr not in PRIVATE_NODE_ATTRS, (
                f"DistributedConsensus directly accesses private node state: {node.attr}"
            )


# --- 12/22. all commands still pass through SafetySupervisor ---------------

def test_run_still_routes_every_candidate_through_safety_evaluate():
    """Re-affirms the Phase 4 guarantee (test_safety_supervisor_architecture.py)
    still holds after Phase 5's changes to run(): evaluate() is still
    called, and it is still called before to_velocity_command()."""
    tree = _tree(mission_module)
    run_fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "run")
    calls = [n for n in ast.walk(run_fn) if isinstance(n, ast.Call)]

    def call_name(call):
        f = call.func
        if isinstance(f, ast.Attribute):
            return f.attr
        if isinstance(f, ast.Name):
            return f.id
        return None

    evaluate_calls = [c for c in calls if call_name(c) == "evaluate"]
    to_velocity_calls = [c for c in calls if call_name(c) == "to_velocity_command"]
    assert evaluate_calls, "run() must still call SafetySupervisor.evaluate()"
    assert to_velocity_calls, "run() must still call SpeedController.to_velocity_command()"
    assert min(c.lineno for c in evaluate_calls) < min(c.lineno for c in to_velocity_calls)


def test_distributed_consensus_and_board_never_feed_speed_control_or_pid_directly():
    """Same guarantee as test_safety_supervisor_architecture.py's
    consensus/board check, re-verified after adding self.distributed_consensus:
    neither self.consensus, self.board, nor self.distributed_consensus may
    ever be referenced as an argument to to_velocity_command() or
    computeControlFromState() inside run()."""
    tree = _tree(mission_module)
    run_fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "run")
    watched = {"consensus", "board", "distributed_consensus"}
    for node in ast.walk(run_fn):
        if isinstance(node, ast.Call):
            f = node.func
            name = f.attr if isinstance(f, ast.Attribute) else (f.id if isinstance(f, ast.Name) else None)
            if name in ("to_velocity_command", "computeControlFromState"):
                for arg in list(node.args) + [kw.value for kw in node.keywords]:
                    for sub in ast.walk(arg):
                        if isinstance(sub, ast.Attribute) and sub.attr in watched:
                            raise AssertionError(
                                f"{name}() call references self.{sub.attr} directly - "
                                "a candidate must reach actuation only via the safety-evaluated safe_vel"
                            )
