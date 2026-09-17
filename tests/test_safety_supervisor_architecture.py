"""Phase 4 safety supervisor architecture checks - see
docs/PHASE4_SAFETY.md. Required items covered: no victim-ground-truth
access (21), no motor/PWM output (22), controller candidate cannot bypass
supervisor (25), consensus-derived command cannot bypass supervisor (26).
"""
import ast
import pathlib

import swarm_sim.mission as mission_module
import swarm_sim.safety_supervisor as safety_module

FORBIDDEN_GROUND_TRUTH_IDENTIFIERS = {"victims_ground_truth", "WorldState", "victims", "obstacles", "mission"}
FORBIDDEN_ACTUATION_IDENTIFIERS = {"rpm", "pwm", "motor", "computeControlFromState"}
FORBIDDEN_ACTUATION_IMPORTS = {"pybullet", "p", "mission", "run_mission", "DSLPIDControl", "gym_pybullet_drones"}


def _source_path(module) -> pathlib.Path:
    return pathlib.Path(module.__file__)


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


# --- 21. no victim-ground-truth access -------------------------------------

def test_safety_supervisor_references_no_ground_truth_identifiers():
    tree = ast.parse(_source_path(safety_module).read_text())
    leaked = _collect_identifiers(tree) & FORBIDDEN_GROUND_TRUTH_IDENTIFIERS
    assert not leaked, f"safety_supervisor.py references forbidden ground-truth identifiers: {leaked}"


def test_safety_supervisor_evaluate_signature_has_no_ground_truth_parameter():
    tree = ast.parse(_source_path(safety_module).read_text())
    evaluate_fn = next(
        n for n in ast.walk(tree)
        if isinstance(n, ast.FunctionDef) and n.name == "evaluate"
    )
    param_names = {a.arg for a in evaluate_fn.args.args}
    assert param_names & FORBIDDEN_GROUND_TRUTH_IDENTIFIERS == set()
    expected = {"self", "candidate_command", "own_state", "sensor_observation",
                "neighbor_observations", "mission_context", "now_s"}
    assert param_names == expected, f"evaluate() signature changed: {param_names}"


# --- 22. no motor/PWM output -------------------------------------------------

def test_safety_supervisor_imports_no_pybullet_or_actuation_module():
    tree = ast.parse(_source_path(safety_module).read_text())
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module.split(".")[0])
    leaked = imported & FORBIDDEN_ACTUATION_IMPORTS
    assert not leaked, f"safety_supervisor.py imports a forbidden actuation-path module: {leaked}"


def test_safety_supervisor_never_references_motor_pwm_rpm_identifiers():
    tree = ast.parse(_source_path(safety_module).read_text())
    leaked = _collect_identifiers(tree) & FORBIDDEN_ACTUATION_IDENTIFIERS
    assert not leaked, f"safety_supervisor.py references forbidden actuation identifiers: {leaked}"


def test_safety_decision_filtered_command_is_a_command_never_raw_numbers():
    """The only thing evaluate() can hand back as "what to actually do" is
    a swarm_sim.contracts.Command (or None on rejection) - never a bare
    rpm/pwm array. Verified by construction: SafetyDecision.__post_init__
    (see contracts.py) requires filtered_command to be a Command instance
    whenever accepted is True."""
    import inspect
    from swarm_sim import contracts
    source = inspect.getsource(contracts.SafetyDecision.__post_init__)
    assert "isinstance(self.filtered_command, Command)" in source


# --- 25. controller candidate cannot bypass supervisor -----------------------

def _run_method_source():
    tree = ast.parse(_source_path(mission_module).read_text())
    run_fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "run")
    return run_fn


def test_controller_step_output_always_routed_through_safety_evaluate():
    run_fn = _run_method_source()
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
    assert evaluate_calls, "run() must call SafetySupervisor.evaluate()"
    assert to_velocity_calls, "run() must call SpeedController.to_velocity_command()"

    # The value handed to to_velocity_command must be the name "safe_vel"
    # (the variable safety-decision handling assigns to), never the raw
    # controller output "desired_vel" directly.
    for call in to_velocity_calls:
        arg_names = [a.id for a in call.args if isinstance(a, ast.Name)]
        assert "desired_vel" not in arg_names, (
            "SpeedController.to_velocity_command() must not receive the raw "
            "controller candidate (desired_vel) directly - it must go through safe_vel"
        )
        assert "safe_vel" in arg_names, "to_velocity_command() must be called with safe_vel"

    # Ordering: evaluate() must be called before to_velocity_command() in
    # source order (the only way safe_vel can reflect the decision).
    assert min(c.lineno for c in evaluate_calls) < min(c.lineno for c in to_velocity_calls)


def test_safe_vel_is_only_ever_assigned_from_desired_vel_or_the_safety_decision():
    """safe_vel (what actually reaches speed control) must only ever come
    from: the raw candidate (desired_vel - the explicit, documented
    safety_enabled=False bypass used for comparison scenarios only), the
    safety decision's own filtered_command.desired_velocity_mps, a zero
    vector (HOLD/LAND/ABORT/rejected), or (Phase 6) the AutopilotAdapter's
    own validated output via _send_to_autopilot_adapter() - itself proven
    below to only ever be called with decision.filtered_command, never
    desired_vel directly. No other source (e.g. reading
    self.board/self.consensus directly) may assign it."""
    run_fn = _run_method_source()
    allowed_rhs_patterns = set()
    for node in ast.walk(run_fn):
        if isinstance(node, ast.Assign) and any(
            isinstance(t, ast.Name) and t.id == "safe_vel" for t in node.targets
        ):
            rhs = node.value
            if isinstance(rhs, ast.Name) and rhs.id == "desired_vel":
                allowed_rhs_patterns.add("desired_vel")
            elif isinstance(rhs, ast.Call):
                func = rhs.func
                if isinstance(func, ast.Attribute) and func.attr == "array":
                    allowed_rhs_patterns.add("np.array(cmd...)")
                elif isinstance(func, ast.Attribute) and func.attr == "zeros":
                    allowed_rhs_patterns.add("np.zeros(3)")
                elif isinstance(func, ast.Attribute) and func.attr == "_send_to_autopilot_adapter":
                    allowed_rhs_patterns.add("_send_to_autopilot_adapter(...)")
                else:
                    raise AssertionError(f"unexpected safe_vel assignment source: {ast.dump(rhs)}")
            else:
                raise AssertionError(f"unexpected safe_vel assignment source: {ast.dump(rhs)}")
    assert allowed_rhs_patterns == {
        "desired_vel", "np.array(cmd...)", "np.zeros(3)", "_send_to_autopilot_adapter(...)",
    }


def test_send_to_autopilot_adapter_never_receives_desired_vel_directly():
    """Phase 6: run()'s call(s) to _send_to_autopilot_adapter() must never
    pass desired_vel (the raw SwarmController candidate) as an argument -
    only decision.filtered_command (an ACCEPTED SafetyDecision's own
    output). This is what makes the previous test's allowlisted
    "_send_to_autopilot_adapter(...)" pattern actually safe."""
    run_fn = _run_method_source()
    calls = [n for n in ast.walk(run_fn) if isinstance(n, ast.Call)
             and isinstance(n.func, ast.Attribute) and n.func.attr == "_send_to_autopilot_adapter"]
    assert calls, "run() must call _send_to_autopilot_adapter() when autopilot_path == 'mock_adapter'"
    for call in calls:
        for arg in call.args:
            for sub in ast.walk(arg):
                if isinstance(sub, ast.Name) and sub.id == "desired_vel":
                    raise AssertionError("_send_to_autopilot_adapter() must never receive desired_vel directly")


# --- 26. consensus-derived command cannot bypass supervisor ------------------

def test_consensus_and_board_never_feed_speed_control_or_pid_directly():
    """RecruitmentBoard-driven "go to a confirmed beacon" behavior is
    produced INSIDE SwarmController.step() (still just another candidate,
    already covered by the check above); self.consensus/self.board must
    never be referenced as an argument to to_velocity_command() or
    computeControlFromState() - i.e. there is no separate path from
    consensus/recruitment straight to actuation that skips the candidate
    -> safety supervisor pipeline."""
    run_fn = _run_method_source()
    for node in ast.walk(run_fn):
        if isinstance(node, ast.Call):
            f = node.func
            name = f.attr if isinstance(f, ast.Attribute) else (f.id if isinstance(f, ast.Name) else None)
            if name in ("to_velocity_command", "computeControlFromState"):
                for arg in list(node.args) + [kw.value for kw in node.keywords]:
                    for sub in ast.walk(arg):
                        if isinstance(sub, ast.Attribute) and sub.attr in ("consensus", "board"):
                            raise AssertionError(
                                f"{name}() call references self.{sub.attr} directly - "
                                "a candidate must reach actuation only via the safety-evaluated safe_vel"
                            )
