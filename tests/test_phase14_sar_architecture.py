"""Architecture checks for the Phase 14 SAR mission modules - see
docs/PHASE14_SAR_WEBOTS.md. AST-based structural proof mirroring Phase
10/11/12/13's own patterns.
"""
import ast
import ipaddress
import os
import pathlib
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"))

import swarm_sim.sar_mission as sar_mission  # noqa: E402
import swarm_sim.sar_search as sar_search  # noqa: E402
import swarm_sim.sar_world as sar_world  # noqa: E402

import run_phase14_sar_mission as phase14  # noqa: E402

FORBIDDEN_MODULE_REFERENCES = {
    "pybullet", "flightgear_bridge", "telemetry_mapping", "distributed_consensus", "recruitment", "consensus",
}
FORBIDDEN_SWARM_IDENTIFIERS = {
    "FloodSearchMission", "DistributedConsensus", "DroneConsensusNode", "RecruitmentBoard", "ConsensusBoard",
    "CandidateCommand6", "num_drones", "VEHICLE_IDS",
}


def _tree(module) -> ast.AST:
    return ast.parse(pathlib.Path(module.__file__).read_text())


def _source(module) -> str:
    return pathlib.Path(module.__file__).read_text()


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


def _fn(module, name: str):
    return next(n for n in ast.walk(_tree(module)) if isinstance(n, ast.FunctionDef) and n.name == name)


def _cls(module, name: str):
    return next(n for n in ast.walk(_tree(module)) if isinstance(n, ast.ClassDef) and n.name == name)


# ==========================================================================
# 1. hidden ground-truth isolation - sar_search.py must never touch it
# ==========================================================================

def test_sar_search_never_imports_sar_world_private_or_ground_truth_names():
    source = _source(sar_search)
    for forbidden in ("_victims_gt", "VictimGroundTruth", "victims_xy"):
        assert forbidden not in source, f"sar_search.py references {forbidden!r} - a planner-facing module must not"


def test_sar_search_module_has_no_rng_dependency():
    """The search pattern is a pure, deterministic function of the search
    area/altitude/spacing - no randomness, no victim knowledge."""
    imports = _collect_imports(_tree(sar_search))
    assert "random" not in imports and "numpy" not in imports


def test_sar_world_victims_attribute_is_underscore_private():
    tree = _tree(sar_world)
    cls = _cls(sar_world, "SARWorld")
    assigned_attrs = set()
    for node in ast.walk(cls):
        if isinstance(node, ast.Attribute) and isinstance(node.ctx, ast.Store) and isinstance(node.value, ast.Name) \
                and node.value.id == "self":
            assigned_attrs.add(node.attr)
    victim_holding = [a for a in assigned_attrs if "victim" in a.lower() and not a.startswith("_")]
    # every non-underscore attribute with "victim" in its name must be a
    # count/id-tuple property, never the raw ground-truth list itself
    assert "_victims_gt" in assigned_attrs
    assert "victims_gt" not in assigned_attrs   # the exact non-underscore name must not exist
    assert victim_holding == [] or all("count" in a or "ids" in a for a in victim_holding)


def test_sar_mission_never_reads_sar_world_private_ground_truth_attribute():
    source = _source(sar_mission)
    assert "_victims_gt" not in source
    assert "world.victims_xy" not in source


# ==========================================================================
# 2. command path: raw candidates cannot bypass SafetySupervisor
# ==========================================================================

def test_tick_calls_supervisor_evaluate_before_building_any_adapter_command():
    source = _source(sar_mission)
    tick_fn = _fn(sar_mission, "tick")
    tick_source = ast.get_source_segment(source, tick_fn)
    evaluate_pos = tick_source.index("self.supervisor.evaluate(")
    adapter_command_pos = tick_source.index("AdapterCommand(command=decision.filtered_command")
    assert evaluate_pos < adapter_command_pos


def test_tick_never_calls_send_command_itself():
    """SARMission.tick() must only ever RETURN an AdapterCommand - sending
    it is the caller's job (run_sar_mission_offline / the live script) -
    see module docstring for why."""
    source = _source(sar_mission)
    tick_fn = _fn(sar_mission, "tick")
    tick_source = ast.get_source_segment(source, tick_fn)
    assert ".send_command(" not in tick_source


def test_adapter_command_only_ever_built_from_decision_filtered_command():
    source = _source(sar_mission)
    tree = ast.parse(source)
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == "AdapterCommand":
            for kw in node.keywords:
                if kw.arg == "command":
                    assert isinstance(kw.value, ast.Attribute) and kw.value.attr == "filtered_command", (
                        "AdapterCommand(command=...) must always be built from a SafetyDecision.filtered_command, "
                        "never a raw candidate"
                    )


def test_live_script_never_sends_a_raw_candidate_to_the_adapter():
    source = _source(phase14)
    assert "adapter.send_command(candidate" not in source
    assert "adapter.send_command(tick_result.adapter_command)" in source


# ==========================================================================
# 3. no six-drone / swarm / consensus / FlightGear / PyBullet
# ==========================================================================

def test_no_swarm_module_references():
    for module in (sar_mission, sar_search, sar_world, phase14):
        imports = _collect_imports(_tree(module))
        identifiers = _collect_identifiers(_tree(module))
        assert not (imports & FORBIDDEN_MODULE_REFERENCES), f"{module.__name__} imports a forbidden module"
        assert not (identifiers & FORBIDDEN_SWARM_IDENTIFIERS), f"{module.__name__} references a forbidden identifier"


def test_no_pybullet_or_flightgear_source_text():
    for module in (sar_mission, sar_search, sar_world, phase14):
        source = _source(module)
        assert "pybullet" not in source.lower()
        assert "flightgear" not in source.lower()


def test_single_vehicle_construction_only():
    for module in (sar_mission, phase14):
        tree = _tree(module)
        for node in ast.walk(tree):
            if isinstance(node, ast.Call):
                fn = node.func
                name = fn.id if isinstance(fn, ast.Name) else (fn.attr if isinstance(fn, ast.Attribute) else None)
                if name in ("ArduPilotSITLTransport", "FakeSITLTransport"):
                    for kw in node.keywords:
                        if kw.arg in ("endpoints",) and isinstance(kw.value, ast.Dict):
                            assert len(kw.value.keys) == 1
                        if kw.arg == "vehicle_ids" and isinstance(kw.value, ast.Tuple):
                            assert len(kw.value.elts) == 1
                    for arg in node.args:
                        if isinstance(arg, ast.Dict):
                            assert len(arg.keys) == 1


# ==========================================================================
# 4. live mode: default never arms, all three flags required, hard caps
# ==========================================================================

def test_default_cli_args_never_select_live_mode():
    args = phase14.build_parser().parse_args([])
    assert args.webots_only is False
    assert args.allow_webots_arm_test is False
    assert args.confirm_webots_flight_test is False
    assert args.dry_run is False
    assert args.offline is False


def test_main_dispatch_requires_all_three_flags_together():
    source = _source(phase14)
    main_fn = _fn(phase14, "main")
    main_source = ast.get_source_segment(source, main_fn)
    assert "args.webots_only and args.allow_webots_arm_test and args.confirm_webots_flight_test" in main_source


def test_main_falls_back_to_dry_run():
    source = _source(phase14)
    main_fn = _fn(phase14, "main")
    main_source = ast.get_source_segment(source, main_fn)
    assert "run_dry_run(args)" in main_source


def test_offline_mode_is_reachable_without_any_arm_flag():
    args = phase14.build_parser().parse_args(["--offline"])
    assert args.webots_only is False and args.allow_webots_arm_test is False


def test_hard_caps_are_conservative():
    assert phase14.HARD_MAX_ALTITUDE_M <= 2.0
    assert phase14.HARD_MAX_SPEED_MPS <= 0.25
    assert phase14.HARD_MISSION_DURATION_S <= 90.0


def test_default_search_altitude_and_duration_within_hard_caps():
    args = phase14.build_parser().parse_args([])
    assert args.search_altitude <= phase14.HARD_MAX_ALTITUDE_M
    assert args.duration <= phase14.HARD_MISSION_DURATION_S
    assert args.max_speed <= phase14.HARD_MAX_SPEED_MPS


def test_gate_checks_namespace_and_all_three_flags():
    source = _source(phase14)
    gate_fn = _fn(phase14, "_check_sar_live_gate")
    gate_source = ast.get_source_segment(source, gate_fn)
    assert "webots_only" in gate_source
    assert "allow_webots_arm_test" in gate_source
    assert "REQUIRED_NAMESPACE" in gate_source or "sitl/drone0" in gate_source


# ==========================================================================
# 5. no hardcoded non-local endpoint
# ==========================================================================

def test_default_connection_is_loopback():
    args = phase14.build_parser().parse_args([])
    assert args.connection.split(":")[1] in ("127.0.0.1", "localhost")


def test_no_hardcoded_non_loopback_ip_literal():
    ip_pattern = re.compile(r"^\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}$")
    for module in (sar_mission, sar_search, sar_world, phase14):
        for node in ast.walk(_tree(module)):
            if isinstance(node, ast.Constant) and isinstance(node.value, str) and ip_pattern.match(node.value):
                ip = ipaddress.ip_address(node.value)
                assert ip.is_loopback or ip.is_private or ip.is_unspecified, (
                    f"{module.__name__} has an unexpected non-local hardcoded IP literal: {node.value!r}"
                )


def test_no_serial_or_radio_endpoint_default():
    args = phase14.build_parser().parse_args([])
    assert "COM" not in args.connection
    assert "/dev/" not in args.connection


# ==========================================================================
# 6. no process spawning/killing, no param_set, no force-arm
# ==========================================================================

def test_no_subprocess_module_imported():
    for module in (sar_mission, sar_search, sar_world, phase14):
        assert "subprocess" not in _collect_imports(_tree(module))


def test_never_calls_param_set_anywhere():
    for module in (sar_mission, phase14):
        identifiers = _collect_identifiers(_tree(module))
        assert "param_set_send" not in identifiers
        assert "mav_param_set_send" not in identifiers


def test_never_forces_arm_with_the_force_magic_value():
    for module in (sar_mission, phase14):
        for node in ast.walk(_tree(module)):
            if isinstance(node, ast.Constant) and node.value == 21196:
                raise AssertionError(f"{module.__name__} references the force-arm magic value (21196)")


# ==========================================================================
# 7. reuse, not duplication - Phase 10/12/13 remain unaffected
# ==========================================================================

def test_live_script_reuses_phase12_and_phase13_helpers_instead_of_duplicating():
    source = _source(phase14)
    assert "from run_phase12_webots_smoke_test import" in source
    assert "from run_phase13_webots_flight_test import" in source
    assert "from run_phase10_sitl_flight_test import" in source
    # never imports Phase 13's own top-level run function/main - only its small helpers
    imported_names = set()
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.ImportFrom) and node.module == "run_phase13_webots_flight_test":
            imported_names.update(alias.name for alias in node.names)
    assert "run_webots_flight_test" not in imported_names
    assert "main" not in imported_names


def test_phase13_module_unaffected_by_phase14_existing():
    import run_phase13_webots_flight_test as phase13_module
    tree = ast.parse(pathlib.Path(phase13_module.__file__).read_text())
    forbidden = {"SARMission", "sar_mission", "sar_world", "sar_search"}
    assert not (_collect_identifiers(tree) & forbidden)
    assert not (_collect_imports(tree) & {"sar_mission", "sar_world", "sar_search"})


def test_phase10_and_phase12_modules_unaffected_by_phase14_existing():
    import run_phase10_sitl_flight_test as phase10_module
    import run_phase12_webots_smoke_test as phase12_module
    for module in (phase10_module, phase12_module):
        tree = ast.parse(pathlib.Path(module.__file__).read_text())
        assert not (_collect_imports(tree) & {"sar_mission", "sar_world", "sar_search"})


def test_ardupilot_transport_module_still_never_references_arm_or_takeoff():
    import swarm_sim.sitl.ardupilot_transport as transport_module
    tree = ast.parse(pathlib.Path(transport_module.__file__).read_text())
    identifiers = _collect_identifiers(tree)
    assert "MAV_CMD_COMPONENT_ARM_DISARM" not in identifiers
    assert "MAV_CMD_NAV_TAKEOFF" not in identifiers


def test_fake_sitl_transport_still_importable_and_functional():
    from swarm_sim.sitl.fake_transport import FakeSITLTransport
    t = FakeSITLTransport(vehicle_ids=("d",))
    assert t.start().success
