"""Architecture checks for Phase 15D - see
docs/PHASE15_SWARM124_LANGOSTINO_INTEGRATION.md. AST-based structural
proof mirroring Phase 10-14's own pattern (tests/test_phase14_sar_architecture.py):
these properties are checked by inspecting the actual source/imports, not
just documented in prose.
"""
from __future__ import annotations

import ast
import os
import pathlib
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"))

import swarm_sim.external.swarm124_policy_adapter as swarm124_policy_adapter  # noqa: E402
import swarm_sim.external_policy_mission as external_policy_mission  # noqa: E402

import run_phase15d_external_policy_mission as phase15d  # noqa: E402

_FORBIDDEN_TRANSPORT_MODULES = {"pymavlink", "mavutil", "serial", "pyserial", "socket"}
_FORBIDDEN_ARM_TAKEOFF_IDENTIFIERS = {
    "MAV_CMD_COMPONENT_ARM_DISARM", "MAV_CMD_NAV_TAKEOFF", "component_arm_disarm_send",
}
_FORBIDDEN_SWARM_IDENTIFIERS = {
    "FloodSearchMission", "DistributedConsensus", "DroneConsensusNode", "RecruitmentBoard", "ConsensusBoard",
    "num_drones", "VEHICLE_IDS",
}


def _tree(module) -> ast.AST:
    return ast.parse(pathlib.Path(module.__file__).read_text())


def _source(module) -> str:
    return pathlib.Path(module.__file__).read_text()


def _collect_imports(tree_or_node):
    imported = set()
    for node in ast.walk(tree_or_node):
        if isinstance(node, ast.Import):
            imported.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module.split(".")[0])
    return imported


def _collect_identifiers(tree_or_node):
    names = set()
    for node in ast.walk(tree_or_node):
        if isinstance(node, ast.Name):
            names.add(node.id)
        elif isinstance(node, ast.Attribute):
            names.add(node.attr)
    return names


def _function_source(module, fn_name: str) -> str:
    tree = _tree(module)
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == fn_name)
    return ast.get_source_segment(_source(module), fn)


# ==========================================================================
# 1. swarm_sim/external_policy_mission.py never touches a transport/MAVLink
#    directly, and never arms/takes off - it only ever builds candidates
# ==========================================================================

def test_mission_module_never_imports_a_transport_or_mavlink_library():
    imports = _collect_imports(_tree(external_policy_mission))
    assert not (imports & _FORBIDDEN_TRANSPORT_MODULES), \
        f"external_policy_mission.py imports {imports & _FORBIDDEN_TRANSPORT_MODULES} directly"


def test_mission_module_never_arms_or_takes_off():
    identifiers = _collect_identifiers(_tree(external_policy_mission))
    assert not (identifiers & _FORBIDDEN_ARM_TAKEOFF_IDENTIFIERS), \
        f"external_policy_mission.py references an arm/takeoff identifier: " \
        f"{identifiers & _FORBIDDEN_ARM_TAKEOFF_IDENTIFIERS}"


def test_tick_never_calls_send_command_itself():
    """Mirrors SARMission's own equivalent guarantee
    (swarm_sim/sar_mission.py) - tick() must return a TickResult and let
    the caller actually send the command, keeping the state machine
    testable without any transport at all."""
    tick_source = _function_source(external_policy_mission, "tick")
    assert "send_command(" not in tick_source
    assert ".mav." not in tick_source


def test_evaluate_is_called_before_any_adapter_command_is_built():
    """The ONLY place tick() constructs an AdapterCommand must be after
    self.supervisor.evaluate(...) has already run and accepted the
    decision - proving the external policy's output cannot reach an
    AdapterCommand without first passing through SafetySupervisor."""
    tick_source = _function_source(external_policy_mission, "tick")
    evaluate_idx = tick_source.index("self.supervisor.evaluate(")
    adapter_command_idx = tick_source.index("adapter_command = AdapterCommand(")
    assert evaluate_idx < adapter_command_idx
    # and the AdapterCommand must be built from the DECISION's filtered
    # command, never straight from the untrusted candidate/policy action
    assert "AdapterCommand(command=decision.filtered_command" in tick_source


def test_external_policy_action_always_goes_through_the_shared_validator():
    """Every RUNNING-state candidate must come from
    convert_swarm124_action_to_candidate() (the shared Phase 15A pipeline
    - swarm_sim/external/external_contracts.py) - this module never
    constructs a CandidateCommand directly from raw policy output."""
    tick_source = _function_source(external_policy_mission, "tick")
    assert "convert_swarm124_action_to_candidate(" in tick_source
    # a raw, un-validated Swarm124Action must never itself become the
    # candidate passed to the supervisor
    assert "candidate = action" not in tick_source


def test_rejected_external_action_returns_before_reaching_the_supervisor():
    """A rejected external action must short-circuit (return) before
    self.supervisor.evaluate() is ever reached for that tick - the
    external-contract validator is a strictly earlier, separate gate, not
    a replacement for SafetySupervisor (see
    swarm_sim/external/external_contracts.py's own module docstring)."""
    tick_source = _function_source(external_policy_mission, "tick")
    reject_idx = tick_source.index("if not external_result.accepted:")
    return_idx = tick_source.index("return TickResult(external_rejected=True)")
    evaluate_idx = tick_source.index("self.supervisor.evaluate(")
    assert reject_idx < return_idx < evaluate_idx


# ==========================================================================
# 2. scripts/run_phase15d_external_policy_mission.py reuses, never
#    redefines, Phase 10/12/13/14's own gate/arm/takeoff/land helpers
# ==========================================================================

def test_live_script_imports_gate_helpers_from_earlier_phases_not_its_own():
    imports = _collect_imports(_tree(phase15d))
    for module_name in ("run_phase10_sitl_flight_test", "run_phase12_webots_smoke_test",
                        "run_phase13_webots_flight_test", "run_phase14_sar_mission"):
        assert module_name in imports, f"expected an import from {module_name}, reusing its gate/arm/land helpers"


def test_live_script_never_redefines_reused_helper_names():
    """None of the specific helper functions this file imports from
    earlier phases may also be defined locally - that would be silent
    duplication/shadowing instead of reuse."""
    reused_names = {
        "_check_geofence_gate", "_collect_statustexts", "_read_params", "_detect_webots_installation",
        "_official_example_paths", "_port_appears_bound", "_webots_controller_port", "_ActuatorLogger",
        "_check_arming_check_gate", "_classify_altitude_result", "_request_servo_output_stream",
        "_send_command_long_and_wait_ack", "_set_mode_guided", "_check_sar_live_gate",
        "_wait_for_stable_prearm_health", "_wait_for_climb_and_vz_to_settle", "_operator_confirmed",
    }
    defined_locally = {n.name for n in ast.walk(_tree(phase15d)) if isinstance(n, ast.FunctionDef)}
    assert not (defined_locally & reused_names), \
        f"run_phase15d_external_policy_mission.py redefines a reused helper: {defined_locally & reused_names}"


def test_live_script_never_imports_sar_mission_state_machine():
    """This phase drives ExternalPolicyMission, not SARMission - it must
    never import swarm_sim.sar_mission (the SAR search state machine
    module), proving the two mission types are kept fully separate. Not a
    substring check - `run_phase14_sar_mission` (this file's own
    legitimate gate-helper import) contains "sar_mission" as a substring
    of its own filename but is NOT the forbidden module."""
    forbidden_dotted_module = "swarm_sim.sar_mission"
    for node in ast.walk(_tree(phase15d)):
        if isinstance(node, ast.ImportFrom) and node.module:
            assert node.module != forbidden_dotted_module and not node.module.endswith(".sar_mission")
        elif isinstance(node, ast.Import):
            for alias in node.names:
                assert alias.name != forbidden_dotted_module and not alias.name.endswith(".sar_mission")


def test_live_script_arm_and_takeoff_precede_any_policy_construction():
    """The ScriptedSwarm124Policy must not be constructed until AFTER a
    real arm+takeoff has already happened - proving the external policy
    cannot arm or take off a vehicle, only ever fly one that is already
    airborne under this file's own gated control."""
    source = _source(phase15d)
    live_fn_source = _function_source(phase15d, "run_live_external_policy_mission")
    arm_idx = live_fn_source.index('mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM')
    takeoff_idx = live_fn_source.index('mavutil.mavlink.MAV_CMD_NAV_TAKEOFF')
    policy_idx = live_fn_source.index("ScriptedSwarm124Policy(")
    assert arm_idx < policy_idx
    assert takeoff_idx < policy_idx


def test_live_script_never_sends_a_raw_policy_command():
    """adapter.send_command() may only ever be called with
    tick_result.adapter_command - never anything derived straight from the
    policy's own Swarm124Action."""
    live_fn_source = _function_source(phase15d, "run_live_external_policy_mission")
    assert "adapter.send_command(tick_result.adapter_command)" in live_fn_source
    assert "adapter.send_command(action" not in live_fn_source


def test_live_script_advances_the_transport_clock_every_tick():
    """Regression for the same frozen-sim-clock class of bug Phase 14
    found live (see docs/PHASE14_SAR_WEBOTS.md) - this second live tick
    loop must not re-introduce it."""
    live_fn_source = _function_source(phase15d, "run_live_external_policy_mission")
    assert "transport.set_sim_time(" in live_fn_source
    set_time_idx = live_fn_source.index("transport.set_sim_time(time.monotonic()")
    receive_telem_idx = live_fn_source.index("transport.receive_telemetry(vehicle_id)\n                now_s")
    assert set_time_idx < receive_telem_idx


# ==========================================================================
# 3. no six-drone / swarm / consensus references anywhere in this phase
# ==========================================================================

def test_no_swarm_module_references():
    for module in (external_policy_mission, swarm124_policy_adapter, phase15d):
        imports = _collect_imports(_tree(module))
        identifiers = _collect_identifiers(_tree(module))
        assert "distributed_consensus" not in imports
        assert not (identifiers & _FORBIDDEN_SWARM_IDENTIFIERS), \
            f"{module.__name__} references a forbidden swarm identifier"


def test_no_pybullet_or_flightgear_source_text():
    for module in (external_policy_mission, phase15d):
        source = _source(module).lower()
        assert "pybullet" not in source
        assert "flightgear" not in source


def test_single_vehicle_construction_only():
    for module in (external_policy_mission, phase15d):
        source = _source(module)
        assert "vehicle_ids=(config.vehicle_id,)" in source or "{vehicle_id: endpoint}" in source \
               or "drone1" not in source
        assert "drone1" not in source
        assert "drone2" not in source
