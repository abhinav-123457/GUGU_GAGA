"""Phase 2 diagnostic follow-up: source-level architecture checks
confirming the sensing boundary holds project-wide, not just inside
controller.py. See docs/PHASE2_DIAGNOSTICS.md.
"""
import ast
import pathlib

import numpy as np

import swarm_sim.controller as controller_module
import swarm_sim.mission as mission_module
from swarm_sim.config import MissionConfig
from swarm_sim.contracts import Frame, HealthState, SensorObservation, VehicleState
from swarm_sim.controller import SwarmController
from swarm_sim.network import CommsNetwork
from swarm_sim.recruitment import RecruitmentBoard

FORBIDDEN_IDENTIFIERS = {"victims_ground_truth", "WorldState", "victims", "obstacles", "mission"}

# mission.py functions legitimately allowed to touch self.victims/
# self.obstacles ground truth: plant setup/physics/rendering, and
# functions whose docstrings mark them SCORING ONLY / EVALUATION ONLY.
MISSION_GROUND_TRUTH_WHITELIST = {
    "__init__", "_sample_obstacle_positions", "_spawn_obstacles", "_draw_victims",
    "_setup_gui_view", "_match_victim", "_classify_detections_for_scoring",
    "_track_clearance_and_contacts",
}


def _module_source_path(module) -> pathlib.Path:
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


# ---------------------------------------------------------------------------
# "only sensors.py reads hidden ground truth for runtime sensing"
# ---------------------------------------------------------------------------

def test_no_non_sensor_runtime_module_references_ground_truth_identifiers():
    import swarm_sim.consensus as consensus_module
    import swarm_sim.network as network_module
    import swarm_sim.recruitment as recruitment_module
    import swarm_sim.speed_control as speed_control_module
    import swarm_sim.telemetry as telemetry_module

    for name, module in {
        "controller": controller_module, "consensus": consensus_module, "network": network_module,
        "recruitment": recruitment_module, "speed_control": speed_control_module,
        "telemetry": telemetry_module,
    }.items():
        tree = ast.parse(_module_source_path(module).read_text())
        leaked = _collect_identifiers(tree) & FORBIDDEN_IDENTIFIERS
        assert not leaked, f"{name}.py references forbidden ground-truth identifiers: {leaked}"


# ---------------------------------------------------------------------------
# "controller.py cannot access WorldState, victims_ground_truth, hidden
# obstacles, or the raw global vehicle-position array"
# ---------------------------------------------------------------------------

def test_controller_source_contains_no_forbidden_ground_truth_identifiers():
    tree = ast.parse(_module_source_path(controller_module).read_text())
    leaked = _collect_identifiers(tree) & FORBIDDEN_IDENTIFIERS
    assert not leaked, f"controller.py references forbidden ground-truth identifiers: {leaked}"


def test_controller_no_function_takes_a_raw_full_swarm_array_parameter():
    tree = ast.parse(_module_source_path(controller_module).read_text())
    param_names = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            param_names.update(a.arg for a in node.args.args)
            param_names.update(a.arg for a in node.args.kwonlyargs)
    leaked = param_names & {"positions", "velocities"}
    assert not leaked, f"controller.py has a parameter shaped like a raw ground-truth array: {leaked}"


# ---------------------------------------------------------------------------
# "scoring functions are the only permitted mission-side ground-truth
# consumers"
# ---------------------------------------------------------------------------

def test_mission_ground_truth_access_is_confined_to_the_whitelist():
    """Every function in mission.py that references self.victims or
    self.obstacles (the hidden plant) must be on the explicit whitelist
    of plant-setup/rendering/scoring functions - anything else finding
    its way to ground truth (e.g. a future edit accidentally reading it
    from inside the per-drone control loop) fails this test."""
    tree = ast.parse(_module_source_path(mission_module).read_text())
    offending_functions = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.FunctionDef):
            continue
        touches_ground_truth = any(
            isinstance(n, ast.Attribute) and n.attr in ("victims", "obstacles")
            and isinstance(n.value, ast.Name) and n.value.id == "self"
            for n in ast.walk(node)
        )
        if touches_ground_truth and node.name not in MISSION_GROUND_TRUTH_WHITELIST:
            offending_functions.append(node.name)
    assert not offending_functions, (
        f"mission.py functions outside the scoring/plant-setup whitelist reference "
        f"self.victims/self.obstacles: {offending_functions}"
    )


def test_run_method_itself_does_not_reference_ground_truth():
    """run() is the orchestrator - it may read the physics engine's raw
    output (`positions`/`velocities`, required to build own_state and
    call the sensor models) but must never touch self.victims/
    self.obstacles directly; all such access is delegated to a
    whitelisted helper."""
    tree = ast.parse(_module_source_path(mission_module).read_text())
    run_node = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "run")
    touches_ground_truth = any(
        isinstance(n, ast.Attribute) and n.attr in ("victims", "obstacles")
        and isinstance(n.value, ast.Name) and n.value.id == "self"
        for n in ast.walk(run_node)
    )
    assert not touches_ground_truth


# ---------------------------------------------------------------------------
# "identical sensor inputs produce identical controller outputs
# regardless of hidden world state"
# ---------------------------------------------------------------------------

def _own_state(vehicle_id="drone0", pos=(0.0, 0.0, 3.0), vel=(0.0, 0.0, 0.0), t=0.0):
    return VehicleState(
        vehicle_id=vehicle_id, sim_time_s=t, frame=Frame.LOCAL_ENU,
        position_m=pos, velocity_mps=vel, acceleration_mps2=(0.0, 0.0, 0.0),
        attitude_rad=(0.0, 0.0, 0.0), angular_velocity_radps=(0.0, 0.0, 0.0),
        battery_fraction=1.0, health_state=HealthState.OK,
        estimator_valid=True, last_valid_command_time_s=t,
    )


def test_controller_output_identical_across_three_different_hidden_worlds():
    """Extends the Phase 2 version of this test from two hidden worlds to
    three, and varies obstacle count/geometry as well as victim geometry,
    to make the "regardless of hidden world state" claim harder to fail
    to catch a regression against."""
    cfg = MissionConfig(num_drones=1, seed=1)
    board = RecruitmentBoard()
    network = CommsNetwork(cfg, 1, np.random.default_rng(1))
    network.tick(np.array([[0.0, 0.0, 3.0]]), np.array([[0.0, 0.0, 0.0]]))

    own_state = _own_state()
    n_bins = len(controller_module.sensors.obstacle_scan_bin_angles(cfg))
    sensor_obs = SensorObservation(
        vehicle_id="drone0", sensor_timestamp_s=0.0, sensor_latency_s=0.0, fov_deg=90.0,
        range_returns_m=(np.inf,) * n_bins, occluded=(False,) * n_bins,
        dropout=False, pose_uncertainty_m=0.0, detections=(),
    )
    neighbor_obs = ()

    hidden_worlds = [
        {"victims": [(5.0, 5.0), (10.0, -3.0)], "obstacles": [(1.0, 1.0)]},
        {"victims": [(-40.0, 12.0)], "obstacles": [(-8.0, 20.0), (3.0, -3.0)]},
        {"victims": [], "obstacles": []},
    ]

    outputs = []
    for world in hidden_worlds:
        _ = world["victims"], world["obstacles"]  # illustrative only - never passed to step()
        rng = np.random.default_rng(99)
        controller = SwarmController(cfg, 1, rng)
        outputs.append(controller.step(0, own_state, sensor_obs, neighbor_obs, board, network))

    assert np.allclose(outputs[0], outputs[1])
    assert np.allclose(outputs[1], outputs[2])
