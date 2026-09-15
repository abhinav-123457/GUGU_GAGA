"""Phase 3 common-controller-contract tests spanning all four behavior
models through SwarmController itself - see docs/PHASE3_BEHAVIORS.md.
Required items covered: deterministic replay, finite outputs for every
behavior, sensor-only neighbor inputs, ground-truth isolation, behavior
output cannot bypass the controller boundary.
"""
import ast
import pathlib

import numpy as np
import pytest

import swarm_sim.behaviors.boids as boids_module
import swarm_sim.behaviors.common as common_module
import swarm_sim.behaviors.couzin as couzin_module
import swarm_sim.behaviors.olfati_saber as olfati_saber_module
import swarm_sim.behaviors.vicsek as vicsek_module
from swarm_sim.config import MissionConfig
from swarm_sim.contracts import Frame, HealthState, SensorObservation, VehicleState
from swarm_sim.controller import SwarmController
from swarm_sim.network import CommsNetwork
from swarm_sim.recruitment import RecruitmentBoard

FLOCK_MODELS = ["boids", "vicsek", "couzin", "olfati_saber"]
BEHAVIOR_MODULES = [boids_module, vicsek_module, couzin_module, olfati_saber_module, common_module]

FORBIDDEN_IDENTIFIERS = {"victims_ground_truth", "WorldState", "victims", "obstacles", "mission"}
FORBIDDEN_ACTUATION_IMPORTS = {"pybullet", "p", "mission", "run_mission", "DSLPIDControl"}


def _own_state(pos=(0.0, 0.0, 3.0), vel=(0.0, 0.0, 0.0), t=0.0):
    return VehicleState(
        vehicle_id="drone0", sim_time_s=t, frame=Frame.LOCAL_ENU,
        position_m=pos, velocity_mps=vel, acceleration_mps2=(0.0, 0.0, 0.0),
        attitude_rad=(0.0, 0.0, 0.0), angular_velocity_radps=(0.0, 0.0, 0.0),
        battery_fraction=1.0, health_state=HealthState.OK,
        estimator_valid=True, last_valid_command_time_s=t,
    )


def _sensor_obs(n_bins, detections=()):
    return SensorObservation(
        vehicle_id="drone0", sensor_timestamp_s=0.0, sensor_latency_s=0.0, fov_deg=90.0,
        range_returns_m=(np.inf,) * n_bins, occluded=(False,) * n_bins,
        dropout=False, pose_uncertainty_m=0.0, detections=detections,
    )


def _step_once(flock_model, seed=7, own_pos=(0.0, 0.0, 3.0), num_drones=3):
    cfg = MissionConfig(num_drones=num_drones, seed=seed, flock_model=flock_model)
    board = RecruitmentBoard()
    network = CommsNetwork(cfg, num_drones, np.random.default_rng(seed))
    positions = np.array([own_pos] + [[float(k + 1) * 2.0, 0.0, 3.0] for k in range(num_drones - 1)])
    velocities = np.zeros((num_drones, 3))
    network.tick(positions, velocities)

    controller = SwarmController(cfg, num_drones, np.random.default_rng(seed))
    own_state = _own_state(pos=own_pos)
    sensor_obs = _sensor_obs(len(controller._obstacle_bin_angles))
    return controller.step(0, own_state, sensor_obs, (), board, network)


# --- deterministic replay -------------------------------------------------

@pytest.mark.parametrize("flock_model", FLOCK_MODELS)
def test_deterministic_replay_same_seed_same_output(flock_model):
    out_a = _step_once(flock_model, seed=123)
    out_b = _step_once(flock_model, seed=123)
    assert np.array_equal(out_a, out_b)


# --- finite outputs for every behavior ------------------------------------

@pytest.mark.parametrize("flock_model", FLOCK_MODELS)
def test_finite_output_no_neighbors(flock_model):
    out = _step_once(flock_model, seed=1, num_drones=1)
    assert np.all(np.isfinite(out))


@pytest.mark.parametrize("flock_model", FLOCK_MODELS)
def test_finite_output_coincident_neighbors(flock_model):
    # All drones start at the exact same position.
    out = _step_once(flock_model, seed=2, own_pos=(5.0, 5.0, 3.0), num_drones=4)
    assert np.all(np.isfinite(out))


@pytest.mark.parametrize("flock_model", FLOCK_MODELS)
def test_finite_output_many_neighbors(flock_model):
    out = _step_once(flock_model, seed=3, num_drones=10)
    assert np.all(np.isfinite(out))


# --- sensor-only neighbor inputs / ground-truth isolation -----------------

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


@pytest.mark.parametrize("module", BEHAVIOR_MODULES, ids=lambda m: m.__name__)
def test_behavior_module_references_no_ground_truth_identifiers(module):
    tree = ast.parse(_module_source_path(module).read_text())
    leaked = _collect_identifiers(tree) & FORBIDDEN_IDENTIFIERS
    assert not leaked, f"{module.__name__} references forbidden ground-truth identifiers: {leaked}"


def test_controller_perceives_olfati_saber_neighbors_only_through_comms_network():
    """The olfati_saber branch added in Phase 3 must source neighbor
    positions/velocities the same way boids/vicsek/couzin already do:
    from _perceived_state's local_pos/local_vel (CommsNetwork-mediated),
    never from a raw ground-truth array."""
    import swarm_sim.controller as controller_module
    source_path = _module_source_path(controller_module)
    tree = ast.parse(source_path.read_text())
    step_fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "step")
    source = ast.get_source_segment(source_path.read_text(), step_fn)
    assert "olfati_saber.olfati_saber_steer(" in source
    assert "local_pos[0]" in source and "local_pos[1:]" in source


# --- behavior output cannot bypass the controller boundary ---------------

@pytest.mark.parametrize("module", BEHAVIOR_MODULES, ids=lambda m: m.__name__)
def test_behavior_module_has_no_actuation_or_mission_import(module):
    """None of the behavior modules may import pybullet, mission.py, or a
    flight-control class directly - every one of their outputs is a
    candidate the caller (SwarmController) may still override, never
    something applied to a vehicle from inside behaviors/*.py itself."""
    tree = ast.parse(_module_source_path(module).read_text())
    imported_names = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported_names.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported_names.add(node.module.split(".")[0])
    leaked = imported_names & FORBIDDEN_ACTUATION_IMPORTS
    assert not leaked, f"{module.__name__} imports a forbidden actuation-path module: {leaked}"


def test_behavior_modules_export_no_vehicle_command_application_function():
    """Sanity check on the contract's name, not just its imports: no
    function in any behavior module is named as if it applies/sends/
    actuates a command - every public function name is a *_steer,
    *_direction, *_heading, or *_candidate_command generator."""
    allowed_prefixes = ("_", "sigma", "bump", "phi", "mean_heading", "order_parameter",
                         "build_adjacency", "connected_components", "is_fragmented",
                         "algebraic_connectivity", "sanitize", "bounded", "clamp", "all_finite",
                         "separation", "alignment", "cohesion", "navigation_term", "spacing_term")
    forbidden_substrings = ("apply", "send", "actuate", "write_command", "set_rpm")
    for module in BEHAVIOR_MODULES:
        tree = ast.parse(_module_source_path(module).read_text())
        for node in ast.walk(tree):
            if isinstance(node, ast.FunctionDef):
                name = node.name.lower()
                assert not any(s in name for s in forbidden_substrings), (
                    f"{module.__name__}.{node.name} looks like a direct-actuation function"
                )
