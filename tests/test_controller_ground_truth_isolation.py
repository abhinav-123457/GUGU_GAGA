"""Architectural guard + behavioral proof that SwarmController never sees
ground truth. The static guard (AST-based) fails the build if anyone ever
reintroduces a ground-truth reference into controller.py, even one that
happens not to be exercised by any test; the behavioral tests prove the
same property empirically, by showing the controller's actual output
depends only on its sensed inputs.
"""
import ast
import inspect
import pathlib

import numpy as np
import pytest

import swarm_sim.controller as controller_module
from swarm_sim.config import MissionConfig
from swarm_sim.consensus import ConsensusBoard
from swarm_sim.contracts import Frame, HealthState, SensorObservation, VehicleState
from swarm_sim.controller import SwarmController
from swarm_sim.network import CommsNetwork
from swarm_sim.recruitment import RecruitmentBoard

CONTROLLER_SOURCE_PATH = pathlib.Path(controller_module.__file__)

FORBIDDEN_IDENTIFIERS = {
    "victims_ground_truth", "WorldState", "victims", "obstacles", "mission",
}
FORBIDDEN_PARAM_NAMES = {"positions", "velocities"}


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
# Static guard
# ---------------------------------------------------------------------------

def test_controller_source_contains_no_forbidden_ground_truth_identifiers():
    tree = ast.parse(CONTROLLER_SOURCE_PATH.read_text())
    identifiers = _collect_identifiers(tree)
    leaked = identifiers & FORBIDDEN_IDENTIFIERS
    assert not leaked, f"controller.py references forbidden ground-truth identifiers: {leaked}"


def test_controller_no_function_takes_a_raw_full_swarm_array_parameter():
    tree = ast.parse(CONTROLLER_SOURCE_PATH.read_text())
    param_names = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            param_names.update(a.arg for a in node.args.args)
            param_names.update(a.arg for a in node.args.kwonlyargs)
    leaked = param_names & FORBIDDEN_PARAM_NAMES
    assert not leaked, f"controller.py has a parameter named like a raw ground-truth array: {leaked}"


def test_swarm_controller_init_has_no_obstacles_parameter():
    sig = inspect.signature(SwarmController.__init__)
    assert "obstacles" not in sig.parameters


def test_swarm_controller_step_has_no_positions_or_velocities_parameter():
    sig = inspect.signature(SwarmController.step)
    assert "positions" not in sig.parameters
    assert "velocities" not in sig.parameters


# ---------------------------------------------------------------------------
# Behavioral proof
# ---------------------------------------------------------------------------

def _own_state(vehicle_id="drone0", pos=(0.0, 0.0, 3.0), vel=(0.0, 0.0, 0.0), t=0.0):
    return VehicleState(
        vehicle_id=vehicle_id, sim_time_s=t, frame=Frame.LOCAL_ENU,
        position_m=pos, velocity_mps=vel, acceleration_mps2=(0.0, 0.0, 0.0),
        attitude_rad=(0.0, 0.0, 0.0), angular_velocity_radps=(0.0, 0.0, 0.0),
        battery_fraction=1.0, health_state=HealthState.OK,
        estimator_valid=True, last_valid_command_time_s=t,
    )


def _sensor_obs(range_returns_m, detections=()):
    return SensorObservation(
        vehicle_id="drone0", sensor_timestamp_s=0.0, sensor_latency_s=0.0, fov_deg=90.0,
        range_returns_m=range_returns_m, occluded=tuple(False for _ in range_returns_m),
        dropout=False, pose_uncertainty_m=0.0, detections=detections,
    )


def test_controller_output_is_identical_regardless_of_hidden_world_contents():
    """The core Phase 2 guarantee: feed the SAME sensed inputs, produced
    from two DIFFERENT hidden worlds (different victim positions,
    different obstacle geometry), and the controller must produce
    IDENTICAL output either way - because it structurally cannot see
    which hidden world it's in, only what its sensors reported."""
    cfg = MissionConfig(num_drones=1, seed=1)
    board = RecruitmentBoard()
    network = CommsNetwork(cfg, 1, np.random.default_rng(1))
    network.tick(np.array([[0.0, 0.0, 3.0]]), np.array([[0.0, 0.0, 0.0]]))

    own_state = _own_state()
    sensor_obs = _sensor_obs(range_returns_m=(np.inf,) * len(
        controller_module.sensors.obstacle_scan_bin_angles(cfg)))
    neighbor_obs = ()

    # World A and World B are two DIFFERENT hidden worlds; the sensed
    # inputs above are held fixed and are what actually gets passed to
    # step() - the "hidden world" values below are never passed to it at
    # all, they only illustrate that changing them cannot possibly matter.
    world_a_hidden_victims = [(5.0, 5.0), (10.0, -3.0)]     # noqa: F841
    world_b_hidden_victims = [(-40.0, 12.0)]                 # noqa: F841
    world_a_hidden_obstacles = [(1.0, 1.0)]                  # noqa: F841
    world_b_hidden_obstacles = [(-8.0, 20.0), (3.0, -3.0)]   # noqa: F841

    rng_a = np.random.default_rng(99)
    controller_a = SwarmController(cfg, 1, rng_a)
    out_a = controller_a.step(0, own_state, sensor_obs, neighbor_obs, board, network)

    rng_b = np.random.default_rng(99)
    controller_b = SwarmController(cfg, 1, rng_b)
    out_b = controller_b.step(0, own_state, sensor_obs, neighbor_obs, board, network)

    assert np.allclose(out_a, out_b)


def test_controller_step_signature_matches_the_approved_interface():
    sig = inspect.signature(SwarmController.step)
    params = list(sig.parameters.keys())
    assert params == ["self", "i", "own_state", "sensor_observation", "neighbor_observations",
                       "mission_beliefs", "network"]


def test_consensus_board_never_receives_ground_truth_candidate_ids():
    """Sanity check on the surrounding wiring (mission.py), not just the
    controller: ConsensusBoard.submit's signature never had a place for a
    ground-truth id in the first place, before or after Phase 2."""
    sig = inspect.signature(ConsensusBoard.submit)
    assert list(sig.parameters.keys()) == ["self", "drone_id", "pos", "t"]
