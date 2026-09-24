"""Architecture checks for Phase 16C - see docs/PHASE16C_FLIGHT_LIFECYCLE.md.

AST-based (never substring matching) proof that:
  * the autonomy-side flight package can neither name ground truth / GNSS nor import the plant side,
  * the PID - the one control loop fed TRUE state - sits behind the plant-side actuator boundary,
  * only the mission imports the plant-side actuator and the flight scorer,
  * the altitude command is built from the drone's own VehicleState, never from physics arrays."""
import ast
import pathlib

import pytest

import swarm_sim
import swarm_sim.mission as mission_module

PKG = pathlib.Path(swarm_sim.__file__).parent
FLIGHT_FILES = sorted((PKG / "flight").glob("*.py"))

TRUTH_IDENTIFIERS = {"PlantTruth", "plant_truth", "plant_odometry", "OdometrySuite", "LocalizationScorer",
                     "GeofenceExcursionScorer", "FlightScorer", "SimulatedAutopilot", "DSLPIDControl",
                     "computeControlFromState"}
FORBIDDEN_IMPORTS = {"plant_truth", "plant_odometry", "metrics", "plant_actuator", "flight_scoring", "pybullet",
                     "gym_pybullet_drones", "mission", "estimation"}
GNSS_SUBSTRINGS = ("gps", "gnss", "latitude", "longitude")


def _tree(path):
    return ast.parse(path.read_text(encoding="utf-8"))


def _identifiers(tree):
    names = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Name):
            names.add(node.id)
        elif isinstance(node, ast.Attribute):
            names.add(node.attr)
        elif isinstance(node, ast.arg):
            names.add(node.arg)
        elif isinstance(node, (ast.FunctionDef, ast.ClassDef)):
            names.add(node.name)
    return names


def _imported_modules(tree):
    found = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                found.update(alias.name.split("."))
        elif isinstance(node, ast.ImportFrom):
            if node.module:
                found.update(node.module.split("."))
            found.update(alias.name for alias in node.names)
    return found


def _callee(call):
    f = call.func
    return f.attr if isinstance(f, ast.Attribute) else (f.id if isinstance(f, ast.Name) else None)


def _names_in(node):
    return {n.id for n in ast.walk(node) if isinstance(n, ast.Name)}


def _method(class_name, method_name):
    tree = _tree(pathlib.Path(mission_module.__file__))
    cls = next(n for n in ast.walk(tree) if isinstance(n, ast.ClassDef) and n.name == class_name)
    return next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == method_name)


def test_the_flight_package_exists_and_is_not_empty():
    assert {p.name for p in FLIGHT_FILES} >= {"__init__.py", "vertical.py"}


@pytest.mark.parametrize("path", FLIGHT_FILES, ids=lambda p: p.name)
def test_flight_modules_cannot_name_ground_truth_or_the_plant_side(path):
    tree = _tree(path)
    assert not (_identifiers(tree) & TRUTH_IDENTIFIERS), path
    assert not (_imported_modules(tree) & FORBIDDEN_IMPORTS), path


@pytest.mark.parametrize("path", FLIGHT_FILES, ids=lambda p: p.name)
def test_flight_modules_have_no_gnss_identifier(path):
    bad = {n for n in _identifiers(_tree(path)) if any(s in n.lower() for s in GNSS_SUBSTRINGS)}
    assert not bad, (path, bad)


def test_the_supervisor_does_not_import_the_flight_package_or_the_actuator():
    imported = _imported_modules(_tree(PKG / "safety_supervisor.py"))
    assert not (imported & {"flight", "plant_actuator", "flight_scoring"})


def _importers(module_name):
    out = set()
    for path in PKG.rglob("*.py"):
        if path.stem == module_name:
            continue
        if module_name in _imported_modules(_tree(path)):
            out.add(str(path.relative_to(PKG)).replace("\\", "/"))
    return out


def test_only_the_mission_imports_the_plant_actuator_and_the_flight_scorer():
    assert _importers("plant_actuator") == {"mission.py"}
    assert _importers("flight_scoring") == {"mission.py"}


def test_the_plant_actuator_does_not_import_the_truth_carrier():
    """It takes plain arrays; the truth boundary tests keep `plant_truth` importable only by the mission and
    the plant odometry."""
    assert "plant_truth" not in _imported_modules(_tree(PKG / "plant_actuator.py"))


def test_nothing_in_the_autonomy_stack_imports_the_flight_package_except_the_mission():
    assert _importers("flight") <= {"mission.py"}


# ==========================================================================
# FloodSearchMission.run()
# ==========================================================================

def test_run_never_calls_the_pid_directly():
    run = _method("FloodSearchMission", "run")
    assert "computeControlFromState" not in {_callee(c) for c in ast.walk(run) if isinstance(c, ast.Call)}


def test_run_reaches_the_pid_only_through_the_autopilot_object():
    run = _method("FloodSearchMission", "run")
    rpm_calls = [c for c in ast.walk(run) if isinstance(c, ast.Call) and _callee(c) == "rpm"]
    assert len(rpm_calls) == 1
    func = rpm_calls[0].func
    assert isinstance(func, ast.Attribute) and isinstance(func.value, ast.Attribute) and func.value.attr == "autopilot"


def test_the_mission_module_no_longer_imports_the_pid_class():
    assert "DSLPIDControl" not in _imported_modules(_tree(pathlib.Path(mission_module.__file__)))


def test_the_altitude_command_is_built_from_the_drones_own_state_only():
    helper = _method("FloodSearchMission", "_with_altitude_command")
    assert not (_names_in(helper) & {"truth", "obs", "positions", "velocities", "rpys", "PlantTruth"})
    run = _method("FloodSearchMission", "run")
    calls = [c for c in ast.walk(run) if isinstance(c, ast.Call) and _callee(c) == "_with_altitude_command"]
    assert len(calls) == 1
    for arg in list(calls[0].args) + [k.value for k in calls[0].keywords]:
        assert not (_names_in(arg) & {"truth", "obs"}), ast.dump(arg)
    assert "own_state" in {n for a in calls[0].args for n in _names_in(a)}


def test_the_altitude_loop_is_fed_the_estimate_of_the_altitude():
    helper = _method("FloodSearchMission", "_with_altitude_command")
    calls = [c for c in ast.walk(helper) if isinstance(c, ast.Call) and _callee(c) == "command"]
    assert len(calls) == 1
    z_arg = calls[0].args[2]
    # own_state.position_m[2]
    assert isinstance(z_arg, ast.Subscript) and isinstance(z_arg.value, ast.Attribute) \
        and z_arg.value.attr == "position_m" and _names_in(z_arg) == {"own_state"}
