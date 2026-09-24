"""Architecture checks for Phase 16B - see docs/PHASE16B_ESTIMATION.md.

AST-based structural proof (never substring matching on source text) that:
  * ground truth reaches the autonomy stack ONLY through `_autonomy_view`,
  * the truth-carrying modules are imported by nobody they should not be,
  * no autonomy module can even name GNSS/GPS.
"""
import ast
import pathlib

import pytest

import swarm_sim
import swarm_sim.mission as mission_module

PKG = pathlib.Path(swarm_sim.__file__).parent

# Modules that make decisions from what a drone believes. None may touch ground truth or GNSS.
AUTONOMY_FILES = (
    [PKG / n for n in ("controller.py", "safety_supervisor.py", "recruitment.py", "distributed_consensus.py",
                       "consensus.py", "network.py", "speed_control.py")]
    + sorted((PKG / "behaviors").glob("*.py"))
    + sorted((PKG / "mapping").glob("*.py"))
    + [PKG / "estimation" / n for n in ("estimator.py", "models.py", "bridge.py", "profiles.py", "__init__.py")]
)

TRUTH_IDENTIFIERS = {"PlantTruth", "plant_truth", "plant_odometry", "OdometrySuite", "LocalizationScorer",
                     "GeofenceExcursionScorer"}
TRUTH_MODULES = {"plant_truth", "plant_odometry", "metrics"}
GNSS_SUBSTRINGS = ("gps", "gnss", "latitude", "longitude")


def _tree(path: pathlib.Path) -> ast.AST:
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
    """Every module path component named by an import, absolute or relative."""
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


def _method(class_name, method_name, module=mission_module):
    tree = _tree(pathlib.Path(module.__file__))
    cls = next(n for n in ast.walk(tree) if isinstance(n, ast.ClassDef) and n.name == class_name)
    return next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == method_name)


def _callee_name(call: ast.Call):
    f = call.func
    return f.attr if isinstance(f, ast.Attribute) else (f.id if isinstance(f, ast.Name) else None)


def _names_in(node):
    return {n.id for n in ast.walk(node) if isinstance(n, ast.Name)}


# ==========================================================================
# 1. the autonomy modules cannot even name ground truth or GNSS
# ==========================================================================

@pytest.mark.parametrize("path", AUTONOMY_FILES, ids=lambda p: str(p.relative_to(PKG)))
def test_autonomy_module_never_references_the_truth_carriers(path):
    tree = _tree(path)
    assert not (_identifiers(tree) & TRUTH_IDENTIFIERS), path
    assert not (_imported_modules(tree) & TRUTH_MODULES), path


@pytest.mark.parametrize("path", AUTONOMY_FILES, ids=lambda p: str(p.relative_to(PKG)))
def test_autonomy_module_has_no_gnss_identifier(path):
    """Structural proof that GPS/GNSS is not an input anywhere a decision is made."""
    bad = {name for name in _identifiers(_tree(path)) if any(s in name.lower() for s in GNSS_SUBSTRINGS)}
    assert not bad, (path, bad)


def test_the_gnss_check_would_actually_catch_an_offender():
    offender = ast.parse("def f(gps_fix):\n    return gps_fix.latitude\n")
    bad = {n for n in _identifiers(offender) if any(s in n.lower() for s in GNSS_SUBSTRINGS)}
    assert bad == {"gps_fix", "latitude"}


@pytest.mark.parametrize("name", ["safety_supervisor.py", "controller.py", "network.py", "recruitment.py",
                                  "consensus.py", "distributed_consensus.py"])
def test_decision_modules_do_not_import_the_estimation_or_mapping_packages(name):
    imported = _imported_modules(_tree(PKG / name))
    assert not (imported & {"estimation", "mapping"}), (name, imported & {"estimation", "mapping"})


# ==========================================================================
# 2. who may import the truth-carrying modules
# ==========================================================================

def _importers(module_name):
    out = set()
    for path in PKG.rglob("*.py"):
        if path.stem == module_name:
            continue
        if module_name in _imported_modules(_tree(path)):
            out.add(str(path.relative_to(PKG)).replace("\\", "/"))
    return out


def test_only_the_mission_and_plant_odometry_import_plant_truth():
    assert _importers("plant_truth") == {"mission.py", "estimation/plant_odometry.py"}


def test_only_the_mission_imports_the_plant_side_estimation_modules():
    assert _importers("plant_odometry") == {"mission.py"}
    assert _importers("metrics") == {"mission.py"}


def _import_sources(tree):
    """The module each import statement pulls FROM (top-level package for absolute imports, first
    component for relative ones) - not the symbols it binds."""
    out = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            out.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            if node.module:
                out.add(node.module.split(".")[0])
            else:
                out.update(alias.name for alias in node.names)
    return out


def test_the_truth_free_estimator_modules_import_only_the_allowed_sources():
    allowed = {"__future__", "dataclasses", "math", "typing", "numpy", "contracts", "models", "profiles"}
    for name in ("estimator.py", "models.py", "bridge.py", "profiles.py"):
        stray = _import_sources(_tree(PKG / "estimation" / name)) - allowed
        assert not stray, (name, stray)


# ==========================================================================
# 3. FloodSearchMission.run(): truth only reaches whitelisted plant-side calls
# ==========================================================================

# Calls that ARE the autonomy stack (or feed it): ground truth must never appear anywhere in their arguments.
AUTONOMY_SINKS = {
    "step",                                   # controller.step  (env.step takes only rpm_action)
    "evaluate", "announce", "try_recruit", "nearest_available_beacon", "service_check", "decay",
    "deliver", "send_message", "pump_messages", "poll_inbox",
    "_submit_detections", "_submit_detections_distributed", "_resolve_confirmed_detections",
    "_resolve_confirmed_detections_distributed", "_send_to_autopilot_adapter",
    "to_vehicle_state", "VehicleState", "SensorObservation", "MissionContext", "CandidateCommand",
    "to_velocity_command", "_to_plant_frame", "_announce_beacon_for_confirmation",
    "own_est_xy", "yaw_error", "pose_sigma",
}
TRUTH_NAMES = {"obs", "truth"}
BARE_TRUTH_ARRAYS = {"positions", "velocities", "rpys"}


def test_run_never_binds_or_reads_a_bare_truth_array():
    run = _method("FloodSearchMission", "run")
    assert not (_names_in(run) & BARE_TRUTH_ARRAYS), "run() must reach physics state only through `truth`/`view`"


def test_no_truth_name_appears_in_the_arguments_of_any_autonomy_call():
    run = _method("FloodSearchMission", "run")
    offenders = []
    for call in (n for n in ast.walk(run) if isinstance(n, ast.Call)):
        if _callee_name(call) in AUTONOMY_SINKS:
            leaked = set()
            for arg in list(call.args) + [k.value for k in call.keywords]:
                leaked |= _names_in(arg) & TRUTH_NAMES
            if leaked:
                offenders.append((_callee_name(call), call.lineno, leaked))
    assert not offenders, offenders


def test_the_sink_list_actually_covers_the_calls_it_is_meant_to_police():
    run = _method("FloodSearchMission", "run")
    called = {_callee_name(n) for n in ast.walk(run) if isinstance(n, ast.Call)}
    assert {"step", "evaluate", "service_check", "_submit_detections_distributed", "to_vehicle_state",
            "VehicleState", "SensorObservation", "MissionContext"} <= (called & AUTONOMY_SINKS)


def test_radio_payload_and_beacon_service_are_fed_from_the_view():
    run = _method("FloodSearchMission", "run")
    tick = next(c for c in ast.walk(run) if isinstance(c, ast.Call) and _callee_name(c) == "tick"
                and any(k.arg == "payload_positions" for k in c.keywords))
    for kw in tick.keywords:
        if kw.arg in ("payload_positions", "payload_velocities"):
            assert isinstance(kw.value, ast.Attribute) and isinstance(kw.value.value, ast.Name) \
                and kw.value.value.id == "view", kw.arg
    service = next(c for c in ast.walk(run) if isinstance(c, ast.Call) and _callee_name(c) == "service_check")
    assert isinstance(service.args[0], ast.Attribute) and service.args[0].value.id == "view"


def test_own_vehicle_state_is_built_from_the_view_only():
    run = _method("FloodSearchMission", "run")
    constructors = [c for c in ast.walk(run) if isinstance(c, ast.Call) and _callee_name(c) in ("VehicleState",
                                                                                                "to_vehicle_state")]
    assert constructors
    for c in constructors:
        assert "view" in _names_in(c) and not (_names_in(c) & TRUTH_NAMES)


def test_truth_wrapper_is_built_once_from_the_env_output_and_only_the_bridge_and_scoring_take_it_whole():
    run = _method("FloodSearchMission", "run")
    whole = []       # every place the whole `truth` object (not truth.<field>) is handed to a call
    for call in (n for n in ast.walk(run) if isinstance(n, ast.Call)):
        for arg in list(call.args) + [k.value for k in call.keywords]:
            if isinstance(arg, ast.Name) and arg.id == "truth":
                whole.append(_callee_name(call))
    assert sorted(whole) == ["_autonomy_view", "_score_localization"]


def test_only_the_bridge_scoring_and_setup_methods_touch_truth_or_the_plant_side_models():
    tree = _tree(pathlib.Path(mission_module.__file__))
    cls = next(n for n in ast.walk(tree) if isinstance(n, ast.ClassDef) and n.name == "FloodSearchMission")
    touching_truth = set()
    touching_odometry = set()
    for fn in (n for n in cls.body if isinstance(n, ast.FunctionDef)):
        names = _names_in(fn)
        attrs = {a.attr for a in ast.walk(fn) if isinstance(a, ast.Attribute)}
        if "PlantTruth" in names or "truth" in names:
            touching_truth.add(fn.name)
        if "_odometry" in attrs:
            touching_odometry.add(fn.name)
    assert touching_truth == {"run", "_autonomy_view", "_score_localization"}
    assert touching_odometry == {"__init__", "_autonomy_view"}


def test_the_plant_frame_and_beacon_helpers_never_see_ground_truth():
    for name in ("_to_plant_frame", "_announce_beacon_for_confirmation"):
        fn = _method("FloodSearchMission", name)
        assert not (_names_in(fn) & (TRUTH_NAMES | BARE_TRUTH_ARRAYS | {"PlantTruth"})), name


# ==========================================================================
# 4. the plant side stays plant side
# ==========================================================================

def test_the_estimator_never_receives_a_plant_truth():
    fn = _method("StateEstimator", "step", module=__import__("swarm_sim.estimation.estimator",
                                                             fromlist=["estimator"]))
    arg_names = [a.arg for a in fn.args.args]
    assert arg_names == ["self", "bundle"]


def test_plant_odometry_is_the_only_estimation_module_that_reads_truth_fields():
    for name in ("estimator.py", "models.py", "bridge.py", "profiles.py"):
        attrs = {a.attr for a in ast.walk(_tree(PKG / "estimation" / name)) if isinstance(a, ast.Attribute)}
        assert not ({"positions", "velocities", "rpys"} & attrs), name
