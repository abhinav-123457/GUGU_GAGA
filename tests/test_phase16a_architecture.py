"""Architecture checks for the Phase 16A mission-frame/grid modules - see
docs/PHASE16A_MISSION_FRAME_GRID.md. AST-based structural proof (never
substring matching), mirroring the earlier phases' patterns."""
import ast
import pathlib

import swarm_sim.mapping.frame as frame_mod
import swarm_sim.mapping.grid as grid_mod
import swarm_sim.mapping as mapping_pkg
import swarm_sim.mission_rules as rules_mod

MAPPING_MODULES = (frame_mod, grid_mod, mapping_pkg)
PURE_GEOMETRY_MODULES = MAPPING_MODULES + (rules_mod,)

ALLOWED_ABSOLUTE_IMPORTS = {
    "__future__", "dataclasses", "hashlib", "json", "math", "re", "typing", "numpy",
}
# Relative imports allowed inside swarm_sim.mapping: the shared contracts (Frame / GeofenceSpec) and
# the package's own siblings. Nothing else in swarm_sim - in particular no sensors / mission /
# sar_world / network / controller / estimation / plant_truth.
ALLOWED_RELATIVE_MODULES = {"contracts", "grid", "frame"}

FORBIDDEN_IDENTIFIERS = {
    "random", "default_rng", "RandomState", "SeedManager", "perf_counter", "monotonic", "sleep",
    "pybullet", "getBasePositionAndOrientation", "VictimGroundTruth", "FloodSearchMission",
    "PlantTruth", "gps", "gnss", "latitude", "longitude",
}


def _tree(module) -> ast.AST:
    return ast.parse(pathlib.Path(module.__file__).read_text(encoding="utf-8"))


def _imports(tree):
    absolute, relative = set(), set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            absolute.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            if node.level:
                # `from .grid import X` -> module "grid"; `from .. import X` -> the names themselves
                relative.update([node.module.split(".")[0]] if node.module else [a.name for a in node.names])
            elif node.module:
                absolute.add(node.module.split(".")[0])
    return absolute, relative


def _identifiers(tree):
    names = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Name):
            names.add(node.id)
        elif isinstance(node, ast.Attribute):
            names.add(node.attr)
        elif isinstance(node, ast.arg):
            names.add(node.arg)
    return names


def test_mapping_package_lists_exactly_the_expected_modules():
    files = sorted(p.name for p in pathlib.Path(mapping_pkg.__file__).parent.glob("*.py"))
    assert files == ["__init__.py", "frame.py", "grid.py"]


def test_pure_geometry_modules_import_only_stdlib_numpy_and_their_siblings():
    for module in PURE_GEOMETRY_MODULES:
        absolute, relative = _imports(_tree(module))
        assert absolute <= ALLOWED_ABSOLUTE_IMPORTS, (module.__name__, absolute - ALLOWED_ABSOLUTE_IMPORTS)
        assert relative <= ALLOWED_RELATIVE_MODULES, (module.__name__, relative - ALLOWED_RELATIVE_MODULES)


def test_rules_module_uses_no_swarm_sim_imports_at_all():
    absolute, relative = _imports(_tree(rules_mod))
    assert not relative and "swarm_sim" not in absolute


def test_no_randomness_clock_simulator_or_truth_identifiers():
    for module in PURE_GEOMETRY_MODULES:
        found = _identifiers(_tree(module)) & FORBIDDEN_IDENTIFIERS
        assert not found, (module.__name__, found)


def test_no_module_level_mutable_state_in_the_geometry_modules():
    """Geometry must be a pure function of its arguments: no module-level
    list / dict / set that could carry state between calls."""
    for module in PURE_GEOMETRY_MODULES:
        for node in _tree(module).body:
            if isinstance(node, (ast.Assign, ast.AnnAssign)):
                targets = node.targets if isinstance(node, ast.Assign) else [node.target]
                if any(isinstance(t, ast.Name) and t.id == "__all__" for t in targets):
                    continue   # the export list is read-only by convention, not state
                value = node.value
                assert not isinstance(value, (ast.List, ast.Dict, ast.Set, ast.ListComp, ast.DictComp, ast.SetComp)), (
                    module.__name__, ast.dump(node)[:80])


def test_production_mapping_does_not_import_the_sar_world_parity_oracle():
    for module in MAPPING_MODULES:
        absolute, relative = _imports(_tree(module))
        assert "sar_world" not in absolute | relative
