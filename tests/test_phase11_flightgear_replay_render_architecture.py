"""Architecture checks for scripts/run_phase11_flightgear_replay_render.py -
see docs/PHASE11_FLIGHTGEAR_REPLAY_RENDER.md. AST-based structural proof
that the replay renderer has NO MAVLink capability at all (a stronger
guarantee than "chooses not to send a command"), never fabricates its own
FGNetFDM packing, and never claims to be LIVE.
"""
import ast
import os
import pathlib
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"))

import run_phase11_flightgear_replay_render as replay_module  # noqa: E402

FORBIDDEN_MAVLINK_IDENTIFIERS = {
    "command_long_send", "param_set_send", "mav_param_set_send",
    "MAV_CMD_COMPONENT_ARM_DISARM", "MAV_CMD_NAV_TAKEOFF", "MAV_CMD_NAV_LAND",
    "MAV_CMD_DO_SET_MODE", "set_position_target_local_ned_send",
    "ArduPilotSITLTransport", "mavutil",
}


def _tree() -> ast.AST:
    return ast.parse(pathlib.Path(replay_module.__file__).read_text())


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


def test_never_imports_pymavlink_or_mavutil():
    imports = _collect_imports(_tree())
    assert "pymavlink" not in imports
    assert "mavutil" not in imports


def test_never_imports_the_sitl_transport_module():
    imports = _collect_imports(_tree())
    assert "swarm_sim.sitl.ardupilot_transport" not in imports
    # Also check via the dotted module path some ImportFrom nodes may use.
    tree = _tree()
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module:
            assert "ardupilot_transport" not in node.module
            assert "sitl" not in node.module.split(".")


def test_no_mavlink_or_flight_command_identifiers_anywhere():
    identifiers = _collect_identifiers(_tree())
    leaked = identifiers & FORBIDDEN_MAVLINK_IDENTIFIERS
    assert not leaked, f"replay renderer references forbidden MAVLink/flight-command identifiers: {leaked}"


def test_never_forces_arm_with_the_force_magic_value():
    tree = _tree()
    for node in ast.walk(tree):
        if isinstance(node, ast.Constant) and node.value == 21196:
            raise AssertionError("force-arm/disarm magic value (21196) must never appear in the replay renderer")


def test_no_subprocess_or_process_lifecycle_identifiers():
    imports = _collect_imports(_tree())
    assert "subprocess" not in imports
    identifiers = _collect_identifiers(_tree())
    assert not (identifiers & {"Popen", "terminate", "SIGKILL", "SIGTERM"})


def test_reuses_fgnetfdm_packing_instead_of_reimplementing_it():
    """The replay renderer must call telemetry_mapping's own
    build_fg_net_fdm_fields/pack_fg_net_fdm - it must not define its own
    struct.Struct/field-spec (which would risk drifting from the one
    documented, tested FGNetFDM v24 layout)."""
    source = pathlib.Path(replay_module.__file__).read_text()
    assert "tmap.build_fg_net_fdm_fields(" in source
    assert "tmap.pack_fg_net_fdm(" in source
    tree = _tree()
    assert not any(
        isinstance(node, ast.Attribute) and node.attr == "Struct"
        for node in ast.walk(tree)
    ), "must not define its own struct.Struct - reuse telemetry_mapping.py's"


def test_reuses_flightgear_bridge_instead_of_a_new_socket_wrapper():
    source = pathlib.Path(replay_module.__file__).read_text()
    assert "FlightGearBridge(" in source
    assert "parse_flightgear_endpoint(" in source
    tree = _tree()
    # The only socket-module use must come through the reused FlightGearBridge;
    # this file itself never imports the socket module directly.
    imports = _collect_imports(tree)
    assert "socket" not in imports


def test_reuses_update_rate_bounds_from_the_live_bridge_module():
    source = pathlib.Path(replay_module.__file__).read_text()
    assert "from run_phase11_flightgear_viewer import" in source
    assert "MIN_UPDATE_RATE_HZ" in source
    assert "MAX_UPDATE_RATE_HZ" in source


def test_no_hardcoded_non_loopback_ip_literal():
    ip_pattern = re.compile(r"^\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}$")
    for node in ast.walk(_tree()):
        if isinstance(node, ast.Constant) and isinstance(node.value, str) and ip_pattern.match(node.value):
            raise AssertionError(f"unexpected hardcoded IP literal: {node.value!r}")


def test_default_flightgear_endpoint_is_none_never_guessed():
    args = replay_module.build_parser().parse_args([])
    assert args.flightgear_endpoint is None


def test_view_label_is_always_replay_never_live():
    """AST-based (not a raw substring search, which would also match this
    module's own prose docstring): every dict literal that sets a
    'view_label' key must set it to the string 'REPLAY', never 'LIVE'."""
    found_any = False
    for node in ast.walk(_tree()):
        if isinstance(node, ast.Dict):
            for key, value in zip(node.keys, node.values):
                if isinstance(key, ast.Constant) and key.value == "view_label":
                    assert isinstance(value, ast.Constant) and value.value == "REPLAY"
                    found_any = True
    assert found_any, "expected at least one report dict to set view_label"


def test_does_not_modify_the_live_bridge_or_phase10_flight_test_files():
    """This is a regression guard, not a git check: the live bridge and
    Phase 10 flight-test scripts must still pass their own unmodified
    architecture guarantees, proving Phase 11's replay-render work did not
    touch their flight-command safety logic."""
    import run_phase10_sitl_flight_test as phase10_module
    import run_phase11_flightgear_viewer as phase11_viewer_module

    forbidden = {"MAV_CMD_COMPONENT_ARM_DISARM", "MAV_CMD_NAV_TAKEOFF", "MAV_CMD_NAV_LAND"}
    for module in (phase10_module,):
        tree = ast.parse(pathlib.Path(module.__file__).read_text())
        for fn_name in ("run_dry_run", "run_prearm_diagnostics"):
            fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == fn_name)
            assert not (_collect_identifiers(fn) & forbidden)

    tree = ast.parse(pathlib.Path(phase11_viewer_module.__file__).read_text())
    assert not (_collect_identifiers(tree) & forbidden)


def test_source_result_default_points_at_a_tracked_docs_file_not_gitignored_results():
    assert "results" not in os.path.relpath(
        replay_module.DEFAULT_SOURCE_RESULT, os.path.dirname(os.path.dirname(replay_module.__file__))
    ).split(os.sep)[:1]
    assert os.path.dirname(replay_module.DEFAULT_SOURCE_RESULT).endswith("docs")
