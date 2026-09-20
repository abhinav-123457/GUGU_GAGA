"""Architecture checks for Phase 15's external research-reference
adapters - see docs/PHASE15_SWARM124_LANGOSTINO_INTEGRATION.md. AST-based
structural proof mirroring Phase 10-14's own pattern: these properties
are checked by inspecting the actual source/imports, not just documented
in prose.
"""
from __future__ import annotations

import ast
import pathlib

import swarm_sim.distributed_consensus as distributed_consensus
import swarm_sim.external.external_contracts as external_contracts
import swarm_sim.external.langostino_reference as langostino_reference
import swarm_sim.external.swarm124_policy_adapter as swarm124_policy_adapter

_EXTERNAL_MODULES = (external_contracts, langostino_reference, swarm124_policy_adapter)

_FORBIDDEN_HARDWARE_MODULES = {"serial", "pyserial", "socket"}
_FORBIDDEN_HARDWARE_IDENTIFIERS = {
    "MSP_SET_RAW_RC", "MSP", "msp", "RC_CHANNELS_OVERRIDE", "rc_channels_override_send",
    "Serial", "pyserial",
}
_FORBIDDEN_ARM_TAKEOFF_IDENTIFIERS = {
    "MAV_CMD_COMPONENT_ARM_DISARM", "MAV_CMD_NAV_TAKEOFF", "arm", "disarm", "component_arm_disarm_send",
}
_FORBIDDEN_TRANSPORT_MODULES = {"pymavlink", "mavutil"}


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


# ==========================================================================
# 1. Langostino: no serial, no MSP, no RC override, no hardware endpoint
# ==========================================================================

def test_langostino_reference_never_touches_serial_or_msp():
    imports = _collect_imports(_tree(langostino_reference))
    assert not (imports & _FORBIDDEN_HARDWARE_MODULES), \
        f"langostino_reference.py imports a forbidden hardware module: {imports & _FORBIDDEN_HARDWARE_MODULES}"
    identifiers = _collect_identifiers(_tree(langostino_reference))
    assert not (identifiers & _FORBIDDEN_HARDWARE_IDENTIFIERS), \
        f"langostino_reference.py references a forbidden hardware identifier: " \
        f"{identifiers & _FORBIDDEN_HARDWARE_IDENTIFIERS}"


def test_no_external_module_opens_a_serial_port_or_socket():
    for module in _EXTERNAL_MODULES:
        imports = _collect_imports(_tree(module))
        assert not (imports & _FORBIDDEN_HARDWARE_MODULES), \
            f"{module.__name__} imports a forbidden hardware module: {imports & _FORBIDDEN_HARDWARE_MODULES}"


def test_no_rc_override_path_exists_anywhere_in_external_package():
    for module in _EXTERNAL_MODULES:
        source = _source(module).lower()
        for forbidden in ("rc_override", "set_raw_rc", "rc_channels_override"):
            assert forbidden not in source, f"{module.__name__} contains a forbidden RC-override reference"


def test_langostino_reference_never_accepts_a_real_hardware_endpoint():
    """No function in this module takes a serial port path, baud rate, I2C
    bus number, or IP/hostname - it only ever takes and returns this
    project's own plain dataclasses (see module docstring)."""
    tree = _tree(langostino_reference)
    forbidden_arg_names = {"port", "baudrate", "i2c_bus", "host", "hostname", "ip_address"}
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef):
            arg_names = {a.arg for a in node.args.args}
            assert not (arg_names & forbidden_arg_names), \
                f"{node.name}() takes a hardware-endpoint-shaped argument: {arg_names & forbidden_arg_names}"


# ==========================================================================
# 2. no external code can reach a transport, arm, or take off
# ==========================================================================

def test_no_external_module_imports_a_transport_or_mavlink_library():
    for module in _EXTERNAL_MODULES:
        imports = _collect_imports(_tree(module))
        assert not (imports & _FORBIDDEN_TRANSPORT_MODULES), \
            f"{module.__name__} imports {imports & _FORBIDDEN_TRANSPORT_MODULES} directly - " \
            "external adapters must only ever produce a CandidateCommand, never touch MAVLink themselves"


def test_no_external_module_imports_from_swarm_sim_autopilot_package():
    """AST-based (not a substring scan): a module's own explanatory
    docstring may legitimately mention `swarm_sim.autopilot.types` by name
    (e.g. to explain a design parallel) without ever importing it - only a
    real `ast.ImportFrom`/`ast.Import` node counts as this module actually
    reaching into the autopilot/transport layer."""
    for module in _EXTERNAL_MODULES:
        for node in ast.walk(_tree(module)):
            if isinstance(node, ast.ImportFrom) and node.module:
                assert "autopilot" not in node.module, \
                    f"{module.__name__} imports from {node.module!r} - it must stop at CandidateCommand"
            elif isinstance(node, ast.Import):
                for alias in node.names:
                    assert "autopilot" not in alias.name, \
                        f"{module.__name__} imports {alias.name!r} - it must stop at CandidateCommand"


def test_no_external_module_can_arm_or_take_off():
    for module in _EXTERNAL_MODULES:
        identifiers = _collect_identifiers(_tree(module))
        assert not (identifiers & _FORBIDDEN_ARM_TAKEOFF_IDENTIFIERS), \
            f"{module.__name__} references an arm/takeoff identifier: " \
            f"{identifiers & _FORBIDDEN_ARM_TAKEOFF_IDENTIFIERS}"


def test_swarm124_adapter_never_constructs_an_adapter_command_directly():
    """Only a SafetyDecision.filtered_command may become an AdapterCommand
    (see swarm_sim/sar_mission.py's own tick()) - this adapter must stop
    at CandidateCommand and go no further."""
    source = _source(swarm124_policy_adapter)
    assert "AdapterCommand(" not in source


# ==========================================================================
# 3. every external candidate is the SAME untrusted type the rest of the
#    project already routes through SafetySupervisor.evaluate()
# ==========================================================================

def test_swarm124_adapter_only_ever_returns_the_projects_own_candidate_command_type():
    tree = _tree(swarm124_policy_adapter)
    convert_fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)
                       and n.name == "convert_swarm124_action_to_candidate")
    source = ast.get_source_segment(_source(swarm124_policy_adapter), convert_fn)
    assert "validate_and_build_candidate(" in source
    # the function must return whatever validate_and_build_candidate gives
    # back, not construct its own separate result type
    assert "return validate_and_build_candidate(" in source


def test_external_contracts_candidate_is_the_real_safety_supervisor_type():
    from swarm_sim.safety_supervisor import CandidateCommand
    assert external_contracts.CandidateCommand is CandidateCommand


# ==========================================================================
# 4. distributed consensus already documents (and this proves) it cannot
#    bypass SafetySupervisor - unmodified this phase, checked as a
#    regression against Phase 15's own "consensus cannot bypass
#    SafetySupervisor" requirement
# ==========================================================================

def test_distributed_consensus_never_imports_a_transport_or_mavlink_library():
    imports = _collect_imports(_tree(distributed_consensus))
    assert not (imports & _FORBIDDEN_TRANSPORT_MODULES), \
        f"distributed_consensus.py imports {imports & _FORBIDDEN_TRANSPORT_MODULES} directly - " \
        "a consensus result must become a candidate intent and pass through SafetySupervisor like any other"
    assert "AdapterCommand(" not in _source(distributed_consensus)
