"""Phase 10: one-vehicle SITL flight-readiness and controlled-flight test -
see docs/PHASE10_SITL_FLIGHT_TEST.md.

STRICTLY SIMULATOR-ONLY. ATTACH mode only (same as Phase 9 - this script
never spawns or kills a process). Never connects to anything but
127.0.0.1/localhost. Never arms or takes off unless every explicit
opt-in flag AND every safety gate below passes.

Four explicitly separate modes, never merged (see each function's own
report shape):

    --dry-run (or no arm-test flags at all - THE DEFAULT, always safe)
        Validates configuration/gate shape only - never opens a socket.

    --diagnose-prearm
        Read-only: attaches, reads a fixed set of parameters
        (ARMING_CHECK, INS_ACCOFFS_*/INS_ACCSCAL_*, BATT_MONITOR,
        FENCE_ENABLE/FENCE_TYPE/FENCE_ALT_MAX/FENCE_RADIUS/FENCE_MARGIN/
        FENCE_ACTION - the last six also surfaced in their own
        report["geofence"] section) and listens for ArduPilot's own STATUSTEXT
        "PreArm: ..." messages (ArduPilot broadcasts these on its own,
        continuously, whenever a check fails - no arm attempt is sent or
        needed). Never writes a parameter, never arms.

    --attach (telemetry_only - delegates to Phase 9's own
        run_telemetry_visualization, completely unmodified, so this
        phase never duplicates or changes Phase 9's telemetry behavior)

    --sitl-only --allow-sitl-arm-test --confirm-sitl-flight-test
        The actual flight test: arm -> takeoff (<= 2m by default) ->
        hold -> land -> confirm disarmed. Every one of Step 2's gates
        (see _check_flight_test_gate) must pass, and operator
        confirmation must be supplied, before any command that could
        arm the vehicle is ever sent. If ArduPilot rejects arming or
        takeoff, this script reports the exact rejection and stops - it
        never retries, forces, or bypasses a rejection.

`hold_test` and `planner_preview` (also required to stay "clearly
separate") are Phase 9's own existing modes, unchanged - see
scripts/run_phase9_single_vehicle_visual.py. This script imports and
reuses them directly rather than re-implementing or modifying them.

**Command authority**: the arm/GUIDED/takeoff/land sequence in
`run_sitl_flight_test` is a DEDICATED, EXPLICIT, MANUAL SITL test command
path (per Phase 10's own requirement 4: "the first flight-readiness test
may use a dedicated, explicit, manual SITL test command path rather than
the swarm planner") - it talks directly to the already-open pymavlink
connection (`transport._channel(vehicle_id).connection`), NEVER through
`ArduPilotSITLTransport.send_command()`. This is deliberate, not an
oversight: `swarm_sim/sitl/ardupilot_transport.py` structurally can never
reference an arm/takeoff identifier at all (Phase 8's own architecture
guarantee, unmodified this phase - see
tests/test_ardupilot_sitl_architecture.py), so there is no CommandType or
transport method this manual path could route through even if it wanted
to. `hold_test`/`planner_preview`, by contrast, still go through the full
SafetySupervisor -> AdapterCommand -> ArduPilotSITLTransport chain
exactly as Phase 9 built it.

**No prohibited shortcuts anywhere in this module** (see
docs/PHASE10_SITL_FLIGHT_TEST.md's full list and
tests/test_phase10_sitl_flight_test_architecture.py): no
`param_set`/`param_set_send` call exists anywhere in this file - every
parameter this script touches is READ ONLY, never written, and
ARMING_CHECK is never referenced as a value to write. No force-arm magic
parameter is ever sent (MAV_CMD_COMPONENT_ARM_DISARM's param2 is always
0). No battery/estimator/heartbeat/geofence value is ever fabricated in
software - every check reads real MAVLink telemetry from the real SITL
process.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import run_phase9_single_vehicle_visual as phase9  # noqa: E402

from swarm_sim.sitl.ardupilot_transport import (  # noqa: E402
    ArduPilotSITLTransport, ArduPilotTransportError, PYMAVLINK_AVAILABLE, PYMAVLINK_VERSION,
    dry_run_validate, parse_connection_string,
)

try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None

OUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "results",
                        "phase10_sitl_flight_test")

# --------------------------------------------------------------------------
# Conservative, hard-coded safety ceilings - defense in depth ON TOP OF (never
# instead of) the CLI's own validated arguments. A caller passing
# --max-altitude 100 is rejected by the gate below, not silently clamped.
# --------------------------------------------------------------------------
HARD_MAX_ALTITUDE_M = 3.0
HARD_MAX_SPEED_MPS = 1.0
HARD_MAX_DURATION_S = 120.0
REQUIRED_FLIGHT_TEST_NAMESPACE = "sitl/drone0"

_ARM_TAKEOFF_LAND_COMMAND_TIMEOUT_S = 10.0
_ARM_CONFIRM_TIMEOUT_S = 10.0
_TAKEOFF_CLIMB_TIMEOUT_S = 20.0
_LAND_DISARM_TIMEOUT_S = 30.0

_SAFETY_BANNER = (
    "Phase 10: one-vehicle SITL flight-readiness and controlled-flight test.\n"
    "  - Simulator-only. No Pixhawk, serial port, radio, real vehicle, or hardware-in-the-loop.\n"
    "  - Localhost (127.0.0.1) only - the transport refuses anything else.\n"
    "  - The vehicle only arms/takes off if EVERY explicit opt-in flag and safety gate below\n"
    "    passes, AND operator confirmation is supplied. Any failure stops the test - never forced.\n"
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Phase 10: one-vehicle SITL flight-readiness and controlled-flight test "
                    "(simulator-only, ATTACH mode only).",
    )
    parser.add_argument("--connection", default="tcp:127.0.0.1:5760")
    parser.add_argument("--system-id", type=int, default=1)
    parser.add_argument("--component-id", type=int, default=1)
    parser.add_argument("--namespace", default="sitl/drone0")
    parser.add_argument("--startup-timeout", type=float, default=20.0)
    parser.add_argument("--duration", type=float, default=60.0,
                         help="hold duration in seconds for sitl_flight_test / telemetry_only streaming duration")
    parser.add_argument("--dry-run", action="store_true", help="validate configuration only - never opens a socket")
    parser.add_argument("--attach", action="store_true",
                         help="telemetry_only mode - delegates to Phase 9's own telemetry_visualization, unchanged")
    parser.add_argument("--diagnose-prearm", action="store_true",
                         help="prearm_diagnostics mode - read-only, never arms")
    parser.add_argument("--sitl-only", action="store_true",
                         help="required (with --allow-sitl-arm-test and --confirm-sitl-flight-test) for sitl_flight_test")
    parser.add_argument("--allow-sitl-arm-test", action="store_true",
                         help="required explicit opt-in for sitl_flight_test")
    parser.add_argument("--confirm-sitl-flight-test", action="store_true",
                         help="required explicit operator confirmation for sitl_flight_test "
                              "(an interactive 'yes' prompt is used instead if this is omitted and stdin is a tty)")
    parser.add_argument("--max-altitude", type=float, default=2.0,
                         help=f"takeoff altitude in meters, hard-capped at {HARD_MAX_ALTITUDE_M}m")
    parser.add_argument("--max-speed", type=float, default=0.5,
                         help=f"reserved horizontal speed ceiling in m/s, hard-capped at {HARD_MAX_SPEED_MPS}m/s "
                              "(this minimal sequence never commands a horizontal velocity - see docs)")
    return parser


# --------------------------------------------------------------------------
# Gate check (Phase 10 Step 2) - every requirement checked BEFORE any socket
# is opened for the flight-test path. Never mutates anything.
# --------------------------------------------------------------------------

def _check_flight_test_gate(args) -> dict:
    gate = {"passed": False, "checks": {}, "reasons_failed": []}

    def require(name, condition, detail=None):
        gate["checks"][name] = {"passed": bool(condition), "detail": detail}
        if not condition:
            gate["reasons_failed"].append(name)

    require("sitl_only_flag", args.sitl_only)
    require("allow_sitl_arm_test_flag", args.allow_sitl_arm_test)
    require("namespace_is_sitl_drone0", args.namespace == REQUIRED_FLIGHT_TEST_NAMESPACE, args.namespace)
    require("max_altitude_within_hard_cap", 0.0 < args.max_altitude <= HARD_MAX_ALTITUDE_M, args.max_altitude)
    require("max_speed_within_hard_cap", 0.0 < args.max_speed <= HARD_MAX_SPEED_MPS, args.max_speed)
    require("duration_within_hard_cap", 0.0 < args.duration <= HARD_MAX_DURATION_S, args.duration)
    require("system_id_configured", isinstance(args.system_id, int) and 1 <= args.system_id <= 255, args.system_id)
    require("component_id_configured", isinstance(args.component_id, int) and 1 <= args.component_id <= 255,
            args.component_id)
    require("pymavlink_available", PYMAVLINK_AVAILABLE)
    require("command_timeout_configured", _ARM_TAKEOFF_LAND_COMMAND_TIMEOUT_S > 0)
    require("heartbeat_timeout_configured", args.startup_timeout > 0)

    try:
        proto, host, port = parse_connection_string(args.connection)
        require("localhost_connection", host in ("127.0.0.1", "localhost", "::1"), host)
    except ArduPilotTransportError as e:
        require("localhost_connection", False, str(e))

    gate["passed"] = not gate["reasons_failed"]
    return gate


def _check_geofence_gate(fence_enable_value) -> dict:
    """Live, post-attach gate - separate from `_check_flight_test_gate`
    because `FENCE_ENABLE` can only be known from a real PARAM_VALUE read,
    not from CLI args alone. Read-only: never sends `param_set_send`. See
    docs/PHASE10_SITL_FLIGHT_TEST.md's "SITL-only geofence configuration
    procedure" for how an operator enables this via ArduPilot's own
    `--defaults` file mechanism - this project's own code never writes the
    parameter for them."""
    gate = {"passed": False, "checks": {}, "reasons_failed": []}

    def require(name, condition, detail=None):
        gate["checks"][name] = {"passed": bool(condition), "detail": detail}
        if not condition:
            gate["reasons_failed"].append(name)

    require("fence_enabled", bool(fence_enable_value), fence_enable_value)

    gate["passed"] = not gate["reasons_failed"]
    return gate


def _operator_confirmed(args) -> bool:
    if args.confirm_sitl_flight_test:
        return True
    if sys.stdin.isatty():
        try:
            answer = input(
                f"Type exactly 'yes' to confirm this SITL-only flight test "
                f"(arm, takeoff to {args.max_altitude:.1f}m, hold {args.duration:.0f}s, land) - "
                f"anything else cancels: "
            )
        except EOFError:
            return False
        return answer.strip() == "yes"
    return False


# --------------------------------------------------------------------------
# Mode: dry_run
# --------------------------------------------------------------------------

def run_dry_run(args) -> dict:
    report = {
        "mode": "dry_run", "namespace": args.namespace, "connection_string": args.connection,
        "system_id": args.system_id, "component_id": args.component_id,
        "sitl_only_flag": args.sitl_only, "ok": False, "remaining_failures": [],
    }
    gate = _check_flight_test_gate(args)
    report["gate_checks"] = gate["checks"]
    report["gate_reasons_failed"] = gate["reasons_failed"]

    vehicle_id, endpoint, allowed_ports = phase9._try_build_endpoint(args, report)
    endpoint_ok = vehicle_id is not None
    if endpoint_ok:
        dr = dry_run_validate({vehicle_id: endpoint}, allowed_ports=allowed_ports)
        report["endpoint_dry_run"] = {"ok": dr.ok, "problems": list(dr.problems)}
        endpoint_ok = dr.ok

    # A plain --dry-run (without every arm-test flag) only needs the
    # ENDPOINT shape to be valid - it is not itself refused for lacking
    # --allow-sitl-arm-test/--confirm-sitl-flight-test, since dry-run's own
    # job is "would this configuration be safe to attach to", independent
    # of whether the caller ever intends to run the flight test.
    report["ok"] = endpoint_ok
    report["pymavlink_available"] = PYMAVLINK_AVAILABLE
    report["pymavlink_version"] = PYMAVLINK_VERSION
    return report


# --------------------------------------------------------------------------
# Mode: prearm_diagnostics (read-only, never arms, never writes a parameter)
# --------------------------------------------------------------------------

_DIAGNOSTIC_PARAM_NAMES = (
    "ARMING_CHECK", "INS_ACCOFFS_X", "INS_ACCOFFS_Y", "INS_ACCOFFS_Z",
    "INS_ACCSCAL_X", "INS_ACC2OFFS_X", "INS_ACC2SCAL_X", "BATT_MONITOR",
    "FENCE_ENABLE", "FENCE_TYPE", "FENCE_ALT_MAX", "FENCE_RADIUS", "FENCE_MARGIN", "FENCE_ACTION",
)

# Reported verbatim in run_prearm_diagnostics's report["geofence"] - read-only,
# no gate/safety behavior change (see _check_geofence_gate, unmodified).
_GEOFENCE_REPORT_PARAM_NAMES = (
    "FENCE_ENABLE", "FENCE_TYPE", "FENCE_ALT_MAX", "FENCE_RADIUS", "FENCE_MARGIN", "FENCE_ACTION",
)


def _read_params(conn, sysid: int, compid: int, names, timeout_s: float) -> dict:
    """Read-only PARAM_REQUEST_READ/PARAM_VALUE round trip - never sends
    PARAM_SET. Bounded, paced (0.05s), never a tight loop."""
    for name in names:
        conn.mav.param_request_read_send(sysid, compid, name.encode(), -1)
    values = {}
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and len(values) < len(names):
        msg = conn.recv_match(type="PARAM_VALUE", blocking=False)
        if msg is None:
            time.sleep(0.05)
            continue
        pid = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode(errors="replace")
        pid = pid.rstrip("\x00")
        if pid in names:
            values[pid] = msg.param_value
    return values


def _collect_statustexts(conn, timeout_s: float):
    texts = []
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        msg = conn.recv_match(type="STATUSTEXT", blocking=False)
        if msg is None:
            time.sleep(0.1)
            continue
        text = msg.text if isinstance(msg.text, str) else msg.text.decode(errors="replace")
        texts.append(text.rstrip("\x00"))
    return texts


def run_prearm_diagnostics(args) -> dict:
    report = {
        "mode": "prearm_diagnostics", "namespace": args.namespace, "connection_string": args.connection,
        "system_id": args.system_id, "component_id": args.component_id,
        "attach": {"succeeded": False, "reason": None}, "heartbeat": {"received": False},
        "parameters": {}, "statustexts": [], "prearm_messages": [],
        "battery": {}, "geofence": {}, "diagnosis": {}, "shutdown": {"clean": False}, "remaining_failures": [],
    }
    vehicle_id, endpoint, allowed_ports = phase9._try_build_endpoint(args, report)
    if vehicle_id is None:
        return report

    transport = ArduPilotSITLTransport({vehicle_id: endpoint}, allowed_ports=allowed_ports,
                                        startup_timeout_s=args.startup_timeout)
    try:
        transport.start()
    except ArduPilotTransportError as e:
        report["attach"]["reason"] = str(e)
        report["remaining_failures"].append(f"attach failed: {e}")
        return report
    report["attach"]["succeeded"] = True
    channel = transport._channel(vehicle_id)
    conn = channel.connection
    report["heartbeat"]["received"] = channel.heartbeat_ok

    report["parameters"] = _read_params(conn, args.system_id, args.component_id, _DIAGNOSTIC_PARAM_NAMES, 5.0)
    report["statustexts"] = _collect_statustexts(conn, 6.0)
    report["prearm_messages"] = [t for t in report["statustexts"] if t.startswith("PreArm")]

    telem = transport.receive_telemetry(vehicle_id)
    batt_monitor = report["parameters"].get("BATT_MONITOR")
    report["battery"] = {
        "monitor_param_value": batt_monitor,
        "monitor_configured": bool(batt_monitor),
        "battery_fraction_reported": telem.battery_fraction,
    }
    # Reporting-only: surfaces the same read-only parameters
    # _check_geofence_gate() itself gates on, plus the rest of the fence
    # shape, for operator visibility. Does not change the gate, any
    # safety behavior, or what's written (nothing is ever written) -
    # values are simply copied from report["parameters"] when present,
    # None otherwise ("when available").
    report["geofence"] = {name: report["parameters"].get(name) for name in _GEOFENCE_REPORT_PARAM_NAMES}

    accel_offsets = [report["parameters"].get(n) for n in ("INS_ACCOFFS_X", "INS_ACCOFFS_Y", "INS_ACCOFFS_Z")]
    seen_offsets = [v for v in accel_offsets if v is not None]
    accel_all_zero = bool(seen_offsets) and all(v == 0.0 for v in seen_offsets)
    report["diagnosis"]["accel_offsets_are_zero"] = accel_all_zero
    report["diagnosis"]["accel_cal_explanation"] = (
        "INS_ACCOFFS_X/Y/Z are exactly 0.0. AP_InertialSensor::accel_calibrated_ok_all() "
        "(libraries/AP_InertialSensor/AP_InertialSensor.cpp) treats an exactly-zero offset as "
        "'not calibrated', which is what produces 'PreArm: 3D Accel calibration needed'. Standard fix: "
        "launch with --defaults Tools/autotest/default_params/copter.parm (ArduPilot's own official SITL "
        "default parameter file), which sets small non-zero INS_ACCOFFS_*/INS_ACCSCAL_* values - see "
        "docs/PHASE10_SITL_FLIGHT_TEST.md."
        if accel_all_zero else
        "INS_ACCOFFS_X/Y/Z are non-zero - accelerometer offsets already look calibrated to ArduPilot's own check."
    )
    report["diagnosis"]["battery_explanation"] = (
        f"BATT_MONITOR={batt_monitor!r} (0/None = AP_BattMonitor::Type::NONE, ArduPilot's own compiled-in "
        "default). 0.00V/0.0A/0% is the correct, expected report when no battery monitor is configured - not "
        "a Mission Planner display bug and not a swarm_sim bug. ArduPilot's SITL backend still internally "
        "simulates a battery (SIM_BATT_VOLTAGE defaults to 12.6V - libraries/SITL/SITL.cpp) but only reports "
        "it once BATT_MONITOR names a real type (the standard default_params/copter.parm sets BATT_MONITOR=4, "
        "Analog Voltage and Current) - see docs/PHASE10_SITL_FLIGHT_TEST.md."
    )
    arming_check = report["parameters"].get("ARMING_CHECK")
    report["diagnosis"]["arming_check_value"] = arming_check
    report["diagnosis"]["arming_check_stayed_enabled"] = arming_check not in (None, 0.0)

    stop_result = transport.stop()
    report["shutdown"]["clean"] = stop_result.success
    return report


# --------------------------------------------------------------------------
# Mode: sitl_flight_test - the dedicated, explicit, manual SITL test command
# path. See module docstring's "Command authority" section for why this
# never routes through ArduPilotSITLTransport.send_command().
# --------------------------------------------------------------------------

def _send_command_long_and_wait_ack(conn, sysid: int, compid: int, command_id: int, timeout_s: float,
                                     p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0):
    """Dedicated manual command path for arm/mode/takeoff/land only - never
    used for setpoints, never routed through ArduPilotSITLTransport (which
    must never reference these identifiers at all - Phase 8's own
    architecture guarantee, unmodified). p2 is never a force-arm magic
    value; this function accepts whatever ArduPilot's own COMMAND_ACK says
    and never retries past `timeout_s` or bypasses a rejection."""
    conn.mav.command_long_send(sysid, compid, command_id, 0, p1, p2, p3, p4, p5, p6, p7)
    deadline = time.monotonic() + timeout_s
    statustexts = []
    while time.monotonic() < deadline:
        msg = conn.recv_match(blocking=False)
        if msg is None:
            time.sleep(0.05)
            continue
        if msg.get_srcSystem() != sysid or msg.get_srcComponent() != compid:
            continue
        if msg.get_type() == "STATUSTEXT":
            text = msg.text if isinstance(msg.text, str) else msg.text.decode(errors="replace")
            statustexts.append(text.rstrip("\x00"))
        elif msg.get_type() == "COMMAND_ACK" and msg.command == command_id:
            return msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED, msg.result, statustexts
    return False, None, statustexts   # bounded timeout - never bypassed, caller must stop


def _set_mode_guided(conn, sysid: int, compid: int, timeout_s: float):
    mode_number = {v: k for k, v in mavutil.mode_mapping_acm.items()}.get("GUIDED")
    if mode_number is None:
        return False, None, []
    return _send_command_long_and_wait_ack(
        conn, sysid, compid, mavutil.mavlink.MAV_CMD_DO_SET_MODE, timeout_s,
        p1=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, p2=mode_number,
    )


def run_sitl_flight_test(args) -> dict:
    report = {
        "mode": "sitl_flight_test", "namespace": args.namespace, "connection_string": args.connection,
        "system_id": args.system_id, "component_id": args.component_id,
        "gate": None, "operator_confirmed": False,
        "attach": {"succeeded": False, "reason": None}, "heartbeat": {"received": False},
        "identity_verified": False, "estimator_valid": None, "telemetry_valid": None,
        "initially_disarmed": None, "prearm_clear": None, "prearm_messages": [],
        "battery_state": {}, "geofence_enabled": None, "geofence_gate": None,
        "version": {"confirmed": False, "raw": None},
        "events": [],
        "arm": {"attempted": False, "accepted": None, "confirmed_by_heartbeat": None, "statustexts": []},
        "takeoff": {"attempted": False, "accepted": None, "max_altitude_observed_m": None, "statustexts": []},
        "hold": {"duration_s": None, "sample_count": 0},
        "land": {"attempted": False, "accepted": None},
        "disarm_confirmed": None, "emergency_action": None,
        "shutdown": {"clean": False}, "remaining_failures": [], "flight_actually_happened": False,
    }

    def log_event(event, **fields):
        report["events"].append({"monotonic_s": time.monotonic(), "event": event, **fields})

    gate = _check_flight_test_gate(args)
    report["gate"] = gate
    if not gate["passed"]:
        report["remaining_failures"].append(f"gate check failed: {gate['reasons_failed']}")
        return report   # stop without arming - no fallback to another mode

    report["operator_confirmed"] = _operator_confirmed(args)
    if not report["operator_confirmed"]:
        report["remaining_failures"].append("operator confirmation not supplied - stopping without arming")
        return report

    vehicle_id, endpoint, allowed_ports = phase9._try_build_endpoint(args, report)
    if vehicle_id is None:
        return report

    transport = ArduPilotSITLTransport({vehicle_id: endpoint}, allowed_ports=allowed_ports,
                                        startup_timeout_s=args.startup_timeout)
    try:
        transport.start()
    except ArduPilotTransportError as e:
        report["attach"]["reason"] = str(e)
        report["remaining_failures"].append(f"attach failed: {e}")
        return report
    report["attach"]["succeeded"] = True
    channel = transport._channel(vehicle_id)
    conn = channel.connection
    report["heartbeat"]["received"] = channel.heartbeat_ok
    report["identity_verified"] = channel.heartbeat_ok
    log_event("attached")

    armed_this_session = [False]

    def emergency_cleanup(reason):
        report["emergency_action"] = reason
        log_event("emergency_action", reason=reason)
        if armed_this_session[0]:
            _send_command_long_and_wait_ack(conn, args.system_id, args.component_id,
                                             mavutil.mavlink.MAV_CMD_NAV_LAND, 5.0)
            time.sleep(1.0)
            telem_now = transport.receive_telemetry(vehicle_id)
            if telem_now.armed:
                _send_command_long_and_wait_ack(conn, args.system_id, args.component_id,
                                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 5.0, p1=0)

    try:
        conn.mav.command_long_send(args.system_id, args.component_id,
                                    mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES, 0, 1, 0, 0, 0, 0, 0, 0)
        deadline = time.monotonic() + 5.0
        version_msg = None
        while time.monotonic() < deadline:
            msg = conn.recv_match(type="AUTOPILOT_VERSION", blocking=False)
            if msg is not None:
                version_msg = msg
                break
            time.sleep(0.1)
        if version_msg is None:
            report["remaining_failures"].append("could not confirm ArduPilot version (no AUTOPILOT_VERSION reply)")
            return report
        report["version"] = {"confirmed": True, "raw": {"flight_sw_version": int(version_msg.flight_sw_version)}}

        warm_deadline = time.monotonic() + 10.0
        telem = None
        while time.monotonic() < warm_deadline:
            telem = transport.receive_telemetry(vehicle_id)
            if telem.position_m is not None:
                break
            time.sleep(0.2)
        report["telemetry_valid"] = telem is not None and telem.position_m is not None
        report["estimator_valid"] = bool(telem is not None and telem.estimator_valid)
        if not report["telemetry_valid"] or not report["estimator_valid"]:
            report["remaining_failures"].append("telemetry or estimator not valid - stopping without arming")
            return report

        report["initially_disarmed"] = not telem.armed
        if telem.armed:
            report["remaining_failures"].append("vehicle already armed before this test started - refusing to proceed")
            return report

        gate_params = _read_params(conn, args.system_id, args.component_id, ("BATT_MONITOR", "FENCE_ENABLE"), 5.0)
        batt_monitor = gate_params.get("BATT_MONITOR")
        report["battery_state"] = {
            "monitor_param_value": batt_monitor, "battery_fraction": telem.battery_fraction,
            "documented_sitl_model": (
                "BATT_MONITOR disabled (0/None) - accepted per documented SITL default (see "
                "docs/PHASE10_SITL_FLIGHT_TEST.md); no live voltage/current reading available this run."
                if not batt_monitor else "battery monitor configured - live reading used"
            ),
        }
        fence_enabled = gate_params.get("FENCE_ENABLE")
        geofence_gate = _check_geofence_gate(fence_enabled)
        report["geofence_gate"] = geofence_gate
        report["geofence_enabled"] = geofence_gate["checks"]["fence_enabled"]["passed"]
        if not geofence_gate["passed"]:
            report["remaining_failures"].append(
                f"geofence gate failed: {geofence_gate['reasons_failed']} - FENCE_ENABLE is 0 (disabled). "
                "A bounded local geofence must be enabled before a flight test; see "
                "docs/PHASE10_SITL_FLIGHT_TEST.md's SITL-only geofence configuration procedure "
                "(FENCE_ENABLE=1, FENCE_TYPE=7, FENCE_ALT_MAX=10, FENCE_RADIUS=10, FENCE_MARGIN=2)."
            )
            return report

        report["prearm_messages"] = [t for t in _collect_statustexts(conn, 6.0) if t.startswith("PreArm")]
        report["prearm_clear"] = not report["prearm_messages"]
        if not report["prearm_clear"]:
            report["remaining_failures"].append(
                f"pre-arm messages present: {report['prearm_messages']} - stopping without arming"
            )
            return report

        mode_ok, mode_result, _mode_texts = _set_mode_guided(conn, args.system_id, args.component_id, 10.0)
        log_event("mode_set_guided", accepted=mode_ok)
        if not mode_ok:
            report["remaining_failures"].append(f"could not set GUIDED mode (MAV_RESULT={mode_result})")
            return report

        report["arm"]["attempted"] = True
        arm_ok, arm_result, arm_texts = _send_command_long_and_wait_ack(
            conn, args.system_id, args.component_id, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            _ARM_TAKEOFF_LAND_COMMAND_TIMEOUT_S, p1=1, p2=0,
        )
        report["arm"]["accepted"] = arm_ok
        report["arm"]["statustexts"] = arm_texts
        log_event("arm_attempt", accepted=arm_ok, result=arm_result)
        if not arm_ok:
            report["remaining_failures"].append(f"arm rejected by ArduPilot: result={arm_result}, statustexts={arm_texts}")
            return report

        confirm_deadline = time.monotonic() + _ARM_CONFIRM_TIMEOUT_S
        confirmed_armed = False
        while time.monotonic() < confirm_deadline:
            telem = transport.receive_telemetry(vehicle_id)
            if telem.armed:
                confirmed_armed = True
                break
            time.sleep(0.1)
        report["arm"]["confirmed_by_heartbeat"] = confirmed_armed
        if not confirmed_armed:
            report["remaining_failures"].append("arm command accepted but heartbeat never confirmed armed state")
            emergency_cleanup("arm not confirmed by heartbeat")
            return report
        armed_this_session[0] = True
        log_event("armed_confirmed")

        report["takeoff"]["attempted"] = True
        takeoff_ok, takeoff_result, takeoff_texts = _send_command_long_and_wait_ack(
            conn, args.system_id, args.component_id, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            _ARM_TAKEOFF_LAND_COMMAND_TIMEOUT_S, p7=args.max_altitude,
        )
        report["takeoff"]["accepted"] = takeoff_ok
        report["takeoff"]["statustexts"] = takeoff_texts
        log_event("takeoff_attempt", accepted=takeoff_ok, result=takeoff_result)
        if not takeoff_ok:
            report["remaining_failures"].append(f"takeoff rejected by ArduPilot: result={takeoff_result}, statustexts={takeoff_texts}")
            emergency_cleanup("takeoff rejected after arming")
            return report

        climb_deadline = time.monotonic() + _TAKEOFF_CLIMB_TIMEOUT_S
        max_alt_observed = 0.0
        while time.monotonic() < climb_deadline:
            telem = transport.receive_telemetry(vehicle_id)
            if telem.position_m is not None:
                max_alt_observed = max(max_alt_observed, telem.position_m[2])
                if telem.position_m[2] >= args.max_altitude * 0.8:
                    break
            time.sleep(0.2)
        report["takeoff"]["max_altitude_observed_m"] = max_alt_observed
        report["flight_actually_happened"] = max_alt_observed > 0.3
        if not report["flight_actually_happened"]:
            report["remaining_failures"].append(
                f"takeoff accepted but altitude never rose meaningfully (observed {max_alt_observed:.2f}m) - "
                "not claiming a successful flight"
            )
            emergency_cleanup("no observed climb after takeoff accepted")
            return report
        log_event("takeoff_confirmed", altitude_m=max_alt_observed)

        hold_deadline = time.monotonic() + args.duration
        sample_count = 0
        while time.monotonic() < hold_deadline:
            transport.receive_telemetry(vehicle_id)
            sample_count += 1
            time.sleep(1.0)
        report["hold"] = {"duration_s": args.duration, "sample_count": sample_count}
        log_event("hold_complete")

        report["land"]["attempted"] = True
        land_ok, land_result, _land_texts = _send_command_long_and_wait_ack(
            conn, args.system_id, args.component_id, mavutil.mavlink.MAV_CMD_NAV_LAND,
            _ARM_TAKEOFF_LAND_COMMAND_TIMEOUT_S,
        )
        report["land"]["accepted"] = land_ok
        log_event("land_attempt", accepted=land_ok, result=land_result)
        if not land_ok:
            report["remaining_failures"].append(f"LAND rejected by ArduPilot: result={land_result}")
            emergency_cleanup("LAND command rejected")
            return report

        disarm_deadline = time.monotonic() + _LAND_DISARM_TIMEOUT_S
        disarmed = False
        while time.monotonic() < disarm_deadline:
            telem = transport.receive_telemetry(vehicle_id)
            if not telem.armed:
                disarmed = True
                break
            time.sleep(0.2)
        if not disarmed:
            # Documented, non-forced safety-net disarm (param2 is always 0 -
            # never the force-arm/disarm magic value) if auto-disarm-after-
            # landing hasn't fired within the bounded window.
            _send_command_long_and_wait_ack(conn, args.system_id, args.component_id,
                                             mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 5.0, p1=0)
            time.sleep(1.0)
            telem = transport.receive_telemetry(vehicle_id)
            disarmed = not telem.armed
            if disarmed:
                report["emergency_action"] = "auto-disarm-after-land did not fire in time; sent one explicit, non-forced disarm"
        report["disarm_confirmed"] = disarmed
        armed_this_session[0] = not disarmed
        if not disarmed:
            report["remaining_failures"].append("vehicle did not disarm after landing - reporting honestly")
    finally:
        stop_result = transport.stop()
        report["shutdown"]["clean"] = stop_result.success

    return report


# --------------------------------------------------------------------------

def main(argv=None) -> None:
    args = build_parser().parse_args(argv)
    os.makedirs(OUT_DIR, exist_ok=True)
    print(_SAFETY_BANNER)

    if args.dry_run:
        result, out_name = run_dry_run(args), "phase10_dry_run_result.json"
    elif args.diagnose_prearm:
        result, out_name = run_prearm_diagnostics(args), "phase10_prearm_diagnostics_result.json"
    elif args.sitl_only and args.allow_sitl_arm_test and args.confirm_sitl_flight_test:
        result, out_name = run_sitl_flight_test(args), "phase10_sitl_flight_test_result.json"
    elif args.attach:
        result, out_name = phase9.run_telemetry_visualization(args), "phase10_telemetry_only_result.json"
    else:
        result, out_name = run_dry_run(args), "phase10_dry_run_result.json"

    print(json.dumps(result, indent=2, default=str))
    with open(os.path.join(OUT_DIR, out_name), "w") as f:
        json.dump(result, f, indent=2, default=str)


if __name__ == "__main__":
    main()
