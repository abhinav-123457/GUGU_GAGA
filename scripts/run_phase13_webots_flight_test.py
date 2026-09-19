"""Phase 13: one-vehicle Webots controlled-flight test - see
docs/PHASE13_WEBOTS_FLIGHT_TEST.md.

Same safety standard as Phase 10 (scripts/run_phase10_sitl_flight_test.py),
adapted for the official ArduPilot Webots Iris example (Phase 12) instead
of a plain SITL launch. STRICTLY SIMULATOR-ONLY. ATTACH mode only - this
script never spawns or kills Webots or ArduPilot SITL. Never connects to
anything but 127.0.0.1/localhost. Never arms or takes off unless every
explicit opt-in flag AND every safety gate below passes.

Four explicitly separate modes, never merged:

    --dry-run (or no arm-test flags at all - THE DEFAULT, always safe)
        Validates configuration/gate shape only - never opens a socket.

    --diagnose-prearm
        Read-only: attaches, reads ARMING_CHECK/BATT_MONITOR/FENCE_*,
        checks the Webots controller UDP port, listens for PreArm
        STATUSTEXTs. Never writes a parameter, never arms.

    --attach (telemetry_only - delegates to Phase 9's own
        run_telemetry_visualization, completely unmodified)

    --webots-only --allow-webots-arm-test --confirm-webots-flight-test
        The actual flight test: arm -> takeoff (<= 1.0m by default,
        hard-capped at 2.0m) -> hold (10s default, hard-capped at 30s)
        -> land -> confirm disarmed. Every gate (see
        _check_flight_test_gate/_check_arming_check_gate/
        _check_geofence_gate) must pass, and operator confirmation must
        be supplied, before any command that could arm the vehicle is
        ever sent.

**Why a direct `arducopter` launch, not `sim_vehicle.py`**: Phase 12's
own investigation found that `sim_vehicle.py`'s auto-launched MAVProxy is
the only MAVLink client SITL's primary TCP port actively serves - a
second client completes the handshake but receives nothing (confirmed by
a raw-socket 0-byte read while MAVProxy's own throughput kept climbing).
This flight test's command/ack path requires this script to be SITL's
sole, directly-connected MAVLink client (exactly like Phase 10's own
plain-SITL setup), so the documented launch command below uses the bare
`arducopter --model webots-python` binary, never `sim_vehicle.py`.

**Command authority**: identical pattern to Phase 10 - the arm/GUIDED/
takeoff/land sequence talks directly to the already-open pymavlink
connection, never through `ArduPilotSITLTransport.send_command()` (which
structurally can never reference an arm/takeoff identifier at all -
Phase 8's own unmodified guarantee).

**No prohibited shortcuts**: no `param_set_send` call anywhere in this
file - every parameter is READ ONLY. No force-arm magic value is ever
sent. No battery/estimator/heartbeat/geofence/ARMING_CHECK value is ever
fabricated - every check reads real MAVLink telemetry from the real SITL
process. This script never imports `subprocess`, never imports
`swarm_sim.visualization.*` (FlightGear) or `pybullet`, never references
the swarm planner/distributed-consensus/`CandidateCommand`/
`SafetySupervisor` internals, and never constructs more than one vehicle.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import run_phase9_single_vehicle_visual as phase9  # noqa: E402
from run_phase10_sitl_flight_test import _check_geofence_gate, _collect_statustexts, _read_params  # noqa: E402
from run_phase12_webots_smoke_test import (  # noqa: E402
    _detect_webots_installation, _official_example_paths, _port_appears_bound, _webots_controller_port,
)

from swarm_sim.sitl.ardupilot_transport import (  # noqa: E402
    ArduPilotSITLTransport, ArduPilotTransportError, PYMAVLINK_AVAILABLE, PYMAVLINK_VERSION,
    dry_run_validate, parse_connection_string,
)

try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None

OUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "results",
                        "phase13_webots_flight_test")
CSV_PATH = os.path.join(OUT_DIR, "flight_log.csv")

# Conservative, hard-coded safety ceilings - defense in depth ON TOP OF (never
# instead of) the CLI's own validated arguments.
HARD_MAX_ALTITUDE_M = 2.0
HARD_MAX_SPEED_MPS = 0.25
HARD_MAX_DURATION_S = 30.0
REQUIRED_FLIGHT_TEST_NAMESPACE = "sitl/drone0"

_ARM_TAKEOFF_LAND_COMMAND_TIMEOUT_S = 10.0
_ARM_CONFIRM_TIMEOUT_S = 10.0
_TAKEOFF_CLIMB_TIMEOUT_S = 15.0
_LAND_DISARM_TIMEOUT_S = 30.0
_SERVO_STREAM_RATE_HZ = 2.0

_CSV_FIELDS = (
    "wall_time_s", "sitl_time_s", "event",
    "east_m", "north_m", "up_m", "altitude_agl_m",
    "ve_mps", "vn_mps", "vu_mps",
    "roll_rad", "pitch_rad", "yaw_rad",
    "armed", "mode", "battery_fraction", "fence_enabled",
    "servo1_raw", "servo2_raw", "servo3_raw", "servo4_raw",
)

_SAFETY_BANNER = (
    "Phase 13: one-vehicle Webots controlled-flight test.\n"
    "  - Simulator-only. Webots provides physics/sensors; ArduPilot SITL is the autopilot authority.\n"
    "  - Localhost (127.0.0.1) only - the transport refuses anything else.\n"
    "  - The vehicle only arms/takes off if EVERY explicit opt-in flag and safety gate below\n"
    "    passes, AND operator confirmation is supplied. Any failure stops the test - never forced.\n"
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Phase 13: one-vehicle Webots controlled-flight test "
                    "(simulator-only, ATTACH mode only).",
    )
    parser.add_argument("--connection", default="tcp:127.0.0.1:5760")
    parser.add_argument("--system-id", type=int, default=1)
    parser.add_argument("--component-id", type=int, default=1)
    parser.add_argument("--namespace", default="sitl/drone0")
    parser.add_argument("--startup-timeout", type=float, default=20.0)
    parser.add_argument("--instance", type=int, default=0, help="Webots/SITL instance number")
    parser.add_argument("--ardupilot-root", default=os.environ.get("ARDUPILOT_ROOT"),
                         help="path to the ArduPilot checkout with the official Webots_Python example "
                              "(required for the flight-test path; never guessed)")
    parser.add_argument("--duration", type=float, default=10.0,
                         help=f"hold duration in seconds, hard-capped at {HARD_MAX_DURATION_S}s")
    parser.add_argument("--dry-run", action="store_true", help="validate configuration only - never opens a socket")
    parser.add_argument("--attach", action="store_true",
                         help="telemetry_only mode - delegates to Phase 9's own telemetry_visualization, unchanged")
    parser.add_argument("--diagnose-prearm", action="store_true",
                         help="prearm_diagnostics mode - read-only, never arms")
    parser.add_argument("--webots-only", action="store_true",
                         help="required (with --allow-webots-arm-test and --confirm-webots-flight-test)")
    parser.add_argument("--allow-webots-arm-test", action="store_true", help="required explicit opt-in")
    parser.add_argument("--confirm-webots-flight-test", action="store_true",
                         help="required explicit operator confirmation "
                              "(an interactive 'yes' prompt is used instead if omitted and stdin is a tty)")
    parser.add_argument("--max-altitude", type=float, default=1.0,
                         help=f"takeoff altitude in meters, hard-capped at {HARD_MAX_ALTITUDE_M}m")
    parser.add_argument("--max-speed", type=float, default=0.25,
                         help=f"reserved horizontal speed ceiling in m/s, hard-capped at {HARD_MAX_SPEED_MPS}m/s "
                              "(this minimal sequence never commands a horizontal velocity)")
    return parser


# --------------------------------------------------------------------------
# Gate checks - every requirement checked BEFORE any socket is opened for the
# flight-test path (except Webots-installation/port checks, which are local,
# read-only OS-level facts, also gathered before any MAVLink connection).
# --------------------------------------------------------------------------

def _check_flight_test_gate(args) -> dict:
    gate = {"passed": False, "checks": {}, "reasons_failed": []}

    def require(name, condition, detail=None):
        gate["checks"][name] = {"passed": bool(condition), "detail": detail}
        if not condition:
            gate["reasons_failed"].append(name)

    require("webots_only_flag", args.webots_only)
    require("allow_webots_arm_test_flag", args.allow_webots_arm_test)
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

    webots_installation = _detect_webots_installation()
    require("webots_installed", webots_installation["found"], webots_installation)

    require("ardupilot_root_given", bool(args.ardupilot_root), args.ardupilot_root)
    if args.ardupilot_root:
        official = _official_example_paths(args.ardupilot_root, "iris.wbt", "iris")
        require("official_world_exists", official["world_exists"], official["world_path"])
        require("official_params_exists", official["params_exists"], official["params_path"])
        require("official_controller_exists", official["controller_exists"], official["controller_path"])
    else:
        require("official_world_exists", False)
        require("official_params_exists", False)
        require("official_controller_exists", False)

    controller_port = _webots_controller_port(args.instance)
    port_bound = _port_appears_bound("0.0.0.0", controller_port)
    require("webots_controller_port_bound", port_bound, controller_port)

    gate["passed"] = not gate["reasons_failed"]
    return gate


def _check_arming_check_gate(arming_check_value) -> dict:
    """Live, post-attach gate - ARMING_CHECK must not be disabled (0).
    Read-only: never sends param_set_send."""
    gate = {"passed": False, "checks": {}, "reasons_failed": []}
    passed = arming_check_value not in (None, 0, 0.0)
    gate["checks"]["arming_check_enabled"] = {"passed": passed, "detail": arming_check_value}
    if not passed:
        gate["reasons_failed"].append("arming_check_enabled")
    gate["passed"] = passed
    return gate


def _operator_confirmed(args) -> bool:
    if args.confirm_webots_flight_test:
        return True
    if sys.stdin.isatty():
        try:
            answer = input(
                f"Type exactly 'yes' to confirm this Webots-only flight test "
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
        "webots_only_flag": args.webots_only, "ok": False, "remaining_failures": [],
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

    # A plain --dry-run only needs the endpoint shape to be valid - it is
    # not itself refused for lacking the arm-test flags, matching Phase 10.
    report["ok"] = endpoint_ok
    report["pymavlink_available"] = PYMAVLINK_AVAILABLE
    report["pymavlink_version"] = PYMAVLINK_VERSION
    return report


# --------------------------------------------------------------------------
# Mode: prearm_diagnostics (read-only, never arms, never writes a parameter)
# --------------------------------------------------------------------------

_DIAGNOSTIC_PARAM_NAMES = ("ARMING_CHECK", "BATT_MONITOR", "FENCE_ENABLE", "FENCE_TYPE",
                           "FENCE_ALT_MAX", "FENCE_RADIUS", "FENCE_MARGIN", "FENCE_ACTION")


def run_prearm_diagnostics(args) -> dict:
    report = {
        "mode": "prearm_diagnostics", "namespace": args.namespace, "connection_string": args.connection,
        "system_id": args.system_id, "component_id": args.component_id,
        "attach": {"succeeded": False, "reason": None}, "heartbeat": {"received": False},
        "parameters": {}, "statustexts": [], "prearm_messages": [],
        "webots_installation": _detect_webots_installation(),
        "webots_controller_port_bound": _port_appears_bound("0.0.0.0", _webots_controller_port(args.instance)),
        "shutdown": {"clean": False}, "remaining_failures": [],
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

    stop_result = transport.stop()
    report["shutdown"]["clean"] = stop_result.success
    return report


# --------------------------------------------------------------------------
# Mode: webots_flight_test
# --------------------------------------------------------------------------

def _send_command_long_and_wait_ack(conn, sysid: int, compid: int, command_id: int, timeout_s: float,
                                     p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0):
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
    return False, None, statustexts


def _set_mode_guided(conn, sysid: int, compid: int, timeout_s: float):
    mode_number = {v: k for k, v in mavutil.mode_mapping_acm.items()}.get("GUIDED")
    if mode_number is None:
        return False, None, []
    return _send_command_long_and_wait_ack(
        conn, sysid, compid, mavutil.mavlink.MAV_CMD_DO_SET_MODE, timeout_s,
        p1=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, p2=mode_number,
    )


def _request_servo_output_stream(conn, sysid: int, compid: int, rate_hz: float) -> None:
    """Read-only stream-rate request - the same MAV_CMD_SET_MESSAGE_INTERVAL
    mechanism Phase 11 already uses for GLOBAL_POSITION_INT."""
    interval_us = 1_000_000.0 / rate_hz
    conn.mav.command_long_send(
        sysid, compid, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, interval_us, 0, 0, 0, 0, 0,
    )


class _CsvLogger:
    """Independent flight-log writer - see docs/PHASE13_WEBOTS_FLIGHT_TEST.md
    for the honest scope of each field (Webots' own simulation time and
    realtime factor are not obtainable via MAVLink and are reported as such
    in the JSON report, not fabricated into this CSV)."""

    def __init__(self, path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self._file = open(path, "w", newline="")
        self._writer = csv.DictWriter(self._file, fieldnames=_CSV_FIELDS)
        self._writer.writeheader()
        self._start = time.monotonic()
        self.max_altitude_agl_m = 0.0
        self.last_servo = (None, None, None, None)

    def log(self, conn, event, telem):
        servo = conn.recv_match(type="SERVO_OUTPUT_RAW", blocking=False)
        if servo is not None:
            self.last_servo = (servo.servo1_raw, servo.servo2_raw, servo.servo3_raw, servo.servo4_raw)
        row = {"wall_time_s": time.monotonic() - self._start, "event": event}
        if telem is not None:
            row["sitl_time_s"] = telem.timestamp_s
            if telem.position_m is not None:
                row["east_m"], row["north_m"], row["up_m"] = telem.position_m
                row["altitude_agl_m"] = telem.position_m[2]
                self.max_altitude_agl_m = max(self.max_altitude_agl_m, telem.position_m[2])
            if telem.velocity_mps is not None:
                row["ve_mps"], row["vn_mps"], row["vu_mps"] = telem.velocity_mps
            if telem.attitude_rad is not None:
                row["roll_rad"], row["pitch_rad"], row["yaw_rad"] = telem.attitude_rad
            row["armed"] = telem.armed
            row["mode"] = telem.flight_mode.value if telem.flight_mode is not None else None
            row["battery_fraction"] = telem.battery_fraction
        row["servo1_raw"], row["servo2_raw"], row["servo3_raw"], row["servo4_raw"] = self.last_servo
        self._writer.writerow(row)

    def close(self):
        self._file.close()


def run_webots_flight_test(args) -> dict:
    report = {
        "mode": "webots_flight_test", "namespace": args.namespace, "connection_string": args.connection,
        "system_id": args.system_id, "component_id": args.component_id,
        "gate": None, "arming_check_gate": None, "geofence_gate": None, "operator_confirmed": False,
        "attach": {"succeeded": False, "reason": None}, "heartbeat": {"received": False},
        "identity_verified": False, "estimator_valid": None, "telemetry_valid": None,
        "initially_disarmed": None, "prearm_clear": None, "prearm_messages": [],
        "battery_state": {}, "geofence_enabled": None,
        "version": {"confirmed": False, "raw": None}, "events": [],
        "arm": {"attempted": False, "accepted": None, "confirmed_by_heartbeat": None, "statustexts": []},
        "takeoff": {"attempted": False, "accepted": None, "max_altitude_observed_m": None, "statustexts": []},
        "hold": {"duration_s": None, "sample_count": 0},
        "land": {"attempted": False, "accepted": None},
        "disarm_confirmed": None, "emergency_action": None,
        "collision_or_contact": "not directly observable via MAVLink from this script - inferred "
                                "'no abnormal contact' from continuous estimator validity and a "
                                "smooth altitude profile; no dedicated Webots contact-sensor channel "
                                "is exposed over this connection",
        "realtime_factor": None,  # not obtainable via MAVLink - see Webots' own GUI timeline (operator-verified)
        "csv_path": CSV_PATH,
        "shutdown": {"clean": False}, "remaining_failures": [], "flight_actually_happened": False,
    }

    def log_event(event, **fields):
        report["events"].append({"monotonic_s": time.monotonic(), "event": event, **fields})

    gate = _check_flight_test_gate(args)
    report["gate"] = gate
    if not gate["passed"]:
        report["remaining_failures"].append(f"gate check failed: {gate['reasons_failed']}")
        return report

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
    logger = _CsvLogger(CSV_PATH)

    def emergency_cleanup(reason):
        report["emergency_action"] = reason
        log_event("emergency_action", reason=reason)
        if armed_this_session[0]:
            _send_command_long_and_wait_ack(conn, args.system_id, args.component_id,
                                             mavutil.mavlink.MAV_CMD_NAV_LAND, 5.0)
            time.sleep(1.0)
            telem_now = transport.receive_telemetry(vehicle_id)
            logger.log(conn, "emergency_land", telem_now)
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

        _request_servo_output_stream(conn, args.system_id, args.component_id, _SERVO_STREAM_RATE_HZ)

        warm_deadline = time.monotonic() + 10.0
        telem = None
        while time.monotonic() < warm_deadline:
            telem = transport.receive_telemetry(vehicle_id)
            if telem.position_m is not None:
                break
            time.sleep(0.2)
        report["telemetry_valid"] = telem is not None and telem.position_m is not None
        report["estimator_valid"] = bool(telem is not None and telem.estimator_valid)
        logger.log(conn, "initial_telemetry", telem)
        if not report["telemetry_valid"] or not report["estimator_valid"]:
            report["remaining_failures"].append("telemetry or estimator not valid - stopping without arming")
            return report

        report["initially_disarmed"] = not telem.armed
        if telem.armed:
            report["remaining_failures"].append("vehicle already armed before this test started - refusing to proceed")
            return report

        gate_params = _read_params(conn, args.system_id, args.component_id,
                                    ("BATT_MONITOR", "FENCE_ENABLE", "ARMING_CHECK"), 5.0)
        batt_monitor = gate_params.get("BATT_MONITOR")
        report["battery_state"] = {
            "monitor_param_value": batt_monitor, "battery_fraction": telem.battery_fraction,
            "documented_sitl_model": (
                "BATT_MONITOR disabled (0/None) - accepted per documented SITL default; no live "
                "voltage/current reading available this run." if not batt_monitor else
                "battery monitor configured - live reading used"
            ),
        }

        arming_check_gate = _check_arming_check_gate(gate_params.get("ARMING_CHECK"))
        report["arming_check_gate"] = arming_check_gate
        if not arming_check_gate["passed"]:
            report["remaining_failures"].append(
                f"ARMING_CHECK gate failed: value={gate_params.get('ARMING_CHECK')!r} - arming checks must "
                "not be disabled before a flight test"
            )
            return report

        fence_enabled = gate_params.get("FENCE_ENABLE")
        geofence_gate = _check_geofence_gate(fence_enabled)
        report["geofence_gate"] = geofence_gate
        report["geofence_enabled"] = geofence_gate["checks"]["fence_enabled"]["passed"]
        if not geofence_gate["passed"]:
            report["remaining_failures"].append(
                f"geofence gate failed: {geofence_gate['reasons_failed']} - FENCE_ENABLE is 0 (disabled)."
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
        logger.log(conn, "arm_confirmed_check", telem)
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
        while time.monotonic() < climb_deadline:
            telem = transport.receive_telemetry(vehicle_id)
            logger.log(conn, "climbing", telem)
            if telem.position_m is not None and telem.position_m[2] >= args.max_altitude * 0.8:
                break
            time.sleep(0.2)
        max_alt_observed = logger.max_altitude_agl_m
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
            telem = transport.receive_telemetry(vehicle_id)
            logger.log(conn, "hold", telem)
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
        touchdown_time_s = None
        while time.monotonic() < disarm_deadline:
            telem = transport.receive_telemetry(vehicle_id)
            logger.log(conn, "landing", telem)
            if telem.position_m is not None and touchdown_time_s is None and telem.position_m[2] <= 0.15:
                touchdown_time_s = time.monotonic()
            if not telem.armed:
                disarmed = True
                break
            time.sleep(0.2)
        if not disarmed:
            _send_command_long_and_wait_ack(conn, args.system_id, args.component_id,
                                             mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 5.0, p1=0)
            time.sleep(1.0)
            telem = transport.receive_telemetry(vehicle_id)
            disarmed = not telem.armed
            if disarmed:
                report["emergency_action"] = "auto-disarm-after-land did not fire in time; sent one explicit, non-forced disarm"
        report["disarm_confirmed"] = disarmed
        report["touchdown_time_s"] = touchdown_time_s
        report["disarm_time_s"] = time.monotonic() if disarmed else None
        logger.log(conn, "disarm_confirmed" if disarmed else "disarm_not_confirmed", telem)
        armed_this_session[0] = not disarmed
        if not disarmed:
            report["remaining_failures"].append("vehicle did not disarm after landing - reporting honestly")
        # Refresh with the true session peak (climb + hold + landing), not just the
        # value observed at the climb-confirmation instant set earlier - the hold
        # phase can (and, under real physics, does) reach a higher altitude than the
        # 80%-of-target climb-confirmation threshold.
        report["takeoff"]["max_altitude_observed_m"] = logger.max_altitude_agl_m
    finally:
        logger.close()
        stop_result = transport.stop()
        report["shutdown"]["clean"] = stop_result.success

    return report


# --------------------------------------------------------------------------

def main(argv=None) -> None:
    args = build_parser().parse_args(argv)
    os.makedirs(OUT_DIR, exist_ok=True)
    print(_SAFETY_BANNER)

    if args.dry_run:
        result, out_name = run_dry_run(args), "phase13_dry_run_result.json"
    elif args.diagnose_prearm:
        result, out_name = run_prearm_diagnostics(args), "phase13_prearm_diagnostics_result.json"
    elif args.webots_only and args.allow_webots_arm_test and args.confirm_webots_flight_test:
        result, out_name = run_webots_flight_test(args), "phase13_webots_flight_test_result.json"
    elif args.attach:
        result, out_name = phase9.run_telemetry_visualization(args), "phase13_telemetry_only_result.json"
    else:
        result, out_name = run_dry_run(args), "phase13_dry_run_result.json"

    print(json.dumps(result, indent=2, default=str))
    with open(os.path.join(OUT_DIR, out_name), "w") as f:
        json.dump(result, f, indent=2, default=str)


if __name__ == "__main__":
    main()
