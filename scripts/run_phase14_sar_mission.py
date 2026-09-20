"""Phase 14: one-vehicle simulated SAR mission - see docs/PHASE14_SAR_WEBOTS.md.

    This is a one-drone simulated SAR mission prototype - not an
    operational search-and-rescue system. Webots provides simulated
    physics and sensors; ArduPilot SITL provides autopilot control. The
    swarm planner is not yet integrated. No Pixhawk, serial port, radio,
    real aircraft, outdoor vehicle, motor/PWM hardware, or
    hardware-in-the-loop was used.

Three explicitly separate modes, never merged:

    --dry-run (or no arm-test flags at all - THE DEFAULT, always safe)
        Validates configuration/gate shape only - never opens a socket.

    --offline
        Runs the full offline mission (swarm_sim.sar_mission.run_sar_mission_offline)
        against FakeSITLTransport - no Webots, no ArduPilot SITL, no socket
        to a real process at all. Always safe, always available.

    --webots-only --allow-webots-arm-test --confirm-webots-flight-test
        The live mission: arm -> takeoff (to --search-altitude, hard-capped
        at HARD_MAX_ALTITUDE_M) -> run the SAME SARMission state machine
        (swarm_sim.sar_mission) that the offline mode uses, driven by REAL
        ArduPilot SITL telemetry via build_ardupilot_sitl_adapter() over an
        already-attached ArduPilotSITLTransport -> once SARMission reaches
        LAND_REQUESTED, hand off to the existing, gated Phase 13 land/
        disarm sequence (never a velocity-setpoint "descent"). Every gate
        below, and operator confirmation, must pass before any command
        that could arm the vehicle is ever sent.

**Reuse, not duplication** (see tests/test_phase14_sar_architecture.py):
this file never redefines arm/takeoff/land/geofence/arming-check gate
logic - it imports Phase 12's Webots-detection helpers and Phase 13's
arm/takeoff/land helpers directly, and imports the entire mission state
machine, search pattern, and sensor/world model from swarm_sim.sar_mission/
sar_search/sar_world unmodified. scripts/run_phase13_webots_flight_test.py
itself is never imported for its own run_webots_flight_test/main - only
its small, already-tested helper functions - so Phase 13's own dedicated
manual flight-test path is completely unaffected by this file's existence.

**Command authority, restated for this phase**: every velocity command
that reaches the vehicle during the SEARCH/RETURN_HOME phases has already
passed through swarm_sim.safety_supervisor.SafetySupervisor.evaluate()
inside SARMission.tick() - this file never calls
adapter.send_command()/conn.mav.*_send() with a raw planner candidate.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from run_phase10_sitl_flight_test import _check_geofence_gate, _collect_statustexts, _read_params  # noqa: E402
from run_phase12_webots_smoke_test import (  # noqa: E402
    _detect_webots_installation, _official_example_paths, _port_appears_bound, _webots_controller_port,
)
from run_phase13_webots_flight_test import (  # noqa: E402
    _ActuatorLogger, _check_arming_check_gate, _classify_altitude_result, _request_servo_output_stream,
    _send_command_long_and_wait_ack, _set_mode_guided,
)

from swarm_sim.autopilot.ardupilot_sitl import build_ardupilot_sitl_adapter  # noqa: E402
from swarm_sim.sar_mission import (  # noqa: E402
    _ACTUATOR_CSV_FIELDS, _COMMAND_CSV_FIELDS, _DETECTION_CSV_FIELDS, _SAFETY_CSV_FIELDS, _TELEMETRY_CSV_FIELDS,
    _write_csv, MissionState, SARMission, SARMissionConfig, TERMINAL_STATES, run_sar_mission_offline,
)
from swarm_sim.sar_world import RectangularSearchArea  # noqa: E402
from swarm_sim.manifest import build_run_manifest  # noqa: E402
from swarm_sim.seeding import SeedManager  # noqa: E402
from swarm_sim.sitl.ardupilot_transport import (  # noqa: E402
    ArduPilotSITLTransport, ArduPilotTransportError, PYMAVLINK_AVAILABLE, PYMAVLINK_VERSION,
    dry_run_validate, parse_connection_string,
)

import run_phase9_single_vehicle_visual as phase9  # noqa: E402

try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None

OUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "results",
                        "phase14_sar_mission")

# Conservative, hard-coded safety ceilings for the LIVE mode - unrelated to
# (and never loosening) Phase 13's own HARD_MAX_ALTITUDE_M/SPEED/DURATION,
# but deliberately matching their values per the phase's own "Use
# conservative live limits initially" requirement.
HARD_MAX_ALTITUDE_M = 2.0
HARD_MAX_SPEED_MPS = 0.25
HARD_MISSION_DURATION_S = 90.0
REQUIRED_NAMESPACE = "sitl/drone0"

_ARM_TAKEOFF_LAND_COMMAND_TIMEOUT_S = 10.0
_ARM_CONFIRM_TIMEOUT_S = 10.0
_TAKEOFF_CLIMB_TIMEOUT_S = 15.0
_LAND_DISARM_TIMEOUT_S = 30.0
_SERVO_STREAM_RATE_HZ = 2.0
_MISSION_TICK_DT_S = 1.0

_SAFETY_BANNER = (
    "Phase 14: one-drone simulated SAR mission prototype.\n"
    "  - Webots provides simulated physics/sensors; ArduPilot SITL is the autopilot authority.\n"
    "  - SafetySupervisor is the final command authority for every search/return-home command.\n"
    "  - Localhost (127.0.0.1) only - the transport refuses anything else.\n"
    "  - The vehicle only arms/takes off if EVERY explicit opt-in flag and safety gate below\n"
    "    passes, AND operator confirmation is supplied. Any failure stops the test - never forced.\n"
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Phase 14: one-drone simulated SAR mission prototype (simulator-only).",
    )
    parser.add_argument("--connection", default="tcp:127.0.0.1:5760")
    parser.add_argument("--system-id", type=int, default=1)
    parser.add_argument("--component-id", type=int, default=1)
    parser.add_argument("--namespace", default="sitl/drone0")
    parser.add_argument("--startup-timeout", type=float, default=20.0)
    parser.add_argument("--instance", type=int, default=0)
    parser.add_argument("--ardupilot-root", default=os.environ.get("ARDUPILOT_ROOT"))
    parser.add_argument("--seed", type=int, default=0)

    parser.add_argument("--search-width-m", type=float, default=10.0)
    parser.add_argument("--search-height-m", type=float, default=10.0)
    parser.add_argument("--search-altitude", type=float, default=1.0,
                         help=f"hard-capped at {HARD_MAX_ALTITUDE_M}m")
    parser.add_argument("--lane-spacing-m", type=float, default=3.0)
    parser.add_argument("--victim-x", type=float, default=5.0)
    parser.add_argument("--victim-y", type=float, default=5.0)
    parser.add_argument("--max-speed", type=float, default=0.25, help=f"hard-capped at {HARD_MAX_SPEED_MPS} m/s")
    parser.add_argument("--duration", type=float, default=90.0,
                         help=f"mission timeout in seconds, hard-capped at {HARD_MISSION_DURATION_S}s")

    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--offline", action="store_true",
                         help="run the full offline mission against FakeSITLTransport - always safe")
    parser.add_argument("--webots-only", action="store_true")
    parser.add_argument("--allow-webots-arm-test", action="store_true")
    parser.add_argument("--confirm-webots-flight-test", action="store_true")
    return parser


def _sar_config_from_args(args) -> SARMissionConfig:
    return SARMissionConfig(
        vehicle_id="drone0", namespace=args.namespace, system_id=args.system_id, component_id=args.component_id,
        seed=args.seed, home_m=(0.0, 0.0, 0.0),
        search_area=RectangularSearchArea(0.0, 0.0, args.search_width_m, args.search_height_m),
        search_altitude_m=args.search_altitude, lane_spacing_m=args.lane_spacing_m, geofence_margin_m=1.0,
        victim_positions_m=((args.victim_x, args.victim_y),), max_speed_mps=args.max_speed,
        mission_timeout_s=args.duration,
    )


def _check_sar_live_gate(args) -> dict:
    checks = {}

    def require(name, ok, detail=None):
        checks[name] = {"passed": bool(ok), "detail": detail}

    require("webots_only_flag", args.webots_only)
    require("allow_webots_arm_test_flag", args.allow_webots_arm_test)
    require("namespace_is_sitl_drone0", args.namespace == REQUIRED_NAMESPACE, args.namespace)
    require("search_altitude_within_hard_cap", 0.0 < args.search_altitude <= HARD_MAX_ALTITUDE_M, args.search_altitude)
    require("max_speed_within_hard_cap", 0.0 < args.max_speed <= HARD_MAX_SPEED_MPS, args.max_speed)
    require("duration_within_hard_cap", 0.0 < args.duration <= HARD_MISSION_DURATION_S, args.duration)
    require("system_id_configured", isinstance(args.system_id, int) and args.system_id > 0, args.system_id)
    require("component_id_configured", isinstance(args.component_id, int) and args.component_id > 0, args.component_id)
    require("pymavlink_available", PYMAVLINK_AVAILABLE)

    try:
        proto, host, port = parse_connection_string(args.connection)
        require("localhost_connection", host in ("127.0.0.1", "localhost"), host)
    except Exception as e:  # noqa: BLE001 - reported as a failed gate, never raised
        require("localhost_connection", False, str(e))

    webots_install = _detect_webots_installation()
    require("webots_installed", webots_install["found"], webots_install)
    require("ardupilot_root_given", bool(args.ardupilot_root), args.ardupilot_root)
    if args.ardupilot_root:
        paths = _official_example_paths(args.ardupilot_root, "iris.wbt", "iris")
        require("official_world_exists", paths["world_exists"], paths["world_path"])
        require("official_params_exists", paths["params_exists"], paths["params_path"])
        require("official_controller_exists", paths["controller_exists"], paths["controller_path"])
        port_num = _webots_controller_port(args.instance)
        require("webots_controller_port_bound", _port_appears_bound("127.0.0.1", port_num), port_num)
    else:
        require("official_world_exists", False)
        require("official_params_exists", False)
        require("official_controller_exists", False)
        require("webots_controller_port_bound", False)

    reasons_failed = [name for name, c in checks.items() if not c["passed"]]
    return {"passed": not reasons_failed, "checks": checks, "reasons_failed": reasons_failed}


def _operator_confirmed(args) -> bool:
    if args.confirm_webots_flight_test:
        return True
    if not sys.stdin.isatty():
        return False
    answer = input("Type 'yes' to confirm the live Phase 14 SAR mission (arm/takeoff/search/land): ")
    return answer.strip().lower() == "yes"


def run_dry_run(args) -> dict:
    gate = _check_sar_live_gate(args)
    return {
        "mode": "dry_run", "ok": True, "connection_string": args.connection, "namespace": args.namespace,
        "gate": gate, "gate_reasons_failed": gate["reasons_failed"],
        "sar_config": {
            "search_area_m": [args.search_width_m, args.search_height_m], "search_altitude_m": args.search_altitude,
            "lane_spacing_m": args.lane_spacing_m, "victim_m": [args.victim_x, args.victim_y],
            "max_speed_mps": args.max_speed, "mission_timeout_s": args.duration,
        },
    }


def run_offline(args) -> dict:
    config = _sar_config_from_args(args)
    out_dir = os.path.join(OUT_DIR, "offline")
    return run_sar_mission_offline(config, out_dir=out_dir)


# --------------------------------------------------------------------------
# Live mode
# --------------------------------------------------------------------------

def run_live_sar_mission(args) -> dict:
    report = {
        "mode": "webots_sar_mission", "namespace": args.namespace, "connection_string": args.connection,
        "gate": None, "arming_check_gate": None, "geofence_gate": None, "operator_confirmed": False,
        "attach": {"succeeded": False, "reason": None}, "heartbeat": {"received": False},
        "estimator_valid": None, "telemetry_valid": None, "initially_disarmed": None,
        "prearm_clear": None, "prearm_messages": [], "battery_state": {}, "geofence_enabled": None,
        "version": {"confirmed": False, "raw": None}, "events": [],
        "arm": {"attempted": False, "accepted": None, "confirmed_by_heartbeat": None},
        "takeoff": {"attempted": False, "accepted": None, "max_altitude_observed_m": None},
        "sar_mission_final_state": None, "sar_summary": None,
        "land": {"attempted": False, "accepted": None}, "disarm_confirmed": None, "emergency_action": None,
        "actuator_evidence": None, "altitude_result": None, "realtime_factor": None,
        "shutdown": {"clean": False}, "remaining_failures": [], "flight_actually_happened": False,
    }

    def log_event(event, **fields):
        report["events"].append({"monotonic_s": time.monotonic(), "event": event, **fields})

    gate = _check_sar_live_gate(args)
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
    log_event("attached")

    seed_manager = SeedManager(args.seed)
    sar_config = _sar_config_from_args(args)

    def emergency_cleanup(reason):
        report["emergency_action"] = reason
        log_event("emergency_action", reason=reason)
        _send_command_long_and_wait_ack(conn, args.system_id, args.component_id,
                                         mavutil.mavlink.MAV_CMD_NAV_LAND, 5.0)
        time.sleep(1.0)
        _send_command_long_and_wait_ack(conn, args.system_id, args.component_id,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 5.0, p1=0)

    try:
        # ---- version / telemetry / prearm gates (Phase 13's own pattern, reused inline) ----
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
        if not report["telemetry_valid"] or not report["estimator_valid"]:
            report["remaining_failures"].append("telemetry or estimator not valid - stopping without arming")
            return report

        report["initially_disarmed"] = not telem.armed
        if telem.armed:
            report["remaining_failures"].append("vehicle already armed before this mission started - refusing")
            return report

        gate_params = _read_params(conn, args.system_id, args.component_id,
                                    ("BATT_MONITOR", "FENCE_ENABLE", "ARMING_CHECK"), 5.0)
        report["battery_state"] = {"monitor_param_value": gate_params.get("BATT_MONITOR"),
                                    "battery_fraction": telem.battery_fraction}

        arming_check_gate = _check_arming_check_gate(gate_params.get("ARMING_CHECK"))
        report["arming_check_gate"] = arming_check_gate
        if not arming_check_gate["passed"]:
            report["remaining_failures"].append("ARMING_CHECK gate failed - arming checks must not be disabled")
            return report

        geofence_gate = _check_geofence_gate(gate_params.get("FENCE_ENABLE"))
        report["geofence_gate"] = geofence_gate
        report["geofence_enabled"] = geofence_gate["checks"]["fence_enabled"]["passed"]
        if not geofence_gate["passed"]:
            report["remaining_failures"].append("geofence gate failed - FENCE_ENABLE is 0 (disabled)")
            return report

        report["prearm_messages"] = [t for t in _collect_statustexts(conn, 6.0) if t.startswith("PreArm")]
        report["prearm_clear"] = not report["prearm_messages"]
        if not report["prearm_clear"]:
            report["remaining_failures"].append(f"pre-arm messages present: {report['prearm_messages']}")
            return report

        # ---- arm + takeoff (Phase 13's own gated mechanism, reused inline) ----
        mode_ok, _mode_result, _texts = _set_mode_guided(conn, args.system_id, args.component_id, 10.0)
        log_event("mode_set_guided", accepted=mode_ok)
        if not mode_ok:
            report["remaining_failures"].append("could not set GUIDED mode")
            return report

        report["arm"]["attempted"] = True
        arm_ok, arm_result, _texts = _send_command_long_and_wait_ack(
            conn, args.system_id, args.component_id, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            _ARM_TAKEOFF_LAND_COMMAND_TIMEOUT_S, p1=1, p2=0)
        report["arm"]["accepted"] = arm_ok
        log_event("arm_attempt", accepted=arm_ok, result=arm_result)
        if not arm_ok:
            report["remaining_failures"].append(f"arm rejected by ArduPilot: result={arm_result}")
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
        log_event("armed_confirmed")

        report["takeoff"]["attempted"] = True
        takeoff_ok, takeoff_result, _texts = _send_command_long_and_wait_ack(
            conn, args.system_id, args.component_id, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            _ARM_TAKEOFF_LAND_COMMAND_TIMEOUT_S, p7=sar_config.search_altitude_m)
        report["takeoff"]["accepted"] = takeoff_ok
        log_event("takeoff_attempt", accepted=takeoff_ok, result=takeoff_result)
        if not takeoff_ok:
            report["remaining_failures"].append(f"takeoff rejected by ArduPilot: result={takeoff_result}")
            emergency_cleanup("takeoff rejected after arming")
            return report

        climb_deadline = time.monotonic() + _TAKEOFF_CLIMB_TIMEOUT_S
        max_alt_observed = 0.0
        while time.monotonic() < climb_deadline:
            telem = transport.receive_telemetry(vehicle_id)
            if telem.position_m is not None:
                max_alt_observed = max(max_alt_observed, telem.position_m[2])
                if telem.position_m[2] >= sar_config.search_altitude_m * 0.8:
                    break
            time.sleep(0.2)
        report["takeoff"]["max_altitude_observed_m"] = max_alt_observed
        report["flight_actually_happened"] = max_alt_observed > 0.3
        if not report["flight_actually_happened"]:
            report["remaining_failures"].append(f"takeoff accepted but altitude never rose meaningfully "
                                                  f"(observed {max_alt_observed:.2f}m)")
            emergency_cleanup("no observed climb after takeoff accepted")
            return report
        log_event("takeoff_confirmed", altitude_m=max_alt_observed)

        # ---- the SAR mission itself: SAME SARMission driving REAL telemetry ----
        world_module = __import__("swarm_sim.sar_world", fromlist=["SARWorld"])
        world = world_module.SARWorld(sar_config.search_area, sar_config.victim_positions_m,
                                       rng=seed_manager.rng("victim_sensor"), sensor_config=sar_config.sensor_config,
                                       vehicle_id=sar_config.vehicle_id)
        mission = SARMission(sar_config, world)
        mission.state = MissionState.TAKEOFF_REQUESTED   # arm/takeoff already done for real above
        mission._mission_start_s = telem.timestamp_s

        adapter = build_ardupilot_sitl_adapter(vehicle_id, transport)
        actuator_logger = _ActuatorLogger(os.path.join(OUT_DIR, "live", "actuator_log.csv"))
        mission_start_wall = time.monotonic()
        mission_deadline = mission_start_wall + args.duration + _LAND_DISARM_TIMEOUT_S
        try:
            while time.monotonic() < mission_deadline:
                telem = transport.receive_telemetry(vehicle_id)
                now_s = telem.timestamp_s
                vehicle_telem = adapter.read_telemetry()
                actuator_logger.poll(channel, mission.state.value, telem)
                if mission.state == MissionState.LAND_REQUESTED:
                    break
                tick_result = mission.tick(now_s, vehicle_telem)
                adapter_result = None
                if tick_result.adapter_command is not None:
                    adapter_result = adapter.send_command(tick_result.adapter_command)
                mission.record_command_result(tick_result, adapter_result, now_s)
                if mission.state in TERMINAL_STATES:
                    break
                time.sleep(_MISSION_TICK_DT_S)
        finally:
            actuator_logger.close()
        report["sar_mission_final_state"] = mission.state.value

        # ---- land + disarm (Phase 13's own gated mechanism, reused inline) ----
        report["land"]["attempted"] = True
        land_ok, land_result, _texts = _send_command_long_and_wait_ack(
            conn, args.system_id, args.component_id, mavutil.mavlink.MAV_CMD_NAV_LAND,
            _ARM_TAKEOFF_LAND_COMMAND_TIMEOUT_S)
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
            _send_command_long_and_wait_ack(conn, args.system_id, args.component_id,
                                             mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 5.0, p1=0)
            time.sleep(1.0)
            telem = transport.receive_telemetry(vehicle_id)
            disarmed = not telem.armed
        report["disarm_confirmed"] = disarmed
        if not disarmed:
            report["remaining_failures"].append("vehicle did not disarm after landing - reporting honestly")

        report["altitude_result"] = _classify_altitude_result(
            target_altitude_m=sar_config.search_altitude_m, measured_peak_altitude_m=mission.max_altitude_m,
            hold_altitudes_m=mission.hold_band_altitudes_m, hard_ceiling_m=HARD_MAX_ALTITUDE_M)
        report["actuator_evidence"] = {
            "messages_received": channel.servo_output_messages_received,
            "log_path": actuator_logger._file.name if hasattr(actuator_logger, "_file") else None,
        }
        report["sar_summary"] = mission.build_summary(mode="live", extra={
            "webots_version": None, "ardupilot_version": report["version"]["raw"],
            "realtime_factor": "not obtainable via MAVLink - see Webots' own GUI timeline",
        })

        live_out_dir = os.path.join(OUT_DIR, "live")
        os.makedirs(live_out_dir, exist_ok=True)
        manifest = build_run_manifest(config=vars(args), master_seed=args.seed, seed_manager=seed_manager,
                                       map_id="phase14_sar_rectangular_area", model_id="webots_iris_ardupilot_sitl",
                                       event_log=mission.events)
        manifest.save_json(os.path.join(live_out_dir, "manifest.json"))
        with open(os.path.join(live_out_dir, "mission_events.jsonl"), "w") as f:
            for event in mission.events:
                f.write(json.dumps(event, default=str) + "\n")
        _write_csv(os.path.join(live_out_dir, "telemetry.csv"), _TELEMETRY_CSV_FIELDS, mission.telemetry_rows)
        _write_csv(os.path.join(live_out_dir, "safety_decisions.csv"), _SAFETY_CSV_FIELDS,
                   mission.safety_decision_rows)
        _write_csv(os.path.join(live_out_dir, "commands.csv"), _COMMAND_CSV_FIELDS, mission.command_rows)
        _write_csv(os.path.join(live_out_dir, "detections.csv"), _DETECTION_CSV_FIELDS, mission.detection_rows)
        with open(os.path.join(live_out_dir, "summary.json"), "w") as f:
            json.dump(report["sar_summary"], f, indent=2, default=str)
    finally:
        stop_result = transport.stop()
        report["shutdown"]["clean"] = stop_result.success

    return report


def main(argv=None) -> None:
    args = build_parser().parse_args(argv)
    os.makedirs(OUT_DIR, exist_ok=True)
    print(_SAFETY_BANNER)

    if args.offline:
        result, out_name = run_offline(args), "phase14_offline_result.json"
    elif args.webots_only and args.allow_webots_arm_test and args.confirm_webots_flight_test:
        result, out_name = run_live_sar_mission(args), "phase14_live_sar_mission_result.json"
    else:
        result, out_name = run_dry_run(args), "phase14_dry_run_result.json"

    print(json.dumps(result, indent=2, default=str))
    with open(os.path.join(OUT_DIR, out_name), "w") as f:
        json.dump(result, f, indent=2, default=str)


if __name__ == "__main__":
    main()
