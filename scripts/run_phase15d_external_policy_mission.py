"""Phase 15D: one-drone live Webots integration for the Swarm124 external
policy adapter - see docs/PHASE15_SWARM124_LANGOSTINO_INTEGRATION.md.

    This runs a bounded external-research-reference policy (a
    deterministic, non-learned ScriptedSwarm124Policy - see
    swarm_sim/external/swarm124_policy_adapter.py) as a candidate-command
    SOURCE only. Every action it produces still passes through
    convert_swarm124_action_to_candidate() (rejecting malformed/stale/
    expired/wrong-vehicle/too-fast actions) and then through the SAME
    SafetySupervisor.evaluate() every other command path in this project
    uses - this file never sends a raw policy output to ArduPilot.

Three explicitly separate modes, same discipline as Phase 14
(scripts/run_phase14_sar_mission.py):

    --dry-run (or no arm-test flags at all - THE DEFAULT, always safe)
        Validates configuration/gate shape only - never opens a socket.

    --offline
        Runs the full offline mission
        (swarm_sim.external_policy_mission.run_external_policy_mission_offline)
        against FakeSITLTransport - no Webots, no ArduPilot SITL, no
        socket to a real process at all. Always safe, always available.

    --webots-only --allow-webots-arm-test --confirm-webots-flight-test
        The live mission: arm -> takeoff (to --search-altitude, hard-capped
        at HARD_MAX_ALTITUDE_M, same limits Phase 14 uses) -> run
        ExternalPolicyMission (swarm_sim.external_policy_mission) driven by
        REAL ArduPilot SITL telemetry via build_ardupilot_sitl_adapter()
        over an already-attached ArduPilotSITLTransport -> once the
        mission reaches LAND_REQUESTED, hand off to the existing, gated
        Phase 13 land/disarm sequence. Every gate below, and operator
        confirmation, must pass before any command that could arm the
        vehicle is ever sent.

**Reuse, not duplication** (see
tests/test_phase15d_external_policy_architecture.py): this file never
redefines arm/takeoff/land/geofence/arming-check/prearm-health gate logic
- it imports Phase 10/12/13's own helpers and Phase 14's own
_check_sar_live_gate/_wait_for_stable_prearm_health/
_wait_for_climb_and_vz_to_settle/_operator_confirmed unmodified, and
imports the entire external-policy mission state machine from
swarm_sim.external_policy_mission unmodified. Neither
scripts/run_phase13_webots_flight_test.py nor
scripts/run_phase14_sar_mission.py is ever imported for its own
run_*_flight_test/run_live_*_mission/main - only their small,
already-tested helper functions - so neither phase's own dedicated
manual/SAR flight-test path is affected by this file's existence.

**Command authority, restated for this phase**: every velocity command
that reaches the vehicle during the live mission has already passed
through swarm_sim.safety_supervisor.SafetySupervisor.evaluate() inside
ExternalPolicyMission.tick() - this file never calls
adapter.send_command()/conn.mav.*_send() with a raw policy action.
This file also never arms or takes off the vehicle from policy output -
arm/takeoff below are the same real, gated MAVLink commands Phase 13/14
use, and only run BEFORE the external policy is ever consulted.
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
from run_phase14_sar_mission import (  # noqa: E402
    HARD_MAX_ALTITUDE_M, HARD_MAX_SPEED_MPS, HARD_MISSION_DURATION_S, REQUIRED_NAMESPACE,
    _ARM_CONFIRM_TIMEOUT_S, _ARM_TAKEOFF_LAND_COMMAND_TIMEOUT_S, _LAND_DISARM_TIMEOUT_S, _MISSION_TICK_DT_S,
    _SERVO_STREAM_RATE_HZ, _TAKEOFF_CLIMB_TIMEOUT_S, _check_sar_live_gate, _operator_confirmed,
    _wait_for_climb_and_vz_to_settle, _wait_for_stable_prearm_health,
)

from swarm_sim.autopilot.ardupilot_sitl import build_ardupilot_sitl_adapter  # noqa: E402
from swarm_sim.external.swarm124_policy_adapter import ScriptedSwarm124Policy  # noqa: E402
from swarm_sim.external_policy_mission import (  # noqa: E402
    ExternalPolicyMission, ExternalPolicyMissionConfig, ExternalPolicyMissionState, TERMINAL_STATES,
    _COMMAND_CSV_FIELDS, _SAFETY_CSV_FIELDS, _TELEMETRY_CSV_FIELDS, _write_csv, run_external_policy_mission_offline,
)
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
                        "phase15d_external_policy_mission")

_SAFETY_BANNER = (
    "Phase 15D: one-drone live Webots integration for the Swarm124 external policy adapter.\n"
    "  - The external policy only ever produces a candidate command - SafetySupervisor is the\n"
    "    final authority for every command actually sent, exactly like Phase 14's SAR mission.\n"
    "  - Webots provides simulated physics/sensors; ArduPilot SITL is the autopilot authority.\n"
    "  - Localhost (127.0.0.1) only - the transport refuses anything else.\n"
    "  - The vehicle only arms/takes off via the SAME real, gated MAVLink commands Phase 13/14\n"
    "    use, BEFORE the external policy is ever consulted - it never arms/takes off itself.\n"
    "  - The vehicle only arms/takes off if EVERY explicit opt-in flag and safety gate below\n"
    "    passes, AND operator confirmation is supplied. Any failure stops the test - never forced.\n"
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Phase 15D: one-drone live external-policy integration (simulator-only).",
    )
    parser.add_argument("--connection", default="tcp:127.0.0.1:5760")
    parser.add_argument("--system-id", type=int, default=1)
    parser.add_argument("--component-id", type=int, default=1)
    parser.add_argument("--namespace", default="sitl/drone0")
    parser.add_argument("--startup-timeout", type=float, default=20.0)
    parser.add_argument("--instance", type=int, default=0)
    parser.add_argument("--ardupilot-root", default=os.environ.get("ARDUPILOT_ROOT"))
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--run-id", default="phase15d-live-run0")

    parser.add_argument("--search-altitude", type=float, default=1.0,
                         help=f"hard-capped at {HARD_MAX_ALTITUDE_M}m")
    parser.add_argument("--target-x", type=float, default=3.0)
    parser.add_argument("--target-y", type=float, default=3.0)
    parser.add_argument("--max-speed", type=float, default=0.25, help=f"hard-capped at {HARD_MAX_SPEED_MPS} m/s")
    parser.add_argument("--duration", type=float, default=60.0,
                         help=f"mission timeout in seconds, hard-capped at {HARD_MISSION_DURATION_S}s")

    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--offline", action="store_true",
                         help="run the full offline mission against FakeSITLTransport - always safe")
    parser.add_argument("--webots-only", action="store_true")
    parser.add_argument("--allow-webots-arm-test", action="store_true")
    parser.add_argument("--confirm-webots-flight-test", action="store_true")
    return parser


def _ep_config_from_args(args) -> ExternalPolicyMissionConfig:
    return ExternalPolicyMissionConfig(
        vehicle_id="drone0", namespace=args.namespace, system_id=args.system_id, component_id=args.component_id,
        seed=args.seed, run_id=args.run_id, home_m=(0.0, 0.0, 0.0), search_altitude_m=args.search_altitude,
        target_xy=(args.target_x, args.target_y), max_speed_mps=args.max_speed, mission_timeout_s=args.duration,
    )


def run_dry_run(args) -> dict:
    gate = _check_sar_live_gate(args)
    return {
        "mode": "dry_run", "ok": True, "connection_string": args.connection, "namespace": args.namespace,
        "gate": gate, "gate_reasons_failed": gate["reasons_failed"],
        "external_policy_config": {
            "target_xy": [args.target_x, args.target_y], "search_altitude_m": args.search_altitude,
            "max_speed_mps": args.max_speed, "mission_timeout_s": args.duration,
        },
    }


def run_offline(args) -> dict:
    config = _ep_config_from_args(args)
    out_dir = os.path.join(OUT_DIR, "offline")
    return run_external_policy_mission_offline(config, out_dir=out_dir)


# --------------------------------------------------------------------------
# Live mode
# --------------------------------------------------------------------------

def run_live_external_policy_mission(args) -> dict:
    report = {
        "mode": "webots_external_policy_mission", "namespace": args.namespace, "connection_string": args.connection,
        "gate": None, "arming_check_gate": None, "geofence_gate": None, "operator_confirmed": False,
        "attach": {"succeeded": False, "reason": None}, "heartbeat": {"received": False},
        "estimator_valid": None, "telemetry_valid": None, "initially_disarmed": None,
        "prearm_clear": None, "prearm_messages": [], "battery_state": {}, "geofence_enabled": None,
        "prearm_health_check": None,
        "version": {"confirmed": False, "raw": None}, "events": [],
        "arm": {"attempted": False, "accepted": None, "confirmed_by_heartbeat": None, "status_texts": []},
        "takeoff": {"attempted": False, "accepted": None, "max_altitude_observed_m": None, "status_texts": []},
        "mission_final_state": None, "mission_summary": None,
        "land": {"attempted": False, "accepted": None, "status_texts": []}, "disarm_confirmed": None,
        "emergency_action": None,
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
    ep_config = _ep_config_from_args(args)
    # Same live-loop-vs-config-default dt_s mismatch fix as Phase 14
    # (scripts/run_phase14_sar_mission.py) - this script's real per-tick
    # sleep is _MISSION_TICK_DT_S, not ExternalPolicyMissionConfig's own
    # default (tuned for a faster offline loop).
    ep_config.dt_s = _MISSION_TICK_DT_S

    def emergency_cleanup(reason):
        report["emergency_action"] = reason
        log_event("emergency_action", reason=reason)
        _send_command_long_and_wait_ack(conn, args.system_id, args.component_id,
                                         mavutil.mavlink.MAV_CMD_NAV_LAND, 5.0)
        time.sleep(1.0)
        _send_command_long_and_wait_ack(conn, args.system_id, args.component_id,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 5.0, p1=0)

    try:
        # ---- version / telemetry / prearm gates (Phase 13/14's own pattern, reused inline) ----
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

        health_check = _wait_for_stable_prearm_health(conn, args.system_id, args.component_id, timeout_s=20.0)
        report["prearm_health_check"] = {
            "stable_healthy": health_check["stable_healthy"], "sample_count": len(health_check["samples"]),
            "last_samples": health_check["samples"][-5:],
        }
        if not health_check["stable_healthy"]:
            report["remaining_failures"].append(
                "PreArm health bit never reported stably healthy within 20s - refusing to arm "
                f"(last samples: {health_check['samples'][-5:]})")
            return report

        # ---- arm + takeoff (Phase 13/14's own gated mechanism, reused inline -
        # the external policy has no involvement whatsoever up to this point) ----
        mode_ok, _mode_result, mode_texts = _set_mode_guided(conn, args.system_id, args.component_id, 10.0)
        log_event("mode_set_guided", accepted=mode_ok, status_texts=mode_texts)
        if not mode_ok:
            report["remaining_failures"].append(f"could not set GUIDED mode - status texts: {mode_texts}")
            return report

        report["arm"]["attempted"] = True
        arm_ok, arm_result, arm_texts = _send_command_long_and_wait_ack(
            conn, args.system_id, args.component_id, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            _ARM_TAKEOFF_LAND_COMMAND_TIMEOUT_S, p1=1, p2=0)
        report["arm"]["accepted"] = arm_ok
        report["arm"]["status_texts"] = arm_texts
        log_event("arm_attempt", accepted=arm_ok, result=arm_result, status_texts=arm_texts)
        if not arm_ok:
            report["remaining_failures"].append(
                f"arm rejected by ArduPilot: result={arm_result} - status texts: {arm_texts}")
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
        takeoff_ok, takeoff_result, takeoff_texts = _send_command_long_and_wait_ack(
            conn, args.system_id, args.component_id, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            _ARM_TAKEOFF_LAND_COMMAND_TIMEOUT_S, p7=ep_config.search_altitude_m)
        report["takeoff"]["accepted"] = takeoff_ok
        report["takeoff"]["status_texts"] = takeoff_texts
        log_event("takeoff_attempt", accepted=takeoff_ok, result=takeoff_result, status_texts=takeoff_texts)
        if not takeoff_ok:
            report["remaining_failures"].append(
                f"takeoff rejected by ArduPilot: result={takeoff_result} - status texts: {takeoff_texts}")
            emergency_cleanup("takeoff rejected after arming")
            return report

        max_alt_observed = _wait_for_climb_and_vz_to_settle(
            transport, vehicle_id, ep_config.search_altitude_m, _TAKEOFF_CLIMB_TIMEOUT_S, HARD_MAX_SPEED_MPS)
        report["takeoff"]["max_altitude_observed_m"] = max_alt_observed
        report["flight_actually_happened"] = max_alt_observed > 0.3
        if not report["flight_actually_happened"]:
            report["remaining_failures"].append(f"takeoff accepted but altitude never rose meaningfully "
                                                  f"(observed {max_alt_observed:.2f}m)")
            emergency_cleanup("no observed climb after takeoff accepted")
            return report
        log_event("takeoff_confirmed", altitude_m=max_alt_observed)

        # ---- the external-policy mission itself: driven by REAL telemetry,
        # every action still gated by SafetySupervisor.evaluate() ----
        policy = ScriptedSwarm124Policy(target_xy=ep_config.target_xy, max_speed_mps=ep_config.max_speed_mps,
                                         seed=ep_config.seed, run_id=ep_config.run_id)
        mission = ExternalPolicyMission(ep_config, policy)
        mission.state = ExternalPolicyMissionState.RUNNING   # arm/takeoff already done for real above

        adapter = build_ardupilot_sitl_adapter(vehicle_id, transport)
        actuator_logger = _ActuatorLogger(os.path.join(OUT_DIR, "live", "actuator_log.csv"))
        mission_start_wall = time.monotonic()
        mission_deadline = mission_start_wall + args.duration + _LAND_DISARM_TIMEOUT_S

        # Same frozen-sim-clock fix as Phase 14 (see
        # scripts/run_phase14_sar_mission.py's own extensive comment on
        # transport.set_sim_time() - ArduPilotSITLTransport never advances
        # its own clock automatically).
        transport.set_sim_time(0.0)
        mission._mission_start_s = 0.0
        try:
            while time.monotonic() < mission_deadline:
                transport.set_sim_time(time.monotonic() - mission_start_wall)
                telem = transport.receive_telemetry(vehicle_id)
                now_s = telem.timestamp_s
                vehicle_telem = adapter.read_telemetry()
                actuator_logger.poll(channel, mission.state.value, telem)
                if mission.state == ExternalPolicyMissionState.LAND_REQUESTED:
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
        report["mission_final_state"] = mission.state.value

        # ---- land + disarm (Phase 13/14's own gated mechanism, reused inline) ----
        report["land"]["attempted"] = True
        land_ok, land_result, land_texts = _send_command_long_and_wait_ack(
            conn, args.system_id, args.component_id, mavutil.mavlink.MAV_CMD_NAV_LAND,
            _ARM_TAKEOFF_LAND_COMMAND_TIMEOUT_S)
        report["land"]["accepted"] = land_ok
        report["land"]["status_texts"] = land_texts
        log_event("land_attempt", accepted=land_ok, result=land_result, status_texts=land_texts)
        if not land_ok:
            report["remaining_failures"].append(
                f"LAND rejected by ArduPilot: result={land_result} - status texts: {land_texts}")
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
            target_altitude_m=ep_config.search_altitude_m, measured_peak_altitude_m=mission.max_altitude_m,
            hold_altitudes_m=[], hard_ceiling_m=HARD_MAX_ALTITUDE_M)
        report["actuator_evidence"] = {
            "messages_received": channel.servo_output_messages_received,
            "log_path": actuator_logger._file.name if hasattr(actuator_logger, "_file") else None,
        }
        report["mission_summary"] = mission.build_summary(mode="live", extra={
            "ardupilot_version": report["version"]["raw"],
            "realtime_factor": "not obtainable via MAVLink - see Webots' own GUI timeline",
        })

        live_out_dir = os.path.join(OUT_DIR, "live")
        os.makedirs(live_out_dir, exist_ok=True)
        manifest = build_run_manifest(config=vars(args), master_seed=args.seed, seed_manager=seed_manager,
                                       map_id="phase15d_external_policy_point_to_point",
                                       model_id="webots_iris_ardupilot_sitl", event_log=mission.events)
        manifest.save_json(os.path.join(live_out_dir, "manifest.json"))
        with open(os.path.join(live_out_dir, "mission_events.jsonl"), "w") as f:
            for event in mission.events:
                f.write(json.dumps(event, default=str) + "\n")
        _write_csv(os.path.join(live_out_dir, "telemetry.csv"), _TELEMETRY_CSV_FIELDS, mission.telemetry_rows)
        _write_csv(os.path.join(live_out_dir, "safety_decisions.csv"), _SAFETY_CSV_FIELDS,
                   mission.safety_decision_rows)
        _write_csv(os.path.join(live_out_dir, "commands.csv"), _COMMAND_CSV_FIELDS, mission.command_rows)
        with open(os.path.join(live_out_dir, "summary.json"), "w") as f:
            json.dump(report["mission_summary"], f, indent=2, default=str)
    finally:
        stop_result = transport.stop()
        report["shutdown"]["clean"] = stop_result.success

    return report


def main(argv=None) -> None:
    args = build_parser().parse_args(argv)
    os.makedirs(OUT_DIR, exist_ok=True)
    print(_SAFETY_BANNER)

    if args.offline:
        result, out_name = run_offline(args), "phase15d_offline_result.json"
    elif args.webots_only and args.allow_webots_arm_test and args.confirm_webots_flight_test:
        result, out_name = run_live_external_policy_mission(args), "phase15d_live_result.json"
    else:
        result, out_name = run_dry_run(args), "phase15d_dry_run_result.json"

    print(json.dumps(result, indent=2, default=str))
    with open(os.path.join(OUT_DIR, out_name), "w") as f:
        json.dump(result, f, indent=2, default=str)


if __name__ == "__main__":
    main()
