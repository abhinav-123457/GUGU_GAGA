"""Phase 9: one-vehicle real ArduPilot SITL visual integration - see
docs/PHASE9_SINGLE_VEHICLE_VISUAL.md.

ATTACH MODE ONLY. This script never launches or kills a process (see
tests/test_phase9_single_vehicle_visual_architecture.py's "no process
spawning/killing" checks) - it only ever connects to an ArduCopter SITL
instance the OPERATOR already started manually, e.g.:

    cd /home/swarmbuild/ardupilot
    ./build/sitl/bin/arducopter --model quad --speedup 1 -I0 \\
        --home -35.363261,149.165230,584,353 --sysid 1

Real ArduPilot SITL is the simulated autopilot and simulated vehicle - it
is NOT a real drone. Mission Planner/MAVProxy (started separately by the
operator, connecting directly to the same localhost endpoint) provide the
visual map/HUD - THIS SCRIPT DOES NOT RENDER A 3D VIEW and is NOT a
PyBullet simulation. This phase does not control or demonstrate the full
six-drone search-and-rescue swarm, and never arms or takes off.

Four explicitly separate modes (never merged - see module's own
docstrings and docs/PHASE9_SINGLE_VEHICLE_VISUAL.md's "modes" section):

    --dry-run (or no --attach at all - this is the default, safe mode)
        Validates configuration only (localhost-ness, port, system/
        component id, namespace shape) - never opens a socket.

            python scripts/run_phase9_single_vehicle_visual.py --dry-run \\
                --connection tcp:127.0.0.1:5760 --system-id 1 \\
                --component-id 1 --namespace sitl/drone0

    --attach (telemetry_visualization mode)
        Attaches to the operator-started SITL instance, requests
        telemetry, and prints a continuously updated compact status line
        for --duration seconds (Ctrl+C stops early, cleanly).

            python scripts/run_phase9_single_vehicle_visual.py --attach \\
                --connection tcp:127.0.0.1:5760 --system-id 1 \\
                --component-id 1 --namespace sitl/drone0 --duration 120

    --attach --hold-test (hold_command_test mode)
        Sends one safe, non-arming HOLD command and verifies its
        acknowledgement, then attempts one VELOCITY_SETPOINT command
        while the vehicle is disarmed and records the correct safe
        result (vehicle stays disarmed and stationary) - never forces
        arming to make it "succeed".

            python scripts/run_phase9_single_vehicle_visual.py --attach \\
                --hold-test --connection tcp:127.0.0.1:5760 \\
                --system-id 1 --component-id 1 --namespace sitl/drone0

    --attach --planner-preview (planner_preview mode)
        Runs one minimal, deterministic single-drone demo planner
        through the real SafetySupervisor before anything reaches the
        adapter/transport - every CandidateCommand, SafetyDecision,
        AdapterCommand, transport result, and acknowledgement is
        recorded. Never arms, never takes off; the vehicle may (and, in
        this phase, always does) remain stationary while disarmed.

            python scripts/run_phase9_single_vehicle_visual.py --attach \\
                --planner-preview --connection tcp:127.0.0.1:5760 \\
                --system-id 1 --component-id 1 --namespace sitl/drone0

`autopilot_path == "ardupilot_sitl"` (swarm_sim/config.py, Phase 8) already
requires explicit localhost connection configuration - this script does
not add a second configuration mechanism; it talks to
swarm_sim.sitl.ardupilot_transport.ArduPilotSITLTransport directly,
deliberately never constructing a full swarm_sim.mission.FloodSearchMission
(that would pull in PyBullet and the full six-drone pipeline, both out of
scope this phase - see docs/PHASE9_SINGLE_VEHICLE_VISUAL.md).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from swarm_sim.autopilot import AdapterCommand
from swarm_sim.autopilot.ardupilot_sitl import build_ardupilot_sitl_adapter
from swarm_sim.contracts import Command, CommandType, Frame, GeofenceSpec, HealthState, SensorObservation, VehicleState
from swarm_sim.safety_supervisor import CandidateCommand, MissionContext, SafetySupervisor
from swarm_sim.sitl.ardupilot_transport import (
    ArduPilotSITLTransport, ArduPilotTransportError, ArduPilotVehicleEndpoint, PYMAVLINK_AVAILABLE,
    PYMAVLINK_VERSION, dry_run_validate, parse_connection_string,
)
from swarm_sim.sitl.telemetry import SITLTelemetry

OUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "results",
                        "phase9_single_vehicle_visual")

_NAMESPACE_PREFIX = "sitl/"

_SAFETY_BANNER = (
    "Phase 9: one-vehicle real ArduPilot SITL visual integration (ATTACH mode only).\n"
    "  - Real ArduPilot SITL is a simulated autopilot + simulated vehicle - this is NOT a real drone.\n"
    "  - This script does not render a 3D view and is NOT a PyBullet simulation - the visual map/HUD\n"
    "    comes from Mission Planner/MAVProxy connecting independently to the same localhost endpoint.\n"
    "  - The vehicle must remain disarmed. This script never arms, never takes off, and never spawns\n"
    "    or kills any process (ATTACH mode only - the operator starts ArduCopter SITL manually).\n"
    "  - This does not control or demonstrate the full six-drone search-and-rescue swarm.\n"
)


def vehicle_id_from_namespace(namespace: str) -> str:
    """Namespaces are always exactly f"sitl/{vehicle_id}" (see
    swarm_sim/sitl/vehicle_namespace.py's namespace_for) - never a
    separately settable value. Raises ValueError on any other shape."""
    if not isinstance(namespace, str) or not namespace.startswith(_NAMESPACE_PREFIX) or namespace == _NAMESPACE_PREFIX:
        raise ValueError(
            f"--namespace must be exactly 'sitl/<vehicle_id>' (see swarm_sim/sitl/vehicle_namespace.py's "
            f"namespace_for), got {namespace!r}"
        )
    return namespace[len(_NAMESPACE_PREFIX):]


_HELP_EPILOG = """\
Example (PowerShell - single line, simplest, always works):
  python scripts/run_phase9_single_vehicle_visual.py --attach --connection tcp:127.0.0.1:5760 --system-id 1 --component-id 1 --namespace sitl/drone0 --duration 120

Example (PowerShell - multiline, use a backtick ` - NOT a trailing backslash):
  python scripts/run_phase9_single_vehicle_visual.py `
    --attach `
    --connection tcp:127.0.0.1:5760 `
    --duration 120

Example (WSL / Linux Bash - multiline, use a trailing backslash \\):
  python scripts/run_phase9_single_vehicle_visual.py \\
    --attach \\
    --connection tcp:127.0.0.1:5760 \\
    --duration 120

Example (Windows CMD - multiline, use a caret ^):
  python scripts/run_phase9_single_vehicle_visual.py ^
    --attach ^
    --connection tcp:127.0.0.1:5760 ^
    --duration 120

See docs/PHASE9_SINGLE_VEHICLE_VISUAL.md's "Command syntax by platform"
section for the full reference (WSL/Windows setup, execution-policy notes).
"""


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Phase 9: one-vehicle real ArduPilot SITL visual integration (ATTACH mode only).",
        epilog=_HELP_EPILOG, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--connection", default="tcp:127.0.0.1:5760",
                         help="pymavlink connection string to an ALREADY RUNNING ArduCopter SITL instance - "
                              "localhost only, never spawned by this script")
    parser.add_argument("--system-id", type=int, default=1)
    parser.add_argument("--component-id", type=int, default=1)
    parser.add_argument("--namespace", default="sitl/drone0",
                         help="must be exactly 'sitl/<vehicle_id>'")
    parser.add_argument("--duration", type=float, default=30.0,
                         help="seconds to keep telemetry_visualization/planner_preview running")
    parser.add_argument("--dry-run", action="store_true",
                         help="validate configuration only - never opens a socket (default if --attach is omitted)")
    parser.add_argument("--attach", action="store_true",
                         help="attach to the already-running SITL instance - never spawns or kills a process")
    parser.add_argument("--hold-test", action="store_true",
                         help="hold_command_test mode (requires --attach)")
    parser.add_argument("--planner-preview", action="store_true",
                         help="planner_preview mode: one drone's demo planner + real SafetySupervisor "
                              "(requires --attach)")
    parser.add_argument("--startup-timeout", type=float, default=20.0,
                         help="bounded seconds to wait for the operator-started SITL's heartbeat (testing hook)")
    return parser


def build_attach_endpoint(args) -> ArduPilotVehicleEndpoint:
    """ATTACH mode only: executable_path/wsl_distro/working_directory are
    always left at their None defaults - this project's own code must
    never launch or kill a process in this phase."""
    vehicle_id = vehicle_id_from_namespace(args.namespace)
    return ArduPilotVehicleEndpoint(
        vehicle_id=vehicle_id, connection_string=args.connection,
        system_id=args.system_id, component_id=args.component_id,
    )


def _try_build_endpoint(args, report: dict):
    """Shared, safe endpoint/allowed-ports construction for every mode
    below - a bad --namespace/--connection/id must always produce a clean,
    structured failure in `report`, never an uncaught traceback."""
    try:
        vehicle_id = vehicle_id_from_namespace(args.namespace)
        endpoint = build_attach_endpoint(args)
        allowed_ports = frozenset({parse_connection_string(args.connection)[2]})
        return vehicle_id, endpoint, allowed_ports
    except (ValueError, ArduPilotTransportError) as e:
        report["remaining_failures"].append(f"configuration rejected: {e}")
        return None, None, None


def _base_report(mode: str, args) -> dict:
    return {
        "mode": mode,
        "namespace": args.namespace, "connection_string": args.connection,
        "system_id": args.system_id, "component_id": args.component_id,
        "attach": {"succeeded": False, "reason": None},
        "heartbeat": {"received": False},
        "shutdown": {"clean": False},
        "remaining_failures": [],
    }


# --------------------------------------------------------------------------
# Mode: dry_run
# --------------------------------------------------------------------------

def run_dry_run(args) -> dict:
    report = _base_report("dry_run", args)
    vehicle_id, endpoint, allowed_ports = _try_build_endpoint(args, report)
    if vehicle_id is None:
        report["ok"] = False
        return report

    dr = dry_run_validate({vehicle_id: endpoint}, allowed_ports=allowed_ports)
    report["ok"] = dr.ok
    report["vehicle_id"] = vehicle_id
    report["problems"] = list(dr.problems)
    report["pymavlink_available"] = PYMAVLINK_AVAILABLE
    report["pymavlink_version"] = PYMAVLINK_VERSION
    return report


# --------------------------------------------------------------------------
# Telemetry formatting shared by telemetry_visualization and planner_preview
# --------------------------------------------------------------------------

def _fmt_vec(v, prec: int = 2, unit: str = "") -> str:
    if v is None:
        return "n/a"
    return "(" + ", ".join(f"{c:.{prec}f}" for c in v) + ")" + unit


def _telemetry_summary(telem: SITLTelemetry) -> dict:
    return {
        "timestamp_s": telem.timestamp_s, "position_m": telem.position_m, "velocity_mps": telem.velocity_mps,
        "attitude_rad": telem.attitude_rad, "angular_velocity_radps": telem.angular_velocity_radps,
        "battery_fraction": telem.battery_fraction, "armed": telem.armed,
        "flight_mode": telem.flight_mode.value if telem.flight_mode is not None else None,
        "failsafe": telem.failsafe, "heartbeat_ok": telem.heartbeat_ok,
        "estimator_valid": telem.estimator_valid, "last_command_ack_status": telem.last_command_ack_status,
    }


def status_line(elapsed_s: float, vehicle_id: str, namespace: str, sysid: int, compid: int,
                 telem: SITLTelemetry) -> str:
    """The compact, continuously-printed status line - covers every field
    Phase 9 requires (item 4): elapsed time, vehicle/namespace/ids,
    lat/lon (honestly reported as unavailable - see module docstring and
    docs/PHASE9_SINGLE_VEHICLE_VISUAL.md), ENU position/altitude,
    velocity, roll/pitch/yaw, battery, armed state, flight mode,
    heartbeat, estimator state, and last command acknowledgement. Never
    prints a "flight successful"-style claim - this vehicle is expected
    to remain disarmed and stationary throughout Phase 9."""
    pos = _fmt_vec(telem.position_m)
    alt = f"{telem.position_m[2]:.2f}m" if telem.position_m is not None else "n/a"
    vel = _fmt_vec(telem.velocity_mps, unit="m/s")
    if telem.attitude_rad is not None:
        rpy = _fmt_vec(tuple(math.degrees(a) for a in telem.attitude_rad), prec=1, unit="deg")
    else:
        rpy = "n/a"
    batt = f"{telem.battery_fraction * 100:.0f}%" if telem.battery_fraction is not None else "n/a"
    armed = "YES" if telem.armed else "NO"
    mode = telem.flight_mode.value if telem.flight_mode is not None else "UNKNOWN"
    hb = "OK" if telem.heartbeat_ok else "LOST"
    ekf = "OK" if telem.estimator_valid else "INVALID"
    fs = "ACTIVE" if telem.failsafe else "clear"
    ack = telem.last_command_ack_status or "n/a"
    return (
        f"[t={elapsed_s:7.1f}s] {vehicle_id} ({namespace}) sysid={sysid}/{compid} | "
        f"lat/lon=n/a (not requested by this transport - see docs) | "
        f"ENU pos={pos} alt={alt} | vel={vel} | rpy={rpy} | batt={batt} | "
        f"armed={armed} | mode={mode} | hb={hb} | ekf={ekf} | failsafe={fs} | last_ack={ack}"
    )


# --------------------------------------------------------------------------
# Mode: telemetry_visualization
# --------------------------------------------------------------------------

def run_telemetry_visualization(args) -> dict:
    report = _base_report("telemetry_visualization", args)
    vehicle_id, endpoint, allowed_ports = _try_build_endpoint(args, report)
    if vehicle_id is None:
        return report
    report["vehicle_id"] = vehicle_id
    report["duration_s"] = args.duration
    report["telemetry_samples"] = 0
    report["last_telemetry"] = None

    transport = ArduPilotSITLTransport({vehicle_id: endpoint}, allowed_ports=allowed_ports, startup_timeout_s=args.startup_timeout)
    try:
        transport.start()
    except ArduPilotTransportError as e:
        report["attach"]["reason"] = str(e)
        report["remaining_failures"].append(f"attach failed: {e}")
        return report
    report["attach"]["succeeded"] = True
    channel = transport._channel(vehicle_id)
    report["heartbeat"]["received"] = channel.heartbeat_ok

    print(_SAFETY_BANNER)
    print(f"Attached to real ArduPilot SITL at {args.connection} "
          f"(system_id={args.system_id}, component_id={args.component_id}, namespace={args.namespace}).")
    print(f"Streaming telemetry_visualization for {args.duration:.0f}s. Press Ctrl+C to stop early.\n")

    deadline = time.monotonic() + args.duration
    t0 = time.monotonic()
    try:
        while time.monotonic() < deadline:
            telem = transport.receive_telemetry(vehicle_id)
            report["telemetry_samples"] += 1
            report["last_telemetry"] = _telemetry_summary(telem)
            print(status_line(time.monotonic() - t0, vehicle_id, args.namespace, args.system_id,
                               args.component_id, telem))
            time.sleep(0.5)   # paced, bounded - never a tight loop
    except KeyboardInterrupt:
        print("\nStopped by operator (Ctrl+C).")
    finally:
        stop_result = transport.stop()
        report["shutdown"]["clean"] = stop_result.success
    return report


# --------------------------------------------------------------------------
# Mode: hold_command_test
# --------------------------------------------------------------------------

def run_hold_command_test(args) -> dict:
    report = _base_report("hold_command_test", args)
    vehicle_id, endpoint, allowed_ports = _try_build_endpoint(args, report)
    if vehicle_id is None:
        return report
    report["vehicle_id"] = vehicle_id
    report["hold_command"] = {"sent": False, "accepted": None, "reason": None}
    report["rejected_navigation_command_test"] = {"attempted": False}

    transport = ArduPilotSITLTransport({vehicle_id: endpoint}, allowed_ports=allowed_ports, startup_timeout_s=args.startup_timeout)
    try:
        transport.start()
    except ArduPilotTransportError as e:
        report["attach"]["reason"] = str(e)
        report["remaining_failures"].append(f"attach failed: {e}")
        return report
    report["attach"]["succeeded"] = True
    channel = transport._channel(vehicle_id)
    report["heartbeat"]["received"] = channel.heartbeat_ok

    print(_SAFETY_BANNER)
    print(f"hold_command_test: attached to {args.connection} - sending one safe HOLD command.")

    # Bounded, paced telemetry warm-up (never a tight loop) - see
    # docs/PHASE9_SINGLE_VEHICLE_VISUAL.md and Phase 8's own identical
    # rationale for why real ArduPilot's first telemetry burst is not
    # instantaneous.
    warm_deadline = time.monotonic() + 5.0
    telem = None
    while time.monotonic() < warm_deadline:
        telem = transport.receive_telemetry(vehicle_id)
        if channel.last_state_update_sim_s is not None:
            break
        time.sleep(0.2)

    adapter = build_ardupilot_sitl_adapter(vehicle_id, transport)
    adapter.connect()
    now_s = telem.timestamp_s if telem is not None else 0.0

    hold_cmd = Command(vehicle_id=vehicle_id, command_type=CommandType.HOLD, frame=Frame.LOCAL_ENU,
                        desired_position_m=None, desired_velocity_mps=None, yaw_rad=None, yaw_rate_radps=None,
                        timestamp_s=now_s, expiration_time_s=now_s + 5.0, source="phase9_hold_test", confidence=0.9)
    result = adapter.send_command(AdapterCommand(command=hold_cmd, sequence=0))
    report["hold_command"] = {"sent": True, "accepted": result.accepted, "reason": result.reason}
    if not result.accepted:
        report["remaining_failures"].append(f"HOLD command not accepted: {result.reason}")

    # Optional rejected-navigation-command test: this project's own
    # transport synthesizes a local "accepted" ack for setpoint sends
    # (there is no real per-setpoint MAVLink ack - see
    # ardupilot_transport.py's module docstring), so the honest signal
    # that ArduPilot itself did not actually execute a navigation move is
    # the vehicle's OWN observed state, never our transport's send-side
    # ack: armed must stay False and position must not materially change.
    # This is recorded as the correct, safe result - never forced to
    # "succeed" by arming.
    armed_before = telem.armed if telem is not None else None
    position_before = telem.position_m if telem is not None else None
    nav_cmd = Command(vehicle_id=vehicle_id, command_type=CommandType.VELOCITY_SETPOINT, frame=Frame.LOCAL_ENU,
                       desired_position_m=None, desired_velocity_mps=(0.3, 0.0, 0.0), yaw_rad=None,
                       yaw_rate_radps=None, timestamp_s=now_s + 0.1, expiration_time_s=now_s + 5.1,
                       source="phase9_hold_test", confidence=0.9)
    nav_result = adapter.send_command(AdapterCommand(command=nav_cmd, sequence=1))
    time.sleep(1.0)   # bounded pause to observe any real effect
    telem_after = transport.receive_telemetry(vehicle_id)
    still_disarmed = not telem_after.armed
    report["rejected_navigation_command_test"] = {
        "attempted": True,
        "transport_ack_accepted": nav_result.accepted,
        "armed_before": armed_before, "armed_after": telem_after.armed,
        "position_before": position_before, "position_after": telem_after.position_m,
        "interpretation": (
            "correct safe result: the setpoint was sent but the vehicle remained disarmed and did not "
            "fly - ArduPilot never executes a navigation setpoint while disarmed"
            if still_disarmed else
            "UNEXPECTED: vehicle reports armed=True - this must never happen in Phase 9"
        ),
    }
    if not still_disarmed:
        report["remaining_failures"].append("vehicle reported armed=True during Phase 9 - safety violation")

    stop_result = transport.stop()
    report["shutdown"]["clean"] = stop_result.success
    return report


# --------------------------------------------------------------------------
# Mode: planner_preview
# --------------------------------------------------------------------------

def _vehicle_state_from_telemetry(vehicle_id: str, telem: SITLTelemetry, now_s: float) -> VehicleState:
    """Real ArduPilot LOCAL_POSITION_NED/ATTITUDE telemetry (already
    converted to this transport's operating_frame - Phase 8, unmodified)
    carries no acceleration field, so acceleration_mps2 is reported as
    zero (a real, documented simplification, not a silent guess at a
    nonzero value) - battery_fraction/attitude/velocity fall back to a
    safe default only until the corresponding MAVLink message has ever
    arrived once."""
    return VehicleState(
        vehicle_id=vehicle_id, sim_time_s=now_s, frame=Frame.LOCAL_ENU,
        position_m=telem.position_m if telem.position_m is not None else (0.0, 0.0, 0.0),
        velocity_mps=telem.velocity_mps if telem.velocity_mps is not None else (0.0, 0.0, 0.0),
        acceleration_mps2=(0.0, 0.0, 0.0),
        attitude_rad=telem.attitude_rad if telem.attitude_rad is not None else (0.0, 0.0, 0.0),
        angular_velocity_radps=telem.angular_velocity_radps if telem.angular_velocity_radps is not None else (0.0, 0.0, 0.0),
        battery_fraction=telem.battery_fraction if telem.battery_fraction is not None else 1.0,
        health_state=HealthState.OK,
        estimator_valid=telem.estimator_valid,
        last_valid_command_time_s=None,
    )


def run_planner_preview(args) -> dict:
    """Runs ONE minimal, deterministic single-drone demo planner (a fixed
    small forward-velocity candidate - there is no meaningful flocking/
    search behavior to demonstrate with a single, disarmed, stationary
    vehicle, so this is deliberately not a re-run of the full swarm
    behavior stack) through the real SafetySupervisor on every tick -
    never sends a raw candidate straight to the adapter/transport (see
    tests/test_phase9_single_vehicle_visual_architecture.py)."""
    report = _base_report("planner_preview", args)
    vehicle_id, endpoint, allowed_ports = _try_build_endpoint(args, report)
    if vehicle_id is None:
        return report
    report["vehicle_id"] = vehicle_id
    report["ticks"] = []
    report["armed_at_any_point"] = False

    transport = ArduPilotSITLTransport({vehicle_id: endpoint}, allowed_ports=allowed_ports, startup_timeout_s=args.startup_timeout)
    try:
        transport.start()
    except ArduPilotTransportError as e:
        report["attach"]["reason"] = str(e)
        report["remaining_failures"].append(f"attach failed: {e}")
        return report
    report["attach"]["succeeded"] = True
    channel = transport._channel(vehicle_id)
    report["heartbeat"]["received"] = channel.heartbeat_ok

    print(_SAFETY_BANNER)
    print(f"planner_preview: attached to {args.connection} - running one drone's demo planner + SafetySupervisor.")

    adapter = build_ardupilot_sitl_adapter(vehicle_id, transport)
    adapter.connect()

    warm_deadline = time.monotonic() + 10.0
    telem = None
    while time.monotonic() < warm_deadline:
        telem = transport.receive_telemetry(vehicle_id)
        if telem.position_m is not None:
            break
        time.sleep(0.2)
    if telem is None or telem.position_m is None:
        report["remaining_failures"].append(
            "no telemetry available within 10s - refusing to run planner_preview without real position data"
        )
        stop_result = transport.stop()
        report["shutdown"]["clean"] = stop_result.success
        return report

    supervisor = SafetySupervisor()
    geofence = GeofenceSpec(frame=Frame.LOCAL_ENU, center_m=(0.0, 0.0), half_extents_m=(50.0, 50.0),
                             floor_alt_m=-10.0, ceiling_alt_m=50.0)
    n_ticks = max(1, int(args.duration))   # ~1 tick/second, bounded by --duration
    sequence = 0
    for k in range(n_ticks):
        telem = transport.receive_telemetry(vehicle_id)
        now_s = telem.timestamp_s
        own_state = _vehicle_state_from_telemetry(vehicle_id, telem, now_s)
        candidate = CandidateCommand(
            vehicle_id=vehicle_id, desired_velocity_mps=(0.2, 0.0, 0.0), frame=Frame.LOCAL_ENU,
            timestamp_s=now_s, expiration_time_s=now_s + 2.0, source="Phase9SingleVehicleDemoPlanner",
        )
        sensor_observation = SensorObservation(
            vehicle_id=vehicle_id, sensor_timestamp_s=now_s, sensor_latency_s=0.0, fov_deg=360.0,
            range_returns_m=(), occluded=(), dropout=False, pose_uncertainty_m=0.0, detections=(),
        )
        mission_context = MissionContext(geofence=geofence, operator_abort=False, contact_detected=False,
                                          safe_point_m=None, own_agent_isolated=False)
        # SafetySupervisor is the final command authority - evaluate()
        # ALWAYS runs before anything can reach the adapter/transport, and
        # only decision.filtered_command (never the raw `candidate` above)
        # is ever sent below.
        decision = supervisor.evaluate(candidate, own_state, sensor_observation, (), mission_context, now_s)

        tick_record = {
            "tick": k, "sim_time_s": now_s,
            "candidate_command": {
                "desired_velocity_mps": candidate.desired_velocity_mps, "frame": candidate.frame.value,
            },
            "safety_decision": {
                "accepted": decision.accepted, "reason": decision.reason,
                "emergency_state": decision.emergency_state.value,
                "active_constraints": list(decision.active_constraints),
            },
            "adapter_command": None, "transport_result": None,
        }
        if decision.accepted:
            adapter_command = AdapterCommand(command=decision.filtered_command, sequence=sequence)
            sequence += 1
            result = adapter.send_command(adapter_command)
            tick_record["adapter_command"] = {
                "command_type": decision.filtered_command.command_type.value,
                "desired_velocity_mps": decision.filtered_command.desired_velocity_mps,
            }
            tick_record["transport_result"] = {"accepted": result.accepted, "reason": result.reason}
        report["ticks"].append(tick_record)
        report["armed_at_any_point"] = report["armed_at_any_point"] or telem.armed
        print(status_line(float(k), vehicle_id, args.namespace, args.system_id, args.component_id, telem))
        time.sleep(1.0)   # paced, bounded - never a tight loop

    if report["armed_at_any_point"]:
        report["remaining_failures"].append("vehicle reported armed=True at some point - safety violation")

    stop_result = transport.stop()
    report["shutdown"]["clean"] = stop_result.success
    report["safety_event_log"] = [e.to_dict() for e in supervisor.event_log]
    return report


# --------------------------------------------------------------------------

def main(argv=None) -> None:
    args = build_parser().parse_args(argv)
    os.makedirs(OUT_DIR, exist_ok=True)

    if args.hold_test and args.planner_preview:
        print("Error: --hold-test and --planner-preview are mutually exclusive - pick exactly one mode per run.")
        sys.exit(2)
    if (args.hold_test or args.planner_preview) and not args.attach:
        print("Error: --hold-test/--planner-preview require --attach.")
        sys.exit(2)

    if args.dry_run or not args.attach:
        result = run_dry_run(args)
        out_name = "phase9_dry_run_result.json"
    elif args.hold_test:
        result = run_hold_command_test(args)
        out_name = "phase9_hold_command_test_result.json"
    elif args.planner_preview:
        result = run_planner_preview(args)
        out_name = "phase9_planner_preview_result.json"
    else:
        result = run_telemetry_visualization(args)
        out_name = "phase9_telemetry_visualization_result.json"

    print(json.dumps(result, indent=2, default=str))
    with open(os.path.join(OUT_DIR, out_name), "w") as f:
        json.dump(result, f, indent=2, default=str)


if __name__ == "__main__":
    main()
