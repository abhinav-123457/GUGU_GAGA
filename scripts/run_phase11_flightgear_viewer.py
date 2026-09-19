"""Phase 11A: optional FlightGear visual-rendering integration - see
docs/PHASE11_FLIGHTGEAR_VIEWER.md.

STRICTLY SIMULATOR-ONLY, ATTACH mode only (same invariant as Phase 9/10 -
this script never spawns or kills a process, ArduPilot or FlightGear).
Never connects to anything but 127.0.0.1/localhost, for BOTH the ArduPilot
MAVLink endpoint and the FlightGear UDP endpoint. ArduPilot SITL remains
the only flight-physics and command authority; FlightGear is used only to
RENDER already-computed telemetry - it never provides physics and this
script never sends it anything that could be mistaken for one (no arm,
takeoff, land, mode, or setpoint command is ever constructed here).

**Investigation summary** (full citations in
docs/PHASE11_FLIGHTGEAR_VIEWER.md): contrary to the concern that ArduPilot's
FlightGear documentation is Plane/HIL-focused, ArduPilot's SITL *does* have
a native, vehicle-agnostic, read-only FlightGear *rendering* output built
directly into `SITL_State.cpp` (`--enable-fgview --fg <address>`, UDP port
5503, protocol version 24 aka "native FDM" / FGNetFDM), and even ships a
dedicated `Tools/autotest/fg_quad_view.sh` + `Tools/autotest/aircraft/
arducopter/` model specifically for Copter. That native path is entirely
internal to the `arducopter` C++ process talking directly to FlightGear -
it never goes through MAVLink, so there is nothing for this project's own
`ArduPilotSITLTransport`-based bridge to "reuse" from it. **This script is
therefore an independent, MAVLink-sourced telemetry-to-FlightGear renderer
prototype that happens to speak the same wire protocol** (native FDM /
FGNetFDM, version 24, UDP) as that native path - chosen specifically
because it is a real, source-verified, already-interoperating protocol
(see swarm_sim/visualization/telemetry_mapping.py's own citations), not
because this project invented or guessed it. It is NOT the same pipe as
ArduPilot's own internal one, and does not claim to be.

Four modes, never merged (see each function's own report shape):

    --dry-run (default - never opens any socket, MAVLink or UDP)
        Validates configuration shape only.

    --diagnose
        Read-only, no connection of any kind (not even MAVLink) - reports
        the documented protocol facts (version, struct size, the
        recommended FlightGear startup command) and validates argument
        shapes only.

    --attach (telemetry_only - delegates to Phase 9's own
        run_telemetry_visualization, completely unmodified)

    --flightgear-view
        Attaches to ArduPilot SITL (ATTACH only), reads telemetry via
        `ArduPilotSITLTransport.receive_telemetry()` plus a read-only
        `GLOBAL_POSITION_INT` listen (see below), converts it to an
        FGNetFDM frame, and sends it to the configured, loopback-only
        `--flightgear-endpoint` at a bounded rate. Never sends anything
        else to either endpoint.

    --replay <telemetry.csv>
        Offline only - never opens a MAVLink or UDP socket. Re-runs
        previously-saved rows (produced by --flightgear-view/--attach)
        back through the same conversion pipeline to produce a summary
        report, for demonstration/regression purposes without needing
        ArduPilot or FlightGear running at all.

**Why GLOBAL_POSITION_INT, and why this script reads telemetry itself
rather than calling `ArduPilotSITLTransport.receive_telemetry()`**:
FGNetFDM's position fields are absolute geodetic latitude/longitude/
altitude, not a local NED/ENU offset - a real protocol requirement, not
this project's choice. `ArduPilotSITLTransport` does not request or parse
`GLOBAL_POSITION_INT` (it only requests LOCAL_POSITION_NED/ATTITUDE/
SYS_STATUS/EKF_STATUS_REPORT - see ardupilot_transport.py), so this script
separately requests it with one `MAV_CMD_SET_MESSAGE_INTERVAL` command
(the same read-only stream-rate mechanism the transport itself already
uses for its own messages - the ONLY command this script ever sends).
However, a single MAVLink connection is a single-consumer byte stream:
`receive_telemetry()`'s own `_poll_incoming()` does a non-blocking drain of
*every* pending message and silently discards any type it does not
recognize (confirmed by reproducing it live - see
docs/PHASE11_FLIGHTGEAR_VIEWER.md), so calling it and then separately
`recv_match(type="GLOBAL_POSITION_INT", ...)` on the same connection race
each other: whichever runs first consumes and discards the messages the
other one needed. Per this phase's own item 2 ("read telemetry using the
existing ArduPilotSITLTransport **or a strictly read-only telemetry
client**"), this script uses the transport for attach/heartbeat/identity/
namespace verification only (Phase 8's own proven, unmodified ATTACH
logic), then reads the *same already-open connection* itself for the
actual streaming loop - one unified, non-blocking, no-type-filter drain
per tick, dispatching HEARTBEAT/LOCAL_POSITION_NED/ATTITUDE/
GLOBAL_POSITION_INT. The NED->ENU/yaw conversion math is not
reimplemented - `_read_telemetry_tick` below calls the exact same
`swarm_sim.autopilot.frames.convert_vector_frame`/`_yaw_ned_to_enu`
functions `ardupilot_transport.py` itself calls internally, so this is a
different *read loop*, not a different *conversion*.
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

from swarm_sim.sitl.ardupilot_transport import (  # noqa: E402
    ArduPilotSITLTransport, ArduPilotTransportError, PYMAVLINK_AVAILABLE, PYMAVLINK_VERSION,
    dry_run_validate, parse_connection_string,
)
from swarm_sim.autopilot.frames import _yaw_ned_to_enu, convert_vector_frame  # noqa: E402
from swarm_sim.contracts import Frame  # noqa: E402
from swarm_sim.visualization import telemetry_mapping as tmap  # noqa: E402
from swarm_sim.visualization.flightgear_bridge import (  # noqa: E402
    FlightGearBridge, FlightGearBridgeError, parse_flightgear_endpoint,
)

try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None

OUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "results",
                        "phase11_flightgear")
CSV_PATH = os.path.join(OUT_DIR, "telemetry.csv")

MIN_UPDATE_RATE_HZ = 1.0
MAX_UPDATE_RATE_HZ = 30.0
DEFAULT_UPDATE_RATE_HZ = 10.0

_CSV_FIELDS = (
    "timestamp_s", "system_id", "component_id", "namespace",
    "east_m", "north_m", "up_m", "altitude_m", "agl_m", "lat_deg", "lon_deg",
    "ve_mps", "vn_mps", "vu_mps", "roll_rad", "pitch_rad", "yaw_enu_rad",
    "connection_status", "fg_send_result", "dropped_invalid_reason",
)

_SAFETY_BANNER = (
    "Phase 11A: optional FlightGear visual-rendering integration.\n"
    "  - Simulator-only. ArduPilot SITL remains the only flight-physics and command authority.\n"
    "  - FlightGear (if used) is a read-only renderer - this script never sends it, or ArduPilot,\n"
    "    an arm/takeoff/land/mode/setpoint command.\n"
    "  - Both the ArduPilot connection and the FlightGear endpoint must be loopback (127.0.0.1).\n"
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Phase 11A: optional FlightGear visual-rendering integration "
                    "(simulator-only, ATTACH mode only, read-only renderer).",
    )
    parser.add_argument("--connection", default="tcp:127.0.0.1:5760")
    parser.add_argument("--system-id", type=int, default=1)
    parser.add_argument("--component-id", type=int, default=1)
    parser.add_argument("--namespace", default="sitl/drone0")
    parser.add_argument("--startup-timeout", type=float, default=20.0)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--update-rate-hz", type=float, default=DEFAULT_UPDATE_RATE_HZ,
                         help=f"bounded [{MIN_UPDATE_RATE_HZ}, {MAX_UPDATE_RATE_HZ}] Hz")
    parser.add_argument("--dry-run", action="store_true", help="validate configuration only - never opens a socket")
    parser.add_argument("--diagnose", action="store_true",
                         help="report documented protocol facts - never opens any connection")
    parser.add_argument("--attach", action="store_true",
                         help="telemetry_only mode - delegates to Phase 9's own telemetry_visualization, unchanged")
    parser.add_argument("--flightgear-view", action="store_true",
                         help="attach to SITL and send read-only renderer telemetry to FlightGear")
    parser.add_argument("--flightgear-endpoint", default=None,
                         help="loopback host:port FlightGear is listening on, e.g. 127.0.0.1:5503 "
                              "(required for --flightgear-view; must be explicit, never guessed)")
    parser.add_argument("--replay", default=None, metavar="TELEMETRY_CSV",
                         help="offline mode - replay a previously-saved telemetry CSV; "
                              "never opens a MAVLink or UDP socket")
    return parser


# --------------------------------------------------------------------------
# Mode: dry_run
# --------------------------------------------------------------------------

def run_dry_run(args) -> dict:
    report = {
        "mode": "dry_run", "namespace": args.namespace, "connection_string": args.connection,
        "system_id": args.system_id, "component_id": args.component_id,
        "ok": False, "remaining_failures": [],
    }

    if not (MIN_UPDATE_RATE_HZ <= args.update_rate_hz <= MAX_UPDATE_RATE_HZ):
        report["remaining_failures"].append(
            f"--update-rate-hz {args.update_rate_hz} outside bounded range "
            f"[{MIN_UPDATE_RATE_HZ}, {MAX_UPDATE_RATE_HZ}]"
        )

    vehicle_id, endpoint, allowed_ports = phase9._try_build_endpoint(args, report)
    endpoint_ok = vehicle_id is not None
    if endpoint_ok:
        dr = dry_run_validate({vehicle_id: endpoint}, allowed_ports=allowed_ports)
        report["endpoint_dry_run"] = {"ok": dr.ok, "problems": list(dr.problems)}
        endpoint_ok = dr.ok

    report["flightgear_endpoint"] = None
    if args.flightgear_endpoint is not None:
        try:
            fg_endpoint = parse_flightgear_endpoint(args.flightgear_endpoint)
            report["flightgear_endpoint"] = {"host": fg_endpoint.host, "port": fg_endpoint.port, "ok": True}
        except FlightGearBridgeError as e:
            report["flightgear_endpoint"] = {"ok": False, "reason": str(e)}
            report["remaining_failures"].append(f"flightgear endpoint invalid: {e}")

    report["ok"] = endpoint_ok and not report["remaining_failures"]
    report["pymavlink_available"] = PYMAVLINK_AVAILABLE
    report["pymavlink_version"] = PYMAVLINK_VERSION
    return report


# --------------------------------------------------------------------------
# Mode: diagnose - no connection of any kind, MAVLink or UDP
# --------------------------------------------------------------------------

def run_diagnose(args) -> dict:
    report = {
        "mode": "diagnose",
        "protocol": {
            "name": "FlightGear native FDM (FGNetFDM)",
            "version": tmap.FG_NET_FDM_VERSION,
            "struct_size_bytes": tmap.FG_NET_FDM_STRUCT_SIZE,
            "transport": "UDP",
            "direction": "this project -> FlightGear only (send-only; nothing is ever read back)",
            "default_port": 5503,
            "source": (
                "ArduPilot libraries/AP_HAL_SITL/SITL_State.cpp _output_to_flightgear() and "
                "libraries/SITL/SIM_JSBSim.h class FGNetFDM, commit 92b0cd788ec29406f26c6f9c31d5ceedbd1cc538 "
                "(Copter-4.6.3) - see docs/PHASE11_FLIGHTGEAR_VIEWER.md"
            ),
            "native_ardupilot_flag": "--enable-fgview --fg <address> (passed directly to arducopter, "
                                       "or via sim_vehicle.py -A \"--fg <address>\")",
            "native_ardupilot_script": "Tools/autotest/fg_quad_view.sh (Copter-specific, ships an "
                                         "'arducopter' aircraft model under Tools/autotest/aircraft/arducopter/)",
            "is_native_ardupilot_integration": (
                "The --enable-fgview flag above IS a native ArduPilot-to-FlightGear integration, verified in "
                "source for Copter. This SCRIPT, however, is a SEPARATE, independent, MAVLink-sourced "
                "telemetry-renderer prototype that speaks the same wire protocol - it does not tap into that "
                "native path, which never touches MAVLink at all."
            ),
        },
        "arguments": {"remaining_failures": []},
    }

    if not PYMAVLINK_AVAILABLE:
        report["arguments"]["remaining_failures"].append("pymavlink not available")
    try:
        parse_connection_string(args.connection)
    except ArduPilotTransportError as e:
        report["arguments"]["remaining_failures"].append(f"--connection invalid: {e}")
    if not (MIN_UPDATE_RATE_HZ <= args.update_rate_hz <= MAX_UPDATE_RATE_HZ):
        report["arguments"]["remaining_failures"].append(
            f"--update-rate-hz {args.update_rate_hz} outside bounded range "
            f"[{MIN_UPDATE_RATE_HZ}, {MAX_UPDATE_RATE_HZ}]"
        )
    if args.flightgear_endpoint is not None:
        try:
            fg_endpoint = parse_flightgear_endpoint(args.flightgear_endpoint)
            report["arguments"]["flightgear_endpoint"] = {"host": fg_endpoint.host, "port": fg_endpoint.port}
        except FlightGearBridgeError as e:
            report["arguments"]["remaining_failures"].append(f"--flightgear-endpoint invalid: {e}")

    report["arguments"]["ok"] = not report["arguments"]["remaining_failures"]
    return report


# --------------------------------------------------------------------------
# Mode: flightgear_view
# --------------------------------------------------------------------------

def _request_global_position_stream(conn, sysid: int, compid: int, rate_hz: float) -> None:
    """The ONLY command this script ever sends - a read-only stream-rate
    request, the same mechanism ardupilot_transport.py already uses for
    its own messages. Never arm/takeoff/land/mode/setpoint."""
    interval_us = 1_000_000.0 / rate_hz
    conn.mav.command_long_send(
        sysid, compid, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, interval_us, 0, 0, 0, 0, 0,
    )


class _ReadOnlyTelemetryState:
    """This script's own minimal read-only telemetry cache - see module
    docstring's "why this script reads telemetry itself" section. Deliberately
    a thin dispatch loop, not a reimplementation of any conversion math:
    NED->ENU and yaw both call the exact same `swarm_sim.autopilot.frames`
    functions `ardupilot_transport.py` itself uses."""

    def __init__(self):
        self.heartbeat_ok = False
        self.position_enu = None
        self.velocity_enu = None
        self.roll_rad = None
        self.pitch_rad = None
        self.yaw_enu_rad = None
        self.angular_velocity_radps = None
        self.global_position = None   # {"lat_deg", "lon_deg", "altitude_m", "agl_m"}


def _poll_telemetry_tick(conn, state: "_ReadOnlyTelemetryState") -> None:
    """One non-blocking, no-type-filter drain of every pending message -
    the only way to safely read GLOBAL_POSITION_INT and
    LOCAL_POSITION_NED/ATTITUDE/HEARTBEAT off the same connection (see
    module docstring). Read-only: never calls anything that sends a
    message."""
    while True:
        msg = conn.recv_match(blocking=False)
        if msg is None:
            break
        msg_type = msg.get_type()
        if msg_type == "HEARTBEAT":
            state.heartbeat_ok = True
        elif msg_type == "LOCAL_POSITION_NED":
            ned_position = (float(msg.x), float(msg.y), float(msg.z))
            ned_velocity = (float(msg.vx), float(msg.vy), float(msg.vz))
            state.position_enu = convert_vector_frame(ned_position, Frame.LOCAL_NED, Frame.LOCAL_ENU)
            state.velocity_enu = convert_vector_frame(ned_velocity, Frame.LOCAL_NED, Frame.LOCAL_ENU)
        elif msg_type == "ATTITUDE":
            state.roll_rad = float(msg.roll)
            state.pitch_rad = float(msg.pitch)
            state.yaw_enu_rad = _yaw_ned_to_enu(float(msg.yaw))
            state.angular_velocity_radps = (float(msg.rollspeed), float(msg.pitchspeed), float(msg.yawspeed))
        elif msg_type == "GLOBAL_POSITION_INT":
            state.global_position = {
                "lat_deg": msg.lat / 1.0e7, "lon_deg": msg.lon / 1.0e7,
                "altitude_m": msg.alt / 1000.0, "agl_m": msg.relative_alt / 1000.0,
            }


def run_flightgear_view(args) -> dict:
    report = {
        "mode": "flightgear_view", "namespace": args.namespace, "connection_string": args.connection,
        "system_id": args.system_id, "component_id": args.component_id,
        "loopback_verified": False, "attach": {"succeeded": False, "reason": None},
        "heartbeat": {"received": False}, "identity_verified": False,
        "flightgear_endpoint": None, "csv_path": CSV_PATH,
        "sample_count": 0, "dropped_count": 0, "fg_sent_count": 0, "fg_error_count": 0,
        "shutdown": {"clean": False}, "remaining_failures": [],
    }

    if not (MIN_UPDATE_RATE_HZ <= args.update_rate_hz <= MAX_UPDATE_RATE_HZ):
        report["remaining_failures"].append(
            f"--update-rate-hz {args.update_rate_hz} outside bounded range "
            f"[{MIN_UPDATE_RATE_HZ}, {MAX_UPDATE_RATE_HZ}]"
        )
        return report

    if args.flightgear_endpoint is None:
        report["remaining_failures"].append("--flightgear-endpoint is required for --flightgear-view")
        return report
    try:
        fg_endpoint = parse_flightgear_endpoint(args.flightgear_endpoint)
    except FlightGearBridgeError as e:
        report["remaining_failures"].append(f"--flightgear-endpoint invalid: {e}")
        return report
    report["flightgear_endpoint"] = {"host": fg_endpoint.host, "port": fg_endpoint.port}

    vehicle_id, endpoint, allowed_ports = phase9._try_build_endpoint(args, report)
    if vehicle_id is None:
        return report
    report["loopback_verified"] = True  # both endpoints loopback-validated by this point

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

    fg_bridge = FlightGearBridge(fg_endpoint)
    fg_bridge.open()
    os.makedirs(OUT_DIR, exist_ok=True)
    state = _ReadOnlyTelemetryState()
    state.heartbeat_ok = channel.heartbeat_ok

    try:
        _request_global_position_stream(conn, args.system_id, args.component_id, args.update_rate_hz)

        with open(CSV_PATH, "w", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=_CSV_FIELDS)
            writer.writeheader()

            period_s = 1.0 / args.update_rate_hz
            deadline = time.monotonic() + args.duration
            try:
                while time.monotonic() < deadline:
                    tick_start = time.monotonic()
                    _poll_telemetry_tick(conn, state)   # the ONLY read of this connection each tick - see module docstring

                    row = {
                        "timestamp_s": tick_start, "system_id": args.system_id,
                        "component_id": args.component_id, "namespace": args.namespace,
                        "connection_status": "ok" if state.heartbeat_ok else "lost",
                        "fg_send_result": "", "dropped_invalid_reason": "",
                    }
                    if state.position_enu is not None:
                        row["east_m"], row["north_m"], row["up_m"] = state.position_enu
                    if state.velocity_enu is not None:
                        row["ve_mps"], row["vn_mps"], row["vu_mps"] = state.velocity_enu
                    if state.yaw_enu_rad is not None:
                        row["roll_rad"], row["pitch_rad"], row["yaw_enu_rad"] = (
                            state.roll_rad, state.pitch_rad, state.yaw_enu_rad
                        )

                    can_send = (
                        state.position_enu is not None and state.velocity_enu is not None
                        and state.yaw_enu_rad is not None and state.global_position is not None
                    )
                    if not can_send:
                        reason = "no_telemetry_yet" if state.position_enu is None else "no_global_position_yet"
                        row["dropped_invalid_reason"] = reason
                        report["dropped_count"] += 1
                    else:
                        gp = state.global_position
                        row["lat_deg"] = gp["lat_deg"]
                        row["lon_deg"] = gp["lon_deg"]
                        row["altitude_m"] = gp["altitude_m"]
                        row["agl_m"] = gp["agl_m"]
                        angvel = state.angular_velocity_radps or (0.0, 0.0, 0.0)
                        fields = tmap.build_fg_net_fdm_fields(
                            latitude_deg=gp["lat_deg"], longitude_deg=gp["lon_deg"],
                            altitude_m=gp["altitude_m"], agl_m=gp["agl_m"],
                            roll_rad=state.roll_rad, pitch_rad=state.pitch_rad, yaw_enu_rad=state.yaw_enu_rad,
                            rollspeed_radps=angvel[0], pitchspeed_radps=angvel[1], yawspeed_radps=angvel[2],
                            velocity_enu_mps=state.velocity_enu,
                        )
                        sent = fg_bridge.send_frame(tmap.pack_fg_net_fdm(fields))
                        row["fg_send_result"] = "sent" if sent else "error"
                        report["sample_count"] += 1
                        if sent:
                            report["fg_sent_count"] += 1
                        else:
                            report["fg_error_count"] += 1

                    writer.writerow(row)
                    elapsed = time.monotonic() - tick_start
                    time.sleep(max(0.0, period_s - elapsed))
            except KeyboardInterrupt:
                report["remaining_failures"].append("stopped by operator (Ctrl+C)")
    finally:
        fg_bridge.close()
        stop_result = transport.stop()   # closes THIS script's own connection only - never terminates ArduPilot
        report["shutdown"]["clean"] = stop_result.success

    return report


# --------------------------------------------------------------------------
# Mode: replay - offline only, never opens a MAVLink or UDP socket
# --------------------------------------------------------------------------

def run_replay(args) -> dict:
    report = {
        "mode": "replay", "csv_path": args.replay,
        "row_count": 0, "sent_row_count": 0, "dropped_row_count": 0,
        "min_altitude_m": None, "max_altitude_m": None, "remaining_failures": [],
    }
    if not args.replay or not os.path.isfile(args.replay):
        report["remaining_failures"].append(f"--replay file not found: {args.replay!r}")
        return report

    with open(args.replay, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            report["row_count"] += 1
            if row.get("dropped_invalid_reason"):
                report["dropped_row_count"] += 1
                continue
            try:
                lat = float(row["lat_deg"]); lon = float(row["lon_deg"])
                alt = float(row["altitude_m"]); agl = float(row["agl_m"])
                roll = float(row["roll_rad"]); pitch = float(row["pitch_rad"]); yaw = float(row["yaw_enu_rad"])
                ve = float(row["ve_mps"]); vn = float(row["vn_mps"]); vu = float(row["vu_mps"])
            except (KeyError, ValueError):
                report["dropped_row_count"] += 1
                continue
            # Re-run the exact same offline conversion/packing pipeline used
            # live by run_flightgear_view - never opens a socket here.
            fields = tmap.build_fg_net_fdm_fields(
                latitude_deg=lat, longitude_deg=lon, altitude_m=alt, agl_m=agl,
                roll_rad=roll, pitch_rad=pitch, yaw_enu_rad=yaw,
                rollspeed_radps=0.0, pitchspeed_radps=0.0, yawspeed_radps=0.0,
                velocity_enu_mps=(ve, vn, vu),
            )
            tmap.pack_fg_net_fdm(fields)   # validates the row round-trips through packing without error
            report["sent_row_count"] += 1
            report["min_altitude_m"] = alt if report["min_altitude_m"] is None else min(report["min_altitude_m"], alt)
            report["max_altitude_m"] = alt if report["max_altitude_m"] is None else max(report["max_altitude_m"], alt)

    return report


# --------------------------------------------------------------------------

def main(argv=None) -> None:
    args = build_parser().parse_args(argv)
    os.makedirs(OUT_DIR, exist_ok=True)
    print(_SAFETY_BANNER)

    if args.dry_run:
        result, out_name = run_dry_run(args), "phase11_dry_run_result.json"
    elif args.diagnose:
        result, out_name = run_diagnose(args), "phase11_diagnose_result.json"
    elif args.replay:
        result, out_name = run_replay(args), "phase11_replay_result.json"
    elif args.flightgear_view:
        result, out_name = run_flightgear_view(args), "phase11_flightgear_view_result.json"
    elif args.attach:
        result, out_name = phase9.run_telemetry_visualization(args), "phase11_telemetry_only_result.json"
    else:
        result, out_name = run_dry_run(args), "phase11_dry_run_result.json"

    print(json.dumps(result, indent=2, default=str))
    with open(os.path.join(OUT_DIR, out_name), "w") as f:
        json.dump(result, f, indent=2, default=str)


if __name__ == "__main__":
    main()
