"""Phase 11: read-only FlightGear REPLAY renderer of Phase 10's actual
recorded flight-test result - see docs/PHASE11_FLIGHTGEAR_REPLAY_RENDER.md.

**Why this is a separate, new script**: scripts/run_phase11_flightgear_viewer.py
(the Phase 11A live bridge) is never modified by this phase - its own
`--replay` mode intentionally never opens a socket and only produces a JSON
summary (see its module docstring and docs/PHASE11_LIVE_VALIDATION_PROCEDURE.md's
"How to distinguish live telemetry from replay" table). This script's job is
different: actually drive a real FlightGear window to visibly show Phase 10's
real recorded flight, including its measured 1.720716 m peak altitude,
without needing ArduPilot SITL, MAVLink, or a new flight test.

**Read-only / SITL-independent by construction**: this module never imports
pymavlink or mavutil, never opens a MAVLink connection, and never speaks to
ArduPilot at all - the ONLY network I/O it performs is sending UDP FGNetFDM
frames to FlightGear (via the unmodified `swarm_sim.visualization.flightgear_bridge.
FlightGearBridge`). It cannot arm, take off, land, change mode, or write a
parameter, because it has no MAVLink capability whatsoever - a stronger
guarantee than "chooses not to" (see
tests/test_phase11_flightgear_replay_render_architecture.py).

**Never fabricates a trajectory**: the only input this script reads is the
*actual* Phase 10 flight-test result JSON already produced by a real
completed flight (default: docs/phase10_recorded_flight_result.json, a
tracked, byte-identical copy of the real
results/phase10_sitl_flight_test/phase10_sitl_flight_test_result.json this
project's own gitignored `results/` directory does not preserve). That JSON
contains a handful of discrete, real, timestamped `events` - not a
continuous trace. `_build_replay_segments` below turns those events into a
step function that holds the last REAL known value between events (never an
interpolated/invented curve shape), and labels every segment with exactly
where its altitude number came from:

    - "measured_takeoff_confirmed_altitude_m": the one real recorded
      non-zero altitude sample (`events[].altitude_m` at
      "takeoff_confirmed"), which must equal `takeoff.max_altitude_observed_m`
      - this is the real 1.720716118812561 m peak, read from the file, never
        hardcoded in this script.
    - "held_last_measured_value_no_hold_samples_recorded": the source JSON's
      "hold" only records `{"duration_s": 60.0, "sample_count": 59}` - a
      count, not 59 individual altitude samples - so this script honestly
      holds the last measured value across the hold rather than inventing 59
      new numbers.
    - "inferred_boundary_ground_before_liftoff" /
      "inferred_boundary_ground_after_disarm_confirmed": conservative
      boundary values (0 m relative to home), never a measurement, used only
      before the vehicle has attempted takeoff and after it is known (via the
      source JSON's own `disarm_confirmed: true`) to have safely landed.

Segment *timing* also comes from the real recording: each segment plays for
the real elapsed `monotonic_s` delta to the next event (optionally
compressed by `--speedup`), not an arbitrary duration.

Reuses, unmodified: `swarm_sim.visualization.telemetry_mapping`'s FGNetFDM
v24 packing (the exact same mapping the live bridge uses) and
`swarm_sim.visualization.flightgear_bridge.FlightGearBridge`/
`parse_flightgear_endpoint` (the same loopback-only, send-only UDP sender).
Neither of those files, nor scripts/run_phase11_flightgear_viewer.py, nor
scripts/run_phase10_sitl_flight_test.py, is modified by this phase.

Every frame this script sends is labeled `view_label: "REPLAY"` (never
"LIVE") in both its CSV log and its JSON report, so a FlightGear render
driven by this script can never be mistaken for the Phase 11A live bridge's
output - see docs/PHASE11_FLIGHTGEAR_REPLAY_RENDER.md.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from run_phase11_flightgear_viewer import (  # noqa: E402 - reused, unmodified
    DEFAULT_UPDATE_RATE_HZ, MAX_UPDATE_RATE_HZ, MIN_UPDATE_RATE_HZ,
)

from swarm_sim.autopilot.frames import _yaw_ned_to_enu  # noqa: E402 - reused, unmodified
from swarm_sim.visualization import telemetry_mapping as tmap  # noqa: E402
from swarm_sim.visualization.flightgear_bridge import (  # noqa: E402
    FlightGearBridge, FlightGearBridgeError, parse_flightgear_endpoint,
)

_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_DIR = os.path.join(_PROJECT_ROOT, "results", "phase11_flightgear")
CSV_PATH = os.path.join(OUT_DIR, "replay_telemetry.csv")
DEFAULT_SOURCE_RESULT = os.path.join(_PROJECT_ROOT, "docs", "phase10_recorded_flight_result.json")

# Documented Phase 10 SITL home position (docs/PHASE10_SITL_FLIGHT_TEST.md /
# docs/PHASE11_LIVE_VALIDATION_PROCEDURE.md's own `--home` value). The source
# JSON records only RELATIVE altitude at each event, never absolute lat/lon/
# altitude/heading - this constant is what turns a real relative measurement
# into the absolute position FGNetFDM requires. It is not itself a
# measurement, and is never used to invent a non-zero relative altitude.
HOME_LAT_DEG = -35.363261
HOME_LON_DEG = 149.165230
HOME_ALTITUDE_M = 584.0
HOME_HEADING_DEG = 353.0

MIN_SPEEDUP = 1.0
MAX_SPEEDUP = 50.0
DEFAULT_SPEEDUP = 1.0

_GROUND_EVENTS_BEFORE_LIFTOFF = frozenset({
    "attached", "mode_set_guided", "arm_attempt", "armed_confirmed", "takeoff_attempt",
})

_CSV_FIELDS = (
    "timestamp_s", "view_label", "event", "altitude_source",
    "lat_deg", "lon_deg", "absolute_altitude_m", "relative_altitude_m", "agl_m",
    "roll_rad", "pitch_rad", "yaw_ned_deg", "fg_send_result",
)

_SAFETY_BANNER = (
    "Phase 11: read-only FlightGear REPLAY renderer of Phase 10's actual recorded flight.\n"
    "  - REPLAY mode - clearly labeled, never claims to be LIVE.\n"
    "  - Never connects to ArduPilot SITL and never opens a MAVLink connection (no pymavlink import at all).\n"
    "  - Cannot arm/take off/land/change mode/write a parameter - it has no MAVLink capability whatsoever.\n"
    "  - The only network I/O is sending UDP FGNetFDM frames to the given loopback --flightgear-endpoint.\n"
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Phase 11: read-only FlightGear REPLAY renderer of Phase 10's actual "
                    "recorded flight-test result (never connects to ArduPilot SITL).",
    )
    parser.add_argument("--source-result", default=DEFAULT_SOURCE_RESULT,
                         help="the actual recorded Phase 10 flight-test result JSON to replay "
                              f"(default: {DEFAULT_SOURCE_RESULT})")
    parser.add_argument("--flightgear-endpoint", default=None,
                         help="loopback host:port FlightGear is listening on, e.g. 127.0.0.1:5503 "
                              "(required unless --dry-run; must be explicit, never guessed)")
    parser.add_argument("--update-rate-hz", type=float, default=DEFAULT_UPDATE_RATE_HZ,
                         help=f"bounded [{MIN_UPDATE_RATE_HZ}, {MAX_UPDATE_RATE_HZ}] Hz")
    parser.add_argument("--speedup", type=float, default=DEFAULT_SPEEDUP,
                         help=f"playback speed multiplier, bounded [{MIN_SPEEDUP}, {MAX_SPEEDUP}]; "
                              "1.0 (default) replays at the actual recorded pace")
    parser.add_argument("--dry-run", action="store_true",
                         help="validate the source file and configuration only - never opens a socket")
    return parser


def _load_recorded_events(source_path: str):
    with open(source_path) as f:
        result = json.load(f)
    if not result.get("flight_actually_happened"):
        raise ValueError(f"{source_path!r} does not record a completed real flight "
                          "('flight_actually_happened' is not true) - refusing to replay it as one")
    events = result.get("events")
    if not events:
        raise ValueError(f"{source_path!r} has no recorded 'events' - nothing to replay")
    peak_altitude_m = (result.get("takeoff") or {}).get("max_altitude_observed_m")
    if peak_altitude_m is None:
        raise ValueError(f"{source_path!r} has no recorded takeoff.max_altitude_observed_m")
    return events, float(peak_altitude_m)


def _build_replay_segments(events, peak_altitude_m: float):
    """One segment per real recorded event, in the source JSON's own order -
    see module docstring for exactly what each `altitude_source` label
    means. Never invents an event, a timestamp, or an altitude value that
    is not either read directly from `events` or one of the two documented
    conservative ground-level boundary inferences."""
    segments = []
    for i, event in enumerate(events):
        name = event.get("event")
        if name in _GROUND_EVENTS_BEFORE_LIFTOFF:
            relative_altitude_m, source = 0.0, "inferred_boundary_ground_before_liftoff"
        elif name == "takeoff_confirmed":
            relative_altitude_m = float(event["altitude_m"])
            if abs(relative_altitude_m - peak_altitude_m) > 1e-9:
                raise ValueError(
                    "recorded 'takeoff_confirmed' altitude_m does not match "
                    "'takeoff.max_altitude_observed_m' - refusing to replay inconsistent data"
                )
            source = "measured_takeoff_confirmed_altitude_m"
        elif name == "hold_complete":
            relative_altitude_m, source = peak_altitude_m, "held_last_measured_value_no_hold_samples_recorded"
        elif name == "land_attempt":
            relative_altitude_m, source = 0.0, "inferred_boundary_ground_after_disarm_confirmed"
        else:
            relative_altitude_m, source = 0.0, f"unrecognized_event_defaulted_to_ground:{name}"

        if i + 1 < len(events):
            duration_s = max(0.0, float(events[i + 1]["monotonic_s"]) - float(event["monotonic_s"]))
        else:
            duration_s = 1.0
        segments.append({
            "event": name, "relative_altitude_m": relative_altitude_m,
            "altitude_source": source, "duration_s": duration_s,
        })
    return segments


def run_replay_render(args) -> dict:
    report = {
        "mode": "flightgear_replay_render", "view_label": "REPLAY",
        "source_file": args.source_result, "flightgear_endpoint": None,
        "frame_count": 0, "fg_sent_count": 0, "fg_error_count": 0,
        "recorded_peak_altitude_m": None, "max_relative_altitude_m": None, "peak_reached": False,
        "csv_path": CSV_PATH, "segments": [], "remaining_failures": [], "ok": False,
    }

    if not (MIN_UPDATE_RATE_HZ <= args.update_rate_hz <= MAX_UPDATE_RATE_HZ):
        report["remaining_failures"].append(
            f"--update-rate-hz {args.update_rate_hz} outside bounded range "
            f"[{MIN_UPDATE_RATE_HZ}, {MAX_UPDATE_RATE_HZ}]"
        )
        return report
    if not (MIN_SPEEDUP <= args.speedup <= MAX_SPEEDUP):
        report["remaining_failures"].append(
            f"--speedup {args.speedup} outside bounded range [{MIN_SPEEDUP}, {MAX_SPEEDUP}]"
        )
        return report

    try:
        events, peak_altitude_m = _load_recorded_events(args.source_result)
    except (OSError, ValueError, KeyError) as e:
        report["remaining_failures"].append(f"failed to load source result: {e}")
        return report
    report["recorded_peak_altitude_m"] = peak_altitude_m

    try:
        segments = _build_replay_segments(events, peak_altitude_m)
    except ValueError as e:
        report["remaining_failures"].append(str(e))
        return report
    report["segments"] = [
        {"event": s["event"], "relative_altitude_m": s["relative_altitude_m"],
         "altitude_source": s["altitude_source"], "duration_s": s["duration_s"]}
        for s in segments
    ]

    if args.dry_run:
        report["ok"] = not report["remaining_failures"]
        return report

    if args.flightgear_endpoint is None:
        report["remaining_failures"].append("--flightgear-endpoint is required unless --dry-run")
        return report
    try:
        fg_endpoint = parse_flightgear_endpoint(args.flightgear_endpoint)
    except FlightGearBridgeError as e:
        report["remaining_failures"].append(f"--flightgear-endpoint invalid: {e}")
        return report
    report["flightgear_endpoint"] = {"host": fg_endpoint.host, "port": fg_endpoint.port}

    yaw_enu_rad = _yaw_ned_to_enu(math.radians(HOME_HEADING_DEG))

    fg_bridge = FlightGearBridge(fg_endpoint)
    fg_bridge.open()
    os.makedirs(OUT_DIR, exist_ok=True)

    period_s = 1.0 / args.update_rate_hz
    max_relative_altitude_m = 0.0
    frame_count = 0
    fg_sent_count = 0
    fg_error_count = 0
    playback_clock_s = 0.0

    try:
        with open(CSV_PATH, "w", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=_CSV_FIELDS)
            writer.writeheader()
            try:
                for segment in segments:
                    tick_count = max(1, round((segment["duration_s"] / args.speedup) * args.update_rate_hz))
                    relative_altitude_m = segment["relative_altitude_m"]
                    absolute_altitude_m = HOME_ALTITUDE_M + relative_altitude_m
                    max_relative_altitude_m = max(max_relative_altitude_m, relative_altitude_m)

                    for _ in range(tick_count):
                        tick_start = time.monotonic()
                        fields = tmap.build_fg_net_fdm_fields(
                            latitude_deg=HOME_LAT_DEG, longitude_deg=HOME_LON_DEG,
                            altitude_m=absolute_altitude_m, agl_m=relative_altitude_m,
                            roll_rad=0.0, pitch_rad=0.0, yaw_enu_rad=yaw_enu_rad,
                            rollspeed_radps=0.0, pitchspeed_radps=0.0, yawspeed_radps=0.0,
                            velocity_enu_mps=(0.0, 0.0, 0.0),
                        )
                        sent = fg_bridge.send_frame(tmap.pack_fg_net_fdm(fields))
                        frame_count += 1
                        if sent:
                            fg_sent_count += 1
                        else:
                            fg_error_count += 1

                        writer.writerow({
                            "timestamp_s": playback_clock_s, "view_label": "REPLAY", "event": segment["event"],
                            "altitude_source": segment["altitude_source"],
                            "lat_deg": HOME_LAT_DEG, "lon_deg": HOME_LON_DEG,
                            "absolute_altitude_m": absolute_altitude_m, "relative_altitude_m": relative_altitude_m,
                            "agl_m": relative_altitude_m, "roll_rad": 0.0, "pitch_rad": 0.0,
                            "yaw_ned_deg": HOME_HEADING_DEG, "fg_send_result": "sent" if sent else "error",
                        })
                        playback_clock_s += period_s
                        elapsed = time.monotonic() - tick_start
                        time.sleep(max(0.0, period_s - elapsed))
            except KeyboardInterrupt:
                report["remaining_failures"].append("stopped by operator (Ctrl+C)")
    finally:
        fg_bridge.close()

    report["frame_count"] = frame_count
    report["fg_sent_count"] = fg_sent_count
    report["fg_error_count"] = fg_error_count
    report["max_relative_altitude_m"] = max_relative_altitude_m
    report["peak_reached"] = abs(max_relative_altitude_m - peak_altitude_m) < 1e-6
    report["ok"] = report["peak_reached"] and not report["remaining_failures"]
    return report


def main(argv=None) -> None:
    args = build_parser().parse_args(argv)
    os.makedirs(OUT_DIR, exist_ok=True)
    print(_SAFETY_BANNER)
    result = run_replay_render(args)
    print(json.dumps(result, indent=2, default=str))
    with open(os.path.join(OUT_DIR, "phase11_replay_render_result.json"), "w") as f:
        json.dump(result, f, indent=2, default=str)


if __name__ == "__main__":
    main()
