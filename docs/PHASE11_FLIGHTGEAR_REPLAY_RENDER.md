# Phase 11: FlightGear REPLAY Render of Phase 10's Actual Recorded Flight

The live Phase 11A bridge (`scripts/run_phase11_flightgear_viewer.py
--flightgear-view`) correctly renders the current, disarmed, stationary
vehicle - it has never rendered Phase 10's actual `1.720716118812561` m
flight, and that script's own `--replay` mode intentionally never opens a
socket (see `docs/PHASE11_LIVE_VALIDATION_PROCEDURE.md`'s "How to
distinguish live telemetry from replay"). This is a **new, separate**
script that closes that gap: it drives a real FlightGear window to show
Phase 10's actual recorded flight, including its measured peak altitude,
without touching ArduPilot SITL, MAVLink, or the existing bridge/flight-
test code at all.

## Source of truth

`results/phase10_sitl_flight_test/phase10_sitl_flight_test_result.json` is
the real file Phase 10's actual completed flight test wrote - but
`results/` is entirely gitignored, so that file does not survive a fresh
clone. A byte-identical, tracked copy is committed at
**`docs/phase10_recorded_flight_result.json`** and is this script's
default `--source-result`. Its `takeoff.max_altitude_observed_m` is
`1.720716118812561` - the real measured peak, read from the file at
runtime, never hardcoded in the script.

## Never a fabricated trajectory

The source JSON contains a handful of discrete, real, timestamped
`events` - not a continuous trace (its `"hold"` field records only
`{"duration_s": 60.0, "sample_count": 59}`, a count, not 59 samples). The
replay renderer (`scripts/run_phase11_flightgear_replay_render.py`,
`_build_replay_segments`) turns these into a **step function that holds
the last real known value between events** - never an interpolated or
invented curve - and labels every segment with exactly where its number
came from:

| `altitude_source` | Meaning |
|---|---|
| `measured_takeoff_confirmed_altitude_m` | The one real recorded non-zero sample (`events[].altitude_m` at `"takeoff_confirmed"`) - the actual `1.720716118812561` m peak. |
| `held_last_measured_value_no_hold_samples_recorded` | The hold phase has no per-sample data, so the last measured value is held rather than inventing new numbers. |
| `inferred_boundary_ground_before_liftoff` / `inferred_boundary_ground_after_disarm_confirmed` | Conservative 0 m (home) boundary values, used only where the source JSON's own events (`arm_attempt`/`armed_confirmed` before liftoff, `disarm_confirmed: true` after landing) make ground level the only honest assumption - never a measurement. |

Segment **timing** is also real: each segment plays for the real elapsed
`monotonic_s` delta between that event and the next (optionally
compressed by `--speedup`), not an arbitrary duration.

## What's reused, unchanged

- `swarm_sim/visualization/telemetry_mapping.py` - the exact same FGNetFDM
  v24 packing (`build_fg_net_fdm_fields`/`pack_fg_net_fdm`) the live
  bridge uses. Not reimplemented.
- `swarm_sim/visualization/flightgear_bridge.py` - the same loopback-only,
  send-only UDP `FlightGearBridge`/`parse_flightgear_endpoint`.
- `scripts/run_phase11_flightgear_viewer.py`'s `MIN_UPDATE_RATE_HZ`/
  `MAX_UPDATE_RATE_HZ` bounds (imported, not duplicated).

**Not modified**: `scripts/run_phase11_flightgear_viewer.py`,
`scripts/run_phase10_sitl_flight_test.py`,
`swarm_sim/visualization/flightgear_bridge.py`,
`swarm_sim/visualization/telemetry_mapping.py`. See
`tests/test_phase11_flightgear_replay_render_architecture.py::test_does_not_modify_the_live_bridge_or_phase10_flight_test_files`.

## No MAVLink capability at all

`scripts/run_phase11_flightgear_replay_render.py` never imports
`pymavlink`/`mavutil`, never imports `ArduPilotSITLTransport`, and never
opens a TCP socket - the only network I/O it performs is UDP `sendto()` to
the given loopback `--flightgear-endpoint`. It cannot arm, take off, land,
change mode, or write a parameter, because it has no MAVLink capability
whatsoever - proven structurally in
`tests/test_phase11_flightgear_replay_render_architecture.py`, and
behaviorally in
`tests/test_phase11_flightgear_replay_render.py::test_replay_never_requires_a_mavlink_port_to_be_listening`
(the test passes with nothing listening on the MAVLink port at all).

## REPLAY label

Every frame's CSV row and the JSON report set `view_label: "REPLAY"` -
never `"LIVE"` - so a render from this script can never be mistaken for
the Phase 11A live bridge's output.

## Usage

```bash
# Validate only - never opens a socket:
python3 scripts/run_phase11_flightgear_replay_render.py --dry-run

# Actually render it (FlightGear must already be listening, e.g. via
# fg_quad_view.sh on 5503 - see docs/PHASE11_LIVE_VALIDATION_PROCEDURE.md):
python3 scripts/run_phase11_flightgear_replay_render.py \
    --flightgear-endpoint 127.0.0.1:5503 --update-rate-hz 10 --speedup 1.0
```

`--speedup 1.0` (default) replays at the actual recorded pace (~74 s
total, including the real 60 s hold). Output: JSON report at
`results/phase11_flightgear/phase11_replay_render_result.json` and a
labeled CSV at `results/phase11_flightgear/replay_telemetry.csv`.

## Verification performed this session

- `--dry-run` against the real tracked source: `recorded_peak_altitude_m:
  1.720716118812561`, 8 segments, all `ok: true`.
- Full send against a real local UDP socket (standing in for FlightGear,
  same pattern as Phase 11A's own tests): every received datagram exactly
  408 bytes, `version == 24`; unpacked `altitude` field reaches
  `584.0 + 1.720716118812561 = 585.720716118812561` m - the recorded peak
  is present in the actual wire bytes, not just claimed in the JSON
  report.
- Full test suite and compile checks - see the accompanying report
  message for counts.

**Not performed this session**: pointing this script at a real running
FlightGear window and visually confirming the climb is rendered - that
requires an operator to watch the window, the same limitation documented
throughout Phase 11 (Claude cannot see a GUI).
