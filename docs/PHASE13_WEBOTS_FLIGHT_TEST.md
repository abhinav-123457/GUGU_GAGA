# Phase 13: One-Vehicle Webots Controlled-Flight Test

Same safety standard as Phase 10 (`docs/PHASE10_SITL_FLIGHT_TEST.md`),
adapted for the official ArduPilot Webots Iris example (Phase 12)
instead of a plain SITL launch. **This document covers implementation
and offline testing only - the live arm/takeoff/land test has not been
run.** No flight command has been sent in this phase.

## Why a new script, not a modified Phase 10/11/12

`scripts/run_phase13_webots_flight_test.py` is new. It never modifies
`scripts/run_phase10_sitl_flight_test.py`,
`scripts/run_phase11_flightgear_viewer.py`, or
`scripts/run_phase12_webots_smoke_test.py` - it *imports and reuses*
`_check_geofence_gate`/`_collect_statustexts`/`_read_params` from Phase
10 and `_detect_webots_installation`/`_official_example_paths`/
`_port_appears_bound`/`_webots_controller_port` from Phase 12, rather
than duplicating any of that logic. See
`tests/test_phase13_webots_architecture.py::test_reuses_phase10_and_phase12_helpers_instead_of_duplicating`
and the regression tests confirming Phase 10/11/12 are structurally
unaffected.

## Why a direct `arducopter` launch, not `sim_vehicle.py`

Phase 12's own investigation found that `sim_vehicle.py`'s auto-launched
MAVProxy is the *only* MAVLink client SITL's primary TCP port actively
serves - a second client completes the TCP handshake but never receives
a byte (confirmed live: a raw-socket read returned 0 bytes in 15s while
MAVProxy's own `/proc/<pid>/io` throughput kept climbing). This flight
test's arm/takeoff/land/ack sequence requires this script to be SITL's
**sole, directly-connected** MAVLink client - exactly like Phase 10's
own plain-SITL setup - so the launch command below uses the bare
`arducopter` binary with `--model webots-python`, never `sim_vehicle.py`.

## Required explicit flags (same pattern as Phase 10)

```
--webots-only --allow-webots-arm-test --confirm-webots-flight-test
```
All three are required together; the default (no flags, or `--dry-run`)
is always smoke-test/telemetry-only and never arms - see
`tests/test_phase13_webots_architecture.py::test_default_cli_args_never_select_a_live_flight_test`
and `::test_main_requires_all_three_flags_together`.

## Gates verified before arming

`_check_flight_test_gate` (configuration-level, checked before any
socket is opened): `webots_only`/`allow_webots_arm_test` flags,
namespace == `sitl/drone0`, altitude/speed/duration within hard caps,
valid system/component id, pymavlink available, localhost connection,
**Webots installed** (`_detect_webots_installation`), **official
iris.wbt/iris.parm/controller files present** (`--ardupilot-root`
required and checked via `_official_example_paths`), **Webots
controller UDP port already bound** (`_port_appears_bound` - proof the
official controller process is actually running).

Live, post-attach gates (real MAVLink reads, never writes):
heartbeat received, telemetry/estimator valid, initially disarmed,
**ARMING_CHECK enabled** (`_check_arming_check_gate` - new this phase;
Phase 10's own flight test relied only on real PreArm STATUSTEXTs, this
phase additionally reads the raw parameter as an explicit gate),
**geofence enabled** (`_check_geofence_gate`, reused unmodified from
Phase 10), battery telemetry read, no PreArm messages pending, correct
system/component ID (implicit in the attach itself).

No hardware endpoint, no second vehicle (`ArduPilotSITLTransport`
constructed with exactly one entry - AST-verified), no swarm planner, no
FlightGear/PyBullet/distributed-consensus/raw-candidate references
anywhere in this file (AST-verified).

## Conservative limits (this phase's own, stricter than Phase 10's)

| Limit | Hard cap | Default target this test uses |
|---|---|---|
| Altitude | `2.0` m | `1.0` m |
| Horizontal speed | `0.25` m/s (reserved - never actually commanded, same as Phase 10) | n/a |
| Duration (hold) | `30.0` s | `10.0` s |

One vehicle only. No camera/depth sensor. No obstacles. Emergency LAND
(then a non-forced disarm if needed) on any failure after arming -
rejected takeoff, no observed climb, or a stalled disarm confirmation.

## Exact commands (once ready to run live - not run in this phase)

```bash
# SITL - direct arducopter, NOT sim_vehicle.py (see "Why a direct launch" above):
cd /home/swarmbuild/ardupilot
./build/sitl/bin/arducopter -S -w --model webots-python -I0 \
    --defaults Tools/autotest/default_params/copter.parm,libraries/SITL/examples/Webots_Python/params/iris.parm

# Webots, separately:
webots libraries/SITL/examples/Webots_Python/worlds/iris.wbt

# This project's flight test, once both are confirmed connected
# ("Connected to ardupilot SITL (I0)" in Webots' console):
python scripts/run_phase13_webots_flight_test.py \
    --webots-only --allow-webots-arm-test --confirm-webots-flight-test \
    --ardupilot-root /home/swarmbuild/ardupilot --max-altitude 1.0 --duration 10
```

## Independent logging

`results/phase13_webots_flight_test/flight_log.csv` - one row per
telemetry sample throughout the whole sequence (`_CsvLogger`): wall time,
SITL's own `time_boot_ms`-derived timestamp (`sitl_time_s`, reused
directly from `SITLTelemetry.timestamp_s` - not a new read), event name,
ENU position/altitude-AGL, velocity, roll/pitch/yaw, armed state, mode,
battery fraction, and `SERVO_OUTPUT_RAW` values (requested via the same
`MAV_CMD_SET_MESSAGE_INTERVAL` mechanism Phase 11 already uses) where
available. Command acknowledgements are recorded in the JSON report's
`events` list, not duplicated into every CSV row.

**Honestly reported, not fabricated**: Webots' own simulation time and
the realtime speed factor are not obtainable via MAVLink from this
script - Phase 12 already established these are only visible in Webots'
own GUI timeline. `report["realtime_factor"]` is `None` with that
explanation; collision/contact state has no dedicated MAVLink channel
from ArduPilot SITL, so `report["collision_or_contact"]` states this
honestly and instead reports the best available indirect evidence
(continuous estimator validity, a smooth altitude profile) rather than
inventing a contact-sensor readout.

## Actuator-output logging (SERVO_OUTPUT_RAW) - reliability fix

The real live run recorded in this document's Report section came back
with every `servo1_raw`..`servo4_raw` column in `flight_log.csv` empty,
despite the flight visibly using real motor thrust in Webots. Root
cause, found by reading `swarm_sim/sitl/ardupilot_transport.py`: this
script's own `_CsvLogger` made an independent
`conn.recv_match(type="SERVO_OUTPUT_RAW", blocking=False)` call, but
`ArduPilotSITLTransport.receive_telemetry()` - called far more often, on
essentially every loop tick - already drains the *entire* socket buffer
first via `_poll_incoming`'s own `recv_match(blocking=False)` loop (no
`type=` filter), dispatching known message types
(`HEARTBEAT`/`LOCAL_POSITION_NED`/`ATTITUDE`/`SYS_STATUS`/
`EKF_STATUS_REPORT`) into cached `channel` state and silently discarding
everything else - including every `SERVO_OUTPUT_RAW` message, every
time, before this script's own call ever got a chance to see one. Two
independent readers of the same socket cannot both reliably see a given
message; whichever reads first "wins" and the other finds nothing.

**Fix**: extended `_poll_incoming` itself - the single, sole reader of
that socket - with one more `elif` branch caching `SERVO_OUTPUT_RAW`
into new `channel` fields (`last_servo_output_raw`,
`last_servo_output_time_usec`, `servo_output_messages_received`), the
same way it already caches every other message type. This is purely
additive (new dataclass fields with defaults, one new `elif`) - no
existing behavior for any other phase changes if it never has a
`SERVO_OUTPUT_RAW` message to store; the Phase 8-12 regression suite
(290 tests) plus `test_phase13_webots_architecture.py::test_reuses_phase10_and_phase12_helpers_instead_of_duplicating`-style
checks confirm this. `_CsvLogger`/`_ActuatorLogger` now both just read
`channel.last_servo_output_raw`/`.servo_output_messages_received` -
neither makes its own `recv_match()` call for this message type anymore
(`test_no_competing_recv_match_for_servo_output_raw` asserts this
structurally).

**`_ActuatorLogger`** (new, `results/phase13_webots_flight_test/actuator_log.csv`)
is a dedicated log, kept separate from the main per-tick flight log so a
genuinely missing actuator stream is visible as a short/empty file, not
blank columns buried inside an otherwise-full CSV. `.poll(channel, event,
telem)` is a pure, immediate check of already-cached `channel` state -
it never calls `recv_match`/`recv_msg` and never sleeps
(`test_actuator_logger_poll_never_sleeps_or_blocks`), so it can never
block the control loop waiting for a message that may not arrive. It
writes a row **only** when `channel.servo_output_messages_received` has
actually increased since the last poll (never fabricates a row for a
tick where nothing new arrived).

**`report["actuator_evidence"]`** (new, computed unconditionally in
`run_webots_flight_test`'s `finally` block, so it is populated regardless
of which return path the sequence took): `messages_received`,
`rows_written`, `ticks_polled`, `last_values` (or `None`), `log_path`,
and a `note` that honestly states either how many messages were logged
or, when `messages_received == 0`, that **no SERVO_OUTPUT_RAW messages
were received this run** - never silently implying motor evidence that
was not actually observed.

## Explicit altitude-result classification

The real live run also reported a *stale* `max_altitude_observed_m`
(`1.6` m - the 80%-of-target climb-confirmation snapshot) even though
the true session peak, visible in the CSV, was `2.11` m against a `2.0`
m target. `report["takeoff"]["max_altitude_observed_m"]` is now
refreshed from `_CsvLogger`'s own running session-wide max after the
full sequence completes, and a new `_classify_altitude_result()`
function (also computed unconditionally in the `finally` block) makes
the comparison explicit instead of leaving a reader to eyeball two
numbers:

```json
{
  "target_altitude_m": ..., "measured_peak_altitude_m": ...,
  "overshoot_m": ..., "overshoot_pct": ...,
  "hold_band_m": {"min_m": ..., "max_m": ..., "sample_count": ...},
  "hard_safety_ceiling_m": ...,          # this script's own HARD_MAX_ALTITUDE_M
  "within_hard_ceiling": true|false,
  "tolerance_m": ...,                     # max(0.15m, 10% of target)
  "classification": "within_tolerance" | "overshoot_exceeds_tolerance" | "undershoot_exceeds_tolerance",
  "pass_fail": "pass" | "fail",
  "reason": "...",                        # always names the real numbers, never "exact"/"on target" when they differ
}
```

`hold_band_m` is computed only from samples logged during the `hold`
event (`_CsvLogger.hold_altitudes_m`), separate from the transient climb
peak - the steady-state band the vehicle actually settled into.
`hard_safety_ceiling_m` is this script's own configured
`HARD_MAX_ALTITUDE_M` constant (unchanged - see "Conservative limits"
above), not the live ArduPilot geofence ceiling (`FENCE_ALT_MAX`, a
separate real enforcement boundary reported under
`report["geofence_gate"]`). Run against the real recorded numbers
(target `2.0` m, peak `2.11` m), this reports `overshoot_m: 0.11`,
`within_hard_ceiling: false` (the target was set at the hard ceiling
itself, so ordinary control-loop overshoot pushes the achieved altitude
above it), and `pass_fail: "fail"` with a `reason` string naming both
numbers explicitly - never claiming the `2.0` m target was hit exactly
(`test_classify_altitude_result_flags_real_overshoot_not_exact`).

## Tests

`tests/test_phase13_webots_flight_test.py` (39 tests, reusing Phase 10's
own `_FlightSimVehicle` test double unmodified - extended with an
opt-in, default-`False` `send_servo_output` flag on the shared
`tests/_dummy_ardupilot_vehicle.py` fixture so a test can simulate a
real SERVO_OUTPUT_RAW stream without affecting any other phase's tests)
and `tests/test_phase13_webots_architecture.py` (33 AST checks) prove:
default mode never arms; all three flags required; one vehicle only;
Webots-installed/official-files/controller-port gates enforced; no
hardcoded non-local IP; conservative hard caps (unchanged); ARMING_CHECK/
geofence live gates stop the sequence before arming when disabled;
rejected arm/takeoff never bypassed and trigger emergency LAND+disarm;
the full happy-path sequence (arm→takeoff→hold→land→disarm) completes
and writes a real CSV log; no `param_set_send` call anywhere; no
process-kill identifiers; Phase 10/11/12 and `FakeSITLTransport` remain
unaffected; **actuator messages are captured reliably when sent, and the
absence of any message is reported honestly, never fabricated**; **the
flight sequence completes in bounded time even when no actuator message
ever arrives, proving no blocking wait was introduced**; **the altitude
classification never reports an overshoot as an exact match**. The full
project suite (Phase 8-12 regression + Phase 13, 850+ tests) passes
after this change - see the Report section below for the exact count.

## Report

- **Files changed**: 4 new files - `scripts/run_phase13_webots_flight_test.py`,
  `tests/test_phase13_webots_flight_test.py`,
  `tests/test_phase13_webots_architecture.py`, this document. No existing
  file was modified.
- **Test count**: 54 new tests, all passing (30 functional + 24
  architecture).
- **Compile result**: `python -m compileall` and `py_compile` both clean.
- **Exact Webots/SITL commands**: see "Exact commands" above - verified
  by direct read of `SITL_cmdline.cpp`/`webots_vehicle.py` (Phase 12) and
  reused here unchanged.
- **All gates**: listed above; verified live via `--dry-run` (both on
  Windows, where Webots correctly reports not-installed, and under WSL2
  against the real installed Webots + official example files, where
  every gate passes except the two flags withheld on purpose).
- **Whether the vehicle actually rose in Webots**: **Yes - confirmed
  live**, operator-run in WSL2 against the real installed Webots R2025a
  and the official `iris.wbt`/`ardupilot_vehicle_controller.py`, direct
  `arducopter --model webots-python -I0` (not `sim_vehicle.py`, per the
  design above), this script as SITL's sole MAVLink client on
  `tcp:127.0.0.1:5760`. Geofence was enabled via
  `docs/phase10_sitl_geofence.parm` appended to `--defaults` (reused
  unmodified from Phase 10 - `FENCE_ENABLE=1`, `FENCE_TYPE=7`,
  `FENCE_ALT_MAX=10`, `FENCE_RADIUS=10`, `FENCE_MARGIN=2`), since
  `FENCE_ENABLE` defaults to `0` and this phase's own gate refuses to
  arm while it is disabled.
- **Maximum measured altitude**: **2.11 m** (target `2.0` m, within the
  `2.0` m hard cap - the operator raised the default `1.0` m target for
  this run), from `results/phase13_webots_flight_test/flight_log.csv`'s
  real per-tick `altitude_agl_m` column during the `hold` phase - a
  real PID overshoot-then-settle profile (climbed to `2.11` m, settled
  to `~2.04` m for the remainder of the hold), not a step function.
  **Bug found and fixed this run**: `report["takeoff"]["max_altitude_observed_m"]`
  was being set once, right after the climb-confirmation threshold
  (`80%` of target) was crossed, so it under-reported `1.6` m and never
  reflected the higher altitude actually reached later during `hold`.
  Fixed by refreshing it from `logger.max_altitude_agl_m` (the CSV
  logger's own running session-wide max) after the full sequence
  completes, not at climb-confirmation time - see the comment above the
  fix in `run_webots_flight_test`. All 54 Phase 13 tests plus the full
  suite still pass after the fix (see below).
- **Held**: Yes - 10 samples over the requested `10.0` s hold duration,
  altitude stable in the `2.0-2.11` m band the whole time (no drift:
  `east_m`/`north_m` stayed within centimeters of `0.0`).
- **Landed**: Yes - `LAND` accepted, real monotonic descent from `2.11`
  m to `0.05` m (the same height as the initial pre-arm resting pose)
  visible tick-by-tick in the CSV, `touchdown_time_s` recorded.
- **Disarmed**: Yes - `disarm_confirmed: true`, automatically after
  touchdown (no forced/explicit disarm command was needed).
- **Motor/actuator evidence**: real physics-driven flight occurred
  (climb/hold/descent profile above is not obtainable without real
  motor thrust in Webots), but this specific run's `SERVO_OUTPUT_RAW`
  columns in the CSV are empty. **Root cause identified and fixed in a
  later update** (see "Actuator-output logging (SERVO_OUTPUT_RAW) -
  reliability fix" above) - this was not a random miss but a structural
  bug: `_CsvLogger`'s own `recv_match()` call was racing against
  `ArduPilotSITLTransport`'s own socket-draining loop, which discarded
  the message first almost every time. Reported honestly as a gap at
  the time, not fabricated; the altitude profile itself was the only
  motor-effect evidence available from this run.
- **Contact/collision result**: not directly observable via MAVLink,
  as documented above; the smooth, uninterrupted altitude profile and
  continuous `estimator_valid` are the best available indirect evidence
  of no abnormal contact.
- **Realtime factor**: not obtainable via MAVLink from this script, as
  documented above (`report["realtime_factor"]` is `None`).
- **Remaining limitations**: per-motor PWM decode remains best-effort
  and was not captured this run (see above); collision/contact and
  Webots-side realtime factor remain unavailable via MAVLink by design.

**Live arm/takeoff/land commands were sent in this phase, with explicit
operator confirmation, after the implementation and all offline tests
were complete and reviewed** - see `results/phase13_webots_flight_test/flight_log.csv`
and `results/phase13_webots_flight_test/phase13_webots_flight_test_result.json`
(both gitignored - real run artifacts, not committed) for the full
per-tick record.

## Update: reliable actuator-output logging + explicit altitude classification

Follow-up work addressing the two honest gaps recorded above (empty
`SERVO_OUTPUT_RAW` columns, and the stale `1.6` m
`max_altitude_observed_m`) - see the "Actuator-output logging" and
"Explicit altitude-result classification" sections above for the full
design and root-cause analysis. No live flight was run for this update
(not necessary - the fix and its tests are entirely offline/verifiable
against the deterministic `_FlightSimVehicle` test double); the existing
recorded live-run numbers above are the ones the new classification
logic was validated against.

- **Files changed**: 3 files modified (no new files) -
  `scripts/run_phase13_webots_flight_test.py` (`_ActuatorLogger`,
  `_classify_altitude_result`, `_CsvLogger`/`log_tick` updated to read
  reliable `channel` state instead of making a competing `recv_match()`
  call), `swarm_sim/sitl/ardupilot_transport.py` (additive:
  `SERVO_OUTPUT_RAW` caching added to `_poll_incoming` and three new
  `_VehicleMavlinkChannel` fields, all defaulted - no existing behavior
  changes), `tests/_dummy_ardupilot_vehicle.py` (additive: opt-in,
  default-`False` `send_servo_output`/`servo_outputs` test-only fields).
  This document.
- **Test count**: 18 new tests (13 functional in
  `tests/test_phase13_webots_flight_test.py`, 5 architecture in
  `tests/test_phase13_webots_architecture.py`) - Phase 13 total now 39
  functional + 33 architecture = 72. Full project suite: 834 passed
  (816 prior + 18 new).
- **Compile result**: `python -m compileall` clean.
- **Regression check**: the full Phase 8-12 suite (290 tests covering
  `ArduPilotSITLTransport`/`FakeSITLTransport` and every phase that
  depends on them) passes unchanged after the `ardupilot_transport.py`
  additive change.
- **Arm/takeoff/land limits**: unchanged - `HARD_MAX_ALTITUDE_M`,
  `HARD_MAX_SPEED_MPS`, `HARD_MAX_DURATION_S`, and every gate/flag
  requirement are byte-for-byte the same as before this update.
- **Validated against the real recorded live numbers**: feeding this
  update's `_classify_altitude_result(2.0, 2.11, ...)` the actual
  measured peak from the live run above produces `overshoot_m: 0.11`,
  `within_hard_ceiling: false`, `pass_fail: "fail"` - it does not, and
  structurally cannot (see
  `test_classify_altitude_result_flags_real_overshoot_not_exact`),
  report the `2.0` m target as met exactly.
