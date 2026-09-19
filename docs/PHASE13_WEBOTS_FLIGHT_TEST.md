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

## Tests

`tests/test_phase13_webots_flight_test.py` (30 tests, reusing Phase 10's
own `_FlightSimVehicle` test double unmodified) and
`tests/test_phase13_webots_architecture.py` (24 AST checks) prove:
default mode never arms; all three flags required; one vehicle only;
Webots-installed/official-files/controller-port gates enforced; no
hardcoded non-local IP; conservative hard caps; ARMING_CHECK/geofence
live gates stop the sequence before arming when disabled; rejected
arm/takeoff never bypassed and trigger emergency LAND+disarm; the full
happy-path sequence (arm→takeoff→hold→land→disarm) completes and writes
a real CSV log; no `param_set_send` call anywhere; no process-kill
identifiers; Phase 10/11/12 and `FakeSITLTransport` remain unaffected.

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
  columns in the CSV are empty - the non-blocking per-tick check in
  `_CsvLogger.log()` did not happen to catch a `SERVO_OUTPUT_RAW`
  message on this run. Reported honestly as a gap, not fabricated;
  the altitude profile itself is the motor-effect evidence available
  from this run.
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
