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
- **Whether the vehicle actually rose in Webots**: **not yet attempted -
  no live arm/takeoff/land test has been run this phase**, per the
  instruction to complete and review implementation/offline tests first.
- **Maximum measured altitude / held / landed / disarmed / motor
  evidence / contact result / realtime factor**: all **not yet
  measured** - live run pending a separate, explicit go-ahead.
- **Remaining limitations**: the live flight test itself; exact
  per-motor PWM decode is best-effort (`SERVO_OUTPUT_RAW`, honestly
  reported if absent); collision/contact and Webots-side realtime factor
  are not obtainable via MAVLink and are reported as such, not
  fabricated.

**No arm/takeoff/land command has been sent in this phase.**
