# Phase 10: One-Vehicle SITL Flight-Readiness and Controlled-Flight Test

Strictly simulator-only. ATTACH mode only - this phase never spawns or
kills a process (same invariant as Phase 9). Never connects to anything
but `127.0.0.1`/`localhost`. No Pixhawk, serial port, radio, real
aircraft, outdoor vehicle, motor, PWM output, or hardware-in-the-loop was
used anywhere in this phase.

**Exact ArduPilot version**: `Copter-4.6.3`, commit `92b0cd788ec29406f26c6f9c31d5ceedbd1cc538`
(unchanged from Phase 8/9 - the same build, same checkout).

## Step 1: pre-arm and battery diagnosis

### "PreArm: 3D Accel calibration needed" - root cause (source-verified)

The manual command used throughout Phase 8/9 does not pass `--defaults`:

```bash
./build/sitl/bin/arducopter --model quad --speedup 1 -I0 \
    --home -35.363261,149.165230,584,353 --sysid 1
```

On a fresh EEPROM this leaves `INS_ACCOFFS_X/Y/Z` at their compiled-in
factory default of exactly `0.0`. ArduCopter's own arming check is, byte
for byte, in `libraries/AP_InertialSensor/AP_InertialSensor.cpp`:

```cpp
bool AP_InertialSensor::accel_calibrated_ok_all() const
{
    for (uint8_t i=0; i<get_accel_count(); i++) {
        if (!_accel_id_ok[i]) {
            return false;
        }
        // exactly 0.0 offset is extremely unlikely
        if (_accel_offset(i).get().is_zero()) {
            return false;
        }
        // zero scaling also indicates not calibrated
        if (_accel_scale(i).get().is_zero()) {
            return false;
        }
        ...
```

called from `libraries/AP_Arming/AP_Arming.cpp`'s `ins_checks()`:

```cpp
if (!ins.accel_calibrated_ok_all()) {
    check_failed(ARMING_CHECK_INS, report, "3D Accel calibration needed");
    return false;
}
```

An exactly-zero `INS_ACCOFFS_*` is ArduPilot's own, hard-coded signal for
"never calibrated" - this is not a swarm_sim bug, not a Mission Planner
bug, and not something this project's code can or should paper over.

(Note: SITL's `AP_InertialSensor::register_accel()` unconditionally forces
`_accel_id_ok[i] = true` under `CONFIG_HAL_BOARD == HAL_BOARD_SITL` - the
accelerometer-*ID*-matching half of the check is always satisfied in
SITL, by ArduPilot's own design; the failing half is exclusively the
zero-offset check above.)

**Standard fix**: ArduPilot's own official SITL parameter file,
`Tools/autotest/default_params/copter.parm` - the same file
`Tools/autotest/sim_vehicle.py` (the official SITL launch helper) applies
automatically for every quad - contains, verbatim:

```
# we need small INS_ACC offsets so INS is recognised as being calibrated
INS_ACCOFFS_X   0.001
INS_ACCOFFS_Y   0.001
INS_ACCOFFS_Z   0.001
INS_ACCSCAL_X   1.001
INS_ACCSCAL_Y   1.001
INS_ACCSCAL_Z   1.001
INS_ACC2OFFS_X   0.001
INS_ACC2OFFS_Y   0.001
INS_ACC2OFFS_Z   0.001
INS_ACC2SCAL_X   1.001
INS_ACC2SCAL_Y   1.001
INS_ACC2SCAL_Z   1.001
```

This is ArduPilot's own documented convention for "recognised as
calibrated" in simulation - not a workaround invented by this project,
and not "faking accelerometer calibration" in the sense the task
prohibits (nothing in `swarm_sim` fabricates a calibration result; this
is configuring the *simulator itself* via its own official,
file-based mechanism, exactly as "prefer the standard ArduPilot SITL
launch/configuration procedure" asks for).

### "Battery: 0.00 V, 0.0 A, 0%" - root cause (source-verified)

`BATT_MONITOR`'s compiled-in default is `AP_BattMonitor::Type::NONE` (0) -
`libraries/AP_BattMonitor/AP_BattMonitor_Params.cpp`:

```cpp
AP_GROUPINFO_FLAGS("MONITOR", 1, AP_BattMonitor_Params, _type,
                    int8_t(AP_BattMonitor::Type::NONE), AP_PARAM_FLAG_ENABLE),
```

This is a **missing battery monitor configuration** (one of the four
possibilities the task asked to distinguish between) - not a Mission
Planner display bug and not a swarm_sim bug. ArduCopter's SITL physics
backend always internally simulates a battery regardless
(`libraries/SITL/SITL.cpp`: `AP_GROUPINFO("BATT_VOLTAGE", 19, SIM,
batt_voltage, 12.6f)` - a fully-charged 12.6V simulated pack by default),
but that value is only ever reported over MAVLink once a real
`BATT_MONITOR` type is configured. The standard `copter.parm` file sets:

```
BATT_MONITOR    4
MOT_BAT_VOLT_MIN 9.6
MOT_BAT_VOLT_MAX 12.8
```

(`4` = Analog Voltage and Current.)

### Parameter change record

| Parameter | Factory/observed value | Standard `copter.parm` value | Reason | Limited to SITL? | `ARMING_CHECK` affected? |
|---|---|---|---|---|---|
| `INS_ACCOFFS_X/Y/Z` | `0.0` (uncalibrated) | `0.001` | ArduPilot's own "recognised as calibrated" convention for simulated IMUs | Yes - this file is exclusively used for SITL launches | No |
| `INS_ACCSCAL_X/Y/Z` | `1.0` (already fine) | `1.001` | Set redundantly alongside the offsets by the same official file | Yes | No |
| `INS_ACC2OFFS_X/Y/Z` | `0.0` | `0.001` | Second simulated IMU, same reason | Yes | No |
| `INS_ACC2SCAL_X/Y/Z` | `1.0` | `1.001` | Second simulated IMU, same reason | Yes | No |
| `BATT_MONITOR` | `0` (NONE) | `4` (Analog Voltage and Current) | Makes SITL's already-simulated battery voltage/current visible over MAVLink | Yes - `BATT_MONITOR=4` reads simulated analog pins that only exist in SITL's physics backend | No |
| `MOT_BAT_VOLT_MIN`/`MOT_BAT_VOLT_MAX` | compiled defaults | `9.6`/`12.8` | Lets ArduCopter compute a meaningful battery percentage from the simulated voltage | Yes | No |

**`ARMING_CHECK` proof it stays enabled**: its compiled-in default is
`ARMING_CHECK_ALL` (`libraries/AP_Arming/AP_Arming.cpp`:
`AP_GROUPINFO("CHECK", 8, AP_Arming, checks_to_perform, ARMING_CHECK_ALL)`)
- **all checks enabled**. `Tools/autotest/default_params/copter.parm`
does not mention `ARMING_CHECK` at all, so it is never touched, in either
direction, by the standard-procedure fix above. `scripts/run_phase10_sitl_flight_test.py`
never calls `param_set`/`param_set_send` anywhere (see
`tests/test_phase10_sitl_flight_test_architecture.py::test_never_calls_param_set_anywhere_in_module`)
- every parameter this project's own code touches is **read-only**.

### Comparison with the official ArduPilot SITL procedure

The official helper is `Tools/autotest/sim_vehicle.py -v ArduCopter`,
which automatically resolves and passes the correct `--defaults` file for
the chosen frame, wipes EEPROM only on a genuinely first run, and
launches MAVProxy alongside (optional, `--no-mavproxy` to skip it). The
recommended exact command for this project's own manual, MAVProxy-free
workflow, extending the Phase 9 command with the one missing standard
flag, is:

```bash
cd /home/swarmbuild/ardupilot
./build/sitl/bin/arducopter \
    --model quad \
    --speedup 1 \
    -I0 \
    --home -35.363261,149.165230,584,353 \
    --sysid 1 \
    --defaults Tools/autotest/default_params/copter.parm
```

or, using the fully official helper:

```bash
cd /home/swarmbuild/ardupilot
Tools/autotest/sim_vehicle.py -v ArduCopter --no-mavproxy -I0 \
    --home=-35.363261,149.165230,584,353 --sysid=1
```

### SITL-only geofence configuration procedure (required - `FENCE_ENABLE=0` blocks the flight-test gate)

Neither command above enables the geofence. `AC_Fence.cpp`'s own
compiled-in default (commit `92b0cd78`) is `AP_GROUPINFO("ENABLE", 0,
AC_Fence, _enabled, 0)` - disabled. `run_sitl_flight_test`'s
`_check_geofence_gate()` reads the live `FENCE_ENABLE` parameter after
attach and refuses to proceed (no GUIDED mode change, no arm command) if it
is `0` - confirmed live in this session's own diagnostic run
(`FENCE_ENABLE=0.0`, all other diagnostics clean).

**Exact parameters and the bounded values this project recommends**, all
read from `libraries/AC_Fence/AC_Fence.cpp` at commit `92b0cd78`:

| Parameter | Compiled-in default | Recommended bounded value for this 2m test | Reason |
|---|---|---|---|
| `FENCE_ENABLE` | `0` (disabled) | `1` | Required - `_check_geofence_gate` refuses to proceed while this is `0`. |
| `FENCE_TYPE` | `7` (`ALT_MAX\|CIRCLE\|POLYGON`) | `7` (unchanged) | Already covers an altitude ceiling and a horizontal boundary; the min-altitude/floor bit (`8`) is intentionally left out - this test's flight never goes below ground level, so a floor fence adds no protection. |
| `FENCE_ALT_MAX` | `100.0` m | `10` m | 5x the default `--max-altitude` (2.0m) and under the code's own hard cap (`HARD_MAX_ALTITUDE_M = 3.0` m) - a real, bounded backstop instead of a 100m formality. |
| `FENCE_RADIUS` | `300.0` m (`copter.parm` itself sets `150.0` m) | `10` m | This test's manual sequence never sends a horizontal setpoint at all (GUIDED's own station-keeping after `MAV_CMD_NAV_TAKEOFF` is what "hold" means here), so 10m is a generous but still tightly bounded horizontal margin around a vehicle that should not be drifting anywhere. |
| `FENCE_MARGIN` | `2.0` m | `2` m (unchanged) | ArduPilot's own default buffer distance before a breach is declared - reasonable relative to a 10m radius, no override needed. |
| `FENCE_ACTION` | `1` (`RTL_AND_LAND`) | `1` (unchanged) | Already a conservative, safe breach response. |

**Exact operator configuration command** (documented `--defaults`
mechanism - `sim_vehicle.py`/`arducopter` both accept a comma-separated
list of parameter files, applied in order, later files overriding
earlier ones; this is ArduPilot's own official mechanism, not something
this project invented):

```bash
cd /home/swarmbuild/ardupilot
./build/sitl/bin/arducopter \
    --model quad \
    --speedup 1 \
    -I0 \
    --home -35.363261,149.165230,584,353 \
    --sysid 1 \
    --defaults Tools/autotest/default_params/copter.parm,/mnt/c/Users/abhin/OneDrive/Desktop/swarm/docs/phase10_sitl_geofence.parm
```

**The second path must be absolute (or otherwise resolvable from the
directory `arducopter` is launched from), not a path relative to the
`swarm` project.** `arducopter` is launched with its working directory set
to the ArduPilot checkout (`/home/swarmbuild/ardupilot` above) - a bare
`docs/phase10_sitl_geofence.parm` resolves against *that* directory, which
has its own unrelated `docs/` folder, not this project's. Getting this
wrong produces exactly `PANIC: Failed to load defaults from
Tools/autotest/default_params/copter.parm,docs/phase10_sitl_geofence.parm`
(source-confirmed: `AP_Param::load_defaults_file()` in
`libraries/AP_Param/AP_Param.cpp` `strtok_r`-splits the comma list
correctly, then calls `count_defaults_in_file()` on each path in turn and
panics with the *original, unsplit* string if any single one can't be
opened - so the panic text doesn't tell you which of the two paths
failed). Use the WSL path to wherever this project is actually checked
out, e.g. for a project at Windows path `C:\Users\<you>\...\swarm`, the
WSL2-mounted equivalent is `/mnt/c/Users/<you>/.../swarm/docs/phase10_sitl_geofence.parm`.

The second file is this project's own
[`docs/phase10_sitl_geofence.parm`](phase10_sitl_geofence.parm), containing
exactly the six lines in the table above with their reasoning as comments.
It is a **SITL startup parameter file**, applied once at EEPROM/parameter
load time by ArduPilot itself - not a runtime write. Nothing in
`swarm_sim`/`scripts/run_phase10_sitl_flight_test.py` ever calls
`param_set`/`param_set_send` (see
`tests/test_phase10_sitl_flight_test_architecture.py::test_never_calls_param_set_anywhere_in_module`
and the new
`test_geofence_gate_never_calls_param_set`/`test_flight_test_never_sends_param_set`
tests, which additionally prove this live at the wire level via the test
double).

**Restart required**: `--defaults` is only read at process startup - an
already-running `arducopter` instance must be stopped and relaunched with
the command above for `FENCE_ENABLE`/`FENCE_TYPE`/`FENCE_ALT_MAX`/`FENCE_RADIUS`
to take effect. A parameter change made live afterwards (e.g. via Mission
Planner's or MAVProxy's own `param set FENCE_ENABLE 1` - also a
documented, official ArduPilot mechanism, just an operator command instead
of a file) does not require a restart and is an acceptable alternative to
editing the defaults file, but this project's own code still never issues
that command on the operator's behalf.

## An environment-level finding from this phase's own diagnosis (read before attempting a live test)

While diagnosing Step 1 in this development session, every attempt to
connect *any* MAVLink or even raw-TCP client (Python/pymavlink, a bare
socket, with or without `--wipe`/`--defaults`, before and after a full
`wsl --shutdown` restart) to a freshly-launched ArduCopter SITL instance
on this particular machine caused the process to accept the TCP
connection, print `Smoothing reset at 0.001`, and immediately `exit(1)`,
with **zero bytes ever exchanged**. A pure Windows-native loopback TCP
test (no WSL involved at all) worked normally in the same session,
isolating the fault to the Windows-to-WSL2 forwarded-port path
specifically, not a general host or ArduCopter-code problem. This is the
same crash signature Phase 8 called "SPAWN-mode crash-on-connect", now
observed on a plain, shell-launched instance - which was reliably stable
throughout Phase 8/9's own extensive testing, including the Mission
Planner session shown in this project's own Phase 9 verification.

**This finding blocked live execution of the actual arm/takeoff/land
sequence in this development session.** Per this phase's own explicit
instruction ("Do not run the arm/takeoff test until the pre-arm report is
clean and the implementation report has been reviewed"), the correct,
honest action is to stop and report rather than force a workaround or
claim a result that was not observed. Everything below Step 1 was
therefore implemented and verified against
`tests/_dummy_ardupilot_vehicle.DummyArduPilotVehicle` (the same
real-MAVLink-over-a-real-localhost-socket fixture Phase 8/9's own tests
use) rather than the real ArduCopter SITL binary - this is explicitly
**not** the same as a live confirmation against real ArduPilot, and this
document does not claim otherwise anywhere below.

## Step 2: profiles and required gates

Two profiles, never merged:

```
profile = "telemetry_only"     # default - scripts/run_phase10_sitl_flight_test.py --attach
profile = "sitl_flight_test"   # explicit opt-in - see gate list below
```

`sitl_flight_test` requires **every** one of the following before any
command that could arm the vehicle is sent (`_check_flight_test_gate` in
`scripts/run_phase10_sitl_flight_test.py`, plus the live checks performed
during `run_sitl_flight_test` itself):

| Requirement | Where enforced |
|---|---|
| `--sitl-only` | gate |
| `--allow-sitl-arm-test` | gate |
| exactly one vehicle | structural - the script has no multi-vehicle option at all |
| `127.0.0.1`/`localhost` connection | gate + `ArduPilotSITLTransport`'s own (Phase 8) enforcement |
| expected system ID / component ID | ATTACH heartbeat-wait (Phase 8, unmodified) - a wrong identity never satisfies startup |
| namespace `sitl/drone0` | gate |
| confirmed ArduPilot version | live - `AUTOPILOT_VERSION` must be received |
| successful heartbeat | live |
| valid estimator state | live - `telemetry.estimator_valid` |
| valid telemetry | live - `telemetry.position_m is not None` |
| valid battery state or documented SITL battery model | live - either a real reading, or an explicit "`BATT_MONITOR` disabled, documented SITL default" note - never fabricated |
| geofence enabled | live - `_check_geofence_gate()` reads `FENCE_ENABLE`, must be non-zero; a bounded local fence (`FENCE_TYPE`/`FENCE_ALT_MAX`/`FENCE_RADIUS`/`FENCE_MARGIN`) is separately recommended - see the "SITL-only geofence configuration procedure" above |
| command timeout configured | gate - `_ARM_TAKEOFF_LAND_COMMAND_TIMEOUT_S` finite and positive |
| operator acknowledgement | `--confirm-sitl-flight-test`, or an interactive `yes` prompt if stdin is a tty |
| no hardware endpoint | structural - only `ArduPilotVehicleEndpoint` with `executable_path=None` (ATTACH only) is ever constructed |
| no PyBullet mission | structural - this script never imports `pybullet`/constructs `FloodSearchMission` |
| no distributed swarm planner | structural - never imports `distributed_consensus` |
| no automatic fallback to another mode | `run_sitl_flight_test` returns a failed report on any gate/step failure - it never calls another mode's function internally |

If any requirement fails, the function returns immediately with the exact
reason recorded in `remaining_failures` - no arm command is ever sent.

## Step 3: the controlled flight sequence

Implemented in `run_sitl_flight_test()`:

1. Attach, verify heartbeat.
2. Identity verified (heartbeat only ever matches the configured system/component ID - Phase 8, unmodified).
3. Confirm ArduPilot version (`AUTOPILOT_VERSION`).
4. Verify telemetry and estimator validity.
5. Verify vehicle is disarmed (refuses to proceed if already armed).
6. Verify battery state, and evaluate the geofence gate (`_check_geofence_gate()` - refuses to proceed while live `FENCE_ENABLE` reads `0`).
7. Verify pre-arm STATUSTEXT stream is clear (no `PreArm: ...` messages in a 6s window).
8. Set GUIDED mode, wait for a real `COMMAND_ACK`.
9. Send arm (`MAV_CMD_COMPONENT_ARM_DISARM`, `param1=1`, `param2=0` - **never** the force-arm magic value) only now that every check above has passed.
10. Confirm armed state from a real heartbeat/telemetry poll (bounded).
11. Send `MAV_CMD_NAV_TAKEOFF` to `--max-altitude` meters, confirm a real, non-trivial altitude climb is observed (never claimed without evidence).
12. Hold for `--duration` seconds, polling telemetry throughout.
13. Send `MAV_CMD_NAV_LAND`, confirm disarm (falling back to one explicit, non-forced disarm if auto-disarm-after-landing hasn't fired within a bounded window), then `transport.stop()` - no further flight command is ever sent.

This is a **dedicated, explicit, manual SITL test command path** - see the
script's own module docstring's "Command authority" section for why it
talks directly to the open pymavlink connection rather than through
`ArduPilotSITLTransport.send_command()` (which structurally can never
reference an arm/takeoff identifier at all - Phase 8's own guarantee,
unmodified this phase).

### Conservative limits

| Limit | Default | Hard cap (code-enforced, not just documented) |
|---|---|---|
| Vehicles | 1 | 1 (structural) |
| Max altitude | 2.0 m | 3.0 m |
| Max horizontal speed | 0.5 m/s | 1.0 m/s (reserved - this minimal sequence never commands a horizontal velocity at all; GUIDED's own station-keeping after `MAV_CMD_NAV_TAKEOFF` is what "hold" means here) |
| Max test duration | 60 s | 120 s |
| Command ack timeout | 10 s | - |
| Heartbeat/startup timeout | 20 s (`--startup-timeout`) | - |

**Emergency behavior**: if the vehicle is ever armed and something after
that fails or times out (heartbeat never confirms armed, takeoff
rejected, LAND rejected), `emergency_cleanup()` immediately sends
`MAV_CMD_NAV_LAND` and, if still armed after that, one plain (never
forced) disarm - recorded in the report's `emergency_action` field. RTL is
never sent (explicitly out of scope - "no RTL unless explicitly tested
separately"). No swarm behavior, autonomous search, obstacle course, or
external endpoint is ever involved.

If ArduPilot rejects arming or takeoff, the exact `COMMAND_ACK` result and
any `PreArm`/rejection `STATUSTEXT` are recorded in the report and the
function returns immediately - never retried, never forced, never
bypassed (verified: `tests/test_phase10_sitl_flight_test.py::test_flight_test_stops_on_rejected_arm_never_bypasses`).

## Step 4: command authority and logging

```
candidate -> SafetySupervisor.evaluate() -> accepted filtered command -> AdapterCommand -> ArduPilotSITLTransport
```

remains exactly Phase 6-9's chain for `hold_test`/`planner_preview`
(imported and reused from `scripts/run_phase9_single_vehicle_visual.py`
completely unmodified - Phase 9's own telemetry/hold/planner behavior is
untouched, per this phase's explicit instruction). `sitl_flight_test`'s
arm/GUIDED/takeoff/land sequence is the one, deliberately separate,
manual test command path described above.

Explicitly separate modes, results never merged (each produces its own
report shape and its own `results/phase10_sitl_flight_test/*.json` file):

| Mode | Source |
|---|---|
| `telemetry_only` | Phase 9's `run_telemetry_visualization`, reused via `--attach` |
| `prearm_diagnostics` | new this phase, read-only |
| `hold_test` | Phase 9's `run_hold_command_test` (`scripts/run_phase9_single_vehicle_visual.py --hold-test`) |
| `sitl_flight_test` | new this phase |
| `planner_preview` | Phase 9's `run_planner_preview` (`scripts/run_phase9_single_vehicle_visual.py --planner-preview`) |
| FakeSITL | Phase 7, unaffected |
| PyBullet simulation | `run_mission.py --gui`, unaffected |

Every event in `sitl_flight_test` (attach, mode change, arm attempt,
armed-confirmed, takeoff attempt, takeoff-confirmed, hold-complete, land
attempt, emergency action) is appended to the report's `events` list with
a monotonic timestamp; the full report additionally records vehicle
ID/namespace/system/component ID, heartbeat, estimator state, battery
state, pre-arm messages, mode changes (via `COMMAND_ACK` results),
arm/disarm state, takeoff altitude observed, landing/disarm confirmation,
timeouts, failure reasons, and any emergency action taken.

## Step 5: safety architecture

`tests/test_phase10_sitl_flight_test_architecture.py` (21 tests) proves,
via AST inspection of `scripts/run_phase10_sitl_flight_test.py`:

- the default CLI arguments never select the flight test (`sitl_only`/`allow_sitl_arm_test`/`confirm_sitl_flight_test` all default `False`);
- `run_dry_run`/`run_prearm_diagnostics` never reference an arm/takeoff identifier;
- the gate literally checks `args.sitl_only`/`args.allow_sitl_arm_test`;
- operator confirmation is checked before the endpoint is even built, which is checked before `transport.start()`;
- every `ArduPilotSITLTransport(...)` construction uses a single-entry vehicle dict (one vehicle only, structurally);
- no `num_drones`/`VEHICLE_IDS`-style multi-vehicle identifier exists anywhere;
- no `param_set`/`param_set_send` call exists anywhere in the module (parameters are read-only);
- the force-arm/disarm magic value (`21196`) never appears anywhere in the module;
- the manual flight path never imports `CandidateCommand`/`SafetySupervisor`/`evaluate`;
- no `distributed_consensus`/`pybullet`/`FloodSearchMission` reference exists;
- the manual flight path never calls `ArduPilotSITLTransport.send_command()`;
- `ardupilot_transport.py` itself (Phase 8, unmodified) still never references an arm/takeoff identifier (regression re-check);
- Phase 9's own module is imported, not reimplemented;
- `_check_geofence_gate()` literally checks `fence_enabled`, never calls `param_set_send`/`mav_param_set_send`;
- `run_sitl_flight_test` calls `_check_geofence_gate()` before setting GUIDED mode or arming.

`tests/test_phase10_sitl_flight_test.py` (26 tests) proves the behavioral
side against `_FlightSimVehicle` (a test-only, real-MAVLink-over-localhost
extension of Phase 8/9's own `DummyArduPilotVehicle`, living entirely in
this test file - the shared fixture itself is never modified): every
required gate individually refuses to arm when missing; wrong system ID
is rejected; `FENCE_ENABLE=0` blocks flight-test authorization
(`geofence_gate["passed"] is False`); `FENCE_ENABLE=1` with a realistic
bounded local fence (`FENCE_TYPE=7`/`FENCE_ALT_MAX=10`/`FENCE_RADIUS=10`/`FENCE_MARGIN=2`,
matching `docs/phase10_sitl_geofence.parm`) passes that gate and lets the
sequence proceed to arm; a rejected arm attempt is never retried or
bypassed and the vehicle never reports armed; a full
happy-path arm-takeoff-hold-land-disarm sequence produces a real,
non-trivial observed altitude climb and an honest report, and an
independent live check confirms `ARMING_CHECK` was never touched and no
`PARAM_SET` was ever sent; a takeoff
rejected after a successful arm triggers the emergency LAND/disarm path;
`prearm_diagnostics` reads real parameters, never sends an arm command,
and never sends a `PARAM_SET`; `--attach` delegates to Phase 9's own
function; and FakeSITL remains importable and functional.

## Mission Planner observation procedure

1. Start ArduCopter SITL with the standard-procedure command above (Step 1), extended with the geofence `--defaults` file from the "SITL-only geofence configuration procedure" section (`--defaults Tools/autotest/default_params/copter.parm,<absolute-path-to-swarm>/docs/phase10_sitl_geofence.parm` - the second path must be absolute, see that section's warning about `arducopter`'s working directory) - this requires a fresh restart, not a live parameter change, since `--defaults` is only read at startup.
2. Alternatively, without restarting: set `FENCE_ENABLE=1`, `FENCE_TYPE=7`, `FENCE_ALT_MAX=10`, `FENCE_RADIUS=10`, `FENCE_MARGIN=2` via Mission Planner's parameter list (a documented, official ArduPilot mechanism - this project's own code never issues this write itself).
3. Confirm in Mission Planner's HUD that `PreArm` messages have cleared and battery shows a real value.
4. Run `python scripts/run_phase10_sitl_flight_test.py --diagnose-prearm ...` first and confirm a clean report.
5. Watch Mission Planner's map/HUD while a human operator runs the flight-test command with explicit confirmation - the vehicle icon should climb to the configured altitude, hold, then descend and show `DISARMED` again.

## Required validation commands

```bash
python -m compileall -q swarm_sim run_mission.py tests scripts
python -m pytest -q tests
```

Both were run in this session: **630 tests passed**, compile clean (up
from 624 - 5 new functional tests plus 3 new architecture tests for the
geofence gate, minus 2 removed/merged in the earlier gate-flag test fix).

## Results (this session)

- **Whether the vehicle actually armed**: **No** - the live flight test was never run against real ArduCopter SITL in this session, due to the environment-level connection regression documented above. `sitl_flight_test`'s gating, sequencing, rejection-handling, and emergency-cleanup logic were fully verified against the simulated-flight test double instead.
- **Whether it actually took off**: **No**, for the same reason.
- **Whether it landed and disarmed**: **Not applicable** - flight was never attempted live.
- **All failures**: the WSL2-forwarded-port connection regression (isolated to Windows<->WSL2 specifically; a pure Windows-native loopback TCP test worked normally in the same session; a full `wsl --shutdown` restart did not resolve it).
- **Remaining risks**: (1) the connection regression itself must be resolved (or independently reproduced/ruled out on a clean host) before any live arm/takeoff test is attempted; (2) the operator must explicitly set `FENCE_ENABLE=1`/`FENCE_TYPE`/`FENCE_ALT_MAX` before the gate will pass, since neither the plain command nor `copter.parm` enables it; (3) the live flight sequence's timing constants (climb/disarm timeouts) were chosen conservatively but have not been tuned against real ArduCopter SITL physics, only the simulated test double.

Nothing in this phase claims flight success without an actually-observed,
non-trivial altitude change - see `report["flight_actually_happened"]`,
which is only ever set `True` after a real climb is observed via live
telemetry.

## Update: geofence gate formalized, live pre-arm diagnostic now confirmed clean

In a later session, the operator reported running `--diagnose-prearm`
against a real, live ArduCopter SITL instance (the environment-level
connection regression above no longer reproduced on their host) and got a
clean report: heartbeat received, attach succeeded, `ARMING_CHECK=1`,
accelerometer offsets non-zero, `BATT_MONITOR=4`, battery fraction `1.0`,
no pre-arm messages, clean shutdown - **except** `FENCE_ENABLE=0.0`. This
independently confirms the Step 1 diagnosis above against a real vehicle,
not just the simulated test double.

In response, the geofence check (previously an inline check inside
`run_sitl_flight_test`) was formalized into its own `_check_geofence_gate()`
function, given its own `report["geofence_gate"]` result shape (matching
`_check_flight_test_gate`'s `{"passed", "checks", "reasons_failed"}`
shape), and documented above under "SITL-only geofence configuration
procedure" with an exact bounded-fence parameter file
([`docs/phase10_sitl_geofence.parm`](phase10_sitl_geofence.parm)). This did
**not** change: `telemetry_only`/`prearm_diagnostics` modes (untouched);
`ARMING_CHECK` (never referenced as a value to write, before or after);
whether the code arms/takes off by default (still never, and the live
arm/takeoff/land test was **still not run** in this update - only
`--diagnose-prearm` was reported as run, which never arms). No parameter
write was added anywhere - see the new
`test_geofence_gate_never_calls_param_set` (AST) and
`test_flight_test_never_sends_param_set`/`test_prearm_diagnostics_never_sends_param_set`
(live, wire-level: the test double now records any `PARAM_SET` it
receives and both tests assert that list stays empty) tests.
