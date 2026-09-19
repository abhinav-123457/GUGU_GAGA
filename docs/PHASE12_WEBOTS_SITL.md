# Phase 12: Official ArduPilot SITL + Webots Closed-Loop Smoke Test

This phase uses the official ArduPilot Webots SITL integration. ArduPilot
SITL remains the autopilot authority. Webots provides the simulated
vehicle physics and sensors. No Pixhawk, serial port, radio, real
aircraft, outdoor vehicle, motor/PWM hardware, or hardware-in-the-loop
was used.

**This is genuinely different from Phase 11.** FlightGear (Phase 11) is a
read-only pose renderer with no physics of its own - it never computed
gravity, contact, or motor response; it only drew a position/attitude
this project already knew. Webots is a real physics simulator: gravity,
rigid-body dynamics, contact, and simulated sensors all happen *inside*
Webots, and ArduPilot SITL closes the loop by reading those sensors and
commanding Webots' own motors over the OFFICIAL ArduPilot Webots Python
protocol - never a protocol this project invented.

## Hardware target (this session, verified live)

| Property | Verified value |
|---|---|
| Model | Samsung Galaxy Book3 |
| CPU | 13th Gen Intel(R) Core(TM) i7-1355U, 10 cores / 12 logical processors (`Get-CimInstance Win32_Processor`) |
| RAM | 16,847,876,096 bytes (~15.7 GiB) (`Get-CimInstance Win32_ComputerSystem`) |
| GPU | Intel(R) Iris(R) Xe Graphics, driver `31.0.101.4502`, dated 2023-06-15 (`Get-CimInstance Win32_VideoController`) |
| OS | Windows 11 Home Single Language, `10.0.26200`, 64-bit |

**Intel Iris Xe compatibility warning (per this phase's own requirement)**:
Webots officially requires OpenGL 3.3 and recommends recent NVIDIA/AMD
graphics. Intel graphics "may work with a current driver but are not
guaranteed." This session's driver (`31.0.101.4502`, June 2023) is not
the newest available from Intel as of this writing. **Update**: this risk
did not materialize - see "Operator-verified GUI observation" below for
the live, stable result on this exact hardware/driver.

## Result this session

**Initially: Webots was not installed anywhere** (checked live on both
Windows and WSL2 `Ubuntu-22.04` - no `webots` on `PATH`, no standard
install directory, no matching `dpkg` package). This was reported
honestly as "Webots not installed / not live verified" - not claimed as
success - and Phase 11's FlightGear was not silently substituted for it
(`scripts/run_phase12_webots_smoke_test.py` never imports
`telemetry_mapping` or `FlightGearBridge` - see
`tests/test_phase12_webots_architecture.py::test_no_flightgear_or_pybullet_fallback`).

**Update: the operator installed Webots in WSL2 `Ubuntu-22.04`**, via the
apt package `webots` **version `2025a`** (binary `/usr/local/bin/webots`,
confirmed live via `dpkg -l` and `webots --version`). Re-running
`--dry-run --ardupilot-root /home/swarmbuild/ardupilot` now returns
`"webots_installation": {"found": true, ...}`, all three official files
found, `"ok": true`.

**Version compatibility caveat, not yet resolved**: ArduPilot's own docs
state the Webots-Python controller "was built for Webots 2023a and is
not backward compatible. Newer versions should also work, however."
R2025a is two major releases newer than the version this integration was
built against - this is a real, documented risk this phase has not yet
tested, not a confirmed pass. `--run` (the actual sensor/actuator smoke
test, requiring the operator to start Webots+SITL manually first) has
not yet been performed - see the next steps in this document's own
usage section above.

**Update: `--run` was performed live, against the real official example,
and succeeded.** The operator started SITL and Webots manually per the
official commands below:

```bash
cd /home/swarmbuild/ardupilot
Tools/autotest/sim_vehicle.py -v ArduCopter -w --model webots-python \
    --add-param-file=libraries/SITL/examples/Webots_Python/params/iris.parm
# separately:
webots libraries/SITL/examples/Webots_Python/worlds/iris.wbt
```

Webots' own console printed `Connected to ardupilot SITL (I0)` - the
exact documented handshake string, read directly from
`webots_vehicle.py::_handle_sitl` (see "Official example" below).

**Real finding, worth recording**: `sim_vehicle.py`'s primary MAVLink port
(`tcp:127.0.0.1:5760`) only actively serves its *first* connected client
(MAVProxy, which `sim_vehicle.py` launches automatically) - a second raw
TCP client completes the handshake but never receives a byte (confirmed
by a raw-socket read returning 0 bytes in 15s, while `/proc/<mavproxy_pid>/io`
showed MAVProxy's own `rchar` climbing continuously, proving SITL's loop
was running normally the whole time). This is standard ArduPilot/MAVProxy
practice, not a bug: additional consumers attach through a MAVProxy
output, added live via its own console with `output add
127.0.0.1:14551`, then pointing this script's `--connection` at
`udpin:127.0.0.1:14551` instead of the primary port.

With that connection, `--run --duration 15` returned:

```json
{
  "attach": {"succeeded": true, "reason": null},
  "heartbeat": {"received": true},
  "sensor_flow_evidence": "estimator_valid held true across the observation window",
  "armed": false, "estimator_valid": true,
  "telemetry_sample_count": 149,
  "controller_port_bound": true,
  "shutdown": {"clean": true},
  "remaining_failures": [],
  "ok": true
}
```

**This is genuine closed-loop evidence, not a pose-only render like Phase
11.** `estimator_valid` staying `true` for the full window means
ArduPilot's EKF was continuously consuming real IMU/GPS data that only
exists because Webots was computing real physics and sensor output every
tick - a static/frozen renderer could not produce this result. The
vehicle remained disarmed throughout (`armed: false`); no flight command
was sent. **The 2023a-vs-2025a version-compatibility risk flagged above
did not materialize for sensor flow** - R2025a interoperated correctly
with the official controller for this check. Actuator-direction PWM
values were not independently decoded by this script (honestly reported
as `actuator_flow_evidence`); whether the Webots window itself rendered
the model stably and without visual glitches for the full window is a
GUI observation this script cannot make - see the operator-verification
note in "Remaining limitations" below.

## Official example (verified against this session's real ArduPilot checkout)

ArduPilot commit `92b0cd788ec29406f26c6f9c31d5ceedbd1cc538` (Copter-4.6.3,
the same commit Phase 9-11 used), checked out at
`/home/swarmbuild/ardupilot` in WSL2 `Ubuntu-22.04`. All three official
files exist there, confirmed by direct file read (not assumed from the
docs):

| File | Path (relative to the ArduPilot checkout) |
|---|---|
| World | `libraries/SITL/examples/Webots_Python/worlds/iris.wbt` |
| Params | `libraries/SITL/examples/Webots_Python/params/iris.parm` |
| Controller | `libraries/SITL/examples/Webots_Python/controllers/ardupilot_vehicle_controller/ardupilot_vehicle_controller.py` (imports `webots_vehicle.py` in the same directory) |

`iris.wbt`'s `WorldInfo.basicTimeStep` is **`2`** ms - matches the official
requirement of 1 or 2 ms; not modified.

`--model webots-python` maps to `WebotsPython::create` in
`libraries/AP_HAL_SITL/SITL_cmdline.cpp:177` (confirmed by direct source
read). The controller (`webots_vehicle.py::_handle_sitl`) binds UDP port
`9002 + 10*instance` (`SIM_Webots_Python.h`'s own `_webots_port = 9002`
default), waits for SITL's first packet, prints exactly
`"Connected to ardupilot SITL (I<n>)"`, then sends FDM/sensor data to
`sitl_address:port+1` while receiving actuator data back on `port`.

## Exact commands (from the official docs, cross-checked against the real source)

**SITL** (WSL2, Arrangement A - everything local):
```bash
cd /home/swarmbuild/ardupilot
Tools/autotest/sim_vehicle.py -v ArduCopter -w --model webots-python \
    --add-param-file=libraries/SITL/examples/Webots_Python/params/iris.parm
```

**SITL** (Arrangement B - Webots on Windows, SITL in WSL2), add:
```bash
--sim-address=<Windows-host-IP>
```

**Webots**: open `libraries/SITL/examples/Webots_Python/worlds/iris.wbt`
in the Webots GUI. For Arrangement B, also edit the world's
`controllerArgs` to add `"--sitl-address" "<WSL-IP>"` (per the official
docs) before loading it, or pass it via Webots' own controller-args
mechanism.

**This project's smoke test** (never spawns either of the above - see
"Safety" below):
```bash
# Phase 12A - hardware/file-presence diagnostic, no socket opened:
python scripts/run_phase12_webots_smoke_test.py --dry-run \
    --world iris.wbt --vehicle iris --ardupilot-root /home/swarmbuild/ardupilot

# Phase 12B - verify an ALREADY-RUNNING official example (Arrangement A):
python scripts/run_phase12_webots_smoke_test.py --run \
    --world iris.wbt --vehicle iris --duration 60

# Arrangement B - addresses must be supplied by the operator, never guessed:
python scripts/run_phase12_webots_smoke_test.py --run \
    --world iris.wbt --vehicle iris \
    --sitl-address <WSL-IP> --sim-address <WINDOWS-HOST-IP> --duration 60
```

`<WSL-IP>`/`<WINDOWS-HOST-IP>` are placeholders in this document only -
the script itself has no default for either flag (`None`) and rejects
any value that is not loopback or RFC1918-private
(`validate_local_or_private_address`), so a real run cannot proceed
without the operator's own real addresses.

## Verified live this session (`--dry-run`, on this actual machine)

```json
{
  "operating_system": "Windows-11-10.0.26200-SP0",
  "cpu_count": 12,
  "memory_total_bytes": 16847876096,
  "webots_installation": {"found": false, "path": null, "method": "not_found"},
  "remaining_failures": ["Webots not installed / not live verified"],
  "ok": false
}
```
And under WSL2, pointed at the real checkout (`--ardupilot-root
/home/swarmbuild/ardupilot`): `world_exists`/`params_exists`/
`controller_exists` all `true`, `webots_installation.found: false`,
`ok: false` - exactly the honest "files present, engine absent" state.

## Safety

Two profiles exist; only one is implemented:

- `webots_offline_smoke_test` (**default, only implemented profile**):
  never arms, never takes off, never lands, never sends a MAVLink flight
  command, never writes a parameter, never spawns or kills Webots/
  ArduPilot, never touches the swarm planner/consensus code.
- `webots_sitl_flight_test` (**not implemented** - selecting it via
  `--profile` produces a configuration error, never a silent fallback to
  the smoke-test behavior).

`scripts/run_phase12_webots_smoke_test.py` never spawns or kills Webots
or ArduPilot SITL - the operator starts both manually (matching Phase
9/10/11's own ATTACH-only pattern); `--run` only reads a real MAVLink
heartbeat/EKF-valid state from an already-running SITL (reusing
`ArduPilotSITLTransport`, unmodified) and non-invasively checks whether
the Webots controller's own UDP port already has something bound to it.
See `tests/test_phase12_webots_architecture.py` (24 checks) and
`tests/test_phase12_webots_config.py` (20 checks) for the full structural
and behavioral proof, including: no `subprocess`/hardware imports, no
flight-command identifiers anywhere, no hardcoded public IP, explicit-
only `--sitl-address`/`--sim-address` (validated loopback/RFC1918-private,
rejecting public/global addresses), Phase 10/11/FakeSITL all still
verified unaffected, and clean handling of Webots-not-installed/missing-
official-files/connection-timeout without ever hanging or raising
uncaught.

## Sensor/actuator verification design (for when Webots is installed)

A visual quadcopter model alone is not proof of closed-loop physics.
`--run` gathers two independent, read-only pieces of evidence instead of
trusting the window:

1. **Sensor flow (indirect but real)**: attaches read-only over MAVLink
   and observes `estimator_valid`. ArduPilot's EKF can only initialize
   and stay valid by continuously consuming real IMU/GPS data - if
   Webots' sensors were not reaching SITL, the estimator could not stay
   valid. This is the same estimator-validity signal Phase 9/10 already
   rely on as real evidence, not a new invented proxy.
2. **Actuator/controller-channel evidence**: a non-invasive bind-test on
   the documented Webots controller UDP port (`9002 + 10*instance`) -
   if something is already bound there, the official controller process
   is genuinely running and exchanging data on the documented port,
   without this script itself joining that channel or sending anything
   through it.

Decoding individual motor PWM values would require either the operator's
own controller console output or instrumenting Webots' own log - the
script says so honestly (`actuator_flow_evidence`) rather than
fabricating a per-motor readout it cannot actually observe.

## Operator-verified GUI observation (Iris Xe, WSL2, real screenshot this session)

The operator shared a live screenshot of the running Webots window,
confirmed as follows (**operator-verified, not project-automated** - see
this phase's own requirement to label manual observations honestly):

- Console shows exactly: `Listening for ardupilot SITL (I0) at
  127.0.0.1:9002` then `Connected to ardupilot SITL (I0)` - the real
  handshake sequence, matching `webots_vehicle.py`'s own source exactly.
- Simulation time elapsed `0:04:22.294` running at `0.82x` realtime speed
  on Intel Iris Xe integrated graphics - **stable well beyond the
  required 60 seconds**, with no crash and no repeated errors.
- The Iris quadcopter model rendered correctly (body, four motors/props
  visible), not frozen, sitting still (consistent with `armed: false`).
- The GUI (timeline controls, scene tree, console, text editor panes)
  was fully interactive and responsive throughout.
- Two `ERROR` lines appeared exactly once each: `Extrusion.proto:771:5:
  ... Skipped unknown 'solid' field` - a benign R2023a-vs-R2025a
  proto-schema mismatch on the unrelated road-scenery mesh, not the Iris
  vehicle or the ArduPilot link. Two `WARNING` lines (a non-power-of-two
  advertising-board texture, and an Iris mesh vertex-count advisory) are
  likewise cosmetic. None of the four affects the sensor/actuator link,
  and none repeated/flooded the console.

**Hardware-acceptance verdict for this session: Intel Iris Xe, driver
`31.0.101.4502`, on this exact Galaxy Book3, runs the official
ArduPilot+Webots Iris example reliably** - GUI stable, model correctly
rendered, real closed-loop sensor evidence (`estimator_valid` held true),
running at a modest but entirely workable `0.82x` realtime on integrated
graphics alone. This directly answers the phase's own open hardware-risk
question, in the machine's favor, for this specific check.

## Remaining limitations

- Exact OpenGL version and precise CPU/memory-under-load numbers were
  not captured this session (the `0.82x` realtime factor is the
  strongest available proxy for performance headroom).
- No propeller motion, vehicle lift, or landing was observed - none was
  attempted; the vehicle was never armed in this phase, and stayed
  `armed: false` throughout the verified run.
- The Arrangement B (Windows Webots + WSL2 SITL) networking path is
  documented and safety-checked but not live-exercised - this session
  used Arrangement A (everything in WSL2) throughout.
- `--run`'s actuator-flow check is evidence (EKF validity + controller
  UDP port bound), not a full per-channel PWM decode - documented above
  as an honest scope limit, not a gap to paper over.
- The `output add 127.0.0.1:14551` step is a manual, operator-run
  MAVProxy console command - it is not automated by this project's
  script (which never spawns or controls MAVProxy), and must be repeated
  each time SITL/MAVProxy is restarted.

**No arm/takeoff/land command was sent in this phase.**
