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
the newest available from Intel as of this writing - **this is a real,
documented risk, not resolved by this phase**. See "Result" below.

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

**This phase stops here for live verification.** Installing Webots (a
large third-party GUI application) is the operator's call, not something
done automatically. Once installed, `--dry-run` (below) will confirm it
and `--run` will attempt the real sensor/actuator checks.

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

## Remaining limitations

- Webots itself was not installed this session on this exact hardware -
  the central hardware-acceptance question (does Iris Xe run Webots
  reliably) is **unanswered**, not answered favorably.
- No OpenGL version, frame-rate, CPU/memory-under-load, or rendering-
  stability data exists yet for this machine.
- No propeller motion, vehicle lift, or landing was observed - none was
  attempted; the vehicle was never armed in this phase.
- The Arrangement B networking path is documented and safety-checked but
  not live-exercised, since there was nothing running to connect to.
- `--run`'s actuator-flow check is evidence, not a full per-channel PWM
  decode - documented above as an honest scope limit, not a gap to paper
  over.

**No arm/takeoff/land command was sent in this phase.**
