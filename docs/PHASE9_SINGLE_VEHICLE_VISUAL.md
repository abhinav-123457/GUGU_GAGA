# Phase 9: One-Vehicle Real ArduPilot SITL Visual Integration

Phase 8 (`docs/PHASE8_ARDUPILOT_SITL.md`) proved this project's own
`ArduPilotSITLTransport` can attach to, communicate with, and cleanly
detach from a real, independently-started ArduCopter SITL process
(ATTACH mode). Phase 9 builds one visual, one-vehicle integration on top
of that already-accepted result: connect the existing swarm software to
one real ArduCopter SITL vehicle and make its real MAVLink telemetry
visible in Mission Planner or MAVProxy.

**This phase is ATTACH mode only.** SPAWN mode (this project launching its
own ArduCopter process) is a separate, still-open limitation - see Phase
8's docs - and is explicitly out of scope here.

## What this is, and what it is not

- **Real ArduPilot SITL** is a simulated autopilot running a simulated
  vehicle. It is **not a real drone**, and nothing in this phase connects
  to a Pixhawk, a serial port, a radio link, or any non-loopback address.
- **Mission Planner / MAVProxy** provide the visual telemetry/map/HUD view
  in this phase. They are separate programs the operator runs, connecting
  independently to the same localhost MAVLink endpoint.
- `scripts/run_phase9_single_vehicle_visual.py` itself **prints a text
  status line, it does not render any graphics** - it is not a PyBullet 3D
  simulation, and it never claims to be one.
- The vehicle **stays disarmed** for the entirety of this phase. Nothing
  here arms, takes off, or produces motor/PWM output. A disarmed,
  stationary vehicle is the *expected, correct* result, not a failure.
- This phase controls **exactly one** drone. It does not run, and makes no
  claims about, the full six-drone search-and-rescue swarm.

## Command syntax by platform (read this first if you got a PowerShell error)

> **Do not paste Bash's trailing `\` into PowerShell.** PowerShell does not
> treat `\` as a line-continuation character - it errors. Likewise, never
> copy an accidental `-->` prompt-arrow prefix from a terminal recording
> into a real command; it is not part of the command.
>
> - **PowerShell**: put the whole command on one line, or continue lines
>   with a backtick (`` ` ``).
> - **Windows CMD**: continue lines with a caret (`^`).
> - **WSL / Linux Bash**: continue lines with a backslash (`\`).

### Which terminal am I using?

| Terminal | Line continuation |
|---|---|
| Windows PowerShell (`powershell.exe`, `pwsh`) | backtick `` ` `` (or just use one line) |
| Windows CMD (`cmd.exe`) | caret `^` |
| WSL / Linux Bash | backslash `\` |

### PowerShell - single line (simplest, always works)

```powershell
python scripts/run_phase9_single_vehicle_visual.py --attach --connection tcp:127.0.0.1:5760 --system-id 1 --component-id 1 --namespace sitl/drone0 --duration 120
```

### PowerShell - multiline (backtick continuation)

```powershell
python scripts/run_phase9_single_vehicle_visual.py `
  --attach `
  --connection tcp:127.0.0.1:5760 `
  --system-id 1 `
  --component-id 1 `
  --namespace sitl/drone0 `
  --duration 120
```

### WSL / Linux Bash - multiline (backslash continuation)

```bash
python scripts/run_phase9_single_vehicle_visual.py \
  --attach \
  --connection tcp:127.0.0.1:5760 \
  --system-id 1 \
  --component-id 1 \
  --namespace sitl/drone0 \
  --duration 120
```

### Windows CMD - multiline (caret continuation)

```cmd
python scripts/run_phase9_single_vehicle_visual.py ^
  --attach ^
  --connection tcp:127.0.0.1:5760 ^
  --system-id 1 ^
  --component-id 1 ^
  --namespace sitl/drone0 ^
  --duration 120
```

### Windows setup (PowerShell)

```powershell
cd C:\path\to\GUGU_GAGA
.\.venv\Scripts\Activate.ps1
```

If PowerShell's execution policy blocks activation (an error like
`... cannot be loaded because running scripts is disabled on this
system`), use a **session-only** bypass - this does not change your
machine-wide execution policy:

```powershell
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
```

### WSL / Linux setup

```bash
cd /path/to/GUGU_GAGA
source .venv/bin/activate
```

Every other command example in this document (Steps 2/3/5/6 below) is
written in Bash/WSL syntax - use the PowerShell/CMD equivalents above if
you are on Windows outside WSL.

## Architecture

```
    real ArduCopter SITL
        |  MAVLink over tcp:127.0.0.1:5760
    ArduPilotSITLTransport   (swarm_sim/sitl/ardupilot_transport.py, Phase 8, unmodified)
        |
    SITLAdapter              (swarm_sim/autopilot/sitl.py, unmodified)
        |
    validated contracts.Command
        |
    SafetySupervisor         (swarm_sim/safety_supervisor.py, unmodified - final command authority)
        |
    one-drone demo planner / hold-command test / telemetry read
```

`SafetySupervisor` remains the final command authority: in `planner_preview`
mode, every candidate command is evaluated by the real `SafetySupervisor`
before anything reaches the adapter/transport - see
`tests/test_phase9_single_vehicle_visual_architecture.py`'s
`test_planner_preview_calls_evaluate_before_send_command`.

`autopilot_path == "ardupilot_sitl"` (`swarm_sim/config.py`, Phase 8)
already requires explicit localhost connection configuration
(`ardupilot_sitl_connection_strings`/`ardupilot_sitl_system_ids` must be
set, or `FloodSearchMission.__init__` raises `ValueError` immediately) -
Phase 9 does not add a second configuration mechanism on top of it. This
script deliberately does **not** construct a `MissionConfig`/
`FloodSearchMission` at all: doing so would pull in PyBullet and the full
six-drone pipeline, both explicitly out of scope this phase.

## Step 1: start ArduCopter SITL manually (operator)

In a WSL terminal:

```bash
cd /home/swarmbuild/ardupilot

./build/sitl/bin/arducopter \
    --model quad \
    --speedup 1 \
    -I0 \
    --home -35.363261,149.165230,584,353 \
    --sysid 1
```

This binds `SERIAL0` on TCP port **5760** (instance `-I0`'s default),
`system_id=1`. Leave this terminal open - the project attaches to this
already-running process and never starts or stops it.

## Step 2: dry-run (always safe, never opens a socket)

```bash
python scripts/run_phase9_single_vehicle_visual.py --dry-run \
    --connection tcp:127.0.0.1:5760 --system-id 1 --component-id 1 \
    --namespace sitl/drone0
```

This is also the **default** if `--attach` is omitted entirely - the
script never spawns a process and never opens a socket unless `--attach`
is explicitly passed.

## Step 3: attach and visualize telemetry

```bash
python scripts/run_phase9_single_vehicle_visual.py --attach \
    --connection tcp:127.0.0.1:5760 --system-id 1 --component-id 1 \
    --namespace sitl/drone0 --duration 120
```

Prints a continuously updated compact status line (elapsed time, vehicle
id/namespace, system/component id, ENU position/altitude, velocity,
roll/pitch/yaw, battery, armed state, flight mode, heartbeat, estimator
state, last command acknowledgement) once per second for `--duration`
seconds, or until `Ctrl+C` (stops cleanly either way - the transport is
always `.stop()`-ed, which only closes this project's own socket/adapter
state, never the SITL process itself).

## Step 4: connect Mission Planner or MAVProxy (operator, separate program)

ArduCopter SITL's TCP listener on port 5760 accepts multiple simultaneous
MAVLink clients - this project's own script and a GCS can both be attached
at once, each as an independent client.

**MAVProxy** (works without any GUI/X server):

```bash
mavproxy.py --master=tcp:127.0.0.1:5760 --console --map
```

**Mission Planner** (does not need to be installed inside WSL - if it is
on the Windows host, since WSL's `localhost` forwards to Windows for a
`tcp:` listener like this one): connect to `TCP`, host `127.0.0.1`, port
`5760`.

If Mission Planner is not available, **MAVProxy's `--console --map` is an
explicit, accepted alternative** for this phase.

**What telemetry should appear**: heartbeat/link-up indication, attitude
(HUD artificial horizon), local position, battery percentage, flight mode,
disarmed status. Mission Planner/MAVProxy request their own
`GLOBAL_POSITION_INT` stream independently when they connect (ArduPilot's
`MAV_CMD_SET_MESSAGE_INTERVAL` is scoped per connected MAVLink channel),
so the map view is driven by their own connection, not by this project's
script - see "Known limitation: lat/lon" below.

## Step 5: safe command demonstrations

HOLD command + a safe rejected-navigation-command check, never arming:

```bash
python scripts/run_phase9_single_vehicle_visual.py --attach --hold-test \
    --connection tcp:127.0.0.1:5760 --system-id 1 --component-id 1 \
    --namespace sitl/drone0
```

Sends one `HOLD` command (a mode-change request, never a setpoint) and
records its real acknowledgement, then sends one small `VELOCITY_SETPOINT`
while the vehicle is disarmed. This project's transport synthesizes a
local "accepted" ack for any setpoint send (there is no real per-setpoint
MAVLink acknowledgement in the MAVLink protocol itself - see
`docs/PHASE8_ARDUPILOT_SITL.md`), so the honestly-reported signal that
ArduPilot did **not** actually fly is the vehicle's own observed state
afterward: `armed` stays `False` and position does not change. This is
recorded as `"correct safe result"` - the script never arms the vehicle to
force the setpoint to visibly "succeed".

## Step 6: supervised one-drone planner preview (optional)

```bash
python scripts/run_phase9_single_vehicle_visual.py --attach --planner-preview \
    --connection tcp:127.0.0.1:5760 --system-id 1 --component-id 1 \
    --namespace sitl/drone0
```

Runs one minimal, deterministic single-drone demo planner (a fixed small
forward-velocity candidate - there is no meaningful flocking/search
behavior to demonstrate with one disarmed, stationary vehicle, so this
does not re-run the full boids/couzin/vicsek swarm behavior stack) through
the real `SafetySupervisor` every tick, and only ever sends
`SafetyDecision.filtered_command` - never the raw candidate - to the
adapter/transport. Every candidate, `SafetyDecision`, `AdapterCommand`,
transport result, and acknowledgement is recorded in the output JSON. The
vehicle is expected to remain stationary and disarmed throughout - this is
the correct, safe result, not a demonstration of flight.

## Step 7: stop the session

- `Ctrl+C` on this project's own script (any mode) stops that script's own
  socket/loop cleanly - it never touches the ArduCopter SITL process.
- The operator stops ArduCopter SITL itself with `Ctrl+C` in **its own**
  terminal (Step 1) whenever they are done - this project never sends any
  process-termination signal to it (ATTACH mode has no process handle to
  signal in the first place - see the architecture checks below).
- Stop Mission Planner/MAVProxy by disconnecting/closing them, independent
  of this project's own script.

## Explicitly separate modes (never merged)

| Mode | What it demonstrates | What it does not claim |
|---|---|---|
| `dry_run` | Configuration is well-formed | Nothing about a live vehicle |
| `telemetry_visualization` | Real telemetry flows and displays correctly | No commands are sent in this mode |
| `hold_command_test` | HOLD command + safe handling of a rejected/no-effect navigation setpoint | Not a planner or SafetySupervisor demonstration |
| `planner_preview` | One drone's demo planner gated by the real SafetySupervisor | Not the full six-drone swarm; vehicle stays stationary |
| FakeSITL (Phase 7, unaffected) | Deterministic offline SITL, unchanged | Not real ArduPilot |
| PyBullet visual simulation (`run_mission.py --gui`, unaffected) | 3D physics simulation | Not connected to real ArduPilot in this phase |

Each mode's report JSON (`results/phase9_single_vehicle_visual/`) contains
only its own mode's fields - a `dry_run` report never contains telemetry
samples; a `telemetry_visualization` report never contains `ticks`/safety
decisions; see `tests/test_phase9_single_vehicle_visual.py`'s
`test_modes_produce_disjoint_top_level_report_shapes`.

## Known limitation: lat/lon

This project's own transport (`ArduPilotSITLTransport`) only requests
`LOCAL_POSITION_NED`/`ATTITUDE`/`SYS_STATUS`/`EKF_STATUS_REPORT` (Phase 8,
unmodified) - it does not request `GLOBAL_POSITION_INT`, so this script's
own status line/report honestly prints `lat/lon=n/a`. This does **not**
block the Mission Planner/MAVProxy map view: those programs request their
own `GLOBAL_POSITION_INT` stream on their own independent MAVLink
connection to the same port, since ArduPilot's `MAV_CMD_SET_MESSAGE_INTERVAL`
is scoped per connected channel, not shared across every client. Adding a
`GLOBAL_POSITION_INT` request to this project's own transport was
deliberately left out of this phase's scope (Phase 8's `ardupilot_transport.py`
is otherwise unmodified this phase).

## Architecture checks (`tests/test_phase9_single_vehicle_visual_architecture.py`)

AST-level guarantees, mirroring Phase 8's own established pattern:

- No `subprocess` import anywhere in the script, and no
  `ArduPilotVehicleEndpoint(...)` call site ever passes
  `executable_path`/`wsl_distro`/`working_directory` - **this phase cannot
  spawn a process, structurally, not just by convention**.
- No process-lifecycle identifiers (`Popen`, `terminate`, `kill`,
  `_SITLProcessHandle`, ...) referenced anywhere - **this phase cannot kill
  a process either**.
- No motor/PWM/arm/disarm/takeoff identifiers referenced anywhere.
- No hardware-specific imports (`serial`, `RPi`, `dronekit`, ...).
- No hardcoded non-loopback IP literal; the default `--connection` is
  loopback.
- `run_planner_preview` calls `SafetySupervisor.evaluate()` strictly
  before `send_command()`, and the `AdapterCommand` actually sent is built
  from `decision.filtered_command`, never the raw candidate.
- No reference to `distributed_consensus`/`DistributedConsensus`.
- No reference to `pybullet`/`FloodSearchMission` - this script never
  constructs the full mission pipeline.
- `FakeSITLTransport` remains importable, selectable, and functional
  (Phase 7 regression check).

## Files changed this phase

- `scripts/run_phase9_single_vehicle_visual.py` (new) - the four-mode
  ATTACH-only CLI described above.
- `tests/test_phase9_single_vehicle_visual.py` (new) - functional tests
  against `tests/_dummy_ardupilot_vehicle.DummyArduPilotVehicle` (same
  real-MAVLink-over-localhost fixture Phase 8's own tests use).
- `tests/test_phase9_single_vehicle_visual_architecture.py` (new) - AST
  architecture checks.
- `docs/PHASE9_SINGLE_VEHICLE_VISUAL.md` (new, this file).

**Unmodified this phase**: `swarm_sim/sitl/ardupilot_transport.py`,
`swarm_sim/sitl/fake_transport.py`, `swarm_sim/autopilot/*`,
`swarm_sim/safety_supervisor.py`, `swarm_sim/contracts.py`,
`swarm_sim/mission.py`, `scripts/run_phase8_ardupilot_sitl.py`, and every
existing test file.

## Explicitly out of scope this phase

- SPAWN mode (still an open, documented limitation - see
  `docs/PHASE8_ARDUPILOT_SITL.md`).
- Connecting a real Pixhawk, serial port, or radio.
- Six-vehicle integration.
- Connecting the PyBullet physics vehicle to real ArduPilot.
- Hardware-in-the-loop.
- Arming or takeoff, automatic or otherwise.

## Required statement

> Real ArduPilot SITL was attached over localhost. No Pixhawk, serial
> port, radio, real vehicle, arming, takeoff, motor output, or
> hardware-in-the-loop was used.
