# Phase 11 Live Operator-Validation Procedure

This document is a **runbook**, not a new implementation - it does not
change `scripts/run_phase11_flightgear_viewer.py`,
`swarm_sim/visualization/*.py`, or any flight-control code, and it does
not perform an arm/takeoff/land test. Every command below was checked
against the actual code in this repository and, where noted, **already run
live in this session** against a real ArduCopter SITL instance - nothing
here is guessed.

**Safety, restated and unchanged from `docs/PHASE11_FLIGHTGEAR_VIEWER.md`:**
localhost only; one vehicle only; FlightGear is render-only; the bridge
never sends arm/takeoff/land/mode/setpoint commands (the only command it
ever sends is a read-only `MAV_CMD_SET_MESSAGE_INTERVAL` stream-rate
request); no parameter writes; no process spawning/killing by the bridge;
no Pixhawk/serial/radio/hardware; no six-drone swarm; no new flight test.

## Two paths - never run both on the same port at the same time

```
A. ArduPilot native FlightGear view (no MAVLink involved at all):
   ArduCopter SITL --enable-fgview--> native FGNetFDM/UDP --> FlightGear

B. This project's telemetry-renderer prototype (the Phase 11 deliverable):
   ArduCopter SITL --MAVLink--> scripts/run_phase11_flightgear_viewer.py
                                  --FGNetFDM/UDP--> FlightGear
```

Both paths speak the exact same wire protocol to FlightGear on the exact
same default port (`5503`), so **FlightGear's own command line is
identical for both** - the only difference is which process is sending to
that port. Running A and B at the same time, both targeting port 5503,
would mean two senders racing to write the same UDP port; FlightGear would
render whichever packet arrived last, from either source, with no
indication of which one it was. If you want to compare them side by side,
give Path B an explicit different port (`--flightgear-endpoint
127.0.0.1:5504`) and start a second `fgfs` listening on `5504` instead -
otherwise, use one path at a time. **Every command below states which
path it belongs to.**

## Verified environment (this session)

All verification below was performed on your existing WSL2 `Ubuntu-22.04`
distribution, where **both ArduPilot and FlightGear are already
installed** (confirmed live, not assumed):

| Component | Verified value |
|---|---|
| ArduPilot checkout | `/home/swarmbuild/ardupilot`, commit `92b0cd788ec29406f26c6f9c31d5ceedbd1cc538` (Copter-4.6.3) - the same commit `docs/PHASE10_SITL_FLIGHT_TEST.md`/`docs/PHASE11_FLIGHTGEAR_VIEWER.md` cite |
| ArduPilot SITL binary | `build/sitl/bin/arducopter` (already built) |
| FlightGear | version `2020.3.6` (`fgfs --version`), package `flightgear` `1:2020.3.6+dfsg-1build1`, binary `/usr/games/fgfs` (on `PATH` by default) |
| Copter FlightGear model | `Tools/autotest/aircraft/arducopter-set.xml` (confirmed present) |
| This project | reachable at `/mnt/c/Users/abhin/OneDrive/Desktop/swarm` from the same distro - no cross-distro filesystem access needed |
| Python | system `python3` already has `pymavlink` installed; `scripts/run_phase11_flightgear_viewer.py` imports cleanly under it (checked live) |

Because ArduPilot, FlightGear, and this project's script are all reachable
from **the same WSL2 distro**, everything below runs there and never
crosses the Windows<->WSL2 boundary except the one clearly-marked optional
Mission Planner step - see "Remaining WSL2-to-Windows networking
limitations" at the end.

## 1. Installing FlightGear on Windows

Not needed for the recommended setup below (FlightGear is already
installed in your WSL2 distro), and not independently tested in this
session. If you want a native-Windows FlightGear anyway: download the
Windows installer from the official page,
[flightgear.org/download](https://www.flightgear.org/download/), run it,
and launch it from the Start Menu. Whether it can receive UDP packets
from a script running inside WSL2 depends on WSL2's `localhostForwarding`
behavior for UDP, which was **not** verified live in this session - see
the limitations section.

## 2. Installing FlightGear on Ubuntu/WSL

Already done in your `Ubuntu-22.04` distro (confirmed above). For
reference, the command that installs it is:

```bash
sudo apt update && sudo apt install flightgear
```

## 3. Starting ArduCopter SITL

**Already running** in your `Ubuntu-22.04` distro at the time this was
written (verified via `pgrep`/`ss`), using exactly the command documented
in `docs/PHASE10_SITL_FLIGHT_TEST.md`/`docs/PHASE11_FLIGHTGEAR_VIEWER.md`:

```bash
cd /home/swarmbuild/ardupilot
./build/sitl/bin/arducopter \
    --model quad --speedup 1 -I0 \
    --home -35.363261,149.165230,584,353 \
    --sysid 1 \
    --defaults Tools/autotest/default_params/copter.parm,/mnt/c/Users/abhin/OneDrive/Desktop/swarm/docs/phase10_sitl_geofence.parm
```

**No extra flag is needed on this command for Path B** (this project's
bridge talks over MAVLink, already open on `tcp:127.0.0.1:5760` - it does
not need anything from ArduPilot beyond what's already running).

**For Path A only**, add `--enable-fgview --fg 127.0.0.1` to the command
above (or pass them via `sim_vehicle.py -A "--enable-fgview --fg 127.0.0.1"`
if using the official helper). If SITL is already running (as it is right
now) and you want to test Path A, stop it (`Ctrl+C` in its terminal - this
never affects FlightGear or this project's bridge) and relaunch with the
extra flags, since `--enable-fgview` is only read at startup.

## 4. Starting FlightGear manually

Exact, verified values - not guessed:

| Property | Value | Verified how |
|---|---|---|
| Aircraft/model | `arducopter` | `Tools/autotest/aircraft/arducopter-set.xml` confirmed present |
| Protocol | native FDM (`--native-fdm`) | `Tools/autotest/fg_quad_view.sh`, confirmed present and executable |
| Protocol version | `24` (`0x18`) | `SIM_JSBSim.h`/`SITL_State.cpp`, cross-checked against public FlightGear documentation |
| Direction | UDP, FlightGear is the `in` side (receives) | `--native-fdm=socket,in,10,,5503,udp` |
| Loopback address | `127.0.0.1` | this phase's own requirement; matches ArduPilot's own default `--fg` address |
| Port | `5503` | `SITL_cmdline.cpp`'s `FG_VIEW_PORT`; confirmed free (`ss -ulnp`) before this session's live test |
| Update rate | `10 Hz` (the `10` in `--native-fdm=socket,in,10,,5503,udp`) | `fg_quad_view.sh`; matches this project's own `--update-rate-hz` default |
| Coordinate-frame assumptions | position: absolute geodetic (WGS84 lat/lon/alt), **not** local NED/ENU; velocity: NED-labeled, feet/sec; attitude: standard aerospace Euler angles, radians | `SIM_JSBSim.h` field comments - see `docs/PHASE11_FLIGHTGEAR_VIEWER.md`'s "Protocol details" |

**Recommended: run the actual verified script** (identical for Path A and
Path B - see above):

```bash
cd /home/swarmbuild/ardupilot
./Tools/autotest/fg_quad_view.sh
```

This is ArduPilot's own file, already executable in your checkout - not a
command reconstructed by hand. It opens a `650x550` FlightGear window
using the `arducopter` model, listening on UDP `5503`, with several
cosmetic flags disabled (sound, fullscreen, random scenery objects) to
keep it light under WSLg. It starts near `KSFO` by default; since
`--fdm=external` means FlightGear discards its own starting position the
moment the first native-fdm packet arrives, this only matters for the
first second or two before a sender (Path A's SITL, or Path B's bridge)
connects.

If you'd rather see the correct starting position immediately (optional).
This is the same script's own flags, written out explicitly with `--lat`/
`--lon`/`--altitude`/`--heading` matching this project's actual SITL home
instead of `--airport=KSFO`:

```bash
cd /home/swarmbuild/ardupilot
nice fgfs \
    --native-fdm=socket,in,10,,5503,udp \
    --fdm=external \
    --aircraft=arducopter \
    --fg-aircraft="Tools/autotest/aircraft" \
    --lat=-35.363261 --lon=149.165230 --altitude=584 --heading=353 \
    --geometry=650x550 --bpp=32 \
    --disable-hud-3d --disable-horizon-effect --timeofday=noon \
    --disable-sound --disable-fullscreen --disable-random-objects \
    --disable-ai-models --fog-disable --disable-specular-highlight \
    --disable-anti-alias-hud --wind=0@0
```

## 5. Starting this project's read-only bridge (Path B)

**Already verified live in this session** (see "What was already verified
live" below) - this exact command, with `--duration` extended so you have
time to actually look at the FlightGear window:

```bash
cd /mnt/c/Users/abhin/OneDrive/Desktop/swarm
python3 scripts/run_phase11_flightgear_viewer.py --flightgear-view \
    --connection tcp:127.0.0.1:5760 --system-id 1 --component-id 1 \
    --namespace sitl/drone0 --flightgear-endpoint 127.0.0.1:5503 \
    --duration 300 --update-rate-hz 10
```

Run this from a **third** terminal (SITL and FlightGear each need their
own). It attaches to the already-running SITL (step 3), reads telemetry
read-only, and streams renderer frames to FlightGear (step 4) for 5
minutes, printing a JSON report and writing
`results/phase11_flightgear/telemetry.csv` when it finishes (or when you
stop it - see step 8).

## 6. Connecting Mission Planner independently (optional)

Not required for either path - purely an extra, independent view of the
same real vehicle, exactly as already proven working in Phase 9's own
session (`docs/PHASE9_SINGLE_VEHICLE_VISUAL.md`). From Windows, connect
Mission Planner to `tcp:127.0.0.1:5760` (the same port SITL already has
open - ArduPilot accepts multiple simultaneous MAVLink clients, which is
exactly how this already works). This is the one step in this whole
procedure that crosses the Windows<->WSL2 boundary - see the limitations
section.

## 7. Running the offline replay of the recorded 1.720716 m flight

**Already verified in this session** (see
`docs/PHASE11_FLIGHTGEAR_VIEWER.md`'s "Offline demonstration" section) -
this needs neither SITL nor FlightGear running:

```bash
cd /mnt/c/Users/abhin/OneDrive/Desktop/swarm  # or the Windows path directly
python3 scripts/run_phase11_flightgear_viewer.py --replay results/phase11_flightgear/phase10_reused_demo.csv
```

Expected output (already observed):

```json
{
  "mode": "replay",
  "row_count": 3,
  "sent_row_count": 3,
  "dropped_row_count": 0,
  "min_altitude_m": 584.0,
  "max_altitude_m": 585.720716,
  "remaining_failures": []
}
```

`585.720716 - 584.0 = 1.720716` - Phase 10's own recorded peak altitude,
reprocessed correctly. **`--replay` never opens a socket and, in this
implementation, never sends anything to FlightGear** - it only produces
this JSON summary. It cannot currently drive a live FlightGear render by
itself (see "How to distinguish live telemetry from replay" below).

## 8. Stopping FlightGear and the bridge safely

- **The bridge** (`--flightgear-view`, step 5): `Ctrl+C` in its terminal.
  It catches this cleanly, closes its UDP socket, and calls
  `transport.stop()` - which closes only *this script's own* MAVLink
  connection. It never sends anything to ArduPilot as part of shutting
  down, and it never touches the FlightGear process.
- **FlightGear**: close its window normally (or `Ctrl+C` in the terminal
  that launched it, for either Path A's `fg_quad_view.sh` or Path B's
  manual `fgfs` command). This never affects ArduPilot SITL.
- **ArduPilot SITL**: unaffected by either of the above. Stop it yourself,
  separately, with `Ctrl+C` in *its own* terminal, only when you're done -
  neither this project's bridge nor FlightGear ever terminates it (see
  the architecture tests in `tests/test_phase11_flightgear_viewer_architecture.py`
  for the structural proof that this project's code contains no process-
  killing call at all).

## What was already verified live in this session (not guessed, not simulated)

With ArduCopter SITL already running (disarmed the entire time - no
arm/takeoff/land command was sent at any point), the following were run
for real against it:

1. `--dry-run --flightgear-endpoint 127.0.0.1:5503` -> `"ok": true`.
2. `--diagnose --flightgear-endpoint 127.0.0.1:5503` -> reported the same
   documented protocol facts as `docs/PHASE11_FLIGHTGEAR_VIEWER.md`.
3. `--attach --duration 5` -> real telemetry: `heartbeat_ok: true`,
   `estimator_valid: true`, `armed: false`, real position/attitude/battery
   values.
4. `--flightgear-view` for 5 seconds at 10 Hz, with a plain Python UDP
   socket bound to `127.0.0.1:5503` standing in for FlightGear (so no
   FlightGear window was needed for this specific check): **50/50 samples
   sent, 0 dropped**, every received datagram exactly `408` bytes with
   `version == 24`, and the logged CSV's `lat_deg`/`lon_deg`/`altitude_m`
   (`-35.3632611`, `149.1652299`, `584.09`) matched the vehicle's actual
   configured home position, with `roll_rad`/`pitch_rad` near zero (level,
   disarmed) as expected.

**What this does *not* verify**: that a real FlightGear window correctly
renders those bytes. Claude cannot see or interact with a GUI window -
that visual confirmation is the one remaining step, which is why you
offered to run FlightGear yourself. Steps 4-5 above, run together with
FlightGear actually listening on port 5503 instead of the stand-in socket,
is that remaining step.

## How to verify position, attitude, yaw, and altitude

Once Path A or B is running with a real FlightGear window open:

1. **In FlightGear itself**: its 2D/3D panel and map both show the
   aircraft's current position; the artificial horizon shows roll/pitch;
   the compass/heading indicator shows yaw. While the vehicle sits
   disarmed on the ground (no new flight is being run), these should stay
   essentially constant - position near `-35.363261, 149.165230`, altitude
   near `584 m`, roll/pitch near `0`.
2. **Cross-check against this project's own log**: `results/phase11_flightgear/telemetry.csv`
   (Path B only - Path A doesn't produce one, since it never goes through
   this project's code at all) records the exact same fields
   (`lat_deg`/`lon_deg`/`altitude_m`/`agl_m`/`roll_rad`/`pitch_rad`/`yaw_enu_rad`)
   for every sample sent. If FlightGear's display matches a row in this
   CSV for the same rough time, the whole pipeline (MAVLink -> conversion
   -> FGNetFDM -> FlightGear) is confirmed correct end-to-end - not just
   "bytes were sent," but "bytes were sent *and rendered correctly*."
3. **Cross-check against Mission Planner** (if connected, step 6) as a
   third, fully independent view of the same real ArduPilot state - if
   Mission Planner, this project's CSV, and FlightGear's display all agree,
   that's the strongest possible confirmation available without
   instrumenting FlightGear's own internals.

## How to distinguish live telemetry from replay

| | `--flightgear-view` (live) | `--replay` (offline) |
|---|---|---|
| `report["mode"]` | `"flightgear_view"` | `"replay"` |
| Requires SITL running? | Yes | No |
| Requires FlightGear running? | No (sends regardless; only useful with one listening) | No - **never sends to FlightGear at all** |
| Opens any socket? | Yes (MAVLink + UDP) | No |
| Timing | Real-time, paced to `--update-rate-hz` over wall-clock `--duration` | Processes every row as fast as Python can loop - not paced to any original timing |
| Can drive a live FlightGear render? | Yes | **No, in this implementation** - it only produces a JSON summary report |

If you want to *watch* the recorded `1.720716 m` figure in FlightGear
rather than just see it in a JSON report, that would require replay mode
to also send to FlightGear (paced to realistic timing) - which is not
what `--replay` does today; extending it would be a new, separate change,
not something this procedure document performs.

## Remaining WSL2-to-Windows networking limitations

**For Path A and Path B as documented above (everything inside the same
WSL2 `Ubuntu-22.04` distro), there is no Windows<->WSL2 networking
involved at all** - ArduPilot, FlightGear, and this project's bridge all
share one network namespace. The FlightGear *window* appears on your
Windows desktop via WSLg (a display-forwarding mechanism, not a network
port), which is already confirmed working (the Start Menu entry you
showed).

The **one exception** is step 6 (Mission Planner, optional): that's a
Windows-native application connecting to WSL2's `tcp:127.0.0.1:5760`,
which relies on WSL2's `localhostForwarding` (enabled by default) for
TCP. This exact path was already proven working in Phase 9's own session.
It is also the one path this project's documentation has previously
recorded as having intermittently regressed in this exact environment
(`docs/PHASE10_SITL_FLIGHT_TEST.md`'s "environment-level finding" section,
from an earlier session) before apparently resolving itself - worth
knowing about if Mission Planner ever fails to connect, though it is not
expected to affect Path A or B themselves.

If you ever do want a **native-Windows FlightGear** (item 1) talking to
WSL2's ArduPilot/bridge instead of the all-in-WSL2 setup above, that would
cross the Windows<->WSL2 boundary for the UDP FlightGear traffic too, and
was **not tested in this session** - the all-in-WSL2 setup above avoids
that question entirely, which is why it's the recommended path.
