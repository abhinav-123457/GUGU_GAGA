# Phase 11 Live Verification Report

This is a **verification report**, not a new implementation or a new
flight test - it records what was observed during the live FlightGear
session run per `docs/PHASE11_LIVE_VALIDATION_PROCEDURE.md`. No code was
changed to produce it, and **no arm, takeoff, land, mode, or
parameter-write command was sent** during this verification.

## 1. Which path produced the view

**Path B - this project's telemetry-renderer prototype**
(`ArduPilot SITL -> MAVLink -> scripts/run_phase11_flightgear_viewer.py
--flightgear-view -> FGNetFDM/UDP -> FlightGear`), confirmed by:

- The running `arducopter` process's exact command line (captured live via
  `pgrep -af`) does **not** include `--enable-fgview`/`--fg` - ArduPilot's
  native path (Path A) was never enabled.
- `scripts/run_phase11_flightgear_viewer.py --flightgear-view` was running
  (pid confirmed live) and its own JSON result file
  (`results/phase11_flightgear/phase11_flightgear_view_result.json`) shows
  `"fg_sent_count": 2995`, `"fg_error_count": 0`, `"shutdown": {"clean":
  true}` - it was the only sender to port 5503.

This was **not** offline replay: the run's `"mode"` is
`"flightgear_view"`, not `"replay"`; `--replay` never opens a socket at
all (structurally verified), and this session's CSV timestamps advance in
real wall-clock seconds matching `--duration 300`.

## 2. Exact commands used (captured live from running processes)

**ArduPilot SITL:**
```bash
cd /home/swarmbuild/ardupilot
./build/sitl/bin/arducopter \
    --model quad --speedup 1 -I0 \
    --home -35.363261,149.165230,584,353 \
    --sysid 1 \
    --defaults Tools/autotest/default_params/copter.parm,/mnt/c/Users/abhin/OneDrive/Desktop/swarm/docs/phase10_sitl_geofence.parm
```

**FlightGear** (via the `dbus-run-session` wrapper needed in this WSL2
environment - see below):
```bash
cd /home/swarmbuild/ardupilot
dbus-run-session -- ./Tools/autotest/fg_quad_view.sh
```
which itself launches (captured live from the running `fgfs` process):
```
fgfs --native-fdm=socket,in,10,,5503,udp --fdm=external --aircraft=arducopter \
    --fg-aircraft=./Tools/autotest/aircraft --airport=KSFO --geometry=650x550 --bpp=32 \
    --disable-hud-3d --disable-horizon-effect --timeofday=noon --disable-sound \
    --disable-fullscreen --disable-random-objects --disable-ai-models --fog-disable \
    --disable-specular-highlight --disable-anti-alias-hud --wind=0@0
```

**Project bridge (Path B):**
```bash
cd /mnt/c/Users/abhin/OneDrive/Desktop/swarm
python3 scripts/run_phase11_flightgear_viewer.py --flightgear-view \
    --connection tcp:127.0.0.1:5760 --system-id 1 --component-id 1 \
    --namespace sitl/drone0 --flightgear-endpoint 127.0.0.1:5503 \
    --duration 300 --update-rate-hz 10
```

**Note on `dbus-run-session`**: not part of the original procedure doc -
added because `fgfs` aborted on first launch in this WSL2 distro
(`dbus_connection_set_exit_on_disconnect(): assertion "connection !=
NULL" failed`). Root cause, confirmed live: `DBUS_SESSION_BUS_ADDRESS`
pointed at `/run/user/1000/bus`, but no per-user D-Bus session bus was
ever started for this WSL login (only the system bus was running).
`dbus-run-session` gives `fgfs`'s process tree a private, working session
bus with no system changes. This does not touch ArduPilot, the bridge, or
any project file.

## 3. FlightGear version, aircraft model, protocol, port, update rate

| Property | Verified value |
|---|---|
| FlightGear version | `2020.3.6` (package `flightgear` `1:2020.3.6+dfsg-1build1`) |
| Aircraft model | `arducopter` (`Tools/autotest/aircraft/arducopter-set.xml`) |
| Protocol | native FDM / FGNetFDM, version `24` (`0x18`) |
| Direction | UDP, FlightGear is the `in` (receiving) side |
| Port | `5503` |
| Update rate | `10 Hz` (both `fgfs`'s `--native-fdm=socket,in,10,,5503,udp` and the bridge's `--update-rate-hz 10`) |

## 4. Live telemetry vs. replay

**Live telemetry**, confirmed: `report["mode"] == "flightgear_view"`
(never `"replay"`), the run required and used a real, already-running
ArduCopter SITL MAVLink connection (`"attach": {"succeeded": true}`,
`"heartbeat": {"received": true}`), and every one of its 2995 samples
carries a distinct, monotonically increasing wall-clock
`timestamp_s` spanning the full 300 s duration - a `--replay` run does
not attach to SITL at all and never touches the FlightGear socket.

## 5. Coordinate mapping verification (against this run's actual data)

Using the run's final sample (`timestamp_s=6352.637165497`):

| Field | Value | Check |
|---|---|---|
| `lat_deg`, `lon_deg` | `-35.3632609`, `149.1652297` | Matches configured `--home -35.363261,149.165230` to within EKF/GPS noise (~10 cm) - position mapping correct. |
| `east_m`, `north_m` | `-0.024`, `0.012` | Near zero, as expected for a vehicle stationary at its own EKF origin/home. |
| `altitude_m` (absolute) | `584.09` (constant, min=max, across all 2995 samples) | Matches home altitude `584` m to within 9 cm - altitude sign/scale correct (this is `GLOBAL_POSITION_INT.alt`, AMSL, passed through unmodified into FGNetFDM's `altitude` field). |
| `roll_rad`, `pitch_rad` | `-0.00083`, `-0.00101` rad (≈ `-0.05°`, `-0.06°`) | Matches your on-screen reading of "roll ≈ 0, pitch ≈ 0" exactly - passthrough confirmed (FGNetFDM's `phi`/`theta` are a direct, unmodified copy of ArduPilot's own Euler angles per source). |
| `yaw_enu_rad` | `1.689282` rad | Converting back to ArduPilot's native NED yaw via this project's own involution (`_yaw_ned_to_enu`, applied a second time to undo itself): `NED yaw = π/2 − 1.689282 = −0.118485 rad = −6.79°`. **This matches your reported FlightGear heading of "≈ −6.7" almost exactly**, and is consistent with the configured home heading of `353°` (`353 − 360 = −7°`). This is a real, independent, live cross-check that the yaw/heading axis mapping is correct end-to-end, not just unit-tested in isolation. |

## 6. FlightGear altitude reading vs. Phase 10 MAVLink telemetry

**The authoritative source is the MAVLink telemetry actually sent to
FlightGear, not a visual read of FlightGear's own altimeter dial**, and it
shows something more precise than the "≈584.8" estimate:

- Across all **2995 of 2995** samples sent in this run, `altitude_m` was
  **constant at exactly `584.09` m** (`min_altitude_m == max_altitude_m
  == 584.09`) - it never changed for the full 300 seconds.
- Home altitude is `584.0` m.
- **Relative altitude above home, per telemetry: `584.09 − 584.0 = 0.09
  m`** (≈ 9 cm) - consistent with a vehicle sitting on the ground,
  disarmed, the entire time.

Your visual estimate of "≈584.8" on FlightGear's own gauge is about 0.7 m
higher than what this run's telemetry ever contained. Since the data
feeding FlightGear never moved from `584.09` for the entire run, this gap
is most consistent with **needle/gauge reading imprecision on a small
(650×550) analog altimeter display**, not a discrepancy in the data
pipeline itself - the CSV is the ground truth for what was actually sent.

**Per your own instruction, this must be reported honestly from
telemetry: relative altitude in this run was ≈0.09 m AGL. It was not
0.8 m, and it was absolutely not the Phase 10 flight's `1.720716` m peak
- that number belongs to a separate, earlier, real arm/takeoff test and
was never replayed or reproduced live in this session.**

## 7. Status display fields (source of each)

This run's CSV (`results/phase11_flightgear/telemetry.csv`) and the
independent read-only check below together cover every field requested:

| Field | Value (this run) | Source |
|---|---|---|
| Absolute altitude | `584.09` m (constant) | `GLOBAL_POSITION_INT.alt` via the bridge's CSV |
| Relative altitude above home | `0.09` m (constant) | computed: CSV `altitude_m` − configured home altitude (584.0 m) |
| Armed/disarmed | **disarmed** (`armed: false`) | fresh `--attach` read (below) and independent `HEARTBEAT.base_mode` check |
| Flight mode | `LAND` (left over from Phase 10's earlier completed flight test; vehicle has not been re-armed since) | fresh `--attach` read (below) |
| Telemetry source | live MAVLink, this session | `--flightgear-view`'s own `"mode": "flightgear_view"` result |
| Live vs. replay | **live** | see item 4 above |

No field in `_CSV_FIELDS` was added or changed to produce this table -
these are read directly from already-logged columns plus one independent
read-only check, not a new capability added to the script.

## 8. Motor outputs and command safety (independently verified live)

A **separate, one-off, read-only** MAVLink check (not part of the
project's own code - a throwaway verification script, deleted after use)
connected to the same already-running SITL and read `HEARTBEAT` and
`SERVO_OUTPUT_RAW` directly:

```
armed_from_heartbeat_base_mode: False
servo1_raw 1000
servo2_raw 1000
servo3_raw 1000
servo4_raw 1000
```

`1000` is ArduPilot's motor-off/idle PWM value (its output range is
`1000`-`2000`) - **all four motor outputs are at the disarmed floor**,
confirming FlightGear's "motor outputs 0" display reflects a genuinely
disarmed vehicle, not just this project's own (separate, unrelated)
choice to always send zero into FGNetFDM's unused engine fields.

**No arm/takeoff/land/mode/setpoint/parameter-write command was sent by
the bridge** during this or any other run this phase - this is both
structurally proven (`tests/test_phase11_flightgear_viewer_architecture.py`:
`test_only_command_sent_is_set_message_interval`,
`test_never_calls_param_set_anywhere`, `test_no_flight_command_identifiers_anywhere_in_script`,
`test_never_forces_arm_with_the_force_magic_value`) and behaviorally
proven at the wire level
(`tests/test_phase11_flightgear_viewer.py::test_bridge_never_sends_arm_takeoff_land_or_mode_command`
and the "only MAV_CMD_SET_MESSAGE_INTERVAL ever sent" test) - unchanged
from Phase 11A, since no code was modified for this verification.

## 9. Live verification snapshot

| Field | Value |
|---|---|
| Source path | B (project telemetry-renderer prototype) |
| FlightGear endpoint | `127.0.0.1:5503` (UDP) |
| Protocol | native FDM / FGNetFDM v24 |
| Position (lat, lon) | `-35.3632609, 149.1652297` |
| Position (east, north, up, m) | `-0.024, 0.012, 0.0053` |
| Absolute altitude | `584.09` m |
| Relative altitude above home | `0.09` m |
| Attitude (roll, pitch, rad) | `-0.00083, -0.00101` |
| Heading (NED yaw, computed) | `-6.79°` (device-observed ≈ `-6.7`) |
| Armed state | `false` |
| Motor state | all 4 outputs at `1000` (disarmed floor) |
| Sample count (sent) | `2995` |
| Dropped samples | `0` |
| Maximum observed relative altitude (this run) | `0.09` m |
| Duration | `300` s at `10` Hz |
| Timestamp of this report | see `results/phase11_flightgear/phase11_live_verification_report.json` |

A machine-readable copy of this same snapshot is saved at
`results/phase11_flightgear/phase11_live_verification_report.json`.

## 10. Tests and compile checks

- `python -m py_compile` on `scripts/run_phase11_flightgear_viewer.py`,
  `swarm_sim/visualization/flightgear_bridge.py`,
  `swarm_sim/visualization/telemetry_mapping.py`,
  `scripts/run_phase10_sitl_flight_test.py`: **clean, no errors.**
- Full existing test suite: see the accompanying report message for the
  pass count - **no flight-control code, gate, or safety logic was
  modified to produce this verification**, so no regression is expected.

## Honest classification of what this session's screenshot shows

**This is live Path B rendering**: a real, currently-disarmed ArduCopter
SITL vehicle's real MAVLink telemetry, converted through this project's
tested conversion pipeline, sent over real UDP FGNetFDM packets, rendered
by a real FlightGear process. It is **not** ArduPilot's native
FlightGear view (Path A was never enabled this run), and it is **not**
offline replay (no `--replay` run touched the FlightGear socket in this
session). It shows the vehicle sitting still on the ground, disarmed, at
its home position - it does not show, replay, or reproduce Phase 10's
separate `1.720716` m flight.
