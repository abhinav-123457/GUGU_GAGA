# Phase 11A: Optional FlightGear Visual-Rendering Integration

Strictly simulator-only. ATTACH mode only - this phase never spawns or
kills a process (ArduPilot or FlightGear), same invariant as Phase 9/10.
Never connects to anything but `127.0.0.1`/`localhost`, for **both** the
ArduPilot MAVLink endpoint and the FlightGear UDP endpoint. No Pixhawk,
serial port, radio, real aircraft, outdoor vehicle, motor, PWM output, or
hardware-in-the-loop was used anywhere in this phase.

**Exact ArduPilot version**: `Copter-4.6.3`, commit
`92b0cd788ec29406f26c6f9c31d5ceedbd1cc538` (unchanged from Phase 8/9/10 -
the same build, same checkout).

## Required investigation

### 1-2. ArduPilot Copter-4.6.3 FlightGear support status / native output

**Contrary to the initial concern that ArduPilot's FlightGear documentation
is Plane/HIL-focused, ArduPilot's SITL has a native, vehicle-agnostic,
read-only FlightGear *rendering* output, and it explicitly supports
Copter.** Source-verified at the exact commit this project's SITL binary
was built from:

- `libraries/AP_HAL_SITL/SITL_State_common.h:234-235`:
  ```
  // output socket for flightgear viewing
  SocketAPM_native fg_socket{true};
  ```
  (`{true}` means "datagram" - UDP, confirmed against
  `AP_RCProtocol_UDP.h`'s identical constructor comment.)
- `libraries/AP_HAL_SITL/SITL_State.cpp` - `_sitl_setup()`:
  ```cpp
  if (_use_fg_view) {
      fg_socket.connect(_fg_address, _fg_view_port);
  }
  ```
  and `_output_to_flightgear()` (called from `_fdm_input_local()` on every
  physics step, unconditionally for any vehicle type):
  ```cpp
  void SITL_State::_output_to_flightgear(void)
  {
      SITL::FGNetFDM fdm {};
      const SITL::sitl_fdm &sfdm = _sitl->state;
      fdm.version = 0x18;
      fdm.longitude = DEG_TO_RAD_DOUBLE*sfdm.longitude;
      fdm.latitude = DEG_TO_RAD_DOUBLE*sfdm.latitude;
      fdm.altitude = sfdm.altitude;
      fdm.agl = sfdm.altitude;
      fdm.phi   = radians(sfdm.rollDeg);
      fdm.theta = radians(sfdm.pitchDeg);
      fdm.psi   = radians(sfdm.yawDeg);
      ...
      fdm.ByteSwap();
      fg_socket.send(&fdm, sizeof(fdm));
  }
  ```
  This code lives in the **shared `SITL_State` HAL layer**, not in any
  vehicle-specific file - it is not gated by vehicle type at all, so it is
  exactly as available to ArduCopter as to ArduPlane.
- `libraries/AP_HAL_SITL/SITL_cmdline.cpp` - the exact CLI flags:
  ```
  --fg|-F ADDRESS          set Flight Gear view address, defaults to 127.0.0.1
  --enable-fgview          enable Flight Gear view
  ```
  with `_fg_address = "127.0.0.1"` and `FG_VIEW_PORT = 5503` as compiled-in
  defaults (offset by `+10 * instance` for a non-zero `-I`/`--instance`).
- **Copter-specific, official tooling**: `Tools/autotest/fg_quad_view.sh`
  (verbatim, at this exact commit):
  ```sh
  #!/bin/sh
  AUTOTESTDIR=$(dirname $0)
  nice fgfs \
      --native-fdm=socket,in,10,,5503,udp \
      --fdm=external \
      --aircraft=arducopter \
      --fg-aircraft="$AUTOTESTDIR/aircraft" \
      --airport=KSFO \
      ...
  ```
  and a dedicated, official quadcopter 3D model at
  `Tools/autotest/aircraft/arducopter/` (`plus_quad2-set.xml` plus `.ac`/
  `.dae`/`.skp` model files and a `quad.nas` Nasal script) - this is real,
  shipped, Copter-specific FlightGear support, not a Plane-only feature.

**Conclusion**: native ArduCopter-to-FlightGear *rendering* support exists
and is real. It was **not independently re-verified against a running
FlightGear instance in this session** (FlightGear is not installed in this
sandboxed environment - see item 4) - this conclusion rests on reading the
actual source and the official autotest tooling, not on running it.

### 3. Protocol used

**Native FDM (FGNetFDM), over UDP** - not the generic protocol, not the
property-tree/telnet protocol. Confirmed by `fg_quad_view.sh`'s own
`--native-fdm=socket,in,10,,5503,udp` flag and by the `FGNetFDM`
struct/`ByteSwap()`/`fg_socket.send()` implementation above. `--fdm=external`
is paired with it on the FlightGear command line specifically so FlightGear
disables its *own* internal physics and treats the incoming struct purely
as a rendering feed - see item 9.

### 4. Exact FlightGear version tested

**None was installed or tested live in this session.** This is stated
honestly, not glossed over: this sandboxed development environment does
not have FlightGear installed, and no live FlightGear window was opened at
any point in this phase. What is documented instead: `FG_NET_FDM_VERSION`
(the `fdm.version = 0x18` / 24 field) has been described by FlightGear's
own developers as frozen for backward compatibility - a proposed bump to
25 was reverted specifically to preserve it - and is understood to remain
current through the FlightGear 2020.3.x-2024.x release line ([FlightGear
forum: "FGNetFDM version 24"](https://flightgear-devel.narkive.com/YSoGWndx/fgnetfdm-version-24)).
This project's own struct implementation (`telemetry_mapping.py`) matches
ArduPilot's own working copy of that struct byte-for-byte (see "Protocol
details" below) - but "the bytes are correctly formed" is not the same
claim as "a specific FlightGear release rendered them correctly," which
this session could not observe.

### 5. Exact aircraft/model used

ArduPilot's own bundled `arducopter` aircraft
(`Tools/autotest/aircraft/arducopter/plus_quad2-set.xml`) - the same model
`fg_quad_view.sh` loads via `--aircraft=arducopter --fg-aircraft=.../aircraft`.
Not independently rendered/observed in this session (see item 4).

### 6. Exact ports and loopback endpoints

| Path | Address | Port | Direction |
|---|---|---|---|
| ArduPilot's own native `--enable-fgview` | `--fg <address>`, default `127.0.0.1` | UDP `5503` (`FG_VIEW_PORT`, `+10*instance`) | `arducopter` process -> FlightGear |
| This project's own bridge (`--flightgear-view`) | `--flightgear-endpoint` - **required, explicit, loopback-only** (`127.0.0.1`/`localhost`/`::1`; every other host is rejected before any socket opens) | operator-chosen (no default assumed - matches the CLI requirement's own `<documented-port>` placeholder; `5503` if following ArduPilot's own convention) | this project's script -> FlightGear |

### 7. Whether FlightGear receives position and attitude correctly

**Not independently confirmed live** - no running FlightGear instance was
available to observe in this session (see item 4). Confidence basis for
this project's own bridge: its wire struct is copied field-for-field,
including byte order, from ArduPilot's own already-working
`_output_to_flightgear()` (see "Protocol details"), its total size
matches the publicly documented 408-byte `FG_NET_FDM_VERSION=24` layout
(`tests/test_phase11_flightgear_viewer.py::test_fg_net_fdm_struct_size_is_408_bytes`),
and every axis/unit conversion has a dedicated test asserting the exact
documented formula (`test_ned_to_enu_matches_documented_formula`,
`test_yaw_enu_to_ned_facing_east_is_compass_heading_90deg`, etc. - see
"Axis and unit conversions" below). None of that substitutes for actually
watching a FlightGear window.

### 8. Whether FlightGear sends anything back to ArduPilot

**No, by protocol design, in both paths.** ArduPilot's native
`_output_to_flightgear()` only ever calls `fg_socket.send()` - there is no
corresponding `fg_socket.recv()` anywhere in `SITL_State.cpp`/`.h` for this
socket. This project's own `FlightGearBridge`
(`swarm_sim/visualization/flightgear_bridge.py`) is structurally the same:
it opens a plain UDP socket and only ever calls `.sendto()` - it never
calls `.recv()`/`.recvfrom()` at all, which a UDP `sendto()` doesn't even
require (no session/handshake is ever established either way). Verified
structurally: `tests/test_phase11_flightgear_viewer_architecture.py::test_bridge_never_calls_socket_recv`.

### 9. Native flight-dynamics integration or read-only visualization bridge

**Read-only visualization, in both paths.** `fg_quad_view.sh` pairs
`--native-fdm=socket,in,...` with `--fdm=external`, which is FlightGear's
own documented way of disabling its internal flight-dynamics model and
treating the incoming struct purely as something to render - FlightGear
provides no physics back over this channel (see item 8). This project's
own bridge is explicitly, repeatedly labeled throughout this document and
its own module docstrings as **"a telemetry-renderer prototype,"** never
claimed as a native flight-dynamics integration - see the closing
statement at the end of this document.

### 10. Whether the bridge can run on Windows while ArduPilot SITL runs in WSL2

**Likely yes, but not independently verified live in this session** (no
FlightGear installed - see item 4). WSL2's default `localhostForwarding`
setting forwards both TCP and UDP traffic on `127.0.0.1` between Windows
and WSL2 in modern Windows/WSL builds, which would let a Windows-native
FlightGear receive UDP packets from a Python bridge running inside WSL2
without any extra configuration. However, this project's own prior,
hard-won experience in this exact environment (Phase 10's documented
WSL2<->Windows **TCP** forwarding regression, which reproduced reliably
across multiple attempts before apparently resolving itself by the time of
Phase 10's live flight test) is reason enough to flag the analogous UDP
path as "should work, but verify independently before relying on it,"
rather than asserting it with confidence nothing in this session could
back up. **The lowest-risk configuration, with zero Windows<->WSL2
networking involved for this specific data path, is running FlightGear
*inside* WSL2 as well** (via WSLg, supported on modern Windows 11 WSL2
setups) alongside ArduPilot SITL - see "FlightGear startup" below for both
configurations.

## What this means for the implementation

This project's own `--flightgear-view` bridge is **not** the same pipe as
ArduPilot's native `--enable-fgview` path - that native path is entirely
internal to the `arducopter` C++ process and never touches MAVLink at all,
so there is nothing for a MAVLink-based bridge to "reuse" from it. This
phase's required deliverable (`swarm_sim/visualization/flightgear_bridge.py`,
built on `ArduPilotSITLTransport`) is therefore an **independent,
MAVLink-sourced telemetry-to-FlightGear renderer prototype** that
deliberately speaks the *same wire protocol* (native FDM / FGNetFDM,
version 24, UDP) as the native path - chosen specifically because it is a
real, source-verified, already-interoperating protocol, not invented or
guessed for this task. Per this phase's own explicit instruction:

> Native ArduCopter FlightGear integration was not available or was not
> verified for this ArduPilot version. This implementation is a read-only
> telemetry renderer prototype and does not replace ArduPilot SITL physics.

(More precisely: native integration for *rendering* **is** available and
source-verified, as documented above - what was **not** verified is a live
FlightGear instance actually rendering anything, from either path, in this
session.)

## Target architecture

```
ArduCopter SITL (physics + command authority, unchanged from Phase 8-10)
    |
    +-- MAVLink --> existing swarm_sim telemetry/transport (ArduPilotSITLTransport,
    |                unmodified - used here only for ATTACH/heartbeat/identity
    |                verification, see "Why this bridge reads telemetry itself" below)
    |
    +-- MAVLink --> this phase's own read-only telemetry read loop
                        |
                        v
                 telemetry_mapping.py (pure NED->ENU->FlightGear-NED conversion,
                 no I/O - reuses swarm_sim.autopilot.frames, never reinvents it)
                        |
                        v
                 flightgear_bridge.py (send-only UDP socket, loopback-only)
                        |
                        v
                 FlightGear 3D window (operator-started manually - this project
                 never spawns it; renders position/attitude/altitude/terrain/
                 sky/aircraft model; never commands the vehicle)
```

## Protocol details (FGNetFDM, version 24)

**Field layout, order, and exact struct size (408 bytes, no compiler
padding) are copied verbatim from ArduPilot's own working reference
implementation** - `class FGNetFDM` in `libraries/SITL/SIM_JSBSim.h`
(commit `92b0cd78`) - not from FlightGear's upstream source directly,
since ArduPilot's copy is the one *this exact SITL binary* has been
proven (via `fg_quad_view.sh`) to correctly interoperate with. See
`swarm_sim/visualization/telemetry_mapping.py`'s own module docstring for
the complete field-by-field citation.

| Property | Value | Source |
|---|---|---|
| Protocol file | none needed - native FDM is a fixed binary struct, not a configurable `.xml` protocol file (that's the *generic* protocol, not used here) | `fg_quad_view.sh` |
| Aircraft model | `arducopter` (`Tools/autotest/aircraft/arducopter/plus_quad2-set.xml`) | `fg_quad_view.sh` |
| Input/output direction | this project -> FlightGear only; FlightGear is the `in` side of `--native-fdm=socket,in,...` (it receives) | `fg_quad_view.sh`, `SITL_State.cpp` |
| Port | operator-chosen via `--flightgear-endpoint` (this project never assumes one); `5503` if following ArduPilot's own convention | `SITL_cmdline.cpp` (`FG_VIEW_PORT`) |
| Update rate | bounded `1-30 Hz`, default `10 Hz` (`--update-rate-hz`) | this phase's own requirement |
| Units | latitude/longitude: radians; altitude/agl: meters; phi/theta/psi/rates: radians/(rad/s); v_north/v_east/v_down/climb_rate: **feet/sec** | `SIM_JSBSim.h` field comments |
| Coordinate frame | position: absolute geodetic (WGS84 lat/lon/alt) - **not** a local NED/ENU offset; velocity: NED-labeled (`v_north`/`v_east`/`v_down`), in ft/s | `SIM_JSBSim.h`, `SITL_State.cpp::_output_to_flightgear()` |
| Timestamp semantics | `cur_time`/`warp` deliberately left `0` - ArduPilot's own send path never sets them either (confirmed by reading `_output_to_flightgear()` - it only sets the fields listed at the top of item 1-2) | `SITL_State.cpp` |
| Read-only? | Yes, both paths - see item 8/9 | see above |
| Byte order | **big-endian (network byte order)** - `FGNetFDM::ByteSwap()` calls `ntohl()` on every 32-bit word before `fg_socket.send()`, which on the little-endian host SITL runs on converts host-native values to network order for the wire; `telemetry_mapping.pack_fg_net_fdm` packs directly in that same big-endian form (`>` struct format) | `SIM_JSBSim.cpp::FGNetFDM::ByteSwap()` |

### Fields this project's bridge deliberately leaves at zero (never fabricated)

- `vcas` - ArduPilot's own two usages of this exact field **disagree on
  units** (feet/sec in the send path's `/0.3048` division vs. knots in an
  unrelated receive-only path's `KNOTS_TO_METERS_PER_SECOND` multiply), so
  it is not reliably documented even within ArduPilot's own source, and is
  not one of this phase's required render fields (position/attitude/
  altitude/terrain/sky/aircraft model).
- `alpha`/`beta`/`stall_warning`/`slip_deg` - fixed-wing-only concepts, not
  meaningful for a multirotor.
- `v_body_u/v/w`, `A_X/Y/Z_pilot` - this project's telemetry has no
  body-frame velocity or acceleration data (Phase 9 documented the same
  honest simplification for acceleration).
- `num_engines = 0` and every engine/gear/fuel array - no per-motor RPM
  data is available from this project's own telemetry, so it reports zero
  engines rather than animate rotors that were never actually measured.

## Axis and unit conversions (no guessing - see tests)

Per `ArduPilotSITLTransport`'s own documented behavior
(`ardupilot_transport.py`'s `_poll_incoming` comments), a telemetry
snapshot already carries **position/velocity converted to this project's
canonical `LOCAL_ENU`** (its default `operating_frame`), with **yaw also
converted to ENU**, but **roll/pitch left in ArduPilot's raw NED/body
Euler convention** (nothing in this project's `frames.py` defines a
roll/pitch NED<->ENU conversion, since nothing consumes it besides the
yaw-only BODY-frame math). This phase's own read loop (see next section)
reconstructs the same state itself, reusing the exact same
`swarm_sim.autopilot.frames.convert_vector_frame`/`_yaw_ned_to_enu`
functions.

Going the other way, into an FGNetFDM frame:

- **position -> lat/lon/alt**: FGNetFDM's position fields are *absolute
  geodetic* (a real protocol requirement - see item 3/6), not a NED/ENU
  offset, so this project's own local ENU position (`east`/`north`/`up`
  relative to home) is **not** used for FGNetFDM's `latitude`/`longitude`
  at all; those come from a separately-read `GLOBAL_POSITION_INT` message
  instead (see next section). `agl` uses `GLOBAL_POSITION_INT`'s own
  `relative_alt` field (height above the home/arming point) rather than
  ArduPilot's own send-path simplification of just duplicating `altitude`
  - a real, correctly-scoped MAVLink field, not an invented one.
- **velocity -> v_north/v_east/v_down**: this project's canonical ENU
  velocity is converted back to NED (`telemetry_mapping.enu_to_ned` - the
  same relabeling run backward, it is its own inverse) and then to feet/sec
  (FGNetFDM's documented unit) - `east=NED.y, north=NED.x, up=-NED.z` per
  `frames.py`'s own documented convention, verified by
  `test_ned_to_enu_matches_documented_formula`/`test_enu_to_ned_is_the_inverse_relabeling`.
- **yaw -> psi**: the transport's ENU yaw is converted back to NED/compass
  heading via `telemetry_mapping.yaw_enu_to_ned_for_flightgear` (reusing
  `frames.py`'s own `_yaw_ned_to_enu` formula, `pi/2 - yaw` - an
  involution, so reusing it in this direction is exact, not an
  approximation). Verified with a known heading:
  `test_yaw_enu_to_ned_facing_east_is_compass_heading_90deg` (ENU yaw 0 =
  facing East = compass heading 90 deg) and
  `test_yaw_enu_to_ned_facing_north_is_compass_heading_0deg` (ENU yaw pi/2
  = facing North = compass heading 0 deg). Getting this backwards would
  point the rendered aircraft in the wrong compass direction while still
  "looking" numerically plausible - exactly the "guessed axes" failure
  mode this phase warned against.
- **roll/pitch -> phi/theta**: passed straight through, unconverted - the
  transport already leaves them in the raw NED/body Euler convention
  FlightGear's own `_output_to_flightgear()` also uses directly
  (`fdm.phi = radians(sfdm.rollDeg)`, no sign flip).

## Why this bridge reads telemetry itself rather than only calling `ArduPilotSITLTransport.receive_telemetry()`

A real, session-discovered subtlety, documented honestly: `ArduPilotSITLTransport.receive_telemetry()`'s
own `_poll_incoming()` does a **non-blocking drain of every pending
message** on the connection and silently discards any type it does not
itself recognize (it only updates state for `HEARTBEAT`/
`LOCAL_POSITION_NED`/`ATTITUDE`/`SYS_STATUS`/`EKF_STATUS_REPORT`). This
project needs `GLOBAL_POSITION_INT` too (for absolute lat/lon/alt - see
above), which the transport does not request or parse. A MAVLink
connection is a single-consumer byte stream, so calling
`receive_telemetry()` and then separately `recv_match(type="GLOBAL_POSITION_INT")`
on the *same* connection race each other: whichever runs first consumes
and discards the messages the other one needed - reproduced live while
building this phase (every sample was reported as `dropped_invalid_reason`
until this was fixed).

A second MAVLink connection to the same vehicle was considered and
rejected for the automated test suite specifically: `pymavlink`'s
`tcpin:` socket class (used by `tests/_dummy_ardupilot_vehicle.py`, the
shared real-MAVLink-over-a-real-socket test fixture) only ever accepts one
client (`self.listen.listen(1)`, and `recv()` only calls `accept()` once,
never again while a client is already connected) - a second connection
would work against real ArduPilot SITL (which is a proper multi-client
MAVLink router, as Phase 9's own live Mission-Planner-plus-this-project's-
transport demo already proved) but would hang against the test double,
making the whole test suite depend on real ArduPilot SITL being available.

Per this phase's own item 2 ("read telemetry using the existing
`ArduPilotSITLTransport` **or a strictly read-only telemetry client**"),
the chosen design uses the transport for attach/heartbeat/identity/
namespace verification only (Phase 8's own proven, unmodified ATTACH
logic - see `run_flightgear_view`'s own `transport.start()` call), then
reads the *same already-open connection* itself for the actual streaming
loop: one unified, non-blocking, no-type-filter drain per tick
(`_poll_telemetry_tick`), dispatching `HEARTBEAT`/`LOCAL_POSITION_NED`/
`ATTITUDE`/`GLOBAL_POSITION_INT` together. The conversion math is not
reimplemented - it calls the exact same
`swarm_sim.autopilot.frames.convert_vector_frame`/`_yaw_ned_to_enu`
functions `ardupilot_transport.py` itself calls internally, so this is a
different *read loop*, not a different *conversion*.

## Viewer modes (never merged - separate report shapes, separate result files)

| Mode | Behavior | Opens a socket? |
|---|---|---|
| `--dry-run` (default) | Validates configuration shape only | Never |
| `--diagnose` | Reports the documented protocol facts above; validates argument shapes | Never - not even to ArduPilot |
| `--attach` | Delegates to Phase 9's `run_telemetry_visualization`, completely unmodified | MAVLink only |
| `--flightgear-view` | Attaches to ArduPilot SITL, reads telemetry, sends read-only renderer frames to FlightGear | MAVLink + UDP (both loopback-only) |
| `--replay <telemetry.csv>` | Offline - re-runs previously-saved rows back through the same conversion/packing pipeline for a summary report | Never |
| `sitl_flight_test`/`hold_test`/`planner_preview` | Phase 10/9's own modes, entirely unchanged | (their own, unaffected) |
| FakeSITL | Phase 7, unaffected | N/A |
| PyBullet | `run_mission.py --gui`, unaffected | N/A |

Every mode writes its own `results/phase11_flightgear/phase11_<mode>_result.json`
- reports are never merged.

## Required CLI

```bash
# Dry-run (always safe, never opens a socket)
python scripts/run_phase11_flightgear_viewer.py \
  --dry-run \
  --connection tcp:127.0.0.1:5760 \
  --system-id 1 \
  --component-id 1 \
  --namespace sitl/drone0

# Diagnose (reports the protocol facts above, never connects to anything)
python scripts/run_phase11_flightgear_viewer.py \
  --diagnose \
  --connection tcp:127.0.0.1:5760 \
  --flightgear-endpoint 127.0.0.1:5503

# Attach (telemetry_only, delegates to Phase 9 unmodified)
python scripts/run_phase11_flightgear_viewer.py \
  --attach \
  --connection tcp:127.0.0.1:5760 \
  --system-id 1 \
  --component-id 1 \
  --namespace sitl/drone0 \
  --duration 120

# FlightGear view (the actual bridge)
python scripts/run_phase11_flightgear_viewer.py \
  --flightgear-view \
  --connection tcp:127.0.0.1:5760 \
  --system-id 1 \
  --component-id 1 \
  --namespace sitl/drone0 \
  --flightgear-endpoint 127.0.0.1:5503 \
  --duration 120

# Replay (offline, no ArduPilot or FlightGear needed)
python scripts/run_phase11_flightgear_viewer.py \
  --replay results/phase11_flightgear/telemetry.csv
```

`--flightgear-endpoint` is always explicit and always loopback-checked
before any endpoint is built (`swarm_sim/visualization/flightgear_bridge.py::parse_flightgear_endpoint`)
- every non-loopback host is rejected, in every mode that accepts it.

## FlightGear startup

**No FlightGear version was tested live in this session** (see item 4) -
the commands below are the documented, source-verified commands for the
protocol this project's bridge speaks (matching `fg_quad_view.sh` exactly
for the native-FDM flags), not commands this session ran and observed.

### Recommended: FlightGear inside WSL2 (avoids the Windows<->WSL2 boundary entirely)

Requires WSLg (GUI app support in WSL2, standard on modern Windows 11) and
FlightGear installed inside the WSL2 distro:

```bash
fgfs \
    --native-fdm=socket,in,10,,5503,udp \
    --fdm=external \
    --aircraft=arducopter \
    --fg-aircraft="$(dirname "$(which arducopter)")/../../Tools/autotest/aircraft" \
    --lat=-35.363261 --lon=149.165230 --altitude=584 --heading=353 \
    --disable-hud-3d --disable-random-objects --disable-ai-models
```

Then, in a separate WSL2 terminal (ArduPilot SITL, as already established
in Phase 8-10):

```bash
./build/sitl/bin/arducopter --model quad --speedup 1 -I0 \
    --home -35.363261,149.165230,584,353 --sysid 1 \
    --defaults Tools/autotest/default_params/copter.parm
```

Then this project's bridge, also inside WSL2, pointed at the same loopback:

```bash
python scripts/run_phase11_flightgear_viewer.py --flightgear-view \
    --connection tcp:127.0.0.1:5760 --namespace sitl/drone0 \
    --flightgear-endpoint 127.0.0.1:5503 --duration 120
```

### Alternative: FlightGear on native Windows, ArduPilot SITL in WSL2

Not independently verified in this session (see item 10). Windows'
FlightGear command (PowerShell), assuming WSL2's default
`localhostForwarding` correctly forwards the UDP packets from WSL2:

```powershell
& "C:\Program Files\FlightGear 2024.1\bin\fgfs.exe" `
    --native-fdm=socket,in,10,,5503,udp `
    --fdm=external `
    --lat=-35.363261 --lon=149.165230 --altitude=584 --heading=353
```

(Windows FlightGear does not ship ArduPilot's `arducopter` aircraft model
by default - either copy `Tools/autotest/aircraft/arducopter/` from the
WSL2 checkout into FlightGear's own `data/Aircraft/` directory first, or
omit `--aircraft=arducopter` to use a generic FlightGear aircraft purely
to confirm the UDP link works before chasing down the model.)

### Ubuntu (native, no WSL at all)

Identical to the WSL2-inside command above - WSL2 Ubuntu and native Ubuntu
behave the same way for this purely-loopback, same-host setup.

## Safety architecture

`tests/test_phase11_flightgear_viewer_architecture.py` (24 tests) proves,
via AST inspection of `scripts/run_phase11_flightgear_viewer.py` and
`swarm_sim/visualization/{flightgear_bridge,telemetry_mapping}.py`:

- no arm/takeoff/land/mode/setpoint identifiers or calls anywhere;
- the force-arm magic value (`21196`) never appears;
- the **only** `command_long_send` target anywhere is `MAV_CMD_SET_MESSAGE_INTERVAL`;
- `flightgear_bridge.py` never calls `command_long_send` at all;
- no `param_set_send`/`mav_param_set_send` call exists anywhere;
- no process-spawning/killing identifiers, no `subprocess` import;
- no hardware imports (serial/RPi/etc.);
- default connection is loopback; no hardcoded non-loopback IP literal anywhere;
- `parse_flightgear_endpoint` structurally checks against a loopback-only host list;
- `flightgear_bridge.py` never calls `.recv()`/`.recvfrom()` at all (structural proof FlightGear can never influence this project);
- every `ArduPilotSITLTransport(...)` construction uses a single-vehicle dict;
- no `num_drones`/`VEHICLE_IDS` identifiers;
- no `distributed_consensus`/`pybullet`/`FloodSearchMission` references;
- no `CandidateCommand`/`SafetySupervisor` access (no raw planner candidate path);
- the default CLI never selects a live mode, and `main()`'s dispatch falls back to `run_dry_run`;
- FakeSITL remains importable and functional;
- `ardupilot_transport.py` (Phase 8, unmodified) still never references an arm/takeoff identifier;
- Phase 10's own dry-run/diagnostics functions still never reference one either;
- Phase 9's module is imported for `--attach`, not reimplemented;
- `telemetry_mapping.py` never imports `socket` or `pymavlink` (pure conversion only).

`tests/test_phase11_flightgear_viewer.py` (36 tests) proves the behavioral
side, including dedicated pure-function tests for every axis/unit
conversion formula (see "Axis and unit conversions" above), a full
`--flightgear-view` happy path against a real local UDP socket standing in
for FlightGear (asserting the received datagram is exactly 408 bytes with
`version == 24`), a wire-level check that the vehicle only ever receives
`MAV_CMD_SET_MESSAGE_INTERVAL`, an honest-dropping test (no
`GLOBAL_POSITION_INT` available -> zero frames sent, never a fabricated
lat/lon=0 frame), `--replay` round-tripping a previously-saved CSV, and
regression checks for FakeSITL and Phase 10.

## Required validation commands

```bash
python -m compileall -q swarm_sim run_mission.py tests scripts
python -m pytest -q tests
```

## Results (this session)

- **FlightGear installed or unavailable**: **unavailable** - not installed in this sandboxed development environment.
- **Exact version**: not applicable (never launched).
- **Exact protocol**: native FDM / FGNetFDM, version 24, UDP - source-verified (see above), not live-verified against a running FlightGear.
- **Exact model**: `arducopter` (ArduPilot's own bundled model) - documented, not live-rendered.
- **Exact endpoint**: not applicable (no live run) - the CLI requires an explicit, loopback-only `--flightgear-endpoint` in every case it is used.
- **Whether the window opened**: No - FlightGear was never launched in this session.
- **Whether the aircraft appeared**: Not applicable.
- **Whether altitude/attitude changed**: Not applicable.
- **Whether position and yaw were correct**: Not applicable to observe live; the axis/unit conversion formulas themselves are unit-tested against the documented protocol (see "Axis and unit conversions").
- **Whether any commands were sent to ArduPilot**: Yes, exactly one class of command - `MAV_CMD_SET_MESSAGE_INTERVAL` (a read-only telemetry-stream-rate request), verified both structurally (AST) and live against the test double (`test_flightgear_view_never_sends_anything_but_message_interval_request`). No arm, takeoff, land, mode, or setpoint command was ever sent.
- **Maximum displayed altitude**: not applicable (no live FlightGear render) - the offline `--replay` demonstration below processes Phase 10's own recorded `1.720716 m` result through the same conversion pipeline and reports it correctly as `max_altitude_m - min_altitude_m = 1.720716`, but this is **not** a live FlightGear display.
- **Native integration or telemetry-renderer prototype**: **telemetry-renderer prototype** - see "What this means for the implementation" above. Native ArduCopter-to-FlightGear *rendering* integration does exist in this ArduPilot version (source-verified, item 1-2), but this project's own deliverable is a separate, independent, MAVLink-sourced bridge that speaks the same wire protocol; it was not itself verified live.

### Offline demonstration reusing Phase 10's real recorded result

Phase 10's own `results/phase10_sitl_flight_test/phase10_sitl_flight_test_result.json`
records `takeoff.max_altitude_observed_m = 1.720716` from an actual live
arm-takeoff-hold-land-disarm sequence (Phase 10's own report, not
reproduced or re-run in this phase - no arm/takeoff command was sent in
Phase 11). Phase 10 does not save a continuous telemetry trace, so a small,
explicitly-labeled reconstruction
(`results/phase11_flightgear/phase10_reused_demo.csv`, 3 rows: ground
before takeoff, the recorded peak altitude, ground after landing) was built
to demonstrate `--replay` against that real number rather than a
fabricated one:

```bash
$ python scripts/run_phase11_flightgear_viewer.py --replay results/phase11_flightgear/phase10_reused_demo.csv
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

`585.720716 - 584.0 = 1.720716` - matching Phase 10's own recorded value
exactly. This is an **offline replay of a reconstructed row derived from
Phase 10's own report, not a live flight and not a live FlightGear
render** - stated plainly, not implied otherwise.

## Final report

- **ArduPilot SITL remained the physics authority**: Yes, in every mode - this phase never sends an arm, takeoff, land, mode, or setpoint command (verified structurally and behaviorally - see "Safety architecture").
- **FlightGear was used only for rendering**: Yes, by design and by protocol - see items 8/9. It was never launched live in this session (see above), so this is a design/structural guarantee, not an observed one this time.
- **No Pixhawk, serial, radio, real aircraft, outdoor vehicle, motor/PWM output, or hardware-in-the-loop was used.**
- **Whether native ArduCopter FlightGear support was actually verified**: Source-verified (yes, it exists and is Copter-specific - item 1-2), but **not** live-verified against a running FlightGear instance in this session (item 4/7).
- **Whether the 1.720716 m flight was replayed or observed live**: **Replayed offline**, from a small reconstruction of Phase 10's own recorded result (see above) - it was not observed live in FlightGear, and no arm/takeoff/land command was sent in this phase.

This phase uses only a local ArduPilot SITL simulated vehicle and, where
discussed, a local FlightGear renderer. No Pixhawk, serial port, radio,
real aircraft, outdoor vehicle, motor, PWM output, or hardware-in-the-loop
was used.
