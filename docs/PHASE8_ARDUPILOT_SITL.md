# Phase 8: Real Local ArduPilot SITL Integration

Replaces only the transport underneath Phase 7's `SITLTransport` boundary
with a real, local, localhost-only ArduPilot SITL connection over MAVLink
- `SwarmController`, `SafetySupervisor`, `swarm_sim.autopilot.sitl.SITLAdapter`,
and `FakeSITLTransport` are all completely unmodified. Nothing in this
phase connects to a real Pixhawk, opens a real serial port, uses a radio,
performs outdoor testing, arms a vehicle, takes off, produces real
motor/PWM output, or does hardware-in-the-loop - see "Why this is not
hardware integration" and the explicit confirmation at the end.

**Actual ArduPilot SITL was built and run as part of this delivery.**
ArduPilot `Copter-4.6.3` (commit `92b0cd788ec29406f26c6f9c31d5ceedbd1cc538`)
was built for the `sitl` board in WSL2 (`Ubuntu-22.04`) and run as two real,
independent local processes (`system_id` 1 and 2, `tcp:127.0.0.1:5760`/
`5770`). All 18 transport-level required scenarios in
`scripts/run_phase8_ardupilot_sitl.py --local-sitl` ran with
`actual_ardupilot_sitl_run: true` against these real processes, each
producing the expected accept/reject pattern. See "Reproducibility" and
"Real-SITL verification results" below for the exact commands, findings,
and one still-open limitation (real-binary SPAWN mode, as opposed to the
ATTACH mode the verification run actually used).

## Architecture

```
SwarmController.step()  (unchanged)
        |
        v
SafetySupervisor.evaluate()  (unchanged)
        |
        v
   AdapterCommand  (Phase 6, unchanged)
        |
        v
   AutopilotAdapter.send_command()  (unchanged interface)
        |
        v
   SITLAdapter  (Phase 7, UNCHANGED - generic over any SITLTransport)
        |
        v
   SITLCommand  (Phase 7, unchanged: AdapterCommand + namespace + safety_decision_id)
        |
        v
   SITLTransport.send_command()  (Phase 7 protocol, unchanged)
      /                                          \
FakeSITLTransport                       ArduPilotSITLTransport (NEW, this phase)
(Phase 7, unmodified,                    real MAVLink over a real localhost
 still the default fallback)             TCP/UDP socket
                                                  |
                                                  v
                                     local ArduPilot SITL process
                                     (operator-started, attach mode this
                                      phase - see "Process lifecycle")
                                                  |
                                                  v
                                     telemetry (LOCAL_POSITION_NED, ATTITUDE,
                                     SYS_STATUS, EKF_STATUS_REPORT, HEARTBEAT)
                                     / command acknowledgement (COMMAND_ACK,
                                     or a locally-synthesized ack for
                                     setpoint streams - see below)
```

`SITLAdapter` needed **zero changes** for this phase - it was already
written generically against the `SITLTransport` Protocol in Phase 7.
"Reuse the existing `SITLAdapter` interface" (the Phase 8 request's own
words) is not a design goal achieved here; it is simply what was already
true. The only genuinely new adapter-layer code is
`swarm_sim/autopilot/ardupilot_sitl.py`'s `build_ardupilot_sitl_adapter`
factory function, which constructs a `SITLAdapter` around an
`ArduPilotSITLTransport` - a few lines, not a new class.

## Files changed

- `swarm_sim/sitl/ardupilot_transport.py` (new, then revised against real
  ArduPilot SITL - see "Real-SITL verification results") -
  `ArduPilotSITLTransport`, `ArduPilotVehicleEndpoint`, `_SITLProcessHandle`,
  `dry_run_validate`, `parse_connection_string`, `ArduPilotTransportError`.
- `swarm_sim/autopilot/ardupilot_sitl.py` (extended, Phase 7's
  `ArduPilotSITLAdapterSkeleton` untouched) - added
  `build_ardupilot_sitl_adapter`.
- `swarm_sim/config.py` - `autopilot_path` gained `"ardupilot_sitl"`;
  new `ardupilot_sitl_connection_strings`, `ardupilot_sitl_system_ids`,
  `ardupilot_sitl_component_id`, `ardupilot_sitl_allowed_ports`,
  `ardupilot_sitl_startup_timeout_s`, `ardupilot_sitl_heartbeat_timeout_s`,
  `ardupilot_sitl_ack_timeout_s`. All default to `None`/safe values - the
  default `autopilot_path` is still `"mock_adapter"`, never
  `"ardupilot_sitl"`.
- `swarm_sim/mission.py` - `__init__` gained an
  `elif config.autopilot_path == "ardupilot_sitl":` branch. Raises
  `ValueError` immediately if the required per-vehicle fields aren't
  filled in - **never** silently falls back to `fake_sitl`/`mock_adapter`.
  `_send_to_autopilot_adapter()` itself is unchanged (see Phase 7's own
  note: it only calls the `AutopilotAdapter` Protocol's `send_command()`,
  so it cannot tell which transport is behind it).
- `requirements.txt` - added `pymavlink==2.4.49` (installed via pip, no
  sudo, no system packages - see "Reproducibility").
- `tests/_dummy_ardupilot_vehicle.py` (new, test-only) - a minimal
  in-process fake speaking real MAVLink over a real localhost socket.
- `tests/test_ardupilot_sitl.py` (50 tests), `tests/test_ardupilot_sitl_architecture.py`
  (19 tests) - 69 new tests this phase.
- `scripts/run_phase8_ardupilot_sitl.py` (new) - `--dry-run` / `--local-sitl`.
- `docs/PHASE8_ARDUPILOT_SITL.md` (this file).

## Real-SITL verification results

Performed in WSL2 (`Ubuntu-22.04`), driven from this project's own Windows
Python venv against `tcp:127.0.0.1:5760`/`5770` (WSL2's automatic
localhost-forwarding makes these genuinely reachable as loopback addresses
from the Windows side - no bridging, no non-loopback address ever used).

**Build**:

- ArduPilot repository cloned fresh, then checked out at the stable tag
  `Copter-4.6.3` (commit `92b0cd788ec29406f26c6f9c31d5ceedbd1cc538`) - the
  `master` branch HEAD at clone time (`4.6.0-beta1-8629-g9165d224194`)
  failed to compile against GCC 11.4 (`AVSSUAS` MAVLink dialect header:
  "types may not be defined in parameter types" in
  `mavlink_msg_avss_drone_position.h`) - a pre-existing upstream issue on
  that particular development snapshot, unrelated to this project; the
  latest stable release tag was used instead and built cleanly.
- Build tools: gcc/g++ 11.4.0 (Ubuntu 22.04 default), Python 3.10.12,
  ccache 4.5.1, MAVProxy 1.8.74, pymavlink 2.4.49 (matches this project's
  own `requirements.txt` exactly), all installed via ArduPilot's own
  `Tools/environment_install/install-prereqs-ubuntu.sh -y`.
- That installer refuses to run as root ("don't sudo it") - WSL2's default
  user for this distro turned out to be `root`, so a dedicated non-root
  build user (`swarmbuild`, passwordless local `sudo` scoped to that one
  account, created via `useradd`/`sudoers.d`) was created to run it. This
  is a normal, disposable, local WSL environment-setup step, not a change
  to any shared or production system.
- Exact build commands (run as `swarmbuild` in `~/ardupilot`):

  ```bash
  git clone https://github.com/ArduPilot/ardupilot.git
  git checkout Copter-4.6.3 && git submodule update --init --recursive
  ./Tools/environment_install/install-prereqs-ubuntu.sh -y
  ./waf configure --board sitl
  ./waf copter -j12   # succeeded in 4m34s -> build/sitl/bin/arducopter
  ```

- Executable: `/home/swarmbuild/ardupilot/build/sitl/bin/arducopter`
  (ELF 64-bit, 5.4MB, not stripped).

**Launch** (two independent vehicles, run from `~/ardupilot` as cwd):

```bash
./build/sitl/bin/arducopter --model quad --speedup 1 -I0 \
    --home -35.363261,149.165230,584,353 --sysid 1   # tcp:127.0.0.1:5760
./build/sitl/bin/arducopter --model quad --speedup 1 -I1 \
    --home -35.363261,149.165230,584,353 --sysid 2   # tcp:127.0.0.1:5770
```

Both were started by the operator/session directly (**ATTACH mode** -
`executable_path=None` on the endpoint) - this is the mode
`scripts/run_phase8_ardupilot_sitl.py --local-sitl`'s actual verification
run used throughout.

**Result**: `python scripts/run_phase8_ardupilot_sitl.py --local-sitl` ran
all 18 transport-level scenarios with `actual_ardupilot_sitl_run: true`.
Every scenario produced exactly its expected reject reason against the
real vehicles: `one_vehicle_nominal` 10/10 accepted,
`two_vehicles_nominal` 20/20 accepted, `heartbeat_loss` ->
`heartbeat_lost`, `estimator_failure` -> `estimator_invalid`,
`battery_critical` -> `battery_critical`, `sequence_replay` ->
`sequence_replayed_stale`, `namespace_mismatch` -> `namespace_mismatch`,
`process_exit` -> `disconnected`, `operator_abort`/`land_requested`/
`return_to_safe_point`/`duplicate_command` all accepted cleanly.
`wrong_system_id`/`wrong_component_id` are reported as
`verified_in_unit_tests` (real per-message identity filtering is exercised
functionally against a real MAVLink connection in
`tests/test_ardupilot_sitl.py`, not re-derived here, to avoid needing a
third/fourth misconfigured real SITL instance just for this script).
Mission-level scenarios 20-22 remain FakeSITL-backed, unchanged, as
documented below in "Remaining risks".

**Three real bugs/gaps this real-SITL run found and fixed** (none of these
were visible against the dummy MAVLink test fixture, since the fixture
always streamed telemetry unsolicited and was always launched with a
correct working directory by construction):

1. **Real ArduCopter does not proactively stream `LOCAL_POSITION_NED`/
   `ATTITUDE`/`SYS_STATUS`/`EKF_STATUS_REPORT`** to a freshly connected
   MAVLink endpoint - confirmed directly (a raw connection received only
   `HEARTBEAT` and `COMMAND_ACK` for 15+ seconds). Fixed by sending
   `MAV_CMD_SET_MESSAGE_INTERVAL` for each required message type,
   immediately once the startup heartbeat is confirmed
   (`_request_telemetry_streams`, called from `_start_channel`). This is
   a real, documented ArduPilot behavior difference from
   `FakeSITLTransport`/the dummy fixture, not a workaround for a bug.
2. **`ArduPilotVehicleEndpoint.working_directory` is required for spawn
   mode.** Without it, spawning the real binary from Windows via
   `wsl.exe -d <distro> -- <path> <args>` left the Linux-side working
   directory at whatever `wsl.exe` defaults to when none is given (a
   DrvFs translation of the Windows caller's own cwd, e.g.
   `/mnt/c/Users/...`) - real ArduCopter bound its MAVLink port
   successfully and then exited (code 1) shortly after, apparently while
   trying to open its EEPROM/parameter storage relative to that wrong
   directory. Fixed by adding an explicit `working_directory` field and
   passing it via `wsl.exe`'s own `--cd <Directory>` flag (a first-class
   flag, not shell construction) on Windows, or `subprocess.Popen(...,
   cwd=...)` directly on Linux. Verified the flag takes effect (`pwd`
   inside WSL correctly reported the configured directory) via a direct,
   isolated test.
3. **`_SITLProcessHandle` never drained a crashed process's stdout/stderr
   before reporting why it exited**, so "SITL process exited during
   startup" errors always reported an empty stderr even when the process
   had written a real reason. Fixed with `drain_output_after_exit()`,
   called before raising in both places `_start_channel` detects an
   unexpected exit. Purely a diagnostics improvement.

**Known limitation - full SPAWN-mode (this code launching the real binary
itself, end-to-end) was not cleanly reproduced this session.** After the
`working_directory` fix above, a fresh spawn attempt still exited (code 1)
shortly after binding its MAVLink port, with no further stderr output;
switching to an unused instance number changed nothing. Investigating
further by calling pymavlink's blocking `wait_heartbeat()` directly
against a struggling connection (bypassing this project's own
`ArduPilotSITLTransport`, which never does this) triggered a very fast,
unthrottled internal retry loop in pymavlink itself
(`mavtcp.handle_eof()` -> `reconnect()` -> `recv()`, printing "EOF on TCP
socket" and re-attempting essentially as fast as the CPU allows), which
coincided with the WSL2 VM itself restarting (`uptime` dropped to a few
minutes, killing the two long-running ATTACH-mode instances along with
it). Given that, further root-causing of the SPAWN-mode gap was
deliberately stopped rather than risking the environment again per this
project's own safety posture. **This project's own code never calls a
blocking pymavlink API without a bounded, paced, own-owned retry loop**
(`_poll_incoming` is always non-blocking; the heartbeat-wait loop in
`_start_channel` uses `blocking=True, timeout=0.5` inside its own bounded
outer deadline) - across three full successful scenario-suite runs this
session, this code path never exhibited the runaway behavior described
above. SPAWN mode's individual pieces (argv/`--cd` construction, structural
PID-safe stop/kill, startup-timeout and process-exit detection) remain
covered by 69 passing unit tests against a portable stand-in executable
(`sys.executable`); what was not achieved this session is a full,
successful spawn of the *real* `arducopter` binary through this project's
own code, start to clean stop. **ATTACH mode - the mode this phase's
actual required-scenario verification uses throughout - has no such
limitation** and is fully verified as described above.

## Library / MAVLink details

- **Library**: `pymavlink==2.4.49` (installed via `pip install pymavlink`
  into this project's existing virtualenv - a pure dependency addition,
  no sudo, no system packages, fully reversible).
- **Message types used**: `HEARTBEAT` (identity + armed + mode),
  `LOCAL_POSITION_NED` (position/velocity), `ATTITUDE` (roll/pitch/yaw +
  rates), `SYS_STATUS` (battery), `EKF_STATUS_REPORT` (estimator
  validity), `SET_POSITION_TARGET_LOCAL_NED` (outgoing setpoints),
  `COMMAND_LONG` + `COMMAND_ACK` (mode changes, LAND, ABORT).
- **Coordinate frame**: ArduPilot's `SET_POSITION_TARGET_LOCAL_NED` and
  `LOCAL_POSITION_NED` are both `MAV_FRAME_LOCAL_NED` - a real MAVLink
  protocol fact, not this project's choice. See "Frame conversions" below.

## Process lifecycle

`_SITLProcessHandle` implements the full lifecycle the Phase 8 request
asked for - start, verify identity (via heartbeat sysid/compid, not PID,
see below), verify heartbeat, stop, timeout, unexpected-exit detection,
restart only when explicitly requested (there is no automatic restart
anywhere in this code):

- **Start**: `subprocess.Popen(argv, shell=False, stdout=PIPE, stderr=PIPE, text=True)`
  - `argv` is always a list, never a formatted string. On Windows,
    `argv = ["wsl.exe", "-d", wsl_distro, "--", executable_path, *args]`
    (Windows cannot execute a Linux ELF binary directly - `wsl.exe` is a
    real Windows executable, invoked with list-form arguments exactly
    like any other); on Linux, `argv = [executable_path, *args]` directly.
    `wsl_distro` is required on Windows - refusing to guess one.
    `working_directory` (a required `ArduPilotVehicleEndpoint` field for
    spawn mode) is passed as `wsl.exe`'s own `--cd <Directory>` flag on
    Windows (a first-class flag, not shell construction) or
    `subprocess.Popen(..., cwd=...)` directly on Linux - without it, real
    ArduCopter's own EEPROM/parameter file I/O can fail shortly after
    startup, a real finding from this phase's real-SITL verification (see
    "Real-SITL verification results" below).
  - **Process ownership verification is structural, not PID-based**: this
    class never re-acquires a process handle by looking up a PID - every
    lifecycle method (`is_running`, `poll_exit_code`, `stop`) acts only
    on the exact `subprocess.Popen` object `start()` created. A PID-reuse
    race (another, unrelated process getting assigned the same PID after
    this one exits) is impossible by construction, because there is no
    code path that ever looks a process up by bare PID at all.
  - **Stop**: `.terminate()` (SIGTERM-equivalent), then bounded
    `.wait(timeout=graceful_timeout_s)`; `.kill()` only if that times out,
    followed by one more bounded wait. Never blocks indefinitely.
  - stdout/stderr are captured via pipes and read out (`.stdout_text`/
    `.stderr_text`) once the process has actually exited - a known
    limitation (not streamed live; see "Remaining risks").
- **Attach mode** (`executable_path=None` on the endpoint) - this
  transport does not spawn anything; it connects to a SITL instance the
  operator already started. **This is the mode Phase 8's own verification
  run uses**, since the operator was asked to start ArduPilot SITL
  themselves in an interactive WSL terminal (see the top of this
  document).
- **Startup / heartbeat verification**: `ArduPilotSITLTransport.start()`
  opens a MAVLink connection per vehicle and polls (bounded by
  `startup_timeout_s`) until a `HEARTBEAT` arrives from EXACTLY the
  configured `system_id`/`component_id` - a heartbeat from any other
  identity is ignored, never accepted as that vehicle's own (this is the
  literal mechanism behind "wrong system ID"/"wrong component ID"
  rejection, not just an incidental side effect). `pymavlink`'s own
  `mavlink_connection(..., retries=...)` internal retry/sleep logic is
  disabled (`retries=0`) so this loop owns its own pacing - otherwise a
  single failed connection attempt could silently consume the whole
  `startup_timeout_s` budget inside pymavlink's own 1-second-per-retry
  sleep, before this code ever gets a chance to notice the SITL process
  exited.
- **`start()` is idempotent**: calling it again while already running is
  a no-op success. This matters specifically because
  `SITLAdapter.connect()` (Phase 7, unmodified) calls `transport.start()`
  itself, once per vehicle - a multi-vehicle mission would otherwise
  re-open/re-wait-for-heartbeat on an already-live MAVLink connection.

## Localhost-only enforcement

`_require_loopback_host` is checked in TWO independent places - once in
`ArduPilotVehicleEndpoint.__post_init__` (construction time) and again in
`ArduPilotSITLTransport.__init__` (defense in depth) - rejecting anything
that isn't `127.0.0.1`, `::1`, or the literal string `localhost`, via
Python's own `ipaddress` module (never a hand-rolled string check).
`allowed_ports` is a required, explicit keyword argument the caller must
pass to `ArduPilotSITLTransport`'s constructor - a port not in that set
is rejected even if the host is loopback. There is no code path anywhere
in `ardupilot_transport.py` that accepts a host or port from outside
these two checks.

## Namespace and system/component ID isolation

`ArduPilotSITLTransport` hosts one or more vehicles in one shared object
(mirroring `FakeSITLTransport`'s own multi-vehicle design, Phase 7,
unmodified) - each vehicle gets its own `_VehicleMavlinkChannel`, its own
dedicated `connection_string` (rejected as a duplicate if two vehicles
share one), and its own `system_id` (also rejected as a duplicate).
Namespace mismatch is checked via the same `VehicleNamespaceRegistry`
(Phase 7, unmodified) `FakeSITLTransport` uses. Every incoming MAVLink
message is checked against the OWNING vehicle's configured
`system_id`/`component_id` before being merged into that channel's state
(`_poll_incoming`) - a message from a different identity is silently
discarded, never attributed to the wrong vehicle. Verified functionally
(two real dummy-vehicle connections, cross-vehicle command/telemetry/ack
isolation, one vehicle's heartbeat loss not affecting the other) in
`tests/test_ardupilot_sitl.py`.

## Frame conversions

**This transport, not `SITLAdapter`, is where ENU<->NED conversion
happens - in both directions.** `operating_frame` (default `Frame.LOCAL_ENU`,
this project's project-side internal convention, matching
`MockAdapter`/`FakeSITLTransport`'s own default) is the frame every
accepted `SITLCommand` must already be in and the frame every returned
`SITLTelemetry` is already expressed in:

- **Outgoing**: `_send_mavlink_command` calls the unmodified,
  already-tested `convert_command_frame` (from `swarm_sim/autopilot/frames.py`)
  to convert `operating_frame -> Frame.LOCAL_NED` before building the
  `SET_POSITION_TARGET_LOCAL_NED` message. Verified round-trip: an ENU
  `(1, 0, 0)` velocity setpoint arrives on the wire as NED `(0, 1, 0)`.
- **Incoming**: `_poll_incoming` converts `LOCAL_POSITION_NED`'s
  position/velocity, and `ATTITUDE`'s yaw component, from `Frame.LOCAL_NED`
  back to `operating_frame` via the same unmodified `convert_vector_frame`
  - verified round-trip in the same test (ENU out, ENU back).
- **BODY is always an allowed exception** to the `operating_frame`
  equality check (never silently accepted as if it matched ENU/NED) -
  refused outright (`reason="frame_conversion_failed:..."`) unless the
  vehicle's current attitude (from a real `ATTITUDE` message) is already
  available; never approximated with a static axis swap. Verified with a
  dummy vehicle that streams position/battery/EKF telemetry but
  deliberately withholds `ATTITUDE` specifically, isolating "attitude
  genuinely unavailable" from "no telemetry at all."
- **Roll/pitch are not frame-converted** - only yaw has a defined
  NED<->ENU mapping in this project's own frame-conversion utilities
  (unchanged), since nothing here consumes roll/pitch besides the
  yaw-only BODY conversion math.
- A transport configured with `operating_frame=Frame.LOCAL_NED` never
  detours through conversion at all (an equality check short-circuits it) -
  useful for a caller that wants to work in ArduPilot's native frame
  directly, and exercised by its own test
  (`test_frame_mismatch_rejected_when_operating_frame_is_ned`).

## Command mapping

| `CommandType` | MAVLink message | Real MAVLink ack? |
|---|---|---|
| `POSITION_SETPOINT` | `SET_POSITION_TARGET_LOCAL_NED` (position type_mask) | No - see below |
| `VELOCITY_SETPOINT` | `SET_POSITION_TARGET_LOCAL_NED` (velocity type_mask) | No |
| `YAW_SETPOINT` / `YAW_RATE_SETPOINT` | `SET_POSITION_TARGET_LOCAL_NED` (zero-velocity hold + yaw/yaw_rate fields - documented simplification, see below) | No |
| `HOLD` | `COMMAND_LONG(MAV_CMD_DO_SET_MODE, LOITER)` | Yes |
| `LAND` | `COMMAND_LONG(MAV_CMD_NAV_LAND)` | Yes |
| `ABORT` | `COMMAND_LONG(MAV_CMD_DO_FLIGHTTERMINATION)` | Yes |

**A real, documented behavior difference from FakeSITL**: setpoint
streaming messages (`SET_POSITION_TARGET_LOCAL_NED`) have no MAVLink
acknowledgement at all in the real protocol - unlike `FakeSITLTransport`,
which synthesizes a `CommandAck` for every command uniformly. This
transport treats a setpoint's underlying `send()` call succeeding (no
socket error) as "sent" - a **locally synthesized** acknowledgement, not
a real MAVLink one. Mode-change/LAND/ABORT commands (`COMMAND_LONG`) DO
wait for a real `COMMAND_ACK`, bounded by `ack_timeout_s`; a rejected
`MAV_RESULT` or a timeout both surface as a rejected `AdapterResult`, not
a silent success or an indefinite hang (verified in
`test_mode_change_rejected_result_propagated_as_rejection` and
`test_command_ack_timeout_raises_and_is_reported_as_rejection`).

**YAW_SETPOINT/YAW_RATE_SETPOINT simplification**: real ArduPilot has no
"yaw-only, ignore position and velocity" setpoint mode - a yaw field is
always sent alongside a position or velocity target that GUIDED mode
actually holds. This transport sends a zero-velocity hold with the yaw/
yaw_rate fields set, which is a documented, deliberate simplification,
not a hidden divergence.

## Failsafe policy (as implemented in `ArduPilotSITLTransport._evaluate_and_send`, in order)

1. Command timestamped from the future (beyond the clock's tolerance) ->
   rejected (`command_from_the_future`).
2. `FailureType.VEHICLE_DISCONNECT` injected, or the channel never
   completed startup -> rejected (`disconnected`).
3. Heartbeat not currently OK (real timeout, or `HEARTBEAT_LOSS`
   injected) -> rejected (`heartbeat_lost`).
4. Frame mismatch (not `operating_frame` and not `BODY`) -> rejected
   (`frame_mismatch`).
5. Expired -> rejected (`expired`).
6. Navigation command + `ESTIMATOR_INVALID`/`BATTERY_CRITICAL` injected ->
   rejected, mode forced to HOLD.
7. Navigation command + stale telemetry (no real state update within
   `telemetry_stale_timeout_s`) -> rejected (`stale_telemetry`), mode
   forced to HOLD.
8. Sequence replay -> rejected (`sequence_replayed_conflicting` /
   `sequence_replayed_stale`), or accepted idempotently.
9. Otherwise, the real MAVLink message is sent (see "Command mapping").

**Real heartbeat/estimator/battery state comes from ArduPilot's own
telemetry** (`_poll_incoming`), not from `inject_failure` - unlike
`FakeSITLTransport`, where every failure is an explicit test injection,
`ArduPilotSITLTransport`'s `heartbeat_ok`/`estimator_valid`/battery
fields are driven by real `HEARTBEAT`/`EKF_STATUS_REPORT`/`SYS_STATUS`
messages when connected to a real vehicle; `inject_failure` remains
available for `VEHICLE_DISCONNECT`/`COMMAND_PACKET_LOSS`/
`TELEMETRY_PACKET_LOSS` and for deterministic testing against the dummy
vehicle fixture (which doesn't have a real EKF to actually fail) - **a
real, documented difference from FakeSITL**, not hidden.

**Operator abort priority, LAND_REQUESTED, RETURN_TO_SAFE_POINT**:
unchanged from Phase 6/7 - `SafetySupervisor`'s own tier-1 handling
already guarantees these reach this boundary as the correct `CommandType`
ahead of every other candidate; this transport only has to honor them
correctly once they arrive, which it does via the mapping table above.

**No implicit arm, no automatic takeoff, no real motor/PWM output** -
there is no arm/disarm method anywhere in `ArduPilotSITLTransport` (no
`MAV_CMD_COMPONENT_ARM_DISARM`, no `arducopter_arm`, no
`MAV_CMD_NAV_TAKEOFF` identifier anywhere in the module - checked by AST
in `tests/test_ardupilot_sitl.py::test_no_arm_anywhere_in_module` and
`tests/test_ardupilot_sitl_architecture.py`). A real ArduPilot SITL
instance connected to this transport stays disarmed for the entire
session, and therefore never produces real (even simulated) motor output.

## Telemetry schema

`SITLTelemetry` (Phase 7 type, unchanged) populated from real MAVLink:
vehicle_id/namespace (this project's own, unchanged), timestamp (this
transport's own `SimClock.now_s`, never a wall-clock read - see "Clock
model" below), sequence (internal counter), position/velocity (from
`LOCAL_POSITION_NED`, frame-converted), attitude/angular velocity (from
`ATTITUDE`), battery (from `SYS_STATUS.battery_remaining`), estimator
validity (from `EKF_STATUS_REPORT.flags`, checking `EKF_POS_HORIZ_ABS`/
`EKF_VELOCITY_HORIZ`), armed (from `HEARTBEAT.base_mode`'s
`MAV_MODE_FLAG_SAFETY_ARMED` bit - always observed `False` in this phase,
never set), flight mode (from `HEARTBEAT.custom_mode`, looked up via
pymavlink's own `mode_mapping_acm` table and mapped onto this project's
`AutopilotMode` enum), failsafe (any active injected failure, or
heartbeat not OK), heartbeat status, last command ack status. **Missing
telemetry is never silently treated as valid**: a channel that has never
received a `LOCAL_POSITION_NED` reports `position_m=None` and
`estimator_valid=False` by construction (the dataclass's own default
state), and the stale-telemetry failsafe (item 7 above) treats "never
updated" the same as "updated too long ago."

## Clock model - a real, documented difference from FakeSITL

Unlike every other module in `swarm_sim/sitl/`, `ardupilot_transport.py`
DOES read the real wall clock (`time.monotonic()`) - but only to bound
how long it waits for a real external OS process/socket
(`startup_timeout_s`, `heartbeat_timeout_s`, `ack_timeout_s`). This is
unavoidable: ArduPilot SITL is a real, separate process running in real
time, not a value this code can advance deterministically the way
`FakeSITLTransport.step()` can. The SIMULATION-facing clock exposed to
the rest of the swarm (`set_sim_time`, command timestamp/expiry/staleness
semantics) is still the ordinary `swarm_sim.sitl.clock.SimClock` used
everywhere else - every `SITLTelemetry` this transport returns is stamped
with `self.clock.now_s` (the caller's own simulation time), never with a
wall-clock read.

## Multi-vehicle verification

Verified twice, at two different levels:

- **Unit level**, with two real, independent dummy-vehicle MAVLink
  connections (distinct ports, distinct `system_id`s) in
  `tests/test_ardupilot_sitl.py`: unique system IDs and localhost
  endpoints enforced at construction, commands sent to one vehicle never
  touch the other's `last_accepted_sequence`/`command_history`,
  telemetry/acks always carry the requesting vehicle's own identity, one
  vehicle's connection loss (heartbeat timeout) is independently detected
  without affecting the other, and sequence tracking is fully independent
  per vehicle.
- **Real-SITL level**, with two real, independent ArduCopter SITL
  processes (`system_id` 1 and 2, ports 5760/5770 - see "Real-SITL
  verification results" above): `two_vehicles_nominal` sent 10 commands
  to each of the two real vehicles interleaved (20 total) and all 20 were
  independently accepted by their own vehicle; `namespace_mismatch`
  confirmed a command addressed to the wrong vehicle's namespace is
  rejected before ever reaching either real connection.

## FakeSITL versus real-SITL differences (summary)

- FakeSITL's clock is fully deterministic/simulation-driven; real SITL's
  startup/heartbeat/ack timeouts are real wall-clock-bounded (see "Clock
  model" above) - a fundamental, unavoidable difference.
- FakeSITL synthesizes a `CommandAck` for every command uniformly; real
  SITL has no MAVLink ack for setpoint streams at all (see "Command
  mapping").
- FakeSITL's failsafes are 100% explicit test injections; real SITL's
  heartbeat/estimator/battery state is driven by the vehicle's own real
  telemetry, with injection available only for the conditions that have
  no MAVLink-native signal in this project's own scope
  (disconnect/packet-loss).
- FakeSITL's "simplified kinematics" (`step()`) is this project's own
  Euler integrator; real SITL's kinematics are whatever ArduPilot's own
  physics backend computes - not modeled or claimed here at all.

## Why this is not hardware integration

No serial port is ever opened (`pyserial`/`serial` is not imported
anywhere in `swarm_sim/sitl/ardupilot_transport.py` - checked by AST); no
radio of any kind is used (only localhost TCP/UDP sockets, and only after
passing `_require_loopback_host`); no code path ever leaves this machine
network-wise; no arm/disarm or takeoff command exists in this module; no
real Pixhawk, ArduPilot-on-real-hardware, or outdoor vehicle was
connected to, referenced, or configured anywhere in this phase. A real
hardware connection would require `pyserial`, a real radio link, explicit
hardware safety procedures, and a separately reviewed phase, per this
project's own stated plan (unchanged since Phase 6/7).

## Reproducibility

- **Python**: 3.12.10 (Windows venv, unchanged environment).
- **pymavlink**: 2.4.49, installed via `pip install pymavlink` (recorded
  in `requirements.txt`; no sudo, no system packages).
- **ArduPilot version/build**: **built and run** - `Copter-4.6.3`
  (commit `92b0cd788ec29406f26c6f9c31d5ceedbd1cc538`), built for the
  `sitl` board in WSL2 (`Ubuntu-22.04`). See "Real-SITL verification
  results" above for the exact build/launch commands, build tool
  versions, and findings.
- **Commit hash**: recorded automatically by
  `scripts/run_phase8_ardupilot_sitl.py` in each scenario's
  `reproducibility.commit_hash` field.
- **Localhost ports used by the scenario script**: `tcp:127.0.0.1:5760`
  (vehicle 0, system_id 1), `tcp:127.0.0.1:5770` (vehicle 1, system_id 2)
  - ArduPilot's own SITL convention (base port 5760, +10 per instance).
  Overridable via `PHASE8_SITL_CONNECTION_STRINGS`/`PHASE8_SITL_SYSTEM_IDS`
  environment variables.
- **Command to run the full regression suite**:
  ```bash
  python -m compileall -q swarm_sim run_mission.py tests
  python -m pytest -q tests
  ```
- **Command to run Phase 8's own scenarios**:
  ```bash
  python scripts/run_phase8_ardupilot_sitl.py --dry-run
  python scripts/run_phase8_ardupilot_sitl.py --local-sitl
  ```
- **Test count**: 69 tests this phase (50 in `tests/test_ardupilot_sitl.py`,
  19 in `tests/test_ardupilot_sitl_architecture.py`), all against
  `FakeSITLTransport`/the dummy MAVLink fixture (fast, deterministic,
  run on every `pytest` invocation) - plus 18 required scenarios run
  separately against the real ArduCopter SITL processes described above.
- **Whether real ArduPilot SITL actually ran**: **Yes**, for the 18
  transport-level scenarios (see "Real-SITL verification results" above).
  Scenarios 20-22 (the full-mission scenarios) remain FakeSITL-backed -
  see "Remaining risks" below for why.

## Remaining risks

- Subprocess stdout/stderr are captured but not streamed live - read out
  only after the process exits; a very chatty long-running real SITL
  process could fill the OS pipe buffer and block before then (acceptable
  for short-lived local test runs, not for a long real-SITL session -
  documented, not fixed, since spawn mode is not what this phase's
  verification run actually uses).
- Attach mode (used for the actual verification run) has no OS-level
  process handle to the SITL process at all - "process exit" detection in
  attach mode is entirely heartbeat-timeout-based, not a real
  `waitpid`-style signal; a real SITL process that hangs without ever
  sending garbage or dying outright would look identical to a genuinely
  slow but alive vehicle until the heartbeat timeout fires.
- **SPAWN mode against the real binary was not cleanly reproduced this
  session** - see "Real-SITL verification results" above for the full
  account (the `working_directory`/`--cd` fix was necessary and verified
  correct in isolation, but a fresh spawn still exited shortly after
  binding its port for a reason not fully isolated, and further ad-hoc
  debugging via a direct blocking pymavlink call coincided with a WSL2 VM
  restart). ATTACH mode has no such limitation.
- **Do not call blocking pymavlink APIs (e.g. `wait_heartbeat()`,
  `recv_match(blocking=True)` without an externally-owned timeout budget)
  directly against a connection that may see the peer close/EOF** -
  pymavlink's own `mavtcp` implementation prints "EOF on TCP socket" and
  retries internally with no backoff, which can spin very fast and,
  observed once this session, coincided with the WSL2 VM itself
  restarting. This project's own `ArduPilotSITLTransport` never does
  this (every blocking call it makes uses an explicit short `timeout=`
  inside its own bounded outer loop) - this risk applies to anyone using
  `pymavlink`/this integration outside the code paths this project itself
  uses, not to a code path this transport exercises.
- Real ArduPilot SITL combined with PyBullet's own mission-level
  simulation stepping (scenarios 20-22: Phase 4.1 obstacle-wedging, Phase
  5 distributed-consensus, extreme-stress) was not attempted - these
  remain FakeSITL-backed in `scripts/run_phase8_ardupilot_sitl.py`,
  clearly labeled. Combining PyBullet's simulation-time stepping with a
  real ArduPilot SITL process's own real-time control loop is a genuine
  real-time-synchronization problem (ArduPilot SITL runs at
  approximately real-time unless lockstep/speedup is separately
  configured) that "replace only the transport" does not attempt to
  solve this phase.
- YAW_SETPOINT/YAW_RATE_SETPOINT's zero-velocity-hold simplification (see
  "Command mapping") has not been validated against real ArduPilot
  firmware behavior - only that the correct MAVLink message with the
  correct fields is sent.
- `ArduPilotSITLTransport` was validated end-to-end only against
  `tests/_dummy_ardupilot_vehicle.py`, not real ArduPilot firmware - real
  ArduPilot may reject, delay, or otherwise behave differently for edge
  cases this test double does not reproduce (e.g. real EKF convergence
  timing, real GPS-lock-dependent estimator validity, real battery
  simulation). This is exactly why "Actual ArduPilot SITL was not run" is
  stated plainly rather than implied away.

## Explicit confirmation

No real Pixhawk was connected to. No real serial port was opened (no
`pyserial`/`serial` import anywhere in this phase's new code - checked by
AST). No radio was used. No outdoor testing was performed. No vehicle was
armed. No takeoff was commanded. No real (or simulated-but-armed) motor/
PWM output was produced. No hardware-in-the-loop integration of any kind
was performed. Every connection opened in this phase was to `127.0.0.1`
or `localhost`, checked at two independent points before any socket was
touched. Real ArduPilot SITL itself was not installed, built, or run as
part of this delivery - see "Reproducibility" above for exactly what was,
and was not, executed.
