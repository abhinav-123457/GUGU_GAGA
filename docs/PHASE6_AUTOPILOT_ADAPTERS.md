# Phase 6: Autopilot Adapter Boundary

A simulator-safe adapter abstraction between `SafetySupervisor`'s
validated output and whatever ultimately moves a vehicle - PyBullet
today, a real ArduPilot/PX4 transport later. Nothing in this phase
connects to real hardware, transmits MAVLink, arms anything, or starts
SITL - see "Why this is not SITL" and "Why this is not hardware-ready"
below, and the explicit confirmation at the end of this document.

## Architecture

```
SwarmController.step()  (unchanged)
        |
        v
   CandidateCommand  (unvalidated - Phase 4's own untrusted-input type)
        |
        v
SafetySupervisor.evaluate()  (unchanged - Phase 4/4.1)
        |
        v
   SafetyDecision.filtered_command  (a validated contracts.Command)
        |
        v
   AdapterCommand  (wraps the Command + a monotonic sequence number)
        |
        v
   AutopilotAdapter.send_command()
      /            |             \
MockAdapter   ArduPilotAdapter  PX4Adapter
(deterministic  Skeleton         Skeleton
 in-memory,     (offline only -  (offline only -
 this phase's    connect()        connect()
 reference       always fails)    always fails)
 path)
        |
        v
   AdapterResult (accepted + transformed_command, or rejected + reason)
        |
        v
   mission.py applies transformed_command.desired_velocity_mps
        |
        v
   PyBullet (this phase's only real execution target)
```

Every arrow above is a real, traceable call in the code - not aspirational.
`swarm_sim/mission.py`'s `run()` calls `SafetySupervisor.evaluate()`, then
(when `cfg.autopilot_path == "mock_adapter"`, the default) calls
`_send_to_autopilot_adapter()`, which itself calls
`MockAdapter.send_command()`. This ordering, and that a raw
`SwarmController` candidate (`desired_vel`) can never reach
`send_command()`, is checked mechanically in
`tests/test_autopilot_architecture.py` and
`tests/test_safety_supervisor_architecture.py`.

**Why a raw candidate cannot reach an adapter, structurally**:
`AutopilotAdapter.send_command()` accepts only an `AdapterCommand`
(`swarm_sim/autopilot/types.py`), whose only field is `command: Command` -
an already-validated `swarm_sim.contracts.Command`. There is no
constructor path from a bare numpy velocity array to a `Command` without
going through `Command.__post_init__`'s own validation (frame, finite
values, matching required/forbidden fields per `CommandType`), and in
`mission.py` the only `Command` ever handed to `_send_to_autopilot_adapter`
is `decision.filtered_command` - the output of an ACCEPTED
`SafetySupervisor.evaluate()` call.

## Files changed

- `swarm_sim/autopilot/__init__.py`, `types.py`, `base.py`, `frames.py`,
  `mock.py`, `ardupilot.py`, `px4.py` (all new).
- `swarm_sim/config.py` - `autopilot_path` (`"mock_adapter"` default |
  `"direct"`), `autopilot_command_latency_s`, `autopilot_telemetry_latency_s`,
  `autopilot_command_packet_loss_prob`, `autopilot_telemetry_packet_loss_prob`,
  `autopilot_telemetry_stale_timeout_s`.
- `swarm_sim/mission.py` - constructs one `MockAdapter` per drone when
  `autopilot_path == "mock_adapter"`; new `_send_to_autopilot_adapter()`
  helper, called from `run()` right after a safety decision is accepted.
  `SafetySupervisor`, the sensor models, the flocking equations, and
  distributed consensus are untouched.
- `tests/test_autopilot.py` (50 tests), `tests/test_autopilot_architecture.py`
  (9 tests). `tests/test_safety_supervisor_architecture.py` gained one
  new allowed `safe_vel` assignment pattern plus its own guarding test
  (see "Command lifecycle" below).
- `scripts/run_phase6_adapter_scenarios.py` (new).
- `docs/PHASE6_AUTOPILOT_ADAPTERS.md` (this file).

## Command lifecycle

1. `SwarmController.step()` produces a raw candidate velocity - never
   seen by any adapter.
2. `SafetySupervisor.evaluate()` (unchanged) produces a `SafetyDecision`.
   Only on `decision.accepted` does `decision.filtered_command` exist.
3. `mission.py`'s `_send_to_autopilot_adapter()`:
   a. Builds a `VehicleTelemetry` from the mission's own (real PyBullet)
      `VehicleState` and pushes it to the adapter
      (`MockAdapter.push_ground_truth_state`) - the mock's equivalent of
      a real adapter's own telemetry stream.
   b. Wraps `decision.filtered_command` in an `AdapterCommand` with this
      drone's next monotonic sequence number.
   c. Calls `adapter.send_command(adapter_command)`.
4. On acceptance, `mission.py` reads
   `result.transformed_command.command.desired_velocity_mps` and applies
   it (via the existing, unchanged `SpeedController`/PID path) exactly
   like Phase 4's own `safe_vel`. On rejection, velocity is zeroed - the
   same conservative default Phase 4 already uses for a rejected safety
   decision.
5. `autopilot_path == "direct"` skips steps 3-4 entirely (the pre-Phase-6
   behavior, kept only as a **temporary comparison baseline** - see
   "Direct vs. mock-adapter path" below).

## Vehicle connection state / autopilot mode / telemetry / command / result

Defined in `swarm_sim/autopilot/types.py`, exactly as required:

- `ConnectionState`: DISCONNECTED, CONNECTING, CONNECTED, DEGRADED,
  FAILSAFE, DISARMED, ARMED, LANDING, UNKNOWN.
- `AutopilotMode`: HOLD, OFFBOARD, GUIDED, RTL, LAND, ABORT, DISARMED,
  UNKNOWN. `MockAdapter` requests `OFFBOARD` for setpoint-style commands
  (PX4's own term for "tracking external setpoints"); a real ArduPilot
  transport would map the same request onto ArduPilot's `GUIDED` (see
  `ardupilot.py`'s note) - `GUIDED` and `OFFBOARD` are kept as distinct
  enum members for exactly this reason, not merged into one.
- `VehicleTelemetry`: vehicle_id, timestamp_s, frame, position_m,
  velocity_mps, acceleration_mps2 (optional), attitude_rad,
  angular_velocity_radps, battery_fraction, estimator_valid,
  connection_state, autopilot_mode, armed, failsafe, sequence.
- `AdapterCommand`: wraps `command: Command` + `sequence: int` (see
  "Sequence-number semantics").
- `AdapterResult`: accepted, reason, timestamp_s, command_sequence,
  adapter_state, transformed_command (only set when `accepted` is a
  `send_command()` result - `connect()`/`set_mode()`/etc. reuse the same
  result type for non-command acceptance and leave it `None`).

## Frame conventions

Full axis/sign documentation lives in `swarm_sim/autopilot/frames.py`'s
module docstring (the single source of truth - summarized here):

- **LOCAL_ENU** (this project's convention for PyBullet's world frame,
  `contracts.PYBULLET_LOCAL_ENU_AXIS_CONVENTION`): x=East-like,
  y=North-like, z=Up. Yaw 0 points along +x, increasing counter-clockwise -
  matches PyBullet's own Euler yaw.
- **LOCAL_NED** (MAVLink/ArduCopter's real protocol convention):
  x=North, y=East, z=Down. Yaw 0 points along +x (North), increasing
  clockwise (compass heading).
- **BODY** (this project's convention for MAVLink's BODY_FRD): x=Forward,
  y=Right, z=Down, relative to the vehicle's CURRENT yaw - not a fixed
  world relabeling.

**ENU<->NED** is a fixed, always-valid relabeling: `x_ned=y_enu` (North),
`y_ned=x_enu` (East), `z_ned=-z_enu` (Down); applying it twice is the
identity. No attitude needed.

**ENU<->BODY** requires yaw (`attitude_rad[2]`): Forward =
`(cos(yaw), sin(yaw), 0)`, Right = `(sin(yaw), -cos(yaw), 0)` (verified
right-handed FRD: Forward x Right = Down = `(0,0,-1)`). This is a
**documented yaw-only simplification** - roll/pitch are ignored, exact
only for level flight, the same simplification this codebase's own
heading vectors already use elsewhere. **NED<->BODY** always pivots
through LOCAL_ENU (one conversion path, not three).

**If `vehicle_attitude_rad` is not supplied, BODY conversion is refused
outright** (`ValueError`) - never approximated with a static axis swap.
This was an explicit requirement ("do not convert BODY_FRD using only a
static axis swap") and is enforced in `convert_command_frame` and
`convert_vector_frame` alike.

**`convert_command_frame(command, target_frame, vehicle_attitude_rad=None)`**
is the required explicit conversion entry point - returns a new `Command`,
never mutates the input, and raises `ValueError` for an unknown
source/target frame. `yaw_rad`/`yaw_rate_radps` convert between ENU and
NED (`yaw_enu = pi/2 - yaw_ned`, an involution; `yaw_rate` flips sign,
since ENU's z-axis and NED's point opposite ways) but conversion is
refused outright if either field is set and BODY is involved - a
body-relative absolute heading is not a well-defined quantity, and no
`CommandType` in this project actually needs one in BODY frame.

**Adapters never convert frames implicitly.** `MockAdapter.send_command`
rejects (`reason="frame_mismatch"`) any command whose frame does not
match the adapter's configured `operating_frame` - the caller must call
`convert_command_frame` first.

## Timestamp and expiry semantics

`Command.timestamp_s`/`expiration_time_s` (Phase 1, unchanged) are
preserved unmodified through `AdapterCommand` and any frame conversion
(`dataclasses.replace` only touches frame/setpoint/yaw fields). A command
is "fresh" iff `timestamp_s <= now_s <= expiration_time_s`
(`is_command_fresh`, shared by every adapter implementation).

**Clock model**: `send_command()` has no wall-clock dependency - "now"
for a given call is the command's own `timestamp_s` plus the adapter's
configured `command_latency_s` (the "how long did this take to actually
arrive" proxy). This keeps the whole adapter deterministic under a fixed
call sequence (no `time.time()`/`time.monotonic()` anywhere in
`mock.py`), matching this project's simulation-time conventions
elsewhere (`SafetySupervisor.evaluate(..., now_s=...)`). A consequence:
`command_latency_s` can push an otherwise-valid command's effective
arrival time past its own `expiration_time_s`, which is rejected as
`"expired"` - this is how "configurable command latency" and "expired
command rejection" interact (see `scripts/run_phase6_adapter_scenarios.py`'s
`command_latency` and `command_expiry` scenarios).

## Sequence-number semantics

Each `AdapterCommand.sequence` must be non-negative; each `MockAdapter`
tracks the last ACCEPTED sequence per its one vehicle:

- `sequence > last_accepted`: normal case, accepted (subject to every
  other check).
- `sequence == last_accepted` **and identical command content**
  (vehicle_id, command_type, frame, setpoints, yaw fields, timestamp,
  expiry - `MockAdapter._same_command_content`): treated as an
  **idempotent replay** - the exact prior `AdapterResult` is returned
  again, not reprocessed or re-validated. This is the "commands are
  idempotent only when sequence semantics explicitly allow it" case (a
  retried send after a lost acknowledgment, not a new decision).
- `sequence == last_accepted` **with different content**: rejected,
  `reason="sequence_replayed_conflicting"` - the sender is trying to
  reuse a sequence number for a genuinely different command, which is
  never safe to allow.
- `sequence < last_accepted`: rejected, `reason="sequence_replayed_stale"`
  regardless of content - an old sequence number is never re-accepted,
  even if by coincidence its content matches some past command.

## Telemetry validity and stale-telemetry policy

`MockAdapter.push_ground_truth_state()` is the mock's equivalent of a
real adapter's own telemetry stream (not part of the `AutopilotAdapter`
Protocol - a real ArduPilot/PX4 transport would poll MAVLink telemetry
itself; the mock has to be told, since it has no real link). Subject to
`telemetry_packet_loss_prob` (silently dropped) and `telemetry_latency_s`
(delivered only once enough sim-time has passed since the last delivery -
mirrors `CommsNetwork`'s own step-based latency model). `read_telemetry()`
always returns the most recently delivered state (or a `DISCONNECTED`/
`estimator_valid=False` placeholder before the first successful push).

**Stale-telemetry failsafe**: if the gap since the last delivered
telemetry exceeds `telemetry_stale_timeout_s` (default 1.0s) at the
moment a **navigation** command (POSITION/VELOCITY/YAW/YAW_RATE setpoint)
arrives, that command is rejected (`reason="stale_telemetry"`) and the
adapter's own mode is forced to HOLD - exactly the required "stale
telemetry -> reject navigation command and request HOLD" policy.
Non-navigation commands (HOLD/LAND/ABORT) are never blocked by stale
telemetry - blocking the one thing that's already safe would be
counterproductive.

## Failsafe policy (as implemented in `MockAdapter.send_command`, in order)

1. Wrong `vehicle_id` -> rejected (`wrong_vehicle_id`).
2. Disconnected/connecting/unknown state -> rejected (`disconnected`) -
   no implicit reconnection, no queued retry.
3. Failsafe active -> rejected (`failsafe_active`), highest-priority
   rejection short of the two structural checks above.
4. Frame mismatch -> rejected (`frame_mismatch`), never silently converted.
5. Expired (after latency) -> rejected (`expired`).
6. Stale telemetry + navigation command -> rejected (`stale_telemetry`),
   mode forced to HOLD.
7. Sequence replay -> rejected (`sequence_replayed_conflicting` /
   `sequence_replayed_stale`), or accepted idempotently.
8. Packet loss (probabilistic) -> rejected (`packet_loss`).
9. Otherwise accepted; `CommandType.ABORT`/`LAND`/`HOLD` set the adapter's
   own mode directly (ABORT and LAND additionally force
   `ConnectionState.LANDING` for LAND); everything else requests
   `OFFBOARD`.

**Operator abort priority**: `SafetySupervisor`'s own tier-1
operator-abort handling (Phase 4, unchanged) already guarantees an abort
reaches the adapter as a `CommandType.ABORT` `Command` ahead of every
other candidate, every tick - the adapter layer does not re-implement
this priority, it only has to honor the `ABORT` command type correctly
once it arrives (which it does, unconditionally on acceptance).

**LAND_REQUESTED / RETURN_TO_SAFE_POINT**: `SafetyState.LAND_REQUESTED`
maps to a `CommandType.LAND` `Command`, which the adapter turns into
`AutopilotMode.LAND` + `ConnectionState.LANDING` - **explicitly simulated
only**, there is no real descent-rate or motor-cutoff model behind it
(same limitation Phase 4 already documented for the PID's own LAND
handling). `SafetyState.RETURN_TO_SAFE_POINT` maps to a velocity setpoint
toward the safe point (Phase 4's existing behavior, unchanged); the
adapter's own `request_return_to_launch()` is a separate, explicit
mode-request method for a future transport's native RTL mode, not
currently invoked by `mission.py`'s per-tick loop (which routes
RETURN_TO_SAFE_POINT as a velocity setpoint like any other tier).

**No implicit arm/disarm, no automatic takeoff, no real motor output** -
`MockAdapter` never sets `self.armed = True` anywhere in this phase; the
concept exists in `VehicleTelemetry`/`ConnectionState` for a future phase
to use, not exercised here.

## Direct vs. mock-adapter path

`MissionConfig.autopilot_path`:
- `"mock_adapter"` (default) - the architectural reference path required
  by this phase.
- `"direct"` - the pre-Phase-6 behavior (`safe_vel` reaches
  `SpeedController` unchanged after `SafetySupervisor`), kept only as a
  **temporary simulator-compatibility comparison baseline**. Verified
  empirically to produce identical results to `"mock_adapter"` under
  nominal conditions (zero configured adapter latency/loss) - see
  "Mock-adapter results" below - because the adapter's default
  configuration is a fully transparent pass-through when nothing is
  configured to degrade it.

## Mock adapter limitations

- No motor/ESC/aerodynamic model of any kind - it validates and forwards
  a velocity setpoint, nothing about how that setpoint would actually be
  tracked by real flight-controller PID loops.
- No real radio propagation, no MAVLink message framing/CRC, no packet
  fragmentation - latency and packet loss are both modeled as simple
  probabilistic/threshold arithmetic, not a simulated RF channel.
- `armed`/battery fields exist in the type system but are not driven by
  any real battery or arming logic in this phase (see "No implicit
  arm/disarm" above).
- Single-vehicle, single-vehicle-ID design (matches this project's
  per-drone `MockAdapter` instantiation in `mission.py` - one adapter per
  drone, not a multi-vehicle router).
- `read_telemetry()`/`send_command()`'s effective clock is entirely
  driven by caller-supplied timestamps (see "Clock model" above) - there
  is no independent adapter-side heartbeat/watchdog timer that would fire
  in the absence of any calls at all (a real transport's own
  connection-timeout detection is not modeled).

## ArduPilot skeleton limitations

`swarm_sim/autopilot/ardupilot.py`'s `ArduPilotAdapterSkeleton` is an
**offline skeleton only**: `connect()` always returns a rejected
`AdapterResult` and the adapter never leaves `ConnectionState.DISCONNECTED`,
which structurally prevents every other method (`send_command`,
`set_mode`, `request_land`, ...) from ever doing anything. There is no
MAVLink, no serial/UDP socket, no ArduPilot-specific message handling of
any kind - it exists only to prove the `AutopilotAdapter` interface CAN
be implemented for a future ArduPilot/MAVLink transport without touching
`SwarmController` or `SafetySupervisor`. A real transport would need to:
open and maintain a MAVLink connection, translate `AdapterCommand` into
`SET_POSITION_TARGET_LOCAL_NED`/`SET_POSITION_TARGET_GLOBAL_INT` messages,
poll `GLOBAL_POSITION_INT`/`ATTITUDE`/`SYS_STATUS`/`HEARTBEAT` for
telemetry, map `AutopilotMode.OFFBOARD` requests onto ArduPilot's
`GUIDED` flight mode (ArduPilot has no mode literally named "OFFBOARD" -
that's PX4's term), and implement real arm/disarm state tracking. None of
that exists here, by design, and is explicitly a later, separately
reviewed phase.

## PX4 skeleton limitations

`swarm_sim/autopilot/px4.py`'s `PX4AdapterSkeleton` is the same offline
skeleton, relabeled - `connect()` always fails, nothing here can reach a
real vehicle. A real transport would need MAVSDK (or PX4-native MAVLink),
PX4's own `OFFBOARD` mode (which happens to share this project's own
`AutopilotMode.OFFBOARD` name - convenient, not a design shortcut), and
real telemetry/arming plumbing - none of it implemented here, for the
same reason as the ArduPilot skeleton.

## Exact boundary between simulator and future transport

Everything from `SwarmController` through `AdapterResult.transformed_command`
is transport-agnostic - it is plain Python dataclasses and validation
logic with no dependency on PyBullet, MAVLink, or any specific vehicle.
The ONE place a future transport plugs in is a new class implementing
`AutopilotAdapter` (like `MockAdapter`, but backed by a real MAVLink/MAVSDK
connection instead of in-memory state) - `mission.py` would only need a
new `autopilot_path` value and adapter-construction branch to use it; no
change to `SwarmController`, `SafetySupervisor`, the sensor models, or
distributed consensus would be required. That is the entire point of this
phase's abstraction.

## Why this is not SITL

SITL (Software-In-The-Loop) means running the REAL ArduPilot or PX4
autopilot firmware binary, talking real MAVLink, against a physics
simulator standing in for the airframe. Nothing here runs ArduPilot or
PX4 firmware, opens a MAVLink connection (real or simulated transport),
or exercises the firmware's own flight-mode/failsafe/EKF logic - every
"mode transition" and "failsafe" in this phase is `MockAdapter`'s own
small Python state machine, standing in for what a real autopilot would
eventually do, not a rehearsal of the real thing. The Phase 6 request is
explicit that a SITL integration phase comes later, separately reviewed.

## Why this is not hardware-ready

No serial port or UDP socket is ever opened (see
`tests/test_autopilot_architecture.py`'s import checks - `pymavlink`,
`MAVSDK`, `serial`, `socket`, `pybullet`, and `gym_pybullet_drones` are
all absent from every file in `swarm_sim/autopilot/`), no arm/disarm
sequence exists, no real motor/PWM output is ever produced, and the
ArduPilot/PX4 classes cannot connect to anything by construction (see
their own "limitations" sections above). Real Pixhawk connection would
require explicit hardware safety procedures and a separately reviewed
phase, per this project's own stated plan.

## Mock-adapter results

`scripts/run_phase6_adapter_scenarios.py` (raw numbers:
`results/phase6_scenarios/phase6_scenario_results.json`, gitignored,
regenerate with the command below):

**Synthetic scenarios (1-12)** each isolate one required behavior -
`nominal_operation` (20/20 accepted), `command_expiry`/`command_latency`
(rejected as `expired`), `adapter_disconnect` (`disconnected`),
`stale_telemetry` (`stale_telemetry`, mode forced to HOLD),
`frame_mismatch` (one rejected unconverted, one accepted after explicit
`convert_command_frame`), `estimator_failure`/`operator_abort`/
`land_requested`/`return_to_safe_point` (HOLD/ABORT/LAND/RTL mode
transitions, all accepted), `repeated_command_replay` (idempotent replay
accepted, conflicting replay rejected), `packet_loss` (seeded
`random.Random(7)`, `command_packet_loss_prob=0.3` over 50 sends -
deterministic, reproducible reject count).

**Mission scenarios (13-15)**, all routed through the Phase 6 adapter
boundary (`autopilot_path="mock_adapter"`, the new default):

| scenario | adapter accepted/rejected | ground contacts | obstacle contacts | drone-drone | victims | replay |
|---|---|---|---|---|---|---|
| obstacle_wedging (Phase 4.1 exact config) | 12960/0 | 0 | 9 | 0 | 4/5 | identical |
| phase5_distributed_consensus (perfect comm) | 3000/0 | 0 | 0 | 0 | 1/4 | identical |
| extreme_comm_partition_radius_3m (seed 7, STRESS - see below) | 3000/0 | 175 | 1 | 0 | 1/4 | identical |

`obstacle_wedging`'s 0 ground / 9 obstacle contacts match Phase 4.1's and
Phase 5's own historical numbers exactly - the adapter boundary is a
fully transparent pass-through under nominal (zero configured
latency/loss) conditions, confirmed by 0 adapter rejections across every
mission scenario tested. This directly answers review checklist item 7
("the existing Phase 4.1 scenario still has zero ground contacts").

**`extreme_comm_partition_radius_3m` is a labeled NON-ACCEPTANCE STRESS
SCENARIO, not new to this phase** - see
`docs/PHASE5_DISTRIBUTED_CONSENSUS.md`'s "A pre-existing safety edge
case" section for the full 7-seed investigation and its
`known pre-existing Phase 4.1 safety limitation` /
`not a distributed-consensus defect` / `not a safe-flight result` labels,
which still apply unchanged. It is re-run here (one representative seed)
only to confirm the Phase 6 adapter boundary does not change its nature -
confirmed: rerunning `scripts/run_phase5_consensus_scenarios.py`'s own
`extreme_comm_partition_radius_3m` scenario after this phase's changes
reproduces the exact same 7-seed ground-contact numbers as before
(0, 0, 284/297, 97/6, 0, 0, 0/175 - centralized/distributed) - answering
review checklist item 8 (it remains documented, not silently removed).

## Reproducibility

- **Python**: 3.12.10 (see `README.md`'s `Setup` and
  `docs/PHASE5_DISTRIBUTED_CONSENSUS.md`'s "Reproducible environment" for
  the full PyBullet/gym-pybullet-drones environment - unchanged this
  phase).
- **Commit hash**: recorded automatically by
  `scripts/run_phase6_adapter_scenarios.py` in each scenario's
  `reproducibility.commit_hash` field (`git rev-parse HEAD`).
- **Command to run the full regression suite**:
  ```bash
  python -m compileall -q swarm_sim run_mission.py tests
  python -m pytest -q tests
  ```
- **Command to (re)generate every scenario number in this document**:
  ```bash
  python scripts/run_phase6_adapter_scenarios.py
  ```
- **Test count**: 60 new tests this phase (50 in `tests/test_autopilot.py`,
  9 in `tests/test_autopilot_architecture.py`), plus 1 new guarding test
  in `tests/test_safety_supervisor_architecture.py` - see "Tests" in the
  phase report.

## Remaining risks

- The mock adapter's clock model (derived entirely from caller-supplied
  timestamps, see "Clock model") has no independent watchdog for "the
  mission stopped calling send_command entirely" - a real transport would
  need its own heartbeat-timeout detection, not modeled here.
- `armed`/battery fields are present in the type system but inert in
  this phase (see "Mock adapter limitations") - a future phase must not
  assume they reflect anything real yet.
- BODY frame conversion's yaw-only simplification (roll/pitch ignored) is
  exact only for level flight - documented, not fixed, since none of
  this project's current command flows exercise significant roll/pitch
  at the moment of conversion.
- ArduPilot/PX4 skeletons are placeholders with zero real capability -
  filling them in is explicitly future, separately reviewed work.

## Explicit confirmation

No MAVLink library (pymavlink or otherwise), no MAVSDK, no SITL, no
Pixhawk/ArduPilot/PX4 connection, no serial port, no UDP socket, and no
hardware integration of any kind was installed, imported, opened, or
started in this phase. `swarm_sim/autopilot/ardupilot.py` and
`swarm_sim/autopilot/px4.py` are offline skeletons whose `connect()`
always fails by construction. Every executed scenario in this phase ran
against PyBullet, `MockAdapter` (deterministic and in-memory), or
deterministic offline adapter tests only.
