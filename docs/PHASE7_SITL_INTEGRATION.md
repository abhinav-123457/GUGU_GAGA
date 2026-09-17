# Phase 7: Local, Offline SITL Integration Boundary

A local, deterministic SITL *transport* boundary one layer below Phase 6's
`AutopilotAdapter` - so the swarm can later target a real ArduPilot SITL or
PX4 SITL process without changing `SwarmController`, `SafetySupervisor`,
or the `AutopilotAdapter` interface itself. Nothing in this phase connects
to a real Pixhawk, opens a real serial port or UDP socket, transmits
MAVLink, arms anything, or starts a real ArduPilot/PX4 SITL binary - see
"Why this is not SITL," "Why this is not hardware integration," and the
explicit confirmation at the end of this document.

**Actual ArduPilot/PX4 SITL was not run. Only the deterministic FakeSITL
transport was tested.**

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
   AdapterCommand  (Phase 6 - Command + monotonic sequence number)
        |
        v
   AutopilotAdapter.send_command()  (unchanged interface - Phase 6)
      /                |                  \
MockAdapter      SITLAdapter (NEW)     ArduPilot/PX4
(Phase 6,        (this phase)          adapter skeletons
 unchanged)            |               (Phase 6, unchanged)
                       v
              SITLCommand  (AdapterCommand + namespace + safety_decision_id)
                       |
                       v
              SITLTransport.send_command()   (swarm_sim/sitl/transport.py)
                  /                      \
        FakeSITLTransport          a future real ArduPilot/PX4 SITL
        (this phase - no            transport (NOT this phase - no such
         sockets, no serial,        class exists anywhere in this
         no MAVLink, deterministic, codebase; see
         in-memory, multi-vehicle)  autopilot/ardupilot_sitl.py,
                  |                 autopilot/px4_sitl.py, which remain
                  v                 pure offline OfflineSkeletonAdapter
     simulated telemetry (SITLTelemetry)   placeholders whose connect()
     and command acknowledgements          always fails)
     (CommandAck), read back via
     SITLAdapter.read_telemetry()
```

Every arrow above is a real, traceable call in the code, not aspirational.
`mission.py`'s `run()` still calls `SafetySupervisor.evaluate()` before
`_send_to_autopilot_adapter()` (Phase 6, unchanged - re-verified by
`tests/test_sitl_architecture.py`), which calls
`AutopilotAdapter.send_command()` - now, when `autopilot_path ==
"fake_sitl"`, that is `SITLAdapter.send_command()`, which itself builds a
`SITLCommand` and calls `SITLTransport.send_command()`
(`FakeSITLTransport` this phase). A raw `SwarmController` candidate
(`desired_vel`) still cannot reach any of this - see below.

**Why a raw candidate cannot reach the adapter or transport, structurally**:
`SITLAdapter.send_command()` accepts only an `AdapterCommand` (Phase 6's
own type, unchanged), which itself only ever wraps an already-validated
`contracts.Command`. `SITLCommand` (this phase, `swarm_sim/sitl/commands.py`)
in turn only ever wraps an already-validated `AdapterCommand` - there is no
constructor path from a bare numpy velocity array to a `SITLCommand`
without first building a real `Command` (which requires passing through
`SafetySupervisor.evaluate()` in the mission pipeline) and then a real
`AdapterCommand`. `FakeSITLTransport.send_command()` accepts only
`SITLCommand`. This is checked mechanically (not just by convention) in
`tests/test_sitl_architecture.py` and `tests/test_autopilot_architecture.py`.

## Files changed

- `swarm_sim/sitl/__init__.py`, `clock.py`, `vehicle_namespace.py`,
  `telemetry.py`, `commands.py`, `transport.py`, `fake_transport.py` (all
  new - the required package).
- `swarm_sim/autopilot/sitl.py` (new - `SITLAdapter`),
  `ardupilot_sitl.py`, `px4_sitl.py` (new - offline skeletons, same
  pattern as Phase 6's `ardupilot.py`/`px4.py`). Deliberately **not**
  re-exported from `swarm_sim/autopilot/__init__.py` - see "A genuine
  circular import, and how it was avoided" below.
- `swarm_sim/config.py` - `autopilot_path` gained a third value,
  `"fake_sitl"`; new `sitl_ack_timeout_s`, `sitl_future_tolerance_s`.
  Phase 6's `autopilot_command_latency_s`/`autopilot_telemetry_latency_s`/
  `autopilot_command_packet_loss_prob`/`autopilot_telemetry_packet_loss_prob`/
  `autopilot_telemetry_stale_timeout_s` are reused as-is by the fake-SITL
  path (not duplicated under a new name) for exactly the same concepts.
- `swarm_sim/mission.py` - `__init__` gained an `elif config.autopilot_path
  == "fake_sitl":` branch constructing one shared, multi-vehicle
  `FakeSITLTransport` plus one `SITLAdapter` per drone (`self.sitl_transport`,
  `self.autopilot_adapters`). **`_send_to_autopilot_adapter()` itself (the
  Phase 6 helper) is completely unchanged** - it only ever calls the
  `AutopilotAdapter` Protocol's own `send_command()`, so it cannot tell,
  and does not need to know, whether `self.autopilot_adapters[i]` is a
  `MockAdapter` or a `SITLAdapter`. `run()`'s per-drone loop gained one
  extra accepted value in a tuple check (`cfg.autopilot_path in
  ("mock_adapter", "fake_sitl")`). `SafetySupervisor`, the sensor models,
  the flocking equations, and distributed consensus are untouched.
- `tests/test_sitl.py` (68 tests), `tests/test_sitl_architecture.py`
  (13 tests) - 81 new tests this phase.
- `scripts/run_phase7_sitl_scenarios.py` (new).
- `docs/PHASE7_SITL_INTEGRATION.md` (this file).

## A genuine circular import, and how it was avoided

`swarm_sim.sitl.commands`/`telemetry` import `AdapterCommand`/`AutopilotMode`
from `swarm_sim.autopilot.types` (Phase 6's types, reused rather than
duplicated). If `swarm_sim/autopilot/__init__.py` also eagerly imported
`.sitl` (this phase's `SITLAdapter`), then importing `swarm_sim.sitl`
*before* `swarm_sim.autopilot` would recurse back into a still-initializing
`swarm_sim.sitl.commands` module - a genuine circular import, not just an
ordering nuisance. The fix: `SITLAdapter`/`ArduPilotSITLAdapterSkeleton`/
`PX4SITLAdapterSkeleton` are **not** re-exported from
`swarm_sim/autopilot/__init__.py` at all; callers import them directly
from their own submodule (`from swarm_sim.autopilot.sitl import
SITLAdapter`, exactly as `mission.py` does). Verified both import orders
(`import swarm_sim.autopilot` first, and `import swarm_sim.sitl` first)
work cleanly.

## Command / telemetry / acknowledgement schemas

**`SITLCommand`** (`swarm_sim/sitl/commands.py`) - `adapter_command:
AdapterCommand` (carries vehicle_id, sequence, command_type, frame,
position/velocity setpoint, yaw/yaw-rate, timestamp, expiry, source via
its own properties, unchanged from Phase 6), plus this phase's two
additions: `namespace: str` and `safety_decision_id: str`.

**`safety_decision_id`** is a deterministic string derived purely from
already-public fields (`derive_safety_decision_id(vehicle_id, sim_time_s,
filtered_command)` - a SHA-256 digest of vehicle_id/timestamp/command
content). `contracts.SafetyDecision` itself gained **no new field** -
modifying it (or `safety_supervisor.py`) was explicitly out of scope per
the Phase 5 reviewer's carried-forward instruction, so the id is computed
at the SITL boundary instead of stored upstream.

**`SITLTelemetry`** (`swarm_sim/sitl/telemetry.py`) - vehicle_id,
namespace, timestamp_s, sequence, position_m, velocity_mps, attitude_rad,
angular_velocity_radps, battery_fraction, estimator_valid, armed,
flight_mode (`AutopilotMode`), failsafe, heartbeat_ok,
last_command_ack_status. Richer than Phase 6's `VehicleTelemetry` because
namespace/heartbeat/ack-status are SITL-transport-specific concepts;
`SITLAdapter.read_telemetry()` is the only place a `SITLTelemetry` is
translated into a Phase 6 `VehicleTelemetry` for the rest of the swarm.

**`CommandAck`** (`swarm_sim/sitl/telemetry.py`) - vehicle_id, namespace,
command_sequence, accepted, reason, timestamp_s, autopilot_mode, failsafe.
This is the vehicle's own acceptance/rejection of a command's *content*
(frame, sequence, failsafe state) - distinct from...

**`TransportResult`** (`swarm_sim/sitl/telemetry.py`) - success, reason,
timestamp_s. This is the *transport hand-off's* own outcome
(`start()`/`stop()`/`send_command()`'s immediate return) - a
`TransportResult` can be `success=False` (`"transport_not_started"`,
`"unknown_vehicle_id"`, `"namespace_mismatch"`) even before a command ever
reaches a vehicle's own validation logic and produces a `CommandAck`. This
two-tier split (transport-level routing failure vs. vehicle-level command
rejection) mirrors how a real message bus and a real flight controller are
two separate things that can each independently refuse a command.

## Clock model

`swarm_sim/sitl/clock.py`'s `SimClock` is the only time source anywhere in
`swarm_sim/sitl/` - no `time.time()`, no `time.monotonic()`, no hidden
wall-clock read anywhere (checked by AST in
`tests/test_sitl_architecture.py`). Two modes, both plain float
arithmetic:

- **Fixed-step**: `SimClock.advance(dt_s)` (used by
  `FakeSITLTransport.step(dt_s)` for the standalone synthetic scenarios
  that have no real physics behind them).
- **Manually advanced**: `SimClock.set(now_s)` (used by
  `FakeSITLTransport.push_vehicle_state(...)`, called from
  `SITLAdapter.push_ground_truth_state()` in the PyBullet-integrated path -
  the clock jumps straight to the mission's own current `t` every tick).

Both raise `ValueError` on an attempted decrease - the clock is
monotonic, not just "usually moves forward." `is_from_the_future(timestamp_s)`
rejects a command timestamped further ahead of `now_s` than a configured
`future_tolerance_s` (default 0.05s - reused for the exact-boundary
expiry test the same way `Command.is_expired`'s own boundary convention
does: expired *at*, and after, the expiration timestamp, never a grace
period). Command timestamps/expiry/sequence numbers are preserved
unmodified through `SITLAdapter` and `FakeSITLTransport` - nothing in this
phase's chain calls `dataclasses.replace` on a command's temporal fields
(only Phase 6's `frames.py` ever does, for frame conversion, unchanged
this phase).

## Vehicle namespace model

`swarm_sim/sitl/vehicle_namespace.py`'s `VehicleNamespaceRegistry` is the
structural guarantee behind "commands cannot cross vehicle namespaces":
`namespace_for(vehicle_id) = f"sitl/{vehicle_id}"`, registered exactly
once per vehicle_id at `FakeSITLTransport.__init__` time. Two vehicles
attempting to register the same vehicle_id (the collision model this phase
implements - see "namespace collision attempt" below) raises
`NamespaceError` at **construction time**, before any command/telemetry
ever flows - not discovered later as cross-talk.

`FakeSITLTransport` hosts **one shared, multi-vehicle instance**, not one
transport per vehicle - deliberately, because "drone0 commands cannot
affect drone1," "telemetry from drone1 cannot satisfy drone0's ack," and
"failures injected into one vehicle do not alter another" are only
meaningful guarantees against a transport that actually holds multiple
vehicles' state at once and could, if built carelessly, let it leak. Every
piece of per-vehicle state lives in its own `_VehicleChannel`, looked up
strictly by vehicle_id/namespace; `send_command()` calls
`VehicleNamespaceRegistry.require_match(vehicle_id, namespace)` before
touching any channel state at all (checked at the AST level too - see
`test_fake_sitl_transport_send_command_checks_namespace_before_touching_channel_state`).
A forged `SITLCommand` whose `namespace` doesn't match its own
`vehicle_id`'s registered namespace is rejected
(`TransportResult(success=False, reason="namespace_mismatch")`) before any
per-vehicle evaluation runs.

**Distributed-consensus messages cannot be confused with autopilot
messages**: `swarm_sim.distributed_consensus` has zero references to any
`sitl`/`autopilot` identifier or import (re-verified this phase in
`test_distributed_consensus_never_references_sitl_or_adapter_identifiers`,
extending Phase 6's own equivalent check) - `CommsNetwork`'s message
inbox/outbox and `FakeSITLTransport`'s per-vehicle command/telemetry/ack
histories are entirely separate objects with no shared queue or storage a
message could leak through.

## Sequence-number semantics

Identical policy to Phase 6's `MockAdapter`, now enforced inside
`FakeSITLTransport._evaluate_command` (per vehicle channel) instead of
inside the adapter itself:

- `sequence > last_accepted`: normal case, accepted (subject to every
  other check).
- `sequence == last_accepted` **and identical content**: idempotent
  replay - the exact prior `CommandAck` is returned again, not
  reprocessed.
- `sequence == last_accepted` **with different content**: rejected,
  `"sequence_replayed_conflicting"` (this is also Phase 7's "duplicate
  command" test case: same sequence, different content, must never be
  silently accepted as if idempotent).
- `sequence < last_accepted`: rejected, `"sequence_replayed_stale"`
  regardless of content.

Tracked independently per `_VehicleChannel` - two vehicles can be at
completely different sequence numbers on the same shared transport with
no interference (see "namespace isolation" tests).

## Frame conventions

Unchanged from Phase 6 - `swarm_sim/autopilot/frames.py`'s
`convert_command_frame`/`convert_vector_frame` are the only frame
conversion in this codebase, and this phase does not touch them.
`FakeSITLTransport` rejects (`"frame_mismatch"`) any `SITLCommand` whose
frame doesn't match its configured `operating_frame` - exactly like
`MockAdapter`, never a silent conversion. See
`docs/PHASE6_AUTOPILOT_ADAPTERS.md`'s "Frame conventions" section for the
full ENU/NED/BODY_FRD axis/sign math (LOCAL_ENU/LOCAL_NED fixed
relabeling; BODY_FRD is a yaw-only rotation, refused outright without
`vehicle_attitude_rad`).

## Telemetry validity / stale-telemetry policy

A vehicle channel's own ground-truth state is only ever refreshed by
`FakeSITLTransport.step()` (standalone scenarios) or
`push_vehicle_state()` (PyBullet-integrated path, called from
`SITLAdapter.push_ground_truth_state()`) - both go through the shared
`_emit_telemetry()` helper, which records `_last_state_update_at_s`. A
**navigation** command (POSITION/VELOCITY/YAW/YAW_RATE setpoint) is
rejected (`"stale_telemetry"`, mode forced to HOLD) if the gap since that
last refresh exceeds `telemetry_stale_timeout_s` (default 1.0s) - "never
refreshed at all" counts as stale, matching Phase 6's own
`MockAdapter._telemetry_is_stale` convention exactly. Non-navigation
commands (HOLD/LAND/ABORT) are never blocked by staleness. In the
PyBullet-integrated path this never fires under nominal configuration,
because `_send_to_autopilot_adapter()` pushes fresh telemetry every single
tick immediately before calling `send_command()` - verified empirically
(zero `stale_telemetry` rejections in the full-mission scenarios below).

## Failsafe policy (as implemented in `FakeSITLTransport._evaluate_command`, in order)

1. Command timestamped from the future (beyond `future_tolerance_s`) ->
   rejected (`"command_from_the_future"`).
2. `FailureType.VEHICLE_DISCONNECT` active -> rejected (`"disconnected"`) -
   no implicit reconnection.
3. Frame mismatch -> rejected (`"frame_mismatch"`), never silently
   converted.
4. Expired (after `command_latency_s`) -> rejected (`"expired"`).
5. Navigation command + `HEARTBEAT_LOSS`/`ESTIMATOR_INVALID`/
   `BATTERY_CRITICAL` active -> rejected (`"heartbeat_lost"` /
   `"estimator_invalid"` / `"battery_critical"`), mode forced to HOLD.
6. Navigation command + stale telemetry -> rejected (`"stale_telemetry"`),
   mode forced to HOLD.
7. Sequence replay -> rejected (`"sequence_replayed_conflicting"` /
   `"sequence_replayed_stale"`), or accepted idempotently.
8. Command packet loss (`FailureType.COMMAND_PACKET_LOSS` active, or the
   configured probability) -> rejected (`"command_packet_loss"`).
9. Otherwise accepted; `ABORT`/`LAND`/`HOLD` set the channel's mode
   directly (LAND additionally zeroes velocity - the "simplified
   kinematics" the FakeSITL implements, see below); everything else
   requests `OFFBOARD`.

**Operator abort priority**: unchanged from Phase 6 - `SafetySupervisor`'s
tier-1 operator-abort handling already guarantees an abort reaches this
boundary as a `CommandType.ABORT` `Command` ahead of every other
candidate, every tick; this phase's transport only has to honor `ABORT`
correctly once it arrives, which it does unconditionally on acceptance.

**LAND_REQUESTED / RETURN_TO_SAFE_POINT**: identical mapping to Phase 6
(`SafetyState.LAND_REQUESTED` -> `CommandType.LAND` -> `AutopilotMode.LAND`,
explicitly simulated only; `RETURN_TO_SAFE_POINT` -> a velocity setpoint
toward the safe point, unchanged Phase 4 behavior) - now additionally
validated by `FakeSITLTransport` on the way through, same as any other
command.

**Heartbeat behavior**: `SITLTelemetry.heartbeat_ok` is `True` unless
`FailureType.HEARTBEAT_LOSS` is active for that vehicle
(`inject_failure(vehicle_id, HEARTBEAT_LOSS)`) - modeled as a discrete,
injectable failure rather than a real periodic HEARTBEAT-message timeout,
since there is no real MAVLink heartbeat message anywhere in this phase
(see `MAVLINK_HEARTBEAT_MAPPING_SPEC` in `sitl/telemetry.py` for the
naming a future real transport would map this onto).

**Battery/estimator behavior**: `battery_fraction` decays at a
configurable `battery_drain_per_s` only under `step()`'s own simplified
kinematics mode (never under `push_vehicle_state`, where the real
mission's own battery figure is authoritative); crossing
`battery_critical_threshold` auto-injects `FailureType.BATTERY_CRITICAL`.
`estimator_valid` is `False` whenever either the pushed/stepped state says
so, or `FailureType.ESTIMATOR_INVALID` is injected - either source alone
is sufficient.

**No implicit arm, no automatic takeoff, no real motor output** -
`_VehicleChannel.armed` is initialized to `False` and never assigned
`True` anywhere in `swarm_sim/sitl/` (checked by AST -
`test_fake_sitl_transport_never_sets_armed_true` - not just by
convention).

## Simplified kinematics ("fake-SITL limitations")

`FakeSITLTransport.step(dt_s)` integrates **only** the last accepted
`VELOCITY_SETPOINT`'s velocity via plain Euler integration
(`position += velocity * dt`) - used only by the standalone synthetic
scenarios that have no real physics engine behind them.
`push_vehicle_state(...)` (the PyBullet-integrated path's own hook,
called every tick from `SITLAdapter.push_ground_truth_state()`) overwrites
a channel's ground-truth state directly from real PyBullet physics,
bypassing `step()`'s integrator entirely - the two modes are never
combined for one vehicle in a real run. **This is explicitly not a flight-
dynamics model**: no attitude response to a commanded velocity, no
motor/propeller/airframe dynamics, no PID/controller stabilization loop of
any kind - a real ArduPilot/PX4 SITL process would run the actual firmware
against a physics-accurate airframe model, which is precisely what makes
it SITL and this fake-SITL not.

## Direct / mock-adapter / fake-SITL paths

`MissionConfig.autopilot_path`:
- `"mock_adapter"` (default, unchanged from Phase 6) - one `MockAdapter`
  per drone, no `SITLTransport` involved at all.
- `"fake_sitl"` (this phase) - one shared `FakeSITLTransport` plus one
  `SITLAdapter` per drone, adding the transport layer described above.
- `"direct"` (pre-Phase-6, unchanged) - kept only as a temporary
  simulator-compatibility comparison baseline.

Both `"mock_adapter"` and `"fake_sitl"` are empirically confirmed
transparent pass-throughs under nominal (zero configured latency/loss)
conditions - see "Full-mission scenario results" below, which reproduces
Phase 4.1's and Phase 5's exact historical numbers unchanged under
`"fake_sitl"` specifically.

## Why this is not SITL

SITL (Software-In-The-Loop) means running the REAL ArduPilot or PX4
autopilot firmware binary, talking real MAVLink, against a physics
simulator standing in for the airframe. `FakeSITLTransport` is a plain
Python class with an in-memory dict of vehicle channels - it does not
launch a process, does not speak MAVLink, does not run any autopilot
firmware's own EKF/flight-mode/failsafe logic. Every "failsafe" and "mode
transition" in this phase is this transport's own small, deterministic
Python state machine standing in for what a real autopilot would
eventually do - not a rehearsal of the real thing. **Actual ArduPilot/PX4
SITL was not run in this phase; only the deterministic FakeSITL transport
was tested**, exactly as instructed.

## Why this is not hardware integration

No serial port or UDP socket is ever opened; no `pymavlink`, `MAVSDK`,
`serial`, `socket`, `subprocess`, or `asyncio` import exists anywhere in
`swarm_sim/sitl/` or the new `autopilot/sitl.py`/`ardupilot_sitl.py`/
`px4_sitl.py` (checked by AST in `tests/test_sitl_architecture.py`, both
a package-wide check and a stricter zero-imports-beyond-this-project
check on the two named skeletons - the same pattern Phase 6 established).
No arm/disarm sequence exists (`armed` is never assigned `True`
anywhere), no real motor/PWM output is ever produced, and
`ArduPilotSITLAdapterSkeleton`/`PX4SITLAdapterSkeleton` cannot connect to
anything by construction (`OfflineSkeletonAdapter.connect()` always
fails, unchanged from Phase 6). A future ArduPilot/PX4 SITL transport
would implement `SITLTransport` for real (launching/attaching to a local
`arducopter`/`px4` SITL binary over loopback MAVLink) - no such class
exists anywhere in this codebase. Real Pixhawk connection requires
explicit hardware safety procedures and a separately reviewed phase, per
this project's own stated plan.

## Future ArduPilot SITL mapping (specification only)

`swarm_sim/sitl/commands.py`'s `MAVLINK_FRAME_MAPPING_SPEC`,
`MAVLINK_COMMAND_TYPE_MAPPING_SPEC`, `MAVLINK_MODE_NAME_MAPPING_SPEC`, and
`swarm_sim/sitl/telemetry.py`'s `MAVLINK_HEARTBEAT_MAPPING_SPEC`,
`MAVLINK_COMMAND_ACK_MAPPING_SPEC`, `MAVLINK_FAILSAFE_MAPPING_SPEC` are
plain Python dicts naming what a future ArduPilot SITL transport would map
this project's own vocabulary onto: `LOCAL_NED` commands ->
`SET_POSITION_TARGET_LOCAL_NED`; `HOLD`/`RTL`/`LAND` -> ArduPilot's
`LOITER`/`RTL`/`LAND` flight modes (`OFFBOARD` maps to ArduPilot's
`GUIDED` - ArduPilot has no mode literally named "OFFBOARD", that is PX4's
term, unchanged from Phase 6's own note); `ABORT` -> `MAV_CMD_DO_
FLIGHTTERMINATION` (simulated only, never a real command). **These are
mapping specifications only - plain dicts, no pymavlink import, nothing
here parses or emits an actual MAVLink message.**

## Future PX4 SITL mapping (specification only)

Same tables, PX4 column: `OFFBOARD` maps directly onto PX4's own
`OFFBOARD` mode (a naming coincidence noted, not a design shortcut, same
as Phase 6); `RTL`/`LAND` -> PX4's `AUTO.RTL`/`AUTO.LAND`; `HOLD` -> PX4's
own `HOLD` mode. A real PX4 SITL transport would use MAVSDK (or PX4-native
MAVLink) instead of pymavlink - `PX4SITLAdapterSkeleton` exists only to
prove `AutopilotAdapter` is implementable for that future transport
without touching `SwarmController`/`SafetySupervisor`.

## Full-mission scenario results

`scripts/run_phase7_sitl_scenarios.py` (raw numbers:
`results/phase7_sitl_scenarios/phase7_scenario_results.json`, gitignored,
regenerate with the command in "Reproducibility"):

**Synthetic scenarios (1-17)** each isolate one required behavior against
`FakeSITLTransport`/`SITLAdapter` directly (no PyBullet): one- and
six-vehicle nominal operation (all accepted, zero namespace-isolation
violations), command/telemetry latency, command/telemetry packet loss,
heartbeat loss, estimator failure, battery critical, vehicle disconnect,
transport stop, namespace collision attempt (raises `NamespaceError` at
construction, not silently accepted), duplicate command (idempotent replay
accepted, conflicting replay rejected), sequence replay (stale sequence
rejected), operator abort / LAND_REQUESTED / RETURN_TO_SAFE_POINT (mode
transitions verified directly).

**Mission scenarios (18-20)**, all routed through the Phase 7 fake-SITL
boundary (`autopilot_path="fake_sitl"`):

| scenario | adapter accepted/rejected | ground contacts | obstacle contacts | drone-drone | victims | namespace violations | replay |
|---|---|---|---|---|---|---|---|
| delayed_distributed_consensus (comm_latency_steps=4, radius=10) | 3000/0 | 0 | 0 | 0 | 1/4 | 0 | identical |
| obstacle_wedging (Phase 4.1 exact config) | 12960/0 | 0 | 9 | 0 | 4/5 | 0 | identical |
| extreme_comm_partition_radius_3m (seed 7, STRESS - see below) | 3000/0 | 175 | 1 | 0 | 1/4 | 0 | identical |

`obstacle_wedging`'s 0 ground / 9 obstacle contacts match Phase 4.1's,
Phase 5's, and Phase 6's own historical numbers exactly under
`"fake_sitl"` too - confirmed by zero adapter rejections across the
mission scenarios, directly answering acceptance criterion "Phase 4.1
remains at zero ground contacts."

**`extreme_comm_partition_radius_3m` is a labeled NON-ACCEPTANCE STRESS
SCENARIO, not new to this phase** - see
`docs/PHASE5_DISTRIBUTED_CONSENSUS.md`'s "A pre-existing safety edge case"
section for the full 7-seed investigation and its
`known pre-existing Phase 4.1 safety limitation` /
`not a distributed-consensus defect` / `not a safe-flight result` labels,
which still apply unchanged. Re-run here (one representative seed) only to
confirm the Phase 7 fake-SITL boundary does not change its nature -
confirmed: identical numbers to Phase 5's and Phase 6's own runs -
answering acceptance criterion "the 3m stress scenario remains present and
labeled."

## Reproducibility

- **Python**: 3.12.10 (unchanged environment - see
  `docs/PHASE5_DISTRIBUTED_CONSENSUS.md`'s "Reproducible environment").
- **Commit hash**: recorded automatically by
  `scripts/run_phase7_sitl_scenarios.py` in each scenario's
  `reproducibility.commit_hash` field (`git rev-parse HEAD`).
- **Random seeds**: one fixed seed per synthetic scenario (1-17, recorded
  in each scenario's own `seed` field); mission scenarios reuse Phase
  4.1/5/6's own seeds (42 for `obstacle_wedging`, 7 for
  `extreme_comm_partition_radius_3m`, 7 for `delayed_distributed_consensus`).
- **Command to run the full regression suite**:
  ```bash
  python -m compileall -q swarm_sim run_mission.py tests
  python -m pytest -q tests
  ```
- **Command to (re)generate every scenario number in this document**:
  ```bash
  python scripts/run_phase7_sitl_scenarios.py
  ```
- **Test count**: 81 new tests this phase (68 in `tests/test_sitl.py`, 13
  in `tests/test_sitl_architecture.py`) - see the phase report for the
  full-suite total.

## Remaining risks

- `FakeSITLTransport`'s clock has no independent watchdog for "the
  mission stopped calling anything entirely" beyond the per-command
  staleness/ack-timeout checks already described - a real transport would
  need its own connection-level heartbeat-timeout detection.
- `armed`/battery fields exist in the schema but battery decay is only
  modeled under `step()`'s own simplified kinematics mode, never under the
  PyBullet-integrated `push_vehicle_state` path (where the mission's real
  state, whatever it tracks, is authoritative) - a future phase must not
  assume `FakeSITLTransport` itself models battery physics.
- `SITLAdapter.set_mode()`/`request_land()`/`request_return_to_launch()`/
  `abort()` operate directly on the adapter's own state without routing
  through the transport (same design Phase 6's `MockAdapter` used) -
  `mission.py`'s per-tick loop never calls these directly today (only
  `send_command`), so this is a documented limitation, not an exercised
  gap.
- The FakeSITL's simplified kinematics model only integrates
  `VELOCITY_SETPOINT` commands - `POSITION_SETPOINT` is stored but not
  separately tracked toward a target (out of scope; `mission.py` only
  ever issues velocity setpoints).
- ArduPilot/PX4 SITL skeletons remain pure placeholders with zero real
  transport capability - filling them in with an actual local SITL
  process is explicitly the next, separately reviewed phase.

## Explicit confirmation

No real ArduPilot SITL, no real PX4 SITL, no MAVLink library (pymavlink or
otherwise), no MAVSDK, no Pixhawk/ArduPilot/PX4 hardware connection, no
serial port, no UDP socket, and no hardware-in-the-loop integration of any
kind was installed, imported, opened, or started in this phase.
`swarm_sim/autopilot/ardupilot_sitl.py` and `swarm_sim/autopilot/px4_sitl.py`
are offline skeletons whose `connect()` always fails by construction.
Every executed scenario in this phase ran against `FakeSITLTransport`
(deterministic, in-memory, no sockets) directly, `SITLAdapter` backed by
`FakeSITLTransport`, or PyBullet via the same `FakeSITLTransport`-backed
path - never a real SITL binary.
