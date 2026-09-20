# Phase 14: One-Drone Simulated SAR Mission Prototype

    This is a one-drone simulated SAR mission prototype - not an
    operational search-and-rescue system. Webots provides simulated
    physics and sensors. ArduPilot SITL provides autopilot control. The
    swarm planner is not yet integrated. No Pixhawk, serial port, radio,
    real aircraft, outdoor vehicle, motor/PWM hardware, or
    hardware-in-the-loop was used.

## Architecture

```
SAR mission planner (swarm_sim.sar_mission.SARMission)
    -> sensor/mission belief layer (swarm_sim.sar_world.SARWorld -> SensorObservation)
    -> SafetySupervisor.evaluate()          (swarm_sim.safety_supervisor, reused unmodified)
    -> validated Command (SafetyDecision.filtered_command)
    -> AdapterCommand
    -> ArduPilot SITL (via SITLAdapter, offline: FakeSITLTransport / live: ArduPilotSITLTransport)
    -> Webots physics and simulated sensors (live mode only)
    -> telemetry/events
    -> SAR planner (next tick)
```

One Webots Iris vehicle only. ArduPilot SITL remains the autopilot
authority. Webots remains the vehicle-physics and sensor authority (live
mode). `SafetySupervisor` remains the final command authority for every
movement command - see "Safety/command path" below.

Files:

| File | Role |
|---|---|
| `swarm_sim/sar_world.py` | Hidden victim ground truth + `RectangularSearchArea` + sensor-observation generation (reuses `swarm_sim.sensors.VictimSensorModel` unmodified) |
| `swarm_sim/sar_search.py` | Deterministic boustrophedon/lawnmower waypoint generator + waypoint follower (no ground truth access) |
| `swarm_sim/sar_mission.py` | `MissionState` enum, `SARMissionConfig`, `SARMission` state machine, `run_sar_mission_offline()` |
| `scripts/run_phase14_sar_mission.py` | CLI: `--dry-run` (default) / `--offline` / live (`--webots-only --allow-webots-arm-test --confirm-webots-flight-test`) |
| `tests/test_phase14_sar_mission.py` | Functional tests (state machine, search pattern, sensor scenarios, safety/failure handling, reproducibility) |
| `tests/test_phase14_sar_architecture.py` | AST structural checks |

**Reuse, not duplication** (see `tests/test_phase14_sar_architecture.py`):
`SafetySupervisor`, `swarm_sim.contracts` types, `swarm_sim.sensors.VictimSensorModel`,
`swarm_sim.manifest.build_run_manifest`, `swarm_sim.seeding.SeedManager`,
`swarm_sim.sitl.fake_transport.FakeSITLTransport`, `swarm_sim.autopilot.sitl.SITLAdapter`,
and (live mode only) `swarm_sim.autopilot.ardupilot_sitl.build_ardupilot_sitl_adapter`
are all reused unmodified. The live script imports Phase 12's Webots-detection
helpers (`_detect_webots_installation`, `_official_example_paths`,
`_port_appears_bound`, `_webots_controller_port`) and Phase 13's arm/takeoff/
land/gate helpers (`_check_arming_check_gate`, `_send_command_long_and_wait_ack`,
`_set_mode_guided`, `_request_servo_output_stream`, `_ActuatorLogger`,
`_classify_altitude_result`) and Phase 10's `_check_geofence_gate`/
`_collect_statustexts`/`_read_params` directly - it never redefines any of
them, and never imports `run_phase13_webots_flight_test`'s own
`run_webots_flight_test`/`main`. Phase 10/12/13's own files are untouched
(`tests/test_phase14_sar_architecture.py`'s regression checks confirm
this) - **Phase 13's dedicated manual SITL flight-test path was not
modified.**

**No swarm planner integration yet**: `swarm_sim.mission.FloodSearchMission`,
`RecruitmentBoard`, `ConsensusBoard`, and `distributed_consensus` are never
imported - a single vehicle has no distinct drones to recruit or reach
consensus with (see the Phase 14 research note in project history for the
full justification). No PyBullet, no FlightGear.

## Mission states

```
INIT -> PREFLIGHT_CHECK -> TAKEOFF_REQUESTED -> TRANSIT -> SEARCHING
  -> DETECTION_CANDIDATE -> DETECTION_CONFIRMED -> RETURN_HOME
  -> LAND_REQUESTED -> LANDED
(any non-terminal state) -> FAILED | ABORTED
```

Every transition is declared in `swarm_sim.sar_mission.ALLOWED_TRANSITIONS`
and enforced by `SARMission._transition()`, which **raises** on any
transition not in that table - "document every state transition" is a
structural guarantee, not just a comment
(`tests/test_phase14_sar_mission.py::TestMissionStates::test_illegal_transition_raises`).

| From | To | Trigger |
|---|---|---|
| INIT | PREFLIGHT_CHECK | mission started |
| PREFLIGHT_CHECK | TAKEOFF_REQUESTED | position + estimator valid |
| PREFLIGHT_CHECK | FAILED | no position or invalid estimator |
| TAKEOFF_REQUESTED | TRANSIT | search altitude reached |
| TRANSIT | SEARCHING | first waypoint reached |
| SEARCHING | DETECTION_CANDIDATE | a detection observed this tick |
| DETECTION_CANDIDATE | SEARCHING | no detection repeated next tick |
| SEARCHING / DETECTION_CANDIDATE | DETECTION_CONFIRMED | a cluster reached `confirmation_min_detections` |
| DETECTION_CONFIRMED | RETURN_HOME | confirmation scored (real or false) |
| SEARCHING / DETECTION_CANDIDATE | RETURN_HOME | all waypoints exhausted, no confirmation |
| (any active state) | RETURN_HOME | `SafetySupervisor` reports LOW_BATTERY / RETURN_TO_SAFE_POINT / GEOFENCE_RISK |
| RETURN_HOME | LAND_REQUESTED | home position reached |
| (any active state) | LAND_REQUESTED | mission timeout (emergency, skips RETURN_HOME - see "Limits"), or `SafetySupervisor` LAND_REQUESTED |
| LAND_REQUESTED | LANDED | telemetry shows ground altitude + near-zero velocity |
| (any non-terminal state) | FAILED | estimator/heartbeat/stale-telemetry persisting past its grace period, landing timeout exceeded, command timeout (persistent SafetySupervisor rejection), no position telemetry |
| (any non-terminal state) | ABORTED | `SafetySupervisor` ABORT state |

**LANDED vs. FAILED, by design**: a mission that returns home early (low
battery, geofence risk, or a hard mission timeout) and then lands
successfully still ends in **LANDED**, not FAILED - "the mission ended
early for reason X" is recorded in `mission_events.jsonl`/`summary.json`,
not conflated with "something actually went wrong". FAILED is reserved
for a genuine fault: telemetry/estimator/link problems that persisted, a
landing that never completed, or a persistently-rejected candidate.

## Search pattern

Deterministic boustrophedon/lawnmower coverage
(`swarm_sim.sar_search.generate_lawnmower_waypoints`): parallel lanes
along x, alternating direction, spaced `lane_spacing_m` apart in y, every
waypoint clamped into `[area.min + margin, area.max - margin]` - a subset
of the search area by construction, so waypoints can never lie outside
the geofence built from that same area
(`tests/test_phase14_sar_mission.py::TestSearchPattern`). No obstacle
avoidance, no swarm coordination, no optimization claim, and no guarantee
of victim detection - a fixed, bounded coverage path, nothing more.
`LawnmowerSearchPattern` is a plain index-based follower
(`advance_if_reached`/`current_waypoint`/`is_complete`), and
`velocity_toward()` is the only guidance law used - a proportional vector
toward the current target capped at `max_speed_mps`.

## Sensor model / detection confirmation

`SARWorld` reuses `swarm_sim.sensors.VictimSensorModel` unmodified
(constructed with `num_drones=1`, no obstacles). Each tick's
`SensorObservation` carries FOV/range-limited, occlusion-gated, Gaussian-
noised `DetectionCandidate`s, with false negatives, false positives,
whole-observation dropout, and configurable latency - see
`swarm_sim/sensors.py`'s own docstring for the full model. Verified
scenarios (`tests/test_phase14_sar_mission.py::TestSensorAndDetectionScenarios`):
perfect detection, noisy detection, missed detection, false positive,
delayed observation (via `victim_sensor_latency_steps`), no victim found,
duplicate observation, and same-seed determinism.

**Confirmation rule** (`SARMission.ingest_observation`): detections above
`confirmation_min_confidence` are greedily clustered by proximity
(within `confirmation_cluster_radius_m` of a cluster's running centroid);
a cluster is CONFIRMED once it accumulates `confirmation_min_detections`
distinct (non-duplicate) members. This mission always returns home after
its *first* confirmation - a deliberate, honestly-scoped simplification
for this one-drone prototype's first scenario (one search area, one or a
few victims, one confirmed report), not a multi-victim tour. Duplicate
`candidate_id`s are ignored and counted (`duplicate_observations`);
observations older than `stale_observation_max_age_s` are rejected and
counted (`stale_observations_rejected`), never used.

## Hidden-truth isolation

`SARWorld._victims_gt` (a list of `contracts.VictimGroundTruth`) is the
only place a real victim position exists. `sar_search.py` never imports
`sar_world.py`'s private attributes at all
(`tests/test_phase14_sar_architecture.py::test_sar_search_never_imports_sar_world_private_or_ground_truth_names`).
`SARWorld`'s only planner-facing method, `observe()`, returns a
`contracts.SensorObservation` - a `DetectionCandidate` carries a
`candidate_id`/position/confidence/uncertainty, never a link back to
which (if any) real victim it came from, and false positives are
assigned IDs from the exact same counter as genuine detections (reused
directly from `VictimSensorModel`'s own existing guarantee). The only
other `SARWorld` method the mission layer calls, `mark_found_near(xy,
radius)`, takes the mission's own **believed** (sensor-reported) position
- never ground truth - and returns only a victim_id or `None`; this is
the one place a false confirmation is detected, and it happens entirely
inside the world/scoring layer, never inside the search/planner code
(`tests/test_phase14_sar_mission.py::TestHiddenTruthIsolation`).

## Safety/command path

```
SARMission builds a raw CandidateCommand (velocity toward the current target)
    -> SafetySupervisor.evaluate()   <- ALWAYS runs, every tick, before anything is sent
    -> SafetyDecision.filtered_command (or accepted=False, nothing sent)
    -> AdapterCommand(command=decision.filtered_command, ...)   <- never a raw candidate
    -> SITLAdapter.send_command()
```

`SafetySupervisor` remains the final command authority: its own tiered
checks (geofence, altitude, battery, estimator, operator-abort) run on
**every** tick regardless of mission state, using the vehicle's real
current telemetry - `SARMission._apply_safety_emergency_state()` only
maps the supervisor's own resulting `emergency_state` onto a **mission**-
level state transition (e.g. `LOW_BATTERY`/`GEOFENCE_RISK` -> mission
RETURN_HOME); it never second-guesses or overrides the supervisor's own
already-safe `filtered_command`. `SARMission.tick()` never calls
`send_command()` itself - it returns a `TickResult`, and the caller
(`run_sar_mission_offline`, or the live script) sends it and reports the
result back via `record_command_result()`
(`tests/test_phase14_sar_architecture.py::test_tick_never_calls_send_command_itself`).

Additional mission-level checks the generic `SafetySupervisor` does not
itself model, layered on top (never replacing it): estimator-invalid/
heartbeat-loss/stale-telemetry persistence grace periods, mission
timeout, landing timeout, and command timeout (persistent rejection) -
"never silently continue" after any of these; every one produces a
logged, explicit state transition, never a quiet retry loop.

## Webots/SITL setup (live mode - not yet run this phase)

Same official ArduPilot Webots Python integration Phase 12/13 verified
(`libraries/SITL/examples/Webots_Python/`), same direct-`arducopter`
launch pattern (not `sim_vehicle.py`) Phase 13 established, run from the
same verified WSL2 checkout:

```bash
# SITL:
cd /home/swarmbuild/ardupilot
./build/sitl/bin/arducopter -S -w --model webots-python -I0 \
    --defaults Tools/autotest/default_params/copter.parm,libraries/SITL/examples/Webots_Python/params/iris.parm,/mnt/c/Users/<you>/.../swarm/docs/phase10_sitl_geofence.parm

# Webots, separately:
webots libraries/SITL/examples/Webots_Python/worlds/iris.wbt

# This project's SAR mission, once connected ("Connected to ardupilot SITL (I0)"):
python scripts/run_phase14_sar_mission.py \
    --webots-only --allow-webots-arm-test --confirm-webots-flight-test \
    --ardupilot-root /home/swarmbuild/ardupilot --search-altitude 1.0 --duration 90
```

**Exact network addresses** (never guessed, never hardcoded as a
non-loopback default - `tests/test_phase14_sar_architecture.py::test_no_hardcoded_non_loopback_ip_literal`):
`--connection` defaults to `tcp:127.0.0.1:5760` (SITL's own MAVLink
port, this script as its sole client - Phase 13's own finding);
Webots' controller listens on UDP `127.0.0.1:9002` (`9002 + 10*instance`,
reused from Phase 12's `_webots_controller_port`). No non-local address is
ever used or accepted (`localhost_connection` gate check).

## Limits (live mode)

| Limit | Value |
|---|---|
| Search altitude (target) | `1.0` m (`--search-altitude`) |
| Hard altitude ceiling | `2.0` m (`HARD_MAX_ALTITUDE_M`) |
| Max horizontal speed | `0.25` m/s (`HARD_MAX_SPEED_MPS`) |
| Max mission duration | `90` s (`HARD_MISSION_DURATION_S`) |
| Search area | one rectangle, no obstacles |
| Victims | one, this phase's first scenario |
| Return-to-home | always attempted before LAND, except a hard mission-timeout (see below) |
| Emergency LAND on timeout | yes - see note |

**Mission-timeout note**: a normal end of search (waypoints exhausted, or
a victim confirmed) always goes through RETURN_HOME before LAND_REQUESTED.
A hard **mission timeout** (the 90s budget already exhausted) instead
jumps directly to LAND_REQUESTED, skipping RETURN_HOME - RETURN_HOME's own
travel time is itself unbounded relative to an already-exhausted budget,
so landing in place is the safer emergency response. This is a
documented engineering choice, not a certified guarantee - see
`swarm_sim/sar_mission.py`'s own tick() comments.

**Live gates enforced before arming** (`_check_sar_live_gate`, mirroring
Phase 13's pattern exactly): `--webots-only`/`--allow-webots-arm-test`
flags, namespace `sitl/drone0`, altitude/speed/duration within the hard
caps above, valid system/component id, pymavlink available, localhost
connection, Webots installed, official world/params/controller files
present, Webots controller UDP port already bound. Live, post-attach
gates (real MAVLink reads, never writes): heartbeat, estimator valid,
initially disarmed, `ARMING_CHECK` enabled, geofence enabled, battery
telemetry read, no PreArm messages pending. Arm/takeoff/land themselves
reuse Phase 13's own gated, tested helper functions unmodified - **Phase
13's own arm/takeoff/land safety gates were not changed.**

## Logs (per run directory)

```
manifest.json           # swarm_sim.manifest.RunManifest - config, seeds, code version
mission_events.jsonl    # one JSON object per state transition / notable event
telemetry.csv           # per-tick position/velocity/battery/estimator/armed/mode/failsafe
safety_decisions.csv    # per-tick SafetyDecision (accepted, reason, emergency_state, active_constraints)
commands.csv            # per-tick candidate + filtered command + adapter/ack result
detections.csv          # every raw DetectionCandidate observed (believed position, confidence, duplicate flag)
actuator_log.csv        # live mode: real SERVO_OUTPUT_RAW via Phase 13's _ActuatorLogger; offline: header-only, honestly noted as not applicable (FakeSITLTransport has no actuator/PWM model)
summary.json            # the mission-level summary - see SARMission.build_summary()
```

`summary.json` fields separate **ground-truth scoring**
(`victim_ground_truth_count`, `missed_victims`, `false_positive_confirmations`)
from **sensor observations** (`observations_generated`,
`detections_used_for_confirmation`, `duplicate_observations_ignored`,
`stale_observations_rejected`) from **planner/mission belief**
(`confirmed_victim_ids`, `final_state`) from **safety decisions**
(`safety_state_histogram`, `geofence_violation_ticks`, `command_rejections`)
from **actual telemetry** (`maximum_altitude_m`, `maximum_speed_mps`,
`battery_minimum_fraction`, `hold_band_m`, `landing_result`) from
**actuator evidence** (`actuator_evidence`, live mode only, honestly
`None`/not-applicable offline).

## Acceptance criteria

**Offline** (met - see Test results below): all mission states tested;
the lawnmower path stays inside the geofence; hidden truth is isolated;
sensor noise/dropout is deterministic by seed; raw candidates cannot
bypass `SafetySupervisor`; failure states stop or land safely; logs are
reproducible (same seed -> same summary); Phase 8-13 regression tests
pass.

**Live** (not yet met - three live attempts so far, each surfacing a
different real bug, see "Live results" below): one vehicle takes off in
Webots (met on all three attempts); the vehicle follows at least one
search waypoint (**not met** - attempt 1 stuck in `TAKEOFF_REQUESTED` on
a geofence-margin bug, now fixed offline; attempt 3, after that fix, lost
altitude control entirely - see the sim-clock bug below - so a search was
never reached); telemetry confirms position and altitude (met); simulated
observations are generated (not met, same reasons); the mission returns
home (not met - the script's own hard wall-clock timeout forced landing
both times a flight got that far, never the mission's own `RETURN_HOME`
sequence); the vehicle lands (met - safely, on all three attempts, either
via the gate refusing to arm or via the script's own forced `LAND`); disarm
is confirmed (met when arming succeeded); no collision occurs (met);
actuator exchange is logged when available (met - up to 171
`SERVO_OUTPUT_RAW` messages captured per attempt); no safety gate was
bypassed (met - every fix below widens or corrects the mission's own
internal safety/timing logic, none weakens or bypasses a check). **The
most serious finding**: attempt 3 reached a measured peak altitude of
8.05 m, over 4x the 2.0 m hard ceiling, due to the transport's simulation
clock never advancing (see bug 4 below) - now fixed, but not yet
re-verified live.

## Offline results

Full offline suite: **54 new Phase 14 tests** (25 functional classes'
worth of cases in `tests/test_phase14_sar_mission.py` + 29 architecture
checks in `tests/test_phase14_sar_architecture.py`) pass, alongside the
full pre-existing suite (Phase 8-13 regression) - see the Report section
below for the exact combined count. A representative offline run
(`python scripts/run_phase14_sar_mission.py --offline --seed 3`, 10x10m
area, one victim at (5,5), search altitude 1.0m): reached `LANDED`,
`waypoints_completed: 3/6` (search ended early on confirmation),
`confirmed_victim_ids: ["victim-0"]`, `false_positive_confirmations: 0`,
`maximum_altitude_m` within a few mm of the 1.0m target,
`command_rejections: 0`, `landing_result: true`. A full run without a
confirmation (search pattern exhausted, e.g. seed `1`) also lands
successfully (`RETURN_HOME` triggered by `search_pattern_complete_no_confirmation`).

**Two real bugs found and fixed by this phase's own offline smoke
testing** (both now covered by regression tests):

1. **Geofence built from the search area alone livelocked at takeoff**:
   with `home_m` at a corner of the search rectangle, the geofence (built
   without margin around just the search area) placed home right on its
   own boundary, so `SafetySupervisor` immediately raised a persistent
   `GEOFENCE_RISK` override before the mission could ever leave
   `TAKEOFF_REQUESTED`. Fixed by `SARMissionConfig.build_geofence()` now
   enclosing **both** `home_m` and the search area, each with buffer (see
   bug 3 below for why that buffer had to be widened again after the
   first live run).
2. **Landing never completed**: with the geofence floor set exactly at
   ground level (`0.0`), `SafetySupervisor`'s own altitude-floor
   protection (a no-go boundary, not a landing target) prevented the
   vehicle from ever descending the last distance to touch down, so
   `LAND_REQUESTED` always ran into `landing_timeout_exceeded`. Fixed by
   adding a separate `ground_altitude_m` (the real landing target,
   `0.0`) distinct from `geofence_floor_alt_m` (now `-2.0` by default -
   below ground, mirroring Phase 10/13's own `FENCE_ALT_MAX` being set
   well above, not at, the operating altitude).

## Live results

**First live attempt (operator-run, `--search-altitude 1.0 --duration 60`,
real ArduPilot SITL + Webots `iris.wbt`, geofence enabled via
`docs/phase10_sitl_geofence.parm`)**: the flight itself succeeded honestly
(armed, climbed to a measured peak of **1.09 m** - target 1.0 m, `pass`,
within the 0.15 m tolerance and the 2.0 m hard ceiling - landed, and
disarm was confirmed), but **the SAR mission logic never worked**: it
stayed in `TAKEOFF_REQUESTED` for the entire flight
(`waypoints_completed: 0`, `observations_generated: 0`) - the vehicle
only hovered near home before the live script's own outer hard timeout
forced a direct `LAND`, never the mission's intended `RETURN_HOME ->
LAND_REQUESTED` sequence.

**Third real bug found - this time only by the live run (offline testing
could not have caught it)**: `build_geofence()`'s one-`geofence_margin_m`
buffer
   placed `home_m` exactly on the same threshold distance that
   `SafetySupervisor.geofence_horizontal_margin_m()` uses to raise
   `GEOFENCE_RISK` - not strictly inside it. `FakeSITLTransport`'s
   noiseless offline kinematics sits exactly on that boundary and never
   crosses it, but the real SITL vehicle's GPS/accel noise and PID
   hunting crossed it on 86 of ~90 ticks, permanently overriding every
   candidate and preventing the mission from ever confirming "search
   altitude reached." Fixed by widening the buffer to `2 *
   geofence_margin_m`, giving `home_m` a full `geofence_margin_m` of real
   headroom inside the risk band - verified offline
   (`geofence_horizontal_margin_m(home_m, geofence) > geofence_margin_m`,
   regression-tested in `test_home_sits_with_real_headroom_inside_the_geofence_risk_margin`)
   and by re-running the same offline scenario the live flight used
   (`waypoints_completed: 3`, `observations_generated: 114`,
   `confirmed_victims: 1`, zero `GEOFENCE_RISK` ticks, `final_state:
   LANDED`).

**Second live attempt (same command, after the geofence-margin fix)**:
arming was rejected outright (`result=4`, `MAV_RESULT_FAILED`) with no
accompanying STATUSTEXT - traced to ArduPilot's real-time `SYS_STATUS`
PREARM_CHECK health bit flapping unhealthy for a few seconds at a time
while the rate-limited "PreArm: ..." STATUSTEXT banner our own gate
scans for happened to stay quiet, so the gate reported "clear" while the
vehicle was still momentarily unhealthy. Fixed by gating the arm attempt
on that real-time health bit directly (3 consecutive healthy reads,
`_wait_for_stable_prearm_health`) instead of a fixed STATUSTEXT window,
and by recording the actual STATUSTEXT reasons for any arm/takeoff/land
rejection instead of discarding them.

**Third live attempt (same command, after the arm-gate fix)**: this time
armed and took off cleanly, but the SAR mission control loop produced a
**serious, real safety-relevant failure**: measured peak altitude reached
**8.05 m** - more than 4x the documented 2.0 m hard safety ceiling - and
measured speed reached 1.72 m/s against a configured 0.25 m/s cap. The
vehicle landed and disarmed safely once the script's own outer timeout
forced a `LAND`, but the SAR mission's own command loop lost control of
altitude for the full ~90 second flight.

**Fourth real bug found - the most serious of the four, and only visible
live**: `ArduPilotSITLTransport` never advances its own simulation clock
automatically (unlike `FakeSITLTransport.step()`, which calls
`set_sim_time()` internally - see the transport's own "Clock model"
docstring) - the live tick loop never called `transport.set_sim_time()`,
so every telemetry sample was timestamped `0.0` for the entire flight.
`SafetySupervisor.evaluate()` computes its own real-dt-based acceleration
limiter (`bound_velocity_step`) from consecutive telemetry timestamps;
with `dt_s` clamped to its `1e-3` floor every single tick, that limiter
treated almost any velocity change as exceeding the allowed acceleration
and froze each commanded velocity within a hair of the vehicle's own
last REAL velocity - including its residual post-takeoff climb rate -
instead of correcting it toward the intended hold/search altitude. The
vehicle coasted upward on its own residual momentum for far longer than
intended before the ceiling-avoidance override could gain any real
authority. The same frozen clock also meant `SARMissionConfig.mission_timeout_s`
could never fire on its own (elapsed time was always `0.0 - 0.0`),
relying entirely on this script's separate outer wall-clock deadline to
end the flight - masking the fact that the mission's own timeout logic
was inert. Fixed by calling `transport.set_sim_time(time.monotonic() -
mission_start_wall)` at the top of every tick, before reading telemetry -
regression-tested structurally in
`test_live_mission_loop_advances_the_transport_clock_every_tick`
(`tests/test_phase14_sar_architecture.py`), since this failure mode
requires a real transport and cannot be reproduced against
`FakeSITLTransport` (which advances its clock for you).

**Fourth live attempt (operator-run, same command, after the sim-clock
fix)**: a clear improvement - the mission actually progressed for the
first time (`TAKEOFF_REQUESTED -> TRANSIT -> RETURN_HOME`, reaching
`LAND_REQUESTED` as its final state, `observations_generated: 1`,
`GEOFENCE_RISK` down from 88 ticks to 16), armed/landed/disarmed cleanly.
But measured peak altitude still reached **3.06 m**, still over the 2.0 m
hard ceiling (`altitude_result.pass_fail: "fail"`) - much smaller than
the 8.05 m excursion, but still a real ceiling breach, not a pass.

**Fifth real bug found**, traced from the actual per-tick `commands.csv`/
`telemetry.csv` for this run: at the moment `SARMission`'s tick loop took
over from ArduPilot's own internal takeoff climb, the vehicle still had
real residual upward velocity (`+1.36 m/s`, matching Phase 13's own
well-documented takeoff-overshoot-then-settle pattern - not itself a
bug). The very first `SafetySupervisor.evaluate()` call for a newly-
tracked vehicle has no prior timestamp to compute a real `dt` from, so it
falls back to `SafetySupervisorConfig.fallback_dt_s`, which defaults to
`1/24 s` - a frame-rate assumption tuned for a much faster control loop
elsewhere in this codebase, not this mission's ~1 Hz live tick rate. That
tiny fallback `dt` gave the very first, momentum-critical correction an
acceleration budget roughly 24x too small (observed: commanded
`vz` barely moved from `+1.36` to `+1.20 m/s` on that first tick, instead
of a realistic one-second correction), so the vehicle kept coasting
upward on real leftover momentum for another two full ticks before a
correctly-sized `dt` took over on tick 2 and the ceiling-avoidance
override (a steady, correct `-0.125 m/s`) finally brought it back down.
Fixed two ways: `SARMissionConfig.build_safety_config()` now passes
`fallback_dt_s=self.dt_s` instead of leaving the mismatched default, and
the live script now sets `sar_config.dt_s = _MISSION_TICK_DT_S` (its own
real per-tick sleep interval, `1.0s`) right after building the config, so
that fallback matches the live loop's actual cadence - regression-tested
in `test_build_safety_config_uses_mission_dt_as_the_first_tick_fallback`
and `test_explicit_safety_config_override_is_not_clobbered`
(`tests/test_phase14_sar_mission.py`).

**Fifth live attempt (operator-run, same command, after the fallback_dt_s
fix)**: the two safety-critical bugs are confirmed fixed. The mission
again progressed properly (`TAKEOFF_REQUESTED -> ... -> LAND_REQUESTED`,
`observations_generated: 1`, armed/landed/disarmed cleanly), and this
time measured peak altitude was **1.70 m - within the 2.0 m hard ceiling**
(`altitude_result.within_hard_ceiling: true`). The prearm-health gate
also proved itself: it correctly waited through 80 samples, including
several genuinely unhealthy ones, before letting the arm attempt proceed
(compare the first arm-gate fix's typical 3-sample case).

The remaining `altitude_result.pass_fail: "fail"` is a much smaller,
different kind of finding: the peak overshot the 1.0 m target by 0.70 m
(+70%, outside the 0.15 m tolerance band) while staying safely under the
hard ceiling - the same *category* of real, physical takeoff-overshoot
behavior already documented and accepted for Phase 13's own live run
(2.11 m vs. a 2.0 m target), not a new safety bug. It is plausibly the
same residual post-takeoff-momentum handoff effect at a smaller
magnitude now that the timing fix is in, and could be further reduced by
waiting for vertical velocity (not just altitude) to settle before
handing off from ArduPilot's own takeoff to the mission's tick loop -
left as a documented, non-blocking characteristic pending operator
direction rather than chased further on its own.

Two safety-relevant bugs (the frozen sim-clock and the mismatched
fallback `dt`) are now fixed and live-verified. A further live attempt
to tighten the tolerance-band overshoot, or to move on to reviewing
Phase 14 as a whole, is an operator decision, not assumed.

## Limitations

- The live mode's SAR-search phase (velocity setpoints via
  `build_ardupilot_sitl_adapter`) has only been exercised against the
  offline `FakeSITLTransport`, never against real ArduPilot SITL - the
  underlying `SITLAdapter`/command-translation path itself was already
  proven live in Phase 9's `run_planner_preview`, but the SAR-specific
  tick loop built on top of it has not.
- Confirmation stops the search after the *first* confirmed victim - not
  a multi-victim tour (see "Sensor model" above).
- No obstacle avoidance, no polygon search areas, no swarm coordination -
  by design, per this phase's scope.
- Offline `actuator_log.csv` is honestly empty (header only) -
  `FakeSITLTransport` has no actuator/PWM model to report.
- `collisions_or_contacts` is not modeled offline; live mode inherits
  Phase 13's own honest "not directly observable via MAVLink" limitation.
- `realtime_factor` is `None` in both modes for the same reason Phase 12/13
  documented: not obtainable via MAVLink, only visible in Webots' own GUI
  timeline.
