# Phase 15: bounded Swarm124/Langostino research integration

**Status: Phase 15A-15C (offline) complete and tested. Phase 15D (one-drone
Webots integration) and 15E (two-vehicle smoke test) have not been
attempted - see "What remains" below.**

## Explicit conclusion (required statement)

> Langostino is used as a single-aircraft ROS2/INAV reference.
> Swarm124 is used as a simulation/policy-benchmark reference.
> The current project remains the authoritative safety and ArduPilot/Webots
> integration layer.
> No external learned policy or raw actuator path bypasses SafetySupervisor.

## What was verified from Langostino

Directly fetched from the real repository and its GitHub API metadata
(not inferred, not guessed):

- **Repository**: [`swarm-subnet/Langostino`](https://github.com/swarm-subnet/Langostino)
- **Commit verified**: `035b334d76984808ea057f2e9c3e1689a8e81e05` (2026-02-24)
- **License**: MIT (SPDX `MIT`)
- **Homepage**: `https://swarm124.com/` (confirming Langostino and
  "Swarm124" are the same ecosystem/organization, not unrelated projects)
- **Description** (verbatim from the repo): "An open-source autonomous
  drone platform using ROS2 and AI-powered flight control. A complete
  reference implementation for building, understanding, and extending
  real-world drone autonomy."
- **Topics**: `ai, autonomous-flight, bittensor, drone, gps, inav, lidar,
  raspberry-pi, reinforcement-learning, robotics, ros2`
- **Stack** (from the README): INAV flight controller (custom firmware
  builds included in-repo), Raspberry Pi companion computer, ROS2 (Humble),
  MSP protocol (INAV's own serial protocol, spoken by the companion
  computer to the flight controller), LiDAR (I2C), GPS.
- **Top-level layout** (from the README's own description):
  `src/swarm_ai_integration/` (main ROS2 package), `scripts/`, `docs/`,
  `assets/` (3D-printable STL files), `inav-custom-firmware/`, `mapproxy/`,
  `model/`, `flight-logs/`.

## What was verified from Swarm124

- **Repository**: [`swarm-subnet/swarm`](https://github.com/swarm-subnet/swarm)
  (the actual policy-benchmark platform; `swarm-subnet/swarm-gym-pybullet-drones`
  is the org's separate PyBullet simulation fork this platform is built on)
- **Commit verified**: `fce8d303fc74d8d9d0731599a43964096a0b5d35` (2026-09-17)
- **License**: MIT
- **Confirmed**: this is Bittensor **Subnet 124** - "Swarm124" in the
  operator's own naming refers to this subnet, not a separate product.
- **Observation space** (verbatim from the README): a single front-facing
  depth camera ("what's in front of it") plus the vehicle's own position
  and speed - **explicitly no map, GPS, or obstacle list** - updated at
  50Hz.
- **Action space** (verbatim): "a direction, a speed, a turn" - continuous,
  sent at 50Hz.
- **Missions include an explicit team SAR mission**: "Swarm Search and
  Rescue - Send a team of drones to sweep the area and find the victim
  together," and "Swarm Autopilot - Land a whole team of drones, fast and
  without collisions." Exact team size (this project assumed 2-8 per its
  own spec) is **not stated** in the README - the 2-8 range used in this
  phase's own offline fixture is this project's own bounded assumption,
  not a verified Swarm124 constant.
- **Scoring**: success 45% / speed 45% / safety 10% (safety term dropped
  for the two pure-pursuit interceptor missions), averaged across 1,000
  seeded worlds per 14-day epoch. Seeds are "the same on every validator"
  within an epoch but "secret until the epoch closes" - genuinely
  deterministic per-epoch, not reproducible in advance from outside.
- **Not specified anywhere in the README**: an exact observation/action
  array shape or dtype, a policy-version field, or a model-checksum field.
  This project's `Swarm124Observation`/`Swarm124Action` (see
  `swarm_sim/external/swarm124_policy_adapter.py`) therefore add those
  fields as this project's OWN requirement for its own offline fixture,
  not as a Swarm124 spec.

## What was inferred (not directly verified)

- The exact shape/dtype of Swarm124's real depth-camera tensor (this
  project's `depth_returns_m` is a bounded 8-element tuple standing in for
  "a single depth camera," never a real image, and never claimed to match
  Swarm124's own internal representation).
- Any teammate-relative observation shape for the multi-agent SAR mission
  (Swarm124's README does not describe one) - `teammate_relative_m` is
  this project's own bounded (at most 7 teammates, i.e. up to 8 agents
  total) invention for its own offline fixture.
- Langostino's exact ROS2 topic names/message types (not enumerated in
  the top-level README fetched this phase) - this project's fixtures
  (`LangostinoLidarFixture`, `LangostinoTelemetryFixture`,
  `LangostinoLinkFixture`) are typed by CONCEPT (a LiDAR reading, GPS/
  attitude/altitude telemetry, a companion-computer link status), not by
  any specific ROS2 message schema.

## Licensing notes

Both `swarm-subnet/Langostino` and `swarm-subnet/swarm` are MIT-licensed.
Nothing from either repository's source code is copied, vendored, or
imported into this project - only publicly-stated facts (README text,
repo metadata) are used to document the interface this project's own,
independently-written fixtures approximate. No MSP, ROS2, or PyBullet
code from either project runs anywhere in this repository.

## External-to-internal adapter boundaries

```
Swarm124-style policy action (direction, speed, turn + this project's
own required metadata: policy_version, seed, run_id, action_expiry_s)
    -> swarm_sim.external.swarm124_policy_adapter.convert_swarm124_action_to_candidate()
    -> swarm_sim.external.external_contracts.validate_and_build_candidate()
       (finite-value / frame / staleness / expiry / vehicle-ID / speed /
        accel / metadata checks - see "Reject" list below)
    -> swarm_sim.safety_supervisor.CandidateCommand   (the SAME untrusted
       type SwarmController itself produces - no special-casing)
    -> SafetySupervisor.evaluate()                    (unmodified, this
       phase never touches safety_supervisor.py)
    -> SafetyDecision.filtered_command -> AdapterCommand -> ArduPilot adapter
```

```
Langostino-style sensor/telemetry/link fixture (offline dataclass)
    -> swarm_sim.external.langostino_reference.map_*()
    -> swarm_sim.contracts.SensorObservation / VehicleState / HealthState
       (this project's own existing, already-tested contracts - never a
        new Langostino-specific type the rest of the project would need
        to learn)
```

Both boundaries are checked structurally, not just documented - see
`tests/test_phase15_external_architecture.py`: no external module imports
`pymavlink`/`mavutil`/`serial`/`socket`, none imports from
`swarm_sim.autopilot`, none constructs `AdapterCommand` directly, and none
references an arm/takeoff/RC-override identifier anywhere.

## Frame/unit conversions

Every `Swarm124Observation`/`Swarm124Action` and every Langostino fixture
is declared in `Frame.LOCAL_ENU` (this project's own convention - see
`swarm_sim/contracts.py`'s `Frame` docstring). `convert_swarm124_action_to_candidate`
fixes `expected_frame=Frame.LOCAL_ENU` explicitly; any action claiming a
different frame is rejected as `WRONG_FRAME`, not silently converted -
frame conversion (ENU<->NED) remains exactly where Phase 6 put it
(`swarm_sim/autopilot/frames.py`), not duplicated here. `direction_xy` is
normalized to a unit vector before being scaled by `speed_mps` (handling
a non-unit-length direction from any future policy gracefully); a
zero-length or non-finite direction is either treated as "hold position"
(finite, near-zero norm) or rejected outright (non-finite norm) - never
silently coerced into the other case (see
`test_nan_direction_component_is_rejected_not_silently_zeroed`, a real
bug this phase's own testing caught and fixed before it shipped).

## Policy timing

The verified Swarm124 action rate (50Hz) is NOT what this project's own
offline fixture runs at - `ScriptedSwarm124Policy` is driven per-call by
whatever `SARMission`-style loop calls it (this phase's offline tests use
a single-shot `.act()` call per scenario, not a running loop). No claim is
made that this project reproduces Swarm124's real-time control rate; the
`action_expiry_s` field exists precisely so a caller can bound how long a
stale action remains acceptable regardless of the actual calling cadence.

## Failure handling

Every rejection reason in `ExternalActionRejection` (NaN/inf, wrong units,
wrong frame, stale, expired, unknown/duplicate vehicle ID, speed/accel
limit, malformed observation, unexpected action dimensions, missing seed/
run-id, missing policy version) is a distinct, testable enum value - a
test can assert exactly which check fired, not just that something failed
(see `tests/test_phase15_external_contracts.py`). An invalid Langostino
altitude reading is substituted with a `0.0` sentinel and
`estimator_valid=False`/`health_state=DEGRADED` rather than passed through
as NaN, because `VehicleState.__post_init__` (unlike `CandidateCommand`)
requires every field to already be finite - a real bug
(`ValueError` crashing the caller instead of degrading it) this phase's
own smoke test caught before any test file was even written.

## One-drone results

**Not yet attempted.** Phase 15D (run one external-policy fixture through
`SafetySupervisor` and a real ArduPilot/Webots vehicle) requires the same
live Webots environment as Phase 14B. At the time of this document,
ArduPilot SITL was up and reachable, but Webots' own simulation could not
be brought into a running state from this automated environment (see
docs/PHASE14_SAR_WEBOTS.md's own live-attempt log for the parallel
Phase 14B blocker) - a real, discovered limitation of driving Webots' GUI
from a non-interactive shell, not a Phase 15 code defect.

## Two-drone results

**Not attempted.** Per Phase 15's own explicit gate, a two-vehicle test
requires Phase 14B to succeed and Phase 15D (one-drone) to pass first -
neither has happened yet.

## What remains simulation-only

Every fixture in this phase (`Swarm124Observation`/`Swarm124Action`,
`LangostinoLidarFixture`/`LangostinoTelemetryFixture`/`LangostinoLinkFixture`)
is a plain, offline Python dataclass. Nothing constructed by this phase
has ever been sent to a real ArduPilot instance, a real Webots simulation,
or real Langostino/INAV hardware.

## What remains unverified for hardware

- Whether a real Langostino LiDAR/GPS/altitude reading actually arrives in
  the shape this project's fixtures assume (only the CONCEPT is verified
  from the README; no real Langostino ROS2 messages were captured or
  replayed).
- Whether a real Swarm124-trained policy's actual output distribution
  (magnitude, noise, latency) resembles the deterministic scripted policy
  used for this phase's first pass - Phase 15B explicitly required a
  learned model NOT be required for this pass, and none was used.
- Everything Phase 15D/15E would have exercised: real telemetry evidence,
  actual actuator response, real two-vehicle routing/identity.

## Why neither external project is treated as a safety-certified flight stack

Langostino is a single-aircraft reference platform built around INAV, a
different flight-control firmware with its own arming/failsafe model -
its limits are not assumed to transfer to ArduPilot (Phase 15's own
explicit restriction), and this project never imports its MSP/serial code.
Swarm124 is a simulation/policy-benchmark platform scored entirely in
PyBullet against synthetic depth/state observations, with no
MAVLink routing, no per-vehicle identity/lease handling, no geofencing, no
collision deconfliction, no link-loss behavior, and no hardware validation
of its own (see Phase 15's own "The benchmark does not replace" list) - a
high benchmark score is a policy-quality signal, not real-flight evidence.
This project's own `SafetySupervisor` remains the only component in this
whole system with the authority to accept or reject a command before it
reaches ArduPilot; neither external project's own logic is ever in that
position.

## Files changed this phase

- `swarm_sim/external/__init__.py` (new)
- `swarm_sim/external/external_contracts.py` (new) - shared validation
  pipeline (`ExternalActionRejection`, `ExternalPolicyAction`,
  `ExternalAdapterResult`, `validate_and_build_candidate`,
  `validate_observation_shape`)
- `swarm_sim/external/swarm124_policy_adapter.py` (new) -
  `Swarm124Observation`, `Swarm124Action`, `ScriptedSwarm124Policy`,
  `convert_swarm124_action_to_candidate`, `validate_observation`
- `swarm_sim/external/langostino_reference.py` (new) - fixture
  dataclasses, `map_lidar_to_sensor_observation`,
  `map_telemetry_to_vehicle_state`, `map_link_status_to_health_state`,
  `is_altitude_valid`, `is_observation_stale`, and one named builder
  function per required scenario (normal LiDAR, missing LiDAR, normal
  telemetry, stale observation, invalid altitude, emergency-landing
  request, companion-computer link loss)
- `tests/test_phase15_external_contracts.py` (new, 21 tests)
- `tests/test_phase15_swarm124_adapter.py` (new, 20 tests)
- `tests/test_phase15_langostino_reference.py` (new, 15 tests)
- `tests/test_phase15_external_architecture.py` (new, 11 tests)
- `docs/PHASE15_SWARM124_LANGOSTINO_INTEGRATION.md` (this file)

`swarm_sim/distributed_consensus.py`, `swarm_sim/safety_supervisor.py`,
and every Phase 10-14 file are **unmodified** this phase.

## Two real bugs found and fixed by this phase's own offline testing

1. **NaN direction silently became a zero-velocity command instead of
   being rejected**: `convert_swarm124_action_to_candidate`'s original
   normalization computed `dir_norm = hypot(dir_x, dir_y)` and treated any
   `dir_norm` that was not `> 1e-9` (which is true for NaN, since every
   NaN comparison is False) as "no direction, hold position" - silently
   masking a malformed input as an innocuous command instead of letting
   the downstream finite-value check reject it. Fixed by leaving a
   non-finite direction's components unchanged (not defaulted) so
   `validate_and_build_candidate`'s own NaN/Inf check catches it
   explicitly - regression-tested in
   `test_nan_direction_component_is_rejected_not_silently_zeroed`.
2. **Invalid-altitude mapping crashed instead of degrading**:
   `map_telemetry_to_vehicle_state` originally passed a NaN altitude
   straight into `VehicleState.position_m`, which raised `ValueError`
   inside `VehicleState.__post_init__` (it requires every field finite,
   unlike `CandidateCommand`'s deliberately-permissive pattern) - a
   crash, not the intended graceful `estimator_valid=False`/
   `health_state=DEGRADED` degradation. Fixed by substituting a `0.0`
   sentinel altitude when invalid, never a NaN, with the health flags
   doing the real work of telling the caller not to trust it -
   regression-tested in `test_invalid_altitude_degrades_without_crashing`.

## Validation

```
python -m compileall -q swarm_sim run_mission.py tests scripts   # clean
python -m pytest -q tests                                        # see report
```
