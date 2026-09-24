# Phase 16: GNSS-denied swarm SAR - roadmap, requirements traceability, assumptions

**Status: Phase 16A (mission frame + 1x1 m grid) and Phase 16B (estimated
state, drift, covariance-aware geofence) are built and tested - see
[PHASE16A_MISSION_FRAME_GRID.md](PHASE16A_MISSION_FRAME_GRID.md) and
[PHASE16B_ESTIMATION.md](PHASE16B_ESTIMATION.md). Phases 16C-16J are outlines
only.**

## Why this phase exists

The competition rules deny GPS from the start of the mission. Before this
phase the PyBullet swarm simulation was **perfect-state**:
`FloodSearchMission.run()` fed PyBullet ground-truth position, velocity and
attitude straight into the vehicle state, the sensors, the network and the
recruitment board (`pose_uncertainty_m` was hard-coded to 0.0). Nothing in
the swarm could experience localisation drift, so nothing about its
GNSS-denied behaviour could be measured. Phase 16 replaces that with an
estimator-driven pipeline, a mission-local frame anchored at the launch zone,
and a 1x1 m grid in which survivors are reported.

## What this phase can and cannot claim

- **Can:** show how the swarm (search, consensus, safety, return-to-zone)
  behaves against an *assumed, parameterised* drift/noise model, and that no
  autonomy module reads ground truth or GNSS (enforced by AST tests).
- **Cannot:** validate the accuracy of real VIO, optical flow, or LiDAR
  odometry. Every drift number in the estimator profiles will be labelled
  "illustrative, not hardware-verified". Real accuracy needs hardware-in-the-
  loop, bench, and tethered/indoor flight - the same path Phase 15's
  real-life roadmap already lays out.
- The 25 kg limit can only be checked against a *declared* airframe mass; the
  simulated CF2X is roughly 27 g.
- Claims from third-party design discussions (specific research systems for
  swarm LiDAR-inertial odometry, UWB-free relative localisation, etc.) and
  ArduPilot external-navigation behaviour (EKF3 source selection,
  VISION_POSITION_ESTIMATE / ODOMETRY) have **not been verified against
  primary sources** and must not be cited in any doc until they are.

## Requirements traceability

Requirements are paraphrased from the competition rules as supplied by the
operator; the rule numbers are not reproduced here.

| Requirement | Phase | Status |
|---|---|---|
| >= 2 drones as one coordinated system | existing swarm + 16E | existing (centralised parts remain until 16E) |
| Combined all-up weight <= 25 kg | 16A (`mission_rules.check_fleet_mass`) | constants + check built; only meaningful on declared masses |
| All drones take off from and land in a 12 ft x 12 ft launch zone | 16A (`LaunchZone`, slots), 16C (lifecycle), 16F (RTH-in-zone) | zone geometry + slots built; flight behaviour not yet |
| Autonomous search of the mission area | existing `FloodSearchMission` | existing; runs on estimated state in `localization_mode="estimated"` (16B) |
| Detect up to ten survivors | 16A (`MAX_SURVIVORS`), 16D | constant only |
| Autonomously geotag each survivor; show on GCS | 16A (1x1 m cell), 16D (cell + sigma), 16H | cell naming built; geotag pipeline not yet |
| Divide the map into 1x1 m grid; tag the block a survivor is found in | 16A (`GridSpec`, `cell_id`) | built and tested |
| Autonomous 200 g kit delivery to detected survivors | 16A (`KIT_*`), 16G | constants only |
| Single GCS, unified view, supervisory only | 16H | not started |
| GCS shows status, camera feed, position/estimate, task/area, survivors, kit status, health, progress | 16H | not started |
| No manual waypoint/path/release/tag/replan during execution | 16H (read-only GCS) + AST test | not started |
| No external network for coordination or data exchange | existing `CommsNetwork` (local, lossy) + 16E | existing |
| Stay within mission boundary (geofence) | existing SafetySupervisor + 16B (sigma-aware margin) | 16B built: enforced on the drone's own estimate, with an optional covariance-aware margin; true excursions are measured. RTH/abort under denial is 16F |
| Return-to-Home, comm-loss recovery, low-battery failsafe, mission abort | existing SafetySupervisor states + 16C/16F | partly existing; denial-aware versions in 16F |
| GPS denied from the start of the mission | 16B (estimator), 16J (GNSS only as an ablation baseline) | 16B built: no autonomy module can name GNSS/GPS or read ground truth (AST-enforced); dead-reckoning only, no aiding until 16F |

## Roadmap

| Phase | Scope |
|---|---|
| **16A** | `MissionArea`, `LaunchZone`, `GridSpec`/`CellIndex`, `mission_rules` - pure data, no existing file touched. **Done.** |
| **16B** | `EstimatedState`, drift profiles (VIO, optical flow + rangefinder, LiDAR odometry), EKF with covariance, truth-boundary AST tests, `localization_mode` flag defaulting to legacy behaviour. **Done.** |
| 16C | Per-drone `FlightPhase` state machine (grounded / takeoff / search / deliver / RTH / land / abort), launch-slot spawn, landing, battery drain; **plus a real altitude hold** decoupled from the horizontal safety limiter (16B finding 7: the legacy vertical channel only damps, so estimated-vertical-speed noise random-walks altitude and confounds mission-level results) |
| 16D | Survivor localisation to cell + sigma; cell-based evidence with origin-drone dedup; covariance-aware consensus gating |
| 16E | Distributed grid belief (gossip, monotonic status lattice) and distributed task allocation; removes the centralised `RecruitmentBoard` and the omniscient confirmed-id set |
| 16F | Failsafes under denial: RTH into the 3.66 m zone (needs launch-zone landmark aiding - dead-reckoning alone cannot hit it), comm-loss, low battery, abort, landing-in-zone scoring; cooperative aiding via covariance intersection |
| 16G | Simulated kit delivery with terminal visual servoing on the survivor (a relative, drift-free measurement) |
| 16H | Read-only GCS with a publish hook |
| 16I | Scenario matrix and metrics (cell-hit rate vs truth, true boundary excursions, NEES, RTH-in-zone rate) |
| 16J | GNSS baseline for ablation only; AST-forbidden from autonomy modules |

Phase 15E (live two-vehicle Webots/SITL smoke test) is deferred. All of
Phase 16 targets the PyBullet swarm simulation; the earlier hold on live
multi-vehicle Webots deployment is untouched.

## Assumptions register

Each item below was **inferred**, not stated by the rules. Changing one
should be a deliberate decision, recorded here.

| # | Assumption | Where it lives | Risk if wrong |
|---|---|---|---|
| A1 | The launch zone lies entirely inside the search area | `MissionMap` validation (override: `allow_zone_outside_area`) | Maps whose zone is outside the area need the override |
| A2 | The grid is axis-aligned with the mission frame | `GridSpec` | A rotated field grid needs a frame rotation, not a grid change |
| A3 | The mission-frame origin is the launch-zone centre | `frame.py` docstring, `zone_at_corner` | Only cosmetic in 16A; matters when estimates are anchored (16B) |
| A4 | The search area is an axis-aligned rectangle | `MissionArea` | Polygonal areas are not supported |
| A5 | A cell owns its lower edges; the grid's far edge belongs to the last cell; points within 1e-9 cells of a grid line snap onto it | `GridSpec.cell_of` | Survivors exactly on a line could be assigned to the other cell by an organiser using a different tie rule |
| A6 | Cell 0,0 is at the area's minimum-x / minimum-y corner; ids are `X<ix>Y<iy>` (zero-padded to 2 digits) | `GridSpec.cell_id` | The organisers' block naming may differ; only a GCS display mapping changes |
| A7 | The launch zone does not align to cell boundaries (3.6576 m is not a whole number of 1 m cells) | - | None functionally; noted so nobody assumes it does |
| A8 | Each drone starts knowing its own spawn pose in the mission frame to the profile's launch-slot accuracy | `estimation/profiles.py`, `OdometrySuite.initial_offsets` | If drones cannot be placed that precisely, the initial error is larger |
| A9 | Odometry error is scale-factor, velocity-bias and yaw-rate-bias random walks plus white noise, dropouts and degenerate ticks; all magnitudes illustrative | `estimation/profiles.py` | Real sensors have other failure modes (loop-closure jumps, tracking loss, map-relative behaviour); numbers are not hardware-verified |
| A10 | In `estimated` mode every consensus confirmation becomes a beacon | `FloodSearchMission._announce_beacon_for_confirmation` | Legacy `truth_state` still gates beacons on ground truth; the two swarms differ by design |
| A11 | A drone's motion is its estimated-frame command rotated by the heading error only | `FloodSearchMission._to_plant_frame` | Scale and bias errors would perturb real motion by about 1 % more |
| A12 | The radio reports true link range to the flocking controller | `CommsNetwork.tick` (`dist`) | A real radio's ranging is noisy or absent |
