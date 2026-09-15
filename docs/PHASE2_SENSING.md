# Phase 2 — Sensing and Ground-Truth Isolation

Adds `swarm_sim/sensors.py` and rewires `mission.py`/`controller.py` so
`SwarmController` never receives ground truth. Does not touch
`ConsensusBoard`, `CommsNetwork`, `RecruitmentBoard`, or anything
MAVLink/SITL/Pixhawk-related — those are unchanged from Phase 1.

**Not flight-certified. Sensing here is imperfect by design; there is no
independent safety supervisor yet (that's Phase 4) — a controller acting
on a bad or stale sensor reading today has nothing else checking it.**

## The sensing boundary

```
Hidden simulation plant (ground-truth positions/velocities/obstacles/victims)
                |
                v
  VictimSensorModel / ObstacleRangeSensor / NeighborSensorModel   (sensors.py)
                |
                v
    SensorObservation / NeighborObservation      (contracts.py, Phase 1)
                |
                v
              SwarmController                      (never sees ground truth)
```

`sensors.py` is the *only* module permitted to read hidden ground truth in
order to produce what a drone's own sensors would actually measure. Ground
truth remains available separately, only for:
- **scoring**: `mission.py`'s `_match_victim`, `_classify_detections_for_scoring`
- **collision/contact verification**: `_track_clearance_and_contacts`'
  ground-truth-clearance half, and the real PyBullet `getContactPoints()`
  check in `run()`
- **evaluation metrics**: the scenario report below

Two flocking-adjacent channels were already correctly separated from
ground truth *before* Phase 2 and are unchanged: `CommsNetwork` (radio
range/loss/latency, used only by `_perceived_state` for cohesion) and
`RecruitmentBoard` (beacon beliefs, populated only from consensus-
confirmed centroids). Phase 2's own scope was specifically the three
leaks identified in `docs/PHASE0_AUDIT.md`: victim-detection ground-truth
gating, and ground-truth obstacle/neighbor-position access for avoidance.

## The controller interface

```python
controller.step(i, own_state, sensor_observation, neighbor_observations, mission_beliefs, network)
```

- `own_state` (`contracts.VehicleState`) — this drone's own telemetry.
  Legitimate: a drone knows its own position/velocity (Phase 2 does not
  yet model own-state estimator noise - see Known Limitations).
- `sensor_observation` (`contracts.SensorObservation`) — this tick's
  victim-detection candidates (`.detections`) and obstacle range scan
  (`.range_returns_m`/`.occluded`).
- `neighbor_observations` (tuple of `contracts.NeighborObservation`) —
  this drone's onboard proximity-sensor readings, used only by the hard-
  safety avoidance branch.
- `mission_beliefs` — `RecruitmentBoard`: confirmed beacon beliefs, once
  removed from truth via noisy sensing + consensus, never ground truth.
- `network` — `CommsNetwork`: the already comms-realistic channel
  flocking cohesion perceives other drones through.

`neighbor_observations` (onboard sensor, no radio) and `network` (radio-
mediated) are deliberately kept as two separate parameters rather than
merged into one, even though an idealized interface would only need one
"other drones" argument. Reason: collision avoidance must never depend on
a channel that can be degraded by a bad radio link — exactly the
principle `CommsNetwork`'s own docstring already stated before Phase 2.
Merging them would quietly reintroduce that coupling.

`SwarmController.__init__` no longer takes an `obstacles` parameter at
all, and `step()` takes no `positions`/`velocities` array covering more
than one drone. This is enforced both by a static AST guard and a
behavioral test — see Testing below.

## Sensor models (`swarm_sim/sensors.py`)

### VictimSensorModel

Per drone, per tick: for each not-yet-confirmed victim, gated in order by
range → horizontal FOV → vertical FOV (see Coordinate frame below) →
obstacle occlusion → false-negative roll. A surviving detection gets
Gaussian position noise and a `confidence`/`localization_uncertainty_m`.
Independently, a false-positive roll may add one phantom detection inside
the sensor's range/FOV, facing direction, at a plausible-but-invented
position. All surviving candidates (real and phantom) then get IDs from
one shared counter (`det-1`, `det-2`, ...) — **nothing about a candidate's
ID or shape reveals the hidden victim index or whether it's real**; see
Testing. The whole observation is then subject to dropout (candidates ->
`()`, `dropout=True`) and fixed latency (a deque-based delay: whatever's
returned this tick was computed exactly `victim_sensor_latency_steps`
ticks ago; during the initial ramp-up, nothing is deliverable yet).

### ObstacleRangeSensor

A fixed-angular-bin range scan (`obstacle_sensor_num_bins` bins spanning
`obstacle_sensor_fov_deg`, centered on the facing direction) using real
ray-circle intersection against every obstacle. Each bin reports the range
to the *nearest* obstacle edge in that slice, or `+inf`. A farther
obstacle behind a nearer one in the same narrow bin is never reported —
this is the sensor's entire occlusion model, and needs no separate flag:
`SensorObservation.occluded` is always all-`False` for this channel (see
Known Limitations). Dropout reads as all-`+inf` (fail-empty, not a
separate signal) rather than a boolean flag, since "nothing in range" and
"sensor failed this tick" are indistinguishable to anything downstream
either way. Subject to the same fixed-latency delay mechanism.

### NeighborSensorModel

Structurally mirrors `CommsNetwork.tick()` (a proven, already-tested
shape for "bounded-range channel with loss and latency"), because it
solves the same kind of problem over the same kind of data — just as an
onboard sensor, not a radio link. Per (receiver, sender) pair: range →
FOV → dropout roll; a surviving reading gets Gaussian noise on position
*and* velocity, and is queued with a delivery delay. `observations_for(i,
dt)` returns every neighbor currently cached for `i`, each carrying
`sample_timestamp_s`/`delivery_timestamp_s`/`packet_age_s` (`packet_age_s`
is exactly the configured `neighbor_sensor_latency_steps * dt`) and a
`stale` flag once `age_steps > neighbor_sensor_stale_timeout_steps` — a
stale entry is still returned (spec: "stale observations are marked
stale," not "discarded"), just with a halved confidence, until it exceeds
`5x` the timeout and is pruned entirely so indefinitely-old ghosts don't
accumulate forever.

## Coordinate frame and units

Same as Phase 1's `contracts.Frame.LOCAL_ENU` (x/y an application-level
convention over PyBullet's generic Z-up world - see
`docs/PHASE1_CONTRACT.md`). All ranges/positions in meters, angles in
degrees at the config layer (converted to radians internally), velocities
in m/s.

**Vertical FOV is a cone measured from nadir (straight down), not from
the horizontal plane.** An earlier draft of this measured elevation from
horizontal, which perversely made a target *directly below* the drone
harder to see than one far away at a shallow angle - backwards for a
downward-looking SAR camera. Measuring from nadir means directly-below
targets are always inside any positive `victim_sensor_vfov_deg`, and the
cone narrows the sensor's effective footprint as range grows relative to
altitude. The default (`140°` full-angle) is wide enough that a target at
`victim_sensor_range_m` and the default `flight_altitude` still falls
inside the cone; a narrower value is a real, tested constraint (see
`tests/test_sensors.py::test_narrow_vertical_fov_excludes_far_shallow_target`).

**Facing direction** for FOV purposes is each drone's own *movement
heading* (`SwarmController.headings[i]`, read by `mission.py` and handed
to the sensor calls *before* `step()` runs, then read again by `step()`
itself for interpreting the returned range scan against the same bin
layout) — not airframe yaw. This sim holds yaw at a fixed 0 setpoint (see
`mission.py`'s PID call), so using true yaw as a facing direction would
make every drone's sensor always point along world +X regardless of
travel direction. Movement heading is the physically sensible proxy given
that constraint; see Known Limitations.

## Timestamp and latency semantics

- `SensorObservation.sensor_timestamp_s`/`sensor_latency_s` describe the
  **victim-detection channel** specifically: `sensor_timestamp_s = t -
  victim_sensor_latency_steps * dt`, `sensor_latency_s =
  victim_sensor_latency_steps * dt`.
- The **obstacle range-scan channel** (`range_returns_m`/`occluded`) has
  its own, independently configurable latency
  (`obstacle_sensor_latency_steps`), applied internally by
  `ObstacleRangeSensor` before the tuple is even constructed. The
  contract's single `sensor_timestamp_s`/`sensor_latency_s` pair does not
  additionally describe this channel — a documented simplification from
  reusing one `SensorObservation` for two channels with potentially
  different latencies, rather than reopening the (already-reviewed) Phase
  1 contract. See Known Limitations.
- `NeighborObservation`'s own `sample_timestamp_s`/`delivery_timestamp_s`/
  `packet_age_s` are exact and independently timestamped per neighbor per
  Phase 1's contract - no simplification needed there.

## Dropout semantics

- Victim detection: whole-observation dropout, explicit `dropout: bool`.
- Obstacle scan: fail-empty (`+inf` everywhere), no separate flag.
- Neighbor sensing: per-(receiver, sender) - a dropped tick simply doesn't
  refresh that pair's cached reading; the old one persists and eventually
  becomes `stale=True` rather than disappearing outright.

## False-positive model

A false positive is generated by the *same* code path and *same* ID
counter as a genuine detection (see `VictimSensorModel.sense`) - the only
difference is a lower default `confidence`
(`victim_sensor_false_positive_confidence` vs `victim_sensor_confidence`).
This is a deliberate, documented simplification: a real object detector's
false positives often do carry lower average confidence than genuine
detections (a partial glimpse of driftwood vs. a clear sighting), so this
is a defensible statistical correlation, not a "tell" - nothing in the
`DetectionCandidate` a controller/`ConsensusBoard` receives marks it as
fake, and `ConsensusBoard`'s quorum logic (unchanged) has no way to
distinguish them either, exactly as intended.

## Noise distributions

All position/velocity noise is i.i.d. Gaussian (`rng.normal(scale=...)`),
per axis where relevant. No correlated-false-positive model exists yet
(two drones independently hallucinating the *same* false location is
possible only by coincidence, not by a shared bias) - that is explicitly
a later phase's scope (the distributed-consensus migration's "correlated
false positives" test list).

## RNG streams

Three independent `np.random.Generator` streams
(`swarm_sim.seeding.SeedManager`, from Phase 1), scoped to
`"victim_sensor"`, `"obstacle_sensor"`, `"neighbor_sensor"` — never one
shared stream. `mission.py` creates one `SeedManager(config.seed)` just
for these three; the rest of the mission's existing single shared `rng`
(obstacle/victim placement, `CommsNetwork`, `SwarmController`'s internal
randomness) is untouched, since migrating *that* onto `SeedManager` was
explicitly deferred in Phase 1 and remains out of Phase 2's scope too.

## Known limitations

- **Own-state estimation is currently perfect.** `own_state` is built
  directly from PyBullet's exact telemetry - no EKF-realistic pose
  uncertainty is modeled yet for a drone's own position/velocity (only
  for what it senses about *others*/victims). `pose_uncertainty_m` is
  always `0.0`.
- **Obstacle-scan `occluded` is always `False`.** Self-occlusion is
  already encoded in which obstacle's edge a bin's range value reports
  (nearest-hit-only); there's no *additional* per-bin occlusion signal in
  this simplified range-scan model.
- **Obstacle-scan latency isn't separately visible in
  `SensorObservation`'s timestamp fields** (see Timestamp semantics
  above) - it's applied internally but not exposed as its own field.
- **Facing direction is movement heading, not airframe yaw** (see
  Coordinate frame above) - a consequence of this sim not modeling yaw
  dynamics at all yet.
- **No correlated false positives.** Each drone's false-positive roll is
  independent; nothing models a shared environmental cause (e.g. the same
  floating debris fooling multiple drones' sensors identically).
- **The false-negative/false-positive/dropout/occlusion mechanisms are
  all independent random rolls** - no modeling of, e.g., detection
  probability degrading smoothly with range rather than being a hard
  cutoff, or false-negative probability rising near the FOV/range edge.
  A hard cutoff was chosen for testability (see `tests/test_sensors.py`)
  over a smoother, harder-to-verify model.
- **The scenario-report classification metrics are diagnostic, not
  exact.** `_classify_detections_for_scoring`'s "observation opportunity"
  count is per (drone, victim, tick) and can double-count when multiple
  drones observe the same victim in the same tick - it is a useful rate
  indicator, not a rigorously unique partition of outcomes.
- **No independent safety supervisor exists yet** (Phase 4). A stale,
  dropped-out, or noisy sensor reading today changes what the controller
  decides; nothing else checks whether that decision was actually safe.

## Testing

`tests/test_sensors.py` (28 tests): range/FOV/occlusion gating in
isolation, false-negative/false-positive forcing, deterministic noise
under a fixed seed (and divergent noise under a different one), whole-
observation dropout, exact-latency delivery timing, candidate-ID scheme
never encoding victim identity (including a same-position-different-
hidden-index swap test), the `DetectionCandidate` contract having no
ground-truth-shaped field at all, obstacle self-occlusion via nearest-hit-
only ray casting, neighbor range/FOV enforcement, non-ground-truth
measured neighbor positions, neighbor latency timestamp correctness,
stale-marking after a dropout run, and independent RNG streams (both at
the sensor-model level and via `SeedManager` directly).

`tests/test_controller_ground_truth_isolation.py` (7 tests): an AST-based
static guard that fails the build if `controller.py` ever references
`victims_ground_truth`/`WorldState`/`victims`/`obstacles`/`mission` as an
identifier, or takes a parameter literally named `positions`/`velocities`;
signature checks confirming `SwarmController.__init__` has no `obstacles`
parameter and `step()` has no `positions`/`velocities` parameter; and the
core behavioral proof - two `SwarmController` instances fed the exact same
sensed inputs (own_state/sensor_observation/neighbor_observations),
generated from two *different* hidden worlds, produce identical output.

Run:
```bash
python -m compileall -q swarm_sim run_mission.py tests
python -m pytest -q
```
