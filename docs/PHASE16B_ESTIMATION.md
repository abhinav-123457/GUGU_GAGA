# Phase 16B: GNSS-denied state estimation, drift and covariance

**Status: built and tested. `FloodSearchMission` can now run in
`localization_mode="estimated"`: every drone acts on its own dead-reckoned
estimate, and ground truth is confined to the plant-side sensor models and to
scoring. The default (`truth_state`) is the legacy pipeline, bit-for-bit
unchanged.** See [PHASE16_GNSS_DENIED_ROADMAP.md](PHASE16_GNSS_DENIED_ROADMAP.md)
for why, and [PHASE16A_MISSION_FRAME_GRID.md](PHASE16A_MISSION_FRAME_GRID.md)
for the frame and grid this builds on.

## What this can and cannot claim

The simulation validates how the *swarm* copes with **an assumed, parameterised
error model**. It does **not** validate the accuracy of real visual-inertial,
optical-flow or LiDAR odometry: every drift number in
`swarm_sim/estimation/profiles.py` is illustrative and not hardware-verified.
Real numbers need bench, hardware-in-the-loop and tethered/indoor flight.

## The truth boundary

```
 PyBullet truth ──► PlantTruth ──► OdometrySuite ──► OdometryBundle ──► StateEstimator ──► EstimatedState
   (plant side)     plant_truth.py   plant_odometry.py    models.py          estimator.py       models.py
                         │                                                                          │
                         ├──► victim / obstacle / neighbour sensor models                            ▼
                         ├──► CommsNetwork link geometry (who can hear whom)              _AutonomyView.positions
                         ├──► PID / physics                                                          │
                         └──► scoring (LocalizationScorer, GeofenceExcursionScorer)                  ▼
                                                                          controller · safety · recruitment · consensus · radio payload
```

`FloodSearchMission._autonomy_view()` is the only place truth becomes the
drones' belief. `tests/test_phase16b_architecture.py` proves the split with AST
checks (not substring matching):

- `PlantTruth`, `plant_truth`, `plant_odometry`, `metrics`, `OdometrySuite` and
  the scorers appear in **no** autonomy module (controller, safety supervisor,
  recruitment, both consensus modules, network, speed control, every behaviour,
  `mapping/`, and the truth-free half of `estimation/`).
- No identifier in those modules contains `gps`, `gnss`, `latitude` or
  `longitude` - a structural proof that GNSS is not an input anywhere a
  decision is made.
- Only `mission.py` and `estimation/plant_odometry.py` import `plant_truth`;
  only `mission.py` imports `plant_odometry` and `metrics`.
- `run()` binds and reads no bare `positions` / `velocities` / `rpys`; the names
  `obs` and `truth` appear in the arguments of **none** of the autonomy calls
  (controller step, supervisor evaluate, beacon board, consensus, adapter,
  `VehicleState` / `SensorObservation` / `MissionContext` construction); the
  radio payload, beacon service and own-state are fed from `view`; only
  `_autonomy_view` and `_score_localization` ever receive the whole `truth`.

These tests were **mutation-checked**: injecting each of four leaks (truth into
`service_check`, truth into `VehicleState`, a bare `positions` array back into
`run()`, `PlantTruth` imported by the safety supervisor) made the intended tests
fail, and the files were restored.

## Plant-side odometry (`estimation/plant_odometry.py`)

Per drone and per odometry source there are three hidden random walks -
scale-factor error `k`, body-frame velocity bias `b`, yaw-rate bias `w` - plus
white noise, dropouts, and "degenerate" ticks (low texture / featureless) in
which noise and drift steps are multiplied. A dropout loses the
**translational** measurement only; the heading-rate channel is the IMU
gyro's and keeps being reported (see finding 6). Over one tick `dt`:

```
dp_meas   = (1 + k) * dp_true_body + b * dt + sigma_v * dt * n
v_meas    = (1 + k) * v_true_body + b + sigma_v * n
dyaw_meas = dpsi_true + w * dt + sigma_w * dt * n
vz_meas   = vz_true + sigma_vz * n          # own, much smaller density: see finding 7
```

`dp_true_body` is the exact displacement rotated into the previous-tick body
frame, so the `ideal` profile reproduces truth to ~1e-14. A downward
rangefinder gives height; an IMU gives roll and pitch. Each (sensor, drone) has
its own RNG stream `odometry/<sensor>/drone<i>` on the mission's `SeedManager`,
a fixed number of draws is made per tick regardless of dropouts, and creating
the suite consumes nothing from any existing stream.

Each drone also starts knowing its own spawn pose only to the profile's
launch-slot accuracy (`init_pos_sigma_m`, `init_yaw_sigma_rad`), drawn plant-side.

### Profiles (illustrative, not hardware-verified)

| profile | sensors | note |
|---|---|---|
| `ideal` | one perfect source | reference for drift effects |
| `vio` | camera + IMU | scale RW 1e-3, bias RW 1e-3 m/s/sqrt(s), yaw-bias RW 1.5e-5 rad/s/sqrt(s) |
| `flow_rf` | optical flow + rangefinder | roughly 3x worse scale/bias, low-texture degeneracy 5 % |
| `lidar` | LiDAR odometry | best white noise, featureless-terrain degeneracy 3 % x5 |
| `fused` | vio + flow_rf + lidar | inverse-variance fusion (weights from white noise only) |
| `stress` | one poor source | 10 % dropout, 10 % degenerate ticks; exercises the validity gate |

Vertical-speed noise `sigma_vz` is a separate parameter per sensor (0.02 m/s
`vio` and `lidar`, 0.03 `flow_rf`, 0.05 `stress`, 0 `ideal`), not the
horizontal 0.03-0.15 m/s velocity noise: a real autopilot's vertical rate is a
filtered IMU / barometer / rangefinder estimate. The mission is sensitive to it
(finding 7), so it must stay small until 16C gives the mission a real altitude
hold. These values are as illustrative as the rest.

Measured on this simulator with the estimator alone (90 s at 2.5 m/s, 225 m
path, 24 Hz, 12 seeds): mean final position error 1.8 m (0.8 % of path) for
`vio` / `lidar` / `fused`, 5.9 m (2.6 %) for `flow_rf`, 9.5 m (4.2 %) for
`stress`. **That is 1.7-1.8 grid cells after a 90 s flight even for the good
sensors**, which is why cell-level survivor reporting will need aiding (16F) and
a probabilistic cell assignment (16D).

## The estimator (`estimation/estimator.py`)

A 7-state EKF `x = [px, py, psi, bx, by, s, wb]` (position, heading, body-frame
velocity bias, scale error, yaw-rate bias). Per tick, with fused body-frame
displacement `dp` and yaw change `dpsi`:

```
d      = dp - b*dt              m = 1 - s          u = R(psi) d
p'     = p + m*u
psi'   = psi + dpsi - wb*dt
b, s, wb: random walks (mean stays 0 until an external fix arrives)

F: dp'/dpsi = m R'(psi) d     dp'/db = -m R dt     dp'/ds = -u     dpsi'/dwb = -dt
P' = F (P + Q_walk) F^T + Q_white
```

The hidden-state random-walk noise enters *before* `F` because that is the order
in which the plant generates it. Bias, scale and yaw-bias are unobservable from
odometry alone, so position variance grows faster than linearly (tested on a
straight path; on a curved path the errors partly cancel as the heading turns).
Height is a scalar KF from the rangefinder, propagated with the fused vertical
velocity. Multiple valid sources are fused by inverse white-noise variance; the
implied random-walk variances combine as `sum(w^2 q)`. Translation and
heading are fused separately (translation over the sources whose visual/scan
tracking is up, heading over every source that still reports its gyro). When
translation is lost, the last velocity is extrapolated and its uncertainty grows
at an assumed 2 m/s^2, while the heading keeps integrating.

The estimator knows only the sensors' **nominal** noise densities times
`assumed_noise_scale`. It never sees the realised drift, the dropout schedule, or
which ticks were degenerate, so it can be - and is, measurably - over-confident.

`apply_position_fix(xy, sigma, t, gate_nis=None)` is a Joseph-form EKF update
(with an optional NIS gate) for external aiding such as a launch-zone fiducial.
It is unit-tested here and **not wired into the mission until 16F**.

### Validity, degraded, uncertainty

- `pose_uncertainty_m` = `sqrt(lambda_max(P_xy))`: the 1-sigma horizontal
  uncertainty along the worst axis.
- `valid` = finite, positive semi-definite, `sigma <= sigma_invalid_m` (5 m) and
  odometry no older than `odometry_timeout_s` (1 s); `degraded` = valid and
  `sigma >= sigma_degraded_m` (1 m). Both thresholds are illustrative.
- The bridge never reports `HealthState.FAILED` (which would make the supervisor
  *abort*); an untrustworthy estimate becomes `DEGRADED` plus
  `estimator_valid=False`, which takes the supervisor's own SAFE_HOLD path. 16B
  keeps that behaviour; a proper policy arrives in 16F.

## How the mission uses it

- **Own state** comes from the estimate; the drone's roll/pitch are the IMU's,
  its yaw is the estimated yaw.
- **Body-relative measurements** (victim camera, neighbour proximity sensor) are
  placed into the drone's own frame using its own estimate:
  `reported = own_estimate + R(yaw_error) * (relative vector + noise)`. A drone's
  drift therefore shifts every geotag it reports (verified to be an exact
  translation), and an observer's own drift **cancels** out of the neighbour
  separation the supervisor uses (verified for offsets to 40 m and heading errors
  to 2 rad).
- **Commands** are in the drone's own frame. A real autopilot closes its loop on
  its own estimated velocity, so the motion that actually happens is the
  command rotated by `true_yaw - estimated_yaw`: this is how heading drift bends
  the real flight path. Sensors are pointed the same way.
- **Radio**: which drones can hear each other still follows true positions (a
  radio is physical); what each broadcasts about itself is its estimate.
- **Beacons**: in `estimated` mode every consensus confirmation becomes a
  beacon, because a drone cannot know a confirmation is false. The legacy
  pipeline only announces confirmations that match a real victim - a use of
  ground truth kept, and documented, only to leave `truth_state` unchanged.
- **Covariance-aware geofence** (`safety_pose_sigma_geofence_k`, default 0): the
  "approaching the boundary" response triggers at
  `margin + min(k * pose_sigma, cap)`. The already-outside response is *not*
  inflated: an uncertain estimate is not "outside", and inflating it would pin a
  drone at the boundary forever. `pose_sigma` arrives through the existing
  `SensorObservation.pose_uncertainty_m` (which the supervisor did not read
  before); the supervisor imports nothing from `estimation`.
- **Scoring only**: `LocalizationScorer` (error, drift as a fraction of path,
  NEES, estimated-vs-true 1x1 m cell, invalid/degraded ticks) and
  `GeofenceExcursionScorer` (how often and how far drones really leave the
  geofence, in both modes).

## Verified (tests and measurement)

| claim | evidence |
|---|---|
| Default `truth_state` is unchanged | telemetry SHA-256 over 1152 records and all detection/safety counts identical between commit `7cd17b6` and the wired tree on the same seed; the pre-existing suite passes |
| The estimator is statistically consistent when its model matches | mean NEES over 60 runs: 1.95 (`vio`), 1.64 (`fused`), 1.50 (`lidar`), 2.08 (`flow_rf`); theory says 2. With the profile's unmodelled dropouts and degenerate ticks switched on, `vio` is still 1.98 |
| ... and the test can detect inconsistency | mean NEES 21.7 at `assumed_noise_scale=0.3`, 0.22 at 3.0; `stress` (unmodelled dropouts + degeneracy) 5.3 |
| `ideal` reproduces truth | estimate error 1.4e-14 over a full mission; controller receives truth to 1e-9; positions match the legacy pipeline to 1e-6 for the first 2 s |
| Covariance grows without aiding; a fix shrinks it | tests: >2x growth over a doubled straight path; a 5 cm fix cuts sigma by >70 % and also heading uncertainty |
| The controller and supervisor act on the estimate | spy tests on `controller.step` / `safety.evaluate`: estimate differs from truth, error matches the scorer, `pose_uncertainty_m` grows |
| A dropout no longer loses heading | on a wobbling-yaw path with 30 % dropouts the final yaw error is < 0.05 rad over 8 seeds; the estimator integrates heading through a translation-lost bundle exactly; mutation-checked (bug reintroduced: mean NEES 138, three tests fail) |
| Determinism and stream isolation | same seed identical telemetry; the extra RNG streams are all named `odometry/...` and existing streams keep their seeds |
| The boundary holds | AST tests above, mutation-checked |
| Vertical-speed noise does not crash the swarm | a 14 s `flow_rf` mission (4 drones, seed 4) keeps every drone between 1.5 m and 6 m with no ground contact; with the old vertical noise the same run climbed to 10.3 m and had 129 contact steps (mutation-checked) |

## Findings made while building this

1. **The swarm simulation is chaotic at float precision.** With the `ideal`
   profile the estimate equals truth to 1e-14, the first ticks are bit-identical,
   and the trajectory then diverges from `truth_state` to metres within ~12 s.
   Parity is therefore a *short-horizon* property (tested to 1e-6 for 2 s), and
   any single-seed comparison of drift effects is unreliable; means over seeds
   are needed (the scenario runner does this).
2. **`ideal` estimated is a different swarm from `truth_state`** because
   beacons are no longer truth-gated (a drone cannot know a consensus is false).
   The `ideal` profile, not `truth_state`, is the reference for drift.
3. **A rangefinder-fusion bug**: with a noise-free rangefinder the height filter
   blended prediction and measurement 50/50 on the first tick, leaking 1.8e-7 m
   (from the initial 9 mm fall) into altitude and breaking short-horizon parity.
   Fixed by flooring the measurement variance far below the process noise.
4. **Where the frame mapping is applied is constrained by an existing test**
   (`test_safety_supervisor_architecture.py` requires `to_velocity_command` to be
   called with `safe_vel` and forbids other assignments to it), so the rotation
   is applied to the speed governor's *output*, which is exact because rotation
   preserves the speed cap.
5. **The supervisor never read `pose_uncertainty_m`** (it existed on the
   contract, hard-coded to 0.0); 16B is the first user of it.
6. **A dropout was discarding the heading change** - found only through a
   mission-level diagnostic, not by the standalone tests. In the first
   scenario sweep the estimated yaw of a `vio` drone drifted 0.14-0.33 rad in
   30 s although the modelled yaw-bias could explain < 0.01 rad. Cause: a
   dropped tick reported nothing, including the heading change, so every
   dropout permanently lost that tick's yaw increment. The standalone tests
   used a constant-yaw path (yaw increments always 0) and could not see it; the
   real simulator's drones wobble in yaw by up to ~0.25 rad. Fixed: a dropout
   loses translation only (a real gyro keeps integrating when visual tracking
   drops). New tests use a wobbling-yaw path with dropouts; with the bug
   reintroduced they fail (mean NEES 138 versus 0.9) and pass with the fix. The
   first sweep's numbers were discarded.
7. **Vertical-speed noise, not horizontal drift, was crashing the swarm - and
   my first explanation was wrong.** In the sweep before this fix, 4 of 4
   `flow_rf` runs, 4 of 4 `stress` runs and 2 of 4 each `vio` / `lidar` /
   `fused` runs had a drone hit the ground within 30 s, against 0 of 4 for
   `ideal` and 1 of 4 for `truth_state`. A first version of this finding blamed
   the horizontal geofence response (most first contacts happen while the
   supervisor reports `GEOFENCE_RISK`). A trace of one crash refuted that: the
   drone was 15 m inside a 40 m arena with a 0.15 m position error. It sank from
   3.1 m at 3.6 s while the supervisor said `NORMAL`, reached the 0.5 m floor
   response at 6.9 s (which commands +2.5 m/s), overshot the 8 m ceiling to
   10.1 m, and the ceiling response's descent grew to -12 m/s into the ground at
   11.5 s. `GEOFENCE_RISK` was the altitude floor/ceiling, not the boundary.
   *Mechanism:* the plant gave the **vertical** speed the horizontal 0.05-0.15
   m/s white-noise density (x3-5 on degenerate ticks). The supervisor's accel
   limiter returns a vertical command proportional to the drone's own vertical
   speed, and the mission replaces the altitude setpoint by `z + vz_cmd*dt`
   whenever that command is non-zero. That is ~97 % of ticks in **every** mode,
   including the legacy `truth_state` pipeline, so altitude is damped rather
   than held, and noise on the vertical-speed reading is integrated into an
   altitude random walk. *Evidence:* giving the estimator the true vertical speed
   (everything else unchanged) took `flow_rf` from 4/4 crashed runs to 0/4 and
   `vio` to 0/4. *Fix:* vertical-speed noise is its own per-sensor parameter
   (0-0.05 m/s, see the profile note above). In 14 s missions the true altitude
   standard deviation is 0.05-0.09 m in `truth_state`, 0.06-0.10 m in `ideal`,
   0.27-0.39 m in `flow_rf` with the new density and 0.45-1.7 m with the old one.
   *What this does not fix:* the vertical channel is still open-loop against
   noise, only a small noise density keeps it benign, and a `truth_state`
   ground contact (seed 2, 21 s, floor response) shows the legacy pipeline has
   the same weakness. A real altitude hold decoupled from the horizontal
   safety limiter belongs with the flight-phase work (16C); it is out of scope
   for 16B and `truth_state` was deliberately left bit-for-bit unchanged.

## Results

30 s missions, 4 drones, 3 survivors, 2 obstacles, seeds 1-4, 40 m x 40 m arena
(`python scripts/run_phase16b_scenarios.py --num-seeds 4`, after the finding-7
fix; the earlier sweeps were discarded). `k` is `safety_pose_sigma_geofence_k`.

```
scenario           runs crash victims  locErr   worst  drift%    NEES NEESclean   cell% outside%  maxExc  GFrisk   holds  altMin  altMax
----------------------------------------------------------------------------------------------------------------------------------------
truth_state           4     1    1.25       -       -       -       -         -       -     0.00    0.00     104       4   -0.00   10.03
ideal/k=0             4     0    0.50    0.00    0.00    0.00    0.00      0.00  100.00     0.00    0.00       0      20    2.67    4.31
ideal/k=3             4     0    0.50    0.00    0.00    0.00    0.00      0.00  100.00     0.00    0.00       0      20    2.67    4.31
vio/k=0               4     0    0.75    0.13    0.52    0.66    2.50      2.50   83.91     0.00    0.00       7       0    0.75    4.48
vio/k=3               4     0    0.75    0.13    0.52    0.66    2.50      2.50   83.33     0.00    0.00      20       0    0.75    4.48
flow_rf/k=0           4     1    1.00    0.27    1.16    1.88    2.44      2.77   72.26     0.00    0.00       8      43    0.07    9.47
flow_rf/k=3           4     1    1.00    0.27    1.16    1.95    2.46      2.81   72.19     0.00    0.00      57      51    0.71    9.91
lidar/k=0             4     2    0.50    0.11    0.41    0.63    2.93      2.59   84.04     0.00    0.00      71      24    0.01    9.16
lidar/k=3             4     2    0.25    0.11    0.41    0.62    2.96      2.59   85.94     0.00    0.00      74      24    0.01    9.16
fused/k=0             4     2    1.00    0.11    0.40    0.61    2.55      1.14   87.82     0.00    0.00      56      36    0.00   10.04
fused/k=3             4     2    1.00    0.11    0.39    0.59    2.57      1.14   87.75     0.00    0.00      70      36    0.00   10.04
stress/k=0            4     4    1.00    0.70    6.92    3.42    2.53         -   53.62     5.42   34.48    1225      54   -0.09   10.32
stress/k=3            4     4    1.00    0.63    3.18    3.54    2.55         -   53.70     5.49    4.96    1230      52   -0.09   10.32
```

`crash` = runs (of 4) in which any drone touched anything: another drone, an
obstacle or the ground. `victims` = mean survivors confirmed of 3. `locErr` /
`worst` = mean / worst position error in metres. `drift%` = final error as a
share of path length. `NEES` = mean over all runs, `NEESclean` over runs without
contact (2 = consistent). `cell%` = share of ticks in which a drone's *estimated*
1 m x 1 m cell equals its *true* cell. `outside%` / `maxExc` = share of
drone-ticks truly outside the geofence and the worst true excursion (m).
`GFrisk` / `holds` = mean GEOFENCE_RISK / SAFE_HOLD ticks per run.
`altMin` / `altMax` = lowest / highest true altitude after the first 2 s
(cruise 3 m, geofence floor 0.5 m, ceiling 8 m). **Four seeds per row: read
these as orders of magnitude, not as rankings.**

**What the sweep supports**

- **Position error at 30 s** is 0.11-0.13 m for `vio` / `lidar` / `fused`, 0.27 m
  for `flow_rf` and 0.70 m for `stress` - 0.6-0.7 %, 1.9 % and 3.4 % of path, in
  line with the estimator-alone numbers above. `ideal` is exact (0.00).
- **Cell accuracy is already visibly wrong.** After only 30 s of flight a drone's
  estimated 1 m x 1 m cell is the true cell on 84-88 % of ticks with the good
  sensors, 72 % with `flow_rf` and 54 % with `stress`. Reporting survivors by
  cell without aiding will misreport a large share of them, which is what 16D
  and 16F have to solve.
- **Covariance consistency in the mission is only roughly right.** Mean NEES on
  runs without contact is 2.5-2.8 for `vio` / `flow_rf` / `lidar` (`fused`, 1.1,
  rests on 2 runs; 2 is consistent), i.e. mildly over-confident, and it is
  heterogeneous: single runs, with or without contact, range from 0.9 to 5.4
  (e.g. `lidar` seed 3, 4.2, with no contact). The standalone estimator tests
  give 1.5-2.1. I did not find the cause of the in-mission excess.
- **`ideal` never leaves the altitude band** (2.67-4.31 m) and has no contact in
  4 seeds, so with perfect odometry the estimated-mode plumbing itself is
  benign.

**What the sweep does not support**

- **No claim that the covariance-aware geofence helps.** With `k = 3` versus
  `k = 0` no drifting profile except `stress` ever left the geofence in either
  setting: 30 s of flight rarely reaches the edge of a 40 m arena, so this
  sweep does not exercise the feature. In `stress` all the excursions are one
  crashed run (seed 1: 402 contact steps, 10.3 m altitude): worst excursion
  34.5 m at `k = 0` versus 5.0 m at `k = 3`, and the outside share is 5.4 % versus
  5.5 %. That is one run, not evidence. An earlier exploratory 4-seed result that
  said `k = 3` removed excursions was produced before findings 6 and 7 and is
  withdrawn. `k = 3` does make the supervisor respond earlier (mean GEOFENCE_RISK
  ticks `flow_rf` 8 -> 57, `vio` 7 -> 20). Testing it properly needs longer
  missions or a smaller arena (16I).
- **No claim about how survivor search degrades with drift.** Survivors found
  (0.25-1.25 of 3) and contact rates are **confounded by a pre-existing weakness
  of the mission's vertical channel** (finding 7), so they cannot be read as an
  effect of localisation error. Over
  12 seeds of 30 s the legacy `truth_state` pipeline had a ground or other
  contact in 1 run and left the 0.5-8 m altitude band in 2; `ideal` 1 and 1;
  `lidar` 3 and 2; `fused` 4 and 3. In the 4-seed sweep every run with a ground
  contact had first climbed above the 8 m ceiling (`altMax` 9.2-10.3 m). 1/12 against 3/12 or 4/12
  cannot be told apart from the baseline at this sample size, so I cannot say
  whether horizontal drift raises the crash rate. `stress` (4 of 4 runs with
  contact, altitude up to 10.3 m) is the failure-mode profile and behaves like
  one, but I did not separate how much of that is its 10 % dropouts (a stale
  vertical speed is held during a dropout) versus its noise level.
- **First contacts after the fix** (18 first contacts, one per drone, in the
  4-seed census of the drifting profiles): 12 ground contacts, all with the
  supervisor in GEOFENCE_RISK - and, since every such run had also climbed above
  the ceiling, the altitude response rather than the horizontal boundary - 4
  obstacle contacts while the supervisor was in SAFE_HOLD (not diagnosed: a held
  drone should not reach an obstacle; possibly it was still moving when the
  estimate went invalid) and one drone-drone touch involving two drones at spawn
  (`flow_rf` seed 1, 0.42 s).

## Limitations

- **No aiding in the mission.** Landmark and cooperative aiding (16F) are not
  active, so error only grows. `apply_position_fix` exists and is tested.
- **The initial pose is known.** Each drone starts knowing its spawn pose to the
  launch-slot accuracy; drones still spawn airborne at random positions, not in
  the launch zone (16C).
- **Command mapping models heading error only.** Scale and bias errors are not
  fed back into the true motion (they would perturb it by ~1 %).
- **Confirmation scoring mixes frames.** `_match_victim` compares a consensus
  centroid in the drones' drifting frame with true victim positions, so under
  large drift a real confirmation can be scored a false confirmation (and any
  confirmation still becomes a beacon). Cell-based scoring arrives with 16D.
- **Consensus and recruitment are still centralised/global in places** and
  still use a shared frame for beacons; distributed belief is 16E.
- **The radio reports true link range** (`dist`), an idealised ranging
  measurement, to the flocking controller.
- **The mission's vertical channel is open-loop against noise** (finding 7). The
  legacy pipeline damps rather than holds altitude, so any noise on the
  estimated vertical speed becomes an altitude random walk, and mission-level
  outcomes (survivors found, contacts) cannot yet be attributed to localisation
  drift. Fix it in 16C (a real altitude hold decoupled from the horizontal
  safety limiter) before drawing conclusions from those metrics.
- **A crashed drone keeps being simulated and scored.** After ground contact
  a drone tumbles, planar odometry is meaningless (a real VIO would report
  tracking loss) and the estimator's covariance is no longer valid, so
  consistency statistics (NEES) are reported separately for runs without any
  contact.
- **Geofence only via a margin inflation.** There is no RTH into the launch
  zone, no comm-loss/low-battery policy under denial and no landing scoring
  (16F). An invalid estimate holds in place (SAFE_HOLD) forever.
- **`controller._boundary_repulsion` assumes an origin-centred arena.**
- **Sim mass is toy-scale**; the 25 kg budget is only checkable on declared masses.
- **Illustrative parameters.** See the top of this document.

## Files

New: `swarm_sim/plant_truth.py`; `swarm_sim/estimation/{profiles,models,plant_odometry,estimator,bridge,metrics}.py`;
`scripts/run_phase16b_scenarios.py`; `tests/test_phase16b_{estimation,components,mission,architecture,scenarios_script}.py`.
Changed: `config.py` (localisation and covariance-geofence fields, defaults
preserve legacy); `mission.py` (view, plant-side bridge, scoring, beacon policy,
map-derived geofence); `network.py` (payload vs link geometry; cached link
geometry for `send_message`); `sensors.py` (`rotate_xy`, optional placement
arguments); `safety_supervisor.py` (covariance-aware geofence, default off);
`mapping/frame.py` (relaxation flags on `centered_square`).

## Reproduce

```
python -m pytest tests/test_phase16b_*.py -q
python scripts/run_phase16b_scenarios.py --quick             # ~20 s smoke sweep
python scripts/run_phase16b_scenarios.py --num-seeds 4       # the sweep reported above
```
