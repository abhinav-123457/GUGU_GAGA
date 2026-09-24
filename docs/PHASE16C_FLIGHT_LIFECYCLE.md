# Phase 16C: vertical control, the plant-actuator boundary and the flight lifecycle

**Status: 16C-1 (vertical control) built, tested and gated (G1-G4 pass). 16C-2 (ground spawn in the
launch zone, `FlightPhase` machine, takeoff / RTH / landing, battery model) is not started.**
The default (`flight_control_mode="legacy"`) is the pre-16C pipeline, bit-for-bit. See
[PHASE16_GNSS_DENIED_ROADMAP.md](PHASE16_GNSS_DENIED_ROADMAP.md) for where this sits and
[PHASE16B_ESTIMATION.md](PHASE16B_ESTIMATION.md) (finding 7) for why it was needed.

## What this can and cannot claim

The simulation validates that the swarm's **vertical control** stays stable under an *assumed*
noise model for the drone's own altitude estimate. It does not validate a real autopilot: the
plant side is an idealised velocity-tracking PID with exact gravity feed-forward, no wind and no
mass error, and every gain and noise density here is illustrative and not hardware-verified.

## Why: what the code actually did

Phase 16B's mission-level results were confounded by crashes and altitude excursions (0 to 10 m
against a 3 m cruise) that were not caused by localisation drift. Reading the code shows three
compounding causes, of which the first was mis-stated when 16B was written up:

1. **The mission-candidate limiter still bounds the combined 3D acceleration.** Phase 4.1's
   independent horizontal/vertical bound lives in `bound_velocity_step`, which only the override
   tiers use. The candidate path (`safety_supervisor.py`, `_evaluate_mission_candidate`) bounds
   `|delta_3D| / dt`, so whenever the horizontal acceleration limit fires (nearly every tick) the
   vertical command becomes `own_vz * (1 - scale)`: **a fraction of the drone's own measured
   vertical speed**, non-zero on ~97 % of NORMAL ticks in every mode including `truth_state`.
2. **The plant re-anchors altitude to the truth whenever that command is non-zero.** At the end
   of the per-drone loop `target_z` is the fixed cruise altitude only if `|safe_vel[2]| <= 1e-9`;
   otherwise it is `true z + safe_vel[2] * dt`. So altitude is *damped*, never *held*.
3. **The floor/ceiling tiers are protection, not control** (fixed thresholds, +/-0.5-1x max speed)
   and separation/obstacle overrides emit `z = 0`. Noise on the vertical-speed estimate is
   integrated into an altitude random walk which those tiers then over-correct.

## Design

```
autonomy side (estimated state only)                       plant side (may read truth)
 EstimatedState -> VehicleState                              plant_actuator.SimulatedAutopilot
   -> flight.vertical.VerticalController                       velocity setpoint (own frame -> true frame)
        vz* = clip(kp (z_target - z_est)), slewed              -> DSLPIDControl (position target = true position,
   -> SwarmController horizontal candidate                        pure velocity tracking in x, y and z)
   -> SafetySupervisor (vertical axis independent)
   -> safe (vx, vy, vz)  ------------------------------------> rpm -> PyBullet
```

- **`flight/vertical.py`** (autonomy side): the outer altitude loop. `vz* = clip(kp (z_target -
  z_est), -max_descent, +max_climb)`, then slew-limited against **its own previous output**. The
  measured vertical speed is not an input. `z_est` is the drone's own estimate (the rangefinder
  filter in estimated mode, the physics altitude in `truth_state`).
- **`plant_actuator.py`** (plant side): `SimulatedAutopilot` wraps the per-drone `DSLPIDControl`.
  `mode="legacy"` reproduces the old altitude handling operation for operation; `mode="velocity"`
  sets the position target to the *current* true position so the airframe is driven only by the
  commanded velocity in all three axes. The PID is fed true state, which is fine for an actuator
  model but is now one explicit object (`FloodSearchMission.run()` never calls the PID itself;
  AST-tested), the single place a real PX4/ArduPilot autopilot would replace. It takes plain arrays
  and does not import `plant_truth`.
- **`safety_supervisor.py`**, two flags on `SafetySupervisorConfig`, both off by default:
  `independent_vertical_axis` bounds horizontal acceleration on the horizontal delta only and the
  vertical axis against the previous **commanded** vertical speed (never `own_vel[2]`);
  `preserve_candidate_vertical` makes separation, obstacle, horizontal-geofence and safe-point
  overrides keep the candidate's vertical speed instead of commanding zero. The floor/ceiling tiers
  still own the vertical axis. The supervisor imports nothing from `flight/`.
- **The altitude loop keeps running under a supervisor HOLD, ABORT or rejected command.** Those
  are mapped to "no horizontal motion"; in `altitude_hold` the vertical component then comes from
  the outer loop instead of zero (`mission.py`, `supervisor_hold`). Without it a drone held after a
  braking transient keeps whatever altitude sag the transient caused, indefinitely.
- **The simulated autopilot limits how fast its velocity setpoint may change**
  (`autopilot_max_accel_mps2`, default 4 m/s^2, horizontal magnitude and vertical component
  separately, `altitude_hold` only), as every real autopilot does. The mission turns HOLD / LAND /
  ABORT / rejection into an *instantaneous* zero-velocity step, and a 27 g airframe stopped from
  5 m/s in one tick rolls past 90 degrees and falls.
- **`flight_scoring.py`** (plant side, scoring only): true altitude statistics after a 2 s settle,
  both over all samples and over the samples **before each drone's first contact**, and a **contact
  root-cause attribution** (below).
- **Config** (`flight_control_mode`: `legacy` or `altitude_hold`; `altitude_kp_per_s`,
  `altitude_max_climb_mps`, `altitude_max_descent_mps`, `altitude_slew_mps2`,
  `autopilot_max_accel_mps2`); CLI `run_mission.py --flight-control altitude_hold`.

### Contact attribution

Each drone's **first** contact decides its root cause, because everything after it is a
consequence: an obstacle is `obstacle`, another drone is `swarm` (both drones), and the ground is
`vertical_control` **only if the drone was upright**; if it had been tilted past 1 rad (57 degrees)
in the preceding second it is `attitude_loss` (a tumble, e.g. an abrupt stop from speed, then a
fall). A later ground strike of a drone that had already touched something is counted separately
(`post_contact_ground_events`). This is bookkeeping, not proof: two contacts in one tick are
attributed in the order the physics engine reports them, and the 1 rad / 1 s tumble rule is a
heuristic.

## Spike results (throwaway script, before any design was frozen)

One CF2X drone, 24 Hz control, pure velocity tracking with the outer loop on a noisy altitude:

| test | result |
|---|---|
| hover at 3 m, no noise | 2.979-3.000 m, no horizontal drift |
| hover, altitude feedback noise 0.03 / 0.05 m | altitude std 5 / 9 mm |
| 3 -> 4 m step, kp 1.0, slew 1 m/s^2 | settles to +/-0.1 m in 2.6 s after the step, no overshoot |
| constant 0.05 m/s inner-loop velocity bias | steady offset 0.05 m (= bias / kp): there is no integral term |
| takeoff from the ground (z0 0.05 m) | lifts off after ~0.5 s, 2.85 m at 4.3 s, no overshoot; z0 0.02 m drifted 13 cm horizontally on the ground, so spawn at 0.05 m |
| landing 0.5 m/s, 0.25 m/s below 0.5 m | touchdown at 0.25 m/s; **cutting the motors at 0.10 m drops the drone at ~1.06 m/s**, so 16C-2 must keep descending to rest and disarm only when at rest, never in the air |

## Verified (tests and measurement)

| gate | evidence |
|---|---|
| **G1** legacy is unchanged | telemetry SHA-256 and all detection / safety / contact counts are identical to commit `df7e2ca` for five legacy configurations - `truth_state` (two seeds, one with a ground contact), `estimated`/`fused`, `estimated`/`flow_rf` and `estimated`/`stress` with the covariance geofence on - before and after all 16C-1 changes. The pre-existing suite passes |
| **G3** step response | closed loop in PyBullet (one drone): a 3 -> 4 m step settles to +/-0.1 m within 4 s, overshoot <= 0.2 m, final error < 2 cm, no horizontal drift; altitude std < 2 cm with 5 cm of noise on the altitude feedback |
| **G4** avoidance does not break the hold | 23 supervisor unit tests (overrides keep the candidate's vertical speed, the altitude tiers still override, the horizontal limit is unchanged, the measured vertical speed is ignored); a crowded scenario (6 drones, 14 m arena, 3 obstacles, `flow_rf`) has >= 20 override ticks and altitude within 30 cm of cruise on every one |
| mutation checks | 12 mutations (old combined-3D limiter, overrides zeroing vz, measured vz as reference, legacy re-anchoring, truth altitude fed to the loop, PID called from `run()`, no slew limit, no hold handling, no setpoint shaping, tumbles blamed on altitude, post-crash samples in the gate, gate reading all samples) each make at least one test fail; the files were restored |

**G2 - vertical-noise immunity** (`python scripts/run_phase16c_gate.py`, 12 seeds x 30 s, 4 drones,
3 survivors, 2 obstacles; `flow_rf_oldvz` / `stress_vz3` reproduce the Phase 16B failure with
vertical-speed noise of 0.10 / 0.15 m/s):

```
scenario                   runs contact  vert  tilt  obst swarm victims  preStd  preErr  preMin  preMax  allMin  allMax    G2
-----------------------------------------------------------------------------------------------------------------------------
legacy/truth_state           12       1     1     0     0     0    0.92   0.061   7.032    0.13   10.03   -0.00   10.03     -
legacy/ideal                 12       1     0     0     1     0    0.67   0.073   7.229    0.34   10.23    0.34   10.23     -
legacy/vio                   12       1     0     0     0     2    0.75   0.094   2.247    0.75    4.48    0.75    4.48     -
legacy/flow_rf               12       5     3     1     1     2    0.92   0.149   7.217   -0.01   10.22   -0.08   10.22     -
legacy/lidar                 12       3     0     1     1     2    0.58   0.116   6.157    0.25    9.16   -0.02    9.16     -
legacy/fused                 12       4     1     2     1     0    1.08   0.122   7.161    0.05   10.16    0.00   10.16     -
legacy/stress                12      12    10    22     2     0    0.92   1.709   7.318    0.01   10.32   -0.10   10.32     -
legacy/flow_rf_oldvz         12      10     4    13     0     2    0.67   0.736   7.255    0.04   10.26   -0.08   10.26     -
legacy/stress_vz3            12      12    16    24     0     0    0.83   2.158   7.217    0.01   10.22   -0.08   10.26     -
altitude_hold/truth_state    12       2     0     0     0     4    0.83   0.001   0.008    2.99    3.01    2.99    3.01  PASS
altitude_hold/ideal          12       0     0     0     0     0    1.00   0.001   0.013    2.99    3.01    2.99    3.01  PASS
altitude_hold/vio            12       1     0     0     1     0    0.67   0.003   0.014    2.99    3.01    2.99    3.01  PASS
altitude_hold/flow_rf        12       2     0     0     2     2    1.25   0.004   0.017    2.98    3.02    0.01    3.03  PASS
altitude_hold/lidar          12       1     0     0     1     0    0.67   0.002   0.019    2.98    3.01    2.98    3.01  PASS
altitude_hold/fused          12       1     0     0     0     2    0.83   0.003   0.023    2.98    3.01    2.90    3.01  PASS
altitude_hold/stress         12       0     0     0     0     0    0.83   0.007   0.027    2.97    3.03    2.97    3.03  PASS
altitude_hold/flow_rf_oldvz   12       3     0     0     1     5    0.83   0.005   0.079    2.92    3.02    0.01    3.02  PASS
altitude_hold/stress_vz3     12       2     0     0     2     0    1.00   0.009   0.047    2.95    3.03    2.95    3.03  PASS
```

`contact` = runs with any contact; `vert` / `tilt` / `obst` / `swarm` = drones whose **first**
contact was an upright ground strike / a tumble then a strike / an obstacle / another drone.
`preStd` = median per-run median-per-drone altitude std, `preErr` = worst |z - 3 m|, `preMin` /
`preMax` = altitude extremes, all **before each drone's first contact** and after a 2 s settle;
`allMin` / `allMax` are over every sample including post-crash ones. The gate criteria (on
`altitude_hold` rows): zero `vert` first contacts, `preErr` <= 0.5 m, `preStd` <= 0.10 m.

- **All nine `altitude_hold` scenarios pass.** Zero upright ground strikes and zero tumbles in 108
  runs; before its first contact no drone is ever more than 8 cm from cruise (worst: 7.9 cm with
  the old 0.10 m/s vertical noise), median altitude std 1-9 mm.
- **The same seeds in `legacy`** show the failure being fixed: altitude extremes of -0.1 to 10.3 m,
  median std 6-15 cm with the shipped noise (`stress`: 1.7 m) and worst-case errors of 2-7 m, and in the noisy cases
  (`flow_rf_oldvz`, `stress_vz3`) 4-16 upright ground strikes plus 13-24 tumbles.
- **What remains in `altitude_hold` is horizontal**: 12 of the 108 runs still have a contact (13
  drones' first contact was with another drone, 7 with an obstacle), which 16C does not address. The
  literal all-samples altitude band would fail for `flow_rf` and `flow_rf_oldvz` (`allMin` 0.01 m)
  because a drone that collides with another then falls; that is why the gate is evaluated
  before the first contact (finding 4). Survivors found (0.7-1.3 of 3 in 30 s) are reported, not
  interpreted.

## Findings

1. **The candidate-path limiter, not just the missing controller, was the coupling** (see Why).
   A first plan of "add an altitude controller" would have left the measured vertical speed inside
   the vertical command.
2. **My first version of this mode had a hole: a HOLD stopped the altitude loop.** The design
   note said hover on a HOLD is exact in this plant. It is not: the gate found `stress` seed 3
   sitting 0.87 m low with the supervisor in GEOFENCE_RISK (a HOLD), commanded vertical speed 0.
   Fixed by keeping the altitude loop running under HOLD / ABORT / rejection.
3. **An instantaneous stop from speed flips the airframe, and this is horizontal, not
   vertical.** Fixing (2) exposed the next failure on the same seed: a drone flying at 4.8 m/s in
   the supervisor's geofence escape was switched to HOLD (zero-velocity step in one control tick),
   rolled to -1.6 rad and fell. In isolation (one CF2X, no mission) an unshaped stop from 3 / 5 m/s
   tilts it 1.20 / 1.33 rad and sags it to 2.8 / 2.0 m; with the autopilot's 4 m/s^2 setpoint
   limit the same stops tilt it 0.48 / 0.46 rad and hold altitude (min z 3.00 / 2.99). This is
   the hazard Phase 4.1 fixed for the supervisor's velocity commands (`bound_velocity_step`); HOLD,
   LAND, ABORT and rejection were never bound because they are not velocity commands. It also
   showed that the ground contacts in the legacy `stress` sweeps are mostly tumbles, not altitude
   failures (finding 5).
4. **The gate criterion I first wrote was too coarse, and I changed it.** As planned (every drone
   within +/-0.5 m for the whole run) the gate failed four `altitude_hold` scenarios. Inspection
   found two causes: crash aftermath (a drone-drone collision, then a fall to the ground), and the
   hold hole (2), which fixing exposed the abrupt-stop flip (3) behind. After fixing (2) and (3)
   the two remaining literal failures (`flow_rf`, `flow_rf_oldvz`) are crash aftermath. The gate now judges altitude only **before each drone's first
   contact** (a collision's consequences are not evidence about the altitude controller), with the
   first-contact root cause reported separately. This cannot excuse a vertical failure: an upright
   ground strike is its own criterion, and an excursion before a contact is counted. Both the
   literal and the refined numbers are in `runs.csv`; the first, failing run is kept locally in
   `results/phase16c_gate/g2_first_attempt`.
5. **What crashed the legacy pipeline was a mixture.** Splitting the legacy ground-first contacts
   by the tumble rule: with the shipped noise `flow_rf` had 3 upright ground strikes and 1 tumble,
   `lidar` 0 and 1, `fused` 1 and 2; `stress` 10 and 22. The 16B write-up attributed the shipped-noise
   crashes to vertical noise (correct for the `flow_rf` trace it examined) and did not know about
   the tumbles. The right reading is: vertical-speed noise in an open-loop altitude channel, and
   abrupt stops from speed, both crash the legacy pipeline, and neither is localisation drift.
6. **Reference for the vertical bound.** With `independent_vertical_axis` the acceleration limit is
   applied against the previous commanded vertical speed, reset to 0 on a HOLD / LAND / ABORT /
   rejection (the mission commands zero then).

## Limitations

- **No takeoff, landing or RTH yet** (16C-2). Drones still spawn airborne at 3 m at random
  positions, and the mission-level results are still not measured against a launch zone.
- **The plant is idealised.** Exact gravity feed-forward, no wind, no motor lag or mass error. A
  real vertical velocity loop has bias, which this P-only outer loop turns into an altitude offset
  of `bias / kp`.
- **Horizontal problems are untouched.** Drone-drone and obstacle contacts (some while the supervisor
  is in SAFE_HOLD), drones held outside the geofence indefinitely, and the covariance-aware
  geofence's ineffective regime (16B) are unchanged; the attribution separates them from vertical
  failures instead of hiding them. A stopping drone is now decelerated at the autopilot's 4 m/s^2
  limit, but the supervisor still commands the 5 m/s geofence escape that makes those stops
  dangerous.
- **The gate was refined after seeing a failure** (finding 4). It is a fair measurement of the
  altitude controller, not a claim that no `altitude_hold` mission ever touches anything.
- **`legacy` still tumbles.** The setpoint limit applies only in `altitude_hold`; changing the
  legacy plant would change 1,383 pinned tests' behaviour.
- **Gains and noise densities are illustrative.** In particular the shipped vertical-speed noise
  (0.02-0.05 m/s) no longer matters to this mode; that is a property of the design, not evidence
  about real sensors.
- **The `legacy` mode is kept as the default** because 1,383+ existing tests pin its behaviour.
  Whether `altitude_hold` becomes the default is decided after 16C-2.

## Files

New: `swarm_sim/plant_actuator.py`, `swarm_sim/flight/{__init__,vertical}.py`,
`swarm_sim/flight_scoring.py`, `scripts/run_phase16c_gate.py`,
`tests/test_phase16c_{vertical,supervisor_vertical,mission,architecture,scoring,gate_script}.py`.
Changed: `config.py` (flight fields), `mission.py` (autopilot boundary, altitude command, hold
handling, scoring, results), `safety_supervisor.py` (two flags), `run_mission.py`
(`--flight-control`), `tests/test_phase16b_cli.py`.

## Reproduce

```
python -m pytest tests/test_phase16c_*.py -q
python scripts/run_phase16c_gate.py --quick                      # ~10 s smoke run
python scripts/run_phase16c_gate.py --num-seeds 12               # the G2 gate reported above (~15 min)
python run_mission.py --flight-control altitude_hold --localization estimated --estimator-profile flow_rf
```
