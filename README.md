# Bio-Inspired Drone Swarm Simulation — Flood Search & Rescue

A physics-based simulation of a drone swarm searching a flooded area for
survivors, using coordination rules borrowed from collective animal
behavior. Built as a v1 "core algorithm engine": the coordination/search/
speed-control/telemetry logic, running on realistic quadrotor physics -
not a mission-specific dashboard or hardware integration.

## Biological grounding

Each paper you asked for maps to a specific, runnable piece of the system:

| Paper | Role in this system |
|---|---|
| Reynolds (1987), *Flocks, herds and schools* | `swarm_sim/behaviors/boids.py` - separation/alignment/cohesion, one of three selectable flocking models |
| Vicsek et al. (1995) | `swarm_sim/behaviors/vicsek.py` - noisy heading-averaging, a lighter-weight alternative flocking model |
| Couzin et al. (2002), *Collective memory and spatial sorting* | `swarm_sim/behaviors/couzin.py` - zonal repulsion/orientation/attraction with a field of view; the default flocking model |
| Bonabeau et al. (1996), self-organization in social insects | `swarm_sim/recruitment.py` - the recruitment signal decays like a stigmergic trace rather than broadcasting forever |
| Viswanathan et al. (1999), Levy-flight foraging | `swarm_sim/behaviors/levy_flight.py` - heavy-tailed step lengths for area search, proven near-optimal for sparse random targets |
| von Frisch (1967), bee waggle-dance | `swarm_sim/recruitment.py` - a drone that finds a victim recruits a bounded number of nearby swarm-mates to it |

## Architecture

```
swarm_sim/
  config.py        MissionConfig - every tunable parameter, one place
  behaviors/        boids.py, vicsek.py, couzin.py, levy_flight.py (pure functions, one per paper)
  network.py        CommsNetwork - simulated UAV<->UAV radio: range, packet loss, latency
  sensors.py        VictimSensorModel / ObstacleRangeSensor / NeighborSensorModel -
                     the only place ground truth is read to produce noisy/range-limited
                     SensorObservation/NeighborObservation contracts (see PHASE2_SENSING.md)
  contracts.py      Versioned data contracts (WorldState/VehicleState/SensorObservation/
                     NeighborObservation/Command/SafetyDecision) - see PHASE1_CONTRACT.md
  consensus.py       ConsensusBoard - multi-drone agreement before a detection becomes a beacon
  recruitment.py    RecruitmentBoard - von Frisch/Bonabeau-style beacon signaling
  speed_control.py  SpeedController - per-drone speed cap, real m/s, runtime-adjustable
  telemetry.py      TelemetryHub - every drone's state, every control step, in one place
  controller.py     SwarmController - composes the above into one decision per drone per step
  mission.py        FloodSearchMission - wires the swarm into a PyBullet quadrotor physics env
run_mission.py      CLI entry point
third_party/gym-pybullet-drones/   vendored physics + PID control library (see Setup)
```

### Communication network and consensus

Flocking coordination (boids/vicsek/couzin) no longer reads every other
drone's exact live position out of a shared array. It goes through
`CommsNetwork`: a radio model with a range limit (`communication_radius`),
distance-dependent packet loss (`comm_dropout_base` at zero range, rising to
`comm_dropout_at_max_range` at max range), and fixed message latency
(`comm_latency_steps`). A drone only flocks with neighbors it currently has
a live link to, using their last *received* position/velocity - not
instantaneous ground truth. Degrading the network (`--comm-range`,
`--packet-loss`) visibly loosens flocking cohesion, which you can watch
happen via each drone's `num_neighbors` telemetry field and the mission's
`swarm_connectivity_fraction` summary metric.

**Collision-avoidance proximity sensing deliberately does not go through
this network** (see `SwarmController._avoidance_vector`/`_neighbors`) - a
real drone senses an imminent collision with an onboard sensor
(vision/lidar/radar), not a radio link, so a degraded comms network can
never be the thing that degrades hard safety. This mirrors the
architecture principle of keeping safety-critical control below the
experimental swarm-intelligence layer.

Victim detection is no longer instant/exact either. `swarm_sim/sensors.py`'s
`VictimSensorModel` gates each drone's sensing by range, horizontal/vertical
field of view, and obstacle occlusion, then applies Gaussian position noise,
a false-negative probability, an independent false-positive probability,
whole-observation dropout, and fixed latency, before anything reaches
`ConsensusBoard` - see `docs/PHASE2_SENSING.md` for the full model. A
location only becomes an actionable beacon once `consensus_quorum`
independent drones' reports agree (within `consensus_cluster_radius`, inside
a `consensus_window_sec` time window) - one drone's noisy reading is never
enough on its own. Reports that cluster together but don't match any real
victim are counted as `false_confirmations` in the mission summary.

Collision avoidance and obstacle repulsion are sensed the same way -
`ObstacleRangeSensor` (a fixed-angular-bin range scan) and
`NeighborSensorModel` (onboard proximity sensing, independent of
`CommsNetwork`) replace what used to be a direct read of ground-truth
obstacle/drone positions. `SwarmController` itself never receives ground
truth at all now - enforced by both a static guard and a behavioral test,
see `docs/PHASE2_SENSING.md`.

This is bookkeeping-level consensus (`ConsensusBoard` sees all reports
directly, the same way `RecruitmentBoard` already worked), not a
peer-to-peer protocol where each drone carries its own belief state and
reports only propagate through `CommsNetwork` messages. That's the natural
next step if you want the consensus mechanism itself to be subject to the
same packet-loss/latency realism as flocking already is.

Physics comes from [`gym-pybullet-drones`](https://github.com/utiasDSL/gym-pybullet-drones)
(`CtrlAviary` + `DSLPIDControl`, driven directly rather than through its
`VelocityAviary` shortcut - see *Design notes* below for why). Each drone is
a real Crazyflie 2.X model (27g, thrust-to-weight 2.25, rated 30 km/h)
simulated with actual rigid-body dynamics, not a kinematic point-mass.

## Setup

Requires a dedicated Python environment - `gym-pybullet-drones`'s current
version needs Python >=3.12, and `pybullet` has no prebuilt wheel yet for
brand-new Python releases (it has to compile from source, which needs that
Python's dev headers). This repo was validated on **Python 3.12**.

`third_party/gym-pybullet-drones` is its own upstream git repo, not tracked
inside this one (see `.gitignore`) - clone it in place first:

```bash
git clone https://github.com/utiasDSL/gym-pybullet-drones third_party/gym-pybullet-drones
git -C third_party/gym-pybullet-drones checkout 7ebad1ecabd28a7000add2d05f888aa2e837c2cc

py -3.12 -m venv .venv
./.venv/Scripts/python.exe -m pip install -r requirements.txt
./.venv/Scripts/python.exe -m pip install -e third_party/gym-pybullet-drones --no-deps
```

For running this project's own test suite too (not required to run a
mission): `./.venv/Scripts/python.exe -m pip install -r requirements-dev.txt`,
then `./.venv/Scripts/python.exe -m pytest -q` (see `docs/PHASE1_CONTRACT.md`,
"Running tests", for the vendored package's own separate test command).

(`--no-deps` skips the library's own heavy RL-training extras - torch,
stable-baselines3, ray - none of which the physics/control classes used
here actually need.)

## Usage

```bash
./.venv/Scripts/python.exe run_mission.py --drones 6 --victims 5 --obstacles 4 --duration 90
```

Key options:
- `--flock-model {couzin,boids,vicsek}` - swap the coordination model
- `--max-speed`, `--cruise-speed` - per-run speed caps (m/s); `SpeedController.set_max_speed(drone_id, mps)` also allows changing an individual drone's cap at runtime
- `--comm-range`, `--packet-loss`, `--comm-latency` - stress the UAV<->UAV network (radio range, loss probability at max range, message delay in control steps); watch `swarm_connectivity_fraction` in the summary and `num_neighbors` in the telemetry CSV respond
- `--consensus-quorum` - how many independent drones must agree before a victim detection is confirmed (1 = old instant single-drone behavior)
- `--gui` - open the PyBullet viewer
- `--out` - telemetry CSV path (every drone, every control step: position, velocity, speed, orientation, mode, neighbor count)

## Design notes / known limitations

**Speed control** is real: each drone's velocity command is tracked in
actual m/s (not a fraction of some internal library scale), and
`SpeedController` enforces a per-drone cap you can change per drone,
per mission.

**Telemetry** is centralized: `TelemetryHub` receives every drone's full
state every control step, independent of anything mission-specific -
`all_latest()` gives a live snapshot, `save_csv()` persists the full log.

**Collision avoidance is a hard priority layer**, not a soft blend: getting
close to another drone or a physical obstacle overrides recruitment and
search alike, and slows the drone down rather than swerving at full speed.
This was a real fix, not a design preference - the simplified PID this
physics library uses derives its thrust-vector tilt directly from velocity
*tracking error*, so demanding a sharp full-speed direction change (dodging
at cruise speed) can itself induce enough tilt to collapse vertical thrust
and crash. Braking first keeps that error small. Validated stable across
10 random seeds at 120s each with no crashes; very close passes (well
under a drone-length) can still happen at higher swarm density than tested
here - a production system would want a larger safety margin, and ideally
should not rely on a single simplified PID for collision-critical braking.

**This is a research/prototype simulation of the coordination algorithms,
not a flight-ready or certified system.** `SwarmController` no longer sees
ground truth (`docs/PHASE2_SENSING.md`) - victim detection, obstacle
avoidance, and collision avoidance all go through range/FOV/noise/dropout/
latency-limited sensor models now - but a drone's own position/velocity
estimate is still perfect (no simulated GPS-denied localization/EKF drift
on *own* state yet, even though the paper folder this project started from
is full of exactly that literature), there's still no independent safety
supervisor to catch a bad sensor reading (Phase 4), battery/endurance
modeling doesn't exist, and nothing here has undergone regulatory/airspace
compliance or safety certification appropriate to flying multiple aircraft
near people. Treat this as the algorithm layer to build on, not the
finished product.
