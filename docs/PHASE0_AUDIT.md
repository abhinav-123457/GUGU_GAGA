# Phase 0 — Repository Audit

Grounded against the actual code as of this commit (`swarm_sim/*.py`, `run_mission.py`,
`README.md`). File:line references point at the reviewed source.

**Not flight-certified. Nothing here proves real-world safety.** This document
describes what exists in simulation and where it diverges from what a real
deployment would require.

## 1. Architecture (current)

```
run_mission.py (CLI)
  -> FloodSearchMission (mission.py)
       -> CtrlAviary (gym-pybullet-drones)         [PyBullet physics, ground truth]
       -> DSLPIDControl x N                        [per-drone PID -> RPM]
       -> SwarmController (controller.py)          [candidate velocity setpoints]
            - _neighbors()          ground truth, safety/collision only
            - _perceived_state()    via CommsNetwork, flocking only
            - boids / vicsek / couzin behaviors
            - levy_flight search
            - _avoidance_vector()   hard override branch, still ground truth
       -> CommsNetwork (network.py)                [range / loss / latency radio model]
       -> ConsensusBoard (consensus.py)             [centralized report clustering]
       -> RecruitmentBoard (recruitment.py)         [beacon broadcast / decay]
       -> SpeedController (speed_control.py)        [per-drone speed cap]
       -> TelemetryHub (telemetry.py)               [central log]
```

Separately, outside this package (WSL, not in version control alongside
`swarm_sim/`): `swarm_bridge.py` hand-builds MAVLink `SET_POSITION_TARGET_LOCAL_NED`
messages from a raw Python loop, talking to ArduCopter SITL. It has no adapter
interface, no coordinate-frame documentation, no command rejection, and no
supervisor. This is the "claimed SITL/MAVLink bridge" that is absent from the
package itself.

**Key finding:** there is no safety-supervisor layer anywhere. `SwarmController`'s
output goes directly into `DSLPIDControl.computeControlFromState()` at
`mission.py:242-248`, and directly into `set_position_target_local_ned_send()` in
`swarm_bridge.py`. Collision avoidance is a *priority branch inside the planner*
(`controller.py:135-143`), not an independent layer capable of rejecting the
planner's output.

## 2. Data flow (per control step, `mission.py:215-261`)

1. `env.step(rpm_action)` → ground-truth `positions`, `rpys`, `velocities` (PyBullet)
2. `network.tick(positions, velocities)` — feeds ground truth as the "transmitted"
   payload into the radio model (the network model's loss/latency mechanics are
   real; the payload content is not degraded by any sensor/estimator noise)
3. `_check_detections(positions, t)` (`mission.py:164-191`) — victim *detectability*
   decided by ground-truth Euclidean distance (`mission.py:175-176`); only the
   *reported position* gets sensor noise added
4. `ConsensusBoard.try_confirm()` (`consensus.py:38-54`) — scans one global,
   instantly-visible list of every drone's reports; no latency/loss modeled on
   the reports themselves (unlike flocking data, which does go through
   `CommsNetwork`)
5. `controller.step(...)` — ground-truth `_neighbors()` for collision avoidance,
   `CommsNetwork`-filtered `_perceived_state()` for flocking, combined into
   `desired_vels`
6. `speed_ctrl.to_velocity_command()` — speed cap
7. `pid.computeControlFromState(target_pos=[x, y, fixed_altitude], target_rpy=[0,0,0], target_vel=...)`
   (`mission.py:241-248`) — **no supervisor gate**, `target_pos`'s x/y and z are
   built directly from ground truth, `target_rpy` is hardcoded to all-zero
8. `telemetry.record(...)`

Four logically separate "channels" exist today, and only one is actually
network-realistic:
| Channel | Goes through `CommsNetwork`? |
|---|---|
| Flocking neighbor data | Yes (`_perceived_state`) |
| Collision-avoidance proximity | No — ground truth by design (see leak #1) |
| Victim detection reports | No — straight to the global `ConsensusBoard` list |
| Recruitment beacons | No — straight to the global `RecruitmentBoard` dict |

## 3. Assumptions currently baked in (implicit, not documented as such before now)

- Perfect state estimation — no estimator-validity concept exists (no analog of
  `VehicleState.estimator_validity`)
- No command timestamps or expiry anywhere — `desired_vels` is a bare numpy array
- No battery model — no drain, no reserve, no failsafe
- No arming/health-state model — a drone is implicitly always "armed"
- No hard geofence — `arena_size` only drives a soft push-back force
  (`controller.py:59-67`), not a violation/ABORT state
- `ConsensusBoard` is single-authority/global, not distributed
- Single shared `np.random.default_rng(config.seed)` (`mission.py:41`) draws for
  obstacle placement, victim placement, network dropout, *and* all of
  `SwarmController`'s internal randomness — reproducible today, but not
  decomposed enough to isolate which random source drove a given outcome, and
  nothing records code version / map version / model version / event-log hash
  alongside the seed
- Zero project-level tests exist (no `tests/` dir, no test framework in
  `requirements.txt`) — every claim in `README.md` about validated stability is
  an anecdotal manual observation, not a regression test

## 4. Ground-truth leaks (concrete)

1. `controller.py:24-31` `_neighbors()` — literal ground-truth position array.
   Docstring calls it "onboard sensor," but there is no noise, no range-dependent
   per-return dropout, no blind spot/FOV, no latency. The *architectural*
   separation from `CommsNetwork` is correct; the *sensor model* itself doesn't
   exist yet.
2. `controller.py:69-77, 89-108` — obstacle positions (`self.obstacles`) are known
   exactly; no obstacle-sensor uncertainty.
3. `mission.py:175-176` — victim detectability gated on ground-truth distance
   only; no occlusion by the mission's own spawned obstacles, no FOV, no false
   positives, no missed-detection probability beyond a hard range cutoff.
4. `mission.py:241` — `target_pos` built directly from ground-truth
   `positions[i]`, fed straight back as PID feedback; no estimator in the loop.
5. `consensus.py:26-54` — `ConsensusBoard.reports` is one global Python list,
   scanned directly; no per-drone belief state, no message that had to survive
   `CommsNetwork`.
6. `network.py` itself is high-fidelity for what it's wired to, but detection
   reports (`mission.py:178`) and recruitment beacons (`recruitment.py`) never
   pass through it — only flocking data does.
7. `mission.py:52-54,74` — initial positions and victim placement share the same
   RNG stream as network dropout and behavior noise.

## 5. Missing safety states

None of the following exist in any form: `NORMAL`, `DEGRADED_SENSOR`,
`DEGRADED_LINK`, `LOW_BATTERY`, `LOST_AGENT`, `GEOFENCE_RISK`, `SEPARATION_RISK`,
`ABORT`, `SAFE_HOLD`, `RETURN_TO_SAFE_POINT`, `LAND_REQUESTED`. The entire safety
"state" surface today is one boolean, `repulsion_flags[i]`
(`controller.py:113,133`), distinguishing "actively avoiding" from "not." There
is no supervisor object; the closest analog is the
`if avoidance_active: ... continue` branch inside `SwarmController.step()`
(`controller.py:135-143`) — a priority rule inside the planner, not an
independent layer that can reject the planner's own output.

## 6. Missing tests

Zero project-level tests exist anywhere in the repo. Everything Phase 8 of the
implementation plan lists is currently absent, including the basics:
determinism/replay, accel-cap rate-limiter behavior, `ConsensusBoard` quorum
logic against adversarial inputs (duplicate `drone_id`, stale reports outside
`consensus_window_sec`), `CommsNetwork` dropout-probability-curve and
latency-delivery timing, and PyBullet contact/collision absence across seeds
(the README's "validated stable across 10 random seeds" claim is a manual
observation, not an automated regression test).

## 7. Sim-to-real risk register

| # | Risk | Where it lives today | Consequence if unaddressed | Addressed in |
|---|---|---|---|---|
| 1 | Ground-truth collision sensing | `controller.py` `_neighbors` | Controller tuned against perfect knowledge may be unsafe against a real sensor's noisy/blind-spot/latent picture | Phase 2, Phase 4 |
| 2 | No supervisor between planner and actuation | `mission.py:242-248`, `swarm_bridge.py` | Any planner bug or bad input becomes a motor/MAVLink command with zero independent check | Phase 4 |
| 3 | No command timestamp/expiry | `Command` has no code analog | A stale command (frozen loop, real network delay) would be sent to a real autopilot as if fresh | Phase 1, Phase 6 |
| 4 | No estimator-validity concept | positions/velocities always trusted exactly | Real EKF can diverge (GPS-denied, vibration, mag interference); nothing here knows to distrust its own state | Phase 1, Phase 4 |
| 5 | No battery/health model | absent | No reserve/return-to-base logic even conceptually, despite this being a SAR-endurance-critical mission | Phase 4 |
| 6 | Soft boundary only, no hard geofence | `controller.py:59-67` | A real geofence breach is a regulatory/safety incident, not a tuning issue | Phase 4 |
| 7 | `ConsensusBoard` is a centralized oracle | `consensus.py` | Any future "peer-local consensus" claim built on top of this without replacing it would be false; Phase 5 requires keeping this only as `CENTRALIZED_ORACLE` benchmark | Phase 5 |
| 8 | No coordinate-frame documentation | PyBullet frame (mission.py) vs MAVLink NED frame (swarm_bridge.py) are different, unstated conventions already | Silent frame mismatches are exactly how a sim-validated controller flies the wrong direction on real hardware — this was the literal cause of last session's `type_mask` bug | Phase 6 |
| 9 | `swarm_bridge.py` bypasses `swarm_sim`'s own abstractions | WSL script, outside the package, hand-built MAVLink | Confirmed missing from the "submitted archive"; any future PX4/STM32 path repeats ad hoc integration instead of reusing a tested adapter | Phase 6 |
| 10 | No test suite | repo-wide | Every stability/behavior claim is anecdotal; refactors have no regression safety net | Phase 8 |
| 11 | Single shared RNG seed across unrelated randomness sources | `mission.py:41` | Cannot isolate which random source caused a given failure; not a real "seed + code version + model version" reproducibility contract | Phase 1 |
| 12 | "Deterministic replay" is aspirational, not verified | `mission.py`, `config.py` (control_freq_hz vs sim_freq_hz substeps) | No config+seed+code-version+event-log hash is recorded anywhere | Phase 1 |

## 8. Explicit scope reminder

This audit and everything that follows it describes a research-quality
simulator. Passing any test suite built from this audit is not airworthiness,
not certification, and not permission to operate near people. Any future
hardware step must be prop-off, restrained, tethered, or indoor-supervised.
