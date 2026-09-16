# Phase 4: Independent Safety Supervisor

**This is not a certified collision-avoidance system.** It is a
simulation-scoped safety layer built from conservative heuristics over
noisy, latent, dropout-prone sensor data (Phase 2), evaluated once per
control step. It has no formal proof of correctness, has not been
verified against any airworthiness standard (DO-178C, ASTM F3269, or
otherwise), and every numeric threshold in `SafetySupervisorConfig` is a
documented engineering choice, not a certified limit. Nothing in this
document should be read as a safety guarantee for real flight.

## Pipeline

```
Sensor observations
      |
      v
SwarmController candidate command      <- CandidateCommand (untrusted, unvalidated - can be
      |                                    NaN, expired, wrong-shaped; see "why a separate type" below)
      v
Independent SafetySupervisor.evaluate()
      |
      v
SafetyDecision: accepted Command (velocity-setpoint/HOLD/LAND/ABORT) OR rejected (no command)
      |
      v
PyBullet (mission.py's DSLPIDControl call) or a future autopilot adapter
```

`SwarmController` (Phase 3's behavior models) and `SafetySupervisor`
(this phase) are separate modules, imported and called independently -
`safety_supervisor.py` imports nothing from `controller.py` or
`behaviors/`, and is never called from inside either. Every candidate a
behavior model produces passes through `evaluate()` before it can reach
`mission.py`'s PID/PyBullet call; see
`tests/test_safety_supervisor_architecture.py` for the AST-based proof
this ordering can't be silently bypassed by a future edit.

**Why `CandidateCommand` is not `swarm_sim.contracts.Command`**:
`Command.__post_init__` already validates finiteness at construction -
a NaN velocity would raise before ever reaching a safety check, which
would make "reject malformed commands" untestable and pointless.
`CandidateCommand` carries the same information without that validation,
specifically so this module's own validation has something real to
catch. Every `Command` this module *constructs* (as
`SafetyDecision.filtered_command`) goes through the full validated
contract type - nothing invalid can leave this module, whatever came in.

## Safety states

The 11 required states (`swarm_sim.contracts.SafetyState`, defined in
Phase 1, populated for the first time in Phase 4):

```
                    ┌─────────┐
        ┌───────────┤ NORMAL  ├───────────┐
        │           └────┬────┘           │
        │                │                │
   sensor dropout   link/estimator    proximity/geofence/
   this tick only    degrades         battery/contact
        │                │                │
        v                v                v
 DEGRADED_SENSOR   DEGRADED_LINK   GEOFENCE_RISK / SEPARATION_RISK
        │           (own link          / SAFE_HOLD / LOW_BATTERY
        │            stale too long)        │
        │                │                  │
        │        sustained isolation        │
        │        from swarm graph           │
        │                v                  │
        │           LOST_AGENT              │
        │                │                  │
        │                └──────┬───────────┘
        │                       v
        │           battery critical -> LAND_REQUESTED
        │           link/battery + safe point known -> RETURN_TO_SAFE_POINT
        │                       │
        └───────────────────────┴──────────────┐
                                                 v
                          operator_abort OR health_state==FAILED
                                                 │
                                                 v
                                              ABORT  (highest priority - preempts everything above)
```

Not a strict linear FSM: which state appears on a given tick is
recomputed from scratch every `evaluate()` call by walking the 9-tier
priority list below and returning at the first tier that fires (or
`NORMAL`/`DEGRADED_SENSOR` if none do) - there is no persistent "current
state" driving transitions, except for the handful of things that
genuinely need memory across ticks (documented under "Stateful
elements" below). This matches "independently callable, testable" better
than a hand-rolled state machine would: every decision is a pure
function of (inputs, that memory), reproducible in a unit test.

## Priority policy (9 tiers, evaluated in this order, first match wins)

1. **Operator abort / emergency state** - `mission_context.operator_abort`
   or `own_state.health_state == FAILED` → `ABORT`, command type `ABORT`.
2. **Geofence and keep-out zones** (horizontal only - see "Assumptions")
   - outside the box → `HOLD` (never autonomously re-enter across
   unknown terrain); within `geofence_margin_m` → inward-biased velocity.
   → `GEOFENCE_RISK`.
3. **Inter-drone separation** - uncertainty- and closing-velocity-
   lookahead-inflated body clearance below `separation_hard_margin_m` →
   repulsion velocity away from the nearest offending neighbor. →
   `SEPARATION_RISK`.
4. **Obstacle avoidance + stuck/wedging response** - same lookahead
   logic against the sensed range scan, plus the Phase 2 wedging fix
   (see below). → `SAFE_HOLD`.
5. **Altitude floor and ceiling** - mapped onto `GEOFENCE_RISK` (see
   "Assumptions" - the required 11 states have no separate altitude
   state, and `GeofenceSpec` already bundles floor/ceiling with the
   horizontal box as one spec).
6. **Battery reserve** - below `battery_reserve_fraction` →
   `LOW_BATTERY` (or `RETURN_TO_SAFE_POINT` if a safe point is known);
   below `battery_critical_fraction` → `LAND_REQUESTED`.
7. **Lost-link behavior** - own command staleness beyond
   `link_timeout_s` → `DEGRADED_LINK`; sustained isolation from the
   swarm's communication graph beyond `lost_agent_timeout_s` →
   `LOST_AGENT`.
8. **Estimator validity** - `own_state.estimator_valid == False` →
   `SAFE_HOLD` (nothing can be navigated safely without a trustworthy
   position estimate).
9. **Mission candidate command** - the only tier that actually looks at
   `candidate_command`: reject if expired or malformed/non-finite,
   otherwise bound speed/acceleration/turn-rate independently of
   whatever the planner already tried to enforce. → `NORMAL`, or
   `DEGRADED_SENSOR` if `sensor_observation.dropout` was set this tick
   with nothing else wrong.

## Addressing the Phase 2 obstacle-wedging failure

The delayed-observation scenario (Phase 2 diagnostic follow-up, later
reconciled to raw=291/deduplicated=233/steps=229 - see
`docs/PHASE2_DIAGNOSTICS.md`) showed drones getting physically wedged
against obstacles for extended periods with nothing reacting, because
Phase 2's contact classification was retrospective evaluation only.

Phase 4's obstacle tier does four things Phase 2 could not:
- **Treats an actual PyBullet contact (`mission_context.contact_detected`,
  a real, evaluation-input physical event - see `MissionContext`'s
  docstring, not victim ground truth) as an immediate SAFE_HOLD trigger,
  regardless of what the (possibly stale/optimistic) obstacle sensor
  currently reports.** This is the exact Phase 2 gap: a sensor reading
  "all clear" must never be allowed to mask a real contact.
- **Detects persistent near-zero progress within a documented distance of
  a sensed obstacle** (`stuck_min_progress_m` over `stuck_window_s`, for
  `stuck_violation_streak` consecutive checks) even before any contact
  actually occurs.
- **Applies a command-latency + braking-distance lookahead to the sensed
  clearance itself** (see "A real bug this phase caught" below) - this
  is what actually keeps a vehicle from reaching contact in the first
  place, rather than only reacting after the fact.
- **Commands a bounded escape** (`escape_speed_mps`, capped, direction
  derived from the sensed range-scan bin - never ground-truth obstacle
  geometry) for `escape_duration_s`, logging both the trigger
  (`stuck_detected`) and the outcome (`recovery`, with
  `obstacle_surface_clearance_m` recorded so a human can tell whether it
  actually recovered).

**Results, same exact scenario 6 config, seed, and duration as Phase
2/3's reruns** (`python scripts/run_phase4_safety_scenarios.py
obstacle_wedging`):

| | Without supervisor (Phase 2/3 baseline) | With supervisor (final) |
|---|---|---|
| Victims found | 2/5 | **4/5** |
| drone_obstacle contacts (dedup) | 233 | **32** (-86%) |
| drone_drone contacts | 0 | 0 |
| drone_ground contacts (dedup) | 0 | **4179** (new - see below) |
| min ground-truth obstacle clearance | 0.061 m | 0.029 m |
| swarm connectivity fraction | 0.684 | 0.484 |

The primary target metric - persistent drone-obstacle wedging - is
substantially fixed (233 -> 32 events, an 86% reduction, with more
victims found in the same run). **This is not an unqualified win,
reported as such rather than rounded off**: eliminating most of the
obstacle contacts, by making the supervisor override the flocking
candidate far more often (`safety_override_count` above 12800/12960
ticks in the final tuning), exposed a DIFFERENT failure mode - frequent,
sometimes rapidly-alternating safety overrides (separation repulsion,
obstacle escape, altitude correction) can put sustained roll/pitch
demand on `DSLPIDControl` that this simulator's simple PID loop isn't
robust against, and the vehicle can sag in altitude enough to contact
the ground plane, tracked here as `drone_ground_contact_count`. This did
not exist in the Phase 2/3 baseline (0 ground contacts) - the baseline's
one problem (wedging) has been traded for most of the way toward being
solved, at the cost of a new, smaller-in-clearance-terms-but-more-
frequent-in-count problem. See "Remaining risks" - this is flagged there
as open, unresolved work, not claimed as fixed.

**A real bug this phase caught, while investigating an unexpectedly bad
first result**: an early version of `_evaluate_obstacles`/
`_evaluate_separation` computed `predicted_travel_distance_m` (the
"account for command latency"/"account for braking distance"
requirements) but never actually applied it anywhere - the obstacle/
separation checks only compared *instantaneous* sensed clearance against
a fixed uncertainty margin. In the delayed-observation scenario
(`obstacle_sensor_latency_steps=5`, ~0.2s), a drone at cruise speed could
close nearly half a meter during the sensor's own reporting delay before
the instantaneous check ever fired - by the time clearance read as
"below margin," the vehicle could already be inside the obstacle mesh,
and PyBullet's contact-penetration recovery produced large corrective
forces that occasionally destabilized `DSLPIDControl`'s attitude loop
badly enough to crash the vehicle into the ground (observed:
`drone_ground_contact_count` in the thousands, `LOST_AGENT` also
initially over-firing from an unrelated design mistake - see next
paragraph). Fixing this meant actually wiring the already-built
lookahead into both clearance checks (obstacle: applied unconditionally;
separation: gated on closing velocity, since direction data is available
there and a receding vehicle needs no predictive penalty) - see
`tests/test_safety_supervisor.py`'s `test_closing_velocity_triggers_earlier...`/
`test_obstacle_lookahead_triggers_earlier...` tests for the fix's
regression coverage. **This is exactly the kind of gap a certified
system's verification process exists to catch before flight - here it
was caught by an unexpectedly bad simulation result, not a proof.**

**A second design mistake found the same way**: `LOST_AGENT` was
initially derived from the short-range onboard proximity sensor
(`neighbor_observations`) timing out - but that sensor's range is
deliberately short (collision-avoidance only, a few meters), so "no
proximity-sensor neighbor right now" is completely routine during normal
search dispersal, and the supervisor was holding drones in place at a
very high false-positive rate. Fixed by sourcing `LOST_AGENT` from
`MissionContext.own_agent_isolated`, computed by `mission.py` from the
same already-comms-realistic `CommsNetwork` connectivity graph
`swarm_connectivity_fraction` already uses (added
`network.component_size_per_drone()`, purely additive), with a
sustained-duration requirement (`lost_agent_timeout_s`) so a one-tick
radio dropout doesn't fire it either.

**A third bug found the same way, only partially resolved**: the
override tiers (geofence/separation/obstacle) originally only speed-
capped their output - they let the supervisor command an instantaneous
velocity reversal every time one fired, with no acceleration or turn-
rate bound relative to the vehicle's actual current velocity (that
bounding previously existed only on the mission-candidate path, tier 9).
Fixed by adding `bound_velocity_step()` and applying it to every command
this module constructs (see `test_override_commands_are_acceleration_bounded_not_instantaneous`).
This fix, plus wiring the latency/braking lookahead above, took
`drone_obstacle_contact_count` from 233 down to 32 and raised victims
found from 2/5 to 4/5 - genuine progress - but did **not** fully resolve
the underlying interaction: with the supervisor overriding the flocking
candidate on the large majority of ticks in this specific hard scenario,
`drone_ground_contact_count` rose from 0 to 4179. The most likely
explanation, not fully confirmed: frequent, sometimes rapidly alternating
override directions (separation one tick, obstacle escape the next) can
sustain roll/pitch demand on `DSLPIDControl` for long enough that the
vehicle sags in altitude, even though each individual step is
acceleration-bounded. Diagnosing and fixing this properly would mean
either smoothing/hysteresis between override tiers (avoid flipping
direction every tick) or characterizing `DSLPIDControl`'s own closed-loop
lag well enough to bound commands against it directly - both real,
scoped follow-up work, not done in this phase. Reported as an open
finding, not silently tuned away or hidden behind an average.

## Safety geometry, disambiguated (per `SafetyDecision`'s Phase 4 fields)

| Field | Meaning |
|---|---|
| `center_to_center_clearance_m` | Nearest neighbor, drone-center to drone-center |
| `vehicle_body_clearance_m` | Nearest neighbor, edge-to-edge (may be negative - already overlapping) |
| `obstacle_surface_clearance_m` | Nearest sensed obstacle edge (may be negative) |
| `geofence_distance_m` | Signed distance to nearest geofence boundary (negative = outside) |
| `uncertainty_margin_m` | The uncertainty inflation actually applied to reach the numbers above |
| `braking_distance_m` | This vehicle's own predicted stopping distance at its current speed |
| `min_predicted_clearance_m` | Kept from Phase 1 as the single worst-case-across-all-of-the-above figure, for callers that only want one number |

All new fields are additive, defaulted `Optional[...] = None`
(`contract_version` bumped 0.1.0 -> 0.2.0, same major - see
`contracts.check_contract_version_compatible`'s own documented
additive-only policy) - every `SafetyDecision` constructed before this
phase remains valid.

## Assumptions (read before treating any number here as a guarantee)

- **Independence is structural, not just organizational**: this module's
  own `SafetySupervisorConfig` (speed/accel/turn-rate bounds, margins) is
  entirely separate from `MissionConfig`'s flocking tuning - a planner
  change can never implicitly loosen what the supervisor enforces. They
  happen to share numeric defaults with `MissionConfig` today; nothing
  forces them to stay in sync.
- **Keep-out zones are not modeled** - `GeofenceSpec` today is a single
  flight-area box plus floor/ceiling. Arbitrary interior keep-out
  polygons are not threaded through the config in this phase.
- **Altitude floor/ceiling reuses `GEOFENCE_RISK`**, not a dedicated
  state - the required 11-state vocabulary has none, and floor/ceiling
  are already part of the same `GeofenceSpec`.
- **The obstacle lookahead is unconditional** (not closing-velocity-
  gated like separation's) - the range scan's per-bin data doesn't cheaply
  expose "is this vehicle converging on THIS specific bin," so obstacle
  clearance is treated conservatively regardless of heading. This can
  make the supervisor react to a nearby-but-not-actually-approached
  obstacle a little more readily than strictly necessary.
- **No dedicated actuator/motor-cutoff model** - `ABORT`/`LAND` here mean
  "stop commanding horizontal velocity" (`mission.py` maps them to a
  zero `safe_vel`); there is no motor-cutoff or controlled-descent-rate
  concept in this simulator's PID loop. A `LAND_REQUESTED` decision does
  not currently make the vehicle descend - it stops it in place, in the
  same way `HOLD` does, until a future phase's autopilot adapter can
  actually implement mode changes.
- **Command latency/braking assumptions are fixed constants**
  (`command_latency_s`, `assumed_brake_decel_mps2`), not derived from any
  real flight-controller characterization - they are the same kind of
  documented engineering choice as every other threshold in this file.
- **`own_agent_isolated` is a boolean snapshot, not a graph history** -
  mission.py recomputes it from `CommsNetwork` every tick; the supervisor
  adds the "sustained for `lost_agent_timeout_s`" requirement on top of
  that snapshot itself (`_isolated_since`), so a flapping link can still
  reset the clock repeatedly without ever firing `LOST_AGENT`, which is
  the intended (conservative-toward-not-crying-wolf) behavior but is
  itself a design choice, not a law of nature.
- **One `SafetySupervisor` instance per mission, keyed internally by
  vehicle_id** - mirrors `SwarmController`'s existing one-instance-many-
  drones pattern. Nothing in this module is safe to share across two
  concurrent missions.

## Required tests (28 items)

All in `tests/test_safety_supervisor.py` (behavioral, ~45 tests) and
`tests/test_safety_supervisor_architecture.py` (AST-based: no ground
truth, no motor/PWM, controller/consensus cannot bypass the supervisor).
See the test files' module docstrings for the exact item-to-test mapping.

## Required scenarios and benchmarks

`scripts/run_phase4_safety_scenarios.py` runs the 15 required scenarios
(nominal, expired command, sensor dropout, stale sensor, delayed
observation, obstacle approach, obstacle wedging, crossing drones, dense
swarm, communication partition, lost drone, low battery, invalid
estimator, geofence approach, operator abort) and reports, per scenario:
candidate/accepted/rejected counts, safety overrides, active state
histogram, clearance metrics (worst-case AND percentile, not just
average), contact counts (reusing Phase 2's now-disambiguated raw/
deduplicated/step fields unchanged), stuck-detection latency, recovery
outcome, and CPU time per supervisor evaluation. See that script's own
docstring for which of the 14 required report fields are computed
exactly vs. approximated, and why, in the same disclosed-approximation
style as Phase 2's diagnostic report.

## Remaining risks

- **Open, unresolved, and the most important entry in this list**:
  frequent/alternating safety overrides can induce ground contacts that
  did not exist in the pre-Phase-4 baseline (0 -> 4179 deduplicated
  events in the obstacle-wedging scenario) - see "A third bug found the
  same way, only partially resolved" above. The primary target (drone-
  obstacle wedging) improved substantially (233 -> 32 events); this new
  failure mode did not, and is not yet fixed.
- The obstacle-avoidance lookahead is unconditional (see "Assumptions")
  and can make the supervisor more cautious than strictly necessary near
  an obstacle the vehicle isn't actually heading toward.
- No real actuator/motor-cutoff or descent-rate model exists - `ABORT`/
  `LAND_REQUESTED` currently just stop horizontal motion in place.
- Thresholds are engineering choices tuned against this simulator's own
  dynamics and sensor models, not derived from any certified source or
  validated against real flight data.
- `own_agent_isolated`/`DEGRADED_LINK` are both derived from the same
  underlying `CommsNetwork` model Phase 1/2 already built for flocking -
  a bug in that model would affect both the flocking cohesion behavior
  and the safety supervisor's lost-link detection identically, which is
  a shared-fate risk worth knowing about even though nothing found by
  this phase suggests one exists.
- This module has not been fuzzed, formally verified, or run against
  adversarial sensor inputs beyond what the required test list covers.

## Explicitly not started

Peer-local distributed consensus, MAVLink, Pixhawk/ArduPilot/PX4
adapters, SITL, and any real-hardware integration. No file under any of
those areas was touched in this phase.
