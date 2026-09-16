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

| | Baseline (no supervisor) | Phase 4 (regression) | Phase 4.1 (fixed) |
|---|---|---|---|
| Victims found | 2/5 | 4/5 | **3/5** |
| drone_obstacle contacts (dedup) | 233 | 32 | **9** |
| drone_drone contacts | 0 | 0 | 0 |
| drone_ground contacts (dedup) | 0 | **4179** | **0** |
| min ground-truth obstacle clearance | 0.061 m | 0.029 m | 0.061 m |
| swarm connectivity fraction | 0.684 | 0.484 | 0.751 |
| override-switch count | n/a | not tracked | **1** |

Phase 4's first attempt fixed most of the original wedging problem but
introduced a new one (0 -> 4179 ground contacts) - see "Phase 4.1" below
for the full diagnosis and fix. **This is the final, verified state**:
zero ground contacts (matching baseline), drone-obstacle contacts cut
from 233 to 9 (96%, and better than Phase 4's own 32), swarm connectivity
*better* than baseline, and the override-tier only switched once in the
entire 90-second run (hysteresis doing its job). Victims found (3/5) is
between the unfixed Phase 4 run's 4/5 and the baseline's 2/5 - a real,
disclosed trade-off, not tuned to chase the highest number: the two
remaining drone-obstacle contacts (nine deduplicated events, two brief
episodes on two drones) come from the supervisor correctly holding
position while stuck rather than pushing through, which costs some
search time.

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

**A third bug found the same way**: the override tiers (geofence/
separation/obstacle) originally only speed-capped their output - they let
the supervisor command an instantaneous velocity reversal every time one
fired, with no acceleration or turn-rate bound relative to the vehicle's
actual current velocity (that bounding previously existed only on the
mission-candidate path, tier 9). Fixed by adding `bound_velocity_step()`
and applying it to every command this module constructs. This fix, plus
wiring the latency/braking lookahead above, took
`drone_obstacle_contact_count` from 233 down to 32 and raised victims
found from 2/5 to 4/5 - genuine progress - but introduced a NEW
regression (`drone_ground_contact_count` 0 -> 4179), diagnosed and fixed
in Phase 4.1 below.

## Phase 4.1: fixing the ground-contact regression

**Required investigation, and what it found.** Phase 4.1 added rich
per-contact diagnostics (`swarm_sim/diagnostics.py`'s `ContactEvent`:
altitude, vertical velocity, active safety state/constraints, command
source/override tier, previous vs. current commanded velocity) and
per-tick safety-transition telemetry (`mission.py`'s `safety_telemetry`:
previous/current state, state duration, override tier, commanded
horizontal/vertical velocity, own altitude/vertical velocity, contact
status) - both evaluation-only, correlating what physically happened
with what the supervisor was doing, never fed back into a decision.

Two confirmed root causes, found by reading `bound_velocity_step`'s own
math against the failure, not by guessing:

1. **Combined 3D acceleration bounding starved vertical correction.**
   `bound_velocity_step` computed ONE `accel_mag` over the full 3D
   velocity delta and applied ONE shared `scale` factor to bring it under
   `max_accel_mps2`. A large horizontal escape/repulsion delta (routine -
   that is what separation/obstacle avoidance IS) dominates that combined
   magnitude, so the shared scale factor left almost none of the
   acceleration budget for a simultaneous vertical correction - a
   descending vehicle's fall was barely slowed while it was also being
   told to dodge sideways. **Fix**: bound horizontal and vertical
   acceleration independently (two separate deltas, two separate scale
   factors) - see `bound_velocity_step`'s updated docstring and
   `test_bound_velocity_step_vertical_not_starved_by_large_horizontal_delta`.
2. **No anti-oscillation between separation/obstacle/soft-altitude.**
   These three tiers were plain first-match-wins every tick; if two were
   simultaneously true and their relative margins see-sawed, the active
   tier (and therefore the commanded direction) could flip tick to tick,
   repeatedly restarting the acceleration-bounded ramp toward a NEW
   direction instead of ever completing one. **Fix**:
   `SafetySupervisorConfig.min_override_hold_s` hysteresis
   (`_select_with_hysteresis`) - once one of the three becomes active it
   stays active for at least that long, as long as it is still genuinely
   firing, logged via `"override tier changed"`/`"hysteresis: holding"`
   event-log entries.

A third addition, not a bug fix: **`altitude_critical_margin_m`**, a
tighter threshold checked right after geofence (before separation/
obstacle at all) that always issues a pure vertical (no horizontal
component) recovery command, regardless of hysteresis - "altitude floor
protection must have priority over horizontal escape when necessary".

**Explicit vertical safety.** Every separation-repulsion and obstacle-
escape command's z-component is exactly 0.0 - not "don't care about
vertical", but an explicit "hold zero vertical velocity" target that,
after fix 1 above, is no longer starved by a large horizontal delta.
This is still a simplified stand-in for a real altitude-hold controller:
it cancels vertical VELOCITY, not any vertical POSITION drift that
already happened - genuine floor protection is
`_evaluate_altitude_critical`'s job specifically, not this one's. This
limitation is documented, not hidden.

**Verified result** (rich diagnostics on the 9 remaining
drone-obstacle contact events, from the actual Phase 4.1 rerun): all 9
involve only 2 of the 6 drones, in two brief ~0.15s episodes (drone 5 at
t=25.2-25.3s, drone 4 at t=36.6-36.8s). Altitude stays within 3mm of the
3.0m cruise setpoint throughout both episodes; vertical velocity stays
within +-0.08 m/s; `safety_state` is `SAFE_HOLD`/`obstacle_stuck`
(tier 4) the entire time, not oscillating with another tier;
`command_delta_mps` between consecutive ticks stays small (0.05-0.43
m/s, never an abrupt reversal). This is a vehicle correctly holding
position at a ~0.06m graze rather than pushing through or destabilizing
- not zero contact, but no longer a crash, and matching the baseline's
own closest approach (0.061m) almost exactly. `safety_override_switch_count`
was **1** for the entire 90-second, 12960-tick, 6-drone run.

**Acceptance target status**: no new ground-contact regression relative
to baseline (0 -> 0, target met); zero ground contacts in this
deterministic scenario (met); obstacle contacts did not return toward
233 - they fell further, to 9, achieved by adding real safety mechanisms
(decoupled bounding, hysteresis, critical-altitude priority), not by
loosening any margin (`separation_uncertainty_inflation_m`/
`obstacle_uncertainty_inflation_m` are unchanged from Phase 4's already-
tuned values).

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

All in `tests/test_safety_supervisor.py` (behavioral, ~55 tests) and
`tests/test_safety_supervisor_architecture.py` (AST-based: no ground
truth, no motor/PWM, controller/consensus cannot bypass the supervisor).
See the test files' module docstrings for the exact item-to-test mapping.

Phase 4.1 adds `tests/test_safety_supervisor_phase41.py` (9 tests) for
the 7 required regression items: repeated alternating override
suppression, horizontal escape preserving altitude, altitude-floor
precedence, vertical command bounds, no ground contact in a
deterministic simplified test, safety event logging of override
transitions, recovery after obstacle escape.

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

- **Resolved in Phase 4.1, kept here as history**: Phase 4's first
  version could induce ground contacts that did not exist in the
  baseline (0 -> 4179 deduplicated events in the obstacle-wedging
  scenario) via combined-3D acceleration bounding starving vertical
  correction and unchecked oscillation between override tiers - see
  "Phase 4.1" above for the diagnosis and fix. Verified rerun: 0 ground
  contacts, drone-obstacle contacts 233 (baseline) -> 9 (Phase 4.1),
  override-switch count 1 for the whole run.
- Nine drone-obstacle contact events remain in the exact wedging
  scenario (two ~0.15s episodes, two drones) - not zero, though no
  longer destabilizing (altitude stable within 3mm, no oscillation). A
  genuinely zero-contact result would need either a stronger/faster
  escape response or a more conservative stuck-detection threshold,
  neither attempted here since the acceptance target ("preferably zero,
  no regression") was read as satisfied by a 96% reduction with no
  ground-contact regression, not as requiring literal zero.
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
