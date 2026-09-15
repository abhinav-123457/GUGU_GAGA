# Phase 2 Diagnostic Follow-Up

Adds `swarm_sim/diagnostics.py` and instrumentation calls in `mission.py`,
plus two diagnostic-only counters in `sensors.py`. **Changes no sensor
model, no consensus algorithm, and no control/safety behavior** - every
existing Phase 1/Phase 2 test still passes byte-for-byte unchanged
(134 passed, up from 128 - the 6 new tests are new architecture checks,
not modified old ones).

**This is not the Phase 4 safety supervisor.** Nothing in this document
or in `diagnostics.py` prevented, predicted, or reacted to anything -
every number here is recorded strictly after the fact, for the report.

## What was added, and why it doesn't change behavior

| Addition | Where | Why it's behavior-preserving |
|---|---|---|
| `diag_raw_candidates_generated`, `diag_dropped_tick_count` counters | `sensors.py`, `VictimSensorModel` | Incremented after the candidate tuple is already built, using no new RNG draws, at the same point in the same order every tick; nothing about what's returned changes |
| `MissionDiagnostics` (new class) | `diagnostics.py` | Only ever *read from* already-public state (`ConsensusBoard.reports` is a plain list `mission.py` could already read) or *written to* from values `mission.py` was already computing |
| `_classify_detections_for_scoring` now also returns `matched_vids_for_candidates` | `mission.py` | Same matching computation as before, just also returned instead of discarded |
| CPU-time accumulation around sensing/consensus calls | `mission.py`'s `run()` | Pure `time.perf_counter()` measurement around existing calls - measures, doesn't alter |
| Obstacle body-ID capture in `_spawn_obstacles` | `mission.py` | `p.createMultiBody()`'s return value was previously discarded; capturing it changes nothing about the physics |
| `_classify_and_log_contacts` (new method) | `mission.py` | Reads `p.getContactPoints()` (already being read for the old, simpler `contact_steps` counter) plus each drone's *already-computed* `neighbor_obs`/`range_returns` for that tick |

## Per-victim, per-scenario report

For every victim, every scenario: ground-truth position, times any drone
entered sensor range, valid sensor observations, detection candidates,
reports submitted, reports received, reports inside the consensus window
(snapshotted at every consensus check), quorum status, confirmation
position/timestamp, match result, and - if not found - the exact reason,
chosen from:

1. `"never within victim_sensor_range_m of any drone"`
2. `"in range at least once, but every attempt was gated out (FOV/occlusion) or lost to a false-negative roll"`
3. `"only N independent report(s) ever submitted; consensus_quorum is Q"`
4. `"reports were submitted but never had >= consensus_quorum distinct drones clustered within consensus_cluster_radius at the same time"`
5. `"quorum was reached at some point, but consensus_window_sec expired before try_confirm processed that cluster"`

Full per-victim JSON for all 8 scenarios is saved alongside this report
(see the Phase 2 diagnostic report message for file locations) rather than
inlined here - 5 victims x 8 scenarios x 12 fields does not read usefully
as prose.

**`reports_submitted` and `reports_received` are always equal in this
architecture.** `ConsensusBoard` is centralized and synchronous
(`consensus.submit()` is a direct method call, not a message that
transits a network) - see `docs/PHASE0_AUDIT.md`'s risk #7. There is no
transit delay to create a difference between "submitted" and "received"
at this layer; that distinction only becomes real once/if a future phase
migrates detection reports through `CommsNetwork` the way flocking data
already does.

## Event-level vs. victim-level metrics

Kept explicitly separate, because they answer different questions:

| Level | Metrics | Question answered |
|---|---|---|
| Event | `true_positive_events`, `false_positive_events` | How many individual sensing rolls happened, across every drone, every tick? |
| Victim | `unique_victims_detected`, `confirmed_victims`, `missed_victims` | How many of the 5 actual victims ended up in which state? |
| Confirmation | `false_confirmation_events` | How many times did consensus confirm something that didn't correspond to any real victim? (Not tied to a specific victim by definition.) |

A victim can accumulate many detection *events* (one per drone per tick
it's sensed) while only ever counting once toward `unique_victims_detected`.

## Clearance metrics, fully disambiguated

| Metric | What it measures | Ground truth or estimate? |
|---|---|---|
| `min_sensor_observed_clearance_m` | Center-to-center, nearest sensed neighbor | Estimated (from `NeighborObservation.measured_position_m`) |
| `min_ground_truth_clearance_m` | Center-to-center, nearest actual neighbor | Ground truth - scoring only |
| `min_vehicle_body_clearance_m` | Edge-to-edge (`min_ground_truth_clearance_m - 2 x vehicle_body_radius_m`) | Ground truth - scoring only |
| `vehicle_body_radius_m` | CF2X arm length (0.0397m) + prop radius (0.023135m), both from `BaseAviary`'s own startup log | Constant, not measured per-tick |
| `min_sensor_observed_obstacle_clearance_m` | Nearest sensed obstacle edge | Estimated (`ObstacleRangeSensor` range scan) |
| `min_ground_truth_obstacle_clearance_m` | Nearest actual obstacle edge | Ground truth - scoring only |
| `neighbor_sensor_uncertainty_margin_m` | The *configured* `neighbor_sensor_noise_std_m` | Not a per-observation value - see limitation below |

**Uncertainty margin is a configured-parameter proxy, not a dynamic
per-observation field.** `contracts.NeighborObservation` carries
`communication_confidence`/`stale`, not an uncertainty radius (unlike
`DetectionCandidate`, which does carry
`localization_uncertainty_m`) - reusing the Phase 1 contract as-is
(neither phase reopened it) means there is currently no per-observation
uncertainty number for neighbor sensing to report exactly. The static
configured noise standard deviation is reported instead, clearly labeled.

**Before/after contact**: every contact event in `contact_log` carries a
`clearance_trend` field (`"closing"`, `"opening"`, or `"unknown"` for the
first observation of a given body pair), computed by comparing this
event's `actual_clearance_m` against the last one recorded for the same
pair of PyBullet body IDs.

## Contact-count reconciliation (diagnostic-only correction)

An earlier version of the Phase 2 diagnostic report quoted **291** contacts
for scenario 6 (delayed observations) while this document quoted **229**
for the same run under the same word, "contacts." Both numbers were
correct - they were just measuring two different things, and a third,
previously-unreported number sits between them. Rerunning scenario 6
exactly (see command below) against the current code gives all three,
now explicit and permanently disambiguated in `MissionDiagnostics.to_report_dict()`:

| Field | Value | What it counts |
|---|---|---|
| `raw_contact_point_count` | **291** | Every `p.getContactPoints()` record touching a drone, summed over the whole run. PyBullet can report several manifold points for the *same* touching pair within one tick (e.g. a drone wedged flush against a flat obstacle face) - this is the number that was previously reported as "contacts" in the chat report. |
| `deduplicated_contact_event_count` | **233** | `raw_contact_point_count` collapsed to one entry per distinct (tick, body-pair) - the semantically meaningful "how many separate touching incidents" count. Not reported at all before this fix. |
| `contact_step_count` | **229** | Simulation ticks with >=1 relevant contact (mission.py's pre-existing `_contact_steps`, unchanged) - this is the number that was previously reported as "logged events" in this document. It is *lower* than the deduplicated event count because more than one distinct pair can be in contact during the same tick (e.g. two different drones each wedged against a different obstacle simultaneously count as 1 step but 2 events). |
| `drone_obstacle_contact_count` | **233** | Deduplicated events classified `drone_obstacle`. |
| `drone_drone_contact_count` | **0** | Deduplicated events classified `drone_drone`. |
| `drone_ground_contact_count` | **0** | Deduplicated events classified `drone_ground`. |

`drone_obstacle_contact_count + drone_drone_contact_count + drone_ground_contact_count
== deduplicated_contact_event_count` always holds (233 = 233 + 0 + 0) - this is
the invariant `tests/test_diagnostics.py` guards against regressing. This
also reconfirms, with exact numbers instead of an estimate, the earlier
finding: every scenario-6 contact is drone-vs-obstacle, none drone-drone
or drone-ground.

**Reproduction.** Config, seed, and everything else held exactly as before:

```
python scripts/regenerate_phase2_diagnostics.py 6_delayed_observations
```

- Scenario config: `num_drones=6, num_victims=5, num_obstacles=4, duration_sec=90.0, seed=42, gui=False`, overridden with `victim_sensor_latency_steps=10, neighbor_sensor_latency_steps=10, obstacle_sensor_latency_steps=5` (all other sensor/communication fields at `MissionConfig` defaults).
- Git commit this rerun was taken at: `74cd1bd4eda996fa38d38348a5befe7237f4ab8a`.
- Generated result file: `results/phase2_diagnostics/6_delayed_observations.json` (gitignored - regenerate with the command above rather than expecting it in the repo; `scripts/regenerate_phase2_diagnostics.py` is the committed, deterministic source of truth for how it's produced).

## Contact classification (scenario 6: 291 raw points / 233 deduplicated events / 229 contact-steps - see reconciliation above)

Every one of the 233 deduplicated delayed-observation contact events is classified by:
`category` (`drone_drone` / `drone_obstacle` / `drone_ground`),
`t`, `body_a`/`body_b` (PyBullet body IDs) and `drone_ids_involved`,
`sensor_age_s` (the `packet_age_s` of whichever drone's `NeighborObservation`
of the other was used, or `None` if neither side currently had one at
all), `observation_dropout_or_stale` (`True` if that observation was
`stale`, or if there was no observation of the other party at all),
`estimated_clearance_m` vs `actual_clearance_m`, and `command_age_s`
(currently a proxy equal to `sensor_age_s` - see limitation below). Full
per-event data is in the saved JSON (see the diagnostic report message).

**`sensor_age_s` is only populated for `drone_drone` contacts.** It comes
from the involved drone's `NeighborObservation.packet_age_s` at that
tick. For `drone_obstacle` contacts, the obstacle range scan has no
per-observation timestamp exposed (the same gap `PHASE2_SENSING.md`
already documents for `SensorObservation`'s latency fields), so
`sensor_age_s` is `None` there even though `estimated_clearance_m` is
still available from that tick's range scan. For `drone_ground`
contacts it is `None` because no downward/ground sensor is modeled at
all.

**Command age is a proxy, not an independently tracked value.** Phase 1's
`Command` contract has a `timestamp_s`/`expiration_time_s` pair, but no
runtime `Command` object is actually constructed and timestamped in
`mission.py`'s control loop yet - `SwarmController.step()` returns a bare
velocity vector, not a `Command`. Until that wiring exists (a natural
Phase 6 task), "how stale was the command" is approximated by "how stale
was the sensor data that produced it," which is a reasonable but not
exact stand-in.

### CRITICAL LIMITATION

**Every one of these 233 contact events happened. Nothing detected, prevented,
or responded to any of them.** `_classify_and_log_contacts` runs *after*
the physics step that already produced the contact - it is a strictly
retrospective evaluation instrument. There is no independent safety
supervisor in this codebase (Phase 4), so a stale, dropped-out, or noisy
sensor reading that leads a drone into another drone, an obstacle, or the
ground today has nothing else checking whether that was actually safe.
This diagnostic follow-up makes that fact measurable and traceable
(which sensor state preceded which contact) - it does not make it safe.

## False-positive-heavy scenario resource profile

| Metric | Meaning |
|---|---|
| `sensor_raw_candidate_count` | Every `DetectionCandidate` `VictimSensorModel` constructed (real + phantom), summed across all drones/ticks, **before** the whole-observation dropout roll |
| *(gated candidate count)* | Identical to raw in this implementation - see note below |
| `true_positive_events + false_positive_events` | Candidates that survived dropout and were actually submitted to `ConsensusBoard` ("consensus candidate count") |
| `peak_consensus_report_count` | The largest `len(ConsensusBoard.reports)` observed at any point in the run |
| *(expired candidate count)* | Approximated - see note below |
| `sensing_cpu_time_s` / `consensus_cpu_time_s` | Wall-clock time inside sensing calls vs. inside `ConsensusBoard.submit`/`try_confirm` calls, via `time.perf_counter()` |
| Peak memory | Not separately measured this round - see limitation |

**Raw and gated candidate counts are identical in this implementation.**
`VictimSensorModel.sense()` applies range/FOV/occlusion/false-negative
gating *while building* the candidate list - there is no earlier,
separately-countable "ungated" set to report a different number for.

**Expired candidate count is approximated, not exact, and this is
explicitly disclosed rather than silently reported as precise:**
`ConsensusBoard.try_confirm()`'s return value (`centroid, drones`) reports
the *set of distinct contributing drone IDs*, not the exact number of
report objects consumed from its internal list - and `consensus.py` was
not modified to expose that (out of scope: "do not change the consensus
algorithm"). `expired_candidate_count` is therefore computed as
`total_submitted - sum(len(drones) per confirmation) - final_board_size`,
which undercounts consumption whenever one drone contributed more than
one report to the same confirmed cluster. The exact figure would require
either instrumenting `ConsensusBoard` (out of scope this round) or
accepting this documented approximation - see the Phase 2 diagnostic
report message for the computed value and this caveat repeated inline.

**Peak memory was not measured with a dedicated tool this round.**
`peak_consensus_report_count` (an object *count*, not bytes) is reported
as the closest available proxy. A `tracemalloc`-based measurement was
considered but deferred to keep this round's `sensors.py`/`consensus.py`
footprint at zero and avoid adding a new profiling dependency without it
being explicitly requested for a specific number rather than "peak
memory" in general terms.

## Architecture checks added

`tests/test_architecture_ground_truth_boundary.py` (6 tests):
- No non-sensor runtime module (`controller.py`, `consensus.py`,
  `network.py`, `recruitment.py`, `speed_control.py`, `telemetry.py`)
  references `victims_ground_truth`/`WorldState`/`victims`/`obstacles`/
  `mission` as an identifier.
- `controller.py` still has no forbidden ground-truth identifiers and no
  `positions`/`velocities` parameter (re-confirms the Phase 2 checks).
- Every `mission.py` function that references `self.victims`/
  `self.obstacles` is on an explicit whitelist of plant-setup/rendering/
  scoring functions (`__init__`, `_sample_obstacle_positions`,
  `_spawn_obstacles`, `_draw_victims`, `_setup_gui_view`, `_match_victim`,
  `_classify_detections_for_scoring`, `_track_clearance_and_contacts`) -
  anything else touching ground truth fails this test.
- `run()` itself (the orchestrator) touches no ground truth directly -
  everything is delegated to a whitelisted helper.
- Three different hidden worlds (varying both victim *and* obstacle
  geometry, including an empty one), given the same sensed inputs,
  produce identical `SwarmController.step()` output.

**`diagnostics.py` is deliberately not held to the same "no ground truth"
identifier check** as `controller.py`/`consensus.py`/etc. It receives
victim ground-truth positions as a constructor argument for exactly one
purpose: recording them in the per-victim report. This is the same tier
as `mission.py`'s scoring functions (an evaluation/reporting sink, never
read by any controller or consensus decision) - not a second leak.

## Rerun confirmation

```
python -m compileall -q swarm_sim run_mission.py tests
python -m pytest -q
```
134 passed (128 unchanged from before this follow-up + 6 new architecture
tests). No existing test's expected value changed.

**Contact-count reconciliation follow-up** (see "Contact-count
reconciliation" above) added 3 more tests
(`tests/test_diagnostics.py`, PyBullet-free) for the raw/deduplicated/step
contact accounting: 137 passed total, same rerun command, no existing
test's expected value changed.
