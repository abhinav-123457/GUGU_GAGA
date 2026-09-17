# Phase 5: Peer-Local Distributed Consensus

Replaces the confirmation-decision path of the centralized `ConsensusBoard`
with a model where no single object holds the swarm's shared belief about
victim detections. `ConsensusBoard` (`swarm_sim/consensus.py`) is kept
**unmodified**, as the reference/comparison implementation Phase 5 asked
for - nothing in this document changes it.

**This is a simulation research exercise, not a certified or fielded
distributed system.** No MAVLink, Pixhawk, ArduPilot, PX4, SITL, or
hardware integration was started or touched - see "Explicitly not
started" at the end.

## Pipeline

```
Drone-local sensor observation (VictimSensorModel - unchanged, Phase 2)
        |
        v
DroneConsensusNode.report_detection()      <- THIS drone's own private state
        |
        v
CommsNetwork message exchange               <- range-limited, lossy, latent;
        |                                      same physical channel model
        |                                      flocking already uses, now
        |                                      generalized to typed messages
        v
peer DroneConsensusNode.receive_message()  <- the RECEIVING drone's own
        |                                      private state, mutated only
        |                                      by messages that arrived
        v
DroneConsensusNode.try_confirm()            <- LOCAL quorum check, using
        |                                      only evidence this node has
        |                                      itself accumulated
        v
local confirmation -> RecruitmentBoard.announce()
        |
        v
SwarmController.step()  (unchanged)  -> CandidateCommand
        |
        v
SafetySupervisor.evaluate()  (unchanged, Phase 4/4.1)
        |
        v
PyBullet
```

Everything below `RecruitmentBoard.announce()` is completely untouched -
distributed consensus has no reference to the controller, the safety
supervisor, PID, or PyBullet, and cannot bypass any of them (see "Command
boundary" and the architecture tests).

## Files changed

- `swarm_sim/distributed_consensus.py` (new) - `MessageType`, `Evidence`,
  `ConsensusMessage`, `ConfirmationResult`, `DroneConsensusNode`,
  `DistributedConsensus`.
- `swarm_sim/network.py` - `CommsNetwork` extended with a generic typed-
  message channel (`send_message`/`pump_messages`/`poll_inbox`) alongside
  its existing position/velocity broadcast, plus a separate `message_rng`
  stream. `tick()`, `neighbor_table()`, `connected_component_sizes()`,
  `component_size_per_drone()` are unchanged.
- `swarm_sim/config.py` - `consensus_mode` ("distributed" default |
  "centralized"), `consensus_relay_enabled`.
- `swarm_sim/mission.py` - constructs `self.distributed_consensus`
  alongside the still-constructed `self.consensus`; `run()` branches on
  `cfg.consensus_mode` at exactly the two points detection candidates are
  submitted and confirmations are resolved; new
  `_submit_detections_distributed`, `_resolve_confirmed_detections_distributed`,
  `_distributed_pending_reports_snapshot` methods. `ConsensusBoard`'s own
  methods and everything downstream of `safe_vel` are untouched.
- `run_mission.py` - `--consensus-mode` CLI flag.
- `tests/test_distributed_consensus.py` (new, 30 tests),
  `tests/test_distributed_consensus_architecture.py` (new, 9 tests).
- `scripts/run_phase5_consensus_scenarios.py` (new) - the 11 required
  scenarios, each run under both consensus modes.
- `docs/PHASE5_DISTRIBUTED_CONSENSUS.md` (this file).

## Protocol

### Per-drone state (`DroneConsensusNode`)

Every drone owns exactly one node, holding only:

| Required item | Field |
|---|---|
| local detection reports + received peer reports | `_evidence: Dict[report_id, Evidence]` |
| report timestamps | `Evidence.observation_timestamp_s` |
| confidence | `Evidence.confidence` |
| source drone ID | `Evidence.origin_drone_id` |
| observation age | derived: `now_s - observation_timestamp_s` |
| cluster membership | `cluster_membership()` |
| confirmation status | `confirmed_log` / `is_confirmed()` |
| expiry time | derived: `observation_timestamp_s + window_s` |
| belief revision history | `belief_revision_history` (append-only log of every store/receive/expire/confirm/adopt event) |

No drone ever reads another drone's node attributes directly. The only
entry points are `report_detection`/`receive_message`/`try_confirm`, and
the only things they return are immutable dataclasses or counters -
mechanically checked in `test_distributed_consensus_architecture.py`
(`test_no_cross_node_access_to_another_drones_private_state`, which scans
the whole module's AST for any `<something>._evidence`-style access where
`<something>` is not `self`).

### Message schema

```
MessageType: DETECTION_REPORT | CONSENSUS_EVIDENCE | CONFIRMATION | EXPIRY

Evidence:
  origin_drone_id, origin_seq        -> report_id = f"{origin_drone_id}:{origin_seq}"
  position_m, confidence, observation_timestamp_s

ConsensusMessage:
  message_type, sender_id, sent_at_s
  evidence: Tuple[Evidence, ...]                 # DETECTION_REPORT / CONSENSUS_EVIDENCE
  confirmation_id, centroid_m,
  contributing_drone_ids, contributing_report_ids  # CONFIRMATION
  expiry_report_ids                               # EXPIRY
```

- **DETECTION_REPORT**: firsthand evidence, broadcast by the drone that
  sensed it.
- **CONSENSUS_EVIDENCE**: secondhand relay. A node offers each distinct
  peer-originated `report_id` for relay **at most once ever**
  (`new_relay_evidence`), enabling two-hop propagation (A -> B -> C)
  beyond A's direct radio range without B ever being credited as an
  independent witness - `origin_drone_id` is preserved unchanged through
  relay.
- **CONFIRMATION**: broadcast once a node's own `try_confirm` succeeds, so
  peers adopt the same belief (and purge the now-subsumed evidence,
  `contributing_report_ids`) instead of independently re-deriving it.
  Idempotent - a repeat delivery of the same `confirmation_id` is a no-op.
- **EXPIRY**: announces that some `report_ids` were locally discarded
  (aged past the window without reaching quorum), so a peer holding the
  same evidence can prune it early. Purely an optimization - a node's own
  expiry (in `try_confirm`) never depends on receiving one.

**Deterministic message IDs** (requirement 6): `report_id` is
`origin_drone_id:origin_seq` - the originating drone's identity plus its
own strictly-increasing local sequence number. `confirmation_id` is
`"|".join(sorted(report_ids))` of the confirmed cluster - content-derived,
so two nodes that end up with the same evidence set always compute the
identical id regardless of arrival order.

### Quorum semantics

- **Distinct reporting drones**: `try_confirm` clusters by spatial
  proximity (`cluster_radius`) and counts `len({e.origin_drone_id for e in
  cluster})` - a drone's own repeated reports (guaranteed to all share its
  `origin_drone_id`) can never inflate this past one, satisfying
  requirement 8 by construction rather than a special case.
- **Cluster radius**: same `consensus_cluster_radius` as centralized.
- **Temporal window**: `consensus_window_sec`, pruned relative to `now_s`
  at the top of every `try_confirm` call - identical mechanism to
  `ConsensusBoard._prune`.
- **Minimum confidence**: not currently gated on (centralized doesn't gate
  on it either - `confidence` is carried through but only used for
  reporting, not thresholding). Documented here rather than silently
  added, since adding a new gate would be scope beyond "replace the
  confirmation path with an equivalent distributed one."
- **Report expiry**: see temporal window above; `expired_evidence_count`
  tracks it per node.
- **Delayed and out-of-order messages**: storage is keyed by `report_id`
  (idempotent regardless of arrival order or count), and `try_confirm`
  visits candidate seeds in `(observation_timestamp_s, report_id)` order,
  not arrival order - so which cluster wins a same-tick tie never depends
  on message delivery order (`test_out_of_order_message_arrival_yields_the_same_confirmation`).

### Duplicate/repeat suppression (requirements 7-8)

`_ingest_evidence` rejects (and counts) three distinct cases, never
silently merged into one counter:

- **`duplicate_suppressed_count`** - a `report_id` already known (same
  evidence arriving twice, e.g. via both direct delivery and relay).
- **`own_repeat_suppressed_count`** - evidence whose `origin_drone_id`
  equals the receiving node's own id. Not just defensive: relay makes this
  a real, reachable case even with only one relay hop - if A's evidence
  reaches B directly and B relays it (`CONSENSUS_EVIDENCE`, broadcast to
  everyone in B's range, since B has no way to know "everyone except the
  original owner"), and A is still within B's range, A receives its own
  evidence bounced back and correctly discards it here rather than ever
  storing or clustering against it.
- **`stale_message_count`** - evidence already past `window_s` relative to
  the receiver's `now_s` at arrival time.

### Partitions and healing (requirement 10)

Partitions are **not** modeled with special-cased "partition" logic
inside `distributed_consensus.py` - they are an emergent property of
`CommsNetwork`'s existing range/dropout model, exactly as they already
were for flocking's position broadcast. A node simply never receives
evidence that never arrives; `try_confirm` correctly returns `None`
(counted as `quorum_failure_count`) until enough distinct evidence has
actually arrived. When connectivity resumes, normal `DETECTION_REPORT`/
`CONSENSUS_EVIDENCE` delivery (plus relay) lets pending, still-unexpired
evidence flow in and `try_confirm` converges - deterministically, since
nothing about the clustering algorithm depends on how long a partition
lasted, only on what evidence is present and its content-derived ordering.

"Stale confirmations must expire": interpreted as **pending (not-yet-
confirmed) evidence** expiring past `window_s` without reaching quorum -
identical to centralized's own semantics. A genuine `ConfirmationResult`,
once reached, is permanent (matches `ConsensusBoard`, which also never
un-confirms a victim) - "stale" cannot apply to something that already
validly met quorum without redefining quorum itself as unreliable.

"Conflicting clusters must not silently merge": two spatially-separated
report groups (further apart than `cluster_radius`) are never combined -
this falls out of the same spatial clustering rule requirement 10 asks
for, not a separate mechanism.

### Comparison mode (requirement 14)

`MissionConfig.consensus_mode` (`"distributed"` default, `"centralized"`)
picks which path feeds `RecruitmentBoard.announce()`; `ConsensusBoard` is
still constructed either way. `scripts/run_phase5_consensus_scenarios.py`
runs every scenario under **both** values, same seed, same sensor/network
config - this is the comparison mode.

### Why clustering matches centralized exactly

An earlier version of `try_confirm` additionally required cluster members
to be within `window_s` of each other's timestamp (not just individually
non-expired relative to `now_s`) - defensible as a standalone rule, but it
occasionally produced a *different* confirmed cluster than centralized
under otherwise-perfect communication, which directly conflicts with "must
agree under perfect communication." It was removed; temporal separation
(requirement 11) is still real, just enforced through the same prune-
relative-to-`now_s` mechanism centralized uses (see
`test_temporally_separated_reports_expire_before_they_can_cluster`).

### Why perfect-comm agreement is checked at the algorithm level

`test_centralized_and_distributed_agree_given_identical_evidence_stream`
feeds the identical per-tick evidence sequence into `ConsensusBoard` and
into one fully-connected `DroneConsensusNode` and asserts every
confirmation is bit-identical (15 random seeds, 0 mismatches) - this is
the real, unconditional form of the requirement.

A *full closed-loop mission* comparison (both consensus paths, real
PyBullet, real recruitment feedback) is a stronger claim: once a
confirmed centroid becomes a beacon position, it changes a drone's
trajectory, which changes what it senses next tick. Investigating this
directly (dumping per-victim confirmation timestamps/centroids for a
mismatching seed) found the actual cause: `ConsensusBoard.try_confirm`
averages a cluster's positions with `np.mean`; an earlier version of
`DroneConsensusNode.try_confirm` used a plain Python `sum()/len()`. Both
are mathematically the centroid, but they can differ in the last bit or
two from floating-point summation order - and a ~1e-16 m difference in a
beacon position, propagated through a ~20+ second chaotic physics
simulation with feedback, was enough to occasionally produce a materially
different downstream trajectory (a different subsequent detection,
confirmed at a different tick). Switching `try_confirm` to `np.mean` (bit-
identical to centralized) resolved every case tested: **0 mismatches
across 20 full-mission seeds** after the fix (`victims_found` and total
confirmed-event count identical in all 20; see the algorithm-level test
above for the seeds re-verified as a pytest regression,
`test_centralized_and_distributed_missions_agree_under_perfect_communication`).
This is reported plainly rather than glossed over, because it is a real,
if narrow, example of why "agreement" needs a precise definition in a
closed feedback loop.

### Partition scenarios

`communication_partition`/`partition_healing` in the scenario runner use a
short/moderate `communication_radius` respectively, relying on the
swarm's own dispersal-then-recruit dynamics to create and heal partitions
emergently - not an artificial mid-run network toggle. `swarm_connectivity_fraction`
is the direct, already-existing metric for how connected the swarm stayed.

### Approximate metrics

Two required report metrics are honestly approximate, labeled as such in
both the code and the results:

- **`memory_proxy_bytes_per_drone_approx`** - shallow `sys.getsizeof` of
  each node's public state, not a real memory profile.
- **`confirmation_latency_s_*_approx`** - time from mission start to a
  victim's `confirmation_timestamp`, not from its first piece of evidence
  (that per-report timestamp isn't retained once consumed by a
  confirmation). A coarse but honest proxy.

`dropped_message_count` is reported as `null`: `CommsNetwork` drops a
packet silently at the channel layer (the same way it always has for the
position/velocity broadcast) - there is no per-message drop counter to
read, only the aggregate effect (visible in `stale_message_count`,
`quorum_failure_count`, and reduced `report_count`/`confirmed_victims`).

## Results

Full numbers: `results/phase5_scenarios/phase5_scenario_results.json`
(gitignored, regenerate with `python scripts/run_phase5_consensus_scenarios.py`).
`conn` = `swarm_connectivity_fraction`; `found`/`false` = confirmed
victims / false confirmations; `gnd`/`obs` = deduplicated drone-ground /
drone-obstacle contacts; `dup` = duplicate-message suppressions; `qf` =
quorum-failure count (distributed only - centralized has no per-tick
"insufficient evidence" concept to count); `replay` = deterministic-replay
check (same seed rerun).

| scenario | conn | central found/false | dist found/false | gnd (c/d) | obs (c/d) | dist dup/qf | replay |
|---|---|---|---|---|---|---|---|
| perfect_communication | 1.000 | 1/10 | 1/10 | 0/0 | 0/0 | 456/2925 | identical |
| moderate_packet_loss | 0.997 | 1/7 | 1/11 | 0/0 | 0/0 | 337/2968 | identical |
| high_packet_loss | 0.997 | 1/4 | 1/13 | 0/0 | 0/2 | 91/2665 | identical |
| high_latency | 0.920 | 1/10 | 1/25 | 0/0 | 0/0 | 609/2767 | identical |
| communication_partition | 0.090 | 1/6 | 1/9 | 0/0 | 2/1 | 62/2753 | identical |
| partition_healing | 0.165 | 1/10 | 1/12 | 0/0 | 0/0 | 100/2968 | identical |
| false_positive_heavy | 0.997 | 1/107 | 1/115 | 0/0 | 0/0 | 1413/2978 | identical |
| dropout_heavy | 0.997 | 1/4 | 1/5 | 0/0 | 0/0 | 235/2792 | identical |
| delayed_sensing | 0.997 | 1/14 | 1/21 | 0/0 | 0/0 | 330/2802 | identical |
| mixed_degradation | 0.902 | 2/49 | 2/85 | 0/0 | 0/0 | 354/2931 | identical |
| obstacle_wedging | 0.751 | 3/20 | 4/25 | 0/0 | 9/9 | 943/12567 | identical |

**Reading this table**: `confirmed_victims` (true positives) matches or
exceeds centralized in every scenario; `false_confirmations` is
consistently higher for distributed under degraded connectivity (see "Degraded
communication can increase false confirmations" below - a disclosed,
explained property, not a defect); `drone_ground_contact_count` is 0/0
everywhere, including `obstacle_wedging` - the literal Phase 4.1
regression scenario - confirming no new ground-contact regression;
`drone_obstacle_contact_count` on `obstacle_wedging` is 9/9, matching
Phase 4.1's own reported result exactly (`docs/PHASE4_SAFETY.md`);
`deterministic_replay_result` is "identical" in all 11.

### Regression tests

- `python -m compileall -q swarm_sim run_mission.py tests`: clean.
- `python -m pytest -q tests`: **327 passed, 0 failed** (288 pre-existing +
  30 in `test_distributed_consensus.py` + 9 in
  `test_distributed_consensus_architecture.py`).
- No change to `safety_supervisor.py`, the sensor models, or the
  Boids/Vicsek/Couzin/Olfati-Saber equations - verified by not touching
  those files, and by the pre-existing Phase 3/4/4.1 test suites (all
  still passing unmodified).
- Phase 4.1's ground-contact regression fix (decoupled acceleration
  bounding, override-tier hysteresis, critical-altitude tier) is
  untouched code; the `obstacle_wedging` scenario below reruns it under
  both consensus modes as the required regression check.

### Safety-gate check

Every candidate command from either consensus path still reaches PyBullet
only through `SafetySupervisor.evaluate()` - `distributed_consensus.py`
has no reference to the controller, the supervisor, PID, or PyBullet, and
cannot call any of them (checked mechanically: no forbidden import, no
motor/PWM/rpm identifier, and neither `_submit_detections_distributed`
nor `_resolve_confirmed_detections_distributed` calls
`to_velocity_command`/`computeControlFromState`). `safety_override_count`
in the scenario results is identical between consensus modes in every
scenario tested (same trajectories under perfect communication; the
supervisor's own behavior is completely independent of which consensus
path produced a recruitment beacon).

### Degraded communication can increase false confirmations (not a bug)

`mixed_degradation` and the harsher packet-loss scenarios show
`false_confirmations` sometimes **higher** under distributed consensus
than centralized at the same true-positive (`confirmed_victims`) count
(e.g. `mixed_degradation`: centralized 49, distributed 85, both correctly
finding the same 2 real victims). This is not a quorum violation - every
one of those confirmations genuinely satisfied quorum with genuinely
distinct drones - and it is not an "unsupported confirmation" as the
acceptance criteria use the term. The mechanism: under good connectivity,
one node reaches quorum on a spurious noise cluster, broadcasts
`CONFIRMATION`, and every other node adopts it and stops independently
re-deriving it. Under degraded connectivity, that `CONFIRMATION` broadcast
is itself subject to loss/latency, so several nodes can independently
reach quorum on overlapping-but-not-identical noise-driven evidence
subsets before any of them converge - each one a distinct, individually
valid `confirmation_id`. This is a genuine, disclosed limitation of
peer-local consensus under degraded connectivity (false-positive risk can
rise, not just true-positive recall falling), not a hidden defect.

### A pre-existing safety edge case, found and deliberately not fixed here

The first `communication_partition` draft used `communication_radius=3.0`
(below every Couzin flocking zone radius - `r_repulsion=2.5`,
`r_orientation=4.0`, `r_attraction=6.0`). That produced a severe
ground-contact spike in one run (175 deduplicated ground contacts,
distributed mode, seed 7) - alarming enough to investigate immediately
rather than report as a passing number, per the standard this project
holds Phase 4/4.1 regressions to.

**Investigation**: the contact log (rich per-contact fields added in
Phase 4.1, still present) showed the drone in `GEOFENCE_RISK`,
`active_constraints=('altitude_floor_critical',)` at the moment of
contact - Phase 4.1's critical-altitude tier *was* firing - but
`current_command_velocity_mps` still carried a large horizontal component
(~3.8 m/s) alongside a barely-positive vertical one. Cause: at
`communication_radius=3.0`, flocking has no usable neighbor data at all
(the radius is smaller than even `r_repulsion`), so a drone can reach a
large horizontal velocity before `_evaluate_altitude_critical` ever
triggers; `bound_velocity_step`'s per-axis acceleration bound (Phase 4.1)
correctly decouples horizontal from vertical, but it still cannot
*instantly* zero a large existing horizontal velocity - deceleration
takes several ticks, during which the vehicle can graze the ground
repeatedly while horizontal velocity ramps down.

**Is this a Phase 5 regression?** No - verified directly: rerunning the
identical `communication_radius=3.0` config across 7 seeds in **both**
consensus modes shows ground contacts in **both** modes, seed-dependently
(seed 3: centralized 284, distributed 297; seed 4: centralized 97,
distributed 6; seed 7: centralized 0, distributed 175; four other seeds:
0 in both). This is a pre-existing `SafetySupervisor` limitation
(unbounded-horizontal-velocity-at-trigger-time), reachable by *either*
consensus mode given a bad-luck trajectory, not something distributed
consensus introduces. What distributed consensus *does* do is
independently confirmed earlier in this document: it produces a
genuinely different trajectory than centralized from the same seed (the
`np.mean` finding), which changes *which* seeds happen to hit this
pre-existing edge case - it does not change whether the edge case exists.

**Action taken**: `safety_supervisor.py` is explicitly out of scope this
phase ("Do not modify: safety_supervisor.py behavior or thresholds").
Rather than report a scenario that gratuitously exercises an out-of-scope
defect, `communication_partition`/`partition_healing` were retuned to
`communication_radius` >= 6.5 (above every flocking zone radius) -
verified safe (0 ground contacts, both modes) across 7 seeds at both 6.5
and 7.0 - while still producing genuine, substantial partitioning
(connectivity fraction as low as 0.0-0.12 at these radii, depending on
seed). The 3.0 finding is recorded here rather than discarded, since it
is a real, reproducible limitation worth a future safety-focused phase
picking up.

## Remaining risks

- **Confidence is not a quorum gate.** Carried through the whole
  pipeline but not currently thresholded, matching centralized behavior -
  flagged here rather than silently added as new scope.
- **Relay is unbounded in hop count across a run**, though bounded in
  volume (`new_relay_evidence` offers each `report_id` at most once per
  node) - a large, sparse, multi-hop swarm could see a report take many
  ticks to reach a distant quorum. Not measured directly in this phase's
  scenarios (the 5-6 drone swarms tested stay within one or two hops).
- **`memory_proxy_bytes_per_drone_approx` is a shallow proxy**, not a real
  profile - see "Approximate metrics."
- **Not fielded/certified.** Same caveat as Phase 4's safety supervisor:
  this is a simulation research exercise. Quorum thresholds, cluster
  radii, and relay policy are all tunable constants chosen for this
  scenario set, not validated against a real multi-agent deployment.

## Explicitly not started

Distributed consensus in this phase means peer-local message-passing
consensus **within the existing simulation**. Nothing here touches or
begins: MAVLink, Pixhawk, ArduPilot, or PX4 adapters; SITL; hardware
integration; real-flight support. `safety_supervisor.py`, the sensor
models, the Phase 3 flocking equations, and Phase 4.1's hysteresis/
altitude-protection code are byte-for-byte unmodified.
