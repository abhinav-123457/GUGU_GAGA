# Phase 3: Behavior Models

Corrects and tests the three classic local flocking models already in
this codebase (Reynolds Boids, Vicsek, Couzin) and adds a fourth,
Olfati-Saber-inspired controller. Every model remains a **candidate
command generator only** - `SwarmController.step()` (unchanged in its
role from Phase 2) is still the only thing that composes flocking with
search/recruitment/hard-avoidance and is still the only thing whose output
reaches the vehicle; no behavior model actuates anything directly, and
none of them bypass the future Phase 4 safety supervisor (which doesn't
exist yet, and isn't started here - see "Not started" at the bottom).

The Phase 2 sensor boundary (behavior models never see ground truth -
`swarm_sim/contracts.py`'s hidden-plant isolation, and the AST-based
architecture tests in `tests/test_architecture_ground_truth_boundary.py`)
and the Phase 2 diagnostic/contact-accounting code
(`swarm_sim/diagnostics.py`, `_classify_and_log_contacts`) are untouched.

## Common controller contract (`swarm_sim/behaviors/common.py`)

Every behavior function's public entry point (`*_steer`/`*_direction`/
`*_heading` for the raw math, `*_candidate_command` for the bounded,
contract-compliant wrapper) returns a 3-vector velocity in the LOCAL_ENU
frame (`swarm_sim.contracts.Frame.LOCAL_ENU`), units meters/second, z
always 0 (this simulator holds altitude via a separate position-hold
loop - see `mission.py`'s PID call).

**Native output categories** (which raw quantity each model computes
before bounding is applied - see `swarm_sim/behaviors/common.py`'s module
docstring for the full explanation):
- **Acceleration-shaped**: Boids, Olfati-Saber.
- **Heading-shaped**: Vicsek, Couzin.

Three shared guarantees, enforced by `common.py`'s helpers rather than
reimplemented per model:

- **Finite, never NaN** (`sanitize_vector`/`bounded_heading`): a
  degenerate raw result (zero-length, NaN, Inf - e.g. from a coincident
  neighbor or zero neighbors) always falls back to a supplied, already-
  finite value (typically the agent's current heading/velocity) instead
  of propagating.
- **Bounded**, in each model's own native quantity: Boids and the
  Olfati-Saber controller are acceleration-shaped (their steering force
  is directly interpretable as `u_i` in a double-integrator, matching
  Reynolds' and Olfati-Saber's own formulations) - `clamp_acceleration`
  bounds the raw force, then one Euler step is taken and `clamp_speed`
  bounds the result. Vicsek and Couzin are heading-shaped (they output a
  *direction*, not a force) - `clamp_turn_rate` bounds how fast that
  heading may change from the previous step, then the result is scaled to
  a speed and `clamp_speed` bounds that.
- **Deterministic under a fixed seed**: every RNG draw (Vicsek's angular
  noise) takes an explicit `rng` argument from the caller - no model
  seeds its own randomness - so identical `(state, rng)` inputs always
  reproduce identical output (`tests/test_behavior_contract.py`'s
  parametrized deterministic-replay tests, one per model, going through
  `SwarmController.step()` itself).

## Reynolds Boids (`swarm_sim/behaviors/boids.py`)

Separation + alignment + cohesion, corrected in Phase 3:
- **Explicit priority**: any neighbor within `priority_radius` (default
  `radius / 2`) makes separation *alone* the output - alignment/cohesion
  never get a chance to blend a too-close encounter away. Otherwise, a
  weighted blend (`w_sep=1.5 > w_align=w_coh=1.0` by default).
- **Coincident neighbors** (distance below `COINCIDENCE_EPS_M = 1e-6`)
  are excluded from every term, the same as "not visible" - the previous
  version instead clipped the divisor to `1e-6`, which is finite but
  produces a spuriously enormous force in exactly the case that has no
  well-defined separation direction anyway.
- `boids_candidate_command` is the bounded entry point (see common
  contract above); `boids_steer` is the raw, unbounded steering force,
  kept public for `SwarmController`'s existing blended-heading pipeline
  (unchanged call site: `boids.boids_steer(local_pos, local_vel, 0,
  perceived_ids, cfg.r_attraction)`).

## Vicsek (`swarm_sim/behaviors/vicsek.py`)

**This is the canonical baseline, not an approximation of it.** The
previous version had two real bugs: it averaged 3D velocity *vectors*
and added 3D Gaussian noise directly to the (already near-unit) vector,
rather than the paper's atan2 circular mean perturbed by angular noise. A
linear vector mean happens to approximate a circular mean when headings
are already close together, which is why it didn't look obviously wrong -
it breaks down for broadly-spread headings (e.g. two neighbors facing
exactly opposite ways) and doesn't match the paper's noise model at all.

- `mean_heading_angle`: sums unit heading vectors, then one `atan2` -
  correct across the +-pi wraparound, unlike averaging `atan2(y,x)`
  values directly. Returns `None` (not zero, not a wrong angle) when the
  sum is degenerate - handled by falling back to the agent's own heading.
- `vicsek_heading` (**SAR mode**, non-periodic): neighbor selection is
  entirely up to the caller - this simulator's bounded search arena has
  no wraparound, so there's nothing for this function to wrap. Noise is
  uniform in `[-noise_strength, +noise_strength]` radians
  (`MissionConfig.vicsek_noise`, unchanged field, now correctly
  interpreted as radians of angular noise rather than a Gaussian std on a
  raw vector).
- `vicsek_periodic_step` + `order_parameter` (**periodic regression
  mode**): a whole-population, toroidal-boundary update exactly matching
  Vicsek et al. (1995)'s own model - kept only so
  `tests/test_behaviors_vicsek.py` can check this package's math
  reproduces the textbook order-parameter-vs-noise phase transition.
  `SwarmController` never calls this; the arena isn't periodic.
- `vicsek_candidate_command`: the bounded entry point, added in Phase 3
  so Vicsek has the same turn-rate/speed-bounded contract as the other
  three models (`SwarmController`'s existing call site still uses the raw
  `vicsek_heading`, unchanged, for its blended-heading pipeline).

## Couzin (`swarm_sim/behaviors/couzin.py`)

Zonal repulsion/orientation/attraction with strict priority (repulsion
always wins) and a field-of-view blind zone, mostly unchanged in
substance from Phase 2 - Phase 3 adds:
- `couzin_candidate_command`, the turn-rate- and speed-bounded entry
  point (previously this bounding only happened far downstream, inside
  `SwarmController.step()`'s end-of-step rate limiter, shared across all
  three old models rather than being part of Couzin's own contract).
- Explicit documentation and tests for what was already true but
  incidental: a coincident neighbor (distance below `1e-9`) is excluded
  from every zone, the same as "out of range" - deterministic, no
  randomness, never NaN.
- Repulsion is not FOV-gated (a collision risk from directly behind still
  triggers avoidance) - orientation/attraction are.

## Olfati-Saber-inspired controller (`swarm_sim/behaviors/olfati_saber.py`)

New in Phase 3. Explicitly split so it is never mistaken for a
guarantee this codebase doesn't provide:

**From the paper** (Olfati-Saber 2006), implemented per its published
formulas: the sigma-norm (`sigma_norm`, eq. 3.3 - a smooth stand-in for
Euclidean distance with no gradient singularity at zero separation) and
its gradient direction (`sigma_gradient`, eq. 3.4); the bump function
(`bump`, eq. 3.6) and pairwise action function (`phi_action`/`phi_alpha`,
eq. 3.9-3.11) that together produce the spacing term (smooth repulsion
below the desired distance `d_alpha`, attraction above it, zero at
`d_alpha`, cut off entirely beyond sensing range `r_alpha`); the
velocity-consensus alignment term (eq. 3.13's bump-weighted `p_j - p_i`
sum). `d_alpha`/`r_alpha` are taken from config in real meters
(`os_desired_spacing_m`/`os_interaction_range_m`) and converted to the
sigma-norm scale internally (`_sigma_norm_scalar`) - a real bug caught
while writing `tests/test_behaviors_olfati_saber.py`: comparing a
sigma-norm distance directly against a raw-meter range silently cuts off
every real neighbor far too early, since `sigma_norm(d) > d` for any
`d > 0`.

**Engineering approximations, not from the paper's formal guarantees**:
- *Navigation*: the paper tracks a dedicated virtual "gamma-agent" with
  its own dynamics; `SwarmController`'s `olfati_saber` branch instead
  passes the drone's own position as `target_pos` (no positional pull)
  and the current search heading at cruise speed as `target_vel` - there
  is no virtual-leader subsystem in this codebase.
- *Connectivity/fragmentation* (`build_adjacency`, `connected_components`,
  `is_fragmented`, `algebraic_connectivity`): these **measure** a
  snapshot of the locally-sensed interaction graph. The paper's
  connectivity-preservation theorems assume continuous-time gradient
  dynamics with no sensing noise, latency, or dropout; this simulator's
  sensing is noisy, latent, and dropout-prone (Phase 2) and control is
  discrete-step. Nothing here proves or enforces connectivity - it is a
  monitoring/diagnostic tool, used as such in the benchmarks below.

`olfati_saber_candidate_command` follows Boids' acceleration-shaped
bounding pattern (bound accel, integrate one step, bound speed) - see the
common contract section.

## Required tests

17 required items, all covered by real (non-trivial) assertions:
`tests/test_behaviors_common.py` (contract helpers), `test_behaviors_boids.py`,
`test_behaviors_vicsek.py`, `test_behaviors_couzin.py`,
`test_behaviors_olfati_saber.py` (per-model correctness), and
`test_behavior_contract.py` (cross-cutting: deterministic replay, finite
outputs, ground-truth isolation, no-actuation-path, all parametrized
across all 4 models through `SwarmController.step()` itself, not just the
raw behavior functions). 85 new tests; full suite 222 passed, 0 failed,
0 skipped (`python -m compileall -q swarm_sim run_mission.py tests` /
`python -m pytest -q`).

## Benchmarks

`scripts/run_phase3_benchmarks.py`. 10 scenarios are a lightweight,
deterministic, PyBullet-free kinematic harness built on the *actual*
`swarm_sim.network.CommsNetwork` (same range/dropout/latency model
`SwarmController` uses in production) feeding each `*_candidate_command`
entry point every step; the 11th (obstacle-wedging) reruns the real
PyBullet `FloodSearchMission` with Phase 2 scenario 6's exact config, once
per behavior model, since a synthetic harness has no obstacles to wedge
against. Full results (44 rows: 4 models x 11 scenarios):
`results/phase3_benchmarks/phase3_benchmark_results.json` (gitignored -
regenerate with `python scripts/run_phase3_benchmarks.py`).

**Important interpretation caveat**: scenarios 1-10 call each behavior
model's raw candidate-command function directly - they measure the
flocking rule alone, *not* the full `SwarmController.step()` pipeline,
which layers a separate hard-avoidance override on top (see
`SwarmController._avoidance_vector`). A small worst-case clearance number
below reflects what the flocking rule alone would do, never what this
simulator's actual collision-safety layer allows through in scenario 11 or
in flight.

**Headline results**:
- **Zero finite-output failures and zero speed-bound violations across
  all 4 models x all 11 scenarios** (440 model-scenario-agent-step
  aggregates) - the common-contract fixes hold up under noise, dropout,
  latency, a full communication partition, and 30-agent density.
- **Acceleration- vs turn-rate-bounded models trade one failure mode for
  the other, exactly as their different native contracts predict**: Boids
  and Olfati-Saber (acceleration-bounded) show 0 acceleration violations
  everywhere but nonzero turn-rate excursions at low speed (e.g. Boids
  dense_swarm: 398 turn-rate "violations" against the evaluation
  threshold, despite never exceeding its own accel/speed bounds); Vicsek
  and Couzin (turn-rate-bounded) show 0 turn-rate violations everywhere
  but large acceleration-metric counts at cruise speed (Couzin
  dense_swarm: 5879) - because for circular motion `a = v*omega`, a
  turn-rate cap does not imply a low instantaneous acceleration at cruise
  speed. **Neither is a contract violation** - Couzin/Vicsek were only
  ever required to bound speed and turn rate, not acceleration - but it
  is a real, measured property worth carrying into Phase 4: a real
  airframe has an acceleration limit regardless of which quantity a given
  flocking rule natively bounds. Not fixed here (would be new scope).
- **Fragmentation/connectivity metrics behave exactly as expected**:
  `sparse_swarm` and `communication_partition` both show
  `fragmentation_duration_s == 10.0` (the full benchmark duration) and
  `connectivity_mean ~= 0` for every model - agents that are never in
  radio range of each other are correctly measured as permanently split,
  not spuriously "connected".
- **Scenario 11 (obstacle-wedging, Phase 2's scenario 6 config, one real
  PyBullet run per model)**:

  | Model | Victims found | Worst ground-truth clearance | Contact-steps (dedup events) | Swarm connectivity |
  |---|---|---|---|---|
  | boids | 3/5 | 0.793 m | 139 (157) | 0.498 |
  | vicsek | 2/5 | 1.328 m | 109 (109) | 0.879 |
  | couzin (Phase 2 baseline) | 2/5 | 0.602 m | 229 (233) | 0.684 |
  | olfati_saber | 2/5 | 1.287 m | **0 (0)** | 0.999 |

  Couzin's numbers exactly reproduce the Phase 2 diagnostic-reconciliation
  report's 291 raw / 233 deduplicated / 229 contact-step figures -
  confirms this rerun is bit-for-bit consistent with that earlier work.
  The olfati_saber run produced **zero** PyBullet contacts of any kind in
  this scenario and the highest swarm connectivity of the four, at the
  cost of one fewer victim found than boids - a genuinely interesting
  result from a single seed/scenario, not a general claim that it is
  "safer"; it is reported here as a benchmark finding to follow up on, not
  a conclusion. `cpu_time_per_control_step_s` for scenario 11 measures
  sensing+consensus time (mission.py doesn't separately instrument
  steering CPU time), unlike scenarios 1-10 where it measures the
  behavior function's own call time directly - not a fair like-for-like
  comparison across the two harnesses; reported separately, not merged.

## Not started (out of scope this phase, confirmed)

Phase 4 safety supervisor, peer-local distributed consensus, and any
MAVLink/Pixhawk/ArduPilot/PX4/SITL/hardware integration work. No file
under those areas was touched.
