# Phase 1 — Deterministic Simulator Contract

Defines `swarm_sim/contracts.py` (six versioned data interfaces),
`swarm_sim/seeding.py` (per-subsystem deterministic RNG), and
`swarm_sim/manifest.py` (run-manifest recording). **This phase adds types
only — no existing mission, controller, PyBullet, or MAVLink behavior was
changed.** See the Phase 1 report for the diff-free confirmation.

Not flight-certified. This is a data-contract layer for a research
simulator, not a safety system.

## Contract version

`swarm_sim.contracts.CONTRACT_VERSION = "0.1.0"`. Every dataclass carries
this as a `contract_version` field, defaulted so existing call sites don't
need to pass it. Bump it (semver) whenever a field is added, removed,
renamed, or its meaning changes; a manifest's `contract_version` is what
later lets a replay tool tell "this event log was written against an older
contract shape."

## Units and coordinate conventions

| Quantity | Unit |
|---|---|
| Position | meters |
| Velocity | meters/second |
| Acceleration | meters/second² |
| Angle (attitude, yaw) | radians |
| Angular velocity / yaw rate | radians/second |
| Time | seconds (`sim_time_s`), plus integer `step_index` where the discrete step matters |
| Confidence / probability-like fields | unitless, `[0, 1]` |
| Battery | fraction, `[0, 1]`, `1.0` = full |

**Frame is an explicit field, never assumed**, via `contracts.Frame`:
- `LOCAL_ENU` — x/y meaning is an *application-level convention*, not a
  PyBullet property (see below); z=Up is a real PyBullet convention.
- `LOCAL_NED` — x=North, y=East, z=Down. This one genuinely is fixed by
  the MAVLink protocol (`SET_POSITION_TARGET_LOCAL_NED`), not by this
  project's choice.
- `BODY` — vehicle body frame.

**PyBullet is not inherently ENU.** PyBullet is a generic Z-up rigid-body
world (gravity is applied along -Z — that part is a real PyBullet
convention); what its X/Y axes represent is whatever the calling code
decides. `mission.py` today chooses to treat PyBullet's world frame as
`LOCAL_ENU`, and that choice — not a PyBullet guarantee — is written down
once, as data, in `contracts.PYBULLET_LOCAL_ENU_AXIS_CONVENTION`:

```python
PYBULLET_LOCAL_ENU_AXIS_CONVENTION = {
    "x": "application-chosen East-like axis (PyBullet world +X)",
    "y": "application-chosen North-like axis (PyBullet world +Y)",
    "z": "Up (PyBullet world +Z) - an actual PyBullet convention",
}
```

`tests/test_frame_convention.py` is a fixture pinning this down so Phase 6
has one documented mapping to write real conversion tests against, instead
of each call site re-deriving what "ENU" was assumed to mean here.

No function in `contracts.py` converts between frames. That conversion is
explicitly out of scope until Phase 6 (the autopilot adapter) — converting
silently is exactly how last session's `swarm_bridge.py` bug happened
(mixing position/velocity fields across an implicit frame assumption).

## Timestamp semantics

- All timestamps are simulation seconds (`sim_time_s`, `sensor_timestamp_s`,
  `sample_timestamp_s`, `delivery_timestamp_s`, `timestamp_s`), always
  `>= 0`, validated in every `__post_init__`.
- `Command.expiration_time_s` must be strictly greater than
  `Command.timestamp_s` — a command that expires at or before it was issued
  cannot be constructed. `Command.is_expired(now_s)` is `now_s >
  expiration_time_s`.
- `NeighborObservation.packet_age_s` must equal
  `delivery_timestamp_s - sample_timestamp_s` (checked to `1e-6`) — this is
  a derived quantity, kept as its own field (per the Phase 1 spec) but
  validated for consistency rather than trusted as an independent input,
  so a caller can't construct a self-contradictory observation.

## Ownership and mutability

Every vector-shaped field (`Vec2 = Tuple[float, float]`,
`Vec3 = Tuple[float, float, float]`) is a plain Python `tuple`, never a
NumPy array. This is a structural choice, not a convention to remember:

- A tuple cannot be mutated in place — there is no `[i] =` that could
  corrupt a value another object is still holding a reference to.
- A tuple cannot be aliased into accidental sharing the way
  `positions[i]` (a NumPy array slice) can be handed to two different
  callers who each assume they own it.

Every dataclass is `frozen=True`, so an entire contract object also can't be
reassigned field-by-field after construction. The existing NumPy-based code
(`mission.py`, `controller.py`, `swarm_bridge.py`) is unaffected — nothing
there was changed in this phase, and adopting these contracts later just
means converting at the boundary (`np.array(some_vec3)` in, `tuple(arr)`
out), always producing a fresh array.

## Validation

Every dataclass validates itself in `__post_init__` and raises `ValueError`
immediately on:
- non-finite numbers (NaN always rejected; `+inf` is rejected everywhere
  **including `dt_s` and `radius_m`**, except `SensorObservation.range_returns_m`,
  where `+inf` is the legitimate "no return within max range" sentinel)
- `bool` passed where a genuine number or int is required — Python's `bool`
  is a subclass of `int`, so `True`/`False` would otherwise silently pass
  as `1`/`0` for `battery_fraction`, `step_index`, `radius_m`,
  `master_seed`, etc.; every numeric/int check explicitly excludes `bool`
- wrong vector length (`Vec2`/`Vec3` shape mismatches)
- out-of-range confidence/probability/battery fields (must be in `[0, 1]`)
- negative timestamps, or expiry not strictly after issue time
- wrong type for an enum field (e.g. passing the string `"NORMAL"` where a
  `SafetyState` is required — enums are never auto-coerced from strings at
  construction, only through `from_dict`, see below)
- empty ID/source strings, and empty strings inside
  `SafetyDecision.active_constraints`
- non-`bool` elements inside `SensorObservation.occluded`
- `NeighborObservation` cross-field inconsistency (packet age vs. the two
  timestamps it's derived from; delivery before sample)
- **`Command` field combinations that don't match its `command_type`** —
  see below
- **`SafetyDecision.accepted`/`filtered_command` inconsistency**, and a
  `filtered_command` whose `vehicle_id` doesn't match the decision's own —
  see below

### Command semantics

Every `Command` is single-purpose: exactly the fields its `command_type`
implies must be set, everything else must be `None`. This is enforced by a
per-type required/forbidden table in `contracts.py`
(`_COMMAND_TYPE_RULES`), not left as a combination the caller has to get
right on their own:

| `command_type` | Requires | Forbids |
| --- | --- | --- |
| `POSITION_SETPOINT` | `desired_position_m` | `desired_velocity_mps`, `yaw_rad`, `yaw_rate_radps` |
| `VELOCITY_SETPOINT` | `desired_velocity_mps` | `desired_position_m`, `yaw_rad`, `yaw_rate_radps` |
| `YAW_SETPOINT` | `yaw_rad` | `desired_position_m`, `desired_velocity_mps`, `yaw_rate_radps` |
| `YAW_RATE_SETPOINT` | `yaw_rate_radps` | `desired_position_m`, `desired_velocity_mps`, `yaw_rad` |
| `HOLD` / `LAND` / `ABORT` | (none) | all four setpoint fields |

A `POSITION_SETPOINT` can never also carry a velocity field, and vice
versa — mixing position and velocity fields on one setpoint is exactly the
ArduCopter `type_mask` bug `swarm_bridge.py` hit last session, so the
contract makes that combination impossible to construct rather than
possible-but-wrong.

### SafetyDecision semantics

- `accepted == True` **iff** `filtered_command is not None`: an accepted
  decision is the one that says what was actually allowed through, so it
  must carry that command; a rejected decision sent nothing on, so it must
  not carry one.
- When `filtered_command` is present, its `vehicle_id` must equal the
  `SafetyDecision`'s own `vehicle_id` — a decision can't be about one
  vehicle while filtering a command addressed to another.

## The six contracts

| Type | Purpose |
|---|---|
| `WorldState` | Sim time/step/dt, frame, static obstacle geometry, geofence, and victim ground truth **used only for scoring** — never a valid controller input |
| `VehicleState` | One vehicle's kinematic state, battery, health, estimator validity, last-valid-command time |
| `SensorObservation` | A vehicle's own noisy local sensing: range returns (with occlusion/dropout), FOV, pose uncertainty, victim-detection candidates |
| `NeighborObservation` | What one vehicle received about another, through the (simulated) comms network — sample vs. delivery time, confidence, staleness |
| `Command` | A bounded setpoint: position or velocity, yaw or yaw-rate, timestamped with an expiry, tagged with a source and confidence |
| `SafetyDecision` | The (future) supervisor's verdict on a `Command`: accepted/rejected, the filtered command actually allowed through, active constraints, reason, emergency state, clearance/TTC |

`SafetyDecision.emergency_state` uses `contracts.SafetyState`, which already
enumerates all 11 states from the eventual safety-supervisor phase (`NORMAL`
through `LAND_REQUESTED`). Defining the enum now is vocabulary, not
behavior — no supervisor exists yet to produce these values; that's a later
phase, deliberately not touched here.

## Serialization format

`contracts.to_dict(obj)` recursively converts any contract dataclass (or
plain value) to a JSON-compatible structure: `Enum` → `.value`, nested
dataclass → `dict`, `tuple`/`list` → `list`, everything else passed through.
`contracts.from_dict(cls, data)` reverses it, field-name-driven against
this module's fixed set of enum/nested-dataclass/tuple fields (deliberately
not a fully generic typing-introspection deserializer — this module has a
small, fixed shape, and field-name-driven reconstruction is easier to audit
than one that reflects on type hints).

`manifest.RunManifest` additionally provides `.to_dict()`/`.from_dict()` via
plain `dataclasses.asdict` (its fields are already flat JSON-compatible
types — dicts, strings, ints) and `.save_json(path)`/`.load_json(path)` for
round-tripping through a file.

## Deterministic seeding

`swarm_sim.seeding.SeedManager(master_seed)` replaces the single shared
`np.random.default_rng(config.seed)` that today drives obstacle placement,
victim placement, network dropout, and every internal `SwarmController`
random draw from one stream (`mission.py:41`). Each subsystem gets its own
`np.random.Generator`, deterministically derived via
`derive_seed(master_seed, subsystem_name)` — a stable SHA-256-based hash
(not Python's salted `hash()`), so the same `(master_seed, subsystem)` pair
always yields the same derived seed and the same draw sequence, in any
process, on any run. Calling `.rng("network")` twice returns the *same*
generator object — each subsystem's stream persists across calls rather
than resetting.

This phase does **not** wire `SeedManager` into `mission.py` — that
migration (replacing `self.rng` there with per-subsystem streams from a
`SeedManager`) is left for a later phase, since it would change which exact
random draw obstacle/victim placement gets on a given run, and Phase 1 must
not change existing behavior.

## Run-manifest recording

`swarm_sim.manifest.build_run_manifest(config, master_seed, seed_manager,
map_id, model_id, event_log=None)` assembles a `RunManifest` recording:
contract version, the run's config (as a flat dict), the master seed, every
subsystem seed `seed_manager` had actually derived by that point, a code
version, Python version, key dependency versions, a map/scenario id, a
model id, and (optionally) a hash of an event log.

**Code version**: this repository has no `.git` directory today. Rather
than silently omitting code identity, `get_code_version()` falls back to a
SHA-256 content hash over every `swarm_sim/*.py` file (path + bytes, sorted)
— so two runs against byte-identical code always report the same
`code_version`, and any source edit (committed or not) changes it. If a
`.git` directory appears later, the same function automatically prefers
`git rev-parse HEAD` (suffixed `+dirty` when the working tree has
uncommitted changes) with no caller change needed.

**Event log hash**: Phase 1 produces no event log (that's a later phase's
job), so `event_log_hash` is `None` on a manifest built today. The hashing
utility (`manifest.hash_event_log`) is ready for that phase to call.

## Compatibility policy

`check_contract_version_compatible()` enforces this policy on every one of
the six contracts and on `RunManifest`, in **both** places a
`contract_version` value can enter the system: `from_dict` (below) and
each type's own `__post_init__`. Direct construction with an incompatible
or malformed version — e.g. `Command(..., contract_version="99.0.0")` —
fails immediately, the same as it would through deserialization; version
checking isn't only a deserialization-path concern.

`from_dict` **enforces** this policy too, on every one of the six
contracts and on `RunManifest` — it is not just documentation:

- For any type that carries a `contract_version` field, `from_dict`
  requires that key to be present in the serialized data and checks its
  **major** version against `contracts.CONTRACT_VERSION`'s major version
  via `check_contract_version_compatible()`. A major-version mismatch
  raises `ValueError` immediately, naming both versions — reconstruction
  never silently proceeds against a shape it can't verify.
- Within the same major version, any minor/patch difference is accepted.
  That's what "additive, non-breaking" is for: a new optional field or a
  new enum member bumps minor/patch, and old data missing that field still
  reconstructs correctly.
- Every field is otherwise **required** in `data` — `from_dict` computes
  exactly which field names are missing and raises one `ValueError` naming
  all of them (`"{Type}.from_dict missing required fields: [...]"`),
  rather than letting a missing key fall through to a constructor
  `TypeError` on some other field's default, or reconstruct silently
  wrong. The nested helper types (`Obstacle`, `GeofenceSpec`,
  `VictimGroundTruth`, `DetectionCandidate`) have no `contract_version`
  field, so for those every field is required and no version check runs.
- `RunManifest` follows the same rule, with exactly one documented
  exception: `event_log_hash` may be omitted from serialized data (it is
  `None` whenever a run produced no event log — true of every manifest
  Phase 1 itself builds, since Phase 1 adds no event logger).
- Breaking changes — removing a field, changing a field's type/unit/frame,
  or tightening validation to reject previously-valid values — must bump
  `CONTRACT_VERSION`'s major component. Old serialized data then fails
  `from_dict` loudly, by design, rather than silently parsing into the
  wrong shape.

## Running tests

```bash
./.venv/Scripts/python.exe -m pip install -r requirements-dev.txt   # once, adds pytest
./.venv/Scripts/python.exe -m pytest -q
```

`pytest.ini` sets `testpaths = tests`, so this command — and a bare
`pytest -q` run from the repo root — only collects this project's own
`tests/`. Without it, pytest would also walk
`third_party/gym-pybullet-drones/tests/`, which needs that vendored
package's full (non-`--no-deps`) requirements — including `torch` and
`stable-baselines3` — installed to fully pass; this project deliberately
skips those RL-training extras (see `README.md`'s Setup section) since
nothing here imports them.

To run the vendored package's own tests separately:

```bash
./.venv/Scripts/python.exe -m pytest third_party/gym-pybullet-drones/tests -q
```

As of this phase that yields 4 passed, 1 failed
(`test_examples.py::test_learn` — needs `torch`, which is intentionally
not installed). That failure is pre-existing and orthogonal to this
project's own code; it is not part of "the project test suite."
