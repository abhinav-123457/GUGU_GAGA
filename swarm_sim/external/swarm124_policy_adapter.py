"""Phase 15A/15B: bounded adapter for Swarm124 (swarm-subnet/swarm,
Bittensor Subnet 124) policy-style actions.

**Verified from swarm-subnet/swarm's own README** (commit fce8d30, see
docs/PHASE15_SWARM124_LANGOSTINO_INTEGRATION.md): observations are a
single front-facing depth camera plus the vehicle's own position/speed -
explicitly NO GPS, map, or obstacle list - updated at 50Hz; actions are a
continuous "direction, speed, turn" sent at 50Hz; missions include an
explicit team "Swarm Search and Rescue" mission. None of that README
specifies an exact array shape, a policy-version/model-checksum field, or
a 2-8 teammate-relative observation shape - those are this project's own
INFERRED extension (documented as such), needed only so this project's
own offline fixture (Phase 15B) has something concrete to validate
against; nothing here is copied from or claims to run Swarm124's own code.

This module never returns a raw MAVLink/RC/PWM command. It only ever
produces a `swarm_sim.safety_supervisor.CandidateCommand` via
`external_contracts.validate_and_build_candidate()` - the same untrusted
type `SwarmController` itself produces - so it must still pass through
`SafetySupervisor.evaluate()` exactly like every other candidate (see
`convert_swarm124_action_to_candidate` below, and
tests/test_phase15_swarm124_adapter.py's
`test_swarm124_action_never_bypasses_safety_supervisor`).
"""
from __future__ import annotations

import dataclasses
import math
from typing import Optional, Tuple

from ..contracts import Frame, Vec3
from .external_contracts import (
    ExternalActionRejection, ExternalAdapterResult, ExternalPolicyAction, validate_and_build_candidate,
    validate_observation_shape,
)

# This project's own bounded extension of the verified Swarm124 observation
# (depth + position/speed) to the "2-8 synthetic agents offline" case Phase
# 15B asks for - a fixed-length teammate slot count, not a Swarm124 spec.
MAX_TEAMMATES = 7            # up to 8 agents total (self + 7 teammates)
DEPTH_BEAM_COUNT = 8         # a coarse, bounded stand-in for "a single depth camera"


@dataclasses.dataclass(frozen=True)
class Swarm124Observation:
    """This project's own typed stand-in for a Swarm124-style observation -
    see module docstring for what is verified vs. inferred. `depth_returns_m`
    stands in for "a single depth camera" as a bounded tuple of ray
    lengths (never a real image); `teammate_relative_m` is this project's
    own addition for the 2-8-agent offline case, always relative-position
    tuples, never absolute teammate state and never hidden victim
    ground truth."""
    vehicle_id: str
    timestamp_s: float
    position_m: Vec3
    velocity_mps: Vec3
    depth_returns_m: Tuple[float, ...]              # length == DEPTH_BEAM_COUNT
    teammate_relative_m: Tuple[Vec3, ...]            # length in [0, MAX_TEAMMATES]
    seed: int
    run_id: str


@dataclasses.dataclass(frozen=True)
class Swarm124Action:
    """This project's own typed stand-in for a Swarm124-style action -
    "direction, speed, turn" per the verified README, plus the metadata
    (policy_version/model_checksum/seed/run_id/action_expiry_s) that
    README does not specify but this project's own validation pipeline
    requires of every external action (see external_contracts.py)."""
    vehicle_id: str
    timestamp_s: float
    direction_xy: Tuple[float, float]     # need not be pre-normalized
    speed_mps: float
    turn_rate_radps: float                # carried for fidelity to the verified action shape;
                                           # NOT forwarded into CandidateCommand (see module docstring)
    policy_version: str
    seed: int
    run_id: str
    action_expiry_s: float
    model_checksum: Optional[str] = None


def validate_observation(observation: Swarm124Observation) -> Optional[str]:
    """Malformed-observation / unexpected-dimension check for the INPUT to
    a policy - returns None if valid, else a human-readable reason. Kept
    separate from action validation (external_contracts.py) since a
    malformed observation should never even reach a policy, scripted or
    learned."""
    detail = validate_observation_shape("depth_returns_m", observation.depth_returns_m, DEPTH_BEAM_COUNT)
    if detail:
        return detail
    if not isinstance(observation.teammate_relative_m, tuple) or len(observation.teammate_relative_m) > MAX_TEAMMATES:
        return (f"teammate_relative_m must be a tuple of at most {MAX_TEAMMATES} entries, "
                f"got {observation.teammate_relative_m!r}")
    for i, teammate in enumerate(observation.teammate_relative_m):
        detail = validate_observation_shape(f"teammate_relative_m[{i}]", teammate, 3)
        if detail:
            return detail
    return None


class ScriptedSwarm124Policy:
    """A deterministic, non-learned policy matching the verified "direction,
    speed, turn" action shape - Phase 15B's own required first pass ("Use
    a deterministic scripted policy first. A learned model is optional and
    must not be required."). Simply steers toward a fixed target point at
    a bounded speed; the same (target, max_speed, seed) always produces
    the same action for the same observation - no RNG, no hidden state
    beyond the target itself."""

    policy_version = "scripted-v1"
    model_checksum = None   # honestly None - there is no model file for a scripted policy

    def __init__(self, target_xy: Tuple[float, float], max_speed_mps: float, seed: int, run_id: str):
        self.target_xy = target_xy
        self.max_speed_mps = max_speed_mps
        self.seed = seed
        self.run_id = run_id

    def act(self, observation: Swarm124Observation, action_expiry_s: float) -> Swarm124Action:
        dx = self.target_xy[0] - observation.position_m[0]
        dy = self.target_xy[1] - observation.position_m[1]
        dist = math.hypot(dx, dy)
        direction = (dx / dist, dy / dist) if dist > 1e-9 else (0.0, 0.0)
        speed = min(self.max_speed_mps, dist)
        return Swarm124Action(
            vehicle_id=observation.vehicle_id, timestamp_s=observation.timestamp_s, direction_xy=direction,
            speed_mps=speed, turn_rate_radps=0.0, policy_version=self.policy_version, seed=self.seed,
            run_id=self.run_id, action_expiry_s=action_expiry_s, model_checksum=self.model_checksum,
        )


def convert_swarm124_action_to_candidate(
    action: Swarm124Action, *, known_vehicle_ids: Tuple[str, ...], now_s: float, max_speed_mps: float,
    max_accel_mps2: float, max_staleness_s: float, previous_velocity_mps: Optional[Vec3] = None,
    previous_timestamp_s: Optional[float] = None,
) -> ExternalAdapterResult:
    """The only function that may turn a Swarm124-shaped action into a
    `CandidateCommand`. `turn_rate_radps` is deliberately dropped here -
    `CandidateCommand` (this project's untrusted velocity-only candidate
    type) has no yaw/turn field, and adding one would be a project-wide
    contract change out of Phase 15A's own bounded scope; a future phase
    could route it through `CommandType.YAW_RATE_SETPOINT` separately if
    ever needed."""
    if not isinstance(action.direction_xy, tuple) or len(action.direction_xy) != 2:
        return ExternalAdapterResult(
            accepted=False, candidate=None, rejection=ExternalActionRejection.UNEXPECTED_ACTION_DIMENSIONS,
            detail=f"direction_xy must be a 2-tuple, got {action.direction_xy!r}",
            policy_version=action.policy_version, seed=action.seed, run_id=action.run_id,
            model_checksum=action.model_checksum,
        )
    dir_x, dir_y = action.direction_xy
    dir_norm = math.hypot(dir_x, dir_y)
    if math.isfinite(dir_norm) and dir_norm > 1e-9:
        dir_x, dir_y = dir_x / dir_norm, dir_y / dir_norm
    elif math.isfinite(dir_norm):
        dir_x, dir_y = 0.0, 0.0
    # else: dir_norm is NaN/inf (direction_xy itself held a NaN/inf
    # component) - dir_x/dir_y are deliberately left unchanged (still
    # NaN/inf) rather than silently defaulted to (0, 0), so the velocity
    # built below still carries the bad value and
    # validate_and_build_candidate's finite-value check rejects it
    # explicitly, instead of this function masking it as an innocuous
    # zero-velocity command (a real bug caught by this module's own
    # smoke test - see tests/test_phase15_swarm124_adapter.py).
    velocity_mps = (dir_x * action.speed_mps, dir_y * action.speed_mps, 0.0)
    external_action = ExternalPolicyAction(
        vehicle_id=action.vehicle_id, desired_velocity_mps=velocity_mps, frame=Frame.LOCAL_ENU,
        timestamp_s=action.timestamp_s, action_expiry_s=action.action_expiry_s,
        policy_version=action.policy_version, seed=action.seed, run_id=action.run_id,
        model_checksum=action.model_checksum, source="swarm124_policy_adapter",
    )
    return validate_and_build_candidate(
        external_action, known_vehicle_ids=known_vehicle_ids, now_s=now_s, expected_frame=Frame.LOCAL_ENU,
        max_speed_mps=max_speed_mps, max_accel_mps2=max_accel_mps2, max_staleness_s=max_staleness_s,
        previous_velocity_mps=previous_velocity_mps, previous_timestamp_s=previous_timestamp_s,
    )
