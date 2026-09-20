"""Phase 15A: the shared, bounded validation pipeline every external
research-reference output (Swarm124 policy actions today; any future
external source) must pass through before it may become a
`swarm_sim.safety_supervisor.CandidateCommand`.

This module is deliberately the ONLY place external output is validated -
`swarm124_policy_adapter.py` builds an `ExternalPolicyAction` and calls
`validate_and_build_candidate()` here; it never constructs a
`CandidateCommand` itself. Mirrors this project's existing pattern
(`swarm_sim.contracts`) of one shared validation layer instead of each
call site re-implementing checks slightly differently.

Validating here does NOT replace `SafetySupervisor.evaluate()` - it is a
strictly earlier, stricter gate that rejects malformed/untrustworthy
*external* input before it is even allowed to become a normal internal
candidate. A candidate that passes this module still goes through the
exact same `SafetySupervisor.evaluate()` as every other candidate in the
project (see `swarm124_policy_adapter.convert_action_to_candidate`) - this
module has no authority to accept a command on its own; it can only
reject one before the supervisor ever sees it.
"""
from __future__ import annotations

import dataclasses
import math
from enum import Enum
from typing import Optional, Tuple

from ..contracts import Frame, Vec3
from ..safety_supervisor import CandidateCommand

_MAX_ACTION_DIMENSIONS = 3  # (vx, vy, vz)-shaped velocity actions only, this phase


class ExternalActionRejection(Enum):
    """Every reason `validate_and_build_candidate()` can refuse an
    external action - one member per bullet in
    docs/PHASE15_SWARM124_LANGOSTINO_INTEGRATION.md's "Reject" list, so a
    test can assert on the exact reason, not just "it was rejected"."""
    NAN_OR_INF_VALUE = "NAN_OR_INF_VALUE"
    WRONG_UNITS = "WRONG_UNITS"
    WRONG_FRAME = "WRONG_FRAME"
    STALE_OUTPUT = "STALE_OUTPUT"
    EXPIRED_OUTPUT = "EXPIRED_OUTPUT"
    UNKNOWN_VEHICLE_ID = "UNKNOWN_VEHICLE_ID"
    DUPLICATE_VEHICLE_ID = "DUPLICATE_VEHICLE_ID"
    SPEED_LIMIT_EXCEEDED = "SPEED_LIMIT_EXCEEDED"
    ACCEL_LIMIT_EXCEEDED = "ACCEL_LIMIT_EXCEEDED"
    MALFORMED_OBSERVATION = "MALFORMED_OBSERVATION"
    UNEXPECTED_ACTION_DIMENSIONS = "UNEXPECTED_ACTION_DIMENSIONS"
    MISSING_SEED_OR_RUN_ID = "MISSING_SEED_OR_RUN_ID"
    MISSING_POLICY_VERSION = "MISSING_POLICY_VERSION"


@dataclasses.dataclass(frozen=True)
class ExternalPolicyAction:
    """The untrusted, unvalidated shape every external policy adapter must
    normalize its own output into before calling
    `validate_and_build_candidate()`. Deliberately NOT validated in
    __post_init__ - exactly like `CandidateCommand`, this type may legally
    hold NaN/Inf/wrong-length data; catching that is this module's job,
    not this dataclass's constructor."""
    vehicle_id: str
    desired_velocity_mps: Tuple[float, ...]   # must be exactly 3 finite floats to be accepted
    frame: Frame
    timestamp_s: float
    action_expiry_s: float                    # absolute sim time this action is valid until
    policy_version: Optional[str]
    seed: Optional[int]
    run_id: Optional[str]
    model_checksum: Optional[str] = None
    source: str = "external_policy"


@dataclasses.dataclass(frozen=True)
class ExternalAdapterResult:
    """What `validate_and_build_candidate()` always returns - exactly one
    of (candidate, rejection) is populated, mirroring
    `SafetyDecision`'s own accepted-XOR-filtered_command invariant."""
    accepted: bool
    candidate: Optional[CandidateCommand]
    rejection: Optional[ExternalActionRejection]
    detail: str
    policy_version: Optional[str] = None
    model_checksum: Optional[str] = None
    seed: Optional[int] = None
    run_id: Optional[str] = None


def _reject(reason: ExternalActionRejection, detail: str, action: ExternalPolicyAction) -> ExternalAdapterResult:
    return ExternalAdapterResult(
        accepted=False, candidate=None, rejection=reason, detail=detail,
        policy_version=getattr(action, "policy_version", None), seed=getattr(action, "seed", None),
        run_id=getattr(action, "run_id", None), model_checksum=getattr(action, "model_checksum", None),
    )


def _is_finite_number(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def validate_and_build_candidate(
    action: ExternalPolicyAction, *, known_vehicle_ids: Tuple[str, ...], now_s: float,
    expected_frame: Frame, max_speed_mps: float, max_accel_mps2: float, max_staleness_s: float,
    previous_velocity_mps: Optional[Vec3] = None, previous_timestamp_s: Optional[float] = None,
) -> ExternalAdapterResult:
    """The single, shared validation pipeline for every external policy
    action in this project (see module docstring). Every check below maps
    directly to one bullet in docs/PHASE15_SWARM124_LANGOSTINO_INTEGRATION.md's
    "Reject" list - checked in a fixed order so the same malformed input
    always produces the same rejection reason, not whichever check
    happens to run first.
    """
    if not action.policy_version:
        return _reject(ExternalActionRejection.MISSING_POLICY_VERSION,
                        "external action carries no policy_version", action)
    if action.seed is None or not action.run_id:
        return _reject(ExternalActionRejection.MISSING_SEED_OR_RUN_ID,
                        "external action carries no seed and/or run_id", action)

    if not isinstance(action.vehicle_id, str) or not action.vehicle_id:
        return _reject(ExternalActionRejection.UNKNOWN_VEHICLE_ID,
                        "external action has no vehicle_id", action)
    if action.vehicle_id not in known_vehicle_ids:
        return _reject(ExternalActionRejection.UNKNOWN_VEHICLE_ID,
                        f"vehicle_id {action.vehicle_id!r} is not a known vehicle", action)
    if len(known_vehicle_ids) != len(set(known_vehicle_ids)):
        return _reject(ExternalActionRejection.DUPLICATE_VEHICLE_ID,
                        f"known_vehicle_ids contains a duplicate: {known_vehicle_ids!r}", action)

    if not isinstance(action.desired_velocity_mps, tuple) or len(action.desired_velocity_mps) != _MAX_ACTION_DIMENSIONS:
        return _reject(ExternalActionRejection.UNEXPECTED_ACTION_DIMENSIONS,
                        f"desired_velocity_mps must be a {_MAX_ACTION_DIMENSIONS}-tuple, "
                        f"got {action.desired_velocity_mps!r}", action)
    for i, component in enumerate(action.desired_velocity_mps):
        if not _is_finite_number(component):
            return _reject(ExternalActionRejection.NAN_OR_INF_VALUE,
                            f"desired_velocity_mps[{i}] is not finite: {component!r}", action)
    if not _is_finite_number(action.timestamp_s):
        return _reject(ExternalActionRejection.NAN_OR_INF_VALUE,
                        f"timestamp_s is not finite: {action.timestamp_s!r}", action)
    if not _is_finite_number(action.action_expiry_s):
        return _reject(ExternalActionRejection.NAN_OR_INF_VALUE,
                        f"action_expiry_s is not finite: {action.action_expiry_s!r}", action)

    if not isinstance(action.frame, Frame):
        return _reject(ExternalActionRejection.WRONG_UNITS,
                        f"frame {action.frame!r} is not a swarm_sim.contracts.Frame member", action)
    if action.frame != expected_frame:
        return _reject(ExternalActionRejection.WRONG_FRAME,
                        f"expected {expected_frame}, got {action.frame}", action)

    if action.timestamp_s > now_s + 1e-6:
        return _reject(ExternalActionRejection.STALE_OUTPUT,
                        f"timestamp_s {action.timestamp_s} is in the future of now_s {now_s}", action)
    if now_s - action.timestamp_s > max_staleness_s:
        return _reject(ExternalActionRejection.STALE_OUTPUT,
                        f"action is {now_s - action.timestamp_s:.3f}s old, "
                        f"exceeds max_staleness_s={max_staleness_s}", action)
    if now_s > action.action_expiry_s:
        return _reject(ExternalActionRejection.EXPIRED_OUTPUT,
                        f"now_s {now_s} is past action_expiry_s {action.action_expiry_s}", action)

    speed = math.sqrt(sum(c * c for c in action.desired_velocity_mps))
    if speed > max_speed_mps + 1e-9:
        return _reject(ExternalActionRejection.SPEED_LIMIT_EXCEEDED,
                        f"requested speed {speed:.3f} m/s exceeds max_speed_mps={max_speed_mps}", action)

    if previous_velocity_mps is not None and previous_timestamp_s is not None:
        dt_s = action.timestamp_s - previous_timestamp_s
        if dt_s > 1e-6:
            delta = math.sqrt(sum((a - b) ** 2 for a, b in zip(action.desired_velocity_mps, previous_velocity_mps)))
            accel = delta / dt_s
            if accel > max_accel_mps2 + 1e-9:
                return _reject(ExternalActionRejection.ACCEL_LIMIT_EXCEEDED,
                                f"implied acceleration {accel:.3f} m/s^2 exceeds "
                                f"max_accel_mps2={max_accel_mps2}", action)

    candidate = CandidateCommand(
        vehicle_id=action.vehicle_id, desired_velocity_mps=tuple(float(c) for c in action.desired_velocity_mps),
        frame=action.frame, timestamp_s=action.timestamp_s, expiration_time_s=action.action_expiry_s,
        source=action.source,
    )
    return ExternalAdapterResult(
        accepted=True, candidate=candidate, rejection=None, detail="accepted",
        policy_version=action.policy_version, model_checksum=action.model_checksum,
        seed=action.seed, run_id=action.run_id,
    )


def validate_observation_shape(name: str, observation: Tuple[float, ...], expected_length: int) -> Optional[str]:
    """Shared malformed-observation check, reused by both the Swarm124
    fixture (depth/state arrays) and any future external observation
    shape - returns None if valid, else a human-readable rejection detail.
    Kept separate from `validate_and_build_candidate` because observation
    validation happens on the INPUT to a policy, before it ever produces
    an action, not on the action itself."""
    if not isinstance(observation, tuple):
        return f"{name} must be a tuple, got {type(observation).__name__}"
    if len(observation) != expected_length:
        return f"{name} must have length {expected_length}, got {len(observation)}"
    for i, value in enumerate(observation):
        if not _is_finite_number(value):
            return f"{name}[{i}] is not finite: {value!r}"
    return None
