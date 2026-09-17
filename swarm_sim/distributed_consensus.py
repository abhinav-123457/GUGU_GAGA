"""Phase 5: peer-local distributed consensus on victim detections.

Replaces the confirmation-decision path of the centralized ConsensusBoard
(swarm_sim/consensus.py, kept unmodified as a reference/comparison
implementation - see docs/PHASE5_DISTRIBUTED_CONSENSUS.md) with a model
where no single object holds the swarm's shared belief. Instead:

    drone-local sensor observation (VictimSensorModel - unchanged)
          |
          v
    DroneConsensusNode.report_detection()   <- this drone's OWN private state
          |
          v
    CommsNetwork message exchange (range-limited, lossy, latent - unchanged
    channel model, extended with a generic typed-message queue alongside
    its existing position/velocity broadcast)
          |
          v
    peer DroneConsensusNode.receive_message()  <- each recipient's OWN
          |                                        private state, mutated
          |                                        only by messages that
          |                                        actually arrived
          v
    DroneConsensusNode.try_confirm()  <- LOCAL quorum check, using only
          |                               evidence this node has itself
          |                               accumulated
          v
    local confirmation -> RecruitmentBoard.announce() (mission.py) -> ...
    exactly the existing SwarmController -> SafetySupervisor -> PyBullet
    pipeline, untouched.

No drone ever reads another drone's `DroneConsensusNode` attributes
directly - the only cross-node interaction is passing an immutable
`ConsensusMessage` through `CommsNetwork.send_message`/`poll_inbox`, and
even the mission-level `DistributedConsensus` orchestrator below only
calls each node's public methods and reads their return values, never
reaches into `_evidence`/`_confirmed_ids`/etc. This is checked mechanically
in tests/test_distributed_consensus_architecture.py.

Ground-truth isolation: nothing in this file reads WorldState, victim
ground truth, obstacle geometry, or the mission's global drone-position
array. The only positions this module ever sees are noisy, range/FOV/
occlusion-limited `DetectionCandidate.position_m` values that already
passed through VictimSensorModel (mission.py's job, same Phase 2
boundary), and the 2-D positions carried inside `Evidence`/messages that
originate from that same sensor path. See docs/PHASE2_SENSING.md.

Command boundary: this module never calls PyBullet, PID, motor, or speed-
control APIs, and produces no candidate command of any kind - it only
ever tells mission.py "this cluster of evidence now meets quorum," which
mission.py turns into a `RecruitmentBoard.announce()` call. From there the
existing SwarmController -> CandidateCommand -> SafetySupervisor.evaluate()
-> PyBullet pipeline is completely unchanged; this module cannot bypass it
because it has no reference to any of those objects.

Not implemented (out of scope for Phase 5 - see docs/PHASE5_DISTRIBUTED_CONSENSUS.md):
MAVLink, Pixhawk/ArduPilot/PX4 adapters, SITL, hardware integration, and
any change to safety_supervisor.py, the sensor models, or the flocking
behavior equations.
"""
from __future__ import annotations

import dataclasses
import math
from enum import Enum
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np


class MessageType(Enum):
    # Firsthand evidence, broadcast by the drone that actually sensed it.
    DETECTION_REPORT = "DETECTION_REPORT"
    # Secondhand evidence, relayed by a drone that already trusts it (one
    # hop beyond the original sensor's own direct radio range) - see
    # DroneConsensusNode.new_relay_evidence. Provenance (origin_drone_id/
    # origin_seq) is preserved unchanged through relay, so quorum counting
    # never credits the relayer as an independent witness.
    CONSENSUS_EVIDENCE = "CONSENSUS_EVIDENCE"
    # Broadcast once a drone's own local quorum check succeeds, so peers
    # converge on the same belief without each independently re-deriving
    # it (and stop treating its evidence as still-pending).
    CONFIRMATION = "CONFIRMATION"
    # Announces that some report_ids have been locally discarded (expired
    # past the consensus window without reaching quorum) - lets peers
    # that hold the same evidence prune it without waiting out their own
    # clock. Optional; a node discarding its own expired evidence never
    # depends on receiving this from anyone else.
    EXPIRY = "EXPIRY"


@dataclasses.dataclass(frozen=True)
class Evidence:
    """One piece of firsthand detection evidence, with full provenance.
    `report_id` (origin_drone_id + origin_seq) is the ONLY identity used
    anywhere for deduplication and quorum counting - never how many times
    a piece of evidence has been relayed, nor who relayed it."""
    origin_drone_id: str
    origin_seq: int
    position_m: Tuple[float, float]
    confidence: float
    observation_timestamp_s: float

    def __post_init__(self):
        if not isinstance(self.origin_drone_id, str) or not self.origin_drone_id:
            raise ValueError("origin_drone_id must be a non-empty str")
        if not isinstance(self.origin_seq, int) or isinstance(self.origin_seq, bool) or self.origin_seq < 0:
            raise ValueError("origin_seq must be a non-negative int")
        if len(self.position_m) != 2 or not all(math.isfinite(v) for v in self.position_m):
            raise ValueError(f"position_m must be a finite (x, y), got {self.position_m!r}")
        if not (0.0 <= self.confidence <= 1.0):
            raise ValueError(f"confidence must be in [0, 1], got {self.confidence!r}")
        if not math.isfinite(self.observation_timestamp_s) or self.observation_timestamp_s < 0.0:
            raise ValueError("observation_timestamp_s must be a finite, non-negative float")

    @property
    def report_id(self) -> str:
        return f"{self.origin_drone_id}:{self.origin_seq}"


@dataclasses.dataclass(frozen=True)
class ConsensusMessage:
    """The only thing that ever crosses a drone-to-drone boundary in this
    module - always via CommsNetwork.send_message/poll_inbox, always
    opaque to the network layer itself."""
    message_type: MessageType
    sender_id: str
    sent_at_s: float
    evidence: Tuple[Evidence, ...] = ()
    confirmation_id: Optional[str] = None
    centroid_m: Optional[Tuple[float, float]] = None
    contributing_drone_ids: Tuple[str, ...] = ()
    contributing_report_ids: Tuple[str, ...] = ()
    expiry_report_ids: Tuple[str, ...] = ()


@dataclasses.dataclass(frozen=True)
class ConfirmationResult:
    confirmation_id: str
    centroid_m: Tuple[float, float]
    contributing_drone_ids: Tuple[str, ...]
    consumed_report_ids: Tuple[str, ...]
    confirmed_by_drone_id: str
    confirmed_at_s: float


def _dist(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _confirmation_id(report_ids: Sequence[str]) -> str:
    """Deterministic identity for a confirmed cluster - independent of
    message arrival order (two drones that end up with the same evidence
    set must compute the exact same id, so peers can recognize each
    other's CONFIRMATION as "the same thing" rather than a duplicate)."""
    return "|".join(sorted(report_ids))


class DroneConsensusNode:
    """One drone's own private consensus state. Required per-drone state
    (Phase 5 requirement 2) all lives here:
      - local detection reports + received peer reports -> self._evidence
      - report timestamps -> Evidence.observation_timestamp_s
      - confidence -> Evidence.confidence
      - source drone ID -> Evidence.origin_drone_id
      - observation age -> derived at read time (now_s - observation_timestamp_s)
      - cluster membership -> cluster_membership()
      - confirmation status -> confirmed_log / is_confirmed()
      - expiry time -> derived from observation_timestamp_s + window_s
      - belief revision history -> belief_revision_history

    Nothing outside this class ever mutates these attributes; the only
    entry points are report_detection/receive_message/try_confirm, and the
    only things they return are immutable dataclasses or counters."""

    def __init__(self, drone_id: str):
        self.drone_id = drone_id
        self._own_seq = 0
        self._evidence: Dict[str, Evidence] = {}
        self._confirmed_ids: set = set()
        self._relayed_ids: set = set()
        self.confirmed_log: List[ConfirmationResult] = []
        self.belief_revision_history: List[dict] = []

        # Bookkeeping for the required per-scenario report metrics
        # (Phase 5 requirement, "for each mode and scenario report").
        self.duplicate_suppressed_count = 0
        self.own_repeat_suppressed_count = 0
        self.stale_message_count = 0
        self.expired_evidence_count = 0
        self.quorum_failure_count = 0

    # -- outgoing ---------------------------------------------------------

    def report_detection(self, position_m, confidence, t: float) -> Evidence:
        """A brand-new local sensor detection. Always gets a fresh,
        strictly-increasing origin_seq - this drone's own repeated reports
        are always distinguishable from each other (different report_id),
        but always share the same origin_drone_id, so quorum clustering
        (which counts distinct origin_drone_id, not distinct report_id)
        can never count them as independent corroborating drones."""
        ev = Evidence(self.drone_id, self._own_seq,
                       (float(position_m[0]), float(position_m[1])), float(confidence), float(t))
        self._own_seq += 1
        self._evidence[ev.report_id] = ev
        self._log(t, "reported", ev.report_id, self.drone_id)
        return ev

    def new_relay_evidence(self) -> Tuple[Evidence, ...]:
        """Evidence received from ANOTHER drone that this node has not yet
        offered for relay - each distinct report_id is relayed at most
        once, ever, per node (bounded fan-out, no relay storms), enabling
        two-hop propagation (A -> B -> C) beyond direct radio range
        without B being credited as an independent witness (provenance is
        unchanged: still A's origin_drone_id)."""
        fresh = tuple(ev for report_id, ev in self._evidence.items()
                       if ev.origin_drone_id != self.drone_id and report_id not in self._relayed_ids)
        self._relayed_ids.update(ev.report_id for ev in fresh)
        return fresh

    # -- incoming -----------------------------------------------------------

    def receive_message(self, msg: ConsensusMessage, now_s: float, window_s: float) -> None:
        if msg.message_type in (MessageType.DETECTION_REPORT, MessageType.CONSENSUS_EVIDENCE):
            for ev in msg.evidence:
                self._ingest_evidence(ev, now_s, window_s, source=msg.sender_id)
        elif msg.message_type == MessageType.CONFIRMATION:
            self._adopt_confirmation(msg, now_s)
        elif msg.message_type == MessageType.EXPIRY:
            for report_id in msg.expiry_report_ids:
                if report_id in self._evidence:
                    del self._evidence[report_id]
                    self._log(now_s, "expiry_received", report_id, msg.sender_id)

    def _ingest_evidence(self, ev: Evidence, now_s: float, window_s: float, source: str) -> None:
        if ev.origin_drone_id == self.drone_id:
            # Reachable via relay, not just defensive: if peer B relays
            # this node's own evidence back (CONSENSUS_EVIDENCE is
            # broadcast to everyone in B's range, since B has no way to
            # know "everyone except the original owner"), and this node is
            # still within B's range, it receives its own evidence bounced
            # back and must discard it here rather than storing or
            # clustering against it (requirement 8).
            self.own_repeat_suppressed_count += 1
            return
        if ev.report_id in self._evidence or ev.report_id in self._confirmed_report_ids():
            self.duplicate_suppressed_count += 1
            return
        if now_s - ev.observation_timestamp_s > window_s:
            self.stale_message_count += 1
            return
        self._evidence[ev.report_id] = ev
        self._log(now_s, "received", ev.report_id, source)

    def _confirmed_report_ids(self) -> set:
        out: set = set()
        for c in self.confirmed_log:
            out.update(c.consumed_report_ids)
        return out

    def _adopt_confirmation(self, msg: ConsensusMessage, now_s: float) -> None:
        cid = msg.confirmation_id
        if cid in self._confirmed_ids:
            return
        self._confirmed_ids.add(cid)
        for report_id in msg.contributing_report_ids:
            self._evidence.pop(report_id, None)
        self.confirmed_log.append(ConfirmationResult(
            confirmation_id=cid, centroid_m=msg.centroid_m,
            contributing_drone_ids=msg.contributing_drone_ids,
            consumed_report_ids=msg.contributing_report_ids,
            confirmed_by_drone_id=msg.sender_id, confirmed_at_s=now_s,
        ))
        self._log(now_s, "adopted_confirmation", cid, msg.sender_id)

    def _log(self, t: float, action: str, ref_id: str, detail: Optional[str]) -> None:
        self.belief_revision_history.append({"t": t, "action": action, "ref_id": ref_id, "detail": detail})

    # -- local consensus attempt ---------------------------------------------

    def is_confirmed(self, confirmation_id: str) -> bool:
        return confirmation_id in self._confirmed_ids

    def pending_evidence_count(self) -> int:
        return len(self._evidence)

    def pending_evidence(self) -> Tuple[Evidence, ...]:
        """Public, read-only disclosure of this node's OWN pending
        evidence - for diagnostics only (see
        DistributedConsensus.pending_evidence_snapshot). A node may always
        answer a query about its own state through its own method; this
        is not another drone reaching into it."""
        return tuple(self._evidence.values())

    def cluster_membership(self, cluster_radius: float) -> List[Tuple[str, ...]]:
        """Read-only introspection: current pending evidence (whatever is
        in `_evidence` right now - call try_confirm first if expired
        evidence should be pruned before grouping) grouped by the same
        spatial-proximity rule try_confirm uses, without consuming or
        mutating anything - for tests/diagnostics."""
        return [tuple(e.report_id for e in cluster)
                for cluster in self._clusters(self._evidence.values(), cluster_radius)]

    def _clusters(self, items: Sequence[Evidence], cluster_radius: float) -> List[List[Evidence]]:
        ordered = sorted(items, key=lambda e: (e.observation_timestamp_s, e.report_id))
        return [
            [e for e in ordered if _dist(e.position_m, seed.position_m) <= cluster_radius]
            for seed in ordered
        ]

    def try_confirm(self, now_s: float, quorum: int, cluster_radius: float,
                     window_s: float) -> Optional[ConfirmationResult]:
        """Local-only quorum check - reads and mutates ONLY this node's own
        `_evidence`/`_confirmed_ids`. Deliberately mirrors centralized
        ConsensusBoard.try_confirm's algorithm exactly (prune anything
        older than `window_s` relative to `now_s`, then group by spatial
        `cluster_radius` alone) - see docs/PHASE5_DISTRIBUTED_CONSENSUS.md's
        "why clustering matches centralized exactly" note: an earlier
        version added an extra pairwise-temporal clustering constraint,
        which is defensible in isolation but occasionally produced a
        different confirmed cluster than centralized under otherwise-
        perfect communication, violating the "must agree under perfect
        communication" acceptance requirement - removed for that reason.
        Temporal separation is still enforced, just via the same
        prune-relative-to-now mechanism centralized uses (see
        test_temporally_separated_reports_expire_before_they_can_cluster).
        Seeds are visited in (timestamp, report_id) order, not arrival/
        insertion order, so the outcome of ties does not depend on message
        delivery order (Phase 5 requirement: deterministic under
        out-of-order messages)."""
        expired = [rid for rid, e in self._evidence.items() if now_s - e.observation_timestamp_s > window_s]
        for report_id in expired:
            del self._evidence[report_id]
            self.expired_evidence_count += 1
            self._log(now_s, "expired", report_id, None)

        seeds = sorted(self._evidence.values(), key=lambda e: (e.observation_timestamp_s, e.report_id))
        for seed in seeds:
            cluster = [e for e in seeds if _dist(e.position_m, seed.position_m) <= cluster_radius]
            distinct_drones = sorted({e.origin_drone_id for e in cluster})
            if len(distinct_drones) < quorum:
                continue
            report_ids = tuple(e.report_id for e in cluster)
            cid = _confirmation_id(report_ids)
            if cid in self._confirmed_ids:
                continue
            # np.mean (not a plain Python sum()/len()) - centralized
            # ConsensusBoard.try_confirm computes its centroid the same
            # way, and the two can otherwise differ in the last bit or two
            # from float summation order alone. That is normally
            # negligible, but this mission's closed feedback loop
            # (a confirmed centroid becomes a beacon position -> changes
            # a drone's trajectory -> changes what it senses next) is
            # chaotically sensitive to exactly that kind of ULP-level
            # difference over a long run - see
            # docs/PHASE5_DISTRIBUTED_CONSENSUS.md's "why perfect-comm
            # agreement is checked at the algorithm level" note.
            centroid_arr = np.mean([e.position_m for e in cluster], axis=0)
            centroid = (float(centroid_arr[0]), float(centroid_arr[1]))
            for report_id in report_ids:
                del self._evidence[report_id]
            self._confirmed_ids.add(cid)
            result = ConfirmationResult(cid, centroid, tuple(distinct_drones), report_ids, self.drone_id, now_s)
            self.confirmed_log.append(result)
            self._log(now_s, "confirmed", cid, None)
            return result

        if self._evidence:
            self.quorum_failure_count += 1
        return None


class DistributedConsensus:
    """Mission-level orchestrator wiring one DroneConsensusNode per drone
    to CommsNetwork message passing. This is the only object that spans
    multiple drones, and even it never reaches into a node's private
    attributes - only calls each node's public methods and passes back
    their return values (Phase 5 requirement 3)."""

    def __init__(self, drone_ids: Sequence[str], quorum: int, cluster_radius: float, window_s: float):
        self._order = tuple(drone_ids)
        self.nodes: Dict[str, DroneConsensusNode] = {did: DroneConsensusNode(did) for did in self._order}
        self.quorum = quorum
        self.cluster_radius = cluster_radius
        self.window_s = window_s
        self._globally_confirmed_ids: set = set()
        self.report_count = 0

    def outgoing_detection_message(self, drone_id: str, candidates, t: float) -> Optional[ConsensusMessage]:
        """Build (and locally record) this tick's DETECTION_REPORT for
        `drone_id`, from its own already-sensed `candidates`
        (VictimSensorModel.DetectionCandidate instances - never ground
        truth). Returns None if there is nothing to report this tick."""
        if not candidates:
            return None
        node = self.nodes[drone_id]
        evidence = tuple(node.report_detection(c.position_m, c.confidence, t) for c in candidates)
        self.report_count += len(evidence)
        return ConsensusMessage(MessageType.DETECTION_REPORT, drone_id, t, evidence=evidence)

    def outgoing_relay_message(self, drone_id: str, t: float) -> Optional[ConsensusMessage]:
        fresh = self.nodes[drone_id].new_relay_evidence()
        if not fresh:
            return None
        return ConsensusMessage(MessageType.CONSENSUS_EVIDENCE, drone_id, t, evidence=fresh)

    def deliver(self, drone_id: str, inbox: Sequence[Tuple[str, ConsensusMessage]], now_s: float) -> None:
        """Feed `drone_id`'s node everything that arrived in its inbox
        this tick (from CommsNetwork.poll_inbox) - the ONLY way evidence
        or confirmations ever reach a node other than its own sensing."""
        node = self.nodes[drone_id]
        for _src, msg in inbox:
            node.receive_message(msg, now_s, self.window_s)

    def try_confirm_all(self, now_s: float) -> List[Tuple[str, ConfirmationResult, ConsensusMessage]]:
        """Runs try_confirm on every node, in a fixed drone order (never
        re-sorted per call), and returns only NEWLY, globally-unique
        confirmations. Several nodes independently reaching the SAME
        swarm-wide realization in the same tick is expected and correct
        under good connectivity (it is exactly what "distributed and
        centralized must agree under perfect communication" requires) -
        it must be scored/announced once, not once per agreeing drone.
        Each entry also carries the CONFIRMATION message that drone should
        broadcast so peers converge onto the same belief.

        Each node drains ALL of its own currently-confirmable clusters
        this call (looping try_confirm until it returns None), not just
        one - mirroring centralized ConsensusBoard's own per-tick
        confirmation loop in mission.py's _resolve_confirmed_detections
        (`while result is not None: ...`). Confirming only one cluster per
        node per tick here while centralized drains every confirmable
        cluster per tick would let extra evidence pile up between ticks
        and change which reports end up in which cluster - a real
        divergence this fix was found to cause under otherwise-perfect
        communication (see docs/PHASE5_DISTRIBUTED_CONSENSUS.md)."""
        out = []
        for drone_id in self._order:
            node = self.nodes[drone_id]
            result = node.try_confirm(now_s, self.quorum, self.cluster_radius, self.window_s)
            while result is not None:
                if result.confirmation_id not in self._globally_confirmed_ids:
                    self._globally_confirmed_ids.add(result.confirmation_id)
                    confirm_msg = ConsensusMessage(
                        MessageType.CONFIRMATION, drone_id, now_s,
                        confirmation_id=result.confirmation_id, centroid_m=result.centroid_m,
                        contributing_drone_ids=result.contributing_drone_ids,
                        contributing_report_ids=result.consumed_report_ids,
                    )
                    out.append((drone_id, result, confirm_msg))
                result = node.try_confirm(now_s, self.quorum, self.cluster_radius, self.window_s)
        return out

    # -- report-only aggregate metrics (Phase 5 required per-scenario report) --

    def pending_evidence_snapshot(self) -> List[dict]:
        """Every currently-pending (unconfirmed) piece of evidence across
        every node, flattened - for diagnostics parity with centralized
        ConsensusBoard.reports (see mission.py's
        _distributed_pending_reports_snapshot). Read-only."""
        out = []
        for node in self.nodes.values():
            for ev in node.pending_evidence():
                out.append({"drone_id": ev.origin_drone_id, "pos": ev.position_m, "t": ev.observation_timestamp_s})
        return out

    def total_duplicate_suppressed_count(self) -> int:
        return sum(n.duplicate_suppressed_count for n in self.nodes.values())

    def total_own_repeat_suppressed_count(self) -> int:
        return sum(n.own_repeat_suppressed_count for n in self.nodes.values())

    def total_stale_message_count(self) -> int:
        return sum(n.stale_message_count for n in self.nodes.values())

    def total_expired_evidence_count(self) -> int:
        return sum(n.expired_evidence_count for n in self.nodes.values())

    def total_quorum_failure_count(self) -> int:
        return sum(n.quorum_failure_count for n in self.nodes.values())

    def confirmed_victim_count(self) -> int:
        return len(self._globally_confirmed_ids)
