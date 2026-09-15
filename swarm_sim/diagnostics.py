"""Phase 2 diagnostic instrumentation.

Read-only observation of the existing sensing/consensus/contact pipeline,
added strictly for reporting. This module changes no sensor behavior, no
consensus algorithm, and no control decision - it only records numbers
that were already being computed (or trivially derivable from already-
public state, like ConsensusBoard.reports) and organizes them into a
report. This is NOT the Phase 4 safety supervisor: nothing here is read
back by any controller or consensus decision, and none of it should be
mistaken for one. See docs/PHASE2_DIAGNOSTICS.md.
"""
from __future__ import annotations

import dataclasses
import math
from typing import Dict, List, Optional, Tuple


@dataclasses.dataclass
class VictimDiagnostic:
    victim_id: int
    ground_truth_position: Tuple[float, float]
    times_in_sensor_range: int = 0
    valid_sensor_observations: int = 0
    detection_candidates: int = 0
    reports_submitted: int = 0
    reports_received: int = 0
    reports_in_window_at_last_check: int = 0
    quorum_ever_reached: bool = False
    confirmation_position: Optional[Tuple[float, float]] = None
    confirmation_timestamp: Optional[float] = None
    match_result: str = "unresolved"
    reason: str = ""
    localization_errors_m: List[float] = dataclasses.field(default_factory=list)

    def to_dict(self) -> dict:
        d = dataclasses.asdict(self)
        d["mean_localization_error_m"] = sum(self.localization_errors_m) / len(self.localization_errors_m) \
            if self.localization_errors_m else None
        return d


@dataclasses.dataclass
class ContactEvent:
    """One PyBullet-verified physical contact involving at least one
    drone, classified and correlated with each involved drone's own
    sensing state at that same tick. Evaluation-only: nothing here fed
    back into any decision, and nothing here is a safety supervisor -
    see the explicit limitation this class exists to document."""
    t: float
    category: str  # "drone_drone" | "drone_obstacle" | "drone_ground"
    body_a: int
    body_b: int
    drone_ids_involved: Tuple[int, ...]
    estimated_clearance_m: Optional[float]
    actual_clearance_m: Optional[float]
    sensor_age_s: Optional[float]
    command_age_s: Optional[float]
    observation_dropout_or_stale: bool
    clearance_trend: str  # "closing" | "opening" | "unknown" - before vs after this tick

    def to_dict(self) -> dict:
        return dataclasses.asdict(self)


class MissionDiagnostics:
    """One instance per FloodSearchMission run. mission.py calls the
    record_* hooks at points it already computes this data (or has
    trivial read access to, like len(ConsensusBoard.reports)); nothing
    here changes what mission.py, controller.py, sensors.py, or
    consensus.py actually do."""

    def __init__(self, victims_xy, consensus_quorum: int, consensus_cluster_radius: float):
        self.consensus_quorum = consensus_quorum
        self.consensus_cluster_radius = consensus_cluster_radius
        self.victims: Dict[int, VictimDiagnostic] = {
            vid: VictimDiagnostic(victim_id=vid, ground_truth_position=(float(xy[0]), float(xy[1])))
            for vid, xy in enumerate(victims_xy)
        }
        self.true_positive_events = 0
        self.false_positive_events = 0
        self.false_confirmation_events = 0
        self.confirmed_victim_count = 0
        self.peak_consensus_report_count = 0
        self.sensing_cpu_time_s = 0.0
        self.consensus_cpu_time_s = 0.0
        # Approximate, not exact - see docs/PHASE2_DIAGNOSTICS.md's
        # "expired candidate count" note for why this undercounts whenever
        # one drone contributed more than one report to the same cluster.
        self.reports_consumed_by_confirmations = 0
        self.contact_log: List[ContactEvent] = []
        self._prev_actual_clearance: Dict[Tuple[int, int], float] = {}

    # -- victim detection ---------------------------------------------

    def record_range_check(self, vid: int, in_range: bool) -> None:
        if in_range:
            self.victims[vid].times_in_sensor_range += 1

    def record_detection_event(self, matched_vid: Optional[int], localization_error: Optional[float]) -> None:
        if matched_vid is not None:
            v = self.victims[matched_vid]
            v.valid_sensor_observations += 1
            v.detection_candidates += 1
            v.localization_errors_m.append(localization_error)
            self.true_positive_events += 1
        else:
            self.false_positive_events += 1

    def record_submission(self, matched_vid: Optional[int]) -> None:
        if matched_vid is not None:
            v = self.victims[matched_vid]
            v.reports_submitted += 1
            v.reports_received += 1  # ConsensusBoard is centralized/instant - see docs

    def record_consensus_report_snapshot(self, reports) -> None:
        """`reports` is ConsensusBoard.reports (its own public list - not
        modified, just read) at this point in time."""
        self.peak_consensus_report_count = max(self.peak_consensus_report_count, len(reports))
        for v in self.victims.values():
            if v.match_result == "confirmed":
                continue
            gx, gy = v.ground_truth_position
            nearby_drones = set()
            count = 0
            for r in reports:
                dx = r["pos"][0] - gx
                dy = r["pos"][1] - gy
                if math.hypot(dx, dy) <= self.consensus_cluster_radius:
                    count += 1
                    nearby_drones.add(r["drone_id"])
            v.reports_in_window_at_last_check = count
            if len(nearby_drones) >= self.consensus_quorum:
                v.quorum_ever_reached = True

    def record_confirmation(self, vid: int, centroid, t: float, num_contributing_drones: int) -> None:
        self.confirmed_victim_count += 1
        self.reports_consumed_by_confirmations += num_contributing_drones
        v = self.victims[vid]
        v.confirmation_position = (float(centroid[0]), float(centroid[1]))
        v.confirmation_timestamp = float(t)
        v.match_result = "confirmed"
        v.reason = "confirmed by consensus quorum, matched to nearest ground-truth victim"

    def record_false_confirmation(self, num_contributing_drones: int) -> None:
        self.false_confirmation_events += 1
        self.reports_consumed_by_confirmations += num_contributing_drones

    def finalize(self) -> None:
        for v in self.victims.values():
            if v.match_result == "confirmed":
                continue
            v.match_result = "missed"
            if v.times_in_sensor_range == 0:
                v.reason = "never within victim_sensor_range_m of any drone"
            elif v.valid_sensor_observations == 0:
                v.reason = ("in range at least once, but every attempt was gated out "
                            "(FOV/occlusion) or lost to a false-negative roll")
            elif v.reports_submitted < self.consensus_quorum:
                v.reason = (f"only {v.reports_submitted} independent report(s) ever submitted; "
                            f"consensus_quorum is {self.consensus_quorum}")
            elif not v.quorum_ever_reached:
                v.reason = ("reports were submitted but never had >= consensus_quorum distinct "
                            "drones clustered within consensus_cluster_radius at the same time")
            else:
                v.reason = ("quorum was reached at some point, but consensus_window_sec expired "
                             "before try_confirm processed that cluster (reports pruned as stale) "
                             "- run ended before the mechanism caught up")

    # -- clearance / contacts ------------------------------------------

    def classify_contact(self, t, body_a, body_b, drone_id_set, obstacle_body_ids,
                          estimated_clearance_m, actual_clearance_m, sensor_age_s, command_age_s,
                          observation_dropout_or_stale, drone_ids_involved, pair_key) -> ContactEvent:
        if body_a in drone_id_set and body_b in drone_id_set:
            category = "drone_drone"
        elif (body_a in drone_id_set and body_b in obstacle_body_ids) or \
             (body_b in drone_id_set and body_a in obstacle_body_ids):
            category = "drone_obstacle"
        else:
            category = "drone_ground"

        prev_actual = self._prev_actual_clearance.get(pair_key)
        if prev_actual is None or actual_clearance_m is None:
            trend = "unknown"
        elif actual_clearance_m < prev_actual:
            trend = "closing"
        elif actual_clearance_m > prev_actual:
            trend = "opening"
        else:
            trend = "unknown"
        if actual_clearance_m is not None:
            self._prev_actual_clearance[pair_key] = actual_clearance_m

        event = ContactEvent(
            t=t, category=category, body_a=body_a, body_b=body_b,
            drone_ids_involved=tuple(drone_ids_involved),
            estimated_clearance_m=estimated_clearance_m, actual_clearance_m=actual_clearance_m,
            sensor_age_s=sensor_age_s, command_age_s=command_age_s,
            observation_dropout_or_stale=observation_dropout_or_stale, clearance_trend=trend,
        )
        self.contact_log.append(event)
        return event

    # -- report assembly ------------------------------------------------

    def to_report_dict(self) -> dict:
        return {
            "victims": {vid: v.to_dict() for vid, v in self.victims.items()},
            "true_positive_events": self.true_positive_events,
            "false_positive_events": self.false_positive_events,
            "false_confirmation_events": self.false_confirmation_events,
            "unique_victims_detected": sum(1 for v in self.victims.values() if v.detection_candidates > 0),
            "confirmed_victims": self.confirmed_victim_count,
            "missed_victims": sum(1 for v in self.victims.values() if v.match_result == "missed"),
            "peak_consensus_report_count": self.peak_consensus_report_count,
            "reports_consumed_by_confirmations_approx": self.reports_consumed_by_confirmations,
            "sensing_cpu_time_s": self.sensing_cpu_time_s,
            "consensus_cpu_time_s": self.consensus_cpu_time_s,
            "contact_log": [c.to_dict() for c in self.contact_log],
        }
