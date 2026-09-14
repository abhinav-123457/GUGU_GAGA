"""Distributed consensus on candidate victim detections.

Previously, a single drone getting within a pinpoint radius of a victim was
enough to instantly confirm it and broadcast an exact location to the whole
swarm - i.e. one noisy sensor reading, trusted absolutely. That is not a
distributed decision, and it is not how real onboard sensing works: a
camera/IR detection at range is uncertain, and a single false positive
would otherwise commit the swarm's recruitment budget for nothing.

Here, each drone that senses something within its own sensor range reports
a noisy candidate location. A detection is only confirmed - and only then
turned into an actionable RecruitmentBoard beacon - once enough
independent drones (consensus_quorum) have reported mutually-consistent
positions (within consensus_cluster_radius of each other) inside a bounded
time window (consensus_window_sec). This is bookkeeping-level (like
RecruitmentBoard, it lives above any single drone), but the trigger
condition it enforces - independent corroboration, not one agent's word -
is what makes it a real consensus requirement rather than instant
single-drone confirmation. A fully peer-to-peer version, where each drone
carries its own belief state and reports propagate only through
CommsNetwork rather than a shared board, is the natural next step.
"""
import numpy as np


class ConsensusBoard:
    def __init__(self, config):
        self.cfg = config
        self.reports = []  # [{"drone_id": int, "pos": np.array, "t": float}, ...]

    def submit(self, drone_id, pos, t):
        self.reports.append({"drone_id": drone_id, "pos": np.array(pos, dtype=float), "t": t})

    def _prune(self, t):
        window = self.cfg.consensus_window_sec
        self.reports = [r for r in self.reports if t - r["t"] <= window]

    def try_confirm(self, t):
        """Find a cluster of reports from >= consensus_quorum distinct
        drones, all within consensus_cluster_radius of some seed report.
        Consumes the reports it uses and returns (centroid, drone_ids), or
        None if no cluster currently meets quorum."""
        self._prune(t)
        cfg = self.cfg
        for seed in self.reports:
            cluster = [r for r in self.reports
                       if np.linalg.norm(r["pos"] - seed["pos"]) <= cfg.consensus_cluster_radius]
            drones = {r["drone_id"] for r in cluster}
            if len(drones) >= cfg.consensus_quorum:
                centroid = np.mean([r["pos"] for r in cluster], axis=0)
                consumed = {id(r) for r in cluster}
                self.reports = [r for r in self.reports if id(r) not in consumed]
                return centroid, drones
        return None
