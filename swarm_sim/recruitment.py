"""Recruitment signaling for confirmed detections.

Modeled on two classic mechanisms cited together because they solve the same
problem in nature: telling nearby group members where something important is.

- von Frisch, K. (1967) - The Dance Language and Orientation of Bees: a
  forager that finds something valuable communicates its location directly
  to nearby hive-mates, recruiting a bounded number of helpers.
- Bonabeau, E. et al. (1996) - self-organization in social insects: the
  recruitment signal is a decaying, shared "stigmergic" trace, not a
  permanent broadcast - once serviced it fades and stops attracting agents.
"""
import numpy as np


class RecruitmentBoard:
    def __init__(self, decay_rate=0.02):
        self.decay_rate = decay_rate
        self.beacons = {}  # victim_id -> beacon dict

    def announce(self, victim_id, pos, drone_id, t):
        if victim_id in self.beacons:
            return
        self.beacons[victim_id] = {
            "id": victim_id,
            "pos": np.array(pos, dtype=float),
            "strength": 1.0,
            "found_by": drone_id,
            "t_found": t,
            "recruits": set(),
        }

    def try_recruit(self, victim_id, drone_id, max_recruits):
        b = self.beacons.get(victim_id)
        if b is None or b["strength"] <= 0.0:
            return False
        if drone_id in b["recruits"]:
            return True
        if len(b["recruits"]) >= max_recruits:
            return False
        b["recruits"].add(drone_id)
        return True

    def nearest_available_beacon(self, pos, max_range):
        best, best_d = None, max_range
        for b in self.beacons.values():
            if b["strength"] <= 0.0:
                continue
            d = np.linalg.norm(b["pos"] - pos)
            if d <= best_d:
                best, best_d = b, d
        return best

    def service_check(self, positions, arrival_radius=1.5):
        """Once all recruits assigned to a beacon have physically arrived,
        mark it serviced so the swarm redirects back to open search."""
        for b in self.beacons.values():
            if b["strength"] <= 0.0 or not b["recruits"]:
                continue
            arrived = all(np.linalg.norm(positions[d] - b["pos"]) < arrival_radius for d in b["recruits"])
            if arrived:
                b["strength"] = 0.0

    def decay(self, dt):
        for b in self.beacons.values():
            if b["strength"] > 0.0:
                b["strength"] = max(0.0, b["strength"] - self.decay_rate * dt)
