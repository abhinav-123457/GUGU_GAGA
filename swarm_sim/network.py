"""Simulated UAV<->UAV communication network.

Replaces ground-truth neighbor lookups (every drone reading every other
drone's exact live position out of a shared array) with a physical radio
model: range-limited, with distance-dependent packet loss and fixed message
latency. A drone's neighbor table is built only from what actually arrived
over its link, not instantaneous global state - the same distinction the
Olfati-Saber flocking literature draws between an agent's local information
graph and the true world state.

Collision-avoidance proximity sensing is deliberately NOT routed through
this network (see SwarmController._avoidance_vector) - a real drone senses
an imminent collision with an onboard sensor (vision/lidar/radar), not a
radio link, and a degraded comms link must never be able to degrade hard
safety.
"""
import numpy as np


class CommsNetwork:
    def __init__(self, config, num_drones, rng):
        self.cfg = config
        self.n = num_drones
        self.rng = rng
        self.step = 0
        # last_seen[dst][src] = {"step": int, "pos": np.array, "vel": np.array, "dist": float}
        self.last_seen = [dict() for _ in range(num_drones)]
        self._inflight = []  # (deliver_step, src, dst, pos, vel, dist)

    def _dropout_prob(self, dist):
        cfg = self.cfg
        r = cfg.communication_radius
        if dist >= r:
            return 1.0
        frac = dist / r
        return cfg.comm_dropout_base + frac * (cfg.comm_dropout_at_max_range - cfg.comm_dropout_base)

    def tick(self, positions, velocities):
        """One round of broadcast: every drone attempts to send its current
        state to every other drone in range. Each link drops independently
        (probability rising with distance); surviving packets are only
        visible to the receiver after comm_latency_steps."""
        cfg = self.cfg
        cur_step = self.step
        deliver_step = cur_step + cfg.comm_latency_steps

        for src in range(self.n):
            for dst in range(self.n):
                if src == dst:
                    continue
                dist = float(np.linalg.norm(positions[src] - positions[dst]))
                if dist > cfg.communication_radius:
                    continue
                if self.rng.random() < self._dropout_prob(dist):
                    continue
                self._inflight.append((deliver_step, src, dst, positions[src].copy(), velocities[src].copy(), dist))

        still_pending = []
        for deliver_step_msg, src, dst, pos, vel, dist in self._inflight:
            if deliver_step_msg <= cur_step:
                self.last_seen[dst][src] = {"step": cur_step, "pos": pos, "vel": vel, "dist": dist}
            else:
                still_pending.append((deliver_step_msg, src, dst, pos, vel, dist))
        self._inflight = still_pending
        self.step += 1

    def neighbor_table(self, i, max_age_steps):
        """Everything drone i currently has a non-stale received sample
        for - its actual, local, possibly-incomplete view of the swarm."""
        cur = self.step
        return {j: e for j, e in self.last_seen[i].items() if cur - e["step"] <= max_age_steps}

    def connected_component_sizes(self, max_age_steps):
        """Undirected connectivity graph built from currently-live links -
        for swarm-fragmentation metrics (one connected group, or has the
        network split into isolated pieces?)."""
        cur = self.step
        adj = {i: set() for i in range(self.n)}
        for i in range(self.n):
            for j, e in self.last_seen[i].items():
                if cur - e["step"] <= max_age_steps:
                    adj[i].add(j)
                    adj[j].add(i)

        seen, sizes = set(), []
        for start in range(self.n):
            if start in seen:
                continue
            stack, comp = [start], set()
            while stack:
                node = stack.pop()
                if node in comp:
                    continue
                comp.add(node)
                stack.extend(adj[node] - comp)
            seen |= comp
            sizes.append(len(comp))
        return sizes
