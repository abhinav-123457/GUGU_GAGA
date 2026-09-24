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
    def __init__(self, config, num_drones, rng, message_rng=None):
        self.cfg = config
        self.n = num_drones
        self.rng = rng
        # Phase 5: generic typed-message channel (distributed consensus -
        # see swarm_sim/distributed_consensus.py) draws its own dropout
        # decisions from a SEPARATE stream than the one used for the
        # position/velocity broadcast above. Reusing `rng` for both would
        # mean turning distributed consensus on/off changes how many draws
        # `rng` makes per tick, silently perturbing the position-broadcast
        # dropout sequence (and therefore flocking's neighbor tables) -
        # exactly the kind of cross-subsystem RNG leakage
        # swarm_sim/seeding.py was written to prevent. Defaults to `rng`
        # itself so every existing call site (tests, scripts) that never
        # calls send_message is unaffected.
        self.message_rng = message_rng if message_rng is not None else rng
        self.step = 0
        # last_seen[dst][src] = {"step": int, "pos": np.array, "vel": np.array, "dist": float}
        self.last_seen = [dict() for _ in range(num_drones)]
        self._inflight = []  # (deliver_step, src, dst, pos, vel, dist)
        # Phase 5 generic message queue - see send_message/pump_messages/poll_inbox.
        self._message_inflight = []  # (deliver_step, src, dst, message)
        self._message_inbox = [[] for _ in range(num_drones)]
        # Phase 16B: the radio's own link geometry (real inter-drone distances) as of the last tick(),
        # kept so send_message() can gate delivery physically without callers handing it positions.
        self._link_positions = None

    def _dropout_prob(self, dist):
        cfg = self.cfg
        r = cfg.communication_radius
        if dist >= r:
            return 1.0
        frac = dist / r
        return cfg.comm_dropout_base + frac * (cfg.comm_dropout_at_max_range - cfg.comm_dropout_base)

    def tick(self, positions, velocities, payload_positions=None, payload_velocities=None):
        """One round of broadcast: every drone attempts to send its current
        state to every other drone in range. Each link drops independently
        (probability rising with distance); surviving packets are only
        visible to the receiver after comm_latency_steps.

        `positions` / `velocities` are the physical (plant) state: they decide
        WHICH links exist and how lossy they are, exactly as radio propagation
        does. `payload_positions` / `payload_velocities` (Phase 16B) are what
        each drone actually transmits about itself - its own estimate. They
        default to `positions` / `velocities` (the legacy perfect-state
        behaviour). The stored per-link `dist` is the physical link range."""
        cfg = self.cfg
        cur_step = self.step
        deliver_step = cur_step + cfg.comm_latency_steps
        self._link_positions = np.array(positions, dtype=float)
        payload_pos = positions if payload_positions is None else payload_positions
        payload_vel = velocities if payload_velocities is None else payload_velocities

        for src in range(self.n):
            for dst in range(self.n):
                if src == dst:
                    continue
                dist = float(np.linalg.norm(positions[src] - positions[dst]))
                if dist > cfg.communication_radius:
                    continue
                if self.rng.random() < self._dropout_prob(dist):
                    continue
                self._inflight.append((deliver_step, src, dst, payload_pos[src].copy(), payload_vel[src].copy(), dist))

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

    # -- Phase 5: generic typed-message channel --------------------------------
    # Distributed consensus (swarm_sim/distributed_consensus.py) needs to move
    # arbitrary discrete messages (detection reports, confirmations), not just
    # "latest known state" like the position/velocity broadcast above. Same
    # physical model - independent per-link range check + distance-dependent
    # packet loss + fixed latency - applied to an opaque payload instead.

    def send_message(self, src, message, positions=None):
        """Queue `message` (any opaque, picklable-ish object - never
        inspected here) for delivery from drone `src` to every other drone
        currently within communication_radius, each independently subject
        to the same range-dependent dropout model tick() uses. `positions`
        is the tick's physical position array, used only for the distance
        check; when omitted (Phase 16B) the link geometry cached by the most
        recent tick() is used, so callers never have to hand this module
        ground truth."""
        cfg = self.cfg
        if positions is None:
            if self._link_positions is None:
                raise ValueError("send_message() needs positions, or a prior tick() to take the link geometry from")
            positions = self._link_positions
        deliver_step = self.step + cfg.comm_latency_steps
        for dst in range(self.n):
            if dst == src:
                continue
            dist = float(np.linalg.norm(positions[src] - positions[dst]))
            if dist > cfg.communication_radius:
                continue
            if self.message_rng.random() < self._dropout_prob(dist):
                continue
            self._message_inflight.append((deliver_step, src, dst, message))

    def pump_messages(self):
        """Move everything whose latency has elapsed into each
        destination's inbox. Call exactly once per control tick, after
        every drone's send_message() calls for that tick (mirrors how
        tick() resolves its own _inflight queue)."""
        cur_step = self.step
        still_pending = []
        for deliver_step_msg, src, dst, message in self._message_inflight:
            if deliver_step_msg <= cur_step:
                self._message_inbox[dst].append((src, message))
            else:
                still_pending.append((deliver_step_msg, src, dst, message))
        self._message_inflight = still_pending

    def poll_inbox(self, dst):
        """Drain and return dst's inbox: a list of (src, message) tuples
        delivered since the last poll. Each message is handed back
        exactly once, to exactly the drone it was addressed to."""
        inbox = self._message_inbox[dst]
        self._message_inbox[dst] = []
        return inbox

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

    def component_size_per_drone(self, max_age_steps):
        """Same connectivity graph as connected_component_sizes, but
        returned as one size per drone index (size of THAT drone's own
        component) rather than one entry per component - added for Phase
        4's safety supervisor, which needs to know whether a specific
        vehicle is currently isolated (its own component size == 1), not
        just the overall size distribution. Pure read of the same
        already-comms-realistic last_seen data; changes nothing about
        connected_component_sizes' own behavior."""
        cur = self.step
        adj = {i: set() for i in range(self.n)}
        for i in range(self.n):
            for j, e in self.last_seen[i].items():
                if cur - e["step"] <= max_age_steps:
                    adj[i].add(j)
                    adj[j].add(i)

        sizes = [0] * self.n
        seen = set()
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
            for node in comp:
                sizes[node] = len(comp)
        return sizes
