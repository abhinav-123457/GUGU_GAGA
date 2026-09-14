"""Per-step swarm decision layer: composes flocking + search + recruitment
into one desired-velocity vector per drone."""
import numpy as np

from .behaviors import boids, vicsek, couzin, levy_flight


class SwarmController:
    def __init__(self, config, num_drones, rng, obstacles=None):
        self.cfg = config
        self.n = num_drones
        self.rng = rng
        self.obstacles = obstacles if obstacles is not None else np.zeros((0, 2))
        headings = rng.normal(size=(num_drones, 3))
        headings[:, 2] = 0.0
        norms = np.linalg.norm(headings, axis=1, keepdims=True)
        norms[norms < 1e-9] = 1.0
        self.headings = headings / norms
        self._levy_dir = [None] * num_drones
        self._levy_remaining = np.zeros(num_drones)
        self.committed_beacon = [None] * num_drones
        self.prev_vel = np.zeros((num_drones, 3))

    def _neighbors(self, positions, i):
        """Ground-truth proximity check for hard-safety collision avoidance
        only (see _avoidance_vector) - modeled as an onboard sensor
        (vision/lidar/radar), not a radio link, so a degraded comms network
        can never be the thing that degrades collision safety."""
        d = np.linalg.norm(positions - positions[i], axis=1)
        d[i] = np.inf
        return np.where(d < self.cfg.sensing_radius)[0]

    def _perceived_state(self, positions, velocities, network, i):
        """Local view for flocking coordination ONLY: index 0 is self
        (ground truth), indices 1..k are neighbors as perceived through the
        comms network - i.e. only those currently linked, using their last
        successfully-received (possibly stale) position/velocity, not
        instantaneous ground truth. A neighbor's heading isn't itself
        transmitted, only its kinematic state is, so heading is inferred
        from the received velocity vector. This is what lets flocking
        degrade realistically under range limits, packet loss, or latency,
        instead of every drone silently knowing every other drone's exact
        live state (Olfati-Saber's local-information-graph distinction)."""
        table = network.neighbor_table(i, self.cfg.comm_max_age_steps)
        linked = [e for e in table.values() if e["dist"] <= self.cfg.sensing_radius]

        local_pos = [positions[i]]
        local_vel = [velocities[i]]
        local_head = [self.headings[i]]
        for e in linked:
            local_pos.append(e["pos"])
            local_vel.append(e["vel"])
            speed = np.linalg.norm(e["vel"])
            local_head.append(e["vel"] / speed if speed > 1e-6 else self.headings[i])

        neighbor_ids = list(range(1, len(local_pos)))
        return np.array(local_pos), np.array(local_vel), np.array(local_head), neighbor_ids

    def _boundary_repulsion(self, pos, margin=5.0, gain=3.0):
        half = self.cfg.arena_size / 2
        force = np.zeros(3)
        for axis in (0, 1):
            if pos[axis] > half - margin:
                force[axis] -= gain * (pos[axis] - (half - margin)) / margin
            elif pos[axis] < -half + margin:
                force[axis] += gain * ((-half + margin) - pos[axis]) / margin
        return force

    def _obstacle_repulsion(self, pos, margin=4.0, gain=2.0):
        force = np.zeros(3)
        activation = self.cfg.obstacle_radius + margin
        for ox, oy in self.obstacles:
            rel = pos[:2] - np.array([ox, oy])
            dist = np.linalg.norm(rel)
            if 1e-6 < dist < activation:
                force[:2] += gain * (rel / dist) * (activation - dist) / activation
        return force

    def _avoidance_vector(self, positions, i, neighbor_ids):
        """Hard-priority escape vector covering both nearby swarm-mates (Couzin's
        zone of repulsion) and physical obstacles' collision surfaces. This always
        overrides normal flocking/search/recruitment, the same way Couzin's zonal
        model gives repulsion absolute priority - extended here to static hazards,
        since a solid obstacle is exactly as non-negotiable as another drone."""
        vec = np.zeros(3)
        active = False

        if len(neighbor_ids) > 0:
            rel = positions[i] - positions[neighbor_ids]
            dists = np.linalg.norm(rel, axis=1)
            close = dists < self.cfg.r_repulsion
            if np.any(close):
                active = True
                vec += (rel[close] / np.clip(dists[close, None], 1e-6, None)).sum(axis=0)

        hard_zone = self.cfg.obstacle_radius + 1.5
        for ox, oy in self.obstacles:
            rel_xy = positions[i, :2] - np.array([ox, oy])
            dist = np.linalg.norm(rel_xy)
            if dist < hard_zone:
                active = True
                if dist > 1e-6:
                    vec[:2] += rel_xy / dist

        if not active:
            return vec, False
        n = np.linalg.norm(vec)
        return (vec / n if n > 1e-9 else np.array([1.0, 0.0, 0.0])), True

    def step(self, positions, velocities, board, network):
        cfg = self.cfg
        desired_vels = np.zeros_like(positions)
        repulsion_flags = np.zeros(self.n, dtype=bool)

        for i in range(self.n):
            beacon = None
            vid = self.committed_beacon[i]
            if vid is not None:
                beacon = board.beacons.get(vid)
                if beacon is None or beacon["strength"] <= 0.0:
                    self.committed_beacon[i] = None
                    beacon = None

            if beacon is None:
                candidate = board.nearest_available_beacon(positions[i], cfg.communication_radius)
                if candidate is not None and board.try_recruit(candidate["id"], i, cfg.max_recruits_per_beacon):
                    self.committed_beacon[i] = candidate["id"]
                    beacon = candidate

            obstacle_force = self._obstacle_repulsion(positions[i])
            neighbor_ids = self._neighbors(positions, i)
            avoidance_vec, avoidance_active = self._avoidance_vector(positions, i, neighbor_ids)
            repulsion_flags[i] = avoidance_active

            if avoidance_active:
                # Hard safety layer: overrides recruitment and search alike - a beacon
                # or a search leg is never worth a collision. Slow down rather than
                # swerve at full speed: keeps velocity-tracking error (and the tilt
                # angle DSLPIDControl derives from it) small enough to stay flyable.
                self.headings[i] = avoidance_vec
                desired_vels[i] = avoidance_vec * cfg.repulsion_speed_mps
                desired_vels[i, 2] = 0.0
                continue

            if beacon is not None:
                to_beacon = beacon["pos"] - positions[i]
                dist = np.linalg.norm(to_beacon)
                direction = to_beacon / dist if dist > 1e-6 else self.headings[i]
                self.headings[i] = direction
                desired_vels[i] = direction * min(cfg.cruise_speed_mps, max(dist, 0.1)) + obstacle_force
                desired_vels[i, 2] = 0.0  # altitude held separately by the position controller
                continue

            local_pos, local_vel, local_head, perceived_ids = self._perceived_state(positions, velocities, network, i)

            if cfg.flock_model == "boids":
                flock_vec = boids.boids_steer(local_pos, local_vel, 0, perceived_ids, cfg.r_attraction)
            elif cfg.flock_model == "vicsek":
                flock_vec = vicsek.vicsek_heading(local_head, 0, perceived_ids, cfg.vicsek_noise, self.rng)
            else:
                flock_vec = couzin.couzin_direction(
                    local_pos, local_head, 0, perceived_ids,
                    cfg.r_repulsion, cfg.r_orientation, cfg.r_attraction, cfg.fov_deg,
                )

            direction, remaining = levy_flight.levy_flight_leg(
                self.rng, cfg.levy_alpha, self._levy_dir[i], self._levy_remaining[i],
                dt=1.0 / cfg.control_freq_hz, speed_mps=cfg.cruise_speed_mps,
            )
            self._levy_dir[i] = direction
            self._levy_remaining[i] = remaining

            combined = flock_vec + cfg.levy_weight * direction + self._boundary_repulsion(positions[i]) + obstacle_force
            norm = np.linalg.norm(combined)
            heading = combined / norm if norm > 1e-6 else self.headings[i]
            self.headings[i] = heading
            desired_vels[i] = heading * cfg.cruise_speed_mps
            desired_vels[i, 2] = 0.0  # altitude held separately by the position controller

        # Rate-limit commanded velocity changes: a real airframe can't track an
        # instantaneous full-speed reversal. Active collision avoidance gets a much
        # higher allowance than routine cruising - the same distinction a real flight
        # controller draws between nominal path-following and emergency maneuvering.
        dt = 1.0 / cfg.control_freq_hz
        accel_caps = np.where(repulsion_flags, cfg.emergency_accel_mps2, cfg.max_accel_mps2)
        max_delta = accel_caps[:, None] * dt
        delta = desired_vels - self.prev_vel
        norms = np.linalg.norm(delta, axis=1, keepdims=True)
        scale = np.minimum(1.0, max_delta / np.clip(norms, 1e-9, None))
        desired_vels = self.prev_vel + delta * scale
        self.prev_vel = desired_vels.copy()

        return desired_vels
