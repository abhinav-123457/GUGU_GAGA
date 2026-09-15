"""Per-step swarm decision layer: composes flocking + search + recruitment
into one desired-velocity vector for ONE drone at a time.

Phase 2: this module never receives ground truth. `step()` takes this
drone's own state (its own telemetry - legitimate, a drone knows its own
position/velocity) plus whatever its sensors actually produced this tick
(swarm_sim/sensors.py's SensorObservation/NeighborObservation - noisy,
range-limited, latent, dropout-prone) and RecruitmentBoard's confirmed
beacon beliefs (themselves once-removed from truth via noisy sensing +
consensus, never ground truth either). It is never handed the hidden
victim list, hidden obstacle geometry, or the mission's raw global
drone-position array. See docs/PHASE2_SENSING.md.
"""
import math

import numpy as np

from . import sensors
from .behaviors import boids, vicsek, couzin, levy_flight, olfati_saber


class SwarmController:
    def __init__(self, config, num_drones, rng):
        self.cfg = config
        self.n = num_drones
        self.rng = rng
        self._obstacle_bin_angles = sensors.obstacle_scan_bin_angles(config)
        headings = rng.normal(size=(num_drones, 3))
        headings[:, 2] = 0.0
        norms = np.linalg.norm(headings, axis=1, keepdims=True)
        norms[norms < 1e-9] = 1.0
        self.headings = headings / norms
        self._levy_dir = [None] * num_drones
        self._levy_remaining = np.zeros(num_drones)
        self.committed_beacon = [None] * num_drones
        self.prev_vel = np.zeros((num_drones, 3))

    def _perceived_state(self, own_state, network, i):
        """Local view for flocking coordination: index 0 is self (this
        drone's own telemetry), indices 1..k are neighbors as perceived
        through the comms network - only those currently linked, using
        their last successfully-received (possibly stale) position/
        velocity, not instantaneous ground truth. A neighbor's heading
        isn't itself transmitted, only its kinematic state is, so heading
        is inferred from the received velocity vector. This lets flocking
        degrade realistically under range limits, packet loss, or latency
        (Olfati-Saber's local-information-graph distinction). Unchanged
        from before Phase 2 - CommsNetwork was already comms-realistic,
        not a ground-truth leak; only its caller's inputs changed."""
        table = network.neighbor_table(i, self.cfg.comm_max_age_steps)
        linked = [e for e in table.values() if e["dist"] <= self.cfg.sensing_radius]

        own_pos = np.array(own_state.position_m)
        own_vel = np.array(own_state.velocity_mps)
        local_pos = [own_pos]
        local_vel = [own_vel]
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

    def _obstacle_repulsion_from_scan(self, heading_xy, range_returns_m, margin=4.0, gain=2.0):
        """Soft repulsion driven by the sensed range scan (edge-distance
        per angular bin), not ground-truth obstacle centers. Replaces the
        pre-Phase-2 version that read self.obstacles directly."""
        force = np.zeros(3)
        base_angle = sensors.facing_angle(heading_xy)
        for offset, r in zip(self._obstacle_bin_angles, range_returns_m):
            if math.isfinite(r) and r < margin:
                ray_dir = np.array([math.cos(base_angle + offset), math.sin(base_angle + offset)])
                force[:2] -= ray_dir * gain * (margin - r) / margin
        return force

    def _avoidance_vector(self, own_pos, heading_xy, neighbor_observations, range_returns_m):
        """Hard-priority escape vector covering both nearby swarm-mates
        and physical obstacles' collision surfaces, always overriding
        normal flocking/search/recruitment - Couzin's zonal model gives
        repulsion absolute priority, extended here to static hazards.
        Driven entirely by sensed data now: `neighbor_observations` (this
        drone's onboard proximity sensor - swarm_sim.sensors.
        NeighborSensorModel, never ground truth) and `range_returns_m`
        (the obstacle range scan), never the ground-truth position array
        or obstacle centers the pre-Phase-2 version read directly."""
        vec = np.zeros(3)
        active = False

        if neighbor_observations:
            rels = np.array([own_pos - np.array(n.measured_position_m) for n in neighbor_observations])
            dists = np.linalg.norm(rels, axis=1)
            close = dists < self.cfg.r_repulsion
            if np.any(close):
                active = True
                vec += (rels[close] / np.clip(dists[close, None], 1e-6, None)).sum(axis=0)

        hard_zone = self.cfg.obstacle_radius + 1.5
        base_angle = sensors.facing_angle(heading_xy)
        for offset, r in zip(self._obstacle_bin_angles, range_returns_m):
            if math.isfinite(r) and r < hard_zone:
                active = True
                ray_dir = np.array([math.cos(base_angle + offset), math.sin(base_angle + offset)])
                vec[:2] -= ray_dir

        if not active:
            return vec, False
        n = np.linalg.norm(vec)
        return (vec / n if n > 1e-9 else np.array([1.0, 0.0, 0.0])), True

    def step(self, i, own_state, sensor_observation, neighbor_observations, mission_beliefs, network):
        """Returns this ONE drone's desired 3-vector velocity.

        own_state: this drone's own VehicleState (its own telemetry).
        sensor_observation: this drone's SensorObservation this tick -
            `.detections` (victim candidates, informational only -
            mission.py feeds ConsensusBoard directly, the controller
            doesn't act on detections itself) and `.range_returns_m`
            (obstacle range scan, consumed below).
        neighbor_observations: this drone's onboard proximity sensor
            readings this tick (tuple of NeighborObservation, possibly
            empty) - used only by the hard-safety avoidance branch.
        mission_beliefs: RecruitmentBoard - confirmed beacon beliefs
            (post-consensus, never ground truth).
        network: CommsNetwork - the already comms-realistic channel
            flocking cohesion perceives other drones through; kept
            separate from `neighbor_observations` on purpose (see
            _avoidance_vector's docstring and docs/PHASE2_SENSING.md) so
            a degraded radio link can never be what degrades collision
            safety.
        """
        cfg = self.cfg
        own_pos = np.array(own_state.position_m)
        heading_xy = self.headings[i][:2]

        beacon = None
        vid = self.committed_beacon[i]
        if vid is not None:
            beacon = mission_beliefs.beacons.get(vid)
            if beacon is None or beacon["strength"] <= 0.0:
                self.committed_beacon[i] = None
                beacon = None

        if beacon is None:
            candidate = mission_beliefs.nearest_available_beacon(own_pos, cfg.communication_radius)
            if candidate is not None and mission_beliefs.try_recruit(candidate["id"], i, cfg.max_recruits_per_beacon):
                self.committed_beacon[i] = candidate["id"]
                beacon = candidate

        range_returns_m = sensor_observation.range_returns_m
        obstacle_force = self._obstacle_repulsion_from_scan(heading_xy, range_returns_m)
        avoidance_vec, avoidance_active = self._avoidance_vector(
            own_pos, heading_xy, neighbor_observations, range_returns_m,
        )

        if avoidance_active:
            # Hard safety layer: overrides recruitment and search alike - a beacon
            # or a search leg is never worth a collision. Slow down rather than
            # swerve at full speed: keeps velocity-tracking error (and the tilt
            # angle DSLPIDControl derives from it) small enough to stay flyable.
            self.headings[i] = avoidance_vec
            desired_vel = avoidance_vec * cfg.repulsion_speed_mps
            desired_vel[2] = 0.0
        elif beacon is not None:
            to_beacon = np.array(beacon["pos"]) - own_pos
            dist = np.linalg.norm(to_beacon)
            direction = to_beacon / dist if dist > 1e-6 else self.headings[i]
            self.headings[i] = direction
            desired_vel = direction * min(cfg.cruise_speed_mps, max(dist, 0.1)) + obstacle_force
            desired_vel[2] = 0.0  # altitude held separately by the position controller
        else:
            local_pos, local_vel, local_head, perceived_ids = self._perceived_state(own_state, network, i)

            if cfg.flock_model == "boids":
                flock_vec = boids.boids_steer(local_pos, local_vel, 0, perceived_ids, cfg.r_attraction)
            elif cfg.flock_model == "vicsek":
                flock_vec = vicsek.vicsek_heading(local_head, 0, perceived_ids, cfg.vicsek_noise, self.rng)
            elif cfg.flock_model == "olfati_saber":
                # Engineering approximation (see behaviors/olfati_saber.py):
                # no dedicated virtual-leader agent exists in this
                # simulator, so the navigation term's target is just this
                # drone's own position (no positional pull) with a target
                # velocity along the current search heading - the term
                # degrades to "match the search direction" rather than
                # "track a leader's trajectory".
                neighbor_pos = local_pos[1:]
                neighbor_vel = local_vel[1:]
                flock_vec = olfati_saber.olfati_saber_steer(
                    local_pos[0], local_vel[0], neighbor_pos, neighbor_vel,
                    target_pos=local_pos[0], target_vel=self.headings[i] * cfg.cruise_speed_mps,
                    d_alpha=cfg.os_desired_spacing_m, r_alpha=cfg.os_interaction_range_m,
                    c_spacing=cfg.os_c_spacing, c_align=cfg.os_c_align, c_nav=cfg.os_c_nav,
                )
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

            combined = flock_vec + cfg.levy_weight * direction + self._boundary_repulsion(own_pos) + obstacle_force
            norm = np.linalg.norm(combined)
            heading = combined / norm if norm > 1e-6 else self.headings[i]
            self.headings[i] = heading
            desired_vel = heading * cfg.cruise_speed_mps
            desired_vel[2] = 0.0  # altitude held separately by the position controller

        # Rate-limit commanded velocity changes: a real airframe can't track an
        # instantaneous full-speed reversal. Active collision avoidance gets a much
        # higher allowance than routine cruising - the same distinction a real flight
        # controller draws between nominal path-following and emergency maneuvering.
        dt = 1.0 / cfg.control_freq_hz
        accel_cap = cfg.emergency_accel_mps2 if avoidance_active else cfg.max_accel_mps2
        max_delta = accel_cap * dt
        delta = desired_vel - self.prev_vel[i]
        norm = np.linalg.norm(delta)
        scale = min(1.0, max_delta / max(norm, 1e-9))
        desired_vel = self.prev_vel[i] + delta * scale
        self.prev_vel[i] = desired_vel.copy()

        return desired_vel
