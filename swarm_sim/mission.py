"""Flood search-and-rescue mission: open arena, sparse obstacles, a swarm
searching for scattered victims using bio-inspired flocking + Levy-flight
search + von Frisch/Bonabeau-style recruitment, with full telemetry and
per-drone speed control.

Drives DSLPIDControl directly (rather than VelocityAviary's action
shortcut) so target velocities are tracked in real m/s with no hidden
scaling ceiling - the per-drone speed cap in MissionConfig/SpeedController
is the actual physical limit that gets applied.
"""
import time as timemod

import numpy as np
import pybullet as p

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.utils.utils import sync

from .consensus import ConsensusBoard
from .controller import SwarmController
from .network import CommsNetwork
from .recruitment import RecruitmentBoard
from .speed_control import SpeedController
from .telemetry import TelemetryHub

# One distinct, high-contrast color per drone (cycles if num_drones > len(this)),
# so an individual drone can be told apart at a glance instead of reading as
# identical tiny gizmos.
DRONE_COLORS = [
    [0.95, 0.35, 0.10, 1], [0.15, 0.65, 0.95, 1], [0.90, 0.80, 0.15, 1],
    [0.60, 0.30, 0.90, 1], [0.20, 0.80, 0.45, 1], [0.95, 0.50, 0.75, 1],
    [0.40, 0.75, 0.95, 1], [0.85, 0.45, 0.20, 1],
]


class FloodSearchMission:
    def __init__(self, config):
        self.cfg = config
        self.rng = np.random.default_rng(config.seed)
        self.telemetry = TelemetryHub()
        self.board = RecruitmentBoard(decay_rate=config.beacon_decay_rate)
        self.network = CommsNetwork(config, config.num_drones, self.rng)
        self.consensus = ConsensusBoard(config)
        self.false_confirmations = 0
        self._connected_steps = 0
        self._connectivity_checks = 0

        half = config.arena_size / 2
        init_xyz = np.zeros((config.num_drones, 3))
        init_xyz[:, 0] = self.rng.uniform(-half * 0.3, half * 0.3, config.num_drones)
        init_xyz[:, 1] = self.rng.uniform(-half * 0.3, half * 0.3, config.num_drones)
        init_xyz[:, 2] = config.flight_altitude

        self.env = CtrlAviary(
            drone_model=DroneModel.CF2X,
            num_drones=config.num_drones,
            initial_xyzs=init_xyz,
            physics=Physics.PYB,
            neighbourhood_radius=config.sensing_radius,
            pyb_freq=config.sim_freq_hz,
            ctrl_freq=config.control_freq_hz,
            gui=config.gui,
        )
        self.pid = [DSLPIDControl(drone_model=DroneModel.CF2X) for _ in range(config.num_drones)]

        airframe_max_speed = self.env.MAX_SPEED_KMH * 1000.0 / 3600.0
        self.speed_ctrl = SpeedController(config.num_drones, config.max_speed_mps, airframe_max_speed)

        self.obstacles = self._sample_obstacle_positions(half, init_xyz[:, :2])
        self.controller = SwarmController(config, config.num_drones, self.rng, obstacles=self.obstacles)

        self.victims = self.rng.uniform(-half * 0.85, half * 0.85, size=(config.num_victims, 2))
        self.victims_found = set()
        self._victim_bodies = []
        self._drone_label_ids = [-1] * config.num_drones

        self._spawn_obstacles()
        if config.gui:
            self._setup_gui_view(half)
            self._draw_victims()

    def _setup_gui_view(self, half):
        """The stock PyBullet debug viewer defaults to a tight, arbitrary
        close-up with its engineering side-panels on - useless for watching
        a whole mission unfold. Give it an overview camera, hide the clutter,
        add a floodwater plane, and color-code each drone so individuals are
        actually distinguishable."""
        client = self.env.getPyBulletClient()
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=client)
        p.resetDebugVisualizerCamera(
            cameraDistance=self.cfg.arena_size * 0.85, cameraYaw=50, cameraPitch=-40,
            cameraTargetPosition=[0, 0, self.cfg.flight_altitude * 0.5], physicsClientId=client,
        )
        water_half = half * 1.15
        water_vis = p.createVisualShape(
            p.GEOM_BOX, halfExtents=[water_half, water_half, 0.02],
            rgbaColor=[0.16, 0.36, 0.55, 0.55], physicsClientId=client,
        )
        p.createMultiBody(baseMass=0, baseVisualShapeIndex=water_vis, basePosition=[0, 0, 0.02], physicsClientId=client)

        for i, drone_id in enumerate(self.env.getDroneIds()):
            color = DRONE_COLORS[i % len(DRONE_COLORS)]
            p.changeVisualShape(drone_id, -1, rgbaColor=color, physicsClientId=client)

    def _sample_obstacle_positions(self, half, drone_xy, max_tries=200):
        """Reject-sample obstacle centers so none overlap a drone's spawn point
        or another obstacle - an obstacle spawned on top of a drone causes an
        instant physical collision at t=0, not a control failure."""
        min_clearance = self.cfg.obstacle_radius + 2.0
        positions = []
        for _ in range(self.cfg.num_obstacles):
            candidate = None
            for _ in range(max_tries):
                c = self.rng.uniform(-half * 0.7, half * 0.7, size=2)
                if np.min(np.linalg.norm(drone_xy - c, axis=1)) < min_clearance:
                    continue
                if positions and np.min(np.linalg.norm(np.array(positions) - c, axis=1)) < min_clearance:
                    continue
                candidate = c
                break
            positions.append(candidate if candidate is not None else c)
        return np.array(positions)

    def _spawn_obstacles(self):
        # Height (0 to 6m) intersects the drones' flight altitude (3m) - these are
        # real collision hazards, not decoration - so only the color changes here:
        # waterlogged debris/timber rather than a construction-cone orange pillar.
        client = self.env.getPyBulletClient()
        for ox, oy in self.obstacles:
            col = p.createCollisionShape(p.GEOM_CYLINDER, radius=self.cfg.obstacle_radius, height=6.0, physicsClientId=client)
            vis = -1
            if self.cfg.gui:
                vis = p.createVisualShape(p.GEOM_CYLINDER, radius=self.cfg.obstacle_radius, length=6.0,
                                           rgbaColor=[0.34, 0.28, 0.20, 1], physicsClientId=client)
            p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
                               basePosition=[ox, oy, 3.0], physicsClientId=client)

    def _draw_victims(self):
        client = self.env.getPyBulletClient()
        for vx, vy in self.victims:
            vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.6, rgbaColor=[0.95, 0.15, 0.25, 1], physicsClientId=client)
            body = p.createMultiBody(baseMass=0, baseVisualShapeIndex=vis, basePosition=[vx, vy, 0.3], physicsClientId=client)
            self._victim_bodies.append(body)

    def _match_victim(self, centroid):
        """Map a consensus-confirmed centroid back to the nearest
        still-unconfirmed victim, purely for scoring - the swarm itself
        only ever acts on `centroid`, never on ground truth. Returns None
        if nothing plausible is nearby (a false-positive consensus, e.g.
        sensor noise from two drones independently misreading empty water
        as the same false location)."""
        cfg = self.cfg
        best_vid, best_d = None, cfg.consensus_cluster_radius * 2
        for vid, victim_xy in enumerate(self.victims):
            if vid in self.victims_found:
                continue
            d = np.linalg.norm(victim_xy - centroid)
            if d < best_d:
                best_vid, best_d = vid, d
        return best_vid

    def _check_detections(self, positions, t):
        """Each drone within its own sensor range of a victim submits a
        noisy candidate report (von-Frisch-style: report what you sensed,
        don't assume it's confirmed). A victim only becomes an actionable
        beacon once ConsensusBoard sees enough independent, mutually
        consistent reports - see consensus.py."""
        client = self.env.getPyBulletClient() if self.cfg.gui else None
        cfg = self.cfg
        for vid, (vx, vy) in enumerate(self.victims):
            if vid in self.victims_found:
                continue
            dists = np.linalg.norm(positions[:, :2] - np.array([vx, vy]), axis=1)
            for drone_id in np.where(dists < cfg.sensor_range)[0]:
                noisy = np.array([vx, vy]) + self.rng.normal(scale=cfg.sensor_noise_std, size=2)
                self.consensus.submit(int(drone_id), noisy, t)

        result = self.consensus.try_confirm(t)
        while result is not None:
            centroid, drones = result
            vid = self._match_victim(centroid)
            if vid is not None:
                self.victims_found.add(vid)
                self.board.announce(vid, [centroid[0], centroid[1], cfg.flight_altitude], min(drones), t)
                if cfg.gui and vid < len(self._victim_bodies):
                    p.changeVisualShape(self._victim_bodies[vid], -1, rgbaColor=[0.25, 0.85, 0.55, 1], physicsClientId=client)
            else:
                self.false_confirmations += 1
            result = self.consensus.try_confirm(t)

    def _update_drone_labels(self, positions):
        """Floating 'D{i}' label above each drone, updated in place each step
        (via replaceItemUniqueId) rather than accumulating a new debug item
        every frame - answers "which drone is which" directly, independent
        of how small the physical drone mesh reads on screen."""
        client = self.env.getPyBulletClient()
        for i in range(self.cfg.num_drones):
            color = DRONE_COLORS[i % len(DRONE_COLORS)][:3]
            self._drone_label_ids[i] = p.addUserDebugText(
                f"D{i}", positions[i] + np.array([0, 0, 0.25]),
                textColorRGB=color, textSize=1.3,
                replaceItemUniqueId=self._drone_label_ids[i],
                physicsClientId=client,
            )

    def run(self):
        cfg = self.cfg
        n_steps = int(cfg.duration_sec * cfg.control_freq_hz)
        rpm_action = np.zeros((cfg.num_drones, 4))
        start_wall = timemod.time()
        t = 0.0

        for step in range(n_steps):
            t = step / cfg.control_freq_hz
            obs, _, _, _, _ = self.env.step(rpm_action)

            positions = obs[:, 0:3]
            rpys = obs[:, 7:10]
            velocities = obs[:, 10:13]

            self.network.tick(positions, velocities)
            self._check_detections(positions, t)
            self.board.service_check(positions, arrival_radius=cfg.arrival_radius)
            self.board.decay(1.0 / cfg.control_freq_hz)

            comp_sizes = self.network.connected_component_sizes(cfg.comm_max_age_steps)
            self._connected_steps += int(len(comp_sizes) == 1)
            self._connectivity_checks += 1

            desired_vels = self.controller.step(positions, velocities, self.board, self.network)

            for i in range(cfg.num_drones):
                target_vel, applied_speed = self.speed_ctrl.to_velocity_command(i, desired_vels[i])
                # Anchor altitude via real position feedback (z held at flight_altitude);
                # x/y are left to pure velocity tracking so the swarm behavior drives motion.
                # Yaw is held at a fixed setpoint (0) rather than "current yaw" - the latter
                # gives zero yaw error every step, i.e. no restoring torque, which lets any
                # residual spin (excited by continuously-curving swarm paths) run away freely.
                target_pos = np.array([positions[i, 0], positions[i, 1], cfg.flight_altitude])
                rpm_action[i], _, _ = self.pid[i].computeControlFromState(
                    control_timestep=self.env.CTRL_TIMESTEP,
                    state=obs[i],
                    target_pos=target_pos,
                    target_rpy=np.array([0.0, 0.0, 0.0]),
                    target_vel=target_vel,
                )
                mode = "recruit" if self.controller.committed_beacon[i] is not None else "search"
                num_neighbors = len(self.network.neighbor_table(i, cfg.comm_max_age_steps))
                self.telemetry.record(t, i, positions[i], velocities[i], rpys[i],
                                       float(np.linalg.norm(velocities[i])), mode, applied_speed,
                                       num_neighbors)

            if cfg.gui:
                if step % 4 == 0:  # a few times a second is plenty legible, and far gentler on the debug renderer
                    self._update_drone_labels(positions)
                sync(step, start_wall, self.env.CTRL_TIMESTEP)

            if len(self.victims_found) == cfg.num_victims:
                break

        self.env.close()
        connectivity_fraction = (self._connected_steps / self._connectivity_checks
                                  if self._connectivity_checks else 1.0)
        return {
            "victims_found": len(self.victims_found),
            "total_victims": cfg.num_victims,
            "time_elapsed": t,
            "telemetry": self.telemetry,
            "false_confirmations": self.false_confirmations,
            "swarm_connectivity_fraction": connectivity_fraction,
        }
