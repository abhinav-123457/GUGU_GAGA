"""Flood search-and-rescue mission: open arena, sparse obstacles, a swarm
searching for scattered victims using bio-inspired flocking + Levy-flight
search + von Frisch/Bonabeau-style recruitment, with full telemetry and
per-drone speed control.

Drives DSLPIDControl directly (rather than VelocityAviary's action
shortcut) so target velocities are tracked in real m/s with no hidden
scaling ceiling - the per-drone speed cap in MissionConfig/SpeedController
is the actual physical limit that gets applied.

Phase 2 sensing boundary: this file is where hidden ground truth (victim
positions, obstacle geometry, the physics engine's exact drone-position
array) legitimately lives, because it is the simulator's hidden plant -
but past this file, it goes one of exactly two ways:
  1. through swarm_sim/sensors.py's sensor models, which turn it into
     noisy/range-limited/latent SensorObservation and NeighborObservation
     contracts before SwarmController ever sees it, or
  2. into a function clearly marked as scoring/evaluation-only
     (_match_victim, _classify_detections_for_scoring, the connectivity/
     clearance/contact bookkeeping in run()) - never fed back into a
     controller decision.
See docs/PHASE2_SENSING.md for the full model.
"""
import math
import time as timemod

import numpy as np
import pybullet as p

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.utils.utils import sync

from . import distributed_consensus as dconsensus
from . import sensors
from .autopilot import AdapterCommand as AutopilotAdapterCommand
from .autopilot import ConnectionState as AutopilotConnectionState
from .autopilot import MockAdapter
from .autopilot import VehicleTelemetry as AutopilotVehicleTelemetry
from .consensus import ConsensusBoard
from .contracts import CommandType, Frame, GeofenceSpec, HealthState, SensorObservation, VehicleState
from .controller import SwarmController
from .diagnostics import MissionDiagnostics
from .network import CommsNetwork
from .recruitment import RecruitmentBoard
from .safety_supervisor import (
    TIER_MISSION_CANDIDATE, CandidateCommand, MissionContext, SafetySupervisor, SafetySupervisorConfig, tier_of,
)
from .seeding import SeedManager
from .sensors import NeighborSensorModel, ObstacleRangeSensor, VictimSensorModel
from .speed_control import SpeedController
from .telemetry import TelemetryHub

# CF2X body radius, for vehicle-body (edge-to-edge) clearance: arm length
# (0.0397m) plus propeller radius (0.023135m), both printed by
# BaseAviary's own startup log from the vendored URDF - not a guess.
DRONE_BODY_RADIUS_M = 0.0397 + 0.023135

# One distinct, high-contrast color per drone (cycles if num_drones > len(this)),
# so an individual drone can be told apart at a glance instead of reading as
# identical tiny gizmos.
DRONE_COLORS = [
    [0.95, 0.35, 0.10, 1], [0.15, 0.65, 0.95, 1], [0.90, 0.80, 0.15, 1],
    [0.60, 0.30, 0.90, 1], [0.20, 0.80, 0.45, 1], [0.95, 0.50, 0.75, 1],
    [0.40, 0.75, 0.95, 1], [0.85, 0.45, 0.20, 1],
]


def _dedupe_contacts_by_pair(contacts):
    """Collapse PyBullet's possibly-several contact-point records for the
    same touching pair of bodies within one tick (one manifold point each,
    same physical event) down to one representative record per (body_a,
    body_b) pair. `contacts` is a `getContactPoints()`-shaped list (or any
    subset of it): tuples whose index 1 and 2 are the two body unique ids.
    Pure/stateless - does not affect physics or what PyBullet reports, only
    how many diagnostic events they log as. See docs/PHASE2_DIAGNOSTICS.md's
    raw-vs-deduplicated contact-count reconciliation. Returns a dict keyed
    by the sorted (body_a, body_b) pair, in first-seen order."""
    seen = {}
    for c in contacts:
        pair_key = (min(c[1], c[2]), max(c[1], c[2]))
        seen.setdefault(pair_key, c)
    return seen


class FloodSearchMission:
    def __init__(self, config):
        self.cfg = config
        self.rng = np.random.default_rng(config.seed)
        self.telemetry = TelemetryHub()
        self.board = RecruitmentBoard(decay_rate=config.beacon_decay_rate)
        self.network = CommsNetwork(config, config.num_drones, self.rng)
        # Reference/comparison implementation - kept unmodified (Phase 5
        # requirement 13). Constructed unconditionally so scripts/tests can
        # always compare it against the distributed path below, regardless
        # of which one config.consensus_mode actually drives this mission.
        self.consensus = ConsensusBoard(config)
        self._drone_ids = tuple(f"drone{i}" for i in range(config.num_drones))
        self._drone_index = {did: i for i, did in enumerate(self._drone_ids)}
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
        self.victims = self.rng.uniform(-half * 0.85, half * 0.85, size=(config.num_victims, 2))
        self.victims_found = set()
        self._victim_bodies = []
        self._drone_label_ids = [-1] * config.num_drones
        self._drone_id_set = set(self.env.getDroneIds())
        self._drone_body_id_to_index = {body_id: i for i, body_id in enumerate(self.env.getDroneIds())}
        self._obstacle_body_ids = set()

        self.controller = SwarmController(config, config.num_drones, self.rng)

        # Phase 4: independent safety supervisor - see
        # swarm_sim/safety_supervisor.py and docs/PHASE4_SAFETY.md. Its own
        # config is deliberately separate from the flocking config above.
        self.safety = SafetySupervisor(SafetySupervisorConfig(
            max_speed_mps=config.safety_max_speed_mps,
            max_accel_mps2=config.safety_max_accel_mps2,
            max_turn_rate_radps=config.safety_max_turn_rate_radps,
            command_latency_s=config.safety_command_latency_s,
            assumed_brake_decel_mps2=config.safety_assumed_brake_decel_mps2,
            vehicle_body_radius_m=DRONE_BODY_RADIUS_M,
            separation_hard_margin_m=config.safety_separation_hard_margin_m,
            separation_uncertainty_inflation_m=config.safety_separation_uncertainty_inflation_m,
            obstacle_hard_margin_m=config.safety_obstacle_hard_margin_m,
            obstacle_uncertainty_inflation_m=config.safety_obstacle_uncertainty_inflation_m,
            stuck_window_s=config.safety_stuck_window_s,
            stuck_min_progress_m=config.safety_stuck_min_progress_m,
            stuck_near_obstacle_m=config.safety_stuck_near_obstacle_m,
            stuck_violation_streak=config.safety_stuck_violation_streak,
            escape_duration_s=config.safety_escape_duration_s,
            escape_speed_mps=config.safety_escape_speed_mps,
            geofence_margin_m=config.safety_geofence_margin_m,
            altitude_margin_m=config.safety_altitude_margin_m,
            battery_reserve_fraction=config.safety_battery_reserve_fraction,
            battery_critical_fraction=config.safety_battery_critical_fraction,
            link_timeout_s=config.safety_link_timeout_s,
            lost_agent_timeout_s=config.safety_lost_agent_timeout_s,
        ))
        self._geofence = GeofenceSpec(
            frame=Frame.LOCAL_ENU, center_m=(0.0, 0.0), half_extents_m=(half, half),
            floor_alt_m=config.safety_altitude_floor_m, ceiling_alt_m=config.safety_altitude_ceiling_m,
        )
        self.operator_abort = False   # tests/scenarios may set this directly to model an operator-triggered abort
        self._safety_candidate_count = 0
        self._safety_accepted_count = 0
        self._safety_rejected_count = 0
        self._safety_override_count = 0
        self._safety_state_counts = {}
        self._safety_cpu_time_s = 0.0
        # Phase 4.1 diagnostics: per-tick safety-transition telemetry
        # (required investigation item 2) plus the extra bookkeeping
        # needed to compute override-switch-count/longest-override-duration
        # for the Phase 4.1 rerun report - see docs/PHASE4_SAFETY.md.
        self.safety_telemetry = []
        self._prev_safety_state = {}
        self._state_since = {}
        self._prev_safety_tier = {}
        self._prev_command_velocity = {}
        self._safety_override_switch_count = 0
        self._safety_longest_override_duration_s = 0.0

        # Phase 2 sensing: three independent RNG streams (never one shared
        # stream - see docs/PHASE2_SENSING.md), scoped just to sensing so
        # the rest of this mission's existing RNG usage is untouched.
        self._sensor_seeds = SeedManager(config.seed)
        self.victim_sensor = VictimSensorModel(
            config, config.num_drones, self.victims, self.obstacles,
            self._sensor_seeds.rng("victim_sensor"),
        )
        self.obstacle_sensor = ObstacleRangeSensor(
            config, config.num_drones, self.obstacles, self._sensor_seeds.rng("obstacle_sensor"),
        )
        self.neighbor_sensor = NeighborSensorModel(
            config, config.num_drones, self._sensor_seeds.rng("neighbor_sensor"),
        )

        # Phase 5: peer-local distributed consensus (swarm_sim/distributed_consensus.py).
        # Its own message-dropout RNG stream is deliberately separate from
        # self.network.rng (see network.py's CommsNetwork.__init__ docstring)
        # so enabling/disabling it never perturbs the position-broadcast
        # dropout sequence flocking depends on.
        self.network.message_rng = self._sensor_seeds.rng("distributed_consensus_network")
        self.distributed_consensus = dconsensus.DistributedConsensus(
            self._drone_ids, config.consensus_quorum, config.consensus_cluster_radius,
            config.consensus_window_sec,
        )

        # Phase 6: autopilot adapter boundary (swarm_sim/autopilot/) - see
        # docs/PHASE6_AUTOPILOT_ADAPTERS.md. "mock_adapter" (default) is
        # the architectural reference path: every safety-evaluated Command
        # additionally passes through MockAdapter.send_command() before
        # reaching PyBullet. "direct" is the pre-Phase-6 path, kept only
        # as a temporary comparison baseline - see run()'s per-drone loop.
        self.autopilot_adapters = {}
        if config.autopilot_path == "mock_adapter":
            adapter_rng = self._sensor_seeds.rng("autopilot_mock")
            self.autopilot_adapters = {
                i: MockAdapter(
                    vehicle_id=self._drone_ids[i], operating_frame=Frame.LOCAL_ENU,
                    command_latency_s=config.autopilot_command_latency_s,
                    telemetry_latency_s=config.autopilot_telemetry_latency_s,
                    command_packet_loss_prob=config.autopilot_command_packet_loss_prob,
                    telemetry_packet_loss_prob=config.autopilot_telemetry_packet_loss_prob,
                    telemetry_stale_timeout_s=config.autopilot_telemetry_stale_timeout_s,
                    rng=adapter_rng,
                )
                for i in range(config.num_drones)
            }
            for adapter in self.autopilot_adapters.values():
                adapter.connect()
        self._adapter_sequence = {i: 0 for i in range(config.num_drones)}
        self._adapter_candidate_count = 0
        self._adapter_accepted_count = 0
        self._adapter_rejected_count = 0
        self._adapter_reject_reasons = {}
        self._adapter_stale_telemetry_count = 0
        self._adapter_replayed_command_count = 0
        self._adapter_mode_transition_count = 0
        self._adapter_failsafe_transition_count = 0
        self._adapter_cpu_time_s = 0.0

        # Scenario-report bookkeeping (scoring/evaluation only - see run()).
        self._true_positive_detections = 0
        self._false_positive_detections = 0
        self._missed_detections = 0
        self._observation_opportunities = 0
        self._localization_errors = []
        self._detection_latencies_s = []
        self._neighbor_obs_ages_s = []
        self._min_sensor_observed_clearance_m = math.inf
        self._min_ground_truth_clearance_m = math.inf
        self._min_sensor_observed_obstacle_clearance_m = math.inf
        self._min_ground_truth_obstacle_clearance_m = math.inf
        self._contact_steps = 0

        # Phase 2 diagnostic follow-up (docs/PHASE2_DIAGNOSTICS.md): purely
        # observational bookkeeping, added strictly for reporting - see
        # swarm_sim/diagnostics.py's module docstring for what this is not.
        self.diagnostics = MissionDiagnostics(
            self.victims, config.consensus_quorum, config.consensus_cluster_radius,
        )

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
            body_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
                                          basePosition=[ox, oy, 3.0], physicsClientId=client)
            self._obstacle_body_ids.add(body_id)

    def _draw_victims(self):
        client = self.env.getPyBulletClient()
        for vx, vy in self.victims:
            vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.6, rgbaColor=[0.95, 0.15, 0.25, 1], physicsClientId=client)
            body = p.createMultiBody(baseMass=0, baseVisualShapeIndex=vis, basePosition=[vx, vy, 0.3], physicsClientId=client)
            self._victim_bodies.append(body)

    def _match_victim(self, centroid):
        """SCORING ONLY. Map a consensus-confirmed centroid back to the
        nearest still-unconfirmed victim - the swarm itself only ever acts
        on `centroid`, never on ground truth. Returns None if nothing
        plausible is nearby (a false-positive consensus, e.g. two drones
        independently hallucinating the same false location)."""
        cfg = self.cfg
        best_vid, best_d = None, cfg.consensus_cluster_radius * 2
        for vid, victim_xy in enumerate(self.victims):
            if vid in self.victims_found:
                continue
            d = np.linalg.norm(victim_xy - centroid)
            if d < best_d:
                best_vid, best_d = vid, d
        return best_vid

    def _classify_detections_for_scoring(self, drone_xy, heading_xy, detections, already_found_mask):
        """SCORING ONLY - reads ground truth to label this tick's already-
        sensed detections as matching a real victim or not, and to count
        missed-detection opportunities, for the scenario report. Reuses
        VictimSensorModel.is_occluded (pure geometry, no RNG) so
        "observable" here means exactly what the sensor itself would have
        been able to see before its own noise/false-negative draw - this
        never changes what was already sensed or fed to consensus, it only
        labels it after the fact.

        Returns `matched_vids_for_candidates`, a list the same length and
        order as `detections`, so the caller can attribute each
        already-submitted consensus report to a victim (or None) for the
        Phase 2 diagnostic report without this function recomputing or
        re-triggering anything."""
        cfg = self.cfg
        # Floored so this doesn't degenerate to a zero-radius window (and
        # therefore reject even an exact, noise-free match) when
        # victim_sensor_noise_std_m == 0 - e.g. a "perfect sensing"
        # scenario config.
        match_radius = max(cfg.victim_sensor_noise_std_m * 4, 0.25)
        matched_vids = set()
        matched_vids_for_candidates = []
        for candidate in detections:
            cand_xy = np.array(candidate.position_m)
            best_vid, best_d = None, match_radius
            for vid, victim_xy in enumerate(self.victims):
                d = float(np.linalg.norm(victim_xy - cand_xy))
                if d < best_d:
                    best_vid, best_d = vid, d
            matched_vids_for_candidates.append(best_vid)
            if best_vid is not None:
                self._true_positive_detections += 1
                self._localization_errors.append(best_d)
                matched_vids.add(best_vid)
                self.diagnostics.record_detection_event(best_vid, best_d)
            else:
                self._false_positive_detections += 1
                self.diagnostics.record_detection_event(None, None)

        for vid, victim_xy in enumerate(self.victims):
            rel_xy = victim_xy - drone_xy
            horiz_dist = float(np.linalg.norm(rel_xy))
            self.diagnostics.record_range_check(vid, horiz_dist <= cfg.victim_sensor_range_m)

            if already_found_mask[vid]:
                continue
            if horiz_dist > cfg.victim_sensor_range_m:
                continue
            if sensors._horizontal_bearing(heading_xy, rel_xy) > math.radians(cfg.victim_sensor_hfov_deg) / 2:
                continue
            if self.victim_sensor.is_occluded(drone_xy, victim_xy):
                continue
            self._observation_opportunities += 1
            if vid not in matched_vids:
                self._missed_detections += 1

        return matched_vids_for_candidates

    def _submit_detections(self, drone_id, detections, t):
        """Feed this drone's sensed victim-detection candidates (from
        VictimSensorModel - never ground truth) into ConsensusBoard.
        ConsensusBoard itself is unchanged by Phase 2 - only what feeds it
        changed."""
        for candidate in detections:
            self.consensus.submit(drone_id, np.array(candidate.position_m), t)

    def _submit_detections_distributed(self, drone_id, detections, t, positions):
        """Phase 5 equivalent of _submit_detections: this drone's sensed
        candidates (still never ground truth - same VictimSensorModel
        output) go into ONLY its own private DroneConsensusNode, then get
        broadcast (plus any evidence it's relaying on another drone's
        behalf) over CommsNetwork's now-generic message channel - not into
        any shared object. See docs/PHASE5_DISTRIBUTED_CONSENSUS.md."""
        did = self._drone_ids[drone_id]
        msg = self.distributed_consensus.outgoing_detection_message(did, detections, t)
        if msg is not None:
            self.network.send_message(drone_id, msg, positions)
        if self.cfg.consensus_relay_enabled:
            relay_msg = self.distributed_consensus.outgoing_relay_message(did, t)
            if relay_msg is not None:
                self.network.send_message(drone_id, relay_msg, positions)

    def _resolve_confirmed_detections_distributed(self, t, positions):
        """Phase 5 equivalent of _resolve_confirmed_detections: drains
        every drone's own local quorum check (already deduplicated
        globally by DistributedConsensus.try_confirm_all - several drones
        independently reaching the same swarm-wide realization in the same
        tick is expected and correct, and must be scored/announced once).
        Ground truth (_match_victim) is read here ONLY to score a
        confirmed centroid against the nearest still-missing victim -
        identical use to the centralized path, just fed by a different
        confirmation source."""
        client = self.env.getPyBulletClient() if self.cfg.gui else None
        self.diagnostics.record_consensus_report_snapshot(self._distributed_pending_reports_snapshot())
        for drone_id, result, confirm_msg in self.distributed_consensus.try_confirm_all(t):
            i = self._drone_index[drone_id]
            self.network.send_message(i, confirm_msg, positions)
            centroid = np.array(result.centroid_m)
            vid = self._match_victim(centroid)
            first_drone_idx = self._drone_index[result.contributing_drone_ids[0]]
            if vid is not None:
                self.victims_found.add(vid)
                self.board.announce(vid, [centroid[0], centroid[1], self.cfg.flight_altitude], first_drone_idx, t)
                if self.cfg.gui and vid < len(self._victim_bodies):
                    p.changeVisualShape(self._victim_bodies[vid], -1, rgbaColor=[0.25, 0.85, 0.55, 1], physicsClientId=client)
                self.diagnostics.record_confirmation(vid, centroid, t, len(result.contributing_drone_ids))
            else:
                self.false_confirmations += 1
                self.diagnostics.record_false_confirmation(len(result.contributing_drone_ids))

    def _distributed_pending_reports_snapshot(self):
        """Adapts DistributedConsensus's per-node pending evidence into the
        exact dict shape ConsensusBoard.reports already uses, so
        MissionDiagnostics.record_consensus_report_snapshot (Phase 2,
        unmodified) works identically for both consensus_mode values -
        needed for a fair centralized-vs-distributed diagnostics
        comparison. Evaluation-only; never fed back into a decision."""
        return [
            {"drone_id": r["drone_id"], "pos": np.array(r["pos"]), "t": r["t"]}
            for r in self.distributed_consensus.pending_evidence_snapshot()
        ]

    def _resolve_confirmed_detections(self, t):
        """Drain ConsensusBoard for anything newly confirmed this tick
        (run once, after every drone's reports for this tick are in).
        Ground truth is read here ONLY to score a confirmed centroid
        against the nearest still-missing victim (_match_victim) - the
        swarm itself only ever acts on the centroid via
        RecruitmentBoard.announce."""
        client = self.env.getPyBulletClient() if self.cfg.gui else None
        # Snapshotted BEFORE try_confirm() runs, on purpose: try_confirm()
        # consumes (removes) exactly the reports that reach quorum, so a
        # snapshot taken after the loop would never see the cluster that
        # just got confirmed - this is the state quorum was actually
        # evaluated against.
        self.diagnostics.record_consensus_report_snapshot(self.consensus.reports)
        result = self.consensus.try_confirm(t)
        while result is not None:
            centroid, drones = result
            vid = self._match_victim(centroid)
            if vid is not None:
                self.victims_found.add(vid)
                self.board.announce(vid, [centroid[0], centroid[1], self.cfg.flight_altitude], min(drones), t)
                if self.cfg.gui and vid < len(self._victim_bodies):
                    p.changeVisualShape(self._victim_bodies[vid], -1, rgbaColor=[0.25, 0.85, 0.55, 1], physicsClientId=client)
                self.diagnostics.record_confirmation(vid, centroid, t, len(drones))
            else:
                self.false_confirmations += 1
                self.diagnostics.record_false_confirmation(len(drones))
            result = self.consensus.try_confirm(t)

    def _track_clearance_and_contacts(self, i, positions, neighbor_obs, range_returns):
        """SCORING ONLY. Several deliberately separate clearance metrics
        (docs/PHASE2_DIAGNOSTICS.md has the full breakdown):
        - center-to-center, estimated (from this drone's own sensed
          neighbor positions) vs actual (ground truth)
        - vehicle-body (edge-to-edge), derived from center-to-center minus
          2x DRONE_BODY_RADIUS_M
        - obstacle clearance (edge distance), estimated (sensed range
          scan) vs actual (ground truth obstacle centers)
        None of this is fed back into any decision - see
        SwarmController.step(), which never receives ground truth or this
        method's output."""
        cfg = self.cfg
        if neighbor_obs:
            sensed_center = min(
                float(np.linalg.norm(positions[i] - np.array(n.measured_position_m))) for n in neighbor_obs
            )
            self._min_sensor_observed_clearance_m = min(self._min_sensor_observed_clearance_m, sensed_center)

        true_dists = np.linalg.norm(positions - positions[i], axis=1)
        true_dists[i] = np.inf
        if cfg.num_drones > 1:
            self._min_ground_truth_clearance_m = min(self._min_ground_truth_clearance_m, float(np.min(true_dists)))

        finite_returns = [r for r in range_returns if math.isfinite(r)]
        if finite_returns:
            self._min_sensor_observed_obstacle_clearance_m = min(
                self._min_sensor_observed_obstacle_clearance_m, min(finite_returns),
            )
        if len(self.obstacles) > 0:
            true_obstacle_edge_dists = np.linalg.norm(self.obstacles - positions[i, :2], axis=1) - cfg.obstacle_radius
            self._min_ground_truth_obstacle_clearance_m = min(
                self._min_ground_truth_obstacle_clearance_m, float(np.min(true_obstacle_edge_dists)),
            )

    def _drone_has_contact_now(self, i: int) -> bool:
        """Evaluation-only input to the Phase 4 safety supervisor: is
        drone i touching anything (another drone, an obstacle, the
        ground) RIGHT NOW, per PyBullet's own physics this tick - not
        victim ground truth. Queried via its own (cheap, bodyA/bodyB-
        filtered) getContactPoints() call, deliberately separate from
        _classify_and_log_contacts below so Phase 2's diagnostic contact
        accounting is untouched by Phase 4 - see docs/PHASE4_SAFETY.md."""
        client = self.env.getPyBulletClient()
        body_id = self.env.getDroneIds()[i]
        if p.getContactPoints(bodyA=body_id, physicsClientId=client):
            return True
        return bool(p.getContactPoints(bodyB=body_id, physicsClientId=client))

    def _send_to_autopilot_adapter(self, i, own_state, cmd, t):
        """Phase 6: routes an already safety-evaluated `contracts.Command`
        through drone i's `AutopilotAdapter` (see swarm_sim/autopilot/) -
        the last gate before a velocity setpoint can reach PyBullet when
        `cfg.autopilot_path == "mock_adapter"`. Only ever called with
        `cmd` = `decision.filtered_command` (an ACCEPTED SafetyDecision's
        own output) - never the raw SwarmController candidate `desired_vel`
        (see tests/test_autopilot_architecture.py). Returns the velocity
        to actually apply this tick (np.zeros(3) on any adapter
        rejection)."""
        adapter = self.autopilot_adapters[i]
        telemetry = AutopilotVehicleTelemetry(
            vehicle_id=self._drone_ids[i], timestamp_s=t, frame=Frame.LOCAL_ENU,
            position_m=own_state.position_m, velocity_mps=own_state.velocity_mps,
            acceleration_mps2=own_state.acceleration_mps2, attitude_rad=own_state.attitude_rad,
            angular_velocity_radps=own_state.angular_velocity_radps, battery_fraction=own_state.battery_fraction,
            estimator_valid=own_state.estimator_valid, connection_state=adapter.state, autopilot_mode=adapter.mode,
            armed=adapter.armed, failsafe=adapter.failsafe, sequence=self._adapter_sequence[i],
        )
        adapter.push_ground_truth_state(telemetry)

        sequence = self._adapter_sequence[i]
        self._adapter_sequence[i] += 1
        adapter_command = AutopilotAdapterCommand(command=cmd, sequence=sequence)
        prev_mode, prev_state = adapter.mode, adapter.state

        adapter_t0 = timemod.perf_counter()
        result = adapter.send_command(adapter_command)
        self._adapter_cpu_time_s += timemod.perf_counter() - adapter_t0

        self._adapter_candidate_count += 1
        if result.accepted:
            self._adapter_accepted_count += 1
            transformed = result.transformed_command.command
            if transformed.command_type == CommandType.VELOCITY_SETPOINT:
                vel = np.array(transformed.desired_velocity_mps)
            else:
                vel = np.zeros(3)
        else:
            self._adapter_rejected_count += 1
            self._adapter_reject_reasons[result.reason] = self._adapter_reject_reasons.get(result.reason, 0) + 1
            if result.reason == "stale_telemetry":
                self._adapter_stale_telemetry_count += 1
            if result.reason in ("sequence_replayed_conflicting", "sequence_replayed_stale"):
                self._adapter_replayed_command_count += 1
            vel = np.zeros(3)

        if adapter.mode != prev_mode:
            self._adapter_mode_transition_count += 1
        if adapter.state != prev_state and AutopilotConnectionState.FAILSAFE in (adapter.state, prev_state):
            self._adapter_failsafe_transition_count += 1
        return vel

    def _classify_and_log_contacts(self, t, positions, velocities, tick_neighbor_obs, tick_range_returns,
                                     tick_safety_context):
        """SCORING/EVALUATION ONLY. Classifies every PyBullet-verified
        physical contact this tick involving a drone, correlating it with
        each involved drone's own sensing state at that same tick.

        Phase 4.1 extends this with each involved drone's SAFETY state at
        that tick (tick_safety_context - altitude, vertical velocity,
        active safety state/constraints, command source/tier, previous
        vs. current commanded velocity) - required investigation item 1
        ("classify all ground contacts"). Still evaluation-only: reading
        this richer context for the report does not feed anything back
        into a decision.

        CRITICAL LIMITATION (see docs/PHASE2_DIAGNOSTICS.md): this is
        evaluation instrumentation, not a safety mechanism (Phase 4's
        SafetySupervisor is the actual safety mechanism; this method
        still only records, after the fact, what happened and what state
        the supervisor was in)."""
        client = self.env.getPyBulletClient()
        contacts = p.getContactPoints(physicsClientId=client)
        relevant = [c for c in contacts if c[1] in self._drone_id_set or c[2] in self._drone_id_set]
        if relevant:
            self._contact_steps += 1
        # raw_contact_point_count: every getContactPoints() record touching
        # a drone, summed over the whole run - PyBullet can report several
        # manifold points for the SAME touching pair within one tick, which
        # is why this is larger than the deduplicated event count below.
        # See docs/PHASE2_DIAGNOSTICS.md's raw-vs-deduplicated reconciliation.
        self.diagnostics.raw_contact_point_count += len(relevant)

        for pair_key, c in _dedupe_contacts_by_pair(relevant).items():
            body_a, body_b = c[1], c[2]
            drone_ids_involved = [idx for body in (body_a, body_b)
                                   for idx in ([self._drone_body_id_to_index[body]]
                                               if body in self._drone_body_id_to_index else [])]

            estimated_clearance_m = None
            actual_clearance_m = None
            sensor_age_s = None
            stale_or_dropout = True

            if body_a in self._drone_id_set and body_b in self._drone_id_set:
                a, b = self._drone_body_id_to_index[body_a], self._drone_body_id_to_index[body_b]
                actual_clearance_m = float(np.linalg.norm(positions[a] - positions[b]))
                for observer, other in ((a, b), (b, a)):
                    match = next((n for n in tick_neighbor_obs.get(observer, ())
                                  if n.sender_id == f"drone{other}"), None)
                    if match is not None:
                        estimated_clearance_m = float(np.linalg.norm(
                            positions[observer] - np.array(match.measured_position_m)))
                        sensor_age_s = match.packet_age_s
                        stale_or_dropout = match.stale
                        break
            elif body_a in self._drone_id_set or body_b in self._drone_id_set:
                drone_idx = drone_ids_involved[0]
                if body_a in self._obstacle_body_ids or body_b in self._obstacle_body_ids:
                    obstacle_body = body_a if body_a in self._obstacle_body_ids else body_b
                    obstacle_pos = np.array(p.getBasePositionAndOrientation(obstacle_body, physicsClientId=client)[0][:2])
                    actual_clearance_m = float(np.linalg.norm(positions[drone_idx, :2] - obstacle_pos)) - self.cfg.obstacle_radius
                    finite_returns = [r for r in tick_range_returns.get(drone_idx, ()) if math.isfinite(r)]
                    estimated_clearance_m = min(finite_returns) if finite_returns else None
                    stale_or_dropout = estimated_clearance_m is None
                else:
                    actual_clearance_m = float(positions[drone_idx, 2])  # height above the ground plane
                    estimated_clearance_m = None  # no downward/ground sensor modeled - see limitation

            # Phase 4.1: correlate with the FIRST involved drone's own
            # safety-supervisor context this tick (drone_ground/drone_obstacle
            # contacts involve exactly one drone; drone_drone picks the
            # lower-indexed one - a documented, simple choice, not both).
            safety_ctx = tick_safety_context.get(drone_ids_involved[0], {}) if drone_ids_involved else {}

            self.diagnostics.classify_contact(
                t=t, body_a=body_a, body_b=body_b, drone_id_set=self._drone_id_set,
                obstacle_body_ids=self._obstacle_body_ids,
                estimated_clearance_m=estimated_clearance_m, actual_clearance_m=actual_clearance_m,
                sensor_age_s=sensor_age_s, command_age_s=sensor_age_s,  # proxy - see docs/PHASE2_DIAGNOSTICS.md
                observation_dropout_or_stale=stale_or_dropout,
                drone_ids_involved=drone_ids_involved, pair_key=pair_key,
                altitude_m=safety_ctx.get("altitude_m"), vertical_velocity_mps=safety_ctx.get("vertical_velocity_mps"),
                safety_state=safety_ctx.get("safety_state"), active_constraints=safety_ctx.get("active_constraints", ()),
                command_source=safety_ctx.get("command_source"), override_tier=safety_ctx.get("override_tier"),
                previous_command_velocity_mps=safety_ctx.get("previous_command_velocity_mps"),
                current_command_velocity_mps=safety_ctx.get("current_command_velocity_mps"),
            )

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
        dt = 1.0 / cfg.control_freq_hz
        rpm_action = np.zeros((cfg.num_drones, 4))
        start_wall = timemod.time()
        t = 0.0

        for step in range(n_steps):
            t = step / cfg.control_freq_hz
            obs, _, _, _, _ = self.env.step(rpm_action)

            # Ground truth from here to the sensor-model calls below is fed
            # ONLY into: (a) the sensor models (their explicitly-permitted
            # hidden-plant input), (b) CommsNetwork (already comms-realistic,
            # unchanged since before Phase 2), and (c) scoring/telemetry.
            # SwarmController.step() below never receives `positions` or
            # `velocities` directly.
            positions = obs[:, 0:3]
            rpys = obs[:, 7:10]
            velocities = obs[:, 10:13]

            self.network.tick(positions, velocities)
            self.neighbor_sensor.tick(positions, velocities, self.controller.headings)
            self.board.service_check(positions, arrival_radius=cfg.arrival_radius)
            self.board.decay(dt)

            comp_sizes = self.network.connected_component_sizes(cfg.comm_max_age_steps)
            self._connected_steps += int(len(comp_sizes) == 1)
            self._connectivity_checks += 1
            # Phase 4: per-drone isolation, for MissionContext.own_agent_isolated
            # (LOST_AGENT) - same already-comms-realistic network data as
            # swarm_connectivity_fraction above, just broken out per drone.
            per_drone_component_size = self.network.component_size_per_drone(cfg.comm_max_age_steps)

            already_found_mask = [vid in self.victims_found for vid in range(cfg.num_victims)]
            tick_neighbor_obs = {}
            tick_range_returns = {}
            tick_safety_context = {}

            for i in range(cfg.num_drones):
                heading_xy = self.controller.headings[i][:2].copy()

                own_state = VehicleState(
                    vehicle_id=f"drone{i}", sim_time_s=t, frame=Frame.LOCAL_ENU,
                    position_m=tuple(float(x) for x in positions[i]),
                    velocity_mps=tuple(float(x) for x in velocities[i]),
                    acceleration_mps2=(0.0, 0.0, 0.0),
                    attitude_rad=tuple(float(x) for x in rpys[i]),
                    angular_velocity_radps=(0.0, 0.0, 0.0),
                    battery_fraction=1.0, health_state=HealthState.OK,
                    estimator_valid=True, last_valid_command_time_s=t,
                )

                sensing_t0 = timemod.perf_counter()
                detections, dropped = self.victim_sensor.sense(
                    i, positions[i], heading_xy, t, already_found_mask,
                )
                range_returns = self.obstacle_sensor.scan(i, positions[i, :2], heading_xy)
                sensor_obs = SensorObservation(
                    vehicle_id=f"drone{i}",
                    sensor_timestamp_s=max(0.0, t - cfg.victim_sensor_latency_steps * dt),
                    sensor_latency_s=cfg.victim_sensor_latency_steps * dt,
                    fov_deg=cfg.victim_sensor_hfov_deg,
                    range_returns_m=range_returns,
                    occluded=tuple(False for _ in range_returns),
                    dropout=dropped, pose_uncertainty_m=0.0,
                    detections=detections,
                )
                neighbor_obs = self.neighbor_sensor.observations_for(i, dt)
                self.diagnostics.sensing_cpu_time_s += timemod.perf_counter() - sensing_t0
                tick_neighbor_obs[i] = neighbor_obs
                tick_range_returns[i] = range_returns

                matched_vids_for_candidates = self._classify_detections_for_scoring(
                    positions[i, :2], heading_xy, detections, already_found_mask,
                )
                consensus_t0 = timemod.perf_counter()
                if cfg.consensus_mode == "distributed":
                    self._submit_detections_distributed(i, detections, t, positions)
                else:
                    self._submit_detections(i, detections, t)
                self.diagnostics.consensus_cpu_time_s += timemod.perf_counter() - consensus_t0
                for matched_vid in matched_vids_for_candidates:
                    self.diagnostics.record_submission(matched_vid)
                self._detection_latencies_s.extend([sensor_obs.sensor_latency_s] * len(detections))
                self._neighbor_obs_ages_s.extend(n.packet_age_s for n in neighbor_obs)
                self._track_clearance_and_contacts(i, positions, neighbor_obs, range_returns)

                desired_vel = self.controller.step(
                    i, own_state, sensor_obs, neighbor_obs, self.board, self.network,
                )

                safe_vel = desired_vel
                if cfg.safety_enabled:
                    candidate = CandidateCommand(
                        vehicle_id=f"drone{i}", desired_velocity_mps=tuple(float(v) for v in desired_vel),
                        frame=Frame.LOCAL_ENU, timestamp_s=t, expiration_time_s=t + 4 * dt,
                        source="SwarmController",
                    )
                    mission_context = MissionContext(
                        geofence=self._geofence, operator_abort=self.operator_abort,
                        contact_detected=self._drone_has_contact_now(i),
                        own_agent_isolated=(cfg.num_drones > 1 and per_drone_component_size[i] == 1),
                    )
                    safety_t0 = timemod.perf_counter()
                    decision = self.safety.evaluate(candidate, own_state, sensor_obs, neighbor_obs,
                                                       mission_context, now_s=t)
                    self._safety_cpu_time_s += timemod.perf_counter() - safety_t0

                    self._safety_candidate_count += 1
                    state_name = decision.emergency_state.value
                    self._safety_state_counts[state_name] = self._safety_state_counts.get(state_name, 0) + 1
                    if decision.accepted:
                        self._safety_accepted_count += 1
                        if decision.active_constraints and decision.active_constraints != ("sensor_dropout",):
                            self._safety_override_count += 1
                        cmd = decision.filtered_command
                        if cmd.command_type == CommandType.VELOCITY_SETPOINT:
                            safe_vel = np.array(cmd.desired_velocity_mps)
                        else:
                            # HOLD / LAND / ABORT: no setpoint fields (see
                            # contracts.Command's _COMMAND_TYPE_RULES) - the
                            # conservative default of holding horizontal
                            # position is the only one of these three this
                            # sim's PID loop (which has no motor-cutoff or
                            # descent-rate concept - see docs/PHASE4_SAFETY.md's
                            # "Assumptions") can actually realize.
                            safe_vel = np.zeros(3)
                    else:
                        self._safety_rejected_count += 1
                        safe_vel = np.zeros(3)

                    # Phase 4.1 required investigation item 2: per-tick
                    # safety-transition telemetry, plus the bookkeeping
                    # needed for the rerun report's override-switch-count/
                    # longest-override-duration metrics.
                    prev_state = self._prev_safety_state.get(i)
                    if prev_state != state_name:
                        if prev_state is not None and prev_state != "NORMAL":
                            duration = t - self._state_since.get(i, t)
                            self._safety_longest_override_duration_s = max(
                                self._safety_longest_override_duration_s, duration,
                            )
                        self._state_since[i] = t
                    self._prev_safety_state[i] = state_name

                    tier = tier_of(decision.active_constraints)
                    prev_tier = self._prev_safety_tier.get(i)
                    if (prev_tier is not None and tier is not None and prev_tier != tier
                            and prev_tier != TIER_MISSION_CANDIDATE and tier != TIER_MISSION_CANDIDATE):
                        self._safety_override_switch_count += 1
                    self._prev_safety_tier[i] = tier

                    current_cmd_vel = tuple(float(v) for v in safe_vel)
                    prev_cmd_vel = self._prev_command_velocity.get(i, (0.0, 0.0, 0.0))
                    tick_safety_context[i] = {
                        "altitude_m": float(positions[i, 2]), "vertical_velocity_mps": float(velocities[i, 2]),
                        "safety_state": state_name, "active_constraints": decision.active_constraints,
                        "command_source": (decision.filtered_command.source
                                            if decision.filtered_command else "SwarmController_bypass"),
                        "override_tier": tier, "previous_command_velocity_mps": prev_cmd_vel,
                        "current_command_velocity_mps": current_cmd_vel,
                    }
                    self._prev_command_velocity[i] = current_cmd_vel
                    self.safety_telemetry.append({
                        "t": t, "drone_id": i, "previous_state": prev_state, "current_state": state_name,
                        "state_duration_s": t - self._state_since.get(i, t), "override_tier": tier,
                        "commanded_horizontal_velocity_mps": math.hypot(current_cmd_vel[0], current_cmd_vel[1]),
                        "commanded_vertical_velocity_mps": current_cmd_vel[2],
                        "own_altitude_m": float(positions[i, 2]), "own_vertical_velocity_mps": float(velocities[i, 2]),
                        "contact_status": bool(mission_context.contact_detected),
                    })

                    # Phase 6: adapter boundary (swarm_sim/autopilot/) - see
                    # docs/PHASE6_AUTOPILOT_ADAPTERS.md. Only runs on an
                    # ACCEPTED safety decision (there is a real, validated
                    # Command to forward); "direct" keeps the pre-Phase-6
                    # behavior (safe_vel unchanged) as a temporary
                    # comparison baseline only.
                    if cfg.autopilot_path == "mock_adapter" and decision.accepted:
                        safe_vel = self._send_to_autopilot_adapter(i, own_state, decision.filtered_command, t)

                target_vel, applied_speed = self.speed_ctrl.to_velocity_command(i, safe_vel)
                # Anchor altitude via real position feedback (z held at flight_altitude);
                # x/y are left to pure velocity tracking so the swarm behavior drives motion.
                # Yaw is held at a fixed setpoint (0) rather than "current yaw" - the latter
                # gives zero yaw error every step, i.e. no restoring torque, which lets any
                # residual spin (excited by continuously-curving swarm paths) run away freely.
                # Altitude is otherwise held at a fixed cruise setpoint (see
                # comment above) - the one exception is a nonzero z from the
                # safety supervisor's own filtered command (altitude floor/
                # ceiling correction, see safety_supervisor.py's
                # _evaluate_altitude), integrated one step and clamped
                # inside the geofence's floor/ceiling.
                target_z = cfg.flight_altitude
                if cfg.safety_enabled and abs(safe_vel[2]) > 1e-9:
                    target_z = float(np.clip(positions[i, 2] + safe_vel[2] * dt,
                                              self._geofence.floor_alt_m, self._geofence.ceiling_alt_m))
                target_pos = np.array([positions[i, 0], positions[i, 1], target_z])
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

            consensus_t0 = timemod.perf_counter()
            if cfg.consensus_mode == "distributed":
                self.network.pump_messages()
                for i in range(cfg.num_drones):
                    inbox = self.network.poll_inbox(i)
                    self.distributed_consensus.deliver(self._drone_ids[i], inbox, t)
                self._resolve_confirmed_detections_distributed(t, positions)
            else:
                self._resolve_confirmed_detections(t)
            self.diagnostics.consensus_cpu_time_s += timemod.perf_counter() - consensus_t0

            self._classify_and_log_contacts(t, positions, velocities, tick_neighbor_obs, tick_range_returns,
                                              tick_safety_context)

            if cfg.gui:
                if step % 4 == 0:  # a few times a second is plenty legible, and far gentler on the debug renderer
                    self._update_drone_labels(positions)
                sync(step, start_wall, self.env.CTRL_TIMESTEP)

            if len(self.victims_found) == cfg.num_victims:
                break

        self.env.close()
        self.diagnostics.contact_step_count = self._contact_steps
        self.diagnostics.finalize()
        connectivity_fraction = (self._connected_steps / self._connectivity_checks
                                  if self._connectivity_checks else 1.0)
        mean_localization_error = (float(np.mean(self._localization_errors))
                                    if self._localization_errors else None)
        mean_neighbor_obs_age = (float(np.mean(self._neighbor_obs_ages_s))
                                  if self._neighbor_obs_ages_s else None)

        def _finite_or_none(x):
            return None if math.isinf(x) else x

        return {
            "victims_found": len(self.victims_found),
            "total_victims": cfg.num_victims,
            "time_elapsed": t,
            "telemetry": self.telemetry,
            "false_confirmations": self.false_confirmations,
            "swarm_connectivity_fraction": connectivity_fraction,
            # Phase 2 sensing/scoring metrics - see docs/PHASE2_SENSING.md
            "true_positive_detections": self._true_positive_detections,
            "false_positive_detections": self._false_positive_detections,
            "missed_detections": self._missed_detections,
            "observation_opportunities": self._observation_opportunities,
            "mean_localization_error_m": mean_localization_error,
            "mean_neighbor_observation_age_s": mean_neighbor_obs_age,
            "min_sensor_observed_clearance_m": _finite_or_none(self._min_sensor_observed_clearance_m),
            "min_ground_truth_clearance_m": _finite_or_none(self._min_ground_truth_clearance_m),
            "vehicle_body_radius_m": DRONE_BODY_RADIUS_M,
            "min_vehicle_body_clearance_m": (
                None if math.isinf(self._min_ground_truth_clearance_m)
                else self._min_ground_truth_clearance_m - 2 * DRONE_BODY_RADIUS_M
            ),
            "min_sensor_observed_obstacle_clearance_m": _finite_or_none(self._min_sensor_observed_obstacle_clearance_m),
            "min_ground_truth_obstacle_clearance_m": _finite_or_none(self._min_ground_truth_obstacle_clearance_m),
            "neighbor_sensor_uncertainty_margin_m": cfg.neighbor_sensor_noise_std_m,
            "contact_steps": self._contact_steps,
            # Phase 2 diagnostic follow-up - see docs/PHASE2_DIAGNOSTICS.md
            "diagnostics": self.diagnostics.to_report_dict(),
            "sensor_raw_candidate_count": self.victim_sensor.diag_raw_candidates_generated,
            "sensor_dropped_tick_count": self.victim_sensor.diag_dropped_tick_count,
            # Approximate - see docs/PHASE2_DIAGNOSTICS.md's "expired candidate
            # count" note. total_submitted - consumed_by_confirmations -
            # still_pending_at_end = expired mid-run without ever confirming.
            "consensus_candidate_count": self._true_positive_detections + self._false_positive_detections,
            "consensus_expired_candidate_count_approx": (
                (self._true_positive_detections + self._false_positive_detections)
                - self.diagnostics.reports_consumed_by_confirmations
                - len(self.consensus.reports)
            ),
            "consensus_final_pending_report_count": len(self.consensus.reports),
            # Phase 4 safety supervisor - see docs/PHASE4_SAFETY.md.
            "safety_enabled": cfg.safety_enabled,
            "safety_candidate_count": self._safety_candidate_count,
            "safety_accepted_count": self._safety_accepted_count,
            "safety_rejected_count": self._safety_rejected_count,
            "safety_override_count": self._safety_override_count,
            "safety_state_counts": dict(self._safety_state_counts),
            "safety_cpu_time_s": self._safety_cpu_time_s,
            "safety_event_log": [e.to_dict() for e in self.safety.event_log],
            # Phase 4.1 - see docs/PHASE4_SAFETY.md's Phase 4.1 section.
            "safety_override_switch_count": self._safety_override_switch_count,
            "safety_longest_override_duration_s": self._safety_longest_override_duration_s,
            "safety_telemetry": self.safety_telemetry,
            # Phase 5 distributed consensus - see docs/PHASE5_DISTRIBUTED_CONSENSUS.md.
            "consensus_mode": cfg.consensus_mode,
            "distributed_report_count": self.distributed_consensus.report_count,
            "distributed_confirmed_victim_count": self.distributed_consensus.confirmed_victim_count(),
            "distributed_duplicate_suppressed_count": self.distributed_consensus.total_duplicate_suppressed_count(),
            "distributed_own_repeat_suppressed_count": self.distributed_consensus.total_own_repeat_suppressed_count(),
            "distributed_stale_message_count": self.distributed_consensus.total_stale_message_count(),
            "distributed_expired_evidence_count": self.distributed_consensus.total_expired_evidence_count(),
            "distributed_quorum_failure_count": self.distributed_consensus.total_quorum_failure_count(),
            "distributed_pending_evidence_count": sum(
                n.pending_evidence_count() for n in self.distributed_consensus.nodes.values()
            ),
            # Phase 6 autopilot adapter boundary - see docs/PHASE6_AUTOPILOT_ADAPTERS.md.
            "autopilot_path": cfg.autopilot_path,
            "adapter_candidate_count": self._adapter_candidate_count,
            "adapter_accepted_count": self._adapter_accepted_count,
            "adapter_rejected_count": self._adapter_rejected_count,
            "adapter_reject_reasons": dict(self._adapter_reject_reasons),
            "adapter_stale_telemetry_count": self._adapter_stale_telemetry_count,
            "adapter_replayed_command_count": self._adapter_replayed_command_count,
            "adapter_mode_transition_count": self._adapter_mode_transition_count,
            "adapter_failsafe_transition_count": self._adapter_failsafe_transition_count,
            "adapter_cpu_time_s": self._adapter_cpu_time_s,
        }
