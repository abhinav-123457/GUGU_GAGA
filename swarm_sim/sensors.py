"""Phase 2 sensor models.

This module is the ONLY place permitted to read hidden ground-truth state
(victim positions, obstacle geometry, other drones' exact kinematic
state) in order to produce what a drone's own onboard sensors would
actually measure this tick. Everything it returns is a Phase 1 contract
type (SensorObservation / DetectionCandidate / NeighborObservation) -
noisy, range-limited, FOV-limited, latent, occlusion- and dropout-prone,
exactly like a real sensor. Nothing downstream of this module
(SwarmController, ConsensusBoard, RecruitmentBoard) ever sees ground
truth again once it has passed through here. See docs/PHASE2_SENSING.md
for the full model.

    Hidden simulation plant (ground-truth positions/velocities/obstacles)
                    |
                    v
   VictimSensorModel / ObstacleRangeSensor / NeighborSensorModel   <- HERE
                    |
                    v
    SensorObservation / NeighborObservation      (swarm_sim.contracts)
                    |
                    v
              SwarmController                      (never sees ground truth)

Ground truth stays available in parallel, exclusively for scoring
(mission.py's _match_victim, false_confirmations, connectivity, and the
ground-truth-clearance metric) - never fed back into a controller
decision.
"""
from __future__ import annotations

import math
from collections import deque
from typing import Tuple

import numpy as np

from .contracts import DetectionCandidate, Frame, NeighborObservation


def obstacle_scan_bin_angles(config) -> np.ndarray:
    """The fixed angular offsets (radians, relative to facing direction)
    ObstacleRangeSensor scans and SwarmController must interpret its
    range_returns_m against. A pure function of config so the sensor and
    its consumer can never drift apart - both call this, neither hardcodes
    the layout."""
    fov = math.radians(config.obstacle_sensor_fov_deg)
    return np.linspace(-fov / 2.0, fov / 2.0, config.obstacle_sensor_num_bins)


def facing_angle(heading_xy) -> float:
    """The world-frame angle (radians) a 2D heading vector points along;
    0.0 for a near-zero heading (hovering) rather than an undefined atan2."""
    h = np.asarray(heading_xy, dtype=float)
    if np.linalg.norm(h) < 1e-9:
        return 0.0
    return float(math.atan2(h[1], h[0]))


def _horizontal_bearing(facing_xy, to_target_xy) -> float:
    """Angle in [0, pi] between a facing direction and the direction to a
    target, in the horizontal (x/y) plane. 0 if either vector is
    ~zero-length (can't judge bearing against no direction, don't reject
    on that basis)."""
    f = np.asarray(facing_xy, dtype=float)
    t = np.asarray(to_target_xy, dtype=float)
    fn, tn = float(np.linalg.norm(f)), float(np.linalg.norm(t))
    if fn < 1e-9 or tn < 1e-9:
        return 0.0
    cos_angle = float(np.clip(np.dot(f, t) / (fn * tn), -1.0, 1.0))
    return math.acos(cos_angle)


def rotate_xy(vec, angle_rad: float) -> np.ndarray:
    """`vec` (2- or 3-vector) with its x/y components rotated counter-clockwise
    by `angle_rad`; z (if any) is left alone. Exact identity for angle 0."""
    out = np.array(vec, dtype=float)
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    x, y = out[0], out[1]
    out[0] = c * x - s * y
    out[1] = s * x + c * y
    return out


def _segment_min_distance_to_point(p0, p1, c) -> float:
    """Shortest 2D distance from point c to the segment p0-p1."""
    p0, p1, c = np.asarray(p0, float), np.asarray(p1, float), np.asarray(c, float)
    seg = p1 - p0
    seg_len_sq = float(np.dot(seg, seg))
    if seg_len_sq < 1e-12:
        return float(np.linalg.norm(c - p0))
    tt = float(np.clip(np.dot(c - p0, seg) / seg_len_sq, 0.0, 1.0))
    closest = p0 + tt * seg
    return float(np.linalg.norm(c - closest))


class VictimSensorModel:
    """Per-drone victim detection: range + horizontal/vertical FOV +
    obstacle-occlusion gated, with Gaussian position noise, false
    negatives, false positives, whole-observation latency, and dropout.

    Reads hidden victim ground truth and obstacle geometry internally
    (the simulator's hidden plant - explicitly permitted, see module
    docstring). Returns only DetectionCandidate tuples: no candidate ever
    carries the hidden victim index, and false positives are assigned IDs
    from the exact same scheme as genuine detections, so nothing about a
    candidate's ID or shape reveals whether it is real.
    """

    def __init__(self, config, num_drones, victims_xy, obstacles_xy, rng):
        self.cfg = config
        self.n = num_drones
        self.victims_xy = np.asarray(victims_xy, dtype=float).reshape(-1, 2)
        self.obstacles_xy = np.asarray(obstacles_xy, dtype=float).reshape(-1, 2)
        self.rng = rng
        self._delay = [deque() for _ in range(num_drones)]
        self._id_counter = 0
        # Diagnostic-only counters (added for the Phase 2 diagnostic
        # follow-up): pure bookkeeping, read nothing new, consume no RNG
        # state, and do not affect any returned value - see
        # docs/PHASE2_DIAGNOSTICS.md. Incremented in the same place every
        # tick regardless of whether diagnostics are ever inspected.
        self.diag_raw_candidates_generated = 0
        self.diag_dropped_tick_count = 0

    def is_occluded(self, drone_xy, victim_xy) -> bool:
        """Pure ground-truth geometry, no RNG - safe to reuse from
        mission.py's scoring code (e.g. to classify a tick's detections)
        without that being a ground-truth leak, since it returns only a
        bool and mission.py already legitimately holds this geometry for
        scoring purposes."""
        for obstacle_xy in self.obstacles_xy:
            if _segment_min_distance_to_point(drone_xy, victim_xy, obstacle_xy) < self.cfg.obstacle_radius:
                return True
        return False

    def _next_candidate_id(self) -> str:
        self._id_counter += 1
        return f"det-{self._id_counter}"

    def sense(self, i: int, drone_xyz, heading_xy, t_s: float,
              already_found_mask, own_estimated_xy=None,
              own_yaw_error_rad: float = 0.0) -> Tuple[Tuple[DetectionCandidate, ...], bool]:
        """One tick's victim-detection observation for drone `i`: returns
        (detections, dropped). `dropped` is True when this tick's whole
        observation was lost (sensor dropout, or still inside the initial
        latency ramp-up) - `detections` is always () in that case, but
        `dropped` is what distinguishes "lost observation" from "sensor
        worked, genuinely saw nothing" for SensorObservation.dropout.

        Phase 16B: a camera measures the victim RELATIVE to the drone, and the
        drone then places that measurement in the mission frame using its own
        pose estimate. With `own_estimated_xy` (the drone's estimated x/y) and
        `own_yaw_error_rad` (estimated minus true heading) given, each
        reported position is `own_estimated_xy + R(yaw_error) * (relative
        vector + noise)`, so the drone's localisation error propagates into
        the geotag exactly as it would on real hardware. With both omitted
        (the default) the legacy absolute-truth-plus-noise output is
        unchanged, bit for bit. `drone_xyz` / `heading_xy` are plant-side
        inputs here (true pose, true-frame heading), like the rest of this
        module."""
        cfg = self.cfg
        drone_xy = np.asarray(drone_xyz[:2], dtype=float)
        est_xy = None if own_estimated_xy is None else np.asarray(own_estimated_xy, dtype=float)

        def _placed(relative_xy):
            return est_xy + rotate_xy(relative_xy, own_yaw_error_rad)

        raw = []

        for vid in range(len(self.victims_xy)):
            if already_found_mask[vid]:
                continue  # plant-level: consensus already confirmed this one, stop generating reports for it
            victim_xy = self.victims_xy[vid]
            rel_xy = victim_xy - drone_xy
            horiz_dist = float(np.linalg.norm(rel_xy))
            if horiz_dist > cfg.victim_sensor_range_m:
                continue
            if _horizontal_bearing(heading_xy, rel_xy) > math.radians(cfg.victim_sensor_hfov_deg) / 2.0:
                continue
            # Vertical FOV as a cone measured from nadir (straight down) -
            # this is a downward-looking SAR camera, so directly-below
            # targets should be the EASIEST to see, not excluded by a
            # horizontal-centered cone (an earlier version of this measured
            # elevation from the horizontal plane, which perversely made
            # close-range targets harder to see than far ones - fixed here).
            height_above_victim = drone_xyz[2] - 0.3  # victim rests ~0.3m up, see _draw_victims
            elevation_from_nadir = math.atan2(horiz_dist, max(height_above_victim, 1e-6))
            if elevation_from_nadir > math.radians(cfg.victim_sensor_vfov_deg) / 2.0:
                continue
            if self.is_occluded(drone_xy, victim_xy):
                continue
            if self.rng.random() < cfg.victim_sensor_false_negative_prob:
                continue
            noise_xy = self.rng.normal(scale=cfg.victim_sensor_noise_std_m, size=2)
            noisy_xy = victim_xy + noise_xy if est_xy is None else _placed(rel_xy + noise_xy)
            raw.append((noisy_xy, cfg.victim_sensor_confidence))

        if self.rng.random() < cfg.victim_sensor_false_positive_rate:
            r = float(self.rng.uniform(0.5, cfg.victim_sensor_range_m))
            theta = float(self.rng.uniform(-1.0, 1.0)) * math.radians(cfg.victim_sensor_hfov_deg) / 2.0
            ang = facing_angle(heading_xy) + theta
            phantom_rel = r * np.array([math.cos(ang), math.sin(ang)])
            phantom_xy = drone_xy + phantom_rel if est_xy is None else _placed(phantom_rel)
            raw.append((phantom_xy, cfg.victim_sensor_false_positive_confidence))

        # Candidate IDs are assigned here, after real and phantom entries are
        # already combined, from one counter - nothing about an ID's shape
        # distinguishes a false positive from a genuine detection.
        candidates = tuple(
            DetectionCandidate(
                candidate_id=self._next_candidate_id(), position_m=(float(xy[0]), float(xy[1])),
                frame=Frame.LOCAL_ENU, confidence=confidence,
                localization_uncertainty_m=cfg.victim_sensor_localization_uncertainty_m,
            )
            for xy, confidence in raw
        )
        self.diag_raw_candidates_generated += len(candidates)  # diagnostic only, see __init__

        dropped = self.rng.random() < cfg.victim_sensor_dropout_prob
        if dropped:
            self.diag_dropped_tick_count += 1  # diagnostic only, see __init__
        payload = ((), True) if dropped else (candidates, False)
        self._delay[i].append(payload)
        if len(self._delay[i]) > cfg.victim_sensor_latency_steps:
            return self._delay[i].popleft()
        return (), True


class ObstacleRangeSensor:
    """Fixed-angular-bin range scan for obstacle avoidance/repulsion -
    replaces SwarmController's direct read of ground-truth obstacle
    centers. Each bin reports the range to the nearest obstacle edge in
    that angular slice (or +inf if none within range); a farther obstacle
    behind a nearer one in the same narrow bin is never reported, which is
    exactly the self-occlusion a real range sensor exhibits, with no
    separate model needed for it.
    """

    def __init__(self, config, num_drones, obstacles_xy, rng):
        self.cfg = config
        self.n = num_drones
        self.obstacles_xy = np.asarray(obstacles_xy, dtype=float).reshape(-1, 2)
        self.rng = rng
        self._delay = [deque() for _ in range(num_drones)]
        self.bin_angles = obstacle_scan_bin_angles(config)

    def scan(self, i: int, drone_xy, heading_xy) -> Tuple[float, ...]:
        cfg = self.cfg
        n_bins = len(self.bin_angles)

        if self.rng.random() < cfg.obstacle_sensor_dropout_prob:
            ranges = (math.inf,) * n_bins
        else:
            base_angle = facing_angle(heading_xy)
            drone_xy = np.asarray(drone_xy, dtype=float)
            out = []
            for offset in self.bin_angles:
                ray_dir = np.array([math.cos(base_angle + offset), math.sin(base_angle + offset)])
                best = math.inf
                for obstacle_xy in self.obstacles_xy:
                    rel = obstacle_xy - drone_xy
                    proj = float(np.dot(rel, ray_dir))
                    if proj <= 0.0 or proj > cfg.obstacle_sensor_range_m + cfg.obstacle_radius:
                        continue
                    perp_dist = float(np.linalg.norm(rel - proj * ray_dir))
                    if perp_dist > cfg.obstacle_radius:
                        continue
                    hit_range = proj - math.sqrt(max(cfg.obstacle_radius ** 2 - perp_dist ** 2, 0.0))
                    if 0.0 < hit_range < best:
                        best = hit_range
                if best <= cfg.obstacle_sensor_range_m:
                    noisy = best + float(self.rng.normal(scale=cfg.obstacle_sensor_noise_std_m))
                    out.append(max(0.0, noisy))
                else:
                    out.append(math.inf)
            ranges = tuple(out)

        self._delay[i].append(ranges)
        if len(self._delay[i]) > cfg.obstacle_sensor_latency_steps:
            return self._delay[i].popleft()
        return (math.inf,) * n_bins


class NeighborSensorModel:
    """Local onboard proximity sensing for collision avoidance - replaces
    SwarmController's direct read of the ground-truth drone-position
    array. Deliberately independent of CommsNetwork: this models an
    onboard sensor (vision/lidar/radar), not a radio link, so a degraded
    comms link can never be what degrades collision safety (same
    principle CommsNetwork's own docstring states for the pre-Phase-2
    ground-truth version this replaces).

    Structurally mirrors CommsNetwork.tick()'s per-(receiver, sender)
    last-seen cache + delivery delay + staleness, because it is solving
    the same kind of problem (a bounded-range channel with loss and
    latency) over the same kind of data - reusing that proven shape
    rather than inventing a different one.
    """

    def __init__(self, config, num_drones, rng):
        self.cfg = config
        self.n = num_drones
        self.rng = rng
        self.step = 0
        self._last_seen = [dict() for _ in range(num_drones)]  # last_seen[i][j] = {...}
        self._inflight = []  # (deliver_step, i, j, pos, vel, dist, sample_step)

    def tick(self, positions, velocities, headings, own_estimated_positions=None, own_yaw_errors=None) -> None:
        """positions/velocities: ground-truth Nx3 arrays - the hidden
        plant input this sensor is allowed to read (see module
        docstring). headings: per-drone facing-direction unit vectors -
        each drone's OWN state (used only for that drone's own FOV
        check), not another drone's ground truth.

        Phase 16B: an onboard neighbour sensor measures the neighbour
        RELATIVE to the observer, which the observer then places in its own
        frame using its own pose estimate. With `own_estimated_positions`
        (Nx3, each observer's estimated position) and `own_yaw_errors` (N,
        each observer's estimated-minus-true heading) given, the reported
        position is `estimate_i + R(yaw_error_i) * (relative vector + noise)`
        and the reported velocity is `R(yaw_error_i) * (velocity + noise)`,
        so an observer's own drift cancels out of the relative geometry the
        safety supervisor uses (separation) but shows up in absolute
        position. Omitted (default) = the legacy absolute output, bit for
        bit. `headings` must already be in the true frame in that mode."""
        cfg = self.cfg
        cur = self.step
        deliver_step = cur + cfg.neighbor_sensor_latency_steps
        stale_cutoff = cfg.neighbor_sensor_stale_timeout_steps * 5

        for i in range(self.n):
            for j in range(self.n):
                if i == j:
                    continue
                rel = positions[j] - positions[i]
                dist = float(np.linalg.norm(rel))
                if dist > cfg.neighbor_sensor_range_m:
                    continue
                if _horizontal_bearing(headings[i][:2], rel[:2]) > math.radians(cfg.neighbor_sensor_fov_deg) / 2.0:
                    continue
                if self.rng.random() < cfg.neighbor_sensor_dropout_prob:
                    continue
                pos_noise = self.rng.normal(scale=cfg.neighbor_sensor_noise_std_m, size=3)
                vel_noise = self.rng.normal(scale=cfg.neighbor_sensor_velocity_noise_std_mps, size=3)
                if own_estimated_positions is None:
                    noisy_pos = positions[j] + pos_noise
                    noisy_vel = velocities[j] + vel_noise
                else:
                    yaw_err = 0.0 if own_yaw_errors is None else float(own_yaw_errors[i])
                    noisy_pos = np.asarray(own_estimated_positions[i], dtype=float) + rotate_xy(rel + pos_noise, yaw_err)
                    noisy_vel = rotate_xy(velocities[j] + vel_noise, yaw_err)
                self._inflight.append((deliver_step, i, j, noisy_pos.copy(), noisy_vel.copy(), dist, cur))

        still_pending = []
        for deliver_step_msg, i, j, pos, vel, dist, sample_step in self._inflight:
            if deliver_step_msg <= cur:
                self._last_seen[i][j] = {
                    "step": cur, "sample_step": sample_step, "pos": pos, "vel": vel, "dist": dist,
                }
            else:
                still_pending.append((deliver_step_msg, i, j, pos, vel, dist, sample_step))
        self._inflight = still_pending

        for i in range(self.n):
            self._last_seen[i] = {j: e for j, e in self._last_seen[i].items() if cur - e["step"] <= stale_cutoff}

        self.step += 1

    def observations_for(self, i: int, dt_s: float) -> Tuple[NeighborObservation, ...]:
        cfg = self.cfg
        cur = self.step
        out = []
        for j, e in sorted(self._last_seen[i].items()):
            age_steps = cur - e["step"]
            stale = age_steps > cfg.neighbor_sensor_stale_timeout_steps
            sample_t = e["sample_step"] * dt_s
            delivery_t = e["step"] * dt_s
            confidence = cfg.neighbor_sensor_confidence * (0.5 if stale else 1.0)
            out.append(NeighborObservation(
                receiver_id=f"drone{i}", sender_id=f"drone{j}", frame=Frame.LOCAL_ENU,
                measured_position_m=tuple(float(x) for x in e["pos"]),
                measured_velocity_mps=tuple(float(x) for x in e["vel"]),
                sample_timestamp_s=sample_t, delivery_timestamp_s=delivery_t,
                packet_age_s=delivery_t - sample_t,
                communication_confidence=confidence, stale=stale,
            ))
        return tuple(out)
