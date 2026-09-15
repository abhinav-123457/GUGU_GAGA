import math
import re

import numpy as np
import pytest

from swarm_sim.config import MissionConfig
from swarm_sim.contracts import DetectionCandidate
from swarm_sim.sensors import NeighborSensorModel, ObstacleRangeSensor, VictimSensorModel


def _cfg(**overrides):
    return MissionConfig(**overrides)


def _quiet_victim_cfg(**overrides):
    """A config with false-negative/false-positive/dropout/latency all
    zeroed, so a test can isolate exactly one gating mechanism (range,
    FOV, occlusion) at a time."""
    defaults = dict(
        victim_sensor_false_negative_prob=0.0, victim_sensor_false_positive_rate=0.0,
        victim_sensor_dropout_prob=0.0, victim_sensor_latency_steps=0,
    )
    defaults.update(overrides)
    return _cfg(**defaults)


# ---------------------------------------------------------------------------
# Victim sensor: range, FOV, occlusion, noise, false negatives/positives
# ---------------------------------------------------------------------------

def test_victim_in_range_and_fov_is_detected():
    cfg = _quiet_victim_cfg()
    model = VictimSensorModel(cfg, 1, [(3.0, 0.0)], [], np.random.default_rng(1))
    detections, dropped = model.sense(0, np.array([0.0, 0.0, 3.0]), (1.0, 0.0), 0.0, [False])
    assert dropped is False
    assert len(detections) == 1
    assert isinstance(detections[0], DetectionCandidate)


def test_sensor_range_is_enforced():
    cfg = _quiet_victim_cfg(victim_sensor_range_m=6.0)
    far_victim = (100.0, 0.0)
    model = VictimSensorModel(cfg, 1, [far_victim], [], np.random.default_rng(1))
    detections, _ = model.sense(0, np.array([0.0, 0.0, 3.0]), (1.0, 0.0), 0.0, [False])
    assert detections == ()


def test_horizontal_fov_is_enforced():
    cfg = _quiet_victim_cfg(victim_sensor_hfov_deg=60.0)
    behind_victim = (-3.0, 0.0)  # directly behind a drone facing +x
    model = VictimSensorModel(cfg, 1, [behind_victim], [], np.random.default_rng(1))
    detections, _ = model.sense(0, np.array([0.0, 0.0, 3.0]), (1.0, 0.0), 0.0, [False])
    assert detections == ()


def test_narrow_vertical_fov_excludes_far_shallow_target():
    cfg = _quiet_victim_cfg(victim_sensor_vfov_deg=10.0, victim_sensor_range_m=20.0)
    # At altitude 3m over a victim at ~0.3m, a target 6m away sits at a
    # shallow ~55 degree angle from nadir - well outside a 10 degree cone.
    shallow_victim = (6.0, 0.0)
    model = VictimSensorModel(cfg, 1, [shallow_victim], [], np.random.default_rng(1))
    detections, _ = model.sense(0, np.array([0.0, 0.0, 3.0]), (1.0, 0.0), 0.0, [False])
    assert detections == ()


def test_directly_below_target_is_within_any_positive_vfov():
    cfg = _quiet_victim_cfg(victim_sensor_vfov_deg=1.0)
    directly_below = (0.0, 0.0)
    model = VictimSensorModel(cfg, 1, [directly_below], [], np.random.default_rng(1))
    detections, _ = model.sense(0, np.array([0.0, 0.0, 3.0]), (1.0, 0.0), 0.0, [False])
    assert len(detections) == 1


def test_obstacle_occlusion_suppresses_detection():
    cfg = _quiet_victim_cfg(victim_sensor_range_m=20.0, victim_sensor_hfov_deg=180.0,
                             victim_sensor_vfov_deg=180.0, obstacle_radius=1.0)
    victim_xy = (10.0, 0.0)
    obstacle_directly_between = (5.0, 0.0)
    model = VictimSensorModel(cfg, 1, [victim_xy], [obstacle_directly_between], np.random.default_rng(1))
    detections, _ = model.sense(0, np.array([0.0, 0.0, 3.0]), (1.0, 0.0), 0.0, [False])
    assert detections == ()


def test_no_occlusion_when_obstacle_is_off_the_line_of_sight():
    cfg = _quiet_victim_cfg(victim_sensor_range_m=20.0, victim_sensor_hfov_deg=180.0,
                             victim_sensor_vfov_deg=180.0, obstacle_radius=1.0)
    victim_xy = (10.0, 0.0)
    obstacle_off_to_the_side = (5.0, 8.0)
    model = VictimSensorModel(cfg, 1, [victim_xy], [obstacle_off_to_the_side], np.random.default_rng(1))
    detections, _ = model.sense(0, np.array([0.0, 0.0, 3.0]), (1.0, 0.0), 0.0, [False])
    assert len(detections) == 1


def test_false_negative_probability_one_always_misses():
    cfg = _quiet_victim_cfg(victim_sensor_false_negative_prob=1.0)
    model = VictimSensorModel(cfg, 1, [(3.0, 0.0)], [], np.random.default_rng(1))
    detections, _ = model.sense(0, np.array([0.0, 0.0, 3.0]), (1.0, 0.0), 0.0, [False])
    assert detections == ()


def test_false_positive_rate_one_always_produces_a_phantom():
    cfg = _quiet_victim_cfg(victim_sensor_false_positive_rate=1.0)
    model = VictimSensorModel(cfg, 1, [], [], np.random.default_rng(1))  # no real victims at all
    detections, _ = model.sense(0, np.array([0.0, 0.0, 3.0]), (1.0, 0.0), 0.0, [])
    assert len(detections) == 1


def test_already_found_victim_generates_no_candidate():
    cfg = _quiet_victim_cfg()
    model = VictimSensorModel(cfg, 1, [(3.0, 0.0)], [], np.random.default_rng(1))
    detections, _ = model.sense(0, np.array([0.0, 0.0, 3.0]), (1.0, 0.0), 0.0, [True])
    assert detections == ()


def test_position_noise_deterministic_under_fixed_seed():
    cfg = _quiet_victim_cfg(victim_sensor_noise_std_m=1.0)
    model_a = VictimSensorModel(cfg, 1, [(3.0, 0.0)], [], np.random.default_rng(123))
    model_b = VictimSensorModel(cfg, 1, [(3.0, 0.0)], [], np.random.default_rng(123))
    det_a, _ = model_a.sense(0, np.array([0.0, 0.0, 3.0]), (1.0, 0.0), 0.0, [False])
    det_b, _ = model_b.sense(0, np.array([0.0, 0.0, 3.0]), (1.0, 0.0), 0.0, [False])
    assert det_a[0].position_m == det_b[0].position_m


def test_position_noise_differs_across_seeds():
    cfg = _quiet_victim_cfg(victim_sensor_noise_std_m=1.0)
    model_a = VictimSensorModel(cfg, 1, [(3.0, 0.0)], [], np.random.default_rng(1))
    model_b = VictimSensorModel(cfg, 1, [(3.0, 0.0)], [], np.random.default_rng(2))
    det_a, _ = model_a.sense(0, np.array([0.0, 0.0, 3.0]), (1.0, 0.0), 0.0, [False])
    det_b, _ = model_b.sense(0, np.array([0.0, 0.0, 3.0]), (1.0, 0.0), 0.0, [False])
    assert det_a[0].position_m != det_b[0].position_m


def test_whole_observation_dropout():
    cfg = _quiet_victim_cfg(victim_sensor_dropout_prob=1.0)
    model = VictimSensorModel(cfg, 1, [(3.0, 0.0)], [], np.random.default_rng(1))
    detections, dropped = model.sense(0, np.array([0.0, 0.0, 3.0]), (1.0, 0.0), 0.0, [False])
    assert dropped is True
    assert detections == ()


def test_latency_delays_delivery_by_exactly_n_ticks():
    cfg = _quiet_victim_cfg(victim_sensor_latency_steps=2)
    model = VictimSensorModel(cfg, 1, [(3.0, 0.0)], [], np.random.default_rng(1))
    own = np.array([0.0, 0.0, 3.0])
    d0, dropped0 = model.sense(0, own, (1.0, 0.0), 0.0, [False])
    d1, dropped1 = model.sense(0, own, (1.0, 0.0), 1.0, [False])
    d2, dropped2 = model.sense(0, own, (1.0, 0.0), 2.0, [False])
    assert dropped0 is True and d0 == ()   # still ramping up
    assert dropped1 is True and d1 == ()   # still ramping up
    assert dropped2 is False and len(d2) == 1   # first tick's real sensing, delayed 2 steps


# ---------------------------------------------------------------------------
# Candidate IDs never encode which victim (or whether real/phantom) they are
# ---------------------------------------------------------------------------

def test_candidate_id_scheme_does_not_encode_victim_identity():
    cfg = _quiet_victim_cfg(victim_sensor_hfov_deg=360.0, victim_sensor_vfov_deg=180.0,
                             victim_sensor_range_m=50.0)
    # Two victims at very different hidden indices/positions; the returned
    # candidate_id format must not depend on which one triggered it.
    model = VictimSensorModel(cfg, 1, [(3.0, 0.0), (4.0, 4.0)], [], np.random.default_rng(1))
    detections, _ = model.sense(0, np.array([0.0, 0.0, 3.0]), (1.0, 1.0), 0.0, [False, False])
    assert len(detections) == 2
    for d in detections:
        assert re.fullmatch(r"det-\d+", d.candidate_id), d.candidate_id
        assert "v0" not in d.candidate_id and "v1" not in d.candidate_id


def test_changing_ground_truth_victim_id_does_not_change_candidate_id_scheme():
    """Swap which hidden index corresponds to the sensed victim - the
    candidate_id format/value the sensor produces must be identical
    either way, since it is generated by a call-order counter, never
    derived from the hidden victim index."""
    cfg = _quiet_victim_cfg(victim_sensor_hfov_deg=360.0, victim_sensor_vfov_deg=180.0)
    own = np.array([0.0, 0.0, 3.0])

    # Case A: the sensed victim is hidden index 0.
    model_a = VictimSensorModel(cfg, 1, [(3.0, 0.0), (999.0, 999.0)], [], np.random.default_rng(1))
    det_a, _ = model_a.sense(0, own, (1.0, 0.0), 0.0, [False, True])

    # Case B: the SAME physical victim is now hidden index 1 instead.
    model_b = VictimSensorModel(cfg, 1, [(999.0, 999.0), (3.0, 0.0)], [], np.random.default_rng(1))
    det_b, _ = model_b.sense(0, own, (1.0, 0.0), 0.0, [True, False])

    assert len(det_a) == 1 and len(det_b) == 1
    assert det_a[0].candidate_id == det_b[0].candidate_id  # same counter state, same result either way


def test_detection_candidate_contract_has_no_ground_truth_field():
    import dataclasses
    field_names = {f.name for f in dataclasses.fields(DetectionCandidate)}
    assert not (field_names & {"victim_id", "ground_truth_id", "true_position_m"})


# ---------------------------------------------------------------------------
# Obstacle range sensor
# ---------------------------------------------------------------------------

def test_obstacle_scan_reports_finite_range_when_in_front():
    cfg = _cfg(obstacle_sensor_dropout_prob=0.0, obstacle_sensor_noise_std_m=0.0)
    sensor = ObstacleRangeSensor(cfg, 1, [(3.0, 0.0)], np.random.default_rng(1))
    ranges = sensor.scan(0, np.array([0.0, 0.0]), (1.0, 0.0))
    assert any(math.isfinite(r) for r in ranges)
    assert min(ranges) < 3.0  # edge distance, strictly less than center distance


def test_obstacle_scan_all_inf_when_nothing_in_range():
    cfg = _cfg(obstacle_sensor_dropout_prob=0.0)
    sensor = ObstacleRangeSensor(cfg, 1, [(100.0, 100.0)], np.random.default_rng(1))
    ranges = sensor.scan(0, np.array([0.0, 0.0]), (1.0, 0.0))
    assert all(math.isinf(r) for r in ranges)


def test_obstacle_scan_dropout_reads_as_all_inf():
    cfg = _cfg(obstacle_sensor_dropout_prob=1.0)
    sensor = ObstacleRangeSensor(cfg, 1, [(3.0, 0.0)], np.random.default_rng(1))
    ranges = sensor.scan(0, np.array([0.0, 0.0]), (1.0, 0.0))
    assert all(math.isinf(r) for r in ranges)


def test_obstacle_scan_nearer_obstacle_occludes_farther_one_in_same_bin():
    cfg = _cfg(obstacle_sensor_dropout_prob=0.0, obstacle_sensor_noise_std_m=0.0, obstacle_sensor_num_bins=1,
                obstacle_sensor_fov_deg=1.0)
    sensor = ObstacleRangeSensor(cfg, 1, [(3.0, 0.0), (6.0, 0.0)], np.random.default_rng(1))
    ranges = sensor.scan(0, np.array([0.0, 0.0]), (1.0, 0.0))
    assert len(ranges) == 1
    assert ranges[0] < 6.0 - cfg.obstacle_radius + 0.01  # reports the nearer obstacle, not the farther one


# ---------------------------------------------------------------------------
# Neighbor sensor: range/FOV, noise, latency, dropout, staleness
# ---------------------------------------------------------------------------

def test_neighbor_within_range_and_fov_is_observed():
    cfg = _cfg(neighbor_sensor_dropout_prob=0.0, neighbor_sensor_latency_steps=0)
    model = NeighborSensorModel(cfg, 2, np.random.default_rng(1))
    positions = np.array([[0.0, 0.0, 3.0], [2.0, 0.0, 3.0]])
    velocities = np.zeros((2, 3))
    headings = np.array([[1.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    model.tick(positions, velocities, headings)
    obs = model.observations_for(0, dt_s=1.0 / 24)
    assert len(obs) == 1
    assert obs[0].sender_id == "drone1"


def test_neighbor_out_of_range_is_not_observed():
    cfg = _cfg(neighbor_sensor_range_m=3.0, neighbor_sensor_dropout_prob=0.0)
    model = NeighborSensorModel(cfg, 2, np.random.default_rng(1))
    positions = np.array([[0.0, 0.0, 3.0], [100.0, 0.0, 3.0]])
    velocities = np.zeros((2, 3))
    headings = np.array([[1.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    model.tick(positions, velocities, headings)
    assert model.observations_for(0, dt_s=1.0 / 24) == ()


def test_neighbor_measured_position_is_not_exact_ground_truth():
    cfg = _cfg(neighbor_sensor_dropout_prob=0.0, neighbor_sensor_noise_std_m=1.0)
    model = NeighborSensorModel(cfg, 2, np.random.default_rng(1))
    true_pos_j = np.array([2.0, 0.0, 3.0])
    positions = np.array([[0.0, 0.0, 3.0], true_pos_j])
    velocities = np.zeros((2, 3))
    headings = np.array([[1.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    model.tick(positions, velocities, headings)
    obs = model.observations_for(0, dt_s=1.0 / 24)
    assert obs[0].measured_position_m != tuple(true_pos_j)


def test_neighbor_latency_produces_correct_sample_and_delivery_timestamps():
    cfg = _cfg(neighbor_sensor_dropout_prob=0.0, neighbor_sensor_latency_steps=3)
    model = NeighborSensorModel(cfg, 2, np.random.default_rng(1))
    positions = np.array([[0.0, 0.0, 3.0], [2.0, 0.0, 3.0]])
    velocities = np.zeros((2, 3))
    headings = np.array([[1.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    dt = 1.0 / 24
    for _ in range(4):
        model.tick(positions, velocities, headings)
    obs = model.observations_for(0, dt_s=dt)
    assert len(obs) == 1
    n = obs[0]
    assert n.delivery_timestamp_s - n.sample_timestamp_s == pytest.approx(3 * dt)
    assert n.packet_age_s == pytest.approx(3 * dt)


def test_neighbor_dropout_prevents_fresh_update_but_stale_cache_persists():
    cfg = _cfg(neighbor_sensor_dropout_prob=0.0, neighbor_sensor_stale_timeout_steps=2)
    model = NeighborSensorModel(cfg, 2, np.random.default_rng(1))
    positions = np.array([[0.0, 0.0, 3.0], [2.0, 0.0, 3.0]])
    velocities = np.zeros((2, 3))
    headings = np.array([[1.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    model.tick(positions, velocities, headings)  # one good sighting

    model.cfg.neighbor_sensor_dropout_prob = 1.0  # force dropout on every subsequent tick
    for _ in range(5):
        model.tick(positions, velocities, headings)

    obs = model.observations_for(0, dt_s=1.0 / 24)
    assert len(obs) == 1          # the old cached reading is still returned...
    assert obs[0].stale is True   # ...but correctly marked stale


def test_neighbor_sensor_stream_unaffected_by_other_sensors_stream_usage():
    """Regression for 'do not use one shared sensor RNG stream': heavy use
    of a DIFFERENT sensor's rng object must not perturb the neighbor
    sensor's own output at all, since (as SeedManager gives each
    subsystem in mission.py) they are independent Generator objects, not
    one shared stream."""
    cfg = _cfg()
    positions = np.array([[0.0, 0.0, 3.0], [2.0, 0.0, 3.0]])
    velocities = np.zeros((2, 3))
    headings = np.array([[1.0, 0.0, 0.0], [1.0, 0.0, 0.0]])

    rng_neighbor_a = np.random.default_rng(7)
    model_a = NeighborSensorModel(cfg, 2, rng_neighbor_a)
    model_a.tick(positions, velocities, headings)
    obs_a = model_a.observations_for(0, dt_s=1.0 / 24)

    # A separate, same-seeded "victim sensor" stream gets burned through
    # heavily first - since it's a different Generator object entirely,
    # this must have zero effect on the neighbor sensor's own stream/output.
    rng_victim_like_b = np.random.default_rng(7)
    rng_victim_like_b.normal(size=10_000)
    rng_neighbor_b = np.random.default_rng(7)  # neighbor's own stream: same seed as A, untouched
    model_b = NeighborSensorModel(cfg, 2, rng_neighbor_b)
    model_b.tick(positions, velocities, headings)
    obs_b = model_b.observations_for(0, dt_s=1.0 / 24)

    assert obs_a[0].measured_position_m == obs_b[0].measured_position_m


def test_mission_sensor_seed_manager_gives_three_independent_streams():
    """Mirrors how mission.py actually wires sensing: one SeedManager,
    three named subsystem streams. Using one heavily must not perturb
    another's sequence."""
    from swarm_sim.seeding import SeedManager

    mgr_a = SeedManager(master_seed=42)
    mgr_a.rng("victim_sensor").normal(size=5000)  # burn through victim_sensor's stream
    neighbor_draws_a = mgr_a.rng("neighbor_sensor").random(10)

    mgr_b = SeedManager(master_seed=42)
    # obstacle_sensor's stream burned through instead this time - different
    # subsystem, but neighbor_sensor's own stream must still match mgr_a's.
    mgr_b.rng("obstacle_sensor").normal(size=999)
    neighbor_draws_b = mgr_b.rng("neighbor_sensor").random(10)

    assert list(neighbor_draws_a) == list(neighbor_draws_b)
