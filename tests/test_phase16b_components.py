"""Phase 16B: the small, simulator-free pieces the estimator is wired through -
network payload vs link geometry, body-relative sensor placement, the
covariance-aware geofence, and the config knobs. See
docs/PHASE16B_ESTIMATION.md. The PyBullet-level wiring is in
test_phase16b_mission.py."""
import dataclasses
import math

import numpy as np
import pytest

from swarm_sim.config import MissionConfig
from swarm_sim.contracts import (
    Frame, GeofenceSpec, HealthState, SafetyState, SensorObservation, VehicleState,
)
from swarm_sim.network import CommsNetwork
from swarm_sim.safety_supervisor import CandidateCommand, MissionContext, SafetySupervisor, SafetySupervisorConfig
from swarm_sim.sensors import NeighborSensorModel, VictimSensorModel, rotate_xy


# ==========================================================================
# rotate_xy
# ==========================================================================

class TestRotateXY:
    def test_zero_angle_is_an_exact_identity(self):
        v = np.array([1.234567890123, -9.87654321, 4.2])
        assert np.array_equal(rotate_xy(v, 0.0), v)

    def test_quarter_turn_and_z_untouched(self):
        assert rotate_xy((1.0, 0.0, 7.0), math.pi / 2) == pytest.approx((0.0, 1.0, 7.0), abs=1e-12)

    def test_two_vectors_and_norm_preserved(self):
        v = rotate_xy((3.0, 4.0), 1.1)
        assert len(v) == 2 and math.hypot(*v) == pytest.approx(5.0)

    def test_inverse(self):
        v = np.array([2.0, -3.0, 1.0])
        assert rotate_xy(rotate_xy(v, 0.7), -0.7) == pytest.approx(v)

    def test_does_not_mutate_its_input(self):
        v = np.array([1.0, 0.0, 0.0])
        rotate_xy(v, 1.0)
        assert list(v) == [1.0, 0.0, 0.0]


# ==========================================================================
# network: physical link geometry vs transmitted payload
# ==========================================================================

def _net(n=3, **overrides):
    cfg = MissionConfig(num_drones=n, comm_latency_steps=0, comm_dropout_base=0.0,
                        comm_dropout_at_max_range=0.0, communication_radius=15.0, **overrides)
    return CommsNetwork(cfg, n, np.random.default_rng(0))


class TestNetworkPayload:
    POS = np.array([[0.0, 0.0, 3.0], [5.0, 0.0, 3.0], [10.0, 0.0, 3.0]])
    VEL = np.zeros((3, 3))

    def test_default_payload_is_the_positions_passed_in(self):
        net = _net()
        net.tick(self.POS, self.VEL)
        assert np.array_equal(net.last_seen[1][0]["pos"], self.POS[0])
        assert net.last_seen[1][0]["dist"] == pytest.approx(5.0)

    def test_payload_positions_are_what_neighbours_receive_while_dist_stays_physical(self):
        net = _net()
        est = self.POS + np.array([[100.0, 0, 0], [-40.0, 3.0, 0], [7.0, 7.0, 0]])
        net.tick(self.POS, self.VEL, payload_positions=est, payload_velocities=self.VEL + 0.5)
        entry = net.last_seen[1][0]
        assert np.array_equal(entry["pos"], est[0])
        assert np.array_equal(entry["vel"], self.VEL[0] + 0.5)
        assert entry["dist"] == pytest.approx(5.0)          # physical link range, not |est_1 - est_0|

    def test_link_existence_follows_true_positions_not_the_payload(self):
        net = _net()
        far_true = np.array([[0.0, 0.0, 3.0], [30.0, 0.0, 3.0]])
        est_close = np.array([[0.0, 0.0, 3.0], [1.0, 0.0, 3.0]])
        net2 = _net(2)
        net2.tick(far_true, np.zeros((2, 3)), payload_positions=est_close)
        assert net2.last_seen[1] == {} and net2.last_seen[0] == {}    # out of radio range physically
        close_true = np.array([[0.0, 0.0, 3.0], [5.0, 0.0, 3.0]])
        est_far = np.array([[0.0, 0.0, 3.0], [500.0, 0.0, 3.0]])
        net3 = _net(2)
        net3.tick(close_true, np.zeros((2, 3)), payload_positions=est_far)
        assert 0 in net3.last_seen[1]                                  # in radio range physically

    def test_payload_is_copied_not_aliased(self):
        net = _net()
        est = self.POS.copy()
        net.tick(self.POS, self.VEL, payload_positions=est)
        est[0, 0] = -999.0
        assert net.last_seen[1][0]["pos"][0] == 0.0

    def test_send_message_uses_cached_link_geometry_when_no_positions_given(self):
        net = _net()
        net.tick(self.POS, self.VEL)
        net.send_message(0, "hello")
        net.pump_messages()
        assert net.poll_inbox(1) == [(0, "hello")] and net.poll_inbox(2) == [(0, "hello")]

    def test_cached_and_explicit_positions_agree(self):
        a, b = _net(), _net()
        a.tick(self.POS, self.VEL)
        b.tick(self.POS, self.VEL)
        a.send_message(0, "m")
        b.send_message(0, "m", self.POS)
        a.pump_messages()
        b.pump_messages()
        assert a.poll_inbox(1) == b.poll_inbox(1) and a.poll_inbox(2) == b.poll_inbox(2)

    def test_send_message_without_positions_before_any_tick_raises(self):
        with pytest.raises(ValueError, match="prior tick"):
            _net().send_message(0, "hello")

    def test_message_range_gating_follows_the_cached_true_positions(self):
        net = _net()
        pos = np.array([[0.0, 0.0, 3.0], [5.0, 0.0, 3.0], [60.0, 0.0, 3.0]])
        net.tick(pos, np.zeros((3, 3)))
        net.send_message(0, "hello")
        net.pump_messages()
        assert net.poll_inbox(1) == [(0, "hello")] and net.poll_inbox(2) == []


# ==========================================================================
# sensors: body-relative measurements placed in the drone's own frame
# ==========================================================================

def _victim_model(seed=5, victims=((5.0, 3.0), (2.0, 6.0)), **overrides):
    defaults = dict(victim_sensor_noise_std_m=0.4, victim_sensor_false_negative_prob=0.0,
                    victim_sensor_false_positive_rate=0.0, victim_sensor_dropout_prob=0.0,
                    victim_sensor_latency_steps=0, victim_sensor_range_m=20.0, victim_sensor_hfov_deg=360.0,
                    victim_sensor_vfov_deg=170.0, num_obstacles=0)
    defaults.update(overrides)
    cfg = MissionConfig(**defaults)
    return VictimSensorModel(cfg, 1, np.array(victims, dtype=float), np.zeros((0, 2)), np.random.default_rng(seed))


DRONE = np.array([2.0, 3.0, 3.0])
HEADING = np.array([1.0, 0.0])


class TestVictimSensorPlacement:
    def test_legacy_call_is_unchanged(self):
        a, b = _victim_model(), _victim_model()
        d1, _ = a.sense(0, DRONE, HEADING, 0.0, [False, False])
        d2, _ = b.sense(0, DRONE, HEADING, 0.0, [False, False], own_estimated_xy=None, own_yaw_error_rad=0.0)
        assert [c.position_m for c in d1] == [c.position_m for c in d2]

    def test_noise_free_detection_is_placed_relative_to_the_estimate(self):
        model = _victim_model(victim_sensor_noise_std_m=0.0, victims=((5.0, 3.0),))
        est_xy, theta = np.array([12.0, 13.0]), 0.5
        (det,), _ = model.sense(0, DRONE, HEADING, 0.0, [False], own_estimated_xy=est_xy, own_yaw_error_rad=theta)
        rel = np.array([3.0, 0.0])                                       # true victim - true drone
        expected = est_xy + np.array([math.cos(theta) * rel[0] - math.sin(theta) * rel[1],
                                      math.sin(theta) * rel[0] + math.cos(theta) * rel[1]])
        assert det.position_m == pytest.approx(tuple(expected))

    def test_estimated_output_is_the_legacy_output_rebased_onto_the_estimate(self):
        a, b = _victim_model(seed=9), _victim_model(seed=9)          # identical RNG streams
        est_xy, theta = np.array([-7.0, 4.0]), -0.3
        d_legacy, _ = a.sense(0, DRONE, HEADING, 0.0, [False, False])
        d_est, _ = b.sense(0, DRONE, HEADING, 0.0, [False, False], own_estimated_xy=est_xy, own_yaw_error_rad=theta)
        assert len(d_legacy) == len(d_est) == 2
        for legacy, est in zip(d_legacy, d_est):
            expected = est_xy + rotate_xy(np.array(legacy.position_m) - DRONE[:2], theta)
            assert est.position_m == pytest.approx(tuple(expected))

    def test_a_perfect_estimate_reproduces_the_legacy_detection(self):
        a, b = _victim_model(seed=3), _victim_model(seed=3)
        d_legacy, _ = a.sense(0, DRONE, HEADING, 0.0, [False, False])
        d_est, _ = b.sense(0, DRONE, HEADING, 0.0, [False, False], own_estimated_xy=DRONE[:2], own_yaw_error_rad=0.0)
        for legacy, est in zip(d_legacy, d_est):
            assert est.position_m == pytest.approx(legacy.position_m, abs=1e-12)

    def test_drift_translates_the_geotag_by_exactly_the_drift(self):
        a, b = _victim_model(seed=4), _victim_model(seed=4)
        shifted = DRONE[:2] + np.array([2.5, -1.5])
        d0, _ = a.sense(0, DRONE, HEADING, 0.0, [False, False], own_estimated_xy=DRONE[:2], own_yaw_error_rad=0.0)
        d1, _ = b.sense(0, DRONE, HEADING, 0.0, [False, False], own_estimated_xy=shifted, own_yaw_error_rad=0.0)
        for p0, p1 in zip(d0, d1):
            assert np.array(p1.position_m) - np.array(p0.position_m) == pytest.approx([2.5, -1.5])

    def test_phantom_false_positives_are_placed_the_same_way(self):
        kw = dict(victims=(), victim_sensor_false_positive_rate=1.0)
        a, b = _victim_model(seed=6, **kw), _victim_model(seed=6, **kw)
        est_xy, theta = np.array([30.0, -20.0]), 0.8
        (legacy,), _ = a.sense(0, DRONE, HEADING, 0.0, [])
        (est,), _ = b.sense(0, DRONE, HEADING, 0.0, [], own_estimated_xy=est_xy, own_yaw_error_rad=theta)
        expected = est_xy + rotate_xy(np.array(legacy.position_m) - DRONE[:2], theta)
        assert est.position_m == pytest.approx(tuple(expected))


def _neighbor_model(**overrides):
    defaults = dict(neighbor_sensor_noise_std_m=0.0, neighbor_sensor_velocity_noise_std_mps=0.0,
                    neighbor_sensor_dropout_prob=0.0, neighbor_sensor_latency_steps=0,
                    neighbor_sensor_range_m=50.0, neighbor_sensor_fov_deg=360.0)
    defaults.update(overrides)
    return NeighborSensorModel(MissionConfig(num_drones=2, **defaults), 2, np.random.default_rng(1))


POS2 = np.array([[1.0, 2.0, 3.0], [4.0, 6.0, 3.5]])
VEL2 = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.25]])
HEAD2 = np.array([[1.0, 0.0, 0.0], [1.0, 0.0, 0.0]])


class TestNeighborSensorPlacement:
    def test_legacy_output_is_the_absolute_truth(self):
        m = _neighbor_model()
        m.tick(POS2, VEL2, HEAD2)
        (obs,) = m.observations_for(0, 1 / 24)
        assert obs.measured_position_m == pytest.approx((4.0, 6.0, 3.5))
        assert obs.measured_velocity_mps == pytest.approx((1.0, 0.0, 0.25))

    def test_estimated_output_is_the_relative_vector_placed_at_the_observers_estimate(self):
        m = _neighbor_model()
        est = np.array([[50.0, -20.0, 3.1], [0.0, 0.0, 0.0]])
        yaw_err = np.array([0.4, 0.0])
        m.tick(POS2, VEL2, HEAD2, own_estimated_positions=est, own_yaw_errors=yaw_err)
        (obs,) = m.observations_for(0, 1 / 24)
        rel = POS2[1] - POS2[0]
        assert obs.measured_position_m == pytest.approx(tuple(est[0] + rotate_xy(rel, 0.4)))
        assert obs.measured_velocity_mps == pytest.approx(tuple(rotate_xy(VEL2[1], 0.4)))

    def test_an_observers_own_drift_cancels_out_of_the_relative_geometry(self):
        # separation is what the safety supervisor uses: |measured - own_estimate| must equal the true
        # separation whatever the observer's position error or heading error is.
        for est_offset, yaw_err in [((0.0, 0.0), 0.0), ((13.0, -8.0), 0.9), ((-40.0, 25.0), -2.0)]:
            m = _neighbor_model()
            est = POS2.copy()
            est[0, :2] += est_offset
            m.tick(POS2, VEL2, HEAD2, own_estimated_positions=est, own_yaw_errors=np.array([yaw_err, 0.0]))
            (obs,) = m.observations_for(0, 1 / 24)
            separation = np.linalg.norm(np.array(obs.measured_position_m) - est[0])
            assert separation == pytest.approx(np.linalg.norm(POS2[1] - POS2[0]))

    def test_missing_yaw_errors_default_to_zero(self):
        m = _neighbor_model()
        est = np.array([[9.0, 9.0, 3.0], [0.0, 0.0, 0.0]])
        m.tick(POS2, VEL2, HEAD2, own_estimated_positions=est)
        (obs,) = m.observations_for(0, 1 / 24)
        assert obs.measured_position_m == pytest.approx(tuple(est[0] + (POS2[1] - POS2[0])))

    def test_consuming_no_extra_random_draws(self):
        # same seed, with and without placement: the noise stream is consumed identically.
        a, b = _neighbor_model(neighbor_sensor_noise_std_m=0.3), _neighbor_model(neighbor_sensor_noise_std_m=0.3)
        for _ in range(4):
            a.tick(POS2, VEL2, HEAD2)
            b.tick(POS2, VEL2, HEAD2, own_estimated_positions=POS2, own_yaw_errors=np.zeros(2))
        assert a.rng.random() == b.rng.random()


# ==========================================================================
# safety supervisor: covariance-aware geofence
# ==========================================================================

GEOFENCE = GeofenceSpec(frame=Frame.LOCAL_ENU, center_m=(0.0, 0.0), half_extents_m=(20.0, 20.0),
                        floor_alt_m=0.5, ceiling_alt_m=10.0)


def _state(x):
    return VehicleState(
        vehicle_id="drone0", sim_time_s=1.0, frame=Frame.LOCAL_ENU, position_m=(x, 0.0, 3.0),
        velocity_mps=(0.0, 0.0, 0.0), acceleration_mps2=(0.0, 0.0, 0.0), attitude_rad=(0.0, 0.0, 0.0),
        angular_velocity_radps=(0.0, 0.0, 0.0), battery_fraction=0.9, health_state=HealthState.OK,
        estimator_valid=True, last_valid_command_time_s=1.0,
    )


def _obs(sigma):
    return SensorObservation(
        vehicle_id="drone0", sensor_timestamp_s=1.0, sensor_latency_s=0.0, fov_deg=270.0,
        range_returns_m=(math.inf,) * 16, occluded=(False,) * 16, dropout=False, pose_uncertainty_m=sigma,
        detections=(),
    )


def _decide(x, sigma, **cfg):
    sup = SafetySupervisor(SafetySupervisorConfig(**cfg))
    cand = CandidateCommand(vehicle_id="drone0", desired_velocity_mps=(1.0, 0.0, 0.0), frame=Frame.LOCAL_ENU,
                            timestamp_s=1.0, expiration_time_s=2.0)
    return sup.evaluate(cand, _state(x), _obs(sigma), (), MissionContext(geofence=GEOFENCE), now_s=1.0)


class TestCovarianceAwareGeofence:
    def test_default_ignores_pose_uncertainty_entirely(self):
        legacy = _decide(17.0, 0.0)
        with_sigma = _decide(17.0, 5.0)              # margin 3.0 m > geofence_margin_m 2.0 -> not at risk
        assert legacy.emergency_state is not SafetyState.GEOFENCE_RISK
        assert with_sigma.emergency_state is legacy.emergency_state
        assert with_sigma.uncertainty_margin_m is None and with_sigma.reason == legacy.reason
        assert with_sigma.filtered_command == legacy.filtered_command

    def test_k_zero_is_identical_at_the_boundary_too(self):
        for x in (17.9, 18.5, 19.9):
            a, b = _decide(x, 0.0), _decide(x, 4.0, pose_sigma_geofence_k=0.0)
            assert (a.reason, a.emergency_state, a.filtered_command, a.geofence_distance_m) == \
                   (b.reason, b.emergency_state, b.filtered_command, b.geofence_distance_m)

    def test_uncertainty_widens_the_boundary_response(self):
        d = _decide(17.0, 2.0, pose_sigma_geofence_k=1.0)          # 3.0 - 2.0 = 1.0 < 2.0 -> risk
        assert d.emergency_state is SafetyState.GEOFENCE_RISK and d.accepted
        assert d.uncertainty_margin_m == pytest.approx(2.0)
        assert d.geofence_distance_m == pytest.approx(3.0)          # the raw estimated distance is still reported
        assert "uncertainty allowance" in d.reason
        assert d.filtered_command.desired_velocity_mps[0] < 0.0     # pushed inward

    def test_no_response_when_the_widened_margin_is_still_comfortable(self):
        d = _decide(10.0, 2.0, pose_sigma_geofence_k=1.0)
        assert d.emergency_state is not SafetyState.GEOFENCE_RISK

    def test_the_inflation_is_capped(self):
        # uncapped, 10 * 5 = 50 m would treat the whole arena as boundary; the cap holds it to 3 m
        assert _decide(14.9, 5.0, pose_sigma_geofence_k=10.0, pose_sigma_cap_m=3.0).emergency_state \
            is not SafetyState.GEOFENCE_RISK                          # 5.1 - 3.0 = 2.1 >= 2.0
        d = _decide(15.1, 5.0, pose_sigma_geofence_k=10.0, pose_sigma_cap_m=3.0)   # 4.9 - 3.0 = 1.9 < 2.0
        assert d.emergency_state is SafetyState.GEOFENCE_RISK and d.uncertainty_margin_m == pytest.approx(3.0)

    def test_already_outside_is_not_inflated(self):
        # an uncertain estimate that says "outside" is treated exactly like a certain one: hold.
        with_k = _decide(21.0, 2.0, pose_sigma_geofence_k=1.0)
        without = _decide(21.0, 2.0)
        assert with_k.reason == without.reason and "outside geofence" in with_k.reason
        assert with_k.uncertainty_margin_m is None

    def test_uncertainty_alone_never_turns_a_safely_inside_estimate_into_an_outside_hold(self):
        d = _decide(19.0, 30.0, pose_sigma_geofence_k=3.0, pose_sigma_cap_m=3.0)   # margin 1.0 m, inflation capped at 3
        assert "outside geofence" not in d.reason and d.emergency_state is SafetyState.GEOFENCE_RISK
        assert d.filtered_command.desired_velocity_mps[0] < 0.0    # inward push, not a permanent hold

    def test_zero_uncertainty_adds_nothing_even_with_k_set(self):
        a, b = _decide(17.0, 0.0), _decide(17.0, 0.0, pose_sigma_geofence_k=3.0)
        assert a.emergency_state is b.emergency_state and b.uncertainty_margin_m is None


# ==========================================================================
# config knobs
# ==========================================================================

class TestConfigDefaults:
    def test_defaults_preserve_the_legacy_pipeline(self):
        cfg = MissionConfig()
        assert cfg.localization_mode == "truth_state"
        assert cfg.safety_pose_sigma_geofence_k == 0.0
        assert cfg.estimator_assumed_noise_scale == 1.0 and cfg.estimator_init_pos_sigma_m is None
        assert cfg.est_command_frame_mapping is True and cfg.estimator_profile == "fused"

    def test_supervisor_config_defaults_match(self):
        assert SafetySupervisorConfig().pose_sigma_geofence_k == 0.0
        assert SafetySupervisorConfig().pose_sigma_cap_m == 3.0
