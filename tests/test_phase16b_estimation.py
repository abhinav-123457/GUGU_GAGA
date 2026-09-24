"""Phase 16B: odometry drift, the covariance-carrying estimator, and the
bridge to existing contract types - see docs/PHASE16B_ESTIMATION.md. Pure
numpy: no PyBullet, no mission. The mission-level wiring is covered in
test_phase16b_mission.py."""
import dataclasses
import functools
import math

import numpy as np
import pytest

from swarm_sim.contracts import Frame, HealthState
from swarm_sim.estimation import (
    PROFILES, DriftProfile, EstimatedState, OdometryBundle, OdometryMeasurement, SensorDriftSpec,
    StateEstimator, get_profile, health_state_of, pose_uncertainty_m, to_vehicle_state,
)
from swarm_sim.estimation.metrics import NEES_95_2DOF, LocalizationScorer
from swarm_sim.estimation.plant_odometry import OdometrySuite
from swarm_sim.mapping import GridSpec, MissionArea
from swarm_sim.plant_truth import PlantTruth
from swarm_sim.seeding import SeedManager

DT = 1.0 / 24.0


def _truth(t, pos, vel, yaw=0.0):
    return PlantTruth(t=t, positions=np.array([pos], dtype=float), velocities=np.array([vel], dtype=float),
                      rpys=np.array([[0.0, 0.0, yaw]]))


def _path(steps, speed=1.0, turn_rate=0.05, z=3.0, start=(0.0, 0.0), dt=DT):
    """Constant-speed path - the velocity direction turns at `turn_rate` (0 = straight line). The body
    yaw is supplied separately (`_yaw_at`): the real simulator holds it near 0 but it wobbles."""
    pos = np.array([start[0], start[1], z])
    out = []
    for k in range(steps):
        heading = turn_rate * k * dt
        vel = np.array([speed * math.cos(heading), speed * math.sin(heading), 0.0])
        pos = pos + vel * dt
        out.append(((k + 1) * dt, pos.copy(), vel.copy()))
    return out


def _yaw_at(t, yaw_amp, yaw_freq=1.5):
    return yaw_amp * math.sin(yaw_freq * t)


def _run(profile, seed, steps, speed=1.0, turn_rate=0.05, with_offsets=True, dt=DT, yaw_amp=0.0):
    """Simulate one drone; returns (estimator, scorer-friendly list of (EstimatedState, true_xyz, true_yaw))."""
    seeds = SeedManager(seed)
    suite = OdometrySuite(profile, 1, seeds)
    start_xyz = np.array([0.0, 0.0, 3.0])
    dx, dy, dyaw = suite.initial_offsets(0) if with_offsets else (0.0, 0.0, 0.0)
    est = StateEstimator("drone0", profile, (start_xyz[0] + dx, start_xyz[1] + dy, start_xyz[2]), 0.0 + dyaw)
    suite.start(0, start_xyz, 0.0)
    trace = []
    for t, pos, vel in _path(steps, speed, turn_rate, dt=dt):
        yaw = _yaw_at(t, yaw_amp)
        bundle = suite.measure(0, _truth(t, pos, vel, yaw=yaw), dt)
        trace.append((est.step(bundle), pos.copy(), yaw))
    return est, suite, trace


def _err(state, true_xyz):
    return math.hypot(state.position_m[0] - true_xyz[0], state.position_m[1] - true_xyz[1])


def _nees(state, true_xyz):
    e = np.array([state.position_m[0] - true_xyz[0], state.position_m[1] - true_xyz[1]])
    sxx, sxy, syy = state.pos_cov_xy_m2
    return float(e @ np.linalg.solve(np.array([[sxx, sxy], [sxy, syy]]), e))


def _quiet(profile: DriftProfile) -> DriftProfile:
    """The profile with dropouts and degenerate ticks switched off - the case the EKF's own model matches exactly."""
    sensors = tuple(dataclasses.replace(s, dropout_prob=0.0, degeneracy_prob=0.0) for s in profile.sensors)
    return dataclasses.replace(profile, sensors=sensors)


# ==========================================================================
# profiles
# ==========================================================================

class TestProfiles:
    def test_presets_exist_and_are_valid(self):
        assert set(PROFILES) == {"ideal", "vio", "flow_rf", "lidar", "fused", "stress"}
        assert [s.sensor_id for s in PROFILES["fused"].sensors] == ["vio", "flow_rf", "lidar"]
        for name in PROFILES:
            assert get_profile(name) is PROFILES[name]

    def test_unknown_profile(self):
        with pytest.raises(ValueError, match="unknown estimator profile"):
            get_profile("gps")

    def test_ideal_profile_has_no_error_terms(self):
        p = PROFILES["ideal"]
        assert p.rangefinder_sigma_m == p.attitude_sigma_rad == p.init_pos_sigma_m == p.init_yaw_sigma_rad == 0.0
        s = p.sensors[0]
        assert (s.vel_noise_std_mps, s.scale_error_rw_per_sqrt_s, s.vel_bias_rw_mps_per_sqrt_s,
                s.yaw_bias_rw_radps_per_sqrt_s, s.dropout_prob, s.degeneracy_prob) == (0, 0, 0, 0, 0, 0)
        assert s.vz_noise_std_mps == 0.0

    @pytest.mark.parametrize("bad", [
        dict(vel_noise_std_mps=-1.0), dict(dropout_prob=1.5), dict(degeneracy_drift_multiplier=0.5),
        dict(sensor_id=""), dict(scale_error_rw_per_sqrt_s=float("nan")), dict(vz_noise_std_mps=-0.01),
    ])
    def test_sensor_spec_validation(self, bad):
        base = dataclasses.asdict(PROFILES["vio"].sensors[0])
        base.update(bad)
        with pytest.raises(ValueError):
            SensorDriftSpec(**base)

    @pytest.mark.parametrize("bad", [
        dict(sensors=()), dict(assumed_noise_scale=0.0), dict(sigma_invalid_m=0.5), dict(odometry_timeout_s=0.0),
        dict(rangefinder_sigma_m=-1.0),
    ])
    def test_profile_validation(self, bad):
        base = dict(name="x", sensors=PROFILES["vio"].sensors, rangefinder_sigma_m=0.02, attitude_sigma_rad=0.005,
                    init_pos_sigma_m=0.05, init_yaw_sigma_rad=0.01)
        base.update(bad)
        with pytest.raises(ValueError):
            DriftProfile(**base)

    def test_duplicate_sensor_ids_rejected(self):
        s = PROFILES["vio"].sensors[0]
        with pytest.raises(ValueError, match="duplicate"):
            DriftProfile("x", (s, s), 0.02, 0.005, 0.05, 0.01)

    def test_with_helpers_do_not_mutate(self):
        p = PROFILES["vio"]
        assert p.with_assumed_noise_scale(0.3).assumed_noise_scale == 0.3 and p.assumed_noise_scale == 1.0
        assert p.with_init_pos_sigma(2.0).init_pos_sigma_m == 2.0 and p.init_pos_sigma_m == 0.05
        assert p.to_dict()["name"] == "vio"


# ==========================================================================
# the ideal profile telescopes to exact truth
# ==========================================================================

class TestIdealProfile:
    def test_estimate_equals_truth_over_a_long_curved_path(self):
        est, _, trace = _run(PROFILES["ideal"], seed=3, steps=3000)
        worst = max(_err(s, xyz) for s, xyz, _ in trace)
        assert worst < 1e-9
        assert max(abs(s.position_m[2] - xyz[2]) for s, xyz, _ in trace) < 1e-6
        assert all(s.valid and not s.degraded for s, _, _ in trace)
        assert abs(trace[-1][0].attitude_rad[2]) < 1e-12

    def test_ideal_velocity_matches_truth(self):
        _, _, trace = _run(PROFILES["ideal"], seed=3, steps=400)
        state, xyz, _ = trace[-1]
        heading = 0.05 * 399 * DT
        assert state.velocity_mps[0] == pytest.approx(math.cos(heading), abs=1e-9)
        assert state.velocity_mps[1] == pytest.approx(math.sin(heading), abs=1e-9)

    def test_ideal_covariance_is_negligible(self):
        _, _, trace = _run(PROFILES["ideal"], seed=3, steps=500)
        assert pose_uncertainty_m(trace[-1][0]) < 1e-3


# ==========================================================================
# determinism and drift behaviour
# ==========================================================================

@functools.lru_cache(maxsize=None)
def _mean_errors(profile: DriftProfile, n_seeds: int = 16, steps: int = 800):
    """(mean error after 100 ticks, mean error after `steps` ticks) at the simulator's 24 Hz."""
    short, long_ = [], []
    for seed in range(1, n_seeds + 1):
        trace = _run(profile, seed, steps)[2]
        short.append(_err(trace[99][0], trace[99][1]))
        long_.append(_err(trace[-1][0], trace[-1][1]))
    return float(np.mean(short)), float(np.mean(long_))


class TestDriftBehaviour:
    def test_same_seed_identical_trace_and_different_seed_differs(self):
        a = [s.position_m for s, _, _ in _run(PROFILES["fused"], 7, 300)[2]]
        b = [s.position_m for s, _, _ in _run(PROFILES["fused"], 7, 300)[2]]
        c = [s.position_m for s, _, _ in _run(PROFILES["fused"], 8, 300)[2]]
        assert a == b
        assert a != c

    @pytest.mark.parametrize("name", ["vio", "flow_rf", "lidar", "fused"])
    def test_error_grows_with_path_length(self, name):
        short, long_ = _mean_errors(PROFILES[name])
        assert long_ > 2.0 * short

    def test_uncertainty_grows_faster_than_linearly_along_a_straight_path(self):
        # straight line: scale-factor and bias errors all point along/across the same axis and add up
        # (on a curved path they partly cancel as the heading turns).
        trace = _run(_quiet(PROFILES["vio"]), 1, 2400, turn_rate=0.0)[2]
        s_half, s_full = pose_uncertainty_m(trace[1199][0]), pose_uncertainty_m(trace[2399][0])
        assert s_full / s_half > 2.0

    def test_profile_ordering_lidar_beats_vio_beats_flow(self):
        lidar, vio, flow = (_mean_errors(PROFILES[n])[1] for n in ("lidar", "vio", "flow_rf"))
        assert lidar < vio < flow

    def test_fusing_independent_sensors_beats_the_average_single_sensor(self):
        singles = [_mean_errors(PROFILES[n])[1] for n in ("vio", "flow_rf", "lidar")]
        assert _mean_errors(PROFILES["fused"])[1] < float(np.mean(singles))

    def test_hidden_state_is_not_visible_in_the_estimated_state(self):
        est, suite, trace = _run(PROFILES["vio"], 5, 600)
        scale, bias, yaw_bias = suite.hidden_state(0, "vio")
        assert scale != 0.0 and yaw_bias != 0.0
        # the estimator's own mean of the bias/scale/yaw-bias states never learns them without a fix
        assert np.allclose(est.state_vector[3:], 0.0)


# ==========================================================================
# consistency (NEES): the covariance must match the error actually made
# ==========================================================================

@functools.lru_cache(maxsize=None)
def _mean_nees(profile: DriftProfile, n_runs: int = 60, steps: int = 300, dt: float = 0.25,
               yaw_amp: float = 0.0) -> float:
    """Mean NEES at the end of `steps` ticks of `dt` seconds (75 s / 75 m by default), over n_runs seeds.
    A coarse dt keeps the test fast; the estimator is dt-agnostic."""
    vals = []
    for seed in range(1, n_runs + 1):
        trace = _run(profile, seed, steps, dt=dt, yaw_amp=yaw_amp)[2]
        vals.append(_nees(trace[-1][0], trace[-1][1]))
    return float(np.mean(vals))


class TestConsistency:
    # E[NEES] = 2 for a consistent 2-dof estimator; the mean of 60 independent samples has std ~0.26.
    def test_matched_model_is_consistent(self):
        assert 1.4 < _mean_nees(_quiet(PROFILES["vio"])) < 2.7

    def test_matched_fused_model_is_consistent(self):
        assert 1.4 < _mean_nees(_quiet(PROFILES["fused"])) < 2.7

    def test_overconfident_estimator_is_flagged(self):
        assert _mean_nees(_quiet(PROFILES["vio"]).with_assumed_noise_scale(0.3)) > 5.0

    def test_conservative_estimator_is_flagged_the_other_way(self):
        assert _mean_nees(_quiet(PROFILES["vio"]).with_assumed_noise_scale(3.0)) < 1.0

    def test_unmodelled_dropouts_and_degeneracy_make_the_stress_profile_overconfident(self):
        assert _mean_nees(PROFILES["stress"]) > 3.0

    def test_a_wobbling_heading_with_dropouts_stays_consistent(self):
        # The real simulator's drones wobble in yaw by ~0.2 rad; a dropout must not lose that heading change.
        # (A constant-yaw path cannot expose the bug where a dropout tick discarded the heading rate.)
        dropouts_only = dataclasses.replace(
            PROFILES["vio"], sensors=tuple(dataclasses.replace(s, dropout_prob=0.10, degeneracy_prob=0.0)
                                           for s in PROFILES["vio"].sensors))
        # measured ~0.9: the dropout-gap covariance inflation makes it slightly conservative, never over-confident
        assert 0.3 < _mean_nees(dropouts_only, yaw_amp=0.3) < 3.0


# ==========================================================================
# validity, degraded, dropout gaps
# ==========================================================================

def _bundle(t, valid=True, sensor_id="vio", v=(1.0, 0.0), z=3.0):
    dp = (v[0] * DT, v[1] * DT)
    m = (OdometryMeasurement(sensor_id, True, dp, v, 0.0, 0.0) if valid else OdometryMeasurement(sensor_id, False))
    return OdometryBundle(t=t, dt=DT, measurements=(m,), range_z_m=z, roll_pitch_rad=(0.0, 0.0))


class TestHeadingSurvivesTranslationDropout:
    def test_heading_keeps_integrating_when_translation_is_lost(self):
        est = StateEstimator("d", dataclasses.replace(PROFILES["ideal"], odometry_timeout_s=60.0), (0.0, 0.0, 3.0))
        for k in range(10):
            lost = OdometryMeasurement("ideal", False, dyaw_rad=0.1)
            est.step(OdometryBundle(t=DT * (k + 1), dt=DT, measurements=(lost,), range_z_m=3.0,
                                    roll_pitch_rad=(0.0, 0.0)))
        assert est.state().attitude_rad[2] == pytest.approx(1.0, abs=1e-9)

    def test_dropout_does_not_accumulate_heading_error_on_a_wobbling_path(self):
        base = _quiet(PROFILES["vio"])
        flaky = dataclasses.replace(base, sensors=tuple(dataclasses.replace(s, dropout_prob=0.30)
                                                        for s in base.sensors))
        errors = []
        for seed in range(1, 9):
            trace = _run(flaky, seed, 600, yaw_amp=0.5)[2]
            state, _, true_yaw = trace[-1]
            errors.append(abs(state.attitude_rad[2] - true_yaw))
        assert max(errors) < 0.05       # a discarded heading change per dropped tick would give ~0.4 rad

    def test_heading_only_measurements_do_not_refresh_the_translation_timeout(self):
        prof = dataclasses.replace(PROFILES["vio"], odometry_timeout_s=0.5)
        est = StateEstimator("d", prof, (0.0, 0.0, 3.0))
        last = None
        for k in range(30):
            last = est.step(OdometryBundle(t=DT * (k + 1), dt=DT,
                                           measurements=(OdometryMeasurement("vio", False, dyaw_rad=0.0),),
                                           range_z_m=3.0, roll_pitch_rad=(0.0, 0.0)))
        assert not last.valid and "no valid odometry" in last.reason


class TestValidity:
    def test_fresh_estimator_is_valid_and_not_degraded(self):
        est = StateEstimator("d", PROFILES["vio"], (0.0, 0.0, 3.0))
        s = est.step(_bundle(DT))
        assert s.valid and not s.degraded and s.reason == ""

    def test_odometry_timeout_invalidates(self):
        prof = dataclasses.replace(PROFILES["vio"], odometry_timeout_s=0.5)
        est = StateEstimator("d", prof, (0.0, 0.0, 3.0))
        states = [est.step(_bundle(DT * (k + 1), valid=False)) for k in range(30)]
        assert states[5].valid                       # 0.25 s without odometry - still inside the timeout
        assert not states[-1].valid and "no valid odometry" in states[-1].reason

    def test_recovery_after_a_short_gap(self):
        prof = dataclasses.replace(PROFILES["vio"], odometry_timeout_s=0.5)
        est = StateEstimator("d", prof, (0.0, 0.0, 3.0))
        for k in range(20):
            est.step(_bundle(DT * (k + 1), valid=False))
        s = est.step(_bundle(DT * 21, valid=True))
        assert s.valid and s.reason == "" and s.last_odometry_time_s == pytest.approx(DT * 21)

    def test_dropout_gap_extrapolates_position_and_inflates_covariance(self):
        est = StateEstimator("d", PROFILES["vio"], (0.0, 0.0, 3.0))
        est.step(_bundle(DT))
        before = est.step(_bundle(2 * DT))
        after = [est.step(_bundle((3 + k) * DT, valid=False)) for k in range(12)][-1]
        assert after.position_m[0] > before.position_m[0] + 0.3          # kept moving at the last velocity
        assert pose_uncertainty_m(after) > pose_uncertainty_m(before)

    def test_degraded_when_sigma_crosses_threshold_and_invalid_when_it_crosses_the_next(self):
        prof = dataclasses.replace(PROFILES["stress"], sigma_degraded_m=0.5, sigma_invalid_m=1.0,
                                   sensors=tuple(dataclasses.replace(s, dropout_prob=0.0) for s in PROFILES["stress"].sensors))
        seeds = SeedManager(2)
        suite = OdometrySuite(prof, 1, seeds)
        est = StateEstimator("d", prof, (0.0, 0.0, 3.0))
        suite.start(0, (0.0, 0.0, 3.0), 0.0)
        seen_ok = seen_degraded = seen_invalid = False
        for t, pos, vel in _path(4000):
            s = est.step(suite.measure(0, _truth(t, pos, vel), DT))
            seen_ok |= s.valid and not s.degraded
            seen_degraded |= s.valid and s.degraded
            if not s.valid:
                seen_invalid = True
                assert "exceeds invalid threshold" in s.reason
                break
        assert seen_ok and seen_degraded and seen_invalid

    def test_unknown_sensor_id_rejected(self):
        est = StateEstimator("d", PROFILES["vio"], (0.0, 0.0, 3.0))
        with pytest.raises(ValueError, match="unknown sensor id"):
            est.step(_bundle(DT, sensor_id="gps"))

    def test_bad_dt_rejected(self):
        est = StateEstimator("d", PROFILES["vio"], (0.0, 0.0, 3.0))
        bad = dataclasses.replace(_bundle(DT), dt=0.0)
        with pytest.raises(ValueError):
            est.step(bad)


# ==========================================================================
# external aiding (position fixes) - unit-tested here, wired in 16F
# ==========================================================================

class TestPositionFix:
    def _drifted(self, seed=4, steps=1500):
        est, _, trace = _run(PROFILES["vio"], seed, steps)
        return est, trace[-1][0], trace[-1][1]

    def test_fix_shrinks_uncertainty_and_pulls_the_estimate_towards_it(self):
        est, before, truth_xyz = self._drifted()
        result = est.apply_position_fix((truth_xyz[0], truth_xyz[1]), 0.05, t=before.t, source="fiducial")
        after = est.state()
        assert result.accepted
        assert pose_uncertainty_m(after) < 0.3 * pose_uncertainty_m(before)
        assert _err(after, truth_xyz) < _err(before, truth_xyz) + 1e-9
        assert _err(after, truth_xyz) < 0.15
        assert after.last_aiding_time_s == pytest.approx(before.t)

    def test_fix_also_reduces_heading_uncertainty_through_correlation(self):
        est, before, truth_xyz = self._drifted()
        est.apply_position_fix((truth_xyz[0], truth_xyz[1]), 0.05, t=before.t)
        assert est.state().sigma_yaw_rad < before.sigma_yaw_rad

    def test_covariance_stays_symmetric_and_psd_after_a_fix(self):
        est, before, truth_xyz = self._drifted()
        est.apply_position_fix((truth_xyz[0], truth_xyz[1]), 0.05, t=before.t)
        P = est.covariance
        assert np.allclose(P, P.T)
        assert np.linalg.eigvalsh(P).min() > -1e-9

    def test_gate_rejects_an_inconsistent_fix_and_leaves_the_belief_alone(self):
        est, before, truth_xyz = self._drifted()
        P_before = est.covariance
        result = est.apply_position_fix((truth_xyz[0] + 50.0, truth_xyz[1]), 0.05, t=before.t, gate_nis=NEES_95_2DOF)
        assert not result.accepted and result.nis > 100.0
        assert np.array_equal(est.covariance, P_before)
        assert est.state().last_aiding_time_s is None

    def test_matrix_noise_accepted_and_bad_input_rejected(self):
        est, before, truth_xyz = self._drifted()
        assert est.apply_position_fix((truth_xyz[0], truth_xyz[1]), np.diag([0.04, 0.09]), t=before.t).accepted
        with pytest.raises(ValueError):
            est.apply_position_fix((float("nan"), 0.0), 0.05, t=0.0)
        with pytest.raises(ValueError):
            est.apply_position_fix((0.0, 0.0), -1.0, t=0.0)
        with pytest.raises(ValueError):
            est.apply_position_fix((0.0, 0.0), np.eye(3), t=0.0)

    def test_after_a_fix_the_estimator_keeps_propagating_consistently(self):
        est, _, _ = self._drifted()
        seeds = SeedManager(4)
        # keep drifting for another 10 s of the same path: error stays bounded relative to sigma
        est2, suite, trace = _run(PROFILES["vio"], 4, 1500)
        last_t = trace[-1][0].t
        truth_xyz = trace[-1][1]
        est2.apply_position_fix((truth_xyz[0], truth_xyz[1]), 0.05, t=last_t)
        pos = truth_xyz.copy()
        for k in range(240):
            heading = 0.05 * (1500 + k) * DT
            vel = np.array([math.cos(heading), math.sin(heading), 0.0])
            pos = pos + vel * DT
            s = est2.step(suite.measure(0, _truth(last_t + (k + 1) * DT, pos, vel), DT))
        assert s.valid
        assert _err(s, pos) < 5.0 * max(pose_uncertainty_m(s), 0.05)


# ==========================================================================
# bridge to the existing contracts
# ==========================================================================

def _state(**kw):
    base = dict(t=1.0, position_m=(1.0, 2.0, 3.0), velocity_mps=(0.1, 0.2, 0.0), attitude_rad=(0.0, 0.0, 0.3),
                pos_cov_xy_m2=(0.04, 0.0, 0.09), sigma_z_m=0.02, sigma_yaw_rad=0.01, valid=True, degraded=False,
                reason="", last_odometry_time_s=1.0, last_aiding_time_s=None)
    base.update(kw)
    return EstimatedState(**base)


class TestBridge:
    def test_pose_uncertainty_is_the_worst_axis_sigma(self):
        assert pose_uncertainty_m(_state()) == pytest.approx(0.3)
        assert pose_uncertainty_m(_state(pos_cov_xy_m2=(0.25, 0.0, 0.25))) == pytest.approx(0.5)

    def test_pose_uncertainty_respects_correlation(self):
        # perfectly correlated x/y: eigenvalues 2*s and 0 -> worst-axis sigma sqrt(2 s)
        assert pose_uncertainty_m(_state(pos_cov_xy_m2=(0.1, 0.1, 0.1))) == pytest.approx(math.sqrt(0.2))

    def test_pose_uncertainty_never_negative_or_nan_for_a_slightly_indefinite_matrix(self):
        assert pose_uncertainty_m(_state(pos_cov_xy_m2=(1e-15, 2e-15, 1e-15))) >= 0.0

    def test_health_mapping_never_reports_failed(self):
        assert health_state_of(_state()) is HealthState.OK
        assert health_state_of(_state(degraded=True)) is HealthState.DEGRADED
        assert health_state_of(_state(valid=False, reason="x")) is HealthState.DEGRADED

    def test_to_vehicle_state_uses_the_estimate(self):
        vs = to_vehicle_state(_state(), "drone3", sim_time_s=2.0, battery_fraction=0.9)
        assert vs.vehicle_id == "drone3" and vs.frame is Frame.LOCAL_ENU
        assert vs.position_m == (1.0, 2.0, 3.0) and vs.velocity_mps == (0.1, 0.2, 0.0)
        assert vs.attitude_rad == (0.0, 0.0, 0.3) and vs.estimator_valid is True
        assert vs.battery_fraction == 0.9 and vs.last_valid_command_time_s == 2.0
        assert vs.health_state is HealthState.OK

    def test_invalid_estimate_becomes_estimator_valid_false(self):
        vs = to_vehicle_state(_state(valid=False, reason="x"), "d", 2.0)
        assert vs.estimator_valid is False and vs.health_state is HealthState.DEGRADED


# ==========================================================================
# plant-side odometry (truth side)
# ==========================================================================

class TestPlantOdometry:
    def test_rng_streams_are_named_per_sensor_and_drone(self):
        seeds = SeedManager(11)
        OdometrySuite(PROFILES["fused"], 2, seeds)
        names = set(seeds.derived_seeds())
        assert {"odometry/vio/drone0", "odometry/lidar/drone1", "odometry/imu_range/drone0",
                "odometry/init/drone1"} <= names

    def test_creating_the_suite_does_not_touch_existing_streams(self):
        seeds = SeedManager(11)
        before = seeds.rng("victim_sensor").random()
        seeds2 = SeedManager(11)
        OdometrySuite(PROFILES["fused"], 3, seeds2)
        assert seeds2.rng("victim_sensor").random() == before

    def test_fixed_draw_count_per_tick_regardless_of_dropout(self):
        always_drop = dataclasses.replace(PROFILES["vio"], sensors=(dataclasses.replace(PROFILES["vio"].sensors[0], dropout_prob=1.0),))
        never_drop = dataclasses.replace(PROFILES["vio"], sensors=(dataclasses.replace(PROFILES["vio"].sensors[0], dropout_prob=0.0),))
        s1, s2 = SeedManager(5), SeedManager(5)
        a, b = OdometrySuite(always_drop, 1, s1), OdometrySuite(never_drop, 1, s2)
        for k in range(5):
            a.measure(0, _truth(k * DT, (0, 0, 3), (1, 0, 0)), DT)
            b.measure(0, _truth(k * DT, (0, 0, 3), (1, 0, 0)), DT)
        assert s1.rng("odometry/vio/drone0").random() == s2.rng("odometry/vio/drone0").random()

    def test_dropout_loses_translation_but_keeps_the_gyro_heading_channel(self):
        prof = dataclasses.replace(PROFILES["vio"], sensors=(dataclasses.replace(PROFILES["vio"].sensors[0], dropout_prob=1.0),))
        suite = OdometrySuite(prof, 1, SeedManager(1))
        suite.start(0, (0, 0, 3), 0.0)
        m = suite.measure(0, _truth(DT, (0.04, 0, 3), (1, 0, 0), yaw=0.2), DT).measurements[0]
        assert not m.valid
        assert m.dp_body_xy_m is None and m.v_body_xy_mps is None and m.vz_mps is None
        assert m.dyaw_rad == pytest.approx(0.2, abs=0.01)          # the true 0.2 rad heading change, plus tiny noise

    def test_vertical_speed_noise_follows_its_own_density_not_the_horizontal_one(self):
        prof = PROFILES["stress"]
        spec = prof.sensors[0]
        assert spec.vz_noise_std_mps < spec.vel_noise_std_mps / 2          # the case this test is about
        no_dropout = dataclasses.replace(prof, sensors=(dataclasses.replace(spec, dropout_prob=0.0, degeneracy_prob=0.0),))
        suite = OdometrySuite(no_dropout, 1, SeedManager(3))
        suite.start(0, (0.0, 0.0, 3.0), 0.0)
        vz_err, vx_err = [], []
        for k in range(3000):
            m = suite.measure(0, _truth(k * DT, (0.0, 0.0, 3.0), (0.0, 0.0, 0.7)), DT).measurements[0]
            vz_err.append(m.vz_mps - 0.7)
            vx_err.append(m.v_body_xy_mps[0])       # true horizontal speed is 0, so this is pure error
        assert np.std(vz_err) == pytest.approx(spec.vz_noise_std_mps, rel=0.1)
        assert np.std(vx_err) > 2.0 * np.std(vz_err)

    def test_first_measurement_without_start_reports_no_motion(self):
        suite = OdometrySuite(PROFILES["ideal"], 1, SeedManager(1))
        m = suite.measure(0, _truth(0.0, (5.0, 5.0, 3.0), (1, 0, 0)), DT).measurements[0]
        assert m.dp_body_xy_m == (0.0, 0.0) and m.dyaw_rad == 0.0

    def test_ideal_displacement_is_exact_in_the_body_frame(self):
        suite = OdometrySuite(PROFILES["ideal"], 1, SeedManager(1))
        suite.start(0, (0.0, 0.0, 3.0), yaw_rad=math.pi / 2)
        m = suite.measure(0, _truth(DT, (0.0, 1.0, 3.0), (0, 24.0, 0), yaw=math.pi / 2), DT).measurements[0]
        assert m.dp_body_xy_m == pytest.approx((1.0, 0.0), abs=1e-12)     # moved +y while facing +y = forward
        assert m.v_body_xy_mps == pytest.approx((24.0, 0.0), abs=1e-12)

    def test_range_and_attitude_channels(self):
        suite = OdometrySuite(PROFILES["ideal"], 1, SeedManager(1))
        b = suite.measure(0, PlantTruth(0.0, np.array([[0, 0, 2.5]]), np.zeros((1, 3)), np.array([[0.1, -0.2, 0.0]])), DT)
        assert b.range_z_m == 2.5 and b.roll_pitch_rad == (0.1, -0.2)

    def test_initial_offsets_scale_with_the_profile_and_are_zero_for_ideal(self):
        assert OdometrySuite(PROFILES["ideal"], 1, SeedManager(1)).initial_offsets(0) == (0.0, 0.0, 0.0)
        offs = np.array([OdometrySuite(PROFILES["stress"], 1, SeedManager(s)).initial_offsets(0) for s in range(200)])
        assert offs[:, :2].std() == pytest.approx(PROFILES["stress"].init_pos_sigma_m, rel=0.2)
        assert offs[:, 2].std() == pytest.approx(PROFILES["stress"].init_yaw_sigma_rad, rel=0.25)


# ==========================================================================
# truth-side scorer
# ==========================================================================

class TestScorer:
    def test_perfect_estimate_scores_zero_error(self):
        _, _, trace = _run(PROFILES["ideal"], 1, 200)
        scorer = LocalizationScorer(1)
        for s, xyz, yaw in trace:
            scorer.record(0, s, xyz, yaw)
        out = scorer.summary()
        assert out["max_error_m"] < 1e-9 and out["mean_invalid_tick_fraction"] == 0.0
        assert out["per_drone"][0]["path_length_m"] == pytest.approx(199 * DT, rel=1e-6)

    def test_drift_fraction_and_cell_match(self):
        area = MissionArea(-50.0, -50.0, 50.0, 50.0)
        grid = GridSpec.from_area(area)
        _, _, trace = _run(PROFILES["vio"], 2, 1200)
        scorer = LocalizationScorer(1, grid=grid)
        for s, xyz, yaw in trace:
            scorer.record(0, s, xyz, yaw)
        out = scorer.summary()
        d = out["per_drone"][0]
        assert d["final_drift_fraction"] == pytest.approx(d["final_error_m"] / d["path_length_m"])
        assert 0.0 <= out["mean_cell_match_fraction"] <= 1.0
        assert d["max_error_m"] >= d["mean_error_m"] > 0.0

    def test_no_samples_summary_is_safe(self):
        out = LocalizationScorer(2).summary()
        assert out["mean_error_m"] is None and out["per_drone"] == [{"drone": 0, "samples": 0}, {"drone": 1, "samples": 0}]
