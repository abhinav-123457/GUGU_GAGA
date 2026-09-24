"""Phase 16B: FloodSearchMission in `estimated` localisation mode - what the
drones act on, what is scored against truth, and that the default
`truth_state` pipeline is untouched. See docs/PHASE16B_ESTIMATION.md.

These run real (short) PyBullet missions; keep them small."""
import re
from types import SimpleNamespace

import numpy as np
import pytest

from swarm_sim.config import MissionConfig
from swarm_sim.mission import FloodSearchMission, _AutonomyView
from swarm_sim.recruitment import RecruitmentBoard
from swarm_sim.sensors import rotate_xy

N = 3
TICKS_PER_S = 24


def _cfg(duration_sec=3.0, **kw):
    base = dict(num_drones=N, duration_sec=duration_sec, seed=11, num_victims=2, num_obstacles=1)
    base.update(kw)
    return MissionConfig(**base)


def _positions(result):
    """Truth positions per tick, shape (ticks, N, 3), from the telemetry log."""
    rows = [[r["x"], r["y"], r["z"]] for r in result["telemetry"].log]
    return np.array(rows).reshape(-1, N, 3)


@pytest.fixture(scope="module")
def truth_run():
    return FloodSearchMission(_cfg(localization_mode="truth_state")).run()


@pytest.fixture(scope="module")
def ideal_run():
    return FloodSearchMission(_cfg(localization_mode="estimated", estimator_profile="ideal")).run()


# ==========================================================================
# configuration
# ==========================================================================

class TestModeSelection:
    @pytest.mark.parametrize("bad", ["gps", "", "Estimated", None])
    def test_unknown_mode_is_rejected_before_anything_is_built(self, bad):
        with pytest.raises(ValueError, match="localization_mode"):
            FloodSearchMission(_cfg(localization_mode=bad))

    def test_unknown_profile_is_rejected(self):
        with pytest.raises(ValueError, match="unknown estimator profile"):
            FloodSearchMission(_cfg(localization_mode="estimated", estimator_profile="gps"))

    def test_the_default_is_the_legacy_pipeline(self, truth_run):
        assert truth_run["localization_mode"] == "truth_state"
        assert truth_run["estimator_profile"] is None and truth_run["localization"] is None
        assert truth_run["synthetic_beacon_count"] == 0

    def test_map_id_and_true_geofence_are_reported_in_both_modes(self, truth_run, ideal_run):
        for res in (truth_run, ideal_run):
            assert re.fullmatch(r"[0-9a-f]{12}", res["mission_map_id"])
            assert res["true_geofence"]["ticks"] == N * int(3.0 * TICKS_PER_S)
            assert res["true_geofence"]["ticks_outside"] == 0
        assert truth_run["mission_map_id"] == ideal_run["mission_map_id"]

    def test_estimator_streams_are_extra_named_streams_only(self):
        legacy = FloodSearchMission(_cfg(localization_mode="truth_state"))
        est = FloodSearchMission(_cfg(localization_mode="estimated", estimator_profile="vio"))
        legacy_streams, est_streams = legacy._sensor_seeds.derived_seeds(), est._sensor_seeds.derived_seeds()
        extra = set(est_streams) - set(legacy_streams)
        assert extra and all(name.startswith("odometry/") for name in extra)
        assert all(est_streams[name] == seed for name, seed in legacy_streams.items())   # existing streams unchanged

    def test_safety_config_reaches_the_supervisor(self):
        m = FloodSearchMission(_cfg(safety_pose_sigma_geofence_k=3.0, safety_pose_sigma_cap_m=2.5))
        assert m.safety.cfg.pose_sigma_geofence_k == 3.0 and m.safety.cfg.pose_sigma_cap_m == 2.5

    def test_geofence_comes_from_the_mission_map_with_the_legacy_numbers(self):
        m = FloodSearchMission(_cfg())
        g = m._geofence
        assert g.center_m == (0.0, 0.0) and g.half_extents_m == (20.0, 20.0)
        assert (g.floor_alt_m, g.ceiling_alt_m) == (0.5, 8.0)
        assert m._mission_map.area.width_m == 40.0 and m._mission_map.grid.n_cells == 1600


# ==========================================================================
# the ideal profile reproduces the legacy pipeline over a short horizon
# ==========================================================================

class TestIdealParity:
    def test_estimate_equals_truth_for_the_whole_run(self, ideal_run):
        assert ideal_run["localization"]["max_error_m"] < 1e-9
        assert ideal_run["localization"]["mean_invalid_tick_fraction"] == 0.0
        assert ideal_run["localization"]["mean_cell_match_fraction"] == 1.0

    def test_first_two_seconds_match_the_legacy_pipeline_to_1e6(self, truth_run, ideal_run):
        # Beyond a few seconds the swarm simulation amplifies float rounding (1e-14 in the estimate grows to
        # metres over ~10 s: flocking + safety switching are chaotic), so parity is a SHORT-horizon property.
        a, b = _positions(truth_run), _positions(ideal_run)
        horizon = 2 * TICKS_PER_S
        assert np.abs(a[:horizon] - b[:horizon]).max() < 1e-6

    def test_the_reference_for_drift_is_the_ideal_profile_not_truth_state(self, truth_run, ideal_run):
        # every confirmation becomes a beacon in estimated mode (a drone cannot know it is false), so
        # even ideal odometry is a different, more realistic swarm than the truth-gated legacy one.
        assert ideal_run["localization_mode"] == "estimated" and ideal_run["estimator_profile"] == "ideal"


# ==========================================================================
# drifting profiles: the drones act on the estimate
# ==========================================================================

class _Spy:
    """Records what the autonomy stack was actually given, per drone-tick."""

    def __init__(self, mission):
        self.states, self.sigmas = [], []
        orig_step, orig_eval = mission.controller.step, mission.safety.evaluate

        def step(i, own_state, *a, **k):
            self.states.append(own_state)
            return orig_step(i, own_state, *a, **k)

        def evaluate(candidate, own_state, sensor_observation, *a, **k):
            self.sigmas.append(sensor_observation.pose_uncertainty_m)
            return orig_eval(candidate, own_state, sensor_observation, *a, **k)

        mission.controller.step, mission.safety.evaluate = step, evaluate


def _run_with_spy(**kw):
    mission = FloodSearchMission(_cfg(duration_sec=6.0, **kw))
    spy = _Spy(mission)
    return mission.run(), spy


@pytest.fixture(scope="module")
def stress():
    return _run_with_spy(localization_mode="estimated", estimator_profile="stress")


class TestDriftingRun:
    def test_the_controller_gets_the_estimate_which_differs_from_truth(self, stress):
        result, spy = stress
        truth = _positions(result).reshape(-1, 3)
        seen = np.array([s.position_m for s in spy.states])
        assert seen.shape == truth.shape
        assert np.linalg.norm(seen[:, :2] - truth[:, :2], axis=1).max() > 0.05
        assert result["localization"]["max_error_m"] == pytest.approx(
            np.linalg.norm(seen[:, :2] - truth[:, :2], axis=1).max(), rel=1e-6)

    def test_the_supervisor_is_given_a_real_pose_uncertainty_that_grows(self, stress):
        _, spy = stress
        assert min(spy.sigmas) > 0.0
        per_drone_first, per_drone_last = spy.sigmas[:N], spy.sigmas[-N:]
        assert max(per_drone_last) > max(per_drone_first)

    def test_localisation_results_are_populated(self, stress):
        result, _ = stress
        loc = result["localization"]
        assert result["localization_mode"] == "estimated" and result["estimator_profile"] == "stress"
        assert loc["mean_error_m"] > 0.0 and len(loc["per_drone"]) == N
        assert loc["per_drone"][0]["samples"] == 6 * TICKS_PER_S
        assert loc["mean_nees"] is not None and loc["max_sigma_m"] > 0.0

    def test_vehicle_state_carries_estimator_validity_from_the_estimate(self, stress):
        _, spy = stress
        assert all(isinstance(s.estimator_valid, bool) for s in spy.states)
        assert all(s.attitude_rad[2] == s.attitude_rad[2] for s in spy.states)        # estimated yaw, finite

    def test_the_ideal_profile_hands_the_controller_exact_truth(self):
        result, spy = _run_with_spy(localization_mode="estimated", estimator_profile="ideal")
        truth = _positions(result).reshape(-1, 3)
        seen = np.array([s.position_m for s in spy.states])
        assert np.abs(seen - truth).max() < 1e-9
        assert set(spy.sigmas) and max(spy.sigmas) < 1e-3

    def test_legacy_mode_reports_zero_pose_uncertainty(self):
        _, spy = _run_with_spy(localization_mode="truth_state")
        assert set(spy.sigmas) == {0.0}


@pytest.fixture(scope="module")
def flow_rf_run():
    """flow_rf drift on the sweep's 4-drone configuration, seed 4, 14 s. With the vertical-speed noise
    at the horizontal 0.10 m/s density (the model before Phase 16B finding 7) this exact run climbed to
    10.3 m, past the 8 m ceiling, and had 129 ground-contact steps."""
    cfg = MissionConfig(num_drones=4, duration_sec=14.0, seed=4, num_victims=3, num_obstacles=2,
                        localization_mode="estimated", estimator_profile="flow_rf")
    return FloodSearchMission(cfg).run()


class TestAltitudeUnderDrift:
    """Phase 16B finding 7: noise on the estimated vertical speed is integrated into an altitude random
    walk by the supervisor's accel limiter plus the mission's altitude-setpoint bypass, so it must stay
    small (see SensorDriftSpec.vz_noise_std_mps)."""

    def test_drones_stay_in_a_sane_altitude_band_and_never_touch_the_ground_under_flow_rf_drift(self, flow_rf_run):
        z = np.array([r["z"] for r in flow_rf_run["telemetry"].log])
        assert flow_rf_run["contact_steps"] == 0
        assert 1.5 < z.min() and z.max() < 6.0, f"altitude left the band: [{z.min():.2f}, {z.max():.2f}] m from a 3 m cruise"


class TestDeterminismAndKnobs:
    def test_estimated_mode_is_deterministic(self):
        a = FloodSearchMission(_cfg(localization_mode="estimated", estimator_profile="fused")).run()
        b = FloodSearchMission(_cfg(localization_mode="estimated", estimator_profile="fused")).run()
        assert a["telemetry"].log == b["telemetry"].log
        assert a["localization"] == b["localization"]

    def test_different_seeds_give_different_drift(self):
        a = FloodSearchMission(_cfg(seed=1, localization_mode="estimated", estimator_profile="vio")).run()
        b = FloodSearchMission(_cfg(seed=2, localization_mode="estimated", estimator_profile="vio")).run()
        assert a["localization"]["mean_error_m"] != b["localization"]["mean_error_m"]

    def test_frame_mapping_off_still_runs(self):
        res = FloodSearchMission(_cfg(localization_mode="estimated", estimator_profile="vio",
                                      est_command_frame_mapping=False)).run()
        assert res["localization"]["num_drones"] == N and res["localization"]["mean_error_m"] is not None

    def test_over_confident_assumed_noise_scale_is_applied(self):
        m = FloodSearchMission(_cfg(localization_mode="estimated", estimator_profile="vio",
                                    estimator_assumed_noise_scale=0.25, estimator_init_pos_sigma_m=1.5))
        assert m._estimators[0].profile.assumed_noise_scale == 0.25
        assert m._estimators[0].profile.init_pos_sigma_m == 1.5

    def test_initial_estimate_is_offset_from_the_true_spawn_by_the_launch_slot_accuracy(self):
        m = FloodSearchMission(_cfg(localization_mode="estimated", estimator_profile="stress"))
        first = m._estimators[0].state()
        assert first.valid and (first.pos_cov_xy_m2[0] ** 0.5) > 0.05


# ==========================================================================
# frame mapping and beacon policy (unit level, no simulator)
# ==========================================================================

def _view(yaw_errors):
    z = np.zeros((2, 3))
    return _AutonomyView(positions=z, velocities=z, attitudes=z, states=None if yaw_errors is None else [],
                         yaw_errors=None if yaw_errors is None else np.array(yaw_errors, dtype=float))


class TestFrameMapping:
    def _stub(self, mapping=True):
        return SimpleNamespace(cfg=MissionConfig(est_command_frame_mapping=mapping))

    def test_truth_mode_and_zero_error_return_the_very_same_vector(self):
        v = np.array([1.0, 2.0, 3.0])
        stub = self._stub()
        assert FloodSearchMission._to_plant_frame(stub, v, _view(None), 0) is v
        assert FloodSearchMission._to_plant_frame(stub, v, _view([0.0, 0.0]), 0) is v

    def test_command_is_rotated_by_minus_the_heading_error(self):
        # a drone whose estimated heading is 0.3 rad too far counter-clockwise: its estimated-frame "east"
        # command really moves it 0.3 rad clockwise of east.
        out = FloodSearchMission._to_plant_frame(self._stub(), np.array([2.0, 0.0, 0.5]), _view([0.3, 0.0]), 0)
        assert out == pytest.approx([2.0 * np.cos(0.3), -2.0 * np.sin(0.3), 0.5])

    def test_drones_use_their_own_heading_error(self):
        v = np.array([1.0, 0.0, 0.0])
        a = FloodSearchMission._to_plant_frame(self._stub(), v, _view([0.3, -0.6]), 0)
        b = FloodSearchMission._to_plant_frame(self._stub(), v, _view([0.3, -0.6]), 1)
        assert a[1] < 0.0 < b[1]

    def test_speed_and_altitude_component_are_preserved(self):
        v = np.array([3.0, 4.0, 1.5])
        out = FloodSearchMission._to_plant_frame(self._stub(), v, _view([1.2, 0.0]), 0)
        assert np.hypot(out[0], out[1]) == pytest.approx(5.0) and out[2] == 1.5

    def test_placing_a_sensor_reading_and_mapping_a_command_are_inverse_rotations(self):
        v = np.array([1.0, 0.5, 0.0])
        placed = rotate_xy(v, 0.4)                                       # true frame -> drone's frame
        back = FloodSearchMission._to_plant_frame(self._stub(), placed, _view([0.4, 0.0]), 0)
        assert back == pytest.approx(v)

    def test_disabled_mapping_is_the_identity(self):
        v = np.array([1.0, 0.0, 0.0])
        assert FloodSearchMission._to_plant_frame(self._stub(False), v, _view([0.9, 0.0]), 0) is v


class TestBeaconPolicy:
    def _stub(self, estimated):
        return SimpleNamespace(cfg=MissionConfig(), board=RecruitmentBoard(), _synthetic_beacon_count=0,
                               _estimators=[object()] if estimated else None)

    def test_truth_state_never_chases_a_confirmation_that_matches_no_victim(self):
        stub = self._stub(estimated=False)
        FloodSearchMission._announce_beacon_for_confirmation(stub, None, np.array([1.0, 2.0]), 0, 1.0)
        assert stub.board.beacons == {} and stub._synthetic_beacon_count == 0

    def test_truth_state_announces_a_matched_victim_under_its_id(self):
        stub = self._stub(estimated=False)
        FloodSearchMission._announce_beacon_for_confirmation(stub, 4, np.array([1.0, 2.0]), 0, 1.0)
        assert list(stub.board.beacons) == [4]

    def test_estimated_mode_announces_every_confirmation(self):
        stub = self._stub(estimated=True)
        FloodSearchMission._announce_beacon_for_confirmation(stub, None, np.array([1.0, 2.0]), 0, 1.0)
        FloodSearchMission._announce_beacon_for_confirmation(stub, None, np.array([9.0, 9.0]), 1, 2.0)
        FloodSearchMission._announce_beacon_for_confirmation(stub, 3, np.array([5.0, 5.0]), 0, 3.0)
        assert sorted(stub.board.beacons) == [-2, -1, 3]                 # synthetic ids are negative and unique
        assert stub._synthetic_beacon_count == 2
        assert stub.board.beacons[-1]["pos"][:2].tolist() == [1.0, 2.0]
