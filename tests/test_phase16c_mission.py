"""Phase 16C-1: FloodSearchMission in `flight_control_mode="altitude_hold"` - see docs/PHASE16C_FLIGHT_LIFECYCLE.md.

Real (short) PyBullet missions; keep them small. The default `legacy` mode must be untouched; its
bit-for-bit equivalence to Phase 16B is proved by the telemetry fingerprint recorded in the doc, not by a golden
value here (numpy generator streams are not a stable thing to pin)."""
import dataclasses

import numpy as np
import pytest

from swarm_sim.config import MissionConfig
from swarm_sim.estimation import profiles as P
from swarm_sim.mission import FloodSearchMission


def _cfg(duration_sec=8.0, **kw):
    base = dict(num_drones=3, duration_sec=duration_sec, seed=11, num_victims=2, num_obstacles=1)
    base.update(kw)
    return MissionConfig(**base)


def _z(result):
    return np.array([r["z"] for r in result["telemetry"].log if r["t"] >= 2.0])


@pytest.fixture(scope="module")
def noisy_vertical_profile():
    """The Phase 16B failure case: vertical-speed noise at the horizontal 0.10 m/s density."""
    old = dataclasses.replace(P._FLOW_RF, vz_noise_std_mps=0.10)
    prof = dataclasses.replace(P.PROFILES["flow_rf"], name="flow_rf_oldvz", sensors=(old,))
    with pytest.MonkeyPatch.context() as mp:
        mp.setitem(P.PROFILES, "flow_rf_oldvz", prof)
        yield "flow_rf_oldvz"


# ==========================================================================
# configuration
# ==========================================================================

class TestModeSelection:
    def test_unknown_mode_is_rejected_before_anything_is_built(self):
        with pytest.raises(ValueError, match="flight_control_mode"):
            FloodSearchMission(_cfg(flight_control_mode="autopilot"))

    def test_the_default_is_the_legacy_pipeline(self):
        m = FloodSearchMission(_cfg(duration_sec=1.0))
        assert m.cfg.flight_control_mode == "legacy"
        assert m.vertical is None and m.autopilot.mode == "legacy"
        assert m.safety.cfg.independent_vertical_axis is False and m.safety.cfg.preserve_candidate_vertical is False
        assert m.pid is m.autopilot.pid

    def test_altitude_hold_builds_the_controller_and_switches_the_supervisor_and_actuator(self):
        m = FloodSearchMission(_cfg(duration_sec=1.0, flight_control_mode="altitude_hold"))
        assert m.vertical is not None and m.autopilot.mode == "velocity"
        assert m.safety.cfg.independent_vertical_axis is True and m.safety.cfg.preserve_candidate_vertical is True


# ==========================================================================
# holding altitude
# ==========================================================================

@pytest.fixture(scope="module")
def legacy_truth():
    return FloodSearchMission(_cfg()).run()


@pytest.fixture(scope="module")
def hold_truth():
    return FloodSearchMission(_cfg(flight_control_mode="altitude_hold")).run()


class TestHoldingAltitude:
    def test_it_holds_the_cruise_altitude_tightly(self, hold_truth):
        z = _z(hold_truth)
        assert np.abs(z - 3.0).max() < 0.05 and z.std() < 0.02
        assert hold_truth["contact_steps"] == 0

    def test_it_is_much_tighter_than_the_legacy_damped_altitude(self, legacy_truth, hold_truth):
        assert _z(hold_truth).std() < _z(legacy_truth).std() / 3.0

    def test_the_flight_results_are_reported_in_both_modes(self, legacy_truth, hold_truth):
        for res, mode in ((legacy_truth, "legacy"), (hold_truth, "altitude_hold")):
            assert res["flight_control_mode"] == mode
            alt = res["flight"]["altitude"]
            assert alt["samples"] > 0 and alt["target_m"] == 3.0
            assert set(res["flight"]["contacts"]["first_contact_root_cause_per_drone"]) == {
                "vertical_control", "attitude_loss", "obstacle", "swarm"}

    def test_the_mission_altitude_setpoint_is_followed(self):
        m = FloodSearchMission(_cfg(duration_sec=10.0, flight_control_mode="altitude_hold"))
        m.cfg.flight_altitude = 4.0           # the drones spawned at 3 m; the outer loop must climb them to 4 m
        z = np.array([r["z"] for r in m.run()["telemetry"].log if r["t"] >= 8.0])
        assert abs(z.mean() - 4.0) < 0.05 and z.std() < 0.02


OLD_FAILURE_CFG = dict(num_drones=4, duration_sec=14.0, seed=4, num_victims=3, num_obstacles=2,
                       localization_mode="estimated")


@pytest.fixture(scope="module")
def old_failure_legacy(noisy_vertical_profile):
    return FloodSearchMission(MissionConfig(estimator_profile=noisy_vertical_profile, **OLD_FAILURE_CFG)).run()


@pytest.fixture(scope="module")
def old_failure_hold(noisy_vertical_profile):
    return FloodSearchMission(MissionConfig(estimator_profile=noisy_vertical_profile,
                                            flight_control_mode="altitude_hold", **OLD_FAILURE_CFG)).run()


class TestTheOldFailureCase:
    """Vertical-speed noise at 0.10 m/s made Phase 16B's legacy drones sink, overshoot the ceiling and crash."""

    def test_the_legacy_pipeline_still_fails_on_it(self, old_failure_legacy):
        """Negative control: proves this scenario is capable of failing."""
        assert old_failure_legacy["flight"]["altitude"]["std_m"] > 0.3

    def test_altitude_hold_is_unaffected(self, old_failure_hold):
        alt = old_failure_hold["flight"]["altitude"]
        roots = old_failure_hold["flight"]["contacts"]["first_contact_root_cause_per_drone"]
        assert roots["vertical_control"] == 0 and roots["attitude_loss"] == 0       # (an obstacle graze is horizontal)
        assert alt["std_m"] < 0.05 and alt["min_m"] > 2.8 and alt["max_m"] < 3.2


# ==========================================================================
# what the outer loop reads
# ==========================================================================

class TestTheOuterLoopReadsTheEstimate:
    @staticmethod
    def _run_with_spy(**kw):
        m = FloodSearchMission(_cfg(duration_sec=4.0, flight_control_mode="altitude_hold", **kw))
        calls = []
        orig = m.vertical.command

        def spy(i, z_target, z_est, dt):
            calls.append(z_est)
            return orig(i, z_target, z_est, dt)

        m.vertical.command = spy
        res = m.run()
        return np.array(calls), np.array([r["z"] for r in res["telemetry"].log])

    def test_in_truth_state_it_sees_the_true_altitude(self):
        seen, true_z = self._run_with_spy()
        assert seen.shape == true_z.shape and np.allclose(seen, true_z, atol=1e-9)

    def test_in_estimated_mode_it_sees_the_estimate_not_the_truth(self):
        seen, true_z = self._run_with_spy(localization_mode="estimated", estimator_profile="flow_rf")
        assert seen.shape == true_z.shape
        assert not np.allclose(seen, true_z, atol=1e-6)          # a rangefinder-based estimate, not the physics
        assert np.abs(seen - true_z).max() < 0.3


# ==========================================================================
# gate G4: horizontal avoidance must not destroy the altitude hold
# ==========================================================================

@pytest.fixture(scope="module")
def crowded_hold():
    """Six drones in a 14 m arena with three obstacles: constant separation / obstacle overrides."""
    cfg = MissionConfig(num_drones=6, arena_size=14.0, duration_sec=12.0, seed=5, num_victims=2, num_obstacles=3,
                        localization_mode="estimated", estimator_profile="flow_rf",
                        flight_control_mode="altitude_hold")
    return FloodSearchMission(cfg).run()


class TestAvoidanceDoesNotBreakTheHold:
    OVERRIDES = ("SEPARATION_RISK", "SAFE_HOLD", "GEOFENCE_RISK")

    def test_the_scenario_really_exercises_the_overrides(self, crowded_hold):
        ticks = [r for r in crowded_hold["safety_telemetry"] if r["current_state"] in self.OVERRIDES and r["t"] > 2.0]
        assert len(ticks) >= 20

    def test_altitude_stays_within_30_cm_of_cruise_on_every_override_tick(self, crowded_hold):
        ticks = [r for r in crowded_hold["safety_telemetry"] if r["current_state"] in self.OVERRIDES and r["t"] > 2.0]
        assert max(abs(r["own_altitude_m"] - 3.0) for r in ticks) <= 0.3

    def test_and_over_the_whole_run(self, crowded_hold):
        alt = crowded_hold["flight"]["altitude"]
        assert alt["max_abs_error_m"] <= 0.3 and crowded_hold["flight"]["contacts"][
            "first_contact_root_cause_per_drone"]["vertical_control"] == 0


class TestTheAltitudeLoopKeepsRunningUnderAHold:
    """A supervisor HOLD / ABORT / rejected command means no horizontal motion, not no altitude control."""

    @staticmethod
    def _run(hold_from_start: bool):
        m = FloodSearchMission(_cfg(duration_sec=9.0, flight_control_mode="altitude_hold"))
        m.cfg.flight_altitude = 4.0                     # the loop must climb the drones from 3 m to 4 m
        if hold_from_start:
            m.operator_abort = True                     # the supervisor answers ABORT -> no horizontal motion
        return m.run()

    def test_under_an_abort_the_drones_still_reach_the_altitude_setpoint(self):
        res = self._run(hold_from_start=True)
        z = np.array([r["z"] for r in res["telemetry"].log if r["t"] >= 7.5])
        speed = np.array([r["speed"] for r in res["telemetry"].log if r["t"] >= 7.5])
        assert abs(z.mean() - 4.0) < 0.05 and speed.max() < 0.3          # holding position, at the new altitude
        assert res["safety_state_counts"].get("ABORT", 0) > 0

    def test_without_a_hold_it_of_course_does_too(self):
        z = np.array([r["z"] for r in self._run(hold_from_start=False)["telemetry"].log if r["t"] >= 7.5])
        assert abs(z.mean() - 4.0) < 0.05
