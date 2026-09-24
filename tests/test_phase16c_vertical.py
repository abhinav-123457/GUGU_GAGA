"""Phase 16C: the outer altitude loop, the plant-side actuator boundary, and their closed loop in PyBullet
(one drone, no mission) - see docs/PHASE16C_FLIGHT_LIFECYCLE.md. Gate G3 (step response) lives here."""
import contextlib
import io

import numpy as np
import pytest

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics

from swarm_sim.config import MissionConfig
from swarm_sim.flight import VerticalConfig, VerticalController
from swarm_sim.plant_actuator import SimulatedAutopilot

DT = 1.0 / 24.0


# ==========================================================================
# the controller
# ==========================================================================

class TestVerticalController:
    def _ctl(self, **kw):
        return VerticalController(2, VerticalConfig(**kw))

    def test_zero_at_the_target(self):
        assert self._ctl().command(0, 3.0, 3.0, DT) == 0.0

    def test_proportional_to_the_error_below_the_limits(self):
        c = self._ctl(kp_per_s=1.0, slew_mps2=1000.0)
        assert c.command(0, 3.0, 2.6, DT) == pytest.approx(0.4)
        assert c.command(1, 3.0, 3.3, DT) == pytest.approx(-0.3)

    def test_climb_and_descent_are_saturated_separately(self):
        c = self._ctl(max_climb_mps=1.5, max_descent_mps=0.5, slew_mps2=1000.0)
        assert c.command(0, 10.0, 0.0, DT) == pytest.approx(1.5)
        assert c.command(1, 0.0, 10.0, DT) == pytest.approx(-0.5)

    def test_the_output_never_changes_faster_than_the_slew_limit(self):
        c = self._ctl(slew_mps2=1.0)
        prev = 0.0
        for k in range(60):
            out = c.command(0, 10.0 if k < 30 else -10.0, 0.0, DT)
            assert abs(out - prev) <= 1.0 * DT + 1e-12
            prev = out

    def test_the_slew_reference_is_its_own_previous_output_not_a_measurement(self):
        """A noisy altitude reading moves the command by at most the slew step per tick."""
        c = self._ctl(slew_mps2=1.0)
        rng = np.random.default_rng(0)
        outs = [c.command(0, 3.0, 3.0 + 0.5 * rng.standard_normal(), DT) for _ in range(200)]
        assert np.max(np.abs(np.diff(outs))) <= 1.0 * DT + 1e-12

    @pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
    def test_a_non_finite_estimate_stops_the_motion_instead_of_guessing(self, bad):
        c = self._ctl(slew_mps2=1000.0)
        assert c.command(0, 3.0, 2.0, DT) > 0.0
        assert c.command(0, 3.0, bad, DT) == 0.0

    def test_drones_are_independent(self):
        c = self._ctl(slew_mps2=1000.0)
        c.command(0, 5.0, 3.0, DT)
        assert c.previous(1) == 0.0 and c.command(1, 3.0, 3.0, DT) == 0.0

    def test_reset_clears_one_drone(self):
        c = self._ctl(slew_mps2=1000.0)
        c.command(0, 5.0, 3.0, DT)
        c.reset(0)
        assert c.previous(0) == 0.0

    @pytest.mark.parametrize("bad", [dict(kp_per_s=0.0), dict(max_climb_mps=-1.0), dict(max_descent_mps=float("nan")),
                                     dict(slew_mps2=float("inf")), dict(kp_per_s=True)])
    def test_configuration_is_validated(self, bad):
        with pytest.raises(ValueError):
            VerticalConfig(**bad)

    @pytest.mark.parametrize("bad_dt", [0.0, -1.0, float("nan")])
    def test_bad_dt_is_rejected(self, bad_dt):
        with pytest.raises(ValueError):
            self._ctl().command(0, 3.0, 3.0, bad_dt)

    def test_the_mission_config_defaults_build_a_valid_controller(self):
        cfg = VerticalConfig.from_mission_config(MissionConfig())
        assert cfg == VerticalConfig()


# ==========================================================================
# the actuator boundary
# ==========================================================================

class _FakePID:
    def __init__(self):
        self.calls = []

    def computeControlFromState(self, **kw):
        self.calls.append(kw)
        return np.array([1.0, 2.0, 3.0, 4.0]), None, None


def _autopilot(mode, **kw):
    ap = SimulatedAutopilot(1, mode, flight_altitude_m=3.0, safety_enabled=kw.pop("safety_enabled", True),
                            floor_alt_m=0.5, ceiling_alt_m=8.0, max_accel_mps2=kw.pop("max_accel_mps2", None))
    ap.pid[0] = _FakePID()
    return ap


def _call(ap, position=(1.0, 2.0, 2.4), target_vel=(0.5, 0.0, 0.2), safe_vz=0.0):
    rpm = ap.rpm(0, np.zeros(20), np.array(position), np.array(target_vel), safe_vz, DT, DT)
    return rpm, ap.pid[0].calls[-1]


class TestActuatorLegacy:
    def test_holds_the_flight_altitude_when_the_safety_layer_asks_for_no_vertical_motion(self):
        rpm, call = _call(_autopilot("legacy"), safe_vz=0.0)
        assert list(rpm) == [1.0, 2.0, 3.0, 4.0]
        assert call["target_pos"].tolist() == [1.0, 2.0, 3.0]
        assert call["target_vel"].tolist() == [0.5, 0.0, 0.2]

    def test_re_anchors_to_true_z_when_vertical_motion_is_requested(self):
        _, call = _call(_autopilot("legacy"), position=(0.0, 0.0, 2.4), safe_vz=0.6)
        assert call["target_pos"][2] == pytest.approx(2.4 + 0.6 * DT)

    def test_the_re_anchored_target_is_clamped_to_the_floor_and_ceiling(self):
        _, low = _call(_autopilot("legacy"), position=(0.0, 0.0, 0.5), safe_vz=-40.0)
        _, high = _call(_autopilot("legacy"), position=(0.0, 0.0, 8.0), safe_vz=40.0)
        assert low["target_pos"][2] == 0.5 and high["target_pos"][2] == 8.0

    def test_a_tiny_vertical_command_below_1e9_does_not_re_anchor(self):
        _, call = _call(_autopilot("legacy"), safe_vz=5e-10)
        assert call["target_pos"][2] == 3.0

    def test_without_the_safety_layer_the_altitude_is_never_re_anchored(self):
        _, call = _call(_autopilot("legacy", safety_enabled=False), safe_vz=0.6)
        assert call["target_pos"][2] == 3.0


class TestActuatorVelocityMode:
    def test_the_position_target_is_the_current_true_position_in_all_three_axes(self):
        _, call = _call(_autopilot("velocity"), position=(1.0, 2.0, 2.4), safe_vz=0.7)
        assert call["target_pos"].tolist() == [1.0, 2.0, 2.4]

    def test_the_velocity_setpoint_passes_through_unchanged(self):
        _, call = _call(_autopilot("velocity"), target_vel=(0.5, -0.5, 0.25))
        assert call["target_vel"].tolist() == [0.5, -0.5, 0.25]

    def test_yaw_is_held_at_zero(self):
        _, call = _call(_autopilot("velocity"))
        assert call["target_rpy"].tolist() == [0.0, 0.0, 0.0]


def test_unknown_mode_is_rejected():
    with pytest.raises(ValueError):
        SimulatedAutopilot(1, "position", 3.0, True, 0.5, 8.0)


# ==========================================================================
# the closed loop in PyBullet
# ==========================================================================

def _fly(z0, target_fn, seconds, z_noise=0.0, seed=0, kp=1.0):
    with contextlib.redirect_stdout(io.StringIO()):
        env = CtrlAviary(drone_model=DroneModel.CF2X, num_drones=1, initial_xyzs=np.array([[0.0, 0.0, z0]]),
                         physics=Physics.PYB, pyb_freq=240, ctrl_freq=24, gui=False)
    ap = SimulatedAutopilot(1, "velocity", 3.0, True, 0.5, 8.0)
    vc = VerticalController(1, VerticalConfig(kp_per_s=kp))
    rng = np.random.default_rng(seed)
    rpm = np.zeros((1, 4))
    rows = []
    for k in range(int(seconds * 24)):
        t = k * DT
        obs, *_ = env.step(rpm)
        s = obs[0]
        z_est = s[2] + z_noise * rng.standard_normal()
        vz = vc.command(0, target_fn(t), z_est, DT)
        rpm[0] = ap.rpm(0, s, s[0:3], np.array([0.0, 0.0, vz]), vz, DT, env.CTRL_TIMESTEP)
        rows.append((t, s[2], s[12], np.hypot(s[0], s[1])))
    env.close()
    return np.array(rows)


@pytest.fixture(scope="module")
def step_response():
    return _fly(3.0, lambda t: 3.0 if t < 2.0 else 4.0, 12.0)


class TestClosedLoopStepResponse:
    """Gate G3: a 1 m altitude step settles within 4 s with at most 0.2 m overshoot and no oscillation."""

    def test_it_holds_altitude_before_the_step(self, step_response):
        r = step_response[step_response[:, 0] < 2.0]
        assert np.abs(r[:, 1] - 3.0).max() < 0.03

    def test_it_settles_to_within_10_cm_within_4_s_of_the_step(self, step_response):
        r = step_response
        outside = np.where(np.abs(r[:, 1] - 4.0) > 0.1)[0]
        assert r[outside[-1], 0] - 2.0 <= 4.0

    def test_overshoot_is_at_most_20_cm(self, step_response):
        assert step_response[:, 1].max() - 4.0 <= 0.2

    def test_the_final_error_is_small_and_it_does_not_oscillate(self, step_response):
        tail = step_response[step_response[:, 0] > 8.0]
        assert abs(tail[:, 1].mean() - 4.0) < 0.02 and tail[:, 1].std() < 0.01

    def test_it_does_not_drift_horizontally(self, step_response):
        assert step_response[:, 3].max() < 0.05


def test_altitude_hold_is_immune_to_a_noisy_altitude_estimate():
    r = _fly(3.0, lambda t: 3.0, 25.0, z_noise=0.05, seed=3)
    tail = r[r[:, 0] > 3.0]
    assert tail[:, 1].std() < 0.02 and np.abs(tail[:, 1] - 3.0).max() < 0.06


def test_a_ground_takeoff_reaches_cruise_without_overshoot():
    r = _fly(0.05, lambda t: 3.0, 12.0)
    assert r[:, 1].max() < 3.15 and abs(r[-1, 1] - 3.0) < 0.05
    assert r[:, 3].max() < 0.1


# ==========================================================================
# velocity-setpoint shaping (the autopilot's own acceleration limit)
# ==========================================================================

class TestSetpointShaping:
    def _ap(self, accel=4.0, n=1):
        return SimulatedAutopilot(n, "velocity", 3.0, True, 0.5, 8.0, max_accel_mps2=accel)

    def test_a_step_to_zero_from_speed_is_spread_over_several_ticks(self):
        ap = self._ap()
        ap.shaped_velocity(0, (5.0, 0.0, 0.0), 10.0)            # settle the reference at 5 m/s (big dt: no limit)
        out = [ap.shaped_velocity(0, (0.0, 0.0, 0.0), DT)[0] for _ in range(40)]
        step = 4.0 * DT
        assert out[0] == pytest.approx(5.0 - step)
        assert all(abs(a - b) <= step + 1e-12 for a, b in zip(out, out[1:]))
        assert out[-1] == 0.0 and out.index(0.0) > 20            # ~5 / 0.167 ticks of braking

    def test_horizontal_change_is_limited_as_a_vector_not_per_axis(self):
        ap = self._ap()
        out = ap.shaped_velocity(0, (3.0, 4.0, 0.0), DT)
        assert np.hypot(out[0], out[1]) == pytest.approx(4.0 * DT)
        assert out[0] / out[1] == pytest.approx(3.0 / 4.0)       # direction preserved

    def test_vertical_is_limited_independently_of_horizontal(self):
        ap = self._ap()
        out = ap.shaped_velocity(0, (5.0, 0.0, 1.0), DT)
        assert out[2] == pytest.approx(4.0 * DT)                  # the vertical step is not shrunk by the big horizontal one

    def test_a_setpoint_within_the_limit_passes_through_exactly(self):
        ap = self._ap()
        out = ap.shaped_velocity(0, (0.1, -0.05, 0.02), DT)
        assert out.tolist() == [0.1, -0.05, 0.02]

    def test_drones_are_independent(self):
        ap = self._ap(n=2)
        ap.shaped_velocity(0, (5.0, 0.0, 0.0), DT)
        assert ap.shaped_velocity(1, (0.0, 0.0, 0.0), DT).tolist() == [0.0, 0.0, 0.0]

    def test_the_limit_applies_only_in_velocity_mode(self):
        ap = SimulatedAutopilot(1, "legacy", 3.0, True, 0.5, 8.0, max_accel_mps2=4.0)
        ap.pid[0] = _FakePID()
        ap.rpm(0, np.zeros(20), np.zeros(3), np.array([5.0, 0.0, 0.0]), 0.0, DT, DT)
        assert ap.pid[0].calls[-1]["target_vel"].tolist() == [5.0, 0.0, 0.0]

    def test_rpm_hands_the_shaped_setpoint_to_the_pid(self):
        ap = self._ap()
        ap.pid[0] = _FakePID()
        ap.rpm(0, np.zeros(20), np.zeros(3), np.array([5.0, 0.0, 0.0]), 0.0, DT, DT)
        assert ap.pid[0].calls[-1]["target_vel"][0] == pytest.approx(4.0 * DT)

    def test_no_limit_configured_means_no_shaping(self):
        ap = self._ap(accel=None)
        ap.pid[0] = _FakePID()
        ap.rpm(0, np.zeros(20), np.zeros(3), np.array([5.0, 0.0, 0.0]), 0.0, DT, DT)
        assert ap.pid[0].calls[-1]["target_vel"].tolist() == [5.0, 0.0, 0.0]

    @pytest.mark.parametrize("bad", [0.0, -1.0, float("nan"), float("inf")])
    def test_the_limit_is_validated(self, bad):
        with pytest.raises(ValueError):
            SimulatedAutopilot(1, "velocity", 3.0, True, 0.5, 8.0, max_accel_mps2=bad)


def _stop_from_speed(accel, speed=5.0):
    """Cruise at `speed`, then an instantaneous zero-velocity setpoint. Returns (worst tilt after the stop, lowest z)."""
    with contextlib.redirect_stdout(io.StringIO()):
        env = CtrlAviary(drone_model=DroneModel.CF2X, num_drones=1, initial_xyzs=np.array([[0.0, 0.0, 3.0]]),
                         physics=Physics.PYB, pyb_freq=240, ctrl_freq=24, gui=False)
    ap = SimulatedAutopilot(1, "velocity", 3.0, True, 0.5, 8.0, max_accel_mps2=accel)
    vc = VerticalController(1, VerticalConfig())
    rpm = np.zeros((1, 4))
    worst_tilt, min_z = 0.0, 9.0
    for k in range(9 * 24):
        obs, *_ = env.step(rpm)
        s = obs[0]
        vz = vc.command(0, 3.0, s[2], DT)
        vx = speed if k < 4 * 24 else 0.0
        rpm[0] = ap.rpm(0, s, s[0:3], np.array([vx, 0.0, vz]), vz, DT, env.CTRL_TIMESTEP)
        if k > 4 * 24:
            worst_tilt, min_z = max(worst_tilt, abs(s[7]), abs(s[8])), min(min_z, s[2])
    env.close()
    return worst_tilt, min_z


class TestAbruptStop:
    """The 16C gate finding: the mission maps HOLD / ABORT / rejection to an instantaneous zero-velocity setpoint,
    and a 27 g airframe braked from speed in one tick rolls far past what it can recover from."""

    def test_an_unshaped_stop_from_5_mps_tilts_the_airframe_past_a_radian_and_makes_it_sag(self):
        tilt, min_z = _stop_from_speed(None)
        assert tilt > 1.0 and min_z < 2.5                    # negative control: the hazard is real

    def test_the_autopilots_acceleration_limit_removes_it(self):
        tilt, min_z = _stop_from_speed(4.0)
        assert tilt < 0.7 and min_z > 2.9
