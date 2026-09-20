"""Phase 14 functional tests - see docs/PHASE14_SAR_WEBOTS.md and
swarm_sim/sar_mission.py. Architecture/AST checks live in
tests/test_phase14_sar_architecture.py.

Most tests drive `SARMission.tick()` directly with hand-built
`VehicleTelemetry` objects - fast, deterministic, no transport needed, and
precise about exactly which condition is being exercised (mirrors Phase
10/13's own style of testing gate/helper functions directly). A smaller
set of end-to-end tests drive the full `run_sar_mission_offline()` runner
against a real (fake) transport for the happy path and reproducibility
checks.
"""
from __future__ import annotations

import itertools
import math
import os

import numpy as np
import pytest

from swarm_sim.autopilot.types import AdapterResult, AutopilotMode, ConnectionState, VehicleTelemetry
from swarm_sim.contracts import DetectionCandidate, Frame, SensorObservation
from swarm_sim.safety_supervisor import SafetySupervisor, SafetySupervisorConfig, geofence_horizontal_margin_m
from swarm_sim.sar_mission import (
    ALLOWED_TRANSITIONS, MissionState, SARMission, SARMissionConfig, TERMINAL_STATES, run_sar_mission_offline,
)
from swarm_sim.sar_search import LawnmowerSearchPattern, generate_lawnmower_waypoints, velocity_toward
from swarm_sim.sar_world import RectangularSearchArea, SARWorld, SensorModelConfig

_seq = itertools.count()


def _telem(position_m=(0.0, 0.0, 0.0), velocity_mps=(0.0, 0.0, 0.0), battery_fraction=1.0,
           estimator_valid=True, failsafe=False, timestamp_s=0.0, armed=False,
           mode=AutopilotMode.OFFBOARD) -> VehicleTelemetry:
    return VehicleTelemetry(
        vehicle_id="drone0", timestamp_s=timestamp_s, frame=Frame.LOCAL_ENU, position_m=position_m,
        velocity_mps=velocity_mps, acceleration_mps2=(0.0, 0.0, 0.0), attitude_rad=(0.0, 0.0, 0.0),
        angular_velocity_radps=(0.0, 0.0, 0.0), battery_fraction=battery_fraction,
        estimator_valid=estimator_valid, connection_state=ConnectionState.CONNECTED, autopilot_mode=mode,
        armed=armed, failsafe=failsafe, sequence=next(_seq),
    )


def _config(**overrides) -> SARMissionConfig:
    defaults = dict(
        search_area=RectangularSearchArea(0.0, 0.0, 10.0, 10.0), search_altitude_m=1.0, lane_spacing_m=3.0,
        geofence_margin_m=1.0, victim_positions_m=((5.0, 5.0),), seed=7, dt_s=0.5, max_speed_mps=0.25,
        mission_timeout_s=90.0, command_timeout_s=5.0, landing_timeout_s=30.0,
    )
    defaults.update(overrides)
    return SARMissionConfig(**defaults)


def _mission(config=None, victim_positions_m=None) -> SARMission:
    config = config or _config()
    if victim_positions_m is not None:
        config.victim_positions_m = victim_positions_m
    world = SARWorld(config.search_area, config.victim_positions_m, rng=np.random.default_rng(config.seed),
                      sensor_config=config.sensor_config, vehicle_id=config.vehicle_id)
    return SARMission(config, world, SafetySupervisor(config.build_safety_config()))


def _run_ticks(mission: SARMission, positions, dt_s=0.5, start_t=0.0):
    """Drives tick() with a fixed sequence of positions (test convenience -
    ignores the returned command's velocity and just pretends the vehicle
    was already there, which is fine for state-machine-only tests)."""
    t = start_t
    results = []
    for pos in positions:
        t += dt_s
        results.append(mission.tick(t, _telem(position_m=pos, timestamp_s=t)))
    return results


# ==========================================================================
# 1. mission states / transitions
# ==========================================================================

class TestMissionStates:
    def test_every_mission_state_is_reachable_in_allowed_transitions(self):
        # every state named in the spec must appear as a key or a value
        for name in ("INIT", "PREFLIGHT_CHECK", "TAKEOFF_REQUESTED", "TRANSIT", "SEARCHING",
                     "DETECTION_CANDIDATE", "DETECTION_CONFIRMED", "RETURN_HOME", "LAND_REQUESTED",
                     "LANDED", "ABORTED", "FAILED"):
            state = MissionState(name)
            assert state in ALLOWED_TRANSITIONS

    def test_illegal_transition_raises(self):
        mission = _mission()
        with pytest.raises(ValueError):
            mission._transition(MissionState.LANDED, "not a real path from INIT")

    def test_terminal_states_never_transition_again(self):
        mission = _mission()
        mission.state = MissionState.LANDED
        result = mission.tick(1.0, _telem(position_m=(0.0, 0.0, 0.0), timestamp_s=1.0))
        assert result.adapter_command is None
        assert mission.state == MissionState.LANDED

    def test_preflight_check_fails_without_position(self):
        mission = _mission()
        telem = VehicleTelemetry(
            vehicle_id="drone0", timestamp_s=0.5, frame=Frame.LOCAL_ENU, position_m=None, velocity_mps=None,
            acceleration_mps2=None, attitude_rad=None, angular_velocity_radps=None, battery_fraction=None,
            estimator_valid=False, connection_state=ConnectionState.CONNECTED, autopilot_mode=AutopilotMode.UNKNOWN,
            armed=False, failsafe=False, sequence=0,
        )
        mission.tick(0.5, telem)
        assert mission.state == MissionState.FAILED
        assert mission.failure_reason == "preflight_check_failed"

    def test_full_offline_happy_path_reaches_landed(self):
        summary = run_sar_mission_offline(_config(seed=1))
        assert summary["final_state"] == "LANDED"
        assert summary["landing_result"] is True
        assert summary["failure_reason"] is None


# ==========================================================================
# 2. lawnmower search path stays inside the geofence
# ==========================================================================

class TestSearchPattern:
    def test_waypoints_stay_within_the_margin_bounded_area(self):
        area = RectangularSearchArea(0.0, 0.0, 20.0, 14.0)
        waypoints = generate_lawnmower_waypoints(area, altitude_m=1.5, lane_spacing_m=3.0, margin_m=1.0)
        assert len(waypoints) > 0
        for x, y, z in waypoints:
            assert 1.0 <= x <= 19.0
            assert 1.0 <= y <= 13.0
            assert z == 1.5

    def test_waypoints_are_within_the_geofence_built_from_the_same_area(self):
        config = _config()
        geofence = config.build_geofence()
        waypoints = generate_lawnmower_waypoints(config.search_area, config.search_altitude_m,
                                                   config.lane_spacing_m, margin_m=config.geofence_margin_m)
        cx, cy = geofence.center_m
        hx, hy = geofence.half_extents_m
        for x, y, _z in waypoints:
            assert cx - hx <= x <= cx + hx
            assert cy - hy <= y <= cy + hy

    def test_lane_spacing_must_be_positive(self):
        area = RectangularSearchArea(0.0, 0.0, 10.0, 10.0)
        with pytest.raises(ValueError):
            generate_lawnmower_waypoints(area, 1.0, lane_spacing_m=0.0)

    def test_margin_too_large_raises(self):
        area = RectangularSearchArea(0.0, 0.0, 2.0, 2.0)
        with pytest.raises(ValueError):
            generate_lawnmower_waypoints(area, 1.0, lane_spacing_m=1.0, margin_m=5.0)

    def test_home_sits_with_real_headroom_inside_the_geofence_risk_margin(self):
        """Regression for the first live Webots run (docs/PHASE14_SAR_WEBOTS.md):
        home_m must sit MORE than geofence_margin_m from the built fence
        boundary, not exactly on that threshold - otherwise real telemetry
        noise flips SafetySupervisor's GEOFENCE_RISK check on almost every
        tick and the mission never leaves TAKEOFF_REQUESTED, even though
        offline's noiseless FakeSITLTransport never showed the problem."""
        config = _config(home_m=(0.0, 0.0, 0.0), geofence_margin_m=1.0)
        geofence = config.build_geofence()
        margin_at_home = geofence_horizontal_margin_m(config.home_m, geofence)
        assert margin_at_home > config.geofence_margin_m
        assert margin_at_home >= 2 * config.geofence_margin_m - 1e-9

    def test_home_headroom_scales_with_margin_for_a_home_at_search_area_corner(self):
        area = RectangularSearchArea(0.0, 0.0, 20.0, 14.0)
        config = _config(search_area=area, home_m=(0.0, 0.0, 0.0), geofence_margin_m=1.5)
        geofence = config.build_geofence()
        margin_at_home = geofence_horizontal_margin_m(config.home_m, geofence)
        assert margin_at_home > config.geofence_margin_m

    def test_generation_is_deterministic(self):
        area = RectangularSearchArea(0.0, 0.0, 15.0, 15.0)
        a = generate_lawnmower_waypoints(area, 1.0, lane_spacing_m=2.5, margin_m=0.5)
        b = generate_lawnmower_waypoints(area, 1.0, lane_spacing_m=2.5, margin_m=0.5)
        assert a == b

    def test_velocity_toward_caps_at_max_speed(self):
        v = velocity_toward((0.0, 0.0, 0.0), (100.0, 0.0, 0.0), max_speed_mps=0.25)
        assert math.isclose(math.dist(v, (0, 0, 0)), 0.25, abs_tol=1e-9)

    def test_velocity_toward_zero_at_target(self):
        v = velocity_toward((1.0, 2.0, 3.0), (1.0, 2.0, 3.0), max_speed_mps=0.25)
        assert v == (0.0, 0.0, 0.0)

    def test_pattern_follower_advances_and_completes(self):
        pattern = LawnmowerSearchPattern(((0.0, 0.0, 1.0), (1.0, 0.0, 1.0)), reach_tolerance_m=0.1)
        assert pattern.waypoints_completed == 0
        assert not pattern.advance_if_reached((5.0, 5.0, 1.0))
        assert pattern.advance_if_reached((0.05, 0.0, 1.0))
        assert pattern.waypoints_completed == 1
        assert pattern.advance_if_reached((1.0, 0.0, 1.0))
        assert pattern.is_complete()
        assert pattern.current_waypoint is None


# ==========================================================================
# 3. hidden ground-truth isolation
# ==========================================================================

class TestHiddenTruthIsolation:
    def test_sar_world_public_surface_never_returns_a_position(self):
        world = SARWorld(RectangularSearchArea(0, 0, 10, 10), [(5.0, 5.0)], rng=np.random.default_rng(1))
        public_attrs = [a for a in dir(world) if not a.startswith("_")]
        for name in public_attrs:
            value = getattr(world, name)
            if callable(value):
                continue
            # every non-callable public attribute must be a count/id/area, never raw coordinates
            assert not (isinstance(value, tuple) and len(value) == 2 and all(isinstance(v, float) for v in value)), \
                f"public attribute {name!r} looks like a bare (x, y) position: {value!r}"

    def test_observe_never_includes_a_ground_truth_field(self):
        world = SARWorld(RectangularSearchArea(0, 0, 10, 10), [(5.0, 5.0)], rng=np.random.default_rng(1),
                          sensor_config=SensorModelConfig(victim_sensor_false_negative_prob=0.0,
                                                            victim_sensor_dropout_prob=0.0,
                                                            victim_sensor_latency_steps=0,
                                                            victim_sensor_range_m=100.0,
                                                            victim_sensor_hfov_deg=360.0,
                                                            victim_sensor_vfov_deg=360.0))
        obs = world.observe((5.0, 5.0, 1.0), (1.0, 0.0), 0.0)
        assert isinstance(obs, SensorObservation)
        for d in obs.detections:
            assert isinstance(d, DetectionCandidate)
            # a DetectionCandidate has no ground-truth-linking field at all
            assert not hasattr(d, "victim_id")
            assert not hasattr(d, "ground_truth_id")

    def test_mark_found_near_returns_id_only_never_a_position(self):
        world = SARWorld(RectangularSearchArea(0, 0, 10, 10), [(5.0, 5.0)], rng=np.random.default_rng(1))
        matched = world.mark_found_near((5.1, 5.1), radius_m=1.0)
        assert matched == "victim-0"
        assert isinstance(matched, str)


# ==========================================================================
# 4. sensor / detection scenarios
# ==========================================================================

def _perfect_sensor_cfg(**overrides):
    defaults = dict(victim_sensor_range_m=100.0, victim_sensor_hfov_deg=360.0, victim_sensor_vfov_deg=360.0,
                     victim_sensor_noise_std_m=0.0, victim_sensor_false_negative_prob=0.0,
                     victim_sensor_false_positive_rate=0.0, victim_sensor_latency_steps=0,
                     victim_sensor_dropout_prob=0.0, victim_sensor_confidence=0.9,
                     victim_sensor_localization_uncertainty_m=0.1)
    defaults.update(overrides)
    return SensorModelConfig(**defaults)


class TestSensorAndDetectionScenarios:
    def test_1_perfect_detection(self):
        world = SARWorld(RectangularSearchArea(0, 0, 10, 10), [(5.0, 5.0)], rng=np.random.default_rng(0),
                          sensor_config=_perfect_sensor_cfg())
        obs = world.observe((5.0, 5.0, 1.0), (1.0, 0.0), 0.0)
        assert len(obs.detections) == 1
        assert obs.dropout is False
        assert math.dist(obs.detections[0].position_m, (5.0, 5.0)) < 1e-6

    def test_2_noisy_detection(self):
        world = SARWorld(RectangularSearchArea(0, 0, 10, 10), [(5.0, 5.0)], rng=np.random.default_rng(3),
                          sensor_config=_perfect_sensor_cfg(victim_sensor_noise_std_m=0.5))
        obs = world.observe((5.0, 5.0, 1.0), (1.0, 0.0), 0.0)
        assert len(obs.detections) == 1
        d = math.dist(obs.detections[0].position_m, (5.0, 5.0))
        assert 0.0 < d < 3.0   # noisy but not wildly off (3-sigma-ish bound for std=0.5)

    def test_3_missed_detection(self):
        world = SARWorld(RectangularSearchArea(0, 0, 10, 10), [(5.0, 5.0)], rng=np.random.default_rng(0),
                          sensor_config=_perfect_sensor_cfg(victim_sensor_false_negative_prob=1.0))
        obs = world.observe((5.0, 5.0, 1.0), (1.0, 0.0), 0.0)
        assert obs.detections == ()
        assert obs.dropout is False   # sensor worked, genuinely saw nothing - not the same as dropout

    def test_4_false_positive(self):
        # no real victim in range at all - any detection must be a phantom
        world = SARWorld(RectangularSearchArea(0, 0, 10, 10), [(500.0, 500.0)], rng=np.random.default_rng(0),
                          sensor_config=_perfect_sensor_cfg(victim_sensor_false_positive_rate=1.0,
                                                              victim_sensor_false_positive_confidence=0.3))
        obs = world.observe((5.0, 5.0, 1.0), (1.0, 0.0), 0.0)
        assert len(obs.detections) == 1
        assert obs.detections[0].confidence == pytest.approx(0.3)

    def test_5_delayed_observation(self):
        world = SARWorld(RectangularSearchArea(0, 0, 10, 10), [(5.0, 5.0)], rng=np.random.default_rng(0),
                          sensor_config=_perfect_sensor_cfg(victim_sensor_latency_steps=2))
        obs0 = world.observe((5.0, 5.0, 1.0), (1.0, 0.0), 0.0)
        obs1 = world.observe((5.0, 5.0, 1.0), (1.0, 0.0), 1.0)
        obs2 = world.observe((5.0, 5.0, 1.0), (1.0, 0.0), 2.0)
        # the real detection only becomes available after latency_steps ticks
        assert obs0.dropout is True and obs0.detections == ()
        assert obs1.dropout is True and obs1.detections == ()
        assert obs2.dropout is False and len(obs2.detections) == 1

    def test_6_no_victim_found(self):
        world = SARWorld(RectangularSearchArea(0, 0, 10, 10), [], rng=np.random.default_rng(0),
                          sensor_config=_perfect_sensor_cfg())
        obs = world.observe((5.0, 5.0, 1.0), (1.0, 0.0), 0.0)
        assert obs.detections == ()
        assert world.victim_ground_truth_count == 0
        assert world.missed_victim_count == 0

    def test_7_duplicate_observation_handled_by_mission(self):
        mission = _mission()
        d = DetectionCandidate(candidate_id="det-dup", position_m=(5.0, 5.0), frame=Frame.LOCAL_ENU,
                                confidence=0.9, localization_uncertainty_m=0.2)
        obs = SensorObservation(vehicle_id="drone0", sensor_timestamp_s=1.0, sensor_latency_s=0.0, fov_deg=360.0,
                                 range_returns_m=(), occluded=(), dropout=False, pose_uncertainty_m=0.0,
                                 detections=(d,))
        mission.ingest_observation(obs, 1.0)
        assert mission.duplicate_observations == 0
        mission.ingest_observation(obs, 1.1)   # exact same candidate_id again
        assert mission.duplicate_observations == 1
        assert mission.detections_used_for_confirmation == 1   # only counted once, not twice

    def test_deterministic_by_seed(self):
        cfg = _perfect_sensor_cfg(victim_sensor_noise_std_m=0.5, victim_sensor_false_negative_prob=0.3,
                                    victim_sensor_dropout_prob=0.2)

        def _run(seed):
            world = SARWorld(RectangularSearchArea(0, 0, 10, 10), [(5.0, 5.0), (2.0, 8.0)],
                              rng=np.random.default_rng(seed), sensor_config=cfg)
            return [world.observe((5.0, 5.0, 1.0), (1.0, 0.0), float(t)) for t in range(10)]

        run_a = _run(123)
        run_b = _run(123)
        for oa, ob in zip(run_a, run_b):
            assert oa.dropout == ob.dropout
            assert len(oa.detections) == len(ob.detections)
            for da, db in zip(oa.detections, ob.detections):
                assert da.position_m == db.position_m
                assert da.confidence == db.confidence


# ==========================================================================
# 5. command path: SafetySupervisor always runs, raw candidates never bypass it
# ==========================================================================

class TestCommandPath:
    def test_build_safety_config_uses_mission_dt_as_the_first_tick_fallback(self):
        """Regression for this phase's third live finding: SafetySupervisorConfig's
        own fallback_dt_s (1/24s) is tuned for a fast PyBullet-style swarm
        loop, not this mission's own control period. Left at that default,
        a vehicle's very first evaluate() call - exactly when it may still
        carry real residual momentum from a just-completed real takeoff -
        gets an acceleration budget ~24x too small to correct it, letting
        real telemetry coast well past the intended altitude before a
        realistic dt takes over on the next tick. build_safety_config()
        must use the mission's own dt_s instead."""
        config = _config(dt_s=1.0)
        safety_config = config.build_safety_config()
        assert safety_config.fallback_dt_s == 1.0

    def test_explicit_safety_config_override_is_not_clobbered(self):
        override = SafetySupervisorConfig(max_speed_mps=0.1, geofence_margin_m=3.0, fallback_dt_s=2.5)
        config = _config(dt_s=1.0, safety_config=override)
        assert config.build_safety_config() is override

    def test_tick_never_returns_a_command_without_a_safety_decision(self):
        mission = _mission()
        results = _run_ticks(mission, [(0.0, 0.0, 0.5)])
        for r in results:
            if r.adapter_command is not None:
                assert r.decision is not None
                assert r.decision.accepted is True
                assert r.decision.filtered_command is r.adapter_command.command

    def test_rejected_candidate_never_produces_an_adapter_command(self):
        mission = _mission()
        # force a bogus own-state indirectly is hard from tick(); instead
        # directly exercise the supervisor's own rejection path the way
        # tick() does, proving the wiring: a non-finite candidate is
        # rejected and no AdapterCommand is ever built from it.
        from swarm_sim.safety_supervisor import CandidateCommand, MissionContext
        supervisor = SafetySupervisor()
        bad = CandidateCommand(vehicle_id="drone0", desired_velocity_mps=(float("nan"), 0.0, 0.0),
                                frame=Frame.LOCAL_ENU, timestamp_s=0.0, expiration_time_s=1.0)
        # Solidly inside the area, far from any fence edge - otherwise the
        # supervisor's own (correct, higher-priority) geofence-proximity
        # tier overrides before ever reaching the candidate-validity check,
        # which is a different, also-real safety behavior, not this test's
        # target.
        own_state = mission._vehicle_state_from_telemetry(_telem(position_m=(5.0, 5.0, 1.0)), 0.0)
        geofence = mission.geofence
        decision = supervisor.evaluate(bad, own_state, _telem_observation(), (),
                                        MissionContext(geofence=geofence), 0.0)
        assert decision.accepted is False
        assert decision.filtered_command is None

    def test_command_timeout_after_persistent_rejection_fails_mission(self, monkeypatch):
        mission = _mission(_config(command_timeout_s=1.0))

        class _AlwaysRejectSupervisor:
            def evaluate(self, *args, **kwargs):
                from swarm_sim.contracts import SafetyState
                from swarm_sim.contracts import SafetyDecision
                return SafetyDecision(vehicle_id="drone0", sim_time_s=kwargs.get("now_s", 0.0) if kwargs else args[-1],
                                       accepted=False, filtered_command=None, active_constraints=(),
                                       reason="forced_test_rejection", emergency_state=SafetyState.NORMAL,
                                       min_predicted_clearance_m=None, time_to_collision_s=None)

        mission.supervisor = _AlwaysRejectSupervisor()
        mission.state = MissionState.TAKEOFF_REQUESTED
        mission._mission_start_s = 0.0
        t = 0.0
        for _ in range(10):
            t += 0.5
            mission.tick(t, _telem(position_m=(0.0, 0.0, 0.2), timestamp_s=t))
            if mission.state in TERMINAL_STATES:
                break
        assert mission.state == MissionState.FAILED
        assert mission.failure_reason == "command_timeout_supervisor_rejected"


def _telem_observation():
    return SensorObservation(vehicle_id="drone0", sensor_timestamp_s=0.0, sensor_latency_s=0.0, fov_deg=360.0,
                              range_returns_m=(), occluded=(), dropout=True, pose_uncertainty_m=0.0, detections=())


# ==========================================================================
# 6. failure/safety-condition handling ("never silently continue")
# ==========================================================================

class TestFailureConditions:
    def test_estimator_invalid_persisting_fails_mission(self):
        mission = _mission()
        mission.state = MissionState.SEARCHING
        mission._mission_start_s = 0.0
        t = 0.0
        for _ in range(10):
            t += 1.0   # grace is 2.0s -> exceeded well before 10 ticks at 1.0s each
            mission.tick(t, _telem(position_m=(1.0, 1.0, 1.0), estimator_valid=False, timestamp_s=t))
            if mission.state in TERMINAL_STATES:
                break
        assert mission.state == MissionState.FAILED
        assert mission.failure_reason == "estimator_invalid_persisted"

    def test_estimator_invalid_transient_blip_does_not_fail_mission(self):
        mission = _mission()
        mission.state = MissionState.SEARCHING
        mission._mission_start_s = 0.0
        mission.tick(1.0, _telem(position_m=(1.0, 1.0, 1.0), estimator_valid=False, timestamp_s=1.0))
        mission.tick(1.2, _telem(position_m=(1.0, 1.0, 1.0), estimator_valid=True, timestamp_s=1.2))
        assert mission.state != MissionState.FAILED

    def test_heartbeat_loss_persisting_fails_mission(self):
        mission = _mission()
        mission.state = MissionState.SEARCHING
        mission._mission_start_s = 0.0
        t = 0.0
        for _ in range(10):
            t += 1.0
            mission.tick(t, _telem(position_m=(1.0, 1.0, 1.0), failsafe=True, timestamp_s=t))
            if mission.state in TERMINAL_STATES:
                break
        assert mission.state == MissionState.FAILED
        assert mission.failure_reason == "heartbeat_or_link_loss_persisted"

    def test_stale_telemetry_persisting_fails_mission(self):
        mission = _mission()
        mission.state = MissionState.SEARCHING
        mission._mission_start_s = 0.0
        # telemetry timestamp frozen far in the past relative to now_s each tick
        for now_s in (10.0, 11.0, 12.0, 13.0):
            mission.tick(now_s, _telem(position_m=(1.0, 1.0, 1.0), timestamp_s=0.0))
            if mission.state in TERMINAL_STATES:
                break
        assert mission.state == MissionState.FAILED
        assert mission.failure_reason == "stale_telemetry_persisted"

    def test_mission_timeout_forces_land_requested(self):
        mission = _mission(_config(mission_timeout_s=5.0))
        mission.state = MissionState.SEARCHING
        mission._mission_start_s = 0.0
        mission.tick(6.0, _telem(position_m=(3.0, 3.0, 1.0), timestamp_s=6.0))
        assert mission.state in (MissionState.LAND_REQUESTED, MissionState.LANDED)

    def test_landing_timeout_fails_mission_if_never_touches_down(self):
        mission = _mission(_config(landing_timeout_s=1.0))
        mission.state = MissionState.LAND_REQUESTED
        mission._mission_start_s = 0.0
        t = 0.0
        for _ in range(10):
            t += 1.0
            # stays hovering well above ground the whole time - never lands
            mission.tick(t, _telem(position_m=(0.0, 0.0, 1.0), timestamp_s=t))
            if mission.state in TERMINAL_STATES:
                break
        assert mission.state == MissionState.FAILED
        assert mission.failure_reason == "landing_timeout_exceeded"

    def test_landing_confirmed_only_by_real_telemetry_not_by_a_command_ack(self):
        """A LAND command being accepted is not, by itself, landing
        confirmation - only telemetry showing the vehicle actually at
        ground level with near-zero velocity does."""
        mission = _mission()
        mission.state = MissionState.LAND_REQUESTED
        mission._mission_start_s = 0.0
        # still well above ground - must NOT be reported as landed
        result = mission.tick(1.0, _telem(position_m=(0.0, 0.0, 0.9), timestamp_s=1.0))
        assert mission.state == MissionState.LAND_REQUESTED
        # now genuinely at ground level with near-zero velocity
        mission.tick(2.0, _telem(position_m=(0.0, 0.0, 0.01), velocity_mps=(0.0, 0.0, 0.0), timestamp_s=2.0))
        assert mission.state == MissionState.LANDED

    def test_low_battery_triggers_return_home(self):
        cfg = _config(safety_config=SafetySupervisorConfig(max_speed_mps=0.25, battery_reserve_fraction=0.5,
                                                              battery_critical_fraction=0.1))
        mission = _mission(cfg)
        mission.state = MissionState.SEARCHING
        mission._mission_start_s = 0.0
        mission.tick(1.0, _telem(position_m=(3.0, 3.0, 1.0), battery_fraction=0.3, timestamp_s=1.0))
        assert mission.state in (MissionState.RETURN_HOME, MissionState.LAND_REQUESTED, MissionState.LANDED)

    def test_geofence_risk_triggers_return_home(self):
        mission = _mission()
        mission.state = MissionState.SEARCHING
        mission._mission_start_s = 0.0
        # deep outside the geofence built from a 10x10 area with 1m margin
        mission.tick(1.0, _telem(position_m=(50.0, 50.0, 1.0), timestamp_s=1.0))
        assert mission.state in (MissionState.RETURN_HOME, MissionState.LAND_REQUESTED, MissionState.LANDED)

    def test_malformed_observation_vehicle_id_mismatch_is_rejected_not_silently_used(self):
        mission = _mission()
        bad = SensorObservation(vehicle_id="not-drone0", sensor_timestamp_s=1.0, sensor_latency_s=0.0,
                                 fov_deg=360.0, range_returns_m=(), occluded=(), dropout=False,
                                 pose_uncertainty_m=0.0,
                                 detections=(DetectionCandidate(candidate_id="x", position_m=(1.0, 1.0),
                                                                 frame=Frame.LOCAL_ENU, confidence=0.9,
                                                                 localization_uncertainty_m=0.1),))
        result = mission.ingest_observation(bad, 1.0)
        assert result is None
        assert mission.malformed_observations == 1
        assert mission.detections_used_for_confirmation == 0

    def test_malformed_observation_dropout_with_detections_is_rejected(self):
        mission = _mission()
        bad = SensorObservation(vehicle_id="drone0", sensor_timestamp_s=1.0, sensor_latency_s=0.0, fov_deg=360.0,
                                 range_returns_m=(), occluded=(), dropout=True, pose_uncertainty_m=0.0,
                                 detections=(DetectionCandidate(candidate_id="x", position_m=(1.0, 1.0),
                                                                 frame=Frame.LOCAL_ENU, confidence=0.9,
                                                                 localization_uncertainty_m=0.1),))
        result = mission.ingest_observation(bad, 1.0)
        assert result is None
        assert mission.malformed_observations == 1

    def test_stale_observation_is_rejected(self):
        mission = _mission(_config(stale_observation_max_age_s=1.0))
        stale = SensorObservation(vehicle_id="drone0", sensor_timestamp_s=0.0, sensor_latency_s=0.0,
                                   fov_deg=360.0, range_returns_m=(), occluded=(), dropout=False,
                                   pose_uncertainty_m=0.0,
                                   detections=(DetectionCandidate(candidate_id="x", position_m=(1.0, 1.0),
                                                                   frame=Frame.LOCAL_ENU, confidence=0.9,
                                                                   localization_uncertainty_m=0.1),))
        result = mission.ingest_observation(stale, 5.0)   # 5s old, max age 1.0s
        assert result is None
        assert mission.stale_observations_rejected == 1


# ==========================================================================
# 7. reproducibility
# ==========================================================================

class TestReproducibility:
    def test_same_seed_same_summary(self, tmp_path):
        cfg1 = _config(seed=99)
        cfg2 = _config(seed=99)
        s1 = run_sar_mission_offline(cfg1, out_dir=str(tmp_path / "a"))
        s2 = run_sar_mission_offline(cfg2, out_dir=str(tmp_path / "b"))
        for key in ("final_state", "waypoints_completed", "observations_generated",
                    "detections_used_for_confirmation", "confirmed_victim_ids", "missed_victims"):
            assert s1[key] == s2[key], f"mismatch in {key!r}: {s1[key]!r} vs {s2[key]!r}"

    def test_different_seed_can_differ(self):
        s1 = run_sar_mission_offline(_config(seed=1))
        s2 = run_sar_mission_offline(_config(seed=999))
        # not asserting they MUST differ (they could coincidentally match),
        # just that the run completes safely for multiple distinct seeds
        assert s1["final_state"] in ("LANDED", "FAILED")
        assert s2["final_state"] in ("LANDED", "FAILED")

    def test_offline_run_writes_all_required_files(self, tmp_path):
        run_sar_mission_offline(_config(seed=5), out_dir=str(tmp_path))
        for name in ("manifest.json", "mission_events.jsonl", "telemetry.csv", "safety_decisions.csv",
                     "commands.csv", "detections.csv", "actuator_log.csv", "summary.json"):
            assert os.path.isfile(tmp_path / name), f"missing required output file: {name}"
