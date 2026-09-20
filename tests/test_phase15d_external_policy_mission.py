"""Phase 15D functional tests - see
docs/PHASE15_SWARM124_LANGOSTINO_INTEGRATION.md and
swarm_sim/external_policy_mission.py. Architecture/AST checks live in
tests/test_phase15d_external_policy_architecture.py.

Mirrors tests/test_phase14_sar_mission.py's own style: most tests drive
`ExternalPolicyMission.tick()` directly with hand-built
`VehicleTelemetry` objects, plus a smaller set of end-to-end tests
against the full `run_external_policy_mission_offline()` runner.
"""
from __future__ import annotations

import dataclasses
import itertools

import pytest

from swarm_sim.autopilot.types import AutopilotMode, ConnectionState, VehicleTelemetry
from swarm_sim.contracts import Frame, SafetyState
from swarm_sim.external.swarm124_policy_adapter import ScriptedSwarm124Policy
from swarm_sim.external_policy_mission import (
    ALLOWED_TRANSITIONS, ExternalPolicyMission, ExternalPolicyMissionConfig, ExternalPolicyMissionState,
    TERMINAL_STATES, run_external_policy_mission_offline,
)
from swarm_sim.safety_supervisor import SafetySupervisorConfig

_seq = itertools.count()


def _telem(position_m=(0.0, 0.0, 1.0), velocity_mps=(0.0, 0.0, 0.0), battery_fraction=1.0,
           estimator_valid=True, failsafe=False, timestamp_s=0.0, armed=True,
           mode=AutopilotMode.OFFBOARD) -> VehicleTelemetry:
    return VehicleTelemetry(
        vehicle_id="drone0", timestamp_s=timestamp_s, frame=Frame.LOCAL_ENU, position_m=position_m,
        velocity_mps=velocity_mps, acceleration_mps2=(0.0, 0.0, 0.0), attitude_rad=(0.0, 0.0, 0.0),
        angular_velocity_radps=(0.0, 0.0, 0.0), battery_fraction=battery_fraction,
        estimator_valid=estimator_valid, connection_state=ConnectionState.CONNECTED, autopilot_mode=mode,
        armed=armed, failsafe=failsafe, sequence=next(_seq),
    )


def _config(**overrides) -> ExternalPolicyMissionConfig:
    defaults = dict(target_xy=(5.0, 5.0), search_altitude_m=1.0, goal_reach_tolerance_m=0.5,
                     max_speed_mps=0.25, mission_timeout_s=60.0, command_timeout_s=5.0,
                     landing_timeout_s=30.0, dt_s=1.0)
    defaults.update(overrides)
    return ExternalPolicyMissionConfig(**defaults)


def _mission(config=None) -> ExternalPolicyMission:
    config = config or _config()
    policy = ScriptedSwarm124Policy(target_xy=config.target_xy, max_speed_mps=config.max_speed_mps,
                                     seed=config.seed, run_id=config.run_id)
    return ExternalPolicyMission(config, policy)


def _run_ticks(mission, telems, dt_s=1.0):
    results = []
    now_s = 0.0
    for telem in telems:
        telem = dataclasses.replace(telem, timestamp_s=now_s)
        tick_result = mission.tick(now_s, telem)
        results.append(tick_result)
        now_s += dt_s
    return results


class TestStateMachine:
    def test_illegal_transition_raises(self):
        mission = _mission()
        with pytest.raises(ValueError):
            mission._transition(ExternalPolicyMissionState.LANDED, "not_allowed_from_init")

    def test_every_allowed_transition_target_is_a_real_state(self):
        for state, targets in ALLOWED_TRANSITIONS.items():
            assert isinstance(state, ExternalPolicyMissionState)
            for t in targets:
                assert isinstance(t, ExternalPolicyMissionState)

    def test_terminal_states_have_no_outgoing_transitions(self):
        for state in TERMINAL_STATES:
            assert ALLOWED_TRANSITIONS[state] == frozenset()

    def test_preflight_check_failure_with_no_position(self):
        mission = _mission()
        telem = _telem(position_m=None)
        mission.tick(0.0, telem)
        assert mission.state == ExternalPolicyMissionState.FAILED
        assert mission.failure_reason == "preflight_check_failed"


class TestHappyPath:
    def test_reaches_target_and_lands(self):
        mission = _mission(_config(target_xy=(1.0, 0.0), mission_timeout_s=120.0))
        state = mission
        pos = [0.0, 0.0, 1.0]
        now_s = 0.0
        for _ in range(200):
            telem = _telem(position_m=tuple(pos), timestamp_s=now_s)
            tick_result = mission.tick(now_s, telem)
            if tick_result.adapter_command is not None:
                cmd = tick_result.adapter_command.command
                pos[0] += cmd.desired_velocity_mps[0] * 1.0
                pos[1] += cmd.desired_velocity_mps[1] * 1.0
                pos[2] += cmd.desired_velocity_mps[2] * 1.0
            if mission.state in TERMINAL_STATES:
                break
            now_s += 1.0
        assert mission.state == ExternalPolicyMissionState.LANDED
        assert mission.goal_reached is True

    def test_never_returns_a_command_without_a_safety_decision(self):
        mission = _mission(_config(target_xy=(1.0, 0.0)))
        results = _run_ticks(mission, [_telem(position_m=(0.0, 0.0, 1.0))] * 3)
        for r in results:
            if r.adapter_command is not None:
                assert r.decision is not None
                assert r.decision.accepted is True
                assert r.decision.filtered_command is r.adapter_command.command

    def test_policy_version_and_checksum_recorded_in_summary(self):
        mission = _mission()
        summary = mission.build_summary(mode="offline")
        assert summary["policy_version"] == "scripted-v1"
        assert "model_checksum" in summary


class TestFaultHandling:
    def test_no_position_telemetry_fails_immediately(self):
        mission = _mission()
        mission.tick(0.0, _telem(position_m=(0.0, 0.0, 1.0)))
        mission.tick(1.0, _telem(position_m=None, timestamp_s=1.0))
        assert mission.state == ExternalPolicyMissionState.FAILED
        assert mission.failure_reason == "no_position_telemetry"

    def test_estimator_invalid_persisted_fails_after_grace_period(self):
        config = _config(estimator_invalid_grace_s=2.0)
        mission = _mission(config)
        mission.tick(0.0, _telem(estimator_valid=True, timestamp_s=0.0))
        mission.tick(1.0, _telem(estimator_valid=False, timestamp_s=1.0))
        assert mission.state != ExternalPolicyMissionState.FAILED
        mission.tick(3.5, _telem(estimator_valid=False, timestamp_s=3.5))
        assert mission.state == ExternalPolicyMissionState.FAILED
        assert mission.failure_reason == "estimator_invalid_persisted"

    def test_stale_telemetry_persisted_fails_after_grace_period(self):
        config = _config(stale_telemetry_max_age_s=2.0)
        mission = _mission(config)
        mission.tick(0.0, _telem(timestamp_s=0.0))
        mission.tick(5.0, _telem(timestamp_s=0.0))   # stale from the very next tick
        assert mission.state != ExternalPolicyMissionState.FAILED
        mission.tick(9.0, _telem(timestamp_s=0.0))
        assert mission.state == ExternalPolicyMissionState.FAILED
        assert mission.failure_reason == "stale_telemetry_persisted"

    def test_mission_timeout_lands_in_place_not_return_home(self):
        config = _config(target_xy=(1000.0, 1000.0), mission_timeout_s=2.0, landing_timeout_s=30.0)
        mission = _mission(config)
        mission.tick(0.0, _telem(position_m=(0.0, 0.0, 1.0), timestamp_s=0.0))
        mission.tick(1.0, _telem(position_m=(0.05, 0.05, 1.0), timestamp_s=1.0))
        result = mission.tick(3.0, _telem(position_m=(0.1, 0.1, 1.0), timestamp_s=3.0))
        assert mission.state == ExternalPolicyMissionState.LAND_REQUESTED
        assert any(e.get("event") == "mission_timeout" for e in mission.events)

    def test_landing_timeout_exceeded_fails(self):
        config = _config(landing_timeout_s=1.0)
        mission = _mission(config)
        mission._transition(ExternalPolicyMissionState.RUNNING, "policy_mission_started")
        mission._mission_start_s = 0.0
        mission._transition(ExternalPolicyMissionState.LAND_REQUESTED, "policy_target_reached")
        mission.tick(0.0, _telem(position_m=(5.0, 5.0, 1.0), timestamp_s=0.0))
        mission.tick(5.0, _telem(position_m=(5.0, 5.0, 1.0), timestamp_s=5.0))
        assert mission.state == ExternalPolicyMissionState.FAILED
        assert mission.failure_reason == "landing_timeout_exceeded"


class TestSafetyVeto:
    """`build_geofence()` always encloses home AND target with a full
    buffer (see its own docstring), so the policy's own target is safe
    from GEOFENCE_RISK by construction - exactly like SARMission's search
    area (swarm_sim/sar_mission.py). SARMission's own equivalent tests
    (tests/test_phase14_sar_mission.py's TestGeofenceHeadroom) verify that
    headroom directly rather than engineering exact geometry through a
    full tick() simulation to accidentally trigger a live-noise-only
    condition; these tests follow the same approach, unit-testing
    _apply_safety_emergency_state's state-mapping directly."""

    @pytest.mark.parametrize("emergency_state", [
        SafetyState.GEOFENCE_RISK, SafetyState.RETURN_TO_SAFE_POINT, SafetyState.LOW_BATTERY,
        SafetyState.DEGRADED_LINK,
    ])
    def test_soft_emergency_tiers_force_landing_not_continued_travel(self, emergency_state):
        mission = _mission()
        mission._transition(ExternalPolicyMissionState.RUNNING, "policy_mission_started")
        mission._apply_safety_emergency_state(emergency_state)
        assert mission.state == ExternalPolicyMissionState.LAND_REQUESTED

    def test_abort_tier_aborts_the_mission(self):
        mission = _mission()
        mission._transition(ExternalPolicyMissionState.RUNNING, "policy_mission_started")
        mission._apply_safety_emergency_state(SafetyState.ABORT)
        assert mission.state == ExternalPolicyMissionState.ABORTED

    def test_normal_emergency_state_does_not_change_mission_state(self):
        mission = _mission()
        mission._transition(ExternalPolicyMissionState.RUNNING, "policy_mission_started")
        mission._apply_safety_emergency_state(SafetyState.NORMAL)
        assert mission.state == ExternalPolicyMissionState.RUNNING


class TestExternalPolicyRejection:
    """Uses a fake policy object (matching ScriptedSwarm124Policy's own
    interface: policy_version/model_checksum attributes and an
    act(observation, action_expiry_s) method) to force a rejection at the
    external-contract validation layer, proving the mission never sends a
    command built from a rejected action and aborts safely if rejections
    persist - see swarm_sim/external/external_contracts.py."""

    class _NanDirectionPolicy:
        policy_version = "fake-v1"
        model_checksum = None

        def act(self, observation, action_expiry_s):
            from swarm_sim.external.swarm124_policy_adapter import Swarm124Action
            return Swarm124Action(
                vehicle_id=observation.vehicle_id, timestamp_s=observation.timestamp_s,
                direction_xy=(float("nan"), 0.0), speed_mps=0.1, turn_rate_radps=0.0,
                policy_version=self.policy_version, seed=0, run_id="fake-run", action_expiry_s=action_expiry_s,
                model_checksum=self.model_checksum,
            )

    def test_rejected_action_never_produces_an_adapter_command(self):
        config = _config(command_timeout_s=100.0)
        mission = ExternalPolicyMission(config, self._NanDirectionPolicy())
        result = mission.tick(0.0, _telem(position_m=(0.0, 0.0, 1.0), timestamp_s=0.0))
        assert result.adapter_command is None
        assert result.external_rejected is True
        assert mission.external_rejections == 1

    def test_persisted_external_rejection_fails_safely_after_command_timeout(self):
        config = _config(command_timeout_s=2.0)
        mission = ExternalPolicyMission(config, self._NanDirectionPolicy())
        mission.tick(0.0, _telem(position_m=(0.0, 0.0, 1.0), timestamp_s=0.0))
        mission.tick(1.0, _telem(position_m=(0.0, 0.0, 1.0), timestamp_s=1.0))
        mission.tick(3.5, _telem(position_m=(0.0, 0.0, 1.0), timestamp_s=3.5))
        assert mission.state == ExternalPolicyMissionState.FAILED
        assert mission.failure_reason == "command_timeout_external_rejected"


class TestOfflineRunner:
    def test_run_external_policy_mission_offline_reaches_goal_and_lands(self, tmp_path):
        config = _config(target_xy=(1.0, 0.0), mission_timeout_s=120.0)
        summary = run_external_policy_mission_offline(config, out_dir=str(tmp_path))
        assert summary["final_state"] == "LANDED"
        assert summary["goal_reached"] is True
        assert summary["command_rejections"] == 0
        assert summary["external_rejections"] == 0
        assert (tmp_path / "telemetry.csv").exists()
        assert (tmp_path / "commands.csv").exists()
        assert (tmp_path / "safety_decisions.csv").exists()
        assert (tmp_path / "summary.json").exists()

    def test_offline_run_is_deterministic_for_the_same_seed(self, tmp_path):
        config = _config(target_xy=(2.0, 3.0), mission_timeout_s=120.0, seed=7)
        summary1 = run_external_policy_mission_offline(config)
        summary2 = run_external_policy_mission_offline(config)
        assert summary1["final_state"] == summary2["final_state"]
        assert summary1["maximum_altitude_m"] == summary2["maximum_altitude_m"]
        assert summary1["policy_actions_generated"] == summary2["policy_actions_generated"]

    def test_never_arms_or_takes_off_default_starts_armed_and_airborne(self, tmp_path):
        """This module never arms/takes off a vehicle itself (see module
        docstring) - the offline runner's own FakeSITLTransport starts the
        vehicle already at search_altitude_m, standing in for a real
        arm+takeoff that already happened before this mission's tick loop
        is ever entered, exactly like the live script will do."""
        config = _config(search_altitude_m=1.5, target_xy=(0.1, 0.1), mission_timeout_s=120.0)
        summary = run_external_policy_mission_offline(config)
        assert summary["target_altitude_m"] == 1.5
