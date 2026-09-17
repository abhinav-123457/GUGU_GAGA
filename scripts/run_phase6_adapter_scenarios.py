"""Phase 6 required-scenario runner - see docs/PHASE6_AUTOPILOT_ADAPTERS.md.

Scenarios 1-12 are a lightweight, deterministic, PyBullet-free harness
driving a MockAdapter directly with synthetic AdapterCommand/
VehicleTelemetry sequences - the fast, precise way to prove each required
adapter behavior in isolation (same style as Phase 4's safety-scenario
runner). Scenarios 13-15 rerun the real, full FloodSearchMission (real
PyBullet physics, real SwarmController/SafetySupervisor/adapter
pipeline): Phase 4.1's exact obstacle-wedging config, a Phase 5
distributed-consensus scenario, and the extreme communication_radius=3m
stress scenario (kept visible here too, per Phase 5's own commitment not
to silently drop it) - reusing swarm_sim.mission.FloodSearchMission
directly, same as scripts/run_phase5_consensus_scenarios.py.

Metrics not applicable to a given scenario's nature are reported as null
rather than a misleading zero (e.g. "minimum clearance" for a synthetic
adapter-only scenario that never runs physics) - same disclosed-
approximation style as docs/PHASE2_DIAGNOSTICS.md and
docs/PHASE5_DISTRIBUTED_CONSENSUS.md.

Usage:
    python scripts/run_phase6_adapter_scenarios.py                 # all 15
    python scripts/run_phase6_adapter_scenarios.py nominal_operation
"""
import json
import os
import random
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from swarm_sim.autopilot import AdapterCommand, AutopilotMode, ConnectionState, MockAdapter
from swarm_sim.autopilot.frames import convert_command_frame
from swarm_sim.autopilot.types import VehicleTelemetry
from swarm_sim.contracts import Command, CommandType, Frame

OUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "results", "phase6_scenarios")


def _command(vid="drone0", ctype=CommandType.VELOCITY_SETPOINT, frame=Frame.LOCAL_ENU,
             vel=(1.0, 0.0, 0.0), t=0.0, ttl=1.0, source="SafetySupervisor"):
    return Command(vehicle_id=vid, command_type=ctype, frame=frame, desired_position_m=None,
                   desired_velocity_mps=(vel if ctype not in (CommandType.HOLD, CommandType.LAND, CommandType.ABORT) else None),
                   yaw_rad=None, yaw_rate_radps=None, timestamp_s=t, expiration_time_s=t + ttl,
                   source=source, confidence=0.9)


def _telemetry(t=0.0, vid="drone0", frame=Frame.LOCAL_ENU):
    return VehicleTelemetry(
        vehicle_id=vid, timestamp_s=t, frame=frame, position_m=(0.0, 0.0, 3.0), velocity_mps=(0.0, 0.0, 0.0),
        acceleration_mps2=None, attitude_rad=(0.0, 0.0, 0.0), angular_velocity_radps=None, battery_fraction=1.0,
        estimator_valid=True, connection_state=ConnectionState.CONNECTED, autopilot_mode=AutopilotMode.OFFBOARD,
        armed=False, failsafe=False, sequence=0,
    )


class ScenarioResult:
    def __init__(self, name):
        self.name = name
        self.candidate_command_count = 0
        self.safety_decision_counts = {}
        self.adapter_accepted_count = 0
        self.adapter_rejected_count = 0
        self.reject_reasons = {}
        self.mode_transitions = 0
        self.failsafe_transitions = 0
        self.stale_command_count = 0
        self.replayed_command_count = 0
        self.frame_conversion_count = 0
        self.frame_rejection_count = 0
        self.min_clearance_m = None
        self.drone_obstacle_contacts = None
        self.drone_ground_contacts = None
        self.drone_drone_contacts = None
        self.deterministic_replay_result = None
        self.cpu_times_s = []
        self.memory_per_vehicle_bytes_approx = None

    def record_result(self, result, cpu_time_s=None):
        self.candidate_command_count += 1
        if result.accepted:
            self.adapter_accepted_count += 1
        else:
            self.adapter_rejected_count += 1
            self.reject_reasons[result.reason] = self.reject_reasons.get(result.reason, 0) + 1
            if result.reason == "stale_telemetry":
                self.stale_command_count += 1
            if result.reason in ("sequence_replayed_conflicting", "sequence_replayed_stale"):
                self.replayed_command_count += 1
        if cpu_time_s is not None:
            self.cpu_times_s.append(cpu_time_s)

    def to_dict(self):
        return {
            "scenario": self.name,
            "candidate_command_count": self.candidate_command_count,
            "safety_decisions": dict(self.safety_decision_counts),
            "adapter_accepted_count": self.adapter_accepted_count,
            "adapter_rejected_count": self.adapter_rejected_count,
            "reject_reasons": dict(self.reject_reasons),
            "mode_transitions": self.mode_transitions,
            "failsafe_transitions": self.failsafe_transitions,
            "stale_command_count": self.stale_command_count,
            "replayed_command_count": self.replayed_command_count,
            "frame_conversion_count": self.frame_conversion_count,
            "frame_rejection_count": self.frame_rejection_count,
            "min_clearance_m": self.min_clearance_m,
            "drone_obstacle_contacts": self.drone_obstacle_contacts,
            "drone_ground_contacts": self.drone_ground_contacts,
            "drone_drone_contacts": self.drone_drone_contacts,
            "deterministic_replay_result": self.deterministic_replay_result,
            "cpu_time_per_adapter_call_s_mean": (float(np.mean(self.cpu_times_s)) if self.cpu_times_s else None),
            "memory_per_vehicle_bytes_approx": self.memory_per_vehicle_bytes_approx,
        }


def _memory_proxy_bytes(adapter):
    return (sys.getsizeof(adapter.command_history) + sys.getsizeof(adapter.mode_transition_log)
            + sys.getsizeof(adapter.connection_transition_log))


# --- 1-12: synthetic MockAdapter scenarios ----------------------------------

def scenario_nominal_operation():
    result = ScenarioResult("nominal_operation")
    a = MockAdapter("drone0")
    a.connect()
    for k in range(20):
        t = k * (1 / 24)
        a.push_ground_truth_state(_telemetry(t=t))
        t0 = time.perf_counter()
        r = a.send_command(AdapterCommand(command=_command(t=t, vel=(1.0, 0.0, 0.0)), sequence=k))
        result.record_result(r, time.perf_counter() - t0)
    result.mode_transitions = len(a.mode_transition_log)
    result.failsafe_transitions = len(a.connection_transition_log)
    result.memory_per_vehicle_bytes_approx = _memory_proxy_bytes(a)
    return result


def scenario_command_expiry():
    result = ScenarioResult("command_expiry")
    a = MockAdapter("drone0", command_latency_s=0.05)
    a.connect()
    a.push_ground_truth_state(_telemetry(t=0.0))
    r = a.send_command(AdapterCommand(command=_command(t=0.0, ttl=0.01), sequence=0))
    result.record_result(r)
    return result


def scenario_adapter_disconnect():
    result = ScenarioResult("adapter_disconnect")
    a = MockAdapter("drone0")
    a.connect()
    a.push_ground_truth_state(_telemetry(t=0.0))
    r0 = a.send_command(AdapterCommand(command=_command(t=0.0), sequence=0))
    result.record_result(r0)
    a.inject_disconnect(at_s=0.1)
    r1 = a.send_command(AdapterCommand(command=_command(t=0.1), sequence=1))
    result.record_result(r1)
    result.failsafe_transitions = len(a.connection_transition_log)
    return result


def scenario_stale_telemetry():
    result = ScenarioResult("stale_telemetry")
    a = MockAdapter("drone0", telemetry_stale_timeout_s=0.5)
    a.connect()
    a.push_ground_truth_state(_telemetry(t=0.0))
    r = a.send_command(AdapterCommand(command=_command(t=10.0, ttl=5.0), sequence=0))
    result.record_result(r)
    return result


def scenario_frame_mismatch():
    result = ScenarioResult("frame_mismatch")
    a = MockAdapter("drone0", operating_frame=Frame.LOCAL_ENU)
    a.connect()
    a.push_ground_truth_state(_telemetry(t=0.0))
    # Rejected: NED command sent straight to an ENU-operating adapter.
    r0 = a.send_command(AdapterCommand(command=_command(t=0.0, frame=Frame.LOCAL_NED), sequence=0))
    result.record_result(r0)
    result.frame_rejection_count = 1
    # Accepted: same command, explicitly converted first.
    converted = convert_command_frame(_command(t=0.1, frame=Frame.LOCAL_NED), Frame.LOCAL_ENU)
    r1 = a.send_command(AdapterCommand(command=converted, sequence=1))
    result.record_result(r1)
    result.frame_conversion_count = 1
    return result


def scenario_estimator_failure():
    """Mirrors the mission-level policy (estimator invalid -> HOLD/LAND
    per the safety decision): a HOLD command (what SafetySupervisor would
    produce for ESTIMATOR_INVALID) must still be accepted and honored."""
    result = ScenarioResult("estimator_failure")
    a = MockAdapter("drone0")
    a.connect()
    a.push_ground_truth_state(_telemetry(t=0.0))
    r = a.send_command(AdapterCommand(command=_command(t=0.0, ctype=CommandType.HOLD), sequence=0))
    result.record_result(r)
    result.mode_transitions = len(a.mode_transition_log)
    return result


def scenario_operator_abort():
    result = ScenarioResult("operator_abort")
    a = MockAdapter("drone0")
    a.connect()
    a.push_ground_truth_state(_telemetry(t=0.0))
    a.send_command(AdapterCommand(command=_command(t=0.0, ctype=CommandType.VELOCITY_SETPOINT), sequence=0))
    r = a.send_command(AdapterCommand(command=_command(t=0.1, ctype=CommandType.ABORT), sequence=1))
    result.record_result(r)
    result.mode_transitions = len(a.mode_transition_log)
    assert a.mode == AutopilotMode.ABORT
    return result


def scenario_land_requested():
    result = ScenarioResult("land_requested")
    a = MockAdapter("drone0")
    a.connect()
    a.push_ground_truth_state(_telemetry(t=0.0))
    r = a.send_command(AdapterCommand(command=_command(t=0.0, ctype=CommandType.LAND), sequence=0))
    result.record_result(r)
    result.mode_transitions = len(a.mode_transition_log)
    assert a.mode == AutopilotMode.LAND and a.state == ConnectionState.LANDING
    return result


def scenario_return_to_safe_point():
    result = ScenarioResult("return_to_safe_point")
    a = MockAdapter("drone0")
    a.connect()
    r = a.request_return_to_launch(now_s=0.0)
    result.candidate_command_count = 1
    if r.accepted:
        result.adapter_accepted_count = 1
    else:
        result.adapter_rejected_count = 1
    result.mode_transitions = len(a.mode_transition_log)
    assert a.mode == AutopilotMode.RTL
    return result


def scenario_repeated_command_replay():
    result = ScenarioResult("repeated_command_replay")
    a = MockAdapter("drone0")
    a.connect()
    a.push_ground_truth_state(_telemetry(t=0.0))
    r0 = a.send_command(AdapterCommand(command=_command(t=0.0, vel=(1.0, 0.0, 0.0)), sequence=0))
    result.record_result(r0)
    r1 = a.send_command(AdapterCommand(command=_command(t=0.0, vel=(1.0, 0.0, 0.0)), sequence=0))  # idempotent
    result.record_result(r1)
    r2 = a.send_command(AdapterCommand(command=_command(t=0.0, vel=(9.0, 0.0, 0.0)), sequence=0))  # conflicting
    result.record_result(r2)
    return result


def scenario_packet_loss():
    result = ScenarioResult("packet_loss")
    a = MockAdapter("drone0", command_packet_loss_prob=0.3, rng=random.Random(7))
    a.connect()
    for k in range(50):
        t = k * (1 / 24)
        a.push_ground_truth_state(_telemetry(t=t))
        r = a.send_command(AdapterCommand(command=_command(t=t), sequence=k))
        result.record_result(r)
    return result


def scenario_command_latency():
    result = ScenarioResult("command_latency")
    a = MockAdapter("drone0", command_latency_s=0.2)
    a.connect()
    a.push_ground_truth_state(_telemetry(t=0.0))
    r0 = a.send_command(AdapterCommand(command=_command(t=0.0, ttl=0.1), sequence=0))  # latency exceeds ttl
    result.record_result(r0)
    r1 = a.send_command(AdapterCommand(command=_command(t=0.1, ttl=1.0), sequence=1))  # latency within ttl
    result.record_result(r1)
    return result


# --- 13-15: real full-mission scenarios -------------------------------------

def _mission_scenario(name, overrides, base=None):
    from swarm_sim.config import MissionConfig
    from swarm_sim.mission import FloodSearchMission

    cfg_kwargs = dict(base or dict(num_drones=5, num_victims=4, num_obstacles=2, duration_sec=25.0, seed=7))
    cfg_kwargs.update(overrides)
    cfg_kwargs.setdefault("autopilot_path", "mock_adapter")

    def run():
        cfg = MissionConfig(**cfg_kwargs)
        mission = FloodSearchMission(cfg)
        t0 = time.perf_counter()
        mission_result = mission.run()
        wall_s = time.perf_counter() - t0
        return mission, mission_result, wall_s

    mission, mission_result, wall_s = run()
    diag = mission_result["diagnostics"]

    result = ScenarioResult(name)
    result.candidate_command_count = mission_result["adapter_candidate_count"]
    result.safety_decision_counts = mission_result["safety_state_counts"]
    result.adapter_accepted_count = mission_result["adapter_accepted_count"]
    result.adapter_rejected_count = mission_result["adapter_rejected_count"]
    result.reject_reasons = mission_result["adapter_reject_reasons"]
    result.mode_transitions = mission_result["adapter_mode_transition_count"]
    result.failsafe_transitions = mission_result["adapter_failsafe_transition_count"]
    result.stale_command_count = mission_result["adapter_stale_telemetry_count"]
    result.replayed_command_count = mission_result["adapter_replayed_command_count"]
    result.min_clearance_m = mission_result["min_ground_truth_clearance_m"]
    result.drone_obstacle_contacts = diag["drone_obstacle_contact_count"]
    result.drone_ground_contacts = diag.get("drone_ground_contact_count")
    result.drone_drone_contacts = diag["drone_drone_contact_count"]
    result.cpu_times_s = [mission_result["adapter_cpu_time_s"] / max(1, mission_result["adapter_candidate_count"])]
    result.memory_per_vehicle_bytes_approx = float(np.mean([
        _memory_proxy_bytes(adapter) for adapter in mission.autopilot_adapters.values()
    ])) if mission.autopilot_adapters else None

    _, replay_result, _ = run()
    result.deterministic_replay_result = (
        "identical" if (mission_result["victims_found"] == replay_result["victims_found"]
                         and mission_result["adapter_accepted_count"] == replay_result["adapter_accepted_count"]
                         and diag.get("drone_ground_contact_count") == replay_result["diagnostics"].get("drone_ground_contact_count"))
        else "DIVERGED"
    )

    d = result.to_dict()
    d["victims_found"] = f"{mission_result['victims_found']}/{mission_result['total_victims']}"
    d["swarm_connectivity_fraction"] = mission_result["swarm_connectivity_fraction"]
    d["consensus_mode"] = mission_result["consensus_mode"]
    d["wall_time_s"] = wall_s
    return d


def scenario_obstacle_wedging():
    """Phase 4.1's exact wedging config, now also routed through the
    Phase 6 mock-adapter boundary - the required regression check that
    ground contacts stay at zero."""
    return _mission_scenario("obstacle_wedging", dict(
        num_drones=6, num_victims=5, num_obstacles=4, duration_sec=90.0,
        victim_sensor_latency_steps=10, neighbor_sensor_latency_steps=10, obstacle_sensor_latency_steps=5,
        safety_enabled=True,
    ), base={"seed": 42})


def scenario_distributed_consensus():
    """A Phase 5 distributed-consensus scenario (perfect communication),
    now also routed through the Phase 6 mock-adapter boundary."""
    return _mission_scenario("phase5_distributed_consensus", dict(
        consensus_mode="distributed", communication_radius=1e6,
        comm_dropout_base=0.0, comm_dropout_at_max_range=0.0, comm_latency_steps=0,
    ))


def scenario_extreme_comm_partition_radius_3m():
    """NON-ACCEPTANCE STRESS SCENARIO (see
    docs/PHASE5_DISTRIBUTED_CONSENSUS.md's "A pre-existing safety edge
    case" section) - kept visible here too, per that phase's own
    commitment not to silently drop it now that Phase 6 adds another
    layer between the candidate and PyBullet. Single representative seed
    (7) - see scripts/run_phase5_consensus_scenarios.py's
    extreme_comm_partition_radius_3m for the full 7-seed characterization;
    this is not a repeat of that investigation, only a check that the
    Phase 6 adapter boundary does not change the finding's nature."""
    return _mission_scenario("extreme_comm_partition_radius_3m_STRESS_NOT_ACCEPTANCE", dict(
        communication_radius=3.0, comm_dropout_base=0.02, comm_dropout_at_max_range=0.3, comm_latency_steps=2,
    ), base={"num_drones": 5, "num_victims": 4, "num_obstacles": 2, "duration_sec": 25.0, "seed": 7})


SCENARIOS = {
    "nominal_operation": scenario_nominal_operation,
    "command_expiry": scenario_command_expiry,
    "adapter_disconnect": scenario_adapter_disconnect,
    "stale_telemetry": scenario_stale_telemetry,
    "frame_mismatch": scenario_frame_mismatch,
    "estimator_failure": scenario_estimator_failure,
    "operator_abort": scenario_operator_abort,
    "land_requested": scenario_land_requested,
    "return_to_safe_point": scenario_return_to_safe_point,
    "repeated_command_replay": scenario_repeated_command_replay,
    "packet_loss": scenario_packet_loss,
    "command_latency": scenario_command_latency,
    "obstacle_wedging": scenario_obstacle_wedging,
    "distributed_consensus": scenario_distributed_consensus,
    "extreme_comm_partition_radius_3m": scenario_extreme_comm_partition_radius_3m,
}

_MISSION_SCENARIOS = {"obstacle_wedging", "distributed_consensus", "extreme_comm_partition_radius_3m"}


_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _git_commit_hash():
    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=_REPO_ROOT).decode().strip()
    except Exception:
        return None


def _reproducibility_record():
    return {
        "python_version": sys.version.split()[0],
        "commit_hash": _git_commit_hash(),
        "command_used": "python " + " ".join(sys.argv),
    }


def main():
    names = [n for n in sys.argv[1:] if not n.startswith("-")] or list(SCENARIOS.keys())
    os.makedirs(OUT_DIR, exist_ok=True)
    all_results = []
    for name in names:
        fn = SCENARIOS[name]
        d = fn() if name in _MISSION_SCENARIOS else fn().to_dict()
        d["reproducibility"] = _reproducibility_record()
        print(json.dumps(d, default=str), flush=True)
        all_results.append(d)
    with open(os.path.join(OUT_DIR, "phase6_scenario_results.json"), "w") as f:
        json.dump(all_results, f, indent=2, default=str)


if __name__ == "__main__":
    main()
