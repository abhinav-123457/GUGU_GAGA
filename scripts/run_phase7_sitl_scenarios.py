"""Phase 7 required-scenario runner - see docs/PHASE7_SITL_INTEGRATION.md.

Scenarios 1-17 are a lightweight, deterministic, PyBullet-free harness
driving a FakeSITLTransport (directly, and through a SITLAdapter) with
synthetic AdapterCommand/ground-truth-state sequences - same style as
scripts/run_phase6_adapter_scenarios.py's own synthetic scenarios.
Scenarios 18-20 rerun the real, full FloodSearchMission with
autopilot_path="fake_sitl" (real PyBullet physics, real SwarmController/
SafetySupervisor/SITLAdapter/FakeSITLTransport pipeline): a distributed-
consensus scenario with deliberately delayed communication, Phase 4.1's
exact obstacle-wedging config, and the extreme communication_radius=3m
stress scenario (kept visible here too, per Phase 5's own commitment not
to silently drop it, now one layer further down the stack).

No real ArduPilot/PX4 SITL binary is installed or executed anywhere in
this script - see docs/PHASE7_SITL_INTEGRATION.md's explicit statement.

Usage:
    python scripts/run_phase7_sitl_scenarios.py                 # all 20
    python scripts/run_phase7_sitl_scenarios.py nominal_operation_one_vehicle
"""
import json
import os
import random
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from swarm_sim.autopilot import AdapterCommand, AutopilotMode, ConnectionState
from swarm_sim.autopilot.sitl import SITLAdapter
from swarm_sim.contracts import Command, CommandType, Frame
from swarm_sim.sitl.fake_transport import FakeSITLTransport
from swarm_sim.sitl.telemetry import FailureType

OUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "results", "phase7_sitl_scenarios")


def _command(vid="d", ctype=CommandType.VELOCITY_SETPOINT, frame=Frame.LOCAL_ENU,
             vel=(1.0, 0.0, 0.0), t=0.0, ttl=1.0, source="SafetySupervisor"):
    return Command(vehicle_id=vid, command_type=ctype, frame=frame, desired_position_m=None,
                   desired_velocity_mps=(vel if ctype not in (CommandType.HOLD, CommandType.LAND, CommandType.ABORT) else None),
                   yaw_rad=None, yaw_rate_radps=None, timestamp_s=t, expiration_time_s=t + ttl,
                   source=source, confidence=0.9)


def _push_state(transport, vid, t, position=(0.0, 0.0, 3.0), velocity=(0.0, 0.0, 0.0), battery=1.0, estimator_valid=True):
    transport.push_vehicle_state(vid, position, velocity, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                                  battery, estimator_valid, now_s=t)


class ScenarioResult:
    """Fields match Phase 7 requirement 9's own metric list."""

    def __init__(self, name, seed=None, config=None, num_vehicles=1):
        self.name = name
        self.seed = seed
        self.configuration = config or {}
        self.num_vehicles = num_vehicles
        self.candidate_command_count = 0
        self.safety_decision_counts = None
        self.adapter_accepted_count = 0
        self.adapter_rejected_count = 0
        self.transport_success_count = 0
        self.transport_failure_count = 0
        self.acknowledgement_count = 0
        self.reject_reasons = {}
        self.stale_telemetry_count = 0
        self.heartbeat_loss_count = 0
        self.failsafe_transitions = 0
        self.mode_transitions = 0
        self.sequence_replay_count = 0
        self.namespace_isolation_violations = 0
        self.ground_contacts = None
        self.obstacle_contacts = None
        self.drone_drone_contacts = None
        self.min_altitude_m = None
        self.min_clearance_m = None
        self.deterministic_replay_result = None
        self.cpu_time_per_vehicle_per_step_s = None
        self.memory_per_vehicle_bytes_approx = None

    def record_adapter_result(self, result):
        self.candidate_command_count += 1
        if result.accepted:
            self.adapter_accepted_count += 1
        else:
            self.adapter_rejected_count += 1
            self.reject_reasons[result.reason] = self.reject_reasons.get(result.reason, 0) + 1
            if result.reason == "stale_telemetry":
                self.stale_telemetry_count += 1
            if result.reason == "heartbeat_lost":
                self.heartbeat_loss_count += 1
            if result.reason in ("sequence_replayed_conflicting", "sequence_replayed_stale"):
                self.sequence_replay_count += 1

    def record_transport_result(self, transport_result):
        if transport_result.success:
            self.transport_success_count += 1
        else:
            self.transport_failure_count += 1

    def to_dict(self):
        return {
            "scenario": self.name,
            "seed": self.seed,
            "configuration": self.configuration,
            "num_vehicles": self.num_vehicles,
            "candidate_command_count": self.candidate_command_count,
            "safety_decisions": self.safety_decision_counts,
            "adapter_accepted_count": self.adapter_accepted_count,
            "adapter_rejected_count": self.adapter_rejected_count,
            "transport_success_count": self.transport_success_count,
            "transport_failure_count": self.transport_failure_count,
            "acknowledgement_count": self.acknowledgement_count,
            "reject_reasons": dict(self.reject_reasons),
            "stale_telemetry_count": self.stale_telemetry_count,
            "heartbeat_loss_count": self.heartbeat_loss_count,
            "failsafe_transitions": self.failsafe_transitions,
            "mode_transitions": self.mode_transitions,
            "sequence_replay_count": self.sequence_replay_count,
            "namespace_isolation_violations": self.namespace_isolation_violations,
            "ground_contacts": self.ground_contacts,
            "obstacle_contacts": self.obstacle_contacts,
            "drone_drone_contacts": self.drone_drone_contacts,
            "min_altitude_m": self.min_altitude_m,
            "min_clearance_m": self.min_clearance_m,
            "deterministic_replay_result": self.deterministic_replay_result,
            "cpu_time_per_vehicle_per_step_s": self.cpu_time_per_vehicle_per_step_s,
            "memory_per_vehicle_bytes_approx": self.memory_per_vehicle_bytes_approx,
        }


def _memory_proxy_bytes(adapter):
    return (sys.getsizeof(adapter.command_history) + sys.getsizeof(adapter.mode_transition_log)
            + sys.getsizeof(adapter.connection_transition_log))


def _namespace_isolation_check(transport, vehicle_ids):
    """Structural cross-check reused by every synthetic scenario: no
    vehicle's telemetry/ack ever reports another vehicle's identity."""
    violations = 0
    for vid in vehicle_ids:
        telem = transport.receive_telemetry(vid)
        ack = transport.receive_ack(vid)
        if telem.vehicle_id != vid or telem.namespace != transport.registry.namespace_of(vid):
            violations += 1
        if ack.vehicle_id != vid or ack.namespace != transport.registry.namespace_of(vid):
            violations += 1
    return violations


# --- 1-17: synthetic FakeSITLTransport/SITLAdapter scenarios ----------------

def scenario_nominal_operation_one_vehicle():
    config = dict(num_vehicles=1, seed=1)
    t = FakeSITLTransport(vehicle_ids=("drone0",), rng=random.Random(config["seed"]))
    t.start()
    a = SITLAdapter("drone0", t.registry.namespace_of("drone0"), t)
    a.connect()
    result = ScenarioResult("nominal_operation_one_vehicle", seed=config["seed"], config=config, num_vehicles=1)
    t0 = time.perf_counter()
    for k in range(20):
        tk = k * (1 / 24)
        _push_state(t, "drone0", tk)
        r = a.send_command(AdapterCommand(command=_command(vid="drone0", t=tk, vel=(1.0, 0.0, 0.0)), sequence=k))
        result.record_adapter_result(r)
    result.cpu_time_per_vehicle_per_step_s = (time.perf_counter() - t0) / 20
    result.mode_transitions = len(a.mode_transition_log)
    result.failsafe_transitions = len(a.connection_transition_log)
    result.namespace_isolation_violations = _namespace_isolation_check(t, ["drone0"])
    result.memory_per_vehicle_bytes_approx = _memory_proxy_bytes(a)
    return result


def scenario_nominal_operation_six_vehicles():
    config = dict(num_vehicles=6, seed=2)
    vids = tuple(f"drone{i}" for i in range(6))
    t = FakeSITLTransport(vehicle_ids=vids, rng=random.Random(config["seed"]))
    t.start()
    adapters = {vid: SITLAdapter(vid, t.registry.namespace_of(vid), t) for vid in vids}
    for a in adapters.values():
        a.connect()
    result = ScenarioResult("nominal_operation_six_vehicles", seed=config["seed"], config=config, num_vehicles=6)
    t0 = time.perf_counter()
    steps = 0
    for k in range(20):
        tk = k * (1 / 24)
        for i, vid in enumerate(vids):
            _push_state(t, vid, tk, position=(float(i), 0.0, 3.0))
            r = adapters[vid].send_command(AdapterCommand(command=_command(vid=vid, t=tk, vel=(1.0, 0.0, 0.0)), sequence=k))
            result.record_adapter_result(r)
            steps += 1
    elapsed = time.perf_counter() - t0
    result.cpu_time_per_vehicle_per_step_s = elapsed / steps
    result.namespace_isolation_violations = _namespace_isolation_check(t, vids)
    result.memory_per_vehicle_bytes_approx = float(np.mean([_memory_proxy_bytes(a) for a in adapters.values()]))
    return result


def scenario_command_latency():
    config = dict(command_latency_s=0.2, seed=3)
    t = FakeSITLTransport(vehicle_ids=("d",), command_latency_s=config["command_latency_s"], rng=random.Random(config["seed"]))
    t.start()
    _push_state(t, "d", 0.0)
    a = SITLAdapter("d", t.registry.namespace_of("d"), t)
    a.connect()
    result = ScenarioResult("command_latency", seed=config["seed"], config=config)
    result.record_adapter_result(a.send_command(AdapterCommand(command=_command(t=0.0, ttl=0.1), sequence=0)))  # latency > ttl
    result.record_adapter_result(a.send_command(AdapterCommand(command=_command(t=0.03, ttl=1.0), sequence=1)))  # latency within ttl
    return result


def scenario_telemetry_latency():
    config = dict(telemetry_latency_s=1.0, seed=4)
    t = FakeSITLTransport(vehicle_ids=("d",), telemetry_latency_s=config["telemetry_latency_s"], rng=random.Random(config["seed"]))
    t.start()
    _push_state(t, "d", 0.0, position=(5.0, 0.0, 3.0))
    result = ScenarioResult("telemetry_latency", seed=config["seed"], config=config)
    not_yet = t.receive_telemetry("d").position_m is None
    t.set_sim_time(1.0)
    arrived = t.receive_telemetry("d").position_m == (5.0, 0.0, 3.0)
    result.candidate_command_count = 0
    result.reject_reasons = {"telemetry_delayed_then_delivered": int(not_yet and arrived)}
    return result


def scenario_command_packet_loss():
    config = dict(command_packet_loss_prob=0.3, seed=5)
    t = FakeSITLTransport(vehicle_ids=("d",), command_packet_loss_prob=config["command_packet_loss_prob"],
                           rng=random.Random(config["seed"]))
    t.start()
    a = SITLAdapter("d", t.registry.namespace_of("d"), t)
    a.connect()
    result = ScenarioResult("command_packet_loss", seed=config["seed"], config=config)
    for k in range(50):
        tk = k * (1 / 24)
        _push_state(t, "d", tk)
        r = a.send_command(AdapterCommand(command=_command(t=tk), sequence=k))
        result.record_adapter_result(r)
    return result


def scenario_telemetry_packet_loss():
    config = dict(telemetry_packet_loss_prob=1.0, seed=6)
    t = FakeSITLTransport(vehicle_ids=("d",), telemetry_packet_loss_prob=config["telemetry_packet_loss_prob"],
                           rng=random.Random(config["seed"]))
    t.start()
    _push_state(t, "d", 0.0, position=(9.0, 9.0, 9.0))
    result = ScenarioResult("telemetry_packet_loss", seed=config["seed"], config=config)
    result.reject_reasons = {"telemetry_dropped": int(t.receive_telemetry("d").position_m is None)}
    return result


def scenario_heartbeat_loss():
    config = dict(seed=7)
    t = FakeSITLTransport(vehicle_ids=("d",), rng=random.Random(config["seed"]))
    t.start()
    _push_state(t, "d", 0.0)
    t.inject_failure("d", FailureType.HEARTBEAT_LOSS)
    a = SITLAdapter("d", t.registry.namespace_of("d"), t)
    a.connect()
    result = ScenarioResult("heartbeat_loss", seed=config["seed"], config=config)
    result.record_adapter_result(a.send_command(AdapterCommand(command=_command(t=0.0), sequence=0)))
    result.failsafe_transitions = len(a.connection_transition_log)
    return result


def scenario_estimator_failure():
    config = dict(seed=8)
    t = FakeSITLTransport(vehicle_ids=("d",), rng=random.Random(config["seed"]))
    t.start()
    _push_state(t, "d", 0.0, estimator_valid=False)
    t.inject_failure("d", FailureType.ESTIMATOR_INVALID)
    a = SITLAdapter("d", t.registry.namespace_of("d"), t)
    a.connect()
    result = ScenarioResult("estimator_failure", seed=config["seed"], config=config)
    # Navigation is refused; HOLD (what SafetySupervisor would emit for
    # ESTIMATOR_INVALID) is still accepted - same policy as Phase 6.
    result.record_adapter_result(a.send_command(AdapterCommand(command=_command(t=0.0), sequence=0)))
    result.record_adapter_result(
        a.send_command(AdapterCommand(command=_command(t=0.03, ctype=CommandType.HOLD), sequence=1))
    )
    return result


def scenario_battery_critical():
    config = dict(seed=9)
    t = FakeSITLTransport(vehicle_ids=("d",), rng=random.Random(config["seed"]))
    t.start()
    _push_state(t, "d", 0.0, battery=0.05)
    t.inject_failure("d", FailureType.BATTERY_CRITICAL)
    a = SITLAdapter("d", t.registry.namespace_of("d"), t)
    a.connect()
    result = ScenarioResult("battery_critical", seed=config["seed"], config=config)
    result.record_adapter_result(a.send_command(AdapterCommand(command=_command(t=0.0), sequence=0)))
    return result


def scenario_vehicle_disconnect():
    config = dict(seed=10)
    t = FakeSITLTransport(vehicle_ids=("d",), rng=random.Random(config["seed"]))
    t.start()
    _push_state(t, "d", 0.0)
    a = SITLAdapter("d", t.registry.namespace_of("d"), t)
    a.connect()
    result = ScenarioResult("vehicle_disconnect", seed=config["seed"], config=config)
    result.record_adapter_result(a.send_command(AdapterCommand(command=_command(t=0.0), sequence=0)))
    t.inject_failure("d", FailureType.VEHICLE_DISCONNECT)
    result.record_adapter_result(a.send_command(AdapterCommand(command=_command(t=0.03), sequence=1)))
    result.failsafe_transitions = len(a.connection_transition_log)
    return result


def scenario_transport_stop():
    config = dict(seed=11)
    t = FakeSITLTransport(vehicle_ids=("d",), rng=random.Random(config["seed"]))
    t.start()
    _push_state(t, "d", 0.0)
    a = SITLAdapter("d", t.registry.namespace_of("d"), t)
    a.connect()
    result = ScenarioResult("transport_stop", seed=config["seed"], config=config)
    result.record_adapter_result(a.send_command(AdapterCommand(command=_command(t=0.0), sequence=0)))
    t.stop()
    result.record_adapter_result(a.send_command(AdapterCommand(command=_command(t=0.03), sequence=1)))
    return result


def scenario_namespace_collision_attempt():
    config = dict(seed=12)
    result = ScenarioResult("namespace_collision_attempt", seed=config["seed"], config=config)
    try:
        FakeSITLTransport(vehicle_ids=("drone0", "drone0"), rng=random.Random(config["seed"]))
        result.reject_reasons = {"collision_correctly_rejected": 0}
    except Exception as e:
        result.reject_reasons = {"collision_correctly_rejected": 1, "error_type": type(e).__name__}
    return result


def scenario_duplicate_command():
    config = dict(seed=13)
    t = FakeSITLTransport(vehicle_ids=("d",), rng=random.Random(config["seed"]))
    t.start()
    _push_state(t, "d", 0.0)
    a = SITLAdapter("d", t.registry.namespace_of("d"), t)
    a.connect()
    result = ScenarioResult("duplicate_command", seed=config["seed"], config=config)
    result.record_adapter_result(a.send_command(AdapterCommand(command=_command(t=0.0, vel=(1.0, 0.0, 0.0)), sequence=0)))
    result.record_adapter_result(a.send_command(AdapterCommand(command=_command(t=0.0, vel=(1.0, 0.0, 0.0)), sequence=0)))  # idempotent
    result.record_adapter_result(a.send_command(AdapterCommand(command=_command(t=0.0, vel=(9.0, 0.0, 0.0)), sequence=0)))  # conflicting
    result.sequence_replay_count = result.reject_reasons.get("sequence_replayed_conflicting", 0)
    return result


def scenario_sequence_replay():
    config = dict(seed=14)
    t = FakeSITLTransport(vehicle_ids=("d",), rng=random.Random(config["seed"]))
    t.start()
    _push_state(t, "d", 0.0)
    a = SITLAdapter("d", t.registry.namespace_of("d"), t)
    a.connect()
    result = ScenarioResult("sequence_replay", seed=config["seed"], config=config)
    result.record_adapter_result(a.send_command(AdapterCommand(command=_command(t=0.0), sequence=5)))
    result.record_adapter_result(a.send_command(AdapterCommand(command=_command(t=0.0), sequence=3)))  # stale replay
    return result


def scenario_operator_abort():
    config = dict(seed=15)
    t = FakeSITLTransport(vehicle_ids=("d",), rng=random.Random(config["seed"]))
    t.start()
    _push_state(t, "d", 0.0)
    a = SITLAdapter("d", t.registry.namespace_of("d"), t)
    a.connect()
    result = ScenarioResult("operator_abort", seed=config["seed"], config=config)
    result.record_adapter_result(a.send_command(AdapterCommand(command=_command(t=0.0), sequence=0)))
    result.record_adapter_result(a.send_command(AdapterCommand(command=_command(t=0.03, ctype=CommandType.ABORT), sequence=1)))
    result.mode_transitions = len(a.mode_transition_log)
    assert a.mode == AutopilotMode.ABORT
    return result


def scenario_land_requested():
    config = dict(seed=16)
    t = FakeSITLTransport(vehicle_ids=("d",), rng=random.Random(config["seed"]))
    t.start()
    _push_state(t, "d", 0.0)
    a = SITLAdapter("d", t.registry.namespace_of("d"), t)
    a.connect()
    result = ScenarioResult("land_requested", seed=config["seed"], config=config)
    result.record_adapter_result(a.send_command(AdapterCommand(command=_command(t=0.0, ctype=CommandType.LAND), sequence=0)))
    result.mode_transitions = len(a.mode_transition_log)
    assert a.mode == AutopilotMode.LAND and a.state == ConnectionState.LANDING
    return result


def scenario_return_to_safe_point():
    config = dict(seed=17)
    t = FakeSITLTransport(vehicle_ids=("d",), rng=random.Random(config["seed"]))
    t.start()
    a = SITLAdapter("d", t.registry.namespace_of("d"), t)
    a.connect()
    result = ScenarioResult("return_to_safe_point", seed=config["seed"], config=config)
    r = a.request_return_to_launch(now_s=0.0)
    result.candidate_command_count = 1
    if r.accepted:
        result.adapter_accepted_count = 1
    else:
        result.adapter_rejected_count = 1
    result.mode_transitions = len(a.mode_transition_log)
    assert a.mode == AutopilotMode.RTL
    return result


SYNTHETIC_SCENARIOS = {
    "nominal_operation_one_vehicle": scenario_nominal_operation_one_vehicle,
    "nominal_operation_six_vehicles": scenario_nominal_operation_six_vehicles,
    "command_latency": scenario_command_latency,
    "telemetry_latency": scenario_telemetry_latency,
    "command_packet_loss": scenario_command_packet_loss,
    "telemetry_packet_loss": scenario_telemetry_packet_loss,
    "heartbeat_loss": scenario_heartbeat_loss,
    "estimator_failure": scenario_estimator_failure,
    "battery_critical": scenario_battery_critical,
    "vehicle_disconnect": scenario_vehicle_disconnect,
    "transport_stop": scenario_transport_stop,
    "namespace_collision_attempt": scenario_namespace_collision_attempt,
    "duplicate_command": scenario_duplicate_command,
    "sequence_replay": scenario_sequence_replay,
    "operator_abort": scenario_operator_abort,
    "land_requested": scenario_land_requested,
    "return_to_safe_point": scenario_return_to_safe_point,
}


# --- 18-20: real full-mission scenarios (autopilot_path="fake_sitl") --------

def _mission_scenario(name, overrides, base=None):
    from swarm_sim.config import MissionConfig
    from swarm_sim.mission import FloodSearchMission

    cfg_kwargs = dict(base or dict(num_drones=5, num_victims=4, num_obstacles=2, duration_sec=25.0, seed=7))
    cfg_kwargs.update(overrides)
    cfg_kwargs.setdefault("autopilot_path", "fake_sitl")

    def run():
        cfg = MissionConfig(**cfg_kwargs)
        mission = FloodSearchMission(cfg)
        t0 = time.perf_counter()
        mission_result = mission.run()
        wall_s = time.perf_counter() - t0
        return mission, mission_result, wall_s

    mission, mission_result, wall_s = run()
    diag = mission_result["diagnostics"]

    result = ScenarioResult(name, seed=cfg_kwargs["seed"], config=cfg_kwargs, num_vehicles=cfg_kwargs["num_drones"])
    result.candidate_command_count = mission_result["adapter_candidate_count"]
    result.safety_decision_counts = mission_result["safety_state_counts"]
    result.adapter_accepted_count = mission_result["adapter_accepted_count"]
    result.adapter_rejected_count = mission_result["adapter_rejected_count"]
    result.reject_reasons = mission_result["adapter_reject_reasons"]
    result.mode_transitions = mission_result["adapter_mode_transition_count"]
    result.failsafe_transitions = mission_result["adapter_failsafe_transition_count"]
    result.stale_telemetry_count = mission_result["adapter_stale_telemetry_count"]
    result.sequence_replay_count = mission_result["adapter_replayed_command_count"]
    result.min_clearance_m = mission_result["min_ground_truth_clearance_m"]
    result.obstacle_contacts = diag["drone_obstacle_contact_count"]
    result.ground_contacts = diag.get("drone_ground_contact_count")
    result.drone_drone_contacts = diag["drone_drone_contact_count"]
    result.transport_success_count = mission.sitl_transport.total_command_history_count()
    result.namespace_isolation_violations = _namespace_isolation_check(mission.sitl_transport, mission._drone_ids)
    result.cpu_time_per_vehicle_per_step_s = (
        mission_result["adapter_cpu_time_s"] / max(1, mission_result["adapter_candidate_count"])
    )
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
    d["consensus_mode"] = mission_result["consensus_mode"]
    d["wall_time_s"] = wall_s
    return d


def scenario_delayed_distributed_consensus():
    """Distributed consensus (Phase 5) with deliberately delayed
    communication (comm_latency_steps > 0, a finite communication_radius),
    now also routed through the Phase 7 fake-SITL boundary - proves the
    added transport layer does not change distributed consensus's own
    behavior even when messages are genuinely delayed, not just under
    perfect communication."""
    return _mission_scenario("delayed_distributed_consensus", dict(
        consensus_mode="distributed", communication_radius=10.0, comm_latency_steps=4,
        comm_dropout_base=0.02, comm_dropout_at_max_range=0.2,
        autopilot_command_latency_s=0.05, autopilot_telemetry_latency_s=0.05,
    ))


def scenario_obstacle_wedging():
    """Phase 4.1's exact wedging config, now also routed through the
    Phase 7 fake-SITL boundary - the required regression check that
    ground contacts stay at zero."""
    return _mission_scenario("obstacle_wedging", dict(
        num_drones=6, num_victims=5, num_obstacles=4, duration_sec=90.0,
        victim_sensor_latency_steps=10, neighbor_sensor_latency_steps=10, obstacle_sensor_latency_steps=5,
        safety_enabled=True,
    ), base={"seed": 42})


def scenario_extreme_comm_partition_radius_3m():
    """NON-ACCEPTANCE STRESS SCENARIO (see
    docs/PHASE5_DISTRIBUTED_CONSENSUS.md's "A pre-existing safety edge
    case" section) - kept visible here too, per that phase's own
    commitment not to silently drop it now that Phase 7 adds another
    layer between the candidate and PyBullet. Single representative seed
    (7) - see scripts/run_phase5_consensus_scenarios.py for the full
    7-seed characterization; this is only a check that the Phase 7
    fake-SITL boundary does not change the finding's nature."""
    return _mission_scenario("extreme_comm_partition_radius_3m_STRESS_NOT_ACCEPTANCE", dict(
        communication_radius=3.0, comm_dropout_base=0.02, comm_dropout_at_max_range=0.3, comm_latency_steps=2,
    ), base={"num_drones": 5, "num_victims": 4, "num_obstacles": 2, "duration_sec": 25.0, "seed": 7})


MISSION_SCENARIOS = {
    "delayed_distributed_consensus": scenario_delayed_distributed_consensus,
    "obstacle_wedging": scenario_obstacle_wedging,
    "extreme_comm_partition_radius_3m": scenario_extreme_comm_partition_radius_3m,
}

SCENARIOS = {**SYNTHETIC_SCENARIOS, **MISSION_SCENARIOS}

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
        d = fn() if name in MISSION_SCENARIOS else fn().to_dict()
        d["reproducibility"] = _reproducibility_record()
        d["actual_sitl_binary_executed"] = False   # see docs/PHASE7_SITL_INTEGRATION.md
        print(json.dumps(d, default=str), flush=True)
        all_results.append(d)
    with open(os.path.join(OUT_DIR, "phase7_scenario_results.json"), "w") as f:
        json.dump(all_results, f, indent=2, default=str)


if __name__ == "__main__":
    main()
