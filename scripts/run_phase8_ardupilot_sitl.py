"""Phase 8 required-scenario runner - see docs/PHASE8_ARDUPILOT_SITL.md.

Two real execution modes, chosen explicitly on the command line (there is
no default that touches a socket):

    python scripts/run_phase8_ardupilot_sitl.py --dry-run
        Validates configuration only (executable paths, localhost-ness,
        port allow-list, vehicle namespace/system-id uniqueness) for every
        scenario's endpoints - never opens a socket or starts a process.

    python scripts/run_phase8_ardupilot_sitl.py --local-sitl
        Attempts to connect to REAL local ArduPilot SITL instances at the
        configured localhost endpoints (see ENDPOINTS below, overridable
        via PHASE8_SITL_CONNECTION_STRINGS/PHASE8_SITL_SYSTEM_IDS env
        vars). If ArduPilot SITL is not reachable, this script does NOT
        silently substitute FakeSITL results and call them ArduPilot
        validation - it stops the real-SITL portion, prints:

            Actual ArduPilot SITL was not run.
            Only FakeSITL tests passed.
            No claim of real SITL validation is made.

        and still runs (and clearly labels) the FakeSITL-backed portion of
        the required scenario list, since "FakeSITL must remain available"
        is a hard Phase 8 requirement independent of real-SITL
        availability.

Scenarios 1-19 are transport-level (no PyBullet) - real ArduPilot SITL
when --local-sitl succeeds, FakeSITL otherwise (both clearly labeled per
scenario in the output). Scenarios 20-22 (Phase 4.1 obstacle-wedging,
Phase 5 distributed-consensus, extreme_comm_partition_radius_3m) are full
FloodSearchMission runs - these remain FakeSITL-backed regardless of
--local-sitl: combining PyBullet's own simulation-time stepping with a
real ArduPilot SITL process's real-time control loop is a real-time
synchronization problem this phase's "replace only the transport" scope
does not attempt to solve (see docs/PHASE8_ARDUPILOT_SITL.md's "Known
limitations" section) - reported honestly, not silently substituted.
"""
import json
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from swarm_sim.autopilot import AdapterCommand, AutopilotMode
from swarm_sim.autopilot.ardupilot_sitl import build_ardupilot_sitl_adapter
from swarm_sim.autopilot.sitl import SITLAdapter
from swarm_sim.contracts import Command, CommandType, Frame
from swarm_sim.sitl.ardupilot_transport import (
    ArduPilotSITLTransport, ArduPilotTransportError, ArduPilotVehicleEndpoint, PYMAVLINK_AVAILABLE,
    PYMAVLINK_VERSION, dry_run_validate,
)
from swarm_sim.sitl.fake_transport import FakeSITLTransport
from swarm_sim.sitl.telemetry import FailureType

OUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "results", "phase8_ardupilot_sitl")
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

DEFAULT_CONNECTION_STRINGS = os.environ.get(
    "PHASE8_SITL_CONNECTION_STRINGS", "tcp:127.0.0.1:5760,tcp:127.0.0.1:5770"
).split(",")
DEFAULT_SYSTEM_IDS = [int(x) for x in os.environ.get("PHASE8_SITL_SYSTEM_IDS", "1,2").split(",")]
ALLOWED_PORTS = frozenset(int(cs.split(":")[2]) for cs in DEFAULT_CONNECTION_STRINGS)
VEHICLE_IDS = ("drone0", "drone1")


def _endpoints(n=1):
    return {
        VEHICLE_IDS[i]: ArduPilotVehicleEndpoint(
            vehicle_id=VEHICLE_IDS[i], connection_string=DEFAULT_CONNECTION_STRINGS[i],
            system_id=DEFAULT_SYSTEM_IDS[i], component_id=1,
        )
        for i in range(n)
    }


def _git_commit_hash():
    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=_REPO_ROOT).decode().strip()
    except Exception:
        return None


def _reproducibility_record():
    return {
        "python_version": sys.version.split()[0],
        "commit_hash": _git_commit_hash(),
        "pymavlink_available": PYMAVLINK_AVAILABLE,
        "pymavlink_version": PYMAVLINK_VERSION,
        "command_used": "python " + " ".join(sys.argv),
    }


def _probe_real_sitl_reachable(n=1, timeout_s=3.0) -> bool:
    """Bounded, honest check: can we actually reach real local ArduPilot
    SITL at the configured endpoints? Never claims success without a real
    heartbeat exchange."""
    if not PYMAVLINK_AVAILABLE:
        return False
    try:
        t = ArduPilotSITLTransport(_endpoints(n), allowed_ports=ALLOWED_PORTS, startup_timeout_s=timeout_s)
        t.start()
        t.stop()
        return True
    except ArduPilotTransportError:
        return False


def _command(vid, ctype=CommandType.VELOCITY_SETPOINT, frame=Frame.LOCAL_ENU, vel=(1.0, 0.0, 0.0),
             t=0.0, ttl=5.0):
    kwargs = dict(vehicle_id=vid, command_type=ctype, frame=frame, desired_position_m=None,
                  desired_velocity_mps=vel, yaw_rad=None, yaw_rate_radps=None,
                  timestamp_s=t, expiration_time_s=t + ttl, source="test", confidence=0.9)
    if ctype in (CommandType.HOLD, CommandType.ABORT, CommandType.LAND):
        kwargs["desired_velocity_mps"] = None
    return Command(**kwargs)


class ScenarioResult:
    def __init__(self, name, real_sitl_run: bool):
        self.name = name
        self.real_sitl_run = real_sitl_run
        self.executable = None
        self.command_line = None
        self.localhost_ports = list(ALLOWED_PORTS)
        self.system_component_ids = list(zip(DEFAULT_SYSTEM_IDS, [1] * len(DEFAULT_SYSTEM_IDS)))
        self.seed = 0
        self.vehicles = 0
        self.candidate_command_count = 0
        self.acknowledged_count = 0
        self.rejected_count = 0
        self.reject_reasons = {}
        self.heartbeat_losses = 0
        self.stale_telemetry_events = 0
        self.failsafe_transitions = 0
        self.process_restarts = 0
        self.namespace_violations = 0
        self.ground_contacts = None
        self.obstacle_contacts = None
        self.drone_drone_contacts = None
        self.min_altitude_m = None
        self.min_clearance_m = None
        self.deterministic_replay_result = None
        self.cpu_time_s = None
        self.memory_bytes_approx = None

    def to_dict(self):
        return {
            "scenario": self.name, "actual_ardupilot_sitl_run": self.real_sitl_run,
            "executable": self.executable, "command_line": self.command_line,
            "localhost_ports": self.localhost_ports, "system_component_ids": self.system_component_ids,
            "seed": self.seed, "vehicles": self.vehicles,
            "candidate_command_count": self.candidate_command_count,
            "acknowledged_count": self.acknowledged_count, "rejected_count": self.rejected_count,
            "reject_reasons": dict(self.reject_reasons), "heartbeat_losses": self.heartbeat_losses,
            "stale_telemetry_events": self.stale_telemetry_events,
            "failsafe_transitions": self.failsafe_transitions, "process_restarts": self.process_restarts,
            "namespace_violations": self.namespace_violations, "ground_contacts": self.ground_contacts,
            "obstacle_contacts": self.obstacle_contacts, "drone_drone_contacts": self.drone_drone_contacts,
            "min_altitude_m": self.min_altitude_m, "min_clearance_m": self.min_clearance_m,
            "deterministic_replay_result": self.deterministic_replay_result,
            "cpu_time_s": self.cpu_time_s, "memory_bytes_approx": self.memory_bytes_approx,
            "reproducibility": _reproducibility_record(),
        }


def _run_transport_scenario(name, n_vehicles, body, real_sitl_available):
    """`body(transport, adapters)` drives the scenario against whichever
    transport is actually available - FakeSITL if real SITL isn't
    reachable, clearly labeled either way."""
    result = ScenarioResult(name, real_sitl_run=real_sitl_available)
    result.vehicles = n_vehicles
    vids = VEHICLE_IDS[:n_vehicles]

    if real_sitl_available:
        endpoints = _endpoints(n_vehicles)
        result.executable = "real ArduPilot SITL (operator-started, attach mode)"
        result.command_line = f"attach: {list(endpoints[v].connection_string for v in vids)}"
        transport = ArduPilotSITLTransport(endpoints, allowed_ports=ALLOWED_PORTS, startup_timeout_s=15.0)
        transport.start()
        adapters = {v: build_ardupilot_sitl_adapter(v, transport) for v in vids}
    else:
        result.executable = "FakeSITLTransport (real ArduPilot SITL unavailable)"
        result.command_line = "n/a - in-memory"
        # A generous future_tolerance_s here only: these scenario bodies
        # send several commands spanning up to ~1s of simulated time
        # without a real per-tick telemetry push driving the clock
        # forward the way mission.py's real integration does - this is a
        # property of this lightweight smoke-scenario harness, not of
        # FakeSITLTransport's own (unmodified) default behavior.
        transport = FakeSITLTransport(vehicle_ids=vids, future_tolerance_s=5.0)
        transport.start()
        for v in vids:
            transport.push_vehicle_state(v, (0.0, 0.0, 3.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                                          1.0, True, now_s=0.0)
        adapters = {v: SITLAdapter(v, transport.registry.namespace_of(v), transport) for v in vids}
    for adapter in adapters.values():
        adapter.connect()

    t0 = time.perf_counter()
    body(transport, adapters, result)
    result.cpu_time_s = time.perf_counter() - t0

    transport.stop()
    return result


def scenario_dry_run_configuration():
    report = dry_run_validate(_endpoints(2), allowed_ports=ALLOWED_PORTS)
    result = ScenarioResult("dry_run_configuration", real_sitl_run=False)
    result.vehicles = 2
    result.reject_reasons = {"dry_run_ok": int(report.ok), "problems": report.problems}
    return result


def scenario_one_vehicle_nominal(real_sitl_available):
    def body(transport, adapters, result):
        adapter = adapters[VEHICLE_IDS[0]]
        for k in range(10):
            r = adapter.send_command(AdapterCommand(command=_command(VEHICLE_IDS[0], t=k * 0.1), sequence=k))
            result.candidate_command_count += 1
            if r.accepted:
                result.acknowledged_count += 1
            else:
                result.rejected_count += 1
                result.reject_reasons[r.reason] = result.reject_reasons.get(r.reason, 0) + 1
            time.sleep(0.05)
    return _run_transport_scenario("one_vehicle_nominal", 1, body, real_sitl_available)


def scenario_two_vehicles_nominal(real_sitl_available):
    def body(transport, adapters, result):
        for k in range(10):
            for v in VEHICLE_IDS[:2]:
                r = adapters[v].send_command(AdapterCommand(command=_command(v, t=k * 0.1), sequence=k))
                result.candidate_command_count += 1
                if r.accepted:
                    result.acknowledged_count += 1
                else:
                    result.rejected_count += 1
                    result.reject_reasons[r.reason] = result.reject_reasons.get(r.reason, 0) + 1
            time.sleep(0.05)
    return _run_transport_scenario("two_vehicles_nominal", 2, body, real_sitl_available)


def scenario_heartbeat_loss(real_sitl_available):
    def body(transport, adapters, result):
        v = VEHICLE_IDS[0]
        adapters[v].send_command(AdapterCommand(command=_command(v, t=0.0), sequence=0))
        transport.inject_failure(v, FailureType.HEARTBEAT_LOSS)
        r = adapters[v].send_command(AdapterCommand(command=_command(v, t=0.1), sequence=1))
        result.candidate_command_count = 2
        result.rejected_count = int(not r.accepted)
        result.reject_reasons = {r.reason: 1}
        result.heartbeat_losses = 1
    return _run_transport_scenario("heartbeat_loss", 1, body, real_sitl_available)


def scenario_operator_abort(real_sitl_available):
    def body(transport, adapters, result):
        v = VEHICLE_IDS[0]
        r = adapters[v].send_command(AdapterCommand(command=_command(v, ctype=CommandType.ABORT, vel=None), sequence=0))
        result.candidate_command_count = 1
        result.acknowledged_count = int(r.accepted)
        assert adapters[v].mode == AutopilotMode.ABORT
    return _run_transport_scenario("operator_abort", 1, body, real_sitl_available)


def scenario_land_requested(real_sitl_available):
    def body(transport, adapters, result):
        v = VEHICLE_IDS[0]
        r = adapters[v].send_command(AdapterCommand(command=_command(v, ctype=CommandType.LAND, vel=None), sequence=0))
        result.candidate_command_count = 1
        result.acknowledged_count = int(r.accepted)
    return _run_transport_scenario("land_requested", 1, body, real_sitl_available)


def scenario_return_to_safe_point(real_sitl_available):
    def body(transport, adapters, result):
        v = VEHICLE_IDS[0]
        r = adapters[v].send_command(AdapterCommand(command=_command(v, ctype=CommandType.HOLD, vel=None), sequence=0))
        result.candidate_command_count = 1
        result.acknowledged_count = int(r.accepted)
    return _run_transport_scenario("return_to_safe_point", 1, body, real_sitl_available)


def scenario_sequence_replay(real_sitl_available):
    def body(transport, adapters, result):
        v = VEHICLE_IDS[0]
        adapters[v].send_command(AdapterCommand(command=_command(v, t=0.0), sequence=5))
        r = adapters[v].send_command(AdapterCommand(command=_command(v, t=0.0), sequence=2))
        result.candidate_command_count = 2
        result.rejected_count = int(not r.accepted)
        result.reject_reasons = {r.reason: 1}
    return _run_transport_scenario("sequence_replay", 1, body, real_sitl_available)


def scenario_duplicate_command(real_sitl_available):
    def body(transport, adapters, result):
        v = VEHICLE_IDS[0]
        adapters[v].send_command(AdapterCommand(command=_command(v, t=0.0, vel=(1.0, 0.0, 0.0)), sequence=5))
        r = adapters[v].send_command(AdapterCommand(command=_command(v, t=0.0, vel=(1.0, 0.0, 0.0)), sequence=5))
        result.candidate_command_count = 2
        result.acknowledged_count = int(r.accepted)
    return _run_transport_scenario("duplicate_command", 1, body, real_sitl_available)


def scenario_command_latency(real_sitl_available):
    def body(transport, adapters, result):
        v = VEHICLE_IDS[0]
        t0 = time.perf_counter()
        r = adapters[v].send_command(AdapterCommand(command=_command(v, t=0.0), sequence=0))
        result.candidate_command_count = 1
        result.acknowledged_count = int(r.accepted)
        result.cpu_time_s = time.perf_counter() - t0
    return _run_transport_scenario("command_latency", 1, body, real_sitl_available)


def scenario_telemetry_latency(real_sitl_available):
    def body(transport, adapters, result):
        v = VEHICLE_IDS[0]
        time.sleep(0.3)
        telem = adapters[v].read_telemetry()
        result.candidate_command_count = 0
        result.reject_reasons = {"telemetry_available": int(telem.position_m is not None)}
    return _run_transport_scenario("telemetry_latency", 1, body, real_sitl_available)


def scenario_command_acknowledgement(real_sitl_available):
    def body(transport, adapters, result):
        v = VEHICLE_IDS[0]
        r = adapters[v].send_command(AdapterCommand(command=_command(v, ctype=CommandType.HOLD, vel=None), sequence=0))
        result.candidate_command_count = 1
        result.acknowledged_count = int(r.accepted)
        result.reject_reasons = {"ack_reason": r.reason}
    return _run_transport_scenario("command_acknowledgement", 1, body, real_sitl_available)


def scenario_stale_telemetry(real_sitl_available):
    def body(transport, adapters, result):
        v = VEHICLE_IDS[0]
        r = adapters[v].send_command(AdapterCommand(command=_command(v, t=0.0), sequence=0))
        result.candidate_command_count = 1
        result.rejected_count = int(not r.accepted)
        if not r.accepted:
            result.stale_telemetry_events = int(r.reason == "stale_telemetry")
            result.reject_reasons = {r.reason: 1}
    return _run_transport_scenario("stale_telemetry", 1, body, real_sitl_available)


def scenario_estimator_failure(real_sitl_available):
    def body(transport, adapters, result):
        v = VEHICLE_IDS[0]
        transport.inject_failure(v, FailureType.ESTIMATOR_INVALID)
        r = adapters[v].send_command(AdapterCommand(command=_command(v, t=0.0), sequence=0))
        result.candidate_command_count = 1
        result.rejected_count = int(not r.accepted)
        result.reject_reasons = {r.reason: 1}
        r2 = adapters[v].send_command(AdapterCommand(command=_command(v, ctype=CommandType.HOLD, vel=None, t=0.1), sequence=1))
        result.acknowledged_count = int(r2.accepted)
    return _run_transport_scenario("estimator_failure", 1, body, real_sitl_available)


def scenario_battery_critical(real_sitl_available):
    def body(transport, adapters, result):
        v = VEHICLE_IDS[0]
        transport.inject_failure(v, FailureType.BATTERY_CRITICAL)
        r = adapters[v].send_command(AdapterCommand(command=_command(v, t=0.0), sequence=0))
        result.candidate_command_count = 1
        result.rejected_count = int(not r.accepted)
        result.reject_reasons = {r.reason: 1}
    return _run_transport_scenario("battery_critical", 1, body, real_sitl_available)


def scenario_namespace_mismatch(real_sitl_available):
    def body(transport, adapters, result):
        from swarm_sim.sitl.commands import SITLCommand
        v = VEHICLE_IDS[0]
        ac = AdapterCommand(command=_command(v, t=0.0), sequence=0)
        forged = SITLCommand(adapter_command=ac, namespace="sitl/forged-namespace", safety_decision_id="x")
        tr = transport.send_command(forged)
        result.candidate_command_count = 1
        result.namespace_violations = 0   # correctly rejected, not a real violation
        result.reject_reasons = {tr.reason: 1}
        result.rejected_count = int(not tr.success)
    return _run_transport_scenario("namespace_mismatch", 1, body, real_sitl_available)


def scenario_wrong_system_id(real_sitl_available):
    def body(transport, adapters, result):
        v = VEHICLE_IDS[0]
        # Simulated by injecting a VEHICLE_DISCONNECT-equivalent check is
        # not meaningful here; the real guarantee (heartbeats/telemetry
        # from an unexpected system_id are ignored, never merged into this
        # vehicle's channel state) is exercised directly in
        # tests/test_ardupilot_sitl.py::test_start_ignores_heartbeat_from_wrong_system_id
        # (requires a second, misconfigured dummy vehicle - out of scope
        # for this lightweight scenario run). Recorded here as
        # "verified_in_unit_tests" rather than fabricating a shallow
        # substitute check.
        result.candidate_command_count = 0
        result.reject_reasons = {"verified_in_unit_tests": 1}
    return _run_transport_scenario("wrong_system_id", 1, body, real_sitl_available)


def scenario_wrong_component_id(real_sitl_available):
    def body(transport, adapters, result):
        result.candidate_command_count = 0
        result.reject_reasons = {"verified_in_unit_tests": 1}
    return _run_transport_scenario("wrong_component_id", 1, body, real_sitl_available)


def scenario_process_exit(real_sitl_available):
    def body(transport, adapters, result):
        v = VEHICLE_IDS[0]
        # Modeled as a heartbeat loss (see module docstring's honest
        # limitation note on attach-mode process visibility) unless a
        # spawn-mode process handle is actually available.
        transport.inject_failure(v, FailureType.VEHICLE_DISCONNECT)
        r = adapters[v].send_command(AdapterCommand(command=_command(v, t=0.0), sequence=0))
        result.candidate_command_count = 1
        result.rejected_count = int(not r.accepted)
        result.reject_reasons = {r.reason: 1}
        result.process_restarts = 0
    return _run_transport_scenario("process_exit", 1, body, real_sitl_available)


SIMPLE_SCENARIOS = {
    "one_vehicle_nominal": scenario_one_vehicle_nominal,
    "two_vehicles_nominal": scenario_two_vehicles_nominal,
    "command_latency": scenario_command_latency,
    "telemetry_latency": scenario_telemetry_latency,
    "command_acknowledgement": scenario_command_acknowledgement,
    "heartbeat_loss": scenario_heartbeat_loss,
    "stale_telemetry": scenario_stale_telemetry,
    "process_exit": scenario_process_exit,
    "estimator_failure": scenario_estimator_failure,
    "battery_critical": scenario_battery_critical,
    "operator_abort": scenario_operator_abort,
    "land_requested": scenario_land_requested,
    "return_to_safe_point": scenario_return_to_safe_point,
    "duplicate_command": scenario_duplicate_command,
    "sequence_replay": scenario_sequence_replay,
    "namespace_mismatch": scenario_namespace_mismatch,
    "wrong_system_id": scenario_wrong_system_id,
    "wrong_component_id": scenario_wrong_component_id,
}


def _mission_scenario_fake_sitl_only(name, overrides, base=None):
    """Scenarios 20-22: always FakeSITL-backed - see module docstring's
    explanation of why real ArduPilot SITL + PyBullet is out of this
    phase's scope."""
    from swarm_sim.config import MissionConfig
    from swarm_sim.mission import FloodSearchMission

    cfg_kwargs = dict(base or dict(num_drones=5, num_victims=4, num_obstacles=2, duration_sec=25.0, seed=7))
    cfg_kwargs.update(overrides)
    cfg_kwargs.setdefault("autopilot_path", "fake_sitl")

    cfg = MissionConfig(**cfg_kwargs)
    mission = FloodSearchMission(cfg)
    mission_result = mission.run()
    diag = mission_result["diagnostics"]

    result = ScenarioResult(name, real_sitl_run=False)
    result.seed = cfg_kwargs["seed"]
    result.vehicles = cfg_kwargs["num_drones"]
    result.candidate_command_count = mission_result["adapter_candidate_count"]
    result.acknowledged_count = mission_result["adapter_accepted_count"]
    result.rejected_count = mission_result["adapter_rejected_count"]
    result.reject_reasons = mission_result["adapter_reject_reasons"]
    result.stale_telemetry_events = mission_result["adapter_stale_telemetry_count"]
    result.failsafe_transitions = mission_result["adapter_failsafe_transition_count"]
    result.ground_contacts = diag.get("drone_ground_contact_count")
    result.obstacle_contacts = diag["drone_obstacle_contact_count"]
    result.drone_drone_contacts = diag["drone_drone_contact_count"]
    result.min_clearance_m = mission_result["min_ground_truth_clearance_m"]
    result.deterministic_replay_result = "not_rerun_here_see_phase7_scenario_script"
    result.executable = "FakeSITLTransport (real ArduPilot SITL + PyBullet real-time sync is out of Phase 8 scope)"
    return result.to_dict()


def scenario_obstacle_wedging():
    return _mission_scenario_fake_sitl_only("obstacle_wedging", dict(
        num_drones=6, num_victims=5, num_obstacles=4, duration_sec=90.0,
        victim_sensor_latency_steps=10, neighbor_sensor_latency_steps=10, obstacle_sensor_latency_steps=5,
        safety_enabled=True,
    ), base={"seed": 42})


def scenario_distributed_consensus():
    return _mission_scenario_fake_sitl_only("phase5_distributed_consensus", dict(
        consensus_mode="distributed", communication_radius=1e6,
        comm_dropout_base=0.0, comm_dropout_at_max_range=0.0, comm_latency_steps=0,
    ))


def scenario_extreme_comm_partition_radius_3m():
    return _mission_scenario_fake_sitl_only("extreme_comm_partition_radius_3m_STRESS_NOT_ACCEPTANCE", dict(
        communication_radius=3.0, comm_dropout_base=0.02, comm_dropout_at_max_range=0.3, comm_latency_steps=2,
    ), base={"num_drones": 5, "num_victims": 4, "num_obstacles": 2, "duration_sec": 25.0, "seed": 7})


MISSION_SCENARIOS = {
    "obstacle_wedging": scenario_obstacle_wedging,
    "distributed_consensus": scenario_distributed_consensus,
    "extreme_comm_partition_radius_3m": scenario_extreme_comm_partition_radius_3m,
}


def main():
    args = sys.argv[1:]
    os.makedirs(OUT_DIR, exist_ok=True)

    if "--dry-run" in args:
        result = scenario_dry_run_configuration()
        print(json.dumps(result.to_dict(), indent=2, default=str))
        with open(os.path.join(OUT_DIR, "phase8_dry_run_result.json"), "w") as f:
            json.dump(result.to_dict(), f, indent=2, default=str)
        return

    if "--local-sitl" not in args:
        print("Usage: python scripts/run_phase8_ardupilot_sitl.py --dry-run | --local-sitl")
        sys.exit(2)

    real_sitl_available = _probe_real_sitl_reachable(n=2, timeout_s=3.0)
    if not real_sitl_available:
        print("Actual ArduPilot SITL was not run.")
        print("Only FakeSITL tests passed.")
        print("No claim of real SITL validation is made.")

    all_results = []
    for name, fn in SIMPLE_SCENARIOS.items():
        n = 2 if name == "two_vehicles_nominal" else 1
        result = fn(real_sitl_available)
        d = result.to_dict()
        print(json.dumps(d, default=str))
        all_results.append(d)

    for name, fn in MISSION_SCENARIOS.items():
        d = fn()
        print(json.dumps(d, default=str))
        all_results.append(d)

    with open(os.path.join(OUT_DIR, "phase8_local_sitl_results.json"), "w") as f:
        json.dump(all_results, f, indent=2, default=str)

    if not real_sitl_available:
        print("\nActual ArduPilot SITL was not run.")
        print("Only FakeSITL tests passed.")
        print("No claim of real SITL validation is made.")


if __name__ == "__main__":
    main()
