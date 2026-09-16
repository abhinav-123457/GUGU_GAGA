"""Phase 4 required-scenario runner - see docs/PHASE4_SAFETY.md.

14 of the 15 required scenarios are a lightweight, deterministic,
PyBullet-free harness that calls SafetySupervisor.evaluate() directly,
injecting exactly the condition each scenario names (an expired command,
a low battery_fraction, estimator_valid=False, operator_abort=True, a
geofence-violating position, a sustained own_agent_isolated, etc.) - this
is the fast, precise way to prove each required behavior in isolation,
the same way Phase 3's synthetic flocking-benchmark harness worked. The
one exception is "obstacle_wedging" (scenario 7), which reruns the real
PyBullet FloodSearchMission with Phase 2 scenario 6's exact config - that
is literally where the wedging failure was originally observed, and a
synthetic harness has no real obstacle geometry/contact physics to
reproduce it with.

Reported per scenario, where applicable to that scenario's nature (see
each scenario function's own notes for anything approximated rather than
exact - same disclosed-approximation style as
docs/PHASE2_DIAGNOSTICS.md): candidate/accepted/rejected counts, safety
overrides, active-state histogram, worst-case AND percentile (not just
average) clearance metrics, contact count, stuck-detection latency,
recovery outcome, false-trigger rate, CPU time per evaluation.

Usage:
    python scripts/run_phase4_safety_scenarios.py                  # all 15
    python scripts/run_phase4_safety_scenarios.py obstacle_wedging  # just one
"""
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from swarm_sim.contracts import (
    Frame, GeofenceSpec, HealthState, NeighborObservation, SensorObservation, VehicleState,
)
from swarm_sim.safety_supervisor import (
    CandidateCommand, MissionContext, SafetySupervisor, SafetySupervisorConfig,
)

OUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "results", "phase4_scenarios")

GEOFENCE = GeofenceSpec(frame=Frame.LOCAL_ENU, center_m=(0.0, 0.0), half_extents_m=(20.0, 20.0),
                          floor_alt_m=0.5, ceiling_alt_m=8.0)
DT = 1.0 / 24.0


def _own(pos=(0.0, 0.0, 3.0), vel=(1.0, 0.0, 0.0), batt=0.9, health=HealthState.OK,
          est=True, last_cmd=0.0, t=0.0, vid="drone0"):
    return VehicleState(
        vehicle_id=vid, sim_time_s=t, frame=Frame.LOCAL_ENU, position_m=pos, velocity_mps=vel,
        acceleration_mps2=(0.0, 0.0, 0.0), attitude_rad=(0.0, 0.0, 0.0), angular_velocity_radps=(0.0, 0.0, 0.0),
        battery_fraction=batt, health_state=health, estimator_valid=est, last_valid_command_time_s=last_cmd,
    )


def _sensor(dropout=False, ranges=None, n_bins=16, vid="drone0"):
    r = ranges if ranges is not None else (math.inf,) * n_bins
    return SensorObservation(
        vehicle_id=vid, sensor_timestamp_s=0.0, sensor_latency_s=0.0, fov_deg=270.0,
        range_returns_m=r, occluded=(False,) * len(r), dropout=dropout, pose_uncertainty_m=0.0, detections=(),
    )


def _neighbor(pos, stale=False, sender="drone1", vid="drone0"):
    return NeighborObservation(
        receiver_id=vid, sender_id=sender, frame=Frame.LOCAL_ENU, measured_position_m=pos,
        measured_velocity_mps=(0.0, 0.0, 0.0), sample_timestamp_s=0.0, delivery_timestamp_s=0.0,
        packet_age_s=0.0, communication_confidence=0.9, stale=stale,
    )


def _cand(vel=(1.0, 0.0, 0.0), t=0.0, ttl=1.0, vid="drone0"):
    return CandidateCommand(vehicle_id=vid, desired_velocity_mps=vel, frame=Frame.LOCAL_ENU,
                              timestamp_s=t, expiration_time_s=t + ttl)


def _ctx(**kwargs):
    kwargs.setdefault("geofence", GEOFENCE)
    return MissionContext(**kwargs)


class ScenarioResult:
    def __init__(self, name):
        self.name = name
        self.candidate_commands = 0
        self.accepted_commands = 0
        self.rejected_commands = 0
        self.safety_overrides = 0
        self.state_counts = {}
        self.estimated_clearances = []
        self.ground_truth_clearances = []
        self.vehicle_body_clearances = []
        self.obstacle_clearances = []
        self.contact_count = 0
        self.cpu_times = []
        self.stuck_detected_at = None
        self.recovered = None
        self.false_triggers = 0
        self.true_triggers = 0

    def record(self, decision, ground_truth_clearance=None, cpu_time=None):
        self.candidate_commands += 1
        if decision.accepted:
            self.accepted_commands += 1
            if decision.active_constraints and decision.active_constraints != ("sensor_dropout",):
                self.safety_overrides += 1
        else:
            self.rejected_commands += 1
        state = decision.emergency_state.value
        self.state_counts[state] = self.state_counts.get(state, 0) + 1
        if decision.min_predicted_clearance_m is not None:
            self.estimated_clearances.append(decision.min_predicted_clearance_m)
        if decision.vehicle_body_clearance_m is not None:
            self.vehicle_body_clearances.append(decision.vehicle_body_clearance_m)
        if decision.obstacle_surface_clearance_m is not None:
            self.obstacle_clearances.append(decision.obstacle_surface_clearance_m)
        if ground_truth_clearance is not None:
            self.ground_truth_clearances.append(ground_truth_clearance)
            triggered = "separation_risk" in decision.active_constraints or "obstacle_clearance" in decision.active_constraints \
                or "obstacle_stuck" in decision.active_constraints
            if triggered and ground_truth_clearance > 1.0:
                self.false_triggers += 1
            elif triggered:
                self.true_triggers += 1
        if cpu_time is not None:
            self.cpu_times.append(cpu_time)

    def _pct(self, values, p):
        if not values:
            return None
        return float(np.percentile(values, p))

    def to_dict(self):
        return {
            "scenario": self.name,
            "candidate_commands": self.candidate_commands,
            "accepted_commands": self.accepted_commands,
            "rejected_commands": self.rejected_commands,
            "safety_overrides": self.safety_overrides,
            "active_state_counts": self.state_counts,
            "min_estimated_clearance_m": min(self.estimated_clearances) if self.estimated_clearances else None,
            "p50_estimated_clearance_m": self._pct(self.estimated_clearances, 50),
            "min_ground_truth_clearance_m_eval_only": min(self.ground_truth_clearances) if self.ground_truth_clearances else None,
            "worst_vehicle_body_clearance_m": min(self.vehicle_body_clearances) if self.vehicle_body_clearances else None,
            "worst_obstacle_clearance_m": min(self.obstacle_clearances) if self.obstacle_clearances else None,
            "contact_count": self.contact_count,
            "stuck_detected_at_s": self.stuck_detected_at,
            "recovery_success": self.recovered,
            "false_safety_trigger_rate": (
                self.false_triggers / max(1, self.false_triggers + self.true_triggers)
                if (self.false_triggers + self.true_triggers) else None
            ),
            "cpu_time_per_evaluation_s_mean": float(np.mean(self.cpu_times)) if self.cpu_times else None,
            "cpu_time_per_evaluation_s_p99": self._pct(self.cpu_times, 99),
        }


def _run_supervisor_steps(name, sup, steps, cfg_overrides=None):
    """steps: list of dicts with keys candidate, own, sensor, neighbors,
    ctx, ground_truth_clearance (optional, evaluation-only, never passed
    to evaluate())."""
    result = ScenarioResult(name)
    for step in steps:
        t0 = time.perf_counter()
        decision = sup.evaluate(step["candidate"], step["own"], step["sensor"],
                                  step.get("neighbors", ()), step["ctx"], now_s=step["own"].sim_time_s)
        cpu = time.perf_counter() - t0
        result.record(decision, ground_truth_clearance=step.get("ground_truth_clearance"), cpu_time=cpu)
    for e in sup.event_log:
        if e.event_type == "stuck_detected" and result.stuck_detected_at is None:
            result.stuck_detected_at = e.sim_time_s
        if e.event_type == "recovery":
            result.recovered = "recovered" in e.reason
    return result


# --- scenario definitions --------------------------------------------------

def scenario_nominal():
    sup = SafetySupervisor(SafetySupervisorConfig())
    steps = []
    for k in range(20):
        t = k * DT
        steps.append(dict(candidate=_cand(t=t), own=_own(pos=(k * 0.1, 0.0, 3.0), t=t, last_cmd=t),
                            sensor=_sensor(), ctx=_ctx(), ground_truth_clearance=100.0))
    return _run_supervisor_steps("nominal_operation", sup, steps)


def scenario_expired_command():
    sup = SafetySupervisor(SafetySupervisorConfig())
    steps = [dict(candidate=_cand(t=0.0, ttl=0.0001), own=_own(t=0.5, last_cmd=0.5), sensor=_sensor(), ctx=_ctx())]
    return _run_supervisor_steps("expired_command", sup, steps)


def scenario_sensor_dropout():
    sup = SafetySupervisor(SafetySupervisorConfig())
    steps = [dict(candidate=_cand(t=0.0), own=_own(t=0.0, last_cmd=0.0), sensor=_sensor(dropout=True), ctx=_ctx())]
    return _run_supervisor_steps("sensor_dropout", sup, steps)


def scenario_stale_sensor():
    """Stale NEIGHBOR observation specifically (see MissionContext's
    NeighborObservation.stale flag) - approximated here via a single
    close-but-stale reading rather than a full stream, since the
    conservative-handling behavior itself doesn't depend on tick count."""
    sup = SafetySupervisor(SafetySupervisorConfig())
    steps = [dict(candidate=_cand(vel=(0, 0, 0), t=0.0), own=_own(vel=(0, 0, 0), t=0.0, last_cmd=0.0),
                   sensor=_sensor(), neighbors=(_neighbor((0.5, 0.0, 3.0), stale=True),), ctx=_ctx(),
                   ground_truth_clearance=0.5)]
    return _run_supervisor_steps("stale_sensor", sup, steps)


def scenario_delayed_observation():
    """Approximates Phase 2's obstacle_sensor_latency_steps via a fixed
    reporting lag baked into the injected range value (the true distance
    closes faster than the reported one) - the full physical version is
    scenario 7 (obstacle_wedging)."""
    sup = SafetySupervisor(SafetySupervisorConfig())
    steps = []
    for k in range(15):
        t = k * DT
        true_dist = max(0.1, 3.0 - k * 0.25)
        reported_dist = max(0.1, true_dist + 0.4)  # sensor reports farther than reality - the latency effect
        ranges = (reported_dist,) + (math.inf,) * 15
        steps.append(dict(candidate=_cand(vel=(2.5, 0, 0), t=t), own=_own(vel=(2.5, 0, 0), t=t, last_cmd=t),
                            sensor=_sensor(ranges=ranges), ctx=_ctx(), ground_truth_clearance=true_dist))
    return _run_supervisor_steps("delayed_observation", sup, steps)


def scenario_obstacle_approach():
    sup = SafetySupervisor(SafetySupervisorConfig())
    steps = []
    for k in range(15):
        t = k * DT
        dist = max(0.05, 3.0 - k * 0.2)
        ranges = (dist,) + (math.inf,) * 15
        steps.append(dict(candidate=_cand(vel=(2.0, 0, 0), t=t), own=_own(vel=(2.0, 0, 0), t=t, last_cmd=t),
                            sensor=_sensor(ranges=ranges), ctx=_ctx(), ground_truth_clearance=dist))
    return _run_supervisor_steps("obstacle_approach", sup, steps)


def scenario_obstacle_wedging():
    """The one scenario that reruns the real PyBullet mission - see
    module docstring. Reuses Phase 2 scenario 6's exact config."""
    from swarm_sim.config import MissionConfig
    from swarm_sim.mission import FloodSearchMission
    cfg = MissionConfig(
        num_drones=6, num_victims=5, num_obstacles=4, duration_sec=90.0, seed=42, gui=False,
        victim_sensor_latency_steps=10, neighbor_sensor_latency_steps=10, obstacle_sensor_latency_steps=5,
        safety_enabled=True,
    )
    mission = FloodSearchMission(cfg)
    mission_result = mission.run()
    diag = mission_result["diagnostics"]
    result = ScenarioResult("obstacle_wedging")
    result.candidate_commands = mission_result["safety_candidate_count"]
    result.accepted_commands = mission_result["safety_accepted_count"]
    result.rejected_commands = mission_result["safety_rejected_count"]
    result.safety_overrides = mission_result["safety_override_count"]
    result.state_counts = mission_result["safety_state_counts"]
    result.obstacle_clearances = [mission_result["min_ground_truth_obstacle_clearance_m"]]
    result.vehicle_body_clearances = [mission_result["min_vehicle_body_clearance_m"]]
    result.ground_truth_clearances = [mission_result["min_ground_truth_clearance_m"]]
    result.contact_count = diag["deduplicated_contact_event_count"]
    result.cpu_times = [mission_result["safety_cpu_time_s"] / max(1, mission_result["safety_candidate_count"])]
    for e in mission_result["safety_event_log"]:
        if e["event_type"] == "stuck_detected" and result.stuck_detected_at is None:
            result.stuck_detected_at = e["sim_time_s"]
    d = result.to_dict()
    d["victims_found"] = f"{mission_result['victims_found']}/{mission_result['total_victims']}"
    d["swarm_connectivity_fraction"] = mission_result["swarm_connectivity_fraction"]
    d["drone_obstacle_contact_count"] = diag["drone_obstacle_contact_count"]
    d["drone_drone_contact_count"] = diag["drone_drone_contact_count"]
    d["drone_ground_contact_count"] = diag.get("drone_ground_contact_count")
    return d, mission_result


def scenario_crossing_drones():
    sup = SafetySupervisor(SafetySupervisorConfig())
    steps = []
    for k in range(15):
        t = k * DT
        own_pos = (-3.0 + k * 0.4, 0.0, 3.0)
        neighbor_pos = (3.0 - k * 0.4, 0.0, 3.0)
        dist = math.hypot(own_pos[0] - neighbor_pos[0], own_pos[1] - neighbor_pos[1])
        steps.append(dict(candidate=_cand(vel=(2.0, 0, 0), t=t), own=_own(pos=own_pos, vel=(2.0, 0, 0), t=t, last_cmd=t),
                            sensor=_sensor(), neighbors=(_neighbor(neighbor_pos),), ctx=_ctx(),
                            ground_truth_clearance=dist))
    return _run_supervisor_steps("crossing_drones", sup, steps)


def scenario_dense_swarm():
    sup = SafetySupervisor(SafetySupervisorConfig())
    rng = np.random.default_rng(0)
    steps = []
    for k in range(15):
        t = k * DT
        neighbors = tuple(
            _neighbor((float(x), float(y), 3.0)) for x, y in rng.uniform(-1.0, 1.0, size=(8, 2))
        )
        dists = [math.hypot(n.measured_position_m[0], n.measured_position_m[1]) for n in neighbors]
        steps.append(dict(candidate=_cand(t=t), own=_own(t=t, last_cmd=t), sensor=_sensor(),
                            neighbors=neighbors, ctx=_ctx(), ground_truth_clearance=min(dists)))
    return _run_supervisor_steps("dense_swarm", sup, steps)


def scenario_communication_partition():
    sup = SafetySupervisor(SafetySupervisorConfig(lost_agent_timeout_s=0.3))
    steps = []
    for k in range(15):
        t = k * DT
        steps.append(dict(candidate=_cand(t=t), own=_own(t=t, last_cmd=t), sensor=_sensor(),
                            ctx=_ctx(own_agent_isolated=True)))
    return _run_supervisor_steps("communication_partition", sup, steps)


def scenario_lost_drone():
    """Distinct from communication_partition: models THIS vehicle
    correctly linked to the ground station (own last_valid_command_time_s
    fresh) but isolated from its swarm peers specifically."""
    sup = SafetySupervisor(SafetySupervisorConfig(lost_agent_timeout_s=0.3, link_timeout_s=100.0))
    steps = []
    for k in range(15):
        t = k * DT
        steps.append(dict(candidate=_cand(t=t), own=_own(t=t, last_cmd=t), sensor=_sensor(),
                            ctx=_ctx(own_agent_isolated=True)))
    return _run_supervisor_steps("lost_drone", sup, steps)


def scenario_low_battery():
    sup = SafetySupervisor(SafetySupervisorConfig())
    steps = []
    for k, batt in enumerate([0.9, 0.5, 0.25, 0.15, 0.1, 0.05]):
        t = k * DT
        steps.append(dict(candidate=_cand(t=t), own=_own(batt=batt, t=t, last_cmd=t), sensor=_sensor(), ctx=_ctx()))
    return _run_supervisor_steps("low_battery", sup, steps)


def scenario_invalid_estimator():
    sup = SafetySupervisor(SafetySupervisorConfig())
    steps = [dict(candidate=_cand(t=0.0), own=_own(est=False, t=0.0, last_cmd=0.0), sensor=_sensor(), ctx=_ctx())]
    return _run_supervisor_steps("invalid_estimator", sup, steps)


def scenario_geofence_approach():
    sup = SafetySupervisor(SafetySupervisorConfig())
    steps = []
    for k in range(15):
        t = k * DT
        x = 15.0 + k * 0.4
        dist_to_boundary = 20.0 - x
        steps.append(dict(candidate=_cand(vel=(1.0, 0, 0), t=t), own=_own(pos=(x, 0.0, 3.0), vel=(1.0, 0, 0), t=t, last_cmd=t),
                            sensor=_sensor(), ctx=_ctx(), ground_truth_clearance=dist_to_boundary))
    return _run_supervisor_steps("geofence_approach", sup, steps)


def scenario_operator_abort():
    sup = SafetySupervisor(SafetySupervisorConfig())
    steps = [dict(candidate=_cand(t=0.0), own=_own(t=0.0, last_cmd=0.0), sensor=_sensor(), ctx=_ctx(operator_abort=True))]
    return _run_supervisor_steps("operator_abort", sup, steps)


SCENARIOS = {
    "nominal": scenario_nominal,
    "expired_command": scenario_expired_command,
    "sensor_dropout": scenario_sensor_dropout,
    "stale_sensor": scenario_stale_sensor,
    "delayed_observation": scenario_delayed_observation,
    "obstacle_approach": scenario_obstacle_approach,
    "obstacle_wedging": scenario_obstacle_wedging,
    "crossing_drones": scenario_crossing_drones,
    "dense_swarm": scenario_dense_swarm,
    "communication_partition": scenario_communication_partition,
    "lost_drone": scenario_lost_drone,
    "low_battery": scenario_low_battery,
    "invalid_estimator": scenario_invalid_estimator,
    "geofence_approach": scenario_geofence_approach,
    "operator_abort": scenario_operator_abort,
}


def main():
    names = [n for n in sys.argv[1:] if not n.startswith("-")] or list(SCENARIOS.keys())
    os.makedirs(OUT_DIR, exist_ok=True)
    all_results = []
    for name in names:
        fn = SCENARIOS[name]
        if name == "obstacle_wedging":
            d, _ = fn()
        else:
            result = fn()
            d = result.to_dict()
        print(json.dumps(d, default=str), flush=True)
        all_results.append(d)
    with open(os.path.join(OUT_DIR, "phase4_scenario_results.json"), "w") as f:
        json.dump(all_results, f, indent=2, default=str)


if __name__ == "__main__":
    main()
