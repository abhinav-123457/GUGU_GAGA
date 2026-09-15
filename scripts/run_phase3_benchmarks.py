"""Phase 3 behavior-model benchmarks - see docs/PHASE3_BEHAVIORS.md.

Runs each of the 4 behavior models (boids, vicsek, couzin, olfati_saber)
through 11 required scenarios and reports, per (model, scenario): average/
worst pairwise clearance, speed/acceleration/turn-rate violation counts,
finite-output failures, fragmentation duration, connectivity metrics, CPU
time per control step, deterministic-replay status, and (where a search
objective actually exists - scenario 11 only) coverage/search progress.

Scenarios 1-10 are a lightweight, deterministic, PyBullet-free kinematic
harness: real swarm_sim.network.CommsNetwork (the same range/dropout/
latency model SwarmController uses in production) feeds each agent's
locally-perceived neighbor view into that model's *_candidate_command
entry point every step; positions are advanced by simple Euler
integration of the returned (already-bounded) velocity. This is
deliberately NOT the full PyBullet mission - these 10 scenarios are about
the behavior model's own steering output under controlled neighbor/
sensing conditions, not end-to-end SAR mission scoring.

Scenario 11 (obstacle-wedging, carried over from Phase 2) is the one
exception: it reruns the actual PyBullet FloodSearchMission with
scenario 6's exact delayed-observation config (see
scripts/regenerate_phase2_diagnostics.py) once per behavior model, because
that is literally where the Phase 2 wedging was observed and a synthetic
harness has no obstacles to wedge against.

Usage:
    python scripts/run_phase3_benchmarks.py               # all 4 models x 11 scenarios
    python scripts/run_phase3_benchmarks.py boids couzin   # just these models
"""
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from swarm_sim.behaviors import boids, couzin, olfati_saber, vicsek
from swarm_sim.config import MissionConfig
from swarm_sim.network import CommsNetwork

OUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "results", "phase3_benchmarks")

MODELS = ["boids", "vicsek", "couzin", "olfati_saber"]
DT_S = 1 / 24
T_STEPS = 240  # 10s of synthetic flight - enough to reach steady-state behavior


def _adjacency_from_network(network, n, max_age_steps):
    cur = network.step
    adj = np.zeros((n, n), dtype=int)
    for i in range(n):
        for j, e in network.last_seen[i].items():
            if cur - e["step"] <= max_age_steps:
                adj[i, j] = 1
                adj[j, i] = 1
    return adj


def _local_view(i, positions, velocities, headings, network, cfg, noise_rng=None, noise_std=0.0):
    """Mirrors SwarmController._perceived_state: index 0 = self, the rest
    are neighbors this agent's CommsNetwork link currently has a live
    sample for. Optionally perturbs the perceived (not the agent's own)
    positions/velocities with Gaussian noise, for the noisy_observations
    scenario - added here rather than inside CommsNetwork so the network
    model itself stays exactly what production code uses."""
    table = network.neighbor_table(i, cfg.comm_max_age_steps)
    linked = [e for e in table.values() if e["dist"] <= cfg.sensing_radius]
    pos = [positions[i].copy()]
    vel = [velocities[i].copy()]
    head = [headings[i].copy()]
    for e in linked:
        p, v = e["pos"].copy(), e["vel"].copy()
        if noise_rng is not None and noise_std > 0.0:
            p = p + noise_rng.normal(scale=noise_std, size=3)
            v = v + noise_rng.normal(scale=noise_std, size=3)
        pos.append(p)
        vel.append(v)
        speed = np.linalg.norm(v)
        head.append(v / speed if speed > 1e-6 else headings[i])
    ids = list(range(1, len(pos)))
    return np.array(pos), np.array(vel), np.array(head), ids


def _candidate(model, i, positions, velocities, headings, neighbor_ids, prev_vel, cfg, rng):
    max_speed, max_accel, max_turn = cfg.max_speed_mps, cfg.max_accel_mps2, cfg.max_turn_rate_radps
    if model == "boids":
        return boids.boids_candidate_command(
            positions, velocities, 0, neighbor_ids, cfg.r_attraction, prev_vel,
            max_accel, max_speed, DT_S,
        )
    if model == "vicsek":
        return vicsek.vicsek_candidate_command(
            headings, 0, neighbor_ids, cfg.vicsek_noise, rng,
            headings[0], min(np.linalg.norm(prev_vel), cfg.cruise_speed_mps) or cfg.cruise_speed_mps,
            max_speed, max_turn, DT_S,
        )
    if model == "couzin":
        return couzin.couzin_candidate_command(
            positions, headings, 0, neighbor_ids, cfg.r_repulsion, cfg.r_orientation, cfg.r_attraction,
            headings[0], cfg.cruise_speed_mps, max_speed, max_turn, DT_S, cfg.fov_deg,
        )
    if model == "olfati_saber":
        return olfati_saber.olfati_saber_candidate_command(
            positions[0], velocities[0], positions[1:], velocities[1:],
            target_pos=positions[0], target_vel=headings[0] * cfg.cruise_speed_mps,
            d_alpha=cfg.os_desired_spacing_m, r_alpha=cfg.os_interaction_range_m,
            max_accel_mps2=max_accel, max_speed_mps=max_speed, dt_s=DT_S,
            c_spacing=cfg.os_c_spacing, c_align=cfg.os_c_align, c_nav=cfg.os_c_nav,
        )
    raise ValueError(model)


SCENARIOS = {
    "1_no_neighbors": dict(n=1, layout="isolated", cfg_overrides={}),
    "2_one_neighbor": dict(n=2, layout="pair", cfg_overrides={}),
    "3_symmetric_neighbors": dict(n=7, layout="ring", cfg_overrides={}),
    "4_crossing_trajectories": dict(n=2, layout="head_on", cfg_overrides={}),
    "5_dense_swarm": dict(n=30, layout="cluster", cfg_overrides={"sensing_radius": 8.0}),
    "6_sparse_swarm": dict(n=6, layout="sparse", cfg_overrides={}),
    "7_delayed_observations": dict(n=6, layout="cluster", cfg_overrides={"comm_latency_steps": 10, "comm_max_age_steps": 14}),
    "8_noisy_observations": dict(n=6, layout="cluster", cfg_overrides={}, noise_std=1.0),
    "9_dropped_observations": dict(n=6, layout="cluster", cfg_overrides={"comm_dropout_base": 0.5, "comm_dropout_at_max_range": 0.95}),
    "10_communication_partition": dict(n=8, layout="two_clusters", cfg_overrides={"communication_radius": 5.0}),
}


def _initial_state(layout, n, cfg, rng):
    positions = np.zeros((n, 3))
    positions[:, 2] = cfg.flight_altitude
    if layout == "isolated":
        pass
    elif layout == "pair":
        positions[1, 0] = 3.0
    elif layout == "ring":
        for k in range(1, n):
            angle = 2 * math.pi * (k - 1) / (n - 1)
            positions[k, 0] = 3.0 * math.cos(angle)
            positions[k, 1] = 3.0 * math.sin(angle)
    elif layout == "head_on":
        positions[0, 0], positions[1, 0] = -8.0, 8.0
    elif layout == "cluster":
        positions[:, :2] = rng.uniform(-4.0, 4.0, size=(n, 2))
    elif layout == "sparse":
        positions[:, :2] = rng.uniform(-25.0, 25.0, size=(n, 2))
    elif layout == "two_clusters":
        half = n // 2
        positions[:half, :2] = rng.uniform(-2.0, 2.0, size=(half, 2))
        positions[half:, :2] = rng.uniform(-2.0, 2.0, size=(n - half, 2)) + np.array([30.0, 30.0])
    else:
        raise ValueError(layout)

    velocities = np.zeros((n, 3))
    if layout == "head_on":
        velocities[0, 0], velocities[1, 0] = 2.0, -2.0
    headings = rng.normal(size=(n, 3))
    headings[:, 2] = 0.0
    norms = np.linalg.norm(headings, axis=1, keepdims=True)
    norms[norms < 1e-9] = 1.0
    headings = headings / norms
    return positions, velocities, headings


def run_synthetic_scenario(model, scenario_name, spec, seed=42):
    n = spec["n"]
    cfg = MissionConfig(**{**{"num_drones": n, "seed": seed}, **spec["cfg_overrides"]})
    noise_std = spec.get("noise_std", 0.0)

    def _one_pass():
        rng = np.random.default_rng(seed)
        network = CommsNetwork(cfg, n, np.random.default_rng(seed + 1))
        positions, velocities, headings = _initial_state(spec["layout"], n, cfg, np.random.default_rng(seed + 2))

        pairwise_clearances = []
        speed_violations = acceleration_violations = turn_violations = finite_failures = 0
        fragmentation_steps = 0
        connectivity_values = []
        cpu_times = []
        path_lengths = np.zeros(n)

        for _ in range(T_STEPS):
            network.tick(positions, velocities)
            new_velocities = velocities.copy()
            new_headings = headings.copy()

            for i in range(n):
                pos_local, vel_local, head_local, nbr_ids = _local_view(
                    i, positions, velocities, headings, network, cfg,
                    noise_rng=rng if noise_std > 0 else None, noise_std=noise_std,
                )
                t0 = time.perf_counter()
                candidate = _candidate(model, i, pos_local, vel_local, head_local, nbr_ids, velocities[i], cfg, rng)
                cpu_times.append(time.perf_counter() - t0)

                if not np.all(np.isfinite(candidate)):
                    finite_failures += 1
                    candidate = np.zeros(3)
                speed = float(np.linalg.norm(candidate))
                if speed > cfg.max_speed_mps + 1e-6:
                    speed_violations += 1
                accel = float(np.linalg.norm(candidate - velocities[i]) / DT_S)
                if accel > cfg.max_accel_mps2 + cfg.emergency_accel_mps2 + 1e-6:
                    acceleration_violations += 1
                if speed > 1e-9 and np.linalg.norm(velocities[i]) > 1e-9:
                    prev_angle = math.atan2(velocities[i][1], velocities[i][0])
                    new_angle = math.atan2(candidate[1], candidate[0])
                    d_angle = abs(math.atan2(math.sin(new_angle - prev_angle), math.cos(new_angle - prev_angle)))
                    if d_angle / DT_S > cfg.max_turn_rate_radps + 1e-3:
                        turn_violations += 1

                new_velocities[i] = candidate
                if speed > 1e-9:
                    new_headings[i] = candidate / speed

            velocities, headings = new_velocities, new_headings
            positions = positions + velocities * DT_S
            path_lengths += np.linalg.norm(velocities * DT_S, axis=1)

            if n > 1:
                dmat = np.linalg.norm(positions[:, None, :] - positions[None, :, :], axis=-1)
                np.fill_diagonal(dmat, np.inf)
                pairwise_clearances.append(float(dmat.min()))

            adj = _adjacency_from_network(network, n, cfg.comm_max_age_steps)
            if n > 1:
                if olfati_saber.is_fragmented(adj) or n == 1:
                    fragmentation_steps += 1
                connectivity_values.append(olfati_saber.algebraic_connectivity(adj))

        return dict(
            positions=positions, velocities=velocities,
            avg_pairwise_clearance_m=float(np.mean(pairwise_clearances)) if pairwise_clearances else None,
            worst_pairwise_clearance_m=float(np.min(pairwise_clearances)) if pairwise_clearances else None,
            speed_violations=speed_violations, acceleration_violations=acceleration_violations,
            turn_rate_violations=turn_violations, finite_output_failures=finite_failures,
            fragmentation_duration_s=fragmentation_steps * DT_S,
            connectivity_min=float(np.min(connectivity_values)) if connectivity_values else None,
            connectivity_mean=float(np.mean(connectivity_values)) if connectivity_values else None,
            cpu_time_per_control_step_s=float(np.mean(cpu_times)) if cpu_times else None,
            movement_utilization_fraction=float(np.mean(path_lengths) / (cfg.max_speed_mps * T_STEPS * DT_S)),
        )

    result_a = _one_pass()
    result_b = _one_pass()
    deterministic = bool(np.array_equal(result_a["positions"], result_b["positions"]) and
                          np.array_equal(result_a["velocities"], result_b["velocities"]))
    result_a.pop("positions")
    result_a.pop("velocities")
    result_a["deterministic_replay"] = deterministic
    result_a["coverage_or_search_progress"] = (
        "not applicable - no search objective in this synthetic flocking scenario; "
        "see scenario 11 for victims_found-based coverage"
    )
    result_a["scenario"] = scenario_name
    result_a["model"] = model
    result_a["num_agents"] = n
    return result_a


def run_obstacle_wedging_scenario(model):
    from swarm_sim.mission import FloodSearchMission
    cfg = MissionConfig(
        num_drones=6, num_victims=5, num_obstacles=4, duration_sec=90.0, seed=42, gui=False,
        flock_model=model,
        victim_sensor_latency_steps=10, neighbor_sensor_latency_steps=10, obstacle_sensor_latency_steps=5,
    )
    mission = FloodSearchMission(cfg)
    result = mission.run()
    diag = result["diagnostics"]
    return dict(
        scenario="11_obstacle_wedging_phase2", model=model, num_agents=cfg.num_drones,
        coverage_or_search_progress=f"{result['victims_found']}/{result['total_victims']} victims found",
        avg_pairwise_clearance_m=None,  # not tracked per-tick as an average in mission.py - see limitations
        worst_pairwise_clearance_m=result["min_ground_truth_clearance_m"],
        worst_vehicle_body_clearance_m=result["min_vehicle_body_clearance_m"],
        worst_obstacle_clearance_m=result["min_ground_truth_obstacle_clearance_m"],
        speed_violations=None,  # not independently instrumented at the mission level - see limitations
        acceleration_violations=None,
        turn_rate_violations=None,
        finite_output_failures=None,
        fragmentation_duration_s=None,  # CommsNetwork fragmentation, not this scenario's focus - see swarm_connectivity_fraction
        swarm_connectivity_fraction=result["swarm_connectivity_fraction"],
        raw_contact_point_count=diag["raw_contact_point_count"],
        deduplicated_contact_event_count=diag["deduplicated_contact_event_count"],
        contact_step_count=diag["contact_step_count"],
        drone_obstacle_contact_count=diag["drone_obstacle_contact_count"],
        drone_drone_contact_count=diag["drone_drone_contact_count"],
        cpu_time_per_control_step_s=(diag["sensing_cpu_time_s"] + diag["consensus_cpu_time_s"]) / max(
            1, int(result["time_elapsed"] * cfg.control_freq_hz)),
        deterministic_replay="not rechecked in this script - see docs/PHASE2_DIAGNOSTICS.md/PHASE1_CONTRACT.md for the seeded-determinism guarantee this mission already relies on",
    )


def main():
    models = [m for m in sys.argv[1:] if not m.startswith("-")] or MODELS
    os.makedirs(OUT_DIR, exist_ok=True)
    all_results = []
    for model in models:
        for scenario_name, spec in SCENARIOS.items():
            res = run_synthetic_scenario(model, scenario_name, spec)
            all_results.append(res)
            print(json.dumps(res, default=str), flush=True)
        wedge = run_obstacle_wedging_scenario(model)
        all_results.append(wedge)
        print(json.dumps(wedge, default=str), flush=True)

    out_path = os.path.join(OUT_DIR, "phase3_benchmark_results.json")
    with open(out_path, "w") as f:
        json.dump(all_results, f, indent=2, default=str)
    print(f"written: {out_path}", file=sys.stderr)


if __name__ == "__main__":
    main()
