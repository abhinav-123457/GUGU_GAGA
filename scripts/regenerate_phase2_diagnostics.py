"""Deterministic regeneration of the Phase 2 diagnostic JSON reports.

Reruns the 8 scenarios documented in docs/PHASE2_DIAGNOSTICS.md with fixed
seeds and writes one JSON file per scenario to results/phase2_diagnostics/
(gitignored - regenerate rather than commit the output; this script is the
committed, deterministic source of truth for how those numbers are
produced). Evaluation/reporting only - running this script does not change
any sensor, controller, consensus, or safety behavior.

Usage:
    python scripts/regenerate_phase2_diagnostics.py                     # all 8
    python scripts/regenerate_phase2_diagnostics.py 6_delayed_observations  # just one
"""
import json
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from swarm_sim.config import MissionConfig
from swarm_sim.mission import FloodSearchMission

OUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "results", "phase2_diagnostics")

BASE = dict(num_drones=6, num_victims=5, num_obstacles=4, duration_sec=90.0, seed=42, gui=False)

SCENARIOS = {
    "1_perfect_sensing_baseline": dict(
        victim_sensor_noise_std_m=0.0, victim_sensor_false_negative_prob=0.0,
        victim_sensor_false_positive_rate=0.0, victim_sensor_dropout_prob=0.0,
        victim_sensor_latency_steps=0, obstacle_sensor_noise_std_m=0.0,
        obstacle_sensor_dropout_prob=0.0, neighbor_sensor_noise_std_m=0.0,
        neighbor_sensor_velocity_noise_std_mps=0.0, neighbor_sensor_dropout_prob=0.0,
    ),
    "2_noisy_sensing_defaults": dict(),
    "3_heavy_dropout": dict(
        victim_sensor_dropout_prob=0.5, obstacle_sensor_dropout_prob=0.4,
        neighbor_sensor_dropout_prob=0.4,
    ),
    "4_severe_occlusion": dict(num_obstacles=10, obstacle_radius=2.0),
    "5_false_positive_heavy": dict(
        victim_sensor_false_positive_rate=0.3, victim_sensor_false_negative_prob=0.0,
    ),
    "6_delayed_observations": dict(
        victim_sensor_latency_steps=10, neighbor_sensor_latency_steps=10,
        obstacle_sensor_latency_steps=5,
    ),
    "7_degraded_neighbor_sensing": dict(
        neighbor_sensor_noise_std_m=1.5, neighbor_sensor_dropout_prob=0.5,
        neighbor_sensor_range_m=3.0, neighbor_sensor_fov_deg=90.0,
    ),
    "8_comms_plus_sensor_degradation": dict(
        comm_dropout_at_max_range=0.9, communication_radius=8.0, comm_latency_steps=4,
        victim_sensor_dropout_prob=0.4, neighbor_sensor_dropout_prob=0.4,
        neighbor_sensor_noise_std_m=1.0,
    ),
}


def _git_commit_hash():
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        ).decode().strip()
    except Exception:
        return None


def run_one(name, overrides):
    merged = dict(BASE)
    merged.update(overrides)
    cfg = MissionConfig(**merged)
    mission = FloodSearchMission(cfg)
    result = mission.run()
    diag = result["diagnostics"]

    summary = {
        "scenario": name,
        "config": merged,
        "git_commit": _git_commit_hash(),
        "victims_found": f"{result['victims_found']}/{result['total_victims']}",
        "time_elapsed_s": result["time_elapsed"],
        # Contact accounting - disambiguated, see docs/PHASE2_DIAGNOSTICS.md.
        "raw_contact_point_count": diag["raw_contact_point_count"],
        "deduplicated_contact_event_count": diag["deduplicated_contact_event_count"],
        "contact_step_count": diag["contact_step_count"],
        "drone_drone_contact_count": diag["drone_drone_contact_count"],
        "drone_obstacle_contact_count": diag["drone_obstacle_contact_count"],
        "drone_ground_contact_count": diag["drone_ground_contact_count"],
        "true_positive_events": diag["true_positive_events"],
        "false_positive_events": diag["false_positive_events"],
        "confirmed_victims": diag["confirmed_victims"],
        "missed_victims": diag["missed_victims"],
    }

    os.makedirs(OUT_DIR, exist_ok=True)
    out_path = os.path.join(OUT_DIR, f"{name}.json")
    with open(out_path, "w") as f:
        json.dump({"summary": summary, "full_diagnostics": diag}, f, indent=2, default=str)
    return summary, out_path


def main():
    names = sys.argv[1:] or list(SCENARIOS.keys())
    for name in names:
        summary, out_path = run_one(name, SCENARIOS[name])
        print(json.dumps(summary, default=str))
        print(f"written: {out_path}", file=sys.stderr)


if __name__ == "__main__":
    main()
