"""Phase 16B scenario sweep: the PyBullet swarm simulation under GNSS-denied
localisation - see docs/PHASE16B_ESTIMATION.md.

For every (estimator profile x geofence sigma-multiplier k x seed) it runs a
real `FloodSearchMission` in `estimated` mode (plus a `truth_state`
baseline) and records, per run: how wrong the drones' own estimates were
(error, drift as a fraction of path length, NEES consistency, how often the
estimated 1x1 m grid cell is the true one), how often they REALLY left the
geofence, and the mission outcome (victims found, false confirmations,
safety-state counts).

Read the results with the following in mind (all limits of the simulation,
not of the estimator):
  * The `ideal` profile - not `truth_state` - is the reference for drift:
    in estimated mode every consensus confirmation becomes a beacon (a drone
    cannot know it is false), which changes swarm behaviour on its own.
  * The swarm simulation amplifies float rounding (see the doc), so a single
    seed says little; compare means over seeds.
  * All drift numbers come from ILLUSTRATIVE, not hardware-verified, error
    models (estimation/profiles.py).

Usage:
    python scripts/run_phase16b_scenarios.py                 # full sweep
    python scripts/run_phase16b_scenarios.py --quick         # small smoke sweep
    python scripts/run_phase16b_scenarios.py --profiles vio,stress --k-values 0,3 --num-seeds 8
Results go to results/phase16b_scenarios/<run-id>/ (gitignored): runs.csv,
summary.json, manifests.json.
"""
import argparse
import csv
import dataclasses
import json
import os
import sys
import time
from typing import Dict, List, Optional, Sequence

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from swarm_sim.config import MissionConfig
from swarm_sim.estimation import PROFILES
from swarm_sim.manifest import build_run_manifest
from swarm_sim.mission import FloodSearchMission

RESULTS_ROOT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                            "results", "phase16b_scenarios")

RUN_COLUMNS = [
    "scenario", "localization_mode", "profile", "k", "seed", "duration_s", "num_drones",
    "victims_found", "total_victims", "false_confirmations", "synthetic_beacons",
    "loc_mean_error_m", "loc_max_error_m", "loc_mean_final_drift_fraction", "loc_mean_nees",
    "loc_nees_within_95", "loc_max_sigma_m", "loc_cell_match_fraction", "loc_invalid_tick_fraction",
    "true_geofence_outside_fraction", "true_geofence_max_excursion_m",
    "safety_geofence_risk_ticks", "safety_safe_hold_ticks", "safety_separation_risk_ticks", "contact_steps",
    "alt_min_m", "alt_max_m", "wall_seconds",
]

# (summary key, run column, aggregate[, "clean"]) - "clean" restricts the aggregate to runs in which no drone
# touched anything (contact_steps == 0). A drone that has hit the ground tumbles, and planar odometry is
# meaningless from then on (a real VIO would report tracking loss), so consistency statistics are only
# meaningful on clean runs.
SUMMARY_FIELDS = [
    ("victims_found", "victims_found", "mean"),
    ("false_confirmations", "false_confirmations", "mean"),
    ("synthetic_beacons", "synthetic_beacons", "mean"),
    ("loc_mean_error_m", "loc_mean_error_m", "mean"),
    ("loc_worst_error_m", "loc_max_error_m", "max"),
    ("loc_mean_final_drift_fraction", "loc_mean_final_drift_fraction", "mean"),
    ("loc_mean_nees", "loc_mean_nees", "mean"),
    ("loc_mean_nees_clean_runs", "loc_mean_nees", "mean", "clean"),
    ("loc_mean_error_clean_runs_m", "loc_mean_error_m", "mean", "clean"),
    ("loc_nees_within_95", "loc_nees_within_95", "mean"),
    ("loc_max_sigma_m", "loc_max_sigma_m", "max"),
    ("loc_cell_match_fraction", "loc_cell_match_fraction", "mean"),
    ("loc_invalid_tick_fraction", "loc_invalid_tick_fraction", "mean"),
    ("true_geofence_outside_fraction", "true_geofence_outside_fraction", "mean"),
    ("true_geofence_worst_excursion_m", "true_geofence_max_excursion_m", "max"),
    ("safety_geofence_risk_ticks", "safety_geofence_risk_ticks", "mean"),
    ("safety_safe_hold_ticks", "safety_safe_hold_ticks", "mean"),
    ("safety_separation_risk_ticks", "safety_separation_risk_ticks", "mean"),
    ("contact_steps", "contact_steps", "mean"),
    ("true_altitude_min_m", "alt_min_m", "min"),
    ("true_altitude_max_m", "alt_max_m", "max"),
]


@dataclasses.dataclass(frozen=True)
class Scenario:
    name: str
    localization_mode: str
    profile: Optional[str]
    k: float


def build_scenarios(profiles: Sequence[str], k_values: Sequence[float], include_baseline: bool = True) -> List[Scenario]:
    scenarios = [Scenario("truth_state", "truth_state", None, 0.0)] if include_baseline else []
    for profile in profiles:
        if profile not in PROFILES:
            raise ValueError(f"unknown profile {profile!r}; known: {sorted(PROFILES)}")
        for k in k_values:
            scenarios.append(Scenario(f"{profile}/k={k:g}", "estimated", profile, float(k)))
    return scenarios


def build_config(scenario: Scenario, seed: int, duration_s: float, num_drones: int) -> MissionConfig:
    kwargs = dict(num_drones=num_drones, duration_sec=duration_s, seed=seed, num_victims=3, num_obstacles=2,
                  localization_mode=scenario.localization_mode, safety_pose_sigma_geofence_k=scenario.k)
    if scenario.profile is not None:
        kwargs["estimator_profile"] = scenario.profile
    return MissionConfig(**kwargs)


def run_one(scenario: Scenario, seed: int, duration_s: float, num_drones: int):
    """Returns (row dict for runs.csv, manifest dict)."""
    cfg = build_config(scenario, seed, duration_s, num_drones)
    mission = FloodSearchMission(cfg)
    t0 = time.time()
    res = mission.run()
    wall = time.time() - t0
    loc = res["localization"] or {}
    geo = res["true_geofence"]
    counts = res["safety_state_counts"]
    # True altitude range over all drones after the first 2 s (the spawn transient). Vertical wander is a
    # first-order artefact of the estimated vertical speed (finding 7), so every run reports it.
    z = [r["z"] for r in res["telemetry"].log if r["t"] > 2.0]
    row = {
        "scenario": scenario.name, "localization_mode": scenario.localization_mode,
        "profile": scenario.profile or "", "k": scenario.k, "seed": seed, "duration_s": duration_s,
        "num_drones": num_drones,
        "victims_found": res["victims_found"], "total_victims": res["total_victims"],
        "false_confirmations": res["false_confirmations"], "synthetic_beacons": res["synthetic_beacon_count"],
        "loc_mean_error_m": loc.get("mean_error_m"), "loc_max_error_m": loc.get("max_error_m"),
        "loc_mean_final_drift_fraction": loc.get("mean_final_drift_fraction"),
        "loc_mean_nees": loc.get("mean_nees"), "loc_nees_within_95": loc.get("mean_nees_within_95_fraction"),
        "loc_max_sigma_m": loc.get("max_sigma_m"), "loc_cell_match_fraction": loc.get("mean_cell_match_fraction"),
        "loc_invalid_tick_fraction": loc.get("mean_invalid_tick_fraction"),
        "true_geofence_outside_fraction": geo["outside_fraction"], "true_geofence_max_excursion_m": geo["max_excursion_m"],
        "safety_geofence_risk_ticks": counts.get("GEOFENCE_RISK", 0), "safety_safe_hold_ticks": counts.get("SAFE_HOLD", 0),
        "safety_separation_risk_ticks": counts.get("SEPARATION_RISK", 0), "contact_steps": res["contact_steps"],
        "alt_min_m": min(z) if z else None, "alt_max_m": max(z) if z else None,
        "wall_seconds": round(wall, 2),
    }
    manifest = build_run_manifest(
        config=dataclasses.asdict(cfg), master_seed=seed, seed_manager=mission._sensor_seeds,
        map_id=res["mission_map_id"],
        model_id=f"localization={scenario.localization_mode}/profile={scenario.profile or 'none'}/k={scenario.k:g}",
    ).to_dict()
    return row, manifest


def summarize(rows: Sequence[Dict]) -> Dict[str, Dict]:
    out: Dict[str, Dict] = {}
    for name in dict.fromkeys(r["scenario"] for r in rows):
        group = [r for r in rows if r["scenario"] == name]
        summary: Dict = {"runs": len(group), "seeds": [r["seed"] for r in group],
                         "runs_with_contact": sum(1 for r in group if (r.get("contact_steps") or 0) > 0)}
        for key, column, how, *flags in SUMMARY_FIELDS:
            rows_used = [r for r in group if not flags or (r.get("contact_steps") or 0) == 0]
            vals = [r[column] for r in rows_used if r.get(column) is not None]
            agg = {"mean": np.mean, "max": np.max, "min": np.min}[how]
            summary[key] = float(agg(vals)) if vals else None
        out[name] = summary
    return out


def _fmt(v, width=7, digits=2):
    return f"{'-':>{width}}" if v is None else f"{v:>{width}.{digits}f}"


def print_table(summary: Dict[str, Dict]) -> None:
    header = (f"{'scenario':18s} {'runs':>4} {'crash':>5} {'victims':>7} {'locErr':>7} {'worst':>7} {'drift%':>7} "
              f"{'NEES':>7} {'NEESclean':>9} {'cell%':>7} {'outside%':>8} {'maxExc':>7} {'GFrisk':>7} {'holds':>7} "
              f"{'altMin':>7} {'altMax':>7}")
    print(header)
    print("-" * len(header))
    for name, s in summary.items():
        drift = None if s["loc_mean_final_drift_fraction"] is None else 100 * s["loc_mean_final_drift_fraction"]
        cell = None if s["loc_cell_match_fraction"] is None else 100 * s["loc_cell_match_fraction"]
        outside = None if s["true_geofence_outside_fraction"] is None else 100 * s["true_geofence_outside_fraction"]
        print(f"{name:18s} {s['runs']:>4d} {s['runs_with_contact']:>5d} {_fmt(s['victims_found'])} "
              f"{_fmt(s['loc_mean_error_m'])} {_fmt(s['loc_worst_error_m'])} {_fmt(drift)} "
              f"{_fmt(s['loc_mean_nees'])} {_fmt(s['loc_mean_nees_clean_runs'], 9)} {_fmt(cell)} "
              f"{_fmt(outside, 8)} {_fmt(s['true_geofence_worst_excursion_m'])} {_fmt(s['safety_geofence_risk_ticks'], 7, 0)} "
              f"{_fmt(s['safety_safe_hold_ticks'], 7, 0)} {_fmt(s['true_altitude_min_m'])} {_fmt(s['true_altitude_max_m'])}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--profiles", default=",".join(PROFILES), help="comma-separated estimator profiles")
    ap.add_argument("--k-values", default="0,3", help="comma-separated safety_pose_sigma_geofence_k values")
    ap.add_argument("--num-seeds", type=int, default=4)
    ap.add_argument("--duration", type=float, default=30.0, help="seconds of simulated flight per run")
    ap.add_argument("--num-drones", type=int, default=4)
    ap.add_argument("--no-baseline", action="store_true", help="skip the truth_state baseline")
    ap.add_argument("--quick", action="store_true", help="tiny smoke sweep (ideal + stress, k 0 and 3, 1 seed, 6 s)")
    ap.add_argument("--out-dir", default=None)
    args = ap.parse_args(argv)

    if args.quick:
        args.profiles, args.k_values, args.num_seeds, args.duration = "ideal,stress", "0,3", 1, 6.0
    profiles = [p for p in args.profiles.split(",") if p]
    k_values = [float(k) for k in args.k_values.split(",") if k]
    scenarios = build_scenarios(profiles, k_values, include_baseline=not args.no_baseline)
    seeds = list(range(1, args.num_seeds + 1))
    out_dir = args.out_dir or os.path.join(RESULTS_ROOT, time.strftime("%Y%m%d-%H%M%S"))
    os.makedirs(out_dir, exist_ok=True)

    total = len(scenarios) * len(seeds)
    print(f"{len(scenarios)} scenarios x {len(seeds)} seeds = {total} runs of {args.duration:g} s, "
          f"{args.num_drones} drones -> {out_dir}")
    rows, manifests, done = [], {}, 0
    for scenario in scenarios:
        for seed in seeds:
            row, manifest = run_one(scenario, seed, args.duration, args.num_drones)
            rows.append(row)
            manifests.setdefault(scenario.name, manifest)        # one manifest per scenario (first seed)
            done += 1
            print(f"  [{done}/{total}] {scenario.name} seed {seed}: victims {row['victims_found']}/"
                  f"{row['total_victims']}, loc err {row['loc_mean_error_m'] if row['loc_mean_error_m'] is None else round(row['loc_mean_error_m'], 2)}, "
                  f"outside {row['true_geofence_outside_fraction']:.3f} ({row['wall_seconds']} s)")

    with open(os.path.join(out_dir, "runs.csv"), "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=RUN_COLUMNS)
        writer.writeheader()
        writer.writerows(rows)
    summary = summarize(rows)
    with open(os.path.join(out_dir, "summary.json"), "w") as f:
        json.dump({"scenarios": summary, "note": "illustrative drift models; see docs/PHASE16B_ESTIMATION.md"}, f,
                  indent=2, sort_keys=True)
    with open(os.path.join(out_dir, "manifests.json"), "w") as f:
        json.dump(manifests, f, indent=2, sort_keys=True, default=str)
    print()
    print_table(summary)
    print(f"\nwrote {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
