"""Phase 16C validation gate: does altitude control survive noisy estimated vertical speed, and what do
the drones touch? See docs/PHASE16C_FLIGHT_LIFECYCLE.md.

For every (flight_control_mode x localisation profile x seed) it runs a real `FloodSearchMission` and records the
TRUE altitude behaviour (mean/std/extremes after a 2 s settle), how many drones touched something and the root
cause of each drone's first contact (vertical_control / obstacle / swarm), plus localisation error and survivors.

Two extra profiles reproduce the Phase 16B failure on purpose (test-only; registered only while a run is
built, never left in the global profile table):
  flow_rf_oldvz  flow_rf with vertical-speed noise at the horizontal 0.10 m/s density
  stress_vz3     stress with vertical-speed noise at 0.15 m/s (3x the shipped stress value)

Gate G2 (evaluated on the `altitude_hold` rows only; `legacy` rows are reported alongside on the same seeds):
  * zero drones whose first contact was `vertical_control` (an upright drone reaching the ground);
  * every drone stays within +/-0.5 m of the cruise altitude after the settle time and BEFORE its first contact;
  * the median per-run median-per-drone altitude standard deviation, over the same pre-contact samples, is <= 0.10 m.
The pre-contact restriction is deliberate: a drone that collides and then tumbles to the ground has a huge
altitude error that is a consequence of the collision, not evidence about the altitude controller. The
unrestricted all-samples numbers are still reported (`alt_*` columns) next to the pre-contact ones (`pre_*`).
`attitude_loss` (a tumble before a ground strike), `obstacle` and `swarm` first-contact counts are reported, not
gated: they are horizontal-flight behaviour that 16C does not change.

Usage:
    python scripts/run_phase16c_gate.py                       # 12 seeds x 30 s, all profiles, both modes
    python scripts/run_phase16c_gate.py --quick               # tiny smoke run
    python scripts/run_phase16c_gate.py --modes altitude_hold --profiles vio,stress --num-seeds 4
Results go to results/phase16c_gate/<run-id>/ (gitignored): runs.csv, summary.json, manifests.json."""
import argparse
import contextlib
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
from swarm_sim.estimation import profiles as _profiles
from swarm_sim.manifest import build_run_manifest
from swarm_sim.mission import FloodSearchMission

RESULTS_ROOT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "results", "phase16c_gate")

CRUISE_M = 3.0
BAND_M = 0.5                    # G2: every drone within +/- this of cruise after the settle time
MEDIAN_STD_LIMIT_M = 0.10       # G2: median per-run median-per-drone altitude std
MODES = ("legacy", "altitude_hold")
TRUTH = "truth_state"

# Test-only profiles that reproduce the Phase 16B vertical-noise failure.
_EXTRA_PROFILES = {
    "flow_rf_oldvz": ("flow_rf", "_FLOW_RF", 0.10),
    "stress_vz3": ("stress", "_STRESS", 0.15),
}


def extra_profile(name: str):
    """The test-only profile `name` (see _EXTRA_PROFILES), built on demand from the shipped one."""
    base, spec_attr, vz = _EXTRA_PROFILES[name]
    spec = dataclasses.replace(getattr(_profiles, spec_attr), vz_noise_std_mps=vz)
    return dataclasses.replace(PROFILES[base], name=name, sensors=(spec,))


@contextlib.contextmanager
def profile_available(name: str):
    """Make `name` resolvable by `get_profile` for the duration of a run. Shipped profiles need nothing; the
    test-only ones are registered on entry and removed on exit so the global table is never left modified."""
    if name in _EXTRA_PROFILES and name not in PROFILES:
        PROFILES[name] = extra_profile(name)
        try:
            yield
        finally:
            PROFILES.pop(name, None)
    else:
        yield


def known_profiles() -> List[str]:
    return [TRUTH] + sorted(PROFILES) + sorted(n for n in _EXTRA_PROFILES if n not in PROFILES)


DEFAULT_PROFILES = [TRUTH, "ideal", "vio", "flow_rf", "lidar", "fused", "stress", "flow_rf_oldvz", "stress_vz3"]

RUN_COLUMNS = [
    "scenario", "flight_control_mode", "profile", "seed", "duration_s", "num_drones",
    "victims_found", "total_victims", "contact_steps", "drones_with_contact",
    "first_contact_vertical_control", "first_contact_attitude_loss", "first_contact_obstacle", "first_contact_swarm",
    "post_contact_ground_events",
    "alt_mean_abs_error_m", "alt_std_m", "alt_median_drone_std_m", "alt_max_abs_error_m", "alt_min_m", "alt_max_m",
    "alt_outside_band_fraction",
    "pre_std_m", "pre_median_drone_std_m", "pre_max_abs_error_m", "pre_min_m", "pre_max_m", "loc_mean_error_m", "loc_mean_nees", "wall_seconds",
]

SUMMARY_FIELDS = [
    ("victims_found", "victims_found", "mean"),
    ("contact_steps", "contact_steps", "mean"),
    ("alt_std_m", "alt_std_m", "mean"),
    ("alt_median_drone_std_m", "alt_median_drone_std_m", "median"),
    ("alt_worst_abs_error_m", "alt_max_abs_error_m", "max"),
    ("alt_min_m", "alt_min_m", "min"),
    ("alt_max_m", "alt_max_m", "max"),
    ("pre_median_drone_std_m", "pre_median_drone_std_m", "median"),
    ("pre_worst_abs_error_m", "pre_max_abs_error_m", "max"),
    ("pre_min_m", "pre_min_m", "min"),
    ("pre_max_m", "pre_max_m", "max"),
    ("loc_mean_error_m", "loc_mean_error_m", "mean"),
    ("loc_mean_nees", "loc_mean_nees", "mean"),
]


@dataclasses.dataclass(frozen=True)
class Scenario:
    name: str
    flight_control_mode: str
    profile: str          # "truth_state" or an estimator profile name


def build_scenarios(modes: Sequence[str], profiles: Sequence[str]) -> List[Scenario]:
    out = []
    for mode in modes:
        if mode not in MODES:
            raise ValueError(f"unknown flight_control_mode {mode!r}; known: {list(MODES)}")
        for profile in profiles:
            if profile not in known_profiles():
                raise ValueError(f"unknown profile {profile!r}; known: {known_profiles()}")
            out.append(Scenario(f"{mode}/{profile}", mode, profile))
    return out


def build_config(scenario: Scenario, seed: int, duration_s: float, num_drones: int) -> MissionConfig:
    kwargs = dict(num_drones=num_drones, duration_sec=duration_s, seed=seed, num_victims=3, num_obstacles=2,
                  flight_control_mode=scenario.flight_control_mode)
    if scenario.profile == TRUTH:
        kwargs["localization_mode"] = "truth_state"
    else:
        kwargs.update(localization_mode="estimated", estimator_profile=scenario.profile)
    return MissionConfig(**kwargs)


def run_one(scenario: Scenario, seed: int, duration_s: float, num_drones: int):
    cfg = build_config(scenario, seed, duration_s, num_drones)
    with profile_available(scenario.profile):
        mission = FloodSearchMission(cfg)         # resolves the estimator profile at construction
    t0 = time.time()
    res = mission.run()
    wall = time.time() - t0
    alt = res["flight"]["altitude"]
    pre = alt.get("pre_contact", {})
    contacts = res["flight"]["contacts"]
    roots = contacts["first_contact_root_cause_per_drone"]
    loc = res["localization"] or {}
    row = {
        "scenario": scenario.name, "flight_control_mode": scenario.flight_control_mode, "profile": scenario.profile,
        "seed": seed, "duration_s": duration_s, "num_drones": num_drones,
        "victims_found": res["victims_found"], "total_victims": res["total_victims"],
        "contact_steps": res["contact_steps"], "drones_with_contact": contacts["drones_with_a_contact"],
        "first_contact_vertical_control": roots["vertical_control"], "first_contact_attitude_loss": roots["attitude_loss"],
        "first_contact_obstacle": roots["obstacle"], "first_contact_swarm": roots["swarm"],
        "post_contact_ground_events": contacts["post_contact_ground_events"],
        "alt_mean_abs_error_m": alt.get("mean_abs_error_m"), "alt_std_m": alt.get("std_m"),
        "alt_median_drone_std_m": alt.get("median_per_drone_std_m"), "alt_max_abs_error_m": alt.get("max_abs_error_m"),
        "alt_min_m": alt.get("min_m"), "alt_max_m": alt.get("max_m"),
        "alt_outside_band_fraction": alt.get("outside_band_fraction"),
        "pre_std_m": pre.get("std_m"), "pre_median_drone_std_m": pre.get("median_per_drone_std_m"),
        "pre_max_abs_error_m": pre.get("max_abs_error_m"), "pre_min_m": pre.get("min_m"), "pre_max_m": pre.get("max_m"),
        "loc_mean_error_m": loc.get("mean_error_m"), "loc_mean_nees": loc.get("mean_nees"),
        "wall_seconds": round(wall, 2),
    }
    manifest = build_run_manifest(
        config=dataclasses.asdict(cfg), master_seed=seed, seed_manager=mission._sensor_seeds,
        map_id=res["mission_map_id"],
        model_id=f"flight={scenario.flight_control_mode}/localization={cfg.localization_mode}/profile={scenario.profile}",
    ).to_dict()
    return row, manifest


def summarize(rows: Sequence[Dict]) -> Dict[str, Dict]:
    agg = {"mean": np.mean, "median": np.median, "max": np.max, "min": np.min}
    out: Dict[str, Dict] = {}
    for name in dict.fromkeys(r["scenario"] for r in rows):
        group = [r for r in rows if r["scenario"] == name]
        summary: Dict = {
            "runs": len(group), "seeds": [r["seed"] for r in group],
            "runs_with_contact": sum(1 for r in group if (r.get("contact_steps") or 0) > 0),
            "drones_first_contact_vertical_control": sum(int(r.get("first_contact_vertical_control") or 0) for r in group),
            "drones_first_contact_attitude_loss": sum(int(r.get("first_contact_attitude_loss") or 0) for r in group),
            "drones_first_contact_obstacle": sum(int(r.get("first_contact_obstacle") or 0) for r in group),
            "drones_first_contact_swarm": sum(int(r.get("first_contact_swarm") or 0) for r in group),
        }
        for key, column, how in SUMMARY_FIELDS:
            vals = [r[column] for r in group if r.get(column) is not None]
            summary[key] = float(agg[how](vals)) if vals else None
        out[name] = summary
    return out


def evaluate_gate_g2(rows: Sequence[Dict], summary: Dict[str, Dict]) -> Dict[str, Dict]:
    """Per `altitude_hold` scenario: the three G2 criteria and whether they hold."""
    out = {}
    for name, s in summary.items():
        if not name.startswith("altitude_hold/"):
            continue
        group = [r for r in rows if r["scenario"] == name]
        worst = s["pre_worst_abs_error_m"]
        median_std = float(np.median([r["pre_median_drone_std_m"] for r in group if r["pre_median_drone_std_m"] is not None]))
        checks = {
            "no_vertical_control_contacts": s["drones_first_contact_vertical_control"] == 0,
            "within_band": worst is not None and worst <= BAND_M,
            "median_altitude_std_ok": median_std <= MEDIAN_STD_LIMIT_M,
        }
        out[name] = {"pass": all(checks.values()), "checks": checks, "worst_abs_error_m": worst,
                     "median_of_median_drone_std_m": median_std}
    return out


def _fmt(v, width=7, digits=3):
    return f"{'-':>{width}}" if v is None else f"{v:>{width}.{digits}f}"


def print_table(summary: Dict[str, Dict], gate: Dict[str, Dict]) -> None:
    header = (f"{'scenario':26s} {'runs':>4} {'contact':>7} {'vert':>5} {'tilt':>5} {'obst':>5} {'swarm':>5} {'victims':>7} "
              f"{'preStd':>7} {'preErr':>7} {'preMin':>7} {'preMax':>7} {'allMin':>7} {'allMax':>7} {'G2':>5}")
    print(header)
    print("-" * len(header))
    for name, s in summary.items():
        g = gate.get(name)
        verdict = "-" if g is None else ("PASS" if g["pass"] else "FAIL")
        print(f"{name:26s} {s['runs']:>4d} {s['runs_with_contact']:>7d} {s['drones_first_contact_vertical_control']:>5d} "
              f"{s['drones_first_contact_attitude_loss']:>5d} {s['drones_first_contact_obstacle']:>5d} "
              f"{s['drones_first_contact_swarm']:>5d} {_fmt(s['victims_found'], 7, 2)} "
              f"{_fmt(s['pre_median_drone_std_m'])} {_fmt(s['pre_worst_abs_error_m'])} {_fmt(s['pre_min_m'], 7, 2)} "
              f"{_fmt(s['pre_max_m'], 7, 2)} {_fmt(s['alt_min_m'], 7, 2)} {_fmt(s['alt_max_m'], 7, 2)} {verdict:>5}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--modes", default=",".join(MODES), help="comma-separated flight_control_mode values")
    ap.add_argument("--profiles", default=",".join(DEFAULT_PROFILES),
                    help="comma-separated: truth_state and/or estimator profile names")
    ap.add_argument("--num-seeds", type=int, default=12)
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--num-drones", type=int, default=4)
    ap.add_argument("--quick", action="store_true", help="tiny smoke run (truth_state + stress, 1 seed, 6 s)")
    ap.add_argument("--out-dir", default=None)
    args = ap.parse_args(argv)
    if args.quick:
        args.profiles, args.num_seeds, args.duration = f"{TRUTH},stress", 1, 6.0

    scenarios = build_scenarios([m for m in args.modes.split(",") if m], [p for p in args.profiles.split(",") if p])
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
            manifests.setdefault(scenario.name, manifest)
            done += 1
            print(f"  [{done}/{total}] {scenario.name} seed {seed}: alt std {row['alt_std_m']:.3f}, range "
                  f"[{row['alt_min_m']:.2f}, {row['alt_max_m']:.2f}], first contacts vert/tilt/obst/swarm "
                  f"{row['first_contact_vertical_control']}/{row['first_contact_attitude_loss']}/"
                  f"{row['first_contact_obstacle']}/{row['first_contact_swarm']} "
                  f"({row['wall_seconds']} s)", flush=True)

    with open(os.path.join(out_dir, "runs.csv"), "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=RUN_COLUMNS)
        writer.writeheader()
        writer.writerows(rows)
    summary = summarize(rows)
    gate = evaluate_gate_g2(rows, summary)
    with open(os.path.join(out_dir, "summary.json"), "w") as f:
        json.dump({"scenarios": summary, "gate_g2": gate, "band_m": BAND_M, "median_std_limit_m": MEDIAN_STD_LIMIT_M,
                   "note": "illustrative drift models; see docs/PHASE16C_FLIGHT_LIFECYCLE.md"}, f, indent=2, sort_keys=True)
    with open(os.path.join(out_dir, "manifests.json"), "w") as f:
        json.dump(manifests, f, indent=2, sort_keys=True, default=str)
    print()
    print_table(summary, gate)
    print(f"\nwrote {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
