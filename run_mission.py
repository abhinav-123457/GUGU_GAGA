import argparse
import os

from swarm_sim.config import MissionConfig
from swarm_sim.estimation import PROFILES
from swarm_sim.mission import FloodSearchMission


def build_parser():
    """Split out from main() so tests/test_run_mission_cli.py can exercise
    argument parsing (e.g. --flock-model's accepted/rejected choices)
    without running a full mission."""
    parser = argparse.ArgumentParser(description="Bio-inspired drone swarm SAR simulation (flood scenario)")
    parser.add_argument("--drones", type=int, default=6)
    parser.add_argument("--victims", type=int, default=5)
    parser.add_argument("--obstacles", type=int, default=4)
    parser.add_argument("--duration", type=float, default=90.0)
    parser.add_argument("--flock-model", choices=["couzin", "boids", "vicsek", "olfati_saber"], default="couzin")
    parser.add_argument("--max-speed", type=float, default=5.0, help="per-drone speed cap, m/s")
    parser.add_argument("--cruise-speed", type=float, default=2.5, help="nominal search speed, m/s")
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--out", type=str, default="results/telemetry.csv")
    parser.add_argument("--comm-range", type=float, default=15.0, help="UAV<->UAV radio range, meters")
    parser.add_argument("--packet-loss", type=float, default=0.6,
                         help="packet loss probability for a link right at --comm-range (0-1)")
    parser.add_argument("--comm-latency", type=int, default=2, help="message latency, control steps")
    parser.add_argument("--consensus-quorum", type=int, default=2,
                         help="independent drones' reports required to confirm a detection")
    parser.add_argument("--consensus-mode", choices=["distributed", "centralized"], default="distributed",
                         help="distributed: peer-local consensus over CommsNetwork (Phase 5, default); "
                              "centralized: reference ConsensusBoard implementation, for comparison")
    parser.add_argument("--localization", choices=["truth_state", "estimated"], default="truth_state",
                         help="truth_state: drones act on simulator ground truth (default, legacy); "
                              "estimated: GNSS-denied - each drone dead-reckons its own drifting pose "
                              "(Phase 16B, docs/PHASE16B_ESTIMATION.md)")
    parser.add_argument("--estimator-profile", choices=sorted(PROFILES), default="fused",
                         help="odometry drift profile used with --localization estimated "
                              "(illustrative, not hardware-verified)")
    parser.add_argument("--estimator-noise-scale", type=float, default=1.0,
                         help="estimator noise multiplier; < 1 makes the estimator over-confident")
    parser.add_argument("--geofence-sigma-k", type=float, default=0.0,
                         help="widen the geofence response by k x the drone's own position uncertainty "
                              "(0 = off, the legacy behaviour)")
    parser.add_argument("--flight-control", choices=["legacy", "altitude_hold"], default="legacy",
                         help="legacy: the pre-16C vertical channel, which damps but does not hold altitude "
                              "(default); altitude_hold: an outer altitude loop on the drone's own estimated "
                              "altitude (Phase 16C, docs/PHASE16C_FLIGHT_LIFECYCLE.md)")
    return parser


def main():
    args = build_parser().parse_args()

    cfg = MissionConfig(
        num_drones=args.drones,
        num_victims=args.victims,
        num_obstacles=args.obstacles,
        duration_sec=args.duration,
        flock_model=args.flock_model,
        max_speed_mps=args.max_speed,
        cruise_speed_mps=args.cruise_speed,
        gui=args.gui,
        seed=args.seed,
        communication_radius=args.comm_range,
        comm_dropout_at_max_range=args.packet_loss,
        comm_latency_steps=args.comm_latency,
        consensus_quorum=args.consensus_quorum,
        consensus_mode=args.consensus_mode,
        localization_mode=args.localization,
        estimator_profile=args.estimator_profile,
        estimator_assumed_noise_scale=args.estimator_noise_scale,
        safety_pose_sigma_geofence_k=args.geofence_sigma_k,
        flight_control_mode=args.flight_control,
    )

    mission = FloodSearchMission(cfg)
    result = mission.run()

    print("\nMission summary:")
    print(f"  Flock model:        {cfg.flock_model}")
    print(f"  Consensus mode:     {result['consensus_mode']}")
    print(f"  Victims found:      {result['victims_found']}/{result['total_victims']}")
    print(f"  False confirmations:{result['false_confirmations']:>3}  (independent reports that agreed on empty space)")
    print(f"  Swarm connectivity: {result['swarm_connectivity_fraction'] * 100:.1f}% of steps fully connected")
    print(f"  Time elapsed:       {result['time_elapsed']:.1f}s")
    print("\nPhase 2 sensing (see docs/PHASE2_SENSING.md):")
    print(f"  Sensor detections:  {result['true_positive_detections']} true-positive, "
          f"{result['false_positive_detections']} false-positive, {result['missed_detections']} missed "
          f"(of {result['observation_opportunities']} observable opportunities)")
    if result["mean_localization_error_m"] is not None:
        print(f"  Mean localization error: {result['mean_localization_error_m']:.2f} m")
    if result["mean_neighbor_observation_age_s"] is not None:
        print(f"  Mean neighbor observation age: {result['mean_neighbor_observation_age_s']:.3f} s")
    print(f"  Min sensor-observed neighbor clearance: {result['min_sensor_observed_clearance_m']} m")
    print(f"  Min ground-truth neighbor clearance (scoring only): {result['min_ground_truth_clearance_m']} m")
    print(f"  Steps with a PyBullet contact involving a drone: {result['contact_steps']}")
    geo = result["true_geofence"]
    print(f"  True geofence excursions (scoring only): {geo['ticks_outside']}/{geo['ticks']} drone-ticks outside, "
          f"worst {geo['max_excursion_m']:.2f} m")
    alt = result["flight"]["altitude"]
    if alt["samples"]:
        roots = result["flight"]["contacts"]["first_contact_root_cause_per_drone"]
        print(f"\nPhase 16C flight control ({result['flight_control_mode']}), true altitude after the first 2 s: "
              f"std {alt['std_m']:.3f} m, range [{alt['min_m']:.2f}, {alt['max_m']:.2f}] m; first contacts by root cause "
              f"{roots}")
    loc = result["localization"]
    if loc is not None:
        print(f"\nPhase 16B GNSS-denied localization ({result['estimator_profile']} profile, illustrative drift model):")
        print(f"  Estimate error: mean {loc['mean_error_m']:.2f} m, worst {loc['max_error_m']:.2f} m; "
              f"final drift {100 * (loc['mean_final_drift_fraction'] or 0.0):.2f}% of path")
        print(f"  NEES {loc['mean_nees']:.2f} (2 = consistent, higher = over-confident); "
              f"estimated 1x1 m cell was the true one {100 * loc['mean_cell_match_fraction']:.0f}% of the time")
        print(f"  Estimator invalid {100 * loc['mean_invalid_tick_fraction']:.1f}% of ticks; "
              f"beacons raised for unverified confirmations: {result['synthetic_beacon_count']}")

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    result["telemetry"].save_csv(args.out)
    print(f"  Telemetry saved to: {args.out}")


if __name__ == "__main__":
    main()
