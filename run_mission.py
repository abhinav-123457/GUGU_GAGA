import argparse
import os

from swarm_sim.config import MissionConfig
from swarm_sim.mission import FloodSearchMission


def main():
    parser = argparse.ArgumentParser(description="Bio-inspired drone swarm SAR simulation (flood scenario)")
    parser.add_argument("--drones", type=int, default=6)
    parser.add_argument("--victims", type=int, default=5)
    parser.add_argument("--obstacles", type=int, default=4)
    parser.add_argument("--duration", type=float, default=90.0)
    parser.add_argument("--flock-model", choices=["couzin", "boids", "vicsek"], default="couzin")
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
    args = parser.parse_args()

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
    )

    mission = FloodSearchMission(cfg)
    result = mission.run()

    print("\nMission summary:")
    print(f"  Flock model:        {cfg.flock_model}")
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

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    result["telemetry"].save_csv(args.out)
    print(f"  Telemetry saved to: {args.out}")


if __name__ == "__main__":
    main()
