from dataclasses import dataclass


@dataclass
class MissionConfig:
    """Tunable parameters for a flood search-and-rescue swarm mission."""

    # Swarm size / arena
    num_drones: int = 6
    arena_size: float = 40.0       # meters, side length of the open search area
    flight_altitude: float = 3.0

    # Mission content (flood scenario: open area, sparse obstacles)
    num_victims: int = 5
    num_obstacles: int = 4
    obstacle_radius: float = 1.0

    # Flocking / neighbor awareness (Couzin 2002 zone radii by default)
    sensing_radius: float = 6.0
    r_repulsion: float = 2.5
    r_orientation: float = 4.0
    r_attraction: float = 6.0
    fov_deg: float = 270.0
    flock_model: str = "couzin"     # "couzin" | "boids" | "vicsek" | "olfati_saber"
    vicsek_noise: float = 0.15      # uniform angular noise half-width, radians (Phase 3 - see behaviors/vicsek.py)
    max_turn_rate_radps: float = 3.0   # bounded-turn-rate half of the Phase 3 common controller
                                        # contract (swarm_sim/behaviors/common.py); used by the
                                        # standalone *_candidate_command entry points - the blended
                                        # search pipeline below already rate-limits via max_accel_mps2/
                                        # emergency_accel_mps2 on the resulting velocity instead.

    # Olfati-Saber-inspired controller (Phase 3 - see behaviors/olfati_saber.py
    # for which parts are from Olfati-Saber (2006) and which are engineering
    # approximations). d_alpha/r_alpha are in the same meters used elsewhere;
    # the module converts to sigma-norm units internally.
    os_desired_spacing_m: float = 4.0
    os_interaction_range_m: float = 6.0
    os_c_spacing: float = 1.0
    os_c_align: float = 1.0
    os_c_nav: float = 1.0

    # Levy-flight search bias (Viswanathan et al. 1999)
    levy_alpha: float = 1.5
    levy_weight: float = 1.0

    # Recruitment / stigmergic signaling (von Frisch 1967, Bonabeau et al. 1996)
    communication_radius: float = 15.0   # also doubles as the comms network's physical radio range
    arrival_radius: float = 1.5          # how close a recruited drone must get to mark a beacon serviced
    max_recruits_per_beacon: int = 2
    beacon_decay_rate: float = 0.02

    # Communication network realism (UAV<->UAV links, Olfati-Saber-style local
    # information graph instead of ground truth)
    comm_latency_steps: int = 2              # control steps a message takes to arrive
    comm_max_age_steps: int = 6              # a received sample older than this is dropped from the neighbor table
    comm_dropout_base: float = 0.02          # packet loss probability for a link at zero range
    comm_dropout_at_max_range: float = 0.6   # packet loss probability for a link right at communication_radius

    # Distributed consensus on victim detections (replaces single-drone instant
    # confirmation): each drone reports a noisy candidate; a location is only
    # confirmed once independent drones' reports agree. Unchanged in Phase 2 -
    # see swarm_sim/sensors.py for what now feeds it.
    consensus_quorum: int = 2            # independent drones' reports required before confirming
    consensus_cluster_radius: float = 3.0    # max spread among reports to count as the same detection
    consensus_window_sec: float = 10.0       # how long an unconfirmed report stays eligible

    # Phase 2 sensing: victim detection sensor (swarm_sim/sensors.py
    # VictimSensorModel). Replaces the old ground-truth-distance-gated
    # detection check - range/FOV/occlusion/noise/false-negative/
    # false-positive/latency/dropout all apply before anything reaches
    # ConsensusBoard.
    victim_sensor_range_m: float = 6.0
    victim_sensor_hfov_deg: float = 90.0         # horizontal field of view, full cone angle
    victim_sensor_vfov_deg: float = 140.0        # vertical FOV, full cone angle measured from nadir
                                                  # (straight down) - wide enough that a target at max
                                                  # victim_sensor_range_m and default flight_altitude
                                                  # is still inside the cone; see sensors.py for why
                                                  # nadir, not horizontal, is the cone's center
    victim_sensor_noise_std_m: float = 0.4       # gaussian noise on a genuine detection's reported position
    victim_sensor_false_negative_prob: float = 0.08   # probability an in-range, in-FOV, unoccluded victim is missed
    victim_sensor_false_positive_rate: float = 0.01   # probability per drone per step of a spurious detection
    victim_sensor_latency_steps: int = 1         # control steps between sensing and the observation being usable
    victim_sensor_dropout_prob: float = 0.03     # probability this tick's whole detection observation is lost
    victim_sensor_confidence: float = 0.85       # confidence assigned to a genuine detection
    victim_sensor_false_positive_confidence: float = 0.45   # confidence assigned to a spurious detection
    victim_sensor_localization_uncertainty_m: float = 0.4   # reported 1-sigma uncertainty radius

    # Phase 2 sensing: obstacle range sensor (swarm_sim/sensors.py
    # ObstacleRangeSensor). Replaces SwarmController's direct read of
    # ground-truth obstacle centers with a fixed-angular-bin range scan.
    obstacle_sensor_range_m: float = 6.0
    obstacle_sensor_fov_deg: float = 270.0
    obstacle_sensor_num_bins: int = 16
    obstacle_sensor_noise_std_m: float = 0.25
    obstacle_sensor_dropout_prob: float = 0.02   # a dropped-out scan reads as "nothing in range" (+inf), not a
                                                   # separate flag - fail-empty, same as a real sensor with no return
    obstacle_sensor_latency_steps: int = 0

    # Phase 2 sensing: local neighbor proximity sensor (swarm_sim/sensors.py
    # NeighborSensorModel). Replaces SwarmController's direct read of the
    # ground-truth drone-position array for collision avoidance. Distinct
    # from CommsNetwork/sensing_radius above, which remain the (already
    # comms-realistic) channel flocking cohesion uses - this sensor is
    # onboard (no radio), used only by the hard-safety avoidance branch, so
    # a degraded comms link still can never be what degrades collision
    # safety.
    neighbor_sensor_range_m: float = 6.0
    neighbor_sensor_fov_deg: float = 270.0
    neighbor_sensor_noise_std_m: float = 0.3
    neighbor_sensor_velocity_noise_std_mps: float = 0.2
    neighbor_sensor_latency_steps: int = 0
    neighbor_sensor_dropout_prob: float = 0.03
    neighbor_sensor_stale_timeout_steps: int = 3   # age beyond which a still-cached reading is marked stale
    neighbor_sensor_confidence: float = 0.9

    # Speed control
    cruise_speed_mps: float = 2.5
    max_speed_mps: float = 5.0
    max_accel_mps2: float = 2.0     # rate-limits commanded velocity changes so PID setpoints stay flyable
    emergency_accel_mps2: float = 4.0   # allowance for active collision avoidance (repulsion zone)
    repulsion_speed_mps: float = 1.0    # avoidance slows down rather than swerving at full cruise speed -
                                         # keeps velocity-tracking error (and thus commanded tilt) small

    # Simulation
    duration_sec: float = 90.0
    control_freq_hz: int = 24
    sim_freq_hz: int = 240
    seed: int = 42
    gui: bool = False
