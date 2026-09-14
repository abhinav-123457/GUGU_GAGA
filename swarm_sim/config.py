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
    flock_model: str = "couzin"     # "couzin" | "boids" | "vicsek"
    vicsek_noise: float = 0.15

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
    # confirmed once independent drones' reports agree.
    sensor_range: float = 3.0            # onboard sensor detection range, meters (a cone, not a pinpoint)
    sensor_noise_std: float = 0.5        # gaussian noise (meters) on a raw sensed victim position
    consensus_quorum: int = 2            # independent drones' reports required before confirming
    consensus_cluster_radius: float = 3.0    # max spread among reports to count as the same detection
    consensus_window_sec: float = 10.0       # how long an unconfirmed report stays eligible

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
