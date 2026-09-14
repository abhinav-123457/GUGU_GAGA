"""Per-drone speed governor.

Clamps a desired 3D velocity vector to a per-drone speed cap (in real m/s,
never exceeding the airframe's rated top speed) and hands back the vector
DSLPIDControl should track directly as target_vel. Caps can be changed
per-drone at runtime via set_max_speed.
"""
import numpy as np


class SpeedController:
    def __init__(self, num_drones, default_max_speed_mps, airframe_max_speed_mps):
        self.airframe_limit = airframe_max_speed_mps
        self.max_speed = np.full(num_drones, min(default_max_speed_mps, airframe_max_speed_mps), dtype=float)

    def set_max_speed(self, drone_id, mps):
        self.max_speed[drone_id] = float(np.clip(mps, 0.0, self.airframe_limit))

    def to_velocity_command(self, drone_id, desired_velocity):
        speed = float(np.linalg.norm(desired_velocity))
        if speed < 1e-6:
            return np.zeros(3), 0.0
        direction = desired_velocity / speed
        capped_speed = min(speed, self.max_speed[drone_id])
        return direction * capped_speed, capped_speed
