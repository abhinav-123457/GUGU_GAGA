"""Central telemetry hub. Every drone's state is pushed here every control
step, so the full swarm's telemetry is available from one place (queryable
live via all_latest(), persisted via save_csv())."""
import csv
from dataclasses import dataclass, field


@dataclass
class TelemetryHub:
    log: list = field(default_factory=list)
    latest: dict = field(default_factory=dict)

    def record(self, t, drone_id, pos, vel, rpy, speed, mode, target_speed, num_neighbors=0):
        entry = {
            "t": round(float(t), 3),
            "drone_id": drone_id,
            "x": float(pos[0]), "y": float(pos[1]), "z": float(pos[2]),
            "vx": float(vel[0]), "vy": float(vel[1]), "vz": float(vel[2]),
            "speed": float(speed),
            "target_speed": float(target_speed),
            "roll": float(rpy[0]), "pitch": float(rpy[1]), "yaw": float(rpy[2]),
            "mode": mode,
            "num_neighbors": int(num_neighbors),
        }
        self.log.append(entry)
        self.latest[drone_id] = entry

    def all_latest(self):
        """Snapshot of every drone's most recent telemetry, keyed by drone id."""
        return dict(self.latest)

    def save_csv(self, path):
        if not self.log:
            return
        with open(path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(self.log[0].keys()))
            writer.writeheader()
            writer.writerows(self.log)
