"""Phase 16B: GNSS-denied state estimation. See docs/PHASE16B_ESTIMATION.md.

Only the truth-free names are re-exported here on purpose. `plant_odometry`
and `metrics` are the two modules allowed to see ground truth; import them
explicitly by module path so every use is visible and greppable.
"""
from .bridge import health_state_of, pose_uncertainty_m, to_vehicle_state
from .estimator import FixResult, StateEstimator
from .models import EstimatedState, OdometryBundle, OdometryMeasurement
from .profiles import PROFILES, DriftProfile, SensorDriftSpec, get_profile

__all__ = [
    "DriftProfile", "EstimatedState", "FixResult", "OdometryBundle", "OdometryMeasurement", "PROFILES",
    "SensorDriftSpec", "StateEstimator", "get_profile", "health_state_of", "pose_uncertainty_m",
    "to_vehicle_state",
]
