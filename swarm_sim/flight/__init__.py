"""Phase 16C: per-drone flight control that runs on the drone's OWN estimated state
(see docs/PHASE16C_FLIGHT_LIFECYCLE.md). Nothing in this package may import ground truth, the
plant-side actuator or any GNSS source - tests/test_phase16c_architecture.py enforces it."""
from .vertical import VerticalConfig, VerticalController

__all__ = ["VerticalConfig", "VerticalController"]
