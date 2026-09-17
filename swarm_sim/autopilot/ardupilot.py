"""Phase 6: ArduPilot adapter - OFFLINE SKELETON ONLY.

See docs/PHASE6_AUTOPILOT_ADAPTERS.md's "ArduPilot skeleton limitations"
section for the full explanation. In short: this class proves the
`AutopilotAdapter` interface CAN be implemented for a future
ArduPilot/MAVLink transport, without touching `SwarmController` or
`SafetySupervisor` - it does not implement that transport. There is no
MAVLink here, no serial/UDP socket, no connection of any kind:
`ArduPilotAdapterSkeleton.connect()` always fails, which structurally
prevents every other method from ever doing anything (see
`OfflineSkeletonAdapter`, which this class only relabels).

Do not import pymavlink, MAVSDK, serial, or socket in this file - see
tests/test_autopilot_architecture.py, which asserts exactly that.

Future-phase note: a real ArduPilot transport would map this project's
`AutopilotMode.OFFBOARD` request to ArduPilot's own `GUIDED` flight mode
(ArduPilot does not have a mode literally named "OFFBOARD" - that is
PX4's term) - see px4.py for the PX4-side equivalent note. Wiring in that
real mapping, and a real MAVLink connection, is explicitly a later,
separately reviewed phase (see this project's Phase 6 request: "Only
after this phase is complete should you consider a later, separately
reviewed SITL integration phase").
"""
from __future__ import annotations

from .base import OfflineSkeletonAdapter


class ArduPilotAdapterSkeleton(OfflineSkeletonAdapter):
    """Offline skeleton for a future ArduPilot/MAVLink transport. See
    module docstring - `connect()` always fails; nothing here can reach a
    real vehicle."""

    PROTOCOL_NAME = "ArduPilot/MAVLink"
