"""Phase 6: PX4 adapter - OFFLINE SKELETON ONLY.

See docs/PHASE6_AUTOPILOT_ADAPTERS.md's "PX4 skeleton limitations"
section for the full explanation. In short: this class proves the
`AutopilotAdapter` interface CAN be implemented for a future PX4/MAVSDK
(or PX4-native MAVLink) transport, without touching `SwarmController` or
`SafetySupervisor` - it does not implement that transport. There is no
MAVSDK here, no MAVLink, no serial/UDP socket, no connection of any kind:
`PX4AdapterSkeleton.connect()` always fails, which structurally prevents
every other method from ever doing anything (see `OfflineSkeletonAdapter`,
which this class only relabels).

Do not import pymavlink, MAVSDK, serial, or socket in this file - see
tests/test_autopilot_architecture.py, which asserts exactly that.

Future-phase note: a real PX4 transport would map this project's
`AutopilotMode.OFFBOARD` request directly onto PX4's own OFFBOARD mode
(the two happen to share a name - see ardupilot.py's note on ArduPilot's
own GUIDED-mode equivalent). Wiring in that real mapping, and a real
MAVSDK/MAVLink connection, is explicitly a later, separately reviewed
phase (see this project's Phase 6 request: "Only after this phase is
complete should you consider a later, separately reviewed SITL
integration phase").
"""
from __future__ import annotations

from .base import OfflineSkeletonAdapter


class PX4AdapterSkeleton(OfflineSkeletonAdapter):
    """Offline skeleton for a future PX4/MAVSDK transport. See module
    docstring - `connect()` always fails; nothing here can reach a real
    vehicle."""

    PROTOCOL_NAME = "PX4/MAVSDK"
