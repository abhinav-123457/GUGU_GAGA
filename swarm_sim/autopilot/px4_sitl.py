"""Phase 7: PX4-SITL-targeted adapter skeleton - see
docs/PHASE7_SITL_INTEGRATION.md's "future PX4 SITL mapping" section.

This is NOT a connection to PX4 SITL. It is the same
`OfflineSkeletonAdapter` pattern Phase 6 established
(swarm_sim/autopilot/px4.py): `connect()` always fails, which structurally
makes every other method's "must be connected" check fail too. A future,
separately reviewed phase would replace this class's transport with a real
`SITLTransport` implementation that actually launches/attaches to a
`px4` SITL binary over local loopback MAVSDK/MAVLink (see
`swarm_sim.sitl.commands.MAVLINK_*_MAPPING_SPEC` for the naming this
project has already worked out for that eventual mapping) - no such
transport exists anywhere in this codebase today.
"""
from __future__ import annotations

from .base import OfflineSkeletonAdapter


class PX4SITLAdapterSkeleton(OfflineSkeletonAdapter):
    PROTOCOL_NAME = "PX4-SITL/MAVSDK (offline skeleton, no SITL process)"
