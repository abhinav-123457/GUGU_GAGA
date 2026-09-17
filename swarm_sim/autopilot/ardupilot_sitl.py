"""Phase 7: ArduPilot-SITL-targeted adapter skeleton - see
docs/PHASE7_SITL_INTEGRATION.md's "future ArduPilot SITL mapping" section.

This is NOT a connection to ArduPilot SITL. It is the same
`OfflineSkeletonAdapter` pattern Phase 6 established
(swarm_sim/autopilot/ardupilot.py): `connect()` always fails, which
structurally makes every other method's "must be connected" check fail
too. A future, separately reviewed phase would replace this class's
transport with a real `SITLTransport` implementation that actually
launches/attaches to an `arducopter` SITL binary over local loopback
MAVLink (see `swarm_sim.sitl.commands.MAVLINK_*_MAPPING_SPEC` for the
naming this project has already worked out for that eventual mapping) -
no such transport exists anywhere in this codebase today.
"""
from __future__ import annotations

from .base import OfflineSkeletonAdapter


class ArduPilotSITLAdapterSkeleton(OfflineSkeletonAdapter):
    PROTOCOL_NAME = "ArduPilot-SITL/MAVLink (offline skeleton, no SITL process)"
