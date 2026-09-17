"""Phase 6: autopilot adapter boundary - see docs/PHASE6_AUTOPILOT_ADAPTERS.md.

    SwarmController -> SafetySupervisor -> Command/SafetyDecision
                              |
                              v
                     AutopilotAdapter interface
                    /          |            \\
              MockAdapter  ArduPilot      PX4
             (deterministic  skeleton    skeleton
              in-memory,     (offline    (offline
              this phase's    only)       only)
              reference path)
                              |
                              v
              Simulator vehicle (PyBullet, this phase)
              or a future real transport (not this phase)

Only PyBullet, mock/in-memory adapters, and deterministic offline adapter
tests are permitted execution targets in this phase - no MAVLink, no
SITL, no serial/UDP, no hardware. See each submodule's own docstring.
"""
from .ardupilot import ArduPilotAdapterSkeleton
from .base import AutopilotAdapter, OfflineSkeletonAdapter, is_command_fresh, is_navigation_command
from .frames import convert_command_frame, convert_vector_frame
from .mock import MockAdapter
from .px4 import PX4AdapterSkeleton
from .types import AdapterCommand, AdapterResult, AutopilotMode, ConnectionState, VehicleTelemetry

# Phase 7's SITLAdapter/ArduPilotSITLAdapterSkeleton/PX4SITLAdapterSkeleton
# (swarm_sim/autopilot/sitl.py, ardupilot_sitl.py, px4_sitl.py) are
# deliberately NOT re-exported here. swarm_sim.sitl.commands/telemetry
# import from swarm_sim.autopilot.types, and importing swarm_sim.autopilot.*
# always runs this __init__.py first - if this file also imported .sitl
# eagerly, importing swarm_sim.sitl before swarm_sim.autopilot would
# recurse back into a still-initializing swarm_sim.sitl.commands module
# (a genuine circular import, not just an ordering nuisance - see
# docs/PHASE7_SITL_INTEGRATION.md). Import them directly instead:
# `from swarm_sim.autopilot.sitl import SITLAdapter`, etc.

__all__ = [
    "AutopilotAdapter",
    "OfflineSkeletonAdapter",
    "is_command_fresh",
    "is_navigation_command",
    "MockAdapter",
    "ArduPilotAdapterSkeleton",
    "PX4AdapterSkeleton",
    "convert_command_frame",
    "convert_vector_frame",
    "AdapterCommand",
    "AdapterResult",
    "AutopilotMode",
    "ConnectionState",
    "VehicleTelemetry",
]
