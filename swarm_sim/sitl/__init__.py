"""Phase 7: local, offline SITL integration boundary - see
docs/PHASE7_SITL_INTEGRATION.md.

    SwarmController -> SafetySupervisor.evaluate() -> validated Command
                                    |
                                    v
                          AutopilotAdapter (swarm_sim.autopilot.sitl.SITLAdapter)
                                    |
                                    v
                          SITLTransport (this package's own protocol)
                                /                              \\
                    FakeSITLTransport (this phase)      a future real ArduPilot/PX4
                    - deterministic, in-memory,           SITL transport (NOT this phase -
                      no sockets/serial/MAVLink            no such transport exists in this
                                                            package; see autopilot/ardupilot_sitl.py
                                                            and autopilot/px4_sitl.py, which remain
                                                            pure offline skeletons)

Only local, in-process, deterministic execution is permitted in this
phase - no real SITL binary, no MAVLink/MAVSDK, no serial/UDP socket, no
hardware. See each submodule's own docstring and
tests/test_sitl_architecture.py.
"""
from .clock import SimClock
from .commands import SITLCommand, derive_safety_decision_id
from .fake_transport import FakeSITLTransport
from .telemetry import CommandAck, FailureType, SITLTelemetry, TransportResult
from .transport import SITLTransport
from .vehicle_namespace import NamespaceError, VehicleNamespaceRegistry, namespace_for

__all__ = [
    "SimClock",
    "SITLCommand",
    "derive_safety_decision_id",
    "FakeSITLTransport",
    "CommandAck",
    "FailureType",
    "SITLTelemetry",
    "TransportResult",
    "SITLTransport",
    "NamespaceError",
    "VehicleNamespaceRegistry",
    "namespace_for",
]
