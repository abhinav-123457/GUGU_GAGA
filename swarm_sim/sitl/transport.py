"""Phase 7: SITLTransport protocol - see docs/PHASE7_SITL_INTEGRATION.md.

This is the boundary swarm_sim.autopilot.sitl.SITLAdapter sits on top of:

    AutopilotAdapter (SITLAdapter) -> SITLTransport -> FakeSITLTransport (this phase)
                                                     \\-> a future real ArduPilot/PX4 SITL transport (not this phase)

Adapted from the spec's "methods similar to" list: `receive_telemetry`,
`receive_ack`, and `inject_failure` take an explicit `vehicle_id` because a
single transport instance can (and, per Phase 7's namespace-isolation
requirement, must) host multiple vehicles at once - see
fake_transport.py's own docstring for why one shared multi-vehicle
transport, not one instance per vehicle, is what actually exercises
namespace isolation. `send_command` already carries a vehicle id (via
SITLCommand); `start`/`stop`/`set_sim_time` remain transport-global exactly
as specified, since they describe the local SITL process itself, not any
one vehicle within it.

No pymavlink/MAVSDK/serial/socket import anywhere in this module - see
tests/test_sitl_architecture.py.
"""
from __future__ import annotations

from typing import Protocol, runtime_checkable

from .commands import SITLCommand
from .telemetry import CommandAck, FailureType, SITLTelemetry, TransportResult


@runtime_checkable
class SITLTransport(Protocol):
    def start(self) -> TransportResult: ...

    def stop(self) -> TransportResult: ...

    def send_command(self, command: SITLCommand) -> TransportResult: ...

    def receive_telemetry(self, vehicle_id: str) -> SITLTelemetry: ...

    def receive_ack(self, vehicle_id: str) -> CommandAck: ...

    def set_sim_time(self, now_s: float) -> None: ...

    def inject_failure(self, vehicle_id: str, failure: FailureType, active: bool = True) -> None: ...
