"""Phase 11: the actual FlightGear wire connection - a thin, send-only UDP
socket wrapper. See docs/PHASE11_FLIGHTGEAR_VIEWER.md.

This module never imports MAVLink and never imports anything from
`swarm_sim.sitl`/`swarm_sim.autopilot` - it only knows how to turn an
already-built FGNetFDM byte frame (see `telemetry_mapping.py`) into one UDP
packet. Deliberately send-only: `FlightGearBridge` never calls `.recv()` on
its socket, so there is no code path in this module through which anything
FlightGear sends could ever reach - let alone influence - this project.
That is a structural property, not a documented promise: reading the
source is enough to see no receive path exists (see
tests/test_phase11_flightgear_viewer_architecture.py).
"""
from __future__ import annotations

import dataclasses
import socket
from typing import Optional

_LOOPBACK_HOSTS = ("127.0.0.1", "localhost", "::1")


class FlightGearBridgeError(Exception):
    pass


@dataclasses.dataclass(frozen=True)
class FlightGearEndpoint:
    host: str
    port: int


def parse_flightgear_endpoint(endpoint_str: str) -> FlightGearEndpoint:
    """Parse "host:port" and reject anything but a loopback host. Never
    guesses a default host/port - both must be explicit, per this phase's
    "must reject every non-loopback address" / "must be explicitly
    configured" requirements."""
    if ":" not in endpoint_str:
        raise FlightGearBridgeError(f"--flightgear-endpoint must be host:port, got {endpoint_str!r}")
    host, _, port_str = endpoint_str.rpartition(":")
    if host not in _LOOPBACK_HOSTS:
        raise FlightGearBridgeError(
            f"--flightgear-endpoint host must be loopback ({_LOOPBACK_HOSTS}), got {host!r}"
        )
    try:
        port = int(port_str)
    except ValueError as e:
        raise FlightGearBridgeError(f"--flightgear-endpoint port must be an integer, got {port_str!r}") from e
    if not (1 <= port <= 65535):
        raise FlightGearBridgeError(f"--flightgear-endpoint port out of range: {port}")
    return FlightGearEndpoint(host=host, port=port)


class FlightGearBridge:
    """Send-only UDP connection to a FlightGear `--native-fdm=socket,in,...`
    listener. Never arms, never takes off, never lands, never changes mode,
    never sends a setpoint - it only ever calls `socket.sendto()` with an
    already-packed FGNetFDM frame built by `telemetry_mapping.py`."""

    def __init__(self, endpoint: FlightGearEndpoint):
        self.endpoint = endpoint
        self._sock: Optional[socket.socket] = None
        self.sent_count = 0
        self.send_error_count = 0

    def open(self) -> None:
        if self._sock is not None:
            return
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_frame(self, frame_bytes: bytes) -> bool:
        """Send one already-packed frame. Returns True on success, False on
        any socket error (never raises - a dropped FlightGear frame must
        never crash telemetry logging or, more importantly, ever be
        retried in a way that could accumulate/backpressure real MAVLink
        reads)."""
        if self._sock is None:
            self.open()
        try:
            self._sock.sendto(frame_bytes, (self.endpoint.host, self.endpoint.port))
            self.sent_count += 1
            return True
        except OSError:
            self.send_error_count += 1
            return False

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None
