"""Phase 7/8: ArduPilot-SITL adapter - see
docs/PHASE7_SITL_INTEGRATION.md's "future ArduPilot SITL mapping" section
and docs/PHASE8_ARDUPILOT_SITL.md.

`ArduPilotSITLAdapterSkeleton` (Phase 7, unchanged) is NOT a connection to
ArduPilot SITL - it is the `OfflineSkeletonAdapter` pattern Phase 6
established: `connect()` always fails, which structurally makes every
other method's "must be connected" check fail too. It remains here as an
always-safe default placeholder (e.g. for code that wants "the
AutopilotAdapter interface, but definitely no transport").

`build_ardupilot_sitl_adapter` (Phase 8, new) is the real wiring: it
constructs a plain `swarm_sim.autopilot.sitl.SITLAdapter` (the SAME
generic class Phase 7 already wrote against the `SITLTransport` Protocol
for FakeSITL - unmodified) around a real
`swarm_sim.sitl.ardupilot_transport.ArduPilotSITLTransport`. No new
adapter class was needed for this - "reuse the existing SITLAdapter
interface" literally means there is nothing ArduPilot-specific about the
adapter layer at all; only the transport underneath it differs.
"""
from __future__ import annotations

from ..contracts import Frame
from ..sitl.ardupilot_transport import ArduPilotSITLTransport
from .base import OfflineSkeletonAdapter
from .sitl import SITLAdapter


class ArduPilotSITLAdapterSkeleton(OfflineSkeletonAdapter):
    PROTOCOL_NAME = "ArduPilot-SITL/MAVLink (offline skeleton, no SITL process)"


def build_ardupilot_sitl_adapter(vehicle_id: str, transport: ArduPilotSITLTransport,
                                  operating_frame: Frame = Frame.LOCAL_ENU) -> SITLAdapter:
    """Builds a `SITLAdapter` wired to a real `ArduPilotSITLTransport` for
    one vehicle already registered on that transport. `operating_frame`
    defaults to `Frame.LOCAL_ENU` - this project's own project-side
    internal convention (matching MockAdapter/FakeSITLTransport's own
    default, and what mission.py's SafetySupervisor output already uses)
    - and must match the `operating_frame` the `ArduPilotSITLTransport`
    itself was constructed with. ArduPilot's MAVLink messages always
    carry LOCAL_NED on the wire; `ArduPilotSITLTransport` is the one place
    that explicit ENU<->NED conversion happens (both directions - see its
    own `_send_mavlink_command`/`_poll_incoming`), never left implicit and
    never done here or anywhere upstream."""
    namespace = transport.registry.namespace_of(vehicle_id)
    return SITLAdapter(vehicle_id=vehicle_id, namespace=namespace, transport=transport,
                        operating_frame=operating_frame)
