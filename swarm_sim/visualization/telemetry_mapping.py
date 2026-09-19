"""Phase 11: pure telemetry <-> FlightGear FGNetFDM conversion - see
docs/PHASE11_FLIGHTGEAR_VIEWER.md's investigation section for the full
source citations. No MAVLink, no sockets, no I/O of any kind in this
module - every function here is a pure, independently-testable conversion.

**Frame conversion**: this project's ONLY canonical NED<->ENU relabeling
lives in `swarm_sim.autopilot.frames` (Phase 6) - `ned_to_enu`/`enu_to_ned`
below are thin, tested wrappers around it, never a separate/ad-hoc axis
swap (matching frames.py's own "there is exactly one conversion path"
principle). Per that module's documented convention: NED (x=north,
y=east, z=down) <-> ENU (x=east, y=north, z=up) is a fixed relabeling -
east = NED's y, north = NED's x, up = -NED's z.

**Why this module converts ENU back to NED before building an FGNetFDM
frame**: `swarm_sim.sitl.ardupilot_transport.ArduPilotSITLTransport`
already converts `LOCAL_POSITION_NED`'s raw wire values to this project's
canonical LOCAL_ENU convention (its default `operating_frame`) before
`receive_telemetry()` ever returns them - see that module's own
`_poll_incoming` comments. FlightGear's native FDM protocol, independently,
expects NED-labeled fields (`v_north`/`v_east`/`v_down`) - a real protocol
requirement, not this project's choice (source: FlightGear's own
`net_fdm.hxx`; see docs/PHASE11_FLIGHTGEAR_VIEWER.md). So this bridge's
data path is: ArduPilot wire NED -> (transport, already done) -> this
project's canonical ENU -> (this module) -> FlightGear-protocol NED. Both
conversions go through the one canonical utility; nothing here ever
special-cases FlightGear's axes.

**Yaw is the one field the transport partially converts** -
`ardupilot_transport.py`'s own comment: "roll/pitch are stored as
ArduPilot reports them (NED Euler convention); ... yaw is converted
NED->ENU (or left as-is)". So a telemetry snapshot read through the
transport (operating_frame=LOCAL_ENU, the default) has roll/pitch already
in the raw NED/body Euler convention FlightGear's phi/theta expect
directly, but yaw already converted to this project's ENU convention -
`yaw_enu_to_ned_for_flightgear` below undoes exactly that one conversion
(reusing frames.py's own involution formula) before it goes into psi.
Getting this backwards would point the rendered aircraft in the wrong
compass direction while still "looking" numerically plausible - which is
exactly the "guessed axes" failure mode this phase's own instructions warn
against, so it is covered by a dedicated test using a known heading.

**FGNetFDM protocol** (version 24 / 0x18): field list, order, and exact
struct layout (58 named fields all told, 408 bytes, no compiler padding)
are copied verbatim from ArduPilot's OWN working reference implementation
- `class FGNetFDM` in `libraries/SITL/SIM_JSBSim.h` (commit
`92b0cd788ec29406f26c6f9c31d5ceedbd1cc538`) - not FlightGear's upstream
source directly, since ArduPilot's copy is the one *this exact SITL
binary* has been proven (via `Tools/autotest/fg_quad_view.sh`) to
correctly interoperate with. Field semantics/units for the fields this
module actually sets are taken from that same header's own comments, cross-
checked against `SITL_State.cpp::_output_to_flightgear()`'s (the SEND
path, matching this bridge's own role) and `SIM_JSBSim.cpp::recv_fdm()`'s
(the RECEIVE path, a different ArduPilot feature that happens to share the
same struct) actual field usage. Full citations in
docs/PHASE11_FLIGHTGEAR_VIEWER.md.

Wire byte order is big-endian (network byte order) - ArduPilot's own
`FGNetFDM::ByteSwap()` calls `ntohl()` on every 32-bit word before
`fg_socket.send()`, which on the little-endian host SITL always runs on
converts host-native (little-endian) values to network (big-endian) order
for the wire; this module packs directly into that same big-endian form.

**Fields this module deliberately leaves at zero** (never fabricated):
`vcas` (ArduPilot's own two usages of this field disagree on units -
ft/s in the send path's `/0.3048` division, knots in the unrelated receive
path's `KNOTS_TO_METERS_PER_SECOND` multiply - so it is not reliably
documented even within ArduPilot's own source, and is not one of this
phase's required render fields); `alpha`/`beta`/`stall_warning`/`slip_deg`
(fixed-wing-only concepts, not meaningful for a multirotor); `v_body_*`/
`A_*_pilot` (this project's own telemetry never carries body-frame
velocity or acceleration - Phase 9 documented the same honest
simplification for acceleration); `num_engines=0` and all engine/gear/fuel
arrays (this project's telemetry has no per-motor RPM data, so it reports
zero engines rather than animate rotors that were never actually measured).
"""
from __future__ import annotations

import math
import struct
from typing import Dict, Mapping, Sequence, Tuple

from ..autopilot.frames import _yaw_ned_to_enu  # involution - reused both directions, see module docstring
from ..autopilot.frames import convert_vector_frame
from ..contracts import Frame

Vector3 = Tuple[float, float, float]

FG_NET_FDM_VERSION = 24  # 0x18 - frozen/stable (see module docstring)
_METERS_PER_FOOT = 0.3048


def ned_to_enu(vector_ned: Sequence[float]) -> Vector3:
    """(north, east, down) -> (east, north, up). Thin wrapper around this
    project's one canonical relabeling - see module docstring."""
    return convert_vector_frame(tuple(float(c) for c in vector_ned), Frame.LOCAL_NED, Frame.LOCAL_ENU)


def enu_to_ned(vector_enu: Sequence[float]) -> Vector3:
    """(east, north, up) -> (north, east, down) - the same relabeling run
    backward (it is its own inverse)."""
    return convert_vector_frame(tuple(float(c) for c in vector_enu), Frame.LOCAL_ENU, Frame.LOCAL_NED)


def yaw_enu_to_ned_for_flightgear(yaw_enu_rad: float) -> float:
    """Undo the transport's ENU yaw conversion before sending to
    FlightGear, which wants NED/compass-convention heading (psi) - see
    module docstring. `_yaw_ned_to_enu` is an involution (`pi/2 - yaw`), so
    reusing it in this direction is exact, not an approximation."""
    return _yaw_ned_to_enu(float(yaw_enu_rad))


def mps_to_fps(value_mps: float) -> float:
    return float(value_mps) / _METERS_PER_FOOT


# --------------------------------------------------------------------------
# FGNetFDM (version 24) exact field layout - see module docstring for the
# source citation. (name, struct-format-char, count) in wire order; this is
# the single source of truth for both the struct.Struct format string and
# the packing order below, so they can never drift apart.
# --------------------------------------------------------------------------
_FG_NET_FDM_FIELD_SPEC: Tuple[Tuple[str, str, int], ...] = (
    ("version", "I", 1), ("padding", "I", 1),
    ("longitude", "d", 1), ("latitude", "d", 1), ("altitude", "d", 1),
    ("agl", "f", 1), ("phi", "f", 1), ("theta", "f", 1), ("psi", "f", 1),
    ("alpha", "f", 1), ("beta", "f", 1),
    ("phidot", "f", 1), ("thetadot", "f", 1), ("psidot", "f", 1),
    ("vcas", "f", 1), ("climb_rate", "f", 1),
    ("v_north", "f", 1), ("v_east", "f", 1), ("v_down", "f", 1),
    ("v_body_u", "f", 1), ("v_body_v", "f", 1), ("v_body_w", "f", 1),
    ("a_x_pilot", "f", 1), ("a_y_pilot", "f", 1), ("a_z_pilot", "f", 1),
    ("stall_warning", "f", 1), ("slip_deg", "f", 1),
    ("num_engines", "I", 1), ("eng_state", "I", 4), ("rpm", "f", 4),
    ("fuel_flow", "f", 4), ("fuel_px", "f", 4), ("egt", "f", 4), ("cht", "f", 4),
    ("mp_osi", "f", 4), ("tit", "f", 4), ("oil_temp", "f", 4), ("oil_px", "f", 4),
    ("num_tanks", "I", 1), ("fuel_quantity", "f", 4),
    ("num_wheels", "I", 1), ("wow", "I", 3),
    ("gear_pos", "f", 3), ("gear_steer", "f", 3), ("gear_compression", "f", 3),
    ("cur_time", "I", 1), ("warp", "i", 1), ("visibility", "f", 1),
    ("elevator", "f", 1), ("elevator_trim_tab", "f", 1),
    ("left_flap", "f", 1), ("right_flap", "f", 1),
    ("left_aileron", "f", 1), ("right_aileron", "f", 1),
    ("rudder", "f", 1), ("nose_wheel", "f", 1),
    ("speedbrake", "f", 1), ("spoilers", "f", 1),
)

_FG_NET_FDM_STRUCT_FORMAT = ">" + "".join(f"{count}{ch}" for _, ch, count in _FG_NET_FDM_FIELD_SPEC)
_FG_NET_FDM_STRUCT = struct.Struct(_FG_NET_FDM_STRUCT_FORMAT)
FG_NET_FDM_STRUCT_SIZE = _FG_NET_FDM_STRUCT.size  # expected: 408 bytes - see module docstring


def pack_fg_net_fdm(fields: Mapping[str, object]) -> bytes:
    """Pack a {field_name: value} mapping into a wire-ready FGNetFDM frame.
    Any field not present in `fields` (or any array field) defaults to
    zero - never a fabricated/guessed value. Raises KeyError for a name
    that is not part of the documented struct (typo protection)."""
    known = {name for name, _, _ in _FG_NET_FDM_FIELD_SPEC}
    unknown = set(fields) - known
    if unknown:
        raise KeyError(f"not a documented FGNetFDM field: {sorted(unknown)}")

    values = []
    for name, _ch, count in _FG_NET_FDM_FIELD_SPEC:
        raw = fields.get(name)
        if count == 1:
            values.append(0 if raw is None else raw)
        else:
            seq = tuple(raw) if raw is not None else ()
            seq = seq + (0,) * (count - len(seq))
            values.extend(seq[:count])
    return _FG_NET_FDM_STRUCT.pack(*values)


def build_fg_net_fdm_fields(*, latitude_deg: float, longitude_deg: float, altitude_m: float, agl_m: float,
                             roll_rad: float, pitch_rad: float, yaw_enu_rad: float,
                             rollspeed_radps: float, pitchspeed_radps: float, yawspeed_radps: float,
                             velocity_enu_mps: Vector3) -> Dict[str, object]:
    """Build the {field_name: value} mapping for `pack_fg_net_fdm`, doing
    every unit/frame conversion this phase requires:

    - latitude/longitude: degrees (as read from MAVLink GLOBAL_POSITION_INT)
      -> radians (FGNetFDM's documented unit).
    - psi: this project's ENU yaw -> FlightGear/NED compass heading, via
      `yaw_enu_to_ned_for_flightgear` (see module docstring).
    - phi/theta: passed straight through - the transport already leaves
      roll/pitch in the raw NED/body Euler convention FlightGear expects
      (see module docstring).
    - v_north/v_east/v_down: this project's canonical ENU velocity ->
      NED (via `enu_to_ned`) -> feet/sec (FGNetFDM's documented unit).
    - climb_rate: -v_down (a positive climb is a negative NED-down rate),
      in feet/sec.

    Never sets `vcas`/`alpha`/`beta`/engine/gear/fuel fields - see module
    docstring for why those are deliberately left at zero.
    """
    yaw_ned_rad = yaw_enu_to_ned_for_flightgear(yaw_enu_rad)
    north_mps, east_mps, down_mps = enu_to_ned(velocity_enu_mps)
    v_north_fps, v_east_fps, v_down_fps = mps_to_fps(north_mps), mps_to_fps(east_mps), mps_to_fps(down_mps)

    return {
        "version": FG_NET_FDM_VERSION,
        "padding": 0,
        "longitude": math.radians(float(longitude_deg)),
        "latitude": math.radians(float(latitude_deg)),
        "altitude": float(altitude_m),
        "agl": float(agl_m),
        "phi": float(roll_rad),
        "theta": float(pitch_rad),
        "psi": yaw_ned_rad,
        "phidot": float(rollspeed_radps),
        "thetadot": float(pitchspeed_radps),
        "psidot": float(yawspeed_radps),
        "climb_rate": -v_down_fps,
        "v_north": v_north_fps,
        "v_east": v_east_fps,
        "v_down": v_down_fps,
        "num_engines": 0,
    }
