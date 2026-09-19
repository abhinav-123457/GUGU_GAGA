"""Phase 11 functional tests - see docs/PHASE11_FLIGHTGEAR_VIEWER.md and
scripts/run_phase11_flightgear_viewer.py. Architecture/AST checks live in
tests/test_phase11_flightgear_viewer_architecture.py.

`_FlightGearTestVehicle` extends `_dummy_ardupilot_vehicle.DummyArduPilotVehicle`
(the same real-MAVLink-over-a-real-localhost-socket fixture Phase 8/9/10's
own tests use) with an unconditional GLOBAL_POSITION_INT stream (the base
fixture already streams LOCAL_POSITION_NED/ATTITUDE/etc. unconditionally
regardless of any requested rate - this mirrors that same simplification,
never modifying the shared fixture itself).
"""
import csv
import itertools
import math
import os
import socket
import struct
import sys
import time

import pytest
from pymavlink import mavutil

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"))

import run_phase11_flightgear_viewer as phase11  # noqa: E402

from swarm_sim.visualization import telemetry_mapping as tmap  # noqa: E402

from _dummy_ardupilot_vehicle import DummyArduPilotVehicle  # noqa: E402

_port_counter = itertools.count(19300)


def _next_port() -> int:
    return next(_port_counter)


class _Namespace:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


def _args(**overrides):
    port = overrides.pop("port", 0)
    defaults = dict(
        connection=f"tcp:127.0.0.1:{port}", system_id=1, component_id=1, namespace="sitl/drone0",
        startup_timeout=3.0, duration=1.0, update_rate_hz=10.0,
        dry_run=False, diagnose=False, attach=False, flightgear_view=False,
        flightgear_endpoint=None, replay=None,
    )
    defaults.update(overrides)
    return _Namespace(**defaults)


class _FlightGearTestVehicle(DummyArduPilotVehicle):
    """See module docstring. Fixed, known lat/lon/alt/agl reported on every
    telemetry tick - enough to exercise the read-only GLOBAL_POSITION_INT
    path without needing real geodetic math in the test double."""

    def __init__(self, *args, lat_deg=-35.363261, lon_deg=149.165230, altitude_m=584.5, agl_m=1.72, **kwargs):
        super().__init__(*args, **kwargs)
        self.lat_deg = lat_deg
        self.lon_deg = lon_deg
        self.altitude_m = altitude_m
        self.agl_m = agl_m

    def _send_telemetry(self):
        super()._send_telemetry()
        if self._conn.port is not None:
            self._conn.mav.global_position_int_send(
                0, int(self.lat_deg * 1.0e7), int(self.lon_deg * 1.0e7),
                int(self.altitude_m * 1000), int(self.agl_m * 1000),
                0, 0, 0, 65535,
            )


@pytest.fixture
def fg_vehicle():
    port = _next_port()
    v = _FlightGearTestVehicle(port=port, system_id=1, component_id=1)
    v.start()
    yield v, port
    v.stop()


# ==========================================================================
# 0. telemetry_mapping.py - pure axis/unit conversion, from the actual
#    protocol documentation (see module docstring's citations) - never
#    guessed. No MAVLink, no sockets, no live vehicle needed for any of
#    these.
# ==========================================================================

def test_ned_to_enu_matches_documented_formula():
    # north=1, east=2, down=-3 -> east=2, north=1, up=3 (frames.py's own
    # documented NED<->ENU relabeling).
    assert tmap.ned_to_enu((1.0, 2.0, -3.0)) == (2.0, 1.0, 3.0)


def test_enu_to_ned_is_the_inverse_relabeling():
    assert tmap.enu_to_ned((2.0, 1.0, 3.0)) == (1.0, 2.0, -3.0)


def test_ned_enu_round_trip_is_identity():
    original = (5.5, -2.25, 0.75)
    assert tmap.enu_to_ned(tmap.ned_to_enu(original)) == original


def test_yaw_enu_to_ned_facing_east_is_compass_heading_90deg():
    # ENU yaw 0 rad = facing +x (East-like, per frames.py's own convention).
    # FlightGear/NED psi is a compass heading (0=North, clockwise-positive),
    # so facing East must be pi/2 (90 degrees).
    assert tmap.yaw_enu_to_ned_for_flightgear(0.0) == pytest.approx(math.pi / 2.0)


def test_yaw_enu_to_ned_facing_north_is_compass_heading_0deg():
    # ENU yaw pi/2 = facing +y (North-like) -> compass heading 0.
    assert tmap.yaw_enu_to_ned_for_flightgear(math.pi / 2.0) == pytest.approx(0.0)


def test_yaw_enu_to_ned_is_an_involution():
    for yaw in (0.1, 1.0, -2.3, 3.0):
        assert tmap.yaw_enu_to_ned_for_flightgear(tmap.yaw_enu_to_ned_for_flightgear(yaw)) == pytest.approx(yaw)


def test_mps_to_fps_uses_documented_conversion_factor():
    assert tmap.mps_to_fps(0.3048) == pytest.approx(1.0)


def test_fg_net_fdm_struct_size_is_408_bytes():
    # The frozen/documented FG_NET_FDM_VERSION=24 struct size - see
    # telemetry_mapping.py's module docstring citation.
    assert tmap.FG_NET_FDM_STRUCT_SIZE == 408


def test_pack_fg_net_fdm_rejects_unknown_field_name():
    with pytest.raises(KeyError):
        tmap.pack_fg_net_fdm({"not_a_real_field": 1.0})


def test_pack_fg_net_fdm_defaults_missing_fields_to_zero():
    packed = tmap.pack_fg_net_fdm({"version": 24})
    unpacked = tmap._FG_NET_FDM_STRUCT.unpack(packed)
    assert unpacked[0] == 24            # version
    assert unpacked[1] == 0             # padding, never set - defaults to 0
    assert all(v == 0 for v in unpacked[2:])


def test_pack_fg_net_fdm_uses_big_endian_wire_order():
    packed = tmap.pack_fg_net_fdm({"version": 24})
    assert packed[0:4] == struct.pack(">I", 24)
    assert packed[0:4] != struct.pack("<I", 24)


def test_build_fg_net_fdm_fields_converts_lat_lon_to_radians():
    fields = tmap.build_fg_net_fdm_fields(
        latitude_deg=-35.363261, longitude_deg=149.165230, altitude_m=584.0, agl_m=1.72,
        roll_rad=0.0, pitch_rad=0.0, yaw_enu_rad=0.0,
        rollspeed_radps=0.0, pitchspeed_radps=0.0, yawspeed_radps=0.0,
        velocity_enu_mps=(0.0, 0.0, 0.0),
    )
    assert fields["latitude"] == pytest.approx(math.radians(-35.363261))
    assert fields["longitude"] == pytest.approx(math.radians(149.165230))
    assert fields["version"] == 24


def test_build_fg_net_fdm_fields_climb_rate_positive_when_ascending():
    # velocity_enu_mps up-component positive (ascending) must produce a
    # POSITIVE climb_rate - a sign-flip here would show the aircraft
    # descending while it's actually climbing.
    fields = tmap.build_fg_net_fdm_fields(
        latitude_deg=0.0, longitude_deg=0.0, altitude_m=0.0, agl_m=0.0,
        roll_rad=0.0, pitch_rad=0.0, yaw_enu_rad=0.0,
        rollspeed_radps=0.0, pitchspeed_radps=0.0, yawspeed_radps=0.0,
        velocity_enu_mps=(0.0, 0.0, 1.0),   # climbing at 1 m/s
    )
    assert fields["climb_rate"] > 0
    assert fields["v_down"] < 0   # NED down-velocity must be negative while climbing


def test_build_fg_net_fdm_fields_never_sets_vcas_or_engine_fields():
    """See telemetry_mapping.py's module docstring - vcas is ambiguous even
    within ArduPilot's own source, and engine data was never measured."""
    fields = tmap.build_fg_net_fdm_fields(
        latitude_deg=0.0, longitude_deg=0.0, altitude_m=0.0, agl_m=0.0,
        roll_rad=0.0, pitch_rad=0.0, yaw_enu_rad=0.0,
        rollspeed_radps=0.0, pitchspeed_radps=0.0, yawspeed_radps=0.0,
        velocity_enu_mps=(0.0, 0.0, 0.0),
    )
    assert "vcas" not in fields
    assert fields["num_engines"] == 0


# ==========================================================================
# 1. dry-run
# ==========================================================================

def test_dry_run_ok_for_valid_config():
    report = phase11.run_dry_run(_args(port=19400))
    assert report["ok"] is True


def test_dry_run_never_opens_a_socket():
    t0 = time.monotonic()
    report = phase11.run_dry_run(_args(port=19401))
    assert time.monotonic() - t0 < 1.0
    assert report["mode"] == "dry_run"


def test_dry_run_rejects_out_of_bounds_update_rate():
    report = phase11.run_dry_run(_args(port=19402, update_rate_hz=1000.0))
    assert report["ok"] is False


def test_dry_run_rejects_non_loopback_flightgear_endpoint():
    report = phase11.run_dry_run(_args(port=19403, flightgear_endpoint="8.8.8.8:5503"))
    assert report["ok"] is False
    assert report["flightgear_endpoint"]["ok"] is False


def test_dry_run_accepts_valid_flightgear_endpoint():
    report = phase11.run_dry_run(_args(port=19404, flightgear_endpoint="127.0.0.1:5503"))
    assert report["ok"] is True
    assert report["flightgear_endpoint"] == {"host": "127.0.0.1", "port": 5503, "ok": True}


def test_dry_run_rejects_non_loopback_mavlink_connection():
    args = _args(port=0)
    args.connection = "tcp:8.8.8.8:5760"
    report = phase11.run_dry_run(args)
    assert report["ok"] is False


# ==========================================================================
# 2. diagnose - no connection at all
# ==========================================================================

def test_diagnose_never_connects_even_to_unreachable_host():
    args = _args(port=0)
    args.connection = "tcp:127.0.0.1:1"   # nothing listens here
    t0 = time.monotonic()
    report = phase11.run_diagnose(args)
    assert time.monotonic() - t0 < 1.0
    assert report["mode"] == "diagnose"


def test_diagnose_reports_documented_protocol_facts():
    report = phase11.run_diagnose(_args(port=19405))
    assert report["protocol"]["version"] == 24
    assert report["protocol"]["struct_size_bytes"] == 408
    assert report["protocol"]["transport"] == "UDP"
    assert "read back" not in report["protocol"]["direction"] or "nothing is ever read back" in report["protocol"]["direction"]


def test_diagnose_validates_flightgear_endpoint_shape():
    ok = phase11.run_diagnose(_args(port=19406, flightgear_endpoint="127.0.0.1:5503"))
    assert ok["arguments"]["ok"] is True
    bad = phase11.run_diagnose(_args(port=19407, flightgear_endpoint="8.8.8.8:5503"))
    assert bad["arguments"]["ok"] is False


# ==========================================================================
# 3. attach - delegates to Phase 9, unmodified
# ==========================================================================

def test_attach_mode_delegates_to_phase9_telemetry_visualization(fg_vehicle):
    vehicle, port = fg_vehicle
    args = _args(port=port, attach=True, duration=0.5)
    import run_phase9_single_vehicle_visual as phase9
    report = phase9.run_telemetry_visualization(args)
    assert report["mode"] == "telemetry_visualization"
    assert vehicle.armed is False


# ==========================================================================
# 4. flightgear_view - gating before any attach
# ==========================================================================

def test_flightgear_view_requires_endpoint():
    report = phase11.run_flightgear_view(_args(port=0))
    assert report["attach"]["succeeded"] is False
    assert any("flightgear-endpoint" in f for f in report["remaining_failures"])


def test_flightgear_view_rejects_non_loopback_endpoint_before_attaching():
    report = phase11.run_flightgear_view(_args(port=0, flightgear_endpoint="8.8.8.8:5503"))
    assert report["attach"]["succeeded"] is False


def test_flightgear_view_rejects_out_of_bounds_update_rate_before_attaching():
    report = phase11.run_flightgear_view(
        _args(port=0, flightgear_endpoint="127.0.0.1:5503", update_rate_hz=999.0)
    )
    assert report["attach"]["succeeded"] is False


# ==========================================================================
# 5. flightgear_view - full happy path against a real local UDP "FlightGear"
# ==========================================================================

def test_flightgear_view_sends_correctly_sized_versioned_frames(fg_vehicle, tmp_path, monkeypatch):
    vehicle, port = fg_vehicle
    monkeypatch.setattr(phase11, "OUT_DIR", str(tmp_path))
    monkeypatch.setattr(phase11, "CSV_PATH", str(tmp_path / "telemetry.csv"))

    fake_fg = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    fake_fg.bind(("127.0.0.1", 0))
    fake_fg.settimeout(2.0)
    fg_port = fake_fg.getsockname()[1]

    try:
        args = _args(port=port, flightgear_view=True, flightgear_endpoint=f"127.0.0.1:{fg_port}",
                     duration=1.0, update_rate_hz=10.0)
        report = phase11.run_flightgear_view(args)

        assert report["attach"]["succeeded"] is True
        assert report["heartbeat"]["received"] is True
        assert report["remaining_failures"] == []
        assert report["fg_sent_count"] > 0
        assert vehicle.armed is False

        data, _addr = fake_fg.recvfrom(4096)
        assert len(data) == tmap.FG_NET_FDM_STRUCT_SIZE
        version = struct.unpack(">I", data[0:4])[0]
        assert version == 24
    finally:
        fake_fg.close()


def test_flightgear_view_writes_csv_with_expected_columns(fg_vehicle, tmp_path, monkeypatch):
    vehicle, port = fg_vehicle
    csv_path = tmp_path / "telemetry.csv"
    monkeypatch.setattr(phase11, "OUT_DIR", str(tmp_path))
    monkeypatch.setattr(phase11, "CSV_PATH", str(csv_path))

    fake_fg = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    fake_fg.bind(("127.0.0.1", 0))
    fg_port = fake_fg.getsockname()[1]
    try:
        args = _args(port=port, flightgear_view=True, flightgear_endpoint=f"127.0.0.1:{fg_port}",
                     duration=1.0, update_rate_hz=10.0)
        phase11.run_flightgear_view(args)
    finally:
        fake_fg.close()

    assert csv_path.exists()
    with open(csv_path, newline="") as f:
        rows = list(csv.DictReader(f))
    assert len(rows) > 0
    assert set(phase11._CSV_FIELDS) == set(rows[0].keys())


def test_flightgear_view_never_sends_anything_but_message_interval_request(fg_vehicle, tmp_path, monkeypatch):
    vehicle, port = fg_vehicle
    monkeypatch.setattr(phase11, "OUT_DIR", str(tmp_path))
    monkeypatch.setattr(phase11, "CSV_PATH", str(tmp_path / "telemetry.csv"))
    fake_fg = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    fake_fg.bind(("127.0.0.1", 0))
    fg_port = fake_fg.getsockname()[1]
    try:
        args = _args(port=port, flightgear_view=True, flightgear_endpoint=f"127.0.0.1:{fg_port}",
                     duration=0.5, update_rate_hz=10.0)
        phase11.run_flightgear_view(args)
    finally:
        fake_fg.close()

    assert vehicle.armed is False
    forbidden = {
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        mavutil.mavlink.MAV_CMD_NAV_LAND, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    }
    seen_commands = {c.command for c in vehicle.received_commands}
    assert not (seen_commands & forbidden)
    assert seen_commands <= {mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL}


def test_flightgear_view_drops_samples_when_no_global_position_available(tmp_path, monkeypatch):
    """A plain DummyArduPilotVehicle (never overridden to send
    GLOBAL_POSITION_INT) must never cause a fabricated lat/lon=0 frame to
    be sent - every sample must be honestly dropped instead."""
    port = _next_port()
    v = DummyArduPilotVehicle(port=port, system_id=1, component_id=1)
    v.start()
    monkeypatch.setattr(phase11, "OUT_DIR", str(tmp_path))
    monkeypatch.setattr(phase11, "CSV_PATH", str(tmp_path / "telemetry.csv"))
    fake_fg = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    fake_fg.bind(("127.0.0.1", 0))
    fake_fg.settimeout(0.2)
    fg_port = fake_fg.getsockname()[1]
    try:
        args = _args(port=port, flightgear_view=True, flightgear_endpoint=f"127.0.0.1:{fg_port}",
                     duration=0.5, update_rate_hz=10.0)
        report = phase11.run_flightgear_view(args)
        assert report["fg_sent_count"] == 0
        assert report["dropped_count"] > 0
        with pytest.raises(socket.timeout):
            fake_fg.recvfrom(4096)
    finally:
        fake_fg.close()
        v.stop()


# ==========================================================================
# 6. replay - offline only
# ==========================================================================

def test_replay_reprocesses_a_previously_saved_csv(fg_vehicle, tmp_path, monkeypatch):
    vehicle, port = fg_vehicle
    csv_path = tmp_path / "telemetry.csv"
    monkeypatch.setattr(phase11, "OUT_DIR", str(tmp_path))
    monkeypatch.setattr(phase11, "CSV_PATH", str(csv_path))
    fake_fg = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    fake_fg.bind(("127.0.0.1", 0))
    fg_port = fake_fg.getsockname()[1]
    try:
        args = _args(port=port, flightgear_view=True, flightgear_endpoint=f"127.0.0.1:{fg_port}",
                     duration=1.0, update_rate_hz=10.0)
        live_report = phase11.run_flightgear_view(args)
    finally:
        fake_fg.close()

    replay_report = phase11.run_replay(_args(port=0, replay=str(csv_path)))
    assert replay_report["mode"] == "replay"
    assert replay_report["remaining_failures"] == []
    assert replay_report["row_count"] > 0
    assert replay_report["sent_row_count"] == live_report["fg_sent_count"]
    assert replay_report["max_altitude_m"] is not None


def test_replay_never_opens_any_socket():
    t0 = time.monotonic()
    report = phase11.run_replay(_args(port=0, replay="does/not/exist.csv"))
    assert time.monotonic() - t0 < 1.0
    assert report["remaining_failures"] != []


# ==========================================================================
# 7. mode separation + regression checks
# ==========================================================================

def test_modes_produce_disjoint_report_shapes():
    dry = phase11.run_dry_run(_args(port=19408))
    diag = phase11.run_diagnose(_args(port=19409))
    assert "protocol" not in dry
    assert "protocol" in diag
    assert dry["mode"] != diag["mode"]


def test_fake_sitl_transport_still_importable_and_functional():
    from swarm_sim.sitl.fake_transport import FakeSITLTransport
    t = FakeSITLTransport(vehicle_ids=("d",))
    assert t.start().success


def test_phase10_module_unaffected():
    import run_phase10_sitl_flight_test as phase10
    args = phase10.build_parser().parse_args([])
    assert args.sitl_only is False
    assert args.allow_sitl_arm_test is False
