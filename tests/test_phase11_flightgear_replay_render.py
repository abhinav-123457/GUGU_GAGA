"""Tests for scripts/run_phase11_flightgear_replay_render.py - see
docs/PHASE11_FLIGHTGEAR_REPLAY_RENDER.md. Proves the replay renderer:
reaches the actual recorded 1.720716118812561 m peak, never requires
ArduPilot SITL, sends frames labeled REPLAY, and sends no MAVLink command
of any kind (it has no MAVLink capability at all - see the companion
architecture test file for the structural proof).
"""
import csv
import json
import os
import socket
import struct
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"))

import run_phase11_flightgear_replay_render as replay_module  # noqa: E402

REAL_SOURCE_RESULT = replay_module.DEFAULT_SOURCE_RESULT
REAL_RECORDED_PEAK_ALTITUDE_M = 1.720716118812561


def _make_args(**overrides):
    args = replay_module.build_parser().parse_args([])
    for key, value in overrides.items():
        setattr(args, key, value)
    return args


def _free_loopback_udp_port() -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()
    return port


# --------------------------------------------------------------------------
# The real recorded source file
# --------------------------------------------------------------------------

def test_default_source_is_the_tracked_real_phase10_result():
    assert os.path.isfile(REAL_SOURCE_RESULT), (
        "default --source-result must point at the tracked, real Phase 10 "
        "flight-test result, not a gitignored/ephemeral path"
    )
    with open(REAL_SOURCE_RESULT) as f:
        result = json.load(f)
    assert result["flight_actually_happened"] is True
    assert result["takeoff"]["max_altitude_observed_m"] == REAL_RECORDED_PEAK_ALTITUDE_M


def test_build_replay_segments_from_real_source_reaches_recorded_peak():
    events, peak = replay_module._load_recorded_events(REAL_SOURCE_RESULT)
    assert peak == REAL_RECORDED_PEAK_ALTITUDE_M
    segments = replay_module._build_replay_segments(events, peak)
    max_relative_altitude_m = max(s["relative_altitude_m"] for s in segments)
    assert max_relative_altitude_m == REAL_RECORDED_PEAK_ALTITUDE_M

    peak_segments = [s for s in segments if s["relative_altitude_m"] == REAL_RECORDED_PEAK_ALTITUDE_M]
    assert any(s["altitude_source"] == "measured_takeoff_confirmed_altitude_m" for s in peak_segments), (
        "the real recorded peak must come from a 'measured' segment, not an inferred/held one"
    )

    ground_segments = [s for s in segments if s["event"] in replay_module._GROUND_EVENTS_BEFORE_LIFTOFF]
    assert all(s["relative_altitude_m"] == 0.0 for s in ground_segments)
    assert all(s["duration_s"] >= 0.0 for s in segments)


def test_load_recorded_events_rejects_result_without_flight_actually_happened(tmp_path):
    fixture = tmp_path / "not_a_real_flight.json"
    fixture.write_text(json.dumps({
        "flight_actually_happened": False,
        "events": [{"monotonic_s": 0.0, "event": "attached"}],
        "takeoff": {"max_altitude_observed_m": 99.0},
    }))
    with pytest.raises(ValueError, match="does not record a completed real flight"):
        replay_module._load_recorded_events(str(fixture))


def test_load_recorded_events_rejects_missing_peak_altitude(tmp_path):
    fixture = tmp_path / "no_peak.json"
    fixture.write_text(json.dumps({
        "flight_actually_happened": True,
        "events": [{"monotonic_s": 0.0, "event": "attached"}],
        "takeoff": {},
    }))
    with pytest.raises(ValueError, match="max_altitude_observed_m"):
        replay_module._load_recorded_events(str(fixture))


def test_build_replay_segments_rejects_inconsistent_recorded_altitude():
    events = [
        {"monotonic_s": 0.0, "event": "armed_confirmed"},
        {"monotonic_s": 1.0, "event": "takeoff_confirmed", "altitude_m": 5.0},
    ]
    with pytest.raises(ValueError, match="does not match"):
        replay_module._build_replay_segments(events, peak_altitude_m=6.0)


# --------------------------------------------------------------------------
# dry-run - never opens a socket, works without ArduPilot/FlightGear running
# --------------------------------------------------------------------------

def test_dry_run_against_real_source_never_opens_a_socket_and_succeeds():
    args = _make_args(dry_run=True, source_result=REAL_SOURCE_RESULT)
    report = replay_module.run_replay_render(args)
    assert report["ok"] is True
    assert report["frame_count"] == 0
    assert report["flightgear_endpoint"] is None
    assert report["recorded_peak_altitude_m"] == REAL_RECORDED_PEAK_ALTITUDE_M


def test_dry_run_rejects_missing_source_file(tmp_path):
    args = _make_args(dry_run=True, source_result=str(tmp_path / "does_not_exist.json"))
    report = replay_module.run_replay_render(args)
    assert report["ok"] is False
    assert report["remaining_failures"]


def test_default_cli_args_never_select_a_live_send_without_dry_run_or_endpoint():
    args = replay_module.build_parser().parse_args([])
    assert args.dry_run is False
    assert args.flightgear_endpoint is None
    # Running with neither --dry-run nor --flightgear-endpoint must fail
    # cleanly and never attempt to open a socket.
    report = replay_module.run_replay_render(args)
    assert report["ok"] is False
    assert "flightgear-endpoint" in report["remaining_failures"][0]


def test_update_rate_and_speedup_are_bounds_checked():
    args = _make_args(dry_run=True, update_rate_hz=999.0)
    report = replay_module.run_replay_render(args)
    assert report["ok"] is False

    args = _make_args(dry_run=True, speedup=0.0)
    report = replay_module.run_replay_render(args)
    assert report["ok"] is False

    args = _make_args(dry_run=True, speedup=9999.0)
    report = replay_module.run_replay_render(args)
    assert report["ok"] is False


# --------------------------------------------------------------------------
# Real send: a local UDP socket stands in for FlightGear (same pattern as
# tests/test_phase11_flightgear_viewer.py's own live-send tests) - proves
# real 408-byte v24 frames are sent and the recorded peak is actually
# reached in the wire data, not just claimed in the JSON report.
# --------------------------------------------------------------------------

def test_replay_render_sends_frames_reaching_the_recorded_peak_over_real_udp():
    port = _free_loopback_udp_port()
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("127.0.0.1", port))
    receiver.settimeout(5.0)

    args = _make_args(
        source_result=REAL_SOURCE_RESULT, flightgear_endpoint=f"127.0.0.1:{port}",
        update_rate_hz=10.0, speedup=50.0,
    )
    report = replay_module.run_replay_render(args)

    assert report["ok"] is True
    assert report["view_label"] == "REPLAY"
    assert report["peak_reached"] is True
    assert report["max_relative_altitude_m"] == REAL_RECORDED_PEAK_ALTITUDE_M
    assert report["frame_count"] > 0
    assert report["fg_sent_count"] == report["frame_count"]
    assert report["fg_error_count"] == 0

    received = []
    try:
        while True:
            data, _addr = receiver.recvfrom(4096)
            received.append(data)
    except socket.timeout:
        pass
    finally:
        receiver.close()

    assert len(received) == report["frame_count"]
    max_altitude_on_wire = 0.0
    for frame in received:
        assert len(frame) == 408
        version, _padding, _lon, _lat, altitude = struct.unpack_from(">IIddd", frame, 0)
        assert version == 24
        max_altitude_on_wire = max(max_altitude_on_wire, altitude)

    assert max_altitude_on_wire == pytest.approx(
        replay_module.HOME_ALTITUDE_M + REAL_RECORDED_PEAK_ALTITUDE_M, abs=1e-6
    )


def test_replay_csv_labels_every_row_replay_and_records_peak_row():
    port = _free_loopback_udp_port()
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("127.0.0.1", port))

    args = _make_args(
        source_result=REAL_SOURCE_RESULT, flightgear_endpoint=f"127.0.0.1:{port}",
        update_rate_hz=10.0, speedup=50.0,
    )
    report = replay_module.run_replay_render(args)
    receiver.close()

    assert os.path.isfile(report["csv_path"])
    with open(report["csv_path"], newline="") as f:
        rows = list(csv.DictReader(f))
    assert rows, "replay must write at least one CSV row"
    assert all(row["view_label"] == "REPLAY" for row in rows)
    assert all(row["view_label"] != "LIVE" for row in rows)

    peak_rows = [row for row in rows if row["altitude_source"] == "measured_takeoff_confirmed_altitude_m"]
    assert peak_rows, "at least one row must carry the measured (not inferred/held) peak"
    assert float(peak_rows[0]["relative_altitude_m"]) == REAL_RECORDED_PEAK_ALTITUDE_M


def test_replay_never_requires_a_mavlink_port_to_be_listening():
    """No process is listening on tcp:127.0.0.1:5760 (or any MAVLink port)
    in this test, at all - if this script tried to open a MAVLink
    connection it would hang or raise. It succeeds anyway, proving it
    never needed one."""
    port = _free_loopback_udp_port()
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("127.0.0.1", port))
    try:
        args = _make_args(
            source_result=REAL_SOURCE_RESULT, flightgear_endpoint=f"127.0.0.1:{port}",
            update_rate_hz=10.0, speedup=50.0,
        )
        report = replay_module.run_replay_render(args)
        assert report["ok"] is True
    finally:
        receiver.close()


def test_flightgear_endpoint_still_rejects_non_loopback():
    args = _make_args(source_result=REAL_SOURCE_RESULT, flightgear_endpoint="8.8.8.8:5503")
    report = replay_module.run_replay_render(args)
    assert report["ok"] is False
    assert any("loopback" in failure or "invalid" in failure for failure in report["remaining_failures"])
