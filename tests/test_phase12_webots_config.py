"""Behavioral tests for scripts/run_phase12_webots_smoke_test.py - see
docs/PHASE12_WEBOTS_SITL.md. Covers the required diagnostic/error-handling
matrix: Webots not installed, missing official files, bad addresses,
connection timeout, and bounded duration - without needing Webots or a
real SITL installed to run this test suite itself.
"""
import os
import socket
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"))

import run_phase12_webots_smoke_test as phase12  # noqa: E402


def _make_args(**overrides):
    args = phase12.build_parser().parse_args([])
    for key, value in overrides.items():
        setattr(args, key, value)
    return args


# --------------------------------------------------------------------------
# validate_local_or_private_address
# --------------------------------------------------------------------------

@pytest.mark.parametrize("addr", ["127.0.0.1", "192.168.1.5", "172.24.220.98", "10.0.0.5"])
def test_validate_address_accepts_loopback_and_private(addr):
    assert phase12.validate_local_or_private_address(addr) == addr


@pytest.mark.parametrize("addr", ["8.8.8.8", "1.1.1.1", "93.184.216.34"])
def test_validate_address_rejects_public(addr):
    with pytest.raises(phase12.Phase12ConfigError, match="not a local/private address"):
        phase12.validate_local_or_private_address(addr)


def test_validate_address_rejects_garbage():
    with pytest.raises(phase12.Phase12ConfigError, match="not a valid IP address"):
        phase12.validate_local_or_private_address("not-an-ip")


# --------------------------------------------------------------------------
# dry-run: hardware/file-presence diagnostic - never opens a socket
# --------------------------------------------------------------------------

def test_dry_run_reports_webots_not_installed_when_absent(monkeypatch):
    monkeypatch.setattr(phase12, "_detect_webots_installation",
                         lambda: {"found": False, "path": None, "method": "not_found"})
    report = phase12.run_dry_run(_make_args())
    assert report["ok"] is False
    assert "Webots not installed / not live verified" in report["remaining_failures"]


def test_dry_run_never_claims_success_from_installed_path_alone(monkeypatch):
    """Installed-but-nothing-else-verified must not read as overall success -
    Webots version/OpenGL are honestly reported as not determined."""
    monkeypatch.setattr(phase12, "_detect_webots_installation",
                         lambda: {"found": True, "path": "/usr/bin/webots", "method": "PATH"})
    report = phase12.run_dry_run(_make_args())
    assert report["webots_version"] is None
    assert report["opengl_version"] is None


def test_dry_run_detects_official_example_files(tmp_path):
    root = tmp_path / "ardupilot"
    world_dir = root / "libraries" / "SITL" / "examples" / "Webots_Python" / "worlds"
    params_dir = root / "libraries" / "SITL" / "examples" / "Webots_Python" / "params"
    controller_dir = (root / "libraries" / "SITL" / "examples" / "Webots_Python" / "controllers"
                       / "ardupilot_vehicle_controller")
    for d in (world_dir, params_dir, controller_dir):
        d.mkdir(parents=True)
    (world_dir / "iris.wbt").write_text("# fixture")
    (params_dir / "iris.parm").write_text("# fixture")
    (controller_dir / "ardupilot_vehicle_controller.py").write_text("# fixture")

    report = phase12.run_dry_run(_make_args(ardupilot_root=str(root)))
    assert report["world_exists"] is True
    assert report["params_exists"] is True
    assert report["controller_exists"] is True


def test_dry_run_reports_missing_official_files_honestly(tmp_path):
    root = tmp_path / "empty_checkout"
    root.mkdir()
    report = phase12.run_dry_run(_make_args(ardupilot_root=str(root)))
    assert report["world_exists"] is False
    assert report["params_exists"] is False
    assert report["controller_exists"] is False
    assert any("not found" in f for f in report["remaining_failures"])


def test_dry_run_rejects_bad_sitl_or_sim_address():
    report = phase12.run_dry_run(_make_args(sitl_address="8.8.8.8"))
    assert report["ok"] is False
    assert any("--sitl-address invalid" in f for f in report["remaining_failures"])

    report = phase12.run_dry_run(_make_args(sim_address="not-an-ip"))
    assert report["ok"] is False
    assert any("--sim-address invalid" in f for f in report["remaining_failures"])


def test_dry_run_rejects_non_smoke_test_profile():
    report = phase12.run_dry_run(_make_args(profile=phase12.PROFILE_FLIGHT_TEST))
    assert report["ok"] is False
    assert any("not implemented" in f for f in report["remaining_failures"])


def test_dry_run_bounds_duration():
    report = phase12.run_dry_run(_make_args(duration=99999.0))
    assert report["ok"] is False
    report = phase12.run_dry_run(_make_args(duration=0.0))
    assert report["ok"] is False


def test_dry_run_never_opens_a_socket(monkeypatch):
    def _fail(*a, **k):
        raise AssertionError("dry-run must never open a socket")
    monkeypatch.setattr(socket, "socket", _fail)
    try:
        phase12.run_dry_run(_make_args())
    finally:
        monkeypatch.undo()


# --------------------------------------------------------------------------
# --run: handles Webots not installed / missing files / connection timeout
# --------------------------------------------------------------------------

def test_run_reports_webots_not_installed(monkeypatch):
    monkeypatch.setattr(phase12, "_detect_webots_installation",
                         lambda: {"found": False, "path": None, "method": "not_found"})
    report = phase12.run_smoke_test(_make_args())
    assert report["ok"] is False
    assert report["webots_status"] == "not_installed"
    assert "Webots not installed / not live verified" in report["remaining_failures"]


def test_run_rejects_flight_test_profile_without_attempting_anything(monkeypatch):
    def _fail(*a, **k):
        raise AssertionError("must not attempt detection for an unimplemented profile")
    monkeypatch.setattr(phase12, "_detect_webots_installation", _fail)
    report = phase12.run_smoke_test(_make_args(profile=phase12.PROFILE_FLIGHT_TEST))
    assert report["ok"] is False
    assert any("not implemented" in f for f in report["remaining_failures"])


def test_run_detects_controller_port_not_bound_when_nothing_is_listening(monkeypatch):
    monkeypatch.setattr(phase12, "_detect_webots_installation",
                         lambda: {"found": True, "path": "/usr/bin/webots", "method": "PATH"})
    report = phase12.run_smoke_test(_make_args(connection="tcp:127.0.0.1:1"))
    assert report["controller_port_bound"] is False
    assert any("does not appear to be running" in f for f in report["remaining_failures"])


def test_run_handles_mavlink_connection_timeout_cleanly(monkeypatch):
    """No SITL is listening on this port at all - must fail cleanly with a
    descriptive reason, never hang or raise uncaught."""
    monkeypatch.setattr(phase12, "_detect_webots_installation",
                         lambda: {"found": True, "path": "/usr/bin/webots", "method": "PATH"})
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    free_port = sock.getsockname()[1]
    sock.close()

    report = phase12.run_smoke_test(_make_args(
        connection=f"tcp:127.0.0.1:{free_port}", startup_timeout=0.5,
    ))
    assert report["ok"] is False
    assert report["attach"]["succeeded"] is False
    assert report["remaining_failures"]


def test_run_rejects_bad_network_addresses_before_touching_webots(monkeypatch):
    def _fail(*a, **k):
        raise AssertionError("must validate addresses before any detection/attach attempt")
    monkeypatch.setattr(phase12, "_detect_webots_installation", _fail)
    report = phase12.run_smoke_test(_make_args(sitl_address="8.8.8.8"))
    assert report["ok"] is False


def test_run_bounds_duration_before_any_attach_attempt(monkeypatch):
    def _fail(*a, **k):
        raise AssertionError("must validate duration before any detection/attach attempt")
    monkeypatch.setattr(phase12, "_detect_webots_installation", _fail)
    report = phase12.run_smoke_test(_make_args(duration=99999.0))
    assert report["ok"] is False


# --------------------------------------------------------------------------
# CLI defaults
# --------------------------------------------------------------------------

def test_cli_defaults_never_select_a_live_run():
    args = phase12.build_parser().parse_args([])
    assert args.dry_run is False
    assert args.run is False
    assert args.ardupilot_root in (None, os.environ.get("ARDUPILOT_ROOT"))


def test_webots_controller_port_matches_official_default_scheme():
    assert phase12.WEBOTS_CONTROLLER_BASE_PORT == 9002
    assert phase12._webots_controller_port(0) == 9002
    assert phase12._webots_controller_port(1) == 9012
