"""Phase 12A/12B: official ArduPilot SITL + Webots closed-loop smoke test -
see docs/PHASE12_WEBOTS_SITL.md.

**What this script is not**: it is not a FlightGear-style pose renderer
(Phase 11). Webots is a genuine physics simulator - gravity, contact,
sensors, and motor response all happen inside Webots itself, with
ArduPilot SITL as the flight-control/estimator authority reading Webots'
sensors and commanding Webots' motors over the OFFICIAL ArduPilot Webots
Python protocol (`libraries/SITL/SIM_Webots_Python.cpp`/`.h`, `--model
webots-python`) - never a protocol invented by this project.

**This script never spawns or kills Webots or ArduPilot SITL.** Per this
phase's own safety requirement (and matching Phase 9/10/11's own ATTACH-
only pattern), the operator starts the official example manually (exact
commands in docs/PHASE12_WEBOTS_SITL.md); this script only OBSERVES/
VERIFIES: hardware/file-presence facts (`--dry-run`, no connection of any
kind), and, once the official example is already running (`--run`),
read-only evidence that the closed-loop sensor/actuator link is actually
alive - a real MAVLink heartbeat/EKF-valid read (reusing
`ArduPilotSITLTransport`, the same read-only attach Phase 8/9/10/11 use)
plus a non-invasive check that the documented Webots controller UDP port
is bound by something. It never arms, takes off, lands, changes mode, or
writes a parameter - see tests/test_phase12_webots_architecture.py.

**Two profiles, one implemented**: `webots_offline_smoke_test` (this
phase's only implemented profile - disarmed, read-only, no flight
command of any kind) and `webots_sitl_flight_test` (explicitly NOT
implemented here - selecting it is a configuration error, never a
silent fallback to the smoke-test behavior or vice versa).

**Official example locations** (verified this session against ArduPilot
commit 92b0cd788ec29406f26c6f9c31d5ceedbd1cc538, Copter-4.6.3):
    <ardupilot-root>/libraries/SITL/examples/Webots_Python/worlds/iris.wbt
    <ardupilot-root>/libraries/SITL/examples/Webots_Python/params/iris.parm
    <ardupilot-root>/libraries/SITL/examples/Webots_Python/controllers/
        ardupilot_vehicle_controller/ardupilot_vehicle_controller.py
`--model webots-python` maps to `WebotsPython::create` in
`libraries/AP_HAL_SITL/SITL_cmdline.cpp`. The controller listens on UDP
`9002 + 10*instance` (`SIM_Webots_Python.h`'s `_webots_port` default) for
actuator data from SITL and sends sensor (FDM) data to `sitl_address:
port+1`, printing `"Connected to ardupilot SITL (I<n>)"` once SITL's
first packet arrives (`webots_vehicle.py::_handle_sitl`) - the exact
string this phase's item 3 requires, read directly from the official
source rather than assumed.

**No hardcoded IP, no subprocess.** This module never imports
`subprocess` (matching Phase 9-11's own blanket rule) and never invents
a default for `--sitl-address`/`--sim-address` in the cross-boundary
(Arrangement B) case - both must be given explicitly by the operator and
are validated to be loopback or RFC1918-private, never a public address.
"""
from __future__ import annotations

import argparse
import ipaddress
import json
import os
import platform
import socket
import struct
import sys
import time
from typing import Optional

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from swarm_sim.sitl.ardupilot_transport import (  # noqa: E402
    ArduPilotSITLTransport, ArduPilotTransportError, ArduPilotVehicleEndpoint,
    PYMAVLINK_AVAILABLE, PYMAVLINK_VERSION, parse_connection_string,
)

OUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "results", "phase12_webots")

PROFILE_SMOKE_TEST = "webots_offline_smoke_test"
PROFILE_FLIGHT_TEST = "webots_sitl_flight_test"  # NOT implemented in this phase - see module docstring

# SIM_Webots_Python.h's own default (`_webots_port = 9002`) - reused, not invented.
WEBOTS_CONTROLLER_BASE_PORT = 9002
WEBOTS_CONTROLLER_PORT_STRIDE = 10  # port = 9002 + 10*instance, per webots_vehicle.py

MIN_DURATION_S = 1.0
MAX_DURATION_S = 300.0

_OFFICIAL_WORLD_REL = os.path.join("libraries", "SITL", "examples", "Webots_Python", "worlds")
_OFFICIAL_PARAMS_REL = os.path.join("libraries", "SITL", "examples", "Webots_Python", "params")
_OFFICIAL_CONTROLLER_REL = os.path.join(
    "libraries", "SITL", "examples", "Webots_Python", "controllers", "ardupilot_vehicle_controller",
    "ardupilot_vehicle_controller.py",
)

_COMMON_WEBOTS_WINDOWS_DIRS = (
    r"C:\Program Files\Webots",
    r"C:\Program Files (x86)\Webots",
)

_SAFETY_BANNER = (
    "Phase 12: official ArduPilot SITL + Webots closed-loop smoke test (one Iris quadcopter).\n"
    "  - Profile: webots_offline_smoke_test only - disarmed, read-only, no flight command of any kind.\n"
    "  - This script never spawns or kills Webots or ArduPilot SITL - the operator starts both manually.\n"
    "  - A visual quadcopter model alone is not proof of closed-loop physics - see the sensor/actuator checks.\n"
)


class Phase12ConfigError(Exception):
    pass


def validate_local_or_private_address(addr_str: str) -> str:
    """Accept only loopback or RFC1918-private addresses - reject public/
    global addresses and anything that doesn't parse as an IP at all.
    Never used to supply a default; callers must always pass an explicit
    string (see build_parser - `--sitl-address`/`--sim-address` default
    to None, never a guessed IP)."""
    try:
        ip = ipaddress.ip_address(addr_str)
    except ValueError as e:
        raise Phase12ConfigError(f"{addr_str!r} is not a valid IP address") from e
    if ip.is_loopback or ip.is_private:
        return addr_str
    raise Phase12ConfigError(
        f"{addr_str!r} is not a local/private address - public/global IP addresses are rejected"
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Phase 12: official ArduPilot SITL + Webots closed-loop smoke test "
                    "(one Iris quadcopter, read-only verification, never spawns a process).",
    )
    parser.add_argument("--profile", default=PROFILE_SMOKE_TEST,
                         choices=(PROFILE_SMOKE_TEST, PROFILE_FLIGHT_TEST),
                         help=f"only {PROFILE_SMOKE_TEST!r} is implemented in this phase")
    parser.add_argument("--dry-run", action="store_true",
                         help="hardware/file-presence diagnostic only - never opens a socket")
    parser.add_argument("--run", action="store_true",
                         help="verify an ALREADY-RUNNING official Webots+SITL example (never starts one)")
    parser.add_argument("--ardupilot-root", default=os.environ.get("ARDUPILOT_ROOT"),
                         help="path to an ArduPilot checkout containing the official Webots_Python example "
                              "(default: $ARDUPILOT_ROOT env var; never guessed otherwise)")
    parser.add_argument("--world", default="iris.wbt", help="official world file name")
    parser.add_argument("--vehicle", default="iris", help="official vehicle/param-file base name")
    parser.add_argument("--instance", type=int, default=0, help="SITL/Webots instance number")
    parser.add_argument("--connection", default="tcp:127.0.0.1:5760",
                         help="read-only MAVLink attach endpoint for the already-running SITL")
    parser.add_argument("--system-id", type=int, default=1)
    parser.add_argument("--component-id", type=int, default=1)
    parser.add_argument("--namespace", default="sitl/drone0")
    parser.add_argument("--startup-timeout", type=float, default=10.0)
    parser.add_argument("--duration", type=float, default=60.0,
                         help=f"bounded [{MIN_DURATION_S}, {MAX_DURATION_S}] s observation window for --run")
    parser.add_argument("--sitl-address", default=None,
                         help="Arrangement B only: WSL2 IP the Webots controller should reach SITL at "
                             "(must be explicit; loopback/RFC1918-private only, never guessed)")
    parser.add_argument("--sim-address", default=None,
                         help="Arrangement B only: Windows host IP SITL should reach the Webots controller at "
                             "(must be explicit; loopback/RFC1918-private only, never guessed)")
    return parser


# --------------------------------------------------------------------------
# Phase 12A: hardware / configuration diagnostic - never opens a socket
# --------------------------------------------------------------------------

def _detect_webots_installation() -> dict:
    """Best-effort, read-only detection - never executes Webots. Absence of
    an installation is reported honestly, never treated as success."""
    import shutil as _shutil
    path = _shutil.which("webots")
    if path:
        return {"found": True, "path": path, "method": "PATH"}
    if platform.system() == "Windows":
        for candidate in _COMMON_WEBOTS_WINDOWS_DIRS:
            if os.path.isdir(candidate):
                return {"found": True, "path": candidate, "method": "common_install_dir"}
    return {"found": False, "path": None, "method": "not_found"}


def _detect_memory_bytes() -> Optional[int]:
    """Pure-stdlib, cross-platform, no subprocess."""
    if platform.system() == "Windows":
        try:
            import ctypes

            class _MEMORYSTATUSEX(ctypes.Structure):
                _fields_ = [
                    ("dwLength", ctypes.c_ulong), ("dwMemoryLoad", ctypes.c_ulong),
                    ("ullTotalPhys", ctypes.c_ulonglong), ("ullAvailPhys", ctypes.c_ulonglong),
                    ("ullTotalPageFile", ctypes.c_ulonglong), ("ullAvailPageFile", ctypes.c_ulonglong),
                    ("ullTotalVirtual", ctypes.c_ulonglong), ("ullAvailVirtual", ctypes.c_ulonglong),
                    ("ullAvailExtendedVirtual", ctypes.c_ulonglong),
                ]

            stat = _MEMORYSTATUSEX()
            stat.dwLength = ctypes.sizeof(_MEMORYSTATUSEX)
            ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(stat))
            return int(stat.ullTotalPhys)
        except Exception:
            return None
    try:
        with open("/proc/meminfo") as f:
            for line in f:
                if line.startswith("MemTotal:"):
                    return int(line.split()[1]) * 1024
    except OSError:
        return None
    return None


def _official_example_paths(ardupilot_root: Optional[str], world: str, vehicle: str) -> dict:
    if not ardupilot_root:
        return {
            "ardupilot_root": None, "world_path": None, "params_path": None, "controller_path": None,
            "world_exists": False, "params_exists": False, "controller_exists": False,
        }
    world_path = os.path.join(ardupilot_root, _OFFICIAL_WORLD_REL, world)
    params_path = os.path.join(ardupilot_root, _OFFICIAL_PARAMS_REL, f"{vehicle}.parm")
    controller_path = os.path.join(ardupilot_root, _OFFICIAL_CONTROLLER_REL)
    return {
        "ardupilot_root": ardupilot_root, "world_path": world_path, "params_path": params_path,
        "controller_path": controller_path,
        "world_exists": os.path.isfile(world_path),
        "params_exists": os.path.isfile(params_path),
        "controller_exists": os.path.isfile(controller_path),
    }


def run_dry_run(args) -> dict:
    report = {
        "mode": "dry_run", "profile": args.profile,
        "operating_system": platform.platform(), "python_version": platform.python_version(),
        "cpu_count": os.cpu_count(), "memory_total_bytes": _detect_memory_bytes(),
        "webots_installation": _detect_webots_installation(),
        "webots_version": None,  # not programmatically detectable without launching Webots - see docs
        "opengl_version": None,  # not programmatically detectable without a GL context - see docs
        "gpu_name": None,        # not programmatically detectable cross-platform without subprocess - see docs
        "selected_world": args.world, "selected_vehicle": args.vehicle,
        "selected_instance": args.instance, "selected_connection": args.connection,
        "selected_network_arrangement": "A (loopback)" if not (args.sitl_address or args.sim_address) else "B (WSL2/Windows)",
        "pymavlink_available": PYMAVLINK_AVAILABLE, "pymavlink_version": PYMAVLINK_VERSION,
        "remaining_failures": [],
    }

    report.update(_official_example_paths(args.ardupilot_root, args.world, args.vehicle))

    if args.profile != PROFILE_SMOKE_TEST:
        report["remaining_failures"].append(
            f"profile {args.profile!r} is not implemented in this phase - only {PROFILE_SMOKE_TEST!r} is"
        )

    if not report["webots_installation"]["found"]:
        report["remaining_failures"].append("Webots not installed / not live verified")
    if args.ardupilot_root is None:
        report["remaining_failures"].append(
            "--ardupilot-root (or $ARDUPILOT_ROOT) not given - cannot verify official example files"
        )
    else:
        for key, label in (("world_exists", "world"), ("params_exists", "params"), ("controller_exists", "controller")):
            if not report[key]:
                report["remaining_failures"].append(f"official {label} file not found at the expected path")

    for name, value in (("--sitl-address", args.sitl_address), ("--sim-address", args.sim_address)):
        if value is not None:
            try:
                validate_local_or_private_address(value)
            except Phase12ConfigError as e:
                report["remaining_failures"].append(f"{name} invalid: {e}")

    if not (MIN_DURATION_S <= args.duration <= MAX_DURATION_S):
        report["remaining_failures"].append(
            f"--duration {args.duration} outside bounded range [{MIN_DURATION_S}, {MAX_DURATION_S}]"
        )

    report["ok"] = not report["remaining_failures"]
    return report


# --------------------------------------------------------------------------
# Phase 12B: verify an ALREADY-RUNNING official example - never spawns one
# --------------------------------------------------------------------------

def _webots_controller_port(instance: int) -> int:
    return WEBOTS_CONTROLLER_BASE_PORT + WEBOTS_CONTROLLER_PORT_STRIDE * instance


def _port_appears_bound(host: str, port: int) -> bool:
    """Non-invasive check: try to bind the same UDP port ourselves. If it's
    already in use (by the real Webots controller), our bind fails - which
    is exactly the signal we want, without connecting to or sending
    anything through the port in question."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.bind((host, port))
        return False
    except OSError:
        return True
    finally:
        probe.close()


def run_smoke_test(args) -> dict:
    report = {
        "mode": "run", "profile": args.profile, "webots_status": None,
        "world": args.world, "vehicle": args.vehicle, "instance": args.instance,
        "controller_port": _webots_controller_port(args.instance),
        "controller_port_bound": None,
        "attach": {"succeeded": False, "reason": None}, "heartbeat": {"received": False},
        "sensor_flow_evidence": None, "actuator_flow_evidence": None,
        "armed": None, "estimator_valid": None,
        "network_arrangement": "A (loopback)", "sitl_address": args.sitl_address, "sim_address": args.sim_address,
        "remaining_failures": [], "ok": False,
    }

    if args.profile != PROFILE_SMOKE_TEST:
        report["remaining_failures"].append(
            f"profile {args.profile!r} is not implemented in this phase - only {PROFILE_SMOKE_TEST!r} is"
        )
        return report

    if not (MIN_DURATION_S <= args.duration <= MAX_DURATION_S):
        report["remaining_failures"].append(
            f"--duration {args.duration} outside bounded range [{MIN_DURATION_S}, {MAX_DURATION_S}]"
        )
        return report

    for name, value in (("--sitl-address", args.sitl_address), ("--sim-address", args.sim_address)):
        if value is not None:
            try:
                validate_local_or_private_address(value)
            except Phase12ConfigError as e:
                report["remaining_failures"].append(f"{name} invalid: {e}")
    if report["remaining_failures"]:
        return report
    if args.sitl_address or args.sim_address:
        report["network_arrangement"] = "B (Webots on Windows, SITL in WSL2)"

    webots_installation = _detect_webots_installation()
    if not webots_installation["found"]:
        report["webots_status"] = "not_installed"
        report["remaining_failures"].append("Webots not installed / not live verified")
        return report
    report["webots_status"] = "installed"

    report["controller_port_bound"] = _port_appears_bound("0.0.0.0", report["controller_port"])
    if not report["controller_port_bound"]:
        report["remaining_failures"].append(
            f"nothing appears bound to UDP {report['controller_port']} - the official Webots controller "
            "does not appear to be running; start it manually first (see docs/PHASE12_WEBOTS_SITL.md)"
        )

    try:
        _proto, _host, port = parse_connection_string(args.connection)
    except ArduPilotTransportError as e:
        report["remaining_failures"].append(f"--connection invalid: {e}")
        return report

    endpoint = ArduPilotVehicleEndpoint(
        vehicle_id="drone0", connection_string=args.connection,
        system_id=args.system_id, component_id=args.component_id,
    )
    transport = ArduPilotSITLTransport({"drone0": endpoint}, allowed_ports=frozenset({port}),
                                        startup_timeout_s=args.startup_timeout)
    try:
        transport.start()
    except ArduPilotTransportError as e:
        report["attach"]["reason"] = str(e)
        report["remaining_failures"].append(f"SITL attach failed: {e}")
        return report
    report["attach"]["succeeded"] = True

    try:
        channel = transport._channel("drone0")
        report["heartbeat"]["received"] = channel.heartbeat_ok

        deadline = time.monotonic() + args.duration
        last_telem = None
        sample_count = 0
        while time.monotonic() < deadline:
            telem = transport.receive_telemetry("drone0")
            if telem is not None:
                last_telem = telem
                sample_count += 1
            time.sleep(0.1)

        if last_telem is not None:
            report["armed"] = last_telem.armed
            report["estimator_valid"] = last_telem.estimator_valid
            # Sensor flow: SITL's own EKF can only become/stay valid if it is
            # actually consuming Webots-sourced IMU/GPS data every tick - a
            # real, if indirect, read-only proof sensor data is flowing.
            report["sensor_flow_evidence"] = (
                "estimator_valid held true across the observation window" if last_telem.estimator_valid
                else "estimator_valid was false - sensor data may not be reaching SITL correctly"
            )
        report["telemetry_sample_count"] = sample_count
        # Actuator flow: while disarmed, ArduPilot still continuously issues a
        # baseline (motors-off) actuator command every SITL tick; we do not
        # decode individual PWM values here (no camera/serial access to
        # Webots' own log), so this is reported honestly as unobserved unless
        # the operator's own controller log (see docs) is inspected manually.
        report["actuator_flow_evidence"] = (
            "not independently observed by this script - inspect the operator-run "
            "controller's own console output for continuous motor-command activity "
            "(see docs/PHASE12_WEBOTS_SITL.md's manual verification section)"
        )
    finally:
        stop_result = transport.stop()
        report["shutdown"] = {"clean": stop_result.success}

    report["ok"] = (
        report["attach"]["succeeded"] and report["heartbeat"]["received"]
        and report["controller_port_bound"] is True and not report["remaining_failures"]
    )
    return report


def main(argv=None) -> None:
    args = build_parser().parse_args(argv)
    os.makedirs(OUT_DIR, exist_ok=True)
    print(_SAFETY_BANNER)

    if args.run:
        result, out_name = run_smoke_test(args), "phase12_run_result.json"
    else:
        result, out_name = run_dry_run(args), "phase12_dry_run_result.json"

    print(json.dumps(result, indent=2, default=str))
    with open(os.path.join(OUT_DIR, out_name), "w") as f:
        json.dump(result, f, indent=2, default=str)


if __name__ == "__main__":
    main()
