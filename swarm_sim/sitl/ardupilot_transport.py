"""Phase 8: real local ArduPilot SITL transport - see docs/PHASE8_ARDUPILOT_SITL.md.

This module is the ONLY place in this project that imports pymavlink or
launches a real ArduPilot SITL process. Everything upstream of it
(SwarmController, SafetySupervisor, mission.py, distributed_consensus.py,
swarm_sim.sitl.fake_transport, swarm_sim.autopilot.sitl.SITLAdapter) is
completely unaware of MAVLink - `ArduPilotSITLTransport` implements the
exact same `swarm_sim.sitl.transport.SITLTransport` Protocol
`FakeSITLTransport` (Phase 7, unmodified) implements, so `SITLAdapter`
works identically over either.

Library / version: pymavlink==2.4.49 (see requirements.txt). Message
types used: HEARTBEAT, LOCAL_POSITION_NED, ATTITUDE, SYS_STATUS,
EKF_STATUS_REPORT, SET_POSITION_TARGET_LOCAL_NED (outgoing setpoints),
COMMAND_LONG + COMMAND_ACK (mode/LAND/ABORT). Coordinate frame: ArduPilot's
SET_POSITION_TARGET_LOCAL_NED and LOCAL_POSITION_NED are both
MAV_FRAME_LOCAL_NED - this project's own `Frame.LOCAL_NED` convention
exactly (see `swarm_sim/autopilot/frames.py`, unmodified this phase, whose
already-tested `convert_command_frame`/`convert_vector_frame` this module
calls directly rather than reimplementing any conversion math).

**This transport, not `SITLAdapter`, is where ENU<->NED conversion
happens - in BOTH directions.** `self.operating_frame` (default
`Frame.LOCAL_ENU`, this project's project-side internal convention -
matching `MockAdapter`/`FakeSITLTransport`'s own default and what
`mission.py`'s `SafetySupervisor` output already uses) is the frame every
`SITLCommand` this transport accepts must already be in, and the frame
every `SITLTelemetry` it returns is already expressed in:
`_send_mavlink_command` explicitly converts an accepted command from
`operating_frame` to `Frame.LOCAL_NED` before building the outgoing
MAVLink message; `_poll_incoming` explicitly converts incoming
`LOCAL_POSITION_NED`/`ATTITUDE` fields from `Frame.LOCAL_NED` back to
`operating_frame` before caching them. `SITLAdapter` itself (Phase 7,
unmodified) still never converts anything - by the time it sees a command
or hands back telemetry, this transport has already done the one
conversion needed, explicitly, never silently.

**Localhost-only, by construction**: every endpoint's host is validated by
`_require_loopback_host` before any connection is attempted - a
non-loopback address raises `ArduPilotTransportError` immediately, in the
constructor, before `start()` is ever called. There is no code path in
this module that accepts a host from outside `{127.0.0.1, ::1,
localhost}`. Ports are validated against an explicit allow-list the
caller must pass (`allowed_ports`) - an endpoint whose port isn't in that
list is rejected the same way. This is deliberately conservative: the
project-wide default (`autopilot_path="fake_sitl"` or `"mock_adapter"`)
never touches this module at all - see `swarm_sim/config.py`.

**No silent fallback**: if this transport cannot reach or verify the SITL
process it was configured for, `start()`/`send_command()` raise
`ArduPilotTransportError` - they NEVER fall back to `FakeSITLTransport`
behavior. A caller (mission.py, a scenario script) that wants a FakeSITL
fallback must catch this exception and explicitly choose to do so; this
module never does it for them.

**No arm, no takeoff, no motor/PWM output**: this module never sends
`MAV_CMD_COMPONENT_ARM_DISARM`, `MAV_CMD_NAV_TAKEOFF`, or anything that
would arm the vehicle - there is no arm/disarm method anywhere in this
class (checked by AST - see tests/test_ardupilot_sitl_architecture.py).
ArduPilot SITL itself stays disarmed (and therefore produces no real
motor output, even in its own internal simulated-physics sense) unless
explicitly armed, which this project never does.

**Clock model - a real, documented difference from FakeSITL**: unlike
every other module in `swarm_sim/sitl/`, this one DOES read the real wall
clock (`time.monotonic()`) - but only to bound how long it waits for a
real external OS process/socket (`startup_timeout_s`, `heartbeat_timeout_s`,
`ack_timeout_s`). This is unavoidable: ArduPilot SITL is a real, separate
process running in real time, not a value this code can advance
deterministically. The SIMULATION-facing clock exposed to the rest of the
swarm (`SITLTransport.set_sim_time`, command timestamp/expiry/staleness
semantics) is still the ordinary `swarm_sim.sitl.clock.SimClock` used
everywhere else - every telemetry object this transport returns is
stamped with `self.clock.now_s` (the caller's own simulation time), never
with a wall-clock read. See docs/PHASE8_ARDUPILOT_SITL.md's "Clock model
differences" section.
"""
from __future__ import annotations

import dataclasses
import ipaddress
import platform
import shlex
import subprocess
import threading
import time
from typing import Dict, FrozenSet, List, Optional, Sequence, Tuple

try:
    import pymavlink
    from pymavlink import mavutil
    PYMAVLINK_AVAILABLE = True
    PYMAVLINK_VERSION = getattr(pymavlink, "__version__", "unknown")
except ImportError:
    mavutil = None
    PYMAVLINK_AVAILABLE = False
    PYMAVLINK_VERSION = None

import math

from ..autopilot.frames import convert_command_frame, convert_vector_frame
from ..autopilot.types import AutopilotMode
from ..contracts import CommandType, Frame
from .clock import SimClock
from .commands import SITLCommand
from .telemetry import CommandAck, FailureType, SITLTelemetry, TransportResult
from .vehicle_namespace import NamespaceError, VehicleNamespaceRegistry

_LOOPBACK_NAMES = {"localhost"}
_NAVIGATION_COMMAND_TYPES = frozenset({
    CommandType.POSITION_SETPOINT, CommandType.VELOCITY_SETPOINT,
    CommandType.YAW_SETPOINT, CommandType.YAW_RATE_SETPOINT,
})

# Standard MAVLink SET_POSITION_TARGET_LOCAL_NED type_mask bit meanings
# (documented MAVLink common-dialect convention, not this project's own
# invention): bit N=1 means "ignore this field". See
# docs/PHASE8_ARDUPILOT_SITL.md's "Command mapping" section.
_TYPEMASK_IGNORE_VX_VY_VZ_AFX_AFY_AFZ_YAW_YAWRATE = 0x0DF8   # use only position
_TYPEMASK_IGNORE_PX_PY_PZ_AFX_AFY_AFZ_YAW_YAWRATE = 0x0DC7   # use only velocity
_TYPEMASK_IGNORE_PX_PY_PZ_AFX_AFY_AFZ = 0x0DC0               # use velocity + yaw/yaw_rate (yaw-only setpoint hold)


class ArduPilotTransportError(RuntimeError):
    """Raised on any real-SITL configuration/connection/process failure.
    Never caught anywhere in this module to silently substitute FakeSITL
    behavior - see module docstring's "No silent fallback"."""


def _require_loopback_host(host: str) -> None:
    if host in _LOOPBACK_NAMES:
        return
    try:
        ip = ipaddress.ip_address(host)
    except ValueError:
        raise ArduPilotTransportError(
            f"endpoint host {host!r} is not 'localhost' and not a valid IP literal - "
            "refusing a non-loopback ArduPilot SITL endpoint"
        )
    if not ip.is_loopback:
        raise ArduPilotTransportError(
            f"endpoint host {host!r} is not a loopback address - real ArduPilot SITL "
            "transport only ever accepts localhost endpoints"
        )


def parse_connection_string(conn_str: str) -> Tuple[str, str, int]:
    """Parses pymavlink's own 'proto:host:port' connection-string format
    (not a standard URI) - e.g. 'tcp:127.0.0.1:5760'. Raises
    ArduPilotTransportError on anything else, including a bare hostname
    with no explicit port (never inferred)."""
    parts = conn_str.split(":")
    if len(parts) != 3:
        raise ArduPilotTransportError(
            f"unsupported connection string {conn_str!r} - expected exactly 'proto:host:port'"
        )
    proto, host, port_str = parts
    if proto not in ("tcp", "udp", "udpin", "udpout"):
        raise ArduPilotTransportError(f"unsupported connection protocol {proto!r} in {conn_str!r}")
    try:
        port = int(port_str)
    except ValueError:
        raise ArduPilotTransportError(f"invalid port in connection string {conn_str!r}")
    return proto, host, port


@dataclasses.dataclass(frozen=True)
class ArduPilotVehicleEndpoint:
    """One vehicle's real-SITL configuration. `executable_path=None` means
    ATTACH mode: connect to a SITL instance the operator already started
    (this is what Phase 8's actual verification run uses - see
    docs/PHASE8_ARDUPILOT_SITL.md). A non-None `executable_path` means
    SPAWN mode: this transport launches that binary itself (see
    `_SITLProcessHandle`)."""
    vehicle_id: str
    connection_string: str          # e.g. "tcp:127.0.0.1:5760" - localhost only, checked in __post_init__
    system_id: int
    component_id: int = 1           # MAV_COMP_ID_AUTOPILOT1
    executable_path: Optional[str] = None
    extra_args: Tuple[str, ...] = ()
    wsl_distro: Optional[str] = None   # required when spawning from a Windows host - see _SITLProcessHandle
    # The Linux-side directory to run the SITL binary from (e.g. an
    # ArduPilot checkout root such as "/home/user/ardupilot") - real
    # ArduCopter SITL opens its EEPROM/parameter files relative to its
    # working directory and can exit shortly after startup if this is
    # wrong or unwritable. Required for spawn mode: without it, on
    # Windows, wsl.exe defaults the Linux-side cwd to a DrvFs translation
    # of the *Windows* caller's own working directory (e.g.
    # "/mnt/c/Users/..."), not any Linux path related to the SITL
    # checkout - confirmed during Phase 8 real-SITL verification, where
    # this caused ArduCopter to bind its MAVLink port successfully and
    # then exit(1) immediately after. See docs/PHASE8_ARDUPILOT_SITL.md.
    working_directory: Optional[str] = None

    def __post_init__(self):
        proto, host, port = parse_connection_string(self.connection_string)
        _require_loopback_host(host)
        if not (1 <= self.system_id <= 255):
            raise ArduPilotTransportError(f"system_id must be in [1, 255], got {self.system_id!r}")
        if not (1 <= self.component_id <= 255):
            raise ArduPilotTransportError(f"component_id must be in [1, 255], got {self.component_id!r}")


@dataclasses.dataclass
class DryRunReport:
    """Result of `ArduPilotSITLTransport.dry_run_validate` - validates
    configuration only, never opens a connection or starts a process
    (Phase 8 requirement 10)."""
    ok: bool
    vehicle_ids: Tuple[str, ...]
    problems: Tuple[str, ...]
    endpoints_checked: Tuple[dict, ...]

    def to_dict(self) -> dict:
        return {
            "ok": self.ok, "vehicle_ids": list(self.vehicle_ids),
            "problems": list(self.problems), "endpoints_checked": list(self.endpoints_checked),
        }


def dry_run_validate(endpoints: Dict[str, ArduPilotVehicleEndpoint],
                      allowed_ports: Optional[FrozenSet[int]] = None) -> DryRunReport:
    """Validates every endpoint's executable path, localhost-ness, port
    allow-list, vehicle namespace uniqueness, and system/component ID
    uniqueness - WITHOUT starting any process or opening any socket.
    pymavlink need not even be importable for this to run (only used to
    check configuration shape)."""
    problems: List[str] = []
    registry = VehicleNamespaceRegistry()
    seen_sysids: Dict[int, str] = {}
    seen_conns: Dict[str, str] = {}
    checked = []
    for vehicle_id, endpoint in endpoints.items():
        entry = {"vehicle_id": vehicle_id, "connection_string": endpoint.connection_string,
                  "system_id": endpoint.system_id, "component_id": endpoint.component_id,
                  "executable_path": endpoint.executable_path, "mode": "spawn" if endpoint.executable_path else "attach"}
        checked.append(entry)
        try:
            registry.register(vehicle_id)
        except NamespaceError as e:
            problems.append(str(e))
            continue
        try:
            proto, host, port = parse_connection_string(endpoint.connection_string)
            _require_loopback_host(host)
        except ArduPilotTransportError as e:
            problems.append(str(e))
            continue
        if allowed_ports is not None and port not in allowed_ports:
            problems.append(f"{vehicle_id}: port {port} is not in the configured allowed_ports {sorted(allowed_ports)}")
        if endpoint.system_id in seen_sysids:
            problems.append(f"{vehicle_id}: duplicate system_id {endpoint.system_id} (already used by {seen_sysids[endpoint.system_id]!r})")
        else:
            seen_sysids[endpoint.system_id] = vehicle_id
        if endpoint.connection_string in seen_conns:
            problems.append(f"{vehicle_id}: duplicate connection_string {endpoint.connection_string!r} (already used by {seen_conns[endpoint.connection_string]!r})")
        else:
            seen_conns[endpoint.connection_string] = vehicle_id
        if endpoint.executable_path is not None:
            if platform.system() == "Windows" and not endpoint.wsl_distro:
                problems.append(f"{vehicle_id}: spawning a local SITL binary from Windows requires wsl_distro to be set")
            if not endpoint.working_directory:
                problems.append(f"{vehicle_id}: spawn mode requires working_directory to be set "
                                 f"(the SITL binary's own checkout directory - see ArduPilotVehicleEndpoint docstring)")
    if not PYMAVLINK_AVAILABLE:
        problems.append("pymavlink is not importable - real ArduPilot SITL transport cannot run (see requirements.txt)")
    return DryRunReport(ok=(len(problems) == 0), vehicle_ids=tuple(endpoints.keys()),
                         problems=tuple(problems), endpoints_checked=tuple(checked))


class _SITLProcessHandle:
    """Owns exactly one subprocess this transport itself launched. Never
    re-acquires a process handle by PID lookup - every lifecycle method
    (`poll_exit_code`, `is_running`, `stop`) acts only on the exact
    `subprocess.Popen` object created in `start()`, so a PID-reuse race
    misidentifying an unrelated process after this one has already exited
    is impossible by construction (there is no code path that looks up a
    process by bare PID at all). `shell=False` always; args are a list,
    never a formatted string - see module docstring.

    stdout/stderr are drained CONTINUOUSLY by background reader threads,
    not read only after the process exits - a chatty long-running SITL
    process filling the OS pipe buffer while nobody reads it would
    otherwise block that process's own writes indefinitely (a real
    deadlock risk this project avoids by construction, not by luck)."""

    _MAX_BUFFERED_LINES = 4000  # bounded - a long real-SITL session must not grow this without limit

    def __init__(self, executable_path: str, args: Sequence[str] = (), wsl_distro: Optional[str] = None,
                 working_directory: Optional[str] = None):
        self.executable_path = executable_path
        self.args = tuple(args)
        self.wsl_distro = wsl_distro
        self.working_directory = working_directory
        self._popen: Optional[subprocess.Popen] = None
        self.pid: Optional[int] = None
        self.launched_argv: Optional[Tuple[str, ...]] = None
        self.exit_code: Optional[int] = None
        self._output_lock = threading.Lock()
        self._stdout_lines: List[str] = []
        self._stderr_lines: List[str] = []
        self._stdout_thread: Optional[threading.Thread] = None
        self._stderr_thread: Optional[threading.Thread] = None

    def _reader_loop(self, pipe, buffer_list: List[str]) -> None:
        # readline() blocks until a line (or EOF) is available - safe here
        # only because it runs on its own dedicated daemon thread, never on
        # the caller's thread, and returns '' (falsy) at EOF instead of
        # raising, which cleanly ends this loop when the process exits.
        try:
            for line in iter(pipe.readline, ""):
                with self._output_lock:
                    buffer_list.append(line)
                    if len(buffer_list) > self._MAX_BUFFERED_LINES:
                        del buffer_list[: len(buffer_list) - self._MAX_BUFFERED_LINES]
        except (ValueError, OSError):
            pass  # pipe closed under us during shutdown - not an error worth surfacing

    @property
    def stdout_text(self) -> str:
        with self._output_lock:
            return "".join(self._stdout_lines)

    @property
    def stderr_text(self) -> str:
        with self._output_lock:
            return "".join(self._stderr_lines)

    def start(self) -> None:
        popen_cwd = None
        if platform.system() == "Windows":
            if not self.wsl_distro:
                raise ArduPilotTransportError(
                    "spawning a local SITL binary from a Windows host requires wsl_distro "
                    "(e.g. 'Ubuntu-22.04') - Windows cannot directly execute a Linux ELF binary"
                )
            # Real finding from Phase 8 real-SITL verification: invoking
            # the binary directly after wsl.exe's own "--" (no shell) lets
            # ArduCopter bind its MAVLink port and then crash (exit 1) the
            # INSTANT a real client connects - reproduced repeatedly,
            # isolated down to this exact invocation shape (confirmed
            # stable for 10+ idle seconds with no connection; confirmed
            # crashing within the same second a client connects). Routing
            # the exact same argv through "bash -c" instead (still
            # `shell=False` at the Python/subprocess level - this list is
            # never parsed by a shell on the Windows side; only WSL's own
            # bash, given fixed, shlex-quoted, internally-controlled
            # arguments, ever sees a string) does not exhibit this at all -
            # this matches exactly how this project's own long-running
            # verification instances (Real-SITL verification results) were
            # started, and is not itself shell injection: every value
            # quoted here comes from this transport's own validated
            # ArduPilotVehicleEndpoint, never from unvalidated external
            # input. `exec` replaces the shell process so the tracked
            # child is arducopter itself, not a lingering bash wrapper.
            parts = [self.executable_path, *self.args]
            quoted = " ".join(shlex.quote(p) for p in parts)
            if self.working_directory:
                inner_cmd = f"cd {shlex.quote(self.working_directory)} && exec {quoted}"
            else:
                inner_cmd = f"exec {quoted}"
            argv = ("wsl.exe", "-d", self.wsl_distro, "--", "bash", "-c", inner_cmd)
        else:
            argv = (self.executable_path, *self.args)
            popen_cwd = self.working_directory
        self.launched_argv = argv
        self._popen = subprocess.Popen(list(argv), shell=False, stdout=subprocess.PIPE,
                                        stderr=subprocess.PIPE, text=True, cwd=popen_cwd)
        self.pid = self._popen.pid
        # Guard on `is not None` rather than assuming a pipe: `stdout=PIPE`
        # always gives a real file object on an unmocked Popen, but a test
        # that mocks subprocess.Popen leaves `.stdout`/`.stderr` as
        # MagicMock objects whose `.readline()` never returns "" - without
        # this guard, `_reader_loop`'s `iter(pipe.readline, "")` would spin
        # forever (a real, observed hang: a mocked-Popen test left daemon
        # threads busy-looping indefinitely, spinning up unbounded mock
        # call-history memory and starving every later test's CPU time).
        if self._popen.stdout is not None:
            self._stdout_thread = threading.Thread(
                target=self._reader_loop, args=(self._popen.stdout, self._stdout_lines), daemon=True)
            self._stdout_thread.start()
        if self._popen.stderr is not None:
            self._stderr_thread = threading.Thread(
                target=self._reader_loop, args=(self._popen.stderr, self._stderr_lines), daemon=True)
            self._stderr_thread.start()

    def poll_exit_code(self) -> Optional[int]:
        if self._popen is None:
            return None
        self.exit_code = self._popen.poll()
        return self.exit_code

    def is_running(self) -> bool:
        return self._popen is not None and self.poll_exit_code() is None

    def drain_output_after_exit(self) -> None:
        """Gives the background reader threads a brief bounded moment to
        catch up on any final output once the process has exited - the
        threads already drain continuously while it runs, so this is only
        a small join, never a blocking read of a live pipe."""
        if self._popen is None or self._popen.poll() is None:
            return
        if self._stdout_thread is not None:
            self._stdout_thread.join(timeout=1.0)
        if self._stderr_thread is not None:
            self._stderr_thread.join(timeout=1.0)

    def stop(self, graceful_timeout_s: float = 5.0) -> None:
        if self._popen is None:
            return
        if self._popen.poll() is None:
            self._popen.terminate()
            try:
                self._popen.wait(timeout=graceful_timeout_s)
            except subprocess.TimeoutExpired:
                self._popen.kill()
                self._popen.wait(timeout=graceful_timeout_s)
        self.exit_code = self._popen.returncode
        self.drain_output_after_exit()


@dataclasses.dataclass
class _VehicleMavlinkChannel:
    vehicle_id: str
    namespace: str
    endpoint: ArduPilotVehicleEndpoint
    process: Optional[_SITLProcessHandle] = None
    connection: object = None
    connected: bool = False
    heartbeat_ok: bool = False
    last_heartbeat_monotonic_s: Optional[float] = None
    last_state_update_sim_s: Optional[float] = None
    last_accepted_sequence: Optional[int] = None
    last_accepted_command: Optional[SITLCommand] = None
    last_ack: Optional[CommandAck] = None
    position_m: Optional[Tuple[float, float, float]] = None
    velocity_mps: Optional[Tuple[float, float, float]] = None
    attitude_rad: Optional[Tuple[float, float, float]] = None
    angular_velocity_radps: Optional[Tuple[float, float, float]] = None
    battery_fraction: Optional[float] = None
    estimator_valid: bool = False
    armed: bool = False
    mode: AutopilotMode = AutopilotMode.UNKNOWN
    telemetry_sequence: int = 0
    last_servo_output_raw: Optional[Tuple[int, int, int, int]] = None
    last_servo_output_time_usec: Optional[int] = None
    servo_output_messages_received: int = 0
    active_failures: set = dataclasses.field(default_factory=set)
    command_history: List[Tuple[SITLCommand, CommandAck]] = dataclasses.field(default_factory=list)
    ack_history: List[CommandAck] = dataclasses.field(default_factory=list)
    failure_log: List[Tuple[float, FailureType, bool]] = dataclasses.field(default_factory=list)
    mode_transition_log: List[Tuple[float, AutopilotMode]] = dataclasses.field(default_factory=list)

    def set_mode(self, mode: AutopilotMode, at_s: float) -> None:
        if mode != self.mode:
            self.mode_transition_log.append((at_s, mode))
        self.mode = mode


_MAV_MODE_NAME_FOR_AUTOPILOT_MODE = {
    AutopilotMode.HOLD: "LOITER", AutopilotMode.RTL: "RTL", AutopilotMode.LAND: "LAND",
    AutopilotMode.OFFBOARD: "GUIDED", AutopilotMode.GUIDED: "GUIDED",
}


class ArduPilotSITLTransport:
    """Real local ArduPilot SITL transport - see module docstring. Hosts
    one or more vehicles, each its own dedicated localhost MAVLink
    connection (and, in spawn mode, its own subprocess) - mirrors
    FakeSITLTransport's own multi-vehicle-in-one-object design (Phase 7,
    unmodified) so namespace isolation is checked the same way."""

    def __init__(self, endpoints: Dict[str, ArduPilotVehicleEndpoint], *,
                 allowed_ports: FrozenSet[int], operating_frame: Frame = Frame.LOCAL_ENU,
                 startup_timeout_s: float = 30.0, heartbeat_timeout_s: float = 5.0,
                 ack_timeout_s: float = 3.0, future_tolerance_s: float = 0.5,
                 telemetry_stale_timeout_s: float = 1.0, clock: Optional[SimClock] = None):
        if not PYMAVLINK_AVAILABLE:
            raise ArduPilotTransportError(
                "pymavlink is not installed - real ArduPilot SITL transport requires it "
                "(pip install pymavlink==2.4.49, see requirements.txt). Refusing to start; "
                "this transport never silently falls back to FakeSITL."
            )
        self.registry = VehicleNamespaceRegistry()
        self._channels: Dict[str, _VehicleMavlinkChannel] = {}
        seen_sysids: Dict[int, str] = {}
        seen_conns: Dict[str, str] = {}
        for vehicle_id, endpoint in endpoints.items():
            namespace = self.registry.register(vehicle_id)
            proto, host, port = parse_connection_string(endpoint.connection_string)
            _require_loopback_host(host)
            if port not in allowed_ports:
                raise ArduPilotTransportError(
                    f"{vehicle_id}: port {port} is not in the configured allowed_ports {sorted(allowed_ports)} - "
                    "reject unexpected ports by default"
                )
            if endpoint.system_id in seen_sysids:
                raise ArduPilotTransportError(f"duplicate system_id {endpoint.system_id} for {vehicle_id!r}")
            seen_sysids[endpoint.system_id] = vehicle_id
            if endpoint.connection_string in seen_conns:
                raise ArduPilotTransportError(f"duplicate connection_string {endpoint.connection_string!r} for {vehicle_id!r}")
            seen_conns[endpoint.connection_string] = vehicle_id
            self._channels[vehicle_id] = _VehicleMavlinkChannel(vehicle_id, namespace, endpoint)

        self.allowed_ports = allowed_ports
        self.operating_frame = operating_frame
        self.startup_timeout_s = startup_timeout_s
        self.heartbeat_timeout_s = heartbeat_timeout_s
        self.ack_timeout_s = ack_timeout_s
        self.telemetry_stale_timeout_s = telemetry_stale_timeout_s
        self.clock = clock if clock is not None else SimClock(future_tolerance_s=future_tolerance_s)
        self._running = False
        self._ever_started = False
        self.start_stop_log: List[Tuple[float, bool]] = []

    def _channel(self, vehicle_id: str) -> _VehicleMavlinkChannel:
        if vehicle_id not in self._channels:
            raise NamespaceError(f"vehicle_id {vehicle_id!r} is not registered on this transport")
        return self._channels[vehicle_id]

    @property
    def is_running(self) -> bool:
        return self._running

    # -- lifecycle: start / verify identity / verify heartbeat --------------

    def start(self) -> TransportResult:
        if self._running:
            # Idempotent, matching MockAdapter/FakeSITLTransport's own
            # documented connect()/start() convention - important here
            # specifically because SITLAdapter.connect() (Phase 7,
            # unmodified, called once per vehicle) calls transport.start()
            # itself, so a multi-vehicle mission.py construction calls
            # this once explicitly and then once again per adapter -
            # real MAVLink connections must never be re-opened/re-waited
            # for as a result.
            return TransportResult(success=True, reason="already_started", timestamp_s=self.clock.now_s)
        started_vehicle_ids = []
        try:
            for vehicle_id, channel in self._channels.items():
                self._start_channel(channel)
                started_vehicle_ids.append(vehicle_id)
        except ArduPilotTransportError:
            # Partial start - clean up whatever we did manage to start
            # before re-raising, so a failed start never leaves an orphan
            # process/connection behind.
            for vehicle_id in started_vehicle_ids:
                self._stop_channel(self._channels[vehicle_id])
            raise
        self._running = True
        self._ever_started = True
        self.start_stop_log.append((self.clock.now_s, True))
        return TransportResult(success=True, reason="started", timestamp_s=self.clock.now_s)

    def _start_channel(self, channel: _VehicleMavlinkChannel) -> None:
        endpoint = channel.endpoint
        if endpoint.executable_path is not None:
            channel.process = _SITLProcessHandle(endpoint.executable_path, endpoint.extra_args, endpoint.wsl_distro,
                                                  endpoint.working_directory)
            channel.process.start()
            # A freshly spawned real SITL binary needs a moment to parse
            # its own arguments, load its home location/frame parameters
            # and open its EEPROM/parameter storage before it is at all
            # ready to accept a MAVLink connection - connecting into that
            # narrow window (rather than the "port bound, cleanly waiting"
            # steady state) is a real, observed source of instability
            # during Phase 8's real-SITL verification. This is a single
            # bounded sleep, not a loop, and does not affect ATTACH mode
            # (executable_path is None there) at all.
            time.sleep(1.5)

        deadline = time.monotonic() + self.startup_timeout_s

        # For a "tcp:" (client) connection string, pymavlink's own
        # mavlink_connection() actively connect()s immediately and raises
        # a plain socket OSError (e.g. ConnectionRefusedError) if nothing
        # is listening yet - real, expected during the window before a
        # freshly spawned SITL process (or an operator-started one the
        # caller hasn't launched yet) starts listening. Retried within the
        # same bounded startup_timeout_s rather than failing on the first
        # attempt.
        while channel.connection is None:
            if channel.process is not None and not channel.process.is_running():
                channel.process.drain_output_after_exit()
                raise ArduPilotTransportError(
                    f"{channel.vehicle_id}: SITL process exited during startup "
                    f"(exit code {channel.process.exit_code}); stderr: {channel.process.stderr_text[-500:]}"
                )
            try:
                # retries=0: pymavlink's own internal retry/sleep loop for
                # "tcp:" client connections defaults to 3 retries with a
                # 1s sleep between each - that would silently eat most of
                # our own bounded startup_timeout_s inside a single call,
                # leaving this loop unable to react promptly to the SITL
                # process exiting. This loop owns its own pacing instead.
                channel.connection = mavutil.mavlink_connection(
                    endpoint.connection_string, source_system=255, source_component=0, retries=0,
                )
            except OSError:
                if time.monotonic() >= deadline:
                    raise ArduPilotTransportError(
                        f"{channel.vehicle_id}: could not connect to {endpoint.connection_string} "
                        f"within {self.startup_timeout_s}s"
                    )
                time.sleep(0.2)

        while time.monotonic() < deadline:
            if channel.process is not None and not channel.process.is_running():
                channel.process.drain_output_after_exit()
                raise ArduPilotTransportError(
                    f"{channel.vehicle_id}: SITL process exited during startup "
                    f"(exit code {channel.process.exit_code}); stderr: {channel.process.stderr_text[-500:]}"
                )
            # blocking=True is deliberately avoided here: pymavlink's own
            # blocking wait uses select() internally, and a socket that has
            # already seen EOF (e.g. the SITL process died mid-wait) is
            # perpetually select()-ready, so a blocking call busy-spins for
            # its ENTIRE timeout window instead of waiting quietly - a real,
            # observed behavior (confirmed via source inspection and a live
            # reproduction: thousands of "EOF on TCP socket" prints within
            # a single call). Non-blocking + this loop's own explicit sleep
            # keeps pacing entirely in this code's own hands.
            msg = channel.connection.recv_match(type="HEARTBEAT", blocking=False)
            if msg is None:
                time.sleep(0.1)
                continue
            if msg.get_srcSystem() == endpoint.system_id and msg.get_srcComponent() == endpoint.component_id:
                channel.connected = True
                channel.heartbeat_ok = True
                channel.last_heartbeat_monotonic_s = time.monotonic()
                self._request_telemetry_streams(channel)
                return
            # Heartbeat from an identity we did not configure for this
            # vehicle - ignored, never accepted as this vehicle's own.
        raise ArduPilotTransportError(
            f"{channel.vehicle_id}: timed out waiting {self.startup_timeout_s}s for a HEARTBEAT "
            f"from system_id={endpoint.system_id}, component_id={endpoint.component_id} "
            f"on {endpoint.connection_string}"
        )

    # Real ArduPilot (unlike this project's dummy test vehicle, which
    # always streams telemetry unsolicited) does not proactively send
    # LOCAL_POSITION_NED/ATTITUDE/SYS_STATUS/EKF_STATUS_REPORT to a freshly
    # connected MAVLink endpoint - confirmed against a real local
    # ArduCopter SITL instance (Phase 8 real-SITL verification): these
    # messages simply never arrive without this explicit per-message
    # opt-in. This is a real, documented ArduPilot behavior difference
    # from FakeSITLTransport, not a workaround - see
    # docs/PHASE8_ARDUPILOT_SITL.md.
    _REQUIRED_TELEMETRY_MESSAGE_IDS = (
        mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED if PYMAVLINK_AVAILABLE else None,
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE if PYMAVLINK_AVAILABLE else None,
        mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS if PYMAVLINK_AVAILABLE else None,
        mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT if PYMAVLINK_AVAILABLE else None,
    )

    def _request_telemetry_streams(self, channel: "_VehicleMavlinkChannel", rate_hz: float = 10.0) -> None:
        endpoint = channel.endpoint
        interval_us = 1_000_000.0 / rate_hz
        for message_id in self._REQUIRED_TELEMETRY_MESSAGE_IDS:
            channel.connection.mav.command_long_send(
                endpoint.system_id, endpoint.component_id,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                message_id, interval_us, 0, 0, 0, 0, 0,
            )

    def stop(self) -> TransportResult:
        for channel in self._channels.values():
            self._stop_channel(channel)
        self._running = False
        self.start_stop_log.append((self.clock.now_s, False))
        return TransportResult(success=True, reason="stopped", timestamp_s=self.clock.now_s)

    def _stop_channel(self, channel: _VehicleMavlinkChannel) -> None:
        if channel.connection is not None:
            try:
                channel.connection.close()
            except Exception:
                pass
            channel.connection = None
        if channel.process is not None:
            channel.process.stop()
        channel.connected = False
        channel.heartbeat_ok = False

    def set_sim_time(self, now_s: float) -> None:
        self.clock.set(now_s)

    def inject_failure(self, vehicle_id: str, failure: FailureType, active: bool = True) -> None:
        """Only VEHICLE_DISCONNECT/COMMAND_PACKET_LOSS/TELEMETRY_PACKET_LOSS
        are meaningfully injectable on a REAL connection this way (a real
        heartbeat/estimator/battery failure comes from ArduPilot's own
        state, polled in `_poll_incoming`, not from this method) - see
        docs/PHASE8_ARDUPILOT_SITL.md's "FakeSITL versus real-SITL
        differences" section for the honest limitation this implies."""
        channel = self._channel(vehicle_id)
        if active:
            channel.active_failures.add(failure)
        else:
            channel.active_failures.discard(failure)
        channel.failure_log.append((self.clock.now_s, failure, active))

    # -- incoming telemetry ---------------------------------------------------

    def _poll_incoming(self, channel: _VehicleMavlinkChannel) -> None:
        """Non-blocking drain of every pending MAVLink message for this
        vehicle's connection, updating cached channel state. Every
        message is checked against this vehicle's configured
        system_id/component_id before being accepted - a message from an
        unexpected identity is discarded, never merged into this
        channel's state (this is what makes "wrong system ID"/"wrong
        component ID" rejection real at the telemetry layer, not just the
        command layer)."""
        if channel.connection is None:
            return
        endpoint = channel.endpoint
        while True:
            msg = channel.connection.recv_match(blocking=False)
            if msg is None:
                return
            if msg.get_srcSystem() != endpoint.system_id or msg.get_srcComponent() != endpoint.component_id:
                continue
            msg_type = msg.get_type()
            if msg_type == "HEARTBEAT":
                channel.heartbeat_ok = True
                channel.last_heartbeat_monotonic_s = time.monotonic()
                channel.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                mode_name = mavutil.mode_mapping_acm.get(msg.custom_mode) if mavutil.mode_mapping_acm else None
                if mode_name:
                    channel.mode = _autopilot_mode_from_ardupilot_mode_name(mode_name)
            elif msg_type == "LOCAL_POSITION_NED":
                # ArduPilot's wire frame is always MAV_FRAME_LOCAL_NED -
                # explicitly converted here to this transport's own
                # `operating_frame` (LOCAL_ENU by default, the
                # project-side internal convention) via the unmodified,
                # already-tested convert_vector_frame - never left as raw
                # NED numbers mislabeled under a different frame.
                ned_position = (float(msg.x), float(msg.y), float(msg.z))
                ned_velocity = (float(msg.vx), float(msg.vy), float(msg.vz))
                if self.operating_frame in (Frame.LOCAL_NED, Frame.BODY):
                    # BODY is never a meaningful TELEMETRY frame (an
                    # instantaneous "position relative to current
                    # heading" is not a lasting quantity) - a transport
                    # configured with operating_frame=BODY (an allowed,
                    # if unusual, command-frame choice - see the
                    # frame-mismatch check above) still reports raw NED
                    # telemetry rather than attempting a nonsensical
                    # conversion or crashing when attitude is absent.
                    channel.position_m = ned_position
                    channel.velocity_mps = ned_velocity
                else:
                    channel.position_m = convert_vector_frame(ned_position, Frame.LOCAL_NED, self.operating_frame)
                    channel.velocity_mps = convert_vector_frame(ned_velocity, Frame.LOCAL_NED, self.operating_frame)
                channel.last_state_update_sim_s = self.clock.now_s
            elif msg_type == "ATTITUDE":
                # Yaw is converted NED->ENU (or left as-is) the same way;
                # roll/pitch are stored as ArduPilot reports them (NED
                # Euler convention) - this project's own frame-conversion
                # utilities only define a NED<->ENU mapping for yaw, not
                # full roll/pitch, since nothing here consumes roll/pitch
                # besides the yaw-only BODY conversion math (which only
                # ever reads attitude_rad[2]) - documented, not hidden.
                yaw = _yaw_ned_to_enu(float(msg.yaw)) if self.operating_frame == Frame.LOCAL_ENU else float(msg.yaw)
                channel.attitude_rad = (float(msg.roll), float(msg.pitch), yaw)
                channel.angular_velocity_radps = (float(msg.rollspeed), float(msg.pitchspeed), float(msg.yawspeed))
            elif msg_type == "SYS_STATUS":
                if msg.battery_remaining >= 0:
                    channel.battery_fraction = max(0.0, min(1.0, msg.battery_remaining / 100.0))
            elif msg_type == "EKF_STATUS_REPORT":
                channel.estimator_valid = bool(
                    msg.flags & (mavutil.mavlink.EKF_POS_HORIZ_ABS | mavutil.mavlink.EKF_VELOCITY_HORIZ)
                )
            elif msg_type == "SERVO_OUTPUT_RAW":
                # Cached here - the only place this connection's socket is
                # ever read - rather than via a second, competing
                # recv_match() call elsewhere: a second reader racing this
                # loop would either miss messages this loop already
                # consumed, or itself consume a HEARTBEAT/ATTITUDE/etc.
                # message meant for the branches above. See
                # docs/PHASE13_WEBOTS_FLIGHT_TEST.md's "Actuator-output
                # logging" section for the reliability bug this fixes.
                channel.last_servo_output_raw = (
                    int(msg.servo1_raw), int(msg.servo2_raw), int(msg.servo3_raw), int(msg.servo4_raw),
                )
                channel.last_servo_output_time_usec = int(msg.time_usec)
                channel.servo_output_messages_received += 1

        # Heartbeat timeout: if we haven't seen ANY heartbeat recently
        # (real wall-clock bound - see module docstring's clock-model note).

    def _check_heartbeat_timeout(self, channel: _VehicleMavlinkChannel) -> None:
        if channel.last_heartbeat_monotonic_s is None:
            channel.heartbeat_ok = False
            return
        if time.monotonic() - channel.last_heartbeat_monotonic_s > self.heartbeat_timeout_s:
            channel.heartbeat_ok = False

    def receive_telemetry(self, vehicle_id: str) -> SITLTelemetry:
        channel = self._channel(vehicle_id)
        if channel.connection is not None:
            self._poll_incoming(channel)
            self._check_heartbeat_timeout(channel)
        last_ack_status = None
        if channel.last_ack is not None:
            last_ack_status = "accepted" if channel.last_ack.accepted else f"rejected:{channel.last_ack.reason}"
        return SITLTelemetry(
            vehicle_id=vehicle_id, namespace=channel.namespace, timestamp_s=self.clock.now_s,
            sequence=channel.telemetry_sequence, position_m=channel.position_m, velocity_mps=channel.velocity_mps,
            attitude_rad=channel.attitude_rad, angular_velocity_radps=channel.angular_velocity_radps,
            battery_fraction=channel.battery_fraction,
            estimator_valid=(channel.estimator_valid and FailureType.ESTIMATOR_INVALID not in channel.active_failures),
            armed=channel.armed, flight_mode=channel.mode,
            failsafe=bool(channel.active_failures) or not channel.heartbeat_ok,
            heartbeat_ok=channel.heartbeat_ok and FailureType.HEARTBEAT_LOSS not in channel.active_failures,
            last_command_ack_status=last_ack_status,
        )

    # -- outgoing commands ----------------------------------------------------

    def send_command(self, command: SITLCommand) -> TransportResult:
        if not isinstance(command, SITLCommand):
            raise TypeError(f"send_command requires a SITLCommand, got {type(command)!r}")
        now_s = self.clock.now_s
        if not self._running:
            reason = "transport_stopped" if self._ever_started else "transport_not_started"
            return TransportResult(success=False, reason=reason, timestamp_s=now_s)
        if command.vehicle_id not in self._channels:
            return TransportResult(success=False, reason="unknown_vehicle_id", timestamp_s=now_s)
        try:
            self.registry.require_match(command.vehicle_id, command.namespace)
        except NamespaceError:
            return TransportResult(success=False, reason="namespace_mismatch", timestamp_s=now_s)

        channel = self._channel(command.vehicle_id)
        self._poll_incoming(channel)
        self._check_heartbeat_timeout(channel)
        ack = self._evaluate_and_send(channel, command)
        channel.last_ack = ack
        channel.command_history.append((command, ack))
        channel.ack_history.append(ack)
        return TransportResult(success=True, reason="queued", timestamp_s=now_s)

    def _evaluate_and_send(self, channel: _VehicleMavlinkChannel, command: SITLCommand) -> CommandAck:
        cmd_time_s = command.timestamp_s
        is_navigation = command.command_type in _NAVIGATION_COMMAND_TYPES

        def reject(reason: str) -> CommandAck:
            return CommandAck(vehicle_id=command.vehicle_id, namespace=command.namespace,
                               command_sequence=command.sequence, accepted=False, reason=reason,
                               timestamp_s=cmd_time_s, autopilot_mode=channel.mode,
                               failsafe=bool(channel.active_failures) or not channel.heartbeat_ok)

        if self.clock.is_from_the_future(cmd_time_s):
            return reject("command_from_the_future")
        if FailureType.VEHICLE_DISCONNECT in channel.active_failures or not channel.connected:
            return reject("disconnected")
        if not channel.heartbeat_ok or FailureType.HEARTBEAT_LOSS in channel.active_failures:
            return reject("heartbeat_lost")
        if command.frame != self.operating_frame and command.frame != Frame.BODY:
            # BODY is always an allowed exception to the configured
            # operating_frame, never validated by simple equality like
            # ENU/NED - a body-relative setpoint is always attitude-gated
            # at the actual conversion step below instead (see
            # docs/PHASE8_ARDUPILOT_SITL.md's frame-conversion section).
            return reject("frame_mismatch")
        effective_arrival_s = cmd_time_s
        if not (cmd_time_s <= effective_arrival_s <= command.expiration_time_s):
            return reject("expired")
        if FailureType.ESTIMATOR_INVALID in channel.active_failures and is_navigation:
            channel.set_mode(AutopilotMode.HOLD, at_s=cmd_time_s)
            return reject("estimator_invalid")
        if FailureType.BATTERY_CRITICAL in channel.active_failures and is_navigation:
            channel.set_mode(AutopilotMode.HOLD, at_s=cmd_time_s)
            return reject("battery_critical")
        if is_navigation and (channel.last_state_update_sim_s is None
                               or (cmd_time_s - channel.last_state_update_sim_s) > self.telemetry_stale_timeout_s):
            channel.set_mode(AutopilotMode.HOLD, at_s=cmd_time_s)
            return reject("stale_telemetry")

        if channel.last_accepted_sequence is not None:
            if command.sequence == channel.last_accepted_sequence:
                if _same_command_content(command, channel.last_accepted_command):
                    return channel.last_ack if channel.last_ack is not None else reject("no_prior_ack")
                return reject("sequence_replayed_conflicting")
            if command.sequence < channel.last_accepted_sequence:
                return reject("sequence_replayed_stale")

        try:
            self._send_mavlink_command(channel, command)
        except ArduPilotTransportError as e:
            return reject(f"mavlink_send_failed:{e}")
        except ValueError as e:
            # convert_command_frame raises ValueError for a BODY
            # conversion with no vehicle attitude available yet - a real,
            # expected rejection path (see convert_command_frame's own
            # docstring), not a bug to let propagate uncaught.
            return reject(f"frame_conversion_failed:{e}")

        channel.last_accepted_sequence = command.sequence
        channel.last_accepted_command = command
        return CommandAck(vehicle_id=command.vehicle_id, namespace=command.namespace,
                           command_sequence=command.sequence, accepted=True, reason="accepted",
                           timestamp_s=cmd_time_s, autopilot_mode=channel.mode,
                           failsafe=bool(channel.active_failures) or not channel.heartbeat_ok)

    def _send_mavlink_command(self, channel: _VehicleMavlinkChannel, command: SITLCommand) -> None:
        """Converts LOCAL_ENU -> LOCAL_NED (never silently - via the
        unmodified, already-tested `convert_command_frame`) and sends the
        real MAVLink message. Setpoint messages (SET_POSITION_TARGET_LOCAL_NED)
        have no MAVLink acknowledgement - this project treats the send
        call itself succeeding as "sent," a LOCALLY SYNTHESIZED
        acknowledgement, not a real MAVLink ack. MAV_CMD-style messages
        (HOLD/RTL/LAND/ABORT mode changes) DO wait for a real COMMAND_ACK,
        bounded by `ack_timeout_s` - see docs/PHASE8_ARDUPILOT_SITL.md's
        "Command acknowledgement mapping" section for this real,
        documented asymmetry."""
        endpoint = channel.endpoint
        underlying_command = command.adapter_command.command
        target_frame = Frame.LOCAL_NED
        if underlying_command.frame != target_frame:
            underlying_command = convert_command_frame(underlying_command, target_frame,
                                                         vehicle_attitude_rad=channel.attitude_rad)

        ctype = underlying_command.command_type
        conn = channel.connection
        if ctype == CommandType.HOLD:
            self._send_mode_change_and_wait_ack(channel, "LOITER")
            channel.set_mode(AutopilotMode.HOLD, at_s=command.timestamp_s)
        elif ctype == CommandType.LAND:
            self._send_command_long_and_wait_ack(channel, mavutil.mavlink.MAV_CMD_NAV_LAND)
            channel.set_mode(AutopilotMode.LAND, at_s=command.timestamp_s)
        elif ctype == CommandType.ABORT:
            self._send_command_long_and_wait_ack(channel, mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION, param1=1)
            channel.set_mode(AutopilotMode.ABORT, at_s=command.timestamp_s)
        elif ctype == CommandType.POSITION_SETPOINT:
            x, y, z = underlying_command.desired_position_m
            conn.mav.set_position_target_local_ned_send(
                0, endpoint.system_id, endpoint.component_id, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                _TYPEMASK_IGNORE_VX_VY_VZ_AFX_AFY_AFZ_YAW_YAWRATE, x, y, z, 0, 0, 0, 0, 0, 0, 0, 0,
            )
            channel.set_mode(AutopilotMode.OFFBOARD, at_s=command.timestamp_s)
        elif ctype == CommandType.VELOCITY_SETPOINT:
            vx, vy, vz = underlying_command.desired_velocity_mps
            conn.mav.set_position_target_local_ned_send(
                0, endpoint.system_id, endpoint.component_id, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                _TYPEMASK_IGNORE_PX_PY_PZ_AFX_AFY_AFZ_YAW_YAWRATE, 0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0,
            )
            channel.set_mode(AutopilotMode.OFFBOARD, at_s=command.timestamp_s)
        elif ctype in (CommandType.YAW_SETPOINT, CommandType.YAW_RATE_SETPOINT):
            # Documented simplification (module docstring): held in place
            # (zero velocity target) while yaw/yaw_rate is applied -
            # ArduPilot's SET_POSITION_TARGET_LOCAL_NED has no
            # "yaw-only, ignore everything else" mode.
            yaw = underlying_command.yaw_rad if underlying_command.yaw_rad is not None else 0.0
            yaw_rate = underlying_command.yaw_rate_radps if underlying_command.yaw_rate_radps is not None else 0.0
            conn.mav.set_position_target_local_ned_send(
                0, endpoint.system_id, endpoint.component_id, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                _TYPEMASK_IGNORE_PX_PY_PZ_AFX_AFY_AFZ, 0, 0, 0, 0, 0, 0, 0, 0, 0, yaw, yaw_rate,
            )
            channel.set_mode(AutopilotMode.OFFBOARD, at_s=command.timestamp_s)
        else:
            raise ArduPilotTransportError(f"unsupported command_type {ctype!r}")

    def _send_mode_change_and_wait_ack(self, channel: _VehicleMavlinkChannel, mode_name: str) -> None:
        mode_number = {v: k for k, v in mavutil.mode_mapping_acm.items()}.get(mode_name)
        if mode_number is None:
            raise ArduPilotTransportError(f"unknown ArduCopter mode name {mode_name!r}")
        self._send_command_long_and_wait_ack(
            channel, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            param1=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, param2=mode_number,
        )

    def _send_command_long_and_wait_ack(self, channel: _VehicleMavlinkChannel, command_id: int,
                                          param1: float = 0, param2: float = 0, param3: float = 0,
                                          param4: float = 0, param5: float = 0, param6: float = 0,
                                          param7: float = 0) -> None:
        endpoint = channel.endpoint
        channel.connection.mav.command_long_send(
            endpoint.system_id, endpoint.component_id, command_id, 0,
            param1, param2, param3, param4, param5, param6, param7,
        )
        deadline = time.monotonic() + self.ack_timeout_s
        while time.monotonic() < deadline:
            # Non-blocking + explicit sleep, not blocking=True - see the
            # matching comment in _start_channel's heartbeat-wait loop for
            # why a blocking pymavlink call can busy-spin its entire
            # timeout window once the connection has seen EOF.
            msg = channel.connection.recv_match(type="COMMAND_ACK", blocking=False)
            if msg is None:
                time.sleep(0.05)
                continue
            if msg.get_srcSystem() != endpoint.system_id or msg.get_srcComponent() != endpoint.component_id:
                continue
            if msg.command != command_id:
                continue
            if msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                raise ArduPilotTransportError(
                    f"ArduPilot rejected command {command_id} with MAV_RESULT={msg.result}"
                )
            return
        raise ArduPilotTransportError(
            f"timed out waiting {self.ack_timeout_s}s for a COMMAND_ACK to command {command_id}"
        )

    def receive_ack(self, vehicle_id: str) -> CommandAck:
        channel = self._channel(vehicle_id)
        if channel.last_ack is None:
            return CommandAck(vehicle_id=vehicle_id, namespace=channel.namespace, command_sequence=None,
                               accepted=False, reason="no_ack_available", timestamp_s=self.clock.now_s,
                               autopilot_mode=channel.mode, failsafe=bool(channel.active_failures))
        return channel.last_ack


def _same_command_content(a: SITLCommand, b: Optional[SITLCommand]) -> bool:
    if b is None:
        return False
    return (a.vehicle_id == b.vehicle_id and a.command_type == b.command_type and a.frame == b.frame
            and a.desired_position_m == b.desired_position_m and a.desired_velocity_mps == b.desired_velocity_mps
            and a.yaw_rad == b.yaw_rad and a.yaw_rate_radps == b.yaw_rate_radps
            and a.timestamp_s == b.timestamp_s and a.expiration_time_s == b.expiration_time_s)


def _yaw_ned_to_enu(yaw_rad: float) -> float:
    """Same formula as `autopilot/frames.py`'s own private
    `_yaw_ned_to_enu` (an involution: `pi/2 - yaw`) - restated here rather
    than importing a private name, so `autopilot/frames.py` itself stays
    completely unmodified this phase. See that module's docstring for the
    derivation; this project reuses it, not reinvents it."""
    return (math.pi / 2.0) - yaw_rad


def _autopilot_mode_from_ardupilot_mode_name(mode_name: str) -> AutopilotMode:
    reverse = {"LOITER": AutopilotMode.HOLD, "RTL": AutopilotMode.RTL, "LAND": AutopilotMode.LAND,
               "GUIDED": AutopilotMode.OFFBOARD}
    return reverse.get(mode_name, AutopilotMode.UNKNOWN)
