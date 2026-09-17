"""Phase 7: FakeSITLTransport - deterministic, in-memory, multi-vehicle
`SITLTransport` implementation - see docs/PHASE7_SITL_INTEGRATION.md.

No sockets, no serial, no MAVLink/MAVSDK library of any kind, no
wall-clock dependence (every "now" comes from `swarm_sim.sitl.clock.SimClock`,
advanced or set explicitly by the caller). This is a single process-local
object that hosts MANY vehicles at once, each isolated in its own
`_VehicleChannel` keyed by vehicle_id/namespace - deliberately not one
instance per vehicle, because "commands cannot cross vehicle namespaces"
and "failures injected into one vehicle do not alter another" (Phase 7's
own required tests) are only meaningful guarantees against a transport
that actually holds multiple vehicles' state at once and could, if built
carelessly, let them leak into each other.

Two kinds of vehicle motion are supported, and they are never mixed for a
single vehicle in one run:

- `step(dt_s)` - a simplified, fixed-step *kinematic* simulation (plain
  Euler integration of the last accepted VELOCITY_SETPOINT; nothing else
  is modeled) for the standalone scenarios that have no real physics
  engine behind them (see scripts/run_phase7_sitl_scenarios.py's synthetic
  scenarios). This is explicitly NOT a flight-dynamics model: no attitude
  response, no motor/propeller/airframe dynamics, no controller
  stabilization loop - see docs/PHASE7_SITL_INTEGRATION.md's "fake-SITL
  limitations" section.
- `push_vehicle_state(...)` - for the PyBullet-integrated path
  (swarm_sim.autopilot.sitl.SITLAdapter.push_ground_truth_state, used by
  mission.py exactly like Phase 6's MockAdapter.push_ground_truth_state):
  PyBullet is the real physics in that case, so this transport's own
  kinematic integrator is bypassed entirely and the pushed ground truth is
  what gets reported back as telemetry.

Command/telemetry/acknowledgement channel effects (latency, packet loss)
are simulated with plain Python arithmetic and an explicitly-seeded
`random.Random` - never a real socket/serial/radio.
"""
from __future__ import annotations

import random
from typing import Dict, List, Optional, Sequence, Tuple

from ..autopilot.types import AutopilotMode
from ..contracts import CommandType, Frame
from .clock import SimClock
from .commands import SITLCommand
from .telemetry import CommandAck, FailureType, SITLTelemetry, TransportResult
from .vehicle_namespace import NamespaceError, VehicleNamespaceRegistry

_NAVIGATION_COMMAND_TYPES = frozenset({
    CommandType.POSITION_SETPOINT, CommandType.VELOCITY_SETPOINT,
    CommandType.YAW_SETPOINT, CommandType.YAW_RATE_SETPOINT,
})
_FAILSAFE_FAILURES = frozenset({
    FailureType.HEARTBEAT_LOSS, FailureType.ESTIMATOR_INVALID,
    FailureType.BATTERY_CRITICAL, FailureType.VEHICLE_DISCONNECT,
})
_NAVIGATION_BLOCKING_REASON = {
    FailureType.HEARTBEAT_LOSS: "heartbeat_lost",
    FailureType.ESTIMATOR_INVALID: "estimator_invalid",
    FailureType.BATTERY_CRITICAL: "battery_critical",
}


class _VehicleChannel:
    """All mutable state for exactly one vehicle. Never referenced from
    another vehicle's code path - every FakeSITLTransport method looks
    this up strictly by vehicle_id/namespace."""

    def __init__(self, vehicle_id: str, namespace: str, operating_frame: Frame,
                 initial_position_m: Tuple[float, float, float]):
        self.vehicle_id = vehicle_id
        self.namespace = namespace
        self.operating_frame = operating_frame

        self.mode = AutopilotMode.UNKNOWN
        self.armed = False   # never set True anywhere in this class - no implicit arm, ever
        self.position_m = initial_position_m
        self.velocity_mps: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.attitude_rad: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.angular_velocity_radps: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.battery_fraction: float = 1.0
        self.estimator_valid = True
        self._last_state_update_at_s: Optional[float] = None

        self.active_failures: set = set()
        self._last_accepted_sequence: Optional[int] = None
        self._last_accepted_command: Optional[SITLCommand] = None

        self._telemetry_sequence = 0
        self._pending_telemetry: List[Tuple[float, SITLTelemetry]] = []   # (available_at_s, telemetry)
        self._last_delivered_telemetry: Optional[SITLTelemetry] = None

        self.last_ack: Optional[CommandAck] = None
        self._last_ack_produced_at_s: Optional[float] = None
        self._last_command_sent_at_s: Optional[float] = None

        # Public, append-only histories - required by Phase 7 item 4
        # ("stores all commands, telemetry, acknowledgements, and
        # failures").
        self.command_history: List[Tuple[SITLCommand, CommandAck]] = []
        self.telemetry_history: List[SITLTelemetry] = []
        self.ack_history: List[CommandAck] = []
        self.failure_log: List[Tuple[float, FailureType, bool]] = []
        self.mode_transition_log: List[Tuple[float, AutopilotMode]] = []

    def _set_mode(self, mode: AutopilotMode, at_s: float) -> None:
        if mode != self.mode:
            self.mode_transition_log.append((at_s, mode))
        self.mode = mode


class FakeSITLTransport:
    """Deterministic, in-memory `SITLTransport` hosting one or more
    vehicles. See module docstring for the two motion models
    (`step`/`push_vehicle_state`) and docs/PHASE7_SITL_INTEGRATION.md for
    the full behavioral spec."""

    def __init__(self, vehicle_ids: Sequence[str], operating_frame: Frame = Frame.LOCAL_ENU,
                 command_latency_s: float = 0.0, telemetry_latency_s: float = 0.0,
                 command_packet_loss_prob: float = 0.0, telemetry_packet_loss_prob: float = 0.0,
                 ack_timeout_s: float = 5.0, future_tolerance_s: float = 0.05,
                 telemetry_stale_timeout_s: float = 1.0,
                 battery_drain_per_s: float = 0.0, battery_critical_threshold: float = 0.08,
                 initial_positions: Optional[Dict[str, Tuple[float, float, float]]] = None,
                 rng: Optional[random.Random] = None, clock: Optional[SimClock] = None):
        self.registry = VehicleNamespaceRegistry()
        self.operating_frame = operating_frame
        self.command_latency_s = command_latency_s
        self.telemetry_latency_s = telemetry_latency_s
        self.command_packet_loss_prob = command_packet_loss_prob
        self.telemetry_packet_loss_prob = telemetry_packet_loss_prob
        self.ack_timeout_s = ack_timeout_s
        self.telemetry_stale_timeout_s = telemetry_stale_timeout_s
        self.battery_drain_per_s = battery_drain_per_s
        self.battery_critical_threshold = battery_critical_threshold
        self.rng = rng if rng is not None else random.Random(0)
        self.clock = clock if clock is not None else SimClock(future_tolerance_s=future_tolerance_s)

        self._running = False
        self._ever_started = False
        initial_positions = initial_positions or {}
        self._channels: Dict[str, _VehicleChannel] = {}
        for vehicle_id in vehicle_ids:
            namespace = self.registry.register(vehicle_id)   # raises NamespaceError on a duplicate vehicle_id
            self._channels[vehicle_id] = _VehicleChannel(
                vehicle_id, namespace, operating_frame,
                initial_positions.get(vehicle_id, (0.0, 0.0, 0.0)),
            )

        self.start_stop_log: List[Tuple[float, bool]] = []   # (at_s, running)

    # -- internal lookups -----------------------------------------------------

    def _channel(self, vehicle_id: str) -> _VehicleChannel:
        if vehicle_id not in self._channels:
            raise NamespaceError(f"vehicle_id {vehicle_id!r} is not registered on this transport")
        return self._channels[vehicle_id]

    @property
    def is_running(self) -> bool:
        return self._running

    def total_command_history_count(self) -> int:
        """Sum of every vehicle's own command_history length - public
        summary accessor for scripts/mission.py, so callers outside this
        module never need to reach into a per-vehicle _VehicleChannel
        directly."""
        return sum(len(c.command_history) for c in self._channels.values())

    def total_failure_log_count(self) -> int:
        return sum(len(c.failure_log) for c in self._channels.values())

    # -- transport lifecycle ---------------------------------------------------

    def start(self) -> TransportResult:
        self._running = True
        self._ever_started = True
        self.start_stop_log.append((self.clock.now_s, True))
        return TransportResult(success=True, reason="started", timestamp_s=self.clock.now_s)

    def stop(self) -> TransportResult:
        self._running = False
        self.start_stop_log.append((self.clock.now_s, False))
        return TransportResult(success=True, reason="stopped", timestamp_s=self.clock.now_s)

    def set_sim_time(self, now_s: float) -> None:
        self.clock.set(now_s)

    # -- failure injection -------------------------------------------------

    def inject_failure(self, vehicle_id: str, failure: FailureType, active: bool = True) -> None:
        channel = self._channel(vehicle_id)
        if not isinstance(failure, FailureType):
            raise ValueError(f"failure must be a FailureType, got {failure!r}")
        if active:
            channel.active_failures.add(failure)
        else:
            channel.active_failures.discard(failure)
        channel.failure_log.append((self.clock.now_s, failure, active))

    # -- commands -----------------------------------------------------------

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
        channel._last_command_sent_at_s = now_s
        ack = self._evaluate_command(channel, command)
        channel.last_ack = ack
        channel._last_ack_produced_at_s = now_s
        channel.command_history.append((command, ack))
        channel.ack_history.append(ack)
        return TransportResult(success=True, reason="queued", timestamp_s=now_s)

    def _evaluate_command(self, channel: _VehicleChannel, command: SITLCommand) -> CommandAck:
        cmd_time_s = command.timestamp_s
        is_navigation = command.command_type in _NAVIGATION_COMMAND_TYPES

        def reject(reason: str) -> CommandAck:
            return CommandAck(
                vehicle_id=command.vehicle_id, namespace=command.namespace, command_sequence=command.sequence,
                accepted=False, reason=reason, timestamp_s=cmd_time_s, autopilot_mode=channel.mode,
                failsafe=bool(channel.active_failures & _FAILSAFE_FAILURES),
            )

        if self.clock.is_from_the_future(cmd_time_s):
            return reject("command_from_the_future")

        if FailureType.VEHICLE_DISCONNECT in channel.active_failures:
            return reject("disconnected")

        if command.frame != channel.operating_frame:
            return reject("frame_mismatch")

        effective_arrival_s = cmd_time_s + self.command_latency_s
        if not (cmd_time_s <= effective_arrival_s <= command.expiration_time_s):
            return reject("expired")

        for failure, reason in _NAVIGATION_BLOCKING_REASON.items():
            if is_navigation and failure in channel.active_failures:
                channel._set_mode(AutopilotMode.HOLD, at_s=cmd_time_s)
                return reject(reason)

        if is_navigation and (channel._last_state_update_at_s is None
                               or (cmd_time_s - channel._last_state_update_at_s) > self.telemetry_stale_timeout_s):
            channel._set_mode(AutopilotMode.HOLD, at_s=cmd_time_s)
            return reject("stale_telemetry")

        if channel._last_accepted_sequence is not None:
            if command.sequence == channel._last_accepted_sequence:
                if self._same_content(command, channel._last_accepted_command):
                    prior = channel.last_ack
                    if prior is not None:
                        return prior
                else:
                    return reject("sequence_replayed_conflicting")
            elif command.sequence < channel._last_accepted_sequence:
                return reject("sequence_replayed_stale")

        if (FailureType.COMMAND_PACKET_LOSS in channel.active_failures
                or self.rng.random() < self.command_packet_loss_prob):
            return reject("command_packet_loss")

        self._apply_accepted_command(channel, command, cmd_time_s)
        channel._last_accepted_sequence = command.sequence
        channel._last_accepted_command = command
        return CommandAck(
            vehicle_id=command.vehicle_id, namespace=command.namespace, command_sequence=command.sequence,
            accepted=True, reason="accepted", timestamp_s=cmd_time_s, autopilot_mode=channel.mode,
            failsafe=bool(channel.active_failures & _FAILSAFE_FAILURES),
        )

    @staticmethod
    def _same_content(a: SITLCommand, b: Optional[SITLCommand]) -> bool:
        if b is None:
            return False
        return (a.vehicle_id == b.vehicle_id and a.command_type == b.command_type and a.frame == b.frame
                and a.desired_position_m == b.desired_position_m and a.desired_velocity_mps == b.desired_velocity_mps
                and a.yaw_rad == b.yaw_rad and a.yaw_rate_radps == b.yaw_rate_radps
                and a.timestamp_s == b.timestamp_s and a.expiration_time_s == b.expiration_time_s)

    def _apply_accepted_command(self, channel: _VehicleChannel, command: SITLCommand, at_s: float) -> None:
        ctype = command.command_type
        if ctype == CommandType.ABORT:
            channel._set_mode(AutopilotMode.ABORT, at_s=at_s)
            channel.velocity_mps = (0.0, 0.0, 0.0)
        elif ctype == CommandType.LAND:
            channel._set_mode(AutopilotMode.LAND, at_s=at_s)
            channel.velocity_mps = (0.0, 0.0, 0.0)
        elif ctype == CommandType.HOLD:
            channel._set_mode(AutopilotMode.HOLD, at_s=at_s)
            channel.velocity_mps = (0.0, 0.0, 0.0)
        elif ctype == CommandType.VELOCITY_SETPOINT:
            channel._set_mode(AutopilotMode.OFFBOARD, at_s=at_s)
            channel.velocity_mps = command.desired_velocity_mps
        else:
            channel._set_mode(AutopilotMode.OFFBOARD, at_s=at_s)

    def receive_ack(self, vehicle_id: str) -> CommandAck:
        channel = self._channel(vehicle_id)
        if channel.last_ack is None:
            return CommandAck(vehicle_id=vehicle_id, namespace=channel.namespace, command_sequence=None,
                               accepted=False, reason="no_ack_available", timestamp_s=self.clock.now_s,
                               autopilot_mode=channel.mode, failsafe=bool(channel.active_failures & _FAILSAFE_FAILURES))
        if (channel._last_ack_produced_at_s is not None
                and self.clock.now_s - channel._last_ack_produced_at_s > self.ack_timeout_s):
            return CommandAck(vehicle_id=vehicle_id, namespace=channel.namespace,
                               command_sequence=channel.last_ack.command_sequence, accepted=False,
                               reason="acknowledgement_timeout", timestamp_s=self.clock.now_s,
                               autopilot_mode=channel.mode, failsafe=bool(channel.active_failures & _FAILSAFE_FAILURES))
        return channel.last_ack

    # -- telemetry --------------------------------------------------------

    def _emit_telemetry(self, channel: _VehicleChannel) -> None:
        now_s = self.clock.now_s
        channel._last_state_update_at_s = now_s
        channel._telemetry_sequence += 1
        last_ack_status = None
        if channel.last_ack is not None:
            last_ack_status = "accepted" if channel.last_ack.accepted else f"rejected:{channel.last_ack.reason}"
        telemetry = SITLTelemetry(
            vehicle_id=channel.vehicle_id, namespace=channel.namespace, timestamp_s=now_s,
            sequence=channel._telemetry_sequence, position_m=channel.position_m, velocity_mps=channel.velocity_mps,
            attitude_rad=channel.attitude_rad, angular_velocity_radps=channel.angular_velocity_radps,
            battery_fraction=channel.battery_fraction,
            estimator_valid=channel.estimator_valid and FailureType.ESTIMATOR_INVALID not in channel.active_failures,
            armed=channel.armed, flight_mode=channel.mode,
            failsafe=bool(channel.active_failures & _FAILSAFE_FAILURES),
            heartbeat_ok=FailureType.HEARTBEAT_LOSS not in channel.active_failures,
            last_command_ack_status=last_ack_status,
        )
        channel.telemetry_history.append(telemetry)
        if (FailureType.TELEMETRY_PACKET_LOSS in channel.active_failures
                or self.rng.random() < self.telemetry_packet_loss_prob):
            return
        channel._pending_telemetry.append((now_s + self.telemetry_latency_s, telemetry))

    def receive_telemetry(self, vehicle_id: str) -> SITLTelemetry:
        channel = self._channel(vehicle_id)
        now_s = self.clock.now_s
        deliverable = [t for (at_s, t) in channel._pending_telemetry if at_s <= now_s]
        if deliverable:
            channel._last_delivered_telemetry = deliverable[-1]
            channel._pending_telemetry = [(at_s, t) for (at_s, t) in channel._pending_telemetry if at_s > now_s]
        if channel._last_delivered_telemetry is not None:
            return channel._last_delivered_telemetry
        return SITLTelemetry(
            vehicle_id=vehicle_id, namespace=channel.namespace, timestamp_s=now_s, sequence=0,
            position_m=None, velocity_mps=None, attitude_rad=None, angular_velocity_radps=None,
            battery_fraction=None, estimator_valid=False, armed=False, flight_mode=AutopilotMode.UNKNOWN,
            failsafe=bool(channel.active_failures & _FAILSAFE_FAILURES), heartbeat_ok=False,
            last_command_ack_status=None,
        )

    # -- simplified kinematics / mission-integration hooks ------------------

    def step(self, dt_s: float) -> None:
        """Fixed-step mode: advance the shared clock and, for every
        vehicle, integrate simplified kinematics (Euler position update
        from the last accepted VELOCITY_SETPOINT only - see module
        docstring) and drain battery, then emit this tick's telemetry.
        Used only by the standalone scenarios; the PyBullet-integrated
        path uses `push_vehicle_state` instead (see module docstring)."""
        self.clock.advance(dt_s)
        for channel in self._channels.values():
            px, py, pz = channel.position_m
            vx, vy, vz = channel.velocity_mps
            channel.position_m = (px + vx * dt_s, py + vy * dt_s, pz + vz * dt_s)
            if self.battery_drain_per_s > 0.0:
                channel.battery_fraction = max(0.0, channel.battery_fraction - self.battery_drain_per_s * dt_s)
                if (channel.battery_fraction <= self.battery_critical_threshold
                        and FailureType.BATTERY_CRITICAL not in channel.active_failures):
                    self.inject_failure(channel.vehicle_id, FailureType.BATTERY_CRITICAL, active=True)
            self._emit_telemetry(channel)

    def push_vehicle_state(self, vehicle_id: str, position_m, velocity_mps, attitude_rad, angular_velocity_radps,
                            battery_fraction, estimator_valid: bool, now_s: float) -> None:
        """Mission-integration hook (NOT part of SITLTransport): overwrite
        a vehicle's ground-truth state directly from a real physics engine
        (PyBullet, via mission.py/SITLAdapter) and emit telemetry from it -
        bypassing `step`'s own simplified integrator entirely, exactly like
        Phase 6's MockAdapter.push_ground_truth_state. Advances the shared
        clock to `now_s` itself (via `set_sim_time`), so a caller in this
        mode never needs to call `set_sim_time` separately."""
        if now_s > self.clock.now_s:
            self.set_sim_time(now_s)
        channel = self._channel(vehicle_id)
        channel.position_m = position_m
        channel.velocity_mps = velocity_mps
        channel.attitude_rad = attitude_rad
        channel.angular_velocity_radps = angular_velocity_radps
        channel.battery_fraction = battery_fraction
        channel.estimator_valid = estimator_valid
        self._emit_telemetry(channel)
