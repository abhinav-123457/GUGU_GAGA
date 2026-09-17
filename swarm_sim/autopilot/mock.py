"""Phase 6: MockAdapter - the deterministic, in-memory reference
AutopilotAdapter implementation. See docs/PHASE6_AUTOPILOT_ADAPTERS.md.

No network or hardware access of any kind - every "channel" effect
(command latency, telemetry latency, packet loss) is simulated with plain
Python arithmetic and an explicitly-seeded `random.Random`, never a real
socket/serial/radio. This is the architectural reference path Phase 6
asks for: everything a real ArduPilot/PX4 transport would eventually have
to do (validate frame/freshness/sequence, gate on connection/failsafe
state, refuse to silently convert frames) is fully implemented here
against a simulated vehicle, so the exact same validation rules apply
whether the swarm targets PyBullet today or a real transport later - only
the transport itself would ever need to change.

Clock model: this adapter has no wall-clock dependency. "Now," for every
call, is derived from the caller-supplied data - `send_command` uses the
command's own `timestamp_s`, `push_ground_truth_state` uses the pushed
telemetry's own `timestamp_s`. This keeps the whole adapter deterministic
under a fixed sequence of calls, matching this project's simulation-time
conventions elsewhere (e.g. `SafetySupervisor.evaluate(..., now_s=...)`).
"""
from __future__ import annotations

import random
from typing import Dict, List, Optional, Tuple

from ..contracts import CommandType, Frame
from .base import is_command_fresh, is_navigation_command
from .types import AdapterCommand, AdapterResult, AutopilotMode, ConnectionState, VehicleTelemetry

_LANDING_LIKE_STATES = (ConnectionState.LANDING,)


class MockAdapter:
    """Deterministic, in-memory `AutopilotAdapter`. See module docstring
    and docs/PHASE6_AUTOPILOT_ADAPTERS.md's "mock adapter limitations"
    section for what this deliberately does NOT model (motor dynamics,
    real radio propagation, MAVLink message framing, ...)."""

    def __init__(self, vehicle_id: str, operating_frame: Frame = Frame.LOCAL_ENU,
                 command_latency_s: float = 0.0, telemetry_latency_s: float = 0.0,
                 command_packet_loss_prob: float = 0.0, telemetry_packet_loss_prob: float = 0.0,
                 telemetry_stale_timeout_s: float = 1.0, rng: Optional[random.Random] = None):
        self.vehicle_id = vehicle_id
        self.operating_frame = operating_frame
        self.command_latency_s = command_latency_s
        self.telemetry_latency_s = telemetry_latency_s
        self.command_packet_loss_prob = command_packet_loss_prob
        self.telemetry_packet_loss_prob = telemetry_packet_loss_prob
        self.telemetry_stale_timeout_s = telemetry_stale_timeout_s
        self.rng = rng if rng is not None else random.Random(0)

        self.state = ConnectionState.DISCONNECTED
        self.mode = AutopilotMode.UNKNOWN
        self.armed = False
        self.failsafe = False

        self._last_accepted_sequence: Optional[int] = None
        self._last_accepted_command: Optional[AdapterCommand] = None
        self._last_accepted_result: Optional[AdapterResult] = None

        self._last_delivered_telemetry: Optional[VehicleTelemetry] = None
        self._last_delivered_telemetry_at_s: Optional[float] = None
        self._telemetry_sequence = 0

        # Full accept/reject history - (AdapterCommand, AdapterResult) pairs,
        # in call order. Public and append-only; nothing else in this class
        # ever removes an entry.
        self.command_history: List[Tuple[AdapterCommand, AdapterResult]] = []
        self.mode_transition_log: List[Tuple[float, AutopilotMode]] = []
        self.connection_transition_log: List[Tuple[float, ConnectionState]] = []

    # -- connection lifecycle -------------------------------------------------

    def connect(self) -> AdapterResult:
        self._set_state(ConnectionState.CONNECTED, at_s=0.0)
        self.mode = AutopilotMode.HOLD
        return AdapterResult(accepted=True, reason="connected", timestamp_s=0.0,
                              command_sequence=None, adapter_state=self.state)

    def disconnect(self) -> AdapterResult:
        self._set_state(ConnectionState.DISCONNECTED, at_s=0.0)
        return AdapterResult(accepted=True, reason="disconnected", timestamp_s=0.0,
                              command_sequence=None, adapter_state=self.state)

    # -- test/scenario injection hooks (NOT part of AutopilotAdapter) -------
    # These simulate conditions a real transport would report on its own
    # (a dropped radio link, a vehicle-initiated failsafe) - a mock has to
    # be told about them explicitly since it has no real link to lose.

    def inject_disconnect(self, at_s: float = 0.0) -> None:
        self._set_state(ConnectionState.DISCONNECTED, at_s=at_s)

    def inject_failsafe(self, active: bool, at_s: float = 0.0) -> None:
        self.failsafe = active
        self._set_state(ConnectionState.FAILSAFE if active else ConnectionState.CONNECTED, at_s=at_s)

    def _set_state(self, state: ConnectionState, at_s: float) -> None:
        if state != self.state:
            self.connection_transition_log.append((at_s, state))
        self.state = state

    def _set_mode_internal(self, mode: AutopilotMode, at_s: float) -> None:
        if mode != self.mode:
            self.mode_transition_log.append((at_s, mode))
        self.mode = mode

    # -- telemetry --------------------------------------------------------

    def push_ground_truth_state(self, telemetry: VehicleTelemetry) -> None:
        """Test/mission-harness hook (NOT part of AutopilotAdapter): tell
        this mock what the simulated vehicle's true state is right now -
        the mock equivalent of a real adapter's own telemetry stream.
        Subject to `telemetry_packet_loss_prob` (silently dropped, exactly
        like a lost radio packet - `read_telemetry` then keeps returning
        the last successfully delivered state) and
        `telemetry_latency_s` (delivered only once
        `telemetry.timestamp_s - self._last_delivered_telemetry_at_s >= telemetry_latency_s`,
        i.e. a later push can "catch up" and deliver once the gap since
        the last delivery is at least the configured latency - mirrors
        `CommsNetwork`'s own step-based latency model)."""
        if self.rng.random() < self.telemetry_packet_loss_prob:
            return
        if (self._last_delivered_telemetry_at_s is None
                or telemetry.timestamp_s - self._last_delivered_telemetry_at_s >= self.telemetry_latency_s):
            self._telemetry_sequence += 1
            self._last_delivered_telemetry = telemetry
            self._last_delivered_telemetry_at_s = telemetry.timestamp_s

    def read_telemetry(self) -> VehicleTelemetry:
        if self._last_delivered_telemetry is None:
            return VehicleTelemetry(
                vehicle_id=self.vehicle_id, timestamp_s=0.0, frame=self.operating_frame,
                position_m=None, velocity_mps=None, acceleration_mps2=None, attitude_rad=None,
                angular_velocity_radps=None, battery_fraction=None, estimator_valid=False,
                connection_state=self.state, autopilot_mode=self.mode, armed=self.armed,
                failsafe=self.failsafe, sequence=self._telemetry_sequence,
            )
        return self._last_delivered_telemetry

    def _telemetry_is_stale(self, now_s: float) -> bool:
        if self._last_delivered_telemetry_at_s is None:
            return True
        return now_s - self._last_delivered_telemetry_at_s > self.telemetry_stale_timeout_s

    # -- commands -----------------------------------------------------------

    def send_command(self, command: AdapterCommand) -> AdapterResult:
        if not isinstance(command, AdapterCommand):
            raise TypeError(f"send_command requires an AdapterCommand, got {type(command)!r}")

        now_s = command.command.timestamp_s

        def reject(reason: str) -> AdapterResult:
            result = AdapterResult(accepted=False, reason=reason, timestamp_s=now_s,
                                     command_sequence=command.sequence, adapter_state=self.state)
            self.command_history.append((command, result))
            return result

        if command.vehicle_id != self.vehicle_id:
            return reject("wrong_vehicle_id")

        if self.state in (ConnectionState.DISCONNECTED, ConnectionState.CONNECTING, ConnectionState.UNKNOWN):
            return reject("disconnected")

        if self.failsafe or self.state == ConnectionState.FAILSAFE:
            return reject("failsafe_active")

        if command.command.frame != self.operating_frame:
            return reject("frame_mismatch")

        effective_arrival_s = command.command.timestamp_s + self.command_latency_s
        if not (command.command.timestamp_s <= effective_arrival_s <= command.command.expiration_time_s):
            return reject("expired")

        if is_navigation_command(command) and self._telemetry_is_stale(now_s):
            self._set_mode_internal(AutopilotMode.HOLD, at_s=now_s)
            return reject("stale_telemetry")

        # Sequence handling: strictly monotonic per vehicle, with exact
        # idempotent replay of the LAST accepted sequence (same content)
        # tolerated - see types.py/module docstring and
        # docs/PHASE6_AUTOPILOT_ADAPTERS.md's "sequence-number semantics".
        if self._last_accepted_sequence is not None:
            if command.sequence == self._last_accepted_sequence:
                if self._same_command_content(command, self._last_accepted_command):
                    result = self._last_accepted_result
                    self.command_history.append((command, result))
                    return result
                return reject("sequence_replayed_conflicting")
            if command.sequence < self._last_accepted_sequence:
                return reject("sequence_replayed_stale")

        if self.rng.random() < self.command_packet_loss_prob:
            return reject("packet_loss")

        self._apply_command_mode(command, now_s)
        result = AdapterResult(accepted=True, reason="accepted", timestamp_s=now_s,
                                 command_sequence=command.sequence, adapter_state=self.state,
                                 transformed_command=command)
        self._last_accepted_sequence = command.sequence
        self._last_accepted_command = command
        self._last_accepted_result = result
        self.command_history.append((command, result))
        return result

    @staticmethod
    def _same_command_content(a: AdapterCommand, b: Optional[AdapterCommand]) -> bool:
        if b is None:
            return False
        return (a.command.vehicle_id == b.command.vehicle_id
                and a.command.command_type == b.command.command_type
                and a.command.frame == b.command.frame
                and a.command.desired_position_m == b.command.desired_position_m
                and a.command.desired_velocity_mps == b.command.desired_velocity_mps
                and a.command.yaw_rad == b.command.yaw_rad
                and a.command.yaw_rate_radps == b.command.yaw_rate_radps
                and a.command.timestamp_s == b.command.timestamp_s
                and a.command.expiration_time_s == b.command.expiration_time_s)

    def _apply_command_mode(self, command: AdapterCommand, now_s: float) -> None:
        ctype = command.command.command_type
        if ctype == CommandType.ABORT:
            self._set_mode_internal(AutopilotMode.ABORT, at_s=now_s)
        elif ctype == CommandType.LAND:
            self._set_mode_internal(AutopilotMode.LAND, at_s=now_s)
            self._set_state(ConnectionState.LANDING, at_s=now_s)
        elif ctype == CommandType.HOLD:
            self._set_mode_internal(AutopilotMode.HOLD, at_s=now_s)
        else:
            self._set_mode_internal(AutopilotMode.OFFBOARD, at_s=now_s)

    # -- mode requests --------------------------------------------------------

    def set_mode(self, mode: AutopilotMode, now_s: float = 0.0) -> AdapterResult:
        """`now_s` is an extra keyword beyond the `AutopilotAdapter`
        Protocol's own `set_mode(self, mode)` signature - purely so
        callers with a real simulation clock (mission.py, tests) can get
        an informative `AdapterResult.timestamp_s`/transition-log entry;
        it defaults to 0.0 (this adapter's fixed "no clock" placeholder,
        see module docstring) so Protocol-only callers still work."""
        if self.state in (ConnectionState.DISCONNECTED, ConnectionState.CONNECTING, ConnectionState.UNKNOWN):
            return AdapterResult(accepted=False, reason="disconnected", timestamp_s=now_s,
                                  command_sequence=None, adapter_state=self.state)
        self._set_mode_internal(mode, at_s=now_s)
        if mode == AutopilotMode.LAND:
            self._set_state(ConnectionState.LANDING, at_s=now_s)
        return AdapterResult(accepted=True, reason=f"mode_set_{mode.value}", timestamp_s=now_s,
                              command_sequence=None, adapter_state=self.state)

    def request_land(self, now_s: float = 0.0) -> AdapterResult:
        return self.set_mode(AutopilotMode.LAND, now_s=now_s)

    def request_return_to_launch(self, now_s: float = 0.0) -> AdapterResult:
        return self.set_mode(AutopilotMode.RTL, now_s=now_s)

    def abort(self, now_s: float = 0.0) -> AdapterResult:
        return self.set_mode(AutopilotMode.ABORT, now_s=now_s)

    def is_command_fresh(self, command: AdapterCommand, now_s: float) -> bool:
        return is_command_fresh(command, now_s)
