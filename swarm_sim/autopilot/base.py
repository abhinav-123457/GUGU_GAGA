"""Phase 6: AutopilotAdapter interface - see docs/PHASE6_AUTOPILOT_ADAPTERS.md.

This module defines ONLY the interface contract, a couple of small pure
helpers shared by every adapter implementation, and `OfflineSkeletonAdapter`
(the common base for ardupilot.py/px4.py - see its own docstring). It
imports nothing from pybullet, pymavlink, MAVSDK, serial, or socket - see
tests/test_autopilot_architecture.py.
"""
from __future__ import annotations

from typing import Protocol, runtime_checkable

from .types import AdapterCommand, AdapterResult, AutopilotMode, ConnectionState, VehicleTelemetry


@runtime_checkable
class AutopilotAdapter(Protocol):
    """The only interface a candidate command can cross on its way to a
    vehicle (simulated or, in a future phase, real). Every implementation
    (MockAdapter, ArduPilotAdapterSkeleton, PX4AdapterSkeleton) must accept
    only `AdapterCommand` (which itself only wraps an already-validated
    `contracts.Command`) - see types.py's module docstring."""

    def connect(self) -> AdapterResult: ...

    def disconnect(self) -> AdapterResult: ...

    def read_telemetry(self) -> VehicleTelemetry: ...

    def send_command(self, command: AdapterCommand) -> AdapterResult: ...

    def set_mode(self, mode: AutopilotMode) -> AdapterResult: ...

    def request_land(self) -> AdapterResult: ...

    def request_return_to_launch(self) -> AdapterResult: ...

    def abort(self) -> AdapterResult: ...

    def is_command_fresh(self, command: AdapterCommand, now_s: float) -> bool: ...


def is_command_fresh(command: AdapterCommand, now_s: float) -> bool:
    """Shared, pure freshness check every adapter implementation's own
    `is_command_fresh` delegates to - a command is fresh iff `now_s` falls
    within [timestamp_s, expiration_time_s]. Never mutates anything; safe
    to call speculatively before send_command."""
    return command.command.timestamp_s <= now_s <= command.command.expiration_time_s


_NAVIGATION_COMMAND_TYPE_NAMES = frozenset({"POSITION_SETPOINT", "VELOCITY_SETPOINT", "YAW_SETPOINT", "YAW_RATE_SETPOINT"})


def is_navigation_command(command: AdapterCommand) -> bool:
    """True for setpoint-style commands (the ones stale telemetry should
    block); false for HOLD/LAND/ABORT, which are exactly the safe things
    to still allow through under degraded telemetry."""
    return command.command.command_type.name in _NAVIGATION_COMMAND_TYPE_NAMES


class OfflineSkeletonAdapter:
    """Common base for ArduPilotAdapterSkeleton and PX4AdapterSkeleton -
    see their own module docstrings and docs/PHASE6_AUTOPILOT_ADAPTERS.md's
    "ArduPilot/PX4 skeleton limitations" sections.

    THERE IS NO TRANSPORT HERE. `connect()` always fails (the adapter
    never leaves ConnectionState.DISCONNECTED), which makes every other
    method's "must be connected" check fail too - structurally, not by
    convention, this class can never send a command anywhere, arm
    anything, or produce any external side effect. It exists only to
    prove `AutopilotAdapter` can be implemented for a future ArduPilot/PX4
    transport without changing SwarmController or SafetySupervisor; wiring
    in a real transport (MAVLink, MAVSDK, serial, UDP) is explicitly a
    later, separately reviewed phase - see this class's PROTOCOL_NAME."""

    PROTOCOL_NAME = "offline-skeleton"

    # No transport means no clock source either - every rejection uses a
    # fixed, deterministic timestamp (0.0) rather than a wall-clock read,
    # except send_command's, which has a real deterministic time source
    # available (the rejected command's own timestamp_s).
    _NO_CLOCK_TIMESTAMP_S = 0.0

    def __init__(self, vehicle_id: str):
        self.vehicle_id = vehicle_id
        self.state = ConnectionState.DISCONNECTED
        self.mode = AutopilotMode.UNKNOWN

    def _rejected(self, reason: str, timestamp_s: float = _NO_CLOCK_TIMESTAMP_S) -> AdapterResult:
        return AdapterResult(
            accepted=False, reason=reason, timestamp_s=timestamp_s,
            command_sequence=None, adapter_state=self.state,
        )

    def connect(self) -> AdapterResult:
        return self._rejected(
            f"{self.PROTOCOL_NAME} transport not implemented - Phase 6 offline skeleton only, "
            "see docs/PHASE6_AUTOPILOT_ADAPTERS.md"
        )

    def disconnect(self) -> AdapterResult:
        self.state = ConnectionState.DISCONNECTED
        return AdapterResult(accepted=True, reason="already disconnected (no transport exists)",
                              timestamp_s=self._NO_CLOCK_TIMESTAMP_S, command_sequence=None, adapter_state=self.state)

    def read_telemetry(self) -> VehicleTelemetry:
        from ..contracts import Frame
        return VehicleTelemetry(
            vehicle_id=self.vehicle_id, timestamp_s=self._NO_CLOCK_TIMESTAMP_S, frame=Frame.LOCAL_ENU,
            position_m=None, velocity_mps=None, acceleration_mps2=None, attitude_rad=None,
            angular_velocity_radps=None, battery_fraction=None, estimator_valid=False,
            connection_state=self.state, autopilot_mode=self.mode, armed=False, failsafe=False, sequence=0,
        )

    def send_command(self, command: AdapterCommand) -> AdapterResult:
        return self._rejected("disconnected: no transport implemented in this offline skeleton",
                               timestamp_s=command.command.timestamp_s)

    def set_mode(self, mode: AutopilotMode) -> AdapterResult:
        return self._rejected("disconnected: no transport implemented in this offline skeleton")

    def request_land(self) -> AdapterResult:
        return self.set_mode(AutopilotMode.LAND)

    def request_return_to_launch(self) -> AdapterResult:
        return self.set_mode(AutopilotMode.RTL)

    def abort(self) -> AdapterResult:
        return self.set_mode(AutopilotMode.ABORT)

    def is_command_fresh(self, command: AdapterCommand, now_s: float) -> bool:
        return is_command_fresh(command, now_s)
