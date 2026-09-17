"""Phase 7: SITLAdapter - the AutopilotAdapter implementation that sits on
top of a swarm_sim.sitl.transport.SITLTransport (FakeSITLTransport this
phase). See docs/PHASE7_SITL_INTEGRATION.md.

    SwarmController -> SafetySupervisor.evaluate() -> validated Command
                                    |
                                    v
                          AutopilotAdapter (THIS CLASS)
                                    |
                                    v
                        SITLTransport (FakeSITLTransport)

Exposes exactly the same public surface mission.py's `_send_to_autopilot_adapter`
helper already uses on MockAdapter (`.state`, `.mode`, `.armed`, `.failsafe`,
`push_ground_truth_state(...)`) so mission.py needs no branching beyond
picking which adapter type to construct per drone - see mission.py's
`autopilot_path == "fake_sitl"` branch.

This class never reaches for pymavlink/MAVSDK/serial/socket - it only ever
calls methods on the `SITLTransport` object it is given. It never sets
`self.armed = True` anywhere (no implicit arm, ever - Phase 7 requirement).
"""
from __future__ import annotations

from ..contracts import Frame
from ..sitl.commands import SITLCommand, derive_safety_decision_id
from ..sitl.telemetry import SITLTelemetry
from ..sitl.transport import SITLTransport
from .base import is_command_fresh
from .types import AdapterCommand, AdapterResult, AutopilotMode, ConnectionState, VehicleTelemetry


class SITLAdapter:
    """`AutopilotAdapter` implementation backed by a `SITLTransport`. Many
    instances (one per vehicle) typically share one multi-vehicle
    `FakeSITLTransport` - see fake_transport.py's own docstring for why a
    shared transport, not one-per-vehicle, is what actually exercises
    namespace isolation."""

    def __init__(self, vehicle_id: str, namespace: str, transport: SITLTransport,
                 operating_frame: Frame = Frame.LOCAL_ENU):
        self.vehicle_id = vehicle_id
        self.namespace = namespace
        self.transport = transport
        self.operating_frame = operating_frame

        self.state = ConnectionState.DISCONNECTED
        self.mode = AutopilotMode.UNKNOWN
        self.armed = False
        self.failsafe = False

        self.command_history = []          # (AdapterCommand, AdapterResult)
        self.mode_transition_log = []
        self.connection_transition_log = []

    def _set_state(self, state: ConnectionState, at_s: float) -> None:
        if state != self.state:
            self.connection_transition_log.append((at_s, state))
        self.state = state

    def _set_mode(self, mode: AutopilotMode, at_s: float) -> None:
        if mode != self.mode:
            self.mode_transition_log.append((at_s, mode))
        self.mode = mode

    # -- connection lifecycle -------------------------------------------------

    def connect(self) -> AdapterResult:
        result = self.transport.start()
        self._set_state(ConnectionState.CONNECTED if result.success else ConnectionState.DISCONNECTED,
                         at_s=result.timestamp_s)
        if result.success:
            self._set_mode(AutopilotMode.HOLD, at_s=result.timestamp_s)
        return AdapterResult(accepted=result.success, reason=result.reason, timestamp_s=result.timestamp_s,
                              command_sequence=None, adapter_state=self.state)

    def disconnect(self) -> AdapterResult:
        result = self.transport.stop()
        self._set_state(ConnectionState.DISCONNECTED, at_s=result.timestamp_s)
        return AdapterResult(accepted=True, reason=result.reason, timestamp_s=result.timestamp_s,
                              command_sequence=None, adapter_state=self.state)

    # -- mission-harness hook (NOT part of AutopilotAdapter) -----------------

    def push_ground_truth_state(self, telemetry: VehicleTelemetry) -> None:
        """Forwards PyBullet's real state into the transport so
        `read_telemetry` reflects it - exactly like Phase 6's
        MockAdapter.push_ground_truth_state."""
        self.transport.push_vehicle_state(
            vehicle_id=self.vehicle_id, position_m=telemetry.position_m, velocity_mps=telemetry.velocity_mps,
            attitude_rad=telemetry.attitude_rad, angular_velocity_radps=telemetry.angular_velocity_radps,
            battery_fraction=telemetry.battery_fraction, estimator_valid=telemetry.estimator_valid,
            now_s=telemetry.timestamp_s,
        )

    # -- telemetry --------------------------------------------------------

    def read_telemetry(self) -> VehicleTelemetry:
        t: SITLTelemetry = self.transport.receive_telemetry(self.vehicle_id)
        return VehicleTelemetry(
            vehicle_id=t.vehicle_id, timestamp_s=t.timestamp_s, frame=self.operating_frame,
            position_m=t.position_m, velocity_mps=t.velocity_mps, acceleration_mps2=None,
            attitude_rad=t.attitude_rad, angular_velocity_radps=t.angular_velocity_radps,
            battery_fraction=t.battery_fraction, estimator_valid=t.estimator_valid,
            connection_state=self.state, autopilot_mode=t.flight_mode, armed=t.armed,
            failsafe=t.failsafe, sequence=t.sequence,
        )

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

        safety_decision_id = derive_safety_decision_id(self.vehicle_id, now_s, command.command)
        sitl_command = SITLCommand(adapter_command=command, namespace=self.namespace,
                                    safety_decision_id=safety_decision_id)

        transport_result = self.transport.send_command(sitl_command)
        if not transport_result.success:
            return reject(transport_result.reason)

        ack = self.transport.receive_ack(self.vehicle_id)
        self._set_mode(ack.autopilot_mode, at_s=ack.timestamp_s)
        if ack.autopilot_mode == AutopilotMode.LAND:
            self._set_state(ConnectionState.LANDING, at_s=ack.timestamp_s)
        self.failsafe = ack.failsafe
        if ack.failsafe:
            self._set_state(ConnectionState.FAILSAFE, at_s=ack.timestamp_s)
        elif self.state == ConnectionState.FAILSAFE:
            self._set_state(ConnectionState.CONNECTED, at_s=ack.timestamp_s)

        if not ack.accepted:
            return reject(ack.reason)

        result = AdapterResult(accepted=True, reason=ack.reason, timestamp_s=ack.timestamp_s,
                                command_sequence=command.sequence, adapter_state=self.state,
                                transformed_command=command)
        self.command_history.append((command, result))
        return result

    # -- mode requests --------------------------------------------------------

    def set_mode(self, mode: AutopilotMode, now_s: float = 0.0) -> AdapterResult:
        self._set_mode(mode, at_s=now_s)
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
