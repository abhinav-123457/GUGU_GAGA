"""Test-only fixture: a minimal, local, in-process "fake ArduPilot" that
speaks REAL MAVLink over a real localhost TCP socket - used to test
`swarm_sim.sitl.ardupilot_transport.ArduPilotSITLTransport` against the
actual wire protocol (real message encode/decode, real socket accept/
recv/send) without needing real ArduPilot firmware. This is NOT part of
the swarm_sim package and NOT a claim of ArduPilot behavior - it responds
to exactly the messages ArduPilotSITLTransport sends, nothing more, and
never claims to model real flight dynamics, EKF behavior, or failsafe
logic beyond what each test explicitly configures.

Not itself a subprocess - runs as a background thread in the test
process, so "process exit" scenarios are modeled by calling `.stop()`
(closing the socket), not by killing an OS process.
"""
from __future__ import annotations

import threading
import time

from pymavlink import mavutil


class DummyArduPilotVehicle:
    def __init__(self, port: int, system_id: int = 1, component_id: int = 1,
                 send_heartbeat: bool = True, send_telemetry: bool = True, send_attitude: bool = True,
                 ack_commands: bool = True, initial_position=(0.0, 0.0, 0.0),
                 battery_remaining: int = 100, estimator_healthy: bool = True):
        self.port = port
        self.system_id = system_id
        self.component_id = component_id
        self.send_heartbeat = send_heartbeat
        self.send_telemetry = send_telemetry
        self.send_attitude = send_attitude
        self.ack_commands = ack_commands
        self.position = list(initial_position)
        self.velocity = [0.0, 0.0, 0.0]
        self.battery_remaining = battery_remaining
        self.estimator_healthy = estimator_healthy
        self.armed = False
        self.custom_mode = 0   # STABILIZE

        self._conn = mavutil.mavlink_connection(f"tcpin:127.0.0.1:{port}",
                                                  source_system=system_id, source_component=component_id)
        self._stop = threading.Event()
        self._thread = None
        self.received_setpoints = []
        self.received_commands = []

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        try:
            self._conn.close()
        except Exception:
            pass

    def _run(self) -> None:
        last_heartbeat_at = 0.0
        last_telem_at = 0.0
        while not self._stop.is_set():
            msg = self._conn.recv_match(blocking=False)
            if msg is not None:
                self._handle_message(msg)

            now = time.monotonic()
            if self._conn.port is not None:
                if self.send_heartbeat and now - last_heartbeat_at > 0.1:
                    base_mode = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED if self.armed else 0
                    self._conn.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_QUADROTOR, mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                        base_mode, self.custom_mode, 0,
                    )
                    last_heartbeat_at = now
                if self.send_telemetry and now - last_telem_at > 0.1:
                    self._send_telemetry()
                    last_telem_at = now
            time.sleep(0.01)

    def _send_telemetry(self) -> None:
        self._conn.mav.local_position_ned_send(
            0, self.position[0], self.position[1], self.position[2],
            self.velocity[0], self.velocity[1], self.velocity[2],
        )
        if self.send_attitude:
            self._conn.mav.attitude_send(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self._conn.mav.sys_status_send(
            0, 0, 0, 0, 12000, -1, self.battery_remaining, 0, 0, 0, 0, 0, 0,
        )
        flags = (mavutil.mavlink.EKF_POS_HORIZ_ABS | mavutil.mavlink.EKF_VELOCITY_HORIZ) if self.estimator_healthy else 0
        self._conn.mav.ekf_status_report_send(flags, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def _handle_message(self, msg) -> None:
        msg_type = msg.get_type()
        if msg_type == "SET_POSITION_TARGET_LOCAL_NED":
            self.received_setpoints.append(msg)
            type_mask = msg.type_mask
            if not (type_mask & 0x0007) == 0x0007:  # position bits not all ignored -> position setpoint
                if not (type_mask & 0x0001):
                    self.position[0] = msg.x
                if not (type_mask & 0x0002):
                    self.position[1] = msg.y
                if not (type_mask & 0x0004):
                    self.position[2] = msg.z
            if not (type_mask & 0x0008):
                self.velocity[0] = msg.vx
            if not (type_mask & 0x0010):
                self.velocity[1] = msg.vy
            if not (type_mask & 0x0020):
                self.velocity[2] = msg.vz
        elif msg_type == "COMMAND_LONG":
            self.received_commands.append(msg)
            if msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                self.custom_mode = int(msg.param2)
            elif msg.command == mavutil.mavlink.MAV_CMD_NAV_LAND:
                self.custom_mode = {v: k for k, v in mavutil.mode_mapping_acm.items()}["LAND"]
            elif msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                # Test-only safety net: this dummy will never receive this
                # in a correctly-behaving transport (see
                # tests/test_ardupilot_sitl_architecture.py's no-arm
                # check), but if it ever does, it still does NOT arm -
                # only acknowledges, matching "no implicit arm" even for
                # a hypothetical malformed caller.
                pass
            if self.ack_commands and self._conn.port is not None:
                self._conn.mav.command_ack_send(msg.command, mavutil.mavlink.MAV_RESULT_ACCEPTED)
