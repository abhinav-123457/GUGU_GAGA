"""Phase 15: bounded research-reference adapters for Langostino
(swarm-subnet/Langostino, single-aircraft ROS2/INAV reference) and Swarm124
(swarm-subnet/swarm, Bittensor Subnet 124 simulation/policy benchmark).

See docs/PHASE15_SWARM124_LANGOSTINO_INTEGRATION.md for what was verified
from each external project versus what is this project's own inference,
exact commits, and licensing notes.

Nothing in this package is a safety authority. Every module here either:
  - converts an external policy's output into this project's own untrusted
    `swarm_sim.safety_supervisor.CandidateCommand` (swarm124_policy_adapter),
    which must still pass through `SafetySupervisor.evaluate()` exactly
    like every other candidate in the project, or
  - maps an external sensor/telemetry concept into this project's existing
    typed contracts (langostino_reference), never touching MSP, serial, RC,
    or any hardware endpoint.

Nothing in this package ever constructs or sends a raw MAVLink, MSP, RC, or
PWM command, and nothing here ever arms or takes off a vehicle - see
tests/test_phase15_external_architecture.py.
"""
