"""Phase 5: peer-local distributed consensus - see
docs/PHASE5_DISTRIBUTED_CONSENSUS.md and swarm_sim/distributed_consensus.py.

Covers the required test list (numbered below matches the phase request):
 1  local state is private per drone
 2  no direct peer-state access                     -> test_distributed_consensus_architecture.py
 3  message delivery uses CommsNetwork
 4  packet loss is respected
 5  latency is respected
 6  stale reports expire
 7  duplicate reports do not inflate quorum
 8  repeated reports from one drone do not inflate quorum
 9  distinct drones can satisfy quorum
10  spatially separated reports do not form one cluster
11  temporally separated reports do not form one cluster
12  out-of-order messages are handled deterministically
13  network partitions prevent unsupported confirmation
14  network healing produces deterministic convergence
15  contradictory reports do not cause unsafe confirmation
16  confirmation expiry works
17  withdrawal/expiry works
18  deterministic replay under fixed seed
19  different RNG streams do not perturb sensors or safety
20  ground-truth isolation AST test                  -> test_distributed_consensus_architecture.py
21  no direct actuation-path test                    -> test_distributed_consensus_architecture.py
22  all commands still pass through SafetySupervisor  -> test_distributed_consensus_architecture.py
23  centralized and distributed modes agree under perfect communication
24  distributed mode degrades measurably under packet loss and latency
"""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from swarm_sim.consensus import ConsensusBoard
from swarm_sim.distributed_consensus import (
    ConfirmationResult, ConsensusMessage, DistributedConsensus, DroneConsensusNode, Evidence, MessageType,
)
from swarm_sim.network import CommsNetwork


def _net_cfg(radius=100.0, latency=0, drop_base=0.0, drop_max=0.0):
    return SimpleNamespace(
        communication_radius=radius, comm_latency_steps=latency,
        comm_dropout_base=drop_base, comm_dropout_at_max_range=drop_max,
    )


def _detection_msg(sender, evidence):
    return ConsensusMessage(MessageType.DETECTION_REPORT, sender, evidence[0].observation_timestamp_s,
                             evidence=tuple(evidence))


# --- 1. local state is private per drone -----------------------------------

def test_each_drone_has_its_own_independent_node_state():
    dc = DistributedConsensus(("drone0", "drone1"), quorum=2, cluster_radius=3.0, window_s=10.0)
    dc.nodes["drone0"].report_detection((1.0, 2.0), 0.9, 0.0)
    assert dc.nodes["drone0"].pending_evidence_count() == 1
    assert dc.nodes["drone1"].pending_evidence_count() == 0


# --- 3. message delivery uses CommsNetwork ----------------------------------

def test_message_is_delivered_via_commsnetwork_send_and_poll():
    net = CommsNetwork(_net_cfg(), 2, np.random.default_rng(0))
    positions = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    msg = ConsensusMessage(MessageType.DETECTION_REPORT, "drone0", 0.0)
    net.send_message(0, msg, positions)
    net.pump_messages()
    inbox = net.poll_inbox(1)
    assert len(inbox) == 1
    assert inbox[0] == (0, msg)
    assert net.poll_inbox(1) == []  # drained, not re-delivered


# --- 4. packet loss is respected --------------------------------------------

def test_send_message_drops_out_of_range_destinations():
    net = CommsNetwork(_net_cfg(radius=5.0), 2, np.random.default_rng(0))
    positions = np.array([[0.0, 0.0, 0.0], [50.0, 0.0, 0.0]])   # far beyond radius
    net.send_message(0, ConsensusMessage(MessageType.DETECTION_REPORT, "drone0", 0.0), positions)
    net.pump_messages()
    assert net.poll_inbox(1) == []


def test_send_message_respects_dropout_probability():
    net = CommsNetwork(_net_cfg(radius=100.0, drop_base=1.0, drop_max=1.0), 2, np.random.default_rng(0))
    positions = np.zeros((2, 3))
    net.send_message(0, ConsensusMessage(MessageType.DETECTION_REPORT, "drone0", 0.0), positions)
    net.pump_messages()
    assert net.poll_inbox(1) == []


# --- 5. latency is respected -------------------------------------------------

def test_send_message_delays_delivery_by_comm_latency_steps():
    net = CommsNetwork(_net_cfg(latency=2), 2, np.random.default_rng(0))
    positions = np.zeros((2, 3))
    net.send_message(0, ConsensusMessage(MessageType.DETECTION_REPORT, "drone0", 0.0), positions)
    net.pump_messages()
    assert net.poll_inbox(1) == []
    net.step += 1
    net.pump_messages()
    assert net.poll_inbox(1) == []
    net.step += 1
    net.pump_messages()
    assert len(net.poll_inbox(1)) == 1


# --- 6. stale reports expire -------------------------------------------------

def test_stale_incoming_evidence_is_rejected_not_stored():
    node = DroneConsensusNode("drone1")
    ev = Evidence("drone0", 0, (0.0, 0.0), 0.9, 0.0)
    node.receive_message(_detection_msg("drone0", [ev]), now_s=20.0, window_s=10.0)
    assert node.pending_evidence_count() == 0
    assert node.stale_message_count == 1


def test_pending_evidence_expires_via_try_confirm_before_clustering():
    node = DroneConsensusNode("drone0")
    node.report_detection((0.0, 0.0), 0.9, 0.0)
    result = node.try_confirm(now_s=20.0, quorum=1, cluster_radius=1.0, window_s=10.0)
    assert result is None
    assert node.expired_evidence_count == 1
    assert node.pending_evidence_count() == 0


# --- 7. duplicate reports do not inflate quorum -----------------------------

def test_duplicate_message_delivery_is_suppressed_and_does_not_inflate_quorum():
    node = DroneConsensusNode("drone2")
    ev = Evidence("drone0", 0, (0.0, 0.0), 0.9, 0.0)
    msg = _detection_msg("drone0", [ev])
    node.receive_message(msg, 0.0, 10.0)
    node.receive_message(msg, 0.0, 10.0)   # e.g. arrived both directly and via relay
    assert node.pending_evidence_count() == 1
    assert node.duplicate_suppressed_count == 1
    assert node.try_confirm(0.0, quorum=2, cluster_radius=1.0, window_s=10.0) is None


# --- 8. repeated reports from one drone do not inflate quorum --------------

def test_one_drones_repeated_reports_still_count_as_a_single_witness():
    node = DroneConsensusNode("drone3")
    node.report_detection((0.0, 0.0), 0.9, 0.0)
    node.report_detection((0.05, 0.0), 0.9, 0.1)   # same drone, second sighting
    result = node.try_confirm(0.2, quorum=2, cluster_radius=1.0, window_s=10.0)
    assert result is None   # still one distinct origin drone


def test_own_report_bounced_back_by_a_relay_is_suppressed_not_double_counted():
    node = DroneConsensusNode("drone3")
    node.report_detection((0.0, 0.0), 0.9, 0.0)
    bounced = Evidence("drone3", 0, (0.0, 0.0), 0.9, 0.0)   # same report_id, relayed back
    node.receive_message(_detection_msg("drone_relay", [bounced]), 0.1, 10.0)
    assert node.own_repeat_suppressed_count == 1
    assert node.pending_evidence_count() == 1


# --- 9. distinct drones can satisfy quorum ----------------------------------

def test_two_distinct_drones_reach_quorum():
    node = DroneConsensusNode("drone_self")
    node.report_detection((0.0, 0.0), 0.9, 0.0)
    ev = Evidence("drone_peer", 0, (0.1, 0.0), 0.85, 0.0)
    node.receive_message(_detection_msg("drone_peer", [ev]), 0.0, 10.0)
    result = node.try_confirm(0.1, quorum=2, cluster_radius=1.0, window_s=10.0)
    assert result is not None
    assert set(result.contributing_drone_ids) == {"drone_self", "drone_peer"}


# --- 10. spatially separated reports do not form one cluster ---------------

def test_spatially_separated_reports_do_not_cluster():
    node = DroneConsensusNode("drone_self")
    node.report_detection((0.0, 0.0), 0.9, 0.0)
    ev = Evidence("drone_peer", 0, (50.0, 50.0), 0.85, 0.0)
    node.receive_message(_detection_msg("drone_peer", [ev]), 0.0, 10.0)
    assert node.try_confirm(0.1, quorum=2, cluster_radius=3.0, window_s=10.0) is None
    clusters = node.cluster_membership(cluster_radius=3.0)
    assert all(len(c) == 1 for c in clusters)


# --- 11. temporally separated reports do not form one cluster --------------

def test_temporally_separated_reports_expire_before_they_can_cluster():
    """Same design as centralized ConsensusBoard: a report's eligibility is
    pruned relative to `now_s`, not to other reports - so two spatially
    co-located reports far enough apart in time that the older one has
    already aged out by the time the newer one is checked can never end
    up in the same cluster."""
    node = DroneConsensusNode("drone_self")
    node.report_detection((0.0, 0.0), 0.9, t=0.0)
    ev_late = Evidence("drone_peer", 0, (0.05, 0.0), 0.85, observation_timestamp_s=15.0)
    node.receive_message(_detection_msg("drone_peer", [ev_late]), now_s=15.0, window_s=10.0)
    # drone_self's t=0 report is now 15s old - beyond the 10s window - and
    # gets pruned before the spatially-coincident ev_late is ever compared
    # against it.
    result = node.try_confirm(now_s=15.0, quorum=2, cluster_radius=1.0, window_s=10.0)
    assert result is None
    assert node.pending_evidence_count() == 1   # only ev_late survives


# --- 12. out-of-order messages are handled deterministically ---------------

def test_out_of_order_message_arrival_yields_the_same_confirmation():
    ev_a = Evidence("drone_a", 0, (0.0, 0.0), 0.9, 1.0)
    ev_b = Evidence("drone_b", 0, (0.1, 0.0), 0.9, 1.2)
    msg_a = _detection_msg("drone_a", [ev_a])
    msg_b = _detection_msg("drone_b", [ev_b])

    node1 = DroneConsensusNode("drone_x")
    node1.receive_message(msg_a, 1.5, 10.0)
    node1.receive_message(msg_b, 1.5, 10.0)
    result1 = node1.try_confirm(1.5, quorum=2, cluster_radius=1.0, window_s=10.0)

    node2 = DroneConsensusNode("drone_y")
    node2.receive_message(msg_b, 1.5, 10.0)   # reverse arrival order
    node2.receive_message(msg_a, 1.5, 10.0)
    result2 = node2.try_confirm(1.5, quorum=2, cluster_radius=1.0, window_s=10.0)

    assert result1 is not None and result2 is not None
    assert result1.confirmation_id == result2.confirmation_id
    assert result1.centroid_m == result2.centroid_m


# --- 13. network partitions prevent unsupported confirmation ---------------

def test_isolated_node_cannot_confirm_without_quorum():
    node = DroneConsensusNode("drone_iso")
    node.report_detection((0.0, 0.0), 0.9, 0.0)
    assert node.try_confirm(0.1, quorum=2, cluster_radius=1.0, window_s=10.0) is None
    assert node.quorum_failure_count == 1


# --- 14. network healing produces deterministic convergence ----------------

def test_partition_healing_converges_once_peer_evidence_arrives():
    node = DroneConsensusNode("drone_iso")
    node.report_detection((0.0, 0.0), 0.9, 0.0)
    assert node.try_confirm(0.1, quorum=2, cluster_radius=1.0, window_s=10.0) is None
    ev = Evidence("drone_peer", 0, (0.05, 0.0), 0.85, 0.1)
    node.receive_message(_detection_msg("drone_peer", [ev]), 0.2, 10.0)
    result = node.try_confirm(0.2, quorum=2, cluster_radius=1.0, window_s=10.0)
    assert result is not None
    assert set(result.contributing_drone_ids) == {"drone_iso", "drone_peer"}


# --- 15. contradictory reports do not cause unsafe confirmation ------------

def test_contradictory_reports_at_different_locations_do_not_jointly_confirm():
    node = DroneConsensusNode("drone_self")
    node.report_detection((0.0, 0.0), 0.9, 0.0)
    ev_far = Evidence("drone_peer", 0, (40.0, 40.0), 0.85, 0.0)
    node.receive_message(_detection_msg("drone_peer", [ev_far]), 0.0, 10.0)
    result = node.try_confirm(0.0, quorum=2, cluster_radius=1.0, window_s=10.0)
    assert result is None
    assert node.pending_evidence_count() == 2   # kept apart, neither discarded nor merged


# --- 16. confirmation expiry works ------------------------------------------

def test_expired_evidence_metric_is_tracked_and_aggregatable_across_nodes():
    dc = DistributedConsensus(("drone0", "drone1"), quorum=2, cluster_radius=1.0, window_s=5.0)
    dc.nodes["drone0"].report_detection((0.0, 0.0), 0.9, 0.0)
    dc.nodes["drone0"].try_confirm(now_s=100.0, quorum=2, cluster_radius=1.0, window_s=5.0)
    assert dc.total_expired_evidence_count() == 1


# --- 17. withdrawal/expiry works --------------------------------------------

def test_expiry_message_purges_matching_pending_evidence():
    node = DroneConsensusNode("drone_x")
    ev = Evidence("drone_peer", 0, (0.0, 0.0), 0.9, 0.0)
    node.receive_message(_detection_msg("drone_peer", [ev]), 0.0, 10.0)
    assert node.pending_evidence_count() == 1
    expiry = ConsensusMessage(MessageType.EXPIRY, "drone_peer", 1.0, expiry_report_ids=(ev.report_id,))
    node.receive_message(expiry, 1.0, 10.0)
    assert node.pending_evidence_count() == 0


def test_expiry_message_for_unknown_report_id_is_a_harmless_no_op():
    node = DroneConsensusNode("drone_x")
    expiry = ConsensusMessage(MessageType.EXPIRY, "drone_peer", 1.0, expiry_report_ids=("drone_z:7",))
    node.receive_message(expiry, 1.0, 10.0)   # must not raise
    assert node.pending_evidence_count() == 0


def test_confirmation_message_purges_the_evidence_it_subsumes():
    node = DroneConsensusNode("drone_x")
    ev = Evidence("drone_peer", 0, (0.0, 0.0), 0.9, 0.0)
    node.receive_message(_detection_msg("drone_peer", [ev]), 0.0, 10.0)
    confirm = ConsensusMessage(
        MessageType.CONFIRMATION, "drone_other", 1.0, confirmation_id="cid1",
        centroid_m=(0.0, 0.0), contributing_drone_ids=("drone_peer", "drone_other"),
        contributing_report_ids=(ev.report_id, "drone_other:0"),
    )
    node.receive_message(confirm, 1.0, 10.0)
    assert node.pending_evidence_count() == 0
    assert len(node.confirmed_log) == 1
    # replaying the same CONFIRMATION again must be a no-op (idempotent adoption)
    node.receive_message(confirm, 2.0, 10.0)
    assert len(node.confirmed_log) == 1


# --- 18. deterministic replay under fixed seed ------------------------------

def test_deterministic_replay_same_seed_same_result():
    from swarm_sim.config import MissionConfig
    from swarm_sim.mission import FloodSearchMission

    def run():
        cfg = MissionConfig(num_drones=4, num_victims=3, num_obstacles=2, duration_sec=6.0, seed=11,
                             consensus_mode="distributed")
        return FloodSearchMission(cfg).run()

    r1, r2 = run(), run()
    assert r1["victims_found"] == r2["victims_found"]
    assert r1["false_confirmations"] == r2["false_confirmations"]
    assert r1["distributed_report_count"] == r2["distributed_report_count"]
    assert r1["distributed_confirmed_victim_count"] == r2["distributed_confirmed_victim_count"]
    assert r1["distributed_duplicate_suppressed_count"] == r2["distributed_duplicate_suppressed_count"]
    assert r1["distributed_quorum_failure_count"] == r2["distributed_quorum_failure_count"]


# --- 19. different RNG streams do not perturb sensors or safety ------------

def test_message_rng_stream_is_independent_of_position_broadcast_rng():
    """Message-channel activity must never advance CommsNetwork.rng (the
    stream tick()'s own position/velocity broadcast dropout draws from) -
    otherwise turning distributed consensus on/off would silently change
    flocking's neighbor tables for an unrelated reason."""
    cfg = _net_cfg(radius=10.0, drop_base=0.3, drop_max=0.3)
    seed = 123
    positions = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    velocities = np.zeros((2, 3))

    net_no_messages = CommsNetwork(cfg, 2, np.random.default_rng(seed))
    net_with_messages = CommsNetwork(cfg, 2, np.random.default_rng(seed), message_rng=np.random.default_rng(999))

    for _ in range(5):
        net_with_messages.send_message(0, ConsensusMessage(MessageType.DETECTION_REPORT, "drone0", 0.0), positions)
        net_with_messages.pump_messages()
        net_no_messages.tick(positions, velocities)
        net_with_messages.tick(positions, velocities)

    assert net_no_messages.rng.random() == net_with_messages.rng.random()


def test_safety_supervisor_has_no_rng_of_any_kind():
    """Structural proof that no RNG stream - old or new - could ever
    perturb the safety supervisor: it has none."""
    import inspect
    from swarm_sim import safety_supervisor as safety_module
    src = inspect.getsource(safety_module)
    assert "rng" not in src.lower()
    assert "random.random" not in src and "np.random" not in src


# --- 23. centralized and distributed modes agree under perfect communication

def test_centralized_and_distributed_agree_given_identical_evidence_stream():
    """The real, unconditional form of "must agree under perfect
    communication": feed the exact same sequence of per-tick detection
    events, in the exact same order, into ConsensusBoard and into a
    single fully-connected DroneConsensusNode (standing in for "every
    drone has received everything, with zero loss and zero latency") and
    assert every confirmation is identical. This is checked at the
    algorithm level, not via a full closed-loop PyBullet mission -
    see docs/PHASE5_DISTRIBUTED_CONSENSUS.md's "why perfect-comm
    agreement is checked at the algorithm level" note: once a mission's
    confirmation feeds back into vehicle trajectories, even a bit-level
    floating-point difference can cascade into a different downstream
    outcome, which is a property of chaotic closed-loop simulation, not
    of the consensus algorithm itself."""
    import random
    cfg = SimpleNamespace(consensus_quorum=2, consensus_cluster_radius=3.0, consensus_window_sec=10.0)

    for seed in range(15):
        rng = random.Random(seed)
        board = ConsensusBoard(cfg)
        node = DroneConsensusNode("observer")
        seq_by_drone = {}
        central_log, dist_log = [], []

        for tick in range(60):
            t = float(tick)
            for d in range(5):
                did = f"drone{d}"
                if rng.random() < 0.12:
                    x, y = rng.uniform(-10, 10), rng.uniform(-10, 10)
                    board.submit(did, np.array([x, y]), t)
                    seq = seq_by_drone.get(did, 0)
                    seq_by_drone[did] = seq + 1
                    ev = Evidence(did, seq, (x, y), 0.9, t)
                    node.receive_message(_detection_msg(did, [ev]), t, cfg.consensus_window_sec)

            r = board.try_confirm(t)
            while r is not None:
                centroid, drones = r
                central_log.append((round(float(centroid[0]), 6), round(float(centroid[1]), 6), tuple(sorted(drones))))
                r = board.try_confirm(t)

            r2 = node.try_confirm(t, cfg.consensus_quorum, cfg.consensus_cluster_radius, cfg.consensus_window_sec)
            while r2 is not None:
                dist_log.append((round(r2.centroid_m[0], 6), round(r2.centroid_m[1], 6),
                                  tuple(sorted(r2.contributing_drone_ids))))
                r2 = node.try_confirm(t, cfg.consensus_quorum, cfg.consensus_cluster_radius, cfg.consensus_window_sec)

        assert central_log == dist_log, f"seed {seed}: centralized and distributed diverged: {central_log} vs {dist_log}"


@pytest.mark.parametrize("seed", [1, 2, 3, 4])
def test_centralized_and_distributed_missions_agree_under_perfect_communication(seed):
    """Full closed-loop mission check (real PyBullet physics, real
    SwarmController/SafetySupervisor pipeline) - not just the algorithm-
    level check above. Centroid averaging must use np.mean, matching
    ConsensusBoard.try_confirm bit-for-bit (see distributed_consensus.py's
    try_confirm): an earlier plain-Python sum()/len() version agreed with
    centralized at the algorithm level but occasionally diverged in a
    full mission, because a ~1e-16 centroid difference becomes a
    beacon position, which shifts a drone's trajectory, which changes
    what it senses next tick - a real, if narrow, floating-point
    sensitivity in this closed feedback loop. See
    docs/PHASE5_DISTRIBUTED_CONSENSUS.md's comparison-mode section."""
    from swarm_sim.config import MissionConfig
    from swarm_sim.mission import FloodSearchMission

    def run(mode):
        cfg = MissionConfig(num_drones=5, num_victims=4, num_obstacles=2, duration_sec=20.0, seed=seed,
                             consensus_mode=mode, communication_radius=1e6,
                             comm_dropout_base=0.0, comm_dropout_at_max_range=0.0, comm_latency_steps=0)
        return FloodSearchMission(cfg).run()

    rc, rd = run("centralized"), run("distributed")
    assert rc["victims_found"] == rd["victims_found"]
    assert rc["victims_found"] + rc["false_confirmations"] == rd["distributed_confirmed_victim_count"]


# --- 24. distributed mode degrades measurably under packet loss/latency ----

def test_distributed_mode_degrades_under_packet_loss_and_latency():
    from swarm_sim.config import MissionConfig
    from swarm_sim.mission import FloodSearchMission

    def run(comm_range, drop_max, latency, seed=1, duration=25.0):
        cfg = MissionConfig(num_drones=5, num_victims=4, num_obstacles=2, duration_sec=duration, seed=seed,
                             consensus_mode="distributed", communication_radius=comm_range,
                             comm_dropout_base=0.02, comm_dropout_at_max_range=drop_max, comm_latency_steps=latency)
        return FloodSearchMission(cfg).run()

    good = run(comm_range=40.0, drop_max=0.0, latency=0)
    bad = run(comm_range=5.0, drop_max=0.97, latency=24)

    assert bad["victims_found"] <= good["victims_found"]
    assert bad["distributed_confirmed_victim_count"] <= good["distributed_confirmed_victim_count"]
    assert bad["victims_found"] < good["victims_found"]   # this seed shows a strict, measurable drop
