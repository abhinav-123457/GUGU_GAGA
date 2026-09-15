"""Regression tests for contact-metric accounting (diagnostic-only fix -
see docs/PHASE2_DIAGNOSTICS.md's "raw vs. deduplicated vs. step"
reconciliation). These caught, and now guard against regressing, a real
discrepancy: the Phase 2 diagnostic report quoted 291 for scenario 6 while
docs/PHASE2_DIAGNOSTICS.md quoted 229 for the same run - two different,
both-correct counts (raw PyBullet contact-point records vs. ticks with a
contact) that were being called by the same name, "contacts".

Nothing here touches sensor, controller, consensus, or safety behavior -
these tests exercise `swarm_sim.diagnostics` and the pure, stateless
`swarm_sim.mission._dedupe_contacts_by_pair` helper directly, with no
PyBullet/physics dependency.
"""
from swarm_sim.diagnostics import MissionDiagnostics
from swarm_sim.mission import _dedupe_contacts_by_pair


def _fake_contact_point(body_a, body_b):
    """Minimal stand-in for one `p.getContactPoints()` record: a tuple
    whose index 1 and 2 are the two body unique ids (the only fields
    `_dedupe_contacts_by_pair`/`_classify_and_log_contacts` read from it)."""
    return (0, body_a, body_b, (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 1), 0.0)


def test_dedupe_contacts_by_pair_collapses_multiple_manifold_points():
    # Same pair (10, 20) reported 3 times in one tick (as PyBullet does for
    # a multi-point contact manifold), plus one distinct pair (10, 30).
    raw = [
        _fake_contact_point(10, 20),
        _fake_contact_point(20, 10),  # order-swapped, still the same pair
        _fake_contact_point(10, 20),
        _fake_contact_point(10, 30),
    ]
    deduped = _dedupe_contacts_by_pair(raw)
    assert len(raw) == 4  # raw_contact_point_count for this tick
    assert len(deduped) == 2  # deduplicated_contact_event_count for this tick
    assert set(deduped.keys()) == {(10, 20), (10, 30)}


def test_dedupe_contacts_by_pair_empty_input():
    assert _dedupe_contacts_by_pair([]) == {}


def test_diagnostics_contact_counters_are_independent_and_unambiguous():
    """Simulates two ticks' worth of contact bookkeeping the way
    mission.py's _classify_and_log_contacts does: raw_contact_point_count
    accumulates every raw record, while classify_contact is only called
    once per already-deduplicated pair - so the two counts can legitimately
    differ, and the category counts must always sum to the deduplicated
    total."""
    diag = MissionDiagnostics(victims_xy=[(0.0, 0.0)], consensus_quorum=2, consensus_cluster_radius=1.0)
    drone_ids = {10, 20, 30}
    obstacle_ids = {99}

    # Tick 1: drone 10 vs drone 20, reported as 3 raw manifold points -
    # raw count grows by 3, but classify_contact is called once (post-dedup).
    diag.raw_contact_point_count += 3
    diag.classify_contact(
        t=0.0, body_a=10, body_b=20, drone_id_set=drone_ids, obstacle_body_ids=obstacle_ids,
        estimated_clearance_m=0.1, actual_clearance_m=0.08, sensor_age_s=0.02, command_age_s=0.02,
        observation_dropout_or_stale=False, drone_ids_involved=(0, 1), pair_key=(10, 20),
    )

    # Tick 2: drone 30 vs obstacle 99, reported as 1 raw point.
    diag.raw_contact_point_count += 1
    diag.classify_contact(
        t=1.0 / 30.0, body_a=30, body_b=99, drone_id_set=drone_ids, obstacle_body_ids=obstacle_ids,
        estimated_clearance_m=None, actual_clearance_m=0.05, sensor_age_s=None, command_age_s=None,
        observation_dropout_or_stale=True, drone_ids_involved=(2,), pair_key=(30, 99),
    )

    diag.contact_step_count = 2  # would be set from mission.py's own _contact_steps
    report = diag.to_report_dict()

    assert report["raw_contact_point_count"] == 4
    assert report["deduplicated_contact_event_count"] == 2
    assert report["contact_step_count"] == 2
    assert report["drone_drone_contact_count"] == 1
    assert report["drone_obstacle_contact_count"] == 1
    assert report["drone_ground_contact_count"] == 0
    # The three category counts must always account for every deduplicated
    # event exactly once - this is the invariant that makes the category
    # breakdown "unambiguous" rather than another possibly-inconsistent count.
    assert (report["drone_drone_contact_count"] + report["drone_obstacle_contact_count"]
            + report["drone_ground_contact_count"]) == report["deduplicated_contact_event_count"]
    # raw >= deduplicated always holds: dedup can only ever collapse
    # records, never invent new ones.
    assert report["raw_contact_point_count"] >= report["deduplicated_contact_event_count"]
