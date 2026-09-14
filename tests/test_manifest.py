import json

import pytest

from swarm_sim.manifest import RunManifest, build_run_manifest, get_code_version, hash_event_log
from swarm_sim.seeding import SeedManager


def _make_manifest(master_seed=1, config=None):
    config = config if config is not None else {"num_drones": 6, "arena_size": 40.0}
    mgr = SeedManager(master_seed=master_seed)
    mgr.rng("network")
    mgr.rng("obstacles")
    return build_run_manifest(config, master_seed, mgr, map_id="flood_v1", model_id="cf2x")


# ---------------------------------------------------------------------------
# Serialization round trip
# ---------------------------------------------------------------------------

def test_manifest_to_dict_from_dict_round_trip():
    m = _make_manifest()
    d = m.to_dict()
    restored = RunManifest.from_dict(d)
    assert restored == m


def test_manifest_dict_is_json_serializable():
    m = _make_manifest()
    # Must not raise - every field has to be a plain JSON-compatible type.
    text = json.dumps(m.to_dict())
    assert json.loads(text) == m.to_dict()


def test_manifest_save_load_json_round_trip(tmp_path):
    m = _make_manifest()
    path = tmp_path / "manifest.json"
    m.save_json(path)
    restored = RunManifest.load_json(path)
    assert restored == m


# ---------------------------------------------------------------------------
# Identical inputs -> identical manifest
# ---------------------------------------------------------------------------

def test_identical_inputs_produce_identical_manifest():
    m1 = _make_manifest(master_seed=1)
    m2 = _make_manifest(master_seed=1)
    assert m1 == m2
    assert m1.to_dict() == m2.to_dict()


# ---------------------------------------------------------------------------
# Changed inputs -> changed manifest
# ---------------------------------------------------------------------------

def test_changed_master_seed_changes_manifest():
    m1 = _make_manifest(master_seed=1)
    m2 = _make_manifest(master_seed=2)
    assert m1.master_seed != m2.master_seed
    assert m1.subsystem_seeds != m2.subsystem_seeds
    assert m1 != m2


def test_changed_config_changes_manifest():
    m1 = _make_manifest(config={"num_drones": 6, "arena_size": 40.0})
    m2 = _make_manifest(config={"num_drones": 12, "arena_size": 40.0})
    assert m1.config != m2.config
    assert m1 != m2


def test_changed_code_version_changes_manifest():
    m1 = _make_manifest()
    m2 = RunManifest.from_dict({**m1.to_dict(), "code_version": "srchash:deadbeefdeadbeef"})
    assert m1.code_version != m2.code_version
    assert m1 != m2


def test_code_version_is_stable_for_unchanged_source():
    assert get_code_version() == get_code_version()


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------

def test_manifest_rejects_non_dict_config():
    with pytest.raises(ValueError):
        RunManifest(
            contract_version="0.1.0", config="not-a-dict", master_seed=1, subsystem_seeds={},
            code_version="srchash:abc", python_version="3.12.0", dependency_versions={},
            map_id="m", model_id="cf2x",
        )


def test_manifest_rejects_empty_map_id():
    with pytest.raises(ValueError):
        RunManifest(
            contract_version="0.1.0", config={}, master_seed=1, subsystem_seeds={},
            code_version="srchash:abc", python_version="3.12.0", dependency_versions={},
            map_id="", model_id="cf2x",
        )


# ---------------------------------------------------------------------------
# Event-log hash
# ---------------------------------------------------------------------------

def test_hash_event_log_deterministic():
    events = [{"t": 0.0, "kind": "arm"}, {"t": 1.0, "kind": "takeoff"}]
    assert hash_event_log(events) == hash_event_log(events)


def test_hash_event_log_differs_for_different_events():
    a = [{"t": 0.0, "kind": "arm"}]
    b = [{"t": 0.0, "kind": "disarm"}]
    assert hash_event_log(a) != hash_event_log(b)


def test_manifest_with_event_log_sets_hash():
    mgr = SeedManager(master_seed=1)
    events = [{"t": 0.0, "kind": "arm"}]
    m = build_run_manifest({}, 1, mgr, event_log=events)
    assert m.event_log_hash == hash_event_log(events)


def test_manifest_without_event_log_has_no_hash():
    mgr = SeedManager(master_seed=1)
    m = build_run_manifest({}, 1, mgr)
    assert m.event_log_hash is None


# ---------------------------------------------------------------------------
# Contract-version compatibility policy
# ---------------------------------------------------------------------------

def test_manifest_from_dict_requires_contract_version_present():
    d = _make_manifest().to_dict()
    del d["contract_version"]
    with pytest.raises(ValueError, match="contract_version"):
        RunManifest.from_dict(d)


def test_manifest_from_dict_rejects_incompatible_major_version():
    d = _make_manifest().to_dict()
    d["contract_version"] = "99.0.0"
    with pytest.raises(ValueError, match="incompatible contract_version"):
        RunManifest.from_dict(d)


def test_manifest_from_dict_missing_required_field_raises_clear_value_error():
    d = _make_manifest().to_dict()
    del d["master_seed"]
    with pytest.raises(ValueError, match="master_seed"):
        RunManifest.from_dict(d)


def test_manifest_from_dict_event_log_hash_may_be_omitted():
    d = _make_manifest().to_dict()
    del d["event_log_hash"]  # documented-optional: no event log was produced
    restored = RunManifest.from_dict(d)
    assert restored.event_log_hash is None


def test_manifest_rejects_bool_master_seed():
    with pytest.raises(ValueError):
        RunManifest(
            contract_version="0.1.0", config={}, master_seed=True, subsystem_seeds={},
            code_version="srchash:abc", python_version="3.12.0", dependency_versions={},
            map_id="m", model_id="cf2x",
        )
