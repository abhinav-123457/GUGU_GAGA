import pytest

from swarm_sim.seeding import SeedManager, derive_seed


def test_derive_seed_deterministic():
    assert derive_seed(42, "network") == derive_seed(42, "network")


def test_derive_seed_differs_by_subsystem():
    assert derive_seed(42, "network") != derive_seed(42, "obstacles")


def test_derive_seed_differs_by_master_seed():
    assert derive_seed(42, "network") != derive_seed(43, "network")


def test_derive_seed_rejects_bad_inputs():
    with pytest.raises(ValueError):
        derive_seed(42, "")
    with pytest.raises(ValueError):
        derive_seed("42", "network")


def test_derive_seed_rejects_bool_master_seed():
    with pytest.raises(ValueError):
        derive_seed(True, "network")


def test_seed_manager_seed_for_matches_derive_seed():
    mgr = SeedManager(master_seed=7)
    assert mgr.seed_for("obstacles") == derive_seed(7, "obstacles")


def test_seed_manager_returns_same_generator_object():
    mgr = SeedManager(master_seed=7)
    rng1 = mgr.rng("network")
    rng2 = mgr.rng("network")
    assert rng1 is rng2  # same stream, not a fresh one each call


def test_seed_manager_independent_streams_differ():
    mgr = SeedManager(master_seed=7)
    a = mgr.rng("network").random(5)
    b = mgr.rng("obstacles").random(5)
    assert list(a) != list(b)


def test_seed_manager_reproducible_across_instances():
    mgr1 = SeedManager(master_seed=99)
    mgr2 = SeedManager(master_seed=99)
    draws1 = mgr1.rng("controller.levy").random(10)
    draws2 = mgr2.rng("controller.levy").random(10)
    assert list(draws1) == list(draws2)


def test_seed_manager_different_master_seed_differs():
    mgr1 = SeedManager(master_seed=1)
    mgr2 = SeedManager(master_seed=2)
    assert mgr1.seed_for("network") != mgr2.seed_for("network")


def test_seed_manager_using_one_subsystem_does_not_affect_another():
    """Drawing from one subsystem's stream must not perturb another
    subsystem's stream - the exact bug the shared single RNG has today."""
    mgr_a = SeedManager(master_seed=5)
    mgr_a.rng("network").random(1000)  # burn through subsystem A's stream
    later_a = mgr_a.rng("obstacles").random(5)

    mgr_b = SeedManager(master_seed=5)
    # subsystem B's stream untouched by any draw from "network" this time
    later_b = mgr_b.rng("obstacles").random(5)

    assert list(later_a) == list(later_b)


def test_derived_seeds_only_includes_subsystems_actually_used():
    mgr = SeedManager(master_seed=3)
    mgr.rng("network")
    mgr.rng("obstacles")
    seeds = mgr.derived_seeds()
    assert set(seeds.keys()) == {"network", "obstacles"}
