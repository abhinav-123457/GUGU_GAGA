"""Deterministic per-subsystem seed derivation.

Today (pre-Phase-1), `mission.py` draws one `np.random.default_rng(config.seed)`
and hands the same stream to obstacle placement, victim placement, network
dropout, and every piece of SwarmController's internal randomness. That is
reproducible run-to-run, but it means a change to, say, Levy-flight step
count shifts the RNG stream position and silently changes which obstacles
get placed where in the *next* run - there's no way to isolate "did this
outcome change because of the network model or the behavior model?"

SeedManager fixes that: every subsystem gets its own independent
np.random.Generator, deterministically derived from one master seed via a
stable (unsalted) hash - so re-running with the same master seed always
reproduces the same per-subsystem seeds, and changing one subsystem's usage
pattern can never perturb another subsystem's stream.
"""
from __future__ import annotations

import hashlib

import numpy as np

_SEED_MODULUS = 2 ** 32


def derive_seed(master_seed: int, subsystem: str) -> int:
    """Deterministically derive a uint32 seed for `subsystem` from
    `master_seed`. Same inputs always produce the same output, in this
    process or any other - unlike Python's built-in `hash()`, which is
    salted per-process and not reproducible across runs."""
    if not isinstance(master_seed, int) or isinstance(master_seed, bool):
        raise ValueError(f"master_seed must be an int, got {master_seed!r}")
    if not isinstance(subsystem, str) or not subsystem:
        raise ValueError(f"subsystem must be a non-empty str, got {subsystem!r}")
    digest = hashlib.sha256(f"{master_seed}:{subsystem}".encode("utf-8")).digest()
    return int.from_bytes(digest[:8], "big") % _SEED_MODULUS


class SeedManager:
    """Lazily creates and caches one independent RNG stream per subsystem
    name, all deterministically derived from a single master seed."""

    def __init__(self, master_seed: int):
        self.master_seed = master_seed
        self._seeds: dict[str, int] = {}
        self._rngs: dict[str, np.random.Generator] = {}

    def _ensure(self, subsystem: str) -> None:
        if subsystem not in self._seeds:
            self._seeds[subsystem] = derive_seed(self.master_seed, subsystem)
            self._rngs[subsystem] = np.random.default_rng(self._seeds[subsystem])

    def rng(self, subsystem: str) -> np.random.Generator:
        """The independent np.random.Generator for `subsystem`. Repeated
        calls with the same name return the same, already-advanced
        generator - call once per subsystem and hold onto the result."""
        self._ensure(subsystem)
        return self._rngs[subsystem]

    def seed_for(self, subsystem: str) -> int:
        """The derived uint32 seed for `subsystem`, for manifest recording."""
        self._ensure(subsystem)
        return self._seeds[subsystem]

    def derived_seeds(self) -> dict[str, int]:
        """Every subsystem seed derived so far, for manifest recording."""
        return dict(self._seeds)
