"""Phase 7: per-vehicle namespace isolation - see
docs/PHASE7_SITL_INTEGRATION.md's "vehicle namespace model" section.

A namespace is this package's structural guarantee that one vehicle's
commands, telemetry, and acknowledgements can never be read or accepted
under another vehicle's identity - see fake_transport.py, which keys every
piece of per-vehicle state exclusively by namespace/vehicle_id (never by a
bare index or a shared list position), and tests/test_sitl.py's
namespace-isolation tests.
"""
from __future__ import annotations

from typing import Dict, Tuple


class NamespaceError(ValueError):
    """Raised on any attempted cross-namespace access or a namespace
    collision - never silently ignored or coerced into a nearby vehicle."""


def namespace_for(vehicle_id: str) -> str:
    if not isinstance(vehicle_id, str) or not vehicle_id:
        raise NamespaceError(f"vehicle_id must be a non-empty str, got {vehicle_id!r}")
    return f"sitl/{vehicle_id}"


class VehicleNamespaceRegistry:
    """Registers exactly one namespace per vehicle_id and rejects
    collisions before any command/telemetry ever flows - a namespace
    collision (e.g. two channels both trying to claim the same vehicle
    identity) is caught at registration time, not discovered later as
    cross-talk between two vehicles' state."""

    def __init__(self):
        self._namespace_of: Dict[str, str] = {}
        self._vehicle_of_namespace: Dict[str, str] = {}

    def register(self, vehicle_id: str) -> str:
        if vehicle_id in self._namespace_of:
            raise NamespaceError(f"vehicle_id {vehicle_id!r} is already registered")
        namespace = namespace_for(vehicle_id)
        if namespace in self._vehicle_of_namespace:
            raise NamespaceError(
                f"namespace collision: {namespace!r} already belongs to "
                f"{self._vehicle_of_namespace[namespace]!r}"
            )
        self._namespace_of[vehicle_id] = namespace
        self._vehicle_of_namespace[namespace] = vehicle_id
        return namespace

    def namespace_of(self, vehicle_id: str) -> str:
        if vehicle_id not in self._namespace_of:
            raise NamespaceError(f"vehicle_id {vehicle_id!r} is not registered")
        return self._namespace_of[vehicle_id]

    def vehicle_of(self, namespace: str) -> str:
        if namespace not in self._vehicle_of_namespace:
            raise NamespaceError(f"namespace {namespace!r} is not registered")
        return self._vehicle_of_namespace[namespace]

    def require_match(self, vehicle_id: str, namespace: str) -> None:
        """Raises NamespaceError unless `namespace` is exactly the
        registered namespace for `vehicle_id` - the check every incoming
        SITLCommand goes through before it can touch any per-vehicle
        state."""
        if self.namespace_of(vehicle_id) != namespace:
            raise NamespaceError(
                f"namespace mismatch: vehicle_id {vehicle_id!r} does not own namespace {namespace!r}"
            )

    def __contains__(self, vehicle_id: str) -> bool:
        return vehicle_id in self._namespace_of

    @property
    def vehicle_ids(self) -> Tuple[str, ...]:
        return tuple(self._namespace_of.keys())
