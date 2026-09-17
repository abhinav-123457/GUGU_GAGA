"""Phase 7: deterministic simulation clock - see docs/PHASE7_SITL_INTEGRATION.md.

Every SITL time source in this package comes from this module, never from
time.time()/time.monotonic()/datetime.now() - see
tests/test_sitl_architecture.py's no-wall-clock check. "Now" only ever
changes through an explicit `advance(dt_s)` (fixed-step mode) or `set(now_s)`
(manually-advanced mode, e.g. jumping straight to a boundary time for an
expiry test) - both are plain arithmetic on a float this object owns,
matching this project's existing simulation-time convention
(SafetySupervisor.evaluate(..., now_s=...), Phase 6 MockAdapter's own
command-timestamp-derived clock).
"""
from __future__ import annotations

import math


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


class SimClock:
    """Monotonic simulation clock. `now_s` never decreases across
    `advance`/`set` calls - both raise ValueError on an attempted decrease,
    so a bug that tries to rewind time is caught immediately rather than
    silently accepted."""

    def __init__(self, initial_time_s: float = 0.0, future_tolerance_s: float = 0.05):
        _require(math.isfinite(initial_time_s) and initial_time_s >= 0.0,
                  f"initial_time_s must be a finite non-negative number, got {initial_time_s!r}")
        _require(math.isfinite(future_tolerance_s) and future_tolerance_s >= 0.0,
                  f"future_tolerance_s must be a finite non-negative number, got {future_tolerance_s!r}")
        self._now_s = float(initial_time_s)
        self.future_tolerance_s = float(future_tolerance_s)

    @property
    def now_s(self) -> float:
        return self._now_s

    def advance(self, dt_s: float) -> float:
        """Fixed-step mode: move the clock forward by dt_s (> 0)."""
        _require(math.isfinite(dt_s) and dt_s > 0.0, f"dt_s must be a finite positive number, got {dt_s!r}")
        self._now_s += dt_s
        return self._now_s

    def set(self, now_s: float) -> float:
        """Manually-advanced mode: jump the clock directly to now_s, which
        must be >= the current time (monotonic, not just "moves forward
        eventually")."""
        _require(math.isfinite(now_s), f"now_s must be a finite number, got {now_s!r}")
        _require(now_s >= self._now_s,
                  f"SimClock is monotonic: cannot set now_s={now_s!r} before current time {self._now_s!r}")
        self._now_s = now_s
        return self._now_s

    def is_from_the_future(self, timestamp_s: float) -> bool:
        """True if timestamp_s is further ahead of `now_s` than
        `future_tolerance_s` allows - used to reject a command whose own
        timestamp claims to be from later than the receiver's clock."""
        return timestamp_s > self._now_s + self.future_tolerance_s

    def is_stale(self, timestamp_s: float, timeout_s: float) -> bool:
        """True once more than timeout_s has elapsed since timestamp_s."""
        return (self._now_s - timestamp_s) > timeout_s

    def is_expired(self, expiration_time_s: float) -> bool:
        """Matches contracts.Command.is_expired's own boundary convention:
        expired AT, and after, the expiration timestamp - not a grace
        period."""
        return self._now_s >= expiration_time_s
