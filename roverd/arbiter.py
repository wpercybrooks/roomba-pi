"""Command arbitration: priorities, TTLs and the speed cap.

Everything that wants the robot to move submits an ``Intent``. Nothing writes a
velocity directly. Three rules make the stack safe:

* **every** intent expires (500 ms at most), so a dead caller cannot leave a
  sticky command;
* ``safety`` (a ``safety_stop``, not an intent source) outranks everything;
* the speed cap is applied here, not at the call sites.

Priority, highest first: safety > teleop > agent > reflex.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

DEFAULT_MAX_SPEED_MM_S = 200
MAX_TTL_S = 0.5

# Higher wins. "safety" is handled by safety_stop(), not by a submitted intent.
PRIORITY = {"teleop": 3, "agent": 2, "reflex": 1}


@dataclass
class Intent:
    """Something wants to move. ``expires_at`` is monotonic and <= 500 ms ahead."""

    left_mm_s: int
    right_mm_s: int
    expires_at: float
    source: str                          # "reflex" | "teleop" | "agent"
    submitted_at: float = field(default=0.0, compare=False)


class Arbiter:
    def __init__(self, *, max_speed_mm_s: int = DEFAULT_MAX_SPEED_MM_S,
                 clock=time.monotonic):
        self.max_speed_mm_s = max_speed_mm_s
        self._clock = clock
        self._intents: dict[str, Intent] = {}
        self._safety_until = 0.0

    def submit(self, intent: Intent) -> None:
        if intent.source not in PRIORITY:
            raise ValueError(f"unknown intent source {intent.source!r}")
        intent.submitted_at = self._clock()
        self._intents[intent.source] = intent

    def safety_stop(self, now: float, duration: float = 0.5) -> None:
        """Suppress every intent for ``duration`` seconds from ``now``."""
        self._safety_until = max(self._safety_until, now + duration)

    def tick(self, now: float) -> tuple[int, int]:
        """The (left, right) velocity to send, after priority, TTL and caps."""
        if now < self._safety_until:
            return (0, 0)
        live = [i for i in self._intents.values()
                if now < i.expires_at and now < i.submitted_at + MAX_TTL_S]
        if not live:
            return (0, 0)
        chosen = max(live, key=lambda i: PRIORITY[i.source])
        return (self._clip(chosen.left_mm_s), self._clip(chosen.right_mm_s))

    def _clip(self, value: float) -> int:
        cap = self.max_speed_mm_s
        return int(max(-cap, min(cap, value)))
