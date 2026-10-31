"""Arbiter (TTL, priority, cap, safety stop) and reflex state-machine tests."""

from __future__ import annotations

import pytest

from roverd.arbiter import Arbiter, Intent


def test_arbiter_ttl():
    now = [0.0]
    a = Arbiter(clock=lambda: now[0])
    a.submit(Intent(100, 100, expires_at=0.4, source="reflex"))
    assert a.tick(0.0) == (100, 100)
    assert a.tick(0.6) == (0, 0)                   # older than 500 ms: no motion

    # A careless caller cannot buy more than 500 ms.
    a.submit(Intent(100, 100, expires_at=100.0, source="teleop"))
    assert a.tick(0.6) == (0, 0)


def test_arbiter_priority():
    now = [0.0]
    a = Arbiter(clock=lambda: now[0])
    a.submit(Intent(50, 50, expires_at=0.4, source="agent"))
    a.submit(Intent(80, 80, expires_at=0.4, source="reflex"))
    assert a.tick(0.0) == (50, 50)                 # agent beats reflex

    a.submit(Intent(60, 60, expires_at=0.4, source="teleop"))
    assert a.tick(0.0) == (60, 60)                 # teleop beats agent

    # The speed cap is the arbiter's job, even when a caller asks for 500.
    now[0] = 0.1
    a.submit(Intent(500, -500, expires_at=0.5, source="teleop"))
    assert a.tick(0.1) == (200, -200)

    a.safety_stop(0.1, duration=0.5)
    assert a.tick(0.1) == (0, 0)                   # safety beats teleop
    assert a.tick(0.5) == (0, 0)
    now[0] = 0.7
    assert a.tick(0.7) == (0, 0)                   # the intent itself has expired
