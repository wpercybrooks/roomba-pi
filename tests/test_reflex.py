"""Arbiter (TTL, priority, cap, safety stop) and reflex state-machine tests."""

from __future__ import annotations

import pytest

from roverd.arbiter import Arbiter, Intent
from roverd.reflex import Reflex, State
from conftest import Harness, make_frame


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


def test_bump_triggers_backoff_then_rotate():
    reflex = Reflex()
    h = Harness(reflex)
    try:
        h.fake.bump_after(1.0, "left")
        h.run(2.6)
        seq = [(r["from"], r["to"], r["reason"]) for r in h.transitions]
        assert seq[:3] == [("DRIVE", "BACK_OFF", "bump_left"),
                           ("BACK_OFF", "ROTATE", "backoff_done"),
                           ("ROTATE", "DRIVE", "rotate_done")]

        # Rotating away from a left bump means clockwise: left forward, right back.
        t_rotate = next(r["t"] for r in h.transitions if r["to"] == "ROTATE")
        first_rotate = next(i for i in h.intents if i[0] >= t_rotate)
        assert first_rotate[1] > 0 > first_rotate[2]
    finally:
        h.close()


def test_cliff_latches_stopped():
    reflex = Reflex()
    h = Harness(reflex)
    try:
        h.fake.cliff_after(1.0)
        h.run(1.5)
        assert reflex.state is State.STOPPED
        assert h.transitions[-1]["reason"] == "cliff"

        moved = h.fake.distance_total
        h.run(0.5)
        assert h.fake.distance_total == pytest.approx(moved, abs=0.01)
        assert h.intents[-1][1:] == (0, 0)
    finally:
        h.close()


def test_cliff_latch_survives_frames():
    reflex = Reflex()
    latched = reflex.step(make_frame(cliff=True), 0.0)
    assert latched.left_mm_s == 0 and latched.right_mm_s == 0
    assert reflex.state is State.STOPPED

    for i in range(50):                            # clean frames change nothing
        intent = reflex.step(make_frame(distance=20), 0.05 * (i + 1))
        assert (intent.left_mm_s, intent.right_mm_s) == (0, 0)

    reflex.reset(now=3.0)
    assert reflex.state is State.DRIVE
    assert reflex.step(make_frame(distance=0), 3.1).left_mm_s > 0


def test_stall_detection():
    reflex = Reflex()
    h = Harness(reflex)
    try:
        h.fake.stall_after(2.0)
        h.run(3.4)
        stall = next((r for r in h.transitions if r["reason"] == "stall"), None)
        assert stall is not None, "no stall was detected"
        assert stall["to"] == "BACK_OFF"
        assert stall["t"] - h.clock_start <= 2.7      # within 700 ms of the stall
    finally:
        h.close()


def test_giveup_after_five_events():
    reflex = Reflex()
    h = Harness(reflex)
    try:
        for i in range(5):                         # 1.7 s apart, under the 3 s clean window
            h.fake.bump_after(1.0 + 1.7 * i, "left" if i % 2 else "right")
        h.run(10.0)
        assert reflex.state is State.STOPPED
        assert h.transitions[-1]["reason"] == "giveup"
        events = [r for r in h.transitions if r["to"] == "BACK_OFF"]
        # The fifth event does not back off: it latches STOPPED with reason "giveup".
        assert len(events) == 4
    finally:
        h.close()
