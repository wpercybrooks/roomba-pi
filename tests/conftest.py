"""Shared test harness: a deterministic, no-wall-clock run of the whole loop."""

from __future__ import annotations

from roverd.arbiter import Arbiter, Intent
from roverd.oi import RoverOI
from sim.fake_roomba import FakeRoomba


class SimClock:
    """A monotonic clock the test advances by hand."""

    def __init__(self, start: float = 100.0):
        self.now = start

    def __call__(self) -> float:
        return self.now

    def advance(self, dt: float) -> None:
        self.now += dt


class Harness:
    """Fake robot + facade + reflex + arbiter, stepped at 20 Hz with no sleeping."""

    def __init__(self, reflex, *, tick_s: float = 0.05, physics_s: float = 0.1):
        self.fake = FakeRoomba()
        self.oi = RoverOI(self.fake.path)
        self.reflex = reflex
        self.clock = SimClock()
        self.arbiter = Arbiter(clock=self.clock)
        self.tick_s = tick_s
        self.physics_s = physics_s
        self.transitions: list[dict] = []
        self._physics_acc = 0.0
        reflex.on_transition = self.transitions.append

        self.oi.start()
        self.oi.poll()
        self.oi.safe()

    def run(self, seconds: float) -> None:
        steps = int(round(seconds / self.tick_s))
        for _ in range(steps):
            self._physics_acc += self.tick_s
            while self._physics_acc + 1e-9 >= self.physics_s:
                self.fake.step(self.physics_s)
                self._physics_acc -= self.physics_s
            frame = self.oi.poll()
            intent = self.reflex.step(frame, self.clock.now)
            if intent is not None:
                self.arbiter.submit(intent)
            left, right = self.arbiter.tick(self.clock.now)
            self.oi.command(left, right)
            self.clock.advance(self.tick_s)

    def close(self) -> None:
        self.oi.shutdown()
        self.fake.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
