"""Shared test harness: a deterministic, no-wall-clock run of the whole loop."""

from __future__ import annotations

from roverd.arbiter import Arbiter, Intent
from roverd.oi import RoverOI
from sim.fake_roomba import FakeRoomba

import struct

from pycreate2.packets import SensorPacketDecoder


def make_frame(*, bump_left: bool = False, bump_right: bool = False,
               wheeldrop: bool = False, cliff: bool = False,
               distance: int = 0, angle: int = 0) -> "Sensors":
    """An 80-byte packet 100 frame, spec bit order, straight from the decoder.

    Bits follow the OI wire format (bit0 bump-right, bit1 bump-left), which is
    why the facade's bump_side() has to correct the vendored field names.
    """
    buf = bytearray(80)
    if bump_right:
        buf[0] |= 0x01
    if bump_left:
        buf[0] |= 0x02
    if wheeldrop:
        buf[0] |= 0x04 | 0x08
    if cliff:
        buf[3] = 1                          # front-left
    struct.pack_into(">h", buf, 12, distance)
    struct.pack_into(">h", buf, 14, angle)
    struct.pack_into(">H", buf, 17, 15888)
    struct.pack_into(">h", buf, 19, -159)
    struct.pack_into(">b", buf, 21, 20)
    struct.pack_into(">H", buf, 22, 1887)
    struct.pack_into(">H", buf, 24, 2068)
    buf[40] = 3                              # Safe, this unit's +1 quirk
    return SensorPacketDecoder(bytes(buf))


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
        self.clock_start = self.clock.now
        self.arbiter = Arbiter(clock=self.clock)
        self.tick_s = tick_s
        self.physics_s = physics_s
        self.transitions: list[dict] = []
        self.intents: list[tuple[float, int, int]] = []
        self._physics_acc = 0.0
        reflex.on_transition = lambda record, frame: self.transitions.append(record)

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
            self.intents.append((self.clock.now, left, right))
            self.clock.advance(self.tick_s)

    def close(self) -> None:
        self.oi.shutdown()
        self.fake.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
