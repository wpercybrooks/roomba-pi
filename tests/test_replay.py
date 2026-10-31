"""Startup refusal and replay determinism — the two end-to-end guarantees."""

from __future__ import annotations

from roverd.log import read_log
from roverd.main import replay, run_live
from sim.fake_roomba import FakeRoomba


def test_startup_refuses_without_frames():
    fake = FakeRoomba(silent=True)          # alive, but answers no sensor queries
    try:
        rc = run_live(fake.path)
        fake.sync()
        assert rc != 0, "a blind controller must not run"
        assert 131 not in fake.opcodes, "never entered Safe without a frame"
        assert 173 not in fake.opcodes
        assert fake.opcodes[0] == 128       # started Passive, then refused
    finally:
        fake.close()


class LoopClock:
    """A clock whose sleep() advances sim time and steps the fake's physics."""

    def __init__(self, fake, start: float = 100.0):
        self.fake = fake
        self.t = start

    def __call__(self) -> float:
        return self.t

    def sleep(self, dt: float) -> None:
        if dt > 0:
            self.fake.step(dt)
            self.t += dt


def test_replay_determinism(tmp_path):
    log_path = str(tmp_path / "run.jsonl")
    fake = FakeRoomba(physics="manual")
    fake.bump_after(2.0, "left")
    fake.stall_after(4.5)
    try:
        clock = LoopClock(fake)
        rc = run_live(fake.path, run_for=7.0, log_path=log_path,
                      clock=clock, sleep=clock.sleep)
        assert rc == 0
    finally:
        fake.close()

    records = list(read_log(log_path))
    logged = [r for r in records if r["kind"] == "transition"]
    assert logged, "the run produced no transitions to replay"

    lines: list[str] = []
    rc = replay(log_path, out=lines.append)
    assert rc == 0, "\n".join(lines)
    assert lines[-1].startswith("REPLAY OK")
    assert len(lines) - 1 == len(logged)
