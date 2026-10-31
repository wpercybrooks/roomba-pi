"""Tests for the OI facade, the PTY fake, and the safety invariants around them."""

from __future__ import annotations

import ast
import pathlib
import struct
import threading
import time

import pytest

from roverd.oi import Mode, NoSensorFrame, PortBusy, RoverOI, bump_side
from roverd.watchdog import Watchdog
from sim.fake_roomba import FakeRoomba

# The measured 142 0 payload (26 bytes), zero-padded to packet 100's 80.
MEASURED_26 = bytes.fromhex(
    "00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 3e 10 ff 61 14 07 5f 08 14"
)
MEASURED_80 = MEASURED_26 + b"\x00" * 54

ROOT = pathlib.Path(__file__).resolve().parent.parent


def _called_names(path: pathlib.Path) -> set[str]:
    """Every function/method name called in a module, ignoring comments and strings."""
    names: set[str] = set()
    for node in ast.walk(ast.parse(path.read_text())):
        if isinstance(node, ast.Call):
            func = node.func
            if isinstance(func, ast.Attribute):
                names.add(func.attr)
            elif isinstance(func, ast.Name):
                names.add(func.id)
    return names


@pytest.fixture
def fake():
    f = FakeRoomba()
    yield f
    f.close()


@pytest.fixture
def oi(fake):
    o = RoverOI(fake.path)
    yield o
    o.shutdown()


@pytest.fixture
def driving(fake, oi):
    oi.start()
    oi.poll()
    oi.safe()
    return oi


def test_oi_decodes_measured_values():
    from pycreate2.packets import SensorPacketDecoder

    s = SensorPacketDecoder(MEASURED_80)
    assert s.sane is True
    assert s.raw == MEASURED_80
    assert s.voltage == 15888
    assert s.temperature == 20
    assert s.battery_charge == 1887
    assert s.battery_capacity == 2068
    assert s.current == -159
    assert s.distance == 0 and s.angle == 0
    assert bump_side(s) is None


def test_drive_direct_wrapper_reorders(driving, fake):
    driving.drive_direct(left_mm_s=100, right_mm_s=-50)
    fake.sync()
    frames = [w for w in fake.writes if w[0] == 145]
    assert frames, "no drive-direct frame was sent"
    assert frames[-1] == bytes([145]) + struct.pack(">hh", -50, 100)   # right first


def test_oi_rejects_short_burst(driving, fake):
    good = driving.poll()
    assert good.sane is True
    fake.burst_length = 26
    stale = driving.poll()
    assert stale.sane is False
    assert stale.voltage == good.voltage
    assert stale.raw == good.raw


def test_oi_rejects_implausible_payload(driving, fake):
    # Before the first good frame, an implausible burst is a startup refusal.
    fresh = FakeRoomba()
    fresh.bad_frame = True
    alone = RoverOI(fresh.path)
    alone.start()
    try:
        with pytest.raises(NoSensorFrame):
            alone.poll()
    finally:
        alone.shutdown()
        fresh.close()

    good = driving.poll()
    fake.bad_frame = True
    stale = driving.poll()
    assert stale.sane is False
    assert stale.voltage == good.voltage          # last good state retained
    assert stale.raw == good.raw


def test_never_sends_173(driving, fake):
    driving.drive_direct(80, 80)
    fake.step(0.2)
    driving.poll()
    driving.shutdown()
    fake.sync()

    assert 173 not in fake.opcodes
    assert b"\xad" not in b"".join(fake.writes)
    assert fake.opcodes[-1] == 128                 # Passive exit, not Off
    assert fake.mode == 1                          # PASSIVE
    assert fake.trap_entered is False

    # Invariant 9: velocity writes live only in the facade; main goes through
    # command(). Checked on the AST, so docstrings cannot mask or trip it.
    offenders = []
    for path in (ROOT / "roverd").glob("*.py"):
        called = _called_names(path)
        velocity = called & {"drive_direct", "drive"}
        if path.name != "oi.py" and velocity:
            offenders.append(f"{path.name}:{sorted(velocity)}")
        if path.name != "oi.py" and "stop_oi" in called:
            offenders.append(f"{path.name}:stop_oi")
    assert offenders == []


def test_device_is_exclusively_locked(fake):
    first = RoverOI(fake.path)
    try:
        with pytest.raises(PortBusy):
            RoverOI(fake.path)
        assert first.start() is Mode.PASSIVE         # the first one still works
    finally:
        first.shutdown()


def test_mode_gating(fake):
    oi = RoverOI(fake.path)
    try:
        with pytest.raises(RuntimeError):
            oi.drive_direct(10, 10)
        oi.start()
        with pytest.raises(RuntimeError):
            oi.drive_direct(10, 10)                  # Passive cannot drive
        oi.safe()
        oi.drive_direct(10, 10)                      # Safe can
        oi.passive()
        with pytest.raises(RuntimeError):
            oi.drive_direct(10, 10)
        oi.zero()                                    # zeros are always allowed
    finally:
        oi.shutdown()


def test_watchdog_zeroes_on_missed_heartbeat(driving, fake):
    wd = Watchdog(lambda: driving.drive_direct(0, 0), timeout_s=0.3, period_s=0.02)
    # Invariant 8: no lock is shared with the loop it guards.
    assert not any(isinstance(v, type(threading.Lock())) for v in vars(wd).values())
    driving.drive_direct(100, 100)
    fake.sync()
    wd.start()
    try:
        for _ in range(4):                        # beat for ~200 ms, then go quiet
            wd.beat()
            time.sleep(0.05)
        idx = len(fake.writes)
        deadline = time.monotonic() + 0.6
        zero_frame = bytes([145]) + b"\x00\x00\x00\x00"
        while time.monotonic() < deadline:
            fake.sync(0.01)
            if zero_frame in fake.writes[idx:]:
                break
            time.sleep(0.01)
        assert zero_frame in fake.writes[idx:], "watchdog never zeroed the wheels"
        assert wd.tripped >= 1
    finally:
        wd.stop()


def test_fake_integrates_distance(driving, fake):
    driving.drive_direct(120, 120)
    for _ in range(10):
        fake.step(0.1)
    s = driving.poll()
    assert s.distance == pytest.approx(120, abs=1)
    assert s.angle == pytest.approx(0, abs=1)


def test_fake_turning(driving, fake):
    driving.drive_direct(-100, 100)                  # left back, right forward
    for _ in range(10):
        fake.step(0.1)
    s = driving.poll()
    assert s.angle == pytest.approx(48.8, abs=1.0)   # (200/235)*180/pi
    assert s.distance == pytest.approx(0, abs=1)
