"""Safety-first facade over the vendored pycreate2.

This module is thin on purpose: the protocol (framing, decoding, opcodes) lives
in ``vendor/pycreate2``; what it adds is the part the library deliberately
leaves out --

* an exclusive lock on the device node (the library takes it; we surface it as
  a typed error),
* a short serial timeout so a dying link cannot stall the control loop,
* plausibility checks on every frame (there is no checksum on this firmware),
* ``poll()`` that never raises after the first good frame,
* mode tracked from the commands *we* sent, not from the sensor field,
* a ``shutdown()`` that exits with START (128), never STOP (173).

``Sensors`` is pycreate2's namedtuple as patched. It is re-exported, never
redefined.
"""

from __future__ import annotations

import pathlib
import sys
from enum import Enum

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "vendor"))

from pycreate2 import Create2                      # noqa: E402
from pycreate2.packets import Sensors              # noqa: E402,F401  (re-export)

SERIAL_TIMEOUT_S = 0.1


class Mode(Enum):
    OFF = 0
    PASSIVE = 1
    SAFE = 2
    FULL = 3


class NoSensorFrame(RuntimeError):
    """Raised by poll() only before the first good frame of a session."""


class PortBusy(RuntimeError):
    """The device node is already locked by another process (or instance)."""


def _plausible(s: Sensors) -> bool:
    """This firmware has no checksum, so plausibility is the only integrity check."""
    if not (12000 <= s.voltage <= 20000):
        return False
    if not (0 <= s.temperature <= 60):
        return False
    if s.battery_capacity > 5000:
        return False
    if s.battery_charge > s.battery_capacity:
        return False
    return True


def bump_side(s: Sensors) -> str | None:
    """Which side was bumped, in physical terms: "left", "right" or None.

    The vendored decoder's field names match the wire bits (PATCHES.md #8), so
    this reads them directly — no correction layer.
    """
    b = s.bumps_wheeldrops
    if b.bump_left:
        return "left"
    if b.bump_right:
        return "right"
    return None


def wheel_dropped(s: Sensors) -> bool:
    b = s.bumps_wheeldrops
    return bool(b.wheeldrop_left or b.wheeldrop_right)


def cliff_seen(s: Sensors) -> bool:
    return bool(s.cliff_left or s.cliff_front_left or s.cliff_front_right or s.cliff_right)


class RoverOI:
    """Thin, safety-first facade over the vendored Create2."""

    def __init__(self, port: str, *, create=Create2):
        try:
            self._c = create(port, 115200)
        except Exception as exc:                    # the library raises a bare Exception
            if "already in use" in str(exc) or "locked" in str(exc):
                raise PortBusy(f"{port} is already in use") from exc
            raise
        self._c.SCI.ser.timeout = SERIAL_TIMEOUT_S
        # The vendored start()/safe() sleep this long for mode changes; timing belongs
        # to the caller here (the library's own clearSongMemory still sleeps 0.1 s).
        self._c.sleep_timer = 0
        self._mode = Mode.OFF
        self._last_good: Sensors | None = None
        self._closed = False

    # ------------------------------------------------------------- lifecycle
    @property
    def mode(self) -> Mode:
        """Mode as *we* set it. The sensor field is only a cross-check."""
        return self._mode

    @property
    def port(self) -> str:
        return self._c.SCI.ser.port

    def start(self) -> Mode:
        """START (128) -> Passive. This is the correct resting/exit state."""
        self._c.start()
        self._mode = Mode.PASSIVE
        return self._mode

    def passive(self) -> Mode:
        return self.start()

    def safe(self) -> Mode:
        if self._mode not in (Mode.PASSIVE, Mode.SAFE):
            raise RuntimeError(f"safe() needs Passive, mode is {self._mode.name}")
        self._c.safe()
        self._mode = Mode.SAFE
        return self._mode

    def close(self) -> None:
        if not self._closed:
            self._closed = True
            self._c.close()

    def shutdown(self) -> None:
        """Zero, then START (128) -> Passive, then close. Never sends 173."""
        if self._closed:
            return
        try:
            if self._mode in (Mode.SAFE, Mode.FULL):
                self.zero()
            self._c.start()
            self._mode = Mode.PASSIVE
        finally:
            self.close()

    def stop_oi(self, confirm_off: bool = False) -> None:
        """Opcode 173: Off mode, dead buttons. Never called by roverd's own paths."""
        if not confirm_off:
            raise RuntimeError("stop_oi() disables the robot's buttons; pass confirm_off=True")
        self._c.stop(confirm_off=True)
        self._mode = Mode.OFF

    # ---------------------------------------------------------------- motion
    def drive_direct(self, left_mm_s: int, right_mm_s: int) -> None:
        """Drive each wheel; note pycreate2's signature is (right, left) — swapped here."""
        left = max(-500, min(500, int(left_mm_s)))
        right = max(-500, min(500, int(right_mm_s)))
        if (left, right) != (0, 0) and self._mode not in (Mode.SAFE, Mode.FULL):
            raise RuntimeError(f"drive_direct needs Safe/Full, mode is {self._mode.name}")
        self._c.drive_direct(right, left)

    def drive(self, velocity_mm_s: int, radius_mm: int) -> None:
        if self._mode not in (Mode.SAFE, Mode.FULL):
            raise RuntimeError(f"drive needs Safe/Full, mode is {self._mode.name}")
        self._c.drive(velocity_mm_s, radius_mm)

    def zero(self) -> None:
        """Zeros are always safe, so this is allowed in any mode (watchdog path)."""
        self.drive_direct(0, 0)

    def command(self, left_mm_s: int, right_mm_s: int) -> None:
        """The one path for arbiter output: every loop velocity write goes through here."""
        self.drive_direct(left_mm_s, right_mm_s)

    # --------------------------------------------------------------- sensors
    def poll(self) -> Sensors:
        """Fresh frame, or the last good one with sane=False.

        Raises NoSensorFrame only if this session has never had a good frame.
        Never raises after that — a broken link must not take the loop down.
        """
        s: Sensors | None
        try:
            s = self._c.get_sensors()
        except Exception:
            s = None
        if s is not None and _plausible(s):
            self._last_good = s
            return s
        if self._last_good is None:
            raise NoSensorFrame("no plausible 80-byte frame yet")
        return self._last_good._replace(sane=False)

    def mode_field(self, s: Sensors) -> Mode:
        """The sensor's opinion, corrected for this unit's +1 offset."""
        value = s.open_interface_mode - 1
        try:
            return Mode(value)
        except ValueError:
            return Mode.OFF

    def mode_mismatch(self, s: Sensors) -> bool:
        return self.mode_field(s) is not self._mode
