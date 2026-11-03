"""The independent watchdog: if the loop stops breathing, this stops the robot.

It deliberately shares **no lock** with the loop it guards. ``beat()`` only
assigns one float (atomic under the GIL) and the thread calls the same
``drive_direct(0, 0)`` the loop would have. If the loop deadlocks or dies, this
thread still runs and still zeroes the wheels.

Bench-grade: no real-time scheduling, no priority. Three hundred milliseconds
is the contract; do not make it longer because a slow test liked it.
"""

from __future__ import annotations

import threading
import time


class Watchdog:
    def __init__(self, zero, *, timeout_s: float = 0.3, period_s: float = 0.05,
                 clock=time.monotonic):
        self._zero = zero
        self._clock = clock
        self._timeout_s = timeout_s
        self._period_s = period_s
        self._last_beat = clock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.tripped = 0                 # read from tests; written only by the watchdog

    def beat(self) -> None:
        """Called by the control loop every tick. No lock: one float assignment."""
        self._last_beat = self._clock()

    def start(self) -> "Watchdog":
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, name="watchdog", daemon=True)
            self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=1.0)
            self._thread = None

    def check_once(self) -> bool:
        """One firing decision. Returns True if this call zeroed the wheels.

        Public so tests can drive it from an injected clock instead of sleeping;
        ``_run`` is a thin loop over it.
        """
        if self._clock() - self._last_beat <= self._timeout_s:
            return False
        try:
            self._zero()
        except Exception:
            pass                     # a dead port must not kill the watchdog thread
        self.tripped += 1
        self._last_beat = self._clock()    # fire once per timeout, not per period
        return True

    def _run(self) -> None:
        while not self._stop.wait(self._period_s):
            self.check_once()

    def __enter__(self) -> "Watchdog":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()
