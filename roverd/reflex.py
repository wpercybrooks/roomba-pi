"""The reflex state machine: sensors in, one Intent out.

Pure and deterministic — no I/O, no wall clock, no randomness. The caller owns
timing (20 Hz) and passes ``now`` (monotonic). The same class therefore replays
a log exactly.

    DRIVE --bump--> BACK_OFF --400 ms--> ROTATE --1 s--> DRIVE
    DRIVE --cliff/wheel drop/stall--> STOPPED (latched; reset() to leave)
    five bump/stall events without 3 s of clean driving --> STOPPED (give-up)

Stall detection keys off the ``distance`` field, never encoder ticks: measured
on this robot the tick-to-mm ratio disagreed with ``distance`` by 50% between
runs, so ticks are only a "did both wheels turn?" cross-check.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum

from roverd.arbiter import Intent
from roverd.oi import Sensors, bump_side, cliff_seen, wheel_dropped


class State(Enum):
    DRIVE = "DRIVE"
    BACK_OFF = "BACK_OFF"
    ROTATE = "ROTATE"
    STOPPED = "STOPPED"


@dataclass
class ReflexConfig:
    drive_speed_mm_s: int = 120
    backoff_speed_mm_s: int = 80
    backoff_s: float = 0.4
    rotate_speed_mm_s: int = 100
    rotate_s: float = 1.0
    stall_window_s: float = 0.5
    stall_min_mm: int = 5          # deadband, not zero: encoders idle at 3/3 counts
    giveup_events: int = 5
    giveup_clean_s: float = 3.0
    intent_ttl_s: float = 0.2      # well inside the arbiter's 500 ms ceiling


class Reflex:
    def __init__(self, config: ReflexConfig | None = None, *,
                 clock=time.monotonic):
        self.cfg = config or ReflexConfig()
        self._clock = clock
        self.state = State.DRIVE
        self.transitions: list[dict] = []
        self.on_transition = None      # callable(record: dict, frame: Sensors)
        self._state_since = 0.0
        self._started = False
        self._rotate_sign = 1          # +1 = CCW (turn left), -1 = CW (turn right)
        self._window: list[tuple[float, int]] = []
        self._last_distance = 0
        self._events = 0
        self._last_event_at = 0.0

    # ------------------------------------------------------------------ public
    def reset(self, now: float | None = None) -> None:
        """Leave STOPPED. The only way out of a latched fault."""
        now = self._clock() if now is None else now
        self._enter(State.DRIVE, "reset", now, None)

    def step(self, frame: Sensors, now: float) -> Intent:
        """One 20 Hz tick. Always returns an intent; never raises."""
        if not self._started:
            self._started = True
            self._state_since = now
            self._last_event_at = now

        if self.state is State.STOPPED:
            return self._intent(0, 0, now)

        # A stale frame is still drivable (the TTL protects us), but it must not
        # trigger events: the same held frame must not be counted twice.
        if not frame.sane:
            return self._current_motion(now)

        # Safety faults latch from any state, including mid-manoeuvre.
        if cliff_seen(frame):
            self._latch(State.STOPPED, "cliff", now, frame)
            return self._intent(0, 0, now)
        if wheel_dropped(frame):
            self._latch(State.STOPPED, "wheeldrop", now, frame)
            return self._intent(0, 0, now)

        if self.state is State.DRIVE:
            side = bump_side(frame)
            if side is not None:
                self._event(now, frame)
                if self.state is State.STOPPED:       # give-up
                    return self._intent(0, 0, now)
                self._rotate_sign = -1 if side == "left" else 1
                self._enter(State.BACK_OFF, f"bump_{side}", now, frame)
                return self._intent(-self.cfg.backoff_speed_mm_s,
                                    -self.cfg.backoff_speed_mm_s, now)
            if self._stalled(frame, now):
                self._event(now, frame)
                if self.state is State.STOPPED:       # give-up
                    return self._intent(0, 0, now)
                self._rotate_sign = 1                 # no side: turn left
                self._enter(State.BACK_OFF, "stall", now, frame)
                return self._intent(-self.cfg.backoff_speed_mm_s,
                                    -self.cfg.backoff_speed_mm_s, now)
            return self._intent(self.cfg.drive_speed_mm_s,
                                self.cfg.drive_speed_mm_s, now)

        if self.state is State.BACK_OFF:
            if now - self._state_since >= self.cfg.backoff_s:
                self._enter(State.ROTATE, "backoff_done", now, frame)
                return self._rotate_intent(now)
            return self._intent(-self.cfg.backoff_speed_mm_s,
                                -self.cfg.backoff_speed_mm_s, now)

        if self.state is State.ROTATE:
            if now - self._state_since >= self.cfg.rotate_s:
                self._enter(State.DRIVE, "rotate_done", now, frame)
                self._window.clear()
                return self._intent(self.cfg.drive_speed_mm_s,
                                    self.cfg.drive_speed_mm_s, now)
            return self._rotate_intent(now)

        return self._intent(0, 0, now)

    # ------------------------------------------------------------------ motion
    def _intent(self, left: int, right: int, now: float) -> Intent:
        return Intent(left, right, now + self.cfg.intent_ttl_s, "reflex")

    def _rotate_intent(self, now: float) -> Intent:
        v = self.cfg.rotate_speed_mm_s * self._rotate_sign
        return self._intent(-v, v, now)              # CCW: left back, right forward

    def _current_motion(self, now: float) -> Intent:
        if self.state is State.DRIVE:
            return self._intent(self.cfg.drive_speed_mm_s,
                                self.cfg.drive_speed_mm_s, now)
        if self.state is State.BACK_OFF:
            return self._intent(-self.cfg.backoff_speed_mm_s,
                                -self.cfg.backoff_speed_mm_s, now)
        if self.state is State.ROTATE:
            return self._rotate_intent(now)
        return self._intent(0, 0, now)

    # ------------------------------------------------------------- stall logic
    def _stalled(self, frame: Sensors, now: float) -> bool:
        """Forward drive commanded but < 5 mm accumulated over the last 500 ms."""
        self._window.append((now, frame.distance))
        horizon = now - self.cfg.stall_window_s
        self._window = [w for w in self._window if w[0] >= horizon]
        if now - self._state_since < self.cfg.stall_window_s:
            return False                          # still arming after DRIVE entry
        if self._window[0][0] > horizon + 0.1:
            return False                          # window not yet covered
        moved = sum(delta for _, delta in self._window)
        return moved < self.cfg.stall_min_mm

    # -------------------------------------------------------------- bookkeeping
    def _event(self, now: float, frame: Sensors) -> None:
        cfg = self.cfg
        if now - self._last_event_at >= cfg.giveup_clean_s:
            self._events = 0
        self._events += 1
        self._last_event_at = now
        if self._events >= cfg.giveup_events:
            self._latch(State.STOPPED, "giveup", now, frame)

    def _latch(self, state: State, reason: str, now: float, frame: Sensors) -> None:
        self._enter(state, reason, now, frame)

    def _enter(self, state: State, reason: str, now: float,
               frame: Sensors | None) -> None:
        previous = self.state
        self.state = state
        self._state_since = now
        if state is State.DRIVE:
            self._window.clear()
        record = {
            "t": now,
            "from": previous.value,
            "to": state.value,
            "reason": reason,
        }
        self.transitions.append(record)
        if self.on_transition is not None:
            self.on_transition(record, frame)
