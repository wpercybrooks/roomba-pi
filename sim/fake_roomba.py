"""A PTY-backed iRobot Open Interface emulator for a 600-series Roomba.

It exists so that the whole stack can be developed and tested with no robot on
the desk. It speaks only the OI subset roverd uses, with this firmware's dialect:

* replies are **bare payloads** — no ``19/N/id`` header, no checksum;
* ``142 100`` answers exactly **80 bytes**, ``142 0`` answers 26, ``142 7`` one;
* ``open_interface_mode`` reads one higher than the spec (Passive 2, Safe 3);
* opcode 173 enters Off mode, where the emulator — like the robot — ignores
  serial commands *and its own buttons* until a 128 or a 7.

Two clocks are deliberately separate:

* the **serve thread** always runs: it reads the master side of the PTY and
  answers queries immediately, so a blocking ``read()`` on the slave succeeds.
* the **physics** is stepped either by the caller (``physics="manual"``, the
  default, deterministic) or by a thread at 10 Hz (``physics="realtime"``, used
  by ``--port fake``). Tests therefore never wait on wall-clock time.

Only ``roverd.oi`` opens the slave side; this module never imports pyserial.
"""

from __future__ import annotations

import os
import select
import struct
import threading
import time
import tty

# Measured on the actual robot (see build/v0-session-spec.md).
WHEELBASE_MM = 235.0
IDLE_CURRENT_MA = -159
FIXTURE_VOLTAGE_MV = 15888      # 0x3e10, 15.89 V
FIXTURE_TEMPERATURE_C = 20      # 0x14
FIXTURE_CHARGE_MAH = 1887       # 0x075f
FIXTURE_CAPACITY_MAH = 2068     # 0x0814
MM_PER_ENCODER_TICK = 0.47      # only used for the encoder cross-check

# Modes as the OI numbers them; the sensor field reports mode + 1 on this unit.
OFF, PASSIVE, SAFE, FULL = 0, 1, 2, 3

# Packet ids that can be queried one at a time: (offset in packet 100, length).
_SINGLE_PACKETS = {
    7: (0, 1), 8: (1, 1), 9: (2, 1), 10: (3, 1), 11: (4, 1), 12: (5, 1),
    13: (6, 1), 14: (7, 1), 15: (8, 1), 16: (9, 1), 17: (10, 1), 18: (11, 1),
    19: (12, 2), 20: (14, 2), 21: (16, 1), 22: (17, 2), 23: (19, 2),
    24: (21, 1), 25: (22, 2), 26: (24, 2), 34: (39, 1), 35: (40, 1),
    42: (43, 1), 43: (44, 2), 44: (46, 2), 45: (48, 2), 46: (50, 2),
    47: (52, 2), 48: (54, 2), 56: (56, 1), 58: (79, 1),
}


class FakeRoomba:
    """An emulated 614 on a PTY pair. ``fake.path`` is the port to open."""

    def __init__(self, *, silent: bool = False, physics: str = "manual",
                 clock=time.monotonic):
        if physics not in ("manual", "realtime"):
            raise ValueError("physics must be 'manual' or 'realtime'")
        self.silent = silent
        self.physics_mode = physics
        self._clock = clock

        self._master, self._slave = os.openpty()
        tty.setraw(self._slave)
        self.path = os.ttyname(self._slave)

        self._lock = threading.RLock()
        self._read_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

        # -- state the host can observe -------------------------------------
        self.mode = OFF
        self.opcodes: list[int] = []
        self.writes: list[bytes] = []
        self.trap_entered = False
        self.buttons_byte = 0

        # -- physical state --------------------------------------------------
        self.sim_time = 0.0
        self.vel_right = 0
        self.vel_left = 0
        self.distance_total = 0.0      # signed mm since power-on, for tests
        self.angle_total = 0.0         # signed degrees, positive = CCW
        self.encoder_left = 3          # measured idle baseline: 3/3
        self.encoder_right = 3
        self.charge_mah = float(FIXTURE_CHARGE_MAH)
        self.capacity_mah = FIXTURE_CAPACITY_MAH
        self.voltage_mv = FIXTURE_VOLTAGE_MV
        self.temperature_c = FIXTURE_TEMPERATURE_C
        self._dist_pending = 0.0       # delta since the last sensor read
        self._angle_pending = 0.0

        # -- scenario state --------------------------------------------------
        self._bumps = {"left": False, "right": False}
        self._cliff = False
        self._wheel_drop = False
        self._stalled = False
        self._scenarios: list[list] = []   # [at, callable, fired]

        self._rx = bytearray()
        self._start_serve_thread()

    # ------------------------------------------------------------------ setup
    @property
    def slave_fd(self) -> int:
        return self._slave

    def _start_serve_thread(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._serve, name="fake-roomba", daemon=True)
        self._thread.start()

    def start(self) -> "FakeRoomba":
        """Start physics (realtime only); the serve thread is already running."""
        if self.physics_mode == "realtime" and not getattr(self, "_phys_thread", None):
            self._phys_thread = threading.Thread(
                target=self._run_physics, name="fake-physics", daemon=True)
            self._phys_thread.start()
        return self

    def __enter__(self) -> "FakeRoomba":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.close()

    def close(self) -> None:
        """Stop both threads and close the PTY. Idempotent."""
        if self._stop.is_set():
            return
        self._stop.set()
        for t in (self._thread, getattr(self, "_phys_thread", None)):
            if t is not None and t is not threading.current_thread():
                t.join(timeout=1.0)
        for fd in (self._master, self._slave):
            try:
                os.close(fd)
            except OSError:
                pass

    # -------------------------------------------------------------- physics
    def step(self, dt: float) -> None:
        """Advance the physical model by ``dt`` seconds (deterministic tests)."""
        if dt <= 0:
            return
        self._drain()
        with self._lock:
            self.sim_time += dt
            self._fire_scenarios()
            if self.mode in (SAFE, FULL) and not self._stalled:
                self._integrate(dt)
            self._drain_battery(dt)

    def _run_physics(self) -> None:
        interval = 0.1
        while not self._stop.wait(interval):
            self._drain()
            with self._lock:
                self.sim_time += interval
                self._fire_scenarios()
                if self.mode in (SAFE, FULL) and not self._stalled:
                    self._integrate(interval)
                self._drain_battery(interval)

    def _integrate(self, dt: float) -> None:
        d_right = self.vel_right * dt
        d_left = self.vel_left * dt
        d = (d_right + d_left) / 2.0
        self.distance_total += d
        self._dist_pending += d
        a = (d_right - d_left) / WHEELBASE_MM * 57.29577951308232
        self.angle_total += a
        self._angle_pending += a
        self.encoder_left = (self.encoder_left + int(round(d_left / MM_PER_ENCODER_TICK))) % 65536
        self.encoder_right = (self.encoder_right + int(round(d_right / MM_PER_ENCODER_TICK))) % 65536

    def _drain_battery(self, dt: float) -> None:
        self.charge_mah = max(0.0, self.charge_mah - abs(IDLE_CURRENT_MA) * dt / 3600.0)

    # ------------------------------------------------------------ scenarios
    def _schedule(self, at: float, fn) -> None:
        with self._lock:
            self._scenarios.append([at, fn, False])

    def _fire_scenarios(self) -> None:
        for item in self._scenarios:
            at, fn, fired = item
            if not fired and self.sim_time >= at:
                item[2] = True
                fn()

    def bump_after(self, seconds: float, side: str = "left", hold: float = 0.2) -> None:
        """Bump on ``side`` at ``seconds`` of sim time, released after ``hold``."""
        if side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'")

        def hit():
            self._bumps[side] = True
            self._schedule(self.sim_time + hold, lambda: self._bumps.__setitem__(side, False))

        self._schedule(seconds, hit)

    def cliff_after(self, seconds: float) -> None:
        """Raise a front-left cliff at ``seconds`` and leave it raised."""
        self._schedule(seconds, lambda: self._set_cliff(True))

    def clear_cliff(self) -> None:
        self._set_cliff(False)

    def _set_cliff(self, value: bool) -> None:
        with self._lock:
            self._cliff = value

    def wheel_drop_after(self, seconds: float) -> None:
        self._schedule(seconds, lambda: self._set_wheel_drop(True))

    def _set_wheel_drop(self, value: bool) -> None:
        with self._lock:
            self._wheel_drop = value

    def stall_after(self, seconds: float) -> None:
        """From ``seconds`` on, velocity commands are accepted but nothing moves."""
        self._schedule(seconds, lambda: self._set_stalled(True))

    def _set_stalled(self, value: bool) -> None:
        with self._lock:
            self._stalled = value

    def press_button(self) -> bool:
        """Simulate a button press. Dead in Off mode, exactly like the robot."""
        with self._lock:
            if self.mode == OFF:
                return False
            self.buttons_byte ^= 0x01      # CLEAN
            return True

    # ---------------------------------------------------------- wire parser
    def _arg_count(self, opcode: int, buf: bytes) -> int | None:
        """How many argument bytes follow this opcode (None = need more bytes)."""
        fixed = {128: 0, 131: 0, 132: 0, 173: 0, 7: 0, 142: 1,
                 137: 4, 145: 4, 139: 3, 141: 1}
        if opcode in fixed:
            return fixed[opcode]
        if opcode == 140:                 # SONG: song, count, then 2*count bytes
            if len(buf) < 2:
                return None
            return 2 + 2 * buf[1]
        return 0                          # unknown opcode: consume nothing else

    def _poll_once(self, timeout: float) -> bool:
        """Read and parse one chunk from the host. Returns True if bytes arrived."""
        with self._read_lock:
            try:
                ready, _, _ = select.select([self._master], [], [], timeout)
            except (OSError, ValueError):
                return False
            if not ready:
                return False
            try:
                chunk = os.read(self._master, 4096)
            except OSError:
                return False
            if not chunk:
                return False
            with self._lock:
                self._rx += chunk
                self._parse()
            return True

    def _serve(self) -> None:
        while not self._stop.is_set():
            self._poll_once(0.005)

    def _drain(self) -> None:
        """Parse anything already written by the host (non-blocking)."""
        while self._poll_once(0):
            pass

    def sync(self, timeout: float = 0.3) -> None:
        """Block until the host's writes have been parsed (for test assertions)."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._poll_once(0.005):
                continue
            with self._read_lock, self._lock:
                if not self._rx:
                    return

    def _parse(self) -> None:
        while self._rx:
            opcode = self._rx[0]
            args = self._arg_count(opcode, bytes(self._rx[1:]))
            if args is None:
                return
            if len(self._rx) < 1 + args:
                return
            frame = bytes(self._rx[:1 + args])
            del self._rx[:1 + args]
            self.writes.append(frame)
            self.opcodes.append(opcode)
            self._handle(opcode, frame[1:])

    def _handle(self, opcode: int, data: bytes) -> None:
        if opcode == 128:                 # START -> Passive, from anywhere
            self.mode = PASSIVE
            self.vel_left = self.vel_right = 0
        elif opcode == 131:               # SAFE, only from Passive/Safe
            if self.mode in (PASSIVE, SAFE):
                self.mode = SAFE
        elif opcode == 132:               # FULL (tracked, never used by roverd)
            if self.mode in (PASSIVE, SAFE, FULL):
                self.mode = FULL
        elif opcode == 173:               # STOP -> Off; the trap
            self.mode = OFF
            self.trap_entered = True
            self.vel_left = self.vel_right = 0
        elif opcode == 7:                 # RESET -> Off, like pulling the battery
            self.mode = OFF
            self.vel_left = self.vel_right = 0
        elif opcode == 137:               # DRIVE velocity, radius
            if self.mode in (SAFE, FULL):
                v, radius = struct.unpack(">hh", data)
                self._apply_drive(v, radius)
        elif opcode == 145:               # DRIVE_DIRECT right, left
            if self.mode in (SAFE, FULL):
                r, l = struct.unpack(">hh", data)
                self.vel_right, self.vel_left = r, l
        elif opcode == 142:               # SENSORS packet id
            self._reply_sensor(data[0])
        # 139 LED, 140 SONG, 141 PLAY: accepted and ignored, like the robot.

    def _apply_drive(self, velocity: int, radius: int) -> None:
        if radius == 0x8000 or radius == 0x7FFF:      # straight
            self.vel_left = self.vel_right = velocity
        elif radius == 0x0001:                        # spin CCW in place
            self.vel_left, self.vel_right = -velocity, velocity
        elif radius == -1 or radius == 0xFFFF:        # spin CW in place
            self.vel_left, self.vel_right = velocity, -velocity
        else:                                         # arc
            vr = velocity * (radius + WHEELBASE_MM / 2) / radius
            vl = velocity * (radius - WHEELBASE_MM / 2) / radius
            self.vel_right = int(max(-500, min(500, vr)))
            self.vel_left = int(max(-500, min(500, vl)))

    def _reply_sensor(self, packet_id: int) -> None:
        if self.silent or self.mode == OFF:
            return                        # in Off mode the robot does not answer
        if packet_id == 100:
            payload = self._packet_100()
        elif packet_id == 0:
            payload = self._packet_100()[:26]
        elif packet_id in _SINGLE_PACKETS:
            offset, length = _SINGLE_PACKETS[packet_id]
            payload = self._packet_100()[offset:offset + length]
        else:
            return
        try:
            os.write(self._master, payload)
        except OSError:
            pass

    def _packet_100(self) -> bytes:
        buf = bytearray(80)
        buf[0] = ((0x01 if self._bumps["right"] else 0)
                  | (0x02 if self._bumps["left"] else 0)
                  | (0x04 if self._wheel_drop else 0)
                  | (0x08 if self._wheel_drop else 0))
        buf[1] = 0                              # wall
        if self._cliff:
            buf[3] = 1                          # cliff front-left
        buf[7] = 0
        buf[11] = self.buttons_byte
        d, a = self._take_motion_deltas()
        struct.pack_into(">h", buf, 12, d)
        struct.pack_into(">h", buf, 14, a)
        buf[16] = 0                             # charger state: off dock
        struct.pack_into(">H", buf, 17, self.voltage_mv)
        struct.pack_into(">h", buf, 19, IDLE_CURRENT_MA)
        struct.pack_into(">b", buf, 21, self.temperature_c)
        struct.pack_into(">H", buf, 22, int(self.charge_mah))
        struct.pack_into(">H", buf, 24, self.capacity_mah)
        buf[39] = 0                             # charging sources
        buf[40] = self.mode + 1                 # this unit's +1 quirk
        struct.pack_into(">h", buf, 44, (self.vel_left + self.vel_right) // 2)
        struct.pack_into(">H", buf, 46, 0x8000) # radius: straight
        struct.pack_into(">h", buf, 48, self.vel_right)
        struct.pack_into(">h", buf, 50, self.vel_left)
        struct.pack_into(">H", buf, 52, self.encoder_left)
        struct.pack_into(">H", buf, 54, self.encoder_right)
        return bytes(buf)

    def _take_motion_deltas(self) -> tuple[int, int]:
        d = int(self._dist_pending)
        a = int(self._angle_pending)
        self._dist_pending -= d
        self._angle_pending -= a
        return max(-32768, min(32767, d)), max(-32768, min(32767, a))
