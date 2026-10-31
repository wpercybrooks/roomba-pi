"""CLI that ties the stack together: poll, reflex, arbitrate, drive, log.

    python -m roverd.main --port fake --run-for 30 --log /tmp/run.jsonl
    python -m roverd.main --port /dev/ttyUSB0
    python -m roverd.main --replay /tmp/run.jsonl

`--port fake` spawns :class:`sim.fake_roomba.FakeRoomba` on a real PTY; every
other value is a real serial device. Startup refuses to enter Safe unless a
plausible 80-byte frame arrives within 500 ms: a controller that cannot see
must not drive. Exit is always zero velocity -> START (128) -> close. Never 173.
"""

from __future__ import annotations

import argparse
import signal
import sys
import threading
import time
from dataclasses import asdict

from roverd.arbiter import Arbiter
from roverd.log import RunLog, dict_to_frame, frame_to_dict, read_log
from roverd.oi import NoSensorFrame, PortBusy, RoverOI
from roverd.reflex import Reflex, ReflexConfig, State
from roverd.watchdog import Watchdog

TICK_HZ = 20
TICK_S = 1.0 / TICK_HZ
STARTUP_TIMEOUT_S = 0.5
SAFE_SETTLE_S = 0.1


def _now(clock) -> float:
    return clock()


def run_live(port: str, *, run_for: float | None = None, log_path: str | None = None,
             config: ReflexConfig | None = None, max_speed_mm_s: int = 200,
             clock=time.monotonic, sleep=time.sleep,
             stop_event: threading.Event | None = None) -> int:
    """Run the control loop against ``port``. Returns a process exit code."""
    config = config or ReflexConfig()
    stop_event = stop_event or threading.Event()

    try:
        oi = RoverOI(port)
    except PortBusy as exc:
        print(f"roverd: {exc}", file=sys.stderr)
        return 2
    except Exception as exc:
        print(f"roverd: cannot open {port}: {exc}", file=sys.stderr)
        return 2

    log = RunLog(log_path) if log_path else None
    watchdog: Watchdog | None = None
    try:
        if log is not None:
            log.write("meta", _now(clock),
                      config={**asdict(config), "max_speed_mm_s": max_speed_mm_s,
                              "tick_hz": TICK_HZ, "port": port})

        # --- startup self-check -------------------------------------------------
        oi.start()                                  # 128 -> Passive; safe, buttons live
        deadline = _now(clock) + STARTUP_TIMEOUT_S
        first = None
        while _now(clock) < deadline:
            try:
                first = oi.poll()
                break
            except NoSensorFrame:
                sleep(0.02)
        if first is None:
            print("roverd: no plausible sensor frame within 500 ms; refusing to enter Safe",
                  file=sys.stderr)
            if log is not None:
                log.write("fault", _now(clock), reason="startup_no_frame")
            return 1

        oi.safe()                                   # 131
        sleep(SAFE_SETTLE_S)                        # caller owns timing; the robot needs a beat

        reflex = Reflex(config, clock=clock)

        def on_transition(record: dict, frame) -> None:
            if log is None:
                return
            fields = {k: record[k] for k in ("from", "to", "reason")}
            log.write("transition", record["t"], frame=frame_to_dict(frame) if frame else None,
                      **fields)

        reflex.on_transition = on_transition
        arbiter = Arbiter(max_speed_mm_s=max_speed_mm_s, clock=clock)
        watchdog = Watchdog(oi.zero).start()

        started = _now(clock)
        next_tick = started                       # start the cadence here, not before startup
        last_fault_at = -1.0
        stop_logged = False

        while not stop_event.is_set():
            now = _now(clock)
            if run_for is not None and now - started >= run_for:
                break
            watchdog.beat()

            frame = oi.poll()                       # never raises after the first good frame
            if log is not None:
                log.write("frame", now, **frame_to_dict(frame))
            if not frame.sane and now - last_fault_at >= 1.0:
                last_fault_at = now
                if log is not None:
                    log.write("fault", now, reason="stale_frame")
            elif frame.sane and oi.mode_mismatch(frame) and now - last_fault_at >= 1.0:
                last_fault_at = now
                if log is not None:
                    log.write("fault", now, reason="mode_mismatch",
                              field=frame.open_interface_mode, tracked=oi.mode.value)

            intent = reflex.step(frame, now)
            arbiter.submit(intent)
            left, right = arbiter.tick(now)
            oi.command(left, right)                 # the only velocity write path
            if log is not None:
                log.write("intent", now, left_mm_s=left, right_mm_s=right,
                          source=intent.source, state=reflex.state.value)

            if reflex.state is State.STOPPED and not stop_logged:
                stop_logged = True
                if log is not None:
                    log.write("stop", now, reason=reflex.transitions[-1]["reason"])

            next_tick += TICK_S
            delay = next_tick - _now(clock)
            if delay > 0:
                sleep(delay)
            elif delay < -TICK_S:
                # More than a tick behind (a stale poll blocks for 100 ms). Resync
                # instead of bursting back-to-back ticks to "catch up".
                next_tick = _now(clock)

    finally:
        if watchdog is not None:
            watchdog.stop()
        try:
            oi.shutdown()                           # zero -> 128 (Passive) -> close
        finally:
            if log is not None:
                log.close()
    return 0


def replay(path: str, *, out=print) -> int:
    """Re-run the reflex over a logged frame sequence and compare transitions."""
    records = list(read_log(path))
    meta = next((r for r in records if r.get("kind") == "meta"), None)
    cfg_fields = dict(meta["config"]) if meta else {}
    cfg_fields = {k: v for k, v in cfg_fields.items()
                  if k in ReflexConfig.__dataclass_fields__}
    config = ReflexConfig(**cfg_fields)

    frames = [(r["t"], dict_to_frame(r)) for r in records if r.get("kind") == "frame"]
    logged = [r for r in records if r.get("kind") == "transition"]
    if not frames:
        out("REPLAY FAILED: no frame records in log")
        return 1

    reflex = Reflex(config)
    for t, frame in frames:
        reflex.step(frame, t)

    def key(r: dict) -> tuple:
        return (round(r["t"], 9), r["from"], r["to"], r["reason"])

    if [key(r) for r in reflex.transitions] != [key(r) for r in logged]:
        out(f"REPLAY FAILED: {len(reflex.transitions)} transitions replayed, "
            f"{len(logged)} logged")
        for a, b in zip(reflex.transitions, logged):
            if key(a) != key(b):
                out(f"  first difference: replayed {key(a)} logged {key(b)}")
                break
        return 3

    for r in reflex.transitions:
        out(f"{r['t']:.3f} {r['from']} -> {r['to']} ({r['reason']})")
    out(f"REPLAY OK: {len(reflex.transitions)} transitions identical")
    return 0


def _demo_scenarios(fake) -> None:
    """Events for `--port fake`: a bump each side, then a stall that clears."""
    fake.bump_after(5.0, "left")
    fake.bump_after(12.0, "right")
    fake.stall_after(20.0)
    fake.clear_stall_after(24.0)          # let the rover recover instead of giving up


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="roverd", description=__doc__.splitlines()[0])
    p.add_argument("--port", default="/dev/ttyUSB0",
                   help="serial device, or 'fake' to spawn the PTY simulator")
    p.add_argument("--run-for", type=float, default=None,
                   help="seconds to run (default: until SIGINT/SIGTERM)")
    p.add_argument("--log", default=None, help="JSONL run log to append to")
    p.add_argument("--replay", default=None, help="replay a JSONL log instead of driving")
    p.add_argument("--speed", type=int, default=ReflexConfig.drive_speed_mm_s,
                   help="reflex drive speed in mm/s")
    p.add_argument("--max-speed", type=int, default=200, help="arbiter speed cap in mm/s")
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.replay:
        return replay(args.replay)

    config = ReflexConfig(drive_speed_mm_s=args.speed)
    stop = threading.Event()
    if threading.current_thread() is threading.main_thread():
        for sig in (signal.SIGTERM, signal.SIGINT):
            signal.signal(sig, lambda *_: stop.set())

    fake = None
    port = args.port
    if args.port == "fake":
        from sim.fake_roomba import FakeRoomba
        fake = FakeRoomba(physics="realtime").start()
        _demo_scenarios(fake)
        port = fake.path
    try:
        return run_live(port, run_for=args.run_for, log_path=args.log, config=config,
                        max_speed_mm_s=args.max_speed, stop_event=stop)
    finally:
        if fake is not None:
            fake.close()


if __name__ == "__main__":
    sys.exit(main())
