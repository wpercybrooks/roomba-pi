# roomba-pi — a Roomba 614 rover control stack (v0)

An old iRobot **Roomba 614** (600-series chassis) driven over its native serial port with the iRobot Open
Interface. The controller is a Raspberry Pi 4. This is the safety-first v0: a reflex loop, an OI codec, an
arbiter, a run log, a watchdog — all tested against an emulated robot, with no robot on the desk.

```
sensors (80 bytes) ──▶ reflex state machine ──▶ Intent ──▶ arbiter ──▶ RoverOI.command
                                  │                       (TTL + cap)
                                  └──▶ JSONL log ──▶ replay (identical transitions)
```

## ⚠ Safety: this robot has no link watchdog of its own

If the controller dies, the robot **keeps its last commanded velocity** — nothing on the Roomba notices. That is
the single most important fact in this repository. `roverd` limits the damage with four independent things:

1. **Every intent expires.** The arbiter refuses any velocity older than 500 ms; the reflex refreshes its
   intent every 50 ms. A dead caller becomes a stopped robot within half a second.
2. **An independent watchdog thread.** If the loop misses its heartbeat by 300 ms, a separate thread writes
   `drive_direct(0, 0)` itself. It shares no lock with the loop it guards.
3. **Startup refusal.** No plausible 80-byte sensor frame within 500 ms → exit non-zero, never enter Safe. A
   controller that cannot see must not drive.
4. **A Passive exit, never Off.** Shutdown is zero velocity → `START` (128) → close. It never sends `STOP`
   (173), which would put the robot in Off mode with its own buttons dead.

Even with all four, treat this stack as bench-grade. There is no real-time scheduling, no hardware E-stop, and
no sensor tells you the controller is alive.

## What is here

| Module | What it does |
| --- | --- |
| `roverd/oi.py` | Safety-first facade over the vendored `pycreate2`: exclusive lock, 100 ms serial timeout, plausibility checks, mode tracked from commands sent, `poll()` that never raises after the first good frame |
| `roverd/reflex.py` | `DRIVE / BACK_OFF / ROTATE / STOPPED` state machine, 20 Hz, deterministic (pluggable clock) |
| `roverd/arbiter.py` | Priority (`safety > teleop > agent > reflex`), 500 ms TTL ceiling, 200 mm/s cap |
| `roverd/log.py` | JSONL run log (frames, transitions, intents, stops, faults), run-id scoped for replay, plus the exact frame codec |
| `roverd/watchdog.py` | The independent heartbeat thread |
| `roverd/main.py` | CLI: `--port`, `--run-for`, `--log`, `--replay`, `--speed`, `--max-speed`; 20 Hz loop; startup self-check; signal handling |
| `sim/fake_roomba.py` | PTY-backed OI emulator: 80-byte packet 100 replies, mode tracking, the Off-mode trap, scripted bumps/cliffs/stalls/wheel drops |
| `vendor/pycreate2/` | Upstream 0.8.0 (MIT), vendored and patched — see `vendor/pycreate2/PATCHES.md` |

`pyserial` is the **only** dependency. `pycreate2` is vendored under `vendor/` (not installed, not published as
a fork); the paths are wired in `roverd/oi.py`.

## Hardware bill of materials

| Part | Notes |
| --- | --- |
| iRobot Roomba 614 | 600-series OI, same protocol as the Create 2 |
| FTDI **FT231X** USB-serial cable | `0403:6015`, 7-pin mini-DIN, TTL, appears as `/dev/ttyUSB0`; driver in-tree |
| Raspberry Pi 4 (8 GB) | runs `roverd`; Python ≥3.11 |
| USB power bank | powers the Pi; the robot runs on its own pack |
| microSD ≥16 GB (A1/A2) | endurance card preferred: this device logs continuously |

Link settings: **115200 8N1**. The adapter drives 3.3 V into the robot's 5 V TTL input; that is fine.

## Firmware quirks measured on this specific robot

These override the protocol PDF and every LLM's memory of a Create 2. They are why the code looks the way it
does (measured 2026-10-30/31):

- **Replies are bare payloads.** No `19/N/id` header, no checksum. `142 100` → exactly **80 bytes**; `142 0` →
  26; `142 7` → 1. A parser waiting for a checksum waits forever.
- **Length is the contract; plausibility is the only integrity check.** Voltage 12 000–20 000 mV, temperature
  0–60 °C, capacity ≤ 5000 mAh, charge ≤ capacity. A frame failing any range becomes `sane=False` and never
  replaces the last good state.
- **`open_interface_mode` reads one higher than the spec** (Passive 2, Safe 3). Mode is tracked from the
  commands we sent; the field is only a cross-check.
- **Encoder ticks are unreliable between runs** (0.47 vs 0.69 mm/tick; counters seen at 65525/65529 and 7/7).
  Stall detection uses the `distance` field; ticks are a "did both wheels turn?" check only. Encoders idle at
  3/3 counts, so the stall threshold is a deadband, not zero.
- **`distance` and `angle` are deltas since the previous read**, not cumulative totals.
- **Never send 173.** `STOP` puts the robot in Off mode, where it "will no longer respond to commands" —
  buttons included — until another `128`, a `7`, a lift, or a battery cycle. `START` (128) is the resting
  state: motors off, buttons live, pack charges, sleeps after 5 min.
- **pycreate2's `__del__` sends 173.** The vendored copy neutralises it; do not reintroduce a destructor that
  writes to the port.
- **`wake()` does not work with the official iRobot cable** (the library's own source says so). Keep the robot
  awake with a periodic `start()`, not `wake()`.
- **Idle draw while the OI is awake is ~100–160 mA** (≈2.5 W); overnight it eats about 60% of the pack. The
  simulator models 159 mA. Unplug the cable when you are done.
- **The bump bits decode into swapped field names in the vendored decoder.** `BumpsAndWheelDrop.bump_left` is
  filled from OI bit 0 (bump *right*) and `bump_right` from bit 1. `roverd.oi.bump_side()` corrects this once,
  where the protocol knowledge belongs. Verify the side on real hardware before trusting an unattended turn.

Battery seen on this unit: 2068 mAh capacity, 1887 mAh charge, ~15.89 V at rest; runtime still unmeasured.

## Run it

Python **3.12.3** was used for development; the project requires ≥3.11.

```bash
python3 -m venv .venv
.venv/bin/pip install -e ".[dev]"

# 1. 19 tests, no hardware, under 5 s
.venv/bin/pytest -q

# 2. a 30 s run against a real PTY running the simulator (~30 s wall-clock)
.venv/bin/python -m roverd.main --port fake --run-for 30 --log /tmp/run.jsonl

# 3. replay that log and compare the transition sequence
.venv/bin/python -m roverd.main --replay /tmp/run.jsonl
```

`--port fake` spawns `sim.FakeRoomba` and connects to its PTY — there is no `/dev/fake` to create. Any other
`--port` value is a real serial device:

```bash
.venv/bin/python -m roverd.main --port /dev/ttyUSB0 --log /tmp/real.jsonl
```

`--run-for` defaults to "until SIGINT/SIGTERM"; shutdown is always zero → `128` → close. `--log` appends:
every run is stamped with a run id, and `--replay` replays the most recent run in the file.

Also useful:

```bash
python vendor/_verify_patches.py     # prove the vendored patches are intact, no hardware
```

`--speed` sets the reflex drive speed and `--max-speed` the arbiter cap, both in mm/s.

### The simulator

`FakeRoomba` owns a PTY pair and implements the OI subset this stack uses. It integrates commanded wheel
velocities at 10 Hz with the real geometry (`wheelbase ≈ 235 mm`; `angle = (Δright − Δleft) / 235 × 180/π`),
models the ±1 mode offset, the Off-mode trap, and the 159 mA idle draw. Opcodes it accepts but does not act on
are ignored; anything else it does not recognise is recorded in `fake.ignored` instead of desyncing the parser.
Scenarios are scripted in sim time:

```python
fake = FakeRoomba()
fake.bump_after(5.0, "left")
fake.cliff_after(8.0)
fake.stall_after(12.0)
fake.clear_stall_after(15.0)
fake.wheel_drop_after(18.0, "right")
```

Tests step physics by hand (`fake.step(dt)`) with an injected clock, so the simulation itself is deterministic
and sub-second. The two real-time tests (watchdog firing, startup refusal) add a few hundred milliseconds; the
whole suite stays under 5 s.

## Safety invariants (these have tests, not trust)

1. The device node is opened with an exclusive lock; a second `RoverOI` raises `PortBusy`.
2. No intent older than 500 ms can produce motion.
3. The arbiter clips to 200 mm/s even when a caller asks for 500.
4. Cliff/wheel-drop latches survive further sensor frames until an explicit `reset()`.
5. `passive()` sends `128`; afterwards `drive_direct()` raises (zeros excepted — the watchdog needs them). Opcode
   `173` appears in no default path, and a test asserts that over a whole session and by scanning the modules.
6. Startup refuses to enter Safe without a plausible frame within 500 ms.
7. A short or implausible burst sets `sane=False` and keeps the last good frame.
8. The watchdog zeroes the robot within 300 ms of a missed heartbeat and holds no lock shared with the loop.
9. Every velocity write goes through the facade (`RoverOI.command`); the AST call-graph test fails CI otherwise.

## Working on this repo

The checkout may live on exFAT (an SD card, a shared folder), where every file is mode 777 and Git reports the
whole tree as modified with empty diffs. This clone sets `core.filemode false`, so `git status` shows only real
content changes. If you re-clone onto such media, set it again: `git config core.filemode false`.

## Layout

```
roverd/   oi.py arbiter.py reflex.py log.py watchdog.py main.py
sim/      fake_roomba.py
tests/    test_oi.py test_reflex.py test_replay.py
vendor/   _verify_patches.py + pycreate2/ (MIT, patched)
```

## Licence

MIT — see `LICENSE`. Vendored `pycreate2` 0.8.0 is MIT, copyright Kevin Walchko and contributors; its notice
travels inside `vendor/pycreate2/LICENSE`. Third-party details and the patch list are in `ATTRIBUTION.md` and
`vendor/pycreate2/PATCHES.md`.
