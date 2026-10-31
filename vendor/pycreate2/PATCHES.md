# Deviations from pycreate2 0.8.0

**Seven patches, no refactors.** Each exists because of something measured on this specific robot (a Roomba
614 over an FTDI FT231X cable at 115200 8N1). Re-basing onto a newer upstream — or swapping to a successor
library — means re-applying exactly this list.

Layout note: this file, `LICENSE` and `UPSTREAM` ship **inside** the package directory so they travel with the
code; `../_verify_patches.py` is the tool that proves these patches still hold.

## `create2api.py`

| # | Patch | Why |
| --- | --- | --- |
| 1 | `__del__` is a **no-op**; lifecycle is explicit via `shutdown()` | upstream's destructor calls `drive_stop()`, blanks the LEDs and then `stop()` = **opcode 173**, which puts the robot in Off mode where it ignores commands *and its own buttons*. Every clean program exit would brick the buttons. `shutdown()` zeroes velocity, sends `start()` (Passive) and closes — never 173 |
| 2 | `stop()` requires `confirm_off=True` | the Off-mode trap becomes a deliberate act instead of a cleanup default |
| 3 | `passive()` added as an alias of `start()` | the safe resting state deserves a name that says so |
| 4 | `get_sensors()` accumulates until 80 bytes or the serial timeout, and returns **`None`** on a short burst instead of raising | this firmware answers packet 100 as a bare 80-byte payload with no header and no checksum, and a burst can start late. A control loop needs to keep its last good frame, not take an exception |

## `createSerial.py`

| # | Patch | Why |
| --- | --- | --- |
| 5 | `open(port, baud, timeout=0.2)`: shorter default timeout, **no stdout banner**, **exclusive `flock`**; new `read_until(n, timeout)`; `close()` silent | the 1 s default stalls a 20 Hz loop; a library must not print (on open *or* close); two writers on one robot port is a corrupted mode command |

## `packets.py`

| # | Patch | Why |
| --- | --- | --- |
| 6 | `Sensors` gains `sane` (always True when it decodes) and `raw` (the 80 bytes) | the facade logs raw frames and validates plausibility itself, since there is no checksum to lean on |

## `__init__.py`

| # | Patch | Why |
| --- | --- | --- |
| 7 | `__version__` falls back to `"0.8.0+vendored"` when `importlib.metadata` cannot find the distribution | a vendored directory is not an installed package, so upstream's `version("pycreate2")` raises on import |

## Deliberately not changed

No type hints, no renaming, no refactoring, no new dependencies, no touching the opcode tables. The package
still imports as `pycreate2`.

## Verifying the patches

```
python vendor/_verify_patches.py        # ten checks, no hardware required
```

