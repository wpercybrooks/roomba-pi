"""JSONL run log: one object per line, append-only, replayable.

    {"t": <monotonic>, "kind": "...", "run": "<id>", ...}

Every record carries a ``run`` id. ``--log`` appends, so a file can hold several
runs; ``replay()`` scopes itself to the most recent ``meta`` record, which also
means the log always says which config produced what you are replaying.

``raw`` keeps the exact 80-byte payload, so a replay feeds the reflex precisely
what it saw the first time — no re-deriving, no drift.
"""

from __future__ import annotations

import json
import os
import uuid
from typing import Iterator

from roverd.oi import Sensors


def frame_to_dict(s: Sensors) -> dict:
    """The fields the reflex consumes, plus the raw payload for exact replay."""
    b = s.bumps_wheeldrops
    return {
        "bumps": {
            "bump_left": b.bump_left,
            "bump_right": b.bump_right,
            "wheeldrop_left": b.wheeldrop_left,
            "wheeldrop_right": b.wheeldrop_right,
        },
        "cliff_left": s.cliff_left,
        "cliff_front_left": s.cliff_front_left,
        "cliff_front_right": s.cliff_front_right,
        "cliff_right": s.cliff_right,
        "wall": s.wall,
        "distance": s.distance,
        "angle": s.angle,
        "voltage": s.voltage,
        "current": s.current,
        "temperature": s.temperature,
        "battery_charge": s.battery_charge,
        "battery_capacity": s.battery_capacity,
        "charger_state": s.charger_state,
        "open_interface_mode": s.open_interface_mode,
        "encoder_counts_left": s.encoder_counts_left,
        "encoder_counts_right": s.encoder_counts_right,
        "sane": s.sane,
        "raw": s.raw.hex() if s.raw else None,
    }


def dict_to_frame(d: dict) -> Sensors:
    """Rebuild the exact Sensors namedtuple from a logged ``frame`` record."""
    from pycreate2.packets import SensorPacketDecoder

    raw = d.get("raw")
    if not raw:
        raise ValueError("frame record has no raw payload; cannot replay exactly")
    s = SensorPacketDecoder(bytes.fromhex(raw))
    return s._replace(sane=bool(d.get("sane", True)))


class RunLog:
    """Append-only writer. ``path`` is never truncated: consecutive runs append.

    Each instance gets its own ``run_id`` so a file holding several runs can
    still be replayed one run at a time.
    """

    SYNC_EVERY = 20

    def __init__(self, path: str):
        self.path = path
        self.run_id = uuid.uuid4().hex[:12]
        self._fh = open(path, "a", encoding="utf-8")
        self._writes = 0

    def write(self, kind: str, t: float, **fields) -> dict:
        record = {"t": t, "kind": kind, "run": self.run_id}
        record.update(fields)
        self._fh.write(json.dumps(record, separators=(",", ":"), default=_default) + "\n")
        self._fh.flush()
        self._writes += 1
        if self._writes % self.SYNC_EVERY == 0:
            os.fsync(self._fh.fileno())
        return record

    def close(self) -> None:
        try:
            self._fh.flush()
            os.fsync(self._fh.fileno())
        finally:
            self._fh.close()


def _default(obj):
    if isinstance(obj, tuple):
        return list(obj)
    return str(obj)


def read_log(path: str) -> Iterator[dict]:
    """Yield records in order; a torn final line (power cut) is skipped."""
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                yield json.loads(line)
            except json.JSONDecodeError:
                continue          # truncated last line: the run died mid-write


def last_run(records: list[dict]) -> list[dict]:
    """The records of the most recent run in a possibly appended-to log file."""
    metas = [i for i, r in enumerate(records) if r.get("kind") == "meta"]
    if not metas:
        return records
    start = metas[-1]
    run_id = records[start].get("run")
    if run_id is None:            # pre-run-id log: the file is a single run
        return records[start:]
    return [r for r in records[start:] if r.get("run") == run_id]
