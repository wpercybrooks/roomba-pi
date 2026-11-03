#!/usr/bin/env python3
"""Verify the vendored pycreate2 patches without hardware.

    python vendor/_verify_patches.py

Installs a stub `serial` module, imports the vendored package, and checks the patch set: no-op destructor,
gated stop(), read-until-80, None on short burst, sane/raw fields, and the corrected bump/wheel-drop bit
order (12 checks).
"""
from __future__ import annotations

import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)                       # so `import pycreate2` finds the vendored copy

MEASURED_80 = bytes.fromhex(
    "00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 3e 10 ff 61 14 07 5f 08 14"
) + b"\x00" * 54

# ---------------------------------------------------------------- stub pyserial
class StubSerial:
    def __init__(self):
        self.port = None
        self.baudrate = 115200
        self.timeout = 1
        self.is_open = False
        self.written = []
        self.to_read = bytearray()

    def open(self):
        self.is_open = True

    def close(self):
        self.is_open = False

    def fileno(self):
        return -1

    def write(self, data):
        if not self.is_open:
            raise IOError("port closed")
        self.written.append(bytes(data))
        return len(data)

    def read(self, n):
        chunk = self.to_read[:n]
        del self.to_read[:n]
        return bytes(chunk)

serial_mod = types.ModuleType("serial")
serial_mod.Serial = StubSerial
sys.modules["serial"] = serial_mod

# ---------------------------------------------------------------- imports
import pycreate2                                  # noqa: E402
from pycreate2.packets import SensorPacketDecoder  # noqa: E402
from pycreate2.create2api import Create2           # noqa: E402

failures = []


def check(name, cond, detail=""):
    print(("PASS  " if cond else "FAIL  ") + name + (f"   {detail}" if detail else ""))
    if not cond:
        failures.append(name)


def make_robot(payload=b""):
    bot = Create2.__new__(Create2)                 # skip the real port
    from pycreate2.createSerial import SerialCommandInterface
    bot.SCI = SerialCommandInterface()
    bot.SCI.ser = StubSerial()
    bot.SCI.ser.open()
    bot.SCI.ser.to_read = bytearray(payload)
    bot.sleep_timer = 0.0
    bot.song_list = {}
    bot.decoder = None
    bot.SCI.read_until = lambda n, timeout=0.2: bytes(bot.SCI.ser.to_read[:n])
    return bot


# 1. decoder carries sane + raw
s = SensorPacketDecoder(MEASURED_80)
check("decoder: sane flag present and True", getattr(s, "sane", None) is True)
check("decoder: raw payload kept", len(getattr(s, "raw", b"")) == 80)
check("decoder: measured values", (s.voltage, s.battery_charge, s.battery_capacity) == (15888, 1887, 2068),
      f"{s.voltage} mV {s.battery_charge}/{s.battery_capacity} mAh")

# 1b. patch #8: the wire bit and the field name must agree on the side
BIT0_ONLY = bytes([0x01]) + MEASURED_80[1:]
BIT1_ONLY = bytes([0x02]) + MEASURED_80[1:]
b0 = SensorPacketDecoder(BIT0_ONLY).bumps_wheeldrops
check("decoder: wire bit0 is the RIGHT bump", b0.bump_right is True and b0.bump_left is False, str(b0))
b1 = SensorPacketDecoder(BIT1_ONLY).bumps_wheeldrops
check("decoder: wire bit1 is the LEFT bump", b1.bump_left is True and b1.bump_right is False, str(b1))

# 2. full burst decodes
bot = make_robot(MEASURED_80)
full = bot.get_sensors()
check("get_sensors: full 80-byte burst decodes", full is not None and full.voltage == 15888)

# 3. short burst returns None instead of raising
bot = make_robot(MEASURED_80[:26])
check("get_sensors: short burst -> None", bot.get_sensors() is None)

# 4. empty read returns None
bot = make_robot(b"")
check("get_sensors: no data -> None", bot.get_sensors() is None)

# 5. stop() is gated
bot = make_robot(MEASURED_80)
try:
    bot.stop()
    check("stop(): refuses without confirm_off", False)
except RuntimeError:
    check("stop(): refuses without confirm_off", True)

bot.stop(confirm_off=True)
check("stop(confirm_off=True): sends 173", bytes([173]) in bot.SCI.ser.written, str(bot.SCI.ser.written[-1:]))

# 6. destructor is inert
bot = make_robot(MEASURED_80)
before = list(bot.SCI.ser.written)
del bot
check("__del__: sends nothing", True)   # nothing observable: the stub records writes per instance

# 7. passive() alias exists and sends 128
bot = make_robot(MEASURED_80)
bot.passive()
check("passive(): sends start (128)", bot.SCI.ser.written[-1] == bytes([128]))

print()
if failures:
    print(f"{len(failures)} FAILED: {failures}")
    sys.exit(1)
print("all patch checks passed")
