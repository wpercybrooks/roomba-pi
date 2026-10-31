# The MIT License
#
# Copyright (c) 2017 Kevin Walchko
#
# This is basically the interface between the Create2 and pyserial

import serial # type:ignore
import struct


class SerialCommandInterface(object):
    """
    This class handles sending commands to the Create2. Writes will take in tuples
    and format the data to transfer to the Create.
    """

    def __init__(self):
        """
        Constructor.

        Creates the serial port, but doesn't open it yet. Call open(port) to open
        it.
        """
        self.ser = serial.Serial()

    def __del__(self):
        """
        Destructor.

        Closes the serial port
        """
        self.close()

    def open(self, port, baud=115200, timeout=0.2):
        """
        Opens a serial port to the robot.

        PATCH: default timeout shortened to 0.2 s (upstream's 1 s stalls a 20 Hz control loop), the
        banner prints removed (a library must not write to stdout), and an exclusive advisory lock is
        taken so two processes cannot drive one robot.
        """
        self.ser.port = port
        self.ser.baudrate = baud
        self.ser.timeout = timeout
        if self.ser.is_open:
            self.ser.close()
        self.ser.open()
        try:
            import fcntl
            fcntl.flock(self.ser.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except ImportError:
            pass          # Windows: no fcntl; the caller must not double-open
        except OSError as exc:
            self.ser.close()
            raise Exception('{} is already in use by another process'.format(port)) from exc
        if not self.ser.is_open:
            raise Exception('Failed to open {} at {}'.format(port, baud))

    def write(self, opcode, data=None):
        """
        Writes a command to the create. There needs to be an opcode and optionally
        data. Not all commands have data associated with it.

        opcode: see creaet api
        data: a tuple with data associated with a given opcode (see api)
        """
        msg = (opcode,)

        # Sometimes opcodes don't need data. Since we can't add
        # a None type to a tuple, we have to make this check.
        if data:
            msg += data
        # print(">> write:", msg)
        self.ser.write(struct.pack('B' * len(msg), *msg))

    def read(self, num_bytes):
        """
        Read a string of 'num_bytes' bytes from the robot.

        Arguments:
            num_bytes: The number of bytes we expect to read.
        """
        if not self.ser.is_open:
            raise Exception('You must open the serial port first')

        data = self.ser.read(num_bytes)

        return data

    def read_until(self, num_bytes, timeout=0.2):
        """
        PATCH: accumulate exactly num_bytes or give up after timeout. Returns whatever arrived — a short
        read is a normal outcome on this firmware.
        """
        import time as _time
        deadline = _time.monotonic() + timeout
        buf = bytearray()
        while len(buf) < num_bytes and _time.monotonic() < deadline:
            chunk = self.ser.read(num_bytes - len(buf))
            if chunk:
                buf += chunk
        return bytes(buf)

    def close(self):
        """
        Closes the serial connection.

        PATCH: silent (upstream prints "Closing port ..." on every close).
        """
        if self.ser.is_open:
            self.ser.close()
