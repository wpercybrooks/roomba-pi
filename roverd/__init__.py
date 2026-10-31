"""roverd — the only process that touches the robot's serial port.

Safety-first v0: OI facade, command arbiter, reflex state machine, JSONL run log,
an independent watchdog thread, and the CLI that ties them together.
"""
