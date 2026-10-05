"""The calibration hand-over in serve: for a mark job, each arm's phase is flown by the
calibration driver (calib.py, libfranka directly) instead of the arm's ROS stack.

Before the phase: the arm's stack is paused (its process group stopped and waited for, so its
FCI connection is closed), then the calibration driver connects.  After the phase: the
calibration driver disconnects, the stack is resumed, and serve waits until the stack reports
the arm's joints again.  A mark job runs one arm per phase, so the other arms keep their stacks,
parked.

One FCI connection per robot at a time is guaranteed by serve being the only process that
starts either: it owns the stacks (Stacks) and the calibration drivers, does the two steps in
this order under one job (one job at a time), and gives up the phase rather than connect when a
stack does not exit.  libfranka itself refuses a second connection to the same robot, which is
the backstop.
"""
from __future__ import annotations

import time
from contextlib import contextmanager


class Switch:
    """A slot's driver for a mark job: the ROS driver, or the calibration driver while that
    arm's phase runs.  Every verb goes to the one in charge."""

    def __init__(self, slot: str, base):
        self.arm_id, self.base, self.calib = slot, base, None

    def __getattr__(self, name):
        return getattr(self.calib if self.calib is not None else self.base, name)


class HandOver:
    """`around(phase)` for runner.run_job.  `make_calib(slot, say)` -> a connected CalibArm.
    `stacks`: serve's Stacks, or None (simulated arms: no stack to stop)."""

    def __init__(self, switches: dict, make_calib, stacks, say, stack_wait_s: float = 90.0):
        self.switches, self.make_calib, self.stacks = switches, make_calib, stacks
        self.say, self.stack_wait_s = say, stack_wait_s

    @contextmanager
    def around(self, phase):
        taken, why = [], ""
        try:
            for a in phase.active:
                why = self._to_calib(a)
                if why:
                    break
                taken.append(a)
            yield why
        finally:
            for a in taken:
                self._to_stack(a)

    def _to_calib(self, slot) -> str:
        if self.stacks is not None:
            why = self.stacks.pause(slot)
            if why:
                self.stacks.resume(slot)
                return why
        self.say("calibration driver: stack stopped", arm=slot)
        try:
            self.switches[slot].calib = self.make_calib(slot, self.say)
        except Exception as e:                   # no connection: the stack takes the arm back
            if self.stacks is not None:
                self.stacks.resume(slot)
            return f"the calibration driver could not connect: {type(e).__name__}: {e}"
        self.say("calibration driver: connected", arm=slot,
                 q=[float(x) for x in self.switches[slot].state().q])
        return ""

    def _to_stack(self, slot) -> None:
        sw = self.switches[slot]
        calib, sw.calib = sw.calib, None
        if calib is not None:
            calib.close()
        self.say("calibration driver: disconnected", arm=slot)
        if self.stacks is None:
            return
        self.stacks.resume(slot)
        t_end = time.monotonic() + self.stack_wait_s
        while time.monotonic() < t_end:
            if "no joint states" not in sw.base.state().flags:
                self.say("calibration driver: stack back", arm=slot,
                         q=[float(x) for x in sw.base.state().q])
                return
            time.sleep(0.5)
        self.say("calibration driver: stack not back", arm=slot, waited_s=self.stack_wait_s)
