"""The executor: one arm, one queue.  Runs the motions in order and says what happened.

For each motion: confirm the arm is able to move and stands at the motion's start (to
`start_tol`), send it to the driver (`draw` for a drawing motion, `move` for the rest), wait
for it, report it done, go on.  While the queue has nothing new, the arm holds.  The first
failure ends the run: the arm stops and holds, and the rest of the queue is not run.  The
executor never plans and knows nothing about other arms.  Every state change goes to the log.
"""
from __future__ import annotations

import threading
from collections import deque
from dataclasses import dataclass

import numpy as np

from aris.execute.drivers import Driver
from aris.execute.log import EventLog
from aris.execute.queue import End, Queue

# rad, largest joint difference between where the arm stands and where the next motion
# starts.  The rig does not state one yet (see docs/modules/execute.md); the server passes it.
START_TOL = 5e-3
REST_QD = 1e-3          # rad/s, "standing still" for the parked test


@dataclass(frozen=True)
class ArmRun:
    """How one arm's queue went."""
    arm_id: int
    phase: str
    status: str             # "finished", "failed" or "stopped"
    done: int               # motions run to the end
    why: str = ""
    failed_index: int = -1  # the motion that failed or was refused, -1 if none
    q: np.ndarray | None = None
    complete: bool = True   # False: the queue's end marker says the plan was cut short
    parked: bool = False    # finished, standing still where the last motion ended, able to move


class Executor:
    def __init__(self, arm_id: int, driver: Driver, log: EventLog, start_tol: float = START_TOL):
        self.arm_id, self.driver, self.log, self.start_tol = arm_id, driver, log, start_tol

    def run(self, queue: Queue, stop: threading.Event | None = None,
            poll: float = 0.01) -> ArmRun:
        """Blocks until the queue's end marker has been reached, a motion failed, or `stop`
        was set."""
        stop = stop or threading.Event()
        phase, done, cursor, ready, waiting = queue.phase, 0, queue.cursor(), deque(), False
        last_end = None
        self._log("started", phase)
        while True:
            if stop.is_set():
                return self._halt(phase, done, "stopped", "stop requested", -1)
            ready.extend(cursor.poll())
            if not ready:
                if not waiting:                                 # empty queue: hold
                    self.driver.hold()
                    self._log("holding", phase, why="queue empty", done=done)
                    waiting = True
                stop.wait(poll)
                continue
            waiting = False
            item = ready.popleft()
            if isinstance(item, End):
                s = self.driver.state()
                parked = bool(s.ok and np.max(np.abs(s.qd)) <= REST_QD and (
                    last_end is None or np.max(np.abs(s.q - last_end)) <= self.start_tol))
                self._log("finished", phase, done=done, complete=item.complete,
                          parked=parked, note=item.note, q=s.q)
                return ArmRun(self.arm_id, phase, "finished", done, item.note, -1, s.q,
                              item.complete, parked)
            refused = self._refuse_start(item.motion)
            if refused:
                return self._halt(phase, done, "failed", refused, item.index)
            self._log("motion started", phase, index=item.index, kind=item.motion.kind,
                      duration=float(item.motion.traj.t[-1] - item.motion.traj.t[0]))
            r = (self.driver.draw(item.motion) if item.motion.kind == "draw"
                 else self.driver.move(item.motion.traj))
            if not r.done:
                status = "stopped" if stop.is_set() else "failed"
                return self._halt(phase, done, status, r.why, item.index)
            done += 1
            last_end = item.motion.q_end
            self._log("motion done", phase, index=item.index, q=r.q)

    def _refuse_start(self, motion) -> str:
        s = self.driver.state()
        if not s.ok:
            return "arm not able to move: " + ", ".join(s.flags)
        gap = np.abs(s.q - motion.q_start)
        if float(gap.max()) > self.start_tol:
            j = int(gap.argmax())
            return (f"not at the start: joint {j + 1} is {gap[j]:.4g} rad away "
                    f"(tolerance {self.start_tol:g})")
        return ""

    def _halt(self, phase, done, status, why, index) -> ArmRun:
        if status == "stopped":
            self.driver.stop()
        self.driver.hold()
        q = self.driver.state().q
        self._log(status, phase, done=done, index=index, why=why, q=q)
        return ArmRun(self.arm_id, phase, status, done, why, index, q)

    def _log(self, event, phase, **fields):
        self.log.write(event, arm=self.arm_id, phase=phase, **fields)
