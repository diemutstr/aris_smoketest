"""The simulated arm: the driver verbs on a perfect arm.

A move plays the trajectory in a background thread, reading the cubic between the samples
(`kernel.retime.sample`) at `speed` times real time, and lands exactly on the last sample.  It
has no contact, no compliance and no controller error: the arm is exactly where the trajectory
says, always.  For tests, a failure can be injected at a given time of the arm's motion clock
(seconds of motion flown since it was made, at the trajectory's own timing).
"""
from __future__ import annotations

import math
import threading
import time

import numpy as np

from aris.execute.drivers import ArmState, Result
from aris.kernel.retime import sample
from aris.types import Motion, Trajectory

START_GUARD = 1e-3      # rad, the driver's own refusal: a trajectory that starts elsewhere


class SimArm:
    def __init__(self, arm_id: int, q0, speed: float = 1.0, fail_at: float | None = None,
                 fail_why: str = "injected failure", tick: float = 0.002):
        """`speed`: times real time (math.inf: at once).  `fail_at`: seconds of the motion
        clock at which the arm faults.  `tick`: wall seconds between two updates."""
        if not speed > 0:
            raise ValueError("speed must be positive")
        self.arm_id = arm_id
        self.speed, self.tick = float(speed), float(tick)
        self.fail_at, self.fail_why = fail_at, fail_why
        self.clock = 0.0                    # s of motion flown, at the trajectories' timing
        self._q = np.asarray(q0, float).reshape(7).copy()
        self._qd = np.zeros(7)
        self._fault = ""                    # why the arm faulted, "" if not
        self._stopped = False
        self._moving = False
        self._lock = threading.Lock()
        self._halt = threading.Event()

    # ------------------------------------------------------------------ verbs

    def state(self) -> ArmState:
        with self._lock:
            flags = (("moving",) if self._moving else ("holding",))
            flags += (("stopped",) if self._stopped else ())
            flags += ((f"fault: {self._fault}",) if self._fault else ())
            return ArmState(self._q.copy(), self._qd.copy(),
                            not (self._fault or self._stopped), flags)

    def move(self, traj: Trajectory) -> Result:
        with self._lock:
            if self._fault or self._stopped:
                why = f"fault: {self._fault}" if self._fault else "stopped"
                return Result.failed(f"arm will not move ({why}); recover first", self._q.copy())
            gap = float(np.max(np.abs(traj.q[0] - self._q)))
            if gap > START_GUARD:
                return Result.failed(f"trajectory starts {gap:.4g} rad from the arm",
                                     self._q.copy())
            self._moving = True
            self._halt.clear()
        out: list[Result] = []
        worker = threading.Thread(target=self._play, args=(traj, out), daemon=True)
        worker.start()
        worker.join()
        return out[0]

    def draw(self, motion: Motion) -> Result:
        return self.move(motion.traj)

    def hold(self) -> None:
        with self._lock:
            if not self._moving:
                self._qd = np.zeros(7)

    def stop(self) -> None:
        with self._lock:
            self._stopped = True
        self._halt.set()

    def recover(self) -> Result:
        with self._lock:
            if self._moving:
                return Result.failed("still moving", self._q.copy())
            self._fault, self._stopped = "", False
            return Result.ok(self._q.copy())

    # ------------------------------------------------------------------ the player

    def _play(self, traj: Trajectory, out: list) -> None:
        duration = float(traj.t[-1] - traj.t[0])
        clock0, w0 = self.clock, time.perf_counter()
        while True:
            tau = duration if math.isinf(self.speed) else (time.perf_counter() - w0) * self.speed
            fail = self.fail_at is not None and clock0 <= self.fail_at < clock0 + duration \
                and clock0 + tau >= self.fail_at
            if fail:
                tau = self.fail_at - clock0
            if fail or self._halt.is_set() or tau >= duration:
                break
            q, qd, _ = sample(traj, [traj.t[0] + tau])
            with self._lock:
                self._q, self._qd = q[0], qd[0]
            self._halt.wait(self.tick)
        with self._lock:
            if fail:
                self._q = sample(traj, [traj.t[0] + tau])[0][0]
                self._fault = self.fail_why
                result = Result.failed(self.fail_why, self._q.copy())
            elif self._halt.is_set():
                result = Result.failed("stopped", self._q.copy())
            else:
                tau = duration
                self._q = traj.q[-1].copy()
                result = Result.ok(self._q.copy())
            self._qd = np.zeros(7)          # a perfect arm stops at once and holds
            self._moving = False
            self.clock = clock0 + tau
        out.append(result)
