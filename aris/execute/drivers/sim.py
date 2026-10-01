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
                 fail_why: str = "injected failure", tick: float = 0.002, paper=None,
                 tip_of=None):
        """`speed`: times real time (math.inf: at once).  `fail_at`: seconds of the motion
        clock at which the arm faults.  `tick`: wall seconds between two updates.
        `paper`: (normal (3,), offset) of the fake paper in this arm's base frame, the normal
        pointing up toward the arm (paper where normal . p = offset); `tip_of(Q (N,7)) ->
        (N,3)` the pen tip in the base frame.  Both are needed for `touch`."""
        if not speed > 0:
            raise ValueError("speed must be positive")
        self.arm_id = arm_id
        self.speed, self.tick = float(speed), float(tick)
        self.fail_at, self.fail_why = fail_at, fail_why
        self.paper, self.tip_of = paper, tip_of
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

    def touch(self, motion: Motion) -> Result:
        """Flies the whole down-and-up (the arm ends back at the hover) and answers the joints
        where the planned descent, carried on straight for `extra_depth`, meets the fake
        paper; failed("no contact") if it does not within that depth."""
        if self.paper is None or self.tip_of is None:
            return Result.failed("no paper in this simulation")
        r = self.move(motion.traj)
        if not r.done:
            return r
        q = contact(motion, self.paper, self.tip_of)
        if q is None:
            return Result.failed("no contact", self._q.copy())
        return Result.ok(q)

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


# --------------------------------------------------------------------------- the fake paper


def contact(motion: Motion, paper, tip_of) -> np.ndarray | None:
    """Where the descent of a touch motion meets the paper: the descent's joint samples (up
    to its lowest tip), then on straight in joint space at the last samples' rate for
    `extra_depth` of tip travel; the first place the tip's height above the paper turns
    negative, found between samples by bisection.  None if it never does."""
    normal, offset = np.asarray(paper[0], float), float(paper[1])
    q = motion.traj.q
    h = tip_of(q) @ normal - offset
    bottom = int(np.argmin(h))
    down = q[:bottom + 1]
    if len(down) >= 2 and motion.extra_depth > 0.0:
        # the direction of the last 2 mm of the descent (the samples near the bottom are
        # dense, the arm slowing to rest), carried on in 1 mm steps
        tips = tip_of(down)
        far = np.linalg.norm(tips - tips[-1], axis=1)
        j = int(np.flatnonzero(far >= 0.002)[-1]) if np.any(far >= 0.002) else 0
        if far[j] > 0.0:
            per_mm = (down[-1] - down[j]) / (far[j] * 1e3)
            k = int(np.ceil(motion.extra_depth * 1e3))
            down = np.vstack([down, down[-1] + np.arange(1, k + 1)[:, None] * per_mm])
    hd = tip_of(down) @ normal - offset
    below = np.flatnonzero(hd <= 0.0)
    if not len(below):
        return None
    i = int(below[0])
    if i == 0:
        return down[0].copy()
    a, b = down[i - 1], down[i]
    lo, hi = 0.0, 1.0
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if float(tip_of((a + mid * (b - a))[None])[0] @ normal) - offset > 0.0:
            lo = mid
        else:
            hi = mid
    return a + hi * (b - a)
