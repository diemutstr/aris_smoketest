"""The simulated arm with the calibration touch: the package's `SimArm` (a perfect arm, no
ROS) plus `touch` under simulated position control against a fake paper.  For
`aris-robot serve --sim-speed` and the tests."""
from __future__ import annotations

import numpy as np

from aris.execute.drivers import Result
from aris.execute.drivers.sim import SimArm
from aris.kernel.retime import sample

from aris_robot import touch as T

TICK = 0.004            # s of motion between two readings


class SimPosition:
    """Flies a trajectory under position control, one reading every TICK of motion time: the
    actual joints follow the commanded ones exactly until the pen tip meets the fake paper,
    where the tip stops (the commanded joints go on: the lag); the force is the paper's for
    where the arm would be without the paper (plus noise): it rises as the arm pushes on, from
    the moment the tip arrives, also on an arm that trails.  A cancel stops it where it is."""

    def __init__(self, q0, paper: T.FakePaper, noise: float = 0.02, seed: int = 0,
                 trail_s: float = 0.0):
        self.q, self.paper, self.trail_s = np.array(q0, float), paper, trail_s
        self.rng = np.random.default_rng(seed)
        self.noise, self.flights = noise, []

    def _F(self, q):
        return self.paper.force(q) + self.noise * self.rng.standard_normal(3)

    def _above(self, q) -> bool:
        return self.paper is None or \
            float(self.paper.kin.tip(q)[0] @ self.paper.n) >= self.paper.c

    def fly(self, traj, watch) -> str:
        """`trail_s`: the actual arm follows the commanded trajectory that much late (a
        controller that trails), and catches up after its end."""
        gap = float(np.abs(traj.q[0] - self.q).max())
        if gap > 1e-6:
            return f"the trajectory starts {gap:.3g} rad away"
        self.flights.append(traj)
        t0, t1 = traj.t[0], traj.t[-1]
        at = lambda u: sample(traj, [min(max(u, t0), t1)])[0][0]       # noqa: E731
        u_free, stopped = t0, None
        for t in np.arange(t0, t1 + self.trail_s + TICK, TICK):
            u = min(t, t1 + self.trail_s) - self.trail_s    # where the actual arm would be
            q_free = at(u)
            if stopped is None and not self._above(q_free):
                lo, hi = u_free, u                       # where the tip meets the paper
                for _ in range(40):
                    mid = 0.5 * (lo + hi)
                    lo, hi = (mid, hi) if self._above(at(mid)) else (lo, mid)
                stopped = at(lo)
            if stopped is None:
                u_free = u
            self.q = q_free if stopped is None else stopped
            if watch(self.q, self._F(q_free), min(t, t1)):
                return "cancelled"
        if stopped is None:
            self.q = traj.q[-1].copy()
        return ""

    def forces(self, seconds: float) -> list:
        return [self._F(self.q) for _ in range(max(3, int(seconds / TICK)))]

    def joints(self):
        return self.q.copy()


class SimTouchArm(SimArm):
    """`SimArm` with `touch`.  `paper_m`: where the fake paper is, m above the nominal paper."""

    def __init__(self, rig, arm_id: int, q0, speed: float = 1.0, paper_m: float = 0.0,
                 bias: float = 1.0, settings: T.TouchSettings = T.TouchSettings()):
        super().__init__(arm_id, q0, speed=speed)
        self.settings, self.paper_m, self.bias = settings, paper_m, bias
        self.retarget(rig)

    def retarget(self, rig) -> None:
        """A new rig (a new calibration): the paper and the pen move with it."""
        self.kin = T.Kinematics.of(rig, self.arm_id)
        self.paper = T.FakePaper(self.kin, rig.paper(self.arm_id), self.paper_m, bias=self.bias)

    def touch(self, motion) -> Result:
        with self._lock:
            if self._fault or self._stopped:
                return Result.failed("arm will not move; recover first", self._q.copy())
            pos = SimPosition(self._q, self.paper)
        r = T.touch(motion, pos, self.kin, self.settings)
        with self._lock:
            self._q, self._qd = pos.q.copy(), np.zeros(7)
            if r.held:
                self._stopped = True
        return Result.ok(r.q_contact) if r.done else Result.failed(r.why, pos.q.copy())

    def guide(self, motion) -> Result:
        """A simulated person seats the pen at once, where the arm hovers: registered."""
        with self._lock:
            if self._fault or self._stopped:
                return Result.failed("arm will not move; recover first", self._q.copy())
            return Result(True, "check", self._q.copy())
