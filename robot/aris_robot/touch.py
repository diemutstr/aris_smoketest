"""The calibration touch, under position control.  No ROS.

A "touch" motion (aris.types.Motion, kind "touch") is a checked down-and-up from a hover pose
onto the nominal paper.  The arm flies its descent half with the stock joint-trajectory
controller, at the motion's own (slow) timing, and watches its force estimate against an air
zero taken standing at the hover:

  contact   the force above the zero over `contact_n` (1.0 N) for `contact_ticks` readings
            in a row: the trajectory is cancelled; the joints of the first of those readings
            are the touch (DESIGN 6 step 1: the encoders say where the paper is, the force
            only when)
  further   the planned end reached without contact: straight on in the same direction for
            the motion's `extra_depth` (at most `extra_max`), at `extra_speed`, the hand
            keeping its orientation (joints by the arm's IK, as the planner made the descent)
  cap       the force above the zero over `cap_n` (3 N) at any reading: stop and hold where
            it is, report; no way back is flown (a person looks first)
  back      after a contact, or none: back to the hover along the path flown, reversed

`touch` drives anything with the small `PositionArm` interface: the real arm (driver.py, the
trajectory controller and the robot state broadcaster), or the simulated one in the tests.
"""
from __future__ import annotations

import dataclasses
from dataclasses import dataclass, fields
from typing import Protocol

import numpy as np

from aris.kernel.retime import retime
from aris.types import JointPath, Refusal, Trajectory

from aris_robot.force import ForceSettings, Tare, normal_force


@dataclass(frozen=True)
class TouchSettings:
    contact_n: float = 1.0         # N above the air zero: contact
    contact_ticks: int = 3         # readings in a row
    cap_n: float = 3.0             # N above the air zero at any reading: stop and hold
    extra_speed: float = 0.002     # m/s past the planned end
    extra_max: float = 0.03        # m, the most a touch may go past its planned end
    step: float = 0.001            # m between IK samples of that extension
    tare_s: float = 0.2
    tare_max_n: float = 8.0
    sign: float = 1.0

    @staticmethod
    def from_site(block: dict | None, force_sign: float = 1.0) -> "TouchSettings":
        names = {f.name for f in fields(TouchSettings)}
        kw = {k: v for k, v in (block or {}).items() if k in names}
        kw.setdefault("sign", force_sign)
        return TouchSettings(**kw)


class PositionArm(Protocol):
    def fly(self, traj: Trajectory, watch) -> str:
        """Fly `traj` under position control at its own timing, calling `watch(q, F)` for every
        reading (joints, external force on the arm in the base frame) as they come; when it
        returns True, cancel and stand still.  -> "" (reached the end), "cancelled", or why it
        failed."""

    def forces(self, seconds: float) -> list:
        """The force readings (3,) taken standing still for `seconds`."""

    def joints(self) -> np.ndarray:
        """Where the arm stands."""


@dataclass
class TouchResult:
    done: bool                     # touched the paper and came back to the hover
    why: str = ""
    q_contact: np.ndarray | None = None
    depth_past_end: float = 0.0    # m beyond the planned end at contact (0: within the plan)
    held: bool = False             # stopped where it was (force cap): no way back flown
    air_zero: float = 0.0


def descent_half(motion, kin) -> tuple[Trajectory, np.ndarray]:
    """The trajectory from the hover to the deepest knot (stands still there), and the pen
    tips at its knots."""
    tips = motion.tip_base if motion.tip_base is not None else kin.tip(motion.traj.q)
    tips = np.asarray(tips, float)
    k = int(np.argmax((tips - tips[0]) @ np.asarray(kin.down, float)))
    tr = motion.traj
    qd = tr.qd[:k + 1].copy()
    qd[-1] = 0.0                   # the path turns back here: the arm is at rest at the cusp
    return Trajectory(tr.t[:k + 1].copy(), tr.q[:k + 1].copy(), qd), tips[:k + 1]


def straight_on(arm, limits, rules, q, direction, depth: float, speed: float,
                step: float = 0.001):
    """From q, the pen tip straight along `direction` for `depth` with the hand's orientation
    and joint 7 kept: joints by the arm's IK (the branch nearest the previous sample), timed
    at `speed` along the line.  -> Trajectory, or a Refusal."""
    d_unit = np.asarray(direction, float) / np.linalg.norm(direction)
    q = np.asarray(q, float)
    T0 = arm.fk(q[None])[0]
    d = np.linspace(0.0, depth, max(2, int(round(depth / step)) + 1))
    path, prev = [q], q
    for di in d[1:]:
        T = T0.copy()
        T[:3, 3] += di * d_unit
        Q, ok = arm.ik(T[None], q[6])
        if not ok[0].any():
            return Refusal("unreachable", f"no arm configuration {di * 1000:.0f} mm on")
        cand = Q[0][ok[0]]
        prev = cand[np.argmin(np.abs(cand - prev).max(axis=1))]
        if np.abs(prev - path[-1]).max() > 0.05:
            return Refusal("branch", "the line would jump between arm shapes")
        path.append(prev)
    return retime(JointPath(np.array(path)), limits,
                  dataclasses.replace(rules, draw_speed=speed), s=d)


@dataclass(frozen=True)
class Kinematics:
    """What the touch needs of the arm and the rig: the arm model, its limits, the rules, the
    paper normal (base frame, up) as `down` = minus it."""
    arm: object
    rules: object
    normal: np.ndarray

    @property
    def down(self) -> np.ndarray:
        return -np.asarray(self.normal, float) / np.linalg.norm(self.normal)

    def tip(self, Q) -> np.ndarray:
        return self.arm.tip(np.atleast_2d(Q))

    @staticmethod
    def of(rig, arm_id: int) -> "Kinematics":
        return Kinematics(rig.arm(arm_id), rig.rules(), np.asarray(rig.paper(arm_id).normal))


class _Watch:
    """Reads the force against the air zero during a flight; remembers the path flown."""

    def __init__(self, s: TouchSettings, zero: float, kin: Kinematics):
        self.s, self.zero, self.kin = s, zero, kin
        self.run, self.first_q, self.contact_q, self.over = 0, None, None, ""
        self.flown = []

    def __call__(self, q, F) -> bool:
        q = np.asarray(q, float)
        self.flown.append(q)
        f = normal_force(F, self.kin.normal, self.s.sign) - self.zero
        if f > self.s.cap_n:
            self.over = f"pen force {f:.2f} N above the cap {self.s.cap_n} N"
            return True
        if f > self.s.contact_n:
            self.first_q = q if self.run == 0 else self.first_q
            self.run += 1
            if self.run >= self.s.contact_ticks:
                self.contact_q = self.first_q
                return True
        else:
            self.run = 0
        return False


def touch(motion, pos: PositionArm, kin: Kinematics, s: TouchSettings) -> TouchResult:
    if motion.extra_depth > s.extra_max:
        return TouchResult(False, f"extra depth {motion.extra_depth * 1000:.0f} mm is more "
                                  f"than the cap {s.extra_max * 1000:.0f} mm")
    tare = Tare(ForceSettings(tare_max_n=s.tare_max_n, cap_n=max(s.cap_n, 1.0)))
    for F in pos.forces(s.tare_s):
        tare.add(normal_force(F, kin.normal, s.sign))
    zero = tare.result()
    if isinstance(zero, Refusal):
        return TouchResult(False, f"{zero.reason}: {zero.detail}")
    descent, tips = descent_half(motion, kin)
    watch = _Watch(s, zero, kin)
    out = TouchResult(False, air_zero=zero)
    status = pos.fly(descent, watch)
    knots = list(descent.q)
    if status == "" and watch.contact_q is None and not watch.over:
        if motion.extra_depth > 0.0:
            direction = tips[-1] - tips[0]
            ext = straight_on(kin.arm, kin.arm.limits, kin.rules, pos.joints(), direction,
                              motion.extra_depth, s.extra_speed, s.step)
            if isinstance(ext, Refusal):
                status = f"cannot go on past the planned end: {ext.detail}"
            else:
                status = pos.fly(ext, watch)
                knots += list(ext.q[1:])
        if status == "" and watch.contact_q is None and not watch.over:
            status = (f"no contact within {motion.extra_depth * 1000:.0f} mm past the planned "
                      f"end")
    if watch.over:
        out.why, out.held = watch.over, True
        return out
    if watch.contact_q is not None:
        out.done, out.q_contact = True, watch.contact_q
        depth = (kin.tip(watch.contact_q)[0] - tips[-1]) @ kin.down
        out.depth_past_end = max(0.0, float(depth))
    else:
        out.why = status if status not in ("", "cancelled") else "no contact"
    back = _way_back(pos.joints(), knots, motion.q_end, kin)
    went = pos.fly(back, lambda q, F: False)
    if went:
        out.done, out.why = False, f"{out.why + '; ' if out.why else ''}the way back: {went}"
    return out


def _way_back(q_now, knots, q_hover, kin: Kinematics):
    """From where the arm stands, back over the knots it passed, to the hover."""
    down = kin.down
    tips = kin.tip(np.array(knots))
    here = float(kin.tip(q_now)[0] @ down)
    passed = [q for q, p in zip(knots, tips) if float(p @ down) < here - 1e-6]
    path = np.array([np.asarray(q_now, float)] + passed[::-1] + [np.asarray(q_hover, float)])
    keep = np.concatenate([[True], np.abs(np.diff(path, axis=0)).max(axis=1) > 1e-9])
    return retime(JointPath(path[keep]), kin.arm.limits, kin.rules)


def manual_touch(kin: Kinematics, q_hover, depth: float, extra: float, speed: float = 0.005):
    """A touch motion from where the arm stands, for the hand tool (`aris-robot touch`): straight
    down `depth`, timed at `speed`, and back up the same way; not planned by the server and so
    not checked.  -> Motion, or a Refusal."""
    from aris.types import Motion
    down = straight_on(kin.arm, kin.arm.limits, kin.rules, q_hover, kin.down, depth, speed)
    if isinstance(down, Refusal):
        return down
    path = np.concatenate([down.q, down.q[-2::-1]])
    tips = kin.tip(path)
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(tips, axis=0), axis=1))])
    traj = retime(JointPath(path), kin.arm.limits, dataclasses.replace(kin.rules, draw_speed=speed),
                  s=s)
    if isinstance(traj, Refusal):
        return traj
    return Motion("touch", traj, tip_base=kin.tip(traj.q), extra_depth=extra)


class FakePaper:
    """The force a paper at `height` (m along the normal from the nominal paper; + is higher)
    with stiffness `k` pushes back with, from where the pen tip is.  For fake hardware and
    the tests: the robot's force estimate stands in for it."""

    def __init__(self, kin: Kinematics, rig_paper, height: float = 0.0, k: float = 5000.0,
                 bias: float = 0.0):
        self.kin, self.k, self.bias = kin, k, bias
        n = np.asarray(rig_paper.normal, float)
        self.n = n / np.linalg.norm(n)
        self.c = float(rig_paper.offset) + height

    def force(self, q) -> np.ndarray:
        pen = self.c - float(self.kin.tip(q)[0] @ self.n)
        return self.n * (self.bias + self.k * max(0.0, pen))
