"""The calibration touch, under position control.  No ROS.

A "touch" motion (aris.types.Motion, kind "touch") is a checked down-and-up from a hover pose
onto the nominal paper.  The arm flies its descent half with the stock joint-trajectory
controller, at the motion's own (slow) timing, and watches its force estimate:

  zero      the mean of `tare_readings` (20) readings standing still at the hover, before the
            descent starts (2026-10-08: a zero taken during the descent included the press
            when the paper was higher than planned, and every touch ran into the cap)
  arming    the detector arms `arm_after_s` (0.1 s, planned time) into each flight: a trip
            before that is the flight's own start jolt (it tripped 60 mm above the paper on
            2026-10-07), not the paper: the flight goes on
  contact   the force above the zero over `contact_n` (3 N) for `contact_ticks` (15)
            readings in a row: the trajectory is cancelled; the joints of the first of those
            readings are the touch (DESIGN 6 step 1: the encoders say where the paper is, the
            force only when)
  further   the planned end reached without contact: straight on in the same direction for
            the motion's `extra_depth` (at most `extra_max`), at `extra_speed`, the hand
            keeping its orientation (joints by the arm's IK, as the planner made the
            descent); this flight arms itself the same way
  cap       the force over the zero above `cap_n` (6 N): stop.  Armed, that is a CONTACT
            found by the cap (a steep rise): the touch is the first reading of the last run
            over `contact_n` in the kept history (the last `history_s` of readings), with a
            row saying how fast the force rose; the way back is flown.  Before arming (the
            first 0.1 s) it is a failure: stop and hold where it is, no way back is flown
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



def normal_force(F_ext_base, normal_base, sign: float = 1.0) -> float:
    """The external force on the arm (base frame) along the paper normal, times `sign`:
    positive when the paper pushes the pen up."""
    return float(sign) * float(np.dot(np.asarray(F_ext_base, float), np.asarray(normal_base)))


def tare(readings) -> float | Refusal:
    """The air zero: the mean of the readings.  Refused only when there are none to speak of:
    without a force signal the descent would be blind."""
    r = np.asarray(readings, float)
    if len(r) < 3:
        return Refusal("no_tare", f"only {len(r)} force readings in the air: no force signal")
    return float(r.mean())


@dataclass(frozen=True)
class TouchSettings:
    contact_n: float = 3.0         # N above the armed zero: contact
    contact_ticks: int = 15        # readings in a row
    cap_n: float = 6.0             # N above the hover's air zero at any reading: stop, hold
    extra_speed: float = 0.002     # m/s past the planned end (the one speed setting: the
                                   # planned descent is timed by the planner)
    arm_after_s: float = 0.1       # s of planned flight time before the detector arms (the
                                   # start jolt)
    tare_readings: int = 20        # readings at the hover averaged for the zero
    history_s: float = 1.0         # s of readings kept, to backdate a contact found by the cap
    extra_max: float = 0.03        # m, the most a touch may go past its planned end
    step: float = 0.001            # m between IK samples of that extension
    tare_s: float = 0.2
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
    air_zero: float = 0.0          # N, standing at the hover: what contact is judged against
    armed_zero: float | None = None   # the same (kept for the report's readers)
    by_cap: str = ""               # a contact found by the cap: how the force rose
    early_trips: int = 0           # readings over the threshold before arming (the transient)


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


def glide(arm, limits, rules, q, tip_to, R_to=None, q7_to=None, step: float = 0.001,
          tip_budget_m: float = 0.001):
    """From q, the pen tip along the straight line to `tip_to` while the hand turns (slerp)
    to `R_to` and joint 7 goes linearly to `q7_to` (default: both kept): joints by the arm's
    IK (the branch nearest the previous sample), timed at the free speed (the joint limits
    alone, no cap along the line), the tip within `tip_budget_m` of the line.
    -> Trajectory, or a Refusal."""
    from scipy.spatial.transform import Rotation, Slerp
    q = np.asarray(q, float)
    T0 = arm.fk(q[None])[0]
    p0 = arm.tip(q[None])[0]
    tip_to = np.asarray(tip_to, float)
    R_to = T0[:3, :3] if R_to is None else np.asarray(R_to, float)
    q7_to = q[6] if q7_to is None else float(q7_to)
    length = float(np.linalg.norm(tip_to - p0))
    turn = Rotation.from_matrix(T0[:3, :3]).inv() * Rotation.from_matrix(R_to)
    n = max(2, int(np.ceil(max(length / step, turn.magnitude() / 0.01,
                               abs(q7_to - q[6]) / 0.01))) + 1)
    u = np.linspace(0.0, 1.0, n)
    Rs = Slerp([0.0, 1.0], Rotation.from_matrix([T0[:3, :3], R_to]))(u).as_matrix()
    path, prev = [q], q
    for k in range(1, n):
        T = np.eye(4)
        T[:3, :3] = Rs[k]
        T[:3, 3] = p0 + u[k] * (tip_to - p0) - Rs[k] @ arm.tool.tip_hand
        Q, ok = arm.ik(T[None], q[6] + u[k] * (q7_to - q[6]))
        if not ok[0].any():
            return Refusal("unreachable", f"no arm configuration {u[k] * length * 1000:.0f} mm "
                           "along the line")
        cand = Q[0][ok[0]]
        prev = cand[np.argmin(np.abs(cand - prev).max(axis=1))]
        if np.abs(prev - path[-1]).max() > 0.05:
            return Refusal("branch", "the line would jump between arm shapes")
        path.append(prev)
    if length < 1e-6:                                   # a turn on the spot: no line to time
        return retime(JointPath(np.array(path)), limits, rules)
    return retime(JointPath(np.array(path)), limits, rules, s=u * length, smooth=True,
                  speed_cap=lambda u_: np.full(np.shape(u_), np.inf), tip_of=arm.tip, tip_budget_m=tip_budget_m)


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


def _planned_clock(traj, kin, rate: float = 250.0):
    """Q -> planned time into the flight, from how deep the tip is along `kin.down` (the
    flight goes down monotonically), so the watch needs no timestamps on the readings."""
    from aris.kernel.retime import sample
    t = np.arange(traj.t[0], traj.t[-1] + 0.5 / rate, 1.0 / rate)
    tips = kin.tip(sample(traj, np.minimum(t, traj.t[-1]))[0])
    depth = np.maximum.accumulate((tips - tips[0]) @ kin.down)
    t0 = float(traj.t[0])

    def clock(q):
        d = float((kin.tip(q)[0] - tips[0]) @ kin.down)
        i = int(np.searchsorted(depth, d, side="left"))
        return float(t[min(i, len(t) - 1)] - t0) if d < depth[-1] else float(t[-1] - t0)
    return clock


class _Watch:
    """Reads the force during a flight against the zero taken standing at the hover.  The
    detector arms `arm_after_s` into each flight (planned time); before that a reading over
    the threshold is the start jolt.  Contact: `contact_n` over `contact_ticks` readings.  The
    cap: armed, a contact found by the cap, backdated to the first reading of the last run over
    the threshold in the kept history; unarmed, a failure."""

    def __init__(self, s: TouchSettings, zero: float, kin: Kinematics):
        self.s, self.zero, self.kin = s, zero, kin
        self.contact_q, self.over, self.by_cap, self.early_trips = None, "", "", 0
        self.history = []                       # (planned t, q, force over the zero)

    def start(self, traj) -> None:
        """A new flight: its own start jolt, its own arming."""
        self.clock = _planned_clock(traj, self.kin)
        self.t_offset = self.history[-1][0] + 0.004 if self.history else 0.0
        self.run, self.first_q, self.t_flight0 = 0, None, None

    def __call__(self, q, F) -> bool:
        q = np.asarray(q, float)
        f = normal_force(F, self.kin.normal, self.s.sign) - self.zero
        t_in = self.clock(q)
        t = self.t_offset + t_in
        self.history.append((t, q, f))
        while self.history and self.history[0][0] < t - self.s.history_s:
            self.history.pop(0)
        armed = t_in >= self.s.arm_after_s - 1e-9
        if f > self.s.cap_n:
            if not armed:
                self.over = (f"pen force {f:.2f} N above the cap {self.s.cap_n} N in the first "
                             f"{self.s.arm_after_s:g} s of the flight, before the detector armed")
                return True
            k = len(self.history) - 1
            while k > 0 and self.history[k - 1][2] > self.s.contact_n:
                k -= 1
            before = self.history[k - 1] if k > 0 else self.history[k]
            self.contact_q = self.history[k][1]
            self.by_cap = (f"contact found by the cap: the force rose {before[2]:.1f} \u2192 "
                           f"{f:.1f} N within {t - before[0]:.2f} s")
            return True
        if not armed:
            if f > self.s.contact_n:
                self.early_trips += 1           # the start jolt: not a contact
            return False
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
    at_hover = [normal_force(F, kin.normal, s.sign) for F in pos.forces(s.tare_s)]
    zero = tare(at_hover[-s.tare_readings:])
    if isinstance(zero, Refusal):
        return TouchResult(False, f"{zero.reason}: {zero.detail}")
    descent, tips = descent_half(motion, kin)
    watch = _Watch(s, zero, kin)
    out = TouchResult(False, air_zero=zero)
    watch.start(descent)
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
                watch.start(ext)
                status = pos.fly(ext, watch)
                knots += list(ext.q[1:])
        if status == "" and watch.contact_q is None and not watch.over:
            status = (f"no contact within {motion.extra_depth * 1000:.0f} mm past the planned "
                      f"end")
    out.early_trips, out.armed_zero, out.by_cap = watch.early_trips, watch.zero, watch.by_cap
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
