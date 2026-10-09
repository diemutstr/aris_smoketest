"""The calibration touch, under position control.  No ROS.

A "touch" motion (aris.types.Motion, kind "touch") is a checked down-and-up from a hover pose
onto the nominal paper.  The arm flies its descent half with the stock joint-trajectory
controller, at the motion's own (slow) timing, and watches its force estimate:

  lag       contact is where the tip STOPPED: at every reading the tip of the commanded
            joints (the planned sample at the reading's time into the flight) and the tip of
            the actual joints (the controller's own commanded joints from its controller_state
            when it publishes them), `lag = (tip_commanded - tip_actual) . down`, less the
            baseline: the median lag over the first `baseline_s` (0.3 s) of the descent at
            speed, in the air (the lag rule waits for it; the force cap does not).  Over
            `lag_m` (0.3 mm) for `lag_ticks` (3)
            readings in a row: the trajectory is cancelled; the ACTUAL joints of the first of
            those readings are the touch.  (2026-10-08: the force estimate of arms 31 and 2
            is worthless at 3 N, holder mass and friction; contacts were called in the air.)
  force cap the force over the hover zero (the mean of `tare_readings` readings standing
            still there) above `force_cap_n` (8 N): the safety stop, and a contact at the
            actual joints of that reading ("stopped by the force cap")
  further   the planned end reached without contact: straight on in the same direction for
            the motion's `extra_depth` (at most `extra_max`), at `extra_speed`, the hand
            keeping its orientation (joints by the arm's IK, as the planner made the
            descent), under the same rules
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
    lag_m: float = 0.0003          # m the actual tip trails the commanded one along the
                                   # descent (less the baseline): contact
    lag_ticks: int = 3             # readings in a row
    baseline_s: float = 0.3        # s of the descent at speed whose median lag is the baseline
    force_cap_n: float = 8.0       # N over the hover zero: the safety stop, also a contact
    extra_speed: float = 0.002     # m/s past the planned end (the one speed setting: the
                                   # planned descent is timed by the planner)
    tare_readings: int = 20        # readings at the hover averaged for the force zero
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
        """Fly `traj` under position control at its own timing, calling `watch(q, F, t, q_ref)`
        for every reading (the ACTUAL joints, the external force on the arm in the base frame,
        the reading's time on the trajectory's clock, and the controller's commanded joints
        then if it publishes them, else None: then the trajectory at t stands for them) as
        they come; when it returns True, cancel and stand still.  -> "" (reached the
        end), "cancelled", or why it failed."""

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
    held: bool = False             # stopped where it was: no way back flown (unused since
                                   # the force cap is a contact, 2026-10-08)
    air_zero: float = 0.0          # N, the force standing at the hover
    lag0: float = 0.0              # m, the lag standing at the hover
    lag_baseline: float | None = None   # m, the median lag in the air at descent speed
    rule: str = ""                 # what stopped it: "lag" or "force cap"
    lag_at_contact: float | None = None    # m, less the baseline
    force_at_contact: float | None = None  # N over the hover zero


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


def _at_speed(traj, kin, rate: float = 250.0) -> float:
    """When (trajectory time) the flight first runs at its constant speed (98 % of its
    fastest along `kin.down`): the end of its acceleration ramp."""
    from aris.kernel.retime import sample
    t = np.arange(traj.t[0], traj.t[-1], 1.0 / rate)
    if len(t) < 3:
        return float(traj.t[0])
    depth = (kin.tip(sample(traj, t)[0]) - kin.tip(traj.q[:1])[0]) @ kin.down
    v = np.gradient(depth, t)
    return float(t[int(np.argmax(v >= 0.98 * v.max()))])


class _Watch:
    """Reads every reading of a flight: the lag of the actual tip behind the commanded one
    along the descent, and the force over the hover zero.  The lag's baseline is the median
    over the first `baseline_s` of the descent at speed (in the air by construction: every
    descent starts 20 mm or more above the paper), taken once, in the first flight; until it
    is there, the lag standing at the hover stands in.  Contact: the lag over the baseline by
    `lag_m` for `lag_ticks` readings (the actual joints of the first), or the force over the
    cap (the actual joints of that reading)."""

    def __init__(self, s: TouchSettings, zero: float, lag0: float, kin: Kinematics):
        self.s, self.zero, self.lag0, self.kin = s, zero, lag0, kin
        self.d = np.asarray(kin.down, float)
        self.contact_q, self.rule, self.lag_at, self.force_at = None, "", None, None
        self.max_lag, self.baseline, self.window = 0.0, None, []

    def start(self, traj) -> None:
        from aris.kernel.retime import sample
        self.traj, self.sample = traj, sample
        self.run, self.first = 0, None
        self.t_speed = _at_speed(traj, self.kin) if self.baseline is None else None

    def raw_lag(self, q, t, q_ref=None) -> float:
        if q_ref is None:
            t = float(np.clip(t, self.traj.t[0], self.traj.t[-1]))
            q_ref = self.sample(self.traj, [t])[0][0]
        tips = self.kin.tip(np.array([np.asarray(q_ref, float), np.asarray(q, float)]))
        return float((tips[0] - tips[1]) @ self.d)

    def __call__(self, q, F, t, q_ref=None) -> bool:
        q = np.asarray(q, float)
        raw = self.raw_lag(q, t, q_ref)
        learning = False
        if self.baseline is None and self.t_speed is not None:
            if t <= self.t_speed + self.s.baseline_s:
                learning = True                 # the ramp and the window: in the air
                if t >= self.t_speed:
                    self.window.append(raw)
            else:
                self.baseline = float(np.median(self.window)) if self.window else self.lag0
        base = self.baseline if self.baseline is not None else self.lag0
        lag = raw - base
        f = None if F is None else normal_force(F, self.kin.normal, self.s.sign) - self.zero
        self.max_lag = max(self.max_lag, lag)
        if f is not None and f > self.s.force_cap_n:
            self.contact_q, self.rule, self.lag_at, self.force_at = q, "force cap", lag, f
            return True
        if lag > self.s.lag_m and not learning:
            if self.run == 0:
                self.first = (q, lag, f)
            self.run += 1
            if self.run >= self.s.lag_ticks:
                self.contact_q, self.lag_at, self.force_at = self.first
                self.rule = "lag"
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
    tip_cmd, tip_act = kin.tip(np.array([descent.q[0], pos.joints()]))
    lag0 = float((tip_cmd - tip_act) @ np.asarray(kin.down, float))
    watch = _Watch(s, zero, lag0, kin)
    out = TouchResult(False, air_zero=zero, lag0=lag0)
    watch.start(descent)
    status = pos.fly(descent, watch)
    knots = list(descent.q)
    if status == "" and watch.contact_q is None:
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
        if status == "" and watch.contact_q is None:
            status = (f"no contact within {motion.extra_depth * 1000:.0f} mm past the planned "
                      f"end (largest lag {watch.max_lag * 1000:.2f} mm)")
    out.rule, out.lag_at_contact, out.force_at_contact = watch.rule, watch.lag_at, watch.force_at
    out.lag_baseline = watch.baseline
    if watch.contact_q is not None:
        out.done, out.q_contact = True, watch.contact_q
        depth = (kin.tip(watch.contact_q)[0] - tips[-1]) @ kin.down
        out.depth_past_end = max(0.0, float(depth))
    else:
        out.why = status if status not in ("", "cancelled") else "no contact"
    back = _way_back(pos.joints(), knots, motion.q_end, kin)
    went = pos.fly(back, lambda q, F, t, q_ref=None: False)
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
