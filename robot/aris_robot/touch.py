"""The calibration touch, under position control.  No ROS.

A "touch" motion (aris.types.Motion, kind "touch") is a checked down-and-up from a hover pose
onto the nominal paper.  The arm flies its descent half with the stock joint-trajectory
controller, at the motion's own (slow) timing, and watches its force estimate:

  rest      first the arm is given time to STAND STILL at the hover (`rest_s` without the
            tip moving more than `rest_m`, waited for up to `rest_wait_s`): the flight there
            is fast and the arm wobbles after it (2026-10-09).  The force zero is taken
            after that.  An arm that never comes to rest is flown all the same (the stall
            rule does not depend on it) and the touch's row says so (`not_at_rest`).
  stall     contact is where the tip STOPPED GOING DOWN.  Two depths along the descent, both
            counted from the flight's first knot: the commanded one (the PLANNED trajectory at
            the reading's time, one clock for the whole flight) and the actual one (the measured
            joints).  Over a sliding window (`stall_window_s`, longer when the flight is slow)
            the commanded tip advances; contact is when the actual tip advanced at most
            `stall_ratio` of that AND fell short of it by the threshold, for `lag_ticks`
            readings in a row.  Only CHANGES within the window count, so a constant offset
            (the arm standing 0.6 mm off its commanded pose, a controller that trails, the
            delay between sending the goal and its start) cannot look like a contact.  The
            rule looks only once the flight has run at its constant speed for `settle_s`, in
            the air (every descent starts 20 mm or more above the paper); what the shortfall
            does in that time is this arm's own noise, and the threshold is `lag_m` or
            `noise_factor` times that noise, whichever is larger.  The touch is the ACTUAL
            joints of the reading where the tip came to rest.
            (2026-10-09, arm 1L: the earlier rule compared the lag's LEVEL with a baseline
            from 0.3 s of flight and fired in the air twice, 0.39 and 0.30 mm over a 0.3 mm
            threshold at 1.1 N, the arm standing 0.65 mm off before it even moved.)
  force cap the force over the zero (standing at the hover; once the stall rule looks, the
            median in the air at speed) above `force_cap_n` (8 N) for `force_ticks` readings in
            a row: the safety stop.  It is a contact only when the tip also fell short of its
            commanded advance; a force without that is the estimate, or something in the way:
            the touch fails and says so, and nothing is recorded as paper
  further   the planned end reached without contact: straight on in the same direction for
            the motion's `extra_depth` (at most `extra_max`), at `extra_speed`, the hand
            keeping its orientation (joints by the arm's IK, as the planner made the
            descent), under the same rules
  back      after a contact, or none: back to the hover along the path flown, reversed

`touch` drives anything with the small `PositionArm` interface: the real arm (driver.py, the
trajectory controller and the robot state broadcaster), or the simulated one in the tests.
"""
from __future__ import annotations

import bisect
import dataclasses
from dataclasses import dataclass, field, fields
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
    lag_m: float = 0.0004          # m: the least the actual tip must fall short of the
                                   # commanded advance within the window (the threshold's floor)
    lag_ticks: int = 3             # readings in a row
    stall_window_s: float = 0.3    # s: the window the advances are compared over (longer when
                                   # the flight is slow, up to `stall_window_max_s`)
    stall_window_max_s: float = 1.5
    stall_ratio: float = 0.5       # contact: the actual tip advanced at most this share of the
                                   # commanded advance over the window
    settle_s: float = 1.0          # s at constant speed, in the air, before the stall rule
                                   # looks (its noise is measured at the end of that time)
    late_max_s: float = 0.5        # s: the most the arm may start after the reading's clock
                                   # (the goal is sent, accepted, then flown); the noise is
                                   # measured on windows that begin after it
    noise_factor: float = 2.5      # the threshold is at least this times that noise
    rest_s: float = 0.5            # s the arm must stand still at the hover before the descent
    rest_m: float = 0.00005        # m the tip may move in that time
    rest_wait_s: float = 6.0       # s waited for it at most
    force_cap_n: float = 8.0       # N over the zero: the safety stop
    force_ticks: int = 3           # readings in a row over the cap
    cap_behind_m: float = 0.0001   # m: a stop by the force cap is the paper when the tip is at
                                   # least this far behind its command.  Where the arm is stiff
                                   # 8 N are reached 0.3 mm into the paper (1L, 2026-10-09:
                                   # 0.26, 0.43, 0.64 mm), before the other rules can speak;
                                   # in the air the tip is within 0.1 mm of where it should be
    extra_speed: float = 0.002     # m/s past the planned end (the descent itself is timed by
                                   # the planner)
    back_speed: float = 0.030      # m/s of the pen tip on the way back up to the hover (it
                                   # flew at the free-flight speed until 2026-10-09: "the
                                   # retraction is crazy fast", Pete at the rig)
    tare_readings: int = 20        # readings at the hover averaged for the force zero
    readings_min_hz: float = 30.0  # readings a second the robot must give at the hover (it
                                   # gave about 85 on 2026-10-09); fewer: no descent
    extra_max: float = 0.05        # m, the most a touch may go past its planned end (the
                                   # planner's largest is calibrate.UNCAL_DEPTH, 40 mm)
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

    # optional: def still(self, rest_s, rest_m, wait_s) -> str
    #     Wait (at most wait_s) until the pen tip has not moved more than rest_m for rest_s.
    #     -> "" once it stands still, else why not.  An arm without it is taken as standing.


@dataclass
class TouchResult:
    done: bool                     # touched the paper and came back to the hover
    why: str = ""
    q_contact: np.ndarray | None = None
    depth_past_end: float = 0.0    # m beyond the planned end at contact (0: within the plan)
    held: bool = False             # stopped where it was: no way back flown (unused since
                                   # the force cap is a contact, 2026-10-08)
    air_zero: float = 0.0          # N, the force standing at the hover
    lag0: float = 0.0              # m, commanded minus actual tip standing at the hover
    lag_baseline: float | None = None   # m, the median lag in the air at descent speed
    rule: str = ""                 # what stopped it: "stall" or "force cap"
    lag_at_contact: float | None = None    # m, the shortfall over the window at the stop
    force_at_contact: float | None = None  # N over the zero
    stats: dict = field(default_factory=dict)   # what the detector saw (for the touch's row)
    trace: list = field(default_factory=list)   # (flight, t, commanded depth, actual depth,
                                                # force over the zero) per reading


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
    """Reads every reading of a touch's flights (the descent, then the extension) and says
    when to stop: the stall rule and the force cap of the module's docstring.  After a stop,
    `contact_q` is the touch (None: no contact; then `why` says what stopped it, if anything
    did)."""

    TRACE_MAX = 20000

    def __init__(self, s: TouchSettings, zero: float, lag0: float, kin: Kinematics):
        self.s, self.zero, self.lag0, self.kin = s, zero, lag0, kin
        self.d = np.asarray(kin.down, float)
        self.contact_q, self.rule, self.lag_at, self.force_at = None, "", None, None
        self.why = ""
        self.noise = None              # m: the largest shortfall seen in the air (flight 1)
        self.zero_motion = None        # N: the median force in the air at speed (flight 1)
        self.air_lag = None            # m: the median lag in the air at speed (flight 1)
        self.late = None               # s: how long after the commanded tip the actual one
                                       # started to move (flight 1)
        self.delay = None              # s: the arm follows the planned trajectory this late
        self.noise_res = None          # m: the largest residual seen in the air (flight 1)
        self.max_short, self.max_force, self.max_res = 0.0, None, 0.0
        self.flight, self.readings, self.trace = 0, 0, []
        self.stop_at = None            # what the readings were at the stop

    # ------------------------------------------------------------------ a flight

    def start(self, traj) -> None:
        from aris.kernel.retime import sample
        self.traj, self.sample = traj, sample
        self.flight += 1
        self.tip0 = self.kin.tip(traj.q[:1])[0]
        if self.flight == 1:
            self.tip_hover = self.tip0
        # depths are counted from the flight's first knot; `off` makes them depths below the
        # hover for the trace and the row
        self.off = float((self.tip0 - self.tip_hover) @ self.d)
        self.t_speed = _at_speed(traj, self.kin)
        w, late = self.s.stall_window_s, self.s.late_max_s
        # the first flight measures the arm's own noise in the air, and how late the arm
        # starts; a later one (the extension, which may start right on the paper) waits only
        # for that lateness and its window (as long as the arm has not started, standing
        # still is not a contact)
        if self.noise is None:
            settle = max(self.s.settle_s, late + w + 0.1)
        else:
            settle = (late if self.late is None else min(late, self.late + 0.1)) + w + 0.05
        self.t_armed = self.t_speed + settle
        self._t_cmd_moved = self._t_act_moved = self._da0 = None
        self.ts, self.dcs, self.das, self.qs, self.rs = [], [], [], [], []
        self.run, self.frun, self.rrun, self.first = 0, 0, 0, None
        self._air_short, self._air_force, self._air_lag = [], [], []
        self._t_first, self._t_last = None, None
        self.lag_start = None          # m: commanded minus actual depth as the flight starts

    def threshold(self) -> float:
        return max(self.s.lag_m, self.s.noise_factor * (self.noise or 0.0))

    def threshold_res(self) -> float:
        return max(3.0 * self.s.lag_m, self.s.noise_factor * (self.noise_res or 0.0))

    LAG_LEAD, LAG_SPAN = 1.0, 2.0      # s: the residual is compared with its own median
                                       # between LAG_LEAD + LAG_SPAN and LAG_LEAD ago

    def _excess(self, t, res) -> float:
        """The residual over its own recent level.  On 1L (2026-10-09) the lag crept up 0.7 mm
        over 11 s of free descent, in steps of 0.3 to 0.45 mm, and a fixed level called a
        contact 26 mm above the table at 0.7 N.  A contact grows by the commanded advance, at
        least 2 mm/s: past any threshold here well inside LAG_LEAD."""
        lo = bisect.bisect_left(self.ts, t - self.LAG_LEAD - self.LAG_SPAN)
        hi = bisect.bisect_right(self.ts, t - self.LAG_LEAD)
        past = [r for r in self.rs[lo:hi] if r is not None]
        return res - (float(np.median(past)) if len(past) >= 5 else 0.0)

    def _depth_cmd(self, t) -> float:
        tc = float(np.clip(t, self.traj.t[0], self.traj.t[-1]))
        return float((self.kin.tip(self.sample(self.traj, [tc])[0])[0] - self.tip0) @ self.d)

    def _residual(self, t, da) -> float | None:
        """How far the actual tip is behind where the planned trajectory had it `delay` ago,
        less what it was behind as the flight started: about zero while the arm follows, at
        any speed, also while the flight slows down and stops; it grows by the commanded
        advance once the tip stands on the paper."""
        if self.delay is None or self.lag_start is None:
            return None
        return self._depth_cmd(t - self.delay) - da - self.lag_start

    def _learn_delay(self) -> None:
        """At the end of the air window: the delay from the lag at speed, and the residual's
        own noise over the readings of that window."""
        k0 = bisect.bisect_left(self.ts, self.t_speed + self.s.late_max_s)
        if self.air_lag is None or self.lag_start is None or len(self.ts) - k0 < 6:
            return
        span = self.ts[-1] - self.ts[k0]
        v = (self.dcs[-1] - self.dcs[k0]) / span if span > 0 else 0.0
        if v <= 0.0:
            return
        self.delay = float(np.clip((self.air_lag - self.lag_start) / v, 0.0, self.s.late_max_s))
        res = [self._depth_cmd(self.ts[k] - self.delay) - self.das[k] - self.lag_start
               for k in range(k0, len(self.ts))]
        self.noise_res = float(np.max(np.abs(res)))
        self.rs[k0:] = res                 # the window's residuals: the first recent level

    def _depths(self, q, t) -> tuple[float, float]:
        tc = float(np.clip(t, self.traj.t[0], self.traj.t[-1]))
        q_cmd = self.sample(self.traj, [tc])[0][0]
        tips = self.kin.tip(np.array([q_cmd, q]))
        return float((tips[0] - self.tip0) @ self.d), float((tips[1] - self.tip0) @ self.d)

    def _window(self, t, need: float, since: float | None):
        """-> (i, commanded advance, actual advance) over the window ending now: it starts
        `stall_window_s` back, or further back (at most `stall_window_max_s`) until the
        commanded tip has advanced `need`; not before `since`.  None: not enough readings."""
        n = len(self.ts)
        if n < 6:
            return None
        i = bisect.bisect_right(self.ts, t - self.s.stall_window_s) - 1
        if need > 0.0:
            i = min(i, bisect.bisect_right(self.dcs, self.dcs[-1] - need) - 1)
        lo = bisect.bisect_left(self.ts, t - self.s.stall_window_max_s)
        if since is not None:
            lo = max(lo, bisect.bisect_left(self.ts, since))
        i = min(i, n - 6)                 # few readings a second: a longer window, six of them
        i = max(i, lo)
        if i < 0 or n - i < 6 or t - self.ts[i] < 0.6 * self.s.stall_window_s:
            return None
        # the middle of three readings at each end: one bad reading moves nothing
        c0, c1 = np.median(self.dcs[i:i + 3]), np.median(self.dcs[-3:])
        a0, a1 = np.median(self.das[i:i + 3]), np.median(self.das[-3:])
        return i, float(c1 - c0), float(a1 - a0)

    def _rest(self, i: int) -> np.ndarray:
        """The actual joints where the tip came to rest: the first reading of the window that
        is as deep as the tip is now (within 0.1 mm)."""
        now = float(np.median(self.das[-3:]))
        for k in range(i, len(self.das)):
            if self.das[k] >= now - 1e-4:
                return self.qs[k]
        return self.qs[-1]

    def __call__(self, q, F, t, q_ref=None) -> bool:
        # (q_ref, the controller's own commanded joints, is not used: it arrives on another
        # topic at another rate, and mixing it with the planned sample moved the lag by
        # v * latency from one reading to the next)
        s = self.s
        q, t = np.asarray(q, float), float(t)
        if self.ts and t <= self.ts[-1]:
            t = self.ts[-1] + 1e-6                    # readings in order, whatever the clock
        dc, da = self._depths(q, t)
        self.ts.append(t), self.dcs.append(dc), self.das.append(da), self.qs.append(q)
        self.rs.append(None)
        self.readings += 1
        self._t_first = t if self._t_first is None else self._t_first
        self._t_last = t
        if self.flight == 1 and self._t_act_moved is None:      # how late the arm starts
            if self._da0 is None and len(self.das) >= 3:
                self._da0 = float(np.median(self.das[:3]))
            if self._t_cmd_moved is None and dc >= 5e-4:
                self._t_cmd_moved = t
            if self._da0 is not None and da - self._da0 >= 5e-4:
                self._t_act_moved = t
                if self._t_cmd_moved is not None:
                    self.late = float(np.clip(t - self._t_cmd_moved, 0.0, s.late_max_s))
        if self.lag_start is None and len(self.ts) >= 3:
            self.lag_start = float(np.median(np.subtract(self.dcs[:3], self.das[:3])))
        armed = t >= self.t_armed
        if armed and self.noise is None:              # the air window is over: its numbers
            self.noise = float(np.max(np.abs(self._air_short))) if self._air_short else 0.0
            self.zero_motion = float(np.median(self._air_force)) if self._air_force else None
            self.air_lag = float(np.median(self._air_lag)) if self._air_lag else None
            self._learn_delay()
        zero = self.zero_motion if (armed and self.zero_motion is not None) else self.zero
        fn = None if F is None else normal_force(F, self.kin.normal, s.sign)
        f = None if fn is None else fn - zero
        if len(self.trace) < self.TRACE_MAX:
            self.trace.append((self.flight, t, dc + self.off, da + self.off, f))
        thr = self.threshold()
        learning = self.noise is None and t >= self.t_speed
        win = self._window(t, 2.0 * thr, self.t_speed)
        if learning:
            if win is not None and self.ts[win[0]] >= self.t_speed + s.late_max_s:
                self._air_short.append(win[1] - win[2])
            if fn is not None:
                self._air_force.append(fn)
            if t >= self.t_speed + s.late_max_s:
                self._air_lag.append(dc - da)
        if f is not None:
            self.max_force = f if self.max_force is None else max(self.max_force, f)
        # the lag rule's signal: known from the end of the first flight's air window on, and
        # from the first readings of every later flight
        res = None if (self.flight == 1 and not armed) else self._residual(t, da)
        if res is not None:
            self.rs[-1] = res
            res = self._excess(t, res)         # over its own recent level, from here on
            self.max_res = max(self.max_res, res)
        back = max(0, bisect.bisect_left(self.ts, t - s.stall_window_max_s))
        # ---- the force cap: the safety stop, whenever
        self.frun = self.frun + 1 if (f is not None and f > s.force_cap_n) else 0
        if self.frun >= s.force_ticks:
            any_win = self._window(t, 0.0, None)
            short = None if any_win is None else any_win[1] - any_win[2]
            behind = max([x for x in (short, res) if x is not None], default=None)
            self.rule, self.force_at, self.lag_at = "force cap", f, behind
            self.stop_at = dict(t=t, depth_m=da + self.off, commanded_m=dc + self.off)
            if behind is not None and behind >= s.cap_behind_m:
                self.contact_q = self._rest(back)
            else:
                self.why = (f"stopped by the force cap in the air: {f:.1f} N over the zero "
                            f"while the tip was following (it was "
                            f"{0.0 if behind is None else behind * 1000:.2f} mm behind): not a "
                            "contact. Something in the way, or this arm's force estimate "
                            "(robot/site.json touch.force_cap_n)")
            return True
        # ---- the lag rule: the tip is behind where the trajectory had it, by more than the
        # arm's own delay explains (it also sees a paper met while the flight slows down)
        if res is not None and res >= self.threshold_res():
            if self.rrun == 0:
                self.first_res = (self._rest(back), res, f)
            self.rrun += 1
            if self.rrun >= s.lag_ticks:
                self.contact_q, self.lag_at, self.force_at = self.first_res
                self.rule = "lag"
                self.stop_at = dict(t=t, depth_m=da + self.off, commanded_m=dc + self.off)
                return True
        else:
            self.rrun = 0
        # ---- the stall rule
        if armed and win is not None:
            i, d_cmd, d_act = win
            short = d_cmd - d_act
            self.max_short = max(self.max_short, short)
            if short >= thr and d_act <= s.stall_ratio * d_cmd:
                if self.run == 0:
                    self.first = (self._rest(i), short, f)
                self.run += 1
                if self.run >= s.lag_ticks:
                    self.contact_q, self.lag_at, self.force_at = self.first
                    self.rule = "stall"
                    self.stop_at = dict(t=t, depth_m=da + self.off, commanded_m=dc + self.off)
                    return True
                return False
        self.run = 0
        return False

    def stats(self) -> dict:
        """What the detector saw, for the touch's row (the numbers a person needs to tell a
        true contact from a false one without the robot PC)."""
        span = (self._t_last - self._t_first) if self._t_first is not None else 0.0
        mm = lambda x: None if x is None else round(1000.0 * float(x), 3)     # noqa: E731
        num = lambda x: None if x is None else round(float(x), 3)             # noqa: E731
        return dict(readings=self.readings, flights=self.flight,
                    rate_hz=num(len(self.ts) / span) if span > 0 else None,
                    at_speed_s=num(self.t_speed - self.traj.t[0]),
                    armed_s=num(self.t_armed - self.traj.t[0]),
                    air_noise_mm=mm(self.noise), air_lag_mm=mm(self.air_lag),
                    late_s=num(self.late), delay_s=num(self.delay),
                    air_residual_mm=mm(self.noise_res),
                    lag_threshold_mm=mm(self.threshold_res()) if self.delay is not None else None,
                    largest_residual_mm=mm(self.max_res),
                    threshold_mm=mm(self.threshold()), largest_shortfall_mm=mm(self.max_short),
                    force_zero_standing_n=num(self.zero), force_zero_moving_n=num(self.zero_motion),
                    largest_force_n=num(self.max_force),
                    stop_s=None if self.stop_at is None else num(self.stop_at["t"] - self.traj.t[0]),
                    stop_depth_mm=None if self.stop_at is None else mm(self.stop_at["depth_m"]),
                    stop_commanded_mm=None if self.stop_at is None
                    else mm(self.stop_at["commanded_m"]))


def stand_still(pos, s: TouchSettings) -> str:
    """Wait until the arm stands still at the hover ("" then, else why not).  An arm that
    cannot say (no `still`) is taken as standing."""
    still = getattr(pos, "still", None)
    return "" if still is None else still(s.rest_s, s.rest_m, s.rest_wait_s)


def touch(motion, pos: PositionArm, kin: Kinematics, s: TouchSettings) -> TouchResult:
    if motion.extra_depth > s.extra_max:
        return TouchResult(False, f"extra depth {motion.extra_depth * 1000:.0f} mm is more "
                                  f"than the cap {s.extra_max * 1000:.0f} mm")
    restless = stand_still(pos, s)      # waited for; an arm that keeps moving is noted in
                                        # the row (the stall rule does not need it still)
    at_hover = [normal_force(F, kin.normal, s.sign) for F in pos.forces(s.tare_s)]
    zero = tare(at_hover[-s.tare_readings:])
    if isinstance(zero, Refusal):
        return TouchResult(False, f"{zero.reason}: {zero.detail}")
    if len(at_hover) < s.readings_min_hz * s.tare_s:
        # the descent is flown on these readings: too few of them standing still is no
        # state to go down in (a robot PC in trouble, a stack that just restarted)
        return TouchResult(False, f"only {len(at_hover)} readings in {s.tare_s:g} s at the hover "
                                  f"(at least {s.readings_min_hz:g} a second are needed): the "
                                  f"robot's readings are too sparse to descend on")
    descent, tips = descent_half(motion, kin)
    tip_cmd, tip_act = kin.tip(np.array([descent.q[0], pos.joints()]))
    lag0 = float((tip_cmd - tip_act) @ np.asarray(kin.down, float))
    watch = _Watch(s, zero, lag0, kin)
    out = TouchResult(False, air_zero=zero, lag0=lag0)
    watch.start(descent)
    status = pos.fly(descent, watch)
    knots = list(descent.q)
    if status == "" and watch.rule == "":
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
        if status == "" and watch.rule == "":
            status = (f"no contact within {motion.extra_depth * 1000:.0f} mm past the planned "
                      f"end (largest shortfall {watch.max_short * 1000:.2f} mm, threshold "
                      f"{watch.threshold() * 1000:.2f} mm)")
    out.rule, out.lag_at_contact, out.force_at_contact = watch.rule, watch.lag_at, watch.force_at
    out.lag_baseline = watch.air_lag
    out.stats, out.trace = dict(watch.stats(), not_at_rest=restless or None), watch.trace
    if watch.contact_q is not None:
        out.done, out.q_contact = True, watch.contact_q
        depth = (kin.tip(watch.contact_q)[0] - tips[-1]) @ kin.down
        out.depth_past_end = max(0.0, float(depth))
    else:
        out.why = watch.why or (status if status not in ("", "cancelled") else "no contact")
    # the way back up.  An arm that gives no reading any more (a reflex, a dropped link: the
    # robot stopped by itself) is left where it is and the touch says so: on 2026-10-09 this
    # place crashed on the missing reading and took the touch's numbers with it.
    q_now = np.asarray(pos.joints(), float)
    sep = "; " if out.why else ""
    if not np.all(np.isfinite(q_now)):
        out.done, out.why = False, f"{out.why}{sep}the arm gives no reading: no way back flown"
        return out
    back = _way_back(q_now, knots, motion.q_end, kin, s.back_speed)
    if isinstance(back, Refusal):
        out.done = False
        out.why = f"{out.why}{sep}no way back: {back.reason} {back.detail}"
        return out
    went = pos.fly(back, lambda q, F, t, q_ref=None: False)
    if went:
        out.done, out.why = False, f"{out.why}{sep}the way back: {went}"
    return out


def _way_back(q_now, knots, q_hover, kin: Kinematics, speed: float | None = None):
    """From where the arm stands, back over the knots it passed, to the hover; the pen tip at
    `speed` (m/s) at most (None: as fast as a free flight)."""
    down = kin.down
    tips = kin.tip(np.array(knots))
    here = float(kin.tip(q_now)[0] @ down)
    passed = [q for q, p in zip(knots, tips) if float(p @ down) < here - 1e-6]
    path = np.array([np.asarray(q_now, float)] + passed[::-1] + [np.asarray(q_hover, float)])
    keep = np.concatenate([[True], np.abs(np.diff(path, axis=0)).max(axis=1) > 1e-9])
    path = path[keep]
    if speed is not None and len(path) >= 2:
        along = np.linalg.norm(np.diff(kin.tip(path), axis=0), axis=1)
        if np.all(along > 1e-9):                # timed along the pen tip's way, at `speed`
            slow = retime(JointPath(path), kin.arm.limits,
                          dataclasses.replace(kin.rules, draw_speed=float(speed)),
                          s=np.concatenate([[0.0], np.cumsum(along)]))
            if not isinstance(slow, Refusal):
                return slow
    return retime(JointPath(path), kin.arm.limits, kin.rules)


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
