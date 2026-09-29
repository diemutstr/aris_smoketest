"""Timing: turn a joint path into a timed trajectory that the FR3 driver accepts at 1 kHz.

Four steps, each small:

1. Round the corners (`spline.round_corners`).  The path handed in is made of straight pieces;
   at every corner the acceleration would be an impulse however slowly it is flown (lesson L44),
   so no choice of speed can fix it.  Each corner is rounded, within the deviation budget.
2. Choose the speed along the rounded path (`speed.profile`): the fastest speed at every point
   that keeps each joint inside its velocity and acceleration limit, found by one forward and one
   backward sweep over the path (a time-optimal path parameterisation).  The speed is also capped
   where the path bends so sharply that jerk would bind; wherever the path turns, so that the
   turn lasts at least TURN_TIME (a 1 kHz measurement then sees the whole turn, whatever the
   phase of its samples); and, for drawing, at the draw speed.
3. Soften the speed changes (`_time_law`).  The sweeps switch between full acceleration and full
   braking instantly; that is infinite jerk.  The progress along the path over time is averaged
   over a short window (three box averages, `blend_time` long each), which bounds the jerk and
   leaves every constant-speed stretch exactly at its speed.  The speed cap was lowered
   beforehand around each slow spot by the distance travelled in one window, so the averaging
   cannot carry a fast speed into a slow spot.
4. Write out samples (`_to_trajectory`).  Samples are placed where the path turns or the speed
   changes, and at least every `knot_dt` seconds.  Their velocities are those of the clamped
   cubic spline, so acceleration is continuous across samples and a measurement at any rate
   converges instead of growing.  The exact extremes of the cubic pieces are then compared with
   the targets; if anything is over, the whole clock is slowed by the exact factor that brings
   it back (velocity scales with 1/k, acceleration 1/k^2, jerk 1/k^3).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.ndimage import uniform_filter1d

from aris.kernel import speed
from aris.kernel.speed import sweeps_numpy  # noqa: F401  (public, re-exported)
from aris.kernel.spline import (Rounded, evaluate, hermite, peaks, polyline_at,
                                rest_to_rest_velocities, round_corners)
from aris.types import DrawRules, JointPath, Limits, Refusal, Trajectory


# Refusal reasons: too_few_samples, not_finite, outside_limits, no_motion, bad_arc_length,
# bad_rules, cannot_smooth (over the deviation budget), leaves_limits, too_slow

KNOT_TURN = 0.05      # rad of direction change between two output samples, at most
KNOT_DV = 0.1         # of the velocity target: joint speed change between two samples, at most
KNOT_DA = 0.05         # of the acceleration target: its change between two samples, at most
KNOT_MIN_DT = 0.006   # s, but never closer than this (a turn lasts at least TURN_TIME)
END_DENSE = 0.03      # s at each end sampled every KNOT_MIN_DT
FINE_DT = 2e-3        # s, time grid of step 3
MAX_DURATION = 600.0  # s; a slower motion is refused
POSITION_TOL = 1e-7   # rad, the driver's own tolerance on the position box
TIP_GAIN = 1.3        # m/rad, most the FR3 pen tip moves per radian of joint motion (1.28
                      # measured, 20 000 random configurations, lateral pen)


@dataclass(frozen=True)
class CheckReport:
    """What a trajectory does, per joint, against the limits.

    From `check`: sampled at `rate_hz` and differenced like the driver does.  From `retime`
    (rate_hz = 0): the exact extremes of the cubic pieces, which no sampling can exceed.
    """
    rate_hz: float
    qd_peak: np.ndarray      # (7,) largest |velocity|
    qdd_peak: np.ndarray     # (7,) largest |acceleration|
    qddd_peak: np.ndarray    # (7,) largest |jerk|
    q_low: np.ndarray        # (7,) smallest position reached
    q_high: np.ndarray       # (7,) largest position reached
    qd_ratio: np.ndarray     # (7,) peak over the limit
    qdd_ratio: np.ndarray
    qddd_ratio: np.ndarray
    inside: bool             # every ratio <= 1 and every position inside the limits


@dataclass(frozen=True)
class RetimeResult:
    traj: Trajectory
    s: np.ndarray | None     # drawing: arc length along the line at each sample; else None
    deviation: float         # rad, largest joint-space distance from the input at equal path position
    tip_deviation: float | None  # m, largest pen-tip distance from the input's pen path, if asked
    width: float             # the narrowest corner-rounding window used, in path units (rad, or m)
    stretch: float           # the final clock slow-down of step 4; 1.0 means none was needed
    report: CheckReport      # exact extremes, against the full limits


def retime(path: JointPath, limits: Limits, rules: DrawRules, s: np.ndarray | None = None,
           tip_budget_m: float | None = None, tip_of=None, **options) -> Trajectory | Refusal:
    """Timed trajectory for `path`; with `s` (arc length per sample, m) a drawing motion.

    With `tip_of` (Q -> pen tips), the pen tip stays within `tip_budget_m` of the input's pen
    path (default 0.1 mm for drawing).  Other options: see `retime_detailed`.
    """
    result = retime_detailed(path, limits, rules, s, tip_budget_m=tip_budget_m, tip_of=tip_of,
                             **options)
    return result if isinstance(result, Refusal) else result.traj


def retime_detailed(path: JointPath, limits: Limits, rules: DrawRules,
                    s: np.ndarray | None = None, *, deviation: float = 1.5e-4,
                    tip_budget_m: float | None = None, tip_of=None,
                    accel_fraction: float = 0.9, jerk_fraction: float = 0.9,
                    blend_time: float = 0.025, knot_dt: float = 0.05
                    ) -> RetimeResult | Refusal:
    """`retime`, plus what it did.

    deviation       rad, joint-space distance allowed between the flown path and the input at
                    the same path position.  1.5e-4 rad keeps the pen tip within 0.2 mm: the
                    largest singular value of the FR3 tip Jacobian over the whole joint box is
                    1.28 m/rad (20 000 random configurations, lateral pen).
    tip_budget_m    m, pen-tip distance allowed between the flown pen and the input's pen path
                    (the pen positions of the input joint path) at the same path position.
                    Needs `tip_of` (Q (N,7) -> tips (N,3)).  Default with `tip_of`: 0.1 mm for
                    drawing motions, none for free motions.  Only the corners that break it are
                    rounded more tightly.
    accel_fraction  of the acceleration limit, the most the result may use.
    jerk_fraction   of the jerk limit, likewise.  Velocity uses rules.speed_fraction.
    blend_time      s, length of each of the three box averages that soften speed changes.
    knot_dt         s, the longest gap between output samples.
    """
    prepared = _prepare(path, limits, rules, s)
    if isinstance(prepared, Refusal):
        return prepared
    if tip_budget_m is not None and tip_of is None:
        return Refusal("bad_rules", "tip_budget_m needs tip_of")
    if tip_of is not None and tip_budget_m is None and s is not None:
        tip_budget_m = 1e-4
    u_knots, q_knots = prepared
    targets = (rules.speed_fraction * limits.qd_max, accel_fraction * limits.qdd_max,
               jerk_fraction * limits.qddd_max)
    # The written-out samples add a little to the rounding; leave them a tenth of each budget.
    rounded = round_corners(u_knots, q_knots, 0.9 * deviation, tip_of,
                            None if tip_budget_m is None else 0.9 * tip_budget_m)
    if isinstance(rounded, str):
        return Refusal("cannot_smooth", rounded)
    cap = rules.draw_speed if s is not None else None
    nodes, x, rates = speed.profile(rounded, *targets, cap, blend_time)
    t_fine, u_fine = _time_law(nodes, x, blend_time)
    if isinstance(t_fine, Refusal):
        return t_fine
    for gap in (knot_dt, 0.25 * knot_dt, 0.0625 * knot_dt):   # denser if the cubic strays
        traj, knots, mid, q_mid = _to_trajectory(rounded, t_fine, u_fine, nodes, rates, gap,
                                                 *targets[:2])
        dev, tip_dev = _flown_deviation(rounded, traj, mid, q_mid, u_fine, tip_of, tip_budget_m)
        if dev > deviation or (tip_dev is not None and tip_dev > tip_budget_m):
            why = Refusal("cannot_smooth", f"flown path {dev:.3g} rad from the input" +
                          ("" if tip_dev is None else f", pen {tip_dev * 1e3:.3g} mm"))
            continue
        traj, stretch, report = _enforce(traj, limits, *targets)
        if report.inside:
            break
        why = Refusal("leaves_limits", "the rounded path leaves the joint limits")
    else:
        return why
    s_out = (u_fine[knots] + float(s[0])) if s is not None else None
    width = float(rounded.w.min()) if len(rounded.w) else float(u_knots[-1])
    return RetimeResult(traj, s_out, dev, tip_dev, width, stretch, report)


def backend() -> str:
    """Which engine runs the speed sweeps: "native" (compiled) or "numpy" (same numbers)."""
    return speed.backend()


def sample(traj: Trajectory, t) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(q, qd, qdd) at times t, each (len(t), 7).  Before the start and after the end: at rest."""
    return hermite(traj.t, traj.q, traj.qd, t)


def check(traj: Trajectory, limits: Limits, rate_hz: float = 1000.0) -> CheckReport:
    """Sample at `rate_hz`, difference like the driver, compare with the limits per joint.

    Three samples of holding still are added at each end, so starting and stopping are checked.
    """
    dt = 1.0 / rate_hz
    n = int(np.ceil((traj.t[-1] - traj.t[0]) * rate_hz))
    t = traj.t[0] + np.arange(-3, n + 4) * dt
    q = hermite(traj.t, traj.q, traj.qd, t)[0]
    qd = np.max(np.abs(np.diff(q, 1, axis=0)), axis=0) / dt
    qdd = np.max(np.abs(np.diff(q, 2, axis=0)), axis=0) / dt ** 2
    qddd = np.max(np.abs(np.diff(q, 3, axis=0)), axis=0) / dt ** 3
    return _report(rate_hz, q.min(axis=0), q.max(axis=0), qd, qdd, qddd, limits)


def _report(rate_hz, low, high, qd, qdd, qddd, limits: Limits) -> CheckReport:
    ratios = (qd / limits.qd_max, qdd / limits.qdd_max, qddd / limits.qddd_max)
    inside = (max(r.max() for r in ratios) <= 1.0
              and bool(np.all(low >= limits.q_min - POSITION_TOL))
              and bool(np.all(high <= limits.q_max + POSITION_TOL)))
    return CheckReport(rate_hz, qd, qdd, qddd, low, high, *ratios, bool(inside))


# --------------------------------------------------------------------------- step 0: input


def _prepare(path: JointPath, limits: Limits, rules: DrawRules, s):
    """Path parameter u per sample (starting at 0, strictly increasing) and the samples."""
    q = np.asarray(path.q, dtype=float)
    if q.ndim != 2 or q.shape[1] != 7 or len(q) < 2:
        return Refusal("too_few_samples", f"need at least two 7-joint samples, got {q.shape}")
    if not np.all(np.isfinite(q)):
        return Refusal("not_finite", "the path contains NaN or infinity")
    outside = np.any((q < limits.q_min) | (q > limits.q_max), axis=1)
    if outside.any():
        return Refusal("outside_limits",
                       f"sample {int(np.argmax(outside))} is outside the joint limits")
    if not (0.0 < rules.speed_fraction <= 1.0) or (s is not None and not rules.draw_speed > 0.0):
        return Refusal("bad_rules", "speed_fraction must be in (0, 1], draw_speed > 0")
    step = np.linalg.norm(np.diff(q, axis=0), axis=1)
    if s is None:
        keep = np.concatenate([[True], step > 1e-12])
        q = q[keep]
        u = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(q, axis=0), axis=1))])
    else:
        s = np.asarray(s, dtype=float)
        if s.shape != (len(q),) or not np.all(np.isfinite(s)):
            return Refusal("bad_arc_length", "s must be finite, one value per sample")
        ds = np.diff(s)
        if np.any(ds < 0.0):
            return Refusal("bad_arc_length", "s decreases; the pen would go backwards")
        if np.any((ds == 0.0) & (step > 1e-12)):
            return Refusal("bad_arc_length",
                           "the joints move where s does not; the pen would have to stop")
        keep = np.concatenate([[True], ds > 0.0])
        q, u = q[keep], s[keep] - s[0]
    if len(q) < 2 or u[-1] <= 0.0:
        return Refusal("no_motion", "all samples are the same")
    return u, q


# --------------------------------------------------------------------------- step 3: soften


def _time_law(nodes, x_nodes, blend_time):
    """Path position over time on a uniform time grid, with speed changes softened."""
    v = np.sqrt(np.maximum(x_nodes, 0.0))
    t_nodes = np.concatenate([[0.0], np.cumsum(2.0 * np.diff(nodes) / (v[:-1] + v[1:]))])
    T, L = float(t_nodes[-1]), float(nodes[-1])
    if T > MAX_DURATION:
        return Refusal("too_slow", f"the motion would take {T:.0f} s"), None
    n = int(np.ceil(T / FINE_DT))
    n_box = max(1, int(round(blend_time / FINE_DT))) | 1
    half = 3 * (n_box // 2)                       # reach of three boxes
    t = np.arange(-2 * half, n + 2 * half + 1) * FINE_DT
    # Within a cell the path acceleration is constant, so position is exactly quadratic in time.
    k = np.clip(np.searchsorted(t_nodes, t, side="right") - 1, 0, len(nodes) - 2)
    tau = np.clip(t - t_nodes[k], 0.0, t_nodes[k + 1] - t_nodes[k])
    accel = (x_nodes[k + 1] - x_nodes[k]) / (2.0 * (nodes[k + 1] - nodes[k]))
    u = np.clip(nodes[k] + v[k] * tau + 0.5 * accel * tau * tau, nodes[k], nodes[k + 1])
    u[t <= 0.0] = 0.0
    u[t >= T] = L
    for _ in range(3):
        u = uniform_filter1d(u, n_box, mode="nearest")
    # Output index k is centred on t[k]; the window of t = -half*dt sees only u = 0, and the
    # window of t = n*dt + half*dt sees only u = L.  Everything between moves.
    us = np.clip(u[half:n + 3 * half + 1], 0.0, L)
    us[0], us[-1] = 0.0, L
    return np.arange(len(us)) * FINE_DT, us


# --------------------------------------------------------------------------- step 4: write out, check


def _to_trajectory(r: Rounded, t_fine, u_fine, nodes, rates, knot_dt, v_max, a_max):
    """Output samples where the path turns, where the joints' speed or acceleration change, and
    every knot_dt.  (A cubic follows steady motion exactly, however far apart its samples.)"""
    du = np.diff(u_fine)
    speed = du / FINE_DT
    accel = np.diff(speed, prepend=0.0) / FINE_DT
    u_mid = 0.5 * (u_fine[1:] + u_fine[:-1])
    n1, n2, turn = (np.interp(u_mid, nodes, rate) for rate in rates)
    joint_speed = n1 * speed
    joint_accel = n1 * accel + n2 * speed * speed
    cost = np.maximum.reduce([np.full(len(du), FINE_DT / knot_dt), du * turn / KNOT_TURN,
                              np.abs(np.diff(joint_speed, prepend=0.0)) / (KNOT_DV * v_max.max()),
                              np.abs(np.diff(joint_accel, prepend=0.0)) / (KNOT_DA * a_max.max())])
    # Every derivative starts and ends at zero; dense samples there keep the spline's end
    # acceleration near zero too (it cannot be pinned: a cubic spline has one condition per end).
    ends = int(round(END_DENSE / FINE_DT))
    cost[:ends] = cost[-ends:] = np.inf
    cost = np.minimum(cost, FINE_DT / KNOT_MIN_DT)
    count = np.floor(np.concatenate([[0.0], np.cumsum(cost)]))
    knots = np.flatnonzero(np.diff(count) > 0) + 1
    knots = np.unique(np.concatenate([[0], knots, [len(t_fine) - 1]]))
    t = t_fine[knots]
    # The rounded path at the samples and halfway between them, in one evaluation; the halfway
    # points are where the written-out cubic is checked against it.
    mid = (knots[:-1] + knots[1:]) // 2
    both = np.empty(len(knots) + len(mid), dtype=int)
    both[0::2], both[1::2] = knots, mid
    q_both = evaluate(r, u_fine[both])[0]
    q = q_both[0::2].copy()
    q[0], q[-1] = r.q_knots[0], r.q_knots[-1]
    return Trajectory(t, q, rest_to_rest_velocities(t, q)), knots, mid, q_both[1::2]


def _flown_deviation(r: Rounded, traj, mid, q_mid, u_fine, tip_of, tip_budget):
    """Deviation of the written-out cubic halfway between samples, where it strays most.

    The pen is first bounded without forward kinematics: the cubic's pen can be no further from
    the rounded path's pen than TIP_GAIN times their joint-space distance.  The pen is measured
    only at the points where that bound is over the budget.
    """
    u = u_fine[mid]
    q = hermite(traj.t, traj.q, traj.qd, mid * FINE_DT)[0]
    dev = max(r.deviation, float(np.max(np.linalg.norm(q - polyline_at(r.u_knots, r.q_knots, u),
                                                       axis=1))))
    if tip_budget is None:
        return dev, None
    spline_err = np.linalg.norm(q - q_mid, axis=1)
    bound = r.tip_deviation + TIP_GAIN * spline_err
    unsure = bound > tip_budget                       # measure the pen only where it could be over
    if not unsure.any():
        return dev, float(bound.max())
    lin = polyline_at(r.u_knots, r.q_knots, u[unsure])
    measured = np.linalg.norm(tip_of(q[unsure]) - tip_of(lin), axis=1)
    return dev, float(max(bound[~unsure].max(initial=r.tip_deviation), measured.max()))


def _enforce(traj: Trajectory, limits: Limits, v_max, a_max, j_max):
    """Slow the clock until the exact extremes of the cubic pieces are inside every target."""
    low, high, qd, qdd, qddd = peaks(traj.t, traj.q, traj.qd)
    k = max(np.max(qd / v_max), np.sqrt(np.max(qdd / a_max)), np.cbrt(np.max(qddd / j_max)))
    stretch = 1.0
    if k > 1.0:
        stretch = k * (1.0 + 1e-9)
        traj = Trajectory(traj.t * stretch, traj.q, traj.qd / stretch)
        qd, qdd, qddd = qd / stretch, qdd / stretch ** 2, qddd / stretch ** 3
    return traj, stretch, _report(0.0, low, high, qd, qdd, qddd, limits)
