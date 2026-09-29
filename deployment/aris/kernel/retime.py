"""Timing: turn a joint path into a timed trajectory that the FR3 driver accepts at 1 kHz.

Four steps, each small:

1. Smooth the geometry (`spline.smooth_path`).  The path handed in is made of straight pieces;
   at every corner the acceleration would be an impulse however slowly it is flown (lesson L44),
   so no choice of speed can fix it.  The corners are rounded first, within a deviation budget.
2. Choose the speed along the smooth path (`_speed_profile`): the fastest speed at every point
   that keeps each joint inside its velocity and acceleration limit, found by one forward and one
   backward sweep over the path (a time-optimal path parameterisation, written out here).  The
   speed is also capped where the path bends so sharply that jerk would bind; wherever the path
   turns, so that the turn lasts at least TURN_TIME (a 1 kHz measurement then sees the whole
   turn, whatever the phase of its samples); and, for drawing, at the draw speed.
3. Soften the speed changes (`_time_law`).  The sweeps switch between full acceleration and full
   braking instantly; that is infinite jerk.  The progress along the path over time is averaged
   over a short window (three box averages, `blend_time` long each), which bounds the jerk and
   leaves every constant-speed stretch exactly at its speed.  The speed cap was lowered
   beforehand around each slow spot by the distance travelled in one window, so the averaging
   cannot carry a fast speed into a slow spot.
4. Write out samples and check (`_to_trajectory`, `check`).  Samples are placed densely where the
   path bends and at least every `knot_dt` seconds otherwise.  Their velocities are those of the
   clamped cubic spline, so the acceleration is continuous across samples and a measurement at
   any rate converges instead of growing.  The result is sampled at 1 kHz and differentiated the
   way the driver does; if anything is over its target, the whole clock is slowed by the exact
   factor that brings it back (velocity scales with 1/k, acceleration 1/k^2, jerk 1/k^3).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.ndimage import correlate1d, minimum_filter1d

from aris.kernel.spline import (SmoothPath, box3_kernel, hermite, path_at, polyline_at,
                                rest_to_rest_velocities, smooth_path)
from aris.types import DrawRules, JointPath, Limits, Refusal, Trajectory

# Refusal reasons: too_few_samples, not_finite, outside_limits, no_motion, bad_arc_length,
# bad_rules, cannot_smooth (over the deviation budget), leaves_limits

# How much of each target the speed choice (step 2) may plan with.  The rest is headroom for the
# softening of step 3 and the terms the sweeps do not model; the measured result must still be
# inside the full target, and step 4 enforces that.
PLAN_ACCEL = 0.95     # of the acceleration target, for the sweeps
PLAN_CURVE = 0.7      # of the acceleration target, for the curvature term alone
PLAN_JERK = 0.35      # of the jerk target, for the path-bending term alone
TURN_ANGLE = 0.05     # rad; a path "turns" where its direction changes this much in one window
TURN_TIME = 0.02      # s, the shortest time a turn may take
TURN_MINOR = 0.1      # of the acceleration and jerk targets: turns weaker than this are exempt
KNOT_TURN = 0.05      # rad of direction change between two output samples, at most
KNOT_MIN_DT = 0.002   # s, but never closer than this (a turn lasts at least TURN_TIME)
POSITION_TOL = 1e-7   # rad, the driver's own tolerance on the position box
MAX_CELLS = 20_000    # cells of the speed sweeps; each cell takes its worst point
FINE_DT = 5e-4        # s, time grid of step 3
TIP_TRIES = 12        # tightenings of the joint budget to meet a tip budget


@dataclass(frozen=True)
class CheckReport:
    """What a trajectory does when sampled at `rate_hz` and differentiated like the driver does."""
    rate_hz: float
    qd_peak: np.ndarray      # (7,) largest |first difference| / dt
    qdd_peak: np.ndarray     # (7,) largest |second difference| / dt^2
    qddd_peak: np.ndarray    # (7,) largest |third difference| / dt^3
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
    tip_deviation: float | None  # m, largest pen-tip distance from the input's tip polyline, if asked
    width: float             # the narrowest corner-rounding window used, in path units (rad, or m)
    stretch: float           # the final clock slow-down of step 4; 1.0 means none was needed
    report: CheckReport      # at 1 kHz, against the full limits


def retime(path: JointPath, limits: Limits, rules: DrawRules, s: np.ndarray | None = None,
           tip_budget_m: float | None = None, tip_of=None, **options) -> Trajectory | Refusal:
    """Timed trajectory for `path`; with `s` (arc length per sample, m) a drawing motion.

    With `tip_of` (Q -> pen tips), the pen tip stays within `tip_budget_m` of the input's tip
    polyline (default 0.1 mm for drawing).  Other options: see `retime_detailed`.
    """
    result = retime_detailed(path, limits, rules, s, tip_budget_m=tip_budget_m, tip_of=tip_of,
                             **options)
    return result if isinstance(result, Refusal) else result.traj


def retime_detailed(path: JointPath, limits: Limits, rules: DrawRules,
                    s: np.ndarray | None = None, *, deviation: float = 1.5e-4,
                    tip_budget_m: float | None = None, tip_of=None,
                    accel_fraction: float = 0.9, jerk_fraction: float = 0.9,
                    blend_time: float = 0.025, knot_dt: float = 0.005
                    ) -> RetimeResult | Refusal:
    """`retime`, plus what it did.

    deviation       rad, joint-space distance allowed between the flown path and the input at
                    the same path position.  1.5e-4 rad keeps the pen tip within 0.2 mm: the
                    largest singular value of the FR3 tip Jacobian over the whole joint box is
                    1.28 m/rad (20 000 random configurations, lateral pen).
    tip_budget_m    m, pen-tip distance allowed between the flown tip and the input's tip
                    polyline at the same path position.  Needs `tip_of` (Q (N,7) -> tips (N,3)).
                    Default with `tip_of`: 0.1 mm for drawing motions, none for free motions.
                    The joint budget is tightened until the tip is inside it.
    accel_fraction  of the acceleration limit, the most the result may use at 1 kHz.
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
    cap = rules.draw_speed if s is not None else None
    tips_in = tip_of(q_knots) if tip_budget_m is not None else None
    for _ in range(TIP_TRIES):
        out = _attempt(u_knots, q_knots, deviation, targets, cap, blend_time, knot_dt)
        if isinstance(out, Refusal):
            return out
        traj, knots, t_fine, u_fine, dev, width = out
        if tips_in is None:
            tip_dev = None
            break
        q_flown = hermite(traj.t, traj.q, traj.qd, t_fine)[0]
        tip_dev = float(np.max(np.linalg.norm(tip_of(q_flown)
                                              - polyline_at(u_knots, tips_in, u_fine), axis=1)))
        if tip_dev <= tip_budget_m:
            break
        deviation = min(deviation, dev) * max(0.2, 0.9 * tip_budget_m / tip_dev)
    else:
        return Refusal("cannot_smooth", f"pen tip {tip_dev * 1e3:.3g} mm from the line after "
                       f"{TIP_TRIES} tightenings (budget {tip_budget_m * 1e3:.3g} mm)")
    traj, stretch, report = _enforce(traj, limits, *targets)
    if not _positions_inside(report, limits):
        return Refusal("leaves_limits", "the rounded path leaves the joint limits")
    s_out = (u_fine[knots] + float(s[0])) if s is not None else None
    return RetimeResult(traj, s_out, dev, tip_dev, width, stretch, report)


def _attempt(u_knots, q_knots, deviation, targets, cap, blend_time, knot_dt):
    """Steps 1 to 4 (before the final check) for one joint-space deviation budget."""
    smooth = smooth_path(u_knots, q_knots, 0.9 * deviation)
    if isinstance(smooth, str):
        return Refusal("cannot_smooth", smooth)
    u_nodes, x_nodes = _speed_profile(smooth, *targets, cap, blend_time)
    t_fine, u_fine = _time_law(u_nodes, x_nodes, blend_time)
    traj, knots = _to_trajectory(smooth, q_knots, t_fine, u_fine, knot_dt)
    q_flown = hermite(traj.t, traj.q, traj.qd, t_fine)[0]
    dev = float(np.max(np.linalg.norm(q_flown - polyline_at(u_knots, q_knots, u_fine), axis=1)))
    if dev > deviation:
        return Refusal("cannot_smooth", f"flown path is {dev:.3g} rad from the input")
    return traj, knots, t_fine, u_fine, dev, float(smooth.width.min())


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
    report = CheckReport(rate_hz, qd, qdd, qddd, q.min(axis=0), q.max(axis=0),
                         qd / limits.qd_max, qdd / limits.qdd_max, qddd / limits.qddd_max, False)
    ok = (max(report.qd_ratio.max(), report.qdd_ratio.max(), report.qddd_ratio.max()) <= 1.0
          and _positions_inside(report, limits))
    return CheckReport(**{**report.__dict__, "inside": bool(ok)})


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


# --------------------------------------------------------------------------- step 2: speed


def _speed_profile(path: SmoothPath, v_max, a_max, j_max, cap, blend_time):
    """Squared path speed x = (du/dt)^2 at cell boundaries, from a forward and a backward sweep.

    The path is cut into cells; each cell takes the worst |dq/du|, |d2q/du2|, |d3q/du3| of its
    fine-grid points, which makes the sweeps conservative for everything inside the cell.
    """
    n_fine = len(path.u)
    n_cells = min(MAX_CELLS, n_fine - 1)
    size = max(1, int(np.ceil((n_fine - 1) / n_cells)))
    starts = np.arange(0, n_fine - 1, size)
    ends = np.minimum(starts + size, n_fine - 1)
    u_nodes = np.concatenate([path.u[starts], [path.u[-1]]])

    def cell_max(a):
        a = np.abs(a)
        return np.maximum(np.maximum.reduceat(a, starts, axis=0), a[ends])

    c1, c2, c3 = cell_max(path.dq), cell_max(path.ddq), cell_max(path.dddq)
    with np.errstate(divide="ignore"):
        ceilings = [np.min((v_max / c1) ** 2, axis=1),
                    np.min(PLAN_CURVE * a_max / c2, axis=1),
                    np.min((PLAN_JERK * j_max / c3) ** (2.0 / 3.0), axis=1)]
    # Wherever the path turns noticeably within one rounding window, pass slowly enough that the
    # turn lasts at least TURN_TIME: then a 1 kHz measurement sees the whole turn, not a blip.
    # A turn so gentle at this speed that its acceleration and jerk stay under TURN_MINOR of the
    # targets is exempt: misreading it by a sampling phase cannot matter.
    w_cell = np.minimum(np.minimum.reduceat(path.width, starts), path.width[ends])
    turning = np.any(c2 * w_cell[:, None] > TURN_ANGLE * np.maximum(c1, 1e-12), axis=1)
    with np.errstate(divide="ignore"):
        minor = np.minimum(np.min(TURN_MINOR * a_max / c2, axis=1),
                           np.min((TURN_MINOR * j_max / c3) ** (2.0 / 3.0), axis=1))
    ceilings.append(np.where(turning, np.maximum((w_cell / TURN_TIME) ** 2, minor), np.inf))
    x_cell = np.minimum.reduce(ceilings)
    if cap is not None:
        x_cell = np.minimum(x_cell, cap ** 2)
    x_node = np.minimum(np.concatenate([x_cell[:1], x_cell]), np.concatenate([x_cell, x_cell[-1:]]))
    x_node = _erode(np.minimum(x_node, 1e6), path.u[-1] / len(starts), 1.5 * blend_time)
    du = np.diff(u_nodes)
    with np.errstate(divide="ignore", invalid="ignore"):
        beta = np.where(c1 > 0, 2.0 * du[:, None] * PLAN_ACCEL * c2 / np.maximum(c1, 1e-300), np.inf)
        alpha = np.where(c1 > 0, 2.0 * du[:, None] * PLAN_ACCEL * a_max / np.maximum(c1, 1e-300),
                         np.inf)
    return u_nodes, _sweeps(x_node, alpha, beta)


def _erode(x_node, cell, window_time):
    """Lower the speed ceiling ahead of and behind every slow spot.

    Step 3 averages the motion over window_time either side of each moment.  Moving at speed v,
    that window spans v * window_time of path, and all of it must allow speed v.  The largest
    such v is found on a ladder of speeds: for each rung, the ceiling eroded over the distance
    that rung covers.
    """
    v_ceiling = np.sqrt(x_node)
    v_top, v_low = float(v_ceiling.max()), float(max(v_ceiling.min(), 1e-9))
    rungs = v_top * 1.2 ** -np.arange(int(np.ceil(np.log(v_top / v_low) / np.log(1.2))) + 2)
    best = np.zeros_like(v_ceiling)
    for v in rungs:
        size = 2 * int(np.ceil(v * window_time / cell)) + 1
        best = np.maximum(best, np.minimum(v, minimum_filter1d(v_ceiling, size, mode="nearest")))
    return best ** 2


def _sweeps(x_ceiling, alpha, beta):
    """Forward (accelerate as hard as allowed) then backward (brake as hard as allowed).

    Over one cell the path acceleration a = (x1 - x0) / (2 du) must satisfy, for every joint,
    |dq/du| |a| + |d2q/du2| x <= A, evaluated at the faster end of the cell.  Solving that for the
    faster end gives x_fast <= (x_slow + alpha) / (1 + beta), one line per joint.
    """
    finite = np.isfinite(alpha) & np.isfinite(beta)
    p = np.where(finite, 1.0 / (1.0 + np.where(finite, beta, 0.0)), 0.0)
    r = np.where(finite, np.where(finite, alpha, 0.0) * p, np.inf)
    p_rows, r_rows = p.tolist(), r.tolist()
    x = x_ceiling.tolist()
    x[0] = 0.0
    x[-1] = 0.0
    n = len(x) - 1
    for i in range(n):                                   # forward
        pi, ri, xi = p_rows[i], r_rows[i], x[i]
        best = x[i + 1]
        for j in range(7):
            v = pi[j] * xi + ri[j]
            if v < best:
                best = v
        x[i + 1] = best
    for i in range(n - 1, -1, -1):                       # backward
        pi, ri, xn = p_rows[i], r_rows[i], x[i + 1]
        best = x[i]
        for j in range(7):
            v = pi[j] * xn + ri[j]
            if v < best:
                best = v
        x[i] = best
    return np.asarray(x)


# --------------------------------------------------------------------------- step 3: soften


def _time_law(u_nodes, x_nodes, blend_time):
    """Path position over time on a fine uniform time grid, with speed changes softened."""
    v = np.sqrt(np.maximum(x_nodes, 0.0))
    t_nodes = np.concatenate([[0.0], np.cumsum(2.0 * np.diff(u_nodes) / (v[:-1] + v[1:]))])
    T, L = float(t_nodes[-1]), float(u_nodes[-1])
    dt = max(FINE_DT, T / 4e6)
    n = int(np.ceil(T / dt))
    n_box = max(1, int(round(blend_time / dt))) | 1
    kernel = box3_kernel(n_box)
    half = len(kernel) // 2
    t = np.arange(-2 * half, n + 2 * half + 1) * dt
    # Within a cell the path acceleration is constant, so position is exactly quadratic in time.
    k = np.clip(np.searchsorted(t_nodes, t, side="right") - 1, 0, len(u_nodes) - 2)
    tau = np.clip(t - t_nodes[k], 0.0, t_nodes[k + 1] - t_nodes[k])
    accel = (x_nodes[k + 1] - x_nodes[k]) / (2.0 * (u_nodes[k + 1] - u_nodes[k]))
    u = np.clip(u_nodes[k] + v[k] * tau + 0.5 * accel * tau * tau, u_nodes[k], u_nodes[k + 1])
    u[t <= 0.0] = 0.0
    u[t >= T] = L
    us = np.clip(correlate1d(u, kernel, mode="nearest"), 0.0, L)
    # Output index k is centred on t[k]; the window of t = -half*dt sees only u = 0, and the
    # window of t = n*dt + half*dt sees only u = L.  Everything between moves.
    i0, i1 = half, n + 3 * half
    us = us[i0:i1 + 1].copy()
    us[0], us[-1] = 0.0, L
    return np.arange(len(us)) * dt, us


# --------------------------------------------------------------------------- step 4: write out, check


def _to_trajectory(path: SmoothPath, q_knots, t_fine, u_fine, knot_dt):
    """Output samples: at least every knot_dt, one per KNOT_TURN of direction change."""
    dt = t_fine[1] - t_fine[0]
    speed = np.linalg.norm(path.dq, axis=1)
    turn_rate = np.linalg.norm(path.ddq, axis=1) / np.maximum(speed, 1e-9 * speed.max())
    u_mid = 0.5 * (u_fine[1:] + u_fine[:-1])
    cost = np.maximum(dt / knot_dt,
                      np.diff(u_fine) * np.interp(u_mid, path.u, turn_rate) / KNOT_TURN)
    # Samples closer than KNOT_MIN_DT would resolve the fine grid the smooth path is stored on,
    # and its interpolation error would show up as jerk.
    cost = np.minimum(cost, dt / KNOT_MIN_DT)
    count = np.floor(np.concatenate([[0.0], np.cumsum(cost)]))
    knots = np.flatnonzero(np.diff(count) > 0) + 1
    knots = np.unique(np.concatenate([[0], knots, [len(t_fine) - 1]]))
    t = t_fine[knots]
    q = path_at(path, u_fine[knots])
    q[0], q[-1] = q_knots[0], q_knots[-1]
    return Trajectory(t, q, rest_to_rest_velocities(t, q)), knots


def _enforce(traj: Trajectory, limits: Limits, v_max, a_max, j_max):
    """Slow the clock until the 1 kHz measurement is inside every target."""
    stretch = 1.0
    for _ in range(5):
        report = check(traj, limits)
        k = max(np.max(report.qd_peak / v_max), np.sqrt(np.max(report.qdd_peak / a_max)),
                np.cbrt(np.max(report.qddd_peak / j_max)))
        if k <= 1.0:
            return traj, stretch, report
        k *= 1.0005
        stretch *= k
        traj = Trajectory(traj.t * k, traj.q, traj.qd / k)
    return traj, stretch, check(traj, limits)


def _positions_inside(report: CheckReport, limits: Limits) -> bool:
    return bool(np.all(report.q_low >= limits.q_min - POSITION_TOL)
                and np.all(report.q_high <= limits.q_max + POSITION_TOL))
