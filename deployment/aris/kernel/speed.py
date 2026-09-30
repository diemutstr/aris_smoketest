"""The speed choice of the timing step (step 2 of `aris.kernel.retime`).

Along the rounded path, the fastest speed at every point that keeps each joint inside its
velocity and acceleration limit: one forward sweep (accelerate as hard as allowed) and one
backward sweep (brake as hard as allowed) over a few hundred points along the path, plus a dozen
across every sharp corner.  The speed is also capped where the path bends so sharply that jerk
would bind; wherever the path turns, so that the turn lasts at least TURN_TIME (a 1 kHz
measurement then sees the whole turn); and, for drawing, at the draw speed.  Around every slow
spot the cap is lowered by the distance travelled in one softening window, so that the softening
of step 3 cannot carry a fast speed into a slow spot.

The two sweeps run in the compiled module `aris_retime_native` when it is installed, else in
`sweeps_numpy`; both give the same numbers bit for bit.
"""
from __future__ import annotations

import numpy as np

from aris.kernel.spline import Rounded, evaluate

try:
    from aris_retime_native import sweeps as _native_sweeps
except ImportError:            # the numpy fallback below gives the same numbers, slower
    _native_sweeps = None

# How much of each target the speed choice may plan with.  The rest is headroom for the
# softening of step 3 and the terms the sweeps do not model; step 4 enforces the full target.
PLAN_ACCEL = 0.95     # of the acceleration target, for the sweeps
PLAN_CURVE = 0.7      # of the acceleration target, for the curvature term alone
PLAN_JERK = 0.35      # of the jerk target, for the path-bending term alone
TURN_TIME = 0.02      # s, the shortest time a turn may take
TURN_MINOR = 0.1      # of the acceleration and jerk targets: turns weaker than this are exempt
N_UNIFORM = 512       # speed nodes spread evenly along the path
N_CORNER = 12         # extra speed nodes across each corner narrower than two of those,
CORNER_ANGLE = 0.02   # rad, that turns by more than this
PEN_SAFETY = 1.003    # the pen gain is read from chords half a cell long, which can miss a
                      # little of its peak
RUNG = 1.4            # ratio between the speeds tried when lowering the cap near slow spots


def backend() -> str:
    """Which engine runs the sweeps: "native" (compiled) or "numpy"."""
    return "native" if _native_sweeps is not None else "numpy"


def _nodes(r: Rounded):
    """Where the speed is chosen: evenly along the path, plus densely across narrow corners."""
    L = float(r.u_knots[-1])
    seg = np.clip(np.searchsorted(r.u_knots, r.u_c, side="right") - 1, 1, len(r.slope) - 1)
    angle = np.linalg.norm(r.dm, axis=1) / np.maximum(np.linalg.norm(r.slope[seg - 1], axis=1),
                                                     1e-300)
    sharp = (3.0 * r.w < 2.0 * L / N_UNIFORM) & (angle > CORNER_ANGLE)
    across = (r.u_c[sharp, None]
              + np.outer(r.w[sharp], np.linspace(-1.5, 1.5, N_CORNER))).ravel()
    nodes = np.unique(np.concatenate([np.linspace(0.0, L, N_UNIFORM + 1), across]))
    return nodes[np.concatenate([[True], np.diff(nodes) > 1e-12 * L])]


def profile(r: Rounded, v_max, a_max, j_max, cap, blend_time, tip_of=None):
    """Squared path speed x = (du/dt)^2 at the nodes, from a forward and a backward sweep.

    Each cell between two nodes takes the worst |dq/du|, |d2q/du2|, |d3q/du3| of its two ends
    and its middle, which makes the sweeps conservative over the cell.

    With a draw speed `cap` and `tip_of`, the cap applies to the pen itself: where the pen moves
    more than a metre per metre of path (between the planner's IK samples a straight joint move
    need not keep the pen in step with s), the path speed is lowered by that much.  The pen is
    never sent faster than the path.
    """
    nodes = _nodes(r)
    probe = np.empty(2 * len(nodes) - 1)
    probe[0::2], probe[1::2] = nodes, 0.5 * (nodes[1:] + nodes[:-1])
    d1, d2, d3 = evaluate(r, probe, orders=(1, 2, 3))
    pen_gain = None
    if cap is not None and tip_of is not None:
        tips = tip_of(evaluate(r, probe)[0])
        half = np.linalg.norm(np.diff(tips, axis=0), axis=1) / np.diff(probe)
        pen_gain = np.maximum(half[0::2], half[1::2])            # per cell, metre of pen per u
    n1, n2, n3 = (np.linalg.norm(d, axis=1) for d in (d1, d2, d3))
    rates = (n1[0::2], n2[0::2], (n2 / np.maximum(n1, 1e-300))[0::2])   # |q'|, |q''|, turn

    def cell(a):
        return np.maximum.reduce([a[0:-2:2], a[1::2], a[2::2]])

    c1, c2, c3 = (cell(np.abs(d)) for d in (d1, d2, d3))
    with np.errstate(divide="ignore", invalid="ignore"):
        minor = np.minimum(np.min(TURN_MINOR * a_max / c2, axis=1),
                           np.min((TURN_MINOR * j_max / c3) ** (2.0 / 3.0), axis=1))
        turn = np.where(cell(n3) > 0, (cell(n2) / (cell(n3) * TURN_TIME)) ** 2, np.inf)
        x_cell = np.minimum.reduce([np.min((v_max / c1) ** 2, axis=1),
                                    np.min(PLAN_CURVE * a_max / c2, axis=1),
                                    np.min((PLAN_JERK * j_max / c3) ** (2.0 / 3.0), axis=1),
                                    np.maximum(turn, minor)])
    if cap is not None:
        gain = 1.0 if pen_gain is None else np.maximum(PEN_SAFETY * pen_gain, 1.0)
        x_cell = np.minimum(x_cell, (cap / gain) ** 2)
    x_node = np.minimum(np.concatenate([x_cell[:1], x_cell]), np.concatenate([x_cell, x_cell[-1:]]))
    x_node = _erode(np.minimum(x_node, 1e6), nodes, 1.5 * blend_time)
    du = np.diff(nodes)[:, None]
    finite = c1 > 0
    safe = np.where(finite, c1, 1.0)
    p = np.where(finite, 1.0 / (1.0 + 2.0 * du * PLAN_ACCEL * c2 / safe), 0.0)
    q = np.where(finite, 2.0 * du * PLAN_ACCEL * a_max / safe * p, np.inf)
    sweep = _native_sweeps if _native_sweeps is not None else sweeps_numpy
    return nodes, sweep(np.ascontiguousarray(x_node), np.ascontiguousarray(p),
                        np.ascontiguousarray(q)), rates


def _erode(x_node, nodes, window_time):
    """Lower the speed ceiling ahead of and behind every slow spot.

    Step 3 averages the motion over window_time either side of each moment.  Moving at speed v,
    that window spans v * window_time of path, and all of it must allow speed v.  The largest
    such v is found on a ladder of speeds: for each rung, the ceiling's minimum over the
    distance that rung covers.
    """
    v_ceiling = np.sqrt(x_node)
    v_top, v_low = float(v_ceiling.max()), float(max(v_ceiling.min(), 1e-9))
    rungs = v_top * RUNG ** -np.arange(int(np.ceil(np.log(v_top / v_low) / np.log(RUNG))) + 2)
    n = len(v_ceiling)
    table = [v_ceiling]                     # table[k][i] = min of v_ceiling[i : i + 2^k]
    while 2 ** len(table) <= n:
        prev, half = table[-1], 2 ** (len(table) - 1)
        table.append(np.concatenate([np.minimum(prev[:-half], prev[half:]),
                                     np.full(half, np.inf)]))
    table = np.stack(table)
    reach = (rungs * window_time)[:, None]                          # (rungs, 1)
    lo = np.searchsorted(nodes, nodes - reach, side="left")       # (rungs, nodes)
    hi = np.searchsorted(nodes, nodes + reach, side="right")
    k = np.floor(np.log2(hi - lo)).astype(int)
    span = np.minimum(table[k, lo], table[k, hi - 2 ** k])
    best = np.minimum(rungs[:, None], span).max(axis=0)
    return best ** 2


def sweeps_numpy(x_ceiling, p, q):
    """Forward (accelerate as hard as allowed) then backward (brake as hard as allowed).

    Over one cell the path acceleration a = (x1 - x0) / (2 du) must satisfy, for every joint,
    |dq/du| |a| + |d2q/du2| x <= A, evaluated at the faster end of the cell.  Solving that for the
    faster end gives x_fast <= p * x_slow + q, one line per joint.  The compiled module
    `aris_retime_native` does the same arithmetic in the same order.
    """
    p_rows, q_rows = p.tolist(), q.tolist()
    x = x_ceiling.tolist()
    x[0] = 0.0
    x[-1] = 0.0
    n = len(x) - 1
    for i in range(n):
        pi, qi, xi = p_rows[i], q_rows[i], x[i]
        best = x[i + 1]
        for j in range(len(pi)):
            v = pi[j] * xi + qi[j]
            if v < best:
                best = v
        x[i + 1] = best
    for i in range(n - 1, -1, -1):
        pi, qi, xn = p_rows[i], q_rows[i], x[i + 1]
        best = x[i]
        for j in range(len(pi)):
            v = pi[j] * xn + qi[j]
            if v < best:
                best = v
        x[i] = best
    return np.asarray(x)
