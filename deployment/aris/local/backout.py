"""The exact path along a route, and its verification.

The search works on a coarse grid (a layer every 1-2 cm, spins and elbow values on a grid).
The path it returns is then solved again, densely: every `dense_step` along the line, the
spin, lean and elbow value are read off a smooth curve through the route's nodes, and the IK
is solved for that pose in the route's slot.  Joint angles are never interpolated, so every
sample puts the pen exactly on the line.  Where the straight joint motion between two samples
would take the pen off the line by more than `chord_tol`, a sample is added halfway.

`verify` then re-derives everything from the joint samples alone: the tip on the line at every
sample and near it halfway between samples, every gate at every sample, no joint jump, the arm
clear of itself, and the clearance along the whole path including the motion between samples.
A plan that fails is never returned; the planner bans the nodes around the failure and
searches again.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.interpolate import CubicSpline, PchipInterpolator
from scipy.ndimage import gaussian_filter1d

from aris.local import polyline
from aris.local.gates import Judge
from aris.local.lattice import Lattice
from aris.local.settings import Settings


@dataclass(frozen=True)
class Dense:
    s: np.ndarray            # (N,) arc length along the line
    q: np.ndarray            # (N, 7); NaN where the IK has no answer in the slot
    tip: np.ndarray          # (N, 3) where the tip must be
    spin: np.ndarray         # (N,)
    lean: np.ndarray         # (N, 2)
    layer: np.ndarray        # (N,) the layer at or before each sample
    ik_poses: int


@dataclass(frozen=True)
class Verdict:
    ok: bool
    reason: str              # what failed first; "" when ok
    where: int               # sample index of the first failure; -1 when ok
    clearance: float         # lower bound on the clearance along the path (nan if not reached)
    worst: dict              # the smallest value of every measured quantity


def _route_curve(lat: Lattice, k0: int, nodes: np.ndarray, smooth: float):
    """-> (curve, slot, s of the route's layers).  curve(s) gives spin, elbow value and lean
    (4 columns) at arc lengths s; beyond the route's ends it holds the end values."""
    k1 = k0 + len(nodes) - 1
    s_lay = lat.s[k0:k1 + 1]
    spin, q7, lean, slot = [], [], [], []
    for i, k in enumerate(range(k0, k1 + 1)):
        a, b, c, d = lat.params(k, nodes[i:i + 1])
        spin.append(a[0]), q7.append(b[0]), lean.append(c[0]), slot.append(d[0])
    values = np.column_stack([np.unwrap(np.array(spin)), q7, np.array(lean)])
    if smooth > 0.0:
        values = gaussian_filter1d(values, smooth, axis=0, mode="nearest")
        inner = CubicSpline(s_lay, values, axis=0, bc_type="natural")
    else:
        inner = PchipInterpolator(s_lay, values, axis=0)
    return (lambda s: inner(np.clip(s, s_lay[0], s_lay[-1]))), int(slot[0]), s_lay


def _solver(lat, curve, slot, points, s_points):
    arm, normal = lat.arm, lat.judge.normal

    def solve(s):
        v = curve(s)
        tips = polyline.at(points, s_points, s)
        Q, _ = arm.ik(arm.hand_pose(tips, normal, v[:, 0], v[:, 2:4]), v[:, 1])
        return Q[:, slot], tips, v
    return solve


def dense_path(lat: Lattice, k0: int, nodes: np.ndarray, points: np.ndarray,
               s_points: np.ndarray, cfg: Settings, smooth: float,
               ends: tuple[float, float] | None = None) -> Dense:
    """Re-solve the route k0.. (one node per layer) every `cfg.dense_step` along the line,
    from its first to its last layer, or over `ends` (s0, s1) when the piece was extended.

    The spin, lean and elbow value are read off a smooth curve through the route's nodes, so
    the joint path has no kink where the route steps from one grid value to the next (the
    timing step would have to round every kink off, slowly).  With `smooth` > 0 the node
    values are first averaged over about that many layers either side (a Gaussian), which
    turns the grid's staircase into the drift it stands for; with 0 the curve passes through
    every node and never overshoots them (PCHIP).  Past the route's end nodes the curve holds.
    """
    curve, slot, s_lay = _route_curve(lat, k0, nodes, smooth)
    solve = _solver(lat, curve, slot, points, s_points)
    s0, s1 = (s_lay[0], s_lay[-1]) if ends is None else ends
    s = polyline.dense_positions(s0, s1, s_points, cfg.dense_step)
    q, tips, v = solve(s)
    poses = len(s)
    arm = lat.arm
    # Between two samples the arm moves straight in joint space, and the pen leaves the line
    # a little.  Where it leaves by more than `chord_tol`, a sample is added halfway.
    for _ in range(cfg.max_refine):
        mid = 0.5 * (s[1:] + s[:-1])
        off = np.linalg.norm(arm.tip(0.5 * (q[1:] + q[:-1])) - polyline.at(points, s_points, mid),
                             axis=1)
        add = mid[off > cfg.chord_tol]
        if not len(add):
            break
        qa, ta, va = solve(add)
        poses += len(add)
        order = np.argsort(np.concatenate([s, add]), kind="stable")
        s = np.concatenate([s, add])[order]
        q, tips, v = (np.concatenate([x, y])[order] for x, y in ((q, qa), (tips, ta), (v, va)))
    j = np.clip(np.searchsorted(s_lay, s, side="right") - 1, 0, len(s_lay) - 2)
    return Dense(s, q, tips, np.mod(v[:, 0], 2.0 * np.pi), v[:, 2:4], j + k0, poses)


def reach_out(lat: Lattice, judge: Judge, k0: int, nodes: np.ndarray, points: np.ndarray,
              s_points: np.ndarray, cfg: Settings, smooth: float, at_end: bool,
              limit: float) -> float:
    """How far past its last (or before its first) layer the route can really go, towards
    `limit`: the end node's spin, lean and elbow value are held and the pen walks on along
    the line in `dense_step` steps while every sample passes the gates and the joints move
    smoothly; the last step is then halved a few times.  -> the arc length where it stops."""
    curve, slot, s_lay = _route_curve(lat, k0, nodes, smooth)
    solve = _solver(lat, curve, slot, points, s_points)
    start = s_lay[-1] if at_end else s_lay[0]
    q_prev = solve(np.array([start]))[0][0]

    def good(s, q_before):
        q = solve(np.array([s]))[0]
        ok = bool(np.all(np.isfinite(q)))
        ok = ok and np.abs(q[0] - q_before).max() <= cfg.dense_jump
        ok = ok and judge.reachable(q)[0] and judge.obstacle_clear(q)[0] >= 0.0
        return ok, q[0]

    n = max(1, int(np.ceil(abs(limit - start) / cfg.dense_step)))
    last = start
    for i in range(1, n + 1):
        s = start + (limit - start) * i / n
        ok, q = good(s, q_prev)
        if not ok:
            bad = s
            for _ in range(4):                   # halve the last step
                mid = 0.5 * (last + bad)
                ok, q = good(mid, q_prev)
                if ok:
                    last, q_prev = mid, q
                else:
                    bad = mid
            return last
        last, q_prev = s, q
    return last


def verify(judge: Judge, q: np.ndarray, tip: np.ndarray, cfg: Settings) -> Verdict:
    """Check a drawing path from its joint samples alone.  `tip`: where the pen must be."""
    q = np.asarray(q, float)
    n = len(q)
    worst = {}
    fails = []                                    # (first sample index, reason)

    def note(name, values, floor, shift=0):
        values = np.asarray(values, float)
        worst[name] = float(np.nanmin(values)) if len(values) else np.inf
        bad = np.flatnonzero(~(values >= floor))
        if len(bad):
            fails.append((int(bad[0]) + shift, name))

    finite = np.all(np.isfinite(q), axis=1)
    if not finite.all():
        i = int(np.flatnonzero(~finite)[0])
        return Verdict(False, "no IK answer in the slot", i, np.nan, {})
    if n < 2:
        return Verdict(False, "fewer than two samples", 0, np.nan, {})
    arm, g = judge.arm, judge.gates
    note("tip_on_line", -np.linalg.norm(arm.tip(q) - tip, axis=1), -cfg.tip_tol)
    # Every corner of the line is a sample, so between two samples the line is the straight
    # piece between their tips; halfway in joint space the pen must be near its middle.
    chord = np.linalg.norm(arm.tip(0.5 * (q[1:] + q[:-1])) - 0.5 * (tip[1:] + tip[:-1]), axis=1)
    note("between_samples", -chord, -cfg.chord_tol, shift=1)
    note("limit_margin", arm.limit_margin(q), g.limit_margin)
    note("sigma_min", arm.sigma_min(q), g.sigma_min)
    step = np.abs(np.diff(q, axis=0)).max(axis=1)
    note("joint_step", -step, -cfg.dense_jump, shift=1)
    note("self", judge.self_clear(q), 0.0)
    note("paper", judge.paper_clear(q), 0.0)
    note("obstacles", judge.obstacle_clear(q), 0.0)
    worst["tip_on_line"] = -worst["tip_on_line"]
    worst["joint_step"] = -worst["joint_step"]
    worst["between_samples"] = -worst["between_samples"]
    if fails:
        where, reason = min(fails)
        return Verdict(False, reason, where, np.nan, worst)
    # The timing step flies a path within `timing_deviation` of this one; the clearance must
    # cover what that can cost.
    floor = judge.lipschitz * cfg.timing_deviation
    clear = judge.path_clear(q)
    worst["path"] = clear
    if clear < floor:
        return Verdict(False, "path", _locate(judge, q, floor), clear, worst)
    return Verdict(True, "", -1, clear, worst)


def _locate(judge: Judge, q: np.ndarray, floor: float) -> int:
    """A sample next to where the path clearance goes below `floor`, by halving the path."""
    a, b = 0, len(q) - 1
    while b - a > 1:
        m = (a + b) // 2
        if judge.path_clear(q[a:m + 1]) < floor:
            b = m
        else:
            a = m
    return b
