"""Lift-offs: from a drawing configuration, the pen up into free space.  The set-down is the
same motion flown backwards.

A lift is a short joint path, solved by the IK every `step` metres of the pen's travel, each
sample the answer nearest the last (so the arm keeps its shape; a jump over `max_jump` means it
cannot).  Two kinds of path (which one is tried when: ladder.py):
- `rise_path`: the pen straight up along the paper normal; on the way up joint 7 and the hand's
  turn about the normal may change, and the pen may drift toward the base axis
- `along_path`: the pen back along the line just drawn, rising to a small height above it
`finish` turns a path into a `Lift`: every sample inside the gates (joint-limit margin, singular
value), the timed motion free as flown (the pen exempt from the paper, it is on its way to or
from it), and the top configuration holdable with the pen judged against the paper too (that
is what the free-space planner asks of a start).  Lesson L53: the lift ends at the nearest
pen-up configuration, never at one chosen for clearance elsewhere.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from aris.kernel.retime import retime_detailed
from aris.sequencer.guard import Guard
from aris.types import DrawRules, JointPath, Motion, Plane, Refusal, Trajectory

SAME = 1e-6           # rad: the IK answer that reproduces the drawing configuration itself


@dataclass(frozen=True)
class Lift:
    """A lift-off from a drawing configuration: `up` goes from it to `q_up`."""
    q_draw: np.ndarray
    q_up: np.ndarray
    up: Motion                        # kind "lift": q_draw -> q_up
    down: Motion                      # kind "lower": q_up -> q_draw, the same motion backwards
    how: str = ""                     # which rung of the ladder made it


def normal_of(paper: Plane) -> np.ndarray:
    n = np.asarray(paper.normal, float)
    return n / np.linalg.norm(n)


def follow(arm, q0: np.ndarray, T: np.ndarray, q7: np.ndarray, max_jump: float):
    """Joint samples through the hand poses T (first: q0's own), the IK answer nearest the
    last each time.  -> (N,7) path starting exactly at q0, or why not."""
    Q, ok = arm.ik(T, q7)
    first = np.where(ok[0], np.linalg.norm(np.nan_to_num(Q[0] - q0, nan=1e9), axis=1), np.inf)
    if first.min() > SAME:
        return "the IK does not reproduce the drawing configuration"
    out = [q0]
    for i in range(1, len(T)):
        d = np.where(ok[i], np.linalg.norm(np.nan_to_num(Q[i] - out[-1], nan=1e9), axis=1),
                     np.inf)
        b = int(np.argmin(d))
        if d[b] > max_jump:
            return f"the arm cannot keep its shape (sample {i} of {len(T) - 1})"
        out.append(Q[i, b])
    return np.array(out)


def rise_path(arm, q: np.ndarray, paper: Plane, height: float, step: float, max_jump: float,
              turn7: float = 0.0, spin: float = 0.0, inward: float = 0.0):
    """The pen straight up by `height`; in proportion to the rise joint 7 turns by `turn7`,
    the hand by `spin` about the normal through the tip, and the tip drifts `inward` metres
    toward the base axis.  -> path or why not."""
    n = normal_of(paper)
    T0 = arm.fk(q[None])[0]
    tip0 = arm.tip(q[None])[0]
    toward = -(tip0 - np.array([0.0, 0.0, tip0[2]]))          # to the base axis (base z)
    toward -= (toward @ n) * n
    toward /= max(np.linalg.norm(toward), 1e-12)
    k = int(np.ceil(np.hypot(height, inward) / step)) + 1
    rise = np.linspace(0.0, 1.0, k)
    T = np.repeat(T0[None], k, axis=0)
    T[:, :3, :3] = _about(n, spin * rise) @ T0[:3, :3]
    tips = tip0 + rise[:, None] * (height * n + inward * toward)
    T[:, :3, 3] = tips - T[:, :3, :3] @ arm.tool.tip_hand
    return follow(arm, q, T, q[6] + turn7 * rise, max_jump)


def along_path(arm, Q_in: np.ndarray, paper: Plane, height: float, back: float,
               max_jump: float):
    """The pen back along the line just drawn: through the drawing configurations `Q_in`
    (first: the end the pen leaves from, then inward), each with its own hand orientation and
    joint 7, the tip raised by min(height, distance travelled) above the line, for up to `back`
    metres.  -> (path, distance travelled at each sample) or why not."""
    n = normal_of(paper)
    tips = arm.tip(Q_in)
    d = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(tips, axis=0), axis=1))])
    keep = d <= back + 1e-12
    Q_in, tips, d = Q_in[keep], tips[keep], d[keep]
    if len(Q_in) < 2:
        return "the line is too short to go back along"
    T = arm.fk(Q_in)
    T[:, :3, 3] += np.minimum(height, d)[:, None] * n
    path = follow(arm, Q_in[0], T, Q_in[:, 6], max_jump)
    return path if isinstance(path, str) else (path, d)


def finish(arm, guard: Guard, path: np.ndarray, rules: DrawRules, how: str) -> Lift | str:
    """The lift along `path`, timed and checked; or why not."""
    g = rules.gates
    margin = arm.limit_margin(path)
    if margin.min() < g.limit_margin:
        return f"comes {margin.min():.3f} rad from a joint limit (gate {g.limit_margin})"
    sig = arm.sigma_min(path)
    if sig.min() < g.sigma_min:
        return f"singular value {sig.min():.3f} (gate {g.sigma_min})"
    why = guard.hold(path[-1], touching=False)
    if why is not None:
        return f"the free-space planner cannot start there: {why}"
    res = retime_detailed(JointPath(path), arm.limits, rules)
    if isinstance(res, Refusal):
        return f"cannot be timed: {res.reason} {res.detail}"
    flown = guard.flown(res.traj, touching=True)
    if flown < 0.0:
        return f"the lift comes {-flown * 1e3:.2f} mm too close"
    return Lift(q_draw=path[0], q_up=res.traj.q[-1], up=Motion("lift", res.traj),
                down=Motion("lower", reverse(res.traj)), how=how)


def _about(n: np.ndarray, angles: np.ndarray) -> np.ndarray:
    """(k,3,3) rotations by `angles` about the unit axis n (Rodrigues)."""
    K = np.array([[0, -n[2], n[1]], [n[2], 0, -n[0]], [-n[1], n[0], 0]])
    a = np.asarray(angles, float)[:, None, None]
    return np.eye(3) + np.sin(a) * K + (1.0 - np.cos(a)) * (K @ K)


def reverse(traj: Trajectory) -> Trajectory:
    """The same timed motion flown backwards (the cubic between two samples is symmetric)."""
    return Trajectory(t=traj.t[-1] - traj.t[::-1], q=traj.q[::-1].copy(), qd=-traj.qd[::-1])


def straight_limit(arm, guard: Guard, paper: Plane, q_draw: np.ndarray, rules: DrawRules,
                   step: float = 5e-4, top: float | None = None) -> tuple[float, str]:
    """How far the pen can be raised straight up from `q_draw` with the arm's shape held
    (hand orientation and joint 7 fixed, the IK answer nearest the last), and the gate that
    stops it: -> (metres, what stops it; "" if it reaches `top`, default `rules.lift_height`).
    Judged at samples `step` apart (a measurement, not a verdict)."""
    top = rules.lift_height if top is None else top
    n = np.asarray(paper.normal, float) / np.linalg.norm(paper.normal)
    k = int(np.ceil(top / step)) + 1
    h = np.linspace(0.0, top, k)
    T = np.repeat(arm.fk(q_draw[None]), k, axis=0)
    T[:, :3, 3] += h[:, None] * n
    Q, ok = arm.ik(T, np.full(k, q_draw[6]))
    g = rules.gates
    prev = np.asarray(q_draw, float)
    for i in range(1, k):
        d = np.where(ok[i], np.linalg.norm(np.nan_to_num(Q[i] - prev, nan=1e9), axis=1), np.inf)
        b = int(np.argmin(d))
        # 200 rad per metre of rise: a jump, a change of shape.  (A pose at a fold of the IK,
        # where two shapes meet, moves fast but continuously: about 90 rad/m at the fold.)
        if d[b] > 200 * step:
            return h[i - 1], "the IK changes the arm's shape"
        q = Q[i, b]
        m = float(arm.limit_margin(q[None])[0])
        if m < g.limit_margin:
            j = int(np.argmin(np.minimum(q - arm.limits.q_min, arm.limits.q_max - q)))
            return h[i - 1], f"joint-limit margin (joint {j + 1}, {m:.3f} rad)"
        s = float(arm.sigma_min(q[None])[0])
        if s < g.sigma_min:
            return h[i - 1], f"singular value {s:.3f}"
        why = guard.hold(q, touching=True)
        if why is not None:
            return h[i - 1], f"clearance: {why}"
        prev = q
    return top, ""
