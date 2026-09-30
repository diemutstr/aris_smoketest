"""The lift-off: from a drawing configuration, raise the pen tip `lift_height` straight up along
the paper normal.  The set-down is the same motion flown backwards.

The hand keeps its orientation and joint 7 its angle; the IK is solved every `step` metres of
rise and each sample takes the answer nearest the previous one, so the arm keeps its shape
(the same IK branch).  A jump larger than `max_jump` means the shape cannot be followed: no
lift-off here.  Every sample must pass the gates (joint-limit margin, singular value); the timed
motion must be free as flown (pen exempt from the paper, it is on its way to or from it); the
top configuration must be holdable with the pen judged against the paper too.

Lesson L53: the lift-off is the nearest pen-up configuration to the drawing one, never one
chosen for clearance elsewhere.
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
    up: Motion                        # q_draw -> q_up
    down: Motion                      # q_up -> q_draw, the same motion backwards


def lift_path(arm, q: np.ndarray, paper: Plane, height: float, step: float,
              max_jump: float, turn7: float = 0.0, spin: float = 0.0) -> np.ndarray | str:
    """Joint samples from q (first row, exactly) to the raised configuration, or why not.
    In proportion to the rise, joint 7 turns by `turn7` and the hand by `spin` about the
    paper normal through the pen tip (the tip itself goes straight up)."""
    n = np.asarray(paper.normal, float)
    n = n / np.linalg.norm(n)
    k = int(np.ceil(height / step)) + 1
    T0 = arm.fk(q[None])[0]
    tip0 = arm.tip(q[None])[0]
    rise = np.linspace(0.0, 1.0, k)
    T = np.repeat(T0[None], k, axis=0)
    T[:, :3, :3] = _about(n, spin * rise) @ T0[:3, :3]
    T[:, :3, 3] = tip0 + height * rise[:, None] * n - T[:, :3, :3] @ arm.tool.tip_hand
    Q, ok = arm.ik(T, q[6] + turn7 * rise)
    first = np.where(ok[0], np.linalg.norm(np.nan_to_num(Q[0] - q, nan=1e9), axis=1), np.inf)
    if first.min() > SAME:
        return "the IK does not reproduce the drawing configuration"
    out = [q]
    for i in range(1, k):
        d = np.where(ok[i], np.linalg.norm(np.nan_to_num(Q[i] - out[-1], nan=1e9), axis=1),
                     np.inf)
        b = int(np.argmin(d))
        if d[b] > max_jump:
            return f"the arm cannot keep its shape {i * height / (k - 1) * 1e3:.0f} mm up"
        out.append(Q[i, b])
    return np.array(out)


def _about(n: np.ndarray, angles: np.ndarray) -> np.ndarray:
    """(k,3,3) rotations by `angles` about the unit axis n (Rodrigues)."""
    K = np.array([[0, -n[2], n[1]], [n[2], 0, -n[0]], [-n[1], n[0], 0]])
    a = np.asarray(angles, float)[:, None, None]
    return np.eye(3) + np.sin(a) * K + (1.0 - np.cos(a)) * (K @ K)


def reverse(traj: Trajectory) -> Trajectory:
    """The same timed motion flown backwards (the cubic between two samples is symmetric)."""
    return Trajectory(t=traj.t[-1] - traj.t[::-1], q=traj.q[::-1].copy(), qd=-traj.qd[::-1])


def lift(arm, guard: Guard, paper: Plane, q_draw: np.ndarray, rules: DrawRules,
         step: float, max_jump: float, turns=((0.0, 0.0),)) -> Lift | str:
    """The lift-off from `q_draw`, timed and checked; or why there is none.

    The first of `turns` that gives a lift-off wins: (joint 7's turn, the hand's turn about the
    paper normal) during the lift, rad.  A lift with the arm's shape held can run into a
    joint-limit margin, a change of shape, or an obstacle beside the arm (lesson L7: lifting
    can swing the elbow toward a neighbour); turning the elbow or the hand a little on the way
    up often keeps clear.  The reason given is the first try's."""
    first = None
    for turn in turns:
        got = _lift(arm, guard, paper, np.asarray(q_draw, float), rules, step, max_jump, *turn)
        if not isinstance(got, str):
            return got
        first = first or got
    return first


def _lift(arm, guard, paper, q_draw, rules, step, max_jump, turn7, spin) -> Lift | str:
    path = lift_path(arm, q_draw, paper, rules.lift_height, step, max_jump, turn7, spin)
    if isinstance(path, str):
        return path
    g = rules.gates
    margin = arm.limit_margin(path)
    if margin.min() < g.limit_margin:
        return f"comes {margin.min():.3f} rad from a joint limit (gate {g.limit_margin})"
    sig = arm.sigma_min(path)
    if sig.min() < g.sigma_min:
        return f"singular value {sig.min():.3f} (gate {g.sigma_min})"
    res = retime_detailed(JointPath(path), arm.limits, rules)
    if isinstance(res, Refusal):
        return f"cannot be timed: {res.reason} {res.detail}"
    flown = guard.flown(res.traj, touching=True)
    if flown < 0.0:
        return f"the lift comes {-flown * 1e3:.2f} mm too close"
    why = guard.hold(path[-1], touching=False)
    if why is not None:
        return f"cannot hold the lifted pen: {why}"
    up = Motion("free", res.traj)
    return Lift(q_draw=path[0], q_up=res.traj.q[-1], up=up, down=Motion("free", reverse(res.traj)))
