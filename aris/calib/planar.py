"""The planar solve of the mark job: every slot's (x, y, yaw) and every mark's (x, y) from the
touches, each seen from its slot's base origin in table axes (a horizontal vector e: the mark is
at Rz(yaw) e + (x, y)).  Pure numpy and one scipy least squares.  Called by `marks.solve_marks`.

The touches fix everything up to one planar rigid motion of the whole layout (the frame).  Marks
solved before ("fixed", two or more) hold it; otherwise the layout is solved with one slot held
where it is, then moved as a whole onto the nominal mountings (rig.nominal_pose): the mean
yaw error is zero and the mean axis lands on the mean nominal one (`frame_motion`).  The marks
are hand-drawn, centimetres off nominal, and do not fix the frame.
"""
from __future__ import annotations

import numpy as np
from scipy.optimize import least_squares


def rz(a) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _rot2(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s], [s, c]])


def yaw_between(Ra, Rb) -> float:
    """The turn about table z that takes rotation Rb to Ra (exact when they differ by a turn
    about z, the leading part when tilts differ slightly)."""
    M = Ra @ Rb.T
    return float(np.arctan2(M[1, 0] - M[0, 1], M[0, 0] + M[1, 1]))


def kabsch2(src, dst):
    """The planar rigid motion (angle, shift) taking points src (N,2) onto dst best."""
    sc, dc = src.mean(axis=0), dst.mean(axis=0)
    A, B = src - sc, dst - dc
    a = float(np.arctan2(np.sum(A[:, 0] * B[:, 1] - A[:, 1] * B[:, 0]),
                         np.sum(A[:, 0] * B[:, 0] + A[:, 1] * B[:, 1])))
    return a, dc - _rot2(a) @ sc


def _closed_form(rig, slots, obs, fixed):
    """Place slots one by one: the first where the rig has it (unless marks are fixed), then
    each slot sharing two placed marks by a rigid fit, else one placed mark with its turn as
    the rig has it.  Each placed slot places its other marks."""
    mean = {}
    for s, n, e in obs:
        mean.setdefault((s, n), []).append(e)
    mean = {k: np.mean(v, axis=0) for k, v in mean.items()}
    xy, pose = dict(fixed), {}
    todo = list(slots)
    while todo:
        k_of = {s: [n for (t, n) in mean if t == s and n in xy] for s in todo}
        s = max(todo, key=lambda t: (len(k_of[t]), -todo.index(t)))
        names = k_of[s]
        if len(names) >= 2:
            a, t = kabsch2(np.array([mean[s, n] for n in names]), np.array([xy[n] for n in names]))
        elif names:
            a = 0.0
            t = xy[names[0]] - mean[s, names[0]]
        else:
            a, t = 0.0, rig.T_table_base(s)[:2, 3]
        pose[s] = np.r_[t, a]
        for (u, n), e in mean.items():
            if u == s and n not in xy:
                xy[n] = _rot2(a) @ e + t
        todo.remove(s)
    return pose, xy


def solve(rig, obs, slots, names, fixed):
    """-> (pose {slot: (x, y, yaw against the rig's rotation)}, xy {mark: (2,)}, per-touch
    residual norms (N,), what fixed the frame) or a refusal string."""
    if not slots or not obs:
        return "no touches"
    if len(slots) == 1 and not fixed:
        return f"{slots[0]} alone needs marks solved before (two of its marks known)"
    held = None if fixed else slots[0]
    moving = [s for s in slots if s != held]
    free = [n for n in names if n not in fixed]
    si = np.array([slots.index(s) for s, _, _ in obs])
    mi = np.array([names.index(n) for _, n, _ in obs])
    e = np.array([o[2] for o in obs])
    pose0, xy0 = _closed_form(rig, slots, obs, fixed)

    def unpack(v):
        pose = dict(pose0) if held else {}
        pose.update({s: v[3 * i:3 * i + 3] for i, s in enumerate(moving)})
        k = 3 * len(moving)
        xy = dict(fixed)
        xy.update({n: v[k + 2 * i:k + 2 * i + 2] for i, n in enumerate(free)})
        return pose, xy

    def residuals(v):
        pose, xy = unpack(v)
        P = np.array([pose[s] for s in slots])
        M = np.array([xy[n] for n in names])
        c, s = np.cos(P[si, 2]), np.sin(P[si, 2])
        pred = np.column_stack([c * e[:, 0] - s * e[:, 1], s * e[:, 0] + c * e[:, 1]])
        return (pred + P[si, :2] - M[mi]).reshape(-1)

    v0 = np.concatenate([pose0[s] for s in moving] + [xy0[n] for n in free])
    fit = least_squares(residuals, v0, method="trf", xtol=1e-15, ftol=1e-15, gtol=1e-15)
    sv = np.linalg.svd(fit.jac, compute_uv=False)
    if len(sv) < v0.size or sv[-1] < 1e-9 * sv[0]:
        return "the set is not rigid: some slots hang on the rest by one mark only"
    pose, xy = unpack(fit.x)
    r = np.linalg.norm(fit.fun.reshape(-1, 2), axis=1)
    if fixed:
        return pose, xy, r, f"held by the known marks {sorted(fixed)}"
    a, t = _frame_fit(rig, pose)
    pose = {s: np.r_[_rot2(a) @ p[:2] + t, p[2] + a] for s, p in pose.items()}
    xy = {n: _rot2(a) @ p + t for n, p in xy.items()}
    return pose, xy, r, f"fitted onto the nominal mountings of {sorted(pose)}"


def frame_motion(axes, yaws, nominal_axes):
    """The planar rigid motion (angle, shift) of the whole layout that makes the mean yaw error
    zero and puts the mean axis on the mean nominal one.  axes, nominal_axes (N,2); yaws (N,)
    each slot's turn against its nominal pose.  The turn comes from the yaws, not from the axis
    positions: two arms 0.61 m apart, each 1-2 cm off, would tilt a fit of the positions by up
    to 4 deg, while each arm's own yaw is off by mrad."""
    a = -float(np.mean(yaws))
    return a, np.mean(nominal_axes, axis=0) - _rot2(a) @ np.mean(axes, axis=0)


def _frame_fit(rig, pose):
    """`frame_motion` for the solved poses."""
    slots = sorted(pose)
    yaws = [yaw_between(rz(pose[s][2]) @ rig.T_table_base(s)[:3, :3],
                        rig.nominal_pose(s)[:3, :3]) for s in slots]
    return frame_motion(np.array([pose[s][:2] for s in slots]), np.array(yaws),
                        np.array([rig.nominal_pose(s)[:2, 3] for s in slots]))
