"""The meetings calibration of one row (Pete, 2026-10-08): the two arms' pen tips touching in
the air, both arms hand-guided (the person switches Desk's modes herself), the joints of both
read at standstill.

A meeting is one physical point seen by both arms: T_L p_L = T_R p_R, with p the pen tip in each
arm's base frame (forward kinematics with each slot's tool as the rig has it: the touch-off's
tip when the pen part applies, else the pen's nominal one).  With the true pose of each arm
written as its believed pose turned by psi about the vertical through its base origin and
shifted by delta, a meeting reads, horizontally,

    Rz(psi_R) e_R - Rz(psi_L) e_L + (t_R - t_L) + (delta_R - delta_L) = 0,   e = (R^ p)_xy.

Moving or turning both arms together changes no meeting, so the frame is the mark job's: the
pair's mean position and mean yaw on the nominal mountings, built into the unknowns
(psi = c -/+ u, delta = (sum -/+ D) / 2).
- one meeting: D only; the relative turn stays nominal (`yaw_from` "nominal");
- two or more, far apart: u and D by least squares (a 1-D Gauss-Newton on u, D linear in it,
  no small-angle approximation); the leftover is the residual.
The height is not solved (the plane job's), but how far the two tips disagree in height is
reported.  Written with `files.write_mark_solution`, method "meetings".  See
docs/modules/calib.md.
"""
from __future__ import annotations

import numpy as np

from aris.calib import planar
from aris.calib.marks import POSE_SHIFT_MAX, POSE_YAW_MAX, MarkSolution, SlotFit
from aris.types import Slot

RESIDUAL_MAX_M = 0.002    # with two or more meetings, they do not fit one rigid correction:
                          # the tips were not touching, or an arm moved while being read
YAW_BASE_MIN_M = 0.30     # meetings closer than this cannot tell a turn from a shift


def _refuse(why) -> MarkSolution:
    nan = float("nan")
    return MarkSolution(False, why, {}, {}, nan, nan, (), (), "")


def _tips(rig, meetings, slots):
    """-> (e_L (K,2), e_R (K,2), p_table_L (K,3), p_table_R (K,3)) or a refusal string."""
    Q = {s: [] for s in slots}
    for k, m in enumerate(meetings):
        if not isinstance(m, dict) or set(m) != set(slots):
            return f"meeting {k} must give the joints of exactly {slots[0]} and {slots[1]}"
        for s in slots:
            q = np.asarray(m[s], float).reshape(-1)
            if q.shape != (7,) or not np.all(np.isfinite(q)):
                return f"meeting {k}: {s}'s joints are not 7 numbers"
            Q[s].append(q)
    out = []
    for s in slots:
        p = rig.arm(s).tip(np.array(Q[s]))
        T = rig.T_table_base(s)
        out.append((p @ T[:3, :3].T)[:, :2])
        out.append(p @ T[:3, :3].T + T[:3, 3])
    return out[0], out[2], out[1], out[3]


def _pair(rig, eL, eR, slots, solve_yaw):
    """-> ({slot: (x, y, yaw against the rig's rotation)}, per-meeting residual norms)."""
    L, R = slots
    tL, tR = rig.T_table_base(L)[:2, 3], rig.T_table_base(R)[:2, 3]
    y0 = [planar.yaw_between(rig.T_table_base(s)[:3, :3], rig.nominal_pose(s)[:3, :3])
          for s in slots]
    c = -(y0[0] + y0[1]) / 2.0
    total = 2.0 * np.mean([rig.nominal_pose(s)[:2, 3] for s in slots], axis=0) - tL - tR
    rot = lambda a, e: e @ planar._rot2(a).T

    def gap(u):               # per meeting, what D must cancel
        return rot(c + u, eR) - rot(c - u, eL) + (tR - tL)

    u = (y0[0] - y0[1]) / 2.0                 # the relative turn as nominal
    if solve_yaw:
        u = planar.kabsch2(eR, eL)[0] / 2.0     # start: the rigid turn taking R's view to L's
        for _ in range(30):
            f = lambda v: (gap(v) - gap(v).mean(axis=0)).reshape(-1)
            r0, h = f(u), 1e-7
            J = (f(u + h) - r0) / h
            step = -float(J @ r0) / float(J @ J)
            u += step
            if abs(step) < 1e-14:
                break
    D = -gap(u).mean(axis=0)
    res = np.linalg.norm(gap(u) + D, axis=1)
    return {L: np.r_[tL + (total - D) / 2.0, c - u],
            R: np.r_[tR + (total + D) / 2.0, c + u]}, res


def solve_meetings(rig, meetings, slots: tuple[Slot, Slot]) -> MarkSolution:
    """The row's two poses from meetings [{L: q_L, R: q_R}, ...] (joints at standstill, the pen
    tips touching).  Never raises on bad data: a refusal says what is wrong."""
    if len(slots) != 2 or slots[0] == slots[1]:
        return _refuse(f"two different slots are needed, got {slots}")
    for s in slots:
        if s not in rig.arm_ids:
            return _refuse(f"no arm in slot {s!r} (mounted: {rig.arm_ids})")
    if not meetings:
        return _refuse("no meetings")
    got = _tips(rig, list(meetings), slots)
    if isinstance(got, str):
        return _refuse(got)
    eL, eR, pL, pR = got
    where = (pL + pR) / 2.0
    apart = max(np.linalg.norm(a[:2] - b[:2]) for a in where for b in where)
    solve_yaw = len(meetings) >= 2
    if solve_yaw and apart < YAW_BASE_MIN_M:
        return _refuse(f"the meetings are {apart:.2f} m apart; yaw needs them far apart (at "
                       f"least {YAW_BASE_MIN_M:g} m) — use the second spot")
    pose, res = _pair(rig, eL, eR, slots, solve_yaw)
    yaw_from = "meetings" if solve_yaw else "nominal"
    dz = float(np.max(np.abs(pL[:, 2] - pR[:, 2])))
    notes = [f"the tips disagree in height by up to {dz * 1e3:.2f} mm (heights are the plane "
             f"job's and the pens'; not solved here)"]
    if not solve_yaw:
        notes.insert(0, "one meeting: shift only, the relative yaw kept nominal")
    fits, why = {}, []
    for s, (x, y, yaw) in pose.items():
        T0, Tn = rig.T_table_base(s), rig.nominal_pose(s)
        T = T0.copy()
        T[:3, :3] = planar.rz(yaw) @ T0[:3, :3]
        T[:2, 3] = (x, y)
        shift = float(np.linalg.norm(T[:2, 3] - Tn[:2, 3]))
        ynom = planar.yaw_between(T[:3, :3], Tn[:3, :3])
        fits[s] = SlotFit(s, T, T0, rig.arm(s).tool.tip_hand.copy(), None, "", shift, ynom,
                          float(np.sqrt(np.mean(res ** 2))), len(meetings), rig.pen_name, None,
                          "meetings", yaw_from)
        if shift > POSE_SHIFT_MAX or abs(ynom) > POSE_YAW_MAX:
            why.append(f"refused: {s} {shift:.3f} m and {np.rad2deg(ynom):+.2f} deg from its "
                       f"nominal pose — wrong slot or wrong robot?")
    k = int(np.argmax(res))
    if solve_yaw and res[k] > RESIDUAL_MAX_M:
        why.append(f"the meetings do not fit one correction: {res[k] * 1e3:.2f} mm left at "
                   f"meeting {k} (limit {RESIDUAL_MAX_M * 1e3:g}); the tips were not touching "
                   f"or an arm moved, meet again")
    return MarkSolution(not why, "; ".join(why), fits, {}, float(np.sqrt(np.mean(res ** 2))),
                        float(res.max()), (), tuple(notes),
                        f"meetings of {slots[0]}/{slots[1]} ({len(meetings)}); the pair's mean "
                        f"position and mean yaw on the nominal mountings; yaw from {yaw_from}")
