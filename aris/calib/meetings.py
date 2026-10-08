"""The meetings calibration (Pete, 2026-10-08): two arms' pen tips touching in the air, both
hand-guided (the person switches Desk's modes herself), the joints of both read at standstill.
Any pair that shares a spot may meet: the row pairs at one or two spots, the column pairs at the
seam spots (the 7 pairs rig.json's marks define).

A meeting is one physical point seen by both arms: T_a p_a = T_b p_b, with p the pen tip in each
arm's base frame (forward kinematics with each slot's tool as the rig has it: the touch-off's
tip when the pen part applies, else the pen's nominal one).  Each slot's true pose is its
believed pose turned by psi about the vertical through its base origin and shifted by delta;
horizontally a meeting reads

    Rz(psi_a) e_a + t_a + delta_a - Rz(psi_b) e_b - t_b - delta_b = 0,   e = (R^ p)_xy,

two equations per meeting.  The frame (moving or turning everything together changes no
meeting) is fixed by a reference slot held exactly at its nominal pose (rig.json
`marks.reference_slot`).  A slot's yaw is solved when it met at points at least 0.3 m apart and
the meetings pin it (the Jacobian keeps full rank); otherwise it is held at nominal and the
result says so.  Gauss-Newton (scipy least squares, exact rotations); heights are not solved,
their disagreement is reported.  Written with `files.write_mark_solution`, method "meetings".
See docs/modules/calib.md.
"""
from __future__ import annotations

import numpy as np
from scipy.optimize import least_squares

from aris.calib import planar
from aris.calib.marks import POSE_SHIFT_MAX, POSE_YAW_MAX, MarkSolution, SlotFit

RESIDUAL_MAX_M = 0.002    # a meeting the solution misses by more: the tips were not touching,
                          # or an arm moved while being read
YAW_BASE_MIN_M = 0.30     # a slot's meetings closer together than this cannot tell its turn
                          # from its shift
RANK_TOL = 1e-6           # relative singular value below which a direction is not determined


def _refuse(why, notes=()) -> MarkSolution:
    nan = float("nan")
    return MarkSolution(False, why, {}, {}, nan, nan, (), tuple(notes), "")


def _read(rig, meetings):
    """-> (rows [(a, b, spot)], e_a (K,2), e_b (K,2), p_a table (K,3), p_b table (K,3)) or why."""
    rows, ea, eb, pa, pb = [], [], [], [], []
    for k, m in enumerate(meetings):
        if not isinstance(m, (tuple, list)) or len(m) != 5:
            return f"meeting {k} must be (slot_a, q_a, slot_b, q_b, spot)"
        a, qa, b, qb, spot = m
        if a == b:
            return f"meeting {k}: {a} cannot meet itself"
        for s, q in ((a, qa), (b, qb)):
            if s not in rig.arm_ids:
                return f"meeting {k}: no arm in slot {s!r} (mounted: {rig.arm_ids})"
            q = np.asarray(q, float).reshape(-1)
            if q.shape != (7,) or not np.all(np.isfinite(q)):
                return f"meeting {k}: {s}'s joints are not 7 numbers"
        for s, q, e, p in ((a, qa, ea, pa), (b, qb, eb, pb)):
            T = rig.T_table_base(s)
            tip = rig.arm(s).tip(np.asarray(q, float).reshape(1, 7))[0]
            e.append((T[:3, :3] @ tip)[:2])
            p.append(T[:3, :3] @ tip + T[:3, 3])
        rows.append((a, b, str(spot)))
    return rows, np.array(ea), np.array(eb), np.array(pa), np.array(pb)


def _groups(slots, rows):
    """Connected groups of slots, linked by meetings, in `slots` order."""
    left, out = list(slots), []
    while left:
        group, todo = {left[0]}, [left[0]]
        while todo:
            s = todo.pop()
            for a, b, _ in rows:
                for u, v in ((a, b), (b, a)):
                    if u == s and v not in group:
                        group.add(v)
                        todo.append(v)
        out.append([s for s in slots if s in group])
        left = [s for s in left if s not in group]
    return out


class _Problem:
    """The unknowns: delta (2) for every slot but the reference, psi for the slots whose yaw is
    free; held yaws sit at nominal (psi = -yaw of the believed pose against nominal)."""

    def __init__(self, rig, slots, ref, free_yaw, rows, ea, eb):
        self.slots, self.ref, self.rows, self.ea, self.eb = slots, ref, rows, ea, eb
        self.t = {s: rig.T_table_base(s)[:2, 3] for s in slots}
        self.y0 = {s: planar.yaw_between(rig.T_table_base(s)[:3, :3],
                                         rig.nominal_pose(s)[:3, :3]) for s in slots}
        self.ref_delta = rig.nominal_pose(ref)[:2, 3] - self.t[ref]
        self.moving = [s for s in slots if s != ref]
        self.free = [s for s in slots if s in free_yaw and s != ref]

    def unpack(self, v):
        n = 2 * len(self.moving)
        delta = {s: v[2 * i:2 * i + 2] for i, s in enumerate(self.moving)}
        delta[self.ref] = self.ref_delta
        psi = {s: -self.y0[s] for s in self.slots}
        psi.update({s: v[n + i] for i, s in enumerate(self.free)})
        return delta, psi

    def size(self):
        return 2 * len(self.moving) + len(self.free)

    def residuals(self, v):
        delta, psi = self.unpack(v)
        out = []
        for k, (a, b, _) in enumerate(self.rows):
            out.append(planar._rot2(psi[a]) @ self.ea[k] + self.t[a] + delta[a]
                       - planar._rot2(psi[b]) @ self.eb[k] - self.t[b] - delta[b])
        return np.concatenate(out)

    def jacobian(self, v):
        _, psi = self.unpack(v)
        J = np.zeros((2 * len(self.rows), self.size()))
        n = 2 * len(self.moving)
        d_rot = lambda a: np.array([[-np.sin(a), -np.cos(a)], [np.cos(a), -np.sin(a)]])
        for k, (a, b, _) in enumerate(self.rows):
            for s, sign, e in ((a, 1.0, self.ea[k]), (b, -1.0, self.eb[k])):
                if s in self.moving:
                    i = self.moving.index(s)
                    J[2 * k:2 * k + 2, 2 * i:2 * i + 2] += sign * np.eye(2)
                if s in self.free:
                    J[2 * k:2 * k + 2, n + self.free.index(s)] += sign * d_rot(psi[s]) @ e
        return J


def _choose_yaws(rig, slots, ref, rows, ea, eb, pa, pb, notes):
    """The slots whose yaw is solved.  Held at nominal: a slot whose meetings are closer than
    YAW_BASE_MIN_M, then, while the Jacobian is short of full rank, the yaw that takes the
    largest part in the undetermined direction."""
    free = set()
    for s in slots:
        pts = [p for (a, b, _), p_a, p_b in zip(rows, pa, pb) for u, p in ((a, p_a), (b, p_b))
               if u == s]
        far = max(np.linalg.norm(p[:2] - q[:2]) for p in pts for q in pts)
        if far >= YAW_BASE_MIN_M:
            free.add(s)
        elif s != ref:
            notes.append(f"{s}'s yaw is nominal: it met at one point only"
                         f"{'' if far < 1e-3 else f' (its meetings are {far:.2f} m apart)'}; "
                         f"a second meeting (--yaw) determines it")
    while True:
        P = _Problem(rig, slots, ref, free, rows, ea, eb)
        J = P.jacobian(np.zeros(P.size()))
        if not J.size:
            return free
        _, sv, Vt = np.linalg.svd(J)
        if len(sv) >= J.shape[1] and sv[-1] > RANK_TOL * sv[0]:
            return free
        null = Vt[-1] if len(sv) >= J.shape[1] else Vt[len(sv):][0]
        yaw_part = np.abs(null[2 * len(P.moving):])
        if not len(yaw_part) or yaw_part.max() < 1e-6:
            return None                       # a shift is free: the set is not rigid
        s = P.free[int(np.argmax(yaw_part))]
        free.discard(s)
        notes.append(f"{s}'s yaw is nominal: these meetings do not determine it; a second "
                     f"meeting of its row (--yaw) does")


def solve_meetings(rig, meetings, slots=None, reference=None) -> MarkSolution:
    """Every slot's x, y, yaw from meetings [(slot_a, q_a, slot_b, q_b, spot), ...].

    slots: the slots expected (each must meet someone); default those that appear.  reference:
    the slot held at its nominal pose; default rig.json's `marks.reference_slot`, else (or when
    it is not among the solved slots) the first solved slot in rig order.  Never raises on bad
    data: a refusal says what is wrong."""
    meetings = list(meetings or [])
    if not meetings:
        return _refuse("no meetings")
    got = _read(rig, meetings)
    if isinstance(got, str):
        return _refuse(got)
    rows, ea, eb, pa, pb = got
    met = {s for a, b, _ in rows for s in (a, b)}
    order = [s for s in rig.arm_ids if s in met or (slots and s in slots)]
    lonely = [s for s in (slots or ()) if s not in met]
    if lonely:
        return _refuse(f"{'/'.join(lonely)} met no other arm")
    notes = []
    ref = reference or getattr(rig, "reference_slot", None)
    if ref is None:
        notes.append(f"the rig names no reference slot; {order[0]} (the first solved slot in "
                     f"rig order) is held at its nominal pose")
        ref = order[0]
    elif ref not in order:
        notes.append(f"the reference slot {ref} is not in this solve; {order[0]} (the first "
                     f"solved slot in rig order) is held at its nominal pose instead")
        ref = order[0]
    groups = _groups(order, rows)
    if len(groups) > 1:
        apart = [g for g in groups if ref not in g]
        return _refuse("; ".join(f"{'/'.join(g)} {'is' if len(g) == 1 else 'are'} not linked "
                                 f"to {ref} by any meeting" for g in apart))
    free = _choose_yaws(rig, order, ref, rows, ea, eb, pa, pb, notes)
    if free is None:
        return _refuse("the meetings do not hold every slot in place (a shift is free)", notes)
    P = _Problem(rig, order, ref, free, rows, ea, eb)
    fit = least_squares(P.residuals, np.zeros(P.size()), jac=P.jacobian, method="trf",
                        xtol=1e-15, ftol=1e-15, gtol=1e-15)
    res = np.linalg.norm(fit.fun.reshape(-1, 2), axis=1)
    delta, psi = P.unpack(fit.x)
    dz = float(np.max(np.abs(pa[:, 2] - pb[:, 2])))
    notes.append(f"the tips disagree in height by up to {dz * 1e3:.2f} mm (heights are the "
                 f"plane job's and the pens'; not solved here)")
    return _result(rig, order, ref, free, delta, psi, res, rows, notes)


def _result(rig, order, ref, free, delta, psi, res, rows, notes) -> MarkSolution:
    fits, why = {}, []
    for s in order:
        T0, Tn = rig.T_table_base(s), rig.nominal_pose(s)
        T = T0.copy()
        T[:3, :3] = planar.rz(psi[s]) @ T0[:3, :3]
        T[:2, 3] = T0[:2, 3] + delta[s]
        shift = float(np.linalg.norm(T[:2, 3] - Tn[:2, 3]))
        ynom = planar.yaw_between(T[:3, :3], Tn[:3, :3])
        mine = [k for k, (a, b, _) in enumerate(rows) if s in (a, b)]
        yaw_from = "reference" if s == ref else "meetings" if s in free else "nominal"
        fits[s] = SlotFit(s, T, T0, rig.arm(s).tool.tip_hand.copy(), None, "", shift, ynom,
                          float(np.sqrt(np.mean(res[mine] ** 2))), len(mine), rig.pen_name,
                          None, "meetings", yaw_from)
        if shift > POSE_SHIFT_MAX or abs(ynom) > POSE_YAW_MAX:
            why.append(f"refused: {s} {shift:.3f} m and {np.rad2deg(ynom):+.2f} deg from its "
                       f"nominal pose — wrong slot or wrong robot?")
    k = int(np.argmax(res))
    if res[k] > RESIDUAL_MAX_M:
        a, b, spot = rows[k]
        why.append(f"the meetings do not fit one layout: {res[k] * 1e3:.2f} mm left at the "
                   f"meeting of {a} and {b} at {spot} (limit {RESIDUAL_MAX_M * 1e3:g}); the tips "
                   f"were not touching or an arm moved, meet again")
    return MarkSolution(not why, "; ".join(why), fits, {}, float(np.sqrt(np.mean(res ** 2))),
                        float(res.max()), (), tuple(notes),
                        f"{ref} held at its nominal pose (the reference slot); {len(rows)} "
                        f"meetings")
