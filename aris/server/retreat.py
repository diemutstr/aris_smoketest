"""Retreat: an arm standing with a joint closer to its limit than the gates' margin (or past
it, up to PAST_MAX: an arm stopped or guided there) first moves only those joints, straight in
joint space, to the margin plus INSIDE; the other joints stay.  Then, or else: an arm standing
closer to another arm than the rig's arm-to-arm clearance (an
interrupted meeting of two pen tips, a stop at the wrong moment) first moves only AWAY: its pen
tip straight up RISE, then horizontally straight away from the nearest other arm (from that
arm's base axis toward its own) until the two bodies are the clearance plus MARGIN apart.  The
tip path is followed by IK with the hand's orientation kept, timed at the free speed, and
checked as a motion of kind "retreat" (the checker: the distance to every other arm never
decreases; paper, steel, fences and limits as for a free move).  One arm at a time, in rig
order, each in its own phase "retreat <slot>", the others standing.  Every job that plans from
where the arms stand (park, and through it mark and crosses; calibrate and touch-off) does this
first; a drawing is refused instead (park first).
"""
from __future__ import annotations

from dataclasses import replace

import numpy as np

from aris.check import check
from aris.kernel.geometry import segment_segment_distance
from aris.kernel.retime import retime_detailed
from aris.types import JointPath, Motion

RISE = 0.030             # m straight up first
INSIDE = 0.05            # rad beyond the gates' limit margin where a joint retreat ends
PAST_MAX = 0.10          # rad: a joint further past its limit than this is not moved
JOINT_STEP = 0.01        # rad between samples of a joint retreat
MARGIN = 0.020           # m beyond the clearance where the retreat ends
STEP = 0.003             # m between IK samples along the tip path
MAX_AWAY = 0.40          # m: the farthest it goes sideways


def clearance(rig) -> float:
    """The arm-to-arm clearance the planners keep (rig.json, with its planning allowance)."""
    return float(rig.clearance["arm_to_arm_m"] + rig.allowance["arm_to_arm_m"])


def _body_table(rig, a, q):
    b = rig.arm(a).body(np.asarray(q, float)[None])
    T = rig.T_table_base(a)
    move = ~np.asarray(b.is_fixed) if hasattr(b, "is_fixed") else np.ones(len(b.radius), bool)
    p0 = b.p0[0][move] @ T[:3, :3].T + T[:3, 3]
    p1 = b.p1[0][move] @ T[:3, :3].T + T[:3, 3]
    return p0, p1, np.asarray(b.radius)[move]


def gap(rig, a, qa, b, qb) -> float:
    """The smallest distance between arm a's and arm b's moving bodies (capsule surfaces), m."""
    A0, A1, ra = _body_table(rig, a, qa)
    B0, B1, rb = _body_table(rig, b, qb)
    d = segment_segment_distance(A0[:, None], A1[:, None], B0[None], B1[None])
    return float((d - ra[:, None] - rb[None]).min())


def nearest(rig, a, now) -> tuple[str | None, float]:
    """(the other arm nearest to arm a as they stand, the gap) — (None, inf) alone."""
    best = (None, np.inf)
    for b in rig.arm_ids:
        if b != a:
            g = gap(rig, a, now[a], b, now[b])
            if g < best[1]:
                best = (b, g)
    return best


def _track(rig, a, q0, tips) -> np.ndarray | str:
    """Joints along the table-frame tip path `tips`, the hand's orientation kept (joint 7 free
    within ±0.05 rad per sample), never closer to a joint limit than the gates' margin (or
    than the start, if that is closer)."""
    arm = rig.arm(a)
    floor = min(float(rig.gates().limit_margin), float(arm.limit_margin(np.asarray(q0)[None])[0]))
    T_tb = rig.T_base_table(a)
    T0 = arm.fk(np.asarray(q0, float)[None])[0]
    tip_hand = arm.tool.tip_hand
    out, q = [np.asarray(q0, float)], np.asarray(q0, float)
    for p in tips[1:]:
        pb = T_tb[:3, :3] @ p + T_tb[:3, 3]
        T = T0.copy()
        T[:3, 3] = pb - T0[:3, :3] @ tip_hand
        q7s = q[6] + np.linspace(-0.05, 0.05, 5)
        Q, ok = arm.ik(np.repeat(T[None], len(q7s), axis=0), q7s)
        flat, good = Q.reshape(-1, 7), ok.reshape(-1)
        good = good & (arm.limit_margin(np.nan_to_num(flat)) >= floor - 1e-9)
        if not good.any():
            return "no arm configuration follows the retreat"
        d = np.where(good, np.abs(np.nan_to_num(flat - q, nan=1e9)).max(axis=1), np.inf)
        k = int(np.argmin(d))
        if d[k] > 0.2:
            return "the retreat would swing the arm (no nearby configuration)"
        q = flat[k]
        out.append(q)
    return np.array(out)


def plan_retreat(st, a, now) -> Motion | str:
    """The retreat of arm a from where everything stands (`now`), unchecked, or why not."""
    rig = st.rig
    other, g0 = nearest(rig, a, now)
    want = clearance(rig) + MARGIN
    tip = rig.to_table(a, rig.arm(a).tip(np.asarray(now[a], float)[None])[0])
    away = rig.T_table_base(a)[:2, 3] - rig.T_table_base(other)[:2, 3]
    away = away / max(np.linalg.norm(away), 1e-9)
    up = [tip + np.array([0.0, 0.0, z]) for z in np.arange(0.0, RISE + 1e-9, STEP)]
    top = up[-1]
    path = _track(rig, a, now[a], np.array(up))
    if isinstance(path, str):
        return path
    for s in np.arange(STEP, MAX_AWAY + 1e-9, STEP):
        if gap(rig, a, path[-1], other, now[other]) >= want:
            break
        more = _track(rig, a, path[-1], np.array([top + (s - STEP) * np.r_[away, 0.0],
                                                  top + s * np.r_[away, 0.0]]))
        if isinstance(more, str):
            return more
        path = np.vstack([path, more[1:]])
    else:
        return f"{a} cannot get {want * 1e3:.0f} mm from {other} within {MAX_AWAY:g} m"
    res = retime_detailed(JointPath(path), rig.arm(a).limits, st.rules_for(a), smooth=True,
                          tip_of=rig.arm(a).tip)
    if not hasattr(res, "traj"):
        return f"the retreat cannot be timed: {res.reason} {res.detail}"
    return Motion("retreat", res.traj)


def joint_retreat(st, a, q) -> Motion | str | None:
    """The joints of arm a closer to a limit than the gates' margin moved straight to the margin
    plus INSIDE (the others unchanged), timed at the free speed: a Motion, None when every
    joint is inside the margin, or why not."""
    rig = st.rig
    lim, margin = rig.arm(a).limits, float(rig.gates().limit_margin)
    q = np.asarray(q, float)
    lo, hi = np.asarray(lim.q_min, float), np.asarray(lim.q_max, float)
    goal = q.copy()
    for j in range(7):
        if q[j] - lo[j] < margin:
            goal[j] = lo[j] + margin + INSIDE
        elif hi[j] - q[j] < margin:
            goal[j] = hi[j] - margin - INSIDE
        past = max(lo[j] - q[j], q[j] - hi[j])
        if past > PAST_MAX:
            return (f"joint {j + 1} stands {past:.3f} rad past its limit (more than "
                    f"{PAST_MAX:g}): a person must look")
    if np.array_equal(goal, q):
        return None
    n = max(2, int(np.ceil(np.abs(goal - q).max() / JOINT_STEP)) + 1)
    path = q + np.linspace(0.0, 1.0, n)[:, None] * (goal - q)
    # timed with the position box widened to where the arm stands (speeds, accelerations and
    # jerks as they are): the retimer refuses a start outside the box, and the joint only
    # moves inward from there
    box = replace(lim, q_min=np.minimum(lo, q - 1e-3), q_max=np.maximum(hi, q + 1e-3))
    res = retime_detailed(JointPath(path), box, st.rules_for(a))
    if not hasattr(res, "traj"):
        return f"the joint retreat cannot be timed: {res.reason} {res.detail}"
    return Motion("retreat", res.traj)


def _one(st, a, now):
    """Arm a's retreat from where everything stands: a Step (with why when refused), or None
    when it needs none."""
    from aris.server.steps import Scene, Step
    rig = st.rig
    obs, standing, phase = Scene(rig).of(a, now, (a,), (), f"retreat {a}")
    motions, q = [], now[a]
    joints = joint_retreat(st, a, q)
    if isinstance(joints, str):
        return Step(a, phase, why=f"retreat from the joint limits: {joints}")
    if joints is not None:
        motions.append(joints)
        q = joints.q_end
    b, g = nearest(rig, a, {**now, a: q})
    if b is not None and g < clearance(rig):
        m = plan_retreat(st, a, {**now, a: q})
        if isinstance(m, str):
            return Step(a, phase, why=f"retreat from {b}: {m}")
        motions.append(m)
    if not motions:
        return None
    verdicts, qb = [], now[a]
    for m in motions:
        v = check(st.config_dir, a, m, phase, qb, standing=standing)
        if not v.passed:
            return Step(a, phase, why="retreat: checker: " + ", ".join(v.failed))
        verdicts.append(v)
        qb = m.q_end
    return Step(a, phase, tuple(motions), tuple(verdicts), dict(standing))


def retreats(st, now) -> tuple[list, dict]:
    """Steps "retreat <slot>" for every arm standing with a joint inside the gates' limit
    margin (the joint move first) or within the clearance of another arm (then the
    up-and-away), and where everything stands after them.  Rig order, in passes: an arm whose
    retreat is refused is tried again after the others have moved (the one that can get away
    goes first); one still refused when nobody can move is a Step with why."""
    rig = st.rig
    now = {a: np.asarray(q, float) for a, q in now.items()}
    steps, pending = [], list(rig.arm_ids)
    while pending:
        refused, moved = {}, False
        for a in list(pending):
            got = _one(st, a, now)
            if got is None:
                pending.remove(a)
            elif got.why:
                refused[a] = got
            else:
                steps.append(got)
                now[a] = got.motions[-1].q_end
                pending.remove(a)
                moved = True
        if not moved:
            steps += [refused[a] for a in pending]
            break
    return steps, now


def too_close(st, now) -> str:
    """Why a drawing must not start: two arms standing within the clearance ("")."""
    rig = st.rig
    for i, a in enumerate(rig.arm_ids):
        for b in rig.arm_ids[i + 1:]:
            g = gap(rig, a, now[a], b, now[b])
            if g < clearance(rig):
                return (f"arms {a} and {b} stand {g * 1e3:.0f} mm apart (the clearance is "
                        f"{clearance(rig) * 1e3:.0f} mm): run `aris park` first (it moves them "
                        "apart)")
    return ""
