"""Retreat: an arm standing closer to another arm than the rig's arm-to-arm clearance (an
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

import numpy as np

from aris.check import check
from aris.kernel.geometry import segment_segment_distance
from aris.kernel.retime import retime_detailed
from aris.types import JointPath, Motion

RISE = 0.030             # m straight up first
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
    """Joints along the table-frame tip path `tips`, the hand's orientation kept."""
    arm = rig.arm(a)
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
    res = retime_detailed(JointPath(path), rig.arm(a).limits, st.rules, smooth=True,
                          tip_of=rig.arm(a).tip)
    if not hasattr(res, "traj"):
        return f"the retreat cannot be timed: {res.reason} {res.detail}"
    return Motion("retreat", res.traj)


def retreats(st, now) -> tuple[list, dict]:
    """Steps "retreat <slot>" for every arm, in rig order, standing within the clearance of
    another, and where everything stands after them.  A refused retreat is a Step with why."""
    from aris.server.steps import Scene, Step
    rig = st.rig
    now = {a: np.asarray(q, float) for a, q in now.items()}
    steps = []
    for a in rig.arm_ids:
        b, g = nearest(rig, a, now)
        if b is None or g >= clearance(rig):
            continue
        obs, standing, phase = Scene(rig).of(a, now, (a,), (), f"retreat {a}")
        m = plan_retreat(st, a, now)
        if isinstance(m, str):
            steps.append(Step(a, phase, why=f"retreat from {b}: {m}"))
            continue
        v = check(st.config_dir, a, m, phase, now[a], standing=standing)
        if not v.passed:
            steps.append(Step(a, phase, why="retreat: checker: " + ", ".join(v.failed)))
            continue
        steps.append(Step(a, phase, (m,), (v,), dict(standing)))
        now[a] = m.q_end
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
