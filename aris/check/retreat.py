"""A retreat: flown from a pose already inside the arm-to-arm clearance (after a pen-tip meeting
that was interrupted), so the fixed clearance to the other arms cannot hold at its start.  It is
judged instead by its direction: the distance to every other arm (parked, or standing where the
caller says) never decreases along the flown motion, between the samples included, and at the
end it is back at the demanded clearance, or the motion is short (a few centimetres up and away
from a meeting, not a journey).

The same for a joint that starts closer to its limit than the planners' gate (or past the limit
itself, by at most `PAST_LIMIT`: the arm is physically there): its distance from that limit
never decreases and ends at the gate.  Every other joint stays inside its limits as usual.
"""
from __future__ import annotations

from dataclasses import replace

import numpy as np

from aris.check import timing
from aris.check.model import pose, tip
from aris.check.scene import Scene, clearance_of
from aris.check.sweep import _class_travel, _times, _travel
from aris.check.verdict import measure

APPROACH_TOL = 1e-3   # m, how much the distance to another arm may fall back (sampling, bound)
SHORT = 0.25          # m of tip travel: a retreat this short need not end at the clearance
LIMIT_BACK_TOL = 1e-5 # rad, how much a recovering joint may fall back toward its limit
PAST_LIMIT = 0.10     # rad, how far past its limit a joint may start a retreat


def retreat_rows(scene: Scene, traj, step: float) -> list:
    """One row per other arm (how far the motion ever approaches it), and one for the end."""
    tr = timing.travel(traj.t, traj.q, traj.qd)
    at = lambda t: timing.hermite(traj.t, traj.q, traj.qd, t)[0]
    t = _times(scene, tr, at, step)
    Q = at(t)
    A, B, D = pose(scene.model, Q, scene.T_table_base)
    T = _class_travel(scene, _travel(tr.between(t[:-1], t[1:]), D[:-1], D[1:],
                                     scene.model.travel))["parked"]
    margin = scene.margin["parked"]
    p = tip(scene.model, Q, scene.T_table_base)
    length = float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())
    rows, ends = [], []
    for other in dict.fromkeys(n.split(":")[0] for n in scene.other_names):
        own = np.array([n.split(":")[0] == other for n in scene.other_names])
        alone = replace(scene, other_names=tuple(np.array(scene.other_names)[own]),
                        other_a=scene.other_a[own], other_b=scene.other_b[own],
                        other_r=scene.other_r[own])
        d = clearance_of(alone, A, B).value["parked"] + margin              # gap, per sample
        low = timing.interval_bound(d[:-1], d[1:], T)                      # between samples
        before = np.maximum.accumulate(d)[:-1]                             # best so far
        back = np.maximum(before - low, 0.0)
        k = int(np.argmax(back))
        slot = other.removeprefix("parked").removeprefix("standing")
        rows.append(measure(f"retreat approaches {slot}", back[k], APPROACH_TOL, "max", "m",
                            f"gap {d[0] * 1e3:.1f} mm at the start, {d[-1] * 1e3:.1f} mm at the "
                            f"end; falls back near t = {t[k]:.3f} s"))
        ends.append((d[-1], slot))
    if not ends:
        return rows
    if length <= SHORT:
        rows.append(measure("retreat length", length, SHORT, "max", "m",
                            "tip travel: short enough not to have to end clear"))
    else:
        gap, slot = min(ends)
        rows.append(measure("retreat ends clear", gap, margin, "min", "m",
                            f"to {slot}; tip travel {length * 1e3:.0f} mm"))
    return rows


def _margin(model, Q):
    """(N,7) distance of each joint from its nearer limit, negative past it."""
    return np.minimum(Q - model.q_min, model.q_max - Q)


def recovering(model, q0, gate) -> np.ndarray:
    """(7,) bool: the joints that start closer to a limit than the gate."""
    return _margin(model, np.asarray(q0, float)[None])[0] < gate


def limit_rows(model, Q, joints, gate) -> list:
    """For each recovering joint, on the driver's samples `Q` (N,7): how far past the limit it
    starts, how far it ever falls back toward it, and where it ends against the gate."""
    rows = []
    M = _margin(model, Q)
    for j in np.flatnonzero(joints):
        m = M[:, j]
        back = float(np.max(np.maximum.accumulate(m) - m))
        name = f"limit of joint {j + 1}"
        rows += [measure(f"retreat starts near the {name}", -m[0], PAST_LIMIT, "max", "rad",
                         f"{m[0]:+.4f} rad from the limit at the start"),
                 measure(f"retreat approaches the {name}", back, LIMIT_BACK_TOL, "max", "rad",
                         "how far it ever falls back toward the limit"),
                 measure(f"retreat clears the {name}", m[-1], gate, "min", "rad",
                         "distance from the limit at the end, against the planners' gate")]
    return rows
