"""`check_phase_end`: can the next phase start from where every arm stands now?

Everything stands still, so this is a check of one configuration per arm: every pair of arms
against each other (whole bodies, bases included) at the arm-to-arm clearance, and every arm
against the steel, the paper and itself at the demanded clearances, except an arm the phase did
not move and that stands away from its park (`standing`): it is accepted where it stands.  No walls: the walls keep
moving arms apart, and here the pairs are measured directly.
"""
from __future__ import annotations

import numpy as np

from aris.check import geometry as geo
from aris.check.config import read_rig
from aris.check.model import capsules
from aris.check.scene import build_scene, clearance, model_of
from aris.check.verdict import Verdict, measure, verdict
from aris.types import Phase

_TITLE = dict(steel="steel", links="paper (links)", tool="paper (tool)", pen="paper (pen)",
              self="self")


def check_phase_end(config_dir, phase: Phase, q_by_arm: dict, standing=None) -> Verdict:
    """`q_by_arm`: slot ("2R") -> (7,) where it stands.  A parked arm left out stands at its park
    configuration; an active arm left out is a failure.

    `standing` ({slot: (7,)} or slots): arms the phase did not move and that do not stand at
    their park (a pen still down while another arm's pen is lifted, an arm stopped mid-drawing).
    Such an arm is accepted wherever it stands: it is where it was, and every motion of the
    phase was checked against that pose.  It still counts in every pair; the rules for one arm
    (steel, paper, itself) apply to the others.  Its configuration is taken from `q_by_arm`,
    else from `standing`."""
    standing = standing if isinstance(standing, dict) else dict.fromkeys(standing or ())
    moved = [a for a in standing if a in phase.active]
    if moved:
        return _refuse(f"slots {moved} move in {phase.name}: they cannot be standing")
    try:
        rig = read_rig(config_dir)
    except (OSError, ValueError, KeyError) as e:
        return _refuse(f"cannot read the rig: {e}")
    q = {}
    for a, m in rig.mounts.items():
        if a in q_by_arm:
            q[a] = np.asarray(q_by_arm[a], float)
        elif standing.get(a) is not None:
            q[a] = np.asarray(standing[a], float)
        elif a in phase.active:
            return _refuse(f"active slot {a} has no configuration")
        else:
            q[a] = m.park_q
        if q[a].shape != (7,) or not np.all(np.isfinite(q[a])):
            return _refuse(f"slot {a}: the configuration is not 7 numbers")
    unknown = [a for a in (*q_by_arm, *standing) if a not in rig.mounts]
    if unknown:
        return _refuse(f"slots {unknown} have no arm on this rig")

    ms = [measure("well formed", 1.0, 1.0, "min", "", ranked=False)]
    ms += _pairs(rig, q)
    for a in rig.mounts:
        if a not in standing:
            ms.append(_alone(rig, a, q[a]))
    worst = min((m for m in ms[1:]), key=lambda m: m.value - m.limit)
    notes = rig.notes + sum((m.notes for m in rig.mounts.values()), ())
    return verdict(ms, worst.value - worst.limit, f"{worst.name}: {worst.detail}", notes)


def _refuse(why) -> Verdict:
    return verdict([measure("well formed", 0.0, 1.0, "min", "", why, ranked=False)])


def _pairs(rig, q):
    ids = list(rig.mounts)
    bodies = {}
    for a in ids:
        m = model_of(rig, a)
        A, B = capsules(m, q[a][None], rig.mounts[a].T_table_base)
        bodies[a] = (A[0], B[0], m.radius, m.names)
    out = []
    margin = rig.clearance["arm_to_arm_m"]
    for i, a in enumerate(ids):
        for b in ids[i + 1:]:
            A1, B1, r1, n1 = bodies[a]
            A2, B2, r2, n2 = bodies[b]
            d = (geo.segment_segment(A1[:, None], B1[:, None], A2[None], B2[None])
                 - r1[:, None] - r2[None])
            k, j = np.unravel_index(np.argmin(d), d.shape)
            out.append(measure(f"arms {a} and {b}", d[k, j], margin, "min", "m",
                               f"{n1[k]} / {n2[j]}"))
    return out


def _alone(rig, a, q):
    """One arm against steel, paper and itself: the tightest of those, beyond its margin."""
    scene = build_scene(rig, a, (), (), drawing=False)
    cl = clearance(scene, q[None])
    c = min(("steel", "links", "tool", "pen", "self"), key=lambda c: cl.value[c][0])
    return measure(f"arm {a}: {_TITLE[c]}", cl.value[c][0] + scene.margin[c], scene.margin[c],
                   "min", "m", cl.closest(c, 0))
