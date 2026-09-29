"""Groups for the collision check's first pass: skip whole groups of pairs that cannot matter.

Part of `aris.kernel.collide`.  Grouping only ever saves work; the answers do not depend on it
(see `collide.py`).

Body groups: capsules whose names share the part before the first "." ("link3.0", "link3.1"
-> "link3"; "hand.*", "holder.*", "pen"): one group per link and per part of the tool.  Each
configuration bounds a group by a sphere around its capsules.

Obstacle groups (`pack`): the planes are never grouped.  Capsules named "<owner>:<part>.<n>"
(a parked arm, "parked71:link3.0") are grouped by owner and part, so a parked arm becomes a
dozen groups; other capsules by a 0.5 m grid cell of their midpoint.  Boxes are gathered
greedily: a box joins the first group whose first box's centre is within 0.4 m of its own.
Every obstacle group carries one fat capsule that contains all its members (the coarse stand-in
used for the far test) and the largest margin any member demands.
"""
from __future__ import annotations

import numpy as np

BOX_CLUSTER = 0.4       # m, centre distance under which boxes share a group
CAP_CELL = 0.5          # m, grid cell for capsules that name no owner


def enclosing_capsule(points: np.ndarray, radii: np.ndarray):
    """A capsule (a, b, r) containing every ball (points[i], radii[i]), hence their hull.

    The axis is the points' main direction; the segment spans their projections; the radius is
    the largest distance from the axis plus that ball's radius.  A ball's centre projects onto
    the segment, so the ball lies within the radius of that point of the segment.
    """
    P = np.asarray(points, float).reshape(-1, 3)
    r = np.asarray(radii, float).reshape(-1)
    c = P.mean(axis=0)
    if len(P) > 1 and np.ptp(P, axis=0).max() > 0.0:
        u = np.linalg.svd(P - c, full_matrices=False)[2][0]
    else:
        u = np.array([1.0, 0.0, 0.0])
    t = (P - c) @ u
    off = np.linalg.norm(P - c - t[:, None] * u, axis=1)
    # a hair of slack so rounding can never leave a member poking out
    R = float(np.max(off + r)) * (1 + 1e-12) + 1e-12
    return c + t.min() * u, c + t.max() * u, R


def body_groups(names) -> tuple[np.ndarray, np.ndarray]:
    """(start (G+1,), members (K,)) grouping capsule indices by name part before the first "."."""
    keys: dict[str, list[int]] = {}
    for k, n in enumerate(names):
        keys.setdefault(str(n).split(".")[0], []).append(k)
    members = [k for ks in keys.values() for k in ks]
    start = np.cumsum([0] + [len(ks) for ks in keys.values()])
    return np.asarray(start, np.int64), np.asarray(members, np.int64)


def obstacle_groups(box_R, box_c, box_h, cap_a, cap_b, cap_r, cap_names, Mp):
    """-> og_start (G+1,), og_members (Mb+Mc,) global obstacle indices, og_a, og_b (G,3), og_r (G,)."""
    Mb, Mc = len(box_c), len(cap_a)
    groups: list[list[int]] = []
    firsts: list[np.ndarray] = []
    for o in range(Mb):
        for g, c0 in zip(groups, firsts):
            if np.linalg.norm(box_c[o] - c0) <= BOX_CLUSTER:
                g.append(o)
                break
        else:
            groups.append([o])
            firsts.append(box_c[o])
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float)
    coarse = []
    for g in groups:
        pts = np.concatenate([box_c[o] + (corners * box_h[o]) @ box_R[o].T for o in g])
        coarse.append(enclosing_capsule(pts, np.zeros(len(pts))))
    keyed: dict[object, list[int]] = {}
    for m in range(Mc):
        name = str(cap_names[m])
        if ":" in name:
            owner, part = name.split(":", 1)
            key = (owner, part.split(".")[0])
        else:
            key = ("cell",) + tuple(np.floor(0.5 * (cap_a[m] + cap_b[m]) / CAP_CELL).astype(int))
        keyed.setdefault(key, []).append(m)
    for ms in keyed.values():
        groups.append([Mb + Mp + m for m in ms])
        pts = np.concatenate([cap_a[ms], cap_b[ms]])
        coarse.append(enclosing_capsule(pts, np.concatenate([cap_r[ms], cap_r[ms]])))
    start = np.cumsum([0] + [len(g) for g in groups]).astype(np.int64)
    members = np.array([o for g in groups for o in g], np.int64)
    a = np.array([c[0] for c in coarse], float).reshape(-1, 3)
    b = np.array([c[1] for c in coarse], float).reshape(-1, 3)
    r = np.array([c[2] for c in coarse], float).reshape(-1)
    return start, members, a, b, r


def group_margins(start, members, box_m, cap_margin, Mb, Mp) -> np.ndarray:
    """(G,) the largest margin any member of each group demands."""
    marg = np.concatenate([box_m, np.zeros(Mp), cap_margin])
    if len(start) < 2:
        return np.zeros(0)
    return np.maximum.reduceat(marg[members], start[:-1]).astype(float)
