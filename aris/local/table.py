"""The kinematic table: the graph's nodes looked up instead of solved (optimisation note 1).

The arm is symmetric about its base axis.  For a paper square to that axis at a given height,
turning the pen tip about the axis by an angle only adds that angle to joint 1 and to the spin
(measured from the direction pointing away from the axis the spin is unchanged); joints 2 to 7,
the IK slot, the singular value, joints 2-7's distance to their limits, the arm's clearance to
itself and its clearance to the paper do not change at all.  So one table, indexed by

    distance of the tip from the axis (every `table_dr`), spin relative to the outward direction
    (the graph's spin grid), lean, elbow value (the graph's grids) and IK slot,

holds everything the graph needs except joint 1's own limit and the obstacles.  The graph
counts its spins the same way, so a node of the graph is a lookup: the two entries at the
tabulated distances around the tip's are blended (joint angles, singular value, margins and
clearances alike).  Joint 1 is the
blended offset plus the tip's angle about the axis; its limit and the obstacles are checked
live.  (Towards the edge of what the arm can do one of the two entries may be missing; the
other then stands for the node if it is the nearer.)  The table only guides the search: every piece is still solved exactly and verified on
the arm's true paper plane.

The table is built once per (tool, limits, paper height, paper margins, gates, grids),
saved as two .npy files in a cache directory the caller names, with a digest of every input
in the file name, and read back memory-mapped.
"""
from __future__ import annotations

import hashlib
import os
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from aris.kernel import collide as C
from aris.local.gates import Judge
from aris.local.settings import Settings
from aris.types import Gates, Obstacles, Plane

VERSION = 1
N_SLOT = 8                                # IK answers per pose
NORMAL = np.array([0.0, 0.0, -1.0])      # the paper normal in the base frame, pointing at the arm
G_SIGMA, G_LIMIT, G_SELF, G_PAPER = range(4)


@dataclass(frozen=True)
class Grids:
    r: np.ndarray            # (R,) tip distance from the base axis
    spin: np.ndarray         # (S,) spin relative to the outward direction
    lean: np.ndarray         # (L, 2)
    q7: np.ndarray           # (Q,)


@dataclass(frozen=True)
class KinTable:
    grids: Grids
    height: float            # m, the paper below the base, along the base axis
    joints: np.ndarray       # (R, S, L, Q, 8, 7) joint 1 offset (joint 1 minus the tip's angle), joints 2-7; NaN: none
    gates: np.ndarray        # (R, S, L, Q, 8, 4) singular value, limit margin of joints 2-7, self and paper clearance
    path: Path


def grids(arm, lean_max: float, cfg: Settings, limit_margin: float) -> Grids:
    from aris.local.lattice import lean_set          # the graph's own grids, exactly
    q7_top = arm.limits.q_max[6] - limit_margin
    q7_low = arm.limits.q_min[6] + limit_margin
    k = np.arange(np.ceil(q7_low / cfg.q7_step - 1e-9), np.floor(q7_top / cfg.q7_step + 1e-9) + 1)
    n_r = int(np.floor(cfg.table_r_max / cfg.table_dr + 1e-9)) + 1
    return Grids(r=np.arange(n_r) * cfg.table_dr,
                 spin=np.arange(cfg.n_spin) * (2.0 * np.pi / cfg.n_spin),
                 lean=lean_set(lean_max, cfg.lean_rings, cfg.lean_dirs), q7=k * cfg.q7_step)


def paper_height(paper: Plane) -> float:
    """Where the paper crosses the base axis, below the base (the table's height)."""
    n = np.asarray(paper.normal, float) / np.linalg.norm(paper.normal)
    return float(paper.offset / n[2])


def _digest(arm, paper: Plane, gates: Gates, g: Grids, cfg: Settings) -> str:
    h = hashlib.blake2b(digest_size=12)
    tool = arm.tool
    parts = [np.array([VERSION, paper_height(paper), paper.margin,
                       -1.0 if paper.pen_margin is None else paper.pen_margin,
                       -1.0 if paper.tool_margin is None else paper.tool_margin,
                       gates.limit_margin, gates.sigma_min, gates.self_margin], float),
             tool.tip_hand, tool.pen_axis_hand, arm.limits.q_min, arm.limits.q_max,
             g.r, g.spin, g.lean, g.q7]
    parts += [np.concatenate([c.p0, c.p1, [c.radius]]) for c in tool.capsules_hand]
    for p in parts:
        h.update(np.ascontiguousarray(p, float).tobytes())
    h.update(repr((tool.pen_names, tuple(c.name for c in tool.capsules_hand),
                   arm.body(np.zeros((1, 7))).is_tool is not None)).encode())
    return h.hexdigest()


def load_or_build(arm, paper: Plane, gates: Gates, lean_max: float, cfg: Settings,
                  cache_dir) -> KinTable:
    """The table for this arm, tool and paper height, from `cache_dir` if it is there."""
    g = grids(arm, lean_max, cfg, gates.limit_margin)
    stem = Path(cache_dir) / f"local_table_{_digest(arm, paper, gates, g, cfg)}"
    jf, gf = stem.with_suffix(".joints.npy"), stem.with_suffix(".gates.npy")
    if not (jf.exists() and gf.exists()):
        joints, gate = _build(arm, paper, gates, g)
        Path(cache_dir).mkdir(parents=True, exist_ok=True)
        for f, a in ((jf, joints), (gf, gate)):       # written whole, then renamed: never half a file
            tmp = f.with_suffix(f".{os.getpid()}.tmp.npy")
            np.save(tmp, a)
            os.replace(tmp, f)
    return KinTable(g, paper_height(paper), np.load(jf, mmap_mode="r"),
                    np.load(gf, mmap_mode="r"), stem)


def _build(arm, paper: Plane, gates: Gates, g: Grids):
    """Solve every entry with the tip on the +x side of the axis (the tip's angle is zero)."""
    h = paper_height(paper)
    flat = Plane("paper", NORMAL, -h, paper.margin, "paper", paper.pen_margin, paper.tool_margin)
    judge = Judge(arm, Obstacles(planes=(flat,)), gates)
    shape = (len(g.r), len(g.spin), len(g.lean), len(g.q7), N_SLOT)
    joints = np.full(shape + (7,), np.nan, np.float32)
    gate = np.full(shape + (4,), np.nan, np.float16)
    S, L, Q = np.meshgrid(np.arange(len(g.spin)), np.arange(len(g.lean)), np.arange(len(g.q7)),
                          indexing="ij")
    S, L, Q = S.ravel(), L.ravel(), Q.ravel()
    for i, r in enumerate(g.r):
        tip = np.broadcast_to(np.array([r, 0.0, h]), (len(S), 3))
        Qs, ok = arm.ik(arm.hand_pose(tip, NORMAL, g.spin[S], g.lean[L]), g.q7[Q])
        m, b = np.nonzero(ok)
        if not len(m):
            continue
        q = Qs[m, b]
        vals = np.column_stack([arm.sigma_min(q), _margin_2to7(arm, q),
                                C.self_clearance_q(judge.tables, q, arm.self_pairs, 0.0),
                                judge.paper_clear(q)])
        joints[i, S[m], L[m], Q[m], b] = q.astype(np.float32)
        gate[i, S[m], L[m], Q[m], b] = np.clip(vals, -60000, 60000).astype(np.float16)
    return joints, gate


def _margin_2to7(arm, q):
    lim = arm.limits
    return np.min(np.minimum(q[:, 1:] - lim.q_min[1:], lim.q_max[1:] - q[:, 1:]), axis=1)


def lookup(table: KinTable, arm, tip: np.ndarray, lean_idx, gates: Gates):
    """Nodes of one layer from the table.

    tip (3,) the pen tip in the base frame; lean_idx the leans wanted.  The graph's spins are
    the table's (counted from the outward direction), so only the tip's distance from the axis
    is blended, between the two tabulated distances around it.  -> (spin idx, lean idx, q7 idx,
    slot, q (n,7), answers) for every entry inside the gates that do not depend on obstacles
    (joint 1's limit checked here, live), or None beyond the table.
    """
    g = table.grids
    theta = float(np.arctan2(tip[1], tip[0]))
    x = float(np.hypot(tip[0], tip[1])) / (g.r[1] - g.r[0])
    i0 = int(np.floor(x))
    if i0 + 1 >= len(g.r):
        return None
    u = x - i0
    lean_idx = np.asarray(lean_idx)
    Js = [np.asarray(table.joints[i][:, lean_idx], float) for i in (i0, i0 + 1)]
    G = [np.asarray(table.gates[i][:, lean_idx], float) for i in (i0, i0 + 1)]
    have = [np.all(np.isfinite(a), axis=-1) for a in Js]                   # 2 x (S, L, Q, 8)
    # Where only one of the two distances has an entry (towards the edge of what the arm can
    # do), it stands for the node if it is the nearer one.
    wts = [np.where(have[0], 1.0 - u, 0.0), np.where(have[1], u, 0.0)]
    total = wts[0] + wts[1]
    valid = total >= 0.5
    ref = np.where(have[0], Js[0][..., 0], Js[1][..., 0])
    q = np.zeros(total.shape + (7,))
    gate = np.zeros(total.shape + (4,))
    for wt, a, g_ in zip(wts, Js, G):
        a = np.nan_to_num(a)
        a[..., 0] = ref + np.mod(a[..., 0] - ref + np.pi, 2 * np.pi) - np.pi   # never across the wrap
        q += wt[..., None] * a
        gate += wt[..., None] * np.nan_to_num(g_)
    with np.errstate(invalid="ignore", divide="ignore"):
        q /= total[..., None]
        gate /= total[..., None]
    q[..., 0] = np.mod(q[..., 0] + theta + np.pi, 2.0 * np.pi) - np.pi
    lim = arm.limits
    m1 = np.minimum(q[..., 0] - lim.q_min[0], lim.q_max[0] - q[..., 0])
    with np.errstate(invalid="ignore"):
        ok = (valid & (np.minimum(gate[..., G_LIMIT], m1) >= gates.limit_margin)
              & (gate[..., G_SIGMA] >= gates.sigma_min)
              & (gate[..., G_SELF] >= gates.self_margin) & (gate[..., G_PAPER] >= 0.0))
    si, li, qi, b = np.nonzero(ok)
    return si, lean_idx[li], qi, b, q[si, li, qi, b], int(valid.sum())
