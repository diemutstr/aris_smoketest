"""How far the arm's capsules can move along a straight joint-space move, and what that says
about clearance between two measured configurations.

Clearance is 1-Lipschitz in the displacement of a capsule's axis segment: if no point of the
capsule has moved further than d from where it was at an end, the clearance is at most d lower
than at that end.  So on a straight move from qa to qb (fraction s in [0, 1]),

    clearance(s) >= max(ca - da * s, cb - db * (1 - s))

where da bounds how far a point can get from its place at qa per unit of s, and db likewise from
qb.  Two bounds on da:

- zero order (from the arm's `reach` table): joint j moves any point of capsule k at most
  reach[j, k] |dq_j|, so da <= sum_j reach[j, k] |dq_j|.  Loose, because it adds the joints'
  contributions as if they all pointed the same way.
- first order: at qa each point moves with velocity v = sum_j dq_j z_j x (p - o_j) (the joints'
  axes o_j, z_j at qa).  Along the move the velocity changes by at most A per unit of s, so the
  point travels at most |v| s + A s^2 / 2 <= (|v| + A / 2) s.  A comes from the reach table
  (`accel_bound`).  The velocity is largest at one of the capsule's two segment ends, because
  it is affine along the segment.

The smaller of the two is used.  On short moves the first order is 2 to 3 times tighter.

When a move is halved, both halves point the same way as the whole: the velocity at any point
of it, per unit of the half's own s, is half the whole's, A a quarter, the zero order half.  So
each measured configuration needs only its speed along the move's direction (`speeds`).
"""
from __future__ import annotations

import numpy as np

BIG = 1e6        # stands in for "nothing to hit" (a fixed capsule reads +inf)


def interval_bound(ca, cb, da, db):
    """Lowest value of max(ca - da s, cb - db (1 - s)) over s in [0, 1]."""
    ca, cb = np.minimum(ca, BIG), np.minimum(cb, BIG)
    tot = da + db
    safe = np.where(tot > 0.0, tot, 1.0)
    s = np.where(tot > 0.0, np.clip((ca - cb + db) / safe, 0.0, 1.0), 0.5)
    return np.maximum(ca - da * s, cb - db * (1.0 - s))


def self_pair_weights(reach: np.ndarray, pairs: np.ndarray) -> np.ndarray:
    """(P, 7): per self pair, how far joint j can move one capsule relative to the other per
    radian.  A joint that moves both turns them rigidly together and is not charged."""
    moves = reach > 0.0                                     # (7, K)
    i, j = pairs[:, 0], pairs[:, 1]
    only_i = moves[:, i] & ~moves[:, j]
    only_j = moves[:, j] & ~moves[:, i]
    return (np.where(only_i, reach[:, i], 0.0) + np.where(only_j, reach[:, j], 0.0)).T


def speeds(frames: np.ndarray, p0: np.ndarray, p1: np.ndarray, n_moving: np.ndarray,
           U: np.ndarray) -> np.ndarray:
    """(N,K): the larger speed of each capsule's two segment ends when the joints move with
    rates U (N,7).

    frames (N,10,4,4) from `Arm.link_frames` (joint j turns frame j+1 about its z axis);
    n_moving (K,) how many joints, from the base, move capsule k.  The velocity of a point p
    moved by joints 0..m-1 is w_m x p + c_m, with w_m = sum_{j<m} U_j z_j and
    c_m = -sum_{j<m} U_j z_j x o_j (a twist, summed along the chain)."""
    o = frames[:, 1:8, :3, 3]
    z = frames[:, 1:8, :3, 2]
    uz = U[:, :, None] * z                                   # (N,7,3)
    zero = np.zeros((len(U), 1, 3))
    w = np.concatenate([zero, np.cumsum(uz, axis=1)], axis=1)                   # (N,8,3)
    c = np.concatenate([zero, np.cumsum(-np.cross(uz, o), axis=1)], axis=1)     # (N,8,3)
    wk, ck = w[:, n_moving], c[:, n_moving]                 # (N,K,3)
    v0 = np.linalg.norm(np.cross(wk, p0) + ck, axis=-1)
    v1 = np.linalg.norm(np.cross(wk, p1) + ck, axis=-1)
    return np.maximum(v0, v1)


def accel_bound(dq_abs: np.ndarray, reach: np.ndarray) -> np.ndarray:
    """(E,K): how fast a point's velocity (per unit of s) can change along the move.

    The velocity is sum_j dq_j w_j with w_j = z_j x (p - o_j), |w_j| <= reach[j].  Turning the
    joints before j carries axis j and the point round together, which turns w_j at rate
    sum_{i<j} |dq_i|; joints j and after move the point relative to axis j by at most
    sum_{i>=j} reach[i] |dq_i|.  So |dw_j/ds| <= (sum_{i<j}|dq_i|) reach[j] + sum_{i>=j}
    reach[i] |dq_i|, and the acceleration is at most sum_j |dq_j| |dw_j/ds|."""
    w = dq_abs[:, :, None] * reach[None]                     # (E,7,K)
    from_j = np.cumsum(w[:, ::-1], axis=1)[:, ::-1]          # sum over i >= j
    before = (np.cumsum(dq_abs, axis=1) - dq_abs)[:, :, None]  # sum over i < j
    return np.einsum("ij,ijk->ik", dq_abs, before * reach[None] + from_j)


def n_moving(reach: np.ndarray) -> np.ndarray:
    """(K,): how many joints, counted from the base, move each capsule (reach > 0 marks them;
    along a serial chain they are always the first few)."""
    moves = reach > 0.0
    m = moves.sum(axis=0)
    if not np.all(moves == (np.arange(7)[:, None] < m[None])):
        raise ValueError("reach does not describe a serial chain")
    return m
