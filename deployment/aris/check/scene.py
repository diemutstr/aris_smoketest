"""What one arm must stay clear of in one phase, in the table frame, and how far it is.

Obstacle classes, each with the clearance `rig.json` demands (not the planning allowance):
  steel    every steel box of the rig, the arm's own struts, plate and clamp included
  paper    the paper plane, for every moving capsule except the pen
  pen      the pen against the paper (free motions only), at the lifted-pen clearance
  walls    the walls of the phase that have this arm on one side
  parked   the arms standing parked in this phase, at their park configurations
  self     the arm against itself
Capsules bolted to the base (link0) are not checked against obstacles, only against the arm
itself.  Values are the clearance beyond the demanded one ("gap minus margin"), per sample:
exact up to a threshold the caller gives, a true lower bound above it.  Far pairs are skipped
only when a cheap lower bound proves they cannot change the answer.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from aris.check import geometry as geo
from aris.check.config import RigData
from aris.check.model import ArmModel, capsules, load_model

CLASSES = ("steel", "paper", "pen", "walls", "parked", "self")
_BLOCK = 16                     # consecutive samples that share one ball per capsule


@dataclass(frozen=True)
class Scene:
    model: ArmModel
    T_table_base: np.ndarray
    drawing: bool
    margin: dict                # class -> demanded clearance, m
    box_names: tuple
    box_lo: np.ndarray          # (B,3)
    box_hi: np.ndarray
    plane_names: tuple          # walls
    plane_n: np.ndarray         # (W,3), pointing to this arm's side
    plane_d: np.ndarray         # (W,)
    paper_z: float
    other_names: tuple          # parked arms' capsules
    other_a: np.ndarray         # (C,3)
    other_b: np.ndarray
    other_r: np.ndarray


@dataclass(frozen=True)
class Clearance:
    """Per class: (N,) clearance beyond the demanded margin, and what is closest at each sample."""
    value: dict                 # class -> (N,)
    pair: dict                  # class -> (N,) int, the closest pair (-1 where nothing is checked)
    label: dict                 # class -> function: pair -> "capsule / obstacle"

    def closest(self, cls: str, n: int) -> str:
        p = int(self.pair[cls][n])
        return self.label[cls](p) if p >= 0 else ""


def build_scene(rig: RigData, arm_id: int, walls, parked, drawing: bool) -> Scene:
    mount = rig.mounts[arm_id]
    model = load_model(mount.tip_hand)
    base = mount.T_table_base[:3, 3]
    names, n, d = [], [], []
    for w in walls:
        if arm_id not in w.arms:
            continue
        normal = np.asarray(w.normal_table, float)
        normal = normal / np.linalg.norm(normal)
        side = normal @ (base - np.asarray(w.point_table, float))
        if side < 0:
            normal = -normal
        names.append(w.name), n.append(normal), d.append(normal @ np.asarray(w.point_table))
    o_names, o_a, o_b, o_r = [], [], [], []
    for p in parked:
        other = rig.mounts[p]
        m = load_model(other.tip_hand)
        a, b = capsules(m, other.park_q[None], other.T_table_base)
        o_names += [f"parked{p}:{x}" for x in m.names]
        o_a.append(a[0]), o_b.append(b[0]), o_r.append(m.radius)
    c = rig.clearance
    margin = dict(steel=c["steel_m"], paper=c["body_to_paper_m"],
                  pen=c["pen_lifted_to_paper_m"], walls=c["wall_m"],
                  parked=c["arm_to_arm_m"], self=c["self_m"])
    cat = lambda xs, k: np.concatenate(xs) if xs else np.zeros((0,) + k)
    return Scene(model, mount.T_table_base, drawing, margin, rig.box_names, rig.box_lo,
                 rig.box_hi, tuple(names), np.array(n, float).reshape(-1, 3),
                 np.array(d, float), rig.paper_z, tuple(o_names), cat(o_a, (3,)),
                 cat(o_b, (3,)), cat(o_r, ()))


def clearance(scene: Scene, Q, thr=None) -> Clearance:
    """Clearance of every class at configurations Q (N,7).

    `thr` (class -> metres, default: no limit): a value is computed exactly wherever it is at
    most thr[class]; above that the result may be a lower bound (still true, possibly lower
    than the exact value, never higher).  Pairs are skipped only when a cheap lower bound
    proves they cannot change the answer.
    """
    A, B = capsules(scene.model, Q, scene.T_table_base)
    return clearance_of(scene, A, B, thr)


def clearance_of(scene: Scene, A, B, thr=None) -> Clearance:
    """`clearance` for capsule end points already computed (N,K,3)."""
    thr = thr or {}
    m = scene.model
    names = np.array(m.names)
    move = ~m.is_fixed
    body, pen = move & ~m.is_pen, move & m.is_pen
    up, paper = np.array([[0.0, 0.0, 1.0]]), np.array([scene.paper_z])
    Am, Bm, rm = A[:, move], B[:, move], m.radius[move]
    t = lambda c: thr.get(c, np.inf)
    out = {
        "steel": _boxes(scene, Am, Bm, rm, names[move], t("steel")),
        "paper": _plane(A[:, body], B[:, body], m.radius[body], names[body], up, paper,
                        ("paper",), scene.margin["paper"]),
        "pen": (_nothing(len(A)) if scene.drawing else
                _plane(A[:, pen], B[:, pen], m.radius[pen], names[pen], up, paper, ("paper",),
                       scene.margin["pen"])),
        "walls": _plane(Am, Bm, rm, names[move], scene.plane_n, scene.plane_d,
                        scene.plane_names, scene.margin["walls"]),
        "parked": _parked(scene, Am, Bm, rm, names[move], t("parked")),
        "self": _self(A, B, m, scene.margin["self"], t("self")),
    }
    return Clearance({c: v[0] for c, v in out.items()}, {c: v[1] for c, v in out.items()},
                     {c: v[2] for c, v in out.items()})


# --------------------------------------------------------------------------- per class
# Each returns (value (N,), closest pair (N,) or -1, label: pair -> "capsule / obstacle").
# Values are gap minus margin.


def _nothing(n):
    return np.full(n, np.inf), np.full(n, -1), str


def _plane(A, B, r, names, normals, offsets, plane_names, margin):
    N, K = A.shape[:2]
    W = len(normals)
    if K == 0 or W == 0:
        return _nothing(N)
    h = geo.segment_plane_height(A[:, :, None, :], B[:, :, None, :], normals, offsets)
    flat = (h - r[None, :, None] - margin).reshape(N, -1)                 # (N, K*W)
    i = np.argmin(flat, axis=1)
    return flat[np.arange(N), i], i, lambda p: f"{names[p // W]} / {plane_names[p % W]}"


def _segment_min(N, n_i, x):
    """Per sample, the smallest x over the entries of that sample; n_i must be sorted."""
    out = np.full(N, np.inf)
    if len(n_i):
        starts = np.flatnonzero(np.r_[True, n_i[1:] != n_i[:-1]])
        out[n_i[starts]] = np.minimum.reduceat(x, starts)
    return out


def _gather(N, n_i, lb, ub, exact, thr, floor):
    """Smallest value per sample over the listed pairs (n_i sorted, lb, ub): exact for every pair
    whose lower bound is at most min(thr, that sample's best upper bound), the lower bound for
    the rest; `floor` (N,) is a lower bound for everything not listed.  -> value, pair index."""
    best_ub = _segment_min(N, n_i, ub)
    cand = lb <= np.minimum(thr, best_ub[n_i])
    val = np.minimum(floor, _segment_min(N, n_i, np.where(cand, np.inf, lb)))
    ci = np.flatnonzero(cand)
    pair = np.full(N, -1)
    if len(ci):
        ex = exact(ci)
        order = np.lexsort((ex, n_i[ci]))
        first = order[np.unique(n_i[ci][order], return_index=True)[1]]
        nn = n_i[ci][first]
        better = ex[first] <= val[nn]
        val[nn[better]] = ex[first][better]
        pair[nn[better]] = ci[first][better]
    return val, pair


def _block_balls(mid, rad):
    """Over each block of `_BLOCK` consecutive samples, one ball per capsule holding every
    position it takes in that block: centres (nb,K,3), radii (nb,K)."""
    N, K = mid.shape[:2]
    G = _BLOCK
    nb = -(-N // G)
    pad = nb * G - N                        # repeat the last sample to fill the last block
    midb = np.concatenate([mid, np.repeat(mid[-1:], pad, 0)]).reshape(nb, G, K, 3)
    radb = np.concatenate([rad, np.repeat(rad[-1:], pad, 0)]).reshape(nb, G, K)
    cen = midb.mean(axis=1)
    return cen, (np.linalg.norm(midb - cen[:, None], axis=-1) + radb).max(axis=1)


def _expand(keep, lb_block, N):
    """Blocks x pairs kept -> (sample, pair) lists sorted by sample, and per sample the lowest
    block bound of the pairs not kept."""
    G = _BLOCK
    floor = np.repeat(np.where(keep, np.inf, lb_block).min(axis=1, initial=np.inf), G)[:N]
    b1, p1 = np.nonzero(keep)
    n_i = (b1[:, None] * G + np.arange(G)).ravel()
    p_i = np.repeat(p1, G)
    ok = n_i < N
    order = np.argsort(n_i[ok], kind="stable")
    return n_i[ok][order], p_i[ok][order], floor


def _two_level(A, B, r, ob_lb, pair_bounds, pair_exact, n_obs, thr):
    """Moving capsules against static obstacles, pruned in three levels.

    Level 0: over a block of consecutive samples the whole arm fits in one ball (a motion is
    continuous, so the ball is small); obstacles further than `thr` from every such ball are
    dropped.  Level 1: the same per capsule and block.  Level 2: each remaining (sample,
    capsule, obstacle) by the capsule's own ball, then exactly.  Pair index = obstacle*K + k.
    `ob_lb(points (M,3), obstacles (O,))` -> (M,O) lower bound on the distance beyond margin.
    """
    N, K = A.shape[:2]
    mid = 0.5 * (A + B)
    rad = 0.5 * np.linalg.norm(B - A, axis=-1) + r
    cen, big = _block_balls(mid, rad)                                       # (nb,K,3), (nb,K)
    nb = len(cen)
    arm_c = cen.mean(axis=1)
    arm_r = (np.linalg.norm(cen - arm_c[:, None], axis=-1) + big).max(axis=1)
    lb0 = ob_lb(arm_c, np.arange(n_obs)) - arm_r[:, None]                   # (nb,O)
    live = np.flatnonzero((lb0 <= thr).any(axis=0))
    floor0 = np.repeat(np.delete(lb0, live, axis=1).min(axis=1, initial=np.inf),
                       _BLOCK)[:N]
    lb1 = (ob_lb(cen.reshape(-1, 3), live).reshape(nb, K, -1) - big[:, :, None])
    lb1 = lb1.transpose(0, 2, 1).reshape(nb, len(live) * K)                 # (o, k) pairs
    n_i, p_i, floor = _expand(lb1 <= thr, lb1, N)
    k_i, o_i = p_i % K, live[p_i // K]
    lb, ub = pair_bounds(mid[n_i, k_i], rad[n_i, k_i], r[k_i], o_i)
    val, sel = _gather(N, n_i, lb, ub,
                       lambda ci: pair_exact(A[n_i[ci], k_i[ci]], B[n_i[ci], k_i[ci]],
                                             r[k_i[ci]], o_i[ci]), thr, np.minimum(floor, floor0))
    s = np.maximum(sel, 0)
    return val, (np.where(sel >= 0, o_i[s] * K + k_i[s], -1) if len(p_i) else sel)


def _boxes(scene, A, B, r, names, thr):
    N, K = A.shape[:2]
    lo, hi, nb = scene.box_lo, scene.box_hi, len(scene.box_lo)
    margin = scene.margin["steel"]
    if K == 0 or nb == 0:
        return _nothing(N)

    def bounds(mid, rad, rk, o):
        d = geo.point_box(mid, lo[o], hi[o])
        return d - rad - margin, d - rk - margin

    val, pair = _two_level(
        A, B, r, lambda c, o: geo.point_box(c[:, None], lo[o], hi[o]) - margin, bounds,
        lambda a, b, rk, o: geo.segment_box(a, b, lo[o], hi[o]) - rk - margin, nb, thr)
    return val, pair, lambda p: f"{names[p % K]} / {scene.box_names[p // K]}"


def _parked(scene, A, B, r, names, thr):
    N, K = A.shape[:2]
    OA, OB, orad = scene.other_a, scene.other_b, scene.other_r
    C, margin = len(OA), scene.margin["parked"]
    if K == 0 or C == 0:
        return _nothing(N)
    omid = 0.5 * (OA + OB)
    oball = 0.5 * np.linalg.norm(OB - OA, axis=-1) + orad

    def bounds(mid, rad, rk, o):
        d = np.linalg.norm(mid - omid[o], axis=-1)
        return d - rad - oball[o] - margin, d - rk - orad[o] - margin

    val, pair = _two_level(
        A, B, r,
        lambda c, o: np.linalg.norm(c[:, None] - omid[o], axis=-1) - oball[o] - margin,
        bounds,
        lambda a, b, rk, o: geo.segment_segment(a, b, OA[o], OB[o]) - rk - orad[o] - margin,
        C, thr)
    return val, pair, lambda p: f"{names[p % K]} / {scene.other_names[p // K]}"


def _self(A, B, m: ArmModel, margin, thr):
    N = len(A)
    i, j = m.self_pairs[:, 0], m.self_pairs[:, 1]
    if len(i) == 0:
        return _nothing(N)
    rr = m.radius[i] + m.radius[j] + margin
    mid = 0.5 * (A + B)
    half = 0.5 * np.linalg.norm(B - A, axis=-1)
    cen, big = _block_balls(mid, half)
    lb1 = np.linalg.norm(cen[:, i] - cen[:, j], axis=-1) - big[:, i] - big[:, j] - rr
    n_i, p_i, floor = _expand(lb1 <= thr, lb1, N)
    a, b = i[p_i], j[p_i]
    dm = np.linalg.norm(mid[n_i, a] - mid[n_i, b], axis=-1)
    ub = dm - rr[p_i]
    lb = ub - half[n_i, a] - half[n_i, b]

    def exact(ci):
        n, p = n_i[ci], p_i[ci]
        return geo.segment_segment(A[n, i[p]], B[n, i[p]], A[n, j[p]], B[n, j[p]]) - rr[p]

    val, sel = _gather(N, n_i, lb, ub, exact, thr, floor)
    pair = np.where(sel >= 0, p_i[np.maximum(sel, 0)] if len(p_i) else -1, -1)
    return val, pair, lambda p: f"{m.names[i[p]]} / {m.names[j[p]]}"
