"""Measure the capsules of the hand, the Fat finger blades, the pen holder and the pencil tail.
Run once, by hand (about two minutes):

    cd deployment && ../.venv/bin/python tests/oracle/fit_arm_capsules.py

Prints capsule rows to paste into `aris/kernel/fr3.py` (hand, blades) and `aris/kernel/tool.py`
(holder, pencil tail), and the containment of the link capsules taken over from the old
`aris_sixarm/selfcoll.py` BODY_CAPSULES.

Two demands on every part bolted to the hand (2026-09-29): every mesh vertex inside the part's
capsules, and, with the hand leaned up to 15 deg from square to the paper in any direction,
the lowest point of the capsules no more than 1.5 mm below the lowest mesh vertex (so a
clearance to the paper measured on the capsules is the clearance of the metal).

`fit_facing` does it greedily.  A candidate capsule is seeded at an uncovered vertex, with an
axis direction and a radius from fixed lists, and placed so that the vertex lies on the
capsule's BOTTOM (the side facing the paper): its surface then sticks out below that vertex by
at most r (1 - cos 15 deg) anywhere in the cone.  The segment is extended along its axis as far
as the part reaches and as far as the capsule stays within FIT_TOL of the part's lowest point
in every cone direction (a set of linear constraints on the two end parameters).  Each round
takes the candidate covering the most uncovered vertices per volume**beta, until every vertex
is covered; redundant capsules are then dropped.  The upper part of a big body (the hand
above its lowest 50 mm) is first covered by the old banded fit, kept only where admissible.
The banded fit is the old `scripts/self_collision_audit.py` method: search segment directions,
shrink the segment while the enclosing radius falls, radius = exact maximum over the vertices.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from arm_meshes import LEAD_CENTRE, LEAD_LEN, U, load_bodies  # noqa: E402


def pt_seg(P, a, b):
    ab = b - a
    t = np.clip((P - a) @ ab / max(ab @ ab, 1e-18), 0.0, 1.0)
    return np.linalg.norm(P - (a + t[:, None] * ab), axis=1)


def fit_capsule(V, n_dir=400, seed=0):
    V = np.asarray(V, float)
    c = V.mean(0)
    X = V - c
    vt = np.linalg.svd(X, full_matrices=False)[2]
    rng = np.random.default_rng(seed)
    dirs = list(vt) + [d / np.linalg.norm(d) for d in rng.normal(size=(n_dir, 3))]
    best = None
    for d in dirs:
        t = X @ d
        perp = np.linalg.norm(X - np.outer(t, d), axis=1)
        t0, t1 = float(t.min()), float(t.max())

        def rad(a, b):
            dd = np.where(t < a, np.hypot(a - t, perp),
                          np.where(t > b, np.hypot(t - b, perp), perp))
            return float(dd.max())

        r = rad(t0, t1)
        for step in (0.04, 0.01, 0.0025, 0.0005):
            while True:
                cand = [(rad(t0 + step, t1), t0 + step, t1), (rad(t0, t1 - step), t0, t1 - step)]
                cand = [c2 for c2 in cand if c2[2] > c2[1]]
                if not cand:
                    break
                r2, a2, b2 = min(cand)
                if r2 < r - 1e-9:
                    r, t0, t1 = r2, a2, b2
                else:
                    break
        if best is None or r < best[0]:
            best = (r, d, t0, t1)
    r, d, t0, t1 = best
    return c + t0 * d, c + t1 * d


def banded(V, bands):
    a0, b0 = fit_capsule(V)
    d = (b0 - a0) / np.linalg.norm(b0 - a0)
    t = (V - a0) @ d
    edges = np.linspace(t.min(), t.max(), bands + 1)
    edges[0] -= 1.0
    edges[-1] += 1.0
    rows = []
    for k in range(bands):
        W = V[(t >= edges[k]) & (t < edges[k + 1])]
        a, b = fit_capsule(W)
        r = pt_seg(W, a, b).max()
        rows.append((a, b, float(np.ceil(r * 1000.0) / 1000.0), r))
    return rows


def show(name, frame, rows):
    for k, (a, b, r, r_fit) in enumerate(rows):
        print(f'    ("{name}.{k}", {frame}, ({a[0]:.4f}, {a[1]:.4f}, {a[2]:.4f}), '
              f'({b[0]:.4f}, {b[1]:.4f}, {b[2]:.4f}), {r:.3f}),   # mesh {r_fit:.4f}')


FIT_TOL = 1.2e-3        # m; the demand is 1.5 mm, checked on finer leans by the test
CONE = 15.0             # deg
# per part: radii to try (mm), volume exponent beta, height of the top part given to the
# banded fit first (m, 0 = none), number of bands there
PARAMS = {
    "hand": ((3, 4.5, 6.5, 9, 13, 18, 25, 32, 40), 0.15, 0.05, 4),
    "finger_left": ((2, 3, 4.5, 6.5, 9, 13, 18, 25), 0.08, 0.0, 0),
    "finger_right": ((2, 3, 4.5, 6.5, 9, 13, 18, 25), 0.08, 0.0, 0),
    "holder": ((2, 3, 4.5, 6.5, 9, 13, 18, 25), 0.2, 0.0, 0),
    "pen_tail": ((2, 3, 3.6, 4.5), 0.0, 0.0, 0),
}


def cone_dirs(deg=CONE, step=1.0, naz=48):
    """Downward directions in the hand frame: +z tilted up to `deg` in every direction."""
    out = [np.array([0.0, 0.0, 1.0])]
    for t in np.arange(step, deg + 1e-9, step):
        a = np.deg2rad(t)
        for p in np.linspace(0, 2 * np.pi, naz, endpoint=False):
            out.append([np.sin(a) * np.cos(p), np.sin(a) * np.sin(p), np.cos(a)])
    return np.array(out)


def _axis_dirs():
    i = np.arange(40) + 0.5
    phi, th = np.arccos(1 - 2 * i / 40), np.pi * (1 + 5 ** 0.5) * i
    d = np.stack([np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)], 1)
    d = list(d[d[:, 2] >= 0]) + [U, np.array([1.0, 0, 0]), np.array([0, 1.0, 0])]
    return np.array([e / np.linalg.norm(e) for e in d if abs(e[2]) < 0.97])


def _admissible(D, H, a, b, r):
    return float((np.maximum(D @ a, D @ b) + r - H).max())


def _candidate(V, D, H, v, e, r, z=np.array([0.0, 0.0, 1.0])):
    w = z - (z @ e) * e
    w /= np.linalg.norm(w)
    c = v - (r - 1e-6) * w
    const, ed = D @ c + r - H - FIT_TOL, D @ e
    if np.any(const[np.abs(ed) < 1e-12] > 0):
        return None
    pos, neg = ed > 1e-12, ed < -1e-12
    thi = np.min(-const[pos] / ed[pos]) if pos.any() else np.inf
    tlo = np.max(-const[neg] / ed[neg]) if neg.any() else -np.inf
    if not (tlo <= 1e-12 and thi >= -1e-12):
        return None
    X = V - c
    pr = X @ e
    near = np.linalg.norm(X - np.outer(pr, e), axis=1) <= r
    lo, hi = max(tlo, min(pr[near].min(), 0.0)), min(thi, max(pr[near].max(), 0.0))
    return c + lo * e, c + hi * e


def fit_facing(V, radii_mm, beta, slab, bands, seed=0, n_seed=30):
    rng = np.random.default_rng(seed)
    D = cone_dirs()
    H = (V @ D.T).max(0)
    E = _axis_dirs()
    unc, caps = np.ones(len(V), bool), []
    if slab > 0:
        up = V[:, 2] < V[:, 2].max() - slab
        for a, b, r, _ in banded(V[up], bands):
            if _admissible(D, H, a, b, r) <= FIT_TOL:
                caps.append((a, b, r))
                unc &= ~(pt_seg(V, a, b) <= r)
    while unc.any():
        idx = np.flatnonzero(unc)
        seeds = np.unique(np.concatenate([idx[np.argsort(-V[idx, 2])[:n_seed // 2]],
                                          rng.choice(idx, min(len(idx), n_seed // 2), False)]))
        best = None
        for s in seeds:
            for e in E:
                for r in np.asarray(radii_mm) * 1e-3:
                    ab = _candidate(V, D, H, V[s], e, r)
                    if ab is None:
                        continue
                    a, b = ab
                    n = (pt_seg(V[idx], a, b) <= r).sum()
                    vol = np.pi * r * r * np.linalg.norm(b - a) + 4 / 3 * np.pi * r ** 3
                    score = n / vol ** beta
                    if best is None or score > best[0]:
                        best = (score, a, b, r)
        _, a, b, r = best
        caps.append((a, b, r))
        unc &= ~(pt_seg(V, a, b) <= r)
    inside = np.array([pt_seg(V, a, b) <= r for a, b, r in caps])
    keep = list(range(len(caps)))
    for k in sorted(keep, key=lambda k: inside[k].sum()):
        others = [j for j in keep if j != k]
        if others and inside[others].any(0).all():
            keep = others
    return [caps[k] for k in keep], D, H


def rounded(V, caps, D, H):
    """Round ends to 0.01 mm, then give each capsule the radius that keeps its vertices in."""
    own = np.argmin(np.array([pt_seg(V, a, b) - r for a, b, r in caps]), axis=0)
    rows = []
    for k, (a, b, r) in enumerate(caps):
        a, b = np.round(a, 5), np.round(b, 5)
        W = V[own == k]
        need = pt_seg(W, a, b).max() if len(W) else 0.0
        rows.append((a, b, float(np.ceil(max(need, 1e-5) * 1e5) / 1e5)))
    worst = max(_admissible(D, H, a, b, r) for a, b, r in rows)
    return rows, worst


def main():
    bodies = load_bodies()
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    from aris.kernel import fr3
    print("containment of the link capsules taken from the old selfcoll table (mm):")
    for i in range(8):
        name = f"link{i}"
        V = bodies[name][1]
        caps = [c for c in fr3.LINK_CAPSULES if c[0].startswith(name + ".")]
        d = np.min([pt_seg(V, np.array(a), np.array(b)) - r for _, _, a, b, r in caps], axis=0)
        print(f"  {name}: worst vertex {1e3 * d.max():+.2f} (<= 0 is inside), {len(V)} vertices")
    for part, (radii, beta, slab, bands) in PARAMS.items():
        V = bodies[part][1]
        caps, D, H = fit_facing(V, radii, beta, slab, bands)
        rows, worst = rounded(V, caps, D, H)
        vol = sum(np.pi * r * r * np.linalg.norm(b - a) + 4 / 3 * np.pi * r ** 3 for a, b, r in rows)
        print(f"\n{part}: {len(rows)} capsules, {1e3 * vol:.3f} L, lowest point at most "
              f"{1e3 * worst:.2f} mm below the mesh's in the cone")
        for k, (a, b, r) in enumerate(rows):
            print(f'    ("{part}.{k}", 9, ({a[0]:.5f}, {a[1]:.5f}, {a[2]:.5f}), '
                  f'({b[0]:.5f}, {b[1]:.5f}, {b[2]:.5f}), {r:.5f}),')
    tip = LEAD_CENTRE + 0.5 * LEAD_LEN * U
    print(f"\npen: from {(LEAD_CENTRE - 0.5 * LEAD_LEN * U).round(7)} to the tip {tip.round(7)}")


if __name__ == "__main__":
    main()
