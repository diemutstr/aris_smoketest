"""Measure the capsules of the hand, the Fat fingers and the pen holder.  Run once, by hand.

    cd deployment && ../.venv/bin/python tests/oracle/fit_arm_capsules.py

Prints capsule rows to paste into `aris/kernel/fr3.py` (hand, fingers) and
`aris/kernel/tool.py` (holder, pen tail, pen), and the containment of the link capsules that
were taken over from the old `aris_sixarm/selfcoll.py` BODY_CAPSULES.

The fit is the old `scripts/self_collision_audit.py` method: search segment directions (the
principal axes plus 400 random ones), shrink the segment while the enclosing radius falls,
then take the radius as the exact maximum over every vertex, rounded up to the millimetre.
A body is split into bands along its own fitted axis; every vertex lies in exactly one band,
so the union of the band capsules contains the body.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from arm_meshes import LEAD_CENTRE, LEAD_LEN, LEAD_R, TAIL_CENTRE, TAIL_LEN, U, load_bodies  # noqa: E402


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
    print("\nhand + fingers (paste into fr3.py):")
    show("hand", 9, banded(bodies["hand"][1], 3))
    for side in ("finger_left", "finger_right"):
        show(side, 9, banded(bodies[side][1], 2))
    print("\ntool (paste into tool.py):")
    show("holder", 9, banded(bodies["holder"][1], 3))
    r = 0.005
    tail = (TAIL_CENTRE - 0.5 * TAIL_LEN * U, TAIL_CENTRE + 0.5 * TAIL_LEN * U)
    print(f"    pen_tail  {tail[0].round(7)} -> {tail[1].round(7)}  r {r}  (stick r {LEAD_R})")
    tip = LEAD_CENTRE + 0.5 * LEAD_LEN * U
    pen = (LEAD_CENTRE - 0.5 * LEAD_LEN * U, tip - r * U)
    print(f"    pen       {pen[0].round(7)} -> {pen[1].round(7)}  r {r}  tip {tip.round(7)}")


if __name__ == "__main__":
    main()
