"""Run the old distance code on fixed random inputs and save its answers.

Run as its own process from the repository root:
    ARIS_RIG=proposed ARIS_TOOL=lateral .venv/bin/python deployment/tests/oracle/make_collide_reference.py
Writes deployment/tests/data/collide_old_segbox.npz and collide_old_segseg.npz.
"""
import os
import sys
from pathlib import Path

import numpy as np

assert os.environ.get("ARIS_RIG") == "proposed" and os.environ.get("ARIS_TOOL") == "lateral"
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from aris_sixarm.rig_final import segment_box_clearance  # noqa: E402
from aris_sixarm.coordination import seg_seg_dist  # noqa: E402

OUT = ROOT / "deployment" / "tests" / "data"
rng = np.random.default_rng(20260929)

# ---- segment vs axis-aligned box: random, crossing, grazing a face, parallel to a face
n = 4000
lo = rng.uniform(-0.3, 0.1, (n, 3))
hi = lo + rng.uniform(0.02, 0.4, (n, 3))
A = rng.uniform(-0.6, 0.6, (n, 3))
B = A + rng.normal(size=(n, 3)) * 0.3
k = n // 4
ctr = 0.5 * (lo + hi)
B[:k] = 2 * ctr[:k] - A[:k]                                  # through the centre
A[k:2 * k, 2] = hi[k:2 * k, 2] + rng.uniform(0, 1e-3, k)       # parallel above the top face
B[k:2 * k, 2] = A[k:2 * k, 2]
A[2 * k:3 * k] = hi[2 * k:3 * k] + rng.uniform(0, 1e-4, (k, 3))  # starting at a corner
old_box = np.array([segment_box_clearance(A[i:i + 1], B[i:i + 1], [{"lo": lo[i], "hi": hi[i]}])[0]
                    for i in range(n)])
np.savez(OUT / "collide_old_segbox.npz", A=A, B=B, lo=lo, hi=hi, old=old_box)

# ---- segment vs segment: random, parallel, collinear, crossing, a point
m = 10000
p0 = rng.normal(size=(m, 3)) * 0.3
p1 = p0 + rng.normal(size=(m, 3)) * 0.3
q0 = rng.normal(size=(m, 3)) * 0.3
q1 = q0 + rng.normal(size=(m, 3)) * 0.3
j = m // 5
off = rng.normal(size=(j, 3)) * 0.05
q0[:j], q1[:j] = p0[:j] + off, p1[:j] + off + rng.uniform(-0.5, 0.5, (j, 1)) * (p1[:j] - p0[:j])  # parallel
q0[j:2 * j] = p0[j:2 * j] + 0.7 * (p1[j:2 * j] - p0[j:2 * j])          # collinear, overlapping
q1[j:2 * j] = p0[j:2 * j] + 1.6 * (p1[j:2 * j] - p0[j:2 * j])
mid = p0[2 * j:3 * j] + 0.4 * (p1[2 * j:3 * j] - p0[2 * j:3 * j])         # crossing
q0[2 * j:3 * j], q1[2 * j:3 * j] = mid - q1[2 * j:3 * j] + q0[2 * j:3 * j], mid + 0.5 * (q1[2 * j:3 * j] - q0[2 * j:3 * j])
q1[3 * j:4 * j] = q0[3 * j:4 * j]                                        # a point
old_seg = seg_seg_dist(p0, p1, q0, q1)
np.savez(OUT / "collide_old_segseg.npz", p0=p0, p1=p1, q0=q0, q1=q1, old=old_seg)
print("box", old_box.shape, "seg", old_seg.shape)
