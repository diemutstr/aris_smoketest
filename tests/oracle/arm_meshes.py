"""The FR3 meshes of the real build, as vertex clouds in each body's own frame.

Used twice: by `fit_arm_capsules.py`, which measured the hand, finger and tool capsules in
`aris/kernel/fr3.py` and `aris/kernel/tool.py`, and by `test_kernel_arm.py`, which checks that
every vertex lies inside the capsules.  Test-side only: needs `trimesh` (already in the venv).

Sources, all under `assets/system_model/` (the model of the physical installation):
  link0..link7, hand   manufacturer collision shell (meshes/collision/*.obj) UNION the
                       high-quality visual mesh (meshes/fr3/*.gltf); glTF is Y-up and is turned
                       to the link frame with Rx(+90 deg), the same as Drake and the old audit do
  finger_left/right    the Fat Franka Finger blade (meshes/fatfinger/), placed as in
                       installation_fatfingers.urdf
  holder               pen-holder housing and cap (meshes/penholder/), already in the hand frame
  pen, pen_tail        the graphite stick (radius 3.5 mm), from the URDF's pen_lead and
                       graphite_tail cylinders; the exposed lead is sharpened to a point at the tip
"""
from __future__ import annotations

from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
MESH = ROOT / "assets" / "system_model" / "meshes"
GLTF_ZUP = np.array([[1.0, 0, 0], [0, 0, -1.0], [0, 1.0, 0]])

# frame index as `Arm.link_frames` returns them: 0..7 = link0..link7, 8 = flange, 9 = hand
FRAME = {f"link{i}": i for i in range(8)}
FRAME.update(hand=9, finger_left=9, finger_right=9, holder=9, pen=9, pen_tail=9)

LEAN = np.deg2rad(23.0)
U = np.array([np.sin(LEAN), 0.0, np.cos(LEAN)])        # bore direction, hand frame
TCP_Z = 0.1034
LEAD_R = 0.0035
LEAD_CENTRE = np.array([0.0821295963326126, 0.0, 0.03682113982894393 + TCP_Z])
LEAD_LEN = 0.020000015470188197
TAIL_CENTRE = np.array([0.030809987258879113, 0.0, 0.019303972968167944])
TAIL_LEN = 0.072514
SHARPEN_LEN = 0.010      # the lead narrows to a point over its last 10 mm


def _obj(path):
    import trimesh
    return np.asarray(trimesh.load(path, process=False, force="mesh").vertices, float)


def _gltf(name):
    import trimesh
    sc = trimesh.load(MESH / "fr3" / f"{name}.gltf", process=False)
    V = []
    for node in sc.graph.nodes_geometry:
        T, g = sc.graph[node]
        V.append(trimesh.transform_points(sc.geometry[g].vertices, np.asarray(T)))
    return np.concatenate(V) @ GLTF_ZUP.T


def _rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _stick(centre, length, sharpen=False, n_ring=48, n_len=60):
    """Vertices of a round graphite stick along U, optionally sharpened at its far end."""
    e1 = np.array([np.cos(LEAN), 0.0, -np.sin(LEAN)])
    e2 = np.array([0.0, 1.0, 0.0])
    s = np.linspace(-0.5 * length, 0.5 * length, n_len)
    r = np.full_like(s, LEAD_R)
    if sharpen:
        tip_dist = 0.5 * length - s
        r = np.minimum(r, LEAD_R * tip_dist / SHARPEN_LEN)
    a = np.linspace(0, 2 * np.pi, n_ring, endpoint=False)
    ring = np.cos(a)[:, None] * e1 + np.sin(a)[:, None] * e2
    P = centre + s[:, None, None] * U + r[:, None, None] * ring[None]
    return np.vstack([P.reshape(-1, 3), centre + s[:, None] * U])


def load_bodies():
    """-> {part: (frame_index, V (n,3) in that frame)}."""
    out = {}
    for i in range(8):
        n = f"link{i}"
        out[n] = (i, np.vstack([_obj(MESH / "collision" / f"{n}.obj"), _gltf(n)]))
    out["hand"] = (9, np.vstack([_obj(MESH / "collision" / "hand.obj"), _gltf("hand")]))
    finger = _obj(MESH / "fatfinger" / "fatfinger_leftfinger.obj")
    out["finger_left"] = (9, finger + np.array([0.0, 0.0169337, 0.0584]))
    right = finger @ _rz(np.pi).T + np.array([0.069, 0.0, 0.0])
    out["finger_right"] = (9, right + np.array([0.0, -0.0169337, 0.0584]))
    out["holder"] = (9, np.vstack([_obj(MESH / "penholder" / "penholder22_housing_hand.obj"),
                                   _obj(MESH / "penholder" / "penholder22_cap_hand.obj")]))
    out["pen"] = (9, _stick(LEAD_CENTRE, LEAD_LEN, sharpen=True))
    out["pen_tail"] = (9, _stick(TAIL_CENTRE, TAIL_LEN))
    return out
