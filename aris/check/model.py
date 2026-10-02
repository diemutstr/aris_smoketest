"""The checker's own arm: the FR3 with the pen holder, built from `fr3.json`.

Written apart from the planner's kernel on purpose.  The kinematics follow the URDF: each
joint is a fixed transform (a shift, then a roll-pitch-yaw turn) followed by a turn of q about
the new z axis.  (The kernel uses Denavit-Hartenberg parameters instead; the two only agree if
both are right.)  Everything is computed straight in the table frame, starting from the arm's
mount pose, because the checker's obstacles live there.
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

DATA = Path(__file__).with_name("fr3.json")
N_FRAMES = 9          # link0 .. link7, then the hand


def _rpy(r, p, y) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


@dataclass(frozen=True)
class ArmModel:
    joint_R: np.ndarray        # (7,3,3) fixed turn of each joint's origin
    joint_t: np.ndarray        # (7,3)   fixed shift of each joint's origin, in the parent frame
    hand_R: np.ndarray         # (3,3)   link7 -> hand
    hand_t: np.ndarray         # (3,)
    cap_frame: np.ndarray      # (K,) 0..7 link, 8 hand
    cap_a: np.ndarray          # (K,3) in the capsule's frame
    cap_b: np.ndarray          # (K,3)
    radius: np.ndarray         # (K,)
    names: tuple
    is_pen: np.ndarray         # (K,) bool
    is_fixed: np.ndarray       # (K,) bool, bolted to the mount (link0)
    is_tool: np.ndarray        # (K,) bool, bolted to the flange, the pen excluded
    tip_hand: np.ndarray       # (3,)
    q_min: np.ndarray
    q_max: np.ndarray
    qd_max: np.ndarray
    qdd_max: np.ndarray
    qddd_max: np.ndarray
    self_pairs: np.ndarray     # (P,2)
    travel: np.ndarray         # (7,K) metres a capsule point can move per radian of joint j


def load_model(tip_hand=None, pen_length=None, pen_radius=None) -> ArmModel:
    """The arm from `fr3.json`.  `tip_hand` replaces the pen tip (a measured tip); without it,
    `pen_length` (the pen's nominal length, rig.json) moves the model's tip along the pen axis.
    `pen_radius` replaces the pen capsule's radius.  A moved pen capsule keeps its direction:
    it lies on the line through the tip along the pen axis, its far end where the model's
    projects onto that line, its surface reaching exactly to the tip."""
    d = json.loads(DATA.read_text())
    joints = d["chain"]["joints"]
    joint_R = np.array([_rpy(*j["rpy"]) for j in joints])
    joint_t = np.array([j["xyz"] for j in joints], float)
    hand_R, hand_t = np.eye(3), np.zeros(3)
    for f in d["chain"]["link7_to_hand"]:
        hand_t = hand_t + hand_R @ np.asarray(f["xyz"], float)
        hand_R = hand_R @ _rpy(*f["rpy"])

    lean = np.deg2rad(d["tool"]["pen_lean_deg"])
    pen_axis = np.array([np.sin(lean), 0.0, np.cos(lean)])
    model_tip = np.asarray(d["tool"]["tip_hand"], float)
    if tip_hand is not None:
        tip = np.asarray(tip_hand, float).reshape(3)
    elif pen_length is not None:
        tip = model_tip + (float(pen_length) - d["tool"]["pen_length_m"]) * pen_axis
    else:
        tip = model_tip
    rows = d["capsules"]["rows"]
    names = tuple(r[0] for r in rows)
    radius = np.array([pen_radius if r[3] == "tip" and pen_radius is not None else r[4]
                       for r in rows], float)
    cap_a = np.array([r[2] for r in rows], float)
    cap_b = np.array([tip - radius[k] * pen_axis if r[3] == "tip" else r[3]
                      for k, r in enumerate(rows)], float)
    if not np.array_equal(tip, model_tip):        # the model as built is left unrounded
        for k, r in enumerate(rows):
            if r[3] == "tip":
                cap_a[k] = cap_b[k] - max(float((cap_b[k] - cap_a[k]) @ pen_axis), 0.0) * pen_axis
    cap_frame = np.array([r[1] for r in rows], int)

    lim = d["limits"]
    sc = d["self_collision"]
    body = [n.split(".")[0] for n in names]
    pos = np.array([sc["chain_position"][b] for b in body])
    pairs = np.array([(i, j) for i in range(len(rows)) for j in range(i + 1, len(rows))
                      if abs(pos[i] - pos[j]) >= sc["min_gap"]], int).reshape(-1, 2)
    return ArmModel(
        joint_R=joint_R, joint_t=joint_t, hand_R=hand_R, hand_t=hand_t,
        cap_frame=cap_frame, cap_a=cap_a, cap_b=cap_b, radius=radius, names=names,
        is_pen=np.array([n in d["tool"]["pen"] for n in names]), is_fixed=np.isin(cap_frame, d["capsules"]["fixed_frames"]),
        is_tool=np.array([b in d["tool"]["tool_bodies"] and n not in d["tool"]["pen"]
                          for b, n in zip(body, names)]),
        tip_hand=tip, q_min=np.array(lim["q_min"], float), q_max=np.array(lim["q_max"], float),
        qd_max=np.array(lim["qd_max"], float), qdd_max=np.array(lim["qdd_max"], float),
        qddd_max=np.array(lim["qddd_max"], float), self_pairs=pairs,
        travel=_travel(joint_t, hand_t, cap_frame, cap_a, cap_b))


def _travel(joint_t, hand_t, cap_frame, cap_a, cap_b) -> np.ndarray:
    """(7,K): turning joint j by one radian moves any point of capsule k's segment by at most
    its distance from joint j's axis, whatever the other joints do.

    Joint j turns frame j+1 about its own z axis.  A capsule on frame j+1 itself: the distance
    is exactly sqrt(x^2 + y^2) of its end points (the larger one bounds the whole segment).
    A capsule further down the chain: at most the distance to frame j+1's origin, which the
    triangle inequality bounds by the fixed shifts in between plus the point's own distance
    from its frame origin.  Capsules on frames at or before j do not move with joint j.
    """
    shifts = [np.linalg.norm(t) for t in joint_t] + [np.linalg.norm(hand_t)]   # into frame 1..8
    far = np.maximum(np.linalg.norm(cap_a, axis=1), np.linalg.norm(cap_b, axis=1))
    near = np.maximum(np.hypot(cap_a[:, 0], cap_a[:, 1]), np.hypot(cap_b[:, 0], cap_b[:, 1]))
    out = np.zeros((7, len(cap_frame)))
    for j in range(7):
        for k, f in enumerate(cap_frame):
            if f == j + 1:
                out[j, k] = near[k]
            elif f > j + 1:
                out[j, k] = sum(shifts[j + 1:f]) + far[k]
    return out


def frames(model: ArmModel, Q, T_table_base) -> tuple[np.ndarray, np.ndarray]:
    """(N,7) -> rotations (N,9,3,3) and origins (N,9,3) of link0..link7 and the hand, table frame.

    Frame i+1's z axis is joint i's axis and its origin lies on it.
    """
    Q = np.asarray(Q, float).reshape(-1, 7)
    c, s = np.cos(Q), np.sin(Q)
    Rs = [np.broadcast_to(T_table_base[:3, :3], (len(Q), 3, 3))]
    ps = [np.broadcast_to(T_table_base[:3, 3], (len(Q), 3))]
    for i in range(7):
        J = model.joint_R[i]
        # the joint's own turn: J @ Rz(q), column by column
        L = np.empty((len(Q), 3, 3))
        L[:, :, 0] = J[:, 0] * c[:, i, None] + J[:, 1] * s[:, i, None]
        L[:, :, 1] = J[:, 1] * c[:, i, None] - J[:, 0] * s[:, i, None]
        L[:, :, 2] = J[:, 2]
        ps.append(ps[-1] + np.einsum("nij,j->ni", Rs[-1], model.joint_t[i]))
        Rs.append(Rs[-1] @ L)
    ps.append(ps[-1] + np.einsum("nij,j->ni", Rs[-1], model.hand_t))
    Rs.append(Rs[-1] @ model.hand_R)
    return np.stack(Rs, axis=1), np.stack(ps, axis=1)


def capsules(model: ArmModel, Q, T_table_base) -> tuple[np.ndarray, np.ndarray]:
    """(N,7) -> capsule end points (N,K,3) and (N,K,3) in the table frame."""
    return _ends(model, *frames(model, Q, T_table_base))


def _ends(model, R, p):
    """Capsule end points, frame by frame: (N,3,3) @ (3, ends on that frame)."""
    n, K = len(R), len(model.cap_frame)
    a, b = np.empty((n, K, 3)), np.empty((n, K, 3))
    for f in np.unique(model.cap_frame):
        k = np.flatnonzero(model.cap_frame == f)
        X = np.concatenate([model.cap_a[k], model.cap_b[k]]).T                  # (3, 2m)
        Y = (R[:, f] @ X).transpose(0, 2, 1) + p[:, f, None]                   # (N, 2m, 3)
        a[:, k], b[:, k] = Y[:, :len(k)], Y[:, len(k):]
    return a, b


def pose(model: ArmModel, Q, T_table_base):
    """Capsule end points (N,K,3) twice, and D (N,7,K): how far each capsule's segment is, at
    most, from each joint's axis right now (0 where the joint does not move the capsule)."""
    R, p = frames(model, Q, T_table_base)
    a, b = _ends(model, R, p)
    o, z = p[:, 1:8], np.ascontiguousarray(R[:, 1:8, :, 2])   # joint j turns about z_j at o_j
    moves = model.travel > 0
    oo, oz = np.einsum("nji,nji->nj", o, o), np.einsum("nji,nji->nj", o, z)

    def off_axis(x):
        # |x - o|^2 - ((x - o).z)^2, written with batched products (N,7,3) @ (N,3,K)
        xt = np.ascontiguousarray(x.transpose(0, 2, 1))
        d2 = (np.einsum("nki,nki->nk", x, x)[:, None] - 2 * (o @ xt) + oo[:, :, None]
              - ((z @ xt) - oz[:, :, None]) ** 2)
        return np.sqrt(np.maximum(d2, 0.0)) + 1e-9       # the 1e-9 covers rounding
    return a, b, np.maximum(off_axis(a), off_axis(b)) * moves


def tip(model: ArmModel, Q, T_table_base) -> np.ndarray:
    """(N,7) -> pen tip (N,3) in the frame `T_table_base` maps to (pass eye(4) for the base)."""
    R, p = frames(model, Q, T_table_base)
    return p[:, 8] + R[:, 8] @ model.tip_hand
