"""The arm model: one Franka FR3 with the pen holder, in its own base frame.

The same object serves all six arms (identical robots, identical tools).  It knows nothing
about the table, the paper or the other arms.  Every call takes a batch: joint arrays are
(N, 7), poses are (N, 4, 4).

Frames (see fr3.py): the base frame is link0; the hand frame is the stock Franka hand mounting
(flange turned -45 deg).  `T_base_hand` is always that hand frame.  The tool is given in it.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from aris.kernel import fr3
from aris.kernel.tool import default_tool
from aris.types import Body, Limits, Tool

try:
    import aris_fr3_ik
except ImportError as e:        # an installation fault, not a refusal
    raise ImportError("the FR3 IK is not installed; from deployment/ run "
                      "`../.venv/bin/pip install ./native/fr3_ik`") from e

__all__ = ["Arm", "CapsuleTable", "ChainTable", "default_tool"]

N_BRANCH = aris_fr3_ik.N_SOL   # 8: two elbow roots x two forearm roots x two shoulder roots
IK_TOL = 1e-7           # m, and Frobenius norm on the rotation; answers land within ~1e-9,
                        # near singularities within 1e-6; a wrong answer misses by far more


@dataclass(frozen=True)
class CapsuleTable:
    """The K body capsules as data: capsule k runs from a[k] to b[k] in frame `frame[k]`
    (frame indices as in `ChainTable`)."""
    frame: np.ndarray        # (K,) int
    a: np.ndarray            # (K,3)
    b: np.ndarray            # (K,3)
    radius: np.ndarray       # (K,)
    is_pen: np.ndarray       # (K,) bool
    is_fixed: np.ndarray     # (K,) bool
    names: tuple
    is_tool: np.ndarray      # (K,) bool: bolted to the flange (gripper, blades, holder, tail)


@dataclass(frozen=True)
class ChainTable:
    """The kinematic chain as data.  Frame 0 is the base.  Frame i+1 (i = 0..6) is
    frame i @ RotX(dh[i,0]) @ TransX(dh[i,1]) @ RotZ(q[i]) @ TransZ(dh[i,2]) (modified DH).
    Then the fixed frames 8 + f (the flange, the hand): frame parent[f] @ [R[f], t[f]].
    The tool is not a frame of its own; its capsules sit in the hand frame (9)."""
    dh: np.ndarray           # (7,3) alpha, a, d
    parent: np.ndarray       # (F,) int
    R: np.ndarray            # (F,3,3)
    t: np.ndarray            # (F,3)
    names: tuple             # the F fixed frames' names


class Arm:
    """Kinematics, IK, limits and collision body of one FR3 carrying `tool`."""

    def __init__(self, tool: Tool):
        self.tool = tool
        self.limits = Limits(q_min=fr3.Q_MIN.copy(), q_max=fr3.Q_MAX.copy(),
                             qd_max=fr3.QD_MAX.copy(), qdd_max=fr3.QDD_MAX.copy(),
                             qddd_max=fr3.QDDD_MAX.copy())
        rows = [(n, f, a, b, r) for n, f, a, b, r in fr3.LINK_CAPSULES + fr3.HAND_CAPSULES]
        rows += [(c.name, 9, c.p0, c.p1, c.radius) for c in tool.capsules_hand]
        self._cap_frame = np.array([r[1] for r in rows], int)
        self._cap_a = np.array([r[2] for r in rows], float)
        self._cap_b = np.array([r[3] for r in rows], float)
        self._radius = np.array([r[4] for r in rows], float)
        self._names = tuple(r[0] for r in rows)
        self._is_pen = np.array([n in tool.pen_names for n in self._names])
        # link0 and link1: they only turn about the base axis, so their clearance to the rig is
        # checked once by the rig over the whole turn of joint 1, not per pose
        self._is_fixed = self._cap_frame <= 1
        tool_names = {c.name for c in tool.capsules_hand}
        self._is_tool = np.array([(n in tool_names or n.split(".")[0] in
                                   ("hand", "finger_left", "finger_right"))
                                  and n not in tool.pen_names for n in self._names])
        body_of = ["tool" if n in tool_names else n.split(".")[0] for n in self._names]
        pos = [fr3.CHAIN_POS[b] for b in body_of]
        self.self_pairs = np.array(
            [(i, j) for i in range(len(rows)) for j in range(i + 1, len(rows))
             if abs(pos[i] - pos[j]) >= fr3.SELF_CHAIN_GAP], int).reshape(-1, 2)
        self.reach = _joint_reach(self._cap_frame, self._cap_a, self._cap_b, self._radius)

    # ------------------------------------------------------------ the model as data

    def capsule_table(self) -> CapsuleTable:
        """The body capsules as plain arrays (copies), for code that builds its own `body`."""
        return CapsuleTable(frame=self._cap_frame.copy(), a=self._cap_a.copy(),
                            b=self._cap_b.copy(), radius=self._radius.copy(),
                            is_pen=self._is_pen.copy(), is_fixed=self._is_fixed.copy(),
                            names=self._names, is_tool=self._is_tool.copy())

    def chain_table(self) -> ChainTable:
        """The kinematic chain as plain arrays (copies); see `ChainTable`."""
        c, s = np.cos(fr3.HAND_TWIST), np.sin(fr3.HAND_TWIST)
        R = np.array([np.eye(3), [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]])
        t = np.array([[0.0, 0.0, fr3.D_FLANGE], [0.0, 0.0, 0.0]])
        return ChainTable(dh=np.array(fr3.DH, float), parent=np.array([7, 8]), R=R, t=t,
                          names=("flange", "hand"))

    # ------------------------------------------------------------ forward kinematics

    def _frames(self, Q):
        """(N,7) -> rotations (N,10,3,3) and origins (N,10,3) of link0..link7, flange, hand."""
        Q = np.asarray(Q, float).reshape(-1, 7)
        n = len(Q)
        R = np.empty((n, 10, 3, 3))
        p = np.empty((n, 10, 3))
        R[:, 0] = np.eye(3)
        p[:, 0] = 0.0
        for i, (al, a, d) in enumerate(fr3.DH):
            ca, sa = np.cos(al), np.sin(al)
            ct, st = np.cos(Q[:, i]), np.sin(Q[:, i])
            A = np.zeros((n, 3, 3))
            A[:, 0, 0], A[:, 0, 1] = ct, -st
            A[:, 1, 0], A[:, 1, 1], A[:, 1, 2] = st * ca, ct * ca, -sa
            A[:, 2, 0], A[:, 2, 1], A[:, 2, 2] = st * sa, ct * sa, ca
            R[:, i + 1] = R[:, i] @ A
            p[:, i + 1] = p[:, i] + R[:, i] @ np.array([a, -sa * d, ca * d])
        R[:, 8] = R[:, 7]
        p[:, 8] = p[:, 7] + fr3.D_FLANGE * R[:, 7, :, 2]
        c, s = np.cos(fr3.HAND_TWIST), np.sin(fr3.HAND_TWIST)
        R[:, 9] = R[:, 8] @ np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        p[:, 9] = p[:, 8]
        return R, p

    def link_frames(self, Q) -> np.ndarray:
        """(N,7) -> (N,10,4,4): link0..link7, the flange and the hand, in the base frame."""
        R, p = self._frames(Q)
        T = np.zeros(R.shape[:2] + (4, 4))
        T[..., :3, :3], T[..., :3, 3], T[..., 3, 3] = R, p, 1.0
        return T

    def fk(self, Q) -> np.ndarray:
        """(N,7) -> T_base_hand (N,4,4)."""
        R, p = self._frames(Q)
        T = np.zeros((len(R), 4, 4))
        T[:, :3, :3], T[:, :3, 3], T[:, 3, 3] = R[:, 9], p[:, 9], 1.0
        return T

    def tip(self, Q) -> np.ndarray:
        """(N,7) -> pen tip in the base frame (N,3)."""
        R, p = self._frames(Q)
        return p[:, 9] + R[:, 9] @ self.tool.tip_hand

    def pen_axis(self, Q) -> np.ndarray:
        """(N,7) -> unit vector along the pen, pointing out of the tip, base frame (N,3)."""
        R, _ = self._frames(Q)
        return R[:, 9] @ self.tool.pen_axis_hand

    # ------------------------------------------------------------ collision body

    def body(self, Q) -> Body:
        """(N,7) -> the capsules of links, hand, fingers, holder and pen at each configuration.

        The link0 and link1 capsules are marked `is_fixed`: they only turn about the base axis.
        """
        R, p = self._frames(Q)
        Rk, pk = R[:, self._cap_frame], p[:, self._cap_frame]
        return Body(p0=np.einsum("nkij,kj->nki", Rk, self._cap_a) + pk,
                    p1=np.einsum("nkij,kj->nki", Rk, self._cap_b) + pk,
                    radius=self._radius.copy(), names=self._names, is_pen=self._is_pen.copy(),
                    is_fixed=self._is_fixed.copy(), is_tool=self._is_tool.copy())

    # ------------------------------------------------------------ inverse kinematics

    def ik(self, T_base_hand, q7, with_flags=False):
        """All analytic solutions for each (hand pose, joint-7 angle).

        (M,4,4), (M,) -> Q (M,8,7), valid (M,8); with `with_flags`, also flags (M,8).
        Slot b is always the same root (elbow root, forearm root, shoulder branch), so the same
        slot means the same arm shape from one pose to the next.  A slot is valid only if the
        answer lies inside `limits` and our own forward kinematics puts the hand on the pose
        to IK_TOL; invalid slots are NaN.  A nonzero flag marks a degenerate answer: 1 means
        q2 = 0, where only q1 + q3 is fixed and one member of that family is returned (see
        native/fr3_ik/src/fr3_ik.hpp for 2 and 4).
        """
        T = np.ascontiguousarray(np.asarray(T_base_hand, float).reshape(-1, 4, 4))
        q7 = np.ascontiguousarray(np.broadcast_to(np.asarray(q7, float), (len(T),)))
        if not len(T):
            out = np.zeros((0, N_BRANCH, 7)), np.zeros((0, N_BRANCH), bool)
            return out + (np.zeros((0, N_BRANCH), np.uint8),) if with_flags else out
        raw, flags = aris_fr3_ik.solve(T, q7, self.limits.q_min, self.limits.q_max)
        ok = np.all(np.isfinite(raw), axis=-1)
        m, b = np.nonzero(ok)
        if len(m):
            R, p = self._frames(raw[m, b])
            pos_err = np.linalg.norm(p[:, 9] - T[m, :3, 3], axis=1)
            rot_err = np.linalg.norm(R[:, 9] - T[m, :3, :3], axis=(1, 2))
            ok[m, b] = (pos_err <= IK_TOL) & (rot_err <= IK_TOL)
        Q = np.where(ok[..., None], raw, np.nan)
        return (Q, ok, np.where(ok, flags, 0).astype(np.uint8)) if with_flags else (Q, ok)

    def hand_pose(self, tip_base, normal_base, spin, lean) -> np.ndarray:
        """The hand pose that puts the pen tip on `tip_base`.  -> T_base_hand (M,4,4).

        tip_base (M,3); normal_base (3,), the paper normal pointing toward the arm;
        spin (M,) rad; lean (M,2) rad.
        With lean = 0 the hand's z axis points against the normal (into the paper) and its x
        axis is the base x axis laid into the paper plane (base y if base x is within 26 deg
        of the normal), turned by `spin` about the normal.  `lean = (tx, ty)` then turns the
        hand by the rotation vector (tx, ty, 0) written in that hand frame, i.e. a rotation
        about an axis lying in the paper plane, by |lean| radians.  This is the old
        lateral-holder convention (planner.tool_lean after rotz(phi) @ rotx(pi)).
        """
        tip = np.asarray(tip_base, float).reshape(-1, 3)
        m = len(tip)
        spin = np.broadcast_to(np.asarray(spin, float), (m,))
        lean = np.broadcast_to(np.asarray(lean, float), (m, 2))
        n = np.asarray(normal_base, float)
        n = n / np.linalg.norm(n)
        ref = np.array([1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        e1 = ref - (ref @ n) * n
        e1 /= np.linalg.norm(e1)
        e2 = np.cross(n, e1)
        c, s = np.cos(spin)[:, None], np.sin(spin)[:, None]
        x = c * e1 + s * e2
        z = np.broadcast_to(-n, (m, 3))
        R0 = np.stack([x, np.cross(z, x), z], axis=-1)
        R = R0 @ _rot_in_plane(lean)
        T = np.zeros((m, 4, 4))
        T[:, :3, :3] = R
        T[:, :3, 3] = tip - R @ self.tool.tip_hand
        T[:, 3, 3] = 1.0
        return T

    # ------------------------------------------------------------ quality measures

    def tip_jacobian(self, Q) -> np.ndarray:
        """(N,7) -> (N,3,7): d tip / d q, column i = z_i x (tip - p_i)."""
        R, p = self._frames(Q)
        tip = p[:, 9] + R[:, 9] @ self.tool.tip_hand
        return np.cross(R[:, 1:8, :, 2], tip[:, None, :] - p[:, 1:8]).transpose(0, 2, 1)

    def sigma_min(self, Q) -> np.ndarray:
        """(N,7) -> (N,) smallest singular value of the tip Jacobian (old metrics.py)."""
        J = self.tip_jacobian(Q)
        return np.linalg.svd(J, compute_uv=False)[:, -1] if len(J) else np.zeros(0)

    def limit_margin(self, Q) -> np.ndarray:
        """(N,7) -> (N,) radians to the nearest joint position limit (negative: outside)."""
        Q = np.asarray(Q, float).reshape(-1, 7)
        return np.min(np.minimum(Q - fr3.Q_MIN, fr3.Q_MAX - Q), axis=1)


def _joint_reach(frame, a, b, radius):
    """(7, K): for joint j and capsule k, an upper bound over all configurations on the
    distance of any point of the capsule (radius included) from joint j's axis; 0 if joint j
    does not move the capsule.  Turning the joints by dq moves any point of capsule k along a
    path no longer than  sum_j reach[j, k] |dq_j|.

    Joint j turns frame j+1 about its own z axis.  A capsule rigidly fixed to frame j+1 has an
    exact distance: its endpoints' distance from that z axis (for the last joint the flange and
    hand frames are rigid to link7 too).  Further down the chain the bound is the triangle
    inequality: the sideways part of the first link offset, plus the length of every further
    offset, plus the endpoint's distance from its own frame's origin.
    """
    c, s = np.cos(fr3.HAND_TWIST), np.sin(fr3.HAND_TWIST)
    to7 = {8: (np.eye(3), np.array([0.0, 0.0, fr3.D_FLANGE])),
           9: (np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]),
               np.array([0.0, 0.0, fr3.D_FLANGE]))}
    reach = np.zeros((7, len(frame)))
    for k, f in enumerate(frame):
        ends = np.array([a[k], b[k]])
        if f in to7:                              # express in link7's frame, which is rigid
            R, t = to7[f]
            ends, f = ends @ R.T + t, 7
        for j in range(f):                        # joints 0..f-1 move frame f
            if f == j + 1:
                d = np.hypot(ends[:, 0], ends[:, 1]).max()
            else:
                al, aa, dd = fr3.DH[j + 1]
                d = (np.hypot(aa, np.sin(al) * dd)
                     + sum(np.hypot(fr3.DH[i][1], fr3.DH[i][2]) for i in range(j + 2, f))
                     + np.linalg.norm(ends, axis=1).max())
            reach[j, k] = d + radius[k]
    return reach


def _rot_in_plane(v):
    """(M,2) rotation vectors (vx, vy, 0) -> (M,3,3), Rodrigues; the identity at zero."""
    a = np.hypot(v[:, 0], v[:, 1])
    safe = np.where(a > 1e-12, a, 1.0)
    kx, ky = v[:, 0] / safe, v[:, 1] / safe
    K = np.zeros((len(v), 3, 3))
    K[:, 0, 2], K[:, 2, 0] = ky, -ky
    K[:, 1, 2], K[:, 2, 1] = -kx, kx
    s, c = np.sin(a)[:, None, None], (1.0 - np.cos(a))[:, None, None]
    return np.eye(3) + s * K + c * (K @ K)
