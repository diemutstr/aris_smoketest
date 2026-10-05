"""The simulated person of the mark job, and the "true" rig they work in.

The station believes the rig it loaded; the simulated world can differ: another config
directory (`aris serve --sim-truth <dir>`: its base poses, pen tips and mark positions are the
truth), or the believed rig with every base moved by a fixed amount (`--sim-base-error
mm,mrad`: each slot's base shifted that much horizontally and turned that much about the
vertical, in a direction of its own that follows from its name, so the same run gives the
same truth; x, y and yaw are what the marks find, height and tilt are the plane job's).
`--sim-mark-error cm` moves every true mark that far from its nominal position, in a
direction of its own: marks are taped by hand, 2 to 5 cm off is normal.

The person of a `guide`: from the hover, they seat the TRUE pen tip on the TRUE mark keeping
the hover's hand orientation as the true arm holds it, a little off (0.3 mm of guiding
error, from a seeded generator), and press a button: "check" unless a scripted list says
otherwise (`buttons`: e.g. ["check", "cross", "check", "circle"], used in order, then
"check"; a cross stays inside the driver, the person tries again), or fail the hand-over (a
button "fail: <why>").
"""
from __future__ import annotations

import hashlib
import threading

import numpy as np

GUIDING_ERROR = 0.0003   # m


def _seed(*parts) -> int:
    return int.from_bytes(hashlib.blake2b("|".join(map(str, parts)).encode(),
                                          digest_size=8).digest(), "little")


def _rot(v) -> np.ndarray:
    v = np.asarray(v, float)
    a = float(np.linalg.norm(v))
    if a < 1e-15:
        return np.eye(3)
    k = v / a
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * (K @ K)


class Truth:
    """The true base poses, pen tips and mark positions of the simulated world."""

    def __init__(self, rig, truth_rig=None, base_error=None, mark_error=None):
        self.rig = rig
        self.T, self.tip, self.marks = {}, {}, {}
        for a in rig.arm_ids:
            if truth_rig is not None:
                self.T[a] = truth_rig.T_table_base(a)
                self.tip[a] = truth_rig.arm(a).tool.tip_hand
            else:
                T = rig.T_table_base(a)
                if base_error is not None:
                    mm, mrad = base_error
                    g = np.random.default_rng(_seed("base error", a))
                    d = g.normal(size=2)
                    sign = 1.0 if g.uniform() < 0.5 else -1.0
                    T = T.copy()
                    T[:2, 3] += 1e-3 * mm * d / np.linalg.norm(d)
                    R = _rot([0.0, 0.0, sign * 1e-3 * mrad])
                    T[:3, :3] = R @ T[:3, :3]
                    T[:2, 3] = (R @ np.r_[T[:2, 3], 0.0])[:2]   # turned about the table z
                self.T[a] = T
                self.tip[a] = rig.arm(a).tool.tip_hand
        src = truth_rig if truth_rig is not None else rig
        self.marks = {n: np.asarray(src.marks[n][0], float) for n in src.marks}
        if mark_error:                     # the marks taped where they are, not where planned
            for n in self.marks:
                d = np.random.default_rng(_seed("mark error", n)).normal(size=2)
                self.marks[n] = self.marks[n] + 1e-2 * float(mark_error) * d / np.linalg.norm(d)
        self.paper_z = src.paper_z


class Person:
    """`person(motion, q)` for one simulated arm (`SimArm(person=...)`)."""

    def __init__(self, truth: Truth, slot, buttons=(), error: float = GUIDING_ERROR):
        self.truth, self.slot = truth, slot
        self.buttons, self.error = list(buttons), float(error)
        self.count, self.crossed, self._lock = 0, 0, threading.Lock()

    def _button(self) -> str:
        """The next button; a ✗ (cross) never leaves the driver: the person seats the pen
        again and the next button is the answer."""
        with self._lock:
            self.count += 1
            while self.buttons and self.buttons[0] == "cross":
                self.buttons.pop(0)
                self.crossed += 1
            return self.buttons.pop(0) if self.buttons else "check"

    def __call__(self, motion, q_now):
        button = self._button()
        if button.startswith("fail"):
            return button.split(":", 1)[-1].strip() or "the hand-over failed"
        rig, tr, a = self.truth.rig, self.truth, self.slot
        mark = motion.piece.line_id if motion.piece is not None else ""
        if mark not in tr.marks:
            return f"no mark {mark!r} in the simulated world"
        arm = rig.arm(a)
        # the hand's orientation in the table frame, as the true arm holds it at the hover
        R_table = (tr.T[a] @ arm.fk(np.asarray(q_now, float)[None])[0])[:3, :3]
        g = np.random.default_rng(_seed("guide", a, self.count))
        off = g.normal(size=3)
        off[2] = 0.0
        off = self.error * off / max(np.linalg.norm(off), 1e-12)
        tip_t = np.array([*tr.marks[mark], tr.paper_z]) + off
        T_tb = np.linalg.inv(tr.T[a])
        T = np.eye(4)
        T[:3, :3] = T_tb[:3, :3] @ R_table
        T[:3, 3] = T_tb[:3, :3] @ (tip_t - R_table @ tr.tip[a]) + T_tb[:3, 3]
        # the person keeps the arm's shape: the answer nearest the hover, joint 7 free nearby
        q7s = q_now[6] + np.linspace(-0.3, 0.3, 13)
        Q, ok = arm.ik(np.repeat(T[None], len(q7s), axis=0), q7s)
        if not ok.any():
            return "the person cannot seat the pen on the mark from this hover"
        flat, good = Q.reshape(-1, 7), ok.reshape(-1)
        d = np.where(good, np.linalg.norm(np.nan_to_num(flat - q_now, nan=1e9), axis=1), np.inf)
        return flat[int(np.argmin(d))], button
