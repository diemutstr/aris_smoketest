"""The simulated person of the mark job (the meeting of two pen tips), and the "true" rig.

The station believes the rig it loaded; the simulated world can differ: another config
directory (`aris serve --sim-truth <dir>`: its base poses, pen tips and mark positions are the
truth), or the believed rig with every base moved by a fixed amount (`--sim-base-error
mm,mrad`: each slot's base shifted that much horizontally and turned that much about the
vertical, in a direction of its own that follows from its name, so the same run gives the
same truth; x, y and yaw are what the marks find, height and tilt are the plane job's).
`--sim-mark-error cm` moves every true mark that far from its nominal position, in a
direction of its own: marks are taped by hand, 2 to 5 cm off is normal.

The person: see `Person`.
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
    """`person(motion, q)` for one simulated arm (`SimArm(person=...)`): the person of the mark
    job's meeting.  Both arms of a row are guided so that their TRUE pen tips meet at one TRUE
    point: the spot's nominal place (the guide's `piece`), MEET_HEIGHT above the true paper,
    0.3 mm off (GUIDING_ERROR, seeded), the hand kept as the true arm holds it at the hover.
    The answer is "check" unless a scripted list says otherwise ("fail: <why>" fails the
    hand-over)."""

    def __init__(self, truth: Truth, slot, buttons=(), error: float = GUIDING_ERROR):
        self.truth, self.slot = truth, slot
        self.buttons, self.error = list(buttons), float(error)
        self.count, self._lock = 0, threading.Lock()

    def _button(self) -> str:
        with self._lock:
            self.count += 1
            return self.buttons.pop(0) if self.buttons else "check"

    def __call__(self, motion, q_now):
        from aris.server.mark import MEET_HEIGHT
        button = self._button()
        if button.startswith("fail"):
            return button.split(":", 1)[-1].strip() or "the hand-over failed"
        rig, tr, a = self.truth.rig, self.truth, self.slot
        spot = motion.piece.line_id if motion.piece is not None else ""
        if spot not in rig.marks:
            return f"no spot {spot!r} on this rig"
        arm = rig.arm(a)
        R_table = (tr.T[a] @ arm.fk(np.asarray(q_now, float)[None])[0])[:3, :3]
        g = np.random.default_rng(_seed("guide", a, self.count))
        off = g.normal(size=3)
        off = self.error * off / max(np.linalg.norm(off), 1e-12)
        tip_t = np.array([*np.asarray(rig.marks[spot][0], float),
                          tr.paper_z + MEET_HEIGHT]) + off
        T_tb = np.linalg.inv(tr.T[a])
        q7s = q_now[6] + np.linspace(-0.6, 0.6, 25)
        for turn in np.deg2rad([0.0, 10.0, -10.0, 20.0, -20.0, 30.0, -30.0]):
            Rt = _rot([0.0, 0.0, turn]) @ R_table
            T = np.eye(4)
            T[:3, :3] = T_tb[:3, :3] @ Rt
            T[:3, 3] = T_tb[:3, :3] @ (tip_t - Rt @ tr.tip[a]) + T_tb[:3, 3]
            Q, ok = arm.ik(np.repeat(T[None], len(q7s), axis=0), q7s)
            if not ok.any():
                continue
            flat, good = Q.reshape(-1, 7), ok.reshape(-1)
            d = np.where(good, np.linalg.norm(np.nan_to_num(flat - q_now, nan=1e9), axis=1),
                         np.inf)
            return flat[int(np.argmin(d))], button
        return "the person cannot bring the pen tip to the meeting point from this hover"
