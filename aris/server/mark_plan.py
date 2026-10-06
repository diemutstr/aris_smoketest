"""The mark job's planning (mark.py runs the job): which touches an arm makes, the hover for
each touch and orientation, and the arm's whole sequence of checked motions.

For each touch a free move to the hover (the pen HOVER above the mark, `rig.mark_xy`), by way
of WAYPOINT_UP higher when turning the pen near the paper is cramped, then a `guide` standing
there; after the last touch, home.  The pivot's orientations: upright, and tilts of LEAN toward
AZIMUTHS (nudged by up to 60 degrees where out of reach), the hand's turn about the pen free;
of all hand turns and arm shapes that pass the gates at the hover and seated on the mark, the
one nearest where the arm is.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from aris.check import check
from aris.execute.queue import verdict_numbers
from aris.free import plan as free_plan
from aris.sequencer.guard import Guard
from aris.server.steps import Scene, lift_pens, pen_down
from aris.types import Motion, Piece, Refusal, Trajectory

HOVER = 0.030                    # m, the pen above the mark at the hover
LEAN = np.deg2rad(30.0)        # the pivot tilts the hand: turning alone leaves the pen length free
AZIMUTHS = np.deg2rad(np.arange(5) * 72.0)      # the pivot's five tilts, 72 degrees apart ...
NUDGES = np.deg2rad([0.0, 30.0, -30.0, 60.0, -60.0])   # ... each turned this much if unreachable
SPINS = np.arange(24) * np.deg2rad(15.0)
Q7_TRIES = np.deg2rad([0.0, 15.0, -15.0, 30.0, -30.0, 45.0, -45.0])
WAYPOINT_UP = 0.08               # m: turning the pen near the paper goes by way of this high


@dataclass(frozen=True)
class Task:
    mark: str
    orientation: int             # 0 upright; 1..3 the pivot's leans

    @property
    def lean(self):
        """(tx, ty) at hand spin 0: no lean for the upright touch, else LEAN toward its tilt's
        azimuth (`ArmPlan.hover` turns it with the spin, keeping the tilt's direction)."""
        if self.orientation == 0:
            return (0.0, 0.0)
        a = AZIMUTHS[self.orientation - 1]
        return (LEAN * np.cos(a), LEAN * np.sin(a))


def slot_marks(rig, slot, slots) -> list[str]:
    """The marks a slot touches in a job of `slots`: those it shares within the job; when it
    shares fewer than two there (a single slot, or a group cut through a pair), every mark it
    shares with any controlled slot, whose places an earlier run must have solved."""
    inside = [m for m in rig.marks_for(slots) if slot in rig.marks[m][1]]
    if len(inside) >= 2:
        return inside
    return [m for m in rig.marks if slot in rig.marks[m][1]
            and set(rig.marks[m][1]) <= set(rig.arm_ids)]


def tasks_for(rig, slot, slots, first: str | None = None) -> list[Task]:
    """This slot's touches: six orientations at its first mark (the pivot; `first`, else the
    first in rig.json order), one at each other mark."""
    marks = slot_marks(rig, slot, slots)
    if first in marks:
        marks.remove(first)
        marks.insert(0, first)
    out = []
    for k, m in enumerate(marks):
        out += [Task(m, o) for o in (range(1 + len(AZIMUTHS)) if k == 0 else (0,))]
    return out


PIVOT_TILTS_MIN = 3      # tilted orientations a pivot needs (turns alone leave the tip free)


def choose_pivot(st, slot, now, slots) -> list[Task]:
    """The slot's touches with a first mark whose tilted hovers the arm can reach (at least
    PIVOT_TILTS_MIN of them): the marks in rig.json order, the first that works."""
    rig = st.rig
    marks = slot_marks(rig, slot, slots)
    obs, standing, phase = Scene(rig).of(slot, now, (slot,), (), "pivot")
    ap, q = ArmPlan(st, slot, obs, standing, phase), np.asarray(now[slot], float)
    for m in marks:
        tilts = [Task(m, o) for o in range(1, 1 + len(AZIMUTHS))]
        if sum(ap.hover(t, q) is not None for t in tilts) >= PIVOT_TILTS_MIN:
            return tasks_for(rig, slot, slots, first=m)
    return tasks_for(rig, slot, slots)


@dataclass
class ArmPlan:
    """What one arm needs to plan its motions: the scene as it stands."""
    st: object
    slot: str
    obs: object
    standing: dict
    phase: object

    def __post_init__(self):
        rig = self.st.rig
        self.arm = rig.arm(self.slot)
        self.rules = self.st.rules
        self.guard = Guard(self.arm, self.obs, self.rules.gates)
        self.paper = rig.paper(self.slot, for_planning=True)

    def _tip_base(self, mark, height):
        rig = self.st.rig
        xy = rig.mark_xy(mark)
        T = rig.T_base_table(self.slot)
        return T[:3, :3] @ np.array([xy[0], xy[1], rig.paper_z + height]) + T[:3, 3]

    def hover(self, task: Task, q_near) -> np.ndarray | None:
        """A hover for this mark and orientation: the pen upright, or tilted LEAN toward the
        orientation's azimuth (turned by up to 60 degrees when that is out of reach); over the
        hand's turns about the pen (15 degrees apart) and arm shapes whose hover and seated
        poses pass the gates, the one nearest `q_near` (the arm keeps its shape from touch to
        touch, so the moves between them stay short)."""
        if task.orientation == 0:
            return self._nearest(task.mark, lambda spin: (0.0, 0.0), q_near)
        base = AZIMUTHS[task.orientation - 1]
        for nudge in NUDGES:
            a = base + nudge
            got = self._nearest(task.mark, lambda spin: (LEAN * np.cos(a - spin),
                                                          LEAN * np.sin(a - spin)), q_near)
            if got is not None:
                return got
        return None

    def _nearest(self, mark, lean_at, q_near) -> np.ndarray | None:
        arm, g, gates = self.arm, self.guard, self.rules.gates
        up, seat = self._tip_base(mark, HOVER), self._tip_base(mark, 0.0)
        q7s = self.st.rig.park_q(self.slot)[6] + Q7_TRIES
        best = None
        for spin in SPINS:
            lean = np.array([lean_at(spin)])
            Th = arm.hand_pose(up[None], self.paper.normal, np.array([spin]), lean)
            Ts = arm.hand_pose(seat[None], self.paper.normal, np.array([spin]), lean)
            Qh, okh = arm.ik(np.repeat(Th, len(q7s), axis=0), q7s)
            Qs, oks = arm.ik(np.repeat(Ts, len(q7s), axis=0), q7s)
            for i in range(len(q7s)):
                for b in np.flatnonzero(okh[i] & oks[i]):
                    h, s = Qh[i, b], Qs[i, b]
                    if (arm.limit_margin(np.stack([h, s])).min() < gates.limit_margin
                            or arm.sigma_min(np.stack([h, s])).min() < gates.sigma_min
                            or g.hold(h, touching=False) is not None
                            or g.hold(s, touching=True) is not None):
                        continue
                    d = float(np.linalg.norm(h - q_near))
                    if best is None or d < best[0]:
                        best = (d, h)
        return None if best is None else best[1]

    def go_to(self, q_from, task: Task) -> list | str:
        """[free move to the hover (with its verdict), guide] or why not."""
        h = self.hover(task, q_from)
        if h is None:
            return f"no hand pose seats the pen on mark {task.mark} (orientation " \
                   f"{task.orientation}) inside the gates"
        legs = self._way(q_from, h)
        if isinstance(legs, str):
            return f"no way to the hover of {task.mark}: {legs}"
        out, q = [], q_from
        for m in legs:
            v = check(self.st.config_dir, self.slot, m, self.phase, q, standing=self.standing)
            if not v.passed:
                return f"the way to the hover of {task.mark} fails the checker: " + \
                    ", ".join(v.failed)
            out.append((m, v))
            q = m.q_end
        # the guide stands at the hover, which the checker held as the last move's end
        g = Motion("guide", Trajectory(np.zeros(1), q[None], np.zeros((1, 7))),
                   Piece(task.mark, 0.0, 0.0), self._tip_base(task.mark, 0.0)[None],
                   checked=dict(verdict_numbers(out[-1][1]), tightest="the hover, held at "
                                "the free move's end"))
        return out + [(g, None)]

    def _up(self, q, rise: float):
        """The same hand pose `rise` higher along the paper normal, the nearest answer."""
        arm = self.arm
        n = np.asarray(self.paper.normal, float) / np.linalg.norm(self.paper.normal)
        T = arm.fk(np.asarray(q, float)[None])[0]
        T[:3, 3] += rise * n
        q7s = q[6] + np.linspace(-0.3, 0.3, 7)
        Q, ok = arm.ik(np.repeat(T[None], len(q7s), axis=0), q7s)
        flat, good = Q.reshape(-1, 7), ok.reshape(-1)
        order = np.argsort(np.where(good, np.linalg.norm(np.nan_to_num(flat - q, nan=1e9),
                                                         axis=1), np.inf))
        for k in order:
            if not good[k]:
                break
            if self.guard.hold(flat[k], touching=False) is None:
                return flat[k]
        return None

    def _way(self, q_from, h) -> list | str:
        """Free moves from q_from to h: straight there, or, close to the paper where turning
        the pen is cramped, up WAYPOINT_UP first, across, and down to the hover."""
        m = free_plan(self.arm, q_from, h, self.obs, self.rules, seed_extra=b"mark")
        if not isinstance(m, Refusal):
            return [m]
        a, b = self._up(q_from, WAYPOINT_UP), self._up(h, WAYPOINT_UP)
        if a is None or b is None:
            return f"{m.reason}: {m.detail}"
        legs, q = [], q_from
        for goal in (a, b, h):
            leg = free_plan(self.arm, q, goal, self.obs, self.rules, seed_extra=b"mark via")
            if isinstance(leg, Refusal):
                return f"{leg.reason}: {leg.detail} (also by way of {WAYPOINT_UP * 1e3:.0f} mm up)"
            legs.append(leg)
            q = leg.q_end
        return legs

    def home(self, q_from) -> list | str:
        """[(free move, verdict), ...] back to the park, or why not."""
        legs = self._way(q_from, self.st.rig.park_q(self.slot))
        if isinstance(legs, str):
            return f"no way home: {legs}"
        out, q = [], q_from
        for m in legs:
            v = check(self.st.config_dir, self.slot, m, self.phase, q, standing=self.standing)
            if not v.passed:
                return "the way home fails the checker: " + ", ".join(v.failed)
            out.append((m, v))
            q = m.q_end
        return out


def plan_arm(st, slot, now, tasks, name) -> tuple | str:
    """One arm's whole sequence, planned and checked up front: (a lift if its pen is at the
    paper,) for each task a free move to its hover and the guide, then home.  A task with no
    hover inside the gates or no way there is dropped and noted.
    -> (phase, standing, [(motion, verdict)], {queue index: Task}, notes) or why not."""
    rig = st.rig
    obs, standing, phase = Scene(rig).of(slot, now, (slot,), (), name)
    ap, items, at, notes = ArmPlan(st, slot, obs, standing, phase), [], {}, []
    cur = np.asarray(now[slot], float)
    if pen_down(rig, slot, cur):                     # a pen at the paper rises first
        up = lift_pens(st, Scene(rig), now, [slot])[0]
        if up.why:
            return up.why
        items += list(zip(up.motions, up.verdicts))
        cur = up.motions[-1].q_end
    for t in tasks:
        go = ap.go_to(cur, t)
        if isinstance(go, str):
            notes.append(f"{slot}: {go}")
            continue
        items += go
        at[len(items) - 1] = t
        cur = go[-1][0].q_end
    home = ap.home(cur)
    if isinstance(home, str):
        return home
    return phase, standing, items + home, at, notes
