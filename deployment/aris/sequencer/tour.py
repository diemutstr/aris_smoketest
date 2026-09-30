"""The sequencer: from the local planner's bunches to the tour of one arm.

    tour(arm, bunches, q_start, obstacles, rules, q_end=None, free_options=None)
        -> yields Motion, returns list[Leftover]

Greedy.  From where the arm is, every piece still to draw, in each of its alternatives and each
direction, is priced by a cheap estimate of the move to its first drawing configuration
(`price`).  The cheapest is tried: its lift-offs and its drawing motion are made (and kept for
later), then the free-space planner is asked for the move.  If anything refuses, the next
cheapest is tried.  Nothing is priced by running the free-space planner (lesson L75).

Per piece, four motions, each timed and ending where the arm can stand:
    free   the move from where the arm is to the piece's first lift-off configuration
    free   the set-down: the lift-off flown backwards, the pen onto the paper
    draw   the piece (two or more motions where the pen stops at a sharp corner anyway)
    free   the lift-off at its end
and at the end one free move to `q_end` (default `q_start`).  See docs/modules/sequencer.md.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np

from aris import free
from aris.sequencer.draw import draw_motions, oriented
from aris.sequencer.guard import Guard
from aris.sequencer.lift import Lift, lift
from aris.types import Bunch, DrawRules, Leftover, Motion, Obstacles, Plane, Refusal


@dataclass(frozen=True)
class TourOptions:
    max_tries: int = 12            # refused free-space moves in one step before giving up there
    lift_step: float = 0.002       # m of tip rise between IK samples of a lift-off
    lift_jump: float = 0.05        # rad; a larger joint change between two of them is a new shape
    draw_deviation: float = 1.5e-4  # rad, how far timing may round the drawing path's corners
    cut_angle: float = np.deg2rad(120.0)  # a drawing stops at a corner of the line sharper than this
    # (joint 7, hand about the paper normal) turns during a lift-off, rad, tried smallest first
    lift_turns: tuple = tuple(sorted(
        ((a, b) for a in (0.0, 0.1, -0.1, 0.2, -0.2, 0.35, -0.35, 0.5, -0.5)
         for b in (0.0, 0.25, -0.25, 0.5, -0.5)), key=lambda t: abs(t[0]) + abs(t[1])))


@dataclass
class TourReport:
    """What one tour did.  Filled in as the tour runs."""
    pieces: int = 0                # pieces drawn
    lifts: int = 0                 # pen lifts (one lift-off per piece drawn)
    draw_time: float = 0.0         # s, pen on the paper
    penup_time: float = 0.0        # s, every free motion: moves, set-downs, lift-offs
    move_time: float = 0.0         # s, of which the moves between lift-offs (and home)
    longest_free: float = 0.0      # s, the longest free motion
    free_length: float = 0.0       # rad, joint-space length of the moves between lift-offs
    free_calls: int = 0            # free-space planner calls
    refusals: dict = field(default_factory=dict)   # free-space refusals by reason
    motions: int = 0
    end_refusal: str = ""          # why the move to q_end failed; "" if it did not
    first_cpu: float = -1.0        # s of CPU from the call to the first motion
    first_wall: float = -1.0
    cpu: float = 0.0               # s of CPU for the whole tour
    wall: float = 0.0
    leftovers: list = field(default_factory=list)

    @property
    def penup_share(self) -> float:
        total = self.draw_time + self.penup_time
        return self.penup_time / total if total > 0 else 0.0


def price(q_from: np.ndarray, Q_to: np.ndarray, rules: DrawRules, qd_max: np.ndarray):
    """Cheap estimate of the move from q_from to each row of Q_to: seconds the slowest joint
    needs at the speed allowed, (M,)."""
    return np.max(np.abs(Q_to - q_from) / (rules.speed_fraction * qd_max), axis=1)


def tour_all(arm, bunches, q_start, obstacles, rules, q_end=None, free_options=None, **kw):
    """The whole tour at once: -> (motions, leftovers, report)."""
    report = kw.pop("report", None) or TourReport()
    gen = tour(arm, bunches, q_start, obstacles, rules, q_end, free_options, report=report, **kw)
    motions = []
    while True:
        try:
            motions.append(next(gen))
        except StopIteration as stop:
            return motions, stop.value, report


def tour(arm, bunches: list[Bunch], q_start, obstacles: Obstacles, rules: DrawRules,
         q_end=None, free_options=None, *, report: TourReport | None = None,
         intensity: dict | None = None, options: TourOptions | None = None):
    """Yields the tour's motions in order; returns the pieces it could not draw."""
    c0, w0 = time.process_time(), time.perf_counter()
    rep = report if report is not None else TourReport()
    s = _State(arm, bunches, obstacles, rules, free_options or free.Options(),
               options or TourOptions(), intensity or {}, rep)
    q_cur = np.array(q_start, float)
    q_end = q_cur.copy() if q_end is None else np.array(q_end, float)
    alive = list(range(len(bunches)))
    leftovers: list[Leftover] = []

    def out(m: Motion):
        _count(rep, m)
        if rep.first_cpu < 0:
            rep.first_cpu, rep.first_wall = time.process_time() - c0, time.perf_counter() - w0
        return m

    while alive:
        found, refused, dead = s.step(q_cur, alive)
        for b, why in dead + [(b, s.refused_why(b, refused)) for b in refused if found is None]:
            leftovers.append(s.leftover(b, why))
            alive.remove(b)
        if found is None:
            continue
        b, move, entry, draw, exit_ = found
        alive.remove(b)
        rep.pieces += 1
        rep.lifts += 1
        rep.move_time += float(move.traj.t[-1])
        rep.free_length += _length(move)
        for m in (move, entry.down, *draw, exit_.up):
            yield out(m)
        q_cur = exit_.q_up
    home = s.move(q_cur, q_end)
    if isinstance(home, Refusal):
        rep.end_refusal = f"{home.reason}: {home.detail}"
    elif len(home.traj.t) > 1:                  # a move of zero length is no motion
        rep.move_time += float(home.traj.t[-1])
        rep.free_length += _length(home)
        yield out(home)
    rep.leftovers = leftovers
    rep.cpu, rep.wall = time.process_time() - c0, time.perf_counter() - w0
    return leftovers


def _count(rep: TourReport, m: Motion) -> None:
    d = float(m.traj.t[-1])
    rep.motions += 1
    if m.kind == "draw":
        rep.draw_time += d
    else:
        rep.penup_time += d
        rep.longest_free = max(rep.longest_free, d)


def _length(m: Motion) -> float:
    return float(np.linalg.norm(np.diff(m.traj.q, axis=0), axis=1).sum())


class _State:
    """What the tour knows about every candidate: made once, kept for later steps.

    A candidate is (bunch, alternative, direction).  Its lift-offs and drawing motion do not
    depend on where the arm is, so a failure there kills it for good; a refused move only
    counts from where the arm was."""

    def __init__(self, arm, bunches, obstacles, rules, fopt, opt, intensity, rep):
        self.arm, self.bunches, self.obstacles, self.rules = arm, bunches, obstacles, rules
        self.fopt, self.opt, self.intensity, self.rep = fopt, opt, intensity, rep
        self.guard = Guard(arm, obstacles, rules.gates)
        papers = [p for p in obstacles.planes if p.kind == "paper"]
        if len(papers) != 1:
            raise ValueError(f"the obstacles must hold exactly one paper plane, not {len(papers)}")
        self.paper: Plane = papers[0]
        self.lifts: dict = {}          # (b, p, end 0/1) -> Lift | str
        self.draws: dict = {}          # (b, p, backwards) -> [Motion] | str
        self.dead: dict = {}           # (b, p, d) -> why
        self.cands = [(b, p, d) for b in range(len(bunches))
                      for p in range(len(bunches[b].plans)) for d in (0, 1)]
        self.entry = np.array([bunches[b].plans[p].q[-1 if d else 0] for b, p, d in self.cands]
                              ).reshape(-1, 7)

    # ------------------------------------------------------------------ one step

    def step(self, q_cur, alive):
        """-> (found or None, {bunch: [refusals]} of this step, [(bunch, why)] dead for good)."""
        live = set(alive)
        idx = [i for i, c in enumerate(self.cands) if c[0] in live and c not in self.dead]
        cost = price(q_cur, self.entry[idx], self.rules, self.arm.limits.qd_max)
        refused: dict = {}
        tries = 0
        for i in np.array(idx, int)[np.lexsort((idx, cost))] if idx else []:
            b, p, d = self.cands[i]
            made = self.prepare(b, p, d)
            if isinstance(made, str):
                self.dead[(b, p, d)] = made
                continue
            entry, draw, exit_ = made
            move = self.move(q_cur, entry.q_up)
            if isinstance(move, Refusal):
                refused.setdefault(b, []).append(f"{move.reason}: {move.detail}")
                tries += 1
                if tries >= self.opt.max_tries:
                    break
                continue
            return (b, move, entry, draw, exit_), refused, self._dead_pieces(live, {b})
        return None, refused, self._dead_pieces(live, set(refused))

    def _dead_pieces(self, live, skip):
        """Pieces all of whose candidates are dead for good."""
        out = []
        for b in sorted(live - skip):
            n = 2 * len(self.bunches[b].plans)
            whys = [w for (bb, _, _), w in self.dead.items() if bb == b]
            if len(whys) == n:
                out.append((b, whys[0]))
        return out

    def refused_why(self, b, refused) -> str:
        return f"refused from where the arm was, {len(refused[b])} tries; first: {refused[b][0]}"

    def leftover(self, b, why: str) -> Leftover:
        # A piece that cannot be flown to (no move, no lift-off) is "no_free_path"; one whose
        # drawing motion cannot be timed or flown is "unreachable", as in the local planner.
        reason = "unreachable" if why.startswith("drawing:") else "no_free_path"
        return Leftover(self.bunches[b].piece, reason, why)

    # ------------------------------------------------------------------ the parts

    def move(self, q_a, q_b) -> Motion | Refusal:
        m = free.plan(self.arm, q_a, q_b, self.obstacles, self.rules, self.rules.gates,
                      options=self.fopt)
        self.rep.free_calls += 1
        if isinstance(m, Refusal):
            self.rep.refusals[m.reason] = self.rep.refusals.get(m.reason, 0) + 1
            return m
        why = self.guard.hold(m.q_end, touching=False)
        return m if why is None else Refusal("blocked", f"cannot hold the end: {why}")

    def _lift(self, b, p, end) -> Lift | str:
        key = (b, p, end)
        if key not in self.lifts:
            plan = self.bunches[b].plans[p]
            self.lifts[key] = lift(self.arm, self.guard, self.paper, plan.q[-1 if end else 0],
                                   self.rules, self.opt.lift_step, self.opt.lift_jump,
                                   self.opt.lift_turns)
        return self.lifts[key]

    def prepare(self, b, p, d):
        """-> (entry Lift, drawing motions, exit Lift), or why this candidate can never be flown."""
        entry, exit_ = self._lift(b, p, d), self._lift(b, p, 1 - d)
        for name, x in (("start", entry), ("end", exit_)):
            if isinstance(x, str):
                return f"no lift-off at its {name}: {x}"
        key = (b, p, d)
        if key not in self.draws:
            plan = oriented(self.bunches[b].plans[p], bool(d))
            self.draws[key] = draw_motions(self.arm, self.guard, plan, self.rules,
                                          self.opt.draw_deviation, self.opt.cut_angle,
                                          self.intensity.get(plan.piece.line_id, 1.0))
        draw = self.draws[key]
        if isinstance(draw, str):
            return f"drawing: {draw}"
        for m in (*draw, entry.down):
            why = self.guard.hold(m.q_end, touching=True)
            if why is not None:
                return f"drawing: cannot hold the pen on the paper: {why}"
        return entry, draw, exit_
