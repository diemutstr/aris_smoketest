"""The sequencer: from the local planner's bunches to the tour of one arm.

    tour(arm, bunches, q_start, obstacles, rules, q_end=None)
        -> yields Motion, returns list[Leftover]

Greedy.  From where the arm is, every piece still to draw, in each of its alternatives and each
direction, is priced by a cheap estimate of the move to its first drawing configuration
(`price`).  The cheapest is tried: its lift-offs and its drawing motion are made (and kept for
later), then the free-space planner is asked for the move.  If anything refuses, the next
cheapest is tried.  Nothing is priced by running the free-space planner (lesson L75).

Per piece, four motions, each timed and ending where the arm can stand:
    free   the move from where the arm is to the piece's first lift-off configuration
    lower  the set-down: the lift-off flown backwards, the pen onto the paper
    draw   the piece, one motion
    lift   the lift-off at its end
and at the end one free move to `q_end` (default `q_start`).  See docs/modules/sequencer.md.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field, replace

import numpy as np

from aris import free
from aris.sequencer.drag import pulled_shares
from aris.sequencer.draw import draw_motions, oriented
from aris.sequencer.guard import Guard
from aris.sequencer.lift import end_lift, trim
from aris.types import Bunch, DrawRules, Leftover, Motion, Obstacles, Piece, Plane, Refusal


@dataclass(frozen=True)
class TourOptions:
    max_tries: int = 12            # refused free-space moves in one step before giving up there
    lift_step: float = 0.002       # m of pen rise between IK samples of a lift-off
    lift_jump: float = 0.15        # rad, the most a joint path may move per step (a larger jump
                                   # is a change of arm shape; next to a fold of the IK a lift
                                   # moves 30 to 90 rad/m, continuously)
    lift_extra: float = 0.002      # m above the pen clearance: the lift height (rule 1)
    # (hand spin about the normal, joint 7) change over a whole lift, rad, tried smallest first;
    # each within 0.5 rad
    lift_turns: tuple = tuple(sorted(
        ((a, b) for a in (0.0, 0.1, -0.1, 0.25, -0.25, 0.5, -0.5)
         for b in (0.0, 0.1, -0.1, 0.25, -0.25, 0.5, -0.5)),
        key=lambda t: (abs(t[0]) + abs(t[1]), abs(t[1]))))
    cut_step: float = 0.01         # m a piece is shortened at an end per try (rule 2)
    max_cut: float = 0.05          # m, the most it is shortened
    draw_deviation: float = 1.5e-4  # rad, how far timing may round the drawing path's corners


@dataclass
class TourReport:
    """What one tour did.  Filled in as the tour runs."""
    pieces: int = 0                # pieces drawn
    lifts: int = 0                 # pen lifts (one lift-off per piece drawn)
    draw_time: float = 0.0         # s, pen on the paper
    penup_time: float = 0.0        # s, every other motion: moves, set-downs, lift-offs
    move_time: float = 0.0         # s, of which the moves between lift-offs (and home)
    longest_free: float = 0.0      # s, the longest free motion
    free_length: float = 0.0       # rad, joint-space length of the moves between lift-offs
    free_calls: int = 0            # free-space planner calls
    cut: float = 0.0               # m cut off pieces because rule 1 failed at an end (rule 2)
    drag_notes: list = field(default_factory=list)  # drag-only pens: (piece, share pulled) of
                                   # pieces drawn though neither direction is pulled throughout
    drag_alternatives: dict = field(default_factory=dict)  # drag-only: alternatives whose
                                   # directions are "both" pulled, "one", "none" (wholly)
    drag_drawn: dict = field(default_factory=dict)  # the same, over the alternatives drawn
    refusals: dict = field(default_factory=dict)   # free-space refusals by reason
    motions: int = 0
    end_refusal: str = ""          # why the move to q_end failed; "" if it did not
    verified: int = 0              # motions given to `verify`
    failed_check: int = 0          # pieces left over because `verify` refused a motion
    verify_wall: float = 0.0       # s spent inside `verify` (wall; it may run elsewhere)
    first_cpu: float = -1.0        # s of CPU from the call to the first motion
    first_wall: float = -1.0
    cpu: float = 0.0               # s of CPU for the whole tour
    wall: float = 0.0

    @property
    def penup_share(self) -> float:
        total = self.draw_time + self.penup_time
        return self.penup_time / total if total > 0 else 0.0


def price(q_from: np.ndarray, Q_to: np.ndarray, rules: DrawRules, qd_max: np.ndarray):
    """Cheap estimate of the move from q_from to each row of Q_to: seconds the slowest joint
    needs at the speed allowed, (M,)."""
    return np.max(np.abs(Q_to - q_from) / (rules.speed_fraction * qd_max), axis=1)


def tour(arm, bunches: list[Bunch], q_start, obstacles: Obstacles, rules: DrawRules,
         q_end=None, *, report: TourReport | None = None, intensity: dict | None = None,
         verify=None,
         batches=None, refill: int = 32):
    """Yields the tour's motions in order; returns the pieces it could not draw.

    `batches`: an iterator of further lists of bunches (after `bunches`).  The next batch is
    taken, waiting for it if need be, whenever fewer than `refill` pieces are alive.  That
    depends on the pieces only, never on when a batch arrives, so the tour is the same however
    fast the batches come.

    `verify(motion, q_before) -> dict` (at least "passed", "tightest"): the independent
    checker.  With it, a piece is handed on only if every motion of its group (the move to it,
    lower, drawing, lift) passes, checked in order, each from where the one before ended; the
    first refusal leaves the piece over as "failed_check" and the tour goes on from where the
    arm stands.  Every motion handed on then carries the checker's dict as `checked`.  Nothing
    is ever planned on top of a motion that has not passed."""
    c0, w0 = time.process_time(), time.perf_counter()
    rep = report if report is not None else TourReport()
    s = _State(arm, bunches, obstacles, rules, TourOptions(), intensity or {}, rep)
    q_cur = np.array(q_start, float)
    q_end = q_cur.copy() if q_end is None else np.array(q_end, float)
    alive = list(range(len(bunches)))
    leftovers: list[Leftover] = []
    more = iter(()) if batches is None else iter(batches)

    def refill_alive():
        nonlocal more
        while more is not None and len(alive) < refill:
            batch = next(more, None)
            if batch is None:
                more = None
            else:
                alive.extend(s.add(batch))

    def out(m: Motion):
        _count(rep, m)
        if rep.first_cpu < 0:
            rep.first_cpu, rep.first_wall = time.process_time() - c0, time.perf_counter() - w0
        return m

    while True:
        refill_alive()
        if not alive:
            break
        found, refused, dead = s.step(q_cur, alive)
        for b, why in dead + [(b, s.refused_why(b, refused)) for b in refused if found is None]:
            leftovers.append(s.leftover(b, why))
            alive.remove(b)
        if found is None:
            continue
        b, move, entry, draw, exit_, cut_off = found
        alive.remove(b)
        leftovers += cut_off
        rep.cut += sum(x.piece.s1 - x.piece.s0 for x in cut_off)
        group = [m for m in (move, entry.down, *draw, exit_.up) if len(m.traj.t) > 1]
        if verify is not None:
            group, why = _verified(group, q_cur, verify, rep)
            if group is None:
                leftovers.append(Leftover(draw[0].piece if len(draw) == 1 else
                                          Piece(draw[0].piece.line_id,
                                                min(d.piece.s0 for d in draw),
                                                max(d.piece.s1 for d in draw)),
                                          "failed_check", why))
                rep.failed_check += 1
                continue
        rep.pieces += 1
        if rules.drag_only:
            kind = s.drag_kind[(b, s.chosen_plan)]
            rep.drag_drawn[kind] = rep.drag_drawn.get(kind, 0) + 1
        if (b, s.chosen_plan) in s.drag_note:
            rep.drag_notes.append((s.bunches[b].piece, s.drag_note[(b, s.chosen_plan)]))
        rep.lifts += 1
        rep.move_time += float(move.traj.t[-1])
        rep.free_length += _length(move)
        for m in group:
            yield out(m)
        q_cur = exit_.q_up
    home = s.move(q_cur, q_end)
    if not isinstance(home, Refusal) and len(home.traj.t) > 1 and verify is not None:
        checked, why = _verified([home], q_cur, verify, rep)
        if checked is None:
            home = Refusal("failed_check", f"the move to q_end: {why}")
        else:
            home = checked[0]
    if isinstance(home, Refusal):
        rep.end_refusal = f"{home.reason}: {home.detail}"
    elif len(home.traj.t) > 1:                  # a move of zero length is no motion
        rep.move_time += float(home.traj.t[-1])
        rep.free_length += _length(home)
        yield out(home)
    rep.cpu, rep.wall = time.process_time() - c0, time.perf_counter() - w0
    return leftovers


_ROLE = {"free": "move", "lower": "lower", "draw": "drawing", "lift": "lift"}


def _verified(group, q_before, verify, rep):
    """The group's motions with the checker's word attached, each checked from where the one
    before ended; or (None, which motion was refused and the checker's tightest)."""
    out, q = [], np.asarray(q_before, float)
    for i, m in enumerate(group):
        w = time.perf_counter()
        word = verify(m, q)
        rep.verify_wall += time.perf_counter() - w
        rep.verified += 1
        if not word.get("passed", False):
            return None, (f"the {_ROLE.get(m.kind, m.kind)} ({m.kind} motion {i + 1} of "
                          f"{len(group)} of the piece's group): {word.get('tightest', '')}")
        out.append(replace(m, checked=word))
        q = m.q_end
    return out, ""


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

    def __init__(self, arm, bunches, obstacles, rules, opt, intensity, rep):
        self.arm, self.bunches, self.obstacles, self.rules = arm, bunches, obstacles, rules
        self.opt, self.intensity, self.rep = opt, intensity, rep
        self.guard = Guard(arm, obstacles, rules.gates)
        papers = [p for p in obstacles.planes if p.kind == "paper"]
        if len(papers) != 1:
            raise ValueError(f"the obstacles must hold exactly one paper plane, not {len(papers)}")
        self.paper: Plane = papers[0]
        self.chosen_plan = -1          # the alternative of the piece found last
        self.drag_note: dict = {}      # (b, p) -> share pulled, where neither direction is pulled
        self.drag_kind: dict = {}      # (b, p) -> "both", "one", "none" (drag-only pens)
        self.lifts: dict = {}          # (b, p, end 0/1) -> (Lift, cut) | str
        self.draws: dict = {}          # (b, p, backwards) -> [Motion] | str
        self.dead: dict = {}           # (b, p, d) -> why
        self.bunches, self.cands, self.entry = [], [], np.zeros((0, 7))
        self.add(bunches)

    def add(self, bunches) -> list[int]:
        """More bunches (a batch); -> their indices."""
        first = len(self.bunches)
        self.bunches.extend(bunches)
        new = [(b, p, d) for b in range(first, len(self.bunches))
               for p in range(len(self.bunches[b].plans)) for d in (0, 1)]
        self.cands += new
        self.entry = np.concatenate([self.entry, np.array(
            [self.bunches[b].plans[p].q[-1 if d else 0] for b, p, d in new]).reshape(-1, 7)])
        if self.rules.drag_only:
            for b in range(first, len(self.bunches)):
                for p in range(len(self.bunches[b].plans)):
                    self._drag(b, p)
        return list(range(first, len(self.bunches)))

    def _drag(self, b, p) -> None:
        """Drag-only pens: only the direction(s) in which the pen is pulled stay candidates.  If
        neither is pulled over the whole piece, the one pulled over more of it stays, noted."""
        plan = self.bunches[b].plans[p]
        fwd, back = pulled_shares(self.arm, plan, self.paper.normal)
        whole = 1.0 - 1e-9
        keep = [d for d, f in ((0, fwd), (1, back)) if f >= whole] or [0 if fwd >= back else 1]
        if max(fwd, back) < whole:
            self.drag_note[(b, p)] = max(fwd, back)
        kind = "both" if len(keep) == 2 else ("one" if max(fwd, back) >= whole else "none")
        self.drag_kind[(b, p)] = kind
        self.rep.drag_alternatives[kind] = self.rep.drag_alternatives.get(kind, 0) + 1
        for d in (0, 1):
            if d not in keep:
                self.dead[(b, p, d)] = (f"drag-only pen: pushed over "
                                        f"{1 - (fwd, back)[d]:.0%} of the piece this way")

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
            entry, draw, exit_, cut_off = made
            move = self.move(q_cur, entry.q_up)
            if isinstance(move, Refusal):
                refused.setdefault(b, []).append(f"{move.reason}: {move.detail}")
                tries += 1
                if tries >= self.opt.max_tries:
                    break
                continue
            self.chosen_plan = p
            return (b, move, entry, draw, exit_, cut_off), refused, \
                self._dead_pieces(live, {b})
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
        m = free.plan(self.arm, q_a, q_b, self.obstacles, self.rules, self.rules.gates)
        self.rep.free_calls += 1
        if isinstance(m, Refusal):
            self.rep.refusals[m.reason] = self.rep.refusals.get(m.reason, 0) + 1
            return m
        why = self.guard.hold(m.q_end, touching=False)
        return m if why is None else Refusal("blocked", f"cannot hold the end: {why}")

    def _end(self, b, p, end):
        """The lift-off at one end (0: the plan's first sample, 1: its last) of an alternative,
        by the two rules (lift.py): -> (Lift, metres cut at that end) or why not."""
        key = (b, p, end)
        if key not in self.lifts:
            self.lifts[key] = end_lift(self.arm, self.guard, self.paper,
                                       self.bunches[b].plans[p], end, self.rules, self.opt)
        return self.lifts[key]

    def prepare(self, b, p, d):
        """-> (entry Lift, drawing motions, exit Lift, leftovers cut off), or why this candidate
        can never be flown."""
        entry, exit_ = self._end(b, p, d), self._end(b, p, 1 - d)
        for name, x in (("start", entry), ("end", exit_)):
            if isinstance(x, str):
                return f"no lift-off at its {name}: {x}"
        (entry, cut_in), (exit_, cut_out) = entry, exit_
        plan = self.bunches[b].plans[p]
        whole = plan.piece
        for end, cut in ((d, cut_in), (1 - d, cut_out)):
            if cut > 0.0:
                plan = trim(plan, end, cut)
                if plan is None or plan.piece.s1 - plan.piece.s0 < self.rules.min_piece:
                    return "no lift-off: nothing left after cutting both ends"
        cut_off = [Leftover(Piece(whole.line_id, a, z), "no_free_path",
                            "cut off at the end of a piece: no straight lift-off there")
                   for a, z in ((whole.s0, plan.piece.s0), (plan.piece.s1, whole.s1)) if z > a]
        key = (b, p, d)
        if key not in self.draws:
            self.draws[key] = draw_motions(self.arm, self.guard, oriented(plan, bool(d)),
                                          self.rules, self.opt.draw_deviation,
                                          self.intensity.get(plan.piece.line_id, 1.0))
        draw = self.draws[key]
        if isinstance(draw, str):
            return f"drawing: {draw}"
        for m in (*draw, entry.down):
            why = self.guard.hold(m.q_end, touching=True)
            if why is not None:
                return f"drawing: cannot hold the pen on the paper: {why}"
        return entry, draw, exit_, cut_off
