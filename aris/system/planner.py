"""The system planner: a whole drawing -> which arm draws which line, in which phase, behind
which walls; runs the arm planners and hands out their motions.

    plan(rig, lines, rules, arm_configs=None, cache_dir=None, workers=1)
        -> yields (phase name, arm id, Motion), returns list[Leftover]

The leaders of a phase draw, their row partners stand parked (docs/DESIGN.md section 3).

1. The drawable maps of every (phase, arm) are read from `cache_dir` or built (maps.py).
2. Before each phase, everything still to draw that has no arm yet is allocated over this phase
   and the ones after it (allocate.py).  A stretch keeps the arm it was given.
3. The phase's arms plan in parallel (run.py), each from where it stands to its park; their
   motions are handed on as they arrive, tagged with the phase and the arm, their pieces in the
   input line's own arc length.
4. What an arm could not draw goes back into the pool for the phases after; "too short" is left
   over at once.  After the last phase the pool is the leftover list.

Every arm ends every phase at its park, so every phase starts from all arms parked.
See docs/modules/system.md.
"""
from __future__ import annotations

import os
import time
from dataclasses import dataclass, field, replace
from functools import partial

import numpy as np

from aris.system import area
from aris.system import maps as maps_mod
from aris.system.allocate import allocate
from aris.system.phases import is_fill
from aris.system.phases import phases as all_phases
from aris.system.run import ArmJob, run_phase
from aris.system.settings import Settings
from aris.system.stretch import Stretch, of_line
from aris.types import DrawRules, Leftover, Line, Motion, Piece, Slot



@dataclass
class ArmReport:
    arm_id: Slot
    stretches: int = 0          # stretches handed to the arm
    offered: float = 0.0        # m
    drawn: float = 0.0          # m, drawing motions produced
    motions: int = 0
    motion_time: float = 0.0    # s the arm moves in this phase
    cpu: float = 0.0            # s, the arm planner (its process and its children)
    wall: float = 0.0
    first_cpu: float = -1.0     # s from its start to its first motion, the arm planner's clock
    first_wall: float = -1.0
    first_phase_wall: float = -1.0   # s from the phase's start to its first motion, as received
    checked: int = 0            # motions handed on with the checker's word (`Motion.checked`);
    # the refused ones never arrive: their pieces come back as handed_back["failed_check"]
    handed_back: dict = field(default_factory=dict)   # reason -> m it could not draw
    stats: object = None        # arm_planner.PlanStats


@dataclass
class PhaseReport:
    name: str
    active: tuple
    arms: dict = field(default_factory=dict)   # arm id -> ArmReport
    wall: float = 0.0           # s, planning the whole phase
    idle: dict = field(default_factory=dict)   # arm id -> ArmReport of an arm that drew nothing

    @property
    def duration(self) -> float:
        """s the phase takes on the rig: its longest arm."""
        return max((r.motion_time for r in self.arms.values()), default=0.0)


@dataclass
class Report:
    coverage: dict = field(default_factory=dict)     # maps.coverage
    map_cpu: float = 0.0        # s, building the maps (0 when all came from the cache)
    map_wall: float = 0.0
    phases: list = field(default_factory=list)       # PhaseReport, in the order run
    cpu: float = 0.0            # s, everything (this process, arm planners, map builders)
    wall: float = 0.0
    first_wall: float = -1.0    # s from the call to the first motion
    drawing_centre: np.ndarray | None = None         # (2,) m, its centre (table frame)
    drawing_area: np.ndarray | None = None           # (2,) m, the maps' area about that centre
    # the tightest checked clearance over every motion (m beyond the demanded one), and where
    tightest: float = float("inf")
    tightest_at: str = ""
    cuts: int = 0               # joins made by cutting stretches (allocate)
    # Phases not run, and why: nothing was allocated to them, or their arms planned and drew
    # nothing (everything handed back).  [(phase name, why)]
    skipped: list = field(default_factory=list)

    @property
    def drawing_time(self) -> float:
        """s on the rig: the phases one after another."""
        return sum(p.duration for p in self.phases)


def _cpu() -> float:
    t = os.times()
    return t.user + t.system + t.children_user + t.children_system


def start_configs(rig, arm_configs, first_active) -> dict:
    """Where every arm stands.  An arm away from its park must move in the first phase, since
    every other phase assumes the arms it does not move stand parked."""
    q = {a: rig.park_q(a) for a in rig.arm_ids}
    for a, qa in (arm_configs or {}).items():
        if a not in q:
            raise ValueError(f"arm_configs names arm {a}, which is not on this rig")
        q[a] = np.asarray(qa, float).reshape(7)
        if not rig.at_park(a, q[a]) and a not in first_active:
            raise ValueError(f"arm {a} is not at its park and does not move in the first phase")
    return q


def plan(rig, lines, rules: DrawRules | None = None, arm_configs=None, cache_dir=None,
         workers: int = 1, settings: Settings | None = None, report: Report | None = None,
         verify=None, surface=None, rules_by_slot: dict | None = None):
    """Yields (phase name, arm id, Motion) as the arm planners produce them; returns the
    leftovers.
    `rules`: default `rig.rules()`, the only source; the maps and the allocation use it.
    `rules_by_slot`: {slot: DrawRules}, each arm's own (its pen: press, speed on the paper,
    drag only); a slot left out uses `rules`.
    `lines`: table-frame Lines with distinct ids.  `arm_configs`: arm id -> where it
    stands (default: its park).  `cache_dir`: where the drawable maps and the local planner's
    kinematic table are kept.  `workers`: processes (map building, arms of a phase).
    `verify(slot, phase, motion, q_before) -> dict`: the independent checker, picklable; each
    arm planner gets it with its slot and phase bound and hands on only motions that passed.
    None: nothing is checked here.
    `surface`: the measured paper (`calib.paper.Surface`, anything with `z(x, y)`): each point is
    drawn at `surface.z(x, y) - rules.press` instead of `paper_z - rules.press`.  The drawable
    maps and the clearances keep the flat paper."""
    cfg = settings or Settings()
    rules = rig.rules() if rules is None else rules
    rep = report if report is not None else Report()
    c0, w0 = _cpu(), time.perf_counter()
    phases = all_phases(rig)
    if len({x.id for x in lines}) != len(lines):
        raise ValueError("two lines share an id")
    q_now = start_configs(rig, arm_configs, phases[0].active)
    maps = maps_mod.load_or_build(rig, phases, rules.gates, cfg, cache_dir, workers, rules.press)
    rep.map_cpu, rep.map_wall = _cpu() - c0, time.perf_counter() - w0
    rep.coverage = maps_mod.coverage(maps, phases)
    # The drawing area is the server's business (it fits drawings into rig.json's area and
    # checks that against the maps when it starts).  Here it is only reported: a point no map
    # holds is left over as "unreachable", like any other.
    centre = getattr(rig, "drawing_area_centre_m", None)
    rep.drawing_centre = np.asarray((0.0, 0.0) if centre is None else centre, float).reshape(2)
    rep.drawing_area = area.admissible(maps, centre=rep.drawing_centre)
    # the drawing surface: every point `press` below the paper (the planners below put the tip
    # where the points are; the real paper stays the plane for clearances)
    z = rig.paper_z - rules.press
    pool, left = [of_line(on_surface(x, z)) for x in lines], []
    for k, ph in enumerate(phases):
        cands = [(j, a, maps[(phases[j].name, a)], not is_fill(phases[j]))
                 for j in range(k, len(phases)) for a in phases[j].active]
        pool, new_left, cuts = allocate(pool, cands, rules, cfg)
        left += new_left
        rep.cuts += cuts
        mine = {a: [s for s in pool if s.target == (k, a)] for a in ph.active}
        pool = [s for s in pool if s.target[0] != k]
        todo = [a for a in ph.active if mine[a] or not rig.at_park(a, q_now[a])]
        if not todo:
            rep.skipped.append((ph.name, "nothing allocated to it"))
            continue
        pool, final = yield from _phase(rig, phases, k, todo, mine, pool, q_now, rules,
                                        cfg, cache_dir, workers, rep, w0, verify, surface,
                                        rules_by_slot or {})
        left += final
        for a in todo:
            q_now[a] = rig.park_q(a)
    _, new_left, _ = allocate(pool, [], rules, cfg)
    rep.cpu, rep.wall = _cpu() - c0, time.perf_counter() - w0
    return left + new_left


def _phase(rig, phases, k, todo, mine, pool, q_now, rules, cfg, cache_dir, workers, rep,
           w0, verify, surface, by_slot):
    """Runs phase k.  Yields the tagged motions; returns (the pool for the phases after,
    leftovers)."""
    ph = phases[k]
    pr = PhaseReport(ph.name, ph.active)
    rep.phases.append(pr)
    t0 = time.perf_counter()
    jobs = {a: (rig.obstacles_for(a, ph), mine[a], q_now[a]) for a in todo}
    back, final = yield from _run(rig, ph, jobs, pr, rules, cache_dir, workers, rep, t0, w0,
                                  verify, surface, cfg, by_slot)
    pr.wall = time.perf_counter() - t0
    for a in [a for a, r in pr.arms.items() if r.motions == 0]:     # lesson L82: no EMPTY rows
        pr.idle[a] = pr.arms.pop(a)
    if not pr.arms:
        rep.phases.remove(pr)
        rep.skipped.append((ph.name, "nothing allocated to it" if not pr.idle else
                            "its arms drew nothing; handed back: " + ", ".join(
                                f"arm {a} {k} {m:.2f} m" for a, r in pr.idle.items()
                                for k, m in sorted(r.handed_back.items()))))
    return pool + back, final


def on_surface(line: Line, z: float) -> Line:
    """The line with every point at height z (the drawing surface); x and y as drawn."""
    p = np.array(line.points, float).reshape(-1, 3)
    p[:, 2] = z
    return replace(line, points=p)


def _run(rig, ph, jobs: dict, pr, rules, cache_dir, workers, rep, t0, w0, verify,
         surface=None, cfg: Settings | None = None, by_slot: dict | None = None):
    """The arm planners of `jobs` {slot: (obstacles, stretches, q_start)}, in parallel, each
    back to its park, each with `verify` bound to its slot and the phase.  Yields the tagged
    motions; returns (handed back, leftovers)."""
    subs, arm_jobs = {}, []
    for a, (obs, sts, q0) in jobs.items():
        pr.arms[a] = ArmReport(a, len(sts), float(sum(s.length for s in sts)))
        base = []
        own = (by_slot or {}).get(a, rules)
        for i, st in enumerate(sts):
            line, arcs = _on_paper(st, f"{st.line_id}#{i}", surface, own.press, cfg,
                                   rig.paper_z if own.press != rules.press else None)
            subs[(a, line.id)] = (st, arcs)
            base.append(rig.to_base(a, line))
        bound = None if verify is None else partial(verify, a, ph)
        arm_jobs.append(ArmJob(a, tuple(base), obs, q0, rig.park_q(a), bound,
                               None if own is rules else own))
    back, final = [], []
    for a, kind, payload in run_phase(rig, arm_jobs, rules, cache_dir, workers):
        ar = pr.arms[a]
        if kind == "motion":
            m = _retag(payload, subs, a)
            ar.motions += 1
            ar.motion_time += float(m.traj.t[-1] - m.traj.t[0])
            if m.kind == "draw":
                ar.drawn += m.piece.s1 - m.piece.s0
            if m.checked is not None:
                ar.checked += 1
                c = m.checked.get("min_clearance")
                if c is not None and c < rep.tightest:
                    rep.tightest = float(c)
                    rep.tightest_at = (f"{ph.name}, arm {a}, {m.kind} motion {ar.motions}: "
                                       f"{m.checked.get('tightest', '')}")
            if ar.first_phase_wall < 0:
                ar.first_phase_wall = time.perf_counter() - t0
            if rep.first_wall < 0:
                rep.first_wall = time.perf_counter() - w0
            yield ph.name, a, m
            continue
        leftovers, st = payload
        ar.stats, ar.cpu, ar.wall = st, st.cpu, st.wall
        ar.first_cpu, ar.first_wall = st.first_cpu, st.first_wall
        for x in leftovers:
            s, arcs = subs[(a, x.piece.line_id)]
            part = s.sub(_flat(s, arcs, x.piece.s0), _flat(s, arcs, x.piece.s1))
            ar.handed_back[x.reason] = ar.handed_back.get(x.reason, 0.0) + part.length
            detail = f"{ph.name}, arm {a}: {x.detail}"
            if x.reason == "too_short":
                final.append(Leftover(part.piece, x.reason, detail))
            else:
                back.append(replace(part, reason=x.reason, detail=detail))
    return back, final


def _retag(m: Motion, subs: dict, arm_id: Slot) -> Motion:
    """The motion with its piece in the input line's own id and arc length."""
    if m.piece is None:
        return m
    s, arcs = subs[(arm_id, m.piece.line_id)]
    a, b = (float(min(max(_flat(s, arcs, x), s.s0), s.s1)) for x in (m.piece.s0, m.piece.s1))
    return replace(m, piece=Piece(s.line_id, a, b))


def _on_paper(st: Stretch, line_id: str, surface, press: float, cfg, paper_z=None):
    """The stretch as the line its arm planner draws.  Without a surface: as it is (every point
    at paper_z - the planner's press), or, with `paper_z` (an arm whose pen presses otherwise),
    at paper_z - `press`.  With a surface: points at most `cfg.surface_step` apart along it,
    each at `surface.z(x, y) - press`, and the arc lengths (along this line, along the drawing)
    that take the planner's pieces back to the drawing's own arc length.
    -> (Line, arcs or None)."""
    if surface is None:
        line = st.as_line(line_id)
        if paper_z is None:
            return line, None
        p = np.array(line.points, float)
        p[:, 2] = paper_z - press
        return replace(line, points=p), None
    u, p = st.samples(cfg.surface_step)
    p = p.copy()
    p[:, 2] = np.asarray(surface.z(p[:, 0], p[:, 1]), float) - press
    s3 = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))])
    return Line(line_id, p, "table", st.intensity), (s3, u)


def _flat(st: Stretch, arcs, s_line: float) -> float:
    """Arc length along the drawing of a point `s_line` along the line the arm planner drew."""
    return st.s0 + s_line if arcs is None else float(np.interp(s_line, arcs[0], arcs[1]))


def plan_all(rig, lines, rules, arm_configs=None, cache_dir=None, workers=1, settings=None,
             verify=None, surface=None, rules_by_slot=None):
    """-> ([(phase name, arm id, Motion)], leftovers)."""
    motions, left, _ = plan_detailed(rig, lines, rules, arm_configs, cache_dir, workers, settings,
                                     verify, surface, rules_by_slot)
    return motions, left


def plan_detailed(rig, lines, rules, arm_configs=None, cache_dir=None, workers=1, settings=None,
                  verify=None, surface=None, rules_by_slot=None):
    """`plan_all`, plus the Report (maps, per phase and arm: times, lengths, what came back)."""
    rep = Report()
    gen = plan(rig, lines, rules, arm_configs, cache_dir, workers, settings, rep, verify, surface,
               rules_by_slot)
    out = []
    while True:
        try:
            out.append(next(gen))
        except StopIteration as stop:
            return out, stop.value, rep
