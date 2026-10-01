"""The system planner: a whole drawing -> which arm draws which line, in which phase, behind
which walls; runs the arm planners and hands out their motions.

    plan(rig, lines, rules, arm_configs=None, cache_dir=None, workers=1)
        -> yields (phase name, arm id, Motion), returns list[Leftover]

Step 1 of the design (docs/DESIGN.md section 3): leaders only, followers parked.

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
from aris.system import followers
from aris.system.allocate import allocate, take_whole
from aris.system.phases import follower_phase, is_fill
from aris.system.phases import phases as all_phases
from aris.system.run import ArmJob, run_phase
from aris.system.settings import Settings
from aris.system.stretch import Stretch, of_line
from aris.types import DrawRules, Leftover, Motion, Phase, Piece, Refusal

AT_PARK = 1e-6          # rad


@dataclass
class ArmReport:
    arm_id: int
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
    drawing_area: np.ndarray | None = None           # (2,) m, rig.json's, checked against the maps
    # Step 2: per (phase name, follower) its leader's footprint in its frame (the checker needs
    # it: `check(..., fields=)`), and its drawable map against it
    fields: dict = field(default_factory=dict)
    follower_maps: dict = field(default_factory=dict)
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
        if not at_park(rig, a, q[a]) and a not in first_active:
            raise ValueError(f"arm {a} is not at its park and does not move in the first phase")
    return q


def at_park(rig, arm_id, q) -> bool:
    return float(np.max(np.abs(np.asarray(q) - rig.park_q(arm_id)))) <= AT_PARK


def plan(rig, lines, rules: DrawRules | None = None, arm_configs=None, cache_dir=None,
         workers: int = 1, settings: Settings | None = None, report: Report | None = None,
         verify=None):
    """Yields (phase name, arm id, Motion) as the arm planners produce them; returns the
    leftovers, or a Refusal (before anything is planned) when rig.json has no drawing area, when
    the maps disagree with it by more than a grid cell, or when a point of the drawing lies
    outside it.  `rules`: default `rig.rules()`, the only source.
    `lines`: table-frame Lines with distinct ids.  `arm_configs`: arm id -> where it
    stands (default: its park).  `cache_dir`: where the drawable maps and the local planner's
    kinematic table are kept.  `workers`: processes (map building, arms of a phase).
    `verify(arm_id, phase, fields, motion, q_before) -> dict`: the independent checker,
    picklable; each arm planner gets it with the first three bound (the Phase and footprints
    that arm's motions are checked in, `check_view`) and hands on only motions that passed.
    None: nothing is checked here."""
    cfg = settings or Settings()
    rules = rig.rules() if rules is None else rules
    rep = report if report is not None else Report()
    c0, w0 = _cpu(), time.perf_counter()
    phases = all_phases(rig)
    if len({x.id for x in lines}) != len(lines):
        raise ValueError("two lines share an id")
    q_now = start_configs(rig, arm_configs, phases[0].active)
    maps = maps_mod.load_or_build(rig, phases, rules.gates, cfg, cache_dir, workers)
    rep.map_cpu, rep.map_wall = _cpu() - c0, time.perf_counter() - w0
    rep.coverage = maps_mod.coverage(maps, phases)
    file_area = getattr(rig, "drawing_area_m", None)
    if file_area is None:
        return Refusal("no_drawing_area", "config/rig.json has no canvas.drawing_area_m")
    rep.drawing_area = np.asarray(file_area, float).reshape(2)
    maps_area = area.admissible(maps)
    # the file's area may be smaller than the maps' on purpose (a conservative choice), never
    # larger by more than a grid cell (a stale file)
    if np.max(rep.drawing_area - maps_area) > cfg.grid_step + 1e-9:
        return Refusal("stale_drawing_area",
                       f"rig.json's drawing area {rep.drawing_area[0]:.3f} x "
                       f"{rep.drawing_area[1]:.3f} m is larger than the maps' {maps_area[0]:.3f} x "
                       f"{maps_area[1]:.3f} m by more than one grid cell ({cfg.grid_step} m)")
    out = area.first_outside(lines, rep.drawing_area)
    if out is not None:
        return Refusal("outside_drawing_area",
                       f"line {out[0]}: point ({out[1][0]:.4f}, {out[1][1]:.4f}) m lies outside "
                       f"the drawing area {rep.drawing_area[0]:.3f} x {rep.drawing_area[1]:.3f} m "
                       "centred on the table")
    pool, left = [of_line(x) for x in lines], []
    for k, ph in enumerate(phases):
        cands = [(j, a, maps[(phases[j].name, a)], not is_fill(phases[j]))
                 for j in range(k, len(phases)) for a in phases[j].active]
        pool, new_left, cuts = allocate(pool, cands, rules, cfg)
        left += new_left
        rep.cuts += cuts
        mine = {a: [s for s in pool if s.target == (k, a)] for a in ph.active}
        pool = [s for s in pool if s.target[0] != k]
        todo = [a for a in ph.active if mine[a] or not at_park(rig, a, q_now[a])]
        if not todo and is_fill(ph):
            rep.skipped.append((ph.name, "nothing allocated to it"))
            continue
        pool, final = yield from _phase(rig, phases, maps, k, todo, mine, pool, q_now, rules,
                                        cfg, cache_dir, workers, rep, w0, verify)
        left += final
        for a in todo:
            q_now[a] = rig.park_q(a)
    _, new_left, _ = allocate(pool, [], rules, cfg)
    rep.cpu, rep.wall = _cpu() - c0, time.perf_counter() - w0
    return left + new_left


def _phase(rig, phases, maps, k, todo, mine, pool, q_now, rules, cfg, cache_dir, workers, rep,
           w0, verify):
    """Runs phase k: its arms, then (in a leader phase) the followers against their leaders'
    footprints.  Yields the tagged motions; returns (the pool for the phases after, leftovers)."""
    ph = phases[k]
    pr = PhaseReport(ph.name, ph.active)
    rep.phases.append(pr)
    t0 = time.perf_counter()
    jobs = {a: (rig.obstacles_for(a, ph), mine[a], q_now[a], ph, ()) for a in todo}
    back, final, trajs = yield from _run(rig, ph, jobs, pr, rules, cache_dir, workers, rep, t0, w0,
                                         verify)
    if cfg.followers and not is_fill(ph):
        back, pool, more = yield from _followers(rig, phases, maps, k, back, pool, trajs, rules,
                                                 cfg, cache_dir, workers, rep, pr, t0, w0,
                                                 verify)
        final += more
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


def _followers(rig, phases, maps, k, back, pool, trajs, rules, cfg, cache_dir, workers, rep, pr,
               t0, w0, verify):
    """Step 2.  The leaders' footprints go to their row partners; each follower takes, whole,
    what its map then holds among what the leaders handed back and what waits for a fill phase
    (allocate.take_whole), and plans it from its park back to its park.  Yields its motions;
    returns (handed back, the pool without what was taken, leftovers)."""
    ph = phases[k]
    fill_bound = [s.target is not None and is_fill(phases[s.target[0]]) for s in pool]
    waiting = [s for s, w in zip(pool, fill_bound) if w]
    kept = [s for s, w in zip(pool, fill_bound) if not w]
    # The footprint only takes space away: nothing a follower's map without its leader (its fill
    # phase's, cached) cannot hold whole is worth building footprints for.
    alone = [(k, f, maps[(p.name, f)]) for _, f in followers.pairs(rig, ph)
             for p in phases if is_fill(p) and f in p.active]
    if not take_whole(back + waiting, alone, cfg)[0]:
        return back, pool, []
    setups = followers.setup_all(rig, ph, trajs, rules.gates, cfg, workers)
    for f, (fld, _, m) in setups.items():
        rep.fields[(ph.name, f)], rep.follower_maps[(ph.name, f)] = fld, m
    taken, rest = take_whole(back + waiting, [(k, f, m) for f, (_, _, m) in setups.items()], cfg)
    jobs = {f: (obs, [s for s in taken if s.target[1] == f], rig.park_q(f),
                *check_view(rig, ph, f, rep))
            for f, (_, obs, _) in setups.items() if any(s.target[1] == f for s in taken)}
    back2, final, _ = yield from _run(rig, ph, jobs, pr, rules, cache_dir, workers, rep, t0, w0,
                                      verify)
    return back2, kept + rest, final


def check_view(rig, phase: Phase, arm_id: int, report) -> tuple[Phase, tuple]:
    """The Phase and footprints (`Field`s) an arm's motions in `phase` are checked in: its own
    phase for a leader or a fill arm; for a follower of a leader phase, the phase as it sees it
    (its leader's walls, nothing parked) with its leader's footprint from `report.fields`."""
    if arm_id in phase.active:
        return phase, ()
    fld = report.fields.get((phase.name, arm_id)) if report is not None else None
    return follower_phase(rig, phase, arm_id), (() if fld is None else (fld,))


def _run(rig, ph, jobs: dict, pr, rules, cache_dir, workers, rep, t0, w0, verify):
    """The arm planners of `jobs` {arm: (obstacles, stretches, q_start, check phase, check
    fields)}, in parallel, each back to its park, each with `verify` bound to its arm, phase and
    fields.  Yields the tagged motions; returns (handed back, leftovers, {arm: [traj]})."""
    subs, arm_jobs = {}, []
    for a, (obs, sts, q0, cph, cfields) in jobs.items():
        pr.arms[a] = ArmReport(a, len(sts), float(sum(s.length for s in sts)))
        base = []
        for i, st in enumerate(sts):
            subs[(a, f"{st.line_id}#{i}")] = st
            base.append(rig.to_base(a, st.as_line(f"{st.line_id}#{i}")))
        bound = None if verify is None else partial(verify, a, cph, cfields)
        arm_jobs.append(ArmJob(a, tuple(base), obs, q0, rig.park_q(a), bound))
    back, final, trajs = [], [], {}
    for a, kind, payload in run_phase(rig, arm_jobs, rules, cache_dir, workers):
        ar = pr.arms[a]
        if kind == "motion":
            m = _retag(payload, subs, a)
            trajs.setdefault(a, []).append(m.traj)
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
            s = subs[(a, x.piece.line_id)]
            part = s.sub(s.s0 + x.piece.s0, s.s0 + x.piece.s1)
            ar.handed_back[x.reason] = ar.handed_back.get(x.reason, 0.0) + part.length
            detail = f"{ph.name}, arm {a}: {x.detail}"
            if x.reason == "too_short":
                final.append(Leftover(part.piece, x.reason, detail))
            else:
                back.append(replace(part, reason=x.reason, detail=detail))
    return back, final, trajs


def _retag(m: Motion, subs: dict, arm_id: int) -> Motion:
    """The motion with its piece in the input line's own id and arc length."""
    if m.piece is None:
        return m
    s: Stretch = subs[(arm_id, m.piece.line_id)]
    a, b = (float(min(max(s.s0 + x, s.s0), s.s1)) for x in (m.piece.s0, m.piece.s1))
    return replace(m, piece=Piece(s.line_id, a, b))


def plan_all(rig, lines, rules, arm_configs=None, cache_dir=None, workers=1, settings=None,
             verify=None):
    """-> ([(phase name, arm id, Motion)], leftovers)."""
    motions, left, _ = plan_detailed(rig, lines, rules, arm_configs, cache_dir, workers, settings,
                                     verify)
    return motions, left


def plan_detailed(rig, lines, rules, arm_configs=None, cache_dir=None, workers=1, settings=None,
                  verify=None):
    """`plan_all`, plus the Report (maps, per phase and arm: times, lengths, what came back)."""
    rep = Report()
    gen = plan(rig, lines, rules, arm_configs, cache_dir, workers, settings, rep, verify)
    out = []
    while True:
        try:
            out.append(next(gen))
        except StopIteration as stop:
            return out, stop.value, rep
