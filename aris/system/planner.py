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

import numpy as np

from aris.system import maps as maps_mod
from aris.system.allocate import allocate
from aris.system.phases import phases as all_phases
from aris.system.run import ArmJob, run_phase
from aris.system.settings import Settings
from aris.system.stretch import Stretch, of_line
from aris.types import DrawRules, Leftover, Motion, Piece

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
    handed_back: dict = field(default_factory=dict)   # reason -> m it could not draw
    stats: object = None        # arm_planner.PlanStats


@dataclass
class PhaseReport:
    name: str
    active: tuple
    arms: dict = field(default_factory=dict)   # arm id -> ArmReport
    wall: float = 0.0           # s, planning the whole phase

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


def plan(rig, lines, rules: DrawRules, arm_configs=None, cache_dir=None, workers: int = 1,
         settings: Settings | None = None, report: Report | None = None):
    """Yields (phase name, arm id, Motion) as the arm planners produce them; returns the
    leftovers.  `lines`: table-frame Lines with distinct ids.  `arm_configs`: arm id -> where it
    stands (default: its park).  `cache_dir`: where the drawable maps and the local planner's
    kinematic table are kept.  `workers`: processes (map building, arms of a phase)."""
    cfg = settings or Settings()
    rep = report if report is not None else Report()
    c0, w0 = _cpu(), time.perf_counter()
    phases = all_phases(rig)
    if len({x.id for x in lines}) != len(lines):
        raise ValueError("two lines share an id")
    q_now = start_configs(rig, arm_configs, phases[0].active)
    maps = maps_mod.load_or_build(rig, phases, rules.gates, cfg, cache_dir, workers)
    rep.map_cpu, rep.map_wall = _cpu() - c0, time.perf_counter() - w0
    rep.coverage = maps_mod.coverage(maps, phases)
    pool, left = [of_line(x) for x in lines], []
    for k, ph in enumerate(phases):
        cands = [(j, a, maps[(phases[j].name, a)]) for j in range(k, len(phases))
                 for a in phases[j].active]
        pool, new_left = allocate(pool, cands, rules, cfg)
        left += new_left
        mine = {a: [s for s in pool if s.target == (k, a)] for a in ph.active}
        pool = [s for s in pool if s.target[0] != k]
        todo = [a for a in ph.active if mine[a] or not at_park(rig, a, q_now[a])]
        if not todo:
            continue
        back, final = yield from _phase(rig, ph, todo, mine, q_now, rules, cache_dir, workers,
                                        rep, w0)
        pool += back
        left += final
        for a in todo:
            q_now[a] = rig.park_q(a)
    _, new_left = allocate(pool, [], rules, cfg)
    rep.cpu, rep.wall = _cpu() - c0, time.perf_counter() - w0
    return left + new_left


def _phase(rig, ph, todo, mine, q_now, rules, cache_dir, workers, rep: Report, w0):
    """Runs one phase.  Yields the tagged motions; returns (stretches handed back, leftovers)."""
    pr = PhaseReport(ph.name, ph.active)
    rep.phases.append(pr)
    t0 = time.perf_counter()
    subs, jobs = {}, []
    for a in todo:
        ar = pr.arms[a] = ArmReport(a, len(mine[a]), float(sum(s.length for s in mine[a])))
        base = []
        for i, st in enumerate(mine[a]):
            sid = f"{st.line_id}#{i}"
            subs[(a, sid)] = st
            base.append(rig.to_base(a, st.as_line(sid)))
        jobs.append(ArmJob(a, tuple(base), rig.obstacles_for(a, ph), q_now[a], rig.park_q(a)))
    back, final = [], []
    for a, kind, payload in run_phase(rig, jobs, rules, cache_dir, workers):
        ar = pr.arms[a]
        if kind == "motion":
            m = _retag(payload, subs, a)
            ar.motions += 1
            ar.motion_time += float(m.traj.t[-1] - m.traj.t[0])
            if m.kind == "draw":
                ar.drawn += m.piece.s1 - m.piece.s0
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
    pr.wall = time.perf_counter() - t0
    return back, final


def _retag(m: Motion, subs: dict, arm_id: int) -> Motion:
    """The motion with its piece in the input line's own id and arc length."""
    if m.piece is None:
        return m
    s: Stretch = subs[(arm_id, m.piece.line_id)]
    a, b = (float(min(max(s.s0 + x, s.s0), s.s1)) for x in (m.piece.s0, m.piece.s1))
    return replace(m, piece=Piece(s.line_id, a, b))


def plan_all(rig, lines, rules, arm_configs=None, cache_dir=None, workers=1, settings=None):
    """-> ([(phase name, arm id, Motion)], leftovers)."""
    motions, left, _ = plan_detailed(rig, lines, rules, arm_configs, cache_dir, workers, settings)
    return motions, left


def plan_detailed(rig, lines, rules, arm_configs=None, cache_dir=None, workers=1, settings=None):
    """`plan_all`, plus the Report (maps, per phase and arm: times, lengths, what came back)."""
    rep = Report()
    gen = plan(rig, lines, rules, arm_configs, cache_dir, workers, settings, rep)
    out = []
    while True:
        try:
            out.append(next(gen))
        except StopIteration as stop:
            return out, stop.value, rep
