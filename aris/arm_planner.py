"""The arm planner: one arm, a list of lines -> the motions that draw them, and what it could not.

    plan(arm, lines, obstacles, q_start, rules, q_end=None, workers=1, cache_dir=None)
        -> yields Motion, returns list[Leftover]

The local planner turns every line into bunches of alternative drawing plans (with the
kinematic table kept in `cache_dir`, if given; `workers` processes); the sequencer orders them
into a tour of drawing and free-space motions, from `q_start` to `q_end` (default `q_start`).
The leftovers of both are merged.  Everything is in the arm's base frame.
See docs/modules/arm_planner.md.
"""
from __future__ import annotations

import os
import time
from dataclasses import dataclass, field

from aris import local
from aris.sequencer import TourReport, tour
from aris.types import DrawRules, Leftover, Motion, Obstacles


@dataclass
class PlanStats:
    """Times and counts of one arm plan.  CPU counts this process and its finished children
    (the local planner's worker processes)."""
    lines: int = 0
    bunches: int = 0
    local_cpu: float = 0.0
    local_wall: float = 0.0
    first_cpu: float = -1.0        # s from the call to the first motion
    first_wall: float = -1.0
    cpu: float = 0.0               # s, the whole plan
    wall: float = 0.0
    local_leftovers: list = field(default_factory=list)
    tour: TourReport = field(default_factory=TourReport)


def _cpu() -> float:
    t = os.times()
    return t.user + t.system + t.children_user + t.children_system


def plan(arm, lines, obstacles: Obstacles, q_start, rules: DrawRules, q_end=None,
         workers: int = 1, cache_dir=None, *, stats: PlanStats | None = None,
         free_options=None, tour_options=None, verify=None):
    """Yields the arm's motions in order; returns every leftover (local planner's first).
    `verify(motion, q_before) -> dict`: the independent checker, see `sequencer.tour`."""
    st = stats if stats is not None else PlanStats()
    c0, w0 = _cpu(), time.perf_counter()
    st.lines = len(lines)
    bunches, left_local = local.plan(arm, lines, obstacles, rules, workers=workers,
                                     cache_dir=cache_dir)
    st.local_cpu, st.local_wall = _cpu() - c0, time.perf_counter() - w0
    st.bunches, st.local_leftovers = len(bunches), list(left_local)
    gen = tour(arm, bunches, q_start, obstacles, rules, q_end, free_options, report=st.tour,
               intensity={x.id: x.intensity for x in lines}, options=tour_options,
               verify=verify)
    while True:
        try:
            m = next(gen)
        except StopIteration as stop:
            left_tour = stop.value
            break
        if st.first_cpu < 0:
            st.first_cpu, st.first_wall = _cpu() - c0, time.perf_counter() - w0
        yield m
    st.cpu, st.wall = _cpu() - c0, time.perf_counter() - w0
    return list(left_local) + list(left_tour)


def plan_all(arm, lines, obstacles, q_start, rules, q_end=None, workers=1, cache_dir=None,
             **kw) -> tuple[list[Motion], list[Leftover]]:
    motions, leftovers, _ = plan_detailed(arm, lines, obstacles, q_start, rules, q_end, workers,
                                          cache_dir, **kw)
    return motions, leftovers


def plan_detailed(arm, lines, obstacles, q_start, rules, q_end=None, workers=1, cache_dir=None,
                  **kw) -> tuple[list[Motion], list[Leftover], PlanStats]:
    """`plan_all`, plus the times and counts (`PlanStats`, the tour's report inside)."""
    st = PlanStats()
    gen = plan(arm, lines, obstacles, q_start, rules, q_end, workers, cache_dir, stats=st, **kw)
    motions = []
    while True:
        try:
            motions.append(next(gen))
        except StopIteration as stop:
            return motions, stop.value, st
