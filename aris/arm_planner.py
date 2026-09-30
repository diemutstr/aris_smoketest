"""The arm planner: one arm, a list of lines -> the motions that draw them, and what it could not.

    plan(arm, lines, obstacles, q_start, rules, q_end=None, workers=1, cache_dir=None)
        -> yields Motion, returns list[Leftover]

The local planner turns every line into bunches of alternative drawing plans (with the
kinematic table kept in `cache_dir`, if given; `workers` processes); the sequencer orders them
into a tour of drawing and free-space motions, from `q_start` to `q_end` (default `q_start`).
The leftovers of both are merged.  Everything is in the arm's base frame.

The lines are planned in batches of `batch` lines, nearest first (by the distance from the pen
tip at `q_start` to the line's nearest point); the pool of workers plans them all in that
order, in the background, while the sequencer draws from the batches that have arrived.
`batch=None` plans every line before the tour starts.
See docs/modules/arm_planner.md.
"""
from __future__ import annotations

import os
import time
from dataclasses import dataclass, field

import numpy as np

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
    batches: int = 0               # batches of lines handed to the sequencer
    last_batch_wall: float = -1.0  # s from the call until the last batch was planned
    tour: TourReport = field(default_factory=TourReport)


def _cpu() -> float:
    t = os.times()
    return t.user + t.system + t.children_user + t.children_system


def plan(arm, lines, obstacles: Obstacles, q_start, rules: DrawRules, q_end=None,
         workers: int = 1, cache_dir=None, *, stats: PlanStats | None = None,
         free_options=None, tour_options=None, verify=None, batch: int | None = 32,
         refill: int | None = 128):
    """Yields the arm's motions in order; returns every leftover (local planner's first).
    `verify(motion, q_before) -> dict`: the independent checker, see `sequencer.tour`.
    `batch`: lines per batch (nearest first), None for all at once.  `refill`: the sequencer
    takes the next batch when fewer pieces than this are alive (None: `batch`).  128 was
    chosen on 1 000-line sets (2026-09-30): first motion 4 to 5 s, pen-up moves at most 5.5 %
    longer than all at once (under 1 % of the time on the rig); 32 starts in 1 to 2 s but
    costs up to 19 % of pen-up time."""
    st = stats if stats is not None else PlanStats()
    c0, w0 = _cpu(), time.perf_counter()
    st.lines = len(lines)
    left_local: list[Leftover] = []
    if batch is None:
        bunches, left_local = local.plan(arm, lines, obstacles, rules, workers=workers,
                                         cache_dir=cache_dir)
        st.local_cpu, st.local_wall = _cpu() - c0, time.perf_counter() - w0
        st.bunches, st.local_leftovers = len(bunches), list(left_local)
        batches, refill = None, 0
    else:
        bunches, refill = [], batch if refill is None else refill
        batches = _collect(_batches(arm, lines, obstacles, rules, q_start, workers, cache_dir,
                                    batch), st, left_local, w0)
    gen = tour(arm, bunches, q_start, obstacles, rules, q_end, free_options, report=st.tour,
               intensity={x.id: x.intensity for x in lines}, options=tour_options,
               verify=verify, batches=batches, refill=refill)
    while True:
        try:
            m = next(gen)
        except StopIteration as stop:
            left_tour = stop.value
            break
        if st.first_cpu < 0:
            st.first_cpu, st.first_wall = _cpu() - c0, time.perf_counter() - w0
        yield m
    if batches is not None:
        batches.close()                 # shuts the pool down (it is done by now)
    st.cpu, st.wall = _cpu() - c0, time.perf_counter() - w0
    return list(left_local) + list(left_tour)


def nearest_first(arm, lines, q_start) -> list[int]:
    """Line indices sorted by the distance from the pen tip at q_start to the line's nearest
    point (ties: the given order)."""
    tip = arm.tip(np.asarray(q_start, float)[None])[0]
    dist = []
    for line in lines:
        p = np.asarray(line.points, float)
        if len(p) < 2:
            dist.append(float(np.min(np.linalg.norm(p - tip, axis=1))) if len(p) else np.inf)
            continue
        a, d = p[:-1], np.diff(p, axis=0)
        u = np.clip(np.einsum("ij,ij->i", tip - a, d) / np.maximum(
            np.einsum("ij,ij->i", d, d), 1e-300), 0.0, 1.0)
        dist.append(float(np.min(np.linalg.norm(a + u[:, None] * d - tip, axis=1))))
    return sorted(range(len(lines)), key=lambda i: (dist[i], i))


def _batches(arm, lines, obstacles, rules, q_start, workers, cache_dir, size):
    """Yields (bunches, leftovers) per batch of `size` lines, nearest first.  With workers > 1
    every line is handed to one pool (`local.LinePool`) at once, in that order, so later
    batches are planned while the tour draws from the earlier ones; each batch is yielded when
    all its lines are done, in order.  With one worker a batch is planned when it is asked for."""
    order = nearest_first(arm, lines, q_start)
    chunks = [order[i:i + size] for i in range(0, len(order), size)]
    if workers <= 1 or len(lines) <= 1:
        for c in chunks:
            yield local.plan(arm, [lines[i] for i in c], obstacles, rules, cache_dir=cache_dir)
        return
    pool = local.LinePool(arm, obstacles, rules, workers=workers, cache_dir=cache_dir)
    try:
        futures = [[pool.submit([lines[i]]) for i in c] for c in chunks]
        for fs in futures:
            got = [f.result() for f in fs]
            yield [b for r in got for b in r[0]], [x for r in got for x in r[1]]
    finally:
        pool.close(cancel=True)


def _collect(batches, st: PlanStats, left_local: list, w0: float):
    """The bunches of each batch for the sequencer; the local planner's leftovers and the
    counts kept aside."""
    try:
        for bunches, left in batches:
            st.batches += 1
            st.bunches += len(bunches)
            left_local.extend(left)
            st.local_leftovers = list(left_local)
            st.last_batch_wall = time.perf_counter() - w0
            yield bunches
    finally:
        batches.close()


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
