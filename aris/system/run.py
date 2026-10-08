"""Running the arm planners of one phase: one process per active arm, motions handed on as
they arrive.

Every active arm plans alone: its lines, its obstacles (paper, steel, the walls next to it, the
parked arms), from where it stands to its park.  Nothing is shared between the arms of a phase,
so they can run in any order or at once and give the same motions.

With `workers` <= 1 the arms are planned one after another in this process.  Otherwise each arm
gets its own process (at most `workers` at a time), and processes left over go to each arm's
local planner (`workers // arms`).
"""
from __future__ import annotations

import queue as queue_mod
import traceback
from dataclasses import dataclass
from multiprocessing import get_context

import numpy as np

from aris import arm_planner
from aris.types import DrawRules, Obstacles, Slot


@dataclass(frozen=True)
class ArmJob:
    arm_id: Slot
    lines: tuple                # base-frame Lines
    obstacles: Obstacles
    q_start: np.ndarray
    q_end: np.ndarray
    verify: object = None       # verify(motion, q_before) -> dict, picklable; None: unchecked
    rules: DrawRules | None = None   # this arm's rules (its pen); None: the phase's


def stream(rig, job: ArmJob, rules: DrawRules, cache_dir, local_workers: int):
    """Events of one arm plan: ("motion", Motion) ..., then ("done", (leftovers, PlanStats))."""
    st = arm_planner.PlanStats()
    kw = {} if job.verify is None else dict(verify=job.verify)
    gen = arm_planner.plan(rig.arm(job.arm_id), list(job.lines), job.obstacles, job.q_start,
                           job.rules or rules, job.q_end, local_workers, cache_dir, stats=st,
                           **kw)
    while True:
        try:
            yield "motion", next(gen)
        except StopIteration as stop:
            yield "done", (list(stop.value), st)
            return


def _child(args, out) -> None:
    rig, job, rules, cache_dir, local_workers = args
    try:
        for kind, payload in stream(rig, job, rules, cache_dir, local_workers):
            out.put((job.arm_id, kind, payload))
    except BaseException:                       # handed to the parent, which raises it
        out.put((job.arm_id, "error", traceback.format_exc()))


def run_phase(rig, jobs: list, rules: DrawRules, cache_dir=None, workers: int = 1):
    """Yields (arm id, "motion", Motion) as motions arrive and (arm id, "done", (leftovers,
    PlanStats)) once per arm.  An arm planner that raises stops the phase: re-raised here."""
    if workers <= 1 or len(jobs) <= 1:
        for job in jobs:
            for kind, payload in stream(rig, job, rules, cache_dir, max(1, workers)):
                yield job.arm_id, kind, payload
        return
    ctx = get_context("spawn")
    out = ctx.Queue()
    n_proc = min(workers, len(jobs))
    local_workers = max(1, workers // len(jobs))
    waiting, running = list(jobs), {}
    try:
        while waiting or running:
            while waiting and len(running) < n_proc:
                job = waiting.pop(0)
                p = ctx.Process(target=_child, args=((rig, job, rules, cache_dir, local_workers),
                                                     out))
                p.start()
                running[job.arm_id] = p
            try:
                arm_id, kind, payload = out.get(timeout=5.0)
            except queue_mod.Empty:
                dead = [a for a, p in running.items() if not p.is_alive() and p.exitcode]
                if dead:
                    raise RuntimeError(f"the arm planner of arm {dead[0]} died "
                                       f"(exit code {running[dead[0]].exitcode})")
                continue
            if kind == "error":
                raise RuntimeError(f"the arm planner of arm {arm_id} raised:\n{payload}")
            yield arm_id, kind, payload
            if kind == "done":
                running.pop(arm_id).join()
    finally:
        for p in running.values():
            p.terminate()
            p.join()
