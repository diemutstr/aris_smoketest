"""From the planner's stream into the job's queues, through the independent checker.

`feed(job, tagged, config_dir, phase_of)` takes `(phase name, arm id, Motion)` as the system
planner yields them, checks each one (`aris.check.check`, from where that arm's previous motion
ended), appends it to its queue with the verdict, and writes the phase list as phases begin.
When a phase is over (the next one begins, or the stream ends) every queue of it is closed, an
empty one for a moving arm that got nothing.  A motion the checker refuses is not queued; the
rest of that arm's motions in the phase are dropped and its queue is closed as cut short, so
the coordinator does not start the next phase.  This is the drawing server's job; it lives
here so the execution path can be run and tested whole.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from aris.check import check
from aris.execute.queue import Job
from aris.types import Refusal


@dataclass
class FeedReport:
    queued: dict = field(default_factory=dict)       # (phase, arm) -> motions queued
    refused: list = field(default_factory=list)      # (phase, arm, index, why)
    dropped: dict = field(default_factory=dict)      # (phase, arm) -> motions not queued after
    tagged: list = field(default_factory=list)       # everything the planner handed over
    result: object = None                            # what the planner returned at the end


def feed(job: Job, tagged, config_dir, phase_of, start=None, check_fn=check) -> FeedReport:
    """`tagged`: the system planner's generator (or any iterable of (phase, arm, Motion)).
    `phase_of`: phase name -> Phase (`aris.system.phase_named` with the rig).  `start`: arm
    id -> where it stands before the job (default: where its first motion starts)."""
    rep, queues, cut, phase = FeedReport(), {}, {}, None
    where = dict(start or {})
    it = iter(tagged)
    while True:
        try:
            name, arm, motion = next(it)
        except StopIteration as e:
            rep.result = e.value
            break
        rep.tagged.append((name, arm, motion))
        if phase is None or name != phase.name:
            if phase is not None:
                _close_phase(job, phase, queues, cut)
            phase = phase_of(name)
            job.add_phase(phase)
        key = (name, arm)
        if key in cut:
            rep.dropped[key] = rep.dropped.get(key, 0) + 1
            continue
        q = queues.setdefault(key, job.queue(name, arm))
        verdict = check_fn(config_dir, arm, motion, phase, where.get(arm, motion.q_start))
        res = q.append(motion, verdict)
        if isinstance(res, Refusal):
            rep.refused.append((name, arm, rep.queued.get(key, 0), f"{res.reason}: {res.detail}"))
            cut[key] = f"motion {rep.queued.get(key, 0)} refused ({res.reason})"
            continue
        rep.queued[key] = rep.queued.get(key, 0) + 1
        where[arm] = motion.q_end
    if phase is not None:
        _close_phase(job, phase, queues, cut)
    job.end_phases("the planner is done")
    return rep


def _close_phase(job, phase, queues, cut) -> None:
    for a in phase.active:
        q = queues.get((phase.name, a)) or job.queue(phase.name, a)
        why = cut.get((phase.name, a), "")
        q.close(complete=not why, note=why)
