"""`aris check <job dir>`: the independent checker again on every motion of a job's queues.

Each queue is checked in its phase as the job's phase list records it, with the arms that
stood still off their parks (`standing`, recorded in the same line for park and calibrate
jobs).  Each motion from where the previous one of its queue ended; the first of a queue from
its own start (the queue itself guaranteed that the motions join).
"""
from __future__ import annotations

import json
from concurrent.futures import Future, ProcessPoolExecutor
from multiprocessing import get_context
from pathlib import Path

from aris.check import check
from aris.execute.queue import Job


def check_one(args):
    """One motion through the checker; runs in a worker process."""
    config_dir, arm, motion, phase, q_before, standing = args
    return check(config_dir, arm, motion, phase, q_before, standing=standing)


def check_pool(workers: int):
    """A pool of checker processes, or None for checking in this thread."""
    return None if workers <= 1 else ProcessPoolExecutor(workers,
                                                         mp_context=get_context("spawn"))


def submit(pool, args) -> Future:
    if pool is not None:
        return pool.submit(check_one, args)
    f = Future()
    f.set_result(check_one(args))
    return f


def recheck(job_dir: Path, config_dir: Path, workers: int, say, verdict, assumptions_line):
    from aris.server import open_station
    head = json.loads((job_dir / "job.json").read_text())
    st = open_station(config_dir, uncalibrated=True, cache_dir=None, with_arms=False,
                      with_area=False, workers=1)
    if not hasattr(st, "rig"):
        return verdict(False, f"{st.reason}: {st.detail}")
    a = st.assumptions()
    say(assumptions_line(dict(a, driver=head.get("driver"), speed=head.get("speed"))))
    same = head.get("rig_digest") == a["rig_digest"] and \
        head.get("calibration_digest") == a["calibration_digest"]
    if not same:
        say("NOTE: the job was planned against another rig or calibration than the one here")
    pool = check_pool(workers)
    rows = []
    try:
        job = Job(job_dir)
        for phase, standing in job.phases():
            for arm in phase.active:
                q = job.queue(phase.name, arm)
                entries = q.read() if q.path.exists() else []
                for e in entries:
                    q_before = entries[e.index - 1].motion.q_end if e.index \
                        else e.motion.q_start
                    rows.append((phase.name, arm, e, submit(pool, (
                        st.config_dir, arm, e.motion, phase, q_before, standing))))
        n_pass = 0
        for name, arm, e, fut in rows:
            v = fut.result()
            n_pass += bool(v.passed)
            say(f"{name:<12} arm {arm:<3} motion {e.index:<4} {e.motion.kind:<5} "
                f"{'pass' if v.passed else 'FAIL'}  tightest {v.tightest}, clearance "
                f"{1e3 * float(v.min_clearance):.1f} mm"
                + ("" if v.passed else f"  failed: {', '.join(v.failed)}"))
    finally:
        if pool is not None:
            pool.shutdown()
    return verdict(bool(rows) and n_pass == len(rows) and same,
                   f"{n_pass} of {len(rows)} motions pass the checker"
                   + ("" if same else "; the rig or calibration differs"))


__all__ = ["recheck", "check_one"]
