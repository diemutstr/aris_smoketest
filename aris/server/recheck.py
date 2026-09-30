"""`aris check <job dir>`: the independent checker again on every motion of a job's queues.

Each queue is checked in the phase it was planned in: the named phase, or the context saved
next to the queue (a follower's view with its leader's footprint; a park phase with the arms
that were not parked yet).  Each motion from where the previous one of its queue ended; the
first of a queue from its own start (the queue itself guaranteed that the motions join).
"""
from __future__ import annotations

import json
from pathlib import Path

from aris.execute.queue import Job
from aris.server.pipeline import check_one, check_pool, context_path, load_context, submit
from aris.server.report import job_phases
from aris.system import phase_named


def contexts(rig, job: Job):
    """(phase name, arm, Phase to check in, fields, entries) for every queue, in run order."""
    for ph in job_phases(job):
        for a in ph.active:
            q = job.queue(ph.name, a)
            if not q.path.exists():
                continue
            cp = context_path(q)
            if cp.exists():
                cph, fields = load_context(cp)
            else:
                try:
                    cph = phase_named(rig, ph.name)
                except KeyError:
                    cph = ph
                fields = ()
            yield ph.name, a, cph, fields, q.read()


def recheck(job_dir: Path, config_dir: Path, workers: int, say, verdict, assumptions_line):
    from aris.server import open_station
    head = json.loads((job_dir / "job.json").read_text())
    st = open_station(config_dir, uncalibrated=True, cache_dir=None, with_arms=False,
                      with_area=False, workers=1, check_workers=workers)
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
        for name, arm, cph, fields, entries in contexts(st.rig, Job(job_dir)):
            for e in entries:
                q_before = entries[e.index - 1].motion.q_end if e.index else e.motion.q_start
                rows.append((name, arm, e, submit(pool, (st.config_dir, arm, e.motion, cph,
                                                         q_before, fields))))
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


__all__ = ["recheck", "contexts", "check_one"]
