"""The coordinator: runs a job phase by phase.  The only part that knows about phases at run
time.

For each phase, in order: start one executor per moving arm on its queue, each in its own
thread; wait until every one has reached its queue's end marker and stands parked; ask the
independent checker (`check_phase_end`) whether every arm, where it actually stands, may stay
there while the next phase runs; then start the next phase.  No timing across arms: inside a
phase the arms run independently (the walls keep them apart), the coordinator only waits at
the phase ends.

A failure of one arm stops only that arm; the others of the phase finish their queues, and the
job ends there (no next phase: the plan for it assumed every arm parked).  `stop()` stops every
arm at once.  A stopped or failed job is finished: nothing resumes it; what is left to draw is
a new drawing.
"""
from __future__ import annotations

import threading
from dataclasses import dataclass, field

import numpy as np

from aris.check import check_phase_end
from aris.execute.executor import START_TOL, ArmRun, Executor
from aris.execute.log import EventLog
from aris.execute.queue import Job
from aris.types import Phase


@dataclass
class JobRun:
    """How the job went."""
    status: str = "running"              # "done", "failed" or "stopped" at the end
    why: str = ""
    phases_done: list = field(default_factory=list)       # names, in order
    arms: list = field(default_factory=list)              # ArmRun, every executor run
    phase_ends: list = field(default_factory=list)        # (phase name, passed, tightest, m)
    where: dict = field(default_factory=dict)             # arm id -> q at the end


class Coordinator:
    def __init__(self, job: Job, drivers: dict, config_dir, start_tol: float = START_TOL,
                 poll: float = 0.01):
        """`drivers`: arm id -> Driver, every arm of the rig that is switched on."""
        self.job, self.drivers, self.config_dir = job, dict(drivers), config_dir
        self.start_tol, self.poll = start_tol, poll
        self.log = EventLog(job.log_path)
        self._stop = threading.Event()

    def stop(self) -> None:
        """Every arm stops now and holds; `run` returns "stopped" with where every arm is."""
        self._stop.set()
        for d in self.drivers.values():
            d.stop()

    def run(self, phases=None) -> JobRun:
        """Blocks until the job is done, failed or stopped.  `phases`: the phases in order;
        default: as the job's writer adds them (`Job.add_phase`), until `Job.end_phases`."""
        out = JobRun()
        self.log.write("job started", job=str(self.job.dir))
        source = phases if phases is not None else self.job.watch_phases(self.poll, self._stop)
        for phase in source:
            if self._stop.is_set():
                break
            why = self._run_phase(phase, out)
            if why:
                out.status, out.why = ("stopped" if self._stop.is_set() else "failed"), why
                break
            out.phases_done.append(phase.name)
        else:
            if self._stop.is_set():
                out.status, out.why = "stopped", "stop requested"
            else:
                out.status = "done"
        if out.status == "running":
            out.status, out.why = "stopped", "stop requested"
        out.where = {a: d.state().q for a, d in self.drivers.items()}
        self.log.write("job " + out.status, why=out.why, phases_done=out.phases_done,
                       where={str(a): q for a, q in out.where.items()})
        return out

    def _run_phase(self, phase: Phase, out: JobRun) -> str:
        """Runs one phase; returns why the job cannot go on, or ""."""
        missing = [a for a in phase.active if a not in self.drivers]
        if missing:
            return f"{phase.name}: no driver for arms {missing}"
        self.log.write("phase started", phase=phase.name, active=list(phase.active))
        runs: dict[int, ArmRun] = {}

        def work(a):
            ex = Executor(a, self.drivers[a], self.log, self.start_tol)
            runs[a] = ex.run(self.job.queue(phase.name, a), self._stop, self.poll)

        threads = [threading.Thread(target=work, args=(a,), daemon=True) for a in phase.active]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        out.arms += [runs[a] for a in phase.active]
        bad = [r for r in runs.values() if r.status != "finished" or not r.parked
               or not r.complete]
        if bad:
            why = "; ".join(f"arm {r.arm_id} {r.status}"
                            + (f" at motion {r.failed_index}" if r.failed_index >= 0 else "")
                            + (": " + r.why if r.why else "")
                            + ("" if r.parked or r.status != "finished" else " (not parked)")
                            for r in bad)
            self.log.write("phase failed", phase=phase.name, why=why)
            return f"{phase.name}: {why}"
        q = {a: np.asarray(d.state().q, float) for a, d in self.drivers.items()}
        v = check_phase_end(self.config_dir, phase, q)
        out.phase_ends.append((phase.name, v.passed, v.tightest, v.min_clearance))
        self.log.write("phase end check", phase=phase.name, passed=v.passed,
                       tightest=v.tightest, min_clearance=v.min_clearance,
                       failed=list(v.failed))
        if not v.passed:
            return f"{phase.name}: the phase end check failed: {', '.join(v.failed)}"
        self.log.write("phase done", phase=phase.name)
        return ""
