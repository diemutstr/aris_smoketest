"""Jobs that are a short list of planned motions (park, calibrate): queue them, run them.

`queue_steps` writes the steps' motions into the job, phase by phase in the order the steps
come, each with its checker verdict (or the one the motion carries); `run_queued` runs the job:
on the simulated arms with the coordinator here, or, with `--driver robot`, by waiting for the
operator PC's runner to report the end.
"""
from __future__ import annotations

from aris.execute import Coordinator


def queue_steps(job, rec, steps, note: str) -> bool:
    """-> whether anything moves.  Steps with a `why` are not queued."""
    from aris.server.pipeline import context_path, save_context
    moving = [s for s in steps if s.motions and not s.why]
    for name in dict.fromkeys(s.phase.name for s in moving):        # phases in order
        these = [s for s in moving if s.phase.name == name]
        job.add_phase(these[0].phase)
        for arm in dict.fromkeys(s.arm for s in these):
            q = job.queue(name, arm)
            for s in (s for s in these if s.arm == arm):
                for m, v in zip(s.motions, s.verdicts):
                    q.append(m, v)
                    rec.count_queued(name, arm)
                if s.fields:
                    save_context(context_path(q), s.phase, s.fields)
            q.close()
    job.end_phases(note)
    return bool(moving)


def run_queued(st, rec, job, moving: bool, where: dict):
    """The coordinator's JobRun for the queued job (the robot's, read from its rows)."""
    from aris.server.runner import _robot_run, _wait_robot
    if st.remote:                        # the operator PC runs it and reports
        run = _wait_robot(rec) if moving else _robot_run(rec)
        if not moving:
            run.status, run.why = "done", ""
        run.where = {**where, **run.where}
        return run
    coord = Coordinator(job, st.drivers, st.config_dir, st.rig)
    rec.coordinator = coord
    if rec.stop.is_set():
        coord.stop()
    return coord.run()
