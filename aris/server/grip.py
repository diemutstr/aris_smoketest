"""The grip job: one gripper command on one slot — home, open or close.

    POST /grip/{slot}   {"verb": "home" | "open" | "close", ...params}

A job of kind "grip" like any other (one job at a time, a header, events, a report), with no
phases and no queues: its header names the slot, the verb and the params, and the operator PC
runs it through the slot's gripper driver (robot/aris_robot/runner.py run_grip).  It posts
"grip started" {arm, verb, width_m}, then "grip done" {arm, verb, width_before_m,
width_after_m, grasped (close: true/false, open: false, home: null), nothing_to_do, why} or
"failed" {arm, verb, why, width_before_m, width_after_m}, then "job done / failed".  The report
shows the width before and after, grasped, and whether there was nothing to do.

Refused only when it cannot run: a slot the rig does not have, a verb that is not one of
the three, a slot without a fresh joint reading (its stack is down), or another job running.

With the simulated arms the gripper is simulated here: open and home leave it at
SIM_OPEN_M, close at `width_m` (default 0).
"""
from __future__ import annotations

from aris.server import runner
from aris.types import Refusal

VERBS = ("home", "open", "close")
SIM_OPEN_M = 0.08


def submit_grip(st, store, slot: str, verb: str, params: dict | None = None):
    params = {k: v for k, v in (params or {}).items() if k != "verb"}
    if slot not in st.rig.arm_ids:
        return Refusal("no_slot", f"no slot {slot} on this rig ({', '.join(st.rig.arm_ids)})")
    if verb not in VERBS:
        return Refusal("verb", f"the gripper verbs are {', '.join(VERBS)}, not {verb!r}")
    if st.remote:
        q, why = st.positions.known(slot)
        if q is None:
            return Refusal("no_joint_states", why or f"no joint states for {slot} (FCI off?)")
    return runner.start(st, store, "grip", f"grip {slot} {verb}", _work(slot, verb, params),
                        lambda rec: dict(slot=slot, verb=verb, params=params),
                        lambda rec, job: job.end_phases("a grip job has no phases"))


def _work(slot, verb, params):
    def work(st, rec, job):
        if st.remote:
            run = runner.wait_robot(rec)
            got = [r for r in rec.log.read() if r.get("event") in ("grip done", "failed")
                   and str(r.get("arm")) == slot]
            g = got[-1] if got else {}
            state = run.status if run.status in ("done", "failed", "stopped") else "failed"
            why = run.why
        else:
            before = st.grippers.get(slot, SIM_OPEN_M)
            after = float(params.get("width_m", 0.0)) if verb == "close" else SIM_OPEN_M
            st.grippers[slot] = after
            g = dict(width_before_m=before, width_after_m=after,
                     grasped=dict(home=None, open=False).get(verb, after > 0.0))
            rec.log.write("grip done", arm=slot, verb=verb, **g)
            state, why = ("stopped", "stop requested") if rec.stop.is_set() else ("done", "")
        if state == "done" and not g:
            state, why = "failed", "the operator PC reported no grip result"
        rep = dict(state=state, why=why or g.get("why", ""), kind="grip", slot=slot,
                   verb=verb, params=params, width_before_m=g.get("width_before_m"),
                   width_after_m=g.get("width_after_m"), grasped=g.get("grasped"),
                   nothing_to_do=bool(g.get("nothing_to_do", False)))
        runner.finish_job(rec, job.dir, rep, state, why)
    return work
