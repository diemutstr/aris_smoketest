"""The job report: what was drawn and what was not, with reasons.

Every line of the (fitted) drawing is accounted for by arc length: the stretches whose drawing
motion ran to the end are drawn; everything else is left over, with the first reason that
applies:
  1. the planner's reason (unreachable, blocked, too short, no free path, ...),
  2. "failed_check": the checker refused the motion, or it was dropped after a refusal,
  3. the job's end: "stopped" or "failed" (queued but not run, or not planned yet).
When a job is done, 3 must be empty; anything there is reported as "unaccounted" (a bug).
"""
from __future__ import annotations

import json

import numpy as np

from aris.execute.queue import Job
from aris.server.pipeline import phase_from
from aris.system.planner import AT_PARK

TINY = 1e-6              # m, stretches shorter than this are rounding, not leftovers


def line_length(line) -> float:
    p = np.asarray(line.points, float)
    return float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)))


def _union(iv):
    out = []
    for a, b in sorted(iv):
        if out and a <= out[-1][1] + 1e-12:
            out[-1][1] = max(out[-1][1], b)
        else:
            out.append([a, b])
    return out


def _minus(a, b, covered):
    """[a, b] without the (sorted, disjoint) covered intervals."""
    out, x = [], a
    for c0, c1 in covered:
        if c1 <= x or c0 >= b:
            continue
        if c0 > x:
            out.append((x, c0))
        x = max(x, c1)
    if x < b:
        out.append((x, b))
    return out


def account(lines, drawn, left, rest_reason: str) -> dict:
    """`drawn`: Pieces drawn.  `left`: (Piece, reason, detail) in order of priority.
    -> drawn length, left over by reason, and every leftover stretch."""
    by_line = {x.id: line_length(x) for x in lines}
    drawn_iv = {k: [] for k in by_line}
    for p in drawn:
        drawn_iv.setdefault(p.line_id, []).append((p.s0, p.s1))
    covered = {k: _union(v) for k, v in drawn_iv.items()}
    drawn_m = sum(b - a for v in covered.values() for a, b in v)
    stretches = []
    for piece, reason, detail in left:
        cov = covered.setdefault(piece.line_id, [])
        for a, b in _minus(piece.s0, piece.s1, cov):
            if b - a > TINY:
                stretches.append(dict(line=piece.line_id, s0=a, s1=b, length=b - a,
                                      reason=reason, detail=detail))
        covered[piece.line_id] = _union(cov + [(piece.s0, piece.s1)])
    for lid, L in by_line.items():
        for a, b in _minus(0.0, L, covered.get(lid, [])):
            if b - a > TINY:
                stretches.append(dict(line=lid, s0=a, s1=b, length=b - a, reason=rest_reason,
                                      detail=""))
    by_reason = {}
    for s in stretches:
        by_reason[s["reason"]] = by_reason.get(s["reason"], 0.0) + s["length"]
    return dict(length_m=sum(by_line.values()), drawn_m=drawn_m,
                left_m=sum(by_reason.values()), left_by_reason=by_reason, leftovers=stretches)


def queued_entries(job: Job) -> dict:
    """(phase, arm) -> the queue's entries, for every queue of the job."""
    out = {}
    for ph in job_phases(job):
        for a in ph.active:
            q = job.queue(ph.name, a)
            if q.path.exists():
                out[(ph.name, a)] = q.read()
    return out


def job_phases(job: Job) -> list:
    """The phases written to the job so far, in order."""
    path = job.dir / "phases.jsonl"
    rows = [json.loads(x) for x in path.read_text().splitlines() if x] if path.exists() else []
    return [phase_from(d) for d in rows if not d.get("end")]


def split_run(entries: dict, done: dict) -> tuple[list, list]:
    """Draw pieces that ran to the end, and draw pieces queued but not run.  `done`:
    (phase, arm) -> motions done (None: all of them, for a plan that is not run)."""
    ran, not_run = [], []
    for key, es in entries.items():
        n = len(es) if done is None else done.get(key, 0)
        for e in es:
            if e.motion.piece is not None:
                (ran if e.index < n else not_run).append(e.motion.piece)
    return ran, not_run


def draw_report(st, rec, job: Job, out, run, done: dict | None, first_s, state: str,
                why: str) -> dict:
    """The report of a drawing job (or of `aris plan`, with `run` None and `done` None)."""
    entries = queued_entries(job)
    ran, not_run = split_run(entries, done)
    rest = "unaccounted" if state == "done" or run is None else state
    left = [(x.piece, x.reason, x.detail) for x in out.leftovers]
    left += [(p, "failed_check", d) for p, d in out.cut_pieces]
    left += [(p, rest, "queued, not run") for p in not_run]
    acc = account(rec.lines, ran, left, rest)
    rep = dict(state=state, why=why, kind="draw", name=rec.name,
               drawing=dict(lines=len(rec.lines), scale=rec.fit.scale if rec.fit else 1.0,
                            bbox_in=rec.fit.bbox_in if rec.fit else None,
                            bbox=rec.fit.bbox_out if rec.fit else None),
               **acc,
               checker=dict(checked=out.checked, passed=out.passed,
                            tightest_clearance_m=None if out.checked == 0 else out.tightest,
                            tightest_at=out.tightest_at),
               queued=sum(len(v) for v in entries.values()),
               planner_refusal=None if out.refusal is None else
               f"{out.refusal.reason}: {out.refusal.detail}",
               planner_error=out.error or None, planning_s=out.planning_s,
               first_motion_s=first_s,
               assumptions=st.assumptions())
    if run is not None:
        rep["phases"] = [dict(name=n, end_check_passed=bool(p), tightest=t, clearance_m=c)
                         for n, p, t, c in run.phase_ends]
        rep["where"] = {str(a): [float(v) for v in q] for a, q in run.where.items()}
        rep["at_park"] = {str(a): bool(np.max(np.abs(np.asarray(q) - st.rig.park_q(a)))
                                       <= AT_PARK) for a, q in run.where.items()}
    return rep
