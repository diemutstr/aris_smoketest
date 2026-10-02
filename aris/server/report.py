"""The job report: what was drawn and what was not, with reasons.

The account is the system planner's no-drop account (`aris.system.account`), taken on what ran:
the stretches whose drawing motion ran to the end are drawn; the left-over stretches are
  1. the planner's leftovers, with its reasons (unreachable, blocked, too short,
     failed_check, ...),
  2. drawing motions queued but not run, with the job's end ("stopped" or "failed"),
  3. what was never planned (a job stopped or failed while the planner still worked), with the
     job's end; on a job that is done there must be none, and any would be "unaccounted".
The account then holds that drawn and left over cover every line exactly, joins aside.
"""
from __future__ import annotations

import numpy as np

from aris.execute.queue import Job
from aris.system import NoDropViolation, line_length
from aris.system import account as no_drop_account
from aris.types import Leftover, Piece

TINY = 1e-6              # m, gaps shorter than this are rounding


def gaps(lines, pieces) -> list[Piece]:
    """The stretches of `lines` that none of `pieces` covers: what was never planned."""
    by = {}
    for p in pieces:
        by.setdefault(p.line_id, []).append((p.s0, p.s1))
    out = []
    for x in lines:
        at = 0.0
        for a, b in sorted(by.get(x.id, [])):
            if a > at + TINY:
                out.append(Piece(x.id, at, a))
            at = max(at, b)
        L = line_length(x)
        if L > at + TINY:
            out.append(Piece(x.id, at, L))
    return out


def account(lines, ran, leftovers, rest: str, join: float) -> dict:
    """Drawn and left over, by the system planner's account.  `ran`: drawn Pieces;
    `leftovers`: Leftovers; what neither covers is left over as `rest`."""
    left = list(leftovers) + [Leftover(p, rest, "not planned") for p in
                              gaps(lines, list(ran) + [x.piece for x in leftovers])]
    stretches = [dict(line=x.piece.line_id, s0=x.piece.s0, s1=x.piece.s1,
                      length=x.piece.s1 - x.piece.s0, reason=x.reason, detail=x.detail)
                 for x in left]
    try:
        acc = no_drop_account(lines, list(ran), left, join)
    except NoDropViolation as e:                   # a bug: reported, never hidden
        return dict(length_m=sum(line_length(x) for x in lines), drawn_m=None, left_m=None,
                    left_by_reason={}, leftovers=stretches, account_error=str(e))
    return dict(length_m=acc.length, drawn_m=acc.drawn, left_m=acc.left if acc.left > TINY else 0.0,
                left_by_reason={k: v for k, v in acc.left_by_reason.items() if v > TINY},
                leftovers=stretches)


def queued_entries(job: Job) -> dict:
    """(phase, arm) -> the queue's entries, for every queue of the job."""
    out = {}
    for ph, _ in job.phases():
        for a in ph.active:
            q = job.queue(ph.name, a)
            if q.path.exists():
                out[(ph.name, a)] = q.read()
    return out


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
    left = list(out.leftovers) + [Leftover(p, rest, "queued, not run") for p in not_run]
    acc = account(rec.lines, ran, left, rest, st.rules.min_piece)
    rep = dict(state=state, why=why, kind="draw", name=rec.name, note=rec.note,
               air_mm=rec.air_mm,
               rest_of=rec.rest_of, pen=st.pen().get("name"), tracking=st.tracking,
               drawing=dict(lines=len(rec.lines), scale=rec.fit.scale if rec.fit else 1.0,
                            bbox_in=rec.fit.bbox_in if rec.fit else None,
                            bbox=rec.fit.bbox_out if rec.fit else None),
               **acc,
               checker=checker_numbers(rec.system, entries, job.dir / "refused"),
               queued=sum(len(v) for v in entries.values()),
               planner_refusal=None if out.refusal is None else
               f"{out.refusal.reason}: {out.refusal.detail}",
               planner_error=out.error or None, planning_s=out.planning_s,
               first_motion_s=first_s,
               assumptions=st.assumptions())
    rep["planner"] = planner_numbers(rec.system)
    # The verdict on the job: every queued motion carries a passing check, and the job ran to
    # its end.  What was left over is reported, not a failure.
    rep["passed"] = bool(state == "done" and rep["checker"]["all_queued_checked"])
    if run is not None:
        rep["phases"] = [dict(name=n, end_check_passed=bool(p), tightest=t, clearance_m=c)
                         for n, p, t, c in run.phase_ends]
        rep["where"] = {str(a): [float(v) for v in q] for a, q in run.where.items()}
        rep["at_park"] = {str(a): bool(st.rig.at_park(a, q)) for a, q in run.where.items()}
    return rep


def planner_numbers(sr) -> dict | None:
    """The system planner's own times: in all, and per phase and arm (wall seconds from the
    phase's start to that arm's first motion, as received)."""
    if sr is None:
        return None
    return dict(cpu_s=sr.cpu, wall_s=sr.wall, first_motion_s=sr.first_wall,
                maps_wall_s=sr.map_wall, cuts=sr.cuts, skipped=[list(x) for x in sr.skipped],
                phases=[dict(name=p.name, wall_s=p.wall, rig_s=p.duration,
                             arms={str(a): dict(first_motion_s=r.first_phase_wall,
                                                cpu_s=r.cpu, wall_s=r.wall, motions=r.motions,
                                                drawn_m=r.drawn, offered_m=r.offered)
                                   for a, r in list(p.arms.items())})
                        for p in list(sr.phases)])


def checker_numbers(sr, entries: dict, refused_dir) -> dict:
    """The checker's part: motions checked inside the planners (the system planner's counts),
    refused (files in `refused/`), every queued motion's verdict, and the tightest clearance."""
    checked = 0
    if sr is not None:
        checked = sum(r.checked for p in list(sr.phases)
                      for r in list(p.arms.values()) + list(p.idle.values()))
    refused = sorted(p.name for p in refused_dir.glob("*.npz")) if refused_dir.exists() else []
    queued = [e for es in entries.values() for e in es]
    tight = None if sr is None or not np.isfinite(sr.tightest) else float(sr.tightest)
    return dict(checked=checked, refused=len(refused), refused_files=refused,
                all_queued_checked=all(bool(e.verdict.get("passed")) for e in queued),
                tightest_clearance_m=tight, tightest_at="" if sr is None else sr.tightest_at)
