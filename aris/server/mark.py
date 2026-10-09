"""The mark job (`aris mark`, `POST /mark`): where each row's two arms hang against each other,
found by bringing their pen tips together in the air.  No ruler, no marks to hit.

For every row pair (L, R) that shares spots (rig.json `marks`), in rig order, at the first
shared spot (and with `--yaw` also at the second):

1. every arm parked first (the park job's steps);
2. "meet <spot> <L>": L flies (free motion, checked) from its park to a hover above the spot,
   its pen upright, tip MEET_HEIGHT above the paper and a half gap toward −x; then
   "meet <spot> <R>": R the same toward +x, L standing at its hover (the half gap: the first
   of HALF_GAPS both arms reach);
3. "meet <spot>": both arms active, each a `guide`.  At the arms the person switches both to
   Desk's programming mode, brings the two pen tips together in the air, lets go, and switches
   both back to execution mode with FCI on; each driver answers "check" with the joints at
   standstill (the executor's "registered" row) and holds the arm where it stands;
4. once "meet <spot>" has run, the way home is planned from where the arms REALLY stand (the
   drivers leave them holding where the person let go): retreats first for an arm too close
   to the other or past a joint limit, then "park <L> after <spot>", "park <R> after <spot>",
   one arm at a time.

At the end, per pair, `aris.server.meetings.calibrate_from_meetings(config_dir, (L, R),
[{L: q, R: q}, ...])` (the calib's `solve_meetings`, written when it passes): where the two tips met is one point, so the joints give the row's
relative x and y (one meeting: yaw nominal) and yaw (two meetings).  On a pass the rig
reloads.  The report lists the meetings, the solved x, y, yaw per slot and the residual.
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace

import numpy as np

from aris.check import check
from aris.free import plan as free_plan
from aris.sequencer.guard import Guard
from aris.server.steps import Scene, Step
from aris.types import Motion, Piece, Refusal, Trajectory

MEET_HEIGHT = 0.030      # m above the paper: where the tips are brought together
# m: each tip this far from the spot along the pair at the hover, the first that both arms
# reach inside the arm-to-arm clearance (half gaps: two-arm rig 50 mm at A, 120 at B; six-arm rig 100
# at A, R1b, S12L, 150 at B, S12R, 200 at R1a; 30 mm, tips 60 mm apart, is refused everywhere)
HALF_GAPS = (0.050, 0.060, 0.080, 0.100, 0.120, 0.150, 0.200, 0.250)
SPINS = np.deg2rad(np.arange(0.0, 360.0, 15.0))
Q7_TRIES = np.linspace(-1.2, 1.2, 9)

TO_DO = ("at the two arms: switch BOTH to programming mode in Desk;",
         "bring the two pen tips together in the air, touching, and let go;",
         "switch BOTH back to execution mode with FCI on: the arms then return and park.")


def neighbour_pairs(rig, slots) -> list[dict]:
    """[{slots: (a, b), spots: [spot, ...], kind: "row" | "column"}] for every two slots of
    `slots` that share spots (rig.json `marks` `shared_by`): a row's L and R (L first), or two
    neighbours along the table (rig order first); in rig order; spots in rig.json order."""
    order = {x: i for i, x in enumerate(rig.arm_ids)}
    out = {}
    for name, (xy, sharers) in rig.marks.items():
        s = tuple(sorted(sharers, key=lambda x: order.get(x, 99)))
        if len(s) != 2 or not set(s) <= set(slots):
            continue
        row = s[0][0] == s[1][0]
        if row:
            s = tuple(x for side in "LR" for x in s if x[1] == side)
        out.setdefault(s, dict(slots=s, spots=[], kind="row" if row else "column"))
        out[s]["spots"].append(name)
    return sorted(out.values(), key=lambda p: (min(order[x] for x in p["slots"]),
                                               max(order[x] for x in p["slots"])))


def row_pairs(rig, slots) -> list[dict]:
    """The row pairs among `neighbour_pairs`."""
    return [p for p in neighbour_pairs(rig, slots) if p["kind"] == "row"]


def hover(st, slot, tip_table, guard, q_near) -> np.ndarray | None:
    """The joints that hold the pen upright with its tip at `tip_table`, inside the gates,
    nearest `q_near` over the hand's turns about the pen and the arm's shapes."""
    rig, gates = st.rig, st.rules_for(slot).gates
    arm, paper = rig.arm(slot), rig.paper(slot, for_planning=True)
    T = rig.T_base_table(slot)
    p = T[:3, :3] @ np.asarray(tip_table, float) + T[:3, 3]
    q7s = rig.park_q(slot)[6] + Q7_TRIES
    best = None
    for spin in SPINS:
        Th = arm.hand_pose(p[None], paper.normal, np.array([spin]), np.zeros((1, 2)))
        Q, ok = arm.ik(np.repeat(Th, len(q7s), axis=0), q7s)
        for i in range(len(q7s)):
            for b in np.flatnonzero(ok[i]):
                h = Q[i, b]
                if (arm.limit_margin(h[None]).min() < gates.limit_margin
                        or arm.sigma_min(h[None]).min() < gates.sigma_min
                        or guard.hold(h, touching=False) is not None):
                    continue
                d = float(np.linalg.norm(h - q_near))
                if best is None or d < best[0]:
                    best = (d, h)
    return None if best is None else best[1]


@dataclass
class MarkPlan:
    steps: list = field(default_factory=list)
    pairs: list = field(default_factory=list)
    meetings: list = field(default_factory=list)   # [{pair, spot, phase, hovers}]
    why: str = ""

    def failed(self) -> dict:
        return dict(pairs=self.pairs, meetings=self.meetings)


def _fly(st, slot, now, spot, side, half_gap, along) -> tuple | str:
    """(Step flying `slot` from where it stands to its hover at `spot`, `side * half_gap` along
    the unit vector `along` from the spot, the hover, its verdict, its tip) or why."""
    rig = st.rig
    xy = np.asarray(rig.marks[spot][0], float) + side * half_gap * np.asarray(along, float)
    tip = np.array([xy[0], xy[1], rig.paper_z + MEET_HEIGHT])
    obs, standing, phase = Scene(rig).of(slot, now, (slot,), (), f"meet {spot} {slot}")
    h = hover(st, slot, tip, Guard(rig.arm(slot), obs, st.rules_for(slot).gates), now[slot])
    if h is None:
        return f"{slot}: no upright pen pose over {spot} inside the gates"
    m = free_plan(rig.arm(slot), now[slot], h, obs, st.rules_for(slot), seed_extra=b"meet")
    if isinstance(m, Refusal):
        return f"{slot}: no way to its hover over {spot}: {m.reason}: {m.detail}"
    v = check(st.config_dir, slot, m, phase, now[slot], standing=standing)
    if not v.passed:
        return f"{slot}: the way to its hover over {spot} fails the checker: " + \
            ", ".join(v.failed)
    return Step(slot, phase, (m,), (v,), dict(standing)), h, v, tip


def _home(st, now, suffix: str):
    """Steps from where the arms really stand to their parks (retreats first, then lifts and
    one park phase per arm), phase names suffixed; or why not."""
    from aris.server.park import plan_park
    steps = plan_park(st, now)
    bad = [x for x in steps if x.why and x.why != "already at its park"]
    if bad:
        return "; ".join(f"{x.arm}: {x.why}" for x in bad)
    return [replace(x, phase=replace(x.phase, name=f"{x.phase.name}{suffix}"))
            for x in steps if x.motions]


def plan_meeting(st, now, pair, spot) -> tuple | str:
    """From the arms parked (`now`): both fly to their hovers at `spot` ("meet A 2L", "meet A
    2R"), then "meet A" with one guide each.  -> (steps, meeting record) or why not."""
    from aris.execute.queue import verdict_numbers
    rig = st.rig
    L, R = pair
    along = rig.T_table_base(R)[:2, 3] - rig.T_table_base(L)[:2, 3]
    along = along / np.linalg.norm(along)
    got, why, half = None, "", None
    for half in HALF_GAPS:
        trial, at = {}, dict(now)
        for slot, side in ((L, -1.0), (R, +1.0)):
            f = _fly(st, slot, at, spot, side, half, along)
            if isinstance(f, str):
                why = f
                break
            trial[slot], at[slot] = f, f[1]
        if len(trial) == 2:
            got, now = trial, at
            break
    if got is None:
        return f"{why} (tips up to {2e3 * HALF_GAPS[-1]:.0f} mm apart tried)"
    steps, name = [got[L][0], got[R][0]], f"meet {spot}"
    for slot in (L, R):
        obs, standing, phase = Scene(rig).of(slot, now, (L, R), (), name)
        phase = replace(phase, contact=True)     # the two arms may end tip to tip
        h, v, tip = got[slot][1], got[slot][2], got[slot][3]
        T = rig.T_base_table(slot)
        g = Motion("guide", Trajectory(np.zeros(1), h[None], np.zeros((1, 7))),
                   Piece(spot, 0.0, 0.0), (T[:3, :3] @ tip + T[:3, 3])[None],
                   checked=dict(verdict_numbers(v), tightest="the hover, held at the free "
                                "move's end"))
        steps.append(Step(slot, phase, (g,), (None,), dict(standing)))
    return steps, dict(pair=[L, R], spot=spot, phase=name, gap_m=2 * half,
                       hovers={x: got[x][1].tolist() for x in (L, R)})


def _work(pairs, yaw: bool):
    """The mark job, phase by phase: park, then per meeting the flights and the guides; after
    each meeting the way on (home, retreats first) is planned from where the arms REALLY stand
    (the drivers leave them holding where the person let go), then the next meeting."""
    def work(st, rec, job) -> None:
        import threading
        import time
        from aris.server import runner
        from aris.server.steps import add_steps, run_queued, wait_phase
        where = runner.where_now(st, need_all=True)
        if isinstance(where, Refusal):
            return runner.fail_job(st, rec, job, where.detail)
        rec.set_state("planning")
        t0 = time.perf_counter()
        plan = MarkPlan(pairs=pairs)
        box = {}
        runner_thread = threading.Thread(
            target=lambda: box.update(run=run_queued(st, rec, job, True, where)), daemon=True)
        first = _home(st, where, "")
        if isinstance(first, str):
            job.end_phases("mark not planned")
            return runner.fail_job(st, rec, job, f"the arms cannot all be parked first: {first}")
        add_steps(job, rec, first)
        plan.steps += first
        runner_thread.start()
        rec.set_state("moving")
        now, why = {a: st.rig.park_q(a) for a in st.rig.arm_ids}, ""
        for p in pairs:
            for spot in p["spots"][:2 if yaw and p["kind"] == "row" else 1]:
                got = plan_meeting(st, now, tuple(p["slots"]), spot)
                if isinstance(got, str):
                    why = got
                    break
                add_steps(job, rec, got[0])
                plan.steps += got[0]
                plan.meetings.append(got[1])
                why = wait_phase(rec, got[1]["phase"])
                if why:
                    break
                real = runner.where_now(st, need_all=True)        # where they really stand
                if isinstance(real, Refusal):
                    why = real.detail
                    break
                home = _home(st, real, f" after {spot}")
                if isinstance(home, str):
                    why = f"no way home after {spot}: {home}"
                    break
                add_steps(job, rec, home)
                plan.steps += home
                now = {a: st.rig.park_q(a) for a in st.rig.arm_ids}
            if why:
                break
        job.end_phases(why or "mark planned")
        runner_thread.join()
        run = box["run"]
        if why and run.status == "done":
            run.status, run.why = "failed", why
        rep, state, why2 = _report(st, rec, plan, run, time.perf_counter() - t0)
        runner.finish_job(rec, job.dir, rep, state, why2)
    return work


def group_slots(rig, slots=(), group: str | None = None):
    from aris.server.crosses import group_slots as gs
    return gs(rig, slots, group)


def submit_mark(st, store, slots=(), group: str | None = None, yaw: bool = False):
    """Admit and start the mark job: refused for slots or a group the rig does not have, or
    without two neighbours that share a spot (and, with yaw, a row pair sharing two)."""
    from aris.server import runner
    chosen = group_slots(st.rig, tuple(slots), group)
    if isinstance(chosen, Refusal):
        return chosen
    pairs = neighbour_pairs(st.rig, chosen)
    if not pairs:
        return Refusal("no_pair", f"slots {', '.join(chosen)} hold no two neighbours that "
                       "share a spot")
    short = [p for p in pairs if yaw and p["kind"] == "row" and len(p["spots"]) < 2]
    if short:
        return Refusal("no_second_spot", f"--yaw needs two shared spots; "
                       f"{'/'.join(short[0]['slots'])} share only {short[0]['spots']}")
    return runner.start(st, store, "mark", "mark " + " ".join(chosen) + (" yaw" if yaw else ""),
                        _work(pairs, yaw),
                        lambda rec: dict(slots=list(chosen), group=group, yaw=yaw,
                                         pairs=pairs, to_do=list(TO_DO)),
                        need_positions=True)


def _report(st, rec, plan, run, planning_s):
    rows = [r for r in rec.log.read() if r.get("event") == "registered"]
    meetings = []
    for m in plan.meetings:
        got = {str(r["arm"]): r["q"] for r in rows if r.get("phase") == m["phase"]
               and str(r.get("arm")) in m["pair"] and str(r.get("button")) == "check"}
        meetings.append(dict(m, q=got))
    if rec.stop.is_set():
        state, why = "stopped", "stop requested"
    elif run.status != "done":
        state, why = "failed", run.why
    else:
        state, why = "done", ""
    solved = None
    if state == "done":
        solved = solve(st, [m for m in meetings if len(m["q"]) == 2])
        if solved["passed"]:
            st.reload()
        else:
            state, why = "failed", f"the solve: {solved['why']}"
    rep = dict(state=state, why=why, kind="mark", pairs=plan.pairs, meetings=meetings,
               solved=solved, to_do=list(TO_DO),
               phases=[dict(name=n, end_check_passed=bool(p), tightest=t, clearance_m=c)
                       for n, p, t, c in run.phase_ends],
               planning_s=planning_s)
    return rep, state, why


def solve(st, meetings) -> dict:
    """Every registered meeting to the calib's solver at once -> plain {passed, why, slots:
    {slot: {x_mm, y_mm, yaw_mrad, moved_mm, turned_mrad, yaw}}, residual_mm, worst_mm,
    reference, notes, frame, written}; `aris.server.meetings.calibrate_from_meetings(config_dir,
    [(slot_a, q_a, slot_b, q_b, spot), ...])`, the graph solve, written when it passes."""
    from aris.server.meetings import calibrate_from_meetings as solver
    if not meetings:
        return dict(passed=False, why="no meeting was registered")
    q = lambda v: np.asarray(v, float)
    from aris.server.robots import today
    robot = {a: today(st, a) for a in st.rig.arm_ids if today(st, a)}
    sol, written = solver(st.config_dir, [(m["pair"][0], q(m["q"][m["pair"][0]]),
                                           m["pair"][1], q(m["q"][m["pair"][1]]), m["spot"])
                                          for m in meetings], robot=robot or None)
    ref = next((str(s) for s, f in (sol.slots or {}).items()
                if getattr(f, "yaw_from", "") == "reference"), None)
    slots = {}
    for s, f in (sol.slots or {}).items():
        T, T0 = f.T_table_base, f.T_before
        yaw, yaw0 = (float(np.arctan2(M[1, 0], M[0, 0])) for M in (T, T0))
        slots[str(s)] = dict(x_mm=round(1e3 * float(T[0, 3]), 2),
                             y_mm=round(1e3 * float(T[1, 3]), 2),
                             yaw_mrad=round(1e3 * yaw, 3),
                             moved_mm=round(1e3 * float(np.linalg.norm(T[:2, 3] - T0[:2, 3])), 2),
                             turned_mrad=round(1e3 * (yaw - yaw0), 3),
                             yaw=getattr(f, "yaw_from", "measured"))
    fin = lambda x: None if x is None or not np.isfinite(x) else round(1e3 * float(x), 3)
    return dict(passed=bool(sol.passed), why=sol.why, slots=slots, residual_mm=fin(sol.rms),
                worst_mm=fin(sol.max_residual), reference=ref,
                notes=list(sol.notes or ()), frame=sol.frame, written=[str(p) for p in written])
