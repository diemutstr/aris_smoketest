"""The calibrate job, step 1 (DESIGN.md section 6): the paper under one arm, touched with its
own pen on a grid, and the plane through the touches written as the arm's calibration file.

Planning, for one arm standing at its park with every other arm parked:
1. Grid points inside the rig's drawing area and within `radius` of the arm's axis.
2. One hand spin for every point, pen upright: the 24 spins 15 degrees apart are tried in
   order; the first under which every point the arm can reach at all is reachable is taken.
   A point no spin reaches is dropped.  The spin is never changed between points: with one
   orientation an unknown pen length moves every touch alike, so the tilt stays exact.
3. At each point the pen's straight path from the nominal paper up to the hover `hover` above
   it: an IK answer every `step` at the same hand orientation (the sequencer's rise rule,
   `rise_path`, inside the gates of rig.json), timed slowly (`speed` along the line) and
   flown back down: the "touch" motion is that descent and the climb back, starting and
   ending at the hover, with `extra_depth` the declared uncertainty of the paper's height.
4. A free motion from the park to the first hover, between hovers, and back to the park.
Every motion is checked by the independent checker (a touch as its descent, a lower, and its
climb, a lift: the checker knows no touch) and queued in one phase, "calibrate <arm>".

When the job has run, the contact rows (the joints where each touch met the paper) go to the
solver (`aris.calib.calibration_from_events`); a passing result is written as
`calibration/<arm>.json` in the server's config directory and the station reloads the rig.
"""
from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from aris.check import check
from aris.execute.queue import verdict_numbers
from aris.free import plan as free_plan
from aris.kernel.retime import retime_detailed
from aris.sequencer.guard import Guard
from aris.sequencer.lift import reverse, rise_path
from aris.server.park import Step, _lift_pens, _Scene, at_park, pen_down
from aris.types import JointPath, Motion, Phase, Refusal, Trajectory

SPINS = np.arange(24) * np.deg2rad(15.0)
MIN_CONTACTS = 9         # the solver needs at least this many touches
Q7_TRIES = np.deg2rad([0.0, 15.0, -15.0, 30.0, -30.0, 45.0, -45.0])


@dataclass(frozen=True)
class CalibSettings:
    grid: int = 5               # points per side of the grid
    radius: float = 0.6         # m from the arm's axis
    edge: float = 0.02          # m kept inside the drawing area
    hover: float = 0.06         # m above the nominal paper
    step: float = 0.002         # m between IK samples of the descent
    max_jump: float = 0.15      # rad, the most a joint moves between two samples
    speed: float = 0.005        # m/s of the pen along the descent
    extra_depth: float = 0.020  # m past the nominal paper the arm may go before giving up


@dataclass(frozen=True)
class Plan:
    arm: int
    phase: Phase
    steps: list                 # park.Step, one per motion, in order (all of this arm)
    points_table: np.ndarray    # (N,2) the grid points touched, in order
    dropped: list               # (x, y) of points no spin reaches
    spin: float | None
    why: str = ""               # why there is no plan


def grid_points(rig, arm_id: int, area, cfg: CalibSettings) -> np.ndarray:
    """(N,2) table xy: a grid over the part of the drawing area near the arm."""
    axis = rig.T_table_base(arm_id)[:2, 3]
    half = 0.5 * np.asarray(area, float) - cfg.edge
    lo = np.maximum(-half, axis - cfg.radius)
    hi = np.minimum(half, axis + cfg.radius)
    xs, ys = np.linspace(lo[0], hi[0], cfg.grid), np.linspace(lo[1], hi[1], cfg.grid)
    pts = [(x, y) for i, x in enumerate(xs)
           for y in (ys if i % 2 == 0 else ys[::-1])]          # a serpentine order
    pts = np.array(pts)
    return pts[np.linalg.norm(pts - axis, axis=1) <= cfg.radius + 1e-9]


def _rise(rig, arm, guard, paper, a, xy, spin, cfg, gates):
    """The path from the paper at xy up to the hover, under this spin, or None."""
    T_bt = rig.T_base_table(a)
    tip = T_bt[:3, :3] @ np.array([xy[0], xy[1], rig.paper_z]) + T_bt[:3, 3]
    T = arm.hand_pose(tip[None], paper.normal, np.array([spin]), np.zeros((1, 2)))
    q7s = rig.park_q(a)[6] + Q7_TRIES
    Q, ok = arm.ik(np.repeat(T, len(q7s), axis=0), q7s)
    for i in range(len(q7s)):
        for b in np.flatnonzero(ok[i]):
            q = Q[i, b]
            c = q[None]
            if (arm.limit_margin(c)[0] < gates.limit_margin or arm.sigma_min(c)[0] < gates.sigma_min
                    or guard.hold(q, touching=True) is not None):
                continue
            path = rise_path(arm, guard, q, paper, cfg.hover, cfg.step, cfg.max_jump, gates)
            if not isinstance(path, str) and guard.hold(path[-1], touching=False) is None:
                return path
    return None


def _choose_spin(rig, arm, guard, paper, a, pts, cfg, gates):
    """-> (spin, {point index: path}, [dropped indices]) under the first spin that reaches
    every point some spin reaches."""
    seen = {}
    for spin in SPINS:
        paths = {k: _rise(rig, arm, guard, paper, a, xy, spin, cfg, gates)
                 for k, xy in enumerate(pts)}
        got = {k: p for k, p in paths.items() if p is not None}
        seen[float(spin)] = got
        if len(got) == len(pts):
            return float(spin), got, []
    reach = set().union(*[set(g) for g in seen.values()]) if seen else set()
    for spin, got in seen.items():                          # in the order tried
        if set(got) == reach and reach:
            return spin, got, [k for k in range(len(pts)) if k not in reach]
    spin = max(seen, key=lambda s: len(seen[s]))           # no spin reaches them all
    return spin, seen[spin], [k for k in range(len(pts)) if k not in seen[spin]]


def touch_motion(arm, guard, path_up, rules, cfg) -> Motion | str:
    """The descent and the climb along `path_up` (paper -> hover), timed slowly."""
    tips = arm.tip(path_up)
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(tips, axis=0), axis=1))])
    res = retime_detailed(JointPath(path_up), arm.limits, replace(rules, draw_speed=cfg.speed),
                          s=s, tip_of=arm.tip, smooth=True)
    if isinstance(res, Refusal):
        return f"cannot be timed: {res.reason} {res.detail}"
    up = res.traj
    if guard.flown(up, touching=True) < 0.0:
        return "the timed descent comes too close"
    down = reverse(up)
    T = float(down.t[-1])
    traj = Trajectory(np.concatenate([down.t, T + up.t[1:]]),
                      np.vstack([down.q, up.q[1:]]), np.vstack([down.qd, up.qd[1:]]))
    return Motion("touch", traj, tip_base=arm.tip(traj.q), extra_depth=cfg.extra_depth)


def check_touch(config_dir, a, m: Motion, phase, q_before, fields=()):
    """The touch as the checker knows it: its descent as a lower, its climb as a lift.
    -> (passed, numbers of the tighter half, the verdicts)."""
    k = _bottom(m)
    down = Trajectory(m.traj.t[:k + 1], m.traj.q[:k + 1], m.traj.qd[:k + 1])
    up = Trajectory(m.traj.t[k:] - m.traj.t[k], m.traj.q[k:], m.traj.qd[k:])
    v1 = check(config_dir, a, Motion("lower", down), phase, q_before, fields=fields)
    v2 = check(config_dir, a, Motion("lift", up), phase, m.traj.q[k], fields=fields)
    n1, n2 = verdict_numbers(v1), verdict_numbers(v2)
    worst = n1 if n1["min_clearance"] <= n2["min_clearance"] else n2
    return bool(v1.passed and v2.passed), dict(worst, passed=bool(v1.passed and v2.passed)), \
        (v1, v2)


def _bottom(m: Motion) -> int:
    """The sample where the descent ends: the one furthest (in joints) from the hover."""
    return int(np.argmax(np.linalg.norm(m.traj.q - m.traj.q[0], axis=1)))


def plan_calibrate(st, a: int, where: dict, cfg: CalibSettings = CalibSettings()) -> Plan:
    """The calibrate job's motions for arm `a`, from where every arm stands (`where`)."""
    rig, rules = st.rig, st.rules
    now = {b: np.asarray(q, float) for b, q in where.items()}
    scene = _Scene(rig)
    steps, q = [], now[a]
    obs, fields, phase = scene.of(a, now, (a,), (), f"calibrate {a}")
    if not at_park(rig, a, q) and pen_down(rig, a, q):      # as the park job: lift it first
        up = _lift_pens(st, scene, now, [a])[0]
        if up.why:
            return Plan(a, phase, [], np.zeros((0, 2)), [], None, up.why)
        steps.append(Step(a, phase, up.motions, up.verdicts, fields))
        q = up.motions[-1].q_end
    arm, gates = rig.arm(a), rules.gates
    guard = Guard(arm, obs, gates)
    paper = rig.paper(a, for_planning=True)
    pts = grid_points(rig, a, st.drawing_area, cfg)
    spin, paths, dropped = _choose_spin(rig, arm, guard, paper, a, pts, cfg, gates)
    kept, touches = [], []
    for k in sorted(paths):
        go = free_plan(arm, q, paths[k][-1], obs, rules, seed_extra=b"calibrate")
        touch = touch_motion(arm, guard, paths[k], rules, cfg) if not isinstance(go, Refusal) \
            else f"free-space planner: {go.reason}: {go.detail}"
        if isinstance(go, Refusal) or isinstance(touch, str):
            dropped.append(k)
            continue
        v = check(st.config_dir, a, go, phase, q, fields=fields)
        ok, numbers, _ = check_touch(st.config_dir, a, touch, phase, go.q_end, fields)
        if not v.passed or not ok:
            dropped.append(k)
            continue
        steps += [Step(a, phase, (go,), (v,), fields),
                  Step(a, phase, (replace(touch, checked=numbers),), (None,), fields)]
        q, kept = touch.q_end, kept + [k]
    home = free_plan(arm, q, rig.park_q(a), obs, rules, seed_extra=b"calibrate home")
    out = lambda why, st_=(): Plan(a, phase, list(st_), pts[kept],
                                   [tuple(map(float, pts[k])) for k in dropped], spin, why)
    if isinstance(home, Refusal):
        return out(f"no way back to the park: {home.reason}: {home.detail}")
    v = check(st.config_dir, a, home, phase, q, fields=fields)
    if not v.passed:
        return out("the way back to the park fails the checker: " + ", ".join(v.failed))
    steps.append(Step(a, phase, (home,), (v,), fields))
    if len(kept) < MIN_CONTACTS:
        return out(f"only {len(kept)} grid points can be touched (at least {MIN_CONTACTS})")
    return out("", steps)


# --------------------------------------------------------------------------- the job


def submit_calibrate(st, store, arm: int):
    """Admit a calibrate job for `arm` and start it (refused like park: another job runs, or
    with the robot, no position reported)."""
    import threading
    from aris.execute import Job
    from aris.server import runner
    from aris.types import Refusal as _Refusal
    if arm not in st.rig.arm_ids:
        return _Refusal("no_arm", f"arm {arm} is not mounted on this rig ({st.rig.arm_ids})")
    if st.remote:
        where = runner.reported_where(st, need_all=True)
        if isinstance(where, _Refusal):
            return where
    rec = store.admit("calibrate", f"calibrate arm {arm}")
    if isinstance(rec, _Refusal):
        return rec
    job = Job.create(rec.dir, runner._header(st, rec, dict(arm=arm)))
    rec.set_state("received", received=rec.t_received, arm=arm)
    rec.thread = threading.Thread(target=_run, args=(st, rec, job, arm), daemon=True,
                                  name=f"job {rec.id}")
    rec.thread.start()
    runner.announce(st, rec)
    return rec


def _run(st, rec, job, arm: int) -> None:
    import time
    import traceback
    from aris.server import runner
    from aris.server.steps import queue_steps, run_queued
    from aris.types import Refusal as _Refusal
    try:
        where = runner.reported_where(st, need_all=True) if st.remote \
            else runner.prepare_arms(st)
        if isinstance(where, _Refusal):
            return _fail(st, rec, job, where.detail)
        rec.set_state("planning")
        t0 = time.perf_counter()
        plan = plan_calibrate(st, arm, where, st.calib_settings or CalibSettings())
        planning_s = time.perf_counter() - t0
        if plan.why:
            return _fail(st, rec, job, plan.why, plan)
        moving = queue_steps(job, rec, plan.steps, "calibration planned")
        rec.set_state("moving")
        run = run_queued(st, rec, job, moving, where)
        state, why, result, written = _solve(st, rec, plan, run)
        rep = _report(st, rec, plan, run, result, written, planning_s, state, why)
        runner._finish(rec, job.dir, rep, state, why)
    except Exception as e:
        if rec.coordinator is not None:
            rec.coordinator.stop()
        rec.log.write("error", why=traceback.format_exc())
        _fail(st, rec, job, f"internal error: {e!r}")


def _fail(st, rec, job, why, plan=None) -> None:
    from aris.server import runner
    phases = job.dir / "phases.jsonl"
    if not phases.exists() or '"end"' not in phases.read_text():
        job.end_phases("failed")
    rep = dict(state="failed", why=why, kind="calibrate", assumptions=st.assumptions())
    if plan is not None:
        rep.update(points=len(plan.points_table), dropped=plan.dropped, spin_deg=None
                   if plan.spin is None else float(np.rad2deg(plan.spin)))
    runner._finish(rec, job.dir, rep, "failed", why)


def touch_points(plan: Plan) -> dict:
    """Queue index of every touch motion -> its grid point (x, y)."""
    out, i, k = {}, 0, 0
    for s in plan.steps:
        for m in s.motions:
            if m.kind == "touch":
                out[i] = tuple(round(float(x), 4) for x in plan.points_table[k])
                k += 1
            i += 1
    return out


def misses(plan: Plan, rows, arm: int) -> list:
    """The grid points whose touch found no paper (a "contact" row is missing for them)."""
    got = {r.get("index") for r in rows
           if r.get("event") == "contact" and r.get("arm") == arm}
    def tried(r):
        ev = r.get("event")
        return r.get("arm") == arm and (ev in ("motion done", "no contact")
                                        or (ev == "failed" and r.get("why") == "no contact"))
    ran = {r.get("index") for r in rows if tried(r)}
    return [p for i, p in touch_points(plan).items() if i in ran and i not in got]


def _solve(st, rec, plan, run):
    """The solver on the contact rows; a passing result is written and the rig reloaded.  A
    touch that met no paper is skipped; the job fails only below MIN_CONTACTS contacts."""
    from aris.calib import calibration_from_events
    from aris.calib.files import write
    arm = plan.arm
    if rec.stop.is_set():
        return "stopped", "stop requested", None, None
    rows = rec.log.read()
    missed = misses(plan, rows, arm)
    n = sum(1 for r in rows if r.get("event") == "contact" and r.get("arm") == arm)
    if run.status != "done" and not (missed and "no contact" in run.why):
        return "failed", run.why, None, None
    if n < MIN_CONTACTS:
        return "failed", (f"only {n} touches met the paper (at least {MIN_CONTACTS}); no "
                          f"contact at {missed}"), None, None
    result = calibration_from_events(st.rig, arm, rows)
    if not result.passed:
        return "failed", f"the plane fit did not pass: {result.why}", result, None
    path = write(result, st.config_dir)
    st.reload()                                    # the station runs on the new file now
    return "done", "", result, str(path)


def _report(st, rec, plan, run, result, written, planning_s, state, why) -> dict:
    from aris.server.jobs import arm_progress
    rows, first = arm_progress(rec.log.read())
    contacts = sum(1 for r in rec.log.read() if r.get("event") == "contact"
                   and r.get("arm") == plan.arm)
    out = dict(state=state, why=why, kind="calibrate", arm=plan.arm,
               points=len(plan.points_table), points_table=plan.points_table.tolist(),
               dropped=plan.dropped, spin_deg=round(float(np.rad2deg(plan.spin)), 3),
               contacts=contacts, missed=misses(plan, rec.log.read(), plan.arm),
               written=written, planning_s=planning_s,
               phases=[dict(name=n, end_check_passed=bool(p), tightest=t, clearance_m=c)
                       for n, p, t, c in run.phase_ends],
               first_motion_s=None if first is None else first - rec.t_received,
               assumptions=st.assumptions())
    if result is not None:
        f = lambda x: None if x is None or not np.isfinite(x) else round(float(x), 4)
        out["fit"] = dict(passed=bool(result.passed), why=result.why, n_points=result.n_points,
                          rms_mm=f(result.rms * 1e3), max_residual_mm=f(result.max_residual * 1e3),
                          roll_deg=f(np.rad2deg(result.roll)),
                          pitch_deg=f(np.rad2deg(result.pitch)),
                          tilt_deg=f(np.rad2deg(result.tilt)),
                          height_change_mm=f(result.height_change * 1e3),
                          residuals_mm=[round(float(x) * 1e3, 4) for x in result.residuals])
    return out
