"""The calibrate job, step 1 (DESIGN.md section 6): the paper under one arm, touched with its
own pen on a grid, and the plane through the touches written as the arm's calibration file.

Planning, for one arm, from where every arm stands:
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
Every motion is checked by the independent checker (a touch as the checker knows it: its
descent to the real paper and its climb) and queued in one phase, "calibrate <arm>".

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
from aris.server import paper
from aris.server.steps import Scene, Step, lift_pens, pen_down
from aris.types import JointPath, Motion, Phase, Refusal, Trajectory

SPINS = np.arange(24) * np.deg2rad(15.0)
from aris.calib.plane import MIN_POINTS as MIN_CONTACTS   # the plane fit's own minimum
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
    """The calibrate job's steps for one arm (or why there are none)."""
    arm: str
    phase: Phase
    steps: list                 # park.Step, one per motion, in order (all of this arm)
    points_table: np.ndarray    # (N,2) the grid points touched, in order
    dropped: list               # (x, y) of points no spin reaches
    spin: float | None
    why: str = ""               # why there is no plan

    def failed(self) -> dict:
        """What a failed job's report says about the plan."""
        return dict(points=len(self.points_table), dropped=self.dropped,
                    spin_deg=None if self.spin is None else float(np.rad2deg(self.spin)))


def grid_points(rig, arm_id: str, area, cfg: CalibSettings, centre=(0.0, 0.0)) -> np.ndarray:
    """(N,2) table xy: a grid over the part of the drawing area (centred on `centre`) near the
    arm."""
    axis = rig.T_table_base(arm_id)[:2, 3]
    c = np.asarray(centre, float).reshape(2)
    half = 0.5 * np.asarray(area, float) - cfg.edge
    lo = np.maximum(c - half, axis - cfg.radius)
    hi = np.minimum(c + half, axis + cfg.radius)
    xs, ys = np.linspace(lo[0], hi[0], cfg.grid), np.linspace(lo[1], hi[1], cfg.grid)
    pts = [(x, y) for i, x in enumerate(xs)
           for y in (ys if i % 2 == 0 else ys[::-1])]          # a serpentine order
    pts = np.array(pts)
    return pts[np.linalg.norm(pts - axis, axis=1) <= cfg.radius + 1e-9]


def _rise(rig, arm, guard, paper, a, xy, spin, cfg, gates):
    """The path from the paper at xy up to the hover, under this spin, or None."""
    T_bt = rig.T_base_table(a)
    # the touch ends on the nominal paper (the press is a drawing matter, not a probe matter)
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


def check_touch(config_dir, a, m: Motion, phase, q_before, standing=None):
    """The checker on a touch (it splits the touch at its bottom itself: the descent to the
    real paper, the climb).  -> (passed, the numbers the motion carries, the verdict)."""
    v = check(config_dir, a, m, phase, q_before, standing=standing)
    return bool(v.passed), verdict_numbers(v), v


def plan_calibrate(st, a: str, where: dict, cfg: CalibSettings = CalibSettings(),
                   points=None, name: str = "calibrate", min_points: int | None = None,
                   paper_dz: float = 0.0, home: bool = True) -> Plan:
    """The calibrate job's motions for arm `a`, from where every arm stands (`where`).
    `points`: (N,2) table xy to touch instead of the grid (the touch-off: one point);
    `name`: the phase is "<name> <slot>"; `min_points`: fewer reachable is a refusal;
    `paper_dz`: m above the nominal paper where the free moves keep their clearance from it
    (the paper found by a first touch, less a little; the touches still run down to the
    nominal paper and past it); `home`: back to the
    park at the end (else the arm stays at its last hover)."""
    min_points = MIN_CONTACTS if min_points is None else min_points
    rig, rules = st.rig, st.rules_for(a)
    from aris.server.retreat import retreats
    now = {b: np.asarray(q, float) for b, q in where.items()}
    scene = Scene(rig)
    steps, now = retreats(st, now)                 # arms within the clearance move apart first
    q = now[a]
    obs, standing, phase = scene.of(a, now, (a,), (), f"{name} {a}")
    bad = [s for s in steps if s.why]
    if bad:
        return Plan(a, phase, [], np.zeros((0, 2)), [], None, "; ".join(
            f"{s.arm}: {s.why}" for s in bad))
    if not rig.at_park(a, q) and pen_down(rig, a, q):      # as the park job: lift it first
        up = lift_pens(st, scene, now, [a])[0]
        if up.why:
            return Plan(a, phase, [], np.zeros((0, 2)), [], None, up.why)
        steps.append(Step(a, phase, up.motions, up.verdicts, standing))
        q = up.motions[-1].q_end
    if not rig.at_park(a, q):          # standing low over the table: straight up first
        from aris.server.steps import checked_step, rise_first
        rise = rise_first(st, (obs, standing, phase), q, a)
        if rise is not None:
            s = checked_step(st, a, [rise], phase, q, standing)
            if not s.why:
                steps.append(s)
                q = rise.q_end
    arm, gates = rig.arm(a), rules.gates
    guard = Guard(arm, obs, gates)
    paper = rig.paper(a, for_planning=True)
    pts = grid_points(rig, a, st.drawing_area, cfg, st.drawing_centre) if points is None \
        else np.asarray(points, float).reshape(-1, 2)
    free_obs = obs if paper_dz == 0.0 else replace(obs, planes=tuple(
        replace(p, offset=p.offset + paper_dz) if p.kind == "paper" else p for p in obs.planes))
    spin, paths, dropped = _choose_spin(rig, arm, guard, paper, a, pts, cfg, gates)
    kept, touches = [], []
    for k in sorted(paths):
        go = free_plan(arm, q, paths[k][-1], free_obs, rules, seed_extra=b"calibrate")
        touch = touch_motion(arm, guard, paths[k], rules, cfg) if not isinstance(go, Refusal) \
            else f"free-space planner: {go.reason}: {go.detail}"
        if isinstance(go, Refusal) or isinstance(touch, str):
            dropped.append(k)
            continue
        v = check(st.config_dir, a, go, phase, q, standing=standing)
        ok, numbers, _ = check_touch(st.config_dir, a, touch, phase, go.q_end, standing)
        if not v.passed or not ok:
            dropped.append(k)
            continue
        steps += [Step(a, phase, (go,), (v,), standing),
                  Step(a, phase, (replace(touch, checked=numbers),), (None,), standing)]
        q, kept = touch.q_end, kept + [k]
    out = lambda why, st_=(): Plan(a, phase, list(st_), pts[kept],
                                   [tuple(map(float, pts[k])) for k in dropped], spin, why)
    if not kept:
        return out("no point can be touched")
    if not home:
        return out("", steps) if len(kept) >= min_points else \
            out(f"only {len(kept)} points can be touched (at least {min_points})")
    home = free_plan(arm, q, rig.park_q(a), free_obs, rules, seed_extra=b"calibrate home")
    if isinstance(home, Refusal):
        return out(f"no way back to the park: {home.reason}: {home.detail}")
    v = check(st.config_dir, a, home, phase, q, standing=standing)
    if not v.passed:
        return out("the way back to the park fails the checker: " + ", ".join(v.failed))
    steps.append(Step(a, phase, (home,), (v,), standing))
    if len(kept) < min_points:
        return out(f"only {len(kept)} points can be touched (at least {min_points})")
    return out("", steps)


# --------------------------------------------------------------------------- the job


UNCAL_HOVER = 0.150      # m above the nominal paper: the first hover of an arm whose height
                         # is not measured yet (no passing base part).  60 mm until 2026-10-09:
                         # the table may stand centimetres above the nominal one, and the
                         # flight to the hover is planned clear of a paper up to
                         # UNCAL_HOVER - FOUND_HOVER - FOUND_SLACK above it
UNCAL_DEPTH = 0.040      # m past the nominal paper its first touch may go
FOUND_TILT = 0.050       # m: how far above the FIRST contact the paper may stand elsewhere on
                         # the grid.  An unmeasured arm is tilted against the table (that is
                         # what this job measures): 1L, 2026-10-09, 23 mm/m in x and 22 in y,
                         # the paper 18 mm higher at the far points than at the first one.
                         # With a flat paper at the first contact's height the later flights
                         # had no clearance there and one ended on the table.
FOUND_HOVER = 0.020      # m above that highest paper: every later hover of the job
FOUND_SLACK = 0.005      # m: the free moves keep their pen clearance from a paper this much
                         # below it (the hovers sit just at that clearance)


def base_applied(st, arm) -> bool:
    return str(st.rig.calibration_status(arm).get("base", "")).startswith("applied")


def _staged_work(arm, cfg):
    """The calibrate job of an arm whose height is not measured (no passing base part): one
    touch first, from UNCAL_HOVER above the nominal paper and down to UNCAL_DEPTH below it
    (the free move there kept as high as that hover allows: the paper may be well above the
    nominal one);
    then, from where the arm really stands, every other point from a hover FOUND_TILT +
    FOUND_HOVER above the height that touch found, the free moves kept clear of a paper up to
    FOUND_TILT above it (the arm's tilt is not known yet), each touch straight down."""
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
        pts = grid_points(st.rig, arm, st.drawing_area, cfg, st.drawing_centre)
        wide = replace(cfg, hover=UNCAL_HOVER, extra_depth=UNCAL_DEPTH)
        first, k0 = None, 0
        for k0 in range(len(pts)):
            first = plan_calibrate(st, arm, where, wide, points=pts[k0:k0 + 1],
                                   name="calibrate first", min_points=1, home=False,
                                   paper_dz=UNCAL_HOVER - FOUND_HOVER - FOUND_SLACK)
            if not first.why:
                break
        if first is None or first.why:
            job.end_phases("calibrate not planned")
            return runner.fail_job(st, rec, job, "no first touch: " + (first.why if first
                                                                      else "no points"))
        add_steps(job, rec, first.steps)
        box = {}
        th = threading.Thread(target=lambda: box.update(run=run_queued(st, rec, job, True,
                                                                         where)), daemon=True)
        th.start()
        rec.set_state("moving")
        why = wait_phase(rec, first.phase.name)
        rows = [r for r in rec.log.read() if r.get("event") == "contact"
                and r.get("phase") == first.phase.name and r.get("arm") == arm]
        plan = first
        if not why and not rows:
            why = (f"the first touch found no paper down to {UNCAL_DEPTH * 1e3:.0f} mm below "
                   "the nominal paper")
        if not why:
            q = np.asarray(rows[-1]["q"], float)
            dz = float(st.rig.to_table(arm, st.rig.arm(arm).tip(q[None])[0])[2] - st.rig.paper_z)
            top = UNCAL_HOVER - FOUND_TILT - FOUND_HOVER - FOUND_SLACK
            if dz > top:
                # nothing can be planned from a paper this close under the hover, and a real
                # paper there is unlikely: say what it is instead of "no point can be touched"
                why = (f"the first touch reports the paper {dz * 1e3:.0f} mm above the nominal "
                       f"one, only {(UNCAL_HOVER - dz) * 1e3:.0f} mm below the hover it started "
                       f"from: a contact in the air? (the touch's row on the robot PC has the "
                       f"detector's numbers; a paper really higher than {top * 1e3:.0f} mm "
                       f"above the nominal one needs a higher UNCAL_HOVER)")
        if not why:
            real = runner.where_now(st, need_all=True)
            # a hover must clear the nominal paper's planning clearance too (its touch rises
            # from the nominal paper): never lower than that plus FOUND_SLACK
            pl = st.rig.paper(arm, for_planning=True)
            low = float(pl.pen_margin if pl.pen_margin is not None else pl.margin) + FOUND_SLACK
            rest = np.delete(pts, k0, axis=0)
            second = plan_calibrate(st, arm, real if isinstance(real, dict) else where,
                                    replace(cfg, hover=max(dz + FOUND_TILT + FOUND_HOVER, low),
                                            extra_depth=UNCAL_DEPTH),
                                    points=rest, min_points=MIN_CONTACTS - 1,
                                    paper_dz=dz + FOUND_TILT - FOUND_SLACK)
            plan = Plan(arm, second.phase, first.steps + second.steps,
                        np.vstack([first.points_table, second.points_table]),
                        first.dropped + second.dropped, second.spin, second.why)
            if second.why:
                why = second.why
            else:
                add_steps(job, rec, second.steps)
        job.end_phases(why or "calibrate planned")
        th.join()
        run = box["run"]
        if why and run.status == "done":
            run.status, run.why = "failed", why
        rep, state, why2 = _job_report(st, rec, plan, run, time.perf_counter() - t0)
        runner.finish_job(rec, job.dir, rep, state, why2)
    return work


def submit_calibrate(st, store, arm: str, kind: str = "calibrate"):
    """Admit a calibrate job (`kind` "calibrate") or a touch-off ("touchoff") for `arm` and
    start it (refused like park: another job runs, or with the robot, no position reported)."""
    from aris.server import runner, touchoff
    from aris.server.steps import steps_work
    from aris.types import Refusal as _Refusal
    if arm not in st.rig.arm_ids:
        return _Refusal("no_arm", f"arm {arm} is not mounted on this rig ({st.rig.arm_ids})")
    cfg = st.calib_settings or CalibSettings()
    if kind == "touchoff":
        tcfg = cfg if base_applied(st, arm) else replace(cfg, hover=UNCAL_HOVER,
                                                         extra_depth=UNCAL_DEPTH)
        work = steps_work(lambda st_, where: touchoff.plan(st_, arm, where, tcfg),
                          touchoff.job_report)
    elif not base_applied(st, arm):
        work = _staged_work(arm, cfg)
    else:
        work = steps_work(lambda st_, where: plan_calibrate(st_, arm, where, cfg), _job_report)
    return runner.start(st, store, kind, f"{kind} {arm}", work, lambda rec: dict(arm=arm),
                        need_positions=True)


def _job_report(st, rec, plan, run, planning_s):
    state, why, result, written = _solve(st, rec, plan, run)
    return _report(st, rec, plan, run, result, written, planning_s, state, why), state, why


def touch_points(plan: Plan) -> dict:
    """(phase, queue index) of every touch motion of the arm -> its grid point (x, y)."""
    out, k, count = {}, 0, {}
    for s in plan.steps:
        name = s.phase.name if s.phase is not None else ""
        for m in s.motions:
            i = count[(name, s.arm)] = count.get((name, s.arm), -1) + 1
            if m.kind == "touch" and s.arm == plan.arm:
                out[(name, i)] = tuple(round(float(x), 4) for x in plan.points_table[k])
                k += 1
    return out


def misses(plan: Plan, rows, arm: str) -> list:
    """The grid points whose touch found no paper (a "contact" row is missing for them)."""
    key = lambda r: (r.get("phase") or "", r.get("index"))
    got = {key(r) for r in rows if r.get("event") == "contact" and r.get("arm") == arm}
    def tried(r):
        ev = r.get("event")
        return r.get("arm") == arm and (ev in ("motion done", "no contact")
                                        or (ev == "failed" and r.get("why") == "no contact"))
    ran = {key(r) for r in rows if tried(r)}
    return [p for i, p in touch_points(plan).items() if i in ran and i not in got]


def _solve(st, rec, plan, run):
    """The solver on the contact rows; a passing result is written and the rig reloaded.  A
    touch that met no paper is skipped; the job fails only below MIN_CONTACTS contacts."""
    from aris.calib import calibration_from_events
    from aris.calib.files import write_base
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
    from aris.server.robots import write_kwargs
    path = write_base(result, st.config_dir, **write_kwargs(st, write_base, arm))
    surface = paper.rebuild(st.config_dir)         # the table's height map, from every slot
    st.reload()                                    # the station runs on the new files now
    return "done", "", result, dict(base=str(path), paper_surface=surface)


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
               paper_surface=paper.describe(st.surface),
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
