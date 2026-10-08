"""The crosses job (`aris crosses`, `POST /crosses`): the visual check after the mark
calibration (`aris mark`): both arms of a row draw at the same spots, and their shapes should
sit on each other.

For every spot two slots of one row share (rig.json `marks`: a mark whose sharers are that
row's L and R slot), each of them draws a small shape at the spot's nominal place, on the real
paper, with the pen in: the L slot a CROSS (two 30 mm strokes, along and across the table), the
R slot a CIRCLE (20 mm across, a 24-sided polygon).  Where the circle lies from the cross is
how far the two arms still disagree there.

Planning: every arm parked first (the park job's steps: pens lifted one arm per phase, then
each arm home), then one arm at a time, in rig order, a phase "crosses <slot>": its shapes on
the drawing surface (the paper's height map or the paper, less the pen's press) by the arm
planner (local planner and sequencer: set-downs, draws, lifts, free moves), from its park back
to its park, every motion checked as it is planned (the checker, `CheckVerify`).  A shape the
planner cannot draw whole fails the job before anything moves.

Directions, as `docs/FOR_THE_ARTIST.md` section 0 names them: the desk is at the table's +y end
(row 3); L = her left = table +x, R = her right = table −x (renamed 2026-10-08).
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from functools import partial
from pathlib import Path

import numpy as np

from aris.server.steps import Scene, Step, steps_work
from aris.types import Line, Refusal

CROSS_M = 0.030          # each stroke of the cross
CIRCLE_D_M = 0.020       # the circle's diameter
CIRCLE_SIDES = 24
SHAPE = {"L": "cross", "R": "circle"}

INSTRUCTION = ("The visual check of the mark calibration: at each spot the cross and the circle "
               "should sit on each other (the cross through the circle's centre). If one is off "
               "by more than a millimetre or two, run `aris mark` again for that row.")


def row_spots(rig, slots) -> list[dict]:
    """The spots the job draws: [{spot, xy_m, row, cross: L slot, circle: R slot}], rig.json
    order, for every mark shared by one row's L and R slot, both among `slots`."""
    out = []
    for name, (xy, sharers) in rig.marks.items():
        s = sorted(sharers)
        if len(s) == 2 and s[0][0] == s[1][0] and {x[1] for x in s} == {"L", "R"} \
                and set(s) <= set(slots):
            left, right = (x for side in "LR" for x in s if x[1] == side)
            out.append(dict(spot=name, xy_m=[float(v) for v in xy], row=s[0][0],
                            cross=left, circle=right))
    return out


def shape_lines(spot: str, xy, shape: str, z: float) -> list[Line]:
    """The shape at `xy` (table frame) on the drawing surface at height z."""
    x, y = float(xy[0]), float(xy[1])
    h = 0.5 * CROSS_M
    if shape == "cross":
        pts = [[(x - h, y), (x + h, y)], [(x, y - h), (x, y + h)]]
    else:
        a = np.linspace(0.0, 2.0 * np.pi, CIRCLE_SIDES + 1)
        r = 0.5 * CIRCLE_D_M
        pts = [list(zip(x + r * np.cos(a), y + r * np.sin(a)))]
    return [Line(f"{spot} {shape}" + (f" {i + 1}" if len(pts) > 1 else ""),
                 np.array([[px, py, z] for px, py in p], float), "table") for i, p in
            enumerate(pts)]


@dataclass
class CrossPlan:
    steps: list = field(default_factory=list)
    spots: list = field(default_factory=list)
    why: str = ""

    def failed(self) -> dict:
        return dict(spots=self.spots)


def plan_crosses(st, where, slots, spots) -> CrossPlan:
    from aris import arm_planner
    from aris.server.park import plan_park
    from aris.server.verify import CheckVerify
    rig, rules = st.rig, st.rules
    out = CrossPlan(spots=spots)
    parking = plan_park(st, where)
    bad = [s for s in parking if s.why and s.why != "already at its park"]
    if bad:
        out.why = "the arms cannot all be parked first: " + "; ".join(
            f"{s.arm}: {s.why}" for s in bad)
        return out
    out.steps += parking
    now = {a: rig.park_q(a) for a in rig.arm_ids}
    verify = CheckVerify(Path(st.config_dir))
    for a in (b for b in rig.arm_ids if b in slots):
        mine = [(s, SHAPE[a[1]]) for s in spots if a in (s["cross"], s["circle"])]
        if not mine:
            continue
        lines = []
        for s, shape in mine:
            x, y = s["xy_m"]
            z0 = rig.paper_z if st.surface is None else float(st.surface.z(x, y))
            lines += shape_lines(s["spot"], (x, y), shape, z0 - rules.press)
        obs, standing, phase = Scene(rig).of(a, now, (a,), (), f"crosses {a}")
        gen = arm_planner.plan(rig.arm(a), [rig.to_base(a, ln) for ln in lines], obs,
                               now[a], rules, q_end=rig.park_q(a), cache_dir=st.cache_dir,
                               verify=partial(verify, a, phase))
        motions = []
        while True:
            try:
                motions.append(next(gen))
            except StopIteration as e:
                left = list(e.value or ())
                break
        if left:
            out.why = f"{a} cannot draw its shapes whole: " + "; ".join(
                f"{x.piece.line_id}: {x.reason} {x.detail}" for x in left)
            return out
        unchecked = [m.kind for m in motions if not (m.checked or {}).get("passed")]
        if unchecked:
            out.why = f"{a}: the checker did not pass its {', '.join(unchecked)} motions"
            return out
        out.steps.append(Step(a, phase, tuple(motions), (None,) * len(motions),
                              dict(standing)))
    return out


def group_slots(rig, slots=(), group: str | None = None):
    """The slots of the job: the ones named, or a group's (default "all", or every controlled
    slot when the rig has no such group), in rig order."""
    if slots:
        bad = [x for x in slots if x not in rig.arm_ids]
        if bad:
            return Refusal("no_slot", f"slots {bad} are not controlled on this rig")
        return tuple(x for x in rig.arm_ids if x in slots)
    name = group or "all"
    if name in rig.mark_groups:
        return tuple(x for x in rig.arm_ids if x in rig.mark_groups[name])
    if group is None:
        return tuple(rig.arm_ids)
    return Refusal("no_group", f"no mark group {group!r}; groups are {tuple(rig.mark_groups)}")


def submit_crosses(st, store, slots=(), group: str | None = None):
    """Admit and start the crosses job: refused for slots or a group the rig does not have, or
    one with no row pair (nothing to draw)."""
    from aris.server import runner
    chosen = group_slots(st.rig, tuple(slots), group)
    if isinstance(chosen, Refusal):
        return chosen
    spots = row_spots(st.rig, chosen)
    if not spots:
        return Refusal("no_spots", f"slots {', '.join(chosen)} share no spot within a row "
                       "(a row's L and R slot both)")
    plan = lambda st_, where: plan_crosses(st_, where, chosen, spots)
    return runner.start(st, store, "crosses", "crosses " + " ".join(chosen),
                        steps_work(plan, _report),
                        lambda rec: dict(slots=list(chosen), group=group, spots=spots,
                                         shapes=dict(SHAPE)),
                        need_positions=True)


def _report(st, rec, plan, run, planning_s):
    if rec.stop.is_set():
        state, why = "stopped", "stop requested"
    elif run.status != "done":
        state, why = "failed", run.why
    else:
        state, why = "done", ""
    rep = dict(state=state, why=why, kind="crosses", spots=plan.spots, shapes=dict(SHAPE),
               cross_m=CROSS_M, circle_diameter_m=CIRCLE_D_M,
               instruction=INSTRUCTION,
               phases=[dict(name=n, end_check_passed=bool(p), tightest=t, clearance_m=c)
                       for n, p, t, c in run.phase_ends],
               motions=sum(len(s.motions) for s in plan.steps), planning_s=planning_s,
               finished_at=time.time())
    return rep, state, why
