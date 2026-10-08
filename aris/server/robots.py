"""Which robot a slot's calibration was measured on, against which robot is in that slot today.

A calibration part (`base`, `pen`) belongs to a physical arm.  When the slots are renamed or the
arms swapped, a part can end up under a slot whose arm it was not measured on — the planner
would then place that arm where another one hangs.  Parts written by this server carry the
slot's `robot` (from the site table given to `aris serve --site`, or the name the operator PC
reports); a drawing, park or crosses job is refused while any part's `robot` differs from the
robot in its slot today.  Parts without `robot` (written before) are not judged.
"""
from __future__ import annotations

import inspect


def today(st, slot) -> str | None:
    """The robot in `slot` today: as the operator PC names it, else the site table's."""
    p = st.positions.all().get(slot) if st.positions is not None else None
    return (p or {}).get("robot") or st.robots.get(slot)


def mismatch(st) -> str:
    """Why the calibration does not belong to today's arms ("" when it does, or is unknown)."""
    from aris.calib import files
    for slot in st.rig.arm_ids:
        now = today(st, slot)
        cal = files.read(st.config_dir, slot) or {}
        for part in ("base", "pen"):
            was = (cal.get(part) or {}).get("robot")
            if now and was and was != now:
                f = files.calibration_file(st.config_dir, slot)
                return (f"{f.parent.name}/{f.name} was measured on {was}, but {slot} is {now} "
                        "today: recalibrate, or rename the files if the slots were renamed")
    return ""


def write_kwargs(st, writer, slot) -> dict:
    """`robot=` for a calib writer that takes it (the slot's robot today), else nothing."""
    robot = today(st, slot)
    if robot and "robot" in inspect.signature(writer).parameters:
        return dict(robot=robot)
    return {}
