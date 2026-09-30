"""The phases the system planner runs, in order.

A phase is who moves, who stands parked, and the walls between those who move.  Phases 1 and 2
come from the rig (the leaders of each row, diagonal walls between them).  After them comes one
fill phase per arm: that arm alone, every other arm parked, no walls.  The system planner skips
a phase in which nobody has anything to do.
"""
from __future__ import annotations

from aris.types import Phase

LEADER_PHASES = (1, 2)


def fill(rig, arm_id: int) -> Phase:
    """One arm alone; every other arm stands parked."""
    return Phase(f"fill {arm_id}", (arm_id,), tuple(a for a in rig.arm_ids if a != arm_id), ())


def phases(rig) -> list[Phase]:
    """Every phase, in the order they run: 1, 2, then one fill phase per arm (rig order)."""
    return [rig.phase(n) for n in LEADER_PHASES] + [fill(rig, a) for a in rig.arm_ids]


def phase_named(rig, name: str) -> Phase:
    """The phase with this name (what `plan` tags every motion with)."""
    for p in phases(rig):
        if p.name == name:
            return p
    raise KeyError(f"no phase {name!r}")


def is_fill(phase: Phase) -> bool:
    return phase.name.startswith("fill ")
