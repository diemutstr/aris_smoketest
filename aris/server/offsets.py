"""The drawn-offsets calibration of one row, in one call (aris/calib/offsets.py does the maths).

The person has measured with a ruler, per shared spot of the row, R's mark minus L's mark in
table axes (x across, y along), in millimetres.
"""
from __future__ import annotations

import numpy as np


def calibrate_from_offsets(config_dir, row_slots, measured_mm, date=None):
    """Solve the row's two poses from `measured_mm` ({spot: (dx, dy)} mm) and, when it passes,
    write both slots' base parts (`files.write_mark_solution`).  The spots are the marks rig.json
    gives both slots of the row, at their nominal places.  -> (MarkSolution, written paths)."""
    from aris.calib.files import write_mark_solution
    from aris.calib.offsets import solve_offsets
    from aris.rig import Rig
    rig = Rig.load(config_dir)
    L, R = row_slots
    spots = {n: rig.marks[n][0] for n in rig.marks if set(rig.marks[n][1]) == {L, R}}
    measured = {n: np.asarray(v, float) * 1e-3 for n, v in measured_mm.items()}
    sol = solve_offsets(rig, spots, measured, (L, R))
    paths = write_mark_solution(rig, sol, config_dir, date) if sol.passed else []
    return sol, paths
