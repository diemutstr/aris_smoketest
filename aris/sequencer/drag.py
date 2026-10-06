"""Drag-only pens: which way a piece may be drawn.

A pen that leans in its holder (the lateral holder: about 23 degrees) skids when a stroke pushes
its tip forward, toward the side the tip leans to, and draws when it is pulled.  With
`rules.drag_only` a piece is drawn in the direction in which the pen is pulled: at every step
the drawing direction's component along the pen's lean (the pen axis laid into the paper plane,
pointing from the hand toward the tip) is at most zero, i.e. the tip trails.
"""
from __future__ import annotations

import numpy as np

from aris.types import DrawPlan

TOL = 1e-9            # m of tip travel along the lean per step still counted as "not pushed"


def pulled_shares(arm, plan: DrawPlan, normal: np.ndarray) -> tuple[float, float]:
    """(share of the piece's length on which the pen is pulled when drawn as the plan runs,
    the same drawn backwards).  A step across the lean counts for both."""
    n = np.asarray(normal, float) / np.linalg.norm(normal)
    step = np.diff(plan.tip_base, axis=0)                               # (N-1, 3) m
    lean = arm.pen_axis(plan.q)
    lean = lean - (lean @ n)[:, None] * n
    lean = 0.5 * (lean[:-1] + lean[1:])                                 # at mid-step
    along = np.einsum("ij,ij->i", step, lean)
    ln = np.linalg.norm(step, axis=1)
    total = max(float(ln.sum()), 1e-15)
    fwd = float(ln[along <= TOL * np.linalg.norm(lean, axis=1)].sum()) / total
    back = float(ln[-along <= TOL * np.linalg.norm(lean, axis=1)].sum()) / total
    return fwd, back
