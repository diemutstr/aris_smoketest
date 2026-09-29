"""The local planner's own knobs.  Everything else it needs arrives as an argument.

The defaults are the values the numbers in docs/modules/local.md were measured with.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

SPIN_STEP = 2.0 * np.pi / 24


@dataclass(frozen=True)
class Settings:
    # ---- the graph
    step: float = 0.015            # m along the line between two layers
    n_spin: int = 24               # hand spins about the paper normal, a full circle (15 deg apart)
    # Elbow values (joint 7) are spaced like the spins.  At zero lean joint 7 turns about the
    # paper normal too, so turning the spin and joint 7 by one step together is a small move:
    # the diagonal neighbours of the graph are its cheap edges.
    q7_step: float = SPIN_STEP     # rad
    lean_rings: int = 2            # lean magnitudes lean_max * r / lean_rings, r = 1 .. rings
    lean_dirs: int = 6             # lean directions per magnitude
    jump: float = 0.35             # rad, the most any joint may move along one edge
    # Everything bolted to the hand (hand, finger blades, holder, pencil tail) sits a few
    # millimetres above the paper while the pen draws, so while drawing it must only not touch
    # the paper; the arm's links keep the paper's own margin.  See docs/modules/local.md.
    hand_paper_margin: float = 0.0  # m

    # ---- the kinematic table (used when the caller gives a cache directory)
    table_dr: float = 0.01         # m between tabulated distances of the tip from the base axis
    table_r_max: float = 0.95      # m, the farthest tabulated distance

    # ---- the lazy obstacle check (optimisation note 4)
    lazy: bool = True              # check nodes against the obstacles only on the routes used
    lazy_rounds: int = 12          # searches per route before every surviving node is checked

    # ---- the search
    lift_cost: float = 5.0         # rad of joint motion one lift is worth
    gap_cost: float = 1000.0       # per layer step left undrawn; far above any drawing route
    collar: float = 0.06           # m of line opened to the lean around a place the narrow plan fails

    # ---- the exact path
    dense_step: float = 0.002      # m between samples of the exact path, at most
    chord_tol: float = 2e-5        # m the pen may leave the line between two exact samples
    max_refine: int = 8            # halvings of an exact step where the pen leaves the line more
    smooth_layers: float = 2.0    # layers over which the route's spin, lean, elbow value are averaged
    dense_jump: float = 0.10       # rad, the most any joint may move between two exact samples
    tip_tol: float = 1e-6          # m, pen tip off the line
    max_repairs: int = 40          # searches again after an exact path failed, per line
    corner_angle: float = np.deg2rad(30.0)  # the pen stops where the line turns more than this
    timing_deviation: float = 1.5e-4  # rad the timed path may leave the exact one (kernel.retime)

    # ---- the alternatives
    n_alternatives: int = 4        # plans per piece, at most
    spin_sectors: int = 4          # spin at an end counts as different when in another quarter turn
    distinct: float = 0.5          # rad; two plans differ when a joint differs this much at an end
