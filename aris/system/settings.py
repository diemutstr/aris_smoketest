"""The system planner's own knobs.  Everything else it needs arrives as an argument."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Settings:
    # ---- the drawable maps
    grid_step: float = 0.02        # m between grid points over the canvas
    n_spin: int = 8                # hand spins tried per grid point (45 degrees apart), no lean
    reach: float = 0.85            # m from an arm's axis; grid points further out are not tried
    # A map is shrunk by this many grid steps before a line is judged against it, so a point
    # judged by its nearest grid point is only counted when every grid point around it passed.
    erode: int = 1

    # ---- the allocation
    sample_step: float = 0.004     # m between the points of a line judged against the maps
