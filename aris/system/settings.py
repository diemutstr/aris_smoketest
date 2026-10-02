"""The system planner's own knobs.  Everything else it needs arrives as an argument."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Settings:
    # ---- the drawable maps
    grid_step: float = 0.02        # m between grid points over the canvas
    n_spin: int = 8                # hand spins tried per grid point (45 degrees apart), no lean
    reach: float = 0.85            # m from an arm's axis; grid points further out are not tried

    # ---- the allocation
    sample_step: float = 0.004     # m between the points of a line judged against the maps
    # Law 3: a stretch goes to a fill phase only when the leader phases hold less than this
    # share of it.
    leader_share: float = 0.8

