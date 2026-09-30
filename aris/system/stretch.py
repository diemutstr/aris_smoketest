"""A stretch: part of one input line, by arc length along that line, in the table frame.

Everything the system planner moves around (the pool of what is still to draw, what an arm is
given, what it hands back) is a stretch, so its arc lengths always refer to the input line and
the no-drop account can add them up.
"""
from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from aris.local import polyline
from aris.types import Line, Piece


@dataclass(frozen=True)
class Stretch:
    line_id: str
    s0: float                          # m along the input line
    s1: float
    points: np.ndarray                 # (M, 3) table frame, from s0 to s1, repeats removed
    intensity: float = 1.0
    target: tuple | None = None        # (phase index, arm id) it is allocated to, or None
    reason: str = ""                   # why it came back from an arm, if it did
    detail: str = ""

    @property
    def length(self) -> float:
        return self.s1 - self.s0

    @property
    def piece(self) -> Piece:
        return Piece(self.line_id, float(self.s0), float(self.s1))

    def arc(self) -> np.ndarray:
        """(M,) arc length along the input line at each point."""
        d = np.linalg.norm(np.diff(self.points, axis=0), axis=1)
        return self.s0 + np.concatenate([[0.0], np.cumsum(d)])

    def sub(self, a: float, b: float) -> "Stretch":
        """The part from a to b (arc lengths along the input line), exact end points."""
        s = self.arc()
        a, b = float(max(a, self.s0)), float(min(b, self.s1))
        inner = self.points[(s > a + polyline.DUP_TOL) & (s < b - polyline.DUP_TOL)]
        p = np.vstack([polyline.at(self.points, s, a)[None], inner,
                       polyline.at(self.points, s, b)[None]])
        return replace(self, s0=a, s1=b, points=p, target=None, reason="", detail="")

    def samples(self, step: float) -> tuple[np.ndarray, np.ndarray]:
        """Arc lengths (n,) and points (n, 3), both ends and every corner in, at most `step` apart."""
        s = self.arc()
        u = polyline.dense_positions(self.s0, self.s1, s, step)
        return u, polyline.at(self.points, s, u)

    def as_line(self, line_id: str) -> Line:
        return Line(line_id, self.points, "table", self.intensity)


def of_line(line: Line) -> Stretch:
    """The whole line as a stretch (non-finite and repeated points dropped)."""
    if line.frame != "table":
        raise ValueError(f"line {line.id} is in the {line.frame} frame; the system planner "
                         "takes table-frame lines")
    p, s = polyline.clean(line.points)
    if len(p) == 0:
        p = np.zeros((1, 3))
    return Stretch(line.id, 0.0, float(s[-1]), p, line.intensity)


def line_length(line: Line) -> float:
    _, s = polyline.clean(line.points)
    return float(s[-1]) if len(s) else 0.0
