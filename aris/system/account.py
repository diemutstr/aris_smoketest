"""The no-drop account: every line of the drawing is either drawn or left over, and said so.

The account is taken from what was produced, never from what was planned: drawn means a drawing
motion whose `piece` covers that stretch.  Per input line, the drawn pieces and the leftovers
together must cover the line from end to end, and nothing may lie outside it.  Two of them may
overlap only by a join (a cut reaches back `min_piece` over the join, so the join is drawn
twice rather than not at all); any larger overlap means a stretch was handed out twice.

A violation is a bug in the planner, not a refusal, so it raises.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from aris.system.stretch import line_length
from aris.types import DrawRules

TOL = 1e-6          # m


class NoDropViolation(AssertionError):
    """The drawn pieces and the leftovers do not account for a line exactly once."""


@dataclass
class LineAccount:
    length: float
    drawn: float            # length covered by drawing motions
    left: float             # length covered by leftovers and not drawn
    twice: float            # length drawn more than once (the joins)
    left_by_reason: dict = field(default_factory=dict)


@dataclass
class Account:
    lines: dict             # line id -> LineAccount
    length: float
    drawn: float
    left: float
    twice: float
    left_by_reason: dict    # reason -> metres (not drawn)


def _union(iv: list) -> list:
    out = []
    for a, b in sorted(iv):
        if out and a <= out[-1][1] + TOL:
            out[-1][1] = max(out[-1][1], b)
        else:
            out.append([a, b])
    return out


def _measure(iv) -> float:
    return float(sum(b - a for a, b in iv))


def _minus(iv, cut) -> list:
    """Intervals `iv` with the union `cut` removed."""
    out = []
    for a, b in iv:
        segs = [(a, b)]
        for c, d in cut:
            segs = [piece for x, y in segs for piece in ((x, min(y, c)), (max(x, d), y))
                    if piece[1] - piece[0] > TOL]
        out += segs
    return out


def _worst_overlap(iv) -> float:
    iv = sorted(iv)
    worst, reach = 0.0, -np.inf
    for a, b in iv:
        worst = max(worst, min(reach, b) - a)
        reach = max(reach, b)
    return worst


def account(lines, motions, leftovers, join: float = DrawRules().min_piece) -> Account:
    """`motions`: Motion or (phase, arm, Motion); `leftovers`: Leftover.  Raises
    NoDropViolation when a line is not covered end to end, when something lies outside a
    line or belongs to no line, or when two stretches overlap by more than `join`."""
    lengths = {x.id: line_length(x) for x in lines}
    if len(lengths) != len(lines):
        raise NoDropViolation("two input lines share an id")
    drawn = {k: [] for k in lengths}
    left = {k: [] for k in lengths}
    for m in motions:
        m = m[-1] if isinstance(m, tuple) else m
        if m.kind == "draw":
            _add(drawn, lengths, m.piece, m.piece, "drawing motion")
    for x in leftovers:
        _add(left, lengths, x, x.piece, f"leftover ({x.reason})")
    out, tot = {}, dict(length=0.0, drawn=0.0, left=0.0, twice=0.0)
    by_reason_all = {}
    for k, L in lengths.items():
        d_iv = [(p.s0, p.s1) for p in drawn[k]]
        l_iv = [(x.piece.s0, x.piece.s1) for x in left[k]]
        both = _union(d_iv + l_iv)
        if L > TOL and (not both or both[0][0] > TOL or both[-1][1] < L - TOL or len(both) > 1):
            raise NoDropViolation(f"line {k} ({L:.4f} m) is not covered end to end: "
                                  f"covered {[(round(a, 4), round(b, 4)) for a, b in both]}")
        for name, iv in (("drawn pieces", d_iv), ("leftovers", l_iv),
                         ("drawn pieces and leftovers", d_iv + l_iv)):
            w = _worst_overlap(iv)
            if w > join + TOL:
                raise NoDropViolation(f"line {k}: two {name} overlap by {w:.4f} m "
                                      f"(a join is at most {join:.4f})")
        d_u = _union(d_iv)
        drawn_len = _measure(d_u)
        by_reason = {}
        for x in left[k]:
            m = _measure(_minus([(x.piece.s0, x.piece.s1)], d_u))
            by_reason[x.reason] = by_reason.get(x.reason, 0.0) + m
            by_reason_all[x.reason] = by_reason_all.get(x.reason, 0.0) + m
        la = LineAccount(L, drawn_len, L - drawn_len if L > TOL else 0.0,
                         _measure([(a, b) for a, b in d_iv]) - drawn_len, by_reason)
        out[k] = la
        for f in tot:
            tot[f] += getattr(la, f)
    return Account(out, tot["length"], tot["drawn"], tot["left"], tot["twice"], by_reason_all)


def _add(book, lengths, item, piece, what):
    if piece is None:
        raise NoDropViolation(f"a {what} names no piece")
    if piece.line_id not in lengths:
        raise NoDropViolation(f"a {what} names line {piece.line_id!r}, which is not in the drawing")
    L = lengths[piece.line_id]
    if piece.s0 < -TOL or piece.s1 > L + TOL or piece.s1 < piece.s0 - TOL:
        raise NoDropViolation(f"a {what} of line {piece.line_id} ({L:.4f} m) spans "
                              f"{piece.s0:.4f}..{piece.s1:.4f}")
    book[piece.line_id].append(item)
