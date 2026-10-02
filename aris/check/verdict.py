"""The checker's answer: plain data, printable as a short table."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

import numpy as np


@dataclass(frozen=True)
class Measurement:
    """One measured number against its limit.  kind "max": value must not exceed the limit;
    kind "min": value must not fall below it."""
    name: str
    value: float
    limit: float
    kind: Literal["max", "min"]
    unit: str
    passed: bool
    detail: str = ""
    ranked: bool = True        # False: a yes/no fact, never "the tightest" unless it fails

    @property
    def used(self) -> float:
        """How much of the allowance is used: 1.0 is exactly at the limit, above 1 fails.
        NaN where a ratio means nothing (a limit of zero)."""
        v, lim = self.value, self.limit
        if self.kind == "max":
            return v / lim if lim > 0 else float("nan")
        if lim <= 0:
            return float("nan")
        return lim / v if v > 0 else float("inf")


def measure(name, value, limit, kind, unit, detail="", tol=0.0, ranked=True) -> Measurement:
    """`tol` is a numerical allowance on the comparison (not a relaxation of the limit).
    NaN always fails; +inf passes a "min" (nothing there to measure against)."""
    value = float(value)
    if np.isnan(value):
        ok = False
    elif kind == "max":
        ok = value <= limit + tol
    else:
        ok = value >= limit - tol
    return Measurement(name, value, float(limit), kind, unit, bool(ok), detail, ranked)


@dataclass(frozen=True)
class Verdict:
    passed: bool
    measurements: tuple[Measurement, ...]
    tightest: str               # the measurement that uses most of its allowance (or fails)
    min_clearance: float        # m beyond the demanded clearance, over every obstacle class
    min_clearance_at: str       # which class and which pair
    notes: tuple = ()           # how the rig was read: what fell back to nominal, and why

    def get(self, name: str) -> Measurement:
        for m in self.measurements:
            if m.name == name:
                return m
        raise KeyError(name)

    @property
    def failed(self) -> tuple[str, ...]:
        return tuple(m.name for m in self.measurements if not m.passed)

    def __str__(self) -> str:
        head = "PASS" if self.passed else "FAIL: " + ", ".join(self.failed)
        rows = [head, f"tightest: {self.tightest}; smallest clearance beyond demanded "
                      f"{self.min_clearance * 1e3:.2f} mm ({self.min_clearance_at})",
                f"{'':2}{'measurement':32}{'value':>12}{'limit':>12}  unit"]
        for m in self.measurements:
            sign = "<=" if m.kind == "max" else ">="
            rows.append(f"{'  ' if m.passed else '! '}{m.name:32}{m.value:12.5g}"
                        f"{sign:>3}{m.limit:9.4g}  {m.unit}  {m.detail}")
        rows += [f"note: {n}" for n in self.notes]
        return "\n".join(rows)


def verdict(measurements, min_clearance=float("nan"), min_clearance_at="",
            notes=()) -> Verdict:
    ms = tuple(measurements)
    failed = [m for m in ms if not m.passed]
    ranked = [m for m in ms if m.ranked and not np.isnan(m.used)]
    pool = failed or ranked
    tight = max(pool, key=lambda m: m.used if not np.isnan(m.used) else np.inf).name \
        if pool else ""
    return Verdict(not failed and bool(ms), ms, tight, float(min_clearance), min_clearance_at,
                   tuple(notes))
