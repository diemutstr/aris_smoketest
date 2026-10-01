"""The pen force: how hard to press, when, and whether the readings make sense.  No ROS.

The controller adds a force at the pen tip (J^T F); this module decides its size along the
paper normal at every moment of a motion, and reads the arm's own force estimate back:

  tare      the air zero: the estimate is not zero in the air and drifts with the pose, so
            it is measured standing still just before each landing and subtracted
  contact   "the moment the force lifts off the air zero is the table" (Diemut): the force
            above the air zero stays over `contact_n` for `contact_ticks` readings in a row
  setpoint  intensity 0..1 -> the band (0.7 to 1.0 N for graphite), in `levels` steps

Band, levels, cap, ramps and servo are facts of pen and paper: rig.json's `pen` block, which
the server copies into every job header; the runner applies the header's.  The tare limits,
the contact detection and each arm's force sign are facts of this site: site.json.
  ramp      zero while lowering; from zero to the setpoint over the first `ramp_m` of a
            drawing motion; back to zero over the first `lift_ramp_s` of the lift
  guard     the force above the air zero over `cap_n` for `cap_ticks` readings in a row:
            the arm holds (a plausibility check, not the paper's protection)
  servo     a slow correction of the fed-forward force toward the setpoint, from the reading
            (on by default: `servo_ki` 1/s, a 1 s time constant; bounded to +-`trim_max_n`)

Sign: `normal_force` is positive when the paper pushes the pen up (along the paper normal,
which points into free space).  The force the arm applies is along minus the normal.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from aris.types import Refusal


# rig.json `pen` block key -> ForceSettings field
PEN_KEYS = {"force_band_n": "band_n", "force_levels": "levels", "force_cap_n": "cap_n",
            "force_ramp_m": "ramp_m", "lift_ramp_s": "lift_ramp_s", "servo_ki_per_s": "servo_ki",
            "servo_trim_max_n": "trim_max_n"}


@dataclass(frozen=True)
class ForceSettings:
    band_n: tuple[float, float] = (0.7, 1.0)
    levels: int = 9
    cap_n: float = 3.5
    ramp_m: float = 0.002
    lift_ramp_s: float = 0.2
    tare_s: float = 0.2
    tare_max_n: float = 8.0
    tare_spread_n: float = 0.6       # largest max - min of the readings for a still arm
    contact_n: float = 0.25
    contact_ticks: int = 3
    cap_ticks: int = 12
    servo_ki: float = 1.0            # 1/s: the trim closes the force error with a 1 s time constant
    trim_max_n: float = 1.0
    sign: float = 1.0

    @staticmethod
    def from_parts(pen: dict, site_force: dict | None = None, sign: float = 1.0) -> "ForceSettings":
        """`pen`: the rig's `pen` block, as the job header carries it (facts of pen and paper,
        the same on both machines).  `site_force`: site.json's `force` block (the tare and the
        contact detection of this site).  `sign`: this arm's force sign (site.json)."""
        kw = {f: pen[k] for k, f in PEN_KEYS.items() if k in pen}
        site_names = {"tare_s", "tare_max_n", "tare_spread_n", "contact_n", "contact_ticks",
                      "cap_ticks"}
        kw.update({k: v for k, v in (site_force or {}).items() if k in site_names})
        if "band_n" in kw:
            lo, hi = (float(x) for x in kw["band_n"])
            if not 0.0 <= lo <= hi:
                raise ValueError(f"force band {kw['band_n']} is not low <= high")
            kw["band_n"] = (lo, hi)
        s = ForceSettings(sign=float(sign), **kw)
        if s.band_n[1] > s.cap_n:
            raise ValueError("the force cap is below the top of the band")
        return s


def intensity_to_force(intensity: float, s: ForceSettings) -> float:
    """0..1 -> newtons in the band, rounded to one of `levels` steps (the old `_target_force`)."""
    f = min(max(float(intensity), 0.0), 1.0)
    if s.levels > 1:
        f = round(f * (s.levels - 1)) / (s.levels - 1)
    return s.band_n[0] + f * (s.band_n[1] - s.band_n[0])


def normal_force(F_ext_base, normal_base, sign: float = 1.0) -> float:
    """The external force on the arm (base frame) along the paper normal, times `sign`."""
    return float(sign) * float(np.dot(np.asarray(F_ext_base, float), np.asarray(normal_base)))


def profile(kind: str, t_knots, s_knots, intensity: float, s: ForceSettings,
            f_start: float = 0.0):
    """-> f(t): the force setpoint (N, pressing) at motion time t (from the motion's start).

    `s_knots`: arc length of the pen tip at the knots (drawing motions), or None.
    lower, free: zero.  draw: the setpoint, ramped in over the first `ramp_m` of arc length
    (over 0.1 s when the tips are unknown).  lift: from `f_start` to zero over `lift_ramp_s`.
    """
    t_knots = np.asarray(t_knots, float) - float(t_knots[0])
    if kind == "draw":
        top = intensity_to_force(intensity, s)
        if s_knots is None or s.ramp_m <= 0.0:
            return lambda t: top * np.clip(np.asarray(t, float) / 0.1, 0.0, 1.0)
        arc = np.asarray(s_knots, float)
        return lambda t: top * np.clip(np.interp(t, t_knots, arc) / s.ramp_m, 0.0, 1.0)
    if kind == "lift" and f_start > 0.0 and s.lift_ramp_s > 0.0:
        return lambda t: f_start * np.clip(1.0 - np.asarray(t, float) / s.lift_ramp_s, 0.0, 1.0)
    return lambda t: np.zeros_like(np.asarray(t, float))


def arc_length(tip_base) -> np.ndarray:
    """(N, 3) pen tips -> (N,) arc length from the first."""
    tip = np.asarray(tip_base, float)
    return np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(tip, axis=0), axis=1))])


class Tare:
    """The air zero: the mean reading while the arm stands still in the air."""

    def __init__(self, s: ForceSettings):
        self.s, self.readings = s, []

    def add(self, f: float) -> None:
        self.readings.append(float(f))

    def result(self) -> float | Refusal:
        r = np.asarray(self.readings)
        if len(r) < 3:
            return Refusal("no_tare", f"only {len(r)} force readings in the air")
        zero = float(r.mean())
        if abs(zero) > self.s.tare_max_n:
            return Refusal("tare_too_large",
                           f"the force reads {zero:.2f} N in the air (limit {self.s.tare_max_n})")
        if float(r.max() - r.min()) > self.s.tare_spread_n:
            return Refusal("tare_unsteady", f"the air reading moves by {r.max() - r.min():.2f} N "
                           f"(limit {self.s.tare_spread_n}); is the arm still?")
        return zero


class Contact:
    """Force onset against the air zero.  `update` returns True from the confirming reading on;
    `at` is the time of the first reading of the run that confirmed it."""

    def __init__(self, s: ForceSettings):
        self.s, self.run, self.first, self.at = s, 0, None, None

    def update(self, f_rel: float, t: float) -> bool:
        if self.at is not None:
            return True
        if f_rel > self.s.contact_n:
            self.first = t if self.run == 0 else self.first
            self.run += 1
            if self.run >= self.s.contact_ticks:
                self.at = self.first
        else:
            self.run = 0
        return self.at is not None


class Guard:
    """Over the cap for `cap_ticks` readings in a row -> the reason to hold, else ""."""

    def __init__(self, s: ForceSettings):
        self.s, self.run = s, 0

    def update(self, f_rel: float) -> str:
        self.run = self.run + 1 if f_rel > self.s.cap_n else 0
        if self.run >= self.s.cap_ticks:
            return (f"pen force {f_rel:.2f} N above the cap {self.s.cap_n} N "
                    f"for {self.run} readings")
        return ""


class Servo:
    """A slow integral correction of the fed-forward force, only while in contact."""

    def __init__(self, s: ForceSettings):
        self.s, self.trim = s, 0.0

    def update(self, target: float, f_rel: float, dt: float, in_contact: bool) -> float:
        if self.s.servo_ki > 0.0 and in_contact and target > 0.0:
            self.trim += self.s.servo_ki * (target - f_rel) * float(dt)
            self.trim = float(np.clip(self.trim, -self.s.trim_max_n, self.s.trim_max_n))
        return self.trim
