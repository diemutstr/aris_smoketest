"""The gripper that holds the pen holder: home, open, close.  No ROS here; the robot side is a
small port (`GripperPort`: the franka_gripper node's actions and its joint states, rosarm.py).

The parameters are the old working ones (Aris_Kindt arm_orchestrator/arbiter.py), site.json
`gripper`: open to 70 mm at 0.10 m/s; close (a grasp) to 0 mm at 0.10 m/s with 70 N, inner
tolerance 0 and outer 80 mm.  Each command:
  - is a no-op (with a row) when it is done already: open when wider than `open_noop_m`
    (55 mm); close when this process's last command was a grasp that held and the width has
    not changed since;
  - reads the width back afterwards: jaws that moved less than `min_move_m` (0.5 mm) fail, even
    when the gripper said success.  libfranka reports success for a grasp that moved nothing
    (blocked, or the travel calibration lost: homing fixes the latter).
Homing opens the fingers fully and closes them again: it drops the pen holder.  It is done only
when asked.
"""
from __future__ import annotations

from dataclasses import dataclass, fields
from typing import Protocol


@dataclass(frozen=True)
class GripperSettings:
    open_width: float = 0.07
    open_speed: float = 0.10
    close_width: float = 0.0
    close_speed: float = 0.10
    close_force: float = 70.0
    epsilon_inner: float = 0.0
    epsilon_outer: float = 0.08
    open_noop_m: float = 0.055
    min_move_m: float = 0.0005

    @staticmethod
    def from_site(block: dict | None) -> "GripperSettings":
        names = {f.name for f in fields(GripperSettings)}
        return GripperSettings(**{k: float(v) for k, v in (block or {}).items() if k in names})


class GripperPort(Protocol):
    def width(self) -> float | None:
        """The jaws' width now (m; 2 x the finger joint), None without a reading."""

    def homing(self) -> str: ...

    def move(self, width: float, speed: float) -> str: ...

    def grasp(self, width: float, speed: float, force: float, inner: float,
              outer: float) -> str:
        """"" when the gripper says it succeeded, else its error."""


@dataclass
class GripResult:
    done: bool
    why: str = ""
    width_before: float | None = None
    width_after: float | None = None
    noop: bool = False


class Gripper:
    def __init__(self, port: GripperPort, settings: GripperSettings = GripperSettings(),
                 say=None):
        self.port, self.s = port, settings
        self.say = say or (lambda event, **f: None)
        self._held_at = None                 # the width our last grasp held at, None if not

    def run(self, verb: str, **params) -> GripResult:
        """home, open or close, with the site's parameters unless `params` say otherwise."""
        if verb not in ("home", "open", "close"):
            return GripResult(False, f"no gripper verb {verb!r} (home, open, close)")
        return getattr(self, verb)(**params)

    def home(self) -> GripResult:
        before = self.port.width()
        why = self.port.homing()
        self._held_at = None
        return self._after("home", before, why, check_move=False)

    def open(self, width: float | None = None, speed: float | None = None) -> GripResult:
        before = self.port.width()
        if before is None:
            return self._row("open", GripResult(False, "no gripper width reading (is the "
                                                       "gripper node up?)"))
        if before > self.s.open_noop_m:
            return self._row("open", GripResult(True, "already open", before, before, True))
        why = self.port.move(self.s.open_width if width is None else width,
                             self.s.open_speed if speed is None else speed)
        self._held_at = None
        return self._after("open", before, why)

    def close(self, width: float | None = None, speed: float | None = None,
              force: float | None = None, epsilon_inner: float | None = None,
              epsilon_outer: float | None = None) -> GripResult:
        before = self.port.width()
        if before is None:
            return self._row("close", GripResult(False, "no gripper width reading (is the "
                                                        "gripper node up?)"))
        if self._held_at is not None and abs(before - self._held_at) < self.s.min_move_m:
            return self._row("close", GripResult(True, "already grasping", before, before, True))
        s = self.s
        why = self.port.grasp(s.close_width if width is None else width,
                              s.close_speed if speed is None else speed,
                              s.close_force if force is None else force,
                              s.epsilon_inner if epsilon_inner is None else epsilon_inner,
                              s.epsilon_outer if epsilon_outer is None else epsilon_outer)
        r = self._after("close", before, why)
        self._held_at = r.width_after if r.done else None
        return r

    def _after(self, verb: str, before, why: str, check_move: bool = True) -> GripResult:
        after = self.port.width()
        r = GripResult(not why, why, before, after)
        if r.done and check_move and before is not None and after is not None \
                and abs(after - before) < self.s.min_move_m:
            r.done = False
            r.why = (f"the jaws did not move (still {after * 1000:.1f} mm) — blocked, or the "
                     f"travel calibration is lost: home the gripper")
        return self._row(verb, r)

    def _row(self, verb: str, r: GripResult) -> GripResult:
        mm = (lambda w: None if w is None else round(w * 1000, 1))
        self.say(f"gripper {verb}" + (" (nothing to do)" if r.noop else ""), ok=r.done, why=r.why,
                 width_before_mm=mm(r.width_before), width_after_mm=mm(r.width_after))
        return r
