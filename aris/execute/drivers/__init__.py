"""The driver: the handful of verbs one arm offers the executor.

The executor knows nothing else about the arm.  The simulated arm (`sim.py`) and the real one
(`ros.py`, on the operator PC, later) offer the same verbs, so everything above runs without
hardware.  Only this folder may know about ROS, controllers and topics.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol, runtime_checkable

import numpy as np

from aris.types import Motion, Trajectory


@dataclass(frozen=True)
class ArmState:
    """What the arm reports now.  `ok` False means it will not move until `recover()`;
    `flags` say why and what it is doing (e.g. "moving", "holding", "stopped", "fault: ...")."""
    q: np.ndarray                  # (7,) rad
    qd: np.ndarray                 # (7,) rad/s
    ok: bool
    flags: tuple[str, ...] = ()


@dataclass(frozen=True)
class Result:
    """The answer of `move`, `draw` and `recover`: done, or failed with why."""
    done: bool
    why: str = ""
    q: np.ndarray | None = None    # (7,) where the arm was when it finished or gave up

    @staticmethod
    def ok(q=None) -> "Result":
        return Result(True, "", q)

    @staticmethod
    def failed(why: str, q=None) -> "Result":
        return Result(False, why, q)


@runtime_checkable
class Driver(Protocol):
    """One arm.  Every verb may be called from any thread; `move` and `draw` block until the
    motion is finished or has failed, and `stop` (from another thread) makes them return."""

    arm_id: int

    def state(self) -> ArmState:
        """Joint positions, velocities and whether the arm is able to move."""

    def move(self, traj: Trajectory) -> Result:
        """A free motion (pen up).  Position control: fly the timed joint trajectory exactly
        as it is, at its own timing (never faster or slower: the checker's verdict holds at
        that timing only)."""

    def draw(self, motion: Motion) -> Result:
        """A motion with the pen at the paper at some point: kind "draw", "lower" or "lift".
        The real arm reads the kind to know the pen state and ramps the pen force (up on a
        lower, held on a draw, down on a lift); otherwise as `move`."""

    def touch(self, motion: Motion) -> Result:
        """A "touch" motion: fly the descent half of `motion.traj` slowly under position
        control, stop at the first contact with the paper (force onset against the tare, a
        depth cap of the planned end plus `motion.extra_depth`), then return to the hover the
        same way.  done with `q` = the joints at contact; failed("no contact") when nothing was
        met within the depth, with the arm back at the hover."""

    def hold(self) -> None:
        """Stand still where the arm is, for as long as it takes."""

    def stop(self) -> None:
        """Stop now, then hold.  A running `move` or `draw` returns failed("stopped")."""

    def recover(self) -> Result:
        """Clear a fault or a stop so the arm may move again (after a person has looked)."""
