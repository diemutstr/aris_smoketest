"""The calibration driver: one arm on libfranka directly (panda-py), with Desk for the modes
and the pilot buttons.  It replaces the arm's ROS stack for a mark job (serve stops the stack
first), so a mode switch never reaches a running ROS controller (DESIGN 6, 2026-10-05).

  state()      the robot state: joints, speeds, mode, errors
  move(traj)   our timed joint trajectory at its own timing: q(t), qd(t) of the planner's cubic
               streamed at 1 kHz into panda-py's joint position controller (a joint impedance
               around the reference).  Not `move_to_joint_position`: that plans its own timing,
               and the checker's verdict holds at ours only
  guide(m)     the mark: libfranka disconnected, Desk to programming (light white; FCI
               goes off with it); wait for ✓ (check) or ○ (circle), while ✗ (cross) means
               "I am redoing this seat" and the wait goes on (a row says so); Desk to
               execution (light blue), FCI on (confirmed), libfranka connected again, a 1.5 s
               settle, then the joints read standing still (two reads `standstill_dt` apart
               within `standstill_tol`): that is the sample.  Then the pen goes straight up
               `lift_m` (or 2/3, 1/3 of it where the arm cannot reach that far), slowly, and
               the arm flies back to the hover, so the queue's next motion starts where it was
               planned.  done with q = the sample and why = the button
  draw, touch  refused: not this driver
  hold, stop, recover

The arm's turn (one CalibArm, made by serve's hand-over and closed by it): Desk control taken
once at the start (the person presses circle if the browser or an old token holds it), FCI on,
libfranka connected; at the end, and on every failure, libfranka closed and Desk control
released.  While libfranka is off the arm has no reading: `state()` says so (q is NaN,
"no joint states (FCI off)"), never zeros.

The FCI side is behind `Fci` (`PandaFci` for the robot, a fake in the tests); the Desk side is
`desk.Desk`.  `say(event, **fields)` is how serve hears the hand-over ("guide: handed over",
"guide: button x", "guide: taken back").
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Protocol

import numpy as np

from aris.execute.drivers import ArmState, Result
from aris.kernel.retime import retime
from aris.types import JointPath, Refusal

from aris_robot import stream as S
from aris_robot.desk import BUTTONS
from aris_robot.touch import Kinematics, straight_on

MODES = {0: "other", 1: "idle", 2: "move", 3: "guiding", 4: "reflex", 5: "user stopped",
         6: "automatic error recovery"}


@dataclass(frozen=True)
class CalibSettings:
    button_timeout_s: float = 600.0     # no button within 10 min: give up, arm in execution
    standstill_dt: float = 0.5
    standstill_tol: float = 1e-4        # rad, the two reads agree on every joint
    standstill_timeout_s: float = 10.0
    reconnect_s: float = 20.0           # FCI back after the mode change
    control_wait_s: float = 60.0        # for the person's circle press when Desk is held
    settle_s: float = 1.5               # after the hand-back, before the first read
    lift_m: float = 0.03
    lift_speed: float = 0.01            # m/s
    back_max_rad: float = 0.5           # the way back to the hover is a straight joint move


class Fci(Protocol):
    def state(self) -> tuple:
        """(q (7,), qd (7,), robot mode number, [current error names])."""

    def follow(self, traj, halt: threading.Event) -> str:
        """Fly `traj` at its own timing; "" when done, else why (also "stopped" on `halt`)."""

    def recover(self) -> str: ...

    def close(self) -> None: ...


class CalibArm:
    def __init__(self, slot: str, rig, connect, desk, settings: CalibSettings = CalibSettings(),
                 say=None):
        """`connect()` -> a connected Fci (called again after every mode change).  The arm's
        turn starts here: Desk control is taken once (the person presses circle if someone
        else holds it), FCI is switched on, then libfranka connects.  `close()` ends the
        turn and always releases Desk control."""
        self.arm_id, self.rig, self.connect, self.desk = slot, rig, connect, desk
        self.s, self.say = settings, say or (lambda event, **f: None)
        self.kin = Kinematics.of(rig, slot)
        self.start_tol = float(rig.execution().start_tolerance)
        self.fci = None
        self.desk.take_control(self.s.control_wait_s)
        try:
            self.desk.fci(True)                 # confirmed on before libfranka connects
            self.fci = connect()
        except BaseException:
            self.desk.release_control()
            raise
        self._halt = threading.Event()
        self._stopped = False
        self._busy = threading.Lock()

    # ------------------------------------------------------------------ verbs

    def state(self) -> ArmState:
        if self.fci is None:                    # handed over: no reading, never zeros
            return ArmState(np.full(7, np.nan), np.zeros(7), False,
                            ("no joint states (FCI off)",))
        q, qd, mode, errors = self.fci.state()
        flags = ["moving" if self._busy.locked() else "holding", f"mode: {MODES.get(mode, mode)}"]
        flags += [f"fault: {e}" for e in errors]
        if self._stopped:
            flags.append("stopped")
        ok = mode in (1, 2) and not errors and not self._stopped
        return ArmState(np.asarray(q, float), np.asarray(qd, float), ok, tuple(flags))

    def move(self, traj) -> Result:
        with self._busy:
            refused = self._refuse(traj.q[0])
            if refused:
                return refused
            return self._fly(traj)

    def draw(self, motion) -> Result:
        return Result.failed("not this driver: the calibration driver does not draw",
                             self.state().q)

    def touch(self, motion) -> Result:
        return Result.failed("not this driver: the touch runs on the ROS stack",
                             self.state().q)

    def guide(self, motion) -> Result:
        with self._busy:
            refused = self._refuse(motion.q_start)
            if refused:
                return refused
            mark = motion.piece.line_id if motion.piece is not None else ""
            button = self._hand_over(mark)
            back = self._take_back(mark, button)
            if back:
                return Result.failed(back, self._q())
            if button is None:
                return Result.failed(f"no pilot button within {self.s.button_timeout_s:g} s",
                                     self._q())
            q = self._standstill()
            if q is None:
                return Result.failed("the arm did not stand still after the hand-back",
                                     self._q())
            why = self._back_to(motion.q_end, q)
            if why:
                return Result.failed(f"sample read, then {why}", self._q())
            return Result(True, button, q)

    def hold(self) -> None:
        return None

    def stop(self) -> None:
        self._stopped = True
        self._halt.set()

    def recover(self) -> Result:
        if self._busy.locked():
            return Result.failed("still moving", self._q())
        if self.fci is None:
            return Result.failed("no FCI connection", self._q())
        _, _, mode, _ = self.fci.state()
        if mode in (3, 5):
            return Result.failed(f"the arm is in {MODES[mode]}: release it at the arm first",
                                 self._q())
        why = self.fci.recover()
        if why:
            return Result.failed(why, self._q())
        self._halt.clear()
        self._stopped = False
        s = self.state()
        return Result.ok(s.q) if s.ok else Result.failed(", ".join(s.flags), s.q)

    def close(self) -> None:
        """The end of the arm's turn: libfranka disconnects, Desk control is released."""
        try:
            if self.fci is not None:
                self.fci.close()
                self.fci = None
        finally:
            self.desk.release_control()

    # ------------------------------------------------------------------ the parts

    def _q(self) -> np.ndarray:
        return self.state().q

    def _refuse(self, q_start) -> Result | None:
        s = self.state()
        if not s.ok:
            return Result.failed("arm will not move: " + ", ".join(s.flags), s.q)
        gap = float(np.max(np.abs(np.asarray(q_start) - s.q)))
        if gap > self.start_tol:
            return Result.failed(f"trajectory starts {gap:.4g} rad from the arm", s.q)
        return None

    def _fly(self, traj) -> Result:
        why = self.fci.follow(traj, self._halt)
        return Result.ok(self._q()) if not why else Result.failed(why, self._q())

    def _hand_over(self, mark: str) -> str | None:
        """The arm to the person; the pilot button they pressed, or None (timeout)."""
        self.fci.close()
        self.fci = None
        self.desk.mode("programming")                    # FCI goes off with it
        self.say("guide: handed over", arm=self.arm_id, mark=mark)
        for b in self.desk.buttons(self.s.button_timeout_s):
            if b == "cross":                     # "I want to redo this": keep the arm, wait on
                self.say("guide: button cross, waiting", arm=self.arm_id, mark=mark,
                         button=b)
            elif b in BUTTONS:
                self.say("guide: button " + b, arm=self.arm_id, mark=mark, button=b)
                return b
        self.say("guide: no button", arm=self.arm_id, mark=mark,
                 timeout_s=self.s.button_timeout_s)
        return None

    def _take_back(self, mark: str, button) -> str:
        """Execution mode, FCI on, connected again; why not, or ""."""
        self.desk.mode("execution")
        self.desk.fci(True)
        t_end, why = time.monotonic() + self.s.reconnect_s, ""
        while True:
            try:
                self.fci = self.connect()
                break
            except Exception as e:              # the robot refuses until FCI is up again
                why = f"FCI did not come back: {type(e).__name__}: {e}"
                if time.monotonic() > t_end:
                    self.say("guide: not taken back", arm=self.arm_id, mark=mark, why=why)
                    return why
                time.sleep(0.5)
        self.say("guide: taken back", arm=self.arm_id, mark=mark, button=button,
                 q=[float(x) for x in self._q()])
        return ""

    def _standstill(self) -> np.ndarray | None:
        time.sleep(self.s.settle_s)
        t_end = time.monotonic() + self.s.standstill_timeout_s
        q0 = self._q()
        while time.monotonic() < t_end:
            time.sleep(self.s.standstill_dt)
            q1 = self._q()
            if np.abs(q1 - q0).max() <= self.s.standstill_tol:
                return q1
            q0 = q1
        return None

    def _back_to(self, q_hover, q) -> str:
        """Pen straight up `lift_m`, slowly; then a straight joint move to the hover."""
        for h in (self.s.lift_m, 2 * self.s.lift_m / 3, self.s.lift_m / 3):  # as far as it can
            up = straight_on(self.kin.arm, self.kin.arm.limits, self.kin.rules, q,
                             self.kin.normal, h, self.s.lift_speed)
            if not isinstance(up, Refusal):
                break
        if isinstance(up, Refusal):
            return f"cannot lift straight up: {up.detail}"
        r = self._fly(up)
        if not r.done:
            return f"the lift failed: {r.why}"
        gap = float(np.abs(np.asarray(q_hover) - up.q[-1]).max())
        if gap > self.s.back_max_rad:
            return (f"the arm was left {gap:.2f} rad from the hover (at most "
                    f"{self.s.back_max_rad}); it holds above the mark")
        if gap > 1e-9:
            back = retime(JointPath(np.array([up.q[-1], q_hover])), self.kin.arm.limits,
                          self.kin.rules)
            r = self._fly(back)
            if not r.done:
                return f"the way back to the hover failed: {r.why}"
        return ""


class PandaFci:
    """One FCI connection through panda-py (`pip install aris_robot[calib]`)."""

    def __init__(self, ip: str, rate_hz: float = 1000.0):
        import panda_py                                   # optional dependency
        from panda_py import controllers
        self.panda, self.controllers, self.rate = panda_py.Panda(ip), controllers, rate_hz

    def state(self) -> tuple:
        st = self.panda.get_state()
        errs = [name for name in dir(st.current_errors)
                if not name.startswith("_") and getattr(st.current_errors, name) is True]
        return np.array(st.q), np.array(st.dq), int(st.robot_mode), errs

    def follow(self, traj, halt: threading.Event) -> str:
        t0, duration = float(traj.t[0]), float(traj.t[-1] - traj.t[0])
        ctrl = self.controllers.JointPosition()
        self.panda.start_controller(ctrl)
        try:
            with self.panda.create_context(frequency=self.rate, max_runtime=duration + 1.0) as ctx:
                while ctx.ok():
                    if halt.is_set():
                        return "stopped"
                    t = min(ctx.num_ticks / self.rate, duration)
                    q, qd = S.cubic(traj.t, traj.q, traj.qd, [t0 + t])
                    ctrl.set_control(q[0], qd[0])
                    if t >= duration:
                        break
        except RuntimeError as e:                         # libfranka's control exceptions
            return f"libfranka: {e}"
        finally:
            self.panda.stop_controller()
        return ""

    def recover(self) -> str:
        try:
            self.panda.recover()
        except RuntimeError as e:
            return f"error recovery failed: {e}"
        return ""

    def close(self) -> None:
        try:
            self.panda.get_robot().stop()
        except Exception:
            pass
        self.panda = None
