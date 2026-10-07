"""The real arm: the driver verbs of `aris.execute.drivers` for one FR3 on the operator PC.
One tracking mode: joint position control through the stock joint trajectory controller
(Pete, 2026-10-07).  The press is geometric: the plan already runs below the paper.

  state()     joints and speeds (joint states), robot mode and errors (robot state
              broadcaster); a reading older than `stale_s` is no reading (NaN, flagged)
  move(traj)  the trajectory controller flies the trajectory exactly as planned (every knot,
              its velocity, its time; never re-timed), after checking the arm stands at its
              first point; done once the arm stands still, or failed with the controller's
              own error string
  draw(m)     lower, draw and lift: the same as `move`
  touch(m)    the calibration touch under the trajectory controller (touch.py): down until
              the force onset, the joints there, back to the hover
  hold()      nothing to do: the trajectory controller holds where the last motion ended
  stop()      at once: the running goal is cancelled, the arm holds
  recover()   error recovery, the hardware component back, the controllers back, the joint
              states fresh (else the stack restarted); one row per step
  set_collision(which)         the site's "job" or "normal" collision thresholds

With fake hardware `touch` runs as on the real arm, with a fake paper (`fake_paper_m`)
standing in for the force estimate.
"""
from __future__ import annotations

import threading
import time

import numpy as np

from aris.execute.drivers import ArmState, Result
from aris_robot import touch as T
from aris_robot.rosarm import BROADCASTERS, MODES, TRAJECTORY, ArmNode, wait

TICK = 0.005               # s between two looks at a running goal


class RosArm:
    def __init__(self, site, rig, arm_id: str, fake: bool = False, fake_paper_m: float = 0.0):
        """`fake_paper_m`: with fake hardware, where the fake paper is (m above the nominal
        paper); its push stands in for the force estimate that fake hardware lacks."""
        self.arm_id, self.fake = arm_id, fake
        self.collision = site.collision
        ex = site.execution
        self.rest_qd = float(ex.get("rest_qd_rad_per_s", 5e-3))
        self.settle_s = float(ex.get("settle_s", 1.5))
        self.stale_s = float(ex.get("stale_s", 2.0))
        self.fresh_wait_s = float(ex.get("fresh_wait_s", 10.0))
        self.hw_component = site.hardware_component
        self.say = lambda event, **f: None       # serve: its rows
        self.restart_stack = None                # serve: restart this arm's ROS stack
        self.ts = T.TouchSettings.from_site(site.touch, site.arm(arm_id).force_sign)
        self.kin = T.Kinematics.of(rig, arm_id)
        self.fake_paper = T.FakePaper(self.kin, rig.paper(arm_id), fake_paper_m) if fake else None
        self.start_tol = float(rig.execution().start_tolerance)
        self.ros = ArmNode(site.arm(arm_id), site.joint_names())
        self._busy = threading.Lock()
        self._halt = threading.Event()
        self._stopped = False
        self.last_report: dict = {}

    # ------------------------------------------------------------------ verbs

    def state(self) -> ArmState:
        q, qd = self.ros.joints()
        if q is None:
            # no reading (stack down, FCI off): NaN, never zeros a planner could start from
            return ArmState(np.full(7, np.nan), np.zeros(7), False, ("no joint states",))
        _, age = self.ros.joint_freshness()
        if age is not None and age > self.stale_s:
            # an old reading is no reading (a stalled stack keeps its last joints)
            return ArmState(np.full(7, np.nan), np.zeros(7), False,
                            (f"stale joint states (last {age:.1f} s ago)",))
        mode, errors = self.ros.mode_and_errors()
        flags = ["moving" if self._busy.locked() else "holding"]
        if mode is not None:
            flags.append(f"mode: {MODES.get(mode, mode)}")
        flags += [f"fault: {e}" for e in errors]
        if self._stopped:
            flags.append("stopped")
        mode_ok = mode in (1, 2) or (mode is None and self.fake)
        return ArmState(q, qd, bool(mode_ok and not errors and not self._stopped), tuple(flags))

    def set_collision(self, which: str) -> str:
        """The site's collision thresholds `which` ("job" or "normal"); "" when set."""
        c = self.collision.get(which)
        if c is None:
            return f"site.json has no {which!r} collision thresholds"
        if self.fake:
            return ""
        return self.ros.set_collision(c["torque_nm"], c["force_n"])

    def move(self, traj) -> Result:
        with self._busy:
            refused = self._refuse(traj.q[0])
            if refused:
                return refused
            why = self._trajectory_controller()
            if why:
                return Result.failed(why, self.state().q)
            return self._follow(traj)

    def draw(self, motion) -> Result:
        return self.move(motion.traj)

    def touch(self, motion) -> Result:
        """The calibration touch (touch.py): the descent under the trajectory controller,
        stopping at the force onset, back to the hover.  done with the joints at contact."""
        with self._busy:
            refused = self._refuse(motion.q_start)
            if refused:
                return refused
            why = self._trajectory_controller()
            if why:
                return Result.failed(why, self.state().q)
            r = T.touch(motion, _Position(self), self.kin, self.ts)
            self.last_report = dict(kind="touch", air_zero=r.air_zero, held=r.held,
                                    depth_past_end=r.depth_past_end, why=r.why,
                                    armed_zero=r.armed_zero, early_trips=r.early_trips)
            if r.held:
                self._stopped = True             # stands where it hit; a person looks first
            if r.done:
                return Result.ok(r.q_contact)
            return Result.failed(r.why, self.state().q)

    def hold(self) -> None:
        return None

    def stop(self) -> None:
        self._stopped = True
        self._halt.set()

    def recover(self) -> Result:
        """Bring the arm back after a fault, a link drop, guiding mode or a stack that started
        with FCI off; one row per step, and every step runs even when one before failed:
        the hardware component inactive -> active (the error-recovery action lives behind
        it), franka error recovery, the trajectory controller and the broadcasters active,
        the joint states fresh (their stamp advancing).  Still not fresh: serve restarts the
        arm's ROS stack and the freshness is checked again."""
        if self._busy.locked():
            return Result.failed("still moving", self.state().q)
        mode, _ = self.ros.mode_and_errors()
        if mode == 5:                       # only the person at the arm can release a user stop
            return Result.failed("the arm is in user stop: release it at the arm first",
                                 self.state().q)
        steps = [] if self.fake else [          # fake hardware has neither
            ("hardware component", lambda: self.ros.reactivate_hardware(self.hw_component)),
            ("error recovery", self.ros.error_recovery)]
        steps.append(("controllers", self._controllers_back))
        failed = []
        for name, fn in steps:
            why = fn()
            self.say(f"recover: {name}", ok=not why, why=why)
            if why:
                failed.append(f"{name}: {why}")
        fresh = self._fresh(self.fresh_wait_s)
        if not fresh:
            self.say("recover: joint states not fresh", waited_s=self.fresh_wait_s)
            if self.restart_stack is not None:
                self.say("recover: restarting the stack")
                self.restart_stack()
                fresh = self._fresh(60.0)
        self.say("recover: joint states fresh" if fresh else "recover: joint states not fresh")
        if not fresh:
            return Result.failed("; ".join(failed + ["the joint states are not fresh"]),
                                 self.state().q)
        self._halt.clear()
        self._stopped = False
        s = self.state()
        if s.ok:
            return Result.ok(s.q)
        return Result.failed("; ".join(failed + [", ".join(s.flags)]), s.q)

    def _trajectory_controller(self) -> str:
        """The trajectory controller active ("" when it is)."""
        active = self.ros.active_controllers()
        if active is None:
            return "cannot list the controllers"
        return "" if TRAJECTORY in active else self.ros.switch([TRAJECTORY], [])

    def _controllers_back(self) -> str:
        """Deactivate ours, then the trajectory controller and the broadcasters active again
        (fresh: it holds where the arm is)."""
        ours = (TRAJECTORY,) + BROADCASTERS
        active = self.ros.active_controllers()
        if active is None:
            return "cannot list the controllers"
        why = self.ros.switch([], [c for c in ours if c in active]) if active & set(ours) else ""
        want = [TRAJECTORY, BROADCASTERS[0]] + ([] if self.fake else [BROADCASTERS[1]])
        return why or self.ros.switch(want, [])

    def _fresh(self, wait_s: float) -> bool:
        """The joint states' stamp advances over 1 s and the newest arrived recently."""
        t_end = time.monotonic() + wait_s
        while time.monotonic() < t_end:
            s0, _ = self.ros.joint_freshness()
            time.sleep(1.0)
            s1, age = self.ros.joint_freshness()
            if s0 is not None and s1 is not None and s1 > s0 and age <= self.stale_s:
                return True
        return False

    def close(self) -> None:
        self.ros.close()

    # ------------------------------------------------------------------ the parts

    def _refuse(self, q_start) -> Result | None:
        s = self.state()
        if not s.ok:
            return Result.failed("arm will not move: " + ", ".join(s.flags), s.q)
        gap = float(np.max(np.abs(np.asarray(q_start) - s.q)))
        if gap > self.start_tol:
            return Result.failed(f"trajectory starts {gap:.4g} rad from the arm", s.q)
        return None

    def _follow(self, traj, watch=None) -> Result:
        """The trajectory controller flies `traj`; `watch(q, F)` sees every reading and may
        cancel it (True): then failed("cancelled") with the arm standing where it stopped."""
        client = self.ros.follow
        if not client.wait_for_server(timeout_sec=2.0):
            return Result.failed("the trajectory controller is not available", self.state().q)
        handle = wait(client.send_goal_async(self.ros.trajectory_goal(traj)), 5.0)
        if handle is None or not handle.accepted:
            return Result.failed("the trajectory controller refused the trajectory",
                                 self.state().q)
        result = handle.get_result_async()
        deadline = time.monotonic() + float(traj.t[-1] - traj.t[0]) + 5.0
        self.ros.drain_readings(self.fake)
        while not result.done():
            if self._halt.is_set():
                wait(handle.cancel_goal_async(), 2.0)
                return Result.failed("stopped", self.state().q)
            if watch is not None:
                for q, f in self.ros.drain_readings(self.fake):
                    if watch(q, self.fake_paper.force(q) if self.fake else f):
                        wait(handle.cancel_goal_async(), 2.0)
                        time.sleep(0.05)                    # the controller holds where it is
                        return Result.failed("cancelled", self.state().q)
            if time.monotonic() > deadline:
                wait(handle.cancel_goal_async(), 2.0)
                return Result.failed("the trajectory did not finish in time", self.state().q)
            time.sleep(TICK)
        res = result.result().result
        if res.error_code != 0:
            return Result.failed(f"{res.error_string} (error code {res.error_code})",
                                 self.state().q)
        s = self._settled()
        return Result.ok(s.q) if s.ok else Result.failed(", ".join(s.flags), s.q)

    def _settled(self):
        """The state once the arm stands still (all |qd| <= rest_qd), at most settle_s."""
        t_end = time.monotonic() + self.settle_s
        s = self.state()
        while time.monotonic() < t_end and not np.all(np.abs(s.qd) <= self.rest_qd):
            time.sleep(0.02)
            s = self.state()
        return s


class _Position:
    """The real arm as touch.py sees it: the trajectory controller and the robot's readings."""

    def __init__(self, arm: RosArm):
        self.arm = arm

    def fly(self, traj, watch) -> str:
        r = self.arm._follow(traj, watch)
        return "" if r.done else r.why

    def forces(self, seconds: float) -> list:
        a = self.arm
        a.ros.drain_readings(a.fake)
        time.sleep(seconds)
        return [a.fake_paper.force(q) if a.fake else f for q, f in a.ros.drain_readings(a.fake)]

    def joints(self):
        return self.arm.state().q
