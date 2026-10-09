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
  guide(m)    the one hand-guided touch per arm, with the PERSON switching Desk's modes in
              the browser (the software never touches Desk): see `guide`
  hold()      nothing to do: the trajectory controller holds where the last motion ended
  stop()      at once: the running goal is cancelled, the arm holds
  recover()   error recovery, the hardware component back, the controllers back, the joint
              states fresh (else the stack restarted); one row per step
  set_collision(which)         the site's "job" or "normal" collision thresholds

With fake hardware `touch` runs as on the real arm, with a fake paper (`fake_paper_m`)
standing in for the force estimate.
"""
from __future__ import annotations

import dataclasses
import threading
import time

import numpy as np

from aris.execute.drivers import ArmState, Result
from aris_robot import touch as T
from aris_robot.gripper import Gripper, GripperSettings
from aris_robot.rosarm import BROADCASTERS, MODES, TRAJECTORY, ArmNode, GripperRos, wait

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
        self.guide_cfg = dict(site.guide)
        self.ros = ArmNode(site.arm(arm_id), site.joint_names())
        self.gripper = Gripper(GripperRos(self.ros), GripperSettings.from_site(site.gripper),
                               say=lambda event, **f: self.say(event, **f))
        self._busy = threading.Lock()
        self._halt = threading.Event()
        self._stopped = False
        self.last_report: dict = {}
        self.trace_dir = None                    # serve: where each touch's readings go

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
            self.last_report = dict(kind="touch", air_zero=r.air_zero, lag0=r.lag0,
                                    rule=r.rule, lag_at_contact=r.lag_at_contact,
                                    force_at_contact=r.force_at_contact,
                                    depth_past_end=r.depth_past_end, why=r.why)
            self.say("touch: " + (f"contact by {r.rule}" if r.done else "no contact"),
                     rule=r.rule or None, lag_at_contact_m=r.lag_at_contact,
                     force_at_contact_n=r.force_at_contact, lag0_m=r.lag0,
                     depth_past_end_m=r.depth_past_end, why=r.why,
                     text=("stopped by the force cap" if r.rule == "force cap" else None),
                     stats=r.stats, trace=self._save_trace(r))
            if r.held:
                self._stopped = True             # stands where it hit; a person looks first
            if r.done:
                return Result.ok(r.q_contact)
            return Result.failed(r.why, self.state().q)

    def _save_trace(self, r) -> str | None:
        """Every reading of the touch as a CSV in `trace_dir` (flight, time, commanded and
        actual depth below the hover in mm, force over the zero in N): what to look at when a
        contact is doubted.  -> the file's name, or None."""
        if self.trace_dir is None or not r.trace:
            return None
        try:
            from pathlib import Path
            d = Path(self.trace_dir)
            d.mkdir(parents=True, exist_ok=True)
            f = d / f"touch_{self.arm_id}_{time.strftime('%Y%m%d-%H%M%S')}.csv"
            with open(f, "w") as out:
                out.write("flight,t_s,commanded_mm,actual_mm,force_n\n")
                for k, t, dc, da, fo in r.trace:
                    out.write(f"{k},{t:.4f},{dc * 1e3:.4f},{da * 1e3:.4f},"
                              f"{'' if fo is None else format(fo, '.3f')}\n")
            return f.name
        except OSError:
            return None

    def guide(self, motion) -> Result:
        """At the hover the trajectory controller is let go and the person is told what to do
        in Desk (site.json `guide.instruction`): programming mode, the pen tip on the cross,
        let go, execution mode, FCI on.  Programming mode switches FCI off, so the link drops
        and the joint states go stale: that is expected.  The arm's stack is restarted every
        `restart_every_s` while they are stale; once they are fresh again the arm is recovered
        with its trajectory controller left OFF, and once it has stood still `settle_s` away
        from the hover (`moved_rad`) its joints are the sample.  Then the controller holds
        again, the pen goes straight up and back to the hover.  done with why "check"."""
        with self._busy:
            refused = self._refuse(motion.q_start)
            if refused:
                return refused
            why = self.ros.switch([], [TRAJECTORY])
            if why:
                return Result.failed(f"cannot let the arm go: {why}", self.state().q)
            text = self.guide_cfg.get("instruction", "guide: your turn on {slot}").format(
                slot=self.arm_id)
            self.say("instruction", text=text)        # the person-facing prompt
            out = self._await_sample(np.asarray(motion.q_start, float))
            why = self._trajectory_controller()           # holds where the arm is
            self.say("guide: trajectory controller back", ok=not why, why=why)
            if why:
                self._recover()
            if not out.done:
                return out
            back = self._back_to_hover(np.asarray(motion.q_end, float), out.q)
            if back:
                # the sample is kept whatever happens after it: hold where the arm stands
                # (after the lift if that went), say so, and report success from there
                why = self._trajectory_controller()
                here = self.state().q
                self.say("guide: registered; stayed at the meeting pose", q=here,
                         holding=not why, why=why,
                         text=f"registered; stayed at the meeting pose (glide failed: {back})")
                return GuideResult(True, "check", out.q, q_end=here)
            return GuideResult(True, "check", out.q, q_end=self.state().q)

    def _await_sample(self, q_hover) -> Result:
        g = self.guide_cfg
        timeout, every = float(g.get("timeout_s", 600.0)), float(g.get("restart_every_s", 15.0))
        settle, still = float(g.get("settle_s", 2.0)), float(g.get("still_rad", 0.002))
        moved_min = float(g.get("moved_rad", 0.02))
        t_end, last_restart, down, since, ref, said_idle = (time.monotonic() + timeout, -1e9,
                                                           False, None, None, False)
        while time.monotonic() < t_end:
            if self._halt.is_set():
                return Result.failed("stopped", self.state().q)
            s = self.state()
            if not np.all(np.isfinite(s.q)):              # the link is down (FCI off): expected
                if not down:
                    self.say("guide: link down (FCI off), waiting for FCI on")
                    down = True
                if self.restart_stack is not None and time.monotonic() - last_restart >= every:
                    last_restart = time.monotonic()
                    self.say("guide: restarting the stack")
                    self.restart_stack()
                since = None
                time.sleep(0.1)
                continue
            if down:                                      # back: recover, controller stays off
                self._recover_off()
                down, since = False, None
                continue
            q = np.asarray(s.q, float)
            if since is None or float(np.abs(q - ref).max()) > still:
                since, ref = time.monotonic(), q
            elif time.monotonic() - since >= settle:
                if float(np.abs(q - q_hover).max()) >= moved_min:
                    self.say("guide: registered", q=[float(x) for x in q])
                    return Result(True, "check", q)
                if not said_idle:
                    self.say("instruction", text=f"nobody moved {self.arm_id}; waiting")
                    said_idle = True
                since = None
            time.sleep(0.05)
        return Result.failed(f"nobody guided {self.arm_id} within {timeout:g} s", self.state().q)

    def _recover_off(self) -> None:
        """After the link came back: the hardware component active, franka error recovery, and
        the trajectory controller NOT active (the sample is read first)."""
        steps = [] if self.fake else [
            ("hardware component", lambda: self.ros.reactivate_hardware(self.hw_component)),
            ("error recovery", self.ros.error_recovery)]
        for name, fn in steps:
            why = fn()
            self.say(f"guide: {name}", ok=not why, why=why)
        active = self.ros.active_controllers() or set()
        if TRAJECTORY in active:                    # a restarted stack comes up with it active
            why = self.ros.switch([], [TRAJECTORY])
            self.say("guide: trajectory controller off", ok=not why, why=why)
        self.say("guide: link back")

    def _back_to_hover(self, q_hover, q) -> str:
        """The retreat after a meeting: the pen straight up `lift_m` (or 2/3, 1/3 of it where
        the arm cannot reach), then the tip on a straight horizontal line (the paper's plane)
        to above the hover, the hand turning to the hover's orientation and joint 7 to its
        angle on the way, then straight along the normal to the hover's height; IK-tracked,
        at the free speed.  The hover lies on the arm's own side, so the line runs away from
        the meeting point and the partner's tip: the two arms, returning at once, only ever
        move apart.  Ends at the hover's joints, where the queue's next motion starts."""
        from aris.types import Refusal
        lift, speed = float(self.guide_cfg.get("lift_m", 0.03)), float(
            self.guide_cfg.get("lift_speed", 0.01))
        for h in (lift, 2 * lift / 3, lift / 3):
            up = T.straight_on(self.kin.arm, self.kin.arm.limits, self.kin.rules, q,
                               self.kin.normal, h, speed)
            if not isinstance(up, Refusal):
                break
        if isinstance(up, Refusal):
            return f"cannot lift straight up: {up.detail}"
        r = self._follow(up)
        if not r.done:
            return f"the lift failed: {r.why}"
        self.say("guide: lifted")
        arm = self.kin.arm
        n = np.asarray(self.kin.normal, float) / np.linalg.norm(self.kin.normal)
        q_up = np.asarray(up.q[-1], float)
        tip_up, tip_hover = arm.tip(q_up[None])[0], arm.tip(q_hover[None])[0]
        above = tip_hover + float((tip_up - tip_hover) @ n) * n      # the hover's x, y
        R_hover = arm.fk(q_hover[None])[0][:3, :3]
        legs = (("away", above, R_hover, q_hover[6]), ("to the hover height", tip_hover,
                                                       R_hover, q_hover[6]))
        q_now = q_up
        for name, to, R, q7 in legs:
            if float(np.linalg.norm(to - arm.tip(q_now[None])[0])) < 1e-4 and name != "away":
                continue
            leg = T.glide(arm, arm.limits, self.kin.rules, q_now, to, R, q7)
            if isinstance(leg, Refusal):
                return f"cannot retreat ({name}): {leg.detail}"
            r = self._follow(leg)
            if not r.done:
                return f"the retreat ({name}) failed: {r.why}"
            q_now = np.asarray(leg.q[-1], float)
            self.say(f"guide: retreat {name}")
        if float(np.abs(q_now - q_hover).max()) > 1e-3:
            return (f"the retreat ended {np.abs(q_now - q_hover).max():.3f} rad from the hover's "
                    "joints (another arm shape)")
        self.say("guide: back at the hover")
        return ""

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
        return self._recover()

    def _recover(self) -> Result:
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
        """The trajectory controller flies `traj`; `watch(q, F, t)` sees every reading (actual
        joints, force, the reading's time on the trajectory's clock, counted from when the goal
        was sent) and may cancel it (True): then failed("cancelled") with the arm standing
        where it stopped."""
        client = self.ros.follow
        if not client.wait_for_server(timeout_sec=2.0):
            return Result.failed("the trajectory controller is not available", self.state().q)
        goal = self.ros.trajectory_goal(traj)
        self.ros.drain_readings(self.fake)
        t_sent = time.monotonic()
        handle = wait(client.send_goal_async(goal), 5.0)
        if handle is None or not handle.accepted:
            return Result.failed("the trajectory controller refused the trajectory",
                                 self.state().q)
        result = handle.get_result_async()
        deadline = time.monotonic() + float(traj.t[-1] - traj.t[0]) + 5.0
        while not result.done():
            if self._halt.is_set():
                wait(handle.cancel_goal_async(), 2.0)
                return Result.failed("stopped", self.state().q)
            if watch is not None:
                for q, f, rx, q_ref in self.ros.drain_readings(self.fake):
                    t = float(traj.t[0]) + (rx - t_sent)    # the fallback for q_ref
                    if watch(q, self.fake_paper.force(q) if self.fake else f, t, q_ref):
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


@dataclasses.dataclass(frozen=True)
class GuideResult(Result):
    """A guide's result: `q` the sample (what the executor registers), `q_end` where the arm
    stands at the end (the hover, or the meeting pose when the retreat could not be flown)."""
    q_end: np.ndarray | None = None


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
        return [a.fake_paper.force(q) if a.fake else f
                for q, f, *_ in a.ros.drain_readings(a.fake)]

    def joints(self):
        return self.arm.state().q

    def still(self, rest_s: float, rest_m: float, wait_s: float) -> str:
        """Wait (at most `wait_s`) until the pen tip has not moved more than `rest_m` for
        `rest_s`.  -> "" then, else what it still did."""
        a, seen, span = self.arm, [], float("nan")
        t_end = time.monotonic() + wait_s
        while True:
            q, now = a.state().q, time.monotonic()
            if np.all(np.isfinite(q)):
                seen.append((now, a.kin.tip(q)[0]))
            seen = [x for x in seen if now - x[0] <= rest_s]
            if seen and now - seen[0][0] >= 0.9 * rest_s:
                tips = np.array([x[1] for x in seen])
                span = float(np.linalg.norm(tips.max(axis=0) - tips.min(axis=0)))
                if span <= rest_m:
                    return ""
            if now > t_end:
                return (f"the tip still moved {span * 1000:.2f} mm in {rest_s:g} s after "
                        f"{wait_s:g} s at the hover")
            time.sleep(0.02)

