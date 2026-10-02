"""The real arm: the driver verbs of `aris.execute.drivers` for one FR3 on the operator PC.

  state()     joints and speeds (joint states), robot mode and errors (robot state
              broadcaster), which controller has the arm, a latched hold of the impedance one
  move(traj)  the joint trajectory controller flies the trajectory exactly as planned (every
              knot, its velocity, its time; never re-timed), after checking the arm stands at
              its first point; done, or failed with the controller's own error string
  draw(m)     lower, draw and lift under the impedance controller: the trajectory streamed at
              1 kHz with the pen force (force.py); a free motion goes to `move`
  touch(m)    the calibration touch under the trajectory controller (touch.py): down until
              the force onset, the joints there, back to the hover
  hold()      nothing to do: both controllers hold where the last motion ended
  stop()      at once: the impedance controller latches a hold, a trajectory goal is cancelled
  recover()   franka error recovery, then the trajectory controller re-activated
  switch(n)   "trajectory" or "impedance": that controller takes the arm
  set_job(pen, tracking)       the job's pen rules and tracking mode (DESIGN 4c): "position"
              (mode A, the default) flies every motion kind through the trajectory controller,
              the press being geometric (the plan runs below the paper); "impedance" (mode B)
              is lower/draw/lift under the impedance controller with the pen force
  set_collision(which)         the site's "job" or "normal" collision thresholds

With fake hardware the impedance controller cannot move the arm (the fake arm ignores
torques), so `draw` goes to `move` and there is no pen force; `touch` runs as on the real
arm, with a fake paper (`fake_paper_m`) standing in for the force estimate.
"""
from __future__ import annotations

import logging
import threading
import time

import numpy as np

from aris.execute.drivers import ArmState, Result
from aris.types import Refusal
from aris_robot import force as F
from aris_robot import stream as S
from aris_robot import touch as T
from aris_robot.rosarm import IMPEDANCE, MODES, TRAJECTORY, ArmNode, wait

log = logging.getLogger("aris_robot")
CONTROLLERS = {"trajectory": TRAJECTORY, "impedance": IMPEDANCE}
TICK = 0.005               # s between two looks at the stream and the status


class RosArm:
    def __init__(self, site, rig, arm_id: int, fake: bool = False, lead: float = 0.1,
                 fake_paper_m: float = 0.0):
        """`fake_paper_m`: with fake hardware, where the fake paper is (m above the nominal
        paper); its push stands in for the force estimate that fake hardware lacks."""
        self.arm_id, self.fake, self.lead = arm_id, fake, lead
        self._site_force, self._sign = site.force, site.arm(arm_id).force_sign
        self.collision = site.collision
        self.tracking = "position"
        self.set_pen(rig.pen())
        self.ts = T.TouchSettings.from_site(site.touch, self.fs.sign)
        self.kin = T.Kinematics.of(rig, arm_id)
        self.fake_paper = T.FakePaper(self.kin, rig.paper(arm_id), fake_paper_m) if fake else None
        self.normal = np.asarray(rig.paper(arm_id).normal, float)   # base frame, up
        self.start_tol = float(rig.execution().start_tolerance)
        self.ros = ArmNode(site.arm(arm_id), site.joint_names())
        self._busy = threading.Lock()
        self._halt = threading.Event()
        self._stopped = False
        self._zero = None             # the air zero of the last landing, N
        self._press = 0.0             # the pressing force at the end of the last motion, N
        self._stream = int(time.time() * 1000) % (1 << 30)
        self.last_report: dict = {}

    # ------------------------------------------------------------------ verbs

    def state(self) -> ArmState:
        q, qd = self.ros.joints()
        if q is None:
            return ArmState(np.zeros(7), np.zeros(7), False, ("no joint states",))
        mode, errors = self.ros.mode_and_errors()
        st = self.ros.status
        holding = bool(st is not None and st.holding)
        flags = ["moving" if self._busy.locked() else "holding"]
        if mode is not None:
            flags.append(f"mode: {MODES.get(mode, mode)}")
        flags += [f"fault: {e}" for e in errors]
        if holding:
            flags.append(f"impedance hold: {st.reason}")
        if self._stopped:
            flags.append("stopped")
        mode_ok = mode in (1, 2) or (mode is None and self.fake)
        return ArmState(q, qd, bool(mode_ok and not errors and not self._stopped), tuple(flags))

    def set_pen(self, pen: dict) -> None:
        """The pen rules (rig.json `pen`, as the job header carries them) for what follows."""
        self.fs = F.ForceSettings.from_parts(pen, self._site_force, self._sign)

    def set_job(self, pen: dict, tracking: str) -> None:
        if tracking not in ("position", "impedance"):
            raise ValueError(f"tracking {tracking!r}: position or impedance")
        self.set_pen(pen)
        self.tracking = tracking

    def set_collision(self, which: str) -> str:
        """The site's collision thresholds `which` ("job" or "normal"); "" when set."""
        c = self.collision.get(which)
        if c is None:
            return f"site.json has no {which!r} collision thresholds"
        if self.fake:
            return ""
        return self.ros.set_collision(c["torque_nm"], c["force_n"])

    def switch(self, name: str) -> str:
        """The named controller takes the arm ("" when it has it)."""
        want = CONTROLLERS[name]
        active = self.ros.active_controllers()
        if active is None:
            return "cannot list the controllers"
        if want in active:
            return ""
        other = [c for c in CONTROLLERS.values() if c != want and c in active]
        return self.ros.switch([want], other)

    def move(self, traj) -> Result:
        with self._busy:
            refused = self._refuse(traj.q[0])
            if refused:
                return refused
            why = self.switch("trajectory")
            if why:
                return Result.failed(why, self.state().q)
            return self._follow(traj)

    def draw(self, motion) -> Result:
        if motion.kind == "free" or self.fake or self.tracking == "position":
            return self.move(motion.traj)
        with self._busy:
            refused = self._refuse(motion.q_start)
            if refused:
                return refused
            why = self.switch("impedance")
            if why:
                return Result.failed(why, self.state().q)
            if motion.kind == "lower":
                zero = self._tare()
                if isinstance(zero, Refusal):
                    return Result.failed(f"{zero.reason}: {zero.detail}", self.state().q)
                self._zero = zero
            arc = None if motion.tip_base is None else F.arc_length(motion.tip_base)
            fn = F.profile(motion.kind, motion.traj.t, arc, motion.intensity, self.fs,
                           f_start=self._press)
            return self._run_stream(S.samples(motion.traj, fn, self.normal), motion.kind, fn)

    def touch(self, motion) -> Result:
        """The calibration touch (touch.py): the descent under the trajectory controller,
        stopping at the force onset, back to the hover.  done with the joints at contact."""
        with self._busy:
            refused = self._refuse(motion.q_start)
            if refused:
                return refused
            why = self.switch("trajectory")
            if why:
                return Result.failed(why, self.state().q)
            r = T.touch(motion, _Position(self), self.kin, self.ts)
            self.last_report = dict(kind="touch", air_zero=r.air_zero, held=r.held,
                                    depth_past_end=r.depth_past_end, why=r.why)
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
        active = self.ros.active_controllers(timeout=0.5) or set()
        if IMPEDANCE in active:
            self.ros.trigger(self.ros.hold_srv, timeout=0.5)

    def recover(self) -> Result:
        if self._busy.locked():
            return Result.failed("still moving", self.state().q)
        mode, _ = self.ros.mode_and_errors()
        if mode in (3, 5):                  # the old stack's rule: never heal these by software
            return Result.failed(f"the arm is in {MODES[mode]}: release it at the arm first",
                                 self.state().q)
        if not self.fake:                                 # fake hardware has no such action
            why = self.ros.error_recovery()
            if why:
                return Result.failed(why, self.state().q)
        active = self.ros.active_controllers() or set()
        why = self.ros.switch([], [c for c in CONTROLLERS.values() if c in active])
        why = why or self.ros.switch([TRAJECTORY], [])    # fresh: holds where the arm is
        if why:
            return Result.failed(why, self.state().q)
        self._halt.clear()
        self._stopped = False
        s = self.state()
        return Result.ok(s.q) if s.ok else Result.failed(", ".join(s.flags), s.q)

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
        s = self.state()
        if res.error_code != 0:
            return Result.failed(f"{res.error_string} (error code {res.error_code})", s.q)
        return Result.ok(s.q) if s.ok else Result.failed(", ".join(s.flags), s.q)

    def _tare(self) -> float | Refusal:
        """The air zero, standing still before the landing."""
        tare = F.Tare(self.fs)
        self.ros.drain_statuses()
        time.sleep(self.fs.tare_s)
        for st in self.ros.drain_statuses():
            tare.add(F.normal_force(st.force, self.normal, self.fs.sign))
        return tare.result()

    def _next_stream(self) -> int:
        self._stream += 1
        return self._stream

    def _run_stream(self, samples: S.Samples, kind: str, fn) -> Result:
        """Streams one motion and watches it.  Returns done or failed with why."""
        sid = self._next_stream()
        pacer = S.Pacer(samples, sid, self.lead)
        contact, guard, servo = F.Contact(self.fs), F.Guard(self.fs), F.Servo(self.fs)
        zero = self._zero if self._zero is not None else 0.0
        trim_dir = -self.normal / np.linalg.norm(self.normal)
        report = dict(kind=kind, stream=sid, contact_at=None, f_max=0.0)
        self.ros.drain_statuses()
        w0, started, t_prev = time.monotonic(), False, None
        duration = float(samples.t[-1])
        while True:
            elapsed = time.monotonic() - w0
            for chunk in pacer.due(elapsed):
                self.ros.publish(chunk, servo.trim * trim_dir)
            if self._halt.is_set():
                self.ros.trigger(self.ros.hold_srv, timeout=0.5)
                return self._end(report, Result.failed("stopped", self.state().q))
            for st in self.ros.drain_statuses():
                if st.rejected == sid:
                    why = st.reason if st.holding else "it does not start where the arm holds"
                    return self._end(report, Result.failed(f"stream refused: {why}", self.state().q))
                if st.stream != sid:
                    continue
                started = True
                if st.holding:
                    j = f" (joint {st.error_joint + 1})" if st.error_joint >= 0 else ""
                    return self._end(report, Result.failed(st.reason + j, self.state().q))
                f_rel = F.normal_force(st.force, self.normal, self.fs.sign) - zero
                report["f_max"] = max(report["f_max"], f_rel)
                over = guard.update(f_rel)
                if over:
                    self.ros.trigger(self.ros.hold_srv, timeout=0.5)
                    return self._end(report, Result.failed(over, self.state().q))
                if kind in ("lower", "draw") and contact.update(f_rel, st.t) \
                        and report["contact_at"] is None:
                    report["contact_at"] = contact.at
                if kind == "draw" and t_prev is not None:
                    servo.update(float(fn(st.t)), f_rel, st.t - t_prev, contact.at is not None)
                t_prev = st.t
                if st.done:
                    self._press = float(fn(duration)) + servo.trim if kind == "draw" else 0.0
                    return self._end(report, Result.ok(self.state().q))
            if elapsed > duration + 2.0:
                why = "the controller did not follow the stream" if started else \
                    "the controller did not answer (is it active?)"
                return self._end(report, Result.failed(why, self.state().q))
            time.sleep(TICK)

    def _end(self, report: dict, r: Result) -> Result:
        report["done"], report["why"] = r.done, r.why
        if report["kind"] == "lower" and r.done and report["contact_at"] is None:
            log.warning("arm %s: no contact while lowering", self.arm_id)
        self.last_report = report
        return r


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
