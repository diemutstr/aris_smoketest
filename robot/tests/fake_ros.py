"""Enough of ROS to run `aris_robot.driver.RosArm` here: stub modules for its imports, and a
fake arm node whose impedance controller follows the stream in real time and reports a force
from a model the test gives.  It checks the driver's logic, not ROS and not the robot."""
from __future__ import annotations

import sys
import threading
import time
import types
from types import SimpleNamespace

import numpy as np

STUBS = ["rclpy", "rclpy.action", "rclpy.executors", "rclpy.qos", "action_msgs",
         "action_msgs.msg", "aris_msgs", "aris_msgs.msg", "builtin_interfaces",
         "builtin_interfaces.msg", "control_msgs", "control_msgs.action",
         "controller_manager_msgs", "controller_manager_msgs.srv", "franka_msgs",
         "franka_msgs.action", "franka_msgs.msg", "franka_msgs.srv", "sensor_msgs", "sensor_msgs.msg", "std_srvs",
         "std_srvs.srv", "trajectory_msgs", "trajectory_msgs.msg"]


class _Any:
    """Stands for any ROS class or constant the driver's modules name at import."""

    def __init__(self, *a, **k):
        pass

    def __getattr__(self, name):
        return _Any()


def install() -> None:
    for name in STUBS:
        if name not in sys.modules:
            m = types.ModuleType(name)
            m.__getattr__ = lambda attr: _Any          # every name resolves
            sys.modules[name] = m


class FakeArmNode:
    """One arm: two controllers, an impedance controller that plays the stream at real time
    and holds, and a force reading `force_model(stream_time, f_ff) -> F_ext (3,)`."""

    def __init__(self, q0, normal, force_model=None, drop_after=None, bias=2.3):
        self.q_d = np.array(q0, float)
        self.normal = np.asarray(normal, float)
        self.force_model = force_model or (lambda t, f: bias * self.normal)   # air
        self.drop_after = drop_after          # stream time after which chunks are lost
        self.active = {"fr3_arm_controller"}
        self.hold_srv, self.resume_srv = "hold", "resume"
        self._lock = threading.Lock()
        self.statuses, self.status = [], None
        self.published = []
        self.goals, self.collision_calls = [], []
        self._reset()
        self._quit = False
        threading.Thread(target=self._run, daemon=True).start()

    def _reset(self):
        self.stream, self.rejected, self.t, self.streaming = 0, 0, 0.0, False
        self.done = self.starved = self.holding = False
        self.reason, self.samples, self.last, self.f_ff = "", [], False, np.zeros(3)

    # ---- what the driver calls
    def joints(self):
        with self._lock:
            return self.q_d.copy(), np.zeros(7)

    def mode_and_errors(self):
        return 2, []

    def drain_statuses(self):
        with self._lock:
            out, self.statuses = self.statuses, []
            return out

    def active_controllers(self, timeout=2.0):
        return set(self.active)

    def switch(self, activate, deactivate, timeout=3.0):
        with self._lock:
            self.active -= set(deactivate)
            self.active |= set(activate)
            if "aris_joint_impedance_controller" in activate:
                self._reset()
        return ""

    def trigger(self, client, timeout=2.0):
        with self._lock:
            if client == "hold":
                self.holding, self.reason, self.streaming = True, "hold requested", False
                self.f_ff = np.zeros(3)
            else:
                self.holding, self.reason = False, ""
        return ""

    def publish(self, chunk, extra_f=None):
        f = chunk.f if extra_f is None else chunk.f + extra_f[None, :]
        with self._lock:
            self.published.append(chunk)
            if self.drop_after is not None and chunk.t[0] > self.drop_after:
                return
            if chunk.stream > self.stream:
                if self.holding or np.abs(chunk.q[0] - self.q_d).max() > 0.01:
                    self.rejected = chunk.stream
                    return
                self.stream, self.t, self.streaming, self.done = chunk.stream, chunk.t[0], True, False
                self.samples, self.last = [], False
            if chunk.stream == self.stream and self.streaming:
                self.samples += list(zip(chunk.t, chunk.q, f))
                self.last = chunk.last

    # ---- the trajectory controller: flies a goal at once and exactly
    @property
    def follow(self):
        return _FakeAction(self)

    def drain_readings(self, joints_only=False):
        return []

    def trajectory_goal(self, traj):
        return traj

    def set_collision(self, torque_nm, force_n, timeout=3.0):
        self.collision_calls.append((list(torque_nm), list(force_n)))
        return ""

    def error_recovery(self, timeout=15.0):
        return ""

    def close(self):
        self._quit = True

    # ---- the controller, at 250 Hz
    def _run(self):
        dt = 0.004
        while not self._quit:
            time.sleep(dt)
            with self._lock:
                if self.streaming and not self.holding:
                    self.t += dt
                    ts = [s[0] for s in self.samples]
                    k = int(np.searchsorted(ts, self.t, side="right")) - 1
                    k = max(0, min(k, len(ts) - 1))
                    self.q_d, self.f_ff = self.samples[k][1].copy(), self.samples[k][2].copy()
                    if self.t >= ts[-1]:
                        if self.last:
                            self.streaming, self.done = False, True
                        elif self.t - ts[-1] > 0.02:
                            self.streaming, self.starved, self.holding = False, True, True
                            self.reason = "starved: the reference stream ran dry"
                st = SimpleNamespace(
                    stream=self.stream, rejected=self.rejected, t=self.t,
                    streaming=self.streaming, done=self.done, starved=self.starved,
                    holding=self.holding, reason=self.reason, error_joint=-1,
                    force=self.force_model(self.t, self.f_ff), f_ff=self.f_ff.copy())
                self.status = st
                self.statuses.append(st)


class _Done:
    """A future that is already done."""

    def __init__(self, value):
        self.value = value

    def add_done_callback(self, fn):
        fn(self)

    def done(self):
        return True

    def result(self):
        return self.value


class _FakeAction:
    def __init__(self, node):
        self.node = node

    def wait_for_server(self, timeout_sec=None):
        return True

    def send_goal_async(self, traj):
        node = self.node
        with node._lock:
            node.goals.append(traj)
            node.q_d = np.array(traj.q[-1], float)
        result = SimpleNamespace(status=4, result=SimpleNamespace(error_code=0, error_string=""))
        return _Done(SimpleNamespace(accepted=True, get_result_async=lambda: _Done(result),
                                     cancel_goal_async=lambda: _Done(None)))
