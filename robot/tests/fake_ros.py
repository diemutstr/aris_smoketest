"""Enough of ROS to run `aris_robot.driver.RosArm` here: stub modules for its imports, and a
fake arm node whose trajectory controller flies a goal at once and exactly.  It checks the
driver's logic, not ROS and not the robot."""
from __future__ import annotations

import sys
import threading
import time
import types
from types import SimpleNamespace

import numpy as np

STUBS = ["rclpy", "rclpy.action", "rclpy.executors", "action_msgs", "action_msgs.msg",
         "builtin_interfaces", "builtin_interfaces.msg", "control_msgs", "control_msgs.action",
         "controller_manager_msgs", "controller_manager_msgs.srv", "franka_msgs",
         "franka_msgs.action", "franka_msgs.msg", "franka_msgs.srv", "sensor_msgs",
         "sensor_msgs.msg", "trajectory_msgs", "trajectory_msgs.msg", "lifecycle_msgs",
         "lifecycle_msgs.msg"]


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
    """One arm: its controllers, its robot mode and errors, joint states that arrive (or, once
    `stalled_at` is set, stopped arriving then), the goals flown and the calls made."""

    def __init__(self, q0):
        self.q_d = np.array(q0, float)
        self.active = {"fr3_arm_controller", "joint_state_broadcaster",
                       "franka_robot_state_broadcaster"}
        self.mode, self.errors = 2, []
        self.stalled_at = None
        self._lock = threading.Lock()
        self.goals, self.collision_calls, self.recovery_steps, self.switches = [], [], [], []
        self.on_switch = None                 # a test's hook: called after every switch

    def joints(self):
        with self._lock:
            return self.q_d.copy(), np.zeros(7)

    def mode_and_errors(self):
        return self.mode, list(self.errors)

    def joint_freshness(self):
        now = time.time()
        if self.stalled_at is None:
            return now, 0.005
        return self.stalled_at, now - self.stalled_at

    def drain_readings(self, joints_only=False):
        return []

    def active_controllers(self, timeout=2.0):
        return set(self.active)

    def switch(self, activate, deactivate, timeout=3.0):
        with self._lock:
            self.switches.append((list(activate), list(deactivate)))
            self.active -= set(deactivate)
            self.active |= set(activate)
        if self.on_switch is not None:
            self.on_switch(activate, deactivate)
        return ""

    @property
    def follow(self):
        return _FakeAction(self)

    def trajectory_goal(self, traj):
        return traj

    def set_collision(self, torque_nm, force_n, timeout=3.0):
        self.collision_calls.append((list(torque_nm), list(force_n)))
        return ""

    def error_recovery(self, timeout=15.0):
        self.recovery_steps.append("error recovery")
        self.mode, self.errors = 2, []
        return ""

    def reactivate_hardware(self, component, timeout=10.0):
        self.recovery_steps.append(f"hardware {component}")
        return ""

    def close(self):
        pass


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
