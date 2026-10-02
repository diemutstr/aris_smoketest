"""One arm's ROS connections: its own DDS domain, its namespace, spun in its own thread.

This is the only file besides driver.py and tools.py that imports ROS.  Names, all under
/arm_<id>:
  franka/joint_states                                    sensor_msgs/JointState (100 Hz)
  franka_robot_state_broadcaster/robot_state             franka_msgs/FrankaRobotState
  aris_joint_impedance_controller/{reference,status}     aris_msgs
  aris_joint_impedance_controller/{hold,resume}          std_srvs/Trigger
  fr3_arm_controller/follow_joint_trajectory             control_msgs action
  action_server/error_recovery                           franka_msgs/ErrorRecovery action
  controller_manager/{list,switch}_controller(s)         controller_manager_msgs
  service_server/set_full_collision_behavior             franka_msgs/SetFullCollisionBehavior
"""
from __future__ import annotations

import collections
import threading

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from aris_msgs.msg import ImpedanceStatus, Reference
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers, SwitchController
from franka_msgs.action import ErrorRecovery
from franka_msgs.msg import FrankaRobotState
from franka_msgs.srv import SetFullCollisionBehavior
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

TRAJECTORY = "fr3_arm_controller"
IMPEDANCE = "aris_joint_impedance_controller"
MODES = {0: "other", 1: "idle", 2: "move", 3: "guiding", 4: "reflex", 5: "user stopped",
         6: "automatic error recovery"}


def wait(future, timeout: float):
    """The future's result, or None after `timeout` seconds (the executor thread fills it)."""
    done = threading.Event()
    future.add_done_callback(lambda _f: done.set())
    if not done.wait(timeout):
        return None
    return future.result()


def duration(seconds: float) -> Duration:
    ns = int(round(seconds * 1e9))
    return Duration(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


class ArmNode:
    def __init__(self, site_arm, joint_names: list[str]):
        self.arm, self.names = site_arm, list(joint_names)
        ns = f"/{site_arm.namespace}"
        self.ctx = rclpy.Context()
        rclpy.init(context=self.ctx, domain_id=site_arm.domain)
        self.node = rclpy.create_node(f"aris_driver_{site_arm.id}", context=self.ctx)
        n = self.node
        self._lock = threading.Lock()
        self.q = self.qd = None
        self.robot_state = None
        self.status = None
        self.statuses = collections.deque(maxlen=4096)   # every status, for the force servo
        self.readings = collections.deque(maxlen=4096)   # (q, F) per robot state, for touch
        self.joint_readings = collections.deque(maxlen=4096)   # q per joint state
        n.create_subscription(JointState, f"{ns}/franka/joint_states", self._on_joints, 10)
        n.create_subscription(FrankaRobotState, f"{ns}/franka_robot_state_broadcaster/robot_state",
                              self._on_robot_state, 10)
        n.create_subscription(ImpedanceStatus, f"{ns}/{IMPEDANCE}/status", self._on_status, 100)
        self.reference = n.create_publisher(
            Reference, f"{ns}/{IMPEDANCE}/reference",
            QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))
        self.follow = ActionClient(n, FollowJointTrajectory,
                                   f"{ns}/{TRAJECTORY}/follow_joint_trajectory")
        self.recovery = ActionClient(n, ErrorRecovery, f"{ns}/action_server/error_recovery")
        self.switch_srv = n.create_client(SwitchController, f"{ns}/controller_manager/switch_controller")
        self.list_srv = n.create_client(ListControllers, f"{ns}/controller_manager/list_controllers")
        self.hold_srv = n.create_client(Trigger, f"{ns}/{IMPEDANCE}/hold")
        self.resume_srv = n.create_client(Trigger, f"{ns}/{IMPEDANCE}/resume")
        self.collision_srv = n.create_client(
            SetFullCollisionBehavior, f"{ns}/service_server/set_full_collision_behavior")
        self.executor = MultiThreadedExecutor(context=self.ctx)
        self.executor.add_node(n)
        self._spin = threading.Thread(target=self.executor.spin, daemon=True)
        self._spin.start()

    # ------------------------------------------------------------------ incoming

    def _ordered(self, js):
        idx = {name: i for i, name in enumerate(js.name)}
        if not all(n in idx for n in self.names):
            return None
        return [idx[n] for n in self.names]

    def _on_joints(self, m: JointState) -> None:
        order = self._ordered(m)
        if order is None:
            return
        q = np.array([m.position[i] for i in order])
        qd = np.array([m.velocity[i] for i in order]) if len(m.velocity) else np.zeros(7)
        with self._lock:
            self.q, self.qd = q, qd
            self.joint_readings.append(q)

    def _on_robot_state(self, m) -> None:
        js = m.measured_joint_state
        order = self._ordered(js)
        f = m.o_f_ext_hat_k.wrench.force
        with self._lock:
            self.robot_state = m
            if order is not None:
                self.readings.append((np.array([js.position[i] for i in order]),
                                      np.array([f.x, f.y, f.z])))

    def drain_readings(self, joints_only: bool = False) -> list:
        """Every (q, F) from the robot state since the last call; `joints_only`: every q from
        the joint states instead (fake hardware has no robot state), with F None."""
        with self._lock:
            if joints_only:
                out = [(q, None) for q in self.joint_readings]
                self.joint_readings.clear()
            else:
                out = list(self.readings)
                self.readings.clear()
            return out

    def _on_status(self, m) -> None:
        with self._lock:
            self.status = m
            self.statuses.append(m)

    def joints(self):
        with self._lock:
            return (None, None) if self.q is None else (self.q.copy(), self.qd.copy())

    def drain_statuses(self) -> list:
        with self._lock:
            out = list(self.statuses)
            self.statuses.clear()
            return out

    def mode_and_errors(self):
        """(robot mode number or None, [names of the current errors])."""
        with self._lock:
            rs = self.robot_state
        if rs is None:
            return None, []
        errs = rs.current_errors
        names = [f for f in errs.get_fields_and_field_types() if getattr(errs, f)]
        return int(rs.robot_mode), names

    # ------------------------------------------------------------------ outgoing

    def publish(self, chunk, extra_f=None) -> None:
        f = chunk.f if extra_f is None else chunk.f + extra_f[None, :]
        self.reference.publish(Reference(
            stream=int(chunk.stream), last=bool(chunk.last), t=[float(x) for x in chunk.t],
            q=chunk.q.ravel().tolist(), qd=chunk.qd.ravel().tolist(), f=f.ravel().tolist(),
            n=chunk.n.ravel().tolist()))

    def trigger(self, client, timeout: float = 2.0) -> str:
        """Calls a Trigger service; "" when done, else why not."""
        if not client.wait_for_service(timeout_sec=timeout):
            return f"{client.srv_name} is not available"
        res = wait(client.call_async(Trigger.Request()), timeout)
        return "" if res is not None and res.success else f"{client.srv_name} did not answer"

    def active_controllers(self, timeout: float = 2.0) -> set[str] | None:
        if not self.list_srv.wait_for_service(timeout_sec=timeout):
            return None
        res = wait(self.list_srv.call_async(ListControllers.Request()), timeout)
        return None if res is None else {c.name for c in res.controller if c.state == "active"}

    def switch(self, activate: list[str], deactivate: list[str], timeout: float = 3.0) -> str:
        if not self.switch_srv.wait_for_service(timeout_sec=timeout):
            return "the controller manager is not available"
        req = SwitchController.Request(activate_controllers=activate,
                                       deactivate_controllers=deactivate,
                                       strictness=SwitchController.Request.STRICT,
                                       activate_asap=True, timeout=duration(timeout))
        res = wait(self.switch_srv.call_async(req), timeout + 1.0)
        if res is None:
            return "switching controllers timed out"
        return "" if res.ok else f"the controller manager refused to switch to {activate}"

    def trajectory_goal(self, traj) -> FollowJointTrajectory.Goal:
        """The trajectory exactly as planned: every knot, its velocity, its time."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.names
        t0 = float(traj.t[0])
        for t, q, qd in zip(traj.t, traj.q, traj.qd):
            goal.trajectory.points.append(JointTrajectoryPoint(
                positions=[float(x) for x in q], velocities=[float(x) for x in qd],
                time_from_start=duration(float(t) - t0)))
        return goal

    def set_collision(self, torque_nm, force_n, timeout: float = 3.0) -> str:
        """The robot's collision thresholds: lower = upper, acceleration = nominal phase.
        "" when set."""
        if not self.collision_srv.wait_for_service(timeout_sec=timeout):
            return "the collision behavior service is not available"
        t, f = [float(x) for x in torque_nm], [float(x) for x in force_n]
        req = SetFullCollisionBehavior.Request(
            lower_torque_thresholds_acceleration=t, upper_torque_thresholds_acceleration=t,
            lower_torque_thresholds_nominal=t, upper_torque_thresholds_nominal=t,
            lower_force_thresholds_acceleration=f, upper_force_thresholds_acceleration=f,
            lower_force_thresholds_nominal=f, upper_force_thresholds_nominal=f)
        res = wait(self.collision_srv.call_async(req), timeout)
        if res is None:
            return "setting the collision thresholds timed out"
        return "" if res.success else f"the robot refused the collision thresholds: {res.error}"

    def error_recovery(self, timeout: float = 15.0) -> str:
        """franka_hardware's automatic error recovery; "" when it succeeded."""
        if not self.recovery.wait_for_server(timeout_sec=2.0):
            return "the error recovery action is not available"
        handle = wait(self.recovery.send_goal_async(ErrorRecovery.Goal()), 5.0)
        if handle is None or not handle.accepted:
            return "the error recovery was refused"
        res = wait(handle.get_result_async(), timeout)
        if res is None or res.status != GoalStatus.STATUS_SUCCEEDED:
            return "the error recovery did not succeed"
        return ""

    def close(self) -> None:
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown(context=self.ctx)
