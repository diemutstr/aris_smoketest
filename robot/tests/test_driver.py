"""The real driver (position control only) against a fake arm node standing in for ROS
(fake_ros.py): every motion kind through the trajectory controller, recovery, stale joints."""
import time
from pathlib import Path

import numpy as np
import pytest

import fake_ros
from aris.kernel.retime import retime
from aris.rig import Rig
from aris.types import JointPath, Motion, Piece
from aris_robot import site as site_mod

fake_ros.install()
from aris_robot import driver as D  # noqa: E402

ROBOT = Path(__file__).resolve().parents[1]
CONFIG = ROBOT.parent / "config"


@pytest.fixture(scope="module")
def rig():
    return Rig.load(CONFIG)


@pytest.fixture(scope="module")
def site():
    return site_mod.load(ROBOT / "site.json")


def _chain(rig, arm_id, kinds, scale=0.08):
    p, arm = rig.park_q(arm_id), rig.arm(arm_id)
    step = scale * np.array([1.0, -1.0, 0.5, 1.0, 0.0, -1.0, 1.0])
    qs = [p + i * step for i in range(len(kinds) + 1)]
    out = []
    for i, k in enumerate(kinds):
        tr = retime(JointPath(np.array([qs[i], qs[i + 1]])), arm.limits, rig.rules())
        tip = arm.tip(tr.q) if k == "draw" else None
        out.append(Motion(k, tr, Piece("a", 0, 0.01) if k == "draw" else None, tip, 1.0))
    return out


def _arm(monkeypatch, rig, site, q0):
    node = fake_ros.FakeArmNode(q0)
    monkeypatch.setattr(D, "ArmNode", lambda site_arm, names: node)
    return D.RosArm(site, rig, "2L"), node


def test_every_kind_flies_through_the_trajectory_controller(monkeypatch, rig, site):
    """lower, draw and lift go to the trajectory controller exactly as planned; it stays
    active and holding; the collision thresholds are the site's."""
    motions = _chain(rig, "2L", ("free", "lower", "draw", "lift"))
    arm, node = _arm(monkeypatch, rig, site, motions[0].q_start)
    for m in motions:
        r = arm.draw(m) if m.kind != "free" else arm.move(m.traj)
        assert r.done, r.why
    assert node.goals == [m.traj for m in motions]
    assert "fr3_arm_controller" in node.active and node.switches == []
    assert arm.set_collision("job") == "" and arm.set_collision("normal") == ""
    c = site.collision
    assert node.collision_calls == [(c["job"]["torque_nm"], c["job"]["force_n"]),
                                    (c["normal"]["torque_nm"], c["normal"]["force_n"])]


def test_a_trajectory_not_starting_at_the_arm_is_refused(monkeypatch, rig, site):
    (m,) = _chain(rig, "2L", ("free",))
    arm, node = _arm(monkeypatch, rig, site, m.q_start + np.array([0, 0.1, 0, 0, 0, 0, 0]))
    r = arm.move(m.traj)
    assert not r.done and "0.1 rad from the arm" in r.why and node.goals == []


def test_fault_then_recover_then_park(monkeypatch, rig, site):
    """A reflex: recovery clears it, brings the hardware and the controllers back (the
    trajectory controller and the broadcasters active), sees the joint states fresh, and a
    park flies without a restart of anything."""
    arm, node = _arm(monkeypatch, rig, site, rig.park_q("2L"))
    said = []
    arm.say = lambda event, **f: said.append(dict(event=event, **f))
    node.mode, node.errors = 4, ["cartesian_reflex"]
    node.q_d = rig.park_q("2L") + np.array([0, 0, 0, 0, 0, 0, 0.05])   # stopped near park
    park = retime(JointPath(np.array([node.q_d, rig.park_q("2L")])), rig.arm("2L").limits,
                  rig.rules())
    assert not arm.move(park).done                              # faulted: refused
    r = arm.recover()
    assert r.done, r.why
    assert node.recovery_steps == ["hardware FrankaHardwareInterface", "error recovery"]
    assert node.active == {"fr3_arm_controller", "joint_state_broadcaster",
                           "franka_robot_state_broadcaster"}
    assert [x["event"] for x in said] == ["recover: hardware component",
                                          "recover: error recovery",
                                          "recover: controllers", "recover: joint states fresh"]
    assert arm.move(park).done and np.allclose(node.q_d, rig.park_q("2L"))
    assert "fr3_arm_controller" in node.active                 # left active and holding


def test_a_stalled_stack_is_restarted_by_recovery(monkeypatch, rig, site):
    arm, node = _arm(monkeypatch, rig, site, rig.park_q("2L"))
    said, restarts = [], []
    arm.say = lambda event, **f: said.append(dict(event=event, **f))
    arm.fresh_wait_s = 1.5
    node.stalled_at = time.time() - 5.0          # joint states 5 s old
    s = arm.state()
    assert not s.ok and np.all(np.isnan(s.q)) and s.flags[0].startswith("stale joint states")
    from aris_robot.runner import reading
    q, why = reading(arm)
    assert q is None and why.startswith("stale joint states (last 5.")

    def restart():
        restarts.append(1)
        node.stalled_at = None                                 # the new stack publishes
    arm.restart_stack = restart
    r = arm.recover()
    assert r.done, r.why
    assert restarts == [1]
    events = [x["event"] for x in said]
    assert events[-3:] == ["recover: joint states not fresh", "recover: restarting the stack",
                           "recover: joint states fresh"]


def test_recover_with_a_dead_hardware_component_ends_fresh(monkeypatch, rig, site):
    """The stack started with FCI off: its hardware component read an error and was taken
    down, the joint states are stale, and error recovery alone fails.  Bringing the component
    back comes first, so the error recovery behind it then works, and the joints are fresh."""
    arm, node = _arm(monkeypatch, rig, site, rig.park_q("2L"))
    said = []
    arm.say = lambda event, **f: said.append(dict(event=event, **f))
    node.hardware_dead, node.stalled_at = True, time.time() - 40.0

    def reactivate(component, timeout=10.0):
        node.recovery_steps.append(f"hardware {component}")
        node.hardware_dead, node.stalled_at = False, None
        return ""

    def error_recovery(timeout=15.0):
        node.recovery_steps.append("error recovery")
        return "the error recovery did not succeed" if node.hardware_dead else ""

    node.reactivate_hardware, node.error_recovery = reactivate, error_recovery
    r = arm.recover()
    assert r.done, r.why
    assert node.recovery_steps == ["hardware FrankaHardwareInterface", "error recovery"]
    assert said[-1]["event"] == "recover: joint states fresh"


def test_recover_runs_every_step_and_restarts_a_stale_stack(monkeypatch, rig, site):
    """A failing step does not stop the sequence; joints still stale after it: the stack
    is restarted and checked again."""
    arm, node = _arm(monkeypatch, rig, site, rig.park_q("2L"))
    said, restarts = [], []
    arm.say = lambda event, **f: said.append(dict(event=event, **f))
    arm.fresh_wait_s = 1.2
    node.stalled_at = time.time() - 40.0
    node.reactivate_hardware = lambda component, timeout=10.0: "no hardware component service"

    def restart():
        restarts.append(1)
        node.stalled_at = None
    arm.restart_stack = restart
    r = arm.recover()
    assert r.done, r.why and restarts == [1]
    events = [x["event"] for x in said]
    assert events[:3] == ["recover: hardware component", "recover: error recovery",
                          "recover: controllers"]
    assert said[0]["ok"] is False and "recover: restarting the stack" in events
    assert events[-1] == "recover: joint states fresh"


class Person:
    """Plays a person at the arm, in a thread, once the trajectory controller is let go:
    `script` is a list of (seconds, mode, joint offset from the hover)."""

    def __init__(self, node, q_hover, script):
        import threading
        self.node, self.q0, self.script = node, np.asarray(q_hover, float), script
        self.thread = threading.Thread(target=self._play, daemon=True)

    def _play(self):
        for dt, mode, off in self.script:
            time.sleep(dt)
            with self.node._lock:
                self.node.mode = mode
                self.node.q_d = self.q0 + off

    def start(self, activate, deactivate):
        if "fr3_arm_controller" in deactivate and not self.thread.is_alive():
            self.thread.start()


SEAT = np.array([0, 0.03, 0, -0.04, 0, 0.02, 0])      # where she puts the pen: 40 mrad away


def _guide_arm(monkeypatch, rig, site, script, settle=0.3, timeout=1.0, mode_known=True):
    hover = rig.park_q("2L")
    arm, node = _arm(monkeypatch, rig, site, hover)
    arm.guide_cfg = dict(arm.guide_cfg, settle_s=settle, timeout_s=timeout)
    said = []
    arm.say = lambda event, **f: said.append(dict(event=event, **f))
    if not mode_known:                  # fake hardware: no robot state broadcaster
        node.mode_and_errors = lambda: (None, [])
        arm.fake = True
    person = Person(node, hover, script)
    node.on_switch = person.start
    m = Motion("guide", __import__("aris.types", fromlist=["Trajectory"]).Trajectory(
        np.array([0.0]), hover[None], np.zeros((1, 7))), piece=Piece("A", 0, 0))
    return arm, node, m, said, hover


def test_a_guide_hands_over_registers_takes_back_and_flies_on(monkeypatch, rig, site):
    script = [(0.1, 3, np.zeros(7)), (0.1, 3, SEAT / 2), (0.1, 3, SEAT), (0.1, 1, SEAT)]
    arm, node, m, said, hover = _guide_arm(monkeypatch, rig, site, script)
    r = arm.guide(m)
    assert r.done and r.why == "check", r.why
    assert np.allclose(r.q, hover + SEAT)                     # the person's pose is the sample
    assert node.switches[0] == ([], ["fr3_arm_controller"])  # let go at the hover
    assert node.switches[1] == (["fr3_arm_controller"], [])  # holds again
    assert "fr3_arm_controller" in node.active
    events = [x["event"] for x in said]
    assert events[0] == "guide: your turn" and said[0]["text"] == "guide: your turn on 2L"
    reg = next(x for x in said if x["event"] == "guide: registered")
    assert reg["episodes"] == 1
    lift, back = node.goals                                   # straight up, then the hover
    assert np.allclose(node.q_d, hover)
    (nxt,) = _chain(rig, "2L", ("free",))                     # the next motion flies
    assert arm.move(nxt.traj).done


def test_a_second_pinch_restarts_the_clock(monkeypatch, rig, site):
    """Let go, then pinch again before the 0.3 s are up (the old ✗): the sample is where she
    leaves it the second time."""
    second = SEAT + np.array([0, 0, 0.03, 0, 0, 0, 0])
    script = [(0.1, 3, SEAT), (0.05, 1, SEAT), (0.15, 3, SEAT), (0.1, 3, second),
              (0.1, 1, second)]
    arm, node, m, said, hover = _guide_arm(monkeypatch, rig, site, script)
    t0 = time.monotonic()
    r = arm.guide(m)
    assert r.done and r.why == "check" and np.allclose(r.q, hover + second)
    assert next(x for x in said if x["event"] == "guide: registered")["episodes"] == 2
    assert time.monotonic() - t0 >= 0.5 + 0.3


def test_a_brief_pinch_without_moving_is_a_skip(monkeypatch, rig, site):
    script = [(0.1, 3, np.zeros(7)), (0.1, 3, np.full(7, 0.005)), (0.05, 1, np.full(7, 0.005))]
    arm, node, m, said, hover = _guide_arm(monkeypatch, rig, site, script)
    r = arm.guide(m)
    assert r.done and r.why == "circle"
    assert any(x["event"] == "guide: skipped" for x in said)


def test_nobody_guides_and_the_motion_fails(monkeypatch, rig, site):
    arm, node, m, said, hover = _guide_arm(monkeypatch, rig, site, [], timeout=0.4)
    r = arm.guide(m)
    assert not r.done and r.why == "nobody guided 2L within 0.4 s"
    assert "fr3_arm_controller" in node.active and np.allclose(node.q_d, hover)
    assert node.goals == []                                   # no lift: it never moved


def test_without_a_robot_mode_the_joints_tell(monkeypatch, rig, site):
    script = [(0.1, None, SEAT / 2), (0.1, None, SEAT * 1.5)]
    arm, node, m, said, hover = _guide_arm(monkeypatch, rig, site, script, mode_known=False)
    r = arm.guide(m)
    assert r.done and r.why == "check" and np.allclose(r.q, hover + SEAT * 1.5)
