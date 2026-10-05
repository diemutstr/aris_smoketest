"""The calibration driver on fakes (a fake panda-py FCI and SimDesk), and serve's hand-over
between an arm's ROS stack and the calibration driver for a mark job."""
import math
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from aris.execute import EventLog, Executor, Job
from aris.kernel.retime import retime
from aris.rig import Rig
from aris.types import JointPath, Motion, Phase, Piece, Trajectory
from aris_robot.calib import CalibArm, CalibSettings
from aris_robot.desk import SimDesk
from aris_robot.remote import Remote
from aris_robot.serve import Operator
from aris_robot.simarm import SimFci, SimTouchArm
from aris_robot.touch import Kinematics, straight_on
from fake_server import Served, create_app
from sim_touch import hover_q

CONFIG = Path(__file__).resolve().parents[2] / "config"
SLOT = "2R"
FAST = CalibSettings(button_timeout_s=0.3, standstill_dt=0.01, standstill_timeout_s=0.5,
                     reconnect_s=1.0)


class Passed:
    passed, tightest, min_clearance, min_clearance_at = True, "test", 0.1, "test"

    def get(self, name):
        raise KeyError(name)


@pytest.fixture(scope="module")
def rig():
    return Rig.load(CONFIG)


class FakeRobot:
    """The robot behind the FCI: joints, mode, the moves it was given, the connections."""

    def __init__(self, q):
        self.q, self.mode, self.errors = np.array(q, float), 2, []
        self.settle = []              # joints the next reads return first (an arm settling)
        self.followed, self.connects, self.closes = [], 0, 0


class FakeFci:
    def __init__(self, robot: FakeRobot):
        robot.connects += 1
        self.r = robot

    def state(self):
        q = self.r.settle.pop(0) if self.r.settle else self.r.q
        return np.array(q, float), np.zeros(7), self.r.mode, list(self.r.errors)

    def follow(self, traj, halt):
        self.r.followed.append(traj)
        self.r.q = np.array(traj.q[-1], float)
        return ""

    def recover(self):
        self.r.mode, self.r.errors = 2, []
        return ""

    def close(self):
        self.r.closes += 1


def _person_pose(rig, q_hover, down=0.04):
    """Where a person leaves the arm: the pen 40 mm below the hover, the hand turned a little."""
    kin = Kinematics.of(rig, SLOT)
    d = straight_on(kin.arm, kin.arm.limits, kin.rules, q_hover, kin.down, down, 0.01)
    q = d.q[-1].copy()
    q[6] += 0.05
    return q


def _arm(rig, script, person=None, settle=None, settings=FAST):
    q0 = hover_q(rig, SLOT)
    robot = FakeRobot(q0)
    said = []

    def on_mode(name):
        if name == "programming" and person is not None:
            robot.q = person.copy()
            robot.mode = 3                                   # guiding
        if name == "execution":
            robot.mode = 2
            robot.settle = list(settle or [])

    desk = SimDesk(script, on_mode=on_mode)

    def connect():
        if desk.current != "execution" or ("fci", False) == next(
                (c for c in reversed(desk.calls) if c[0] == "fci"), ("fci", True)):
            raise RuntimeError("FCI is not active")
        return FakeFci(robot)

    arm = CalibArm(SLOT, rig, connect, desk, settings,
                   say=lambda event, **f: said.append(dict(event=event, **f)))
    return arm, robot, desk, said, q0


def _guide(q0, mark="B"):
    return Motion("guide", Trajectory(np.array([0.0]), q0[None], np.zeros((1, 7))),
                  piece=Piece(mark, 0.0, 0.0))


@pytest.mark.parametrize("button", ["check", "circle"])
def test_a_guide_round_trip(rig, button):
    person = _person_pose(rig, hover_q(rig, SLOT))
    arm, robot, desk, said, q0 = _arm(rig, ["up", button], person=person)
    r = arm.guide(_guide(q0))
    assert r.done and r.why == button
    assert np.array_equal(r.q, person)                         # the person's pose is the sample
    assert desk.calls[:2] == [("fci", False), ("mode", "programming")]
    assert desk.calls[2][0] == "buttons" and desk.calls[3:] == [("mode", "execution"),
                                                                ("fci", True)]
    assert [s["event"] for s in said] == ["guide: handed over", f"guide: button {button}",
                                          "guide: taken back"]
    assert said[0]["mark"] == "B" and said[1]["button"] == button
    assert robot.connects == 2 and robot.closes == 1           # connection made again
    lift, back = robot.followed                                # straight up, then the hover
    kin = Kinematics.of(rig, SLOT)
    rise = (kin.tip(lift.q[-1]) - kin.tip(lift.q[0]))[0] @ kin.normal
    assert rise == pytest.approx(0.03, abs=1e-5)
    assert np.array_equal(robot.q, q0) and np.array_equal(back.q[-1], q0)


def test_cross_keeps_the_arm_with_the_person_until_check_or_circle(rig):
    person = _person_pose(rig, hover_q(rig, SLOT))
    arm, robot, desk, said, q0 = _arm(rig, ["cross", "cross", "check"], person=person)
    r = arm.guide(_guide(q0))
    assert r.done and r.why == "check" and np.array_equal(r.q, person)
    assert [c for c in desk.calls if c[0] == "mode"] == [("mode", "programming"),
                                                        ("mode", "execution")]
    assert [s["event"] for s in said] == ["guide: handed over",
                                          "guide: button cross, waiting",
                                          "guide: button cross, waiting",
                                          "guide: button check", "guide: taken back"]


def test_every_desk_call_is_a_row_with_its_status(monkeypatch):
    import sys
    import types
    from aris_robot.desk import PandaDesk

    class Answer:
        def __init__(self, status):
            self.status_code = status

    class FakeDesk:
        def __init__(self, ip, user, pw, platform):
            self.requests = []

        def take_control(self, force):
            return Answer(200)

        def _request(self, method, path, json=None):
            self.requests.append((method, path, json))
            return Answer(404 if json == {"mode": "Programming"} else 200)

        def activate_fci(self):
            return None

    monkeypatch.setitem(sys.modules, "panda_py", types.SimpleNamespace(Desk=FakeDesk))
    rows = []
    endpoint = dict(method="post", path="/desk/api/operating-mode",
                    body=dict(programming={"mode": "Programming"},
                              execution={"mode": "Execution"}))
    d = PandaDesk("192.168.50.14", "u", "p", endpoint,
                  say=lambda event, **f: rows.append(dict(event=event, **f)))
    d.mode("execution")
    d.fci(True)
    with pytest.raises(RuntimeError, match="404"):
        d.mode("programming")                                 # a wrong endpoint shows at once
    assert [(r["event"], r["status"]) for r in rows] == [
        ("desk: login", None), ("desk: take control", 200), ("desk: mode execution", 200),
        ("desk: fci on", None), ("desk: mode programming", 404)]
    assert d.desk.requests[0] == ("post", "/desk/api/operating-mode", {"mode": "Execution"})


def test_the_sample_waits_for_the_arm_to_stand_still(rig):
    person = _person_pose(rig, hover_q(rig, SLOT))
    drift = [person + 1e-3, person + 5e-4, person + 2e-4]      # settling after the hand-back
    arm, robot, _, _, q0 = _arm(rig, ["check"], person=person, settle=drift)
    r = arm.guide(_guide(q0))
    assert r.done and np.array_equal(r.q, person)
    restless = [person + 1e-3 * (i % 2) for i in range(1000)]  # never still
    arm, robot, _, _, q0 = _arm(rig, ["check"], person=person, settle=restless)
    r = arm.guide(_guide(q0))
    assert not r.done and "did not stand still" in r.why
    assert robot.followed == []                                # it holds where it is


def test_no_button_gives_up_in_execution_mode(rig):
    arm, robot, desk, said, q0 = _arm(rig, [None])
    t0 = time.monotonic()
    r = arm.guide(_guide(q0))
    assert not r.done and r.why.startswith("no pilot button within 0.3 s")
    assert time.monotonic() - t0 < 2.0
    assert desk.current == "execution" and desk.calls[-1] == ("fci", True)
    assert robot.followed == [] and "guide: no button" in [s["event"] for s in said]


def test_the_other_verbs(rig):
    arm, robot, _, _, q0 = _arm(rig, [])
    m = Motion("draw", Trajectory(np.array([0.0, 1.0]), np.array([q0, q0]), np.zeros((2, 7))))
    assert arm.draw(m).why.startswith("not this driver")
    assert arm.touch(m).why.startswith("not this driver")
    traj = retime(JointPath(np.array([q0, q0 + 0.02])), rig.arm(SLOT).limits, rig.rules())
    assert arm.move(traj).done and robot.followed[-1] is traj
    assert not arm.move(traj).done                             # not at its start any more
    robot.mode, robot.errors = 4, ["joint_reflex"]
    assert not arm.state().ok
    assert arm.recover().done and arm.state().ok
    robot.mode = 5
    assert "user stopped" in arm.recover().why


def test_the_executor_writes_the_registered_row(rig, tmp_path):
    person = _person_pose(rig, hover_q(rig, SLOT))
    arm, robot, _, _, q0 = _arm(rig, ["circle"], person=person)
    job = Job.create(tmp_path / "mark", {})
    q = job.queue("mark", SLOT)
    assert q.append(_guide(q0, "A"), Passed()) == 0
    q.close()
    log = EventLog(job.log_path)
    run = Executor(SLOT, arm, log, rig).run(q, threading.Event())
    assert run.status == "finished" and run.done == 1, run.why
    row = next(r for r in log.read() if r["event"] == "registered")
    assert row["button"] == "circle" and row["mark"] == "A"
    assert np.allclose(row["q"], person)


class FakeStacks:
    def __init__(self):
        self.calls, self.state = [], {}

    def pause(self, arm, grace=15.0):
        self.calls.append(("pause", arm, time.time()))
        return ""

    def resume(self, arm):
        self.calls.append(("resume", arm, time.time()))


def test_serve_hands_each_arm_to_the_calibration_driver_for_its_phase(rig, tmp_path):
    slots = ("2R", "3R")
    jobs = tmp_path / "jobs"
    jobs.mkdir()
    from test_runner import _header
    job = Job.create(jobs / "m1", dict(_header(rig), kind="mark"))
    for a in slots:
        park, hover = rig.park_q(a), hover_q(rig, a)
        phase = Phase(f"mark {a}", (a,), tuple(b for b in rig.arm_ids if b != a), ())
        job.add_phase(phase)
        qq = job.queue(phase.name, a)
        for m in (Motion("free", retime(JointPath(np.array([park, hover])), rig.arm(a).limits,
                                        rig.rules())),
                  _guide(hover, f"mark-{a}"),
                  Motion("free", retime(JointPath(np.array([hover, park])), rig.arm(a).limits,
                                        rig.rules()))):
            assert isinstance(qq.append(m, Passed()), int)
        qq.close()
    job.end_phases()
    drivers = {a: SimTouchArm(rig, a, rig.park_q(a), speed=math.inf) for a in slots}
    stacks = FakeStacks()
    made = []

    def make_calib(slot, rig_, say):
        made.append((slot, time.time()))
        return CalibArm(slot, rig_, lambda: SimFci(drivers[slot]), SimDesk(["check"]),
                        FAST, say=say)

    app = create_app(jobs)
    app.state.commands.append(dict(id="c1", command="run", job="m1"))
    with Served(app) as srv:
        op = Operator(Remote(srv.url), CONFIG, tmp_path / "work", drivers, tmp_path / "log",
                      stacks=stacks, idle_s=5.0, wait_s=0.3, make_calib=make_calib)
        t = threading.Thread(target=op.serve, daemon=True)
        t.start()
        ok = False
        t_end = time.monotonic() + 30
        while time.monotonic() < t_end and not ok:
            ok = any(r["event"] == "run ended" for r in app.state.rows)
            time.sleep(0.05)
        op.quit.set()
        t.join(timeout=5)
    ended = next(r for r in app.state.rows if r["event"] == "run ended")
    assert ended["status"] == "done", ended["why"]
    assert [(c[0], c[1]) for c in stacks.calls] == [("pause", "2R"), ("resume", "2R"),
                                                     ("pause", "3R"), ("resume", "3R")]
    reg = {r["arm"]: r for r in app.state.received["m1"] if r["event"] == "registered"}
    assert set(reg) == set(slots) and all(r["button"] == "check" for r in reg.values())
    for i, a in enumerate(slots):                  # stop -> calibration -> start, per arm
        paused, resumed = stacks.calls[2 * i][2], stacks.calls[2 * i + 1][2]
        assert paused <= made[i][1] <= reg[a]["time"] <= resumed
    events = [r["event"] for r in app.state.rows]
    for e in ("calibration driver: stack stopped", "calibration driver: connected",
              "guide: handed over", "guide: button check", "guide: taken back",
              "calibration driver: disconnected"):
        assert events.count(e) == 2, e
    assert np.allclose(ended["where"]["2R"], rig.park_q("2R"))


def test_a_mark_job_without_a_calibration_driver_is_refused(rig, tmp_path):
    jobs = tmp_path / "jobs"
    jobs.mkdir()
    from test_runner import _header
    Job.create(jobs / "m2", dict(_header(rig), kind="mark"))
    app = create_app(jobs)
    app.state.commands.append(dict(id="c", command="run", job="m2"))
    drivers = {"2R": SimTouchArm(rig, "2R", rig.park_q("2R"), speed=math.inf)}
    with Served(app) as srv:
        op = Operator(Remote(srv.url), CONFIG, tmp_path / "w", drivers, tmp_path / "log",
                      idle_s=5.0, wait_s=0.3)
        t = threading.Thread(target=op.serve, daemon=True)
        t.start()
        t_end = time.monotonic() + 10
        while time.monotonic() < t_end and not any(r["event"] == "run refused"
                                                    for r in app.state.rows):
            time.sleep(0.05)
        op.quit.set()
        t.join(timeout=5)
    r = next(r for r in app.state.rows if r["event"] == "run refused")
    assert r["reason"] == "no_calibration_driver"


_ = SimpleNamespace
