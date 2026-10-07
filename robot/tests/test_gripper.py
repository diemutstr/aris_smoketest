"""The gripper on a fake port (the jaws move to where they are sent, or are blocked), and a
grip job through the runner and the stand-in server."""
import json
import math
from pathlib import Path

import pytest

from aris.execute import Job
from aris.execute.drivers.sim import SimArm
from aris.rig import Rig
from aris.types import Refusal
from aris_robot.gripper import Gripper, GripperSettings
from aris_robot.remote import Remote
from aris_robot.runner import run_job
from conftest import CODE
from fake_server import Served, create_app

ROBOT = Path(__file__).resolve().parents[1]
CONFIG = ROBOT.parent / "config"
HOLDER = 0.0359                    # m: the pen holder between the fingers


class FakePort:
    """Jaws at `w`; a move goes to its width, a grasp closes onto the holder (or onto nothing
    when `holder` is None); `blocked`: nothing moves, yet the gripper reports success, as
    libfranka does."""

    def __init__(self, w, holder=HOLDER, blocked=False):
        self.w, self.holder, self.blocked, self.calls = w, holder, blocked, []

    def width(self):
        return self.w

    def homing(self):
        self.calls.append("homing")
        return ""

    def move(self, width, speed):
        self.calls.append(("move", width, speed))
        if not self.blocked:
            self.w = width
        return ""

    def grasp(self, width, speed, force, inner, outer):
        self.calls.append(("grasp", width, speed, force, inner, outer))
        if not self.blocked:
            self.w = self.holder if self.holder is not None else width
        return ""


def _gripper(port):
    site = json.loads((ROBOT / "site.json").read_text())
    rows = []
    g = Gripper(port, GripperSettings.from_site(site["gripper"]),
                say=lambda event, **f: rows.append(dict(event=event, **f)))
    return g, rows


def test_the_old_working_parameters_are_the_defaults():
    g, _ = _gripper(FakePort(0.07))
    port = g.port
    assert g.close().done
    assert port.calls[-1] == ("grasp", 0.0, 0.10, 70.0, 0.0, 0.08)
    port.w = 0.04
    assert g.open().done and port.calls[-1] == ("move", 0.07, 0.10)


def test_open_and_close_are_idempotent_with_a_row():
    g, rows = _gripper(FakePort(0.07))
    r = g.open()
    assert r.done and r.noop and g.port.calls == []
    assert rows[-1]["event"] == "gripper open (nothing to do)"
    assert g.close().done and g.port.w == pytest.approx(HOLDER)
    r = g.close()
    assert r.done and r.noop and len(g.port.calls) == 1
    assert rows[-1]["event"] == "gripper close (nothing to do)"


def test_jaws_that_did_not_move_fail_even_when_the_gripper_says_success():
    g, rows = _gripper(FakePort(HOLDER, blocked=True))
    r = g.close()
    assert not r.done
    assert r.why == ("the jaws did not move (still 35.9 mm) — blocked, or the travel "
                     "calibration is lost: home the gripper")
    assert rows[-1]["width_before_mm"] == 35.9 and rows[-1]["ok"] is False
    r = g.open()
    assert not r.done and "did not move" in r.why


def test_home_and_unknown_verbs():
    g, _ = _gripper(FakePort(HOLDER))
    assert g.run("home").done and g.port.calls == ["homing"]
    assert not g.run("squeeze").done


class Gripping:
    """A simulated arm with a gripper."""

    def __init__(self, slot, port):
        self.arm = SimArm(slot, Rig.load(CONFIG).park_q(slot), speed=math.inf)
        self.arm_id = slot
        self.gripper, _ = _gripper(port)

    def __getattr__(self, name):
        return getattr(self.arm, name)


@pytest.mark.parametrize("blocked", [False, True])
def test_a_grip_job(tmp_path, blocked):
    """The header the server writes: kind grip, slot, verb, params, the code; no phases."""
    jobs = tmp_path / "jobs"
    jobs.mkdir()
    header = dict(kind="grip", slot="2L", verb="close", params=dict(force_n=50.0),
                  code=CODE, pen={})
    Job.create(jobs / "g", header)
    arm = Gripping("2L", FakePort(0.07, blocked=blocked) if not blocked
                   else FakePort(HOLDER, blocked=True))
    app = create_app(jobs)
    with Served(app) as srv:
        res = run_job(Remote(srv.url), "g", Rig.load(CONFIG), CONFIG, tmp_path / "w",
                      {"2L": arm})
    rows = app.state.received["g"]
    assert [r["seq"] for r in rows] == list(range(len(rows)))
    assert arm.gripper.port.calls[0][3] == 50.0              # the job's force
    if not blocked:
        assert res.status == "done"
        assert [r["event"] for r in rows] == ["grip started", "grip done", "job done",
                                              "runner finished"]
        done = rows[1]
        assert done["width_before_m"] == 0.07 and done["width_after_m"] == HOLDER
        assert done["grasped"] is True and done["verb"] == "close"
    else:
        assert res.status == "failed" and "did not move" in res.why
        assert [r["event"] for r in rows] == ["grip started", "failed", "job failed",
                                              "runner finished"]


def test_a_grip_job_for_a_slot_not_here_is_refused(tmp_path):
    jobs = tmp_path / "jobs"
    jobs.mkdir()
    Job.create(jobs / "g", dict(kind="grip", slot="1L", verb="open", params={}, code=CODE))
    with Served(create_app(jobs)) as srv:
        res = run_job(Remote(srv.url), "g", Rig.load(CONFIG), CONFIG, tmp_path / "w", {})
    assert isinstance(res, Refusal) and res.reason == "no_gripper"
