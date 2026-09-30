"""The runner against a stand-in drawing server over real HTTP, with simulated arms.

The server side writes the job while the runner runs it: the phase list and the queue of arm
31 (free, lower, draw, lift) appear motion by motion.  The runner copies them, runs them with
the package's own coordinator and executors, and posts every event back in order.
"""
import math
import threading
import time
from pathlib import Path

import numpy as np
import pytest

from aris.execute import Job
from aris.execute.drivers.sim import SimArm
from aris.execute.queue import digest
from aris.kernel.retime import retime
from aris.rig import Rig
from aris.types import JointPath, Motion, Phase, Piece, Refusal
from aris_robot.remote import Remote
from aris_robot.runner import run_job
from fake_server import Served, create_app

CONFIG = Path(__file__).resolve().parents[2] / "config"
STEP = np.array([0.3, -0.3, 0.0, 0.45, 0.0, -0.45, 0.6])
KINDS = ("free", "lower", "draw", "lift")


class Passed:
    """What the queue keeps of a checker verdict that passed (the checker is not the subject)."""
    passed, tightest, min_clearance, min_clearance_at = True, "test", 0.1, "test"

    def get(self, name):
        raise KeyError(name)


class Recording:
    """A driver that remembers which verb each motion went to."""

    def __init__(self, driver):
        self.driver, self.arm_id, self.calls = driver, driver.arm_id, []

    def move(self, traj):
        self.calls.append("move")
        return self.driver.move(traj)

    def draw(self, motion):
        self.calls.append(f"draw {motion.kind}")
        return self.driver.draw(motion)

    def __getattr__(self, name):
        return getattr(self.driver, name)


@pytest.fixture(scope="module")
def rig():
    return Rig.load(CONFIG)


def _motions(rig, arm_id):
    p, arm = rig.park_q(arm_id), rig.arm(arm_id)
    qs = [p, p + STEP, p + 0.6 * STEP, p + 0.4 * STEP, p]
    out = []
    for i, kind in enumerate(KINDS):
        traj = retime(JointPath(np.array([qs[i], qs[i + 1]])), arm.limits, rig.rules())
        tip = arm.tip(traj.q) if kind == "draw" else None
        out.append(Motion(kind, traj, Piece("a", 0.0, 0.01) if tip is not None else None,
                          tip, intensity=0.5))
    return out


def _header(rig):
    calib = {a: (m.T_table_base, m.tip_hand, m.calibration) for a, m in rig.mounts.items()}
    return dict(rig_digest=digest(rig), calibration_digest=digest(calib))


def _writer(rig, job, active, motions, pause=0.02):
    """The server's side: the phase, then the motions one by one, then the ends."""
    def work():
        phase = Phase("phase 1", active, tuple(a for a in rig.arm_ids if a not in active), ())
        job.add_phase(phase)
        for a in active:
            q = job.queue(phase.name, a)
            for m in motions:
                time.sleep(pause)
                assert q.append(m, Passed()) == KINDS.index(m.kind)
            q.close()
        job.end_phases()
    t = threading.Thread(target=work, daemon=True)
    t.start()
    return t


def test_a_job_runs_as_it_is_written_and_every_event_reaches_the_server(rig, tmp_path):
    server_dir, work = tmp_path / "server", tmp_path / "robot"
    server_dir.mkdir()
    job = Job.create(server_dir / "j1", _header(rig))
    app = create_app(server_dir)
    arm = Recording(SimArm(31, rig.park_q(31), speed=50.0))
    with Served(app) as srv:
        writer = _writer(rig, job, (31,), _motions(rig, 31))
        t0 = time.perf_counter()
        res = run_job(Remote(srv.url), "j1", rig, CONFIG, work, {31: arm})
        wall = time.perf_counter() - t0
        writer.join()
    print(f"job of 4 motions at 50x: {wall:.2f} s wall")
    assert res.status == "done", res.why
    assert arm.calls == ["move", "draw lower", "draw draw", "draw lift"]
    assert np.abs(arm.state().q - rig.park_q(31)).max() <= 1e-9
    local = work / "j1"
    assert (local / "phase_1__arm31.queue").read_bytes() == \
        job.queue("phase 1", 31).path.read_bytes()
    rows = app.state.received["j1"]
    assert [r["seq"] for r in rows] == list(range(len(rows)))
    lines = (local / "events.jsonl").read_text().splitlines()
    assert len(rows) == len(lines)
    names = [r["event"] for r in rows]
    assert names[0] == "job started" and names[-1] == "job done"
    assert names.count("motion started") == 4 and names.count("motion done") == 4
    assert names.index("phase end check") < names.index("phase done")
    assert [r["kind"] for r in rows if r["event"] == "motion started"] == list(KINDS)


def test_a_phase_that_needs_an_unmounted_arm_stops_the_job(rig, tmp_path):
    server_dir = tmp_path / "server"
    server_dir.mkdir()
    job = Job.create(server_dir / "j2", _header(rig))
    app = create_app(server_dir)
    arm = SimArm(31, rig.park_q(31), speed=50.0)
    with Served(app) as srv:
        writer = _writer(rig, job, (31, 71), _motions(rig, 31)[:1], pause=0.0)
        res = run_job(Remote(srv.url), "j2", rig, CONFIG, tmp_path / "robot", {31: arm})
        writer.join()
    assert res.status != "done"
    assert "not mounted" in res.why and "71" in res.why
    events = [r["event"] for r in app.state.received["j2"]]
    assert "refused" in events


def test_a_stop_on_the_server_stops_the_arms(rig, tmp_path):
    server_dir = tmp_path / "server"
    server_dir.mkdir()
    job = Job.create(server_dir / "j3", _header(rig))
    app = create_app(server_dir)
    arm = SimArm(31, rig.park_q(31), speed=1.0)        # real time: about 6 s of motion
    with Served(app) as srv:
        writer = _writer(rig, job, (31,), _motions(rig, 31), pause=0.0)
        threading.Timer(1.0, lambda: app.state.stop.add("j3")).start()
        res = run_job(Remote(srv.url), "j3", rig, CONFIG, tmp_path / "robot", {31: arm})
        writer.join()
    assert res.status == "stopped"
    assert "stopped" in arm.state().flags
    assert app.state.received["j3"][-1]["event"] == "job stopped"


def test_a_stop_while_waiting_for_the_plan_ends_the_job(rig, tmp_path):
    server_dir = tmp_path / "server"
    server_dir.mkdir()
    Job.create(server_dir / "j5", _header(rig))                 # no phase ever written
    app = create_app(server_dir)
    with Served(app) as srv:
        threading.Timer(0.5, lambda: app.state.stop.add("j5")).start()
        res = run_job(Remote(srv.url), "j5", rig, CONFIG, tmp_path / "robot",
                      {31: SimArm(31, rig.park_q(31))})
    assert res.status == "stopped"


def test_the_copy_survives_a_link_that_keeps_dropping(rig, tmp_path):
    server_dir = tmp_path / "server"
    server_dir.mkdir()
    job = Job.create(server_dir / "j6", _header(rig))
    app = create_app(server_dir, max_bytes=2000)              # hangs up every 2000 bytes
    arm = SimArm(31, rig.park_q(31), speed=math.inf)
    with Served(app) as srv:
        _writer(rig, job, (31,), _motions(rig, 31), pause=0.0).join()
        res = run_job(Remote(srv.url), "j6", rig, CONFIG, tmp_path / "robot", {31: arm})
    assert res.status == "done", res.why
    assert (tmp_path / "robot" / "j6" / "phase_1__arm31.queue").read_bytes() == \
        job.queue("phase 1", 31).path.read_bytes()


def test_a_job_planned_for_another_rig_is_refused(rig, tmp_path):
    server_dir = tmp_path / "server"
    server_dir.mkdir()
    Job.create(server_dir / "j4", dict(rig_digest="not this rig"))
    with Served(create_app(server_dir)) as srv:
        res = run_job(Remote(srv.url), "j4", rig, CONFIG, tmp_path / "robot", {})
        assert isinstance(res, Refusal) and res.reason == "wrong_rig"
        missing = run_job(Remote(srv.url), "nope", rig, CONFIG, tmp_path / "robot", {})
        assert isinstance(missing, Refusal) and missing.reason == "server"
    down = run_job(Remote(srv.url, timeout=0.5), "j4", rig, CONFIG, tmp_path / "robot", {})
    assert isinstance(down, Refusal) and down.reason == "unreachable"
