"""The runner against a stand-in drawing server over real HTTP, with simulated arms.

The server side writes the job while the runner runs it: the phase list and the queue of arm
2L (free, lower, draw, lift) appear motion by motion.  The runner copies them, runs them with
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
from conftest import CODE
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
    return dict(rig_digest=digest(rig), calibration_digest=digest(calib), pen=rig.pen(),
                code=CODE)


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
    arm = Recording(SimArm("2L", rig.park_q("2L"), speed=50.0))
    with Served(app) as srv:
        writer = _writer(rig, job, ("2L",), _motions(rig, "2L"))
        t0 = time.perf_counter()
        res = run_job(Remote(srv.url), "j1", rig, CONFIG, work, {"2L": arm})
        wall = time.perf_counter() - t0
        writer.join()
    print(f"job of 4 motions at 50x: {wall:.2f} s wall")
    assert res.status == "done", res.why
    assert arm.calls == ["move", "draw lower", "draw draw", "draw lift"]
    assert np.abs(arm.state().q - rig.park_q("2L")).max() <= 1e-9
    local = work / "j1"
    assert (local / "phase_1__2L.queue").read_bytes() == \
        job.queue("phase 1", "2L").path.read_bytes()
    rows = app.state.received["j1"]
    assert [r["seq"] for r in rows] == list(range(len(rows)))
    lines = (local / "events.jsonl").read_text().splitlines()
    assert len(rows) == len(lines)
    names = [r["event"] for r in rows]
    assert names[0] == "runner started" and names[1] == "job started"
    assert names[-2] == "job done" and names[-1] == "runner finished"
    # where the arm stands: at the start (before anything moved), on every row about the
    # arm, and at the end
    assert rows[0]["job"] == "j1" and set(rows[0]["where"]) == {"2L"}
    assert np.array_equal(rows[0]["where"]["2L"], rig.park_q("2L"))
    about_arm = [r for r in rows if "arm" in r]
    assert about_arm and all(len(r["q"]) == 7 for r in about_arm)
    motions = _motions(rig, "2L")
    starts = [r["q"] for r in rows if r["event"] == "motion started"]
    ends = [r["q"] for r in rows if r["event"] == "motion done"]
    for m, q0, q1 in zip(motions, starts, ends):
        assert np.abs(np.array(q0) - m.q_start).max() <= 1e-9
        assert np.abs(np.array(q1) - m.q_end).max() <= 1e-9
    assert rows[-1]["status"] == "done"
    assert np.abs(np.array(rows[-1]["where"]["2L"]) - rig.park_q("2L")).max() <= 1e-9
    assert names.count("motion started") == 4 and names.count("motion done") == 4
    assert names.index("phase end check") < names.index("phase done")
    assert [r["kind"] for r in rows if r["event"] == "motion started"] == list(KINDS)


def test_an_arm_not_where_the_plan_starts_is_refused_and_holds(rig, tmp_path):
    """The server planned from a position that is no longer true: joint 5 moved 50 mrad."""
    server_dir = tmp_path / "server"
    server_dir.mkdir()
    job = Job.create(server_dir / "j7", _header(rig))
    app = create_app(server_dir)
    off = rig.park_q("2L") + np.array([0, 0, 0, 0, 0.05, 0, 0])   # beyond 0.03
    arm = SimArm("2L", off, speed=50.0)
    with Served(app) as srv:
        _writer(rig, job, ("2L",), _motions(rig, "2L"), pause=0.0).join()
        res = run_job(Remote(srv.url), "j7", rig, CONFIG, tmp_path / "robot", {"2L": arm})
    assert res.status == "failed" and "not at the start" in res.why
    rows = app.state.received["j7"]
    bad = next(r for r in rows if r["event"] == "failed")
    assert bad["arm"] == "2L" and bad["index"] == 0 and "joint 5" in bad["why"]
    assert f"tolerance {rig.execution().start_tolerance:g}" in bad["why"]
    assert np.array_equal(bad["q"], off)                       # where it really stands
    assert "motion started" not in [r["event"] for r in rows]  # nothing moved
    assert np.array_equal(arm.state().q, off) and "holding" in arm.state().flags
    assert np.array_equal(rows[-1]["where"]["2L"], off)


def test_a_park_job_runs_like_any_other(rig, tmp_path):
    """The server's park job: one arm per phase, one free motion each, no drawing in it (no
    piece, no tips, no rig digests in its header); both arms end at their parks."""
    server_dir = tmp_path / "server"
    server_dir.mkdir()
    job = Job.create(server_dir / "p1", dict(kind="park", code=CODE))
    app = create_app(server_dir)
    start = {a: rig.park_q(a) + 0.03 * np.array([1, -1, 1, 1, -1, 1, 1.0]) for a in ("2L", "2R")}
    arms = {a: SimArm(a, q, speed=50.0) for a, q in start.items()}
    for a in ("2L", "2R"):
        phase = Phase(f"park arm {a}", (a,), tuple(b for b in rig.arm_ids if b != a), ())
        job.add_phase(phase)
        q = job.queue(phase.name, a)
        traj = retime(JointPath(np.array([start[a], rig.park_q(a)])), rig.arm(a).limits,
                      rig.rules())
        assert q.append(Motion("free", traj), Passed()) == 0
        q.close()
    job.end_phases()
    with Served(app) as srv:
        res = run_job(Remote(srv.url), "p1", rig, CONFIG, tmp_path / "robot", arms)
    assert res.status == "done", res.why
    rows = app.state.received["p1"]
    for a in ("2L", "2R"):
        assert np.array_equal(rows[0]["where"][str(a)], start[a])
        assert np.abs(np.array(rows[-1]["where"][str(a)]) - rig.park_q(a)).max() <= 1e-9
    assert [r["event"] for r in rows][-2:] == ["job done", "runner finished"]


def test_a_phase_that_needs_an_unmounted_arm_stops_the_job(rig, tmp_path):
    server_dir = tmp_path / "server"
    server_dir.mkdir()
    job = Job.create(server_dir / "j2", _header(rig))
    app = create_app(server_dir)
    arm = SimArm("2L", rig.park_q("2L"), speed=50.0)
    with Served(app) as srv:
        writer = _writer(rig, job, ("2L", "2R"), _motions(rig, "2L")[:1], pause=0.0)
        res = run_job(Remote(srv.url), "j2", rig, CONFIG, tmp_path / "robot", {"2L": arm})
        writer.join()
    assert res.status != "done"
    assert "not mounted" in res.why and "2R" in res.why
    events = [r["event"] for r in app.state.received["j2"]]
    assert "refused" in events


def test_a_stop_on_the_server_stops_the_arms(rig, tmp_path):
    server_dir = tmp_path / "server"
    server_dir.mkdir()
    job = Job.create(server_dir / "j3", _header(rig))
    app = create_app(server_dir)
    arm = SimArm("2L", rig.park_q("2L"), speed=1.0)        # real time: about 6 s of motion
    with Served(app) as srv:
        writer = _writer(rig, job, ("2L",), _motions(rig, "2L"), pause=0.0)
        threading.Timer(1.0, lambda: app.state.stop.add("j3")).start()
        res = run_job(Remote(srv.url), "j3", rig, CONFIG, tmp_path / "robot", {"2L": arm})
        writer.join()
    assert res.status == "stopped"
    assert "stopped" in arm.state().flags
    rows = app.state.received["j3"]
    assert [r["event"] for r in rows][-2:] == ["job stopped", "runner finished"]
    assert rows[-1]["status"] == "stopped"
    assert np.array_equal(rows[-1]["where"]["2L"], arm.state().q)   # stopped mid-motion


def test_a_stop_while_waiting_for_the_plan_ends_the_job(rig, tmp_path):
    server_dir = tmp_path / "server"
    server_dir.mkdir()
    Job.create(server_dir / "j5", _header(rig))                 # no phase ever written
    app = create_app(server_dir)
    with Served(app) as srv:
        threading.Timer(0.5, lambda: app.state.stop.add("j5")).start()
        res = run_job(Remote(srv.url), "j5", rig, CONFIG, tmp_path / "robot",
                      {"2L": SimArm("2L", rig.park_q("2L"))})
    assert res.status == "stopped"


def test_the_copy_survives_a_link_that_keeps_dropping(rig, tmp_path):
    server_dir = tmp_path / "server"
    server_dir.mkdir()
    job = Job.create(server_dir / "j6", _header(rig))
    app = create_app(server_dir, max_bytes=2000)              # hangs up every 2000 bytes
    arm = SimArm("2L", rig.park_q("2L"), speed=math.inf)
    with Served(app) as srv:
        _writer(rig, job, ("2L",), _motions(rig, "2L"), pause=0.0).join()
        res = run_job(Remote(srv.url), "j6", rig, CONFIG, tmp_path / "robot", {"2L": arm})
    assert res.status == "done", res.why
    assert (tmp_path / "robot" / "j6" / "phase_1__2L.queue").read_bytes() == \
        job.queue("phase 1", "2L").path.read_bytes()


@pytest.mark.parametrize("shape", ["pens", "pen", None])
def test_the_job_headers_pen_is_recorded(rig, tmp_path, shape):
    """Each slot's pen from the header's `pens: {slot: pen}` is recorded in the first row (its
    press is the plan's); an older header's one `pen` is every slot's; a header without either
    falls back to this PC's rig file, and the row says so."""
    server_dir = tmp_path / "server"
    server_dir.mkdir()
    header = _header(rig)
    header.pop("pen", None)
    other = dict(rig.pen(), press_m=0.002)
    if shape == "pens":
        header["pens"] = {"2L": other, "2R": dict(rig.pen(), press_m=0.009)}
    elif shape == "pen":
        header["pen"] = other
    job = Job.create(server_dir / "pen", header)
    app = create_app(server_dir)
    arm = Recording(SimArm("2L", rig.park_q("2L"), speed=math.inf))
    with Served(app) as srv:
        _writer(rig, job, ("2L",), _motions(rig, "2L"), pause=0.0).join()
        res = run_job(Remote(srv.url), "pen", rig, CONFIG, tmp_path / "robot", {"2L": arm})
    assert res.status == "done", res.why
    first = app.state.received["pen"][0]
    want = rig.pen() if shape is None else other
    assert first["pens"] == {"2L": want} and first["pen"] == want
    assert first["pen_from"] == {"2L": {"pens": "job header", "pen": "job header (one pen)",
                                        None: "rig file"}[shape]}


class ModeRecording(Recording):
    """Records the collision-threshold calls the runner makes."""

    def __init__(self, driver, collision_fails=False):
        super().__init__(driver)
        self.collision, self.fails = [], collision_fails

    def set_collision(self, which):
        self.collision.append(which)
        return "refused by the robot" if self.fails and which == "job" else ""


@pytest.mark.parametrize("tracking", [None, "position"])
def test_a_job_raises_the_thresholds_and_restores_them(rig, tmp_path, tracking):
    """Joint position control, the one mode (said or not): the collision thresholds are the
    site's "job" ones while the job runs and "normal" after."""
    server_dir = tmp_path / "server"
    server_dir.mkdir()
    header = _header(rig)
    if tracking:
        header["tracking"] = tracking
    job = Job.create(server_dir / "m", header)
    app = create_app(server_dir)
    arm = ModeRecording(SimArm("2L", rig.park_q("2L"), speed=math.inf))
    robots = {"2L": dict(robot="fr3-31", identity="unverified")}
    with Served(app) as srv:
        _writer(rig, job, ("2L",), _motions(rig, "2L"), pause=0.0).join()
        res = run_job(Remote(srv.url), "m", rig, CONFIG, tmp_path / "robot", {"2L": arm},
                      robots=robots)
    assert res.status == "done", res.why
    assert arm.collision == ["job", "normal"]
    first = app.state.received["m"][0]
    assert first["tracking"] == "position" and first["robots"] == robots
    assert first["pen"]["press_m"] == rig.pen()["press_m"]


def test_the_header_sets_the_start_tolerance(rig, tmp_path):
    server_dir = tmp_path / "server"
    server_dir.mkdir()
    header = dict(_header(rig), execution=dict(start_tolerance_rad=0.04))
    job = Job.create(server_dir / "mk", header)
    app = create_app(server_dir)
    arm = ModeRecording(SimArm("2L", rig.park_q("2L"), speed=math.inf))
    with Served(app) as srv:
        _writer(rig, job, ("2L",), _motions(rig, "2L"), pause=0.0).join()
        res = run_job(Remote(srv.url), "mk", rig, CONFIG, tmp_path / "robot", {"2L": arm})
    assert res.status == "done", res.why
    assert arm.collision == ["job", "normal"]
    assert app.state.received["mk"][0]["start_tolerance"] == 0.04


def test_refusals_before_anything_moves(rig, tmp_path):
    server_dir = tmp_path / "server"
    server_dir.mkdir()
    Job.create(server_dir / "bad", dict(_header(rig), tracking="impedance"))
    Job.create(server_dir / "ok", _header(rig))
    arm = ModeRecording(SimArm("2L", rig.park_q("2L"), speed=math.inf), collision_fails=True)
    with Served(create_app(server_dir)) as srv:
        r = Remote(srv.url)
        bad = run_job(r, "bad", rig, CONFIG, tmp_path / "a", {"2L": arm})
        assert isinstance(bad, Refusal) and bad.reason == "bad_header"
        wrong = run_job(r, "ok", rig, CONFIG, tmp_path / "b", {"2L": arm},
                        robots={"2L": dict(robot="fr3-31", identity="mismatch: found fr3-13")})
        assert isinstance(wrong, Refusal) and wrong.reason == "wrong_robot"
    assert arm.calls == []                                     # nothing moved


def test_thresholds_the_robot_refuses_are_noted_not_a_refusal(rig, tmp_path):
    """The arms keep their normal (lower) thresholds; the job runs and says so."""
    server_dir = tmp_path / "server"
    server_dir.mkdir()
    job = Job.create(server_dir / "t", _header(rig))
    app = create_app(server_dir)
    arm = ModeRecording(SimArm("2L", rig.park_q("2L"), speed=math.inf), collision_fails=True)
    with Served(app) as srv:
        _writer(rig, job, ("2L",), _motions(rig, "2L"), pause=0.0).join()
        res = run_job(Remote(srv.url), "t", rig, CONFIG, tmp_path / "r", {"2L": arm})
    assert res.status == "done", res.why
    first = app.state.received["t"][0]
    assert "refused by the robot" in first["collision_thresholds_not_set"]
    assert arm.collision == ["job", "normal"]


def test_a_job_planned_for_another_rig_is_refused(rig, tmp_path):
    server_dir = tmp_path / "server"
    server_dir.mkdir()
    Job.create(server_dir / "j4", dict(rig_digest="not this rig", code=CODE))
    with Served(create_app(server_dir)) as srv:
        res = run_job(Remote(srv.url), "j4", rig, CONFIG, tmp_path / "robot", {})
        assert isinstance(res, Refusal) and res.reason == "wrong_rig"
        missing = run_job(Remote(srv.url), "nope", rig, CONFIG, tmp_path / "robot", {})
        assert isinstance(missing, Refusal) and missing.reason == "server"
    down = run_job(Remote(srv.url, timeout=0.5), "j4", rig, CONFIG, tmp_path / "robot", {})
    assert isinstance(down, Refusal) and down.reason == "unreachable"


def test_a_job_planned_by_other_code_is_refused_before_anything_moves(rig, tmp_path):
    from aris.version import describe
    server_dir = tmp_path / "server"
    server_dir.mkdir()
    other = dict(commit="e7741e5", dirty=False, digest="0" * 16)
    mine = dict(commit="515bbad", dirty=True, digest="1" * 16)
    Job.create(server_dir / "o", dict(_header(rig), code=other))
    old = _header(rig)
    old.pop("code")
    Job.create(server_dir / "u", old)
    Job.create(server_dir / "s", dict(_header(rig), code=mine))
    arm = Recording(SimArm("2L", rig.park_q("2L"), speed=math.inf))
    with Served(create_app(server_dir)) as srv:
        r = Remote(srv.url)
        res = run_job(r, "o", rig, CONFIG, tmp_path / "w", {"2L": arm}, code=mine)
        assert isinstance(res, Refusal) and res.reason == "wrong_code"
        assert res.detail == ("the job was planned by e7741e5, this PC runs 515bbad+local "
                              "changes: update both machines to the same commit")
        res = run_job(r, "u", rig, CONFIG, tmp_path / "w", {"2L": arm}, code=mine)
        assert res.reason == "wrong_code" and "planned by unknown" in res.detail
        # the same code passes this check (the job then waits for its phases: stopped here)
        assert describe(mine) == "515bbad+local changes"
    assert arm.calls == []



def test_a_guides_rows_go_on_the_job_too(rig, tmp_path):
    """The person-facing "instruction" rows and the "guide: ..." stages reach the job's
    events (with the arm), the driver's own rows still get them, and its say is put back."""
    import json
    from types import SimpleNamespace
    from aris_robot.runner import ArmLog, _guide_rows_on_the_job
    own = []
    drv = SimpleNamespace(say=lambda event, **f: own.append(event),
                          state=lambda: SimpleNamespace(q=rig.park_q("2L"), ok=True, flags=()))
    log = ArmLog(tmp_path / "events.jsonl", {"2L": drv})
    first = drv.say
    said = _guide_rows_on_the_job({"2L": drv}, log)
    drv.say("instruction", text="your turn")
    drv.say("guide: link back")
    drv.say("recover: controllers")
    assert own == ["instruction", "guide: link back", "recover: controllers"]
    rows = [json.loads(x) for x in (tmp_path / "events.jsonl").read_text().splitlines()]
    assert [(r["event"], r["arm"]) for r in rows] == [("instruction", "2L"),
                                                      ("guide: link back", "2L")]
    assert rows[0]["text"] == "your turn"
    assert said["2L"] is first
