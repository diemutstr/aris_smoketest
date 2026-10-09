"""The resident process against the stand-in server, with simulated arms (touch included):
commands pulled and acknowledged, calibration fetched before a job, jobs run, a touch's
contact reported, an arm recovered, positions reported while idle; and the stack keeper."""
import json
import math
import shutil
import sys
import threading
import time
from pathlib import Path

import numpy as np
import pytest

from aris.execute import Job
from aris.execute.queue import digest
from aris.kernel.retime import retime
from aris.rig import Rig
from conftest import CODE
from aris.types import JointPath, Motion, Phase
from aris_robot.remote import Remote
from aris_robot.serve import Operator, Rows, Stacks
from aris_robot.simarm import SimTouchArm
from aris_robot.touch import FakePaper, Kinematics
from fake_server import Served, create_app
from sim_touch import hover_q, touch_motion

CONFIG = Path(__file__).resolve().parents[2] / "config"


def _config(tmp_path) -> Path:
    """A copy of the repository's config folder for an Operator.  An Operator mirrors the
    server's calibration files into its config folder and takes the others away; given the
    real folder, a test run on the planning laptop deleted the four calibrations measured
    that day (2026-10-09).  No test hands an Operator the real folder."""
    d = tmp_path / "config"
    if not d.exists():
        d.mkdir()
        for f in ("rig.json", "pens.json"):
            if (CONFIG / f).exists():
                shutil.copy(CONFIG / f, d / f)
        if (CONFIG / "calibration").is_dir():
            shutil.copytree(CONFIG / "calibration", d / "calibration")
    return d


class Passed:
    passed, tightest, min_clearance, min_clearance_at = True, "test", 0.1, "test"

    def get(self, name):
        raise KeyError(name)


def _header(rig):
    calib = {a: (m.T_table_base, m.tip_hand, m.calibration) for a, m in rig.mounts.items()}
    return dict(rig_digest=digest(rig), calibration_digest=digest(calib), pen=rig.pen(),
                code=CODE)


def _free(rig, a, q0, q1):
    return Motion("free", retime(JointPath(np.array([q0, q1])), rig.arm(a).limits, rig.rules()))


def _write_job(rig, job, phase_name, arm, motions):
    job.add_phase(Phase(phase_name, (arm,), tuple(b for b in rig.arm_ids if b != arm), ()))
    q = job.queue(phase_name, arm)
    for m in motions:
        assert isinstance(q.append(m, Passed()), int)
    q.close()
    job.end_phases()


def _calibration_file():
    T = np.array(Rig.load(CONFIG).T_table_base("2L"), float)    # slot 2L as the rig has it,
    T[:3, 3] += [0.002, -0.001, 0.004]                           # 4 mm higher
    return dict(slot="2L", base=dict(date="2026-10-02", passed=True, T_table_base=T.tolist(),
                                     method="test"))


def _wait_for(pred, timeout=20.0):
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        if pred():
            return True
        time.sleep(0.02)
    return False


def test_serve_takes_every_command_from_the_server(tmp_path):
    # the server's config: slot 2L calibrated.  The operator PC starts without that file and
    # with a stale one for slot 2R that the server does not have.
    server_cfg, op_cfg = tmp_path / "server_config", tmp_path / "op_config"
    for d in (server_cfg, op_cfg):
        (d / "calibration").mkdir(parents=True)
        shutil.copy(CONFIG / "rig.json", d / "rig.json")
    (server_cfg / "calibration" / "2L.json").write_text(json.dumps(_calibration_file()))
    (op_cfg / "calibration" / "2R.json").write_text(json.dumps(dict(slot="2R", base=dict(passed=False))))
    rig = Rig.load(server_cfg)
    assert rig.calibration_status("2L")["base"].startswith("applied")

    jobs = tmp_path / "jobs"
    jobs.mkdir()
    p31 = rig.park_q("2L")
    j1 = Job.create(jobs / "j1", _header(rig))
    _write_job(rig, j1, "phase 1", "2L", [_free(rig, "2L", p31, p31 + 0.05),
                                        _free(rig, "2L", p31 + 0.05, p31)])
    hover = hover_q(rig, "2L")
    cal = Job.create(jobs / "cal", _header(rig))
    _write_job(rig, cal, "touch arm 31", "2L", [_free(rig, "2L", p31, hover),
                                              touch_motion(rig, "2L", hover),
                                              _free(rig, "2L", hover, p31)])

    app = create_app(jobs)
    app.state.calibration = {"2L": _calibration_file()}
    nominal = Rig.load(CONFIG)
    drivers = {a: SimTouchArm(nominal, a, nominal.park_q(a), speed=math.inf, paper_m=0.003)
               for a in ("2L", "2R")}
    with Served(app) as srv:
        from aris_robot import site as site_mod             # who hangs where: the site table
        table = site_mod.load(Path(__file__).resolve().parents[1] / "site.json")
        robots = {a: dict(robot=table.arm(a).robot, ip="192.168.50.1", serial_found=None,
                          identity="unverified") for a in ("2L", "2R")}
        op = Operator(Remote(srv.url), op_cfg, tmp_path / "work", drivers, tmp_path / "log",
                      idle_s=0.2, wait_s=0.5, robots=robots)
        t = threading.Thread(target=op.serve, daemon=True)
        t.start()
        app.state.commands += [dict(id="c1", command="report"),
                               dict(id="c2", command="run", job="j1"),
                               dict(id="c3", command="run", job="cal"),
                               dict(id="c4", command="recover", arm="2L"),
                               dict(id="c5", command="recover", arm="1L")]
        assert _wait_for(lambda: any(r["event"] == "recover refused" for r in app.state.rows))
        assert _wait_for(lambda: sum(r["event"] == "where" for r in app.state.rows) >= 2)
        op.quit.set()
        t.join(timeout=5)
    rows = app.state.rows
    events = [r["event"] for r in rows]
    assert app.state.acked == ["c1", "c2", "c3", "c4", "c5"]
    start = rows[0]
    assert start["event"] == "operator started"
    assert start["code"] == CODE and start["code_text"]           # which code this PC runs
    assert start["robots"]["2R"]["robot"] == robots["2R"]["robot"]
    assert start["robots"]["2R"]["identity"] == "unverified"
    assert set(start["where"]) == {"2L", "2R"} and np.allclose(start["where"]["2L"], p31)
    rep = rows[events.index("report")]
    assert all(len(rep["arms"][a]["q"]) == 7 and rep["arms"][a]["ok"] for a in ("2L", "2R"))
    # calibration: the server's file in place, the stale one gone, the job ran (digests agree)
    assert json.loads((op_cfg / "calibration" / "2L.json").read_text()) == _calibration_file()
    assert not (op_cfg / "calibration" / "2R.json").exists()
    ends = [r for r in rows if r["event"] == "run ended"]
    assert [(r["job"], r["status"]) for r in ends] == [("j1", "done"), ("cal", "done")]
    assert np.allclose(ends[-1]["where"]["2L"], p31, atol=1e-9)
    # the touch: the contact row carries the joints, where the (calibrated) fake paper is
    cal_rows = app.state.received["cal"]
    contact = next(r for r in cal_rows if r["event"] == "contact")
    kin = Kinematics.of(rig, "2L")
    paper = FakePaper(kin, rig.paper("2L"), 0.003)
    assert abs(float(kin.tip(np.array(contact["q"]))[0] @ paper.n) - paper.c) < 0.0003
    assert rows[events.index("recovered")]["arm"] == "2L"
    assert "not a mounted arm" in rows[events.index("recover refused")]["why"]
    assert all(set(r["where"]) == {"2L", "2R"} for r in rows if r["event"] == "where")
    assert (tmp_path / "log" / "rows.jsonl").read_text().count("\n") >= len(rows)


def test_a_job_the_operator_cannot_run_is_reported_not_fatal(tmp_path):
    jobs = tmp_path / "jobs"
    jobs.mkdir()
    Job.create(jobs / "x", dict(rig_digest="another rig", code=CODE))
    app = create_app(jobs)
    rig = Rig.load(CONFIG)
    drivers = {"2L": SimTouchArm(rig, "2L", rig.park_q("2L"), speed=math.inf)}
    cfg = tmp_path / "cfg"
    (cfg / "calibration").mkdir(parents=True)
    shutil.copy(CONFIG / "rig.json", cfg / "rig.json")
    with Served(app) as srv:
        op = Operator(Remote(srv.url), cfg, tmp_path / "work", drivers, tmp_path / "log",
                      idle_s=5.0, wait_s=0.3)
        t = threading.Thread(target=op.serve, daemon=True)
        t.start()
        app.state.commands += [dict(id="a", command="run", job="x"),
                               dict(id="b", command="report")]
        assert _wait_for(lambda: any(r["event"] == "report" for r in app.state.rows))
        op.quit.set()
        t.join(timeout=5)
    refused = next(r for r in app.state.rows if r["event"] == "run refused")
    assert refused["reason"] == "wrong_rig" and "2L" in refused["where"]


class _NoServer:
    def post_rows(self, rows):
        return {}


def test_the_stack_keeper_restarts_with_a_growing_pause_and_stops_cleanly(tmp_path):
    rows = Rows(_NoServer(), tmp_path)
    said = []
    rows.say = lambda event, **f: said.append(dict(event=event, **f))
    dying = [sys.executable, "-c", "import sys; sys.exit(3)"]
    lasting = [sys.executable, "-c", "import time; time.sleep(100)"]
    st = Stacks({"2L": dying, "2R": lasting}, rows, tmp_path, first_pause=0.05).start()
    assert _wait_for(lambda: st.state["2L"]["starts"] >= 4, 10.0)
    t0 = time.monotonic()
    st.stop(grace=5.0)
    assert time.monotonic() - t0 < 3.0                         # SIGINT ends the long one
    died = [r for r in said if r["event"] == "stack died" and r["arm"] == "2L"]
    assert all(r["exit_code"] == 3 for r in died)
    pauses = [r["restart_in_s"] for r in died[:3]]
    assert pauses == pytest.approx([0.05, 0.1, 0.2])
    assert st.state["2R"]["starts"] == 1 and not st.state["2R"]["running"]
    assert not any(r["event"] == "stack died" and r["arm"] == "2R" for r in said)


def test_a_paused_stack_stays_down_until_resumed(tmp_path):
    rows = Rows(_NoServer(), tmp_path)
    said = []
    rows.say = lambda event, **f: said.append(dict(event=event, **f))
    lasting = [sys.executable, "-c", "import time; time.sleep(100)"]
    st = Stacks({"2R": lasting, "3R": lasting}, rows, tmp_path, first_pause=0.05).start()
    assert _wait_for(lambda: st.state["2R"]["running"] and st.state["3R"]["running"], 10.0)
    t0 = time.monotonic()
    assert st.pause("2R") == ""                                # returns once it has exited
    assert time.monotonic() - t0 < 3.0 and not st.state["2R"]["running"]
    time.sleep(0.5)
    assert st.state["2R"]["starts"] == 1 and st.state["3R"]["running"]   # not restarted
    assert not any(r["event"] == "stack died" for r in said)
    st.resume("2R")
    assert _wait_for(lambda: st.state["2R"]["running"], 5.0)
    assert st.state["2R"]["starts"] == 2
    st.stop(grace=5.0)


def test_each_stack_is_pinned_to_its_core(tmp_path):
    from aris_robot.serve import launch_commands
    from aris_robot import site as site_mod
    site = site_mod.load(Path(__file__).resolve().parents[1] / "site.json")
    f = tmp_path / "arm_2R.json"
    f.write_text(json.dumps(dict(arm="2R")))
    cmd = launch_commands([f], site)["2R"]
    assert site.arm("2R").rt_core == 17 and site.arm("2L").rt_core == 16
    assert cmd[:3] == ["taskset", "-c", "17"]                 # no chrt: it froze the PC
    assert cmd[3:6] == ["ros2", "launch", "aris_bringup"]
    assert launch_commands([f])["2R"][0] == "ros2"


class _Faulty:
    """An arm with a link-drop fault until recovered; counts the recoveries."""

    def __init__(self, q, fault="fault: communication_constraints_violation"):
        self.q, self.fault, self.recovers = np.array(q, float), fault, 0

    def state(self):
        from aris.execute.drivers import ArmState
        return ArmState(self.q, np.zeros(7), not self.fault,
                        (self.fault,) if self.fault else ("holding",))

    def recover(self):
        from aris.execute.drivers import Result
        self.recovers += 1
        return Result.ok(self.q)


class _Down:
    def state(self):
        from aris.execute.drivers import ArmState
        return ArmState(np.full(7, np.nan), np.zeros(7), False, ("no joint states",))


def test_link_drops_recover_by_themselves_once_per_window_and_where_is_never_zeros(tmp_path):
    rows = Rows(_NoServer(), tmp_path)
    said = []
    rows.say = lambda event, **f: said.append(dict(event=event, **f))
    drop, other = _Faulty(np.zeros(7) + 0.1), _Faulty(np.zeros(7) + 0.2, "fault: joint_reflex")
    op = Operator(_NoServer(), _config(tmp_path), tmp_path / "w", {"2L": drop, "2R": other, "1L": _Down()},
                  tmp_path / "log", rows=rows)
    op.auto_recover = dict(on=True, every_s=120, patterns=["communication_constraints_violation"])
    op._auto_recover()
    op._auto_recover()                                  # inside the window: not again
    assert drop.recovers == 1 and other.recovers == 0  # other faults wait for a person
    assert [r["event"] for r in said] == ["auto recovered"] and said[0]["arm"] == "2L"
    w = op.where_fields()
    assert w["where"]["1L"] is None and w["where_missing"]["1L"] == "no joint states"
    assert w["where"]["2L"] == pytest.approx([0.1] * 7)
    assert json.dumps(w, allow_nan=False)


class _Restartable:
    def __init__(self):
        self.calls, self.state = [], {}

    def restart(self, arm):
        self.calls.append(("restart", arm))
        return ""

    def pause(self, arm, grace=15.0):
        return ""

    def resume(self, arm):
        pass


def _park_job(rig, jobs, jid, slot, off):
    from aris.execute import Job
    from aris.kernel.retime import retime
    from aris.types import JointPath, Motion, Phase
    from test_runner import Passed, _header
    job = Job.create(jobs / jid, dict(_header(rig), kind="park"))
    phase = Phase(f"park {slot}", (slot,), tuple(b for b in rig.arm_ids if b != slot), ())
    job.add_phase(phase)
    q = job.queue(phase.name, slot)
    traj = retime(JointPath(np.array([rig.park_q(slot) + off, rig.park_q(slot)])),
                  rig.arm(slot).limits, rig.rules())
    assert q.append(Motion("free", traj), Passed()) == 0
    q.close()
    job.end_phases()


def _serve_until(app, op, event, timeout=15.0):
    t = threading.Thread(target=op.serve, daemon=True)
    t.start()
    ok = _wait_for(lambda: any(r["event"] == event for r in app.state.rows), timeout)
    op.quit.set()
    t.join(timeout=5)
    return ok


def test_a_park_after_a_stack_restart_runs(tmp_path):
    from aris.rig import Rig
    from aris_robot.remote import Remote
    from aris_robot.simarm import SimTouchArm
    from fake_server import Served, create_app
    rig = Rig.load(CONFIG)
    jobs = tmp_path / "jobs"
    jobs.mkdir()
    off = np.array([0, 0, 0, 0, 0, 0, 0.05])
    _park_job(rig, jobs, "p1", "2L", off)
    stacks = _Restartable()
    stacks.restart("2L")                                   # the stack came back
    drivers = {"2L": SimTouchArm(rig, "2L", rig.park_q("2L") + off, speed=math.inf)}
    app = create_app(jobs)
    app.state.commands.append(dict(id="c", command="run", job="p1"))
    with Served(app) as srv:
        op = Operator(Remote(srv.url), _config(tmp_path), tmp_path / "w", drivers, tmp_path / "log",
                      stacks=stacks, idle_s=5.0, wait_s=0.3)
        t0 = time.monotonic()
        assert _serve_until(app, op, "run ended")
    ended = next(r for r in app.state.rows if r["event"] == "run ended")
    assert ended["status"] == "done", ended["why"]
    assert time.monotonic() - t0 < 10.0


def test_a_job_that_cannot_start_fails_with_a_row_instead_of_hanging(tmp_path):
    """A call into a restarted stack that never answers (here: the collision thresholds)
    must not hang the runner: the job fails, named, and never starts late."""
    from aris.rig import Rig
    from aris_robot.remote import Remote
    from aris_robot.simarm import SimTouchArm
    from fake_server import Served, create_app
    rig = Rig.load(CONFIG)
    jobs = tmp_path / "jobs"
    jobs.mkdir()
    off = np.array([0, 0, 0, 0, 0, 0, 0.05])
    _park_job(rig, jobs, "p2", "2L", off)

    class Stuck(SimTouchArm):
        moves = 0

        def set_collision(self, which):
            if which == "job":
                time.sleep(2.0)                             # never answers in time
            return ""

        def move(self, traj):
            Stuck.moves += 1
            return super().move(traj)

    drivers = {"2L": Stuck(rig, "2L", rig.park_q("2L") + off, speed=math.inf)}
    app = create_app(jobs)
    app.state.commands.append(dict(id="c", command="run", job="p2"))
    with Served(app) as srv:
        op = Operator(Remote(srv.url), _config(tmp_path), tmp_path / "w", drivers, tmp_path / "log",
                      idle_s=5.0, wait_s=0.3)
        op.start_timeout_s = 0.5
        assert _serve_until(app, op, "run failed", timeout=10.0)
        time.sleep(2.5)                                     # the stuck call returns late
    failed = next(r for r in app.state.rows if r["event"] == "run failed")
    assert failed["step"] == "collision thresholds" and "not started" in failed["why"]
    last = app.state.received["p2"][-1]
    assert last["event"] == "runner finished" and last["status"] == "failed"
    assert Stuck.moves == 0 and not (tmp_path / "w" / "p2" / "events.jsonl").exists()


def test_serve_restarts_a_stack_that_never_reads(tmp_path):
    """A stack started while FCI was off reads an error and never joint states again.  Once
    FCI is on, serve restarts it by itself after the stale time, and the joints are fresh;
    while FCI stays off it does not restart more often than the restart period."""
    from aris.execute.drivers import ArmState

    class Arm:
        def __init__(self):
            self.fresh = False

        def state(self):
            if not self.fresh:
                return ArmState(np.full(7, np.nan), np.zeros(7), False, ("no joint states",))
            return ArmState(np.zeros(7) + 0.1, np.zeros(7), True, ("holding",))

    arm, fci = Arm(), {"on": False}

    class Stacks:
        calls = []

        def restart(self, a):
            Stacks.calls.append(a)
            arm.fresh = fci["on"]               # a new stack reads only once FCI is on
            return ""

    rows = Rows(_NoServer(), tmp_path)
    said = []
    rows.say = lambda event, **f: said.append(dict(event=event, **f))
    op = Operator(_NoServer(), _config(tmp_path), tmp_path / "w", {"2L": arm}, tmp_path / "log",
                  stacks=Stacks(), rows=rows)
    op.stale_restart_s, op.restart_every_s = 0.2, 0.5
    op._restart_stale()                                  # stale only just now: nothing
    time.sleep(0.25)
    op._restart_stale()                                  # FCI still off: restarted, no use
    op._restart_stale()                                  # within the period: not again
    assert Stacks.calls == ["2L"] and not arm.fresh
    fci["on"] = True                                     # the person switched FCI on
    time.sleep(0.55)
    op._restart_stale()
    assert Stacks.calls == ["2L", "2L"] and arm.fresh
    texts = [r["text"] for r in said if r["event"] == "restarting the stack"]
    assert texts[0].startswith("restarting the stack of 2L: no joint states for 0 s")
    assert texts[0].endswith("(FCI off? Desk: unlock, activate FCI)")
    op._restart_stale()                                  # fresh now: nothing more
    assert Stacks.calls == ["2L", "2L"]



def test_a_refused_job_is_told_on_the_job_before_serve_moves_on(tmp_path):
    from aris.execute import Job
    from aris.rig import Rig
    from aris_robot.remote import Remote
    from aris_robot.simarm import SimTouchArm
    from fake_server import Served, create_app
    rig = Rig.load(CONFIG)
    jobs = tmp_path / "jobs"
    jobs.mkdir()
    Job.create(jobs / "wc", dict(_header(rig), code=dict(commit="e7741e5", digest="0" * 16)))
    app = create_app(jobs)
    app.state.commands += [dict(id="a", command="run", job="wc"),
                           dict(id="b", command="report")]
    with Served(app) as srv:
        op = Operator(Remote(srv.url), _config(tmp_path), tmp_path / "w",
                      {"2L": SimTouchArm(rig, "2L", rig.park_q("2L"), speed=math.inf)},
                      tmp_path / "log", idle_s=5.0, wait_s=0.3)
        seen = []
        op.rows.say_orig = op.rows.say

        def say(event, **f):                # what the job's log holds when serve says it
            if event == "run refused":
                seen.append(list(app.state.received.get("wc", [])))
            return op.rows.say_orig(event, **f)
        op.rows.say = say
        assert _serve_until(app, op, "report")
    job_rows = app.state.received["wc"]
    assert job_rows[0]["event"] == "runner finished" and job_rows[0]["status"] == "refused"
    assert job_rows[0]["reason"] == "wrong_code" and "e7741e5" in job_rows[0]["why"]
    assert seen and seen[0] == job_rows                 # on the job before serve moved on
    refused = next(r for r in app.state.rows if r["event"] == "run refused")
    assert refused["on_job"] is True


def test_where_rows_keep_flowing_while_a_job_runs(tmp_path):
    """The positions come from the arms, not from a job: a busy serve still says where."""
    rows = Rows(_NoServer(), tmp_path)
    said = []
    rows.say = lambda event, **f: said.append(dict(event=event, **f))
    arm = _Faulty(np.zeros(7) + 0.3, fault="")
    op = Operator(_NoServer(), _config(tmp_path), tmp_path / "w", {"2L": arm}, tmp_path / "log",
                  rows=rows, idle_s=0.05)
    op.busy.set()                                        # a job is running
    import threading as th
    t = th.Thread(target=op._idle, daemon=True)
    t.start()
    assert _wait_for(lambda: sum(r["event"] == "where" for r in said) >= 3, 5.0)
    op.quit.set()
    t.join(timeout=2)
    assert all(r["where"]["2L"] == pytest.approx([0.3] * 7) for r in said if r["event"] == "where")


def test_stacks_start_one_at_a_time(tmp_path):
    """The next arm's stack starts only once the previous one has fresh joint states (or
    after the timeout): never all at once."""
    rows = Rows(_NoServer(), tmp_path)
    said = []
    rows.say = lambda event, **f: said.append(dict(event=event, **f))
    lasting = [sys.executable, "-c", "import time; time.sleep(100)"]
    fresh = set()
    st = Stacks({"1L": lasting, "1R": lasting, "2L": lasting}, rows, tmp_path,
                ready=lambda arm: arm in fresh, ready_timeout=1.0).start()
    assert _wait_for(lambda: st.state["1L"]["running"], 10.0)
    time.sleep(0.4)
    assert not st.state["1R"]["running"]                       # 1L not fresh yet
    fresh.add("1L")
    assert _wait_for(lambda: st.state["1R"]["running"], 5.0)
    assert not st.state["2L"]["running"]
    # 1R never comes up fresh: the next starts after the timeout
    assert _wait_for(lambda: st.state["2L"]["running"], 5.0)
    events = [(r["event"], r["arm"]) for r in said if r["event"].startswith("stack ")]
    assert events.index(("stack ready", "1L")) < events.index(("stack started", "1R"))
    assert ("stack not ready, starting the next", "1R") in events
    st.stop(grace=5.0)


def test_six_stacks_start_one_after_the_other_and_all_report(tmp_path):
    """All six arms mounted (2026-10-09): the site's six stacks start in turn, each once the
    one before has fresh joint states, and all six report ready or running."""
    from aris_robot import site as site_mod
    site = site_mod.load(Path(__file__).resolve().parents[1] / "site.json")
    slots = [a.id for a in site.arms if a.mounted]
    assert sorted(slots) == ["1L", "1R", "2L", "2R", "3L", "3R"]
    assert sorted(a.rt_core for a in site.arms) == [16, 17, 18, 19, 28, 29]
    rows = Rows(_NoServer(), tmp_path)
    said = []
    rows.say = lambda event, **f: said.append(dict(event=event, **f))
    lasting = [sys.executable, "-c", "import time; time.sleep(100)"]
    started = {}

    def fresh(arm):                                   # fresh 0.2 s after its stack started
        if st.state[arm]["running"]:
            started.setdefault(arm, time.monotonic())
        return arm in started and time.monotonic() - started[arm] > 0.2
    st = Stacks({a: lasting for a in slots}, rows, tmp_path, ready=fresh, ready_timeout=5.0)
    st.start()
    assert _wait_for(lambda: all(st.state[a]["running"] for a in slots), 20.0)
    order = [r["arm"] for r in said if r["event"] == "stack started"]
    assert order == slots                              # one after the other, in turn
    ready = [r["arm"] for r in said if r["event"] == "stack ready"]
    assert ready == slots[:-1]                         # each waited for the one before
    st.stop(grace=5.0)


def test_a_core_outside_the_isolated_set_is_refused(tmp_path):
    from aris_robot.serve import isolated_cores, launch_commands
    from aris_robot import site as site_mod
    site = site_mod.load(Path(__file__).resolve().parents[1] / "site.json")
    iso = tmp_path / "isolated"
    iso.write_text("8-19,28-39\n")
    assert isolated_cores(iso) == set(range(8, 20)) | set(range(28, 40))
    files = []
    for a in ("2L", "3R"):
        f = tmp_path / f"{a}.json"
        f.write_text(json.dumps({"arm": a}))
        files.append(f)
    cmds = launch_commands(files, site, isolated=iso)               # 16 and 29: isolated
    assert cmds["3R"][:3] == ["taskset", "-c", "29"]
    iso.write_text("8-19\n")
    with pytest.raises(ValueError, match="rt_core 29 is not an isolated core.*8-19"):
        launch_commands(files, site, isolated=iso)
    iso.write_text("")                                              # nothing to check
    assert launch_commands(files, site, isolated=iso)["3R"][2] == "29"
