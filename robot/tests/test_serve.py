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
from aris.types import JointPath, Motion, Phase
from aris_robot.remote import Remote
from aris_robot.serve import Operator, Rows, Stacks
from aris_robot.simarm import SimTouchArm
from aris_robot.touch import FakePaper, Kinematics
from fake_server import Served, create_app
from sim_touch import hover_q, touch_motion

CONFIG = Path(__file__).resolve().parents[2] / "config"


class Passed:
    passed, tightest, min_clearance, min_clearance_at = True, "test", 0.1, "test"

    def get(self, name):
        raise KeyError(name)


def _header(rig):
    calib = {a: (m.T_table_base, m.tip_hand, m.calibration) for a, m in rig.mounts.items()}
    return dict(rig_digest=digest(rig), calibration_digest=digest(calib), pen=rig.pen())


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
    T = np.eye(4)
    T[:3, :3] = [[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]]
    T[:3, 3] = [-0.305 + 0.002, -0.001, 0.970 + 0.004]          # slot 2L, 4 mm higher
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
        robots = {a: dict(robot=r, ip="192.168.50.1", serial_found=None, identity="unverified")
                  for a, r in (("2L", "fr3-31"), ("2R", "fr3-71"))}
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
    assert start["robots"]["2R"]["robot"] == "fr3-71"
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
    assert abs(float(kin.tip(np.array(contact["q"]))[0] @ paper.n) - paper.c) < 0.0004
    assert rows[events.index("recovered")]["arm"] == "2L"
    assert "not a mounted arm" in rows[events.index("recover refused")]["why"]
    assert all(set(r["where"]) == {"2L", "2R"} for r in rows if r["event"] == "where")
    assert (tmp_path / "log" / "rows.jsonl").read_text().count("\n") >= len(rows)


def test_a_job_the_operator_cannot_run_is_reported_not_fatal(tmp_path):
    jobs = tmp_path / "jobs"
    jobs.mkdir()
    Job.create(jobs / "x", dict(rig_digest="another rig"))
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
