"""The calibrate job (aris/server/calibrate.py) on simulated arms with a fake paper, and the
operator channel the operator PC pulls its work from (aris/server/operator.py)."""
from __future__ import annotations

import json
import math
import shutil
import sys
import threading
import time
from pathlib import Path

import numpy as np
import pytest
from fastapi.testclient import TestClient

from aris import cli
from aris.rig import Rig
from aris.server import open_station
from aris.server.calibrate import CalibSettings, grid_points, plan_calibrate
from aris.server.server import create_app
from aris.server.station import fake_paper
from aris.system.settings import Settings
from aris.types import Refusal

ROOT = Path(__file__).resolve().parents[1]
TWO = ROOT / "config" / "two_arms"
COARSE = Settings(grid_step=0.05)
PAPER = (-0.012, 0.0, 1.0)  # 12 mm low at the table centre, 1 degree about table y: under
                             # arm 31 the paper is 2 to 12 mm low (within the 20 mm extra depth)


class ClientHttp:
    def __init__(self, client):
        self.c = client

    def get(self, path):
        r = self.c.get(path)
        return r.status_code, r.json()

    def post(self, path, body=b""):
        r = self.c.post(path, content=body)
        return r.status_code, r.json()


def _config(tmp_path) -> Path:
    """A copy of config/two_arms with its own (empty) calibration folder."""
    d = tmp_path / "config"
    (d / "calibration").mkdir(parents=True)
    shutil.copy(TWO / "rig.json", d / "rig.json")
    return d


def _wait(c, jid, timeout=300.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        v = c.get(f"/jobs/{jid}").json()
        if v["state"] in ("done", "stopped", "failed") and v["report"] is not None:
            return v
        time.sleep(0.05)
    raise AssertionError(f"job {jid} did not end")


def test_the_fake_paper_and_the_touch_grid(tmp_path):
    rig = Rig.load(_config(tmp_path))
    n, d = fake_paper(rig, 31)
    nominal = rig.paper(31)
    assert np.allclose(n, nominal.normal) and d == pytest.approx(nominal.offset)
    n2, d2 = fake_paper(rig, 31, PAPER)
    assert math.degrees(math.acos(min(1.0, n2 @ n))) == pytest.approx(1.0)
    pts = grid_points(rig, 31, rig.drawing_area_m, CalibSettings())
    axis = rig.T_table_base(31)[:2, 3]
    assert 9 <= len(pts) <= 25 and np.all(np.linalg.norm(pts - axis, axis=1) <= 0.6 + 1e-9)
    assert np.all(np.abs(pts) <= 0.5 * rig.drawing_area_m)


@pytest.mark.slow
def test_calibrate_against_a_low_tilted_paper_then_draw(tmp_path, capsys):
    cfg = _config(tmp_path)
    st = open_station(cfg, speed=math.inf, uncalibrated=True, cache_dir=None,
                      jobs_dir=tmp_path / "jobs", workers=4, settings=COARSE, sim_paper=PAPER)
    assert not isinstance(st, Refusal), st
    true_n, true_d = fake_paper(st.rig, 31, PAPER)       # the simulated arm's truth
    c = TestClient(create_app(st))
    t = time.perf_counter()
    assert cli.main(["calibrate", "31", "--poll", "0.1"], http=ClientHttp(c)) == 0
    print(capsys.readouterr().out, f"calibrate: {time.perf_counter() - t:.1f} s wall")
    rep = c.get(f"/jobs/{c.get('/jobs').json()[-1]['id']}").json()["report"]
    assert rep["state"] == "done" and rep["contacts"] == rep["points"] >= 9
    assert rep["fit"]["passed"] and rep["fit"]["rms_mm"] < 0.01
    written = json.loads((cfg / "calibration" / "31.json").read_text())
    assert written["passed"] and written["arm_id"] == 31
    # the calibrated rig puts the paper where the fake one is, seen from arm 31
    paper = Rig.load(cfg).paper(31)
    tilt = math.degrees(math.acos(min(1.0, float(paper.normal @ true_n))))
    assert tilt < 0.1, tilt
    assert abs(paper.offset - true_d) < 0.0005, (paper.offset, true_d)
    r = c.get("/rig").json()
    assert r["arms"]["31"]["calibration"].startswith("applied")
    assert r["arms"]["71"]["calibration"] == "none"
    files = c.get("/calibration").json()["files"]
    assert [f["arm"] for f in files] == [31] and len(files[0]["digest"]) == 24
    assert c.get("/calibration/31").json()["arm_id"] == 31
    assert c.get("/calibration/71").status_code == 404
    # a drawing on the calibrated rig runs on the simulated arms
    drawing = dict(units="mm", frame="table", lines=[
        dict(id="a", points=[[-350, 150], [-200, 200]]), dict(id="b", points=[[250, -150],
                                                                              [380, -100]])])
    jid = c.post("/jobs", content=json.dumps(drawing).encode()).json()["id"]
    v = _wait(c, jid)
    assert v["state"] == "done" and v["report"]["passed"], v["why"]
    assert v["report"]["assumptions"]["calibration"]["31"].startswith("applied")


@pytest.mark.slow
def test_calibrate_starts_from_where_the_arm_stands(tmp_path):
    cfg = _config(tmp_path)
    st = open_station(cfg, uncalibrated=True, cache_dir=None, with_arms=False,
                      with_area=False)
    st.drawing_area = tuple(st.rig.drawing_area_m)
    rig = st.rig
    where = {a: rig.park_q(a) for a in rig.arm_ids}
    where[31] = where[31] + np.array([0.04, -0.03, 0.02, 0.03, -0.02, 0.03, 0.05])
    where[71] = where[71] + 0.03                       # a neighbour away from its park too
    plan = plan_calibrate(st, 31, where)
    assert plan.why == "", plan.why
    first = plan.steps[0].motions[0]
    assert np.max(np.abs(first.q_start - where[31])) < 1e-9 and first.kind == "free"
    assert plan.steps[-1].motions[-1].q_end == pytest.approx(rig.park_q(31), abs=1e-9)
    assert all(v is None or v.passed for s in plan.steps for v in s.verdicts)
    assert plan.phase.active == (31,) and 71 not in plan.phase.parked   # 71 as it stands


def test_missed_touches_are_skipped_and_named():
    from aris.server.calibrate import Plan, misses
    from aris.server.park import Step
    from aris.types import Motion, Trajectory
    t = Trajectory(np.array([0.0, 1.0]), np.zeros((2, 7)), np.zeros((2, 7)))
    steps = [Step(31, None, (Motion("free", t),)), Step(31, None, (Motion("touch", t),)),
             Step(31, None, (Motion("free", t),)), Step(31, None, (Motion("touch", t),))]
    plan = Plan(31, None, steps, np.array([[0.1, 0.2], [0.3, 0.4]]), [], 0.0)
    rows = [dict(event="motion done", arm=31, index=1), dict(event="contact", arm=31, index=1),
            dict(event="no contact", arm=31, index=3)]
    assert misses(plan, rows, 31) == [(0.3, 0.4)]


# --------------------------------------------------------------------------- the channel

ROBOT = ROOT / "robot"
for _p in (ROBOT, ROBOT / "tests"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))


def test_the_operator_pc_pulls_its_work(tmp_path):
    """The operator PC's resident process (robot/aris_robot/serve.py `Operator`, simulated
    arms) against the server: it takes a drawing job and a recover from the channel, fetches
    the calibration files, runs the job; its own rows land in operator.jsonl."""
    pytest.importorskip("aris_robot")
    from aris.execute.drivers.sim import SimArm
    from aris_robot.remote import Remote
    from aris_robot.serve import Operator
    from fake_server import Served
    cfg = _config(tmp_path)
    (cfg / "calibration" / "71.json").write_text(json.dumps(dict(arm_id=71, passed=False)))
    st = open_station(cfg, driver="robot", uncalibrated=True, cache_dir=None,
                      jobs_dir=tmp_path / "jobs", workers=4, settings=COARSE)
    assert not isinstance(st, Refusal), st
    rig = st.rig
    robot_cfg = tmp_path / "robot_config"            # the operator PC's own copy
    (robot_cfg / "calibration").mkdir(parents=True)
    shutil.copy(cfg / "rig.json", robot_cfg / "rig.json")
    (robot_cfg / "calibration" / "31.json").write_text("{}")    # stale: the server has none
    arms = {a: SimArm(a, rig.park_q(a), speed=math.inf) for a in rig.arm_ids}
    with Served(create_app(st)) as srv:
        http = cli.Http(srv.url)
        assert http.get("/operator/next?wait=0.2")[0] == 204
        assert http.get("/calibration")[1]["arms"] == [71]
        drawing = dict(units="mm", frame="table",
                       lines=[dict(id="a", points=[[-350, 150], [-200, 200]])])
        jid = http.post("/jobs", json.dumps(drawing).encode())[1]["id"]
        code, recov = http.post("/arms/71/recover")
        assert code == 200 and recov["queued"]["command"] == "recover"
        op = Operator(Remote(srv.url), robot_cfg, tmp_path / "robot", arms,
                      tmp_path / "robot_log", idle_s=0.2, wait_s=1.0)
        t = threading.Thread(target=op.serve, daemon=True)
        t.start()
        t0 = time.time()
        while True:
            v = http.get(f"/jobs/{jid}")[1]
            pending = http.get("/operator")[1]["pending"]
            if v["state"] in ("done", "failed", "stopped") and v["report"] and not pending:
                break
            assert time.time() - t0 < 120, "the job did not end"
            time.sleep(0.1)
        time.sleep(0.5)
        op.quit.set()
        t.join(timeout=10)
        seen = http.get("/operator")[1]
        arms_view = http.get("/arms")[1]
    assert v["state"] == "done", v["why"]
    events = [r["event"] for r in seen["last_rows"]]
    assert "operator started" in events and "recovered" in events and "run ended" in events
    assert seen["last_seen"] is not None
    assert sorted(p.name for p in (robot_cfg / "calibration").glob("*.json")) == ["71.json"]
    assert set(arms_view) == {"31", "71"} and all(x["at_park"] for x in arms_view.values())
    lines = (tmp_path / "jobs" / "operator.jsonl").read_text().splitlines()
    assert len(lines) >= 3 and all(json.loads(x)["source"] == "robot" for x in lines)
