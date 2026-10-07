"""Fixes from the first site day (2026-10-06): a single slot's mark job, positions that are no
readings, `aris arms`, the rest of a failed job; then (2026-10-07) pens lifted one arm per
phase, the paper map, SVG import, a one-arm config."""
from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import pytest
from fastapi.testclient import TestClient

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_server_mark import TWO, ClientHttp, _job, _station  # noqa: E402

from aris import cli  # noqa: E402
from aris.rig import Rig  # noqa: E402
from aris.server import open_station  # noqa: E402
from aris.server.mark import needs_solved_marks  # noqa: E402
from aris.server.mark_plan import slot_marks, tasks_for  # noqa: E402
from aris.server.server import create_app  # noqa: E402
from aris.server.station import STALE_S, Positions  # noqa: E402
from aris.system.settings import Settings  # noqa: E402
from aris.types import Refusal  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
SMALL = ROOT / "tests" / "data" / "server_small.json"


# --------------------------------------------------------------------------- 1. one slot


def test_a_single_slot_touches_the_marks_it_shares_and_needs_them_solved(tmp_path):
    rig = Rig.load(TWO)
    assert rig.marks_for(("2L",)) == ()                        # what crashed on site
    assert slot_marks(rig, "2L", ("2L",)) == ["A", "B"]
    assert [t.mark for t in tasks_for(rig, "2L", ("2L",))] == ["A"] * 6 + ["B"]
    why = needs_solved_marks(rig, ("2L",))
    assert why == "2L alone needs marks solved by an earlier run; run `aris mark` (2L, 2R) first"
    assert needs_solved_marks(rig, ("2L", "2R")) == ""


@pytest.mark.slow
def test_a_single_slot_after_a_full_run(tmp_path):
    st = _station(tmp_path, TWO)
    c = TestClient(create_app(st))
    r = c.post("/mark?slots=2L")
    assert r.status_code == 409 and r.json()["refused"] == "needs_solved_marks"
    assert cli.main(["mark", "--poll", "0.05"], http=ClientHttp(c)) == 0       # both arms
    assert all(st.rig.mark_state(n) == "solved" for n in ("A", "B"))
    assert cli.main(["mark", "2L", "--poll", "0.05"], http=ClientHttp(c)) == 0
    rep = _job(c)["report"]
    assert rep["slots"] == ["2L"] and rep["touches"] == 7
    assert {m["state"] for m in rep["marks"].values()} == {"known"}


# --------------------------------------------------------------------------- 2. no reading


def test_a_reading_that_is_no_reading_is_never_used():
    p, now = Positions(), time.time()
    p.from_row(dict(event="where", time=now, where={"2L": [0.1] * 7, "2R": None}), "j")
    assert np.allclose(p.known("2L")[0], 0.1) and p.known("2L")[1] == ""
    assert p.known("2R") == (None, "no joint states for 2R (FCI off?)")
    p.from_row(dict(event="motion done", arm="2L", time=now + 1, q=[0.0] * 7), "j")
    assert p.known("2L") == (None, "no joint states for 2L (FCI off?)")       # all zeros
    p.from_row(dict(event="motion done", arm="2L", time=now + 2, q=[0.2] * 7,
                    robot="fr3-0427"), "j")
    assert p.known("2L")[1] == "" and p.all()["2L"]["robot"] == "fr3-0427"
    q, why = p.known("2L", now=time.time() + STALE_S + 5)
    assert q is None and why.startswith("no joint states for 2L (FCI off?): the last reading")
    assert p.known("1L") == (None, "")                                          # never heard


def test_the_robot_server_refuses_to_plan_from_no_reading_and_aris_arms(tmp_path, capsys):
    st = open_station(TWO, driver="robot", uncalibrated=True, cache_dir=None,
                      jobs_dir=tmp_path / "jobs", workers=2, with_area=False)
    st.drawing_area, st.maps_area = tuple(st.rig.drawing_area_m), tuple(st.rig.drawing_area_m)
    c = TestClient(create_app(st))
    rig = st.rig
    st.positions.from_row(dict(event="where", time=time.time(),
                               where={"2L": list(rig.park_q("2L")), "2R": [0.0] * 7}), "op")
    r = c.post("/park").json()
    assert r["refused"] == "no_joint_states" and "no joint states for 2R" in r["detail"]
    r = c.post("/mark").json()
    assert r["refused"] == "no_joint_states"
    inside = dict(units="mm", frame="table", lines=[dict(id="a", points=[[-300, 100], [-200, 150]])])
    jid = c.post("/jobs", content=json.dumps(inside).encode()).json()["id"]
    t0 = time.time()
    while c.get(f"/jobs/{jid}").json()["state"] not in ("failed", "done"):
        assert time.time() - t0 < 30
        time.sleep(0.05)
    assert "no joint states for 2R (FCI off?)" in c.get(f"/jobs/{jid}").json()["why"]
    assert cli.main(["arms"], http=ClientHttp(c)) == 1
    out = capsys.readouterr().out
    assert "no joint states for 2R" in out and "FAIL: no reading for 2R" in out
    arms = c.get("/arms").json()["arms"]
    assert arms["2L"]["at_park"] is True and arms["2R"]["q"] is None
    assert arms["2R"]["reading"].startswith("no joint states")


def test_the_phase_end_check_waits_for_a_reading_instead_of_judging_zeros(tmp_path):
    from aris.execute import Coordinator, Job
    from aris.execute.drivers import ArmState

    class Blind:
        arm_id = "2R"

        def state(self):
            return ArmState(np.zeros(7), np.zeros(7), True, ("holding",))

    rig = Rig.load(TWO)
    job = Job.create(tmp_path / "j", {})
    coord = Coordinator(job, {"2R": Blind()}, TWO, rig, poll=0.01)
    t0 = time.time()
    assert coord._fresh_joints(wait=0.2) == "no joint states for 2R (FCI off?)"
    assert time.time() - t0 >= 0.2


def test_aris_arms_on_the_simulated_arms(tmp_path, capsys):
    st = open_station(TWO, uncalibrated=True, cache_dir=None, jobs_dir=tmp_path / "jobs",
                      workers=2, with_area=False)
    c = TestClient(create_app(st))
    assert cli.main(["arms"], http=ClientHttp(c)) == 0
    out = capsys.readouterr().out
    assert "2L" in out and "2R" in out and "yes" in out


# --------------------------------------------------------------------------- 4. rest of a failure


@pytest.mark.slow
def test_the_rest_of_a_failed_job(tmp_path):
    from aris.execute.drivers.sim import SimArm
    st = open_station(ROOT / "config", speed=math.inf, uncalibrated=True, cache_dir=None,
                      jobs_dir=tmp_path / "jobs", workers=4, settings=Settings(grid_step=0.05))
    rig = st.rig
    st.drivers["3R"] = SimArm("3R", rig.park_q("3R") + 0.05)       # away, not a phase-1 arm
    c = TestClient(create_app(st))
    jid = c.post("/jobs", content=SMALL.read_bytes()).json()["id"]
    t0 = time.time()
    while (v := c.get(f"/jobs/{jid}").json())["report"] is None:
        assert time.time() - t0 < 60
        time.sleep(0.05)
    assert v["state"] == "failed" and "park all arms first" in v["why"]
    r = c.post(f"/jobs?rest_of={jid}")
    assert r.status_code == 200, r.json()
    rid = r.json()["id"]
    lines = json.loads((st.jobs_dir / rid / "drawing.json").read_text())["lines"]
    assert sorted(x["id"] for x in lines) == ["under13#rest", "under71#rest"]
    c.post(f"/jobs/{rid}/stop")


# --------------------------------------------------------------------------- 2026-10-07: lifts


def _pen_on_paper(st, a):
    """A joint configuration of arm a with its pen tip on the paper (a calibrate touch's descent
    where it meets the nominal paper)."""
    from aris.server.calibrate import CalibSettings, plan_calibrate
    rig = st.rig
    plan = plan_calibrate(st, a, {b: rig.park_q(b) for b in rig.arm_ids},
                          CalibSettings(grid=3), min_points=1)
    touch = next(m for s in plan.steps for m in s.motions if m.kind == "touch")
    tips = rig.arm(a).tip(touch.traj.q)
    z = np.array([rig.to_table(a, t)[2] for t in tips]) - rig.paper_z
    return touch.traj.q[int(np.argmin(np.abs(z)))]


def _two_pens_down(tmp_path):
    from aris.server.steps import pen_down
    st = open_station(ROOT / "config", speed=math.inf, uncalibrated=True, cache_dir=None,
                      jobs_dir=tmp_path / "jobs", workers=2, with_area=False)
    st.drawing_area = st.maps_area = tuple(st.rig.drawing_area_m)
    where = {a: st.rig.park_q(a) for a in st.rig.arm_ids}
    for a in ("1L", "1R"):                               # row partners: never draw together
        where[a] = _pen_on_paper(st, a)
        assert pen_down(st.rig, a, where[a])
    return st, where


def test_row_partners_with_pens_down_lift_one_per_phase(tmp_path):
    from aris.server.park import plan_park
    st, where = _two_pens_down(tmp_path)
    steps = plan_park(st, where)
    assert all(not s.why or s.why == "already at its park" for s in steps), \
        [s.why for s in steps]
    moving = [s.phase.name for s in steps if s.motions]
    assert moving == ["lift pens 1L", "lift pens 1R", "park 1L", "park 1R"]
    assert [s.phase.active for s in steps if s.motions][:2] == [("1L",), ("1R",)]


def test_row_partners_with_pens_down_park_cleanly(tmp_path):
    from aris.execute.drivers.sim import SimArm
    st, where = _two_pens_down(tmp_path)
    rig = st.rig
    for a in ("1L", "1R"):
        st.drivers[a] = SimArm(a, where[a])
    c = TestClient(create_app(st))
    assert cli.main(["park", "--poll", "0.05"], http=ClientHttp(c)) == 0
    for a, d in st.drivers.items():
        assert np.max(np.abs(d.state().q - rig.park_q(a))) < 1e-9, a


# --------------------------------------------------------------------------- 2026-10-07: paper map


class _Surface:
    """A stand-in for aris.calib.paper.Surface: a 1.8 mm slope along x."""
    points = np.array([[-0.5, 0.0, -0.0018], [0.5, 0.0, 0.0018], [0.0, 0.3, 0.0]])
    date, paper_z = "2026-10-07", 0.0

    def z(self, x, y):
        return 0.0036 * np.asarray(x)


def _fake_paper_module(monkeypatch, written):
    import types
    import aris.calib
    mod = types.ModuleType("aris.calib.paper")

    class _Plane(_Surface):
        points = np.zeros((0, 3))
    mod.surface = lambda config_dir: (_Surface() if (Path(config_dir) / "calibration"
                                                     / "paper.json").exists() else _Plane())

    def write_paper(surface, config_dir):
        written.append(Path(config_dir))
        f = Path(config_dir) / "calibration" / "paper.json"
        f.parent.mkdir(parents=True, exist_ok=True)
        f.write_text("{}")
        return f
    mod.write_paper, mod.build_surface = write_paper, lambda config_dir: _Surface()
    monkeypatch.setitem(sys.modules, "aris.calib.paper", mod)
    monkeypatch.setattr(aris.calib, "paper", mod, raising=False)


def test_the_paper_map_reaches_the_planner_rig_and_report(tmp_path, monkeypatch, capsys):
    import shutil
    from aris.server import paper, pipeline
    cfg = tmp_path / "cfg"
    shutil.copytree(TWO, cfg)
    written = []
    _fake_paper_module(monkeypatch, written)
    assert paper.load(cfg) is None                        # the module, but no file yet
    assert paper.rebuild(cfg).endswith("paper.json") and written == [cfg]
    got, real = [], pipeline.system_plan

    def spy(*args, surface=None, **kw):                   # until the planner takes it
        got.append(surface)
        return real(*args, **kw)
    monkeypatch.setattr(pipeline, "system_plan", spy)
    st = open_station(cfg, speed=math.inf, uncalibrated=True, cache_dir=None,
                      jobs_dir=tmp_path / "jobs", workers=2, with_area=False)
    st.drawing_area = st.maps_area = tuple(st.rig.drawing_area_m)
    assert isinstance(st.surface, _Surface)
    c = TestClient(create_app(st))
    assert cli.main(["rig"], http=ClientHttp(c)) == 0
    assert "paper map 3 points, -1.80 to +1.80 mm about the nominal paper, from 2026-10-07" in capsys.readouterr().out
    small = tmp_path / "d.json"
    small.write_text(json.dumps(dict(units="mm", frame="table", lines=[
        dict(id="a", points=[[-300, 100], [-200, 150]])])))
    cli.main(["draw", str(small), "--poll", "0.05"], http=ClientHttp(c))
    out = capsys.readouterr().out
    assert got and isinstance(got[0], _Surface)
    rep = c.get(f"/jobs/{c.get('/jobs').json()[-1]['id']}").json()["report"]
    u = rep["paper_under_drawing"]
    assert u["range_mm"] > 0 and "paper        height map under the drawing" in out


@pytest.mark.slow
def test_a_plane_job_rebuilds_the_paper_map(tmp_path, monkeypatch):
    import shutil
    cfg = tmp_path / "cfg"
    shutil.copytree(TWO, cfg)
    written = []
    _fake_paper_module(monkeypatch, written)
    st = open_station(cfg, speed=math.inf, uncalibrated=True, cache_dir=None,
                      jobs_dir=tmp_path / "jobs", workers=2, with_area=False)
    st.drawing_area = st.maps_area = tuple(st.rig.drawing_area_m)
    c = TestClient(create_app(st))
    assert cli.main(["calibrate", "2L", "--poll", "0.05"], http=ClientHttp(c)) == 0
    rep = c.get(f"/jobs/{c.get('/jobs').json()[-1]['id']}").json()["report"]
    assert written == [cfg] and rep["written"]["paper_surface"].endswith("paper.json")
    assert rep["paper_surface"]["exists"] and rep["paper_surface"]["points"] == 3


# --------------------------------------------------------------------------- 2026-10-07: svg

SVG = """<svg xmlns="http://www.w3.org/2000/svg" width="100mm" height="50mm" viewBox="0 0 100 50">
  <g transform="translate(10,5) scale(2)">
    <path d="M 0 0 C 10 0 10 10 20 10" fill="none" stroke="black"/>
    <polyline points="0,15 5,20 10,15" fill="none" stroke="black"/>
  </g>
  <path d="M 60 40 Q 70 30 80 40" stroke="black" fill="none"/>
  <text x="0" y="0">not drawn</text>
</svg>"""


def test_an_svg_becomes_table_lines(tmp_path):
    from aris.server import drawing, svg
    f = tmp_path / "pic.svg"
    f.write_text(SVG)
    d = svg.to_drawing(f, 0.5, at=(0.1, -0.2))
    lines = drawing.from_dict(d)
    assert [x.id for x in lines] == ["svg1", "svg2", "svg3"]
    p = np.concatenate([x.points[:, :2] for x in lines])
    assert p[:, 0].max() - p[:, 0].min() == pytest.approx(0.5)            # --width
    centre = 0.5 * (p.min(axis=0) + p.max(axis=0))
    assert centre == pytest.approx((0.1, -0.2))                           # --at
    # the picture: x 10..80, y 5..45 user units (the group's transform applied) -> 70 wide
    k, mid = 0.5 / 70.0, (45, 25)
    cubic = lines[0].points[:, :2]
    assert len(cubic) > 10                                                 # flattened
    assert cubic[0] == pytest.approx((0.1 + (10 - 45) * k, -0.2 - (5 - 25) * k))
    assert cubic[-1] == pytest.approx((0.1 + (50 - 45) * k, -0.2 - (25 - 25) * k))
    # every chord within 0.5 mm of the curve: compare with a fine sampling of the Bézier
    t = np.linspace(0, 1, 2001)[:, None]
    P = [np.array(v, float) for v in ((10, 5), (30, 5), (30, 25), (50, 25))]
    curve = ((1 - t) ** 3 * P[0] + 3 * (1 - t) ** 2 * t * P[1] + 3 * (1 - t) * t ** 2 * P[2]
             + t ** 3 * P[3] - mid) * k * (1, -1) + (0.1, -0.2)
    a, b = cubic[:-1], cubic[1:]
    seg = b - a
    for c in curve:
        u = np.clip(np.einsum("ij,ij->i", c - a, seg) / np.einsum("ij,ij->i", seg, seg), 0, 1)
        assert np.min(np.linalg.norm(a + u[:, None] * seg - c, axis=1)) <= 0.5e-3 + 1e-9
    assert len(lines[1].points) == 3 and len(lines[2].points) > 4         # polyline, quad


def test_aris_import_and_draw_an_svg(tmp_path, capsys):
    f = tmp_path / "pic.svg"
    f.write_text(SVG)
    out = tmp_path / "pic.json"
    assert cli.main(["import", str(f), "--width", "0.3", "-o", str(out)]) == 0
    assert len(json.loads(out.read_text())["lines"]) == 3
    assert cli.main(["import", str(f), "-o", str(out)]) == 1               # no --width
    st = open_station(TWO, speed=math.inf, uncalibrated=True, cache_dir=None,
                      jobs_dir=tmp_path / "jobs", workers=2, with_area=False)
    st.drawing_area = st.maps_area = tuple(st.rig.drawing_area_m)
    c = TestClient(create_app(st))
    assert cli.main(["draw", str(f), "--width", "0.3", "--pen", "gel_g2"],
                    http=ClientHttp(c)) == 1
    assert "the pen in is" in capsys.readouterr().out
    cli.main(["draw", str(f), "--width", "0.3", "--at", "-0.3", "0.1", "--poll", "0.05"],
             http=ClientHttp(c))
    jid = c.get("/jobs").json()[-1]["id"]
    d = json.loads((st.jobs_dir / jid / "drawing.json").read_text())
    p = np.concatenate([np.asarray(x["points"]) for x in d["lines"]]) * 1e-3
    assert np.ptp(p[:, 0]) == pytest.approx(0.3)
    assert 0.5 * (p.min(axis=0) + p.max(axis=0)) == pytest.approx((-0.3, 0.1))


# --------------------------------------------------------------------------- one-arm config


@pytest.mark.slow
def test_mounted_rig_for_one_arm_gives_a_working_config(tmp_path):
    import subprocess
    out = tmp_path / "one_arm"
    r = subprocess.run([sys.executable, str(ROOT / "tools" / "mounted_rig.py"), "--arms", "1R",
                        "--out", str(out)], capture_output=True, text=True, timeout=600)
    assert r.returncode == 0, r.stderr
    st = open_station(out, uncalibrated=True, cache_dir=ROOT / "out" / "cache",
                      jobs_dir=tmp_path / "jobs", workers=4)
    assert hasattr(st, "rig") and st.area_problem == "", st
    assert min(st.drawing_area) > 0.3 and st.rig.arm_ids == ("1R",)


# --------------------------------------------------------------------------- code versions


def test_the_operator_pcs_code_against_the_servers(tmp_path, capsys):
    from aris.version import code_version
    st = open_station(TWO, driver="robot", uncalibrated=True, cache_dir=None,
                      jobs_dir=tmp_path / "jobs", workers=2, with_area=False)
    st.drawing_area = st.maps_area = tuple(st.rig.drawing_area_m)
    c = TestClient(create_app(st))
    assert c.get("/rig").json()["code"] == st.code == code_version()
    st.positions.from_row(dict(event="where", time=time.time(), where={
        a: list(st.rig.park_q(a)) for a in st.rig.arm_ids}), "op")
    # the operator PC polls with the same code: `aris arms` says so, a job starts and both
    # versions are in its header and report
    assert c.get("/operator/next", params=dict(wait=0, code=json.dumps(st.code))).status_code \
        in (200, 204)
    assert cli.main(["arms"], http=ClientHttp(c)) == 0
    assert "operator PC code: same (" in capsys.readouterr().out
    r = c.post("/park")
    assert r.status_code == 200, r.json()
    jid = r.json()["id"]
    head = json.loads((st.jobs_dir / jid / "job.json").read_text())
    assert head["code"] == st.code and head["operator_pc_code"] == st.code
    c.post(f"/jobs/{jid}/stop")
    t0 = time.time()
    while (v := c.get(f"/jobs/{jid}").json())["report"] is None:
        assert time.time() - t0 < 60
        time.sleep(0.05)
    assert v["report"]["code"] == dict(server=st.code, operator_pc=st.code)
    # an acknowledgement with other code: `aris arms` FAILs and every job is refused
    other = dict(commit="515bbad", dirty=True, digest="0" * 24)
    cmd = st.operator.push("report")
    c.post("/operator/ack", json=dict(id=cmd["id"], code=other))
    assert cli.main(["arms"], http=ClientHttp(c)) == 1
    out = capsys.readouterr().out
    assert "operator PC code: DIFFERENT — server" in out and "515bbad+local changes" in out
    assert "update both machines to the same commit" in out
    r = c.post("/park").json()
    assert r["refused"] == "wrong_code" and "DIFFERENT" in r["detail"]
