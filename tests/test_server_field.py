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


from aris import cli  # noqa: E402
from aris.rig import Rig  # noqa: E402
from aris.server import open_station  # noqa: E402
from aris.server.server import create_app  # noqa: E402
from aris.server.station import STALE_S, Positions  # noqa: E402
from aris.system.settings import Settings  # noqa: E402
from aris.types import Refusal  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
TWO = ROOT / "config" / "two_arms"


class ClientHttp:
    """The server's endpoints for `aris.cli.main` through a TestClient."""

    def __init__(self, client):
        self.c = client

    def get(self, path):
        r = self.c.get(path)
        return r.status_code, r.json()

    def post(self, path, body=b""):
        r = self.c.post(path, content=body)
        return r.status_code, r.json()
SMALL = ROOT / "tests" / "data" / "server_small.json"


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
    r = c.post("/crosses").json()
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


# --------------------------------------------------------------------------- grip, uploads, gui


def _wait_report(c, jid, limit=60):
    t0 = time.time()
    while (v := c.get(f"/jobs/{jid}").json())["report"] is None:
        assert time.time() - t0 < limit
        time.sleep(0.05)
    return v


def test_grip_on_the_simulated_arms(tmp_path, capsys):
    st = open_station(TWO, speed=math.inf, uncalibrated=True, cache_dir=None,
                      jobs_dir=tmp_path / "jobs", workers=2, with_area=False)
    c = TestClient(create_app(st))
    assert cli.main(["grip", "2L", "close", "--width", "0.02", "--poll", "0.02"],
                    http=ClientHttp(c)) == 0
    assert "gripper      2L close: width 80.0 mm -> 20.0 mm, grasped" in capsys.readouterr().out
    jid = c.get("/jobs").json()[-1]["id"]
    head = json.loads((st.jobs_dir / jid / "job.json").read_text())
    assert head["kind"] == "grip" and head["slot"] == "2L" and head["verb"] == "close"
    assert head["params"] == {"width_m": 0.02}
    rep = c.get(f"/jobs/{jid}/report").json()
    assert rep["width_before_m"] == 0.08 and rep["width_after_m"] == 0.02
    assert c.post("/grip/2L", json=dict(verb="squeeze")).json()["refused"] == "verb"
    assert c.post("/grip/9X", json=dict(verb="open")).json()["refused"] == "no_slot"
    assert c.get("/jobs/nope/report").status_code == 404


def test_grip_with_the_robot_waits_for_the_operator_pc(tmp_path):
    st = open_station(TWO, driver="robot", uncalibrated=True, cache_dir=None,
                      jobs_dir=tmp_path / "jobs", workers=2, with_area=False)
    c = TestClient(create_app(st))
    r = c.post("/grip/2R", json=dict(verb="open")).json()
    assert r["refused"] == "no_joint_states"                   # never reported: stack down?
    st.positions.from_row(dict(event="where", time=time.time(), where={
        a: list(st.rig.park_q(a)) for a in st.rig.arm_ids}), "op")
    jid = c.post("/grip/2R", json=dict(verb="open")).json()["id"]
    assert c.get("/operator").json()["pending"][-1]["kind"] == "grip"
    phases = [json.loads(x) for x in c.get(f"/jobs/{jid}/phases").text.splitlines() if x]
    assert all(x.get("end") for x in phases)                   # no phases, only the end
    assert c.post("/grip/2L", json=dict(verb="open")).json()["refused"] == "busy"
    c.post(f"/jobs/{jid}/events", json=dict(rows=[
        dict(seq=0, event="grip started", arm="2R", verb="open", width_m=0.01),
        dict(seq=1, event="grip done", arm="2R", verb="open", width_before_m=0.01,
             width_after_m=0.079, grasped=False, nothing_to_do=False, why=""),
        dict(seq=2, event="job done", why="")]))
    v = _wait_report(c, jid)
    assert v["state"] == "done", v["why"]
    assert v["report"]["width_before_m"] == 0.01 and v["report"]["width_after_m"] == 0.079


def test_uploaded_drawings_and_the_gui_route(tmp_path):
    st = open_station(TWO, speed=math.inf, uncalibrated=True, cache_dir=None,
                      jobs_dir=tmp_path / "jobs", workers=2, with_area=False)
    st.drawing_area = st.maps_area = tuple(st.rig.drawing_area_m)
    c = TestClient(create_app(st))
    r = c.post("/drawings", files=dict(file=("pic.svg", SVG.encode(), "image/svg+xml")),
               data=dict(width="0.3", at="-0.3,0.1")).json()
    assert r["id"].startswith("pic-") and r["lines"] == 3 and r["kind"] == "svg"
    small = json.dumps(dict(units="mm", lines=[dict(id="a", points=[[-300, 100],
                                                                     [-200, 150]])]))
    r2 = c.post("/drawings", files=dict(file=("small.json", small.encode(), "application/json")))
    assert r2.status_code == 200 and r2.json()["lines"] == 1
    nowidth = c.post("/drawings", files=dict(file=("pic.svg", SVG.encode(), "image/svg+xml")))
    assert nowidth.status_code == 400
    assert nowidth.json()["detail"] == "an SVG needs its width on the table, in metres"
    bad = c.post("/drawings", files=dict(file=("x.png", b"..", "image/png")))
    assert bad.status_code == 400 and bad.json()["refused"] == "format"
    assert [m["id"] for m in c.get("/drawings").json()] == [r["id"], r2.json()["id"]]
    assert c.get("/rig").json()["mark_groups"] == {k: list(v)
                                                    for k, v in st.rig.mark_groups.items()}
    assert (tmp_path / "drawings" / f"{r['id']}.json").exists()
    jid = c.post(f"/jobs?drawing={r2.json()['id']}").json()["id"]
    v = _wait_report(c, jid)
    assert v["report"]["drawing"]["lines"] == 1 and v["name"] == r2.json()["id"]
    assert c.post("/jobs?drawing=nothing-00000000").status_code == 404
    assert c.get("/", follow_redirects=False).headers["location"] == "/gui/"
    assert c.get("/gui/").status_code in (200, 404)            # 200 once the GUI is written


def test_robot_names_from_the_site_table(tmp_path):
    st = open_station(TWO, uncalibrated=True, cache_dir=None, jobs_dir=tmp_path / "jobs",
                      workers=2, with_area=False, site=ROOT / "site" / "aris_2026-10.json")
    c = TestClient(create_app(st))
    assert c.get("/rig").json()["arms"]["2L"]["robot"] == "fr3-97"
    assert c.get("/arms").json()["arms"]["2R"]["robot"] == "fr3-71"
    r = open_station(TWO, uncalibrated=True, with_arms=False, with_area=False,
                     site=tmp_path / "missing.json")
    assert isinstance(r, Refusal) and r.reason == "site"


# --------------------------------------------------------------------------- crosses


def test_the_spots_and_the_shapes():
    from aris.server import crosses
    rig = Rig.load(ROOT / "config")
    spots = crosses.row_spots(rig, rig.mark_groups["rows12"])
    assert [(s["spot"], s["cross"], s["circle"]) for s in spots] == [
        ("A", "2L", "2R"), ("B", "2L", "2R"), ("R1a", "1L", "1R"), ("R1b", "1L", "1R")]
    cross = crosses.shape_lines("A", (0.0, -0.4), "cross", -0.002)
    circle = crosses.shape_lines("A", (0.0, -0.4), "circle", -0.002)
    assert len(cross) == 2 and all(np.ptp(x.points[:, :2], axis=0).max() == pytest.approx(0.03)
                                   for x in cross)
    assert len(circle) == 1 and np.ptp(circle[0].points[:, 0]) == pytest.approx(0.02)


def test_crosses_on_the_simulated_arms(tmp_path, capsys):
    st = open_station(TWO, speed=math.inf, uncalibrated=True, cache_dir=ROOT / "out" / "cache",
                      jobs_dir=tmp_path / "jobs", workers=2, with_area=False)
    c = TestClient(create_app(st))
    assert cli.main(["crosses", "--poll", "0.05"], http=ClientHttp(c)) == 0
    out = capsys.readouterr().out
    assert "the L slots draw a cross, the R slots a circle" in out
    assert "spot A       at (+0.000, -0.400) m: 2L drew the cross, 2R the circle" in out
    assert "the cross and the circle should sit on each other" in out
    jid = c.get("/jobs").json()[-1]["id"]
    from aris.execute.queue import Queue
    drawn, entries = {}, []
    for a in ("2L", "2R"):
        got = Queue(st.jobs_dir / jid / f"crosses_{a}__{a}.queue").read()
        entries += got
        drawn[a] = {e.motion.piece.line_id for e in got if e.motion.kind == "draw"}
    # 2 shapes x 2 spots: 2L both strokes of each cross, 2R each circle, every motion checked
    assert drawn == {"2L": {"A cross 1", "A cross 2", "B cross 1", "B cross 2"},
                     "2R": {"A circle", "B circle"}}
    assert entries and all(e.verdict["passed"] for e in entries)
    done = [r for r in c.get(f"/jobs/{jid}/events").json() if r.get("event") == "motion done"]
    assert len(done) == len(entries)
    for a, d in st.drivers.items():
        assert np.max(np.abs(d.state().q - st.rig.park_q(a))) < 1e-6


# --------------------------------------------------------------------------- mark: the meeting


def test_the_neighbour_pairs():
    from aris.server import mark
    rig = Rig.load(ROOT / "config")
    assert mark.neighbour_pairs(rig, rig.mark_groups["rows12"]) == [
        dict(slots=("1L", "1R"), spots=["R1a", "R1b"], kind="row"),
        dict(slots=("1L", "2L"), spots=["S12L"], kind="column"),
        dict(slots=("1R", "2R"), spots=["S12R"], kind="column"),
        dict(slots=("2L", "2R"), spots=["A", "B"], kind="row")]


def test_a_group_meets_pair_by_pair_and_solves_once(tmp_path, monkeypatch):
    import shutil
    import aris.server.meetings as real
    got, solver = [], real.calibrate_from_meetings

    def spy(config_dir, meetings, *a, **k):            # the real graph solve, watched
        got.append(meetings)
        return solver(config_dir, meetings, *a, **k)
    monkeypatch.setattr(real, "calibrate_from_meetings", spy)
    cfg = tmp_path / "cfg"
    shutil.copytree(ROOT / "config", cfg, symlinks=False)
    st = open_station(cfg, speed=math.inf, uncalibrated=True, cache_dir=ROOT / "out" / "cache",
                      jobs_dir=tmp_path / "jobs", workers=2, with_area=False,
                      sim_base_error=(3.0, 2.0))
    c = TestClient(create_app(st))
    v = _wait_report(c, c.post("/mark?group=rows12").json()["id"], 300)
    assert v["state"] == "done", v["why"]
    (meetings,) = got
    assert [(a, b, spot) for a, _, b, _, spot in meetings] == [
        ("1L", "1R", "R1a"), ("1L", "2L", "S12L"), ("1R", "2R", "S12R"), ("2L", "2R", "A")]
    s = v["report"]["solved"]
    assert s["passed"] and set(s["slots"]) == {"1L", "1R", "2L", "2R"} and s["reference"]


def test_mark_brings_the_tips_together(tmp_path, capsys):
    import shutil
    cfg = tmp_path / "cfg"
    shutil.copytree(TWO, cfg, symlinks=False)
    st = open_station(cfg, speed=math.inf, uncalibrated=True, cache_dir=ROOT / "out" / "cache",
                      jobs_dir=tmp_path / "jobs", workers=2, with_area=False,
                      sim_base_error=(3.0, 2.0))
    truth = {a: st.rig.T_table_base(a) for a in st.rig.arm_ids}
    from aris.server.simtruth import Truth
    true = Truth(st.rig, None, (3.0, 2.0)).T
    c = TestClient(create_app(st))
    assert cli.main(["mark", "--yaw", "--poll", "0.05"], http=ClientHttp(c)) == 0, \
        capsys.readouterr().out
    out = capsys.readouterr().out
    assert "switch BOTH to programming mode in Desk" in out
    assert "meeting      2L/2R at A: registered" in out and "meeting      2L/2R at B: registered" in out
    rep = c.get(f"/jobs/{c.get('/jobs').json()[-1]['id']}/report").json()
    (m1, m2) = rep["meetings"]
    # the TRUE tips met: through the true bases both joints put the tip at one point
    for m in (m1, m2):
        tips = [(true[a] @ np.r_[st.rig.arm(a).tip(np.asarray(m["q"][a])[None])[0], 1.0])[:3]
                for a in ("2L", "2R")]
        assert np.linalg.norm(tips[0] - tips[1]) < 1.5e-3
    s = rep["solved"]
    assert s["passed"] and s["slots"]["2R"]["yaw"] == "meetings" and s["residual_mm"] < 1.0
    assert s["reference"] == "2L" and s["slots"]["2L"]["yaw"] == "reference"
    # the solved seam (2R seen from 2L) is the true one, within the guiding error
    T = {a: st.rig.T_table_base(a) for a in ("2L", "2R")}          # reloaded: solved
    seam = np.linalg.inv(T["2L"]) @ T["2R"]
    seam_true = np.linalg.inv(true["2L"]) @ true["2R"]
    assert np.linalg.norm(seam[:2, 3] - seam_true[:2, 3]) < 1.5e-3
    assert abs(np.arctan2(seam[1, 0], seam[0, 0]) - np.arctan2(seam_true[1, 0],
                                                               seam_true[0, 0])) < 3e-3
    assert any(np.abs(T[a] - truth[a]).max() > 1e-4 for a in T)
    for a, d in st.drivers.items():
        assert np.max(np.abs(d.state().q - st.rig.park_q(a))) < 1e-6


def test_mark_with_one_meeting_keeps_the_yaw(tmp_path):
    import shutil
    cfg = tmp_path / "cfg"
    shutil.copytree(TWO, cfg, symlinks=False)
    st = open_station(cfg, speed=math.inf, uncalibrated=True, cache_dir=ROOT / "out" / "cache",
                      jobs_dir=tmp_path / "jobs", workers=2, with_area=False,
                      sim_base_error=(3.0, 2.0))
    c = TestClient(create_app(st))
    v = _wait_report(c, c.post("/mark").json()["id"], 120)
    assert v["state"] == "done", v["why"]
    s = v["report"]["solved"]
    assert s["slots"]["2R"]["yaw"] == "nominal" and len(v["report"]["meetings"]) == 1
    assert c.post("/mark?slots=2L").json()["refused"] == "no_pair"



# --------------------------------------------------------------------------- retreat


def test_park_retreats_from_touching_tips(tmp_path, capsys):
    from aris.execute.drivers.sim import SimArm
    from aris.execute.queue import Queue
    from aris.sequencer.guard import Guard
    from aris.server import retreat
    from aris.server.mark import MEET_HEIGHT, hover
    from aris.server.steps import Scene
    st = open_station(TWO, speed=math.inf, uncalibrated=True, cache_dir=ROOT / "out" / "cache",
                      jobs_dir=tmp_path / "jobs", workers=2, with_area=False)
    st.drawing_area = st.maps_area = tuple(st.rig.drawing_area_m)
    rig = st.rig
    park = {a: rig.park_q(a) for a in rig.arm_ids}
    xy = rig.marks["A"][0]
    q = {}
    for a, side in (("2L", -1.0), ("2R", 1.0)):        # tips 20 mm apart, each alone in the scene
        obs, _, _ = Scene(rig).of(a, park, (a,), (), "x")
        q[a] = hover(st, a, [xy[0] + side * 0.010, xy[1], rig.paper_z + MEET_HEIGHT],
                     Guard(rig.arm(a), obs, st.rules.gates), park[a])
        st.drivers[a] = SimArm(a, q[a])
    assert retreat.gap(rig, "2L", q["2L"], "2R", q["2R"]) < retreat.clearance(rig)
    c = TestClient(create_app(st))
    small = json.dumps(dict(units="mm", lines=[dict(id="a", points=[[-300, 100], [-200, 150]])]))
    assert c.post("/jobs", content=small.encode()).json()["refused"] == "too_close"
    assert cli.main(["park", "--poll", "0.05"], http=ClientHttp(c)) == 0, capsys.readouterr().out
    jid = c.get("/jobs").json()[-1]["id"]
    names = [json.loads(x)["phase"]["name"] if "phase" in json.loads(x) else json.loads(x).get("name")
             for x in (st.jobs_dir / jid / "phases.jsonl").read_text().splitlines()
             if x and not json.loads(x).get("end")]
    assert names == ["retreat 2L", "park 2L", "park 2R"], names
    got = Queue(st.jobs_dir / jid / "retreat_2L__2L.queue").read()
    assert [e.motion.kind for e in got] == ["retreat"] and all(e.verdict["passed"] for e in got)
    for a, d in st.drivers.items():
        assert np.max(np.abs(d.state().q - rig.park_q(a))) < 1e-6
