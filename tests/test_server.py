"""Tests of the drawing server (aris/server) and the command line (aris/cli/).

Quick set: the FastAPI test client over simulated arms, drawable maps on a 5 cm grid (as the
system planner's quick tests), no cache.  Slow: the word through `aris draw` against a live
`aris serve` in a subprocess, arms at 20 times real time.
"""
from __future__ import annotations

import json
import math
import re
import socket
import subprocess
import sys
import time
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from fastapi.testclient import TestClient

from aris import cli
from aris.execute.drivers.sim import SimArm
from aris.server import drawing, open_station
from aris.server.server import create_app
from aris.system.settings import Settings
from aris.types import Refusal

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config"
DATA = ROOT / "tests" / "data"
SMALL, WIDE, WORD = DATA / "server_small.json", DATA / "server_wide.json", \
    DATA / "server_word.json"
COARSE = Settings(grid_step=0.05)
QUICK = ["--uncalibrated", "--cache", "none", "--map-grid", "0.05", "--workers", "4"]


class ClientHttp:
    """The command line's HTTP, through the FastAPI test client."""

    def __init__(self, client):
        self.c = client

    def get(self, path):
        r = self.c.get(path)
        return r.status_code, r.json()

    def post(self, path, body=b""):
        r = self.c.post(path, content=body)
        return r.status_code, r.json()


@pytest.fixture(scope="module")
def station(tmp_path_factory):
    st = open_station(CONFIG, speed=math.inf, uncalibrated=True, cache_dir=None,
                      jobs_dir=tmp_path_factory.mktemp("jobs"), workers=4,
                      settings=COARSE)
    assert not isinstance(st, Refusal), st
    return st


def _with_arms(st, tmp, speed, q=None):
    rig = st.rig
    drivers = {a: SimArm(a, rig.park_q(a) if q is None else q[a], speed=speed)
               for a in rig.arm_ids}
    return replace(st, drivers=drivers, speed=speed, jobs_dir=tmp)


def _wait(client, jid, timeout=120.0, until=("done", "stopped", "failed")):
    t0 = time.time()
    while time.time() - t0 < timeout:
        v = client.get(f"/jobs/{jid}").json()
        if v["state"] in until and (v["state"] not in ("done", "stopped", "failed")
                                    or v["report"] is not None):
            return v
        time.sleep(0.05)
    raise AssertionError(f"job {jid} still {v['state']} after {timeout} s")


# --------------------------------------------------------------------------- the drawing


def test_drawing_file_read_and_fitted(tmp_path):
    lines = drawing.load(SMALL)
    assert [x.id for x in lines] == ["under13", "under71"]
    assert np.allclose(lines[0].points, [[-0.45, -1.0, 0.0], [-0.30, -0.95, 0.0]])
    assert lines[1].intensity == 0.8 and lines[0].frame == "table"
    # refusals are returned, never raised
    for bad in (b"not json", b'{"units": "inch", "lines": []}',
                b'{"units": "mm", "frame": "base", "lines": [{"points": [[0,0],[1,1]]}]}',
                b'{"units": "mm", "lines": [{"id": "a", "points": [[0, 0]]}]}'):
        assert isinstance(drawing.parse(bad), Refusal), bad
    # a repeated id is renamed, an intensity outside 0..1 clamped: no refusal
    two = drawing.parse(b'{"units": "mm", "lines": [{"id": "a", "points": [[0,0],[1,1]]},'
                        b' {"id": "a", "points": [[0,0],[1,1]], "intensity": 2}]}')
    assert [x.id for x in two] == ["a", "a#2"] and two[1].intensity == 1.0
    # fitting: inside is not touched; outside is scaled about the centre; too big is refused
    area = (1.56, 3.56)
    same, fit = drawing.fit(lines, area)
    assert fit.scale == 1.0 and same[0] is lines[0]
    wide = drawing.load(WIDE)
    out, fit = drawing.fit(wide, area)
    assert fit.scale == pytest.approx(0.78 / 1.1)
    assert fit.bbox_out[2] <= 0.78 + 1e-12 and fit.bbox_in == (1.0, 0.2, 1.1, 0.3)
    # about the drawing area's centre, not the table's
    off, fit = drawing.fit(wide, (0.9, 1.6), centre=(0.3, 0.605))
    assert fit.scale == pytest.approx(0.45 / 0.8) and fit.centre == (0.3, 0.605)
    assert off[0].points[0, 0] == pytest.approx(0.3 + fit.scale * 0.7)
    assert np.all(np.abs(off[0].points[:, :2] - (0.3, 0.605)) <= (0.45 + 1e-12, 0.8))
    huge = drawing.parse(b'{"units": "m", "lines": [{"points": [[0, 0], [2, 0]]}]}')
    small, fit = drawing.fit(huge, area)                  # shrunk however much it takes
    assert fit.scale == pytest.approx(0.78 / 2.0)


# --------------------------------------------------------------------------- the station


def test_refuses_to_start_without_calibration_or_with_an_unbuilt_driver():
    r = open_station(CONFIG, with_arms=False, with_area=False)
    assert isinstance(r, Refusal) and r.reason == "uncalibrated" and "--uncalibrated" in r.detail
    r = open_station(CONFIG, driver="ros", uncalibrated=True, with_arms=False, with_area=False)
    assert isinstance(r, Refusal) and r.reason == "driver"


def test_a_drawing_area_the_maps_do_not_cover_is_named():
    from aris.server.station import area_mismatch
    assert area_mismatch((1.56, 3.56), (1.56, 3.56), 0.02) == ""
    assert area_mismatch((1.56, 3.56), (1.58, 3.56), 0.02) == ""  # within one grid cell
    assert area_mismatch((1.56, 3.56), (1.20, 1.00), 0.02) == ""  # smaller on purpose: allowed
    why = area_mismatch((1.56, 3.56), (1.60, 3.56), 0.02)
    assert "stale" in why and "4.0 cm" in why
    why = area_mismatch((0.0, 0.0), (0.10, 0.90), 0.02, (0.3, -1.2))
    assert "the drawable maps are empty about the centre (+0.300, -1.200)" in why


def test_the_report_reasons_are_the_shared_ones():
    from typing import get_args
    from aris.server.report import account
    from aris.types import Leftover, Line, Piece, Reason
    line = Line("a", np.array([[0.0, 0, 0], [0.1, 0, 0]]), "table")
    for rest in ("stopped", "failed", "unaccounted"):
        acc = account([line], [Piece("a", 0.0, 0.02)], [Leftover(Piece("a", 0.05, 0.06),
                                                                  "blocked")], rest, 0.01)
        assert set(acc["left_by_reason"]) == {"blocked", rest} <= set(get_args(Reason))
        assert acc["drawn_m"] + acc["left_m"] == pytest.approx(0.1)
        assert acc["left_by_reason"][rest] == pytest.approx(0.07)   # never planned: the gaps


def test_rig_and_arms_endpoints(station):
    c = TestClient(create_app(station))
    r = c.get("/rig").json()
    assert set(r["arms"]) == {"1L", "1R", "2L", "2R", "3L", "3R"}
    assert r["arms"]["2L"]["calibration"] == {"base": "none", "pen": "none"}
    assert r["uncalibrated"] is True and "tracking" not in r
    assert r["pen_in"]["name"] == station.rig.pen()["name"] and len(r["drawing_area_centre_m"]) == 2
    assert np.allclose(r["arms"]["2R"]["park_q"], station.rig.park_q("2R"))
    assert len(r["rig_digest"]) == 24 and len(r["calibration_digest"]) == 24
    assert 1.0 < r["drawing_area_m"][0] < 1.8 and 3.0 < r["drawing_area_m"][1] < 3.7
    assert r["drawing_area_m"] == [1.56, 3.56]
    assert np.max(np.abs(np.subtract(r["drawing_area_from_maps_m"], [1.56, 3.56]))) <= 0.05
    assert r["driver"] == "sim" and r["speed"] == "inf"
    arms = c.get("/arms").json()
    assert all(v["at_park"] and v["ok"] for v in arms.values())
    assert c.get("/jobs/nothing").status_code == 404


# --------------------------------------------------------------------------- jobs


@pytest.mark.slow  # 5 to 17 s: over the quick set's budget (orchestrator, 2026-10-01)
def test_small_drawing_runs_to_done_and_a_second_job_is_refused(station, tmp_path):
    c = TestClient(create_app(replace(station, jobs_dir=tmp_path)))
    r = c.post("/jobs?name=server_small.json", content=SMALL.read_bytes())
    assert r.status_code == 200
    jid = r.json()["id"]
    second = c.post("/jobs", content=SMALL.read_bytes())
    assert second.status_code == 409 and second.json()["refused"] == "busy"
    assert c.post("/park").status_code == 409
    v = _wait(c, jid)
    rep = v["report"]
    print(f"small drawing: first motion {rep['first_motion_s']:.1f} s, "
          f"total {rep['total_s']:.1f} s, checker {rep['checker']}")
    assert v["state"] == "done", v["why"]
    assert rep["drawn_m"] == pytest.approx(rep["length_m"]) and rep["left_m"] == 0.0
    assert rep["length_m"] == pytest.approx(2 * np.hypot(0.15, 0.05))
    assert rep["checker"]["checked"] == rep["queued"] == 10 and rep["checker"]["refused"] == 0
    assert rep["checker"]["all_queued_checked"] and rep["passed"]
    assert all(e.verdict["passed"] for e in
               __import__("aris.execute", fromlist=["Job"]).Job(tmp_path / jid)
               .queue("phase 1", "1L").read())
    assert rep["drawing"]["scale"] == 1.0 and all(rep["at_park"].values())
    assert rep["assumptions"]["uncalibrated"] is True
    assert all(p["end_check_passed"] for p in rep["phases"])
    rows = {(r["phase"], r["arm"]): r for r in v["arms"]}
    assert rows[("phase 1", "1L")]["done"] == rows[("phase 1", "1L")]["queued"] == 5
    states = [e["state"] for e in c.get(f"/jobs/{jid}/events").json()
              if e["event"] == "job state"]
    assert states == ["received", "fitted", "planning", "drawing", "done"]
    job_dir = tmp_path / jid
    assert {"job.json", "phases.jsonl", "events.jsonl", "report.json"} <= \
        {p.name for p in job_dir.iterdir()}
    assert cli.main(["status"], http=ClientHttp(c)) == 0


def test_leftovers_are_a_report_not_a_failure():
    done = dict(state="done", passed=True, left_m=0.008, left_by_reason={"unreachable": 0.008})
    assert cli.job_passed(done) and "unreachable 0.008 m" in cli._summary(done)
    assert not cli.job_passed(dict(done, state="stopped", passed=False))


@pytest.mark.slow
def test_an_air_run_flies_every_draw_30_mm_up(tmp_path, capsys):
    from aris.execute import Job
    from aris.rig import Rig
    out = tmp_path / "air"
    assert cli.main(["plan", str(SMALL), "--out", str(out), "--air", "30"] + QUICK) == 0
    text = capsys.readouterr().out
    assert "AIR RUN" in text and text.strip().splitlines()[-1].startswith("PASS")
    rig = Rig.load(CONFIG)
    job = Job(out)
    assert job.header()["air_mm"] == 30.0
    rep = json.loads((out / "report.json").read_text())
    assert rep["air_mm"] == 30.0 and rep["drawn_m"] == pytest.approx(rep["length_m"])
    draws = 0
    for phase, _ in job.phases():
        for a in phase.active:
            for e in job.queue(phase.name, a).read():
                if e.motion.kind == "draw":
                    z = rig.to_table(a, e.motion.tip_base)[:, 2] - rig.paper_z
                    assert np.all(np.abs(z - 0.030) < 1e-4), (a, z.min(), z.max())
                    draws += 1
    assert draws == 2


@pytest.mark.slow  # 5 to 17 s: over the quick set's budget (orchestrator, 2026-10-01)
def test_plan_and_check_commands(tmp_path, capsys):
    out = tmp_path / "plan"
    assert cli.main(["plan", str(SMALL), "--out", str(out)] + QUICK) == 0
    text = capsys.readouterr().out
    assert "UNCALIBRATED" in text and text.strip().splitlines()[-1].startswith("PASS")
    assert json.loads((out / "report.json").read_text())["drawn_m"] == pytest.approx(0.3162,
                                                                                    abs=1e-4)
    assert cli.main(["check", str(out), "--check-workers", "4"]) == 0
    text = capsys.readouterr().out
    assert text.count(" pass  tightest") == 10 and "PASS: 10 of 10" in text


@pytest.mark.slow  # 5 to 17 s: over the quick set's budget (orchestrator, 2026-10-01)
def test_outside_the_area_is_scaled_and_a_stop_leaves_leftovers(station, tmp_path):
    st = _with_arms(station, tmp_path, speed=1.0)
    c = TestClient(create_app(st))
    jid = c.post("/jobs", content=WIDE.read_bytes()).json()["id"]
    v = c.get(f"/jobs/{jid}").json()
    half = 0.5 * np.asarray(st.drawing_area)
    assert v["drawing"]["scale"] == pytest.approx(min(half[0] / 1.1, half[1] / 0.3))
    assert v["drawing"]["bbox"][2] <= half[0] + 1e-9
    # stop while the arm draws: every arm stops and holds, the job ends with leftovers
    t0 = time.time()
    while not any(r["current"] for r in c.get(f"/jobs/{jid}").json()["arms"]):
        assert time.time() - t0 < 120, "no motion started"
        time.sleep(0.05)
    assert c.post(f"/jobs?rest_of={jid}").json()["refused"] == "running"
    while not any(r["current"] and r["current"]["kind"] == "draw"
                  for r in c.get(f"/jobs/{jid}").json()["arms"]):
        assert time.time() - t0 < 120, "no drawing motion started"
        time.sleep(0.05)
    time.sleep(1.0)
    assert c.post(f"/jobs/{jid}/stop").status_code == 200
    v = _wait(c, jid)
    rep = v["report"]
    assert v["state"] == "stopped" and rep["drawing"]["scale"] < 1.0
    assert rep["left_by_reason"].get("stopped", 0.0) > 0.0
    assert rep["drawn_m"] + rep["left_m"] == pytest.approx(rep["length_m"])
    for a, d in st.drivers.items():
        s = d.state()
        assert "moving" not in s.flags and not s.ok and np.all(s.qd == 0.0), (a, s.flags)
    assert c.post(f"/jobs/{jid}/stop").status_code == 200      # already finished: nothing to do
    # park from where the stop left them (a pen at the paper rises first)
    assert cli.main(["park", "--poll", "0.05"], http=ClientHttp(c)) == 0
    for a, d in st.drivers.items():
        assert np.max(np.abs(d.state().q - st.rig.park_q(a))) < 1e-9, a
    # what the stop left over, drawn as a new drawing: the two together are the whole drawing
    assert cli.main(["draw", "--rest-of", jid, "--note", "4H on 120 g paper", "--poll", "0.05"],
                    http=ClientHttp(c)) == 0
    rid = c.get("/jobs").json()[-1]["id"]
    rest = c.get(f"/jobs/{rid}").json()["report"]
    assert rest["rest_of"] == jid and rest["note"] == "4H on 120 g paper"
    assert rest["drawing"]["scale"] == 1.0 and rest["state"] == "done"
    assert rep["drawn_m"] + rest["drawn_m"] == pytest.approx(rep["length_m"],
                                                             abs=st.rules.min_piece)
    head = json.loads((st.jobs_dir / rid / "job.json").read_text())
    assert head["note"] == "4H on 120 g paper" and "tracking" not in head
    assert head["pen"]["name"] == st.rig.pen()["name"] and head["rest_of"] == jid
    assert c.post(f"/jobs?rest_of={rid}").json()["refused"] == "nothing_left"
    # a refused drawing is a FAIL of `aris draw`
    bad = tmp_path / "bad.json"
    bad.write_text('{"units": "inch", "lines": [{"id": "x", "points": [[0, 0], [3, 0]]}]}')
    assert cli.main(["draw", str(bad), "--poll", "0.05"], http=ClientHttp(c)) == 1


@pytest.mark.slow  # 5 to 17 s: over the quick set's budget (orchestrator, 2026-10-01)
def test_park_all_arms_from_near_their_parks(station, tmp_path, capsys):
    rng = np.random.default_rng(3)
    rig = station.rig
    q0 = {a: rig.park_q(a) + rng.uniform(-0.05, 0.05, 7) for a in rig.arm_ids}
    q0["3R"] = rig.park_q("3R")                         # one already parked: left alone
    st = _with_arms(station, tmp_path, speed=math.inf, q=q0)
    c = TestClient(create_app(st))
    t = time.process_time()
    assert cli.main(["park", "--poll", "0.05"], http=ClientHttp(c)) == 0
    text = capsys.readouterr().out
    print(text, f"park: CPU {time.process_time() - t:.1f} s")
    for a, d in st.drivers.items():
        assert np.max(np.abs(d.state().q - rig.park_q(a))) < 1e-9, a
    jid = c.get("/jobs").json()[-1]["id"]
    rep = c.get(f"/jobs/{jid}").json()["report"]
    assert rep["arms"]["3R"]["result"] == "already at its park"
    assert rep["checker"]["checked"] == rep["checker"]["passed"] == 5
    assert [p["name"] for p in rep["phases"]] == ["park 1L", "park 1R", "park 2L", "park 2R",
                                                   "park 3L"]
    # every park queue checks again from the job directory, footprints included
    assert cli.main(["check", str(tmp_path / jid), "--check-workers", "4"]) == 0


# --------------------------------------------------------------------------- slow


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.mark.slow
def test_word_through_aris_draw_against_a_live_server(tmp_path):
    port = _free_port()
    url = f"http://127.0.0.1:{port}"
    jobs = tmp_path / "jobs"
    serve = subprocess.Popen(
        [sys.executable, "-m", "aris.cli", "serve", "--port", str(port), "--speed", "20",
         "--uncalibrated", "--cache", str(tmp_path / "cache"), "--jobs", str(jobs),
         "--workers", "8"],
        cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        t0 = time.time()
        while not _up(url):
            assert serve.poll() is None, serve.stdout.read()
            assert time.time() - t0 < 600, "the server did not come up"
            time.sleep(0.5)
        print(f"server up after {time.time() - t0:.1f} s (drawable maps built from scratch)")
        t1 = time.time()
        r = subprocess.run([sys.executable, "-m", "aris.cli", "draw", str(WORD), "--server",
                            url], cwd=ROOT, capture_output=True, text=True, timeout=900)
        print(r.stdout[-3000:], r.stderr[-2000:])
        assert r.returncode == 0 and r.stdout.strip().splitlines()[-1].startswith("PASS")
        rep = json.loads(next(jobs.glob("*/report.json")).read_text())
        print(f"word: first motion {rep['first_motion_s']:.1f} s, total {rep['total_s']:.1f} s "
              f"(the command: {time.time() - t1:.1f} s); checker {rep['checker']}")
        assert rep["state"] == "done" and rep["left_m"] == 0.0
        assert rep["drawn_m"] == pytest.approx(rep["length_m"])
        assert all(rep["at_park"].values())
        assert re.search(r"first motion\s+[0-9.]+ s", r.stdout)
    finally:
        serve.terminate()
        serve.wait(timeout=30)


def _up(url) -> bool:
    try:
        return cli.Http(url).get("/rig")[0] == 200
    except OSError:
        return False



def test_the_verify_keeps_a_refused_motion_and_pickles(station, tmp_path):
    import pickle
    from aris.kernel.retime import retime
    from aris.server.verify import CheckVerify
    from aris.types import JointPath, Motion, Piece
    rig = station.rig
    p, arm = rig.park_q("1L"), rig.arm("1L")
    traj = retime(JointPath(np.array([p, p + 0.05])), arm.limits, rig.rules())
    motion = Motion("free", traj, Piece("x", 0.0, 0.1))
    verify = pickle.loads(pickle.dumps(CheckVerify(CONFIG, tmp_path / "refused")))
    ok = verify("1L", rig.phase(1), motion, p)
    assert ok["passed"] and "refused_file" not in ok and not (tmp_path / "refused").exists()
    for n in range(2):                             # q_before is not where it starts: refused
        bad = verify("1L", rig.phase(1), motion, p + 0.01)
        assert not bad["passed"] and bad["refused_file"] == f"phase_1__1L__{n}.npz"
    with np.load(tmp_path / "refused" / "phase_1__1L__1.npz") as z:
        assert np.array_equal(z["q"], traj.q) and np.array_equal(z["t"], traj.t)
        assert np.array_equal(z["qd"], traj.qd) and np.allclose(z["q_before"], p + 0.01)
        assert str(z["kind"]) == "free" and json.loads(str(z["piece"])) == ["x", 0.0, 0.1]
        assert "starts at q_before" in str(z["failed"]) and float(z["intensity"]) == 1.0
    # the queue takes the checker's word the motion carries, and nothing unchecked
    from aris.execute.queue import Queue
    q = Queue(tmp_path / "a.queue", "phase 1", "1L")
    assert q.append(motion).reason == "unchecked"
    assert q.append(replace(motion, checked=bad)).reason == "failed_check"
    assert q.append(replace(motion, checked=ok)) == 0
    assert q.read()[0].verdict == ok and q.read()[0].motion.checked == ok
