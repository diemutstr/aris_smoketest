"""Tests of the drawing server (aris/server) and the command line (aris/cli.py).

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
QUICK = ["--uncalibrated", "--cache", "none", "--map-grid", "0.05", "--workers", "4",
         "--check-workers", "4"]


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
                      jobs_dir=tmp_path_factory.mktemp("jobs"), workers=4, check_workers=4,
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
    # the same as .npz
    z = tmp_path / "small.npz"
    raw = json.loads(SMALL.read_text())["lines"]
    arr = np.empty(len(raw), object)
    arr[:] = raw
    np.savez(z, units="mm", frame="table", lines=arr)
    again = drawing.load(z)
    assert all(np.array_equal(a.points, b.points) and a.intensity == b.intensity
               for a, b in zip(lines, again))
    # refusals are returned, never raised
    for bad in (b"not json", b'{"units": "inch", "lines": []}',
                b'{"units": "mm", "frame": "base", "lines": [{"points": [[0,0],[1,1]]}]}',
                b'{"units": "mm", "lines": [{"id": "a", "points": [[0, 0]]}]}',
                b'{"units": "mm", "lines": [{"id": "a", "points": [[0,0],[1,1]]},'
                b' {"id": "a", "points": [[0,0],[1,1]]}]}',
                b'{"units": "mm", "lines": [{"points": [[0,0],[1,1]], "intensity": 2}]}'):
        assert isinstance(drawing.parse(bad), Refusal), bad
    # fitting: inside is not touched; outside is scaled about the centre; too big is refused
    area = (1.56, 3.56)
    same, fit = drawing.fit(lines, area)
    assert fit.scale == 1.0 and same[0] is lines[0]
    wide = drawing.load(WIDE)
    out, fit = drawing.fit(wide, area)
    assert fit.scale == pytest.approx(0.78 / 1.1)
    assert fit.bbox_out[2] <= 0.78 + 1e-12 and fit.bbox_in == (1.0, 0.2, 1.1, 0.3)
    huge = drawing.parse(b'{"units": "m", "lines": [{"points": [[0, 0], [2, 0]]}]}')
    r = drawing.fit(huge, area)
    assert isinstance(r, Refusal) and r.reason == "too_large" and "0.39" in r.detail


# --------------------------------------------------------------------------- the station


def test_refuses_to_start_without_calibration_or_with_an_unbuilt_driver():
    r = open_station(CONFIG, with_arms=False, with_area=False)
    assert isinstance(r, Refusal) and r.reason == "uncalibrated" and "--uncalibrated" in r.detail
    r = open_station(CONFIG, driver="ros", uncalibrated=True, with_arms=False, with_area=False)
    assert isinstance(r, Refusal) and r.reason == "driver"


def test_a_stale_drawing_area_in_the_rig_file_is_refused():
    from aris.server.station import area_mismatch
    assert area_mismatch((1.56, 3.56), (1.56, 3.56), 0.02) == ""
    assert area_mismatch((1.58, 3.56), (1.56, 3.56), 0.02) == ""  # within one grid cell
    why = area_mismatch((1.60, 3.56), (1.56, 3.56), 0.02)
    assert "stale" in why and "4.0 cm" in why


def test_the_report_reasons_are_the_shared_ones():
    from typing import get_args
    from aris.server.report import account
    from aris.types import Line, Piece, Reason
    line = Line("a", np.array([[0.0, 0, 0], [0.1, 0, 0]]), "table")
    for rest in ("stopped", "failed", "unaccounted"):
        acc = account([line], [Piece("a", 0.0, 0.02)], [(Piece("a", 0.05, 0.06), "blocked", "")],
                      rest)
        assert set(acc["left_by_reason"]) <= set(get_args(Reason))
        assert acc["drawn_m"] + acc["left_m"] == pytest.approx(0.1)


def test_rig_and_arms_endpoints(station):
    c = TestClient(create_app(station))
    r = c.get("/rig").json()
    assert set(r["arms"]) == {"13", "17", "31", "71", "2", "97"}
    assert r["arms"]["31"]["calibration"] == "none" and r["uncalibrated"] is True
    assert np.allclose(r["arms"]["71"]["park_q"], station.rig.park_q(71))
    assert len(r["rig_digest"]) == 24 and len(r["calibration_digest"]) == 24
    assert 1.0 < r["drawing_area_m"][0] < 1.8 and 3.0 < r["drawing_area_m"][1] < 3.7
    assert r["drawing_area_m"] == [1.56, 3.56]
    assert np.max(np.abs(np.subtract(r["drawing_area_from_maps_m"], [1.56, 3.56]))) <= 0.05
    assert r["driver"] == "sim" and r["speed"] == "inf"
    arms = c.get("/arms").json()
    assert all(v["at_park"] and v["ok"] for v in arms.values())
    assert c.get("/jobs/nothing").status_code == 404


# --------------------------------------------------------------------------- jobs


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
    assert rep["checker"]["passed"] == rep["checker"]["checked"] == rep["queued"] == 10
    assert rep["drawing"]["scale"] == 1.0 and all(rep["at_park"].values())
    assert rep["assumptions"]["uncalibrated"] is True
    assert all(p["end_check_passed"] for p in rep["phases"])
    rows = {(r["phase"], r["arm"]): r for r in v["arms"]}
    assert rows[("phase 1", 13)]["done"] == rows[("phase 1", 13)]["queued"] == 5
    states = [e["state"] for e in c.get(f"/jobs/{jid}/events").json()
              if e["event"] == "job state"]
    assert states == ["received", "fitted", "planning", "drawing", "done"]
    job_dir = tmp_path / jid
    assert {"job.json", "phases.jsonl", "events.jsonl", "report.json"} <= \
        {p.name for p in job_dir.iterdir()}
    assert cli.main(["status"], http=ClientHttp(c)) == 0


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
    time.sleep(0.3)
    assert c.post(f"/jobs/{jid}/stop").status_code == 200
    v = _wait(c, jid)
    rep = v["report"]
    assert v["state"] == "stopped" and rep["drawing"]["scale"] < 1.0
    assert rep["left_by_reason"].get("stopped", 0.0) > 0.0
    assert rep["drawn_m"] + rep["left_m"] == pytest.approx(rep["length_m"])
    for a, d in st.drivers.items():
        s = d.state()
        assert "moving" not in s.flags and not s.ok and np.all(s.qd == 0.0), (a, s.flags)
    assert c.post(f"/jobs/{jid}/stop").status_code == 409      # already finished
    # a drawing that would shrink below half is a failed job, and `aris draw` says FAIL
    huge = tmp_path / "huge.json"
    huge.write_text('{"units": "m", "lines": [{"id": "x", "points": [[0, 0], [3, 0]]}]}')
    assert cli.main(["draw", str(huge), "--poll", "0.05"], http=ClientHttp(c)) == 1


def test_park_all_arms_from_near_their_parks(station, tmp_path, capsys):
    rng = np.random.default_rng(3)
    rig = station.rig
    q0 = {a: rig.park_q(a) + rng.uniform(-0.05, 0.05, 7) for a in rig.arm_ids}
    q0[97] = rig.park_q(97)                         # one already parked: left alone
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
    assert rep["arms"]["97"]["result"] == "already at its park"
    assert rep["checker"]["checked"] == rep["checker"]["passed"] == 5
    assert [p["name"] for p in rep["phases"]] == ["park 13", "park 17", "park 31", "park 71",
                                                   "park 2"]
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
         "--workers", "8", "--check-workers", "8"],
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

