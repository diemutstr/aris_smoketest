"""The drawing server with `--driver robot`: the operator PC's runner (robot/aris_robot) copies
the queues over real HTTP, runs them on its own (here simulated) arms and posts the events
back; the server runs no executors and its job follows those events."""
from __future__ import annotations

import json
import math
import sys
import threading
import time
import urllib.error
import urllib.request
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
for p in (ROOT / "robot", ROOT / "robot" / "tests"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from aris.execute.drivers.sim import SimArm  # noqa: E402
from aris.server import open_station  # noqa: E402
from aris.server.server import create_app  # noqa: E402
from aris.system.settings import Settings  # noqa: E402
from aris.types import Refusal  # noqa: E402

aris_robot = pytest.importorskip("aris_robot")
from aris_robot.remote import Remote  # noqa: E402
from aris_robot.runner import run_job  # noqa: E402
from fake_server import Served  # noqa: E402  (the uvicorn-in-a-thread helper)

CONFIG = ROOT / "config"
SMALL = ROOT / "tests" / "data" / "server_small.json"


@pytest.fixture(scope="module")
def station(tmp_path_factory):
    st = open_station(CONFIG, driver="robot", uncalibrated=True, cache_dir=None,
                      jobs_dir=tmp_path_factory.mktemp("jobs"), workers=4,
                      settings=Settings(grid_step=0.05))
    assert not isinstance(st, Refusal), st
    return st


def _post(url, body=b""):
    req = urllib.request.Request(url, data=body, method="POST")
    with urllib.request.urlopen(req, timeout=10) as r:
        import json
        return json.loads(r.read())


def _get_json(url):
    import json
    with urllib.request.urlopen(url, timeout=10) as r:
        return json.loads(r.read())


def _wait(url, jid, timeout=120.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        v = _get_json(f"{url}/jobs/{jid}")
        if v["state"] in ("done", "stopped", "failed") and v["report"] is not None:
            return v
        time.sleep(0.05)
    raise AssertionError(f"job {jid} did not end")


@pytest.mark.slow  # 5 to 17 s: over the quick set's budget (orchestrator, 2026-10-01)
def test_the_operator_pc_runs_a_job_of_the_server(station, tmp_path):
    rig = station.rig
    st = replace(station, jobs_dir=tmp_path / "jobs")
    arms = {a: SimArm(a, rig.park_q(a), speed=math.inf) for a in rig.arm_ids}
    with Served(create_app(st)) as srv:
        assert st.drivers == {} and _get_json(f"{srv.url}/rig")["driver"] == "robot"
        with pytest.raises(urllib.error.HTTPError) as e:
            _post(f"{srv.url}/park")
        assert e.value.code == 409
        jid = _post(f"{srv.url}/jobs", SMALL.read_bytes())["id"]
        head = _get_json(f"{srv.url}/jobs/{jid}/header")
        assert head["rig_digest"] == st.digests()["rig_digest"]
        assert head["pen"] == json.loads(json.dumps(rig.pen()))      # travels with the job
        with pytest.raises(urllib.error.HTTPError) as e:           # not written yet
            urllib.request.urlopen(f"{srv.url}/jobs/{jid}/queues/phase%201/13", timeout=10)
        assert e.value.code == 404
        res = run_job(Remote(srv.url), jid, rig, CONFIG, tmp_path / "robot", arms)
        assert res.status == "done", res.why
        v = _wait(srv.url, jid)
        # the copy is the server's queue byte for byte; offsets resume where asked
        server_q = st.jobs_dir / jid / "phase_1__arm13.queue"
        assert (tmp_path / "robot" / jid / "phase_1__arm13.queue").read_bytes() == \
            server_q.read_bytes()
        with urllib.request.urlopen(f"{srv.url}/jobs/{jid}/queues/phase%201/13?offset=100",
                                    timeout=10) as r:
            assert r.read() == server_q.read_bytes()[100:]
    rep = v["report"]
    assert v["state"] == "done", v["why"]
    assert rep["drawn_m"] == pytest.approx(rep["length_m"]) and rep["left_m"] == 0.0
    assert all(p["end_check_passed"] for p in rep["phases"]) and all(rep["at_park"].values())
    rows = {(r["phase"], r["arm"]): r for r in v["arms"]}
    assert rows[("phase 1", 71)]["done"] == rows[("phase 1", 71)]["queued"] == 5
    for a, arm in arms.items():
        assert np.max(np.abs(arm.state().q - rig.park_q(a))) <= 1e-9


def test_events_are_taken_once_in_order(station, tmp_path):
    from fastapi.testclient import TestClient
    st = replace(station, jobs_dir=tmp_path / "jobs")
    c = TestClient(create_app(st))
    jid = c.post("/jobs", content=SMALL.read_bytes()).json()["id"]
    post = lambda rows: c.post(f"/jobs/{jid}/events", json=dict(source="robot", rows=rows)).json()
    row = lambda n: dict(seq=n, event="holding", time=float(n), arm=13, phase="phase 1")
    assert post([row(0), row(1)]) == dict(accepted=2, next_seq=2, stop=False)
    assert post([row(1), row(2), row(4)]) == dict(accepted=1, next_seq=3, stop=False)
    assert c.post(f"/jobs/{jid}/stop").status_code == 200
    assert post([]) == dict(accepted=0, next_seq=3, stop=True)
    got = [r for r in c.get(f"/jobs/{jid}/events").json() if r.get("source") == "robot"]
    assert [r["seq"] for r in got] == [0, 1, 2]
    assert c.post("/jobs/nothing/events", json=dict(rows=[])).status_code == 404


@pytest.mark.slow
def test_a_stop_on_the_server_reaches_the_operator_pc(station, tmp_path):
    rig = station.rig
    st = replace(station, jobs_dir=tmp_path / "jobs")
    arms = {a: SimArm(a, rig.park_q(a), speed=1.0) for a in rig.arm_ids}
    with Served(create_app(st)) as srv:
        jid = _post(f"{srv.url}/jobs", SMALL.read_bytes())["id"]
        out = []
        t = threading.Thread(target=lambda: out.append(run_job(
            Remote(srv.url), jid, rig, CONFIG, tmp_path / "robot", arms)), daemon=True)
        t.start()
        t0 = time.time()
        while not any(r["current"] for r in _get_json(f"{srv.url}/jobs/{jid}")["arms"]):
            assert time.time() - t0 < 120, "no motion started"
            time.sleep(0.05)
        _post(f"{srv.url}/jobs/{jid}/stop")
        t.join(timeout=60)
        v = _wait(srv.url, jid)
    assert out and out[0].status == "stopped"
    assert v["state"] == "stopped" and v["report"]["left_by_reason"].get("stopped", 0) > 0
    assert all("moving" not in a.state().flags for a in arms.values())


def _rchar() -> int:
    for line in Path("/proc/self/io").read_text().splitlines():
        if line.startswith("rchar:"):
            return int(line.split()[1])
    raise AssertionError("no rchar")


def test_a_growing_queue_is_streamed_reading_each_byte_about_once(station, tmp_path):
    from fastapi.testclient import TestClient
    from aris.execute import Job
    from aris.kernel.retime import retime
    from aris.types import JointPath, Motion

    class Passed:
        passed, tightest, min_clearance, min_clearance_at = True, "test", 0.1, "test"

        def get(self, name):
            raise KeyError(name)

    rig = station.rig
    st = replace(station, jobs_dir=tmp_path / "jobs")
    app = create_app(st)
    rec = app.state.store.admit("draw", "test")
    job = Job.create(rec.dir, dict(test=True))
    q = job.queue("phase 1", 13)
    p, arm = rig.park_q(13), rig.arm(13)
    step = np.array([0.02, -0.02, 0.0, 0.03, 0.0, -0.03, 0.04])
    there = retime(JointPath(np.array([p, p + step])), arm.limits, rig.rules())
    back = retime(JointPath(np.array([p + step, p])), arm.limits, rig.rules())
    n = 300
    q.append(Motion("free", there), Passed())           # the file exists before the stream

    def write():
        for i in range(1, n):
            time.sleep(0.002)
            q.append(Motion("free", back if i % 2 else there), Passed())
        q.close()

    got = []
    with TestClient(app) as c:
        r0 = _rchar()
        w = threading.Thread(target=write)
        w.start()
        with c.stream("GET", f"/jobs/{rec.id}/queues/phase%201/13") as r:
            for chunk in r.iter_bytes():
                got.append(chunk)
        w.join()
        read = _rchar() - r0
    size = q.path.stat().st_size
    print(f"queue of {n} motions, {size} bytes, streamed while written: {read} bytes read")
    assert b"".join(got) == q.path.read_bytes()
    assert read < 2 * size + 200_000
    # resuming at an offset inside a record, and after the end marker
    with TestClient(app) as c:
        assert c.get(f"/jobs/{rec.id}/queues/phase%201/13?offset=1000").content == \
            q.path.read_bytes()[1000:]
        assert c.get(f"/jobs/{rec.id}/queues/phase%201/13?offset={size}").content == b""


@pytest.mark.slow
def test_stop_then_park_from_the_reported_positions_then_draw_again(station, tmp_path):
    rig = station.rig
    st = replace(station, jobs_dir=tmp_path / "jobs")
    from aris.server.station import Positions
    st.positions = Positions()                     # this test's own operator PC
    arms = {a: SimArm(a, rig.park_q(a), speed=1.0) for a in rig.arm_ids}
    work = tmp_path / "robot"
    with Served(create_app(st)) as srv:
        with pytest.raises(urllib.error.HTTPError) as e:        # nothing reported yet
            _post(f"{srv.url}/park")
        assert e.value.code == 409 and "no position reported for arm" in e.value.read().decode()
        jid = _post(f"{srv.url}/jobs", SMALL.read_bytes())["id"]
        out = []
        t = threading.Thread(target=lambda: out.append(run_job(
            Remote(srv.url), jid, rig, CONFIG, work, arms)), daemon=True)
        t.start()
        t0 = time.time()
        while not any(r["current"] and r["current"]["kind"] == "draw"
                      for r in _get_json(f"{srv.url}/jobs/{jid}")["arms"]):
            assert time.time() - t0 < 120, "no drawing motion started"
            time.sleep(0.05)
        _post(f"{srv.url}/jobs/{jid}/stop")
        t.join(timeout=60)
        assert _wait(srv.url, jid)["state"] == "stopped"
        seen = _get_json(f"{srv.url}/arms")
        assert set(seen) == {str(a) for a in rig.arm_ids}
        away = [a for a, v in seen.items() if not v["at_park"]]
        assert away and all(v["source"] == "operator PC" for v in seen.values())
        for a, arm in arms.items():
            assert np.allclose(seen[str(a)]["q"], arm.state().q)
            arm.recover()                          # the operator has looked
        # a drawing while arms stand away from their parks outside phase 1 would be refused;
        # park them from where the operator PC said they are
        pid = _post(f"{srv.url}/park")["id"]
        res = run_job(Remote(srv.url), pid, rig, CONFIG, work, arms)
        assert res.status == "done", res.why
        v = _wait(srv.url, pid)
        assert v["state"] == "done", v["why"]
        assert sorted(k for k, r in v["report"]["arms"].items() if r["result"] == "parked") == \
            sorted(away)
        for a, arm in arms.items():
            assert np.max(np.abs(arm.state().q - rig.park_q(a))) <= 1e-9
        assert all(x["at_park"] for x in _get_json(f"{srv.url}/arms").values())
        jid2 = _post(f"{srv.url}/jobs", SMALL.read_bytes())["id"]   # accepted again
        assert _get_json(f"{srv.url}/jobs/{jid2}")["state"] in ("fitted", "planning", "drawing")
        _post(f"{srv.url}/jobs/{jid2}/stop")
        assert _wait(srv.url, jid2)["state"] == "stopped"
