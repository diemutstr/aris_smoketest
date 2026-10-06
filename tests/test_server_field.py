"""Fixes from the first site day (2026-10-06): a single slot's mark job, positions that are no
readings, `aris arms`, the rest of a failed job, contacts tripped at the start."""
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
    arms = c.get("/arms").json()
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


# --------------------------------------------------------------------------- 5. start trips


def test_a_contact_tripped_at_the_start_is_dropped(tmp_path):
    from aris.server.calibrate import CalibSettings, plan_calibrate, tripped
    st = open_station(TWO, uncalibrated=True, cache_dir=None, with_arms=False, with_area=False)
    st.drawing_area = tuple(st.rig.drawing_area_m)
    rig = st.rig
    plan = plan_calibrate(st, "2L", {a: rig.park_q(a) for a in rig.arm_ids},
                          CalibSettings(grid=3), min_points=1)
    touches = [(i, m) for i, m in enumerate(m for s in plan.steps for m in s.motions)
               if m.kind == "touch"]
    (i0, t0), (i1, t1) = touches[:2]
    bottom = lambda m: m.traj.q[int(np.argmax(np.linalg.norm(m.traj.q - m.traj.q[0], axis=1)))]
    early = t0.traj.q[np.searchsorted(t0.traj.t, 0.3)]        # 1.5 mm into the descent
    rows = [dict(event="contact", arm="2L", index=i0, q=list(early)),
            dict(event="contact", arm="2L", index=i1, q=list(bottom(t1)))]
    kept, dropped = tripped(rig, plan, rows, "2L")
    assert [r["index"] for r in kept] == [i1] and len(dropped) == 1
    assert dropped[0] == tuple(round(float(x), 4) for x in plan.points_table[0])
