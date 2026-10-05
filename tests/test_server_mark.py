"""The mark job (aris/server/mark.py) on simulated arms with a simulated person, through the
real executor, queues, solver (`aris.calib.solve_marks`) and writer."""
from __future__ import annotations

import json
import math
import shutil
import time
from pathlib import Path

import numpy as np
import pytest
from fastapi.testclient import TestClient

from aris import cli
from aris.rig import Rig
from aris.server import open_station
from aris.server.mark import Task, group_slots, tasks_for
from aris.server.server import create_app
from aris.system.settings import Settings
from aris.types import Refusal

ROOT = Path(__file__).resolve().parents[1]
TWO, SIX = ROOT / "config" / "two_arms", ROOT / "config"
COARSE = Settings(grid_step=0.05)
ERROR = (3.0, 2.0)                    # mm, mrad: every simulated base is off by this much


class ClientHttp:
    def __init__(self, client):
        self.c = client

    def get(self, path):
        r = self.c.get(path)
        return r.status_code, r.json()

    def post(self, path, body=b""):
        r = self.c.post(path, content=body)
        return r.status_code, r.json()


def _config(tmp_path, src) -> Path:
    d = tmp_path / "config"
    (d / "calibration").mkdir(parents=True)
    shutil.copy(src / "rig.json", d / "rig.json")
    return d


def _station(tmp_path, src, buttons=None):
    st = open_station(_config(tmp_path, src), speed=math.inf, uncalibrated=True, cache_dir=None,
                      jobs_dir=tmp_path / "jobs", workers=4, settings=COARSE,
                      sim_base_error=ERROR, sim_buttons=buttons)
    assert not isinstance(st, Refusal), st
    return st


def _job(c):
    return c.get(f"/jobs/{c.get('/jobs').json()[-1]['id']}").json()


# --------------------------------------------------------------------------- tests


def test_the_touches_of_each_arm_and_the_groups():
    rig = Rig.load(TWO)
    t = tasks_for(rig, "2L", rig.arm_ids)
    assert t == [Task("A", 0), Task("A", 1), Task("A", 2), Task("A", 3), Task("B", 0)]
    assert all(abs(np.hypot(*Task("A", k).lean) - math.radians(25)) < 1e-12 for k in (1, 2, 3))
    assert Task("A", 0).lean == (0.0, 0.0)
    six = Rig.load(SIX)
    assert group_slots(six, (), "rows12") == ("1L", "1R", "2L", "2R")
    assert group_slots(six) == six.arm_ids
    assert isinstance(group_slots(six, (), "row9"), Refusal)
    assert isinstance(group_slots(six, ("4L",)), Refusal)
    assert group_slots(rig) == ("2L", "2R")             # no group "all": every controlled slot


@pytest.mark.slow
def test_marks_on_two_arms_a_cross_then_files_and_rig(tmp_path, capsys):
    st = _station(tmp_path, TWO, {"2L": ["check", "cross"]})
    truth = st.drivers["2L"].person.truth
    c = TestClient(create_app(st))
    t0 = time.perf_counter()
    assert cli.main(["mark", "--poll", "0.05"], http=ClientHttp(c)) == 0
    print(capsys.readouterr().out, f"mark, two arms: {time.perf_counter() - t0:.1f} s wall")
    rep = _job(c)["report"]
    assert rep["state"] == "done" and rep["touches"] == 10, rep["why"]
    assert rep["buttons"] == {"check": 10}           # a cross stays inside the driver
    assert st.drivers["2L"].person.crossed == 1
    rig = Rig.load(st.config_dir)
    for s in ("2L", "2R"):
        T, Tt = rig.T_table_base(s), truth.T[s]
        assert np.linalg.norm(T[:2, 3] - Tt[:2, 3]) < 1e-3, s          # x, y within 1 mm
        yaw = math.atan2(T[1, 0], T[0, 0]) - math.atan2(Tt[1, 0], Tt[0, 0])
        assert abs(yaw) < 1.5e-3, (s, yaw)                              # yaw within 1.5 mrad
        assert rig.calibration_status(s)["base"].startswith("applied")
        assert rep["per_slot"][s]["touches"] == 5
    assert (st.config_dir / "calibration" / "marks.json").exists()
    assert set(rep["marks"]) == {"A", "B"} and rep["rms_mm"] < 1.0
    assert all(p["disagreement_mm"] < 2.0 for p in rep["pairs"])
    assert rig.calibration_status("2L")["pen"].startswith("applied")
    assert st.rig.calibration_status("2L")["base"].startswith("applied")   # reloaded
    assert cli.main(["rig"], http=ClientHttp(c)) == 0
    assert "applied" in capsys.readouterr().out


@pytest.mark.slow
def test_a_skipped_mark_leaves_an_arm_unsolved_and_nothing_written(tmp_path):
    st = _station(tmp_path, TWO, {"2R": ["check"] * 4 + ["circle"]})
    c = TestClient(create_app(st))
    assert cli.main(["mark", "--poll", "0.05"], http=ClientHttp(c)) == 1
    rep = _job(c)["report"]
    assert rep["state"] == "failed" and "the solve did not pass" in rep["why"], rep["why"]
    assert rep["notes"] == ["2R: B orientation 0 skipped"] and rep["buttons"]["circle"] == 1
    assert not list((st.config_dir / "calibration").glob("*.json"))
    for a, d in st.drivers.items():                   # every arm went home all the same
        assert np.max(np.abs(d.state().q - st.rig.park_q(a))) < 1e-9


@pytest.mark.slow
def test_a_failed_hand_over_fails_the_job_with_the_arm_holding(tmp_path):
    st = _station(tmp_path, TWO, {"2L": ["check", "fail: Desk did not answer"]})
    c = TestClient(create_app(st))
    assert cli.main(["mark", "--poll", "0.05"], http=ClientHttp(c)) == 1
    v = _job(c)
    assert v["state"] == "failed" and "Desk did not answer" in v["why"]
    arm = st.drivers["2L"].state()
    assert "moving" not in arm.flags and np.all(arm.qd == 0.0)
    assert np.max(np.abs(arm.q - st.rig.park_q("2L"))) > 0.05       # held at the hover
    assert not list((st.config_dir / "calibration").glob("*.json"))


@pytest.mark.slow
def test_rows12_on_the_six_slot_rig(tmp_path):
    st = _station(tmp_path, SIX)
    c = TestClient(create_app(st))
    t0 = time.perf_counter()
    assert cli.main(["mark", "--group", "rows12", "--poll", "0.05"], http=ClientHttp(c)) == 0
    rep = _job(c)["report"]
    print(f"mark rows12: {rep['touches']} touches, {time.perf_counter() - t0:.1f} s wall")
    assert rep["slots"] == ["1L", "1R", "2L", "2R"]
    assert set(rep["per_slot"]) == {"1L", "1R", "2L", "2R"}
    rig = Rig.load(st.config_dir)
    assert all(rig.calibration_status(s)["base"].startswith("applied")
               for s in ("1L", "1R", "2L", "2R"))
    assert rig.calibration_status("3L")["base"] == "none"


@pytest.mark.slow
def test_a_touch_the_solver_names_as_bad_is_done_once_more(tmp_path, monkeypatch):
    from aris.server import mark
    st = _station(tmp_path, TWO)
    told = []

    def bad(rig, slot, touches):                    # the pivot names 2L's third touch, once
        if slot == "2L" and not told:
            told.append(1)
            return 2
        return None
    monkeypatch.setattr(mark, "_bad_touch", bad)
    c = TestClient(create_app(st))
    assert cli.main(["mark", "--poll", "0.05"], http=ClientHttp(c)) == 0
    rep = _job(c)["report"]
    assert rep["redone"] == ["2L: A orientation 2"] and rep["touches"] == 10
    assert "mark 2L again" in [p["name"] for p in rep["phases"]]
