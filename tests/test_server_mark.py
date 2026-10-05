"""The mark job (aris/server/mark.py) on simulated arms with a simulated person.

The solver (`aris.calib.solve_marks`) and the mark files are the calib agent's; until they land
these tests stand in a small solver of their own (a rigid x, y, yaw fit per slot through its
touches at the marks' nominal positions, which is where the simulated world puts them) and
simple writers in the file format rig.py reads.
"""
from __future__ import annotations

import datetime
import json
import math
import shutil
import time
from pathlib import Path
from types import SimpleNamespace

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


# --------------------------------------------------------------------------- stand-ins


def _stub_solve(rig, touches, known):
    """Per slot: x, y and yaw by a 2D rigid fit of the believed upright tips onto the marks,
    z by the mean height; the pivot's leans give the residuals."""
    slots, res, marks = {}, [0.0] * len(touches), {}
    for s in dict.fromkeys(t["slot"] for t in touches):
        mine = [(k, t) for k, t in enumerate(touches) if t["slot"] == s and not t["skipped"]]
        names = {t["mark"] for _, t in mine}
        if len(names) < 2:
            return SimpleNamespace(passed=False, why=f"slot {s}: one mark only, needs a partner",
                                   slots={}, touch_residuals=res, bad_touch=None, marks={})
        arm, T0 = rig.arm(s), rig.T_table_base(s)
        P = np.array([(T0 @ np.r_[arm.tip(np.asarray(t["q"])[None])[0], 1.0])[:3]
                      for _, t in mine])
        M = np.array([np.r_[rig.marks[t["mark"]][0], rig.paper_z] for _, t in mine])
        pc, mc = P[:, :2].mean(0), M[:, :2].mean(0)
        H = (P[:, :2] - pc).T @ (M[:, :2] - mc)
        U, _, Vt = np.linalg.svd(H)
        R2 = Vt.T @ U.T
        if np.linalg.det(R2) < 0:
            Vt[-1] *= -1
            R2 = Vt.T @ U.T
        D = np.eye(4)
        D[:2, :2] = R2
        D[:2, 3] = mc - R2 @ pc
        D[2, 3] = float(np.mean(M[:, 2] - P[:, 2]))
        T = D @ T0
        for (k, t), p in zip(mine, P):
            res[k] = float(np.linalg.norm((D @ np.r_[p, 1.0])[:3] - np.r_[rig.marks[t["mark"]][0],
                                                                      rig.paper_z]))
        slots[s] = SimpleNamespace(slot=s, T_table_base=T, tip_hand=arm.tool.tip_hand)
        marks.update({t["mark"]: rig.marks[t["mark"]][0] for _, t in mine})
    return SimpleNamespace(passed=True, why="", slots=slots, touch_residuals=res,
                           bad_touch=None, marks=marks)


def _merge(cfg, slot, part, content):
    p = Path(cfg) / "calibration" / f"{slot}.json"
    d = json.loads(p.read_text()) if p.exists() else {}
    d.update(slot=slot, **{part: content})
    p.write_text(json.dumps(d))
    return p


def _stubs(monkeypatch, rig_pen):
    import aris.calib as calib
    from aris.calib import files
    today = datetime.date.today().isoformat()
    monkeypatch.setattr(calib, "solve_marks", _stub_solve, raising=False)
    monkeypatch.setattr(files, "write_base", lambda r, cfg: _merge(cfg, r.slot, "base", dict(
        passed=True, date=today, method="marks",
        T_table_base=np.asarray(r.T_table_base).tolist())), raising=False)
    monkeypatch.setattr(files, "write_pen", lambda r, cfg: _merge(cfg, r.slot, "pen", dict(
        passed=True, date=today, pen=rig_pen, tip_hand_m=np.asarray(r.tip_hand).tolist())),
        raising=False)

    def write_marks(sol, cfg):
        p = Path(cfg) / "calibration" / "marks.json"
        p.write_text(json.dumps({"marks": {n: dict(xy_m=list(map(float, xy)), state="solved",
                                                   date=today, residual_mm=0.0)
                                           for n, xy in sol.marks.items()}}))
        return p
    monkeypatch.setattr(files, "write_marks", write_marks, raising=False)


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
    six = Rig.load(SIX)
    assert group_slots(six, (), "rows12") == ("1L", "1R", "2L", "2R")
    assert group_slots(six) == six.arm_ids
    assert isinstance(group_slots(six, (), "row9"), Refusal)
    assert isinstance(group_slots(six, ("4L",)), Refusal)
    assert group_slots(rig) == ("2L", "2R")             # no group "all": every controlled slot


@pytest.mark.slow
def test_marks_on_two_arms_a_cross_then_files_and_rig(tmp_path, monkeypatch, capsys):
    st = _station(tmp_path, TWO, {"2L": ["check", "cross"]})
    _stubs(monkeypatch, st.rig.pen_name)
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
    assert set(rep["marks"]) == {"A", "B"} and rep["marks"]["A"]["between_arms_mm"] < 2.0
    assert st.rig.calibration_status("2L")["base"].startswith("applied")   # reloaded
    assert cli.main(["rig"], http=ClientHttp(c)) == 0
    assert "applied" in capsys.readouterr().out


@pytest.mark.slow
def test_a_skipped_mark_leaves_an_arm_unsolved_and_nothing_written(tmp_path, monkeypatch):
    st = _station(tmp_path, TWO, {"2R": ["check"] * 4 + ["circle"]})
    _stubs(monkeypatch, st.rig.pen_name)
    c = TestClient(create_app(st))
    assert cli.main(["mark", "--poll", "0.05"], http=ClientHttp(c)) == 1
    rep = _job(c)["report"]
    assert rep["state"] == "failed" and "2R" in rep["why"]
    assert rep["notes"] == ["2R: B orientation 0 skipped"] and rep["buttons"]["circle"] == 1
    assert not list((st.config_dir / "calibration").glob("*.json"))
    for a, d in st.drivers.items():                   # every arm went home all the same
        assert np.max(np.abs(d.state().q - st.rig.park_q(a))) < 1e-9


@pytest.mark.slow
def test_a_failed_hand_over_fails_the_job_with_the_arm_holding(tmp_path, monkeypatch):
    st = _station(tmp_path, TWO, {"2L": ["check", "fail: Desk did not answer"]})
    _stubs(monkeypatch, st.rig.pen_name)
    c = TestClient(create_app(st))
    assert cli.main(["mark", "--poll", "0.05"], http=ClientHttp(c)) == 1
    v = _job(c)
    assert v["state"] == "failed" and "Desk did not answer" in v["why"]
    arm = st.drivers["2L"].state()
    assert "moving" not in arm.flags and np.all(arm.qd == 0.0)
    assert np.max(np.abs(arm.q - st.rig.park_q("2L"))) > 0.05       # held at the hover
    assert not list((st.config_dir / "calibration").glob("*.json"))


@pytest.mark.slow
def test_rows12_on_the_six_slot_rig(tmp_path, monkeypatch):
    st = _station(tmp_path, SIX)
    _stubs(monkeypatch, st.rig.pen_name)
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
    import aris.calib as calib
    st = _station(tmp_path, TWO)
    _stubs(monkeypatch, st.rig.pen_name)
    told = []

    def solve(rig, touches, known):
        sol = _stub_solve(rig, touches, known)
        if {t["slot"] for t in touches} == {"2L"} and not told:     # 2L's own solve, once
            told.append(1)
            sol.bad_touch = 2
        return sol
    monkeypatch.setattr(calib, "solve_marks", solve, raising=False)
    c = TestClient(create_app(st))
    assert cli.main(["mark", "--poll", "0.05"], http=ClientHttp(c)) == 0
    v = _job(c)
    rep = v["report"]
    assert rep["redone"] == ["2L: A orientation 2"] and rep["touches"] == 10
    assert "mark 2L again" in [p["name"] for p in rep["phases"]]
