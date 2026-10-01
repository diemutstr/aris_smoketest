"""The calibration touch under position control, on a simulated arm and a fake paper."""
import math
import threading
from pathlib import Path

import numpy as np
import pytest

from aris.execute import EventLog, Executor, Job
from aris.rig import Rig
from aris.types import Phase
from aris_robot.simarm import SimTouchArm
from aris_robot.touch import FakePaper, Kinematics, TouchSettings, touch
from sim_touch import SimPositionArm, hover_q, touch_motion

CONFIG = Path(__file__).resolve().parents[2] / "config"
ARM = 31


@pytest.fixture(scope="module")
def rig():
    return Rig.load(CONFIG)


@pytest.fixture(scope="module")
def setup(rig):
    q0 = hover_q(rig, ARM)
    return q0, touch_motion(rig, ARM, q0)


def _run(rig, setup, height, k=5000.0, bias=2.3, s=TouchSettings()):
    q0, m = setup
    kin = Kinematics.of(rig, ARM)
    paper = FakePaper(kin, rig.paper(ARM), height=height, k=k, bias=bias)
    pos = SimPositionArm(q0, paper)
    return touch(m, pos, kin, s), pos, kin, paper


def _height_of(kin, paper, q):
    """How far above the fake paper's surface the tip is (m; negative: pressed in)."""
    return float(kin.tip(q)[0] @ paper.n) - paper.c


@pytest.mark.parametrize("height", [0.005, 0.0, -0.004])
def test_contact_within_the_plan_reads_where_the_paper_is(rig, setup, height):
    r, pos, kin, paper = _run(rig, setup, height)
    assert r.done, r.why
    assert r.air_zero == pytest.approx(2.3, abs=0.05)
    # 1 N at 5000 N/m is 0.2 mm in; the reading is taken at the onset
    assert -0.0004 < -_height_of(kin, paper, r.q_contact) < 0.0004
    assert np.abs(pos.q - setup[0]).max() <= 1e-9                  # back at the hover
    assert r.depth_past_end <= max(0.0, -height) + 0.0005        # 1 N lies 0.2 mm in


def test_contact_past_the_planned_end_goes_on_straight_and_slowly(rig, setup):
    r, pos, kin, paper = _run(rig, setup, -0.012)                  # paper 12 mm low
    assert r.done, r.why
    assert r.depth_past_end == pytest.approx(0.012, abs=0.0005)
    assert abs(_height_of(kin, paper, r.q_contact)) < 0.0004
    ext = pos.flights[1]
    tips = kin.tip(ext.q)
    speed = np.linalg.norm(np.diff(tips, axis=0), axis=1) / np.diff(ext.t)
    assert speed.max() <= 0.002 * 1.05
    lateral = (tips - tips[0]) - np.outer((tips - tips[0]) @ kin.down, kin.down)
    assert np.abs(lateral).max() < 2e-5                             # straight on
    assert np.abs(pos.q - setup[0]).max() <= 1e-9


def test_no_contact_gives_up_after_the_extra_depth_and_comes_back(rig, setup):
    r, pos, kin, paper = _run(rig, setup, -0.05)                   # paper 50 mm low
    assert not r.done and r.why.startswith("no contact within 20 mm")
    deepest = max(float(kin.tip(f.q[-1])[0] @ kin.down) for f in pos.flights[:2])
    start = float(kin.tip(setup[0])[0] @ kin.down)
    assert deepest - start == pytest.approx(0.06 + 0.02, abs=1e-5)  # timing rounds corners
    assert np.abs(pos.q - setup[0]).max() <= 1e-9


def test_a_hard_hit_stops_and_holds(rig, setup):
    r, pos, kin, paper = _run(rig, setup, 0.005, k=5e6)            # steel, not paper
    assert not r.done and r.held and "above the cap" in r.why
    assert len(pos.flights) == 1                                   # no way back flown
    assert np.abs(pos.q - setup[0]).max() > 0.01


def test_refusals_before_moving(rig, setup):
    q0, m = setup
    import dataclasses
    r = touch(dataclasses.replace(m, extra_depth=0.05), SimPositionArm(q0, None), Kinematics.of(rig, ARM),
              TouchSettings())
    assert not r.done and "cap" in r.why
    r, pos, _, _ = _run(rig, setup, 0.0, bias=9.0)
    assert not r.done and r.why.startswith("tare_too_large") and not pos.flights


def test_the_executor_logs_the_contact_row_with_the_joints(rig, setup, tmp_path):
    class Passed:
        passed, tightest, min_clearance, min_clearance_at = True, "test", 0.1, "test"

        def get(self, name):
            raise KeyError(name)

    q0, m = setup
    job = Job.create(tmp_path / "cal", {})
    q = job.queue("touch", ARM)
    assert q.append(m, Passed()) == 0
    q.close()
    arm = SimTouchArm(rig, ARM, q0, speed=math.inf, paper_m=0.003)
    log = EventLog(job.log_path)
    run = Executor(ARM, arm, log, rig).run(q, threading.Event())
    assert run.status == "finished" and run.done == 1, run.why
    rows = log.read()
    contact = next(r for r in rows if r["event"] == "contact")
    assert len(contact["q"]) == 7
    assert abs(_height_of(arm.kin, arm.paper, np.array(contact["q"]))) < 0.0004
    done = next(r for r in rows if r["event"] == "motion done")
    assert np.allclose(done["q"], q0, atol=1e-9)
