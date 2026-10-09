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
ARM = "2L"


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
def test_contact_is_where_the_tip_stopped(rig, setup, height):
    """The actual joints stop at the paper while the commanded go on: contact by the lag,
    at the paper within 0.3 mm, whatever the force estimate reads (2026-10-08)."""
    r, pos, kin, paper = _run(rig, setup, height)
    assert r.done and r.rule == "lag", r.why
    assert abs(_height_of(kin, paper, r.q_contact)) < 0.0003
    assert 0.0003 < r.lag_at_contact < 0.0006 and abs(r.lag0) < 1e-9
    assert r.force_at_contact < 8.0
    assert np.abs(pos.q - setup[0]).max() <= 1e-9                  # back at the hover
    assert r.depth_past_end <= max(0.0, -height) + 0.0003


def test_contact_past_the_planned_end_goes_on_straight_and_slowly(rig, setup):
    r, pos, kin, paper = _run(rig, setup, -0.012)                  # paper 12 mm low
    assert r.done and r.rule == "lag", r.why
    assert r.depth_past_end == pytest.approx(0.012, abs=0.0003)
    assert abs(_height_of(kin, paper, r.q_contact)) < 0.0003
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


@pytest.mark.parametrize("height", [0.0, -0.012])
def test_a_controller_that_trails_by_half_a_millimetre_is_no_contact(rig, setup, height):
    """The arm trails the commanded descent by 0.5 mm all the way down (0.1 s at 5 mm/s):
    the baseline in the air takes it out, no false contact, and the paper is still found
    where it is (also on the slower extension)."""
    q0, m = setup
    kin = Kinematics.of(rig, ARM)
    paper = FakePaper(kin, rig.paper(ARM), height=height, bias=2.3)
    pos = SimPositionArm(q0, paper, trail_s=0.1)
    r = touch(m, pos, kin, TouchSettings())
    assert r.done and r.rule == "lag", r.why
    assert r.lag_baseline == pytest.approx(0.0005, abs=0.00005)
    assert abs(_height_of(kin, paper, r.q_contact)) < 0.0003
    assert np.abs(pos.q - q0).max() <= 1e-9


class Spiking(SimPositionArm):
    """The force estimate reads `extra` N more at readings `at` of the first flight (the
    estimate's own errors: holder mass, friction, the start jolt)."""

    def __init__(self, q0, paper, extra, at):
        super().__init__(q0, paper)
        self.extra, self.at = extra, at

    def fly(self, traj, watch):
        n, ticks, first = self.paper.n, [0], not self.flights

        def spiked(q, F, t):
            ticks[0] += 1
            return watch(q, F + (self.extra * n if first and ticks[0] in self.at else 0.0), t)
        return super().fly(traj, spiked)


def test_a_force_spike_without_lag_is_no_contact(rig, setup):
    q0, m = setup
    kin = Kinematics.of(rig, ARM)
    pos = Spiking(q0, FakePaper(kin, rig.paper(ARM), height=-0.05, bias=2.3), 6.0,
                  set(range(1, 60)))
    r = touch(m, pos, kin, TouchSettings())
    assert not r.done and r.why.startswith("no contact"), r.why


def test_a_force_over_the_cap_is_a_contact_at_that_reading(rig, setup):
    q0, m = setup
    kin = Kinematics.of(rig, ARM)
    pos = Spiking(q0, FakePaper(kin, rig.paper(ARM), height=-0.05, bias=2.3), 10.0, {200})
    r = touch(m, pos, kin, TouchSettings())
    assert r.done and r.rule == "force cap", r.why
    assert r.force_at_contact > 8.0 and abs(r.lag_at_contact) < 1e-4
    from aris.kernel.retime import sample
    descent = pos.flights[0]
    q_200 = sample(descent, [descent.t[0] + 199 * 0.004])[0][0]     # the 200th reading
    assert np.allclose(r.q_contact, q_200, atol=1e-9)
    assert np.abs(pos.q - q0).max() <= 1e-9                         # the way back flown


def test_refusals_before_moving(rig, setup):
    q0, m = setup
    import dataclasses
    r = touch(dataclasses.replace(m, extra_depth=0.05), SimPositionArm(q0, None), Kinematics.of(rig, ARM),
              TouchSettings())
    assert not r.done and "cap" in r.why
    class Silent(SimPositionArm):                    # no force signal at all
        def forces(self, seconds):
            return []
    pos = Silent(q0, FakePaper(Kinematics.of(rig, ARM), rig.paper(ARM)))
    r = touch(m, pos, Kinematics.of(rig, ARM), TouchSettings())
    assert not r.done and r.why.startswith("no_tare") and not pos.flights
    r, pos, _, _ = _run(rig, setup, 0.0, bias=9.0)  # a large air reading is no reason to stop
    assert r.done and r.rule == "lag", r.why


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
    assert abs(_height_of(arm.kin, arm.paper, np.array(contact["q"]))) < 0.0003
    done = next(r for r in rows if r["event"] == "motion done")
    assert np.allclose(done["q"], q0, atol=1e-9)
