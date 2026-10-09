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
    """The actual joints stop at the paper while the commanded go on: contact by the stall
    rule, at the paper within 0.3 mm, whatever the force estimate reads."""
    r, pos, kin, paper = _run(rig, setup, height)
    assert r.done and r.rule in ("lag", "stall"), r.why
    assert abs(_height_of(kin, paper, r.q_contact)) < 0.0003
    assert 0.0004 <= r.lag_at_contact < 0.0012 and abs(r.lag0) < 1e-9
    assert r.force_at_contact < 8.0
    assert np.abs(pos.q - setup[0]).max() <= 1e-9                  # back at the hover
    assert r.depth_past_end <= max(0.0, -height) + 0.0003
    st = r.stats                                                    # the touch's own numbers
    assert st["readings"] > 100 and st["air_noise_mm"] < 0.05
    assert st["threshold_mm"] == pytest.approx(0.4) and st["stop_depth_mm"] > 50.0
    assert len(r.trace) == st["readings"]


def test_contact_past_the_planned_end_goes_on_straight_and_slowly(rig, setup):
    r, pos, kin, paper = _run(rig, setup, -0.012)                  # paper 12 mm low
    assert r.done and r.rule in ("lag", "stall"), r.why
    assert r.depth_past_end == pytest.approx(0.012, abs=0.0003)
    assert abs(_height_of(kin, paper, r.q_contact)) < 0.0003
    ext = pos.flights[1]
    tips = kin.tip(ext.q)
    speed = np.linalg.norm(np.diff(tips, axis=0), axis=1) / np.diff(ext.t)
    assert speed.max() <= 0.002 * 1.05
    lateral = (tips - tips[0]) - np.outer((tips - tips[0]) @ kin.down, kin.down)
    assert np.abs(lateral).max() < 2e-5                             # straight on
    assert np.abs(pos.q - setup[0]).max() <= 1e-9


@pytest.mark.parametrize("above", [0.004, 0.0027, 0.0015, 0.0005])
def test_a_paper_met_while_the_descent_slows_down_is_a_contact(rig, setup, above):
    """1L, 2026-10-09: the paper stood 2.7 mm above the planned end, the pen met it while the
    commanded descent was already slowing to its stop, the force cap stopped the arm at 8 N
    and the rule that compares advances over a window called it 'in the air' (0.18 mm short):
    the calibration failed on a real contact.  The lag against the delayed trajectory sees it."""
    q0, m = setup
    kin = Kinematics.of(rig, ARM)
    paper = FakePaper(kin, rig.paper(ARM), height=above, bias=2.3)
    for pos in (SimPositionArm(q0, paper), SimPositionArm(q0, paper, trail_s=0.3)):
        r = touch(m, pos, kin, TouchSettings())
        assert r.done and r.rule in ("lag", "stall"), (r.why, r.rule, r.stats)
        assert abs(_height_of(kin, paper, r.q_contact)) < 0.0003
        assert r.force_at_contact < 8.0
        assert np.abs(pos.q - q0).max() <= 1e-9


def test_the_way_back_up_is_slow(rig, setup):
    """The retraction flew at the free-flight speed (Pete: "crazy fast"); now 30 mm/s."""
    r, pos, kin, paper = _run(rig, setup, 0.0)
    assert r.done
    back = pos.flights[-1]
    tips = kin.tip(back.q)
    speed = np.linalg.norm(np.diff(tips, axis=0), axis=1) / np.diff(back.t)
    assert speed.max() <= 0.030 * 1.05
    assert back.t[-1] - back.t[0] >= 0.055 / 0.030           # 60 mm up, not in half a second


@pytest.mark.parametrize("below", [0.0003, 0.001])
def test_a_paper_just_past_the_planned_end_is_found_by_the_extension(rig, setup, below):
    """The extension starts from rest right above (or on) the paper: its short blind start
    presses the pen in a little, never more than a drawing's press, and the contact is still
    the paper."""
    r, pos, kin, paper = _run(rig, setup, -below)
    assert r.done, r.why
    assert abs(_height_of(kin, paper, r.q_contact)) < 0.0003
    deepest = max(float(kin.tip(f.q[-1])[0] @ kin.down) for f in pos.flights[:2])
    assert r.stats["stop_commanded_mm"] is not None
    assert r.force_at_contact < 8.0 or r.rule == "force cap"


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
    a constant lag is no contact, and the paper is still found where it is (also on the
    slower extension)."""
    q0, m = setup
    kin = Kinematics.of(rig, ARM)
    paper = FakePaper(kin, rig.paper(ARM), height=height, bias=2.3)
    pos = SimPositionArm(q0, paper, trail_s=0.1)
    r = touch(m, pos, kin, TouchSettings())
    assert r.done and r.rule in ("lag", "stall"), r.why
    assert r.lag_baseline == pytest.approx(0.0005, abs=0.00005)
    assert abs(_height_of(kin, paper, r.q_contact)) < 0.0003
    assert np.abs(pos.q - q0).max() <= 1e-9


class Field(SimPositionArm):
    """The arm as it was on 2026-10-09: what the detector READS differs from the perfect arm.
    `off`: the joints read this far (rad, joint 2) from the commanded ones when the flight
    starts and settle with time constant `tau` (the arm had not come to rest at the hover:
    1L stood 0.65 mm off); `late`: the reading's clock runs this many seconds ahead of the
    arm (the goal starts after it was sent); `jitter`: noise on every joint reading (rad);
    `stick`: the arm sticks for this many seconds every 0.5 s and then catches up (friction
    at 5 mm/s)."""

    def __init__(self, q0, paper, off=0.0, tau=0.4, late=0.0, jitter=0.0, stick=0.0):
        super().__init__(q0, paper, trail_s=late)
        self.off, self.tau, self.jitter, self.stick = off, tau, jitter, stick
        self.rng2 = np.random.default_rng(1)

    def fly(self, traj, watch):
        t0, first, held = traj.t[0], not self.flights, {}

        def seen(q, F, t):
            q = np.array(q, float)
            if self.stick and (t - t0) % 0.5 < self.stick and "q" in held:
                q = held["q"]                              # stuck: the reading does not move
            else:
                held["q"] = q.copy()
            if first:
                q[1] += self.off * np.exp(-(t - t0) / self.tau)
            q = q + self.jitter * self.rng2.standard_normal(7)
            return watch(q, F, t)
        return super().fly(traj, seen)


FIELD = [dict(off=0.002), dict(off=-0.002), dict(off=0.004, tau=1.0), dict(late=0.3),
         dict(jitter=2e-5), dict(stick=0.08), dict(off=0.002, late=0.2, jitter=2e-5, stick=0.05)]


@pytest.mark.parametrize("field", FIELD)
def test_nothing_in_the_air_is_a_contact(rig, setup, field):
    """An arm still settling when the descent starts, a late start, noisy joints, an arm that
    sticks and slips: with the paper out of reach there is NO contact (on 2026-10-09 the old
    rule called one 1.2 s into the descent, twice, 54 and 143 mm above the table)."""
    q0, m = setup
    kin = Kinematics.of(rig, ARM)
    pos = Field(q0, FakePaper(kin, rig.paper(ARM), height=-0.05, bias=2.3), **field)
    r = touch(m, pos, kin, TouchSettings())
    assert not r.done and r.why.startswith("no contact within 20 mm"), (r.why, r.rule, r.stats)


@pytest.mark.parametrize("field", FIELD)
@pytest.mark.parametrize("height", [0.02, 0.0, -0.012])
def test_the_paper_is_still_found_on_such_an_arm(rig, setup, field, height):
    q0, m = setup
    kin = Kinematics.of(rig, ARM)
    paper = FakePaper(kin, rig.paper(ARM), height=height, bias=2.3)
    pos = Field(q0, paper, **field)
    r = touch(m, pos, kin, TouchSettings())
    assert r.done, (r.why, r.stats)
    # within half a millimetre of the paper (the noise and the stick are in the reading too)
    assert abs(_height_of(kin, paper, r.q_contact)) < 0.0005, r.stats


class Creeping(SimPositionArm):
    """The arm falls further behind its command as it descends, in steps: 0.45 mm within
    0.7 s halfway down and 0.35 mm more near the end (1L, 2026-10-09, in free air)."""

    def fly(self, traj, watch):
        from aris.kernel.retime import sample
        t0, first = traj.t[0], not self.flights

        def behind(t):
            u = t - t0
            return 0.09 * np.clip((u - 6.5) / 0.7, 0.0, 1.0) + 0.07 * np.clip((u - 10.5) / 0.6, 0.0, 1.0)

        def seen(q, F, t):
            if first:
                q = sample(traj, [max(t0, t - behind(t))])[0][0]
            return watch(q, F, t)
        return super().fly(traj, seen)


def test_a_lag_that_creeps_up_in_the_air_is_no_contact(rig, setup):
    """Job 017, touch 11: the lag crept up 0.7 mm over 11 s of free descent and a fixed level
    called a contact 26 mm above the table at 0.7 N.  The lag rule follows its own recent
    level; a real paper under the same arm is still found."""
    q0, m = setup
    kin = Kinematics.of(rig, ARM)
    pos = Creeping(q0, FakePaper(kin, rig.paper(ARM), height=-0.05, bias=2.3))
    r = touch(m, pos, kin, TouchSettings())
    assert not r.done and r.why.startswith("no contact within 20 mm"), (r.why, r.rule, r.stats)
    assert r.stats["largest_residual_mm"] < 0.8
    paper = FakePaper(kin, rig.paper(ARM), height=0.0, bias=2.3)
    r = touch(m, Creeping(q0, paper), kin, TouchSettings())
    assert r.done and abs(_height_of(kin, paper, r.q_contact)) < 0.0005, (r.why, r.stats)


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


def test_a_force_spike_without_a_stall_is_no_contact(rig, setup):
    q0, m = setup
    kin = Kinematics.of(rig, ARM)
    pos = Spiking(q0, FakePaper(kin, rig.paper(ARM), height=-0.05, bias=2.3), 6.0,
                  set(range(1, 60)))
    r = touch(m, pos, kin, TouchSettings())
    assert not r.done and r.why.startswith("no contact"), r.why
    # over the cap, but for single readings only: still nothing
    pos = Spiking(q0, FakePaper(kin, rig.paper(ARM), height=-0.05, bias=2.3), 12.0,
                  set(range(100, 2000, 7)))
    r = touch(m, pos, kin, TouchSettings())
    assert not r.done and r.why.startswith("no contact"), r.why


def test_a_force_that_starts_with_the_motion_is_not_the_paper(rig, setup):
    """The estimate jumps by 5 N when the arm starts to move and stays there (friction): the
    zero taken in the air at speed takes it out, so the cap is not 3 N away all the way down."""
    q0, m = setup
    kin = Kinematics.of(rig, ARM)
    paper = FakePaper(kin, rig.paper(ARM), height=0.0, bias=2.3)
    pos = Spiking(q0, paper, 5.0, set(range(1, 100000)))
    r = touch(m, pos, kin, TouchSettings())
    assert r.done and r.rule in ("lag", "stall"), r.why
    assert r.stats["force_zero_moving_n"] == pytest.approx(r.air_zero + 5.0, abs=0.2)
    assert abs(_height_of(kin, paper, r.q_contact)) < 0.0003


def test_a_force_over_the_cap_in_the_air_stops_the_arm_and_is_no_contact(rig, setup):
    q0, m = setup
    kin = Kinematics.of(rig, ARM)
    pos = Spiking(q0, FakePaper(kin, rig.paper(ARM), height=-0.05, bias=2.3), 10.0,
                  set(range(200, 260)))
    r = touch(m, pos, kin, TouchSettings())
    assert not r.done and r.rule == "force cap", (r.why, r.rule)
    assert r.why.startswith("stopped by the force cap in the air") and "not a contact" in r.why
    assert r.force_at_contact > 8.0 and r.q_contact is None
    assert len(pos.flights) == 2                                    # the descent, the way back
    assert np.abs(pos.q - q0).max() <= 1e-9


def test_a_stiff_paper_trips_the_force_cap_and_that_is_the_contact(rig, setup):
    """8 N reached half a millimetre in, before the stall rule has its window: the cap stops
    the arm, the tip had stopped following, so it is the paper."""
    q0, m = setup
    kin = Kinematics.of(rig, ARM)
    paper = FakePaper(kin, rig.paper(ARM), height=0.0, k=15000.0, bias=2.3)
    pos = SimPositionArm(q0, paper)
    r = touch(m, pos, kin, TouchSettings())
    assert r.done and r.rule == "force cap", (r.why, r.rule)
    assert r.force_at_contact > 8.0 and r.lag_at_contact >= 0.0004
    assert abs(_height_of(kin, paper, r.q_contact)) < 0.0003
    assert np.abs(pos.q - q0).max() <= 1e-9


def test_the_descent_waits_for_the_arm_to_stand_still(rig, setup):
    q0, m = setup
    kin = Kinematics.of(rig, ARM)

    class Settling(SimPositionArm):
        asked = None

        def still(self, rest_s, rest_m, wait_s):
            Settling.asked = (rest_s, rest_m, wait_s, len(self.flights))
            return self.verdict

    pos = Settling(q0, FakePaper(kin, rig.paper(ARM), height=0.0, bias=2.3))
    pos.verdict = ""
    r = touch(m, pos, kin, TouchSettings())
    assert r.done and Settling.asked == (0.5, 0.00005, 6.0, 0)      # asked before the descent
    assert r.stats["not_at_rest"] is None
    pos = Settling(q0, FakePaper(kin, rig.paper(ARM), height=0.0, bias=2.3))
    pos.verdict = "the tip still moved 0.31 mm in 0.5 s after 6 s at the hover"
    r = touch(m, pos, kin, TouchSettings())     # an arm that never rests: flown, and said
    assert r.done and r.stats["not_at_rest"] == pos.verdict


def test_the_cap_on_the_extra_depth_allows_the_planner_its_deepest_touch():
    """2026-10-09: the planner asked 40 mm for the first touch of an unmeasured slot and the
    robot PC's cap was 30: every first touch was refused.  The code's default and the
    repository's site file both stand above what the planner asks."""
    import json
    from aris.server import calibrate
    deepest = max(calibrate.UNCAL_DEPTH, calibrate.CalibSettings().extra_depth)
    site = json.loads((CONFIG.parent / "robot" / "site.json").read_text())["touch"]
    assert TouchSettings().extra_max >= deepest
    assert TouchSettings.from_site(site).extra_max >= deepest


def test_refusals_before_moving(rig, setup):
    q0, m = setup
    import dataclasses
    r = touch(dataclasses.replace(m, extra_depth=0.06), SimPositionArm(q0, None), Kinematics.of(rig, ARM),
              TouchSettings())
    assert not r.done and "cap" in r.why
    class Silent(SimPositionArm):                    # no force signal at all
        def forces(self, seconds):
            return []
    pos = Silent(q0, FakePaper(Kinematics.of(rig, ARM), rig.paper(ARM)))
    r = touch(m, pos, Kinematics.of(rig, ARM), TouchSettings())
    assert not r.done and r.why.startswith("no_tare") and not pos.flights
    r, pos, _, _ = _run(rig, setup, 0.0, bias=9.0)  # a large air reading is no reason to stop
    assert r.done and r.rule in ("lag", "stall"), r.why


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
