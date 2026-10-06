"""The real driver's pen logic (tare, streaming, contact, ramps, guard, holds) against a fake
arm node standing in for ROS (fake_ros.py).  Real time: each motion takes what it takes."""
from pathlib import Path

import numpy as np
import pytest

import fake_ros
from aris.kernel.retime import retime
from aris.rig import Rig
from aris.types import JointPath, Motion, Piece
from aris_robot import site as site_mod

fake_ros.install()
from aris_robot import driver as D  # noqa: E402

ROBOT = Path(__file__).resolve().parents[1]
CONFIG = ROBOT.parent / "config"
BIAS = 2.3                          # N, the air reading along the normal


@pytest.fixture(scope="module")
def rig():
    return Rig.load(CONFIG)


@pytest.fixture(scope="module")
def site():
    return site_mod.load(ROBOT / "site.json")


class Paper:
    """Force along the paper normal: the air bias, plus the paper once touched."""

    def __init__(self, normal, touch_at=None, extra=0.0):
        self.n, self.touch_at, self.extra = np.asarray(normal), touch_at, extra

    def __call__(self, t, f_ff):
        on = self.touch_at is not None and t >= self.touch_at
        press = (0.8 + 0.8 * np.linalg.norm(f_ff) + self.extra) if on else 0.0
        return (BIAS + press) * self.n


def _chain(rig, arm_id, kinds, scale=0.08):
    p, arm = rig.park_q(arm_id), rig.arm(arm_id)
    step = scale * np.array([1.0, -1.0, 0.5, 1.0, 0.0, -1.0, 1.0])
    qs = [p + i * step for i in range(len(kinds) + 1)]
    out = []
    for i, k in enumerate(kinds):
        tr = retime(JointPath(np.array([qs[i], qs[i + 1]])), arm.limits, rig.rules())
        tip = arm.tip(tr.q) if k == "draw" else None
        out.append(Motion(k, tr, Piece("a", 0, 0.01) if k == "draw" else None, tip, 1.0))
    return out


def _arm(monkeypatch, rig, site, q0, model=None, **kw):
    node = fake_ros.FakeArmNode(q0, rig.paper("2L").normal, model, **kw)
    monkeypatch.setattr(D, "ArmNode", lambda site_arm, names: node)
    arm = D.RosArm(site, rig, "2L")
    arm._sign = 1.0                                 # the fake paper reads positive
    arm.set_job(rig.pen(), "impedance")             # these tests are about mode B
    return arm, node


def _forces(node, stream):
    f = [c.f for c in node.published if c.stream == stream]
    return np.concatenate(f) if f else np.zeros((0, 3))


def test_lower_draw_lift_press_only_while_drawing(monkeypatch, rig, site):
    lower, draw, lift = _chain(rig, "2L", ("lower", "draw", "lift"))
    paper = Paper(rig.paper("2L").normal)
    arm, node = _arm(monkeypatch, rig, site, lower.q_start, paper)
    paper.touch_at = 0.6 * float(lower.traj.t[-1])
    r = arm.draw(lower)
    assert r.done, r.why
    rep = arm.last_report
    assert arm._zero == pytest.approx(BIAS, abs=1e-9)
    assert paper.touch_at <= rep["contact_at"] <= paper.touch_at + 0.05
    assert np.all(_forces(node, rep["stream"]) == 0.0)            # no force while lowering
    paper.touch_at = 0.0
    r = arm.draw(draw)
    assert r.done, r.why
    f = _forces(node, arm.last_report["stream"]) @ -rig.paper("2L").normal
    assert f[0] == 0.0 and f.max() == pytest.approx(1.0)          # ramped in, to intensity 1
    # the fake paper reads 0.8 N + 0.8 x the fed force, 1.6 N at 1 N fed: the servo (on by
    # default, 1 s) trims the feed down toward the 1.0 N setpoint
    press = arm._press
    assert 0.5 < press < 1.0
    r = arm.draw(lift)
    assert r.done, r.why
    f = _forces(node, arm.last_report["stream"]) @ -rig.paper("2L").normal
    assert f[0] == pytest.approx(press) and f[-1] == 0.0         # ramped out
    assert np.all(np.diff(f) <= 1e-12)


def test_a_force_over_the_cap_holds_the_arm(monkeypatch, rig, site):
    lower, draw = _chain(rig, "2L", ("lower", "draw"))
    paper = Paper(rig.paper("2L").normal, touch_at=None)
    arm, node = _arm(monkeypatch, rig, site, lower.q_start, paper)
    assert arm.draw(lower).done
    paper.touch_at, paper.extra = 0.0, 4.0                         # something pushes back hard
    r = arm.draw(draw)
    assert not r.done and "above the cap" in r.why
    assert node.holding


def test_a_stream_that_runs_dry_fails_the_motion(monkeypatch, rig, site):
    (draw,) = _chain(rig, "2L", ("draw",))
    arm, node = _arm(monkeypatch, rig, site, draw.q_start, drop_after=0.15)
    arm._zero = BIAS
    r = arm.draw(draw)
    assert not r.done and "starved" in r.why


def test_a_landing_refuses_an_implausible_air_reading(monkeypatch, rig, site):
    (lower,) = _chain(rig, "2L", ("lower",))
    arm, _ = _arm(monkeypatch, rig, site, lower.q_start,
                  lambda t, f: 9.0 * rig.paper("2L").normal)
    r = arm.draw(lower)
    assert not r.done and r.why.startswith("tare_too_large")


def test_stop_then_recover(monkeypatch, rig, site):
    import threading
    (draw,) = _chain(rig, "2L", ("draw",), scale=0.2)
    arm, node = _arm(monkeypatch, rig, site, draw.q_start)
    arm._zero = BIAS
    threading.Timer(0.2, arm.stop).start()
    r = arm.draw(draw)
    assert not r.done and r.why == "stopped"
    assert not arm.state().ok and "stopped" in arm.state().flags
    assert node.holding
    assert not arm.draw(draw).done                                 # refused until recovered
    rec = arm.recover()
    assert rec.done, rec.why
    # the reflex is cleared first, then the hardware component comes back
    assert node.recovery_steps == ["error recovery", "hardware FrankaHardwareInterface"]
    assert arm.state().ok and node.active == {"fr3_arm_controller"}


def test_a_stream_not_starting_at_the_arm_is_refused(monkeypatch, rig, site):
    (draw,) = _chain(rig, "2L", ("draw",))
    arm, node = _arm(monkeypatch, rig, site, draw.q_start)
    node.q_d = draw.q_start + 0.004                                # within the driver's 5 mrad
    node.switch(["aris_joint_impedance_controller"], ["fr3_arm_controller"])
    node.q_d = draw.q_start + 0.004
    arm._zero = BIAS
    node.trigger("hold")                                           # a latched hold
    r = arm.draw(draw)
    assert not r.done and r.why.startswith("stream refused")


def test_position_tracking_flies_every_kind_through_the_trajectory_controller(monkeypatch, rig,
                                                                             site):
    """Mode A: lower, draw and lift go to the trajectory controller exactly as planned; the
    impedance controller is never streamed to; the collision thresholds are the site's."""
    motions = _chain(rig, "2L", ("lower", "draw", "lift"))
    arm, node = _arm(monkeypatch, rig, site, motions[0].q_start)
    arm.set_job(rig.pen(), "position")
    for m in motions:
        r = arm.draw(m)
        assert r.done, r.why
    assert node.goals == [m.traj for m in motions] and not node.published
    assert "aris_joint_impedance_controller" not in node.active
    assert arm.set_collision("job") == "" and arm.set_collision("normal") == ""
    c = site.collision
    assert node.collision_calls == [(c["job"]["torque_nm"], c["job"]["force_n"]),
                                    (c["normal"]["torque_nm"], c["normal"]["force_n"])]
    with pytest.raises(ValueError):
        arm.set_job(rig.pen(), "fast")
