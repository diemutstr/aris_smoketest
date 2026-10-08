"""The real driver (position control only) against a fake arm node standing in for ROS
(fake_ros.py): every motion kind through the trajectory controller, recovery, stale joints."""
import time
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


@pytest.fixture(scope="module")
def rig():
    return Rig.load(CONFIG)


@pytest.fixture(scope="module")
def site():
    return site_mod.load(ROBOT / "site.json")


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


def _arm(monkeypatch, rig, site, q0):
    node = fake_ros.FakeArmNode(q0)
    monkeypatch.setattr(D, "ArmNode", lambda site_arm, names: node)
    return D.RosArm(site, rig, "2L"), node


def test_every_kind_flies_through_the_trajectory_controller(monkeypatch, rig, site):
    """lower, draw and lift go to the trajectory controller exactly as planned; it stays
    active and holding; the collision thresholds are the site's."""
    motions = _chain(rig, "2L", ("free", "lower", "draw", "lift"))
    arm, node = _arm(monkeypatch, rig, site, motions[0].q_start)
    for m in motions:
        r = arm.draw(m) if m.kind != "free" else arm.move(m.traj)
        assert r.done, r.why
    assert node.goals == [m.traj for m in motions]
    assert "fr3_arm_controller" in node.active and node.switches == []
    assert arm.set_collision("job") == "" and arm.set_collision("normal") == ""
    c = site.collision
    assert node.collision_calls == [(c["job"]["torque_nm"], c["job"]["force_n"]),
                                    (c["normal"]["torque_nm"], c["normal"]["force_n"])]


def test_a_trajectory_not_starting_at_the_arm_is_refused(monkeypatch, rig, site):
    (m,) = _chain(rig, "2L", ("free",))
    arm, node = _arm(monkeypatch, rig, site, m.q_start + np.array([0, 0.1, 0, 0, 0, 0, 0]))
    r = arm.move(m.traj)
    assert not r.done and "0.1 rad from the arm" in r.why and node.goals == []


def test_fault_then_recover_then_park(monkeypatch, rig, site):
    """A reflex: recovery clears it, brings the hardware and the controllers back (the
    trajectory controller and the broadcasters active), sees the joint states fresh, and a
    park flies without a restart of anything."""
    arm, node = _arm(monkeypatch, rig, site, rig.park_q("2L"))
    said = []
    arm.say = lambda event, **f: said.append(dict(event=event, **f))
    node.mode, node.errors = 4, ["cartesian_reflex"]
    node.q_d = rig.park_q("2L") + np.array([0, 0, 0, 0, 0, 0, 0.05])   # stopped near park
    park = retime(JointPath(np.array([node.q_d, rig.park_q("2L")])), rig.arm("2L").limits,
                  rig.rules())
    assert not arm.move(park).done                              # faulted: refused
    r = arm.recover()
    assert r.done, r.why
    assert node.recovery_steps == ["hardware FrankaHardwareInterface", "error recovery"]
    assert node.active == {"fr3_arm_controller", "joint_state_broadcaster",
                           "franka_robot_state_broadcaster"}
    assert [x["event"] for x in said] == ["recover: hardware component",
                                          "recover: error recovery",
                                          "recover: controllers", "recover: joint states fresh"]
    assert arm.move(park).done and np.allclose(node.q_d, rig.park_q("2L"))
    assert "fr3_arm_controller" in node.active                 # left active and holding


def test_a_stalled_stack_is_restarted_by_recovery(monkeypatch, rig, site):
    arm, node = _arm(monkeypatch, rig, site, rig.park_q("2L"))
    said, restarts = [], []
    arm.say = lambda event, **f: said.append(dict(event=event, **f))
    arm.fresh_wait_s = 1.5
    node.stalled_at = time.time() - 5.0          # joint states 5 s old
    s = arm.state()
    assert not s.ok and np.all(np.isnan(s.q)) and s.flags[0].startswith("stale joint states")
    from aris_robot.runner import reading
    q, why = reading(arm)
    assert q is None and why.startswith("stale joint states (last 5.")

    def restart():
        restarts.append(1)
        node.stalled_at = None                                 # the new stack publishes
    arm.restart_stack = restart
    r = arm.recover()
    assert r.done, r.why
    assert restarts == [1]
    events = [x["event"] for x in said]
    assert events[-3:] == ["recover: joint states not fresh", "recover: restarting the stack",
                           "recover: joint states fresh"]


def test_recover_with_a_dead_hardware_component_ends_fresh(monkeypatch, rig, site):
    """The stack started with FCI off: its hardware component read an error and was taken
    down, the joint states are stale, and error recovery alone fails.  Bringing the component
    back comes first, so the error recovery behind it then works, and the joints are fresh."""
    arm, node = _arm(monkeypatch, rig, site, rig.park_q("2L"))
    said = []
    arm.say = lambda event, **f: said.append(dict(event=event, **f))
    node.hardware_dead, node.stalled_at = True, time.time() - 40.0

    def reactivate(component, timeout=10.0):
        node.recovery_steps.append(f"hardware {component}")
        node.hardware_dead, node.stalled_at = False, None
        return ""

    def error_recovery(timeout=15.0):
        node.recovery_steps.append("error recovery")
        return "the error recovery did not succeed" if node.hardware_dead else ""

    node.reactivate_hardware, node.error_recovery = reactivate, error_recovery
    r = arm.recover()
    assert r.done, r.why
    assert node.recovery_steps == ["hardware FrankaHardwareInterface", "error recovery"]
    assert said[-1]["event"] == "recover: joint states fresh"


def test_recover_runs_every_step_and_restarts_a_stale_stack(monkeypatch, rig, site):
    """A failing step does not stop the sequence; joints still stale after it: the stack
    is restarted and checked again."""
    arm, node = _arm(monkeypatch, rig, site, rig.park_q("2L"))
    said, restarts = [], []
    arm.say = lambda event, **f: said.append(dict(event=event, **f))
    arm.fresh_wait_s = 1.2
    node.stalled_at = time.time() - 40.0
    node.reactivate_hardware = lambda component, timeout=10.0: "no hardware component service"

    def restart():
        restarts.append(1)
        node.stalled_at = None
    arm.restart_stack = restart
    r = arm.recover()
    assert r.done, r.why and restarts == [1]
    events = [x["event"] for x in said]
    assert events[:3] == ["recover: hardware component", "recover: error recovery",
                          "recover: controllers"]
    assert said[0]["ok"] is False and "recover: restarting the stack" in events
    assert events[-1] == "recover: joint states fresh"



def _guided(monkeypatch, rig, site, **cfg):
    hover = rig.park_q("2L")
    arm, node = _arm(monkeypatch, rig, site, hover)
    arm.guide_cfg = {**arm.guide_cfg, **dict(timeout_s=8.0, restart_every_s=0.3, settle_s=0.3), **cfg}
    arm.stale_s = 0.2
    said = []
    arm.say = lambda event, **f: said.append(dict(event=event, **f))
    from types import SimpleNamespace
    return arm, node, said, hover, SimpleNamespace(q_start=hover, q_end=hover)


def test_a_guide_survives_fci_off_and_samples_where_the_person_left_the_pen(
        monkeypatch, rig, site):
    """The person switches Desk to programming mode (FCI off: the joints go stale), moves the
    arm, switches FCI back on; the stack is restarted while stale, the arm recovered with its
    trajectory controller off, the sample read away from the hover, then the controller is
    back, the pen lifts, flies back to the hover and the next motion flies."""
    arm, node, said, hover, m = _guided(monkeypatch, rig, site)
    moved = hover + np.array([0.0, 0.06, 0.0, -0.05, 0.0, 0.04, 0.0])
    fci_on, restarts = [False], []

    def person():
        while not any(x["event"] == "instruction" for x in said):
            time.sleep(0.01)
        assert "fr3_arm_controller" not in node.active        # let go before the person comes
        node.stalled_at = time.time()                         # programming mode: FCI off
        time.sleep(0.5)
        with node._lock:
            node.q_d = moved.copy()                           # the pen tip on the other's
        time.sleep(0.5)
        fci_on[0] = True                                      # execution mode, FCI on

    def restart():
        restarts.append(time.time())
        if fci_on[0]:                                         # the new stack publishes
            node.stalled_at = None
            node.active |= {"fr3_arm_controller"}             # it comes up with it active

    arm.restart_stack = restart
    import threading
    threading.Thread(target=person, daemon=True).start()
    r = arm.guide(m)
    assert r.done and r.why == "check", r.why
    assert np.allclose(r.q, moved)
    assert len(restarts) >= 2
    events = [x["event"] for x in said]
    assert said[0]["text"].startswith("your turn: in Desk switch BOTH arms")
    for e in ("guide: link down (FCI off), waiting for FCI on", "guide: restarting the stack",
              "guide: hardware component", "guide: error recovery",
              "guide: trajectory controller off", "guide: link back", "guide: registered",
              "guide: trajectory controller back", "guide: lifted", "guide: back at the hover"):
        assert e in events, e
    # the trajectory controller was off when the sample was read, on again only after it
    assert events.index("guide: trajectory controller off") < events.index("guide: registered") \
        < events.index("guide: trajectory controller back")
    assert node.recovery_steps == ["hardware FrankaHardwareInterface", "error recovery"]
    assert "fr3_arm_controller" in node.active
    assert np.abs(node.q_d - hover).max() < 1e-3
    # the retreat: lift, then away; the tip never comes closer to the partner's tip (3 mm
    # off ours at the meeting, on the far side from our hover) than it was at the sample
    assert "guide: retreat away" in events
    arm_model = rig.arm("2L")
    tip_m, tip_h = arm_model.tip(moved[None])[0], arm_model.tip(hover[None])[0]
    n = arm.kin.normal / np.linalg.norm(arm.kin.normal)
    away = (tip_h - tip_m) - ((tip_h - tip_m) @ n) * n
    partner = tip_m - 0.003 * away / np.linalg.norm(away)
    trace = np.vstack([arm_model.tip(g.q) for g in node.goals])
    d = np.linalg.norm(trace - partner, axis=1)
    assert abs(d[0] - 0.003) < 1e-6 and d.min() >= d[0] - 1e-6
    nxt = _chain(rig, "2L", ("free",))[0]
    assert arm.move(nxt.traj).done


def test_a_guide_nobody_moves_waits_then_times_out(monkeypatch, rig, site):
    arm, node, said, hover, m = _guided(monkeypatch, rig, site, timeout_s=1.5)
    r = arm.guide(m)
    assert not r.done and "nobody guided 2L" in r.why
    texts = [x["text"] for x in said if x["event"] == "instruction"]
    assert texts[0].startswith("your turn") and texts[1:] == ["nobody moved 2L; waiting"]
    assert "fr3_arm_controller" in node.active                # holds again


def test_a_failed_retreat_keeps_the_sample_and_holds_at_the_meeting_pose(monkeypatch, rig, site):
    """The meeting worked, the glide back did not: the sample is registered (its row comes
    before any retreat row), the trajectory controller holds where the arm stands, and the
    result is a success with the sample and the real end pose."""
    arm, node, said, hover, m = _guided(monkeypatch, rig, site)
    moved = hover + np.array([0.0, 0.06, 0.0, -0.05, 0.0, 0.04, 0.0])
    from aris.types import Refusal
    monkeypatch.setattr(D.T, "glide", lambda *a, **k: Refusal("branch", "the line would jump"))

    def person():
        while not any(x["event"] == "instruction" for x in said):
            time.sleep(0.01)
        with node._lock:
            node.q_d = moved.copy()
    import threading
    threading.Thread(target=person, daemon=True).start()
    r = arm.guide(m)
    assert r.done and r.why == "check", r.why
    assert np.allclose(r.q, moved)                                 # the sample
    assert np.abs(r.q_end - hover).max() > 0.01                    # stands off the hover
    assert np.allclose(r.q_end, node.q_d)                          # where it really stands
    assert "fr3_arm_controller" in node.active                     # holding
    events = [x["event"] for x in said]
    reg = events.index("guide: registered")
    assert reg < events.index("guide: lifted")
    stay = next(x for x in said if x["event"] == "guide: registered; stayed at the meeting pose")
    assert stay["text"] == ("registered; stayed at the meeting pose (glide failed: cannot "
                            "retreat (away): the line would jump)")
    assert np.allclose(said[reg]["q"], moved)
