"""Tests of the execution path (aris/execute): queue, executor, simulated arm, coordinator.

Quick set: hand-made motions near the parks of arms 1L and 3L (both move in the phases built
here; they hang 2.42 m apart), each passed by the independent checker.  Slow set: the word
planned by the system planner, queued through the checker while six simulated arms run it.
"""
from __future__ import annotations

import math
import threading
import time
from pathlib import Path

import numpy as np
import pytest

from aris.check import check
from aris.execute import Coordinator, EventLog, Executor, Job, Queue, feed, header_for
from aris.execute.drivers import Driver
from aris.execute.drivers.sim import SimArm
from aris.execute.queue import End, verdict_numbers
from aris.kernel.retime import retime
from aris.rig import Rig
from aris.types import JointPath, Motion, Phase, Piece, Trajectory

CONFIG = Path(__file__).resolve().parents[1] / "config"
DATA = Path(__file__).resolve().parent / "data"
ARMS = ("1L", "3L")
STEP = np.array([0.3, -0.3, 0.0, 0.45, 0.0, -0.45, 0.6])     # rad, a visible move from park


@pytest.fixture(scope="module")
def rig():
    return Rig.load(CONFIG)


def _phase(rig, name) -> Phase:
    return Phase(name, ARMS, tuple(a for a in rig.arm_ids if a not in ARMS), ())


def _tour(rig, arm_id):
    """park -> park + STEP -> park + STEP/2 -> park: three free motions."""
    p, arm = rig.park_q(arm_id), rig.arm(arm_id)
    qs = [p, p + STEP, p + 0.5 * STEP, p]
    return [Motion("free", retime(JointPath(np.array([qs[i], qs[i + 1]])), arm.limits,
                                  rig.rules())) for i in range(3)]


@pytest.fixture(scope="module")
def checked(rig):
    """arm id -> [(motion, verdict)], every one passed by the checker in phase "one"."""
    out = {}
    for a in ARMS:
        ms, q, out[a] = _tour(rig, a), rig.park_q(a), []
        for m in ms:
            v = check(CONFIG, a, m, _phase(rig, "one"), q)
            assert v.passed, str(v)
            out[a].append((m, v))
            q = m.q_end
    return out


class _Pass:
    """A stand-in verdict for the file-format tests (the real path uses the checker)."""
    passed, tightest, min_clearance, min_clearance_at = True, "none", 0.1, "nothing"

    def get(self, name):
        raise KeyError(name)


def _same(a: Motion, b: Motion) -> bool:
    arrays = all(np.array_equal(x, y) for x, y in
                 ((a.traj.t, b.traj.t), (a.traj.q, b.traj.q), (a.traj.qd, b.traj.qd)))
    tips = (a.tip_base is None and b.tip_base is None) or np.array_equal(a.tip_base, b.tip_base)
    return arrays and tips and a.kind == b.kind and a.piece == b.piece \
        and a.intensity == b.intensity


def _draw_like(rng, q0) -> Motion:
    t = np.cumsum(rng.uniform(0.01, 0.05, 40)) - 0.01
    q = q0 + np.cumsum(rng.normal(0, 1e-3, (40, 7)), axis=0)
    q[0] = q0
    return Motion("draw", Trajectory(t, q, rng.normal(0, 1e-2, (40, 7))),
                  Piece("line 7", 0.0123456789, 0.4), rng.normal(0, 0.3, (40, 3)), 0.75)


# --------------------------------------------------------------------------- the queue


def test_queue_written_and_read_back_exactly(tmp_path, checked):
    rng = np.random.default_rng(3)
    motions = [checked["1L"][0][0]]
    for _ in range(2):
        motions.append(_draw_like(rng, motions[-1].q_end))
    q = Queue(tmp_path / "a.queue", "one", "1L")
    for i, m in enumerate(motions):
        assert q.append(m, _Pass()) == i
    assert q.end() is None
    q.close()
    back = Queue(tmp_path / "a.queue").read()
    assert [e.index for e in back] == [0, 1, 2]
    assert all(_same(m, e.motion) for m, e in zip(motions, back))
    assert back[0].verdict == verdict_numbers(_Pass())
    assert Queue(tmp_path / "a.queue").end() == End(3, True, "")


def test_queue_keeps_the_checker_numbers_and_refuses(tmp_path, checked):
    (m0, v0), (m1, v1), _ = checked["1L"]
    q = Queue(tmp_path / "b.queue", "one", "1L")
    assert q.append(m0, v0) == 0
    e = q.read()[0]
    assert e.verdict["passed"] and e.verdict["tightest"] == v0.tightest
    assert e.verdict["min_clearance"] == v0.min_clearance
    assert q.append(m0, v0).reason == "not_continuous"        # starts at park, not at the end
    failed = type("V", (), dict(passed=False, tightest="clearance steel"))()
    assert q.append(m1, failed).reason == "failed_check"
    q.close(complete=False, note="cut short")
    assert q.append(m1, v1).reason == "closed"
    assert q.end() == End(1, False, "cut short")


def test_queue_watched_while_written_from_another_thread(tmp_path):
    rng = np.random.default_rng(5)
    motions, q0 = [], np.zeros(7)
    for _ in range(25):
        motions.append(_draw_like(rng, q0))
        q0 = motions[-1].q_end
    path = tmp_path / "c.queue"

    def writer():
        q = Queue(path, "one", "3L")
        for m in motions:
            q.append(m, _Pass())
            time.sleep(0.003)
        q.close()

    th = threading.Thread(target=writer)
    th.start()
    seen = list(Queue(path).watch(poll=0.001))
    th.join()
    assert isinstance(seen[-1], End) and seen[-1].count == 25
    assert all(_same(m, e.motion) for m, e in zip(motions, seen[:-1]))
    # a record cut off mid-write is not seen
    data = path.read_bytes()
    part = tmp_path / "d.queue"
    part.write_bytes(data[:len(data) // 2])
    got = Queue(part).read()
    assert 0 < len(got) < 25 and all(_same(m, e.motion) for m, e in zip(motions, got))


def test_job_header_and_phase_list(tmp_path, rig):
    job = Job.create(tmp_path / "job", header_for(rig, [], rig.rules()))
    h = job.header()
    assert len(h["rig_digest"]) == 24 and h["rules"]["draw_speed"] == rig.rules().draw_speed
    assert header_for(Rig.load(CONFIG), [], rig.rules())["rig_digest"] == h["rig_digest"]
    job.add_phase(rig.phase(1))
    job.end_phases()
    (p,) = list(job.watch_phases())
    assert p.name == "phase 1" and p.active == rig.phase(1).active
    assert np.array_equal(p.walls[0].normal_table, rig.phase(1).walls[0].normal_table)


# --------------------------------------------------------------------------- one arm


def _queue_of(tmp_path, checked, arm_id, name="q.queue"):
    q = Queue(tmp_path / name, "one", arm_id)
    for m, v in checked[arm_id]:
        q.append(m, v)
    q.close()
    return q


def test_simulated_arm_runs_three_motions_at_50x(tmp_path, rig, checked):
    q = _queue_of(tmp_path, checked, "1L")
    arm = SimArm("1L", rig.park_q("1L"), speed=50.0)
    assert isinstance(arm, Driver)
    log = EventLog(tmp_path / "events.jsonl")
    w0 = time.perf_counter()
    run = Executor("1L", arm, log, rig).run(q)
    wall = time.perf_counter() - w0
    flown = sum(float(m.traj.t[-1]) for m, _ in checked["1L"])
    print(f"three motions, {flown:.2f} s of motion, {wall:.3f} s wall at 50x")
    assert run.status == "finished" and run.done == 3 and run.parked
    last = checked["1L"][-1][0].q_end
    assert np.max(np.abs(arm.state().q - last)) < 1e-9
    assert np.all(arm.state().qd == 0.0)
    assert arm.clock == pytest.approx(flown)
    events = [e["event"] for e in log.read()]
    assert events[0] == "started" and events[-1] == "finished"
    assert events.count("motion started") == 3 and events.count("motion done") == 3


def test_injected_failure_stops_at_the_right_motion_and_holds(tmp_path, rig, checked):
    q = _queue_of(tmp_path, checked, "1L")
    d = [float(m.traj.t[-1]) for m, _ in checked["1L"]]
    fail_at = d[0] + 0.5 * d[1]
    arm = SimArm("1L", rig.park_q("1L"), speed=50.0, fail_at=fail_at, fail_why="joint 4 reflex")
    log = EventLog(tmp_path / "events.jsonl")
    run = Executor("1L", arm, log, rig).run(q)
    assert run.status == "failed" and run.failed_index == 1 and run.done == 1
    assert run.why == "joint 4 reflex"
    m1 = checked["1L"][1][0]
    expect = np.interp(0.5 * d[1], m1.traj.t, m1.traj.q[:, 0])
    assert abs(run.q[0] - expect) < 1e-2                    # where the motion was at the time
    s = arm.state()
    assert not s.ok and "fault: joint 4 reflex" in s.flags and np.all(s.qd == 0.0)
    time.sleep(0.05)
    assert np.array_equal(arm.state().q, run.q)            # it holds there
    assert arm.move(checked["1L"][2][0].traj).done is False   # and will not move before recover
    assert log.read()[-1]["event"] == "failed" and log.read()[-1]["index"] == 1


def test_start_configuration_mismatch_is_refused(tmp_path, rig, checked):
    q = _queue_of(tmp_path, checked, "3L")
    off = rig.park_q("3L") + np.array([0, 0, 0, 0, 0.02, 0, 0])
    arm = SimArm("3L", off, speed=50.0)
    run = Executor("3L", arm, EventLog(tmp_path / "e.jsonl"), rig).run(q)
    assert run.status == "failed" and run.failed_index == 0 and run.done == 0
    assert "not at the start: joint 5" in run.why
    assert np.array_equal(arm.state().q, off) and arm.clock == 0.0


class _Recorder:
    """A simulated arm that notes which verb ran each motion."""

    def __init__(self, arm):
        self.arm, self.arm_id, self.calls = arm, arm.arm_id, []

    def state(self):
        return self.arm.state()

    def move(self, traj):
        self.calls.append(("move", None))
        return self.arm.move(traj)

    def draw(self, motion):
        self.calls.append(("draw", motion.kind))
        return self.arm.draw(motion)

    def touch(self, motion):
        self.calls.append(("touch", motion.kind))
        return self.arm.touch(motion)

    def hold(self):
        self.arm.hold()

    def stop(self):
        self.arm.stop()

    def recover(self):
        return self.arm.recover()


def test_draw_lower_and_lift_go_to_draw_free_goes_to_move(tmp_path, rig, checked):
    from dataclasses import replace
    ms = [replace(m, kind=k) for (m, _), k in zip(checked["1L"], ("lower", "lift", "free"))]
    ms.append(_draw_like(np.random.default_rng(1), ms[-1].q_end))
    q = Queue(tmp_path / "k.queue", "one", "1L")
    for m in ms:
        q.append(m, _Pass())
    q.close()
    arm = _Recorder(SimArm("1L", rig.park_q("1L"), speed=math.inf))
    assert isinstance(arm, Driver)
    run = Executor("1L", arm, EventLog(tmp_path / "k.jsonl"), rig).run(q)
    assert run.status == "finished" and run.done == 4
    assert arm.calls == [("draw", "lower"), ("draw", "lift"), ("move", None), ("draw", "draw")]


# --------------------------------------------------------------------------- the coordinator


def _write_job(job, rig, checked, phases=("one", "two"), delay=0.0):
    for name in phases:
        job.add_phase(_phase(rig, name))
        for a in ARMS:
            q = job.queue(name, a)
            for m, v in checked[a]:
                time.sleep(delay)
                q.append(m, v)
            q.close()
    job.end_phases()


def test_coordinator_runs_two_phases_on_two_arms(tmp_path, rig, checked):
    job = Job.create(tmp_path / "job", header_for(rig, [], rig.rules()))
    arms = {a: SimArm(a, rig.park_q(a), speed=50.0) for a in ARMS}
    coord = Coordinator(job, arms, CONFIG, rig)
    out = {}
    th = threading.Thread(target=lambda: out.setdefault("run", coord.run()))
    th.start()                                  # it waits for the phases as they are written
    _write_job(job, rig, checked, delay=0.01)
    th.join(timeout=60)
    run = out["run"]
    assert run.status == "done", run.why
    assert run.phases_done == ["one", "two"]
    assert [(p, ok) for p, ok, _, _ in run.phase_ends] == [("one", True), ("two", True)]
    assert all(r.status == "finished" and r.parked and r.done == 3 for r in run.arms)
    for a in ARMS:
        assert np.max(np.abs(run.where[a] - rig.park_q(a))) < 1e-9
    ev = [e["event"] for e in EventLog(job.log_path).read()]
    assert ev.count("phase end check") == 2 and ev[-1] == "job done"


def test_coordinator_stop_holds_every_arm(tmp_path, rig, checked):
    job = Job.create(tmp_path / "job", {})
    _write_job(job, rig, checked)
    arms = {a: SimArm(a, rig.park_q(a), speed=2.0) for a in ARMS}
    coord = Coordinator(job, arms, CONFIG, rig)
    threading.Timer(0.25, coord.stop).start()
    run = coord.run()
    assert run.status == "stopped" and run.phases_done == []
    assert {r.status for r in run.arms} == {"stopped"}
    for a in ARMS:
        s = arms[a].state()
        assert not s.ok and "stopped" in s.flags and np.all(s.qd == 0.0)
        assert np.array_equal(run.where[a], s.q)
        assert np.max(np.abs(s.q - rig.park_q(a))) > 1e-3   # stopped mid-way, not at the park


def test_a_queue_cut_short_ends_the_job_after_its_phase(tmp_path, rig, checked):
    job = Job.create(tmp_path / "job", {})
    for name in ("one", "two"):
        job.add_phase(_phase(rig, name))
        for a in ARMS:
            q = job.queue(name, a)
            for m, v in checked[a][:2 if (a == "3L" and name == "one") else 3]:
                q.append(m, v)
            q.close(complete=not (a == "3L" and name == "one"), note="motion 2 refused")
    job.end_phases()
    arms = {a: SimArm(a, rig.park_q(a), speed=100.0) for a in ARMS}
    run = Coordinator(job, arms, CONFIG, rig).run()
    assert run.status == "failed" and run.phases_done == []
    assert "arm 3L" in run.why and "motion 2 refused" in run.why
    assert np.max(np.abs(run.where["1L"] - rig.park_q("1L"))) < 1e-9     # the other arm finished


# --------------------------------------------------------------------------- slow


@pytest.mark.slow
def test_word_on_six_simulated_arms(tmp_path_factory, rig):
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import system_cases as sc
    from aris.system import phase_named, plan

    cache = tmp_path_factory.mktemp("system_cache")
    job_dir = tmp_path_factory.mktemp("jobs") / "word"
    lines, rules = sc.word(), rig.rules()
    job = Job.create(job_dir, header_for(rig, lines, rules))
    arms = {a: SimArm(a, rig.park_q(a), speed=20.0) for a in rig.arm_ids}
    coord = Coordinator(job, arms, CONFIG, rig)
    out = {}
    w0, c0 = time.perf_counter(), time.process_time()
    th = threading.Thread(target=lambda: out.setdefault("run", coord.run()))
    th.start()
    fed = feed(job, plan(rig, lines, rules, cache_dir=cache, workers=4), CONFIG,
               lambda n: phase_named(rig, n))
    w_fed = time.perf_counter() - w0
    th.join(timeout=1800)
    wall, cpu = time.perf_counter() - w0, time.process_time() - c0
    run = out["run"]
    per_arm = {}
    for name, a, _ in fed.tagged:
        per_arm[(name, a)] = per_arm.get((name, a), 0) + 1
    motion_s = sum(float(m.traj.t[-1]) for _, _, m in fed.tagged)
    ev = EventLog(job.log_path).read()
    t_first = min(e["time"] for e in ev if e["event"] == "motion started") - ev[0]["time"]
    t_run = ev[-1]["time"] - ev[0]["time"] - t_first
    print(f"\nword: {len(fed.tagged)} motions {per_arm}, {motion_s:.1f} s of motion; planned, "
          f"checked and queued in {w_fed:.1f} s; first motion ran after {t_first:.1f} s; "
          f"from then to the job's end {t_run:.1f} s at 20x; job done after {wall:.1f} s wall "
          f"(this process CPU {cpu:.1f} s); phase ends {run.phase_ends}")
    assert fed.refused == [] and run.status == "done", (fed.refused, run.why)
    assert len(fed.tagged) > 40 and fed.queued == per_arm
    for (name, a), n in per_arm.items():            # the queues on disk are what was planned
        got = job.queue(name, a).read()
        mine = [m for p, b, m in fed.tagged if (p, b) == (name, a)]
        assert len(got) == n and all(_same(m, e.motion) for m, e in zip(mine, got))
        assert all(e.verdict["passed"] for e in got)
    assert all(ok for _, ok, _, _ in run.phase_ends) and len(run.phase_ends) >= 1
    for a in rig.arm_ids:
        assert np.max(np.abs(run.where[a] - rig.park_q(a))) < 1e-9, f"arm {a} not parked"
    assert all(r.parked for r in run.arms)
    np.savez(DATA / "execute_word.npz", wall=wall, fed_wall=w_fed, cpu=cpu, first=t_first,
             running=t_run,
             motion_s=motion_s, motions=len(fed.tagged),
             per_arm=np.array([f"{p}|{a}|{n}" for (p, a), n in sorted(per_arm.items())]))
