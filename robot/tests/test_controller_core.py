"""The controller's core (law, reference, holds) compiled here without ROS and checked against
the planner's own reading of the trajectory."""
import shutil
import subprocess
import time
from pathlib import Path

import numpy as np
import pytest

from aris.kernel.retime import sample
from aris.rig import Rig
from aris_robot.force import ForceSettings, profile
from aris_robot.stream import samples
from motions import random_trajectory

# compiling with Eigen takes 6 to 15 s on this machine
pytestmark = pytest.mark.slow

PKG = Path(__file__).resolve().parents[1] / "ros2_ws" / "src" / "aris_controllers"
CONFIG = Path(__file__).resolve().parents[2] / "config"
EIGEN = Path("/usr/include/eigen3")


@pytest.fixture(scope="module")
def core_test(tmp_path_factory):
    if shutil.which("g++") is None or not EIGEN.exists():
        pytest.skip("g++ or Eigen missing")
    exe = tmp_path_factory.mktemp("core") / "core_test"
    t0 = time.perf_counter()
    subprocess.run(["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Wpedantic", "-Werror",
                    "-Wno-maybe-uninitialized",          # Eigen's own false alarm at -O2
                    f"-I{EIGEN}", f"-I{PKG / 'include'}", str(PKG / "test" / "core_test.cpp"),
                    "-o", str(exe)], check=True, capture_output=True, text=True)
    print(f"compiled in {time.perf_counter() - t0:.1f} s")
    return exe


def test_scenarios_pass(core_test):
    out = subprocess.run([str(core_test)], capture_output=True, text=True)
    assert out.returncode == 0 and out.stdout.startswith("PASS"), out.stdout


def _follow(exe, s, dt):
    rows = np.column_stack([s.t, s.q, s.qd, s.f])
    text = "\n".join(" ".join(repr(float(x)) for x in r) for r in rows)
    out = subprocess.run([str(exe), "follow", repr(dt)], input=text, capture_output=True,
                         text=True, check=True)
    return np.array([[float(x) for x in line.split()] for line in out.stdout.splitlines()])


@pytest.mark.parametrize("dt", [0.001, 0.0007])
def test_controller_reference_is_the_planned_trajectory(core_test, dt):
    rig = Rig.load(CONFIG)
    traj = random_trajectory(rig, 31, 11)
    fn = profile("draw", traj.t, None, 1.0, ForceSettings())
    s = samples(traj, fn, rig.paper(31).normal)
    got = _follow(core_test, s, dt)
    t = got[:, 0]
    q_ref, qd_ref, _ = sample(traj, traj.t[0] + t)
    err_q = np.abs(got[:, 1:8] - q_ref).max()
    err_qd = np.abs(got[:-1, 8:15] - qd_ref[:-1]).max()
    print(f"dt {dt}: {len(t)} ticks, max |q_d - q(t)| {err_q:.2e} rad, "
          f"max |qd_d - qd(t)| {err_qd:.2e} rad/s")
    # on the millisecond samples the reference is the trajectory to rounding; between them
    # it is the cubic through two samples, exact inside a planner segment and within a few
    # nano-radians across a knot
    assert err_q <= (1e-12 if dt == 0.001 else 1e-8)
    assert err_qd <= (1e-9 if dt == 0.001 else 1e-5)
    assert np.array_equal(got[-1, 1:8], traj.q[-1])             # lands on the last knot
    assert np.all(got[-1, 8:15] == 0.0)                         # and holds there
    f_ref = np.array([np.interp(t, s.t, s.f[:, k]) for k in range(3)]).T
    assert np.abs(got[:, 15:18] - f_ref).max() <= 1e-12


def _drawing_poses(rig):
    """(arm, J 3x7 of the pen tip, paper normal) at the drawing poses of the six arms: pen
    upright on the paper, radius 0.3 to 0.7 m, 8 directions, 3 joint-7 angles, 3 hand turns,
    two arm shapes, inside the gates (sigma_min 0.04, limit margin 0.15)."""
    out = []
    for a in rig.arm_ids:
        arm, n = rig.arm(a), rig.paper(a).normal
        for rad in (0.3, 0.4, 0.5, 0.6, 0.7):
            for ang in np.linspace(0, 2 * np.pi, 8, endpoint=False):
                p = np.array([rad * np.cos(ang), rad * np.sin(ang), 0.97])
                for q7 in (-1.5, 0.0, 1.5):
                    for spin in (0.0, 1.5, 3.0):
                        Q, ok = arm.ik(arm.hand_pose(p[None], n, spin, np.zeros(2)), q7)
                        for q in Q[0][ok[0]][:2]:
                            if arm.sigma_min(q[None])[0] < 0.04 or \
                                    arm.limit_margin(q[None])[0] < 0.15:
                                continue
                            out.append((a, arm.tip_jacobian(q[None])[0], np.asarray(n, float)))
    return out


def _tip(J, K):
    return np.linalg.inv(J @ np.linalg.solve(K, J.T))


def test_the_pen_is_soft_along_the_paper_and_as_before_in_it(core_test):
    """The stiffness the compiled law gives at the pen tip, pen down, at the drawing poses."""
    rig = Rig.load(CONFIG)
    poses = _drawing_poses(rig)
    text = "\n".join(" ".join(repr(float(x)) for x in np.concatenate([J.ravel(), n]))
                     for _, J, n in poses)
    out = subprocess.run([str(core_test), "probe"], input=text, capture_output=True,
                         text=True, check=True).stdout.split("\n")
    k_n = 100.0
    Kq = np.diag([300.0, 300.0, 250.0, 250.0, 40.0, 40.0, 15.0])
    free, held, plane_before, plane_after, plane_err, d_n = [], [], [], [], [], []
    for (_, J, n), line in zip(poses, out):
        v = np.array(line.split(), float)
        Kp, Dp = v[:49].reshape(7, 7), v[49:98].reshape(7, 7)
        Kt0, Kt1 = _tip(J, Kq), _tip(J, Kp)
        free.append(1.0 / (n @ np.linalg.solve(Kt1, n)))           # pen free to slide
        held.append(n @ Kt1 @ n)                                   # pen held sideways
        Dt1 = _tip(J, Dp)
        d_n.append(n @ Dt1 @ n)
        B = np.linalg.svd(np.eye(3) - np.outer(n, n))[0][:, :2]    # the paper plane
        before, after = B.T @ Kt0 @ B, B.T @ Kt1 @ B
        plane_before.append(np.linalg.eigvalsh(before)[0])
        plane_after.append(np.linalg.eigvalsh(after)[0])
        plane_err.append(np.abs(after - before).max() / np.abs(before).max())
    free, held = np.array(free), np.array(held)
    print(f"{len(poses)} poses; along the normal, pen free to slide: min {free.min():.4f} "
          f"median {np.median(free):.4f} max {free.max():.4f} N/m; pen held: "
          f"{held.min():.4f} / {np.median(held):.4f} / {held.max():.4f}; softest in the plane "
          f"before {min(plane_before):.0f} / {np.median(plane_before):.0f} / "
          f"{max(plane_before):.0f}, after {min(plane_after):.0f} / "
          f"{np.median(plane_after):.0f} / {max(plane_after):.0f}; largest relative change of "
          f"the in-plane block {max(plane_err):.1e}; damping along the normal "
          f"{np.median(d_n):.2f} N s/m")
    assert len(poses) >= 600
    assert np.all(np.abs(free / k_n - 1.0) < 1e-6) and np.all(np.abs(held / k_n - 1.0) < 1e-6)
    assert max(plane_err) < 1e-9
    assert np.allclose(d_n, 2.0 * np.sqrt(k_n * 3.0), rtol=1e-6)
