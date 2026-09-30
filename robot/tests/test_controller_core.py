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
