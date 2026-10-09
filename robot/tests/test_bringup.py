"""Launch arguments and controller settings for all six arms, from rig.json and site.json."""
import json
import math
import re
from pathlib import Path

import numpy as np
import pytest
import yaml

from aris.rig import Rig
from aris_robot import bringup, site as site_mod

ROBOT = Path(__file__).resolve().parents[1]
CONFIG = ROBOT.parent / "config"
LAUNCH = ROBOT / "ros2_ws" / "src" / "aris_bringup" / "launch" / "arm.launch.py"


def _R(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


@pytest.fixture(scope="module")
def written(tmp_path_factory):
    rig, site = Rig.load(CONFIG), site_mod.load(ROBOT / "site.json")
    out = tmp_path_factory.mktemp("gen")
    return rig, site, bringup.write(rig, site, out), out


def test_every_arm_gets_its_address_domain_and_hanging_base(written):
    rig, site, files, _ = written
    driven = sorted(a for a in rig.arm_ids if not site.arm(a).never)
    assert sorted(json.loads(f.read_text())["arm"] for f in files) == driven
    assert {"3L", "3R"} <= set(driven)                # row 3 hangs inverted too (2026-10-08)
    for f in files:
        a = json.loads(f.read_text())
        sa = site.arm(a["arm"])
        assert (a["namespace"], a["domain"], a["robot_ip"]) == (f"arm_{sa.id}", sa.domain, sa.ip)
        assert a["mount_to_world"] is True and a["use_fake_hardware"] is False
        T = rig.T_table_base(a["arm"])
        assert np.allclose(_R(a["mroll"], a["mpitch"], a["myaw"]), T[:3, :3], atol=1e-12)
        assert a["mz"] == pytest.approx(0.970)
        assert Path(a["controllers"]).exists()


def test_rpy_round_trips_on_random_rotations():
    rng = np.random.default_rng(0)
    for _ in range(200):
        r, p, y = rng.uniform(-3, 3), rng.uniform(-1.5, 1.5), rng.uniform(-3, 3)
        assert np.allclose(_R(*bringup.rpy(_R(r, p, y))), _R(r, p, y), atol=1e-12)
    assert np.allclose(_R(*bringup.rpy(_R(0.3, math.pi / 2, 0.0))), _R(0.3, math.pi / 2, 0.0))


def test_the_trajectory_controller_is_the_only_one(written):
    rig, site, _, out = written
    for a in (a for a in rig.arm_ids if not site.arm(a).never):
        d = yaml.safe_load((out / f"arm_{a}_controllers.yaml").read_text())
        assert not any("impedance" in k for k in d)
        assert set(d["/**/controller_manager"]["ros__parameters"]) == {
            "update_rate", "fr3_arm_controller", "joint_state_broadcaster",
            "franka_robot_state_broadcaster"}
        jtc = d["/**/fr3_arm_controller"]["ros__parameters"]
        assert jtc["joints"] == [f"fr3_joint{i}" for i in range(1, 8)]
        assert jtc["command_interfaces"] == ["effort"] and jtc["interpolation_method"] == "splines"
        assert set(jtc["gains"]) == set(jtc["joints"])
        assert all(j in jtc["constraints"] for j in jtc["joints"])


def test_fake_hardware_uses_positions(tmp_path):
    rig, site = Rig.load(CONFIG), site_mod.load(ROBOT / "site.json")
    bringup.write(rig, site, tmp_path, fake=True)
    d = yaml.safe_load((tmp_path / "arm_2L_controllers.yaml").read_text())
    assert d["/**/fr3_arm_controller"]["ros__parameters"]["command_interfaces"] == ["position"]
    assert "gains" not in d["/**/fr3_arm_controller"]["ros__parameters"]
    assert json.loads((tmp_path / "arm_2L.json").read_text())["use_fake_hardware"] is True


def test_the_launch_file_reads_only_what_is_written(written):
    _, _, files, _ = written
    keys = set(re.findall(r"a\['(\w+)'\]", LAUNCH.read_text()))
    assert keys and keys <= set(json.loads(files[0].read_text()))


def _site_files(tmp_path, change_site=None, change_table=None):
    d = json.loads((ROBOT / "site.json").read_text())
    t = json.loads((ROBOT.parent / d["site_table"]).read_text())
    (change_site or (lambda x: None))(d)
    (change_table or (lambda x: None))(t)
    (tmp_path / "robot").mkdir(parents=True)
    (tmp_path / "site").mkdir(parents=True)
    d["site_table"] = "site/table.json"
    (tmp_path / "robot" / "site.json").write_text(json.dumps(d))
    (tmp_path / "site" / "table.json").write_text(json.dumps(t))
    return tmp_path / "robot" / "site.json"


def test_site_reads_robots_from_the_site_table(tmp_path):
    s = site_mod.load(_site_files(tmp_path))
    a = s.arm("2R")
    row = json.loads((ROBOT.parent / json.loads((ROBOT / "site.json").read_text())
                      ["site_table"]).read_text())["slots"]["2R"]
    assert (a.robot, a.ip, a.domain, a.namespace) == (row["robot"], row["ip"], row["domain"],
                                                       "arm_2R")
    assert s.mounted == ("1L", "1R", "2L", "2R", "3L", "3R")       # all six since 2026-10-09
    assert site_mod.identity(a, None) == "unverified"
    known = site_mod.SiteArm("2R", "fr3-71", a.ip, 71, True, serial="295341-1234")
    assert site_mod.identity(known, "295341-1234") == "verified"
    assert site_mod.identity(known, "295341-9999").startswith("mismatch")


def test_site_file_is_checked(tmp_path):
    def dup(t):
        t["slots"]["1R"]["domain"] = t["slots"]["1L"]["domain"]
    with pytest.raises(ValueError):
        site_mod.load(_site_files(tmp_path / "a", change_table=dup))
    with pytest.raises(ValueError):
        site_mod.load(_site_files(tmp_path / "b",
                                  change_site=lambda d: d["slots"].update({"4L": {"mounted": False}})))
