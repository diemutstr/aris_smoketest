"""The calibration (aris/calib): the plane job (DESIGN.md section 6 step 1), the touch-off
(step 4, DESIGN 4c) and the two-part file, on synthetic touches.

The kernel's forward kinematics is the truth: a true paper plane is invented (the nominal one
tilted and raised by known amounts), the pen tip is put exactly on it by the IK (pen upright, one
hand spin), the joints get noise, and the solvers must find the tilt, the height and the pen
length again.  Every rig is loaded from a copy of rig.json in a temporary directory, so files
written by the tests (or lying in config/) never leak in.
"""
import json
import shutil
from pathlib import Path

import numpy as np
import pytest

from aris.calib import (calibrate_plane, calibration_from_events, fit_plane, listing, read,
                        touchoff, write_base, write_pen)
from aris.calib import plane as P
from aris.kernel.arm import Arm
from aris.kernel.tool import with_tip
from aris.rig import Rig

CASES = [("config/two_arms", "2R", (1.5, -2.0), 0.020, 0.0),
         ("config/two_arms", "2L", (-2.0, 0.7), -0.015, 1.2),
         ("config", "1L", (0.4, 1.8), 0.008, -0.6)]


def fresh(src, dst, pen=None):
    """A config dir at `dst` with `src`'s rig.json, optionally another pen in."""
    dst.mkdir(parents=True, exist_ok=True)
    cfg = json.loads((Path(src) / "rig.json").read_text())
    if pen is not None:
        cfg["pens"]["table"][pen] = dict(cfg["pens"]["table"][cfg["pens"]["current"]])
        cfg["pens"]["current"] = pen
    (dst / "rig.json").write_text(json.dumps(cfg))
    return dst


@pytest.fixture(scope="module")
def rigs(tmp_path_factory):
    out = {}
    for c in ("config/two_arms", "config"):
        out[c] = Rig.load(fresh(c, tmp_path_factory.mktemp(c.replace("/", "_"))))
    return out


def true_pose(rig, slot, tilt_deg, dz):
    """The pose the rig has, turned by (roll, pitch, 0) about the base origin and raised by dz:
    the solver's convention."""
    T = rig.T_table_base(slot)
    T[:3, :3] = P._rotvec_to_matrix(np.deg2rad([tilt_deg[0], tilt_deg[1], 0.0])) @ T[:3, :3]
    T[2, 3] += dz
    return T


def touch_q(arm, T_true, xy_table, spin, paper_z=0.0):
    """Joints that put `arm`'s tip on the true paper at each table xy, pen upright, best
    sigma_min over q7 and branches."""
    R, t = T_true[:3, :3], T_true[:3, 3]
    xy = np.atleast_2d(xy_table)
    p_table = np.column_stack([xy, np.full(len(xy), paper_z)])
    p_base = (p_table - t) @ R
    normal = R.T @ np.array([0.0, 0.0, 1.0])
    T_hand = arm.hand_pose(p_base, normal, np.full(len(p_base), spin), np.zeros((len(p_base), 2)))
    best_q, best_s = np.full((len(p_base), 7), np.nan), np.full(len(p_base), -1.0)
    for q7 in np.linspace(-2.8, 2.8, 29):
        Q, ok = arm.ik(T_hand, np.full(len(p_base), q7))
        for b in range(Q.shape[1]):
            m = np.nonzero(ok[:, b])[0]
            if not len(m):
                continue
            s = arm.sigma_min(Q[m, b])
            better = s > best_s[m]
            best_q[m[better]], best_s[m[better]] = Q[m[better], b], s[better]
    assert np.all(np.isfinite(best_q)), "test set-up: a point is out of reach"
    assert np.allclose(arm.tip(best_q), p_base, atol=1e-6)
    return best_q


def touches(rig, slot, T_true, spin, noise=0.0, seed=0, half=0.2, n=5, arm=None):
    """The plane job's grid: n x n points, +-half around the slot's axis, plus joint noise."""
    ax = rig.T_table_base(slot)[:2, 3]
    g = np.linspace(-half, half, n)
    xy = np.array([[ax[0] + x, ax[1] + y] for x in g for y in g])
    Q = touch_q(arm or rig.arm(slot), T_true, xy, spin, rig.paper_z)
    return Q + noise * np.random.default_rng(seed).standard_normal(Q.shape)


# ------------------------------------------------------------------ the fit itself

def test_fit_plane_exact_and_oriented():
    rng = np.random.default_rng(1)
    n_true = np.array([0.02, -0.01, -1.0])
    n_true /= np.linalg.norm(n_true)
    uv = rng.uniform(-0.3, 0.3, (30, 2))
    e1 = np.cross(n_true, [1.0, 0, 0]); e1 /= np.linalg.norm(e1)
    e2 = np.cross(n_true, e1)
    p = -0.97 * n_true + uv[:, :1] * e1 + uv[:, 1:] * e2   # plane n.p = -0.97
    n, c, res = fit_plane(p)
    assert np.allclose(n, n_true, atol=1e-12) and abs(c + 0.97) < 1e-12
    assert np.max(np.abs(res)) < 1e-12
    n2, c2, _ = fit_plane(-p)                               # same plane seen from the other side
    assert n2 @ (-p.mean(axis=0)) < 0 and c2 < 0           # still points toward the origin
    with pytest.raises(ValueError):
        fit_plane(p[:2])


# ------------------------------------------------------------------ plane: recovery

@pytest.mark.parametrize("cfg,slot,tilt_deg,dz,spin", CASES)
def test_recovers_tilt_and_height(rigs, cfg, slot, tilt_deg, dz, spin):
    rig = rigs[cfg]
    T_true = true_pose(rig, slot, tilt_deg, dz)
    seed = ord(slot[0]) + ord(slot[1])
    r = calibrate_plane(rig, slot, touches(rig, slot, T_true, spin, 0.5e-3, seed))
    assert r.passed, r.why
    assert r.n_points == 25 and r.slot == slot and r.pen == rig.pen_name
    assert abs(np.rad2deg(r.roll) - tilt_deg[0]) < 0.05
    assert abs(np.rad2deg(r.pitch) - tilt_deg[1]) < 0.05
    assert abs(r.height_change - dz) < 0.2e-3
    T_nom = rig.T_table_base(slot)
    assert np.array_equal(r.T_table_base[:2, 3], T_nom[:2, 3])
    assert np.allclose(r.T_table_base[:3, :3], T_true[:3, :3], atol=1e-3)
    assert r.points_table.shape == (25, 3)
    assert np.allclose(r.points_table[:, 2] - rig.paper_z, r.residuals, atol=1e-12)


def test_noise_numbers(rigs, capsys):
    """Report recovered errors at 0.5 and 2 mrad joint noise; assert only the 0.5 mrad case."""
    rig = rigs["config/two_arms"]
    T_true = true_pose(rig, "2R", (1.5, -2.0), 0.020)
    for noise in (0.5e-3, 2e-3):
        e_tilt, e_h, rms = [], [], []
        for seed in range(10):
            r = calibrate_plane(rig, "2R", touches(rig, "2R", T_true, 0.0, noise, seed))
            e_tilt.append(max(abs(np.rad2deg(r.roll) - 1.5), abs(np.rad2deg(r.pitch) + 2.0)))
            e_h.append(abs(r.height_change - 0.020))
            rms.append(r.rms)
        with capsys.disabled():
            print(f"\n  plane, noise {noise * 1e3:.1f} mrad, 10 seeds: tilt error max "
                  f"{max(e_tilt):.4f} deg, height error max {max(e_h) * 1e3:.4f} mm, "
                  f"fit RMS mean {np.mean(rms) * 1e3:.3f} mm")
        if noise == 0.5e-3:
            assert max(e_tilt) < 0.05 and max(e_h) < 0.2e-3


# ------------------------------------------------------------------ plane: refusals

def test_far_plane_refused(rigs):
    rig = rigs["config/two_arms"]
    r = calibrate_plane(rig, "2R", touches(rig, "2R", true_pose(rig, "2R", (0.5, 0.5), 0.040),
                                           0.0, 0.5e-3, 3))
    assert not r.passed
    assert "from its nominal height" in r.why and "off the plane" not in r.why
    assert abs(r.height_change - 0.040) < 0.2e-3


def test_large_tilt_refused(rigs):
    rig = rigs["config/two_arms"]
    r = calibrate_plane(rig, "2L", touches(rig, "2L", true_pose(rig, "2L", (3.5, 0.0), 0.0), 0.0))
    assert not r.passed and "leans" in r.why and "height" not in r.why


def test_slipped_touch_refused(rigs):
    rig = rigs["config/two_arms"]
    T_true = true_pose(rig, "2R", (1.0, 1.0), 0.010)
    Q = touches(rig, "2R", T_true, 0.0, 0.5e-3, 4)
    T_slip = T_true.copy()
    T_slip[2, 3] -= 0.005                 # touch 7 lands 5 mm above the paper
    Q[7] = touches(rig, "2R", T_slip, 0.0, 0.5e-3, 4)[7]
    r = calibrate_plane(rig, "2R", Q)
    assert not r.passed and r.worst_index == 7
    assert "touch 7 is" in r.why and "slipped" in r.why
    assert r.max_residual > P.MAX_RESIDUAL_M


@pytest.mark.parametrize("Q,words", [
    (np.zeros((0, 7)), "no touches"),
    (np.zeros((2, 7)), "only 2 touches"),
    (np.zeros((5, 6)), "N x 7"),
])
def test_degenerate_inputs_are_refusals(rigs, Q, words):
    r = calibrate_plane(rigs["config"], "1L", Q)
    assert not r.passed and words in r.why
    assert np.array_equal(r.T_table_base, rigs["config"].T_table_base("1L"))


def test_collinear_and_unreadable_are_refusals(rigs):
    rig = rigs["config"]
    Q = touches(rig, "1L", rig.T_table_base("1L"), 0.0)
    r = calibrate_plane(rig, "1L", Q[[0, 1, 2, 3, 4]])          # one grid row: a line
    assert not r.passed and "on a line" in r.why
    r = calibrate_plane(rig, "1L", np.repeat(Q[:1], 12, axis=0))   # one spot, twelve times
    assert not r.passed and "on a line" in r.why
    bad = Q.copy()
    bad[3, 2] = np.nan
    r = calibrate_plane(rig, "1L", bad)
    assert not r.passed and "touch 3 has no joint reading" in r.why
    r = calibrate_plane(rig, "1L", Q[[0, 4, 12, 20, 24]])     # a plane, but too few points
    assert not r.passed and "only 5 touches" in r.why


def _rows(slot, Q, other=None):
    rows = [{"event": "motion started", "arm": slot, "index": 0}]
    for k, q in enumerate(Q):
        rows.append({"event": "contact", "arm": slot, "q": list(map(float, q)), "index": k})
        if other is not None:
            rows.append({"event": "contact", "arm": other, "q": [0.0] * 7, "index": k})
    return rows


def test_from_events_picks_this_slot_in_order(rigs):
    rig = rigs["config/two_arms"]
    Q = touches(rig, "2R", true_pose(rig, "2R", (1.0, -1.0), 0.005), 0.0, 0.5e-3, 5)
    a = calibration_from_events(rig, "2R", _rows("2R", Q, other="2L"))
    b = calibrate_plane(rig, "2R", Q)
    assert a.passed and np.array_equal(a.T_table_base, b.T_table_base)
    assert np.array_equal(a.residuals, b.residuals)
    assert not calibration_from_events(rig, "2L", _rows("2R", Q)).passed


# ------------------------------------------------------------------ the file

def test_base_file_loads_through_rig(rigs, tmp_path):
    rig = rigs["config/two_arms"]
    T_true = true_pose(rig, "2R", (1.2, -0.8), 0.012)
    r = calibrate_plane(rig, "2R", touches(rig, "2R", T_true, 0.0, 0.5e-3, 6))
    assert r.passed
    cfg = fresh("config/two_arms", tmp_path)
    path = write_base(r, cfg, date="2026-10-02")
    assert path == cfg / "calibration" / "2R.json"
    cal = json.loads(path.read_text())
    assert cal["slot"] == "2R" and set(cal) == {"slot", "base"}
    b = cal["base"]
    assert b["passed"] is True and b["why"] == "" and b["method"] == "plane"
    assert b["residuals"]["n_points"] == 25 and len(b["height_map_table_m"]) == 25
    assert abs(b["height_change_mm"] - 12.0) < 0.2 and b["residuals"]["rms_mm"] < 0.5
    assert b["measured_with"]["pen"] == rig.pen_name

    loaded = Rig.load(cfg)
    st = loaded.calibration_status("2R")
    assert st["base"].startswith("applied: 2R.json base (2026-10-02") and st["pen"] == "none"
    assert loaded.calibration_status("2L") == {"base": "none", "pen": "none"}
    T = loaded.T_table_base("2R")
    assert np.allclose(T, r.T_table_base, atol=1e-12)
    assert np.allclose(T[:3, :3], T_true[:3, :3], atol=1e-3)
    assert abs(T[2, 3] - T_true[2, 3]) < 0.2e-3
    assert np.allclose(loaded.paper("2R").normal, r.normal_base, atol=1e-9)

    bad = calibrate_plane(rig, "2L", touches(rig, "2L", true_pose(rig, "2L", (0, 0), 0.040), 0.0))
    write_base(bad, cfg, date="2026-10-02")
    assert read(cfg, "2L")["base"]["why"]
    loaded = Rig.load(cfg)
    assert loaded.calibration_status("2L")["base"].startswith("base part not applied")
    assert np.array_equal(loaded.T_table_base("2L"), rig.T_table_base("2L"))

    ls = {f["slot"]: f for f in listing(cfg)}
    assert ls["2R"]["base"]["passed"] is True and ls["2R"]["pen"] is None
    assert ls["2L"]["base"]["passed"] is False and ls["2L"]["base"]["why"]


# ------------------------------------------------------------------ the touch-off

def plane_then_pen(src, dst, slot, tilt, dz, pen_at_plane, pen_now, ref_offset=(0.1, 0.05),
                   noise=0.0, seed=0):
    """The plane job with a pen `pen_at_plane` m longer than nominal, its base part written and
    loaded, then one touch with a pen `pen_now` m longer at the reference point.
    -> (rig with the base applied, PenCalibration, true tip now, config dir)."""
    cfg = fresh(src, dst)
    rig0 = Rig.load(cfg)
    T_true = true_pose(rig0, slot, tilt, dz)
    tool = rig0.arm(slot).tool
    true_arm = lambda d: Arm(with_tip(tool, tool.tip_hand + d * tool.pen_axis_hand))
    plane = calibrate_plane(rig0, slot, touches(rig0, slot, T_true, 0.3, 0.0, 1,
                                                arm=true_arm(pen_at_plane)))
    assert plane.passed, plane.why
    write_base(plane, cfg, date="2026-10-02")
    rig = Rig.load(cfg)
    ref = rig.T_table_base(slot)[:2, 3] + np.asarray(ref_offset)
    q = touch_q(true_arm(pen_now), T_true, ref, 0.3, rig.paper_z)[0]
    q = q + noise * np.random.default_rng(seed).standard_normal(7)
    return rig, touchoff(rig, slot, q, ref, rig.pen_name), cfg


def test_touchoff_finds_a_longer_pen(tmp_path):
    rig, r, _ = plane_then_pen("config/two_arms", tmp_path, "2R", (1.0, -1.5), 0.010, 0.0, 0.0013)
    assert r.passed, r.why
    u = rig.arm("2R").tool.pen_axis_hand
    shift = r.tip_hand - rig.nominal_tip()
    assert abs(shift @ u - 0.0013) < 0.05e-3
    assert np.linalg.norm(shift - (shift @ u) * u) < 1e-9          # along the axis only
    assert abs(r.correction - 0.0013) < 0.05e-3 and abs(r.change - 0.0013) < 0.05e-3
    assert r.from_reference < 1e-6


def test_first_touchoff_after_plane_is_zero(tmp_path):
    """The plane job's height already holds the pen it was measured with."""
    _, r, _ = plane_then_pen("config/two_arms", tmp_path, "2L", (-0.5, 0.5), -0.008, 0.002, 0.002)
    assert r.passed and abs(r.correction) < 1e-6


def test_touchoff_refusals(rigs, tmp_path):
    _, r, _ = plane_then_pen("config/two_arms", tmp_path / "a", "2R", (0, 0), 0.0, 0.0, 0.008)
    assert not r.passed and "+8.00 mm off its nominal length" in r.why
    _, r, _ = plane_then_pen("config/two_arms", tmp_path / "b", "2R", (0, 0), 0.0, 0.0, 0.001,
                             ref_offset=(0.1, 0.05))
    assert r.passed
    rig = Rig.load(tmp_path / "b")
    far = rig.T_table_base("2R")[:2, 3] + [0.1 + 0.04, 0.05]
    r2 = touchoff(rig, "2R", r.q, far, rig.pen_name)
    assert not r2.passed and "from the reference point" in r2.why
    r3 = touchoff(rig, "2R", r.q, far, "gel_07")
    assert not r3.passed and "'gel_07'" in r3.why
    r4 = touchoff(rig, "2R", r.q[:6], far, rig.pen_name)
    assert not r4.passed and "7 joint readings" in r4.why
    # no base part: the plane must be known first
    r5 = touchoff(rigs["config/two_arms"], "2R", r.q, far, rig.pen_name)
    assert not r5.passed and "run the plane job first" in r5.why


def test_touchoff_noise(tmp_path, capsys):
    """Report the length error of one touch at 0.5 mrad joint noise (not asserted tightly)."""
    errs = []
    for seed in range(10):
        _, r, _ = plane_then_pen("config/two_arms", tmp_path / str(seed), "2R", (1.0, -1.5),
                                 0.010, 0.0, 0.0013, noise=0.5e-3, seed=seed)
        errs.append(abs(r.correction - 0.0013))
    with capsys.disabled():
        print(f"\n  touch-off, noise 0.5 mrad, 10 seeds: length error max "
              f"{max(errs) * 1e3:.3f} mm, mean {np.mean(errs) * 1e3:.3f} mm")
    assert max(errs) < 1e-3


def test_two_parts_written_independently(tmp_path):
    rig, r, cfg = plane_then_pen("config/two_arms", tmp_path, "2R", (1.0, -1.5), 0.010, 0.0,
                                 0.0013)
    before = read(cfg, "2R")["base"]
    write_pen(r, cfg, date="2026-10-03")
    cal = read(cfg, "2R")
    assert cal["base"] == before
    p = cal["pen"]
    assert p["passed"] is True and p["pen"] == rig.pen_name and p["date"] == "2026-10-03"
    assert p["reference_touch"]["q"] == pytest.approx(list(r.q))
    assert abs(p["correction_mm"] - 1.3) < 0.05
    # a new plane job rewrites base and keeps pen
    plane = calibrate_plane(rig, "2R", touches(rig, "2R", rig.T_table_base("2R"), 0.0))
    write_base(plane, cfg, date="2026-10-04")
    cal2 = read(cfg, "2R")
    assert cal2["pen"] == p and cal2["base"]["date"] == "2026-10-04"

    loaded = Rig.load(cfg)
    st = loaded.calibration_status("2R")
    assert st["base"].startswith("applied") and st["pen"].startswith("applied")
    assert loaded.calibrated("2R") and not loaded.calibrated("2L")
    assert np.allclose(loaded.arm("2R").tool.tip_hand, r.tip_hand, atol=1e-9)
    ls = {f["slot"]: f for f in listing(cfg)}
    assert ls["2R"]["pen"]["passed"] and ls["2R"]["pen"]["pen"] == rig.pen_name

    # another pen in: the pen part stays on disk but is not applied
    other = fresh(cfg, tmp_path / "other", pen="gel_07")
    shutil.copytree(cfg / "calibration", other / "calibration")
    st = Rig.load(other).calibration_status("2R")
    assert st["base"].startswith("applied")
    assert st["pen"].startswith("pen part not applied") and not Rig.load(other).calibrated("2R")
    assert rig.pen_name in st["pen"] and "gel_07" in st["pen"]


def test_writer_refuses_a_foreign_file(tmp_path, rigs):
    cfg = fresh("config/two_arms", tmp_path)
    (cfg / "calibration").mkdir()
    (cfg / "calibration" / "2R.json").write_text(json.dumps({"slot": "2L"}))
    r = calibrate_plane(rigs["config/two_arms"], "2R", np.zeros((0, 7)))
    with pytest.raises(ValueError):
        write_base(r, cfg)


# ================================================================== marks (steps 2 and 3)

from aris.calib import (base_tips, pivot, solve_marks, touch_point,  # noqa: E402
                        write_mark_solution)
from aris.calib import marks as MK  # noqa: E402
from aris.calib.plane import _horizontal_turn  # noqa: E402
from aris.calib.simulate import seam_error, simulate_touches, true_marks  # noqa: E402

SIX = ("1L", "1R", "2L", "2R", "3L", "3R")
PEN_LONGER = 0.0013
SPEC = (0.3e-3, 0.3e-3)                  # the brief's guiding error (m) and joint noise (rad)
ENCODER = (0.1e-3, 0.05e-3)              # a seated dimple and encoder-level joint noise


def _rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _yaw(T):
    return float(np.arctan2(T[1, 0], T[0, 0]))


def _write(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data))


def make_truth(src, root, slots, seed=0, big=None):
    """Two config dirs under `root`: "true" (calibration files hold every slot's true pose and
    the true pen, 1.3 mm long) and "nom" (what the plane job left: the true tilt and the height
    it read with the nominal pen, nominal x, y, yaw).  Each slot's truth: 1-2 cm in x and y,
    3-8 mrad yaw, 1 deg roll and pitch, 10 mm z.  `big` = (slot, dx) moves one slot further.
    -> (rig_true, rig_nominal, {slot: T_true}, true tip)."""
    rng = np.random.default_rng(seed)
    for d in ("true", "nom"):
        fresh(src, root / d)
    rn = Rig.load(root / "nom")
    tip = rn.nominal_tip() + PEN_LONGER * rn.arm(slots[0]).tool.pen_axis_hand
    sign = lambda: float(rng.choice([-1.0, 1.0]))
    truth = {}
    for s in slots:
        T0 = rn.T_table_base(s)
        T = T0.copy()
        v = np.deg2rad([sign(), sign(), 0.0])
        T[:3, :3] = _rz(sign() * rng.uniform(3e-3, 8e-3)) @ P._rotvec_to_matrix(v) @ T0[:3, :3]
        T[:2, 3] += [sign() * rng.uniform(0.01, 0.02), sign() * rng.uniform(0.01, 0.02)]
        T[2, 3] += sign() * 0.010
        if big and big[0] == s:
            T[0, 3] = T0[0, 3] + big[1]
        truth[s] = T
        _write(root / "true" / "calibration" / f"{s}.json", {
            "slot": s, "base": {"passed": True, "method": "truth", "T_table_base": T.tolist()},
            "pen": {"passed": True, "pen": rn.pen_name, "tip_hand_m": tip.tolist()}})
        Tp = T0.copy()                  # the plane job: smallest tilt, height with nominal pen
        Tp[:3, :3] = P._rotvec_to_matrix(_horizontal_turn(T0[:3, :3] @ T[:3, :3].T[:, 2])) \
            @ T0[:3, :3]
        Tp[2, 3] = T[2, 3] - (tip - rn.nominal_tip())[2]
        _write(root / "nom" / "calibration" / f"{s}.json", {
            "slot": s, "base": {"passed": True, "method": "plane", "date": "2026-10-04",
                                "T_table_base": Tp.tolist(), "why": ""}})
    return Rig.load(root / "true"), Rig.load(root / "nom"), truth, tip


def _wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def errors(sol, truth, tip, rig_true):
    """-> (worst xy m, worst yaw rad, worst tip m, worst z m, worst seam m)."""
    f = sol.slots
    xy = max(np.linalg.norm(f[s].T_table_base[:2, 3] - truth[s][:2, 3]) for s in f)
    yaw = max(abs(_wrap(_yaw(f[s].T_table_base) - _yaw(truth[s]))) for s in f)
    tp = max(np.linalg.norm(f[s].tip_hand - tip) for s in f)
    z = max(abs(f[s].T_table_base[2, 3] - truth[s][2, 3]) for s in f)
    return xy, yaw, tp, z, seams(sol, rig_true)


SEAMS = (("2L", "2R", (0.0, 0.0)), ("1L", "1R", (0.0, -1.21)), ("3L", "3R", (0.0, 1.21)),
         ("1L", "2L", (-0.2, -0.605)), ("1R", "2R", (0.2, -0.605)),
         ("2L", "3L", (-0.2, 0.605)), ("2R", "3R", (0.2, 0.605)))


def seams(sol, rig_true):
    """The worst distance between where two neighbours put the pen when aimed at the same
    point of their seam, under the recovered calibration (0 when no pair is in the set)."""
    out = 0.0
    for a, b, xy in SEAMS:
        if a in sol.slots and b in sol.slots:
            e = seam_error(rig_true, {s: sol.slots[s].T_table_base for s in (a, b)},
                           {s: sol.slots[s].tip_hand for s in (a, b)}, xy, (a, b))
            out = max(out, float(np.linalg.norm(e[a] - e[b])))
    return out


def scenarios(tmp, noise, seed):
    """(a) 2L+2R with A and B; (b) six slots, ten marks; (c) rows12 after (b), the marks of (b)
    known; (d) 2R alone after (b).  -> {label: (solution, truth, tip, rig_true)}."""
    out = {}
    rt, rn, truth, tip = make_truth("config/two_arms", tmp / "a", ("2L", "2R"), seed)
    m = true_marks(rn, ("A", "B"), seed=seed)
    tq = simulate_touches(rt, rn, ("2L", "2R"), m, noise, seed)
    out["a"] = (solve_marks(rn, tq, base_tips={s: rn.nominal_tip() for s in tq}), truth, tip, rt)
    rt, rn, truth, tip = make_truth("config", tmp / "b", SIX, seed)
    m = true_marks(rn, tuple(rn.marks), seed=seed)
    tq = simulate_touches(rt, rn, SIX, m, noise, seed)
    b = solve_marks(rn, tq, base_tips={s: rn.nominal_tip() for s in SIX})
    out["b"] = (b, truth, tip, rt)
    known = {n: f.xy for n, f in b.marks.items()}
    sub = rn.mark_groups["rows12"]
    tq = simulate_touches(rt, rn, sub, {n: m[n] for n in rn.marks_for(sub)}, noise, seed + 100)
    out["c"] = (solve_marks(rn, tq, known, {s: rn.nominal_tip() for s in sub}), truth, tip, rt)
    mine = {n: m[n] for n in rn.marks if "2R" in rn.marks[n][1]}
    tq = simulate_touches(rt, rn, ("2R",), mine, noise, seed + 200)
    out["d"] = (solve_marks(rn, tq, known, {"2R": rn.nominal_tip()}), truth, tip, rt)
    return out


def test_pivot_exact_and_refusals(tmp_path):
    rt, rn, truth, tip = make_truth("config/two_arms", tmp_path, ("2L", "2R"))
    m = true_marks(rn, ("A", "B"))
    Q = simulate_touches(rt, rn, ("2L",), m, (0.0, 0.0))["2L"]["A"]
    pv = pivot(rn, "2L", Q)
    assert pv.passed and len(Q) == 4 and pv.spread > np.deg2rad(15)
    assert np.linalg.norm(pv.tip_hand - tip) < 1e-9
    a_base = (np.array([*m["A"], 0.0]) - truth["2L"][:3, 3]) @ truth["2L"][:3, :3]
    assert np.linalg.norm(pv.point_base - a_base) < 1e-9
    assert np.linalg.norm(touch_point(rn, "2L", Q[0], pv.tip_hand) - a_base) < 1e-9
    assert "at least 3 needed" in pivot(rn, "2L", Q[:2]).why
    # turning the hand about one axis only: the spread passes, the tip is not pinned
    arm = rt.arm("2L")
    n = truth["2L"][:3, :3].T[:, 2]
    Th = arm.hand_pose(np.repeat(a_base[None], 4, 0), n, np.deg2rad([0, 90, 180, 270]),
                       np.zeros((4, 2)))
    Qs = np.array([next(q[ok][0] for q7 in np.linspace(-2.8, 2.8, 29)
                        for q, ok in [tuple(x[0] for x in arm.ik(T[None], q7))] if ok.any())
                   for T in Th])
    r = pivot(rn, "2L", Qs)
    assert not r.passed and "tilt the hand" in r.why
    assert "span only" in pivot(rn, "2L", Qs[[0, 0, 0]] + [0, 0, 0, 0, 0, 0, 1e-3]).why
    slip = Q.copy()
    slip[2] = simulate_touches(rt, rn, ("2L",), {"A": m["A"] + [0.003, 0.0]}, (0.0, 0.0),
                               pivot_touches=4)["2L"]["A"][2]
    r = pivot(rn, "2L", slip)
    assert not r.passed and "pivot touch" in r.why and "slipped" in r.why


def test_marks_noiseless_exact(tmp_path):
    for label, (sol, truth, tip, rt) in scenarios(tmp_path, (0.0, 0.0), 0).items():
        assert sol.passed, (label, sol.why)
        xy, yaw, tp, z, seam = errors(sol, truth, tip, rt)
        assert xy < 1e-6 and yaw < 1e-6 and tp < 1e-6 and z < 1e-6 and seam < 1e-6, label
    assert sol.marks["A"].state == "known" and set(sol.slots) == {"2R"}


@pytest.mark.slow
def test_marks_noise_numbers(tmp_path, capsys):
    """The brief's noise (0.3 mm guiding, 0.3 mrad joints) and encoder-level noise; every
    number is printed.  Asserted: the brief's targets (x, y 0.3 mm, yaw 0.3 mrad, tip 0.2 mm)
    for (a) and (d) and seams under 0.4 mm everywhere at encoder-level noise; at the brief's
    noise only that every case passes with seams under 1.5 mm (see the page: the targets are
    below what one pivot of four touches and 0.3 mm guiding error can tell)."""
    for noise in (SPEC, ENCODER):
        worst = {}
        for seed in range(3):
            for label, (sol, truth, tip, rt) in scenarios(tmp_path / f"{noise}{seed}", noise,
                                                          seed).items():
                assert sol.passed, (label, sol.why)
                e = errors(sol, truth, tip, rt)
                worst[label] = np.maximum(worst.get(label, 0.0), e)
        with capsys.disabled():
            print(f"\n  marks, guiding {noise[0] * 1e3:.2f} mm, joints {noise[1] * 1e3:.2f} mrad,"
                  f" worst of 3 seeds:")
            for label, (xy, yaw, tp, z, seam) in worst.items():
                print(f"    ({label}) xy {xy * 1e3:.3f} mm, yaw {yaw * 1e3:.3f} mrad, tip "
                      f"{tp * 1e3:.3f} mm, z {z * 1e3:.3f} mm, seam {seam * 1e3:.3f} mm")
        if noise == SPEC:
            assert all(w[4] < 1.5e-3 for w in worst.values())
        else:
            for label in ("a", "d"):
                xy, yaw, tp, _, _ = worst[label]
                assert xy < 0.3e-3 and yaw < 0.3e-3 and tp < 0.2e-3, label
            assert all(w[4] < 0.4e-3 for w in worst.values())


def test_marks_refusals(tmp_path):
    rt, rn, truth, tip = make_truth("config", tmp_path / "six", SIX)
    m = true_marks(rn, tuple(rn.marks))
    full = simulate_touches(rt, rn, SIX, m, (0.0, 0.0))
    # 1R with one mark it shares (S12R) and one nobody else touched
    tq = {s: full[s] for s in ("2L", "2R")}
    tq["1R"] = {"R1a": full["1R"]["R1a"], "S12R": full["1R"]["S12R"]}
    r = solve_marks(rn, tq)
    assert not r.passed and r.why == "1R needs a partner: only 1 shared mark"
    # one slot, no A/B, nothing known
    r = solve_marks(rn, {"2R": {"S12R": full["2R"]["A"], "S23R": full["2R"]["S23R"]}})
    assert not r.passed and r.why.startswith("needs two anchors")
    # a mark touched by one slot only is solved but flagged
    tq = {s: full[s] for s in ("2L", "2R", "3L")}
    tq["3R"] = {k: v for k, v in full["3R"].items() if k != "R3b"}
    r = solve_marks(rn, tq)
    assert r.passed and r.marks["R3b"].note == "determined by one arm"
    assert any("R3b determined by one arm" in n for n in r.notes)
    # B moved between the two arms' touches: the pair check names it
    rt2, rn2, _, _ = make_truth("config/two_arms", tmp_path / "two", ("2L", "2R"))
    m2 = true_marks(rn2, ("A", "B"))
    tq = simulate_touches(rt2, rn2, ("2L",), m2, (0.0, 0.0))
    tq |= simulate_touches(rt2, rn2, ("2R",), {"A": m2["A"], "B": m2["B"] + [0.0, 0.003]},
                           (0.0, 0.0))
    r = solve_marks(rn2, tq)
    assert not r.passed and "2L and 2R disagree by 3.00 mm on the distance A-B" in r.why
    # a slot 45 mm from where rig.json hangs it
    rt3, rn3, _, _ = make_truth("config/two_arms", tmp_path / "far", ("2L", "2R"),
                                big=("2R", 0.045))
    tq = simulate_touches(rt3, rn3, ("2L", "2R"), true_marks(rn3, ("A", "B")), (0.0, 0.0))
    r = solve_marks(rn3, tq)
    assert not r.passed and "refused: 2R 0.04" in r.why and "wrong slot or wrong robot" in r.why
    # bad input
    assert "not a mark" in solve_marks(rn3, {"2L": {"Z": np.zeros((4, 7))}}).why
    assert "K x 7" in solve_marks(rn3, {"2L": {"A": np.zeros((4, 6))}}).why
    assert "no arm in slot" in solve_marks(rn3, {"1L": {"A": np.zeros((4, 7))}}).why


def test_mark_solution_written_and_loaded(tmp_path):
    rt, rn, truth, tip = make_truth("config/two_arms", tmp_path, ("2L", "2R"))
    m = true_marks(rn, ("A", "B"))
    sol = solve_marks(rn, simulate_touches(rt, rn, ("2L", "2R"), m, SPEC),
                      base_tips={s: rn.nominal_tip() for s in ("2L", "2R")})
    assert sol.passed
    cfg = tmp_path / "nom"
    assert base_tips(cfg, ("2L", "2R")) == {}          # make_truth's plane parts name no tip
    write_mark_solution(rn, sol, cfg, date="2026-10-05")
    assert np.allclose(base_tips(cfg, ("2L",))["2L"], sol.slots["2L"].tip_hand, atol=1e-9)
    for s in ("2L", "2R"):
        cal = read(cfg, s)
        assert cal["base"]["method"] == "marks" and cal["base"]["plane"]["method"] == "plane"
        assert cal["pen"]["method"] == "pivot" and cal["pen"]["reference_touch"] is None
    marks = json.loads((cfg / "calibration" / "marks.json").read_text())["marks"]
    assert marks["B"]["state"] == "solved" and marks["A"]["state"] == "solved"
    loaded = Rig.load(cfg)
    for s in ("2L", "2R"):
        assert loaded.calibrated(s), loaded.calibration_status(s)
        assert np.allclose(loaded.T_table_base(s), sol.slots[s].T_table_base, atol=1e-9)
        assert np.allclose(loaded.arm(s).tool.tip_hand, sol.slots[s].tip_hand, atol=1e-9)
    assert loaded.mark_state("B") == "solved"
    assert np.allclose(loaded.mark_xy("B"), sol.marks["B"].xy, atol=1e-9)
    with pytest.raises(ValueError):
        write_mark_solution(rn, MK._refuse("no"), cfg)
