"""The plane calibration (aris/calib, DESIGN.md section 6 step 1) on synthetic touches.

The kernel's forward kinematics is the truth: a true paper plane is invented (the nominal one
tilted and raised by known amounts), the pen tip is put exactly on it at a grid of points by the
IK (pen upright, one hand spin), the joints get noise, and the solver must find the tilt and the
height again.
"""
import json
import shutil

import numpy as np
import pytest

from aris.calib import calibrate_plane, calibration_from_events, fit_plane, write_calibration
from aris.calib import plane as P
from aris.rig import Rig

CASES = [("config/two_arms", 31, (1.5, -2.0), 0.020, 0.0),
         ("config/two_arms", 71, (-2.0, 0.7), -0.015, 1.2),
         ("config", 13, (0.4, 1.8), 0.008, -0.6)]


def _rotvec(v):
    return P._rotvec_to_matrix(np.asarray(v, float))


def true_pose(rig, arm_id, tilt_deg, dz):
    """The nominal base pose turned by (roll, pitch, 0) about the base origin and raised by dz:
    the same convention the solver writes."""
    T = rig.T_table_base(arm_id)
    T[:3, :3] = _rotvec(np.deg2rad([tilt_deg[0], tilt_deg[1], 0.0])) @ T[:3, :3]
    T[2, 3] += dz
    return T


def touches(rig, arm_id, T_true, spin, noise=0.0, seed=0, half=0.2, n=5):
    """Contact configurations whose pen tips lie on the paper z = 0 as the true pose sees it,
    on an n x n grid of +-half around the arm's axis, plus joint noise (rad, per joint)."""
    arm = rig.arm(arm_id)
    R, t = T_true[:3, :3], T_true[:3, 3]
    ax = rig.T_table_base(arm_id)[:2, 3]
    g = np.linspace(-half, half, n)
    p_table = np.array([[ax[0] + x, ax[1] + y, rig.paper_z] for x in g for y in g])
    p_base = (p_table - t) @ R                      # R^T (p - t), row-wise
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
    assert np.all(np.isfinite(best_q)), "test set-up: a grid point is out of reach"
    assert np.allclose(arm.tip(best_q), p_base, atol=1e-6)
    rng = np.random.default_rng(seed)
    return best_q + noise * rng.standard_normal(best_q.shape)


@pytest.fixture(scope="module")
def rigs():
    return {c: Rig.load(c) for c in ("config/two_arms", "config")}


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


# ------------------------------------------------------------------ recovery

@pytest.mark.parametrize("cfg,arm_id,tilt_deg,dz,spin", CASES)
def test_recovers_tilt_and_height(rigs, cfg, arm_id, tilt_deg, dz, spin):
    rig = rigs[cfg]
    T_true = true_pose(rig, arm_id, tilt_deg, dz)
    Q = touches(rig, arm_id, T_true, spin, noise=0.5e-3, seed=arm_id)
    r = calibrate_plane(rig, arm_id, Q)
    assert r.passed, r.why
    assert r.n_points == 25
    assert abs(np.rad2deg(r.roll) - tilt_deg[0]) < 0.05
    assert abs(np.rad2deg(r.pitch) - tilt_deg[1]) < 0.05
    assert abs(r.height_change - dz) < 0.2e-3
    # x, y and the turn about the vertical are untouched
    T_nom = rig.T_table_base(arm_id)
    assert np.allclose(r.T_table_base[:2, 3], T_nom[:2, 3], atol=0)
    assert np.allclose(r.T_table_base[:3, :3], T_true[:3, :3], atol=1e-3)
    # the height map: in the table frame, z is the residual
    assert r.points_table.shape == (25, 3)
    assert np.allclose(r.points_table[:, 2] - rig.paper_z, r.residuals, atol=1e-12)


def test_noise_numbers(rigs, capsys):
    """Report recovered errors at 0.5 and 2 mrad joint noise (asserts only the 0.5 mrad case,
    which the recovery test already covers more tightly)."""
    rig = rigs["config/two_arms"]
    T_true = true_pose(rig, 31, (1.5, -2.0), 0.020)
    for noise in (0.5e-3, 2e-3):
        e_tilt, e_h, rms = [], [], []
        for seed in range(10):
            r = calibrate_plane(rig, 31, touches(rig, 31, T_true, 0.0, noise, seed))
            e_tilt.append(max(abs(np.rad2deg(r.roll) - 1.5), abs(np.rad2deg(r.pitch) + 2.0)))
            e_h.append(abs(r.height_change - 0.020))
            rms.append(r.rms)
        with capsys.disabled():
            print(f"\n  noise {noise * 1e3:.1f} mrad, 10 seeds: tilt error max "
                  f"{max(e_tilt):.4f} deg, height error max {max(e_h) * 1e3:.4f} mm, "
                  f"fit RMS mean {np.mean(rms) * 1e3:.3f} mm")
        if noise == 0.5e-3:
            assert max(e_tilt) < 0.05 and max(e_h) < 0.2e-3


# ------------------------------------------------------------------ refusals

def test_far_plane_refused(rigs):
    rig = rigs["config/two_arms"]
    r = calibrate_plane(rig, 31, touches(rig, 31, true_pose(rig, 31, (0.5, 0.5), 0.040), 0.0,
                                         0.5e-3, 3))
    assert not r.passed
    assert "from its nominal height" in r.why and "off the plane" not in r.why
    assert abs(r.height_change - 0.040) < 0.2e-3          # still measured, just not trusted


def test_large_tilt_refused(rigs):
    rig = rigs["config/two_arms"]
    r = calibrate_plane(rig, 71, touches(rig, 71, true_pose(rig, 71, (3.5, 0.0), 0.0), 0.0))
    assert not r.passed and "leans" in r.why and "height" not in r.why


def test_slipped_touch_refused(rigs):
    rig = rigs["config/two_arms"]
    T_true = true_pose(rig, 31, (1.0, 1.0), 0.010)
    Q_good = touches(rig, 31, T_true, 0.0, 0.5e-3, 4)
    # touch 7 lands 5 mm above the paper: the same grid point on a plane raised by 5 mm
    T_slip = T_true.copy()
    T_slip[2, 3] -= 0.005
    Q = Q_good.copy()
    Q[7] = touches(rig, 31, T_slip, 0.0, 0.5e-3, 4)[7]
    r = calibrate_plane(rig, 31, Q)
    assert not r.passed and r.worst_index == 7
    assert "touch 7 is" in r.why and "slipped" in r.why
    assert r.max_residual > P.MAX_RESIDUAL_M


@pytest.mark.parametrize("Q,words", [
    (np.zeros((0, 7)), "no touches"),
    (np.zeros((2, 7)), "only 2 touches"),
    (np.zeros((5, 6)), "N x 7"),
])
def test_degenerate_inputs_are_refusals(rigs, Q, words):
    r = calibrate_plane(rigs["config"], 13, Q)
    assert not r.passed and words in r.why
    assert np.array_equal(r.T_table_base, rigs["config"].T_table_base(13))


def test_collinear_and_unreadable_are_refusals(rigs):
    rig = rigs["config"]
    Q = touches(rig, 13, rig.T_table_base(13), 0.0)
    line = Q[[0, 1, 2, 3, 4]]                                # one grid row: a line
    r = calibrate_plane(rig, 13, line)
    assert not r.passed and "on a line" in r.why
    r = calibrate_plane(rig, 13, np.repeat(Q[:1], 12, axis=0))   # one spot, twelve times
    assert not r.passed and "on a line" in r.why
    bad = Q.copy()
    bad[3, 2] = np.nan
    r = calibrate_plane(rig, 13, bad)
    assert not r.passed and "touch 3 has no joint reading" in r.why
    few = Q[[0, 4, 12, 20, 24]]                              # a plane, but too few points
    r = calibrate_plane(rig, 13, few)
    assert not r.passed and "only 5 touches" in r.why


# ------------------------------------------------------------------ events and the file

def _rows(arm_id, Q, other=None):
    rows = [{"event": "motion started", "arm": arm_id, "index": 0}]
    for k, q in enumerate(Q):
        rows.append({"event": "contact", "arm": arm_id, "q": list(map(float, q)), "index": k,
                     "phase": 0, "time": 0.0})
        if other is not None:
            rows.append({"event": "contact", "arm": other, "q": [0.0] * 7, "index": k})
    return rows


def test_from_events_picks_this_arm_in_order(rigs):
    rig = rigs["config/two_arms"]
    Q = touches(rig, 31, true_pose(rig, 31, (1.0, -1.0), 0.005), 0.0, 0.5e-3, 5)
    a = calibration_from_events(rig, 31, _rows(31, Q, other=71))
    b = calibrate_plane(rig, 31, Q)
    assert a.passed and np.array_equal(a.T_table_base, b.T_table_base)
    assert np.array_equal(a.residuals, b.residuals)
    assert not calibration_from_events(rig, 71, _rows(31, Q)).passed     # no rows for 71


def test_written_file_loads_through_rig(rigs, tmp_path):
    rig = rigs["config/two_arms"]
    T_true = true_pose(rig, 31, (1.2, -0.8), 0.012)
    r = calibrate_plane(rig, 31, touches(rig, 31, T_true, 0.0, 0.5e-3, 6))
    assert r.passed
    shutil.copy("config/two_arms/rig.json", tmp_path / "rig.json")
    write_calibration(r, tmp_path / "calibration" / "31.json", date="2026-10-01")
    cal = json.loads((tmp_path / "calibration" / "31.json").read_text())
    assert cal["passed"] is True and "why" not in cal and "tip_hand_m" not in cal
    assert cal["n_points"] == 25 and len(cal["height_map_table_m"]) == 25
    assert abs(cal["height_change_mm"] - 12.0) < 0.2 and cal["rms_mm"] < 0.5

    loaded = Rig.load(tmp_path)
    assert loaded.calibration_status(31) == "applied: 31.json (2026-10-01)"
    assert loaded.calibration_status(71) == "none"
    T = loaded.T_table_base(31)
    assert np.allclose(T, r.T_table_base, atol=1e-12)
    assert np.allclose(T[:3, :3], T_true[:3, :3], atol=1e-3)
    assert abs(T[2, 3] - T_true[2, 3]) < 0.2e-3
    assert np.array_equal(T[:2, 3], rig.T_table_base(31)[:2, 3])
    assert loaded.mounts[31].tip_hand is None
    # the paper seen from the calibrated base is the fitted plane
    paper = loaded.paper(31)
    assert np.allclose(paper.normal, r.normal_base, atol=1e-9)

    # a failed calibration is written with its reason, and the rig does not apply it
    bad = calibrate_plane(rig, 71, touches(rig, 71, true_pose(rig, 71, (0, 0), 0.040), 0.0))
    write_calibration(bad, tmp_path / "calibration" / "71.json", date="2026-10-01")
    assert "why" in json.loads((tmp_path / "calibration" / "71.json").read_text())
    loaded = Rig.load(tmp_path)
    assert loaded.calibration_status(71).startswith("not applied")
    assert np.array_equal(loaded.T_table_base(71), rig.T_table_base(71))
