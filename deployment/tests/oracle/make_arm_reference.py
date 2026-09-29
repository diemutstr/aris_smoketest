"""Reference numbers from the OLD code for tests/test_kernel_arm.py.

    cd deployment && ARIS_RIG=proposed ARIS_TOOL=lateral ../.venv/bin/python \
        tests/oracle/make_arm_reference.py

Writes tests/data/arm_reference.npz.  Runs the old package only; never imports `aris`.
"""
import os
from pathlib import Path

import numpy as np

assert os.environ.get("ARIS_RIG") == "proposed" and os.environ.get("ARIS_TOOL") == "lateral", \
    "run with ARIS_RIG=proposed ARIS_TOOL=lateral"

from aris_sixarm import frames, ik, metrics, planner  # noqa: E402

OUT = Path(__file__).resolve().parents[1] / "data" / "arm_reference.npz"
N = 10_000
SEED = np.zeros(7)


def random_q(rng, n):
    return frames.FR3_MIN + rng.random((n, 7)) * (frames.FR3_MAX - frames.FR3_MIN)


def random_rotations(rng, n):
    q = rng.normal(size=(n, 4))
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    w, x, y, z = q.T
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
        np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
        np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1)], 1)


def old_ik(T_tcp, q7):
    Q, valid = ik.solve_batch(T_tcp, q7, SEED)
    return Q, valid


def main():
    assert frames.ACTIVE_TOOL == "lateral" and ik.has_batch()
    rng = np.random.default_rng(20260929)
    out = {}

    # 1. forward kinematics, tip, sigma_min, limit margin
    q = random_q(rng, N)
    L = frames.link_frames_many(q)
    T_tcp, _ = frames.fk_many(q)
    out.update(fk_q=q, fk_hand=L[:, 9], fk_links=L[:1000], fk_tcp=T_tcp[:1000],
               fk_tip=frames.tip_pos_many(q),
               fk_sigma=metrics.sigma_min_many(metrics.tip_jacobian_many(q)),
               fk_margin=frames.joint_margin_many(q))

    # 2. IK on reachable poses (FK of random configurations, their own q7) ...
    q = random_q(rng, N)
    T_tcp, _ = frames.fk_many(q)
    Q, valid = old_ik(T_tcp, q[:, 6])
    out.update(ik_q=q, ik_hand=frames.link_frames_many(q)[:, 9], ik_count=valid.sum(1),
               ik_Q=Q[:1000])
    # ... and on random poses in a 2 m cube, mostly out of reach: the solver's clamping cases
    T = np.tile(np.eye(4), (N, 1, 1))
    T[:, :3, :3] = random_rotations(rng, N)
    T[:, :3, 3] = rng.uniform(-1.0, 1.0, (N, 3))
    q7 = rng.uniform(frames.FR3_MIN[6], frames.FR3_MAX[6], N)
    Q, valid = old_ik(T, q7)
    hand = T.copy()
    hand[:, :3, 3] -= frames.D_HAND_TCP * T[:, :3, 2]
    out.update(ikr_hand=hand, ikr_q7=q7, ikr_count=valid.sum(1))

    # 3. the lateral-holder pose convention: R = rotz(phi) @ rotx(pi), then the tool-frame lean
    tip = rng.uniform(-0.5, 0.5, (2000, 3))
    phi = rng.uniform(-np.pi, np.pi, 2000)
    ang = np.deg2rad(15.0) * np.sqrt(rng.random(2000))
    d = rng.uniform(-np.pi, np.pi, 2000)
    lean = np.column_stack([ang * np.cos(d), ang * np.sin(d)])
    hand = np.tile(np.eye(4), (2000, 1, 1))
    off = frames.tool_offset()
    for k in range(2000):
        R = planner.tool_lean(frames.rotz(phi[k]) @ frames.rotx(np.pi), lean[k])
        hand[k, :3, :3] = R
        hand[k, :3, 3] = tip[k] - R @ off - frames.D_HAND_TCP * R[:, 2]
    out.update(hp_tip=tip, hp_phi=phi, hp_lean=lean, hp_hand=hand, hp_tool_offset=off)

    np.savez_compressed(OUT, **out)
    print(f"wrote {OUT}")
    print(f"reachable: {int(out['ik_count'].sum())} solutions over {N} poses, "
          f"{int((out['ik_count'] == 0).sum())} poses with none")
    print(f"random:    {int(out['ikr_count'].sum())} solutions over {N} poses, "
          f"{int((out['ikr_count'] > 0).sum())} poses with at least one")


if __name__ == "__main__":
    main()
