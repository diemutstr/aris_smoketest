"""Reference numbers for tests/test_check.py, computed with the OLD code on an OLD plan.

The plan is the hardware-day hover schedule (assets/site/h0970/unknown_h0970_hover_schedule.npz):
arm 71 at h = 0.970 with the lateral holder, 48 frames per second.  This is the programme of
lesson L44: the old pacing read its acceleration at 48 Hz and passed it; resampled to 1 kHz
it was far over the driver's limit.

Run as its own process (the old package reads its rig and tool at import):

    ARIS_RIG=proposed ARIS_TOOL=lateral .venv/bin/python \
        tests/oracle/make_check_reference.py

Writes tests/data/check_old_plan.npz:
  t, q                 arm 71's frames (float64), seconds and radians
  tip_world            the old code's pen tip for every frame, old world frame (canvas corner)
  T_world_base         the old base pose of arm 71 at h = 0.970
  shift                old world -> table frame offset (subtract)
  old_acc_48, old_acc_1k   largest |second difference| per joint, at 48 Hz and after linear
                       resampling to 1 kHz (what the old executor sent)
  old_vel_48           largest |first difference| per joint at 48 Hz
"""
import os
import sys
from pathlib import Path

assert os.environ.get("ARIS_RIG") == "proposed" and os.environ.get("ARIS_TOOL") == "lateral", \
    "run with ARIS_RIG=proposed ARIS_TOOL=lateral"

import numpy as np

HERE = Path(__file__).resolve().parent
DEPLOY = HERE.parents[1]
REPO = DEPLOY.parent
sys.path.insert(0, str(REPO))

from aris_sixarm import frames                                          # noqa: E402
from aris_sixarm.fleet import FLEET                                     # noqa: E402

H = 0.970
SHIFT = np.array([0.9017, 1.81532, 0.0])
ARM = 71
PLAN = REPO / "assets" / "site" / "h0970" / "unknown_h0970_hover_schedule.npz"


def main():
    d = np.load(PLAN)
    fps = float(d["fps"])
    q = np.asarray(d[f"q_{ARM}"], float)
    t = np.arange(len(q)) / fps
    T = FLEET[ARM].T_world_base(H)
    tip_base = frames.tip_pos_many(q)
    tip_world = tip_base @ T[:3, :3].T + T[:3, 3]
    vel48 = np.abs(np.diff(q, 1, 0)).max(0) * fps
    acc48 = np.abs(np.diff(q, 2, 0)).max(0) * fps ** 2
    t1k = np.arange(0.0, t[-1], 1e-3)
    q1k = np.stack([np.interp(t1k, t, q[:, j]) for j in range(7)], 1)
    acc1k = np.abs(np.diff(q1k, 2, 0)).max(0) * 1e6
    print(f"arm {ARM}: {len(q)} frames, {t[-1]:.2f} s; old acceleration reading, worst joint: "
          f"{acc48.max():.2f} rad/s^2 at 48 Hz, {acc1k.max():.1f} rad/s^2 at 1 kHz")
    out = DEPLOY / "tests" / "data" / "check_old_plan.npz"
    np.savez(out, t=t, q=q, tip_world=tip_world, T_world_base=T, shift=SHIFT,
             old_acc_48=acc48, old_acc_1k=acc1k, old_vel_48=vel48, arm=ARM, h=H)
    print("wrote", out)


if __name__ == "__main__":
    main()
