"""Plan the spiral with the system planner and save the phase-1 motions of leader 71, the
input of the footprint speed test (tests/test_kernel_footprint.py).

    .venv/bin/python tests/oracle/make_collide_footprint_case.py
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tests"))


def main():
    from system_cases import CONFIG, drawing
    from aris.rig import Rig
    from aris.system.planner import plan_detailed
    rig = Rig.load(CONFIG)
    tagged, _, _ = plan_detailed(rig, drawing("spiral"), rig.rules(),
                                 cache_dir=str(ROOT / "out" / "cache"), workers=24)
    ts, qs, qds = [], [], []
    for phase, arm_id, m in (x[:3] for x in tagged):
        if phase == "phase 1" and arm_id == 71:
            ts.append(m.traj.t), qs.append(m.traj.q), qds.append(m.traj.qd)
    off = np.cumsum([0] + [len(t) for t in ts])
    np.savez_compressed(ROOT / "tests" / "data" / "collide_footprint_spiral71.npz",
                        t=np.concatenate(ts), q=np.concatenate(qs), qd=np.concatenate(qds),
                        offsets=off)
    print(len(ts), "motions,", off[-1], "samples")


if __name__ == "__main__":
    main()
