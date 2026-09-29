"""Reference numbers for tests/test_rig.py, computed with the OLD code.

Run as its own process (the old package reads its rig and tool at import):

    cd deployment && ARIS_RIG=proposed ARIS_TOOL=lateral ../.venv/bin/python \
        tests/oracle/make_rig_reference.py

Writes tests/data/rig_reference.npz:
  arm_ids, T_canvas_base            the old base poses at h = 0.970, canvas frame
  old_box_*                         the old spec.static_obstacles() of every arm, canvas frame
  park_*                            the old model's clearances of every park configuration:
                                    paper (chain centres and capsule surfaces), pen tip height,
                                    self, and steel (old static set, and the NEW steel set read
                                    from config/rig.json, each box scored separately; and the
                                    arm's own struts, which the rig leaves out, on their own)
  reach_sampled                     the furthest any old body capsule surface got from the
                                    shoulder over random configurations
"""
import os
import sys
from pathlib import Path

assert os.environ.get("ARIS_RIG") == "proposed" and os.environ.get("ARIS_TOOL") == "lateral", \
    "run with ARIS_RIG=proposed ARIS_TOOL=lateral"

import numpy as np

HERE = Path(__file__).resolve().parent
DEPLOY = HERE.parents[1]
sys.path.insert(0, str(DEPLOY))

from aris_sixarm import rig_final, selfcoll, validate                    # noqa: E402
from aris_sixarm.fleet import FLEET                                     # noqa: E402
from aris_sixarm.frames import FR3_MAX, FR3_MIN                         # noqa: E402
from aris.rig import Rig                                                # noqa: E402

H = 0.970
SHIFT = np.array([0.9017, 1.81532, 0.0])      # canvas corner -> table centre


class _Shim:
    """Just enough of an old ArmSpec for validate.check_pose, with a chosen box set."""
    def __init__(self, spec, boxes):
        self._spec, self._boxes = spec, boxes
        self.mount, self.rig = spec.mount, "shim"      # "shim": skip the own-boom proxy

    def T_world_base(self, h_inv=None):
        return self._spec.T_world_base(H)

    def static_obstacles(self):
        return list(self._boxes)


def _new_steel_canvas(rig, arm_id, own_struts=False):
    """The new steel as the rig hands it to `arm_id`, or only that arm's own struts."""
    if own_struts:
        keep = [b for b in rig.steel if b.name.startswith(f"strut{arm_id}_")]
    else:
        keep = [b for b in rig.steel if arm_id not in b.not_for]
    return [dict(name=b.name, lo=b.lo_table + SHIFT, hi=b.hi_table + SHIFT) for b in keep]


def _capsule_paper(q, Twb):
    A, B, R = selfcoll.capsule_ends(q[None])
    zA = (A[0] @ Twb[:3, :3].T + Twb[:3, 3])[:, 2]
    zB = (B[0] @ Twb[:3, :3].T + Twb[:3, 3])[:, 2]
    return float(np.min(np.minimum(zA, zB) - R))


def _reach(n=200_000, seed=7):
    rng = np.random.default_rng(seed)
    q = rng.uniform(FR3_MIN, FR3_MAX, size=(n, 7))
    A, B, R = selfcoll.capsule_ends(q)
    s = np.array([0.0, 0.0, 0.333])
    d = np.maximum(np.linalg.norm(A - s, axis=2), np.linalg.norm(B - s, axis=2)) + R
    return float(d.max())


def main():
    rig = Rig.load(DEPLOY / "config")
    ids = list(FLEET)
    T = np.stack([FLEET[a].T_world_base(H) for a in ids])
    names, lo, hi, owner = [], [], [], []
    for a in ids:
        for b in FLEET[a].static_obstacles():
            names.append(b["name"]); lo.append(b["lo"]); hi.append(b["hi"]); owner.append(a)
    park = {k: [] for k in ("q", "chain_z", "caps_paper", "tip_z", "self", "old_steel",
                            "new_steel", "new_steel_box", "own_struts")}
    for a in ids:
        spec, q = FLEET[a], FLEET[a].q_seed
        new = _new_steel_canvas(rig, a)
        rep_old = validate.check_pose(q, _Shim(spec, spec.static_obstacles()))
        rep_new = validate.check_pose(q, _Shim(spec, new))
        per_box = [validate.check_pose(q, _Shim(spec, [b]))["worst"]["min_frame_clearance"]
                   for b in new]
        k = int(np.argmin(per_box))
        own = validate.check_pose(q, _Shim(spec, _new_steel_canvas(rig, a, own_struts=True)))
        park["own_struts"].append(own["worst"]["min_frame_clearance"])
        park["q"].append(q)
        park["chain_z"].append(rep_new["worst"]["min_chain_z"])
        park["caps_paper"].append(_capsule_paper(q, spec.T_world_base(H)))
        park["tip_z"].append(rep_new["worst"]["tip_z"])
        park["self"].append(rep_new["worst"]["min_self_clearance"])
        park["old_steel"].append(rep_old["worst"]["min_frame_clearance"])
        park["new_steel"].append(rep_new["worst"]["min_frame_clearance"])
        park["new_steel_box"].append(new[k]["name"])
        print(f"arm {a:3d}: paper(chain) {park['chain_z'][-1]:.4f}  paper(caps) "
              f"{park['caps_paper'][-1]:.4f}  tip {park['tip_z'][-1]:.4f}  self "
              f"{park['self'][-1]:.4f}  steel old {park['old_steel'][-1]:.4f}  new "
              f"{park['new_steel'][-1]:.4f} ({new[k]['name']})  own struts "
              f"{park['own_struts'][-1]:.4f}")
    reach = _reach()
    print(f"reach from shoulder, old capsules, 200k random configurations: {reach:.4f}")
    out = DEPLOY / "tests" / "data" / "rig_reference.npz"
    np.savez(out, arm_ids=np.array(ids), T_canvas_base=T, h=H, shift=SHIFT,
             old_box_name=np.array(names), old_box_lo=np.array(lo), old_box_hi=np.array(hi),
             old_box_owner=np.array(owner),
             **{f"park_{k}": np.array(v) for k, v in park.items()},
             static_margin=rig_final.STATIC_MARGIN, reach_sampled=reach)
    print("wrote", out)


if __name__ == "__main__":
    main()
