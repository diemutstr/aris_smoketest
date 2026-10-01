"""Write a rig file for the arms that are actually mounted, from config/rig.json.

    .venv/bin/python tools/mounted_rig.py --arms 31,71 --area 1.2 1.0 --out config/two_arms
    .venv/bin/python tools/mounted_rig.py --check config/two_arms      # is it still in step?

The written file is config/rig.json with `"mounted": false` on every other arm and the given
drawing area (x by y, metres, centred on the table; it must lie inside what the drawable maps
allow, which the server checks when it starts).  Everything else — steel, hangers, clearances,
gates — is copied, so there is one source of truth and this file is derived from it; `--check`
says whether the derived file still matches its source.  A `calibration` link next to the file
points at config/calibration, so the same calibration files apply.
"""
import argparse
import hashlib
import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "config" / "rig.json"


def derive(source: dict, arms: list[int], area: tuple[float, float] | None) -> dict:
    out = json.loads(json.dumps(source))
    ids = [int(a["id"]) for a in out["arms"]["list"]]
    for a in arms:
        if a not in ids:
            raise SystemExit(f"no arm {a} in {SOURCE}; arms are {ids}")
    for a in out["arms"]["list"]:
        a["mounted"] = int(a["id"]) in arms
    if area is not None:
        out["canvas"]["drawing_area_m"] = [float(area[0]), float(area[1])]
    out["fences"] = fences(out["arms"]["list"], arms)
    out["canvas"]["drawing_area_note"] = (
        "Chosen for the mounted arms by hand (conservative); it must lie inside the area the "
        "drawable maps give, which the server checks when it starts (aris/server/station.py).")
    out["about"] = {
        "derived_from": "config/rig.json",
        "source_digest": hashlib.blake2b(SOURCE.read_bytes(), digest_size=12).hexdigest(),
        "mounted": arms,
        "note": "Written by tools/mounted_rig.py: the rig with only these arms mounted. Do not "
                "edit by hand; change config/rig.json and run the tool again.",
    }
    return out


def fences(arm_list: list[dict], mounted: list[int]) -> dict:
    """Planes the mounted arms stay behind: one between every row that has no mounted arm and
    the nearest row that has one, halfway, square to the table's length.  An arm that is
    present but not controlled (switched off, hanging from its hanger) is thereby ignored as a
    body and fenced off as a region (Pete, 2026-10-01)."""
    rows = sorted({round(float(a["axis_xy_m"][1]), 6) for a in arm_list})
    live = sorted({round(float(a["axis_xy_m"][1]), 6) for a in arm_list if int(a["id"]) in mounted})
    planes = []
    for y in rows:
        if y in live or not live:
            continue
        near = min(live, key=lambda v: abs(v - y))
        mid, sign = 0.5 * (y + near), 1.0 if near > y else -1.0
        planes.append(dict(name=f"fence_row_{y:+.3f}".replace(".", "p"),
                           point_m=[0.0, round(mid, 6), 0.0], normal=[0.0, sign, 0.0],
                           source=f"halfway between the row at y = {y:+.4f} m (no arm controlled) "
                                  f"and the nearest controlled row at y = {near:+.4f} m"))
    return dict(source="walls that hold in every phase and job: every controlled arm's whole body "
                       "stays on the normal's side, at the wall clearance. Written by "
                       "tools/mounted_rig.py for the rows without a controlled arm.",
                planes=planes)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--arms", help="comma-separated arm ids that are mounted")
    ap.add_argument("--area", nargs=2, type=float, metavar=("X", "Y"),
                    help="drawing area in metres, centred on the table")
    ap.add_argument("--out", help="the config directory to write")
    ap.add_argument("--check", help="a config directory written by this tool: is it in step?")
    a = ap.parse_args()
    source = json.loads(SOURCE.read_text())
    if a.check:
        path = Path(a.check) / "rig.json"
        have = json.loads(path.read_text())
        want = derive(source, have["about"]["mounted"], tuple(have["canvas"]["drawing_area_m"]))
        if have == want:
            print(f"{path}: in step with {SOURCE}")
            return 0
        print(f"{path}: OUT OF STEP with {SOURCE}; run the tool again")
        return 1
    if not (a.arms and a.out):
        ap.error("--arms and --out are needed (or --check)")
    arms = [int(x) for x in a.arms.split(",")]
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "rig.json").write_text(json.dumps(derive(source, arms, a.area), indent=1) + "\n")
    link = out / "calibration"
    if not link.exists():
        os.symlink(os.path.relpath(ROOT / "config" / "calibration", out), link)
    print(f"wrote {out / 'rig.json'} for arms {arms}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
