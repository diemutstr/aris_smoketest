"""Write a rig file for the arms that are actually mounted, from config/rig.json.

    .venv/bin/python tools/mounted_rig.py --arms 2L,2R --area 1.72 0.9 --centre 0 0 \\
        --out config/two_arms
    .venv/bin/python tools/mounted_rig.py --check config/two_arms      # is it still in step?

    .venv/bin/python tools/mounted_rig.py --arms 1R --out config/one_arm   # area computed

The written file is config/rig.json with `"mounted": false` on every other slot and a drawing
area (x by y, metres, around a centre in the table frame).  Given (`--area`, `--centre`, [0, 0]
by default), it must lie inside what the drawable maps allow, which the server checks when it
starts.  Not given, it is computed: the drawable maps of these arms (with the pen's press), the
centre at the middle of what they can draw (to 1 cm), the largest rectangle about it the
server accepts (`aris.system.area.admissible`, rounded down to 1 cm).
Everything else — steel, hangers, clearances, gates, pens — is copied, so there is one source of
truth and this file is derived from it; `--check` says whether the derived file still matches
its source.  A `calibration` link next to the file points at config/calibration, so the same
calibration files apply.
"""
import argparse
import hashlib
import json
import os
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "config" / "rig.json"
SLOT = re.compile(r"[123][LR]")


def derive(source: dict, slots: list[str], area: tuple[float, float] | None,
           centre: tuple[float, float] | None = None) -> dict:
    out = json.loads(json.dumps(source))
    names = [a["slot"] for a in out["slots"]["list"]]
    for s in slots:
        if not SLOT.fullmatch(s) or s not in names:
            raise SystemExit(f"no slot {s!r} in {SOURCE}; slots are {names}")
    for a in out["slots"]["list"]:
        a["mounted"] = a["slot"] in slots
    if area is not None:
        out["canvas"]["drawing_area_m"] = [float(area[0]), float(area[1])]
    if centre is not None:
        out["canvas"]["drawing_area_centre_m"] = [float(centre[0]), float(centre[1])]
    out["fences"] = fences(out["slots"]["list"], slots)
    if "marks" in out:                  # the marks and groups the controlled arms can do alone
        m = out["marks"]
        m["list"] = [e for e in m["list"] if set(e["shared_by"]) <= set(slots)]
        m["groups"] = {k: v for k, v in m.get("groups", {}).items() if set(v) <= set(slots)}
    out["canvas"]["drawing_area_note"] = (
        "Chosen for the mounted arms by hand (a first guess), around drawing_area_centre_m; it "
        "must lie inside the area the drawable maps give, which the server checks when it "
        "starts (aris/server/station.py).")
    out["about"] = {
        "derived_from": "config/rig.json",
        "source_digest": hashlib.blake2b(SOURCE.read_bytes(), digest_size=12).hexdigest(),
        "mounted": list(slots),
        "note": "Written by tools/mounted_rig.py: the rig with only these slots mounted. Do not "
                "edit by hand; change config/rig.json and run the tool again.",
    }
    return out


def computed_area(out_dir: Path) -> tuple[tuple, tuple]:
    """(area, centre) the mounted arms' drawable maps give, for the rig file in out_dir."""
    sys.path.insert(0, str(ROOT))
    import numpy as np
    from aris.rig import Rig
    from aris.system import area as area_mod, maps as maps_mod, phases
    from aris.system.settings import Settings
    rig = Rig.load(out_dir)
    rules = rig.rules()
    maps = maps_mod.load_or_build(rig, phases(rig), rules.gates, Settings(),
                                  ROOT / "out" / "cache", os.cpu_count() or 1, press=rules.press)
    if not maps:
        raise SystemExit("no drawable maps: no mounted arm")
    m0 = next(iter(maps.values()))
    union = np.zeros_like(m0.state, bool)
    for m in maps.values():
        union |= m.state == maps_mod.DRAWABLE
    if not union.any():
        raise SystemExit("the mounted arms can draw nowhere on the paper")
    X, Y = np.meshgrid(m0.x, m0.y, indexing="ij")
    centre = (round(float(X[union].mean()), 2), round(float(Y[union].mean()), 2))
    size = area_mod.admissible(maps, centre=centre)
    area = tuple(float(np.floor(s * 100.0) / 100.0) for s in size)
    if min(area) <= 0.0:
        raise SystemExit(f"no rectangle about {centre} lies inside what the arms can draw; "
                         "give --area and --centre")
    return area, centre


def fences(slot_list: list[dict], mounted: list[str]) -> dict:
    """Planes the mounted arms stay behind: one between every row that has no mounted arm and
    the nearest row that has one, halfway, square to the table's length.  An arm that is
    present but not controlled (switched off, hanging from its hanger) is thereby ignored as a
    body and fenced off as a region (Pete, 2026-10-01).  A row with one arm controlled gets no
    fence: an uncontrolled arm beside a controlled one in the same row is not guarded."""
    rows = sorted({round(float(a["axis_xy_m"][1]), 6) for a in slot_list})
    live = sorted({round(float(a["axis_xy_m"][1]), 6) for a in slot_list
                   if a["slot"] in mounted})
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
    planes += column_fences(slot_list, mounted)
    return dict(source="walls that hold in every phase and job: every controlled arm's whole body "
                       "stays on the normal's side, at the wall clearance. Written by "
                       "tools/mounted_rig.py for the rows without a controlled arm and for the "
                       "uncontrolled arms beside a controlled one in its row.",
                planes=planes)


def column_fences(slot_list: list[dict], mounted: list[str]) -> list[dict]:
    """For every slot not controlled in a row that has a controlled arm: a plane halfway
    between the row's two axes, square to x, normal toward the controlled arm.  The same plane
    from several rows is written once (orchestrator, 2026-10-02: the robots in the other slots
    hang there switched off)."""
    out = {}
    for row in sorted({a["slot"][0] for a in slot_list}):
        here = [a for a in slot_list if a["slot"][0] == row]
        live = [a for a in here if a["slot"] in mounted]
        dead = [a for a in here if a["slot"] not in mounted]
        if not live:
            continue                                     # the row fence covers it
        for d in dead:
            for m in live:
                xd, xm = float(d["axis_xy_m"][0]), float(m["axis_xy_m"][0])
                mid, sign = round(0.5 * (xd + xm), 6), 1.0 if xm > xd else -1.0
                name = f"fence_col_{'minus' if xd < xm else 'plus'}_x"
                if name in out and out[name]["point_m"][0] != mid:
                    name = f"{name}_{mid:+.3f}".replace(".", "p")
                rows = out.get(name, {}).get("rows", []) + [row]
                out[name] = dict(name=name, point_m=[mid, 0.0, 0.0], normal=[sign, 0.0, 0.0],
                                 rows=rows)
    planes = []
    for f in out.values():
        rows = f.pop("rows")
        f["source"] = (f"halfway between the columns at x = {f['point_m'][0]:+.4f} m: the "
                       f"uncontrolled arm(s) of row(s) {', '.join(rows)} on the far side")
        planes.append(f)
    return planes


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--arms", help="comma-separated slots that are mounted, e.g. 2R,3R")
    ap.add_argument("--area", nargs=2, type=float, metavar=("X", "Y"),
                    help="drawing area in metres, x by y (default: computed from the arms' maps)")
    ap.add_argument("--centre", nargs=2, type=float, metavar=("X", "Y"),
                    help="centre of the drawing area, table frame, metres (default 0 0)")
    ap.add_argument("--out", help="the config directory to write")
    ap.add_argument("--check", help="a config directory written by this tool: is it in step?")
    a = ap.parse_args()
    source = json.loads(SOURCE.read_text())
    if a.check:
        path = Path(a.check) / "rig.json"
        have = json.loads(path.read_text())
        canvas = have["canvas"]
        want = derive(source, have["about"]["mounted"], tuple(canvas["drawing_area_m"]),
                      tuple(canvas.get("drawing_area_centre_m", (0.0, 0.0))))
        if have == want:
            print(f"{path}: in step with {SOURCE}")
            return 0
        print(f"{path}: OUT OF STEP with {SOURCE}; run the tool again")
        return 1
    if not (a.arms and a.out):
        ap.error("--arms and --out are needed (or --check)")
    slots = [x.strip() for x in a.arms.split(",")]
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    link = out / "calibration"
    if not link.exists():
        os.symlink(os.path.relpath(ROOT / "config" / "calibration", out), link)
    area, centre = a.area, a.centre
    if area is None:                         # computed from the arms' maps
        (out / "rig.json").write_text(json.dumps(derive(source, slots, None, centre), indent=1))
        area, centre = computed_area(out)
    (out / "rig.json").write_text(json.dumps(derive(source, slots, area, centre), indent=1)
                                  + "\n")
    canvas = derive(source, slots, area, centre)["canvas"]
    area = canvas.get("drawing_area_m")
    centre = canvas.get("drawing_area_centre_m", [0.0, 0.0])
    print(f"wrote {out / 'rig.json'} for slots {slots}")
    print(f"drawing area {area[0]:.3f} x {area[1]:.3f} m about the centre "
          f"({centre[0]:+.3f}, {centre[1]:+.3f}) m" if area else
          f"no drawing area; centre ({centre[0]:+.3f}, {centre[1]:+.3f}) m")
    print("computed from the arms' drawable maps" if a.area is None else
          "given: it must lie inside the area the drawable maps give for these arms about that "
          "centre (the server refuses drawings otherwise)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
