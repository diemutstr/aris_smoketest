"""The rig's steel, from above, with arm 31's hanger dimensioned from above and from the side;
every number from Rig and config/rig.json.  Run: .venv/bin/python tools/draw_rig_steel.py"""
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle

from aris.rig import Rig

ROOT = Path(__file__).resolve().parents[1]
ARM = 31
MM = 1e3
RED = "#b22"


def dim(ax, p, q, off, text, fs=6.5):
    """A dimension line from p to q, moved `off` square to it (+: left of p->q), text on it."""
    p, q = np.asarray(p, float), np.asarray(q, float)
    d = (q - p) / np.linalg.norm(q - p)
    n = np.array([-d[1], d[0]]) * off
    for e in (p, q):
        ax.plot(*np.c_[e, e + 1.1 * n], color="#666", lw=0.4)
    ax.annotate("", q + n, p + n, arrowprops=dict(arrowstyle="<->", lw=0.6, color=RED))
    ax.text(*(0.5 * (p + q) + n), text, fontsize=fs, color=RED, ha="center", va="center",
            rotation=np.degrees(np.arctan2(d[1], d[0])) % 180,
            bbox=dict(fc="white", ec="none", pad=0.4))


def box(ax, b, axes, **kw):
    ax.add_patch(Rectangle(b.lo_table[axes], *(b.hi_table - b.lo_table)[axes], **kw))


def overview(ax, rig):
    for size, style in ((rig.table_size, "-"), (rig.canvas_size, "--")):
        ax.add_patch(Rectangle(-size / 2, *size, fill=False, ls=style, lw=0.8, ec="#8a6a40"))
    for b in rig.steel:
        box(ax, b, [0, 1], fc="#ccc", ec="#333", lw=0.5, alpha=0.8)
        c, s = 0.5 * (b.lo_table + b.hi_table), b.hi_table - b.lo_table
        if b.owner is None:            # hangers are named once per arm below
            ax.text(c[0], c[1] + {"runway": 0.1, "seam_b": 0.6}.get(b.name[:6], 0), b.name,
                    fontsize=5, ha="center", va="center", rotation=90 if s[1] > s[0] else 0)
    for n, style in ((1, "-"), (2, "--")):
        for i, w in enumerate(rig.phase(n).walls):
            d = np.array([-w.normal_table[1], w.normal_table[0]])
            p = w.point_table[:2] + np.array([-0.9, 0.9])[:, None] * d
            ax.plot(p[:, 0], p[:, 1], style, color="#c33", lw=0.9,
                    label=f"walls, phase {n}" if i == 0 else None)
    for a in rig.arm_ids:
        c = rig.T_table_base(a)[:2, 3]
        ax.plot(*c, "+", color="k", ms=7)
        ax.text(c[0] + 0.03, c[1] + 0.13, str(a), fontsize=9, weight="bold")
        ax.text(c[0], c[1] - 0.14, f"strut{a}_minus_x,\nstrut{a}_plus_x,\nplate{a}, clamp{a}",
                fontsize=4.5, ha="center", va="top")
    (x31, y31), x71 = rig.T_table_base(31)[:2, 3], rig.T_table_base(71)[0, 3]
    y13, y2 = rig.T_table_base(13)[1, 3], rig.T_table_base(2)[1, 3]
    dim(ax, (x31, y31), (x71, y31), -0.45, f"column spacing {(x71 - x31) * MM:.1f}")
    dim(ax, (x71, y13), (x71, y31), -0.5, f"row spacing {(y31 - y13) * MM:.1f}")
    dim(ax, (x71, y31), (x71, y2), -0.5, f"row spacing {(y2 - y31) * MM:.1f}")
    T = rig.table_size / 2 + 0.05
    ax.set(xlim=(-T[0], T[0]), ylim=(-T[1], T[1]), aspect="equal")
    ax.set_title("top view, table frame (m): table, canvas (dashed),\nall steel boxes, "
                 "arm axes, walls", fontsize=9)
    ax.legend(fontsize=6, loc="lower left")


def hanger_top(ax, rig, h, bx):
    x0, y0 = rig.T_table_base(ARM)[:2, 3]
    for n, b in bx.items():
        box(ax, b, [0, 1], fc="#ddd" if "strut" in n else "none", ec="#333", lw=0.8,
            ls=":" if "clamp" in n else "-")
    w, nr, p, c = (bx[n.format(ARM)] for n in ("strut{}_minus_x", "strut{}_plus_x", "plate{}",
                                                 "clamp{}"))
    for b in (w, nr):
        ax.text(np.mean([b.lo_table[0], b.hi_table[0]]), b.lo_table[1] - 0.015, b.name,
                fontsize=6, ha="center", va="top")
    ax.text(x0, p.hi_table[1] + 0.008, f"plate{ARM} (solid), clamp{ARM} (dotted)",
            fontsize=6, ha="center")
    ax.plot(x0, y0, "+", color="k", ms=12), ax.axhline(y0, color="#777", lw=0.5, ls="-.")
    ax.text(x0 + 0.004, y0 + 0.008, f"axis {ARM}", fontsize=6)
    sx, sy = h["strut_size_x_m"] * MM, h["strut_size_y_m"] * MM
    lo, hi = w.lo_table, w.hi_table
    dim(ax, (lo[0], hi[1]), (hi[0], hi[1]), 0.03, f"{sx:.1f}")
    dim(ax, (nr.lo_table[0], hi[1]), (nr.hi_table[0], hi[1]), 0.03, f"{sx:.1f}")
    dim(ax, (lo[0], lo[1]), (lo[0], hi[1]), 0.025, f"{sy:.1f}")
    dim(ax, (lo[0], lo[1]), (x0, lo[1]), -0.065,
        f"outer face to axis {h['axis_to_minus_x_outer_face_m'] * MM:.2f}")
    dim(ax, (x0, lo[1]), (nr.hi_table[0], lo[1]), -0.065,
        f"outer face to axis {h['axis_to_plus_x_outer_face_m'] * MM:.2f}")
    dim(ax, p.lo_table[:2], (p.hi_table[0], p.lo_table[1]), -0.1,
        f"plate {h['plate_size_x_m'] * MM:.2f} x {h['plate_size_y_m'] * MM:.0f}")
    dim(ax, (c.lo_table[0], c.hi_table[1]), c.hi_table[:2], 0.075,
        f"clamp {h['clamp_size_x_m'] * MM:.0f} x {h['clamp_size_y_m'] * MM:.1f}")
    pc = 0.5 * (p.lo_table[0] + p.hi_table[0])
    dim(ax, (x0, y0), (pc, y0), 0.035,
        f"plate centre {h['plate_centre_offset_x_m'] * MM:.2f} toward +x")
    off = 0.5 * (w.lo_table[1] + w.hi_table[1]) - y0
    ax.text(x0, y0 - 0.235, f"struts centred on the row line: y offset {off * MM:.1f}",
            fontsize=6.5, ha="center")
    ax.set(xlim=(x0 - 0.24, x0 + 0.25), ylim=(y0 - 0.25, y0 + 0.2), aspect="equal")
    ax.set_title(f"arm {ARM} hanger from above (mm; axes in m, table frame)", fontsize=9)


def hanger_side(ax, rig, h, bx, runway):
    y0 = rig.T_table_base(ARM)[1, 3]
    for b in list(bx.values()) + [runway]:
        box(ax, b, [1, 2], fc="#ddd", ec="#333", lw=0.7, alpha=0.6)
    ax.axhline(rig.paper_z, color="#8a6a40", lw=1)
    ax.text(y0 + 0.2, rig.paper_z + 0.02, f"paper z = {rig.paper_z * MM:.0f}", fontsize=6.5)
    levels = [("strut bottom", h["strut_bottom_z_m"], -0.02), ("plate underside (mount plane)",
              h["plate_bottom_z_m"], -0.01), ("plate top = clamp bottom", h["clamp_bottom_z_m"],
              0.025), ("clamp top", h["clamp_top_z_m"], 0), ("strut top = runway underside",
              h["strut_top_z_m"], -0.015), (f"{runway.name} top", runway.hi_table[2], 0.015)]
    for name, z, dz in levels:
        ax.plot([y0 + 0.1, y0 + 0.19, y0 + 0.2], [z, z, z + dz], color=RED, lw=0.5)
        ax.text(y0 + 0.205, z + dz, f"{name}  {z * MM:.2f}", fontsize=6, color=RED,
                va="center")
    dim(ax, (y0 - 0.12, rig.paper_z), (y0 - 0.12, h["plate_bottom_z_m"]), 0.0,
        f"plate underside {(h['plate_bottom_z_m'] - rig.paper_z) * MM:.0f} above the paper")
    dim(ax, (y0 - 0.2, h["strut_bottom_z_m"]), (y0 - 0.2, h["strut_top_z_m"]), 0.0,
        f"strut {(h['strut_top_z_m'] - h['strut_bottom_z_m']) * MM:.1f} long")
    ax.set(xlim=(y0 - 0.28, y0 + 0.62), ylim=(-0.05, 1.76), aspect="equal")
    ax.set_xlabel("y (m)", fontsize=7), ax.set_ylabel("z (m)", fontsize=7)
    ax.set_title(f"arm {ARM} hanger seen along the row (y-z, mm);\nthe two struts overlap "
                 f"in this view; {runway.name} behind", fontsize=9)


def main():
    rig = Rig.load(ROOT / "config")
    h = json.loads((ROOT / "config" / "rig.json").read_text())["hanger"]
    bx = {b.name: b for b in rig.steel if b.owner == ARM}
    y0 = rig.T_table_base(ARM)[1, 3]
    runway = next(b for b in rig.steel if b.name.startswith("runway")
                  and b.lo_table[1] <= y0 <= b.hi_table[1])
    plt.switch_backend("Agg")
    fig = plt.figure(figsize=(8.27, 11.69))
    gs = fig.add_gridspec(2, 2, width_ratios=[1, 1.1])
    overview(fig.add_subplot(gs[:, 0]), rig)
    hanger_top(fig.add_subplot(gs[0, 1]), rig, h, bx)
    hanger_side(fig.add_subplot(gs[1, 1]), rig, h, bx, runway)
    fig.suptitle("Rig steel from config/rig.json", fontsize=11)
    [ax.tick_params(labelsize=6) for ax in fig.axes]
    fig.tight_layout()
    fig.savefig(ROOT / "out" / "rig_steel_topdown.png", dpi=200)


if __name__ == "__main__":
    main()
