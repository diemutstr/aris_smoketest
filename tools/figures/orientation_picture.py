import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, FancyArrowPatch

c = json.load(open("config/rig.json"))
h = c["hanger"]
tx, ty = c["table"]["size_x_m"], c["table"]["size_y_m"]
cx, cy = c["canvas"]["size_x_m"], c["canvas"]["size_y_m"]
site = json.load(open("site/aris_2026-10.json"))["slots"]
old = {k: v["robot"].replace("fr3-", "") for k, v in site.items()}

# Page is rotated 180 degrees from the drawing frame so that the bases' FORWARD direction
# (the side the J1 axis is offset toward: -x in the frame) points to the RIGHT of the page.
# Pete: forward to the right  <=>  the desk at the bottom.
def page(x, y):
    return -x, -y

fig, ax = plt.subplots(figsize=(7, 11))
ax.add_patch(Rectangle(page(tx/2, ty/2), tx, ty, fill=False, lw=2, color="0.3"))
ax.add_patch(Rectangle(page(cx/2, cy/2), cx, cy, fill=False, lw=1, ls="--", color="0.5"))
for s in c["slots"]["list"]:
    x, y = s["axis_xy_m"]; name = s["slot"]
    pc = x + h["plate_centre_offset_x_m"]
    px, py = page(pc, y)
    ax.add_patch(Rectangle((px - h["plate_size_x_m"]/2, py - h["plate_size_y_m"]/2), h["plate_size_x_m"], h["plate_size_y_m"], fc="#cfd8dc", ec="0.3"))
    for sx in (x - h["axis_to_minus_x_outer_face_m"] + h["strut_size_x_m"]/2, x + h["axis_to_plus_x_outer_face_m"] - h["strut_size_x_m"]/2):
        qx, qy = page(sx, y)
        ax.add_patch(Rectangle((qx - h["strut_size_x_m"]/2, qy - h["strut_size_y_m"]/2), h["strut_size_x_m"], h["strut_size_y_m"], fc="0.55", ec="0.3"))
    ax0, ay0 = page(x, y)
    live = True
    ax.add_patch(Circle((ax0, ay0), 0.045, color="tab:blue" if live else "0.6", zorder=5))
    # forward arrow: the axis is offset toward -x in the frame -> +x on this page
    ax.add_patch(FancyArrowPatch((ax0, ay0), (ax0 + 0.22, ay0), arrowstyle="-|>", mutation_scale=18, lw=2, color="tab:red", zorder=6))
    ax.text(ax0, ay0 + 0.13, f"{name}  (old {old[name]})", ha="center", fontsize=11, weight="bold", color="tab:blue" if live else "0.4", zorder=6)
ax.text(0.9, -1.0, "FORWARD", color="tab:red", fontsize=11, weight="bold", rotation=0)
# the desk at the bottom
ax.add_patch(Rectangle((-0.6, -ty/2 - 0.55), 1.2, 0.35, fc="#ffe0b2", ec="0.3"))
ax.text(0, -ty/2 - 0.375, "Diemut's desk", ha="center", va="center", fontsize=12, weight="bold")
ax.annotate("", xy=(0, -ty/2 + 0.1), xytext=(0, -ty/2 - 0.2), arrowprops=dict(arrowstyle="-|>", lw=1.5, color="0.3"))
ax.text(0.08, -ty/2 - 0.08, "she looks along the table", fontsize=9, color="0.3")
ax.text(-tx/2 - 0.08, 0, "her LEFT (L)", rotation=90, va="center", ha="right", fontsize=11, color="0.3")
ax.text(tx/2 + 0.08, 0, "her RIGHT (R)", rotation=270, va="center", ha="left", fontsize=11, color="0.3")
# frame axes of the drawing (rotated on this page)
ox, oy = page(0, 0)
ax.add_patch(FancyArrowPatch((ox, oy), (ox - 0.35, oy), arrowstyle="-|>", mutation_scale=14, lw=1.2, color="tab:green"))
ax.text(ox - 0.4, oy + 0.05, "+x (frame)", color="tab:green", fontsize=9, ha="right")
ax.add_patch(FancyArrowPatch((ox, oy), (ox, oy - 0.35), arrowstyle="-|>", mutation_scale=14, lw=1.2, color="tab:green"))
ax.text(ox + 0.05, oy - 0.42, "+y (frame)", color="tab:green", fontsize=9)
ax.set_aspect("equal"); ax.set_xlim(-1.45, 1.45); ax.set_ylim(-ty/2 - 0.7, ty/2 + 0.3); ax.axis("off")
ax.set_title("Seen from above, turned so the bases' forward side points RIGHT\n(Pete's rule: forward right = desk at the bottom)\nRow 3 is at her desk; L = her left, R = her right (since 2026-10-08)", fontsize=11)
fig.tight_layout(); fig.savefig("docs/figures/table_orientation.png", dpi=130); print("saved")
