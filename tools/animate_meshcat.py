"""Watch a planned drawing in Meshcat: the six arms play the system plan, phase by phase.

A throwaway viewer, not part of the package.  From the repository root:
    .venv/bin/python tools/animate_meshcat.py [--case spiral] [--port 7010] [--speed 1] [--once]
Open the printed URL; the animation has a time slider (Animations panel) to scrub and replay.
One animation second is one rig second (times --speed).  The phase and the clock are printed
here, since Meshcat has no text in the scene.
"""
import argparse
import sys
import time
from pathlib import Path

import numpy as np
from pydrake.geometry import Box, Sphere, Meshcat, MeshcatParams, MeshcatVisualizer, \
    MeshcatVisualizerParams, Rgba
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import BodyIndex
from pydrake.systems.framework import DiagramBuilder

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tests"))
import system_cases as sc  # noqa: E402

from aris.kernel.retime import sample  # noqa: E402
from aris.rig import Rig  # noqa: E402
from aris.system import phase_named, plan_detailed  # noqa: E402

URDF = ROOT / "assets" / "system_model" / "installation.urdf"
HZ = 20.0            # samples per rig second
PAUSE = 1.0          # rig seconds of stillness between phases
INK_CHUNK = 20       # samples per ink segment object (one rig second)
URL = "http://frankastation.drl.csail.mit.edu:{port}"
# The URDF's links that the rig's own boxes and paper quad replace (the table stays).
URDF_REPLACED = ("paper", "frame_", "leg_", "runway_", "post", "gusset", "clamp", "plate",
                 "seam_bar")


# --------------------------------------------------------------------------- timeline


def timeline(rig, tagged):
    """-> [(phase name, t0, duration, {arm: [(start, Motion)]})] in plan order; within a phase
    every arm starts at the phase's start and plays its motions back to back."""
    order, per = [], {}
    for ph, a, m in tagged:
        if ph not in per:
            order.append(ph)
            per[ph] = {}
        per[ph].setdefault(a, []).append(m)
    out, t0 = [], 0.0
    for ph in order:
        runs, dur = {}, 0.0
        for a, ms in per[ph].items():
            s, run = t0, []
            for m in ms:
                run.append((s, m))
                s += m.traj.t[-1] - m.traj.t[0]
            runs[a] = run
            dur = max(dur, s - t0)
        out.append((ph, t0, dur, runs))
        t0 += dur + PAUSE
    return out


def samples(rig, phases, T):
    """On the grid T -> ({arm: (len(T), 7)} joint positions, {arm: (len(T), 3)} planned pen tip
    in the table frame while drawing, else nan).  Park until the first motion, each motion
    sampled at its own time, its end held until the next."""
    Q = {a: np.tile(rig.park_q(a), (len(T), 1)) for a in rig.arm_ids}
    tips = {a: np.full((len(T), 3), np.nan) for a in rig.arm_ids}
    for _, _, _, runs in phases:
        for a, run in runs.items():
            for s, m in run:
                d = m.traj.t[-1] - m.traj.t[0]
                Q[a][T >= s + d] = m.q_end
                w = (T >= s) & (T < s + d)
                tt = m.traj.t[0] + T[w] - s
                if w.any():
                    Q[a][w] = sample(m.traj, tt)[0]
                if m.kind == "draw" and w.any():
                    tips[a][w] = rig.to_table(a, np.column_stack(
                        [np.interp(tt, m.traj.t, m.tip_base[:, k]) for k in range(3)]))
    return Q, tips


def ink_chunks(tips):
    """-> [(arm, grid index at which it appears, (n, 3) points)]: each run of drawing cut into
    pieces of INK_CHUNK samples that share their end points."""
    out = []
    for a, P in tips.items():
        on = np.concatenate([[False], ~np.isnan(P[:, 0]), [False]]).astype(int)
        starts, ends = np.nonzero(np.diff(on) == 1)[0], np.nonzero(np.diff(on) == -1)[0]
        for i0, i1 in zip(starts, ends):
            for k in range(i0, i1 - 1, INK_CHUNK):
                j = min(k + INK_CHUNK, i1 - 1)
                out.append((a, j, P[k:j + 1]))
    return out


# --------------------------------------------------------------------------- scene


def arm_colours(rig):
    """The arm colours of the system figures."""
    return {a: Rgba(*(int(sc.ARM_COLOUR[i][k:k + 2], 16) / 255 for k in (1, 3, 5)), 1.0)
            for i, a in enumerate(rig.arm_ids)}


def build(meshcat, colours):
    """The URDF's arms and table; a ball in the arm's colour on each hand (the meshes keep
    their own materials)."""
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
    Parser(plant).AddModels(str(URDF))
    for a, c in colours.items():
        plant.RegisterVisualGeometry(plant.GetBodyByName(f"arm{a}_panda_hand"),
                                     RigidTransform([0, 0, 0.03]), Sphere(0.045), "tag",
                                     np.array([c.r(), c.g(), c.b(), 1.0]))
    plant.Finalize()
    vis = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat,
                                         MeshcatVisualizerParams(prefix="rig"))
    diagram = builder.Build()
    return diagram, plant, vis


def check_bases(rig, plant, ctx, T_world_table):
    """Largest disagreement between the URDF's arm bases and the rig's (m, rad)."""
    worst = (0.0, 0.0)
    for a in rig.arm_ids:
        X = plant.GetFrameByName(f"arm{a}_panda_link0").CalcPoseInWorld(ctx).GetAsMatrix4()
        R = T_world_table @ rig.T_table_base(a)
        dp = np.linalg.norm(X[:3, 3] - R[:3, 3])
        dr = np.arccos(np.clip((np.trace(X[:3, :3].T @ R[:3, :3]) - 1) / 2, -1, 1))
        worst = (max(worst[0], dp), max(worst[1], dr))
    return worst


def static_scene(meshcat, rig, steel=Rgba(0.45, 0.47, 0.5, 0.55)):
    """The paper and the rig's steel boxes, under /table (the table frame)."""
    meshcat.SetTransform("/table", RigidTransform([*(0.5 * rig.canvas_size), 0.0]))
    sx, sy = rig.canvas_size
    meshcat.SetObject("/table/paper", Box(sx, sy, 0.0005), Rgba(0.97, 0.96, 0.93, 1))
    meshcat.SetTransform("/table/paper", RigidTransform([0, 0, rig.paper_z - 0.00025]))
    for b in rig.steel:
        p = f"/table/steel/{b.name}"
        meshcat.SetObject(p, Box(*(b.hi_table - b.lo_table)), steel)
        meshcat.SetTransform(p, RigidTransform(0.5 * (b.lo_table + b.hi_table)))


def hide_urdf_duplicates(meshcat, plant):
    """The URDF's own cage and paper, which the rig's boxes and paper replace."""
    for i in range(plant.num_bodies()):
        name = plant.get_body(BodyIndex(i)).name()
        path = f"/drake/rig/aris_system_model/{name}"
        if name.startswith(URDF_REPLACED) and meshcat.HasPath(path):
            meshcat.SetProperty(path, "visible", False)


def walls(meshcat, rig, phases):
    """One translucent vertical quad per wall, per phase; returns {phase: [paths]}."""
    out = {}
    for ph, *_ in phases:
        out[ph] = []
        for w in phase_named(rig, ph).walls:
            p = f"/table/walls/{ph.replace(' ', '_')}/{w.name}"
            yaw = np.arctan2(w.normal_table[1], w.normal_table[0])
            meshcat.SetObject(p, Box(0.003, 3.0, 1.0), Rgba(0.85, 0.2, 0.2, 0.18))
            meshcat.SetTransform(p, RigidTransform(RotationMatrix.MakeZRotation(yaw),
                                                   w.point_table + [0, 0, 0.5]))
            out[ph].append(p)
    return out


# --------------------------------------------------------------------------- main


def record(diagram, plant, vis, rig, Q, T, speed, T_world_table):
    """Play the grid into a Meshcat recording.  -> {arm: (len(T), k, 3)} world points of the
    links and the pen tip (table frame), for the checks."""
    ctx = diagram.CreateDefaultContext()
    pctx = plant.GetMyMutableContextFromRoot(ctx)
    idx = {a: [plant.GetJointByName(f"arm{a}_panda_joint{j}").position_start()
               for j in range(1, 8)] for a in rig.arm_ids}
    bodies = {a: [plant.GetBodyByName(f"arm{a}_{n}") for n in
                  [f"panda_link{j}" for j in range(2, 8)] + ["panda_hand", "pen_tip"]]
              for a in rig.arm_ids}
    pts = {a: np.zeros((len(T), len(bodies[a]), 3)) for a in rig.arm_ids}
    off = T_world_table[:3, 3]
    vis.StartRecording(set_transforms_while_recording=False)
    q = np.zeros(plant.num_positions())
    for i, t in enumerate(T):
        for a in rig.arm_ids:
            q[idx[a]] = Q[a][i]
        plant.SetPositions(pctx, q)
        ctx.SetTime(t / speed)
        diagram.ForcedPublish(ctx)
        for a in rig.arm_ids:
            for k, b in enumerate(bodies[a]):
                pts[a][i, k] = plant.EvalBodyPoseInWorld(pctx, b).translation() - off
    vis.StopRecording()
    for a in rig.arm_ids:                         # the scene outside playback: the start
        q[idx[a]] = Q[a][0]
    plant.SetPositions(pctx, q)
    diagram.ForcedPublish(ctx)
    return pts


def looks(rig, phases, T, pts, tips):
    """Plain numeric looks at the scene: pen under the paper, a link origin inside steel, an
    active arm on the wrong side of a wall of its phase, the model's pen away from the plan's."""
    tip_z = min(p[:, -1, 2].min() for p in pts.values())
    dev = max(np.nanmax(np.linalg.norm(pts[a][:, -1] - tips[a], axis=1), initial=0.0)
              for a in rig.arm_ids)
    in_steel = {}
    for a, P in pts.items():
        X = P.reshape(-1, 3)
        for b in rig.steel:
            n = int(np.all((X > b.lo_table) & (X < b.hi_table), axis=1).sum())
            if n:
                in_steel[(a, b.name)] = n
    behind = {}
    for ph, t0, dur, runs in phases:
        w = (T >= t0) & (T <= t0 + dur)
        for wall in phase_named(rig, ph).walls:
            for a in wall.arms:
                side = np.sign(wall.normal_table[:2] @ (rig.T_table_base(a)[:2, 3]
                                                         - wall.point_table[:2]))
                d = side * ((pts[a][w] - wall.point_table) @ wall.normal_table)
                if d.min() < 0:
                    behind[(ph, a, wall.name)] = float(d.min())
    print(f"looked: lowest pen tip z {tip_z * 1e3:.1f} mm; model pen tip vs planned tip while "
          f"drawing, largest gap {dev * 1e3:.1f} mm")
    print("  link or tip origin inside steel: " + (", ".join(
        f"arm {a} in {n} ({c} samples)" for (a, n), c in in_steel.items()) or "none"))
    print("  link or tip origin behind a wall of its phase: " + (", ".join(
        f"{ph} arm {a} {n} by {-d * 1e3:.0f} mm" for (ph, a, n), d in behind.items()) or "none"))


def still(rig, port):
    """All six arms at park, every steel box opaque light grey, the paper; no walls."""
    meshcat = Meshcat(MeshcatParams(port=port))
    colours = arm_colours(rig)
    diagram, plant, _ = build(meshcat, colours)
    ctx = diagram.CreateDefaultContext()
    pctx = plant.GetMyMutableContextFromRoot(ctx)
    for a in rig.arm_ids:
        for j in range(7):
            plant.GetJointByName(f"arm{a}_panda_joint{j + 1}").set_angle(pctx, rig.park_q(a)[j])
    diagram.ForcedPublish(ctx)
    hide_urdf_duplicates(meshcat, plant)
    static_scene(meshcat, rig, Rgba(0.82, 0.82, 0.82, 1.0))
    c = np.array([*(0.5 * rig.canvas_size), 0.3])
    meshcat.SetCameraPose(c + [3.2, -2.6, 2.2], c)
    return meshcat


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--case", default="spiral", choices=sc.CASES)
    ap.add_argument("--workers", type=int, default=16)
    ap.add_argument("--port", type=int, default=7010)
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--once", action="store_true", help="exit after building")
    ap.add_argument("--still", action="store_true", help="no plan: arms parked, steel opaque")
    args = ap.parse_args()
    rig = Rig.load(ROOT / "config")
    if args.still:
        meshcat = still(rig, args.port)
        print(URL.format(port=args.port), flush=True)
        while not args.once:
            time.sleep(3600)
        return

    t = time.perf_counter()
    tagged, left, _ = plan_detailed(rig, sc.drawing(args.case), rig.rules(),
                                    cache_dir=str(ROOT / "out" / "cache"), workers=args.workers)
    print(f"plan: {args.case}, {len(tagged)} motions, {len(left)} left over, "
          f"{time.perf_counter() - t:.1f} s", flush=True)

    t = time.perf_counter()
    phases = timeline(rig, tagged)
    total = phases[-1][1] + phases[-1][2] if phases else 0.0
    T = np.arange(0.0, total + PAUSE, 1.0 / HZ)
    Q, tips = samples(rig, phases, T)
    meshcat = Meshcat(MeshcatParams(port=args.port))
    colours = arm_colours(rig)
    diagram, plant, vis = build(meshcat, colours)
    T_world_table = np.eye(4)
    T_world_table[:2, 3] = 0.5 * rig.canvas_size
    dp, dr = check_bases(rig, plant, plant.CreateDefaultContext(), T_world_table)
    print(f"URDF arm bases vs rig: {dp * 1e3:.3f} mm, {np.rad2deg(dr):.4f} deg")
    diagram.ForcedPublish(diagram.CreateDefaultContext())
    hide_urdf_duplicates(meshcat, plant)
    static_scene(meshcat, rig)
    wall_paths = walls(meshcat, rig, phases)
    pts = record(diagram, plant, vis, rig, Q, T, args.speed, T_world_table)
    anim = vis.get_mutable_recording()
    frame = lambda i: anim.frame(T[min(i, len(T) - 1)] / args.speed)  # noqa: E731
    for ph, t0, dur, _ in phases:                 # the walls show during their phase only
        i0 = int(np.searchsorted(T, t0))
        for p in (q for paths in wall_paths.values() for q in paths):
            anim.SetProperty(frame(i0), p, "visible", p in wall_paths[ph])
            if ph == phases[0][0]:
                meshcat.SetProperty(p, "visible", p in wall_paths[ph])
        print(f"  {ph:10s} animation {t0 / args.speed:7.1f} s to {(t0 + dur) / args.speed:7.1f} s")
    chunks = ink_chunks(tips)
    for k, (a, i, P) in enumerate(chunks):
        p = f"/table/ink/{a}/{k}"
        meshcat.SetLine(p, (P + [0, 0, 0.0005]).T, 3.0, colours[a])
        meshcat.SetProperty(p, "visible", False)
        anim.SetProperty(0, p, "visible", False)
        anim.SetProperty(frame(i), p, "visible", True)
    vis.PublishRecording()
    c = np.array([*(0.5 * rig.canvas_size), 0.3])
    meshcat.SetCameraPose(c + [3.2, -2.6, 2.2], c)
    print(f"build: {time.perf_counter() - t:.1f} s; animation {total / args.speed:.1f} s "
          f"({len(T)} frames, {len(chunks)} ink segments)")
    looks(rig, phases, T, pts, tips)
    print(URL.format(port=args.port), flush=True)
    if args.once:
        return
    while True:
        time.sleep(3600)


if __name__ == "__main__":
    main()
