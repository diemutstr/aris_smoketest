# PATHWAY CSV v2 — the planner's schedule, in the file the operator already reads

`aris_sixarm/export/pathway.py` implements `docs/ARIS2_CONTRACTS.md` §1 and §4.
It turns a **conducted fleet schedule** into one **pathway CSV per arm** — the
same file the deployed GUI produces and `rtff_pathway_exec.py` loads — plus the
seven optional joint columns and a manifest sidecar.

    .venv/bin/python -m aris_sixarm.export.pathway \
        --schedule out/csail_schedule_h094_v18.npz \
        --program  out/csail_program_h094_v18.json \
        --rig proposed --tool lateral --arms 31 71 --out out/pathways/

```
CSAIL logo, six arms, h = 0.940, v18 (final holder tool 0.0460262/0.0860369, ...)
  rig proposed, tool lateral (tip [0.0860369, 0.0, 0.0460262] in hand TCP), z-mode plane, intensity 1
  arm  31:  2633 rows ( 1800 draw +   833 travel,  12 strokes), draw   4.388 m, paper z +0.9400 m
          bbox x[-0.657,+0.604] y[-0.730,+0.607] z[-0.151,+0.941] m, r_xy<=0.764 |p|<=1.208 m, lean<=15.0 deg
          step<=0.0658 rad/row, biggest stroke boundary 4.698 rad (flown over 833 certified travel rows)
          stroke 5->6: 4.70 rad over 158 rows / 3.36 m of tip travel  [1.4 rad/m]
          stroke 2->3: 4.39 rad over 142 rows / 3.62 m of tip travel  [1.2 rad/m]
          stroke 10->11: 3.14 rad over 181 rows / 4.43 m of tip travel  [0.7 rad/m]
  arm  71:  3895 rows ( 2176 draw +  1694 travel,  22 strokes), draw   6.991 m, paper z +0.9400 m
          ...
  2 arm(s), 6528 rows, 11.379 m of ink -> out/pathways/
  ! stride = 2: the schedule npz is decimated ...
  ! this file crosses 1 ink change(s); the CSV format cannot express the pen-swap barrier ...
  ! no certified retract: ... so a lift_end ramp is solved for
```

`--rig`/`--tool` fall back to `ARIS_RIG`/`ARIS_TOOL`, then to what the programme
json says; the flags win. Other knobs: `--intensity` (constant tone, default
1.0), `--lift` (0.05 m, the generator's own), `--z-mode plane|fk`, `--name`,
`--h-inv` (only ever needed by a legacy rig, see §3),
`--transit-decimate-mm` (default 0 = off, see §2).

## 1. What it reads

| file | what comes out of it |
|---|---|
| `out/<name>_schedule.npz` | the conducted timeline: `q_<arm>` (F,7) on one shared clock, `seg_<arm>` (F,) — the per-frame segment index, `-1` when the pen is up — `fps`, `stride`, `pen_ext`, `phase`, `sheet` |
| `out/<name>_program.json` | labels only: `rig`, `h_inv`, phase names and inks, `draw_speed`, and the per-phase per-arm assignment lists that name each segment's **artwork stroke id** and kind |
| `aris_sixarm.fleet` / `layout` | `rig(name)` -> the `ArmSpec` registry -> `T_world_base` |
| `aris_sixarm.frames` | the FK, the tool offsets (`PEN_EXT`, `PEN_LAT_HOLDER`, `PEN_EXT_HOLDER`) |

Parsing is **not** re-implemented: `aris_sixarm.execute.program.from_schedule`
does it and returns the `FleetProgram`. The one array it does not carry is
`seg_<arm>`, which is read off the npz directly — and its alignment with the
`FleetProgram` tracks is **asserted frame by frame** (`_arm_frames`), because a
pen state that is one phase out of step with the joints is the one mistake this
file must not be able to make quietly.

**The pen state is recoverable, and it is `seg_<arm>`.** `seg >= 0` is exactly
`from_schedule`'s own drawing test, the index is global across phases
(`scripts/csail_schedule.py`'s npz writer adds the running count), and measured
on `csail_schedule_h094_v18` the tip of every `seg >= 0` frame is on the world
paper plane to **0.16 mm** while pen-up frames rise to 1.19 m. The programme
json's per-phase assignment lists line up with those indices in order, so each
CSV stroke also carries its artwork `stroke_id` and `kind` in the manifest.

## 2. What it writes

`<out>/<name>_arm<ID>.csv` and `<out>/<name>_arm<ID>.manifest.json`.

```
stroke_idx,wp_idx,kind,x_m,y_m,z_m,qx,..,intensity,q1,..,q7      (arm 31, real rows)
0, 0,travel,-0.068116,-0.554366,0.885349,..,1.0000, 0.110073,..   <- the planner's own descent
0,14,draw,  -0.068891,-0.556394,0.940000,..,1.0000,-0.242288,..   <- the paper plane, 0.940
0,15,draw,  -0.073886,-0.556631,0.940000,..,1.0000,-0.254941,..
...
1, 0,travel,-0.497042,-0.573385,0.939690,..,1.0000,-1.590402,..   <- the planner's own transit,
1, 1,travel,-0.503555,-0.566750,0.938072,..,1.0000,-1.586425,..      lifting off stroke 0
```

One row per schedule frame, in timeline order, from the first drawing frame to
the last: `draw` while the pen is on the paper (`seg >= 0`), `travel` while it
is up. **Every row carries q1..q7.** `stroke_idx` is the schedule's own per-arm
segment index — a transit row belongs to the stroke it is the approach to,
which is the role the generator's `lift_start` played — and `wp_idx` counts
inside that group.

### THE TRANSITS ARE IN THE FILE, AND THEY HAVE TO BE

The first version of this exporter emitted only the draw rows, bracketed by two
synthesized `lift_start` / `lift_end` rows at the raised point, and left the
travel between strokes to the executor. **That file is not executable.** The
planner *reconfigures the arm* inside its pen-up transits, and a synthesized
straight-line hover travel cannot fly that. Measured on arm 31 of
`csail_schedule_h094_v18` — the joint distance between the last drawing pose of
one stroke and the first of the next:

| boundary | jump | v1: rows between | v2: certified transit rows / tip travel |
|---|---:|---:|---:|
| 2 -> 3 | **4.39 rad** (j3) | 2 synthesized lifts | 142 rows / 3.62 m |
| 5 -> 6 | **4.70 rad** (j5) | 2 synthesized lifts | 158 rows / 3.36 m |
| 10 -> 11 | **3.14 rad** (j3) | 2 synthesized lifts | 181 rows / 4.43 m |
| 8 -> 9 | 2.27 rad | 2 synthesized lifts | 44 rows / 0.63 m |

and on arm 71 the worst is 5.27 rad (stroke 12 -> 13, now 96 rows / 1.18 m).
In the Drake SIL the v1 file drove the arm into its joint limits during the
travel to stroke 4 and it never recovered — touch 0.29, 211 mm rms in-plane
error. With the stage-1 nullspace reference the joint target stepped 4.5 rad in
one waypoint.

The executor walks consecutive non-draw rows as an ordinary Cartesian path
(`spd = self.a.draw_speed if is_draw else self.a.travel_speed`), so the
certified transit is exactly the right thing to hand it. After the change the
largest joint step between **any** two consecutive rows is **0.0658 rad** on
both arms — the conductor's own per-frame bound (`writing.MAX_DQ_FRAME` = 0.04
per sub-step, `stride` = 2) — against 4.70 rad before.

Row counts, before -> after: arm 31 **1824 -> 2633** (1800 draw + 833 travel),
arm 71 **2220 -> 3895** (2176 draw + 1694 travel + 25 solved).

Two things keep that from being bigger than it needs to be:

- **Barrier holds collapse.** Consecutive frames with identical `q` become one
  row: the pen-swap pause is 1177 identical frames on arm 31 and 1777 on arm 71,
  and one row says it exactly. A *drawing* frame is never dropped.
- **`--transit-decimate-mm`** thins pen-up rows to a tip spacing, and only
  across frames whose joints also barely moved. It is **off by default**,
  deliberately: the motion this file exists to carry is a wrist flip with a
  near-stationary tip, which has no tip spacing to be thinned by.

### The ends, and the only thing still synthesized

The file starts at the last part of the planner's own descent onto stroke 1 and
ends with its own retract off the last stroke — walking out from the first and
last drawing frame until the tip is `--lift` clear of the paper, up to a 3 s
window. Nothing is synthesized where that works (arm 31: 14 rows of approach,
12 of retract, all certified).

Where it does not work, a ramp is solved for. Arm 71 finishes its last stroke
and then *holds the pen down* for 216 frames, so there is no retract to take:
25 rows at 2 mm spacing, each from `ik.solve_cc` seeded on the previous — the
same IK branch, not a new configuration — climbing 50 mm along the pen axis.
A ramp and not one row because one row was one 5 cm jump, which measured
0.13 rad in a single step, four times the conductor's own bound. The ramp is
refused outright (and the manifest says so) if the solver gives up, if a step
exceeds 0.04 rad, or if the climb drifts more than 0.35 rad from where it
started — at some certified drawing poses, holding q7 fixed while the tip
climbs costs 1.8 rad of joint 1 against joint 3 for 6 cm, and that is a
reconfiguration, not a retract.

### The rest of the row

- **Frame.** The arm's `fr3_link0`. The schedule is in the planner's canvas
  frame, but the row poses come straight from `frames.fk(q)`, which *is* the
  base frame, so no transform is applied to any row. `T_world_base` enters only
  as the manifest record and as `paper_z_base_m`.
- **Pose.** The EE frame the robot is configured with (`setEE` = NOMINAL pen
  tip, contract §4): position `frames.tip_pos(q)`, orientation the hand-TCP
  rotation of `frames.fk(q)`.
- **`z_m` on draw rows.** `--z-mode plane` (default) writes the paper plane
  expressed in the base frame and **nothing else** — no press; the executor
  adds that. `--z-mode fk` writes the certified pose's own z instead; the two
  differ by at most 0.16 mm on the shipped programme
  (`z.max_fk_deviation_from_plane_m`). Travel rows always carry their own z.
- **`intensity`.** A constant on draw rows, `--intensity`, default 1.0; 1.0 on
  pen-up rows. **Tone mapping is future work** — see §5.

The manifest carries contract §1's set — `arm_id`, `rig`, `tool` (name + tip
offset in the hand-TCP frame), `T_world_base` (4x4 row-major), `paper_z_base_m`,
`joint_columns`, `generator` (module + git sha + branch + dirty),
`source` (schedule, programme, planner job, fps/stride), `speeds`, `created` —
plus `format`, `quaternion`, `z`, `intensity`, `transits` (`in_file`, row count,
solved end rows), **`boundaries`** (one entry per stroke boundary: the joint
jump, how many transit rows carry it, their tip length, their worst row-to-row
step, and `reconfig_rad_per_m`), `stats` (row counts, draw length, base-frame
bbox, max radius, max lean, **`max_row_dq_rad`**, `max_boundary_dq_rad`, max
orientation step) and a `strokes` table (segment index, artwork `stroke_id`,
kind, phase, ink, rows, length, lean, time window). `warnings` carries the
`FleetProgram`'s own, decimation included.

## 3. Conventions, decided

**Base transform.** `ArmSpec.T_world_base()` — the same 4x4
`planner.build_lattice` (`_lattice_setup`), `scene_check._chain`, `atlas` and
`execute` use. `base_transform()` does not *assume* it is right to call it with
no argument: it probes the transform at three `h_inv` values and accepts the
bare call only if the pose does not move. Every current rig carries `R` and `z`
explicitly (`layout.study_spec`, `rig_final`) and so does not move — the trap
`docs/DECISIONS.md` records is **already closed upstream**, and this is the
check that keeps it closed. A legacy hanging spec that *does* move is refused
unless `--h-inv` is given (the programme json's own `h_inv`, when present, is
used automatically and cross-checked).

**Quaternion.** `(x, y, z, w)`, and the deployed meaning: `(1,0,0,0)` is pen
straight down on a floor arm, the pen axis is EE `+Z`.
**No conversion is applied, and none is needed.** The repo's hand-TCP frame
carries the flange's `Rz(-pi/4)` twist (`frames.fk`) — the Franka Hand's own
nominal-end-effector frame, libfranka's `F_T_NE`, which `setEE` (`NE_T_EE`)
extends to the pen tip without turning it — so the repo's tool frame and the
deployed EE frame are the same frame. That is the explanation; the evidence is
the measurement. `tests/test_export_pathway.py` pins
both halves of it: a floor arm with base yaw 0 (legacy `sixarm` arm 13,
`T_world_base` rotation = I) exports **exactly** `(1,0,0,0)` (residual 2e-8, the
analytic IK's own), and a yawed floor arm (final rig arm 13) exports
`Rx(pi) @ Rz(psi)` — `(1,0,0,0)` up to the free rotation about the pen axis.
That yaw is the planner's, from its own IK, and converting it away would break
the FK gate, which is the entire value of the joint columns.

The sign of the quaternion is canonicalised on its **largest** component, not on
`w`: both deployed constants have `w = 0`, where a `w >= 0` rule decides on
rounding noise and emits `(-1,0,0,0)` about half the time. The rest of the file
then chains its sign by dot product, so the stream is continuous throughout.

**Inverted arms.** Confirmed on the shipped programme: every exported DRAWING
row of the inverted arm 31 has pen axis `. +Z > 0` in `fr3_link0`, which is what
`svg_to_pathway_csv --inverted` requires, and pen-up rows sit *below* the draw
rows in base z (the file's bbox runs z -0.151 .. +0.941 m, the paper being
+0.940) — lifting away from an overhead table is base -z, the generator's own
inverted branch.

**Unlike a v1 file the quaternion is not constant, and that has a consequence.**
It carries the planner's per-frame yaw and, on strokes the planner rescued with
a tilt cone (`aris_sixarm.tilt`), up to 15 deg of pen lean on
`csail_schedule_h094_v18`. Along a stroke it barely moves (<= 0.02 deg per row
on the shipped programme — the planner holds one tool yaw per stroke), but
**between** strokes it turns a long way: from the last drawing pose of one
stroke to the first of the next, 15.9 / 0.0 / 117.7 / 154.4 / 174.5 / 40.1 deg
for arms 13 / 17 / 2 / 31 / 71 / 97. In a v1 file that whole turn was one step.

The deployed executor's streaming loop publishes the waypoint's quaternion
as-is — `q = Q[j]`, no slerp — so a hop with a big yaw change is a **step** into
a stiff Cartesian impedance controller. `_ramp`'s own docstring names that
failure ("a 180-deg family change (EE flip) slams the wrist into a reflex") and
it has a slow-slerp remedy, but `_ramp` runs only on the approach and on
recovery, **not on the inter-stroke hop in the stream**. A v1 file could never
hit this: one constant quaternion for the whole run.

**The transit rows are what resolves it.** With the certified pen-up frames in
the file the turn is spread over 20-240 rows instead of happening in one hop,
and the executor turns from row to row as it would along any path. Measured on
the re-exported arm 31: the largest orientation step between **any** two
consecutive rows is **6.08 deg** (0.018 deg inside a stroke), against 154 deg at
a v1 hop — comfortably under the executor's own 20 deg reorient threshold, so
the slow-slerp path never has to fire.
`stats.max_quat_step_within_stroke_deg` / `..._between_strokes_deg` record it.

**Tool.** `--tool` selects the tip model, but the exporter **refuses** to write
a file whose axial tip depth disagrees with the schedule's own `pen_ext` (a
`lateral` plan exported as `inline` would move every row 64 mm along the pen
axis). Same for `--rig` against the programme's.

## 4. What it asserts before it will write

1. **Contract §1's FK gate.** For every row, FK(q1..q7) through the same EE
   frame must reproduce `(x,y,z)` to **< 0.5 mm** and the quaternion to
   **< 0.5 deg**, or nothing is written. The check runs on the **formatted**
   rows — re-parsed exactly as the executor's `_load` does — so what is
   verified is the text, rounding included.
2. Phase-to-frame alignment: each phase's `FleetProgram` track must equal
   `q_<arm>[i0:i0+n]`, or the segment array being read is not this arm's.
3. `T_world_base` h_inv-invariance (§3), and a refusal for an ambiguous spec.
4. The base z axis must be parallel to the world's, so that the world paper
   plane *has* a single `z_m` in the base frame — floor and inverted mounts
   only; a `wall` mount (J1 horizontal) is refused with that reason.
5. `--tool` / `--rig` against what the run was planned with.
6. A solved end ramp must step under 0.04 rad per row and stay within 0.35 rad
   of the drawing pose it came off, or it is not written at all.

`tests/test_export_pathway.py` (18 tests, ~0.9 s) covers the round trip through
a verbatim copy of the executor's `_load`, both quaternion conventions, the FK
gate firing on a corrupted row, the manifest field set, the solved end ramp,
and — on both arm 31 and arm 71 of the shipped programme — that **no row has
empty joint cells**, that **no row-to-row joint step exceeds the conductor's own
`2 * writing.MAX_DQ_FRAME`**, and that **every stroke boundary worth more than
0.5 rad is carried by transit rows** rather than jumped. The fixtures are built
from `planner.build_lattice` and `ik.solve_cc`, so a synthetic schedule has the
same shape a conducted one does.

## 5. Known limits

- **The executor paces by Cartesian arc length, and a reconfiguration has none.**
  The transit rows are in the file, but the streaming loop advances by
  `s += travel_speed * dt` along the *tip* path and takes the waypoint's
  quaternion and joints as they come (`q = Q[j]`, no slerp). Where the planner
  reconfigures with little tip travel, several rows are consumed per tick and
  the joint reference still moves fast: the manifest's
  `boundaries[].reconfig_rad_per_m` is that ratio, and it reaches **8.1 rad/m**
  on arm 71 (stroke 13 -> 14: 2.26 rad over 0.28 m) and 5.9 on arm 31. Nothing
  the exporter can do about it — the fix is executor-side pacing that also
  respects joint distance — but it is the number to watch in the SIL, and the
  file now at least contains the path the arm has to follow.
- **The pen swap is not in the format.** A file spanning more than one ink
  crosses a barrier where a human changes the pen, and the CSV has no way to
  say "stop here": the executor will fly straight through it. The exporter
  warns, and the manifest's `strokes[].ink` says where the change is. Splitting
  the export per phase would fix it and is not done.
- **No tone.** The schedule has no per-waypoint tone channel, so `intensity` is
  a constant. The deployed generator derives it from the source image and the
  traced line weight (`svg_to_pathway_csv --source-image / --intensity-driver`);
  reproducing that on the planner side is future work, and it is the only
  pressure channel there is (contract §1).
- **The rows are the decimated schedule.** The npz is written at `stride`
  (2 on every shipped programme) and `scene_check` graded the full-rate path;
  the CSV inherits that, and the `FleetProgram`'s own warning is copied into the
  manifest. The fix is upstream (`csail_schedule --substeps 1`).
- **Only floor and inverted mounts.** A wall mount has no constant paper `z_m`
  in its base frame (assert 4 above).
- **The file starts near the paper, not at the park pose.** The lead-in is the
  last 5 cm of the planner's descent; getting the arm from wherever it is to
  row 0 is the supervised `goto` of `execute.program.Barrier`, outside this
  file. Row 0's own q is what to drive to.
- **The precision floor is the npz's `float32` q.** The conductor stores joints
  as `float32` (~1e-7 rad), which is ~0.1 mm at the tip — and that, not planner
  tip error, is most of the 0.06..0.16 mm by which the shipped programme's FK z
  misses the nominal paper plane. It is well inside the contract's 0.5 mm
  budget, but `--z-mode plane` spends some of that budget: if a future plan's
  own tip error grows (`validate.TIP_TOL` allows 2 mm), the gate will refuse the
  snap and name `--z-mode fk` as the remedy.
- `speeds.travel_m_s` is `null` on purpose: the plan paces transits in joint
  space (`aris_sixarm.pacing`, `transit`), not at a Cartesian travel speed.
  `speeds.draw_m_s` is the programme's nominal 0.12 m/s;
  `speeds.draw_m_s_measured` is the arm's own tip length over its drawing frames
  on the fleet clock, which on `h094_v18` is 0.059 m/s for arm 31 — the
  conductor's coordination holds, not a second opinion about the speed.
- The exporter writes one file per arm and nothing about the **fleet clock**:
  the CSVs share a timeline (the manifest's `strokes[].t_start_s`) but the
  format has no time column, so nothing downstream can currently re-synchronise
  six arms from these files alone.
