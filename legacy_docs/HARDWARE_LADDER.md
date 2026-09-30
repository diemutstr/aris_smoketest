# The hardware ladder — bringing the planner to the real six-arm rig

Written 2026-09-09, before anything has been powered. **Nothing in this
document has been run on a robot.** Every number in it is either measured off
a file in this repository, measured off the control stacks already on this
machine, or explicitly marked as the thing a rung exists to measure.

---

## 0. What the control stack actually is — read this first

The planner ends at a file. `scripts/csail_schedule.py` conducts the fleet,
`scene_check` grades it, and the run writes
`out/<name>_schedule.npz` + `out/<name>_program.json`. The npz holds
`q_<arm>` as `(F, 7)` float32 on **one shared clock** at 24 fps, decimated by
`stride` from the conductor's own `dt = 1/(fps·substeps)` = 1/48 s.
`aris_sixarm` contains **no `rclpy`, no `lcm`, no `franky`, no socket**. It
cannot move an arm and it never could.

Four stacks on this machine can:

| | what it is | mode / rate | what it eats |
|---|---|---|---|
| **A. ROS 2 MoveIt** (`Aris_Kindt/my_ros2_ws/src/fr3_generic_drawing`, launched over SSH from `franka_control_gui.py` onto the operator box `192.168.50.2`) | **the one that has actually drawn on paper** | joint position via `joint_trajectory_controller`; MoveIt `computeCartesianPath` + TOTG | **Cartesian** pose waypoints |
| **B. Cartesian impedance** (`operator_impedance_helpers/impedance_pathway_exec.py`) | the stroke half of the same installation | equilibrium-pose streaming, **50 Hz**, `k_z` = 1500 N/m | a **Cartesian pathway CSV** |
| **C. Drake station** (`~/git/franka_manipulation_station`, py3.10 venv) | the MIT lab arms `128.30.16.50/.55` | LCM `PANDA_COMMAND`, 200 Hz publish | Drake `CompositeTrajectory` |
| **D. `fr3drivers`** (`~/git/fr3drivers`, `franka_driver_v5`) | **the only stack with a real safety architecture**; run on the lab arms, never on the installation | libfranka **1 kHz**; sender at 400–500 Hz; `position_velocity_accel` | a **piecewise-Bezier joint bundle** npz |

**The decision this ladder assumes, and it is a decision, not a fact.**
`aris_sixarm`'s whole product is a *redundancy resolution* — which of the FR3's
infinitely many arm configurations draws each stroke, chosen for margin and
manipulability. Stacks A and B take **Cartesian** input and re-solve that
themselves, which throws the answer away and replaces it with MoveIt's. Stack D
takes **joint** input and keeps it. So the target is **D**, and the adapter is a
file-format conversion rather than a controller: `fr3drivers/STATUS.md` already
lists the missing piece as a sender-side adapter, and `Fr3BundleBackend.export`
in this repo is now the other half of it.

What D's gate refuses, verbatim from its flags — these are the numbers rungs 2
and 3 are judged against:

```
--fr3_max_joint_velocity   = 2.62 2.62 2.62 2.62 5.26 4.18 5.26   (= frames.QD_MAX)
--fr3_max_joint_acceleration = 10.0        "the single most dangerous field"
--fr3_max_position_step    = 0.15 rad      handover guard
--fr3_command_timeout_sec  = 0.01
--fr3_tracking_fault_rad   = 0.020         latching watchdog, 3 ticks
collision profile "sensitive" = {20,20,20,20,10,10,10} Nm + 20 N
```

**The physical e-stop is the abort path.** The gate brakes at 2 rad/s² and the
watchdog latches, and `fr3drivers/STATUS.md` says outright that neither of those
is the abort. The GUI's `emergency_stop()` is a software pause plus a kill.

---

## 1. What is already true, and the three numbers that are not

`out/csail_schedule_h094_v18.npz` is the shipped programme: 100.0000 % of the
logo, 3 phases, 244.17 s, `scene_check` PASS at 80.59 mm inter-arm against an
80 mm gate. Loaded through the new adapter it reports:

```
$ ARIS_RIG=proposed ARIS_TOOL=lateral python3 -m aris_sixarm.execute \
      out/csail_schedule_h094_v18.npz out/csail_program_h094_v18.json --check
  6 arms, 3 phases, 4 barriers, 244.17 s at 24 fps (conductor dt 20.83 ms, stride 2)
  barrier 1 pen_swap  t 66.708 s  ACK REQUIRED  ... arms [31] must be REPOSITIONED (0.0320 rad)
  barrier 2 pen_swap  t 235.375 s ACK REQUIRED  ... arms  [2] must be REPOSITIONED (0.0329 rad)
```

Three things stand between that file and an arm, and **none of them is a
planning failure**:

1. **The tool has never been measured.** `PEN_LAT_HOLDER` / `PEN_EXT_HOLDER` =
   0.0860369 / 0.0460262 are USER-SPECIFIED, from a photograph and the sentence
   *"it only juts out 3–4 cm max"* → *"about 2 cm"*. The only measured tip in
   the project is the legacy inline pen's `PEN_EXT = 0.110` (arm 31,
   2026-07-12). **Rung 1.**
2. **The rig is not at the planned height.** `layout.LAYOUT_PROPOSED["h"]` =
   0.940; the hardware is **built at 0.850** with the option to trim the
   verticals. And `PARK_GRID_PROPOSED` was searched at 0.940: applied at 0.850
   it lands **−126.0 mm inside another arm's ink**. **Rung 0.**
3. **The timeline is not acceleration-feasible as written.** Measured on v18
   through the new adapter: peak joint speed is a comfortable **30 % of
   `QD_MAX`** on every arm, and peak joint acceleration is **33–37 rad/s²** —
   3.3–3.7× the `fr3drivers` gate's 10. `pacing.py` says why in its own words:
   *"TODO (v2): acceleration and torque limits… the v profile here is a
   ceiling, not a trajectory."* **Rung 2.**

---

## 2. The ladder

Each rung: **goal / runs / measures / passes when / unblocks / missing today.**

### Rung 0 — the as-built survey, ingested

**Goal.** Replace six assumed base poses with six measured ones, and re-plan at
the height the rig actually stands at.

**What runs.**
```
python3 scripts/asbuilt_layout.py --template > out/survey_YYYYMMDD.json   # fill in
python3 scripts/asbuilt_layout.py --survey out/survey_YYYYMMDD.json --check
python3 scripts/asbuilt_layout.py --survey out/survey_YYYYMMDD.json \
        --out out/asbuilt_YYYYMMDD.json [--calib 0.005]
# then, at the surveyed height:
ARIS_TOOL=lateral scripts/height_sweep.py sweep --h 0.850 \
        --out out/atlas_proposed_h0850_lat0860_gated63
ARIS_TOOL=lateral scripts/height_sweep.py park  --h 0.850 \
        --atlas out/atlas_proposed_h0850_lat0860_gated63 \
        --out out/park_search_h0850_lat0860.json
ARIS_RIG=proposed ARIS_TOOL=lateral scripts/replan_at_height.py --h 0.850 \
        --atlas ... --parks ... -- <draw.py args>
```

**What is measured.** Per arm, in the canvas datum of `docs/BUILD_SHEET.md` §0
(origin at the marked corner, z = 0 at the **paper surface**): the **joint-1
axis** x, y (not a plate edge), the **underside of the mounting plate** z, and
the base **yaw**. Plus the paper height at the four canvas corners.

**Passes when.** Every arm within ±10 mm of the build sheet in xy and z, the six
plates coplanar within 3 mm, every yaw within 1°, and — separately — a park
search at the surveyed height that CERTIFIES at the 80 mm gate.

**Unblocks.** Everything. It is also the rung with the largest single payoff
that is not safety: `mounts.MOUNTS.calib` = 0.03 m is carried as fat on every
neighbour's body column **because nobody has measured where the bases are**.
`feasible_workspace.rig(..., calib=…)` already takes the number; a survey is
what earns a smaller one, and 30 mm off five columns is canvas.

**What is missing in the repo today.**
- `layout.build_fleet` takes **one** `h` for all six arms and
  `layout.study_spec` sets **`yaw = 0.0` unconditionally** for every inverted
  arm. Six coplanar-to-3-mm plates are six heights, and a base bolted 2° out has
  no way to be said. `scripts/asbuilt_layout.py` (new, this commit) is the
  workaround: it builds the specs directly and does **not** edit `layout.py`.
- **There is no atlas and no park search at 0.850 for the FINAL tool.**
  `out/atlas_proposed_h0850_lat0860` exists; `park_search_h0850_lat0588.json`
  exists — at the *superseded* 0.0588 tool. The 0.850 park set that certifies
  (87.8 mm, `docs/DECISIONS.md` 2026-09-03) was searched at that older tool. It
  must be re-run before anything at 0.850 is believed.
- Nothing re-checks a *surveyed* fleet's park set against a *surveyed*
  neighbour's ink. `replan_at_height.py` handles the height; the per-arm yaw and
  z are new and go through `asbuilt_layout.load_asbuilt`.

### Rung 1 — touchdown calibration of the pen tip

**Goal.** Turn `PEN_LAT_HOLDER` / `PEN_EXT_HOLDER` from a reading of a
photograph into a measurement, on the holder that is actually mounted.

**What runs.** The touchdowns themselves use the existing operator tooling —
`operator_impedance_helpers/jog_descend.py` (compliant descent at 0.003 m/s,
*"the human eye is the contact sensor"*) or `probe_surface.sh` (5×5 grid,
plane fit) — and each contact is logged as **the joint vector at contact**, the
way the 2026-07-12 arm-31 touchdown was (it survives only as `Q_CONTACT` in
`tests/test_gates.py`, `MZ = 0.924` and `PEN_EXT = 0.110`; **no raw log of it
exists anywhere on disk**). Then:

```
python3 scripts/touchdown_calibrate.py --self-test          # prove the solver
ARIS_RIG=proposed ARIS_TOOL=lateral python3 scripts/touchdown_calibrate.py \
        out/touchdown_arm31.json --json out/touchdown_arm31_fit.json
```
with a log of `{"touchdowns": [{"arm": 31, "q": [7 floats], "paper_z": 0.0,
"label": "..."}, …]}`.

**What is measured.** The tip in the hand-TCP frame, all three components.

**Why several poses and not one.** The old derivation was a division —
`PEN_EXT = (paper_z − TCP_z) / cos(tilt)` — which works for a pen on the wrist
axis. This pen is **86 mm across the hand**, so a single touchdown cannot
separate lateral from axial: any pair that puts the tip on the paper *in that
orientation* fits. Contact is affine in the tip vector, so N touchdowns are N
linear equations in three unknowns and the fit is one `lstsq`. The design
matrix's rows are the paper normal *in the hand frame*, so touchdowns that share
a tool orientation share a row: **vary the tool yaw about the approach axis and
the lean off vertical**, at least 6–8 poses, spread across the arm's reach. The
script refuses a fit whose rows do not span (`cond(A) > 20`) rather than
reporting three digits of noise.

**Passes when.** `cond(A) ≤ 20`, residual rms under ~1 mm, `p_y` ≈ 0 (a `p_y`
of any size is the **holder clocked out of the jaw plane** — a build finding,
not a pen finding), and:

> **THE GATE TO WATCH IS THE PAPER, NOT THE NEIGHBOURS.** v18's chain
> clearance is **20.5 mm against a 20 mm gate — 0.5 mm**, the thinnest margin
> in the programme and thinner than the inter-arm one (arm 31 at t = 80.0 s).
> The final tool holds the wrist lower for the same tip than the 0.0588 one
> did; v15 and v17 had 30 mm there. **A measured tip that is deeper than
> 0.0460262 by more than 0.5 mm eats that gate**, and the whole programme has
> to be re-conducted before it is flown. `touchdown_calibrate.py` computes and
> prints exactly this and exits non-zero on it.

**Unblocks.** Rungs 2–5, and the atlas: `atlas.is_current` puts the pen inside
its model signature, so a moved tip stales every sweep in `out/`.

**What is missing.** Sensitivity, measured on synthetic data by the new test:
**1 mm of contact-detection error → ~1 mm of tip error**, not amplified — so
the *contact* method is the accuracy budget, and `jog_descend`'s "human eye" and
`impedance_pathway_exec`'s 3 mm lag threshold are both coarser than the 0.5 mm
that is at stake. **Nobody has specified how contact will be detected to better
than a millimetre.** That is the open question this rung has to answer first.

### Rung 2 — one arm, one stroke

**Goal.** Prove the executor: that a certified per-arm joint trajectory can be
handed to a driver, followed, and stopped.

**What runs.** `aris_sixarm.execute` against the dry-run backend first
(`python3 -m aris_sixarm.execute … --play --solo 31`), then
`Fr3BundleBackend.export` → `fr3drivers/tools/fr3_sender.py --dry-run` →
`tools/preflight.py`, and only then a powered attempt through
`launch_powered.sh`.

**What is measured.** The driver's own preflight verdicts (CHAIN / ROBOT FIT /
SCENE), then the tracking error against the 20 mrad latching watchdog, and the
drawn line against the commanded one.

**Passes when.** `preflight.py` returns three clean verdicts, and a powered run
completes with no reflex, no tracking fault, and no gate rejection.

**Unblocks.** Rungs 3–5.

**What is missing — this is the rung with real work in it.**
1. **Acceleration.** Measured above: v18 demands 33–37 rad/s² and the gate is
   10. `pacing.py` is velocity-only *by design and says so*. Making the timeline
   flyable is a **planning** change (a real TOPP pass, or a bounded-acceleration
   re-parameterisation) that must keep the path inside the certified tube — the
   `fr3drivers` bundle format makes the tube argument explicit: a time scaling
   *"leaves every control point untouched, so every collision certificate the
   planner earned survives"*, which is the same argument `execute.Governor`
   makes from the other side. Time scaling alone will not do it: it shrinks
   acceleration as `factor⁻²` but the peaks are at C0 corners.
2. **The decimation.** The npz is `stride`-decimated and `scene_check` graded
   the **full-rate** path. The fix is one flag — conduct with `--substeps 1`, or
   have `csail_schedule` write the un-strided `qtraj` beside the animation one.
   `from_schedule` warns on every load and `Fr3BundleBackend.preflight` refuses.
3. **`Fr3BundleBackend.export` produces a degree-1 bundle**, which is *exactly*
   the certified chords and is therefore honest — and has discontinuous velocity
   at every knot, so it is **not flyable in `position_velocity_accel` mode**.
   It is for `--dry-run` and inspection until (1) is done.
4. **A solo run is not certified.** `FleetProgram.solo()` returns the mover's
   track *and the poses the other five must be frozen at*, with
   `recheck_required = True`, because the conducted timeline was certified with
   all six **moving**. A frozen fleet is a different scene and wants
   `scripts/recheck_timeline.py` run against it.
5. **Contact.** Joint position control is stiff. A 1 mm height error becomes
   pen force, and the installation's own answer to this is stack B's Cartesian
   impedance (`k_z` = 1500 N/m) — which stack D does not have. Whether a
   position-controlled pen draws acceptably is unmeasured.

### Rung 3 — one arm, the full solo programme

**Goal.** Pen-up transits, the park/depot round trips, and a whole phase.

**What runs.** The same, with `--solo <arm>` over the full programme.

**What is measured.** Whether the C-space transits (`transit.py`'s RRT-Connect
paths, 189 of 1 142 crossings) survive contact with a real arm's dynamics, and
the makespan against the plan.

**Passes when.** A full solo phase runs to the end with no reflex and the
drawn ink is within `validate.TIP_TOL` (2 mm) of the plan.

**Unblocks.** Rung 4.

**What is missing.** The re-check of §2.4, per phase. And an idle policy that a
*real* arm holds: `idle.conduct` schedules park moves, and a real arm parked for
166 s under position control has thermal and gravity behaviour nothing has
modelled.

### Rung 4 — two arms, one phase barrier

**Goal.** The fleet clock, across processes.

**What runs.** Two arms of one phase, with the barrier logic in
`execute/runner.py` driving both.

**What is measured.** The realised inter-arm clearance against the certified
one; barrier synchronisation jitter.

**Passes when.** Both arms reach the barrier within tolerance, the barrier
verifies, and no pair gets closer than the certified minus the calibration
allowance.

**Unblocks.** Rung 5.

**What is missing — the biggest structural gap in the ladder.**
- **There is no cross-process fleet clock.** libfranka's loop owns its thread
  and its socket, so six arms are six processes (or six `fr3_sender`s), and
  `scene_check`'s certificate is only valid if they advance **together**.
  `execute.Governor` is the right *shape* — one scalar rate, applied to every
  arm, because re-timing one arm moves it against the others at instants nobody
  certified — but it currently lives in one process. Distributing it is real
  engineering and it is not started.
- **Reposition at the barrier is uncertified motion.** Measured on v18: arm 31
  steps **0.0320 rad** and arm 2 **0.0329 rad** between the pose held through a
  pen swap and the first pose of the next phase. In the animation that is one
  invisible frame; on a robot it is a step into a stiff controller, and
  `scene_check` graded the frames on either side of the gap, **not the move
  between them**. `Barrier.reposition()` now names them and the runner makes
  them an explicit supervised `goto` after the human has acknowledged. The
  right fix is upstream: have the conductor start each phase from the pose it
  ended the last one at.

### Rung 5 — six arms, v18

**Goal.** The piece.

**Passes when.** Rungs 0–4 pass and an independent `recheck_timeline.py` at
the as-built rig, as-built tool, re-conducted for acceleration, still says
PASS.

**What is missing.** Everything above, plus: **arm 71 has no IP** and has never
had one (`Aris_Kindt/aris_orchestrator/arm_registry.py` carries `0.0.0.0`;
`franka_control_gui.py`'s `LIVE_ARM_IDS` is `{13, 31, 17, 97, 2}` — five arms).
`Fr3BundleBackend.preflight` refuses a programme with an arm it has no IP for,
which is how that surfaces rather than being discovered at launch.

---

## 3. First day on hardware — the checklist

**Order matters: each step's output is the next step's input.**

**Before anything is powered**

1. **Survey the six bases.** `docs/BUILD_SHEET.md` §0 datum. Per arm: joint-1
   axis x and y (to the base bolt circle centre, *not* a plate edge), underside
   of the mounting plate z, base yaw. Also probe the paper height at the four
   canvas corners.
   → `scripts/asbuilt_layout.py --template > out/survey_<date>.json`, fill it
   in, then `--survey … --check`. **Report every deviation; do not re-centre
   the others to hide one.**
2. **Read back the gripper width** after each pen is clamped. The GUI commands
   **0.0432 m at 70 N** (`franka_control_gui.py::_PEN_GRASP_WIDTH`); libfranka
   only calls a grasp successful **above `width − epsilon_inner`**, so the
   number it reports is a **lower bound on the real jaw gap** — and that bound
   is what rules the holder build in or out (`docs/SYSTEM_MODEL.md` §7).
   Record it in the survey's `gripper` block.
3. **Record the collision profile** each arm is left in. The installation's
   operator *raises* the thresholds after a MoveIt launch so pen contact does
   not trip a reflex (40/40/36/36/32/28/24 Nm, 50/50/60/30/30/30 N);
   `fr3drivers`' default `sensitive` is **half** of that. Which one is in force
   is the difference between "the pen touched down" and "the arm crashed".

**Then, one arm only**

4. **Touchdowns.** 6–8 contacts on one arm, **at different tool yaws and
   leans**, spread across its reach. Log the **joint vector at contact** and the
   paper height there. Use `jog_descend.py` or `probe_surface.sh`.
   → `scripts/touchdown_calibrate.py out/touchdown_arm31.json`.
5. **Read the fit's verdict on the paper-chain gate** before doing anything
   else with it. If the tip is more than 0.5 mm deeper than 0.0460262, **v18 is
   stale** and must be re-conducted.

**The three numbers that must come back before any multi-arm test**

| # | number | who consumes it | why it blocks |
|---|---|---|---|
| **1** | **the as-built survey** — six (x, y, z, yaw), and the plate coplanarity | `scripts/asbuilt_layout.py` → `replan_at_height.py` | the park grid searched at 0.940 sits **−126 mm inside another arm's ink** at 0.850. Six arms parked where the others want to draw, and no ordering fixes it. |
| **2** | **the touchdown tip** — (lat, y, ext) with its residual | `scripts/touchdown_calibrate.py` → `frames.py` → the atlas, the park search, the programme | the shipped tip is a reading of a photograph, and v18's thinnest gate is **0.5 mm** of paper-chain clearance that a deeper tip eats first. |
| **3** | **the gripper width read-back** at the pen clamp | `docs/SYSTEM_MODEL.md` §7, the holder model | it is the only *measurement* of the tool assembly; the rest is CAD and a photograph. |

**And one rule for the day.** The physical e-stop is the abort. The software
gate brakes at 2 rad/s² and the watchdog latches at 20 mrad, and the driver's
own notes say in as many words that **neither of those is the abort path**.

---

## 4. What was added to the repo for this

| | |
|---|---|
| `aris_sixarm/execute/` | the execution-adapter **interface**. `trajectory.py` (`JointTrajectory`, `Governor` — one clock for the whole fleet, and why), `program.py` (`FleetProgram`, `PhaseTrack`, `Barrier`, `from_schedule`), `backends.py` (`Backend` protocol, `RecordingBackend`, `MeshcatDryRun`, `Fr3BundleBackend` **skeleton**), `runner.py` (`play`), `__main__.py` (CLI). |
| `scripts/asbuilt_layout.py` | survey → as-built layout JSON → fleet. Per-arm z and yaw, which `layout.py` cannot express. Report only. |
| `scripts/touchdown_calibrate.py` | touchdowns → tip transform, by least squares, with a conditioning refusal and the paper-chain consequence. Report only. |
| `tests/test_execute.py` | 24 tests, incl. v18 end to end and *"every robot-facing call raises"*. |
| `tests/test_hardware_prep.py` | 16 tests on synthetic surveys and synthetic touchdowns. |

**No robot-facing code was written.** `Fr3BundleBackend.connect`, `.read_state`,
`.goto`, `.start_stream`, `.send` and `.stop` all raise `NotImplementedError`,
and a test asserts each one does.
