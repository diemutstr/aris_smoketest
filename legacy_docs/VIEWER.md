# THE BROWSER GUI: RUNNING THE PLANNER AND WATCHING IT THINK

    .venv/bin/pip install -e '.[dev,gui]'
    .venv/bin/python -m aris_sixarm.gui
    #   Aris stroke planner GUI:  http://127.0.0.1:8765/

Open that URL.  The left column is the parameter form and the job list, the
middle is the 3D installation, the strip under it is the stage timeline, and
the bottom is four panels and the planner's own log.  Press **Plan**.

`--host 0.0.0.0` exposes it to the network (the default is this machine only),
`--port` moves it, `--jobs-dir` moves where jobs are kept.

## What you see while a job runs

The GUI is the same front door as the terminal — it runs `scripts/draw.py` in a
subprocess with the parameters you set — so the difference between a run
started here and one started from a shell is that somebody is listening.

1. **trace** finishes in under a second and the counter says how many strokes
   came out of the picture and how many inks.
2. **placement** puts the drawing on the paper; the strokes appear on the
   canvas in the 3D view, faint, the moment they exist in sheet coordinates.
3. **allocation** is the long one.  Its bar fills by strokes probed, then by
   strokes re-planned, and the coloured bands INSIDE the bar are the
   substages — prefilter, probe, repair, replan, flycheck, balance, merge,
   guarantee, sequence — as shares of the wall clock, with whatever no
   substage accounts for drawn explicitly in grey.  As each span is certified
   it is drawn on the canvas in the colour of the arm that will draw it, so
   the picture fills in by arm and by the time the stage ends you are looking
   at the allocation rather than reading about it.  The
   load bars under the counters are per-arm ink, replaced by per-arm seconds
   once the balancer has spoken.
4. **conduction** conducts each phase; one row appears in the Programme panel
   per phase, with its makespan and its verdict.
5. **scene_check** is bracketed separately so its cost is visible; its verdict
   and the fifteen inter-arm minimum distances land in the log and the panel.

When the job ends the worker exports a **viewer bundle** and the Programme
panel turns into a scrubber: play, scrub, per-arm lanes showing drawing vs
pen-up, and the six arms in the 3D view moving through the conducted timeline
with the ink appearing as it is laid.

Cancelling kills the whole process group, which matters because the allocator
forks pools.

## Where the time goes

This is the point of the thing.  The **Where the time goes** panel and the
bands inside the stage bars are one measurement: `allocate.allocate`'s own
`timing` dict, the same numbers `allocate.report` prints as its last line, and
the same nesting `docs/FAST_PLANNING.md` used to find that 93 % of an
allocation was inside `balance`.  A stage that contains another (`scene_check`
runs inside `conduction`) is reported EXCLUSIVE of it, so the shares add up to
the wall clock.

### The baseline

The smallest end-to-end run this repository can do — the CSAIL mark at 0.300 m
wide, arms 31 and 71, fixed placement, `--max-probes 1 --no-rrt --no-balance
--no-split --no-verify --skip-unconductable`, rig `proposed`, tool `lateral`,
`--image-jobs 3`.  It draws 1.863 m of 2.865 m (65.03 % coverage, 15 certified
segments), conducts to a 55.771 s makespan, and passes `scene_check` at
87.3 mm against an 80 mm margin.  Read straight off its own event stream
(`out/gui_jobs/20260902-142111-2e41/events.jsonl`):

| stage | seconds | share |
|---|---:|---:|
| trace | 0.2 | 0.1 % |
| placement (fixed, no search) | 0.0 | — |
| **allocation** | **310.0** | **90.3 %** |
| &nbsp;&nbsp;— **replan** | **174.1** | **50.7 %** |
| &nbsp;&nbsp;— **flycheck** | **62.2** | **18.1 %** |
| &nbsp;&nbsp;— **merge** | **56.1** | **16.4 %** |
| &nbsp;&nbsp;— probe | 8.8 | 2.6 % |
| &nbsp;&nbsp;— sequence | 4.0 | 1.2 % |
| &nbsp;&nbsp;— guarantee | 3.8 | 1.1 % |
| &nbsp;&nbsp;— repair | 0.8 | 0.2 % |
| &nbsp;&nbsp;— prefilter | 0.0 | — |
| conduction (freeze + conduct + coordination) | 15.8 | 4.6 % |
| scene_check | 14.9 | 4.3 % |
| bundle export | 1.5 | 0.4 % |
| **wall clock** | **343.3** | |

Three things this says that no log said before.

1. **With the balancer off, `replan` is half the run.**  That is the clean
   re-plan of every chosen span (`replan_segment`) plus `fly_shrink` giving
   back the span ends an arm cannot fly to, at one probe per (stroke, arm).
2. **`flycheck` is 18 %.**  `prune_unflyable` prices a whole bag's home legs
   on the transit router, once per arm per ban round, and the ban loop runs
   up to four rounds.  Nobody had measured it; it was inside `timing["balance"]`
   with two other things.
3. **`merge` is 16 %.**  `merge_remainders` re-plans a segment to absorb the
   ink lying against it — on this run it bought 15 mm of ink for 56 seconds.

For contrast, the shipped `_h094_v14` programme (six arms, the full logo, the
balancer ON) reads `prefilter 0.1 · probe 69.8 · repair 0.0 · replan 548.2 ·
balance 929.7 · park 0.0 · sequence 5.7` of a 1600.4 s phase-1 allocation, and
1824.1 s of its 4170.9 s total was the C-space pen-up planner.  The GUI shows
that split as it happens instead of at the end of a log.

## Running a job the way the shipped programme was run

The `_h094_v14` programme (the 100 % run) was
`scripts/csail_schedule.py --tag _h094_v14` under `ARIS_RIG=proposed
ARIS_TOOL=lateral`, with `--two-pass --residual-passes 6 --tilt-max-deg 15
--depot-hover-selective --freeze-refused-phase --skip-unconductable
--image-jobs 6`, a pre-computed placement and the gated atlas.  Every one of
those is a field in the form; `--tag` becomes the **output name**, and the
placement JSON goes in the placement field.  That run took 69 minutes.

## How it is built

    aris_sixarm/progress.py        the hook the pipeline calls; a no-op with no sink
    aris_sixarm/gui/server.py      FastAPI: job API, event websocket, static viewer
    aris_sixarm/gui/jobs.py        job directories, subprocesses, the event file
    aris_sixarm/gui/worker.py      one job, one process, one JSONL
    aris_sixarm/program_schema.py  the typed programme record + the exporters
    scripts/export_viewer_bundle.py   the same exporters, from a terminal
    web/viewer/                    plain ES modules; three.js vendored, no npm

A job is a **subprocess**, not a thread, for three independent reasons: the
planner is CPU-bound python and a thread would hold the GIL exactly while the
interesting stage ran; a cancel has to be unconditional and there is no
cooperative check inside a Held-Karp DP; and the rig and the tool are chosen by
`ARIS_RIG` / `ARIS_TOOL` at `import aris_sixarm` time, so two jobs at different
rigs cannot share an interpreter.  **This is the only place in the project that
sets those variables.**  Library code and tests still pass the rig and the tool
explicitly.

The event bus is an **append-only JSONL** in the job directory, tailed by byte
offset.  That is why replay is free: opening a finished job re-runs its whole
recorded stream through the same reducer the live view uses, so what you see is
what a watcher saw.  It also means `tail -f out/gui_jobs/<id>/events.jsonl` works
from a terminal.

## The event schema

Every event is one JSON object on one line:

```json
{"seq": 412, "t": 88.31, "wall": 1788370444.9,
 "kind": "substage_end", "stage": "allocation",
 "payload": {"sub": "probe", "elapsed_s": 20.5, "ok": true}}
```

`seq` is a gapless per-process counter, `t` is seconds since the sink was
attached, `wall` is `time.time()` for correlating with logs.  `stage` is one of
`trace`, `placement`, `allocation`, `conduction`, `scene_check`, `export`, or
`""` for job-level events.  `kind` is one of:

| kind | payload |
|---|---|
| `job_start` | `params`, `argv`, `rig`, `tool`, `pid`, `job_dir` |
| `job_end` | `ok`, `error`, `bundle_error`, `elapsed_s`, `files` |
| `stage_start` | stage-specific, e.g. `{source, inks, work_px}` for trace |
| `stage_end` | `elapsed_s`, `ok`, plus whatever the stage learned |
| `substage_start` | `sub`, plus context |
| `substage_end` | `sub`, `elapsed_s`, `ok`, plus results |
| `progress` | `done`, `total`, `label` — a loop tick |
| `item` | one unit of work; `what` says which (below) |
| `metric` | `name`, `value`, `unit` |
| `log` | `level`, `msg` — one line of the planner's stdout |
| `artifact` | `name`, `path` — a file appeared on disk |

The `item` payloads, by `what`:

| what | stage | payload |
|---|---|---|
| `sheet_strokes` | placement | `sheet`, `info` (the placement), `strokes` (id, colour, kind, length, decimated points) — sent ONCE; everything later refers to these by `(stroke, s0, s1)` |
| `probe` | allocation | `stroke`, `color`, `length_m`, `spans` `[[arm, s0, s1], …]` |
| `placed` | allocation | `stroke`, `arm`, `s_range`, `length_m`, `direction`, `min_sigma`, `lean_deg` |
| `loads` | allocation | `loads_s` per arm, `max_before_s`, `max_after_s` |
| `sequenced` | allocation | `arm`, `method`, `n`, `transit_s`, `baseline_transit_s`, `n_reversed` |
| `allocated` | allocation | `timing` (the whole substage dict), `n_segments`, `drawn_m`, `dropped_m`, `arm_metres`, `arm_segments` |
| `phase` | conduction | `index`, `name`, `ink`, `duration_s`, `split_kept`, `scene_check_ok`, `min_clearance_m`, `arm_metres`, `arm_draw_s`, `arm_transit_s` |

`stage_end` for `scene_check` carries the whole verdict: `verdict_ok`,
`min_clearance_m`, `margin_m`, `worst_pair`, `per_pair_m`, `segments_failed`,
`frozen_failed`, `paper_failed`, `frame_failed`, `column_failed`, `self_failed`.

### The hook, from python

```python
from aris_sixarm import progress

with progress.stage("trace", source=path) as end:
    strokes = trace_it()
    end["n_strokes"] = len(strokes)

with progress.substage("allocation", "balance"):
    ...

if progress.active():                 # guard anything the payload costs
    progress.item("allocation", what="probe", spans=expensive())
```

With no sink attached `emit` is a global load and a comparison against None,
and the `active()` guard means an expensive payload is not built either.
`progress.recording()` collects events into a list for tests and notebooks.

Measured, with no sink: `emit()` 114 ns, `progress()` 48 ns, and the two
context managers 636 ns each (a generator object, created and thrown away).
The per-item calls are inside the probe and re-plan loops, so a 3000-stroke
picture pays **0.5 ms** for the whole run; the context managers are per stage
and per ban round, at most a few dozen times.  Against a 343-second
allocation that is 1.5 parts per million, and the byte-level diff above is
what says it is actually zero.

**The planner's behaviour is unchanged with the GUI off**, and that is measured
rather than asserted.  The same end-to-end run was executed twice on the same
machine, once with no sink and once with a JSONL sink attached to every event:

* `det_schedule.npz` — all **54 arrays identical**, element for element
  (`q_<arm>`, `seg_<arm>`, `u_<arm>`, `segpts_`, `segoff_`, the ink CSR block,
  every scalar).
* `det_program.json` — **identical** once the recorded wall times are removed.
* `det_strokes.json` — **byte identical**, sha256 and all.
* `det_schedule.json` — identical except **one field**:
  `phases[0].priority_search.wall`, `0.08312726…` against `0.08006644…`, which
  is a stopwatch reading and cannot be equal between two runs.

Nothing about what is drawn, by whom, in what order, or how far anything is
from anything else moved.

## The programme schema

`aris_sixarm/program_schema.py` turns the two artifacts a run writes —
`<name>_schedule.npz` (the conducted timeline) and `<name>_program.json` (the
allocation) — into ONE bundle: `bundle.json` plus `bundle.bin`.

    .venv/bin/python scripts/export_viewer_bundle.py \
        out/csail_schedule_h094_v14.npz out/csail_program_h094_v14.json \
        --out out/v14_bundle.json --summary out/csail_schedule_h094_v14.json

The schema is **strict**: `Bundle.from_dict` refuses an unknown key and names
it, and refuses a missing one.  That strictness is the direct answer to a real
bug — `writing.densify` read `plan["tilt"]` while `stroke_api` wrote
`lean_vec`, so every lean-rescued stroke executed pen-upright and all
forty-seven segments of the record said `0.0` degrees.  So:

> **A pen's lean has exactly one name.**  `Segment.lean_deg` is the lean the
> plan COMMANDED, in degrees.  `Segment.cone_deg` is the permission it was
> planned under.  There is no `tilt`, no `lean_vec`, no `max_lean_deg` and no
> `tilt_max_deg` in the schema; the exporter is the single place the older
> spellings are resolved.  `tests/test_program_schema.py` pins that the field
> set contains neither.

Records: `Meta`, `ArmTrack`, `Segment`, `Stroke`, `Dropped`, `InkChunks`,
`Phase`, `Clearance`, `Timings`, `Bundle`.  Every array field is a `BufferRef`
`{offset, length, dtype, shape}` into the side-car `.bin`, 8-byte aligned so
the browser can build a typed array over it with no copy.  A 200-second
six-arm programme is about 44 kB of JSON and 1.3 MB of binary.

`Clearance` is the one computed field: per-frame minimum distance for every
arm pair and every arm against itself, from `scene_check.pair_clearance` and
`scene_check.self_clearance` over the same chain points the verdict used.  It
is the POINTWISE distance — the shipped `per_pair` numbers are discounted by
the sweep term `0.55 * (stepd_i + stepd_j)`, so the sparkline is a hair
optimistic against the certified figure by design; the certified minimum is in
`Phase.per_pair_m` beside it.  On a long programme the series is strided to
keep it small, so use it to FIND a dip, not to certify one.

### The scene

`export_scene()` writes the rig-dependent half: per-arm base transforms, the
Franka meshes (from `viz/robot_model.load_model()` — the same welding and
decimation the meshcat viewer uses), the static installation as boxes, the
capsule radii table, and a **golden FK check**.  The server builds it in a
subprocess per (rig, tool) and caches it under `out/gui_cache/`.

`web/viewer/js/fk.js` is a second implementation of
`frames.link_frames_many` — necessary, because shipping eleven link poses per
arm per frame is fifteen megabytes and shipping seven joint angles is nothing.
Being a second implementation, it is guarded: the scene carries six joint
vectors and the link poses PYTHON computed for them, the viewer re-derives them
and refuses to draw (red banner) if the two differ by more than a micrometre.

### The tool model (2026-09-16)

The viewer used to draw the tool as **two cylinders** — a bar out along hand
x by `PEN_LAT` and a pencil down hand z by `PEN_EXT`. That is a faithful
picture of the two numbers the planner uses and a useless one for standing
next to the real arm and asking "is that what is bolted on?". It now draws the
**modelled holder**, on every hand:

* the two **Fat Franka Finger** blades (`meshes/fatfinger/fatfinger_leftfinger.obj`,
  one per finger, the right one mirrored as the URDF mirrors it),
* the printed **housing** and **cap**
  (`meshes/penholder/penholder22_{housing,cap}_hand.obj`),
* the **spring, sleeve, clutch** and the graphite buried in the bore, plus the
  72.5 mm of stick standing out of the tail — the URDF's own cylinders,
* the **20 mm of graphite** out of the cap at the 23° bore lean, and
* a small red **tip ball**.

`program_schema._tool_model()` reads every one of those out of
`assets/system_model/installation_fatfingers.urdf` (written by
`scripts/gen_system_model.py` from `rig_final.PENHOLDER22` /
`penholder22_lead` / `FATFINGER` — docs/SYSTEM_MODEL.md §7a–§7e) and composes
each `<visual>` down to **one hand-frame 4×4**, so the viewer's whole job is
`T_world_base @ F[9] @ T_part`. Nothing is re-derived: the picture is wrong
exactly when the URDF is.

**The tip ball is the exception, deliberately.** The URDF welds it at
`(PEN_LAT_HOLDER, 0, PEN_EXT_HOLDER)` off the TCP, but the viewer places it
from the *arm's own* `pen_lat_m`/`pen_ext_m` — literally the expression
`Scene3D.tipOf` evaluates — so it is on the planned tip by construction rather
than by two constants happening to agree. That they do agree is carried in the
scene as `tool_model.tip_urdf_err_m` and asserted at 1e-9 m by
`test_the_tip_marker_lands_on_the_planners_tip`, which rebuilds the viewer's
composition in numpy against `frames.tip_pos` for eight poses × six arms.

The **`tool model`** toggle (the old `pens` layer, relabelled; default **on**)
switches the whole thing. With it off the two-cylinder sketch comes back and
so do the **stock fingers** — the blades *replace* those on the real gripper
(§7d), so the two pairs are never drawn at once. The line under the HUD is the
legend, and it says what kind of number these are:

> tool model: 23° bore, tip 86 mm lateral / 149 mm below the hand-TCP, 20 mm
> graphite — the 2026-09-13 holder from the photo, not a touchdown

`ARIS_TOOL=inline` has no holder (there is no such assembly), so
`tool_model` is `null` and the sketch is all there is. `SCENE_CACHE_V` in
`gui/server.py` must be bumped by hand when the model moves: the scene stamp
hashes `aris_sixarm/**.py` and cannot see `assets/`.

Static bodies come from `system_model.bodies()` for the `proposed` rig — the
surveyed cage, table, paper, drop posts, gussets, clamps and plates — and from
each `ArmSpec.static_obstacles()` for every other rig, because the system model
is built from `layout.FLEET_PROPOSED` and drawing it under `final6_opt` would
put steel where there is none.

## The Drake meshcat view (2026-09-16)

**The three.js viewport is a schematic, and it always was.** `scene3d.js`
builds the fleet out of primitives — one `BoxGeometry`, three
`CylinderGeometry`, three `SphereGeometry` and some line buffers — and there is
not one mesh file anywhere under `web/`. That is the right trade for the thing
it is for: a timeline you scrub, strokes coloured by arm, a clearance witness
line you can click. It is the wrong thing to look at when the question is
"does the machine look right", which is what it was being asked on
2026-09-16 ("i think the models are all strange").

So there is a second window, and it shows the same numbers through the model
everything else is graded against:

    ARIS_RIG=proposed ARIS_TOOL=lateral .venv/bin/python scripts/meshcat_drake.py \
        --npz out/unknown_h0970_home_alt.npz \
        --program out/unknown_h0970_home_program.json \
        --only-arms 31,71 --loop

    #   http://frankastation.drl.csail.mit.edu:7009/

`scripts/meshcat_drake.py` parses `assets/system_model/installation_fatfingers.urdf`
into a pydrake `MultibodyPlant` and serves it through Drake's own
`MeshcatVisualizer` — the same pathway `~/git/franka_manipulation_station` and
`~/git/cc_experiment` use, which is why the arms look like arms. Everything
the URDF has is on the screen: the 80/20 cage, the table and its paper, the
seam bars, six inverted FR3s, the Fat Franka Finger blades, the penholder22
housing and cap, the graphite and the welded tip.

**The meshes.** The six arms wear `assets/system_model/meshes/fr3/*.gltf` with
their PBR textures. Those are not new and nothing was downloaded for them:
all nine geometry `.bin` files are **byte-identical** (sha256) to
`~/git/franka_manipulation_station/assets/franka_description/meshes/visual/`,
and `gen_system_model.py` has been writing them into `<visual>` all along —
the decimated `meshes/collision/*.obj` set it also vendors is for the gates.
`assets/system_model/meshes/HQ_FR3_SOURCES.json` records every hash and the
path it came from; `--write-provenance` regenerates it and `--check-meshes`
fails if a regenerated scene ever puts collision hulls in a `<visual>`. The
script carries a substitution map that would repair such a scene at parse
time; on today's model it is a no-op, and `tests/test_meshcat_drake.py` pins
that it stays one.

**What it draws.** `--npz` plays a conducted programme on one clock at
`--rate` real time and `--loop` repeats it. The ink is a poly-line per
stroke, grown a frame at a time from the plant's own `arm<id>_pen_tip` body —
not from a second kinematic chain, which is the point: the tip on the screen
is the tip the URDF welds, and `tests/test_meshcat_drake.py` pins it against
`frames.tip_pos` to **3.2e-8 m** over ten certified poses. The drawn points
sit within 0.5 mm of the paper plane (world z = 0 IS the paper's top face
here), which the same suite measures.

| flag | what it does |
|---|---|
| `--static` | pose and hold — the programme's first frame, or `Q_PARK_PROPOSED` with no `--npz` |
| `--only-arms 31,71` | hide the four unmounted arms. **Visual only**: `visible = false` on their meshcat paths, the plant untouched |
| `--rate`, `--loop` | playback speed; repeat forever |
| `--record` | publish a scrubbable meshcat animation instead of playing live — the ink ends up drawn in full, since meshcat animates transforms and not objects |
| `--no-camera` | leave meshcat's default camera alone (the default framing stays under the cage's 1.62 m beams, which an eye at beam height looks straight along) |

**One port, 7009, and it is a singleton.** 7000–7008 belong to other people's
scenes on this machine and 8765 is this server. The **Open in Drake Meshcat**
button on the Day 1 panel posts to `POST /api/meshcat`, which kills whatever
the GUI last started there and launches a fresh one for that job's npz — so
asking twice replaces the scene rather than failing to bind. The npz is
resolved and refused if it leaves the repo. The three.js viewer beside it is
unchanged and still loads itself from the bundle.

## Milestone status

| # | milestone | state |
|---|---|---|
| 1 | job panel, live progress, stage timeline, "where the time goes", counters, per-arm loads, event log, live 3D of strokes coloured by arm | **done** |
| 2 | 3D viewport (six arms, paper, table, cage, mounts), per-arm lane timeline, scrubber, play/pause/speed | **done** |
| 3 | clearance inspector: per-pair sparklines, click a dip to jump the scrubber and draw the witness line with the mm value | **done** |
| 4 | ink overlay (target vs drawn vs residual) and per-segment inspector (arm, s-range, σ_min, margin, tip error, lean/cone, draw time) | **mostly** — all three layers are drawn (grey target, the plan's own tip path in the arm's colour, magenta residual) and clicking a segment jumps the scrubber to it; what is missing is colouring the residual by CAUSE |
| 5 | layer toggles for every geometry family; A/B ghost diff of two programmes; workspace-map paint mode | **partly** — layer toggles (arms, tool model, paper, cage, mounts, strokes, ink, bases, capsule chain) are there, and the tool layer is the modelled holder rather than a two-cylinder sketch; A/B diff and the workspace paint mode are not |

### What is next

* **Residual by cause.** The `dropped` spans are in the bundle with their
  `s_range` and where they lie, and they are drawn in magenta; colouring them
  by *why* — no arm certified it, the arm that certified it could not fly to
  it, a parked partner stands on it, the balancer gave it back — needs the
  reason, and `allocate.leftover` does not currently record one.  The
  information exists at the moment of each refusal (`banned`, `unflyable`,
  `park_blocked`, `fly_given_m` all name it) and is thrown away by the time
  the hole list is assembled.
* **A/B ghost diff.** The bundle is already self-contained, so this is a second
  `Program` loaded into a ghost material and a diff of the segment tables.
* **Workspace paint.** `scripts/feasible_workspace.py` writes the JSON; it
  wants a texture on the paper plane rather than geometry.
* **Live arm poses during conduction.** Right now the arms move only once a
  programme exists; the conduct stage could emit a sparse pose per phase.
* **A stop-after-stage control** so an operator can iterate the tracer knobs
  without paying for the allocation (`--trace-only` and `--place-only` are in
  the form, but there is no "resume from here").

## Verification

* `tests/test_progress.py` — the hook is a no-op with no sink (including *not
  building the payload*), records faithfully with one, closes a stage on an
  exception, and never lets a broken sink reach the planner.
* `tests/test_program_schema.py` — strictness (unknown and missing keys),
  the one-name-for-lean rule, 8-byte alignment, the npz→bundle round trip,
  the segment index matching the timeline, the clearance series, and the
  scene's golden FK check matching `frames.link_frames_many` exactly.
* `tests/test_gui_backend.py` — a real subprocess planning a real three-stroke
  picture through the real front door: stage events arrive, the log is teed,
  the artifacts land in the job directory, the websocket replays then tails,
  cancel kills the process group, and a restarted manager adopts what is on
  disk without claiming a dead job is running.

Beyond the unit tests, the browser itself was driven: headless Chrome over CDP
against a live server, four screenshots (`out/gui_live.png`,
`out/gui_programme.png`, `out/gui_clearance.png`,
`out/gui_where_the_time_goes.png`), **zero console errors or exceptions** in
each, WebGL under SwiftShader, and the FK golden check silent — which is how
the `Infinity` bug and the arm-31-coloured residual were found in the first
place.

Run only these plus whatever covers the instrumented stages — the whole suite
is over an hour, and `tests/test_balance.py` alone is most of that:

    .venv/bin/python -m pytest tests/test_progress.py \
        tests/test_program_schema.py tests/test_gui_backend.py \
        tests/test_csail.py -q
