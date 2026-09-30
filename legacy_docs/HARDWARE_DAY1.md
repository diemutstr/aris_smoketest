# Hardware day 1 — two arms write the word "unknown"

**2026-09-16.  Three hours.  Arms 31 and 71 only.**

Pete's success criterion, verbatim:

> have them write the word unknown starting from underneath the one arm and
> then to the other arm so we can see how the two arms interact. and test our
> redundancy resolution planner on the real thing.

Written 2026-09-15, the day before. **Nothing in this document has been run on
a robot.** Every number in it is measured off a file in this repository or is
explicitly marked as the thing a step exists to measure. It is the day-1
instance of `docs/HARDWARE_LADDER.md`: rung 0 (survey), rung 1 (touchdown),
rung 2 (one arm one stroke), rung 4 (two arms) — rungs 3 and 5 are not in
today's box.

**Amended 2026-09-17.** The ladder now runs **hover word per arm → word
per arm → two-arm word** (§4.2b, then §4.3): `scripts/day1.py word --arm N`
plans and certifies the solo word on the spot, and `scripts/day1.py send`
is the one command that hands the resulting CSV to the box that runs the
arms (§5.3).

---

## 0. What the word is, and why it is where it is

`scripts/text_strokes.py` writes `out/unknown_strokes.json`: the word
"unknown" as **13 single-line Hershey strokes, 2.592 m of ink**, x-height
120 mm, the 'k' reaching 180 mm, baseline at **y = 1.8153** — which is the
middle row's own line, the line both arms stand on, and the seam plane
(`mounts.SEAM_Y = 1815.32 mm`). The ink runs **x = 0.350 → 1.450 m**, so:

| | x | why |
|---|---|---|
| first mark | 0.350 | 0.247 m outboard (west) of arm 31's J1 axis at 0.5967 |
| arm 31 | 0.5967 | under the 'n' |
| **hand-over** | **≈ 0.90** | the middle of the contested band — the point of the exercise |
| arm 71 | 1.2067 | under the second 'w' |
| last mark | 1.450 | 0.243 m outboard (east) of arm 71 |

The word starts under one arm and finishes under the other, and the hand-over
falls between them. That is Pete's sentence, turned into two numbers.

Look at **`out/unknown_strokes.png`** before anything else. It is two panels:
the whole 1.8034 × 3.63064 m sheet, and the contested middle with both J1 axes
and both 0.855 m reach circles drawn on it.

The letters are **centrelines, not outlines** — the pen path IS the glyph.
That is what a single-stroke font is for and it is why an ordinary font was not
used (§6 of `scripts/text_strokes.py`'s own docstring has the argument).

**Re-cutting the word takes one second and no re-plan of anything else:**

```
ARIS_RIG=proposed ARIS_TOOL=lateral python3 scripts/text_strokes.py unknown \
    --height 0.12 --x0 0.35 --x1 1.45 --y 1.8153 \
    --out out/unknown_strokes.json --png out/unknown_strokes.png
```

`--height` is the **x-height** (how tall 'u', 'n', 'o', 'w' come out; the 'k'
is 1.5× that). Every re-cut must be followed by a re-plan (§3) — the strokes
file is an input to the conductor, not an output of it.

---

## 1. Prerequisites — before anything is powered

Order matters: each step's output is the next step's input. Steps 1–3 are
`docs/HARDWARE_LADDER.md` §3 verbatim, narrowed to two arms.

### 1.1 Which height? — **ask Pete first, and do it first**

The whole day forks here. `layout.LAYOUT_PROPOSED["h"] = 0.970` is what
everything in `out/` was planned at; the steel **may be at 0.850**. Both are
prepared (§3), but they are *different files* and running the wrong one puts
the pen 120 mm into or above the paper.

**Measure it, do not accept it.** The number wanted is the **underside of the
mounting plate** above the **paper surface**, per arm, to ±3 mm.

> **HEIGHT IS NOT A PREFERENCE HERE — IT DECIDES WHETHER THE DEMONSTRATION
> EXISTS.** Measured on this word: at h = 0.970 arm 31 draws 1.09 m of it. At
> **h = 0.940 — thirty millimetres lower — arm 31 draws NOTHING AT ALL**, and
> the whole word falls to arm 71 (§4.1). An inverted arm that is closer to the
> paper must fold harder for the same reach, and the paper-chain gate bites
> first; arm 31 sits much closer to that limit than arm 71 does.
>
> So if the plate comes in low, the "two arms hand the word over" result
> quietly becomes "one arm draws the word". **Survey before you plan, and if
> the number is below 0.970, look at the per-arm allocation in the re-plan's
> log before believing you still have a two-arm demonstration.** Lower `--h`
> also means a cold leg cache: the 0.970 conduct took 700 s, the re-planned
> ones take ~3700 s.

### 1.2 Survey the two bases

`docs/BUILD_SHEET.md` §0 datum: origin at the marked paper corner, z = 0 at the
**paper surface**. Per arm, the **joint-1 axis** x and y (to the base bolt
circle centre, *not* a plate edge), the underside of the mounting plate z, and
the base yaw. Plus the paper height at the four corners of the drawable area.

```
python3 scripts/asbuilt_layout.py --template > out/survey_20260916.json
# fill in ONLY the 31 and 71 entries; delete the other four
python3 scripts/asbuilt_layout.py --survey out/survey_20260916.json --check
python3 scripts/asbuilt_layout.py --survey out/survey_20260916.json \
        --out out/asbuilt_20260916.json
```

`asbuilt_layout.py` already accepts a **subset** of arms — its `build_asbuilt`
loops over `survey["arms"]` and never cross-checks against six — so a two-arm
survey is a legal survey and reports coplanarity and deviation over the two
arms present.

**Passes when** both arms are within ±10 mm of the build sheet in xy and z, the
two plates are coplanar within 3 mm, and each yaw is within 1°.

**Report every deviation. Do not re-centre one arm to hide the other's.**

> **What to do when it fails.** A deviation that survives re-measurement is a
> re-plan, not a shrug: §3's commands take `--h`, and per-arm z/yaw go through
> `asbuilt_layout.load_asbuilt`. Note the standing caveat from the packaging
> audit: `load_asbuilt` has **no consumer** — `draw.py` and `csail_schedule.py`
> do not accept an as-built fleet. So an as-built survey today is a **report**
> that tells you whether to re-plan at another `--h`, not a fleet you can hand
> the conductor. If the survey disagrees with the nominal by more than the
> ±10 mm bar, say so out loud and re-plan at the surveyed height; do not
> hand-edit a fleet.

### 1.3 Read back the gripper width, per pen

The GUI commands **0.0432 m at 70 N** (`franka_control_gui.py::_PEN_GRASP_WIDTH`).
libfranka only calls a grasp successful **above `width − epsilon_inner`**, so
the number it reports back is a **lower bound on the real jaw gap** — and that
bound is what rules the holder build in or out (`docs/SYSTEM_MODEL.md` §7).
Record it in the survey's `gripper` block. It is the **only measurement** of
the tool assembly that exists; everything else about the pen is CAD and a
photograph.

### 1.4 Touchdown the pen tip — or knowingly skip it

**The shipped tip is a reading of a photograph.** `PEN_LAT_HOLDER = 0.0860369`
and `PEN_EXT_HOLDER = 0.0460262` are USER-SPECIFIED, from a picture and the
sentence *"it only juts out 3–4 cm max"*. The only measured tip in the whole
project is the legacy inline pen's `PEN_EXT = 0.110` (arm 31, 2026-07-12).

```
python3 scripts/touchdown_calibrate.py --self-test
ARIS_RIG=proposed ARIS_TOOL=lateral python3 scripts/touchdown_calibrate.py \
        out/touchdown_arm31.json --json out/touchdown_arm31_fit.json
```

6–8 contacts on one arm, **at different tool yaws and leans**, spread across
its reach; log the **joint vector at contact** and the paper height there. Use
`operator_impedance_helpers/jog_descend.py` (compliant descent at 0.003 m/s) or
`probe_surface.sh`. The pen is 86 mm *across* the hand, so a single touchdown
cannot separate lateral from axial — the script refuses a fit whose design
matrix does not span (`cond(A) > 20`) rather than reporting three digits of
noise.

**Read the fit's verdict on the paper-chain gate before doing anything else
with it.** A tip deeper than 0.0460262 eats the programme's thinnest margin
first, and `touchdown_calibrate.py` prints exactly that and exits non-zero.

**If the box will not hold a touchdown:** skip it, and then **the hover pass
(§4.1) is not optional and the first pen-down is a single stroke on scrap.**
An un-measured tip is exactly the error the hover pass exists to catch before
it becomes force.

### 1.5 Collision profile — write down which one is in force

| | thresholds |
|---|---|
| installation operator, after a MoveIt launch | 40/40/36/36/32/28/24 Nm, 50/50/60/30/30/30 N |
| `fr3drivers` default `sensitive` | 20/20/20/20/10/10/10 Nm + 20 N |

The driver's default is **half** the installation's. Which one is in force is
the difference between "the pen touched down" and "the arm crashed". Record it
per arm, in the survey.

### 1.6 Which stack

Today's exports feed two, and they answer two different questions:

| | file | what it proves |
|---|---|---|
| **B. Cartesian impedance** (`impedance_pathway_exec.py`, 50 Hz, k_z = 1500 N/m) | the pathway CSV of §5.1 | that the ink lands where the plan says, with a compliant pen |
| **D. `fr3drivers`** (libfranka 1 kHz, `position_velocity_accel`) | the joint bundle of §5.2 | **the redundancy resolution** — D is the only stack that takes JOINT input and therefore the only one that does not throw the planner's answer away and re-solve it |

Pete's second sentence — *"test our redundancy resolution planner on the real
thing"* — is a statement about **D**, or about B **with the q1..q7 columns
used as the nullspace reference**. B alone, ignoring those columns, re-solves
the redundancy itself and tests nothing of ours.

**Arm 71 has no IP and never has had one.** `backends.INSTALLATION_IPS[71] is
None`; `franka_control_gui.py::LIVE_ARM_IDS` is `{13, 31, 17, 97, 2}` — five
arms, not including 71. `Fr3BundleBackend.preflight` **refuses** a programme
containing an arm it has no IP for, which is how this surfaces at a desk rather
than at launch. **This is a prerequisite, not a footnote: arm 71 needs an IP
before it can be driven by stack D at all.**

### 1.7 The e-stop

**The physical e-stop is the abort path.** The software gate brakes at
2 rad/s² and the watchdog latches at 20 mrad, and `fr3drivers/STATUS.md` says
in as many words that **neither of those is the abort**. The GUI's
`emergency_stop()` is a software pause plus a kill. One person's only job, all
day, is the button.

---

## 2. What the planner produced, and what each file is for

**THE PROGRAMME TO FLY IS `unknown_h0970_home`.** Everything below is under
`out/`, all of it conducted and certified on 2026-09-15.

### 2.1 The one that matters

`unknown_h0970_home` — the word at the nominal height, **both arms returning
to their parks at the end**. Measured:

| | |
|---|---|
| coverage | **100.0000 %** — 2.5922 m traced, 2.5922 m drawn, 0 spans empty |
| strokes | 13, none cut, none handed between arms |
| arm 31 | 7 segments, 1.09 m, 12.4 s drawing + 23.1 s pen-up |
| arm 71 | 6 segments, 1.50 m, 17.9 s drawing + 20.6 s pen-up |
| makespan | **53.02 s** (14.5 s of it conducted pause) |
| **min inter-arm** | **58.83 mm** (gate 50) — pair **31-71** at t = 34.18 s |
| frame, incl. seam bars | 54.2 mm (gate 50), arm 71 at t = 41.44 s |
| neighbour base column | 155.6 mm (gate 50) |
| paper | chain 29.9 mm (gate 20); tip −0.5 mm (floor −10) |
| self-collision | 41.3 mm (gate 20) |
| joint margin | 0.1035 |
| verdict | **PASS**, and independently re-checked by `recheck_timeline.py` |

Both arms start **and** end at `layout.Q_PARK_PROPOSED`, verified to
**0.00000 rad**. That is the property the whole day rests on — see §4.3.

### 2.2 Every file

| file | what it is |
|---|---|
| `unknown_strokes.json` / `.png` | the word on the paper, and the picture of it |
| `unknown_h0970_home_schedule.npz` + `_program.json` | **the concurrent programme** |
| `unknown_h0970_home_recheck.json` | the independent whole-timeline verdict |
| `unknown_h0970_home_alt.npz` + `_alt_recheck.json` | **the ALTERNATING programme — the one to draw with** |
| `unknown_h0970_home_timing.json` | the timing-tolerance certificate (§4.4) |
| `unknown_h0970_home_timing_dense.json` | the positive-side sweep at 0.25 s steps |
| `unknown_h0970_home_retime.json`, `_alt_retime.json` | peak accel before/after, fleet rate |
| `pathways/unknown_h0970_home_arm{31,71}.csv` | the pathway CSV, **q1..q7 on every row** |
| `pathways/unknown_h0970_home_alt_arm{31,71}.csv` | ditto, alternating |
| `bundles/unknown_h0970_home[_alt]_arm{31,71}.npz` | the `fr3drivers` joint bundles |
| `unknown_h0970_home_topdown.gif` | the plan view — **look at this first** |
| `unknown_h0970_hover_*` | the hover pass (§4.1) |
| `unknown_h0850_*` | the 0.850 branch, if the steel measures that |

**Superseded, kept only for comparison:** `unknown_h0970_*` (without `_home`)
is the same word conducted under the FREEZE idle policy. It certifies as a
concurrent programme (61.77 s, 53.05 mm) but **its alternating variant does
not** — see §4.3. Do not fly it.

**A rehearsal is already running** at
<http://frankastation.drl.csail.mit.edu:7008/static/> — the concurrent
programme, looping, in meshcat.

---

## 3. The re-plan commands — the only thing to run if a measurement moves

Everything below is `scripts/draw.py`, the **proven conductor path**: it is the
path v19 took to draw the CSAIL logo at 100 % coverage. The one thing that is
new is that its `source` may now be a stroke **case file** (`.json`), which
skips the tracer *and the placement search* so the word stays exactly where it
was put. Nothing else in the pipeline changed.

**At the nominal height (h = 0.970):**

```
ARIS_RIG=proposed ARIS_TOOL=lateral python3 scripts/draw.py \
    out/unknown_strokes.json --out unknown_h0970 --arms 31,71 \
    --atlas out/atlas_proposed_h0970_lat0860 --tilt-max-deg 15 \
    --min-len 0.02 --fps 48 --substeps 1 --subcheck 2 \
    --skip-unconductable --freeze-refused-phase --depot-hover-selective \
    --residual-passes 6 --residual-min-gain 0.0005 \
    --program --no-anim
```

> ## **AT h = 0.850 THIS WORD CANNOT BE DRAWN. Conducted and measured, 2026-09-15.**
>
> The command below was run in full. It produced
> `out/unknown_h0850_schedule.npz`, and that file is **13.5 s long and draws
> 1.23 % of the word**: arm 31 moves 0.0329 rad and has 12 drawing frames out
> of 650; **arm 71 does not move at all** (0.0000 rad). `skipped_phases` is
> `['single pass']` and `skipped_m` is **2.15 m of the 2.59 m**.
>
> **Do not be fooled by the log's headline.** It says
> `COVERAGE 84.2074 %` — that is the ALLOCATOR's number, what the planner
> believes it could cover. The CONDUCTED coverage, in the summary json, is
> **`coverage: 0.0123`**. `--skip-unconductable` is exactly the flag that lets
> that gap open quietly: every phase the conductor refused was skipped and the
> run still reported success. **Always read `coverage` out of
> `out/<name>_schedule.json`, never the COVERAGE line in the log.**
>
> **Why it fails.** The conductor's own cross-check rejects phase after phase
> with `sequencer priced transits the timeline does not pay` at **18.8 s,
> 33.6 s, 35.0 s, 36.3 s** of disagreement. At 0.970 the same check trips at
> **0.0267 s** — three orders of magnitude smaller. These are not accounting
> noise; they say the realised pen-up transits need long detours the sequencer
> never priced, because an inverted arm 120 mm closer to the paper has to fold
> so far that the straightforward hover-to-hover crossing no longer exists.
>
> **So if the steel measures 0.850, the honest answer to Pete is: this word, at
> this size, in this place, is not drawable — the day's options are to trim the
> verticals to 0.970, or to re-cut the word smaller and re-plan.** Re-cutting is
> one second (`scripts/text_strokes.py --height ... --x0 ... --x1 ...`); the
> re-plan after it is ~an hour of cold cache. Decide early.

**At 0.850, if that is what the steel measures — but read the box above first:**

```
ARIS_RIG=proposed ARIS_TOOL=lateral python3 scripts/replan_at_height.py \
    --h 0.850 --atlas out/atlas_proposed_h0850_lat0860 \
    --parks out/park_search_h0850_lat0860.json -- \
    out/unknown_strokes.json --out unknown_h0850 --arms 31,71 \
    --tilt-max-deg 15 --min-len 0.02 --fps 48 --substeps 1 --subcheck 2 \
    --skip-unconductable --freeze-refused-phase --depot-hover-selective \
    --residual-passes 6 --residual-min-gain 0.0005 \
    --program --no-anim
```

> **THE LAST FIVE FLAGS ARE NOT OPTIONAL AND THEY COST AN HOUR TO LEARN.**
> They are v19's own job parameters, and a run without them does not finish.
> Measured today, twice, on this word: the conductor refuses the phase with
>
> ```
> sequencer cost model vs frozen timeline: worst disagreement 2.67e-02 s
> !! could not be conducted as allocated: sequencer priced transits the
>    timeline does not pay: arm 31: 0.0267 s
> ```
>
> — a **deterministic 26.7 ms mispricing on one arm-31 transit**. The rescue
> ladder then recurses (both arms → `{31}` and `{71}` → arm 31's tour cut in
> two, twice), certifies two fragments at VERDICT PASS, and bottoms out on a
> third that fails identically every time.
>
> **Neither of the two obvious levers touches it.** `--reseq-tries` cannot:
> `cross_check` raises `SystemExit`, and the retry loop in
> `csail_schedule.build_phase` catches only `idle.Unconductable`, so the
> re-sequence never runs. `--no-verify` cannot either: it gates the
> unsplit-alternative allocation in `run_allocation`, not this check. Nor is it
> the balancer — a `--no-split` run fails at the same 26.7 ms on the same arm.
> `--skip-unconductable` is what lets the rescue ladder ship the fragments that
> DID certify, and `--residual-passes` is what re-allocates the hole the
> skipped one leaves. `--depot-hover-selective` is worth its place on its own:
> without it arm 31 gave back 64 mm of stroke 3 and arm 71 27 mm of stroke 6;
> with it both are kept whole on a hover further round the fiber.
>
> **Read the coverage line in the log before trusting any output.** A run that
> skipped a fragment draws less of the word than it was asked for, and
> `--skip-unconductable` is precisely the flag that lets it do so quietly.

Three things about these commands are deliberate and must not be "tidied":

- **`--arms 31,71` restricts the ALLOCATION, not the scene.** The other four
  arms stay in the model as parked metal. That is correct and it is measured:
  for arms 31 and 71 the nearest box belonging to an absent arm is **1.003 m**
  away, and the six-arm and two-arm static sets certify identical cells over
  150 sampled cells each. Planning against six therefore costs nothing here and
  keeps every published number comparable.
- **`--substeps 1` with `--fps 48`** keeps the conductor's own 1/48 s
  coordination clock while making the npz **un-decimated**. A strided npz is
  what `Fr3BundleBackend.preflight` refuses and what `from_schedule` warns about
  on every load: `scene_check` graded the full-rate path, so shipping the
  decimated one ships a certificate for a file you are not flying.
- **`--tilt-max-deg 15`** is what v19 used. Changing it changes the atlas the
  run is entitled to read.

---

## 4. The run ladder — in this order, and no skipping

Each rung: **what runs / what to log / passes when / abort rule.**
The abort rule is the same for every rung and is stated once: **the physical
e-stop.** Nothing below overrides it.

### 4.1 Rung A — the hover pass, per arm, then both

**The first thing to run, and it never touches the paper.**

**What it is, and why it is a different file.** The hover programme is the same
word planned with the arms **30 mm closer to the paper** (h = 0.940 against the
0.970 rig). Flown on the rig as built, the pen tip therefore rides **30 mm
above** the paper everywhere. This is not the drawing programme with a flag set
— it is a separately conducted, separately certified programme, because a
trajectory whose pen is somewhere else is a different trajectory and
`scene_check` has to say so.

`out/unknown_h0970_hover_schedule.npz`, conducted 2026-09-15. Measured:

| | |
|---|---|
| **coverage** | **74.29 %** — 1.9479 m of 2.5922 m; 0.6665 m in 4 spans left empty |
| makespan | 89.52 s |
| min inter-arm | 119.0 mm (gate 50) |
| frame, incl. seam | 50.9 mm (gate 50), arm 31 at t = 0.00 s |
| paper | chain 24.8 mm (gate 20); tip −1.5 mm (floor −10) |
| phases | **6** — four rescue groups plus two residual passes |
| verdict | **PASS** |

**And the construction is verified, not merely argued.** `recheck_timeline.py`
takes no `--h`, so it grades this programme against the **shipped 0.970
fleet** — which is precisely the "planned at 0.940, flown on the real rig"
case. It reports:

```
  min inter-arm 119.02 mm (gate 50) pair 31-71 at t = 46.02 s -> margin +69.02 mm
  paper chain 55.8 mm (gate 20) arm 71; tip 27.6 mm (floor 10) arm 71
  VERDICT PASS
```

**`tip 27.6 mm`** is the pen's minimum height above the real paper over the
whole programme. The conduct's own `scene_check`, run at 0.940, put the same
tip at **−1.5 mm** — on the paper, in its own frame. The 30 mm offset is
therefore a measured number and not a geometric hope, and **27.6 mm is the
figure §4.1's pass criterion should be read against**: expect the ruler to say
28 mm, not 30.

> ### **STOP. THIS PROGRAMME NEVER MOVES ARM 31.**
>
> Measured: arm 31 draws **0 segments, 0.00 m** — in the main pass and in every
> one of the six residual passes. Its bundle comes out with peak joint speed
> **0.000 rad/s** over all 4212 samples. The whole 74 % is arm 71 working
> alone.
>
> So this file **cannot** prove arm 31's tracking, its tip height, or its
> tool transform. Flying it and calling rung A passed would leave arm 31
> completely untested going into the first pen-down. **Do not do that.**
>
> **Why it happens, and why it matters far beyond the hover.** Thirty
> millimetres closer to the paper is a materially tighter geometry for an
> INVERTED arm: it has to fold harder for the same reach, and the paper-chain
> gate (elbow ≥ 20 mm off the paper) is what bites first. At h = 0.970 arm 31
> draws 1.09 m of the word. At 0.940 it draws **nothing**. Thirty millimetres
> is the whole difference.
>
> **That is a hardware risk, not a hover artefact.** It says arm 31 is sitting
> much closer to its limit than arm 71 is, and that if the as-built plate is
> even slightly lower than 0.970, arm 31 loses its share of the word and the
> "two arms interact" demonstration quietly becomes one arm drawing. **The
> survey number in §1.2 is therefore not bookkeeping — it decides whether the
> day's headline result is possible at all.**
>
> **A 15 mm hover was attempted and DOES NOT EXIST.** `unknown_h0970_hover15`
> at h = 0.955 was conducted and ended `no phase could be conducted` — 15
> refusals, arm 71's transit mispricing running 14–31 s. **There is no npz.**
>
> Two caveats on that attempt, stated because they make the result weaker
> evidence than it looks: it was run against the **0.970 atlas and the 0.970
> park set** (no sweep exists at 0.955, and a park search is ~90 minutes), so
> it planned against a reachability map for a height it was not at. A proper
> 15 mm hover needs `height_sweep.py sweep --h 0.955` (~1 min) **and** a park
> search at that height (~90 min) before it is worth believing. **Not
> attempted today; budget for it if a two-arm hover pass matters.**
>
> **So today's only hover file is the 30 mm one, and it is arm 71 alone.** Run
> it as arm 71's dry run (`--solo 71`). **Arm 31 has no hover pass**, which
> means its first powered motion is either a supervised jog or the drawing
> programme itself — decide which, deliberately, before the day starts.
>
> **One more thing about the 30 mm file.** It has six phases against the
> drawing programme's one, so five barriers to acknowledge on the day's first
> powered run, and its frame gate sits at **50.9 mm against a 50 mm gate** —
> 0.9 mm — on arm 31 at t = 0.00 s, i.e. standing at its park.

> **A clean hover pass is not a certificate for the drawing run** and could
> never be: a different height is different IK is a different joint path. It
> proves the stack, the tracking and the tip height, and nothing about the
> drawing programme's own trajectory.

```
# rehearse it in meshcat first, at quarter speed, one arm at a time
ARIS_RIG=proposed ARIS_TOOL=lateral python3 -m aris_sixarm.execute \
    out/unknown_h0970_hover_schedule.npz out/unknown_h0970_hover_program.json \
    --check --play --solo 31 --rate 0.25
# ...then on the robot, through the stack chosen in §1.6
```

**Log:** joint tracking error per arm against the **20 mrad** latching watchdog
(`--fr3_tracking_fault_rad = 0.020`, 3 ticks); the **joint-5 static offset**
(the 2026-09-10 lab run measured ~**9 mrad** under position control, with
tracking peaks of 8–12 mrad, on an arrival bar of 15–20 mrad — if joint 5 is
quiet today, say so, because that would be new); and the **measured tip height
above the paper** at three points along the word.

**Passes when** both arms complete with no reflex, no tracking fault, no gate
rejection, and the measured tip height is **28 ± 5 mm** everywhere (the planner's
own minimum over this programme is 27.6 mm, verified above — not 30).

> **If the tip height is not 30 mm, STOP and do not draw.** The discrepancy is
> the tool transform, the mounting height, or the paper plane, and you now have
> a number for it: a tip 8 mm low means either the plate is 8 mm lower than
> surveyed or `PEN_EXT_HOLDER` is 8 mm long. Re-plan at the height that makes
> it 30 mm and run the hover pass again. **This is the whole reason the hover
> pass is first.**

### 4.2 Rung B — one stroke per arm, on paper

The smallest thing that makes ink. Use `--solo`, one arm, one stroke — the
word's first stroke for arm 31, its last for arm 71 — and **on scrap taped over
the real sheet** the first time.

```
ARIS_RIG=proposed ARIS_TOOL=lateral python3 -m aris_sixarm.execute \
    out/unknown_h0970_schedule.npz out/unknown_h0970_program.json \
    --solo 31 --check
```

> **`--solo` prints a frozen set and says it is not certified.**
> `FleetProgram.solo()` returns the mover's track **and the poses the other
> arms must be frozen at**, with `recheck_required = True`, because the
> conducted timeline was certified with both arms MOVING. A frozen fleet is a
> different scene. The re-check is `scripts/recheck_timeline.py` against the
> serialised file of §4.3, which is a scene of exactly that shape — so **run
> §4.3's re-check before §4.2, and read its verdict as §4.2's certificate.**

**Log:** pen-down tip position against plan; the drawn line against the planned
one, sampled at three points, against `validate.TIP_TOL` = **2 mm**; contact
force or the absence of a reflex; the collision profile in force.

**Passes when** the ink is on the paper, within 2 mm of plan, with no reflex.

> **Joint position control is stiff.** A 1 mm height error becomes pen force
> with nothing to absorb it — the installation's own answer to this is stack
> B's Cartesian impedance (k_z = 1500 N/m), which stack D does not have.
> Whether a position-controlled pen draws acceptably is **unmeasured**, and
> this rung is where it gets measured. If the line is a scratch or a skip,
> switch to stack B for the drawing rungs and keep D for the joint reference.

### 4.2b Rung B+ — the SOLO word, one arm at a time

**Added 2026-09-17, on Pete's ask: "write unknown under the two center arms …
we'll just write first with one arm and then with the other. for the first we
want to float over the paper and write just to double check."**

This is the rung between one stroke and two arms, and it is the one that gets
run today. It is `line` with thirteen strokes: the same certified single-arm
planner per stroke, the same `writing.arm_program`, the same independent
whole-timeline check with the other five arms frozen at their parks, the same
three files. `scripts/day1.py` **plans it here and now** — unlike §4.3, which
re-checks a file made the day before.

```
# arm 31: float first, then draw.  Then arm 71, the same two.
ARIS_RIG=proposed ARIS_TOOL=lateral python3 scripts/day1.py word --arm 31 --hover
ARIS_RIG=proposed ARIS_TOOL=lateral python3 scripts/day1.py word --arm 31
ARIS_RIG=proposed ARIS_TOOL=lateral python3 scripts/day1.py word --arm 71 --hover
ARIS_RIG=proposed ARIS_TOOL=lateral python3 scripts/day1.py word --arm 71
```

The word is `scripts/text_strokes.py`'s single-stroke Hershey "unknown", sized
to `--width` (0.55 m) and **centred on that arm's own J1 axis** — arm 31's at
x = 0.5967, arm 71's at x = 1.2067 — with its baseline `--dy` off the seam
line, default **−0.10 m**.

> **The default `--dy` is not a taste.** Arm 31 refuses a line whose baseline
> is exactly on the seam line y = 1.8153: the pen-up leg to or from its park
> does not clear the paper plane there (`tests/test_day1.py` pins that
> refusal). −0.10 m clears it for both arms; if a day's geometry moves and
> −0.10 refuses, try `--dy -0.15` and `--dy -0.20`.

**Measured 2026-09-17** (proposed rig, lateral tool, h = 0.970, seam bars on,
all six arms in the scene, the other five frozen at their parks):

| run | verdict | strokes | inter-arm | frame | self | paper chain / tip | duration | CSV rows | planning wall (cold / warm) |
|---|---|---|---|---|---|---|---|---|---|
| `--arm 31 --hover` | **PASS** | 13/13 | 151.0 mm | 52.7 | 29.2 | 54.9 / **+27.1** | 63.5 s | 2828 | 20.4 / 20.3 s |
| `--arm 31` | **PASS** | 13/13 | 164.4 mm | 67.8 | 33.0 | 23.6 / −3.2 | 67.8 s | 2685 | 51.0 / 50.8 s |
| `--arm 71 --hover` | **PASS** | 13/13 | 151.8 mm | 67.0 | 31.0 | 52.3 / **+28.9** | 85.5 s | 3801 | 33.8 / 33.6 s |
| `--arm 71` | **PASS** | 13/13 | 138.1 mm | 67.1 | 31.9 | 23.0 / −2.3 | 67.2 s | 2975 | 24.1 / 24.1 s |

Every gate has room against its own bar (inter-arm/frame/column 50 mm, self 20,
paper chain 20, paper tip floor 10). The hover rows read **+27 to +29 mm of
real tip clearance** — the hover is a 30 mm longer pen in the PLAN and the
certificate is re-derived with the real one, so that number is the ruler's
expected reading, not the asked-for 30.

**Where the planning wall goes**, printed by every run (`wall
plan_strokes … cost_matrix … solve … arm_program … scene_check … export`).
Cold and warm are the same to within 1 %: every run is a fresh process and
nothing is cached on disk, so there is no warm-up to have.

| | strokes | cost matrix | solve | `arm_program` | `scene_check` | export | total |
|---|---|---|---|---|---|---|---|
| 31 hover | 2.4 | 0.3 | 0.1 | 4.7 | 12.7 | 0.1 | **20.2 s** |
| 31 | 2.9 | 0.2 | 0.4 | 33.0 | 13.6 | 0.1 | **50.2 s** |
| 71 hover | 2.3 | 0.3 | 0.1 | 12.5 | 18.2 | 0.1 | **33.5 s** |
| 71 | 2.5 | 0.2 | 0.1 | 6.6 | 14.4 | 0.1 | **23.8 s** |

The thirteen certified stroke plans are 2.5 s of it and the exact Held-Karp
order is 0.1 s. **The cost is pen-up routing and the whole-timeline check**,
which is why the sequencer's cost matrix is built WITHOUT the paper-routing
screen by default: routing all 676 crossings up front costs ~100 s (410 of
them dive) and buys a tour 3 s cheaper in transit. Instead the cheap matrix
proposes an order, `writing.arm_program` refuses any leg it cannot fly and
NAMES it, that one edge is marked infinite, and the solver is asked again —
0.07 s a round. Arm 31 pen-down needs four such rounds and is the 50 s run;
the other three need none. **No gate is anywhere in that trade**: every
certificate is still `plan_stroke`'s per stroke, `arm_program`'s per leg and
`scene_check`'s over the finished timeline. If the retries run out the routed
matrix is built after all, which is the answer the sequencer would have given
from the start.

**What comes out**, exactly as `line` does, in `out/day1/`:
`unknown_<arm>.{npz,csv,json}` for a pen-down run and
`unknown_hover_<arm>.{npz,csv,json}` for a float. The CSV is the impedance
pathway file of §5.1 with **q1..q7 on every row — the planner's own redundancy
resolution for that waypoint**, which is the half of Pete's criterion the
drawing itself does not test, **plus a `t_s` column** (below).

**Refusal rule, and it is stronger than `line`'s.** A stroke the certified
planner will not certify stops the whole run and **nothing is written** — a
word with a letter missing is not the word. `--allow-partial` writes the
certified subset instead and says so in the PASS line and in the json. An
unflyable pen-up leg, a failed scene check, or a joint reference that exceeds
the FR3's velocity limits are all likewise refusals with no CSV.

**The CSV carries a clock — `t_s`, and it is the LAST column.** The exporter's
v1 file has no time in it at all (`stroke_idx, wp_idx, kind`, the pose,
`intensity`, `q1..q7`). `day1.py` appends `t_s`: seconds from the start of that
arm's programme, **on the same clock as the schedule npz** (timeline frame *k*
is *k/fps*, fps = 48), which is the planner's own pacing. It is appended last
**so a positional reader of the first eighteen columns is unaffected**, and
`aris_sixarm/export/pathway.py` is **not** edited for it — the times are
reconstructed in `day1.py` by re-running the exporter's own `_emit_frames`, and
the reconstruction is checked before anything is written.

> **The cross-check that makes `t_s` trustworthy.** The manifest's per-stroke
> `t_start_s` is computed by the exporter through a completely different route
> (`strokes_of`, off the timeline). The first `draw` row of every stroke in the
> CSV lands on it to **3 × 10⁻⁷ s**, measured, and `tests/test_day1.py` pins
> that. A row count the reconstruction cannot account for is a refusal with no
> CSV, not a guessed column.

The file starts at the first certified approach frame and ends at the last
retract frame, so `t_s` runs **1.79 → 61.23 s inside a 63.5 s programme** (arm
31 hover) rather than 0 → duration. That is the same clock, not a rebased one:
the park-to-hover lead-in is in the npz and is deliberately not in the CSV.
Row spacing is **not** uniform — the exporter collapses a run of identical
poses to one row, so a hold shows up as one row and a 0.208 s gap.

**The joint-speed gate, and it is computed from `t_s`.** Every other gate in
this file is about where the arm is; this one is about how fast the joint
reference moves, which is what a stiff controller turns into torque. The PASS
line reports the peak per-joint speed as a fraction of `frames.QD_MAX`
(2.62 2.62 2.62 2.62 5.26 4.18 5.26 rad/s) **at the elapsed time between the
rows' own timestamps**, and again at 1 kHz linear interpolation of those same
rows; the json and the manifest carry all seven numbers plus the peak
accelerations. Charging a collapsed hold the nominal frame period instead would
invent a speed nothing moves at. **All four runs read 30.0 % of the limit** —
`writing.QD_FRAC` is 0.30 and the pacer hits it. Anything near 100 % is a bug
upstream, not a tight day. An over-limit reference is a refusal with no CSV.

> **THE VELOCITY CERTIFICATE HOLDS AT `t_s` AND NO OTHER TIMING.**
> `rtff_pathway_exec` paces by **Cartesian arc length** and can traverse the
> same path faster or slower than the planner did. Run it quicker than `t_s`
> says and every speed above scales with it. That sentence is in the summary
> json (`joint_speed.validity`), in the manifest (`t_s.note`) and on the PASS
> line (`joint speed OK **at t_s**`), because the file outlives the run.

**Log:** the tip height at three points for each hover pass (**28 ± 5 mm or
stop**); the drawn word against the planned one; whether the pen-up legs
between letters look like the file; the joint tracking error against the
20 mrad watchdog.

**Passes when** the word is legible, the ink is within 2 mm of plan, and no
reflex, tracking fault or gate rejection happened.

> **The word is CRAMPED at these defaults, and that is arithmetic, not a bug.**
> At `--height 0.10` the natural width of "unknown" is 0.90 m, and `--width`
> is 0.55 — so the tracking is **−58 mm per gap** and the letters are pushed
> into each other. The run says so, loudly, every time. 0.55 m is what one
> arm can reach; a legible word at that width wants `--height 0.061`
> (x-height ≈ width ÷ 9). **Decide which you want before the run, not after
> looking at the ink.**

---

### 4.3 Rung C — the word, ALTERNATING

**This is the run that satisfies Pete's criterion, and it is the safe one.**
The file is already built: `out/unknown_h0970_home_alt.npz`. It was made by

```
ARIS_RIG=proposed ARIS_TOOL=lateral python3 scripts/serialise_timeline.py \
    out/unknown_h0970_home_schedule.npz --out out/unknown_h0970_home_alt.npz \
    --order 31,71 --recheck --json out/unknown_h0970_home_alt_recheck.json
```

Two blocks, **106.06 s** total. Arm 31 flies its whole track while arm 71
stands at its park; then arm 71 flies its whole track while arm 31 stands at
its park. **The two arms are never both in motion**, and the re-check says:

```
  min inter-arm 155.56 mm (gate 50) pair 31-71 at t = 87.44 s -> margin +105.56 mm
  VERDICT PASS
```

The seam between the blocks has **no step in either arm** — 31 holds exactly
the pose it ended on, 71 starts from exactly the pose it was held at — so there
is no `Barrier.reposition` to supervise. That matters: `HARDWARE_LADDER` §2.4
measured v18 stepping **0.0320 rad** at a barrier and calls it *uncertified
motion into a stiff controller*. This file has none.

> ### **WHY THE ARMS GO HOME, AND WHY THE OTHER FILE IS NOT SAFE**
>
> The first programme conducted for this word used the default FREEZE idle
> policy. It certified as a concurrent programme at 53.05 mm — and its
> alternating variant **FAILED at 8.79 mm**, a hard collision.
>
> The cause is worth understanding, because it is not obvious and it will
> recur. Under `freeze`, an arm finishing its bag **retreats off the paper and
> stops there** — measured, arm 31 ended **4.66 rad** from its park and arm 71
> **4.75 rad**. Serialising then holds arm 31 at that stopped pose for the
> whole of block 2, and that pose is squarely in arm 71's path. The arm was
> never certified to *stand* there while its neighbour worked; it was only
> certified to *pass through* while the neighbour was elsewhere.
>
> `--idle-policy home` makes each arm return to `Q_PARK_PROPOSED` at the end of
> its bag. Both arms then start and end at their parks — verified to
> **0.00000 rad** — and the parks are a set that was searched to be mutually
> clear and re-searched against the seam bars on 2026-09-14. The held pose
> becomes a certified park instead of wherever the pen happened to stop, and
> the alternating clearance goes from **8.79 mm FAIL to 155.56 mm PASS**.
>
> It is also *faster*: 53.02 s against 61.77 s, because going home beats
> holding a pose the next phase has to work around.
>
> **The rule this gives you: never serialise a programme whose arms do not end
> at their parks.** `serialise_timeline.py` will tell you — it re-checks and
> refuses to write a file that does not certify, which is exactly how this was
> caught rather than discovered on the robot.

**Why alternating is first.** The inter-arm certificate is a statement about
two arms at the same instant on **one clock**, and there is no cross-process
fleet clock (`ARCHITECTURE_V2` §5 Q3: the biggest structural gap in the
ladder). With only one arm ever moving, the certificate does not depend on a
clock at all, and a human can witness the hand-over.

**Log:** the hand-over — where arm 31 stops and where arm 71 starts, and the
realised gap between the pens at that moment; the joined letters across
x ≈ 0.90 (is the 'k'/'n' junction continuous?); total ink against plan;
makespan against the file.

**Passes when** the word is legible, both hand-overs happened where the plan
says, and no pair got closer than the re-check's certified minimum minus the
calibration allowance.

### 4.4 Rung D — the word, CONCURRENT — **read this before deciding**

Measured on `unknown_h0970_home`, arm 71's clock displaced against arm 31's:

| Δt (arm 71) | min inter-arm | |
|---|---|---|
| −10 s | −119.17 mm | **FAIL** |
| −5 s | −112.77 mm | **FAIL** |
| −2 s | −60.43 mm | **FAIL** |
| −1 s | −48.89 mm | **FAIL** |
| −0.5 s | −24.56 mm | **FAIL** |
| **0** | **58.83 mm** | PASS |
| +0.5 s | 88.93 mm | PASS |
| +1 s | 48.00 mm | **FAIL** (by 2 mm) |
| +2 s | 106.21 mm | PASS |
| +5 s | 100.50 mm | PASS |
| +10 s | 155.56 mm | PASS |

**`certified_window_s` = 0 symmetrically.** −0.5 s already fails, so no |Δt|
above zero survives in both directions at once. But the failure is strongly
one-sided, and a 41-point sweep of the LATE side at 0.25 s steps
(`out/unknown_h0970_home_timing_dense.json`) shows the structure:

| Δt band | min inter-arm over the band | |
|---|---|---|
| any negative | −24 mm down to −119 mm | **COLLISION** |
| 0 → +0.75 s | 58.8 – 88.9 mm | PASS |
| **+1.00, +1.25 s** | **48.0, 35.8 mm** | **FAIL — the notch** |
| +1.50 s | 50.98 mm (+0.98 mm) | PASS, but marginal |
| **+1.75 → +10 s** | **61 – 156 mm**, every one of 34 points | **PASS** |

> **ARM 71 MUST NEVER START EARLY. THIS IS THE SAFETY RULE OF THE DAY.** Arm 31
> is priority 1 and draws first; arm 71's own schedule already contains
> **14.5 s of conducted pause** before it moves. Start arm 71 early and it
> walks into arm 31 while arm 31 is still drawing — **−119 mm at −10 s is not a
> near miss, it is a collision.**
>
> **And "a second later" is exactly the wrong instruction.** The notch at
> +1.00/+1.25 s is real, it bottoms at **35.8 mm**, and one second is precisely
> the delay a person would pick by instinct. Two seconds is safe; one is not.

**The decision rule for the day.** Rung D is permitted *only* if all of:

1. **arm 31 is started first and is visibly drawing before arm 71 is started**,
   with at least **2 s** between the two starts — not one — and ideally more,
   since everything from +1.75 s to +10 s clears by 61 mm or better;
2. somebody is on the e-stop watching the middle of the paper specifically;
3. rung C has already run cleanly.

Otherwise **rung C is the deliverable and rung D is skipped.** Pete's criterion
is already met by rung C; the concurrent run buys a livelier demonstration and
carries the only collision risk of the day.

> **What the grid does not prove.** It is a grid. The band from +1.75 s to
> +10 s passed at all 34 sampled points, but clearance is not monotone in the
> skew — the notch is the proof — so a narrower notch between two samples
> cannot be ruled out, and nothing beyond +10 s was tested. Re-run with
> `--dense` or a finer `--shifts` list if this number has to carry more weight
> than "start it late, and by more than a second".

The symmetric zero is itself the measurement `ARCHITECTURE_V2` §5 Q3 asks for:
it says the cross-process fleet clock has to be built before six arms can ever
run a piece like this concurrently **without a stated start order**.

**Log:** the realised inter-arm clearance against the certified one, and the
skew actually achieved between the two starts (timestamp both, to the second).

**Passes when** both arms complete, the realised clearance is above the
certified minimum minus the calibration allowance, and the measured skew stayed
inside the passing band the whole run.

---

## 5. The two export formats, and what each one carries

### 5.1 The pathway CSV — for the impedance stack (B)

```
ARIS_RIG=proposed ARIS_TOOL=lateral python3 -m aris_sixarm.export.pathway \
    --schedule out/unknown_h0970_schedule.npz \
    --program  out/unknown_h0970_program.json \
    --rig proposed --tool lateral --arms 31 71 \
    --out out/pathways --name unknown_h0970
```

Columns, one file per arm:

```
stroke_idx,wp_idx,kind,x_m,y_m,z_m,qx,qy,qz,qw,intensity,q1..q7
```

- **Frame: the arm's own `fr3_link0`**, metres. The exporter never applies a
  transform — the row poses come from `frames.fk(q)`, which already IS the base
  frame. `T_world_base` is recorded in the manifest, not in the rows.
- **Pose = the EE frame the robot is configured with** (`setEE`, the nominal
  pen tip). Quaternion order in the file is `qx, qy, qz, qw` — **scalar last**.
- `kind ∈ {travel, draw, lift}`. **The transits are in the file and they have
  to be**: the planner RECONFIGURES the arm inside its pen-up transits (measured
  on v18: 4.39 rad on joint 3 between two strokes), and a straight-line hover
  travel cannot flip a wrist. A CSV of draw rows only is **not executable**.
- `intensity` ∈ [0, 1] is **TONE**, the only channel by which the artwork
  controls pressure. The production band is 0.7–1.0 N over 9 levels — one tone
  step ≈ **0.04 N**. Today's export is a single constant intensity: the word is
  one weight of line.
- **`q1..q7` are on every row.** This is the file that answers *"test our
  redundancy resolution on the real thing"*: it is the controller's nullspace
  reference, the planner's own choice of arm configuration, carried alongside
  the Cartesian pose rather than thrown away and re-solved.

> **The baked `z_m` is advisory.** The executor overrides it with the live
> measured plane and adds press along the pen axis. Say this out loud to
> whoever runs it, because a CSV that looks like it commands a height does not.

**Every claim above was read back off the shipped manifest**, not taken on
trust — `out/pathways/unknown_h0970_home_arm31.manifest.json` says
`format.frame = fr3_link0`, `quaternion.order = xyzw`,
`quaternion.ee_frame = nominal pen tip (setEE)`, `joint_columns = true`,
`transits.in_file = true` with 749 `travel` rows, `intensity.mode = constant`
at 1.0, and `warnings: []`. Three more numbers from it worth having on the day:

| | |
|---|---|
| `T_world_base` translation, arm 31 | (0.5967, **1.81532**, 0.970) — the surveyed base, and the height this file assumes |
| `paper_z_base_m` | 0.970 — what a `draw` row's `z_m` will read |
| `max_fk_deviation_from_plane_m` | **9.8 µm** — every row's FK agrees with the commanded plane to ten microns |
| `speeds.draw_m_s_measured` | **0.0867 m/s**, against the 0.12 m/s asked for — the fleet clock is what it is, and the executor should not be told to expect 0.12 |

### 5.2 The joint bundle — for `fr3drivers` (D)

```
ARIS_RIG=proposed ARIS_TOOL=lateral python3 scripts/retime_bundle.py \
    out/unknown_h0970_schedule.npz out/unknown_h0970_program.json \
    --out out/bundles --tag unknown_h0970 --hz 1000 \
    --json out/unknown_h0970_retime.json
```

**The acceleration problem, and exactly how far this closes it.** The gate is
`--fr3_max_joint_acceleration = 10.0` rad/s², which the driver's own flags call
*"the single most dangerous field"*. `pacing.py` bounds joint **velocity and
nothing else** and says so: *"the v profile here is a ceiling, not a
trajectory"*. The shipped six-arm programme demands 33–37 rad/s².

`retime_bundle.py` applies a **time scaling and nothing else**. Scaling the
clock by 1/s multiplies every sampled speed by s and every sampled acceleration
by s², so the smallest admissible s is closed-form, not a search, and the
duration grows by exactly 1/s. **Every control point is untouched**, which is
the bundle format's own argument for why the collision certificate survives.

> **What this does NOT fix, said plainly.** The path is piecewise linear in
> joint space, so at a corner the TRUE acceleration is impulsive however slowly
> it is flown. What the scaling bounds is the acceleration **the driver
> measures between the samples it is sent**, which is what the gate tests. A
> genuinely C1 path needs a blend or a real TOPP pass — a *planning* change
> this script deliberately does not make. `--max-slowdown` refuses to call a
> bundle flyable if the factor is absurd, and names the file
> `..._NOT_FLYABLE.npz` so it cannot be picked up by accident.
>
> ### **THE ACCELERATION NUMBER DEPENDS ON THE RATE YOU MEASURE IT AT, and
> that is the finding of 2026-09-15**
>
> Measured on v19's arms 31 and 71 through this script:
>
> | measured at | peak &#124;q̈&#124; | scaling needed | verdict |
> |---|---|---|---|
> | the conducted samples (48 Hz) | **37.70 / 36.82** rad/s² | **1.94×** slower | fixable |
> | resampled to the driver's **1000 Hz** | **1422** rad/s² | **11.9×** slower | **NOT FLYABLE** |
>
> Both numbers are of the same trajectory. The second is larger because
> halving the sample spacing doubles the finite-difference acceleration across
> a corner, without limit — which is the impulse above, showing up as soon as
> you look closely enough. So *"the programme needs to be 1.94× slower"* is
> true only of the 48 Hz command stream; it is **not** a claim about what
> libfranka's 1 kHz loop will see if it interpolates between those samples
> itself.
>
> **What to do about it tomorrow.** Run `retime_bundle.py` **both ways** —
> plain, and with `--hz 1000` — and put both numbers in the log. Then hand
> `fr3_sender.py --dry-run` the bundle and let the driver's OWN gate rule; that
> verdict is the one that counts, and it is free. If the 1 kHz figure is what
> the gate tests, the honest answer for day 1 is that **stack D is not flyable
> for this programme** and the drawing runs go through stack B, with the joint
> columns used as the nullspace reference. That still tests the redundancy
> resolution, which is the point.
>
> **One rate for the whole fleet.** The script reports a single `FLEET RATE` =
> the slowest arm's. Re-timing one arm and not the other moves them against
> each other at instants nobody certified — `execute.Governor`'s entire
> argument, and the reason it is one scalar.

The bundle is **degree 1** — exactly the chords `scene_check` graded, bit for
bit, which is honest and which also means **discontinuous velocity at every
knot**. Preflight with `fr3drivers/tools/fr3_sender.py --dry-run` and
`tools/preflight.py` (three verdicts: CHAIN / ROBOT FIT / SCENE) before any
powered attempt.

---

### 5.3 Getting a CSV onto the arms — file in, one command out

> **REWRITTEN 2026-09-17 against the deployed branches.** The first version of
> this section was read off the 2026-09-09 briefing and was wrong in three ways
> that matter: the CSV goes to **`/tmp/impedance_pathway_arm<N>.csv`**, not
> into `pathway_persist`; the supervisor needs **five `RTFF_*` variables** in
> front of it; and **there is no runner for these arms** — the supervisor takes
> the CSV as `$1` and that is the whole interface. Everything machine-specific
> now lives in **`config/site.json`** (`day1.py site` shows it, `--set` edits
> it); no address is hard-coded in any script.

`scripts/day1.py send` is the whole interface between this repository and the
running control stack. It **copies one file and prints one command**; it never
moves an arm from this machine.

```
scripts/day1.py send --arm 31 --file out/day1/unknown_31.csv --dry-run
scripts/day1.py send --arm 31 --file out/day1/unknown_31.csv
```

It refuses anything that is not a pathway CSV with this repo's columns, then:

1. `scp <file> <operator>:/tmp/impedance_pathway_arm<N>.csv`
2. prints the park pose and **how far row 0 is from it**, the **stack-health
   check**, the one draw command, and the log to tail:

```
ssh diemut@192.168.50.2 'RTFF_CONTACT_DESCEND=0 RTFF_FORCE_SIGN=1 \
    RTFF_TRAVEL_SPEED=0.02 RTFF_MODE=observe RTFF_DEPART_LIFT=0 ARM_ID=31 \
    bash ~/RTff/draw_rtff_supervised.sh /tmp/impedance_pathway_arm31.csv \
    1.0 2.5 5 fresh'
ssh diemut@192.168.50.2 'tail -f /tmp/rtff_draw_arm31.log'
```

Each `RTFF_*` is off by default and each default is wrong for this file:
`CONTACT_DESCEND=0` flies the planned z instead of feeling for the paper,
`MODE=observe` keeps depth open-loop for the first passes, `TRAVEL_SPEED=0.02`
is 20 mm/s, and `DEPART_LIFT=0` is because the certified programme already ends
at the park.

> **THE PLAN IS A POSITION, NOT A ROBOT.** `--arm 31` names the **left-middle
> position** and `--arm 71` the right-middle; the CSV's poses are in that
> position's base frame. **Which arm ids are bolted into them is not known**
> (2026-09-17: "may be 97 and 71, position unknown"), so `send --as-arm <id>`
> dispatches a slot's file to whatever arm is actually there — `ARM_ID`, the
> DDS domain and the `/tmp` file name all follow the physical id, and the
> printed line says `slot 31 -> arm 97 (base frame of the left-middle
> position)`. Identify the arm first and record it with
> `day1.py site --set slot31.arm=97 --set slot31.mounted=true`.

> **WHAT THE EXECUTOR ACTUALLY READS.** The tip pose of each row, and nothing
> else: it paces by **Cartesian arc length**, holds each row's orientation
> without slerping, and latches its own nullspace. **It ignores `q1..q7` and
> `t_s`.** Those columns are the record of what the planner chose and the basis
> of the velocity certificate; on the deployed stack they are a check, not a
> command — so today's runs test the tool, the plane and the pose path, and
> *not* the planner's redundancy resolution. Say so if asked.

> **THE START POSE IS ROW 0, NOT THE PARK.** The executor's first move is an
> uncertified straight ramp from the arm's measured configuration to row 0, and
> it cannot execute our park→hover joint transit — it walks rows as a Cartesian
> path with its own nullspace. The certified programme does park at both ends,
> but the exporter begins the CSV at the first frame whose tip is 50 mm clear
> of the paper: **measured, about 0.6 m and 4 rad from the park** on the day-1
> word files. So the arm is driven to **row 0's joints** under POSITION control
> (the operator's `go_start_pos.py`) and the executor's ramp is then zero.
> `send` prints those joints as **START POSE** every time, and
> `send --from-q <measured joints>` **refuses to dispatch** unless the arm is
> within **0.05 rad on every joint and 10 mm at the tip** (`--allow-ramp`
> overrides, deliberately).
>
> **And the arm STOPS at the last row**, because `RTFF_DEPART_LIFT=0`. `send`
> prints that as **END POSE**; return to the park under position control
> afterwards. `day1.py park --arm N` prints the park pose, and
> `--from-q <end joints>` plans and certifies a collision-free joint path back
> to it — a CHECK that the way is clear, since the deployed executor cannot
> execute a joint path.
The supervisor is what runs the ladder gate, drives MoveIt to the start,
switches `fr3_arm_controller` → `cartesian_impedance_controller`, and calls
`rtff_pathway_exec.py --csv <file>`. The arm is chosen by the **`ARM_ID`
environment variable — the DDS domain, not a flag and not an IP.**

**Host, user, remote path, arm ids, IPs, measured paper-z and the `RTFF_*`
environment all live in `config/site.json`** — one tracked file, printed at the
top of every run, shown by `day1.py site` and edited by
`day1.py site --set KEY=VALUE`. `--host` and `--remote` override it per run.
**`--live` is the only way `send` ever runs the draw command over ssh**, and the
default is to print it so a person types it on the operator box with their hand
near the e-stop.

> **Three things `send` prints every time.** (a) The stack must be HEALTHY
> first — hardware active, three controllers, `robot_mode 2`; **never heal on a
> user stop (mode 5) or guiding (mode 3)**. (b) `fresh`, not `resume`: the
> runner decides fresh-vs-resume from a progress file, and a new CSV dropped
> next to an old checkpoint is picked up as a resume of the old one. (c) **The
> ladder gate is SKIPPED for inverted arms, and both middle positions are
> inverted** — nothing downstream will measure the paper plane for them, and
> `RTFF_CONTACT_DESCEND=0` means nothing will feel for it either, so the `z_m`
> baked into the CSV is the plane they will draw at. That is exactly why
> §4.2b's hover pass and a ruler come first, and why `--paper-z` exists.

> **There is NO runner for these arms.** The deployed tree has
> `run_forever_arm13.sh` / `run_forever_arm17.sh` and a `dispatch_arm17_*.sh`,
> and **no runner, no dispatch script and no `pathway_persist` folder for the
> middle positions**. `send` therefore targets the supervisor directly, which
> takes the CSV as `$1`, and leaves any keeper loop alone. If a
> `run_forever_arm<N>.sh` exists by then, **hold it first**
> (`bash ~/RTff/aris_hold.sh hold <N>`) or the keeper will start its own pass
> on top of this one. There is also a **retired** box at 192.168.50.4 whose
> `~/RTff` is a stale copy; never point `--host` at it.

### 5.4 The same thing with buttons — the GUI's "Run on arm" group

`python -m aris_sixarm.gui` → http://localhost:8765. Under the Day 1 panel, in
the left column. It **runs `scripts/day1.py send` as a subprocess** and shows
what it printed: no address, no supervisor line and no `RTFF_*` variable is
built in the GUI, so this section and that file cannot drift apart. Every
command is echoed verbatim (`$ ssh -o BatchMode=yes …`) above its own output,
and a command that cannot reach the operator box shows its refusal and its exit
code rather than spinning.

| control | what it runs | when it is available |
|---|---|---|
| **Site setup** → Save site | writes `config/site.json` (host, per slot: arm id, paper-z, mounting confirmed) | always |
| **Identify arms** | `ARM_ID=<id> source ~/impedance_helpers/arm_env.sh && timeout 5 ros2 topic echo --once /joint_states` for ids 31/71/97, every 4 s; poses the ones that answer in the 3D viewer with the tool model, and offers each to Drake Meshcat | always |
| **Check stack** | `bash ~/RTff/aris_hold.sh stack <id>` | always — **green only on `STACK HEALTHY`**, and the raw output is shown either way |
| **Copy to operator (dry run)** | `day1.py send --arm <slot> [--as-arm <id>] --file <csv> --dry-run` | always — copies nothing |
| **RUN (observe mode)** | the same with `--live`, as an ordinary job whose output streams into the log pane | **only** with a green stack check for that arm (15 min) **and** `RUN <slot>` typed into the confirm box |
| **Hold** / **Kill executor** | `aris_hold.sh hold <id>` (keeps the checkpoint) / `arm_pkill rtff_pathway_exec` | once a run has been started |
| **Start log tail** | `ssh … 'tail -n 50 -f /tmp/rtff_draw_arm<id>.log'` into the panel, stoppable | always |

The gate is the SERVER's, not the button's: `POST /api/operator/run` refuses
without both conditions, so a `curl` at the endpoint meets the same refusal.
The slot → arm mapping is printed on every line the panel and the job produce
(`slot 31 -> arm 97 (left-middle)`, and `MOUNTING NOT CONFIRMED` until somebody
has ticked it). **The physical e-stop is the abort. Nothing in this panel is** —
it says so, in red, above the buttons.

---

## 6. Every assumption in today's files, and what to do when it is wrong

| # | assumption | where it lives | how you find out | what to do |
|---|---|---|---|---|
| 1 | **mounting height h = 0.970** | `layout.LAYOUT_PROPOSED["h"]` | §1.1/§1.2 survey; and the hover pass reads 30 mm | re-plan: §3's 0.850 command, or `replan_at_height.py --h <measured>` with the atlas and parks for that height |
| 2 | **the pen tip** `PEN_EXT_HOLDER = 0.0460262`, `PEN_LAT_HOLDER = 0.0860369` | `frames.py` | §1.4 touchdown; the hover pass's measured height | **re-plan everything.** The tip is inside `atlas.is_current`'s model signature, so a moved tip stales every sweep in `out/` |
| 3 | **the paper plane is flat and at z = 0** | the datum | four-corner probe in §1.2; the executor's own plane fit | the impedance stack measures the plane itself and overrides `z_m`; the joint bundle does **not** — a tilted plane is a reason to prefer stack B today |
| 4 | **the seam bars are where `SEAM_BARS_MM` says** — x = −0.1905..−0.1143 and 1.9177..1.9939 m, y = 1.8153 ± 0.0762, z = −0.027..1.624 | `mounts.py` | eyes and a tape | **see the box below — this is the assumption most likely to bite this particular pair** |
| 5 | **the other four arms are absent but modelled as parked metal** | `--arms 31,71` | — | measured harmless here: nearest absent-arm box is 1.003 m away, identical certification over 150 sampled cells. **Leave it alone.** |
| 6 | **CSV frame = `fr3_link0`, quaternion `qx,qy,qz,qw` scalar-last, no yaw fudge** | `export/pathway.py` | the exporter's own FK-vs-pose gate on every row; `test_export_pathway.py` reproduces the deployed generator's `(1,0,0,0)` for a floor arm pen-down | if the operator's executor disagrees, **do not patch the exporter** — compare against the deployed generator's own output for the same arm first |
| 7 | **arm 71 has an IP** | `backends.INSTALLATION_IPS[71] is None` | preflight refuses | §1.6. This is a networking job, not a planning one, and it blocks stack D for arm 71 entirely |
| 8 | **park poses certify at the running height** | `layout.Q_PARK_PROPOSED` (searched at 0.970) | `scene_check`'s frozen-pose gate in the re-check | the 0.940 grid applied at 0.850 lands **−126 mm inside another arm's ink**. Never carry a park set across a height — use that height's own `park_search_*.json` |

> ### The seam bars bite THIS pair, and there is a measurement to prove it
>
> The two seam bars went into the certified static set on **2026-09-14**. The
> shipped six-arm programme **v19 was conducted on 2026-09-10**, four days
> earlier — so v19 never saw them, and re-checked against them it **FAILS**:
>
> ```
> $ scripts/seam_impact.py ... out/seam_impact_v19_seam.log
>   min frame clearance -59.4 mm (margin 50 mm), arm 31 at t=13.94 s
>   FAIL: arms [31, 71]
>   VERDICT FAIL
> ```
>
> Reproduced independently today by `scripts/timing_tolerance.py` at Δt = 0,
> which reports `frame_failed [31, 71]` while agreeing with v19's published
> inter-arm number to the digit (50.32 mm, pair 13-31, t = 65.98 s).
>
> **The two arms the bars fail are 31 and 71 — the middle row, and tomorrow's
> entire fleet.** That is not a coincidence: the bars stand on the middle row's
> own line (y = 1815.32 mm), 114.3 mm outboard of the canvas edge at each end,
> over the whole 1651 mm from tabletop to runway. They are the one piece of
> steel that is *closest to these two arms and nobody else*.
>
> Everything planned today was conducted with `mounts.SEAM_POSTS_ON` true, so
> the bars are inside the certificate rather than outside it. But it means:
>
> - **Do not fall back to any pre-2026-09-14 file.** v19 and everything beside
>   it is stale for this pair specifically, and stale in the direction that
>   hurts.
> - **Look at the west and east ends of the table before powering up.** The bar
>   geometry is `SEAM_SOURCE`-flagged **REPRESENTATIVE**, not measured: "a bar
>   as wide as two of the corner struts". If the real steel is wider, deeper,
>   or somewhere else, the gate that matters most to these two arms is wrong.
>   Measure it and say so; it is a tape-measure job and it is worth ten minutes.
> - `ARIS_SEAM_POSTS=0` reproduces the pre-seam numbers exactly. It is for
>   regression comparison **only**. Never fly anything planned with it.

---

## 7. The one-page card for the day

1. **Ask Pete the height.** Everything forks there.
2. Survey both bases → `asbuilt_layout.py --check`. Report deviations.
3. Gripper width read-back, per pen. Collision profile, per arm. Write both down.
4. Touchdowns on arm 31, or knowingly skip and accept §4.1 as the safety net.
5. **Hover pass, per arm, then both.** Measure the tip height. **28 ± 5 mm or stop.**
6. One stroke per arm on scrap, then on paper. 2 mm of plan.
7. **The SOLO word, per arm** — `day1.py word --arm 31 --hover`, then
   `--arm 31`, then the same two for 71. All four PASS as planned 2026-09-17.
   `day1.py send --arm N --file out/day1/<name>_N.csv` puts it on the box.
8. **The word, alternating** — `out/unknown_h0970_home_alt.npz`, 106 s,
   155.56 mm of clearance. **This is the deliverable.**
9. The word, concurrent — only with **arm 31 started first and arm 71 at least
   2 s later, never earlier, never at 1 s.**
10. One person on the e-stop, all day, doing nothing else.

**The three numbers to carry in your head:**

| | |
|---|---|
| **58.83 mm** | how close the two arms come, concurrent, on one clock |
| **155.56 mm** | how close they come alternating — the safe run |
| **2 s** | the minimum delay before starting arm 71. One second collides. |

**Abort rule, every rung: the physical e-stop.** The software gate brakes at
2 rad/s² and the watchdog latches at 20 mrad, and the driver's own notes say
neither of those is the abort path.
