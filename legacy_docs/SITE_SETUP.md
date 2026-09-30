# Site setup — installing and running the planner on a machine at the installation

Written 2026-09-15, the evening before the first hardware day with arms **31**
and **71** mounted. Every command in sections 1 and 2 was **run**, in a
throwaway venv outside the repo, against a clean `git archive` of HEAD — not
against the working tree that wrote them. Where something was not run it says
so. Where something failed it is in section 5 with the failure text.

This document is the fixed sequence. It is not a tour of the planner; that is
`README.md`. It assumes nothing is installed and nobody has this repository.

**Revised 2026-09-16 — the IK bindings are now vendored (§1.3) and a clone
installs with nothing to download.** A bundle is no longer needed to *install*;
it is still how you carry the `out/` artefacts (§1.5) to a machine without
network. The install is three pip lines (§1.4).

That revision was re-verified end to end, not assumed. A fresh
`git clone -b aris2` into a throwaway venv on python 3.12.3, with
`ARIS_FRANKA_IK_PATH=/nonexistent` **and the workstation build tree made
invisible to that interpreter** — so neither fallback in §1.3 could fire:

| | |
|---|---|
| `pip install -r requirements-site.txt` | 11 s |
| `pip install ./third_party/franka_analytical_ik` | **7 s**, `batch: True`, loaded from the venv's `site-packages` |
| `pip install -e .` | ok |
| `pytest tests/test_gates.py tests/test_hardware_prep.py -q` | **20 passed** |
| `day1.py line --arm 71 --from 1.15,1.95 --to 1.30,1.95` | **PASS**, inter-arm 253.4 mm, σ ≥ 0.235, tip 1.2e-12 m |
| `day1.py word --variant alt` | **VERDICT PASS**, inter-arm 155.6 mm at t = 87.4 s |

The negative control was run too: with the vendored package *uninstalled* and
the workstation tree still invisible, the import fails with the new
`ImportError` in §5.2 rather than silently succeeding. That is the failure mode
the old import order hid, and it is now the one a robot PC would actually see.

**Make the bundle on the workstation, carry the bundle, follow this file.**

```bash
scripts/make_site_bundle.sh          # -> out/site_bundle_<date>.tar.gz, ~43 MB
```

---

## 0. Machine requirements

| | what was proved | what is probably fine |
|---|---|---|
| OS | Ubuntu 24.04.4 LTS (noble), x86-64 | 22.04; anything with glibc ≥ 2.35 |
| python | **3.12.3** | 3.10–3.13 — but see the wheel caveat below |
| RAM | 8 GB is plenty for planning | 4 GB will plan one stroke; an atlas sweep wants more |
| cores | 32 available; **the planner used 1** for everything in §2 | 4+; atlas sweeps take `--jobs` |
| disk | bundle 43 MB → 65 MB unpacked; venv ≈ 700 MB | 2 GB free |
| network | needed **once**, for `pip install` | see §1.6 for the offline path |
| apt packages | `build-essential`, `libeigen3-dev`, `python3-venv` | — |

**The python version no longer matters much.** The IK bindings are vendored
as *source* (§1.3) and compile for whatever python runs pip, in about five
seconds, on any version. The only python-3.12-specific artefact left is the
prebuilt `ik_wheel/*.whl` in bundles built before 2026-09-16, which nothing
needs any more — ignore it.

**Nothing here is a robot.** `aris_sixarm` contains no rclpy, no libfranka, no
LCM and no socket. It ends at a file. A machine running this cannot move an
arm, and that is a design property, not a gap in the install.

---

## 1. Install

Everything is relative to the unpacked bundle directory.

```bash
tar -xzf site_bundle_<date>.tar.gz && cd site_bundle_<date>
```

### 1.1 System packages

```bash
sudo apt update
sudo apt install -y build-essential libeigen3-dev python3-venv
```

`libeigen3-dev` is needed to **compile** the IK bindings (header-only, used at
build time only). It installs to `/usr/include/eigen3`; if it lands elsewhere,
set `EIGEN_INCLUDE` before §1.3.

### 1.2 The venv

```bash
python3 -m venv ~/aris_venv
source ~/aris_venv/bin/activate
python -V        # expect 3.12.x
pip install --upgrade pip
pip install -r repo/requirements-site.txt
```

`requirements-site.txt` pins the exact versions that were verified together.
It is a **new file**; it does not modify `pyproject.toml`, which is left
exactly as it is. If the site python is not 3.12 and a pin refuses to build,
drop the pins and `pip install -e './repo[dev]'` — nothing in the pin set is
load-bearing for correctness, it is reproducibility only.

### 1.3 The analytic IK bindings — the one dependency that is not on PyPI

Since 2026-09-16 this is **vendored in the repository** and there is nothing to
fetch, carry or build by hand:

```bash
pip install ./repo/third_party/franka_analytical_ik      # from the bundle
pip install ./third_party/franka_analytical_ik           # from a clone
```

It compiles in about five seconds. `libeigen3-dev` (§1.1) and `pybind11`
(in `requirements-site.txt`, §1.2) must already be there.

**Verify it imported, and that the batch path is live:**

```bash
python -c "import aris_sixarm.ik as ik; print(ik._IK.__file__); print('batch:', ik._IK.has_batch)"
```

Expect a path inside your venv's `site-packages/franka_analytical_ik/` and
`batch: True`.

**What it is.** A C++ pybind11 extension from
**`github.com/wernerpe/franka_analytical_ik`** (Pete's own repo, Apache-2.0),
pinned at commit **`0d38d9667b7c1d45acd65e698911e76437405533`** — the commit
that added the *batch* entry points (`solve_batch`, `fk_batch`,
`tip_jacobian_batch`). Older commits are **correct but much slower**:
`aris_sixarm/ik.py` detects the batch functions at call time and falls back to
a python loop without them. Full provenance, including what was left behind
and why: `third_party/franka_analytical_ik/VENDOR.md`.

<details>
<summary><b>Why this does not use the upstream bazel build</b> (read if the above fails)</summary>

Upstream builds with bazel, and `build_wheel.sh` → `bazel build //:franka_ik_wheel`
is the documented route. Two things make it the wrong route for a site machine:
its wheel target hardcodes **`python_tag = "cp310"`** (root `BUILD`) and
`MODULE.bazel` pins a 3.10 toolchain, while the planner runs 3.12; and a
toolchain fetch needs network and builds a large cache (102 GB on the
workstation that has one).

The vendored `setup.py` is a setuptools + pybind11 shim over the **identical
two source files** (`franka_analytic_ik_bindings.cpp`, `franka_ik_He.hpp`). It
builds for whatever python runs pip, in about five seconds, with no bazel.

It was checked, not assumed: the shim-built extension exports the **same six
entry points** as the bazel-built `.so` the workstation has been planning with,
and over **300 random poses × 4 branches × 7 joints** the two agree with
**max absolute difference 0.0** and identical NaN masks. It is the same solver.
</details>

**The older bundle layout.** Bundles built before 2026-09-16 carry the same
sources as `ik_src/` at the bundle root, plus a prebuilt `ik_wheel/*.whl` valid
only for python 3.12 on a glibc at least as new as the build machine's. Both
still work (`pip install ./ik_src`, `pip install ik_wheel/*.whl`); neither is
needed any more.

**The env-var escape hatch, and how the import order changed.**
`aris_sixarm/ik.py` now tries the **installed package first**, and only if that
fails looks for a *build-tree directory* — `ARIS_FRANKA_IK_PATH` if it is set
and exists, then the old hardcoded workstation path
`/home/franka/aris_project/franka_analytical_ik/franka_analytical_ik`. If none
of the three resolves it raises an `ImportError` naming the pip command above.

This order is the fix for a real trap. Until 2026-09-16 the workstation path
was tried **first**, so on the one machine that has a build tree — the
workstation everything is developed on — a completely missing install imported
happily, and the failure only appeared on the robot PC. Set
`ARIS_FRANKA_IK_PATH` only if you are pointing at a build tree on purpose.

### 1.4 The repo

```bash
pip install -e ./repo
```

So the whole install, from a clone, is three pip lines:

```bash
pip install -r requirements-site.txt
pip install ./third_party/franka_analytical_ik
pip install -e .              # or -e '.[gui]' for the browser GUI
```

### 1.5 The `out/` artefacts

`out/` is gitignored, so the archive carries none of it. The bundle does:

```bash
mkdir -p repo/out && cp -r out/* repo/out/
```

What is in there and why, in §3c.

### 1.6 Offline install

`pip download -r repo/requirements-site.txt -d wheels/` on any networked
Ubuntu 24.04 / python 3.12 machine, carry `wheels/`, then
`pip install --no-index --find-links wheels/ -r repo/requirements-site.txt`.
The IK step is already offline: `third_party/franka_analytical_ik` is in the
checkout and compiles locally. **This was not run** — it is the standard pip
flow, stated for planning.

---

## 2. Verify — both of these before anything is powered

### 2.1 The gates

```bash
cd repo
python -m pytest tests/test_gates.py tests/test_hardware_prep.py -q
```

**Verified output:** `20 passed, 1 warning in 0.88s`
(the warning is meshcat importing a deprecated pyzmq ioloop; ignore it).

Run it with **no environment variables**. `test_gates.py` asserts the shipped
constants — the arm-31 touchdown of 2026-07-12, `MZ = 0.924`, `PEN_EXT = 0.110`
— and those are stated at the default rig. Setting `ARIS_RIG` or `ARIS_TOOL`
for this command tests something else. See §6.

### 2.2 The planning smoke test

```bash
cd ..            # back to the bundle root
ARIS_RIG=proposed ARIS_TOOL=lateral python smoke_plan.py
```

**Verified output:**

```
arm 31: base xy = (0.5967, 1.8153)
tool: ext = 0.0460262 m, lat = 0.0860369 m
status = 'ok'  (0.2250 m, 23 lattice steps)
  46 joint samples over 11.25 s
  min_sigma  = 0.2164   (strict-GO gate 0.14)
  min_margin = 0.1586 rad
  max_step   = 0.0120 rad
  tip_err    = 1.12e-12 m  (validator tolerance 2e-3)
  validation: ok=True violations=[]
SMOKE OK
```

This plans one real stroke on **arm 31** through `stroke_api.plan_stroke` and
requires the plan to pass `validate.py`, which re-derives everything from
`frames.fk` and trusts no planner bookkeeping. `tip_err = 1.12e-12 m` is the
number that says the C++ IK is wired up correctly — a bad build gives either an
import error or centimetres.

It needs **no atlas**, which is why it is the install test.

> **Do not use `scripts/demo_stroke.py` as the smoke test.** It is stale at
> HEAD and dies with `KeyError: 'qs'`. Details and proof in §5.1.

---

## 3. First day on hardware — `HARDWARE_LADDER.md` §3 as commands

Read `docs/HARDWARE_LADDER.md` §0 first; it says what the control stack is and
what this repo cannot do. Order matters: each step's output is the next step's
input.

Throughout: `ARIS_RIG=proposed ARIS_TOOL=lateral` on every planning command.
`ARIS_TOOL=lateral` switches planner, atlas, validator, capsules and router
together, and omitting it plans the legacy inline pen.

### 3a. Survey → check → as-built layout

```bash
cd repo
python scripts/asbuilt_layout.py --template > out/survey_20260916.json
# fill it in by hand, then:
ARIS_RIG=proposed ARIS_TOOL=lateral python scripts/asbuilt_layout.py \
    --survey out/survey_20260916.json --check
ARIS_RIG=proposed ARIS_TOOL=lateral python scripts/asbuilt_layout.py \
    --survey out/survey_20260916.json --out out/asbuilt_20260916.json [--calib 0.005]
```

**What to measure**, in the `docs/BUILD_SHEET.md` §0 datum — origin at the
marked corner of the **1803.4 × 3630.6 mm** canvas rectangle, z = 0 at the
**top surface of the paper**:

- per arm: the **joint-1 axis** x and y (the centre of the base bolt circle,
  **not a plate edge**), the **underside of the mounting plate** z, and the
  base **yaw**;
- the paper height at the four canvas corners.

**Expected**: `--check` prints a per-arm deviation table against the build
sheet. It **passes** when every arm is within ±10 mm in xy and z, the plates
are coplanar within 3 mm, and every yaw is within 1°.
**Report every deviation; do not re-centre the others to hide one.**

**The build sheet was re-issued 2026-09-10 at h = 970.0 mm** and supersedes the
850 and 940 issues. If you are holding a printout saying 850 or 2340, throw it
away. The bundle nonetheless carries artefacts at **both** 0.970 and 0.850,
because which one the steel is actually at is what the survey is for.

Also on the day, and neither is a planner command:
**read back the gripper width** after each pen is clamped (the GUI commands
0.0432 m at 70 N; libfranka reports a **lower bound** on the real gap) and
**record the collision profile** each arm is left in. The installation's
operator raises the thresholds after a MoveIt launch (40/40/36/36/32/28/24 Nm);
`fr3drivers`' default `sensitive` is half that. Which is in force is the
difference between "the pen touched down" and "the arm crashed".

### 3b. Touchdown log → fit → verdict

Prove the solver on synthetic data first — it needs no robot and no log:

```bash
python scripts/touchdown_calibrate.py --self-test
```

Then take **6–8 contacts on one arm, at different tool yaws and leans**, spread
across its reach — the pen is 86 mm across the hand, so a single touchdown
cannot separate lateral from axial. Log the **joint vector at contact**:

```json
{"touchdowns": [{"arm": 31, "q": [7 floats], "paper_z": 0.0, "label": "..."}]}
```

```bash
ARIS_RIG=proposed ARIS_TOOL=lateral python scripts/touchdown_calibrate.py \
    out/touchdown_arm31.json --json out/touchdown_arm31_fit.json
```

**Passes when** `cond(A) ≤ 20` (it refuses a fit whose rows do not span, rather
than printing three digits of noise), residual rms under ~1 mm, and `p_y ≈ 0`
— a `p_y` of any size means the **holder is clocked out of the jaw plane**,
which is a build finding, not a pen finding.

> **Read the fit's verdict on the paper-chain gate before doing anything else
> with it.** v18's thinnest margin in the whole programme is **20.5 mm against
> a 20 mm gate — 0.5 mm** of paper-chain clearance. A measured tip deeper than
> `0.0460262` by more than 0.5 mm **eats that gate** and the programme must be
> re-conducted before it is flown. The script computes this and **exits
> non-zero** on it.

A moved tip also stales every atlas in `out/`: `atlas.is_current` puts the pen
inside its model signature.

### 3c. Planning at the as-built height, arms 31 and 71 only

**The two-arm answer, measured — this is the question to read first.**

**There is no flag that makes an arm absent.** `--arms 31,71` exists on
`scripts/draw.py`, `csail_allocate.py` and `csail_schedule.py`, but it means
"these arms do the *drawing*" — the other four stay in the fleet, in the
collision world, and in the conducted timeline as parked metal.

**But for arms 31 and 71 it does not matter, and this was measured.** The
collision world is built per-spec from `spec.static_obstacles()`, which is
baked from the fleet dict at construction. Base-to-nearest-absent-box distance
is **1.003 m** for both 31 and 71; their only sub-metre neighbour is each other
at 0.61 m. Running `atlas.solve_cell` with the real gates over 150 random cells
in each arm's reach disc, six-arm static set versus two-arm static set:

```
arm 31:  6-arm GO 76   2-arm GO 76   newly-GO 0
arm 71:  6-arm GO 75   2-arm GO 75   newly-GO 0
```

**So: plan with the shipped six-arm fleet and use the shipped atlases. They are
valid, and not even conservative, for a 31/71-only rig. Do not re-sweep, and do
not subset anything.** The atlas format is one `.npz` per arm
(`atlas_arm31.npz`, `atlas_arm71.npz`), so a two-arm run reads two files out of
the same directory with no change.

The artefacts, already in `repo/out/` after §1.5:

| h | atlas (final `lateral` tool) | park search |
|---|---|---|
| **0.970** | `atlas_proposed_h0970_lat0860`, `…_gated63` | `park_search_h0970_lat0860.json`, `park_search_h0970_seam.json` |
| **0.850** | `atlas_proposed_h0850_lat0860` | **none at this tool** — see below |

> **The h = 0.850 gap, from `HARDWARE_LADDER.md` rung 0.** There is an atlas at
> 0.850 for the final tool but **no park search at it**.
> `park_search_h0850_lat0588.json` was searched at the **superseded 0.0588**
> tool. If the survey says 0.850, the park search must be re-run before any
> park pose is believed. It is not slow — see the regeneration note below.

If a sweep or park search *is* needed at a surveyed height:

```bash
ARIS_TOOL=lateral python scripts/height_sweep.py sweep \
    --h 0.850 --out out/atlas_proposed_h0850_lat0860_gated63 --jobs 6
ARIS_TOOL=lateral python scripts/height_sweep.py park \
    --h 0.850 --atlas out/atlas_proposed_h0850_lat0860_gated63 \
    --out out/park_search_h0850_lat0860.json --jobs 6
```

**Regeneration is cheap.** A logged six-arm sweep at 2 cm grid took **57 s
wall** at `--jobs 6` (~50 s per arm, run in parallel); the script's own
docstring says 8–25 min for heavier settings. Either way it is minutes, not
hours — the atlases are in the bundle for convenience, not because they are
expensive. Note `height_sweep.py` builds its **own** six-arm fleet internally,
so it ignores any fleet subsetting.

Then re-plan at that height:

```bash
ARIS_RIG=proposed ARIS_TOOL=lateral python scripts/replan_at_height.py \
    --h 0.850 --atlas out/atlas_proposed_h0850_lat0860_gated63 \
    --parks out/park_search_h0850_lat0860.json -- <draw.py args...>
```

`--check` on that script proves the height actually travelled into every module
(it walks `sys.modules` and aborts if one stale reference survives). Use it.

**If you do need the four absent** — no bodies, no columns — it is one function:
`aris_sixarm/layout.py::_proposed()` (around line 1687), which today just
returns `build_fleet(LAYOUT_PROPOSED, q_park=Q_PARK_PROPOSED)`. Have it read an
`ARIS_ARMS` env var, keep that subset, and rebuild each survivor's
`mount_boxes` with `mounts.obstacles_for(a, bare, h)` via `dataclasses.replace`.
That spot works because it runs inside `fleet.activate()`, which runs in
`aris_sixarm/__init__.py` before any script's import block, so every
`from .fleet import FLEET` binding sees two arms from the first instant — no
monkeypatching, no stale-reference audit. **Two cautions if you do it:** it
removes the absent arms' *booms and plates* too, which is wrong in the unsafe
direction if that steel is still bolted to the grid; and do not touch
`ARIS_SEAM_POSTS` — the seam bars are the room, not an arm. Given the 1.003 m
measurement above, **the recommendation for tomorrow is not to do it.**

`scripts/asbuilt_layout.py` *can* already express a two-arm rig — `build_asbuilt`
iterates the survey's own `arms` keys, and a two-arm survey yields 8 static
boxes for arm 31 instead of 32 — but `load_asbuilt()` has **zero callers**
anywhere in the repo. The script is report-only. It is not a planning path
today.

### 3d. Dry-run playback

See §4.

### 3e. Export — what exists and what does not

**This is the gap to raise with Pete before the day, not on it.**

| format | where | usable tomorrow? |
|---|---|---|
| `fr3_bundle` v1 **npz**, joint-space | `execute/backends.py::Fr3BundleBackend.export` — **committed, in the bundle** | **No.** Degree-1 Bézier ⇒ discontinuous velocity at every knot ⇒ not flyable in `position_velocity_accel`. For `fr3_sender.py --dry-run` and inspection. It also has **no CLI** — you must call it from python. |
| **impedance pathway CSV** | `aris_sixarm/export/pathway.py` — **UNTRACKED, NOT in the bundle** | **Not present.** |
| viewer bundle JSON | `program_schema.py::export_bundle`, `scripts/export_viewer_bundle.py` | Browser scrubber only, not robot-consumable. |

The pathway CSV is the format the installation's own executor
(`~/RTff/rtff_pathway_exec.py` on the operator box `192.168.50.2`) actually
consumes:

```
stroke_idx,wp_idx,kind,x_m,y_m,z_m,qx,qy,qz,qw,intensity
```

task-space, arm base frame, metres, `kind ∈ {travel, draw, lift}`, no time
column (the executor paces by arc length). The code that writes it exists on
the workstation but is **not committed**, so `git archive HEAD` — and therefore
this bundle — does not contain it.

**Two ways out, both needing a decision:** commit `aris_sixarm/export/` (plus
`docs/EXPORT_PATHWAY.md`, `tests/test_export_pathway.py`) and rebuild the
bundle; or hand-carry the already-generated
`out/pathways/csail_schedule_h094_v18_arm{31,71}.csv` to
`~/RTff/pathway_persist/` on the operator box. **That work belongs to another
session and was deliberately not committed here.** See §6.

There is also **no plan to export**: `out/` is gitignored, so no schedule npz
or program json ships. Plan on site, or carry the plan.

To produce an `fr3_bundle` npz at HEAD, from python:

```python
from aris_sixarm.execute import from_schedule, Fr3BundleBackend
prog = from_schedule("out/<name>_schedule.npz", "out/<name>_program.json")
Fr3BundleBackend.export(prog.track(31), "out/arm31_bundle.npz", speed_scale=0.37)
```

**Arm 71 has no IP** and never has had one.
`Fr3BundleBackend.preflight` refuses a programme with an arm it has no IP for,
which is how that surfaces rather than being discovered at launch. That is an
execution blocker independent of everything above.

---

## 4. The meshcat viewer on site — the dry run

Neither of these touches a robot; both were **not run in this verification**
(they need a display and a plan file, and no plan file ships).

**Executor rehearsal** — walks the actual execution adapter, barriers and all:

```bash
ARIS_RIG=proposed ARIS_TOOL=lateral python -m aris_sixarm.execute \
    out/<name>_schedule.npz out/<name>_program.json --play --rate 0.25
```

Useful flags: `--solo 31` (one arm moves, the others frozen — it prints the
frozen set and flags `recheck_required`, because the timeline was certified
with all six *moving*); `--fast` (no sleeping); `--check` (gate only, no
viewer). Requires the schedule npz (positional, required) and the program json
(optional, for labels/phases). **No atlas needed.** `ARIS_RIG`/`ARIS_TOOL` must
match the run that produced the schedule — the base transforms come off
`fleet.FLEET` in-process.

**Staged-programme animation**:

```bash
ARIS_RIG=proposed ARIS_TOOL=lateral python scripts/animate_staged.py \
    out/<name>_program_v6.json --meshcat --port 7006 --loop
```

Takes a **staged programme JSON**, not the schedule npz. `--port` matters: the
script runs its own ZMQ bridge and a bare meshcat `Visualizer()` grabs
7000–7005. `--hostname` defaults to the lab workstation — **override it on
site.**

---

## 5. Troubleshooting — every failure actually hit, with its fix

### 5.1 `scripts/demo_stroke.py` dies with `KeyError: 'qs'`

**It is broken at HEAD, in both configurations. This is not your install.**

```
stroke A_rim_arc: 248 steps, len 2.96 m
  DP  : ok=False cut_s=0.247 min_sigma=0.1708 ...
  -> SPLIT at s*=0.2470, ...
Traceback (most recent call last):
  File "scripts/demo_stroke.py", line 65, in <module>
    qs = dp["qs"]
KeyError: 'qs'
```

The demo's two hardcoded strokes no longer plan end to end at the current
rig/tool constants, and the reporting block at line 65 reads `dp["qs"]`
unconditionally — a key that only exists on a plan that did not split. Verified
broken both with `ARIS_RIG=proposed ARIS_TOOL=lateral` (stroke A splits at
s\*=0.247) and with no env vars at all (stroke A splits at s\*=0.000). The
README's description of it — "rim arc planned end to end where greedy dies at
s=0.015" — describes an older rig and tool.

Note the exit code is **0**, so a script checking `$?` will not notice.

**Use `smoke_plan.py` (§2.2) instead.** Fixing the demo is a planner change and
planner development is paused.

### 5.2 `ModuleNotFoundError: No module named 'franka_analytical_ik'`

```
File ".../aris_sixarm/ik.py", line 44, in <module>
    from franka_analytical_ik import _franka_ik as _IK
ModuleNotFoundError: No module named 'franka_analytical_ik'
```

The IK step (§1.3) did not happen, or happened in a different venv.

```bash
pip install ./third_party/franka_analytical_ik
```

and re-check with the one-liner in §1.3.

Since 2026-09-16 the message is longer than the traceback above — `ik.py`
raises its own `ImportError` naming that pip command, after trying the
installed package, `ARIS_FRANKA_IK_PATH`, and the old workstation path in that
order.

**This error used to hide itself on the workstation**: the hardcoded
workstation path was tried *first* and *does* exist there, so an import that
would fail on site succeeded locally — which is how a "clone acceptance test"
passed on 2026-09-15 for entirely the wrong reason. The order is now installed
package first (§1.3), and the solver is vendored, so both halves of that trap
are closed. Verification for this document is still done in a throwaway venv
against a fresh clone, never in the repo's own `.venv`.

### 5.3 `requirements-site.txt` missing from the bundle

Hit while verifying the bundle: `git archive HEAD` ships **committed files
only**, and the file had not been committed yet.

```
ERROR: Could not open requirements file: 'repo/requirements-site.txt'
```

**Commit before you bundle.** More generally: anything uncommitted is not in
the bundle — which is also the mechanism behind the missing pathway exporter in
§3e. If you edit something on the workstation and want it on site, commit it,
then re-run `scripts/make_site_bundle.sh`.

### 5.4 The wheel installs but will not import

Only reachable if you used `ik_wheel/*.whl` from a bundle built before
2026-09-16. It is tagged `cp312` and built against this workstation's glibc: a
different python minor version will refuse it at install time, an older glibc
will fail at import. **Build from `third_party/franka_analytical_ik` instead**
(§1.3) — five seconds, any version.

### 5.5 `fatal error: Eigen/Dense: No such file or directory`

`libeigen3-dev` is missing (§1.1), or Eigen is not at `/usr/include/eigen3`.
Install it, or
`EIGEN_INCLUDE=/path/to/eigen pip install ./third_party/franka_analytical_ik`.

### 5.6 `unrecognized arguments: --timeout`

`pytest-timeout` is not installed and is not a dependency. Drop the flag.

### 5.7 meshcat's pyzmq DeprecationWarning

```
zmq.eventloop.ioloop is deprecated in pyzmq 17
```

Harmless, appears in every test run, ignore it.

---

## 6. What NOT to do

1. **Do not set `ARIS_RIG` or `ARIS_TOOL` when running the tests.** §2.1 is
   `python -m pytest tests/test_gates.py tests/test_hardware_prep.py -q` with a
   clean environment. `test_gates.py` asserts the shipped constants at the
   default rig — the 2026-07-12 arm-31 touchdown, `MZ = 0.924`,
   `PEN_EXT = 0.110`. Setting the env vars makes it test a different rig and
   the result means nothing. Set them for *planning* commands (§3), where they
   are required.

2. **Do not edit gate constants to make something pass.** The strict-GO gates
   (joint margin ≥ 0.30 rad, σ_min ≥ 0.14), the validator's tolerances
   (tip-on-curve < 2 mm, margin ≥ 0.15, σ_min ≥ 0.10, ‖Δq‖∞ ≤ 0.35), the 80 mm
   inter-arm gate and the 20 mm paper-chain gate are field-validated numbers.
   σ_min is the binding one and it is not conservatism: libfranka zeroes the
   external-wrench estimate near singularities, so a low σ_min means the arm is
   **force-blind**. A gate that fails is information. Report it.

3. **Do not commit or ship the foreign untracked files.** At the time of
   writing the workstation tree carries `aris_sixarm/export/`,
   `aris_sixarm/sil/`, `docker/`, `docs/ARIS2_CONTRACTS.md`,
   `docs/EXPORT_PATHWAY.md`, `tests/test_export_pathway.py` and
   `tests/test_sil_*.py`, plus a modified `pyproject.toml` — all belonging to
   another session's in-progress work. `make_site_bundle.sh` ships
   `git archive HEAD` precisely so none of it travels by accident. If the
   pathway exporter is needed on site (§3e), that is a **decision for its
   author**, not something to sweep in.

4. **Do not re-sweep the atlas tomorrow.** The shipped `atlas_arm31.npz` /
   `atlas_arm71.npz` are valid for a 31/71-only rig (§3c, measured). Re-sweep
   only if the survey moves the height or the touchdown moves the tip — and in
   that case the tip change stales *every* atlas, so it is not optional.

5. **Do not trust a `--solo` run as certified.** `FleetProgram.solo()` returns
   the mover's track *and* the poses the other five must be frozen at, with
   `recheck_required = True`, because the timeline was certified with all six
   moving. A frozen fleet is a different scene.

6. **Do not treat the software gate as the abort path.** The gate brakes at
   2 rad/s² and the watchdog latches at 20 mrad, and the driver's own notes say
   in as many words that neither is the abort. **The physical e-stop is the
   abort.**

---

## 7. Open questions only Pete can answer

These block nothing in §1–§2 and everything in §3e.

1. **What is the site machine?** OS, python version, and whether it has
   network. If it is not Ubuntu 24.04 / python 3.12, the pinned
   `requirements-site.txt` becomes advisory — and the IK bindings do not care
   either way now that they are vendored as source (§1.3).
2. **Which control stack is the target tomorrow** — the Cartesian impedance
   executor on the operator box (wants the **pathway CSV**, which is not in the
   bundle), or `fr3drivers` (wants the **joint bundle npz**, which is in the
   bundle but is not flyable and has no CLI)? The answer decides whether
   `aris_sixarm/export/` must be committed tonight.
3. **What is the as-built height** — 0.970 (current build sheet) or 0.850 (what
   the steel was cut for before the re-issue)? Both are in the bundle. If
   0.850, a park search at the final tool must be re-run (§3c).
4. **How will contact be detected to better than a millimetre** for the
   touchdowns? `jog_descend`'s "human eye" and `impedance_pathway_exec`'s 3 mm
   lag threshold are both coarser than the 0.5 mm of paper-chain clearance at
   stake. `HARDWARE_LADDER.md` rung 1 calls this the open question that rung has
   to answer first.
5. **Does arm 71 get an IP?** It has never had one. Planning works without it;
   execution preflight refuses.
