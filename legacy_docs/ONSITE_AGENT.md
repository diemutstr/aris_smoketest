# On-site agent runbook — read this first, you have zero context

You are on a machine at the installation. Six Franka FR3 arms hang upside down
over a paper table; today only the two middle-row positions are used. **WHICH ARM
IDS ARE MOUNTED IN THEM IS NOT KNOWN — identify the arms first** (2026-09-17:
"may be 97 and 71, position unknown"). The PLAN is named after the POSITION:
slot 31 is the left-middle position, slot 71 the right-middle, and `send
--as-arm <id>` dispatches a slot's file to whatever arm is actually bolted
there. The job today: **write the word "unknown" under each arm, one arm at a time,
first floating 30 mm above the paper, then on the paper.** Nothing moves two arms
at once. Pete (the PI) is with you; he holds the physical e-stop, which is the
only abort that counts.

This repo is the PLANNER. It produces a CSV of waypoints per arm. A separate,
already-deployed executor on the "operator PC" turns that CSV into motion. You
never talk to a robot from this repo directly; you copy a file and start a script
over ssh.

---

## 0. Facts you must not get wrong

**Everything site-specific is `config/site.json`** — operator host, per-slot
physical arm id, control-box IP, DDS domain, measured paper-z, whether the
mounting is confirmed, and the `RTFF_*` environment the dispatch line carries.
Run `day1.py site` before anything else, edit it with `day1.py site --set
KEY=VALUE`, and never hand-edit an address into a script: there is none in
there to edit (`tests/test_day1.py` enforces that).


| fact | value |
|---|---|
| mounting height h | **0.970 m** from the paper top to the underside of the mounting plate |
| slot 31 / 71 base, canvas frame | x 0.5967 / 1.2067 m, y 1.8153 m (the seam line), both inverted, same clocking. These are POSITIONS; the CSV's poses are in the position's base frame and are right for whichever arm is mounted there. |
| which arm ids are mounted | **UNKNOWN. Identify first**, then record it: `day1.py site --set slot31.arm=97 --set slot31.mounted=true`. Nothing downstream checks this for you. |
| tool tip in the hand frame | (0.0860, 0, 0.1494) m — from a photograph, **not a touchdown**; the floating pass exists to check it |
| **everything site-specific** | **`config/site.json`** — operator host, each slot's physical arm id / control-box IP / DDS domain / measured paper-z / mounting confirmed, and the `RTFF_*` environment the dispatch line carries. `day1.py site` prints it, `day1.py site --set KEY=VALUE` edits it, every run prints it, and the GUI reads and writes the same file. **No address is hard-coded in any script** (a test enforces that). |
| operator PC | `diemut@192.168.50.2`, key-only ssh (`ssh host 'cmd'`, never `bash -lc`) — from `config/site.json` |
| control boxes | arm 31 → 192.168.50.12, arm 71 → 192.168.50.14, arm 97 → 192.168.50.15. The id is the DDS domain AND the `ARM_ID` the supervisor reads. |
| executor | `~/RTff/draw_rtff_supervised.sh` on the operator PC; it **ignores the joint columns and timestamps** in our CSV, paces by arc length at 0.02 m/s, and keeps its own arm configuration. Today's runs therefore test the tool, the plane and the pose path — not the planner's redundancy resolution. Say so if asked. |
| **START POSE** | **ROW 0 of the CSV — not the park.** The executor ramps in an uncertified straight line from wherever the arm is to row 0, and it cannot execute a joint transit (it walks rows as a Cartesian path with its own nullspace). So drive the arm to **row 0's joints** under POSITION control (`go_start_pos.py`) and the ramp is then zero. `day1.py send` prints those joints; `send --from-q <measured joints>` **refuses to dispatch** if the arm is more than 0.05 rad or 10 mm away (`--allow-ramp` overrides, deliberately). |
| **END POSE** | the **last row** of the CSV. `RTFF_DEPART_LIFT=0`, so the arm stops there rather than lifting 80 mm. `send` prints those joints too; return to the park under POSITION control afterwards. `day1.py park --arm N --from-q <end joints>` gives the certified path and the check that it is clear. |
| paper plane | no automatic gate for inverted arms, and `RTFF_CONTACT_DESCEND=0` means it does not feel for it either: the executor goes to the height baked into the file and the impedance spring holds contact. Measure it by hand first (operator tools `jog_descend.py` or `probe_surface.sh`). **`--measured-float H` is the one to use**: H is what the ruler read on the floating pass, and it computes the offset for you. (`--paper-z Z` is the same thing stated as how far the REAL paper sits ABOVE the modelled plane, Z = 0.030 − H; it lifts the whole plan by Z.) Record it with `day1.py site --set slot31.paper_z=<Z>` and it becomes that slot's default. |
| hover sign | `fr3_link0`'s +z points **down** on an inverted arm, so "30 mm above the paper" is `z_paper - 0.030` in the base frame. Verified in every file we write (`rows.tip_above_paper_base_m` in the json). |
| slot 31 quirk | it refuses a line exactly on the seam line y = 1.815 (go-home leg fails the paper gate); the word is placed at `--dy -0.10` by default. |
| the CSV | 19 columns: the v2 contract's 18, plus **`t_s` last** (the planner's pacing, same clock as the npz). A positional reader of the first 18 is unaffected. The executor ignores `q1..q7` and `t_s`. |
| one arm at a time | always. The other arm is at its park, which is what every certificate assumes. |

Do not edit gate constants, `frames.py`, `layout.py`, `mounts.py`, `scene_check.py`.
Do not run pytest with `ARIS_RIG`/`ARIS_TOOL` exported. Do not push to any remote.

---

## 1. Install and verify (10 minutes)

```bash
sudo apt install -y build-essential libeigen3-dev python3-venv
git clone -b aris2 git@github.com:wernerpe/aris_smoketest.git && cd aris_smoketest
python3 -m venv .venv && . .venv/bin/activate && pip install -U pip
pip install -r requirements-site.txt
pip install ./third_party/franka_analytical_ik      # ~5 s compile, vendored
pip install -e '.[gui]'
python -c "import aris_sixarm.ik as ik; print('batch:', ik._IK.has_batch)"   # True
python -m pytest tests/test_gates.py tests/test_hardware_prep.py -q          # 20 passed (own process!)
python -m pytest tests/test_day1.py -q
export ARIS_RIG=proposed ARIS_TOOL=lateral
```
If any of that fails: `docs/SITE_SETUP.md` §5 lists every failure seen so far.

Network check: `ssh diemut@192.168.50.2 'echo ok'` must print `ok`. If it asks
for a password, the key is not installed; stop and tell Pete.

---

## 2. The commands (all in `scripts/day1.py`, `--help` on each)

| command | what it does | output |
|---|---|---|
| `day1.py site` | prints `config/site.json` — operator, slots, arm ids, IPs, paper-z, dispatch env. **Read it first.** `--set KEY=VALUE` edits it | text |
| `day1.py park --arm 31` | the park joints and tip pose — where the programme begins and ends, and where to put the arm **after** a run. **Not** where a run starts | text |
| `day1.py park --arm 31 --from-q q1,…,q7` | certified joint path from the MEASURED joints to the park (RRT) — a check that the straight ramp is clear; the deployed executor cannot follow joint paths | `out/day1/park_31.*` |
| `day1.py word --arm 31 --hover` | the word under arm 31, floating 30 mm up; plans in ~20–50 s | `out/day1/unknown_hover_31.{csv,npz,json}` |
| `day1.py word --arm 31 [--measured-float H]` | the word on the paper. H = what the ruler read on the floating pass (the tool does the arithmetic; `--paper-z Z` is the raw form) | `out/day1/unknown_31.*` |
| `day1.py line --arm 31 --from x,y --to x,y --name N [--hover 0.03]` | one straight line, same pipeline, seconds | `out/day1/N_31.*` |
| `day1.py send --arm 31 --file out/day1/<file>.csv --dry-run` | PRINTS the START/END poses, the scp and the run line; copies nothing | — |
| `day1.py send … --from-q <measured joints>` | the same, and **refuses** unless the arm is already at row 0 (0.05 rad / 10 mm) | — |
| `day1.py send --arm 31 --file …` | copies the CSV to the operator PC and prints the run line | — |
| `day1.py send --arm 31 --as-arm 97 --file …` | the same, but dispatched to the PHYSICAL arm 97 standing in slot 31's position | — |
| `day1.py send … --live` | the same, and runs it over ssh | the arm moves |
| `python -m aris_sixarm.gui` | http://localhost:8765 — Day 1 panel does the above with buttons, a 3D viewer, and a "Run on arm" group with a typed confirmation | — |

**The GUI's "Run on arm" group** (left column, under Day 1) does §3 and §4 with
buttons and runs **the same `day1.py send` as a subprocess** — it re-implements
nothing. In order: **Check stack** (green only on `STACK HEALTHY`, raw output
shown), **Copy to operator** (`send --dry-run`: copies nothing, prints the run
line), **RUN (observe mode)** — *disabled until the stack check is green AND you
have typed `RUN <slot>` into the confirm box* — then **Hold** and **Kill
executor**. Every command is echoed verbatim before its output; the run's own
output streams into the log pane on the right as an ordinary job, and **Start log
tail** streams `tail -f /tmp/rtff_draw_arm<id>.log` off the operator PC into the
panel. Above them, **Site setup** edits `config/site.json` in place (host, per
slot: arm id / paper-z / mounting confirmed) and **Identify arms** polls ids
31/71/97 for live joints every few seconds, poses the ones that answer in the 3D
viewer with the tool model, and is how the slot → arm mapping gets filled in.
Every line prints `slot 31 → arm 97`. **The physical e-stop is still the abort;
nothing in the GUI is.**

Every planning command prints ONE line `PASS …` or `FAIL …` with the gate numbers
(inter-arm, frame, self, paper, joint speed as a fraction of the FR3 limit). A
FAIL writes **no CSV**. Never hand-edit a CSV.

---

## 3. What to check before an arm moves — every time

1. **Stack health:** `ssh diemut@192.168.50.2 'bash ~/RTff/aris_hold.sh stack 31'` → must contain `STACK HEALTHY`. This script was written for arms 13/17; if it rejects 31, show Pete the raw output.
2. **The arm is at ROW 0 of the file you are about to send** — not at the park. `day1.py send --arm 31 --file <csv> --dry-run` prints the START joints; move the arm there with the operator's `go_start_pos.py` under position control, then run `send … --from-q <measured joints>`, which refuses to dispatch unless the arm is within **0.05 rad on every joint and 10 mm at the tip**. The executor's opening ramp is then zero. Afterwards the arm stops at the END pose `send` printed; return it to the park under position control (`day1.py park --arm 31 --from-q <end joints>` is the certified check that the way back is clear).
3. **The paper height is measured** and baked (`word --measured-float H` with the ruler's reading, or `site --set slot31.paper_z=Z`) for on-paper runs; for the floating run the baked plane is the nominal one and the 30 mm is the check.
4. **The arm id is identified and recorded** (`site --set slot31.arm=<id> --set slot31.mounted=true`), and `send` is given `--as-arm <id>` if it differs from the slot.
4. **The other arm is at its park** and nobody is under the arm.
5. **Pete has the e-stop in hand.**
6. **Ask Pete to confirm which way the fingers and the pen holder are mounted.**
   This is still ambiguous: the holder can sit in the jaw either way round, and the
   model's tool tip (86 mm to one side of the hand) depends on it. Use the GUI's
   **Identify arms** view (reads each arm's live joints from the operator PC and
   poses the model with the tool drawn, refreshing every few seconds — move one
   by hand in guiding mode to see which id it is) and hold it next to the real
   hand: the pen must
   jut out on the same side and lean the same way. If it does not, STOP and tell
   Pete — every plan assumes the modelled side. Take a photo either way.

---

## 4. The run ladder, in order

For arm 31, then repeat everything for arm 71 (`--arm 71`, control box .14, domain 71):

```bash
day1.py word --arm 31 --hover                       # PASS line, tip ≈ +27..30 mm
day1.py send --arm 31 --file out/day1/unknown_hover_31.csv --dry-run   # read the START joints it prints
#   -> move the arm to those joints with the operator's go_start_pos.py (POSITION control)
day1.py send --arm 31 --file out/day1/unknown_hover_31.csv \
        --from-q <measured joints> --live                              # floating pass
```
`--from-q` is the gate: it refuses to dispatch unless the arm is already at row
0 (0.05 rad / 10 mm), because everything between there and row 0 is flown as an
uncertified straight ramp.
Watch: `ssh diemut@192.168.50.2 'tail -f /tmp/rtff_draw_arm31.log'`.
**Pass:** the pen traces the word in the air, about 27–30 mm above the paper by
eye/ruler everywhere, no reflex, no stop, and the arm **stops at the END pose**
`send` printed (`RTFF_DEPART_LIFT=0`) — return it to the park under position
control. **Measure the height with a ruler at three points and write it down:
it is the input to the next step.** If it is wrong by more than 5 mm the tool
transform is off; tell Pete before anything touches paper.

Then on paper:
```bash
day1.py word --arm 31 --measured-float <ruler reading, m>   # e.g. 0.027
day1.py send --arm 31 --file out/day1/unknown_31.csv --dry-run  # read the START joints
#   -> go_start_pos.py to those joints (POSITION control)
day1.py send --arm 31 --file out/day1/unknown_31.csv \
        --from-q <measured joints> --live
```
`--measured-float H` is the ruler's reading from the floating pass; it computes
the paper offset for you (`--paper-z` is the same thing stated raw). Record it:
`day1.py site --set slot31.paper_z=<Z>` makes it that slot's default.
**Pass:** a continuous "unknown", no gaps inside strokes, no gouging, arm returns
to park. The first passes run in `RTFF_MODE=observe` (open-loop depth); closed-
loop force is a later step.

Stop / hold at any time (besides the e-stop):
```bash
ssh diemut@192.168.50.2 'bash ~/RTff/aris_hold.sh hold 31'      # keeps the checkpoint
ssh diemut@192.168.50.2 'ARM_ID=31 source ~/impedance_helpers/arm_env.sh && arm_pkill rtff_pathway_exec'
```

---

## 5. What to record and report back

- The PASS lines of every planning command (copy them verbatim).
- The observed floating height per arm, and the measured paper height you baked.
- Whether `aris_hold.sh stack 31/71` accepted the arm, and the raw output if not.
- The operator log `/tmp/rtff_draw_arm<N>.log` and the force logs in
  `~/RTff/force_logs/` (copy them back: `scp diemut@192.168.50.2:/tmp/rtff_draw_arm31.log out/day1/`).
- A photo of each drawn word.
- Anything the arm did that the 3D viewer did not show.

---

## 6. If something is off

| symptom | what it is | do |
|---|---|---|
| `FAIL … inter-arm` or `frame` | the plan cannot be certified with the other arm parked / against the steel | move the word (`--dy`, `--width`), never lower a gate |
| `FAIL … joint speed` | the pacing exceeds an FR3 joint limit | report it; do not run |
| word planned but 0 strokes for one arm | reach/height problem; at 0.940 arm 31 draws nothing | check h really is 0.970 |
| executor stops during a pen-up move | the deployed controller reconfiguring the arm under its own nullspace | hold, note the row from the log, tell Pete |
| floating height off by > 5 mm | tool transform wrong | stop; Pete decides on a touchdown calibration |
| `aris_hold.sh` rejects arm 31 | the health script only knows 13/17 | show Pete; the operator's own bring-up for inverted arms is Diemut's |

More detail, with every number and its provenance: `docs/HARDWARE_DAY1.md`,
`docs/SITE_SETUP.md`, `docs/SYSTEM_MODEL.md`.
