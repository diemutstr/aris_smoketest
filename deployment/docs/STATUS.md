# Status and hand-over

Written 2026-09-29, evening, at the end of the first build session. Read this first in a new
session, then `DESIGN.md`, `BUILD.md`, `AGENT_BRIEF.md` and the pages under `modules/`.

Everything is committed locally on branch `aris2` of `/home/franka/aris_project/aris_sixarm`
(last commit of the session: the snapshot "deployment round 2: local planner ..."). Nothing of
`deployment/` has been pushed. 145 quick tests pass (`cd deployment && ../.venv/bin/python -m
pytest tests -q -m "not slow"`, under a minute).

## Where each module stands

| module | state | measured |
|---|---|---|
| `aris/types.py` | the contracts; changed by the orchestrator only | |
| `rig` | done | poses match the old model to 1e-16; 37 steel boxes; diagonal walls 678 mm from both bases, 26.75 degrees |
| `kernel/arm` + `native/fr3_ik` | done | new IK: round trip 100 % (old solver 66 %); body 62 capsules, all meshes inside; holder 14.7 mm above the paper upright, 9.4 mm at 15 degrees lean |
| `kernel/collide` + `native/collide` | done; two-level check IN PROGRESS | exact distances; real arm 31 scene 22 000 configurations per second on one thread since the tool refit (85 000 before it); target 100 000 |
| `kernel/retime` + `native/retime` | done | free path 30 waypoints 5 ms; drawing 250 samples 10 ms; limits hold at any sampling rate; pen within 0.1 mm |
| `check` | done | agrees with the kernel to 1e-15 on 60 000 configurations; 14 built-in faults caught; 0.2 s for a 7 s motion |
| `free` | works; speed work IN PROGRESS | 2 000 of 2 000 test pairs solved; 141 / 207 ms median per plan, 31 / 46 ms when the straight move works; target under 100 ms |
| `local` | works; measurements and speed work IN PROGRESS | arm 31, paper only: random lines and curves 100 %, corpus 98 %, word 93 % (the rest is beyond the 0.784 m reach); about 9 in 10 random lines without a lift; 0.7 to 2.8 s of CPU per line |
| `sequencer`, `arm_planner` | not started (round 3) | |
| `system` | not started (round 4) | |
| `execute`, `server`, `cli` | not started (round 5) | |
| `calib` | not started (round 6); Pete builds the hardware (dimple plates, pin) | |
| `gui` | not started (round 7) | |

## Local planner against the old planner (arm 31, measured 2026-09-29)

Share of line length drawn. "Paper only" is the fair comparison, because the old planner knows
no walls. Old planner: `stroke_api.plan_stroke`, tilt 15, the rest of a line re-offered after
every split.

| set | length | new, real obstacles | new, paper only | old |
|---|---|---|---|---|
| word "unknown" | 2.59 m | 0.929 | 0.929 | 0.679 |
| corpus, within 0.80 m | 33.75 m | 0.889 | 0.982 | 0.733 |
| random lines | 90.0 m | 0.951 | 1.000 | 0.932 |
| random curves | 60.7 m | 0.822 | 1.000 | 0.857 |

- With only the paper, the new planner draws at least as much as the old one on EVERY line, and
  more on 124 of them. What it does not draw lies beyond the arm's reach of 0.784 m.
- With the real obstacles, what it draws less than the old planner is, to the centimetre, what
  it reports as blocked by the two phase walls.
- Lines drawn without a lift, paper only: word 92 %, corpus 51 %, lines 92 %, curves 96 %.
- CPU per line, median (loaded machine): new 0.4 to 1.8 s with real obstacles, 0.2 to 0.6 s with
  only the paper; old 0.3 to 7 s, with a 95th percentile of 120 to 390 s and a worst case of
  1 677 s. The new planner's 95th percentile is 1 to 6 s.
- The kinematic table agrees with the live solve within 0.5 % of the share drawn (0.52 % on the
  word: 13 mm at the rim) and cuts the graph time by about 30 %. What remains is the obstacle
  check on every surviving node: 73 to 78 % of the CPU is the graph.

## Work that was running when the session ended

Agents belong to the session that started them; assume they stopped with it. Their files are on
disk and in the snapshot commit. Restart these three tasks from the descriptions below.

1. **Local planner** (`aris/local/`). To do, in order:
   - run the fixed set for both arms (real obstacles, paper only, with the table) on frozen code:
     `tests/local_cases.py`; fill the floors in `tests/test_local.py` (only one is real, the rest
     are placeholders) and the "Measured" section of `docs/modules/local.md`
   - table against live solve: share drawn must agree within 0.5 % (on 43 lines it did: 0.9702
     against 0.9709, CPU 0.76 against 1.07 s per line)
   - the comparison with the old planner: `tests/data/local_reference.npz` exists now
   - then the lazy obstacle check (approved): obstacle check only on the winning route, a band
     around it and the alternatives; re-search when a node fails; fall back to the full check
     after a cap on rounds; accept only if shares agree within 0.5 %
   - delete the module's own tool-to-paper rule (the kernel's rule is in use) and the
     corner-to-corner timing workaround (retime handles a whole drawing in one call; use
     `tip_budget_m` with `tip_of=arm.tip`)
   - target: tens of milliseconds of CPU per line at the median
2. **Free-space planner** (`aris/free/`): switch every edge check to
   `kernel.collide.edges_clearance_q` (a compiled batch of edges), cut the Python bookkeeping;
   target under 100 ms median when the tree is needed, under 20 ms for the straight move; report
   the table of `docs/modules/free.md` before and after. The files in the snapshot are mid-change:
   run `tests/test_free.py` first.
3. **Collision, two-level check** (`aris/kernel/collide*.py`, `native/collide/`): bounds per link
   and per parked arm first, exact capsules only for what is close; same answers to 1e-12;
   target 100 000 configurations per second on the real arm 31 scene, one thread.

## Decisions waiting for Pete

1. How close the pen holder may come to the paper (`rig.json`, `clearances.tool_to_paper_m`, now
   0.0 = must not touch). It is also the limit on pen wear. Holder height: 14.7 mm upright,
   9.4 mm at 15 degrees lean.
2. At the rig: how far the hanging struts reach below the mounting plate (35 mm in the old model;
   12 mm more would break the required clearance of link 1), and which side of each arm the
   240 mm strut gap is on (`strut_wide_side` per arm in `rig.json`, assumed toward -x).

## Decisions taken by the orchestrator that Pete has been told about

- planning allowance on steel lowered from 0.013 to 0.003 (the old number paid for the old
  checker's sampling error); required clearances unchanged
- the arm's base capsules are fixed and are not checked against obstacles; the arm's own struts
  are obstacles for the moving links
- every collision verdict that counts is taken on the timed trajectory ("what is checked is what
  is flown")

## Things that cost time in this session

- The machine ran at a load of 35 to 77 on 32 cores with up to six agents testing at once. Speed
  numbers are CPU time and still swing by up to a factor of two. Re-measure the speed tables on
  a quiet machine before quoting them.
- Agents changing the kernel while planners were measuring on it forced two restarts of the
  local planner's runs. In the next rounds, freeze the kernel while a planner is being measured.
- The scratchpad of the old session contains a `profile.py` that shadows the standard library;
  scripts run from there cannot use cProfile.
