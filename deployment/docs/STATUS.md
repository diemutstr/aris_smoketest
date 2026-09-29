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
| `free` | done; target met on a quiet machine | 2 000 of 2 000 test pairs solved; CPU per plan at machine load 8 (arm 13 / arm 31): tree needed 48 / 75 ms median, 118 / 177 ms at the 95th percentile; straight move 9 / 12 ms; all plans 37 / 53 ms. At load 58 the tree case is 106 / 191 ms |
| `local` | works; measurements and speed work IN PROGRESS | arm 31, paper only: random lines and curves 100 %, corpus 98 %, word 93 % (the rest is beyond the 0.784 m reach); about 9 in 10 random lines without a lift; 0.7 to 2.8 s of CPU per line |
| `sequencer`, `arm_planner` | not started (round 3) | |
| `system` | not started (round 4) | |
| `execute`, `server`, `cli` | not started (round 5) | |
| `calib` | not started (round 6); Pete builds the hardware (dimple plates, pin) | |
| `gui` | not started (round 7) | |

## Local planner against the old planner (both arms, measured 2026-09-29)

Share of line length drawn. "Paper only" is the fair comparison, because the old planner knows
no walls. Old planner: `stroke_api.plan_stroke`, tilt 15, the rest of a line re-offered after
every split. CPU per line is the median on a loaded machine.

| arm, set | new, real obstacles | new, paper only | old | CPU s/line new (live / table) | CPU s/line old |
|---|---|---|---|---|---|
| 31 word | 0.929 | 0.929 | 0.679 | 0.43 / 0.35 | 0.3 |
| 31 corpus | 0.889 | 0.982 | 0.733 | 1.61 / 1.10 | 7.1 |
| 31 lines | 0.951 | 1.000 | 0.932 | 1.25 / 0.95 | 1.4 |
| 31 curves | 0.822 | 1.000 | 0.857 | 1.77 / 1.38 | 4.6 |
| 13 word | 0.926 | 0.929 | 0.679 | 0.37 / 0.32 | 1.7 |
| 13 corpus | 0.951 | 0.991 | 0.947 | 1.61 / 1.07 | 4.4 |
| 13 lines | 0.955 | 1.000 | 0.929 | 0.77 / 0.50 | 1.5 |
| 13 curves | 0.877 | 1.000 | 0.848 | 1.00 / 0.73 | 1.4 |

- With only the paper, the new planner draws at least as much as the old one on EVERY line, and
  more on 246 of the 765. What it does not draw lies beyond the arm's reach (0.784 m).
- With the real obstacles, every line where it draws less is covered by a stretch it reports as
  blocked by a wall or a parked arm.
- The old planner's slowest lines: 80 to 390 s at the 95th percentile, 1 677 s worst.
- The kinematic table (181 MB, built in 14 s) agrees with the live solve within 0.5 % of the
  share drawn except on the word (0.52 % and 0.78 %: 13 and 20 mm at the rim), cuts the IK solves
  per line from 8 000-26 000 to 450-1 800 and the CPU per line by 20 to 35 %.
- Where the CPU goes: graph 73-78 % (now mostly the obstacle check on every surviving node),
  exact-path check 12-17 %, search 4-6 %, timing 1-4 %.
- 795 pieces: 78 % have four alternative plans, 95 % at least two.

## Work that was running when the session ended

Agents belong to the session that started them; assume they stopped with it. Their files are on
disk and in the snapshot commit. Restart these three tasks from the descriptions below.

1. **Local planner** (`aris/local/`). To do, in order:
   - DONE: fixed set for both arms, floors in the tests, "Measured" section of the module page,
     table against live, comparison with the old planner (see the table above)
   - BUILT AND MEASURED, NOT WORTH IT RIGHT NOW: the lazy obstacle check (setting `lazy`). On
     arm 31 with the table it draws exactly what the full check draws (shares, pieces and
     leftovers identical), but on the same kernel it is 10 to 50 % SLOWER at the median: it
     checks 4 to 5 times fewer nodes and pays for it with many more searches along the walls
     (up to 66 per line). An earlier note that it halved the time compared it with a run on the
     slower kernel; that was wrong. Make `lazy=False` the default (it was still True in
     `settings.py` at the end of the session); keep the setting.
   - CURRENT SPEED, full check with the table, arm 31, CPU per line at the median: word 0.14 s,
     corpus 0.39 s, lines 0.30 s, curves 0.40 s (95th percentile 0.3 to 1.9 s). Mean split:
     graph 0.07-0.39 s, search 0.02-0.12, exact-path check 0.04-0.10, timing 0.02-0.03.
     Target is tens of milliseconds. Next levers: the graph (obstacle check of the surviving
     nodes; fewer nodes), and the repeated full-line search after a failed exact path.
   - delete the module's own tool-to-paper rule (the kernel's rule is in use) and the
     corner-to-corner timing workaround (retime handles a whole drawing in one call; use
     `tip_budget_m` with `tip_of=arm.tip`)
   - target: tens of milliseconds of CPU per line at the median
2. **Free-space planner** (`aris/free/`): DONE and committed. Every edge check goes through the
   compiled batch of edges. 94 % of what remains is inside the kernel's edge call. A refusal
   "no free path" costs the full cap of 20 000 edges, 10 to 30 s. The fixed test set still comes
   from random lift-off configurations; regenerate it from the local planner's alternatives.
3. **Collision, two-level check** (`aris/kernel/collide*.py`, `native/collide/`): bounds per link
   and per parked arm first, exact capsules only for what is close; same answers to 1e-12;
   target 100 000 configurations per second on the real arm 31 scene, one thread.
   The code is in the snapshot commit and the tests pass, but its owner had not reported or
   measured when the session ended: measure it first.
   Also asked for by the free-space planner: a tighter bound on how far the arm moves along an
   edge. The kernel charges each joint its largest possible lever arm and needs about 55 body
   evaluations per edge; a bound from the joints' actual speeds at the two ends of the edge plus
   a reach-based change term needed 17 to 20 (measured by the free-space planner's owner before
   that code was deleted; see git history of `aris/free/bound.py`). That is a 2.5 to 3 times
   cut for both planners.

## If the tests fail right after picking up

`ValueError: scene must have 13 arrays` (or another count) means the compiled collision module
that is installed and the Python side in the working tree are out of step: the two-level check
was mid-change when the session ended. Either finish that task, or go back to the last
consistent state:

```
cd /home/franka/aris_project/aris_sixarm
git status --short deployment            # see what the agents left uncommitted
git stash push -- deployment/aris/kernel deployment/native/collide deployment/tests/test_kernel_collide*.py
.venv/bin/pip install ./deployment/native/collide ./deployment/native/fr3_ik ./deployment/native/retime
cd deployment && ../.venv/bin/python -m pytest tests -q -m "not slow"
```

The same three `pip install` lines are what a fresh clone needs before anything runs.

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
