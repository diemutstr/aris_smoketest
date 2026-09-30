# Status and hand-over

Written 2026-09-29, evening, at the end of the first build session. Read this first in a new
session, then `DESIGN.md`, `BUILD.md`, `AGENT_BRIEF.md` and the pages under `modules/`.

BRANCHES (2026-09-30): the clean branch is `aris3` (worktree `/home/franka/aris_project/aris3`),
with the old code removed at Pete's request; `aris2` (worktree `/home/franka/aris_project/aris_sixarm`)
keeps the old code and is where the first session's agents worked. Both are pushed to
`git@github.com:wernerpe/aris_smoketest.git`. Work on `aris3` from now on.

Everything is committed on branch `aris2` of `/home/franka/aris_project/aris_sixarm`
(last commit of the session: the snapshot "deployment round 2: local planner ..."). Nothing of
`deployment/` has been pushed. 145 quick tests pass (`.venv/bin/python -m
pytest tests -q -m "not slow"`, under a minute).

## Where each module stands (updated 2026-10-01 early, HEAD on aris3)

| module | state | measured |
|---|---|---|
| `aris/types.py` | the contracts; changed by the orchestrator only | |
| `rig` | done; struts from the technical drawing; gates and rules in rig.json | drawing area 1.56 x 3.56 m |
| `kernel/arm` + `native/fr3_ik` | done | new IK round trip 100 %; body 62 capsules; reach at the paper 0.805 m geometric, sigma gate 0.04 |
| `kernel/collide` + `native/collide` | done, incl. two-level check, exemptions, distance fields (footprints) | real arm 31 scene 244 000 configurations/s on one thread |
| `kernel/retime` + `native/retime` | done; two models: corners (free paths) and smooth (draw, lower, lift: a C2 spline through the IK samples, cut only at real pen corners) | free path 30 waypoints 5 ms; drawing 250 samples 10 ms; the arc that crawled at 1.7 mm/s now 14.8 mm/s and 5.5 s instead of 7.6 s |
| `check` | done; reads rig.json itself; footprints with its own lookup | agrees with the kernel to 1e-15; 0.2 s for a 7 s motion |
| `free` | done | 2 000 of 2 000 pairs; 48 / 75 ms median when a search is needed (quiet machine) |
| `local` | done | paper only: 100 % of random lines and curves within reach; 0.14 to 0.40 s per line with the table |
| `sequencer` + `arm_planner` | done; two-rule lift; the checker runs inside the tour (`verify`: a piece is drawn only if its whole group passes); lines planned in batches of 32 nearest first, refill 128; flown check 4x finer | 1 533 of 1 533 motions pass, 0 refusals; slowest mid-line speed 5 mm/s except real corners; first motion 1-2 s on the word, 4-5 s on 1 000 lines (was 20 s); verify costs 0.1-0.26 s per motion |
| `system` | done: five laws, leaders first, fill in pairs, drawing area, followers built but OFF; binds `verify` per arm and phase (picklable), failed_check flows to later phases | seven whole-table drawings 100 % drawn, 2 906 of 2 906 motions pass the checker |
| `execute` | done: queue per arm, executor, coordinator, event log, simulated arm | word on six simulated arms: queues on disk bit-identical to the plan |
| `server` + `cli` | done: aris serve / draw / status / stop / park / rig / plan / check; the operator PC's four endpoints (`--driver robot`); no check pool (the planners check; `refused/` keeps what was refused); PASS = ran to the end with every motion checked; park from the runner's reported positions, pens lifted first | 10 000 lines (1 912 m): first motion 4-11 s, 30 min wall, 6 700 CPU-s, 10 GB, 36 processes, 42 304 motions all checked, 65 mm unreachable; 2 000 long lines (2 680 m) all drawn |
| `calib` | not started; Pete builds the hardware | |
| `gui` | not started | |
| `robot/` (operator PC) | written here, not yet run under ROS: joint impedance controller soft along the paper normal (100 N/m exact, plane unchanged, force servo on), driver, force logic, bringup, runner reporting every arm's joints | 44 tests here |

Quick test set: 229 tests, all pass in 87 s at machine load 6 (`.venv/bin/python -m pytest tests -q -m "not slow"`); the nine end-to-end tests over 4.5 s each are marked slow (2026-10-01). The slow set (58 tests) takes about 10 minutes.

## Decided with Pete, 2026-09-30 evening

- Execution path (DESIGN.md 4b): the arm tracks the certified joint trajectory under joint
  impedance, soft along the paper normal, with the pen force fed forward; free moves through the
  joint trajectory controller or the same controller with zero force.
- Drawing input: JSON polylines in mm in the table frame, one pen; it must scale to very big
  drawings (a 10 000-line test is being measured).
- Followers stay off to start. The lift is two rules (done).
- One repository, two roles: `robot/` runs on the operator PC.

## Done 2026-09-30 night / 2026-10-01 early

- README.md is the usage guide (two computers, install, a day as commands, the file, where things
  end up, what is not built).
- Scale: 10 000 lines planned end to end. Findings and fixes: the retimer crawled at smooth
  turning points (smooth mode); one checker refusal dropped 13 % of the ink (the checker now runs
  inside the arm planner's loop, DESIGN.md 4); first motion after 87 s (batches nearest first);
  the server re-parsed a 140 MB queue every 20 ms while streaming (in-band end detection).
- Operator side: endpoints, runner reports positions, park from them with pens lifted first.

## Next

- The serial check doubles an arm's planning wall time at scale (30 min for 10 000 lines,
  still five times ahead of the arms); levers if it matters: check a group's four motions in
  parallel, a cheaper checker (0.45 CPU-s per motion), and the 30 idle local-planner
  processes per phase (10 GB).
- The local planner's spin / joint-7 curves have knees at layer nodes (the retimer's remaining
  dips, e.g. 14.8 mm/s on the arc): smooth them there.
- On the operator PC: build `robot/ros2_ws`, fake-hardware run, then one real arm.
- Calibration software (round 6), GUI (round 7), followers' balance law (later).

## Decisions waiting for Pete (older; 1 to 3 are answered above)

1. Execution path for drawing on the real arms: (a) position control through the joint trajectory
   controller (what is certified is what is flown; no compliance); (b) the impedance controller as
   today (follows the tip pose, picks its own arm shape: what is flown is not what was certified);
   (c) the impedance controller with the joint reference as its nullspace target (controller work
   on the operator PC).
2. Drawing input: assumed a JSON list of polylines in mm in the table frame, one pen, scaled to
   fit the drawing area. SVG, several pens, tone per line are not built.
3. Interruptions: assumed "a stopped job is finished; what is left is a new drawing".
4. Followers: under the five laws they draw nothing. The law that would make them useful: split
   a row's lines between leader and follower to balance the work before the leader plans.
5. The lift: reduce the escape ladder to two rules (rise pen clearance + 2 mm on the same shape,
   else trim 1 cm), with the pen clearance 20 mm until calibration is proven.

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
     slower kernel; that was wrong. `lazy=False` is the default and the setting is kept.
     Confirmed on both arms: identical drawing in all 16 comparisons.
   - CURRENT SPEED, full check with the table, CPU per line at the median (arm 31 / arm 13):
     word 0.14 / 0.14 s, corpus 0.39 / 0.40, lines 0.30 / 0.22, curves 0.40 / 0.29. Mean split:
     graph 0.06-0.39 s, search 0.02-0.12, exact-path check 0.03-0.11, timing 0.02-0.03.
     Target is tens of milliseconds. Next levers, none built: fewer whole-line searches after a
     failed exact path; a cheaper way to open the lean (it multiplies the nodes by 13); sharing
     work between lines.
   - the table moves the share drawn IN ONE PIECE by up to 2.3 % (it blends between tabulated
     distances from the axis, which can move a lift); the share drawn agrees within 0.2 %
     except on the word (0.5 to 0.8 %). Decide whether that is acceptable or refine the table.
   - run the slow fixed-set test once (`pytest tests/test_local.py -m slow`); it has not been
     run since the last changes
   - delete the module's own tool-to-paper rule (the kernel's rule is in use) and the
     corner-to-corner timing workaround (retime handles a whole drawing in one call; use
     `tip_budget_m` with `tip_of=arm.tip`)
   - target: tens of milliseconds of CPU per line at the median
2. **Free-space planner** (`aris/free/`): DONE and committed. Every edge check goes through the
   compiled batch of edges. 94 % of what remains is inside the kernel's edge call. A refusal
   "no free path" costs the full cap of 20 000 edges, 10 to 30 s. The fixed test set still comes
   from random lift-off configurations; regenerate it from the local planner's alternatives.
3. **Collision kernel** (`aris/kernel/collide*.py`, `native/collide/`): the two-level check is
   DONE and committed (groups per link and per parked arm; same answers to 1e-12; path and edge
   bounds moved by at most 0.35 mm, inside their tolerance). Still open, asked for by the
   free-space planner: a tighter bound on how far the arm moves along an edge. The kernel
   charges each joint its largest possible lever arm and needs about 55 body evaluations per
   edge; a bound from the joints' actual speeds at the two ends of the edge plus a reach-based
   change term needed 17 to 20 (see the git history of `aris/free/bound.py`). Both planners
   would gain from it.
   After this kernel change, re-measure both planners: the local planner's and the free-space
   planner's speed tables were taken before the two-level check was finished.

## If the tests fail right after picking up

`ValueError: scene must have 13 arrays` (or another count) means the compiled collision module
that is installed and the Python side in the working tree are out of step: the two-level check
was mid-change when the session ended. Either finish that task, or go back to the last
consistent state:

```
cd /home/franka/aris_project/aris_sixarm
git status --short deployment            # see what the agents left uncommitted
git stash push -- aris/kernel native/collide tests/test_kernel_collide*.py
.venv/bin/pip install ./native/collide ./native/fr3_ik ./native/retime
.venv/bin/python -m pytest tests -q -m "not slow"
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
