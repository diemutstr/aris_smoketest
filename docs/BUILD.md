# How this gets built

Agreed direction (Pete, 2026-09-29): clean rebuild, built by Opus agents, every module small
enough to understand, very clean boundaries. `DESIGN.md` says what the system is. This file says
how the code is laid out and in which order it is built.

## Layout: one folder per box of the design

```
pyproject.toml
config/
  rig.json              the table, the canvas, where the arms hang, the frame, the margins
  calibration/          one dated file per arm, written only by the calibration job
aris/
  types.py              the shared data types; the only thing every module imports
  rig.py                reads config, turns table-frame things into one arm's base frame
  kernel/
    arm.py              the arm: kinematics, IK, limits, collision body
    collide.py          distance between the arm's body and obstacles
    retime.py           joint path -> timed trajectory inside the limits
  local/                local planner
  free/                 free-space planner
  sequencer/            sequencer
  arm_planner.py        sequencer + local + free = the arm planner
  system/               system planner
  check/                independent checker
  execute/              queue, executor, arm drivers (simulated, real)
  server/               drawing server
  calib/                calibration job
  gui/
  cli.py                the one command: `aris ...`
native/                 compiled code: fr3_ik (the IK), collide (the collision check), retime
tests/                  mirrors the folders above
docs/                   design, build plan, status, one page per module
assets/                 the installation model and meshes, the vendor arm description, drawings
legacy_docs/            the old planner's documentation (its code is on branch aris2 only)
```

## What may import what

```
types  <-  kernel, rig  <-  local, free  <-  sequencer, arm_planner  <-  system  <-  server
                                                                         check  <-  server
                                                                         execute <- server
```

- An arrow only ever points down the hierarchy. A test enforces it.
- `execute` may import `check` (to run the checker at the queue and at a phase end) and
  `kernel.retime` (the simulated arm samples trajectories); nothing above them.
- `check` imports `types` and nothing else from this package. It has its own kinematics and its
  own distance code, so that it and the planners can only agree by being right.
- Only `execute/drivers/` may import ROS. Only `rig.py` reads `config/`.
- The old package `aris_sixarm` is a reference. It is never imported by `aris`. Tests may import
  it to compare answers.

## The contracts between modules

Types are in `aris/types.py`. The calls:

| module | call | returns |
|---|---|---|
| kernel.arm | `Arm(tool)`, then `limits`, `self_pairs`, `reach`, `fk(Q)`, `tip(Q)`, `body(Q)`, `ik(T_base_hand, q7)`, `hand_pose(tip, normal, spin, lean)`, `sigma_min(Q)`, `limit_margin(Q)` | arrays, `Body`, `Limits` |
| kernel.collide | from capsules: `clearance(body, obstacles, drawing=False)`, `clearance_detail(...)`, `self_clearance(body, pairs, margin)`, `path_clearance(arm.body, q, obstacles, arm.reach)`. From joint angles, in one compiled call: `pack(obstacles)`, `arm_tables(arm)`, then `clearance_q`, `clearance_detail_q`, `self_clearance_q`, `path_clearance_q`. `backend()` says whether the compiled engine is in use | metres beyond the demanded margin; at least 0 means free. `path_clearance` is a lower bound that also covers the motion between samples |
| kernel.retime | `retime(path, limits, rules, s=None)` (with `s`, the arc length per sample, for drawing motions), `sample(traj, t)`, `check(traj, limits)` | `Trajectory` or `Refusal`, arrays, a report |
| rig | `Rig.load(config_dir)`, `to_base(arm_id, line)`, `obstacles(arm_id, parked, walls)`, `wall_between(a, b)`, `arm(arm_id)` | `Line`, `Obstacles`, `Plane`, `Arm` |
| local | `plan(arm, lines, obstacles, rules, gates=None, workers=1, settings=None)`; `plan_detailed` also returns counts and times; `verify_plan`, `reverse_plan` | `list[Bunch]`, `list[Leftover]` |
| free | `plan(arm, q_start, q_goal, obstacles, rules, gates=None, seed_extra=b"", options=None)`; `plan_detailed` also returns counts and times | a free `Motion` (timed, and checked as flown) or a `Refusal` with reason `outside_limits`, `blocked`, `self_collision`, `no_free_path`, `cannot_time` or `bad_input` |
| sequencer | `tour(arm, bunches, q_start, obstacles, rules)` | iterator of `Motion`, then `list[Leftover]` |
| arm_planner | `plan(arm, lines, obstacles, q_start, rules)` | iterator of `Motion`, then `list[Leftover]` |
| system | `plan(rig, drawing, arm_configs)` | iterator of `(phase, arm_id, Motion)`, then `list[Leftover]` |
| check | `check(config_dir, arm_id, motion, phase, q_before)` | a verdict: pass or fail, every measured number, and the tightest one |

## What is checked is what is flown

Timing rounds the corners of a path, so the path that is flown is not exactly the path that was
searched. Every collision verdict that counts, in the planners and in the checker, is taken on
the timed trajectory, not on the path it was made from.

## Rules for the code

1. One module, one job, one page in `docs/modules/<module>.md` that says: job, input, output,
   how it works, limits. Written so that Pete can read it without reading the code.
2. Small. A file over 400 lines or a function over 60 lines needs a reason.
3. No global state, no environment variables, no hidden caches. Everything a function needs is
   an argument.
4. Same input, same output. Random searches take their seed from their input.
5. Every call returns what it did and what it could not do, with a reason. Nothing is dropped
   silently; nothing raises for an ordinary refusal.
6. Numbers measured on the rig live in `config/` with a note saying where they came from.
7. Every module ships tests that run in seconds without hardware.
8. Simple version first. Speed-ups go to `OPTIMIZATION_NOTES.md` until a measurement asks for them.

## Build order

Each round runs several agents in parallel, one per folder. A round starts when the round
before it has passed its tests and been reviewed.

| round | modules | passes when |
|---|---|---|
| 1 foundation | rig, kernel.arm, kernel.collide, kernel.retime | kinematics and distances agree with the old code on random configurations; timing stays inside the limits when sampled at 1 kHz; speed of the collision check is reported |
| 2 planners | local, free, check | local planner draws the corpus lines the old planner could draw, with timings; free-space planner solves the fixed test set of 1 000 pairs; checker agrees with the old checker on old plans |
| 3 one arm | sequencer, arm_planner | the word "unknown" for one arm plans end to end, passes the checker, and the time is reported against the old 67 s |
| 4 six arms | system | corpus drawings: what each phase draws, what is left and why; nothing unaccounted for |
| 5 running it | execute (simulated arm), server, cli | a drawing submitted to the server runs to the end on simulated arms, motion by motion |
| 6 calibration | calib | the fits recover known errors from simulated touches |
| 7 gui | gui | shows a job: plan, progress, leftovers |
| 8 hardware | execute (real arm) | on the operator PC, with Pete |

## How agents work

- one agent per folder; an agent writes only inside its own folder and its own tests
- `aris/types.py` and this file are changed by the orchestrator only; an agent that needs a
  change asks for it in its report
- agents do not commit; the orchestrator reviews, runs the tests and commits module by module
- every agent reads `DESIGN.md`, `BUILD.md`, `aris/types.py` and `STAGED_LESSONS_2026-09-29.md`
  (the defect list of the old planner) before writing anything
