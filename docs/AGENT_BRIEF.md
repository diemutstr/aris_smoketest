# Brief for every agent working on this repository

## What this is

Six Franka FR3 arms hang upside down from a frame over a paper-covered table and draw with
pens held in a holder on the hand. This repository (branch `aris3`) is a clean rebuild of the planning and
execution software. The old code (`aris_sixarm/`, `scripts/`, about 90 000 lines) works but is
slow and hard to understand. The project lead, Pete, wants to understand every module of the new
code and how the modules compose. Clean and small beats clever.

Repository: branch `aris3`, checked out at `/home/franka/aris_project/aris3`. The package is at the repository root (moved out of `deployment/` on 2026-09-30).

## Read before writing anything

1. `docs/DESIGN.md` — what the system is
2. `docs/BUILD.md` — layout, import rule, the calls between modules, code rules
3. `aris/types.py` — the shared data types
4. `docs/LESSONS_FROM_THE_OLD_PLANNER.md` — what the old planner learned
   the hard way; read the parts that touch your module and do not repeat those mistakes
5. `docs/OPTIMIZATION_NOTES.md` — ideas parked for later; do not build them now unless
   your task says so

## Environment

- Interpreter: `/home/franka/aris_project/aris_sixarm/.venv/bin/python` (Python 3.12, numpy 2.5,
  scipy 1.18, pydrake 1.56, matplotlib, pytest). The package `aris` is installed editable.
- Run your tests with `.venv/bin/python -m pytest tests/<yours> -q`.
- g++ 13, cmake and ninja are installed. numba, pybind11 and toppra are not. Do not add a
  dependency without saying so in your report; prefer none.
- 32 cores. No robot, no ROS on this machine.

## The old code as a reference

- The old planner was removed from this branch (`aris3`) on 2026-09-30. It lives on branch
  `aris2` only, with its documents. This branch keeps the lessons file as
  `docs/LESSONS_FROM_THE_OLD_PLANNER.md`; file and line references in
  it point at branch `aris2`.
- The new package never imports the old one.
- Reference numbers produced by the old code are committed as data under `tests/data/`
  (see `tests/oracle/README.md`); tests read the files, never the old code.
- The rig is at height `h = 0.970` m (paper to mounting plate) with the lateral pen holder
  (`pen_lat = 0.0860369`, `pen_ext = 0.0460262`).

## Rules

- Write only inside the folders your task names, plus your own tests under `tests/`
  and your own page under `docs/modules/`. Other agents are working in the sibling
  folders at the same time.
- Do not edit `aris/types.py`, `docs/BUILD.md`, `docs/DESIGN.md` or this file. If you need a
  change to a shared type or a contract, say so in your report.
- Do not run git commands that change anything (no add, commit, checkout, stash, reset). The
  orchestrator commits.
- No environment variables, no global mutable state, no hidden caches in new code.
- Same input, same output. Anything random takes its seed from its input.
- An ordinary refusal is a return value with a reason, never an exception.
- Units: metres, radians, seconds. Name the frame in the variable (`p_base`, `T_table_base`).
- A file over 400 lines or a function over 60 lines needs a reason. Comments say why, not what.
- Tests that measure speed report the number and assert only a generous ceiling (ten times what
  you measured), using CPU time of the process (`time.process_time`), not wall-clock time: this
  machine is shared and often heavily loaded. Tests that take more than a few seconds each are
  marked `@pytest.mark.slow`; the quick set (`-m "not slow"`) must finish in under a minute.
- `aris/check/` reads `config/` with its own reader, by design (it shares no code with the rest).
  Everything else gets rig facts from `aris/rig.py`.
- Your module page (`docs/modules/<module>.md`) is one page, plain language, no internal jargon:
  the job, what goes in, what comes out, how it works, what it cannot do, the measured numbers.

## Your final report

Short. What you built (files), the test results with numbers, anything you could not do, any
change you need to a shared type or contract, and anything you found that the orchestrator or
Pete should know. Do not paste code.
