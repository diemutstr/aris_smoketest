# Aris: planner and drawing server for the six-arm drawing rig

Six Franka FR3 arms hang upside down from a frame over a paper-covered table and draw with
pens. This repository is the clean rebuild of the planning and execution software (branch
`aris3`). The old code lives on branch `aris2` only; its documentation is kept under
`legacy_docs/` because it is the record of what the old planner got wrong.

Start with `docs/DESIGN.md` (what the system is), `docs/BUILD.md` (how it
is laid out and built), and `docs/STATUS.md` (where each module stands). Every module
has a one-page description under `docs/modules/`.

## Install

```
python3 -m venv .venv && . .venv/bin/activate && pip install -U pip
pip install ./native/fr3_ik ./native/collide ./native/retime
pip install -e .
python -m pytest tests -q -m "not slow"    # under a minute
```

The three `native/` packages are compiled (g++ and cmake needed); without them the same code
runs on numpy fallbacks, slower. Drake (`pip install drake`) is needed for the checker's tests
only.

## Layout

```
aris/             the package
native/           the three compiled parts (IK, collision, timing)
config/           the rig description and, later, the calibration files
tests/            tests, fixed test sets, reference data
docs/             design, build plan, status, one page per module
assets/           the installation model (URDF, meshes), the vendor arm description, drawings
legacy_docs/      the old planner's documentation: decisions, lessons, audits, drawings
```
