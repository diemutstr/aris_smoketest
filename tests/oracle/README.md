# Reference data and its provenance

The files under `../data/` whose names say `reference` or `oracle` were produced by running the
OLD planner (branch `aris2`, commit 4b979ee and before; scripts `tests/oracle/make_*_reference.py`
on that branch) with `ARIS_RIG=proposed ARIS_TOOL=lateral`. The tests read the files, never the
old code. To regenerate one, check out `aris2` and run its script; the keys in those files still
carry the old arm numbers (13, 17, 31, 71, 2, 97), which the tests map to slots.

What is left here runs on this branch:
- `arm_meshes.py`, `fit_arm_capsules.py`: fit the arm's collision capsules to the vendor meshes
  (the capsule table in `aris/kernel/arm.py` is derived from them; the test against the meshes
  is in `tests/test_kernel_arm.py`).
