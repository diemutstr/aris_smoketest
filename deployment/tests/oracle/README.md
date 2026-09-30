# Reference data from the old planner

The scripts here produced the reference files in `../data/` by running the OLD code, which lives
on branch `aris2` only. They cannot run on this branch. The data files are committed; the tests
read the files, never the old code. To regenerate a file, check out `aris2`, run the script from
`deployment/` with `ARIS_RIG=proposed ARIS_TOOL=lateral`, and copy the result here.
