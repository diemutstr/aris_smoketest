# Optimisation notes

Ideas collected while designing, 2026-09-29. None of these is built. Build the simple version
first, measure on the corpus, then pick from this list. Numbers marked (est.) are estimates.

## Local planner

1. **Precomputed kinematic table.** The arm is symmetric around its base axis, so the kinematics
   can be tabulated once for all arms and drawings. Index: distance from base, spin relative to
   the outward direction, lean, elbow value, branch. Store: joint angles, singular value, joint
   limit distance, self-collision. A few million entries, ~100 MB, seconds to build (est.).
   A node becomes a lookup; no IK during the search. Live checks that remain: frame and other-arm
   collision, the joint 1 limit. Caveat: a tilted mount breaks the symmetry slightly, so the table
   guides the search and the final path is re-solved exactly and validated.
2. **Sparser edges.** (a) One change per step: spin, or lean, or elbow value, not several at once;
   about 11 predecessors instead of 60. (b) Slow variables: spin and lean may change only every
   few layers.
3. **Start narrow, widen where needed.** Upright pen and few spins first (~3 000 nodes per layer);
   open the lean only on the stretch where the path dies or gets thin. Old planner: ~70 ms plain,
   110-280 ms with the lean opened.
4. **Cheap tests first, collision last.** Joint limits and singular value on everything; collision
   only on survivors, or only on the winning path with a re-search if it hits.
5. **Coarse search, fine result.** Search at 1-2 cm steps, then re-solve densely along the winner.
6. **Implicit graph + guided search.** Compute a node only when the search asks for it. Pays only
   with an additive objective (gates as yes/no, minimise joint motion); with "maximise the worst
   point" a best-first search floods the graph. Needs a compiled search loop. Menu: keep searching
   until several end families are reached. Infeasible line: the search runs out of nodes.
7. **Lazy menu.** Menu entries are cheap (start shape, end shape, score, estimated time). Only the
   entries the sequencer picks are smoothed, densified, validated and timed.
8. **Parallel over lines.** Lines are independent; 32 cores on the dev machine.
9. **Separate graph from search in the code**, so sweep and guided search are interchangeable and
   both can be measured.

## Collision kernel

10. DONE (round 1): exact capsule-to-box distance without iteration. The old 36-step search was
    ~28 % of a whole single-arm plan; the new one agrees with brute force to 1e-15.
11. DONE (round 1): compiled kernel, joint angles in, clearance out. Real arm 31 scene (33 checked
    capsules, 2 178 pairs per configuration): numpy 12 700 configurations per second, compiled
    85 000 on one thread, 600 000 to 900 000 with threads. One path check of 50 samples: 1.5 ms.
12. Not yet: one bound around a group of obstacles (for example a parked arm's 40 capsules) as a
    first pass; most of the remaining time is the cheap per-pair first pass, 4 ns per pair.

12a. The body has 62 capsules since the tool was refitted to its meshes (33 of them on the tool,
    several only 2-3 mm thick near the paper), and 784 self pairs. Collision cost grows roughly in
    proportion. Idea: two levels, a coarse body of a few capsules against everything far away and
    the fine tool capsules only against the paper and whatever the coarse check says is close.

## IK

12b. IN PROGRESS (round 1): the vendored solver has the Panda's joint limits built in and returns
    one elbow root. For drawing-like poses it answers 58 % of round trips and gives nothing for
    26 % of poses. A corrected solver with the FR3 limits as arguments is being built.

## Sequencer

12. Price transitions on real joint-space move time, not paper distance.
13. Choose alternative, order and direction together along the tour (a chain search), not per piece.
14. Shortcut the RRT result, re-checking every shortcut.

## Measured baseline to beat (old code, single-arm word, 66.8 s)

| share | what |
|---|---|
| 59 % | pen-up routing between strokes |
| 37 % | certification pass |
| 3 % | stroke planner |

## System planner

15. **Smaller footprints, more for the follower.** Split a phase into chunks with a stop between
    them; the leader's footprint per chunk is smaller than for the whole phase. Needs only a
    "both done with chunk k" confirmation, not continuous timing.
16. **Tidy leaders.** Make the leader's free-space moves prefer its own side, so the footprint
    does not grow through an RRT detour.
17. **Footprint as a voxel distance grid.** The follower's check against the leader becomes a
    lookup. Conservative by one voxel.
18. **Six independent blocks as a first phase** (walls between rows and between columns): all six
    arms start at once with no dependency at all. Audit lower bound: 57 % of the canvas.
