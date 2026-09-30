# BOTH COLOURS FOR EVERY ARM, AND THE TWO THINGS THAT STOOD IN THE WAY

The shipped `csail_final6` release draws **86.97 %** of the logo in one pass with
one pen per arm. `out/final6_undrawn_attribution.json` says where the missing
1.2807 m goes, and 61.7 % of it — 0.7897 m — is not a reach problem at all:
the paper is live, an arm can certify the stroke, and that arm is holding the
**other colour**. The remaining 0.4911 m is right-ink-but-refused.

Drawing it in TWO PASSES — all grey, a pen swap, all orange, every arm
available for both — lifts the colour constraint across phases and should
recover most of that. The machinery has existed since the sixarm era
(`--two-pass`, `csail_allocate.run_allocation`). It had never run on
`final6_opt`, and when it was pointed at this rig it did not fail at the
conductor, where `docs/MERGED_CANVAS.md` §3.3 predicted it would. It failed
twice before that, for two reasons neither of which is about colour.

## 1. A bag of ink an arm cannot fly

### 1.1 The symptom

The first two-pass allocation died inside the load balancer:

```
  File "aris_sixarm/allocate.py", line 1003, in score
    L = {a: load(a, own) for a in arms}
  ...
  File "aris_sixarm/sequence.py", line 357, in held_karp
    raise RuntimeError(f"no feasible order over {n} segments")
RuntimeError: no feasible order over 5 segments
```

Not a refusal, a traceback: the whole phase gone before a single timeline was
frozen.

### 1.2 The cause, exactly

Since `docs/PAPER_PLANE.md` the paper is an obstacle for pen-up motion, and
`sequence.cost_matrix` prices a crossing `paper.route` refuses as **`inf`**.
That is the right answer and it has a consequence nobody had had to face: a bag
of spans **each of which an arm certifies as INK** can have **no order in which
the arm can FLY between them**.

Arm 97's piece of stroke 26 is that span. Measured leg by leg, with the same
calls `cost_matrix` makes:

| leg | verdict |
|---|---|
| lower onto either end | `direct` |
| lift off either end | `direct` |
| **depot (ready pose) → its hover**, both ends | **REFUSED** (chain −73 / −105 mm) |
| **each of the arm's other four orange spans → its hover**, both ends | **REFUSED** (chain −137 to −189 mm) |
| its hover → each of those four | **REFUSED** |

So the arm can put the pen down on that span and pick it up again, and there is
no way to get the pen there. Every crossing dives 137–189 mm of chain below the
canvas and no shape on `paper.route`'s 8→40 cm ladder — lift, retract-and-go,
arc, Cartesian walk, fold-through-home — recovers it. In the arm's own transit
matrix the span has **in-degree 0**, and one unreachable span makes every
ordering of the other four unreachable with it.

**This is a reach fact, not a router deficiency.** The span sits at canvas
(0.739, 1.947)–(0.833, 1.801), near the far edge of arm 97's reach across the
mirror plane, and the configuration that inks it is on a branch nothing else can
be continued into.

### 1.3 The fix, in three parts

**Pricing is a question, not a commitment.** `arm_load` and `cluster_arm_load`
now return `allocate.UNFLYABLE` (`inf`) where they used to let the `RuntimeError`
out. `balance_loads` is a pricing function — it asks "what would this arm's
programme cost if it held these spans" — and the honest answer to an unflyable
bag is "more than any alternative", not a traceback.

**`inf` is not an ordering.** Priced `inf` and scored on the old
`(max, sum of squares)` ruler, every assignment containing an unflyable arm is
`(inf, inf)` — all equal, none strictly better — and the balancer's strict `<`
finds no move at all. It is what the second run did, verbatim:

```
balance: 7 of 25 segments are certified by more than one arm; 0 moves and
0 splits taken, busiest arm inf s -> inf s (the phase's floor)
```

`load_score` therefore counts the **unflyable arms first** and computes the old
two terms over the arms that have a tour. That gives the plateau a gradient —
handing the span to an arm that can reach it takes the count 1 → 0 and wins
whatever it costs in seconds. **Where every bag is flyable the first term is 0
for every candidate and the ruler is bit-identical to what it always was**,
which is what keeps every pinned number in the corpus reproducible.

**The tour is what gets asked, not a per-span predicate.** `prune_unflyable`
builds the arm's cost matrix once and drops the spans with no finite predecessor
or no finite successor, falling back to the shortest span when every node has a
neighbour and there is still no Hamiltonian path. The tempting cheap gate —
"can the arm fly to this span from its ready pose" — is **wrong**, and this rig
says so in the same phase: arm 71's orange bag has two spans the depot cannot
reach and a perfectly good 14.5 s tour that reaches them from its other work.
Only the tour knows.

It runs in two places, and they do different jobs:

- **before the cover, as a ban.** A span an arm cannot reach is not that arm's
  span, and the cover is where that belongs: banned there, `greedy_cover` hands
  the paper to somebody else instead of leaving a hole the size of the whole
  span. Arm 97's piece is 0.182 m; banned, arms 2 and 71 take all but the
  0.0497 m neither of them certifies. **The retry is worth 0.132 m of ink** over
  simply dropping what the arm cannot fly to.
- **after the balancer, as the guarantee.** The retry is the optimisation; this
  is what makes the bag the arm is finally handed one it can actually fly,
  whatever the relocations, swaps and splits did to it.

Two leaks had to be closed for the ban to stick. `rebalance` was handed the raw
`ivmap`, so it would relocate stroke 26 straight back to the arm that cannot fly
to it one move after the cover took it away; and `repair_gaps` **probes**, so it
answers about the hole a banned arm just left by asking that arm about it and
certifying what comes back. Both now see the filtered map.

## 2. The pass that may not ask an arm to move

### 2.1 The symptom

With the allocation fixed, phase 1 was refused by the conductor — the failure
`docs/MERGED_CANVAS.md` §3.3 records for all twelve historical two-pass
configurations, and in the same words:

```
arm 2 has no monotone pause schedule inside 116 s; 0 priority orders were
tried; v1 does not re-route; the impossible indices are INK: arm 2
segment(s) [6] — no order fixes that, only a different allocation
```

**INK**, not a transit: a collision while the pen is down, which no ordering and
no pause can fix. The re-sequence ladder was spent, and the unsplit allocation
was refused the same way (`arm 2 segment(s) [5], arm 97 segment(s) [8]`).

### 2.2 The cause

An INK index that is impossible at every instant is blocked by something that
never moves. In the grey pass arms 13, 31 and 17 draw nothing and stand at their
ready poses for the whole phase — and arm 31's is in the middle band, where the
grey ink now goes. In the shipped single-pass run arm 31 was *drawing*, so it was
never parked there. **Two-pass creates a class of arm that single-pass does not
have: one that is idle for an entire phase.**

`idle.py` already knows what to do about an arm standing in the way. Its own
comment says it:

> An arm parked on top of the ink another arm still has to draw is not a delay,
> it is a wall, and no amount of waiting gets past it.

and `plan_retreat` offers it the smallest certified pose that leaves the
interference set. But the rescue is guarded:

```python
if not retreat or policy == POLICY_HOME:
    raise _refusal(exc, progs, dt, orders, specs)
```

and `csail_schedule.build_phases` forces **every pass but the last** to
`POLICY_HOME`, because a pass that is followed by another has to leave the fleet
somewhere the next one can start from. So in a two-pass run, **phase 1 is
structurally the one pass on which no arm can ever be asked to step aside.**
That is why every two-pass attempt on this rig has died in phase 1 while the
single-pass run conducts, and it is not really a fact about the seam or about
go-home transits at all.

### 2.3 The lever

`--freeze-all-phases` puts phase 1 under `POLICY_FREEZE`, which re-enables the
rescue. The pass conducts:

```
priority search: 13 DP solves over the 6 orders of 3 moving arms in 34.6 s
  -> 50.0 s with 2 97 71 instead of 97 2 71
pass baseline: makespan 50.0 s, pause 59.0 s  KEPT
pass      jit: makespan 45.4 s, pause 49.2 s  KEPT
pass  retreat: makespan 45.4 s, pause 21.9 s  KEPT
idle policy: freeze, 1 retreat(s), 2 arm(s) taxiing slowly
```

**45.4 s, and the retreat pass takes the inserted pause from 49.2 s to 21.9 s**
without moving the makespan — the wait it removes is the wall, not a delay.

The flag exists for exactly this and its own help text calls it "the obvious
next thing to try"; what was not known before is *why* it is not optional here.
It has a real cost, which §4 prices: the pen swap happens with the fleet frozen
over the canvas rather than parked at `q_seed`, and `scene_check.check_static`
is what says whether that is safe to walk up to.

## 3. What closing the two leaks was worth on the clock

The ban only bites if nothing hands the paper back afterwards, and two things
did. Closing both is worth more than correctness — it gave the **load balancer
back**, because an arm priced `UNFLYABLE` makes the first term of `load_score`
equal for every candidate and the pass has nothing left to descend on:

| phase | busiest arm, leaks open | busiest arm, leaks closed |
|---|---|---|
| grey | 50.7 → 34.8 s | 50.7 → 34.8 s |
| **orange** | 57.7 → **57.7 s** (no moves possible) | 57.7 → **34.8 s** |

The orange pass's floor falls by 22.9 s purely because arm 97 is no longer
mispriced as unflyable.

## 4. The placement is already the right one

`docs/MERGED_CANVAS.md` §3.3's lever for a frame-clearance veto is to move the
logo, and the 0.20 m anti-corner-post shift is the precedent. Re-scored on a
±0.1 m grid at fixed rotation, on the two-pass allocation:

| placement (canvas centre) | coverage |
|---|---:|
| **(0.9017, 1.81532) — on the mirror plane, shipped** | **99.3907 %** |
| (1.0017, 1.81532) — +0.1 m in x | 98.2466 % |
| (0.8017, 1.81532) — −0.1 m in x | 98.2577 % |
| (0.9017, 1.71532) — −0.1 m in y | 95.1064 % |

**The centred placement wins on every neighbour, and the y move loses four
times as much as either x move.** That is §2's seam result being spent: the
23 cm strip is the best-covered band on the canvas (88.00 % GO, 74.91 % of it
cross-unit), so sliding the logo off the mirror plane trades the only paper two
units can share for paper only one can reach. The placement was chosen for
single-pass coverage and it happens to be even more right for two.

## 5. The gate that was refusing clear transits

With the allocation fixed and phase 1 conducting, EVERY two-pass configuration
was still refused by `scene_check`, always on the same thing: one arm's
clearance to the frame, always just under the 50 mm margin.

| placement | profile | phase | arm | frame clearance |
|---|---|---|---|---:|
| centre | qd0.30 | 1 | 71 | 49.4 mm |
| centre | qd0.60 | 2 (split) | 71 | 49.1 mm |
| centre | qd0.60 | 2 (unsplit) | 71 | 49.1 mm |
| (0, −0.1) | qd0.30 | 2 (unsplit) | 71 | 45.2 mm |
| (−0.1, 0) | qd0.30 | 1 | 97 | 37.4 mm |
| (+0.1, 0) | qd0.30 | 1 | 97 | 34.1 mm |

Eight refusals, one arm, never more than 5 mm short. That is not six
independent near-misses; **49.1 mm turned up twice on two different tours at two
different instants**, which is the signature of a measurement, not a geometry.

It is a measurement. `scene_check`'s frame gate is

```python
lb = static_clearance_lb(P[a], boxes) - 0.55 * stepd[a]
```

— a 1-Lipschitz sweep residual scaled by the SAMPLING STEP, and unlike the paper
gate beside it (`docs/PAPER_PLANE.md` §2.2, which auto-refines until "the
numbers stop moving") the frame gate never refined. So the verdict was a
property of the rate the timeline was handed in at. Measured on one timeline,
one arm, one instant (arm 71, t = 22.75 s in the orange pass):

| `--subcheck` | frame clearance | verdict |
|---|---:|---|
| 2 (default) | **49.1 mm** | FAIL |
| 8 | **53.5 mm** | **PASS** |

4.4 mm of phantom approach charged to a transit that was always clear.
`docs/MERGED_CANVAS.md` §3.3's 40.9 mm veto is very probably the same artefact.

**The fix is the paper gate's own.** `check_timeline` now refines the frame
check to `FRAME_STEP` (5 mm) of per-point motion and recomputes the residual on
the refined samples, so the verdict stops moving. Re-checked at the DEFAULT
`sub=2`, the shipped timeline reads **52.4 mm** and passes, with the gate
refining 11-16x per arm.

## 6. The shipped run

```
ARIS_RIG=final6_opt python3 scripts/csail_schedule.py --arms all --max-probes 5 \
    --rotate 90 --target-width 0.8417 --offset 0.0 0.0 \
    --atlas out/atlas_final6_opt --tag _sub8 --two-pass --freeze-all-phases \
    --qd-frac 0.60 --subcheck 8 --draw-speed 0.15 --transit-speed 0.30 \
    --fps 12 --substeps 4 --final out/csail_final6_2pass_final.png --program

ARIS_RIG=final6_opt /home/franka/git/franka_manipulation_station/.venv/bin/python \
    scripts/csail_drawing_demo.py --schedule out/csail_schedule_sub8.npz \
    --summary out/csail_schedule_sub8.json --out out/csail_final6.html \
    --zip out/csail_final6.zip
```

| | shipped single-pass | **two-pass** |
|---|---|---|
| coverage | 86.9704 % | **99.3907 %** |
| drawn / traced | 8.5488 / 9.8295 m | **9.7796 / 9.8295 m** |
| undrawn | 1.2807 m in 11 spans | **0.0599 m in 2 spans** |
| certified segments | 38 | **42** |
| **makespan** | 60.771 s | **81.688 s** |
| — phase 1 (grey) | — | **27.167 s** |
| — pen swap | — | **2.000 s** |
| — phase 2 (orange) | — | **52.521 s** |
| min inter-arm clearance | 82.9 mm | **82.2 mm** (margin 80) |
| min frame clearance | 65.6 mm | **53.5 mm** (margin 50) |
| pen-swap hold pose | n/a | **326.6 mm**, 6/6 poses pass |
| paper gate | PASS | **PASS**, 0 arms failed |
| animation | 29.5 / 16.4 MiB zip | **32.6 / 16.5 MiB zip** (budget 28) |
| tip vs drake FK | 0.175 mm | **0.195 mm** |

**+12.42 points of coverage for +20.9 s of clock.** The swap itself is 2 s of
that; the rest is two passes of ramp-up and the serialisation two passes impose
— phase 1 puts only arms 2 and 97 on the paper, and 47.2 s of the 81.7 is
conducted pause.

Who draws what, and the same arm draws both inks:

| arm | mount | grey | orange |
|---|---|---:|---:|
| 2 | A, side | 1.653 m | 0.888 m |
| 97 | B, side | 1.801 m | 1.529 m |
| 31 | A, inverted | — | 2.241 m |
| 71 | B, inverted | — | 1.669 m |
| 13 / 17 | floor | — | — |

Priority order 13 31 17 71 97 2 in grey, 13 17 71 2 31 97 in orange. The floor
arms still draw nothing, which is `docs/MERGED_CANVAS.md` §3.4's geometry and
not this allocation's doing.

### What is still undrawn, and why

> **SUPERSEDED, 2026-08-24 — see `docs/FULL_COVERAGE.md`.** Both spans are now
> drawn and coverage is **100.0000 %**, at a makespan of **73.500 s** (down from
> the 81.688 s below). Neither needed the pen tilt this section's diagnosis
> pointed at: `allocate.merge_remainders` lets an arm draw a span AND the ink
> lying against it as ONE segment, and both holes are absorbed by a segment
> their arm was already drawing — arm 97 extends stroke 26 to s = 0.7571, arm
> 31 extends stroke 30 down to s = 0.6488. No extra segment, no extra pen-up,
> and the pen stays perpendicular. §1.2's "arm 97 cannot fly to it" is still
> true of a STANDALONE segment there, and irrelevant to an arm that never flies
> to it because it never stops. The rest of this section is kept as the record
> of what was true before that.

| stroke | span | metres | reason |
|---|---|---:|---|
| 26 | s[0.6213, 0.7571] | 0.0497 | arm 2 certifies to 0.6213 and arm 71 from 0.7571; **arm 97 can ink the gap and cannot fly to it** (§1.2) |
| 30 | s[0.6488, 0.7012] | 0.0102 | 10.2 mm, under the 20 mm minimum stroke length — `degenerate/too_short` |

Both were already undrawn single-pass. **Every other span in the attribution is
recovered**: all 0.7897 m of wrong-ink, and all but 0.0497 m of the 0.4911 m the
right-ink arm refused.

### The placement was not nudged

All three ±0.1 m neighbours are worse on coverage (§4) AND fail to conduct:
(+0.1, 0) and (−0.1, 0) cannot conduct phase 1 at all, and (0, −0.1) certifies
phase 1 and then deadlocks on frozen poses in phase 2. The centred placement is
the only one of the four that gets a certified run.

