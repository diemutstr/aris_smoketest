# One arm, drawing alone — how much of the run, and whether it had to be

`scripts/solo_time.py` over the shipped schedule; numbers in `out/solo_time.json`.
Read-only: nothing is re-planned, and the allocation is re-run only to recover
the one thing the pipeline computes and then throws away — **for every drawn
span, the set of arms that could have drawn it**. `allocate.rebalance` builds
that set (probe coverage, then a clean re-plan that must give back not one
millimetre), uses it, and returns a count. This rebuilds it span by span, and
asks the finer question the balancer cannot: *which PART* of each span the other
arms reach.

## How many pens are down at once

| arms drawing | whole run (105.4 s) | phase 1 (grey) | phase 2 (orange) |
|---|---|---|---|
| 0 | 18.3 % (19.3 s) | 26.9 % | 10.8 % |
| **1** | **35.3 % (37.2 s)** | 31.5 % | 38.7 % |
| 2 | 39.1 % (41.2 s) | 40.6 % | 39.4 % |
| 3 | 5.3 % (5.6 s) | 1.1 % | 7.9 % |
| 4 | 2.0 % (2.1 s) | 0.0 % | 3.2 % |

**Four arms draw together for 2.1 seconds of a 105-second piece.** More than a
third of it is one arm working while three watch, and nearly a fifth is nobody
drawing at all.

## Why the solo stretches are solo

40 stretches, 37.2 s of them. Classified by what the probe data says about the
span being drawn:

| | seconds | share of solo | what it means |
|---|---|---|---|
| STRUCTURAL | 11.9 s | 32 % | no other arm of that colour certifies any part of the span. Only a different PLACEMENT changes this. |
| **SPLITTABLE** | **18.0 s** | **48 %** | another arm certifies PART of the span but not all of it — invisible to a balancer whose only move is the whole segment. |
| ALLOCATABLE | 7.3 s | 20 % | another arm certifies ALL of it and the balancer left it alone, because handing it over whole would only have made the receiver the new busiest arm. |

**Two thirds of the solo time is not structural.** Phase 2 is where it lives:
20 of its 32 spans (4.21 m of ink) are partly reachable by a second arm and only
2 are wholly reachable — exactly the shape of problem whole-segment moves cannot
touch. Phase 1 is the mirror image (5 spans wholly movable, 5 partly, 1.56 m)
and is already within 2.7 s of its floor.

## What a splitting balancer could reach

With spans divisible the min-max load problem has a closed form: for every
subset A of arms, work that can only be done inside A must fit inside A, so
`T* = max over A of (W(A) + O(A)) / |A|`. Cutting each span at its neighbours'
certified interval boundaries gives 22 atoms in phase 1 and 51 in phase 2.

| | floor now | makespan now | split-capable floor (ink only) | with today's pen-up tours |
|---|---|---|---|---|
| phase 1 | 35.7 s | 38.4 s | 20.2 s (binding {31, 71}) | **33.8 s** |
| phase 2 | 65.1 s | 65.1 s | 31.8 s (binding {2, 97}) | **46.7 s** |
| both + swap | — | 105.4 s | — | **≈ 82.5 s** |

**Phase 2 is the whole prize: 65.1 → 46.7 s.** Its floor today is arm 97's own
work — 49.6 s of ink that no other arm certifies *end to end*, plus its own
pen-ups — and it now finishes exactly AT that floor, so there is nothing left
for the conductor to win. Splitting is precisely what dissolves it: at sub-span
granularity only 31.8 s of ink is arm 97's alone.

Three things the estimate does not pay for, all in the optimistic direction:
sub-span eligibility comes from the probe intervals rather than from a clean
re-plan of the sub-span (the stricter test the balancer applies today, and the
one a real splitter would have to pass); each cut adds an entry and an exit to
the receiving arm's tour, which in this piece runs **1.4–1.9 s per pen-up**; and
a floor is a floor — the conductor's pauses sit on top of it. A splitting
allocator that reached 55–60 s in phase 2 would already be most of the win.

**Next milestone: allocation v2 with stroke splitting.** The machinery is nearly
all there — `stroke_api.plan_stroke` is certified-or-split and
`truncate_polyline` cuts at any arc position, `allocate.probe_stroke` already
records per-arm certified intervals, and `balance_loads` already scores on the
timeline's own clock. What is missing is a move that cuts a placed span at an
interval boundary, offers each piece to the arms that certify it, and re-plans
both halves — plus the seam handling `place_cuts` already does for the cover.

## Allocation v2 is built, and the floor is not the makespan (2026-08-19)

`allocate.rebalance` now iterates **move | swap | SPLIT** until nothing
improves. A span on the busiest arm is cut at a chosen `s` and one piece handed
to a lighter arm that re-plans it from scratch; the pieces are `[s0, cut+e]` and
`[cut-e, s1]` so their union is the input span for **every** cut position, both
are held to 5 cm, and the 5 mm they share is ink laid down twice so the two pens
meet. Where the receiver's reach gives out is measured, not guessed —
`_Pricer.probe_span` asks `plan_stroke` about the sub-span the balancer is
actually holding and reads `s_star` off both ends — and the cut is then placed
by bisecting to where the two arms' loads cross. Coverage is invariant by
construction and `coverage_lost` re-derives it anyway: **0.000 mm** given back.

**It does what this study predicted, at the model this study used.** On the
whole logo, at unchanged coverage (99.2139 % of 11.567 m, the same single
0.0909 m span left empty) and unchanged certificates:

| | floor v1 | floor v2 | splits |
|---|---|---|---|
| phase 1 grey | 32.1 s | **30.7 s** | 1 |
| phase 2 orange | 65.0 s | **51.1 s** | 2 |

Phase 2's floor falls 21 %, and this study's estimate for it was 46.7 s "with
today's pen-up tours" — the splitter lands 4.4 s above its own prediction, which
for a greedy against an NP-hard objective is closer than it had any right to be.

**And then the conductor took it all back.** Conducted, phase 2 goes
**65.1 s → 95.3 s**. The floor fell 14 s and the makespan rose 30.

The reason is the one term neither this study nor `balance_loads` has: **the
load model prices every arm as though it were alone on the paper.** It is the
arm's ink plus the arm's own optimal pen-up tour, and it is exact — the frozen
timeline reproduces it to 1e-15 s. What it cannot contain is the other five
arms. Under v1, arm 97 drew 3.79 m of the orange pass largely by itself while
three arms stood still; that is 65 s of *solo* time, and solo time is free of
contention by definition. Balancing it away put arms 2, 97, 71 and 31 into the
middle of the sheet **at the same time**, the conductor refused the first
schedule outright, and the rescue it found spends 110 s of pause — arm 71 waits
64 s to do 31 s of work.

So the objective hierarchy decides it where the information is:
`csail_schedule.build_phases` conducts the split allocation **and** the unsplit
one and ships the faster. The second conduct is normally not paid for —
`nominal_floor` is an exact lower bound on an allocation's makespan and costs no
collision images, so an alternative that could not win even at its floor is
dropped before one is built.

**Phase 1 keeps its cut (38.35 → 36.96 s); phase 2 gives both of its own back
(95.3 → 65.06 s).** The piece runs in **104.02 s against 105.42**, at the same
99.2139 % of 11.567 traced metres, 82.1 mm of clearance against the 80 mm
margin, and `scene_check` PASS on both phases. 1.3 % — which is what an honest
version of this study's 82.5 s looks like once the missing term is paid.

**The honest reading of the 82.5 s projection is that it was missing a term,
not that it was wrong.** Every quantity in it is still true: 48 % of the solo
time really is splittable, the sub-spans really do certify, and the floor really
does fall to 51.1 s. What the projection costed was ink and pen-ups. What it did
not cost is that **solo time is the cheapest second in a six-arm cell** — an arm
drawing alone never waits — so converting solo seconds into concurrent ones buys
a lower floor and sells contention, at an exchange rate this fleet's geometry
sets and no allocator can see. `aris_sixarm/bench` measures that exchange rate
across five drawings that are not this logo. Splitting lowers the floor on
**every one of them** — 4.5 % on the hatching patch, 28.9 % on the single
continuous spiral, at coverage identical to the digit on all five — and the
conductor then keeps the cuts on three of the five and hands them back on two
(`docs/BENCH.md`): **8 cuts kept, 18 reverted.** The two it refuses are the two
where the ink is packed into one region — the hatching patch and the spiral,
both of which end up with an efficiency the fleet cannot improve because one arm
is doing nearly all of it (hatch conducts at efficiency 1.00 with **82 % of the
run one arm drawing alone**). That is the same shape as the CSAIL orange pass,
found twice more on geometry that owes it nothing.

So the finding generalises, and so does its limit: **stroke splitting is worth
5-29 % of the allocation floor, and how much of that survives is decided by
whether the fleet has room to draw concurrently, not by the allocator.** The
next real win in phase 2 is not a better cut; it is a conductor that can
re-route instead of only pausing.
