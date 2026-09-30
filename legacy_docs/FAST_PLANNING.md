# THE ROBOTS START DRAWING SOONER, AND WHAT WAS ACTUALLY IN THE WAY

`scripts/draw.py` takes a picture and gives back a certified programme. On the
Trollface that took **4028.6 s**, and the number worth optimising is not the
whole grid but the **time to the FIRST certified programme** — the only one a
person standing in front of the fleet can feel.

It is now **268.8 s**, and it took two passes. The first went after the
allocation (§§1–5): it moved that stage by **9.2x** and left the first certified
programme at 1916.7 s, of which 178.7 s was planning and **1737 s was one
conduct**. The second went after the conduct (§§6–9) and left it at **95 s** —
and the finding that made that possible is that the conduct had never been the
search its own log blamed. It was the collision geometry the search was
silently building.

Everything below is measured on the same input: the Trollface at the placement
it shipped (0.850 m wide, offset (0, −0.95), rotation 0, `final6_opt`, six
arms, `--max-probes 5`), with the before and after run on the same machine.

## 1. Where the clock was, and the surprise inside it

One profile's allocation, timed stage by stage:

| stage | seconds | share |
|---|---:|---:|
| prefilter | 0.2 | |
| probe | 8.7 | 0.9 % |
| repair | 0.2 | |
| clean re-plans | 3.3 | 0.3 % |
| **balance** | **931.1** | **93.4 %** |
| sequence | 13.7 | 1.4 % |
| **total** | **997.4** | |

and `--select-profile` runs four of those, one after another, for a grid
that took 1646.6 s before a single timeline was conducted.

The balancer's own record for that run says what it bought: **0 relocations, 0
swaps, 5 splits**, on a cover where only 7 of 57 segments have a second arm that
certifies them at all. Fifteen minutes to move nothing and cut five spans.

### 1.1 The four things that were wrong

**Pricing was from scratch.** `arm_load` rebuilt the arm's whole cost matrix for
every candidate bag and re-solved its tour from the greedy seed under the
sequencer's full 2 s budget. Every candidate the balancer considered differed
from the bag before it by ONE span.

**The scan was best-improvement.** Every relocation and every swap out of the
busiest arm was priced before one was taken — and the effect of a move on the
two loads it touches is bounded by the ink it hands over, which costs nothing to
compute.

**Nothing knew when to stop.** A phase whose busiest arm is already at the lower
bound its ink imposes cannot be improved by any assignment, and the pass had no
way to say so.

**The split search enumerated.** Eight segments x five receivers x two coarse
cuts, every one of them re-planning two halves and pricing two arms' tours.

### 1.2 And the thing nobody had measured

A profiler on the fixed balancer said something none of the above predicted:
**94 % of building a cost matrix is `paper.route`.** At this placement **57 % of
the crossings between one arm's spans dive through the canvas**, and each of
those walks the ladder — lift, retract-and-go, arc, Cartesian traverse,
fold-through-home — at about 36 ms. A whole allocation asks about **57 000
distinct crossings**: half an hour of ladder-walking, and everything else in
`sequence.py` is array arithmetic beside it.

That is the real cost model of this pipeline, and it had never been named.

## 2. What replaced them

### 2.1 The crossing is a property of two spans (`sequence.leg_costs`)

`C[a, b]` is the lift off node a, the hover-to-hover crossing and the lower onto
node b. Every term is a function of those two poses, the arm and the run's fixed
speeds — never of which OTHER spans happen to be in the bag. So the matrix over
the union of every span the balancer offers an arm CONTAINS the matrix over any
bag of them as a submatrix, and re-pricing is an index operation.

`allocate._ArmMatrix` keeps one such union per arm. It GROWS (a split makes new
spans): `ensure` appends the missing spans and computes only the new rows
against the old columns, the old rows against the new, and the new block against
itself. `matrix(keys)` slices a bag's `(C, T, e)` out of it. `tests/test_balance.py`
pins that the slice equals the freshly built matrix cell for cell under both cost
models, and that growing the union does not disturb what was in it.

`cost_matrix` and `cluster_cost_matrix` are now assembled from the same three
pieces (`node_lift`, `leg_costs`, `depot_legs`), so there is one definition of a
crossing and not two.

### 2.2 The tour search starts from the tour it already had (`sequence.seed_from`)

A candidate is the current bag plus or minus one span, and the arm's existing
tour already orders everything else in it. `seed_from` filters that tour down to
the segments this bag still has and drops the newcomers in at their cheapest
position; the descent does the rest, under `BALANCE_BUDGET` (0.15 s) instead of
`sequence.TIME_BUDGET` (2 s), and Held-Karp is used only while its state count
stays under `BALANCE_EXACT_STATES`.

The DP, the matrix and the objective are identical — only the effort spent
searching differs, and **the bag that finally ships is re-sequenced at full
budget by `allocate` regardless**. The price is memoised on the bag, which is
what keeps `load_score` a potential: a bag priced once keeps that price, every
accepted move strictly lowers the objective, and the loop still terminates.

### 2.3 First improvement, in an order the ink already knows

`balance_loads` sorts its candidates by the busiest arm the ink alone predicts —
`max(L[src] − d_i, L[b] + d_i)` for a relocation — and takes the best of the
first promising prefix rather than pricing all of them. `BALANCE_PATIENCE` is
why it is a prefix and not literally the first: a candidate that improves only
the sum-of-squares tie-break is an improvement, and taking it costs the round
that a real reduction of the maximum would have used.

Over 200 random synthetic instances (8-40 segments, 2-6 arms) against the
exhaustive scan: **worst case 1.5 % above best-improvement, mean 1.0000, and
5.4x fewer candidates priced.**

### 2.4 The floor is where improvement stops being possible (`_phase_floor`)

Every arm's load is at least its ink, and every segment costs at least what the
cheapest arm that certifies it would spend drawing it. Two exact bounds follow:
the longest single segment (no assignment of undivided segments beats it) and
the total ink spread over the arms that can hold any of it (which survives
splitting, because a cut adds the splice and the total only grows). The
improvement loop stops within `BALANCE_EPS` (2 %) of the first; the split search
never starts if the busiest arm is already at the second.
`tests/test_balance.py` enumerates small instances outright and checks nothing
gets under either.

### 2.5 The split search ranks before it prices

`_cut_gain` judges a cut on the ink and the seam alone — two multiplications —
and only `SPLIT_CAND_TRIES` of that ranking are ever priced; the winner is then
placed exactly by `_bisect_cut`, which is where the cut position was always
really decided. A receiver no more than `SPLIT_MIN_GAIN` lighter than the source
is not probed at all.

Two more things keep the union small. The shortlist is CERTIFIED in bulk and its
matrices grown in ONE step (`_Pricer.grow`), because a growth step's wall clock
is its slowest route and doing that eight times over is eight waits. And
`_Pricer.compact` drops the cuts a round rejected: pure index arithmetic, no
crossing re-screened, and without it the round after prices its candidates
against a union carrying every reject before it — on this picture that took an
arm's union from the 36 spans it can hold to 115.

### 2.6 The crossings are routed on every core there is

`sequence.prewarm` collects the diving crossings of a whole growth step and
routes them in one dispatch over a **persistent** fork pool
(`sequence.screen_pool`); `sequence._PRICED` memoises the PRICED detour, not just
the route, so a crossing is walked exactly once per run however many matrices
ask about it. The batch matters as much as the pool: a batch's wall clock is its
slowest member, and a crossing that CANNOT be routed walks the whole ladder.
Measured on the largest dispatch — 14 314 crossings in 21.6 s — the pool runs at
**23x**.

Three details are load-bearing. The key is computed in the PARENT and handed to
the worker, because `paper`'s memo keys on `id(spec)` and a worker's copy of the
arm is a different object. A crossing the parent has already routed is priced in
the parent, because a worker forked before it was bought would walk the ladder
again — and that is also what makes the other three profiles cheap. And a worker
never forks a pool of its own.

## 3. What the four profiles may share, and what they may not

Nothing the planner does depends on `qd_frac`. `plan_stroke` resolves the
redundancy against reach, clearance and the band objective; the joint-speed cap
enters only when `writing` turns a path into a clock and `sequence` prices a
transit. `allocate.plan_family` is that statement as a key — the strokes, the
arms, each arm's pen and its band objective — and the probe pass and the clean
re-plans are memoised under it in a `share` bag the caller keeps across
allocations. `paper`'s route memo and `sequence`'s priced-crossing memo need no
key at all: a pen-up route knows nothing about `qd_frac` either, and they are
process-global.

The four cells were always run in one process, so the route memo was already
shared; what `plan_family` adds is the PLANNER's half of it — the probes and the
clean re-plans, which are the two stages that still cost real seconds once the
crossings are memoised. Between them they take the second cell of a family from
70.9 s to 70.9 s of pure balancing with nothing bought at all (see §5.2).

That is also why `--profile-jobs` stays at 1. Forking the four allocations buys
cores and gives up both memos, and a worker may not fork the route screen's own
pool either, so each of the four would pay serially for geometry the shared run
buys once.

`tests/test_balance.py` pins that two profiles sharing a bag allocate exactly
what they allocate alone — on the spans themselves, not on a summary — and that
the second one re-probes nothing. The probe map is COPIED into each allocation
because `repair_gaps` appends to it.

The fiber menus are deliberately NOT shared: `menu.drop_lattice` releases the
lattice in place, and the second profile would be re-materialising what the first
one dropped.

## 4. The first programme, and then the rest

`select_profile` used to allocate all four cells, order them by floor, and
conduct them one at a time so each could prune the next. Nothing was written
until all four had been decided.

It now runs in two stages. `FIRST_PROFILE` names the cell this rig has always
shipped — `qd0.60+cluster`, what the CSAIL logo ships at (77.792 s against
104.021 s on the defaults, `docs/BENCH.md`), what the Trollface ships at, what
two-pass ships at — and it is allocated, conducted and certified BEFORE the other
three are allocated at all. The moment `scene_check` signs its timeline off,
`on_certified` fires and `draw.py` puts a runnable programme on disk marked
`provisional`, with `elapsed_s` counted from the picture.

The remaining three are then allocated, pruned against that certified makespan,
and conducted **in parallel** — they no longer prune each other, so conducting
them together costs the longest one instead of the sum. They replace the
provisional best only if one of them is faster, and the files say which is which.
`--conduct-jobs 1` restores the serial chain every published number was measured
on.

## 5. The numbers

### 5.1 One allocation, the profile that ships (`qd0.60+cluster`)

| | before | after |
|---|---:|---:|
| **balance** | **931.1 s** | **132.7 s** |
| probe | 8.7 s | 8.8 s |
| clean re-plans | 3.3 s | 3.4 s |
| sequence | 13.7 s | 4.5 s |
| **allocation total** | **997.4 s** | **154.9 s** |

and the allocation is the same allocation:

| | before | after | delta |
|---|---:|---:|---:|
| coverage | 98.7970 % | 98.7970 % | 0 |
| certified segments | 59 | 59 | 0 |
| splits taken | 5 | 5 | 0 |
| busiest arm, before balancing | 83.840 s | 84.017 s | +0.2 % |
| busiest arm, after balancing | 73.980 s | 74.038 s | **+0.08 %** |
| phase floor (`nominal_floor`) | 73.373 s | 73.417 s | **+0.06 %** |

Well inside the 2 % the mandate allowed, on both numbers it named.

### 5.2 All four, in one process

| profile | allocation | balance | probe | busiest arm | floor |
|---|---:|---:|---:|---:|---:|
| qd0.60+cluster (first) | 160.0 s | 137.3 s | 8.9 s | 84.02 -> 74.04 s | 73.42 s |
| qd0.30 | 63.1 s | 43.8 s | 9.5 s | 149.04 -> 136.71 s | 138.52 s |
| qd0.30+cluster | 56.0 s | 49.4 s | **0.0 s** | 112.96 -> 94.05 s | 99.87 s |
| qd0.60 | 21.8 s | 19.4 s | **0.0 s** | 99.03 -> 91.25 s | 91.76 s |
| **total** | **300.9 s** | | | | |

All four allocate the same 98.7970 % over 59 segments. The two zeroes are
`plan_family` doing its job: `qd0.30+cluster` asks the planner exactly what
`qd0.60+cluster` asked, and `qd0.60` exactly what `qd0.30` asked. The falling
balance times down the column are `paper`'s route memo doing its — by the fourth
cell every crossing on the picture has been walked already.

AND THE GRID IS NOT FOUR TIMES ONE ALLOCATION, WHICH IS WORTH SAYING BECAUSE IT
WAS NOT BEFORE EITHER.  `paper`'s memo has always been process-global, so the
old pipeline was already getting some of this for free and the honest comparison
is the pipeline's own log, not four cold runs:

| | before | after |
|---|---:|---:|
| qd0.30 | 202.8 s | 70.9 s |
| qd0.30+cluster | 1124.8 s | 70.9 s |
| qd0.60 | 87.7 s | 30.8 s |
| qd0.60+cluster | 231.3 s | 178.7 s |
| **four allocations** | **1646.6 s** | **351.3 s** |

(both measured inside `scripts/draw.py --select-profile`, which is why the
per-cell numbers differ from the standalone table above: the order is not the
same, and whichever cell goes first pays for the geometry.)  A COLD allocation —
one profile, nothing memoised, which is what the first cell of any run is —
goes 997.4 s to 154.9 s.

### 5.3 A second picture: the CSAIL logo, two passes

`docs/TWO_PASS.md`'s allocation — both inks, every arm available for both,
`--freeze-all-phases`, `qd0.60 --cluster` — run on the old code and the new,
same input, same rig:

| | before | after |
|---|---:|---:|
| grey pass, balance | 218.9 s | 26.5 s |
| orange pass, balance | 636.2 s | 74.4 s |
| **both passes, wall** | **879.8 s** | **113.9 s** |

and this one is not merely within tolerance, it is the same allocation:

| | before | after |
|---|---:|---:|
| coverage | 100.0000 % | 100.0000 % |
| certified segments | 46 | 46 |
| splits taken | 2 + 5 | 2 + 5 |
| busiest arm, grey | 18.847 s | 18.847 s |
| busiest arm, orange | 19.108 s | 19.108 s |
| **profile floor** | **41.2018 s** | **41.2018 s** |

That last row is what settled one design question. The split search first
stopped at the first candidate that improved, as `balance_loads` does — and on
the orange pass that took 5 splits down to 3 and left the busiest arm 8 %
heavier (20.641 s against 19.108 s). The two cases are not alike: in
`balance_loads` pricing a candidate IS the cost, while in the split search the
cost is the certifying and the matrix growth, which the whole shortlist has
already been charged for before any of it is priced. Pricing all of it costs
about two seconds a round and gets the splits back.

### 5.4 The Trollface, end to end

`scripts/draw.py` on the picture at the placement it shipped,
`--select-profile`, animation off (the render is a separate stage and needs the
drake venv).  Both runs on this machine, at the same time, so neither had it to
itself:

| | before | after |
|---|---:|---:|
| trace | 1.0 s | 1.0 s |
| four allocations | 1646.6 s | 351.3 s |
| — the winning cell alone | 231.3 s (fourth) | 178.7 s (first) |
| conducts | 1921 + 461 s, serial | 1737 s, then 2 in parallel |
| **time to a certified programme ON DISK** | **4028.6 s** | **1916.7 s** |

**2.1x on the headline, and that is the honest number.**  The old pipeline wrote
nothing until the whole grid had been decided, so its first programme and its
last are the same event; the new one writes the provisional best the moment
`scene_check` signs it, and 1737 s of the 1916.7 s that takes is ONE CONDUCT.

The stage this pass was aimed at moved by a lot more.  Time from the picture to
an allocation that could be conducted at all: **1646.6 s to 178.7 s, 9.2x** —
because the old run had to allocate all four cells before it ordered them by
floor, and the new one finishes the cell that has always won first.

and the programme that came out is not merely as good, it is the SAME
programme, to every digit the summary carries:

| | shipped | this run |
|---|---:|---:|
| execution profile | qd0.60+cluster | qd0.60+cluster |
| makespan | 108.375000 s | 108.375000 s |
| coverage | 98.7970 % | 98.7970 % |
| min inter-arm clearance | 82.918 mm | 82.918 mm |
| certified segments | 55 | 55 |
| drawn | 13.114555 m | 13.114555 m |
| conducted pause | 93.166667 s | 93.166667 s |

Which is the point worth making about all of §2: none of it is an
approximation of the old allocator. It is the same cost model, the same DP and
the same objective, asked fewer times and on more cores.

## 6. The conduct, and what its own log was timing

The pass above ended by naming two levers, and got the first one wrong. The way
it was wrong is the interesting part, because the same mistake is available to
anybody reading a profile.

It read this line —

```
priority search: 13 DP solves over the 6 orders of 3 moving arms in 163.1 s
```

— as "13 DP solves at about 12 s each", and concluded that the next order of
magnitude was in parallelising the search. A profiler on the Trollface's first
conduct says otherwise:

| | seconds | share |
|---|---:|---:|
| the 12 DP solves | 2.1 | 1.1 % |
| the 15 collision images they read | 187.1 | 98.9 % |

**The search was never the cost.** `coordinate` builds a collision image the
first time somebody asks for it, and the only thing that ever asks is the
search — so every second of geometry was billed to the line that reports the
search. A DP solve is 0.18 s, not 12.

(The quoted line is from a later conduct of the same run — one that certifies,
and so solves 13 times rather than the 12 the first one gets through before it
refuses. At 0.18 s a solve the distinction is worth 0.2 s, which is the point.)

Four things were wrong with those 187 seconds.

### 6.1 The broad phase rejected nothing

`clearance_matrix` opens with a broad phase over per-sample bounding spheres,
and the module has always described it as the thing that "throws away every pair
that cannot possibly be closer than `cap`". Measured, on this picture, it throws
away **nothing at all**: 100.0 % of the 19.3 M sample pairs its three moving
arms make survive to pay all 49 exact capsule distances.

It cannot do anything else. The sphere is drawn round the WHOLE chain of one arm
at one instant, so its radius is most of the arm's reach; two arms standing about
a metre apart have overlapping bounding spheres in every pose either of them can
hold. The test was asking a question whose answer is always yes.

### 6.2 The picture was built twice, once each way round

`free_cells(b, a)` is `free_cells(a, b)` transposed. The four-corner minimum is
symmetric and so is the sweep term. But the priority search asks for both
directions of every moving pair — an order that puts `a` before `b` and one that
puts `b` before `a` are both in the enumeration — and the old `cells()` built
each from scratch: 46.0 s for (31, 2) and then 45.8 s for (2, 31). Of the 15
ordered images one conduct reads, only **12 are distinct**.

### 6.3 Nothing survived one conduct into the next

`idle.conduct` conducts the same fleet up to four times (baseline, rescue, JIT,
retreat) and each pass re-programmes ONE or TWO arms; `idle._programs` hands the
others back the identical programme object. Every pass then re-sampled all six
arms and rebuilt every image. Above that, `build_phase` re-conducts after a
re-sequencing and `build_phases` conducts a second allocation — on the Trollface
that is **26 conducts** of a fleet in which two of the three moving arms often
did not move between one and the next.

### 6.4 It ran on one core

On a machine with 32.

## 7. What replaced them

Every one of these is an arithmetic saving, not an approximation. The images
that come out are the images that came out before, cell for cell — which §8.4
checks on 26 real conducts rather than asserting.

### 7.1 A broad phase that asks a question with two answers (`live_capsules`)

A capsule lies inside the box of its two endpoints grown by its radius, so the
distance between two such boxes is a lower bound on the distance between the
capsules. If that bound is at least `cap` the pair cannot contribute a value
below `cap` — and a value at or above `cap` is clipped to `cap` anyway. So
dropping it changes nothing, and it costs 49 box tests.

Over the whole path that already helps: on the Trollface's three moving pairs it
retires 14 %, 45 % and 51 % of the exact work, because the base column never
approaches anything and neither does the upper arm of an arm working the far
side of its sheet.

The useful part is doing it per TILE. Over its whole path an arm's forearm gets
everywhere; over 32 consecutive samples it does not, and the tile-local boxes
retire far more than the whole-path ones — on the busiest pair, 42 of 49
capsule pairs survive globally against a mean of 14 of 49 per tile. The tile is
a property of the arithmetic and not of the answer: `clearance_matrix` returns
the same matrix at every tile size, and `tests/test_csail.py` checks three of
them against a reference that does no rejecting at all.

### 7.2 One image per unordered pair, keyed on what it is made of

`build_images` is handed every pair any order could ask for, canonicalises each
to `(min(a,b), max(a,b))`, and reads the other direction as a transpose. The key
is `(ArmPath.key, ArmPath.key, margin, sweep)`, and `ArmPath.key` is a blake2b of
the world-frame capsule endpoints, the radii and the per-sample step — exactly
what `free_cells` reads and nothing else. Two paths with that key have the same
image whoever built them, which is what makes the JIT and retreat passes cheap:
they re-programme one arm and inherit the rest. Keying on `id()` would never
hit, because `idle._capsules` builds fresh objects every pass.

Reading the transpose also makes the image a function of the UNORDERED pair,
which is strictly more determinism than there was before. `seg_seg_dist` clamps
one parameter before the other, so the two directions really can disagree: over
90 ordered pairs of six real conducts, 36 of them have a `clearance_matrix`
that is not exactly its own transpose, worst case 2.5e-07 m — a quarter of a
micron against an 80 mm margin. It never reaches the boolean. Over those same
90 pairs the number of FREE-CELL images that differ between the direct build
and the transpose is **zero**, which is what licenses the shortcut.

The bag is gathered UP FRONT rather than on first use, and that is not
speculation. Arm k is conducted against the k-1 arms before it and every parked
arm, so the union over every order is exactly `moving x everyone else` — the set
a single complete schedule needs. Asking for them together is what turns fifteen
serial waits into one dispatch.

### 7.3 The pool, and why the batch is row blocks

A cell of an image reads two clearance rows and nothing else, so row blocks of
one image are independent. The batch is sliced to about four blocks per worker,
because a conduct's images are not the same size — 13.9 s, 4.8 s and 4.0 s — and
a task per image would leave most of the machine waiting on the largest one.

The pool is FORKED with the paths already in memory: an `ArmPath` is about a
megabyte of float32 and there are six of them, so copy-on-write ships them for
nothing where `pool.map` would pickle them once per task.

`image_jobs` scales the worker count to the BATCH and not to the machine. The
kernel is memory-bound — measured, the build stops getting faster somewhere
around eight to twelve processes — so more workers only cost the fork. And a
process that is already a pool worker takes the serial path, because a daemonic
process may not have children; that is why the two profiles `select_profile`
conducts in parallel improve by less than the one it conducts in the parent
(§8.2).

### 7.4 The DP stops building where it stops looking

`_dp`'s free-cell table is one gather per already-scheduled arm over (n-1) x D
booleans — 29.5 MB apiece, and a measured 184 ms of a 253 ms solve — while the
forward pass breaks the moment the arm can arrive and stop. On the solve
measured, it broke at step 3525 of a horizon of 10639, so two thirds of that
gather was for time steps nothing ever read. It is now built a block at a time,
the first block being the arm's own length because the arm cannot possibly
arrive before it has advanced n-1 times.

This one helps the solves that ARRIVE and not the ones that refuse: a refusal
has to look at the whole horizon before it can say no.

`tests/test_csail.py` pins it against a plain non-blocked implementation of the
same recurrence over 60 randomised instances, deadline and rest-suffix included,
on the arrival step AND the whole progress trace.

### 7.5 What was NOT done, and why

**The priority search still enumerates.** §6 of the previous pass suggested
stopping once a complete order reaches the phase floor. It would work, and now
that the geometry is out of the way the search really is most of a `coordinate`
call — 2.1 s of a 3.9 s cold one, and essentially all of a warm one. But the
winner is currently `min` over complete orders of `(makespan, pause, order)`,
and that third key is what makes it "a function of the geometry alone, with no
dependence on the order the permutations were walked in". Stopping at the first
order to reach the floor would return whichever one the busiest-first descent
happened to find. That is a stated property, and the whole search is 2 s of a
95 s conduct, so it was not traded.

**The split/unsplit A/B is still serial.** The previous pass proposed conducting
the two allocations concurrently, and on the old code that was worth most of an
hour: the split allocation's six conducts cost about 1041 s and all of them
refused. They now cost **24.3 s**, against 9.5 s for the unsplit conducts that
follow — so running them concurrently could save at most 9.5 s of a 94.8 s
conduct, in exchange for speculatively conducting an allocation that usually
loses. The lever was real and this pass spent it, by making the wasted conducts
cheap rather than by hiding them behind the useful ones.

**`select_profile`'s conduct pool is still daemonic.** Making it non-daemonic
would let the profiles it conducts in parallel fork image pools of their own,
which §8.2 shows is worth roughly another 5x on those profiles. It is off the
critical path — the first certified programme comes from the profile conducted
in the PARENT — so it buys grid wall-clock and not time-to-first, and it would
put an untested nesting under the part of the pipeline that decides what ships.

## 8. The numbers

Same machine (32 cores), same rig, same picture and placement as §5. The "before"
column is a `git archive` of the previous commit, run to completion on the same
box; both runs use the shipped flags (`--draw-speed 0.15 --transit-speed 0.3`,
the default 24 fps clock, `dt = 0.0208 s`).

### 8.1 One conduct, taken apart

The Trollface's first conduct — three moving arms, three parked, horizon 10639
steps:

| | before | after |
|---|---:|---:|
| the 15 images the search reads, one core | 187.1 s | 45.4 s |
| the 12 distinct ones, as a bag, one core | — | 22.8 s |
| the same bag, pooled | — | **1.8 s** |
| the same bag, already held | — | **0.02 s** |
| the 12 DP solves | 2.1 s | 2.1 s |
| **one whole `coordinate()`** | **189.2 s** | **3.9 s** cold, **2.1 s** warm |

Every image the new kernel produces is bit-identical to the old one, at every
tile size and job count — checked pair by pair, all 15 of them, `identical=True`.

The DP row is the one that did not move, and it is the point of §6: the stage
the log blamed was already 1 % of the conduct.

### 8.2 The Trollface, end to end

| | before | after |
|---|---:|---:|
| allocation, the profile that ships | 174.2 s | 172.8 s |
| the conduct that certifies it | 1673 s | **95 s** |
| **time to the FIRST certified programme** | **1848.3 s** | **268.8 s** |
| whole four-profile grid | 9044.5 s | 1296.6 s |

**6.9x to the first certified programme, 17.6x on the conduct.**

(§5.4 quotes the same old code at 1737 s and 1916.7 s rather than 1673 s and
1848.3 s. That run shared the machine with its own "before" run, as it says; the
baseline here had the box to itself. The old code is the same old code — this is
what a quiet machine is worth, and it is the column the 6.9x is measured
against, so the comparison is if anything conservative.)

Inside that conduct window the conductor is no longer what is in it:

| conduct window, qd0.60+cluster | before | after |
|---|---:|---:|
| the 9 `coordinate()` calls | 1609.4 s (96 %) | 33.8 s (36 %) |
| `scene_check`, untouched | 36.3 s (2 %) | 36.1 s (38 %) |
| freezing timelines, densify IK, retreat | ~27 s (2 %) | ~25 s (26 %) |

Per profile, the improvement depends on whether the conduct runs in the parent
(which forks an image pool) or in one of `select_profile`'s daemonic workers
(which cannot, and takes the serial path — §7.3):

| profile | conducted in | before | after | |
|---|---|---:|---:|---:|
| qd0.60+cluster (ships) | parent, pooled | 1672.8 s | 94.8 s | 17.6x |
| qd0.30+cluster | pool worker, serial | 7018.2 s | 850.6 s | 8.3x |
| qd0.60 | pool worker, serial | 477.2 s | 65.5 s | 7.3x |

and the programme that comes out is the same programme:

| | before | after |
|---|---:|---:|
| makespan | 108.375 s | 108.375 s |
| profile floor | 73.417 s | 73.417 s |
| conducted pause | 93.2 s | 93.2 s |
| min inter-arm clearance | 82.9 mm (margin 80) | 82.9 mm (margin 80) |
| `scene_check` | PASS | PASS |
| coverage | 98.7970 % over 55 segs | 98.7970 % over 55 segs |
| all four profile floors | 138.516 / 99.873 / 91.760 / 73.417 | identical |
| the outcome table | qd0.30 pruned, two refused, one ships | identical |

That is not a tolerance. `troll_strokes.json` is byte-identical; of the 524
leaves of `troll_schedule.json` and the 6921 of `troll_program.json`, the only
ones that differ are wall-clock measurements; and all **54 arrays** of
`troll_schedule.npz` — the schedule itself — are bit-identical.

### 8.3 The CSAIL logo, two passes

`docs/TWO_PASS.md`'s shipped command, unchanged, on the old code and the new —
a different picture, a different clock (12 fps, 4 substeps), two phases and a
`--freeze-all-phases` allocation:

| | before | after |
|---|---:|---:|
| **whole two-pass run, wall** | **910.9 s** | **157.5 s** |
| phase 1 priority search | 13.05 s | **0.05 s** |
| phase 2 priority search | 93.06 s | **1.04 s** |

The two search lines are the conductor on its own, with nothing else in them,
and they are where the 5.8x comes from. What comes out is again the same:

| | before | after |
|---|---:|---:|
| coverage | 100.0000 % | 100.0000 % |
| grey, makespan / pause | 27.2 s / 1.4 s | 27.2 s / 1.4 s |
| orange split, makespan / pause | 44.3 s / 36.2 s | 44.3 s / 36.2 s |
| orange unsplit, makespan / pause | 54.6 s / 54.6 s | 54.6 s / 54.6 s |
| min inter-arm clearances | 89.4 / 82.3 / 90.3 mm | 89.4 / 82.3 / 90.3 mm |
| `scene_check` | PASS | PASS |

— down to the timestamp and the arm pair each clearance was measured between. Of
the 598 leaves of `csail_schedule_sub8.json`, three differ: the two search wall
times above, and the path the picture was read from.

### 8.4 The schedules are the same schedules

The end-to-end comparison says the shipped programme matches, which is weaker
than it looks: a run only ships one of its conducts. So every conduct of the old
run was captured with its inputs and replayed through both builds, in order, in
one process — which is how the pipeline runs them, so the image memo sees what it
would really see.

The comparison is on the outcome (refused or not), the makespan, the priority
order it chose, the pause total, a hash of every arm's progress array and a hash
of every collision image built:

| | |
|---|---|
| conducts compared | **26** — every one of the run |
| refusals reproduced as refusals | 23 |
| certified conducts, same makespan AND same priority order | 3 |
| progress arrays compared, by hash | 18 |
| collision images compared, by hash | 45 |
| **conducts whose schedule differs** | **none** |
| replay wall clock | 9528.3 s -> 188.9 s (**50.4x**) |

A conduct that refuses is as much a result as one that certifies: the six passes
the split allocation spends before giving up have to give up for the same reason
in both builds, and they do.

`scene_check` is untouched by this pass and remains the independent gate: it
re-derives the swept geometry from the written programme and knows nothing about
`coordination`'s images, so an image made wrong is something it would catch.

## 9. What is in the way now

The conductor is no longer the run. On the Trollface the first certified
programme lands at 268.8 s, of which 172.8 s is the allocation and 95 s the
conduct — and inside that 95 s the largest single item is `scene_check`, at
36.1 s. The three levers that are left, in the order the measurement ranks them:

**The allocation is the run again, at 172.8 s of 268.8 s.** It moved 9.2x in the
previous pass and has not been touched since; it is now 64 % of the wall clock a
person waits. Whatever is next is mostly in §§1–5's territory, not §§6–7's.

**`scene_check` is 38 % of the conduct.** It re-samples 5203 scheduled steps to
10405 and tests 15 pairs, and `scene_check.pair_clearance` does it with no broad
phase whatsoever: all 49 capsule pairs, every sample, exact, broadcast over the
whole timeline in one call. That is not an oversight, it is the gate being
deliberately stupid — it is an independent implementation that shares nothing
with `coordination`, which is exactly what makes it worth having and exactly why
this pass did not touch it. The per-capsule box argument of §7.1 is exact and
would apply to it unchanged, and on this evidence would be worth most of those
36 seconds. But it is a change to the GATE, so it wants its own pass and its own
adversarial tests — and it must NOT become a helper shared with `coordination`,
because sharing code between the thing being checked and the thing checking it
is how a gate stops being independent.

**The priority search is now most of a warm `coordinate` call** — 2.1 s of 2.1 s
— because everything around it got cheap. Bounding it is available (§7.5) at the
cost of a determinism property; parallelising it across the 6 orders is available
without that cost, but 2 s of a 95 s conduct is not yet worth either.

Below those, `select_profile`'s daemonic pool costs the two parallel-conducted
profiles about 5x each (§8.2). It changes the grid wall clock and not the number
a person feels.
