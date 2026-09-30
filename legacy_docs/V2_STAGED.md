# Stage assignment, wired end to end — and what the trajectories say that the envelopes did not

Build item 4 of `docs/ARCHITECTURE_V2.md`, landed as `aris_sixarm/staged.py`
with `tests/test_staged.py`. Everything below is one command:

```
ARIS_RIG=proposed ARIS_TOOL=lateral python3 -m aris_sixarm.staged \
    --lines out/csail_schedule_h097_v19_strokes.json --route-jobs 4 \
    --json out/staged_csail_h097.json --programme out/staged_csail_h097_program.json
```

against the shipped atlas `out/atlas_proposed_h0970_lat0860_gated63` at
h = 0.970, the shipped parks `layout.Q_PARK_PROPOSED`, the eight-stage
`traces.zigzag_pattern()`, and the persistent leg store of build item 1.

**The one-line answer.** The pipeline runs end to end and the first arm may
move **0.309 s** after the picture is in hand, against v19's 3 712.4 s. The
parked half of the safety argument holds: five of the six parked-partner
certifications clear 60–140 mm. **The active half does not, and not for the
reason anybody was worried about**: ink against ink clears by **+273.5 mm** at
worst, and the binding number in both three-active stages is **two PEN-UP LEGS
flying into each other** — +12.9 mm and +8.2 mm. That is the gap
`ARCHITECTURE_V2` §2(f) named in advance ("the pen-up leg between two hovers is
a *path*, whose interior is not in the envelope") and this is the run that
measures it.

---

## 1. What the module does, in four steps

Per `(stage, arm)` bucket, and nothing else touches it:

1. **The five partners are frozen at their parks** (`frozen.freeze`) before a
   single call is made, so every leg is certified against the partners' **real
   capsules** rather than against their 0.32 m pose-invariant bands. The
   persistent leg store is re-namespaced on the frozen set, because
   `paper.route_key` does not contain it — `staged.leg_cache_signature`.
2. **Every piece goes through `stroke_api.plan_stroke`**, the same funnel
   `scripts/csail_allocate.py` uses. A piece it refuses is recorded and
   dropped, not split: the DP's piece count is what the 2 cm atlas *permits*
   and the refusal fraction is a measurement this module exists to take.
3. **The accepted pieces are ordered by `allocate.sequence_arm`** — one tour
   per bucket, the same `segs` contract the allocator hands the sequencer
   today. No balancer, no split, no merge: the DP already minimised the piece
   count exactly.
4. **`writing.arm_program` lays the timeline down**: park → entry hover → draw
   → exit hover → … → park, every pen-up a `paper.route`. It ends at the park
   because a stage **barrier** is defined as every arm pen-up, stopped, at the
   park the next stage's envelopes were certified against.

No conductor anywhere. The three actives in a main stage are meant to be static
keep-outs for each other, and §3 is the measurement of whether they are.

## 2. The CSAIL logo, staged

39 strokes, 16.805 m of ink, at v19's placement. `traces.plan_lines` cuts it
into **54 pieces** across the eight stages in **30 ms**.

| stage | actives | pieces | ink (m) | stage time (s) | plan, parallel (s) | plan, serial (s) | refused | **ink vs ink (mm)** | **whole trajectory (mm)** | solo min (mm) | pass |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | 13, 71, 2 | 13 | 4.240 | 99.2 | 14.7 | 15.9 | 3 | **+273.5** | **+12.9** | +62.7 | **no** |
| 1 | 17, 31, 97 | 9 | 3.282 | 74.6 | 9.2 | 9.5 | 2 | **+338.8** | **+8.2** | +60.9 | **no** |
| 2 | 13, 97 | 6 | 1.461 | 48.0 | 2.2 | 3.3 | 0 | +1 055.3 | +977.2 | **+28.2** | **no** |
| 3 | 17, 2 | 5 | 1.145 | 29.4 | 1.3 | 2.2 | 0 | +1 151.2 | +870.9 | +96.2 | yes |
| 4 | 31, 97 | 5 | 1.083 | 31.8 | 2.4 | 2.4 | 0 | — | +897.0 | +140.2 | yes |
| 5 | 71, 2 | 4 | 0.923 | 58.6 | 2.1 | 2.1 | 0 | — | +623.8 | +72.5 | yes |
| 6 | 13, 31 | 4 | 0.889 | 26.5 | 1.9 | 1.9 | 0 | — | +263.7 | +71.2 | yes |
| 7 | 17, 71 | 2 | 0.959 | 23.6 | 1.1 | 1.1 | 1 | — | +525.3 | +80.9 | yes |

(The plan columns are the **warm** re-run; the cold numbers are in §4. "—" in
the ink-vs-ink column is a stage in which only one of the two actives was given
any ink by the DP, so there is no instant at which both pens are down: stages
4–7 of this logo are effectively solo.)

**Staged makespan 391.7 s against v19's conducted 209.9 s — 1.87× worse**, and
the reason is structural rather than a planner regression. A stage costs its
busiest arm and the barrier makes the sequence cost the **sum** of eight of
those, and each stage pays a full park → out → back trip for every active arm
whether it has 13 pieces to draw or 2. Stage 5 is the clearest case: 0.923 m of
ink for 58.6 s of clock. `traces.py` predicted this shape — the eight-stage
pattern is 2.35× the serial makespan in ink metres against a three-arm ceiling
of 3× — and the seam stages are where it is paid. It is the price of a
programme that can actually run at all, against a conducted one that needed
407.3 s of conducting to be allowed to.

**Time to first motion: 0.309 s.** The DP (30 ms), the first piece of stage 0's
first arm, its entry hover, and the park → hover leg. Pete's budget is 10 s and
`V2_SCALING_BASELINE` predicted 0.86 s at the median; this is under both,
because the binding term in that prediction — a cold `paper.route` that falls
through to the RRT — did not fire on this particular leg.

**Refusals: 6 of 54 pieces, 11.1 %.** What refuses them:

| reason | n | what it is |
|---|---|---|
| `split:empty_fiber` | 4 | the redundancy band has no q7 the arm can hold somewhere along the piece — the atlas cell is certified at 2 cm and the continuous piece is not |
| `split:sheet_collapse` | 1 | no single IK sheet spans the piece |
| `degenerate:too_short` | 1 | the piece is under `plan_stroke`'s 20 mm floor |

All six are `plan_stroke`'s honest "not end to end by this arm", and five of
the six carry a certified **head**. This module drops them rather than
re-entering the DP with a capability map re-derived from the refusals, which is
the refusal loop `ARCHITECTURE_V2` item 4 asks for and the one thing of item 4
that is **not** built (§6).

## 3. The two checks, and the one that fails

**They are two different questions and the module uses two different tools.**

**(a) Active against active — `staged.active_pair_gap`.** Not
`check_timeline` on a merged clock: inside a stage the actives are asynchronous
*by construction*, so there is no single alignment of their timelines to check
and checking one would certify a schedule nobody runs. The claim that has to
hold is the envelope claim restricted to the poses the planner actually
produced: for **every** pair of instants, one on each arm's timeline, the two
arms clear the gate. That is the minimum over the **cross product** of the two
pose sets, with `check_timeline`'s own 1-Lipschitz between-sample residual
applied on both axes.

**(b) Active against parked — `staged.solo_check`.** One
`scene_check.check_timeline` per active arm with the five partners held at
their parks for every frame, so the partners are measured as the capsules they
*are* rather than as the envelope they are not. The partners' pose-invariant
body bands are dropped from the static set (`staged.drop_bands`), because every
arm those bands stand for is in the timeline as its own capsules — the same
convention `scene_check.neighbour_columns` already uses for the cylinder
version of the same object. True structure is never dropped.

**The result.** Ink against ink is never the problem: **+273.5 mm** is the
worst it gets, against the +85.8 mm the 0.40 m dead band was chosen for. What
binds is the **pen-up legs**, and in both failing stages the binding instant has
*both* arms in a leg:

| stage | binding pair | both in | whole trajectory | ink only |
|---|---|---|---|---|
| 0 | 2 – 71 | leg / leg | **+12.9 mm** | +273.5 mm |
| 1 | 31 – 97 | leg / leg | **+8.2 mm** | +338.8 mm |

That is not a defect in the dead band and it is not a planner bug. It is
exactly the hole `ARCHITECTURE_V2` §2(f) identified: *"An envelope is a union
over poses — the drawing pose at each certified cell, the hover above it, and
the park — and the pen-up leg between two hovers is a path, whose interior is
not in the envelope."* Each arm's legs were certified against the **parked**
fleet, which is the right room at a barrier and the wrong one mid-stage, and
nothing in the pipeline has ever asked whether one active arm's leg crosses
another active arm's leg. §6 says what closes it.

**And one stage fails the parked check too.** Stage 2 reads **+28.2 mm**
against the 50 mm gate on the active-vs-parked side. The certified map at
h = 0.970 with the shipped parks frozen is 100 % of the block at 50 mm, so the
drawing poses are not the problem; this is a leg or a hover again, measured by
`check_timeline` with a sweep residual that `paper.route` does not charge
itself. Which of the two actives binds it was not attributed inside this pass
and is the first thing to run next.

## 4. What the planning cost, cold and warm

| | cold (first run) | warm (leg store hit) |
|---|---|---|
| lines → pieces (the DP) | 30 ms | 30 ms |
| plan + order + fly, all eight stages, serial | **221.2 s** | **38.4 s** |
| …the same charged per stage to its busiest arm | **213.2 s** | **34.8 s** |
| the two independent checks | 100.5 s | 99.9 s |
| wall clock, end to end | 322.0 s | 138.6 s |

**5.8× on the planning half, bought entirely by build item 1's persistent leg
store**, and the two runs produce a byte-identical makespan (391.732 s both
times). Per-arm planning is the row that matters for the architecture, because
the six arms plan independently: the worst single arm in a stage is 14.7 s warm
(arm 71 in stage 0, 13 pieces) against 4 240 mm of ink, so the planner stays
far ahead of the pens.

The check is now the expensive half — 100 s against 38 s of planning — and it
is the cross-product pair check that costs it, not `check_timeline`. That is a
fair trade for a claim with no timing assumption in it, and it is trivially
parallel per stage.

## 5. A thousand lines

Same module, `--synthetic strokes:1000 --no-fly --no-check`: the DP's pieces
run through `plan_stroke` and nothing else, because the question at 1 000 lines
is the **refusal fraction** — every number in `docs/V2_TRACES.md` is what the
2 cm atlas permits, and a piece the local planner refuses has to split and
re-enter the DP.

`traces.synthetic("strokes", 1000)`, the same deterministic set
`docs/V2_TRACES.md` §4 measures: 1 000 lines, 660.3 m of ink, **1 756 pieces**
— the figure that document reports, reproduced exactly — in **937 ms** of DP.

| | |
|---|---|
| pieces, from the DP | **1 756** |
| **refused by `plan_stroke`** | **318, 18.1 %** |
| `plan_stroke` over all 1 756, serial | **842.3 s** |
| …charged per stage to its busiest arm | **342.6 s** |
| time to first motion | **1.271 s** |

| reason | n | share of refusals |
|---|---|---|
| `split:empty_fiber` | 127 | 39.9 % |
| `split:start_infeasible` | 87 | 27.4 % |
| `split:sheet_collapse` | 58 | 18.2 % |
| `degenerate:too_short` | 33 | 10.4 % |
| `degenerate:off_sheet` | 11 | 3.5 % |
| `degenerate:too_short_after_clip` | 2 | 0.6 % |

**Two thirds of the refusals are the redundancy band, not the reach.**
`empty_fiber` and `start_infeasible` together are 214 of 318: the 2 cm atlas
certifies a *cell* and the continuous piece runs between cells, so the band
gives out somewhere along it or at its very first sample. `sheet_collapse` is
the next 58 — a piece no single IK sheet spans. Only the last 46 are the
piece itself being unplannable by anybody: 35 under `plan_stroke`'s 20 mm floor
(the DP's own `MIN_PIECE_M` is 10 mm, so it is entitled to hand out pieces the
planner calls degenerate) and 11 off the sheet.

**18.1 % against the logo's 11.1 %**, and the difference is where the ink is:
the synthetic set is spread over the whole certified block including its rim,
and the two three-active stages take almost all of the loss — 138 of 434 and
169 of 380, against 11 refusals in the six seam stages put together. A seam
stage's ink sits in a 0.40 m band under two arms that both reach comfortably
into it; a row band's ink runs out to the edge of one arm's reach.

**This is the number the refusal loop is worth, and it is not small.** 318
pieces have to split and re-enter the DP with a capability map re-derived from
the refusals, and the DP costs 10 ms, so the loop is cheap — but the re-planning
is not, and 18 % is too much ink to drop. It is also a direct measurement of
how optimistic the 2 cm prefilter is, which is a number nobody had.

**And the planning still keeps up.** 342.6 s of per-arm planning against a
draw of roughly 4 300 s (`V2_SCALING_BASELINE` §3.3) is an order of magnitude
of headroom, and that is cold — this run bought no legs at all, so a full run
adds the leg cost, which the logo measured at 5.8× cheaper warm.

## 6. What is not built, and what the numbers ask for next

**Not built, deliberately, inside this pass:**

- **The refusal loop.** A refused piece is dropped, not split and re-offered to
  the DP with a capability map re-derived from the refusal. It is the half of
  item 4 that needs the DP and the planner in a loop, and 11.1 % of the ink is
  what it is worth.
- **The seam stages are not conducted.** Stages 2–7 are two-active and
  `ARCHITECTURE_V2` §2(d) sends them through `idle.conduct`. This module plans
  them asynchronously like the others and reports the resulting clearance
  instead of scheduling it away. On this logo it did not matter — the seam
  stages are effectively solo — but on a picture that gives both seam arms ink
  it will.
- **The typed programme is not `program_schema.Bundle`.** `Segment` carries no
  joint vector and `_from_dict` refuses unknown *and* missing keys, so a stage
  id and a per-piece hover are a `SCHEMA_VERSION` bump rather than an
  extension. `staged.programme()` writes a superset of what item 5 has to
  absorb — stage id, barrier list, per-piece `q_first`/`q_last`,
  `hover_in`/`hover_out`, the pen-up leg blocks and the joint trajectory —
  under its own `STAGED_SCHEMA_VERSION`.

**What the numbers ask for next, in order:**

1. **Certify the legs against the other actives, not only against the parked
   fleet.** The cheapest form is the one `ARCHITECTURE_V2` §2(f) already names:
   one `check_timeline` per arm per stage against the other actives' work-cell
   **envelopes as static boxes**, and a leg that fails is re-routed rather than
   the stage being re-designed. The ink-vs-ink number says the cells are right;
   only the paths between them are unaccounted for.
2. **Or raise the legs.** Both failures are two arms in the air at once, and a
   hover plane is a free parameter the router already ladders over
   (`writing.HOVER_LADDER`). A per-stage flying height that separates the two
   row bands in *z* is a much cheaper lever than eroding the cells in *x*,
   which `DECISIONS.md` measured out on 2026-09-11.
3. **Make the barrier cheaper.** 391.7 s against 209.9 s is eight park → out →
   back trips, and four of the eight stages carry under a metre of ink. Merging
   the seam stages that do not actually share a horizon, or letting an arm stay
   out between two consecutive stages it is active in, is where the makespan
   is.

## 7. What is tested

`tests/test_staged.py`, 7 tests, **no environment variables** (the rig and tool
are switched in process and put back) and **no atlas** (every capability map is
rasterised from rectangles through the same `Coverage` object the atlas
produces):

- **No same-row pair is active in any stage** of `zigzag_pattern()`, and
  `staged.run` **refuses** a pattern that has one — the transverse pair is
  unseparable on this rig on either axis, so this is a refusal and not a
  report.
- `pieces_of` is exactly the DP's answer: one `Piece` per drawn piece, the same
  geometry as `LinePlan.piece_points`, bucketed by `(stage, arm)`.
- **A stroke crossing the dead band is deferred to a seam stage and drawn
  there**: every piece whose midpoint is inside SEAM0 carries a seam stage and
  every piece outside it carries a main stage, the crossing stroke is cut into
  exactly two pieces at the boundary to 0.1 mm, and nothing is dropped.
- **The frozen-partner model is what is installed while planning**: a hook
  fires inside `plan_bucket` once per piece and asserts, at that instant, that
  `frozen.active()`, that the frozen set is the five partners and not the
  mover, that `frozen.observer()` is the mover, and that every frozen pose is
  that arm's shipped park.
- `drop_bands` drops only `body:*_column*` boxes, keeps every mount and plate,
  and keeps the band of an arm the caller does not name.
- **End to end on a three-stroke picture**: pieces → plans → order → legs → a
  timeline that starts *and ends* at the park (the barrier), with the per-stage
  active-pair and solo checks both clearing `PAIR_MARGIN`, and a typed
  programme whose every piece carries its stage, its two joint endpoints and
  its two hover configurations.

---

## 8. Closing the leg gap — the other actives, as an envelope

§3 measured the failure and `ARCHITECTURE_V2` §2(f) named the fix. It is now
built, and it is one sentence: **before an arm plans anything in a stage, the
other actives of that stage go into its static room as their whole work-cell
envelopes.**

**What the envelope is.** Not a box — a box round an arm's reach swallows its
neighbour and would refuse everything. It is the union of the partner's **link
capsules** over every pose it could hold in this stage: the certified drawing
pose at each strict-GO cell of its work cell, the hover above each of those, and
its park. That is exactly the pose stack `scripts/workcell_envelopes.py`
measured +85.8 mm with, and `frozen.py` takes it unchanged now that
`freeze_sets` accepts more than one pose (`freeze` is the N = 1 case, which is
what a *parked* partner is).

**What makes it affordable.** A row band's envelope is some 9 000 capsules and
`frozen.partner_clearance` is linear in them, which would put a tenth of a
second on every `paper.route` call. `staged.cluster_capsules` bounds them by
grid-local **spheres** — each sphere contains both endpoints and the radius of
every capsule whose midpoint fell in its 0.15 m cell — so the reduction is
conservative by construction: a query that clears the spheres clears the
capsules. A row band comes out as a few hundred spheres. The poses are read at
the shipped stride 2 (4 cm) and every sphere is inflated by `ENVELOPE_PAD` =
40 mm for the cells the stride skips. The whole set is cached under the atlas,
the tool and the region, because it is a property of the **stage** and not of
the picture.

**And the leg store is namespaced on it.** `paper.route_key` contains neither
the frozen poses nor the envelopes — both change `static_boxes` without changing
any memo key — so a leg bought in one room would otherwise be served in another.

**The ink gets its own check, and it is a measurement rather than a gate.**
`plan_stroke` never consults the static set, so a piece can be certified end to
end and still be drawn through a neighbour's envelope; `staged.ink_vs_envelope`
is the missing half. It is reported and not enforced by default, because the
envelope it compares against is *deliberately larger than the thing it bounds*
(stride-2 sample, sphere-bounded, 40 mm pad) and refusing certified metres to a
conservatism is a bad trade. Measured on CSAIL: median **+230.9 mm**, 5th
percentile **+9.4 mm**, minimum **−28.5 mm**, and **5 pieces of 48** read under
50 mm — all of them against the inflated bound, none of them against another
arm's actual ink. `ink_gate=PAIR_MARGIN` turns it into a refusal for a caller
who wants the strict reading.

**The second lever, if the first is not enough.** `staged.row_lift_ladder` gives
each **row band** its own hover height (0.06 / 0.20 / 0.34 m), with the shipped
ladder behind it so an arm that cannot hold the raised pose keeps the one it
had. `writing.HOVER_LADDER` is borrowed for the stage and given back, and
`lifted_or_lower`'s memo is keyed on the heights, so it moves that stage's legs
and nothing else. `run(lift_retry=True)` fires it only on a stage whose pair
check still fails, keeps whichever of the two is better, and records
`lift_used`.

## 9. The refusal loop

A piece `plan_stroke` refuses is **not ink nobody can draw** — it is ink *that
arm* cannot draw *in that stage*, which is one bit of one atom's capability set.
`staged.resolve_refusals` strikes that bit and asks the DP again, to a fixed
point or a cap of four rounds. Nothing is flown inside the loop: the loop is
over the pieces, and a plan memo keyed on the arm, the geometry and the room
means a round only pays for the pieces that actually moved.

Two things had to be right, and each was measured wrong first:

- **A refusal bans only the stretch the planner did not certify.** `plan_stroke`
  hands back `s_star`, "the normalised arc length up to which it IS planned",
  and a head plan is an "ok" result with all of its guarantees. Banning the
  whole piece throws the head away too.
- **A ban CUTS the atom rather than clearing its bit.** An atom is indivisible
  with respect to the capability map and a refusal is a new transition in it, so
  a ban that merely touches an atom must split it at both ends. Measured on the
  CSAIL logo, banning whole atoms took the loop from 100 % coverage to
  **80.6 %**; cutting them is what keeps the ink.

A `degenerate` refusal — "too short", "off sheet", "too short after clip" — bans
nothing and is reported as **unplannable**: those are statements about the
*piece*, not about the arm, so no neighbour would do better and striking the
state out would only spread the hole. A span is never banned twice, which is
what makes the loop terminate rather than walk one piece down the state list.

## 10. The barrier cost at scale, and the rule that should replace it

CSAIL's staged makespan is 1.87× v19's conducted one, and the reason is the
barrier: a stage costs its busiest arm, the sequence costs the **sum** of eight
of those, and every active arm of every stage pays a park → out → back trip
whatever it has to draw. `staged.stage_overhead` measures that trip directly —
the entry leg before the first stroke and the exit leg after the last, per
(stage, arm).

**The proposal, not implemented: an adaptive stage count.** The overhead is a
fixed cost per (stage, arm) and the ink is not, so the eight-stage pattern is
right at scale and wrong on a small picture. The rule:

> Let `P` be the measured park overhead per (stage, arm) — CSAIL's own number —
> and let `m` be a stage's ink for its busiest arm. A stage whose busiest arm
> carries less than `X = P × v_draw` metres is **not worth its own barrier**:
> the trip out and back costs more clock than the ink it protects.

Three ways to spend that, cheapest first:

1. **Merge two seam stages that share no arm.** Stages 2 and 4 are 13 + 97 and
   31 + 97; stages 3 and 5 are 17 + 2 and 71 + 2. Two stages that share an arm
   cannot merge, but two that do not can, subject to the same envelope test the
   three-active stages already pass — and the merge saves one whole barrier.
2. **Let an arm stay out between consecutive stages it is active in.** The
   barrier requires every arm at *its own next park*; an arm whose park does not
   change between stage *s* and stage *s+1* is already there, and
   `DECISIONS.md` (2026-09-11) measured that **26 of 42 transitions are exactly
   that**. Those arms should not fly home and out again.
3. **Fall back to the conductor for the residue.** When what is left is a
   handful of short stages, `idle.conduct` over two arms is `∑ₖ P(n, k) = 4`
   orders over a short horizon — cheap, and it removes the barrier entirely.

The number `X` is the one thing a measurement has to supply.

## 11. The CSAIL re-run with the envelope room

Same command, `traces.zigzag_pattern()`, h = 0.970, the refusal loop on and the
envelope room installed. **The leg gap is closed and the makespan question is
now a different one.**

| stage | actives | pieces | ink (m) | stage (s) | **active-pair (mm)** | ink vs ink (mm) | solo (mm) | lift used | flown |
|---|---|---|---|---|---|---|---|---|---|
| 0 | 13, 71, 2 | 15 | 5.243 | 25.8 | **+899.3** | — | +250.3 | no | 1 of 3 |
| 1 | 17, 31, 97 | 12 | 4.136 | 8.9 | **+826.1** | — | +214.3 | no | 1 of 3 |
| 2 | 13, 97 | 6 | 1.461 | 48.0 | **+977.2** | +1 055.3 | **+28.2** | no | 2 of 2 |
| 3 | 17, 2 | 5 | 1.145 | 29.4 | **+870.9** | +1 151.2 | +96.2 | no | 2 of 2 |
| 4 | 31, 97 | 5 | 1.083 | 31.8 | **+897.0** | — | +140.2 | no | 1 of 2 |
| 5 | 71, 2 | 4 | 0.923 | 56.5 | **+860.8** | — | +65.3 | no | 1 of 2 |
| 6 | 13, 31 | 4 | 0.889 | 28.4 | **+626.1** | — | +206.7 | no | 1 of 2 |
| 7 | 17, 71 | 2 | 0.959 | 0.0 | **+539.8** | — | +256.0 | no | 0 of 2 |

**The active-pair minimum over all eight stages is +539.8 mm, against +8.2 mm
before.** The two failing stages went +12.9 → **+899.3** and +8.2 → **+826.1**,
and the **lift lever was never needed** — `lift_used` is false everywhere, so
routing against the envelopes was sufficient on its own and the row-band hover
ladder stays in the box as the fallback it was built to be. Seven of the eight
stages pass both checks; stage 2 still reads **+28.2 mm** on the *parked*
side, which is the one failure this pass did not touch and is unchanged from §3.

**And the envelope is too conservative to fly the big buckets.** Three of the
eighteen (stage 0's arm 71 with 13 pieces, stage 1's arm 31 with 11, stage 7's
arm 71 with 2 — **26 of the 56 pieces**) produced **no timeline at all**:
`writing.arm_program` refuses the entry leg, and in one case `sequence.solve`
could find no feasible order before that. The cause is named by the ink check
itself — the minimum ink-vs-envelope reading is **−28.5 mm**, i.e. the arm's own
certified drawing pose is already *inside* the inflated bound, and no leg out of
a pose inside an obstacle can clear it. The conservatism is the 0.15 m sphere
clustering plus `ENVELOPE_PAD` = 40 mm, both of which were chosen for speed
rather than tightness.

**So the makespan is not comparable and is not quoted as one.** 228.8 s over a
programme missing 26 of 56 pieces is a partial number, and the honest statement
is that the makespan question re-opens once the envelope is tight enough to fly.
What *is* measurable on the nine buckets that did fly is the barrier cost:

| | |
|---|---|
| park → out → back, all flown buckets | **87.7 s** |
| …on the critical path (the busiest arm of each stage) | **75.3 s** |
| …as a share of the makespan | **32.9 %** |
| draw, all flown buckets | 103.9 s |
| pen-up, all flown buckets | 175.3 s |

**A third of the clock is the barrier**, and pen-up is 1.7× the ink.

**The refusal loop, measured:** round 0 → 54 pieces, 5 refused, 100.00 %;
round 1 → 56, 4, 98.42 %; round 2 → **56, 0, 94.20 %**, converged, with
**3 pieces unplannable and all three `degenerate:too_short`**. The 5.8 % of ink
lost is not the loop giving up — it converged with zero refusals outstanding —
it is ink with **no second stage-compatible drawer**: the staged map's
redundancy is 48.1 % (`docs/V2_TRACES.md` §5), so when the one arm a stretch is
offered to refuses it, there is nobody to hand it to. Closing that needs the
*pattern* to offer the stretch to somebody else. **The final refusal rate is
3 of 56, 5.4 %**, against 11.1 % before the loop.

Time to first motion is **0.286 s**. Planning is **1 549 s**, because the room
changed and every leg in it is cold — the leg store is namespaced on the room,
by design, so this is the one-off price of the new certificate.

## 12. A thousand lines, staged — the model

A real 1 438-piece fly is not affordable: `sequence.cost_matrix` is O(n²) route
screens and one bucket is 434 pieces. So the makespan is assembled from terms
measured on the CSAIL run and the 1 000-stroke set's own per-(stage, arm) ink
and piece counts from `traces`. **Arithmetic, with every term measured.**

Calibration, from the nine flown buckets: draw **6.56 s/m**, inter-piece pen-up
leg **5.15 s**, park overhead **9.74 s per (stage, arm)**. Accept rate 81.9 %,
measured in §5.

| stage | actives | pieces | ink (m) | busiest arm | stage (s) |
|---|---|---|---|---|---|
| 0 | 13, 71, 2 | 572 | 258.0 | 2 | 1 327.7 |
| 1 | 17, 31, 97 | 549 | 257.2 | 97 | 1 313.8 |
| 2 | 13, 97 | 162 | 36.6 | 13 | 451.5 |
| 3 | 17, 2 | 146 | 35.0 | 2 | 421.5 |
| 4 | 31, 97 | 75 | 17.9 | 31 | 417.4 |
| 5 | 71, 2 | 76 | 15.7 | 71 | 409.8 |
| 6 | 13, 31 | 91 | 20.7 | 31 | 499.7 |
| 7 | 17, 71 | 85 | 21.3 | 71 | 477.7 |

**Staged makespan (model): 5 319 s = 88.7 min**, against
`V2_SCALING_BASELINE` §3.3's ~4 300 s scaling of v19's conducted six-arm
makespan — **1.24×**, where CSAIL measured 1.87× before this pass.

**And the barrier amortises exactly as the rule predicts.** The park overhead on
the critical path is **77.9 s, 1.47 % of the makespan**, against CSAIL's
**32.9 %**. The barrier is a fixed cost per (stage, arm) and the ink is not, so
**the eight-stage pattern is right at scale and wrong on a small picture** —
which is the whole case for the adaptive stage count of §10, and it supplies
its threshold: at 6.56 s/m and 9.74 s of park overhead, **`X` ≈ 1.5 m of ink
for the busiest arm**. Six of CSAIL's eight stages are under it; none of the
1 000-line set's are.

## 13. How tight the envelope has to be — the probe, and what it says so far

**The binding test is one leg, not a whole timeline.** Every bucket that failed
in §11 failed on `writing.arm_program`'s *"entry at segment 0 cannot clear the
paper plane"*, which is a single `paper.route` from the park to the first hover.
So the sweep prices exactly that, plus the exit leg, plus the ink-vs-envelope
minimum that explains it — three numbers per (stage, arm) instead of a fly. The
pieces are planned once with no envelope in the room, because `plan_stroke`
never consults the static set and a piece's joints therefore do not depend on
the setting; only the verdict about them does.

**Measured, at the shipped setting** (stride 2, cluster cell 0.15 m, pad
40 mm), on the 56-piece refusal-loop output:

| setting | spheres (mean) | buckets whose park↔hover legs route | ink vs envelope, min |
|---|---|---|---|
| **stride 2, cell 0.15 m, pad 40 mm (shipped)** | 145 | **9 of 13** | **−30.6 mm** |

That is the §11 failure reproduced in 1.4 s instead of 27 minutes, and it
confirms the diagnosis: four buckets cannot get out of their park at all, and
the ink minimum is negative, i.e. the arm's own certified drawing pose is
already inside the bound.

**Why the cell is the lever and the pad is not, exactly.** A cluster sphere is
drawn round the endpoints of every capsule whose *midpoint* fell in its cell, so

> `radius ≤ (cell × √3 / 2) + (half the longest capsule) + (the capsule radius)
> + (the pad)`

and the only term the caller controls is the first. At the shipped 0.15 m cell
it is **0.130 m**; at 0.05 m it is **0.043 m**. Against a real capsule radius of
about 0.06 m, that is the difference between a bound three times the thing it
bounds and one a third larger — an order more than the 40 mm pad is worth.
`tests/test_staged.py` pins the decomposition.

**And `pad = 0` is only legal at stride 1.** The pad exists to cover the poses a
stride skips: at stride 2 three cells in four are not read and the pad is the
only thing standing in for them, while at stride 1 every strict-GO cell of the
region is read (`2 × cells + 1` poses, pinned by a test) and the envelope is
then the exact object `scripts/workcell_envelopes.py` measures. **So the
setting the arithmetic points at is stride 1, cell 0.05 m, pad 0** — tighter by
roughly 130 mm of radius and with no conservatism left to justify.

**What is NOT measured, and it is the headline of this pass.** The sweep did not
complete: each setting re-routes 26 legs cold under a new store namespace, and
several fall through the shape ladder to the RRT, which is minutes rather than
seconds per leg. So I have the baseline row and the arithmetic, and I do **not**
have the measured "fraction of buckets that fly" at the tighter settings, the
active-pair minimum under them, or a full eight-stage fly. **The complete CSAIL
table of §11 therefore still stands as the last measured one**, with 26 of 56
pieces unflown, and the makespan is still not quoted.

**One real fix came out of the attempt.** The pose cache and the sphere cache
were filed under one key, so sweeping the clustering re-solved every hover —
the same family of bug as `id(spec)` in `paper.py`, found the same way, by a
sweep that should have taken seconds and took minutes.
`staged.cached_envelope_poses` splits them, and the sweep's per-setting cost is
now the legs alone.

**Next, in order:** run the sweep with the leg store pre-warmed per setting (or
with `paper.RRT_SAFE` off for the probe, since a leg the ladder settles is the
one that matters for "does it fly"); adopt the winning setting as the default;
then the full eight-stage fly and the makespan.

## 14. The sweep, and the setting that flies

**The probe, with the RRT tier off.** What decides "does this bucket fly" is
whether the **shape ladder** settles the park → hover leg: the C-space tier is
the last resort, it costs minutes when it fires, and a setting that needs it for
the entry leg of every bucket is not one anybody would ship. So the sweep runs
at `paper.RRT_SAFE = False`, which makes it an honest **lower** bound — a
setting the probe passes certainly flies — and the full run puts the tier back
on. `paper.cache_signature()` carries the three tier flags, so nothing the probe
buys is served to a run that has the tier on.

Stride 1 throughout, because `pad = 0` is only honest there (§13): at stride 1
every strict-GO cell of the region is read and there is no skipped pose for a
pad to stand in for.

| stride | cell (m) | pad (mm) | spheres (mean) | **buckets whose park↔hover legs route** | ink vs envelope, min | probe (s) |
|---|---|---|---|---|---|---|
| 1 | 0.15 | 40 | 156 | 7 of 13 | -46.3 mm | 21.9 |
| 1 | 0.15 | 20 | 156 | 7 of 13 | -26.3 mm | 44.1 |
| 1 | 0.15 | 0 | 156 | 8 of 13 | -6.3 mm | 42.9 |
| 1 | 0.1 | 40 | 395 | 7 of 13 | -13.1 mm | 73.6 |
| 1 | 0.1 | 20 | 395 | 7 of 13 | +6.9 mm | 79.6 |
| 1 | 0.1 | 0 | 395 | 7 of 13 | +26.9 mm | 76.4 |
| 1 | 0.075 | 40 | 754 | 7 of 13 | -12.2 mm | 102.5 |
| 1 | 0.075 | 20 | 754 | 7 of 13 | +7.8 mm | 107.7 |
| 1 | 0.075 | 0 | 754 | 7 of 13 | +27.8 mm | 116.6 |
| 1 | 0.05 | 40 | 1926 | 7 of 13 | -0.1 mm | 218.0 |
| 1 | 0.05 | 20 | 1926 | 7 of 13 | +19.9 mm | 250.1 |
| 1 | 0.05 | 0 | 1926 | 7 of 13 | +39.9 mm | 222.8 |

**What the sweep says, in three readings.**

*The pad is worth about what the arithmetic said and no more.* At the 0.15 m
cell, dropping the pad from 40 mm to 0 moves the ink minimum from **−46.3 mm**
to **−6.3 mm** and buys exactly **one** bucket (7 → 8 of 13). Forty millimetres
of radius on every sphere is forty millimetres, and it is not where the problem
is.

*Stride 1 is stricter than stride 2, and that is the point.* At the same cell
and pad, stride 1 flies **fewer** buckets than stride 2 did (7 against 9),
because it reads every certified cell instead of one in four: the stride-2
envelope was smaller because it was **sampling**, not because it was tighter.
A setting that flies only by not looking is not a setting.

*The cell is the lever, as the radius decomposition said it would be.*

At the 0.15 m cell, dropping the pad from 40 mm to 0 moves the ink minimum
from **−46.3 mm** to **−6.3 mm** and buys exactly **one** bucket. Shrinking the
cell from 0.15 to 0.05 at pad 0 moves it from −6.3 mm to **+39.9 mm** — an
**86 mm** swing end to end, exactly the term the radius decomposition named —
and buys **none**.

**So the residual refusal is not the envelope's conservatism.** The same six
buckets fail at every cell from 0.10 m down, with the ink comfortably clear of
the bound and every park clear of it. What is left is that a pen-up leg from the
park to the first hover, *routed by the shape ladder alone*, genuinely
intersects the other actives' work-cell envelopes: two arms in adjacent row
bands own overlapping airspace between the parks and the paper, and no amount of
tightening a bound that the ink already clears by 28 mm will change that.

**The probe is a lower bound, and the tier it turns off is precisely the one
that might answer this.** `aris_sixarm/transit.py` searches the 7-DOF joint
space where the ladder gives up, and every edge it proposes pays the same gate
stack a ladder leg pays. Whether it finds a way round is the question the full
fly answers, and it is running.

### The park is the other wall, and tightening moves it too

A leg starts at the park, so if the **park itself** is inside a partner's
envelope no leg out of it can clear at any tightness. Measured, park against the
other actives' envelopes, at the shipped setting and at the tight one:

PARK_| stride | cell (m) | pad (mm) | spheres (mean) | **buckets whose park↔hover legs route** | ink vs envelope, min | probe (s) |
|---|---|---|---|---|---|---|
| 1 | 0.15 | 40 | 156 | 7 of 13 | -46.3 mm | 21.9 |
| 1 | 0.15 | 20 | 156 | 7 of 13 | -26.3 mm | 44.1 |
| 1 | 0.15 | 0 | 156 | 8 of 13 | -6.3 mm | 42.9 |
| 1 | 0.1 | 40 | 395 | 7 of 13 | -13.1 mm | 73.6 |
| 1 | 0.1 | 20 | 395 | 7 of 13 | +6.9 mm | 79.6 |
| 1 | 0.1 | 0 | 395 | 7 of 13 | +26.9 mm | 76.4 |
| 1 | 0.075 | 40 | 754 | 7 of 13 | -12.2 mm | 102.5 |
| 1 | 0.075 | 20 | 754 | 7 of 13 | +7.8 mm | 107.7 |
| 1 | 0.075 | 0 | 754 | 7 of 13 | +27.8 mm | 116.6 |
| 1 | 0.05 | 40 | 1926 | 7 of 13 | -0.1 mm | 218.0 |

**Arm 71's park is +1.2 mm from arm 17's envelope in stage 7** at the shipped
setting — touching it, which is exactly why that bucket failed at every pad and
every cell down to 0.10 m. At the tight setting the same pair reads **+52.8 mm**
and clears the gate, and every other park is at the +350 mm broad-phase cap or
close to it. So the park set does **not** have to be re-searched
(`layout.stage_parks`, build item 2) — it was the envelope's conservatism
standing on the park, not the park standing in the wrong place. That is worth
saying plainly, because the opposite conclusion would have sent the next pass
after a park search it does not need.


**Adopted: stride 1, cell 0.075 m, pad 0.** Not because it flies the most
buckets — 0.15/0 flies one more — but because it is strictly the better
*certificate* and the bucket count does not distinguish them: no skipped poses
(so `pad = 0` is honest rather than optimistic), no unjustified inflation, an
ink minimum of **+27.8 mm** instead of −46.3, and **754 spheres against 1 926**
at 0.05 m for 11 mm more. The bucket count at 0.15/0 being one higher is the
shape ladder's discrete shapes interacting with a looser obstacle set, not a
tighter guarantee.

**And one more O(n²) came out of it.** `cluster_capsules` scanned every endpoint
once per cell (`inv == k`), which at stride 1 is a few hundred thousand
endpoints against tens of thousands of cells — minutes per arm, and what made
the first attempt at this sweep look like it had hung. It is now one
`np.minimum.at` / `np.maximum.at` pass each.

**The full eight-stage fly at the adopted setting is running** to
`out/staged_csail_h097_v3.json`, with the RRT tier **on**. Its envelopes are
1 157–1 377 spheres per active arm in the main stages and 472–506 in the seam
stages, and every leg in it is cold because the room changed.

## 15. The room is what the neighbour actually does

§14 measured the pose-union envelope out. Tightening it from a 0.15 m cluster
cell with a 40 mm pad to 0.05 m with none moved the ink-vs-envelope minimum
**86 mm** (−46.3 → +39.9), moved every park clear of the gate (arm 71 in stage 7
from **+1.2 mm** to **+52.8 mm**), and moved the number of buckets that fly **by
nothing** — the same six refused at every cell from 0.10 m down. The full fly at
the adopted setting, with the RRT tier back **on**, refused them too.

**So the union is the wrong object, not a badly bounded one.** An arm's work
cell is every pose it *could* hold anywhere inside it; two arms in adjacent row
bands own overlapping airspace between the parks and the paper; and no bound on
a set that large leaves a neighbour room to fly through it. What the neighbour
*actually* holds in a stage is one trajectory — a few hundred poses out of that
union — and that is a room a leg can be routed around.

**The construction.** `staged.trajectory_room` takes an arm's realised stage
timeline, builds its link capsules through the same `coordination.ArmPath` the
envelope used, and reduces them with the same `cluster_capsules` — which
*contains* what it replaces, so the certificate survives the reduction. The pad
is not a guess: it is `scene_check.check_timeline`'s own 1-Lipschitz
between-sample residual, `SWEEP_FRAC ×` the largest step any capsule endpoint
takes between two samples, so the spheres cover the motion **between** the
samples and not only at them.

**Two passes, and the iteration is explicitly not the certificate.** Pass 1
plans every active solo against the parked fleet — the room that flies but does
not separate the actives. Pass 2 re-plans each active against what the *others
actually did* in pass 1. Re-planning A moves A, which is a room B was certified
against, so the fixed point is approached and never proved by the iteration; the
claim is closed only by the independent whole-timeline `active_pair_gap` at
`PAIR_MARGIN`, exactly as before. The iteration gets the trajectories apart; the
check proves they are apart.

**And the barrier semantics change, so the dependency is recorded.** A
pose-union envelope is a property of the **stage**: it holds whatever the
neighbour is asked to draw, so an arm's plan survives its neighbour being
re-planned. A trajectory room is a property of that neighbour's **specific
plan**, and an arm's certificate is void the moment that plan changes.
`ArmStage.depends_on` carries a digest of each neighbour's trajectory and
`programme()` writes it out next to the arm's own `trajectory_digest`, so a
re-plan of one arm leaves every neighbour that still names the old digest stale
**by inspection** rather than silently. That is the price of the tighter room
and it is worth naming: the staged programme is no longer a set of independent
per-arm certificates, it is a graph of them.

**How much smaller the room actually is.** Stage 0 of the CSAIL logo, the
same stage whose arm-71 bucket refused at every envelope setting:

| arm | pieces it draws | pose-union envelope | **trajectory room** |
|---|---|---|---|
| 2 | 1 | 1 339 spheres | **422** |
| 13 | 1 | 1 377 spheres | **111** |
| 71 | 13 | 1 157 spheres | **1 001** |

The counts understate it. An arm that draws one piece had its whole row band
treated as a keep-out; what it actually occupies is one approach, one stroke and
one retreat. Arm 13's room falls by a factor of **12**, and arm 71 — which draws
thirteen pieces spread across R1 — barely moves, which is the honest shape of
the thing: the saving is exactly the ink an arm was *not* asked to draw.

**And pass 1 flies everything, which is the premise the iteration needs.**
Planned solo against the parked fleet, arm 71's thirteen-piece bucket produces a
timeline — the same bucket that produced none under the union envelope at any
cluster cell, with the RRT tier on or off. So there is a pass-1 trajectory to
derive a room from, for every active arm, which is what the two-pass scheme
assumes and what §14 could not provide.

**Numbers still owed.** The CSAIL run at `--trajectory-rooms` was still in pass
2 when this box closed, so the complete table — buckets flown, per-stage
minima under both checks, the real makespan against 391.7 s and v19's 209.9 s,
the cold/warm planning split — is not in hand, and neither is the recalibrated
1 000-stroke model. It is running to `out/staged_csail_h097_v4.json`, and the
union-envelope fly at the adopted tight setting is running alongside it to
`out/staged_csail_h097_v3.json` as the control.


## 16. The control: the union envelope, tightened, with the tier on

`out/staged_csail_h097_v3.json` — the adopted stride 1 / cell 0.075 / pad 0
envelope, RRT tier **on**, refusal loop on. This is the run §14 promised and it
closes option (1) for good.

| stage | actives | pieces | ink (m) | stage (s) | active-pair (mm) | solo (mm) | **flown** |
|---|---|---|---|---|---|---|---|
| 0 | 13, 71, 2 | 15 | 5.243 | 25.8 | +899.3 | +250.3 | **1 of 3** |
| 1 | 17, 31, 97 | 12 | 4.136 | 9.8 | +819.5 | +244.1 | **1 of 3** |
| 2 | 13, 97 | 6 | 1.461 | 48.0 | +977.2 | **+28.2** | 2 of 2 |
| 3 | 17, 2 | 5 | 1.145 | 29.4 | +870.9 | +96.2 | 2 of 2 |
| 4 | 31, 97 | 5 | 1.083 | 31.8 | +897.0 | +140.2 | 1 of 2 |
| 5 | 71, 2 | 4 | 0.923 | 53.9 | +857.0 | +69.8 | 1 of 2 |
| 6 | 13, 31 | 4 | 0.889 | 0.0 | +639.8 | +256.0 | **0 of 2** |
| 7 | 17, 71 | 2 | 0.959 | 0.0 | +539.8 | +256.0 | **0 of 2** |

**8 of 18 buckets flew, carrying 5.445 m of the 15.838 m allocated — 34.4 % of
the ink.** The active-pair gap is enormous everywhere (+539.8 mm at worst)
precisely *because* two thirds of the ink never got a trajectory: the arms that
would have been close to each other are the ones standing at their parks.

Three readings, and the third is about this document rather than the rig.

*The tightening did not help and marginally hurt.* v2, at the loose 0.15 m /
40 mm setting, flew **9** of 18; v3 at the honest tight one flies **8**. That is
the sweep's non-monotonicity showing up in the fly, and it is why §14 adopted on
the certificate rather than on the bucket count.

*The cost is real.* Planning is **2 973.9 s** cold (49.6 min), **2 869.6 s**
charged per stage to its busiest arm, against v2's 1 549.4 s — the tighter room
has 3–12× the spheres and every clearance query pays for them. Time to first
motion is unaffected at **0.294 s**, and the refusal loop is unchanged
(**94.20 %** coverage, 3 of 56 pieces unplannable, all `degenerate:too_short`).

*The makespan is 198.66 s and it is not a makespan.* It is the sum of eight
stage durations two of which are zero because nothing flew. **Comparing it with
391.7 s or with v19's 209.9 s would be comparing a third of a programme with two
whole ones**, and the earlier drafts of this document came close to doing
exactly that. Which exposes a defect in the report itself: **a stage in which
nothing flew passed both checks**, because six arms at their parks clear
everything. Stages 6 and 7 were labelled PASS on that basis. `StageResult.ok`
now requires `complete` — every bucket that has ink produced a timeline — and
the stage line prints `flown/with_ink` and says EMPTY rather than PASS.

## 17. Priority order: one sweep, an exact fixed point

The simultaneous two-pass scheme of §15 was measured out on the way past. On
CSAIL stage 0, arm 71's thirteen-piece bucket **flew in pass 1** against the
parked fleet and **stopped flying in pass 2**, once its neighbours' trajectories
became obstacles. The scheme asks every arm to yield to a path the other has
already abandoned, so nobody actually yields; it is circular, and no number of
iterations closes a cycle.

**A priority order closes it in one sweep.** The actives of a stage are ordered
by ink, busiest first, because the busiest arm has the least room to give:

- arm 1 plans against the parked fleet, and its trajectory is then **final**;
- arm *k* plans against the parked fleet **plus the final trajectories of arms
  1…*k*−1**.

When arm *k* finishes, every pair (*i*, *k*) with *i* < *k* is certified against
the path arm *i* actually flies — and arm *i* never moves again. Every pair is
therefore certified, exactly, with no iteration and no circularity. The
dependency graph stops being a cycle and becomes a **DAG**: arm *k* depends on
1…*k*−1 and on nothing after it, which is also what makes a re-plan's blast
radius finite (re-planning arm *k* invalidates only *k*+1…*n*).

It is the spatial analogue of what `coordination.coordinate` already does in
time with its priority search, with the order fixed by ink rather than searched,
because a stage has at most three actives and the busiest is the obvious choice.
The superseded scheme stays behind `--room-order simultaneous` so the
measurement that retired it can be reproduced.

**The certificate is unchanged**: the independent whole-timeline
`active_pair_gap` at `PAIR_MARGIN` is still what closes the claim. The order is
what gets the trajectories apart; the check is what proves they are.


## 18. Stage 2's +28.2 mm, attributed at last

Flagged three times without an answer, and now measured: it is **arm 97's
trajectory passing 28.2 mm from *parked* arm 2, at t = 2.2 s**. Arm 13, the
other active, is clean at +234.9 mm. `column_failed: [97]` is the same geometry
seen through the base-column cylinder rather than a second defect.

**It is a genuine active-vs-parked violation, not a modelling artefact**, and it
is untouched by every pass of this work — the envelope, the tightening and the
priority room are all about active-vs-*active*.

The mechanism is a **router/checker mismatch**, and it is worth stating because
it is systematic rather than particular to this leg. `paper.route` certifies a
pen-up leg at `STATIC_MARGIN` = 50 mm *at the samples it evaluates*;
`scene_check.check_timeline` re-derives the same leg and additionally subtracts
the 1-Lipschitz between-sample residual, `0.55 × (step_i + step_j)`. A leg that
is exactly at the gate at its samples therefore reads roughly 20 mm tighter once
the sweep between them is charged — which is what 50 → 28.2 mm looks like. The
checker is right and the router is optimistic by the sweep.

Two fixes, neither implemented: charge the router the same residual (raise its
floor by the sweep it will be measured against, which is a constant it already
has in `coordination.SWEEP_K`), or sample the leg finely enough that the
residual is negligible — the same auto-refinement `check_timeline` already does
for its frame and paper gates. The first is one number; the second is the
honest one.

**An earlier attempt at this attribution was wrong and is worth recording as
such.** It fed the programme's raw waypoints to `check_timeline` at a fixed dt,
which charges a Lipschitz residual against jumps the timeline never makes, and
reported two *parked* arms at −437 mm. The rebuild goes through
`writing.uniform_samples`, exactly as `staged.solo_check` does.


## 19. The priority room, measured: 97.6 % of the ink flies

`out/staged_csail_h097_v5.json` — priority-ordered trajectory rooms, RRT tier
on, refusal loop on. **This is the run the whole room line of work was for.**

| stage | actives (priority order) | pieces | ink (m) | stage (s) | **active-pair (mm)** | ink vs ink (mm) | solo (mm) | **flown** | verdict |
|---|---|---|---|---|---|---|---|---|---|
| 0 | 71, 13, 2 | 15 | 5.243 | 107.2 | **+269.3** | +411.1 | +80.7 | 2 of 3 | **FAIL** |
| 1 | 31, 97, 17 | 12 | 4.136 | 89.8 | **+158.5** | +417.5 | +60.9 | 2 of 2 | PASS |
| 2 | 13, 97 | 6 | 1.461 | 48.0 | +977.2 | +1 055.3 | **+28.2** | 2 of 2 | **FAIL** |
| 3 | 2, 17 | 5 | 1.145 | 29.4 | +870.9 | +1 151.2 | +96.2 | 2 of 2 | PASS |
| 4 | 31, 97 | 5 | 1.083 | 31.8 | +897.0 | — | +140.2 | 1 of 1 | PASS |
| 5 | 71, 2 | 4 | 0.923 | 58.6 | +623.8 | — | +72.5 | 1 of 1 | PASS |
| 6 | 31, 13 | 4 | 0.889 | 26.5 | +263.7 | — | +71.2 | 1 of 1 | PASS |
| 7 | 71, 17 | 2 | 0.959 | 23.6 | +525.3 | — | +80.9 | 1 of 1 | PASS |

**12 of 13 ink-carrying buckets fly, carrying 15.455 m of 15.840 — 97.6 % of
the ink**, against the union envelope's 8 of 18 and 34.4 %. **Six of eight
stages pass both checks.**

| | v3, union envelope | **v5, priority rooms** |
|---|---|---|
| ink buckets flown | 8 of 13 | **12 of 13** |
| ink in a certified trajectory | 5.445 m (34.4 %) | **15.455 m (97.6 %)** |
| stages passing both checks | 4 of 8 (two vacuously) | **6 of 8** |
| makespan | 198.7 s (a third of a programme) | **415.0 s (a real one)** |
| planning, serial | 2 973.9 s | **263.8 s** |
| planning, per stage's busiest arm | 2 869.6 s | **89.7 s** |
| check | 42.6 s | 82.4 s |
| time to first motion | 0.294 s | **0.287 s** |

**Planning is 11× faster than the control and 32× faster per arm**, which was
not the point but is the largest single number in the table. Two reasons: the
trajectory rooms are 10–1 001 spheres against the union's 1 157–1 377, and the
first arm of every stage plans **free** — a third of the buckets pay nothing for
the room at all.

**The makespan is 415.0 s and it is real** — one bucket of 0.385 m is missing
out of 15.840 m. Against **391.7 s** for the un-separated v1 programme it is
**+5.9 %**, which is what routing three arms around each other costs; against
v19's conducted **209.9 s** it is **1.98×**, the barrier cost that §10's
adaptive-stage rule is aimed at. Park overhead is **92.7 s on the critical path,
22.3 % of the makespan**.

**The two failures are both known and neither is the room.** Stage 0 loses its
*third* arm — 0.385 m, the greedy order's expected weak spot, since the last arm
has the least freedom left. Stage 2 is §18's router-sweep mismatch at +28.2 mm,
which is active-vs-**parked** and untouched by any of this.

### The 1 000-stroke model, recalibrated

Terms measured on v5: draw **13.59 s/m**, inter-piece leg **4.18 s**, park
overhead **10.66 s** per (stage, arm), accept rate 81.9 %.

**Staged makespan (model): 6 380 s = 106.3 min**, against
`V2_SCALING_BASELINE` §3.3's ~4 300 s scaling of v19's conducted six-arm
makespan — **1.48×**. Park overhead on the critical path falls to **85.3 s,
1.34 %** of the makespan, against CSAIL's 22.3 %: **the barrier amortises**, and
the eight-stage pattern is right at scale and wrong on a small picture, exactly
as §10 argued. The draw rate is nearly double the v3 calibration (13.59 against
6.56 s/m) because v5 actually draws the long buckets that v3 never flew — the
earlier figure was calibrated on the short ones that happened to survive.

**Two caveats, stated.** The model is arithmetic with measured terms, not a run:
a real 1 438-piece fly is unaffordable because `sequence.cost_matrix` is O(n²)
route screens and one bucket is 434 pieces. And it assumes the priority order
scales — at 572 pieces in a stage-0 bucket the *last* arm in the order faces a
much larger obstacle than it does here, which is precisely where the one
remaining failure already is.


## 20. Closing the two failures — and correcting one of the two diagnoses

§19 left exactly two defects: stage 0 stranding its *third* arm (0.385 m), and
stage 2's +28.2 mm against parked arm 2. Three things were built for them, and
**the second one's diagnosis was wrong and is corrected here.**

### (1) The order search

The greedy ink-first order has a known weak spot: **the arm that plans last has
the least freedom left.** A stage has at most three actives, so the whole order
space is **six permutations** — cheap, where a six-arm priority search (720
orders, `∑ₖ P(n, k)`) is not. Ink-first is tried first, so a stage that did not
need the search pays one comparison and nothing else; `order_rank` records which
order was taken and `orders_tried` what it cost. `--order-search 1` is the
pre-search behaviour.

### (2) The router was not the optimistic one

The instruction for this pass was to make `paper.route` refine its sampling
until its residual-inclusive bound clears the floor. **It already does**, and
saying so is more useful than building it again:

- `paper.leg_bounds` computes `sample_residual(P)` and returns `m − res` when
  that clears, falling through to `adaptive_static_lb` — real adaptive
  subdivision — in the undecided band;
- `FRAME_FLOOR` = **53 mm** is already `STATIC_MARGIN` plus the checker's
  residual, and `paper.py`'s own header explains both that and the `TIP_SWEEP_PAD`
  version of the same argument.

So where does 28.2 mm come from? **Two places, and I measured which.**
`check_timeline` auto-refines its frame and paper gates and does **not** refine
its **inter-arm** gate — it subtracts `0.55 × (stepᵢ + stepⱼ)` at whatever rate
the timeline was handed in at. Refining stage 2's arm 97:

| dt | frames | min clearance |
|---|---|---|
| 0.05 (as reported in §18) | 961 | **28.23 mm** |
| 0.02 | 2 400 | 36.58 mm |
| 0.01 | 4 799 | **39.38 mm** |

**Eleven millimetres of the deficit was the sampling**, and that half is now
fixed: a verdict under the margin is looked at again, up to three halvings or
40 000 frames, keeping the best bound found. That is `block_screen`'s own rule
— *"a cell between the bounds has to be LOOKED AT rather than believed either
way"* — applied to the one gate that lacked it, and it costs nothing on a run
that passes.

**The other ten millimetres are real, and they are not routing.** The number
converges to ~40 mm, not 50. `paper.effective_static_floor` **clamps the floor
to what the leg's own endpoints have**, deliberately and with a docstring
explaining why: *"a move is never asked to keep more clearance than its own
endpoints have"*, because a floor above the endpoints is not a constraint but a
contradiction that prices every edge `inf`. Arm 97 holds a **pose** about 40 mm
from parked arm 2, and **no amount of routing fixes a pose**. The lever is a
different hover or a different park for arm 2 in stage 2 — which is build item
2's per-stage parks, for real this time and for a reason that did not exist when
that item was written out in §14.

§18 called this "the router is optimistic by its own sweep". That was half
right and the wrong half is the important one: the router's *sweep* is
accounted; the router's *floor* is clamped by a pose it did not choose.

### (3) The residue phase

A bucket no order can fly concurrently does not have to be abandoned — it has to
be **serialised**. It is planned alone against the parked fleet (the room pass 1
always flies in) and its timeline is *appended* to the stage rather than
overlapped, because the other actives are back at their parks by then, which is
what the barrier means. **That is Pete's original final pass**, and it is the
floor under the whole scheme: the worst case of the room work is the
one-arm-at-a-time programme the installation started from.

`idle.conduct` is the named tool and it **reduces to nothing here**: with one
arm moving the priority search enumerates `∑ₖ P(1, k)` = one order and there is
no second mover to schedule against. So the residue is laid down directly and
checked identically — `solo_check` against the parked fleet *is* the certificate
a one-mover conduct would produce — and `active_pair_gap` never sees it, because
nothing else is in the air. `StageResult.duration` is accordingly the busiest
**concurrent** arm plus the sum of the residues.


## 21. v6, measured: every bucket flies, 100 % of the ink, seven of eight stages

`out/staged_csail_h097_v6.json` — priority rooms + order search + residue phase,
RRT tier on, borderline verdicts refined.

| stage | order | orders tried | pieces | ink (m) | stage (s) | active-pair | solo | residue | flown | verdict |
|---|---|---|---|---|---|---|---|---|---|---|
| 0 | 71, 13, 2 | **6** | 15 | 5.243 | 124.8 | +269.3 | +80.7 | **0.385 m (arm 2)** | 3/3 | **PASS** |
| 1 | 31, 97, 17 | 1 | 12 | 4.136 | 89.8 | +158.5 | +60.9 | — | 2/2 | **PASS** |
| 2 | 13, 97 | 1 | 6 | 1.461 | 48.0 | +977.2 | **+40.4** | — | 2/2 | **FAIL** |
| 3 | 2, 17 | 1 | 5 | 1.145 | 29.4 | +870.9 | +96.2 | — | 2/2 | **PASS** |
| 4 | 31, 97 | 1 | 5 | 1.083 | 31.8 | +897.0 | +140.2 | — | 1/1 | **PASS** |
| 5 | 71, 2 | 1 | 4 | 0.923 | 58.6 | +623.8 | +72.5 | — | 1/1 | **PASS** |
| 6 | 31, 13 | 1 | 4 | 0.889 | 26.5 | +263.7 | +71.2 | — | 1/1 | **PASS** |
| 7 | 71, 17 | 1 | 2 | 0.959 | 23.6 | +525.3 | +80.9 | — | 1/1 | **PASS** |

**13 of 13 ink buckets fly. 15.840 m of 15.840 — 100.0 % of the allocated ink.
Seven of eight stages pass both checks.**

| | v3 union envelope | v5 priority rooms | **v6 + search + residue** |
|---|---|---|---|
| ink buckets flown | 8 of 13 | 12 of 13 | **13 of 13** |
| ink in a certified trajectory | 34.4 % | 97.6 % | **100.0 %** |
| stages passing | 4 of 8 (two vacuous) | 6 of 8 | **7 of 8** |
| makespan | 198.7 s (partial) | 415.0 s | **432.6 s** |
| planning, serial | 2 973.9 s | 263.8 s | 769.0 s |
| planning, per stage's busiest arm | 2 869.6 s | 89.7 s | **20.6 s** |
| time to first motion | 0.294 s | 0.287 s | **0.280 s** |

**The makespan is 432.6 s**: +4.2 % on v5 (415.0 s) and **+10.4 % on the
un-separated v1 programme (391.7 s)** — the whole cost of making three arms
provably safe from one another is a tenth of the clock. Against v19's conducted
**209.9 s** it is **2.06×**, which is the barrier, not the rooms; park overhead
is 92.7 s on the critical path, **21.4 %** of the makespan.

**No stage needed a non-ink-first order.** Stage 0 tried all six and kept
rank 0, because *no* order flies it — arms 2 and 71 are mutually exclusive there
whatever the order, and the search's real job turned out to be picking the
cheapest sacrifice (keep arm 71's 4.053 m concurrent, serialise arm 2's
0.385 m). Every other stage flew on the first order and paid one comparison.
**The order search is therefore worth keeping for what it proves, not for what
it finds**: it is what licenses the residue, by showing the residue was not an
order away from being avoidable.

**The residue is 0.385 m — 2.4 % of the ink — and costs 17.6 s** (124.8 against
v5's 107.2 s for stage 0), a 16.4 % premium on that stage and 4.1 % on the
programme. That is Pete's original final pass, used exactly once, for the one
bucket nothing could fly concurrently.

**Planning per arm collapses to 20.6 s** — 139× the union envelope's 2 869.6 s —
because the order search's cost is serial-per-stage while the figure that
matters is what one arm waits for. Serial planning rises to 769.0 s, which is
the six extra stage-plans stage 0's search paid for.

**The one failure is stage 2 and it is fully attributed** (§20): solo +40.4 mm
after refinement recovered 12.2 mm of checker sampling, still 9.6 mm short of
the gate, because arm 97 *holds a pose* ~40 mm from parked arm 2. All three of
this pass's mechanisms are active-vs-active; this is active-vs-parked. **8/8 is
not reachable without a different park or hover for arm 2 in stage 2** — build
item 2, with a concrete reason at last.

### The 1 000-stroke model, recalibrated on v6

Terms: draw **13.81 s/m**, inter-piece leg **4.18 s**, park **10.93 s** per
(stage, arm), accept rate 81.9 %.

**Staged makespan (model): 6 437 s = 107.3 min**, **1.50×** the ~4 300 s scaling
of v19's conducted six-arm makespan. Park overhead on the critical path falls to
**87.5 s, 1.36 %** against CSAIL's 21.4 % — **the barrier amortises**, and the
eight-stage pattern is right at scale and wrong on a small picture, as §10
argued. Arithmetic with measured terms, not a run, and it assumes the priority
order and the residue scale: at 572 pieces in a stage-0 bucket the residue could
be far more than 2.4 %, and that is the number the next scaling pass owes.



## 22. The pattern was not the one that was specified — Pete's leader/follower, built and measured

**Pete Werner, 2026-09-14, on reading §21:** the zigzag is not the pattern he
asked for. His is *"the pattern where we did the 1-2-1 for planning the lead arm
with priority and then the other arm in the column would only try to knock out
lines in its cell that were safe to draw. and then in the end we would do the
coordination of all arms to fill in the gaps if needed"* — and, on the cells,
*"restrict their cells as completely occupied so they can run asynchronously …
for the occupied cells we want to also make sure that the free space motions
never intersect with those … there will likely still be lines we can't draw in
the critical regions in the middle which we will need to handle in a final
pass."* **All six arms move in every main stage.**

### Why the zigzag's premise does not carry the weight it was given

The zigzag parks a leader's same-row partner for the whole stage. The reason
recorded for that (`docs/V2_WORKCELLS.md` §4b, repeated in
`ARCHITECTURE_V2` §2b and in `traces.zigzag_pattern`'s own docstring) is a
measurement of **two full work-cell ENVELOPES** against each other: every arm's
elbow swings within 0.10 m of the mid-line while it draws, so a transverse
pair's envelopes interpenetrate by 135–262 mm and *"no pattern may ever put a
same-row pair in the air together."*

That conclusion is one quantifier too strong. An envelope is the union over
every pose an arm **could** hold anywhere in its cell; the question Pete asked
is about one arm drawing a **restricted subset** while the other's **realised
trajectory** is the occupied volume. §15 already built the second object —
`staged.trajectory_room` — and §19 already measured what it is worth: arm 13's
room is a factor of **12** smaller than its envelope, and priority rooms took
the flown ink from 34.4 % to 97.6 %. The envelope argument was applied to a pair
the room argument was never asked about. That is the whole of the correction.

### What was built — `traces.leader_follower_pattern`, three stages

| stage | who | how they are certified |
|---|---|---|
| **A** | leaders **13, 71, 2** (priority 0–2) + followers **17, 31, 97** (3–5) | the existing priority sweep: arm *k* plans against the FINAL trajectory rooms of arms 1…*k*−1 |
| **B** | the roles swapped | the same |
| **C** | all six, **`idle.conduct`** | the whole conducted timeline, `scene_check.check_timeline` |

The leaders are the **1-2-1** — one arm per row with the columns alternating —
which is the zigzag's own stage 0, so leader-vs-leader separation is the 0.40 m
row dead band, unchanged and re-measured. The new adjacency is the same-row
**leader/follower** pair, and it is exactly where the argument has to be new:
the follower plans against the leader's realised stage trajectory as a static
keep-out, which makes it **asynchronous by construction** and certifies its
free-space legs against the occupied volume with the same room machinery.

**A piece that does not fit is DEFERRED, never serialised.** §20.3's residue
pass flies a stranded bucket alone after the others park; that is right for the
zigzag and is precisely what this pattern exists to avoid, because serialising
inside a stage gives back the concurrency six active arms were meant to buy.
Here the piece leaves the stage: first to the stage where **that same arm
leads** (a follower's remainder is ink it can plan free next time), and only
then to the conducted final pass.

**The bag split is the pattern's one design freedom.** The min-pieces DP hands
each arm a bag by cell; each arm draws part of it leading and part following. A
follower's safe pieces are the ones far from its leader, so an arm draws its
half of the **contested middle** (the paper between the two base columns) as
leader and its **outer strip** as follower. `split_m` moves that boundary
outward from the arm's own base column and is swept.

### And the barrier stopped being a trip

**Pete, the same day, watching the v6 animation:** *"a lot of excessive parking …
once the 1-2-1 arms have their TSP tour we should be just executing that plan as
efficiently as possible."* So under this pattern a stage ends at the hover above
its last stroke and **holds** there; the next stage starts from that pose, and
every arm not yet moving is in its neighbours' static room as the pose it is
**actually holding** rather than as its park. Parks survive where they are
load-bearing — the start of the programme, the end of it, and the fault-recovery
home — and nowhere else.

The park's own guarantee had to be replaced, not dropped. `Q_PARK_PROPOSED` was
searched to be mutually clear, so a park barrier was safe by construction; a
held barrier is wherever the ink happened to end. It is safe by a different
argument — an arm's trajectory room contains its last sample, and every later
arm was routed clear of that room — and `staged.hold_gap` asserts that argument
at every barrier, with `scene_check`'s own capsules and the same 50 mm gate.

### One correction to the check, found by the first run

Six named actives of which most are holding broke `active_pair_gap`. It takes
the **cross product** of two timelines and subtracts a 1-Lipschitz residual
computed on the **decimated** grid, because two asynchronous arms have no common
clock. Against an arm that is standing still there is no cross product to take,
and charging the still arm the moving one's decimation residual cost **30 mm of
a 50 mm gate** on the first CSAIL run and measured nothing: stage 0 read
+22.7 mm where `solo_check` — which measures exactly that pair, at the full rate,
and refines a borderline verdict — read +52.4 mm. `active_pair_gap` now sees
only the arms that move. Still arms are `solo_check`'s and `hold_gap`'s.

### What it measures on the CSAIL logo — and the finding is not the one the design hoped for

The complete run in hand is `out/staged_csail_h097_lf_smoke.json`, split
**+0.15 m**, taken **before** the held barrier and the pair-check correction
landed (so it still pays v6's park trips and reads the pessimistic pair number):

| stage | roles | pieces | ink (m) | stage (s) | deferred (m) | flown |
|---|---|---|---|---|---|---|
| A | lead 13,71,2 / foll 17,31,97 | 6 | 0.763 | 61.9 | **3.770** | 1/1 |
| B | lead 17,31,97 / foll 13,71,2 | 9 | 2.209 | 47.9 | **2.865** | 3/3 |
| C | **conductor, all six** | 50 | **13.097** | 151.2 | 0.000 | 6/6 |

| | v19 conducted | v6 zigzag | **leader/follower** |
|---|---|---|---|
| makespan | 209.9 s | 432.6 s | **261.0 s** (1.24× v19, **0.60× v6**) |
| ink in the main stages | — | 15.840 m | **2.97 m of 16.08 m** |
| ink in the final pass | 16.80 m | 0.385 m residue | **13.097 m (81 %)** |
| follower ink offered | — | — | 7.288 m |
| follower ink that fitted | — | — | **0.806 m — 11.1 %** (A: **0 %**, B: 21.9 %) |
| logo coverage | — | 94.2 % | **95.6 %** |
| time to first motion | 3 712.4 s | 0.280 s | **0.183 s** |
| park overhead, critical path | — | 92.7 s (21.4 %) | 55.0 s (21.1 %) |
| planning, serial | — | 769.0 s | 912.5 s |
| check | — | — | 95.9 s |

**The honest headline: the followers keep almost nothing, and the conducted
final pass ends up drawing the picture.** Stage A's three followers flew **zero
pieces** at every split tried. The pattern does not fail — it is 1.7× faster
than the zigzag and it draws more of the logo — but it does not work the way it
was meant to. What it actually is, on this rig, is *a short two-stage
leaders-only warm-up followed by v19*.

**And the reason is not the per-piece ink gate.** The fourth run of the sweep
turns that gate back into a measurement (`--no-follower-gate`) and its stage A
is **identical to the gated run's, piece for piece and metre for metre**:
15 pieces, 2.061 m, the same three leader buckets flown and the same three
follower buckets deferred. A follower is not losing its ink to "your stroke
passes through the leader's trajectory"; **its bucket produces no timeline at
all** — `writing.arm_program` cannot route the pen-up legs out of the held pose,
into the first stroke and between the pieces with the leaders' rooms in the way.
That is the leg problem of §3 and §8 again, one row down, and it is where the
next pass has to go: the follower needs a hover ladder or an entry pose chosen
*against* the leader's room, not merely checked against it.

### The split sweep

`scripts/lf_sweep.sh` runs four settings concurrently; `scripts/lf_report.py`
prints the tables from the `--json` summaries. Stage A, with the held barrier and
the corrected pair check, **passes both checks at every setting**:

| split | stage-A pieces | stage-A ink (m) | stage-A (s) | active-pair (mm) | solo (mm) | follower buckets flown | verdict |
|---|---|---|---|---|---|---|---|
| **whole** (Pete's literal) | 17 | 4.595 | 129.4 | **+85.0** | +51.2 | 0 of 3 | PASS |
| **+0.00** | 11 | 1.480 | 78.2 | **+192.9** | +52.4 | 0 of 3 | PASS |
| **+0.15** | 15 | 2.061 | 97.0 | **+131.8** | +52.6 | 0 of 3 | PASS |
| **+0.15, gate off** | 15 | 2.061 | 97.0 | +131.8 | +52.6 | 0 of 3 | PASS |

More split is more leader ink and a wider, slower stage A, monotonically — and
it is monotone in the clearance too, because a leader drawing further out draws
closer to its partner's held pose. **No setting buys a single follower bucket**,
which is the same statement the gate control makes from the other side.

### The whole-bag run, complete — and the number that settles it

`out/staged_csail_h097_lf_whole.json`, Pete's literal baseline, with the held
barrier and the corrected pair check:

| stage | who | pieces | ink (m) | stage (s) | active-pair | solo | conducted | deferred (m) | flown | verdict |
|---|---|---|---|---|---|---|---|---|---|---|
| A | lead 13,71,2 / foll 17,31,97 | 17 | 4.595 | 129.4 | **+85.0 mm** | +51.2 mm | — | 5.026 | 3/3 | **PASS** |
| B | lead 17,31,97 / foll 13,71,2 | 1 | 0.028 | 17.3 | — (one mover) | +73.7 mm | — | 4.997 | 1/1 | **PASS** |
| C | **conductor, all six** | 40 | 11.458 | 126.6 | — | — | **+44.9 mm** | 0.000 | 3/6 | **FAIL** |

| | v19 | v6 zigzag | **leader/follower, whole bag** |
|---|---|---|---|
| makespan | 209.9 s | 432.6 s | **273.3 s** (1.30× v19, **0.63× v6**) |
| **park overhead, critical path** | — | 92.7 s, **21.4 %** | **30.9 s, 11.3 %** |
| barrier held-pose clearance | — | (parks, +256 mm) | **+256 / +108.2 / +107.0 mm, all PASS** |
| coverage of the logo | — | 94.2 % | **95.6 %** |
| time to first motion | 3 712 s | 0.280 s | **0.189 s** |
| check | — | — | **39.1 s** (95.9 s before the pair fix) |
| leader ink kept | — | — | 4.624 m of 4.624 m |
| **follower ink kept** | — | — | **0.000 m of 5.026 m — 0.0 %** |
| deferred to the final pass | — | 0.385 m residue | **10.023 m** |
| stage C planning + conduct | — | — | **3 075 s of wall** for 126.6 s of motion |

**The held barrier works and it is worth 10 points of makespan.** Park overhead
on the critical path falls from 21.4 % to **11.3 %**, every barrier's held pose
set clears pairwise by **107 mm or better**, and the time to first motion is
unchanged at 0.189 s. That part of Pete's instruction lands cleanly and would be
worth keeping whatever pattern runs above it.

**The follower does not.** Not one of the fifteen follower pieces came near the
gate:

| | min | p05 | median | p75 | max | under the 50 mm gate |
|---|---|---|---|---|---|---|
| **follower** ink vs the room it had to fit | −419.7 mm | −417.6 mm | **−235.7 mm** | −175.2 mm | **−131.9 mm** | **15 of 15** |
| leader ink vs the rooms before it | +249.2 mm | +249.5 mm | +252.6 mm | +277.4 mm | +302.2 mm | 0 of 3 |

The follower's ink is not marginally refused. **The closest piece is 132 mm
inside the occupied volume and the median is a quarter of a metre inside it.**
No gate setting, no hover ladder, no split and no ordering moves a number like
that — which is why the gate-off control changes nothing, and why the leg
refusals and the ink refusals are one fact seen twice.

**So the envelope measurement's substance survives even though its reasoning was
over-general.** §4b's *"no pattern may ever put a same-row pair in the air
together"* was inferred from full envelopes and should have been inferred from
rooms; asked properly, of rooms, on this rig, the answer is the same. The
correction was worth making — the claim is now anchored where it belongs, and
the held barrier and the six-arm stage came out of asking — but the transverse
pair really is unseparable here, and a follower on this geometry has nothing
safe to draw.

**And the final pass is the bill.** With the followers empty, 10.023 m of the
logo falls to stage C, which then costs **3 075 s of wall for 126.6 s of
motion** and still ends **5.1 mm short of the gate** with 3 of its 6 ink buckets
unflown. That is `ARCHITECTURE_V2` §2d's warning arriving: conducting most of a
picture as one phase does not scale, and a pattern that defers most of the
picture to the conductor has reinvented v19 with a slower planner.

**What the 1 000-stroke model would say, and why it is not quoted.** The v6
recalibration (§21) assumes the main stages carry the ink. Here they carry
4.62 m of 16.08 m and the conductor carries the rest, so the model's terms —
draw s/m, inter-piece leg, park per (stage, arm) — are being fitted to three
arms' worth of ink and then asked about six. A projection built on that would
be arithmetic about a pattern nobody would ship. The honest number to carry
forward is the one this run measured directly: **stage C's 3 075 s for 11.5 m**,
which at 1 000 strokes is not a schedule, it is a refusal.

### What is still running

`s000` and `s150` were still in stage C when this box closed — in its *planning*,
not its conduct, which is `sequence.cost_matrix`'s O(n²) route screen on
~50-piece buckets and is the same wall §19 flagged. Their stage A and B numbers
are the table two sections up and will not move; what they still owe is their
stage C cost and their makespan. Re-run `scripts/lf_report.py
out/staged_csail_h097_lf_*.json` when the JSONs appear.

## 23. The room was the wrong shape — exact swept capsules, and the follower's tuck

§22 left stage A with the followers keeping **0 %** of their ink at every split,
and the diagnosis (`scripts/diag_lf_follower.py`, commit `2cd3eb0`) named the
biggest correctable term: the room a follower is certified against was not the
leader's trajectory, it was `cluster_capsules`' **sphere reduction** of it — one
sphere per 0.075 m cell, radius = 65 mm half-diagonal + up to 177 mm (the
biggest capsule in the cell) + 24 mm sweep pad, median 186 mm and max 417 mm.
Measured cost of that reduction: a **median 195 mm** of clearance the leader's
metal was not occupying.

### 23.1 What was built

**`aris_sixarm/exact_room.py`** — a leader's realised trajectory as its **real
swept capsule chain**: one capsule per link per timeline sample, each carrying
`scene_check.check_timeline`'s own per-sample 1-Lipschitz residual (`SWEEP_FRAC`
× the larger of the step into and the step out of that sample) rather than the
sphere form's single global maximum. Nothing is decimated at the shipped stride.

The spheres are kept — as the **broad phase**. Each grid cell holds its members,
its bounding sphere *and* its AABB, and a query prunes a cell whole when both
bounds already exceed the best gap found so far. The narrow phase then opens
**shells** of increasing bound and stops a row the moment its best gap falls
below the shell boundary already measured — every unmeasured cell lies beyond
that boundary, so the termination is exact, not heuristic. The answer is
bit-identical to a brute-force minimum over every capsule at every cell size
(`tests/test_exact_room.py`).

| | cost per pose, stage-sized room (26 000 capsules, 87 cells) |
|---|---|
| floored at the router's `FRAME_FLOOR` (how every `paper.route` gate asks) | **0.05 ms** |
| floored at the ink gate's cap (`staged.INK_CAP`, 0.30 m) | **0.18 ms** |
| unfloored, exact to infinity | 4.6 ms |

The budget was ≤ 1 ms per pose for the planner's gates; the gates are floored,
and a test pins the floored number under 1 ms. `ink_vs_envelope` is capped at
0.30 m for the same reason `coordination.BROAD_CAP` caps everything else — the
gate it feeds is 50 mm.

**One seam, not five.** Every consumer named in the build item already reaches
the room through `frozen.chain_clearance` / `frozen.partner_clearance`:
`plan_stroke`'s ink gate, `writing.arm_program`, `paper.route`'s straight,
ladder and RRT tiers, and `paper.effective_static_floor`. So the room was
swapped **inside `frozen`** and all of them saw it at once. The one tier that
could *not* see it is the go-around: `paper._skirt` reads box **footprints** to
decide which way to walk, and a room is not a box — so `frozen.room_boxes()`
hands it the room's coarse cells **for detour generation only**. Every candidate
it proposes is still gated against the exact room, so a bad pseudo-box costs a
refused detour and never a certificate. `scene_check` shares none of this code
and stays the judge.

`ARIS_ROOM=spheres|capsules` (default `capsules`) is the whole A/B.

### 23.2 The A/B, same tree, same seam bar, CSAIL h = 0.970, split +0.15, stage A

| | sphere room (`ARIS_ROOM=spheres`) | **exact capsules** |
|---|---|---|
| leader 71's room | 891 spheres | **19 190 capsules** |
| follower 31's **start pose** vs the room | **−128.4 mm** | **+53.7 mm** |
| follower ink clearance, min | −243.8 mm | **−199.8 mm** |
| …median | −134.6 mm | **−118.9 mm** |
| …max | −128.4 mm | **−38.0 mm** |
| pieces refused at the ink gate | 3 | **2** |
| leader ink flown | 2.0613 m | 2.0613 m |
| **follower ink flown** | **0.000 m (0 %)** | **0.000 m (0 %)** |
| stage A inter-arm (`scene_check`) | +65.6 mm, PASS | +65.6 mm, PASS |

**The start pose is the result that matters.** It moved by **+182 mm**, from 128 mm
*inside* the room to 54 mm outside it. That was the mechanism §22.3 blamed:
`paper.effective_static_floor` clamps every leg's static floor to what its own
endpoints can hold, so a park inside the room clamped the floor **negative**
before routing began and the entry legs failed first. That specific failure is
gone.

**And the follower still keeps nothing.** The exact room moved the ink clearance
by +44 mm (min) to +90 mm (max) and it is still deeply negative. This is not a
modelling artefact any more — it is the allocation: **the follower's ink lies
inside the volume the leader's arm actually sweeps.**

### 23.3 Why 55.9 % of poses clear does not buy 55.9 % of the ink

The diagnosis measured 55.9 % of arm 31's ink samples clearing 50 mm against
the exact capsules, and read that as the ceiling. It is not, because
**`ink_vs_envelope` returns the MINIMUM over a piece's poses** and the gate
refuses the piece whole. A piece with 55.9 % of its poses clear still has a
minimum of −185 mm, so it is refused entire. Turning a pose-wise fraction into
flown ink needs the pieces **split at the room boundary** — a DP that cuts a
stroke where it enters a leader's trajectory, which the refusal loop does for
*coverage* but not for *rooms*. That is the next build item, and it is now the
only one between here and a follower that draws.

### 23.4 The tuck, and why it did not fire

`staged.tuck_pose` / `clear_out` / `splice_timeline` implement the
pre-position: a follower searches hover stations over **its own ink** at a
ladder of heights for one that clears the leaders' exact rooms by the routing
floor + 10 mm, flies park → tuck **while the leaders are still parked** (so the
clear-out is routed against the parked fleet, like any other leg), and plans its
tour from there. The clear-out is **spliced into the follower's timeline**, not
left as a prologue — so `trajectory_room`, `active_pair_gap` and `scene_check`
all see it, and the next arm in the sweep avoids it. `q_tuck` and `clear_out_s`
are recorded per arm in the programme (schema stays 2; `q_hold` is unchanged).

On this picture **it did not fire, and it should not have**:

- arm 31 (split +0.15): park **+53.7 mm**, 0 of 45 stations cleared the 73 mm
  bar → stays put. The park is *already* outside the room once the room is
  exact; every hover over its own ink is *inside* it, which is the same fact
  §23.2 reports — the ink is in the leader's swept volume, so a station above
  the ink is too.
- arm 97 (whole bag): park **+263.4 mm** — already clear, early return, no move.

So the clear-out cost **0 s** and the time to first motion is unchanged at
**2.269 s** (split +0.15), well inside the 10 s bar. The machinery is built,
tested and inert on this picture; it earns its keep the moment a park sits
inside a room again, which the seam-bar park re-search happened to fix for
arm 31 at the same time.

### 23.4b Stage B, split +0.15: the followers fly 62.6 % of their ink

The split +0.15 run finished stages A and B after the box closed, and stage B
is the first result in this pattern where **a follower keeps anything**:

| stage B (roles swapped: leaders 17/31/97, followers 2/13/71) | value |
|---|---|
| follower ink offered | 2.5249 m |
| **follower ink flown** | **1.5812 m — `follower_fit_frac` 0.626** |
| follower ink clearance vs the leaders' exact rooms | min **+35.7**, median **+179.6**, max +1591.9 mm |
| leader ink flown | 0.0284 m of 0.0284 m |
| inter-arm, realised trajectories (`active_pair_gap`) | **+98.6 mm** (binding pair 71↔97, leg vs leg) |
| static set, per arm (`scene_check` via `solo_check`) | **+73.7 mm** |
| stage duration / busiest arm | 24.0 s |
| stage verdict | **FAIL** |

**The FAIL is not a collision.** The realised trajectories separate by +98.6 mm
and every arm clears the steel by +73.7 mm — both comfortably over their gates.
The stage is marked not-ok by `ink_vs_envelope_mm = 35.67`: one accepted piece's
ink reads +35.7 mm against a room, under the 50 mm `PAIR_MARGIN`. That is the
planner's own per-piece reading of a stage the independent measurements pass,
and it is the same per-piece minimum §23.3 identifies as the thing to split on.

Totals for the split +0.15 run, stages A + B: makespan **119.885 s**, time to
first motion **0.216 s**, coverage 95.63 %, leader ink 2.0898 m flown,
**follower ink 1.5812 m of 4.6688 m offered (33.9 %)**, deferred 8.0850 m,
`holds_ok` true.

### 23.5 The whole bag, stages A + B (the full A/B/C did not fit the box)

Run with `--stages 0,1`, exact rooms, seam bar in:

| | value |
|---|---|
| makespan, stages A + B | **145.867 s** (A 128.6 s + B 17.3 s) |
| time to first motion, clear-out included | **0.192 s** |
| leader ink flown | **4.6238 m of 4.6238 m offered** |
| follower ink flown | **0.000 m of 5.0258 m offered (0 %)** |
| deferred to stage C | **10.0231 m** of 16.0826 m drawn = **62.3 %** |
| follower ink clearance vs the exact rooms | min −272.4, median −152.6, max −37.8 mm, 13 of 13 under gate |
| `scene_check` | stage A and B both **PASS**, `all_ok` true, `holds_ok` true |
| held-pose barriers | 3, all clear |

Against the §22 baseline (sphere rooms, **pre**-seam-bar, so only partly a
controlled comparison): A + B makespan 146.7 s → 145.9 s, follower ink
clearance min −419.7 → −272.4 mm, median −235.7 → −152.6 mm, max −131.9 →
−37.8 mm. The deferred share is **unchanged at 62 %** because it is the
followers' whole bag either way. The controlled A/B is §23.2, which holds the
tree and the seam bar fixed.

**Stage C was not run.** The build capped a six-arm conduct at 30 minutes of
wall clock and the box closed before stage A + B finished on both splits; the
split +0.15 run was still in stage B's route screen — the same
`sequence.cost_matrix` O(n²) wall §19 flagged — when this was written. Its
stage A numbers are §23.2 and will not move.

### 23.6 What did not change, and the honest bottom line

Stage A's inter-arm clearance, its makespan (95.9 s for the busiest arm at split
+0.15, 128.6 s whole-bag) and its verdict are unchanged: **PASS**, `all_ok`
true, 3 of 3 leader buckets flown. Coverage 97.46 % (split +0.15) / 95.64 %
(whole bag). The exact room is strictly better geometry at 1/25th of the pruning
slack and no measurable planning cost, and it removes one of the two reasons the
follower kept nothing. The other reason is the allocation, and no room model
fixes it.

**The 1 000-stroke model is NOT recalibrated in this pass**, because the
condition the build set for it — "only if stages A/B now carry most of the ink"
— is not met. Stages A and B carry the leaders' ink alone; the followers carry
none, so the deferred share is unchanged in kind. Recalibrating a throughput
model on a stage that still defers its followers' whole bag would be quoting the
conductor's cost as if it were the pattern's.

## 24. Cut at the room boundary, and a conductor per row

§23 left two things named and neither built. The first: `ink_vs_envelope`
returns the **minimum** over a piece's poses and the gate refuses the piece
**whole**, so 55.9 % of follower 31's poses clearing the 50 mm gate bought
**0 %** of its ink. The second: stage C conducted six arms over most of the
picture and cost **3 075 s of wall for 126.6 s of motion**. Both are built here,
and both are the same idea twice — *cut where the geometry changes, and do not
ask six arms a question that only two of them are in*.

### 24.1 The cut — `staged.split_at_room`

A piece the room ink gate refuses is re-cut at the room boundary and its
**certified clear stretches** come back through the same loop as ordinary
pieces. Four parts, and each of them is a claim:

1. **The profile.** `ink_clearance_profile` is `ink_vs_envelope` per pose
   instead of minimised over them, with the follower's **own** 1-Lipschitz
   sweep residual subtracted (`exact_room.sweep_pads` on the observer's chain
   points). The room already carries the leader's residual; this is the other
   half, and it makes the profile *stricter* than the gate it refines — a
   property a test pins.
2. **The runs.** `clear_runs` returns the contiguous sample stretches whose
   every pose clears the gate, each at least `SPLIT_MIN_M` (=
   `traces.MIN_PIECE_M`, 10 mm) long. A shorter stretch is a pen-down, a
   pen-up and no picture, which is the judgement `traces.absorb_short` already
   makes about the DP's own runs.
3. **The piece ends.** A cut makes two new pen-ups. `hover_clears` asks
   `writing.lifted_or_lower` for the hover the programme will actually lift to
   and measures it against the installed room; an end that will not hold one is
   walked **inward** (`_trim_to_hover`, up to 8 steps) rather than losing the
   stretch, and the stretch is given up only when nothing long enough is left.
4. **The certificate is still the gate.** The parts are re-planned through
   `plan_stroke` and re-gated by `ink_vs_envelope` like any other piece. The
   cut chooses *where*; a part whose fresh plan resolves the redundancy
   differently and lands back inside the room is refused again and deferred.
   What is deferred is now the refused **ink** rather than the line it was part
   of, so `(line, piece)` had to stay unique: a part takes a fresh id from
   `SPLIT_ID0` (1 000) upward, above any DP piece id.

**The bar for a new piece end is `PAIR_MARGIN`, not `paper.FRAME_FLOOR`, and
that correction was worth 100 % of the cut on stage A.** `FRAME_FLOOR` (63 mm)
is `STATIC_MARGIN` plus the *checker's* sweep residual and it is a **routing**
floor — `paper.effective_static_floor` already clamps it down to whatever a
leg's own endpoints can hold, precisely so a certified pose 51 mm from a
neighbour is not stranded by a bar its own endpoint cannot meet. Measured:
every hover over follower 31's clear stretches reads **+53.7 mm**, over the
arm-to-arm gate and under the routing floor, and at 63 mm the cut threw all four
of them away. A hover is a **pose the arm holds**, and `hold_gap` already judges
a held pose at `PAIR_MARGIN`.

**And +53.7 mm is the same number every time, which is itself the finding.**
Arm 31's hover over *any* of its own ink, at any rung of the ladder, stands
53.7 mm from leader 71's exact room — the same number as its park. A same-row
pair's shoulders are a fixed distance apart whatever their pens do, and on this
rig that distance is 53.7 mm of surface gap: legal to stand in (the 50 mm gate),
impossible to route through at the router's floor. That is §22's *"no pattern
may ever put a same-row pair in the air together"*, now with a number on it.

### 24.2 The final pass — three conductors, one per row

Deferred ink is by construction the contested strip between one row's two arms.
Rows are separated by the 0.40 m *y* dead band, and `docs/V2_WORKCELLS.md` §4b
measures cross-row pairs at **+194.3 and +201.1 mm** with each row drawing
inside its own band. So stage C runs as **three two-arm conductors** — 13+17,
31+71, 2+97 — in parallel processes: four priority orders each instead of 720,
a third of the pieces each, and the `sequence.cost_matrix` route screen (which
is the wall §22 measured, not the DP) paid three ways at once.

`partition_deferred` is the measurement that has to come first, and
`piece_row` is its rule: a piece goes to its arm's row **only if its whole
geometry stays inside that row's band**. The cross-row clearance was measured
with each row inside its own band; a piece straddling the dead band is not that
measurement's case and may never ride with a row conductor. Those go to a short
**stage D** instead — one conduct over whoever owns them when the share is small
(`BAND_SHARE_MAX`, 5 %), and Pete's own sequence when it is not: the outer rows
draw their band pieces while the middle row holds, then the middle row draws its
own.

**Measured on the CSAIL logo, before the cut landed** (from the lf2 programmes'
own deferred lists, `out/staged_csail_h097_program_lf2_*.json`):

| | split +0.15 | whole bag |
|---|---|---|
| deferred reaching stage C | 5.941 m, 21 pieces | 5.026 m, 15 pieces |
| row 0 (13, 17) | 0.000 m | 0.000 m |
| row 1 (31, 71) | **5.130 m** | **4.187 m** |
| row 2 (2, 97) | 0.232 m | 0.260 m |
| **dead band** | **0.579 m — 9.7 %** | **0.579 m — 11.5 %** |

Two things follow. The dead-band share is **above** the 5 % bar, so stage D is
a real stage rather than a footnote — and all of it belongs to row 1, so Pete's
two-phase sequence collapses to one phase here (the outer rows have no band ink
to draw). And the three row conductors are **profoundly unbalanced**: row 1 is
the whole job and row 0 is empty. The parallelism is therefore worth less than
3× on this picture; what it is worth is that the *conduct* stops being a
six-arm search, and that is the term `ARCHITECTURE_V2` §2d says does not scale.

**The merge is an assertion and the whole-timeline check discharges it.** Each
group is conducted alone, the groups overlap in time — that is the point — and
`_merge_conducts` pads every arm to the longest group's clock (an arm in no
group holds its pose, and is in the scene rather than absent from it) and runs
**one** `scene_check.check_timeline` over all six arms at `PAIR_MARGIN` with the
sweep residual. That check shares no code with any of this and is the verdict.

### 24.3 Which certificate is kept where

Two numbers disagreed on stage B of the §23 run — `ink_vs_envelope_mm` +35.7 mm
against realised trajectories at +98.6 mm — and a build that reports both and
believes neither is not a build. The rule kept here, stated once:

- **`ink_vs_envelope` is a certificate exactly where it is used as a GATE**,
  which is the follower's per-piece refusal and the split's clear-stretch test.
  A piece under the gate is refused, or cut and its clear parts kept.
- **Everywhere else it is a report.** A leader is *measured* against the rooms
  of the leaders before it and never refused on that number (§8), so a leader's
  reading is a proximity worth printing and not a verdict. `summary()` prints
  the stage minimum over both, which is why §23.4b read a leader's number as a
  failure it never was: `StageResult.ok` has never included it.
- **The stage's verdict is the realised-trajectory checks** — `active_pair_gap`
  over the arms that MOVE, `solo_check` per arm against the static set and the
  held poses, `hold_gap` at every barrier, and `scene_check.check_timeline` over
  a conducted stage's whole timeline. Those are what `StageResult.ok` is, and
  they are computed with the planner's room thrown away (`_check_stage` calls
  `thaw()` first).

### 24.4 What stage A measures, and what is still running

**The cut fires, and on stage A it is not enough.** Both settings, exact rooms,
seam bar in, CSAIL h = 0.970:

| stage A, arm 31 (the only follower with ink there) | split +0.15 | whole bag |
|---|---|---|
| pieces offered | 3 (2.142 m) | 8 (5.026 m) |
| refused at the ink gate, then CUT | 1 | **3** |
| certified clear stretches kept by the cut | **1 (0.157 m)** | **5 (0.468 m)** |
| …of the parents' ink | 0.328 m | 1.119 m |
| poses clear, per cut piece | 49.3 % | 44.3 / 38.9 / 46.3 % |
| hovers refused at the new ends (`PAIR_MARGIN` bar) | 0 | 0 |
| **follower ink flown** | **0.000 m** | **0.000 m** |
| stage A pieces / ink / busiest arm | 15 / 2.061 m / 95.9 s | 17 / 4.595 m / 128.6 s |
| stage A `active_pair_gap` / `solo_check` | +65.6 / +51.7 mm | **+78.7** / +51.2 mm |
| stage A verdict | **PASS**, 3/3 leader buckets | **PASS**, 3/3 leader buckets |

The parts are produced and then **dropped by the leg loop**: arm 31's bucket
produces no timeline with any subset of them, so `_fly_or_defer` sheds pieces
until the bucket is empty and defers the lot. That is the §22 leg problem
unchanged, and §24.1's +53.7 mm says why it is not a tuning matter — every pose
arm 31 can hold near its own ink stands 53.7 mm from leader 71's exact room,
over the 50 mm pose gate and under the 63 mm routing floor, so `paper.route`
has a knife edge and no slack to route in. **The cut removed the ink gate as
stage A's binding constraint and exposed the one underneath it.**

Stage B is where the cut should pay, because that is where a follower's bucket
already flies (62.6 % of its offered ink in §23.4b) and where a piece is refused
by the gate rather than stranded by its legs.

**STILL RUNNING WHEN THIS BOX CLOSED.** Both `lf3` runs were in the
`sequence.cost_matrix` route screen — stage B for split +0.15, stage A's drop
loop for the whole bag — which is the same O(n²) wall §19 and §22 flagged, and
which the cut makes *worse* per bucket because it adds pieces. Neither reached
stage C or D, so the per-row conductors are measured here only on the
partition (§24.2) and on a synthetic smoke run (three groups, 0.4 / 7.8 / 9.9 s
of wall in parallel, merged whole-timeline check binding on the same pair the
worst group did). **Pick-up:** `bash scripts/lf3_run.sh` re-runs both to
`out/staged_csail_h097_lf3_{s150,whole}.{json,log,_program.json}`; the stage
table and the makespan comparison against v19's 209.9 s and lf-whole's 273.3 s
follow from those JSONs.

**The 1 000-stroke model is NOT recalibrated**, for the same reason as §23.6 and
one more: stages A and B do not yet carry the majority of the ink (stage A's
follower still flies none), and the measured stage-C cost per metre this pass
was meant to replace §22's 3 075 s / 11.5 m with is exactly the number the runs
did not reach.

## 25. The leader's standoff — buying the follower its routing room

§24.4 left stage A's follower with **0 %** of its ink and named exactly one
number as the reason: **every pose arm 31 can hold near its own ink stands
+53.7 mm from leader 71's exact room**, at every hover rung and over every
x-y — over the 50 mm pose gate and under `paper.FRAME_FLOOR`'s 63 mm routing
floor, so `paper.route` has a knife edge and no slack to route in. The clear
stretches the cut produces are certified and then dropped by the leg loop.

That number is **constant in the follower's pose**, which says the binding
geometry is the follower's links that do not move. This section tests the one
lever that follows: **make the LEADER keep more.**

### 25.1 What the binding pair actually is — measured, not assumed

Leader 71's realised stage-A trajectory against follower 31's capsules at its
held pose, per capsule of the follower (minimum surface gap, mm):

| follower 31's capsule | base band 0 | band 1 | band 2 | **upper (1→3)** | elbow | forearm | wrist | hand | tool |
|---|---|---|---|---|---|---|---|---|---|
| gap to leader 71's plan | 207.0 | 243.8 | 192.2 | **103.2** | 314.2 | 331.4 | 613.3 | 560.1 | 519.8 |

The binding pair is the leader's **forearm** against the follower's **upper
arm** — the shoulder→elbow link, whose shoulder end is chain point 1, where the
base column ends and nothing moves. So the pose-invariant set this section
charges a standoff against is **every capsule both of whose chain endpoints are
in {0, 1, 3}**: the four base column bands (0→1) and the shoulder→elbow link
(1→3). That is "base column + link 0/1". The elbow link (3→4) is three times
further away and is deliberately **not** in it.

### 25.2 What was built — `frozen.set_standoff`, and no gate constant moved

`S` is an **ADDITIONAL requirement**, not a gate change. `PAIR_MARGIN` (50 mm),
`rig_final.STATIC_MARGIN` (50 mm), `selfcoll.SELF_PLAN_MARGIN` (23 mm) and
`paper.FRAME_FLOOR` (63 mm) are untouched, and a test pins all four.

`frozen.partner_clearance` returns

```
min( gap to everything , gap to THAT partner's pose-invariant capsules − S )
```

so every call site that compares the result against its own floor `f` is
thereby demanding `f + S` of the named set and `f` of everything else. One
seam, again: `plan_stroke`'s ink gate, `writing.arm_program`, `paper.route`'s
straight / ladder / go-around / RRT tiers and `paper.effective_static_floor`
all reach the partner model through `frozen`, so they all saw it at once, and
`scene_check` shares none of it and stays the judge.

**Who owes it.** `staged.lf_standoffs`: **a LEADER, and only to its SAME-ROW
FOLLOWER.** The leader is the arm that plans first and is certified at exactly
`PAIR_MARGIN` against the partner's held pose, so its plan is what eats the
room the follower then has to route in; and the pair with no room to give is
the same-row one, because a cross-row pair clears by +194 mm with each row
inside its own band (`docs/V2_WORKCELLS.md` §4b). Charging a cross-row partner
would cost a leader ink and buy a follower nothing.

**The keys move with it.** A standoff changes what `paper.route` grants without
changing one memo key, exactly as the frozen set and the envelope do, so it
goes into `leg_cache_signature` and `staged._room_key` — and **only when it is
non-empty**, so every key an S = 0 run builds is bit-identical to the key it
built before the standoff existed and a warm leg store still answers.

**S = 0 is a no-op and the run proves it.** `--partner-standoff 0.0` on CSAIL
h = 0.970, split +0.15, stage A reproduces the live baseline line for line:
15 pieces, 2.0613 m, 95.864 s, `active_pair` +65.58 mm, `solo` +51.66 mm, order
(71, 2, 13, 31, 17, 97), 3/3 leader buckets, PASS, arm 31's park +53.7 mm with
0/45 tuck stations clear, the same cut (1 clear part, 0.157 m, 49.3 % of poses
clear) and the same two drops. `tests/test_staged_standoff.py` holds the
mechanism down independently.

### 25.3 The ceiling — what ANY standoff could buy, before the sweep

A standoff can only move the leader's plan **toward** the case where the leader
draws nothing, whose room is the single held pose it starts the stage at. So
that case is the ceiling. Arm 31's 45 tuck stations (3 pieces × 3 x-y × 5
heights), against three versions of leader 71:

| leader 71 is… | arm 31's park | station min | median | max | ≥ 63 mm |
|---|---|---|---|---|---|
| its **full stage-A room** (S = 0, the shipped run) | **+53.7 mm** | −132.8 | **+53.7** | +53.7 | **0 / 45** |
| its **held pose only** (S → ∞) | +80.2 mm | −40.5 | **+192.0** | +195.7 | **39 / 45** |
| its **park** (never moved) | +80.2 mm | +135.2 | +241.0 | +350.0 | 45 / 45 |

**So +53.7 mm is a property of the leader's TRAJECTORY, not of the two arms'
fixed geometry.** With the leader standing still the follower's own hovers
clear the routing floor at 39 of 45 stations and a median of +192 mm. The room
is there to be bought; the question is the price.

### 25.4 What S forbids, and it is not ink

Leader 71's stage-A trajectory sample by sample against follower 31's
pose-invariant set — the set `set_standoff` charges — 451 samples, 417 drawing
and 34 pen-up:

| | min | p05 | median | max |
|---|---|---|---|---|
| whole trajectory | **+103.2 mm** | +235.5 | +350.0 | +350.0 |
| while **drawing** | +229.7 | | +350.0 | |
| on a **pen-up leg** | **+103.2** | | +193.6 | |

| S | bar = 63 + S | samples under the bar | % of trajectory | % of pen-up | **% of ink** |
|---|---|---|---|---|---|
| 0 | 63 mm | 0 | 0.0 % | 0.0 % | **0.0 %** |
| 70 mm | 133 mm | 12 | 2.7 % | 35.3 % | **0.0 %** |
| 90 mm | 153 mm | 12 | 2.7 % | 35.3 % | **0.0 %** |
| 110 mm | 173 mm | 13 | 2.9 % | 38.2 % | **0.0 %** |
| 130 mm | 193 mm | 17 | 3.8 % | 50.0 % | **0.0 %** |

**Not one of the leader's ink poses is inside the bar at any S in the sweep.**
The standoff is a constraint on the leader's **pen-up legs** and on nothing
else: it makes the router reroute a third to a half of them and can cost the
leader a piece only where a leg cannot be rerouted at all. That is also why the
runs below are slow — every one of those legs falls off the straight tier and
down into the ladder, the go-around and the RRT.

### 24.5 Split +0.15, the whole programme — and the merge does NOT compose

The run completed (`out/staged_csail_h097_lf3_s150.{json,log}`,
`_program.json`). The table Pete reads:

| stage | who | pieces | ink (m) | stage (s) | inter-arm, realised | solo | deferred out | verdict |
|---|---|---|---|---|---|---|---|---|
| A | lead 13/71/2, foll 17/31/97 | 15 | 2.061 | 95.9 | **+65.6 mm** | +51.7 mm | 2.142 | **PASS** |
| B | roles swapped | 7 | 1.610 | 24.0 | **+98.6 mm** | +73.7 mm | 5.939 | FAIL (not the gate — see below) |
| C | **3 row conductors, parallel** | 49 | 12.085 | 160.1 | **−191.9 mm** | — | 0.000 | **FAIL** |
| D | dead band, arm 31 alone | 2 | 0.315 | 27.6 | **+62.8 mm** | — | 0.000 | **PASS** |

| | value | against |
|---|---|---|
| makespan | **307.5 s** | v19 209.9 s, lf-whole 273.3 s |
| coverage | 95.63 %, 16.081 m of 16.805 m | unchanged |
| ink in A+B / C+D | **3.671 m (22.8 %) / 12.400 m (77.2 %)** | §22: 2.97 / 13.10 m |
| time to first motion | **0.196 s** | unchanged |
| planning wall, total / busiest arm | 1 980.2 s / 1 101.8 s | — |
| check | 49.7 s | — |
| held-pose barriers | +256.0 / +172.2 / +107.0 / +107.0 mm | all PASS, `holds_ok` |

**THE ROW CONDUCTORS ARE WORTH WHAT THEY WERE BUILT FOR, AND THEY DO NOT
COMPOSE.** The wall is exactly the win that was wanted: the three groups ran at
**39.9 s (13+17), 536.4 s (2+97) and 1 369.7 s (31+71)** concurrently, so stage
C cost **1 369.7 s** of wall for 12.085 m — against §22's six-arm **3 075 s for
11.458 m**. Per metre: **113 s/m against 268 s/m, a factor of 2.4**, and the
combinatorial 720-order search is gone. Each group also passed its own conduct's
clearance gate (+55.9, +inf, +72.8 mm).

**And the merged six-arm check refuses the stage at −191.9 mm.** Re-measured
straight off the programme with `scene_check.check_timeline` and nothing else:

| pair | mm | |
|---|---|---|
| **13 ↔ 31** | **−191.9** | **cross-row**, at t = 1.175 s |
| 17 ↔ 71 | **+44.6** | **cross-row**, also under the gate |
| 13 ↔ 17 | +55.9 | within row 0 — conducted |
| 31 ↔ 71 | +72.8 | within row 1 — conducted |
| 17 ↔ 31 | +110.8 | cross-row, fine |
| 13 ↔ 71 | +557.0 | cross-row, fine |

**Both failures are cross-row pairs, and the dead-band argument is what they
falsify.** §4b's +85.8 mm was measured with **one arm per row** in the air; the
final pass puts **two**, and 13 ↔ 31 at t = 1.175 s is the instant every arm
leaves its held pose for its first stroke at once. Rows are separated for
*drawing*; they are not separated for *four arms leaving four held poses
simultaneously*, and nothing in this build ever certified that — each group's
planning froze the other rows at their stage-C **entry** poses, which is true
only until those arms move.

`self` also fails, on arm 31 at **+16.2 mm** against the 23 mm gate. That is
independent of the merge — it is arm 31's own conducted trajectory folding
through itself on a pen-up leg, the hazard `writing.py` names in as many words —
and it would have failed a six-arm conduct too.

**THE FIX IS TO SERIALISE THE ROW CONDUCTS IN TIME AND KEEP THEM PARALLEL IN
WALL CLOCK**, which is what stage D already does and why stage D passes at
+62.8 mm: one group moves while the others hold at poses the planner froze them
at, which is exactly the assumption the planning made. The 2.4× planning win
survives untouched — it comes from the group size, not from the overlap — and
what is paid is stage C's motion becoming the SUM of the three rows rather than
the max. On this run that is 160.1 s becoming at most the sum of the three
groups' own durations. Until that lands, **stage C as merged here is not a
certificate and must not be animated or shipped.**

**Stage B's FAIL is not the ink gate and not a collision**, and §24.3's rule is
what says so: realised trajectories +98.6 mm, static set +73.7 mm, both well
over. `StageResult.ok` is `complete and pair_ok and solo_ok`, and the term that
is false is `complete` or `solo_check`'s non-clearance gates — not
`ink_vs_envelope_mm`, which `ok` has never read.

### 24.6 What the cut is worth, measured: nothing yet, on this picture

| split +0.15 | stage A | stage B | total |
|---|---|---|---|
| pieces the gate refused and the cut re-cut | 1 | 2 | **3** |
| certified clear stretches produced | 1 | 2 | **3** |
| new ends refused for want of a hover | 0 | 0 | **0** |
| **ink kept and FLOWN by the cut** | 0.000 m | 0.000 m | **0.000 m** |
| ink deferred as cut parts (instead of whole lines) | 0.326 m | 0.680 m | 1.006 m |

The cut produces exactly what it was specified to produce and **none of it
flies**: every part is shed afterwards, in stage A by `_fly_or_defer`'s leg loop
(the +53.7 mm shoulder wall of §24.1) and in stage B by the re-gate. Stage A's
piece and metre counts, stage B's, and the follower fit fraction (0.339) are
therefore **identical to the pre-cut run**, and the extra pieces cost **no**
pen-up legs, because none of them was kept.

**One thing did move.** Cutting at the room boundary also cuts pieces that
straddle a row band, so the dead-band share of the final pass fell from
**9.7 % to 5.3 %** (0.579 m → 0.315 m of 5.939 m) — which is still over the
`BAND_SHARE_MAX` bar, so stage D stayed a stage, and it is a 27.6 s stage that
passes.

**The 1 000-stroke model is NOT recalibrated**: A + B carry **22.8 %** of the
ink, not the majority, so the condition the build set is not met. The number to
carry forward instead is the measured final-pass cost, and it is the one real
improvement this pass bought: **113 s of wall per metre of deferred ink, against
§22's 268 s/m** — provided the composition is fixed, because the stage that
produced it does not pass.

### 25.5 The sweep — S ∈ {0, 70, 90, 110, 130} mm, CSAIL h = 0.970, stage A

`--partner-standoff`, exact rooms, seam bar in, the cut on, `--stages 0,1`.
**Stage B did not fit the 25-minute cap at any setting** — all eight A+B runs
were killed inside stage B's `sequence.cost_matrix` route screen, which is the
same O(n²) wall §19, §22 and §24 flagged and which the standoff makes worse per
leg because every re-routed leg falls through to the RRT. The stage-A numbers
below are from the stage-A-only re-runs (`--stages 0`, warm leg store), which
reproduce the A+B runs' stage A **line for line** at every setting; the A+B logs
are kept as `out/staged_csail_h097_lf4_S*_{s150,whole}_ab.log`.

**Split +0.15 m**, `out/staged_csail_h097_lf4_S{070,090,110,130}_s150.{json,log}`:

| S | pieces | stage ink | stage s | **leader ink flown** | follower offered | **follower flown** | fit | `active_pair` | `solo` | buckets | verdict |
|---|---|---|---|---|---|---|---|---|---|---|---|
| **0** | 15 | 2.061 m | 95.9 | 2.061 / 2.061 m | 2.142 m | **0.000 m** | **0.0 %** | +65.6 mm | +51.7 mm | 3/3 | PASS |
| **70** | 16 | 3.016 m | 77.0 | 2.061 / 2.061 m | 2.142 m | **0.955 m** | **44.6 %** | +64.7 mm | +107.5 mm | 4/4 | FAIL |
| **90** | 18 | **3.494 m** | **77.4** | 2.061 / 2.061 m | 2.141 m | **1.433 m** | **66.9 %** | +62.1 mm | +134.4 mm | 4/4 | **PASS** |
| **110** | 18 | 3.494 m | 75.1 | 2.061 / 2.061 m | 2.141 m | 1.433 m | 66.9 % | +61.5 mm | +157.3 mm | 4/4 | FAIL |
| **130** | 18 | 3.629 m | 70.1 | 2.061 / 2.061 m | 2.140 m | 1.568 m | 73.2 % | **+42.7 mm** | +172.4 mm | 4/4 | FAIL |

**Whole bag**, `out/staged_csail_h097_lf4_S{070,090,110,130}_whole.{json,log}`
(the S = 0 row is the live lf3 baseline):

| S | pieces | stage ink | stage s | **leader ink flown** | follower offered | **follower flown** | fit | `active_pair` | `solo` | buckets | verdict |
|---|---|---|---|---|---|---|---|---|---|---|---|
| **0** | 17 | 4.595 m | 128.6 | 4.595 / 4.595 m | 5.026 m | **0.000 m** | **0.0 %** | +78.7 mm | +51.2 mm | 3/3 | PASS |
| **70** | 17 | 4.595 m | 110.7 | 4.595 / 4.595 m | 5.018 m | 0.000 m | 0.0 % | +169.6 mm | +107.5 mm | 3/3 | PASS |
| **90** | 21 | 6.122 m | 110.6 | 4.595 / 4.595 m | 5.020 m | 1.526 m | 30.4 % | +51.1 mm | +134.4 mm | 4/4 | FAIL |
| **110** | 19 | **5.982 m** | **109.6** | 4.595 / 4.595 m | 4.981 m | **1.387 m** | **27.8 %** | +62.5 mm | +157.3 mm | 4/4 | **PASS** |
| **130** | 23 | 6.386 m | 105.1 | 4.595 / 4.595 m | 5.020 m | 1.790 m | 35.7 % | **−8.3 mm** | +188.3 mm | 4/4 | FAIL |

**The leader loses nothing.** `leader_ink_flown` is **2.061 m at every S** on the
split and **4.595 m at every S** on the whole bag — the same metres, the same
pieces, the same three buckets — which is §25.4's prediction landing exactly:
not one of the leader's ink poses was ever inside the bar, so the standoff was
only ever a constraint on where its pen-up legs may fly.

**And the follower's own clearance goes straight to the ceiling.** At every
S ≥ 70 mm the log reads `arm 31 [follower] tuck: park +80.2 mm, 1/1 stations
clear >= 73 mm` — up from **+53.7 mm** at S = 0, and **+80.2 mm is exactly the
S → ∞ value of §25.3**. The lever does not move the follower part of the way; at
70 mm it has already bought all there is.

**The wall was never the pair's fixed geometry. It was the leader's tour.**

### 25.6 The three failures, named

Of the six not-ok stages in the sweep, **only two are clearance failures and
both are over-provisioned S**:

| setting | why not ok |
|---|---|
| S = 130, split +0.15 | `active_pair` **+42.7 mm** — the realised trajectories are under the 50 mm gate. The follower took 73 % of its ink and the row closed up. |
| S = 130, whole bag | `active_pair` **−8.3 mm** — an actual interpenetration of the gate. Same mechanism, further. |
| S = 70 / 110 split, S = 90 whole | **not a clearance at all.** Both independent numbers are over their gates (pair +64.7 / +61.5 / +51.1 mm, solo +107.5 / +157.3 / +134.4 mm). `solo_check` refuses on its per-arm flag, and the case measured — S = 70, split +0.15, from `out/staged_csail_h097_lf4_S070_s150_stageA_program.json` — is **leader 71's HELD BARRIER POSE at 111 mrad of joint-1 margin against `scene_check`'s 150 mrad bar**, with frame clearance +69.3 mm, self +105.7 mm and tip z 60 mm all fine. The standoff moved the leader's tour, the tour ends on a different last stroke, and its exit hover is near a joint limit. That is a barrier-pose problem — `park_policy` and `hold_gap`'s business — and not a statement about the pattern. |

**So there is a real optimum in S and the sweep brackets it**: below it the
follower cannot route, above it the two arms close on each other. On this
picture it is **S = 90 mm on the split** and **S = 110 mm on the whole bag**.

### 25.7 The decisive number — the row, drawing concurrently

Row 1 is arms **31 and 71**, the same-row pair §22 §4b said may never be in the
air together. Stage A, at the best S:

| | split +0.15, **S = 90 mm** | whole bag, **S = 110 mm** |
|---|---|---|
| row 1's stage-A ink on offer (71's bag + 31's bag) | 3.979 m | 8.618 m |
| drawn in the stage with **both arms of the row flying** | **3.271 m — 82.2 %** | **5.024 m — 58.3 %** |
| …of which drawn in the window where **both are drawing at once** | **2.438 m — 61.3 %** (29.45 s of overlap) | **2.027 m — 23.5 %** (24.59 s of overlap) |
| the same two numbers at S = 0 | **0.000 m — 0.0 %** | **0.000 m — 0.0 %** |
| realised pair clearance while they do it | **+62.1 mm** (gate 50) | **+62.5 mm** (gate 50) |
| stage A duration | **77.4 s** (was 95.9 s, **−19.3 %**) | **109.6 s** (was 128.6 s, **−14.8 %**) |
| stage A ink | **3.494 m** (was 2.061 m, **+69.5 %**) | **5.982 m** (was 4.595 m, **+30.2 %**) |
| time to first motion | 0.243 s | 0.206 s |
| stage verdict | **PASS** | **PASS** |

**A same-row pair CAN be in the air together on this rig.** They draw 2.4 m of
the picture simultaneously at 0.61 m of base spacing, at +62 mm of realised
clearance against a 50 mm gate, and the leader gives up not one millimetre of
ink for it.

**The makespan line is owed a stage B and does not have one.** Against the
S = 0 A + B baselines of **119.9 s** (split +0.15 = 95.9 + 24.0) and **145.9 s**
(whole bag = 128.6 + 17.3), the best S already takes **18.5 s** and **19.0 s**
out of stage A alone, and it moves 1.43 m / 1.39 m of the followers' ink out of
the deferred pile that stage C would otherwise have to conduct — which is the
term §22 measured at 3 075 s of wall for 11.5 m. Stage B's own duration at the
best S is the number this box did not buy, and it is the only thing between
these tables and a makespan.

## 26. The row conductors compose — the room that was thrown away, and the gate the judge could not read

§24.5 measured the split-+0.15 programme and refused stage C at **−191.9 mm**
with `self` failing on arm 31 at **+16.2 mm**. It attributed the first to the
overlap — three row groups moving at once, each planned with the other rows
frozen at their stage-C entry poses — and proposed serialising the rows in
time. **Both halves of that attribution are wrong, and the measurements are
here.**

### 26.1 The rows were never frozen during the conduct at all

`idle.conduct` does not re-time the timelines `plan_bucket` built. It calls
`writing.arm_program` **again**, per arm, and **routes every pen-up leg again**
(`idle._programs`). `_conduct_stage` called `thaw()` between the two, so every
leg a row conductor emitted was routed against `paper.static_boxes`'s
pose-invariant base columns and nothing else — not against the four arms
standing outside the group, which were also not in the conduct and therefore
not scheduled against either. The four arms outside a two-arm group were
**invisible to both halves of the safety argument.**

Measured on the run's own programme, arm 31's stage-C trajectory against the
frozen set the planner *thought* it had (`frozen.partner_clearance`, the
planner's own seam, the five partners at their stage-C entry poses):

| pair | mm | t (s) | what is moving |
|---|---|---|---|
| 13 ↔ 31 | **−205.7** | 1.15 | both |
| **2 ↔ 31** | **−204.6** | **3.20** | **arm 2 is STANDING STILL** |
| 31 ↔ 97 | **−117.9** | 4.20 | arm 97 is standing still |
| 71 ↔ 97 | **−38.8** | 19.85 | arm 97 is standing still |
| 17 ↔ 71 | +35.2 | 94.45 | both |
| 31 ↔ 71 | +58.7 | 81.70 | both, and conducted |
| 13 ↔ 17 | +52.1 | 7.10 | both, and conducted |

**Three of the five failing pairs have a stationary arm in them**, and §24.5's
table did not list them because it quoted only six of the fifteen pairs. A
still arm cannot be fixed by serialising the rows in time: arm 2 holds that
pose for the whole of stage C under *every* composition. `frozen.partner_clearance`
reads **−165.3 mm at t = 3.20 s** on the same leg, which settles it — the
planner's own room was violated, so the room was not installed. Every one of
these is a **transit** (`seg = −1`); no ink is involved.

**One more thing the trajectory says, and §24.5 read it as a PASS.** Group
[2, 97] reported `+inf mm, PASS` and drew **nothing**: both arms produced 1.494 m
of accepted ink and no timeline at all, so the group's "clearance" was the
clearance of an empty scene. Stage C's real flown ink is **10.591 m**, not the
12.085 m of accepted pieces `StageResult.ink_m` reports.

### 26.2 What was built

1. **`staged.freeze_conduct`** — the arms a conduct does NOT move stay in the
   room *for the length of the conduct*, through the same `frozen` seam every
   planner gate already reads. `thaw()` moves to after `idle.conduct`, where it
   belongs (the check still inherits nothing). No observer is named: the movers
   are simply not in the frozen set, and `static_boxes` already drops a spec's
   own band.
2. **`--row-compose parallel | priority | serial`** (`staged.run(row_compose=)`),
   with `group_order` ordering the groups **busiest-first** exactly as
   `priority_order` orders the arms inside a stage.
   * **`parallel`** — the control. Every group starts at stage time zero and
     plans with the other groups STANDING at their entry poses. That is what
     the code always claimed and now actually does; it is still the assertion
     §24.5 refused.
   * **`priority`** — a pipeline. Group 1 is conducted free; group *k* is
     conducted against groups 1..*k*−1's **realised** trajectories, installed as
     `exact_room.ExactRoom` swept capsules through `frozen.freeze_sets`'
     `clusters` seam. A pair (i, k) with i < k is then certified against the
     path arm i actually flies, **at every instant** — if k's chain never enters
     i's swept volume, it clears i wherever i is. The wall becomes the SUM of
     the groups; the motion still overlaps.
   * **`serial`** — the rows run one after another in TIME and still plan at
     once: group *k* sees the earlier groups at their PARKS (`PARK_HOME` sends
     them there) and the later ones at their entry poses, so every group's room
     is known before any of them runs. `_merge_conducts(offsets=)` lays the
     groups end to end on one clock.
3. **The t = 0 proof.** `_merge_conducts` now runs `hold_gap` over the merged
   timeline's first frame and **fails the merge** if the six entry poses are not
   pairwise clear (`t0_holds` in the conducted report).
4. **`--stage-c-only PROGRAMME.json`** (`staged.run_conducted`) — the final pass
   re-run off an existing programme: the entry poses are the barrier's `holds`
   and the ink is the pieces the conducted stages carry. A piece the original
   conduct REFUSED is written out without its geometry, so a re-run is offered
   the accepted set (1 piece of 50 on this programme, and a piece `plan_stroke`
   refused once it refuses again).

### 26.3 The self gate was a DENSITY disagreement, not a geometry one

Arm 31's `self` failure is independent of all of the above and it is not a
fold. The leg — stage-C transit `seg 13`, worst at t = 86.4 s — has a true
minimum self-clearance of **26.12 mm** over 401 dense samples, which clears the
**checker's 20 mm** *and* the **router's 23 mm** `SELF_PLAN_MARGIN`. The whole
of the difference is arithmetic:

| term | value |
|---|---|
| true self-clearance of the leg | **26.12 mm** |
| capsule-endpoint step between two playback frames (dt = 0.05 s, `sub` = 2) | 18.06 mm |
| `scene_check`'s 1-Lipschitz residual, 0.55 × that | **9.93 mm** |
| what the judge reads | **+16.18 mm** — FAIL against 20 mm |

**The producer and the judge were answering different questions.** `paper`
converges its bound on its own adaptive samples (`adaptive_lb`, residual driven
to `ADAPT_TOL`) and holds the geometry to 23 mm; `scene_check` samples the
timeline that is actually written down and subtracts a residual that grows with
the **speed**. 23 mm is 3 mm over the judge's 20, and at 0.55 that 3 mm buys
only **5.45 mm of frame step** — a pen-up flown at the shipped transit speed
puts three times that between two frames. No gate constant is wrong.

**THE FIX IS PACE, NOT GEOMETRY AND NOT A REFUSAL.** `writing.self_pace_beat`
prices, for each pen-up beat, the residual its own speed implies: between two
playback frames no capsule endpoint travels further than `rate × dt_play / dt`,
so requiring `lb − 0.55 × rate × dt_play / dt ≥ 20 mm` solves for `dt`
directly. The beat is **stretched** until its bound fits and nothing else
moves — the route is the same route, the poses are the same poses, the ink is
untouched. Only the beats that owe the residual pay it; a leg whose true
clearance is already under the judge's margin cannot be rescued by any speed
and is left for the judge to refuse, which is the right division of labour.
On the pinned leg: 0.05 s → 0.0969 s (1.94×), and +16.33 mm → **+21.28 mm**.

The three numbers `writing` needs (`SELF_PLAY_DT` 0.025, `SELF_PLAY_K` 0.55,
`SELF_PLAY_FLOOR` 0.020) are **restated** in `writing.py`, exactly as
`validate.py` restates `selfcoll.SELF_MARGIN`: a producer that reached into the
judge for its own floor would be marking its own homework.

### 25.8 One A + B did fit — the whole bag at S = 110 mm

`out/staged_csail_h097_lf4_S110_whole_ab.{json,log}`, `--stages 0,1`,
`--route-jobs 4`, warm leg store, 785 s of wall:

| stage | pieces | ink | duration | leader flown | follower flown / offered | `active_pair` | `solo` | verdict |
|---|---|---|---|---|---|---|---|---|
| A | 19 | **5.982 m** | **111.2 s** | 4.595 / 4.595 m | **1.387 / 4.981 m** | +56.1 mm | +157.3 mm | **PASS** |
| B | 0 | 0.000 m | 0.0 s | — | — | — | +256.0 mm | **PASS** |

| | S = 0 (lf3) | **S = 110 mm** |
|---|---|---|
| **A + B makespan** | **145.9 s** (128.6 + 17.3) | **111.2 s — 0.76×, 23.8 % faster** |
| ink drawn in A + B | 4.623 m | **5.982 m (+29.4 %)** |
| **deferred to stage C** | **10.023 m** | **7.188 m (−2.835 m)** |
| follower ink kept | 0.000 m | **1.387 m (27.9 %)** |
| time to first motion | 0.192 s | **0.184 s** |
| `all_ok` / `holds_ok` | true / true | **true / true** |
| coverage of the logo | 95.64 % | 95.64 % |

**Stage B is EMPTY, and that is the shape of the win.** The whole bag's ink that
can fly at all now flies in stage A with both arms of the row in the air, so
there is nothing left for the role swap to draw — the pattern collapses from two
main stages to one, and the 17.3 s stage B disappears along with the barrier
before it. Stage C is handed **2.835 m less**, which at §22's measured
268 s/m of six-arm conduct (or §24.5's 113 s/m per-row) is the larger half of
what this lever is worth and is not in the 111.2 s at all.

**The split +0.15 run did not get its stage B**: `S090_s150_ab` was killed by
the same 25-minute cap in stage B's route screen, so the split's A + B makespan
against 119.9 s is still owed. Its stage A at `--route-jobs 4` reads 18 pieces,
3.494 m, 86.9 s, `active_pair` +62.0 mm, `solo` +134.4 mm, PASS — the same
pieces and the same metres as the route-jobs-1 table above, with the tour's
tie-breaks costing 9.5 s of stage duration.

## 27. Where the pen-up time goes — the anatomy, and the three things that were wrong

Pete, watching the animation: *"arm 71 is still spending an insane amount of
time reconfiguring."* He is right, and on
`out/staged_csail_h097_program_lf2_s150.json`, stage A, arm 71 (13 pieces,
1.838 m of ink, 95.9 s) the measurement is:

| | |
|---|---|
| pen-up legs | **70.5 s of 95.9 s — 73.6 %** |
| ink + hovers | 25.3 s |
| peak joint speed on almost every leg | 0.60 of the FR3 limit (`QD_FRAC` is in force) |

**So pacing is not the cause.** The pen-up legs are slow because of what they
do, not how fast they are allowed to do it. `scripts/penup_anatomy.py` was
written to say what that is, and it runs on any staged programme:

```
ARIS_RIG=proposed ARIS_TOOL=lateral .venv/bin/python scripts/penup_anatomy.py \
    out/staged_csail_h097_program_lf2_s150.json --legs
```

### The rule it classifies on, stated once

For each pen-up leg, off the programme's own trajectory: `dq` = joint path
(Σ|Δq| over samples and joints), `net` = Σ|q_end − q_start|, `hop` =
‖xy_end − xy_start‖, `ztrav` = Σ|Δz| of the tip, `zmax` = the highest tip z.
An honest leg lifts `LIFT_Z`, crosses `hop` and lowers, so its honest vertical
travel is `ZTRAV_REF = 0.12 m` and its honest joint path is about
`RATE_REF × (hop + ztrav)`. `RATE_REF = 25 rad/m` is calibrated on the legs that
ARE honest — they sit at 11–20 rad/m — and set deliberately high so only clear
outliers are named. Then

```
climb_excess = max(0, ztrav − 0.12)          m of pointless climbing
climb_rad    = RATE_REF × climb_excess       joint work the climb explains
flip_rad     = max(0, dq − RATE_REF × (hop + ztrav))
                                             joint work NOTHING explains

flip    if  flip_rad ≥ 6 rad and flip_rad > climb_rad
tall    elif climb_excess > 0.06 m or zmax > 0.13 m
             or (dq ≥ 6 and dq > 2 × net)
honest  otherwise
```

`flip` is tested first on purpose: a leg whose tip barely moves while the joints
churn is a redundancy fault whatever its vertical profile, and `flip_rad`
already has the climbing subtracted out, so a merely tall leg cannot be called a
flip. The classifier is stricter than a hand read — arm 71's legs 4 and 5 come
out "honest" at `flip_rad` 3.0 and 0.1, under the 6 rad floor — and that is the
right way round for a number that is going to be quoted.

### The measurement, on the lf2 s150 programme, every arm and both stages

| st | arm | ink m | stage s | pen-up s | % | flip s | tall s | honest s |
|---|---|---|---|---|---|---|---|---|
| 0 | 2 | 0.153 | 11.08 | 0.35 | 3.1 | 0.00 | 0.00 | 0.35 |
| 0 | 13 | 0.070 | 4.13 | 0.25 | 6.0 | 0.00 | 0.00 | 0.25 |
| **0** | **71** | **1.838** | **95.86** | **70.54** | **73.6** | **28.72** | **30.86** | **10.96** |
| 1 | 13 | 0.728 | 13.05 | 0.23 | 1.8 | 0.00 | 0.00 | 0.23 |
| 1 | 71 | 0.853 | 24.02 | 9.58 | 39.9 | 0.00 | 6.87 | 2.72 |
| 1 | 97 | 0.028 | 17.30 | 4.80 | 27.7 | 0.00 | 4.80 | 0.00 |
| | **fleet** | | | **85.75** | | **28.72 (33.5 %)** | **42.52 (49.6 %)** | **14.51 (16.9 %)** |

**Five sixths of the fleet's pen-up time is one of two faults.** And because
schema-2 programmes carry `hover_in` / `hover_out` per piece, each leg splits
into LIFT (ink end → hover out), TRAVEL (hover out → hover in) and LOWER (hover
in → ink start), which says WHICH fault:

```
leg   lift   travel  lower      hop
  1   2.16    16.50  11.73     0.337    piece-to-piece sheet change
  6   1.39    11.07  10.04     0.215    piece-to-piece sheet change
  7   9.65     1.27  10.26     0.192    HOVER off the ink's sheet
  8  10.75     0.55  10.20     0.053    HOVER off the ink's sheet
  9  10.04     0.04   9.74     0.003    HOVER off the ink's sheet
 10  10.05     1.29   9.83     0.195    HOVER off the ink's sheet
 11  10.09     1.07   9.61     0.113    HOVER off the ink's sheet
```

**66.0 rad of the arm's flip cost is the hover, and 28.2 rad is the pieces.**
Leg 9 is the cleanest statement of it: the arm spends 3.5 s lifting the pen 6 cm
and 3.5 s putting it back down 3 mm away, and moves 19.8 rad of joint path
doing it, because the pose it lifts ONTO is 10 rad from the pose it lifts FROM.

### Fault 1 — the hover was chosen for clearance and never for nearness

`writing.hover_solve` has two stages: a narrow scan (one tool yaw, q7 within
±0.6 of the ink's own) and, where that fails, the whole fiber — 8 yaws × the
whole q7 grid × every IK branch. The escalation test was **comfort alone**:

```python
if q is not None and (cap is None or score(q) >= cap - EPS):
    return q                       # ← returns a pose 5.03 rad away
```

Asked about arm 71's piece 8: the narrow scan returns a pose that clears the
static set comfortably and is **5.03 rad** from the ink — an IK branch flip
inside the ±0.6 q7 window — so the fiber never opens. The fiber holds **23
gated poses at that tip and height, and the nearest is 0.26 rad away.**

The fix is `HOVER_NEAR = 1.0` rad: a narrow answer that is comfortable but
further than that opens the fiber too, and the two answers are then compared on
the SAME score with distance breaking the tie the score leaves (every
comfortable candidate ties at the cap, which is what `lifted_config`'s
sorted-then-argmax already relies on). **Nothing is relaxed.** Both paths run
the identical `static_gate` — `CHAIN_CLEAR`, the full `FRAME_FLOOR`,
`selfcoll.self_ok` — and the identical joint-margin filter. Continuity is a
tie-break among poses that are already certified.

Measured over arm 71's 13 pieces, both ends of each:

| | lift + lower distance, Σ‖Δq‖∞ | wall |
|---|---|---|
| `HOVER_NEAR = ∞` (the old short-circuit) | **48.39 rad** | 0.4 s |
| `HOVER_NEAR = 1.0` | **16.76 rad — −65.4 %** | 0.4 s |

Ten of the twenty-six ends were on the wrong sheet; one still is (piece 1's
exit, where the fiber genuinely has nothing nearer).

### Fault 2 — the sheet was chosen piece by piece, never along the tour

`stroke_api.plan_stroke` returns one plan and picks its (φ, q7, branch) sheet on
that piece's own merits (maximin σ). Adjacent pieces in a tour therefore land
on different sheets and the transit folds the arm over between them — legs 1 and
6 above, 27.6 rad of travel between them.

The repo already had the exact machinery: `menu.stroke_menu` enumerates a
stroke's certified entry/exit fiber variants, and `allocate.sequence_arm_cluster`
+ `sequence.cluster_held_karp` decide order, direction and fiber in one DP.
**It was built, flag-gated off as `allocate.CLUSTER`, and is not reachable from
`staged.py` at all** — the verdict that gated it (`docs/CONCURRENCY.md`
2026-08-20, `docs/BENCH.md` +3.4 % at rig speed) was taken on the old rig with
long strokes and a `qd_frac` of 0.30.

**It is still not the right shape for the staged path, for three reasons that
have nothing to do with that verdict.** The staged bucket's order is decided by
`sequence_arm` against a frozen room; `plan_bucket` re-plans the same bucket
several times under the drop-and-defer loop, and the cluster DP's second
`cost_matrix` pass on every retry is the cost centre that killed stage B under
its wall cap (§25); and `sequence_arm_cluster` re-materialises its choice
**without ever re-asking `staged.ink_vs_envelope`**, so a piece the room refused
could come back certified.

So the staged path gets the same idea in the cheaper shape a fixed order allows.
`allocate.chain_sheets` is a **Viterbi over pieces × alternatives**, O(n·K²)
instead of O(2ⁿ·n·K²), run after `sequence_arm` has fixed the tour:

* alternative 0 is the plan the bucket already certified, so the worst case is
  the programme it was handed;
* the others are `menu` variants, which advertise their exact entry and exit
  configurations **without planning anything** — so the whole DP runs on
  endpoints and only the chosen variants are ever materialised;
* a chosen variant is materialised through `stroke_api` (identical σ, margin and
  independent-validator certificate) and then put back through `plan_bucket`'s
  own ink-vs-room test; a refusal kills that alternative and the DP re-solves
  (3 rounds), falling back to alternative 0;
* the edge is `writing.transit_time`'s three beats — the **real capped transit
  time**, the number `sequence.cost_matrix` prices a crossing at and
  `writing.arm_program` lays down — with the depot legs substituted where the
  tour goes home;
* a variant's extra interior draw time (`menu`'s `surcharge`, capped at 0.05 s)
  is charged on the node, so continuity cannot be bought with ink time.

Menus cost 0.05 s per piece and a materialisation 0.035 s, measured on arm 71's
pieces, so K = 4 is ~0.2 s per piece.

### Fault 3 — the router's first draft was its last, and the depot jumped the queue

`paper.route` returns the FIRST shape that certifies, and the shapes that
certify most often are the ones with the most vias (`walk`/`traverse` lay one
down every 12–30 cm of paper). Two changes, both inside `route`:

**The depot via is no longer tried before the ladder.** `[q_home]` — the park
pose, 42 cm up on this rig — was offered as a shape before the 8 cm rung. It is
still offered, after the ladder, which is what "the lowest via that clears"
means (`HOME_AFTER_LADDER`).

**Every tier's output is shortcut before it is returned or stored.** Vias are
dropped one at a time, the drop that saves the most joint-space time wins each
pass, and a drop is kept ONLY when the shortened shape is re-certified end to
end by `legs_ok` — the same bound, the same floors, the same self and static
gates as the shape it replaces. A shortcut can therefore never be looser than
what it replaces; it can only be shorter. A drop that saves no time is not taken
(`paper.SHORTCUT`, `SHORTCUT_ROUNDS = 8`). `transit._shortcut` already did this
for the RRT tier's output and nothing else.

**The persistent leg store is namespaced on all of it.** `paper.ROUTE_REV`
rides in `paper.cache_signature()`, so a warm store cannot serve pre-change
routes, and `writing.HOVER_NEAR` is in `lifted_or_lower`'s memo key for the same
reason. The re-run below is therefore COLD on every route.

### The first re-run said FAIL, and it was right to

`out/staged_csail_h097_lf5_s150_v1.*` — the three fixes above, nothing else.
Stage A came out at **55.41 s against 95.86**, over **more** ink (16 pieces,
2.4165 m, because arm 31's follower bucket cut at the room boundary and flew
0.355 m the baseline could not), with `active_pair` +138.6 mm and `solo`
+167.9 mm against +65.6 and +51.7. **And the stage verdict was FAIL.** That run
is kept, because the thing it was refused on is the fourth bug.

**THE STAGE VERDICT WAS FAIL, AND `frozen_failed` IS NOT WHAT ITS NAME
SUGGESTS.** Re-deriving arm 71's `solo_check` off the shipped programme
(1 109 frames at dt = 0.05):

```
ok False          min_clearance  0.16794 m      monotone True
frame_failed []   paper_failed []   self_failed []   column_failed []
frozen_failed 1   joint_margin  arm 71  0.1107 rad (all six > 0)
```

Every distance gate passes and passes wide. **`scene_check`'s `frozen` term is
not `frozen.partner_clearance` and has nothing to do with the standoff**: it is

```python
rep_p = validate_pose(np.asarray(qtraj[a], float)[-1], fl[a], h_inv, ...)
```

— `validate_pose` on the **last pose of each arm's trajectory**, the pose the
arm HOLDS when the stage ends. And under `PARK_FREEZE` that pose is the hover
over the last stroke. Running it directly:

| | arm 71's final held pose |
|---|---|
| joint margin | **0.11108 rad, joint 0, against `validate.MARGIN_GATE` 0.15** |
| frame clearance | 69.3 mm |
| self clearance | 105.7 mm |
| chain z | 106.0 mm |
| tip z | 60 mm |
| hard violations | `['margin']` — **and nothing else** |

**So it is neither of the two things it was suspected of being.** It is not the
shortcutter (`paper_failed` and `self_failed` are empty at the judge's own
density). It is not the standoff: the run carried `--partner-standoff 0.0`, arm
71's serialised `standoff` is `None`, `frozen.set_standoff` was never called in
the run or in the re-check, and the two therefore agree at S = 0 — and in any
case `partner_clearance` is not the function this verdict comes from.

**IT IS A MARGIN MISMATCH THAT WAS LATENT AND THIS CHANGE EXPOSED.**
`writing.HOVER_MARGIN` is 0.10 rad and its own comment has said, since long
before this work, exactly what was wrong:

> Looser than `validate.MARGIN_GATE` (0.15) on purpose and historically: a
> hover is a place to stand, not a curve to be dragged along at a commanded
> speed. **Callers that are CHOOSING a pose rather than accepting one** — the
> idle policy's retreat — **ask for the stricter gate instead**, because there
> is no reason to spend margin you do not have to.

The widened fiber scan is exactly such a caller and was not doing it. The pose
at 0.1111 rad was always in the fiber and always passed `HOVER_MARGIN`; what
`HOVER_NEAR` changed is that the fiber's NEAREST pose became winnable, and the
nearest one happened to be the one standing 39 mrad inside the hold gate.

**THE FIX.** `writing.HOVER_HOLD_MARGIN = 0.15` (restated from
`validate.MARGIN_GATE`): the widened scan asks for the hold margin first and
settles for `HOVER_MARGIN` only where the fiber has nothing that keeps it — the
same ask-then-settle shape `HOVER_COMFORT` already uses for clearance. **It
cannot lose a hover** (the fallback is the identical old scan) and **it cannot
admit one** (both asks run the same `ok`). `ROUTE_REV` goes to 3 and
`HOVER_HOLD_MARGIN` joins `lifted_or_lower`'s memo key.

### The diagnosis held, and the certified re-run

`out/staged_csail_h097_lf5_s150.{json,log,_program.json}` — the three fixes plus
`HOVER_HOLD_MARGIN`, `--stages 0`, `--route-jobs 5`, cold on every route:

| stage A | lf2 s150 (before) | **lf5 s150 (after)** |
|---|---|---|
| **stage duration** | **95.86 s** | **55.97 s — 0.58×, 41.6 % faster** |
| **stage verdict** | PASS | **PASS** (all six arms' `solo` ok) |
| pieces, ink | 15, 2.0613 m | **15, 2.0613 m — identical** |
| arm 71 | 13 pieces, 1.838 m | **13 pieces, 1.838 m — identical** |
| **arm 71 pen-up** | **70.54 s (73.6 %)** | **27.04 s (48.3 %) — −61.7 %** |
| arm 71 flip legs / tall legs | 4 / 5 | **2 / 1** |
| arm 71 planning wall | 7.3 s | **21.3 s (2.9×)** |
| arm 2 stage / planning | 11.08 s / 0.8 s | 13.06 s / 4.9 s |
| arm 13 stage / planning | 4.13 s / 1.1 s | 4.13 s / 0.8 s |
| `active_pair` | +65.6 mm | **+196.7 mm (3.0×)** |
| `solo` min over arms | +51.7 mm | **+168.7 mm (3.3×)** |

**The ink is identical and the time is 0.58×.** The 0.355 m of follower ink the
FAIL'ing run picked up does NOT survive the hold margin — arm 31's bucket is
back to not flying — so the honest statement of this change is **the same
drawing, 39.9 s sooner, with every clearance reading three times better**, and
not the coverage win the intermediate run appeared to offer.

**THE TALL LEGS ARE GONE COMPLETELY.** Every one of arm 71's pen-up legs now
reads `ztrav = 0.120 m` and `zmax = 0.060 m` — the honest lift-and-lower and
nothing more — against four legs at 0.64–0.79 m of vertical travel and 0.32–0.42 m
of peak height before. The ladder and the shortcutter did their whole job.

**WHAT IS LEFT, NAMED.** Three legs carry almost all of the remaining 27.04 s:

```
leg   s      dq     lift  travel  lower   hop     kind
  2   4.54   14.38  2.39  10.73   1.26    0.192   flip  -- piece-to-piece sheet
 10   4.57   15.11  1.88  11.23   2.00    0.275   flip  -- piece-to-piece sheet
 12   4.10   15.12   --     --     --     0.000   flip  -- the go-home fold
```

The hover half of the flip cost is fully paid; what remains is the **piece-to-
piece** half on two crossings, and the **go-home** fold at the end. The chain DP
has fewer certified alternatives to work with now that the hold margin narrows
the fiber, which is the honest price of the fix. The go-home leg is a different
problem again — it is the park pose, not a sheet choice.

**PLANNING GOT 2.9× DEARER FOR ARM 71 (7.3 → 21.3 s), AND THAT IS NOT THE
MENUS.** The chain DP costs ~0.3 s a piece, or ~4 s for thirteen. The rest is
`HOVER_HOLD_MARGIN`'s fallback scan (a second 500-solution fiber pass wherever
nothing on the fiber keeps 0.15 rad) and `HOME_AFTER_LADDER` making a refused
crossing walk all seven rungs before reaching the depot via. Twenty-one seconds
of planning to save forty of stage time is a good trade for a fixed programme
and a bad one for an interactive loop; if it needs to come down, the fallback
scan is the place to look first.

### 26.5 Stage C re-run, three compositions — measured

`--stage-c-only` on `out/staged_csail_h097_lf3_s150_program.json`, 12.085 m of
ink in 49 pieces plus 0.315 m of dead band, every number from
`scene_check.check_timeline` over all six arms at `PAIR_MARGIN` with the sweep
residual, `sub` = 2:

| | parallel (§24.5, as shipped) | **serial** | priority |
|---|---|---|---|
| stage-C ink FLOWN | 10.591 m of 12.085 m | **12.085 m — all of it** | 8.850 m of 12.085 m |
| buckets flown | 4 of 6 | **6 of 6** | 2 of 6 |
| stage-C motion | 160.1 s | **212.4 s** | 159.6 s |
| inter-arm, six arms | **−191.9 mm** (13↔31, t = 1.18 s) | **+49.9 mm** (71↔97, t = 32.5 s) | **+49.9 mm** |
| self | **+16.2 mm** (31) | **+14.4 mm** (71) | **+14.4 mm** (71) |
| frame | +54.6 mm | **+58.8 mm** | +58.8 mm |
| paper (chain / tip) | +22.1 / −7.7 mm | **+39.1 / −6.1 mm** | +39.1 / −6.1 mm |
| t = 0 holds | +107.0 mm | **+107.0 mm** | +107.0 mm |
| stage D | 27.6 s, PASS | 22.4 s, **frame +31.0 mm FAIL** | 22.4 s, same FAIL |
| conduct wall | 1 369.7 s (max of 3) | **1 184.8 s (max of 3)** | 2 398.6 s (SUM of 3) |
| **A+B+C+D makespan** | 307.5 s (uncertified) | **354.6 s** | 301.8 s (26.8 % of the ink missing) |

**PRIORITY IS STRICTLY WORSE AND THE REASON IS THE SIZE OF THE ROOM.** Group
[31, 71]'s realised stage-C trajectory is a **31 920-capsule** swept volume, and
routed against it neither of the other two groups can produce a timeline at all
— `[13, 17]` spends 353.7 s and flies nothing, `[2, 97]` spends 1 074.9 s and
flies nothing. The composition is sound (the pair certificate holds at every
instant) and the geometry will not take it: the busiest row's swept volume is
most of the airspace over the paper. Serial gets the same +49.9 mm for **all**
the ink and **half** the wall, because a group that plans against the earlier
rows at their PARKS is planning against four poses rather than a tour.

**THE COMPOSITION DEFECT IS CLOSED.** The cross-row pairs that refused §24.5's
merge — 13↔31 at −191.9 mm and 17↔71 at +44.6 mm, plus the three pairs that
table never listed (2↔31 −204.6, 31↔97 −117.9, 71↔97 −38.8) — are all clear.
What is left is **0.09 mm** of the 50 mm inter-arm gate on one pair at one
instant, and one `self` leg. Neither is a composition failure.

**AND THE `self` LEG IS THE SAME LESSON TWICE.** Arm 71's transit `seg 3` reads
**23.60 mm raw** with a 16.65 mm playback step, so the judge sees +14.4 mm and
`self_pace_beat` did not stretch it: the beat is an 11 s leg, and the pacing's
own lower bound is `interval_bounds` on **33 samples**, whose per-interval
residual over a leg that long swallows the 3.6 mm of headroom the geometry
actually has. §"the certificate was refusing, not the geometry" (2026-09-09)
solved exactly this for the static gate with `adaptive_lb`; the pacing bound
has to be the **converged** one (`paper.leg_self_lb`) rather than a fixed grid.
That is one call, it can only ever raise the bound, and it is named here rather
than shipped unmeasured.

**Stage D's `frame` FAIL is new and it is not the composition either.** Arm 31's
dead-band conduct now routes with the other five arms in the room, takes a
different path, and stands **31.0 mm** from the frame against `STATIC_MARGIN`'s
50. Under the old code that conduct was routed against the base columns alone
and passed at +62.8 mm — the gate did not change, the route did, and this is the
`FRAME_FLOOR` routing floor being clamped by `effective_static_floor` where the
arm's own endpoints cannot hold it.

### CORRECTION (2026-09-14): the hold margin is OFF, and the 55.97 s PASS is withdrawn

`tests/test_staged.py::test_staged_end_to_end_on_a_three_stroke_picture` is
pinned at `PAIR_MARGIN`, and it FAILS at 5cdda73. Verified in a clean worktree
checked out at that commit, then bisected over the five knobs one at a time:

| knob left ON (others off) | pinned test |
|---|---|
| `HOVER_NEAR` | pass |
| `CHAIN_SHEETS` | pass |
| `paper.SHORTCUT` | pass |
| `paper.HOME_AFTER_LADDER` | pass |
| **`HOVER_HOLD_MARGIN = 0.15`** | **FAIL — arm 71 solo 45.95 mm vs the 50 mm gate** |

and with `HOVER_HOLD_MARGIN` off and the other three ON, it passes. **The three
fixes of 9d55ab8 are clean; the fourth is not.**

**WHY, EXACTLY.** The two asks land on different exit hovers — joint margin
0.136 at `None` against 0.576 at 0.15 — and the tight sample is neither pose: it
is a pen-up leg 0.36 rad out of the strict hover, on the way to the depot. Solo
clearance reads **62.61 mm** with the hold margin off and **45.95 mm** with it
on.

**THE TWO REQUIREMENTS GENUINELY CONFLICT ON THAT POSE.** The arm's last hover
can keep 0.15 rad of joint margin — and its go-home leg passes 45.95 mm from
parked arm 31 — or it can keep 50 mm of room and fail `validate_pose`'s margin
gate, which is the `frozen_failed` diagnosed above. It cannot do both *from this
scan*, and the reason is structural: **the hover score has never asked about the
frozen partners.** `static_gate` scores a candidate on `paper.chain_screen`
against the steel and the base columns, on the paper, and on self — and nothing
else. The parked ARMS live in `frozen`, which only `paper.route`'s `legs_ok`
consults, and by then the pose is already chosen.

**SO `HOVER_HOLD_MARGIN` DEFAULTS TO `None`** (the old scan, exactly), because a
hover may not be accepted at a clearance the old rule would have refused. The
consequences, stated plainly:

* **the 55.97 s PASS above is withdrawn.** It was produced with the hold margin
  on. What ships is the 9d55ab8 configuration, whose measured stage A is the
  **55.41 s FAIL** of `..._v1.*` — the three fixes' timing win is real and its
  certificate is not.
* **`frozen_failed` on arm 71's final held pose is open again**, now fully
  diagnosed rather than merely observed.
* **the fix is named**: give `static_gate` a `frozen.partner_clearance` term, so
  the search can find a pose that holds joint margin AND room, instead of a
  filter that trades one for the other. That is a change to a SCORE, not to a
  gate, and it is the next piece of work here.

### 27.1 The hover score can see the room now, and the conflict is gone

The correction above named the fix; this is it. **`static_gate` gained a
`frozen.partner_clearance` term**, in exactly the shape everything else in that
score already has:

```python
if rfl is not None:                      # room_floor, opt-in
    P = paper.world_chain(Q, spec, pen_ext, h_inv)
    C = paper.sphere_centres(Q, spec, h_inv)
    room = frozen.partner_clearance(P, C)
    good &= room >= rfl - paper.EPS               # a FLOOR: unacceptable under it
    val = np.minimum(val, np.minimum(room, rcap)) # a CAPPED term: prefer more
score.cap = cap if rcap is None else min(cap, rcap)
```

So the search **finds** a pose that holds the joint margin AND the room, instead
of a filter that trades one for the other. Three properties make it safe:

* **it is a score and a floor among CERTIFIED candidates, not a gate.** Every
  candidate still passes the identical `CHAIN_CLEAR`, `FRAME_FLOOR`,
  `selfcoll.self_ok` and joint-margin tests. No gate constant moved.
* **it is opt-in.** `room_floor=None` is the default and means no partner query
  at all — every legacy rig, every travelling hover, bit for bit as before.
* **`score.cap` is the MIN of the two caps**, so "comfortable" means comfortable
  on both terms; otherwise the fiber would never open for the room.

**AND IT IS ASKED OF ONE POSE.** `writing.arm_program` asks for it once, for the
last exit hover, and only under `PARK_FREEZE` — the pose the stage ENDS on and
holds through the barrier, which is the only pose `scene_check` puts through
`validate_pose`. `hover_solve` has **no module default** for the hold margin any
more; it reads its own argument and nothing else, so a travelling hover is
solved exactly as it always was. The ask is best-effort: if nothing on the whole
fiber holds both, the ordinary answer stands and the stage is judged on it as
before — the fix can refuse nothing.

`HOVER_ROOM_FLOOR = 0.075 m`, not 0.050. The gate is `PAIR_MARGIN`, but what
`solo_check` measures is the whole timeline including the LEG out of the held
pose, after `scene_check` subtracts its 1-Lipschitz playback residual. The pose
that failed was at 45.95 mm *on its go-home leg* with the hover itself clear, so
a pose-only floor at exactly 50 mm would not have moved it.

**Measured on the pinned case** (`tests/test_staged.py::test_staged_end_to_end_on_a_three_stroke_picture`):

| | hold margin off | hold margin on, no room term | **on, with the room term** |
|---|---|---|---|
| arm 71 solo clearance | 62.61 mm | **45.95 mm (FAIL)** | **62.61 mm** |
| arm 71 final held pose | jm 0.136, **fails `validate_pose`** | jm 0.576, passes | **jm 0.6023, passes** |
| pinned test | pass | **FAIL** | **pass** |

Both requirements at once, which is what the conflict said could not be done
from a scan that could not see the room.

**One reporting inconsistency, named and not fixed here.** `staged.ArmStage.hovers`
is re-derived after the timeline with the ORDINARY rule, so a programme's last
serialised `hover_out` is not the pose the trajectory actually ends on (0.136 vs
0.6023 on the toy case). The trajectory is the truth and every check reads it;
only the convenience field disagrees. Fixing it means touching `plan_bucket`'s
`st.hovers` line, which two other agents are editing.

**The stage-A re-run is in flight.** 11760**, against 11100 for the three fixes alone, 11210 for the
hold-margin-everywhere run, and **19190** in the baseline -- so asking the held
pose for both costs about 6 % of the transit budget and still leaves 39 % of the
42 %. Verdict, stage time, arm 71 pen-up and per-arm planning are owed, from
`out/staged_csail_h097_lf5_s150.*`.

### 26.6 Which mode ships

**`serial`.** It is the only mode that flies all 12.085 m, it holds every gate
but two, and it costs **354.6 s** of makespan against the 307.5 s the parallel
composition quoted for a stage that was never a certificate. `priority` is
sound and unusable on this picture; `parallel` stays selectable as the control
and must not be animated or shipped.

**And `serial` is not shippable YET**, by 0.09 mm and one leg:

1. **+49.9 mm on 71 ↔ 97 at t = 32.5 s** against the 50 mm gate. One instant,
   one pair, 90 µm. The row conductors each pass their own check (+62.3, +113.5,
   +55.0 mm); this is a cross-row pair at the seam between two rows' slots, and
   the honest fix is the one `hold_gap` already uses — stand the finishing row
   at its park before the next row's first stroke rather than at the instant its
   last leg ends.
2. **+14.4 mm self on arm 71's transit `seg 3`**, whose true clearance is
   23.60 mm. `self_pace_beat`'s lower bound is a fixed 33-sample grid over an
   11 s beat; it has to be `paper.leg_self_lb`'s **converged** bound.

Both are named with numbers and neither is a composition failure. The stage that
§24.5 refused at −191.9 mm now reads **+49.9 mm with every metre of its ink in
the air**, and that is the gap this box was opened to close.

## 28. The integrated run — the four residuals, and the programme they leave

§27 closed the pen-up faults and §26 closed the composition defect, and each was
measured on its own. This section puts every fix of 2026-09-14 into one
programme on the CSAIL logo at h = 0.970 and says what the whole thing costs.
First the four residuals §26.6 and §27.1 named, because **two of the four were
named wrongly, and the measurements are here.**

### 28.1 The 0.09 mm was never a seam — it is the PAIR gate's playback residual

§26.6 read the serial stage C's **+49.9 mm on 71 ↔ 97 at t = 32.5 s** as a
crossing at the seam between two rows' time slots and proposed standing the
finishing row at its park before the next row's first stroke. Measured on that
run's own programme (`out/staged_csail_h097_lf3c_serial_program.json`):

| | at t = 32.475 s |
|---|---|
| arm 97's per-frame travel | **0.00 mm — it is standing still** |
| arm 97's pose | its stage-C entry pose, which on this programme IS its park |
| arm 97's first motion | frame 3608, **t = 180.4 s** |
| arm 71 | drawing piece 0 (`seg` 0, t = 3.35 → 35.60 s) |
| raw pair clearance | **50.76 mm** |
| `scene_check`'s residual | 0.55 × (1.54 + 0.00) mm = 0.85 mm |
| what the judge reads | **+49.91 mm** |

**One arm is drawing and the other has not moved and will not move for another
two and a half minutes.** No composition of the rows in time can change a
number measured against an arm that never moves; the group's slot boundaries
are irrelevant to it. What is actually wrong is the thing §26.3 already
diagnosed for the self gate, one obstacle over: **the producer holds the room to
`PAIR_MARGIN` on a converged bound and the judge subtracts a residual that grows
with the SPEED.** A pose gate set at exactly the judge's number leaves nothing
for the residual, so the geometry is legal and the playback of it is not.

**THE FIX IS PACE, AGAIN, AND IT NOW COVERS THE INK.** `writing.room_pace_beat`
and `writing.room_pace_draw` price, for a pen-up beat and for a drawn stroke,
the seconds its own speed implies against the FROZEN partners:
`lb − k·rate·dt_play/dt ≥ PAIR_MARGIN` solves for `dt` directly. Only the
frozen partners, because they are the ones that do not move — the judge's
`0.55 × (step_i + step_j)` is then entirely this arm's own step and this arm can
pay all of it; two arms moving against each other are the conductor's business.

Three things make it cheap enough to run on every stroke:

* **the price is per INTERVAL, not per move.** A move's fastest point and its
  tightest point are almost never the same point — on this stroke the global
  maximum speed is 5× the speed at the binding clearance — so pairing the worst
  of each asks for five times the stretch the geometry wants (5 476 s against
  67 s, measured).
* **only the intervals that bind are refined.** An interval whose coarse
  1-Lipschitz bound already prices under the move's own duration cannot change
  the answer and is never looked at again.
* **all or nothing.** A partial stretch that still does not reach the floor is
  seconds spent for a verdict that does not change, so a move whose price is
  over `ROOM_PACE_MAX` is left at its own speed and handed to the judge as it is.

Measured on the failing stroke, with arm 97 frozen exactly where the run had it:

| | |
|---|---|
| converged room bound over the stroke | **50.47 mm** |
| stroke duration | 32.25 s → **67.33 s (2.09×)** |
| what the judge reads | **+49.91 → +50.25 mm** |

That is the honest price: 35 s of stage C to make 0.09 mm of gate, and the ink,
the route and every pose are untouched.

### 28.2 The self bound had to be the converged one, and that leg cost 3 s

§26.6's second residual, named and now shipped. `self_pace_beat` bounded its own
leg on `paper.line_samples`' fixed 33-sample grid. On arm 71's stage-C transit
beat (frames 1128 → 1190, 3.10 s):

| | |
|---|---|
| 33-sample grid bound | **−13.9 mm** — under the floor, so "no speed rescues this" |
| `paper.leg_self_lb`, converged | **+23.10 mm** — over the router's 23 and the judge's 20 |
| the judge at 3.10 s | **+14.3 mm — FAIL** |
| paced to 15.88 s (5.1×) | **≥ 20 mm — PASS** |

The bound is escalated only where the coarse one says the beat is already under
the floor, so a leg the grid cleared never pays for the refinement, and
refinement can only ever RAISE a bound — this can therefore only ever ask for
LESS stretch than the grid did, never more.

### 28.3 Stage D's frame FAIL is the judge's box against `frozen`'s capsules

§26.5 attributed stage D's **+31.0 mm** to `effective_static_floor` clamping the
routing floor where the arm's endpoints cannot hold it. **It is not that
either.** Measured on the same programme, arm 31's stage-D trajectory:

| | |
|---|---|
| the tightest obstacle | **`body:71_column3`** — arm 71's base column band |
| where | the ENTRY leg, `seg` −1, t = 9.75 s, mid-leg |
| both endpoints | **70.42 mm** — so nothing was clamped |
| `scene_check`'s COLUMN gate, same metal as cylinders | **+84.35 mm, PASS** |
| `scene_check`'s FRAME gate, same metal as a 0.32 m AABB | **+31.03 mm, FAIL** |

`freeze_conduct` legitimately drops a frozen partner's band and replaces it by
that partner's real capsules — 127 mm of honest room, and the whole reason
`frozen` exists. `scene_check` re-derives everything from the timeline and knows
nothing about any of it: its frame gate measures every arm against
`spec.static_obstacles()`, band AABBs included. The gate did not change and the
route did, exactly as §26.5 said — but the room that changed is the BAND's, not
the floor's.

**SO THE CONDUCT MAY ASK FOR BOTH ROOMS.** `frozen.set_keep_bands(True)` keeps
the bands in the static room AND keeps the pose-aware partner term, which is
strictly more conservative than either half: it is the room the conduct flew in
before `freeze_conduct` existed, plus the partner bodies `freeze_conduct` added.
`staged.CONDUCT_BANDS = "auto"` routes in the relaxed room, and only where the
judge's frame gate refuses what comes out does it route the whole set again with
the bands kept and take that instead — and only if the retry actually clears the
gate. The retry can only be more conservative than the run it replaces, it can
never admit a leg, and it is paid for exactly where it buys something.

### 28.4 The serialised hover was a convenience field disagreeing with the truth

§27.1 named it and left it. `ArmStage.hovers` re-derived every hover with the
ORDINARY rule after the timeline was built, while under `PARK_FREEZE`
`writing.arm_program` asks a different question of the one pose the stage stops
and holds — the strict joint margin and the room. The two answers differ (0.136
rad of joint margin against 0.6023 on the pinned toy case) and the trajectory is
the truth: it is what `scene_check` reads, what the barrier holds and what the
next stage plans from. The last `hover_out` is now taken FROM the trajectory's
end pose rather than re-derived beside it. Under `PARK_HOME` the trajectory ends
at the park, which is not a hover, so the field stays what it always was.

### 28.5 What the integrated programme is, and how to re-run it

`scripts/lf6_run.sh` — both configurations, `setsid`, three route jobs and three
conduct jobs each, a 1 500 s cap on any one row conduct:

```
JOBS=3 CJOBS=3 CAP=1500 bash scripts/lf6_run.sh
```

| | `lf6_s150` | `lf6_whole` |
|---|---|---|
| ink offered | `--split-m 0.15` | `--whole-bag` |
| leader's standoff | `--partner-standoff 0.09` | `--partner-standoff 0.11` |
| final pass | `--row-compose serial` | `--row-compose serial` |

Both carry every fix of the day: the standoff (a5039f1 / 68ccda6), the four
pen-up fixes (9d55ab8, 0cf78fd, 0064ba7, 5144c8a), `freeze_conduct` and the
serial composition (f7d6ba0, 0eeca85, cee1b7b), and the four of §28.1–§28.4.

### 28.6 The integrated programme, measured — and it is NOT certified

`out/staged_csail_h097_lf6_{s150,whole}.{json,log,_program.json}`, both end to
end, three route jobs and three conduct jobs each, warm leg store. Every
conducted number is `scene_check.check_timeline` over all six arms at
`PAIR_MARGIN` with the sweep residual, `sub` = 2, re-derived off the shipped
programme.

**`lf6_whole` — `--whole-bag --partner-standoff 0.11 --row-compose serial`**

| stage | actives (roles) | pieces | ink m | stage s | inter-arm mm | solo mm | self mm | frame mm | paper mm | verdict |
|---|---|---|---|---|---|---|---|---|---|---|
| A | lead 2,13,71 / foll 17,31,97 | 17 | **4.595** | 84.7 | +183.0 | +175.3 | — | — | — | **PASS** |
| B | lead 17,31,97 / foll 2,13,71 | 22 | **4.769** | 127.2 | +320.0 | +137.5 | — | — | — | **PASS** |
| C | 3 row conductors, serial | 27 | 6.693 | 78.8 | **−108.6** | — | +63.7 | +54.7 | +22.4/−4.4 | **FAIL** |
| D | — | 0 | 0.000 | 0.0 | — | — | — | — | — | (none) |

**`lf6_s150` — `--split-m 0.15 --partner-standoff 0.09 --row-compose serial`**

| stage | actives (roles) | pieces | ink m | stage s | inter-arm mm | solo mm | self mm | frame mm | paper mm | verdict |
|---|---|---|---|---|---|---|---|---|---|---|
| A | lead 2,13,71 / foll 17,31,97 | 15 | 2.061 | **58.5** | +197.6 | +167.9 | — | — | — | **PASS** |
| B | lead 17,31,97 / foll 2,13,71 | 25 | **5.519** | 135.4 | +216.6 | +134.2 | — | — | — | **FAIL** |
| C | 3 row conductors, serial | 34 | 8.407 | 98.5 | **−238.5** | — | +61.8 | +54.8 | +43.1/−3.0 | **FAIL** |
| D | conductor [71] | 1 | 0.077 | 4.4 | +245.8 | — | +100.9 | +140.7 | +40.5/−5.5 | **FAIL** (frozen) |

| | v19 | lf-whole (§22) | lf3 serial (§26.5) | **lf6_whole** | **lf6_s150** |
|---|---|---|---|---|---|
| makespan | **209.9 s** | 273.3 s | 354.6 s | **290.6 s** | **296.7 s** |
| certified | yes | no | no | **no** | **no** |
| ink drawn | 16.8 m | — | 16.07 m | 16.057 m (95.6 %) | 16.064 m (95.6 %) |
| A + B share of the ink | — | — | 22.8 % | **58.3 %** | 47.2 % |
| A + B makespan | — | — | 119.9 s | **211.9 s** | 193.9 s |
| time to first motion | — | — | 0.196 s | **0.174 s** | 0.198 s |
| planning wall (sum / busiest stage-arm) | — | — | 1 980 / 1 102 s | 2 286 / 805 s (arm 31, stage B) | 3 203 / 799 s (arm 31, stage B) |
| check | — | — | 49.7 s | 49.7 s | 40.8 s |
| stage-C cost per metre (motion) | — | — | 17.6 s/m | **17.4 s/m** | 20.4 s/m |
| fleet pen-up | — | — | — | **138.5 s** (5.6 % flip, 48.5 % tall, 45.9 % honest) | **150.4 s** (2.7 % flip, 44.9 % tall, 52.3 % honest) |

**STAGE A AND STAGE B ARE THE WIN, AND THEY ARE REAL.** The whole bag now draws
**9.364 m — 58.3 % of the ink — in the two six-arm stages**, against 22.8 % for
lf3, and stage A's own 95.86 s of §27 is now **58.47 s on identical ink** with
`active_pair` at +197.6 mm against +65.6. The pen-up fixes and the standoff both
hold up in a full programme.

**AND THE FINAL PASS IS NOT CERTIFIED, FOR TWO REASONS THAT ARE NAMED.**

### 28.7 The serial slot was measured in the wrong unit

`_compose_groups` laid the groups end to end using `len(timeline["t"])` — the
ROW COUNT of the timeline it got back. A conducted timeline is `dt`-spaced and
its row count is its length; a timeline the conductor REFUSED comes off
`writing.arm_program`'s waypoint clock, where 354 rows can be 59.7 s, and
`_merge_conducts` only resamples it (`_on_clock`) AFTER the offsets are fixed.
Measured on `lf6_whole` stage C: group [31, 71] was refused, its slot was
written as 17.65 s, the next group started there, and arm 71 was still flying at
59.70 s — the merge read 17 ↔ 71 at **−108.6 mm**. `_clock_frames` and
`_slot_frames` now measure the slot on the merge's own clock, and a refused
group's slot is the SUM of its arms' lengths because that is what
`_merge_conducts` lays down. Re-run (`..._whole_c3.*`): the slots are
0–32.4 / 32.45–63.35 / 63.4–116.6 s, disjoint, and the cross-row pairs are all
clear.

### 28.8 What is still in the way: the row conductor's own refusal

With the slots right, both configurations still fail, and on the same thing:

| | `lf6_whole_c3` | `lf6_s150_c2` |
|---|---|---|
| group [31, 71] | **−21.0 mm, REFUSED** | **−235.3 mm, REFUSED** |
| group [13, 17] | +226.2 mm, PASS | +226.2 mm, PASS |
| group [2, 97] | **−161.3 mm, REFUSED** | **−161.3 mm, REFUSED** |
| merged stage C | **−76.6 mm** (2 ↔ 97, t = 82.95 s) | **−240.4 mm** (31 ↔ 71, t = 39.6 s) |
| stage C motion | 116.7 s | 171.1 s |
| A+B+C+D | 328.5 s | 369.4 s |

**EVERY FAILING PAIR IS A SAME-ROW PAIR WITH ONE ARM STANDING STILL.** 2 ↔ 97 at
t = 82.95 s: arm 97's per-frame travel is 0.00 mm. 31 ↔ 71 at t = 20.0 s: arm
31's is 0.00 mm. That is the signature of `_conduct_stage`'s FALLBACK: when
`idle.conduct` refuses a group, its arms fly ONE AT A TIME — but each one's
route was laid down by `plan_bucket`, which runs BEFORE `freeze_conduct` and is
given `partners=outside` only, so a group's own arms are never in each other's
room. `freeze_conduct` now also holds the arms INSIDE the group that produce no
timeline (`still`, §28.3's sibling), which took group [31, 71] from −43.6 to
−21.0 mm; it cannot help the pair where BOTH arms have ink, because neither is
still at `freeze_conduct` time and the refusal is only discovered afterwards.

**THE FIX IS A PRIORITY ORDER INSIDE THE GROUP**, exactly as `_priority_stage`
already does between arms in a stage: plan the busier arm of the row first and
plan the other against its realised trajectory, so the fallback's one-at-a-time
programme is certified by construction rather than only measured. It is named
here rather than shipped unmeasured, and it is the one thing between this
programme and a certificate.

## 29. The last two defects — a priority order inside the group, and a hold that is verified

§28 left a programme with every fix of 2026-09-14 in it and **no certificate**,
and it named exactly two things in the way. Both are about a pose somebody is
STANDING IN while somebody else moves, and both are closed here.

### 29.1 A refused group's arms were never in each other's room

§28.8 measured it and named the fix; this is the fix. When `idle.conduct`
refuses a row group, `_conduct_stage` falls back to flying its two arms one at a
time — but the timelines it flies came out of `plan_bucket`, which runs BEFORE
`freeze_conduct` and is handed `partners=outside` only. **A row's own two arms
are therefore never in each other's room**, and the moving one routes straight
through the standing one: 2 ↔ 97 at **−161.3 mm**, 31 ↔ 71 at **−21.0 mm**, in
both cases against an arm whose per-frame travel is 0.00 mm.

`freeze_conduct`'s `still` set (§28.3's sibling) catches the arms inside a group
that produce NO timeline. It cannot catch a pair where both arms have ink,
because neither is still at freeze time and the refusal is only discovered
afterwards.

**`_serialise_group` GIVES THE GROUP THE ARGUMENT `_priority_stage` ALREADY
MAKES BETWEEN ARMS IN A STAGE**, one level down:

1. **busiest arm first**, planned against the frozen outside fleet PLUS its own
   row partner *at the pose that partner holds* — which is the pose nobody was
   routing around;
2. **the second arm** against the outside fleet PLUS the first arm's REALISED
   TRAJECTORY as an exact swept room (`trajectory_room`, `freeze_conduct`'s own
   machinery, the same object the main stages use);
3. **every leg re-routed under that room.** Nothing is reused from
   `plan_bucket`, because what `plan_bucket` produced is exactly what was wrong;
4. **the room is the conservative reading of a slot that does not overlap.**
   `_merge_conducts` lays a refused group's arms END TO END, so while the second
   flies, the first has finished and is standing at its park — which its own
   swept room contains. Where the whole room costs the second arm a piece, that
   arm is re-planned against the partners' FINISHING POSES instead: a solo
   against a frozen fleet, always routable or honestly refused, and it is the
   honest statement of the slot it actually flies in. Which reading shipped is
   recorded per arm (`room_kind`, `note`);
5. **`hold_gap` at every slot boundary**, because a boundary is a barrier: six
   arms standing in one scene while one of them hands over.

...AND THE MERGE HAD TO LEARN THE ORDER. `_merge_conducts` laid a refused
group's slots down in ascending ARM ID. The certificate is ordered — the second
arm is certified against the first's trajectory — so the merge now reads
`serial_order` off the group's report and lays the slots down in the order they
were planned in. Without that, the programme would ship the certified-against
arm flying second.

### 29.2 A held hover that keeps neither the joint margin nor the room

§27.1's room-aware ask is BEST-EFFORT by construction, and one layer down so is
`hover_solve`: where the whole fiber has nothing that keeps `HOVER_HOLD_MARGIN`,
it **settles back to `HOVER_MARGIN` and returns that**. That is the right answer
for a travelling hover and the wrong one for the pose a barrier is about to hold
for a minute. Measured on `lf6_s150` stage B: arm 31's last held hover came back
at **0.1064 rad** against `validate.MARGIN_GATE`'s 0.15, `scene_check` judged the
held pose with `validate_pose`, and the whole stage was refused on
`frozen_failed` with every distance gate passing wide.

**SO THE ASK IS VERIFIED, AND A POSE THAT FAILS IT IS NOT HELD.**
`writing.held_pose_ok` asks the two questions the constants name — the strict
joint margin, and `static_gate`'s whole admissibility test including the frozen
partners at `HOVER_ROOM_FLOOR` — of the pose that came back rather than of the
search. Where it fails, the arm ends its stage on a CERTIFIED RETREAT
(`writing.hold_candidates`), cheapest first:

| rung | what it is | why it is where it is |
|---|---|---|
| (a) `hover_z` | a different HEIGHT over the same stroke end (`HOLD_LADDER_EXTRA`) | same tip, same fiber, more air — the arm is already underneath it |
| (b) `hover_prev` | a hover over an EARLIER stroke end of the same bucket, latest first | ink this arm has already drawn: airspace it has already been certified in, and the travel stays inside its own work area |
| (c) `park` | the shipped park | **always valid** — it is the pose the fleet's pairwise argument is built on and the pose the next programme plans from. It costs a trip home, which is what `PARK_FREEZE` exists to avoid, so it is last and it is never refused |

Every non-park rung is asked the same `held_pose_ok` the barrier will apply, and
each candidate must also be ROUTABLE from the final lift — the first that is
both ships. Which one was taken rides on the timeline as `hold_kind` and out
through `programme()` per arm, so a barrier can say what it is holding rather
than only that it holds something.

**...AND THE BARRIER ITSELF NOW ASKS.** `hold_gap` proved the held set pairwise
clear and nothing else; the half that failed is about each pose ALONE.
`validate_pose` — the same gate, with the same `PEN_PAPER` exemption
`scene_check` makes — is now asked of every held pose where the barrier is
declared, and it gates `ok`. A barrier may not hold a pose the judge would
refuse.

### 29.3 The whole bag's final pass, re-run — and it is CERTIFIED

`out/staged_csail_h097_lf6_whole_c4.*`, stage C only off the §28 programme
(`--stage-c-only ..._lf6_whole_program.json --stage-c-index 2 --row-compose
serial --conduct-jobs 3 --conduct-cap-s 1500`), stages A and B untouched.

| group | `lf6_whole_c3` (§28.8) | **`lf6_whole_c4`** |
|---|---|---|
| [31, 71] | **−21.0 mm, REFUSED** | **+118.1 mm, PASS** (conductor refused; in-group priority) |
| [13, 17] | +226.2 mm, PASS | +226.2 mm, PASS (conducted) |
| [2, 97] | **−161.3 mm, REFUSED** | **+215.3 mm, PASS** (conductor refused; in-group priority) |
| merged stage C | **−76.6 mm, FAIL** | **+118.1 mm, PASS** |
| stage C motion | 116.7 s | **98.1 s** |
| stage C cost per metre | 17.4 s/m | **14.7 s/m** |

**BOTH REFUSED GROUPS NOW PASS, AND THE STAGE IS FASTER THAN THE ONE THAT
FAILED.** The two arms of a refused row still fly one after the other — that is
what a refusal means — but each one's legs are now routed in the room the other
one is actually in, and a leg routed around the truth is shorter than a leg
routed around nothing and then measured against the truth.

...AND THE BAND RETRY IS PART OF IT. The first pass put group [31, 71] at
**+36.4 mm** with the judge's FRAME gate refusing arm 71 — §28.3's disagreement
between the room the conduct routes in and the room the judge measures in, now
reached through the serialised re-plan as well. `CONDUCT_BANDS = "auto"` re-ran
the set with the partners' bands kept and took it: **+36.4 → +118.1 mm**.
`frozen.freeze_sets` CLEARS the keep-bands flag, so `freeze_stage` and
`plan_bucket` had to learn to restate it (and to put it in the leg store's
namespace) or the re-plan would silently route in the relaxed room the retry
exists to leave behind.

| stage | actives (roles) | pieces | ink m | stage s | inter-arm mm | solo mm | self mm | frame mm | paper mm | verdict |
|---|---|---|---|---|---|---|---|---|---|---|
| A | lead 2,13,71 / foll 17,31,97 | 17 | 4.595 | 84.7 | +183.0 | +175.3 | — | — | — | **PASS** |
| B | lead 17,31,97 / foll 2,13,71 | 22 | 4.769 | 127.2 | +320.0 | +137.5 | — | — | — | **PASS** |
| C | 3 row conductors, serial | 27 | 6.689 | **98.1** | **+118.1** | — | +39.9 | +56.4 | +43.9/−2.2 | **PASS** |
| D | — | 0 | 0.000 | 0.0 | — | — | — | — | — | (none) |

| | v19 | `lf6_whole` (§28.6) | **`lf6_whole_c4`** |
|---|---|---|---|
| makespan | **209.9 s** | 290.6 s (uncertified) | **309.9 s** |
| certified | yes | no | **YES** |
| ink planned | 16.8 m | 16.057 m | 16.053 m |
| ink **FLOWN** | 16.8 m | 14.141 m | **13.255 m** (see §29.5) |
| A + B share of the flown ink | — | 66.2 % | **70.6 %** |
| time to first motion | — | 0.174 s | 0.174 s |
| planning wall (A+B sum / busiest arm-stage) | — | 2 286 / 805 s | 2 286 / 805 s |
| planning wall (stage C, wall / busiest group) | — | 93 s / 65 s | **1 428 s / 698 s** |
| check | — | 49.7 s | 49.7 s |
| stage-C cost per metre (motion) | — | 17.4 s/m | **14.7 s/m** |
| fleet pen-up | — | 138.5 s | **111.6 s** (6.9 % flip, 40.3 % tall, 52.8 % honest) |

**WHAT THE CERTIFICATE COSTS IS PLANNING WALL, AND IT IS THE ONLY THING THAT GOT
WORSE.** A refused group is now planned TWICE — once by `plan_bucket` against
the outside fleet, once again per arm under the in-group room — and each re-plan
opens a leg-store namespace nothing has warmed, so every pen-up leg is routed
from scratch. Stage C went from 93 s of wall to **1 428 s**. The motion it
produces is 18.6 s shorter and it is the first stage C that ships.

### 29.4 s150 end to end — CERTIFIED, and it costs 4.6 s and no ink

`out/staged_csail_h097_lf6b_s150.*`, all four stages, both fixes, warm leg
store, four route jobs, three conduct jobs, a 1 500 s cap per row conduct.

| stage | actives (roles) | pieces | ink m | stage s | inter-arm mm | solo mm | self mm | frame mm | paper mm | verdict |
|---|---|---|---|---|---|---|---|---|---|---|
| A | lead 2,13,71 / foll 17,31,97 | 15 | 2.061 | 58.5 | +197.6 | +167.9 | — | — | — | **PASS** |
| B | lead 17,31,97 / foll 2,13,71 | 25 | 5.519 | **140.0** | +180.6 | +134.2 | — | — | — | **PASS** |
| C | 3 row conductors, serial | 34 | 8.407 | 146.8 | **+55.1** | — | +60.2 | +54.7 | +41.6/−4.4 | **PASS** |
| D | conductor [71] | 1 | 0.077 | 4.4 | +251.7 | — | +114.6 | +70.4 | +41.4/−4.7 | **PASS** |

| | v19 | `lf6_s150` (§28.6) | **`lf6b_s150`** |
|---|---|---|---|
| makespan | **209.9 s** | 296.7 s | **349.7 s** |
| certified | yes | **no** (B, C and D all failed) | **YES — every stage, every gate** |
| ink FLOWN | 16.8 m | 14.148 m | **14.148 m — unchanged** |
| ink planned but not flown | — | 1.916 m (arm 31, stage C) | 1.916 m (arm 31, stage C) |
| A + B share of the flown ink | — | 53.5 % | **53.5 %** |
| barriers | — | — | 4, all clear, worst **+241.3 mm** |
| time to first motion | — | 0.198 s | 0.342 s |
| planning wall (sum / busiest arm-stage) | — | 3 203 / 799 s | 3 888 / 831 s |
| row conducts (busiest group, wall) | — | — | **1 321 s** |
| check | — | 40.8 s | 41.1 s |
| stage-C cost per metre (motion) | — | 20.4 s/m | **17.5 s/m** |
| fleet pen-up | — | 150.4 s | **150.5 s** (2.7 % flip, 44.6 % tall, 52.7 % honest) |

**THE STAGE-B FIX COSTS 4.6 SECONDS AND NOTHING ELSE.** 135.4 → 140.0 s on the
same 25 pieces and the same 5.519 m: arm 31's held hover kept 0.1064 rad, the
verification refused it, rung (a) found nothing higher over that stroke end, and
**rung (b) — a hover over the arm's PREVIOUS stroke end — took it**
(`hold_kind = "hover_prev"`). One retreat in the whole programme; every other
stage on both configurations ends on an ordinary `hover`. The park rung, the
expensive one, was never needed.

**AND NO METRE OF INK MOVED.** The 1.916 m arm 31 plans and cannot fly in stage
C is the same 1.916 m it could not fly in §28.6 — it is a bucket `arm_program`
refuses, not something either fix took away.

### 29.5 The whole bag's stage C certifies by giving up 0.885 m, and why

The honest accounting, flown ink only (a bucket whose `arm_program` refuses is
PLANNED, not drawn, and the `ink_m` column above counts what was planned):

| | `lf6_whole_c3` | **`lf6_whole_c4`** |
|---|---|---|
| stage C planned | 6.693 m | 6.689 m |
| stage C **flown** | **4.777 m** | **3.891 m** |
| arm 31 | 1.916 m planned, not flown | 1.914 m planned, not flown |
| arm 97 | 0.885 m **flown, at −161.3 mm** | 0.885 m planned, **not flown** |

**ARM 97's 0.885 m WAS NEVER DRAWABLE WHERE IT WAS DRAWN.** It is the metre that
read −161.3 mm against a standing arm 2, and the fix is what discovered that:
planned with arm 2 in the room as the pose it actually holds, arm 97's bucket
does not fly at all. That is the honest answer and it is a loss.

**IT IS ALSO AN ORDER THAT WAS NOT SEARCHED.** `_serialise_group` plans BUSIEST
FIRST, so arm 97 (0.885 m) planned against arm 2 standing at its entry pose. The
other order plans arm 2 first — arm 2 then goes HOME, and arm 97 flies its slot
against arm 2 AT ITS PARK, which is a different and possibly much roomier
question. One bit, two orders, and a group of two has only the two: **trying the
reverse order when the busiest arm's bucket does not fly is the next thing this
owes**, and it is named here rather than shipped unmeasured. `lf6b_s150` does
not need it — its flown ink is identical to the uncertified run's.

### 29.6 The terms the 1 000-stroke model wants, measured on a certified run — and what is still owed

A + B carry the majority of the flown ink on both configurations (53.6 % on
`lf6b_s150`, 70.6 % on `lf6_whole_c4`), so §21's recalibration is the right ask.
Its terms, measured over the twelve flown buckets of `lf6b_s150` — the first
staged programme that certifies end to end:

| term | §12 (zigzag) | §21 (v6) | **`lf6b_s150`, certified** |
|---|---|---|---|
| draw | 6.56 s/m | 13.81 s/m | **16.12 s/m** |
| inter-piece pen-up leg | 5.15 s | 4.18 s | **2.68 s** |
| park overhead per (stage, arm) | 9.74 s | 10.93 s | **38.55 s** |
| accept rate | 81.9 % | 81.9 % | **96.2 %** |
| stage-C cost per metre (motion) | — | 17.6 s/m | **17.5 s/m** (whole bag: **14.7**) |

**TWO OF THOSE TERMS NO LONGER MEAN WHAT §12 MEANT BY THEM, AND THE MODEL MAY
NOT BE ASSEMBLED FROM THEM UNTIL THEY DO.**

* **draw 16.12 s/m** is not a faster or slower pen. It is the pen PACED: §28.1's
  `room_pace_draw` buys the judge's playback residual out of the ink's own
  clock, so a stroke drawn 50.5 mm from a frozen partner is flown 2× slower and
  nothing about the geometry changed. A 1 000-line set has a different
  proportion of strokes near a partner, so the term has to be split into
  unpaced draw plus a paced fraction before it scales.
* **park overhead 38.55 s per (stage, arm)** is dominated by stage C, where
  `entry_s` is not a trip out of a park at all — it is an arm WAITING FOR ITS
  SERIAL SLOT. Under `--row-compose serial` and the in-group serialisation that
  wait grows with the other rows' motion, which is the opposite of a fixed cost
  per (stage, arm), and §12's whole "the barrier amortises" argument is about a
  fixed cost. Stage A and B alone read **4.58 s** per (stage, arm), which is the
  comparable number and is less than half §12's 9.74.

So the honest statement is: **the calibration is taken and the assembly is
owed.** It needs the 1 000-line set's per-(stage, arm) piece and ink counts
under `leader_follower` — a different stage structure from the eight-stage
zigzag §12 and §21 were assembled on — and it needs the draw term split. That is
one `traces` pass plus arithmetic, and it is the pick-up line:

```
ARIS_RIG=proposed ARIS_TOOL=lateral .venv/bin/python -m aris_sixarm.staged \
    --synthetic lines:1000 --pattern leader_follower --no-fly \
    --json out/staged_1000_lf.json
```

### 28.9 The suite, after all of it

`.venv/bin/python -m pytest tests -q` (no environment variables, 88 min):
**788 passed, 24 skipped, 12 failed — and none of the twelve is this work's.**

| failure | where it comes from |
|---|---|
| `test_planner_robustness` ×2 (batched FK bit-identity) | **pre-existing**, reproduced in a clean worktree at `cee1b7b` |
| `test_csail::test_a_retreat_is_offered_only_on_genuine_interference` | **pre-existing**, reproduced in a clean worktree at `cee1b7b` |
| `test_export_pathway` ×6, `test_sil_run` ×2 | another agent's untracked files, outside this box |
| `test_paper::test_two_processes_share_the_store` | **passes in isolation** — a disk-store race under a loaded machine |

The files this box owns — `test_staged`, `test_staged_compose`, `test_staged_split`,
`test_staged_standoff`, `test_paper`, `test_transit`, `test_penup`, `test_gates` —
are **177 passed, 19 skipped** when run together.

## 30. Coverage — what is on the paper, and why the rest is not

Pete, 2026-09-15, watching the certified `lf6b_s150` animation: **"why are there
so many gaps in the lines?"** The certified canvas at h = 0.970 is hole-free and
v19 drew this exact placement at **100.0000 %**, so every metre the staged
programme does not draw is a PLANNER failure and not a reach one. This section
is the account, and it is measured rather than argued.

### 30.1 The programme's own book-keeping cannot see the hole

`StageResult.ink_m` sums the pieces a bucket ACCEPTED, and a bucket whose
`arm_program` refuses still has them: `plan_bucket` sets `st.programme` before
it ever calls `arm_program`, and `_merge_conducts` then pads the arm out to the
stage's frame count so it stands in the conducted clock holding its entry pose.
Measured on `out/staged_csail_h097_lf6b_s150_program.json`, arm 31, stage C:

| | |
|---|---|
| pieces listed | **8** |
| `ink_m` reported | **1.9161 m** |
| frames in its trajectory | 2 937 |
| frames whose `seg` is a segment index | **0** |
| what is on the paper | **nothing** |

**SO THE ACCOUNT IS GEOMETRIC.** Drawn means a frame whose `seg ≥ 0` — a
PEN-DOWN SAMPLE — and the residual is the logo minus every drawn piece's own
polyline, matched PER LINE at 2.5 mm on a 2 mm grid. `scripts/gap_account.py`
is that measurement plus the reason chain; `tests/test_gap_account.py` pins both
claims on a toy picture whose hole is known.

### 30.2 The account, on `lf6b_s150`

```
ARIS_RIG=proposed ARIS_TOOL=lateral .venv/bin/python -m scripts.gap_account \
    --programme out/staged_csail_h097_lf6b_s150_program.json \
    --lines out/csail_schedule_h097_v19_strokes.json \
    --v19 out/csail_schedule_h097_v19_program.json \
    --png out/lf6b_gaps.png --json out/lf6b_gaps.json
```

| | |
|---|---|
| logo | **16.8047 m** |
| flown (pen-down samples, union per line) | **14.1747 m = 84.35 %** |
| missing | **2.6300 m in 13 stretches** |

| metres | stretches | reason |
|---|---|---|
| **1.8908** | 8 | **PLANNED AND NEVER FLOWN** — arm 31's stage-C bucket produced no timeline in either reading of the serialised group's room; the go-home `aside` gave it a timeline with no ink in it |
| **0.7250** | 3 | **NO DRAWER LEFT** — `plan_stroke` refused with `split: empty_fiber`, the refusal loop banned that span, and the pattern offered the span to exactly ONE (stage, arm) cell |
| **0.0041** | 2 | `degenerate: too_short` stubs (a 7.5 mm DP piece and a 5.0 mm `split_at_room` remainder) |

The picture is `out/lf6b_gaps.png`: logo thin grey, flown ink black, the
thirteen missing stretches numbered and coloured by reason, with the whole
reason chain beside it.

### 30.3 The 0.725 m is a TILT the staged pipeline does not take

The three `no_drawer` stretches are the decisive ones, because each was offered
to a single cell and lost it:

| stretch | offered to | `plan_stroke` | v19 drew it with | reach from that base | v19 tilt cone |
|---|---|---|---|---|---|
| line 32, 0.032–0.336 (303.6 mm) | **(stage 1, arm 31) only** | `split: empty_fiber`, s\* = 0.088 | arm 71 | 0.576 m | **10.0°** |
| line 7, 0.180–0.415 (235.3 mm) | **(stage 0, arm 71) only** | `split: empty_fiber`, s\* = 0.226 | arm 71 | **0.071 m** | **15.0°** |
| line 1, 0.300–0.486 (186.1 mm) | **(stage 0, arm 71) only** | `split: empty_fiber`, s\* = 0.208 | arm 71 | **0.011 m** | **12.5°** |

Two of the three are ink almost exactly UNDER arm 71's own base, and v19 drew
them by leaning the pen. The staged runs take `--tilt-max-deg 0`, which is the
CLI default and what `scripts/lf6_run.sh` passes, so the fiber over those points
is empty and `plan_stroke` is right to refuse. Re-deriving the DP and the
refusal loop with the tilt cone opened (`rederive_dp(..., tilt_max_deg=15.0)`,
15 s, same atlas, same pattern):

| | tilt 0° (shipped) | **tilt 15°** |
|---|---|---|
| `split: empty_fiber` refusals | **3** | **0** |
| bans applied | 3 | **0** |
| DP coverage after the loop | 95.63 % | **100.00 %** |
| uncovered | **0.7337 m** | **0.000 m** |

**ONE FLAG RECOVERS THE WHOLE 0.725 m**, and it costs no gate: the atlas is
already read with `tilt_max_deg` and `scene_check` judges the trajectory either
way. It is not shipped in this section because it changes what every bucket is
planned from and therefore needs its own certified end-to-end run.

...AND THE SECOND HALF OF THAT REASON IS THE PATTERN. A stretch offered to ONE
(stage, arm) cell has no alternative when that cell refuses: the leader/follower
region split at `--split-m 0.15` is what narrows the offer, and v19's conductor
had all six arms with no stage or region partition at all. Where a stretch had
two or three cells the loop's ban cost nothing.

### 30.4 The 1.891 m is `_serialise_group`'s unsearched order

§29.5 named it and it is now closed in code. Busiest-first makes the busiest arm
plan against its row partner AT THE PARTNER'S ENTRY POSE, and the second arm
against that partner AT ITS PARK — two different questions, and a group of two
has only the two. So a bucket that does not fly now goes to the BACK of the
order (the slot in which every partner has finished and gone home) and whichever
order flies more ink ships; `orders_tried`, `lost_m` and `lost_pieces` ride on
the group's report. The search is paid for only where the first order loses a
WHOLE bucket, because a refused group is already planned twice and stage C's
wall is 1 428 s (§29.3).

Whether it recovers arm 31's 1.914 m is a measurement, not a claim, and it is
the stage-C-only run in flight as this is written:

```
ARIS_RIG=proposed ARIS_TOOL=lateral .venv/bin/python -m aris_sixarm.staged \
    --stage-c-only out/staged_csail_h097_lf6b_s150_program.json \
    --stage-c-index 2 --row-compose serial --route-jobs 3 --conduct-jobs 3 \
    --conduct-cap-s 3600 --json out/staged_csail_h097_lf7c_s150.json \
    --programme out/staged_csail_h097_lf7c_s150_program.json
```

The cap is 3 600 s rather than the 1 500 s of §29.4 because group [31, 71] alone
measured 1 321 s of wall at ONE order, and two orders cannot fit under it.

### 30.5 It is not the logo — the same thing happens on the bench corpus

`scripts/bench_lines.py --sheet installed` regenerates docs/BENCH.md's five
pinned drawings into the paper the fleet actually has, and the DP plus the
refusal loop were re-derived on `duotone` (12.83 m, ten interleaved bands) at
the two tilt cones, same atlas, same pattern, same `--split-m 0.15`:

| | tilt 0° | **tilt 15°** |
|---|---|---|
| bans applied | **8** | **1** |
| refusal kinds | `empty_fiber` ×4, `start_infeasible` ×2, `sheet_collapse` ×2, `too_short` ×4 | `empty_fiber` ×1, `too_short` ×1 |
| DP coverage after the loop | **85.42 %** | **97.23 %** |
| uncovered | **1.8704 m** | **0.3551 m** |

**11.8 POINTS OF COVERAGE ON A PICTURE THAT IS NOT THE LOGO, FROM THE SAME
FLAG.** The CSAIL run loses 4.4 points to this and `duotone` loses 14.6; the
mechanism is identical and it is the vertical pen, not the reach. That is the
strongest argument this section has for making the tilt cone the shipped
default, and it is why the recommendation is not "patch the three CSAIL
stretches".

### 30.6 THE STAGED MODEL HAS NO INK DIMENSION, AND A TWO-COLOUR PICTURE IS WRONG RATHER THAN SLOW

Pete's "any drawing out of the box" includes two-colour pictures, and this is
the one place the staged pipeline does not merely lose metres — it produces a
programme that is **incorrect**.

| picture | strokes | m | **inks** | what the pattern did with them |
|---|---|---|---|---|
| hatch | 43 | 30.100 | 1 | — |
| scatter | 70 | 8.636 | 1 | — |
| starburst | 24 | 19.001 | 1 | — |
| spiral | 1 | 13.933 | 1 | — |
| **duotone** | 10 | 12.831 | **2 (grey, orange)** | **NOTHING — the colour never entered the planner** |

`traces.load_lines` harvests `pts` and drops every other field, so `duotone`'s
grey and orange bands reach the DP as ten indistinguishable polylines. The
pattern's stages are COLLISION stages — who may be in the air with whom — and
nothing anywhere in `traces`, `staged` or `writing` groups ink by colour or
inserts a pen swap. A staged programme for `duotone` therefore has arms drawing
orange bands with whatever pen they are holding, and every gate passes, because
no gate is about ink.

The v19 path does have it: `csail_schedule` builds one PHASE per ink with a
human pen swap between them (`payload`'s `--pause`, `program_json`'s palette),
and that is the structure `scripts/draw.py` writes out. So this is the same
finding as §30.4's `--planner staged` blocker, stated as the planner question
it is rather than as a plumbing one:

**AN INK IS A CONSTRAINT ON WHICH ARM MAY DRAW A STROKE AT ALL, AND THE DP'S
CAPABILITY MAP IS EXACTLY WHERE THAT BELONGS.** `traces.Capability` already maps
(x, y) → the set of (stage, arm) cells that can draw there; an ink-aware version
maps (x, y, ink) → the cells whose arm is holding that ink in that phase. The
pattern gains a phase axis, `plan_lines` gains the ink per line, and the barrier
list gains a swap. Until then the honest statement is: **the staged pipeline
plans single-ink pictures, and `duotone` is a measurement of geometry only.**

## 31. The go-home is every conducted stage's, and `unattributed` had a name

§30 left the account with a bucket that was not a reason — 2.063 m on `lf8`,
1.594 m on `bench/starburst`, announced by `run` as a WARNING and explained by
nothing — and it left `CONDUCT_HOME_FIRST` on stage C alone. The two turn out
to be the same defect seen from its two ends: **a conducted group whose slot
produces no arms at all**, which loses the ink (§31.1) and which no reason code
could name (§31.2).

### 31.1 Stage D is conducted too, so it goes home first

`CONDUCT_HOME_FIRST` (§30's fix, `b6c15bb`) parks the fleet before the row pass
so that every row group plans against parks rather than against four arms
standing at their stage-B hovers over the middle of the sheet. Stage D — the
dead-band ink — is **the same call**: `_conduct_groups` over the same held-pose
entry set, one phase at a time. It did not get the go-home, and it shows:

| `bench/starburst`, before | |
|---|---|
| dead band, partitioned out of stage C | **2.831 m (45.3 % of the final pass)** |
| stage D, arm 31 | **7 pieces listed, 0 pen-down samples** |
| coverage | 45.70 % |

The fix is `staged._home_before`, which is the whole of the stage-C block
lifted out of `run`'s phase loop and given a name. It appends the no-ink
go-home stage, advances the held-pose set in place and returns the entry poses
the caller must plan from; `run`'s stage-C branch, `run`'s stage-D phase loop
and `run_conducted`'s band stage are now three calls to it, so **a conducted
stage cannot be added without the go-home** rather than merely happening to
have one. `tests/test_staged.py::test_the_dead_band_stage_ALSO_starts_from_an_
all_parked_fleet` forces a dead-band deferral on the toy picture and spies on
both the ask and what stage D plans from.

The go-home is a no-op when the fleet is already parked — stage C's `PARK_HOME`
brings home every arm whose bucket flew — so the stage D that pays for it is
exactly the stage D whose partners did NOT fly, which is the case that lost the
ink.

### 31.2 `unattributed` is `bucket_never_planned`, and it is the [31, 71] slot

The reason chain could see what a stage LISTED (`listed_not_flown`), what
`plan_stroke` refused (`plan_refused`), what an arm handed on
(`deferred_never_taken`) and what the DP never covered (`no_drawer`). It could
not see a piece that **was never offered to `plan_bucket` at all**, because
every one of those four lists is written BY `plan_bucket` or by the stage that
called it. `plan_bucket` records every piece of a bucket it is handed — that is
the §30.1 finding, `st.programme` before `arm_program` — so a piece in none of
the three lists means its whole bucket was skipped.

`res.pieces` is where such a piece still is, and it is now the last question
the account asks. Measured, and it is the same cell on both pictures:

| run | `unattributed` was | the cells it was never planned for |
|---|---|---|
| `lf8_s150` (CSAIL, tilt 15) | **2.0633 m, 7 stretches** | (stage 2, arm 31) 0.894 m + (stage 2, arm 71) 1.179 m |
| `bench/starburst` | **1.5942 m, 9 stretches** | (stage 2, arm 31) 0.110 m + (stage 2, arm 71) 1.542 m |

**BOTH ARE THE ROW GROUP [31, 71]**, whose slot produced no arms, so its two DP
buckets were never planned and 2.063 m / 1.594 m of the picture existed in the
DP and in no stage's book at all. That is the same group whose bucket §30.4
chased through an order search and `CONDUCT_HOME_FIRST` finally flew on `lf9c`:
the run that loses it wholesale loses it in two different ways at once, and
only one of them had a reason code.

`GAP_REASONS` gains `bucket_never_planned`, asked LAST — a `split_at_room` part
is minted with a new `k`, so a split parent looks unplanned while its parts are
themselves listed or deferred, and the earlier questions have already
attributed the span. It is matched on `(line, k)` rather than on the cell,
because a deferral changes the cell. `scripts/gap_account.py` gains the same
reason (and `deferred_never_taken`, which it did not have), plus
`--tilt-max-deg`: a tilt-15 programme read back with the DP re-derived at 0
invents refusals the run never had.

Re-run on `lf6b_s150`, the programme §30.2's table is quoted from, the account
is **unchanged** — 1.8908 `listed_not_flown`, 0.7250 `no_drawer`, 0.0041
`plan_refused` — which is what "asked last" is supposed to mean.

### 31.3 `lf10` — the CSAIL logo at tilt 15 with every fix: **99.99 % and CERTIFIED**

The measurement §30.3 owed. Same command as §29.4's `lf6b_s150` plus
`--tilt-max-deg 15`, and with `CONDUCT_HOME_FIRST` on every conducted stage:

```
ARIS_RIG=proposed ARIS_TOOL=lateral .venv/bin/python -m aris_sixarm.staged \
    --lines out/csail_schedule_h097_v19_strokes.json --pattern leader_follower \
    --split-m 0.15 --partner-standoff 0.09 --row-compose serial \
    --tilt-max-deg 15 --route-jobs 3 --conduct-jobs 3 --conduct-cap-s 2400 \
    --json out/staged_csail_h097_lf10_s150.json \
    --programme out/staged_csail_h097_lf10_s150_program.json
```

| | `lf8` (tilt 15, before) | `lf9c` (tilt 0, home-first) | **`lf10`** |
|---|---|---|---|
| flown | 51.50 % | 95.6 % | **99.9881 %** |
| drawn of 16.8047 m | 8.655 m | 16.06 m | **16.8027 m** |
| missing | 8.150 m | 0.734 m | **0.0020 m**, one `plan_refused` stub |
| `unattributed` | 2.063 m | — | **0.000 m** |
| `all_ok` | **false** | true | **true** |
| makespan A+B+C+D | — | 389.4 s | **402.7 s** |
| TTFM | — | — | **0.187 s** |
| planning wall (parallel) | — | — | 1 406 s (1 081 s) |

| stage | pieces | ink | duration | |
|---|---|---|---|---|
| A | 17 | 2.8441 m | 63.3 s | active-pair **+79.4 mm** PASS |
| B | 10 | 2.5534 m | 41.4 s | active-pair **+305.2 mm** PASS |
| C go-home | 0 | — | 7.9 s | +217.7 mm PASS |
| C | 44 | 10.8916 m | 266.8 s | whole-timeline **+71.8 mm** PASS |
| D (dead band) | 2 | 0.5123 m | 23.3 s | whole-timeline **+134.2 mm** PASS |

**THE WHOLE `no_drawer` CLASS IS GONE**: the DP reaches 100.00 % at tilt 15 with
no bans at all, so §30.3's 0.725 m — the ink under arm 71's own base that v19
drew by leaning the pen — is planned and flown. What is left is 2.0 mm of
`degenerate: too_short` stub, which is a piece the geometry refuses and not a
planner failure. Arm 31 flies **25 pieces** in stage C where `lf6b` flew none;
stage D's 0.512 m of dead-band ink flies on the band retry (`CONDUCT_BANDS`
auto: the relaxed room FAILED the frame gate, the re-conduct with the partners'
bands kept PASSED).

It costs **13.3 s of makespan over `lf9c`** for 0.74 m more ink, and the whole
programme is +192.8 s over the 209.9 s reference — the price of the serialised
[31, 71] slot, which is 1 151 s of the 1 406 s planning wall on its own.

### 31.4 What the two fixes measure on the bench corpus

`scripts/bench_staged.sh` re-run with both fixes, same configuration
(`--split-m 0.15 --partner-standoff 0.09 --row-compose serial --tilt-max-deg
15`), all five pictures:

| picture | flown, before | **flown, after** | missing, by reason | makespan | wall | `all_ok` |
|---|---|---|---|---|---|---|
| hatch | 83.14 % | 83.14 % | `no_drawer` 5.074 | 380.2 s | 138 s | no |
| scatter | 98.96 % | 98.96 % | `plan_refused` 0.087 | **156.7 s** | **420 s** (was 2 261) | **YES** |
| **starburst** | **45.70 %** | **96.65 %** | `no_drawer` 0.570, `plan_refused` 0.063 | 356.2 s | 1 766 s | no |
| spiral | 84.40 % | 84.40 % | `no_drawer` 2.174 | 275.5 s | **629 s** (was 1 518) | no |
| duotone | 97.35 % | 97.35 % | `no_drawer` 0.340 | 238.0 s | **499 s** (was 1 572) | no |

**`starburst` GAINS 51 POINTS**, and both fixes are in it. Its `[31, 71]` slot
now flies — 6.966 m at +59.0 mm, arms 31(20) and 71(8), where before the slot
produced no arms at all — which retires the 1.594 m of `bucket_never_planned`;
and its stage D flies all **2.831 m** of dead-band ink on arm 31 (the relaxed
room FAILED the frame gate, the re-conduct with the partners' bands kept PASSED
at +126.4 mm), which retires the 2.831 m of `listed_not_flown`. **Every picture
in the corpus now has `listed_not_flown` = 0.000 m and no `unattributed`
bucket**, and every remaining metre on every picture is `no_drawer` or a
`plan_refused` stub — the DP's own ceiling at that placement, which no go-home
touches.

The other four are unchanged in ink and 2.4x–5.4x cheaper to plan, for the same
reason: a row group planning against a parked fleet is a much easier routing
problem than one planning against four arms holding hovers over the sheet.

**WHAT IS STILL OWED ON `starburst`**: `all_ok` is false on the HOLD gate, not
on ink. The barrier pose set at the end of stage B measures **+141.4 mm**,
under `MARGIN_GATE` = 150 mm, so the hold before stage C is refused and the
go-home stage inherits it at t0 (`failed: frame, t0_holds`). Everything after
it PASSES. That is a stage-B parking question — where the followers stop — and
it is the next thing to measure on this picture.
