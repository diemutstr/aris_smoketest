# The 148 s of pauses — where it comes from, and what would buy concurrency

Reconstructed from the shipped programme by `scripts/concurrency_diag.py` (the allocation is deterministic; every per-arm pause it produces reproduces `out/csail_schedule_full.json` to the float, which is asserted, not hoped). Figure `out/concurrency_diagnostic.png`, numbers `out/concurrency_diagnostic.json`. Nothing is re-planned anywhere below — every counterfactual is the same frozen paths read at a different threshold.

**148.0 s of pause is not 148.0 s of lost time.** Phase 1 runs 63.5 s against a floor of 41.0 s (its longest arm's nominal); phase 2 runs 88.0 s against a floor of 88.0 s. **The entire concurrency loss of the piece is 22.5 s, all of it in phase 1, all of it arm 31 waiting on arms 71 (19.1 s) and 97 (3.4 s).** The other 125.5 s is arms standing still inside slack they already had — real seconds of a stopped robot, and worth removing, but they cost the clock nothing. Phase 2 is already optimal: arm 97 draws its 4.52 m in 88.0 s and never waits once.

**Three quarters of the pause is arms held out of their own finished pose.** 116.6 s of the 148.0 (66.4 + 50.2) is the rest-suffix rule — an arm may not *arrive* until the pose it will then stand in is clear all the way to the horizon, so a short programme finishes its ink early and then waits for permission to stop. Adding that rule is what took phase 1 from 4 s of pauses to 90.6 s, and it is a cost of standing still in the wrong place, not of drawing. Only 31.4 s is an arm actually held up on its path.

**Two arms never fight over ink.** Of the 31.4 s spent held on a path: draw-vs-transit 19.4 s, transit-vs-transit 12.0 s, **draw-vs-draw 0.0 s**. The conflict is the hover moves between strokes, not the strokes. And the two floor arms parked at `q_seed` for both passes (13, 17) block **0.00 s** — deleting them from the world entirely changes no number in this document. Every second of contention is among the four inverted arms.

**The whole of phase 1 is six events, and two of them are refused by a fifth of a millimetre.** Every pause in the phase; "min gap" is the tightest swept clearance during the event:

| arm | from | for | blocked by | doing | blocker doing | min gap |
|---|---|---|---|---|---|---|
| 2 | 8.6 s | **44.0 s** | 31 | parking | transit | **79.8 mm** |
| 97 | 10.2 s | **22.5 s** | 71 | parking | transit | **79.3 mm** |
| 31 | 21.4 s | 10.1 s | 71 | transit | seg 3 | 59.0 mm |
| 31 | 9.0 s | 9.0 s | 71 | transit | seg 1 | 60.3 mm |
| 31 | 18.0 s | 3.4 s | 97 | transit | transit | 76.2 mm |
| 97 | 2.6 s | 1.7 s | 71 | transit | transit | 65.9 mm |

The top two are 66.4 s of a robot standing still because its *home* pose is 79.8 mm and 79.3 mm from another arm's hover corridor and the margin wants 80. Phase 2 repeats the pattern once: arm 2 waits 50.2 s to park, at 69.7 mm, against arm 97 transiting. Meanwhile the three events that actually cost clock (arm 31, 22.5 s) bottom out at 59.0–76.2 mm. Relaxing the margin to 65 mm takes phase 1 from 63.5 s to 47.7 s; below that the curve flattens — 47.3 s at 50 mm, 47.1 s at 35 mm — so what is left is no longer a margin problem.

**Nothing interpenetrates. Not once.** Across all 148.0 s of blocking cells the tightest *pose* clearance is 74.4 mm (phase 1) and 77.1 mm (phase 2), and it is never negative. 147.5 s of the 148.0 is blocked at cells whose four corner poses are all ≥ 80 mm apart, and it is only the swept-motion slack (0.55 × the two per-step motions — about 10 mm when one arm is transiting at 0.8 m/s on a 1/48 s clock) that drags the test under the margin. Weighted by the second, the histogram sits between 65 and 80 mm: 0.41 s of the 148.0 is below 65 mm and **0.00 s of it is below 50 mm**. The margin is the obstacle here, and the geometry is not.

| margin | ph1 pause | ph2 pause | total pause | ph1 makespan | total makespan | achieved clearance |
|---|---|---|---|---|---|---|
| **80 mm** (shipped: 50 safety + 30 calib) | 90.6 s | 57.4 s | **148.0 s** | 63.5 s | **151.5 s** | 80.0 mm |
| 65 mm | 70.4 s | 57.2 s | 127.6 s | 47.7 s | 135.7 s | 65.0 mm |
| 50 mm (calib surveyed away) | 12.6 s | 54.1 s | 66.6 s | 47.3 s | 135.4 s | 50.1 mm |
| 35 mm | 11.9 s | 52.6 s | 64.4 s | 47.1 s | 135.1 s | 35.2 mm |

**Priority order is worth as much as the margin, and it is free.** Phase 1 has only **3 of 24** orders feasible at all, and "busiest first, promote whoever deadlocks" landed on the *worst* of the three: `[31, 71, 97, 2]` finishes in **48.3 s** against the shipped 63.5 s, a 15.2 s win for a re-ordering that costs four DPs to evaluate. Phase 2 has 12/24 feasible and the shipped order is already optimal. The conductor stops at the first order that works; it should keep going and rank them.

**A 25 % crawl is worth 10.8 s, and cannot be worth much more.** Integrating the optimistic bound (a stop of L steps becomes a delay of 0.75 L) over every event gives 148.0 → 124.4 s of pause and 151.5 → 140.7 s of makespan. A rest hold is credited nothing on purpose: the arm is not being held up along its path, it is being held out of the pose it wants to finish in, and creeping the last centimetre more slowly still puts it there. Separately, the *geometric* half of the same idea — a creeping link sweeps a thinner tube, so re-conducting with each moving arm's own sweep slack cut to a quarter — recovers phase 1 to 48.3 s on its own, which is the same 15.2 s the re-ordering finds, and for the same reason.

**Tuck poses buy the most pause and the least clock.** Letting every arm retreat out of the way the moment it finishes (and dropping the rest-suffix requirement with it) takes the pause from 148.0 to **51.1 s (−65 %)** but the makespan only from 151.5 to 136.4 s. Removing the two parked floor arms buys exactly nothing (148.0 → 148.0).

**The levers are substitutes, not complements.** Best order alone → 136.4 s; a 50 mm margin alone → 135.4 s; both together → 135.4 s. Every one of them is attacking the same conflict (arm 31 against arm 71 in phase 1), and once it is cleared the phase floors at ≈ 47 s against its 41.0 s ideal. That residual ≈ 6 s survives a 35 mm margin, a tuck, and the best order simultaneously, and is the only part of the piece where the paths genuinely need to be somewhere else.

## Ranked, for what to build next

1. ✅ **Congestion-aware sequencing — build this.** −15.2 s of makespan (phase 1: 63.5 → 48.3 s) for a search the conductor is two lines from doing already: enumerate the priority orders instead of accepting the first feasible one, and rank on makespan rather than on "busiest first". It spends no safety, needs no hardware, re-uses the collision images it has already paid for, and 24 orders cost about a second. **Built** — `coordination._search_priority`, and it lands on exactly the 48.3 s this predicted. See "Makespan pass results" below.
2. **Margin reduction — do the survey, but not for the speed.** −16.1 s, marginally the largest single number, but it is a *substitute* for (1): stacked on top of the best order it adds 1.0 s. The 30 mm `calib` term is an admitted guess about where four preset bases are; surveying them is right regardless, and it would also make the 3-of-24 feasibility in phase 1 much less brittle.
3. **Velocity scheduling.** −10.8 s as a re-timing bound, and it is optimistic. Its real appeal is the geometric half — 99.7 % of the blocking is swept-tube slack over poses that are already 80 mm clear — but cashing that in means re-opening the per-segment velocity certificate `stroke_api` issues, which is the most invasive change on this list for a win (1) gets for free.
4. **Tuck poses.** Only −15.1 s of makespan, but −65 % of the pause and the right fix for a different problem: an arm's *parked* pose can make a schedule infeasible outright rather than slow, which is exactly how the 1.2656 m logo width failed (arm 71's 300 mm pen sweeping through arm 97 at its ready pose). Build it as robustness, and to stop the fleet standing frozen for two minutes of a two-and-a-half-minute run.
5. **Redundancy-space dodging — not yet.** Its entire headroom is the ≈ 6 s of phase 1 that survives every other lever. It is the most machinery on the list for the smallest measured win, and it should wait until (1) and (2) have been taken and re-measured.

## Makespan pass results (2026-08-19)

Lever (1) and one this study did not have — the *allocation* — are built, and the piece runs in **108.6 s against 153.5, 29.3 % off the clock for no coverage, no margin and no certificate**: the same 99.2139 % of 11.567 traced metres over the same 49 segments, both phases signed off by `scene_check` at **82.0 mm** against the 80 mm margin. Pause falls 148.0 → 50.5 s.

**Priority is searched now, not guessed** (`coordination._search_priority`). Every permutation of up to six moving arms is ranked on makespan, sharing one set of collision images and one DP per permutation *prefix*, with the incumbent makespan pushed into the DP as a deadline: 24 orders cost 39 and 55 DP solves. On the allocation this study measured, it lands on `[31, 71, 97, 2]` and 48.3 s — the counterfactual above, to the frame.

**And the allocation balances on seconds, not metres** (`allocate.balance_loads`). The interval cover is optimal for pen-ups and blind to the clock, which is how arm 97 got 4.52 of the 7.45 orange metres and phase 2 got an 88.0 s *floor* that no conductor could have touched. A greedy min-max pass re-assigns spans a second arm has already certified at identical endpoints — three moves in the whole piece — and the floors fall to 35.7 s and 68.3 s. Coverage is invariant by construction rather than by measurement: a candidate re-plan that gives back so much as a millimetre is refused, so the set of drawn spans is identical before and after.

| phase | floor | makespan | pause | → floor | → makespan | → pause |
|---|---|---|---|---|---|---|
| 1 grey | 41.0 s | 63.5 s | 90.6 s | 35.7 s | **38.4 s** | 21.8 s |
| 2 orange | 88.0 s | 88.0 s | 57.4 s | 68.3 s | **68.3 s** | 28.7 s |
| both + 2 s swap | 131.0 s | **153.5 s** | 148.0 s | 106.0 s | **108.6 s** | 50.5 s |

**The levers are complements after all, and this study said otherwise because it was reading one allocation.** Ordering alone is 138.3 s (phase 2's 88.0 s was already the best of its 24 orders, and 10 of them tie); balancing moves the conflict as well as the work, and phase 2 now finishes *at* its floor with no concurrency loss at all while phase 1 is within 2.7 s of its own. The ≈ 6 s of phase 1 called irreducible above is 2.7 s — it shrank because the arm that was waiting is no longer the arm doing all the drawing.

**Single pass was measured, not assumed.** The other way to spend the pen swap is not to have one: all 62 colour partitions of the six arms, at the shipped placement, arms permanently grey or orange. The best of them (arms 2 and 97 orange, the rest grey) certifies **91.088 %** — 1.03 m of the logo left empty against a 99.0 % gate — balances to a 73.9 s floor and conducts, `scene_check` PASS at 82.9 mm, to **74.7 s end to end**. It is 33.9 s faster than the two-pass run and it is refused, because 8.9 % of the picture is not a term in the objective. Two passes ship.

## The L1→L2 interface: entry/exit fiber menus (2026-08-20)

**The complaint this answers.** An arm finishes a stroke at q7 = −1.2 and
starts the next one at q7 = +0.9, and no re-ordering can mend it, because both
numbers were fixed by a band DP that never heard of the neighbouring strokes.
`sequence.py` was choosing an order over ends it was not allowed to move.

**What was built** (`aris_sixarm/menu.py`, `sequence.cluster_*`,
`allocate.sequence_arm_cluster`). Every stroke end sits on a *fiber* — the q7
interval the gates leave open there — so L1 now offers a MENU: ~3 certified q7
candidates per end per spanning sheet, with every (entry, exit) pair the band
cannot connect pruned by the band DP's own forward sweep. L2's state grows from
(segment, direction) to **(segment, direction, variant)**; Held-Karp is still
exact below 16 segments (the node count goes from 2n to 2n·V, and the
relaxation is vectorised so the inner loop stays six array operations), and the
NN + 2-opt + Or-opt fallback gains a **variant re-selection** move — draw the
same stroke in the same place at the same time and simply come off it somewhere
else, a move in neither of the other two neighbourhoods. Nothing is planned
until it is chosen: a menu entry is a promise of two configurations and a cost,
and only the chosen variant is materialised, through the same `stroke_api` path,
with the same independent validator. That laziness is exact — `tests/test_menu.py`
pins that a lazily materialised variant is the eagerly planned one to the float,
and that a one-variant menu reproduces `sequence.cost_matrix` cell for cell and
`held_karp`'s tour node for node.

**It does what it was built to do.** On the shipped CSAIL two-pass run, at
**identical coverage (99.2139 %)**, identical segment count and identical ink.
Three columns, because the numbers this section first published have since
stopped reproducing — twice over, and for two different reasons — and saying
so is the point:

| | as published, 2026-08-20 | the committed code, `qd_frac` 0.30 | today: `--qd-frac 0.6` + cluster-aware pricing |
|---|---|---|---|
| transit (both phases, all arms) | 96.02 → **81.84 s** (−14.8 %) | 96.02 → 101.82 s (+6.0 %) | 64.65 → **57.01 s** (−11.8 %) |
| reconfiguration Σ‖q_exit − q_entry_next‖∞ | 46.93 → **16.41 rad** (−65.0 %) | 46.93 → 43.77 rad (−6.7 %) | 46.32 → **39.90 rad** (−13.9 %) |
| min clearance | 82.1 → 86.0 mm | 82.1 → 83.0 mm | 82.7 → **82.4 mm** |
| makespan | 104.0 → **153.5 s** (+47.6 %) | 104.0 → 123.4 s (+18.6 %) | 84.4 → **77.8 s** (−7.8 %) |
| coverage | 99.2139 % | 99.2139 % | 99.2139 % |

**Why the first column does not reproduce: a repair landed after it was
taken.** `menu.MAX_SURCHARGE` = 0.05 s prunes variants by the interior draw
time they cost, so a menu now offers 2.0–3.0 variants per segment against
4.6–6.0 in the stored `out/csail_schedule_new.json`. Fewer fibers to choose
between is less reconfiguration bought — and much less ink spent buying it,
which is why the makespan column moved the same way. The −65 % and the +48 %
were true of the code that produced them and are true of nothing since;
`docs/BENCH.md` has the diagnosis.

**Why the second column is what it is.** The premise "interior draw time is
invariant across variants" is FALSE: a variant is a different path through the
band, so it has a different |dq/ds|, and `writing.draw_duration` stretches the
ink until no joint exceeds `qd_frac` of its velocity limit. Pinning fibers to
save 14 s of transit across the fleet moved the busiest grey arm's DRAW time
20.2 → 30.9 s over the same 2.00 m, and the floor is what the makespan is made
of. Two attempts to repair it in place are recorded because both failed:
costing the band edge in SECONDS rather than radians
(`pwl._edge_travel(mode="time")`, which makes the menu's surcharge a real
draw-time price the DP can pay) recovered a third of the loss, and capping the
admissible surcharge (`menu.MAX_SURCHARGE`) recovered none of it — because
even ONE pinned variant is a constraint the free band DP would not have chosen.
Both knobs are in the code and both are measured.

**And why the third column exists: both of the things this section said would
have to change, changed.** It named two, and neither has anything to do with
the menus:

1. **The cap.** "At `qd_frac` = 0.30 the ink is the constraint, and every one
   of these levers is spending ink time to buy pen-up time" — so the levers
   were being judged against a cap that was itself the bottleneck, and 0.30 is
   half the cap the measurement behind it used (`writing.QD_FRAC` carries the
   argument; `docs/BENCH.md` carries both the 18.9 % it is worth and the row
   of the corpus that refused it, which is why it is still 0.30 and why the
   third column above is measured at `--qd-frac 0.6` rather than at a
   default). On the defaults alone the cap is 104.0 → 84.4 s.
2. **The pricing.** "`allocate.arm_load` — which prices a candidate assignment
   with the single-variant sequencer — would have to price with the cluster
   model, or the balancer balances loads the sequencer then moves." **Built:**
   `allocate.cost_model` is one object carrying both the price and the tour
   (`allocate.py` section 4b-i), and `allocate` constructs exactly one and
   hands it to the balancer and to the sequencing pass, so the two cannot be
   configured apart. On this run it is worth **84.771 → 77.792 s** with every
   other flag held fixed — and the mechanism is visible in the phases. The
   grey pass conducts the SPLIT allocation now, at **27.77 s**; before the fix
   the same pass shipped its UNSPLIT alternative at 34.75 s, because
   `build_phases` conducts both and ships the faster and the cuts had been
   chosen against a tour the cluster DP was never going to run.

So the makespan objection is gone at the animation's own draw speed **once the
cap is out of the way** — and the cap is not out of the way by default, which
is the honest statement of where this stands. `--cluster` remains off: at the
shipped `qd_frac` = 0.30 it still costs 10.9 % of the floor at 0.12 m/s and
0.9 % at the rig's 0.02 m/s (down from 14.5 % and 2.4 % before the pricing
fix), and a default is flipped by a re-run of the whole corpus, not by one
logo. `docs/BENCH.md`'s sweep has the current A/B at every speed.

Turn it on with `--cluster` (`allocate.CLUSTER`).

**The corpus was re-run, and it did not flip the default — it retired the
question (2026-08-20).** Both halves of this section's conclusion survive: at
`qd_frac` 0.30 the menus cost, at 0.60 they pay, and the 0.60 that pays here is
REFUSED on one of the five corpus drawings. What changed is that the pipeline
stopped needing one answer. `csail_schedule.select_profile` conducts the four
(`qd_frac` x menus) combinations PER PROGRAMME and ships the fastest one
`scene_check` certifies, so `allocate.CLUSTER` and `writing.QD_FRAC` stay where
they are as the starting point of a search rather than as a verdict. The logo
ships this section's third column — **77.792 s, 82.4 mm, 39.90 rad** — as
`out/csail_schedule_full.json`, and that file now carries all four outcomes
beside it. `docs/BENCH.md` has the per-drawing grid.

## Lever 4 is built: see docs/IDLE.md (2026-08-19)

"Tuck poses" above — ranked 4th, worth −65 % of the pause and −15.1 s of clock —
is now `aris_sixarm/idle.py`, and it is the last lever this study named that had
not been turned. An arm that finishes lifts its pen and STOPS THERE instead of
going home; a frozen pose that the rest-suffix rule can be *measured* to be
paying for gets a certified minimal retreat; an arm with slack taxis to its next
entry slowly instead of racing there. **108.6 → 105.4 s**, all of it in the
orange pass (68.3 → 65.1 s, which now finishes exactly at its floor).

This study predicted −15.1 s of makespan for tuck poses and got −3.2 s, and the
gap is instructive: it costed a tuck as free, and a tuck is only free for the
pass nobody follows. The grey pass is followed by the orange one, which has to
start somewhere; walking home BETWEEN two passes is strictly worse than walking
home during one, because during a pass the trip overlaps somebody else's
drawing. The −65 % of pause it predicted went the other way too — pause RISES
50.5 → 101.4 s — because an arm that stops early now stands still rather than
flying home, which is the cheaper of the two and exactly what the objective
hierarchy asks for. `docs/IDLE.md` has the rest.

Two things this study could not have seen, because both are about the pose an
arm parks in rather than the path it takes to get there: at the inverted ready
pose a 200 mm pen sits **16 mm below the paper** and a 300 mm pen **113 mm**
below — v1 parked four arms there three times a run and no gate looked, because
paper clearance is checked on the nine FK chain points and the pen is not one of
them. And re-pricing the sequencer for "no trip home" changes which order it
picks, which is enough to route one arm's pen-up straight through another's base
column: a refusal no amount of waiting or re-prioritising can answer, and the
first time this pipeline has had to hand a conductor's verdict back to the
component that made the choice (`coordination.hard_blocks` →
`allocate.resequence(forbid=...)`).

**Everything above this section is a study of conductor v1 and stays true of
it.** `scripts/concurrency_diag.py` still reproduces it — it now pins
`--idle-policy home` for exactly that reason, and skips its per-arm assertions
when handed a reference schedule conducted under a different policy.

**Caveats.** All of this is conductor v1's move set — advance or wait on a frozen path — so "makespan" means the kinematic playback, not an acceleration-limited run. The counterfactual margins are re-thresholds of the same collision images, checked only by the conductor's own test; a real margin change must go back through `scene_check`. Blame for a wait with no blocker at that instant is assigned to the first cell the arm would run into at full speed from there, which is a defensible reading of "what is in the way" and not the only one; 0.0 s of the 148.0 ended up unattributed.
