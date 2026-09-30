# What an arm does when it is not drawing

`docs/CONCURRENCY.md` measured where the pauses came from and named the fix
without turning the knob: **116.6 s of the 148.0 s of pause was the rest-suffix
rule** — an arm may not *arrive* at its final pose until that pose is clear all
the way to the horizon — and `q_seed`, in the middle of the rig, is exactly
where the other arms' transits go. Conductor v1 sent every arm there when it
finished. This is the policy that stops doing that (`aris_sixarm/idle.py`,
`--idle-policy freeze|home`, default `freeze`).

Nothing below weakens a gate. Every candidate is a PATH, and every path is
conducted by the same exact DP against the same collision images at the same
80 mm margin, re-derived afterwards by the same independent `scene_check`.

## The three policies

**FREEZE IN PLACE (default).** An arm that finishes lifts its pen to hover
height and stops. The pose above the ink it has just laid is a pose the fleet
was already avoiding for the length of that stroke, so the horizon-long
clearance the rest rule wants is usually free. The arm is then static geometry —
which the machinery already models correctly, because `_dp`'s rest suffix keeps
checking it after arrival and `scene_check` re-derives it from the played-back
trajectory. It also removes the trip home from the arm's nominal clock (1.2 s
floor, 3–5 s in practice), which lowers the phase floor as well as the pauses.

**MINIMAL RETREAT.** A frozen pose that is genuinely in the way is offered a way
out. "Genuinely" is measured, not guessed: `coordination.rest_delays` re-runs
each arm's own DP with the rest requirement dropped, and only an arm that
actually lost seconds to *where it stops* is a candidate. The search is a few
poses outward — straight up, then back toward the arm's own base — ordered by
joint travel, each certified in its own right (`validate.check_pose`: joint
limits at the planner's 0.15 rad gate, chain above the paper, boom keep-out) and
required to leave every other arm's REMAINING swept tube. On the CSAIL logo it
is one arm, 1.04 rad, against the 3.68 rad that going home would have been.

**JIT PRE-POSITIONING.** An arm with slack does not need to race to its next
entry and stand there. Every pen-up block of its programme is stretched — the
ink never is — by a fraction of the slack a conducted schedule *measured* it to
have, so it arrives at each entry at or before its slot with a fraction of the
per-step motion, and therefore a fraction of the swept-tube slack it charges
everyone else. Drawing has priority over taxiing twice over: the stretch is only
ever spent out of demonstrated slack, and if the stretched fleet conducts slower
the stretch is thrown away.

## What it was worth on the CSAIL logo

**105.42 s against 108.63 (−2.95 %)**, for the same 99.2139 % of the same
11.567 traced metres, the same 49 segments, the same 80 mm margin and the same
certificates. Both phases pass `scene_check` independently.

| | v1: go home | now | pause | clearance |
|---|---|---|---|---|
| phase 1 (grey) | 38.4 s | 38.4 s — *goes home; it is not the last pass* | 21.8 s | 83.8 mm |
| phase 2 (orange) | 68.3 s | **65.1 s** — freezes | 79.6 s | 84.2 mm |
| both + 2 s swap | 108.6 s | **105.4 s** | 101.4 s | 83.8 mm |

**Phase 2 now finishes exactly at its floor.** Dropping the trip home takes the
floor from 68.3 s to 65.1 s and the conducted makespan lands on it: zero
concurrency loss, nothing left for the conductor to win. What it costs is
pause — 28.7 → 79.6 s — because arms that stop early now stand still instead of
flying home, and standing still is the cheaper of the two. The objective
hierarchy says makespan first and pause as the tie-break, and this is what that
choice looks like when it bites.

Not everything froze. Arm 31 had nowhere clear to stop near its last stroke and
went home ALONE; arms 2 and 71 took minimal retreats (0.392 and 2.283 rad,
against the 3.4–3.7 rad a trip home would have been); arm 97 simply stopped.

The JIT pass is the honest disappointment: on phase 1 it cuts standing-still
time by 46 % (21.8 → 11.8 s) for 0.9 s of clock, and on phase 2 it costs 15 s,
so the guard throws it away both times. It stays in the default policy because
it costs nothing when it does not pay, and because a piece with more slack than
this one is exactly where it should. On a synthetic two-arm scene where the
finishing poses ARE apart, freezing alone is worth 9.67 → 5.94 s (−38.6 %)
— `tests/test_csail.py`.

## Freezing only pays for the pass nobody follows

An arm that stops where it finished has not gone home, and **the next pass has
to start somewhere**. Either it starts from the frozen pose — which is what the
sequencer is re-priced for, and which on this logo the conductor refuses
outright — or it starts from `q_seed`, which means walking home anyway. And
walking home BETWEEN two passes is strictly worse than walking home during one:
during a pass the trip overlaps somebody else's drawing, and between passes
nothing overlaps it.

So the rule is: **an intermediate pass goes home, the last pass freezes.** That
is also what a human walking in to swap the pens would ask for.
`--freeze-all-phases` overrides it for experiment; on the CSAIL logo that run
refuses, for the reason below.

## Why the frozen poses of a crowded pass deadlock

**Freezing pays where the arms finish apart, and deadlocks where they finish on
top of each other.** Asked to freeze at the end of the GREY pass and then start
the orange one from those poses, all four inverted arms end up in the crowded
centre with their finishing hover poses inside each other's swept tubes — arm
31's is 142 mm INSIDE arm 2's, arm 71's 44 mm inside arm 97's, and so on round
the ring. The rest-suffix rule then reads: 31 may not stop until 2 has gone
past, and 2 may not stop until 31 has, which is a cycle no priority order
unpicks. Every one of the four can be scheduled on its own; no group of them
can stop. (Measured, not argued: `--freeze-all-phases` refuses, and the refusal
names the pair and the millimetres.)

When a pass cannot be conducted the policy steps back rather than failing:

  1. **per arm** — an arm with nowhere clear to freeze takes v1's answer alone,
     while the others keep theirs (`idle.conduct`);
  2. **per phase** — a phase that still cannot be conducted is re-sequenced and
     conducted under `home` (`csail_schedule.build_phase`), announced in the
     log, and recorded in the summary as `idle.policy` next to `idle.asked`.

Neither is a fallback to *unsafety*: the margin and the gates are not
negotiable and the makespan is only an objective, so the objective is what gets
given up. What phase 2 keeps is the other half of the change — it starts from
the poses phase 1 froze in rather than from `q_seed`, which is a shorter entry
for every arm.

The lever that would unlock phase 2 is not a bigger retreat (the search already
tries 20 cm up and 30 cm back; the arms would have to leave the region
entirely). It is either a placement with more room between the four inverted
arms, or a conductor that can re-route rather than only pause — item 5 on the
`docs/CONCURRENCY.md` list, and still the most machinery for the smallest
measured win.

## Two things that fell out of building it

**The ready pose puts a long pen through the table.** At `Q_READY_INV` the pen
tip is 71 mm above the paper with the stock 110 mm pen, **16 mm below it with
200 mm and 113 mm below with 300 mm** — and v1 parked four arms there three
times a run. Nothing caught it because the paper-clearance gate looks at the
nine FK chain points and the pen is not one of them. `validate.check_pose` now
checks the tip, `scene_check.check_static` applies it to the pen-swap hold, and
freeze-in-place never visits the pose after `t = 0`.

**A pen-up transit the conductor cannot run is an edge of a tour.** Re-pricing
the sequencer's cost model for freeze (no trip home; the second pass starts
where the first one stopped) changes which order it picks, and one of the new
orders sent arm 97 through arm 71's base column — a progress index that is
impossible whatever anyone else does, which is a refusal no amount of waiting or
re-prioritising can answer. `coordination.hard_blocks` turns the refusal into
the tour edge that caused it and `allocate.resequence(forbid=...)` asks
Held-Karp for the best order that does not use it. The allocation does not move:
the same arm draws the same ink, in a different order.
