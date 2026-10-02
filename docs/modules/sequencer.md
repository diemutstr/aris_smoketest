# sequencer — the tour of one arm

**Job.** The local planner says how each piece of the drawing can be drawn (a bunch of up to
four alternative plans per piece). The sequencer decides which alternative, in which order and
in which direction, and fills in everything between the pieces: the pen coming off the paper,
the move to the next piece, the pen going down again. Out comes the arm's whole job as a list
of timed motions, from where the arm stands back to where it should end.

Files: `aris/sequencer/tour.py` (the loop and the report), `lift.py` (lift-off, set-down, shortening),
`draw.py` (drawing motions), `guard.py` (what the sequencer checks itself).

## In and out

`tour(arm, bunches, q_start, obstacles, rules, q_end=None, *, report=None, verify=None, batches=None, refill=32)`

- **In:** the arm, the local planner's bunches, where the arm is, the obstacles (base frame),
  the drawing rules (draw speed, speed share, gates), where the arm should end
  (default: where it started).
- **Out:** a generator. It hands out `Motion`s one at a time, in order, and at the end returns
  the pieces it could not draw (`Leftover`, reason and detail). A `TourReport` passed in is filled as it goes (pieces, lifts, drawing time, pen-up time and
  share, longest free motion, joint travel of the moves, free-space calls and refusals, time
  to the first motion, CPU and wall time).

Per piece it hands out four motions:

| motion | kind | what |
|---|---|---|
| move | free | from where the arm is to the piece's first lift-off configuration (free-space planner) |
| set-down | lower | the pen straight down onto the paper along the lift-off's path, landing at 10 mm/s |
| drawing | draw | the piece, pen on the paper |
| lift-off | lift | the pen straight up by the pen clearance plus 2 mm (22 mm today) at the end of the piece |

and at the end one move to `q_end`. Every motion starts exactly where the one before ended
(to 1e-9 rad), starts and ends at rest, and ends in a configuration the arm can hold: before a
motion is handed out, its end is checked with the kernel (joint-limit margin, clearance to
every obstacle, the arm against itself; with the pen exempt from the paper where it is meant to
touch it). An empty drawing gives only the move to `q_end` (nothing, if the arm is already
there).

## How the next piece is chosen

Greedy. From where the arm is, every piece still to draw is priced in each of its alternatives
and both directions: the time the slowest joint needs to reach that alternative's first drawing
configuration at the allowed speed (joint distance divided by 0.3 of the joint speed limit).
That costs nothing, so no free-space move is planned just to compare candidates (lesson L75).
The cheapest is tried: its lift-offs and drawing motion are made, then the free-space planner
is asked for the move. If anything refuses, the next cheapest is tried. Ties go to the earlier
piece, so the same input always gives the same tour, also from a fresh process.

**Batches.** The pieces may come in batches (`batches`, an iterator of lists of bunches, from the
arm planner). The sequencer chooses among the pieces it has; whenever fewer than `refill` (32)
are alive, it takes the next batch, waiting for it if it is not there yet. That depends only on
the pieces, never on when a batch arrives, so the tour is the same however fast they come.

## The lift-off and the set-down: two rules

Decided by Pete, 2026-09-30: as simple and reliable as possible.

1. **A piece starts and ends with the pen rising straight up along the paper normal by the pen
   clearance plus 2 mm, following at every 2 mm the nearest IK answer that passes the gates
   (at most 0.15 rad from the last), with the hand's spin about the normal and joint 7 changing
   evenly over the rise by the smallest amounts, each within 0.5 rad, that let every sample
   pass; the timed motion is checked as flown.**
2. **If rule 1 fails at an end, the piece is shortened at that end by 1 cm and rule 1 is tried
   again, up to 5 cm; what is cut off is a leftover "no path".**

The pen clearance is rig.json's `pen_lifted_to_paper_m`: the free-space planner keeps the pen
that far above the paper, so the lift is just enough to hand over to it. Today 20 mm (until the
calibration is proven on the rig; then 3 mm), so the lift is 22 mm. The set-down (motion kind `lower`) follows the lift-off's path at the
piece's first configuration backwards, timed so that the pen never goes faster than
`rules.landing_speed` (rig.json `drawing.landing_speed_m_per_s`, 10 mm/s): the pen's path
length along the descent is the arc length and the landing speed its cap, as on the hardware,
where a fast landing against the controller's soft spring spikes the force. A 22 mm set-down
takes 2.3 s instead of 0.3. The lift-off (kind `lift`) keeps its fast timing: leaving the paper
fast does no harm. "Checked as flown": every IK sample
inside the joint-limit margin and above the singular-value gate, the top configuration one the
arm can hold with the pen judged against the paper (what the free-space planner asks of a
start), and the timed motion free of every obstacle and of itself (the pen exempt from the
paper on its way to or from it). A step of more than 0.15 rad is a change of arm shape; next
to a fold of the IK, where two arm shapes meet, a straight lift moves 30 to 90 rad per metre of
rise, continuously.

**The drawing surface.** The system planner hands lines on the drawing surface, the press
(`rules.press`, 3.5 mm) below the real paper. The pen draws there; the lift is measured from the
real paper (the paper plane among the obstacles), so it ends the pen clearance + 2 mm above the
paper and rises the press further than from the paper itself; the set-down lands on the surface
at the landing speed. Tested with lines 3.5 mm below the paper (`test_arm_planner`).

A piece where rule 2 runs out at an end in every alternative and direction becomes a leftover
`no_free_path`, "no lift-off at its start/end", with rule 1's reason.

The turns are tried smallest first, from none (the shape held) to 0.5 rad each, as even
changes over the whole rise. Turns chosen step by step instead zigzag, and the timing slows at
every corner: such lifts took up to 3.2 s instead of 0.3.

(Earlier the same day: a five-rung ladder, then the two rules with the shape held, which lost a
1.85 m piece of the spiral whose end ran into a joint-limit margin on the way up.)

## The drawing motion

The local planner's joint path is timed here in one call (`kernel.retime`, pen within 0.1 mm of
the planned pen path) and checked as flown against every obstacle: one piece, one drawing
motion. The timing step keeps the pen within 0.5 % of the draw speed (on two of 1 409
drawings it read 3.6 to 3.8 % over at 1 kHz; see "What the checker says").

The local planner times each plan only to report its duration (corner to corner) and hands
over no trajectory, so the sequencer times it again; the local planner's `draw_time` is not the
flown duration (left as is, orchestrator 2026-09-30).

A drawing that cannot be timed or flown makes the candidate unusable; if no alternative works
the piece is a leftover `unreachable`, "drawing: ...".

## When the free-space planner refuses

The refusal counts only from where the arm is; the next cheapest candidate is tried (another
alternative, another piece). A piece all of whose candidates were refused from here becomes a
leftover `no_free_path` with the refusals. After 12 refusals in one step the pieces refused so
far become leftovers and the step starts again with the rest. On the fixed cases below the
free-space planner never refused (0 of 382 calls). If the final move to `q_end` is refused, the
report says so (`end_refusal`); the last motion still ends holdable.

## The checker in the loop (`verify`)

`tour(..., verify=None)`. The drawing server hands in `verify(motion, q_before) -> dict` (at
least `passed`, `tightest`): the independent checker, an opaque callable; the sequencer never
imports the checker. The law: **a piece is drawn only if every motion of its group passes the
checker; otherwise the piece is left over as `failed_check` with the checker's word, and the
arm plans on from where it was.** The group is the move to the piece, the set-down (`lower`),
the drawing and the lift; they are checked in that order, the first from where the arm stands
(the end of the last motion handed on), each next from where the one before ended. The first
refusal ends the group: none of its motions is handed on, and the leftover's detail says which
motion ("the lift (lift motion 4 of 4 of the piece's group): ...") and the checker's tightest.
A motion that passed is handed on carrying the checker's dict (`Motion.checked`). The group is
checked before the next piece is chosen, so nothing is ever planned on top of a motion that has
not passed; no speculation, no rewinding. The move home is checked too; if refused, the report's
`end_refusal` says "failed_check: the move to q_end: ...". Without `verify` everything is as
before (same motions, `checked` None).

Measured 2026-09-30 with smooth timing, the real checker as `verify` (`tests/arm_cases.py
--verify`), machine load 8 to 13, one process (the 100.2 s wall is a load spike):

| arm, case | motions | planning CPU without / with | wall without / with | inside verify | failed_check |
|---|---|---|---|---|---|
| 2L word | 53 | 2.6 / 8.6 s | 2.6 / 8.6 s | 6.1 s | 0 |
| 2L random lines | 421 | 41.5 / 81.3 s | 42.7 / 81.5 s | 41.3 s | 0 |
| 1L word | 53 | 2.3 / 6.9 s | 2.3 / 6.9 s | 4.5 s | 0 |
| 1L random lines | 421 | 57.5 / 92.3 s | 100.2 / 93.5 s | 54.2 s | 0 |

- Every motion handed on carries `checked["passed"]` True; the tours are the same motions as
  without `verify`.
- The checker costs 0.08 to 0.13 s per motion here: the word plans in 7 to 9 s instead of 2 to
  3, 100 lines in 81 to 92 s instead of 42 to 58 (one process). The motions of 100 lines take
  about 40 minutes to draw.

## What the checker says

Every motion of every case below goes through the independent checker (branch aris3,
2026-09-30): **1 517 of 1 517 pass**; the tightest clearance beyond the demanded one is 0.8 mm.
Earlier failures, all settled in the checker or the timing step: link 1 against its own struts,
the pen at the paper end of a set-down or lift-off (now the kinds `lower` and `lift`), the pen
speed on two rim lines 3.7 % over the draw speed (`tests/data/arm_speed_31_lines_*.npz`), and
"never stops" at a sharp corner at 80 mm/s (`tests/data/arm_stop_31_word_0.npz`; not
re-measured since the checker's stop limit changed).

## What it cannot do

- Greedy on the move to the next piece only; it does not look at where a piece leaves the arm.
  On the word, a better order of the same alternatives (moving one piece at a time, with the
  same price) prices the moves about 20 % lower (17.4 against 21.6 s), on a corpus drawing 6 %.
  The next lever: choose order, alternative and direction together along the tour
  (OPTIMIZATION_NOTES 13, lesson L54).
- The price is a lower bound on the move's time; the free-space moves took 1.1 to 3 times it,
  and a detour up to 9 times.
- A piece refused from where the arm is may be reachable from elsewhere; it is not offered
  again later.

## Measured (2026-10-01, branch aris3, smooth timing, set-down landing at 10 mm/s, 22 mm lift, 8 workers, batches of 32 with refill 128, machine load 6 to 17)

From park back to park; obstacles of the arm's leading phase (arm 2L phase 2, arm 1L phase 1);
kinematic table on; drawing speed 20 mm/s. Drawings, set-downs and lifts are timed in the
retimer's smooth mode (commit d8895c8). Planning CPU counts the 8 local-planner workers; first
motion is wall time. "Slowest" is the checker's slowest mid-line pen speed over the case's
drawings. Pen-up time is every motion that is not drawing. No hatch lines lie within 0.80 m of
arm 2L.

| arm, case | lines | pieces drawn | planning CPU / wall s | first motion s | drawing s | pen up s | pen-up share | longest free s | leftovers (m) | checker | slowest mm/s |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 2L word | 13 | 13 | 8.1 / 2.7 | 1.4 | 123.6 | 72.9 | 0.371 | 9.6 | unreachable 0.173 | 53 / 53 | 1.30 |
| 2L scatter | 27 | 24 | 11.4 / 3.9 | 1.4 | 147.9 | 115.4 | 0.438 | 9.6 | blocked 0.374, too short 0.002, unreachable 0.025 | 97 / 97 | 5.01 |
| 2L starburst | 24 | 28 | 22.7 / 6.7 | 2.7 | 722.2 | 140.0 | 0.162 | 7.8 | blocked 1.204, unreachable 0.023 | 113 / 113 | 5.00 |
| 2L spiral | 3 | 7 | 15.6 / 8.9 | 7.5 | 450.9 | 42.3 | 0.086 | 5.7 | blocked 1.388, no path 0.012, unreachable 0.024 | 29 / 29 | 5.02 |
| 2L duotone | 5 | 4 | 9.0 / 2.9 | 2.1 | 198.6 | 30.3 | 0.133 | 5.3 | blocked 0.567, unreachable 0.011 | 17 / 17 | 5.05 |
| 2L lines | 100 | 105 | 64.3 / 20.1 | 6.9 | 2202.4 | 424.3 | 0.162 | 7.5 | blocked 2.325, no path 0.087, unreachable 0.015 | 421 / 421 | 4.99 |
| 1L word | 13 | 13 | 6.8 / 2.1 | 1.1 | 122.5 | 62.9 | 0.339 | 5.5 | no path 0.010, unreachable 0.185 | 53 / 53 | 1.35 |
| 1L hatch | 43 | 43 | 33.2 / 9.2 | 3.9 | 1517.0 | 153.6 | 0.092 | 6.8 | none | 173 / 173 | 5.09 |
| 1L scatter | 24 | 23 | 9.7 / 2.8 | 1.2 | 142.9 | 107.1 | 0.428 | 5.2 | blocked 0.164 | 93 / 93 | 5.00 |
| 1L starburst | 5 | 5 | 6.3 / 1.9 | 1.6 | 54.0 | 23.0 | 0.299 | 3.1 | blocked 0.623, unreachable 0.015 | 21 / 21 | 4.99 |
| 1L spiral | 3 | 3 | 4.0 / 2.1 | 1.8 | 72.6 | 16.6 | 0.186 | 3.1 | blocked 0.613, unreachable 0.015 | 13 / 13 | 4.99 |
| 1L duotone | 5 | 7 | 8.6 / 3.5 | 2.3 | 188.4 | 49.6 | 0.208 | 7.2 | blocked 0.452, no path 0.011, unreachable 0.015 | 29 / 29 | 4.99 |
| 1L lines | 100 | 105 | 53.1 / 16.8 | 5.6 | 2198.3 | 429.4 | 0.163 | 6.1 | blocked 1.643, no path 0.022, unreachable 0.014 | 421 / 421 | 4.99 |

- **Every one of the 1 533 motions passes the independent checker.**
- **Slowest mid-line pen speed:** 5.0 mm/s or more on every drawing (the median drawing's
  slowest point is 5.1 mm/s), except word:10, the "w" of the word, which has a real 148-degree
  corner: 1.30 mm/s on arm 2L, 1.35 mm/s on arm 1L.
- **Against corner timing** (the table before, same lift rules, one process): time on the rig
  0.2 to 2.5 % shorter (arm 2L word 170.8 against 171.2 s; 31 random lines 2 428 against
  2 470 s; 13 random lines 2 416 against 2 451 s; 31 starburst 806 against 827 s); pen-up time
  about the same (the word, arm 2L: 47.2 against 47.4 s).
- **The flown check, 4 times finer** (`Guard.flown`, 2026-09-30): the timed curve is checked
  at its samples plus 3 points of the same cubic between each two, and the cubic's sag is charged
  on that finer gap. Before, arm 2L's scatter lost a 72 mm piece (corpus:scatter:38:0): the
  finger was 0.76 mm beyond the planning margin at the samples, and the bounds over the 50 ms
  gaps took 0.77 mm; it is drawn now (24 pieces). The check stays a lower bound against the
  planning margins. Candidates it no longer refuses change a few tours: pen-up time on the word,
  arm 1L, 37.5 s instead of 43.0; arm 2L's 100 lines 212.8 s instead of 225.8; arm 1L's 100
  lines 219.1 instead of 218.5; the other cases are the same motions. It costs 8 to 20 ms of
  CPU per checked lift or drawing instead of 6 to 12 ms (0.1 to 2.9 s more per case).
- **"No path": 0.131 m in all**, every bit of it cut off at a piece end by rule 2 (spiral arm 2L
  12 mm, word arm 1L 10 mm, random lines 87 and 22 mm); no whole piece is lost. With the arm's
  shape held (the first version of rule 1, same day) it was 3.53 m, 1.85 m of it one piece of
  the spiral; with the five-rung ladder (earlier the same day, 5 mm lift) nothing.
- **The landing speed** (2026-10-01): every set-down takes 2.29 to 2.31 s instead of the lift's
  0.26 to 0.33 s, and the pen never exceeds 9.98 mm/s on it. The word's time on the rig: arm 2L
  196.6 s instead of 170.8 (+15 %), arm 1L 185.3 s instead of 160.0 (+16 %); pen-up share 0.371
  and 0.339. On 100 lines pen-up time roughly doubles (424 against 213 s, arm 2L), 8 % of the
  time on the rig. The checker passes every set-down (its pen-speed rule is for drawings).
  One alternative changes: arm 1L duotone loses 11 mm more at a piece end.
- **Pen-up time on the word, before the landing speed:** arm 2L, 47.2 s (shape held 43.5 s,
  ladder 40.0 s); arm 1L, 37.5 s (37.4 s, 29.5 s). Drawing time 123.6 and 122.5 s: more of the word is drawn than with
  the shape held. The 22 mm lift and its turns cost a few seconds over the ladder's 5 to 25 mm.
- The sequencer's own CPU (lift-offs, timing the drawings, the moves, the checks) is 1.0 s for
  the word, 7.2 to 8.2 s for 100 lines; the rest is the local planner.
- Free-space planner: 0 refusals.
- **Against the old planner, word, arm 2L.** Old: planned in 66.8 s, its motion took 67.8 s (it
  drew at 80 mm/s). New: planned in 2.2 s wall, 7.6 s of CPU over 8 workers (2.6 s in one
  process), first motion after 1.3 s; at 20 mm/s the motion takes 170.8 s (123.6 drawing, 47.2
  pen up).

![the tour of the word for arm 2L](figures/arm_word_31.png)

Tests: `tests/test_sequencer.py` (the price, the lift-off going straight up and the set-down
being the same backwards, one piece being one drawing motion, an empty drawing, a start nothing can leave, a
piece under a box that is left over while the rest is drawn).
