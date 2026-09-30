# sequencer — the tour of one arm

**Job.** The local planner says how each piece of the drawing can be drawn (a bunch of up to
four alternative plans per piece). The sequencer decides which alternative, in which order and
in which direction, and fills in everything between the pieces: the pen coming off the paper,
the move to the next piece, the pen going down again. Out comes the arm's whole job as a list
of timed motions, from where the arm stands back to where it should end.

Files: `aris/sequencer/tour.py` (the loop and the report), `ladder.py` (the escape ladder), `lift.py` (lift paths and their check),
`draw.py` (drawing motions), `guard.py` (what the sequencer checks itself).

## In and out

`tour(arm, bunches, q_start, obstacles, rules, q_end=None, free_options=None)`

- **In:** the arm, the local planner's bunches, where the arm is, the obstacles (base frame),
  the drawing rules (draw speed, lift height 25 mm, speed share, gates), where the arm should end
  (default: where it started), and settings for the free-space planner.
- **Out:** a generator. It hands out `Motion`s one at a time, in order, and at the end returns
  the pieces it could not draw (`Leftover`, reason and detail). `tour_all` collects everything;
  a `TourReport` passed in is filled as it goes (pieces, lifts, drawing time, pen-up time and
  share, longest free motion, joint travel of the moves, free-space calls and refusals, time
  to the first motion, CPU and wall time).

Per piece it hands out four motions:

| motion | kind | what |
|---|---|---|
| move | free | from where the arm is to the piece's first lift-off configuration (free-space planner) |
| set-down | lower | the pen straight down onto the paper: the lift-off flown backwards |
| drawing | draw | the piece, pen on the paper |
| lift-off | lift | the pen straight up 25 mm at the end of the piece |

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

## The lift-off: the escape ladder

The lift only has to bring the pen into free space; from there the free-space planner takes
over. The free-space planner asks of a start: the pen at least its lifted-pen margin (3 mm) off
the paper, the arm clear of everything and of itself, inside the joint-limit margin. So the
**required** lift is 5 mm (3 mm plus 2 mm), and `rules.lift_height`, 25 mm, is only the
**preferred** height, taken when the arm can have it. The rungs, tried in order at each end of
a piece until one gives a start the free-space planner accepts (`ladder.py`):

| rung | what |
|---|---|
| a | straight up to 25 mm, the arm's shape held (hand orientation and joint 7 fixed) |
| b | straight up as far as the gates allow (1 mm below where one fails), if that is 5 mm or more |
| c | back along the line just drawn, the pen rising to 5 mm above it, up to 50 mm back, each sample in the drawing configuration's own hand orientation and joint 7 |
| d | straight up with the shape allowed to change within the gates: joint 7 and the hand's turn about the normal by up to 0.5 rad, and the pen drifting up to 20 mm toward the base axis; to 25 mm, else 5 mm |
| e | the piece shortened at that end by 5, 10, ... 30 mm, and a to d again; what is cut off is handed back as a leftover `no_free_path`, "cut off at the end of a piece" |

The set-down at the start of a piece is the same ladder from the piece's first configuration,
flown backwards. Every lift is a `lift` motion and every set-down a `lower` motion, solved by
the IK every 2 mm (each answer the one nearest the last; a jump over 0.15 rad is a change of
shape), every sample inside the gates, timed, and checked as flown (the pen exempt from the
paper on its way to or from it); its top must be a configuration the arm can hold with the pen
judged against the paper too. The lift ends at the nearest pen-up configuration, never at one
picked for clearance elsewhere (lesson L53).

**Why the old rule left pieces over** (measured 2026-09-30 on every piece the system planner
left over as "no path", with the old rule, straight up 25 mm or the turns of rung d only):

| piece (system planner, phase, arm) | length | straight up with the shape held | what stops the old lift |
|---|---|---|---|
| spiral, fill phase, arm 13 | 1.77 m (0.82 m finally left) | alternative 1: to 25 mm, every gate kept; alternative 2: 5.0 mm, then joint 5's limit margin | alternative 1 starts at a fold of the IK (two arm shapes meet there): the joints move 86 rad per metre of rise at the paper, slower above; the old rule called any step over 0.05 rad per 2 mm (25 rad/m) a change of shape |
| random line 19, fill, arm 71 | 0.53 m (0.27 m finally left) | both alternatives to 25 mm | the same rule: 34 rad/m at the paper |
| random line 6, phase 1, arm 71 (drawn in a later phase) | 0.92 m | both alternatives 2.5 mm, then joint 2's limit margin (0.149 against 0.150 rad) at the start | the limit margin: under the required 5 mm; rung d or c is needed |
| starburst | none left over | | |

So one real gate (a joint-limit margin within 2.5 to 5 mm of the paper) and one rule that was
too strict (the jump allowed per step, now 0.15 rad per 2 mm: next to a fold a lift moves 30 to
90 rad/m, continuously; a change of shape jumps by 1 rad or more).

A piece with no lift-off on any rung, at either end, in any alternative, becomes a leftover
`no_free_path`, "no lift-off at its start/end", with the reason of the first try.

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

## Measured (2026-09-30, branch aris3, with the escape ladder, machine load 6 to 18, one process)

From park back to park; obstacles of the arm's leading phase (arm 31 phase 2, arm 13 phase 1);
kinematic table on; drawing speed 20 mm/s. Planning time includes the local planner. "Moves" is
the joint-space path length of the moves between pieces. Pen-up time is every motion that is not
drawing (moves, set-downs, lift-offs). No hatch lines lie within 0.80 m of arm 31.

| arm, case | lines | pieces drawn | planning CPU / wall s | first motion s | drawing s | pen up s | pen-up share | longest free s | moves rad | leftovers (m) | checker |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 31 word | 13 | 13 | 2.5 / 2.5 | 1.8 | 122.9 | 40.0 | 0.246 | 7.3 | 48.1 | unreachable 0.198 | 53 / 53 |
| 31 scatter | 27 | 24 | 4.8 / 4.8 | 3.2 | 150.7 | 81.6 | 0.351 | 10.3 | 86.8 | blocked 0.320, too short 0.002, unreachable 0.111 | 97 / 97 |
| 31 starburst | 24 | 26 | 13.7 / 13.9 | 11.3 | 736.1 | 62.3 | 0.078 | 5.2 | 59.9 | blocked 0.908, unreachable 0.429 | 105 / 105 |
| 31 spiral | 3 | 7 | 10.6 / 10.7 | 9.8 | 461.6 | 24.6 | 0.051 | 5.3 | 28.2 | blocked 1.135, unreachable 0.278 | 29 / 29 |
| 31 duotone | 5 | 5 | 4.1 / 4.2 | 2.7 | 217.3 | 31.6 | 0.127 | 7.3 | 37.6 | blocked 0.411, unreachable 0.208 | 21 / 21 |
| 31 lines | 100 | 105 | 39.8 / 40.4 | 32.1 | 2263.0 | 174.4 | 0.072 | 5.6 | 141.1 | blocked 2.325, unreachable 0.015 | 421 / 421 |
| 13 word | 13 | 12 | 2.7 / 2.7 | 2.2 | 121.3 | 29.5 | 0.196 | 4.2 | 35.6 | blocked 0.014, unreachable 0.199 | 49 / 49 |
| 13 hatch | 43 | 43 | 19.8 / 20.0 | 15.7 | 1518.7 | 55.6 | 0.035 | 4.5 | 33.5 | none | 173 / 173 |
| 13 scatter | 24 | 23 | 3.6 / 3.6 | 2.3 | 143.8 | 58.8 | 0.290 | 5.9 | 61.9 | blocked 0.164, unreachable 0.011 | 93 / 93 |
| 13 starburst | 5 | 6 | 2.5 / 2.5 | 2.2 | 53.3 | 24.0 | 0.310 | 9.2 | 23.9 | blocked 0.489, unreachable 0.163 | 25 / 25 |
| 13 spiral | 3 | 2 | 1.9 / 1.9 | 1.9 | 73.7 | 6.4 | 0.080 | 2.9 | 5.6 | blocked 0.523, unreachable 0.147 | 9 / 9 |
| 13 duotone | 5 | 5 | 5.1 / 5.1 | 3.8 | 198.1 | 24.0 | 0.108 | 6.7 | 31.5 | blocked 0.286, unreachable 0.307 | 21 / 21 |
| 13 lines | 100 | 105 | 33.9 / 34.6 | 27.2 | 2228.5 | 177.6 | 0.074 | 7.4 | 141.5 | blocked 1.652, unreachable 0.014 | 421 / 421 |

- **Every one of the 1 517 motions passes the independent checker.** No piece is left over as
  "no path" (before the ladder, same code otherwise: 5 pieces, 0.97 m: 31 scatter 0.104 m,
  31 starburst 0.417 m, 31 lines 0.198 m, 13 lines 0.250 m; all motions passed then too, 1 493).
  More is drawn: 31 scatter 24 pieces instead of 22, 13 duotone 0.30 m more.
- Which rung made the 752 lift-offs and set-downs: a (25 mm straight up) 653, b (straight up,
  lower) 69, c (back along the line) 13, d (shape allowed to change) 17, e (piece shortened)
  none. Lower lifts are shorter: the word's pen-up time, arm 31, is 40.0 s instead of 43.1.
- A lift or set-down takes 0.15 to 0.35 s; the moves between pieces are 34 s of the word's 40 s
  pen-up time for arm 31.
- The sequencer's own CPU (lift-offs, timing the drawings, the moves, the checks) is 0.6 to
  0.8 s for the word, 6.8 to 7.7 s for 100 lines; the rest is the local planner. The ladder adds
  up to 1.5 s per case.
- Free-space planner: 0 refusals.
- **Whole drawings, system planner** (`tests/system_cases.py`, same seeds): "no path" left over
  on the spiral 0.824 m before, none after (the spiral is drawn except 0.084 m beyond every
  arm's reach); random 300 lines 0.271 m before, none after; starburst none either way.
- **Against the old planner, word, arm 31.** Old: planned in 66.8 s, its motion took 67.8 s (it
  drew at 80 mm/s). New: planned in 2.5 s of CPU (first motion after 1.8 s); at 20 mm/s the
  motion takes 162.9 s (122.9 drawing, 40.0 pen up); 13 pieces, 0.198 m of the last "n" beyond
  the reach.

![the tour of the word for arm 31](figures/arm_word_31.png)

Tests: `tests/test_sequencer.py` (the price, the lift-off going straight up and the set-down
being the same backwards, one piece being one drawing motion, an empty drawing, a start nothing can leave, a
piece under a box that is left over while the rest is drawn).
