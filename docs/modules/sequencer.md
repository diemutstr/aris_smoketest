# sequencer — the tour of one arm

**Job.** The local planner says how each piece of the drawing can be drawn (a bunch of up to
four alternative plans per piece). The sequencer decides which alternative, in which order and
in which direction, and fills in everything between the pieces: the pen coming off the paper,
the move to the next piece, the pen going down again. Out comes the arm's whole job as a list
of timed motions, from where the arm stands back to where it should end.

Files: `aris/sequencer/tour.py` (the loop and the report), `lift.py` (lift-off, set-down, shortening),
`draw.py` (drawing motions), `guard.py` (what the sequencer checks itself).

## In and out

`tour(arm, bunches, q_start, obstacles, rules, q_end=None, free_options=None)`

- **In:** the arm, the local planner's bunches, where the arm is, the obstacles (base frame),
  the drawing rules (draw speed, speed share, gates), where the arm should end
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
calibration is proven on the rig; then 3 mm), so the lift is 22 mm. `rules.lift_height` is not
used by the sequencer. The set-down is the lift-off at the piece's first configuration flown
backwards (motion kind `lower`; the lift-off is `lift`). "Checked as flown": every IK sample
inside the joint-limit margin and above the singular-value gate, the top configuration one the
arm can hold with the pen judged against the paper (what the free-space planner asks of a
start), and the timed motion free of every obstacle and of itself (the pen exempt from the
paper on its way to or from it). A step of more than 0.15 rad is a change of arm shape; next
to a fold of the IK, where two arm shapes meet, a straight lift moves 30 to 90 rad per metre of
rise, continuously.

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

## Measured (2026-09-30, branch aris3, the two lift rules, pen clearance 20 mm so a 22 mm lift, machine load 24 to 50, one process)

From park back to park; obstacles of the arm's leading phase (arm 31 phase 2, arm 13 phase 1);
kinematic table on; drawing speed 20 mm/s. Planning time includes the local planner. "Moves" is
the joint-space path length of the moves between pieces. Pen-up time is every motion that is not
drawing (moves, set-downs, lift-offs). No hatch lines lie within 0.80 m of arm 31.

| arm, case | lines | pieces drawn | planning CPU / wall s | first motion s | drawing s | pen up s | pen-up share | longest free s | moves rad | leftovers (m) | checker |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 31 word | 13 | 13 | 8.5 / 16.6 | 6.3 | 123.8 | 47.4 | 0.277 | 9.6 | 50.1 | unreachable 0.173 | 53 / 53 |
| 31 scatter | 27 | 24 | 20.6 / 35.5 | 13.1 | 153.1 | 67.5 | 0.306 | 9.6 | 67.6 | blocked 0.374, too short 0.002, unreachable 0.025 | 97 / 97 |
| 31 starburst | 24 | 28 | 39.0 / 78.5 | 33.1 | 740.1 | 86.4 | 0.105 | 7.8 | 81.7 | blocked 1.204, unreachable 0.023 | 113 / 113 |
| 31 spiral | 3 | 7 | 18.9 / 20.9 | 17.9 | 460.5 | 28.3 | 0.058 | 5.7 | 29.7 | blocked 1.388, no path 0.012, unreachable 0.024 | 29 / 29 |
| 31 duotone | 5 | 4 | 5.9 / 6.0 | 5.0 | 206.5 | 23.9 | 0.104 | 5.3 | 23.6 | blocked 0.567, unreachable 0.011 | 17 / 17 |
| 31 lines | 100 | 105 | 73.0 / 82.0 | 54.0 | 2242.7 | 227.5 | 0.092 | 8.2 | 162.8 | blocked 2.325, no path 0.086, unreachable 0.015 | 421 / 421 |
| 13 word | 13 | 13 | 3.5 / 3.5 | 2.3 | 122.5 | 43.5 | 0.262 | 6.7 | 44.9 | no path 0.012, unreachable 0.185 | 53 / 53 |
| 13 hatch | 43 | 43 | 19.6 / 19.7 | 16.3 | 1518.7 | 65.7 | 0.041 | 7.4 | 41.0 | none | 173 / 173 |
| 13 scatter | 24 | 23 | 3.2 / 3.2 | 2.2 | 143.0 | 60.8 | 0.298 | 5.2 | 58.8 | blocked 0.164 | 93 / 93 |
| 13 starburst | 5 | 5 | 2.0 / 2.0 | 1.8 | 54.1 | 14.1 | 0.207 | 3.1 | 10.9 | blocked 0.623, unreachable 0.015 | 21 / 21 |
| 13 spiral | 3 | 3 | 1.6 / 1.6 | 1.4 | 75.7 | 11.0 | 0.127 | 3.1 | 9.9 | blocked 0.613, unreachable 0.015 | 13 / 13 |
| 13 duotone | 5 | 7 | 13.2 / 25.5 | 10.2 | 200.3 | 39.8 | 0.166 | 8.1 | 38.7 | blocked 0.452, unreachable 0.015 | 29 / 29 |
| 13 lines | 100 | 105 | 124.9 / 228.9 | 102.8 | 2228.8 | 222.6 | 0.091 | 6.1 | 153.6 | blocked 1.643, no path 0.022, unreachable 0.014 | 421 / 421 |

- **Every one of the 1 533 motions passes the independent checker.**
- **"No path": 0.132 m in all**, every bit of it cut off at a piece end by rule 2 (spiral arm 31
  12 mm, word arm 13 12 mm, random lines 86 and 22 mm); no whole piece is lost. With the arm's
  shape held (the first version of rule 1, same day) it was 3.53 m, 1.85 m of it one piece of
  the spiral; with the five-rung ladder (earlier the same day, 5 mm lift) nothing.
- **Pen-up time on the word:** arm 31, 47.4 s (shape held 43.5 s, ladder 40.0 s); arm 13,
  43.5 s (37.4 s, 29.5 s). Drawing time 123.8 and 122.5 s: more of the word is drawn than with
  the shape held. The 22 mm lift and its turns cost a few seconds over the ladder's 5 to 25 mm.
- The sequencer's own CPU (lift-offs, timing the drawings, the moves, the checks) is 1.2 to
  2.5 s for the word, 19 to 22 s for 100 lines, measured at load 24 to 50 (about twice what a
  quiet machine gives); the rest is the local planner.
- Free-space planner: 0 refusals.
- **Against the old planner, word, arm 31.** Old: planned in 66.8 s, its motion took 67.8 s (it
  drew at 80 mm/s). New: planned in 2.4 to 8.5 s of CPU depending on the machine's load (first
  motion after 1.7 to 6.3 s); at 20 mm/s the motion takes 171.2 s (123.8 drawing, 47.4 pen up).

![the tour of the word for arm 31](figures/arm_word_31.png)

Tests: `tests/test_sequencer.py` (the price, the lift-off going straight up and the set-down
being the same backwards, one piece being one drawing motion, an empty drawing, a start nothing can leave, a
piece under a box that is left over while the rest is drawn).
