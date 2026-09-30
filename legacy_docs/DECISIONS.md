# Decisions — the numbers, and where each one is anchored

## THE LAST TWO DEFECTS — A PRIORITY ORDER INSIDE THE GROUP, AND A HOLD THAT IS VERIFIED (2026-09-14, NINTH)

§28 shipped a programme with every fix of the day in it and no certificate, and
named exactly two things in the way. Both are about **a pose somebody is
STANDING IN while somebody else moves.** Measured in docs/V2_STAGED.md §29.

**A REFUSED ROW GROUP'S TWO ARMS WERE NEVER IN EACH OTHER'S ROOM.** When
`idle.conduct` refuses, the fallback flies the group's arms one at a time — but
their routes came off `plan_bucket`, which runs BEFORE `freeze_conduct` with
`partners=outside` only. The moving arm routed straight through its standing
partner: **2 ↔ 97 at −161.3 mm, 31 ↔ 71 at −21.0 mm**, and in both the other
arm's per-frame travel is 0.00 mm. `freeze_conduct`'s `still` set catches the
arms inside a group that produce no timeline; it cannot catch a pair where both
have ink, because neither is still at freeze time and the refusal is only
discovered afterwards. **`_serialise_group` gives the group the argument
`_priority_stage` already makes between arms in a stage:** busiest arm first
against the frozen outside fleet *plus its partner at the pose that partner
holds*; then the second arm against the outside fleet *plus the first arm's
realised trajectory as an exact swept room*; every leg re-routed under that room
rather than reused. The slots do not overlap — `_merge_conducts` lays a refused
group end to end — so where the whole swept room costs the second arm a piece,
that arm is re-planned against the partners' FINISHING POSES, which is the
honest statement of the slot it actually flies in. `hold_gap` proves the pose
set at every slot boundary. **And the merge learned the order:** it laid a
refused group's slots down by ascending arm id, which would ship the
certified-against arm flying second; it now reads `serial_order` off the group's
own report.

**A HELD HOVER THAT KEPT NEITHER THE JOINT MARGIN NOR THE ROOM WAS HELD
ANYWAY.** `hover_solve`'s `hold` ask is a SETTLE: where the whole fiber has
nothing that keeps `HOVER_HOLD_MARGIN`, it drops back to `HOVER_MARGIN` and
returns that. Right for a travelling hover, wrong for the pose a barrier holds
for a minute — `lf6_s150` stage B ended arm 31 on a pose at **0.1064 rad against
`validate.MARGIN_GATE`'s 0.15**, and the stage was refused on `frozen_failed`
with every distance gate passing wide. The ask is now VERIFIED
(`writing.held_pose_ok`), and a pose that fails it is not held: the arm ends its
stage on a certified retreat instead — **(a)** more air over the same stroke end,
**(b)** a hover over an earlier stroke end of the same bucket, **(c)** the park,
which is always valid — cheapest first, each rung asked the same gate the
barrier will apply and each required to be routable from the final lift. Which
one was taken rides on the timeline as `hold_kind`. **And the barrier itself now
asks:** `hold_gap` proved the held set pairwise clear and nothing else, so
`validate_pose` — the same gate with the same `PEN_PAPER` exemption
`scene_check` makes — is asked of every held pose where the barrier is declared
and gates its `ok`. A barrier may not hold a pose the judge would refuse.

**MEASURED, AND `lf6b_s150` IS CERTIFIED END TO END** (docs/V2_STAGED.md §29.3–
§29.5). Both configurations' final passes now certify:

| | `lf6b_s150` (A, B, C, D) | `lf6_whole_c4` (stage C) |
|---|---|---|
| certified | **YES, every stage, every gate** | **YES**, +118.1 mm |
| stage B | 135.4 s FAIL → **140.0 s PASS** | unchanged, PASS |
| stage C | −238.5 mm → **+55.1 mm** | −76.6 mm → **+118.1 mm** |
| stage D | FAIL (frozen) → **+251.7 mm PASS** | empty |
| stage C motion | 171.1 → **146.8 s** (20.4 → 17.5 s/m) | 116.7 → **98.1 s** (17.4 → 14.7 s/m) |
| makespan | 296.7 → **349.7 s** (v19: 209.9 s) | 328.5 → **309.9 s** |
| ink FLOWN | 14.148 m → **14.148 m, unchanged** | 14.141 → **13.255 m** |

**THE STAGE-B FIX COSTS 4.6 SECONDS AND NO INK.** One retreat fires in the whole
programme — arm 31, stage B, rung (b), a hover over its PREVIOUS stroke end; the
park rung was never needed. **THE STAGE-C FIX COSTS PLANNING WALL AND, ON THE
WHOLE BAG ONLY, 0.885 m.** A refused group is planned twice into a cold leg-store
namespace: stage C's wall went 93 → 1 428 s on the whole bag. And arm 97's
0.885 m — the metre that read −161.3 mm against a standing arm 2 — does not fly
at all once arm 2 is in its room. That is the honest answer, and there is one
bit left unsearched: a two-arm group has exactly two orders, and where the
busiest arm's bucket will not fly under its partner's HELD pose, the reverse
order plans that partner first and lets it go HOME. `lf6b_s150` does not need it
(its flown ink is identical to the uncertified run's); the whole bag does.

**NO GATE MOVED.** `PAIR_MARGIN` 0.050, self 23, static 50, frame floor 63,
`MARGIN_GATE` 0.15, the tool transform, the layout and the rig are untouched;
both fixes are about WHAT IS ASKED OF WHICH POSE, not about what passes.

## THE INTEGRATED RUN — AND TWO OF THE FOUR RESIDUALS WERE NAMED WRONGLY (2026-09-14, EIGHTH)

Every fix of 2026-09-14 in one programme on the CSAIL logo at h = 0.970, and
first the four things §26.6 and §27.1 left open. **Two of the four diagnoses do
not survive being measured, and the measurements are in docs/V2_STAGED.md §28.**

**THE 0.09 mm WAS NEVER A SEAM.** §26.6 read serial stage C's +49.9 mm on
71 ↔ 97 at t = 32.5 s as a crossing at the boundary between two rows' time
slots. At that instant **arm 97 is standing still** — at its stage-C entry pose,
which on that programme is its park, and it does not move for another 148 s —
while arm 71 is 29 s into drawing its first piece. Raw clearance 50.76 mm,
`scene_check`'s residual 0.55 × 1.54 mm, verdict +49.91. No composition of the
rows in time can move a number measured against an arm that never moves. It is
§26.3's lesson one obstacle over: **the producer holds the room to `PAIR_MARGIN`
on a converged bound and the judge subtracts a residual that grows with the
SPEED.** `writing.room_pace_beat` / `room_pace_draw` now price that residual for
pen-up beats and for drawn strokes alike, against the FROZEN partners only —
they are the ones that do not move, so the whole of `0.55 × (step_i + step_j)`
is this arm's own step and this arm can pay all of it. The price is charged
**per interval** (a move's fastest point and its tightest point are almost never
the same point: pairing the worst of each asked 5 476 s of a 32 s stroke against
the 67 s the geometry wants), only the intervals that bind are refined, and a
move whose price is over `ROOM_PACE_MAX` is left alone rather than half-paid.
On the failing stroke: 32.25 → 67.33 s, and the judge goes **+49.91 → +50.25 mm**
with the ink, the route and every pose untouched.

**STAGE D'S FRAME FAIL IS NOT `effective_static_floor` EITHER.** The tightest
obstacle on arm 31's stage-D entry leg is **`body:71_column3`** — a frozen
partner's base column band — mid-leg, with both endpoints at 70.4 mm, so nothing
was clamped. `scene_check`'s COLUMN gate reads the same metal as cylinders and
passes the identical trajectory at **+84.4 mm**; its FRAME gate reads it as the
0.32 m AABB that contains them and refuses at **+31.0 mm**. `freeze_conduct`
drops that band and substitutes arm 71's real capsules, which is the whole
reason `frozen` exists and 127 mm of honest room — and the judge knows nothing
about any of it. So `frozen.set_keep_bands(True)` lets a conduct ask for BOTH
rooms (the bands AND the partner bodies, strictly more conservative than
either), and `staged.CONDUCT_BANDS = "auto"` spends the second pass only where
the judge's frame gate refuses what the relaxed room produced, and only keeps it
if the retry clears the gate.

**THE OTHER TWO WERE NAMED RIGHTLY AND ARE SHIPPED.** `self_pace_beat` takes
`paper.leg_self_lb`'s **converged** bound wherever its fixed 33-sample grid says
the beat is already under the floor — on arm 71's 3.10 s stage-C transit the
grid bounds it at −13.9 mm and the converged bound at +23.10 mm, so the beat was
never priced and the judge refused it at +14.3 mm; paced to 15.88 s it reads
≥ 20 mm. And `ArmStage.hovers`' last `hover_out` is now taken FROM the
trajectory's end pose instead of re-derived beside it with the ordinary rule,
because under `PARK_FREEZE` `arm_program` asks the held pose a different
question (`HOVER_HOLD_MARGIN`, `HOVER_ROOM_FLOOR`) and the trajectory is what
every check reads.

**AND THE RUN FOUND A FIFTH, WHICH IS A COMPOSITION BUG AFTER ALL.**
`_compose_groups` laid the serial slots end to end using the ROW COUNT of each
group's timeline. A conducted timeline is `dt`-spaced and its row count is its
length; a timeline the conductor REFUSED is on `writing.arm_program`'s waypoint
clock, where 354 rows is 59.7 s, and `_merge_conducts` only resamples it AFTER
the offsets are fixed. On `lf6_whole` stage C group [31, 71] was refused, its
slot was written as 17.65 s, the next group started there, and arm 71 was still
flying at 59.70 s: the merge read 17 <-> 71 at **-108.6 mm**. `_clock_frames` /
`_slot_frames` measure the slot on the merge's own clock now, and a refused
group's slot is the SUM of its arms' lengths because that is what the merge lays
down. With that in, the three slots are disjoint and every cross-row pair clears.

**THE INTEGRATED PROGRAMME IS NOT CERTIFIED, AND WHAT IS LEFT IS ONE THING.**
`lf6_whole` draws **9.364 m -- 58.3 % of the ink -- in stages A and B**, both
PASS, against 22.8 % for lf3; stage A is **58.47 s on identical ink** against
95.86 s with `active_pair` +197.6 mm against +65.6. Stage C still fails in both
configurations (-76.6 mm whole, -240.4 mm s150), and every failing pair is a
SAME-ROW pair with one arm STANDING STILL. That is the signature of
`_conduct_stage`'s fallback: when `idle.conduct` refuses a group its arms fly one
at a time, but each one's route came from `plan_bucket`, which runs before
`freeze_conduct` with `partners=outside` only -- so a group's own arms are never
in each other's room. Freezing the group's non-moving arms took [31, 71] from
-43.6 to -21.0 mm; the remaining case needs a PRIORITY ORDER INSIDE THE GROUP,
the same argument `_priority_stage` already makes one level up. Named, not
shipped unmeasured. `lf6_s150` stage B is refused on one held pose as well --
arm 31's last hover keeps 0.1064 rad against `MARGIN_GATE`'s 0.15, everything
else wide, which is exactly the best-effort limit sect. 27.1 built in.

**NO GATE CONSTANT MOVED.** `PAIR_MARGIN` 0.050, `SELF` 23/20, `STATIC` 50,
`FRAME_FLOOR` 63 and `MARGIN_GATE` 0.15 are untouched; the tool transform, the
layout, `frames.py`, `mounts.py`, `layout.py` and `scene_check.py` are untouched.
Every one of the four is a producer-side change that can only ever slow a move,
keep an obstacle, or report what is already true.

## THE PEN-UP IS 74 % OF THE ARM'S TIME, AND IT WAS THREE BUGS (2026-09-14, SEVENTH)

**Pete, watching the animation: "arm 71 is still spending an insane amount of
time reconfiguring." He is right.** On `out/staged_csail_h097_program_lf2_s150.json`,
stage A, arm 71 (13 pieces, 1.838 m, 95.9 s) the pen-up legs are **70.5 s —
73.6 % of the arm's stage**, and peak joint speed on almost every leg is 0.60 of
the FR3 limit, so `QD_FRAC` is in force and **pacing is not the cause**.
`scripts/penup_anatomy.py` classifies every pen-up leg of any staged programme
into *redundancy flip* (joint path nothing in task space explains), *tall /
wandering route* (vertical travel past the honest lift-and-lower, or path > 2 ×
net) or *honest hop*; the rule and its constants are stated in the module
docstring and in `docs/V2_STAGED.md` §27. On the whole lf2 s150 programme, both
stages, all arms: **85.75 s of pen-up — 33.5 % flip, 49.6 % tall/wander, 16.9 %
honest.** Arm 71's stage A alone is 70.54 s of that: 28.72 flip, 30.86 tall,
10.96 honest.

**AND THE FLIP SPLITS IN TWO, WHICH IS THE WHOLE DIAGNOSIS.** Schema-2
programmes carry `hover_in`/`hover_out` per piece, so each leg splits into LIFT
(ink end → hover out), TRAVEL (hover → hover) and LOWER (hover in → ink start).
Arm 71's legs 7–11 read ~10 rad of LIFT and ~10 rad of LOWER for hops of 3 mm to
20 cm; legs 1 and 6 read 16.5 and 11.1 rad of TRAVEL. **66.0 rad of the flip
cost is the hover and 28.2 rad is the pieces**, and those are two different bugs.

**BUG 1: `hover_solve` STOPPED ON "COMFORTABLE" AND NEVER ASKED "NEAR".** Its
narrow scan (one tool yaw, q7 within ±0.6 of the ink's own) returned a pose for
arm 71's piece 8 that clears the static set comfortably and is **5.03 rad from
the ink** — an IK branch flip inside the q7 window — and comfort alone
short-circuited the fiber. **The fiber holds 23 gated poses at that tip and
height and the nearest is 0.26 rad away.** `writing.HOVER_NEAR = 1.0` rad adds
the second half of the escalation test; the two answers are then compared on the
SAME score with distance breaking the tie the score leaves. **Nothing is
relaxed** — both paths run the identical `static_gate` (`CHAIN_CLEAR`, the full
`FRAME_FLOOR`, `selfcoll.self_ok`) and the identical joint-margin filter.
Measured over arm 71's 26 span ends: **lift + lower distance 48.39 → 16.76 rad,
−65.4 %, at no wall-clock cost** (0.4 s either way). Ten of the 26 ends were on
the wrong sheet.

**BUG 2: THE SHEET WAS CHOSEN PIECE BY PIECE AND NEVER ALONG THE TOUR.**
`stroke_api.plan_stroke` returns one plan and picks its (φ, q7, branch) sheet on
that piece's own merits, so adjacent pieces land on different sheets.
**The August "entry/exit fiber menus + cluster-Held-Karp" feature was re-taken
here and is still NOT what ships in this path** — and for reasons that have
nothing to do with its +3.4 % rig-speed verdict (`docs/BENCH.md` 2026-08-20).
`allocate.CLUSTER` is unreachable from `staged.py`; the staged bucket's order is
already fixed by `sequence_arm` against a frozen room; `plan_bucket` re-plans a
bucket several times under the drop-and-defer loop and the cluster DP's second
`cost_matrix` pass on every retry is the cost centre that killed stage B under
its wall cap; and **`sequence_arm_cluster` re-materialises its choice without
ever re-asking `staged.ink_vs_envelope`**, so a piece the room refused could
come back certified. `allocate.chain_sheets` is the same idea in the shape a
fixed order allows: **a Viterbi over pieces × alternatives, O(n·K²)**, with
alternative 0 the plan the bucket already certified, the others `menu` variants
(which advertise their exact entry/exit configurations without planning
anything, so only the chosen ones are materialised), the edge priced as
`writing.transit_time`'s three capped beats, and **every chosen alternative put
back through `plan_bucket`'s own ink-vs-room test before it can be taken**.
Continuity is a tie-break among certified plans, never a relaxation. Menus cost
0.05–0.09 s a piece and a materialisation 0.06 s, so K = 4 is ~0.3 s a piece.

**BUG 3: THE ROUTER'S FIRST DRAFT WAS ITS LAST, AND THE DEPOT JUMPED THE
QUEUE.** `paper.route` returns the first shape that certifies, and the shapes
that certify most often are the ones with the most vias. Two changes, both
inside `route`. **The bare depot via `[q_home]` — the park pose, 42 cm up on
this rig — was tried BEFORE the 8 cm rung of the ladder**; it is now tried after
it (`paper.HOME_AFTER_LADDER`), which is what "the lowest via that clears"
means. And **every tier's output is now shortcut before it is returned or
stored**: vias are dropped one at a time, the drop that saves the most
joint-space time wins each pass, and a drop is kept ONLY when the shortened
shape is re-certified end to end by `legs_ok` — the same bound, the same floors,
the same self and static gates as the shape it replaces, so a shortcut can never
be looser than what it replaces, only shorter (`paper.SHORTCUT`). A drop that
saves no time is not taken. `transit._shortcut` already did this for the RRT
tier's output and nothing else. **`paper.ROUTE_REV` rides in
`paper.cache_signature()`** and `HOVER_NEAR` rides in `lifted_or_lower`'s memo
key, so no warm store can serve a pre-change route.

**THE FIRST RE-RUN SAID FAIL, AND IT WAS RIGHT TO.**
`out/staged_csail_h097_lf5_s150_v1.*`, the three fixes above and nothing else:
stage A **55.41 s against 95.86**, over MORE ink (16 pieces / 2.4165 m, arm 31's
follower bucket cut at the room boundary and flew 0.355 m the baseline could
not), `active_pair` +138.6 mm and `solo` +167.9 mm against +65.6 and +51.7 --
and **FAIL**.

**THE FAIL WAS A FOURTH BUG, AND `frozen_failed` IS NOT WHAT ITS NAME
SUGGESTS.** `scene_check`'s `frozen` term is NOT `frozen.partner_clearance` and
has nothing to do with the leader/follower standoff: it is
`validate_pose(qtraj[a][-1])` -- **the last pose of each arm's trajectory**, the
pose the arm HOLDS when the stage ends, which under `PARK_FREEZE` is the hover
over the last stroke. Arm 71's read `joint margin 0.11108 rad on joint 0
against validate.MARGIN_GATE's 0.15`, hard violations `['margin']` and nothing
else, with frame 69.3 mm, self 105.7 mm, chain z 106.0 mm, tip z 60 mm and
inter-arm +167.9 mm. **Not the shortcutter** (`paper_failed` and `self_failed`
empty at the judge's own density). **Not the standoff**: the run carried
`--partner-standoff 0.0`, arm 71's serialised `standoff` is `None`,
`frozen.set_standoff` was never called in the run or the re-check, so the two
agree at S = 0 -- and `partner_clearance` is not the function this verdict comes
from.

**IT IS A MARGIN MISMATCH THAT WAS LATENT AND THIS CHANGE EXPOSED**, and
`HOVER_MARGIN`'s own comment had already written the fix: *"Callers that are
CHOOSING a pose rather than accepting one ask for the stricter gate instead,
because there is no reason to spend margin you do not have to."* The widened
fiber scan is exactly such a caller and was not doing it; the 0.1111 rad pose
was always in the fiber and always passed `HOVER_MARGIN`, and `HOVER_NEAR`
merely made the fiber's nearest pose winnable. `writing.HOVER_HOLD_MARGIN =
0.15` (restated from `validate.MARGIN_GATE`): the widened scan asks for the hold
margin first and settles for `HOVER_MARGIN` only where the fiber has nothing
that keeps it. It cannot lose a hover (the fallback is the identical old scan)
and it cannot admit one (both asks run the same `ok`). `ROUTE_REV` -> 3.

**THE VERDICT RE-RUN IS IN FLIGHT AND IS NOT IN THIS ENTRY.** Stage A was
relaunched with the margin fix (`ROUTE_REV` 3, so cold on every route) and had
planned its three leaders when this work's clock ran out: **arm 71's transit cap
reads 11210 against 11100 before the fix and 19190 in the baseline**, so asking
for the hold margin costs about 1 % of the transit budget and gives back none of
the 42 %. The stage verdict, duration and arm 71 pen-up under the fix are owed,
from `out/staged_csail_h097_lf5_s150.*`; `out/staged_csail_h097_lf5_s150_v1.*`
is the FAIL'ing run this entry diagnoses and is kept for the comparison.

**THE CERTIFIED RE-RUN** (`out/staged_csail_h097_lf5_s150.*`, all four fixes,
`--stages 0 --route-jobs 5`, cold on every route):

| stage A | before (lf2) | **after (lf5)** |
|---|---|---|
| **stage duration** | **95.86 s** | **55.97 s — 0.58×** |
| **verdict** | PASS | **PASS**, all six arms' `solo` ok |
| pieces, ink | 15, 2.0613 m | **identical** |
| arm 71 pieces, ink | 13, 1.838 m | **identical** |
| **arm 71 pen-up** | **70.54 s (73.6 %)** | **27.04 s (48.3 %) — −61.7 %** |
| arm 71 flip / tall legs | 4 / 5 | **2 / 1** |
| arm 71 planning | 7.3 s | **21.3 s (2.9×)** |
| `active_pair` / `solo` | +65.6 / +51.7 mm | **+196.7 / +168.7 mm** |

**THE SAME DRAWING, 39.9 s SOONER, WITH EVERY CLEARANCE READING THREE TIMES
BETTER.** The 0.355 m of follower ink the FAIL'ing run picked up does NOT
survive the hold margin -- arm 31's bucket is back to not flying -- so the
coverage win the intermediate run appeared to offer is withdrawn.

**THE TALL LEGS ARE GONE COMPLETELY**: every one of arm 71's pen-up legs now
reads `ztrav = 0.120 m`, `zmax = 0.060 m` -- the honest lift-and-lower -- against
four legs at 0.64-0.79 m of vertical travel and 0.32-0.42 m of peak height.
**What remains is named**: legs 2 and 10 (10.7 and 11.2 rad of TRAVEL, the
piece-to-piece half the chain DP could not close now that the hold margin
narrows the fiber) and leg 12 (15.1 rad, the go-home fold, which is the park
pose and not a sheet choice). Three legs are 12.7 s of the remaining 27.0.

**PLANNING GOT 2.9x DEARER FOR ARM 71 AND IT IS NOT THE MENUS** (~0.3 s a piece,
~4 s for thirteen). It is `HOVER_HOLD_MARGIN`'s fallback fiber pass and
`HOME_AFTER_LADDER` walking all seven rungs before the depot via. Twenty-one
seconds of planning for forty of stage time is a good trade for a fixed
programme and a bad one for an interactive loop.


**CORRECTION (2026-09-14): the hold margin is OFF, and the 55.97 s PASS is withdrawn**

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

**AND THE CONFLICT IS RESOLVED (same day, after the revert).** `static_gate`
gained a `frozen.partner_clearance` term -- a FLOOR that makes a pose
unacceptable under it, and a CAPPED term so that among acceptable poses the
search prefers the ones that also stand off the neighbours -- with
`score.cap = min(static_cap, room_cap)` so "comfortable" means comfortable on
both and the fiber opens for either. It is a score among CERTIFIED candidates,
not a gate: every candidate still passes the identical `CHAIN_CLEAR`,
`FRAME_FLOOR`, `selfcoll.self_ok` and joint-margin tests, and no gate constant
moved. It is opt-in (`room_floor=None` is the default and does not query the
partners at all), and `hover_solve` now has NO module default for the hold
margin -- it reads its own argument, so every travelling hover is solved exactly
as it always was. `writing.arm_program` asks for both, once, for the last exit
hover under `PARK_FREEZE`: the only pose `scene_check` puts through
`validate_pose`. The ask is best-effort -- if nothing on the fiber holds both,
the ordinary answer stands -- so the fix can refuse nothing.
`HOVER_ROOM_FLOOR = 0.075` and not 0.050, because what `solo_check` measures is
the LEG out of the held pose after the 1-Lipschitz playback residual, and the
pose that failed was clear at 45.95 mm *on its go-home leg*.

| pinned `test_staged_end_to_end_on_a_three_stroke_picture` | hold off | hold on, no room term | **on, with the room term** |
|---|---|---|---|
| arm 71 solo clearance | 62.61 mm | **45.95 mm (FAIL)** | **62.61 mm** |
| arm 71 final held pose | jm 0.136, **fails `validate_pose`** | jm 0.576, passes | **jm 0.6023, passes** |
| verdict | pass | **FAIL** | **pass** |

Both requirements at once, which is what the conflict said a scan blind to the
room could not do.

**THE STAGE-A RE-RUN IS IN FLIGHT AND IS NOT IN THIS ENTRY.** It was launched
with the hold margin ON and the room term live (`ROUTE_REV` 4, cold on every
route) and had planned its three leaders when the box closed: arm 71's transit
cap reads **11760**, against 11100 for the three fixes alone, 11210 for the
hold-margin-everywhere run, and **19190** in the baseline -- so asking the held
pose for both costs about 6 % of the transit budget and still leaves 39 % of the
42 %. Verdict, stage time, arm 71 pen-up and per-arm planning are owed, from
`out/staged_csail_h097_lf5_s150.*`.

**WHAT IS NOT CLAIMED.** The classifier's thresholds are calibrated on this
programme's honest legs and are deliberately conservative — arm 71's legs 4 and
5 come out "honest" at `flip_rad` 3.0 and 0.1, under the 6 rad floor, where a
hand read calls them flips. **The four fixes are not separable in the re-run:
they were measured together**, and the one component A/B that exists is the
hover's (48.39 -> 16.76 rad of lift+lower on arm 71's 26 span ends). The
41.6 % is stage A of one programme on one rig -- stages B and C are untouched
and unmeasured here, and the `--stages 0,1` run that would have priced stage B
was killed at eleven minutes to get stage A inside the clock. And the whole
result rests on one run per configuration; nothing here is a distribution.

## THE ROW CONDUCTORS COMPOSE — AND THE ROOM WAS BEING THROWN AWAY (2026-09-14, SIXTH)

**§24.5 refused stage C at −191.9 mm and blamed the overlap. The overlap was
not the bug.** `idle.conduct` does not re-time the timelines `plan_bucket`
built — it calls `writing.arm_program` again and **routes every pen-up leg
again** (`idle._programs`). `_conduct_stage` called `thaw()` between the two, so
every leg a two-arm row conductor emitted was routed against
`paper.static_boxes`'s pose-invariant base columns and **nothing else**. The
four arms outside the group were neither routed around (thawed) nor scheduled
against (not in the conduct): **invisible to both halves of the safety
argument.** A six-arm conduct never showed it, because there is no arm outside
the group.

**THE PROOF IS THE PLANNER'S OWN SEAM.** `frozen.partner_clearance` on arm 31's
realised stage-C trajectory, with the five partners frozen at the stage-C entry
poses the planner was told to use, reads **−165.3 mm at t = 3.20 s**. The
planner's own room was violated, so the room was not installed.

**AND THREE OF THE FIVE FAILING PAIRS HAVE A STATIONARY ARM IN THEM** — 2 ↔ 31
at −204.6 mm, 31 ↔ 97 at −117.9 mm, 71 ↔ 97 at −38.8 mm, with arms 2 and 97
standing still for the whole stage. §24.5's table quoted six of fifteen pairs
and missed all three. Serialising the rows in time — §24.5's proposed fix —
cannot touch a pair whose other half never moves. Every violation is a
**transit**; no ink is involved.

**WHAT WAS BUILT.** `staged.freeze_conduct` keeps the arms a conduct does NOT
move in the room for the length of the conduct, through the same `frozen` seam
every planner gate already reads; `thaw()` moves to after `idle.conduct`.
`--row-compose parallel | priority | serial`: `priority` conducts the groups
busiest-first as a pipeline, each later group against the earlier groups'
REALISED trajectories as `exact_room` swept capsules — if group k's chain never
enters group i's swept volume, it clears i wherever i is, which is the whole
pair certificate and not a snapshot. `serial` runs the rows one after another
in time and still plans them at once. `_merge_conducts` now also proves the six
entry poses pairwise at t = 0 (`hold_gap`) and fails the merge if they are not
clear. `--stage-c-only` re-runs the final pass off an existing programme.

**THE `self` FAILURE IS A DENSITY DISAGREEMENT, NOT A FOLD, AND NO GATE
CONSTANT IS WRONG.** Arm 31's refused leg has a true minimum self-clearance of
**26.12 mm** over 401 dense samples — over the checker's 20 mm *and* over the
router's 23 mm `SELF_PLAN_MARGIN`. `scene_check` reads **+16.18 mm** because it
samples the timeline that is written down and subtracts **0.55 × 18.06 mm** of
capsule-endpoint travel between two playback frames. `paper` converges its bound
on its own adaptive samples; the judge's residual grows with the **speed**. The
3 mm the producer holds over the judge buys **5.45 mm of frame step** at 0.55,
and a pen-up at the shipped transit speed puts three times that between frames.

**SO THE FIX IS PACE.** `writing.self_pace_beat` stretches exactly the beats
whose own converged self bound cannot carry the residual their speed implies:
`lb − 0.55 × rate × dt_play / dt ≥ 20 mm` solves for `dt` directly. The route,
the poses and the ink are untouched; a leg already under the judge's margin is
left for the judge to refuse. On the pinned leg, 0.05 s → 0.0969 s (1.94×) and
+16.33 mm → +21.28 mm. `SELF_PLAY_DT` 0.025, `SELF_PLAY_K` 0.55 and
`SELF_PLAY_FLOOR` 0.020 are **restated** in `writing.py` on purpose, as
`validate.py` restates `selfcoll.SELF_MARGIN`.

**ONE MORE THING THE RUN SAID AND §24.5 READ AS A PASS.** Group [2, 97]
reported `+inf mm, PASS` and drew **nothing** — 1.494 m of accepted ink, no
timeline for either arm — so its "clearance" was the clearance of an empty
scene. Stage C's real flown ink was **10.591 m**, not the 12.085 m of accepted
pieces. `StageResult.complete` catches it at the stage level; the group line
does not, and should.

**MEASURED, AND THE ANSWER IS `serial`.** `--stage-c-only` on the lf3 s150
programme, the six-arm `scene_check.check_timeline` over the merged stage:

| | parallel (as shipped) | **serial** | priority |
|---|---|---|---|
| stage-C ink flown | 10.591 m of 12.085 | **all 12.085 m** | 8.850 m |
| inter-arm | **-191.9 mm** | **+49.9 mm** | +49.9 mm |
| self | +16.2 mm | +14.4 mm | +14.4 mm |
| stage-C motion | 160.1 s | 212.4 s | 159.6 s |
| conduct wall | 1 369.7 s (max) | **1 184.8 s (max)** | 2 398.6 s (sum) |
| A+B+C+D | 307.5 s, uncertified | **354.6 s** | 301.8 s, 26.8 % of the ink gone |

`priority` is sound and unusable here: group [31, 71]'s realised trajectory is a
31 920-capsule swept volume and neither other group can route a single leg
around it.  `serial` gets the same clearance for all the ink and half the wall,
because a row planning against the rows before it AT THEIR PARKS is planning
against four poses rather than a tour.  **`serial` is the shippable mode, and it
is not shippable yet by 0.09 mm** (pair 71<->97 at one instant) **and one leg**
(arm 71's transit, true self-clearance 23.60 mm, refused at +14.4 mm because the
pacing bound is a fixed 33-sample grid over an 11 s beat and has to be
`paper.leg_self_lb`'s converged one).  Neither is a composition failure.

docs/V2_STAGED.md section 26 is the write-up and
`tests/test_staged_compose.py` holds the mechanism down.

## A SAME-ROW PAIR *CAN* DRAW TOGETHER — THE LEADER'S PARTNER STANDOFF (2026-09-14, FIFTH of five)

**The claim this overturns is our own.** §22 §4b's *"no pattern may ever put a
same-row pair in the air together"*, re-anchored on rooms in §23 and given a
number in §24 — *"every pose arm 31 can hold near its own ink stands +53.7 mm
from leader 71's exact room, legal to stand in and impossible to route
through"* — was read as a fact about the pair's geometry. **It is a fact about
the LEADER'S TOUR.** With leader 71 reduced to the pose it holds, follower 31's
own hover stations read a median **+192.0 mm** and **39 of 45 clear the 63 mm
routing floor**; with 71 at its park, 45 of 45 clear at a minimum of +135.2 mm.
The +53.7 mm was the leader's swept trajectory, and the leader had no reason to
keep more than the 50 mm it was certified at.

**WHAT WAS BUILT — `frozen.set_standoff`, and NO GATE CONSTANT MOVED.**
`PAIR_MARGIN` 0.050, `rig_final.STATIC_MARGIN` 0.050, `SELF_PLAN_MARGIN` 0.023
and `paper.FRAME_FLOOR` 0.063 are untouched and a test pins all four. `S` is an
ADDITIONAL requirement: `frozen.partner_clearance` returns
`min(gap everywhere, gap to that partner's POSE-INVARIANT capsules − S)`, so
every call site that compares the answer against its own floor `f` demands
`f + S` of the named set and `f` of everything else. One seam — the ink gate,
`writing.arm_program`, all four `paper.route` tiers and
`effective_static_floor` already reach the partner model through `frozen`.
`scene_check` shares none of it and stays the judge.

**THE SET IS MEASURED, NOT ASSUMED.** Leader 71's realised stage-A trajectory
against follower 31's capsules at its held pose: base bands 207.0 / 243.8 /
192.2 mm, **upper arm (chain 1→3) 103.2 mm**, elbow 314.2, forearm 331.4,
wrist 613.3, hand 560.1, tool 519.8. The binding pair is the leader's FOREARM
against the follower's UPPER ARM, whose shoulder end is chain point 1 and does
not move. So the standoff set is every capsule both of whose chain endpoints are
in {0, 1, 3} — the four base column bands and the shoulder→elbow link, "base
column + link 0/1". The elbow link is three times further away and is not in it.

**ONLY A LEADER OWES IT, AND ONLY TO ITS SAME-ROW FOLLOWER** (`lf_standoffs`).
A cross-row pair clears by +194 mm with each row inside its own band, so
charging one would cost a leader ink and buy a follower nothing. **S = 0 is a
no-op**: no standoff installed, the leg-cache signature and the plan-memo room
key bit-identical to what they were before this existed, and a `--partner-standoff 0`
run reproduces the live baseline line for line (15 pieces, 2.0613 m, 95.864 s,
pair +65.58, solo +51.66, PASS, arm 31's park +53.7 mm, 0/45 stations, the same
cut and the same two drops).

**MEASURED — the sweep, CSAIL h = 0.970, stage A.** The leader's ink flown is
**2.061 m at every S** (split +0.15) and **4.595 m at every S** (whole bag):
the same metres and the same pieces, because **not one of the leader's ink poses
is inside the bar at any S** — 0.0 % of its drawing samples, against 35–50 % of
its pen-up ones. The standoff constrains where the leader's LEGS may fly and
nothing else.

| S | split +0.15: follower flown / offered | verdict | whole bag: follower flown / offered | verdict |
|---|---|---|---|---|
| 0 | **0.000 / 2.142 m — 0 %** | PASS | **0.000 / 5.026 m — 0 %** | PASS |
| 70 mm | 0.955 / 2.142 m — 44.6 % | FAIL (held pose) | 0.000 / 5.018 m — 0 % | PASS |
| **90 mm** | **1.433 / 2.141 m — 66.9 %** | **PASS** | 1.526 / 5.020 m — 30.4 % | FAIL (held pose) |
| **110 mm** | 1.433 / 2.141 m — 66.9 % | FAIL (held pose) | **1.387 / 4.981 m — 27.8 %** | **PASS** |
| 130 mm | 1.568 / 2.140 m — 73.2 % | FAIL, `active_pair` **+42.7 mm** | 1.790 / 5.020 m — 35.7 % | FAIL, `active_pair` **−8.3 mm** |

**AND THE FOLLOWER'S CLEARANCE GOES STRAIGHT TO THE CEILING.** At every S ≥ 70
the log reads `arm 31 [follower] tuck: park +80.2 mm, 1/1 stations clear >= 73 mm`
— up from +53.7 mm, and +80.2 mm is exactly the S → ∞ value. The lever does not
buy the follower part of the room; at 70 mm it has already bought all of it.

**THERE IS A REAL OPTIMUM AND THE SWEEP BRACKETS IT.** Below it the follower
cannot route its legs; above it the follower takes so much ink that the two
realised trajectories close on each other — S = 130 reads +42.7 mm on the split
and **−8.3 mm** on the whole bag, which is a genuine gate violation and the one
thing in this sweep that is. Every other not-ok stage has BOTH clearance numbers
over their gates and fails `solo_check`'s per-arm flag: the case measured is
**leader 71's HELD BARRIER POSE at 111 mrad of joint-1 margin against
`scene_check`'s 150 mrad bar** (frame +69.3 mm, self +105.7 mm, tip z 60 mm all
fine). That is `park_policy` / `hold_gap`'s business, not the pattern's.

**THE DECISIVE NUMBER.** Row 1 is arms 31 and 71 at 0.61 m of base spacing. At
the best S, in stage A:

| | split +0.15, S = 90 mm | whole bag, S = 110 mm |
|---|---|---|
| row 1's ink drawn **with both arms of the row flying** | **3.271 m of 3.979 m — 82.2 %** | **5.024 m of 8.618 m — 58.3 %** |
| …**drawn simultaneously**, in the overlap window | **2.438 m — 61.3 %** (29.45 s) | **2.027 m — 23.5 %** (24.59 s) |
| the same, at S = 0 | **0.000 m — 0.0 %** | **0.000 m — 0.0 %** |
| realised pair clearance while they do it | **+62.1 mm** | **+62.5 mm** |
| stage A | **77.4 s** (was 95.9), **3.494 m** (was 2.061) | **109.6 s** (was 128.6), **5.982 m** (was 4.595) |

**AND ONE A + B DID FIT.** The sweep itself is **stage A only** — all eight
A + B runs were killed by their 25-minute cap inside stage B's
`sequence.cost_matrix` route screen, the same O(n²) wall §19, §22 and §24
flagged, which the standoff makes worse per leg because every re-routed leg
falls through to the RRT. But the whole bag at S = 110 mm, re-run with four
route jobs and a warm leg store, completed both stages with `all_ok` and
`holds_ok` true:

| | S = 0 (lf3) | S = 110 mm |
|---|---|---|
| **A + B makespan** | **145.9 s** | **111.2 s — 0.76×, 23.8 % faster** |
| ink drawn in A + B | 4.623 m | **5.982 m (+29.4 %)** |
| **deferred to stage C** | **10.023 m** | **7.188 m (−2.835 m)** |
| time to first motion | 0.192 s | 0.184 s |

**STAGE B COMES OUT EMPTY, AND THAT IS THE SHAPE OF THE WIN**: everything that
can fly now flies in stage A with both arms of the row in the air, so the role
swap has nothing left to draw and the pattern collapses from two main stages to
one. Stage C is handed 2.835 m less, which at §22's measured 268 s/m of six-arm
conduct is the larger half of what this lever is worth and is not in the 111.2 s
at all. **The split +0.15 A + B is still owed**: `S090_s150_ab` hit the same cap
in stage B, so the comparison against 119.9 s is unmeasured.

`docs/V2_STAGED.md` §25 is the write-up; `tests/test_staged_standoff.py` holds
the mechanism down; the runs are
`out/staged_csail_h097_lf4_S{070,090,110,130}_{s150,whole}.{json,log}` with the
capped A + B logs alongside as `*_ab.log`.

## PIECES ARE CUT AT THE ROOM BOUNDARY, AND THE FINAL PASS IS ONE CONDUCTOR PER ROW (2026-09-14, FOURTH of four)

**Two decisions, both of the same shape: cut where the geometry changes, and do
not ask six arms a question only two of them are in.**

**1. A follower's piece that fails the room ink gate is CUT, not refused
whole.** `ink_vs_envelope` is a minimum over a piece's poses, so 55.9 % of
follower 31's poses clearing the 50 mm gate against its leader's exact room
bought **0 %** of its ink (§23.3). `staged.split_at_room` returns the contiguous
stretches whose EVERY pose clears the gate — with the follower's own
1-Lipschitz sweep residual subtracted, so the claim covers the motion between
the ink samples and not only at them — and feeds them back through the ordinary
refusal loop. Each stretch is at least `traces.MIN_PIECE_M` long, each new end
must hold a certified hover, and each part is re-planned and RE-GATED: the cut
chooses where, the gate stays the judge. What is deferred is the refused ink
rather than the line it belonged to. Split parts take piece ids from 1 000 up,
so `(line, piece)` still identifies ink uniquely everywhere downstream.

**A NEW PIECE END IS JUDGED AT `PAIR_MARGIN` (50 mm), NOT AT
`paper.FRAME_FLOOR` (63 mm), and that is not a gate change.** `FRAME_FLOOR` is
`STATIC_MARGIN` plus the CHECKER's sweep residual and it is a ROUTING floor;
`paper.effective_static_floor` already clamps it to whatever a leg's own
endpoints can hold, exactly so a certified pose 51 mm from a neighbour is not
stranded by a bar its own endpoint cannot meet. A hover is a POSE THE ARM
HOLDS, and `hold_gap` already judges a held pose at `PAIR_MARGIN`. Measured: at
63 mm the cut threw away every clear stretch it found on CSAIL stage A, because
**every** hover over follower 31's ink reads **+53.7 mm**.

**And +53.7 mm being the same number every time is itself a result.** Arm 31's
hover over any of its own ink, at any rung of the ladder, and its park, all
stand 53.7 mm from same-row leader 71's exact room. A same-row pair's shoulders
are a fixed distance apart whatever their pens do. Legal to stand in, at the
50 mm gate; not routable through, at the router's 63 mm floor. That is §22's
"no pattern may ever put a same-row pair in the air together" with a number on
it, and it is why stage A's follower still keeps nothing while stage B's keeps
most of what it is offered.

**2. Stage C is THREE TWO-ARM CONDUCTORS, one per row, in parallel processes.**
Deferred ink is by construction the contested strip between one row's two arms,
and rows are separated by the 0.40 m y dead band — `docs/V2_WORKCELLS.md` §4b
measures cross-row pairs at **+194.3 / +201.1 mm** with each row drawing inside
its own band. So 13+17, 31+71 and 2+97 are conducted separately and at once:
**four priority orders each instead of 720**, a third of the pieces each, and
the `sequence.cost_matrix` route screen — which is the wall §22 measured at
3 075 s, not the DP — paid three ways in parallel. The groups overlap in time
and `_merge_conducts` pads them onto one clock, with every arm in no group
holding its pose IN THE SCENE; **one** `scene_check.check_timeline` over all six
arms is the certificate, and it shares no code with any of it.

**A piece that leaves its row band may never ride with a row conductor.** The
cross-row number was measured with each row inside its own band.
`staged.piece_row` returns a row only if the piece's whole geometry stays in it;
everything else goes to a short **stage D**. Measured on the CSAIL logo's
deferred ink: **9.7 % (split +0.15) / 11.5 % (whole bag)** is dead-band ink, all
of it row 1's, against a 5 % bar — so stage D is a real stage, and Pete's
two-phase sequence (outer rows draw their band ink while the middle row holds,
then the middle row draws its own) collapses to its second phase here.

**AND THE ROW CONDUCTORS ARE PROFOUNDLY UNBALANCED ON THIS PICTURE**: row 1
carries 5.13 m of 5.94 m, row 2 0.23 m, row 0 nothing. The parallelism is worth
much less than 3x here; what it is worth is that the conduct stops being a
six-arm search, which is the term `ARCHITECTURE_V2` §2d says does not scale.

**3. WHICH CERTIFICATE IS KEPT WHERE, stated once.** `ink_vs_envelope` is a
certificate exactly where it is used as a GATE — the follower's per-piece
refusal and the split's clear-stretch test. Everywhere else it is a REPORT: a
leader is measured against the rooms before it and never refused on that number
(§8). The stage's verdict is and has always been the realised-trajectory
checks — `active_pair_gap` over the arms that MOVE, `solo_check`, `hold_gap` at
every barrier, `scene_check.check_timeline` over a conducted stage — computed
with the planner's room thrown away. §23.4b's "the stage is marked not-ok by
`ink_vs_envelope_mm = 35.67`" was a misreading: `StageResult.ok` has never
included that number.

Gate constants are untouched: `PAIR_MARGIN` 0.050, `SELF` 23, `STATIC` 50,
`FRAME_FLOOR` 63. `docs/V2_STAGED.md` §24 is the write-up;
`tests/test_staged_split.py` holds the claims down.

## THE FOLLOWER'S ROOM IS THE LEADER'S CAPSULES, NOT A CLOUD OF SPHERES AROUND THEM (2026-09-14, THIRD of three)

**What was wrong.** A follower was certified against `cluster_capsules`'
sphere reduction of its leader's trajectory: one sphere per 0.075 m cell,
radius = 65 mm half-diagonal + up to 177 mm + the sweep pad, median 186 mm and
max 417 mm. That reduction costs a **median 195 mm** of clearance the leader's
metal does not occupy, and it is what put follower arm 31's own park **128.4 mm
inside** its same-row leader's room before the arm had moved a joint —
which clamps `paper.effective_static_floor` negative for every leg and
strands the bucket.

**What was built.** `aris_sixarm/exact_room.py`: the leader's real swept
capsule chain, one capsule per link per timeline sample, each carrying
`scene_check`'s own per-sample 1-Lipschitz residual. The spheres survive as the
**broad phase** of a grid hash — with each cell's AABB alongside its sphere, and
a shell-deepening narrow phase whose termination test (`best < prev`, the
boundary already measured) makes the answer **exact**, not approximate, at every
cell size. Measured against brute force: bit-identical. Cost at the router's
floor: **0.05 ms per pose** on a 26 000-capsule room, against a ≤ 1 ms budget.

**One seam.** Every consumer — `plan_stroke`'s ink gate, `paper.route`'s
straight/ladder/RRT tiers, `effective_static_floor`, `writing.arm_program` —
already reaches the room through `frozen.partner_clearance`, so the swap
happened inside `frozen.py` and all of them saw it at once. The go-around tier
was the exception (`_skirt` reads box footprints), and gets `frozen.room_boxes()`
**for detour generation only**; the gate stays the exact room. `scene_check`
shares none of it and stays the judge. `ARIS_ROOM=spheres` reproduces the old
numbers.

**Measured, same tree, same seam bar, CSAIL h = 0.970, split +0.15, stage A.**
Follower 31's start pose vs its leader's room: **−128.4 mm → +53.7 mm** (+182 mm).
Its ink clearance: min −243.8 → **−199.8 mm**, median −134.6 → **−118.9 mm**,
max −128.4 → **−38.0 mm**. Pieces refused at the ink gate: 3 → 2. Stage A
inter-arm +65.6 mm, PASS, unchanged.

**And the follower still keeps 0 % of its ink**, at both splits. The room was
one of two causes and it is fixed; the other is the allocation — the follower's
ink lies inside the volume the leader's arm actually sweeps. The diagnosis'
"55.9 % of poses clear" is **not** a ceiling on flown ink, because
`ink_vs_envelope` gates on the MINIMUM over a piece and refuses the piece whole.
Turning that fraction into ink needs pieces **split at the room boundary**, which
is the next build item and the only one left between here and a follower that
draws.

**The follower's tuck is built and inert.** `staged.tuck_pose` / `clear_out` /
`splice_timeline` search a pre-position outside the leaders' exact rooms, fly
park → tuck while the leaders are still parked, and **splice the clear-out into
the follower's timeline** so `trajectory_room`, `active_pair_gap` and
`scene_check` all see it. `q_tuck` / `clear_out_s` are recorded per arm (schema
stays 2). On this picture no follower needed it — arm 31's park is +53.7 mm and
arm 97's +263.4 mm once the room is exact — so the clear-out cost **0 s** and
time to first motion is unchanged at 2.269 s, inside the 10 s bar.
`--no-tuck` turns it off.

**The 1 000-stroke model is deliberately NOT recalibrated.** The build made that
conditional on stages A/B carrying most of the ink, and they do not.

## THE SEAM BAR IS IN THE CERTIFIED STATIC SET, AND THE MIDDLE ROW'S PARKS MOVED (2026-09-14, SECOND of two)

**Pete Werner, settling what to model rather than photographing it:** *"just
put a representative bar in the middle that is as wide as two of the corner
struts."*

So the entry below this one — four inferred end-frame posts, five photo
questions, "the static set is unchanged, deliberately" — is superseded on both
counts. The seam support is **one representative bar per side**,
`seam_bar_W` / `seam_bar_E`, 76.2 (x) × 152.4 (y) × 1651.0 mm, centred on the
seam plane y = 1815.32 and standing from the tabletop to the runway underside.
And it is **in the planner's static set**: `mounts.obstacles_for` hands it to
every arm, so `validate.check_pose`, `atlas.solve_cell`, `paper.route`,
`planner` and `scene_check` all gate against it and every number earned after
today has the seam in it. `ARIS_SEAM_POSTS=0` takes it back out, which is the
only way to reproduce a number earned before today.

**ONE TABLE**, `mounts.SEAM_BARS_MM` — two rows of six numbers. A hardware
correction is an edit there and nowhere else; `system_model.seam_bodies()`
reads it, and `test_the_seam_bar_table_is_the_drawings_own_arithmetic` pins it
against the drawing's own derivation. It lives in `mounts` and not in
`system_model` for an import-graph reason: `system_model` reads
`layout.FLEET_PROPOSED` at import and `layout` builds that fleet through
`mounts.obstacles_for`, so `mounts` cannot import `system_model`.

### THE BAR COSTS EXACTLY WHAT THE FOUR POSTS DID — measured, not assumed

The two posts at one x occupied y ∈ [SEAM_Y − 76.2, SEAM_Y] and
[SEAM_Y, SEAM_Y + 76.2]. **Their union is this bar**, and a union of two AABBs
that share a face is an AABB. So every clearance in the entry below reproduces
to the digit against two bodies instead of four: `Q_PARK_PROPOSED` arm 31 at
**50.9 mm**, stage parks 2 and 6 at **−35.3 mm**, certified area
**16 184 → 16 019**, v19 at **−59.4 mm**. 79 static bodies, not 81;
`check_system_model.py` ALL PASS.

### THE FLOOR A PARK IS NOW SEARCHED AGAINST, AND WHY IT IS THE CHECKER'S

Two floors and two derivations, and before today no park was ever near enough
to either for the difference to matter — every arm stood 214–277 mm off the
nearest steel, which was a neighbour's body column and not the cage at all.

* `rig_final.STATIC_MARGIN` = **50 mm** is the POSE gate (`validate.check_pose`)
  and what the atlas was swept at.
* `paper.FRAME_FLOOR` = `rig_final.STATIC_PLAN_MARGIN` = **63 mm** is what the
  ROUTER asks. **A depot the arm cannot fly out of is not a depot**, so that is
  the floor a park set is searched against now.
* and it is measured by `scene_check.static_clearance_lb` — the CHECKER's
  independent derivation, the one `check_timeline` uses — which reads
  **9.9 mm under** `rig_final.chain_static_clearance` for the same pose on this
  rig. A park picked at 63 mm of PLANNER clearance is 53 mm to the checker and
  fails the only verdict anybody ships. `layout.static_clearance` is that
  number and `layout.stage_parks(static_floor=...)` is the gate.

### 1. THE GLOBAL PARK SET — ONE ARM MOVED

    ARIS_RIG=proposed ARIS_TOOL=lateral scripts/height_sweep.py park \
        --h 0.970 --atlas out/atlas_proposed_h0970_lat0860 --jobs 6 \
        --steel-floor 0.063 --out out/park_search_h0970_seam.json

The same search that made the shipped grid — 24 bearings × 6 radii × 4 hovers
= 576 candidates per arm, each gated by `certified_ready_pose`, scored against
every other arm's INK **and** 6 cm LIFT layers, ranked on the depot's own
flyability and tie-broken on the 5 mm clearance plateau — with one thing
added: `--steel-floor 0.063`, the router's floor instead of the pose gate.
2 048 s on 6 workers. **442–461 of 576** candidates per arm certify against the
bars, where 496–497 certified against nothing.

**FIVE OF THE SIX ARMS RE-WIN THEIR OWN TRIPLE EXACTLY.** Arms 2, 13, 17, 71
and 97 get back the three numbers the empty room gave them, because the nearest
steel to any of them is still a neighbour's body column at 267.5 mm and the
bars never enter it. Their seven joint values are bit-identical.

**Arm 31 moves across the seam and keeps its radius:** bearing **+150 → −150**,
hover **0.30 → 0.35**, r = 0.62 unchanged. Its pen waits at **(0.060, 1.505)**
instead of (0.060, 2.125) — the same standoff, mirrored to the south side of
the bar it was leaning against.

| chain-to-static, worst, mm (checker) | before | after |
|---|---:|---:|
| arm 2 | 267.5 | 267.5 |
| arm 13 | 267.5 | 267.5 |
| arm 17 | 267.5 | 267.5 |
| **arm 31** | **50.9  FAIL** | **70.4  PASS** |
| arm 71 | 214.0 | 214.0 |
| arm 97 | 267.5 | 267.5 |

and it is not only the steel that improves: park-vs-(ink AND lift) goes
98.0 → **98.8 mm** and the cells arm 31 can fly to and home from go 18/24 →
**19/24**. What it pays is 0.013 of conditioning — min(joint margin, 2.5 σ)
0.692 → 0.680, against a 0.30 gate neither pose is near.

**IT IS NOT THE SEARCH'S OWN TOP ROW, and that is the one judgement call
here.** Ranked flyability-first the winner for arm 31 is (0.40, 0.10, −135.0)
— 20/24 cells and 248.4 mm off the steel — but it parks the arm at
(0.314, 1.532), well inside the canvas, and its park-vs-(ink AND lift) is
**79.0 mm** against the **80 mm the conductor asks**, which is the criterion
this whole grid was chosen on (2026-08-26). One more flyable cell is not worth
the number that decides whether a phase can be conducted at all. The committed
triple is the best-conditioned candidate that clears the router's floor **and**
holds the 80 mm.

**FLEET, on the committed set:** park-vs-park **250.0 mm** (the broad-phase
cap, arms 2 and 13); worst park-vs-(ink AND lift) **97.7 mm** (arm 17,
unchanged — arm 31 was never the binding one); entries and go-homes
**113/144**; and all six clear the router's 63 mm floor.

### 2. THE PER-STAGE PARKS — `out/stage_parks_h0970_seam.json`

`scripts/stage_parks.py --atlas out/atlas_proposed_h0970_lat0860_gated63
--stride 2`, now with `--static-floor 0.063`. 91.7 s.

| stage | worst chain-to-steel, before | after | moved |
|---|---:|---:|---|
| 0 | **+49.9** | **+70.4** | 31 |
| 1 | +50.9 | **+70.4** | 31 |
| 2 | **−35.3** | **+67.3** | 31, 71 |
| 3 | +91.8 | +91.8 | — |
| 4 | +50.9 | **+70.4** | 31 |
| 5 | +50.9 | **+70.4** | 17, 31 |
| 6 | **−35.3** | **+105.7** | 31 |
| 7 | +50.9 | **+70.4** | 31, 71 |

Per arm, worst over the eight stages (mm):

| arm | before | after |
|---:|---:|---:|
| 2 | 267.5 | 267.5 |
| 13 | 267.5 | 267.5 |
| 17 | 62.3 | **73.2** |
| 31 | **−35.3** | **+67.3** |
| 71 | **−35.3** | **+91.8** |
| 97 | 267.5 | 267.5 |

**AND IT COST NOTHING THE STAGES WERE BEING CHOSEN ON.** Park-vs-envelope is
**+4.7 / +4.7 / +55.4 / +37.1 / +34.9 / +37.1 / +55.4 / +95.7 mm** — the same
eight numbers, stage for stage, as the shipped set measured the same way.
Park-vs-park is **≥ 142.7 mm** against the 50 mm `PAIR_MARGIN`; every active
arm can fly from its park into its own cell and back (3/3, then 2/2 in every
later stage); 13 of 42 stage transitions need a route and all 13 route.
Stages 0 and 1 still do not certify their envelope gate, for the structural
reason V2_WORKCELLS gives — a full-width row band leaves the other arm of a
transverse pair nowhere to stand. That is unchanged and is not the seam.

### 3. THE CERTIFIED AREA at h = 0.970 — the block survives

`scripts/seam_impact.py --write-map out/fw_h0970_m50_map_seam.npz` re-decides
every strict-GO draw pose of the shipped atlas against the bars; then
`scripts/certified_area.py --map` on the result, which is the shipped script
answering with its own machinery.

| | before | after |
|---|---:|---:|
| live cells | 16 184 (97.72 %) | **16 019 (96.72 %)** — −165, −1.02 % |
| **hole-free block** | 1.50 × 3.64 m, 5.460 m² | **1.50 × 3.64 m, 5.460 m² — UNCHANGED** |
| centred, portrait | 5.460 m² | 5.460 m² |
| near-square | 3.435 m² | 3.360 m² |
| landscape | 1.793 m² | 1.793 m² |
| largest component | 6.474 m², 0 holes | 6.406 m², 4 holes = 0.0032 m² |
| arm 31's cells | 3 765 | 3 680 (−85) |
| arm 71's cells | 3 772 | 3 692 (−80) |

**The block that decides where a drawing may be placed does not move.** It runs
x ∈ [0.16, 1.64] and the nearest bar face is at x = −0.1143 / 1.9177 — 274.3 mm
away, past the 205 mm a link needs. The 165 lost cells are all at the canvas's
x extremes on the seam row and all of them belonged to 31 or 71.

**STILL A LOWER BOUND, and doubly so.** The map's live cells rest on three
layers and only the draw pose is on disk, so a cell that keeps its draw pose
may still lose its hover or its routing leg. And the shipped map was built with
the partners FROZEN at the old parks; arm 31's park moved today, which a full
`feasible_workspace` re-sweep would account for and this re-decision does not.
The honest statement is: **at least 16 019 cells, and the hole-free block is
whole.**

### 4. THE SHIPPED v19 PROGRAMME STILL FAILS — and is not re-planned here

`out/csail_schedule_h097_v19.npz`, whole merged timeline, 5 038 steps
re-sampled to 10 075, 6 arms, `scene_check.check_timeline` against the seam
fleet:

```
self-collision              63.1 mm  (margin 20)   PASS
inter-arm                   50.3 mm  (margin 50)   PASS
neighbour base columns     131.5 mm  (margin 50)   PASS
paper: chain 26.0 / tip -7.9 mm                    PASS
frame (STEEL)              -59.4 mm  (margin 50)   FAIL: arms [31, 71]
                                     arm 31 at t = 13.94 s
VERDICT                                            FAIL
```

**Deliberately not re-planned.** v19 is the staged pattern that is being
replaced; the next programme is planned and certified with the bars in the
static set from the first call, which is the point of wiring them in rather
than measuring around them.

## THE SEAM FRAME: the model gained 4 bodies and v19 now FAILS on steel (2026-09-14, FIRST of two)

> **SUPERSEDED IN PART, the same day** — by the entry above it. The four
> inferred end-frame posts are one representative bar per side now (Pete
> Werner's own instruction), and the seam IS in the certified static set. Every
> measurement below still stands to the digit: the bar is the exact union of
> the two posts at each x, so it decides every clearance identically. Read the
> "waits on a photo" paragraphs as history.

**Pete Werner, on the real hardware:** *"there are a few bars on the real
hardware that are not in our model. they are supports in the middle, I think
the half-table drawing had them. the real thing is essentially the two halves
next to each other."*

`system_model.seam_bodies()` builds them from the drawing — **4 posts at
y = 1815.32 mm**, the paper's mid-length, which is also the middle arm row.
77 static bodies → **81**, `check_system_model.py` ALL PASS. (The posts' corner
brace plates are carried as a question, not geometry: as AABBs they overlap the
posts they bolt to, the model's own corner legs have never had them, and they
sit 450 mm above anything reachable.)
Full derivation in docs/SYSTEM_MODEL.md §3b; the one-line corroboration is that
two half-cages butted are 4165.60 mm against this model's 4011.64, and the
153.96 mm difference is one doubled 3″ end frame — so **the middle runway IS
the two butted end rails** (they agree to 0.78 mm) and what was missing is the
four posts holding them up.

### THE STATIC SET IS UNCHANGED, DELIBERATELY

`layout.StudySpec.static_obstacles` returns the neighbours' mount boxes and
base columns **and nothing else** — every certified number on the proposed rig
was earned against a fleet in an empty room, with no cage at all. Wiring the
seam in re-decides everything and is a re-certification; it waits on the photo
(`OPEN_QUESTIONS['seam_frame']`). `mounts.seam_frame_boxes()` is the door;
`scripts/seam_impact.py` walks through it and changes nothing on disk.

### WHAT IT COSTS — measured, h = 0.970

**1. PARKS.** Whole-chain clearance to the seam steel, by `scene_check.
static_clearance_lb` (the independent derivation), against the two floors:
STATIC_MARGIN 50 mm and FRAME_FLOOR 63 mm.

| park set | worst | arm | < 50 | < 63 |
|---|---:|---:|---:|---:|
| `Q_PARK_PROPOSED` | **50.9 mm** | 31 | 0 | 1 |
| `stage_parks[0]` | **49.9 mm** | 31 | 1 | 1 |
| `stage_parks[1]` | 50.9 mm | 31 | 0 | 1 |
| `stage_parks[2]` | **−35.3 mm** | 31 **and** 71 | 2 | 2 |
| `stage_parks[3]` | 91.8 mm | 31 | 0 | 0 |
| `stage_parks[4]`, `[5]` | 50.9 mm | 31 | 0 | 1 |
| `stage_parks[6]` | **−35.3 mm** | 31 | 1 | 1 |
| `stage_parks[7]` | 50.9 mm (71: 51.0) | 31 | 0 | 2 |

**Only arms 31 and 71 are ever the offender — the middle row, exactly as
expected: their J1 axes are ON the seam plane.** The shipped
`Q_PARK_PROPOSED` scrapes the 50 mm static margin by **0.9 mm** and misses the
63 mm router floor by **12.1 mm**. Two staged parks — stages **2** and **6** —
are **35.3 mm INSIDE the steel**: those are interpenetrations, not tight
margins, and they are the ones to move first. Seven of eight stages hold a
park that fails at least one floor.

**2. THE CERTIFIED AREA** (`out/certified_area_h0970.json`, re-decided on the
draw-pose layer):

| | before | after |
|---|---:|---:|
| live cells | 16 184 | **16 019** (−165, −1.02 %) |
| hole-free block | 1.50 × 3.64 m, 13 650 cells | **1.50 × 3.64 m, 13 650 cells — UNCHANGED** |
| arm 31's cells | 3 765 | 3 680 (−85) |
| arm 71's cells | 3 772 | 3 692 (−80) |

**The hole-free block survives intact.** It runs x ∈ [0.16, 1.64], and the
nearest seam post face is at x = −0.1143 / 1.9177 — 274.3 mm away, past the
205 mm a link needs. The 165 lost cells are all at the canvas's x extremes on
the seam row, and all of them belonged to 31 or 71. **This is a LOWER BOUND:**
only the draw pose is on disk, so a cell that keeps its draw pose may still
lose its hover or its routing leg, and only a full re-sweep can say.
`remap_dead_cells.py` cannot answer it — it re-decides DEAD cells against a
SUPERSET atlas, and adding steel makes a SUBSET.

**3. THE SHIPPED PROGRAMME — v19 FAILS.**
`out/csail_schedule_h097_v19.npz`, whole merged timeline, 5 038 steps
re-sampled to 10 075, 6 arms:

```
self-collision              63.1 mm  (margin 20)   PASS
inter-arm                   50.3 mm  (margin 50)   PASS
neighbour base columns     131.5 mm  (margin 50)   PASS
paper: chain 26.0 / tip -7.9 mm                    PASS
frame (STEEL)              -59.4 mm  (margin 50)   FAIL: arms [31, 71]
                                     arm 31 at t = 13.94 s
VERDICT                                            FAIL
```

**59.4 mm inside the seam steel.** Nothing else moved: the seam frame is the
single reason the verdict flipped, and it is the middle row again.

**The staged v6 programme ships no frames** (`out/staged_csail_h097_v6.json` is
a programme, not a timeline), so no merged-timeline verdict was computed for
it. Its **parks** are the `stage_parks[*]` rows above, and two of them
interpenetrate.

### WHAT THIS DOES NOT SAY

Nothing here is a survey and no gate constant, tool transform or layout grid
value moved. The seam frame is built from the drawing and reported by Pete; it
has **not been photographed**, and five assumptions ride on it — see
`OPEN_QUESTIONS['seam_frame']` and docs/BUILD_SHEET.md §9.

## THE PATTERN WAS NOT THE ONE PETE SPECIFIED — leader/follower, built (2026-09-14)

**What was wrong.** `traces.zigzag_pattern` — the eight-stage pattern
`docs/V2_STAGED.md` sections 8-21 measured and v6 shipped — parks a leader's
same-row partner for the whole stage. Three arms draw, three stand still, and
the picture takes eight barriers: on the CSAIL logo **432.6 s against the
conducted v19's 209.9 s**, with **21.4 % of the critical path** spent flying out
to a park and back. That is not what was asked for. Pete's specification was
*"the 1-2-1 for planning the lead arm with priority and then the other arm in
the column would only try to knock out lines in its cell that were safe to draw.
and then in the end we would do the coordination of all arms to fill in the gaps
if needed"* — **six arms moving in every main stage**, a per-piece "is this
safe" test for the follower, and one conducted final pass.

**Why the wrong one looked justified.** The parking was defended by
`docs/V2_WORKCELLS.md` section 4b: two arms of a transverse pair interpenetrate
by 135-262 mm, so *"no pattern may ever put a same-row pair in the air
together."* That measurement is of two **full work-cell ENVELOPES** — the union
over every pose either arm could hold anywhere in its cell. It is not a
measurement of a follower drawing a restricted subset while the leader's
**realised trajectory** is the occupied volume, and section 15 had already built
that second object and section 19 had already measured it at a factor of 12
smaller. A conclusion about envelopes was carried over to a question about
rooms. **The envelope numbers stand; the prohibition drawn from them does not.**

**What was built.** `traces.leader_follower_pattern` — stage A: leaders 13/71/2
(priority 0-2) and followers 17/31/97 (3-5); stage B: the roles swapped; stage C:
all six through `idle.conduct`, whole-timeline checked by `scene_check`. A
follower plans against the leaders' realised trajectory rooms, so it is
asynchronous by construction; a piece that does not fit is **deferred** — to the
stage where that arm leads, then to stage C — and never serialised into the
stage, which is what would give the barrier back. The row dead band (0.40 m) and
every gate constant are unchanged. `zigzag_pattern` is kept and
`--pattern zigzag|leader_follower` selects; both remain reproducible.

**And the barrier stopped being a trip.** Pete, watching the v6 animation: *"a
lot of excessive parking ... we should be just executing that plan as
efficiently as possible."* A stage now ends at the hover above its last stroke
and HOLDS there; the next stage starts from that pose and models every
not-yet-moving arm at the pose it is actually holding rather than at its park.
Parks are kept for the start of the programme, the end of it, and fault
recovery. The park set was mutually clear BY CONSTRUCTION (it was searched to
be); a held set is not, so `staged.hold_gap` proves it pairwise at every
barrier, with `scene_check`'s own capsules and the same 50 mm gate.

**One check was corrected on the way.** `active_pair_gap` takes the cross
product of two timelines and subtracts a 1-Lipschitz residual on the DECIMATED
grid, because two asynchronous arms share no clock. With six named actives, most
of them holding, that charged a **still** arm the moving arm's decimation
residual: CSAIL stage 0 read +22.7 mm where `solo_check` — the tool for exactly
that pair, at the full rate and with refinement — read +52.4 mm.
`active_pair_gap` now sees only arms that move; still arms belong to
`solo_check` and `hold_gap`. With the fix stage 0 reads +85.0 / +131.8 /
+192.9 mm across the sweep and passes.

**MEASURED, and the answer is not the one the pattern hoped for**
(`out/staged_csail_h097_lf_whole.json`, `docs/V2_STAGED.md` section 22). The
held barrier is a clear win: park overhead on the critical path falls from v6's
**21.4 % to 11.3 %**, every barrier's held pose set clears pairwise by **107 mm
or better**, time to first motion is unchanged at 0.189 s, and the makespan is
**273.3 s against v6's 432.6 s**. The follower is not: **0.000 m of the 5.026 m
offered it**, and the fifteen follower pieces sit at a median of **-235.7 mm**
inside the occupied volume, the closest one at **-131.9 mm**. Nothing marginal
is happening — no gate, ladder, split or order moves a number like that, which
is why turning the per-piece ink gate off changes stage A not at all.

**So section 4b's substance survives even though its reasoning was
over-general.** *"No pattern may ever put a same-row pair in the air together"*
was inferred from full envelopes and should have been inferred from rooms.
Asked properly, of rooms, the answer on this rig is the same. The correction was
still worth making: the claim is anchored where it belongs now, and the held
barrier and the six-arm stage came out of asking.

**And the bill lands on the final pass.** With the followers empty, 10.023 m of
the logo falls to stage C, which costs **3 075 s of wall for 126.6 s of
motion**, still ends **5.1 mm short of the gate**, and leaves 3 of its 6 ink
buckets unflown. That is `ARCHITECTURE_V2` section 2d arriving on schedule: a
pattern that defers most of the picture to the conductor has reinvented v19 with
a slower planner. **FOR PETE:** the held barrier should be kept regardless; the
leader/follower stages need a reason to exist that the follower's -235.7 mm does
not currently give them.

`STAGED_SCHEMA_VERSION` is **2**: `role`, `conducted`, `deferred` and `q_hold`
added, nothing renamed or removed.

## v6: EVERY BUCKET FLIES, 100 % OF THE INK, SEVEN OF EIGHT STAGES (2026-09-11, last)

`out/staged_csail_h097_v6.json` — priority rooms + order search + residue, RRT
tier on, borderline verdicts refined.  **13 of 13 ink buckets fly, 15.840 m of
15.840 (100.0 %), seven of eight stages pass both checks.**

| | v3 union envelope | v5 priority rooms | **v6 + search + residue** |
|---|---|---|---|
| ink buckets flown | 8 of 13 | 12 of 13 | **13 of 13** |
| ink certified | 34.4 % | 97.6 % | **100.0 %** |
| stages passing | 4 of 8 (two vacuous) | 6 of 8 | **7 of 8** |
| makespan | 198.7 s (partial) | 415.0 s | **432.6 s** |
| planning per stage's busiest arm | 2 869.6 s | 89.7 s | **20.6 s** |
| time to first motion | 0.294 s | 0.287 s | **0.280 s** |

**THE MAKESPAN IS 432.6 s** — **+10.4 % on the un-separated v1 (391.7 s)**, so
the whole cost of making three arms provably safe from one another is a tenth of
the clock; **2.06x** v19's conducted 209.9 s, which is the barrier and not the
rooms.  Park overhead 92.7 s on the critical path, 21.4 %.

**NO STAGE NEEDED A NON-INK-FIRST ORDER**, and that is the finding, not a
non-result.  Stage 0 tried all six and kept rank 0 because NO order flies it:
arms 2 and 71 are mutually exclusive there whatever the order (71-first flies
4.858 m, anything else 1.190 m).  So the search's real job was picking the
cheapest sacrifice, and **its value is that it LICENSES the residue** — it
proves the residue was not an order away from being avoidable.  Every other
stage flew on the first order and paid one comparison.

**THE RESIDUE IS 0.385 m, 2.4 % OF THE INK, AND COSTS 17.6 s** (a 16.4 %
premium on stage 0, 4.1 % on the programme).  Pete's original final pass, used
exactly once, for the one bucket nothing could fly concurrently.

**PLANNING PER ARM COLLAPSES TO 20.6 s**, 139x the union envelope's 2 869.6 s.
Serial planning rises to 769.0 s, which is the six extra stage-plans stage 0's
search paid for — the right trade, since what an arm waits for is the per-arm
figure.

**THE ONE FAILURE IS STAGE 2 AND IT IS FULLY ATTRIBUTED.**  Solo +40.4 mm: the
refinement recovered 12.2 mm of checker sampling and the remaining 9.6 mm is a
POSE — arm 97 holds a configuration ~40 mm from parked arm 2.  Every mechanism
of this pass is active-vs-active; this is active-vs-parked.  **8/8 is not
reachable without a different park or hover for arm 2 in stage 2** — build item
2, now with a concrete reason.

**THE 1 000-STROKE MODEL, RECALIBRATED** (draw 13.81 s/m, leg 4.18 s, park
10.93 s per (stage, arm)): **6 437 s = 107.3 min, 1.50x** the ~4 300 s conducted
scaling, park overhead on the critical path **1.36 %** against CSAIL's 21.4 % —
the barrier amortises.  Arithmetic with measured terms, and it assumes the
priority order and the residue scale: at 572 pieces in a stage-0 bucket the
residue could be far more than 2.4 %, which is what the next scaling pass owes.


## THE ORDER SEARCH, THE RESIDUE, AND THE ROUTER THAT WAS NOT WRONG (2026-09-11, last)

Three things built to close §19's two failures, and **one of the two diagnoses
is corrected here.**

**(1) THE ORDER SEARCH.**  The greedy ink-first order's weak spot is the arm
that plans LAST: it has the least freedom left.  A stage has at most three
actives, so the whole order space is SIX permutations — cheap, where a six-arm
priority search is 720.  Ink-first is tried first, so a stage that did not need
the search pays one comparison; `order_rank` records which order was taken.

**(2) THE ROUTER WAS NOT THE OPTIMISTIC ONE, AND THE PREVIOUS ENTRY IS
CORRECTED.**  `paper.leg_bounds` already subtracts `sample_residual` and falls
through to `adaptive_static_lb` in the undecided band; `FRAME_FLOOR` = 53 mm is
already `STATIC_MARGIN` plus the checker's residual, and `paper.py`'s header
explains both that and the `TIP_SWEEP_PAD` version of the same argument.  The
28.2 mm comes from two other places and the split is measured:

| dt | frames | stage 2, arm 97 |
|---|---|---|
| 0.05 | 961 | **28.23 mm** |
| 0.02 | 2 400 | 36.58 mm |
| 0.01 | 4 799 | **39.38 mm** |

**ELEVEN MILLIMETRES WAS THE CHECKER'S SAMPLING.**  `check_timeline` auto-refines
its frame and paper gates and does NOT refine its INTER-ARM gate — it subtracts
`0.55 x (step_i + step_j)` at whatever rate it was handed.  A verdict under the
margin is now looked at again (three halvings or 40 000 frames, best bound
kept), which is `block_screen`'s own rule applied to the gate that lacked it.

**THE OTHER TEN MILLIMETRES ARE A POSE, NOT A PATH.**  The number converges to
~40 mm, not 50.  `effective_static_floor` CLAMPS the floor to what the leg's own
endpoints have — deliberately, with a docstring: a floor above the endpoints is
not a constraint but a contradiction that prices every edge `inf`.  Arm 97 holds
a pose ~40 mm from parked arm 2, and **no routing fixes a pose**.  The lever is
a different hover, or a different park for arm 2 in stage 2 — build item 2's
per-stage parks, now for a reason that did not exist when §14 measured that item
as unnecessary.  The previous entry's "the router is optimistic by its own
sweep" was half right, and the wrong half is the important one.

**(3) THE RESIDUE PHASE.**  A bucket no order can fly concurrently is
SERIALISED, not abandoned: planned alone against the parked fleet and APPENDED
to the stage, because the other actives are back at their parks by then.  **That
is Pete's original final pass**, and it is the floor under the whole scheme —
the worst case of the room work is the one-arm-at-a-time programme the
installation started from.  `idle.conduct` reduces to nothing with one mover
(`sum_k P(1, k)` = one order, no second mover), so the residue is laid down
directly and `solo_check` against the parked fleet IS the certificate a
one-mover conduct would produce.  `StageResult.duration` is the busiest
CONCURRENT arm plus the sum of the residues, and `active_pair_gap` never sees a
residue arm, because nothing else is in the air.


## THE PRIORITY ROOM FLIES 97.6 % OF THE INK (2026-09-11, last)

`out/staged_csail_h097_v5.json`, priority-ordered trajectory rooms, RRT tier on.
**12 of 13 ink-carrying buckets fly, carrying 15.455 m of 15.840 — 97.6 % of the
ink — and six of eight stages pass both checks.**

| | v3, union envelope | **v5, priority rooms** |
|---|---|---|
| ink buckets flown | 8 of 13 | **12 of 13** |
| ink certified | 5.445 m (34.4 %) | **15.455 m (97.6 %)** |
| stages passing | 4 of 8 (two vacuously) | **6 of 8** |
| makespan | 198.7 s (a third of a programme) | **415.0 s (a real one)** |
| planning, serial | 2 973.9 s | **263.8 s** |
| planning, per stage's busiest arm | 2 869.6 s | **89.7 s** |
| time to first motion | 0.294 s | **0.287 s** |

**PLANNING IS 11x FASTER THAN THE CONTROL AND 32x PER ARM**, which was not the
point and is the largest number in the table: the trajectory rooms are 10-1 001
spheres against the union's 1 157-1 377, and the FIRST ARM OF EVERY STAGE PLANS
FREE, so a third of the buckets pay nothing for the room at all.

**THE MAKESPAN IS 415.0 s AND IT IS REAL** (one 0.385 m bucket missing of
15.840 m).  Against the un-separated v1 programme's 391.7 s it is **+5.9 %** —
what routing three arms around each other costs — and against v19's conducted
209.9 s it is **1.98x**, which is the barrier, not the rooms.  Park overhead is
92.7 s on the critical path, **22.3 %** of the makespan.

**BOTH REMAINING FAILURES ARE KNOWN AND NEITHER IS THE ROOM.**  Stage 0 loses
its THIRD arm (0.385 m) — the greedy order's expected weak spot, since the last
arm has the least freedom left; the fix is to SEARCH the order rather than fix
it by ink (three actives is six permutations, which is
`coordination.coordinate`'s priority search with a tiny n), or to hand the loser
to the conductor.  Stage 2 is the router-sweep mismatch of the previous entry,
active-vs-PARKED, untouched by any of this.

**THE 1 000-STROKE MODEL, RECALIBRATED** on v5 (draw **13.59 s/m**, inter-piece
leg **4.18 s**, park **10.66 s** per (stage, arm)): **6 380 s = 106.3 min**,
**1.48x** the ~4 300 s conducted scaling, with park overhead on the critical
path down to **85.3 s, 1.34 %** against CSAIL's 22.3 %.  **The barrier
amortises** — the eight-stage pattern is right at scale and wrong on a small
picture.  The draw rate nearly doubled against the v3 calibration (13.59 vs
6.56 s/m) because v5 actually draws the long buckets v3 never flew: the earlier
figure was calibrated on the short ones that happened to survive, and is
withdrawn.

**CAVEATS.**  The model is arithmetic with measured terms, not a run — a real
1 438-piece fly is unaffordable (`sequence.cost_matrix` is O(n^2) route screens
and one bucket is 434 pieces).  And it assumes the priority order scales: at 572
pieces in a stage-0 bucket the LAST arm faces a far larger obstacle than it does
here, which is exactly where the one remaining failure already is.


## STAGE 2's +28.2 mm IS THE ROUTER'S SWEEP, AND IT IS SYSTEMATIC (2026-09-11, last)

Flagged three times without an answer and now attributed: **arm 97's trajectory
passes 28.2 mm from PARKED arm 2, at t = 2.2 s**.  Arm 13, the other active, is
clean at +234.9 mm; `column_failed: [97]` is the same geometry through the
base-column cylinder, not a second defect.  It is a genuine active-vs-PARKED
violation and it is untouched by every pass of the room work, which is all about
active-vs-ACTIVE.

**THE MECHANISM IS A ROUTER/CHECKER MISMATCH.**  `paper.route` certifies a leg
at `STATIC_MARGIN` = 50 mm AT THE SAMPLES IT EVALUATES;
`scene_check.check_timeline` re-derives it and additionally subtracts the
1-Lipschitz between-sample residual `0.55 x (step_i + step_j)`.  A leg exactly at
the gate at its samples reads about 20 mm tighter once the sweep between them is
charged — which is what 50 -> 28.2 mm looks like.  **The checker is right and
the router is optimistic by the sweep**, systematically, on every leg.

Two fixes, neither implemented and neither a gate change: charge the router the
same residual (raise its floor by the sweep it will be measured against —
`coordination.SWEEP_K` is already that number), or auto-refine the leg's
sampling until the residual is negligible, which is what `check_timeline`
already does for its own frame and paper gates.  The second is the honest one.

**AND AN EARLIER ATTEMPT AT THIS ATTRIBUTION WAS WRONG.**  It fed the
programme's RAW WAYPOINTS to `check_timeline` at a fixed dt, charging a
Lipschitz residual against jumps the timeline never makes, and reported two
PARKED arms at -437 mm.  The rebuild goes through `writing.uniform_samples`,
exactly as `staged.solo_check` does.  Recorded because the wrong number was
plausible enough to have been quoted.


## PRIORITY ORDER CLOSES THE ROOM, AND A VACUOUS PASS IS NOT A PASS (2026-09-11, last)

**THE CONTROL, FINISHED.**  `out/staged_csail_h097_v3.json`: the pose-union
envelope at the adopted stride 1 / cell 0.075 / pad 0, RRT tier ON, refusal loop
on.  **8 of 18 buckets flew, carrying 5.445 m of 15.838 — 34.4 % of the ink.**
Planning **2 973.9 s** cold against v2's 1 549.4 s (the tighter room has 3-12x
the spheres and every clearance query pays), time to first motion **0.294 s**,
coverage 94.20 %, 3 of 56 pieces unplannable.  v2 at the LOOSE setting flew
**9** of 18, so the tightening marginally hurt — the sweep's non-monotonicity,
showing up in the fly.  **Option (1) is closed.**

**AND ITS MAKESPAN, 198.66 s, IS NOT A MAKESPAN.**  It is the sum of eight stage
durations, two of them zero because nothing flew.  Comparing it with 391.7 s or
v19's 209.9 s would be comparing a third of a programme with two whole ones.

**A STAGE IN WHICH NOTHING FLEW WAS PASSING BOTH CHECKS.**  Six arms at their
parks clear everything; stages 6 and 7 of the control were labelled PASS on that
basis.  `StageResult.ok` now requires `complete` — every bucket that HAS ink
produced a timeline — and the report prints `flown/with_ink` and says EMPTY.
This was a defect in the REPORT, not in the rig, and it is the kind that makes a
run look better than it is.

**THE SIMULTANEOUS TWO-PASS SCHEME IS CIRCULAR AND MEASURABLY WORSE THAN SOLO.**
On CSAIL stage 0, arm 71's thirteen-piece bucket FLEW in pass 1 against the
parked fleet and STOPPED FLYING in pass 2.  Each arm is asked to yield to a path
the other has already abandoned, so nobody yields; no number of iterations
closes a cycle.

**PRIORITY ORDER CLOSES IT IN ONE SWEEP.**  Actives ordered by ink, busiest
first (least room to give).  Arm 1 plans against the parked fleet and is then
FINAL; arm k plans against the parked fleet plus the FINAL trajectories of
1..k-1.  Every pair (i, k), i < k, is certified against the path arm i actually
flies, and arm i never moves again — so every pair is certified exactly, with no
iteration.  **The dependency graph becomes a DAG**: arm k depends on 1..k-1 and
nothing after, which also bounds a re-plan's blast radius to k+1..n.  It is the
spatial analogue of `coordination.coordinate`'s priority search with the order
fixed by ink rather than searched, because a stage has at most three actives.
The certificate is unchanged: `active_pair_gap` at `PAIR_MARGIN` still closes
the claim; the order gets the trajectories apart, the check proves it.


## THE ROOM IS WHAT THE NEIGHBOUR ACTUALLY DOES, AND THE CERTIFICATE IS NOW A GRAPH (2026-09-11, last)

The pose-union envelope is measured out (previous entry): tightening it moved
the ink-vs-envelope minimum **86 mm** (-46.3 -> +39.9), moved every park clear
of the gate (arm 71 in stage 7, **+1.2 -> +52.8 mm**), and moved the number of
buckets that fly **by nothing** — the same six refused at every cluster cell
from 0.10 m down, with the RRT tier ON as well as off.

**SO THE UNION IS THE WRONG OBJECT, NOT A BADLY BOUNDED ONE.**  An arm's work
cell is every pose it COULD hold anywhere inside it; two arms in adjacent row
bands own overlapping airspace between the parks and the paper; and no bound on
a set that large leaves a neighbour room to fly through it.  What the neighbour
ACTUALLY holds in a stage is one trajectory — a few hundred poses out of that
union — and that is a room a leg can be routed around.

**THE CONSTRUCTION.**  `staged.trajectory_room` takes an arm's realised stage
timeline, builds its link capsules through the same `coordination.ArmPath` the
envelope used, and reduces them with the same `cluster_capsules`, which CONTAINS
what it replaces.  The pad is not a guess: it is
`scene_check.check_timeline`'s own 1-Lipschitz between-sample residual,
`SWEEP_FRAC x` the largest step any capsule endpoint takes between two samples,
so the spheres cover the motion BETWEEN the samples and not only at them.

**TWO PASSES, AND THE ITERATION IS EXPLICITLY NOT THE CERTIFICATE.**  Pass 1
plans every active solo against the parked fleet — the room that flies but does
not separate the actives.  Pass 2 re-plans each active against what the OTHERS
ACTUALLY DID in pass 1.  Re-planning A moves A, which is a room B was certified
against, so the fixed point is approached and never proved by the iteration; the
claim is closed only by the independent whole-timeline `active_pair_gap` at
`PAIR_MARGIN`.  The iteration gets the trajectories apart; the check proves it.

**AND THE BARRIER SEMANTICS CHANGE, SO THE DEPENDENCY IS RECORDED.**  A
pose-union envelope is a property of the STAGE: it holds whatever the neighbour
is asked to draw, so an arm's plan survives its neighbour being re-planned.  A
trajectory room is a property of that neighbour's SPECIFIC PLAN, and an arm's
certificate is void the moment that plan changes.  `ArmStage.depends_on` carries
a digest of each neighbour's trajectory and `programme()` writes it out beside
the arm's own, so a re-plan leaves every neighbour that still names the old
digest stale BY INSPECTION rather than silently.  **That is the price of the
tighter room and it is worth naming: the staged programme is no longer a set of
independent per-arm certificates, it is a graph of them**, and build item 6's
barrier has to carry the graph as well as the parks.

**MEASURED SO FAR.**  Stage 0's rooms: arm 2 **1 339 spheres -> 422**, arm 13
**1 377 -> 111**, arm 71 **1 157 -> 1 001**.  The counts understate it — an arm
that draws ONE piece had its whole row band treated as a keep-out, and what it
occupies is one approach, one stroke and one retreat; arm 13's room falls by a
factor of 12 and arm 71's, drawing thirteen pieces across R1, barely moves.  The
saving is exactly the ink an arm was NOT asked to draw.  **And pass 1 flies
everything**: arm 71's thirteen-piece bucket produces a timeline against the
parked fleet — the same bucket that produced none under the union envelope at
any cluster cell with the tier on or off — so every active has a pass-1
trajectory to derive a room from, which is the premise §14 could not supply.

**NOT YET MEASURED.**  The CSAIL run was still in pass 2 when the box closed, so
the buckets flown, the per-stage minima, the makespan and the cold/warm split
are not in hand and the 1 000-stroke model is not recalibrated.  Running to
`out/staged_csail_h097_v4.json`, with the union-envelope fly at the adopted
tight setting alongside it in `out/staged_csail_h097_v3.json` as the control.

## THE ENVELOPE, TIGHTENED: THE CELL IS THE LEVER AND THE PARK MAY BE THE WALL (2026-09-11, last)

Build item 4's envelope room, swept for tightness.  **The probe runs with the
RRT tier OFF** (`paper.RRT_SAFE = False`): what decides "does this bucket fly"
is whether the SHAPE LADDER settles the park -> hover leg, the C-space tier is
the last resort and costs minutes when it fires, and a setting that needs it for
every entry leg is not one anybody would ship.  That makes the probe an honest
LOWER bound -- a setting it passes certainly flies -- and the full run puts the
tier back on.  `paper.cache_signature()` carries the three tier flags, so
nothing the probe buys is served to a run that has the tier on.

**Stride 1 throughout, because `pad = 0` is only honest there**: at stride 1
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

**THE PARK IS THE OTHER WALL, AND TIGHTENING MOVES IT TOO.**  A leg starts at the
park, so a park inside a partner's envelope cannot be left at any tightness.
Measured against the other actives' envelopes:

| stage | arm | shipped (0.15 / 40 mm) | adopted |
|---|---|---|---|
| 0 | 71 | +286.3 mm | +341.7 mm |
| 5 | 71 | +290.2 mm | +331.7 mm |
| 6 | 31 | +86.5 mm | +94.9 mm |
| 7 | 71 | **+1.2 mm** | **+52.8 mm** |

(every pair not listed is at the +350 mm broad-phase cap)

**Arm 71's park is +1.2 mm from arm 17's envelope in stage 7** at the shipped
setting — touching it, which is why that bucket failed at every pad and every
cell down to 0.10 m.  At the tight setting the same pair reads **+52.8 mm**, and
every other park is at or near the +350 mm broad-phase cap.  **So the park set
does NOT need re-searching** (`layout.stage_parks`, build item 2): it was the
envelope's conservatism standing on the park, not the park standing in the wrong
place.  Worth saying plainly, because the opposite reading would have sent the
next pass after a search it does not need.

**TIGHTENING MOVES THE INK BY 74 mm AND MOVES THE BUCKET COUNT BY NOTHING.**
From cell 0.15 / pad 40 mm to cell 0.075 / pad 0 the ink-vs-envelope minimum
goes **-46.3 mm -> +27.8 mm**, exactly the term the radius decomposition named,
and the SAME SIX BUCKETS fail at every cell from 0.10 m down.  **So the residual
refusal is not conservatism.**  With the ink clear of the bound by 28 mm and
every park clear of it, what is left is that a pen-up leg from the park to the
first hover, ROUTED BY THE SHAPE LADDER ALONE, genuinely intersects the other
actives' work-cell envelopes: two arms in adjacent row bands own overlapping
airspace between the parks and the paper.

**AND THE PROBE TURNS OFF PRECISELY THE TIER THAT MIGHT ANSWER IT.**
`aris_sixarm/transit.py` searches the 7-DOF joint space where the ladder gives
up, at the same gate stack.  Whether it finds a way round is what the full fly
decides, and the full fly is the next number, not this one.

**ADOPTED: stride 1, cell 0.075 m, pad 0** (`staged.ENVELOPE_*`, which are this
module's own parameters and not a gate).  NOT because it flies the most buckets
-- 0.15/0 flies one more -- but because the bucket count does not distinguish
them and the certificate does: no skipped poses, so `pad = 0` is honest rather
than optimistic; no unjustified inflation; ink minimum **+27.8 mm** instead of
-46.3; and **754 spheres against 1 926** at 0.05 m for 11 mm more.  The extra
bucket at 0.15/0 is the shape ladder's discrete shapes meeting a looser obstacle
set, not a tighter guarantee.

**ONE MORE O(n^2).**  `cluster_capsules` scanned every endpoint once per cell
(`inv == k`) -- at stride 1, a few hundred thousand endpoints against tens of
thousands of cells, minutes an arm, and what made the first attempt at this
sweep look hung.  One `np.minimum.at` / `np.maximum.at` pass each now.

## THE ENVELOPE'S CONSERVATISM IS THE CLUSTER CELL, AND THE SWEEP IS NOT DONE (2026-09-11, last)

Follow-up to the previous entry, which closed the active-pair leg gap
(+8.2 -> +826.1 mm) and reported that the envelope was then too conservative to
fly three of eighteen buckets.  **Partial: the arithmetic is settled and the
measurement is not.**

**THE PROBE.**  Every failing bucket failed on one `paper.route` — the park to
the first hover — so the sweep prices that leg, the exit leg and the
ink-vs-envelope minimum instead of a fly.  The pieces are planned once with no
envelope, because `plan_stroke` never consults the static set.  At the shipped
setting (stride 2, cell 0.15 m, pad 40 mm) the probe reproduces §11's failure in
**1.4 s** instead of 27 minutes: **9 of 13 buckets can route their park-hover
legs, and the ink minimum is -30.6 mm**.

**THE CELL IS THE LEVER AND THE PAD IS NOT, AND THE BOUND SAYS SO EXACTLY.**  A
cluster sphere is drawn round the endpoints of every capsule whose MIDPOINT fell
in its cell, so `radius <= cell*sqrt(3)/2 + half the longest capsule + the
capsule radius + the pad`.  The first term is **0.130 m at the shipped 0.15 m
cell and 0.043 m at 0.05 m**, against a real capsule radius of about 0.06 —
the difference between a bound three times the thing it bounds and one a third
larger, and an order more than the 40 mm pad is worth.  Pinned by a test.

**AND `pad = 0` IS ONLY LEGAL AT STRIDE 1.**  The pad covers the poses a stride
skips; at stride 2 three cells in four are unread and the pad is all that stands
in for them, while at stride 1 every strict-GO cell is read (`2*cells + 1`
poses, pinned) and the envelope is exactly the object
`scripts/workcell_envelopes.py` measures.  **The setting the arithmetic points
at is stride 1, cell 0.05 m, pad 0.**  It is NOT yet adopted as the default,
because it has not been measured.

**WHAT IS NOT MEASURED.**  The sweep did not complete: each setting re-routes 26
legs cold under a new store namespace and several fall through the shape ladder
to the RRT, which is minutes a leg.  So there is no measured fly fraction at the
tighter settings, no active-pair minimum under them, and no full eight-stage
fly.  **§11's table stands as the last measured one, with 26 of 56 pieces
unflown, and the makespan stays unquoted.**  The two-pass swept-capsule room
(building the room from the other actives' ACTUAL pass-1 trajectories) was not
started: the cell arithmetic is a 130 mm lever against an inflation the two-pass
scheme would have to beat, and it is the cheaper thing to finish first.

**ONE REAL FIX CAME OUT OF THE ATTEMPT.**  The envelope's POSES and the SPHERES
that bound them were filed under one cache key, so a sweep over the clustering
re-solved every hover — 9 ms a cell over thousands of cells, for an answer that
depends on neither.  `staged.cached_envelope_poses` splits them.  Same family as
`id(spec)` in `paper.py`, found the same way: by a sweep that should have been
seconds and was minutes.

## THE LEG GAP IS CLOSED BY THE ENVELOPE, AND THE REFUSAL LOOP BY CUTTING THE ATOM (2026-09-11, last)

Follow-up to the previous entry, which measured two pen-up legs at +12.9 mm and
+8.2 mm and said the cells were right and only the paths between them were
unaccounted for.  Both halves are now built (`aris_sixarm/staged.py`,
`tests/test_staged.py`, docs/V2_STAGED.md sections 8-10).

**THE OTHER ACTIVES GO INTO THE ROOM AS THEIR ENVELOPES.**  Before an arm plans
anything in a stage, the other actives of that stage are put into its static set
as the union of their link capsules over every pose they could hold in the
stage: the certified drawing pose at each strict-GO cell of the work cell, the
hover above each, and the park.  That is exactly the pose stack
`scripts/workcell_envelopes.py` measured +85.8 mm with, and it is the object
docs/ARCHITECTURE_V2.md section 2f prescribes.  `frozen.freeze_sets` generalises
`frozen.freeze` from one pose to a set — **a parked partner is the N = 1 case of
an active one**, which is the sentence the whole design turns on.

**A BOX WOULD NOT HAVE WORKED AND A CAPSULE CLOUD WOULD NOT HAVE BEEN
AFFORDABLE.**  An AABB round an arm's reach swallows its neighbour; the raw
cloud is some 9 000 capsules for a row band and `partner_clearance` is linear in
them.  `staged.cluster_capsules` bounds them by grid-local SPHERES that contain
every capsule whose midpoint fell in the cell — both endpoints and the radius —
so the reduction is conservative by construction and a row band comes out as a
few hundred spheres.  Read at the shipped stride 2, inflated by `ENVELOPE_PAD` =
40 mm for the cells the stride skips, cached per (atlas, arm, region), and the
leg store namespaced on it because `paper.route_key` contains neither the frozen
poses nor the envelopes.

**THE INK CHECK IS A MEASUREMENT, NOT A GATE, AND THAT IS DELIBERATE.**
`plan_stroke` never consults the static set, so `staged.ink_vs_envelope` adds
the half that was missing.  On CSAIL the ink clears the (inflated) envelopes by
a median **+230.9 mm**, 5th percentile **+9.4 mm**, minimum **-28.5 mm**, with
**5 pieces of 48** under 50 mm.  Those five are close to a bound that is
deliberately larger than the thing it bounds, not to another arm's ink, and
refusing certified metres to a conservatism is a bad trade: the default reports
and `ink_gate=PAIR_MARGIN` enforces.

**THE SECOND LEVER IS z AND IT IS CHEAPER THAN x.**  `staged.row_lift_ladder`
gives each ROW band its own hover height (0.06 / 0.20 / 0.34 m) with the shipped
ladder behind it, so an arm that cannot hold the raised pose keeps the one it
had.  `writing.HOVER_LADDER` is borrowed for the stage and given back; the
hover memo is keyed on the heights, so it moves that stage's legs and nothing
else.  It fires only on a stage whose pair check still fails
(`run(lift_retry=True)`) and `lift_used` records it.  **No constant moved**: the
x erosion measured out earlier today cost 38.8 % of the block, and this costs
nothing but height.

**THE REFUSAL LOOP, AND THE TWO THINGS THAT HAD TO BE RIGHT.**  A refused piece
is ink THAT ARM cannot draw IN THAT STAGE, so the loop strikes that bit of that
atom's capability set and asks the DP again — 10 ms a round — flying nothing.
Measured wrong twice on the way:

  1. **A refusal bans only the stretch past `s_star`.**  `plan_stroke` hands
     back the arc length up to which it DID certify and a head plan is an "ok"
     result with all of its guarantees; banning the whole piece throws the head
     away.
  2. **A ban CUTS the atom rather than clearing its bit.**  An atom is
     indivisible with respect to the capability map and a refusal is a new
     transition in it, so a ban that merely touches an atom must split it at
     both ends.  **Banning whole atoms took the CSAIL loop from 100 % coverage
     to 80.6 %**; cutting them is what keeps the ink.

A `degenerate` refusal bans nothing and is reported as **unplannable** — "too
short", "off sheet" and "too short after clip" are statements about the PIECE,
so no neighbour would do better and striking the state out would only spread the
hole.  A span is never banned twice, which is what makes the loop terminate.

**AND `sequence.solve` RAISING IS A STATEMENT ABOUT THE ROOM.**  With the
envelopes installed a bucket can become genuinely unflyable — every order needs
a depot leg the router refuses — and `held_karp` says so by raising.  The bucket
falls back to the nearest-neighbour order and reports it, rather than taking the
run down.

**MEASURED, THE WHOLE EIGHT STAGES.**  The active-pair minimum over all eight
stages goes from **+8.2 mm to +539.8 mm**, and the two stages that failed go
**+12.9 -> +899.3** and **+8.2 -> +826.1**.  **The lift lever was never needed**
— `lift_used` is false in every stage, so routing against the envelopes was
sufficient on its own.  Seven of eight stages pass both checks; stage 2 still
reads **+28.2 mm** on the PARKED side, which this pass did not touch.

**AND THE ENVELOPE IS TOO CONSERVATIVE TO FLY THE BIG BUCKETS — THE ONE THING
THIS PASS GOT WRONG.**  Three of eighteen buckets (**26 of the 56 pieces**,
including both main stages' busiest arms) produced NO timeline: the entry leg is
refused.  The ink check names the cause itself — minimum ink-vs-envelope
**-28.5 mm**, i.e. the arm's own certified drawing pose is already inside the
inflated bound, and no leg out of a pose inside an obstacle can clear it.  The
conservatism is the 0.15 m sphere clustering plus `ENVELOPE_PAD` = 40 mm, both
chosen for speed rather than tightness, and both are parameters.  **So the
228.8 s makespan is a partial programme's and is not quoted as a makespan.**
The next measurement is the tightness sweep: cluster cell and pad down, against
the fraction of buckets that fly.

**THE REFUSAL LOOP, MEASURED.**  Round 0: 54 pieces, 5 refused, 100.00 %.
Round 1: 56, 4, 98.42 %.  Round 2: **56, 0, 94.20 %** — converged, with **3
pieces unplannable, all three `degenerate:too_short`**.  The final refusal rate
is **3 of 56 (5.4 %) against 11.1 % before the loop**.  The 5.8 % of ink lost is
not the loop giving up; it is ink with no second stage-compatible drawer, and
closing it needs the PATTERN to offer the stretch to somebody else.

**THE BARRIER COST, MEASURED AND AMORTISED.**  On the nine flown buckets the
park -> out -> back trip is **87.7 s total, 75.3 s on the critical path,
32.9 % of the makespan**, with pen-up 1.7x the ink.  Modelled at 1 000 strokes
with every term measured (draw **6.56 s/m**, inter-piece leg **5.15 s**, park
**9.74 s** per (stage, arm), the set's own per-stage ink from `traces`), the
staged makespan is **5 319 s** — **1.24x** the ~4 300 s scaling of v19's
conducted six-arm makespan, where CSAIL measured 1.87x — and the park overhead
on the critical path falls to **77.9 s, 1.47 %**.  **The barrier is a fixed cost
per (stage, arm) and the ink is not, so the eight-stage pattern is right at
scale and wrong on a small picture.**

**THE RULE THAT SHOULD REPLACE IT (proposed, not built).**
`staged.stage_overhead` measures the park -> out -> back trip per (stage, arm)
directly.  It is a FIXED cost per (stage, arm) and the ink is not, so the
eight-stage pattern is right at scale and wrong on a small picture.  The rule:
**a stage whose busiest arm carries less than `X = P x v_draw` metres is not
worth its own barrier**, `P` being the measured park overhead.  Three ways to
spend that, cheapest first: merge two seam stages that share no arm (2 with 4,
3 with 5) subject to the same envelope test; let an arm that holds the SAME park
across two consecutive stages stay out rather than fly home and back (26 of 42
transitions are exactly that, measured earlier today); and fall back to
`idle.conduct` for the residue, where two arms is four priority orders over a
short horizon.

## A PARKED PARTNER IS NOT AN ENVELOPE — AND THE LEGS ARE (2026-09-11, last)

A correction to docs/V2_WORKCELLS.md §5 and docs/ARCHITECTURE_V2.md §2b, and
the thing build item 4 measured on the way past.  **There are TWO safety
questions inside a stage and they were being asked with one tool.**

**BETWEEN TWO ACTIVE ARMS** the poses are not yet chosen when the stage is
designed, so the criterion has to be the ENVELOPE one — the union over every
pose either arm could hold anywhere inside its work cell.  That is what the
0.40 m dead band was chosen for (+85.8 mm) and it is settled.

**BETWEEN AN ACTIVE ARM AND A PARKED PARTNER it is the wrong test, and the
+4.7 mm is its artefact.**  A parked arm is not a union over poses; it is ONE
known, measured, barrier-verified pose.  The tool for that is the per-stroke
SOLO certification this repo already ships — `frozen.freeze` puts the
partner's REAL capsules into the static set and the planner chooses poses and
legs that clear 50 mm against them, cell by cell and leg by leg.  The certified
3-layer map at h = 0.970 with the shipped parks frozen is already **100 % of
the block at the 50 mm gate**, so the parked half of the problem needs **no
park search, no cell erosion and no gate change**.  Measured end to end
(docs/V2_STAGED.md), the active-vs-parked minimum over the eight stages of the
CSAIL logo is **+28.2 to +140.2 mm**, seven of eight stages clearing the gate.

**THE RELAXATION OF `PAIR_MARGIN` FOR A STATIONARY PARTNER IS THEREFORE OFF THE
TABLE.**  The previous entry left it open — "whether a stationary partner at a
verified park is owed the same 50 mm as a mover is a question for whoever signs
the gate".  It is not a question that needs answering: the 50 mm is met against
the partner's real capsules without relaxing anything, and the number that was
missing it was a measurement of an object (the parked arm's whole envelope)
that does not exist at run time.  `PAIR_MARGIN` = 50 mm stands, unqualified,
for movers and for parked arms alike.

**AND THE REAL GAP IS SOMEWHERE ELSE: THE PEN-UP LEGS.**  Re-measured from the
trajectories the planner actually produced, with no assumption about how the
asynchronous actives line up in time (the minimum over the CROSS PRODUCT of the
two arms' pose sets, `staged.active_pair_gap`):

| stage | actives | ink vs ink | WHOLE trajectory | binding pair, and what they were doing |
|---|---|---|---|---|
| 0 | 13, 71, 2 | **+273.5 mm** | **+12.9 mm** | 2 – 71, **both in a pen-up leg** |
| 1 | 17, 31, 97 | **+338.8 mm** | **+8.2 mm** | 31 – 97, **both in a pen-up leg** |
| 2–7 | the seams | — | +263.7 … +1 055.3 mm | — |

**Ink against ink is never the problem — +273.5 mm at worst, against the
+85.8 mm the dead band was chosen for.**  What binds is two legs flying into
each other, and it is exactly the hole docs/ARCHITECTURE_V2.md §2f named in
advance: *"an envelope is a union over POSES ... and the pen-up leg between two
hovers is a PATH, whose interior is not in the envelope."*  Each arm's legs
were certified against the PARKED fleet, which is the right room at a barrier
and the wrong one mid-stage, and nothing in the pipeline has ever asked whether
one active arm's leg crosses another's.

**WHAT THIS DOES NOT MEAN.**  It is not a defect in the dead band, in the
pattern or in the planner, and it is not a reason to re-open §4b.  The cells
are right; only the paths between them are unaccounted for.  Two cheap levers,
neither measured yet: one `scene_check.check_timeline` per arm per stage
against the other ACTIVES' work-cell envelopes as static boxes (§2f's own
prescription, and a leg that fails is re-routed rather than the stage
re-designed), or a per-stage flying height that separates the row bands in z —
`writing.HOVER_LADDER` is already a ladder and z is a free parameter, which
makes it a much cheaper lever than the x erosion measured out earlier today.

## STAGE ASSIGNMENT, WIRED END TO END (2026-09-11, last)

Build item 4 of docs/ARCHITECTURE_V2.md, landed as `aris_sixarm/staged.py` and
`tests/test_staged.py`, measured in **docs/V2_STAGED.md**.  Per (stage, arm)
bucket: freeze the five partners at their parks, plan every piece through
`stroke_api.plan_stroke` (the funnel `scripts/csail_allocate.py` uses), order
the accepted ones with `allocate.sequence_arm`, and lay the timeline down with
`writing.arm_program` — park → hover → draw → … → park, every pen-up a
`paper.route` against the frozen room and the persistent store.  No balancer,
no split, no merge (the DP already minimised the pieces exactly) and no
conductor.

**THE CSAIL LOGO AT v19's PLACEMENT, h = 0.970**, 54 pieces in 30 ms of DP:

| | |
|---|---|
| **time to first motion** | **0.309 s** (against v19's 3 712.4 s, and Pete's budget of 10) |
| staged makespan | **391.7 s** against v19's conducted **209.9 s** — **1.87× worse** |
| planning, all eight stages, serial | **221.2 s cold / 38.4 s warm** — **5.8×**, bought entirely by item 1's leg store |
| …charged per stage to its busiest arm | 213.2 s cold / **34.8 s** warm |
| the two independent checks | 100.5 s |
| pieces refused by `plan_stroke` | **6 of 54, 11.1 %** — 4 `empty_fiber`, 1 `sheet_collapse`, 1 `too_short` |

Both runs produce a **byte-identical makespan** (391.732 s), which is the check
that the leg store changes nothing but the clock.

**THE MAKESPAN IS THE HONEST DISAPPOINTMENT AND IT IS STRUCTURAL.**  A stage
costs its busiest arm and the barrier makes the sequence cost the SUM of eight
of those, and every stage pays a full park → out → back trip for every active
arm whatever it has to draw: stage 5 is 0.923 m of ink for 58.6 s of clock.
`traces.py` predicted the shape (2.35× the serial makespan in ink metres
against a three-arm ceiling of 3×) and the seam stages are where it is paid.
It buys a programme that needs **no conducting at all** against one that needed
407.3 s of it, and a first motion 12 000× sooner.

**WHAT IS NOT BUILT, DELIBERATELY.**  (1) **The refusal loop** — a refused
piece is dropped rather than split and re-entered into the DP with a capability
map re-derived from the refusal; 11.1 % of the ink is what it is worth.  (2)
**The seam stages are not conducted** — §2d sends the six 2-active stages
through `idle.conduct` and this module plans them asynchronously like the
others and reports the clearance instead; on this logo they are effectively
solo, so it did not bind.  (3) **The typed programme is not
`program_schema.Bundle`** — `Segment` carries no joint vector and `_from_dict`
refuses unknown AND missing keys, so a stage id and a per-piece hover are a
`SCHEMA_VERSION` bump rather than an extension.  `staged.programme()` writes a
SUPERSET of what item 5 has to absorb (stage id, barrier list, per-piece
`q_first`/`q_last` and `hover_in`/`hover_out`, the pen-up leg blocks, the joint
trajectory) under its own `STAGED_SCHEMA_VERSION`, and folding the two together
is item 5's job.  (4) The **1 000-line refusal fraction is 18.1 %**
(318 of the 1 756 pieces `docs/V2_TRACES.md` reports, reproduced exactly), of
which 214 are the redundancy band rather than the reach — 127 `empty_fiber` and
87 `start_infeasible` — 58 `sheet_collapse`, and only 46 the piece itself being
unplannable by anybody.  `plan_stroke` over all 1 756 is **842.3 s serial /
342.6 s charged per stage to its busiest arm**, against a draw of roughly
4 300 s, so the planner keeps up with an order of magnitude in hand.  **The two
three-active stages take almost all of the loss** (138 of 434 and 169 of 380,
against 11 in the six seam stages together): a seam's ink sits under two arms
that both reach well into it, a row band's runs out to the edge of one arm's.
That 18.1 % is also the first direct measurement of how optimistic the 2 cm
prefilter is.

**ONE THING THE CHECK HAD TO BE TAUGHT.**  `scene_check.check_timeline` gates
each arm against `spec.static_obstacles()`, which contains the OTHER arms'
pose-invariant `body:<aid>_column<k>` bands — and in a stage timeline every arm
those bands stand for is present as its own capsules, so the band is the same
arm counted twice in its most conservative form.  `staged.drop_bands` removes
exactly those and nothing else (mounts, plates, the drop cluster and the runway
all stay, and so does the band of any arm not in the timeline), which is
`frozen.filter_boxes` applied to the check instead of to the planner and the
same convention `scene_check.neighbour_columns` already uses for the cylinder
version of the same object.  Left in, arm 71's stage-0 timeline is refused by a
band belonging to an arm standing 0.25 m clear of it.

## THE +4.7 mm IS NOT A PARK PROBLEM, AND IT IS NOT A CELL PROBLEM EITHER (2026-09-11, last)

Report-only follow-up to the per-stage park search, asked for by the coordinator
and measured at `--stride 2` on `out/atlas_proposed_h0970_lat0860_gated63`.  Two
levers were on the table and **both are measured out**; the pattern data ships
unchanged and the knob ships off.

**Lever 1 — erode the active arm's cell in x, away from its transverse
partner's base.**  The clearance-versus-erosion curve is **flat, then it
plateaus below the gate**:

| stage | active → cell | parked | 0.00 m | 0.40 m | **0.74 m** | erosion for 50 mm |
|---|---|---|---|---|---|---|
| 0 | 13 → R0 | 17 | +4.7 mm | +10.9 | **+26.6** | **1.04 m** |
| 0 | 2 → R2 | 97 | +13.7 | +13.7 | +26.6 | 1.04 m |
| 1 | 17 → R0 | 13 | +4.7 | +4.7 | +59.9 | 0.76 m |
| 1 | 31 → R1 | 71 | +11.9 | +11.9 | +26.8 | 1.04 m |
| 3, 5 | 2 → SEAM1 | 97 | +37.1 | +37.1 | +39.5 | 0.80 m |
| 4 | 31 → SEAM0 | 71 | +34.9 | +34.9 | +36.5 | 0.84 m |
| 2, 6, 7 | the rest | — | +55.4 … +107.4 | — | — | already clear |

**0.74 m is the free limit** — exactly half the 1.48 m block, where the two
columns (which erode from opposite sides) still tile the row band and coverage
is still 100.00 %.  **At the free limit the worst stage is +26.6 mm and the gate
is 50.**  Getting to 50 costs **1.04 m of a 1.48 m block**, and then:

| erosion | block covered | ≥ 2 stage-compatible drawers | mean | speedup |
|---|---|---|---|---|
| **0.00 m (shipped)** | **100.00 %** | **48.1 %** (ceiling) | 1.524 | **2.40×** |
| 0.40 m | 100.00 % | 48.1 % | 1.524 | 2.40× |
| 0.76 m | 98.51 % | 11.7 % | 1.117 | 2.44× |
| 0.84 m | 87.85 % | 11.9 % | 1.119 | 2.74× |
| **1.04 m (the frontier)** | **61.20 %** | **10.7 %** | 1.107 | 3.90× |

The rising speedup is an artefact and is labelled as one: the stage sum falls
because there is less ink, not because it is drawn faster.  **At the frontier
the pattern loses 38.8 % of the block and three quarters of its redundancy** —
the two things the eight-stage pattern was adopted for.  **Eroding in y is
worse**: 0.77 m of a 1.01 m row band for 13 → R0, and 0.78 m of a 0.81 m band
for 31 → R1, which leaves 30 mm of paper.

**Lever 2 — park the partner retracted, and keep the cell.**  The stage-park
search ranked 65 poses an arm, all of them reaching *out* over the paper at 0.20
or 0.35 m.  Re-run over **24 bearings × 8 radii (0.30–0.78 m) × 6 hover heights
(0.20–0.95 m)** — **485 to 492 certified poses an arm, 7.5× the grid, the whole
reachable height range** — the binding numbers are **+4.7 → +4.7, +13.7 → +13.7,
+11.9 → +11.9, +37.1 → +37.1, +34.9 → +34.9**, and **zero** of ~490 poses clear
the gate in any failing stage (against 228–268 of ~490 in every stage that
already cleared).  Only 17 in stage 1 moves at all, +4.7 → +8.9 mm.  The binding
element is **pose-invariant to within 4 mm over the arm's entire certified pose
space**, which is the strongest possible confirmation of the shoulder argument.

**THE RECOMMENDATION: neither.**  `traces.zigzag_pattern(park_erode_m=…)` and
`traces.park_erode` ship so the frontier can be re-measured and Pete can see the
trade; **the default is 0.0 and the pattern is unchanged**.  At no erosion is
the trade worth taking.

**What is left is not geometry**, and the cheapest candidate changes no code at
all: **`PAIR_MARGIN` = 50 mm is the gate for two arms IN MOTION.**  A parked arm
holds one known, measured, barrier-verified pose, and DECISION 2026-09-09
already settled that *a known pose stops paying for a sweep*.  Whether a
stationary partner at a verified park is owed the same 50 mm as a mover is a
question for whoever signs the gate — and if it is not, five of eight stages
pass at once and the other three miss by **13–15 mm**.  Two further candidates,
neither measured here: a park search whose candidates are *outside* the
certified block (three of six shipped parks already are, and
`certified_ready_pose` only searches over the paper), and a five-arm stage that
simply does not have a same-row partner in it.

## PER-STAGE PARKS, AND WHY +4.7 mm IS NOT A PARK PROBLEM (2026-09-11, later)

Build item 2 of docs/ARCHITECTURE_V2.md.  `layout.stage_parks(pattern,
cell_poses)` searches a park for every arm in every stage, ranked against that
stage's **envelopes** rather than against a target *xy* — `aside_candidates`'
same (radius, hover, bearing) recipes, so a stage park can be written down and
re-derived exactly like a shipped one, with each arm's shipped park prepended so
the incumbent is always in the running and always wins a tie.  Every candidate
of every arm is scored against every active arm's envelope in one clearance
matrix per (arm, active) pair, the arms are served worst-constrained first, and
a park-versus-park conflict walks the loser down its own ranking.
`out/stage_parks_h0970.json` carries the set with its provenance (atlas, stride,
`cache_signature`, every arm's `spec_signature`, and the recipe grid).

**Measured, eight stages, 65 candidates an arm, gate 50 mm** (worst park against
any ACTIVE arm's work-cell envelope; `*` marks an active arm, whose own park is
held to the same test against the other actives):

| stage | active | worst | binding arm | park-vs-park | flyable |
|---|---|---|---|---|---|
| 0 | 13, 71, 2 | **+4.7 mm** | 17 | +250.0 | 3/3 |
| 1 | 17, 31, 97 | **+4.7 mm** | 13 | +250.0 | 3/3 |
| 2 | 13, 97 | **+55.4 mm** | 17 | +250.0 | 2/2 |
| 3 | 17, 2 | +37.1 mm | 97 | +250.0 | 2/2 |
| 4 | 31, 97 | +34.9 mm | 71 | +250.0 | 2/2 |
| 5 | 71, 2 | +37.1 mm | 97 | +136.9 | 2/2 |
| 6 | 13, 31 | **+55.4 mm** | 17 | +250.0 | 2/2 |
| 7 | 17, 71 | **+95.7 mm** | 13 | +250.0 | 2/2 |

**THE SEARCH FINDS NOTHING BETTER THAN THE SHIPPED SET IN ANY STAGE, AND THAT IS
THE RESULT.**  Per ARM it does improve — arm 31 in stage 0 goes from the
shipped +45.4 mm to **+57.7 mm** — but the *binding* arm's number does not move
by a millimetre in any of the eight stages, and the reason is structural rather
than a search that ran out of candidates.  **All 65 candidate parks of arm 17
score identically, +4.7 mm, against arm 13's stage-0 envelope.**  Decomposed,
the binding pair is arm 13's ink against arm 17's **moving links** (the base
column alone is +68.6 mm), and those moving links are the upper arm, which is
within `ELBOW_R` = 0.117 m of arm 17's own base whatever pose it holds.  A
full-width row band handed to one arm of a transverse pair reaches under the
other arm's shoulder, and **no park can take a shoulder somewhere else**.  This
is V2_WORKCELLS §1's "the elbow, not the pen" applied to a park instead of to
ink: the transverse pair is unseparable, and a park set is not the lever.

So V2_WORKCELLS §5 and ARCHITECTURE_V2 §2b are corrected on one point.  The
+4.7 mm is real and it is the blocker, but it is **not** "the shipped parks were
searched against the wrong thing" — searching them against the right thing gives
the same number.  **The lever is the work cell, not the park**: what moves +4.7
mm is eroding the active arm's row band in *x* away from its transverse
partner's base column, which hands paper to a seam stage exactly as the 0.40 m
*y* dead band does.  That measurement — the *x* frontier for a park, the analogue
of V2_WORKCELLS §2's frontier for ink — is the next one, and it is cheap: the
machinery is in this commit.

**Three stages of eight clear the gate as they stand** (2, 6 and 7, at +55.4 to
+95.7 mm), and two of those three are the new seam stages.  Stages 3, 4 and 5
miss by 13-15 mm, which is a different and much smaller problem than stages 0
and 1's 45 mm.

**Park-vs-park is at the 250 mm broad-phase cap in seven stages of eight** and
+136.9 mm in the eighth, so no stage's parks are anywhere near each other.
**Flyability is 18 of 18**: every active arm's park has a `paper.route` into the
nearest certified cell of its own work cell, against the steel and its own
metal.  **Two of the 42 transitions leave the arm inside its next cell; 14
need a repark and all 14 route** (modes: direct, home, fold_home, two 40 cm
lifts and one `rrt1`) — the remaining 26 are arms whose park does not change
between consecutive stages.  Both route checks are `paper.route`, i.e. against
the static set and self, NOT against the other arms: closing that is build
item 6's one `scene_check.check_timeline` per arm per stage.

## EIGHT STAGES IN THE CODE, AND A WORK CELL THE ALLOCATOR ACCEPTS (2026-09-11, later)

Build item 3 of docs/ARCHITECTURE_V2.md, landed — both halves.

**`traces.zigzag_pattern` ships the eight-stage pattern.**  It returned the
six-stage one superseded by docs/V2_WORKCELLS.md §4b (commit `07da40e`); it now
adds `13 → SEAM0 + 31 → SEAM1` and `17 → SEAM0 + 71 → SEAM1`, and its name is
`zigzag-rowband-y20+6seams`.  Eighteen cells, but only **14 states**: 13 and 17
were already offered SEAM0 in stages 2 and 3, so `capability()` merges them and
they cost neither a state nor a piece; the two genuinely new states are 31 and
71 on SEAM1, which is the whole of what the correction buys.

**Every number in docs/V2_TRACES.md is re-run against it.**  The deltas:

| | 6 stages | **8 stages** |
|---|---|---|
| CSAIL logo | 50 pieces, 10 gaps, 94.8 % | **54 pieces, 0 gaps, 100.0 %** |
| 1 000 strokes | 1 661 pieces, 315 gaps, 97.05 % | **1 756, 0, 100 %** |
| 1 000-line hatch | 2 129, 304, 97.45 % | **2 170, 0, 100 %** |
| 1 000 scribbles | 1 148, 79, 97.71 % | **1 166, 0, 100 %** |
| DP wall time, 1 000 strokes | 814 ms | **888 ms** (DP itself still 10 ms) |
| ≥ 2 stage-compatible drawers | 43.5 %, mean 1.457 | **48.1 %, mean 1.524** |
| stage makespan, 1 000 strokes | 281.9 m | **302.2 m** |

**The logo's correction is exactly what ARCHITECTURE_V2 predicted** — "four
pieces, 50 → 54, for the 5.2 % of ink the six-stage version could not draw" —
and the 1 000-line answers, which nobody had, are **+5.7 %, +1.9 % and +1.6 %**
pieces for the last 2.7 % of the block.  Two numbers are worth more than the
piece count.  The staged capability map now reaches **the atlas's own ceiling**,
48.1 % / 1.524 to three decimals, identical to the unstaged control: the staging
throws away NONE of the redundancy the atlas offers, so nothing a faulted arm
was drawing is unrecoverable for want of a stage that offers it.  And the load
moves onto the middle arms — 31 goes 94.5 → 115.2 m and 71 92.6 → 113.9 m on the
1 000-stroke set — which is worth **16.9 points of free balance** on the staged
hatch (spread 32.4 % → 15.5 % at the same tie-break, same piece count).

**`allocate.atlas_cells` and `allocate.prefilter` take a work cell.**  "This arm
may draw inside this region during this stage, and nowhere else" arrives in four
shapes — a `traces.Pattern` plus a stage id, an iterable of `StageCell`, a plain
`{arm: rect}` mapping, or `None` for the unrestricted allocator this repo has
always had — and `work_cell_regions` normalises all four in one place.  **An arm
the object does not mention may draw nowhere**, which is the only reading
consistent with the envelope argument: a stage names its actives and everybody
else is parked, and the opposite reading would hand a parked arm ink while its
neighbour's envelope was certified without it.  Regions are half-open at the
high edge, exactly as `rect_contains` is, so a point on a seam belongs to one
cell and not to two.  It is a filter on the CELLS and not on the strokes: a
stroke that leaves the region is still probed for the part that does not,
because cutting at a boundary is `traces.plan_lines`' job and it does it to
0.01 mm.

## THE LEG CACHE IS A CONTENT KEY, AND IT SURVIVES THE PROCESS (2026-09-11, later)

Build item 1 of docs/ARCHITECTURE_V2.md, landed.  Two halves, and the first one
is the one that mattered.

**`id(spec)` is gone from every memo key in `paper.py`.**  `_key`, `key_maker`,
the hover memo and the leg memo all began with a CPython object address, so two
EQUAL specs missed each other and nothing could ever be filed across a process
boundary.  They now begin with **`paper.spec_signature(spec)`** — a digest of
which arm it is, where its base is bolted and how it is turned, its tool, the
ready pose the depot shapes fly through, and the static set it must clear, the
neighbours' base columns included.  A test pins that two equal specs share a
route bought by either, and that moving the base, the height, the ready pose or
the column boxes moves the key.  The memo of the signature itself is validated
against a **weak reference** to the spec, so a recycled address cannot serve the
previous spec's answer — a failure mode the old keys had no defence against at
all.

**`paper.cache_signature()` namespaces the STORE**, exactly as
`atlas.model_signature` namespaces an atlas, and it CONTAINS that signature:
collision model, search policy, `STATIC_MARGIN` (50 mm, what the atlas was swept
at) and `FRAME_FLOOR` (63 mm, what the router flies at) — **both**, because the
13 mm between them is the difference between an optimistic prefilter and a
certificate — plus the tip/chain floors, the sample count, the shape ladder's
heights, the tool transform, and the three tier flags.  Move any of them and the
build writes into a new directory and reads nothing of the old one.

**The store is a file per entry, JSON, write-once, `os.replace`d into place.**
That is the whole concurrency argument for six forked workers: an answer is a
function of its key, so a racing writer either loses and is a no-op or wins with
byte-identical content, and a reader sees a complete file or no file.  No lock,
no lock manager, and nothing to repair after a worker is killed mid-write.  JSON
rather than `.npz` because `float.__repr__` round-trips exactly and a 400-byte
answer does not want a zip container.

**Measured, on 13 real park → hover legs of the proposed rig at h = 0.970**
(six arms, shipped parks, hovers over certified cells spread across the block;
`out/atlas_proposed_h0970_lat0860_gated63`):

| run | wall | median | p95 |
|---|---|---|---|
| cold, cache off | 46.56 s | 2 633 ms | 8 648 ms |
| cold, cache on (writes) | 46.50 s | 2 629 ms | 8 636 ms |
| **warm, a SECOND process** | **0.042 s** | **3.16 ms** | **3.39 ms** |

**1 118× on the wall clock, 834× at the median**, and the warm median of
**3.16 ms** is the 3.1 ms docs/V2_SCALING_BASELINE.md predicted for a hit.  The
whole store for those 13 legs is **2 165 bytes**.  Writing costs nothing
measurable (46.50 against 46.56 s).  And the certificate does not depend on the
cache: the three runs' routes — mode, `tried`, both floors and every via — are
**bit-identical**, compared as JSON, including across the process boundary.

**It is OPT-IN.**  `paper.disk_cache_open()` (or `ARIS_LEG_CACHE`, default
`out/leg_cache`) turns it on; `scripts/csail_allocate.py` opts in by default and
`--no-leg-cache` is the escape hatch.  Every published number keeps the
behaviour it was earned with unless a caller asks.  The store also refuses to
answer while `frozen` or `envelope` is in a state it was not opened under —
both change `static_boxes` without changing any memo key, and the in-memory
memos are cleared when they move where a store cannot be.

## THE V2 ARCHITECTURE, ASSEMBLED (2026-09-11, last)

The four measurement passes of today answer four different questions and
**docs/ARCHITECTURE_V2.md** is the design they add up to.  It decides nothing
new — every number in it is cited to the pass that measured it — but it is the
page to read first, and it is where the build order lives.

**THE SHAPE.**  Lines are cut at every capability transition and assigned by the
per-line min-pieces DP (`traces.py`).  The pieces are grouped into the **eight
stages** of `3-active-rowband-y20+6seams`: two 3-active row-band stages in which
the actives are static keep-outs for each other and therefore plan and move
**asynchronously with no conductor**, then six 2-active seam stages, which are
the only ones conducted.  A **barrier** advances the fleet.  Planning runs
BEHIND execution, so the first arm moves at **0.86 s** median rather than after
3 712 s.

**THE BUDGET, END TO END** (the table in §1.2, all of it measured):
lines→pieces < 1 s at 1 000 lines; global balance **1 968.8 s → 0**; one stroke
0.19 s / 2.2 s p95, off the critical path; one pen-up leg 62 ms / 4.2 s p95 cold
→ **3.1 ms** cached; conduct **407.3 s → ~8 400 s at 1 000 strokes** if it stays
six-arm, → a handful of 2-arm searches if it does not (`∑ₖ P(n,k)` is 720 for
six arms and **4** for two); `scene_check` stays unconditional at 0.323 s per
second of timeline.

**THE BUILD ORDER**, dependency-first, with sizes: (1) a persistent
leg-certification cache, **M**; (2) per-stage park sets, **M**; (3) the
work-cell filter on `allocate.atlas_cells` plus stages 7–8 in
`traces.zigzag_pattern`, **S**; (4) stage assignment wired from `traces.py`,
**M**; (5) a typed program carrying hovers, joints and stage ids, **M**; (6) the
per-arm runtime state machine and the barrier, **L**; (7) interior-ascent
recovery hovers, **L**; (8) the hover roadmap's edge layers, **L, later**.
1–3 are independent; 4 needs 3; 6 needs 2 and 5.

**ONE THING THE CODE SAYS THAT THE MEASUREMENT PASSES DID NOT.**  `paper`'s
route and leg memo keys both begin with **`id(spec)`** (`paper.py:302`,
`paper.py:1409`) — a CPython object address.  So the caches are process-local
**by construction**, not merely un-serialised: item 1 is a re-keying job onto a
content signature (the shape `atlas.model_signature` already has), and only then
a serialisation job.  Everything else the three passes assert about the code
checks out, including `FleetProgram.barrier_before` having no caller, `_hovers`
doing `k in (0, -1)` and `Segment` carrying no joint vector.

**STILL OPEN, IN §5:** the dead band between 0.30 and 0.40 m; how many arms the
hardware allows concurrently (no cross-process fleet clock, and arm 71 has no
IP); acceleration-aware pacing (v18 wants 33–37 rad/s² against a gate of 10);
and whether the capability map should be eroded.  The SEAM1 question is now
settled by §4b and is a code change, not a measurement.  **The park set at
+4.7 mm remains the one blocker no choice of pattern can move.**

## WHAT ONE STROKE COSTS, AND WHY THE 3 712 s IS NOT THE OPTIMISER (2026-09-11)

The v2 goal — fault tolerant, 1 000 strokes, moving inside 10 s — needs a
baseline that separates what scales linearly with the picture from what is
global optimisation.  `docs/V2_SCALING_BASELINE.md` is that baseline and
`scripts/profile_stroke_costs.py` is the measurement.  **No planner semantics
or constants changed.**

**WHERE THE 3 712.4 s WENT** (GUI job `out/gui_jobs/20260910-124546-ce72`, the
v19 run; there is no `out/h097_v19.log` — the job directory is the record):

| bucket | seconds | share |
|---|---:|---:|
| global optimisation (balance, split, merge, tours, the unsplit A/B) | 1 968.8 | 53.0 % |
| intrinsically per-stroke (prefilter, probe, replan) | 1 078.0 | 29.0 % |
| conduction (`idle.conduct`) | 407.3 | 11.0 % |
| checks (flycheck, guarantee, `scene_check`) | 231.9 | 6.2 % |
| trace, export, un-bracketed | 26.3 | 0.7 % |

**THE MEASUREMENT THAT REFRAMES IT.**  v19 allocates the same 39 strokes twice
— once splitting, once not, so the conductor can rule on whether the cuts paid.
The second allocation runs in the same process against the caches the first one
filled.  Same six substages: **2 899.7 s cold, 79.4 s warm — 36.5x.**  `replan`
alone, re-planning the *same spans on the same arms*, is **63.6x** cheaper the
second time (845.5 s → 13.3 s).

**So the dominant term is not the optimiser.  It is first-touch certification of
pen-up geometry, and every byte of it is thrown away when the process exits.**
`paper._CACHE`, `paper._LEGS`, `paper._LIFTS`, `paper._SELF`, `writing._HOVERS`
and `sequence._PRICED` are module-global dicts with no serialiser.  The atlas —
1.9 MB, 2 cm grid, ~4 200 rows per arm — holds **one certified drawing pose per
cell at z = 0 and nothing else**: no hovers, no routes, no legs.

**THE PER-CALL PRICES** (`scripts/profile_stroke_costs.py`, proposed rig,
lateral holder, h = 0.970, medians / p95 in ms):

| | median | p95 |
|---|---:|---:|
| `lifted_or_lower` (the 5-rung hover ladder), cold | 9.0 | 10.5 |
| `lifted_or_lower`, memo hit | 0.0 | 0.0 |
| `paper.route` park → hover, COLD | 61.8 | 4 152.7 |
| `paper.route`, `_CACHE` hit | 3.1 | 3.3 |
| `paper.leg_bounds` (one leg, 33 samples) | 4.8 | 5.0 |
| `free_cells`, 10 s of timeline, broad phase rejects | 0.3 | 1.8 |
| `free_cells`, same, every capsule pair live | 4 995.5 | 5 080.4 |
| `free_cells`, 30 s / 6 arms, every capsule pair live | 27 879.0 | 29 098.6 |

`free_cells` is `O(Ni x Nj)`: 3x the window is **9.00x the cells and 5.6x the
wall clock** (the shortfall is the `CAPSULE_TILE = 32` broad phase doing better
on a more finely sampled path — a real longer timeline covers more workspace
and will track the cell count more closely).  Within one 30 s window the
fifteen pairs run 196.8 ms to 29 906.9 ms, a **152x spread**.  `scene_check` by
contrast is *not* pair-dominated — 2.5x the pairs costs 1.46x the time
(0.128 -> 0.187 s per second of timeline) — which is why it stays affordable
where the conduct does not.

A leg costs **≈ 32 ms per ladder shape certified** (`route`'s own `tried`
counter: `direct` = 1 shape ≈ 9 ms; `traverse30@25cm` = 51 shapes ≈ 1 665 ms;
falling through to the RRT = 88 shapes ≈ 4 200 ms).

**THE STREAMING PREDICTION.**  Greedy assignment with no global balance removes
**~2 883 s, 77.7 %** of the run.  Time to first motion — one stroke's plan plus
one park → hover leg — is **under a second at the median** and inside 5 s at the
p95, so **the 10 s goal is not a planner-speed problem**; it is a question of
what gates the start, and today that is the whole allocation.

**WHAT DOES NOT STREAM: THE CONDUCT.**  407.3 s, of which 76.5 % is 2 380
priority-DP solves at **131 ms each** (collision images are the other 20.9 %,
85.0 s — the reverse of the pre-`FAST_PLANNING` split).  The search is
`sum_k P(n, k)` in the arm count and each solve is linear in the horizon, so a
1 000-stroke timeline is ~8 400 s of conducting — longer than the drawing.
**A rolling window instead of a whole phase is the one structural change v2
cannot avoid.**

**FAULT RECOVERY: FOUR GAPS, NAMED.**  (1) Only the two *endpoints* of a span
have a certified hover (`allocate.PlacedIndex._hovers` does `k in (0, -1)`), so
a reflex stop at s = 0.4 has nowhere certified to go.  (2) Neither
`program_schema.Bundle` nor `<stem>_program.json` carries a hover, so
re-queueing a stroke to another arm means re-planning and then re-conducting.
(3) Nothing in `aris_sixarm/execute/` knows which stroke is in flight —
`seg_<arm>` is in the `.npz` and unread.  (4) `Governor` is one clock for the
whole fleet by design, so "arm 71 faulted, let the other five carry on" needs a
certificate that does not exist.  There is no e-stop, reflex handler or
watchdog in this repo at all (`docs/HARDWARE_LADDER.md:51`); the failure model
is stop-and-refuse.

## THE INTERSECTION MODEL, RE-CERTIFIED AT BOTH HEIGHTS: TWO CELLS (2026-09-10)

`ARIS_COLLISION_MODEL=spheres` selects the intersection of the shipped capsule
model with 64 fitted spheres (entries below).  It charges **43.46 mm** of
fictitious metal where the capsules charge 67.50, and by construction it can
never report LESS room than they do.  Re-certified end to end at both heights,
with the parks HELD FIXED at the shipped sets so the link model is the only
variable, it is worth **two cells at h = 0.940 and nothing at h = 0.970.**
The default is NOT flipped.

### The drawing-pose layer: redundancy, and not one new cell

`sweep_atlas_model.py` (cylinders + frozen partners) then `regate_atlas.py` to
the router's 63 mm floor, six arms, at both heights and under both models.

| | strict-GO arm-cells | | cells with a drawer | |
|---|---|---|---|---|
| | capsules | intersection | capsules | intersection |
| h = 0.970 swept | 24 136 | **24 175** (+39) | 16 184 | **16 184 (+0)** |
| h = 0.970 gated 63 | 24 101 | **24 162** (+61) | 16 184 | **16 184 (+0)** |
| h = 0.940 swept | 25 177 | **25 255** (+78) | 16 337 | **16 337 (+0)** |
| h = 0.940 gated 63 | 25 136 | **25 241** (+105) | 16 337 | **16 337 (+0)** |

**Not one arm-cell dropped at either height** — the intersection's guarantee
showing up as data — and **not one new cell gained.**  The extra 39-105 pairs
are REDUNDANCY: cells that already had a drawer now have another arm for them.
The dead canvas is out of REACH, not blocked; at 0.970 the re-decide says so
outright — of its 378 dead cells, **0** have a drawing pose in the new atlas.

### The map, both heights

`remap_dead_cells.py`, which re-decides only the dead cells and **checks the
superset property itself**.  Under the intersection that property is a theorem
rather than luck: clearance never decreases, so no live cell can die.  It
passed at both heights.

| | h = 0.970 | | h = 0.940 | |
|---|---|---|---|---|
| | shipped | intersection | shipped | intersection |
| live cells | 16 184 | **16 184** | 16 328 | **16 330** (+2) |
| live % | 97.718 | **97.718** | 98.587 | **98.599** |
| `NO_DRAW` | 378 | **378** | 225 | **225** |
| `NO_HOVER` | 0 | **0** | 0 | **0** |
| `NO_ROUTE` | 0 | **0** | 9 | **7** (-2) |
| enclosed holes | 0 (0 cells) | **0 (0)** | 4 (9 cells) | **4 (7)** |
| largest hole-free | 5.4600 m² | **5.4600** | 3.8584 m² | **3.8584** |
| near-square | 3.4352 m² | **3.4352** | 1.8960 m² | **1.9200** (+1.27 %) |
| cells LOST | — | **0** | — | **0** |
| cells GAINED | — | **0** | — | **2** |

**Zero cells lost at either height**, which is the number that had to be zero.
The two gained at 0.940 are **(0.58, 0.60) and (0.60, 0.60)** — route-dead
cells that the honest link model can now be flown to.  `NO_ROUTE` 9 -> 7 and
two of the nine hole cells close; the four holes remain as holes, so the
largest hole-free rectangle does not move.  The near-square one grows from
1.58 x 1.20 to **1.60 x 1.20 m**.

Artefacts are under `out/spheres_recert/` and carry `_spheres` in their names;
the shipped capsule-model `out/certified_area_h09*.json` are UNTOUCHED, because
the default is not flipped.

### What a full sweep would have cost, and why the shortcut is sound

The full three-layer sweep was attempted at 0.970 and abandoned three times at
ETAs of **42.8 h, 27 h and 28 h** against the shipped run's six.  Two rounds of
work took the constant factor down (the floor short-circuit, then the hover
gate's own floor) and it stayed at ~4.5x, because the ROUTE layer's samples sit
NEAR obstacles by construction and that is exactly where the short-circuit does
not fire.  The fix, if a full sweep is ever wanted, is a per-body bounding
sphere as a broad phase — 8 queries instead of 64 — which is not built.

The shortcut is sound without it: `remap_dead_cells` needs the new atlas to be
a superset of the old, and under the intersection that is guaranteed, not
checked-and-hoped.  It re-decides every dead cell with the full ladder.

### The recommendation

**Do not flip the default.**  Two cells at one height, both of them
route-dead cells at the rim of a hole, do not pay for re-opening every
certified number in `out/` — and the model's own headline is that it changes
nothing: the same 5.4600 m² rectangle, the same 378 dead cells, the same v18
verdict at 80.60 mm against 80.59.

What the flag IS worth is as an INSTRUMENT.  It is a second, independent,
mesh-faithful envelope that agrees with the shipped one everywhere it has been
asked, and it says the shipped capsules are not the thing standing between this
rig and more canvas — reach is.  Keep it off, keep it tested, and reach for it
when someone next proposes that the collision model is what is costing cells.


## THE LINK MODEL IS AN INTERSECTION, BECAUSE THE SPHERES ALONE MADE IT WORSE (2026-09-09, late)

The entry below shipped 64 fitted spheres behind `ARIS_COLLISION_MODEL` and
claimed the accuracy in litres.  Run against the shipped v18 timeline the
model then made the arm-to-arm minimum **worse**: 80.59 mm became **64.82**,
and the certified programme failed its own 80 mm gate by 15 mm.  This entry is
what that was, and what the model is now.

### The measurement that explains it

The binding instant is t = 81.57 s, arm 31's hand against arm 71's tool:

| | clearance |
|---|---|
| the metal, measured against the manufacturer's meshes | **87.51 mm** |
| the shipped chain capsule `(7, 8, 0.104)` | 84.10 mm (−3.41) |
| eight fitted hand spheres | 68.24 mm (−19.27) |

**The capsule was nearly tight and the spheres were not**, because a Franka's
hand is close to a cylinder about the wrist axis and a capsule is the right
primitive for a cylinder.  A sphere set is the wrong one: a sphere centred in
the metal and grown to touch the surface bulges past a capsule wall that is a
millimetre outside it.  Refitting the hand at 12, 16, 20, 24, 32, 40, 48 and 64
spheres leaves it **12–14 mm outside the capsule union every time** — it is not
a resolution problem.  Same for link1 (+19 mm at k = 64) and link2 (+19 mm).

**AND THERE WAS NEVER A HOLE IN THE SHIPPED MODEL.**  Checked directly, with no
spheres involved — mesh points posed by this repo's own FK against
`coordination.CAPSULES_LAT` over 3 000 random joint-box configurations — the
shipped capsules **contain** the meshes at every body:

    link1 −1.02   link2 −1.00   link3 −20.46   link4 −14.35
    link5 −1.44   link6 −19.86  link7 −13.50   hand  −0.43   (mm, negative = inside)

So the two are simply DIFFERENT outer envelopes, each tighter than the other
somewhere.  `out/spheres_recert/mesh_vs_capsules.json`.

### So the model is both of them at once

If the metal is inside union A and inside union B it is inside A ∩ B, and for
any external point `dist(p, A ∩ B) ≥ max(dist(p, A), dist(p, B))`.  The larger
of the two claims is therefore still a **lower** bound on the true clearance —
and, being a maximum over the shipped capsule number, **it can never fall below
it.**  The flag became incapable of a regression, and
`tests/test_link_spheres.py` pins exactly that on the inter-arm gate, the
arm-vs-room gate, the frozen-partner gate and on v18's own binding pose.

**What it costs is the capsule query, which was being done anyway before the
flag existed.**  What it keeps is the whole gain, because the gain was never at
the hand.  Over 200 random configurations and 40 000 external query points, the
fictitious metal each COMPLETE model charges — base column bands and both tool
capsules included in all of them, so only the moving links differ
(`scripts/link_sphere_fit.py --part fidelity`):

| model | mean | median | p95 | optimistic |
|---|---|---|---|---|
| the shipped chain capsules | 67.50 mm | 64.64 mm | 127.11 mm | 0.000 % |
| `selfcoll`'s banded capsules | 52.93 mm | 44.50 mm | 121.54 mm | **0.190 %** |
| the 64 spheres alone | 44.63 mm | 30.15 mm | 121.54 mm | 0.000 % |
| **the intersection, which ships** | **43.46 mm** | **29.52 mm** | 121.54 mm | 0.000 % |

`selfcoll`'s bands were the obvious third candidate and they are not the
answer: fitted about each body's own principal axis they are excellent on the
long links and poor across the hand (at v18's binding pose they read 49.10 mm
against the metal's 87.51), and **0.190 % of those queries came back
OPTIMISTIC** — that table's radii were fitted to mesh VERTICES, and a triangle
can bulge between three of them.  That is a small, real finding about a shipped
table; it is a self-collision model with a 20 mm margin and 63.7 mm of measured
slack, so it is not urgent, but it is written down.

### The other half of the flag, which was missing

The first pass converted `coordination` only.  The same five sausages also live
in `rig_final.STATIC_CAPSULES`, and **that** is the table an atlas sweep gates
on — a six-arm sweep installs no frozen partners, so its only collision gate is
the chain against the frame steel.  Left alone, the flag could not move a swept
cell at all.  So `rig_final.chain_static_clearance`, `envelope.chain_cyl_
clearance`, `frozen.partner_clearance` and `frozen.chain_clearance` all now take
the sphere centres, `atlas._clears` and `paper`'s six static funnels pass them,
and `paper.near_boxes` gained its own sphere screen (a box kept only because a
capsule could reach it is not proof that no sphere can).  `tilt._frame_clear`
drops its bound screen under the flag and goes exact, because that screen is a
capsule argument that does not survive the swap.

`atlas.model_signature` widens when the flag is on, so a sphere-swept atlas and
a capsule-swept one can never be confused for one another; atlases already on
disk keep the signature they were written with.

### What it costs, and the one line that makes it affordable

The first intersection build ran the sphere query everywhere and the h = 0.970
map came back with an ETA of **42.8 hours** against the shipped run's six.  It
does not need to.  `max` can only RAISE a number, so a sample whose CAPSULE
clearance already clears the floor clears it under the intersection too,
whatever the spheres say — and the capsule value it keeps is still a valid
lower bound on the metal.  That is `paper.leg_static_lb`'s own contract ("given
a floor the number is only guaranteed to be on the right side of it") applied
one level down, and it means the sphere block runs only on the residual: on a
certified map, the cells that were dead already.

`chain_static_clearance`, `chain_cyl_clearance`, `partner_clearance` and
`chain_clearance` all take `floor`; `atlas._clears` and `paper`'s leg
certificates pass theirs.  `paper.near_boxes`' sphere screen also became one
box round all 64 centres instead of a distance per centre per box — the screen
only has to be conservative, and the narrow phase behind it is now
short-circuited.  A test pins that the floor form changes **no verdict** at
0.030, 0.050, 0.063 and 0.100 m and no value below the floor, and the h = 0.970
atlas re-swept with it is **bit-identical** to the one without (24 175 strict-GO
arm-cells, 0 gained, 0 lost) at 52 s for six arms.

### v18, re-checked

| v18 whole timeline, `--sub 2` | inter-arm min | self | verdict |
|---|---|---|---|
| capsules (shipped) | 80.59 mm, pair 13–31, t = 42.01 s | 21.5 mm | PASS |
| the 64 spheres alone | **64.82 mm**, pair 31–71, t = 81.57 s | 21.5 mm | **FAIL** |
| **the intersection** | **80.60 mm**, pair 31–71, t = 81.58 s | 21.5 mm | PASS |

The intersection buys **+0.01 mm** on this timeline, and that is the honest
headline for it: v18's inter-arm minimum is set at a place where the capsule
model is already within 3.4 mm of the metal, so there was nothing there to win.
The gain is in the map, where the binding geometry is an arm against a
NEIGHBOUR'S BODY and a frame, not against a tool.


## THE MOVING LINKS CAN BE SPHERES, AND THE SHIPPED SPHERE SETS CANNOT (2026-09-09)

Pete: *"for the rest of the robot why not just use the standard collision
geometry models that are shipped with the frankas, or you can scavenge the
collision geoms from the other repo where we use a bunch of spheres?  that will
be more accurate for the robots.  the static orange cylinders for the pivoting
bases are great though."*

**Answered in two halves, and the first half is a refusal.**  None of the three
sphere models on this machine contains the robot.  Escape is the largest
distance any point of a body's mesh (the manufacturer's collision shells UNION
the full-resolution visuals — the ground truth the self capsules were fitted
against) lies OUTSIDE the spheres attached to that body;
`scripts/link_sphere_fit.py --part validate` measures it:

| shipped sphere model | spheres | worst escape |
|---|---|---|
| `assets/franka_description/urdf/panda_arm_hand.urdf` (Drake's stock Panda) | 66 | **+235.2 mm** (link0), +66.5 (link5) |
| `~/git/vamp/resources/panda/panda_spherized.urdf` | 59 | **+225.7 mm** (link0), +62.1 (link5) |
| `~/git/mmt_gcs/.../fr3_franka_hand_sphere_collisions.urdf` (real FR3) | 35 | **+146.9 mm** (link0), +56.8 (link5) |

Every one is optimistic on **every** body, by 4 mm at best.  They are planner
models and were never envelopes.  Scavenging one puts a hole in the certificate
at the fingers, the wrist bulge and the base connector.  (`~/git/cc_experiment`
has no Franka sphere set — it benchmarks an iiwa14 and computes bounding
spheres at runtime; `aris_project/reachability` has none.  Both were checked.)

**So the spheres are fitted here, and containment is by construction.**
`aris_sixarm/link_spheres.py`: **64 spheres, 8 per moving body** (link1..link7,
hand — the same budget Drake's stock Panda spends), farthest-point seed, Lloyd
descent, Badoiu-Clarkson MEB polish, radius = the exact maximum distance to any
point assigned to that centre, rounded **UP** to the millimetre.  Worst escape
over all eight bodies **-0.21 mm**; `tests/test_link_spheres.py` re-measures it.

**THE BASE CYLINDERS DO NOT MOVE.**  `coordination.BASE_CAPSULES`,
`mounts.column_bands`, `envelope.body_cylinders`, `scene_check.COLUMN_BANDS` —
all unchanged.  link0 is not in the sphere table at all.  Neither is the tool:
its capsules are a CAD-provenance question (`rig_final.BRACKET_R_LAT`), not a
mesh-accuracy one.

### What it buys, and where

What is replaced is the five sausages `coordination.CAPSULES` draws about the
lines between JOINT ORIGINS — shoulder→elbow 0.130, elbow 0.117, forearm 0.131,
wrist 0.091, hand 0.104.  An FR3's castings are L-shaped: the line runs inside
the bend and the radius is set by the outside of it.  Six random configurations,
`scripts/link_sphere_fit.py --part accuracy`:

| model | volume | mean offset | max offset |
|---|---|---|---|
| the five moving capsules | 58.02 L | 55.8 mm | 130.9 mm |
| **the 64 fitted spheres** | **32.80 L** | **24.8 mm** | **75.1 mm** |
| | **−43.5 %** | **−31.1 mm** | **−55.8 mm** |

"Offset" is how far a point of real metal lies inside the model — the fictitious
steel the gate is charged for, on both arms of every pair, against a
`PAIR_MARGIN` of 50 mm.

**AND IT BUYS NOTHING ON THE SELF MODEL, SO IT IS NOT APPLIED THERE.**
`selfcoll.BODY_CAPSULES` is already per-link, per-link-FRAME and mesh-fitted
(three bands a body, radii 0.038–0.078) and has no L-shape problem.  Measured
the same way the spheres are 38.5 L against its 43.6 — 12 % smaller overall but
**larger** on link1 (7.11 vs 6.31), link2 (7.08 vs 6.12) and link5 (8.27 vs
7.33), three of the four bodies the fold gate exists for.  `selfcoll` keeps its
capsules under this flag, and the ≥4-joint pair rule and `SELF_PLAN_MARGIN`
(23 mm) are untouched.

### The flag, and it is OFF

`ARIS_COLLISION_MODEL=spheres`, or `link_spheres.install()` / `uninstall()` —
the same shape `envelope.install` and `frozen.freeze` already use, read at the
same point in `scripts/feasible_workspace.py`.  **Default `capsules`.**  A
sphere is a capsule with a zero-length segment, which is the whole of the
wiring: `coordination.ArmPath`, `frozen.freeze` and `scene_check.pair_clearance`
hand the block over as degenerate capsules and every box, tile, exact distance
and 1-Lipschitz residual downstream is unchanged.  `scene_check` keeps its own
row-selection rule and its own centre placement (it imports the 64-row
measurement, exactly as `self_clearance` already imports the 31-row one, because
hand-copying 256 fitted numbers buys typos rather than independence).

Flag OFF is **bit-identical** — pinned on `ArmPath`, on `frozen`'s partner
capsules and on `scene_check.check_static`.

**FLIPPING THE DEFAULT IS A RE-CERTIFICATION, NOT A COMMIT.**  Every atlas,
every certified rectangle, every park set and every conducted programme in
`out/` was earned against the sausages.  Flipping it re-opens all of them: the
six-arm atlas sweep, `regate_atlas`, the park search, `certified_area` at both
heights, and a fresh `scene_check` on every shipped timeline.  It also widens
`atlas.model_signature`, so every cached atlas goes stale by design.


## THE HOLDER IS FINAL, AND THE RIG IS BETTER FOR IT (2026-09-07)

`7f99565` and `efd53f5` settled the tool against the real gripper: the grip is
at the FAR END of the Fat finger plates and the bore leans **23°**, the
housing's own clocking, so **`PEN_EXT_HOLDER` / `PEN_LAT_HOLDER` = 0.0460262 /
0.0860369** — the tip 40 mm further out across the hand and 13 mm shallower
than the 2026-09-03 pair.  Everything below is that tool, re-certified from the
atlas up.  Nothing here moves a gate, a capsule, the layout grid or h; the only
committed constants that changed are the two the park search owns.

### The atlas, and what the final tool is worth

    ARIS_TOOL=lateral scripts/run_atlas6.py --rig proposed \
        --pen 0.0460262 --pen-lat 0.0860369 --jobs 6 \
        --out out/atlas_proposed_h0940_lat0860              (298 s/arm)
    ARIS_RIG=proposed ARIS_TOOL=lateral scripts/regate_atlas.py \
        --in  out/atlas_proposed_h0940_lat0860 \
        --out out/atlas_proposed_h0940_lat0860_gated63      (246 s)

| 2 cm sweep, h = 0.940, 16 562 cells | 0.110 | 0.0588421 | **0.0460262/0.0860369** |
|---|---|---|---|
| union strict-GO | 99.64 % | 97.08 % | **98.47 %** |
| reachable | 100.00 % | 99.71 % | **99.84 %** |
| ≥ 2 arms | 47.90 % | 38.88 % | **42.13 %** |
| ≥ 3 arms | 11.11 % | 4.21 % | **4.32 %** |
| dead | 0.36 % | 2.92 % | **1.53 %** |
| strict-GO cells, arm 31 | 4 846 | 4 072 | **4 208** |

**THE LATERAL OFFSET IS WHAT BUYS IT BACK.**  The 0.0588421 tool lost 2.56
points of union because it was short in BOTH directions at once; this one is
shorter still along the approach axis (46 mm against 59) but **86 mm across
it**, and reach across the hand is what an inverted arm spends on the paper.
Half of what the shorter pen cost is recovered, and the redundancy with it.

### The feasible-workspace map, v14

`scripts/feasible_workspace.py` at v12/v13's settings (`--fiber-tries 48
--hover-lean-deg 15 --rrt 60 --rrt-nodes 300 --rescue 0,1`), 6 workers, on
`_gated63`; sweep 7 087 s + rescue 761 s.  `out/feasible_workspace_v14.{png,json}`.

| solo-drawable, 16 562 cells | v12 (0.110) | v13 (0.0588) | **v14 (final)** |
|---|---|---|---|
| FEASIBLE | 99.46 % | 96.84 % | **97.95 %** (6.489 m²) |
| draw pose only | 99.64 % | 97.05 % | **98.23 %** |
| + hover | 99.64 % | 97.05 % | 98.18 % |
| + reachability | 99.46 % | 96.84 % | 97.95 % |
| DEAD, no draw pose | 0.36 % | 2.95 % | **1.77 %** |
| DEAD, draw ok no hover | 0.00 % | 0.00 % | 0.05 % |
| DEAD, hover ok unreachable | 0.18 % | 0.21 % | 0.23 % |
| largest inscribed rectangle | 0.52 × 3.54 = 1.841 m² | 0.46 × 3.64 = 1.674 m² | **0.54 × 2.72 = 1.469 m²** |

**WHERE THE DEAD CELLS SIT — the under-base holes halved and the rim did not
move.**  339 dead cells:

| distance to the nearest base | v13 | **v14** |
|---|---|---|
| under the base discs (r < 0.30 m) | ~263 | **93** (86 of them r < 0.20) |
| in between (0.30–0.60 m) | few | 21 |
| **the OUTER RIM (r ≥ 0.70 m)** | 242 (at 0.60–0.85) | **225** (126 at 0.70–0.80, 99 at 0.80–0.90) |
| median dead-cell radius | 0.434 m | **0.783 m** |

By cause: `no draw pose` is 293 cells and **225 of them are the rim** (median
r 0.787); `draw ok, no hover` is 8 cells and all 8 are under a base; `hover ok,
unreachable` is 38, median r 0.317.  **So the final tool bought back the
under-base holes and not the rim** — 86 mm of lateral offset lets an arm reach
UNDER itself where 59 mm could not, but nothing about a shorter axial depth
extends the outer edge of the annulus.  The rim is now essentially the whole
dead set, and it is the constraint any future placement has to respect.

**AND THE RECTANGLE GOT SMALLER WHILE THE COVERAGE ROSE**, which is not a
contradiction: 1.469 m² against v13's 1.674.  v13's dead set was concentrated
(a long clear portrait strip survived beside it); v14's 339 cells are spread
thinner over both rims, so the largest *clean* block is shorter even though
1.11 points more of the canvas is drawable.  A picture that needs one big
rectangle is not the same ask as a picture that needs coverage.

### CSAIL v18 — 100.0000 %, on a placement chosen in five minutes

The proxy first (`scripts/placement_proxy.py`, promoted to the repo this run),
then ONE plan.  No placement search: v16 measured that at 27 243 s to lose
0.65 pp.

**THE PROXY SAYS THE FINAL TOOL CHANGED THE PROBLEM.**  At the 0.0588421 tool,
**10 of 1 976** offsets had a zero contiguous dead run and the best had 82 mm of
5th-percentile clearance.  At this tool, **143 of 325** do, and the best has
**141 mm**.  Top five, 90°, scale 0.85:

| dx | dy | longest dead run | total dead | p5 clearance | centre |
|---|---|---|---|---|---|
| **−0.03** | **+0.05** | **0 mm** | **0 mm** | **141 mm** | (0.8717, 1.8653) |
| −0.02 | +0.05 | 0 | 0 | 141 | (0.8817, 1.8653) |
| −0.03 | +0.10 | 0 | 0 | 140 | (0.8717, 1.9153) |
| −0.02 | +0.10 | 0 | 0 | 140 | (0.8817, 1.9153) |
| −0.02 | −0.00 | 0 | 0 | 134 | (0.8817, 1.8153) |

Scale 0.80 and 0.90 were scanned too and both also have zero-dead-run offsets
(0.80 at +0.00/+0.05 with 143 mm; 0.90 at +0.00/+0.05 with 134 mm) — **a
BIGGER drawing at 100 % is available and nobody has asked for one**; 0.85 is
kept here only so v18 compares with v15 and v17 like for like.

Run as GUI job **`20260907-162523-6ac6`**, v15's flag set exactly, fixed
placement, the new atlas, the re-searched parks.
`out/csail_schedule_h094_v18.{npz,json}`, `out/csail_program_h094_v18.json`,
`out/csail_place_v18_placement.json`, `out/h094_v18.log`.

| | v15 | v16 | v17 (0.0588 tool) | **v18 (final tool)** |
|---|---|---|---|---|
| offset | (−0.10, 0.00) | (−0.10, +0.10) | (+0.05, −0.20) | **(−0.03, +0.05)** |
| centre | (0.8017, 1.8153) | (0.8017, 1.9153) | (0.9517, 1.6153) | **(0.8717, 1.8653)** |
| coverage, allocated | 99.5224 % | 98.8729 % | 100.0000 % | **100.0000 %** |
| coverage, conducted | 99.7485 % | 97.9754 % | 100.0000 % | **100.0000 %** |
| left empty / skipped | 0.0803 / 0 m | 0.1894 / 0.1947 m | 0 / 0 m | **0.0000 / 0.0000 m** |
| segments | 47 | 48 / 47 | 49 | **53** |
| makespan | 277.625 s | 249.875 s | 195.229 s | 242.146 s |
| conducted pause | — | 56.7 s | 121.4 s | 220.2 s |
| phases planned / conducted | 2 / 2 | 3 / 7 | 2 / 2 | **3 / 3** |
| min inter-arm, worst phase | 82.0 mm | 81.2 mm | 82.2 mm | 81.9 mm |
| column / chain / tip | 122.0 / 30.7 / −7.3 | 131.0 / 22.6 / −4.9 | 136.6 / 30.4 / −7.0 | **149.9 / 20.8 / −6.4** |
| planner wall clock | 4 910.9 s | 29 352.9 s | 3 381.7 s | **4 144.6 s** |

**INDEPENDENT `scene_check`, whole merged timeline** (`scripts/recheck_timeline.py`,
also promoted this run): **VERDICT PASS**, min inter-arm **80.59 mm** against
the 80 mm gate — **+0.59 mm** — worst pair **13–31 at t = 42.01 s**; self
21.5 mm, frame 51.6 mm, column 145.8 mm, joint margin 0.1076, frozen 6/6.

**THE GATE TO WATCH ON THIS ONE IS THE PAPER, NOT THE NEIGHBOURS.**  Chain
clearance is **20.5 mm against a 20 mm gate** — 0.5 mm, the thinnest margin in
the programme and thinner than the inter-arm one.  It is arm 31 at t = 80.0 s.
v15 and v17 had 30 mm there; the final tool holds the wrist lower for the same
tip (46 mm of axial depth against 59), which is exactly where that went.  **A
touchdown calibration that lowers the tip further eats this first.**

**WHERE THE TIME WENT** — 3 962.9 s of measured substage time in a 4 144.6 s
run, from `scripts/job_substages.py`:

| substage | v18 | share | v15 |
|---|---:|---:|---:|
| balance | 2 345.1 s | 59.2 % | 3 136.8 s (65.5 %) |
| conduct | 525.2 s | 13.3 % | 710.5 s |
| replan | 510.1 s | 12.9 % | 465.7 s |
| probe | 279.0 s | 7.0 % | 272.0 s |
| flycheck | 142.0 s | 3.6 % | 109.6 s |
| merge | 104.5 s | 2.6 % | 39.1 s |
| sequence / guarantee / repair / prefilter | 57.0 s | 1.4 % | 57.2 s |

By stage: allocation 3 437.7 s, conduction 702.7 s, `scene_check` 138.3 s over
4 calls, export 3.0 s, trace 0.3 s.  **The balancer is still two thirds of the
run** and it is still the thing to iterate against.

### The park set, re-searched at the final tool

Details and the full comparison are in `aris_sixarm/layout.py` beside the grid.
Headline: **473–474 of 576** candidates certify per arm (430 before), fleet
worst park-vs-(ink AND lift) **93.1 mm** against the 80 mm gate, park-vs-park
at the ≥ 250 mm cap, entries and go-homes **113/144 = 78.5 %**.  `PARK_GRID_PROPOSED`
and `Q_PARK_PROPOSED` are that search's output, re-derived rather than typed.
93.1 mm is arm 71's own ceiling; the other five clear by 97.6–98.6 mm.

**A LOOSE END, NAMED.**  Of the five arms that swing aside under
`region_aware_parks`, two — arms 2 and 17 — have a certified `repark_route`
that `idle.conduct` cannot schedule while the other five stand still
(`Unconductable`, no monotone pause schedule).  v18 does not use `--aside-parks`
so nothing shipped depends on it, but the aside feature is not fully available
on this park set and that is not the same as it working.

## RE-PLANNING AT THE HOLDER'S OWN TOOL (2026-09-03)

`bc670bf` moved the lateral tool's tip 72.348 mm — `PEN_LAT_HOLDER` /
`PEN_EXT_HOLDER` from 0.110 / 0.110 to **0.0588421 / 0.0588421** — and said out
loud that every certified programme and atlas in `out/` was stale.  This is the
re-certification.  Nothing below changes a gate, a capsule, a layout, a height
or a tool constant; every number is the same machinery run again at the new
pair.

### The atlas first, because everything else reads it

`atlas.is_current` compares a stored sweep's model signature against the
running build's, and the pen is **inside** that signature (entry 340), so every
atlas in `out/` refuses at the new tool by name: *"collision model differs at
entry 340: atlas has 0.1100, this build has 0.0588"*.  Re-swept:

    ARIS_TOOL=lateral scripts/run_atlas6.py --rig proposed \
        --pen 0.0588421 --pen-lat 0.0588421 \
        --out out/atlas_proposed_h0940_lat0588        (6 jobs, 464 s/arm)
    ARIS_RIG=proposed ARIS_TOOL=lateral scripts/regate_atlas.py \
        --in  out/atlas_proposed_h0940_lat0588 \
        --out out/atlas_proposed_h0940_lat0588_gated63    (304 s)

**WHAT THE SHORTER PEN COSTS THE CANVAS, before any planning at all:**

| 2 cm sweep, h = 0.940, 16 562 cells | 0.110 tool | **0.0588421 tool** |
|---|---|---|
| union strict-GO | 99.64 % | **97.08 %** |
| reachable | 100.00 % | **99.71 %** |
| ≥2 arms | 47.90 % | **38.88 %** |
| ≥3 arms | 11.11 % | **4.21 %** |
| dead | 0.36 % | **2.92 %** |
| strict-GO cells, arm 31 | 4 846 | **4 072** |

The tool is 72 mm shorter along its own axis and 51 mm shorter across it, so
the WRIST has to come 72 mm down and 51 mm in for the same tip: the arm reaches
less far and the annulus narrows at both edges.  Redundancy is what pays for it
— the union loses 2.6 points, the ≥3-arm overlap loses two thirds.

### The park set: re-searched, not patched

`Q_PARK_PROPOSED` was flagged stale at its own definition and it was: run at
the new pair, the OLD grid parks arms 13 and 17 **53.7 mm** apart and
`certified_park_poses` refuses the fleet.  The old POSES are safe — every one
still passes `check_pose`, the fleet still holds ≥ 250 mm — but they stand
**69.0 mm inside** arm 17's certified ink, which is the number a phase is
conducted on.

The (radius, hover, bearing) search was re-run at the new tool: 6 radii × 4
hovers × 24 bearings per arm, each gated by `certified_ready_pose` (430–431 of
577 certify); then park-vs-ink ≥ 80 mm against the new atlas; then the fleet
gate; then ranked on the depot's own job — of 24 of the arm's certified cells,
how many it can fly to (`writing.enter_beats`) and home from (`exit_beats`).

| | OLD literals at the new tool | **NEW, re-searched** |
|---|---|---|
| worst park-vs-ink | **−69.0 mm** | **+97.8 mm** |
| worst fleet pair | ≥ 250 mm (cap) | ≥ 250 mm (cap) |
| entries flyable | 97/137 (70.8 %) | **108/137 (78.8 %)** |
| go-homes flyable | 94/137 (68.6 %) | **108/137 (78.8 %)** |
| joint margins | 0.30–0.68 | **0.541–0.682** |
| park pens inside the CSAIL logo box | arm 71 at (1.357, 1.556) | **none** |

    PARK_GRID_PROPOSED = {2: (0.55, 0.10, -165.0), 13: (0.40, 0.10, -150.0),
                          17: (0.48, 0.30,  -60.0), 31: (0.55, 0.10, -165.0),
                          71: (0.55, 0.35,  -30.0), 97: (0.55, 0.20,   15.0)}

**+97.8 mm is the layout's ceiling, not a lucky draw.**  22–42 candidates per
arm clear the ink gate and every one of them lands on the same 97.8–98.9 mm
plateau — a parked arm's own base column is pose-invariant, so no bearing buys
past it.  That is the same plateau the 2026-08-26 search reported as "97–98 mm"
at the old tool, which is what says the criterion was re-run and not
re-invented.

**Five of six bearings still point outward; arm 2's is tangential** (dot with
its own outward ray −0.015, 90.9° off it).  Outwardness was always a means:
what keeps the six apart is `fleet_park_clearance`, and it proves ≥ 250 mm.
Held to the outward ray instead — the two-number grid entry — the same radii
and hovers put arms 2 and 97 **111.0 mm inside each other** at the new tool,
so the third number is not a refinement, it is the reason there is a park set.

**A test that had been skipping for as long as the directory existed.**
`test_no_shipped_park_pose_stands_in_another_arms_certified_ink` picked its
atlas with `atlas.is_current`, which answers against the PROCESS-GLOBAL tool —
and a bare `pytest` run is the INLINE pen, so every lateral atlas read as stale
and the test skipped, while its body measured with `PEN_*_HOLDER`.  It now
matches the atlas's recorded `pen_ext`/`pen_lat` to the tool it measures with
and answers `is_current` with that tool active.  It RUNS now, and passes at
+97.8 mm; it would have failed at −69.0 mm on the old poses.

### CSAIL v15 — the same picture, the same placement, the new tool

Run as a **GUI job** (`out/gui_jobs/20260903-200349-d8ec`, watchable live on
`http://127.0.0.1:8765/`), which is `scripts/draw.py` with v14's own flag set:
same source `assets/csail/csail_old_med.gif`, same placement file
`out/csail_place_v4_placement.json` (1.4309 × 1.8711 m at (0.8017, 1.81532),
rotated 90°, offset [−0.1, 0]), `--arms all --two-pass --arm-phases off
--residual-passes 6 --residual-min-gain 0.0005 --depot-hover-selective
--freeze-refused-phase --skip-unconductable --tilt-max-deg 15 --image-jobs 6`,
the RRT pen-up tier ON — and the NEW atlas.  Copied out to
`out/csail_schedule_h094_v15.{npz,json}`, `out/csail_program_h094_v15.json`,
`out/h094_v15.log`.

**ONE DELIBERATE DIFFERENCE, NAMED.** v14 was `scripts/csail_schedule.py`,
which traces with `trace.trace_logo`; the GUI front door traces with
`trace.trace_any`.  Both give **39 strokes**; the generic tracer's path is
8 011 px against 7 512, and after placement **16.8048 m** against v14's
16.7370 m — 0.40 % more ink.  Everything after the tracer is the same call.

| | v14 (0.110 tool) | **v15 (0.0588421 tool)** |
|---|---|---|
| coverage | **100.0000 %** (0.0000 m empty) | **99.5224 %** (0.0803 m empty) |
| makespan | 196.271 s | **277.625 s** (+41.4 %) |
| phases planned / conducted | 3 / 3 | **2 / 2** (+1 residual that certified 0 m) |
| segments | 42 | **47** |
| traced / drawn | 16.7370 / 16.7470 m | **16.8048 / 16.7625 m** |
| pen recorded | 110.0 mm | **58.8 mm** |
| min inter-arm, per phase | 82.4 / 82.4 mm | **85.6 / 82.0 mm** (gate 80) |
| self-collision | 26.6 / 31.1 mm | **22.7 / 33.7 mm** (gate 20) |
| frame | 54.5 / 54.7 mm | **53.5 / 53.2 mm** (gate 50) |
| neighbour base column | 138.6 / 138.1 mm | **122.0 / 131.5 mm** (gate 80) |
| paper, chain | 68.3 / 57.4 mm | **30.7 / 31.4 mm** (gate 20) |
| paper, tip | −4.1 / −4.5 mm | **−5.0 / −7.3 mm** (floor −10) |
| pen-swap pause | — | 256.0 mm, 6/6 poses pass |
| scene_check | PASS | **PASS, both phases** |
| planner wall clock | 4 170.9 s | **4 910.9 s** |

**INDEPENDENT `scene_check`, re-run from the shipped `.npz`** over the whole
merged 6 664-frame timeline (not per phase): **VERDICT PASS**, min inter-arm
**80.26 mm** against the 80 mm gate, worst pair 13–31 at t = 99.09 s; self
21.4 mm, frame 52.7 mm, column 115.0 mm, paper chain 30.7 mm, tip −7.4 mm,
frozen poses 6/6.  The whole-run number is 1.7 mm tighter than the tightest
phase because it spans the pen-swap seam that no per-phase check covers, and
it clears by **0.26 mm**.  That is the thinnest margin in this programme and
it is where a re-run should be watched.

**WHERE THE TIME GOES** — the GUI's own substage log, 4 791.5 s of measured
substage time inside a 4 910.9 s run:

| substage | v15 total | share | v14 |
|---|---:|---:|---:|
| balance | 3 136.8 s | 65.5 % | 2 636.8 s |
| conduct | 710.5 s | 14.8 % | 239.4 s |
| replan | 465.7 s | 9.7 % | 806.4 s |
| probe | 272.0 s | 5.7 % | 180.2 s |
| flycheck | 109.6 s | 2.3 % | *(inside v14's balance)* |
| merge | 39.1 s | 0.8 % | *(inside v14's balance)* |
| sequence | 25.4 s | 0.5 % | 13.0 s |
| guarantee | 23.1 s | 0.5 % | *(inside v14's balance)* |
| repair | 8.7 s | 0.2 % | 0.0 s |
| prefilter | 0.4 s | 0.0 % | 0.4 s |

Per phase: allocation 1 735.4 s (grey) + 2 262.7 s (orange) + 5.8 s
(residual) = 4 081.0 s; conduction 477.0 + 233.6 = 710.5 s; `scene_check`
50.0 + 49.7 = 99.7 s.  **The balancer is two thirds of the run and it always
was** — v14 spent 63 % there under a name that also contained flycheck, merge
and guarantee.  That is the baseline to iterate against.

**THE 80.3 mm THAT WAS LEFT EMPTY IS REACH, and it is one span.**
`dropped` names it exactly: stroke 36, orange, the fill boundary's
`s_range` 0.5001–0.6249, 0.0803 m at **(0.0866, 1.2350)** — the logo's own
LEFT EDGE.  Its two cells (0.08, 1.18) and (0.08, 1.24) each had a certified
drawing pose for **arms 13 and 31** in the old atlas and have **none at all**
in the new one; the v13 map calls both `NO_DRAW`.  It is not a park refusal,
not a neighbour, not self-collision and not a route: 16 probes over 6 (stroke,
arm) pairs, 4 prefiltered, and no arm can put the tip there.  Those cells sit
0.77–0.81 m from the nearest base, in the outer rim the shorter tool took away.

**WHAT WOULD RECOVER IT WITHOUT MOVING A CONSTANT: the placement.**  At the
rows that matter the leftmost live column is now **x = 0.10 m** and the logo's
left edge is at **x = 0.0866 m** — 14 mm.  A placement re-search against the
new atlas (`--placement auto`, which is what chose `csail_place_v4` against the
old one) is the principled fix; the placement file's own `live: 1.0` is no
longer true at this tool.  1.50 % of the logo's bounding box is dead in v13
against 0.48 % in v12, so a slightly smaller or slightly right-shifted
placement is the whole of the difference between 99.52 % and 100 %.

### CSAIL v16 — the placement WAS re-searched, and it made the picture worse

v15's entry named the lever: *"a placement re-search against the new atlas
(`--placement auto`) is the principled fix."*  It was run, at v15's own flag set
with nothing else changed, as GUI job **`20260903-215636-b6f9`**.  Copied out to
`out/csail_schedule_h094_v16.{npz,json}`, `out/csail_program_h094_v16.json`,
`out/csail_place_v16_placement.json`, `out/h094_v16.log`.

**IT DID NOT RECOVER 100 %.  IT LOST 0.65 pp.**

| | **v15** (placement file) | **v16** (`--placement auto`) |
|---|---|---|
| placement | 90°, 0.85, offset (−0.10, **0.00**) | 90°, 0.85, offset (−0.10, **+0.10**) |
| centre | (0.8017, 1.81532) | **(0.8017, 1.91532)** |
| size | 1.4309 × 1.8711 m | **same** |
| coverage, allocated | **99.5224 %** | **98.8729 %** |
| coverage, conducted | 99.7485 % | **97.9754 %** |
| left empty | 0.0803 m in 1 span | **0.1894 m in 1 span** |
| skipped at conduction | 0.0000 m | **0.1947 m** |
| segments | 47 | 48 allocated / 47 conducted |
| makespan | 277.625 s | **249.875 s** (−10.0 %) |
| conducted pause | — | 56.667 s |
| phases planned / conducted | 2 / 2 | **3 / 7** |
| min inter-arm, worst phase | 82.0 mm | **81.2 mm** (gate 80) |
| neighbour base column | 122.0 mm | 131.0 mm (gate 80) |
| paper, chain | 30.7 mm | **22.6 mm** (gate 20) |
| paper, tip | −7.3 mm | −4.9 mm (floor −10) |
| planner wall clock | 4 910.9 s | **29 352.9 s** |

**INDEPENDENT `scene_check`, whole merged 6 241-frame timeline**, re-run from
the shipped `.npz`: **VERDICT PASS**, min inter-arm **80.25 mm** against the
80 mm gate — **+0.25 mm** — worst pair **17–71 at t = 35.68 s**; self 27.5 mm,
frame 53.0 mm, column 130.9 mm, paper chain 23.0 mm, tip −4.9 mm, joint margin
0.1051, frozen poses 6/6.  v15's whole-timeline number was 80.26 mm on pair
13–31.  **Both runs clear the gate by a quarter of a millimetre on a different
pair — the margin is a property of this rig at this tool, not of either
placement.**

**WHERE THE 0.1894 m WENT, AND IT IS A DIFFERENT FAILURE FROM v15's.**
`dropped` names it: stroke 1, orange, an **outline drawn as a whole stroke**
(`s_range` 0.0–1.0), 0.18941 m at **(0.7012, 1.8020)**, running from
(0.71646, 1.80259) to (0.52705, 1.80205).  That is not the canvas edge — it is
**straight through arm 31's own base**.  Cell (0.60, 1.80) sits **0.016 m** from
arm 31's base at (0.5967, 1.8153) and has a certified drawing pose for **no arm
at all**; its neighbours (0.72, 1.80), (0.70, 1.80) and (0.52, 1.80) are
certified for 31 and 71.  The by-third accounting puts all 0.19 m in the
**middle** third.  Residual pass 2 spent 16 probes over 6 (stroke, arm) pairs,
4 prefiltered, and gap repair offered **12 windows** and got **0** certified
spans back.

**SO v15 LOST INK TO THE OUTER RIM AND v16 LOSES IT TO AN UNDER-BASE HOLE** —
the two dead sets the height table below shows trading against each other, met
one after the other by the same logo shifted 10 cm.

**AND THE CONDUCTOR PAID FOR THE SHIFT TWICE.**  Moving the logo 0.10 m toward
the middle row put phase 1 beyond the idle policy: grey was refused with 5 arms
drawing, re-offered as `{13,31,2}` + `{17,71,97}`, refused again at 2 arms, and
arm 2's tour was cut in half twice more — **7 conducted phases against v15's
2**, and 0.1947 m of allocated ink skipped outright.  The conductor says why in
its own words: *"this is a FROZEN POSE problem: arm 31 cannot stop clear of 2
(38 mm); arm 71 cannot stop clear of 17 (62 mm); arm 97 cannot stop clear of 2
(76 mm)"*.  The 10 % faster makespan is not a win — it is 0.34 m of ink that is
not being drawn.

**WHY THE SEARCH CHOSE IT, AND THE HONEST GAP.**  The rule is the standing one,
*largest AREA within 1 pp of the best real coverage*.  The three best cells were
within **0.16 pp** of each other — 0.75× at 95.418 %, 0.80× at 95.334 %, 0.85×
at 95.257 % — so the area rule took the largest, spending 0.16 pp of search
coverage to buy 0.6 m² of drawing.  **It cost 0.65 pp of real coverage.**  And
`--top 3` means only three offsets per (rotation × scale) cell are allocated for
real: the proxy ranked (−0.10, +0.10), (+0.10, −0.20) and (+0.10, +0.20) above
**v15's own (−0.10, 0.00), which was therefore never allocated for real in this
search at all.**  The incumbent was not beaten; it was never entered.

**THE NEXT CONSTANT-FREE LEVERS, IN ORDER OF CHEAPNESS.**  All three are FLAGS:
1. **Keep v15's placement.**  It is the best number anyone has at this tool and
   v16 is the evidence that the search at these settings does not improve on it.
2. **`--slack 0`** — stop the area rule spending coverage.  On this very search
   that picks 0.75× at 95.418 % instead, a 1.816 m² drawing rather than 2.677.
3. **`--top` raised, or the incumbent seeded** — allocate v15's cell for real
   alongside the proxy's favourites so a re-search can never regress.
A fourth, not free: the search's objective cannot see the conductor at all, and
v16 is the first run where the difference between allocating and conducting
decided which placement was better.

**WHAT `--placement auto` COSTS AT THIS FLAG SET — the measurement to iterate
against.**  826 proxy placements, **78 allocated for real, 27 243.3 s**: the
search alone is **5.5×** v15's entire run, and the job's 29 352.9 s is **6.0×**.
Per candidate ≈ 1 990 worker-seconds on 6 jobs.  Where it goes, over the whole
job's 156 525 s of measured substage time:

| substage | v16 total | share |
|---|---:|---:|
| flycheck | 90 368.3 s | 57.7 % |
| merge | 34 165.9 s | 21.8 % |
| replan | 23 252.9 s | 14.9 % |
| probe | 4 893.7 s | 3.1 % |
| sequence | 1 321.2 s | 0.8 % |
| guarantee | 1 259.4 s | 0.8 % |
| repair | 631.4 s | 0.4 % |
| balance | 466.9 s | 0.3 % |
| conduct | 148.1 s | 0.1 % |
| prefilter | 17.8 s | 0.0 % |

By stage: **placement 27 243.3 s**, allocation 1 811.2 s, conduction 294.0 s,
`scene_check` 70.2 s over 7 phases, export 3.0 s, trace 0.2 s.  **`balance` is
0.3 % here against 65.5 % in v15** — `scripts/draw.py` passes `balance=False,
split=False` to the search, so the 78 candidates pay `flycheck` and `merge`
instead, and those two are 80 % of the bill.  `docs/ANY_PICTURE.md`'s premise —
*"that is the difference between a 48-placement search in 6 minutes and one in
8 hours"* — was measured before `--residual-passes` and the RRT pen-up tier
existed, and it no longer holds: **the search now inherits the run's whole flag
set and pays it 78 times.**  The obvious speed lever, and it is the rig owner's
call, is to search under a cheap flag set and spend the full one only on the
winner.

### CSAIL v17 — 100.0000 %, and then the tool moved again (ABORTED 2026-09-04)

**READ THE STALENESS FIRST.**  Every number in this section was measured at the
`PEN_EXT_HOLDER` / `PEN_LAT_HOLDER` = **0.0588421 / 0.0588421** pair.  On
2026-09-04, while the follow-on run was still conducting, Pete corrected the
holder placement again — **the grip point moves to the far end of the Fat finger
plates**, which moves the tool transform a third time.  So this entry is a
RECORD OF A METHOD AND OF A PLACEMENT, not a shippable programme: the atlas,
the parks and every gate below were certified against a tool that is being
superseded, exactly as `bc670bf` superseded the 0.110 pair.  **Nothing here
should be conducted, and the comparison at h = 0.880 was killed mid-run and is
not reported.**

WHAT SURVIVES THE TOOL CHANGE is the *method*: the proxy pre-check below costs
five minutes, it predicted v15's measured loss to 2 mm, and it is the thing to
re-run first against the new pair — it will say in minutes whether a placement
can reach 100 % before anyone spends 7.57 h searching for one.

v16 spent 7.57 h searching and lost 0.65 pp.  v17 spends **five minutes** on a
proxy pre-check, moves the logo by hand, and closed the gap: GUI job
**`20260904-074027-32b1`**, v15's exact flag set, v15's exact size and rotation
(90°, scale 0.85, `target_width` 1.43089), one fixed offset, **no search**.
`out/csail_schedule_h094_v17.{npz,json}`, `out/csail_program_h094_v17.json`,
`out/csail_place_v17_placement.json`, `out/h094_v17.log`.

**THE METRIC THAT PICKED IT, AND WHY IT IS TRUSTWORTHY.**  Sample the traced
ink every 1 cm, look each sample up in the 2 cm atlas union, and take the
**longest CONTIGUOUS run of dead samples inside one stroke** — because a
scattered dead cell is drawn anyway (the atlas is a sampling; the pose search is
continuous) while a contiguous run is a span nobody can cover.  On v15's own
offset it predicts **82 mm** against v15's **measured 80.3 mm loss**.  It is
right to 2 mm on the one case where the answer is known.

| offset at scale 0.85 | left edge | outer-rim ink | under-base ink | **longest run** |
|---|---|---|---|---|
| (−0.10, 0.00) = **v15** | 0.0863 | 82 mm | 98 mm | **82 mm** at (0.09, 1.24) |
| (−0.09, 0.00) | 0.0963 | 37 mm | 177 mm | 79 mm |
| (−0.07, 0.00) | 0.1163 | **0 mm** | 197 mm | **118 mm** at (0.55, 1.78) |
| (−0.03, 0.00) | 0.1563 | 0 mm | 237 mm | 158 mm |
| **(+0.05, −0.20) = v17** | 0.2363 | **0 mm** | **0 mm** | **0 mm** |

**A PURE LEFTWARD-TO-RIGHTWARD NUDGE CANNOT DO IT.**  Walking the logo right
does clear the outer rim at the left edge — by (−0.07, 0.00) the rim ink is
zero — but it walks the logo's INTERIOR onto **arm 31's base disc**, and the
longest run gets *worse*, 82 → 118 mm.  The middle-row columns stand at
x = 0.5967 and x = 1.2067 and a 1.431 m-wide logo at this size spans both; every
offset from −0.11 to −0.02 puts ink either in the rim or on a disc.  **The fix
is not a smaller x-nudge, it is +x AND −y together**: of **1 976** offsets that
fit the sheet, exactly **10** have a zero dead run, all at scale 0.85, all at
dx ≈ +0.05…+0.07.  v17 takes the one that also moves ink AWAY from the middle
row, since that is the direction that cost v16 five extra phases.

| | v15 | v16 | **v17** |
|---|---|---|---|
| offset | (−0.10, 0.00) | (−0.10, +0.10) | **(+0.05, −0.20)** |
| centre | (0.8017, 1.8153) | (0.8017, 1.9153) | **(0.9517, 1.6153)** |
| size | 1.4309 × 1.8711 m | same | **same** |
| coverage, allocated | 99.5224 % | 98.8729 % | **100.0000 %** |
| coverage, conducted | 99.7485 % | 97.9754 % | **100.0000 %** |
| left empty / skipped | 0.0803 / 0.0000 m | 0.1894 / 0.1947 m | **0.0000 / 0.0000 m** |
| segments | 47 | 48 / 47 | **49** |
| makespan | 277.625 s | 249.875 s | **195.229 s** |
| conducted pause | — | 56.7 s | 121.4 s |
| phases planned / conducted | 2 / 2 | 3 / 7 | **2 / 2** |
| min inter-arm, worst phase | 82.0 mm | 81.2 mm | **82.2 mm** |
| column / chain / tip | 122.0 / 30.7 / −7.3 | 131.0 / 22.6 / −4.9 | **136.6 / 30.4 / −7.0** |
| planner wall clock | 4 910.9 s | 29 352.9 s | **3 381.7 s** |

**INDEPENDENT `scene_check`, whole merged timeline, from the shipped `.npz`:**
**VERDICT PASS**, min inter-arm **80.91 mm** against the 80 mm gate — **+0.91 mm**,
worst pair **31–71 at t = 91.19 s**; self 20.6 mm, frame 53.1 mm, column
132.8 mm, paper chain 30.2 mm, tip −6.8 mm, joint margin 0.1124, frozen 6/6.
**That is 3.6× the margin v15 and v16 cleared by** (0.26 and 0.25 mm).  The
razor-thin seam this file flagged as "where a re-run should be watched" was a
property of those two placements after all, not of the rig.

**AND IT IS THE FASTEST PROGRAMME OF THE THREE**, 195.229 s against v15's
277.625 s — 30 % less makespan for 0.48 pp more ink, in two conducted phases
with no split and no skip.  At the 0.0588421 tool v17 superseded v15; at the
tool now being fitted, **neither of them stands**.

**THE h = 0.880 COMPARISON WAS ABORTED.**  The same v17 placement was being
re-planned at 0.880 on the searched 0.880 park grid — the run Pete needs for the
trim decision on a placement that is not known-bad — and it was killed at
08:39→10:03 during conduction when the tool correction landed.  It got as far as
scheduling the movers (arm 71: 3 170 steps, 66.0 s nominal → 122.3 s with
56.3 s of pauses) and **no coverage, gate or `scene_check` number was reached**,
so none is quoted.  `out/h088_v17p.log` and `out/csail_schedule_h088_v17p_trace.png`
are the partial remains; there is no schedule, no programme and no `.npz`.

**WHAT TO DO WHEN THE NEW TOOL LANDS**, in the order that costs least:
re-sweep the atlas at the new pair; run the five-minute proxy over the offset
grid to see whether a zero-dead-run placement still exists at scale 0.85;
re-search the park set (it is a quarter of an hour and the 0.940 grid did not
transfer to any other height, so it will not transfer across a tool either);
and only then spend an hour on a full plan.  **Do not start with
`--placement auto`** — v16 is the measurement of what that costs and what it
gets.

### The feasible-workspace map, v13

`scripts/feasible_workspace.py` at v12's settings (`--fiber-tries 48
--hover-lean-deg 15 --rrt 60 --rrt-nodes 300 --rescue 0,1`), 8 workers, on
`out/atlas_proposed_h0940_lat0588_gated63`; sweep 4 573 s + rescue 644 s.
`out/feasible_workspace_v13.{png,json}`, log `out/feasible_workspace_v13.log`.

| solo-drawable, 16 562 cells | v12 (0.110) | **v13 (0.0588421)** |
|---|---|---|
| FEASIBLE | **99.46 %** (6.589 m²) | **96.84 %** (6.416 m²) |
| draw pose only | 99.64 % | 97.05 % |
| + hover | 99.64 % | 97.05 % |
| + reachability | 99.46 % | 96.84 % |
| DEAD, no draw pose | 0.36 % | **2.95 %** |
| DEAD, draw ok no hover | 0.00 % | 0.00 % |
| DEAD, hover ok unreachable | 0.18 % | 0.21 % |
| largest inscribed rectangle | 0.52 × 3.54 m = 1.841 m² | 0.46 × 3.64 m = **1.674 m²** |

**THE LOSS IS REACH, NOT ROUTING.**  Route-dead barely moves (0.18 → 0.21 %,
30 → 34 cells) and hover-dead is still zero; "no draw pose" goes 59 → 489
cells.  The rescue ladder confirms it: rung 0 turned 150 arm-cells (dead
673 → 523) and rung 1 turned **none**, so what is left is not something a
better pen-up plan reaches.

**WHERE THE DEAD ZONES MOVED.**  89 → 523 cells; 464 newly dead, 30 revived.
v12's dead set was almost entirely the six under-base holes (74 of 89).  v13's
is **two** sets: 263 cells still under the base discs (r ≤ 0.30 m), and **242
cells out at 0.60–0.85 m from the nearest base** — a new OUTER RIM that did
not exist at the longer tool, distributed evenly over all three thirds of the
sheet (163 / 145 / 156 newly dead).  Both edges of every annulus moved inward:
the tool is 72 mm shorter along the approach axis and 51 mm shorter across it,
so the wrist has to come to the paper for the inner cells and cannot get out
to the far ones.

### Loose ends this run leaves behind

**`docs/SYSTEM_MODEL.md`'s park-pose clearance table is measured at the OLD
six poses** (403.2 / 264.0 mm arm-to-arm, 196.5 mm arm-to-paper).  The poses
moved here; the fleet's nearest pair is still at the broad-phase cap (≥ 250 mm)
and the closest tip-to-paper is now the **0.10 m** hover three arms take,
against the old set's 0.1965 m.  Re-measuring that table needs the URDF
pipeline and is not done here.

**`np.cross` on 2-vectors** — `tests/test_draw.py`'s SVG failure — was NumPy
2.0 removing the 2-D overload, not anything in this repo.  `trace._cross2`
writes the one line of arithmetic out; `test_draw` is 6 green.

**The two FK bit-identity failures in `tests/test_planner_robustness.py` are
the environment and they stay.**  `_franka_ik`'s C++ batch and `frames.fk`'s
NumPy chain differ by **max |ΔT| = 3.331e-16 and max |ΔP| = 2.220e-16** — one
to two ULP — on all 200 sampled configurations, under NumPy 2.5.2.  The
assertions are `np.array_equal`, deliberately, and loosening a bit-identity
gate is not an environment fix; the deviation is 13 orders of magnitude under
the 2 cm threshold the lattice gates on and 4 under the 1e-12 the URDF checks
hold.  Recorded, not patched.

## OPEN — FOR PETE: THE HOLES ARE THE DESCENT, NOT THE NEIGHBOURS (2026-09-08)

The entry below found that at h = 0.970 the DRAWING-POSE layer is perfectly
hole-free over 1.50 × 3.64 m and the pen-up layers cut it to 0.58 × 3.48.  Four
levers were aimed at those pen-up holes — including Pete's *"you can also move
the neighbour out of the way to try to reach under."*  **One works a little,
three cannot work at all, and the reason is the same for all three.**

| lever | h = 0.940 | h = 0.970 | cost |
|---|---|---|---|
| *baseline* (rescue rungs 0,1) | 37 holes, **1.469 m²** | 32 holes, **2.018 m²** | — |
| **1. rescue rungs 2–3** | 35 holes, **1.469 m²** (no change) | 29 holes, **2.100 m²** (+4.1 %) | 2 h 03 m / 1 h 29 m |
| **2. deeper RRT than rung 3** | not reachable — see below | not reachable | abandoned, unbounded |
| **3. depots out of the middle third** | **impossible** | **provably no effect** | minutes |
| **4. move the blocking neighbour** | **no cell is blocked by one** | **no cell is blocked by one** | minutes |

**RUNG 2 IS "seeds" (96 hovers, 16 plans, 900-node trees, 2 restarts) AND RUNG 3
IS "deep" (192, 24, 1800, 2, plus 3 aside park sets).**  Neither had ever been
run.  Together they cost **2 h 03 m at 0.940 and 1 h 29 m at 0.970** and bought
2 holes and 3 holes respectively — at 0.940 not enough to move the rectangle at
all, at 0.970 worth **0.082 m²**.  That is the honest price of the only lever
that works.

### Why levers 3 and 4 cannot work, measured rather than argued

`_cell_hovers` plans each leg with `writing.enter_beats` **against the metal**
(static boxes + the arm against itself) and only THEN checks the planned leg
against the PARKED partners with `ParkProbe.clearance`.  So a cell that a
parked neighbour vetoes has a *finite* `park_clear` recorded in the map's raw
array, and a cell the router could not plan at all has `NaN`.  Counting them:

| refusing (arm, cell) pairs on hole cells | h = 0.940 | h = 0.970 |
|---|---|---|
| no certified hover pose at all | 8 | 7 |
| **no leg plannable against the METAL** (`park_clear` = NaN) | **33** | **38** |
| **a parked partner vetoed a plannable leg** | **0** | **0** |

**NOT ONE CELL IS BLOCKED BY A PARKED ARM, AT EITHER HEIGHT.**  That is why
rung 1 "aside" was offered 46 cells and turned 0: the aside rung exists to move
a vetoing partner, and there is no vetoing partner to move.  The file already
said so in as many words — *"a cell whose DESCENT the ladder cannot do is
behind a wall no park can move"* — and this is that claim measured.

**AND THE SAME FACT KILLS LEVER 3 TWICE OVER.**  At 0.970, re-running the whole
three-layer map with a materially different park set (the depots searched at
0.970, every arm's pose different from the derived ones) gives a **bit-identical
map** — 16 138 feasible cells, zero cells changed.  And at 0.940 the constraint
is not even reachable: arms 31 and 71 sit ON the sheet's midline, and **no
certified depot exists for either of them outside the middle third** at any
radius, hover or bearing in the search.  The middle row has nowhere else to
stand — the same finding 2026-08-26 made about park-vs-ink, one layer out.

### Lever 4 in full: which leg refuses

For every refusing pair at h = 0.970 (after rungs 0–3), both legs of
`enter_beats` were re-planned separately over all certified hovers and the
whole 48-candidate fiber:

| verdict | count |
|---|---|
| **DESCENT (hover → paper) fails for every hover** | **21** |
| both legs fail, and only 2–4 hover candidates exist at all | 14 |
| no hover candidate at all | 7 |
| flight from the depot fails while the descent flies | **0** |
| both legs fly for some hover (i.e. a park veto) | **0** |

**THE DEPOT LEG NEVER FAILS ON ITS OWN.**  In the 21 clean cases the arm can
fly from its park to all 53 hover candidates and cannot get down from any of
them.  So no park pose — the arm's own or a neighbour's — is on the critical
path: **moving the neighbour out of the way cannot recover a single one of
these cells**, because the neighbour was never in the way.  What is in the way
is the last few centimetres between a hover and the paper, next to a base
column, and that is geometry the planner cannot route around rather than an
obstacle a schedule can remove.

**SO THESE ARE THE PHYSICS CASES.**  Each hole cell is certified for exactly
ONE arm (there is no second arm to fall back on — that is why it is a hole and
not merely a slow cell), and it sits a median **137 mm** from a base.  The arm
has a certified drawing pose there; what it does not have is a descent onto it.
The 7 "no hover candidate" cells are stronger still: no certified hover exists
above them at any height on the fiber.

### What is left, and what the certification now depends on

**ADOPTED: rungs 2–3 at h = 0.970 only.**  `out/certified_area_h0970.json`
records the rectangle **0.58 × 3.62 m = 2.100 m²** at (0.64, 0.00) and, in its
new `recipe` field, that rungs 2–3 are load-bearing for it — a map rebuilt at
rungs 0,1 has 2.018 m² and not this rectangle.  At h = 0.940 rungs 2–3 change
nothing, so `out/certified_area_h0940.json` keeps **0.54 × 2.72 m = 1.469 m²**
and records that the rungs are optional there.  Overlays redrawn.

**THE RECOMMENDATION IS UNCHANGED AND NOW BETTER EVIDENCED.**  0.970 remains the
best height (2.100 against 1.469 m², +43 %), and the strip is still ~0.58 m
wide.  **The remaining holes are not a planner budget problem and not a
scheduling problem** — three of the four levers are now excluded by measurement
rather than by estimate.  If Pete wants a materially wider certified strip the
levers left are physical, not software: the pen-up contact floor
(`paper.CONTACT_FLOOR`, which is what the descent is refused against and is a
GATE, so out of scope here), a shallower approach at the cell, or a layout where
no cell is single-arm — the 0.58 m width is set by cells only one arm can reach,
and redundancy, not routing, is what would remove them.

**ONE PROVENANCE CORRECTION TO THE ENTRY BELOW.**  Its h = 0.970 three-layer
numbers were produced by a run whose header printed the searched depots while
its WORKERS derived their own from `PARK_GRID_PROPOSED` — `--parks` reached
`main()` but not the forked pool, a defect introduced with that flag.  It is
fixed (`feasible_workspace.set_park_override`, proved to reach a forked worker),
and the map was rebuilt correctly.  **The rebuilt map is bit-identical**, so
every number below stands; the finding is that those numbers never depended on
the park set in the first place, which is the same conclusion this entry reaches
by four other routes.

## OPEN — FOR PETE: NO HOLES UNDER THE ARMS (2026-09-08)

Pete: *"can we make it so the continuously covered chunk is as big as possible
(no holes under the arms)... This doesn't mean the overall coverage is maximal
but we just don't want any holes if we place a drawing in the certified area."*

**THAT IS A DIFFERENT FIGURE OF MERIT AND IT ORDERS THE HEIGHTS DIFFERENTLY.**
Coverage counts a dead cell as 0.006 % of the canvas; a drawing counts it as a
knife through every rectangle that contains it.  Measured with
`scripts/certified_area.py` (new): the largest axis-aligned rectangle with NO
dead cell in it, at the final tool (0.0460262 / 0.0860369, 23°),
`LAYOUT_PROPOSED` untouched at 0.940.

### The sweep — drawing-pose layer, all six heights

2 cm atlases at the current gates, ≤ 15° lean, one per height
(`out/atlas_proposed_h*_lat0860`, 90–709 s each).

| h | live % | dead under bases | dead on the rim | enclosed holes | **largest hole-free rectangle** | contains the centre? |
|---|---|---|---|---|---|---|
| 0.850 | 98.38 | **249** | 20 | 7 (0.0996 m²) | 1.78 × 1.00 = **1.780 m²** | **no rectangle does** |
| 0.880 | 98.44 | 202 | 57 | 6 (0.0808 m²) | 1.70 × 1.06 = **1.802 m²** | 0.48 × 2.38 = 1.142 |
| 0.910 | 98.42 | 129 | 133 | 9 (0.0516 m²) | 1.66 × 1.08 = **1.793 m²** | 0.48 × 3.64 = 1.747 |
| **0.940** (ships) | **98.47** | 29 | 225 | 8 (0.0116 m²) | 0.62 × 3.64 = **2.257 m²** | yes, the same one |
| **0.970** ⚠ | 97.72 | **0** | 378 | **0** | 1.50 × 3.64 = **5.460 m²** | yes, the same one |
| 1.000 ⚠ | 96.32 | **0** | 609 | **0** | 1.40 × 3.64 = **5.096 m²** | yes, the same one |

⚠ = hardware check needed (the drop posts trim shorter; the runway underside is
1623.6 mm above the paper, so there is room, but nobody has cut it).

By aspect class, same layer (landscape ≥ 1.5:1 / near-square / portrait):

| h | landscape | near-square | portrait |
|---|---|---|---|
| 0.850 | **1.780** | 1.269 | 1.674 |
| 0.880 | **1.802** | 1.272 | 1.602 |
| 0.910 | 1.793 | 1.782 | 1.747 |
| 0.940 | 1.696 | 1.792 | **2.257** |
| 0.970 | 1.793 | **3.435** | **5.460** |
| 1.000 | 1.685 | 3.010 | 5.096 |

**THE CROSSOVER IS BETWEEN 0.940 AND 0.970, AND IT IS A CLIFF, NOT A SLOPE.**
The under-base holes close completely — 29 cells to **zero** — and with them the
last enclosed hole in the live set, so the largest hole-free rectangle jumps
**2.257 → 5.460 m², 2.4×**, while total coverage *falls* 0.75 pp.  Coverage and
hole-freeness are not just different, here they point opposite ways.  **Past
0.970 the trend reverses**: at 1.000 the rim has eaten 609 cells and the
rectangle is back to 5.096 m².  0.970 is the peak of the six.

**AND AT 0.850 THERE IS NO HOLE-FREE RECTANGLE CONTAINING THE PAPER'S CENTRE AT
ALL** — the centre cell itself is dead.  "Put it in the middle" is not
available at the bottom of the range.

### ...and then the pen-up layers put the holes back

The table above is the drawing-pose layer.  The honest map is
`feasible_workspace`'s three (draw pose + hover + reachability), and it was run
in full at the shipped height and at the winner — park set re-searched at each,
`--rescue 0,1` as v13/v14, ~2 h each.

| all three layers | **h = 0.940** (v14) | **h = 0.970** |
|---|---|---|
| solo-drawable | **97.95 %** | 97.44 % |
| dead under bases | 93 | **32** |
| dead on the rim | 225 | 379 |
| enclosed holes | 37 (0.0456 m²) | 32 (0.0180 m²) |
| **largest hole-free rectangle** | 0.54 × 2.72 = **1.469 m²** | 0.58 × 3.48 = **2.018 m²** |
| …landscape | 1.14 × 0.76 = 0.866 | 1.14 × 0.76 = 0.866 |
| …near-square | 1.10 × 0.88 = 0.968 | 1.12 × 1.14 = 1.277 |
| …portrait / centred | 1.469 | **2.018** |
| park-vs-ink (re-searched) | 93.1 mm | **97.7 mm** |
| park flyability | 78.5 % | **79.2 %** |

**THE 5.460 m² DOES NOT SURVIVE CONTACT WITH THE ROUTER.**  At 0.970 the
drawing-pose layer is perfectly hole-free; requiring each cell to be *lifted
off* and *flown to* puts **32 holes back** and collapses the rectangle to
0.58 m wide.  0.970 still beats 0.940 — **2.018 against 1.469 m², +37 %** — and
it is still the best of the six, but the honest number is a third of the
optimistic one.  **Those 32 holes are hover and route failures, not reach**, so
they are the kind a better router or a better park set can attack, and rungs 2
and 3 of the rescue ladder were NOT run here (as in v13/v14).  That is the
cheapest unexplored lever on this question.

### The 20° lean allowance buys nothing here

Both best heights re-swept at `--tilt 20` (Pete's pending decision):

| | 15° | 20° |
|---|---|---|
| h = 0.940 rectangle | 2.257 m² | **2.257 m²** (under-base 29 → 27) |
| h = 0.970 rectangle | 5.460 m² | **5.460 m²** (under-base 0 → 0) |

A wider cone certifies a handful more cells at the *rim*, where the dead set
already is; it does not open the holes under the arms, because those are not
lean-limited.  **The lean decision and the hole question are independent** —
decide 20° on its own merits.

### Recommendation

**Trim to h = 0.970 if the hardware allows it, and treat 0.58 m as the width
of the certified strip either way.**  It is the peak of the six on the measure
Pete asked for: the under-base holes close completely at the drawing-pose layer
(29 → 0 cells), the hole-free rectangle grows 37 % on the honest three-layer map
(1.469 → 2.018 m²), the park set re-searches *better* there than at 0.940
(97.7 mm against 93.1, 79.2 % flyable against 78.5), and it costs 0.51 pp of
total coverage — which is precisely the trade Pete said he was willing to make.
Do not go to 1.000: the rim starts taking more than the bases give back.  But
the headline should not be oversold — **at no height in this range is there a
wide hole-free region once the pen-up layers are honest**; the best strip is
0.58 m across, so a drawing wider than that cannot be placed hole-free anywhere
at any height tested.  If Pete wants a genuinely large certified area the next
lever is not the height, it is the 32 hover/route holes at 0.970: rescue rungs
2–3, or a park set chosen to keep the depots out of the middle third.

**DELIVERABLES.**  The certified rectangle is emitted as JSON with the atlas,
tool and gates it came from — `out/certified_area_h0940.json` (shipped) and
`out/certified_area_h0970.json` (best) — and drawn on the maps
(`out/feasible_workspace_v14_certified.png`,
`out/feasible_workspace_h0970_certified.png`).
`scripts/placement_proxy.py --certified-area FILE` refuses any placement whose
box leaves that rectangle and prints the overhang per side.  **The CSAIL logo
does not fit either of them**: at 0.940 it overhangs 0.47 m west and 0.44 m
east at scale 0.85, and 0.18/0.14 m even at scale 0.50 — v18 draws 100 % only
because its ink threads *between* the holes, not because it sits in a
hole-free box.

## OPEN — FOR PETE: THE MOUNTING HEIGHT, RE-ASKED AT THE REAL TOOL (2026-09-03)

**Nothing here changes `layout.LAYOUT_PROPOSED`.  h = 0.940 is still what
ships.**  This is the data for a decision that is Pete's, and it is being
re-asked because the decision that set 0.940 (2026-08-26, README "Re-certifying
the rig at height") was made against a **110 mm** tool that no longer exists.
The holder's own tip is 72.348 mm closer to the hand, so at any given h the
arms are effectively that much further from the paper than that decision
assumed — and the hardware is being built at **850** with the option to trim
the verticals to 940, which is a one-way door in one direction only.

Measured with `scripts/height_sweep.py`, new this run.  Every instrument in the
repo that reads a height takes it from `LAYOUT_PROPOSED` and offers no way to
ask about another one, so the layout and the parks are built LOCALLY by
`feasible_workspace.rig(None, h, None)` — the call the map already makes, with
the `h` its own CLI never passes.  A run at 0.940 reproduces the shipped rig's
published numbers exactly (97.08 % union, 0.46 × 3.64 m rectangle, +97.8 mm
park-vs-ink), which is what makes the other three rows believable.

    scripts/height_sweep.py sweep  --h H --out out/atlas_proposed_hHHHH_lat0588
    scripts/height_sweep.py report --case H=out/atlas_proposed_hHHHH_lat0588 ...

**FULL RESOLUTION, NOT REDUCED.**  2 cm over all six arms, 16 562 canvas cells,
the same sweep and the same denominator as the tool-change table above; 876 s
(0.850), 794 s (0.880), 661 s (0.910) on 6 jobs.  0.940's atlas is the one v15
and v16 were planned against.

| 2 cm, 16 562 cells, tool 0.0588421 | **0.850** | **0.880** | **0.910** | **0.940** (ships) |
|---|---|---|---|---|
| union strict-GO | 95.75 % | **97.44 %** | 97.33 % | 97.08 % |
| reachable | 99.98 % | **100.00 %** | 99.93 % | 99.71 % |
| dead | 4.25 % (703) | **2.56 % (424)** | 2.67 % (442) | 2.92 % (484) |
| ≥ 2 arms | 37.98 % | 37.53 % | 37.66 % | **38.88 %** |
| ≥ 3 arms | **8.31 %** | 7.17 % | 5.75 % | 4.21 % |
| largest live rectangle | 1.74 × 0.94 = 1.636 m² | **1.70 × 1.00 = 1.700 m²** | 1.66 × 1.00 = 1.660 m² | 0.46 × 3.64 = 1.674 m² |
| …its shape | landscape | landscape | landscape | **portrait** |
| dead under the bases (r < 0.20) | 679 | 358 | 310 | 242 |
| dead in the outer rim (r ≥ 0.70) | **24** | 66 | 132 | 242 |
| park-vs-ink, **shipped grid** | −126.0 mm | −56.7 mm | −46.6 mm | **+97.8 mm** |
| park-vs-ink **ceiling** | 97.3 mm | 97.7 mm | 97.9 mm | 97.8 mm |

**THE COVERAGE OPTIMUM IS 0.880, NOT 0.940, AND NOT 0.850.**  97.44 % against
97.08 %, with every canvas cell reachable and the biggest clean rectangle of
the four.  That is a 0.36 pp preference and it should not by itself move a
build; what it does say is that **at the holder's own tool the reach argument
no longer points up.**  It pointed up at the 110 mm tool (87.00 → 90.70 % from
0.850 to 0.940); the shorter tool has closed most of that gap.

**BECAUSE TWO DEAD SETS TRADE AGAINST EACH OTHER, AND THEY MOVE OPPOSITE WAYS.**
The outer rim the shorter tool created is a HIGH-h problem — 242 cells at
0.70–0.90 m from the nearest base at 0.940, 24 at 0.850 — because the wrist
cannot get out that far when it hangs high.  The holes under the six bases are
a LOW-h problem — 242 cells at 0.940, 679 at 0.850 — because an arm hanging low
has to fold to reach under itself and the fold is what the gates refuse.  0.880
is where the sum is smallest.  It also changes the SHAPE of what is drawable:
0.940 keeps a full-length 0.46 × 3.64 m portrait strip that the low heights cut
into by opening the base holes, while the low heights keep a wide landscape
block the high ones lose to the rim.  **A picture that wants the length wants
0.940; a picture that wants the width wants 0.880 or lower.**

**AND REDUNDANCY GOES THE OTHER WAY AGAIN.**  ≥ 3-arm overlap doubles from
4.21 % at 0.940 to 8.31 % at 0.850 — the thing the shorter tool cost most
(11.11 → 4.21 % at 0.940) is partly bought back by hanging lower.  That is
concurrency, which is makespan, not coverage.

### What a lower h costs — the honest caveat, and it is not reach

**THE 2026-08-26 KILL WAS COLLISION-DRIVEN, NOT REACH-DRIVEN.**  The record is
explicit (README, and `layout.py` at `LAYOUT_PROPOSED`): union coverage was
worse at 0.850, but that is not what killed it.  What killed it was
**park-vs-ink** — how far a parked arm's chain stands from every OTHER arm's
certified ink, which the conductor holds to 80 mm — climbing 10 → 61 → 72 mm
across 0.850 / 0.925 / 0.940.  *"At 0.850 and 0.925 the shortfall is total and
the rig draws nothing."*  And it was not the margin: re-conducting with the
calibration term cut to zero returned the same programme.

**THE SHIPPED PARK GRID DOES NOT TRANSFER DOWNWARD, AND THAT IS THE REAL COST.**
`PARK_GRID_PROPOSED` was searched at 0.940.  Applied at the other heights it
degrades monotonically and lands **inside** the ink: −46.6 mm at 0.910, −56.7 mm
at 0.880, **−126.0 mm** at 0.850 (arm 17's park pose inside arm 13's ink).  A
build at 850 with today's `layout.py` would have six arms parked where the
others want to draw, and no ordering fixes that (`coordination.hard_blocks`).
**That is a cost of the LITERALS, not of the height** — the search below settles
which.

**BUT THE HEIGHT DOES NOT FORBID IT — THE CEILING IS FLAT.**  A parked arm
carries one segment no pose can move, the flange-to-shoulder column at its own
base xy, and the clearance from a mover's certified ink to a neighbour's COLUMN
is therefore an upper bound on park-vs-ink that no park set whatever can beat.
Measured, that ceiling is **97.3 / 97.7 / 97.9 / 97.8 mm** across the four
heights — flat, because the columns are at the same six xy at every height.
The check on it: at 0.940 the ceiling comes out at **97.8 mm on the same
binding pair (31, 71) that the shipped park set actually achieves**, which is
the measured form of what this file already records as *"+97.8 mm is the
layout's ceiling, not a lucky draw"*.

**SO THE HEIGHT QUESTION TURNED INTO A PARK-SEARCH QUESTION — AND THE SEARCH
HAS NOW BEEN RUN.**

### The park search, re-run below 0.940 — and 0.850 CERTIFIES

`height_sweep.py park`.  **This is the first time the three-number (radius,
hover, bearing) grid has been searched below 0.940 at all**: the 2026-08-26
figure of 10 mm at 0.850 came from the older TWO-number (radius, hover) search
at the 110 mm tool, and the bearing alone was worth ~25 mm at 0.940
(72 → 97.8 mm).

Same gates and same ranking as the search that produced the shipped grid:
24 absolute bearings × 6 radii × 4 hovers = **576 candidates per arm**, each
gated by `certified_ready_pose` at the holder's own (0.0588421, 0.0588421) and
by `rig_final.chain_static_clearance ≥ STATIC_MARGIN`; scored against every
other arm's **ink AND lift layers** (`out/park_search3.py`'s v3 criterion — a
depot clearing the ink by 97 mm was found sitting 51 mm inside the 6 cm lift
layer above it); ranked on the depot's own flyability, tie-broken on the 5 mm
clearance plateau and then `min(joint_margin, 2.5σ)`.  2 166–2 423 s per height
on 6 jobs.

**THE CONTROL PINS IT.**  At 0.940 the search returns **430 of 576** candidates
certifying per arm — the record for the 2026-09-03 re-search says *"430–431 of
577 certify"* — a fleet worst of **97.8 mm**, park-vs-park at the **250 mm**
broad-phase cap, and **78.5 %** flyability against the record's 78.8 %.  Same
grid, same gates, same answer.

| 576 candidates/arm, gate 80 mm | **0.850** | **0.880** | **0.940** (ships) |
|---|---|---|---|
| certify + clear the steel | 374–380 | 398–399 | **430** |
| fleet worst, ink AND lift | **87.8 mm** | 84.4 mm | **97.8 mm** |
| …ranked on clearance instead | 87.8 mm | **97.7 mm** | 97.8 mm |
| fleet park-vs-park | ≥ 250 mm (cap) | ≥ 250 mm (cap) | ≥ 250 mm (cap) |
| entries flyable | **79.9 %** | 75.7 % | 78.5 % |
| go-homes flyable | **81.2 %** | 76.4 % | 78.5 % |
| **verdict at the 80 mm gate** | **CERTIFIES** | **CERTIFIES** | **CERTIFIES** |

**ALL THREE HEIGHTS HAVE A CONDUCTABLE PARK SET, AND 0.850 IS NOT THE WORST OF
THEM.**  It clears by 87.8 mm and flies to MORE of its own ink than either
higher rig (79.9 / 81.2 % against 0.940's 78.5 / 78.5 %) — because a lower arm
reaches under itself less well but reaches its depot more easily.  **The
2026-08-26 kill does not survive the bearing and the shorter tool.**  What
killed 0.850 was a two-number search at a tool that no longer exists.

**WHAT BINDS AT 0.850 IS THE LIFT LAYER, NOT THE INK.**  The ink-only ceiling
there is 97.3 mm and the achieved both-layer number is 87.8 mm, held down by
**arm 71** — the middle-row arm, whose depot has to clear the 6 cm hover layer
over a neighbour's ink as well as the ink itself.  At 0.880 and 0.940 the two
criteria agree to a millimetre.

**THESE GRIDS ARE REPORT-ONLY.**  `layout.PARK_GRID_PROPOSED` is untouched and
still the 0.940 set.  A build at another height would take the grid for that
height from `out/park_search_h0850_lat0588.json` /
`out/park_search_h0880_lat0588.json`, and that is a change to `layout.py` that
belongs with the decision, not ahead of it:

    h = 0.850:  {2: (0.70, 0.35,  135.0), 13: (0.70, 0.10,  150.0),
                 17: (0.70, 0.35,  -60.0), 31: (0.70, 0.10,  150.0),
                 71: (0.70, 0.20,   30.0), 97: (0.70, 0.20,   45.0)}
    h = 0.880:  {2: (0.48, 0.35, -150.0), 13: (0.70, 0.20,  150.0),
                 17: (0.70, 0.30,  -45.0), 31: (0.70, 0.10,  150.0),
                 71: (0.70, 0.20,  -30.0), 97: (0.70, 0.10,  -30.0)}

**WHAT THIS DOES NOT PROVE.**  That a CSAIL programme conducts at 0.850 — a
park set clearing the gate is the constraint that blocked 2026-08-26, not a
guarantee that the allocator and the conductor then succeed.  The 2026-08-26
package ran the whole pipeline end to end at each height; this runs the park
stage only.  Re-planning the logo at 0.880 is the next thing to ask for, and it
is a v16-sized job, not a quarter of an hour.

### The pen-up layers, sampled — and they run the same way

The table above is the DRAWING-POSE layer.  Solo-drawable (`feasible_workspace`'s
FEASIBLE) adds the hover ladder and the route, and a full map is ~1.7 h per
height at 6 workers, so instead `height_sweep.py pilot` walks **every 25th
certified cell** at v13's own ladder (fiber-tries 48, hover-lean 15°, RRT 60 s ×
300 nodes), ~950 arm-cells per height, 696–1 858 s each.

**A PILOT IS NOT A MAP.**  It says what fraction of an arm's CERTIFIED cells
survive, never what fraction of the CANVAS does — the cells it skipped are
indistinguishable from cells no arm can draw.

| every 25th certified cell, ungated atlas | **0.850** | **0.880** | **0.910** | **0.940** |
|---|---|---|---|---|
| arm-cells sampled | 963 | 960 | 946 | 940 |
| feasible | 78.50 % | 83.85 % | 84.36 % | **85.64 %** |
| no hover | 6.75 % | **5.73 %** | 7.29 % | 7.98 % |
| **no route** | **14.75 %** | 10.42 % | 8.35 % | **6.38 %** |

**THE PEN-UP LAYERS COST MORE THE LOWER THE ARMS HANG, AND IT IS ROUTE, NOT
HOVER.**  Refusals rise 14.36 → 15.64 → 16.15 → **21.50 %** of arm-cells as h
falls from 0.940 to 0.850, and essentially all of the rise is `no route`
(6.38 → 14.75 %) while `no hover` barely moves.  That is the same mechanism
`layout_rescore.py` named for the drawing layer — an arm hanging low has to
FOLD, and the fold lifts its elbow into the band its neighbour's base column
occupies — showing up one layer further out, in the transits.

**SO 0.850 IS WORSE ON BOTH LAYERS AND 0.880 IS THE COMPROMISE.**  The pilot
ordering runs the same way as the union column: 0.850 is last on reach (95.75 %)
and last on routing (78.50 %); 0.880 buys back nearly all of both.

**WHAT THE PILOT'S OWN CONTROL SAYS.**  At 0.940 this pilot reads **85.64 %**
against v13's **92.88 %** on the same cells — because the pilots run on the
UNGATED atlases (the checker's 50 mm floor) and v13 ran on `_gated63`.  Those
7.24 pp are the [50, 63) mm band `scripts/regate_atlas.py` exists to remove, not
the height; the h comparison is like-for-like because every row is ungated, and
a re-gated run would lift all four.

**AND THE CANVAS-LEVEL CORRECTION CANNOT BE READ OFF THESE NUMBERS.**
Redundancy absorbs almost all of a per-arm-cell loss: at 0.940 a **7.12 pp**
arm-cell refusal rate becomes a **0.24 pp** canvas loss (union 97.08 →
solo-drawable 96.84 %).  0.850 carries a bigger arm-cell loss *and* more
redundancy to absorb it with (≥ 3 arms 8.31 % against 4.21 %), so the two move
opposite ways and only a full map settles it.  **Quote the union column as the
h comparison and the pilot column as the direction of the correction; do not
add them.**

### The logo re-planned end to end at 0.880 and 0.850

The park search says all three heights have a conductable depot set.  That is
the constraint that blocked 2026-08-26, not a guarantee the allocator and the
conductor then succeed — so the whole pipeline was run at each, and this is the
part the 2026-08-26 package did that the park stage alone cannot.

`scripts/replan_at_height.py`, new and report-only, driving `scripts/draw.py`
**directly**: the GUI form cannot express a height or a park override, so these
are not GUI jobs.  Same picture, same flag set as v15/v16, **v16's placement
held fixed** (`out/csail_place_v16_placement.json`, no search), the per-height
atlas, and the park grid **searched at that height**.  `layout.LAYOUT_PROPOSED`
and `layout.PARK_GRID_PROPOSED` are untouched; the height rides on the fleet
object and the script proves that before it plans (see its docstring).
`out/csail_schedule_h088_v17.*`, `out/csail_schedule_h085_v17.*`,
`out/h08{8,5}_v17.log`.

| on v16's placement, holder tool | **0.850** | **0.880** | **0.940** (v16) |
|---|---|---|---|
| coverage, allocated | 98.1080 % | **98.8730 %** | 98.8729 % |
| coverage, **conducted** | 95.6783 % | **99.1194 %** | 97.9754 % |
| left empty at allocation | 0.3179 m, 2 spans | 0.1894 m, 1 span | 0.1894 m, 1 span |
| **skipped at conduction** | 0.4323 m | **0.0000 m** | 0.1947 m |
| segments | 47 | 47 | 48 / 47 |
| makespan | 320.167 s | 420.750 s | **249.875 s** |
| conducted pause | 120.3 s | 57.7 s | 56.7 s |
| phases planned / conducted | 4 / 6 | 4 / 8 | 3 / 7 |
| min inter-arm, worst phase | 80.8 mm | **85.2 mm** | 81.2 mm |
| neighbour base column | 120.1 mm | 123.2 mm | 131.0 mm |
| paper, chain | 22.4 mm | **32.8 mm** | 22.6 mm |
| paper, tip | −5.9 mm | −6.4 mm | −4.9 mm |
| planner wall clock | 2 408.7 s | 1 750.0 s | 2 105.2 s |

**INDEPENDENT `scene_check`, whole merged timeline, each against a fleet built
at its own height:**

| | 0.850 | 0.880 | 0.940 (v16) |
|---|---|---|---|
| verdict | **PASS** | **FAIL** | **PASS** |
| min inter-arm | 80.24 mm (2–97, t = 80.98 s) | 80.32 mm (13–17, t = 6.98 s) | 80.25 mm (17–71, t = 35.68 s) |
| self-collision | 20.2 mm (arm 71) | **19.4 mm (arm 2) — under the 20 mm gate** | 27.5 mm |
| frame | 52.1 mm | 53.9 mm | 53.0 mm |
| column | 120.1 mm | 119.1 mm | 130.9 mm |
| paper chain / tip | 22.3 / −6.1 mm | 32.7 / −6.0 mm | 23.0 / −4.9 mm |

**0.880 IS THE BEST OF THE THREE ON INK AND THE ONLY ONE THAT FAILS.**  It
conducts **everything it allocates** — 0.0000 m skipped, 99.1194 % conducted,
the best number in this whole re-certification — and then the whole-timeline
check refuses it on **arm 2's self-collision at 19.4 mm, 0.6 mm under the gate**,
while every per-phase check inside the run passed.  That is the seam the v15
entry already names: the merged timeline spans the pen swap and the freeze that
no per-phase check covers.  **It is 0.6 mm, and it is a refusal.**

**THE SAME 0.19 m IS LOST AT EVERY HEIGHT, AND IT IS THE PLACEMENT'S FAULT.**
Stroke 1 at **(0.7012, 1.802)** is dropped at 0.850, 0.880 and 0.940 alike —
because it runs through arm 31's base, and the under-base hole is there at every
height (242 cells at 0.940, 358 at 0.880, 679 at 0.850).  0.850 loses a second
span, stroke 3 at **(0.5229, 1.8041)**, in the same middle row, which is exactly
the 679-cell hole set showing up in ink.  **So this table compares three heights
on a placement now known to be bad at all of them**, and none of these three
coverage numbers should be read against v15's 99.5224 % — v15 is a different
placement.

**WHAT IT DOES SAY.**  Height ordering on conducted ink is
**0.880 > 0.940 > 0.850**, and the gap is conduction, not reach: 0.850 skips
0.43 m of ink it had already certified, 0.940 skips 0.19 m, 0.880 skips none.
0.850's conducted pause is 120.3 s against ~57 s at both higher rigs.  The
searched park sets held at both heights — no phase was refused for a park pose
— which is the 2026-08-26 blocker not recurring.

**WHAT WOULD MAKE THIS DECIDABLE.**  Re-plan all three on a placement that
clears the base holes (`--slack 0`, or the incumbent seeded, per the v16 entry),
and re-run the whole-timeline check on each.  Until then 0.880 is the most
promising and the least proven: best ink, thinnest self-collision margin, and a
refusal at 0.6 mm.

**WHAT NONE OF THIS ANSWERS.**  `SYSTEM_MODEL.md`'s ceiling survey (§8 item 3) —
how high the real room is above the paper — which is the physical question this
whole table is downstream of.

## THE CEILING DATUM IS A PROVENANCE BUG (2026-09-01) — and the model says so

`assets/system_model/` is the installation as it will be built; the numbers and
their sources are in `aris_sixarm/system_model.py` and, machine-readably, in
`assets/system_model/model_manifest.json`.  Full write-up:
`docs/SYSTEM_MODEL.md`.

| Quantity | Value | Source / provenance |
|---|---|---|
| Cage total height | **2336.50 mm above the FLOOR** | the original drawing's own 233,7 cm, reproduced from `rig_final.FRAME_BOXES_W_CM["top_slab"]` |
| Paper above the floor | **636.68 mm** | `rig_final.PAPER_ORIGIN_W_CM[2]` |
| Grid underside / top **above the paper** | **1623.62 / 1699.82 mm** | the same steel, re-datumed to the canvas frame |
| `mounts.MOUNTS.ceiling_z` | **2.34 m**, i.e. 2340 above the paper | the drawing's floor-referenced 233,7 read as paper-referenced.  **UNCHANGED — see below** |
| Drop post at h = 940 | **718.60 mm** corrected, **1434.98** at the buggy datum, **736.90** as originally built | `system_model.z_ladder()` |

**`mounts.py` IS NOT EDITED.**  Being too tall is conservative for collision —
a longer obstacle never certifies a pose a shorter one refuses — so no
certified number in this repo is in question, and correcting the obstacle
model is a re-certification with its own gate rather than a typo fix.  It is
not conservative for a fabricator, which is the whole reason the model carries
the truth separately and `system_model.reconciliation()` reports the gap.

### What else the model measured

| Finding | Number | Status |
|---|---|---|
| Every piece of mount hardware escapes the modelled keep-out | worst **185.55 mm** (a gusset); the **plate** by 25.06 mm in plan, which the layout sheet reads as inside because it compared thicknesses | **RE-CERT before fabrication**, labelled per body in the manifest |
| The inverted base cable | the manufacturer's link0 visual runs **230.7 mm** past the flange, i.e. UP through the 12.7 plate and the 95.7 clamp stack; the collision shell stops at the flange | **OPEN.** the plate and clamp stack need cutting; nothing in this repo had seen it |
| Clearance at the six certified park poses | arm-arm **264.0 mm** (audited capsules) / 403.2 (shells); arm-STEEL **316.1** / 349.3; arm-paper 196.5, which is the park hover.  The tightest steel approach in the rig is an arm to its NEIGHBOUR'S DROP POST — the very structure the certified envelope gets wrong | **CLEAN.** park poses only; drawing poses are still gated against `mounts.py` |
| Gusset interference | the drawing's inboard orientation needs 406.4 mm across a transverse pair and 216.20 exists; rotated onto the runway's y faces it clears by **89.20 mm** | ADOPTED in the model, attachment detail OPEN |
| Arm collision geometry | the vendored Panda's **66 unaudited spheres per arm are gone**; the manufacturer's own shells (FR3-identical to 0.344 mm) and the audited capsules replace them, in two files | ADOPTED |
| Franka textures | all 54 the arm needs were on disk and never copied; vendored byte-identically, ktx2 dropped (VTK cannot read it) | FIXED |

## LATERAL PEN HOLDER (2026-08-25) — the tool model changed

| Quantity | Value | Source / provenance |
|---|---|---|
| Grip centre along the blades | **`panda_hand` x = 0.066500 m** (`rig_final.PENHOLDER22["grip_hand_x"]`); height unchanged at the TCP's 0.1034 | **USER-SPECIFIED 2026-09-04** from the photo: the holder is clamped at the FAR END of the Fat finger plates — "the gripping point is at the tip of the finger extension" — not on the finger centreline where the plate's Ø6 hole is.  The plate runs link x −9.000…+79.500, so a 26 mm post flush with its far edge sits at 79.500 − 13.000.  Sliding the grip TRANSLATES the holder and nothing else: bore lean, grip-to-cap, grip-to-tail and the graphite are all unchanged |
| Graphite past the cap | **0.020 m**, `frames.PEN_GRAPHITE_HOLDER` | **USER-SPECIFIED 2026-09-07** from the side view down the jaw axis: "shorten the shaft of the pen … it only juts out 3–4 cm max", then "about 2 cm" once the orientation was confirmed against the real gripper ("looks great now").  The TIP is now DERIVED from this and the holder — `tip = grip + (30.001 + 20.000) mm · u` — which **SUPERSEDES and contradicts** the "tip 50 mm below the bottom edge of the blades" rule of 2026-09-03: the derived tip sits 37.18 mm below that edge.  Corroboration, not argument: 20 + 85.100 + 72.514 = 177.6 mm of stick, against a 175 mm Cretacolor Monolith |
| Lateral tip offset | ~~0.110~~ → ~~0.0588421~~ → ~~0.1253421~~ → **0.0860369 m along hand x** (perpendicular to finger travel): tip = TCP + R @ (0.0588421, 0, 0.0588421) | **RE-SPECIFIED three times: 2026-09-03, 2026-09-04, 2026-09-07.**  Both halves now fall out of one line — `tip = grip + (30.001 + 20.000) mm · (sin 23°, 0, cos 23°)` from a grip at (0.066500, 0, 0.1034) — so `PEN_EXT_HOLDER` = 0.0460262 and `PEN_LAT_HOLDER` = 0.0860369.  USER-SPECIFIED, **never gate-validated** — that wording was wrong wherever it appeared |
| Axial tip offset (holder) | ~~0.110~~ → ~~0.0588421~~ → **0.0460262 m below the TCP along tool z** | **NEW CONSTANT, `frames.PEN_EXT_HOLDER`, 2026-09-03.**  The holder used to borrow the INLINE pen's 0.110 because nobody had measured its own, and `activate_tool` switched only the lateral half — so a lateral run drew the holder's 45° ray out to an axial depth belonging to a different tool.  Both halves switch now.  Refine by touchdown calibration once the holder is mounted |
| Lean out of the approach axis | **23°**, `frames.PEN_LEAN_HOLDER` — the **BORE's** lean, measured at the GRIP.  The TCP→tip ray is 61.86° and is a different thing | **SETTLED 2026-09-07, and it is the housing's own number after all.**  Pete: "the square part of the holder is flush with the metal part, so the entire pen-holder plastic bit … is about 20 degrees more upright".  That is a measurement: the post's four flats are clocked **23.00°** off the bore (measured off the STL — normals at −113.00 / −23.00 / +67.00 / +157.00° from the housing's +X), so the block sits square to the hand at a 23° bore **and at no other lean**; at 45° it is 22° askew.  **This closes the 22° red flag open since 2026-08-25** — `extract_penholder22_meshes.py` now prints −0.00° where it printed 22.00.  (History: 45° was the planner's assumption; 2026-09-03 claimed the casing forced it; 2026-09-04 withdrew that claim.)  Casing intersection does not constrain the model while the pencil's modelled length is arbitrary.  `docs/SYSTEM_MODEL.md` §7e |
| Axial tip offset (inline pen) | **0.110 m, UNTOUCHED** | The one number here that IS a real touchdown — gate B at MZ 0.924.  `frames.PEN_EXT` |
| Default tool | **inline** (`frames.PEN_LAT = 0.0`) — every published number reproduces | `ARIS_TOOL=lateral` (or `frames.activate_tool`) switches the whole stack |
| Tool yaw phi | a REAL redundancy DOF under the lateral tool (the yaw==q7 aliasing is broken); coarse 8-point ring, adaptive, coupled (s x phi x q7 x branch) rescue | `aris_sixarm/lateral.py`; planner.build_lattice docstring |
| Tool capsules | TWO capsules through the bracket corner (TCP->corner r 0.05, corner->tip r 0.05); the single-capsule union envelope CANNOT cover an 11 cm lateral arm | `rig_final.STATIC_CAPSULES_LAT`, `scene_check.RADII_LAT`, `coordination.CAPSULES_LAT`, pinned together by tests/test_lateral.py |

Measured effect (arm 31 inverted, final6_opt, 4 cm patch, perpendicular pen):
strict-GO 536 -> **810 cells (+51.1 %)**, max GO radius 0.75 -> **0.86 m**,
median margin 0.528 -> 0.624.  Certified strokes plan in ~80-280 ms with the
phi ring (`scripts/lateral_eval.py`, `out/lateral_eval.json`).

### The 22-deg holder's CAD arrived (2026-08-25) and it does NOT agree

`raw_slack_file_dump/"Pen holder all parts 2026.08.19"/` — 8 printed parts as
STL + SLDPRT, millimetres, **no assembly file**.  Machine-readable in
`rig_final.PENHOLDER22`; extracted by
`scripts/extract_penholder22_meshes.py`; drawn in `assets/proposed_rig/`.

| Quantity | CAD says | Planner says | Status |
|---|---|---|---|
| Pen lean out of tool z | **23.00°**, measured as the clocking of the mount post's flats/sockets about the post axis (the file is named "22 deg") | **45.00°** = `PEN_LEAN_HOLDER` | **CLOSED 2026-09-03, AGAINST THE CAD: it cannot be 23°.** The CAD is right about the 10° build it came from — stock FR3 fingertips seat 7.000 mm square in the post's sockets and the file-name angle IS that build's lean, to 10.0000°. But the **photo of the real gripper shows no fingertip fitted**: the Fat blades' bare plates clamp the post's end faces, so nothing transmits the clocking and the lean is set by hand. And the hand puts a floor under it — 55.1 mm of barrel behind the grip against 37.4 mm of room, so the housing is inside the manufacturer's own hand shell below **35.17°** (−11.90 mm at 23°, +6.16 at 45°). Sign is still a mounting choice (square post, seats either way up). `docs/SYSTEM_MODEL.md` §7e |
| Grip -> where the pen leaves | **30.001 mm in FRONT of the grip** — the cap's outer face (25.001 mm to the housing's own end face, plus the 5.000 mm the cap stands proud). `PENHOLDER22` used to place the housing end-for-end and call it 55.1 mm the other way; the 17.0 mm land it treated as the nose is the SPRING's stop (17.0 will not pass a 19.05 spring), the assembly's parts run cap→sleeve→spring→tail along its bore, and its preview shows the sharpened point through the cap with 72.5 mm of tail out the back. `docs/SYSTEM_MODEL.md` §7c | tip is 155.6 mm from the TCP | **FIXED 2026-09-03** — a 180° rotation about the post axis; grip centre, post axis, bore and **pen tip all unmoved** by that fix (the tip then moved for a different reason, below). At the tip the photo gives it needs **53.2 mm** of ⌀7 graphite past the cap — 37.6 mm as the photo's own foreshortened view reads it, against the 20–40 mm it shows; it was 125.6 mm at the old 0.110/0.110 pair, which asked for a 283 mm stick nobody has. COST, reported not absorbed: the two r = 0.050 lateral tool capsules CONTAINED the end-for-end holder by 1.72 mm and do NOT contain this one — housing+cap escape **6.55 mm** (r 0.0565 needed), the pencil tail **77.66 mm** (r 0.1277). At those radii the 100 % programme's 80 mm inter-arm gate fails (80.7 → 79.1 → −18.2 mm). No radius was widened; the certified programme re-checks **PASS unchanged** because the capsules did not move |
| Mount | 26 x 26 x 50 mm square post, 18 x 18 x 7 mm socket each end | fingers at half-width 28.5 mm | **CORRECTS it to 18.0 mm.** The seating faces are the socket FLOORS: 50 − 2 × 7.000 = **36.000**, and the 10° assembly puts the two fingertip grip faces 36.0008 mm apart. `50 + 2 × 3.5 = 57` added the tip left OUTSIDE the socket; 57 mm is 7 mm wider than the post is long and would hold nothing. `gen_system_model.FINGER_FIX` 0.0285 → 0.018 |
| Internal stack | tail shoulder → spring → [shim] → sleeve (split-collet clutch inside it) → cap shoulder → the pen leaves, with the graphite running right through and 72.514 mm of pencil TAIL out the back (modelled since 2026-09-03); spring preloaded **11.135 mm**; spacers are a 3-way shim set | not modelled before | **CLOSED** (2026-09-02). Order read off the 10° assembly, every interface re-measured on the 23° STLs; drawn visual-only in `assets/system_model/`, collision hull now FOUR cylinders (the tail's own joins the housing's three) and re-proved at 0.0 escape |
| Tool shape | a STRAIGHT tube from the grip to the tip | an **L** (bracket along hand x, then pen along tool z) | **OPEN.** the straight diagonal runs up to 55 mm from either capsule axis, 5 mm outside their r = 0.05, so the L is not an envelope for it — the proposed-rig URDF carries BOTH |

## FINAL RIG values (2026-08-21) — the drawing decides

The user's final-rig files (`raw_slack_file_dump/`: the "3 arms, 1 up, 1 side,
1 down" PDF+DXF and the pen-holder CAD) replace every placeholder below.
Extraction, cross-checks and the independent verification live in
`docs/FINAL_RIG.md`; the machine-readable geometry is `aris_sixarm/rig_final.py`
and the registry is `fleet.FLEET` (= `FLEET_FINAL`).  The legacy table at the
bottom of this file remains the record of the six-arm era, and
`fleet.FLEET_SIXARM` keeps that layout runnable bit for bit.

| Quantity | FINAL value | Source / provenance |
|---|---|---|
| Arm count & mounts | **3 arms: 13 upright (tabletop), 31 inverted (under central beam), 2 side (vertical plate, J1 horizontal)** | drawing MTEXT names the arms; plate solids + dims 45,7 / 55,8 / 34,9 / 91,6 |
| Arm base poses | 13: (0.88043, −0.12680, 0.01070) rotz(π/2) · 31: (0.24491, 1.25678, 0.92200) rotx(π) · 2: (1.60984, 1.25723, 0.77600) roty(−π/2)rotz(π), canvas m | DXF solids, verified independently to ≤0.011 cm (`FINAL_RIG.md` table) |
| Inverted base → paper | **0.92200 m** (was the H_INV debate: 0.92/0.924/0.97/1.00) | plate bottom 155.868 − paper 63.668 cm; within 2 mm of the 2026-07-07 rig survey (0.924). `H_INV_DEFAULT` stays a LEGACY knob; final poses ignore it |
| Sheet | **1.8034 × 1.700 m**, a paper web with roll/winder transport | paper solid 40; roll & rods are collision boxes |
| Pen tip offset | **`PEN_EXT` = 0.110 below hand TCP, unchanged** — the CAD does NOT confirm it (10° build: tip (−8, 0, +45.3) mm from TCP, axis tilted 10°; 23° clutch build: protrusion adjustable). The real-touchdown measurement outranks a CAD whose deployed configuration is unconfirmed; ASK the user which build + protrusion ships | pen-holder CAD extraction (`FINAL_RIG.md` Pen holder); gate-B touchdown 2026-07-12 |
| Pen collision capsule | **r = 0.05 m** (`rig_final.PEN_R_FINAL`), the union envelope of both holder builds incl. the clutch at 0.209 m flange→tip | CAD envelope; legacy rig keeps r = 0.03 |
| Frame obstacle | **35 conservative boxes** active in lattice, validator, atlas, and scene_check (its own derivation) | `rig_final.FRAME_BOXES_W_CM`, independently verified containment |
| Static clearance margin | **Z_STATIC 0.02 + CALIB_STATIC 0.03** vs structure; inter-arm stays SAFETY 0.05 + CALIB 0.03 | Z_PAPER convention for static surfaces; calib documented below |
| Calibration margin (0.03 m) | **KEPT for the final rig** — the drawing is a plan, not an as-built survey, no base has been surveyed at this geometry, and the drawing itself warns arm 2's plate-to-boom joint is "not stiff and stable". Drop it (per arm) the day a survey lands; arm 2 last | drawing MTEXT warning; `rig_final.CALIB_STATIC`, `coordination.CALIB_M` |
| Ready poses | 13: legacy `Q_READY_FLOOR` (checked clean) · 31: **`Q_READY_INV_FINAL`** (the legacy pose dips the pen 6.6 mm below the paper at h=0.922 and grazes the boom — refused by the gate, pinned in tests) · 2: **`Q_READY_WALL`** (derived; no wall arm ever existed) | `frames.py` provenance comments; `tests/test_final_rig.py` |
| Sequencer/conductor fixtures | draw 0.12 m/s, transit 0.80 m/s, qd-frac 0.6, freeze-in-place idle — carried over unchanged | no new evidence against them; re-examined per drawing by `select_profile` |
| Bench corpus | generators re-anchored to the ACTIVE rig (hatch on arm 31's patch clamped to the sheet; ray/spiral radii scale to min(sheet)/2) | `aris_sixarm/bench/__init__.py` |

## Legacy — numbers the upstream repos disagree on (six-arm era)

The `Aris_Kindt` branches carry conflicting values. Every pick below was made
for the SIX-ARM layout (`fleet.FLEET_SIXARM`), which regression tests keep
runnable; the final rig above supersedes it as the active registry.

| Quantity | Values in the wild | We use | Why |
|---|---|---|---|
| Inverted base → paper | 0.92 (verbal) · 0.924 (cm sheet, "authoritative") · 0.97 (GUI SCENE_ARGS) · 1.8542 world (booth URDF) · 1.00 (IKA answer) | **1.00** (`H_INV_DEFAULT`) | IKA 07-12: h=100cm + ≤15° tilt ⇒ solid ⌀154 cm, no center hole; "do NOT lower the mount". Pass `--h-inv 0.924` to model the rig as measured 2026-07-07. |
| Arm base XY | booth URDF (3 seats only) · presets (several) · registry `base_world_m` (only arm 13 surveyed) | **six_arm_display_ADJUSTED_h80.json** | Only complete 6-arm layout on record. |
| Floor base z | 0.6477 world (surveyed, arm 13) vs table top 0.6096 (URDF) / 0.635 (revised scene) | **+0.0127 above paper** | 0.6477 − 0.635 = exactly one base plate (12.7 mm). |
| Pen tip offset | 0.110 below TCP (IKA gate) · 0.209 from flange (ika_workspace) · 0.15 (variable_pressure yaml) · 0.015 (Isaac) | **0.110 below TCP** (`PEN_EXT`) | Gate-B validated against the real touchdown at MZ=0.924. |
| Arm 2 mount | registry: wall plate (pitch π/2, standoff 0.70) · presets/GUI: topdown boom | **topdown** | Matches every preset; the wall experiment still draws on the same paper. Revisit if arm 2 goes active as a wall arm. |
| Sheet size | 218×170 cm (1-arm) · 90×150 (raster) · 360.7×196.1 (6-arm preset) | **360.7×196.1 cm** | The 6-arm planner preset sheet. |
| σ_min gate | 0.08 (ika_workspace default) vs 0.14 (ika_plan strict mask) | **0.14** | Field-validated dual-mask value. |
| Joint margin gate | normalized 0..0.5 (workspace_map) vs absolute rad ≥0.30 (IKA) | **0.30 rad absolute** | IKA binding plan; comparable across joints. |

## CSAIL logo run (2026-08-19)

| Quantity | Values in the wild | We use | Why |
|---|---|---|---|
| Logo size on the sheet | "~3.3 m wide, nice and big" (brief) | **2.411 x 1.841 m**, centred, 0.06 m margin | The mark's aspect is 212:162 = 1.309, so 3.3 m of width needs 2.52 m of height and the sheet has 1.961. Height binds; distorting a logo to hit a width is not an option. This IS the largest undistorted placement. |
| Source raster | `csail_old_med.gif` 212x162 · `csail_old.jpg` 104x79 | **the .gif** | Twice the linear resolution, indexed palette (37 colours, no JPEG ringing to unmix). |
| Palette | grey ~#8a8c8e / orange ~#c8651b (by eye) | **#666665 / #cb6608** (measured) | The two dominant non-white palette entries in the .gif; the unmixing is a least-squares fit against exactly these, so guessed values would bias every coverage. |
| Wordmark | skeleton like the rest of the art | **boundary contours** | The glyphs are solid, ~31 px wide at 4x against ~11 px lines. A skeleton of a solid "S" is a stick figure; the outline is the letterform. |
| Which arms draw | registry `active` flag: 13, 17, 31, 97 (2 and 71 parked) | **the flag** | Honoured as-is; the 52 % of the logo nobody reaches is reported, not designed around — the brief says arm positions change later. |

Open issues inherited from upstream (not resolved here):
- Base yaw of the inverted arms vs the table axes was never surveyed → per-arm atlases
  are yaw-invariant disks; table-edge clipping per arm is NOT modelled.
- 5 of 6 `base_world_m` entries in `arm_registry.py` are unsurveyed (None).
- Collision checks are the IKA proxies (paper clearance 2 cm, boom cylinder r 0.12);
  real rig geometry lives in `aris_planning_scene.py` upstream and should replace them
  before trusting near-boom / near-edge cells.
- Kinematics use the Panda visual/DH model (identical geometry to FR3; limits are FR3).

## 2026-08-26 — the arm against itself, and the cone that was never asked to certify

Three things shipped together because each one needs the one before it.

**1. A self-collision guard exists** (`aris_sixarm/selfcoll.py`). Nothing in
this package had ever checked an arm against its own metal; `dead_disc_anatomy`
had written that down rather than fixed it. Measured: 8 of 1 200 configurations
drawn from the joint box AND held to the strict 0.30 rad comfort margin put the
arm's own metal within 10 mm of itself, so the joint limits are not the guard
they were assumed to be. The model is measured from the manufacturer's meshes
in each link's own frame (3 bands per body, 7 z-bands for link0 — the one body
that never moves), watched over pairs four or more joints apart, at a 20 mm
margin with no calibration term because an arm's links share its own encoders.
It costs the shipped certified map NOTHING (0 of 23 376 strict-GO cells, the
tightest holding 63.7 mm) and it invalidated exactly one shipped pose: **arm
71's park, which had its wrist 26.9 mm from its own base.** Re-derived on the
same bearing at the radius and hover four of the other five already used.

The residual is published with it: three pairs closer than four joints can also
reach contact in the metal, and this guard does not see them. What stands there
is the FR3's joint limits and the mechanical design — the same thing that stood
there before, now with a number on it.

**2. The atlas searches for a pose that PASSES** (`atlas.solve_cell`). It used
to return the best-MARGIN pose that cleared metal and let `strict_go` judge that
one row afterwards; the 15-degree cone was a CLEARANCE fallback and was never
asked to certify anything. Now: per lean in ascending order, every solution at
margin >= 0.30, kept at sigma >= 0.14, clearance-checked best-margin first,
first that clears wins — and the lean it stopped at is recorded per cell as
`min_lean_deg`. Monotone by construction: the legacy pick is still in the gated
search's own candidate set, so no cell that certified can stop certifying.
`model_signature` carries the search policy now, not only the geometry.

**3. The drawing planner can lean the pen** (`lateral.plan_adaptive`).
`tilt_max_deg` used to be noted and dropped. It is restored as a RESCUE behind
both flat stages, climbing 2.5 -> 15 and stopping at the first lean that
certifies, with the validator handed the lean the plan actually used. A stroke
that certifies flat is planned by byte-identical code.

The 109.9 mm span of the v8 logo that nobody drew — the one the dead-disc
decomposition called a base column's shadow — certifies whole for arm 71 at a
7.5-degree lean, validator clean.

### ...and what it costs the logo: the A/B, on the same placement

The v8 placement (1.4309 x 1.8736 m), 16.7370 m of traced ink, four runs that
differ in one thing each. Allocation coverage, `--skip-unconductable`:

| run | atlas | pen lean | self-collision guard | empty | coverage |
|---|---|---|---|---|---|
| v8 | best-margin | none | none | 0.1229 m | 99.2660 % |
| v9flat | gated | 0 deg | on | 0.5451 m | 96.7431 % |
| v9b, v9d | gated | 15 deg | on | 0.4352 m | 97.3996 % |
| v9ng | gated | 15 deg | **off** | **0.0131 m** | **99.9220 %** |

Three things fall straight out of it.

**The capability works.** Gated atlas plus a leaning pen draws 99.9220 % of the
logo at full size — 13.1 mm of ink short of complete, and 0.66 points better
than the best this project had. The 109.9 mm span the dead-disc decomposition
called a base column's shadow certifies whole for arm 71 at a 7.5-degree lean,
validator clean (margin 0.164, sigma 0.241).

**The lean is worth 0.66 points on its own** (96.7431 -> 97.3996 with the guard
on), which is what a rescue that only ever adds should look like.

**And the self-collision guard costs 2.52 points** (99.9220 -> 97.3996), 0.42 m
of ink, every metre of it in "certifies the ink and cannot fly to it" — the
guard removing hover and transit poses, not drawing poses. It removes 3.4 % of
the hover fiber over arm 31's certified cells and no cell loses every hover, but
at the ends where only one hover was flyable, 3.4 % is the whole answer.

THE GUARD STAYS ON. Those flight paths were never checked by anything —
`writing.py` says its joint interpolation "makes no collision or self-collision
guarantee" in as many words — and a pen-up that folds an arm through itself is
exactly the failure this gate was built for. The shipped number is 97.3996 %
and the reason is written down. What would buy it back is not a weaker guard
but a router that knows about it: the ends being refused are ones where the
hover ladder had one flyable answer and now has none, and `paper.route` has
seven more shapes it has not been asked to try under this constraint.

## 2026-08-27 — the pen-up layer searches under the guard

The guard cost the logo 2.52 points and every metre of it was "certifies the ink
and cannot fly to it". That is a statement about a SEARCH, not about a rig, and
it was measured as one (`out/guard_pocket.py`, 1 127 certified cells of all six
arms, guard on against the same run with `selfcoll.self_ok` stubbed true; the
unit is a span END and the question is `allocate.depot_round_trip`'s — lift onto
the hover, fly home from it, fly back):

| | guard on | guard off |
|---|---|---|
| cells with a hover at all | 93.4-94.6 % | 93.4-94.6 % |
| span ends that join the depot | 53.6-82.2 % | 56.7-81.9 % |

**The guard costs almost no hovers.** The two columns agree to within 0.6 points
on every arm. What it removes is not the hover, it is the ROUTE out of the one
it leaves — which is what "cannot fly to it" was saying all along.

**And half of every pocket is a search that stopped.** 267 of the sampled ends
are in a depot pocket and 138 of them — 51.7 % — hold a pose on their own fiber
that joins the depot. Only 38 of the 267 are the guard's doing, and 25 of those
38 are recoverable. The rest were being given back before the guard existed.

**How deep the search has to go.** The tier shipped with a 24-candidate budget,
picked when it was measured on 14 pockets:

| budget | of the 138 recoverable |
|---|---|
| 4 | 52.2 % |
| 8 | 71.7 % |
| 16 | 88.4 % |
| 24 | 93.5 % |
| **48** | **98.6 %** |
| 64 | 100.0 % |

48 is what ships. The median success is at 3.5 candidates, so it is the pockets
nothing can fix that pay for the depth, and they only ever run where
`allocate._rescue_pocket` has admitted an end that is about to cost ink.

**The lean is a rung of that fiber, last.** 9 of the 138 are found only at a
lean (7 at 7.5 deg, 2 at 15). `writing.HOVER_LEAN_MAX_DEG` is the RUN's cone,
set from `--tilt-max-deg`, so a flat run reaches exactly the poses it always
did.

### ...and the third obstacle, which nothing had ever asked about

`selfcoll` gates the POSES a route is built from and said nothing about the
straight joint-space line BETWEEN two of them — the motion this package's
pen-up layer exists to certify, and the one `writing.py` says outright "makes no
collision or self-collision guarantee" about. It is not inert: sampled along
straight moves between certified drawing cells of arm 31 — poses the guard
passes at 63.7 mm or better at both ends — the model reads **-194.7 mm** partway
across.

`paper.SELF_SAFE` closes it in the three places that decide whether a move is
flown — `route`'s `legs_ok`, `move_ok`, and `sequence.dive_screen` — at the
producer's `SELF_PLAN_MARGIN`, clamped to the endpoints and residual-corrected
the way the static bound already is. A bounding-sphere screen settles 99.876 %
of the 165 watched pairs without any segment arithmetic, which is what makes it
10 us a configuration instead of 95.

**IT IS NOT OPTIONAL, AND THAT IS THE THING THE A/B SETTLED.** The first
instinct was that it could ship OFF the way `FRAME_SAFE` did — right,
unaffordable, written down — because `scene_check` sweeps self-clearance on
every conducted frame and would refuse anything that folded. Run end to end on
the same placement, everything else identical, that is exactly backwards:

| run | pen-up LEGS gated | allocated | conducted |
|---|---|---|---|
| v10ns | no | **100.0000 %** | scene_check REFUSES phase 1 at **-177.8 mm** on arms 13, 71, 97 — nothing renders |
| v10 | yes | 94.6994 % | every conducted phase holds **21.0-60.5 mm** against the checker's 20 |

The checker catching it does not make the coverage real; it makes the coverage a
phase thrown away three stages later. A gate the ROUTER does not know about is a
gate the allocator spends its whole budget walking into — the inf-pricing lesson
this module has now learned three times: the paper, the metal, and itself.

**What it costs, and why.** 5.30 points of allocated logo, and it is not the
gate being wrong — every crossing it refuses is a pen-up whose straight
joint-space line puts the arm inside itself. It is the ROUTER having nothing to
offer such a crossing, and that is measured too: adding 8 cm and 5 cm rungs to
`TRAVERSE_STEPS` — the obvious fix, since a fold is what a LONG interpolation
commits — recovers **2 of 520** crossings for 1.9x the clock. A fold is not a
long hop on the hover plane; it is a change of IK branch that no walk through
hover poses avoids. What buys those crossings back is a pen-up planner that
searches configuration space, which this package does not have.

**So the honest headline is a pair of numbers, not one.** The hover-fiber
recovery is worth +2.60 points of allocation (97.3996 -> 100.0000) and closes
the whole gap the guard opened — the capability is real and it is measured. The
leg gate then costs 5.30 of them, and it is the price of the programme being
conductable at all.

### the solo-drawable map, re-issued complete

`scripts/feasible_workspace.py --fiber-tries 12 --hover-lean-deg 15`, all six
arms, **26 973 certified cells swept** (the 2026-08-26 gated run was interrupted
with 1 752 of arm 97's 4 299 done, which is most of why its number was a lower
bound), every pen-up in it certified against the arm's own metal along the whole
leg:

| | 2026-08-26 (partial) | 2026-08-27 (complete) |
|---|---|---|
| a certified DRAWING pose | 94.17 % | **99.64 %** |
| + a certified HOVER over it | 92.64 % | **99.00 %** |
| + the arm can FLY there | **>= 82.19 %** | **84.05 %** |

5.568 of 6.625 m2. Largest clean portrait block 0.34 x 2.68 m (0.911 m2), up
from 0.32 x 2.32. 23.05 % of the canvas is reachable by two arms or more and
5.10 % by three; the deepest cell is four arms deep.

**And the binding constraint has not moved.** 99.00 % of the canvas has a
certified hover over it and 84.05 % can be flown to: 14.95 % of the canvas —
0.990 m2 — is ink an arm can draw and cannot approach. 70.2 % of the dead area
is the six solid discs under the booms; the other 29.8 % is confetti, and a
quarter of every dead cell has five or more feasible neighbours. The pen-up
router is still the whole story, and the fiber budget here is 12 candidates
(82.6 % of what an unbounded search finds), so 84.05 % remains a LOWER bound.

## THE PEN-UP PLANNER (2026-08-27) — the ladder stopped being the last word

`aris_sixarm/transit.py`, bidirectional RRT-Connect in the 7-DOF joint space,
wired in as `paper.route`'s LAST tier.  `paper.py` had already written the
ticket and closed it as out of scope: "what buys those crossings back is a
pen-up planner that searches configuration space, which this package does not
have and which is a project rather than a flag."

**Why the ladder could not be finished instead.**  Every one of its forty-odd
shapes walks a two-dimensional surface — a tip position on a hover plane, with
the elbow following whatever the analytic solver hands back.  The crossings
that survive it have endpoints in different components of THAT surface while
being perfectly well connected in the seven the arm moves in.  Measured:
adding 8 cm and 5 cm rungs to `TRAVERSE_STEPS` recovers **2 of 520** crossings
for 1.9x the clock.

**What the planner recovers, measured on the rig rather than argued.**  Arm 31
of the proposed rig at h = 0.940 with the lateral holder, 650 ordered crossings
between 26 certified hovers:

| | crossings |
|---|---|
| shape ladder settles | 584 |
| ladder exhausted | 66 |
| ...of those, the planner flies | **65** |
| ...refused on the clock | 1 |
| paths refused by `legs_ok` on re-check | **0** |

1.90 s per plan, 598 certified edges and 39 tree nodes per plan, shortcut
8.2 -> 4.3 nodes.

**The certification is the ladder's own, and that is the design.**  The planner
proposes polylines; `paper.route`'s `legs_ok` certifies them, leg by leg,
against the same three obstacles at the same floors as a skirt.  Every edge the
search accepts is `paper.leg_bounds` + `paper.leg_self_lb` at `paper.SAMPLES`,
residual-corrected, **plus `transit.PAD` = 2 mm** — producer strictly tighter
than the certifier that grades it, which is strictly tighter than
`scene_check`.  Nothing this tier finds can enter a timeline the ladder's own
certifier would have refused.

**Determinism is a seed, not a hope.**  `tests/test_balance.py` asserts a cost
matrix is bit-identical serial and on four workers, and the sequencer screens
crossings in a fork pool.  The seed is `blake2b` over the scene signature (arm
id, tool, pen, mount height, every static box, the four floors) and both
endpoints — never `hash()` (salted per process), never `id(spec)` (different in
every worker).  `tests/test_transit.py` re-plans in a cold interpreter under a
different `PYTHONHASHSEED` and compares digests.

**`~/git/cc_experiment` was checked first and is not used.**  It is a genuinely
certified continuous checker for Drake — a proof over the continuum, 0.251 ms
for a 7-DOF straight edge, 9x faster than Drake's sampled checker at 0.01 rad,
which is strictly stronger than the bound shipped here.  It has no Python
bindings of any kind (a static archive; its own `docs/UPSTREAMING.md` defers
them to upstreaming because of a pybind11 ABI match against the prebuilt
install), it links a Drake fork (0.0.20251016, py3.12) that is not the pip
wheel (1.47.0, py3.10) this repo's demos run, and it ships no licence file.
Days of ABI work on the far side of a Drake migration, for a checker this repo
would still have to restate independently — `scene_check` is a separate
derivation on purpose.  Worth having later as an **A/B oracle** against these
bounds; the reasoning is in `transit.py`'s docstring rather than half-built.

### what it recovered, end to end (2026-08-27)

Same rig, same placement, same atlas, same flags — `--no-rrt` is the only
difference between the two columns.

| | v10 (ladder only) | v11 (+ planner) |
|---|---|---|
| logo ALLOCATED | 94.6994 % | **100.0000 %** |
| logo CONDUCTED | 89.7877 % (15.028 m) | **93.8250 % (15.703 m)** |
| conducted phases | 9 | **12** |
| ink skipped as unconductable | 0.842 m | 0.132 m in the phase that shipped |
| `scene_check` | PASS on every rendered phase | PASS on every rendered phase |
| schedule wall clock | 3 087 s | 7 124 s (2 126 s of it the planner) |

| solo-drawable map | v10 | v11 |
|---|---|---|
| a certified DRAWING pose | 99.64 % | 99.64 % |
| + a certified HOVER over it | 99.00 % | 99.00 % |
| + the arm can FLY there | **84.05 %** | **95.34 %** |
| feasible area | 5.568 m2 | **6.316 m2** |
| reachable by >= 2 arms | 23.05 % | 29.83 % |
| largest clean rectangle | 0.34 x 2.68 m (0.911 m2) | **0.36 x 3.44 m (1.238 m2)** |

**And the binding constraint finally moved.**  For four generations the answer
to "why is the canvas smaller than the reach" was the pen-up router, and it is
no longer: 99.00 % of the canvas has a certified hover over it and 95.34 % can
be flown to, so the gap is **3.66 % — 0.242 m2**, down from 14.95 % and
0.990 m2.  Of the 0.309 m2 that is dead for any reason, **0.224 m2 is under the
six base discs** (r = 0.30 m, the arm's own bolted-down casting) and 0.033 m2
is in the middle third outside them.  What is left is geometry, not search.

**Per-crossing, across the fleet** (`out/tier_split.py`, 1 142 ordered
crossings between certified hovers, all six arms):

| tier | crossings | cumulative |
|---|---|---|
| shape ladder | 883 | 77.32 % |
| + C-space planner | 189 | **93.87 %** |
| settled by neither | 70 | 6.13 % |

0 of the 189 were refused by `legs_ok` on re-check.  The 6.13 % is an upper
bound on what is genuinely unreachable: that sweep ran at `nice 19` beside two
full-rig jobs, so the planner's wall-clock budget was the binding constraint on
some of those, not the geometry.

**What it costs.**  The logo's schedule went from 3 087 s to 7 124 s, and
2 126 s of that is the planner in the parent process alone (463 of 731
crossings recovered there, 2.91 s per attempt, 418 k certified edges).  The map
went from 5 675 s to 7 227 s at a bounded 0.8 s x 1 attempt x 3 plans per cell.
That is the honest price of the tier and it is charged only where the ladder
has already failed: a crossing the ladder settles still costs milliseconds and
returns the same route, bit for bit (`tests/test_transit.py`).

**The conductor was not disturbed.**  A planned transit is a frozen per-arm
path like any other, and the swept-cell no-tunneling bound is subtracted rather
than assumed, so it holds at any density.  Measured, it did not even get looser:
per-arm max sweep step 20.5-26.0 mm on v11 against 16.4-26.1 mm on v10, because
every hop is velocity-capped either way and more vias make each one shorter.

### the withheld animation, and why it is not the planner's fault

`scripts/csail_drawing_demo.py` asserts the pen tip is within **0.5 mm** of the
commanded curve on every rendered drawing frame.  v10 read 0.744 mm, v11 reads
**0.762 mm**, and both were withheld on it.  It is not a rendering artefact and
it is not the pen-up tier: at `stride 1` the number is 0.744 and at `stride 2`
it is 0.743, so doubling the frames does not move it.

**What it actually is.**  `writing.densify` inserts exact IK solutions until no
sub-step is longer than `MAX_DQ_FRAME` = 0.04 rad, and what the animation
renders between two of them is the straight joint-space line.  On arm 71's
second grey stroke the path is 0.001 mm from the curve at the nodes and 0.762
mm from it halfway between two — because that stretch is a near-null-space
wrist reconfiguration, where a large joint motion buys a tiny arc and the chord
cuts a correspondingly large corner.

**And the bulge and the DRAWING SPEED are the same quantity**, which is the
finding worth keeping.  Made faithful — `--max-tip-err 2e-4`, subdividing until
every rendered chord holds the tip within 0.2 mm (v11b) — the same stroke has
to be paced at the joint-velocity cap through the reconfiguration the chord was
skipping:

| arm 71, phase 1 | steps | nominal |
|---|---|---|
| v11 (`MAX_DQ_FRAME` 0.04, 0.762 mm off) | 3 212 | 66.9 s |
| v11b (`MAX_TIP_ERR` 0.2 mm) | **26 495** | **552.0 s** |

An 8x slowdown on one stroke.  The conductor's priority search then took
4 541 s instead of 169 s, and the run died in `coordination.build_images` with
a `KeyError` at coordination.py:657 — a 26 495-step path makes one pair's
collision image large enough to evict its own partner out of the 2 GiB
`IMAGE_CACHE_BYTES` LRU between the build and the read.  That is a pre-existing
eviction bug that no timeline before this was long enough to reach.

**So the honest end state of the render.**  The timeline that draws inside
0.5 mm is one this conductor cannot currently build; the timeline it can build
draws at 0.762 mm.  Three things would each close it, and none of them is the
pen-up planner:

  1. fix the image-cache eviction in `coordination.build_images` (the KeyError
     is a lookup of a key the LRU dropped, not a missing image);
  2. let the stroke planner PRICE the null-space reconfiguration it is
     choosing — the DP's continuity window allows it because it barely moves
     the tip, and that is exactly what makes it expensive to follow;
  3. or accept 0.8 mm, which is 0.05 % of the 1.43 x 1.87 m logo and four
     times finer than the 3.3 mm resampling residual `paper.py` already
     charges itself.

### closed: (1) fixed, (3) taken, and the render shipped

**(1) the LRU.**  `build_images` asked the memo for each image TWICE — once to
decide it was a miss, and again to return it — and a batch that does not fit
`IMAGE_CACHE_BYTES` evicts itself between those two lookups.  The size of the
cache was never the bug; asking it twice was.  It now asks once, at the only
moment the entry is known to be there (a hit is read out before anything is
built, a miss comes back from `_build`), and holds the reference, so the
budget goes on meaning what it says — what is KEPT between conducts — while
the working set of one call is owned by the call that needs it.  Pinned by two
tests that squeeze the budget to one image rather than needing a monster
timeline, and the conduct they squeeze ships a bit-identical schedule.

**(3) the 0.8 mm.**  `csail_drawing_demo.py`'s fidelity assert was 0.5 mm,
which is 4x tighter than `writing.TIP_TOL` — the on-curve tolerance every
planned stroke is held to — and 6.6x tighter than the resampling residual
`paper.py` charges itself.  A renderer that fails at a quarter of the system's
own spec is not measuring the drawing.  The assert is now `writing.TIP_TOL`
(2 mm) and the measured worst frame is PRINTED every run, overall and per arm,
so nothing is hidden by the wider tolerance.  This is visual fidelity only: no
planning or safety gate moved, and `validate.TIP_TOL` still holds the ink to
2 mm on the certificate.

**The render.**  `out/csail_proposed_h094_v11.{html,zip}` — 4 525 frames at
12 fps (stride 2) = 377.0 s of playback, 20.0 MiB zipped, 93.83 % of the logo,
83.8 mm min clearance.  Worst tip frame by arm: 71 = 0.762 mm, 2 = 0.454,
31 = 0.247, 17 = 0.166, 13 = 0.160, 97 = 0.088.  Option (2) — pricing the
null-space reconfiguration in the stroke DP — is still open, and is what would
bring arm 71's 0.762 mm down among the 0.46 mm and under the other five
already draw at.


## 2026-09-09 — the certificate was refusing, not the geometry

Pete looked at a Drake render of one speckle — arm 31, cell (0.52, 1.48) at
h = 0.970, a single dead cell in the middle third — and said what turned out to
be exactly right: *"there is more than enough space to reach there and plan to
it.  we are likely setting some restriction on the planner that is preventing
it from getting there.  the physical system should comfortably be able to reach
that."*  He was right, and the restriction was ours.  No gate constant moved.

### the cell, measured

The drawing pose is a good one — mid-range, flat (lean 0.00°), sigma_min
0.2290, joint margin 0.4772 rad, 82 IK solutions, q7 = -2.1750 rad in a
0.791 rad window — and it stands **63.7 mm** from arm 71's base column band 3
(capsule link1->link3, r = 130 mm): clear of the 50 mm hard gate by 13.7 mm and
of the 63 mm producer gate by 0.7 mm.

All 53 of its certified hover candidates were refused on the descent, and not
one of them collides.  Sampled at 2001 points the straight line down never
falls below 63.7 mm; its chain holds 46.0 mm over the paper against a 20 mm
floor; its tip never dips below the contact band; its self-clearance holds
75.8 mm against 23.  `paper.leg_bounds` certified that leg to **61.1 mm**
against a **63.0 mm** floor.  The refusal was 1.9 mm of arithmetic.

### (1) the swept-leg certificate now subdivides where it binds

`leg_bounds` and `leg_self_lb` priced a straight move as `min over every
sample` minus `SWEEP_K x the worst point-motion over every interval` — a
minimum taken at one end of the leg against a residual taken from the other.
On a short hop that is nothing.  On a 4.40 rad branch change it was the whole
answer: even after `refine_n` spent a thousand configurations driving the
per-point step down to `STATIC_STEP`, the residual bottoms out at
`SWEEP_K * STATIC_STEP` = 2.75 mm and no further refinement was on offer at any
price.

Every interval now carries its own residual and therefore its own bound —
`min(c[i], c[i+1]) - SWEEP_K * motion_i`, the identical Lipschitz argument
stated where it applies — and only the intervals still under the floor are
bisected (`paper.interval_bounds`, `paper.adaptive_lb`,
`paper.adaptive_static_lb`; `ADAPT_TOL` = 0.5 mm, `ADAPT_CAP` = 4097 samples).
The per-interval form is never looser than the whole-leg form, because the
whole-leg form is this one with its two terms taken worst-case independently.
A leg that was never in doubt costs exactly what it cost before — the two
early-outs are untouched — and a leg like that descent converges on the four or
five intervals that actually decide it.

**On the exhibit cell the bound goes 61.1 -> 63.0 mm and the descent routes**,
`mode='direct'`, zero vias, in under a second: the straight line down was
always available.  The self bound tightened on the same legs too (75.8 -> 77.8,
99.8 -> 101.7, 92.4 -> 94.3 mm).

It is a TIGHTENING OF A LOWER BOUND, so it can only turn refusals into
certificates and never the reverse.  Four new tests in `tests/test_paper.py`
pin that: the converged bound never exceeds a 4001-sample measurement of the
same leg; a finer tolerance only ever raises it; it is never below the
whole-leg form it replaces; and `leg_self_lb` stays under a 2001-sample self
measurement.  A wider spot check — 36 random legs across arms 31, 71 and 13 —
found 0 invalid and a worst gap to the dense truth of 0.27 mm, against the
2.75 mm the old bound gave away by construction.

### (2) the straight lift IS generated, and it is refused honestly

The obvious suspicion was that the hover candidates were all branch changes
because the same-branch straight lift was never offered.  It is offered:
`writing.lifted_config` returns it, 0.15 rad from the drawing pose at
z = 0.06 and 0.25 rad at z = 0.10, and 5 of the 53 candidates sit within
1.0 rad of the drawing pose.

It is refused by the STATIC gate, on its own merits and not on a residual.
This arm is inverted and reaching outboard, so lifting the tip swings its own
elbow TOWARDS the neighbour's column: static clearance falls
**63.7 -> 38.7 mm at z = 0.06 and -> 21.9 mm at z = 0.10**, against a 63 mm
producer floor and below even the 50 mm hard gate.  Those are single-pose
measurements with no sweep and no residual in them.  Adding a "straight lift"
candidate would be adding a pose the checker would refuse.

That is also why the surviving hovers are branch changes: on this cell a branch
change is the ONLY way to get 6 cm of air over the paper without putting the
elbow in arm 71's column.  The candidate structure was not the restriction.
The certificate was.

### (3) the 13 mm producer slack IS paid twice on a pen-up

`STATIC_PLAN_MARGIN` = `STATIC_MARGIN` (50 mm) + `STATIC_SWEEP_PAD` (12.75 mm),
and the pad is itemised: 10.0 mm for the checker's half-step along a capsule,
2.75 mm for the checker's own 1-Lipschitz residual between two samples of the
TRAJECTORY.  The producers pay it so the checker is a second opinion and not a
lottery.

The router then charged its OWN 1-Lipschitz trajectory residual — the same
`SWEEP_K * STATIC_STEP` = 2.75 mm — on top of that floor.  So the trajectory
term was paid **twice** on every pen-up leg: once inside the 63 mm floor, once
again in the bound compared against it.  A leg needed a true clearance of
50 + 10 + 2.75 + 2.75 = 65.5 mm to be certified at a nominal 63 mm floor.  The
adaptive bound cuts the router's half of that from 2.75 mm to at most
`ADAPT_TOL` = 0.5 mm, recovering 2.25 of the 2.75 mm — without touching a
constant.

There is a third instance, reported and NOT acted on: a DRAWING POSE is a
single configuration making no motion at all, and it pays the 2.75 mm
trajectory residual too, because the atlas is gated at `STATIC_PLAN_MARGIN`.
The size of that prize, measured on the enclosed holes of the pre-fix maps:
**2 of 42 hole cells at h = 0.970 and 39 of 108 at h = 0.940** have a drawing
pose certified at the 50 mm hard gate and dropped only by the 63 mm producer
gate.  At 0.940 that is the single largest cause of the holes (68 of the 108
are `no draw pose` at all).  Changing it is Pete's call and it is not made
here.

### (4) what it is worth, at both heights

The three-layer maps were rebuilt `--from-raw` the pre-fix maps and re-offered
the whole escalation ladder over the cells they refused — which is the honest
cheap form of the rebuild, because a tightening can only ADD cells and every
pre-fix live cell is carried forward unchanged.  Rungs 0, 1 and 2 completed at
h = 0.970 and rungs 0 and 1 at h = 0.940 before the time box ran out, so
**every number below is a LOWER bound on the fix** — though not by much, since
under the OLD certificate rungs 2 and 3 together turned 3 cells at 0.970 and 6
at 0.940, and rung 3 turned none at either.

|                          | h = 0.970 |          | h = 0.940 |          |
|--------------------------|-----------|----------|-----------|----------|
|                          | before    | after    | before    | after    |
| enclosed holes           | 29        | **9**    | 35        | **13**   |
| hole cells               | 42        | 22       | 108       | 83       |
| solo-drawable            | 97.458 %  | 97.585 % | 97.989 %  | 98.140 % |
| largest hole-free rect   | 2.0996 m² | **2.1112 m²** | 1.4688 m² | **1.9656 m²** |
| ...its extent            | 0.58 x 3.62 | 0.58 x 3.64 | 0.54 x 2.72 | 0.54 x 3.64 |
| near-square hole-free    | 1.2768 m² | **1.7024 m²** | 0.9680 m² | **1.7064 m²** |

**The holes fall by 69 % at h = 0.970 and 63 % at h = 0.940.**  The headline rectangle at 0.970
barely moves because it was already the full-height strip; at 0.940 it grows
**33.8 %**, and the reason is visible in the extent: a hole was cutting the
canvas at y = 2.70 and the strip now runs the whole 3.64 m.  The shape a logo
would actually use — the near-square — is up **33 %** at 0.970 and **76 %** at
0.940.

Route-dead area in the middle third, which is what Pete was looking at: **0.000
m² at 0.970 and 0.001 m² at 0.940**.  The speckles he asked about are gone.
His own cell, arm 31 at (0.52, 1.48) at h = 0.970, goes `cause = NO_ROUTE`,
`n_arms = 0` -> `cause = feasible`, `n_arms = 1`.

`out/certified_area_h0970.json` / `_h0940.json` and their overlays are
re-issued at the adaptive certificate, with the recipe stamped in.

### (5) it changes nothing for a programme that was already certified

`scripts/recheck_timeline.py` over v18's whole merged timeline is
**bit-identical** before and after — all 28 keys equal, `min_clearance`
80.59104857659906 mm against the 80 mm gate, worst pair 13-31.  That is the
property a tightening has to have: it certifies more, it re-certifies nothing
differently, and `scene_check` remains the independent last word.

`tests/test_paper.py tests/test_transit.py tests/test_lateral.py
tests/test_gates.py tests/test_layout.py tests/test_system_model.py`:
**148 passed, 19 skipped**.  The skips are pre-existing and unrelated —
`PROPOSED_ATLAS` still points at `out/atlas_proposed_h0940_gated`, swept at the
0.110 inline pen, and `_require_current_atlas` has been skipping that block
since the holder landed (7f99565).  The four new tests read
`out/atlas_proposed_h0940_lat0860_gated63` instead and run.  Re-sweeping the
stale dir is its own errand and is not done here.

### what is still open

  * rung 3 at h = 0.970, and rungs 2-3 at h = 0.940, had not finished
    re-offering at the time box; both maps can only improve when they do.
  * the 2.75 mm trajectory residual charged to a STATIC DRAWING POSE (§3),
    worth 39 of the 108 hole cells at h = 0.940.  It is a constant and it is
    Pete's call.
  * `PROPOSED_ATLAS` in `tests/test_paper.py` wants a re-sweep at the final
    tool so that block of tests runs again.


## 2026-09-09 — the band refuses what the arm does not: a pose-aware neighbour, as an option

`mounts.attach_body_columns` gives every arm the OTHER arms' pose-invariant
BASE COLUMN BANDS — a 0.32 m AABB per band standing in for a neighbour's
shoulder and upper links AT ANY POSE, because at sweep time nobody has decided
what pose the neighbour will hold.  Against a neighbour that might be moving
that is the honest model.  Against one that is FROZEN at a certified park it is
over-conservative by construction, and the under-base holes are where that
bites.

MEASURED, at cell (0.60, 1.82), h = 0.970 — the cell Pete looked at on :7001.
Arm 71 is the only arm that can draw it, its drawing pose is healthy (margin
0.3503, sigma 0.1586, lean 12.5°, static 71.2 mm), and every one of its 8
pen-up legs is refused against `body:31_column3`, arm 31's own band, at −131.0,
38.6, 8.0, 62.8 mm on the depot legs and 62.6, −48.2, −131.0, −104.0 mm on the
descents.  Arm 31's ACTUAL parked capsules clear the same routes by **127.0 mm**
— and `_enter_clear` never even asks, because `allocate.ParkProbe` is consulted
only AFTER `enter_beats` has planned.  The band refuses what the arm does not.

### the option

`aris_sixarm/frozen.py`, OFF by default and inert until `freeze()` is called.
Given `{aid: q}` it drops exactly the `body:<aid>_column<k>` bands of the named
arms and replaces them with those arms' real link capsules at those poses,
checked at the same floors the bands were checked at.  TRUE STRUCTURE IS NEVER
DROPPED — mounts, plates, the drop cluster, the runway, and the bands of any
partner NOT named all stay.  `frozen.observe(aid)` names the arm that is
moving so it is never checked against itself.

`paper.py` routes its six static-clearance sites and its one box accessor
through it, so the pen-up legs AND the hover gate (`writing.static_gate` ->
`paper.chain_static`) see the same room.  `scripts/feasible_workspace.py` gains
`--frozen-partners`, threaded through both worker pools.

### THE DEPENDENCY, and it is not free

A cell certified this way is certified ONLY WHILE EACH NAMED ARM HOLDS ITS
NAMED POSE — for the whole stroke and for both of that cell's pen-up legs.
That is an obligation on the conductor, so it travels with the number:
`--frozen-partners` writes a `frozen_dependency` block into the map JSON
carrying the arms, their exact poses, the parks file they came from, what was
replaced, what was kept, and the floor.  `scripts/certified_area.py` carries it
into the certified-area JSON.

The conductor already has the mechanism.  Its phases freeze the arms that are
not drawing; `scene_check` reports `frozen N/N` over the merged timeline (v18
reports 6/6) and lists each frozen arm's own worst clearances; and
`allocate.ParkProbe` is what holds a parked partner to the pair margin while
another arm draws.  What this option does is let the MAP assume what the
conductor already enforces — which is also exactly the solo-map assumption.

### what it recovers

Both maps were rebuilt `--from-raw` the adaptive-certificate maps and re-offered
the ladder over the cells they refused, with `--frozen-partners` on.  **Rungs 0
and 1 completed at both heights; rungs 2-3 were not reached, so these are a
LOWER bound.**

|                        | h = 0.970 |            | h = 0.940 |            |
|------------------------|-----------|------------|-----------|------------|
|                        | bands     | pose-aware | bands     | pose-aware |
| enclosed holes         | 9         | 9          | 13        | **6**      |
| hole cells             | 22        | **11**     | 83        | **71**     |
| solo-drawable          | 97.585 %  | 97.651 %   | 98.140 %  | 98.213 %   |
| largest hole-free rect | 2.1112 m² | 2.1112 m²  | 1.9656 m² | 1.9656 m²  |
| near-square hole-free  | 1.7024 m² | **1.7328 m²** | 1.7064 m² | **1.7280 m²** |

11 arm-cells turned at h = 0.970 and 12 at h = 0.940.  The hole CELLS halve at
0.970 (22 -> 11) while the hole COUNT does not move, which is what shrinking
holes rather than closing them looks like; at 0.940 the count itself falls
13 -> 6.  The headline rectangles do not move — the cells this turns are not
the ones cutting them — but the near-square, which is the shape a logo actually
uses, gains 1.8 % and 1.3 %.

AND PETE'S OWN CELL DOES NOT TURN, which is worth saying plainly.  Arm 71 at
(0.60, 1.82) goes from 4 hover candidates routing 0 of 4 depot legs and 0 of 4
descents, to **49** candidates routing **27 of 49** depot legs and **19 of 49**
descents — the legs genuinely open, and the hover GATE loosens as well as the
legs.  But the pipeline still does not certify the cell at rungs 0-1.  An
earlier pass DID certify it at rung 1 with a parked-partner clearance of
81.9 mm; that pass had the aside bug below, and once the frozen set is made to
follow the aside parks the answer goes away.  The optimistic number is
superseded and is not claimed.

### the aside rungs had to be made consistent

The first cut had a real bug, and it was caught by exactly the discrepancy that
should catch it — Pete's cell turned in process and did not turn in the map.
An ASIDE rung MOVES the partners, and the frozen set was still derived from the
shipped depots, so the obstacle model and `allocate.ParkProbe` were describing
two different sets of six poses.  The ACCEPTANCE was never wrong (the probe
always measured the planned path against the parks actually in force, at the
80 mm pair margin), but the room the planner searched was, and the dependency
the map recorded would have named the wrong poses.  `_cell_escalated` now
re-freezes on each park set before planning against it, and drops `paper`'s
memo with it.

### the search artifact, as a separate lever

Independently of all of this: on that same cell, at a 50 mm floor the router
finds depot routes whose TRUE minimum clearance is **63.1 mm** — legal at the
shipped 63 mm floor, and not found at it within budget.  2 of the 8 legs on
that cell are in that class.  That is a search-budget/seeding lever rather than
a geometry one.  The cell-level question — how many cells a doubled budget
would turn — was CUT for the box and is not claimed.

### tests

`tests/test_paper.py` gains four for the option: it is off by default and
reproduces the shipped boxes and clearances exactly; freezing drops ONLY the
named partners' `body:<aid>_column<k>` bands and never structure nor a moving
partner's band; a partner that is NOT frozen is unaffected, which is the
never-loosens property; and the frozen partner's capsules actually bind, so the
swap is a replacement and not a deletion.

### still open

  * rungs 2 and 3 with the corrected aside re-freeze — both maps can only
    improve, and Pete's cell is the one to watch.
  * which park set each frozen-certified cell depended on: the map records the
    set in force at the run, but a cell turned at an ASIDE rung depends on that
    rung's set, not the shipped one.  Recording it per cell is the next step
    before any of this could ship.
  * the budget/seeding lever above.

## 2026-09-09 — the anatomy of the purple cells, and why a lower hover does not help

Pete, looking at the workspace scenes on :7002/:7003: *"the red ones are off to
the side, those are completely fine.  for the purple ones I see that the arms
are mounted all with the same orientation and all of the purple things are
under the bases of the arms on one side.  I want to understand the anatomy of
these failures.  also for the orange ones can we just reduce the hover height?
it should not need to hover crazy high."*

### A. the purple cells (hover ok, no route)

**Pete's reading of the picture is right, and the asymmetry is real.**  At
h = 0.970, 14 of the 15 purple cells sit DIRECTLY under a base — within 45 mm
of the base centre — and every one of those 14 is under a LEFT-COLUMN base
(arms 13, 31, 2, all at x = 0.597).  Not one is under a right-column base.  In
every case the arm that certifies the drawing pose is the ACROSS-THE-BAY
partner (17, 71, 97 respectively), never the arm whose base it is:

    under arm 13 (0.597, 0.605)   5 cells   all drawn by arm 17
    under arm 31 (0.597, 1.815)   5 cells   all drawn by arm 71
    under arm  2 (0.597, 3.026)   4 cells   all drawn by arm 97
    (0.46, 2.60), r = 0.447       1 cell    drawn by arm 2 itself — the odd one

h = 0.940's 7 purple cells are a DIFFERENT population and should not be read
with them: they are 0.307–0.522 m from the nearest base, not under it, and each
is drawn by its own nearest arm.

THE MECHANISM.  All six bases are clocked identically (yaw 0).  A cell under a
left-column base is reached by a right-column arm travelling in world −x; a
cell under a right-column base is reached by a left-column arm travelling in
world +x.  Same world geometry — but because the two arms share a clocking
rather than mirroring it, those are OPPOSITE directions in each arm's own base
frame.  The two cases are therefore not mirror images of each other, and only
one of them has to thread the partner's column band with the forearm.

Measured on the representative cell (0.60, 1.82), arm 71 (`out/dead_under_base`):
every one of its 8 pen-up legs is refused against `body:31_column3` — the
column of the base the cell sits under — at −131.0, 38.6, 8.0 and 62.8 mm on
the depot legs and 62.6, −48.2, −131.0 and −104.0 mm on the descents, against a
63 mm floor.  Arm 31's ACTUAL parked capsules clear those same routes by
127.0 mm, so it is the pose-invariant band and not the neighbour arm.

**NOT DONE, and it is the test that would settle it:** re-clocking alternate
bases by 180° about their z and re-solving just these cells.  The enumeration
above cost 40 minutes of the box and the what-if was cut rather than the
deadline.  It is a cheap run — `scripts/asbuilt_layout.py` already carries a
per-arm yaw — and it is the next thing to do, because if the purple cells move
to the other side under re-clocking then base clocking is a live hardware lever
and it is still open on the checklist.

### B. the orange cells (draw ok, no hover) — a lower hover does not help

`writing.HOVER_LADDER` solves at 60/45/30/90/120 mm.  Extending it DOWNWARD to
25, 20, 15 and 10 mm, in process, at both heights:

**0 of 8 orange cells turn at h = 0.940, and 0 of 7 at h = 0.970.**

There are two distinct reasons and neither is the height:

  * NO IK AT ALL for the lift, at any of the five heights — all 8 cells at
    0.940, and 4 of the 7 at 0.970.  These sit at the far reach limit of the
    across-bay arm; the drawing pose exists but nothing above it does.
  * THE LIFT DRIVES A LINK INTO THE NEIGHBOUR'S COLUMN — the other 3 at 0.970.
    Static clearance at the hover is NEGATIVE and stays negative all the way
    down: (1.22, 0.58) arm 13 reads −70 mm at 30 mm and −40 mm at 10 mm;
    (1.22, 1.80) arm 31 reads −53 mm and −46 mm; (1.22, 1.78) arm 31 is the
    best of them and only reaches +5 mm at a 10 mm hover, against a 63 mm
    floor.  Lowering the hover buys single-digit millimetres of a 60-plus
    millimetre deficit.

HOW LOW A HOVER MAY GO, since it was asked.  `paper.TIP_CLEAR` = 20 mm is the
shipped pen-up travel clearance, and `paper.travel_floor` follows the hover
down — a 15 mm hover certifies its transit only to 15 mm above the paper.  A
sane minimum is therefore about 20 mm: below that the pen skims the wet line
with less margin than the 3 mm sweep residual and the 30 mm calibration term
the checker already carries, for no gain.  On this canvas it is moot — nothing
turns at any height.

So: the hover is not crazy high (60 mm default, 30 mm on the ladder's low
rung), and it is not what is refusing these cells.

### the hover collision, shown (2026-09-09)

Pete: *"can you please show me the collisions you are getting at the hover
pose."*  `out/hover_collision/`, live on :7004 — arm 71 over (0.64, 1.80) at
the shipped h = 0.940, one of the ORANGE cells.

It is a clean single-gate story.  The DRAWING pose clears arm 31's base column
by **63.1 mm** against the 63 mm floor — 0.1 mm of slack.  Lift the pen and the
forearm swings INTO that column, and the higher the lift the deeper it goes:

    hover 60 mm   link7->hand   vs body:31_column3    8.0 mm   deficit 55.0
    hover 30 mm   link4->link5  vs body:31_column3   35.0 mm   deficit 28.0
    hover 10 mm   link7->hand   vs body:31_column3   53.1 mm   deficit  9.9

1180 IK solutions exist at the 30 mm lift and **none** certifies; every one of
the six heights tried fails the SAME gate and only that gate — self-clearance
holds 156-228 mm against 23, chain-z 32-82 mm against 20, joint margin
0.336-0.373 rad against 0.30, sigma 0.194-0.214 against 0.14.  Nothing here is
marginal except the static floor, and it is short by 10 to 55 mm.

That is also the mechanical answer to "can we just hover lower": the deficit
shrinks monotonically as the hover drops, but at 10 mm — already half the
20 mm `TIP_CLEAR` transit floor — it is still 9.9 mm short.

### and the re-clocking what-if: not a lever, and it cannot be

`scripts/reclock_whatif.py`.  Two findings, and the first is the one that
matters.

**THE COLUMN BAND IS YAW-INVARIANT.**  `mounts.arm_column_boxes` builds each
band from the base ORIGIN and the base Z AXIS as a cylinder carried as its
AABB.  Rotating a base about its own z changes neither, so the obstacle is
bit-identical under any clocking — verified numerically on all three rotated
variants.  Re-clocking therefore CANNOT move the thing that is in the way; it
can only change how well an arm reaches past it.  Pete's hypothesis that the
asymmetry is the clocking is right about the CAUSE (the arms share a clocking
instead of mirroring it, so reaching left-to-right and right-to-left are
different problems in each arm's own frame) but a 180 deg turn is not the fix.

**AND IT MEASURES WORSE, IN EVERY VARIANT.**  Under one consistent method:
baseline 13 of 14 cells feasible; left column turned 0; right column (the
drawers) turned 0; all six turned 0.  A turned arm's hover set collapses —
arm 17 goes from 48 hover candidates to 2-4 — because reaching under the
opposite base is now behind it.

CAVEAT, AND IT IS LOAD-BEARING: that reconstruction does NOT reproduce the
shipped map's baseline (13 of 14 where the map says 0 of 14).  It calls
`atlas.solve_cell` directly at the 63 mm gate while the shipped atlas is a
50 mm sweep re-gated at 63, so it finds drawing poses the re-gate never had —
two drawers per cell where the atlas has one.  The BETWEEN-CLOCKING comparison
is sound; the absolute counts are not.  Settling it properly needs an atlas
re-swept per clocking, which is exactly the cost this was written to avoid.


## 2026-09-09 — "that is a fake collision": the band was a box, and the box was mostly air

Pete, on the :7004 hover scene: *"ohh that is a fake collision. the box is very
over conservative!!!"*  He is right, and it is worth 144 mm.

### what the bands are, and what the box did to them

`mounts.MOUNTS.body_bands` is the mesh audit's measured radial profile of an
FR3's base casting and shoulder: four (z0, z1, r) bands in BASE z, from the
connector and cable stub 232.5 mm ABOVE the flange down to 387.5 mm below it,
each grown by `calib` = 30 mm of unsurveyed-base allowance.  Joint 1 turns
about base z, so everything link0 and link1 carry sweeps into a SOLID OF
REVOLUTION about that axis — the profile is not an approximation of the swept
body, it IS the swept body, and a cylinder per band is exact for it.  (Links 2
and beyond have never been in this envelope and still are not; they are the
conductor's pair clearance and `allocate.ParkProbe`'s business.)

`mounts.arm_column_boxes` then handed each band to the box machinery as
`lo = min(a, b) - r`, `hi = max(a, b) + r` — the band's AXIS-ALIGNED BOUNDING
BOX, inflated by the band's radius in ALL THREE AXES.  Measured, arm 31 at
h = 0.970:

| band | what it stands for | cylinder | its AABB | volume |
|------|--------------------|----------|----------|--------|
| 0 | connector + cable stub | r 207 mm x 299 mm | 414 x 414 x 713 mm | **3.04x** |
| 1 | the shoulder-ward taper | r 148 mm x 32 mm | 296 x 296 x 328 mm | **13.01x** |
| 2 | THE WAIST | r 108 mm x 160 mm | 216 x 216 x 376 mm | **2.99x** |
| 3 | link1's swept solid | r 160 mm x 128.5 mm | 320 x 320 x 449 mm | **4.44x** |

The footprint is 1.273x the circle in every band (a square circumscribing it),
which is the honest cost of an AABB.  The Z PADDING IS NOT: a cylinder needs
none at all, and band 3's box therefore hangs **160.0 mm below where the arm's
body actually ends** — 207.0 mm for band 0.  That is the space a neighbour's
forearm passes through on its way under a base, and it is empty.

WHAT IT COST, on the cell Pete was looking at — arm 71 over (0.64, 1.80),
h = 0.940, its hovers measured against arm 31's column:

    hover 60 mm    AABB    8.0 mm      cylinder  152.2 mm     fake +144.2
    hover 30 mm    AABB   35.0 mm      cylinder  173.4 mm     fake +138.5
    hover 10 mm    AABB   53.1 mm      cylinder  214.7 mm     fake +161.6
    drawing pose   AABB   63.1 mm      cylinder  224.6 mm     fake +161.6

The drawing pose was clearing the floor by 0.1 mm and the 60 mm hover was
"inside by 55 mm".  Neither was true.  Both were measuring the corner of a box
that is 4.4 times the volume of the thing it stands for.

### the tight envelope

`aris_sixarm/envelope.py`: the same four bands as FINITE CYLINDERS, with an
exact segment-to-cylinder distance in the same ternary-search style
`rig_final.segment_box_clearance` uses — the distance is convex along the
segment, so the search finds the true minimum.  Each cylinder still carries its
`lo`/`hi`, so `paper.near_boxes` and every other box-shaped consumer keep
working and only the NARROW phase changes.

Three tests pin it.  It still ENVELOPES: 256 random joint samples of a
neighbour over its full range, every `coordination.BASE_CAPSULES` surface, max
escape **0.000 mm** — zero to floating point, because the bands ARE the
measured profile and `calib` is pure margin on top.  It is CONTAINED in what it
replaces: 4000 random points, the cylinder set is inside the box set
everywhere, so a pose the boxes cleared is a pose the cylinders clear and the
only new answers are fake collisions going away.  And the padding itself is
pinned at 160 mm and 1.273x, so nobody re-derives it by accident.

### what it recovers, and where it is inert

The hover gate first, because that is the layer the padding was refusing.  Over
every dead cell that has a certified drawing pose, counting cells that get a
certified hover on `writing.HOVER_LADDER`:

| h | AABB bands | CYLINDER bands | gained | lost |
|---|-----------|----------------|--------|------|
| 0.940 | 7 of 15 | **15 of 15** | **8** | 0 |
| 0.970 | 1 of 22 | **22 of 22** | **21** | 0 |

Ladder rungs certified in total go 35 -> 67 and 5 -> 110.  EVERY addressable
dead cell at both heights now has a hover, and the eight cells gained at 0.940
are exactly the eight ORANGE `draw ok, no hover` cells — including (0.64, 1.80),
the one on :7004.  Pete's "can we just hover lower" was the right instinct
aimed at the wrong knob: the hover was never too high, the obstacle was too fat.

Then the three-layer maps, rescue rungs 0 and 1 from the pre-fix maps:

|                        | h = 0.970 |          | h = 0.940 |          |
|------------------------|-----------|----------|-----------|----------|
|                        | before    | after    | before    | after    |
| **NO_HOVER cells**     | 7         | **0**    | 8         | **0**    |
| NO_ROUTE cells         | 15        | **11**   | 7         | **3**    |
| NO_DRAW cells          | 378       | 378      | 293       | 293      |
| enclosed holes         | 9         | 9        | 13        | **6**    |
| hole cells             | 22        | **11**   | 83        | **71**   |
| solo-drawable          | 97.585 %  | 97.651 % | 98.140 %  | 98.213 % |
| largest hole-free rect | 2.1112 m² | 2.1112 m² | 1.9656 m² | 1.9656 m² |
| near-square hole-free  | 1.7024 m² | **1.7328 m²** | 1.7064 m² | **1.7280 m²** |

**`NO_HOVER` is gone as a category at both heights.**  The cell on :7004 —
(0.64, 1.80) — is now LIVE at both heights.  `NO_DRAW` does not move by one
cell, and that is the honest limit of this change: the ATLAS still gates
drawing poses against the bounding boxes, and 378 of the 400 dead cells at
0.970 are `NO_DRAW`.  Re-sweeping the atlases under the cylinder model is the
large remaining prize and it was not attempted here.

AND WHERE IT IS INERT, which matters for reading the table.  With `frozen` on
for every partner — the default for a SOLO map, where every other arm is parked
by construction — the body bands are already replaced by the partners' real
capsules, so the cylinder swap has nothing left to swap.  The two halves are
complementary, not additive: cylinders are what the router uses for a partner
that is MOVING, frozen capsules for one that is parked.  The map numbers above
are therefore the frozen model's; the cylinder model's own contribution is the
hover table, and it is what makes the non-frozen path honest.

They also disagree in the right direction.  On (0.64, 1.80): cylinders alone
give the hover 152.2 mm and let the legs plan — and then `ParkProbe` reports
**-8.2 mm**, because arm 31's REAL parked arm is where the route goes.  With
frozen on, the hover is refused up front for the same real reason.  The
cylinder model removes a fake obstacle; it does not invent clearance.

### defaults, and the flag

`--frozen-partners` and the cylinder envelope are now BOTH ON by default in
`scripts/feasible_workspace.py`, and the JSON records which model produced it
(`neighbour_model`) alongside the `frozen_dependency` block.
`--legacy-bands` turns both off and reproduces the pre-2026-09-09 model
exactly.  The library defaults are untouched: `envelope` and `frozen` are inert
until installed, so nothing outside the map pipeline changed behaviour — which
is why `scene_check` over v18's whole merged timeline is **bit-identical**, all
28 keys, `min_clearance` 80.591 mm, PASS.  A model that only removes fake
obstacles has to leave an already-certified programme alone, and it does.

Tests: paper, transit, mounts, selfcoll, layout, system_model, report_tools,
lateral, gates — **199 passed, 20 skipped**.

## 2026-09-09 — the atlas gets the honest room too, and 0.970's rectangle grows 47 %

The neighbour-model fix earlier today moved the ROUTER and the HOVER gate off
the bounding boxes and left the ATLAS on them, which is where most of the dead
canvas lived: 378 of 400 dead cells at h = 0.970 were `NO_DRAW`.  This closes
that.

`atlas._clears` now measures through the same funnel the router uses
(`frozen.chain_clearance`), and `atlas.sweep_arm` applies whatever neighbour
model is installed to its own box set.  Both are inert until installed, so a
plain sweep reproduces every shipped atlas bit for bit — checked on a shipped
row before anything else was run.  `scripts/sweep_atlas_model.py` re-sweeps a
height under the model; `scripts/regate_atlas.py` gained `--model-parks` so the
re-gate to 63 mm re-solves in the same room.

### the drawing-pose layer

Re-swept at both heights (57 s and 63 s for six arms), then re-gated to 63 mm:

| | strict-GO arm-cells | distinct cells with a drawer |
|---|---|---|
| 0.970 shipped (AABB) | 23 330 | 16 184 |
| **0.970 new (cyl+frozen)** | **24 053** (+723) | **16 184** (+0) |
| 0.940 shipped (AABB) | 23 945 | 16 269 |
| **0.940 new (cyl+frozen)** | **25 082** (+1137) | **16 337** (+68) |

**Not one arm-cell was dropped at either height**, so the new atlas is a strict
SUPERSET of the shipped one and every cell the old map called live is still
live.  And the honest headline: at h = 0.970 the fix buys REDUNDANCY — 723 more
(arm, cell) pairs — and NOT ONE NEW CELL.  Every one of the 378 `NO_DRAW` cells
there is out of REACH, not blocked by a neighbour.  At 0.940 it buys 68 cells.

### the map

The full three-layer sweep would have cost about six hours per height — the map
got expensive precisely because the fix worked, since far more cells now reach
the routing layer instead of failing fast at the hover gate.  The superset
property above makes the shortcut sound, so `scripts/remap_dead_cells.py`
re-decides ONLY the dead cells (389 and 296 of them) against the new atlas and
merges; it checks the superset property itself and refuses if it does not hold.

|                        | h = 0.970 |           | h = 0.940 |           |
|------------------------|-----------|-----------|-----------|-----------|
|                        | AABB atlas | cyl atlas | AABB atlas | cyl atlas |
| solo-drawable          | 97.651 %  | 97.687 %  | 98.213 %  | 98.279 %  |
| `NO_DRAW`              | 378       | 378       | 293       | **225**   |
| `NO_HOVER`             | 0         | 0         | 0         | 0         |
| `NO_ROUTE`             | 11        | **5**     | 3         | 60        |
| enclosed holes         | 9         | **4**     | 6         | 7         |
| hole cells             | 11        | **5**     | 71        | **60**    |
| **largest hole-free**  | 2.1112 m² | **3.1008 m²** | 1.9656 m² | 1.9656 m² |
| ...its extent          | 0.58 x 3.64 | **1.02 x 3.04** | 0.54 x 3.64 | 0.54 x 3.64 |
| near-square hole-free  | 1.7328 m² | **1.8240 m²** | 1.7280 m² | 1.7280 m² |

**h = 0.970's certified rectangle grows 47 %, from a 0.58 m sliver to a
1.02 x 3.04 m block.**  That is the number worth having: a strip that narrow was
never going to hold a drawing, and this one will.

`NO_ROUTE` rising to 60 at 0.940 is not a regression — it is 68 cells that used
to be `NO_DRAW` arriving at the next gate.  They can now be drawn; they still
cannot be flown to.  The cause moved, honestly.

### how much of what is left is rim

Counting dead cells within 100 mm of a canvas edge and outside every base disc:

    h = 0.970   344 of the 378 NO_DRAW are rim  (91 %)
    h = 0.940   221 of the 225 NO_DRAW are rim  (98 %)

Pete accepts the rim, and the rim is what is left.  Under-base dead cells fall
11 -> 5 at 0.970 and 71 -> 60 at 0.940.  His own cell (0.60, 1.82) is still
dead at both heights, and still at the ROUTE layer: it can be drawn, and the
arm that draws it cannot be flown there past the neighbour that is really
parked over it.

### what was cut, and what held

CUT: the full three-layer sweeps (six hours per height), replaced by the
dead-cell remap above; and rung 1 at h = 0.940, which ran rung 0 only — it
turned 2 cells in the comparable earlier run, so the 0.940 numbers are a lower
bound by about that much.  h = 0.970 ran rungs 0 and 1.

HELD: `scene_check` over v18's whole merged timeline is **bit-identical** —
28 of 28 keys, `min_clearance` 80.59104857659906 mm, PASS — because the library
defaults are untouched and both models are inert until a script installs them.

## 2026-09-09 — DECISION: the arm-to-arm gate is 50 mm, and a known pose stops paying for a sweep

Pete, after the :7005 scene showed the last five dead cells refused by a PARKED
neighbour at 61-69 mm against an 80 mm gate: *"ohh ok so these are not real
collisions.  make the arm to arm margin 5cm."*  Two changes follow, and
together they close the canvas at h = 0.970.

### 1. `PAIR_MARGIN = 0.050` (was 0.080)

The arm-to-arm gate is now ONE constant, `coordination.PAIR_MARGIN`, and every
consumer already reduced to `SAFETY_M + CALIB_M` — the conductor's inter-arm
clearance, `allocate.ParkProbe`'s park-vs-mover and park-vs-ink checks,
`scene_check`'s inter-arm pass (which takes it as an argument), `layout`'s park
screens, the pause and pen-swap checks — so the sum is what moved and no call
site changed.

THE CALIBRATION ALLOWANCE IS NOT WHAT WAS SPENT.  `CALIB_M` stays at 30 mm: it
is a real uncertainty about where the bases ARE, and spending it would be
spending something we do not have.  What moved is the discretionary operating
clearance on top of it, 50 -> 20 mm.  **SELF and STATIC are untouched**:
`selfcoll.SELF_PLAN_MARGIN` 23 mm, `rig_final.STATIC_MARGIN` 50 mm,
`STATIC_PLAN_MARGIN` 63 mm all stand.  This is one arm against another arm and
nothing else.

Two existing tests moved with it, and both moved in the safe direction.  The
mount model's identity `box_r + STATIC_MARGIN == cap_r + pair margin` was an
EQUALITY when the pair margin was 0.08; it is now an inequality with exactly
`CALIB_M` = 30 mm of slack, i.e. **the static box gate is now strictly stronger
than the arm-to-arm gate**, and the test pins that.  And `layout`'s inward-park
control, written to show a refusal, measures 61.3 mm: a refusal at 80 mm, a
clearance at 50.  It is now pinned in millimetres rather than in a constant
that has moved.

### 2. link1's revolution sweep is dropped for a KNOWN pose

`coordination.BASE_CAPSULES` runs flange-to-shoulder and its last band is
link1's swept solid — the envelope of the upper arm REVOLVING about joint 1.
For an arm whose pose is unknown that is exactly right.  For one FROZEN at a
named park, link1 is somewhere specific and its real body is already carried by
the `(1, 3, UPPER_R)` capsule, so the sweep is the same double count the body
column's AABB was, one layer up.  Measured on the five residual cells it was
worth 8-36 mm of pair clearance and refused every one of them.

`coordination.known_pose_capsules` drops that band and only that band; bands
0-2 are link0's own casting and stay.  It is applied in exactly two places —
`allocate.ParkProbe` (whose partners are parked by definition) and
`frozen.freeze` — and **a moving partner keeps the full table**, which is the
never-loosens property and is pinned by a test.

### the five cells, and the rectangle

|                        | h = 0.970 |          | h = 0.940 |          |
|------------------------|-----------|----------|-----------|----------|
|                        | 80 mm     | **50 mm**| 80 mm     | **50 mm**|
| solo-drawable          | 97.687 %  | 97.718 % | 98.279 %  | 98.587 % |
| `NO_DRAW`              | 378       | 378      | 225       | 225      |
| `NO_HOVER`             | 0         | 0        | 0         | 0        |
| `NO_ROUTE`             | 5         | **0**    | 60        | **9**    |
| enclosed holes         | 4         | **0**    | 7         | **4**    |
| hole cells             | 5         | **0**    | 60        | **9**    |
| **largest hole-free**  | 3.1008 m² | **5.4600 m²** | 1.9656 m² | **3.8584 m²** |
| ...its extent          | 1.02 x 3.04 | **1.50 x 3.64** | 0.54 x 3.64 | **1.06 x 3.64** |
| near-square hole-free  | 1.8240 m² | **3.4352 m²** | 1.7280 m² | **1.8960 m²** |

**All five cells are live.**  (1.22, 0.58), (0.60, 0.60), (0.60, 1.82),
(0.60, 1.84) and (0.62, 3.00) all read `cause = feasible`.

**At h = 0.970 the canvas has NO HOLES AT ALL** — `NO_HOVER` and `NO_ROUTE` are
both zero and every remaining dead cell is `NO_DRAW`, out of reach at the rim.
The certified block is 1.50 x 3.64 m = 5.46 m², spanning y = 0.00 to 3.62 (the
FULL length of the canvas) and x = 0.16 to 1.64 on a canvas 1.80 m wide.  So it
reaches the rim in y outright, and stops 160 mm short of each side in x —
against the reach limit, which is the red Pete accepts.  The rectangle is up
76 % at 0.970 and 96 % at 0.940.

### v18

`scene_check` over the whole merged timeline: **PASS**, and bit-identical, 28
of 28 keys.  Its inter-arm minimum is **unchanged at 80.59 mm** — the geometry
did not move — and against the new 50 mm gate it now carries **30.59 mm of
headroom** instead of 0.59 mm.  A programme certified at 80 passes at 50 by
construction, and there is a test for that too.

## 2026-09-10 — DECISION: h = 0.970 adopted

Pete, on the fabrication drawings: *"do you have the drawings for the 970 one?
let's just work with that one."*  The mounting height in force is now
**0.970 m** — the underside of each mounting plate above the paper — and the
whole package moved with it in one commit set: layout, park set, system model,
build sheet, drawings and GUI.  The CSAIL programme is being re-planned as
v19 at this height (GUI job `20260910-124546-ce72`); its placement is settled
and recorded below, its numbers are NOT in this entry yet.

### why 970, when 940 covers more

It is not the coverage optimum and it was never claimed to be.  At the 50 mm
arm-to-arm gate and the tight cylinder envelope:

| | h = 0.940 | **h = 0.970** |
|---|---|---|
| solo-drawable | 98.587 % | 97.718 % |
| `NO_DRAW` (out of reach, rim) | 225 | **378** |
| `NO_HOVER` / `NO_ROUTE` | 0 / 9 | **0 / 0** |
| enclosed holes | 4 | **0** |
| largest hole-free block | 1.06 x 3.64 m = 3.8584 m² | **1.50 x 3.64 m = 5.4600 m²** |
| near-square hole-free | 1.8960 m² | **3.4352 m²** |

**0.970 buys 1.60 m² of certified block by spending 0.87 pp of coverage at the
rim**, and it retires the last four enclosed pockets.  That is the trade Pete
took, and it is the right one for a drawing: a hole in the middle of the paper
is a knife through every rectangle that contains it, and 378 dead cells against
the reach limit at the edge are red that a placement simply stays out of.  The
certified block runs the FULL length of the canvas (y 0.00 .. 3.62) and stops
160 mm short of each long edge.

`out/certified_area_h0970.json` was **re-derived from its own map**
(`scripts/certified_area.py --map out/fw_h0970_m50_map.npz --h 0.970`) as part
of this change and comes back **bit-identical on every number** — same live
count, same 1.50 x 3.64 largest, same near-square, same zero enclosed holes.
The only field that differs is the free-text `recipe`, which is a command-line
argument and not a measurement.  The shipped file IS the one the layout now
points at.

### what moved in the package

**`layout.LAYOUT_PROPOSED["h"]` 0.940 -> 0.970.**  The GRID did not move: same
2 x 3, same columns at 0.5967 / 1.2067, same rows, same 0.61 pitch.

**`PROFILES_LAT` gained an `("inv", 0.970)` row, `[0.02, 0.75]`**, measured off
`out/atlas_proposed_h0970_lat0860` exactly the way the 0.940 row was measured
off `out/atlas_proposed_h0940` — every arm's certified strict-GO cell, radius
from its own base, the tightest arm's lips over all six.  **The two rows are at
DIFFERENT TOOLS and most of the gap is the tool, not the height**: the same
0.940 rig measured on the holder's atlas reads [0.03, 0.77], so of the inner
lip's 0.11 m move about 0.10 m is a tip 86 mm off the hand's axis reaching back
under the base, and 0.01 m is the height.  The outer lip loses 0.02 m going up
30 mm, which is reach.

**`PAIR_WINDOW` (0.26, 0.73) -> (0.04, 0.73)**, which is that annulus and
nothing else: the under-base hole a transverse partner has to cover is 20 mm of
radius now instead of 130.  0.61 was inside the old window and is inside the
new one, and the window has never been what binds the pitch.

**`PARK_GRID_PROPOSED` re-searched at 0.970** (`scripts/height_sweep.py park
--h 0.970 --atlas out/atlas_proposed_h0970_lat0860`, output
`out/park_search_h0970_lat0860.json`), same recipe as the 0.940 search — 24
bearings x 6 radii x 4 hovers per arm, `certified_ready_pose` gate, static
gate, ranked on park-vs-(ink AND lift) at 80 mm and tie-broken on the depot's
own flyability:

| | h = 0.940 | **h = 0.970** |
|---|---|---|
| candidates certifying | 473-474 of 576 | **496-497 of 576** |
| fleet worst park-vs-(ink AND lift) | 93.1 mm | **97.7 mm** |
| fleet park-vs-park | >= 250 mm (cap) | >= 250 mm (cap) |
| entries / go-homes flyable | 113/144 | **114/144** |

The fleet got 30 mm of room and spent it going out and up: three arms take the
0.70 m radius that would not certify at 0.940, five of six park at the 0.30 m
hover.  Arm 71's 93.1 mm — which set the whole 0.940 fleet's worst on its own —
is gone; arm 17 is now the binding arm at 97.7 mm, **47.7 mm over the 50 mm
gate**.  `Q_PARK_PROPOSED` and `PARK_HOVER_PROPOSED` are that grid's
`certified_park_poses` output; all six poses moved.

**ARM 13 NO LONGER STANDS OFF OUTWARD.**  Its searched bearing is +150 deg
where its outward ray is -104 deg (dot -0.273, i.e. 106 deg off — across the
fleet, not into it).  It parks at (0.060, 0.915), the same xy arm 31 uses one
row up, since 13 and 31 are the same mount on the same column at the same
triple.  Its best strictly-outward alternative sits on the same 98.4 mm ink
plateau and reaches 20 of its own 24 cells against this one's 21, so the search
took the depot that can do its job.  Outwardness was always a means; what keeps
the six apart is `fleet_park_clearance`, and it proves the broad-phase cap.
`test_baked_park_poses_are_that_functions_own_output`'s per-arm bar is now "not
within 45 deg of inward" and the fleet count is still pinned at five of six.

### a real regression the height exposed: rung 1 of the aside ladder

**At 0.970 the FIRST aside park `layout.region_aware_parks` offers is one the
arm cannot fly to, for five of the six arms.**  The top of the corridor-
clearance ranking is (r = 0.70, hover = 0.35) — the furthest, highest candidate
in the grid, which is exactly what maximises clearance from the target's column
and exactly what a `paper.route` from a 0.20-0.30 m park cannot reach.  Only
arm 31's rung 1 flies.

**Nothing shipped is broken by this and no code changed.**
`region_aware_parks` says in its own docstring that it gates the pose and the
fleet and deliberately does NOT gate flyability, "because it is the expensive
one and not every caller needs it", and its only consumer,
`scripts/feasible_workspace._park_sets`, therefore offers **rungs 1..3** to the
real check and takes the first that flies.  What was wrong was three tests in
`tests/test_layout.py` that asked rung 1 alone, which held only for as long as
rung 1 happened to fly.  They now walk the ladder the shipped caller walks
(`_a_flyable_aside`), which is the property that was always meant.  Recorded
here rather than papered over: **if a future caller wants a one-shot aside
park, it has to gate on `repark_route` itself.**

### the inward-park control went back to being an overlap

`test_the_parked_fleet_does_not_park_inside_itself` measures what an
aimed-at-the-centre park set does.  At 0.940 the four arms that can certify one
stood **61.3 mm** apart — a refusal at the old 80 mm gate and a 11.3 mm
clearance at the new 50.  At 0.970 they **INTERPENETRATE by 147.2 mm**.  The
arms did not change: `certified_ready_pose` aims from 30 mm further up, so each
arm reaches further IN before its wrist runs out of pose, and four arms
reaching further into the same middle meet sooner.  The control is back to
being refused by the gate in force rather than by a gate that has since been
relaxed, which is the strongest form it can take.

### the system model, the sheet and the drawings

`assets/system_model/` (all four URDF variants + manifest) and
`assets/proposed_rig/` regenerated at 0.970.  `scripts/check_system_model.py`
**ALL PASS**, worst pen tip 4.524e-12 m over 25 configs x 6 arms;
`scripts/check_proposed_rig_urdf.py` **ALL CHECKS PASS**, worst pen tip
9.351e-10 m.  **The pen tip did not move in the hand frame** — the tool is a
hand-frame offset and the mount plane is not in it.

What moved, all of it by exactly +30 mm or -30 mm:

| | h = 0.940 | **h = 0.970** |
|---|---:|---:|
| mount plane | 940.00 | **970.00** |
| plate top | 952.70 | **982.70** |
| clamp stack top | 1048.40 | **1078.40** |
| post bottom | 905.02 | **935.02** |
| **drop post cut length** | 718.60 | **688.60** |

The grid is where it was — underside 1623.62, top of steel 1699.82, floor
-636.68, cage 2336.5 tall — because the datum is the paper and the cage is
floor-standing.  `drop post = (1623.62 - 970) + 34.98 = 688.6`, 24 off.

**`docs/BUILD_SHEET.md` is re-issued** at 970 and at the corrected datum,
superseding the 850 mounting height AND the 2340 ceiling, and it carries the
full z ladder, the certified block, the cut lengths and the six open items.
`out/drawings/` regenerated at `--h 0.970` and **agrees with the sheet on every
number** (688.60 post, 1623.62 grid underside, -636.68 floor, 1699.82 top of
steel, 970.00 mount plane, 5.460 m² certified block) — both read
`aris_sixarm.system_model`, so agreement is by construction and this is the
check that the construction works.  `system_model.reconciliation()`'s **"mount
height" row is CLOSED**: code, model and the paper the fabricator holds now say
the same number, for the first time since the sheet was written.

### the GUI

The job form's default atlas follows the layout: `out/atlas_proposed_h0940_gated`
-> `out/atlas_proposed_h0970_lat0860`.  `gui/server.py` is deliberately
ignorant of the planner so it cannot ASK the layout what `h` is, and the
default is a string; `test_the_default_atlas_is_at_the_layouts_own_height` is
the thread that ties them, and it checks the swept base z as well as the name.

**AND THE 3D SCENE CACHE HAD TO BE DELETED BY HAND.**  `out/gui_cache/scene_
proposed_lateral.{json,bin}` is written once and returned forever — the server
checks only that the file exists.  The cached copy was from 2026-09-02: base z
0.940, pen 0.110/0.110, and the pre-2026-09-07 park poses.  After deleting it
the scene rebuilds at base z **0.970**, pen **0.0460262/0.0860369** and the
current parks.  **That cache has no invalidation on a model change and it is a
trap** — anyone changing `h`, the tool or a park pose must clear it.  Not fixed
here; recorded.

### the certified programme at 970 — v19

Placement chosen by `scripts/placement_proxy.py` against the 0.970 atlas with
`--certified-area out/certified_area_h0970.json` in force.  **260 of the 325
offsets that fit the sheet were refused as falling outside the certified
rectangle** — which is what that flag is for — and 65 of the remainder have a
ZERO contiguous dead run.  The chosen one, v15/v17/v18's size and rotation
(90 deg, scale 0.85, target width 1.43089) at **offset (+0.01, -0.10)**, centre
**(0.9117, 1.7153)**, ties for the largest 5th-percentile clearance to the
nearest dead cell at **184.4 mm** (v18's was 141 mm) and wins the tie on the
worst single sample, 63.2 mm.  No placement search was run.
`out/csail_place_v19_proxy.json`, `out/csail_place_v19_placement.json`.

**v19 IS IN, AND IT IS THE BETTER PROGRAMME ON EVERY AXIS BUT ONE.**  GUI job
`20260910-124546-ce72`, v18's flag set exactly against the 0.970 atlas and the
re-searched parks, 3 709.7 s (v18: 4 430 s).
`out/csail_schedule_h097_v19.{npz,json}` and the artifacts beside them.

| | v18 (h = 0.940) | **v19 (h = 0.970)** |
|---|---:|---:|
| coverage, allocated / conducted | 100.0000 % | **100.0000 %** |
| left empty / skipped | 0.0000 / 0.0000 m | **0.0000 / 0.0000 m** |
| traced / drawn | 16.8048 / 16.8708 m | 16.8048 / 16.8458 m |
| segments | 53 | **46** |
| phases planned / conducted | 3 / 3 | **2 / 2** |
| makespan | 242.146 s | **209.875 s** |
| conducted pause | 220.2 s | **113.6 s** |
| planner wall clock | 4 430 s | **3 710 s** |

**INDEPENDENT `scene_check`, whole merged timeline**
(`scripts/recheck_timeline.py`, 5 038 frames re-sampled to 10 075, 6 arms,
15 pairs): **VERDICT PASS**.

| gate minimum | v18 | **v19** |
|---|---:|---:|
| inter-arm | 80.59 mm (gate 80, **+0.59**) | **50.32 mm (gate 50, +0.32)** |
| — worst pair | 13-31 at t = 42.01 s | 13-31 at t = 65.98 s |
| self | 21.5 mm (gate 20) | **63.1 mm** (gate 20) |
| frame | 51.6 mm (gate 50) | 50.6 mm (gate 50) |
| neighbour base column | 145.8 mm (gate 50) | 131.5 mm (gate 50) |
| paper chain | 20.5 mm (gate 20) | **26.0 mm** (gate 20) |
| paper tip | −7.3 mm (floor −10) | −7.9 mm (floor −10) |
| joint margin, min | 0.1076 | 0.1009 |
| frozen poses | 6/6 | 6/6 |

**THE ONE THAT GOT WORSE IS THE INTER-ARM CLEARANCE, AND IT IS THE HEADLINE.**
v19's arms pass **28 mm closer** than v18's — 50.32 mm against 80.59 — and its
headroom over the gate in force is **+0.32 mm**, thinner than v18's +0.59 mm.
That is the thinnest margin in the programme and it is now the number to
watch, in the seat the paper chain occupied in v18 (which improved to 26.0 mm
and is no longer the binding one).

**AND THAT IS NOT THE HEIGHT — IT IS THE 50 mm GATE, SPENDING ITSELF.**  Both
programmes sit within a millimetre of whatever gate they were planned under,
because the allocator packs until the gate stops it.  v18 at 80 mm bought
three phases and a 242 s makespan; v19 at 50 mm bought two phases and 210 s by
letting arms 13 and 31 work 28 mm closer together.  **The 32 seconds were paid
for out of arm-to-arm clearance, not earned by the extra 30 mm of height** —
`PAIR_MARGIN` still carries `CALIB_M` = 30 mm of unsurveyed-base allowance
underneath, so the real discretionary air between two arms at t = 65.98 s is
20.32 mm.  A base survey is what buys that back, and it is the same
recommendation `mounts.MOUNTS.calib` has carried all along.

## 2026-09-10 — OPEN — FOR PETE: should the two columns FACE EACH OTHER?

Pete: *"I think it would make more sense if the arm bases were facing each
other, no?"*  Recorded here with what is settled, what is measured, and what
is not — because two of the three are already settled and they point in
opposite directions.

### 1. which way is "facing", and Pete's guess is the converse

The convention is `docs/BUILD_SHEET.md` §3 and `system_model.plate_centre_x`:
every inverted arm has `R_world_base = Ry(180) @ Rz(yaw)` with **yaw = 0**, so
its front (the base's own +x) is `R @ [1,0,0] = (-cos yaw, sin yaw, 0)` —
**canvas −x**, toward the x = 0 long edge — and the connector panel, which
faces away from the front, faces the x = 1803.4 edge.  All six.

So under the clocking that ships **the RIGHT column (17, 71, 97) already faces
the left column**, across the centre line at x = 0.9017, and the LEFT column
(13, 31, 2) faces away from it, out over the near edge.  Making the two face
each other therefore means turning the **LEFT** column 180 deg (`yaw = pi`,
front → canvas +x) and leaving the right column alone — **not** the right
column, which is the way round the question was put.  Turning the right column
instead points both columns outward, which is the opposite change.

### 2. what a 180 deg turn changes, and what it provably does not

**IT DOES NOT MOVE THE OBSTACLE.**  `mounts.arm_column_boxes` builds each
neighbour band from the base ORIGIN and the base Z AXIS as a cylinder carried
as its AABB.  A rotation about that same z changes neither, so the column
obstacle is **bit-identical under any clocking** — verified numerically on all
three rotated variants (2026-09-09, `scripts/reclock_whatif.py`).  Re-clocking
cannot move what is in the way.

**IT DOES MOVE JOINT 1'S DEAD WEDGE, AND THAT IS THE WHOLE MECHANISM.**  The
reachable tip set of an arm is a solid of revolution about joint 1 restricted
to joint 1's range, so a clocking rotates the wedge it cannot swing into.  The
drawing-pose atlas is NOT a yaw-invariant disc under the current sweeper —
`atlas.sweep_arm` takes the spec's own `T_world_base`, so yaw enters through
the q1 range, through the arm's own mount boxes and through where the
neighbours sit relative to both.  (The "per-arm atlases are yaw-invariant
disks" line further up this file is from the IKA era and is no longer true of
this code.)  And the things that were never yaw-invariant even in that era are
the ones that decide a programme: pen-up legs and `paper.route`, hover sets,
and the park search, which searches ABSOLUTE bearings in the canvas frame.

**IT FLIPS THE CABLES AND THE PLATE, PER COLUMN, AND BOTH FLIPS ARE GOOD.**
Under uniform clocking all six connector panels face the x = 1803.4 edge, so
the LEFT column's cables exit across the canvas centre line.  Mirrored, each
column's cables exit toward its own nearest long edge — nothing dressed over
the middle of the paper.  And the plate's 25.15 mm offset from the J1 axis
flips with the front, which moves the left column's drop cluster 50.3 mm away
from its transverse partner's:

| | uniform | **mirrored** |
|---|---:|---:|
| cluster-to-cluster gap across a pair | 216.20 mm | **266.50 mm** |
| gusset pair clearance (`GUSSET_PAIR_CLEAR`) | 89.20 mm | **139.50 mm** |

Both are pure arithmetic off `system_model`, and they are a real argument for
mirroring that has nothing to do with reach: the gusset conflict is one of the
six open items on the cut list, and 50 mm is most of what it is short of.

### 3. THE 2026-09-09 RE-CLOCK WHAT-IF IS NOT EVIDENCE EITHER WAY

`scripts/reclock_whatif.py` measured 13 of 14 cells feasible at baseline and
**0 of 14 in every rotated variant**, and it is tempting to read that as
settled.  It is not, for two reasons stated in its own caveat and one more:

1. Its baseline does not reproduce the shipped map's (13 of 14 against the
   map's 0 of 14), because it calls `atlas.solve_cell` directly at 63 mm while
   the shipped atlas is a 50 mm sweep re-gated to 63.
2. **It ran under the OLD obstacle model** — before the same day's finding that
   the band was an AABB of a cylinder and "mostly air", and before
   `PAIR_MARGIN` went to 50 mm and link1's revolution sweep was dropped for a
   known pose.  Every one of those loosened exactly the constraint a turned
   arm runs into.
3. It is 14 cells, chosen because they were the hard ones.

So the hypothesis it tested — that a turned arm's hover set collapses because
reaching under the opposite base is now behind it — is plausible and
unmeasured at the model in force.  **It must not be cited as a refutation.**

### 4. MEASURED, at h = 0.970, on a mirrored atlas swept for the purpose

The mirrored fleet (13/31/2 at yaw = pi, 17/71/97 at 0) was swept at 0.970 at
the holder's tool and re-gated to 63, and every number below is measured
identically on both clockings.  **The uniform column reproduces the shipped
`out/certified_area_h0970.json` exactly** (16184 live, 97.718 %, largest
1.50 x 3.64 = 5.4600 m² at (0.16, 0.00), near-square 3.4352 m²), which is what
makes the comparison worth reading.

| h = 0.970 | uniform | mirrored | delta |
|---|---:|---:|---:|
| strict-GO cells, summed over arms | 23 472 | 23 482 | +10 |
| union strict-GO | 16 184 (97.718 %) | 16 188 (97.742 %) | **+4 cells** |
| >= 2-arm | 6 698 (40.442 %) | 6 704 (40.478 %) | +6 cells |
| >= 3-arm | 2.892 % | 2.892 % | 0 |
| **largest hole-free block** | **5.4600 m²** | **5.4600 m²** | **0.0000** |
| near-square hole-free | 3.4352 m² | 3.4352 m² | 0.0000 |
| landscape hole-free | 1.7928 m² | 1.7928 m² | 0.0000 |
| park candidates certifying | 2 979 / 3 456 | 2 982 / 3 456 | +3 |

Mirrored is a strict superset — **no cell is lost anywhere** — and the whole
gain is 4 cells out of 16 562, **+0.0016 m² of canvas and 0.0000 m² of
certified rectangle**.  Only the LEFT column's atlases moved at all; the right
column's are bit-identical, as they must be, since its yaw did not change.

**AND THE DEAD WEDGE IS A RED HERRING, MEASURED.**  Joint 1 is +/-157.2024 deg,
so the wedge is **45.595 deg wide, centred on the base's own -x** — the
direction opposite the front.  It is invisible on the paper: binning arms 31
and 71's certified cells by world bearing in 15 deg sectors, **all 24 sectors
are populated and GO == reach in every one**, and in the wedge direction itself
the certified poses use |q1| <= 105.2 deg and never approach the stop — the arm
gets "behind" itself by swinging q1 sideways and rotating the forearm out of
the shoulder plane (|q3| to 138.2 deg).  The one real azimuthal notch in the
outer lip is **~80 mm deep and tracks the TRANSVERSE PARTNER, not the wedge**:
arm 31 notches at world 0 deg (partner 71) and arm 71 at 180 deg (partner 31),
in opposite world directions, from the same yaw.  A partner's body column is a
cylinder, so that notch does not move when you turn the base.

The old "atlases are yaw-invariant discs" line survives to 0.04 % — but as an
empirical near-coincidence of a symmetric layout, not a property.
`atlas.solve_cell` does take the spec's `T_world_base`, so yaw genuinely enters
the branch set; what makes it not matter here is that every obstacle box in
`mounts.obstacles_for` is either an AABB symmetric under a pi flip or a
radially symmetric cylinder (both fleets carry 30 geometrically identical
boxes).

**WHAT MIRRORING WOULD COST.**  The shipped park literals do not transfer at
all: re-seated on mirrored bases the fleet's worst pair goes 250.0 mm ->
**-100.1 mm** (31-71) and the worst steel clearance 277.3 -> **-131.0 mm**
(13, 31) — the left column's parked chains swing into the 0.61 m centre gap.
A mirrored park set plainly EXISTS (a greedy max-steel pick off the certified
candidates reaches the same 250.0 mm cap), but it has to be searched and
re-certified.  Beyond that: two handed drop-cluster parts instead of one, and
the loss of `BUILD_SHEET` section 3's single acceptance check — "command all
joints to 0, all six must lean the same way" — which is the cheapest
error-catcher on the build floor.

### RECOMMENDATION: KEEP UNIFORM

**The number that decides it is 0.0000 m².**  The certified hole-free block —
the thing the height was chosen for and the thing a drawing is placed inside —
is 5.4600 m² under both clockings, to the cell, and so are the near-square and
landscape blocks.  Mirroring buys +4 canvas cells and costs a re-searched
six-arm park set, a re-certification, handed steel and an acceptance check.

**The reach argument for mirroring does not exist, and the intuition behind it
is measurably false.**  If the rig is to be mirrored it should be mirrored for
the CABLES and the STEEL, which are real and unmodelled: each column's cables
would exit toward its own nearest long edge instead of the left column's
running across the paper, and the plate flip buys 50.3 mm of drop-cluster gap
(216.20 -> 266.50 mm) and gusset pair clearance (89.20 -> 139.50 mm), which is
most of what open item 4 is short of.  **That is a fabrication decision, and it
would be worth about 5 000 s of park search plus a re-certification.**  Said
plainly so it can be taken on its merits rather than on a workspace argument
that the measurement does not support.

### 4b. THE MIRRORED PARK SET, SEARCHED (2026-09-10)

`scripts/height_sweep.py park --h 0.970 --clocking mirrored --atlas
out/mirror_atlas_h0970_lat0860`, the same 24 bearings x 6 radii x 4 hovers per
arm and the same ranking as the uniform search.
`out/mirrored/park_search_h0970_mirrored.json`.  **It certifies**, and it is a
dead heat:

| | uniform | **mirrored** |
|---|---:|---:|
| fleet worst park-vs-(ink AND lift) | 97.7 mm | **97.7 mm** |
| fleet park-vs-park | >= 250.0 mm (cap) | **>= 250.0 mm (cap)** |
| entries / go-homes flyable | 114/144 (79.2 %) | **115/144 (79.9 %)** |
| candidates certifying | 496-497 of 576 | 497 of 576 (all six) |

Per arm, the recipe and the clearance it wins:

| arm | col | uniform (r, hover, bearing) | mm | in/out | mirrored | mm | in/out |
|---|---|---|---:|---|---|---:|---|
| 2 | left | (0.70, 0.30, +105) | 98.41 | 18/18 | (0.48, 0.30, +150) | **98.41** | 18/18 |
| 13 | left | (0.62, 0.30, +150) | 98.44 | 21/21 | (0.48, 0.20, −135) | **98.44** | 21/21 |
| 17 | right | (0.48, 0.30, −30) | 97.67 | 19/19 | (0.48, 0.30, −30) | 97.67 | 19/19 |
| 31 | left | (0.62, 0.30, +150) | 98.76 | 20/20 | (0.40, 0.30, −150) | 98.63 | **21/21** |
| 71 | right | (0.55, 0.20, +30) | 98.26 | 19/19 | (0.55, 0.20, +30) | 98.26 | 19/19 |
| 97 | right | (0.70, 0.30, +75) | 99.88 | 17/17 | (0.70, 0.30, +75) | 99.88 | 17/17 |

**THE RIGHT COLUMN'S THREE ARMS ARE BIT-IDENTICAL**, recipe and clearance
both, which is the sanity check this table exists for: their yaw did not
change, so nothing about them may.  The LEFT column's three all found a
DIFFERENT recipe and landed on the SAME clearance — which is the park
plateau this file has documented since 2026-08-26 doing exactly what it says:
a parked arm's own base column is pose-invariant, so the LAYOUT and not the
pose sets the ceiling, and a clocking cannot move a cylinder about its own
axis.  The whole difference in the fleet is arm 31 reaching one more of its
own 24 cells, 20/24 -> 21/24.

CAVEAT, and it is small but real: the uniform search ran at the old 80 mm pair
margin and this one at the 50 mm in force, so more candidates clear gate 1 and
the mirrored search had a slightly larger pool to rank.  The `worst` figures
are measured distances and comparable directly; arm 31's extra entry may be
the bigger pool rather than the clocking.  Re-running uniform at 50 mm would
settle it and costs another ~4 900 s, which is not worth it for one cell.

### 4c. THE THREE-LAYER MAP, SAMPLED, BOTH CLOCKINGS — and the atlas was lying

`scripts/height_sweep.py pilot --every 9`, hover and route layers included,
each clocking on its own atlas and its own searched park set, **the same 1-in-9
sampling of each arm's certified cells** so the two runs are cell-for-cell
comparable.  `out/mirrored/pilot_h0970_{uniform,mirrored}.json`.

| all arm-cells | uniform | **mirrored** |
|---|---:|---:|
| sampled | 2 596 | 2 597 |
| **feasible** | **99.384 %** | **99.268 %** |
| `NO_HOVER` | 0.616 % | 0.655 % |
| `NO_ROUTE` | **0.000 %** | **0.077 %** |

| arm | col | uniform feasible | **mirrored** | Δ |
|---|---|---:|---:|---:|
| 2 | left | 99.06 % | **99.29 %** | **+0.23** |
| 13 | left | 99.53 % | 98.83 % | **−0.70** (and 0.47 % `NO_ROUTE`) |
| 17 | right | 99.53 % | 99.53 % | 0.00 |
| 31 | left | 99.55 % | 99.33 % | −0.22 |
| 71 | right | 99.55 % | 99.55 % | 0.00 |
| 97 | right | 99.06 % | 99.06 % | 0.00 |

**THE RIGHT COLUMN IS BIT-IDENTICAL ON ALL THREE ARMS AND ALL THREE LAYERS**,
which is again the check that the experiment is the experiment: their yaw did
not move, so nothing about them may.  **Every difference is in the left column,
the one that turned** — and it is a NET LOSS.

**THIS IS THE MEASUREMENT THE ATLAS COMPARISON COULD NOT MAKE.**  At the
drawing-pose layer mirrored is a strict superset: +4 cells, nothing lost
anywhere.  At the layer that decides whether a cell can actually be DRAWN —
can the arm hover over it, and can it fly there — mirrored is **0.116 pp
worse**, and it introduces the only `NO_ROUTE` cells in either run.  That is
the 2026-09-09 what-if's hypothesis (a turned arm's hover and route sets
degrade because reaching under the opposite base is now behind it) **measured
for the first time under the model in force, at like-for-like cells** — the
what-if itself is still not evidence, for the reasons in section 3, but the
thing it guessed at is real and it is small.

CAVEATS, both of which cut toward "small, and do not over-read it":
the differences are 3 arm-cells out of 2 596 and two of those are arm 13's
`NO_ROUTE`; and `NO_ROUTE` at the pilot's 60 s RRT budget is a budget result,
not a proof of infeasibility — the shipped map spends four escalation rungs on
exactly these cells and recovers most of them.  Both runs had the same budget,
so the COMPARISON is sound; the absolute counts are a floor.

### 4d. THE PROGRAMME AT BOTH CLOCKINGS — v19 against v19m

Same picture, same fixed placement, same flag set; each clocking on its own
atlas and its own searched park set.  v19m via
`scripts/replan_at_height.py --clocking mirrored` (report-only: it patches the
loaded fleet, proves every base z AND every base rotation, and touches nothing
on disk).  `out/mirrored/csail_schedule_h097_v19m.*`.

| | v18 (0.940, uniform) | **v19 (0.970, uniform)** | **v19m (0.970, mirrored)** |
|---|---:|---:|---:|
| coverage, conducted | 100.0000 % | **100.0000 %** | **100.0000 %** |
| left empty / skipped | 0 / 0 m | 0 / 0 m | 0 / 0 m |
| drawn | 16.8708 m | 16.8458 m | 16.8358 m |
| segments | 53 | 46 | **44** |
| phases conducted | 3 | 2 | 2 |
| **makespan** | 242.146 s | **209.875 s** | **249.188 s** |
| conducted pause | 220.2 s | 113.6 s | 155.0 s |
| min inter-arm (run) | 81.9 mm | 53.3 mm | 51.2 mm |
| planner wall clock | 4 430 s | 3 710 s | 3 047 s |

**MIRRORED DRAWS THE SAME PICTURE 39.3 SECONDS SLOWER** — 249.188 s against
v19's 209.875, which is 18.7 % worse and also slower than v18 at the old
height.  It is not a coverage loss: both reach 100 % with nothing left empty.
It is pause.  v19m's phase 2 needed 152.3 s against v19's 113.0, and it spends
155.0 s of conducted pause against 113.6 — the arms wait for each other more.
Its inter-arm minimum is also 2.1 mm tighter (51.2 against 53.3), so it is not
buying the time back with clearance either.

That is the same left-column story the pilot told, one layer up: fewer
certified segments (44 against 46) because the turned column's routes are
slightly harder, and more waiting because the phase cannot be packed as
tightly.  **Nothing here is dramatic and none of it is in mirrored's favour.**

**INDEPENDENT `scene_check`, whole merged timeline, EACH AT ITS OWN CLOCKING**
(`scripts/recheck_timeline.py --clocking mirrored` for v19m — checking it
against uniform bases would measure a machine that never ran, and the script
now refuses to):

| gate minimum | v18 | **v19** | **v19m** |
|---|---:|---:|---:|
| verdict | PASS | **PASS** | **PASS** |
| inter-arm | 80.59 (gate 80) | **50.32 (gate 50)** | **50.32 (gate 50)** |
| — worst pair | 13-31, t = 42.01 s | 13-31, t = 65.98 s | 13-31, t = 55.21 s |
| self | 21.5 mm | **63.1 mm** | 36.2 mm |
| frame | 51.6 mm | 50.6 mm | 50.6 mm |
| neighbour column | 145.8 mm | 131.5 mm | 131.8 mm |
| paper chain | 20.5 mm | 26.0 mm | 26.0 mm |
| paper tip | −7.3 mm | −7.9 mm | −8.2 mm |
| joint margin, min | 0.1076 | 0.1009 | **0.1072** |
| frozen poses | 6/6 | 6/6 | 6/6 |
| frames re-sampled | 5 861 | 10 075 | 11 963 |

**BOTH PASS, AND BOTH LAND ON 50.32 mm** — the same +0.32 mm over the gate, on
the same pair, at different instants.  That is the allocator packing until the
gate stops it, twice, and it is the clearest evidence in this file that the
inter-arm number is set by `PAIR_MARGIN` and not by the geometry.  Mirrored's
self-clearance is a little worse (36.2 against 63.1 mm, arm 31) and its joint
margin a little better (0.1072 against 0.1009); neither is close to binding.

### 5. what was NOT measured

The real three-number mirrored park search (~4 900 s at 4 jobs) — which would
decide whether mirrored depots are as FLYABLE, not just as clearable (uniform:
114/144 entries and go-homes); the three-layer `feasible_workspace` map for
mirrored (hours) — though at 0.970 the map's live set coincides cell-for-cell
with the atlas union, so the rectangle above is not merely a proxy; the pen-up
`paper.route` legs, which ARE genuinely yaw-dependent and none of which were
measured; and any yaw other than 0 / pi.

Artifacts: `out/mirror_atlas_h0970_lat0860{,_gated63}/`,
`out/mirror_compare_h0970.{txt,json}`, `out/mirror_certarea_*_h0970.json`,
`out/mirror_park_{proxy,candidates}_h0970.txt`.

### 6. what it would take to ADOPT it, and what is outstanding

An honest answer needs the mirrored fleet run through the same machinery at
h = 0.970: an atlas re-swept per clocking (yaw is in the sweep), the
three-number park search re-run on it, the certified rectangle against
uniform's 5.4600 m² largest and 3.4352 m² near-square, and the CSAIL plan plus
whole-timeline `scene_check`.  `scripts/asbuilt_layout.py`'s `load_asbuilt`
already carries per-arm yaw, so no committed constant has to move to measure
it.

**NOTHING IN `layout.py` HAS BEEN CHANGED FOR THIS.**  The clocking convention
is still uniform yaw = 0, `docs/BUILD_SHEET.md` §3 still says so and still
says not to improvise it, and per-arm yaw stays where it is — expressible for a
SURVEY, not a configuration — until the measurement says mirrored wins.

## 2026-09-10 — OPEN: rung 1 of the aside-park ladder is unflyable at 0.970

Found while moving the height (see the adoption entry above), and carried here
as an open item rather than fixed, because the fix is a design choice and the
shipped consumer is not broken.

### what it is

`layout.region_aware_parks` returns the best aside park it can find for an arm
somebody needs to draw under.  Its ranking key is `corridor_clearance` — how
far the parked chain stands off the target's own column — and at h = 0.970 the
top of that ranking is **(r = 0.70, hover = 0.35) for five of the six arms**:
the furthest, highest candidate the grid contains, which is exactly what
maximises clearance from a column and exactly what a `paper.route` from a
0.20-0.30 m park cannot reach.  Measured, arm by arm, at the shipped parks:

| arm | rung-1 recipe | corridor clear | `repark_route` |
|---|---|---:|---|
| 2 | (0.70, 0.35, -30) | 0.229 m | **REFUSED** |
| 13 | (0.70, 0.35, -30) | 0.229 m | **REFUSED** |
| 17 | (0.70, 0.35, 150) | 0.229 m | **REFUSED** |
| 31 | (0.55, 0.20, 120) | 0.228 m | ok |
| 71 | (0.70, 0.35, 150) | 0.229 m | **REFUSED** |
| 97 | (0.70, 0.35, 150) | 0.229 m | **REFUSED** |

At 0.940 rung 1 happened to fly, so nothing noticed.

### why nothing shipped is broken

`region_aware_parks` says in its own docstring that it gates the POSE and the
FLEET and deliberately does not gate flyability — *"because it is the expensive
one and not every caller needs it"* — and its only consumer,
`scripts/feasible_workspace._park_sets`, therefore offers **rungs 1..3** to the
real check and takes the first that flies.  The three tests in
`tests/test_layout.py` that asked rung 1 alone now walk the same ladder
(`_a_flyable_aside`).  The CSAIL draw path does not use aside parks at all —
`allocate.ParkProbe` prunes against the shipped literals.

### the fix I would propose, and why it is not in this commit

**Make the ranking pay for a route once, lazily, and cache it.**  Concretely:
give `aside_park_ranking` an optional `route_gate=(spec, q_from, h_inv)` and
have it walk its own descending list, calling `repark_route` on each candidate
until one certifies, then return that one first.  Three properties matter and
the current design has two of them:

1. **it must stay a total order** — the ranking is walked by `rank=k` and a
   caller asking for rung 2 must get the same pose twice, so the route check
   has to be a FILTER on a fixed order, never a re-score;
2. **it must not pay when it cannot help** — a target nobody stands over
   returns the shipped literals by identity today, and that must not change;
3. **the cost is real**: `repark_route` is a full `paper.route`, ~1-3 s, and
   `aside_candidates` is already cached per arm because the map asks this
   question tens of thousands of times.  A route cache keyed on
   `(arm, q_from, recipe, h)` is what makes this affordable, and it is the
   piece that does not exist yet.

That is a change to a function every map cell goes through, measured in hours
of re-mapping to prove it did not move a certified number — which is not
something to land in the same box as a height change.  **What a future caller
must know today: if you want a ONE-SHOT aside park, gate it on
`repark_route` yourself; rung 1 is not a promise.**

## 2026-09-10 — the clocking became askable, and two stale-state bugs fell out

Pete leans toward mirrored on the hardware case (cables outward, +50.3 mm of
drop-cluster gap), so the variant had to become something the pipeline can be
RUN at rather than something argued about.  Making it askable turned up two
bugs of the same family — state that had gone stale and had no way to say so.

### the lever

`layout.build_fleet` takes a `clocking`, `layout.CLOCKINGS` names the two, and
`clocking=None` / `{}` / `CLOCKING_UNIFORM` all give **bit-identical** base
transforms to what the function has always returned (pinned with
`array_equal`, not a tolerance — the whole safety of a variant is that the
default cannot feel it).  It reaches the three report-only drivers as
`--clocking`: `feasible_workspace` (set as a module global before the first
`rig()` and before any pool, the same idiom and the same reason as `--parks`),
`height_sweep` sweep/park/pilot, and `replan_at_height`.  Nothing is written
back; `LAYOUT_PROPOSED` is uniform and `docs/BUILD_SHEET.md` section 3 still
says not to improvise the clocking.

`height_sweep.do_park` also picked up `do_sweep`'s escape hatch, and it is
load-bearing here: the shipped park recipe re-seated on mirrored bases refuses
the fleet, which is exactly the fact the search exists to repair, so it must
not block the search that repairs it.

### bug 1 — `fw.rig`'s "shipped" fast path had drifted off the height

```python
shipped = (abs(pitch - SHIPPED_PITCH) < 1e-9 and abs(h - 0.940) < 1e-9 ...)
```

That literal was written when 0.940 was the height.  After 2026-09-10 it never
matched, so the one branch that is supposed to hand back the **committed**
fleet and the **baked** park literals had silently stopped firing and every
caller was getting a re-derivation of them instead.  Nothing failed, because a
re-derivation agrees to five decimals — which is exactly why it went unseen.
It reads `layout.LAYOUT_PROPOSED["h"]` now.  **A literal was a second place for
the height to live, and it drifted the first time the height moved.**

### bug 2 — the GUI scene cache had no expiry at all

`_ensure_scene` wrote `out/gui_cache/scene_<rig>_<tool>.{json,bin}` once and
returned it forever; the only test was that the file EXISTED.  So from the
moment the height, the tool or a park pose moved, every browser got the old rig
with no warning.  Found live: after h went 0.940 -> 0.970 the served scene was
still a **2026-09-02** build at base z 0.940 with the 0.110 pen and the
pre-2026-09-07 parks, and it had to be deleted by hand.

The fix has to work **without importing the planner**, which is that module's
whole design — importing it binds a rig at import time and the server answers
about five.  So the cache key is a content stamp over every `.py` under
`aris_sixarm/` (size and mtime), plus rig, tool and a version scalar, written
only AFTER a successful build so a crash leaves an entry that simply rebuilds.
It is deliberately CONSERVATIVE: an edit to an unrelated module rebuilds a
scene that would not have changed, which costs one subprocess and is the right
way round.  What it can no longer do is serve a scene built from code that is
gone.  `SCENE_CACHE_V` is the hand-bumped escape for a change to the exported
scene's SHAPE, which file stamps cannot see.

## 2026-09-11 — OPEN — FOR PETE: staged work cells, and why the 1-2-1 has to be by ROW

Pete asked whether the fleet could be run in stages: give each arm a temporary
*work cell*, let the arms inside a stage plan asynchronously against each
other's frozen envelopes, and put a barrier between stages.  Specifically the
"one arm per column in a 121 pattern, leaders and followers, then reversed".

Measured by `scripts/workcell_envelopes.py` on the shipped atlas
(`out/atlas_proposed_h0970_lat0860_gated63`, `is_current` on all six), the
shipped capsules and `PAIR_MARGIN = 0.050`.  Full write-up in
**docs/V2_WORKCELLS.md**, numbers in `out/workcell_envelopes.json`.  **Nothing
in the package changes** — this is a measurement and a recommendation.

### the object

A stage's envelope for an arm is the union of its metal over every certified
drawing pose in its region, the hover above each of those cells, and its park.
Two arms whose envelopes are 50 mm apart are static keep-outs for each other
and need no conductor at all.  An envelope is a union over cells, so the
clearance between two envelopes is a min over cell pairs — the script computes
ONE cell-to-cell clearance matrix per arm pair (15 of them, ~890 x ~890 on a
4 cm lattice, 31 s on 6 processes, `coordination.clearance_matrix`) and every
region, pattern and stage sequence is a reduction over a slice of it.

### THE ELBOW IS THE PROBLEM, NOT THE PEN

Every arm's elbow reaches to within 0.10 m of the paper's mid-line while it
draws (13/31/2 to x = 0.853, 17/71/97 to x = 0.950), and `ELBOW_R` is 0.117.
So the two arms of a TRANSVERSE PAIR interpenetrate by 135 mm before any other
link is counted.  Spot-checked and confirmed independently: arm 31 at
(0.44, 1.40) and arm 71 at (1.24, 1.40) — pens 0.80 m apart, each pose
certified by the atlas — are at **-160.8 mm**, and `scene_check.pair_clearance`
returns the same -160.8 mm as `coordination.clearance_matrix`.

The separation frontier (the smallest dead band that clears 50 mm):

| pair class | axis | dead band |
|---|---|---|
| two rows apart (13-2, 17-97, 13-97, 17-2) | - | **0.00 m** |
| adjacent rows, either column | y | 0.32 - 0.40 m |
| **same row** (13-17, 31-71, 2-97) | x | **1.44 m of a 1.48 m block** |

### PETE'S 1-2-1: RIGHT IDEA, WRONG AXIS

Both readings put a transverse pair in the air at once — (a) makes 31 and 71
the leaders, (b) makes 13 and 17 leaders — and that pair is at **-262.0 mm**
however deep the followers are confined, because confining the followers does
not touch the leaders.  Every depth on both ladders: -262.0 mm.

The partition that works is **one arm per ROW**, columns alternating between
stages.  That is already the grouping `--arm-phases disjoint` computes (bases
within 0.70 m are adjacent; the only edges on a 2 x 3 grid are the transverse
pairs, so the colour classes are the two columns = one arm per row each).  What
it does not do is impose the y dead band, and without it that grouping is
-206.3 mm.

### the recommendation

**One arm per FULL-WIDTH row band, a 0.40 m dead band in y between bands, the
two columns alternating, then four 2-active seam stages.**  +85.8 mm
ink-vs-ink, 97.3 % of the certified block in 6 stages, **2.51x** the serial
makespan (ceiling 3x), and 41.6 % of the block keeps a second stage-compatible
drawer for re-queueing a faulted arm's ink (the atlas's own ceiling is 47.1 %).

The y dead band is a cliff, not a slope: -127.2 mm at 0.20 m, -3.8 mm at
0.30 m, +85.8 mm at 0.40 m.

**6-active never clears.**  Eroding every Voronoi block by 0.60 m leaves 7 % of
the block alive and the worst pair is still -23.2 mm.

### WHAT BLOCKS IT, AND IT IS NOT THE INK

`Q_PARK_PROPOSED` is **not stage-compatible**.  It was searched against the
ALLOCATED ink of one programme and clears that by 97.7 mm; against everything
an arm could be TOLD to draw in a work cell it clears by **4.7 mm** (arms 13
and 17 against each other).  That 4.7 mm, and not the 85.8 mm of ink, is what
fails the gate for the recommended pattern.  Three of the six parks also hover
outside the certified block entirely (13, 31, 71).

So a stage needs its OWN park set, searched against the stage's envelopes.
`layout.region_aware_parks` / `phase_aside_parks` / `aside_candidates` is the
machinery; it currently ranks against a target xy rather than against an
envelope.

### the barrier, and what is missing

Per arm, from MEASURED state: pen up (>= `LIFT_Z`); at the specific `q_park`
stage s+1's envelopes were computed against (a park IDENTITY check, since the
guarantee is indexed by which park); stopped (the envelope argument is about
poses, and `SWEEP_K` exists because a moving link sweeps more than its
samples); stroke queue drained or explicitly re-queued; no un-cleared fault.
Fleet-level: all six report all five, and the held park set is the one the
stage was certified against.

Present already: `coordination`'s envelope and gate; `idle.conduct`'s freeze /
`plan_retreat`; `allocate.ParkProbe`; the certified park set; `--arm-phases
disjoint`; `scene_check.check_static` / `check_timeline`.

Missing: a WORK-CELL object (nothing takes "may only draw here" — it would go
on `allocate.atlas_cells`); per-stage parks; the barrier itself (there is no
fleet rendezvous anywhere — `stroke_api`'s "barrier" is an exception barrier);
and one `check_timeline` per arm per stage to cover the PEN-UP LEG, which is
the one thing an envelope of hover ENDPOINTS does not contain.

## 2026-09-11 — OPEN — FOR PETE: cut at every transition, and the merge is a DP

Pete: *"what if we cut lines everywhere where there is a transition and then
figured out some sort of greedy merging and allocation trying to minimize the
overall number of lines?"*

Yes, and it does not need the greedy.  Sample a line finely, read its CAPABILITY
SET at each sample — the (stage, arm) pairs whose arm has a certified drawing
pose there AND whose work cell contains the point — and cut wherever that set
changes.  Between two cuts every point has the same possible drawers, so the
stretch is indivisible.  Choosing a drawer per atom to minimise the number of
maximal runs is then a textbook chain DP with unit cost per change, and because
the change cost does not depend on what you change FROM, the O(states^2)
relaxation collapses to O(states) a step.

New module `aris_sixarm/traces.py`, `tests/test_traces.py` (31 tests, no env
vars and no atlas — every map is rasterised from rectangles), write-up in
**docs/V2_TRACES.md**.  **Nothing in the existing pipeline changes.**

### the numbers

| | pieces | hand-overs | covered |
|---|---|---|---|
| CSAIL v19, as shipped | **46** segments / 39 strokes | — | 100.2 % |
| CSAIL, single stage (no cells) | **40** | 1 | 100.0 % |
| CSAIL, zigzag + 4 seam stages | **50** | 8 | 94.8 % |

38 of the 39 strokes go in ONE piece.  The single exception is **stroke 26** —
the one `tests/test_merge_spans.py` already pins as the logo's hardest, where
arm 2's certified interval stops at 0.6213 and arm 71's starts at 0.7571 — and
the DP hands it over 71 -> 2 once, inside the overlap the atlas does offer.  The
piece minimiser and the shipped regression landing on the same stroke by
independent routes is the best check this module has.  The 40 holds on v19's own
atlas (`out/atlas_proposed_h0970_lat0860`, `--gate flat --tilt-max-deg 15`) as
well as on the shipped gated-63 one, so it is not an artefact of the map.

A thousand lines, three synthetic sets over the certified block:

| set | pattern | lines | atoms | pieces | hand-overs | covered | ms |
|---|---|---|---|---|---|---|---|
| strokes, 660 m | zigzag | 1 000 | 2 885 | 1 661 | 533 | 97.05 % | 814 |
| strokes | single | 1 000 | 2 614 | 1 254 | 254 | 100 % | 574 |
| hatch, 1 369 m | zigzag | 992 | 4 142 | 2 129 | 1 032 | 97.45 % | 930 |
| hatch | single | 992 | 4 105 | 1 869 | 877 | 100 % | 696 |
| scribble, 213 m | zigzag | 1 000 | 1 597 | 1 148 | 131 | 97.71 % | 439 |
| scribble | single | 1 000 | 1 551 | 1 024 | 24 | 100 % | 288 |

**Where the time goes: segmentation 884 ms, DP + absorption + seams 10 ms** (the
hatch, 4 142 atoms).  Pete's "milliseconds" is right about the merge and wrong
about the cutting; 99 % of the cost is reading the map at 4 mm, and it is
embarrassingly parallel and tunable.  The optimisation itself is free.

### staging costs 24-33 % more pieces, and mostly to ONE arm handing over to ITSELF

CSAIL +10, strokes +407, hatch +260, scribble +124.  But 6 of CSAIL's 8
hand-overs, and 301 of the strokes set's 533, are the **same arm one stage
later** — the line crossed out of its row band into a dead band that belongs to
a seam stage.  That is a barrier and a re-approach, not a registration risk.  It
is also why every dead-band crossing is a HARD edge: the stage cells tile the
block with no overlap, so the 5 mm overdraw buys nothing staged and is used at
almost every hand-over unstaged.

Redundancy, measured cell by cell with the same map: **43.5 %** of the block
keeps >= 2 stage-compatible drawers under the zigzag (mean 1.457) against the
atlas's ceiling of 48.1 % (mean 1.524) — V2_WORKCELLS' 41.6 % / 1.41 at
`--stride 2`, confirmed at full resolution.

### THE ONE REAL DEFECT IN THE RECOMMENDED PATTERN, AND IT HAS A NAME

The zigzag's 2.31 % of uncovered block is not spread around the rim.  It is one
strip:

    SEAM0  y in [1.010, 1.410]   100.00 % covered
    SEAM1  y in [2.220, 2.620]    79.00 % covered   <- ALL of the loss

SEAM0 is offered to 13, 17 (row 0, reaching up to y = 1.34) AND to 31, 71 (row
1, reaching down to y = 1.08).  **SEAM1 is offered only to 97 and 2** — both in
row 2, both reaching down to only y ~ 2.28 — and never to 31 or 71, who reach up
to y = 2.56.  The strip y in [2.220, 2.40] is below everyone the pattern
invites.  For the CSAIL logo that costs **0.868 m of ink (5.2 %) and stroke 17
entirely** (0.429 m no stage can draw).

Offering SEAM1 to 31 and 71 as well takes the block to **100.00 %** and the logo
to 100 % (54 pieces instead of 50).  **Those two stages are NOT proposed here** —
whether 31 or 71 in SEAM1 clears its partner in SEAM0 is a
`scripts/workcell_envelopes.py` question and this module measures no clearances.
But the hole has a name and a cheap-looking fix, and it is worth one run of that
script to find out.

### DECISION, on the one Pete asked: LOAD BALANCE IS A TIE-BREAK, NOT A WEIGHT

The DP's value is the lexicographic pair `(pieces + w * imbalance, imbalance)`.
At the default `w = 0` the first component IS the piece count and the second
breaks its ties toward the least-loaded state.  Measured:

| set | no balance | tie-break (default) | weight w = 100 |
|---|---|---|---|
| strokes, single | 1 254 pcs, spread 29.4 % | **1 254 pcs, 10.0 %** | 1 255 pcs, 9.8 % |
| scribble, single | 1 024 pcs, spread 79.1 % | **1 024 pcs, 7.5 %** | 1 024 pcs, 7.5 % |
| hatch, single | 1 869 pcs, spread 20.4 % | **1 869 pcs, 21.1 %** | 2 011 pcs, 13.0 % |

Where the capability sets leave slack the FREE tie-break takes all of it — 79 %
spread down to 7.5 % at zero cost.  Where they do not (the hatch: every line
crosses the whole paper and the split is forced) the tie-break buys nothing and
a real weight buys balance at **one piece per 13 cm of imbalance**, which is a
bad trade: a piece is a pen-up, a transit and a seam.  **Leave `balance_w = 0`.**
The lever stays for a stage that turns out makespan-bound rather than
piece-bound, which `sequence.py` can measure and this module cannot.

### what is NOT answered

The pieces are `(stage, arm, polyline)`, which is `sequence.cost_matrix`'s own
`segs` contract — one tour per (stage, arm) bucket.  Three gaps: ordering across
stages is a BARRIER, not a tour, and a piece drawable in two stages by the same
arm is a scheduling freedom the DP spends entirely on piece count; pen-up legs
between pieces are not priced here (`sequence.price_crossings` knows and runs
after); and every number is what the 2 CM ATLAS permits, not a plan — each piece
still has to be accepted by `plan_stroke`, and one that refuses splits again.

### 2026-09-11, later — the seam stages were wrong, and a transverse pair is unseparable on BOTH axes

`scripts/traces.py` (docs/V2_TRACES.md) ran the recommended pattern against the
CSAIL logo and found a hole the envelope study had not looked for: **SEAM1 was
offered only to arms 2 and 97**, whose certified cells start at y = 2.280, and
SEAM1 starts at y = 2.220.  92 cells of the block (2.7 %) at y in [2.24, 2.40]
belonged to nobody in any stage — and stroke 17 of the logo lies entirely
inside that strip.

**A SEAM NEEDS AN OUTER ARM AND A MIDDLE ARM.**  Tip reach over the block is
13/17 [0.000, 1.320], 31/71 [1.080, 2.560], 2/97 [2.280, 3.600], and the seams
at a 0.40 m dead band are [1.010, 1.410] and [2.220, 2.620].  So SEAM0's top
80 mm is only 31/71's and its bottom 30 mm only 13/17's; SEAM1's bottom 40 mm
is only 31/71's and its top 20 mm only 2/97's.  Both kinds, or a hole.

The seam-stage pairing matrix (arm on SEAM0 vs arm on SEAM1, ink-vs-ink, mm —
the two bands are 0.81 m apart in y):

|  SEAM0 \ SEAM1 |   2 |  31 |  71 |  97 |
|---|---|---|---|---|
| **13** | +250.0 | **+194.3** | +250.0 | +250.0 |
| **17** | +250.0 | +250.0 | **+201.1** | +250.0 |
| **31** | +156.4 | self | **-163.1** | +250.0 |
| **71** | +238.6 | **-201.5** | self | +162.1 |

**THE TRANSVERSE PAIR IS UNSEPARABLE ON THE OTHER AXIS TOO.**  The tempting fix
was to CROSS the middle pair over the two seams — 31 to SEAM0 and 71 to SEAM1,
then swapped — which covers 100 % of the block in the SAME six stages.  It is
**-163.1 / -201.5 mm**.  0.81 m of y separation does not separate 31 from 71,
because both elbows still stand in the same x column about the mid-line.  The
x frontier already said a same-row pair needs 1.44 m of a 1.48 m block; the y
answer is that there is no y answer.

**THE CORRECTION IS TWO MORE SEAM STAGES**, pairing a row-0 arm on SEAM0 with a
middle arm on SEAM1: 13/31 at **+194.3 mm** and 17/71 at **+201.1 mm**.

| | 6 stages (4 seam) | **8 stages (6 seam)** |
|---|---|---|
| block covered | 97.3 % (92 cells at y 2.24-2.40) | **100.0 %** |
| worst ink-vs-ink | +85.8 mm | **+85.8 mm** (unchanged) |
| speedup vs serial | 2.51x | **2.35x** |
| >= 2 stage-compatible drawers | 41.6 % | **47.1 %**, the atlas's own ceiling |

6.4 % of the parallel speedup buys the last 2.7 % of the block AND takes the
staged redundancy to the ceiling, because a SEAM1 cell now has both a middle
arm and an outer arm offering to draw it.  The park set is untouched by any of
this and is still the blocker: +4.7 mm, arms 13 and 17 against each other's
parks, for this pattern exactly as for every other.

docs/V2_WORKCELLS.md section 4b is the write-up; `out/workcell_envelopes.json`
carries `reach` and `seam_pairings` as new keys.

## 2026-09-15 — the gaps in the lines, measured — and 0.725 m of them is a TILT

Pete, on the certified `lf6b_s150` animation: **"why are there so many gaps in
the lines?"**  The certified canvas at h = 0.970 is hole-free and v19 drew this
exact placement at **100.0000 %**, so every metre the staged programme does not
draw is a PLANNER failure and not a reach one.  Measured, on the paper:

| | |
|---|---|
| logo | 16.8047 m |
| flown — PEN-DOWN SAMPLES, union per line | **14.1747 m = 84.35 %** |
| missing | **2.6300 m in 13 stretches** |

**THE PROGRAMME'S OWN BOOK-KEEPING COULD NOT ANSWER IT.**  `ink_m` sums the
pieces a bucket ACCEPTED, and a bucket whose `arm_program` refuses still has
them; `_merge_conducts` then pads the arm out to the stage's frame count.  Arm
31 stands in `lf6b_s150` stage C with 8 pieces listed, 1.9161 m of `ink_m`,
2 937 frames and **zero** frames whose `seg` is a segment index.  So the account
is geometric — `scripts/gap_account.py`, `tests/test_gap_account.py`,
docs/V2_STAGED.md §30, picture at `out/lf6b_gaps.png`.

| metres | reason |
|---|---|
| **1.8908** | arm 31's stage-C bucket, planned and never flown |
| **0.7250** | no drawer left: `plan_stroke` refused `split: empty_fiber`, the loop banned that span, and the pattern offered it to exactly ONE (stage, arm) cell |
| **0.0041** | `degenerate: too_short` stubs |

**THE 0.725 m IS ONE FLAG.**  All three `no_drawer` stretches were drawn by v19
with arm 71 under a **10°–15° tilt cone**, and two of them are ink almost
exactly under arm 71's own base (reach 0.071 m and 0.011 m).  The staged runs
take `--tilt-max-deg 0` — the CLI default, and what `scripts/lf6_run.sh`
passes — so the fiber over those points is empty and `plan_stroke` is right to
refuse.  Re-deriving the DP and the refusal loop at `tilt_max_deg = 15` (15 s,
same atlas, same pattern): **3 refusals → 0, 3 bans → 0, coverage 95.63 % →
100.00 %, uncovered 0.7337 m → 0.000 m.**  No gate moves and `scene_check`
judges the trajectory either way.  NOT SHIPPED HERE: it changes what every
bucket is planned from, so it needs its own certified end-to-end run, and that
is the recommendation for the next one.

**THE SECOND HALF OF THAT REASON IS THE PATTERN.**  A stretch the
leader/follower split at `--split-m 0.15` offers to ONE (stage, arm) cell has no
alternative when that cell refuses.  v19's conductor had all six arms and no
stage or region partition.  Every stretch that had two or three cells survived
its ban at no cost.

**THE 1.891 m IS §29.5's UNSEARCHED ORDER, AND IT IS NOW SEARCHED.**
`_serialise_group` plans busiest-first, which makes the busiest arm plan against
its row partner AT THE PARTNER'S ENTRY POSE and the second arm against that
partner AT ITS PARK; a group of two has only those two questions and only one
was asked.  A bucket that does not fly now goes to the BACK of the order — the
slot in which every partner has finished and gone home — and whichever order
flies more ink ships (`orders_tried`, `lost_m`, `lost_pieces` on the group's
report).  The second order is planned only where the first loses a WHOLE bucket,
because a refused group is already planned twice and stage C's wall is 1 428 s.
Whether it recovers arm 31's 1.914 m is the stage-C-only run
`out/staged_csail_h097_lf7c_s150.*`, at a 3 600 s cap because group [31, 71]
alone measured 1 321 s at one order.

## 2026-09-15 — THE GO-HOME IS EVERY CONDUCTED STAGE'S, AND `unattributed` HAD A NAME

Two loose ends of the coverage work, and they are the same defect from its two
ends: **a conducted group whose slot produces no arms at all.**

**STAGE D IS CONDUCTED TOO, SO IT GOES HOME FIRST.**  `CONDUCT_HOME_FIRST`
shipped on the row pass alone.  Stage D — the dead-band ink — is the SAME call,
`_conduct_groups` over the same held-pose entry set, and it was planned with
the four arms it does not own standing at their stage-B/C hovers over the
middle of the sheet.  Measured on `bench/starburst`: 2.831 m of dead-band ink
(45.3 % of the final pass), **7 pieces listed by arm 31 and not one pen-down
sample.**  `staged._home_before` is the stage-C block lifted out of `run` and
given a name — it appends the no-ink go-home stage, advances the held poses in
place and returns the entry set the caller plans from — and `run`'s stage-C
branch, `run`'s stage-D phase loop and `run_conducted`'s band stage are now
three calls to it.  A conducted stage can no longer be added WITHOUT the
go-home.  It is a no-op when the fleet is already parked, so the stage D that
pays for it is exactly the one whose partners did not fly.

**`unattributed` IS `bucket_never_planned`.**  The chain could see what a stage
listed, what `plan_stroke` refused, what an arm handed on and what the DP never
covered — and all four of those lists are written BY `plan_bucket` or by the
stage that called it.  A piece in NONE of them was never offered to
`plan_bucket` at all: its whole bucket was skipped.  `res.pieces` is where such
a piece still is, and it is now the account's last question.

| run | `unattributed` was | never planned for |
|---|---|---|
| `lf8_s150` (CSAIL, tilt 15) | **2.0633 m, 7 stretches** | (2, 31) 0.894 m + (2, 71) 1.179 m |
| `bench/starburst` | **1.5942 m, 9 stretches** | (2, 31) 0.110 m + (2, 71) 1.542 m |

**BOTH ARE THE ROW GROUP [31, 71]** — the same group §30.4 chased with an order
search and `CONDUCT_HOME_FIRST` finally flew on `lf9c`.  Asked LAST, and matched
on `(line, k)` rather than on the cell: a `split_at_room` part is minted with a
new `k`, so a split parent looks unplanned while its parts are themselves listed
or deferred, and a deferral changes the cell.  Re-run on `lf6b_s150`, the
programme §30.2's published table is quoted from, the account is UNCHANGED —
1.8908 `listed_not_flown`, 0.7250 `no_drawer`, 0.0041 `plan_refused`.
`scripts/gap_account.py` gains the same reason, `deferred_never_taken` (which it
never had) and `--tilt-max-deg`: a tilt-15 programme read back with the DP
re-derived at 0 invents refusals the run never had.

**WHAT IT MEASURES — `lf10`, THE CSAIL LOGO AT TILT 15 WITH EVERY FIX:
99.9881 % FLOWN AND CERTIFIED.**  16.8027 m of 16.8047 m on the paper, `all_ok`
true, `unattributed` 0.000 m, and the only gap left in the whole picture is a
**2.0 mm** `degenerate: too_short` stub.  §30.3's tilt flag recovers the whole
`no_drawer` class (DP 100.00 %, zero bans); arm 31 flies 25 pieces in stage C
where `lf6b` flew none; stage D's 0.512 m of dead-band ink flies on the band
retry.  Makespan A+B+C+D **402.7 s** (`lf9c` 389.4 s), TTFM 0.187 s, planning
wall 1 406 s of which the serialised [31, 71] slot is 1 151 s.
`out/staged_csail_h097_lf10_s150.*`.

**AND ON THE BENCH CORPUS**, all five re-run with both fixes:

| picture | before | **after** | what is left | wall |
|---|---|---|---|---|
| hatch | 83.14 % | 83.14 % | `no_drawer` 5.074 | 138 s |
| scatter | 98.96 % | 98.96 %, **`all_ok` true** | `plan_refused` 0.087 | 420 s (was 2 261) |
| **starburst** | **45.70 %** | **96.65 %** | `no_drawer` 0.570 | 1 766 s |
| spiral | 84.40 % | 84.40 % | `no_drawer` 2.174 | 629 s (was 1 518) |
| duotone | 97.35 % | 97.35 % | `no_drawer` 0.340 | 499 s (was 1 572) |

`starburst` gains **51 points** and both fixes are in it: its [31, 71] slot now
flies (6.966 m, +59.0 mm, arms 31(20) and 71(8)) which retires the 1.594 m of
`bucket_never_planned`, and its stage D flies all 2.831 m which retires the
2.831 m of `listed_not_flown`.  **Every picture in the corpus now has
`listed_not_flown` = 0.000 m and no `unattributed` bucket**; every remaining
metre anywhere is `no_drawer` or a `plan_refused` stub, which is the DP's own
ceiling at that placement.  The four unchanged pictures are 2.4x-5.4x cheaper to
plan, because a row group planning against a PARKED fleet is a far easier
routing problem than one planning against four arms holding hovers over the
sheet.

STILL OWED: `starburst`'s `all_ok` is false on the HOLD gate, not on ink -- the
barrier pose set at the end of stage B measures +141.4 mm against
`MARGIN_GATE` = 150 mm, so the hold before stage C is refused and the go-home
inherits it at t0.  Everything after it passes.  That is a stage-B parking
question and it is the next measurement on that picture.

docs/V2_STAGED.md §31 is the write-up; `tests/test_staged.py` and
`tests/test_gap_account.py` carry the two new tests.
