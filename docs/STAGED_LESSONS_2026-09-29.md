# Lessons the staged planner learned the hard way

Extracted 2026-09-29 by an Opus agent reading `docs/V2_STAGED.md`, `docs/DECISIONS.md`,
`docs/HARDWARE_DAY1.md` and the package, for the rebuild described in `REBUILD_PLAN_2026-09-29.md`.
Purpose: a from-scratch reimplementation must not rediscover these. Every row cites where it was
checked. Items marked "→ADD Cn/Bn/D" refer to an addendum the agent produced in an earlier turn that
was not captured; the citation stands, the explanation is in the cited file.

All paths relative to `/home/franka/aris_project/aris_sixarm`. Package files are `aris_sixarm/<mod>.py`.

## 1.1 Transits diving through the paper / IK branch flips on joint interpolation

| # | rule | citation | breaks without it |
|---|---|---|---|
| L1 | A pen-up straight line in joint space between two certified hovers is **not safe** and was never checked. Shipped `csail_final6`: 3 of 42 pen-up blocks below z = 0, worst pen tip **253.6 mm under the canvas** (arm 97 going home, t = 49.33 s), worst chain point 156.1 mm, one PARKED there for 0.75 s by a conductor pause. Cause: "the long reconfiguration — a metre of travel that swaps elbow branch on the way — that dives." | `paper.py:1-21` | the arm draws through the table |
| L2 | Poses clearing the self-guard by 63.7 mm at both ends read **−194.7 mm partway** across a straight interpolation. `writing.py` makes no collision guarantee and says so. | `docs/DECISIONS.md:3900-3903`, `:3843` | the arm folds through itself on a pen-up |
| L3 | The pen tip is **not a chain point** — FK stops at the flange — so a 110 mm pen can be under the table while every link the validator sees is above it. Hence `TIP_CLEAR` 0.02 separate from `CHAIN_CLEAR` 0.02, plus `TIP_TOL` 0.010 contact band. | `paper.py:36-58` | validator passes a pen 25 cm under the paper |
| L4 | **A fold is an IK branch change and no walk over hover poses avoids it.** Adding 8 cm and 5 cm rungs to `TRAVERSE_STEPS` recovers 2 of 520 crossings for 1.9× the clock. | `DECISIONS.md:3930-3936`, `:3978-3984` | you rebuild the ladder, add rungs, get 2/520 |
| L5 | Therefore a **C-space RRT tier is mandatory**. "the arm can FLY there" 84.05 % → 95.34 %; feasible area 5.568 → 6.316 m². Of 66 ladder-exhausted crossings the planner flies 65; 0 of 189 paths refused by `legs_ok` on re-check. 1.90 s/plan. | `DECISIONS.md:3986-4008`, `:4045-4070`; `transit.py` | 11 points of canvas, permanently |
| L6 | Lowering the hover does **not** help: extending `HOVER_LADDER` to 25/20/15/10 mm turns 0 of 8 cells at h = 0.940 and 0 of 7 at 0.970. | `DECISIONS.md:4517-4546` | a week on the wrong knob |
| L7 | A straight lift can be *worse* than a flip: for an inverted arm reaching outboard, lifting swings the elbow toward the neighbour's column — static clearance 63.7 → 38.7 mm at z = 0.06 → 21.9 mm at z = 0.10. | `DECISIONS.md:4230-4249` | "just lift higher" makes it worse |
| L8 | **Transits must be in the exported file.** Measured on v18, arm 31: 4.4 rad on j3, 4.7 on j5 at stroke boundaries. A file of `draw` rows only is not executable — in SIL the arm hit joint limits at the first boundary. Every row carries q1..q7, pen-up rows included. | `HARDWARE_DAY1.md:855-859`; `ARIS2_CONTRACTS.md:41-47` | the arm dies at the first stroke boundary |
| L9 | The **entry leg park→first hover** is the binding test — every bucket that failed in the first envelope run failed on "entry at segment 0 cannot clear the paper plane". | `V2_STAGED.md:524-528` | you debug the wrong leg |
| L10 | A second, identical hole one obstacle over: `STATIC_SAFE`. | `paper.py:151-193` | →ADD C2 |

## 1.2 Ready/park poses with the pen below the paper; parked and idle arms as obstacles

| # | rule | citation | breaks without it |
|---|---|---|---|
| L11 | The **inverted ready pose puts the pen through the paper**: tip 16 mm below with a 200 mm pen, 113 mm below with a 300 mm one. | `scene_check.py:112-145` | →ADD C1 — conductor v1 parked four arms through the table three times a run and no check noticed |
| L12 | `hold_gap` must preserve the `PEN_PAPER` exemption — a pen resting on the paper is reported, not a hard violation. | `staged.py:1538-1541` | every barrier refuses |
| L13 | **HARDWARE. `--idle-policy home`, not `freeze`.** Under `freeze` arm 31 ended 4.66 rad from its park, in its neighbour's path. Alternating variant 8.79 mm FAIL → 155.56 mm PASS, and faster (53.02 s vs 61.77 s). "The arm was never certified to *stand* there while its neighbour worked; only to *pass through*." | `HARDWARE_DAY1.md:716-743` | a hard collision on the first two-arm run |
| L14 | Parked partners go in as **real capsules**, not pose-invariant bands. Arm 71's 8 pen-up legs all refused against `body:31_column3` while arm 31's actual parked capsules clear by 127 mm. Price: the certificate depends on the named neighbour holding the named pose (`frozen_dependency`). | `frozen.py:13-32` | the band refuses what the arm does not |
| L15 | `drop_bands` drops only `body:*_column*`, and only for arms in the frozen set. True structure is never dropped. | `frozen.py:21-26`; `staged.py:1563` | the same arm counted twice |
| L16 | `scene_check` must drop the base-column band of an arm present in the timeline. | `DECISIONS.md:1796-1806` | false FAIL from self-double-counting |
| L17 | **The bounding box of a cylinder is mostly air.** Band 3's AABB hangs 160.0 mm below where the arm's body ends; measured fake collisions +144.2/+138.5/+161.6 mm. The bands ARE the exact swept solid (J1 turns about base z ⇒ solid of revolution). `NO_HOVER` cells 7→0 and 8→0. | `DECISIONS.md:4602-4689`; `envelope.py` | "that is a fake collision" — Pete |
| L18 | **A park must be searched against the ROUTER's floor (63 mm), not just the pose gate (50).** "A depot the arm cannot fly out of is not a depot." | `layout.py:687-700` | a park you cannot leave |
| L19 | It is **the checker's** number that decides: `scene_check.static_clearance_lb` reads 9.9 mm under `rig_final.chain_static_clearance` for the same pose. | `layout.py:702-709` | the programme fails the only verdict anybody ships |
| L20 | A park candidate inside the steel is not a candidate: `static_floor` removes them before ranking. Two of eight staged parks were 35.3 mm inside metal. | `layout.py:1093-1125`; `DECISIONS.md:987-999` | interpenetrations, not tight margins |
| L21 | **A park set never transfers across a height or a tool.** The 0.940 grid applied lower lands inside the ink: −46.6 mm at 0.910, −126.0 at 0.850. At a new tool the old grid parks 13 and 17 53.7 mm apart. | `DECISIONS.md:3425-3432`, `:2702-2710`; `layout.py:1620-1660` | silent invalidation on every geometry change |
| L22 | Park ranking must include the lift layers and the bearing (held to the outward ray alone, arms 2 and 97 are 111.0 mm inside each other). | `DECISIONS.md:3459-3462`, `:2737-2742` | |
| L23 | The park's ceiling is the **layout**, not the pose: 22–42 candidates/arm all on a 97.8–98.9 mm plateau; 7.5× the grid moved the binding numbers by zero. | `DECISIONS.md:2730-2735`, `:1849-1858` | weeks searching for a number that cannot move |
| L24 | Ranking an aside park by clearance alone yields one the arm cannot fly to: at h = 0.970 rung 1 (r = 0.70, hover 0.35) is REFUSED for five of six arms. | `DECISIONS.md:5477-5538` | |
| L25 | Idle policy is measured: conductor v1 sent every finished arm to `q_seed` and **79 % of every pause-second was an arm waiting to reach its PARK pose**. Three policies: freeze-in-place / minimal retreat / JIT pre-position. | `idle.py:1-45` | 79 % of the pause budget |
| L26 | Park selection must be deterministic by construction: fixed enumeration, total order `(-clearance, index)`, worst-constrained arm served first. | `layout.py:1087-1090` | an unreproducible fleet |
| L27 | **HARDWARE.** `--dy -0.10` is not taste: arm 31 refuses a line on the seam line y = 1.8153 because its go-home leg does not clear the paper plane there. Pinned in `tests/test_day1.py`. | `HARDWARE_DAY1.md:566-571` | |

## 1.3 Sampling-rate-dependent gates and gate refinement

| # | rule | citation | breaks without it |
|---|---|---|---|
| L28 | **Clearance verdicts are rate-dependent.** Same trajectory reads 28.23 / 36.58 / 39.38 mm at dt = 0.05 / 0.02 / 0.01. `scene_check` subtracts a 1-Lipschitz residual `0.55 × (step_i + step_j)`. | `DECISIONS.md:1204-1212`, `:1300-1312`; `V2_STAGED.md:993-1008` | a PASS that flips to FAIL when you change resolution |
| L29 | **Auto-refine every gate, including inter-arm.** Worth 11 mm: "Eleven millimetres of the deficit was the sampling". `REFINE_MAX = 3`, `REFINE_FRAMES = 40000`. | `V2_STAGED.md:993-1008`; `staged.py:1645-1662`; `scene_check.py:719-810` | a false FAIL on the tightest pair |
| L30 | Charge the residual per interval and bisect only where it binds (`ADAPT_TOL` 0.5 mm, `ADAPT_CAP` 4097). | `DECISIONS.md:4194-4214` | geometrically-clear cells refused by arithmetic |
| L31 | **The trajectory residual was being paid TWICE** on every pen-up leg: a leg needed 65.5 mm true clearance for a nominal 63 mm floor. A third instance worth 39 of 108 hole cells at h = 0.940. | `DECISIONS.md:4251-4276` | 2.25 mm of free clearance per leg, thrown away |
| L32 | The producer/judge gap is itemised, not padded: `STATIC_PLAN_MARGIN = 50 + 12.75` (10.0 checker half-step + 2.75 Lipschitz). RRT pays `transit.PAD` = 2 mm more. "The producers pay it so the checker is a second opinion and not a lottery." | `DECISIONS.md:4253`, `:4004`; `paper.py:78-110`; `V2_STAGED.md:869-883` | the checker becomes a coin-flip |
| L33 | `effective_static_floor` clamps a leg's floor to what its endpoints can hold. Corollary: **no routing fixes a pose.** | `DECISIONS.md:1216-1222`; `paper.py:1502`; `V2_STAGED.md:1010-1023` | every edge `inf`, and you blame the router |
| L34 | A park inside a room clamps that floor negative before routing begins; the failure looks like a routing failure. | `V2_STAGED.md:1420-1425` | you chase the router for a placement bug |
| L35 | **The bar for a new piece END is `PAIR_MARGIN` (50), not `FRAME_FLOOR` (63)** — a hover is a pose the arm holds. | `V2_STAGED.md:1584-1596` | the whole room-boundary cut yields nothing |
| L36 | `active_pair_gap` must see only arms that move — charging a still arm the mover's residual cost 30 mm of a 50 mm gate. | `V2_STAGED.md:1201-1211`; `staged.py:1427` | a 60 % false deficit on every held arm |
| L37 | Active-vs-active is the cross product of pose sets, never `check_timeline` on a merged clock: "checking one would certify a schedule nobody runs." | `V2_STAGED.md:110-118` | you certify a schedule nobody runs |
| L38 | `pad = 0` is honest only at stride 1; the cluster cell, not the pad, is the conservatism. | `DECISIONS.md:1517-1531`; `V2_STAGED.md:545-563` | you tune the pad and the cell is the problem |
| L39 | **A setting that flies only by not looking is not a setting.** Stride 1 flies fewer buckets than stride 2 because it reads every cell. Choose on the certificate, not the bucket count. | `V2_STAGED.md:624-628`, `:682-689` | you ship the setting that sees least |
| L40 | Raw programme waypoints must be resampled through `writing.uniform_samples` before checking — a fixed dt over raw waypoints reported two parked arms at −437 mm. | `DECISIONS.md:1314-1319`; `V2_STAGED.md:885-889` | a plausible, fictional number |
| L41 | **Room queries must be FLOORED to be affordable**: 0.05 ms/pose floored at `FRAME_FLOOR`, 4.6 ms unfloored. | `V2_STAGED.md:1381-1390`; `exact_room.py:48-58` | a 90× slower kernel for no extra safety |
| L42 | The room's pad is `scene_check`'s own between-sample residual, so the room covers motion between samples. | `V2_STAGED.md:718-725` | a room only valid at the samples |
| L43 | `exact_room`'s broad-phase grid is a PRUNE chosen for cost; `CELL = 0.10` is the measured knee; the answer is bit-identical at every setting. | `exact_room.py:42-58` | |

## 1.4 Velocity / acceleration caps and where they bind

| # | rule | citation | breaks without it |
|---|---|---|---|
| L44 | **HARDWARE, the most dangerous one.** *Acceleration depends on the rate you measure it at.* Same trajectory: 37.70 rad/s² at 48 Hz vs **1 422 rad/s² resampled to 1 kHz** (driver gate 10.0). `pacing.py` bounds velocity and nothing else. Time-scaling fixes the sampled acceleration, not the path: at a corner the true acceleration is impulsive however slowly you fly it. | `HARDWARE_DAY1.md:908-933`; `pacing.py` | you ship "1.94× slower" and the driver refuses at the rig |
| L45 | **The velocity certificate holds at `t_s` and no other timing.** The pacer hits `QD_FRAC` (0.30) exactly — "Anything near 100 % is a bug upstream." `rtff_pathway_exec` paces by Cartesian arc length: run it quicker than `t_s` says and every speed scales with it. | `HARDWARE_DAY1.md:653-670`; `writing.py:758`; `frames.py:31` | a certificate that does not apply to the run |
| L46 | **One re-time rate for the whole fleet** — `execute.Governor`, one scalar, no per-arm entry point. | `HARDWARE_DAY1.md:944-947`; `execute/` | per-arm retiming voids every pair certificate |
| L47 | Pacing is not why pen-ups are slow: 70.5 s of 95.9 s in pen-up is what the legs do, not how fast they may do it. | `V2_STAGED.md:2197-2202` | you tune speed and the geometry is the problem |
| L48 | The self gate was a density disagreement and the fix is PACE, not a gate: `writing.self_pace_beat` stretches dt until `lb − 0.55·rate·dt_play/dt ≥ 20 mm`. | `V2_STAGED.md:2114-2151`; `writing.py:1702` | you lower the self gate |
| L49 | The pacing bound must be the converged one; refinement can only ever RAISE a bound and ask for LESS stretch. | `V2_STAGED.md:2540-2549`, `:2764-2780`; `DECISIONS.md:449-466` | you stretch a leg 5× for a sampling artefact |
| L50 | The pair gate needed pacing too, and the price is per-interval. | `V2_STAGED.md:2705-2762`; `writing.py:1866`, `:1893` | →ADD C4 (5 476 s vs 67 s) |
| L51 | Rendering fidelity and drawing speed are the same quantity: tightening `writing.densify` from 0.762 mm to 0.2 mm chord error took one stroke 66.9 → 552.0 s (8×) because that stretch is a near-null-space wrist reconfiguration. Fix still open: let the stroke planner price the null-space reconfiguration it chooses. | `DECISIONS.md:4098-4167` | an 8× slowdown from a quality knob |

## 1.5 The pen-up / reconfiguration gate

`paper.py` exists because "everything that touches the paper was certified against it and nothing that flies over it was" (`paper.py:3-4`). Floors: `CHAIN_CLEAR` 0.02, `TIP_CLEAR` 0.02, `TIP_TOL` 0.010, `TIP_SWEEP_PAD` 0.003, `CONTACT_FLOOR` −0.007, `FRAME_FLOOR` 0.063.

| # | rule | citation | breaks without it |
|---|---|---|---|
| L52 | Two near-misses set the last two floors: tip −10.3 mm against a −10 mm floor; "a route that cleared 53 mm exactly read 47.4 mm", moving the floor 50 → 63. | `paper.py:80-110` | a 0.3 mm veto kills a whole run |
| L53 | **The hover was chosen for clearance and never for nearness — an IK branch flip.** `hover_solve` returned a pose 5.03 rad from the ink while the fiber held 23 gated poses, nearest 0.26 rad. `HOVER_NEAR = 1.0` rad: lift+lower 48.39 → 16.76 rad (−65.4 %). "Nothing is relaxed. Continuity is a tie-break among poses that are already certified." | `V2_STAGED.md:2273-2306`; `writing.py:1114` | two thirds of the arm's time is reconfiguration |
| L54 | **The IK sheet must be chosen along the tour, not per piece.** `allocate.chain_sheets` = Viterbi O(n·K²) after `sequence_arm` fixes the order; alternative 0 is the already-certified plan; `menu` variants advertise entry/exit configs without planning anything. | `V2_STAGED.md:2308-2353`; `staged.py:89-98` | the transit folds the arm over between adjacent pieces (27.6 rad on two legs) |
| L55 | The depot via jumped the ladder: `[q_home]` (42 cm up) was offered before the 8 cm rung. `HOME_AFTER_LADDER`. | `V2_STAGED.md:2361-2364`, `:2474-2477`; `paper.py:208` | half the pen-up clock |
| L56 | `paper.SHORTCUT`, 8 rounds: vias dropped one at a time, kept only when re-certified end to end. "A shortcut can never be looser than what it replaces; only shorter." | `V2_STAGED.md:2366-2373`; `paper.py:220` | |
| L57 | The pen-up classifier: `flip` tested before `tall`; `RATE_REF = 25 rad/m` so only clear outliers are named. | `V2_STAGED.md:2210-2238`; `scripts/penup_anatomy.py` | numbers you cannot quote |

## 1.6 The hold gate

| # | rule | citation | breaks without it |
|---|---|---|---|
| L58 | `frozen_failed` is `validate_pose` on the last pose of each arm's trajectory (the pose HELD through the barrier). Arm 71: one hard violation `['margin']` at 0.11108 rad vs `MARGIN_GATE` 0.15. | `V2_STAGED.md:2389-2424`; `validate.py:51` | you debug the standoff for a joint-margin bug |
| L59 | `HOVER_MARGIN` (0.10) vs `MARGIN_GATE` (0.15) was latent for months; callers CHOOSING a pose must ask for the stricter gate. | `V2_STAGED.md:2426-2447` | a barrier holds a pose the judge refuses |
| L60 | The first fix was withdrawn; the real fix was a score, not a gate. | `V2_STAGED.md:2559-2658` | →ADD B2 |
| L61 | `HOVER_ROOM_FLOOR = 0.075`, not 0.050: `solo_check` measures the whole timeline including the LEG out of the held pose. | `V2_STAGED.md:2643-2647`; `writing.py:112` | the fix does not fix it |
| L62 | **A best-effort ask must be VERIFIED, and a pose that fails it is not held.** `writing.held_pose_ok` asks of the pose that came back; `hold_candidates` is the retreat ladder: different height over the same end, hover over an earlier end of the same bucket, the park (always valid, last). Cost: 4.6 s and no ink. | `V2_STAGED.md:2989-3024`, `:3112-3118`; `writing.py:1596`, `:1618`, `:140` | |
| L63 | A serialised convenience field disagreed with the truth (0.136 vs 0.6023 rad). "The trajectory is the truth." | `V2_STAGED.md:2660-2665`, `:2814-2824` | the next stage plans from a pose nobody holds |

## 1.7 The allocator handing an arm ink it cannot fly

| # | rule | citation | breaks without it |
|---|---|---|---|
| L64 | **A gate the ROUTER does not know about is a gate the allocator spends its whole budget walking into.** Ungated pen-up legs → 100.0000 % allocated, then `scene_check` refuses at −177.8 mm; gated → 94.70 % that conducts. | `DECISIONS.md:3923-3926` | coverage numbers are fiction |
| L65 | `plan_stroke` never consults the static set; `staged.ink_vs_envelope` is the missing half. | `V2_STAGED.md:339-342` | certified ink that cannot be flown |
| L66 | Two ways a piece fails the room: its INK passes through (per-piece refusal) or its pen-up LEG is unroutable (a bucket statement). Drop the piece closest to the room, retry ≤ `LF_MAX_DROPS` (4). | `staged.py:2805-2817`, `:88` | you refuse the wrong piece, repeatedly |
| L67 | The ink gate is the follower's gate: a leader's clearance is reported, a follower's is a refusal. | `staged.py:2996-3002` | |
| L68 | **A refusal bans only the stretch past `s_star`** — five of six CSAIL refusals carried a certified head. | `staged.py:1822-1846`; `V2_STAGED.md:371-374` | you throw away the certified 60 % |
| L69 | **A ban must CUT the atom, not clear its bit.** Banning whole atoms took CSAIL from 100 % to 80.6 %. | `staged.py:1694-1700`; `DECISIONS.md:1602-1615` | ~20 points of coverage |
| L70 | A `degenerate` refusal bans nothing; a span is never banned twice (termination). | `staged.py:1822-1846`; `V2_STAGED.md:381-385` | a non-terminating loop, a spreading hole |
| L71 | **A piece the room refuses is CUT at the room boundary, not refused whole.** Runs ≥ `SPLIT_MIN_M` (10 mm); a new end that will not hold a hover is walked inward; parts take ids from `SPLIT_ID0 = 1000`. | `V2_STAGED.md:1554-1582`; `staged.py:909`, `:759-773` | the follower keeps nothing |
| L72 | **A stretch offered to exactly ONE `(stage, arm)` cell has no alternative when that cell refuses.** This is spiral's 1.5 m — the real planner ceiling. | `V2_STAGED.md:3268-3300` | |
| L73 | **The refusal fraction is what the 2 cm atlas permits**: 11.1 % on CSAIL, 18.1 % on 1 000 lines, two thirds the redundancy band (`empty_fiber` + `start_infeasible`), not the reach. | `V2_STAGED.md:180-232`, `:92-99`; `staged.py:1680-1688` | you believe the atlas |
| L74 | Cut at every capability transition, merge with an O(atoms × states) chain DP, not a greedy: 38 of 39 strokes survive in one piece; DP 10 ms. Balance is a lexicographic tie-break, never a weight. Seams go in the middle of an overlap zone. | `traces.py:16-45`; `DECISIONS.md:5703-5754`, `:5792-5810` | confetti instead of corridors |
| L75 | Route the cost matrix cheaply, then refuse-and-reprice: routing all 676 crossings up front costs ~100 s (410 dive) to buy a tour 3 s cheaper. | `HARDWARE_DAY1.md:601-613` | 100 s for 3 s |
| L76 | **The tilt cone is the single biggest coverage lever and it is not the default.** 15° took CSAIL 95.63 → 100.00 % and duotone 85.42 → 97.23 %. "It is the vertical pen, not the reach." | `V2_STAGED.md:3266-3300`, `:3328-3347`; `staged.py:5128` | ~12 points corpus-wide from one default |

## 1.8 Silent ink dropping / the no-drop invariant

| # | rule | citation | breaks without it |
|---|---|---|---|
| L77 | **The programme's own book-keeping cannot see the hole.** Arm 31 stage C: 8 pieces listed, 1.9161 m reported, 2 937 frames, ZERO pen-down frames. The run reported `all_ok: true, coverage: 0.956`. | `staged.py:4426-4441`; `V2_STAGED.md:3218-3238` | a planner that reports 100 % and draws 84 % |
| L78 | So the account is geometric: drawn = a frame whose `seg ≥ 0`; residual = input minus every drawn piece's polyline, matched per line at 2.5 mm on a 2 mm grid. | `staged.py:4442-4444`, `:4498`; `tests/test_gap_account.py:90` | a hole hidden by a neighbouring line |
| L79 | It is a structural invariant with a `raise`. | `staged.py:2382-2398` | →ADD B1 |
| L80 | Six reason codes in a fixed order, `unattributed` last and treated as a bug: `listed_not_flown, no_drawer, plan_refused, deferred_never_taken, bucket_never_planned, unattributed`. | `staged.py:4445-4446`, `:4550-4564` | |
| L81 | `bucket_never_planned` had to be added after 2.063 m had no reason code — a piece never offered to `plan_bucket`. Match on `(line, k)`, not the cell. | `V2_STAGED.md:3423-3460` | metres in the DP and in no stage's book |
| L82 | A stage in which nothing flew passed both checks. `StageResult.ok` now requires `complete`; prints EMPTY, not PASS. | `V2_STAGED.md:816-822` | PASS on an empty stage |
| L83 | A group that flew nothing reported `+inf mm, PASS`. | `V2_STAGED.md:2071-2075`; `DECISIONS.md:468-473` | |
| L84 | `gap_account` must be read at the tilt the run used. | `V2_STAGED.md:3455-3456` | fabricated reason codes |
| L85 | **HARDWARE.** The log's COVERAGE line is the allocator's belief; the conducted number is `coverage` in the schedule JSON. Measured gap: log 84.2 %, json 0.0123. | `HARDWARE_DAY1.md:299-304` | you believe a number off by 68× |
| L86 | A certified run can be certified by giving ink up, and that must be said. | `V2_STAGED.md:3124-3139` | you book a loss as a win |
| L87 | Do not quote a makespan for a partial programme. | `V2_STAGED.md:458-460`, `:814-819` | |

## 1.9 Seam bars and frame gates

| # | rule | citation | breaks without it |
|---|---|---|---|
| L88 | **The shipped v19 programme FAILS against the seam bars** (−59.4 mm, arms 31/71). Bar geometry is REPRESENTATIVE, not measured. `ARIS_SEAM_POSTS=0` is for regression only: never fly anything planned with it. | `HARDWARE_DAY1.md:1110-1145`; `DECISIONS.md:920-939`; `mounts.py:341`, `:546` | you fly a programme certified in a room that no longer exists |
| L89 | Only arms 31 and 71 are ever the offender — their J1 axes are on the seam plane. | `DECISIONS.md:987-999`; `layout.py:1650-1660` | |
| L90 | **The FRAME gate and the COLUMN gate disagree on the same metal**: COLUMN +84.35 mm PASS, FRAME (0.32 m AABB) +31.03 mm FAIL. `CONDUCT_BANDS = "auto"`: retry with bands only where the judge refuses, only if the retry clears. | `V2_STAGED.md:2782-2812`; `DECISIONS.md:104-116`; `frozen.py:126-150`; `staged.py:3834` | an unfixable FAIL on geometry that passes |
| L91 | `frozen.freeze_sets` clears the keep-bands flag; callers must restate it. | `V2_STAGED.md:3052-3055` | →ADD C7 |

## 1.10 Room construction, order, and the shape of the certificate

| # | rule | citation | breaks without it |
|---|---|---|---|
| L92 | **The pose-union work-cell envelope is the wrong object, not a badly bounded one.** What the neighbour actually holds in a stage is one trajectory — a room a leg can be routed around. Ink in a certified trajectory 34.4 % → 97.6 %; 11× faster planning. | `V2_STAGED.md:711-716`, `:915`, `:923-927` | you cannot route a leg anywhere |
| L93 | `cluster_capsules` is conservative by construction — licenses reuse as the exact room's broad phase. | `V2_STAGED.md:325-329`; `exact_room.py:26-32` | an unsound prune |
| L94 | **A priority order closes the room in one sweep and turns the dependency cycle into a DAG** — a re-plan of arm k invalidates only k+1…n. | `V2_STAGED.md:826-851` | →ADD C5 |
| L95 | The iteration is not the certificate: "the iteration gets the trajectories apart; the check proves they are apart." | `V2_STAGED.md:727-734`, `:853-855` | you ship a fixed point as a proof |
| L96 | A staged programme is a graph of certificates: `ArmStage.depends_on` + `trajectory_digest` so a re-plan leaves stale neighbours stale by inspection. | `V2_STAGED.md:736-746`; `staged.py:597-605` | silent staleness |
| L97 | ROLE FIRST, INK SECOND: ink order is a tie-break within a role. | `staged.py:2782-2789` | the labels mean nothing |
| L98 | Pass 1 is solo against the parked fleet, always. | `staged.py:2104-2110` | no seed for the iteration |
| L99 | The LF mode is declared by the pattern, not by a flag: "half of that scheme is not a scheme." | `staged.py:2086-2101` | half a scheme |
| L100 | `staged.run` refuses a pattern putting a same-row pair in the air unless the pattern declares `same_row_ok`. | `staged.py:2091-2095`; `traces.py:554` | |
| L101 | The stage verdict is the realised-trajectory check with the planner's room thrown away (`_check_stage` calls `thaw()` first). "A build that reports both and believes neither is not a build." | `V2_STAGED.md:1663-1668`, `:1649-1662`; `staged.py:4199` | the planner marks its own homework |
| L102 | A refused group's own two arms were never in each other's room (2↔97 −161.3 mm) because `plan_bucket` runs before `freeze_conduct`. `_serialise_group` fixes it and made stage C faster (116.7 → 98.1 s): "a leg routed around the truth is shorter than a leg routed around nothing and then measured against the truth." | `V2_STAGED.md:2921-2980`, `:3041-3045`; `staged.py:3057` | routing straight through a standing arm |
| L103 | `freeze_conduct` must also hold the arms inside the group that produce no timeline. | `V2_STAGED.md:2926-2930` | |
| L104 | The merge had to learn the order: the certificate is ordered. | `V2_STAGED.md:2982-2987` | the certificate is inverted |
| L105 | The serial slot was measured in the wrong unit (row count, not seconds): 17.65 s written where 59.70 s flew → −108.6 mm. | `V2_STAGED.md:2892-2906`; `staged.py:3640`, `:3661` | a 3.4× understated slot and a collision |
| L106 | The merge pads every arm to the longest group's clock and runs one `check_timeline` over all six, plus `hold_gap` over the first frame (`t0_holds`): t = 0 is when all six leave their held poses at once. | `V2_STAGED.md:1642-1647`, `:2104-2106` | six arms leaving at once, uncertified |
| L107 | A piece rides with a row conductor only if its whole geometry stays inside that row's band; straddlers → stage D (`BAND_SHARE_MAX = 0.05`). | `V2_STAGED.md:1613-1621`; `staged.py:3503`, `:100` | a straddler puts an elbow where no argument reaches |
| L108 | Row conductors do not compose as first built. | `V2_STAGED.md:1857-1899` | →ADD C6 |
| L109 | `priority` row-compose is sound and unusable: a 31 920-capsule swept volume nobody can plan against. `serial` ships. | `V2_STAGED.md:2524-2532`, `:2673-2679` | |
| L110 | **A conducted stage must go home first, structurally.** Stage D planned with four arms holding hovers over the sheet: 2.831 m listed, 0 pen-down samples. `_home_before` is the one call all three sites make. Worth +51 points on starburst. | `V2_STAGED.md:3394-3421`, `:3522-3535`; `staged.py:3893`, `:3861` | a whole stage lists ink and draws none |
| L111 | The rows were never frozen during the conduct: `idle.conduct` re-plans and re-routes; `thaw()` between plan and conduct made four arms invisible to both halves of the safety argument (2↔31 at −204.6 mm with arm 2 standing still). | `V2_STAGED.md:2038-2069` | thawing the room mid-pipeline |
| L112 | "It happens first" is not a defence against a claim quantified over all pairs. | `staged.py:2946-2969` | →ADD C3 — directly hits the rebuild plan's incremental dispatch |
| L113 | The residue pass is the honest floor: the one-arm-at-a-time programme. `solo_check` against the parked fleet IS the certificate a one-mover conduct produces. | `V2_STAGED.md:1025-1041`; `staged.py:2515-2534` | no floor |
| L114 | The order search is worth keeping for what it proves, not what it finds. | `V2_STAGED.md:1079-1086` | |
| L115 | A follower's leftover is DEFERRED, never serialised inside a stage. | `V2_STAGED.md:1168-1174`; `staged.py:2569-2575` | you rebuild the barrier you removed |
| L116 | The envelope argument was one quantifier too strong: "no same-row pair in the air together" was inferred from full envelopes; with rooms + standoff it is false. | `V2_STAGED.md:1133-1150`, `:1319-1326`, `:2015-2018` | a permanent, false structural constraint |
| L117 | **A pattern that defers most of the picture to the conductor has reinvented v19 with a slower planner.** Stage C: 3 075 s wall for 126.6 s of motion. | `V2_STAGED.md:1328-1333`; `DECISIONS.md:1122-1129` | the headline metric |
| L118 | Do not recalibrate a throughput model while the main stages do not carry the ink. | `V2_STAGED.md:1335-1342`, `:3165-3180` | →ADD C8 |
| L119 | **The staged model has no ink dimension**; a two-colour picture is wrong rather than slow. An ink is a constraint on which arm may draw a stroke at all: it belongs in the DP's capability map. | `V2_STAGED.md:3349-3383` | an incorrect programme, silently |

## 1.11 Determinism / replay

| # | rule | citation | breaks without it |
|---|---|---|---|
| L120 | **Determinism is a seed, not a hope.** RRT seeds are `blake2b` over the scene signature plus both endpoints — never `hash()`, never `id(spec)`. Tests re-plan in a cold interpreter under a different `PYTHONHASHSEED`. | `transit.py:79-85`, `:199-225`; `DECISIONS.md:4010-4016` | unreproducible plans |
| L121 | The one exception is named: `transit.TIME_BUDGET` (2.5 s/attempt) is not reproducible and says so. | `transit.py:124-134` | |
| L122 | `id(spec)` in a memo key makes a cache process-local. Content digest + weak reference. Worth 1118× wall. | `DECISIONS.md:2002-2047` | 36.5× on the whole pipeline |
| L123 | The store signature must contain both floors (50 and 63); `paper.ROUTE_REV` rides in `cache_signature()`. Write-once JSON + `os.replace`. | `DECISIONS.md:2015-2030`; `paper.py:222`, `:336` | a prefilter served as a certificate |
| L124 | The leg store must be namespaced on the room (frozen poses, envelopes, standoff, keep_bands) and on `HOVER_NEAR` / `HOVER_HOLD_MARGIN`. | `staged.py:297-333`; `V2_STAGED.md:335-338`, `:1776-1780`, `:2375-2378` | a leg certified in the wrong room |
| L125 | S = 0 is a bit-identical no-op: the standoff is written into cache keys only when non-empty. | `frozen.py:117-123`; `staged.py:310-322` | a feature that invalidates every cache |
| L126 | Two caches under one key, and an O(n²) in `cluster_capsules`. | `V2_STAGED.md:574-579`, `:691-695` | →ADD C9 |
| L127 | A cache whose only validity test is "the file exists" is a trap (the GUI 3D scene cache served a stale build). A second home for a constant WILL drift (`abs(h − 0.940) < 1e-9` baked in). | `DECISIONS.md:5579-5597`, `:5565-5578` | a silent, correct-looking wrong answer |
| L128 | `exact_room` is bit-identical to brute force at every cell size; `dead.py` walks a deterministic subsample; warm and cold runs produce byte-identical makespans. | `V2_STAGED.md:1378-1379`, `:166-167`; `dead.py:313-314` | |
| L129 | A regression found by a pinned test means the shipped claim is withdrawn; bisect knob-by-knob in a clean worktree. | `V2_STAGED.md:2559-2574`, `:3194-3208` | |
| L130 | A re-run must be offered the accepted set. | `V2_STAGED.md:2107-2112`; `staged.py:5018` | |

## 1.12 Architecture invariants

| # | rule | citation | breaks without it |
|---|---|---|---|
| L131 | **`scene_check` is an independent second derivation and shares no code with the planner** — own kinematics call, own capsule geometry, own segment-distance derivation. **Every bug in this list was caught because the judge disagreed with the producer.** | `scene_check.py:1-38` | you have no way to find any of the above |
| L132 | Deliberate duplication is load-bearing: capsule radii, column bands and pen radii are restated in `scene_check` with a test pinning them together. | `scene_check.py:44-73` | a shared constant hides a divergence |
| L133 | Tightening a lower bound must be provably monotone and re-checked bit-identically on shipped programmes. | `DECISIONS.md:4221`, `:4317` | an "improvement" that admits a collision |
| L134 | One seam for the room: every consumer reaches it through `frozen.chain_clearance` / `partner_clearance`. | `exact_room.py:34-36`; `V2_STAGED.md:1392-1402` | five places to change and four that drift |
| L135 | **Gate constants do not move**: `PAIR_MARGIN` 0.050, `SELF_MARGIN` 0.020 / `SELF_PLAN_MARGIN` 0.023, `STATIC_MARGIN` 0.050 / `STATIC_PLAN_MARGIN` 0.063, `MARGIN_GATE` 0.15, `FRAME_FLOOR` 0.063, `CHAIN_CLEAR` / `TIP_CLEAR` 0.020, `TIP_TOL` 0.010. `CALIB_M` stays at 30 mm: "a real uncertainty about where the bases ARE." | `coordination.py:205`; `selfcoll.py:127-141`; `rig_final.py:238`; `DECISIONS.md:4828-4842` | you spend calibration allowance as if it were margin |
| L136 | The inter-arm minimum is set by the gate, not the geometry: v19 lands at 50.32 mm. A base survey is what buys discretionary air back. | `DECISIONS.md:5125-5141`, `:5441-5446` | you read a gate as a measurement |
| L137 | Self-collision was absent and joint limits are not it: 8 of 1 200 comfortable configurations within 10 mm of themselves; invalidated arm 71's park. No calibration term. | `DECISIONS.md:3774-3849`; `selfcoll.py:127` | the arm hits itself |
| L138 | Every shipped Franka sphere model is optimistic on every body (worst +235.2 mm on link0): planner models, never envelopes. Shipped answer is the intersection of both models. Flipping the collision model is a re-certification, not a commit. | `DECISIONS.md:2282-2496`, `:2267-2279`; `link_spheres.py` | a vendored model you trusted |
| L139 | **HARDWARE, decisive.** At h = 0.970 arm 31 draws 1.09 m of the word; at h = 0.940 it draws NOTHING; at 0.850 the word draws 1.23 %. Coverage and hole-freeness point in opposite directions. | `HARDWARE_DAY1.md:82-94`, `:290-318`; `DECISIONS.md:3264-3270` | you optimise the wrong scalar and lose the installation |
| L140 | **HARDWARE.** Arm 71 must never start early: +1.00/+1.25 s skew is a notch bottoming at 35.8 mm. "Two seconds is safe; one is not." Clearance is not monotone in the skew. | `HARDWARE_DAY1.md:762-822` | a cross-process fleet clock must exist before six arms run concurrently |
| L141 | The tool tip is user-specified and never gate-validated, re-specified three times; thinnest margin is paper-chain 20.5 mm against a 20 mm gate — "a touchdown calibration that lowers the tip further eats this first." | `DECISIONS.md:3681`, `:2623-2628`; `HARDWARE_DAY1.md:143-146` | the least safe constant to carry over |
| L142 | `cc_experiment` was evaluated as a collision backend and rejected (no Python bindings, ABI, Drake fork, no licence). Keep as an A/B oracle. | `DECISIONS.md:4018-4029` | you redo the evaluation |
| L143 | `BLOCK = (0.16, 0.00, 1.64, 3.62)` withholds a 0.16 m rim of the canvas from every arm; the single largest source of lost ink in the corpus (6.41 m). Not in the rebuild plan's carry-over list. | `traces.py:87`, `:304`, `:317`, `:474` | you inherit the largest coverage loss without knowing it exists |
| L144 | Constants are copied, not imported, where the copy is the point (a pattern that drifted from the geometry it was certified against is a pattern nobody measured). | `traces.py:77-86` | |

## 2. The pipeline as built (for reference)

Front doors: `staged.run` (`staged.py:2038`), `staged.main` (`:5118`), `staged.run_conducted` (`:5018`); `scripts/draw.py` does NOT use the staged planner (trace → artwork → csail_allocate → csail_schedule); `scripts/day1.py` single-arm hardware.

Per-bucket steps: DP cut (`traces.plan_lines:1001`, `capability:617`) → refusal loop (`resolve_refusals:1759`) → freeze partners as capsules (`freeze_partners:189`) → plan pieces + room gate (`plan_bucket:1137`, `ink_vs_envelope:720`) → cut at room boundary (`split_at_room:909`) → fly-or-defer (`:2797`) → order + chain sheets (`allocate.sequence_arm`, `chain_sheets`) → timeline with routed pen-ups (`writing.arm_program` `writing.py:1949`) → room from realised trajectory (`trajectory_room:645`) → checks with room thrown away (`_check_stage:4199`) → geometric account (`coverage_account:4542`) → typed programme (`programme:4295`).

Stages: A leaders 13/71/2 + followers 17/31/97; B swapped; C go-home then three two-arm row conductors composed serially; D dead-band ink.

## 3. The plan file today

- Schedule `.npz` (`csail_schedule.payload`, `day1._payload`): scalars `fps dt stride n_phases pause_s margin min_clearance …`; per arm `q_<a>` float32 (F,7), `seg_<a>` i8 (−1 = pen up: THIS is the pen state), `u_<a>`, `segpts_<a>`, `segoff_<a>`. Canvas/world metres, z = 0 paper. **No base transform stored**; no version key.
- `program_schema.py`: `SCHEMA_VERSION = 1`; `_from_dict` raises on unknown AND missing keys (`:61-81`), so `staged.programme()` writes its own `STAGED_SCHEMA_VERSION = 2`.
- Pathway CSV `stroke_idx,wp_idx,kind,x_m,y_m,z_m,qx,qy,qz,qw,intensity,q1..q7` in fr3_link0; the executor tests `kind == "draw"` and nothing else; no time column in the contract (`day1` appends `t_s` as 19th). Export refusals: FK re-run on the formatted text < 0.5 mm / 0.5°.
- **The deployed executor IGNORES q1..q7 and t_s** (`day1.py:183-191`): the redundancy resolution is discarded at run time.
- **There is no hash of any plan file**; no calibration file exists; the CSV names neither arm nor rig.
- **Nothing streams**: every write is whole-file; `Backend` protocol (`execute/backends.py:9-19`) has no append verb; `send` moves the whole fleet at one program time by design. Delivery = one scp + one ssh line. Gotcha: `fresh`, not `resume`.
- Replay: `scripts/recheck_timeline.py <npz>` is the real independent re-check; `execute --check` is joint envelope only; `--play` full rehearsal into meshcat.

## 4. Kernel vs orchestration (what survives a rebuild)

| category | modules | survives? |
|---|---|---|
| kinematics & rig model (~14 %) | `frames ik rig_final rig_final6 mounts fleet layout link_spheres envelope metrics system_model` | yes, essentially verbatim |
| collision / gates / certification (~8.5 %) | `coordination scene_check exact_room frozen selfcoll validate` | yes — the irreplaceable half; `scene_check` found every bug above |
| single-arm stroke planning (~10 %) | `planner pwl smooth pacing stroke_api menu tilt lateral` | yes |
| pen-up transit / routing (~6 %) | `paper transit` | yes; the most expensive lessons live here (L1–L7, L52–L56, L120–L123) |
| reachability atlas (~2 %) | `atlas dead` | yes |
| planfile / export / execute (~7.5 %) | `program_schema export/pathway execute/*` | yes |
| picture → strokes (~4 %) | `trace artwork letters` | yes |
| single/multi-arm programme (~19 %) | `allocate writing sequence` | mostly yes: `writing.arm_program / hover_solve / held_pose_ok / hold_candidates / self_pace_beat` and `allocate.sequence_arm / chain_sheets` are not staged-specific |
| staged/conducted, the Pete pattern (~16 %) | `staged traces idle progress` | the genuinely pattern-specific part; but `traces` DP, `idle` policy measurements and `coverage_account / GAP_REASONS` are reusable |
| peripheral (~14 %) | `gui sil viz bench` | `sil` and `bench` kept |

Roughly 48 % of the package is kernel that survives largely as-is; ~34 % is orchestration, half of it staged-specific. `staged.py` is a leaf and discardable; `allocate.py` and `writing.py` hold lessons any single-arm planner needs.

Kernel tests worth carrying: `test_paper test_penup test_transit test_selfcoll test_validate_cone test_gates test_export_pathway test_program_schema test_execute test_layout test_mounts test_system_model test_link_spheres test_sil_*`.
