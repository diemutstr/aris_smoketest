# The drawing server's endpoints

One table; the same as in `docs/modules/server.md`. Base URL: `http://<planning laptop>:8420`.

| method and path | body / query | returns |
|---|---|---|
| `GET /` , `GET /gui` | — | redirect to `/gui/` |
| `GET /gui/{file}` | — | the GUI's static files from `aris/server/gui/` (`index.html` for `/gui/`); 404 if missing |
| `GET /rig` | — | `{arms: {slot: {T_table_base, park_q, calibration, robot (from the site table given to `aris serve --site`, else null)}}, mark_groups: {name: [slots]}, marks: {spot: {xy_m, shared_by}}, drawing_area_m, drawing_area_centre_m, drawing_area_from_maps_m, drawing_area_problem (null or why drawing is refused), canvas_m, pen_in, calibration_files, paper_surface {exists, points, z_min_m, z_max_m, date}, code {commit, dirty, digest}, rig_digest, calibration_digest, calibration, uncalibrated, driver, speed, note}` |
| `GET /arms` | — | `{arms: {slot: ...}, code: {same (true/false/null), line, server, operator_pc, operator_pc_reported_at}}`; per slot with simulated arms `{robot, q, qd, ok, flags, at_park}`, with the robot `{robot (as the operator PC names it, else the site table's), q (null: no reading), reading ("fresh" or why not), at_park (null without reading), reported_at, age_s, source, job}` |
| `GET /jobs` | — | every job of this server run: `[{id, kind, name, state, why, ...}]` |
| `POST /jobs` | body: the drawing JSON (409 `too_close` when two arms stand within the clearance: park first); query `name`, `note`, `air_mm`; or `drawing=<id>` (an uploaded drawing, no body); or `rest_of=<job id>` (no body) | `{id, state, why}`; 400 bad file, 404 unknown drawing id, 409 refused (`{refused, detail}`) |
| `GET /jobs/{id}` | — | `{id, kind, name, state, why, received, elapsed_s, first_motion_s, arms: [{phase, arm, queued, done, current, status, handed_back_m, refused}], drawing?, report}` (report null until finished) |
| `GET /jobs/{id}/report` | — | the finished job's report (`report.json`); 404 while running or unknown |
| `GET /jobs/{id}/events` | — | the job's event rows `[{event, time, arm?, ...}]` |
| `POST /jobs/{id}/stop` | — | `{id, stopping: true}` (a finished job: nothing to do, also 200) |
| `POST /park` | — | `{id, state}` of the park job; 409 refused |
| `POST /arms/{slot}/recover` | — | simulated: `{recovered, why}`; robot: `{queued: command}` |
| `POST /calibrate/{slot}` | — | `{id, state}` of the plane job; 409 refused |
| `POST /touchoff/{slot}` | — | `{id, state}` of the touch-off job; 409 refused |
| `POST /mark` | query `slots=2L,2R` and/or `group=row2\|rows12\|rows23\|all`, `yaw=true` (a row pair's second spot too) | `{id, state}` of the mark job (one meeting per pair of neighbours, the pen tips brought together; the driver's `instruction` rows on its events); its report: `{pairs: [{slots, spots, kind}], meetings: [{pair, spot, phase, gap_m, hovers, q: {a, b}}], solved: {passed, why, slots: {slot: {x_mm, y_mm, yaw_mrad, moved_mm, turned_mrad, yaw: "reference"\|"meetings"\|"nominal"}}, residual_mm, worst_mm, reference, notes, frame, written}}`; 409 refused |
| `POST /crosses` | query `slots=2L,2R` and/or `group=row2` | `{id, state}` of the crosses job, the check after `mark` (each row's L slot draws a cross, R slot a circle, at their shared spots); its report: `{spots: [{spot, xy_m, row, cross, circle}], shapes, instruction}`; 409 refused |
| `POST /grip/{slot}` | JSON `{verb: "home"\|"open"\|"close", width_m?, speed_m_per_s?, force_n?, epsilon_inner_m?, epsilon_outer_m?}` | `{id, state}` of the grip job; its report: `{slot, verb, params, width_before_m, width_after_m, grasped, nothing_to_do}`; 409 refused (no reading, a job running, bad verb, unknown slot) |
| `POST /drawings` | multipart: `file` (.json or .svg), `width` (m, needed for .svg: else 400 "an SVG needs its width on the table, in metres"), `at` ("x,y" m, .svg; default the area's centre) | `{id, name, kind, stored_at, lines, points, bbox_m, width_m, at_m}`; stored under `out/drawings/`; 400 refused |
| `GET /drawings` | — | every stored drawing's `{id, name, kind, lines, points, ...}`, oldest first |
| `GET /calibration` | — | `{arms, files: [...], status: {slot: ...}}` |
| `GET /calibration/{slot}` | — | that slot's calibration file; 404 if none |
| `GET /operator` | — | `{pending: [commands], last_seen, stacks, last_rows, ...}` |
| `POST /operator/report` | — | asks the operator PC to report; the command queued |
| `GET /jobs/{id}/header` | — | operator PC: the job header (`job.json`) |
| `GET /jobs/{id}/phases?offset=B` | — | operator PC: the phase list (ndjson) from byte B, held open until its end line |
| `GET /jobs/{id}/queues/{phase}/{slot}?offset=B` | — | operator PC: the queue file from byte B, held open until its end marker |
| `POST /jobs/{id}/events` | `{source, rows: [{seq, event, ...}]}` | operator PC: `{accepted, next_seq, stop}` |
| `GET /operator/next?wait=30&code=<json>` | — | operator PC: the oldest unacknowledged command (`run`, `recover`, `report`) or 204 |
| `POST /operator/ack` | `{id, code?}` | operator PC: `{acknowledged}` |
| `POST /operator/rows` | `{source, rows}` | operator PC: `{accepted}` (rows outside any job; positions updated) |

Every refusal is `{"refused": reason, "detail": text}` with status 400 (bad input) or 409 (cannot
run now). Job kinds: `draw`, `park`, `calibrate`, `touchoff`, `mark`, `crosses`, `grip`. Job states:
`received`, `fitted` (drawings), `planning`, `drawing` or `moving`, then `done`, `failed` or
`stopped`.
