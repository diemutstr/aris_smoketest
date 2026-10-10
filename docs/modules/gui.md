# gui — the drawing arms page

**Job.** One web page for Diemut that looks like the main window of her old PyQt GUI
(`franka_control_gui.py`, `FrankaControlGUI.init_ui`: "ARIS_KINDT - FR3 CONTROL", 1400×800)
and drives this system. Every control is one request to the drawing server; the server's
answer (success, or the refusal word for word) is written to STATUS, the right column. Nothing
asks "are you sure"; the server decides what is safe and the page shows what it said. Old
controls that have no counterpart here stay in place, greyed, with a one-line "not part of this
system", so the layout is the one she knows.

Files: `aris/server/gui/` — `index.html` (the window), `app.js` (polling, wiring, reports in
plain words, the popups), `style.css` (the old stylesheets' colours, fonts, sizes). Plain
HTML/JS/CSS: no build step, no framework, nothing from the internet. The server serves the
folder (`GET /gui/...`; `GET /` and `GET /gui` redirect to `/gui/`). Endpoints:
`aris/server/API.md`.

## Opening it

Start the server as usual (`docs/FOR_THE_ARTIST.md`, section 4), then in a browser:
`http://localhost:8420/gui` on the planning laptop, or `http://<planning laptop>:8420/gui` from
another computer on the robot network (the server started with `--host 0.0.0.0`). Several
browsers may have it open at once; reloading loses nothing.

## The window

Three columns as the old QSplitter (400 | 820 | 173, the middle one scrolls) and the status bar:

- **Left, the dialogue panel:** RESTART GUI · LOG FILES · SSH CHECKS on top; AI VISION and
  AI TEXT / SPEECH below (greyed).
- **Middle, the controls:** A R I S _ K I N D T, the discovery row (greyed),
  A _ K _ M O D U L E S, then the two section bars with their submenus —
  **⇄ MULTITASK / OPERATION CONTROL** (ARM COUNT, MATERIAL, verify arms, CALL OPERATOR, the
  ROBOTS grid, the INDIVIDUAL / SET tabs with GRIPPER WIDTH / FORCE) and **AESTHETIC
  GENERATOR + CODES** (Ae_G_ IMAGE (upload), the generator sections, P_ DRAWING CONTROL,
  OPERATION / ORCHESTRATOR, ACTIVE FILE INFO, WAYPOINT PEEK, SVG (ADVANCED), RASTER (ADVANCED))
  — and EMERGENCY at the bottom. The bars and the ▸ sections fold as before; the page
  remembers which are open.
- **Right, STATUS:** every answer, newest at the bottom; refusals red; a job's report in
  plain words when it ends; a MARK job's instructions to the person as they come.
- **Status bar:** SYSTEM: READY (or NO SERVER), operator PC code different, UNCALIBRATED,
  no drawing area, the running job.

The selected arm is the one in ARM COUNT; the arm buttons (gripper, MATERIAL, CALL OPERATOR,
Z TOUCH, MEASURE SURFACE) act on it. While a job runs the buttons that start a job are greyed
(the server allows one job at a time) and STATUS says which job runs; the stop buttons are lit
only then.

## Old control → what it does now

| old control | now |
|---|---|
| ARM COUNT (1–6) | picks the arm: one entry per slot with its robot number, "2L  #71" (the number only when `/arms` or `/rig` gives it) |
| ROBOTS grid (old SUPERVISION panel) | live: per slot the light (green still / amber moving / red no reading or fault), state, reading age, calibration base / pen |
| MATERIAL | the pen in the selected arm's holder: `GET /pens`, `POST /pens/{slot}` |
| verify arms | `GET /arms`: each arm's joints, at park or not, and the code line (same / DIFFERENT) into STATUS |
| CALL OPERATOR | `POST /arms/{slot}/recover` for the selected arm |
| START POS, START SET POS | `POST /park` (every arm to its park) |
| SSH CHECKS (both) | a popup of checks, PASS / FAIL: server answers, code same, each arm's reading, drawing area, calibration |
| GRIPPER WIDTH / FORCE, HOME GRIP / OPEN / CLOSE (INDIVIDUAL) | `POST /grip/{slot}` `{verb, width_m, force_n}` for the selected arm |
| … SET GRIPPERS (SET) | the same for every arm ticked in SET, one after the other (one job at a time) |
| KILL MOTION, KILL SET MOTION, KILL ALL, • CANCEL SVG, EMERGENCY | `POST /jobs/{id}/stop` of the running job (EMERGENCY also reminds that the arms' own stop is the one in her hand) |
| • SELECT SVG (both) | pick a `.svg` (with WIDTH, optional POSITION x,y) or a drawing `.json`: `POST /drawings`; it becomes the IMAGE FILE |
| IMAGE FILE (SVG ADVANCED) | the stored drawings, newest first (`GET /drawings`, ↻ reads them again); SVG CONFIG (YAML) shows the picked one |
| START DRAWING, • START SVG | `POST /jobs?drawing=<IMAGE FILE>&note=…&air_mm=…` (NOTE and IN THE AIR mm rows in SVG ADVANCED) |
| RESUME DRAWING, • RESUME SVG | `POST /jobs?rest_of=<id>` of the newest drawing job that stopped or failed |
| Z TOUCH | `POST /touchoff/{slot}`: measures the selected arm's pen length |
| MEASURE SURFACE | `POST /calibrate/{slot}`: the paper under the selected arm |
| MARK, MARK + YAW, CROSSES (new row under Z TOUCH) | `POST /mark?group=…` (`&yaw=true`), `POST /crosses?group=…`, the group from `/rig` `mark_groups` |
| PEN HEIGHT mm, force label | the press of the selected arm's pen (`/rig` `pens_in`), read-only — config is never edited from the page |
| PAPER WIDTH / LENGTH, CENTER X / Y | the drawing area and its centre (`/rig`), read-only |
| SYSTEM STATUS | popup: `/rig` + `/arms` + the code line, per arm joints, flags, calibration, pen |
| ACTIVE FILE INFO | the running (or last) job: arms, file, motions done / queued and strokes per arm, state and phase; REFRESH ↻ reads again; FULL DETAILS ▸ shows the job header (`/jobs/{id}/header`) |
| STROKE line, WAYPOINT PEEK | motions done / queued; per phase and arm queued, done, the current motion |
| VIEW LIVE PATH | popup: the running job's events (`/jobs/{id}/events`), following as they come |
| LOG FILES | popup: the jobs of this server run, each with REPORT (plain words), EVENTS and, for a stopped or failed drawing, RESUME DRAWING |
| greyed (in place, "not part of this system") | AI VISION, AI TEXT / SPEECH, discovery, LEGACY / Z-TOUCH / RTffLL modes, MULTITASK, START POS INV, TOUCHDOWN ↓, SELFTEST, FULL RECOVERY, MOVEIT, image generation (Ae_G_, PHOTO → SVG, X-RAY, …), editors, drawing / motion codes, CONFIG FILES, RELAUNCH CLONE / X, SUPERVISION, ORCHESTRATOR, TRANSITION, CLEAR ALL (WIPE!), RESTART GUI, AUTO-RECOVERY, EXPORT PATHWAY CSV, PAUSE / RAISE, raster drawing |

## Adding a control

1. In `index.html`, put the button where the old one was, with the class of its look
   (`qsub` submenu button, `qbar` section bar, `hdr` header, …) and an `id`. Add `need-idle`
   if it starts a job (greyed while one runs), `data-stop="LABEL"` if it stops the running job.
2. In `app.js`, in `wire()`:
   ```js
   $("btn-mything").addEventListener("click", () =>
     act("MY THING", "POST", "/my/endpoint", { some: "body" }));
   ```
   `act(label, method, path, body, opts)` sends the request and writes the answer to STATUS;
   `opts.ok(data)` turns a success into words (default "started job <id>"). Never catch an
   error without showing it.
3. A live readout goes into a `render…()` function (called after every poll); a view in its
   own window is `openPopup(title, viewFn)`.

A new job kind gets its plain-words report in `plainReport()` and a name in `KIND_WORDS`.

## Checked

In headless Chrome against `aris serve --config config --driver sim --uncalibrated --site
site/aris_2026-10.json` (six arms), 2026-10-10: ARM COUNT lists 1L #31, 1R #2, 2L #71,
2R #97, 3L #17, 3R #13 (as that server's site table says); MARK groups row2, rows12, rows23,
all; SELECT SVG upload, verify arms, gripper OPEN, START SVG in the air, the busy refusal shown
verbatim, KILL MOTION, RESUME DRAWING, EMERGENCY, LOG FILES with a report, SYSTEM STATUS,
MATERIAL, Z TOUCH (refused by the job itself: no paper measured yet); no confirmation dialog.
Not yet seen against the robot driver (`--driver robot`).
