# gui — the drawing arms page

**Job.** One web page with big buttons for Diemut, in place of the desktop GUI she lost when
the stack was replaced. It does nothing the `aris` commands cannot do: every button is one
request to the drawing server, and the server's answer (success, or the refusal word for word)
is shown in the status line at the top. The page has no logic of its own about what is safe;
the server decides, the page shows what it said.

Files: `aris/server/gui/` — `index.html` (the panels), `app.js` (polling, buttons, the report
in plain words), `style.css`. Plain HTML, JavaScript and CSS: no build step, no framework, no
internet (nothing is loaded from outside the server). The server serves the folder as static
files (`server.py`, `GET /gui/...`; `GET /` and `GET /gui` redirect to `/gui/`).

## Opening it

Start the server as usual (`docs/FOR_THE_ARTIST.md`, section 4), then open in a browser:

- on the planning laptop itself: `http://localhost:8420/gui`
- from another computer on the robot network: `http://<planning laptop>:8420/gui` (the server
  must have been started with `--host 0.0.0.0`, as in the daily start-up line)

The page can be open in several browsers at once; it only reads, except when a button is pressed.
Reloading the page loses nothing (the job history lives in the server). The `aris` commands keep
working beside it.

## The look

The look of her old PyQt GUI: black background, panels #1A1A1A / #0A0A0A, white text in
Courier New, UPPERCASE section headers, square buttons (#1A1A1A, white bold text, #333 on
hover), primary actions yellow (#FFD700, black text: DRAW, MARK), go actions green (#00CC00,
black text: RESUME DRAWING, UPLOAD, CROSSES), STOP red. No rounded corners, no shadows,
dense. All of it is in `style.css`.

## The sections, top to bottom

**Top.** "server answers" (green) or "NO SERVER" (red). Red banners when the server does not
answer or the robot PC runs different code than the laptop (jobs are refused until both are
updated); yellow when the server runs `--uncalibrated` or cannot draw (no drawing area).

**STATUS.** The answer to the last button: yellow while asking, green for success ("started job
…"), red for a refusal (`refused: busy: job … is planning; one job at a time`) or an error; the
driver's instructions to the person during a MARK job also appear here. Under it the log of the
last 50 answers with the time, and, while a job runs, which one. Nothing is hidden or shortened.

**ARMS.** One card per slot the rig has (`/rig`; six with `config`, two with
`config/two_arms`), labelled slot and robot number, "2L · 71" (the number only when `/arms` or
`/rig` gives it, from the site table the server was started with, `--site`; otherwise the slot
alone). The light: green = joints read and the arm still (the text says whether it is at its
park), amber = moving, red = no reading (with the reason) or a fault (e.g. after STOP, until
RECOVER). Then how old the joint reading is, the calibration state (base, pen), MATERIAL (the
pen in that arm's holder: a choice from `GET /pens` `table`, set with `POST /pens/{slot}`),
GRIPPER HOME / OPEN / CLOSE (`POST /grip/{slot}`, a short job) with the width it was left at,
and RECOVER (`POST /arms/{slot}/recover`).

**DRAWING CONTROL.** STOP (`POST /jobs/{id}/stop`, lit only while a job runs), PARK
(`POST /park`), DRAW (the image picked in SELECT IMAGE: `POST /jobs?drawing=<id>&note=…&air_mm=…`),
RESUME DRAWING (what the newest drawing job left, when it stopped or failed:
`POST /jobs?rest_of=<id>`). A NOTE for the report, and IN THE AIR (draws the given millimetres
above the paper, nothing touches).

**SELECT IMAGE.** The stored images (`GET /drawings`), newest first, as a list; "↻ NEWEST
FIRST" reads the list again at once. ADD AN IMAGE: a `.json` drawing, or an `.svg` with its
WIDTH in metres and, if wanted, its POSITION "x, y" in metres; UPLOAD (`POST /drawings`) picks
the new image at once.

**LIVE DRAWING OUTPUT.** The running job, or the last one: kind, state and reason, phase, lines
in the drawing, motions done of those planned so far, and per arm the strokes drawn and what it
is doing now (`/jobs/{id}` and `/jobs/{id}/events`, once a second). A drawing that stopped or
failed gets a RESUME DRAWING button here too.

**CALIBRATION.** Per mark group of the rig (`/rig` `mark_groups`: row2, rows12, rows23, all —
"all" last): MARK (`POST /mark?group=…`), MARK + YAW (`&yaw=true`), CROSSES
(`POST /crosses?group=…`). Per arm: TOUCH-OFF and CALIBRATE (`POST /touchoff/{slot}`,
`POST /calibrate/{slot}`). Each kind with one line of what it does, in Diemut's words. The
saved calibration files are listed at the bottom.

**RUN GALLERY.** The jobs of this server run, newest first, each with REPORT (and RESUME
DRAWING for a stopped or failed drawing); beside the list the chosen job's report in plain
words (`/jobs/{id}/report`): what was drawn, what was left over and why, the gripper width,
the calibration results. The newest finished job's report is shown by itself.

Every button acts on one click, without a confirmation dialog, and shows the server's answer.
While a job runs, the buttons that start a job are greyed out and STATUS says which job runs
(the server allows one job at a time; the gripper and the material are jobs or refused too).
RECOVER and STOP stay usable.

## Adding a button

1. In `index.html`, put a `<button id="btn-mything" class="btn">MY THING</button>` in the section
   it belongs to (`btn-primary` / `btn-yellow` for a main action, `btn-go` for a go action), with a `<p class="help">` line saying in plain words what it does. Add the
   class `need-idle` if it starts a job (it is then greyed out while a job runs).
2. In `app.js`, in `wire()`:
   ```js
   $("btn-mything").addEventListener("click", () =>
     act("MY THING", "POST", "/my/endpoint", { some: "body" }));
   ```
   `act(label, method, path, body, opts)` sends the request and writes the answer to the status
   line; `opts.ok(data)` turns a success answer into words (default: "started job <id>"). Never catch an error
   without showing it: `act` already shows refusals and failures.
3. Buttons made per slot are built once in `buildCards()` / `buildCalib()` (not on every poll,
   so a click is never lost to a rebuild); their texts are updated in `updateCards()`.

A new job kind gets its plain-words report in `plainReport()` and a name in `KIND_WORDS`.
The endpoints themselves are listed in `aris/server/API.md`.

## Checked

In headless Chrome against `aris serve --config config/two_arms --driver sim --uncalibrated`
(2026-10-07) and against the six-arm rig, `aris serve --config config --driver sim
--uncalibrated --site site/aris_2026-10.json` (2026-10-09): six cards 1L · 31, 1R · 2,
2L · 71, 2R · 97, 3L · 13, 3R · 17; MARK / MARK + YAW / CROSSES for row2, rows12, rows23, all;
↻ NEWEST FIRST; DRAW in the air; the busy refusal shown verbatim; STOP; RESUME DRAWING; a
MATERIAL change; the reports; no confirmation dialog anywhere. Not yet seen against the robot
driver (`--driver robot`): joint ages and "no reading".
