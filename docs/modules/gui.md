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

## The panels

**Top.** "server answers" (green) or "NO SERVER" (red). Red banners when the server does not
answer, when the robot PC runs different code than the laptop (jobs are refused until both are
updated), amber when the server runs `--uncalibrated` or cannot draw (no drawing area).

**Status line.** The answer to the last button, kept until the next one: blue while asking,
green for success ("started job …"), red for a refusal (`refused: busy: job … is planning; one
job at a time`) or an error. "Earlier answers" opens the last 50, with the time. Nothing is
ever hidden or shortened.

**Arms.** One card per slot the rig has (`/rig`), labelled with the slot and the robot number
("2L", "robot 97"; the number only when `/arms` or `/rig` gives it, otherwise the slot
alone, as with simulated arms). The light:
green = joints read and the arm still (the text says whether it is at its park), amber =
moving, red = no reading (with the reason) or a fault. Below: how old the joint reading is, the
calibration state (base, pen), and the gripper buttons HOME / OPEN / CLOSE (`POST /grip/{slot}`,
a short job like any other) with the width the gripper was left at. RECOVER
(`POST /arms/{slot}/recover`).

**Job.** The running job, or the last one: what kind, its state and reason, the phase, the
lines in the drawing, motions done of those planned so far, and per arm the strokes drawn and
what it is doing now (from `/jobs/{id}` and `/jobs/{id}/events`, once a second). STOP
(`POST /jobs/{id}/stop`) is lit only while a job runs. PARK (`POST /park`).
Under it the last finished job's report in plain words (`/jobs/{id}/report`): what was drawn,
what was left over and why, the gripper width, the calibration results. A drawing job that
stopped or failed gets a DRAW THE REST button (`POST /jobs?rest_of=<id>`). The list of earlier
jobs of this server run has a "report" button on each.

**Drawing.** Pick a drawing that was added before (`GET /drawings`), or add one: a `.json`
drawing, or an `.svg` with its width in metres and, if wanted, the centre "x, y" in metres
(`POST /drawings`; the new drawing is picked at once). An optional note goes into the report.
"Fly it in the air" draws it the given millimetres above the paper (`air_mm`). DRAW
(`POST /jobs?drawing=<id>&note=…&air_mm=…`).

**Calibration.** MARK (all) (`POST /mark`), and one MARK button per group of the rig — the
rig's mark groups when `/rig` lists `mark_groups`, else one per row of the frame with two or
more slots (`POST /mark?slots=…`), left out when it is every slot anyway. TOUCH-OFF and
CALIBRATE per slot (`POST /touchoff/{slot}`, `POST /calibrate/{slot}`). Each with one line of
what it does, in the words of `docs/FOR_THE_ARTIST.md`. The saved calibration files are listed
under them.

Every button acts on one click, without a confirmation dialog, and shows the server's answer.
While a job runs, the buttons that
start a job are greyed out and a note says which job runs (the server allows one job at a
time); the gripper buttons are jobs too. RECOVER and STOP stay usable.

## Adding a button

1. In `index.html`, put a `<button id="btn-mything" class="btn">MY THING</button>` in the panel
   it belongs to, with a `<p class="help">` line saying in plain words what it does. Add the
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

Against the simulated server (`aris serve --config config/two_arms --driver sim
--uncalibrated`) in headless Chrome, 2026-10-07: cards, gripper width, upload, DRAW in the air,
the busy refusal shown verbatim, a bad gripper verb's refusal, STOP, the stopped job's report
and DRAW THE REST button, live progress during a drawing. Not yet seen against the robot
driver (`--driver robot`): the joint ages, "no reading" and the robot numbers from `/arms`.
