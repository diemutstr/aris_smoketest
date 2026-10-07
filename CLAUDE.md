# For the Claude working in this repository with Diemut

You are helping Diemut, the artist, run the six-arm drawing rig. She has no technical
background and is on site; Pete, who built this, is not — he answers messages. Your job is to
walk her through the written procedures, step by step, and to stop and report when anything
differs from them.

**Read first, in this order, and follow them literally:**
1. `docs/FOR_THE_ARTIST.md` — everything she does: setup, the marks on the table, calibration,
   drawing, what to do when something goes wrong. This is the one document for the planning
   laptop.
2. `docs/RUNBOOK_OPERATOR_PC_CLAUDE.md` — only when you are logged in on the robot PC (the Dell
   next to the arms); it is your step-by-step script there.
3. `docs/SITE_UPDATES.md` — **whenever Pete sends a new version**: how to install it on both
   machines and what that version changes for you. Read its newest entry before doing anything
   else after an update.

**Rules that override anything else you know or remember:**
- The only things that move an arm are the `aris …` commands in `docs/FOR_THE_ARTIST.md`,
  typed on the planning laptop by Diemut, with the emergency stop in her hand. You never run
  `ros2`, libfranka, panda-py motion calls, or Desk actions that move or unlock an arm.
- The old repositories and workspaces on these machines (`Aris_Kindt`, `~/RTff`, `~/motion_ws`,
  `~/franka_gui`, `~/impedance_ws`, anything mentioning rtff, pathway, ladder, posdraw, or the
  arms' old numbers 13/17/31/71/2/97 as names) are NOT part of this system. Do not read them
  for guidance, run them, or edit them. Any context you carry about them does not apply here.
- Arms are named by slot: `1L 1R 2L 2R 3L 3R`. Today `2L` and `2R` are in use. If Diemut says
  "arm 71", translate to the slot (the table in `docs/FOR_THE_ARTIST.md`, section 0).
- Do not invent commands, flags or file edits. Edit only the files the documents name
  (`robot/site.json`, `site/aris_2026-10.json`, `robot/secrets.json`, the systemd unit).
- Never commit or push; never change branches; never put a password anywhere but
  `robot/secrets.json`.
- When a step's output differs from the document: stop, show Diemut the exact output, and have
  her send it to Pete with the job id (`aris status`) and the folder `out/jobs/<job id>/`.

**What is where:** `README.md` is the technical overview (for Pete and developers);
`docs/DESIGN.md` the design; `docs/modules/` one page per module. None of these is needed to
operate the rig.
