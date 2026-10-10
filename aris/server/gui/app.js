// The old PyQt main window, wired to the drawing server (GET /gui).  Plain JavaScript, no
// framework, no build step, no internet: every number comes from the server's endpoints
// (aris/server/API.md), polled once a second.  Every button's answer goes to STATUS (the right
// column) word for word; nothing asks "are you sure".  docs/modules/gui.md has the table of
// old control -> what it does now, and how to add one.
"use strict";

const POLL_MS = 1000;            // /arms, /jobs and the current job
const SLOW_EVERY = 5;            // /rig, /drawings, /pens: every 5th poll
const FINAL = ["done", "stopped", "failed"];
const KIND_WORDS = {
  draw: "DRAWING", park: "PARK", grip: "GRIPPER", calibrate: "MEASURE SURFACE",
  touchoff: "Z TOUCH", mark: "MARK", crosses: "CROSSES",
};
const MOTION_WORDS = { draw: "drawing a stroke", free: "moving (pen up)", touch: "touching the paper",
                       guide: "waiting for the person" };

const S = {
  online: null, onlineWhy: "", rig: null, rigWhy: "", arms: null, code: null, pens: null,
  jobs: [], view: null, events: [], eventsFor: null, drawings: [],
  grip: {}, gripSeen: {}, lastQ: {}, moving: {}, tick: 0, busyPolling: false,
  keys: {}, saidCount: 0, autoReportId: null, popup: null, header: null, headerFor: null,
};

// --------------------------------------------------------------------------- helpers

const $ = (id) => document.getElementById(id);
function el(tag, attrs, ...kids) {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs || {})) {
    if (k === "class") e.className = v;
    else if (k === "onclick") e.addEventListener("click", v);
    else if (k === "text") e.textContent = v;
    else e.setAttribute(k, v);
  }
  for (const c of kids) e.append(c);
  return e;
}
const mm = (m, d = 1) => (m == null ? "?" : (1000 * Number(m)).toFixed(d) + " mm");
const metres = (m) => (m == null ? "?" : Number(m).toFixed(2) + " m");
const clock = (t) => new Date(t * 1000).toLocaleTimeString();
const once = (key, value) => { if (S.keys[key] === value) return false; S.keys[key] = value; return true; };

function slots() {
  if (S.rig && S.rig.arms) return Object.keys(S.rig.arms);
  return S.arms ? Object.keys(S.arms) : [];
}
// The robot number only when the server gives it (/arms, else /rig).
function robotOf(slot) {
  const a = S.arms && S.arms[slot], r = S.rig && S.rig.arms && S.rig.arms[slot];
  const said = (a && a.robot) || (r && r.robot) || "";
  const m = String(said).match(/(\d+)\s*$/);
  return m ? m[1] : (said ? String(said) : null);
}
const label = (slot) => { const n = robotOf(slot); return n ? `${slot}  #${n}` : slot; };
const running = () => S.jobs.find((j) => !FINAL.includes(j.state)) || null;
const selected = () => $("arm-select").value || slots()[0];

// --------------------------------------------------------------------------- the server

async function call(method, path, body, form) {
  const opt = { method, headers: {} };
  if (form) opt.body = form;
  else if (body !== undefined) { opt.body = JSON.stringify(body); opt.headers["Content-Type"] = "application/json"; }
  let r;
  try { r = await fetch(path, opt); } catch (e) {
    return { ok: false, status: 0, data: null, text: `the drawing server does not answer (${e.message})` };
  }
  const text = await r.text();
  let data = null;
  try { data = text ? JSON.parse(text) : null; } catch (e) { /* not JSON: the text is shown */ }
  return { ok: r.ok, status: r.status, data, text };
}

// What the server said, as close to verbatim as it can be shown.
function answerText(res) {
  if (res.status === 0) return res.text;
  const d = res.data;
  if (d && typeof d === "object" && "refused" in d) return `refused: ${d.refused}: ${d.detail}`;
  if (d && typeof d === "object" && "detail" in d)
    return `HTTP ${res.status}: ` + (typeof d.detail === "string" ? d.detail : JSON.stringify(d.detail));
  return `HTTP ${res.status}: ${res.text || "(empty answer)"}`;
}

// --------------------------------------------------------------------------- STATUS

function say(text, kind = "ok") {
  const box = $("status-display");
  const atEnd = box.scrollHeight - box.scrollTop - box.clientHeight < 30;
  box.append(el("div", { class: kind }, el("span", { class: "t", text: new Date().toLocaleTimeString() + " " }),
                document.createTextNode(text)));
  while (box.children.length > 400) box.firstChild.remove();
  if (atEnd) box.scrollTop = box.scrollHeight;
}

// One button press: ask the server, show its answer (success or refusal) in STATUS.
async function act(what, method, path, body, opts = {}) {
  say(`${what}: asking the server…`, "wait");
  const res = await call(method, path, body, opts.form);
  if (res.ok) {
    const d = res.data || {};
    say(`${what}: ${opts.ok ? opts.ok(d) : d.id ? `started job ${d.id}` : res.text || "done"}`, "ok");
  } else {
    say(`${what}: ${answerText(res)}`, "bad");
  }
  poll(true);
  return res;
}

// A job and its end: for the SET gripper buttons, one slot after the other.
async function untilDone(jid) {
  for (;;) {
    const r = await call("GET", `/jobs/${encodeURIComponent(jid)}`);
    if (!r.ok) return r;
    if (FINAL.includes(r.data.state)) return r;
    await new Promise((ok) => setTimeout(ok, 500));
  }
}

// --------------------------------------------------------------------------- polling

async function poll(now) {
  if (S.busyPolling && !now) return;
  S.busyPolling = true;
  try {
    const slow = S.tick++ % SLOW_EVERY === 0 || !S.rig;
    const [arms, jobs, rig, drawings, pens] = await Promise.all([
      call("GET", "/arms"), call("GET", "/jobs"),
      slow ? call("GET", "/rig") : null, slow ? call("GET", "/drawings") : null,
      slow || now ? call("GET", "/pens") : null]);
    S.online = arms.status !== 0;
    S.onlineWhy = arms.ok ? "" : answerText(arms);
    if (arms.ok) { trackMotion(arms.data.arms); S.arms = arms.data.arms; S.code = arms.data.code; }
    if (jobs.ok) S.jobs = jobs.data || [];
    if (rig) { if (rig.ok) { S.rig = rig.data; S.rigWhy = ""; } else S.rigWhy = answerText(rig); }
    if (drawings && drawings.ok) S.drawings = drawings.data || [];
    if (pens && pens.ok) S.pens = pens.data;
    await pollJob();
    await pollGrips();
  } finally {
    S.busyPolling = false;
  }
  render();
}

function trackMotion(arms) {
  for (const [slot, a] of Object.entries(arms || {})) {
    const q = a.q, last = S.lastQ[slot];
    let moved = false;
    if (q && last && q.length === last.length) moved = q.some((v, i) => Math.abs(v - last[i]) > 0.002);
    if (a.qd) moved = moved || a.qd.some((v) => Math.abs(v) > 0.01);
    S.moving[slot] = moved;
    S.lastQ[slot] = q;
  }
}

// The current job (the running one, else the newest), its events, its header.
async function pollJob() {
  const run = running();
  const cur = run || S.jobs[S.jobs.length - 1];
  if (!cur) { S.view = null; S.events = []; return; }
  const v = await call("GET", `/jobs/${encodeURIComponent(cur.id)}`);
  S.view = v.ok ? v.data : { id: cur.id, kind: cur.kind, state: cur.state, why: cur.why, error: answerText(v) };
  if (run || S.eventsFor !== cur.id) {
    const ev = await call("GET", `/jobs/${encodeURIComponent(cur.id)}/events`);
    S.events = ev.ok ? ev.data || [] : [];
    if (S.eventsFor !== cur.id) S.saidCount = 0;
    S.eventsFor = cur.id;
    // the driver's instructions to the person (mark, crosses): each one once, in STATUS
    const said = S.events.filter((r) => r.event === "instruction");
    for (const r of said.slice(S.saidCount)) say(`${r.arm ? label(r.arm) + ": " : ""}${r.text || r.why || ""}`, "wait");
    S.saidCount = said.length;
  }
  if (S.headerFor !== cur.id) {
    const h = await call("GET", `/jobs/${encodeURIComponent(cur.id)}/header`);
    S.header = h.ok ? h.data : { error: answerText(h) };
    S.headerFor = cur.id;
  }
  // a job that just ended: its report in plain words, once, in STATUS
  const done = [...S.jobs].reverse().find((j) => FINAL.includes(j.state));
  if (done && S.autoReportId !== done.id) {
    const first = S.autoReportId === null;
    S.autoReportId = done.id;
    if (!first) {
      const r = await call("GET", `/jobs/${encodeURIComponent(done.id)}/report`);
      say(`job ${done.id} ended:\n` + (r.ok ? plainReport(r.data) : answerText(r)), done.state === "done" ? "ok" : "bad");
    }
  }
}

// The width each gripper was left at: read once from each finished grip job's report.
async function pollGrips() {
  for (const j of S.jobs) {
    if (j.kind !== "grip" || !FINAL.includes(j.state) || S.gripSeen[j.id]) continue;
    S.gripSeen[j.id] = true;
    const r = await call("GET", `/jobs/${encodeURIComponent(j.id)}/report`);
    if (r.ok && r.data) S.grip[r.data.slot] = r.data;
  }
}

// --------------------------------------------------------------------------- what a job did

function progress(events) {
  const kindOf = {}, strokes = {}, current = {};
  let phase = null;
  for (const e of events) {
    if (e.event === "phase started") phase = e.phase;
    if (e.event === "phase done" && e.phase === phase) phase = phase + " (done)";
    if (e.arm == null) continue;
    const key = `${e.phase}|${e.arm}|${e.index}`;
    if (e.event === "motion started") { kindOf[key] = e.kind; current[e.arm] = e.kind; }
    if (e.event === "motion done") {
      if (kindOf[key] === "draw") strokes[e.arm] = (strokes[e.arm] || 0) + 1;
      current[e.arm] = null;
    }
    if (["finished", "failed", "stopped", "holding"].includes(e.event)) current[e.arm] = e.event;
  }
  return { phase, strokes, current };
}

function plainReport(rep) {
  if (!rep) return "no report";
  if (rep.error) return rep.error;
  const out = [], st = rep.state;
  out.push(st === "done" ? "Finished." : st === "stopped" ? `Stopped${rep.why ? ": " + rep.why : "."}`
           : st === "failed" ? `FAILED: ${rep.why || "(no reason given)"}` : `State: ${st}${rep.why ? " (" + rep.why + ")" : ""}`);
  if (rep.air_mm) out.push(`Air run: flown ${rep.air_mm} mm above the paper, nothing touched.`);
  if (rep.note) out.push(`Note: ${rep.note}`);
  if (rep.kind === "draw") {
    if (rep.name) out.push(`Drawing: ${rep.name}`);
    if (rep.drawn_m != null || rep.length_m != null)
      out.push(`Drew ${metres(rep.drawn_m || 0)} of ${metres(rep.length_m)} of line; ${metres(rep.left_m || 0)} left over.`);
    for (const [why, m] of Object.entries(rep.left_by_reason || {})) out.push(`   left over because ${why}: ${metres(m)}`);
    const sc = rep.drawing && rep.drawing.scale;
    if (sc != null && sc < 0.999) out.push(`The drawing was made smaller (${(100 * sc).toFixed(0)} %) to fit the drawing area.`);
    if (rep.planner_refusal) out.push(`The planner refused: ${rep.planner_refusal}`);
    if (rep.planner_error) out.push(`Planner error: ${rep.planner_error}`);
    if (rep.account_error) out.push(`ACCOUNT ERROR (send to Pete): ${rep.account_error}`);
    if (rep.checker && rep.checker.all_queued_checked === false) out.push("WARNING: not every motion carried a passing check (send to Pete).");
    if (rep.left_m > 0.001 && (st === "stopped" || st === "failed")) out.push("What was left can be drawn with RESUME DRAWING.");
  } else if (rep.kind === "park") {
    for (const [a, r] of Object.entries(rep.arms || {})) out.push(`${label(a)}: ${r.result}`);
  } else if (rep.kind === "grip") {
    out.push(`${label(rep.slot)} gripper ${rep.verb}: width ${mm(rep.width_before_m)} → ${mm(rep.width_after_m)}` +
             (rep.grasped == null || rep.verb !== "close" ? "" : rep.grasped ? " (holding something)" : " (holding nothing)"));
  } else if (rep.kind === "calibrate") {
    out.push(`${label(rep.arm)}: ${rep.points || 0} points touched (${rep.contacts || 0} met the paper), ${(rep.dropped || []).length} out of reach.`);
    if (rep.missed && rep.missed.length) out.push(`No paper found at: ${JSON.stringify(rep.missed)}`);
    const f = rep.fit;
    if (f) out.push(f.passed ? `Paper measured: tilt ${f.tilt_deg}°, height change ${f.height_change_mm} mm, unevenness ${f.rms_mm} mm.`
                             : `Paper measurement FAILED: ${f.why}`);
    if (rep.written) out.push(`Saved: ${rep.written}`);
  } else if (rep.kind === "touchoff") {
    const t = rep.touchoff;
    if (t) out.push(t.passed ? `${label(rep.arm)}: pen length measured.` : `${label(rep.arm)}: pen measurement FAILED: ${t.why}`);
    if (rep.written) out.push(`Saved: ${rep.written}`);
  } else if (rep.kind === "mark") {
    for (const m of rep.meetings || [])
      out.push(`${m.pair.map(label).join(" and ")} at ${m.spot}: ` + (Object.keys(m.q || {}).length === 2 ? "tips met, registered." : "NOT registered."));
    const s = rep.solved;
    if (s && !s.passed) out.push(`not solved: ${s.why}`);
    else if (s) {
      for (const [sl, e] of Object.entries(s.slots || {}))
        out.push(`${label(sl)}: x ${e.x_mm} mm, y ${e.y_mm} mm, yaw ${e.yaw_mrad} mrad (moved ${e.moved_mm} mm, turned ${e.turned_mrad} mrad)` +
                 (e.yaw === "nominal" ? "; yaw kept nominal" : ""));
      if (s.residual_mm != null) out.push(`residual ${s.residual_mm} mm`);
    }
  } else if (rep.kind === "crosses") {
    for (const p of rep.spots || []) out.push(`spot ${p.spot}: ${label(p.cross)} drew the cross, ${label(p.circle)} the circle.`);
    if (rep.instruction && rep.state === "done") out.push(rep.instruction);
  }
  if (rep.total_s != null) out.push(`Took ${Number(rep.total_s).toFixed(0)} s.`);
  return out.join("\n");
}

// --------------------------------------------------------------------------- rendering

function render() {
  renderArmSelect();
  renderRobots();
  renderMaterial();
  renderSetBoxes();
  renderImages();
  renderRig();
  renderActiveFile();
  renderMarkGroups();
  renderBusy();
  renderStatusBar();
  renderPopup();
}

function armState(slot) {
  const a = S.arms ? S.arms[slot] : null;
  const view = S.view && !FINAL.includes(S.view.state) ? S.view : null;
  const busy = view && (view.arms || []).some((r) => String(r.arm) === slot && r.current);
  if (!S.online) return ["grey", "server does not answer"];
  if (!a || !a.q) return ["red", "NO READING" + (a && a.reading ? `: ${a.reading}` : "")];
  if (a.ok === false) return ["red", "FAULT: " + ((a.flags || []).join(", ") || "not able to move")];
  if (busy || S.moving[slot]) return ["amber", "moving"];
  return ["green", a.at_park ? "at park" : "still, not at park"];
}

// ARM COUNT: one entry per slot, "2L  #71".
function renderArmSelect() {
  const sel = $("arm-select");
  const opts = slots().map((s) => [s, label(s)]);
  if (once("armSelect", JSON.stringify(opts))) {
    const keep = sel.value;
    sel.replaceChildren(...opts.map(([s, l]) => el("option", { value: s, text: l })));
    if (keep && slots().includes(keep)) sel.value = keep;
  }
  const [, st] = armState(selected());
  $("arm-select-state").textContent = st;
}

// the robots, live: the old SUPERVISION grid's look
function renderRobots() {
  const g = $("robots-grid");
  const ss = slots();
  const fresh = ss.filter((s) => S.arms && S.arms[s] && S.arms[s].q).length;
  const rows = [el("div", { class: "hdr-row" }, el("b", { text: "ROBOTS" }),
    el("span", { class: "dim", text: `   ${ss.length} slots · ${fresh} reading` +
      (S.code ? ` · ${S.code.same === false ? "OPERATOR PC CODE DIFFERENT" : S.code.line || ""}` : "") }))];
  for (const s of ss) {
    const a = S.arms ? S.arms[s] : null;
    const [light, st] = armState(s);
    const age = a && a.age_s != null ? `${Number(a.age_s).toFixed(1)} s` : a && a.q ? "now" : "";
    const cal = S.rig && S.rig.arms && S.rig.arms[s] ? S.rig.arms[s].calibration : null;
    const calTxt = cal && typeof cal === "object"
      ? Object.entries(cal).map(([k, v]) => `${k} ${String(v).startsWith("applied") ? "✓" : v === "none" ? "–" : "✗"}`).join(" ") : "";
    const cls = s === selected() ? "sel" : "";
    rows.push(el("span", { class: `dot-${light}`, text: light === "grey" ? "○" : "●" }),
              el("b", { class: cls, text: s }), el("span", { class: cls, text: robotOf(s) ? "#" + robotOf(s) : "" }),
              el("span", { class: light === "red" ? "dot-red" : cls, text: st }),
              el("span", { class: "dim", text: age }), el("span", { class: "dim", text: calTxt, title: cal ? JSON.stringify(cal) : "" }));
  }
  g.replaceChildren(...rows);
}

// MATERIAL: the pen in the selected slot's holder, from /pens.
function renderMaterial() {
  const sel = $("material"), p = S.pens || {};
  if (document.activeElement === sel) return;
  const names = p.table && p.table.length ? p.table : [((p.pens_in || {})[selected()]) || "?"];
  if (once("material", names.join(","))) sel.replaceChildren(...names.map((n) => el("option", { value: n, text: n.toUpperCase() })));
  sel.value = (p.pens_in || {})[selected()] || "";
  const pin = S.rig && S.rig.pens_in ? S.rig.pens_in[selected()] : null;
  const press = pin && pin.press_m != null ? (1000 * pin.press_m).toFixed(1) : "";
  $("pen-height").value = press;
  $("pen-height-2").value = press;
  $("force-label").textContent = pin ? `[PEN: ${String(pin.name || (p.pens_in || {})[selected()] || "?").toUpperCase()} | PRESS:${press} mm | SPEED:${pin.speed_m_per_s != null ? (1000 * pin.speed_m_per_s).toFixed(0) + " mm/s" : "?"}]` : "";
}

// SET: one checkbox per slot (the arms the SET buttons act on).
function renderSetBoxes() {
  const box = $("set-boxes");
  if (!once("setBoxes", slots().map(label).join(","))) return;
  const was = new Set([...box.querySelectorAll("input:checked")].map((i) => i.value));
  box.replaceChildren(...slots().map((s) => el("label", {},
    (() => { const i = el("input", { type: "checkbox", value: s }); i.checked = was.has(s); return i; })(),
    document.createTextNode(label(s)))));
}

// IMAGE FILE: the stored drawings, newest first; SVG CONFIG (YAML): the picked one.
function renderImages() {
  const sel = $("image-file");
  const list = [...S.drawings].sort((a, b) => (b.stored_at || 0) - (a.stored_at || 0));
  if (once("images", list.map((d) => d.id).join(","))) {
    const keep = S.pick || sel.value;
    sel.replaceChildren(...(list.length ? list.map((d) => el("option", { value: d.id, text: `${d.name}  (${d.lines} lines)` }))
                                        : [el("option", { value: "", text: "No file selected" })]));
    if (keep && list.some((d) => d.id === keep)) { sel.value = keep; if (S.pick === keep) S.pick = null; }
  }
  const d = S.drawings.find((x) => x.id === sel.value);
  const yaml = d ? Object.entries(d).map(([k, v]) => `${k}: ${Array.isArray(v) ? "[" + v.map((x) => typeof x === "number" ? +x.toFixed(4) : x).join(", ") + "]" : v}`).join("\n") : "";
  if (once("yaml", yaml)) $("svg-config-view").textContent = yaml || "(no drawing picked)";
  $("dc-picked").textContent = d ? `IMAGE FILE: ${d.name}` : "IMAGE FILE: none (SELECT SVG, or pick one under SVG (ADVANCED))";
}

function renderRig() {
  const r = S.rig;
  if (!r) return;
  const a = r.drawing_area_m || [], c = r.drawing_area_centre_m || [];
  $("paper-w").value = a[0] != null ? Number(a[0]).toFixed(3) : "";
  $("paper-l").value = a[1] != null ? Number(a[1]).toFixed(3) : "";
  $("center-x").value = c[0] != null ? Number(c[0]).toFixed(3) : "";
  $("center-y").value = c[1] != null ? Number(c[1]).toFixed(3) : "";
}

// ACTIVE FILE INFO and the SVG section's STROKE line: the running (or last) job.
function renderActiveFile() {
  const v = S.view;
  const p = progress(S.events || []);
  if (!v) {
    $("afi-arm").textContent = "ARM: —";
    $("afi-file").textContent = "FILE: —";
    $("afi-stroke").textContent = "STROKE: -- / --";
    $("afi-status").textContent = "STATUS: IDLE";
    $("afi-eta").textContent = "ETA: --";
    $("stroke-label").textContent = "STROKE:    - / -";
    return;
  }
  const arms = v.arms || [];
  const q = arms.reduce((s, r) => s + (r.queued || 0), 0), d = arms.reduce((s, r) => s + (r.done || 0), 0);
  const per = [...new Set(arms.map((r) => r.arm))];
  const fin = FINAL.includes(v.state);
  $("afi-arm").textContent = "ARM: " + (per.length ? per.map(label).join(", ") : "—") + `   JOB: ${v.id}`;
  $("afi-file").textContent = `FILE: ${v.name || "—"}  (${KIND_WORDS[v.kind] || v.kind})` +
    (v.drawing ? `, ${v.drawing.lines} lines` + (v.drawing.scale < 0.999 ? `, made smaller to ${(100 * v.drawing.scale).toFixed(0)} %` : "") : "");
  const strokes = per.map((a) => `${a}: ${p.strokes[a] || 0}` + (p.current[a] ? ` (${MOTION_WORDS[p.current[a]] || p.current[a]})` : "")).join("  ");
  $("afi-stroke").textContent = `STROKE: ${d} / ${q} motions` + (strokes ? `   strokes drawn  ${strokes}` : "");
  $("afi-status").textContent = `STATUS: ${String(v.state).toUpperCase()}${v.why ? " — " + v.why : ""}` +
    (p.phase ? `   phase ${p.phase}` : "") + (v.error ? `\n${v.error}` : "");
  $("afi-eta").textContent = fin ? "ETA: — (ended)" : `ETA: -- (running ${Number(v.elapsed_s || 0).toFixed(0)} s; planning goes on while the arms draw)`;
  $("stroke-label").textContent = `STROKE:    ${d} / ${q}` + (p.phase ? `   ${p.phase}` : "");
}

// MARK: the rig's mark groups, "all" last.
function renderMarkGroups() {
  const mg = (S.rig && S.rig.mark_groups) || {};
  const names = Object.keys(mg).filter((n) => n !== "all");
  if ("all" in mg) names.push("all");
  if (!once("markGroups", JSON.stringify(names.map((n) => [n, mg[n].map(label)])))) return;
  const sel = $("mark-group"), keep = sel.value;
  sel.replaceChildren(...names.map((n) => el("option", { value: n, text: `${n}  (${mg[n].join(", ")})` })));
  if (keep && names.includes(keep)) sel.value = keep;
}

// One job at a time: the buttons that start a job are greyed while one runs (the server would
// refuse anyway); STATUS says which one runs.
function renderBusy() {
  const run = running();
  for (const b of document.querySelectorAll(".need-idle")) b.disabled = !!run;
  for (const b of document.querySelectorAll("[data-stop]")) b.disabled = !run;
  $("btn-resume-drawing").disabled = $("btn-resume-svg").disabled = !!run || !resumable();
  if (once("running", run ? run.id + run.state : "")) {
    if (run) say(`job ${run.id} (${KIND_WORDS[run.kind] || run.kind}) is ${run.state}; one job at a time`, "wait");
  }
}

function renderStatusBar() {
  const run = running();
  const parts = [S.online === false ? `SYSTEM: NO SERVER — ${S.onlineWhy}` : "SYSTEM: READY"];
  if (S.code && S.code.same === false) parts.push("OPERATOR PC RUNS DIFFERENT CODE — jobs are refused until both are updated");
  if (S.rig && S.rig.uncalibrated) parts.push("UNCALIBRATED (nominal poses)");
  if (S.rig && S.rig.drawing_area_problem) parts.push(`NO DRAWING: ${S.rig.drawing_area_problem}`);
  if (S.rigWhy) parts.push(`/rig: ${S.rigWhy}`);
  parts.push(run ? `JOB ${run.id} ${run.state}` : "no job running");
  const bar = $("statusbar");
  bar.textContent = parts.join("  |  ");
  bar.style.color = S.online === false || (S.code && S.code.same === false) ? "#FF4444" : "#FFD700";
}

// The newest drawing job, when it stopped or failed: what RESUME continues.
function resumable() {
  const d = [...S.jobs].reverse().find((j) => j.kind === "draw");
  return d && (d.state === "stopped" || d.state === "failed") ? d : null;
}

// --------------------------------------------------------------------------- popups (views)

function openPopup(title, renderFn) {
  S.popup = { title, renderFn };
  $("popup").hidden = false;
  $("popup-title").textContent = title;
  renderPopup(true);
}
function renderPopup(now) {
  if (!S.popup) return;
  const body = $("popup-body");
  const keepScroll = body.scrollTop, atEnd = body.scrollHeight - body.scrollTop - body.clientHeight < 30;
  const got = S.popup.renderFn(now);
  if (got === null) return;              // a view that does not change by itself
  body.replaceChildren(...(Array.isArray(got) ? got : [got]));
  body.scrollTop = atEnd && S.popup.follow ? body.scrollHeight : keepScroll;
  if (now) body.scrollTop = S.popup.follow ? body.scrollHeight : 0;
}
const line = (text, cls = "") => el("div", { class: cls, text });

// SSH CHECKS: the checks, each PASS or FAIL.
function checksView() {
  const out = [];
  const check = (ok, what) => out.push(line(`${ok ? "PASS" : "FAIL"}  ${what}`, ok ? "ok" : "bad"));
  check(S.online, S.online ? "the drawing server answers" : `the drawing server does not answer: ${S.onlineWhy}`);
  if (S.code) check(S.code.same !== false, S.code.line || "code line");
  for (const s of slots()) {
    const [light, st] = armState(s);
    check(light === "green" || light === "amber", `${label(s)}: ${st}`);
  }
  if (S.rig) {
    check(!S.rig.drawing_area_problem, S.rig.drawing_area_problem ? `drawing area: ${S.rig.drawing_area_problem}` : "drawing area ok");
    check(!S.rig.uncalibrated, S.rig.uncalibrated ? "running UNCALIBRATED (nominal poses)" : "calibrated");
  }
  return out;
}

// SYSTEM STATUS: /rig + /arms + the code line.
function systemView() {
  const r = S.rig || {}, out = [];
  out.push(line(S.code ? `code: ${S.code.line}` : "code: ?", S.code && S.code.same === false ? "bad" : ""));
  out.push(line(`rig ${r.rig_digest || "?"}, calibration ${r.calibration_digest || "?"}, driver ${r.driver || "?"}, speed ${r.speed || "?"}` +
                (r.uncalibrated ? ", UNCALIBRATED" : "")));
  const a = r.drawing_area_m || [], c = r.drawing_area_centre_m || [];
  out.push(line(`drawing area ${a.map((x) => (+x).toFixed(3)).join(" x ")} m around (${c.map((x) => (+x).toFixed(3)).join(", ")})` +
                (r.drawing_area_problem ? `  NO DRAWING: ${r.drawing_area_problem}` : ""), r.drawing_area_problem ? "bad" : ""));
  const ps = r.paper_surface;
  out.push(line(`paper map: ${ps && ps.exists ? `${ps.points} points, ${ps.date || ""}` : "none (flat paper)"}`));
  out.push(line(""));
  for (const s of slots()) {
    const arm = S.arms ? S.arms[s] || {} : {}, rs = (r.arms || {})[s] || {};
    const [, st] = armState(s);
    const pin = (r.pens_in || {})[s];
    out.push(line(`${label(s)}  ${st}`));
    out.push(line(`   joints ${arm.q ? arm.q.map((x) => (+x).toFixed(3)).join(" ") : "—"}` +
                  (arm.age_s != null ? `   read ${(+arm.age_s).toFixed(1)} s ago` : "") + (arm.flags ? `   flags ${arm.flags.join(", ")}` : ""), "dim"));
    out.push(line(`   calibration ${JSON.stringify(rs.calibration || {})}   pen ${pin ? pin.name || JSON.stringify(pin) : "?"}`, "dim"));
  }
  return out;
}

// LOG FILES: the jobs of this server run, newest first; REPORT and EVENTS for each.
function logFilesView(now) {
  const list = [...S.jobs].reverse();
  // made again only when something in it changed, so a click is never lost to a rebuild
  const key = list.map((j) => j.id + j.state).join(",") + "|" + S.logShown + "|" + (S.logText || "").length;
  if (!now && S.popup.key === key) return null;
  S.popup.key = key;
  const left = el("div", {}, ...(list.length ? list.map((j) => el("div", { class: "item" + (S.logShown === j.id ? " shown" : "") },
    document.createTextNode(`${clock(j.received)}  ${KIND_WORDS[j.kind] || j.kind}  ${j.name || ""}  `),
    el("span", { class: j.state === "done" ? "ok" : FINAL.includes(j.state) ? "bad" : "", text: j.state + (j.why ? ` (${j.why})` : "") }),
    el("br"),
    el("button", { text: "REPORT", onclick: () => showLog(j.id, "report") }),
    el("button", { text: "EVENTS", onclick: () => showLog(j.id, "events") }),
    ...(j.kind === "draw" && (j.state === "stopped" || j.state === "failed")
      ? [el("button", { text: "RESUME DRAWING", onclick: () => resumeJob(j.id) })] : [])))
    : [line("no jobs since the server started", "dim")]));
  const right = el("div", {}, document.createTextNode(S.logText || "pick REPORT or EVENTS"));
  return el("div", { class: "gallery" }, left, right);
}
async function showLog(jid, what) {
  S.logShown = jid;
  if (what === "report") {
    const r = await call("GET", `/jobs/${encodeURIComponent(jid)}/report`);
    S.logText = `job ${jid}\n` + (r.ok ? plainReport(r.data) : answerText(r));
  } else {
    const r = await call("GET", `/jobs/${encodeURIComponent(jid)}/events`);
    S.logText = `job ${jid}: events\n` + (r.ok ? (r.data || []).map(eventLine).join("\n") : answerText(r));
  }
  renderPopup(true);
}
function eventLine(e) {
  const t = e.time ? new Date(e.time * 1000).toLocaleTimeString() : "";
  const rest = Object.entries(e).filter(([k]) => !["event", "time", "q", "seq", "source"].includes(k))
    .map(([k, v]) => `${k}=${typeof v === "object" ? JSON.stringify(v) : v}`).join(" ");
  return `${t} ${e.event} ${rest}`;
}

// VIEW LIVE PATH: the running job's events as they come.
function livePathView() {
  if (!S.view) return line("no job", "dim");
  return [line(`job ${S.view.id} (${KIND_WORDS[S.view.kind] || S.view.kind}) ${S.view.state}`, "ok"),
          ...(S.events || []).slice(-400).map((e) => line(eventLine(e), e.event && /fail|refus|error/.test(e.event) ? "bad" : ""))];
}

// WAYPOINT PEEK: per phase and arm, motions queued / done and the current one.
function peekView() {
  const v = S.view;
  if (!v) return line("no job", "dim");
  const out = [line(`job ${v.id} ${v.state}${v.why ? " — " + v.why : ""}`, "ok")];
  for (const r of v.arms || [])
    out.push(line(`${String(r.phase).padEnd(18)} ${label(r.arm).padEnd(10)} queued ${String(r.queued).padStart(4)}  done ${String(r.done).padStart(4)}  ` +
                  (r.current ? `now #${r.current.index} ${r.current.kind}` : r.status || "")));
  return out;
}

// --------------------------------------------------------------------------- the buttons

function noteAndAir() {
  const out = {};
  const note = $("draw-note").value.trim(), air = Number($("draw-air").value);
  if (note) out.note = note;
  if (air > 0) out.air_mm = String(air);
  return out;
}
function startDrawing(what) {
  const id = $("image-file").value;
  if (!id) { say(`${what}: no IMAGE FILE (SELECT SVG first, or pick one under SVG (ADVANCED))`, "bad"); return; }
  const q = noteAndAir();
  act(`${what} ${id}` + (q.air_mm ? ` in the air (${q.air_mm} mm)` : ""), "POST",
      "/jobs?" + new URLSearchParams(Object.assign({ drawing: id }, q)));
}
function resumeJob(jid) {
  act(`RESUME DRAWING ${jid}`, "POST", "/jobs?" + new URLSearchParams(Object.assign({ rest_of: jid }, noteAndAir())));
}
function resume(what) {
  const r = resumable();
  if (!r) { say(`${what}: no stopped or failed drawing to resume`, "bad"); return; }
  resumeJob(r.id);
}
function stopJob(what) {
  const run = running();
  if (!run) { say(`${what}: no job is running`, "wait"); return; }
  act(`${what} (job ${run.id})`, "POST", `/jobs/${encodeURIComponent(run.id)}/stop`, undefined,
      { ok: () => "every arm stops and holds; the job is ending" });
}
function gripBody(verb, set) {
  const w = Number($(set ? "grip-width-all" : "grip-width").value);
  const f = Number($(set ? "grip-force-all" : "grip-force").value);
  return { verb, width_m: w, force_n: f };
}
async function gripSet(verb) {
  const chosen = [...document.querySelectorAll("#set-boxes input:checked")].map((i) => i.value);
  if (!chosen.length) { say(`${verb.toUpperCase()} SET GRIPPERS: no arm ticked in SET`, "bad"); return; }
  for (const s of chosen) {                       // one job at a time: one slot after the other
    const res = await act(`${label(s)} gripper ${verb.toUpperCase()}`, "POST", `/grip/${encodeURIComponent(s)}`, gripBody(verb, true));
    if (!res.ok) { say(`${verb.toUpperCase()} SET GRIPPERS: stopped at ${label(s)}`, "bad"); return; }
    await untilDone(res.data.id);
  }
}
function uploadPicked() {
  const f = $("file-input").files[0];
  if (!f) return;
  const form = new FormData();
  form.append("file", f, f.name);
  const w = $("up-width").value.trim(), at = $("up-at").value.trim();
  if (w) form.append("width", w);
  if (at) form.append("at", at);
  act(`SELECT SVG ${f.name}`, "POST", "/drawings", undefined, {
    form, ok: (d) => { S.pick = d.id; S.keys.images = null; S.tick = 0;
                       return `stored as "${d.id}": ${d.lines} lines, ${d.points} points; it is the IMAGE FILE now`; } });
  $("file-input").value = "";
}

function wire() {
  // folds and section bars: ▸ / ▾ as the old toggles
  for (const b of document.querySelectorAll("[data-fold]")) {
    b.addEventListener("click", () => {
      const p = $(b.dataset.fold);
      p.hidden = !p.hidden;
      if (b.dataset.label) {
        const adv = b.classList.contains("adv-btn");
        b.textContent = (p.hidden ? (adv ? "▶ " : "▸ ") : (adv ? "▼ " : "▾ ")) + b.dataset.label;
      }
      try { localStorage.setItem("fold-" + b.dataset.fold, p.hidden ? "0" : "1"); } catch (e) { /* no storage */ }
    });
    try {
      const was = localStorage.getItem("fold-" + b.dataset.fold);
      if (was !== null && (was === "1") === $(b.dataset.fold).hidden) b.click();
    } catch (e) { /* no storage */ }
  }
  for (const t of document.querySelectorAll(".tab")) t.addEventListener("click", () => {
    for (const u of document.querySelectorAll(".tab")) { u.classList.toggle("active", u === t); $(u.dataset.tab).hidden = u !== t; }
  });
  $("arm-select").addEventListener("change", () => { S.keys.material = null; render(); });
  $("material").addEventListener("change", (e) => {
    const s = selected(), name = e.target.value;
    act(`MATERIAL ${label(s)}: ${name.toUpperCase()}`, "POST", `/pens/${encodeURIComponent(s)}`, { name },
        { ok: () => `${name.toUpperCase()} is in ${label(s)} now; Z TOUCH its pen before drawing` });
  });
  $("btn-verify").addEventListener("click", async () => {
    const r = await call("GET", "/arms");
    if (!r.ok) { say(`verify arms: ${answerText(r)}`, "bad"); return; }
    for (const [s, a] of Object.entries(r.data.arms || {}))
      say(`verify arms: ${label(s)}  ` + (a.q ? `joints ${a.q.map((x) => (+x).toFixed(3)).join(" ")}` +
          (a.age_s != null ? `  (${(+a.age_s).toFixed(1)} s ago)` : "") + (a.at_park ? "  at park" : "  not at park")
          : `NO READING: ${a.reading || "never reported"}`), a.q ? "ok" : "bad");
    say(`verify arms: ${r.data.code.line}` + (r.data.code.same === false ? "  — DIFFERENT: update both machines" : ""),
        r.data.code.same === false ? "bad" : "ok");
    poll(true);
  });
  $("btn-call-operator").addEventListener("click", () => {
    const s = selected();
    act(`CALL OPERATOR (recover ${label(s)})`, "POST", `/arms/${encodeURIComponent(s)}/recover`, undefined, {
      ok: (d) => d.queued ? "asked the robot PC to recover the arm (watch ROBOTS)" :
                 d.recovered ? "recovered" : `not recovered: ${d.why || "(no reason given)"}` });
  });
  $("btn-start-pos").addEventListener("click", () => act("START POS (park every arm)", "POST", "/park"));
  $("btn-start-set").addEventListener("click", () => act("START SET POS (park every arm)", "POST", "/park"));
  for (const b of document.querySelectorAll("[data-grip]")) b.addEventListener("click", () => {
    const s = selected(), v = b.dataset.grip;
    act(`${label(s)} gripper ${b.textContent}`, "POST", `/grip/${encodeURIComponent(s)}`, gripBody(v, false));
  });
  for (const b of document.querySelectorAll("[data-grip-set]")) b.addEventListener("click", () => gripSet(b.dataset.gripSet));
  for (const b of document.querySelectorAll("[data-stop]")) b.addEventListener("click", () => stopJob(b.dataset.stop));
  $("btn-emergency").addEventListener("click", () => {
    say("EMERGENCY: the arms' own emergency stop is the one in your hand — press it if anything is wrong", "bad");
    stopJob("EMERGENCY");
  });
  for (const id of ["btn-select-svg", "btn-select-svg-2"]) $(id).addEventListener("click", () => $("file-input").click());
  $("file-input").addEventListener("change", uploadPicked);
  $("btn-start-drawing").addEventListener("click", () => startDrawing("START DRAWING"));
  $("btn-start-svg").addEventListener("click", () => startDrawing("START SVG"));
  $("btn-resume-drawing").addEventListener("click", () => resume("RESUME DRAWING"));
  $("btn-resume-svg").addEventListener("click", () => resume("RESUME SVG"));
  $("image-file").addEventListener("change", render);
  $("btn-refresh-images").addEventListener("click", async () => {
    const r = await call("GET", "/drawings");
    if (!r.ok) { say(`IMAGE FILE ↻: ${answerText(r)}`, "bad"); return; }
    S.drawings = r.data || [];
    render();
    say(`IMAGE FILE ↻: ${S.drawings.length} stored, newest first`, "ok");
  });
  $("btn-z-touch").addEventListener("click", () => {
    const s = selected();
    act(`Z TOUCH ${label(s)} (touch-off: measures the pen's length)`, "POST", `/touchoff/${encodeURIComponent(s)}`);
  });
  $("btn-measure").addEventListener("click", () => {
    const s = selected();
    act(`MEASURE SURFACE ${label(s)} (calibrate: the paper under this arm)`, "POST", `/calibrate/${encodeURIComponent(s)}`);
  });
  const markDo = (what, path, extra, said) => {
    const g = $("mark-group").value;
    if (!g) { say(`${what}: the rig has no mark groups`, "bad"); return; }
    act(`${what} ${g}`, "POST", `${path}?` + new URLSearchParams(Object.assign({ group: g }, extra)), undefined,
        { ok: (d) => `started job ${d.id}: ${said}` });
  };
  const toDo = "the arms fly above the spot; then switch BOTH to programming mode in Desk, bring the pen tips together, let go, back to execution mode with FCI on";
  $("btn-mark").addEventListener("click", () => markDo("MARK", "/mark", {}, toDo));
  $("btn-mark-yaw").addEventListener("click", () => markDo("MARK + YAW", "/mark", { yaw: "true" }, "two meetings per pair; " + toDo));
  $("btn-crosses").addEventListener("click", () => markDo("CROSSES", "/crosses", {}, "the L arms draw crosses, the R arms circles"));
  $("btn-system-status").addEventListener("click", () => openPopup("SYSTEM STATUS", systemView));
  $("btn-ssh-checks").addEventListener("click", () => openPopup("SSH CHECKS  (system status)", checksView));
  $("btn-ssh-checks-2").addEventListener("click", () => openPopup("SSH CHECKS  (system status)", checksView));
  $("btn-log-files").addEventListener("click", () => { S.logText = ""; S.logShown = null; openPopup("LOG FILES", logFilesView); });
  $("btn-live-path").addEventListener("click", () => { openPopup("VIEW LIVE PATH", livePathView); S.popup.follow = true; renderPopup(true); });
  $("btn-waypoint-peek").addEventListener("click", () => openPopup("WAYPOINT PEEK", peekView));
  $("btn-afi-refresh").addEventListener("click", () => { S.headerFor = null; poll(true); say("ACTIVE FILE INFO: REFRESH ↻", "wait"); });
  $("btn-afi-details").addEventListener("click", () => openPopup("ACTIVE FILE INFO — the job header",
    () => line(S.header ? JSON.stringify(S.header, null, 1) : "no job")));
  $("popup-close").addEventListener("click", () => { S.popup = null; $("popup").hidden = true; });
  document.addEventListener("keydown", (e) => { if (e.key === "Escape") $("popup-close").click(); });
}

say("ARIS_KINDT READY");
say("AWAITING COMMANDS...");
wire();
poll(true);
setInterval(() => poll(false), POLL_MS);
