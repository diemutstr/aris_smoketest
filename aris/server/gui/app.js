// The drawing arms page (GET /gui).  Plain JavaScript, no framework, no build step, no
// internet: every number on the page comes from the drawing server's endpoints, polled once a
// second.  Every button's answer goes to the status line at the top, word for word.
// docs/modules/gui.md says what each panel does and how to add a button.
"use strict";

const POLL_MS = 1000;            // /arms, /jobs and the current job
const SLOW_EVERY = 5;            // /rig and /drawings: every 5th poll
const FINAL = ["done", "stopped", "failed"];

const KIND_WORDS = {
  draw: "Drawing", park: "Park", grip: "Gripper", calibrate: "Calibrate (paper)",
  touchoff: "Touch-off (pen length)", mark: "Mark (where the arms hang)",
  crosses: "Crosses (the check)",
};

const S = {
  online: null, rig: null, arms: null, code: null,
  jobs: [], view: null, events: [], drawings: [],
  report: null, reportId: null,      // the report shown, and its job
  grip: {}, gripSeen: {},            // slot -> last grip result; grip job ids already read
  lastQ: {}, moving: {},             // slot -> last joints; slot -> moved since last poll
  built: { slots: "", calib: "" }, listKey: "", drawingsKey: "", tick: 0, busyPolling: false,
};

// --------------------------------------------------------------------------- small helpers

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

function mm(m, digits = 1) { return m == null ? "?" : (1000 * Number(m)).toFixed(digits) + " mm"; }
function metres(m) { return m == null ? "?" : Number(m).toFixed(2) + " m"; }
function clock(t) { return new Date(t * 1000).toLocaleTimeString(); }

function slots() {
  if (S.rig && S.rig.arms) return Object.keys(S.rig.arms);
  if (S.arms) return Object.keys(S.arms);
  return [];
}

// The robot number only when the server gives it (/arms, else /rig); otherwise null.
function robotOf(slot) {
  const a = S.arms && S.arms[slot], r = S.rig && S.rig.arms && S.rig.arms[slot];
  const said = (a && a.robot) || (r && r.robot) || "";
  const m = String(said).match(/(\d+)\s*$/);
  return m ? m[1] : (said ? String(said) : null);
}

function label(slot) { const n = robotOf(slot); return n ? `${slot} · robot ${n}` : slot; }

function running() { return S.jobs.find((j) => !FINAL.includes(j.state)) || null; }

// --------------------------------------------------------------------------- the server

async function call(method, path, body, form) {
  const opt = { method, headers: {} };
  if (form) opt.body = form;
  else if (body !== undefined) {
    opt.body = JSON.stringify(body);
    opt.headers["Content-Type"] = "application/json";
  }
  let r;
  try {
    r = await fetch(path, opt);
  } catch (e) {
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

// --------------------------------------------------------------------------- the status line

function setStatus(text, kind) {
  const line = $("status-line");
  line.textContent = text;
  line.className = "status-" + kind;
  if (kind === "wait") return;
  const li = el("li", { class: kind === "bad" ? "bad" : "ok", text: `${new Date().toLocaleTimeString()}  ${text}` });
  const list = $("status-list");
  list.prepend(li);
  while (list.children.length > 50) list.lastChild.remove();
}

// One button press: ask the server, show its answer (success or refusal) in the status line.
async function act(what, method, path, body, opts = {}) {
  setStatus(`${what}: asking the server…`, "wait");
  const res = await call(method, path, body, opts.form);
  if (res.ok) {
    const d = res.data || {};
    let said = opts.ok ? opts.ok(d) : (d.id ? `started job ${d.id}` : (res.text || "done"));
    setStatus(`${what}: ${said}`, "ok");
  } else {
    setStatus(`${what}: ${answerText(res)}`, "bad");
  }
  poll(true);
  return res;
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
      slow || now ? call("GET", "/pens") : null,
    ]);
    if (pens && pens.ok) S.pens = pens.data;
    S.online = arms.status !== 0;
    S.onlineWhy = arms.ok ? "" : answerText(arms);
    if (arms.ok) { trackMotion(arms.data.arms); S.arms = arms.data.arms; S.code = arms.data.code; }
    if (jobs.ok) S.jobs = jobs.data || [];
    if (rig && rig.ok) S.rig = rig.data;
    if (rig && !rig.ok) S.rigWhy = answerText(rig); else if (rig) S.rigWhy = "";
    if (drawings && drawings.ok) S.drawings = drawings.data || [];
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

// The current job (the running one, else the newest), its events while it runs, and the
// report of the newest finished one.
async function pollJob() {
  const run = running();
  const cur = run || S.jobs[S.jobs.length - 1];
  if (!cur) { S.view = null; S.events = []; return; }
  const v = await call("GET", `/jobs/${encodeURIComponent(cur.id)}`);
  S.view = v.ok ? v.data : { id: cur.id, kind: cur.kind, state: cur.state, why: cur.why, error: answerText(v) };
  const needEvents = run || !S.events.length || S.eventsFor !== cur.id;
  if (needEvents) {
    const ev = await call("GET", `/jobs/${encodeURIComponent(cur.id)}/events`);
    S.events = ev.ok ? ev.data || [] : [];
    S.eventsFor = cur.id;
    // the driver's instructions to the person (mark job): the newest one in the status line
    const said = S.events.filter((r) => r.event === "instruction");
    if (run && said.length && S.saidCount !== said.length) {
      S.saidCount = said.length;
      const r = said[said.length - 1];
      setStatus(`${r.arm ? label(r.arm) + ": " : ""}${r.text || r.why || ""}`, "wait");
    }
  }
  const done = [...S.jobs].reverse().find((j) => FINAL.includes(j.state));
  if (done && S.autoReportId !== done.id) {
    S.autoReportId = done.id;
    await showReport(done.id);
  }
}

async function showReport(jid) {
  const r = await call("GET", `/jobs/${encodeURIComponent(jid)}/report`);
  S.reportId = jid;
  S.report = r.ok ? r.data : { error: answerText(r) };
  render();
}

// The width each gripper was left at: read once from each finished grip job's report.
async function pollGrips() {
  for (const j of S.jobs) {
    if (j.kind !== "grip" || !FINAL.includes(j.state) || S.gripSeen[j.id]) continue;
    S.gripSeen[j.id] = true;
    const r = await call("GET", `/jobs/${encodeURIComponent(j.id)}/report`);
    if (!r.ok || !r.data) continue;
    const rep = r.data;
    S.grip[rep.slot] = { verb: rep.verb, state: rep.state, why: rep.why, width: rep.width_after_m,
                         grasped: rep.grasped, when: j.received };
  }
}

// --------------------------------------------------------------------------- what the job did

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

const MOTION_WORDS = { draw: "drawing a stroke", free: "moving (pen up)", touch: "touching the paper",
                       guide: "waiting for you to guide the pen" };

// The report in plain words.
function plainReport(rep) {
  if (!rep) return "No finished job yet.";
  if (rep.error) return rep.error;
  const out = [];
  const st = rep.state;
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
    if (rep.left_m > 0.001 && (st === "stopped" || st === "failed")) out.push("What was left can be drawn with DRAW THE REST below.");
  } else if (rep.kind === "park") {
    for (const [a, r] of Object.entries(rep.arms || {})) out.push(`${label(a)}: ${r.result}`);
  } else if (rep.kind === "grip") {
    out.push(`${label(rep.slot)} gripper ${rep.verb}: width ${mm(rep.width_before_m)} → ${mm(rep.width_after_m)}` +
             (rep.grasped == null || rep.verb !== "close" ? "" : rep.grasped ? " (holding something)" : " (holding nothing)"));
  } else if (rep.kind === "calibrate") {
    out.push(`${label(rep.arm)}: ${rep.points || 0} points touched (${rep.contacts || 0} met the paper), ` +
             `${(rep.dropped || []).length} out of reach.`);
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
      if (s.reference) out.push(`reference: ${label(s.reference)}`);
    }
  } else if (rep.kind === "crosses") {
    for (const p of rep.spots || [])
      out.push(`spot ${p.spot}: ${label(p.cross)} drew the cross, ${label(p.circle)} the circle.`);
    if (rep.instruction && rep.state === "done") out.push(rep.instruction);
  }
  if (rep.total_s != null) out.push(`Took ${Number(rep.total_s).toFixed(0)} s.`);
  return out.join("\n");
}

// --------------------------------------------------------------------------- rendering

function render() {
  renderBanners();
  buildCards();
  updateCards();
  renderJob();
  renderJobList();
  renderDrawings();
  buildCalib();
  renderCalib();
  renderBusy();
}

function renderBanners() {
  const pill = $("server-light");
  pill.className = "pill " + (S.online ? "pill-green" : "pill-red");
  pill.textContent = S.online ? "server answers" : "NO SERVER";
  const b = $("banners");
  b.replaceChildren();
  if (S.online === false) b.append(el("div", { class: "banner banner-red", text: `The drawing server does not answer: ${S.onlineWhy}. Is \`aris serve\` running on the planning laptop?` }));
  else if (S.onlineWhy) b.append(el("div", { class: "banner banner-red", text: `/arms: ${S.onlineWhy}` }));
  if (S.rigWhy) b.append(el("div", { class: "banner banner-red", text: `/rig: ${S.rigWhy}` }));
  if (S.code && S.code.same === false)
    b.append(el("div", { class: "banner banner-red", text: `The robot PC runs DIFFERENT code than this laptop; jobs are refused until both are updated. ${S.code.line}` }));
  if (S.rig && S.rig.drawing_area_problem)
    b.append(el("div", { class: "banner banner-amber", text: `NO DRAWING: ${S.rig.drawing_area_problem} (park, calibrate and marks still run)` }));
  if (S.rig && S.rig.uncalibrated)
    b.append(el("div", { class: "banner banner-amber", text: "Running UNCALIBRATED (nominal poses)." }));
}

// Arm cards are made once per set of slots and then only their texts change, so a click is
// never lost to a card being rebuilt under the pointer.
function buildCards() {
  const key = slots().join(",");
  if (key === S.built.slots) return;
  S.built.slots = key;
  const box = $("arm-cards");
  box.replaceChildren();
  for (const slot of slots()) {
    const grip = (verb) => el("button", { class: "btn need-idle", "data-slot": slot, text: verb.toUpperCase(),
      onclick: () => act(`${label(slot)} gripper ${verb.toUpperCase()}`, "POST", `/grip/${encodeURIComponent(slot)}`, { verb }) });
    const card = el("div", { class: "card", id: `card-${slot}` },
      el("div", { class: "card-head" },
        el("div", { class: "light light-grey", id: `light-${slot}` }),
        el("div", {}, el("div", { class: "card-name", text: slot }),
                      el("div", { class: "card-robot", id: `robot-${slot}` }))),
      el("div", { class: "card-state", id: `state-${slot}` }),
      el("div", { class: "card-small", id: `age-${slot}` }),
      el("div", { class: "card-small", id: `cal-${slot}` }),
      el("div", { class: "btn-row" }, el("span", { class: "card-small", text: "pen in:" }),
        (() => {
          const sel = el("select", { id: `pen-${slot}`, class: "need-idle", "aria-label": `pen in ${slot}` });
          sel.addEventListener("change", () => act(`${label(slot)}: pen ${sel.value}`, "POST",
            `/pens/${encodeURIComponent(slot)}`, { name: sel.value }, {
              ok: () => `${sel.value} is in ${label(slot)} now; touch off its pen before drawing` }));
          return sel;
        })()),
      el("div", { class: "btn-row" }, grip("home"), grip("open"), grip("close")),
      el("div", { class: "card-small", id: `grip-${slot}` }),
      el("div", { class: "btn-row" },
        el("button", { class: "btn btn-small", text: "RECOVER",
          onclick: () => act(`RECOVER ${label(slot)}`, "POST", `/arms/${encodeURIComponent(slot)}/recover`, undefined, {
            ok: (d) => d.queued ? "asked the robot PC to recover the arm (watch the card)" :
                       d.recovered ? "recovered" : `not recovered: ${d.why || "(no reason given)"}` }) })));
    box.append(card);
  }
}

function updateCards() {
  const view = S.view && !FINAL.includes(S.view.state) ? S.view : null;
  for (const slot of slots()) {
    if (!$(`card-${slot}`)) continue;
    const a = S.arms ? S.arms[slot] : null;
    const busyArm = view && (view.arms || []).some((r) => String(r.arm) === slot && r.current);
    let light = "grey", state;
    if (!S.online) { state = "server does not answer"; }
    else if (!a || !a.q) { light = "red"; state = "NO READING" + (a && a.reading ? `: ${a.reading}` : ""); }
    else if (a.ok === false) { light = "red"; state = "FAULT: " + ((a.flags || []).join(", ") || "not able to move"); }
    else if (busyArm || S.moving[slot]) { light = "amber"; state = "moving"; }
    else { light = "green"; state = a.at_park ? "at its park" : "holding still (not at its park)"; }
    if (a && a.ok !== false && a.flags && a.flags.length) state += ` (${a.flags.join(", ")})`;
    $(`light-${slot}`).className = `light light-${light}`;
    $(`robot-${slot}`).textContent = robotOf(slot) ? `robot ${robotOf(slot)}` : "";
    $(`state-${slot}`).textContent = state;
    $(`age-${slot}`).textContent = !a ? "" : a.age_s != null ? `joints read ${Number(a.age_s).toFixed(1)} s ago` :
      a.source ? "" : "joints read now (driver on this computer)";
    const cal = S.rig && S.rig.arms && S.rig.arms[slot] ? S.rig.arms[slot].calibration : null;
    const calTxt = cal && typeof cal === "object"
      ? Object.entries(cal).map(([k, v]) => `${k} ${String(v).startsWith("applied") ? "yes" : v === "none" ? "none" : "NOT applied: " + v}`).join(", ")
      : cal ? String(cal) : "?";
    const calEl = $(`cal-${slot}`);
    calEl.textContent = `calibration: ${calTxt}`;
    calEl.title = cal ? JSON.stringify(cal) : "";
    const sel = $(`pen-${slot}`), pens = S.pens || {};
    if (sel && document.activeElement !== sel) {
      const names = pens.table && pens.table.length ? pens.table : [((pens.pens_in || {})[slot]) || "?"];
      if (sel.dataset.names !== names.join(",")) {
        sel.replaceChildren(...names.map((n) => el("option", { value: n, text: n })));
        sel.dataset.names = names.join(",");
      }
      sel.value = (pens.pens_in || {})[slot] || "";
    }
    const g = S.grip[slot];
    $(`grip-${slot}`).textContent = !g ? "gripper: not used since the server started" :
      g.state !== "done" ? `gripper ${g.verb} ${g.state}: ${g.why || ""}` :
      `gripper ${g.verb} at ${clock(g.when)}: width ${mm(g.width)}` + (g.grasped == null || g.verb !== "close" ? "" : g.grasped ? ", holding something" : ", holding nothing");
  }
}

function renderJob() {
  const box = $("job-current");
  const v = S.view;
  const run = running();
  $("btn-stop").disabled = !run;
  if (!v) { box.textContent = "No job yet (since the server started)."; return; }
  const rows = [];
  const fin = FINAL.includes(v.state);
  rows.push(el("div", { class: "state " + (v.state === "done" ? "state-done" : fin ? "state-failed" : "state-running"),
    text: `${KIND_WORDS[v.kind] || v.kind || "job"}: ${v.state}${v.why ? " — " + v.why : ""}` }));
  if (v.name) rows.push(el("div", { class: "row", text: `what: ${v.name}` }));
  rows.push(el("div", { class: "row card-small", text: `job ${v.id}` + (v.elapsed_s != null && !fin ? `, running ${Number(v.elapsed_s).toFixed(0)} s` : "") }));
  if (v.error) rows.push(el("div", { class: "row status-bad", text: v.error }));
  const p = progress(S.events || []);
  if (p.phase) rows.push(el("div", { class: "row", text: `phase: ${p.phase}` }));
  if (v.drawing) rows.push(el("div", { class: "row", text: `lines in the drawing: ${v.drawing.lines}` +
    (v.drawing.scale != null && v.drawing.scale < 0.999 ? ` (made smaller to ${(100 * v.drawing.scale).toFixed(0)} %)` : "") }));
  const arms = v.arms || [];
  if (arms.length) {
    const q = arms.reduce((s, r) => s + (r.queued || 0), 0), d = arms.reduce((s, r) => s + (r.done || 0), 0);
    rows.push(el("div", { class: "row", text: `motions done: ${d} of ${q} planned so far` + (fin ? "" : " (planning goes on while the arms draw)") }));
    const per = {};
    for (const r of arms) per[r.arm] = r;
    for (const a of Object.keys(per)) {
      const strokes = p.strokes[a] || 0, now = p.current[a];
      const doing = now == null ? "" : MOTION_WORDS[now] || now;
      rows.push(el("div", { class: "row", text: `${label(a)}: strokes drawn ${strokes}` + (doing ? `; now ${doing}` : "") }));
    }
  }
  box.replaceChildren(...rows);
  // the button lives in its own box, made again only when the job or its state changes
  const actKey = `${v.id}:${v.state}`;
  if (actKey !== S.actKey) {
    S.actKey = actKey;
    const acts = $("job-actions");
    acts.replaceChildren();
    if (fin && v.kind === "draw" && (v.state === "stopped" || v.state === "failed"))
      acts.append(restButton(v.id));
  }
  const rep = $("job-report");
  rep.textContent = S.reportId ? `Job ${S.reportId}\n` + plainReport(S.report) : "No finished job yet.";
}

function restButton(jid) {
  return el("button", { class: "btn need-idle", text: "DRAW THE REST",
    onclick: () => act(`DRAW THE REST of ${jid}`, "POST", `/jobs?` + new URLSearchParams(
      Object.assign({ rest_of: jid }, noteAndAir()))) });
}

function renderJobList() {
  const list = [...S.jobs].reverse().slice(0, 12);
  const key = list.map((j) => `${j.id}:${j.state}`).join(",");
  if (key === S.listKey) return;
  S.listKey = key;
  const ul = $("job-list");
  ul.replaceChildren();
  if (!list.length) { ul.append(el("li", { text: "none yet" })); return; }
  for (const j of list) {
    const cls = j.state === "done" ? "state-done" : FINAL.includes(j.state) ? "state-failed" : "state-running";
    const li = el("li", {},
      el("span", { text: clock(j.received) }),
      el("span", { text: `${KIND_WORDS[j.kind] || j.kind}${j.name ? " · " + j.name : ""}` }),
      el("span", { class: cls, text: j.state + (j.why ? ` (${j.why})` : "") }));
    if (FINAL.includes(j.state))
      li.append(el("button", { class: "btn btn-small", text: "report", onclick: () => showReport(j.id) }));
    if (j.kind === "draw" && (j.state === "stopped" || j.state === "failed")) {
      const b = restButton(j.id);
      b.classList.add("btn-small");
      li.append(b);
    }
    ul.append(li);
  }
}

function renderDrawings() {
  const key = S.drawings.map((d) => d.id).join(",");
  const sel = $("drawing-select");
  if (key !== S.drawingsKey) {
    S.drawingsKey = key;
    const keep = S.pick || sel.value;
    sel.replaceChildren();
    if (!S.drawings.length) sel.append(el("option", { value: "", text: "(none uploaded yet: add one below)" }));
    else sel.append(el("option", { value: "", text: "— pick a drawing —" }));
    for (const d of [...S.drawings].reverse()) {
      sel.append(el("option", { value: d.id, text: `${d.name}  (${d.lines} lines, added ${clock(d.stored_at)})` }));
    }
    if (keep && S.drawings.some((d) => d.id === keep)) {
      sel.value = keep;
      if (S.pick === keep) S.pick = null;
    }
  }
  const d = S.drawings.find((x) => x.id === sel.value);
  let info = "";
  if (d) {
    const b = d.bbox_m;
    info = `${d.lines} lines, ${d.points} points` +
      (b && b.length === 4 ? `; ${(b[2] - b[0]).toFixed(2)} × ${(b[3] - b[1]).toFixed(2)} m around (${((b[0] + b[2]) / 2).toFixed(2)}, ${((b[1] + b[3]) / 2).toFixed(2)})` : "");
  }
  $("drawing-info").textContent = info;
}

// Calibration buttons: made once the rig is known.
function buildCalib() {
  const ss = slots();
  const key = ss.join(",") + "|" + JSON.stringify(rowPairs());
  if (!ss.length || key === S.built.calib) return;
  S.built.calib = key;
  const markBox = $("mark-buttons"), checkBox = $("crosses-buttons");
  markBox.replaceChildren();
  checkBox.replaceChildren();
  const toDo = "the arms fly above the spot; then switch BOTH to programming mode in Desk, bring the pen tips together, let go, back to execution mode with FCI on";
  for (const r of rowPairs()) {
    const name = `row ${r.row} (${label(r.slots[0])}, ${label(r.slots[1])})`;
    const q = { slots: r.slots.join(",") };
    markBox.append(el("button", { class: "btn btn-main need-idle", text: `MARK ${name}`,
      onclick: () => act(`MARK ${name}`, "POST", "/mark?" + new URLSearchParams(q), undefined, {
        ok: (d) => `started job ${d.id}: ${toDo}` }) }));
    if (r.spots.length >= 2)
      markBox.append(el("button", { class: "btn need-idle", text: `MARK + YAW ${name}`,
        onclick: () => act(`MARK + YAW ${name}`, "POST", "/mark?" + new URLSearchParams({ ...q, yaw: "true" }), undefined, {
          ok: (d) => `started job ${d.id}: two meetings, ${toDo}` }) }));
    checkBox.append(el("button", { class: "btn need-idle", text: `CROSSES ${name}`,
      onclick: () => act(`CROSSES ${name}`, "POST", "/crosses?" + new URLSearchParams(q), undefined, {
        ok: (d) => `started job ${d.id}: the arms draw their crosses and circles` }) }));
  }
  const tBox = $("touchoff-buttons"), cBox = $("calibrate-buttons");
  tBox.replaceChildren();
  cBox.replaceChildren();
  for (const s of ss) {
    tBox.append(el("button", { class: "btn need-idle", text: `TOUCH-OFF ${label(s)}`,
      onclick: () => act(`TOUCH-OFF ${label(s)}`, "POST", `/touchoff/${encodeURIComponent(s)}`) }));
    cBox.append(el("button", { class: "btn need-idle", text: `CALIBRATE ${label(s)}`,
      onclick: () => act(`CALIBRATE ${label(s)}`, "POST", `/calibrate/${encodeURIComponent(s)}`) }));
  }
}

// The rows whose L and R slot share spots (from /rig `marks`): {row, slots: [L, R], spots}.
function rowPairs() {
  const marks = (S.rig && S.rig.marks) || {}, rows = {};
  for (const [n, m] of Object.entries(marks)) {
    const s = [...m.shared_by].sort();
    if (s.length !== 2 || s[0][0] !== s[1][0] || s[0][1] === s[1][1]) continue;
    const k = s[0][0];
    if (!rows[k]) rows[k] = { row: k, slots: [s.find((x) => x.endsWith("L")), s.find((x) => x.endsWith("R"))], spots: [] };
    rows[k].spots.push(n);
  }
  return Object.values(rows).filter((r) => r.slots.every((x) => slots().includes(x)));
}

function renderCalib() {
  const files = (S.rig && S.rig.calibration_files) || {};
  const parts = [];
  for (const s of slots()) {
    const f = files[s];
    if (!f) { parts.push(`${label(s)}: no calibration file`); continue; }
    const kinds = ["base", "pen"].filter((k) => f[k])
      .map((k) => `${k} ${f[k].passed ? "passed" : "FAILED"} ${f[k].date || ""}`);
    parts.push(`${label(s)}: ${kinds.join("; ") || "file without base or pen"}`);
  }
  $("calib-state").textContent = parts.length ? "Saved calibrations — " + parts.join(" · ") : "";
}

// One job at a time: the buttons that start a job are greyed out while one runs.  The note
// says why; a refusal from the server still shows in the status line.
function renderBusy() {
  const run = running();
  for (const b of document.querySelectorAll(".need-idle")) b.disabled = !!run;
  for (const id of ["btn-park", "btn-draw"]) $(id).disabled = !!run;
  const note = $("busy-note");
  note.hidden = !run;
  if (run) note.textContent = `Job ${run.id} (${KIND_WORDS[run.kind] || run.kind}) is ${run.state}. ` +
    "One job at a time: wait for it to end, or press STOP.";
}

// --------------------------------------------------------------------------- the inputs

function noteAndAir() {
  const out = {};
  const note = $("draw-note").value.trim();
  if (note) out.note = note;
  if ($("draw-air").checked) out.air_mm = String(Number($("draw-air-mm").value) || 30);
  return out;
}

function wire() {
  $("btn-stop").addEventListener("click", () => {
    const run = running();
    if (!run) { setStatus("STOP: no job is running", "idle"); return; }
    act(`STOP job ${run.id}`, "POST", `/jobs/${encodeURIComponent(run.id)}/stop`, undefined,
        { ok: () => "every arm stops and holds; the job is ending" });
  });
  $("btn-park").addEventListener("click", () =>
    act("PARK", "POST", "/park"));
  $("btn-draw").addEventListener("click", () => {
    const id = $("drawing-select").value;
    if (!id) { setStatus("DRAW: pick a drawing first (or add one with ADD DRAWING)", "bad"); return; }
    const air = $("draw-air").checked;
    act(`DRAW ${id}` + (air ? " in the air" : ""), "POST",
        "/jobs?" + new URLSearchParams(Object.assign({ drawing: id }, noteAndAir())));
  });
  $("drawing-select").addEventListener("change", renderDrawings);
  $("btn-upload").addEventListener("click", async () => {
    const f = $("upload-file").files[0];
    if (!f) { setStatus("ADD DRAWING: choose a .json or .svg file first", "bad"); return; }
    const form = new FormData();
    form.append("file", f, f.name);
    const w = $("upload-width").value.trim(), at = $("upload-at").value.trim();
    if (w) form.append("width", w);
    if (at) form.append("at", at);
    const res = await act(`ADD DRAWING ${f.name}`, "POST", "/drawings", undefined, {
      form, ok: (d) => `added as "${d.id}": ${d.lines} lines, ${d.points} points. Picked it for DRAW.` });
    if (res && res.ok && res.data && res.data.id) { S.pick = res.data.id; S.drawingsKey = ""; S.tick = 0; poll(true); }
  });
}

wire();
poll(true);
setInterval(() => poll(false), POLL_MS);
