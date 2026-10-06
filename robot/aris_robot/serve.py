"""The resident process of the operator PC: `aris-robot serve` (DESIGN section 4, end).

Started once, at boot (robot/aris-robot.service), and never touched again.  It
  - brings up one ROS stack per mounted arm (`ros2 launch aris_bringup arm.launch.py`) and
    keeps it up: a stack that exits is started again after a pause that doubles up to a minute
    (reset after a minute of running), and every start and death is reported;
  - asks the drawing server what to do, pulling (`GET /operator/next`, long-poll), so this PC
    opens no port: run a job (a drawing, a park, a calibration: runner.run_job),
    recover an arm, report every arm;
  - fetches the calibration files from the server into its config before every job, so both
    machines hold the same ones (the server owns them);
  - reports where the arms stand every `idle_s` while no job runs (a "where" row);
  - says everything as rows to the server (`POST /operator/rows`) and to its log directory
    (rows.jsonl and serve.log); it needs no terminal.

Why one process supervising `ros2 launch` children, not a systemd unit per arm: the stacks
come and go with site.json's mounted arms without installing units, their deaths are reported
to the server as rows like everything else, and there is one unit to enable.  The children run
in their own process groups and are stopped with SIGINT (then SIGKILL) when serve stops.
"""
from __future__ import annotations

import json
import logging
import os
import signal
import subprocess
import threading
import time
from pathlib import Path

import numpy as np

from aris.rig import Rig
from aris.types import Refusal

from aris_robot.handover import HandOver, Switch
from aris_robot.runner import reading, run_job, where_of

log = logging.getLogger("aris_robot.serve")


def _plain(q):
    return [float(x) for x in q]


def _q_fields(driver) -> dict:
    q, why = reading(driver)
    return dict(q=q) if why is None else dict(q=None, reason=why)


class Rows:
    """Rows to the server, in order; kept and sent again while the server cannot be reached.
    Each row also goes to rows.jsonl in the log directory."""

    def __init__(self, remote, log_dir: Path):
        Path(log_dir).mkdir(parents=True, exist_ok=True)
        self.remote, self.file = remote, Path(log_dir) / "rows.jsonl"
        self.pending, self._lock = [], threading.Lock()

    def say(self, event: str, **fields) -> dict:
        row = dict(time=time.time(), event=event, **fields)
        with self._lock:
            with open(self.file, "a") as f:
                f.write(json.dumps(row, sort_keys=True) + "\n")
            self.pending.append(row)
            if len(self.pending) > 10_000:                # a long outage: keep the newest
                self.pending = self.pending[-10_000:]
        log.info("%s %s", event, {k: v for k, v in fields.items() if k not in ("where", "arms")})
        self.flush()
        return row

    def flush(self) -> bool:
        with self._lock:
            if not self.pending:
                return True
            ans = self.remote.post_rows(self.pending)
            if isinstance(ans, Refusal):
                return False
            self.pending = []
            return True


class Stacks:
    """One child process per arm, kept running; one can be paused (stopped and not restarted)
    while the calibration driver owns that arm's FCI connection."""

    def __init__(self, commands: dict, rows: Rows, log_dir: Path, first_pause: float = 1.0,
                 max_pause: float = 60.0, env: dict | None = None):
        self.commands, self.rows, self.dir = commands, rows, Path(log_dir)
        self.first_pause, self.max_pause, self.env = first_pause, max_pause, env
        self.state = {a: dict(running=False, starts=0, last_exit=None, paused=False)
                      for a in commands}
        self._procs: dict = {}
        self._locks = {a: threading.Lock() for a in commands}
        self._resumed = {a: threading.Event() for a in commands}
        self._quit = threading.Event()
        self._threads = [threading.Thread(target=self._keep, args=(a,), daemon=True)
                         for a in commands]

    def start(self) -> "Stacks":
        for t in self._threads:
            t.start()
        return self

    def _keep(self, arm) -> None:
        pause, st = self.first_pause, self.state[arm]
        while not self._quit.is_set():
            with self._locks[arm]:
                p = None
                if not st["paused"]:
                    with open(self.dir / f"stack_arm{arm}.log", "ab") as out:
                        p = subprocess.Popen(self.commands[arm], stdout=out,
                                             stderr=subprocess.STDOUT, start_new_session=True,
                                             env=self.env)
                    self._procs[arm] = p
                    st.update(running=True, starts=st["starts"] + 1)
            if p is None:                                   # paused: wait to be resumed
                self._resumed[arm].wait(0.1)
                continue
            self.rows.say("stack started", arm=arm, pid=p.pid, starts=st["starts"])
            t0 = time.monotonic()
            code = p.wait()
            st.update(running=False, last_exit=code)
            if self._quit.is_set():
                return
            if st["paused"]:
                pause = self.first_pause
                continue
            if time.monotonic() - t0 > self.max_pause:
                pause = self.first_pause
            self.rows.say("stack died", arm=arm, exit_code=code, restart_in_s=pause)
            if self._quit.wait(pause):
                return
            pause = min(2.0 * pause, self.max_pause)

    def pause(self, arm, grace: float = 15.0) -> str:
        """Stop `arm`'s stack and keep it stopped; returns once the process group has exited
        (its FCI connection closed), or why it could not."""
        with self._locks[arm]:
            self.state[arm]["paused"] = True
            self._resumed[arm].clear()
            p = self._procs.get(arm)
        if p is None or p.poll() is not None:
            return ""
        try:
            os.killpg(p.pid, signal.SIGINT)
        except ProcessLookupError:
            return ""
        try:
            p.wait(grace)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)
            try:
                p.wait(5.0)
            except subprocess.TimeoutExpired:
                return f"the stack of {arm} (pid {p.pid}) does not exit"
        return ""

    def resume(self, arm) -> None:
        """Start `arm`'s stack again (at once)."""
        with self._locks[arm]:
            self.state[arm]["paused"] = False
            self._resumed[arm].set()

    def stop(self, grace: float = 10.0) -> None:
        self._quit.set()
        for a in self._resumed:
            self._resumed[a].set()
        for p in self._procs.values():
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
        deadline = time.monotonic() + grace
        for p in self._procs.values():
            try:
                p.wait(max(0.0, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait()
        for t in self._threads:
            t.join(timeout=1.0)


def sync_calibration(remote, config_dir) -> list[int] | Refusal:
    """The server's calibration files into `config_dir/calibration`; a local file the server
    does not have is removed.  -> the slots that have one."""
    arms = remote.calibration_arms()
    if isinstance(arms, Refusal):
        return arms
    files = {}
    for a in arms:
        f = remote.calibration(a)
        if isinstance(f, Refusal):
            return f
        files[a] = f
    d = Path(config_dir) / "calibration"
    d.mkdir(parents=True, exist_ok=True)
    for p in d.glob("*.json"):
        if p.stem not in files:
            p.unlink()
    for a, f in files.items():
        tmp = d / f"{a}.json.tmp"
        tmp.write_text(json.dumps(f, indent=1, sort_keys=True))
        os.replace(tmp, d / f"{a}.json")
    return sorted(files)


class Operator:
    """The command loop.  `drivers`: arm id -> Driver, every mounted arm, kept for the
    process's life.  `stacks`: the Stacks, or None (simulated arms)."""

    def __init__(self, remote, config_dir, work_dir, drivers: dict, log_dir, stacks=None,
                 idle_s: float = 10.0, wait_s: float = 30.0, rows: Rows | None = None,
                 robots: dict | None = None, make_calib=None):
        """`robots`: slot -> {"robot", "ip", "serial_found", "identity"}: who the site table
        says hangs in each slot and whether that was verified (site.identity).
        `make_calib(slot, rig, say)`: a connected calibration driver (calib.CalibArm), for the
        mark jobs; None: mark jobs are refused."""
        self.remote, self.config, self.work = remote, Path(config_dir), Path(work_dir)
        self.drivers, self.stacks = drivers, stacks
        self.idle_s, self.wait_s = idle_s, wait_s
        self.rows = rows or Rows(remote, Path(log_dir))
        self.robots = robots or {}
        self.make_calib = make_calib
        self.auto_recover: dict = {}         # site.json execution.auto_recover
        self._recovered: dict = {}
        self.busy = threading.Event()
        self.quit = threading.Event()

    def where(self) -> dict:
        """slot -> 7 joints, or None when the arm has no reading (stack down, FCI off)."""
        return where_of(self.drivers)["where"]

    def where_fields(self) -> dict:
        """`where`, plus `where_missing` (slot -> why) when an arm has no reading."""
        return where_of(self.drivers)

    def serve(self) -> None:
        """Until `quit` is set."""
        self.rows.say("operator started", **self.where_fields(), arms=sorted(self.drivers),
                      robots=self.robots,
                      stacks=None if self.stacks is None else self.stacks.state)
        idle = threading.Thread(target=self._idle, daemon=True)
        idle.start()
        down = False
        while not self.quit.is_set():
            cmd = self.remote.next_command(self.wait_s)
            if isinstance(cmd, Refusal):
                if not down:
                    log.warning("the server does not answer: %s", cmd.detail)
                down = True
                self.quit.wait(2.0)
                continue
            if down:
                self.rows.say("server reachable again")
                down = False
            if cmd is not None:
                self.remote.ack(cmd.get("id"))
                self.handle(cmd)
        self.rows.say("operator stopping", **self.where_fields())

    def handle(self, cmd: dict) -> None:
        what = cmd.get("command")
        self.busy.set()
        try:
            if what == "run":
                self.run(str(cmd["job"]))
            elif what == "recover":
                self.recover(str(cmd["arm"]))
            elif what == "report":
                self.report()
            else:
                self.rows.say("command refused", command=what, why="unknown command")
        except Exception as e:                       # the process must outlive any one command
            log.exception("command %s", cmd)
            self.rows.say("command failed", command=what, why=f"{type(e).__name__}: {e}",
                          **self.where_fields())
        finally:
            self.busy.clear()

    def run(self, job: str) -> None:
        cal = sync_calibration(self.remote, self.config)
        if isinstance(cal, Refusal):
            self.rows.say("calibration not fetched", job=job, why=cal.detail)
        else:
            self.rows.say("calibration fetched", job=job, arms=cal)
        rig = Rig.load(self.config)
        for d in self.drivers.values():
            if hasattr(d, "retarget"):
                d.retarget(rig)
        drivers = {a: d for a, d in self.drivers.items() if a in rig.arm_ids}
        around = None
        header = self.remote.header(job)
        if isinstance(header, dict) and header.get("kind") == "mark":
            if self.make_calib is None:
                self.rows.say("run refused", job=job, reason="no_calibration_driver",
                              why="this operator PC has no calibration driver (aris_robot[calib])",
                              **self.where_fields())
                return
            drivers = {a: Switch(a, d) for a, d in drivers.items()}
            around = HandOver(drivers, lambda a, say: self.make_calib(a, rig, say),
                              self.stacks, self.rows.say).around
        res = run_job(self.remote, job, rig, self.config, self.work, drivers,
                      robots=self.robots, around_phase=around)
        if isinstance(res, Refusal):
            self.rows.say("run refused", job=job, reason=res.reason, why=res.detail,
                          **self.where_fields())
        else:
            self.rows.say("run ended", job=job, status=res.status, why=res.why,
                          **self.where_fields())

    def recover(self, arm: str) -> None:
        d = self.drivers.get(arm)
        if d is None:
            self.rows.say("recover refused", arm=arm, why="not a mounted arm of this PC")
            return
        r = d.recover()
        self.rows.say("recovered" if r.done else "recover failed", arm=arm, why=r.why,
                      **_q_fields(d))

    def report(self) -> None:
        arms = {}
        for a, d in self.drivers.items():
            s = d.state()
            arms[str(a)] = dict(**_q_fields(d), ok=bool(s.ok), flags=list(s.flags),
                                qd=_plain(np.nan_to_num(s.qd)))
        self.rows.say("report", arms=arms, **self.where_fields(),
                      stacks=None if self.stacks is None else self.stacks.state)

    def _idle(self) -> None:
        while not self.quit.wait(self.idle_s):
            if not self.busy.is_set():
                self.rows.say("where", **self.where_fields())
                self._auto_recover()
            else:
                self.rows.flush()

    def _auto_recover(self) -> None:
        """A link drop (a fault matching `auto_recover.patterns`) is recovered by itself, at
        most once per `every_s` per arm; any other fault waits for a person."""
        ar = self.auto_recover
        if not ar.get("on", False):
            return
        now = time.monotonic()
        for a, d in self.drivers.items():
            s = d.state()
            faults = " ".join(f for f in s.flags if f.startswith("fault"))
            if not faults or not any(p in faults for p in ar.get("patterns", [])):
                continue
            if now - self._recovered.get(a, -1e9) < float(ar.get("every_s", 120.0)):
                continue
            self._recovered[a] = now
            r = d.recover()
            self.rows.say("auto recovered" if r.done else "auto recover failed", arm=a,
                          fault=faults, why=r.why, **_q_fields(d))


def launch_commands(args_files, site=None) -> dict:
    """slot -> the `ros2 launch` command of its stack, from bringup's argument files; pinned
    to the slot's isolated core (site.json `rt_core`, `taskset -c`) when it has one, so no
    other process's real-time loop preempts its control loop (link drops, 2026-10-06)."""
    out = {}
    for f in args_files:
        a = json.loads(Path(f).read_text())
        cmd = ["ros2", "launch", "aris_bringup", "arm.launch.py", f"args:={f}"]
        core = None if site is None else site.arm(a["arm"]).rt_core
        out[a["arm"]] = cmd if core is None else ["taskset", "-c", str(core)] + cmd
    return out
