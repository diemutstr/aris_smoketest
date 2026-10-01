"""The resident process of the operator PC: `aris-robot serve` (DESIGN section 4, end).

Started once, at boot (robot/aris-robot.service), and never touched again.  It
  - brings up one ROS stack per mounted arm (`ros2 launch aris_bringup arm.launch.py`) and
    keeps it up: a stack that exits is started again after a pause that doubles up to a minute
    (reset after a minute of running), and every start and death is reported;
  - asks the drawing server what to do, pulling (`GET /operator/next`, long-poll), so this PC
    opens no port: run a job (a drawing, a park, a calibration: what `aris-robot run` does),
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

from aris.rig import Rig
from aris.types import Refusal

from aris_robot.runner import run_job

log = logging.getLogger("aris_robot.serve")


def _plain(q):
    return [float(x) for x in q]


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
    """One child process per arm, kept running."""

    def __init__(self, commands: dict, rows: Rows, log_dir: Path, first_pause: float = 1.0,
                 max_pause: float = 60.0, env: dict | None = None):
        self.commands, self.rows, self.dir = commands, rows, Path(log_dir)
        self.first_pause, self.max_pause, self.env = first_pause, max_pause, env
        self.state = {a: dict(running=False, starts=0, last_exit=None) for a in commands}
        self._procs: dict = {}
        self._quit = threading.Event()
        self._threads = [threading.Thread(target=self._keep, args=(a,), daemon=True)
                         for a in commands]

    def start(self) -> "Stacks":
        for t in self._threads:
            t.start()
        return self

    def _keep(self, arm) -> None:
        pause = self.first_pause
        while not self._quit.is_set():
            with open(self.dir / f"stack_arm{arm}.log", "ab") as out:
                p = subprocess.Popen(self.commands[arm], stdout=out, stderr=subprocess.STDOUT,
                                     start_new_session=True, env=self.env)
            self._procs[arm] = p
            st = self.state[arm]
            st.update(running=True, starts=st["starts"] + 1)
            self.rows.say("stack started", arm=arm, pid=p.pid, starts=st["starts"])
            t0 = time.monotonic()
            code = p.wait()
            st.update(running=False, last_exit=code)
            if self._quit.is_set():
                return
            if time.monotonic() - t0 > self.max_pause:
                pause = self.first_pause
            self.rows.say("stack died", arm=arm, exit_code=code, restart_in_s=pause)
            if self._quit.wait(pause):
                return
            pause = min(2.0 * pause, self.max_pause)

    def stop(self, grace: float = 10.0) -> None:
        self._quit.set()
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
    does not have is removed.  -> the arms that have one."""
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
        if p.stem.isdigit() and int(p.stem) not in files:
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
                 idle_s: float = 10.0, wait_s: float = 30.0, rows: Rows | None = None):
        self.remote, self.config, self.work = remote, Path(config_dir), Path(work_dir)
        self.drivers, self.stacks = drivers, stacks
        self.idle_s, self.wait_s = idle_s, wait_s
        self.rows = rows or Rows(remote, Path(log_dir))
        self.busy = threading.Event()
        self.quit = threading.Event()

    def where(self) -> dict:
        return {str(a): _plain(d.state().q) for a, d in self.drivers.items()}

    def serve(self) -> None:
        """Until `quit` is set."""
        self.rows.say("operator started", where=self.where(), arms=sorted(self.drivers),
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
        self.rows.say("operator stopping", where=self.where())

    def handle(self, cmd: dict) -> None:
        what = cmd.get("command")
        self.busy.set()
        try:
            if what == "run":
                self.run(str(cmd["job"]))
            elif what == "recover":
                self.recover(int(cmd["arm"]))
            elif what == "report":
                self.report()
            else:
                self.rows.say("command refused", command=what, why="unknown command")
        except Exception as e:                       # the process must outlive any one command
            log.exception("command %s", cmd)
            self.rows.say("command failed", command=what, why=f"{type(e).__name__}: {e}",
                          where=self.where())
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
        res = run_job(self.remote, job, rig, self.config, self.work, drivers)
        if isinstance(res, Refusal):
            self.rows.say("run refused", job=job, reason=res.reason, why=res.detail,
                          where=self.where())
        else:
            self.rows.say("run ended", job=job, status=res.status, why=res.why,
                          where=self.where())

    def recover(self, arm: int) -> None:
        d = self.drivers.get(arm)
        if d is None:
            self.rows.say("recover refused", arm=arm, why="not a mounted arm of this PC")
            return
        r = d.recover()
        self.rows.say("recovered" if r.done else "recover failed", arm=arm, why=r.why,
                      q=_plain(d.state().q))

    def report(self) -> None:
        arms = {}
        for a, d in self.drivers.items():
            s = d.state()
            arms[str(a)] = dict(q=_plain(s.q), qd=_plain(s.qd), ok=bool(s.ok), flags=list(s.flags))
        self.rows.say("report", arms=arms, where=self.where(),
                      stacks=None if self.stacks is None else self.stacks.state)

    def _idle(self) -> None:
        while not self.quit.wait(self.idle_s):
            if not self.busy.is_set():
                self.rows.say("where", where=self.where())
            else:
                self.rows.flush()


def launch_commands(args_files) -> dict:
    """arm id -> the `ros2 launch` command of its stack, from bringup's argument files."""
    out = {}
    for f in args_files:
        a = json.loads(Path(f).read_text())
        out[a["arm"]] = ["ros2", "launch", "aris_bringup", "arm.launch.py", f"args:={f}"]
    return out
