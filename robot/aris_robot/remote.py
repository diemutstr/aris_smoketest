"""The drawing server, seen from the operator PC.  Standard library only.

The server writes each job as a directory (aris/execute/queue.py).  The operator PC keeps a
copy of the parts it runs, byte for byte, by following files as they grow:

  GET  /jobs/{id}/header                       job.json, as JSON
  GET  /jobs/{id}/phases?offset=B              phases.jsonl from byte B on, held open and
                                               streamed as it grows, closed after its end line
  GET  /jobs/{id}/queues/{phase}/{arm}?offset=B
                                               that queue file from byte B on, held open and
                                               streamed as it grows, closed after the end marker
  POST /jobs/{id}/events                       {"source": "robot", "rows": [{"seq": n, ...}]}:
                                               event rows from the operator PC, appended to the
                                               job's event log; seq counts from 0 and a row the
                                               server already has (seq below its count) is
                                               skipped.  Answers {"accepted": k, "next_seq": n,
                                               "stop": bool}; stop is true once the job was
                                               stopped on the server.

The resident process (`aris-robot serve`, serve.py) also uses:

  GET  /operator/next?wait=S                   long-poll, held up to S seconds: the next
                                               command, {"id": c, "command": "run", "job": j}
                                               | {"id": c, "command": "recover", "arm": a}
                                               | {"id": c, "command": "report"}; 204 (or {})
                                               when there is none
  POST /operator/ack                           {"id": c}: the command is taken
  POST /operator/rows                          {"source": "robot", "rows": [...]}: what the
                                               operator PC says outside a job (started,
                                               where, report, recovered, stack died, a run
                                               refused, ...); every row has "event" and
                                               "time", rows about arms have "where" or "q"
  GET  /calibration                            {"arms": [ids that have a calibration file]}
  GET  /calibration/{arm}                      that file, as JSON (404: none)

A lost link stops nothing: the copy resumes from its own length, and what is already copied
keeps running (DESIGN.md section 5).
"""
from __future__ import annotations

import json
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

from aris.types import Refusal


class Remote:
    def __init__(self, base_url: str, timeout: float = 5.0):
        self.base, self.timeout = base_url.rstrip("/"), float(timeout)

    def url(self, *parts, **query) -> str:
        path = "/".join(urllib.parse.quote(str(p), safe="") for p in parts)
        q = ("?" + urllib.parse.urlencode(query)) if query else ""
        return f"{self.base}/{path}{q}"

    def get_json(self, *parts) -> dict | Refusal:
        try:
            with urllib.request.urlopen(self.url(*parts), timeout=self.timeout) as r:
                return json.loads(r.read())
        except urllib.error.HTTPError as e:
            return Refusal("server", f"{e.code} from {'/'.join(map(str, parts))}: {_body(e)}")
        except (urllib.error.URLError, OSError, ValueError) as e:
            return Refusal("unreachable", f"{self.base}: {e}")

    def header(self, job: str) -> dict | Refusal:
        return self.get_json("jobs", job, "header")

    def _post(self, body: dict, *parts, timeout=None) -> dict | Refusal:
        req = urllib.request.Request(self.url(*parts), data=json.dumps(body).encode(),
                                     method="POST", headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=timeout or self.timeout) as r:
                data = r.read()
                return json.loads(data) if data else {}
        except urllib.error.HTTPError as e:
            return Refusal("server", f"{e.code} from {'/'.join(map(str, parts))}: {_body(e)}")
        except (urllib.error.URLError, OSError, ValueError) as e:
            return Refusal("unreachable", f"{self.base}: {e}")

    def next_command(self, wait: float = 30.0) -> dict | None | Refusal:
        """The server's next command for this PC, waiting up to `wait` s; None: nothing."""
        try:
            with urllib.request.urlopen(self.url("operator", "next", wait=wait),
                                        timeout=wait + self.timeout) as r:
                data = r.read()
                if r.status == 204 or not data.strip():
                    return None
                cmd = json.loads(data)
                return cmd if cmd.get("command") else None
        except urllib.error.HTTPError as e:
            return Refusal("server", f"{e.code} from operator/next: {_body(e)}")
        except (urllib.error.URLError, OSError, ValueError) as e:
            return Refusal("unreachable", f"{self.base}: {e}")

    def ack(self, cmd_id) -> dict | Refusal:
        return self._post(dict(id=cmd_id), "operator", "ack")

    def post_rows(self, rows: list[dict]) -> dict | Refusal:
        return self._post(dict(source="robot", rows=rows), "operator", "rows")

    def calibration_arms(self) -> list[int] | Refusal:
        ans = self.get_json("calibration")
        return ans if isinstance(ans, Refusal) else [int(a) for a in ans.get("arms", [])]

    def calibration(self, arm: int) -> dict | Refusal:
        return self.get_json("calibration", arm)

    def post_events(self, job: str, rows: list[dict]) -> dict | Refusal:
        body = json.dumps(dict(source="robot", rows=rows)).encode()
        req = urllib.request.Request(self.url("jobs", job, "events"), data=body, method="POST",
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                return json.loads(r.read())
        except urllib.error.HTTPError as e:
            return Refusal("server", f"{e.code} posting events: {_body(e)}")
        except (urllib.error.URLError, OSError, ValueError) as e:
            return Refusal("unreachable", f"{self.base}: {e}")

    def follow(self, parts: tuple, dest: Path, finished, stop: threading.Event,
               retry: float = 0.5) -> bool:
        """Appends the remote file to `dest` until `finished(dest)` or `stop`; resumes from the
        local length after a lost link.  Returns True when finished."""
        dest = Path(dest)
        while not stop.is_set():
            if dest.exists() and finished(dest):
                return True
            offset = dest.stat().st_size if dest.exists() else 0
            try:
                with urllib.request.urlopen(self.url(*parts, offset=offset),
                                            timeout=self.timeout) as r, open(dest, "ab") as f:
                    while not stop.is_set():
                        chunk = r.read1(1 << 16)
                        if not chunk:
                            break
                        f.write(chunk)
                        f.flush()
                stop.wait(0.05)                         # closed early: ask again shortly
            except (urllib.error.URLError, OSError):    # includes 404: not written yet
                stop.wait(retry)
        return dest.exists() and finished(dest)


def _body(e: urllib.error.HTTPError) -> str:
    try:
        return e.read().decode(errors="replace")[:300]
    except OSError:
        return ""


class EventForwarder:
    """Posts every line of the local event log to the server, in order, and learns from the
    answer whether the job was stopped there.  A heartbeat goes out when there is nothing new."""

    def __init__(self, remote: Remote, job: str, log_path, on_stop, period: float = 0.2):
        self.remote, self.job, self.path = remote, job, Path(log_path)
        self.on_stop, self.period = on_stop, period
        self.sent, self.offset, self.pending = 0, 0, []
        self._quit = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)

    def start(self) -> "EventForwarder":
        self._thread.start()
        return self

    def _read_new(self) -> None:
        if not self.path.exists():
            return
        with open(self.path, "rb") as f:
            f.seek(self.offset)
            data = f.read()
        end = data.rfind(b"\n") + 1
        for line in data[:end].splitlines():
            if line.strip():
                self.pending.append(json.loads(line))
        self.offset += end

    def push_once(self) -> bool:
        """One post of everything not yet accepted.  True when the server took it."""
        self._read_new()
        rows = [dict(seq=self.sent + i, **r) for i, r in enumerate(self.pending)]
        ans = self.remote.post_events(self.job, rows)
        if isinstance(ans, Refusal):
            return False
        nxt = int(ans.get("next_seq", self.sent + len(rows)))
        taken = max(0, min(len(self.pending), nxt - self.sent))
        self.pending, self.sent = self.pending[taken:], self.sent + taken
        if ans.get("stop"):
            self.on_stop()
        return True

    def _loop(self) -> None:
        while not self._quit.is_set():
            self.push_once()
            self._quit.wait(self.period)

    def close(self, timeout: float = 10.0) -> bool:
        """Stops the loop, then tries until everything is posted or `timeout` passes."""
        self._quit.set()
        self._thread.join()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.push_once() and not self.pending:
                self._read_new()
                if not self.pending:
                    return True
            time.sleep(0.2)
        return False
