"""Franka Desk, the robot's own web interface, behind a tiny interface: the operating mode
(programming: the light goes white and the person may guide the arm; execution: FCI may move
it), the pilot buttons on the arm (check, cross, circle), the brakes, FCI on or off.

`PandaDesk` talks to a real robot through panda-py's `Desk` (the most used open
implementation of Desk's web API).  `SimDesk` is scripted, for the tests.  Credentials come from
`robot/secrets.json` (gitignored, never in the repository):

    {"default": {"username": "...", "password": "..."},
     "fr3-71": {"username": "...", "password": "..."}}     a robot's own entry wins

What could not be checked here (no robot): Desk's operating-mode request on this firmware (its
method, path and bodies are robot/site.json `desk.mode_endpoint`, so fixing it is a config
edit), and the event names of the pilot buttons (named once, below).  Every Desk call of
`PandaDesk` is reported as a row with its HTTP status, so a wrong endpoint shows at once.
"""
from __future__ import annotations

import json
import queue
import threading
import time
from pathlib import Path
from typing import Iterator, Protocol

BUTTONS = ("check", "cross", "circle")
MODES = ("programming", "execution")


class Desk(Protocol):
    def mode(self, name: str) -> None:
        """"programming" (guiding, light white) or "execution" (FCI may move it)."""

    def buttons(self, timeout: float) -> Iterator[str]:
        """Pilot button presses ("check", "cross", "circle", or others), as they come, until
        `timeout` seconds have passed."""

    def unlock(self) -> None: ...

    def lock(self) -> None: ...

    def fci(self, on: bool) -> None: ...


def credentials(path, robot: str) -> tuple[str, str]:
    """(username, password) for `robot` from the secrets file; its own entry, else "default"."""
    d = json.loads(Path(path).read_text())
    e = d.get(robot) or d.get("default")
    if not e:
        raise KeyError(f"{path} has no entry for {robot} and no default")
    return str(e["username"]), str(e["password"])


class SimDesk:
    """A scripted Desk: `script` is the buttons pressed, in order, each after `delay` s (None:
    nobody presses anything).  `on_mode(name)` is called on every mode switch (a test moves its
    fake arm there, as a person would).  Every call is recorded in `calls`."""

    def __init__(self, script=(), delay: float = 0.0, on_mode=None):
        self.script, self.delay, self.on_mode = list(script), delay, on_mode
        self.calls: list[tuple] = []
        self.current = "execution"

    def mode(self, name: str) -> None:
        if name not in MODES:
            raise ValueError(f"no mode {name!r}")
        self.calls.append(("mode", name))
        self.current = name
        if self.on_mode is not None:
            self.on_mode(name)

    def buttons(self, timeout: float) -> Iterator[str]:
        """The next scripted presses; a None in the script: nobody presses during this wait."""
        self.calls.append(("buttons", timeout))
        t_end = time.monotonic() + timeout
        while self.script and time.monotonic() < t_end:
            b = self.script.pop(0)
            if b is None:
                break
            time.sleep(self.delay)
            yield b
        time.sleep(max(0.0, t_end - time.monotonic()))

    def unlock(self) -> None:
        self.calls.append(("unlock",))

    def lock(self) -> None:
        self.calls.append(("lock",))

    def fci(self, on: bool) -> None:
        self.calls.append(("fci", bool(on)))


class PandaDesk:
    """Desk through panda-py (`pip install aris_robot[calib]`).  `endpoint`: site.json
    `desk.mode_endpoint` ({"method", "path", "body": {mode name: JSON body}}).  `say(event,
    **fields)`: every call becomes a row "desk: <call>" with its HTTP status (or the error)."""

    def __init__(self, ip: str, username: str, password: str, endpoint: dict, say=None,
                 platform: str = "fr3"):
        import panda_py                                   # optional dependency
        self.endpoint, self.say = endpoint, say or (lambda event, **f: None)
        self.desk = self._call("login", lambda: panda_py.Desk(ip, username, password,
                                                              platform=platform))
        self._call("take control", lambda: self.desk.take_control(force=True))

    def _call(self, what: str, fn):
        try:
            r = fn()
        except Exception as e:
            self.say("desk: " + what, ok=False, status=getattr(getattr(e, "response", None),
                                                                 "status_code", None),
                     why=f"{type(e).__name__}: {e}")
            raise
        self.say("desk: " + what, ok=True, status=getattr(r, "status_code", None))
        return r

    def mode(self, name: str) -> None:
        if name not in MODES:
            raise ValueError(f"no mode {name!r}")
        e = self.endpoint
        # panda-py has no call for this; its own authenticated request helper is used
        r = self._call(f"mode {name}", lambda: self.desk._request(
            e["method"], e["path"], json=e["body"][name]))
        status = getattr(r, "status_code", None)
        if status is not None and status >= 400:
            raise RuntimeError(f"Desk answered {status} to {e['method']} {e['path']}; fix "
                               f"robot/site.json desk.mode_endpoint")

    def buttons(self, timeout: float) -> Iterator[str]:
        q: queue.Queue = queue.Queue()

        def on_event(event: dict) -> None:
            for name, pressed in event.items():
                if pressed:
                    q.put(name)

        t = threading.Thread(target=self.desk.listen, args=(on_event, timeout), daemon=True)
        t.start()
        self.say("desk: listening to the pilot", timeout_s=timeout)
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            try:
                yield q.get(timeout=max(0.01, min(0.5, t_end - time.monotonic())))
            except queue.Empty:
                continue

    def unlock(self) -> None:
        self._call("unlock", self.desk.unlock)

    def lock(self) -> None:
        self._call("lock", self.desk.lock)

    def fci(self, on: bool) -> None:
        self._call("fci on" if on else "fci off",
                   self.desk.activate_fci if on else self.desk.deactivate_fci)
