"""Franka Desk, the robot's own web interface, behind a tiny interface: control of Desk (one
holder at a time), the operating mode (programming: the light goes white and the person may
guide the arm; execution: FCI may move it), the pilot buttons on the arm, the brakes, FCI.

`PandaDesk` talks to a real robot through panda-py's `Desk`.  `SimDesk` is scripted, for the
tests.  Credentials come from `robot/secrets.json` (gitignored, never in the repository):

    {"default": {"username": "...", "password": "..."},
     "fr3-71": {"username": "...", "password": "..."}}     a robot's own entry wins

Found on the hardware (2026-10-06, FR3 system 5.9):
  - control: `take_control(force=True)` waits for a circle press on the pilot when someone
    else holds control (the browser, or a token left by an earlier attempt).  The calibration
    driver takes control once per arm turn and releases it at the end of the turn, always.
  - mode: `POST /desk/api/operating-mode/programming` (or `/execution`), empty body, header
    `X-Control-Token: <token>`, answers 200.  It is robot/site.json `desk.mode_endpoint`.
  - leaving execution mode switches FCI off; FCI must be switched on (and confirmed) before
    libfranka connects.
  - pilot buttons: check, cross, circle, left, right, up, down.  panda-py 1.1.1's `listen(cb)`
    runs in its own thread; `stop_listen()` ends it.
Every Desk call of `PandaDesk` is reported as a row "desk: <call>" with its HTTP status.
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
    def take_control(self, wait_s: float) -> None:
        """Become Desk's holder; when someone else holds it, wait up to `wait_s` for the
        person's circle press.  Raises when control was not given."""

    def release_control(self) -> None: ...

    def mode(self, name: str) -> None:
        """"programming" (guiding, light white; FCI goes off) or "execution"."""

    def buttons(self, timeout: float) -> Iterator[str]:
        """Pilot button presses, as they come, until `timeout` seconds have passed."""

    def unlock(self) -> None: ...

    def lock(self) -> None: ...

    def fci(self, on: bool) -> None:
        """Switch FCI on or off; returns once Desk confirmed it (raises otherwise)."""


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
    fake arm there, as a person would).  `held_by_other`: someone else holds control, so taking
    it needs the circle press (`grant`: whether the person gives it).  Leaving execution mode
    switches FCI off, as on the robot.  Every call is recorded in `calls`."""

    def __init__(self, script=(), delay: float = 0.0, on_mode=None, held_by_other=False,
                 grant=True):
        self.script, self.delay, self.on_mode = list(script), delay, on_mode
        self.held_by_other, self.grant = held_by_other, grant
        self.calls: list[tuple] = []
        self.current, self.fci_on, self.control = "execution", False, False

    def take_control(self, wait_s: float) -> None:
        self.calls.append(("take control", wait_s))
        if self.held_by_other and not self.grant:
            raise TimeoutError(f"nobody pressed circle within {wait_s:g} s")
        self.control = True

    def release_control(self) -> None:
        self.calls.append(("release control",))
        self.control = False
        self.fci_on = False                   # releasing control switches FCI off

    def mode(self, name: str) -> None:
        if name not in MODES:
            raise ValueError(f"no mode {name!r}")
        if not self.control:
            raise PermissionError("Desk: mode switch without control")
        self.calls.append(("mode", name))
        self.current = name
        if name == "programming":
            self.fci_on = False
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
        if not self.control:
            raise PermissionError("Desk: FCI switch without control")
        self.calls.append(("fci", bool(on)))
        self.fci_on = bool(on) and self.current == "execution"


class PandaDesk:
    """Desk through panda-py (`pip install aris_robot[calib]`).  `endpoint`: site.json
    `desk.mode_endpoint` ({"method", "path" with "{mode}", "token_header"}).  `say(event,
    **fields)`: every call becomes a row "desk: <call>" with its HTTP status (or the error)."""

    def __init__(self, ip: str, username: str, password: str, endpoint: dict, say=None,
                 platform: str = "fr3"):
        import panda_py                                   # optional dependency
        self.endpoint, self.say = endpoint, say or (lambda event, **f: None)
        self.desk = self._call("login", lambda: panda_py.Desk(ip, username, password,
                                                              platform=platform))

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

    def take_control(self, wait_s: float) -> None:
        if self._call("take control", lambda: self.desk.take_control(force=False)):
            return
        self.say("desk: someone holds control: press circle on the pilot", wait_s=wait_s)
        done: list = []
        t = threading.Thread(target=lambda: done.append(self.desk.take_control(force=True)),
                             daemon=True)
        t.start()
        t.join(wait_s)
        if not done or not done[0]:
            self.say("desk: control not given", wait_s=wait_s)
            raise TimeoutError(f"nobody pressed circle within {wait_s:g} s")
        self.say("desk: control given")

    def release_control(self) -> None:
        self._call("release control", self.desk.release_control)

    def mode(self, name: str) -> None:
        if name not in MODES:
            raise ValueError(f"no mode {name!r}")
        e = self.endpoint
        token = getattr(self.desk, "_token", None)
        headers = {e.get("token_header", "X-Control-Token"): str(token)}
        # panda-py has no call for this; its own authenticated request helper is used, with no
        # body (Content-Length 0)
        r = self._call(f"mode {name}", lambda: self.desk._request(
            e["method"], e["path"].format(mode=name), headers=headers))
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

        self._call("listen", lambda: self.desk.listen(on_event))   # its own thread
        try:
            t_end = time.monotonic() + timeout
            while time.monotonic() < t_end:
                try:
                    yield q.get(timeout=max(0.01, min(0.5, t_end - time.monotonic())))
                except queue.Empty:
                    continue
        finally:
            self._call("stop listen", self.desk.stop_listen)

    def unlock(self) -> None:
        self._call("unlock", self.desk.unlock)

    def lock(self) -> None:
        self._call("lock", self.desk.lock)

    def fci(self, on: bool) -> None:
        self._call("fci on" if on else "fci off",
                   self.desk.activate_fci if on else self.desk.deactivate_fci)
