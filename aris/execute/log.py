"""The event log: every state change of every executor and of the coordinator, one JSON object
per line, in the job directory.  Each line goes to the file in one appending write, so any
number of writers (threads or processes) can share it and a reader sees whole lines only."""
from __future__ import annotations

import json
import os
import time
from pathlib import Path

import numpy as np


class EventLog:
    def __init__(self, path):
        self.path = Path(path)

    def write(self, event: str, **fields) -> dict:
        row = dict(time=time.time(), event=event)
        row.update({k: _plain(v) for k, v in fields.items()})
        fd = os.open(self.path, os.O_WRONLY | os.O_APPEND | os.O_CREAT, 0o644)
        try:
            os.write(fd, (json.dumps(row, sort_keys=True) + "\n").encode())
        finally:
            os.close(fd)
        return row

    def read(self) -> list[dict]:
        """Every complete line written so far."""
        if not self.path.exists():
            return []
        text = self.path.read_text()
        return [json.loads(x) for x in text[:text.rfind("\n") + 1].splitlines() if x]


def _plain(v):
    if isinstance(v, np.ndarray):
        return [float(x) for x in v.ravel()]
    if isinstance(v, np.generic):
        return v.item()
    if isinstance(v, (tuple, list)):
        return [_plain(x) for x in v]
    if isinstance(v, dict):
        return {str(k): _plain(x) for k, x in v.items()}
    return v
