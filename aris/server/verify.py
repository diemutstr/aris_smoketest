"""The independent checker as the planners' `verify`: handed to the system planner, which binds
it to each arm, phase and footprints and hands it to the arm planners in their processes.

`CheckVerify(config_dir, refused_dir)(arm_id, phase, fields, motion, q_before)` runs
`aris.check.check` in whichever process calls it and answers the checker's key numbers
(`verdict_numbers`, which is what `Motion.checked` carries).  A motion that fails is also kept
as `refused/<phase>__arm<id>__<n>.npz` in the job directory: the job directory is the record
of what happened, and a refusal without the motion is not a record.  Several processes write
there at once, so n is the first free number for that phase and arm, taken by creating the
file exclusively.
"""
from __future__ import annotations

import io
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from aris.check import check
from aris.execute.queue import slug, verdict_numbers


@dataclass(frozen=True)
class CheckVerify:
    config_dir: Path
    refused_dir: Path | None = None

    def __call__(self, arm_id, phase, fields, motion, q_before) -> dict:
        v = check(self.config_dir, arm_id, motion, phase, q_before, fields=tuple(fields))
        out = verdict_numbers(v)
        if not v.passed and self.refused_dir is not None:
            out["refused_file"] = save_refused(self.refused_dir, phase.name, arm_id, motion,
                                               q_before, v)
        return out


def save_refused(d: Path, phase: str, arm: int, motion, q_before, verdict) -> str:
    """The refused motion with where the arm stood and what the checker said; -> file name."""
    d = Path(d)
    d.mkdir(parents=True, exist_ok=True)
    p = motion.piece
    arrays = dict(t=motion.traj.t, q=motion.traj.q, qd=motion.traj.qd,
                  q_before=np.asarray(q_before, float), kind=np.array(motion.kind),
                  intensity=np.array(float(motion.intensity)),
                  piece=np.array(json.dumps(None if p is None else [p.line_id, p.s0, p.s1])),
                  failed=np.array("\n".join(str(m) for m in verdict.measurements
                                            if not m.passed)),
                  verdict=np.array(str(verdict)))
    if motion.tip_base is not None:
        arrays["tip_base"] = motion.tip_base
    buf = io.BytesIO()
    np.savez(buf, **arrays)
    stem = f"{slug(phase)}__arm{arm}__"
    n = len(list(d.glob(stem + "*.npz")))
    while True:
        try:
            with open(d / f"{stem}{n}.npz", "xb") as f:
                f.write(buf.getvalue())
            return f"{stem}{n}.npz"
        except FileExistsError:
            n += 1
