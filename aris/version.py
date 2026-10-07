"""Which code this is — so the two machines can tell whether they run the same.

`code_version()` gives the git commit of the checkout this package is imported from (and
whether it has uncommitted changes), plus a digest of the package's own source files.  The
digest is what the machines compare: it is the same for the same sources whichever way they
were installed, and it changes with a local patch even when the commit does not."""
from __future__ import annotations

import hashlib
import subprocess
from pathlib import Path

PACKAGE = Path(__file__).resolve().parent
REPO = PACKAGE.parent


def source_digest(root: Path = PACKAGE) -> str:
    """blake2b over every .py file under `root`, by relative path, in sorted order."""
    h = hashlib.blake2b(digest_size=8)
    for p in sorted(root.rglob("*.py")):
        if "__pycache__" in p.parts:
            continue
        h.update(str(p.relative_to(root)).encode())
        h.update(p.read_bytes())
    return h.hexdigest()


def git_commit(repo: Path = REPO) -> tuple[str | None, bool]:
    """(short commit, dirty) of the checkout at `repo`; (None, False) when it is not one."""
    try:
        sha = subprocess.run(["git", "-C", str(repo), "rev-parse", "--short", "HEAD"],
                             capture_output=True, text=True, timeout=5)
        if sha.returncode != 0:
            return None, False
        st = subprocess.run(["git", "-C", str(repo), "status", "--porcelain", "--", "aris", "robot"],
                            capture_output=True, text=True, timeout=5)
        return sha.stdout.strip(), bool(st.stdout.strip())
    except (OSError, subprocess.SubprocessError):
        return None, False


def code_version() -> dict:
    """{"commit": "e7741e5" | None, "dirty": bool, "digest": "..."}; `digest` is what to
    compare between machines, `commit` what to tell a person."""
    commit, dirty = git_commit()
    return dict(commit=commit, dirty=dirty, digest=source_digest())


def describe(v: dict | None) -> str:
    """One word-ish for a person: 'e7741e5', 'e7741e5+local changes', or 'unknown'."""
    if not v:
        return "unknown"
    s = v.get("commit") or f"digest {v.get('digest', '?')[:8]}"
    return s + ("+local changes" if v.get("dirty") else "")


def same(a: dict | None, b: dict | None) -> bool:
    return bool(a and b and a.get("digest") == b.get("digest"))
