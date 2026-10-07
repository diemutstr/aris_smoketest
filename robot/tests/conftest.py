import sys
from pathlib import Path

ROBOT = Path(__file__).resolve().parents[1]
if str(ROBOT) not in sys.path:          # aris_robot without installing it
    sys.path.insert(0, str(ROBOT))

import pytest  # noqa: E402

from aris.version import code_version  # noqa: E402

# This PC's code version, read once: other agents may edit aris/ while the tests run, and the
# digest would change between a header written here and the runner reading its own.
CODE = code_version()


@pytest.fixture(autouse=True)
def _fixed_code(monkeypatch):
    import aris_robot.runner
    import aris_robot.serve
    monkeypatch.setattr(aris_robot.runner, "code_version", lambda: CODE)
    monkeypatch.setattr(aris_robot.serve, "code_version", lambda: CODE)
