import sys
from pathlib import Path

ROBOT = Path(__file__).resolve().parents[1]
if str(ROBOT) not in sys.path:          # aris_robot without installing it
    sys.path.insert(0, str(ROBOT))
