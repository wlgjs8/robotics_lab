from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
TOOLS = ROOT / "tools"
for path in (str(ROOT), str(TOOLS)):
    if path not in sys.path:
        sys.path.insert(0, path)
