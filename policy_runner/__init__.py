"""Import shim for running policy_runner tests from the repository root."""

from __future__ import annotations

from pathlib import Path

_ROOT = Path(__file__).resolve().parent
_PACKAGE = _ROOT / "policy_runner"

__path__ = [str(_PACKAGE), str(_ROOT)]
