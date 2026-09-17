#!/usr/bin/env bash
# Compare live RGB letterbox/crop at 224x224, plus a red pixel-difference overlay.
set -euo pipefail

cd "$(dirname "$0")/.."

if [ -n "${STACK_PYTHON:-}" ]; then
  PYTHON_BIN="$STACK_PYTHON"
elif [ -x ".venv/bin/python" ]; then
  PYTHON_BIN=".venv/bin/python"
else
  PYTHON_BIN="python3"
fi

export PYTHONPATH="$PWD/policy_runner${PYTHONPATH:+:$PYTHONPATH}"
exec "$PYTHON_BIN" -m policy_runner.camera_crop_preview "$@"
