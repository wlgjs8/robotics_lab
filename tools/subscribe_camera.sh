#!/usr/bin/env bash
# Open a side-by-side OpenCV preview of the live left/right wrist RGB streams.
# camera_server publishes bundle metadata over ZMQ and stores pixels in shared
# memory; policy_runner.camera_preview handles both parts of that contract.
set -euo pipefail

cd "$(dirname "$0")/.."

if [ -n "${STACK_PYTHON:-}" ]; then
  PYTHON_BIN="$STACK_PYTHON"
elif [ -x ".venv/bin/python" ]; then
  PYTHON_BIN=".venv/bin/python"
else
  PYTHON_BIN="python3"
fi

if ! OPENCV_PREFLIGHT=$("$PYTHON_BIN" - <<'PY' 2>&1
import cv2
import numpy  # noqa: F401
import zmq  # noqa: F401

gui_line = next(
    (line.strip() for line in cv2.getBuildInformation().splitlines()
     if line.strip().startswith("GUI:")),
    "GUI: unknown",
)
if gui_line.upper() in {"GUI: NONE", "GUI: UNKNOWN"}:
    raise RuntimeError(
        f"OpenCV HighGUI is unavailable ({gui_line}); "
        "opencv-python-headless cannot open a preview window"
    )
PY
); then
  echo "camera preview requires GUI-enabled OpenCV, numpy, and pyzmq for: $PYTHON_BIN" >&2
  echo "$OPENCV_PREFLIGHT" >&2
  echo "Replace the headless wheel with:" >&2
  echo "  $PYTHON_BIN -m pip uninstall -y opencv-python-headless" >&2
  echo "  $PYTHON_BIN -m pip install --upgrade opencv-python numpy pyzmq" >&2
  exit 2
fi

export PYTHONPATH="$PWD/policy_runner${PYTHONPATH:+:$PYTHONPATH}"

echo "[camera-preview] topic=camera.bundle.policy cameras=left_realsense_color,right_realsense_color"
echo "[camera-preview] press q/ESC or close the window to exit"

exec "$PYTHON_BIN" -m policy_runner.camera_preview \
  --zmq-endpoint "tcp://127.0.0.1:5600" \
  --topic "camera.bundle.policy" \
  --cameras "left_realsense_color,right_realsense_color" \
  --max-age-ms 200 \
  --refresh-hz 30 \
  "$@"
