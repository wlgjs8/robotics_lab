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

export PYTHONPATH="$PWD/policy_runner${PYTHONPATH:+:$PYTHONPATH}"

if ! OPENCV_PREFLIGHT=$(
  "$PYTHON_BIN" -m policy_runner.camera_preview --check-gui 2>&1
); then
  echo "camera preview requires GUI-enabled OpenCV, numpy, and pyzmq for: $PYTHON_BIN" >&2
  echo "$OPENCV_PREFLIGHT" >&2
  exit 2
fi

echo "[camera-preview] $OPENCV_PREFLIGHT"
echo "[camera-preview] topic=camera.bundle.policy cameras=left_realsense_color,right_realsense_color"
echo "[camera-preview] press q/ESC or close the window to exit"

exec "$PYTHON_BIN" -m policy_runner.camera_preview \
  --zmq-endpoint "tcp://127.0.0.1:5600" \
  --topic "camera.bundle.policy" \
  --cameras "left_realsense_color,right_realsense_color" \
  --max-age-ms 200 \
  --refresh-hz 30 \
  "$@"
