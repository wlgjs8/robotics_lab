#!/usr/bin/env bash
# Open a side-by-side OpenCV preview of the live left/right wrist RGB streams.
# camera_server publishes bundle metadata over ZMQ and stores pixels in shared
# memory; policy_runner.camera_preview handles both parts of that contract.
set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$repo_root"

if [[ -n "${STACK_PYTHON:-}" ]]; then
  python_bin=$STACK_PYTHON
elif [[ -x ".venv/bin/python" ]]; then
  python_bin=".venv/bin/python"
else
  python_bin="python3"
fi

if ! opencv_gui_backend=$("$python_bin" - <<'PY' 2>&1
import sys

try:
    import cv2
    import numpy  # noqa: F401
    import zmq  # noqa: F401
except Exception as exc:
    print(f"camera preview dependency check failed: {exc}", file=sys.stderr)
    raise SystemExit(1)

backend = None
for line in cv2.getBuildInformation().splitlines():
    stripped = line.strip()
    if stripped.startswith("GUI:"):
        backend = stripped.partition(":")[2].strip()
        break

if not backend or backend.upper() == "NONE":
    detected = backend or "unknown"
    print(
        "OpenCV HighGUI is unavailable "
        f"(GUI: {detected}); opencv-python-headless cannot open a preview window",
        file=sys.stderr,
    )
    raise SystemExit(1)

print(backend)
PY
); then
  echo "camera preview requires GUI-enabled OpenCV, numpy, and pyzmq for: $python_bin" >&2
  echo "$opencv_gui_backend" >&2
  echo "Remove conflicting OpenCV wheels and reinstall the GUI-enabled package:" >&2
  echo "  $python_bin -m pip uninstall -y opencv-python-headless opencv-python" >&2
  echo "  $python_bin -m pip install --upgrade opencv-python numpy pyzmq" >&2
  echo "  $python_bin -m pip install --no-deps -e rb_gui" >&2
  exit 2
fi

# OpenCV wheels with a Qt HighGUI backend occasionally point at a missing
# bundled font directory. Keep a valid operator override; otherwise select a
# known host font directory when one is available.
if [[ "${opencv_gui_backend^^}" == QT* ]] && {
  [[ -z "${QT_QPA_FONTDIR:-}" ]] || [[ ! -d "${QT_QPA_FONTDIR}" ]];
}; then
  for font_dir in \
    /usr/share/fonts/truetype/dejavu \
    /usr/share/fonts/truetype/freefont; do
    if [[ -d "$font_dir" ]]; then
      export QT_QPA_FONTDIR=$font_dir
      break
    fi
  done
fi

export PYTHONPATH="$repo_root/policy_runner${PYTHONPATH:+:$PYTHONPATH}"

echo "[camera-preview] python=$python_bin opencv_gui=$opencv_gui_backend"
echo "[camera-preview] topic=camera.bundle.policy cameras=left_realsense_color,right_realsense_color"
echo "[camera-preview] press q/ESC or close the window to exit"

exec "$python_bin" -m policy_runner.camera_preview \
  --zmq-endpoint "tcp://127.0.0.1:5600" \
  --topic "camera.bundle.policy" \
  --cameras "left_realsense_color,right_realsense_color" \
  --max-age-ms 200 \
  --refresh-hz 30 \
  "$@"
