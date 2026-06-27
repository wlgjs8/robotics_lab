#!/usr/bin/env bash
# camera_server(C++ 캡처) + stereo_worker(추론/발행)를 한 컨테이너에서 함께 기동.
# compose entrypoint로 사용 -> docker compose up / docker start 한 번에 둘 다 실행.
set -m
CFG="${CAMERA_SERVER_CONFIG:-/app/config/triple_realsense.yaml}"

echo "[run_all] starting camera_server --config $CFG"
/app/build/camera_server --config "$CFG" &
CAM=$!

WK_LOOP=""
if [ "${STEREO_WORKER_AUTOSTART:-1}" != "0" ]; then
  (
    sleep 5                       # 번들 스트림 시작 대기
    while kill -0 "$CAM" 2>/dev/null; do
      echo "[run_all] starting stereo_worker --run"
      python3 /app/stereo_worker/worker.py --run
      echo "[run_all] stereo_worker exited; restart in 3s"
      sleep 3
    done
  ) &
  WK_LOOP=$!
fi

trap 'kill "$CAM" "$WK_LOOP" 2>/dev/null' TERM INT
wait "$CAM"                       # 캡처 종료 시 컨테이너 종료
kill "$WK_LOOP" 2>/dev/null
