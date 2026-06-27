#!/usr/bin/env bash
# IR 1280x720(->736 패딩)용 Fast-FoundationStereo TensorRT 엔진 재빌드.
#
# 720은 32의 배수가 아니므로 엔진은 736x1280로 빌드하고, stereo_worker(TrtStereoModel)는
# 캡처 720을 736으로 패딩 후 추론, 출력 disparity를 다시 720으로 crop한다.
#
# GPU + torch + tensorrt 필요 -> camera_server 컨테이너 안에서 실행:
#     make cam-engine-rebuild         (호스트에서: docker exec camera_server ...)
# 또는 컨테이너 셸에서:  bash /app/stereo_worker/rebuild_engine_1280.sh
#
# 재빌드 후 worker가 새 엔진을 로드하려면 컨테이너 재시작(run_all.sh 재기동).
set -euo pipefail

FFS="${FFS_DIR:-/app/Fast-FoundationStereo}"
WEIGHTS="${STEREO_WEIGHTS:-$FFS/weights/23-36-37/model_best_bp2_serialize.pth}"
ENG_DIR="/app/stereo_worker/engines"
OUT="$ENG_DIR/_export_1280"
H="${ENGINE_H:-736}"
W="${ENGINE_W:-1280}"
MAXD="${ENGINE_MAX_DISP:-192}"          # #3(max_disp 상향)은 별도 작업 — 여기선 기존 값 유지
ITERS="${ENGINE_VALID_ITERS:-8}"

echo "[rebuild] weights=$WEIGHTS"
echo "[rebuild] export ONNX ${H}x${W} (max_disp=$MAXD, valid_iters=$ITERS)"
mkdir -p "$OUT"
python3 "$FFS/scripts/make_single_onnx.py" \
  --model_dir "$WEIGHTS" --save_path "$OUT" \
  --height "$H" --width "$W" --max_disp "$MAXD" --valid_iters "$ITERS" \
  --onnx_name fast_foundationstereo

echo "[rebuild] build TRT engine (tf32: fp16은 cost-volume NaN)"
python3 /app/stereo_worker/build_engine.py \
  --onnx "$OUT/fast_foundationstereo.onnx" \
  --engine "$ENG_DIR/fast_foundationstereo.engine" \
  --precision tf32

cp -f "$OUT/fast_foundationstereo.yaml" "$ENG_DIR/fast_foundationstereo.yaml"
echo "[rebuild] done. engine=$ENG_DIR/fast_foundationstereo.engine  image_size=[$H,$W]"
echo "[rebuild] 컨테이너 재시작(docker restart camera_server)으로 새 엔진 반영."
