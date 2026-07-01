#!/usr/bin/env bash
# Run the live OpenPI flow-infer policy as an external command source while
# `make run` keeps rb_servo_server + GUI + teleop_mux + gripper_server alive.
#
# The stack policy_runner owns UDP state port 50376. The flow_real_* configs bind
# 50378, and stack_real/stack_sim fan out state there, so this process can run in
# a separate terminal without ACTION_SOURCE=none.
set -euo pipefail
cd "$(dirname "$0")/.."

PYTHON_BIN="${FLOW_INFER_PYTHON:-${PYTHON:-python3}}"
CHECKPOINT="${FLOW_INFER_CHECKPOINT:-openpi://127.0.0.1:8000}"
CONFIG="${FLOW_INFER_CONFIG:-policy_runner/config/flow_real_realsense.yaml}"
ACTION_HORIZON="${FLOW_INFER_ACTION_HORIZON:-24}"
CHUNK_EXECUTE_STEPS="${FLOW_INFER_CHUNK_EXECUTE_STEPS:-24}"
SPEED_SCALE="${FLOW_INFER_SPEED_SCALE:-2.0}"
CHUNK_CROSSFADE_STEPS="${FLOW_INFER_CHUNK_CROSSFADE_STEPS:-2}"
TCP_REANCHOR_MODE="${FLOW_INFER_TCP_REANCHOR_MODE:-measured_blend}"
TCP_BLEND_STEPS="${FLOW_INFER_TCP_BLEND_STEPS:-8}"
ROLLOUT_SUMMARY="${FLOW_INFER_ROLLOUT_SUMMARY:-outputs/rollout_summary.json}"

mkdir -p "$(dirname "$ROLLOUT_SUMMARY")"
export PYTHONPATH="$PWD/policy_runner${PYTHONPATH:+:$PYTHONPATH}"

echo "[flow-infer] checkpoint=$CHECKPOINT"
echo "[flow-infer] config=$CONFIG"
echo "[flow-infer] rollout_summary=$ROLLOUT_SUMMARY"
echo "[flow-infer] speed_scale=$SPEED_SCALE chunk_execute_steps=$CHUNK_EXECUTE_STEPS crossfade=$CHUNK_CROSSFADE_STEPS reanchor=$TCP_REANCHOR_MODE"
echo "[flow-infer] inherited env: OPENPI_REMOTE_SKIP_WARMUP=${OPENPI_REMOTE_SKIP_WARMUP-<unset>} RB_ALLOW_REAL_GRIPPER=${RB_ALLOW_REAL_GRIPPER-<unset>} DISPLAY=${DISPLAY-<unset>}"

RTC_ARGS=()
if [ "${FLOW_INFER_RTC:-0}" = "1" ]; then
  RTC_ARGS+=(--rtc)
fi

exec "$PYTHON_BIN" -m policy_runner flow-infer \
  --checkpoint "$CHECKPOINT" \
  --config "$CONFIG" \
  --rollout-mode real_policy \
  --action-horizon "$ACTION_HORIZON" \
  --chunk-execute-steps "$CHUNK_EXECUTE_STEPS" \
  --speed-scale "$SPEED_SCALE" \
  --chunk-crossfade-steps "$CHUNK_CROSSFADE_STEPS" \
  --tcp-target-pose-conditioning foh_se3 \
  --tcp-target-pose-reanchor-mode "$TCP_REANCHOR_MODE" \
  --tcp-target-pose-blend-steps "$TCP_BLEND_STEPS" \
  "${RTC_ARGS[@]}" \
  --include-depth \
  --gripper-action-mode absolute \
  --rollout-summary "$ROLLOUT_SUMMARY" \
  "$@"
