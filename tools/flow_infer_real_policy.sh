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
ROLLOUT_MODE="${FLOW_INFER_ROLLOUT_MODE:-real_policy}"
ACTION_HORIZON="${FLOW_INFER_ACTION_HORIZON:-24}"
CHUNK_EXECUTE_STEPS="${FLOW_INFER_CHUNK_EXECUTE_STEPS:-12}"
CHUNK_OVERLAY_RUNWAY_STEPS="${FLOW_INFER_CHUNK_OVERLAY_RUNWAY_STEPS:-4}"
SPEED_SCALE="${FLOW_INFER_SPEED_SCALE:-1.0}"
CHUNK_CROSSFADE_STEPS="${FLOW_INFER_CHUNK_CROSSFADE_STEPS:-2}"
TCP_REANCHOR_MODE="${FLOW_INFER_TCP_REANCHOR_MODE:-measured_blend}"
TCP_BLEND_STEPS="${FLOW_INFER_TCP_BLEND_STEPS:-8}"
ROLLOUT_SUMMARY="${FLOW_INFER_ROLLOUT_SUMMARY:-outputs/rollout_summary.json}"
VELPROPRIO_SOURCE="${FLOW_INFER_VELPROPRIO_SOURCE:-measured}"

mkdir -p "$(dirname "$ROLLOUT_SUMMARY")"
export PYTHONPATH="$PWD/policy_runner${PYTHONPATH:+:$PYTHONPATH}"

# rb_gui predicted action-chunk overlay: publish each inferred chunk's absolute TCP path
# (dots/6DOF triads/'now' cursor/tracking-error) to the GUI's chunk-overlay UDP receiver
# (rb_gui binds udp://0.0.0.0:50262 by default). Telemetry-only; never carries commands.
# Set to "none" or "" to disable, or point at the GUI host if it runs elsewhere.
#
# Fan-out (comma/space-separated): 50262 = rb_gui viser overlay, 50263 = scope
# dashboard (servo_scope_dashboard.py chunk view on :8081), 50264 =
# rb_servo_server network.chunk_frame_bind — the SAME packet feeds the server's
# Ruckig chunk-follower (cartesian_control profile ruckig_follower.enable) as its
# whole-chunk reference input. Removing 50264 (or disabling the overlay) simply
# leaves the server on the legacy pose_track_smd path — fail-safe by construction.
export RB_GUI_CHUNK_OVERLAY_ENDPOINT="${RB_GUI_CHUNK_OVERLAY_ENDPOINT:-udp://127.0.0.1:50262,udp://127.0.0.1:50263,udp://127.0.0.1:50264}"

# Print each freshly-inferred action chunk to the terminal, step-by-step, for both
# arms (position deltas in meters, rotation deltas in degrees). Debug-only; set
# FLOW_INFER_PRINT_CHUNK=1 to enable.
export FLOW_INFER_PRINT_CHUNK="${FLOW_INFER_PRINT_CHUNK:-0}"
# Per-step chunk-vs-actual tracking: as each step executes, compare the model's
# predicted absolute pose against the live measured pose (tracking error in mm/deg)
# plus predicted vs actual per-step displacement, time-stamped from chunk activation.
# Use this to see how well the controller follows the chunk. Set to 1 to enable.
export FLOW_INFER_PRINT_TRACKING="${FLOW_INFER_PRINT_TRACKING:-0}"

echo "[flow-infer] checkpoint=$CHECKPOINT"
echo "[flow-infer] chunk_overlay_endpoint=$RB_GUI_CHUNK_OVERLAY_ENDPOINT (rb_gui '예측 chunk 궤적 표시')"
echo "[flow-infer] config=$CONFIG"
echo "[flow-infer] rollout_mode=$ROLLOUT_MODE"
echo "[flow-infer] rollout_summary=$ROLLOUT_SUMMARY"
echo "[flow-infer] speed_scale=$SPEED_SCALE chunk_execute_steps=$CHUNK_EXECUTE_STEPS overlay_runway_steps=$CHUNK_OVERLAY_RUNWAY_STEPS crossfade=$CHUNK_CROSSFADE_STEPS reanchor=$TCP_REANCHOR_MODE"
echo "[flow-infer] inherited env: OPENPI_REMOTE_SKIP_WARMUP=${OPENPI_REMOTE_SKIP_WARMUP-<unset>} RB_ALLOW_REAL_GRIPPER=${RB_ALLOW_REAL_GRIPPER-<unset>} DISPLAY=${DISPLAY-<unset>}"

RTC_ARGS=()
if [ "${FLOW_INFER_RTC:-0}" = "1" ]; then
  RTC_ARGS+=(--rtc)
  # RTC replan overlap: kick the next inference at PREFETCH_AT consumed steps;
  # the remaining (EXECUTE - PREFETCH_AT) steps run on the OLD plan while the
  # server freezes exactly that prefix of the new chunk and inpaints the rest.
  # Default: half-window kick (e.g. 16 executed -> kick at 8, freeze 8).
  RTC_PREFETCH_AT="${FLOW_INFER_PREFETCH_AT:-$((CHUNK_EXECUTE_STEPS / 2))}"
  RTC_DELAY="${FLOW_INFER_RTC_DELAY:-$((CHUNK_EXECUTE_STEPS - RTC_PREFETCH_AT))}"
  # Old->new overlap mixing shape over the soft window [d .. H-execute):
  #   exp (paper default) = convex ramp, linear = plain 1->0 lerp of the
  #   guidance weight, zeros = hard freeze only (no mixing).
  # Blend window width = H - 2*execute + kick_at (0 = no soft zone).
  RTC_SCHEDULE="${FLOW_INFER_RTC_SCHEDULE:-exp}"
  RTC_ARGS+=(--stream-prefetch-at "$RTC_PREFETCH_AT" --rtc-inference-delay "$RTC_DELAY" --rtc-schedule "$RTC_SCHEDULE")
  echo "[flow-infer] RTC: execute=$CHUNK_EXECUTE_STEPS kick_at=$RTC_PREFETCH_AT frozen_prefix(d)=$RTC_DELAY schedule=$RTC_SCHEDULE"
fi

# Sequential (blocking) chunk verification mode — FLOW_INFER_SEQUENTIAL=1:
# consume the FULL chunk, only then infer (robot holds still for the inference
# latency; no mid-chunk prefetch), and re-anchor the next chunk's deltas to the
# MEASURED pose at activation. Unless explicitly overridden via env, this also
# switches crossfade -> 0 and reanchor -> measured_legacy (pure step semantics).
SEQ_ARGS=()
if [ "${FLOW_INFER_SEQUENTIAL:-0}" = "1" ]; then
  SEQ_ARGS+=(--sequential-chunk-inference)
  [ -z "${FLOW_INFER_CHUNK_CROSSFADE_STEPS:-}" ] && CHUNK_CROSSFADE_STEPS=0
  [ -z "${FLOW_INFER_TCP_REANCHOR_MODE:-}" ] && TCP_REANCHOR_MODE=measured_legacy
  echo "[flow-infer] SEQUENTIAL mode: full-chunk consume -> hold during inference -> measured re-anchor (crossfade=$CHUNK_CROSSFADE_STEPS reanchor=$TCP_REANCHOR_MODE)"
fi

# Chunk-delta integration anchor (FLOW_INFER_CHUNK_ANCHOR):
#   actual  (legacy): measured FK(q_actual) — boundary shortfall is DISCARDED each
#           chunk (unconverged robot -> asymptotic undershoot / Zeno pattern).
#   command: FK(q_sent) — removes servo-lag re-compensation; shortfall vs the
#           sent pose still discarded.
#   chain  : pure plan-chain — each chunk integrates from the PREVIOUS chunk's
#           integrated tail (no robot-state re-anchor at all). Shortfall carries
#           over (lag, never loss). Also switches the per-tick command path to
#           reanchor_mode=last_emitted_continuous unless explicitly overridden.
CHUNK_ANCHOR="${FLOW_INFER_CHUNK_ANCHOR:-actual}"
if [ "$CHUNK_ANCHOR" != "actual" ]; then
  SEQ_ARGS+=(--chunk-anchor-source "$CHUNK_ANCHOR")
  if [ "$CHUNK_ANCHOR" = "chain" ] && [ -z "${FLOW_INFER_TCP_REANCHOR_MODE:-}" ]; then
    TCP_REANCHOR_MODE=last_emitted_continuous
  fi
  echo "[flow-infer] chunk_anchor_source=$CHUNK_ANCHOR (reanchor=$TCP_REANCHOR_MODE)"
fi

# Chunk stitch mode (FLOW_INFER_STITCH):
#   boundary (default): whole-chunk swap at the execute boundary (RTC-inpaintable).
#   ensemble: observation-aligned recursive 2-chunk blend — kick every R
#             (FLOW_INFER_ENSEMBLE_PERIOD, default 6) steps; each executed
#             R-window = lerp(old[2R..3R), new[R..2R)) time-aligned. Requires
#             H >= 3R. Inference cadence doubles vs boundary mode (budget = R
#             steps ~ R*33ms) — check PRINT_CHUNK latency first. RTC optional
#             on top (FLOW_INFER_RTC=1 adds flow-level consistency inpainting).
STITCH_MODE="${FLOW_INFER_STITCH:-boundary}"
if [ "$STITCH_MODE" != "boundary" ]; then
  ENSEMBLE_PERIOD="${FLOW_INFER_ENSEMBLE_PERIOD:-6}"
  # FLOW_INFER_ENSEMBLE_BLEND: linear (lerp old/new over the window) | none
  # (execute the newest chunk's [R..2R) pure; old plan = late runway only).
  ENSEMBLE_BLEND="${FLOW_INFER_ENSEMBLE_BLEND:-linear}"
  SEQ_ARGS+=(--chunk-stitch-mode "$STITCH_MODE" --ensemble-period "$ENSEMBLE_PERIOD" --ensemble-blend "$ENSEMBLE_BLEND")
  echo "[flow-infer] stitch=ensemble R=$ENSEMBLE_PERIOD blend=$ENSEMBLE_BLEND (kick ${ENSEMBLE_PERIOD}-step cadence)"
fi

# Velocity-proprio finite-difference window (only affects --proprio-mode velocity*):
#   replan     (default): difference successive replan-boundary samples, rescaled by
#              policy_dt/wall_dt. The window + wall-clock depend on controller/inference
#              timing, so a slower/burstier controller (or SEQUENTIAL holds) under-reports
#              velocity -> the policy reads "slowing down" and under-shoots (e.g. depth).
#   fixed_step: difference the measured pose over a fixed ~policy_dt window from a per-tick
#              pose history, decoupled from replan cadence + inference latency (legacy
#              normalization preserved).
#   camera_frame: measured TCP local delta over [camera_time - policy_dt, camera_time],
#              no dt normalization, closest to OpenPI/UMI converter semantics.
# Override with FLOW_INFER_VELPROPRIO_SAMPLE=replan|fixed_step|camera_frame.
VELPROPRIO_SAMPLE="${FLOW_INFER_VELPROPRIO_SAMPLE:-camera_frame}"
VELPROPRIO_ARGS=()
PROPRIO_MODE_FROM_ARGS=""
PREV_WAS_PROPRIO_MODE=0
for ARG in "$@"; do
  if [ "$PREV_WAS_PROPRIO_MODE" = "1" ]; then
    PROPRIO_MODE_FROM_ARGS="$ARG"
    PREV_WAS_PROPRIO_MODE=0
    continue
  fi
  case "$ARG" in
    --proprio-mode)
      PREV_WAS_PROPRIO_MODE=1
      ;;
    --proprio-mode=*)
      PROPRIO_MODE_FROM_ARGS="${ARG#--proprio-mode=}"
      ;;
  esac
done
if [[ "$PROPRIO_MODE_FROM_ARGS" == velocity* ]]; then
  if [ "$VELPROPRIO_SAMPLE" != "replan" ]; then
    VELPROPRIO_ARGS+=(--velproprio-sample-mode "$VELPROPRIO_SAMPLE")
    if [ "$VELPROPRIO_SAMPLE" = "camera_frame" ]; then
      echo "[flow-infer] velproprio_sample_mode=$VELPROPRIO_SAMPLE (camera-time measured TCP local delta; no dt normalization)"
    else
      echo "[flow-infer] velproprio_sample_mode=$VELPROPRIO_SAMPLE (velocity from fixed ~policy_dt window; controller-independent)"
    fi
  fi
  VELPROPRIO_ARGS+=(--velproprio-source "$VELPROPRIO_SOURCE")
  echo "[flow-infer] velproprio_source=$VELPROPRIO_SOURCE"
fi

exec "$PYTHON_BIN" -m policy_runner flow-infer \
  --checkpoint "$CHECKPOINT" \
  --config "$CONFIG" \
  --rollout-mode "$ROLLOUT_MODE" \
  --action-horizon "$ACTION_HORIZON" \
  --chunk-execute-steps "$CHUNK_EXECUTE_STEPS" \
  --chunk-overlay-runway-steps "$CHUNK_OVERLAY_RUNWAY_STEPS" \
  --speed-scale "$SPEED_SCALE" \
  --chunk-crossfade-steps "$CHUNK_CROSSFADE_STEPS" \
  --tcp-target-pose-conditioning foh_se3 \
  --tcp-target-pose-reanchor-mode "$TCP_REANCHOR_MODE" \
  --tcp-target-pose-blend-steps "$TCP_BLEND_STEPS" \
  "${RTC_ARGS[@]}" \
  "${SEQ_ARGS[@]}" \
  "${VELPROPRIO_ARGS[@]}" \
  --include-depth \
  --gripper-action-mode absolute \
  --rollout-summary "$ROLLOUT_SUMMARY" \
  "$@"
