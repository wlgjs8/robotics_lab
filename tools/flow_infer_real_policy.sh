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
TCP_BLEND_STEPS="${FLOW_INFER_TCP_BLEND_STEPS:-2}"
ROLLOUT_SUMMARY="${FLOW_INFER_ROLLOUT_SUMMARY:-outputs/rollout_summary.json}"
VELPROPRIO_SOURCE="${FLOW_INFER_VELPROPRIO_SOURCE:-measured}"
# Which gripper signal the model's proprio channel carries: actual (DEFAULT, the
# measured jaw = pre-2026-08-19 behaviour) | command | hybrid. Sweep it with
# FLOW_INFER_GRIPPER_PROPRIO_SOURCE so the .meta sidecar records which arm ran.
GRIPPER_PROPRIO_SOURCE="${FLOW_INFER_GRIPPER_PROPRIO_SOURCE:-actual}"
STEP_LOG="${FLOW_INFER_STEP_LOG:-}"
# Wall-clock duration of ONE action-chunk row. MUST equal the training converter's
# --action-step-frames / dataset fps: 1/30 = 0.0334 for the legacy per-frame-delta checkpoints,
# 3/30 = 0.1 for a K=3 decimated one. Getting it wrong is SILENT and severe -- the chunk is replayed
# at the wrong rate AND the client per-axis clamp (v_max * policy_dt) truncates the action tails.
# Unset -> the runner's own default (0.0334); set it explicitly for any non-30Hz-row checkpoint.
# NOTE: this same value is also the velocity-proprio finite-difference window (openpi_remote
# _arm_body_velocities_camera_frame differences the measured pose over policy_dt), which is correct
# because the converter builds proprio over the same K-frame window.
POLICY_DT="${FLOW_INFER_POLICY_DT:-}"
# Whether to send live D405 depth as observation/*_wrist_0_depth. MUST match the served
# checkpoint's training: the RGB-D converters (depth_z50*) need it, the wrist-RGB-only ones
# (pi05_pika_umi_wrist_*_h24_80k, which drop depth AND base_0_rgb to cut image tokens 1280->512)
# must NOT get it. This used to be hardcoded on, and `--include-depth` is a bare store_true with
# no `--no-` counterpart, so an RGB-only checkpoint could not be driven from this script at all.
# Default stays 1 so existing depth_z50 commands are unchanged; set 0 for the wrist-only models.
INCLUDE_DEPTH="${FLOW_INFER_INCLUDE_DEPTH:-1}"
DEPTH_ARGS=()
case "$INCLUDE_DEPTH" in
  1|true|yes|on)  DEPTH_ARGS=(--include-depth) ;;
  0|false|no|off) DEPTH_ARGS=() ;;
  *) echo "FLOW_INFER_INCLUDE_DEPTH must be 0/1 (got '$INCLUDE_DEPTH')" >&2; exit 2 ;;
esac
POLICY_DT_ARGS=()
if [ -n "$POLICY_DT" ]; then
  POLICY_DT_ARGS=(--policy-dt-sec "$POLICY_DT")
fi

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
STEP_LOG_ARGS=()
if [ -n "$STEP_LOG" ]; then
  STEP_LOG_ARGS+=(--rollout-step-log "$STEP_LOG")
  echo "[flow-infer] rollout_step_log=$STEP_LOG"
fi
echo "[flow-infer] speed_scale=$SPEED_SCALE chunk_execute_steps=$CHUNK_EXECUTE_STEPS overlay_runway_steps=$CHUNK_OVERLAY_RUNWAY_STEPS crossfade=$CHUNK_CROSSFADE_STEPS reanchor=$TCP_REANCHOR_MODE"
echo "[flow-infer] policy_dt_sec=${POLICY_DT:-<runner default 0.0334>} (must match the checkpoint's action-step-frames/fps)"
echo "[flow-infer] include_depth=${INCLUDE_DEPTH} -> args:${DEPTH_ARGS[*]:-<none, RGB-only>} (must match the served checkpoint's training)"
echo "[flow-infer] inherited env: OPENPI_REMOTE_SKIP_WARMUP=${OPENPI_REMOTE_SKIP_WARMUP-<unset>} RB_ALLOW_REAL_GRIPPER=${RB_ALLOW_REAL_GRIPPER-<unset>} DISPLAY=${DISPLAY-<unset>}"

# Next-chunk inference kick point, in consumed steps of the execute window.
# FLOW_INFER_PREFETCH_AT applies with OR without RTC: the budget it buys
# (EXECUTE - kick_at) * policy_dt is what decides whether a boundary stalls, and
# that arithmetic does not care about RTC. Unset -> the runner's own default
# (kick at 1 for execute<=4, else early at 2); see _stream_prefetch_at.
PREFETCH_ARGS=()
if [ -n "${FLOW_INFER_PREFETCH_AT:-}" ] && [ "${FLOW_INFER_RTC:-0}" != "1" ]; then
  PREFETCH_ARGS+=(--stream-prefetch-at "$FLOW_INFER_PREFETCH_AT")
  echo "[flow-infer] prefetch kick_at=$FLOW_INFER_PREFETCH_AT (budget $((CHUNK_EXECUTE_STEPS - FLOW_INFER_PREFETCH_AT)) steps before the boundary)"
fi

RTC_ARGS=()
if [ "${FLOW_INFER_RTC:-0}" = "1" ]; then
  RTC_ARGS+=(--rtc)
  # RTC replan overlap: kick the next inference at PREFETCH_AT consumed steps;
  # the remaining (EXECUTE - PREFETCH_AT) steps run on the OLD plan while the
  # server freezes exactly that prefix of the new chunk and inpaints the rest.
  # The inference must FINISH inside (EXECUTE - PREFETCH_AT) * policy_dt or every
  # chunk boundary stalls: with execute=4 the half-window default (kick at 2)
  # leaves 2*33.4 = 66.8 ms, but the pi05 8001 server measures 88-91 ms p50/p90
  # with RTC guidance on (60 ms with RTC off), so 79% of boundaries stalled,
  # chunks arrived every 159 ms instead of 133.6 (19% time dilation) and the
  # runner logged "inference_delay mismatch: realized=1 vs configured 2"
  # (servo_log_20260819_085727 / sweep 085739_LEC0_e4). Default therefore:
  # kick at 1 for execute<=4 (100 ms budget, frozen prefix 3 = 100 ms structural
  # delay), half-window otherwise (e.g. 16 executed -> kick at 8, freeze 8).
  # Override with FLOW_INFER_PREFETCH_AT (0 = infer at the boundary, freeze all).
  if [ -n "${FLOW_INFER_PREFETCH_AT:-}" ]; then
    RTC_PREFETCH_AT="$FLOW_INFER_PREFETCH_AT"
  elif [ "$CHUNK_EXECUTE_STEPS" -le 4 ]; then
    RTC_PREFETCH_AT=1
  else
    RTC_PREFETCH_AT=$((CHUNK_EXECUTE_STEPS / 2))
  fi
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
# Action encoding of the served checkpoint (FLOW_INFER_ACTION_MODE):
#   delta    (default): rows are per-step ee_local deltas, chained downstream.
#   anchored : rows are chunk-start(t0)-anchored transforms (UMI PD2.1 -- the
#              *_anchored_* checkpoints). Converted to per-step deltas at reception;
#              the RTC freeze is re-anchored to the executed boundary.
ACTION_MODE="${FLOW_INFER_ACTION_MODE:-delta}"
if [ "$ACTION_MODE" != "delta" ]; then
  SEQ_ARGS+=(--action-mode "$ACTION_MODE")
  echo "[flow-infer] action_mode=$ACTION_MODE (anchored rows -> per-step deltas at reception)"
  # anchored + RTC needs the checkpoint's action norm stats for the prev-chunk re-anchor
  # (FLOW_INFER_RTC_NORM_STATS=path/to/norm_stats.json); without it RTC degrades to vanilla.
  if [ -n "${FLOW_INFER_RTC_NORM_STATS:-}" ]; then
    SEQ_ARGS+=(--rtc-norm-stats "$FLOW_INFER_RTC_NORM_STATS")
    echo "[flow-infer] rtc_norm_stats=$FLOW_INFER_RTC_NORM_STATS"
  fi
fi

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
  "${POLICY_DT_ARGS[@]}" \
  --chunk-crossfade-steps "$CHUNK_CROSSFADE_STEPS" \
  --tcp-target-pose-conditioning foh_se3 \
  --tcp-target-pose-reanchor-mode "$TCP_REANCHOR_MODE" \
  --tcp-target-pose-blend-steps "$TCP_BLEND_STEPS" \
  "${RTC_ARGS[@]}" \
  "${PREFETCH_ARGS[@]}" \
  "${SEQ_ARGS[@]}" \
  "${VELPROPRIO_ARGS[@]}" \
  "${DEPTH_ARGS[@]}" \
  --gripper-action-mode absolute \
  --gripper-proprio-source "$GRIPPER_PROPRIO_SOURCE" \
  --rollout-summary "$ROLLOUT_SUMMARY" \
  "${STEP_LOG_ARGS[@]}" \
  "$@"
