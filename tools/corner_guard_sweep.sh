#!/usr/bin/env bash
# Corner-guard parameter sweep on the pgmode controller-simulation stack.
#
# For each (corner_deadband_ang_rad, corner_velocity_scale) pair: patch
# stack_sim.yaml's flow_infer_smooth profile, restart `make run MODE=sim`
# (config is read once at startup), then drive one fixed-duration flow-infer
# rollout with the LIVE camera bundle. Records the servo CSV + rollout JSONL
# that belong to each config so the follower telemetry can be attributed.
#
# No physical motion: stack_sim.yaml holds both control boxes in
# operation_mode=simulation and latches on any real encoder movement
# (controller_simulation_physical_motion_policy: fault_latch, 0.05 deg).
set -uo pipefail
cd "$(dirname "$0")/.."

CFG=rb_servo_server/config/stack_sim.yaml
RUN_SEC="${RUN_SEC:-75}"
OUT=/tmp/corner_sweep
mkdir -p "$OUT"
MANIFEST="$OUT/manifest.tsv"
: > "$MANIFEST"
printf 'tag\tdeadband_ang_rad\tvel_scale\tservo_log\trollout_log\n' >> "$MANIFEST"

patch_cfg() {  # $1=deadband_ang  $2=vel_scale
  python3 - "$CFG" "$1" "$2" <<'PY'
import sys, re, pathlib
p, ang, scale = pathlib.Path(sys.argv[1]), sys.argv[2], sys.argv[3]
t = p.read_text()
t2 = re.sub(r'(^\s*corner_deadband_ang_rad:).*$', r'\1 ' + ang, t, count=1, flags=re.M)
t2 = re.sub(r'(^\s*corner_velocity_scale:).*$', r'\1 ' + scale, t2, count=1, flags=re.M)
assert t2 != t, "no corner keys matched"
p.write_text(t2)
PY
}

stop_stack() {
  pkill -f "tools/run_stack.sh" >/dev/null 2>&1
  pkill -f "rb_servo_server --config" >/dev/null 2>&1
  pkill -f "policy_runner --config" >/dev/null 2>&1
  sleep 4
}

start_stack() {
  GRIPPER_SERVER=0 ./tools/run_stack.sh sim > "$OUT/stack_$1.log" 2>&1 &
  for _ in $(seq 1 40); do
    sleep 1
    grep -q "server up." "$OUT/stack_$1.log" 2>/dev/null && break
  done
  sleep 6
}

run_one() {  # $1=tag
  timeout "$RUN_SEC" env \
    FLOW_INFER_PYTHON=/home/plaif/workspace/openpi/.venv/bin/python \
    FLOW_INFER_CHECKPOINT=openpi://127.0.0.1:8001 FLOW_INFER_INCLUDE_DEPTH=0 \
    FLOW_INFER_CONFIG=policy_runner/config/flow_sim_realsense.yaml \
    FLOW_INFER_ROLLOUT_MODE=controller_sim \
    FLOW_INFER_ACTION_HORIZON=24 FLOW_INFER_STITCH=boundary \
    FLOW_INFER_CHUNK_EXECUTE_STEPS=5 FLOW_INFER_CHUNK_OVERLAY_RUNWAY_STEPS=4 \
    FLOW_INFER_SPEED_SCALE="${SPEED:-1.0}" FLOW_INFER_CHUNK_ANCHOR=command \
    FLOW_INFER_VELPROPRIO_SAMPLE=fixed_step FLOW_INFER_VELPROPRIO_SOURCE=command \
    FLOW_INFER_RTC=1 FLOW_INFER_PREFETCH_AT=2 FLOW_INFER_RTC_SCHEDULE=exp \
    ./tools/flow_infer_sweep_run.sh "$1" --proprio-mode velocity_grip \
    > "$OUT/run_$1.log" 2>&1
}

# tag:deadband_ang_rad:vel_scale
CONFIGS="${CONFIGS:-base:0.0005:0.25 db2:0.002:0.25 db5:0.005:0.25 vs10:0.0005:1.0 db5vs10:0.005:1.0}"

for spec in $CONFIGS; do
  TAG="${spec%%:*}"; rest="${spec#*:}"; ANG="${rest%%:*}"; SCALE="${rest##*:}"
  echo "[sweep] === $TAG  deadband_ang=$ANG  vel_scale=$SCALE ==="
  patch_cfg "$ANG" "$SCALE" || { echo "[sweep] patch failed"; continue; }
  stop_stack
  start_stack "$TAG"
  SV=$(ls -t logs/servo_log_2026*.csv 2>/dev/null | head -1)
  run_one "SW_$TAG"
  RJ=$(ls -t outputs/sweep/*SW_${TAG}.jsonl 2>/dev/null | head -1)
  printf '%s\t%s\t%s\t%s\t%s\n' "$TAG" "$ANG" "$SCALE" "$SV" "$RJ" >> "$MANIFEST"
  echo "[sweep] $TAG -> servo=$SV rollout=$RJ steps=$(wc -l < "${RJ:-/dev/null}" 2>/dev/null || echo 0)"
done

stop_stack
echo "[sweep] DONE"
cat "$MANIFEST"
