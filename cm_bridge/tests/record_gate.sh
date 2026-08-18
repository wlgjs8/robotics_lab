#!/usr/bin/env bash
# record_gate.sh — the RECORDER gate, run ISOLATED beside a live stack.
#
#   ./cm_bridge/tests/record_gate.sh            # isolated (ROS domain 77, alt UDP ports, no-affinity shim)
#
# What it proves: a controller-manager `func write` capture (DataRecorder schema 4) and the
# cm_bridge sidecar describe ONE follow episode on ONE clock — the chunk the follower is playing
# joins to the sidecar's chunk record by stamp, and the gripper command level rides the 2 ms row.
#
# ISOLATION (why every knob below exists): the live controller may be running on this host with
# energized arms. This gate therefore starts its own SILS controller in a SEPARATE ROS domain,
# with a separate console socket and log dir, and — the important one — under an LD_PRELOAD shim
# that turns pthread_setaffinity_np into a no-op (cm_bridge/tests/noaffinity.c), so the test
# instance's 500 Hz loops never land on the live loops' isolated cores (cpu1/cpu2). The test
# bridge binds alternate UDP ports and points its gripper output at a dead port, so nothing here
# reaches the real gripper_server, the real bridge, or the real boxes.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CM="$ROOT/submodules/controller-manager"
SP="${RECORD_GATE_DIR:-$ROOT/logs/record_gate_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$SP"
DOMAIN="${RECORD_GATE_DOMAIN:-77}"
CHUNK=60264; CMD=60256; CTRL=60259; STATE=60378
PLATFORM=monkey
BIN="$CM/install/controller_manager/lib/controller_manager/plaif_chimpanzee"

say() { echo "[record-gate] $*"; }
die() { echo "[record-gate] FATAL: $*" >&2; cleanup; exit 1; }
PIDS=()
cleanup() {
  for p in "${PIDS[@]:-}"; do [ -n "$p" ] && kill -INT "$p" 2>/dev/null || true; done
  sleep 1
  for p in "${PIDS[@]:-}"; do [ -n "$p" ] && kill -TERM "$p" 2>/dev/null || true; done
}
trap cleanup EXIT

[ -x "$BIN" ] || die "controller not built: $BIN"

# 1. the shim
gcc -shared -fPIC -o "$SP/noaffinity.so" "$ROOT/cm_bridge/tests/noaffinity.c" || die "shim build failed"

# 2. an isolated SILS platform dir sharing OUR params (same links as config/monkey-sils)
PDIR="$SP/monkey-sils"; mkdir -p "$PDIR"
ln -sfn "$ROOT/cm_bridge/config/monkey/params-tasks"   "$PDIR/params-tasks"
ln -sfn "$ROOT/cm_bridge/config/monkey/params-presets" "$PDIR/params-presets"
sed -e '0,/ip: ""/s//ip: "127.0.0.1"/' -e '0,/ip: ""/s//ip: "127.0.0.2"/' \
    -e 's/^\(\s*tool:\s*\)none/\1pika/' \
    "$CM/platforms/$PLATFORM/params-presets/dual-arm.yaml" > "$PDIR/active.yaml"

# 3. the controller (SILS, isolated)
export ROS_DOMAIN_ID="$DOMAIN"
export CONTROL_MANAGER_ACTIVE_YAML="$PDIR/active.yaml"
export CONTROL_MANAGER_PLATFORM="$PLATFORM"
export CONTROL_MANAGER_LOG_DIR="$SP/cmlogs"
export CHIMPANZEE_CONSOLE_SOCK="/tmp/${PLATFORM}_record_gate_console.sock"
export CHIMPANZEE_SILS=1 CONTROL_MANAGER_SILS=1
set +u; source "$CM/platforms/$PLATFORM/scripts/env.sh" >/dev/null 2>&1; set -u
# env.sh may have re-exported ACTIVE_YAML from our pre-set (it honours it) - re-assert the rest
export CONTROL_MANAGER_LOG_DIR="$SP/cmlogs" CHIMPANZEE_SILS=1 CONTROL_MANAGER_SILS=1 ROS_DOMAIN_ID="$DOMAIN"
# The test instance is unprivileged SCHED_OTHER (SCHED_FIFO is refused without CAP_SYS_NICE) and,
# with the shim, runs on the general cores - so keep it on the QUIETEST few (TEST_CPUS) rather
# than the whole machine, or the RtJitter guard (>= 2 ms) trips the SILS enable under load.
# (taskset OUTSIDE the LD_PRELOAD scope: the shim would otherwise no-op taskset's own
# sched_setaffinity call and the mask would silently stay the default.)
taskset -c "${TEST_CPUS:-12-15}" env LD_PRELOAD="$SP/noaffinity.so" "$BIN" > "$SP/controller.log" 2>&1 &
PIDS+=("$!")
for i in $(seq 1 60); do grep -aq "TASKCFG.*follow{" "$SP/controller.log" && break; sleep 0.5; done
grep -aq "TASKCFG.*follow{" "$SP/controller.log" || { tail -20 "$SP/controller.log"; die "controller did not boot"; }
say "controller (SILS, domain $DOMAIN) up: $(grep -a 'TASKCFG.*follow{' "$SP/controller.log" | head -1 | sed 's/.*follow{/follow{/' | cut -c1-90)"
say "affinity: $(for t in $(ls /proc/${PIDS[0]}/task); do awk '{print $39}' /proc/${PIDS[0]}/task/$t/stat; done | sort -n | uniq | tr '\n' ' ')  (must not include 1 2)"

# 4. the bridge (alt ports, dead gripper endpoint, sidecar in SP)
SIDECAR="$SP/sidecar.jsonl"
/usr/bin/python3 -X faulthandler "$ROOT/cm_bridge/src/cm_bridge_node.py" --platform "$PLATFORM" \
    --chunk-bind "127.0.0.1:$CHUNK" --command-bind "127.0.0.1:$CMD" --control-bind "127.0.0.1:$CTRL" \
    --state-endpoints "127.0.0.1:$STATE" --gripper-cmd-endpoint 127.0.0.1:60410 \
    --gripper-fb-bind 127.0.0.1:60420 --sidecar "$SIDECAR" > "$SP/bridge.log" 2>&1 &
PIDS+=("$!")
sleep 3
grep -q "sidecar ->" "$SP/bridge.log" || { tail -5 "$SP/bridge.log"; die "bridge did not start"; }

# 5. energize (SILS) + idle + start the recorder on both arms
C() { ros2 service call "/$PLATFORM/$1/cmd/console" cell_msgs/srv/Command "{command: '$2'}" 2>&1 | grep -o "accepted=[A-Za-z]*" | head -1; }
say "enable: $(C cell enable)"; sleep 2.5
say "task on: $(C cell 'task on')"; sleep 2.5
say "left idle: $(C left 'task idle')  right idle: $(C right 'task idle')"; sleep 1
say "func write start: left $(C left 'func write start') right $(C right 'func write start')"
sleep 0.5

# 6. stream a synthetic episode through the bridge
/usr/bin/python3 "$ROOT/cm_bridge/tests/cm_record_gate.py" stream \
    --chunk-port "$CHUNK" --cmd-port "$CMD" --state-port "$STATE" --seconds 6 \
    --obs-dump "$SP/obs_dump" --plan-out "$SP/plan.json" ${RECORD_GATE_LATE_EVERY:+--late-every $RECORD_GATE_LATE_EVERY} || die "stream failed"
sleep 3.5   # the stretched tail (up to ~2 s of remaining sub-deltas) + silence exit + settle

# 7. stop the recorder, then the stack
say "func write stop: left $(C left 'func write stop') right $(C right 'func write stop')"
sleep 1.5
kill -INT "${PIDS[1]}" 2>/dev/null || true       # bridge
kill -INT "${PIDS[0]}" 2>/dev/null || true       # controller (clean shutdown)
for i in $(seq 1 30); do kill -0 "${PIDS[0]}" 2>/dev/null || break; sleep 0.5; done
PIDS=()
BINS=$(ls "$SP"/cmlogs/data_*.bin 2>/dev/null || true)
[ -n "$BINS" ] || { grep -a "func\|write\|record" "$SP/controller.log" | tail -5; die "no data_*.bin under $SP/cmlogs"; }
say "captures: $(echo $BINS | tr '\n' ' ')"

# 8. check
/usr/bin/python3 "$ROOT/cm_bridge/tests/cm_record_gate.py" check --bin $BINS --sidecar "$SIDECAR" --obs-dump "$SP/obs_dump" --plan "$SP/plan.json"
rc=$?
# 9. the replay tool must load this capture (headless: scene + every GUI path, incl. the obs panel)
if [ "$rc" = 0 ] && [ -x "$ROOT/.venv/bin/python" ]; then
  if "$ROOT/.venv/bin/python" "$ROOT/cm_bridge/tools/cm_replay.py" --bin $BINS --sidecar "$SIDECAR" \
        --obs-dump "$SP/obs_dump" --port 8093 --headless-check > "$SP/replay_check.log" 2>&1; then
    say "replay: $(grep -o 'headless check OK.*' "$SP/replay_check.log")"
  else
    tail -8 "$SP/replay_check.log"; say "replay headless check FAILED"; rc=1
  fi
fi
say "artifacts: $SP"
exit $rc
