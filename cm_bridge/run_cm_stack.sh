#!/usr/bin/env bash
# cm_bridge stack launcher — NATIVE controller-manager. NO DOCKER.
#
#   ./cm_bridge/run_cm_stack.sh sim     monkey platform in SILS (no hardware, no box sockets)
#   ./cm_bridge/run_cm_stack.sh real    REAL boxes — operator-supervised, docs/real_bringup.md
#   ./cm_bridge/run_cm_stack.sh gate    SILS + bridge + cm_sils_gate.py (submodule pin-bump gate)
#   ./cm_bridge/run_cm_stack.sh down    stop everything this script started
#   ./cm_bridge/run_cm_stack.sh status
#
# The operator cockpit (upstream's web UI, http://127.0.0.1:8770) comes up with sim/real by
# default; COCKPIT=0 disables it, COCKPIT_PORT moves it. `gate` is always headless.
#
# THE DOCKER PATH WAS RETIRED 2026-08-18 (operator decision). What it bought was a Humble
# userland on a jammy host; tools/cm_local_setup.sh now builds that natively (the submodule's own
# docker/Dockerfile builds this same tree on jammy/Humble, and its ADR 0001 records that
# Humble<->Jazzy needed zero code changes). What it COST is why it went:
#   * the RT pins are hardcoded in the submodule (src/arm/Arm.cpp, Right ? 2 : 1) and SCHED_FIFO
#     through a container adds a layer between the pin and the core (tools/rt_core_isolation.sh
#     isolates cpu1/2/3 on the host for exactly those threads);
#   * every task/tool override rode a SINGLE-FILE bind mount, and rename-on-save editors replace
#     the inode while the container keeps reading the old content — that cost the 2026-08-17
#     zero-compliance day;
#   * container-written artifacts landed root-owned in the submodule tree.
#
# HOW THE OVERRIDES WORK NOW — no bind mounts, no submodule edits. controller-manager resolves
# BOTH its task params and its device presets relative to the LOADED active.yaml's own directory:
#   params-tasks    TaskConfig.cpp:44-50  <active.yaml dir>/params-tasks   (checked FIRST)
#   params-presets  Config.cpp:496,363-365  dir = active.yaml's parent
# and env.sh honours a pre-set CONTROL_MANAGER_ACTIVE_YAML. So cm_bridge/config/monkey/ IS our
# platform directory: our follow.yaml and our measured pika preset are real files, everything we
# do not override is a symlink to the submodule (no copies -> no drift on pull).
set -euo pipefail

MODE="${1:-sim}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CM="$ROOT/submodules/controller-manager"
START="$CM/platforms/monkey/scripts/start.sh"
PLATFORM=monkey
TEMPLATE="$ROOT/cm_bridge/config/active.monkey.real.yaml"
SEED="$CM/platforms/monkey/params-presets/dual-arm.yaml"
RUN="$ROOT/logs/.cm-run"
STAMP="$(date +%Y%m%d_%H%M%S)"

case "$MODE" in
  sim|gate) PDIR="$ROOT/cm_bridge/config/monkey-sils" ;;
  *)        PDIR="$ROOT/cm_bridge/config/monkey" ;;
esac
DEV="$PDIR/active.yaml"

die() { echo "[cm-stack] FATAL: $*" >&2; exit 1; }
say() { echo "[cm-stack] $*"; }

# ---------------------------------------------------------------------------
# Preflight. Every check below is FAIL-CLOSED, and the params checks exist
# because controller-manager treats a MISSING task-param file as non-fatal: it
# runs the compiled-in defaults with a WARN. A silent fallback to a different
# motion envelope is exactly what this repo forbids, so a params directory that
# does not fully resolve stops the launch here instead.
# ---------------------------------------------------------------------------
CM_BIN="$CM/install/controller_manager/lib/controller_manager/plaif_chimpanzee"
preflight_common() {
  pgrep -f "rb_servo_server.*--config" >/dev/null 2>&1 &&
    die "rb_servo_server is running. Stop it first (single controller owner)."
  [ -f "$CM/install/setup.bash" ] ||
    die "controller-manager is not built. Run: tools/cm_local_setup.sh"
  [ -x "$START" ] || die "missing $START (submodule not checked out?)"
  # The recorder schema this binary writes. cm_bridge/upstream/0001-*.patch adds schema 4
  # (follow observability + ext_scalars) and it lives in the submodule WORKING TREE until it is
  # upstream - a fresh submodule checkout silently drops it, and a capture from that binary lacks
  # every column the replay tool is for. The column name is a string literal in the binary.
  if [ -x "$CM_BIN" ] && ! grep -aq "fol_chunk_stamp_ns" "$CM_BIN"; then
    die "the built controller lacks recorder schema 4 (no fol_chunk_stamp_ns in $CM_BIN).
  Apply cm_bridge/upstream/0001-*.patch to the submodule and rebuild (see cm_bridge/upstream/README.md)."
  fi
  # The bridge's default follow mode (commit) is PACED BY THE CONTROLLER's act/follow_step events and
  # fires the gripper from them - a controller without that endpoint (cm_bridge/upstream/0002-*.patch)
  # would leave the stream degraded to replace mode and the gripper never commanded. Fail closed.
  if [ -x "$CM_BIN" ] && ! grep -aq "act/follow_step" "$CM_BIN"; then
    die "the built controller lacks the follow step events / commit_steps (no act/follow_step in $CM_BIN).
  Apply cm_bridge/upstream/0002-*.patch to the submodule and rebuild (see cm_bridge/upstream/README.md)."
  fi
  # RT scheduling. The docker path granted SYS_NICE; a native unprivileged binary gets SCHED_FIFO
  # REFUSED and runs SCHED_OTHER on the isolated cores (Rt.cpp warns per thread). Not fatal here -
  # the isolated cores keep it deterministic enough to run - but it IS a regression to fix once:
  if ! getcap "$CM_BIN" 2>/dev/null | grep -q cap_sys_nice && [ "$(ulimit -r 2>/dev/null || echo 0)" = 0 ]; then
    echo "[cm-stack] WARN: $CM_BIN has no CAP_SYS_NICE and rtprio ulimit is 0 -> the RT threads will run" >&2
    echo "[cm-stack]       SCHED_OTHER (see [RT] SCHED_FIFO refused in the log). Fix once per build:" >&2
    echo "[cm-stack]         sudo setcap cap_sys_nice+ep $CM_BIN" >&2
  fi
}

# Every task file CM opens (TaskConfig.cpp). A missing one = compiled defaults, silently.
check_params_tasks() {
  local f p
  for f in admittance follow idle movd movj movl movp; do
    p="$PDIR/params-tasks/$f.yaml"
    [ -e "$p" ] || die "params-tasks/$f.yaml does not resolve under $PDIR
  (a missing task file makes controller-manager run COMPILED DEFAULTS with only a WARN)"
    [ -s "$p" ] || die "params-tasks/$f.yaml resolves but is EMPTY: $p"
  done
}

# The presets the DEVICE FILE ITSELF names — read them out rather than assuming, so a device-file
# edit cannot outrun this check. The non-empty test is not paranoia: the submodule shipped a
# 0-byte tools/pika.yaml, which under the old bind-mount was invisible and natively would have
# booted with no TCP offset and no payload.
check_params_presets() {
  local kind key dir name p
  for kind in "model models" "tool tools" "eft efts"; do
    key="${kind%% *}"; dir="${kind##* }"
    while read -r name; do
      [ -n "$name" ] || continue
      p="$PDIR/params-presets/$dir/$name.yaml"
      [ -e "$p" ] || die "$key '$name' named by $DEV has no preset: $p"
      [ -s "$p" ] || die "$key '$name' preset resolves but is EMPTY: $p"
    done < <(grep -oP "^\s*${key}:\s*\K[A-Za-z0-9._-]+" "$DEV" | sort -u)
  done
  # PATH-STYLE descriptor references (today: `stand: "stands/<x>.yaml"`). These are resolved the
  # way the cockpit's URDF composer resolves them — `<dir>/<rel>` then `<dir>/params-presets/<rel>`
  # (compose_urdf.py resolve_descriptor). They are checked HERE and not derived from the
  # controller's own loader on purpose: `stand:` is UI/URDF-only, `core/Config.cpp` ignores it, and
  # that asymmetry is exactly how the first version of this platform directory shipped without
  # params-presets/stands/ — the controller booted clean and the cockpit came up with an empty 3D
  # pane and "composed.urdf is not valid XML" instead of a refusal.
  local rel
  while read -r rel; do
    [ -n "$rel" ] || continue
    [ -s "$PDIR/$rel" ] || [ -s "$PDIR/params-presets/$rel" ] ||
      die "descriptor '$rel' named by $DEV does not resolve
  (tried $PDIR/$rel and $PDIR/params-presets/$rel)"
  done < <(grep -oP '^\s*[a-z_]+:\s*"?\K[A-Za-z0-9._/-]+\.yaml' "$DEV" | sort -u)
}

check_no_dangling() {
  local bad
  bad="$(find "$PDIR/" -type l ! -exec test -e {} \; -print 2>/dev/null || true)"
  [ -z "$bad" ] || die "dangling symlink(s) under $PDIR:
$bad
  (submodule not initialised? run: git submodule update --init --recursive)"
}

# ---------------------------------------------------------------------------
# Device file
# ---------------------------------------------------------------------------
install_real_device_file() {
  [ -f "$TEMPLATE" ] || die "missing device template $TEMPLATE"
  # NO serial_number GATE, deliberately. controller-manager only READS the field
  # (Config.cpp:369, `if (a["serial_number"]) arm.serial = ...`) and never validates it —
  # a SILS boot with an empty serial passes its own config gate. An earlier launch failure
  # that looked like a serial problem was the EMPTY tools/pika.yaml preset; that is what
  # check_params_presets() catches now. `ip` IS load-bearing (an unfilled shell is fatal
  # in CM itself), so it keeps its check.
  grep -qE '^\s*ip:\s*""' "$TEMPLATE" && die "a blank arm ip in $TEMPLATE"
  # The template is the source of truth for this rig's device facts. CM writes box-adopted DH
  # back into the working copy; it re-adopts from the box at init, so re-installing is safe.
  cp "$TEMPLATE" "$DEV"
  say "installed REAL device file -> $DEV"
}

seed_sils_device_file() {
  if [ ! -f "$DEV" ]; then
    sed -e '0,/ip: ""/s//ip: "127.0.0.1"/' -e '0,/ip: ""/s//ip: "127.0.0.2"/' \
        -e 's/^\(\s*tool:\s*\)none/\1pika/' "$SEED" > "$DEV"
    say "seeded SILS device shell -> $DEV (loopback, tool: pika)"
  fi
}

# ---------------------------------------------------------------------------
# Launch
# ---------------------------------------------------------------------------
cm_env() {
  export CONTROL_MANAGER_ACTIVE_YAML="$DEV"
  export CONTROL_MANAGER_PLATFORM="$PLATFORM"
  # The named-plan library, for BOTH the cockpit's plan service and the ./plan CLI. Upstream's
  # own working-library location is <ws>/plans/ — inside the submodule, so our repo could not
  # track it — and cockpit/plans/examples.yaml says of itself that it is overwritten on upgrade.
  # planlib resolves this env var FIRST (planlib.py:298-313), so the library lives with us.
  export CONTROL_MANAGER_PLANS_FILE="$ROOT/cm_bridge/config/plans.yaml"
}

start_controller() {   # $1 = "--sils" or ""
  mkdir -p "$RUN" "$ROOT/logs"
  local log="$ROOT/logs/cm_controller_${STAMP}.log"
  local opts="${1:-}"
  # The cockpit is upstream's own operator web UI (http://127.0.0.1:$COCKPIT_PORT) and is what the
  # bring-up ladder is driven from, so it is ON by default for sim/real; `gate` forces it off
  # (headless) and COCKPIT=0 opts out.
  [ "${COCKPIT:-1}" = 1 ] && opts="$opts --cockpit"
  cm_env
  # start.sh runs the controller in the foreground and owns its own pid file, stray detection and
  # clean shutdown (SIGINT). We background IT rather than the binary so that lifecycle is upstream's.
  nohup setsid bash -c "exec '$START' $opts" > "$log" 2>&1 &
  echo "$!" > "$RUN/controller.sh.pid"
  say "controller starting${opts:+ ($opts)} -> $log"
  local i
  for i in $(seq 1 60); do
    grep -aq "TASKCFG.*follow{" "$log" && break
    grep -aqE "ConfigInvalid|EXIT_FAILURE|FATAL|refus" "$log" && {
      echo "----- controller log tail -----" >&2; tail -20 "$log" >&2
      die "controller refused to start (see $log)"
    }
    sleep 0.5
  done
  grep -aq "TASKCFG.*follow{" "$log" || {
    tail -20 "$log" >&2; die "controller did not reach the TASKCFG banner (see $log)"
  }
  # The banner is the ONLY ground truth for which params file is live.
  grep -a "TASKCFG.*task params from\|TASKCFG.*follow{" "$log" | head -2 | sed 's/^/[cm-stack]   /'
}

start_bridge() {
  mkdir -p "$RUN" "$ROOT/logs"
  # ONE bridge only. A previous instance (this launcher's or a hand-started one) would still be
  # bound to 50264/50256 (SO_REUSEADDR) and subscribed to act/follow_step: chunks would reach one
  # of them at random and every one would fire the gripper. Stop it before starting ours.
  pkill -INT -f "cm_bridge_nod[e].py --platform" 2>/dev/null && sleep 1.5 || true
  pkill -TERM -f "cm_bridge_nod[e].py --platform" 2>/dev/null || true
  local log="$ROOT/logs/cm_bridge_${STAMP}.log"
  cm_env
  # rclpy + cell_msgs both come from env.sh (ROS underlay + the submodule's install/). System
  # python3 only — the repo .venv has no rclpy.
  nohup setsid bash -c "set +u; source '$CM/platforms/$PLATFORM/scripts/env.sh' >/dev/null 2>&1;
                        exec /usr/bin/python3 '$ROOT/cm_bridge/src/cm_bridge_node.py' --platform $PLATFORM" \
       > "$log" 2>&1 &
  echo "$!" > "$RUN/bridge.pid"
  say "bridge up (chunk 50264 -> /$PLATFORM/<side>/cmd/follow) -> $log"
}

start_collision_monitor() {
  mkdir -p "$RUN" "$ROOT/logs"
  pkill -TERM -f "collision_monito[r].py" 2>/dev/null || true   # one monitor only (same reason as the bridge)
  local log="$ROOT/logs/collision_monitor_${STAMP}.log"
  # -u: the monitor reports TRIP/CLEAR with print(); block-buffered under nohup that never reached
  # the log while it mattered (2026-08-19: an empty monitor log during a real fault latch).
  nohup setsid "$ROOT/.venv/bin/python" -u "$ROOT/cm_bridge/src/collision_monitor.py" > "$log" 2>&1 &
  echo "$!" > "$RUN/collision.pid"
  say "collision monitor up -> $log"
}

stop_all() {
  cm_env
  [ -x "$START" ] && bash -c "set +u; '$START' --stop" 2>/dev/null || true
  local f p
  for f in bridge collision controller.sh; do
    p="$RUN/$f.pid"
    [ -f "$p" ] && { kill -TERM "$(cat "$p")" 2>/dev/null || true; rm -f "$p"; }
  done
  pkill -f "cm_bridge_nod[e]" 2>/dev/null || true
  pkill -f "collision_monito[r]" 2>/dev/null || true
  say "stopped"
}

case "$MODE" in
  sim)
    preflight_common
    seed_sils_device_file
    check_no_dangling; check_params_tasks; check_params_presets
    start_controller --sils
    start_bridge
    say "SILS up. Console:"
    say "  source $CM/platforms/$PLATFORM/scripts/env.sh && \\"
    say "    ros2 service call /$PLATFORM/cell/cmd/console cell_msgs/srv/Command \"{command: enable}\""
    ;;
  real)
    preflight_common
    install_real_device_file
    check_no_dangling; check_params_tasks; check_params_presets
    start_controller
    start_bridge
    start_collision_monitor
    say "CM real up + bridge + collision monitor. Arms are NOT energized."
    say "Operator ladder: cm_bridge/docs/real_bringup.md"
    say "  source $CM/platforms/$PLATFORM/scripts/env.sh"
    say "  ros2 service call /$PLATFORM/cell/cmd/console cell_msgs/srv/Command \"{command: enable}\""
    say "  ... then {command: mode real}, {command: task on}, per-arm {command: task idle}"
    ;;
  gate)
    preflight_common
    seed_sils_device_file
    check_no_dangling; check_params_tasks; check_params_presets
    COCKPIT=0 start_controller --sils
    cm_env
    bash -c "set +u; source '$CM/platforms/$PLATFORM/scripts/env.sh' >/dev/null 2>&1
for c in enable 'task on'; do
  ros2 service call /$PLATFORM/cell/cmd/console cell_msgs/srv/Command \"{command: \$c}\" >/dev/null; sleep 2.5
done
for s in left right; do
  ros2 service call /$PLATFORM/\$s/cmd/console cell_msgs/srv/Command '{command: task idle}' >/dev/null
done"
    start_bridge
    sleep 3
    rc=0; python3 "$ROOT/cm_bridge/tests/cm_sils_gate.py" || rc=$?
    stop_all
    exit $rc
    ;;
  down|stop)
    stop_all
    ;;
  status)
    cm_env
    bash -c "set +u; '$START' --status" 2>/dev/null || true
    for f in bridge collision; do
      p="$RUN/$f.pid"
      if [ -f "$p" ] && kill -0 "$(cat "$p")" 2>/dev/null; then echo "$f RUNNING pid=$(cat "$p")"
      else echo "$f not running (by pid file)"; fi
    done
    ;;
  *)
    echo "usage: run_cm_stack.sh {sim|real|gate|down|status}" >&2
    exit 2
    ;;
esac
