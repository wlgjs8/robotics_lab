#!/usr/bin/env bash
# Source-build the local teleop stack for DIRECT real-controller operation:
# rb_servo_server (rbpodo backend) + rb_gui + policy_runner.
#
# Re-run after editing source; then `make run` connects straight to the
# physical controllers. Idempotent: the one-time
# rbpodo C++ SDK clone/install is skipped when already present, and the Python
# components are editable installs (subsequent source edits are live, no
# reinstall). The C++ server is rebuilt every run.
#
# Usage: tools/build_stack.sh [--clean]
#   --clean removes only the rb_servo_server build tree used by make run.
# Overrides: BUILD_JOBS, RBPODO_TAG, RBPODO_SRC, STACK_PYTHON.
# BUILD_STACK_PYTHON remains a build-only compatibility override.
set -euo pipefail

CLEAN_BUILD=0
case "${1:-}" in
  "") ;;
  --clean) CLEAN_BUILD=1 ;;
  *)
    echo "usage: $0 [--clean]" >&2
    exit 2
    ;;
esac
[ "$#" -le 1 ] || { echo "usage: $0 [--clean]" >&2; exit 2; }

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

JOBS="${BUILD_JOBS:-$(nproc)}"
BUILD_DIR="rb_servo_server/build/rbpodo_real_gate"   # the path tools/run_stack.sh launches
SERVER_BIN="$BUILD_DIR/rb_servo_server"
RT_CAPS="cap_sys_nice,cap_ipc_lock+ep"

# Python for the editable installs: use the same STACK_PYTHON override that
# run_stack.sh understands, prefer the repo venv, else use system python3.
if [ -n "${STACK_PYTHON:-}" ]; then PY="$STACK_PYTHON"
elif [ -n "${BUILD_STACK_PYTHON:-}" ]; then PY="$BUILD_STACK_PYTHON"
elif [ -x ".venv/bin/python" ]; then PY=".venv/bin/python"
else PY="python3"; fi

# Non-interactive sudo: passwordless first, then the gitignored ./.env
# SUDO_PASSWORD (chmod 600; piped via stdin so it never lands in `ps`/argv),
# then interactive as a last resort. Used for the rbpodo /usr/local install
# and the setcap below so `make build` never blocks on a prompt.
SUDO_PASSWORD=""
[ -f "$ROOT/.env" ] && SUDO_PASSWORD=$(grep -E '^SUDO_PASSWORD=' "$ROOT/.env" | head -1 | cut -d= -f2-)
run_sudo() {
  sudo -n "$@" 2>/dev/null && return 0
  if [ -n "$SUDO_PASSWORD" ]; then
    printf '%s\n' "$SUDO_PASSWORD" | sudo -S -p '' "$@"
    return $?
  fi
  sudo "$@"
}

# 1) rbpodo C++ SDK — find_package(rbpodo) for the RB_SERVO_ENABLE_RBPODO=ON
#    build. Installed once into /usr/local; re-runs detect it and skip.
RBPODO_TAG="${RBPODO_TAG:-v0.16.10}"
RBPODO_SRC="${RBPODO_SRC:-$HOME/rbpodo_src}"
RBPODO_CONFIG=$(find /usr/local /opt -type f \
  \( -name 'rbpodoConfig.cmake' -o -name 'rbpodo-config.cmake' \) \
  -print -quit 2>/dev/null || true)
if [ -z "$RBPODO_CONFIG" ]; then
  echo "[build] rbpodo C++ SDK not found; installing $RBPODO_TAG into /usr/local"
  [ -d "$RBPODO_SRC/.git" ] || git clone --depth 1 --branch "$RBPODO_TAG" \
    https://github.com/RainbowRobotics/rbpodo "$RBPODO_SRC"
  cmake -S "$RBPODO_SRC" -B "$RBPODO_SRC/build_cpp" \
    -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_BINDINGS=OFF -DBUILD_EXAMPLES=OFF
  cmake --build "$RBPODO_SRC/build_cpp" -j "$JOBS"
  run_sudo cmake --install "$RBPODO_SRC/build_cpp"
fi

# 2) rb_servo_server -> the exact path `make run` expects (build/rbpodo_real_gate).
if [ "$CLEAN_BUILD" -eq 1 ]; then
  echo "[build] clean rebuild: removing $BUILD_DIR"
  rm -rf -- "$BUILD_DIR"
fi
echo "[build] rb_servo_server (RB_SERVO_ENABLE_RBPODO=ON) -> $BUILD_DIR"
export CMAKE_PREFIX_PATH="/opt/openrobots:/usr/local${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
cmake -S rb_servo_server -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE=Release -DRB_SERVO_ENABLE_RBPODO=ON
cmake --build "$BUILD_DIR" -j "$JOBS"

# 2b) ODR / stale-object SAFETY guard. cmake's incremental header-dependency tracking
#     can MISS transitively-config-dependent TUs when a struct-layout header
#     (config.hpp) changes, leaving a MIXED-LAYOUT binary (ODR). That silently corrupts
#     runtime config and can DISABLE a safety guard — a real 2026-07 incident where a
#     mixed build let an arm penetrate the floor while FloorViolation was still detected.
#     Fail closed: if any object linked into the SERVER (core lib + exe, not tests) is
#     older than config.hpp, refuse to ship the binary and demand a clean rebuild.
#     Keep STRUCT_HDRS in sync with CMakeLists.txt RB_SERVO_LAYOUT_HEADERS.
STRUCT_HDRS=("rb_servo_server/include/rb_servo/config/config.hpp")
STALE_OBJS=""
for hdr in "${STRUCT_HDRS[@]}"; do
  found=$(find "$BUILD_DIR/CMakeFiles/rb_servo_core.dir" \
               "$BUILD_DIR/CMakeFiles/rb_servo_server.dir" \
               -name '*.o' ! -newer "$hdr" 2>/dev/null || true)
  [ -n "$found" ] && STALE_OBJS+="$found"$'\n'
done
if [ -n "${STALE_OBJS//[$'\n\t ']/}" ]; then
  echo "[build] FATAL: server object(s) are OLDER than a struct-layout header after build." >&2
  echo "[build]        The incremental build is LAYOUT-INCONSISTENT (ODR) — it can silently" >&2
  echo "[build]        corrupt runtime safety config (floor/collision). Clean rebuild required:" >&2
  echo "[build]            make rebuild" >&2
  echo "[build]        stale objects:" >&2
  printf '%s' "$STALE_OBJS" | sed '/^$/d;s/^/[build]          /' >&2
  exit 1
fi
echo "[build] ODR guard: OK (all server objects newer than ${STRUCT_HDRS[*]})"

# 3) RT capabilities — stripped by every rebuild, so (re)apply via run_sudo.
if ! getcap "$SERVER_BIN" 2>/dev/null | grep -q cap_sys_nice; then
  if run_sudo setcap "$RT_CAPS" "$SERVER_BIN"; then
    echo "[build] setcap applied"
  else
    echo "[build] WARN: setcap failed; run_stack.sh will retry, or run manually:" >&2
    echo "[build]       sudo setcap $RT_CAPS $SERVER_BIN" >&2
  fi
fi

# 4) Python components — editable installs (edits live without reinstall).
echo "[build] pip install -e rb_gui + policy_runner[spacemouse,gripper]  (python: $PY)"
if ! "$PY" -m pip install -e rb_gui -e "policy_runner[spacemouse,gripper]"; then
  echo "[build] WARN: editable install failed (PEP 668 / no venv?). Retrying with --user" >&2
  "$PY" -m pip install --user -e rb_gui -e "policy_runner[spacemouse,gripper]" \
    || echo "[build] WARN: pip install failed; install rb_gui/policy_runner deps manually" >&2
fi

echo "[build] done -> $SERVER_BIN"
echo "[build] next: make run            # direct real controller"
echo "[build]       make run MODE=sim   # controller-simulation (same binary)"
