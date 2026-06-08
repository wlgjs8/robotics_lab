#!/usr/bin/env bash
# One-time prerequisites for the native pgmode-simulation stack:
#  1) rbpodo SDK (Python into the repo venv + C++ into /usr/local)
#  2) rb_servo_server built with RB_SERVO_ENABLE_RBPODO=ON
# Assumes Pinocchio is already installed (scripts/install_deps_ubuntu.sh) and a
# repo venv exists at .venv. rbpodo source: RainbowRobotics/rbpodo (public).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

RBPODO_TAG="${RBPODO_TAG:-v0.16.10}"
RBPODO_SRC="${RBPODO_SRC:-$HOME/rbpodo_src}"
PY="${PGMODE_SIM_PYTHON:-.venv/bin/python}"
PIP="${PGMODE_SIM_PIP:-.venv/bin/pip}"
JOBS="${RBPODO_BUILD_JOBS:-4}"
[[ -x "$PY" ]] || { echo "missing venv $PY — create it and install rb_gui first" >&2; exit 1; }

# 1) clone rbpodo
if [[ ! -d "$RBPODO_SRC/.git" ]]; then
    echo "pgmode_sim_build: cloning rbpodo $RBPODO_TAG"
    git clone --depth 1 --branch "$RBPODO_TAG" https://github.com/RainbowRobotics/rbpodo "$RBPODO_SRC"
fi

# 2) Python rbpodo into the venv
echo "pgmode_sim_build: installing rbpodo Python module into the venv"
"$PIP" install "$RBPODO_SRC"
"$PY" -c "import rbpodo; print('rbpodo python OK, CobotData:', hasattr(rbpodo, 'CobotData'))"

# 3) C++ rbpodo into /usr/local (find_package(rbpodo))
echo "pgmode_sim_build: building + installing C++ rbpodo (sudo install to /usr/local)"
cmake -S "$RBPODO_SRC" -B "$RBPODO_SRC/build_cpp" \
    -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_BINDINGS=OFF -DBUILD_EXAMPLES=OFF
cmake --build "$RBPODO_SRC/build_cpp" -j "$JOBS"
sudo cmake --install "$RBPODO_SRC/build_cpp"

# 4) rb_servo_server with rbpodo enabled
echo "pgmode_sim_build: building rb_servo_server (RB_SERVO_ENABLE_RBPODO=ON)"
export CMAKE_PREFIX_PATH="/opt/openrobots:/usr/local${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
cmake -S rb_servo_server -B rb_servo_server/build_rbpodo -DRB_SERVO_ENABLE_RBPODO=ON
cmake --build rb_servo_server/build_rbpodo -j "$JOBS"

echo "pgmode_sim_build: done."
echo "  binary: rb_servo_server/build_rbpodo/rb_servo_server"
echo "  next:   cp rb_servo_server/config/local/pgmode_sim.env.example rb_servo_server/config/local/pgmode_sim.env  (edit IPs)"
echo "          and create rb_servo_server/config/local/vm_dual_cartesian.yaml  (see docs/plans/pgmode-sim-full-operation.md)"
echo "          then: make pgmode-sim-up"
