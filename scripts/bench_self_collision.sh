#!/usr/bin/env bash
# Build + run the self-collision timing benchmark (capsule vs URDF mesh).
set -euo pipefail
HERE="$(cd "$(dirname "$0")/.." && pwd)"
OUT=/tmp/bench_sc
g++ -O3 -DNDEBUG -DPINOCCHIO_WITH_HPP_FCL -DCOAL_DISABLE_HPP_FCL_WARNINGS -std=c++17 \
  "$HERE/scripts/bench_self_collision.cpp" -o "$OUT" \
  -I "$HERE/rb_servo_server/include" -I /opt/openrobots/include -I /usr/include/eigen3 \
  -L /opt/openrobots/lib -Wl,-rpath,/opt/openrobots/lib \
  -lpinocchio_default -lpinocchio_parsers -lpinocchio_collision -lcoal
"$OUT"
