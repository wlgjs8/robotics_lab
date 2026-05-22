#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
JOBS="${JOBS:-$(nproc 2>/dev/null || echo 2)}"

CAMERA_BUILD_DIR="${CAMERA_BUILD_DIR:-${ROOT_DIR}/camera_server/build/hardware_free_gate}"
RB_SERVO_BUILD_DIR="${RB_SERVO_BUILD_DIR:-${ROOT_DIR}/rb_servo_server/build/hardware_free_gate}"
RBSIM_SMOKE_MODE="${RBSIM_SMOKE_MODE:-auto}"
RBSIM_SMOKE_ARTIFACTS_DIR="${RBSIM_SMOKE_ARTIFACTS_DIR:-${ROOT_DIR}/rb_simulator/artifacts/hardware_free_gate}"
RBSIM_EXECUTABLE="${RBSIM_EXECUTABLE:-${ROOT_DIR}/rb_simulator/build/rb_simulator}"
RBSIM_CONFIG="${RBSIM_CONFIG:-${ROOT_DIR}/rb_simulator/config/dual_rb3_730e.yaml}"
RBSIM_SERVO_CONFIG="${RBSIM_SERVO_CONFIG:-${ROOT_DIR}/rb_servo_server/config/dual_rb_simulator.yaml}"

cmake_prefix_args=()
if [[ -n "${CMAKE_PREFIX_PATH:-}" ]]; then
  cmake_prefix_args=(-DCMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH}")
elif [[ -d "${HOME}/miniconda3/share/cmake/nlohmann_json" || -d "${HOME}/miniconda3/lib/cmake/nlohmann_json" ]]; then
  cmake_prefix_args=(-DCMAKE_PREFIX_PATH="${HOME}/miniconda3")
fi

echo "hardware-free gate: camera_server mock/stub CMake + CTest"
cmake \
  -S "${ROOT_DIR}/camera_server" \
  -B "${CAMERA_BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCAMERA_SERVER_FORCE_MOCK_CAMERA=ON \
  -DCAMERA_SERVER_FORCE_ZMQ_STUB=ON \
  -DCAMERA_SERVER_BUILD_TESTS=ON \
  "${cmake_prefix_args[@]}"
cmake --build "${CAMERA_BUILD_DIR}" -j "${JOBS}"
ctest --test-dir "${CAMERA_BUILD_DIR}" --output-on-failure

echo "hardware-free gate: rb_servo mock-only CMake + CTest, including GUI unittest discovery"
cmake \
  -S "${ROOT_DIR}/rb_servo_server" \
  -B "${RB_SERVO_BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE=Debug \
  -DRB_SERVO_ENABLE_RBPODO=OFF \
  -DRB_SERVO_ALLOW_FETCHCONTENT=OFF \
  -DBUILD_TESTING=ON \
  "${cmake_prefix_args[@]}"
cmake --build "${RB_SERVO_BUILD_DIR}" -j "${JOBS}"
ctest --test-dir "${RB_SERVO_BUILD_DIR}" --output-on-failure

echo "hardware-free gate: rb_servo log analyzer self-test"
python3 "${ROOT_DIR}/rb_servo_server/tools/analyze_servo_log.py" --self-test

echo "hardware-free gate: rb_simulator syntax/package build check"
python3 -m compileall -q "${ROOT_DIR}/rb_simulator/src" "${ROOT_DIR}/rb_simulator/tools"

echo "hardware-free gate: rb_simulator deterministic core and loopback protocol unittest discovery"
python3 -m unittest discover "${ROOT_DIR}/rb_simulator/tests"

echo "hardware-free gate: rb_simulator smoke validator self-test"
python3 "${ROOT_DIR}/rb_simulator/tools/rbsim_servo_smoke.py" --self-test

run_rbsim_smoke=false
case "${RBSIM_SMOKE_MODE}" in
  auto)
    if [[ -x "${RBSIM_EXECUTABLE}" && -x "${RB_SERVO_BUILD_DIR}/rb_servo_server" && -f "${RBSIM_SERVO_CONFIG}" ]]; then
      run_rbsim_smoke=true
    else
      echo "hardware-free gate: skipping full rbsim servo smoke; set RBSIM_SMOKE_MODE=required to fail on missing prerequisites"
      echo "  expected simulator: ${RBSIM_EXECUTABLE}"
      echo "  expected servo server: ${RB_SERVO_BUILD_DIR}/rb_servo_server"
      echo "  expected servo config: ${RBSIM_SERVO_CONFIG}"
    fi
    ;;
  required)
    run_rbsim_smoke=true
    ;;
  skip)
    echo "hardware-free gate: full rbsim servo smoke skipped by RBSIM_SMOKE_MODE=skip"
    ;;
  *)
    echo "hardware-free gate: RBSIM_SMOKE_MODE must be auto, required, or skip; got ${RBSIM_SMOKE_MODE}" >&2
    exit 2
    ;;
esac

if [[ "${run_rbsim_smoke}" == true ]]; then
  echo "hardware-free gate: full rb_simulator + rb_servo_server loopback smoke"
  python3 "${ROOT_DIR}/rb_simulator/tools/rbsim_servo_smoke.py" \
    --simulator "${RBSIM_EXECUTABLE}" \
    --simulator-config "${RBSIM_CONFIG}" \
    --server "${RB_SERVO_BUILD_DIR}/rb_servo_server" \
    --server-config "${RBSIM_SERVO_CONFIG}" \
    --artifacts-dir "${RBSIM_SMOKE_ARTIFACTS_DIR}"
fi

echo "hardware-free gate passed"
