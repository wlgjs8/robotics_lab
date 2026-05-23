#!/usr/bin/env bash
set -euo pipefail

TASK="${1:-}"

if [[ -z "$TASK" ]]; then
  echo "Usage: $0 <TASK_ID>" >&2
  exit 2
fi

REPO_ROOT="$(git rev-parse --show-toplevel)"
cd "$REPO_ROOT"

run_servo_gate() {
  local cmake_prefix_args=()
  if [[ -n "${CMAKE_PREFIX_PATH:-}" ]]; then
    cmake_prefix_args=(-DCMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH}")
  elif [[ -d "${HOME}/miniconda3/share/cmake/nlohmann_json" || -d "${HOME}/miniconda3/lib/cmake/nlohmann_json" ]]; then
    cmake_prefix_args=(-DCMAKE_PREFIX_PATH="${HOME}/miniconda3")
  fi

  cmake -S rb_servo_server -B rb_servo_server/build/hardware_free_gate \
    -DCMAKE_BUILD_TYPE=Debug \
    -DRB_SERVO_ENABLE_RBPODO=OFF \
    -DRB_SERVO_ENABLE_PINOCCHIO=OFF \
    -DRB_SERVO_ALLOW_FETCHCONTENT=OFF \
    -DBUILD_TESTING=ON \
    "${cmake_prefix_args[@]}"

  cmake --build rb_servo_server/build/hardware_free_gate -j
  ctest --test-dir rb_servo_server/build/hardware_free_gate --output-on-failure
}

cmake_package_available() {
  local package_name="$1"
  local tmpdir
  tmpdir="$(mktemp -d)"
  cat > "${tmpdir}/CMakeLists.txt" <<EOF
cmake_minimum_required(VERSION 3.16)
project(check_${package_name} LANGUAGES CXX)
find_package(${package_name} REQUIRED)
EOF

  local cmake_prefix_args=()
  if [[ -n "${CMAKE_PREFIX_PATH:-}" ]]; then
    cmake_prefix_args=(-DCMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH}")
  elif [[ -d "${HOME}/miniconda3/share/cmake/nlohmann_json" || -d "${HOME}/miniconda3/lib/cmake/nlohmann_json" ]]; then
    cmake_prefix_args=(-DCMAKE_PREFIX_PATH="${HOME}/miniconda3")
  fi

  if cmake -S "${tmpdir}" -B "${tmpdir}/build" "${cmake_prefix_args[@]}" >/dev/null 2>&1; then
    rm -rf "${tmpdir}"
    return 0
  fi
  rm -rf "${tmpdir}"
  return 1
}

run_optional_pinocchio_gate() {
  if ! cmake_package_available pinocchio; then
    echo "codex_gate: skipping optional Pinocchio ON gate; CMake package pinocchio not found"
    return 0
  fi

  local cmake_prefix_args=()
  if [[ -n "${CMAKE_PREFIX_PATH:-}" ]]; then
    cmake_prefix_args=(-DCMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH}")
  elif [[ -d "${HOME}/miniconda3/share/cmake/nlohmann_json" || -d "${HOME}/miniconda3/lib/cmake/nlohmann_json" ]]; then
    cmake_prefix_args=(-DCMAKE_PREFIX_PATH="${HOME}/miniconda3")
  fi

  cmake -S rb_servo_server -B rb_servo_server/build/pinocchio_gate \
    -DCMAKE_BUILD_TYPE=Debug \
    -DRB_SERVO_ENABLE_RBPODO=OFF \
    -DRB_SERVO_ENABLE_PINOCCHIO=ON \
    -DRB_SERVO_ALLOW_FETCHCONTENT=OFF \
    -DBUILD_TESTING=ON \
    "${cmake_prefix_args[@]}"

  cmake --build rb_servo_server/build/pinocchio_gate -j
  ctest --test-dir rb_servo_server/build/pinocchio_gate --output-on-failure
}

run_simulator_tests() {
  PYTHONPATH=rb_simulator/src python3 -m unittest discover rb_simulator/tests
}

run_gui_tests() {
  python3 -m unittest discover rb_gui/tests
}

run_policy_runner_tests() {
  python3 -m unittest discover policy_runner/tests
}

run_camera_gate() {
  cmake -S camera_server -B camera_server/build/hardware_free_gate \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCAMERA_SERVER_FORCE_MOCK_CAMERA=ON \
    -DCAMERA_SERVER_FORCE_ZMQ_STUB=ON \
    -DCAMERA_SERVER_BUILD_TESTS=ON

  cmake --build camera_server/build/hardware_free_gate -j
  ctest --test-dir camera_server/build/hardware_free_gate --output-on-failure
}

run_shell_syntax_checks() {
  bash -n scripts/codex_gate.sh
  bash -n scripts/codex_run_sequence.sh
  bash -n scripts/hardware_free_validation.sh
  bash -n scripts/tcp_pose_simulator_acceptance.sh
}

grep_existing() {
  local pattern="$1"
  shift

  local paths=()
  local path
  for path in "$@"; do
    if [[ -e "$path" ]]; then
      paths+=("$path")
    fi
  done

  if [[ "${#paths[@]}" -eq 0 ]]; then
    echo "ERROR: no existing paths to grep for pattern: $pattern" >&2
    return 2
  fi

  grep -R -E "$pattern" "${paths[@]}" >/dev/null
}

check_real_config_safety_docs() {
  grep_existing "RB_ALLOW_REAL_MOTION" README.md docs AGENTS.md
  grep_existing "BackendResult|SendServoJResult|ArmWorker" README.md docs AGENTS.md

  grep -E 'ip: "172\.28\.60\.200"' rb_servo_server/config/dual_real.example.yaml >/dev/null
  grep -E 'ip: "172\.28\.60\.201"' rb_servo_server/config/dual_real.example.yaml >/dev/null
  grep -E 'send_servo_commands: false' rb_servo_server/config/dual_real.example.yaml >/dev/null
  grep_existing "dual_real.example.yaml" README.md docs rb_servo_server/docs rb_servo_server/config/dual_real.example.yaml
  grep_existing "dual_real_readonly.yaml" README.md docs rb_servo_server/docs rb_servo_server/config/dual_real.example.yaml
  grep_existing "dual_real_motion.yaml" README.md docs rb_servo_server/docs rb_servo_server/config/dual_real.example.yaml
  grep_existing "RB_ALLOW_REAL_CARTESIAN" README.md docs rb_servo_server/docs rb_servo_server/config/dual_real.example.yaml
}

check_migration_rebaseline_docs() {
  check_real_config_safety_docs
  grep_existing "CommandBuffer -> ServoCoordinator -> Left/Right ArmWorker|CommandBuffer -> ServoCoordinator -> Left ArmWorker" README.md docs rb_servo_server/docs
  grep_existing "RobotFault" README.md docs rb_servo_server/docs
  grep_existing "TransportWriteFailed" README.md docs rb_servo_server/docs
  grep_existing "SuppressedByPolicy" README.md docs rb_servo_server/docs
  grep_existing "WrongMode" README.md docs rb_servo_server/docs
  grep_existing "RB_ALLOW_REAL_ROBOT" README.md docs rb_servo_server/docs rb_servo_server/config/dual_real.example.yaml
  grep_existing "RB_ALLOW_REAL_MOTION" README.md docs rb_servo_server/docs rb_servo_server/config/dual_real.example.yaml
  grep_existing "operator intervention" README.md docs rb_servo_server/docs
  grep_existing "state_pub_rate_hz" README.md docs rb_servo_server/docs
  grep_existing "io_model: direct|direct io_model" README.md docs rb_servo_server/docs
  grep_existing "io_model: worker|worker io_model" README.md docs rb_servo_server/docs
  grep -E "Deprecated|deprecated|Compatibility" rb_servo_server/config/dual_rbsim.yaml >/dev/null
  grep -E "Deprecated|deprecated|Compatibility" rb_servo_server/config/dual_rb_simulator.yaml >/dev/null
  grep -E "Deprecated|deprecated|Compatibility" rb_servo_server/config/dual_rb_simulator_compose.yaml >/dev/null
}

check_mig18_config_rebaseline() {
  check_real_config_safety_docs

  local canonical_configs=(
    rb_servo_server/config/dual_simulator.yaml
    rb_servo_server/config/dual_simulator_compose.yaml
    rb_servo_server/config/dual_simulator_worker.yaml
    rb_simulator/config/left_rb3_730e.yaml
    rb_simulator/config/right_rb3_730e.yaml
  )
  local config_path
  for config_path in "${canonical_configs[@]}"; do
    if [[ ! -f "${config_path}" ]]; then
      echo "ERROR: missing canonical simulator config: ${config_path}" >&2
      return 1
    fi
  done

  grep_existing "Canonical Config Names" README.md
  grep_existing "dual_simulator.yaml" README.md docs/architecture.md rb_servo_server/docs/config_examples.md
  grep_existing "dual_simulator_compose.yaml" README.md docs/architecture.md rb_servo_server/docs/config_examples.md
  grep_existing "dual_simulator_worker.yaml" README.md docs/architecture.md docs/hardware_free_validation.md
  grep_existing "left_rb3_730e.yaml" README.md rb_simulator/README.md rb_simulator/docs/architecture.md
  grep_existing "right_rb3_730e.yaml" README.md rb_simulator/README.md rb_simulator/docs/architecture.md

  grep -E "Deprecated|deprecated|compatibility|Compatibility" rb_servo_server/config/dual_rbsim.yaml >/dev/null
  grep -E "Deprecated|deprecated|compatibility|Compatibility" rb_servo_server/config/dual_rb_simulator.yaml >/dev/null
  grep -E "Deprecated|deprecated|compatibility|Compatibility" rb_servo_server/config/dual_rb_simulator_compose.yaml >/dev/null
  grep -E "NOT RUNNABLE|not runnable|historical" rb_simulator/config/dual_rb3_730e.yaml >/dev/null
  grep_existing "rbsim_local.*simulator|rbsim.*simulator|compatibility names should be removed" README.md docs rb_servo_server/docs rb_simulator/docs
}

run_mig12_gate() {
  run_shell_syntax_checks
  check_migration_rebaseline_docs
  run_simulator_tests
  run_gui_tests
  run_policy_runner_tests
  run_servo_gate
  run_optional_pinocchio_gate
}

run_mig13_gate() {
  run_shell_syntax_checks
  run_simulator_tests
  run_servo_gate
}

run_mig20_gate() {
  run_shell_syntax_checks
  run_servo_gate
  run_optional_pinocchio_gate
}

case "$TASK" in
  P0-A)
    grep_existing "RB_ALLOW_REAL_MOTION" README.md docs
    ;;

  P0-B)
    run_simulator_tests
    ;;

  P0-D|P1-A|P1-B|P1-F|P2-A|P2-C|P3-A|P3-B|P3-C)
    run_servo_gate
    ;;

  P0-G)
    run_camera_gate
    ;;

  P0-H|P2-D|P3-D)
    run_gui_tests
    ;;

  P1-C)
    run_simulator_tests
    run_servo_gate
    ./scripts/hardware_free_validation.sh
    ;;

  P1-D|P1-E|P2-E|P3-E)
    run_policy_runner_tests
    ;;

  P2-B)
    grep_existing "geometry_valid_for_real_policy" calibration geometry docs README.md
    grep_existing "configured_estimate" calibration geometry docs README.md
    ;;

  P3-F)
    bash -n scripts/tcp_pose_simulator_acceptance.sh
    grep_existing "RB_ALLOW_REAL_CARTESIAN" docs/runbooks/tcp_pose_simulator_acceptance.md README.md
    run_optional_pinocchio_gate
    ;;

  MIG-00)
    run_shell_syntax_checks
    check_real_config_safety_docs
    ;;

  MIG-01|MIG-02|MIG-04|MIG-05|MIG-06|MIG-07|MIG-08|MIG-09|MIG-11)
    run_servo_gate
    ;;

  MIG-12)
    run_mig12_gate
    ;;

  MIG-13)
    run_mig13_gate
    ;;

  MIG-14|MIG-15|MIG-16|MIG-21|MIG-22|MIG-23|MIG-24)
    run_servo_gate
    ;;

  MIG-20)
    run_mig20_gate
    ;;

  MIG-10)
    run_simulator_tests
    run_servo_gate
    ./scripts/hardware_free_validation.sh
    ;;

  MIG-03)
    run_simulator_tests
    run_servo_gate
    ;;

  MIG-17)
    run_policy_runner_tests
    ;;

  MIG-18)
    run_shell_syntax_checks
    check_mig18_config_rebaseline
    run_simulator_tests
    run_gui_tests
    run_policy_runner_tests
    ;;

  MIG-19|MIG-25|MIG-26)
    run_shell_syntax_checks
    run_simulator_tests
    run_gui_tests
    run_policy_runner_tests
    ;;

  *)
    echo "ERROR: unknown task: $TASK" >&2
    exit 2
    ;;
esac
