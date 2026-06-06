#!/usr/bin/env bash
set -euo pipefail

TASK="${1:-}"

if [[ -z "$TASK" ]]; then
  echo "Usage: $0 <TASK_ID>" >&2
  exit 2
fi

if git rev-parse --show-toplevel >/dev/null 2>&1; then
  REPO_ROOT="$(git rev-parse --show-toplevel)"
else
  REPO_ROOT="$(pwd)"
fi
cd "$REPO_ROOT"

cmake_prefix_args() {
  if [[ -n "${CMAKE_PREFIX_PATH:-}" ]]; then
    printf '%s\n' "-DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}"
  elif [[ -d "${HOME}/miniconda3/share/cmake/nlohmann_json" || -d "${HOME}/miniconda3/lib/cmake/nlohmann_json" ]]; then
    printf '%s\n' "-DCMAKE_PREFIX_PATH=${HOME}/miniconda3"
  elif [[ -d "/opt/openrobots" ]]; then
    printf '%s\n' "-DCMAKE_PREFIX_PATH=/opt/openrobots"
  fi
}

cmake_package_available() {
  local package_name="$1"
  local tmpdir
  tmpdir="$(mktemp -d)"
  cat > "${tmpdir}/CMakeLists.txt" <<EOF_CMAKE
cmake_minimum_required(VERSION 3.16)
project(check_${package_name} LANGUAGES CXX)
find_package(${package_name} REQUIRED)
EOF_CMAKE

  local args=()
  mapfile -t args < <(cmake_prefix_args)

  if cmake -S "${tmpdir}" -B "${tmpdir}/build" "${args[@]}" >/dev/null 2>&1; then
    rm -rf "${tmpdir}"
    return 0
  fi
  rm -rf "${tmpdir}"
  return 1
}

cpp_base_deps_available() {
  cmake_package_available yaml-cpp && cmake_package_available nlohmann_json
}

rb_servo_cpp_deps_available() {
  cpp_base_deps_available && cmake_package_available Eigen3 && cmake_package_available pinocchio
}

run_ctest_with_retry() {
  local test_dir="$1"
  if ctest --test-dir "${test_dir}" --output-on-failure; then
    return 0
  fi
  echo "codex_gate: ctest failed in ${test_dir}; rerunning failed tests once" >&2
  ctest --test-dir "${test_dir}" --rerun-failed --output-on-failure
}

run_servo_gate() {
  local args=()
  mapfile -t args < <(cmake_prefix_args)

  cmake -S rb_servo_server -B rb_servo_server/build/hardware_free_gate \
    -DCMAKE_BUILD_TYPE=Debug \
    -DRB_SERVO_ENABLE_RBPODO=OFF \
    -DRB_SERVO_ALLOW_FETCHCONTENT=OFF \
    -DBUILD_TESTING=ON \
    "${args[@]}"

  cmake --build rb_servo_server/build/hardware_free_gate -j
  run_ctest_with_retry rb_servo_server/build/hardware_free_gate
}

build_servo_server_only() {
  local args=()
  mapfile -t args < <(cmake_prefix_args)

  cmake -S rb_servo_server -B rb_servo_server/build/hardware_free_gate \
    -DCMAKE_BUILD_TYPE=Debug \
    -DRB_SERVO_ENABLE_RBPODO=OFF \
    -DRB_SERVO_ALLOW_FETCHCONTENT=OFF \
    -DBUILD_TESTING=ON \
    "${args[@]}"

  cmake --build rb_servo_server/build/hardware_free_gate --target rb_servo_server -j
}

run_servo_gate_or_skip_missing_deps() {
  if rb_servo_cpp_deps_available; then
    run_servo_gate
    return 0
  fi

  if [[ "${CODEX_SKIP_MISSING_CPP_DEPS:-0}" == "1" ]]; then
    echo "codex_gate: skipping rb_servo_server C++ gate; required CMake package missing: yaml-cpp, nlohmann_json, Eigen3, or pinocchio"
    return 0
  fi

  echo "ERROR: missing rb_servo_server C++ dependencies: yaml-cpp, nlohmann_json, Eigen3, and pinocchio are required" >&2
  echo "Install on Ubuntu with: scripts/install_deps_ubuntu.sh --profile hardware-free" >&2
  echo "That installs robotpkg-pinocchio under /opt/openrobots; for conda/mamba/source installs, set CMAKE_PREFIX_PATH to the install prefix." >&2
  echo "Or rerun with CODEX_SKIP_MISSING_CPP_DEPS=1 to skip C++ gates temporarily." >&2
  return 1
}

run_servo_pinocchio_gate() {
  if ! rb_servo_cpp_deps_available; then
    if [[ "${CODEX_SKIP_MISSING_CPP_DEPS:-0}" == "1" ]]; then
      echo "codex_gate: skipping mandatory rb_servo_server Pinocchio gate; required CMake package missing"
      return 0
    fi
    echo "ERROR: cannot run rb_servo_server Pinocchio gate because required C++ deps are missing" >&2
    echo "Missing CMake package: pinocchio" >&2
    return 1
  fi

  local args=()
  mapfile -t args < <(cmake_prefix_args)

  cmake -S rb_servo_server -B rb_servo_server/build/pinocchio_gate \
    -DCMAKE_BUILD_TYPE=Debug \
    -DRB_SERVO_ENABLE_RBPODO=OFF \
    -DRB_SERVO_ALLOW_FETCHCONTENT=OFF \
    -DBUILD_TESTING=ON \
    "${args[@]}"

  cmake --build rb_servo_server/build/pinocchio_gate -j
  run_ctest_with_retry rb_servo_server/build/pinocchio_gate
}

run_camera_gate() {
  cmake -S camera_server -B camera_server/build/hardware_free_gate \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCAMERA_SERVER_FORCE_MOCK_CAMERA=ON \
    -DCAMERA_SERVER_FORCE_ZMQ_STUB=ON \
    -DCAMERA_SERVER_BUILD_TESTS=ON

  cmake --build camera_server/build/hardware_free_gate -j
  run_ctest_with_retry camera_server/build/hardware_free_gate
}

run_camera_gate_or_skip_missing_deps() {
  if cpp_base_deps_available; then
    run_camera_gate
    return 0
  fi

  if [[ "${CODEX_SKIP_MISSING_CPP_DEPS:-0}" == "1" ]]; then
    echo "codex_gate: skipping camera_server C++ gate; yaml-cpp or nlohmann_json CMake package is missing"
    return 0
  fi

  echo "ERROR: missing camera_server C++ dependencies: yaml-cpp and/or nlohmann_json" >&2
  echo "Install on Ubuntu: sudo apt-get install -y libyaml-cpp-dev nlohmann-json3-dev" >&2
  return 1
}

loopback_socket_available() {
  python3 - <<'PY'
import socket
import sys
sockets = []
try:
    for kind in (socket.SOCK_STREAM, socket.SOCK_DGRAM):
        sock = socket.socket(socket.AF_INET, kind)
        sock.bind(("127.0.0.1", 0))
        sockets.append(sock)
except OSError:
    sys.exit(1)
finally:
    for sock in sockets:
        sock.close()
PY
}

run_simulator_tests() {
  PYTHONPATH=rb_simulator/src python3 -m unittest discover rb_simulator/tests
}

run_gui_tests() {
  python3 -m unittest discover rb_gui/tests
}

run_policy_runner_tests() {
  PYTHONPATH=policy_runner python3 -m unittest discover policy_runner/tests
}

run_python_surface_tests() {
  run_gui_tests
  run_policy_runner_tests
  run_simulator_tests
}

run_optional_python_help() {
  local script="$1"
  if [[ -f "${script}" ]]; then
    python3 "${script}" --help >/dev/null
  else
    echo "codex_gate: optional script not present: ${script}"
  fi
}

run_optional_script_tests() {
  local pattern="$1"
  if find scripts -maxdepth 1 -name "${pattern}" -print -quit | grep -q .; then
    PYTHONPATH=scripts python3 -m unittest discover scripts -p "${pattern}"
  else
    echo "codex_gate: optional script tests not present: ${pattern}"
  fi
}

require_file() {
  local path="$1"
  if [[ ! -f "${path}" ]]; then
    echo "ERROR: required file is missing: ${path}" >&2
    return 1
  fi
}

run_required_python_help() {
  local script="$1"
  require_file "${script}"
  python3 "${script}" --help >/dev/null
}

run_required_script_tests() {
  local pattern="$1"
  if ! find scripts -maxdepth 1 -name "${pattern}" -print -quit | grep -q .; then
    echo "ERROR: required script tests not present: ${pattern}" >&2
    return 1
  fi
  PYTHONPATH=scripts python3 -m unittest discover -s scripts -p "${pattern}"
}

run_required_policy_runner_tests() {
  local pattern="$1"
  if ! find policy_runner/tests -maxdepth 1 -name "${pattern}" -print -quit | grep -q .; then
    echo "ERROR: required policy_runner tests not present: ${pattern}" >&2
    return 1
  fi
  PYTHONPATH=policy_runner python3 -m unittest discover -s policy_runner/tests -p "${pattern}"
}

run_policy_runner_help() {
  local subcommand="$1"
  PYTHONPATH=policy_runner python3 -m policy_runner "${subcommand}" --help >/dev/null
}

run_required_make_dry_run() {
  local target="$1"
  make -n "${target}" >/dev/null
}

run_optional_rbpodo_measurement_readonly() {
  local script="$1"
  shift

  if [[ "${CODEX_RUN_RBPODO_MEASUREMENT:-0}" != "1" ]]; then
    echo "codex_gate: skipping rbpodo measurement controller read-only check; set CODEX_RUN_RBPODO_MEASUREMENT=1 with explicit args to enable"
    return 0
  fi

  if [[ ! -f "${script}" ]]; then
    echo "ERROR: CODEX_RUN_RBPODO_MEASUREMENT=1 but ${script} is missing" >&2
    return 1
  fi
  if [[ -z "${CODEX_RBPODO_MEASUREMENT_ARGS:-}" ]]; then
    echo "ERROR: CODEX_RUN_RBPODO_MEASUREMENT=1 requires CODEX_RBPODO_MEASUREMENT_ARGS with explicit read-only script arguments" >&2
    return 1
  fi
  if [[ "${CODEX_RBPODO_MEASUREMENT_ARGS}" != *"--i-understand-this-connects-to-real-controller"* ]]; then
    echo "ERROR: CODEX_RBPODO_MEASUREMENT_ARGS must include the tool-level real-controller confirmation flag" >&2
    return 1
  fi

  local required_arg
  for required_arg in "$@"; do
    if [[ "${CODEX_RBPODO_MEASUREMENT_ARGS}" != *"${required_arg}"* ]]; then
      echo "ERROR: CODEX_RBPODO_MEASUREMENT_ARGS must include ${required_arg} for this read-only measurement gate" >&2
      return 1
    fi
  done

  # shellcheck disable=SC2086
  python3 "${script}" ${CODEX_RBPODO_MEASUREMENT_ARGS}
}

run_python_compile_checks() {
  python3 -m compileall -q \
    rb_simulator/src \
    rb_simulator/tools \
    rb_gui/rb_servo_gui \
    policy_runner/policy_runner \
    scripts
}

run_shell_syntax_checks() {
  bash -n scripts/codex_gate.sh
  bash -n scripts/codex_run_sequence.sh
  bash -n scripts/check_deps.sh
  bash -n scripts/hardware_free_validation.sh
  bash -n scripts/tcp_pose_simulator_acceptance.sh
  if [[ -f scripts/install_deps_ubuntu.sh ]]; then
    bash -n scripts/install_deps_ubuntu.sh
  fi
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

  grep -R -E -- "$pattern" "${paths[@]}" >/dev/null
}

grep_absent() {
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
    return 0
  fi

  if grep -R -E -- "$pattern" "${paths[@]}" >/dev/null; then
    echo "ERROR: forbidden pattern found: $pattern" >&2
    grep -R -n -E -- "$pattern" "${paths[@]}" >&2 || true
    return 1
  fi
}

run_optional_tcp_pose_acceptance() {
  if ! cmake_package_available pinocchio; then
    if [[ "${CODEX_SKIP_MISSING_CPP_DEPS:-0}" == "1" ]]; then
      echo "codex_gate: skipping simulator TCP pose acceptance; Missing CMake package: pinocchio"
      return 0
    fi
    echo "ERROR: simulator TCP pose acceptance requires Pinocchio" >&2
    echo "Missing CMake package: pinocchio" >&2
    return 1
  fi

  if ! loopback_socket_available; then
    echo "codex_gate: skipping simulator TCP pose acceptance; AF_INET loopback sockets are unavailable"
    return 0
  fi

  ./scripts/tcp_pose_simulator_acceptance.sh
}

run_cart_harden_05_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q scripts rb_servo_server/tools
  python3 rb_servo_server/tools/send_tcp_linear_move.py --help >/dev/null
  python3 rb_servo_server/tools/send_tcp_twist.py --help >/dev/null
  grep_existing "TcpPoseTarget|TcpLinearMove|TcpTwistLocal|TcpTwistStand|TcpDeltaLocal|TcpDeltaStand" \
    rb_servo_server/docs/network_protocol.md docs/runbooks/tcp_pose_simulator_acceptance.md
  grep_existing "path_s|path_line_deviation_m|orientation preservation|quaternion" \
    docs/runbooks/tcp_pose_simulator_acceptance.md rb_servo_server/docs/network_protocol.md
  if [[ "${CODEX_RUN_CARTESIAN_ACCEPTANCE:-0}" == "1" ]]; then
    run_servo_pinocchio_gate
    ./scripts/tcp_pose_simulator_acceptance.sh --all
  else
    echo "codex_gate: skipping full Cartesian simulator acceptance; set CODEX_RUN_CARTESIAN_ACCEPTANCE=1 to enable"
  fi
}

run_cart_math_gate() {
  run_shell_syntax_checks
  run_python_compile_checks
  run_simulator_tests
  run_gui_tests
  run_policy_runner_tests
  local pinocchio_var
  local forbidden_pinocchio_off
  pinocchio_var='RB_SERVO_ENABLE_PINOCCHIO'
  forbidden_pinocchio_off="-D${pinocchio_var}=OFF"
  grep_absent "${forbidden_pinocchio_off}" scripts rb_servo_server README.md docs AGENTS.md REVIEW.md
  run_servo_pinocchio_gate
  if [[ "${CODEX_RUN_CARTESIAN_ACCEPTANCE:-0}" == "1" ]]; then
    ./scripts/tcp_pose_simulator_acceptance.sh --all
  else
    echo "codex_gate: skipping full Cartesian simulator acceptance; set CODEX_RUN_CARTESIAN_ACCEPTANCE=1 to enable"
  fi
}

run_cart_accept_gate() {
  run_shell_syntax_checks
  if [[ -f scripts/cartesian_acceptance.py ]]; then
    python3 -m compileall -q scripts
  fi
  if [[ -f rb_servo_server/tools/send_tcp_linear_move.py ]]; then
    python3 rb_servo_server/tools/send_tcp_linear_move.py --help >/dev/null
  fi
  if [[ -f rb_servo_server/tools/send_tcp_twist.py ]]; then
    python3 rb_servo_server/tools/send_tcp_twist.py --help >/dev/null
  fi
  if [[ "${CODEX_RUN_CARTESIAN_ACCEPTANCE:-0}" == "1" ]]; then
    run_servo_pinocchio_gate
    ./scripts/tcp_pose_simulator_acceptance.sh --all
  else
    echo "codex_gate: skipping full Cartesian simulator acceptance; set CODEX_RUN_CARTESIAN_ACCEPTANCE=1 to enable"
  fi
}

run_circle_benchmark_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q scripts
  PYTHONPATH=scripts python3 -m unittest discover scripts -p 'test_circle_tracking_benchmark.py'
  python3 scripts/circle_tracking_benchmark.py --help >/dev/null
  python3 scripts/compare_circle_benchmarks.py --help >/dev/null
  grep_existing "circle_tracking_benchmark.py|BENCH-CIRCLE-01|GENE-style|15 cm" \
    docs/runbooks/circle_tracking_benchmark.md docs/runbooks/tcp_pose_simulator_acceptance.md README.md REVIEW.md
  PYTHONPATH=rb_simulator/src python3 scripts/circle_tracking_benchmark.py \
    --root . \
    --mode start-local \
    --server rb_servo_server/build/hardware_free_gate/rb_servo_server \
    --server-config rb_servo_server/config/dual_simulator_tcp_acceptance.yaml \
    --left-config rb_simulator/config/left_rb3_730e.yaml \
    --right-config rb_simulator/config/right_rb3_730e.yaml \
    --profile safe_5cm_10s \
    --artifact-dir artifacts/circle_tracking/preflight_gate \
    --preflight-only >/dev/null
  PYTHONPATH=rb_simulator/src python3 scripts/circle_tracking_benchmark.py \
    --root . \
    --mode start-local \
    --server rb_servo_server/build/hardware_free_gate/rb_servo_server \
    --server-config rb_servo_server/config/dual_simulator_circle_stress.yaml \
    --left-config rb_simulator/config/left_rb3_730e.yaml \
    --right-config rb_simulator/config/right_rb3_730e.yaml \
    --profile gene_15cm_4s \
    --allow-fast-stress \
    --artifact-dir artifacts/circle_tracking/gene_preflight_gate \
    --preflight-only >/dev/null
  run_gui_tests
  run_policy_runner_tests
  run_simulator_tests

  if [[ "${CODEX_RUN_CIRCLE_BENCHMARK:-0}" == "1" || "${CODEX_RUN_GENE_STYLE_CIRCLE:-0}" == "1" ]]; then
    if ! rb_servo_cpp_deps_available; then
      if [[ "${CODEX_SKIP_MISSING_CPP_DEPS:-0}" == "1" ]]; then
        echo "codex_gate: skipping circle benchmark runtime; required C++ deps are missing"
        return 0
      fi
      echo "ERROR: circle benchmark runtime requires yaml-cpp, nlohmann_json, Eigen3, and pinocchio" >&2
      return 1
    fi
    if ! loopback_socket_available; then
      echo "codex_gate: skipping circle benchmark runtime; AF_INET loopback sockets are unavailable"
      return 0
    fi
    build_servo_server_only
  fi

  if [[ "${CODEX_RUN_CIRCLE_BENCHMARK:-0}" == "1" ]]; then
    PYTHONPATH=rb_simulator/src python3 scripts/circle_tracking_benchmark.py \
      --root . \
      --mode start-local \
      --server rb_servo_server/build/hardware_free_gate/rb_servo_server \
      --server-config rb_servo_server/config/dual_simulator_tcp_acceptance.yaml \
      --left-config rb_simulator/config/left_rb3_730e.yaml \
      --right-config rb_simulator/config/right_rb3_730e.yaml \
      --arm left \
      --controller twist_stand \
      --plane xy \
      --profile safe_5cm_10s \
      --repeat 1 \
      --command-rate-hz 50 \
      --max-allowed-rms-error-m 0.01 \
      --max-allowed-p95-error-m 0.02 \
      --max-allowed-orientation-drift-rad 0.1 \
      --artifact-dir artifacts/circle_tracking/bench_circle_01
  else
    echo "codex_gate: skipping full circle tracking benchmark; set CODEX_RUN_CIRCLE_BENCHMARK=1 to enable"
  fi

  if [[ "${CODEX_RUN_GENE_STYLE_CIRCLE:-0}" == "1" ]]; then
    PYTHONPATH=rb_simulator/src python3 scripts/circle_tracking_benchmark.py \
      --root . \
      --mode start-local \
      --server rb_servo_server/build/hardware_free_gate/rb_servo_server \
      --server-config rb_servo_server/config/dual_simulator_circle_stress.yaml \
      --left-config rb_simulator/config/left_rb3_730e.yaml \
      --right-config rb_simulator/config/right_rb3_730e.yaml \
      --arm left \
      --controller twist_stand \
      --plane xy \
      --profile gene_15cm_4s \
      --allow-fast-stress \
      --repeat 1 \
      --command-rate-hz 100 \
      --artifact-dir artifacts/circle_tracking/gene_15cm_4s
  else
    echo "codex_gate: skipping GENE-style 15 cm / 4 s circle stress; set CODEX_RUN_GENE_STYLE_CIRCLE=1 to enable"
  fi
}

run_conservative_circle_benchmark_runtime() {
  if ! rb_servo_cpp_deps_available; then
    if [[ "${CODEX_SKIP_MISSING_CPP_DEPS:-0}" == "1" ]]; then
      echo "codex_gate: skipping circle benchmark runtime; required C++ deps are missing"
      return 0
    fi
    echo "ERROR: circle benchmark runtime requires yaml-cpp, nlohmann_json, Eigen3, and pinocchio" >&2
    return 1
  fi
  if ! loopback_socket_available; then
    echo "codex_gate: skipping circle benchmark runtime; AF_INET loopback sockets are unavailable"
    return 0
  fi

  build_servo_server_only
  PYTHONPATH=rb_simulator/src python3 scripts/circle_tracking_benchmark.py \
    --root . \
    --mode start-local \
    --server rb_servo_server/build/hardware_free_gate/rb_servo_server \
    --server-config rb_servo_server/config/dual_simulator_tcp_acceptance.yaml \
    --left-config rb_simulator/config/left_rb3_730e.yaml \
    --right-config rb_simulator/config/right_rb3_730e.yaml \
    --arm left \
    --controller twist_stand \
    --plane xy \
    --profile safe_5cm_10s \
    --repeat 1 \
    --command-rate-hz 50 \
    --max-allowed-rms-error-m 0.01 \
    --max-allowed-p95-error-m 0.02 \
    --max-allowed-orientation-drift-rad 0.1 \
    --artifact-dir artifacts/circle_tracking/bench_circle_01
}

run_cart_servo_01_gate() {
  run_shell_syntax_checks
  run_servo_gate_or_skip_missing_deps
  run_simulator_tests
  run_gui_tests
  run_policy_runner_tests
}

run_cart_servo_02_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q scripts
  PYTHONPATH=scripts python3 -m unittest discover scripts -p 'test_circle_tracking_benchmark.py'
  python3 scripts/circle_tracking_benchmark.py --help >/dev/null
  python3 scripts/compare_circle_benchmarks.py --help >/dev/null
  run_simulator_tests
  run_gui_tests
  run_policy_runner_tests
  if [[ "${CODEX_RUN_CIRCLE_BENCHMARK:-0}" == "1" ]]; then
    run_conservative_circle_benchmark_runtime
  else
    echo "codex_gate: skipping full circle tracking benchmark; set CODEX_RUN_CIRCLE_BENCHMARK=1 to enable"
  fi
}

run_cart_servo_03_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q scripts
  python3 scripts/circle_tracking_benchmark.py --help >/dev/null
  python3 scripts/compare_circle_benchmarks.py --help >/dev/null
  run_servo_gate_or_skip_missing_deps
  if [[ "${CODEX_RUN_CARTESIAN_ACCEPTANCE:-0}" == "1" ]]; then
    run_cart_harden_05_gate
  else
    echo "codex_gate: skipping full Cartesian simulator acceptance; set CODEX_RUN_CARTESIAN_ACCEPTANCE=1 to enable"
  fi
  if [[ "${CODEX_RUN_CIRCLE_BENCHMARK:-0}" == "1" ]]; then
    run_conservative_circle_benchmark_runtime
  else
    echo "codex_gate: skipping full circle tracking benchmark; set CODEX_RUN_CIRCLE_BENCHMARK=1 to enable"
  fi
}

run_bench_ablation_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q scripts
  python3 scripts/run_circle_ablation.py --help >/dev/null
  PYTHONPATH=scripts python3 -m unittest discover scripts -p 'test_run_circle_ablation.py'
  python3 scripts/circle_tracking_benchmark.py --help >/dev/null
  run_optional_python_help scripts/compare_circle_benchmarks.py
  run_python_surface_tests
  if [[ "${CODEX_RUN_CIRCLE_ABLATION:-0}" == "1" ]]; then
    local matrix
    matrix="${CODEX_CIRCLE_ABLATION_MATRIX:-configs/circle_ablation_15cm.yaml}"
    if [[ -f "${matrix}" ]]; then
      PYTHONPATH=rb_simulator/src python3 scripts/run_circle_ablation.py \
        --root . \
        --matrix "${matrix}" \
        --artifact-root artifacts/circle_tracking/ablation_gate \
        --max-workers 1
    else
      echo "codex_gate: skipping full circle ablation; matrix not found: ${matrix}"
    fi
  else
    echo "codex_gate: skipping full circle ablation; set CODEX_RUN_CIRCLE_ABLATION=1 to enable"
  fi
}

run_cart_tune_02_gate() {
  run_shell_syntax_checks
  run_servo_gate_or_skip_missing_deps
  run_python_surface_tests
  grep_existing "safe_5cm_10s|circle_15cm_16s|circle_15cm_8s|gene_15cm_4s" \
    scripts/circle_tracking_benchmark.py docs/runbooks/circle_tracking_benchmark.md rb_servo_server/config
  local circle_profiles=(
    rb_servo_server/config/dual_simulator_circle_baseline_15cm16s.yaml
    rb_servo_server/config/dual_simulator_circle_stress_15cm4s.yaml
    rb_servo_server/config/dual_simulator_circle_real_candidate_conservative.yaml
  )
  local profile
  for profile in "${circle_profiles[@]}"; do
    if [[ ! -f "${profile}" ]]; then
      echo "ERROR: missing circle tuning profile: ${profile}" >&2
      return 1
    fi
    grep_existing "backend_type:[[:space:]]*simulator" "${profile}"
    grep_existing "run_mode:[[:space:]]*simulation" "${profile}"
    grep_existing "allow_in_simulation:[[:space:]]*true" "${profile}"
    grep_existing "allow_in_real:[[:space:]]*false" "${profile}"
    local required_key
    for required_key in \
      "servo:" \
      "rate_hz:" \
      "state_pub_rate_hz:" \
      "velocity_target_integration:" \
      "path_kp_pos:" \
      "path_kp_ori:" \
      "twist_orientation_hold_kp:" \
      "twist_angular_deadband_rad_s:" \
      "velocity_damping:" \
      "max_twist_linear_m_s:" \
      "max_twist_angular_rad_s:" \
      "max_command_actual_error_deg:" \
      "command_actual_error_policy:"
    do
      grep_existing "${required_key}" "${profile}"
    done
  done
  grep_existing "simulator_baseline|simulator_stress|real_candidate_conservative" "${circle_profiles[@]}" docs/runbooks/circle_tracking_benchmark.md REVIEW.md README.md
  grep_absent "run_mode:[[:space:]]*real|backend_type:[[:space:]]*rbpodo|allow_in_real:[[:space:]]*true|172\\.28\\.60\\.200|172\\.28\\.60\\.201" "${circle_profiles[@]}"
  grep_existing "dual_simulator_circle_baseline_15cm16s|dual_simulator_circle_stress_15cm4s|dual_simulator_circle_real_candidate_conservative" \
    docs/runbooks/circle_tracking_benchmark.md README.md REVIEW.md
}

run_bench_circle_feedback_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q scripts
  PYTHONPATH=scripts python3 -m unittest discover scripts -p 'test_circle_tracking_benchmark.py'
  python3 scripts/circle_tracking_benchmark.py --help >/dev/null
  grep_existing "twist_stand_feedback|twist_local_feedback|feedback-kp-pos" \
    scripts/circle_tracking_benchmark.py docs/runbooks/circle_tracking_benchmark.md
  run_python_surface_tests
  if [[ "${CODEX_RUN_CIRCLE_BENCHMARK:-0}" == "1" ]]; then
    if ! rb_servo_cpp_deps_available; then
      if [[ "${CODEX_SKIP_MISSING_CPP_DEPS:-0}" == "1" ]]; then
        echo "codex_gate: skipping circle feedback benchmark runtime; required C++ deps are missing"
        return 0
      fi
      echo "ERROR: circle feedback benchmark runtime requires yaml-cpp, nlohmann_json, Eigen3, and pinocchio" >&2
      return 1
    fi
    if ! loopback_socket_available; then
      echo "codex_gate: skipping circle feedback benchmark runtime; AF_INET loopback sockets are unavailable"
      return 0
    fi
    build_servo_server_only
    PYTHONPATH=rb_simulator/src python3 scripts/circle_tracking_benchmark.py \
      --root . \
      --mode start-local \
      --server rb_servo_server/build/hardware_free_gate/rb_servo_server \
      --server-config rb_servo_server/config/dual_simulator_circle_baseline_15cm16s.yaml \
      --left-config rb_simulator/config/left_rb3_730e.yaml \
      --right-config rb_simulator/config/right_rb3_730e.yaml \
      --arm left \
      --controller twist_stand_feedback \
      --plane xy \
      --profile safe_5cm_10s \
      --repeat 1 \
      --command-rate-hz 50 \
      --max-allowed-rms-error-m 0.01 \
      --max-allowed-p95-error-m 0.02 \
      --max-allowed-orientation-drift-rad 0.1 \
      --artifact-dir artifacts/circle_tracking/bench_circle_feedback_01
  else
    echo "codex_gate: skipping full circle feedback benchmark; set CODEX_RUN_CIRCLE_BENCHMARK=1 to enable"
  fi
}

run_cart_circle_server_gate() {
  run_servo_gate_or_skip_missing_deps
  run_shell_syntax_checks
  run_python_surface_tests
  if [[ "${CODEX_RUN_CIRCLE_SERVER:-0}" == "1" ]]; then
    if [[ -f scripts/run_circle_server_benchmark.py ]]; then
      PYTHONPATH=rb_simulator/src python3 scripts/run_circle_server_benchmark.py \
        --root . \
        --artifact-dir artifacts/circle_tracking/server_circle_gate
    else
      echo "codex_gate: skipping server-side circle benchmark; scripts/run_circle_server_benchmark.py is not present"
    fi
  else
    echo "codex_gate: skipping server-side circle benchmark; set CODEX_RUN_CIRCLE_SERVER=1 to enable"
  fi
}

run_bench_report_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q scripts
  run_optional_python_help scripts/report_circle_benchmarks.py
  run_optional_python_help scripts/circle_benchmark_report.py
  run_optional_python_help scripts/generate_circle_benchmark_report.py
  grep_existing "baseline|stress|safe_5cm_10s|gene_15cm_4s" \
    docs/runbooks/circle_tracking_benchmark.md REVIEW.md
  echo "codex_gate: skipping full benchmark reporting run by default"
}

run_optional_rbscript_helper_tests() {
  local ran_any=0
  local pattern
  for pattern in 'test_rbscript*.py' 'test_rainbow*.py' 'test_rb_backend*.py'; do
    if find scripts -maxdepth 1 -name "${pattern}" -print -quit | grep -q .; then
      PYTHONPATH=scripts python3 -m unittest discover scripts -p "${pattern}"
      ran_any=1
    fi
  done
  if [[ "${ran_any}" != "1" ]]; then
    echo "codex_gate: optional rbscript/rainbow Python helper tests not present"
  fi
}

run_rbscript_tcp_01_gate() {
  run_shell_syntax_checks
  run_servo_gate_or_skip_missing_deps
  run_python_surface_tests
}

run_rbscript_tcp_02_gate() {
  run_shell_syntax_checks
  run_servo_gate_or_skip_missing_deps
  run_optional_rbscript_helper_tests
}

run_rbscript_ablation_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q scripts
  python3 scripts/rb_backend_ablation.py --help >/dev/null
  run_optional_python_help scripts/compare_backend_ablation.py
  run_optional_rbscript_helper_tests
  grep_existing "rb_backend_ablation.py|rbscript_tcp|command port.*5000|data port.*5001|no-motion" \
    docs/runbooks/rbscript_tcp_ablation.md REVIEW.md
  run_python_surface_tests
  if [[ "${CODEX_RUN_RBSCRIPT_ABLATION:-0}" == "1" ]]; then
    if [[ ! -f scripts/rb_backend_ablation.py ]]; then
      echo "ERROR: CODEX_RUN_RBSCRIPT_ABLATION=1 but scripts/rb_backend_ablation.py is missing" >&2
      return 1
    fi
    if [[ -z "${CODEX_RBSCRIPT_ABLATION_ARGS:-}" ]]; then
      echo "ERROR: CODEX_RUN_RBSCRIPT_ABLATION=1 requires CODEX_RBSCRIPT_ABLATION_ARGS with explicit simulator/read-only config" >&2
      return 1
    fi
    # shellcheck disable=SC2086
    python3 scripts/rb_backend_ablation.py ${CODEX_RBSCRIPT_ABLATION_ARGS}
  else
    echo "codex_gate: skipping rbscript backend ablation; set CODEX_RUN_RBSCRIPT_ABLATION=1 with explicit args to enable"
  fi
}

run_rbscript_rate_probe_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q scripts
  python3 scripts/rainbow_rate_probe.py --help >/dev/null
  run_optional_rbscript_helper_tests
  grep_existing "rainbow_rate_probe.py|M561|M568|M569|M570|disable_waiting_ack|200 Hz|200Hz" \
    docs/runbooks/rbscript_tcp_ablation.md REVIEW.md
  run_python_surface_tests
  if [[ "${CODEX_RUN_REAL_RATE_PROBE:-0}" == "1" ]]; then
    if [[ ! -f scripts/rainbow_rate_probe.py ]]; then
      echo "ERROR: CODEX_RUN_REAL_RATE_PROBE=1 but scripts/rainbow_rate_probe.py is missing" >&2
      return 1
    fi
    if [[ -z "${CODEX_RAINBOW_RATE_PROBE_ARGS:-}" ]]; then
      echo "ERROR: CODEX_RUN_REAL_RATE_PROBE=1 requires CODEX_RAINBOW_RATE_PROBE_ARGS with explicit config and safety preflight flags" >&2
      return 1
    fi
    # shellcheck disable=SC2086
    python3 scripts/rainbow_rate_probe.py ${CODEX_RAINBOW_RATE_PROBE_ARGS}
  else
    echo "codex_gate: skipping real Rainbow rate probe; set CODEX_RUN_REAL_RATE_PROBE=1 with explicit args to enable"
  fi
}

run_rbscript_doc_gate() {
  run_shell_syntax_checks
  grep_existing "rbscript_tcp" README.md REVIEW.md docs rb_servo_server/docs
  grep_existing "command[[:space:]_-]*port[^0-9]*5000" README.md REVIEW.md docs rb_servo_server/docs
  grep_existing "data[[:space:]_-]*port[^0-9]*5001" README.md REVIEW.md docs rb_servo_server/docs
  grep_existing "simulator/read-only first|simulator.*read-only.*first|read-only.*simulator" README.md REVIEW.md docs rb_servo_server/docs
  grep_existing "no UDP direct-to-controller|no UDP.*controller|UDP direct-to-controller" README.md REVIEW.md docs rb_servo_server/docs
}

run_rbpodo_accept_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q scripts
  run_optional_python_help scripts/rbpodo_servo_acceptance.py
  if [[ -f scripts/rbpodo_servo_acceptance.py ]]; then
    python3 scripts/rbpodo_servo_acceptance.py --self-test
  fi
  run_optional_python_help scripts/rbpodo_ack_rate_probe.py
  if [[ "${CODEX_RUN_RBPODO_ACCEPTANCE:-0}" == "1" ]]; then
    if [[ -z "${CODEX_RBPODO_ACCEPTANCE_ARGS:-}" ]]; then
      echo "ERROR: CODEX_RUN_RBPODO_ACCEPTANCE=1 requires CODEX_RBPODO_ACCEPTANCE_ARGS with explicit script arguments and safety preflight flags" >&2
      return 1
    fi
    if [[ -f scripts/rbpodo_servo_acceptance.py ]]; then
      # shellcheck disable=SC2086
      python3 scripts/rbpodo_servo_acceptance.py ${CODEX_RBPODO_ACCEPTANCE_ARGS}
    elif [[ -f scripts/rbpodo_ack_rate_probe.py ]]; then
      # shellcheck disable=SC2086
      python3 scripts/rbpodo_ack_rate_probe.py ${CODEX_RBPODO_ACCEPTANCE_ARGS}
    else
      echo "ERROR: CODEX_RUN_RBPODO_ACCEPTANCE=1 but no rbpodo acceptance script is present" >&2
      return 1
    fi
  else
    echo "codex_gate: skipping rbpodo real/controller acceptance; set CODEX_RUN_RBPODO_ACCEPTANCE=1 with explicit args to enable"
  fi
}

run_rbpodo_readonly_diag_gate() {
  run_shell_syntax_checks
  run_servo_gate_or_skip_missing_deps
  run_python_surface_tests
}

run_rbpodo_status_fields_gate() {
  run_servo_gate_or_skip_missing_deps
}

run_rbpodo_joint_wrap_gate() {
  run_servo_gate_or_skip_missing_deps
}

run_rbpodo_bringup_tools_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q scripts
  run_optional_python_help scripts/rbpodo_state_dump.py
  run_optional_python_help scripts/rbpodo_servo_acceptance.py
}

run_rbpodo_doc_gate() {
  run_shell_syntax_checks
  grep_existing "servo_t1_sec" README.md REVIEW.md docs rb_servo_server/docs rb_servo_server/config
  grep_existing "servo_t2_sec" README.md REVIEW.md docs rb_servo_server/docs rb_servo_server/config
  grep_existing "servo_alpha" README.md REVIEW.md docs rb_servo_server/docs rb_servo_server/config
  grep_existing "disable_waiting_ack" README.md REVIEW.md docs rb_servo_server/docs rb_servo_server/config
  grep_existing "ACK disabled" README.md REVIEW.md docs rb_servo_server/docs rb_servo_server/config
  grep_existing "100[[:space:]]*Hz|100Hz" README.md REVIEW.md docs rb_servo_server/docs rb_servo_server/config
  grep_existing "200[[:space:]]*Hz|200Hz" README.md REVIEW.md docs rb_servo_server/docs rb_servo_server/config
  grep_existing "RB_ALLOW_REAL_MOTION" README.md REVIEW.md AGENTS.md docs rb_servo_server/docs rb_servo_server/config
}

run_rbpodo_circle_config_gate() {
  run_shell_syntax_checks
  grep_existing "rbpodo" README.md REVIEW.md docs rb_servo_server/docs rb_servo_server/config configs
  grep_existing "pgmode simulation|controller simulation|operation_mode:[[:space:]]*simulation" \
    README.md REVIEW.md docs rb_servo_server/docs rb_servo_server/config configs
  grep_existing "send_servo_commands:[[:space:]]*false|send_servo_commands:[[:space:]]*true" \
    rb_servo_server/config configs docs/runbooks REVIEW.md
  grep_existing "RB_ALLOW_REAL_ROBOT" README.md REVIEW.md AGENTS.md docs rb_servo_server/docs rb_servo_server/config configs
  grep_existing "RB_ALLOW_REAL_MOTION" README.md REVIEW.md AGENTS.md docs rb_servo_server/docs rb_servo_server/config configs
  run_python_surface_tests
  run_servo_gate_or_skip_missing_deps
}

run_rbpodo_pgmode_sim_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q scripts tools
  if [[ -f tools/simulation_mode.sh ]]; then
    bash -n tools/simulation_mode.sh
  else
    echo "codex_gate: optional pgmode helper not present: tools/simulation_mode.sh"
  fi
  run_optional_python_help scripts/rainbow_pgmode.py
  if [[ "${CODEX_RUN_RBPODO_PGMODE_CHECK:-0}" == "1" ]]; then
    if [[ ! -f scripts/rainbow_pgmode.py ]]; then
      echo "ERROR: CODEX_RUN_RBPODO_PGMODE_CHECK=1 but scripts/rainbow_pgmode.py is missing" >&2
      return 1
    fi
    if [[ -z "${CODEX_RBPODO_PGMODE_CHECK_ARGS:-}" ]]; then
      echo "ERROR: CODEX_RUN_RBPODO_PGMODE_CHECK=1 requires CODEX_RBPODO_PGMODE_CHECK_ARGS with explicit controller IP, pgmode, and confirmation flags" >&2
      return 1
    fi
    # shellcheck disable=SC2086
    python3 scripts/rainbow_pgmode.py ${CODEX_RBPODO_PGMODE_CHECK_ARGS}
  else
    echo "codex_gate: skipping rbpodo pgmode simulation controller check; set CODEX_RUN_RBPODO_PGMODE_CHECK=1 with explicit args to enable"
  fi
}

run_rbpodo_reference_tcp_gate() {
  run_servo_gate_or_skip_missing_deps
  run_python_surface_tests
}

run_rbpodo_controller_sim_gate() {
  run_shell_syntax_checks
  run_servo_gate_or_skip_missing_deps
  run_optional_python_help scripts/rbpodo_servo_acceptance.py
  run_optional_python_help scripts/rbpodo_circle_tracking_benchmark.py
  if [[ "${CODEX_RUN_RBPODO_CONTROLLER_SIM:-0}" == "1" ]]; then
    if [[ -z "${CODEX_RBPODO_CONTROLLER_SIM_ARGS:-}" ]]; then
      echo "ERROR: CODEX_RUN_RBPODO_CONTROLLER_SIM=1 requires CODEX_RBPODO_CONTROLLER_SIM_ARGS with explicit script arguments and safety preflight flags" >&2
      return 1
    fi
    if [[ -f scripts/rbpodo_circle_tracking_benchmark.py ]]; then
      # shellcheck disable=SC2086
      python3 scripts/rbpodo_circle_tracking_benchmark.py ${CODEX_RBPODO_CONTROLLER_SIM_ARGS}
    elif [[ -f scripts/rbpodo_servo_acceptance.py ]]; then
      # shellcheck disable=SC2086
      python3 scripts/rbpodo_servo_acceptance.py ${CODEX_RBPODO_CONTROLLER_SIM_ARGS}
    else
      echo "ERROR: CODEX_RUN_RBPODO_CONTROLLER_SIM=1 but no rbpodo controller-simulation script is present" >&2
      return 1
    fi
  else
    echo "codex_gate: skipping rbpodo controller-simulation run; set CODEX_RUN_RBPODO_CONTROLLER_SIM=1 with explicit args to enable"
  fi
}

run_rbpodo_circle_bench_gate() {
  python3 -m compileall -q scripts
  run_optional_python_help scripts/rbpodo_circle_tracking_benchmark.py
  python3 scripts/circle_tracking_benchmark.py --help >/dev/null
  if [[ "${CODEX_RUN_RBPODO_CIRCLE:-0}" == "1" ]]; then
    if [[ ! -f scripts/rbpodo_circle_tracking_benchmark.py ]]; then
      echo "ERROR: CODEX_RUN_RBPODO_CIRCLE=1 but scripts/rbpodo_circle_tracking_benchmark.py is missing" >&2
      return 1
    fi
    if [[ -z "${CODEX_RBPODO_CIRCLE_ARGS:-}" ]]; then
      echo "ERROR: CODEX_RUN_RBPODO_CIRCLE=1 requires CODEX_RBPODO_CIRCLE_ARGS with explicit controller-simulation script arguments and safety preflight flags" >&2
      return 1
    fi
    # shellcheck disable=SC2086
    python3 scripts/rbpodo_circle_tracking_benchmark.py ${CODEX_RBPODO_CIRCLE_ARGS}
  else
    echo "codex_gate: skipping rbpodo controller-simulation circle benchmark; set CODEX_RUN_RBPODO_CIRCLE=1 with explicit args to enable"
  fi
}

run_rbpodo_circle_ablation_gate() {
  python3 -m compileall -q scripts
  run_optional_python_help scripts/run_rbpodo_circle_ablation.py
  if [[ "${CODEX_RUN_RBPODO_CIRCLE_ABLATION:-0}" == "1" ]]; then
    if [[ ! -f scripts/run_rbpodo_circle_ablation.py ]]; then
      echo "ERROR: CODEX_RUN_RBPODO_CIRCLE_ABLATION=1 but scripts/run_rbpodo_circle_ablation.py is missing" >&2
      return 1
    fi
    if [[ -z "${CODEX_RBPODO_CIRCLE_ABLATION_ARGS:-}" ]]; then
      echo "ERROR: CODEX_RUN_RBPODO_CIRCLE_ABLATION=1 requires CODEX_RBPODO_CIRCLE_ABLATION_ARGS with explicit matrix/script arguments and safety preflight flags" >&2
      return 1
    fi
    # shellcheck disable=SC2086
    python3 scripts/run_rbpodo_circle_ablation.py ${CODEX_RBPODO_CIRCLE_ABLATION_ARGS}
  else
    echo "codex_gate: skipping rbpodo controller-simulation circle ablation; set CODEX_RUN_RBPODO_CIRCLE_ABLATION=1 with explicit args to enable"
  fi
}

run_rbpodo_circle_report_gate() {
  run_optional_python_help scripts/generate_rbpodo_circle_report.py
  run_optional_python_help scripts/rbpodo_circle_report.py
  python3 scripts/generate_circle_benchmark_report.py --help >/dev/null
  for token in \
    "rbpodo" \
    "pgmode simulation" \
    "controller simulation" \
    "q_ref" \
    "tcp_reference" \
    "15cm" \
    "4s"
  do
    grep_existing "${token}" README.md REVIEW.md docs rb_servo_server/docs
  done
}

run_rbpodo_circle_profile_tuning_gate() {
  python3 -m compileall -q scripts
  python3 scripts/rbpodo_circle_tracking_benchmark.py --help >/dev/null
  PYTHONPATH=scripts python3 -m unittest discover scripts -p 'test_rbpodo_circle_tracking_benchmark.py'
  PYTHONPATH=scripts python3 -m unittest discover scripts -p 'test_circle_tracking_benchmark.py'
  echo "codex_gate: skipping rbpodo controller-simulation benchmark run by default"
}

run_rbpodo_circle_ablation_overrides_gate() {
  python3 -m compileall -q scripts
  python3 scripts/run_rbpodo_circle_ablation.py --help >/dev/null
  PYTHONPATH=scripts python3 -m unittest discover scripts -p 'test_rbpodo_circle_ablation.py'
  PYTHONPATH=scripts python3 -m unittest discover scripts -p 'test_run_circle_ablation.py'
  if [[ "${CODEX_RUN_RBPODO_CIRCLE_ABLATION:-0}" == "1" ]]; then
    if [[ ! -f scripts/run_rbpodo_circle_ablation.py ]]; then
      echo "ERROR: CODEX_RUN_RBPODO_CIRCLE_ABLATION=1 but scripts/run_rbpodo_circle_ablation.py is missing" >&2
      return 1
    fi
    if [[ -z "${CODEX_RBPODO_CIRCLE_ABLATION_ARGS:-}" ]]; then
      echo "ERROR: CODEX_RUN_RBPODO_CIRCLE_ABLATION=1 requires CODEX_RBPODO_CIRCLE_ABLATION_ARGS with explicit matrix/script arguments and safety preflight flags" >&2
      return 1
    fi
    # shellcheck disable=SC2086
    python3 scripts/run_rbpodo_circle_ablation.py ${CODEX_RBPODO_CIRCLE_ABLATION_ARGS}
  else
    echo "codex_gate: skipping rbpodo controller-simulation circle ablation; set CODEX_RUN_RBPODO_CIRCLE_ABLATION=1 with explicit args to enable"
  fi
}

run_rbpodo_circle_stage2_matrices_gate() {
  python3 -m compileall -q scripts
  python3 scripts/run_rbpodo_circle_ablation.py --help >/dev/null
  run_yaml_parse_checks_if_available configs/rbpodo_circle_ablation/*.yaml
  run_rbpodo_circle_matrix_schema_checks
  PYTHONPATH=scripts python3 -m unittest discover scripts -p 'test_rbpodo_circle_ablation.py'
  echo "codex_gate: skipping rbpodo controller-simulation matrix run by default"
}

run_rbpodo_circle_matrix_schema_checks() {
  local matrices=()
  local path
  for path in configs/rbpodo_circle_ablation/*.yaml; do
    if [[ -e "${path}" ]]; then
      matrices+=("${path}")
    fi
  done

  if [[ "${#matrices[@]}" -eq 0 ]]; then
    echo "codex_gate: skipping rbpodo circle matrix schema checks; no matrix YAML files found"
    return 0
  fi

  PYTHONPATH=scripts python3 - "${matrices[@]}" <<'PY'
import sys
from pathlib import Path

import run_rbpodo_circle_ablation as ablation

for raw_path in sys.argv[1:]:
    path = Path(raw_path)
    experiments = ablation.load_matrix(path)
    for index, experiment in enumerate(experiments, start=1):
        ablation.validate_experiment(experiment, index)
PY
}

run_rbpodo_circle_tuning_report_gate() {
  python3 -m compileall -q scripts
  python3 scripts/generate_rbpodo_circle_report.py --help >/dev/null
  PYTHONPATH=scripts python3 -m unittest discover scripts -p 'test_circle_benchmark_report.py'
  echo "codex_gate: skipping rbpodo controller-simulation report generation by default"
}

run_tools_shell_syntax_checks() {
  local tool
  local found=0
  for tool in tools/*.sh; do
    if [[ ! -e "${tool}" ]]; then
      continue
    fi
    bash -n "${tool}"
    found=1
  done
  if [[ "${found}" != "1" ]]; then
    echo "codex_gate: skipping tools shell syntax checks; no tools/*.sh files found"
  fi
}

run_rbpodo_circle_wrapper_help_checks() {
  local wrapper
  for wrapper in \
    tools/create_rbpodo_circle_local_configs.sh \
    tools/rbpodo_circle_prepare.sh \
    tools/rbpodo_circle_benchmark.sh \
    tools/rbpodo_circle_gui.sh \
    tools/simulation_mode.sh
  do
    if [[ -f "${wrapper}" ]]; then
      bash "${wrapper}" --help >/dev/null
    else
      echo "codex_gate: optional wrapper not present: ${wrapper}"
    fi
  done
}

run_rbpodo_circle_tune_runners_gate() {
  run_tools_shell_syntax_checks
  run_rbpodo_circle_wrapper_help_checks
  echo "codex_gate: skipping rbpodo controller-simulation wrapper runs by default"
}

run_rbpodo_measure_state_parity_gate() {
  run_shell_syntax_checks
  run_servo_gate_or_skip_missing_deps
  python3 -m compileall -q scripts
  python3 scripts/rbpodo_state_dump.py --help >/dev/null
  python3 scripts/rbpodo_state_dump.py --self-test
  run_optional_script_tests 'test_rbpodo_state_dump.py'
  run_optional_script_tests 'test_rbpodo_measure_state_parity.py'
  run_optional_rbpodo_measurement_readonly scripts/rbpodo_state_dump.py
}

run_rbpodo_measure_raw_data_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q scripts
  python3 scripts/rainbow_rate_probe.py --help >/dev/null
  run_optional_python_help scripts/rbpodo_raw_data_capture.py
  PYTHONPATH=scripts python3 -m unittest discover scripts -p 'test_rainbow_rate_probe.py'
  run_optional_script_tests 'test_rbpodo_raw_data*.py'
  run_optional_script_tests 'test_rainbow_raw_data*.py'
  run_optional_rbpodo_measurement_readonly \
    scripts/rainbow_rate_probe.py \
    "--backend rbscript_tcp" \
    "--mode read_state" \
    "--capture-raw-data-port"
}

run_rbpodo_measure_timestamp_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q scripts
  run_optional_python_help scripts/rbpodo_timestamp_audit.py
  run_optional_python_help scripts/rbpodo_measure_timestamp_audit.py
  run_optional_script_tests 'test_rbpodo_timestamp*.py'
  run_optional_script_tests 'test_rbpodo_measure_timestamp*.py'
  echo "codex_gate: skipping rbpodo measurement controller run by default"
}

run_rbpodo_circle_error_decomp_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q scripts
  python3 scripts/generate_rbpodo_circle_report.py --help >/dev/null
  run_optional_python_help scripts/rbpodo_circle_error_decomposition.py
  run_optional_python_help scripts/decompose_rbpodo_circle_error.py
  PYTHONPATH=scripts python3 -m unittest discover scripts -p 'test_rbpodo_circle_tracking_benchmark.py'
  PYTHONPATH=scripts python3 -m unittest discover scripts -p 'test_circle_benchmark_report.py'
  run_optional_script_tests 'test_rbpodo_circle_error*.py'
  echo "codex_gate: skipping rbpodo controller-simulation circle benchmark by default"
}

run_rbpodo_measure_reliability_report_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q scripts
  run_optional_python_help scripts/rbpodo_measure_reliability_report.py
  run_optional_python_help scripts/generate_rbpodo_measurement_report.py
  python3 scripts/generate_rbpodo_circle_report.py --help >/dev/null
  run_optional_script_tests 'test_rbpodo_measure_reliability*.py'
  for token in \
    "diagnostics_suspect" \
    "tcp_ref_stand" \
    "lower bound" \
    "timestamp alignment" \
    "q_ref" \
    "raw 5001"
  do
    grep_existing "${token}" README.md REVIEW.md docs rb_servo_server/docs scripts
  done
  echo "codex_gate: skipping rbpodo measurement report generation by default"
}

run_rbpodo_p0_measurement_common_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q scripts
}

run_p0_parity_repair_gate() {
  run_rbpodo_p0_measurement_common_gate
  python3 scripts/rbpodo_state_parity_check.py --help >/dev/null
  python3 scripts/rbpodo_state_dump.py --help >/dev/null
  run_optional_script_tests 'test_rbpodo_state_parity_check.py'
  run_optional_script_tests 'test_rbpodo_state_dump.py'
  run_optional_rbpodo_measurement_readonly \
    scripts/rbpodo_state_parity_check.py \
    "--ips" \
    "--state-endpoint" \
    "--artifact-dir"
}

run_p0_diagnostics_rootcause_gate() {
  run_rbpodo_p0_measurement_common_gate
  python3 scripts/rbpodo_state_dump.py --help >/dev/null
  python3 scripts/rbpodo_state_parity_check.py --help >/dev/null
  python3 scripts/rainbow_data_port_capture.py --help >/dev/null
  run_optional_script_tests 'test_rbpodo_state_dump.py'
  run_optional_script_tests 'test_rbpodo_state_parity_check.py'
  run_optional_script_tests 'test_rainbow_data_port_capture.py'
  run_optional_rbpodo_measurement_readonly \
    scripts/rainbow_data_port_capture.py \
    "--ips" \
    "--artifact-dir" \
    "--also-rbpodo-python"
}

run_p0_raw_payload_fixture_gate() {
  run_rbpodo_p0_measurement_common_gate
  python3 scripts/rainbow_data_port_capture.py --help >/dev/null
  python3 scripts/rainbow_rate_probe.py --help >/dev/null
  PYTHONPATH=scripts python3 -m unittest discover scripts -p 'test_rainbow_data_port_capture.py'
  run_optional_script_tests 'test_rainbow_rate_probe.py'
  run_optional_rbpodo_measurement_readonly \
    scripts/rainbow_data_port_capture.py \
    "--ips" \
    "--artifact-dir"
}

run_p0_measurement_gating_gate() {
  run_rbpodo_p0_measurement_common_gate
  python3 scripts/timestamp_alignment_audit.py --help >/dev/null
  python3 scripts/generate_circle_benchmark_report.py --help >/dev/null
  python3 scripts/generate_rbpodo_measurement_reliability_report.py --help >/dev/null
  run_optional_python_help scripts/error_decomposition.py
  PYTHONPATH=scripts python3 -m unittest discover scripts -p 'test_timestamp_alignment_audit.py'
  PYTHONPATH=scripts python3 -m unittest discover scripts -p 'test_circle_error_decomposition.py'
  PYTHONPATH=scripts python3 -m unittest discover scripts -p 'test_circle_benchmark_report.py'
  for token in \
    "diagnostics_suspect" \
    "tcp_ref_stand" \
    "lower bound" \
    "timestamp alignment" \
    "q_ref" \
    "raw 5001"
  do
    grep_existing "${token}" README.md REVIEW.md docs rb_servo_server/docs scripts
  done
  echo "codex_gate: skipping rbpodo measurement controller run by default"
}

run_optional_rbpodo_500hz_controller_sim() {
  if [[ "${CODEX_RUN_RBPODO_500HZ:-0}" != "1" ]]; then
    echo "codex_gate: skipping rbpodo 500Hz controller-simulation probe; set CODEX_RUN_RBPODO_500HZ=1 with explicit CODEX_RBPODO_500HZ_ARGS to enable"
    return 0
  fi
  if [[ -z "${CODEX_RBPODO_500HZ_ARGS:-}" ]]; then
    echo "ERROR: CODEX_RUN_RBPODO_500HZ=1 requires CODEX_RBPODO_500HZ_ARGS with explicit controller-simulation arguments" >&2
    return 1
  fi
  for required_arg in \
    "--backend rbpodo" \
    "--mode servo_j_simulation_only" \
    "--rates 100,500" \
    "--allow-simulation-servo-j" \
    "--artifact-dir" \
    "--i-understand-this-connects-to-real-controller"
  do
    if [[ "${CODEX_RBPODO_500HZ_ARGS}" != *"${required_arg}"* ]]; then
      echo "ERROR: CODEX_RBPODO_500HZ_ARGS must include ${required_arg}" >&2
      return 1
    fi
  done
  if [[ "${CODEX_RBPODO_500HZ_ARGS}" != *"--verify-pgmode-simulation"* && "${CODEX_RBPODO_500HZ_ARGS}" != *"--set-pgmode-simulation"* ]]; then
    echo "ERROR: CODEX_RBPODO_500HZ_ARGS must include --verify-pgmode-simulation or --set-pgmode-simulation" >&2
    return 1
  fi
  if [[ "${CODEX_RBPODO_500HZ_ARGS}" == *"rbscript_tcp"* || "${CODEX_RBPODO_500HZ_ARGS}" == *"tiny_joint_motion"* || "${CODEX_RBPODO_500HZ_ARGS}" == *"--allow-ack-disabled"* ]]; then
    echo "ERROR: rbpodo 500Hz gates only allow ACK-on rbpodo pgmode-simulation probes; rbscript_tcp, tiny physical motion, and ACK-off are out of scope" >&2
    return 1
  fi
  if [[ "${RB_ALLOW_REAL_CARTESIAN:-0}" == "1" ]]; then
    echo "ERROR: RB_ALLOW_REAL_CARTESIAN must not be set for rbpodo 500Hz controller-simulation gates" >&2
    return 1
  fi
  # shellcheck disable=SC2086
  python3 scripts/rainbow_rate_probe.py ${CODEX_RBPODO_500HZ_ARGS}
}

run_rbpodo_500hz_common_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q scripts
  python3 scripts/rainbow_rate_probe.py --help >/dev/null
  python3 scripts/rbpodo_servo_acceptance.py --help >/dev/null
  python3 scripts/rbpodo_circle_tracking_benchmark.py --help >/dev/null
  run_yaml_parse_checks_if_available \
    rb_servo_server/config/dual_real_rbpodo_readonly.example.yaml \
    rb_servo_server/config/dual_real_rbpodo_sim_noop_*.example.yaml \
    rb_servo_server/config/dual_real_rbpodo_circle_15cm*.example.yaml \
    configs/rbpodo_circle_ablation/*.yaml
  grep_existing "100,500|500 Hz|500Hz" scripts/rainbow_rate_probe.py docs/runbooks/rbpodo_controller_sim_circle.md REVIEW.md
  grep_existing "pgmode simulation|controller pgmode simulation only|operation_mode:[[:space:]]*simulation" \
    docs/runbooks/rbpodo_controller_sim_circle.md rb_servo_server/config/dual_real_rbpodo_*.example.yaml
}

run_rbpodo_500hz_config_gate() {
  run_rbpodo_500hz_common_gate
  echo "codex_gate: skipping rbpodo 500Hz controller run by default"
}

run_rbpodo_500hz_accept_gate() {
  run_rbpodo_500hz_common_gate
  PYTHONPATH=scripts python3 -m unittest discover scripts -p 'test_rainbow_rate_probe.py'
  run_optional_rbpodo_500hz_controller_sim
}

run_rbpodo_500hz_circle_matrix_gate() {
  run_rbpodo_500hz_common_gate
  python3 scripts/run_rbpodo_circle_ablation.py --help >/dev/null
  run_rbpodo_circle_matrix_schema_checks
  PYTHONPATH=scripts python3 -m unittest discover scripts -p 'test_rbpodo_circle_ablation.py'
  echo "codex_gate: skipping rbpodo 500Hz circle matrix run by default; set CODEX_RUN_RBPODO_500HZ=1 for the explicit rate probe only"
}

run_rbpodo_500hz_report_gate() {
  run_rbpodo_500hz_common_gate
  python3 scripts/generate_circle_benchmark_report.py --help >/dev/null
  run_optional_python_help scripts/generate_rbpodo_measurement_reliability_report.py
  PYTHONPATH=scripts python3 -m unittest discover scripts -p 'test_circle_benchmark_report.py'
  echo "codex_gate: skipping rbpodo 500Hz report generation by default"
}

run_optional_rbpodo_async_500hz_controller_sim() {
  local script="$1"
  local require_cartesian="${2:-0}"

  if [[ "${CODEX_RUN_RBPODO_ASYNC_500HZ:-0}" != "1" ]]; then
    echo "codex_gate: skipping rbpodo async ACK-supervised 500Hz controller-simulation run; set CODEX_RUN_RBPODO_ASYNC_500HZ=1 with explicit CODEX_RBPODO_ASYNC_500HZ_ARGS to enable"
    return 0
  fi
  if [[ ! -f "${script}" ]]; then
    echo "ERROR: CODEX_RUN_RBPODO_ASYNC_500HZ=1 but ${script} is missing" >&2
    return 1
  fi
  if [[ -z "${CODEX_RBPODO_ASYNC_500HZ_ARGS:-}" ]]; then
    echo "ERROR: CODEX_RUN_RBPODO_ASYNC_500HZ=1 requires CODEX_RBPODO_ASYNC_500HZ_ARGS with explicit controller-simulation arguments" >&2
    return 1
  fi
  for required_arg in \
    "--artifact" \
    "--i-understand-this-connects-to-real-controller" \
    "--i-confirm-controller-is-in-pgmode-simulation"
  do
    if [[ "${CODEX_RBPODO_ASYNC_500HZ_ARGS}" != *"${required_arg}"* ]]; then
      echo "ERROR: CODEX_RBPODO_ASYNC_500HZ_ARGS must include ${required_arg}" >&2
      return 1
    fi
  done
  if [[ "${CODEX_RBPODO_ASYNC_500HZ_ARGS}" != *"--verify-pgmode-simulation"* && \
        "${CODEX_RBPODO_ASYNC_500HZ_ARGS}" != *"--set-pgmode-simulation"* && \
        "${CODEX_RBPODO_ASYNC_500HZ_ARGS}" != *"--pgmode-summary-json"* ]]; then
    echo "ERROR: CODEX_RBPODO_ASYNC_500HZ_ARGS must include pgmode simulation evidence" >&2
    return 1
  fi
  if [[ "${CODEX_RBPODO_ASYNC_500HZ_ARGS}" != *"--async-ack-supervised"* && \
        "${CODEX_RBPODO_ASYNC_500HZ_ARGS}" != *"--ack-mode async-supervised"* && \
        "${CODEX_RBPODO_ASYNC_500HZ_ARGS}" != *"--ack-mode=async-supervised"* ]]; then
    echo "ERROR: CODEX_RBPODO_ASYNC_500HZ_ARGS must request an explicit async ACK-supervised mode" >&2
    return 1
  fi
  if [[ "${CODEX_RBPODO_ASYNC_500HZ_ARGS}" == *"--disable-waiting-ack-diagnostic"* || \
        "${CODEX_RBPODO_ASYNC_500HZ_ARGS}" == *"--allow-ack-disabled"* || \
        "${CODEX_RBPODO_ASYNC_500HZ_ARGS}" == *"disable_waiting_ack"* || \
        "${CODEX_RBPODO_ASYNC_500HZ_ARGS}" == *"socket_send_only"* || \
        "${CODEX_RBPODO_ASYNC_500HZ_ARGS}" == *"ackoff"* || \
        "${CODEX_RBPODO_ASYNC_500HZ_ARGS}" == *"ack-off"* || \
        "${CODEX_RBPODO_ASYNC_500HZ_ARGS}" == *"operation_mode: real"* ]]; then
    echo "ERROR: rbpodo async ACK-supervised gates do not allow ACK-off/socket-send-only or operation_mode=real evidence" >&2
    return 1
  fi
  run_rbpodo_async_controller_config_preflight
  for required_env in \
    RB_ALLOW_REAL_ROBOT \
    RB_ALLOW_REAL_MOTION \
    RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION \
    RB_RBPODO_PGMODE_SIMULATION_CONFIRMED \
    RB_ALLOW_RBPODO_ASYNC_ACK_SUPERVISED
  do
    if [[ "${!required_env:-0}" != "1" ]]; then
      echo "ERROR: ${required_env}=1 is required for opt-in rbpodo async ACK-supervised controller-simulation gates" >&2
      return 1
    fi
  done
  if [[ "${require_cartesian}" == "1" && "${RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN:-0}" != "1" ]]; then
    echo "ERROR: RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN=1 is required for opt-in rbpodo async circle matrix gates" >&2
    return 1
  fi
  if [[ "${RB_ALLOW_REAL_CARTESIAN:-0}" == "1" ]]; then
    echo "ERROR: RB_ALLOW_REAL_CARTESIAN must not be set for rbpodo async ACK-supervised controller-simulation gates" >&2
    return 1
  fi

  # shellcheck disable=SC2086
  python3 "${script}" ${CODEX_RBPODO_ASYNC_500HZ_ARGS}
}

run_rbpodo_async_controller_config_preflight() {
  python3 - "$REPO_ROOT" <<'PY'
import os
import shlex
import sys
from pathlib import Path

try:
    import yaml
except ModuleNotFoundError:
    print(
        "ERROR: PyYAML is required to verify async rbpodo controller-simulation configs",
        file=sys.stderr,
    )
    raise SystemExit(1)

repo = Path(sys.argv[1])
try:
    args = shlex.split(os.environ.get("CODEX_RBPODO_ASYNC_500HZ_ARGS", ""))
except ValueError as exc:
    print(f"ERROR: failed to parse CODEX_RBPODO_ASYNC_500HZ_ARGS: {exc}", file=sys.stderr)
    raise SystemExit(1)

def arg_values(names):
    values = []
    i = 0
    while i < len(args):
        item = args[i]
        for name in names:
            if item == name and i + 1 < len(args):
                values.append(args[i + 1])
                i += 1
                break
            prefix = name + "="
            if item.startswith(prefix):
                values.append(item[len(prefix):])
                break
        i += 1
    return values

def as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return False

def resolve(path_text):
    path = Path(path_text)
    if not path.is_absolute():
        path = repo / path
    return path

def load_yaml(path):
    if not path.exists():
        print(f"ERROR: async rbpodo gate config path does not exist: {path}", file=sys.stderr)
        raise SystemExit(1)
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    return data if isinstance(data, dict) else {}

def check_config(path):
    data = load_yaml(path)
    for arm_key in ("left_robot", "right_robot"):
        arm = data.get(arm_key)
        if not isinstance(arm, dict):
            continue
        operation_mode = str(arm.get("operation_mode", "")).strip().lower()
        if operation_mode == "real":
            print(
                f"ERROR: async rbpodo gates refuse operation_mode=real in {path}:{arm_key}",
                file=sys.stderr,
            )
            raise SystemExit(1)
        if as_bool(arm.get("disable_waiting_ack")):
            print(
                f"ERROR: async rbpodo gates refuse disable_waiting_ack=true in {path}:{arm_key}",
                file=sys.stderr,
            )
            raise SystemExit(1)
    cartesian = data.get("cartesian_control")
    if isinstance(cartesian, dict) and as_bool(cartesian.get("allow_in_real")):
        print(
            f"ERROR: async rbpodo gates refuse cartesian_control.allow_in_real=true in {path}",
            file=sys.stderr,
        )
        raise SystemExit(1)

def check_matrix(path):
    data = load_yaml(path)
    experiments = data.get("experiments")
    if not isinstance(experiments, list):
        print(f"ERROR: async rbpodo matrix must contain an experiments list: {path}", file=sys.stderr)
        raise SystemExit(1)
    for item in experiments:
        if not isinstance(item, dict):
            continue
        config = item.get("config")
        if config:
            check_config(resolve(str(config)))
        overrides = item.get("config_overrides")
        if isinstance(overrides, dict):
            for key, value in overrides.items():
                key_text = str(key)
                if key_text.endswith(".operation_mode") and str(value).strip().lower() == "real":
                    print(
                        f"ERROR: async rbpodo gates refuse operation_mode=real override in {path}",
                        file=sys.stderr,
                    )
                    raise SystemExit(1)
                if key_text.endswith(".disable_waiting_ack") and as_bool(value):
                    print(
                        f"ERROR: async rbpodo gates refuse disable_waiting_ack=true override in {path}",
                        file=sys.stderr,
                    )
                    raise SystemExit(1)
                if key_text == "cartesian_control.allow_in_real" and as_bool(value):
                    print(
                        f"ERROR: async rbpodo gates refuse cartesian_control.allow_in_real=true override in {path}",
                        file=sys.stderr,
                    )
                    raise SystemExit(1)

config_paths = [resolve(value) for value in arg_values(("--config", "--server-config"))]
matrix_paths = [resolve(value) for value in arg_values(("--matrix",))]
if not config_paths and not matrix_paths:
    print(
        "ERROR: async rbpodo controller runs must provide --config, --server-config, or --matrix",
        file=sys.stderr,
    )
    raise SystemExit(1)
for config_path in config_paths:
    check_config(config_path)
for matrix_path in matrix_paths:
    check_matrix(matrix_path)
PY
}

run_rbpodo_async_common_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q scripts
  run_optional_python_help scripts/rainbow_rate_probe.py
  run_optional_python_help scripts/rbpodo_500hz_acceptance.py
  run_optional_python_help scripts/rbpodo_circle_tracking_benchmark.py
  run_optional_python_help scripts/run_rbpodo_circle_ablation.py
  grep_existing "500 Hz|ACK|pgmode simulation" \
    docs/runbooks/rbpodo_500hz_acceptance.md REVIEW.md scripts/rbpodo_500hz_acceptance.py
}

run_rbpodo_async_contract_probe_gate() {
  run_rbpodo_async_common_gate
  run_optional_script_tests 'test_rainbow_rate_probe.py'
  run_optional_script_tests 'test_rbpodo_500hz_acceptance.py'
  echo "codex_gate: skipping rbpodo async ACK-supervised controller probe by default"
}

run_rbpodo_async_cpp_gate() {
  run_rbpodo_async_common_gate
  run_servo_gate_or_skip_missing_deps
  run_optional_script_tests 'test_rbpodo_500hz_acceptance.py'
  echo "codex_gate: skipping rbpodo async ACK-supervised controller run by default"
}

run_rbpodo_async_acceptance_gate() {
  run_rbpodo_async_common_gate
  python3 scripts/rbpodo_500hz_acceptance.py --help >/dev/null
  PYTHONPATH=scripts python3 -m unittest discover scripts -p 'test_rbpodo_500hz_acceptance.py'
  run_optional_rbpodo_async_500hz_controller_sim scripts/rbpodo_500hz_acceptance.py 0
}

run_rbpodo_async_circle_matrix_gate() {
  run_rbpodo_async_common_gate
  python3 scripts/run_rbpodo_circle_ablation.py --help >/dev/null
  run_rbpodo_circle_matrix_schema_checks
  PYTHONPATH=scripts python3 -m unittest discover scripts -p 'test_rbpodo_circle_ablation.py'
  run_optional_rbpodo_async_500hz_controller_sim scripts/run_rbpodo_circle_ablation.py 1
}

run_rbpodo_async_report_gate() {
  run_rbpodo_async_common_gate
  run_optional_python_help scripts/generate_rbpodo_500hz_report.py
  run_optional_python_help scripts/generate_circle_benchmark_report.py
  run_optional_python_help scripts/generate_rbpodo_measurement_reliability_report.py
  run_optional_script_tests 'test_500hz_report.py'
  run_optional_script_tests 'test_circle_benchmark_report.py'
  echo "codex_gate: skipping rbpodo async ACK-supervised report generation by default"
}

run_rbpodo_async_runbook_gate() {
  run_rbpodo_async_common_gate
  grep_existing "async|ACK-supervised|500 Hz|pgmode simulation" \
    docs/runbooks/rbpodo_500hz_acceptance.md REVIEW.md
  echo "codex_gate: skipping rbpodo async ACK-supervised controller run by default"
}

run_ackon500_followup_common_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q scripts
  run_optional_python_help scripts/rbpodo_circle_tracking_benchmark.py
  run_optional_python_help scripts/run_rbpodo_circle_ablation.py
  run_optional_python_help scripts/generate_ackon500_gene_goal_report.py
  run_optional_python_help scripts/generate_circle_benchmark_report.py
  run_optional_python_help scripts/generate_rbpodo_500hz_report.py
  run_optional_python_help scripts/generate_rbpodo_measurement_reliability_report.py
  run_yaml_parse_checks_if_available configs/rbpodo_circle_ablation/*.yaml
  run_optional_script_tests 'test_*.py'
  run_gui_tests
  run_policy_runner_tests
  run_simulator_tests
}

run_ackon500_rate_accounting_gate() {
  run_ackon500_followup_common_gate
  run_servo_gate_or_skip_missing_deps
}

run_benchmark_lane_canonicalize_gate() {
  run_ackon500_followup_common_gate
  run_servo_gate_or_skip_missing_deps
}

run_ackon500_best_profile_promotion_gate() {
  run_ackon500_followup_common_gate
  bash -n tools/rbpodo_ackon500_gene_goal.sh
  bash -n tools/create_rbpodo_circle_local_configs.sh
  grep_existing "sdk_ack_worker" \
    tools/rbpodo_ackon500_gene_goal.sh \
    tools/create_rbpodo_circle_local_configs.sh \
    configs/rbpodo_circle_ablation \
    rb_servo_server/config
  grep_existing "not physical real|controller-reference lower-bound|physical_motion_expected" \
    docs/runbooks/rbpodo_500hz_acceptance.md \
    docs/runbooks/rbpodo_controller_sim_circle.md \
    REVIEW.md \
    README.md
}

run_ackon500_repeatability_validation_gate() {
  run_ackon500_followup_common_gate
  run_yaml_parse_checks_if_available configs/rbpodo_circle_ablation/ackon500_gene_repeatability.yaml
  grep_existing "repeatability|repeatable_pass|not_repeatable|insufficient_evidence" \
    scripts docs/runbooks REVIEW.md configs/rbpodo_circle_ablation
}

run_physical_readiness_blockers_clarity_gate() {
  run_ackon500_followup_common_gate
  grep_existing "physical_readiness|physical_tracking_result|controller-reference lower-bound|not physical" \
    scripts docs/runbooks REVIEW.md README.md
}

run_gene_umi_control_defaults_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q scripts
  run_required_python_help scripts/validate_control_defaults.py
  run_required_script_tests 'test_validate_control_defaults.py'
  run_yaml_parse_checks_if_available configs/control_defaults/gene_26_5_ackon500_controller_sim.yaml
  python3 scripts/validate_control_defaults.py \
    --defaults configs/control_defaults/gene_26_5_ackon500_controller_sim.yaml
  grep_existing "controller_sim_high_performance_gene_26_5|physical_real_conservative_seed" \
    configs/control_defaults/gene_26_5_ackon500_controller_sim.yaml
  grep_existing "The GENE 26.5 / ACKON500 default is a controller-simulation high-performance default only" \
    docs/runbooks/rbpodo_controller_sim_circle.md \
    docs/runbooks/policy_data_collection.md \
    REVIEW.md
}

run_gene_umi_controller_sim_repeatability_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q scripts
  bash -n tools/rbpodo_ackon500_gene_goal.sh
  run_required_python_help scripts/generate_ackon500_gene_goal_report.py
  run_yaml_parse_checks_if_available configs/rbpodo_circle_ablation/ackon500_gene_repeatability.yaml
  run_required_script_tests 'test_*ackon500*.py'
  tools/rbpodo_ackon500_gene_goal.sh \
    --profile repeatability \
    --dry-run \
    --with-required-env \
    --i-understand-this-connects-to-real-controller \
    --i-confirm-controller-is-in-pgmode-simulation
  grep_existing "best_left_run01|best_right_run01|repeatability|physical_readiness" \
    configs/rbpodo_circle_ablation scripts docs/runbooks REVIEW.md
}

run_gene_umi_physical_transition_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q scripts
  run_required_python_help scripts/rbpodo_physical_transition_acceptance.py
  run_required_python_help scripts/generate_rbpodo_physical_transition_report.py
  run_required_script_tests 'test_*physical*transition*.py'
  run_yaml_parse_checks_if_available rb_servo_server/config/dual_real_rbpodo_physical_*.example.yaml
  grep_existing "pgmode_real_transition|tiny_cartesian|slow_physical_circle|physical_readiness" \
    scripts docs/runbooks rb_servo_server/config
  grep_existing "send_servo_commands:[[:space:]]*false" rb_servo_server/config/dual_real_rbpodo_physical_*.example.yaml
  grep_existing "allow_in_real:[[:space:]]*false" rb_servo_server/config/dual_real_rbpodo_physical_*.example.yaml
  grep_existing "tracking_source:[[:space:]]*tcp_actual_stand|tcp_actual_stand" scripts docs/runbooks
}

run_gene_umi_hdf5_audit_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q policy_runner/policy_runner scripts
  run_policy_runner_help hdf5-audit
  run_required_policy_runner_tests 'test_flow_dataset.py'
  run_required_policy_runner_tests 'test_hdf5_audit*.py'
  grep_existing "robotics_lab\\.policy_runner\\.hdf5_audit\\.v1|DatasetManifest|dataset_manifest" \
    policy_runner/policy_runner policy_runner/tests docs README.md
  if [[ -f episode_002.hdf5 ]]; then
    PYTHONPATH=policy_runner python3 -m policy_runner hdf5-audit \
      --episodes-dir episode_002.hdf5 \
      --output-json /tmp/robotics_lab_hdf5_audit_gate.json \
      --output-md /tmp/robotics_lab_hdf5_audit_gate.md
  else
    echo "codex_gate: skipping uploaded episode audit smoke; episode_002.hdf5 is absent"
  fi
}

run_gene_umi_bimanual_collection_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q policy_runner/policy_runner scripts
  run_policy_runner_help umi-import
  run_policy_runner_help umi-convert
  require_file calibration/umi_retarget.example.yaml
  run_yaml_parse_checks_if_available calibration/umi_retarget.example.yaml
  run_required_policy_runner_tests 'test_umi_*.py'
  grep_existing "robotics_lab\\.umi_retarget\\.v1|retarget_status|configured_estimate|measured" \
    calibration policy_runner docs README.md
}

run_gene_umi_flow_training_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q policy_runner/policy_runner scripts
  run_policy_runner_help flow-train
  run_policy_runner_help ml-preflight
  run_required_policy_runner_tests 'test_flow_matching.py'
  run_required_policy_runner_tests 'test_flow_training*.py'
  PYTHONPATH=policy_runner python3 -m policy_runner ml-preflight --vision-backbone tiny_cnn
  grep_existing "tiny_cnn|dataset-manifest|write-eval-report|flow_eval_summary" \
    policy_runner/policy_runner policy_runner/tests docs README.md
}

run_gene_umi_policy_rollout_modes_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q policy_runner/policy_runner scripts
  run_policy_runner_help flow-infer
  run_required_policy_runner_tests 'test_flow_inference*.py'
  run_required_policy_runner_tests 'test_policy_rollout_modes*.py'
  grep_existing "rollout-mode|controller_simulation|real_supervised|rollout_summary" \
    policy_runner/policy_runner policy_runner/tests docs README.md
}

run_gene_umi_dual_arm_gripper_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q policy_runner/policy_runner scripts
  run_policy_runner_help flow-infer
  run_required_policy_runner_tests 'test_gripper*.py'
  run_required_policy_runner_tests 'test_dual_arm_policy*.py'
  grep_existing "RB_ALLOW_REAL_GRIPPER|allow_real_gripper_motion|collision_model_status|selected[-_]arm" \
    policy_runner/policy_runner policy_runner/tests docs README.md
}

run_gene_umi_docs_ci_artifact_manifest_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q policy_runner/policy_runner scripts
  run_policy_runner_tests
  run_required_script_tests 'test_*.py'
  run_required_python_help scripts/collect_gene_umi_artifact_manifest.py
  require_file docs/runbooks/gene_umi_policy_transition.md
  grep_existing "GENE 26.5|ACKON500|hdf5-audit|flow-infer|real[-_]supervised|[Aa]rtifact manifest|artifact_manifest" \
    docs/runbooks/gene_umi_policy_transition.md README.md REVIEW.md policy_runner/README.md docs/runbooks/policy_data_collection.md
  run_required_make_dry_run policy-hdf5-audit-smoke
  run_required_make_dry_run policy-flow-smoke
  run_required_make_dry_run pgmode-transition-dry-run
}

run_required_policy_runner_tests_any() {
  local description="$1"
  shift

  local ran_any=0
  local pattern
  for pattern in "$@"; do
    if find policy_runner/tests -maxdepth 1 -name "${pattern}" -print -quit | grep -q .; then
      PYTHONPATH=policy_runner python3 -m unittest discover -s policy_runner/tests -p "${pattern}"
      ran_any=1
    fi
  done

  if [[ "${ran_any}" != "1" ]]; then
    echo "ERROR: required policy_runner tests not present for ${description}: $*" >&2
    return 1
  fi
}

require_active_gitignore_entry() {
  local description="$1"
  local pattern="$2"
  if ! grep -Eq "^[[:space:]]*${pattern}" .gitignore; then
    echo "ERROR: .gitignore missing active coverage for ${description}" >&2
    return 1
  fi
}

fail_if_tracked_matches() {
  local description="$1"
  local pattern="$2"
  local matches
  matches="$(git ls-files | grep -E "${pattern}" || true)"
  if [[ -n "${matches}" ]]; then
    echo "ERROR: tracked files include forbidden ${description}:" >&2
    printf '%s\n' "${matches}" >&2
    return 1
  fi
}

fail_if_tracked_real_config_without_example_suffix() {
  local matches
  matches="$(git ls-files 'rb_servo_server/config/dual_real*.yaml' | grep -Ev '\.example\.yaml$' || true)"
  if [[ -n "${matches}" ]]; then
    echo "ERROR: tracked real robot configs must be .example.yaml templates:" >&2
    printf '%s\n' "${matches}" >&2
    return 1
  fi
}

check_spacemouse_pgmode_config_safety() {
  local policy_config="policy_runner/config/rbpodo_pgmode_spacemouse_500hz_ack.yaml"
  local server_template="rb_servo_server/config/dual_real_rbpodo_pgmode_spacemouse_500hz_ack.example.yaml"

  require_file "${policy_config}"
  require_file "${server_template}"

  grep_absent "allow_real_motion:[[:space:]]*true" "${policy_config}"
  grep_existing "allow_rbpodo_controller_simulation_cartesian:[[:space:]]*true|controller_simulation_cartesian" \
    "${policy_config}" policy_runner/policy_runner policy_runner/tests policy_runner/README.md
  grep_existing "allow_in_controller_simulation:[[:space:]]*true|controller_simulation_cartesian" \
    "${server_template}" docs/runbooks/rbpodo_pgmode_spacemouse.md docs/servo_backend_contract.md docs/architecture.md
  grep_existing "allow_in_real:[[:space:]]*false" "${server_template}" docs/runbooks/rbpodo_pgmode_spacemouse.md
  grep_existing "physical_motion_expected:[[:space:]]*false|physical_motion_expected=false" \
    "${policy_config}" "${server_template}" docs/runbooks/rbpodo_pgmode_spacemouse.md policy_runner/README.md
  grep_existing "RB_ALLOW_REAL_MOTION|RB_ALLOW_REAL_ROBOT|RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION" \
    README.md docs/architecture.md docs/servo_backend_contract.md tools/rbpodo_pgmode_spacemouse.sh
  grep_existing "RB_ALLOW_REAL_CARTESIAN.*must not|Do not set.*RB_ALLOW_REAL_CARTESIAN|do not set.*RB_ALLOW_REAL_CARTESIAN" \
    tools/rbpodo_pgmode_spacemouse.sh docs/runbooks/rbpodo_pgmode_spacemouse.md policy_runner/README.md README.md
}

run_pgmode_spacemouse_python_checks() {
  run_shell_syntax_checks
  run_python_compile_checks
  run_policy_runner_tests
}

run_spacemouse_fix_controller_sim_safety_gate() {
  run_pgmode_spacemouse_python_checks
  check_spacemouse_pgmode_config_safety
}

run_pgmode_spacemouse_end_to_end_dryrun_gate() {
  run_shell_syntax_checks
  run_python_compile_checks
  run_policy_runner_tests
  run_gui_tests
  run_servo_gate_or_skip_missing_deps
  bash -n tools/rbpodo_pgmode_spacemouse.sh
  grep_existing "dual_spacemouse_cartesian" \
    policy_runner/policy_runner policy_runner/tests policy_runner/config policy_runner/README.md
  grep_existing "deadman|deadman_button" \
    policy_runner/policy_runner/action_sources policy_runner/tests policy_runner/config policy_runner/README.md
  grep_existing "release[-_ ]?zero|zero.*release|deadman.*released|released.*no motion|emits no motion" \
    policy_runner/policy_runner/action_sources policy_runner/tests policy_runner/README.md docs/runbooks/policy_data_collection.md
  grep_existing "sample_hold_timeout_sec|sample[-_ ]?hold|state_stream_stale|stale.*timeout|timeout.*stale" \
    policy_runner/policy_runner policy_runner/tests policy_runner/config
  grep_existing "TcpTwistLocal|tcp_twist_local" \
    policy_runner/policy_runner policy_runner/tests policy_runner/README.md
  grep_existing "controller_simulation_cartesian_enabled|cartesian_gate|controller_simulation.*readback|lease_readback|50376" \
    policy_runner/policy_runner policy_runner/tests policy_runner/config docs/runbooks/rbpodo_pgmode_spacemouse.md
  grep_existing "device:[[:space:]]*null|path:[[:space:]]*null|Fake.*SpaceMouse|fake.*spacemouse|FakeSpaceMouse" \
    policy_runner/config policy_runner/tests
}

run_spacemouse_flow_infer_rollout_modes_gate() {
  run_gene_umi_policy_rollout_modes_gate
  run_policy_runner_help flow-infer
  run_required_policy_runner_tests 'test_flow_inference*.py'
  run_required_policy_runner_tests 'test_policy_rollout_modes*.py'
  grep_existing "rollout-mode|controller_simulation|rollout_summary" \
    policy_runner/policy_runner policy_runner/tests docs README.md
  grep_existing "real_readonly|real_supervised" \
    policy_runner/policy_runner policy_runner/tests docs README.md
}

run_flow_policy_tcp_twistlocal_controller_sim_gate() {
  run_shell_syntax_checks
  run_python_compile_checks
  run_policy_runner_tests
  run_policy_runner_help flow-infer
  run_required_policy_runner_tests_any \
    "flow policy TcpTwistLocal controller-simulation rollout" \
    'test_flow_policy_tcp_twistlocal*.py' \
    'test_flow_policy_tcp_twist_local*.py' \
    'test_*flow*twistlocal*.py' \
    'test_*flow*tcp_twistlocal*.py'
  grep_existing "TcpTwistLocal|tcp_twist_local" \
    policy_runner/policy_runner policy_runner/tests docs/runbooks/policy_data_collection.md
  grep_existing "bounded.*TcpTwistLocal|TcpTwistLocal.*bounded|clamp|clip|max_linear_velocity_m_s|max_angular_velocity_rad_s" \
    policy_runner/policy_runner policy_runner/tests docs README.md
  grep_existing "policy_dt_sec|policy[-_ ]dt|command_rate_hz|dt_sec" \
    policy_runner/policy_runner policy_runner/tests policy_runner/config docs README.md
  grep_existing "controller_simulation|allow_rbpodo_controller_simulation_cartesian|controller_simulation_cartesian" \
    policy_runner/policy_runner policy_runner/tests policy_runner/config docs README.md
  grep_existing "real_readonly|real_supervised|physical real|physical_real|allow_real_motion:[[:space:]]*false" \
    policy_runner/policy_runner policy_runner/tests policy_runner/config docs README.md
}

run_viser_pgmode_operator_view_gate() {
  run_shell_syntax_checks
  run_python_compile_checks
  run_gui_tests
  grep_existing "50366" \
    rb_gui rb_gui/tests docs/runbooks/rbpodo_pgmode_spacemouse.md tools/rbpodo_pgmode_spacemouse.sh \
    rb_servo_server/config/dual_real_rbpodo_pgmode_spacemouse_500hz_ack.example.yaml
  grep_existing "50376" \
    rb_gui rb_gui/tests docs/runbooks/rbpodo_pgmode_spacemouse.md tools/rbpodo_pgmode_spacemouse.sh \
    rb_servo_server/config/dual_real_rbpodo_pgmode_spacemouse_500hz_ack.example.yaml policy_runner/config
  grep_existing "physical_motion_expected" rb_gui rb_gui/tests docs/runbooks/rbpodo_pgmode_spacemouse.md
  grep_existing "tcp_ref_stand|selected_tcp_source|selected_source|selected TCP|selected tcp" \
    rb_gui rb_gui/tests docs/runbooks/rbpodo_pgmode_spacemouse.md
  grep_existing "lease|source_id|command_source|policy_runner safety readback" \
    rb_gui rb_gui/tests docs/runbooks/rbpodo_pgmode_spacemouse.md tools/rbpodo_pgmode_spacemouse.sh
  grep_existing "controller_simulation_mode|controller simulation|controller_simulation" \
    rb_gui rb_gui/tests docs/runbooks/rbpodo_pgmode_spacemouse.md
  echo "codex_gate: not launching live viewer; 05_viser_pgmode_operator_view is documentation/test evidence only"
}

run_spacemouse_gripper_dual_arm_policy_gate() {
  run_gene_umi_dual_arm_gripper_gate
  grep_existing "RB_ALLOW_REAL_GRIPPER|allow_real_gripper_motion" \
    policy_runner/policy_runner policy_runner/tests docs README.md
  grep_existing "selected[-_]arm|collision_model_status" \
    policy_runner/policy_runner policy_runner/tests docs README.md
  grep_existing "physical.*gripper.*block|gripper.*physical.*block|allow_real_gripper_motion:[[:space:]]*false|blocked by default" \
    policy_runner/policy_runner policy_runner/tests docs README.md
}

run_ml_preflight_gate_stability_gate() {
  run_gene_umi_flow_training_gate
  run_policy_runner_help ml-preflight
  run_policy_runner_help flow-train
  PYTHONPATH=policy_runner python3 -m policy_runner ml-preflight --vision-backbone tiny_cnn
  grep_existing "tiny_cnn" policy_runner/policy_runner policy_runner/tests docs README.md
  grep_existing "seed|deterministic" \
    policy_runner/policy_runner policy_runner/tests docs README.md
  grep_existing "no network|network downloads|no internet|offline|download" \
    policy_runner/policy_runner policy_runner/tests docs README.md
}

run_umi_hdf5_manifest_robustness_gate() {
  run_gene_umi_hdf5_audit_gate
  run_gene_umi_bimanual_collection_gate
  run_policy_runner_help hdf5-audit
  run_policy_runner_help umi-import
  run_policy_runner_help umi-convert
  run_required_policy_runner_tests 'test_hdf5_audit*.py'
  run_required_policy_runner_tests 'test_flow_dataset.py'
  run_required_policy_runner_tests 'test_umi_*.py'
  grep_existing "dataset_manifest|DatasetManifest" policy_runner/policy_runner policy_runner/tests docs README.md
  grep_existing "schema|schema_version|version" policy_runner/policy_runner policy_runner/tests docs README.md
  if [[ -f episode_002.hdf5 ]]; then
    echo "codex_gate: episode_002.hdf5 is present; hdf5 audit smoke remains optional and local"
  else
    echo "codex_gate: episode_002.hdf5 absent as expected; gate does not require committed dataset files"
  fi
}

run_artifact_manifest_docs_makefile_gate() {
  run_gene_umi_docs_ci_artifact_manifest_gate
}

run_source_hygiene_local_configs_gate() {
  run_shell_syntax_checks
  run_python_compile_checks
  require_active_gitignore_entry "HDF5 datasets (*.hdf5)" '\*\.hdf5([[:space:]]|$)'
  require_active_gitignore_entry "HDF5 datasets (*.h5)" '\*\.h5([[:space:]]|$)'
  require_active_gitignore_entry "artifact directories" '(\*\*/)?artifacts/'
  require_active_gitignore_entry "rb_servo_server local YAML configs" 'rb_servo_server/config/local/\*\.ya?ml'
  require_active_gitignore_entry "runtime output directories" '(\*\*/)?(logs|episodes|checkpoints)/'
  fail_if_tracked_matches "large local dataset files" '\.(hdf5|h5)$'
  fail_if_tracked_matches "Python cache files" '(^|/)__pycache__(/|$)|\.pyc$'
  fail_if_tracked_matches "Codex run artifacts" '^artifacts/codex_runs/'
  fail_if_tracked_matches "local rb_servo_server YAML configs" '^rb_servo_server/config/local/.+\.(yaml|yml)$'
  fail_if_tracked_real_config_without_example_suffix
  grep_existing "\\.example\\.yaml|site-specific|placeholder|config/local|Copy to config/local" \
    README.md docs rb_servo_server/config/dual_real.example.yaml rb_servo_server/config/dual_real_rbpodo_pgmode_spacemouse_500hz_ack.example.yaml
}

run_optional_rbpodo_p1_circle_ablation() {
  if [[ "${CODEX_RUN_RBPODO_CIRCLE_ABLATION:-0}" != "1" ]]; then
    echo "codex_gate: skipping rbpodo P1 controller-simulation ablation; set CODEX_RUN_RBPODO_CIRCLE_ABLATION=1 with explicit CODEX_RBPODO_CIRCLE_ABLATION_ARGS to enable"
    return 0
  fi
  if [[ -z "${CODEX_RBPODO_CIRCLE_ABLATION_ARGS:-}" ]]; then
    echo "ERROR: CODEX_RUN_RBPODO_CIRCLE_ABLATION=1 requires CODEX_RBPODO_CIRCLE_ABLATION_ARGS with explicit matrix/script arguments and safety preflight flags" >&2
    return 1
  fi
  for required_arg in \
    "--matrix" \
    "--artifact-root" \
    "--i-understand-this-connects-to-real-controller" \
    "--i-confirm-controller-is-in-pgmode-simulation"
  do
    if [[ "${CODEX_RBPODO_CIRCLE_ABLATION_ARGS}" != *"${required_arg}"* ]]; then
      echo "ERROR: CODEX_RBPODO_CIRCLE_ABLATION_ARGS must include ${required_arg}" >&2
      return 1
    fi
  done
  if [[ "${CODEX_RBPODO_CIRCLE_ABLATION_ARGS}" != *"--verify-pgmode-simulation"* && "${CODEX_RBPODO_CIRCLE_ABLATION_ARGS}" != *"--set-pgmode-simulation"* && "${CODEX_RBPODO_CIRCLE_ABLATION_ARGS}" != *"--pgmode-summary-json"* ]]; then
    echo "ERROR: CODEX_RBPODO_CIRCLE_ABLATION_ARGS must include pgmode simulation evidence" >&2
    return 1
  fi
  if [[ "${RB_ALLOW_REAL_CARTESIAN:-0}" == "1" ]]; then
    echo "ERROR: RB_ALLOW_REAL_CARTESIAN must not be set for rbpodo P1 controller-simulation ablation" >&2
    return 1
  fi
  # shellcheck disable=SC2086
  python3 scripts/run_rbpodo_circle_ablation.py ${CODEX_RBPODO_CIRCLE_ABLATION_ARGS}
}

run_rbpodo_p1_common_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q scripts
  python3 scripts/rbpodo_circle_tracking_benchmark.py --help >/dev/null
  python3 scripts/run_rbpodo_circle_ablation.py --help >/dev/null
  python3 scripts/generate_circle_benchmark_report.py --help >/dev/null
  run_yaml_parse_checks_if_available configs/rbpodo_circle_ablation/*.yaml
  run_rbpodo_circle_matrix_schema_checks
}

run_p1_circle_factor_matrix_gate() {
  run_rbpodo_p1_common_gate
  PYTHONPATH=scripts python3 -m unittest discover scripts -p 'test_rbpodo_circle_ablation.py'
  PYTHONPATH=scripts python3 -m unittest discover scripts -p 'test_circle_benchmark_report.py'
  run_optional_rbpodo_p1_circle_ablation
}

run_p1_orientation_diag_gate() {
  run_rbpodo_p1_common_gate
  run_optional_python_help scripts/error_decomposition.py
  PYTHONPATH=scripts python3 -m unittest discover scripts -p 'test_circle_error_decomposition.py'
  PYTHONPATH=scripts python3 -m unittest discover scripts -p 'test_circle_benchmark_report.py'
  grep_existing "orientation|orientation_limited|orientation_position_equiv" scripts docs REVIEW.md
  echo "codex_gate: skipping rbpodo orientation diagnostic controller run by default"
}

run_p1_deadtime_phase_advance_gate() {
  run_rbpodo_p1_common_gate
  python3 scripts/timestamp_alignment_audit.py --help >/dev/null
  PYTHONPATH=scripts python3 -m unittest discover scripts -p 'test_timestamp_alignment_audit.py'
  PYTHONPATH=scripts python3 -m unittest discover scripts -p 'test_circle_error_decomposition.py'
  grep_existing "phase_lag|estimated_latency|timestamp_alignment|ack_spike" scripts docs REVIEW.md
  echo "codex_gate: skipping rbpodo deadtime/phase controller run by default"
}

run_p1_servo_param_sweep_gate() {
  run_rbpodo_p1_common_gate
  python3 scripts/rainbow_rate_probe.py --help >/dev/null
  PYTHONPATH=scripts python3 -m unittest discover scripts -p 'test_rainbow_rate_probe.py'
  grep_existing "servo_t1_sec|servo_t2_sec|servo_gain|servo_alpha|speed_bar" \
    scripts docs REVIEW.md rb_servo_server/config
  run_optional_rbpodo_p1_circle_ablation
}

run_p1_server_side_circle_track_skeleton_gate() {
  run_rbpodo_p1_common_gate
  grep_existing "server_circle" scripts/rbpodo_circle_tracking_benchmark.py scripts/test_rbpodo_circle_tracking_benchmark.py
  PYTHONPATH=scripts python3 -m unittest discover scripts -p 'test_rbpodo_circle_tracking_benchmark.py'
  echo "codex_gate: skipping server-side circle tracking runtime by default"
}

run_yaml_parse_checks_if_available() {
  local paths=()
  local path
  for path in "$@"; do
    if [[ -e "${path}" ]]; then
      paths+=("${path}")
    fi
  done

  if [[ "${#paths[@]}" -eq 0 ]]; then
    echo "codex_gate: skipping YAML parse checks; no matching YAML files found"
    return 0
  fi

  python3 - "${paths[@]}" <<'PY'
import sys
from pathlib import Path

try:
    import yaml
except ModuleNotFoundError:
    print("codex_gate: skipping YAML parse checks; PyYAML is unavailable")
    raise SystemExit(0)

failed = False
for raw_path in sys.argv[1:]:
    path = Path(raw_path)
    try:
        with path.open("r", encoding="utf-8") as handle:
            yaml.safe_load(handle)
    except Exception as exc:
        print(f"ERROR: failed to parse YAML {path}: {exc}", file=sys.stderr)
        failed = True

raise SystemExit(1 if failed else 0)
PY
}

run_rbpodo_controller_sim_cartesian_gate() {
  run_shell_syntax_checks
  run_servo_gate_or_skip_missing_deps
  run_python_surface_tests
  echo "codex_gate: skipping rbpodo controller-simulation Cartesian controller run by default"
}

run_rbpodo_circle_config_fix_gate() {
  run_shell_syntax_checks
  grep_existing "dual_real_rbpodo_circle_15cm16s|dual_real_rbpodo_circle_15cm4s" \
    README.md REVIEW.md docs rb_servo_server/docs rb_servo_server/config
  grep_existing "backend_type:[[:space:]]*rbpodo" \
    rb_servo_server/config/dual_real_rbpodo_circle_15cm16s.example.yaml \
    rb_servo_server/config/dual_real_rbpodo_circle_15cm4s.example.yaml
  grep_existing "operation_mode:[[:space:]]*simulation" \
    rb_servo_server/config/dual_real_rbpodo_circle_15cm16s.example.yaml \
    rb_servo_server/config/dual_real_rbpodo_circle_15cm4s.example.yaml
  grep_existing "controller pgmode simulation only|physical robot must not move" \
    rb_servo_server/config/dual_real_rbpodo_circle_15cm16s.example.yaml \
    rb_servo_server/config/dual_real_rbpodo_circle_15cm4s.example.yaml \
    docs/runbooks/rbpodo_controller_sim_circle.md
  grep_existing "cartesian_control:" \
    rb_servo_server/config/dual_real_rbpodo_circle_15cm16s.example.yaml \
    rb_servo_server/config/dual_real_rbpodo_circle_15cm4s.example.yaml
  grep_existing "allow_in_real:[[:space:]]*false" \
    rb_servo_server/config/dual_real_rbpodo_circle_15cm16s.example.yaml \
    rb_servo_server/config/dual_real_rbpodo_circle_15cm4s.example.yaml
  run_yaml_parse_checks_if_available \
    rb_servo_server/config/dual_real_rbpodo_circle_15cm16s.example.yaml \
    rb_servo_server/config/dual_real_rbpodo_circle_15cm4s.example.yaml \
    rb_servo_server/config/local/dual_real_rbpodo_circle_15cm16s.yaml \
    rb_servo_server/config/local/dual_real_rbpodo_circle_15cm4s.yaml
  run_python_surface_tests
  run_servo_gate_or_skip_missing_deps
}

run_rbpodo_circle_bench_fix_gate() {
  python3 -m compileall -q scripts
  python3 scripts/rbpodo_circle_tracking_benchmark.py --help >/dev/null
  PYTHONPATH=scripts python3 -m unittest discover scripts -p 'test_*.py'
  if [[ "${CODEX_RUN_RBPODO_CIRCLE:-0}" == "1" ]]; then
    if [[ ! -f scripts/rbpodo_circle_tracking_benchmark.py ]]; then
      echo "ERROR: CODEX_RUN_RBPODO_CIRCLE=1 but scripts/rbpodo_circle_tracking_benchmark.py is missing" >&2
      return 1
    fi
    if [[ -z "${CODEX_RBPODO_CIRCLE_ARGS:-}" ]]; then
      echo "ERROR: CODEX_RUN_RBPODO_CIRCLE=1 requires CODEX_RBPODO_CIRCLE_ARGS with explicit controller-simulation script arguments and safety preflight flags" >&2
      return 1
    fi
    # shellcheck disable=SC2086
    python3 scripts/rbpodo_circle_tracking_benchmark.py ${CODEX_RBPODO_CIRCLE_ARGS}
  else
    echo "codex_gate: skipping rbpodo controller-simulation circle benchmark; set CODEX_RUN_RBPODO_CIRCLE=1 with explicit args to enable"
  fi
}

run_rbpodo_circle_doc_fix_gate() {
  for token in \
    "allow_in_controller_simulation" \
    "RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN" \
    "pgmode simulation" \
    "physical_motion_expected=false" \
    "tcp_ref_stand" \
    "cartesian_control_unavailable"
  do
    grep_existing "${token}" README.md REVIEW.md docs rb_servo_server/docs
  done
}

run_state_fanout_gate() {
  run_servo_gate_or_skip_missing_deps
  run_yaml_parse_checks_if_available rb_servo_server/config/*.yaml rb_servo_server/config/local/*.yaml configs/**/*.yaml
  run_python_surface_tests
}

run_gui_tcp_ref_actual_gate() {
  run_gui_tests
  python3 -m compileall -q rb_gui/rb_servo_gui
}

run_bench_overlay_udp_gate() {
  python3 -m compileall -q scripts
  python3 scripts/rbpodo_circle_tracking_benchmark.py --help >/dev/null
  PYTHONPATH=scripts python3 -m unittest discover scripts -p 'test_*.py'
  echo "codex_gate: skipping rbpodo controller-simulation benchmark; BENCH-OVERLAY-UDP-01 is validation-only by default"
}

run_gui_circle_overlay_gate() {
  run_gui_tests
  python3 -m compileall -q rb_gui/rb_servo_gui
  PYTHONPATH=scripts python3 -m unittest discover scripts -p 'test_*.py'
  echo "codex_gate: skipping controller and benchmark runs; GUI-CIRCLE-OVERLAY-01 is visualization-only by default"
}

run_rbpodo_circle_live_runbook_gate() {
  for token in \
    "state_pub_endpoints" \
    "overlay-pub-endpoint" \
    "tcp_ref_stand" \
    "pgmode simulation" \
    "rb_gui"
  do
    grep_existing "${token}" README.md REVIEW.md docs rb_servo_server/docs
  done
}

run_policy_dataset_schema_gate() {
  run_policy_runner_tests
  grep_existing "robotics_lab\\.policy_runner\\.episode\\.v1|robotics_lab\\.episode\\.v1" \
    policy_runner README.md docs
  grep_existing "dataset|episode" \
    policy_runner/README.md policy_runner/policy_runner policy_runner/tests README.md docs
  grep_existing "observations|action" \
    policy_runner/README.md policy_runner/policy_runner/recording.py policy_runner/tests
  echo "codex_gate: skipping real policy/data collection run by default"
}

run_backend_compare_python_tests() {
  run_python_surface_tests
  run_optional_rbscript_helper_tests
}

run_backend_compare_doc_config_checks() {
  grep_existing "rbpodo" README.md REVIEW.md docs rb_servo_server/docs rb_servo_server/config
  grep_existing "rbscript_tcp" README.md REVIEW.md docs rb_servo_server/docs rb_servo_server/config
  grep_existing "RB_ALLOW_REAL_ROBOT" README.md REVIEW.md docs rb_servo_server/docs rb_servo_server/config
  grep_existing "RB_ALLOW_RBSCRIPT_TCP" README.md REVIEW.md docs rb_servo_server/docs rb_servo_server/config
  grep_existing "RB_ALLOW_RBSCRIPT_TCP_MOTION" README.md REVIEW.md docs rb_servo_server/docs rb_servo_server/config
  grep_existing "send_servo_commands:[[:space:]]*false" rb_servo_server/config docs/runbooks
  grep_existing "read-only|no-motion" README.md REVIEW.md docs rb_servo_server/docs rb_servo_server/config
}

run_backend_compare_config_gate() {
  run_shell_syntax_checks
  run_backend_compare_doc_config_checks
  run_backend_compare_python_tests
}

run_backend_compare_probe_if_requested() {
  if [[ "${CODEX_RUN_BACKEND_COMPARE:-0}" == "1" ]]; then
    if [[ ! -f scripts/rb_backend_ablation.py ]]; then
      echo "ERROR: CODEX_RUN_BACKEND_COMPARE=1 but scripts/rb_backend_ablation.py is missing" >&2
      return 1
    fi
    local args="${CODEX_BACKEND_COMPARE_ABLATION_ARGS:-${CODEX_BACKEND_COMPARE_ARGS:-}}"
    if [[ -z "${args}" ]]; then
      echo "ERROR: CODEX_RUN_BACKEND_COMPARE=1 requires CODEX_BACKEND_COMPARE_ABLATION_ARGS or CODEX_BACKEND_COMPARE_ARGS with explicit tool arguments and safety preflight flags" >&2
      return 1
    fi
    # shellcheck disable=SC2086
    python3 scripts/rb_backend_ablation.py ${args}
  else
    echo "codex_gate: skipping full backend comparison probe; set CODEX_RUN_BACKEND_COMPARE=1 with explicit args to enable"
  fi
}

run_rbscript_persistent_probe_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q scripts
  python3 scripts/rb_backend_ablation.py --help >/dev/null
  python3 scripts/rainbow_rate_probe.py --help >/dev/null
  python3 scripts/compare_backend_ablation.py --help >/dev/null
  run_backend_compare_python_tests
  run_backend_compare_probe_if_requested
}

run_rbscript_servo_noop_gate() {
  run_shell_syntax_checks
  run_servo_gate_or_skip_missing_deps
  run_optional_python_help scripts/rbscript_servo_acceptance.py
  if [[ -f scripts/rbscript_servo_acceptance.py ]]; then
    python3 scripts/rbscript_servo_acceptance.py --self-test
  fi
  run_backend_compare_python_tests
  if [[ "${CODEX_RUN_RBSCRIPT_SERVO_ACCEPTANCE:-0}" == "1" ]]; then
    if [[ ! -f scripts/rbscript_servo_acceptance.py ]]; then
      echo "ERROR: CODEX_RUN_RBSCRIPT_SERVO_ACCEPTANCE=1 but scripts/rbscript_servo_acceptance.py is missing" >&2
      return 1
    fi
    if [[ -z "${CODEX_RBSCRIPT_SERVO_ACCEPTANCE_ARGS:-}" ]]; then
      echo "ERROR: CODEX_RUN_RBSCRIPT_SERVO_ACCEPTANCE=1 requires CODEX_RBSCRIPT_SERVO_ACCEPTANCE_ARGS with explicit script arguments and safety preflight flags" >&2
      return 1
    fi
    # shellcheck disable=SC2086
    python3 scripts/rbscript_servo_acceptance.py ${CODEX_RBSCRIPT_SERVO_ACCEPTANCE_ARGS}
  else
    echo "codex_gate: skipping rbscript ServoJ no-op acceptance; set CODEX_RUN_RBSCRIPT_SERVO_ACCEPTANCE=1 with explicit args to enable"
  fi
}

run_rbscript_data_port_gate() {
  run_shell_syntax_checks
  run_servo_gate_or_skip_missing_deps
  run_optional_rbscript_helper_tests
  grep_existing "data[[:space:]_-]*port[^0-9]*5001|reqdata|rbscript_tcp_state_v1" \
    docs/runbooks/rbscript_tcp_ablation.md docs/servo_backend_contract.md scripts/rb_backend_ablation.py
  echo "codex_gate: skipping real rbscript data-port probe by default"
}

run_backend_compare_matrix_gate() {
  python3 -m compileall -q scripts
  run_optional_python_help scripts/run_backend_comparison_matrix.py
  if [[ "${CODEX_RUN_BACKEND_COMPARE:-0}" == "1" ]]; then
    if [[ ! -f scripts/run_backend_comparison_matrix.py ]]; then
      echo "ERROR: CODEX_RUN_BACKEND_COMPARE=1 but scripts/run_backend_comparison_matrix.py is missing" >&2
      return 1
    fi
    if [[ -z "${CODEX_BACKEND_COMPARE_MATRIX_ARGS:-}" ]]; then
      echo "ERROR: CODEX_RUN_BACKEND_COMPARE=1 requires CODEX_BACKEND_COMPARE_MATRIX_ARGS with explicit script arguments and safety preflight flags" >&2
      return 1
    fi
    # shellcheck disable=SC2086
    python3 scripts/run_backend_comparison_matrix.py ${CODEX_BACKEND_COMPARE_MATRIX_ARGS}
  else
    echo "codex_gate: skipping full backend comparison matrix; set CODEX_RUN_BACKEND_COMPARE=1 with explicit matrix args to enable"
  fi
}

run_backend_compare_report_gate() {
  python3 -m compileall -q scripts
  python3 scripts/compare_backend_ablation.py --help >/dev/null
  run_optional_python_help scripts/report_backend_comparison.py
  run_optional_python_help scripts/backend_comparison_report.py
  for token in \
    "rbpodo" \
    "rbscript_tcp" \
    "apples-to-apples" \
    "read_state" \
    "servo_j_noop" \
    "ACK-on" \
    "ACK-off"
  do
    grep_existing "${token}" README.md REVIEW.md docs rb_servo_server/docs
  done
}

run_doc_hygiene_gate() {
  run_shell_syntax_checks
  if [[ -e README_DOCS_UPDATE.md ]]; then
    echo "ERROR: README_DOCS_UPDATE.md must not exist; fold doc-update notes into source-of-truth docs" >&2
    return 1
  fi
  if [[ -e rb_servo_server/docker-compose.yml ]]; then
    echo "ERROR: rb_servo_server/docker-compose.yml must not exist; use repository-root docker-compose.yml via make sim-up" >&2
    return 1
  fi

  grep_existing "source-of-truth current review" docs/current_review.md
  grep_existing "REVIEW\.md" docs/current_review.md
  local current_review_lines
  current_review_lines="$(wc -l < docs/current_review.md)"
  if [[ "${current_review_lines}" -gt 12 ]]; then
    echo "ERROR: docs/current_review.md must be a short redirect to REVIEW.md, not a mirrored review" >&2
    return 1
  fi

  local forbidden_config
  for forbidden_config in \
    rb_servo_server/config/dual_rbsim.yaml \
    rb_servo_server/config/dual_rb_simulator.yaml \
    rb_servo_server/config/dual_rb_simulator_compose.yaml \
    rb_simulator/config/dual_rb3_730e.yaml
  do
    if [[ -e "${forbidden_config}" ]]; then
      echo "ERROR: deprecated config remains in active config directory: ${forbidden_config}" >&2
      return 1
    fi
  done

  local archived_config
  for archived_config in \
    docs/archive/configs/rb_servo_server_dual_rbsim.legacy.yaml \
    docs/archive/configs/rb_servo_server_dual_rb_simulator.legacy.yaml \
    docs/archive/configs/rb_servo_server_dual_rb_simulator_compose.legacy.yaml \
    docs/archive/configs/rb_simulator_dual_rb3_730e.legacy.yaml
  do
    if [[ ! -f "${archived_config}" ]]; then
      echo "ERROR: missing archived deprecated config: ${archived_config}" >&2
      return 1
    fi
    grep_existing "Deprecated|deprecated|HISTORICAL|historical|archive|archived" "${archived_config}"
  done

  grep_existing "historical reference only|not runnable source-of-truth" docs/archive/configs/README.md README.md README.en.md
  grep_absent '(^|[^[:alnum:]_.-])dual_real\.yaml([^[:alnum:]_.-]|$)' README.md docs AGENTS.md rb_servo_server/docs
}

run_gui_split_gate() {
  run_gui_tests
  python3 -m compileall -q rb_gui/rb_servo_gui
}

check_real_config_safety_docs() {
  grep_existing "RB_ALLOW_REAL_ROBOT" README.md docs AGENTS.md rb_servo_server/docs rb_servo_server/config/dual_real.example.yaml
  grep_existing "RB_ALLOW_REAL_MOTION" README.md docs AGENTS.md rb_servo_server/docs rb_servo_server/config/dual_real.example.yaml
  grep_existing "RB_ALLOW_REAL_CARTESIAN" README.md docs AGENTS.md rb_servo_server/docs rb_servo_server/config/dual_real.example.yaml
  grep_existing "dual_real.example.yaml" README.md docs rb_servo_server/docs rb_servo_server/config/dual_real.example.yaml
  grep_existing "dual_real_readonly.yaml" README.md docs rb_servo_server/docs rb_servo_server/config/dual_real.example.yaml
  grep_existing "dual_real_motion.yaml" README.md docs rb_servo_server/docs rb_servo_server/config/dual_real.example.yaml

  if [[ -f rb_servo_server/config/dual_real.yaml ]]; then
    echo "ERROR: rb_servo_server/config/dual_real.yaml must not exist; use dual_real.example.yaml and config/local/*.yaml" >&2
    return 1
  fi
}

check_backend_contract_docs() {
  grep_existing "BackendResult|SendServoJResult" README.md docs AGENTS.md rb_servo_server/docs
  grep_existing "RobotFault" README.md docs rb_servo_server/docs
  grep_existing "TransportWriteFailed|TransportTimeout|TransportReadFailed" README.md docs rb_servo_server/docs
  grep_existing "SuppressedByPolicy" README.md docs rb_servo_server/docs
  grep_existing "FaultContext|fault_context" README.md docs rb_servo_server/docs
}

check_worker_docs() {
  grep_existing "ArmWorker" README.md docs rb_servo_server/docs
  grep_existing "latest_wins|latest-wins|worker_queue_policy" README.md docs rb_servo_server/docs
  grep_existing "worker_command_drops_total|command_drops_total" README.md docs rb_servo_server/docs
  grep_existing "io_model: worker|worker io_model" README.md docs rb_servo_server/docs rb_servo_server/config
  grep_existing "io_model: direct|direct io_model" README.md docs rb_servo_server/docs rb_servo_server/config
}

check_canonical_config_docs() {
  check_real_config_safety_docs
  grep_existing "Canonical Config Names" README.md docs rb_servo_server/docs || true
  grep_existing "dual_simulator.yaml" README.md docs rb_servo_server/docs rb_servo_server/config
  grep_existing "dual_simulator_compose.yaml" README.md docs rb_servo_server/docs rb_servo_server/config
  grep_existing "dual_simulator_worker.yaml" README.md docs rb_servo_server/docs rb_servo_server/config
  grep_existing "left_rb3_730e" README.md docs rb_simulator/README.md rb_simulator/docs rb_simulator/config
  grep_existing "right_rb3_730e" README.md docs rb_simulator/README.md rb_simulator/docs rb_simulator/config
  grep_absent "dual_real\.yaml" README.md docs AGENTS.md rb_servo_server/docs
}

check_command_source_docs() {
  grep_existing "command_source|Command Source Lease|source_id|lease_token|AcquireLease" \
    README.md docs rb_servo_server/docs rb_servo_server/docs/network_protocol.md policy_runner/README.md
  grep_existing 'enforce_lease: false|defaults to off|defaults to `false`|defaults to false' \
    README.md docs rb_servo_server/docs rb_servo_server/docs/network_protocol.md
}

check_tcp_pose_docs() {
  grep_existing "tcp_pose_simulator_acceptance.sh" README.md docs rb_servo_server/docs
  grep_existing "dual_simulator_tcp_acceptance.yaml" README.md docs rb_servo_server/docs rb_servo_server/config
  grep_existing "quaternion_xyzw|qx|qy|qz|qw" README.md docs rb_servo_server/docs rb_gui/README.md || true
  grep_existing "ik_duration_us|fk_duration_us|cartesian_solve" README.md docs rb_servo_server/docs || true
}

check_camera_acceptance_docs() {
  grep_existing "camera_stale_timeout_sec|requires_camera" README.md docs policy_runner/README.md
  grep_existing "camera acceptance|3-camera|30 FPS|D435f|D405" README.md docs camera_server/README.md camera_server/docs || true
}

check_dev_env_docs() {
  grep_existing "libyaml-cpp-dev" README.md docs scripts || true
  grep_existing "nlohmann-json3-dev" README.md docs scripts || true
  grep_existing "install_deps_ubuntu.sh" README.md docs scripts || true
}

run_p3f_gate() {
  run_shell_syntax_checks
  grep_existing "RB_ALLOW_REAL_CARTESIAN" docs/runbooks/tcp_pose_simulator_acceptance.md README.md rb_servo_server/docs
  run_servo_pinocchio_gate
  run_optional_tcp_pose_acceptance
}

run_mig12_gate() {
  run_shell_syntax_checks
  check_real_config_safety_docs
  check_backend_contract_docs
  check_worker_docs
  run_simulator_tests
  run_gui_tests
  run_policy_runner_tests
  run_servo_gate_or_skip_missing_deps
  run_servo_pinocchio_gate
}

run_mig13_gate() {
  run_shell_syntax_checks
  run_simulator_tests
  run_servo_gate_or_skip_missing_deps
}

run_mig20_gate() {
  run_shell_syntax_checks
  run_servo_gate_or_skip_missing_deps
  run_servo_pinocchio_gate
}

run_mig26_gate() {
  run_shell_syntax_checks
  check_canonical_config_docs
  check_backend_contract_docs
  check_worker_docs
  check_command_source_docs
  check_tcp_pose_docs
  check_camera_acceptance_docs
  check_dev_env_docs
  if [[ "${CODEX_STRICT_DEPS:-0}" == "1" ]]; then
    ./scripts/check_deps.sh --profile hardware-free
  else
    echo "codex_gate: CODEX_STRICT_DEPS is not 1; dependency preflight is advisory"
    ./scripts/check_deps.sh --profile hardware-free || true
  fi
  run_python_compile_checks
  run_simulator_tests
  run_gui_tests
  run_policy_runner_tests
  run_camera_gate_or_skip_missing_deps
  run_servo_gate_or_skip_missing_deps
  if [[ "${CODEX_RUN_FULL_SMOKE:-0}" == "1" ]]; then
    ./scripts/hardware_free_validation.sh
  else
    echo "codex_gate: skipping full hardware_free_validation.sh; set CODEX_RUN_FULL_SMOKE=1 to enable"
  fi
  run_servo_pinocchio_gate
  if [[ "${CODEX_RUN_TCP_ACCEPTANCE:-0}" == "1" ]]; then
    run_optional_tcp_pose_acceptance
  else
    echo "codex_gate: skipping TCP pose acceptance; set CODEX_RUN_TCP_ACCEPTANCE=1 to enable"
  fi
}

case "$TASK" in
  P0-A)
    grep_existing "RB_ALLOW_REAL_MOTION" README.md docs
    ;;
  P0-B)
    run_simulator_tests
    ;;
  P0-D|P1-A|P1-B|P1-F|P2-A|P2-C|P3-A|P3-B|P3-C)
    run_servo_gate_or_skip_missing_deps
    ;;
  P0-G)
    run_camera_gate_or_skip_missing_deps
    ;;
  P0-H|P2-D|P3-D)
    run_gui_tests
    ;;
  P1-C)
    run_simulator_tests
    run_servo_gate_or_skip_missing_deps
    if [[ "${CODEX_RUN_FULL_SMOKE:-0}" == "1" ]]; then
      ./scripts/hardware_free_validation.sh
    fi
    ;;
  P1-D|P1-E|P2-E|P3-E)
    run_policy_runner_tests
    ;;
  P2-B)
    grep_existing "geometry_valid_for_real_policy" calibration geometry docs README.md
    grep_existing "configured_estimate" calibration geometry docs README.md
    ;;
  P3-F)
    run_p3f_gate
    ;;

  MIG-00)
    run_shell_syntax_checks
    check_real_config_safety_docs
    ;;
  MIG-01|MIG-02|MIG-04|MIG-05|MIG-06|MIG-07|MIG-08|MIG-09|MIG-11)
    run_servo_gate_or_skip_missing_deps
    ;;
  MIG-03)
    run_simulator_tests
    run_servo_gate_or_skip_missing_deps
    ;;
  MIG-10)
    run_simulator_tests
    run_servo_gate_or_skip_missing_deps
    if [[ "${CODEX_RUN_FULL_SMOKE:-0}" == "1" ]]; then
      ./scripts/hardware_free_validation.sh
    fi
    ;;
  MIG-12)
    run_mig12_gate
    ;;
  MIG-13)
    run_mig13_gate
    ;;
  MIG-14|MIG-15|MIG-16|MIG-21|MIG-22|MIG-23|MIG-24)
    run_servo_gate_or_skip_missing_deps
    ;;
  MIG-17)
    run_policy_runner_tests
    ;;
  MIG-18)
    run_shell_syntax_checks
    check_canonical_config_docs
    run_simulator_tests
    run_gui_tests
    run_policy_runner_tests
    ;;
  MIG-19|MIG-25)
    run_shell_syntax_checks
    run_simulator_tests
    run_gui_tests
    run_policy_runner_tests
    ;;
  MIG-20)
    run_mig20_gate
    ;;
  MIG-26)
    run_mig26_gate
    ;;

  HARDEN-00)
    run_shell_syntax_checks
    ;;
  HARDEN-01)
    run_shell_syntax_checks
    check_dev_env_docs
    ;;
  HARDEN-02)
    run_shell_syntax_checks
    run_simulator_tests
    run_servo_gate_or_skip_missing_deps
    ;;
  HARDEN-03)
    run_shell_syntax_checks
    run_servo_gate_or_skip_missing_deps
    check_worker_docs
    ;;
  HARDEN-04)
    run_shell_syntax_checks
    run_servo_gate_or_skip_missing_deps
    grep_existing "disable_waiting_ack|rbpodo_stop_unverified|operator intervention|read-only" README.md docs rb_servo_server/docs rb_servo_server/config || true
    ;;
  HARDEN-05)
    run_policy_runner_tests
    run_python_compile_checks
    grep_existing "spacemouse_joint_velocity|tcp_delta|spacemouse_cartesian" policy_runner policy_runner/README.md
    ;;
  HARDEN-06)
    run_shell_syntax_checks
    check_tcp_pose_docs
    run_servo_gate_or_skip_missing_deps
    run_servo_pinocchio_gate
    run_optional_tcp_pose_acceptance
    ;;
  HARDEN-07)
    run_servo_gate_or_skip_missing_deps
    run_policy_runner_tests
    check_command_source_docs
    ;;
  HARDEN-08)
    run_shell_syntax_checks
    run_camera_gate_or_skip_missing_deps
    check_camera_acceptance_docs
    ;;
  HARDEN-09)
    run_shell_syntax_checks
    check_canonical_config_docs
    check_backend_contract_docs
    check_worker_docs
    check_command_source_docs
    run_simulator_tests
    run_gui_tests
    run_policy_runner_tests
    ;;
  HARDEN-10)
    run_mig26_gate
    ;;
  CART-HARDEN-05)
    run_cart_harden_05_gate
    ;;
  CART-MATH-01|CART-MATH-02|CART-MATH-03)
    run_cart_math_gate
    ;;
  SIM-HARDEN-01)
    run_simulator_tests
    run_servo_gate_or_skip_missing_deps
    ;;
  CART-TUNE-01|FAULT-DIAG-01)
    run_servo_gate_or_skip_missing_deps
    ;;
  CART-TUNE-02)
    run_cart_tune_02_gate
    ;;
  CART-ACCEPT-01)
    run_cart_accept_gate
    ;;
  CART-SERVO-01)
    run_cart_servo_01_gate
    ;;
  CART-SERVO-02)
    run_cart_servo_02_gate
    ;;
  CART-SERVO-03)
    run_cart_servo_03_gate
    ;;
  BENCH-CIRCLE-01)
    run_circle_benchmark_gate
    ;;
  BENCH-ABLATION-01)
    run_bench_ablation_gate
    ;;
  BENCH-CIRCLE-FEEDBACK-01)
    run_bench_circle_feedback_gate
    ;;
  CART-CIRCLE-SERVER-01)
    run_cart_circle_server_gate
    ;;
  BENCH-REPORT-01)
    run_bench_report_gate
    ;;
  GATE-RBSCRIPT-00)
    run_shell_syntax_checks
    ;;
  RBSCRIPT-TCP-01)
    run_rbscript_tcp_01_gate
    ;;
  RBSCRIPT-TCP-02)
    run_rbscript_tcp_02_gate
    ;;
  RBSCRIPT-ABLATION-01)
    run_rbscript_ablation_gate
    ;;
  RBSCRIPT-RATE-PROBE-01)
    run_rbscript_rate_probe_gate
    ;;
  RBSCRIPT-DOC-01)
    run_rbscript_doc_gate
    ;;
  GATE-RBPODO-00)
    run_shell_syntax_checks
    ;;
  RBPODO-SERVO-PARAM-01)
    run_shell_syntax_checks
    run_servo_gate_or_skip_missing_deps
    run_python_surface_tests
    ;;
  RBPODO-ACK-01)
    run_shell_syntax_checks
    run_servo_gate_or_skip_missing_deps
    run_python_surface_tests
    ;;
  RBPODO-ACCEPT-01)
    run_rbpodo_accept_gate
    ;;
  RBPODO-READONLY-DIAG-01)
    run_rbpodo_readonly_diag_gate
    ;;
  RBPODO-STATUS-FIELDS-01)
    run_rbpodo_status_fields_gate
    ;;
  RBPODO-JOINT-WRAP-01)
    run_rbpodo_joint_wrap_gate
    ;;
  RBPODO-BRINGUP-TOOLS-01)
    run_rbpodo_bringup_tools_gate
    ;;
  RBPODO-DOC-01)
    run_rbpodo_doc_gate
    ;;
  GATE-RBPODO-CIRCLE-00)
    run_shell_syntax_checks
    ;;
  RBPODO-CIRCLE-CONFIG-01)
    run_rbpodo_circle_config_gate
    ;;
  RBPODO-PGMODE-SIM-01)
    run_rbpodo_pgmode_sim_gate
    ;;
  RBPODO-REFERENCE-TCP-01)
    run_rbpodo_reference_tcp_gate
    ;;
  RBPODO-CONTROLLER-SIM-GATE-01)
    run_rbpodo_controller_sim_gate
    ;;
  RBPODO-CIRCLE-BENCH-01)
    run_rbpodo_circle_bench_gate
    ;;
  RBPODO-CIRCLE-ABLATION-01)
    run_rbpodo_circle_ablation_gate
    ;;
  RBPODO-CIRCLE-REPORT-01)
    run_rbpodo_circle_report_gate
    ;;
  RBPODO-CONTROLLER-SIM-CARTESIAN-00)
    run_shell_syntax_checks
    ;;
  RBPODO-CONTROLLER-SIM-CARTESIAN-01)
    run_rbpodo_controller_sim_cartesian_gate
    ;;
  RBPODO-CIRCLE-CONFIG-FIX-01)
    run_rbpodo_circle_config_fix_gate
    ;;
  RBPODO-CIRCLE-BENCH-FIX-01)
    run_rbpodo_circle_bench_fix_gate
    ;;
  RBPODO-CIRCLE-DOC-01)
    run_rbpodo_circle_doc_fix_gate
    ;;
  RBPODO-LIVE-VIZ-00)
    run_shell_syntax_checks
    ;;
  STATE-FANOUT-01)
    run_state_fanout_gate
    ;;
  GUI-TCP-REF-ACTUAL-01)
    run_gui_tcp_ref_actual_gate
    ;;
  BENCH-OVERLAY-UDP-01)
    run_bench_overlay_udp_gate
    ;;
  GUI-CIRCLE-OVERLAY-01)
    run_gui_circle_overlay_gate
    ;;
  RBPODO-CIRCLE-LIVE-RUNBOOK-01)
    run_rbpodo_circle_live_runbook_gate
    ;;
  RBPODO-TUNE-GATE-00)
    run_shell_syntax_checks
    ;;
  RBPODO-CIRCLE-PROFILES-02)
    run_rbpodo_circle_profile_tuning_gate
    ;;
  RBPODO-CIRCLE-ABLATION-OVERRIDES-01)
    run_rbpodo_circle_ablation_overrides_gate
    ;;
  RBPODO-CIRCLE-STAGE2-MATRICES-01)
    run_rbpodo_circle_stage2_matrices_gate
    ;;
  RBPODO-CIRCLE-TUNING-REPORT-01)
    run_rbpodo_circle_tuning_report_gate
    ;;
  RBPODO-CIRCLE-TUNE-RUNNERS-01)
    run_rbpodo_circle_tune_runners_gate
    ;;
  MEASURE-P0-GATE-00)
    run_shell_syntax_checks
    ;;
  RBPODO-MEASURE-STATE-PARITY-01)
    run_rbpodo_measure_state_parity_gate
    ;;
  RBPODO-MEASURE-RAW-DATA-01)
    run_rbpodo_measure_raw_data_gate
    ;;
  RBPODO-MEASURE-TIMESTAMP-01)
    run_rbpodo_measure_timestamp_gate
    ;;
  RBPODO-CIRCLE-ERROR-DECOMP-01)
    run_rbpodo_circle_error_decomp_gate
    ;;
  RBPODO-MEASURE-RELIABILITY-REPORT-01)
    run_rbpodo_measure_reliability_report_gate
    ;;
  GATE-RBPODO-500-P0-P1-00)
    run_shell_syntax_checks
    ;;
  P0-PARITY-REPAIR-01)
    run_p0_parity_repair_gate
    ;;
  P0-DIAGNOSTICS-ROOTCAUSE-01)
    run_p0_diagnostics_rootcause_gate
    ;;
  P0-RAW-PAYLOAD-FIXTURE-02)
    run_p0_raw_payload_fixture_gate
    ;;
  P0-MEASUREMENT-GATING-01)
    run_p0_measurement_gating_gate
    ;;
  RBPODO-500HZ-CONFIG-01)
    run_rbpodo_500hz_config_gate
    ;;
  RBPODO-500HZ-ACCEPT-01)
    run_rbpodo_500hz_accept_gate
    ;;
  RBPODO-500HZ-CIRCLE-MATRIX-01)
    run_rbpodo_500hz_circle_matrix_gate
    ;;
  RBPODO-500HZ-REPORT-01)
    run_rbpodo_500hz_report_gate
    ;;
  RBPODO-ASYNC-GATE-00)
    run_shell_syntax_checks
    ;;
  RBPODO-ASYNC-CONTRACT-01)
    run_rbpodo_async_contract_probe_gate
    ;;
  RBPODO-ASYNC-SDK-PROBE-01)
    run_rbpodo_async_contract_probe_gate
    ;;
  RBPODO-ASYNC-WORKER-01)
    run_rbpodo_async_cpp_gate
    ;;
  RBPODO-ASYNC-REFERENCE-SUPERVISOR-01)
    run_rbpodo_async_cpp_gate
    ;;
  RBPODO-ASYNC-500HZ-ACCEPT-01)
    run_rbpodo_async_acceptance_gate
    ;;
  RBPODO-ASYNC-CIRCLE-MATRIX-01)
    run_rbpodo_async_circle_matrix_gate
    ;;
  RBPODO-ASYNC-REPORT-01)
    run_rbpodo_async_report_gate
    ;;
  RBPODO-ASYNC-RUNBOOK-01)
    run_rbpodo_async_runbook_gate
    ;;
  ACKON500-RATE-ACCOUNTING-01)
    run_ackon500_rate_accounting_gate
    ;;
  BENCHMARK-LANE-CANONICALIZE-01)
    run_benchmark_lane_canonicalize_gate
    ;;
  01_promote_ackon500_defaults)
    run_gene_umi_control_defaults_gate
    ;;
  02_controller_sim_repeatability)
    run_gene_umi_controller_sim_repeatability_gate
    ;;
  03_pgmode_real_transition_acceptance)
    run_gene_umi_physical_transition_gate
    ;;
  04_umi_hdf5_audit_adapter)
    run_gene_umi_hdf5_audit_gate
    ;;
  05_umi_bimanual_collection)
    run_gene_umi_bimanual_collection_gate
    ;;
  06_flow_training_preflight_eval)
    run_gene_umi_flow_training_gate
    ;;
  07_policy_runner_rollout_modes)
    run_gene_umi_policy_rollout_modes_gate
    ;;
  08_dual_arm_real_policy_and_gripper)
    run_gene_umi_dual_arm_gripper_gate
    ;;
  09_docs_ci_artifact_manifest)
    run_gene_umi_docs_ci_artifact_manifest_gate
    ;;
  01_fix_controller_sim_safety_semantics)
    run_spacemouse_fix_controller_sim_safety_gate
    ;;
  02_pgmode_spacemouse_end_to_end_dryrun)
    run_pgmode_spacemouse_end_to_end_dryrun_gate
    ;;
  03_flow_infer_rollout_modes)
    run_spacemouse_flow_infer_rollout_modes_gate
    ;;
  04_flow_policy_tcp_twistlocal_controller_sim)
    run_flow_policy_tcp_twistlocal_controller_sim_gate
    ;;
  05_viser_pgmode_operator_view)
    run_viser_pgmode_operator_view_gate
    ;;
  06_gripper_and_dual_arm_policy_gate)
    run_spacemouse_gripper_dual_arm_policy_gate
    ;;
  07_ml_preflight_gate_stability)
    run_ml_preflight_gate_stability_gate
    ;;
  08_umi_hdf5_manifest_robustness)
    run_umi_hdf5_manifest_robustness_gate
    ;;
  09_artifact_manifest_docs_makefile)
    run_artifact_manifest_docs_makefile_gate
    ;;
  10_source_hygiene_local_configs)
    run_source_hygiene_local_configs_gate
    ;;
  ACKON500-BEST-PROFILE-PROMOTION-01)
    run_ackon500_best_profile_promotion_gate
    ;;
  ACKON500-REPEATABILITY-VALIDATION-01)
    run_ackon500_repeatability_validation_gate
    ;;
  PHYSICAL-READINESS-BLOCKERS-CLARITY-01)
    run_physical_readiness_blockers_clarity_gate
    ;;
  P1-CIRCLE-FACTOR-MATRIX-01)
    run_p1_circle_factor_matrix_gate
    ;;
  P1-ORIENTATION-DIAG-01)
    run_p1_orientation_diag_gate
    ;;
  P1-DEADTIME-PHASE-ADVANCE-01)
    run_p1_deadtime_phase_advance_gate
    ;;
  P1-SERVO-PARAM-SWEEP-01)
    run_p1_servo_param_sweep_gate
    ;;
  P1-SERVER-SIDE-CIRCLE-TRACK-SKELETON-01)
    run_p1_server_side_circle_track_skeleton_gate
    ;;
  POLICY-DATASET-SCHEMA-01)
    run_policy_dataset_schema_gate
    ;;
  GATE-BACKEND-COMPARE-00)
    run_shell_syntax_checks
    ;;
  BACKEND-COMPARE-CONFIG-01)
    run_backend_compare_config_gate
    ;;
  RBSCRIPT-PERSISTENT-PROBE-01)
    run_rbscript_persistent_probe_gate
    ;;
  RBSCRIPT-SERVO-NOOP-01)
    run_rbscript_servo_noop_gate
    ;;
  RBSCRIPT-DATA-PORT-01)
    run_rbscript_data_port_gate
    ;;
  BACKEND-COMPARE-MATRIX-01)
    run_backend_compare_matrix_gate
    ;;
  BACKEND-COMPARE-REPORT-01)
    run_backend_compare_report_gate
    ;;
  DOC-HYGIENE-01)
    run_doc_hygiene_gate
    ;;
  GUI-SPLIT-01)
    run_gui_split_gate
    ;;

  *)
    echo "ERROR: unknown task: $TASK" >&2
    exit 2
    ;;
esac
