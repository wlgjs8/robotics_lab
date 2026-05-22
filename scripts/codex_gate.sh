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
  cmake -S rb_servo_server -B rb_servo_server/build/hardware_free_gate \
    -DCMAKE_BUILD_TYPE=Debug \
    -DRB_SERVO_ENABLE_RBPODO=OFF \
    -DRB_SERVO_ENABLE_PINOCCHIO=OFF \
    -DRB_SERVO_ALLOW_FETCHCONTENT=OFF \
    -DBUILD_TESTING=ON

  cmake --build rb_servo_server/build/hardware_free_gate -j
  ctest --test-dir rb_servo_server/build/hardware_free_gate --output-on-failure
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

case "$TASK" in
  P0-A)
    grep -R "RB_ALLOW_REAL_MOTION" README.md docs >/dev/null
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
    grep -R "geometry_valid_for_real_policy" calibration geometry docs README.md >/dev/null 2>&1
    grep -R "configured_estimate" calibration geometry docs README.md >/dev/null 2>&1
    ;;

  P3-F)
    bash -n scripts/tcp_pose_simulator_acceptance.sh
    grep -R "RB_ALLOW_REAL_CARTESIAN" docs/runbooks/tcp_pose_simulator_acceptance.md README.md >/dev/null 2>&1
    ;;

  *)
    echo "ERROR: unknown task: $TASK" >&2
    exit 2
    ;;
esac