# Developer Environment

This page documents reproducible local setup for simulator-first development. It does not enable real robot connection, real robot motion, RealSense capture, or real Cartesian/TCP motion.

## Ubuntu Hardware-Free Dependencies

Install the base C++ and Python dependencies:

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential \
  cmake \
  git \
  python3 \
  python3-venv \
  python3-pip \
  libyaml-cpp-dev \
  nlohmann-json3-dev
```

Then run:

```bash
./scripts/check_deps.sh --profile hardware-free
```

If your CMake packages are installed in a non-standard prefix, set:

```bash
export CMAKE_PREFIX_PATH=/path/to/prefix:$CMAKE_PREFIX_PATH
```

## Optional Kinematics Dependencies

Pinocchio is required for FK/IK-enabled Cartesian runtime acceptance.

Check availability:

```bash
./scripts/check_deps.sh --profile kinematics
```

If Pinocchio is installed in a custom prefix, set `CMAKE_PREFIX_PATH` before building.

## Python Checks

```bash
python3 -m unittest discover rb_gui/tests
python3 -m unittest discover policy_runner/tests
PYTHONPATH=rb_simulator/src python3 -m unittest discover rb_simulator/tests
python3 -m compileall -q rb_gui/rb_servo_gui policy_runner/policy_runner rb_simulator/src scripts
```

## C++ Hardware-Free Gate

```bash
./scripts/codex_gate.sh HARDEN-10
```

When dependencies are missing, the gate may skip C++ checks only if explicitly configured to do so. Do not report skipped C++ checks as passed C++ acceptance.

## Cartesian Simulator Acceptance

After C++ and Pinocchio dependencies are available:

```bash
CODEX_RUN_CARTESIAN_ACCEPTANCE=1 ./scripts/codex_gate.sh CART-HARDEN-05
```

This validates simulator-only PTP, Linear, and Twist primitives. It does not enable real robot motion.

## Docker Compose Simulator Stack

```bash
make sim-up
```

Open:

```text
http://127.0.0.1:8080
```

Stop with `Ctrl+C`, then:

```bash
make sim-down
```

## Real Camera Dependencies

Real camera work is separate from hardware-free robot work. Install RealSense and camera dependencies only for camera acceptance. See `docs/runbooks/camera_acceptance.md`.

## Real Robot Dependencies

Real robot work requires rbpodo and site-specific network access. Do not add or run tracked runnable real configs. Use `rb_servo_server/config/dual_real.example.yaml` only as a template and create site-local configs under `rb_servo_server/config/local/`.

## Recommended Development Flow

1. Run Python/unit checks.
2. Run hardware-free C++ gate.
3. Run simulator stack manually in GUI.
4. Run Cartesian simulator acceptance.
5. Repeat until simulator behavior is stable.
6. Only then start a separate real robot read-only acceptance workflow.
