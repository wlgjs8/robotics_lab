COMPOSE ?= docker compose
COMPOSE_FILE ?= docker-compose.yml
PROJECT ?= robotics_lab
POLICY_HDF5_AUDIT_SMOKE ?= $(CODEX_UPLOADED_HDF5_SMOKE)
POLICY_HDF5_AUDIT_OUT ?= /tmp/robotics_lab_policy_hdf5_audit_smoke

.PHONY: run build vm-up vm-down vm-status policy-hdf5-audit-smoke pgmode-transition-dry-run mig-rebaseline deps-hardware-free camera-mock-up camera-real-up pgmode-sim-build pgmode-sim-up pgmode-sim-down ik-infeasible

# Full local teleop stack: rb_servo_server + viser GUI + policy_runner.
# SpaceMouse + UMI teleop run side by side (teleop_mux: the first to engage
# owns the robot until idle; a missing SpaceMouse degrades to UMI-only).
# In real mode it also starts umi_gripper_follow (UDP 50382 -> local Pika
# Grippers), so this PC needs only `make run` — disable with GRIPPER_FOLLOW=0.
#   make run                  -> pgmode real (+ gripper follower)
#   make run MODE=sim         -> pgmode controller-simulation
#   make run VERBOSE=1        -> live teleop input + send/drop stats
#   make run GRIPPER_FOLLOW=0 -> skip the gripper follower
#   ACTION_SOURCE=none make run
MODE ?= sim
run:
	./tools/run_stack.sh $(MODE)

# Source-build the full local stack for DIRECT real-controller work (no VM):
# rb_servo_server (rbpodo backend, RB_SERVO_ENABLE_RBPODO=ON) into the path
# `make run` launches, + setcap, + rb_gui/policy_runner editable installs.
# Run this after editing source, then `make run`. Idempotent (rbpodo SDK
# install is one-time). Override jobs with BUILD_JOBS=N.
build:
	./tools/build_stack.sh

# Rainbow VIRTUAL control-box VMs (vendor OVA): boot 2 VMs and map them to the
# real controller IPs so `make run MODE=sim` works without hardware.
# Regenerate the viser "IK 불가 영역" overlay asset: builds the C++ feasibility
# sampler (reuses the server's real Pinocchio IK), samples the workspace grid,
# and writes rb_servo_server/descriptions/ik_infeasible_rb3_730e.npz. Slow
# (minutes); only needed when the URDF / mount geometry changes. Tunables:
# IK_SPACING, IK_ORIENTATIONS, IK_SEEDS.
ik-infeasible:
	cmake -S rb_servo_server -B rb_servo_server/build
	cmake --build rb_servo_server/build --target ik_feasibility_grid -j
	python3 tools/ik_infeasible_region.py \
		--spacing-m $(or $(IK_SPACING),0.05) \
		--orientations $(or $(IK_ORIENTATIONS),18) \
		--down-rolls $(or $(IK_DOWN_ROLLS),8) \
		--seeds $(or $(IK_SEEDS),2)

vm-up:
	./tools/vm_stack.sh up
vm-down:
	./tools/vm_stack.sh down
vm-status:
	./tools/vm_stack.sh status

policy-hdf5-audit-smoke:
	mkdir -p "$(POLICY_HDF5_AUDIT_OUT)"
	if [ -n "$(POLICY_HDF5_AUDIT_SMOKE)" ] || [ -e episode_002.hdf5 ]; then \
		episodes="$(POLICY_HDF5_AUDIT_SMOKE)"; \
		if [ -z "$$episodes" ]; then episodes="episode_002.hdf5"; fi; \
		PYTHONPATH=policy_runner python3 -m policy_runner hdf5-audit \
			--episodes-dir "$$episodes" \
			--output-json "$(POLICY_HDF5_AUDIT_OUT)/hdf5_audit_smoke.json" \
			--output-md "$(POLICY_HDF5_AUDIT_OUT)/hdf5_audit_smoke.md"; \
	else \
		echo "policy-hdf5-audit-smoke: no CODEX_UPLOADED_HDF5_SMOKE or episode_002.hdf5; skipped"; \
	fi

pgmode-transition-dry-run:
	tools/rbpodo_pgmode_spacemouse.sh check

mig-rebaseline:
	./scripts/codex_gate.sh MIG-26

deps-hardware-free:
	./scripts/install_deps_ubuntu.sh --profile hardware-free

camera-mock-up:
	$(COMPOSE) -p $(PROJECT) -f $(COMPOSE_FILE) --profile mock_camera up --build camera_server_mock

# Container path of the camera config (mounted from ./camera_server/config).
# This site runs two D405 wrist cameras for flow-infer; the 3-camera
# triple_realsense profile is available via
#   make camera-real-up CAMERA_CONFIG=/app/config/triple_realsense.yaml
CAMERA_CONFIG ?= /app/config/dual_realsense_d405.yaml

camera-real-up:
	CAMERA_CONFIG=$(CAMERA_CONFIG) $(COMPOSE) -p $(PROJECT) -f $(COMPOSE_FILE) --profile real_camera up --build camera_server

# --- rbpodo pgmode-simulation (native; dual Virtual ControlBox VMs) ---
# One-command bring-up of rb_servo_server + rb_gui (viser) on this WSL box.
# Controller-simulation only; never sets RB_ALLOW_REAL_CARTESIAN.
pgmode-sim-build:
	bash tools/vm/pgmode_sim_build.sh

pgmode-sim-up:
	bash tools/vm/pgmode_sim_up.sh

pgmode-sim-down:
	bash tools/vm/pgmode_sim_down.sh
