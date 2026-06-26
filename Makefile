COMPOSE ?= docker compose
COMPOSE_FILE ?= docker-compose.yml
PROJECT ?= robotics_lab
POLICY_HDF5_AUDIT_SMOKE ?= $(CODEX_UPLOADED_HDF5_SMOKE)
POLICY_HDF5_AUDIT_OUT ?= /tmp/robotics_lab_policy_hdf5_audit_smoke

.PHONY: run flow-infer-real build vm-up vm-down vm-status policy-hdf5-audit-smoke pgmode-transition-dry-run mig-rebaseline deps-hardware-free camera-mock-up camera-real-up pgmode-sim-build pgmode-sim-up pgmode-sim-down ik-infeasible

# Full local teleop stack: rb_servo_server + viser GUI + policy_runner.
# SpaceMouse + UMI teleop run side by side (teleop_mux: the first to engage
# owns the robot until idle; a missing SpaceMouse degrades to UMI-only).
# In real mode it also starts umi_gripper_follow (UDP 50382 -> local Pika
# Grippers), so this PC needs only `make run` — disable with GRIPPER_FOLLOW=0.
#   make run                  -> pgmode real (+ gripper follower)
#   make run MODE=sim         -> pgmode controller-simulation
#   make run VERBOSE=1        -> live teleop input + send/drop stats
#   make run GRIPPER_FOLLOW=0 -> skip the gripper follower
MODE ?= sim
run:
	./tools/run_stack.sh $(MODE)

# External OpenPI/flow policy entrypoint. Start `make run` first, then run this
# in another terminal; teleop_mux releases the lease while idle and flow-infer
# uses its own state readback port (50378), so ACTION_SOURCE=none is not needed.
flow-infer-real:
	./tools/flow_infer_real_policy.sh

# Source-build the full local stack for DIRECT real-controller work (no VM):
# rb_servo_server (rbpodo backend, RB_SERVO_ENABLE_RBPODO=ON) into the path
# `make run` launches, + setcap, + rb_gui/policy_runner editable installs.
# Run this after editing source, then `make run`. Idempotent (rbpodo SDK
# install is one-time). Override jobs with BUILD_JOBS=N.
build:
	./tools/build_stack.sh

# Rainbow VIRTUAL control-box VMs (vendor OVA): boot 2 VMs and map them to the
# real controller IPs so `make run MODE=sim` works without hardware.
# Regenerate the viser "A 영역 (특이점 원통)" overlay asset: a per-arm base-axis
# (J1) velocity-singularity cylinder (vendor "A 영역"), radius R = v_ref/dq_max,
# axial extent FK-clipped to the reach envelope. Pure Python (seconds); only
# needed when the URDF / mount geometry or the speed cap changes. Tunables:
# IK_CYL_SPEED (m/s, default 0.25 = SMD max_linear_velocity_m_s), IK_CYL_DQMAX
# (deg/s, default 60), IK_CYL_RADIUS (m, explicit override).
ik-infeasible:
	python3 tools/ik_infeasible_region.py \
		--speed-mps $(or $(IK_CYL_SPEED),0.25) \
		--dqmax-deg $(or $(IK_CYL_DQMAX),60) \
		$(if $(IK_CYL_RADIUS),--radius-m $(IK_CYL_RADIUS),)

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
# This site runs two D405 wrist cameras for flow-infer; other profiles:
#   make camera-real-up CAMERA_CONFIG=/app/config/triple_realsense.yaml      # 3-camera
#   make camera-real-up CAMERA_CONFIG=/app/config/quad_realsense_fisheye.yaml # + wrist fisheye (fe65 deploy)
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
