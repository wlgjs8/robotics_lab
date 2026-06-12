COMPOSE ?= docker compose
COMPOSE_FILE ?= docker-compose.yml
PROJECT ?= robotics_lab
FLOW_COMPOSE_FILE ?= docker-compose.flow-train.yml
FLOW_TRAIN_SERVICES ?= policy_flow_train_gpu0 policy_flow_train_gpu1 policy_flow_train_gpu2 policy_flow_train_gpu3 policy_flow_train_gpu4 policy_flow_train_gpu5 policy_flow_train_gpu6 policy_flow_train_gpu7
FLOW_EXPECTED_GPU_COUNT ?= 8
FLOW_RUN_UID ?= $(shell id -u)
FLOW_RUN_GID ?= $(shell id -g)
POLICY_FLOW_AUDIT_HOST_OUT ?= outputs/flow_runs/audit
POLICY_FLOW_AUDIT_CONTAINER_OUT ?= /outputs/flow_runs/audit
POLICY_HDF5_AUDIT_SMOKE ?= $(CODEX_UPLOADED_HDF5_SMOKE)
POLICY_HDF5_AUDIT_OUT ?= /tmp/robotics_lab_policy_hdf5_audit_smoke
export FLOW_EXPECTED_GPU_COUNT FLOW_RUN_UID FLOW_RUN_GID

.PHONY: run vm-up vm-down vm-status build deploy stop policy-train policy-flow-train-config policy-flow-train-build policy-flow-gpu-smoke policy-flow-train-preflight policy-flow-hdf5-audit policy-flow-train-up policy-flow-train-down policy-hdf5-audit-smoke policy-flow-smoke pgmode-transition-dry-run mig-rebaseline deps-hardware-free camera-mock-up camera-real-up pgmode-sim-build pgmode-sim-up pgmode-sim-down

# Full local teleop stack: rb_servo_server + viser GUI + policy_runner.
# SpaceMouse + UMI teleop run side by side (teleop_mux: the first to engage
# owns the robot until idle; a missing SpaceMouse degrades to UMI-only).
#   make run                  -> pgmode real
#   make run MODE=sim         -> pgmode controller-simulation
#   make run VERBOSE=1        -> live teleop input + send/drop stats
MODE ?= real
run:
	./tools/run_stack.sh $(MODE)

# Rainbow VIRTUAL control-box VMs (vendor OVA): boot 2 VMs and map them to the
# real controller IPs so `make run MODE=sim` works without hardware.
vm-up:
	./tools/vm_stack.sh up
vm-down:
	./tools/vm_stack.sh down
vm-status:
	./tools/vm_stack.sh status

build:
	$(COMPOSE) -p $(PROJECT) -f $(COMPOSE_FILE) build

deploy:
	$(COMPOSE) -p $(PROJECT) -f $(COMPOSE_FILE) up -d --remove-orphans

stop:
	$(COMPOSE) -p $(PROJECT) -f $(COMPOSE_FILE) down --remove-orphans

policy-train:
	$(COMPOSE) -p $(PROJECT) -f $(COMPOSE_FILE) --profile ml run --rm policy_train

policy-flow-train-config:
	$(COMPOSE) -p $(PROJECT) -f $(FLOW_COMPOSE_FILE) --profile flow-train config

policy-flow-train-build:
	$(COMPOSE) -p $(PROJECT) -f $(FLOW_COMPOSE_FILE) --profile flow-train build

policy-flow-gpu-smoke:
	FLOW_EXPECTED_GPU_COUNT=$(FLOW_EXPECTED_GPU_COUNT) $(COMPOSE) -p $(PROJECT) -f $(FLOW_COMPOSE_FILE) --profile flow-train run --rm --no-deps policy_flow_gpu_smoke

policy-flow-train-preflight:
	$(COMPOSE) -p $(PROJECT) -f $(FLOW_COMPOSE_FILE) --profile flow-train run --rm --no-deps --entrypoint policy-runner policy_flow_train_gpu0 ml-preflight --vision-backbone resnet18
	$(COMPOSE) -p $(PROJECT) -f $(FLOW_COMPOSE_FILE) --profile flow-train run --rm --no-deps --entrypoint policy-runner policy_flow_train_gpu0 ml-preflight --vision-backbone resnet50

policy-flow-hdf5-audit:
	mkdir -p "$(POLICY_FLOW_AUDIT_HOST_OUT)"
	$(COMPOSE) -p $(PROJECT) -f $(FLOW_COMPOSE_FILE) --profile flow-train run --rm --no-deps --entrypoint policy-runner policy_flow_train_gpu0 hdf5-audit \
		--episodes-dir /data/policy_episodes \
		--output-json "$(POLICY_FLOW_AUDIT_CONTAINER_OUT)/hdf5_audit.json" \
		--output-md "$(POLICY_FLOW_AUDIT_CONTAINER_OUT)/hdf5_audit.md"

policy-flow-train-up:
	$(COMPOSE) -p $(PROJECT) -f $(FLOW_COMPOSE_FILE) --profile flow-train up --build $(FLOW_TRAIN_SERVICES)

policy-flow-train-down:
	$(COMPOSE) -p $(PROJECT) -f $(FLOW_COMPOSE_FILE) down --remove-orphans

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

policy-flow-smoke:
	PYTHONPATH=policy_runner python3 -m policy_runner ml-preflight --vision-backbone tiny_cnn

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
