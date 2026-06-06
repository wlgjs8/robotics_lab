COMPOSE ?= docker compose
COMPOSE_FILE ?= docker-compose.yml
PROJECT ?= robotics_lab
SIM_BACKEND_COMPOSE_FILE ?= docker-compose.sim-backend.yml
SIM_CONTROL_COMPOSE_FILE ?= docker-compose.sim-control.yml
FLOW_COMPOSE_FILE ?= docker-compose.flow-train.yml
FLOW_TRAIN_SERVICES ?= policy_flow_train_gpu0 policy_flow_train_gpu1 policy_flow_train_gpu2 policy_flow_train_gpu3 policy_flow_train_gpu4 policy_flow_train_gpu5 policy_flow_train_gpu6 policy_flow_train_gpu7
POLICY_FLOW_AUDIT_HOST_OUT ?= outputs/flow_runs/audit
POLICY_FLOW_AUDIT_CONTAINER_OUT ?= /outputs/flow_runs/audit
POLICY_HDF5_AUDIT_SMOKE ?= $(CODEX_UPLOADED_HDF5_SMOKE)
POLICY_HDF5_AUDIT_OUT ?= /tmp/robotics_lab_policy_hdf5_audit_smoke

.PHONY: build deploy stop sim-local-up sim-up sim-backend-up sim-control-up sim-down sim-smoke sim-teleop-up sim-infer-up policy-train policy-flow-train-config policy-flow-train-build policy-flow-train-preflight policy-flow-hdf5-audit policy-flow-train-up policy-flow-train-down policy-hdf5-audit-smoke policy-flow-smoke pgmode-transition-dry-run mig-rebaseline deps-hardware-free camera-mock-up camera-real-up

build:
	$(COMPOSE) -p $(PROJECT) -f $(COMPOSE_FILE) build

deploy:
	$(COMPOSE) -p $(PROJECT) -f $(COMPOSE_FILE) up -d --remove-orphans

stop:
	$(COMPOSE) -p $(PROJECT) -f $(COMPOSE_FILE) down --remove-orphans

sim-local-up:
	$(COMPOSE) -p $(PROJECT) -f $(COMPOSE_FILE) up --build rb_gui rb_simulator_left rb_simulator_right policy_runner_record rb_servo_server

sim-up: sim-local-up

sim-backend-up:
	$(COMPOSE) -p $(PROJECT) -f $(COMPOSE_FILE) -f $(SIM_BACKEND_COMPOSE_FILE) up --build rb_simulator_left rb_simulator_right

sim-control-up:
	$(COMPOSE) -p $(PROJECT) -f $(COMPOSE_FILE) -f $(SIM_CONTROL_COMPOSE_FILE) up --build rb_gui policy_runner_record rb_servo_server

sim-down:
	$(COMPOSE) -p $(PROJECT) -f $(COMPOSE_FILE) down --remove-orphans

sim-teleop-up:
	$(COMPOSE) -p $(PROJECT) -f $(COMPOSE_FILE) --profile teleop up --build rb_gui rb_simulator_left rb_simulator_right policy_runner_teleop_record rb_servo_server

sim-infer-up:
	$(COMPOSE) -p $(PROJECT) -f $(COMPOSE_FILE) --profile infer up --build rb_gui rb_simulator_left rb_simulator_right policy_runner_infer rb_servo_server

policy-train:
	$(COMPOSE) -p $(PROJECT) -f $(COMPOSE_FILE) --profile ml run --rm policy_train

policy-flow-train-config:
	$(COMPOSE) -p $(PROJECT) -f $(FLOW_COMPOSE_FILE) --profile flow-train config

policy-flow-train-build:
	$(COMPOSE) -p $(PROJECT) -f $(FLOW_COMPOSE_FILE) --profile flow-train build

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

sim-smoke:
	./scripts/hardware_free_validation.sh

mig-rebaseline:
	./scripts/codex_gate.sh MIG-26

deps-hardware-free:
	./scripts/install_deps_ubuntu.sh --profile hardware-free

camera-mock-up:
	$(COMPOSE) -p $(PROJECT) -f $(COMPOSE_FILE) --profile mock_camera up --build camera_server_mock

camera-real-up:
	$(COMPOSE) -p $(PROJECT) -f $(COMPOSE_FILE) --profile real_camera up --build camera_server
