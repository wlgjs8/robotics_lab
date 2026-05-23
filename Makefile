COMPOSE ?= docker compose
COMPOSE_FILE ?= docker-compose.yml
PROJECT ?= robotics_lab

.PHONY: build deploy stop sim-up sim-down sim-smoke mig-rebaseline deps-hardware-free camera-mock-up camera-real-up

build:
	$(COMPOSE) -p $(PROJECT) -f $(COMPOSE_FILE) build

deploy:
	$(COMPOSE) -p $(PROJECT) -f $(COMPOSE_FILE) up -d --remove-orphans

stop:
	$(COMPOSE) -p $(PROJECT) -f $(COMPOSE_FILE) down --remove-orphans

sim-up:
	$(COMPOSE) -p $(PROJECT) -f $(COMPOSE_FILE) up --build rb_gui rb_simulator_left rb_simulator_right rb_servo_server

sim-down:
	$(COMPOSE) -p $(PROJECT) -f $(COMPOSE_FILE) down --remove-orphans

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
