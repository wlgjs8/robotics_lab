COMPOSE ?= docker compose
COMPOSE_FILE ?= docker-compose.yml
PROJECT ?= robotics_lab

.PHONY: build deploy stop

build:
	$(COMPOSE) -p $(PROJECT) -f $(COMPOSE_FILE) build

deploy:
	$(COMPOSE) -p $(PROJECT) -f $(COMPOSE_FILE) up -d --remove-orphans

stop:
	$(COMPOSE) -p $(PROJECT) -f $(COMPOSE_FILE) down --remove-orphans
