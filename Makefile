COMPOSE ?= docker compose
COMPOSE_FILE ?= docker-compose.yml
PROJECT ?= robotics_lab
POLICY_HDF5_AUDIT_SMOKE ?= $(CODEX_UPLOADED_HDF5_SMOKE)
POLICY_HDF5_AUDIT_OUT ?= /tmp/robotics_lab_policy_hdf5_audit_smoke
FLOW_INFER_ARGS ?=

.PHONY: run flow-infer-real flow-infer-sim-offline flow-infer-training-replay build rebuild vm-up vm-down vm-status policy-hdf5-audit-smoke deps-hardware-free cam-up cam-up-wrists cam-status cam-down pgmode-sim-build pgmode-sim-up pgmode-sim-down ik-infeasible

# Full local teleop stack: rb_servo_server + viser GUI + policy_runner.
# SpaceMouse + UMI teleop run side by side (teleop_mux: the first to engage
# owns the robot until idle; a missing SpaceMouse degrades to UMI-only).
# It also starts gripper_server (UDP 50410/50420 -> local/sim Pika grippers),
# so this PC needs only `make run` — disable with GRIPPER_SERVER=0.
#   make run                  -> pgmode real (+ gripper_server)
#   make run MODE=sim         -> pgmode controller-simulation
#   make run VERBOSE=1        -> live teleop input + send/drop stats
#   make run GRIPPER_SERVER=0 -> skip the gripper server
MODE ?= real
run:
	./tools/run_stack.sh $(MODE)

# External OpenPI/flow policy entrypoint. Start `make run` first, then run this
# in another terminal; teleop_mux releases the lease while idle and flow-infer
# uses its own state readback port (50378), so ACTION_SOURCE=none is not needed.
flow-infer-real:
	./tools/flow_infer_real_policy.sh

# External OpenPI rollout on the rbpodo controller pgmode-simulation stack.
# Start `make run MODE=sim` and `scripts/offline_camera_replay.py` first.
# This target has no real-camera or real/gripper-motion authority. W6 is the
# pgmode-validated sim default; flow-infer-real keeps its existing W12 default.
flow-infer-sim-offline:
	FLOW_INFER_CONFIG=policy_runner/config/flow_sim_offline.yaml \
	FLOW_INFER_ROLLOUT_MODE=controller_sim \
	FLOW_INFER_CHUNK_EXECUTE_STEPS=6 \
	./tools/flow_infer_real_policy.sh $(FLOW_INFER_ARGS)

# Exact saved-observation replay into the live OpenPI server, followed by
# pgmode controller-simulation execution. Requires make run MODE=sim first.
# Set FLOW_TRAINING_EPISODE_HDF5; no live camera or physical gripper is used.
flow-infer-training-replay:
	./tools/flow_infer_training_episode_replay.sh $(FLOW_INFER_ARGS)

# Source-build the full local stack for DIRECT real-controller work (no VM):
# rb_servo_server (rbpodo backend, RB_SERVO_ENABLE_RBPODO=ON) into the path
# `make run` launches, + setcap, + rb_gui/policy_runner editable installs.
# Run this after editing source, then `make run`. Idempotent (rbpodo SDK
# install is one-time). Override jobs with BUILD_JOBS=N.
build:
	./tools/build_stack.sh

# Explicit hard reset for cache/toolchain/build-tree recovery. Normal source
# edits, including config.hpp layout changes, stay on the incremental build path.
rebuild:
	./tools/build_stack.sh --clean

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

deps-hardware-free:
	./scripts/install_deps_ubuntu.sh --profile hardware-free

# --- Camera (D435 head + dual D405 wrists, one container) ---
# 카메라 관련 make 타겟: cam-up / cam-up-wrists(head 없이) / cam-down / cam-status.
# `make run` 으로 로봇 스택을 띄운 뒤 `make cam-up` 한 줄이면 D435 헤드(color+IR) + 손목
# D405 2개 캡처가 한 컨테이너에서 뜬다.
# 다른 리그로 띄우려면 CAMERA_CONFIG 를 덮어쓴다(컨테이너 경로, ./camera_server/config 마운트):
#   make cam-up CAMERA_CONFIG=/app/config/quad_realsense_fisheye.yaml   # + 손목 피쉬아이
#   make cam-up CAMERA_CONFIG=/app/config/head_wrists.yaml   # 헤드 IR 640x480
CAMERA_CONFIG ?= /app/config/d435_head_1280x720.yaml
STEREO_CAM_JSON ?= /app/config/__no_advanced__.json
LIBREALSENSE_VERSION ?= 2.58.1
LIBREALSENSE_REF ?= bf2778061d5dd29776e9aca8765f75852671760b
LIBREALSENSE_BACKEND ?= native

cam-up:
	LIBREALSENSE_VERSION=$(LIBREALSENSE_VERSION) LIBREALSENSE_REF=$(LIBREALSENSE_REF) \
	LIBREALSENSE_BACKEND=$(LIBREALSENSE_BACKEND) \
	CAMERA_CONFIG=$(CAMERA_CONFIG) CAMERA_REALSENSE_JSON=$(STEREO_CAM_JSON) \
		$(COMPOSE) -p $(PROJECT) -f $(COMPOSE_FILE) --profile real_camera up -d --build camera_server
	@echo "camera_server (D435 head + dual D405) up. 상태: make cam-status / 로그: docker logs -f camera_server"

# rb_gui `카메라 품질` 탭의 `head view 표시 (5 Hz)` 는 이 타겟으로 뜬 head 색상
# 스트림(camera.bundle.stereo / head.color)을 보여준다. 모델 추론 입력은 그대로
# 손목 전용(camera.bundle.policy)이라 head 카메라를 켜도 정책이 보는 것은 안 바뀐다.

# head D435 없이 손목 D405 두 대만으로 기동(잦은 head/허브 USB 장애 격리용).
# 손목 RGB-D 번들(camera.bundle.policy + wrist_left/right)은 그대로 동작한다.
cam-up-wrists:
	$(MAKE) cam-up CAMERA_CONFIG=/app/config/dual_realsense_d405.yaml

# `docker ps -a`(-a 필수: 죽은 컨테이너도 보여준다) + 카메라별 USB 링크 속도 + capture 상태.
# USB3 미달 카메라 하나가 preflight에서 camera_server 전체를 죽이므로(required RealSense
# camera is not on USB3), 링크 속도를 capture 상태보다 먼저 보여준다.
cam-status:
	@docker ps -a --filter name=camera_server --format "container: {{.Status}}" | head -1 || true
	@docker logs camera_server 2>&1 | grep -E "\[run\] fps=" | tail -1 || echo "worker: (no fps yet)"
	@(ss -ltn 2>/dev/null || netstat -ltn 2>/dev/null) | grep -q 5601 \
		&& echo "stereo cloud: tcp 5601 LISTEN (viser 연동 가능)" \
		|| echo "stereo cloud: 5601 down (worker 미발행)"
	@rc=0; python3 tools/cam_usb_status.py || rc=1; \
	if ! docker ps --filter name=camera_server --format "{{.Status}}" | grep -q .; then \
		printf '\033[1;31mcamera: CONTAINER DOWN\033[0m\n'; \
		docker logs --tail 200 camera_server 2>&1 | grep -iE "fatal|error" | tail -3 | sed 's/^/  /'; \
		exit 1; \
	fi; \
	status_line=$$(docker logs --tail 300 camera_server 2>&1 | grep -E "\[CAM\] status=" | tail -1); \
	if [ -z "$$status_line" ]; then \
		echo "capture: (no status yet)"; exit 1; \
	fi; \
	echo "$$status_line"; \
	case "$$status_line" in \
	*"status=ok"*) \
		if [ $$rc -eq 0 ]; then printf '\033[32mcamera: OK\033[0m\n'; \
		else printf '\033[1;33mcamera: capture OK / usb link 이상 — 위 usb links 참고\033[0m\n'; fi ;; \
	*) printf '\033[1;31mcamera: %s\033[0m\n' "$$(echo "$$status_line" | sed -n 's/.*status=\([a-z]*\).*/\1/p' | tr 'a-z' 'A-Z')"; \
		echo "$$status_line" | tr '|' '\n' | grep "STALLED" | sed 's/^ */  stalled: /' || true; \
		echo "$$status_line" | grep -o 'reasons=.*' | sed 's/^/  /' || true; \
		exit 1 ;; \
	esac; \
	exit $$rc

cam-down:
	-docker stop camera_server
	-docker rm camera_server

# --- rbpodo pgmode-simulation (native; dual Virtual ControlBox VMs) ---
# One-command bring-up of rb_servo_server + rb_gui (viser) on this WSL box.
# Controller-simulation only; uses stack_sim.yaml and does not enable real motion.
pgmode-sim-build:
	bash tools/vm/pgmode_sim_build.sh

pgmode-sim-up:
	bash tools/vm/pgmode_sim_up.sh

pgmode-sim-down:
	bash tools/vm/pgmode_sim_down.sh
