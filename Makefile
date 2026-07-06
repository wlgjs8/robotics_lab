COMPOSE ?= docker compose
COMPOSE_FILE ?= docker-compose.yml
PROJECT ?= robotics_lab
POLICY_HDF5_AUDIT_SMOKE ?= $(CODEX_UPLOADED_HDF5_SMOKE)
POLICY_HDF5_AUDIT_OUT ?= /tmp/robotics_lab_policy_hdf5_audit_smoke

.PHONY: run flow-infer-real build vm-up vm-down vm-status policy-hdf5-audit-smoke pgmode-transition-dry-run mig-rebaseline deps-hardware-free cam-up cam-engine-rebuild cam-status cam-down pgmode-sim-build pgmode-sim-up pgmode-sim-down ik-infeasible

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

deps-hardware-free:
	./scripts/install_deps_ubuntu.sh --profile hardware-free

# --- Camera (D435 head stereo + dual D405 wrists + stereo_worker, one container) ---
# 카메라 관련 make 타겟은 4개로 통합: cam-up / cam-down / cam-status / cam-engine-rebuild.
# `make run` 으로 로봇 스택을 띄운 뒤 `make cam-up` 한 줄이면 D435 헤드(IR 스테레오) + 손목
# D405 2개 캡처와 스테레오 워커(viser 포인트클라우드 / 박스검출 / external-box 송신)가 한
# 컨테이너에서 함께 뜬다(run_all.sh 가 캡처+워커 둘 다 기동).
# 다른 리그로 띄우려면 CAMERA_CONFIG 를 덮어쓴다(컨테이너 경로, ./camera_server/config 마운트):
#   make cam-up CAMERA_CONFIG=/app/config/quad_realsense_fisheye.yaml   # + 손목 피쉬아이
#   make cam-up CAMERA_CONFIG=/app/config/head_wrists.yaml STEREO_CAM_K=/app/config/d435_ir_640x480_K.txt  # 헤드 IR 640x480
CAMERA_CONFIG ?= /app/config/d435_head_1280x720.yaml
STEREO_CAM_JSON ?= /app/config/__no_advanced__.json
# 헤드 1280x720 IR intrinsics는 camera_server가 기동 시 디바이스에서 덤프(아래 경로).
STEREO_CAM_K ?= /app/stereo_worker/d435_ir_1280x720_K.txt

cam-up:
	CAMERA_CONFIG=$(CAMERA_CONFIG) CAMERA_REALSENSE_JSON=$(STEREO_CAM_JSON) \
	STEREO_INTRINSICS=$(STEREO_CAM_K) \
		$(COMPOSE) -p $(PROJECT) -f $(COMPOSE_FILE) --profile real_camera up -d --build camera_server
	@echo "camera_server (D435 head + dual D405 + stereo_worker) up. 상태: make cam-status / 로그: docker logs -f camera_server"

# IR 1280x720(->736 패딩)용 TRT 엔진 재빌드. GPU+torch+tensorrt 필요 -> camera_server
# 컨테이너 안에서 실행. ONNX 재export 후 tf32 엔진 빌드까지 순차 수행.
cam-engine-rebuild:
	docker exec -it camera_server bash /app/stereo_worker/rebuild_engine_1280.sh
	@echo "엔진 재빌드 완료. worker 재기동(컨테이너 재시작) 후 반영."

cam-status:
	@docker ps --filter name=camera_server --format "container: {{.Status}}" || true
	@docker logs --tail 1 camera_server 2>&1 | grep -E "status=" || echo "capture: (no status yet)"
	@docker logs camera_server 2>&1 | grep -E "\[run\] fps=" | tail -1 || echo "worker: (no fps yet)"
	@(ss -ltn 2>/dev/null || netstat -ltn 2>/dev/null) | grep -q 5601 \
		&& echo "stereo cloud: tcp 5601 LISTEN (viser 연동 가능)" \
		|| echo "stereo cloud: 5601 down (worker 미발행)"

cam-down:
	-docker stop camera_server
	-docker rm camera_server

# --- rbpodo pgmode-simulation (native; dual Virtual ControlBox VMs) ---
# One-command bring-up of rb_servo_server + rb_gui (viser) on this WSL box.
# Controller-simulation only; never sets RB_ALLOW_REAL_CARTESIAN.
pgmode-sim-build:
	bash tools/vm/pgmode_sim_build.sh

pgmode-sim-up:
	bash tools/vm/pgmode_sim_up.sh

pgmode-sim-down:
	bash tools/vm/pgmode_sim_down.sh
