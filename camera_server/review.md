> Historical review/planning document. Some findings may be obsolete.
> The current source of truth is root README.md and docs/architecture.md.

검토했습니다. 결론부터 말하면, 구조 방향은 좋고 mock/simulation 기반 MVP로는 꽤 잘 구현되어 있습니다.
다만 “RealSense 3대 30fps를 sync 맞춰서 policy input으로 안정적으로 공급”이라는 목표 기준에서는 아직 몇 가지 중요한 수정이 필요합니다.

제가 본 현재 상태는 이렇습니다.

현재 camera_server:
  - mock camera 기반 구조 검증: Go
  - shared memory ring buffer 기본 구조: 좋음
  - ZMQ metadata publish: 좋음
  - policy_runner 샘플: 좋음
  - 실제 RealSense 3대 장시간 운용: 아직 No-Go
  - hardware sync 정확도 보장: 아직 No-Go
  - frame loss 없는 recording: 아직 No-Go

참고로 제 환경에서는 yaml-cpp 개발 패키지가 없어 CMake 빌드는 끝까지 못 돌렸습니다. 대신 zip 안의 코드 흐름을 기준으로 정적 검토했습니다. Dockerfile에는 libyaml-cpp-dev가 들어 있어서 해당 패키지가 있는 환경에서는 빌드될 수 있습니다.

1. 잘 구현된 부분
구조 분리 방향이 좋음

전체 구조가 제가 제안했던 방향과 잘 맞습니다.

camera_server
  - RealSense / mock camera capture
  - shared memory ring buffer
  - ZMQ metadata publish
  - health monitor
  - recorder
  - policy_runner sample tools

rb_servo_server와 완전히 분리되어 있고, policy_runner가 camera_server와 rb_servo_server 양쪽을 구독/제어하는 구조로 가기 좋습니다.

POSIX shared memory ring buffer 구조가 좋음

SharedMemoryRingBuffer가 stream별 ring을 만들고, 각 slot에 header + payload를 둔 구조입니다.

/camera_server_frames
  ├── head.color ring
  ├── left_wrist.color ring
  └── right_wrist.color ring

그리고 seq_begin, seq_end를 이용한 seqlock 스타일 read/write도 방향이 좋습니다.
policy_runner가 slot_index, shm_offset, seq를 보고 최신 프레임을 읽는 구조는 맞습니다.

metadata schema도 좋음

FrameMeta, FrameBundleMeta, HealthSnapshot 구성이 좋습니다.

특히 아래 정보가 들어가는 건 좋습니다.

frame_number
host_arrival_time_ns
sensor_timestamp_ns
realsense_timestamp_ms
shm_name
ring_name
slot_index
shm_offset
size_bytes
drop_counters
max_time_diff_ms
complete

이 정도면 policy_runner와 dataset_builder가 필요한 기본 정보를 받을 수 있습니다.

Docker 구성 방향도 좋음

container_name: camera_server, ipc: host, shm_size: 2gb, /dev/bus/usb mount는 방향이 맞습니다.

container_name: camera_server
ipc: host
shm_size: "2gb"
network_mode: host
devices:
  - /dev/bus/usb:/dev/bus/usb

shared memory 기반이면 ipc: host를 둔 것이 중요합니다.

2. 가장 중요한 P0 이슈
P0-1. 현재 FrameSynchronizer는 진짜 “3-camera 30fps bundle sync”가 아님

현재 FrameSynchronizer::push_frame()은 각 stream의 latest frame만 저장합니다.

latest_[meta.ring_name] = meta;

그 다음 required stream이 모두 있으면 bundle을 만듭니다.

문제는, 한 번 모든 stream이 채워진 뒤부터는 카메라 한 대에서 새 프레임이 올 때마다 bundle이 만들어질 수 있다는 점입니다.

3대 RGB 카메라가 각각 30fps이면 기대하는 bundle은 보통:

30 bundles/sec

이어야 합니다.

그런데 현재 구조는 startup 이후:

head frame 들어옴       → bundle publish
left_wrist frame 들어옴 → bundle publish
right_wrist frame 들어옴→ bundle publish

이렇게 되어 최대 90 bundles/sec가 나올 수 있습니다.
그리고 각 bundle은 “동시에 들어온 3개 frame”이 아니라 “방금 들어온 frame + 나머지 두 카메라의 latest frame” 조합입니다.

즉, 이름은 nearest_timestamp지만 실제 구현은:

latest cache bundle

에 가깝습니다.

권장 수정

bundle 발행 정책을 명확히 해야 합니다.

가장 추천하는 방식은 master-camera-driven bundling입니다.

master_camera = head

head frame이 들어올 때만 bundle 생성 시도
  - left_wrist buffer에서 head timestamp와 가장 가까운 frame 선택
  - right_wrist buffer에서 head timestamp와 가장 가까운 frame 선택
  - max_time_diff_ms 이내면 complete bundle publish

이를 위해 latest_ 하나만 두지 말고 stream별 작은 deque를 둡니다.

std::map<std::string, std::deque<FrameMeta>> buffers_;

흐름:

1. 모든 frame은 각 stream buffer에 저장
2. master stream frame이 들어왔을 때만 bundle attempt
3. 각 non-master stream에서 timestamp nearest frame 선택
4. skew가 threshold 이내면 publish
5. 사용한 오래된 frame은 정리

hardware sync를 쓰는 경우는 더 단순합니다.

frame_number 기준으로 head/left/right 같은 frame_number끼리 묶기

지금 구조는 policy 입력으로 쓰기에는 bundle rate와 sync 의미가 애매해질 수 있으므로, 이건 먼저 고치는 게 좋습니다.

P0-2. sync.mode: hardware가 실제 RealSense hardware sync를 설정하지 않음

현재 코드는:

hardware_synced_(cfg.sync.mode == "hardware")

로만 metadata flag를 세웁니다.

하지만 RealSense 장치에 실제로 master/slave sync mode를 설정하는 코드는 없습니다.

즉, config에서:

sync:
  mode: hardware

로 설정하면 metadata에는 hardware_synced=true가 나갈 수 있지만, 실제 카메라는 hardware sync 설정이 안 되어 있을 수 있습니다.

이건 위험합니다. policy나 dataset 쪽에서 “hardware synced frame”이라고 믿게 되기 때문입니다.

권장 수정

sync.mode=hardware일 때는 둘 중 하나여야 합니다.

1. 실제 RealSense inter_cam_sync_mode 설정 성공
   → hardware_synced=true

2. 설정 미구현 또는 실패
   → startup fail
   또는 hardware_synced=false

즉, 절대 단순 flag만 세우면 안 됩니다.

RealSense 쪽 구현에는 대략 이런 개념이 필요합니다.

// 예시 개념
if (camera_name == master_camera) {
    depth_sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, 1); // master
} else {
    depth_sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, 2); // slave
}

정확한 지원 여부는 사용 중인 모델별로 확인해야 합니다. 지원하지 않는 모델이면 hardware 모드에서 startup fail이 맞습니다.

P0-3. SharedMemoryRingBuffer write가 thread-safe하지 않음

현재 각 RealSense camera는 별도 pipeline callback으로 들어올 수 있고, mock camera도 카메라별 thread입니다.

즉, CameraManager::handle_frame()은 여러 thread에서 동시에 호출될 수 있습니다.

그런데 SharedMemoryRingBuffer::write_frame() 내부에서:

RingInfo& ring = it->second;
const uint32_t slot_idx = ring.next_slot++ % ring.desc->slot_count;

처럼 ring.next_slot을 일반 uint32_t로 증가시킵니다.

각 stream ring이 서로 다르면 대부분 문제 없어 보일 수 있지만, C++ 관점에서는 rings_ map 접근과 내부 RingInfo 변경이 완전히 thread-safe하다고 보기 어렵습니다. 특히 같은 stream에 color frame callback이 겹치거나, RealSense callback scheduling이 바뀌면 data race 가능성이 있습니다.

권장 수정

간단한 방법:

std::mutex write_mutex_;

를 SharedMemoryRingBuffer에 넣고 write_frame() 전체를 lock합니다.

하지만 성능을 생각하면 더 좋은 구조는:

struct RingInfo {
    shm_layout::StreamRingHeader* desc{nullptr};
    std::atomic<uint32_t> next_slot{0};
    std::mutex write_mutex;
};

입니다.

최소 수정은 per-ring mutex입니다.

std::lock_guard<std::mutex> lk(ring.write_mutex);
const uint32_t slot_idx = ring.next_slot++ % ring.desc->slot_count;

3대 30fps, 640x480 정도라면 전체 mutex도 일단 괜찮습니다.
하지만 나중에 depth까지 켜거나 해상도를 올릴 생각이면 per-ring lock이 낫습니다.

P0-4. RealSense SDK callback 안에서 너무 많은 일을 함

현재 RealSense callback 흐름은 이렇습니다.

RealSense callback
  → frame을 vector로 copy
  → CameraManager::handle_frame()
    → shared memory write
    → stats mutex lock
    → recorder enqueue
    → synchronizer push
    → ZMQ publish_bundle

callback 안에서 shared memory write 정도까지는 허용 가능하지만, synchronizer, ZMQ publish, stats lock, recorder enqueue까지 들어가면 callback이 늦어질 수 있습니다.

카메라 callback이 늦어지면 결국 frame drop 가능성이 증가합니다.

권장 수정

RealSense callback에서는 가능한 최소만 해야 합니다.

RealSense callback
  → host timestamp 찍기
  → frame data를 shm에 write
  → FrameMeta를 lock-free/SPSC queue에 push
  → return

그리고 별도 metadata thread에서:

metadata thread
  → FrameMeta queue pop
  → stats update
  → synchronizer
  → ZMQ publish
  → health update

즉, CameraManager::handle_frame()를 두 단계로 나누는 게 좋습니다.

capture fast path:
  frame → shm → meta_queue

metadata path:
  meta_queue → synchronizer → ZMQ
P0-5. recording metadata JSONL이 multi-thread unsafe

Recorder는 writer thread를 여러 개 띄울 수 있습니다.

for (uint32_t i = 0; i < threads; ++i)
    workers_.emplace_back(&Recorder::worker_loop, this, i);

그런데 각 worker가 동시에 같은 파일에 append합니다.

std::ofstream meta(metadata, std::ios::app);
meta << line << ...

여러 thread가 같은 camera_metadata.jsonl에 동시에 append하면 JSON line이 interleave되거나 순서가 꼬일 수 있습니다.

또한 매 프레임마다 ofstream을 열고 닫는 것도 overhead가 큽니다.

권장 수정

두 가지 중 하나가 필요합니다.

간단한 수정

metadata write에 mutex 추가:

std::mutex metadata_mutex_;
{
    std::lock_guard<std::mutex> lk(metadata_mutex_);
    metadata_file << line << '\n';
}
더 좋은 수정

image writer와 metadata writer를 분리합니다.

writer threads:
  raw image write
  metadata record를 metadata_queue에 push

metadata_writer thread:
  camera_metadata.jsonl 단일 append

장기적으로는 두 번째가 좋습니다.

3. P1 이슈
P1-1. recording filename이 frame_number만 사용함

현재 raw file 경로가:

camera_name/stream/frame_number.format

입니다.

예:

head/color/123.rgb8

이 방식은 카메라가 재시작되거나 frame_number가 reset되면 파일이 덮어써질 수 있습니다.

권장:

head/color/000000000123456789_fn123.rgb8

또는:

head/color/{recording_seq}_{frame_number}_{host_time_ns}.rgb8

최소한 host_arrival_time_ns나 recording_seq를 filename에 포함하는 게 좋습니다.

P1-2. publish_incomplete_bundles 설정이 사실상 무시됨

FrameSynchronizer::push_frame()는 incomplete bundle도 반환할 수 있습니다.

하지만 CameraManager::handle_frame()에서:

if (bundle && bundle->complete) publisher_.publish_bundle(*bundle);

로 되어 있어서 incomplete bundle은 publish되지 않습니다.

즉:

publish_incomplete_bundles: true

로 설정해도 실제 publish는 안 됩니다.

둘 중 하나로 정리하면 됩니다.

1. incomplete bundle publish를 지원하지 않을 거면 config 제거
2. 지원할 거면 bundle.has_value()면 publish

policy input에는 complete bundle만 쓰는 게 맞으므로, 저는 초기에는 config를 제거하거나 문서에 “currently not published”라고 적는 게 낫다고 봅니다.

P1-3. Reconnect config가 있지만 실제 reconnect가 없음

config에는:

reconnect:
  enabled: true

가 있지만 실제 CameraManager는 start 시점에 카메라를 열고, 이후 camera disconnect/reconnect를 관리하지 않습니다.

RealSense 3대를 실제 로봇 손목에 달면 USB 순간 disconnect 가능성이 있습니다.
특히 wrist camera는 케이블 움직임이 있으니 reconnect/health degrade 정책이 필요합니다.

초기에는 reconnect를 구현하지 않아도 되지만, 그러면 config를 이렇게 두는 게 정직합니다.

reconnect:
  enabled: false

또는 start 시점에:

reconnect.enabled=true is not implemented

warning을 띄우는 게 좋습니다.

P1-4. RealSense color+depth frameset 처리에서 timestamp가 분리됨

현재 rs2::frameset이 오면 color와 depth를 각각 on_video_frame()으로 보냅니다.

if (color) on_video_frame(color, "color", cfg_.color);
if (depth) on_video_frame(depth, "depth", cfg_.depth);

이때 on_video_frame() 안에서 host_t = now_ns()를 각각 찍습니다.
그래서 같은 frameset에서 나온 color/depth라도 host timestamp가 조금 달라집니다.

RGB-only면 큰 문제 없습니다.
하지만 RGB-D를 켜서 policy input으로 쓸 계획이면 frameset 단위 metadata가 필요합니다.

권장:

frameset callback에서 host_t를 한 번만 찍음
color/depth에 같은 frameset_host_time_ns 부여
frameset_number 또는 composite_seq 저장
P1-5. RealSense frame stride 처리

shared memory slot 크기는 config 기준으로:

width * height * bytes_per_pixel

입니다.

그런데 RealSense에서 넘어오는 실제 frame size는:

height * stride_in_bytes

입니다.

대부분 RGB8에서는 stride = width * 3이라 괜찮지만, 모델/format에 따라 stride padding이 있을 수 있습니다.

현재는:

if (size_bytes > expected) throw

이므로 stride가 config 예상보다 크면 frame handling error가 발생합니다.

권장:

slot size를 max_stride_bytes * height 기준으로 잡기
또는 start 시 실제 profile stride를 확인한 뒤 layout 생성

초기 640x480 RGB8에서는 대부분 괜찮지만, robustness 측면에서 보강하면 좋습니다.

P1-6. ZMQ publish가 callback path에 있음

publish()는 ZMQ_DONTWAIT를 사용해서 오래 block되지는 않겠지만, 여전히 callback path에서 호출됩니다.

또한 ZMQ_SNDHWM = 4라서 subscriber가 느리면 metadata는 드롭될 수 있습니다.
이건 latest-frame policy에는 괜찮지만, 반드시 sequence gap으로 감지할 수 있어야 합니다.

권장:

bundle_seq gap 감지
publish_drop_count 또는 publish_error_count 노출
policy_runner는 latest만 사용

metadata loss 자체는 허용 가능합니다. shared memory에는 최신 frame이 있으니까요.
하지만 손실 여부는 health에 나와야 합니다.

4. Docker / 배포 관련
Dockerfile의 librealsense2-dev 설치는 환경 확인 필요

Dockerfile에는:

librealsense2-dev

가 들어 있습니다.

Ubuntu 기본 apt repo에서 이 패키지가 바로 설치되는 환경도 있고, 별도 Intel RealSense apt repo 설정이 필요한 환경도 있습니다. 실제 로봇 PC에서 Docker build가 되는지 확인해야 합니다.

현장 안정성을 위해서는 둘 중 하나가 좋습니다.

1. RealSense apt repo 설정을 Dockerfile에 명시
2. 회사 내부 base image에 librealsense2-dev를 미리 포함
Docker 실행 옵션은 방향이 맞음

아래 옵션은 적절합니다.

ipc: host
shm_size: "2gb"
network_mode: host
privileged: true
/dev/bus/usb mount
/run/udev mount

다만 privileged: true는 강하지만 편합니다.
초기 개발에서는 괜찮고, 나중에 배포할 때만 권한을 줄이면 됩니다.

5. policy_runner 관점에서 현재 구현의 의미

현재 tools/read_latest_bundle.py는 방향이 좋습니다.

ZMQ로 camera.bundle 구독
latest-frame-wins 방식으로 queue drain
shared memory에서 seqlock 확인 후 image read

다만 이 샘플은 bundle마다 shm을 open/mmap합니다.

with open(path, "r+b") as f:
    mm = mmap.mmap(...)

실제 policy_runner에서는 이렇게 하면 안 됩니다.

실제 구현에서는:

startup 시 shm open/mmap 1회
bundle metadata만 계속 수신
각 frame offset만 읽음

으로 가야 합니다.

6. 권장 수정 우선순위
Tier A — 실제 3대 카메라 운용 전 필수
1. FrameSynchronizer를 master-camera-driven 또는 frame_number-driven으로 수정
2. hardware sync mode가 실제 설정되지 않으면 hardware_synced=true 금지
3. SharedMemoryRingBuffer::write_frame thread safety 보강
4. RealSense callback path에서 ZMQ publish/synchronizer 분리
5. Recorder metadata JSONL multi-thread write 보호
Tier B — recording / 장시간 운용 전 필수
1. recording filename에 host_time_ns 또는 recording_seq 추가
2. reconnect.enabled 실제 구현 또는 false로 고정
3. frame drop / metadata drop / recorder drop을 health에 더 명확히 노출
4. RGB-D frameset 단위 timestamp 처리
5. stride/padding 대응
Tier C — 성능 최적화
1. RealSense frame → vector → shm 2-copy 구조 개선
2. camera callback에서 shm direct write
3. metadata queue lock-free/SPSC 적용
4. shm reader가 mmap 1회만 하도록 policy_runner library 제공
5. CPU affinity / thread priority 옵션 추가
7. 지금 코드 상태에서 실험 추천

현재 바로 할 수 있는 검증은 mock mode입니다.

./camera_server --config config/mock_triple_realsense.yaml --run-seconds 10
python3 tools/read_latest_bundle.py --metadata tcp://127.0.0.1:5600 --shm /camera_server_frames_test --once
python3 tools/print_camera_health.py --metadata tcp://127.0.0.1:5600 --once

다만 실제 3대 RealSense에서는 아래 테스트가 필요합니다.

Test 1:
  3 RGB cameras, 640x480@30, recording off, 10 min
  expected:
    frame_number_gap_drop_count = 0
    fps ≈ 30 per stream
    bundle rate ≈ 30, not 90
    max_time_diff_ms stable

Test 2:
  recording on, 10 min
  expected:
    frame_number_gap_drop_count = 0
    recorder_drop_count = 0
    metadata jsonl not corrupted

Test 3:
  policy_runner shm read loop
  expected:
    bundle_seq gap count measured
    shm seqlock failure rate = 0
    image read latency measured

Test 4:
  unplug one camera
  expected:
    health reports disconnected or stale
    server does not crash
    reconnect policy is clear

현재 구현에서는 Test 1에서 bundle rate가 90Hz 가까이 나올 가능성이 있습니다. 이걸 먼저 확인해보면 FrameSynchronizer 문제를 바로 볼 수 있을 겁니다.

8. 최종 판단

현재 camera_server는 좋은 1차 구현입니다.

특히:

shared memory ring
metadata protocol
mock camera
health monitor
Docker structure
policy_runner sample

이 부분은 방향이 맞습니다.

하지만 실제 목표인:

RealSense 3대
각각 30fps
sync 맞춘 image bundle
낮은 latency
frame loss 최소화
policy_runner 안정 입력

까지 가려면, 가장 먼저 고쳐야 할 것은 FrameSynchronizer입니다.

현재는 “latest cache를 bundle로 묶는 구조”라서, 진짜 synchronized 30fps bundle stream이라고 보기 어렵습니다.

제 추천 수정 방향은 명확합니다.

software sync:
  master-camera-driven nearest timestamp bundle

hardware sync:
  frame_number-driven bundle
  단, 실제 RealSense hardware sync option 설정 성공 시에만 hardware_synced=true

policy path:
  latest complete bundle only

recording path:
  all frames async write + drop detection

이 수정까지 들어가면 camera_server는 policy_runner와 실제로 붙일 수 있는 수준에 훨씬 가까워집니다.

카메라의 해상도를 Head 는 기본 1280 x 720 으로 하고, Hand 2개는 640 x 360 을 기본으로 하도록 수정해줘.
