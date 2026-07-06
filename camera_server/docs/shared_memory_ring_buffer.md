# Shared Memory Ring Buffer

## 1. Goal

Image payload should not be sent through TCP/UDP/ZMQ/gRPC as full frame blobs for policy input.

Instead:

```text
image bytes       → POSIX shared memory ring buffer
small metadata    → ZeroMQ PUB/SUB
```

This avoids repeated serialization and large memory copies.

## 2. Shared memory object

Recommended POSIX shared memory name:

```text
/camera_server_frames
```

In Docker with `ipc: host`, this appears under `/dev/shm`.

Recommended metadata/control shared memory name, optional:

```text
/camera_server_control
```

## 3. Slot layout

Use fixed-size slots per stream. Avoid variable-size allocations at runtime.

Example for RGB 640x480:

```text
slot_bytes = align64(width * height * channels)
```

Example for depth 640x480 uint16:

```text
slot_bytes = align64(width * height * 2)
```

Recommended structure:

```cpp
struct ShmHeader {
    uint32_t magic;
    uint32_t version;
    uint32_t camera_count;
    uint32_t stream_count;
    uint64_t total_size_bytes;
    uint64_t created_time_ns;
};

struct StreamRingHeader {
    char camera_name[32];
    char stream_name[16];  // color, depth
    char format[16];       // rgb8, bgr8, z16
    uint32_t width;
    uint32_t height;
    uint32_t channels;
    uint32_t bytes_per_pixel;
    uint32_t slot_count;
    uint64_t slot_bytes;
    uint64_t slots_offset;
};

struct FrameSlotHeader {
    std::atomic<uint64_t> seq_begin;
    std::atomic<uint64_t> seq_end;
    uint64_t frame_number;
    uint64_t host_arrival_time_ns;
    uint64_t sensor_timestamp_ns;
    uint64_t write_time_ns;
    uint32_t width;
    uint32_t height;
    uint32_t stride_bytes;
    uint32_t size_bytes;
    uint32_t valid;
    uint32_t reserved;
    // followed by image bytes
};
```

## 4. Seqlock pattern

To avoid reading a half-written image, use seqlock-like slot protocol.

Writer:

```cpp
slot.seq_begin.fetch_add(1);   // becomes odd, write in progress
copy_image_bytes();
write_metadata_fields();
slot.valid = 1;
slot.seq_end.store(slot.seq_begin.load() + 1);  // even, complete
```

Reader:

```cpp
while (true) {
    uint64_t a = slot.seq_begin.load(std::memory_order_acquire);
    if (a % 2 == 1) continue;  // writer in progress

    copy_or_wrap_image_view();

    uint64_t b = slot.seq_end.load(std::memory_order_acquire);
    if (a + 1 == b && b % 2 == 0) {
        break;  // consistent frame
    }
}
```

Alternative: store `seq_begin == seq_end` after write. The exact convention is less important than having a documented, tested protocol.

## 5. Ring policy

For policy input:

```text
latest-frame-wins
old frames may be overwritten
ring size 2~4 per stream
```

For recording:

```text
separate bounded queue
attempt to write all frames
track dropped frames and queue overflow
```

Do not make policy wait for recorder.

## 6. Recommended initial shared memory sizing

For the legacy RGB-only profile, head 1280x720 plus two wrist 640x360, ring size 4:

```text
2.76 MB/head frame × 4 + 0.69 MB/wrist frame × 4 × 2 ≈ 16.6 MB
```

RGB + depth, 640x480, ring size 4:

```text
RGB   0.92 MB/frame
Depth 0.61 MB/frame
Total 1.53 MB/frame × 4 × 3 ≈ 18.4 MB
```

For the current default `d435_head_1280x720.yaml` profile, budget for head RGB,
head IR stereo, and two wrist RGB-D streams; the tracked config uses 1536 MB
shared memory to leave room for alignment, headers, and stereo-worker consumers.

Use generous allocation to handle alignment, headers, and future modes.

Recommended Docker `shm_size`:

```text
RGB only default mixed profile: 512 MB
RGB + depth 640x480x3:         1 GB
1280x720 RGB/depth:            2 GB
```

## 7. Shared memory lifecycle

`camera_server` should create and initialize the shared memory segment.

On startup:

1. unlink old stale segment if allowed by config
2. create shm object
3. `ftruncate` to configured size
4. `mmap`
5. write headers
6. start cameras

`policy_runner` should only open existing segment.

If `policy_runner` cannot open or validate the magic/version, it should fail clearly.

## 8. Versioning

Include:

```text
magic = 0x43534D52  // example: CSMR
version = 1
```

Any breaking layout change must increment version.

## 구현 파일

POSIX shared memory ring은 `include/camera_server/shm/shm_layout.hpp`와 `src/shm/shared_memory_ring.cpp`에 구현되어 있다. Server는 `shm_open`/`ftruncate`/`mmap`으로 segment를 생성하고 stream descriptor 및 fixed-size slots를 초기화한다. Writer/reader는 even sequence seqlock protocol로 torn read를 방지한다.
