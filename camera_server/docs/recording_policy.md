# Recording Policy

## 1. Separate policy path and recording path

`policy_runner` needs lowest latency. Dataset recording needs minimal frame loss.

Therefore:

```text
policy path:
  latest frame bundle through shared memory
  old frames can be overwritten

recording path:
  async writer queue
  attempts to save all configured frames
  logs drop counters
```

The camera callback must not wait for the recorder.

## 2. Episode directory layout

Recommended initial layout:

```text
episodes/
  episode_000001/
    episode_metadata.json
    camera_metadata.jsonl
    camera_health.jsonl
    robot_state.jsonl, optional copied or merged later
    head/
      color/
        000000.rgb
        000001.rgb
      depth/
        000000.z16
    left_wrist/
      color/
        000000.rgb
    right_wrist/
      color/
        000000.rgb
```

Raw binary files are simple and fast. Metadata stores width/height/format/stride.

Future storage options:

- MCAP
- Zarr
- HDF5
- rosbag2

Do not choose a complex format until raw recording performance is verified.

## 3. Metadata JSONL

Each recorded frame should have one metadata line:

```json
{
  "camera_name": "head",
  "stream": "color",
  "frame_number": 123,
  "host_arrival_time_ns": 1234567890,
  "sensor_timestamp_ns": 1234567800,
  "file": "head/color/000123.rgb",
  "width": 640,
  "height": 480,
  "format": "rgb8",
  "size_bytes": 921600,
  "bundle_seq": 10001
}
```

## 4. Writer queue

Use bounded queues.

Config:

```yaml
recording:
  enabled: true
  output_dir: /data/episodes
  queue_capacity_frames: 300
  drop_policy: drop_oldest
  writer_threads: 4
```

If queue is full:

- do not block capture callback
- apply configured drop policy
- increment `dropped_by_recorder_queue`
- publish health warning

## 5. Disk requirements

Rough bandwidth:

```text
640x480 RGB x 3 x 30fps ≈ 83 MB/s
640x480 RGB+Depth x 3 x 30fps ≈ 138 MB/s
default mixed RGB (head 1280x720 + two wrist 640x360) x 30fps ≈ 124 MB/s
1280x720 RGB x 3 x 30fps ≈ 249 MB/s
```

Use NVMe SSD for lossless raw recording.

## 6. Recording start/stop

Initial implementation can start recording on process start.

Future control channel:

```text
camera.control.start_recording
camera.control.stop_recording
camera.control.mark_episode_boundary
```

For VLA data collection, episode boundaries should align with policy/robot logs.

## 7. Recorder health

Publish:

- queue depth
- dropped_by_queue
- write latency mean/max
- bytes written per second
- disk free space
- current episode path

If disk free space is below threshold, stop recording and publish error.

## 구현 파일

Recording skeleton은 `include/camera_server/recording/recorder.hpp`와 `src/recording/recorder.cpp`에 구현되어 있다. Callback은 recorder를 기다리지 않고 bounded queue에 enqueue만 시도한다. Queue overflow는 drop policy에 따라 처리되고 health snapshot에 dropped count/queue depth로 노출된다.
