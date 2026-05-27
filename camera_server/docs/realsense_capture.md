# RealSense Capture Requirements

## 1. Device mapping

Cameras must be identified by serial number, not by enumeration order.

Config example:

```yaml
cameras:
  head:
    serial: "1234567890"
    enabled: true
  left_wrist:
    serial: "2345678901"
    enabled: true
  right_wrist:
    serial: "3456789012"
    enabled: true
```

On startup:

1. enumerate RealSense devices
2. match required serials
3. fail clearly if required camera is missing
4. print mapping camera name → serial → USB port info if available

## 2. Stream profiles

Initial recommended profile:

```yaml
streams:
  color:
    enabled: true
    width: 640
    height: 480
    fps: 30
    format: rgb8
  depth:
    enabled: false
    width: 640
    height: 480
    fps: 30
    format: z16
```

For VLA RGB policy, start with:

```text
head:       color + optional depth
left_wrist: color only
right_wrist: color only
```

Turn on wrist depth only after bandwidth/drop tests pass.

## 3. Capture callback

Use RealSense callback or polling thread per camera. Callback is preferred for low latency.

Required frame handling:

```text
1. capture host timestamp immediately
2. read frame number
3. read RealSense timestamp
4. copy frame into shared memory slot
5. update frame metadata
6. push small metadata to synchronizer
```

Do not block in callback.

## 4. Frame format

Use uncompressed raw formats for policy path.

Recommended:

```text
color: rgb8 or bgr8
Depth: z16, uint16 millimeter or RealSense depth unit metadata
```

Avoid JPEG/PNG in policy path. Compression can be used only for optional monitoring/recording if it does not interfere with capture.

## 5. Frame drop detection

For each stream:

```cpp
if (current_frame_number != previous_frame_number + 1) {
    drop_count += current_frame_number - previous_frame_number - 1;
}
```

Also track internal drops:

- shared memory slot overwrite before metadata publish
- synchronizer queue overflow
- recorder queue overflow
- writer failure

## 6. Reconnect policy

Config example:

```yaml
reconnect:
  enabled: true
  max_attempts: 5
  retry_interval_ms: 1000
```

If camera disconnects:

1. publish health event
2. stop that device pipeline
3. attempt reconnect if enabled
4. mark bundles incomplete while camera unavailable
5. do not crash the entire server unless configured as required-fail-fast

For real robot experiments, fail-fast may be safer during early development.

## 7. USB diagnostics

Provide a tool or startup printout recommending:

```bash
lsusb -t
rs-enumerate-devices
```

Log warnings if multiple cameras appear to share a single USB controller and frame drops occur.

## 구현 파일

RealSense serial discovery/startup fail-fast와 stream callback은 `src/camera/realsense_device.cpp`와 `src/camera/camera_manager.cpp`에 구현되어 있다. Camera identification은 serial 기반이며 enumeration order에 의존하지 않는다. Frame number gap drop detection은 `CameraManager::handle_frame`에서 stream별 counter로 누적된다.
