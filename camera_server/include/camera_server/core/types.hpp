#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace camera_server {

struct CameraStreamConfig {
  bool enabled{false};
  int width{640};
  int height{480};
  int fps{30};
  std::string format{"rgb8"};
};

// 디바이스(센서) 단위 제어: D435 IR projector(emitter)와 스테레오 모듈 노출/게인을
// config로 고정한다(드라이버 자동값이 리그마다 달라지는 것을 막기 위함).
// 모든 수치는 "-1 = 미설정(드라이버/JSON 값 유지)" 의미. (auto_exposure: -1 유지/0 수동/1 자동)
struct CameraControlsConfig {
  int emitter_enabled{-1};       // RS2_OPTION_EMITTER_ENABLED (0=off,1=on)
  float laser_power{-1.0f};      // RS2_OPTION_LASER_POWER (mW; emitter on일 때만 의미)
  int auto_exposure{-1};         // RS2_OPTION_ENABLE_AUTO_EXPOSURE (0/1)
  int ir_exposure_us{-1};        // RS2_OPTION_EXPOSURE (us; auto_exposure=0일 때 적용)
  int ir_gain{-1};               // RS2_OPTION_GAIN
};

struct CameraConfig {
  std::string name;
  // Capture backend: "realsense" (librealsense, depth-capable) or "uvc"
  // (generic V4L2 UVC camera via OpenCV, MJPG transport, color-only — used for
  // the DECXIN/Sunplus wrist fisheye cameras).
  std::string backend{"realsense"};
  std::string serial;
  // UVC only: V4L2 device. Accepts a /dev/videoN node, an integer index, or a
  // /dev/v4l/by-path/... symlink (preferred — stable across reboots). Ignored by
  // the realsense backend (which selects devices by serial).
  std::string device;
  bool required{true};
  CameraStreamConfig color;
  CameraStreamConfig depth;
  // RealSense 스테레오 IR 페어 (D435f 등). ir_left=infrared index 1, ir_right=index 2.
  // 보통 format "y8". 기본 비활성 — IR 캡처가 필요한 진단 리그에서만 켠다.
  CameraStreamConfig ir_left;
  CameraStreamConfig ir_right;
  // 센서 단위 제어(emitter/노출/게인). realsense 백엔드만 사용.
  CameraControlsConfig controls;
};

struct FrameMeta {
  std::string camera_name;
  std::string serial;
  std::string stream;
  uint64_t frame_number{0};
  uint64_t host_arrival_time_ns{0};
  uint64_t sensor_timestamp_ns{0};
  double realsense_timestamp_ms{0.0};
  // Optional per-frame RealSense metadata. These values describe the image
  // that was actually captured; they stay absent for mock/UVC devices or when
  // the device/backend does not expose the requested metadata attribute.
  std::optional<double> actual_exposure_us;
  std::optional<double> gain_level;
  std::optional<bool> auto_exposure;
  int width{0};
  int height{0};
  int stride_bytes{0};
  std::string format;
  std::string shm_name;
  std::string ring_name;
  uint32_t slot_index{0};
  uint64_t shm_offset{0};
  uint64_t size_bytes{0};
  uint64_t seq{0};
  bool valid{false};
};

struct FrameBundleMeta {
  std::string group_name{"default"};
  uint64_t bundle_seq{0};
  uint64_t bundle_time_ns{0};
  bool hardware_synced{false};
  std::string sync_policy{"nearest_timestamp"};
  double max_time_diff_ms{0.0};
  bool complete{false};
  std::map<std::string, FrameMeta> frames;
  std::map<std::string, uint64_t> drop_counters;
};

struct StreamStats {
  uint64_t frame_count{0};
  uint64_t frame_number_gap_drop_count{0};
  uint64_t shared_memory_write_count{0};
  uint64_t shared_memory_write_errors{0};
  uint64_t internal_queue_drop_count{0};
  uint64_t recorder_drop_count{0};
  uint64_t frame_number_gap_drop_delta{0};
  uint64_t internal_queue_drop_delta{0};
  uint64_t recorder_drop_delta{0};
  uint64_t shared_memory_write_error_delta{0};
  uint64_t last_frame_number{0};
  uint64_t first_frame_time_ns{0};
  uint64_t last_frame_time_ns{0};
  double fps_estimate{0.0};
  double fps_window_hz{0.0};
};

struct CameraCaptureStats {
  uint64_t queue_drop_count{0};
  uint64_t queue_drop_delta{0};
  uint64_t queue_depth{0};
  double callback_enqueue_us_p95{0.0};
  double callback_enqueue_us_max{0.0};
  double queue_wait_us_p95{0.0};
  double queue_wait_us_max{0.0};
  double frame_process_us_p95{0.0};
  double frame_process_us_max{0.0};
};

struct CameraReconnectStats {
  uint64_t attempt_count{0};
  uint64_t success_count{0};
  uint64_t disconnect_count{0};
  uint32_t consecutive_failures{0};
  bool exhausted{false};
  uint64_t last_disconnect_time_ns{0};
  uint64_t last_reconnect_time_ns{0};
  std::string last_error;
};

struct BundleStats {
  std::string topic;
  uint64_t bundle_seq{0};
  uint64_t complete_bundle_count{0};
  uint64_t incomplete_retry_count{0};
  uint64_t dropped_master_count{0};
  double publish_rate_hz{0.0};
  double last_skew_ms{0.0};
  double skew_p50_ms{0.0};
  double skew_p95_ms{0.0};
  double skew_max_ms{0.0};
};

struct HealthSnapshot {
  uint64_t host_time_ns{0};
  double uptime_sec{0.0};
  std::string mode{"capture_only"};
  std::string status{"ok"};
  std::vector<std::string> status_reasons;
  std::map<std::string, std::string> stream_status;
  std::map<std::string, bool> camera_connected;
  std::map<std::string, std::string> camera_serial;
  std::string realsense_sdk_version;
  std::string realsense_backend;
  std::map<std::string, std::string> camera_firmware_version;
  std::map<std::string, std::string> camera_recommended_firmware_version;
  std::map<std::string, std::string> camera_physical_port;
  std::map<std::string, std::string> camera_product_id;
  std::map<std::string, std::string> camera_usb_type;
  std::map<std::string, StreamStats> stream_stats;
  std::map<std::string, CameraCaptureStats> camera_capture_stats;
  std::map<std::string, CameraReconnectStats> camera_reconnect_stats;
  std::map<std::string, BundleStats> bundle_groups;
  uint64_t bundle_seq{0};
  uint64_t complete_bundle_count{0};
  uint64_t incomplete_bundle_count{0};
  double max_time_diff_ms{0.0};
  std::string shm_name;
  uint64_t shm_size_bytes{0};
  uint64_t metadata_publish_count{0};
  uint64_t metadata_publish_errors{0};
  bool recorder_enabled{false};
  uint64_t recorder_queue_depth{0};
  uint64_t recorder_dropped_by_queue{0};
};

inline std::string stream_key(const std::string& camera, const std::string& stream) {
  return camera + "." + stream;
}

// 카메라의 활성 스트림 (이름, 설정) 목록. shm/health/stats/검증 등 열거에 공통 사용한다.
// (realsense enable / 프레임 라우팅은 스트림 타입이 달라 별도 처리.)
inline std::vector<std::pair<std::string, CameraStreamConfig>> enabled_streams(const CameraConfig& cam) {
  std::vector<std::pair<std::string, CameraStreamConfig>> out;
  if (cam.color.enabled) out.emplace_back("color", cam.color);
  if (cam.depth.enabled) out.emplace_back("depth", cam.depth);
  if (cam.ir_left.enabled) out.emplace_back("ir_left", cam.ir_left);
  if (cam.ir_right.enabled) out.emplace_back("ir_right", cam.ir_right);
  return out;
}

}  // namespace camera_server
