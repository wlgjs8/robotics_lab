#pragma once

#include <cstdint>
#include <map>
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
  // Fast-FoundationStereo 입력용. 보통 format "y8".
  CameraStreamConfig ir_left;
  CameraStreamConfig ir_right;
};

struct FrameMeta {
  std::string camera_name;
  std::string serial;
  std::string stream;
  uint64_t frame_number{0};
  uint64_t host_arrival_time_ns{0};
  uint64_t sensor_timestamp_ns{0};
  double realsense_timestamp_ms{0.0};
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
  uint64_t last_frame_number{0};
  uint64_t last_frame_time_ns{0};
  double fps_estimate{0.0};
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
  std::map<std::string, StreamStats> stream_stats;
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
