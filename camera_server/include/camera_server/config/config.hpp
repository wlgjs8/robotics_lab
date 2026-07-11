#pragma once

#include "camera_server/core/clock.hpp"
#include "camera_server/core/types.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace camera_server {

struct SharedMemoryConfig {
  std::string name{"/camera_server_frames"};
  uint64_t size_mb{1024};
  uint32_t ring_slots{4};
  bool unlink_on_start{true};
};

struct MetadataConfig {
  std::string transport{"zmq_pub"};
  std::string pub_bind{"tcp://127.0.0.1:5600"};
  std::string health_topic{"camera.health"};
  std::string bundle_topic{"camera.bundle"};
};

struct SyncConfig {
  std::string mode{"software"};
  std::string master_camera{"head"};
  std::string bundle_policy{"nearest_timestamp"};
  double max_bundle_time_diff_ms{10.0};
  bool publish_incomplete_bundles{false};
};

struct BundleGroupConfig {
  std::string name{"default"};
  std::string topic{"camera.bundle"};
  std::string master_stream;
  std::vector<std::string> required_streams;
  double max_bundle_time_diff_ms{10.0};
  bool publish_incomplete_bundles{false};
};

struct RecordingConfig {
  bool enabled{false};
  std::string output_dir{"/data/episodes"};
  uint32_t queue_capacity_frames{300};
  uint32_t writer_threads{4};
  std::string drop_policy{"drop_oldest"};
  bool raw_format{true};
};

struct HealthConfig {
  double publish_rate_hz{1.0};
  double fps_window_sec{5.0};
  double warn_if_fps_below{29.0};
  double warn_if_frame_age_ms_gt{100.0};
  bool warn_if_drop_count_increases{true};
  double warn_if_bundle_skew_ms_gt{10.0};
};

struct ReconnectConfig {
  bool enabled{false};
  // Number of retries after a failed/stalled connection. Zero means retry
  // indefinitely; the delay remains bounded by retry_interval_ms.
  uint32_t max_attempts{0};
  uint32_t retry_interval_ms{1000};
  uint32_t frame_timeout_ms{2000};
};

struct ServerConfig {
  std::string name{"camera_server"};
  std::string mode{"capture_only"};
  ClockKind clock{ClockKind::MonotonicRaw};
  bool simulate_cameras{false};
};

struct AppConfig {
  ServerConfig server;
  SharedMemoryConfig shared_memory;
  MetadataConfig metadata;
  SyncConfig sync;
  std::vector<BundleGroupConfig> bundle_groups;
  std::vector<CameraConfig> cameras;
  RecordingConfig recording;
  HealthConfig health;
  ReconnectConfig reconnect;
};

AppConfig load_config(const std::string& path);
void validate_config(const AppConfig& cfg);
uint64_t bytes_per_frame(const CameraStreamConfig& stream);
uint32_t bytes_per_pixel_for_format(const std::string& format);
uint32_t channels_for_format(const std::string& format);
uint64_t required_shared_memory_bytes(const AppConfig& cfg);
std::vector<std::string> required_stream_keys(const AppConfig& cfg);
std::vector<BundleGroupConfig> effective_bundle_groups(const AppConfig& cfg);

}  // namespace camera_server
