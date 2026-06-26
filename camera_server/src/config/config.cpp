#include "camera_server/config/config.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <stdexcept>

namespace camera_server {
namespace {

template <typename T>
T node_as(const YAML::Node& node, const T& fallback) {
  return node ? node.as<T>() : fallback;
}

CameraStreamConfig parse_stream(const YAML::Node& node, const CameraStreamConfig& fallback) {
  CameraStreamConfig out = fallback;
  if (!node) return out;
  out.enabled = node_as(node["enabled"], out.enabled);
  out.width = node_as(node["width"], out.width);
  out.height = node_as(node["height"], out.height);
  out.fps = node_as(node["fps"], out.fps);
  out.format = node_as(node["format"], out.format);
  return out;
}

uint64_t aligned_payload_bytes(const CameraStreamConfig& stream) {
  uint64_t bpp = 0;
  if (stream.format == "rgb8" || stream.format == "bgr8") bpp = 3;
  else if (stream.format == "z16" || stream.format == "y16") bpp = 2;
  else if (stream.format == "y8" || stream.format == "mono8") bpp = 1;
  else throw std::runtime_error("unsupported stream format: " + stream.format);
  const uint64_t row_bytes = static_cast<uint64_t>(stream.width) * bpp;
  const uint64_t aligned_row = (row_bytes + 63u) & ~63ull;
  return aligned_row * static_cast<uint64_t>(stream.height);
}

std::string uppercase_ascii(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::toupper(c));
  });
  return value;
}

bool starts_with(const std::string& value, const std::string& prefix) {
  return value.rfind(prefix, 0) == 0;
}

bool is_disallowed_serial_placeholder(const std::string& serial) {
  const std::string upper = uppercase_ascii(serial);
  return starts_with(upper, "REPLACE_") || upper == "TODO" || upper == "CHANGEME" || upper == "UNKNOWN";
}

}  // namespace

uint32_t bytes_per_pixel_for_format(const std::string& format) {
  if (format == "rgb8" || format == "bgr8") return 3;
  if (format == "z16" || format == "y16") return 2;
  if (format == "y8" || format == "mono8") return 1;
  throw std::runtime_error("unsupported stream format: " + format);
}

uint32_t channels_for_format(const std::string& format) {
  if (format == "rgb8" || format == "bgr8") return 3;
  if (format == "z16" || format == "y16" || format == "y8" || format == "mono8") return 1;
  throw std::runtime_error("unsupported stream format: " + format);
}

uint64_t bytes_per_frame(const CameraStreamConfig& stream) {
  return static_cast<uint64_t>(stream.width) * static_cast<uint64_t>(stream.height) *
         static_cast<uint64_t>(bytes_per_pixel_for_format(stream.format));
}

AppConfig load_config(const std::string& path) {
  YAML::Node root = YAML::LoadFile(path);
  AppConfig cfg;

  if (auto n = root["server"]) {
    cfg.server.name = node_as(n["name"], cfg.server.name);
    cfg.server.mode = node_as(n["mode"], cfg.server.mode);
    cfg.server.clock = parse_clock_kind(node_as(n["clock"], std::string("monotonic_raw")));
    cfg.server.simulate_cameras = node_as(n["simulate_cameras"], cfg.server.simulate_cameras);
  }

  if (auto n = root["shared_memory"]) {
    cfg.shared_memory.name = node_as(n["name"], cfg.shared_memory.name);
    cfg.shared_memory.size_mb = node_as(n["size_mb"], cfg.shared_memory.size_mb);
    cfg.shared_memory.ring_slots = node_as(n["ring_slots"], cfg.shared_memory.ring_slots);
    cfg.shared_memory.unlink_on_start = node_as(n["unlink_on_start"], cfg.shared_memory.unlink_on_start);
  }

  if (auto n = root["metadata"]) {
    cfg.metadata.transport = node_as(n["transport"], cfg.metadata.transport);
    cfg.metadata.pub_bind = node_as(n["pub_bind"], cfg.metadata.pub_bind);
    cfg.metadata.health_topic = node_as(n["health_topic"], cfg.metadata.health_topic);
    cfg.metadata.bundle_topic = node_as(n["bundle_topic"], cfg.metadata.bundle_topic);
  }

  if (auto n = root["sync"]) {
    cfg.sync.mode = node_as(n["mode"], cfg.sync.mode);
    cfg.sync.master_camera = node_as(n["master_camera"], cfg.sync.master_camera);
    cfg.sync.bundle_policy = node_as(n["bundle_policy"], cfg.sync.bundle_policy);
    cfg.sync.max_bundle_time_diff_ms = node_as(n["max_bundle_time_diff_ms"], cfg.sync.max_bundle_time_diff_ms);
    cfg.sync.publish_incomplete_bundles = node_as(n["publish_incomplete_bundles"], cfg.sync.publish_incomplete_bundles);
  }

  if (auto cams = root["cameras"]) {
    for (const auto& it : cams) {
      CameraConfig cam;
      cam.name = it.first.as<std::string>();
      const auto n = it.second;
      cam.backend = node_as(n["backend"], std::string("realsense"));
      cam.serial = node_as(n["serial"], std::string());
      cam.device = node_as(n["device"], std::string());
      cam.required = node_as(n["required"], true);
      cam.color.enabled = true;
      cam.color.format = "rgb8";
      cam.color = parse_stream(n["streams"]["color"], cam.color);
      cam.depth.enabled = false;
      cam.depth.format = "z16";
      cam.depth = parse_stream(n["streams"]["depth"], cam.depth);
      cam.ir_left.enabled = false;
      cam.ir_left.format = "y8";
      cam.ir_left = parse_stream(n["streams"]["ir_left"], cam.ir_left);
      cam.ir_right.enabled = false;
      cam.ir_right.format = "y8";
      cam.ir_right = parse_stream(n["streams"]["ir_right"], cam.ir_right);
      cfg.cameras.push_back(cam);
    }
  }

  if (auto n = root["recording"]) {
    cfg.recording.enabled = node_as(n["enabled"], cfg.recording.enabled);
    cfg.recording.output_dir = node_as(n["output_dir"], cfg.recording.output_dir);
    cfg.recording.queue_capacity_frames = node_as(n["queue_capacity_frames"], cfg.recording.queue_capacity_frames);
    cfg.recording.writer_threads = node_as(n["writer_threads"], cfg.recording.writer_threads);
    cfg.recording.drop_policy = node_as(n["drop_policy"], cfg.recording.drop_policy);
    cfg.recording.raw_format = node_as(n["raw_format"], cfg.recording.raw_format);
  }

  if (auto n = root["health"]) {
    cfg.health.publish_rate_hz = node_as(n["publish_rate_hz"], cfg.health.publish_rate_hz);
    cfg.health.warn_if_frame_age_ms_gt = node_as(n["warn_if_frame_age_ms_gt"], cfg.health.warn_if_frame_age_ms_gt);
    cfg.health.warn_if_drop_count_increases = node_as(n["warn_if_drop_count_increases"], cfg.health.warn_if_drop_count_increases);
    cfg.health.warn_if_bundle_skew_ms_gt = node_as(n["warn_if_bundle_skew_ms_gt"], cfg.health.warn_if_bundle_skew_ms_gt);
  }

  if (auto n = root["reconnect"]) {
    cfg.reconnect.enabled = node_as(n["enabled"], cfg.reconnect.enabled);
    cfg.reconnect.max_attempts = node_as(n["max_attempts"], cfg.reconnect.max_attempts);
    cfg.reconnect.retry_interval_ms = node_as(n["retry_interval_ms"], cfg.reconnect.retry_interval_ms);
  }

  validate_config(cfg);
  return cfg;
}

std::vector<std::string> required_stream_keys(const AppConfig& cfg) {
  std::vector<std::string> keys;
  for (const auto& cam : cfg.cameras) {
    for (const auto& [name, scfg] : enabled_streams(cam)) {
      (void)scfg;
      keys.push_back(stream_key(cam.name, name));
    }
  }
  return keys;
}

uint64_t required_shared_memory_bytes(const AppConfig& cfg) {
  // Header + stream descriptors + slots. Keep this conservative and aligned.
  uint64_t total = 4096;
  for (const auto& cam : cfg.cameras) {
    for (const auto& [name, scfg] : enabled_streams(cam)) {
      (void)name;
      total += cfg.shared_memory.ring_slots * (256 + aligned_payload_bytes(scfg));
    }
  }
  return ((total + 4095) / 4096) * 4096;
}

void validate_config(const AppConfig& cfg) {
  if (cfg.server.name != "camera_server") throw std::runtime_error("server.name must be camera_server");
  if (cfg.cameras.empty()) throw std::runtime_error("at least one camera must be configured");
  if (cfg.shared_memory.name.empty() || cfg.shared_memory.name.front() != '/') {
    throw std::runtime_error("shared_memory.name must be a POSIX shm name starting with '/'");
  }
  if (cfg.shared_memory.ring_slots < 2) throw std::runtime_error("shared_memory.ring_slots must be >= 2");
  if (cfg.sync.mode != "software" && cfg.sync.mode != "hardware") {
    throw std::runtime_error("sync.mode must be software or hardware");
  }
  if (cfg.sync.mode == "software" && cfg.sync.bundle_policy != "nearest_timestamp") {
    throw std::runtime_error("sync.mode=software requires sync.bundle_policy=nearest_timestamp");
  }
  if (cfg.sync.mode == "hardware" && cfg.sync.bundle_policy != "frame_number") {
    throw std::runtime_error("sync.mode=hardware requires sync.bundle_policy=frame_number");
  }
  if (cfg.server.simulate_cameras && cfg.sync.mode == "hardware") {
    throw std::runtime_error("sync.mode=hardware requires real RealSense devices; mock cameras cannot verify hardware sync");
  }
  if (cfg.sync.bundle_policy != "nearest_timestamp" && cfg.sync.bundle_policy != "frame_number") {
    throw std::runtime_error("sync.bundle_policy must be nearest_timestamp or frame_number");
  }
  if (cfg.reconnect.enabled) {
    throw std::runtime_error("reconnect.enabled=true is not implemented yet");
  }
  for (const auto& cam : cfg.cameras) {
    if (cam.name.empty()) throw std::runtime_error("camera name cannot be empty");
    if (cam.backend != "realsense" && cam.backend != "uvc") {
      throw std::runtime_error("camera " + cam.name + " has unsupported backend: " + cam.backend);
    }
    if (cam.backend == "uvc") {
      // UVC fisheye is selected by V4L2 device path, not serial. The DECXIN/Sunplus
      // wrist cameras share a fixed serial ("01.00.00") so serial-based discovery is
      // impossible; bypass the serial placeholder/MOCK checks and require a device.
      if (cam.required && cam.device.empty()) {
        throw std::runtime_error("required uvc camera " + cam.name + " has empty device");
      }
      if (cam.depth.enabled) {
        throw std::runtime_error("uvc camera " + cam.name + " does not support a depth stream");
      }
      if (cam.ir_left.enabled || cam.ir_right.enabled) {
        throw std::runtime_error("uvc camera " + cam.name + " does not support infrared streams");
      }
    } else {
      if (cam.required && cam.serial.empty()) throw std::runtime_error("required camera " + cam.name + " has empty serial");
      if (is_disallowed_serial_placeholder(cam.serial)) {
        throw std::runtime_error("camera " + cam.name + " has invalid serial placeholder: " + cam.serial);
      }
      if (!cfg.server.simulate_cameras && starts_with(uppercase_ascii(cam.serial), "MOCK_")) {
        throw std::runtime_error("camera " + cam.name + " uses MOCK_* serial but server.simulate_cameras=false");
      }
    }
    for (const auto& [name, scfg] : enabled_streams(cam)) {
      if (scfg.width <= 0 || scfg.height <= 0) throw std::runtime_error(cam.name + "." + name + " dimensions must be positive");
      if (scfg.fps <= 0) throw std::runtime_error(cam.name + "." + name + " fps must be positive");
      (void)bytes_per_pixel_for_format(scfg.format);
    }
  }
  const uint64_t configured = cfg.shared_memory.size_mb * 1024ull * 1024ull;
  const uint64_t required = required_shared_memory_bytes(cfg);
  if (configured < required) {
    throw std::runtime_error("shared memory size insufficient: configured=" + std::to_string(configured) +
                             " required=" + std::to_string(required));
  }
  if (cfg.metadata.pub_bind.find("0.0.0.0") != std::string::npos) {
    throw std::runtime_error("metadata.pub_bind must not default to 0.0.0.0");
  }
}

}  // namespace camera_server
