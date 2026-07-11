#include "camera_server/publish/metadata_publisher.hpp"

#include <iomanip>
#include <iostream>
#include <sstream>

#if CAMERA_SERVER_HAVE_ZMQ
#include <zmq.h>
#endif

namespace camera_server {
namespace {

std::string esc(const std::string& s) {
  std::ostringstream os;
  for (char c : s) {
    switch (c) {
      case '"': os << "\\\""; break;
      case '\\': os << "\\\\"; break;
      case '\n': os << "\\n"; break;
      case '\r': os << "\\r"; break;
      case '\t': os << "\\t"; break;
      default: os << c; break;
    }
  }
  return os.str();
}

void json_frame_fields(std::ostringstream& os, const FrameMeta& m, bool include_schema) {
  if (include_schema) os << "\"schema\":\"camera_server.frame.v1\",";
  os << "\"camera_name\":\"" << esc(m.camera_name) << "\",";
  os << "\"serial\":\"" << esc(m.serial) << "\",";
  os << "\"stream\":\"" << esc(m.stream) << "\",";
  os << "\"frame_number\":" << m.frame_number << ',';
  os << "\"host_arrival_time_ns\":" << m.host_arrival_time_ns << ',';
  os << "\"sensor_timestamp_ns\":" << m.sensor_timestamp_ns << ',';
  os << "\"realsense_timestamp_ms\":" << std::fixed << std::setprecision(3) << m.realsense_timestamp_ms << ',';
  os << "\"width\":" << m.width << ',';
  os << "\"height\":" << m.height << ',';
  os << "\"stride_bytes\":" << m.stride_bytes << ',';
  os << "\"format\":\"" << esc(m.format) << "\",";
  os << "\"shm_name\":\"" << esc(m.shm_name) << "\",";
  os << "\"ring_name\":\"" << esc(m.ring_name) << "\",";
  os << "\"slot_index\":" << m.slot_index << ',';
  os << "\"shm_offset\":" << m.shm_offset << ',';
  os << "\"size_bytes\":" << m.size_bytes << ',';
  os << "\"seq\":" << m.seq << ',';
  os << "\"valid\":" << (m.valid ? "true" : "false");
}

}  // namespace

std::string frame_to_json(const FrameMeta& meta) {
  std::ostringstream os;
  os << '{';
  json_frame_fields(os, meta, true);
  os << '}';
  return os.str();
}

std::string bundle_to_json(const FrameBundleMeta& b) {
  std::ostringstream os;
  os << "{\"schema\":\"camera_server.bundle.v1\",";
  os << "\"group_name\":\"" << esc(b.group_name) << "\",";
  os << "\"bundle_seq\":" << b.bundle_seq << ',';
  os << "\"bundle_time_ns\":" << b.bundle_time_ns << ',';
  os << "\"hardware_synced\":" << (b.hardware_synced ? "true" : "false") << ',';
  os << "\"sync_policy\":\"" << esc(b.sync_policy) << "\",";
  os << "\"max_time_diff_ms\":" << std::fixed << std::setprecision(3) << b.max_time_diff_ms << ',';
  os << "\"complete\":" << (b.complete ? "true" : "false") << ',';
  os << "\"frames\":{";
  bool first = true;
  for (const auto& [key, frame] : b.frames) {
    if (!first) os << ',';
    first = false;
    os << '\"' << esc(key) << "\":{";
    json_frame_fields(os, frame, false);
    os << '}';
  }
  os << "},\"drop_counters\":{";
  first = true;
  for (const auto& [key, count] : b.drop_counters) {
    if (!first) os << ',';
    first = false;
    os << '\"' << esc(key) << "\":" << count;
  }
  os << "}}";
  return os.str();
}

std::string health_to_json(const HealthSnapshot& h) {
  std::ostringstream os;
  os << "{\"schema\":\"camera_server.health.v1\",";
  os << "\"host_time_ns\":" << h.host_time_ns << ',';
  os << "\"uptime_sec\":" << std::fixed << std::setprecision(3) << h.uptime_sec << ',';
  os << "\"mode\":\"" << esc(h.mode) << "\",";
  os << "\"status\":\"" << esc(h.status) << "\",";
  os << "\"status_reasons\":[";
  bool first_reason = true;
  for (const auto& reason : h.status_reasons) {
    if (!first_reason) os << ',';
    first_reason = false;
    os << '\"' << esc(reason) << '\"';
  }
  os << "],";
  os << "\"bundle\":{\"bundle_seq\":" << h.bundle_seq << ",\"complete_bundle_count\":" << h.complete_bundle_count
     << ",\"incomplete_bundle_count\":" << h.incomplete_bundle_count << ",\"max_time_diff_ms\":" << h.max_time_diff_ms << "},";
  os << "\"bundle_groups\":{";
  bool first_group = true;
  for (const auto& [name, b] : h.bundle_groups) {
    if (!first_group) os << ',';
    first_group = false;
    os << '\"' << esc(name) << "\":{\"topic\":\"" << esc(b.topic)
       << "\",\"bundle_seq\":" << b.bundle_seq
       << ",\"complete_bundle_count\":" << b.complete_bundle_count
       << ",\"incomplete_retry_count\":" << b.incomplete_retry_count
       << ",\"dropped_master_count\":" << b.dropped_master_count
       << ",\"publish_rate_hz\":" << b.publish_rate_hz
       << ",\"last_skew_ms\":" << b.last_skew_ms
       << ",\"skew_p50_ms\":" << b.skew_p50_ms
       << ",\"skew_p95_ms\":" << b.skew_p95_ms
       << ",\"skew_max_ms\":" << b.skew_max_ms << '}';
  }
  os << "},";
  os << "\"cameras\":{";
  bool first_cam = true;
  for (const auto& [cam, connected] : h.camera_connected) {
    if (!first_cam) os << ',';
    first_cam = false;
    os << '\"' << esc(cam) << "\":{\"serial\":\"" << esc(h.camera_serial.count(cam) ? h.camera_serial.at(cam) : "")
       << "\",\"connected\":" << (connected ? "true" : "false");
    auto capture_it = h.camera_capture_stats.find(cam);
    if (capture_it != h.camera_capture_stats.end()) {
      const auto& c = capture_it->second;
      os << ",\"capture\":{\"queue_depth\":" << c.queue_depth
         << ",\"queue_drop_count\":" << c.queue_drop_count
         << ",\"queue_drop_delta\":" << c.queue_drop_delta
         << ",\"callback_enqueue_us_p95\":" << c.callback_enqueue_us_p95
         << ",\"callback_enqueue_us_max\":" << c.callback_enqueue_us_max
         << ",\"queue_wait_us_p95\":" << c.queue_wait_us_p95
         << ",\"queue_wait_us_max\":" << c.queue_wait_us_max
         << ",\"frame_process_us_p95\":" << c.frame_process_us_p95
         << ",\"frame_process_us_max\":" << c.frame_process_us_max << '}';
    }
    for (const auto& suffix : {std::string("color"), std::string("depth"),
                               std::string("ir_left"), std::string("ir_right")}) {
      const auto key = stream_key(cam, suffix);
      auto it = h.stream_stats.find(key);
      if (it == h.stream_stats.end()) continue;
      const auto& s = it->second;
      const uint64_t total_drop_count =
          s.frame_number_gap_drop_count + s.internal_queue_drop_count + s.recorder_drop_count;
      os << ",\"status_" << suffix << "\":\""
         << esc(h.stream_status.count(key) ? h.stream_status.at(key) : std::string("unknown")) << "\"";
      os << ",\"fps_" << suffix << "\":" << std::setprecision(3) << s.fps_estimate;
      os << ",\"fps_window_" << suffix << "\":" << std::setprecision(3) << s.fps_window_hz;
      os << ",\"drop_count_" << suffix << "\":" << s.frame_number_gap_drop_count;
      os << ",\"drops_" << suffix << "\":{\"frame_number_gap\":" << s.frame_number_gap_drop_count
         << ",\"internal_queue\":" << s.internal_queue_drop_count << ",\"recorder\":" << s.recorder_drop_count
         << ",\"total\":" << total_drop_count
         << ",\"frame_number_gap_delta\":" << s.frame_number_gap_drop_delta
         << ",\"internal_queue_delta\":" << s.internal_queue_drop_delta
         << ",\"recorder_delta\":" << s.recorder_drop_delta
         << ",\"shared_memory_write_error_delta\":" << s.shared_memory_write_error_delta << '}';
      os << ",\"frame_count_" << suffix << "\":" << s.frame_count;
      os << ",\"last_frame_age_ms_" << suffix << "\":"
         << (h.host_time_ns > s.last_frame_time_ns ? static_cast<double>(h.host_time_ns - s.last_frame_time_ns) / 1e6 : 0.0);
    }
    os << '}';
  }
  os << "},\"shared_memory\":{\"name\":\"" << esc(h.shm_name) << "\",\"size_bytes\":" << h.shm_size_bytes
     << ",\"write_errors\":";
  uint64_t write_errors = 0;
  for (const auto& [_, s] : h.stream_stats) write_errors += s.shared_memory_write_errors;
  os << write_errors << "},";
  os << "\"metadata\":{\"publish_count\":" << h.metadata_publish_count << ",\"publish_errors\":" << h.metadata_publish_errors << "},";
  os << "\"recorder\":{\"enabled\":" << (h.recorder_enabled ? "true" : "false") << ",\"queue_depth\":"
     << h.recorder_queue_depth << ",\"dropped_by_queue\":" << h.recorder_dropped_by_queue << "}}";
  return os.str();
}

MetadataPublisher::MetadataPublisher(MetadataConfig cfg) : cfg_(std::move(cfg)) {}
MetadataPublisher::~MetadataPublisher() { stop(); }

void MetadataPublisher::start() {
  if (started_) return;
#if CAMERA_SERVER_HAVE_ZMQ
  context_ = zmq_ctx_new();
  socket_ = zmq_socket(context_, ZMQ_PUB);
  int sndhwm = 4;
  zmq_setsockopt(socket_, ZMQ_SNDHWM, &sndhwm, sizeof(sndhwm));
  if (zmq_bind(socket_, cfg_.pub_bind.c_str()) != 0) {
    ++error_count_;
    throw std::runtime_error("zmq_bind failed for " + cfg_.pub_bind);
  }
#else
  std::cerr << "[CAM] ZeroMQ backend not compiled; metadata will be printed to stderr only\n";
#endif
  started_ = true;
}

void MetadataPublisher::stop() {
#if CAMERA_SERVER_HAVE_ZMQ
  if (socket_) {
    zmq_close(socket_);
    socket_ = nullptr;
  }
  if (context_) {
    zmq_ctx_term(context_);
    context_ = nullptr;
  }
#endif
  started_ = false;
}

bool MetadataPublisher::publish(const std::string& topic, const std::string& payload) {
  std::lock_guard<std::mutex> lk(publish_mutex_);
  if (!started_) start();
#if CAMERA_SERVER_HAVE_ZMQ
  const int flags_more = ZMQ_SNDMORE | ZMQ_DONTWAIT;
  if (zmq_send(socket_, topic.data(), topic.size(), flags_more) < 0) {
    ++error_count_;
    return false;
  }
  if (zmq_send(socket_, payload.data(), payload.size(), ZMQ_DONTWAIT) < 0) {
    ++error_count_;
    return false;
  }
#else
  if (topic == cfg_.health_topic) std::cerr << topic << ' ' << payload << '\n';
#endif
  ++publish_count_;
  return true;
}

bool MetadataPublisher::publish_bundle(const FrameBundleMeta& bundle) { return publish(cfg_.bundle_topic, bundle_to_json(bundle)); }
bool MetadataPublisher::publish_bundle(const std::string& topic, const FrameBundleMeta& bundle) {
  return publish(topic, bundle_to_json(bundle));
}
bool MetadataPublisher::publish_health(const HealthSnapshot& health) { return publish(cfg_.health_topic, health_to_json(health)); }

}  // namespace camera_server
