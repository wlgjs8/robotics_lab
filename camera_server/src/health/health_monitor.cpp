#include "camera_server/health/health_monitor.hpp"

#include <chrono>
#include <iostream>

namespace camera_server {
namespace {

uint64_t counter_delta(uint64_t current, uint64_t previous) {
  return current >= previous ? current - previous : current;
}

uint64_t total_drop_delta(const StreamStats& st) {
  return st.frame_number_gap_drop_delta + st.internal_queue_drop_delta + st.recorder_drop_delta;
}

void mark_degraded(HealthSnapshot& snapshot, const std::string& reason) {
  if (snapshot.status == "ok") snapshot.status = "degraded";
  snapshot.status_reasons.push_back(reason);
}

void mark_unhealthy(HealthSnapshot& snapshot, const std::string& reason) {
  snapshot.status = "unhealthy";
  snapshot.status_reasons.push_back(reason);
}

}  // namespace

void populate_health_deltas(HealthSnapshot& snapshot, const HealthSnapshot* previous) {
  for (auto& [key, current] : snapshot.stream_stats) {
    if (!previous) continue;
    const auto it = previous->stream_stats.find(key);
    if (it == previous->stream_stats.end()) continue;
    const auto& prior = it->second;
    current.frame_number_gap_drop_delta =
        counter_delta(current.frame_number_gap_drop_count, prior.frame_number_gap_drop_count);
    current.internal_queue_drop_delta =
        counter_delta(current.internal_queue_drop_count, prior.internal_queue_drop_count);
    current.recorder_drop_delta = counter_delta(current.recorder_drop_count, prior.recorder_drop_count);
    current.shared_memory_write_error_delta =
        counter_delta(current.shared_memory_write_errors, prior.shared_memory_write_errors);
  }
  for (auto& [camera, current] : snapshot.camera_capture_stats) {
    if (!previous) continue;
    const auto it = previous->camera_capture_stats.find(camera);
    if (it == previous->camera_capture_stats.end()) continue;
    current.queue_drop_delta = counter_delta(current.queue_drop_count, it->second.queue_drop_count);
  }
}

void apply_health_thresholds(const HealthConfig& cfg, HealthSnapshot& snapshot) {
  snapshot.status = "ok";
  snapshot.status_reasons.clear();
  snapshot.stream_status.clear();

  for (const auto& [camera, connected] : snapshot.camera_connected) {
    if (!connected) mark_unhealthy(snapshot, "camera_disconnected:" + camera);
  }

  if (snapshot.bundle_groups.empty()) {
    if (snapshot.max_time_diff_ms > cfg.warn_if_bundle_skew_ms_gt) {
      mark_degraded(snapshot, "bundle_skew_ms_gt_threshold");
    }
  } else {
    for (const auto& [name, group] : snapshot.bundle_groups) {
      if (group.last_skew_ms > cfg.warn_if_bundle_skew_ms_gt) {
        mark_degraded(snapshot, "bundle_skew_ms_gt_threshold:" + name);
      }
      if (snapshot.uptime_sec >= cfg.fps_window_sec && group.complete_bundle_count >= 2 &&
          cfg.warn_if_fps_below > 0.0 && group.publish_rate_hz < cfg.warn_if_fps_below) {
        mark_degraded(snapshot, "bundle_rate_below_threshold:" + name);
      }
    }
  }

  for (const auto& [key, st] : snapshot.stream_stats) {
    std::string status = "ok";
    if (st.shared_memory_write_errors > 0) {
      status = "unhealthy";
      mark_unhealthy(snapshot, "shared_memory_write_errors:" + key);
    }
    if (cfg.warn_if_drop_count_increases && total_drop_delta(st) > 0) {
      if (status == "ok") status = "degraded";
      mark_degraded(snapshot, "drops_observed:" + key);
    }
    if (snapshot.uptime_sec >= cfg.fps_window_sec && cfg.warn_if_fps_below > 0.0 &&
        st.fps_window_hz < cfg.warn_if_fps_below) {
      if (status == "ok") status = "degraded";
      mark_degraded(snapshot, "fps_below_threshold:" + key);
    }
    if (st.last_frame_time_ns != 0 && snapshot.host_time_ns > st.last_frame_time_ns) {
      const double age_ms = static_cast<double>(snapshot.host_time_ns - st.last_frame_time_ns) / 1e6;
      if (age_ms > cfg.warn_if_frame_age_ms_gt) {
        if (status == "ok") status = "degraded";
        mark_degraded(snapshot, "frame_age_ms_gt_threshold:" + key);
      }
    }
    snapshot.stream_status[key] = status;
  }
  for (const auto& [camera, capture] : snapshot.camera_capture_stats) {
    if (capture.queue_drop_delta > 0) mark_degraded(snapshot, "capture_queue_drops:" + camera);
  }
}

HealthMonitor::HealthMonitor(HealthConfig cfg, MetadataPublisher& publisher, SnapshotFn snapshot_fn)
    : cfg_(cfg), publisher_(publisher), snapshot_fn_(std::move(snapshot_fn)) {}
HealthMonitor::~HealthMonitor() { stop(); }

void HealthMonitor::start() {
  if (running_.exchange(true)) return;
  thread_ = std::thread(&HealthMonitor::loop, this);
}

void HealthMonitor::stop() {
  if (!running_.exchange(false)) return;
  if (thread_.joinable()) thread_.join();
}

void HealthMonitor::loop() {
  const double rate = cfg_.publish_rate_hz <= 0.0 ? 1.0 : cfg_.publish_rate_hz;
  const auto period = std::chrono::duration<double>(1.0 / rate);
  while (running_) {
    auto snap = snapshot_fn_();
    populate_health_deltas(snap, have_previous_snapshot_ ? &previous_snapshot_ : nullptr);
    apply_health_thresholds(cfg_, snap);
    publisher_.publish_health(snap);
    std::cerr << "[CAM] status=" << snap.status << " bundle=" << snap.bundle_seq << " complete=" << snap.complete_bundle_count
              << " incomplete=" << snap.incomplete_bundle_count << " skew=" << snap.max_time_diff_ms << "ms";
    for (const auto& [key, st] : snap.stream_stats) {
      std::cerr << " | " << key << ' ' << st.fps_window_hz << "fps(window) drop="
                << st.frame_number_gap_drop_count << "(+" << st.frame_number_gap_drop_delta << ')';
    }
    std::cerr << " | shm_ok=" << (snap.shm_size_bytes > 0 ? 1 : 0) << " | rec_q=" << snap.recorder_queue_depth << '\n';
    previous_snapshot_ = snap;
    have_previous_snapshot_ = true;
    std::this_thread::sleep_for(period);
  }
}

}  // namespace camera_server
