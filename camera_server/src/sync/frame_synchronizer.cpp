#include "camera_server/sync/frame_synchronizer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace camera_server {

FrameSynchronizer::FrameSynchronizer(const AppConfig& cfg)
    : required_keys_(required_stream_keys(cfg)), master_key_(stream_key(cfg.sync.master_camera, "color")),
      sync_(cfg.sync), hardware_synced_(cfg.sync.mode == "hardware") {
  if (std::find(required_keys_.begin(), required_keys_.end(), master_key_) == required_keys_.end() && !required_keys_.empty()) {
    master_key_ = required_keys_.front();
  }
}

namespace {

const FrameMeta* nearest_by_timestamp(const std::deque<FrameMeta>& frames, uint64_t target_time_ns) {
  const FrameMeta* best = nullptr;
  uint64_t best_diff = std::numeric_limits<uint64_t>::max();
  for (const auto& frame : frames) {
    const uint64_t diff = frame.host_arrival_time_ns > target_time_ns
                              ? frame.host_arrival_time_ns - target_time_ns
                              : target_time_ns - frame.host_arrival_time_ns;
    if (!best || diff < best_diff) {
      best = &frame;
      best_diff = diff;
    }
  }
  return best;
}

const FrameMeta* exact_frame_number(const std::deque<FrameMeta>& frames, uint64_t frame_number) {
  for (const auto& frame : frames) {
    if (frame.frame_number == frame_number) return &frame;
  }
  return nullptr;
}

void trim_buffer(std::deque<FrameMeta>& frames, size_t max_size) {
  while (frames.size() > max_size) frames.pop_front();
}

}  // namespace

std::optional<FrameBundleMeta> FrameSynchronizer::push_frame(const FrameMeta& meta,
                                                             const std::map<std::string, uint64_t>& drop_counters) {
  std::lock_guard<std::mutex> lk(mu_);
  auto& current_buffer = buffers_[meta.ring_name];
  current_buffer.push_back(meta);
  trim_buffer(current_buffer, max_buffered_frames_);

  // Master-camera-driven bundling: every frame may complete the latest pending
  // master frame, but each master frame is emitted at most once. This keeps the
  // bundle cadence at the master camera rate without requiring the master frame
  // callback to arrive after the other cameras.
  auto master_it = buffers_.find(master_key_);
  if (master_it == buffers_.end() || master_it->second.empty()) return std::nullopt;
  const FrameMeta master = master_it->second.back();
  if (master.frame_number != 0 && master.frame_number <= last_emitted_master_frame_number_) return std::nullopt;

  std::map<std::string, FrameMeta> selected;
  selected[master.ring_name] = master;

  bool complete = true;
  uint64_t min_t = UINT64_MAX;
  uint64_t max_t = 0;
  for (const auto& key : required_keys_) {
    const FrameMeta* candidate = nullptr;
    if (key == master.ring_name) {
      candidate = &master;
    } else {
      auto it = buffers_.find(key);
      if (it != buffers_.end()) {
        candidate = hardware_synced_ ? exact_frame_number(it->second, master.frame_number)
                                     : nearest_by_timestamp(it->second, master.host_arrival_time_ns);
      }
    }
    if (!candidate || !candidate->valid) {
      complete = false;
      continue;
    }
    selected[key] = *candidate;
    min_t = std::min(min_t, candidate->host_arrival_time_ns);
    max_t = std::max(max_t, candidate->host_arrival_time_ns);
  }

  double diff_ms = 0.0;
  if (min_t != UINT64_MAX && max_t >= min_t) diff_ms = static_cast<double>(max_t - min_t) / 1e6;
  last_max_time_diff_ms_ = diff_ms;
  if (complete && diff_ms > sync_.max_bundle_time_diff_ms) complete = false;

  if (!complete && !sync_.publish_incomplete_bundles) {
    ++incomplete_bundle_count_;
    return std::nullopt;
  }

  FrameBundleMeta bundle;
  bundle.bundle_seq = ++bundle_seq_;
  bundle.hardware_synced = hardware_synced_;
  bundle.sync_policy = sync_.bundle_policy;
  bundle.max_time_diff_ms = diff_ms;
  bundle.complete = complete;
  bundle.drop_counters = drop_counters;
  if (min_t == UINT64_MAX) {
      bundle.bundle_time_ns = master.host_arrival_time_ns;
  } else {
    bundle.bundle_time_ns = min_t + (max_t - min_t) / 2;
  }
  bundle.frames = std::move(selected);
  if (complete) ++complete_bundle_count_;
  else ++incomplete_bundle_count_;
  if (master.frame_number != 0) last_emitted_master_frame_number_ = master.frame_number;

  for (const auto& key : required_keys_) {
    auto it = buffers_.find(key);
    if (it == buffers_.end()) continue;
    auto& frames = it->second;
    if (hardware_synced_) {
      while (!frames.empty() && frames.front().frame_number < master.frame_number) frames.pop_front();
      if (complete) {
        while (!frames.empty() && frames.front().frame_number <= master.frame_number) frames.pop_front();
      }
    } else {
      const uint64_t horizon_ns = master.host_arrival_time_ns >
                                          static_cast<uint64_t>(sync_.max_bundle_time_diff_ms * 1e6)
                                      ? master.host_arrival_time_ns -
                                            static_cast<uint64_t>(sync_.max_bundle_time_diff_ms * 1e6)
                                      : 0;
      while (!frames.empty() && frames.front().host_arrival_time_ns < horizon_ns) frames.pop_front();
    }
  }
  return bundle;
}

uint64_t FrameSynchronizer::complete_bundle_count() const {
  std::lock_guard<std::mutex> lk(mu_);
  return complete_bundle_count_;
}

uint64_t FrameSynchronizer::incomplete_bundle_count() const {
  std::lock_guard<std::mutex> lk(mu_);
  return incomplete_bundle_count_;
}

uint64_t FrameSynchronizer::bundle_seq() const {
  std::lock_guard<std::mutex> lk(mu_);
  return bundle_seq_;
}

double FrameSynchronizer::last_max_time_diff_ms() const {
  std::lock_guard<std::mutex> lk(mu_);
  return last_max_time_diff_ms_;
}

}  // namespace camera_server
