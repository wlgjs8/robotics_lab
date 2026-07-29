#pragma once

#include "camera_server/config/config.hpp"
#include "camera_server/core/types.hpp"

#include <functional>
#include <deque>
#include <memory>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace camera_server {

class FrameSynchronizer {
 public:
  explicit FrameSynchronizer(const AppConfig& cfg);
  FrameSynchronizer(SyncConfig sync, BundleGroupConfig group, double stats_window_sec = 5.0);

  std::optional<FrameBundleMeta> push_frame(const FrameMeta& meta,
                                            const std::map<std::string, uint64_t>& drop_counters);
  bool reset_camera_generation(const std::string& camera_name);
  uint64_t complete_bundle_count() const;
  uint64_t incomplete_bundle_count() const;
  uint64_t bundle_seq() const;
  double last_max_time_diff_ms() const;
  BundleStats stats() const;
  const std::string& topic() const { return group_.topic; }
  const std::string& group_name() const { return group_.name; }

 private:
  std::vector<std::string> required_keys_;
  std::string master_key_;
  SyncConfig sync_;
  BundleGroupConfig group_;
  bool hardware_synced_{false};
  mutable std::mutex mu_;
  std::map<std::string, std::deque<FrameMeta>> buffers_;
  size_t max_buffered_frames_{64};
  uint64_t last_emitted_master_frame_number_{0};
  uint64_t bundle_seq_{0};
  uint64_t complete_bundle_count_{0};
  uint64_t incomplete_bundle_count_{0};
  uint64_t incomplete_retry_count_{0};
  uint64_t dropped_master_count_{0};
  uint64_t pending_master_frame_number_{0};
  double last_max_time_diff_ms_{0.0};
  double stats_window_sec_{5.0};
  std::deque<std::pair<uint64_t, double>> completed_samples_;
};

struct PublishedBundle {
  std::string topic;
  FrameBundleMeta bundle;
};

class FrameSynchronizerSet {
 public:
  explicit FrameSynchronizerSet(const AppConfig& cfg);
  std::vector<PublishedBundle> push_frame(const FrameMeta& meta,
                                          const std::map<std::string, uint64_t>& drop_counters);
  size_t reset_camera_generation(const std::string& camera_name);
  std::map<std::string, BundleStats> stats() const;
  BundleStats compatibility_stats() const;

 private:
  std::vector<std::unique_ptr<FrameSynchronizer>> groups_;
};

}  // namespace camera_server
