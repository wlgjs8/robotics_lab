#pragma once

#include "camera_server/config/config.hpp"
#include "camera_server/core/types.hpp"

#include <functional>
#include <deque>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace camera_server {

class FrameSynchronizer {
 public:
  explicit FrameSynchronizer(const AppConfig& cfg);

  std::optional<FrameBundleMeta> push_frame(const FrameMeta& meta,
                                            const std::map<std::string, uint64_t>& drop_counters);
  uint64_t complete_bundle_count() const;
  uint64_t incomplete_bundle_count() const;
  uint64_t bundle_seq() const;
  double last_max_time_diff_ms() const;

 private:
  std::vector<std::string> required_keys_;
  std::string master_key_;
  SyncConfig sync_;
  bool hardware_synced_{false};
  mutable std::mutex mu_;
  std::map<std::string, std::deque<FrameMeta>> buffers_;
  size_t max_buffered_frames_{64};
  uint64_t last_emitted_master_frame_number_{0};
  uint64_t bundle_seq_{0};
  uint64_t complete_bundle_count_{0};
  uint64_t incomplete_bundle_count_{0};
  double last_max_time_diff_ms_{0.0};
};

}  // namespace camera_server
