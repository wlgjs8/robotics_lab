#pragma once

#include "camera_server/config/config.hpp"
#include "camera_server/core/types.hpp"
#include "camera_server/publish/metadata_publisher.hpp"

#include <atomic>
#include <functional>
#include <thread>

namespace camera_server {

void apply_health_thresholds(const HealthConfig& cfg, HealthSnapshot& snapshot);

class HealthMonitor {
 public:
  using SnapshotFn = std::function<HealthSnapshot()>;
  HealthMonitor(HealthConfig cfg, MetadataPublisher& publisher, SnapshotFn snapshot_fn);
  ~HealthMonitor();
  void start();
  void stop();

 private:
  void loop();
  HealthConfig cfg_;
  MetadataPublisher& publisher_;
  SnapshotFn snapshot_fn_;
  std::atomic<bool> running_{false};
  std::thread thread_;
};

}  // namespace camera_server
