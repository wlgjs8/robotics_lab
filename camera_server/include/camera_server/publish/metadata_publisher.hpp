#pragma once

#include "camera_server/config/config.hpp"
#include "camera_server/core/types.hpp"

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

namespace camera_server {

std::string frame_to_json(const FrameMeta& meta);
std::string bundle_to_json(const FrameBundleMeta& bundle);
std::string health_to_json(const HealthSnapshot& health);

class MetadataPublisher {
 public:
  explicit MetadataPublisher(MetadataConfig cfg);
  ~MetadataPublisher();
  MetadataPublisher(const MetadataPublisher&) = delete;
  MetadataPublisher& operator=(const MetadataPublisher&) = delete;

  void start();
  void stop();
  bool publish(const std::string& topic, const std::string& payload);
  bool publish_bundle(const FrameBundleMeta& bundle);
  bool publish_health(const HealthSnapshot& health);
  uint64_t publish_count() const { return publish_count_.load(); }
  uint64_t error_count() const { return error_count_.load(); }

 private:
  MetadataConfig cfg_;
  void* context_{nullptr};
  void* socket_{nullptr};
  std::atomic<uint64_t> publish_count_{0};
  std::atomic<uint64_t> error_count_{0};
  std::mutex publish_mutex_;
  bool started_{false};
};

}  // namespace camera_server
