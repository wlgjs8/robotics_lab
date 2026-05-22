#pragma once

#include "camera_server/camera/realsense_device.hpp"
#include "camera_server/config/config.hpp"
#include "camera_server/core/bounded_queue.hpp"
#include "camera_server/core/types.hpp"
#include "camera_server/publish/metadata_publisher.hpp"
#include "camera_server/recording/recorder.hpp"
#include "camera_server/shm/shared_memory_ring.hpp"
#include "camera_server/sync/frame_synchronizer.hpp"

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace camera_server {

class CameraManager {
 public:
  CameraManager(const AppConfig& cfg, SharedMemoryRingBuffer& shm, MetadataPublisher& publisher,
                FrameSynchronizer& synchronizer, Recorder& recorder);
  ~CameraManager();
  void start();
  void stop();
  HealthSnapshot snapshot() const;

 private:
  struct ProcessedFrame {
    FrameMeta meta;
  };

  void handle_frame(CapturedFrame&& frame);
  void metadata_loop();
  void process_frame_metadata(ProcessedFrame&& frame);
  void ensure_required_cameras_present() const;

  AppConfig cfg_;
  SharedMemoryRingBuffer& shm_;
  MetadataPublisher& publisher_;
  FrameSynchronizer& synchronizer_;
  Recorder& recorder_;
  std::vector<std::unique_ptr<ICameraDevice>> devices_;
  mutable std::mutex mu_;
  std::map<std::string, StreamStats> stats_;
  std::map<std::string, bool> connected_;
  BoundedQueue<ProcessedFrame> metadata_queue_;
  std::thread metadata_thread_;
  uint64_t start_time_ns_{0};
  std::atomic<bool> running_{false};
};

}  // namespace camera_server
