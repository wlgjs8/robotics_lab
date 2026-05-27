#pragma once

#include "camera_server/config/config.hpp"
#include "camera_server/core/bounded_queue.hpp"
#include "camera_server/core/types.hpp"

#include <atomic>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace camera_server {

struct RecordingFrame {
  FrameMeta meta;
  std::vector<uint8_t> bytes;
};

class Recorder {
 public:
  explicit Recorder(RecordingConfig cfg);
  ~Recorder();
  void start();
  void stop();
  bool enqueue(RecordingFrame frame);
  bool enqueue_copy(FrameMeta meta, const uint8_t* data, size_t size_bytes);
  bool enabled() const { return cfg_.enabled; }
  uint64_t dropped_by_queue() const { return queue_.dropped_count(); }
  uint64_t queue_depth() const { return queue_.size(); }

 private:
  void worker_loop(size_t worker_id);
  RecordingConfig cfg_;
  BoundedQueue<RecordingFrame> queue_;
  std::atomic<bool> running_{false};
  std::vector<std::thread> workers_;
  std::atomic<uint64_t> file_seq_{0};
  std::mutex metadata_mutex_;
};

}  // namespace camera_server
