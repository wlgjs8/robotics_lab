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
#include <chrono>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace camera_server {

class CameraManager {
 public:
  using DeviceFactory = std::function<std::unique_ptr<ICameraDevice>(const CameraConfig&)>;
  CameraManager(const AppConfig& cfg, SharedMemoryRingBuffer& shm, MetadataPublisher& publisher,
                FrameSynchronizerSet& synchronizer, Recorder& recorder,
                DeviceFactory device_factory = {});
  ~CameraManager();
  void start();
  void stop();
  HealthSnapshot snapshot() const;

 private:
  struct ProcessedFrame {
    FrameMeta meta;
  };

  struct CameraRuntime {
    explicit CameraRuntime(CameraConfig camera) : cfg(std::move(camera)) {}
    CameraConfig cfg;
    mutable std::mutex device_mu;
    std::unique_ptr<ICameraDevice> device;
    std::thread supervisor_thread;
    std::condition_variable wake_cv;
    std::mutex wake_mu;
    std::atomic<uint64_t> last_frame_time_ns{0};
  };

  void handle_frame(CapturedFrame&& frame);
  void metadata_loop();
  void process_frame_metadata(ProcessedFrame&& frame);
  void ensure_required_cameras_present();
  std::unique_ptr<ICameraDevice> make_device(const CameraConfig& cam) const;
  void supervise_camera(CameraRuntime& runtime);
  bool wait_or_stopping(CameraRuntime& runtime, std::chrono::milliseconds duration);

  AppConfig cfg_;
  SharedMemoryRingBuffer& shm_;
  MetadataPublisher& publisher_;
  FrameSynchronizerSet& synchronizer_;
  Recorder& recorder_;
  DeviceFactory device_factory_;
  std::vector<std::unique_ptr<CameraRuntime>> camera_runtimes_;
  mutable std::mutex mu_;
  std::map<std::string, StreamStats> stats_;
  std::map<std::string, bool> connected_;
  std::map<std::string, CameraReconnectStats> reconnect_stats_;
  std::map<std::string, RealSenseDeviceInfo> realsense_device_info_;
  std::string realsense_sdk_version_;
  std::string realsense_backend_;
  std::map<std::string, std::deque<uint64_t>> frame_time_windows_;
  BoundedQueue<ProcessedFrame> metadata_queue_;
  std::thread metadata_thread_;
  uint64_t start_time_ns_{0};
  std::atomic<bool> running_{false};
};

}  // namespace camera_server
