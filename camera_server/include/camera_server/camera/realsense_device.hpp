#pragma once

#include "camera_server/config/config.hpp"

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace camera_server {

struct CapturedFrame {
  std::string camera_name;
  std::string serial;
  std::string stream;
  uint64_t frame_number{0};
  uint64_t host_arrival_time_ns{0};
  uint64_t sensor_timestamp_ns{0};
  double realsense_timestamp_ms{0.0};
  uint32_t width{0};
  uint32_t height{0};
  uint32_t stride_bytes{0};
  std::string format;
  const uint8_t* data{nullptr};
  uint32_t size_bytes{0};
  std::vector<uint8_t> bytes;
};

using FrameCallback = std::function<void(CapturedFrame&&)>;

std::vector<std::string> discover_realsense_serials();

class ICameraDevice {
 public:
  virtual ~ICameraDevice() = default;
  virtual void start(FrameCallback cb) = 0;
  virtual void stop() = 0;
};

std::unique_ptr<ICameraDevice> make_realsense_device(const CameraConfig& cfg, ClockKind clock);
std::unique_ptr<ICameraDevice> make_realsense_device(const CameraConfig& cfg, ClockKind clock, const SyncConfig& sync);
std::unique_ptr<ICameraDevice> make_mock_camera_device(const CameraConfig& cfg, ClockKind clock);

}  // namespace camera_server
