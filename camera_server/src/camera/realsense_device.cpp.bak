#include "camera_server/camera/realsense_device.hpp"

#include "camera_server/core/clock.hpp"

#include <chrono>
#include <algorithm>
#include <cstring>
#include <iostream>
#include <stdexcept>

#if CAMERA_SERVER_HAVE_REALSENSE
#include <librealsense2/rs.hpp>
#endif

namespace camera_server {

std::vector<std::string> discover_realsense_serials() {
  std::vector<std::string> serials;
#if CAMERA_SERVER_HAVE_REALSENSE
  rs2::context ctx;
  for (auto&& dev : ctx.query_devices()) serials.emplace_back(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
#endif
  return serials;
}

class MockCameraDevice final : public ICameraDevice {
 public:
  MockCameraDevice(CameraConfig cfg, ClockKind clock) : cfg_(std::move(cfg)), clock_(clock) {}
  void start(FrameCallback cb) override {
    if (running_.exchange(true)) return;
    cb_ = std::move(cb);
    thread_ = std::thread(&MockCameraDevice::loop, this);
  }
  void stop() override {
    if (!running_.exchange(false)) return;
    if (thread_.joinable()) thread_.join();
  }
  ~MockCameraDevice() override { stop(); }

 private:
  void emit_stream(const std::string& stream, const CameraStreamConfig& scfg, uint64_t frame_no, uint64_t t) {
    if (!scfg.enabled) return;
    CapturedFrame f;
    f.camera_name = cfg_.name;
    f.serial = cfg_.serial.empty() ? ("MOCK_" + cfg_.name) : cfg_.serial;
    f.stream = stream;
    f.frame_number = frame_no;
    f.host_arrival_time_ns = t;
    f.sensor_timestamp_ns = t;
    f.realsense_timestamp_ms = static_cast<double>(t) / 1e6;
    f.width = static_cast<uint32_t>(scfg.width);
    f.height = static_cast<uint32_t>(scfg.height);
    f.format = scfg.format;
    const uint32_t bpp = (scfg.format == "rgb8" || scfg.format == "bgr8") ? 3u : ((scfg.format == "z16") ? 2u : 1u);
    f.stride_bytes = f.width * bpp;
    f.bytes.resize(static_cast<size_t>(f.stride_bytes) * f.height);
    std::fill(f.bytes.begin(), f.bytes.end(), static_cast<uint8_t>((frame_no + cfg_.name.size()) % 251));
    f.data = f.bytes.data();
    f.size_bytes = static_cast<uint32_t>(f.bytes.size());
    cb_(std::move(f));
  }

  void loop() {
    uint64_t frame_no = 0;
    const int fps = cfg_.color.enabled ? cfg_.color.fps : (cfg_.depth.enabled ? cfg_.depth.fps : 30);
    const auto period = std::chrono::nanoseconds(1000000000ll / std::max(1, fps));
    auto next = std::chrono::steady_clock::now();
    while (running_) {
      next += period;
      ++frame_no;
      const uint64_t t = now_ns(clock_);
      emit_stream("color", cfg_.color, frame_no, t);
      emit_stream("depth", cfg_.depth, frame_no, t + 100000);
      std::this_thread::sleep_until(next);
    }
  }

  CameraConfig cfg_;
  ClockKind clock_;
  FrameCallback cb_;
  std::atomic<bool> running_{false};
  std::thread thread_;
};

std::unique_ptr<ICameraDevice> make_mock_camera_device(const CameraConfig& cfg, ClockKind clock) {
  return std::make_unique<MockCameraDevice>(cfg, clock);
}

#if CAMERA_SERVER_HAVE_REALSENSE
class RealSenseDevice final : public ICameraDevice {
 public:
  RealSenseDevice(CameraConfig cfg, ClockKind clock, SyncConfig sync)
      : cfg_(std::move(cfg)), clock_(clock), sync_(std::move(sync)) {}
  ~RealSenseDevice() override { stop(); }

  void start(FrameCallback cb) override {
    cb_ = std::move(cb);
    configure_hardware_sync_if_requested();
    rs2::config rs_cfg;
    rs_cfg.enable_device(cfg_.serial);
    if (cfg_.color.enabled) {
      rs_cfg.enable_stream(RS2_STREAM_COLOR, cfg_.color.width, cfg_.color.height,
                           cfg_.color.format == "bgr8" ? RS2_FORMAT_BGR8 : RS2_FORMAT_RGB8, cfg_.color.fps);
    }
    if (cfg_.depth.enabled) {
      rs_cfg.enable_stream(RS2_STREAM_DEPTH, cfg_.depth.width, cfg_.depth.height, RS2_FORMAT_Z16, cfg_.depth.fps);
    }
    pipe_.start(rs_cfg, [this](const rs2::frame& frame) { on_frame(frame); });
  }

  void stop() override {
    try { pipe_.stop(); } catch (...) {}
  }

 private:
  void configure_hardware_sync_if_requested() {
    if (sync_.mode != "hardware") return;

    rs2::context ctx;
    bool configured = false;
    const float sync_mode = cfg_.name == sync_.master_camera ? 1.0f : 2.0f;
    for (auto&& dev : ctx.query_devices()) {
      if (cfg_.serial != dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)) continue;
      for (auto&& sensor : dev.query_sensors()) {
        if (!sensor.supports(RS2_OPTION_INTER_CAM_SYNC_MODE)) continue;
        sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, sync_mode);
        configured = true;
      }
      break;
    }
    if (!configured) {
      throw std::runtime_error("sync.mode=hardware requested but RealSense inter_cam_sync_mode is unsupported for " +
                               cfg_.name + " serial=" + cfg_.serial);
    }
    std::cerr << "[CAM] hardware sync configured for " << cfg_.name
              << " role=" << (cfg_.name == sync_.master_camera ? "master" : "slave") << '\n';
  }

  void on_video_frame(const rs2::video_frame& vf, const std::string& stream_name, const CameraStreamConfig& scfg,
                      uint64_t host_t) {
    CapturedFrame f;
    f.camera_name = cfg_.name;
    f.serial = cfg_.serial;
    f.stream = stream_name;
    f.frame_number = vf.get_frame_number();
    f.host_arrival_time_ns = host_t;
    f.realsense_timestamp_ms = vf.get_timestamp();
    f.sensor_timestamp_ns = static_cast<uint64_t>(vf.get_timestamp() * 1e6);
    f.width = static_cast<uint32_t>(vf.get_width());
    f.height = static_cast<uint32_t>(vf.get_height());
    f.stride_bytes = static_cast<uint32_t>(vf.get_stride_in_bytes());
    f.format = scfg.format;
    const size_t bytes = static_cast<size_t>(vf.get_height()) * static_cast<size_t>(vf.get_stride_in_bytes());
    f.data = static_cast<const uint8_t*>(vf.get_data());
    f.size_bytes = static_cast<uint32_t>(bytes);
    cb_(std::move(f));
  }

  void on_frame(const rs2::frame& frame) {
    try {
      if (auto fs = frame.as<rs2::frameset>()) {
        const uint64_t host_t = now_ns(clock_);
        if (cfg_.color.enabled) {
          auto color = fs.get_color_frame();
          if (color) on_video_frame(color, "color", cfg_.color, host_t);
        }
        if (cfg_.depth.enabled) {
          auto depth = fs.get_depth_frame();
          if (depth) on_video_frame(depth, "depth", cfg_.depth, host_t);
        }
      } else if (auto vf = frame.as<rs2::video_frame>()) {
        const auto profile = vf.get_profile().stream_type();
        const uint64_t host_t = now_ns(clock_);
        if (profile == RS2_STREAM_COLOR && cfg_.color.enabled) on_video_frame(vf, "color", cfg_.color, host_t);
        if (profile == RS2_STREAM_DEPTH && cfg_.depth.enabled) on_video_frame(vf, "depth", cfg_.depth, host_t);
      }
    } catch (const std::exception& e) {
      std::cerr << "[CAM] RealSense callback error for " << cfg_.name << ": " << e.what() << '\n';
    }
  }

  CameraConfig cfg_;
  ClockKind clock_;
  SyncConfig sync_;
  FrameCallback cb_;
  rs2::pipeline pipe_;
};
#endif

std::unique_ptr<ICameraDevice> make_realsense_device(const CameraConfig& cfg, ClockKind clock, const SyncConfig& sync) {
#if CAMERA_SERVER_HAVE_REALSENSE
  return std::make_unique<RealSenseDevice>(cfg, clock, sync);
#else
  (void)cfg;
  (void)clock;
  (void)sync;
  throw std::runtime_error("RealSense backend not compiled");
#endif
}

std::unique_ptr<ICameraDevice> make_realsense_device(const CameraConfig& cfg, ClockKind clock) {
  SyncConfig sync;
  return make_realsense_device(cfg, clock, sync);
}

}  // namespace camera_server
