#include "camera_server/camera/realsense_device.hpp"

#include "camera_server/core/clock.hpp"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#if CAMERA_SERVER_HAVE_OPENCV
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#endif

namespace camera_server {

namespace {
bool all_digits(const std::string& s) {
  return !s.empty() && std::all_of(s.begin(), s.end(), [](unsigned char c) { return std::isdigit(c) != 0; });
}
}  // namespace

int resolve_v4l2_index(const std::string& device) {
  if (device.empty()) return -1;
  // Plain integer index ("12" -> /dev/video12).
  if (all_digits(device)) {
    try {
      return std::stoi(device);
    } catch (...) {
      return -1;
    }
  }
  // Resolve a /dev/v4l/by-path/... symlink (preferred, stable across reboots) to the
  // real /dev/videoN, then parse the trailing index.
  std::string path = device;
  std::error_code ec;
  const auto canon = std::filesystem::canonical(path, ec);
  if (!ec) path = canon.string();
  const std::string base = std::filesystem::path(path).filename().string();
  const std::string prefix = "video";
  if (base.rfind(prefix, 0) == 0) {
    const std::string num = base.substr(prefix.size());
    if (all_digits(num)) {
      try {
        return std::stoi(num);
      } catch (...) {
        return -1;
      }
    }
  }
  return -1;
}

bool uvc_device_present(const std::string& device) {
  if (device.empty()) return false;
  std::error_code ec;
  if (all_digits(device)) return std::filesystem::exists("/dev/video" + device, ec);
  // exists() follows symlinks, so a dangling by-path link reports absent.
  return std::filesystem::exists(device, ec);
}

#if CAMERA_SERVER_HAVE_OPENCV
class UvcDevice final : public ICameraDevice {
 public:
  UvcDevice(CameraConfig cfg, ClockKind clock) : cfg_(std::move(cfg)), clock_(clock) {}
  ~UvcDevice() override { stop(); }

  void start(FrameCallback cb) override {
    if (running_.exchange(true)) return;
    cb_ = std::move(cb);
    open_capture();  // throws on failure (fail-closed startup, like RealSense)
    thread_ = std::thread(&UvcDevice::loop, this);
  }

  void stop() override {
    if (!running_.exchange(false)) return;
    if (thread_.joinable()) thread_.join();
    if (cap_.isOpened()) cap_.release();
  }

 private:
  void open_capture() {
    const int index = resolve_v4l2_index(cfg_.device);
    bool opened = false;
    if (index >= 0) opened = cap_.open(index, cv::CAP_V4L2);
    if (!opened) opened = cap_.open(cfg_.device, cv::CAP_V4L2);  // fallback: raw path
    if (!opened) {
      throw std::runtime_error("uvc camera " + cfg_.name + " failed to open device: " + cfg_.device);
    }
    // MJPG transport keeps USB bandwidth low so the fisheye does not starve the
    // RealSense sharing the same hub — mirrors pika/pika_win/fisheye.py. The decoded
    // payload is still raw RGB in shared memory (no compression in the policy path).
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, cfg_.color.width);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, cfg_.color.height);
    cap_.set(cv::CAP_PROP_FPS, cfg_.color.fps);
    // NOTE: do NOT set CAP_PROP_BUFFERSIZE=1 here. On these DECXIN UVC cameras the
    // V4L2 backend with a single MMAP buffer cannot double-buffer (the driver has no
    // free buffer to fill while we hold/decode the current one), which HALVES the
    // capture rate to ~15 fps. The default buffering sustains the full 30 fps and the
    // tight read loop below keeps latency at ~1 frame anyway. Matches pika fisheye.py
    // (which also leaves the buffer count at the driver default). Measured 2026-06-22:
    // BUFFERSIZE=1 -> 15 fps, default -> 30 fps (single and dual, same USB2 bus).
    const int aw = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
    const int ah = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
    std::cerr << "[CAM] uvc " << cfg_.name << " open " << aw << "x" << ah << "@" << cfg_.color.fps
              << " MJPG device=" << cfg_.device << " (index=" << index << ")\n";
  }

  void loop() {
    uint64_t frame_no = 0;
    cv::Mat bgr;
    cv::Mat rgb;
    while (running_) {
      if (!cap_.read(bgr) || bgr.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        continue;
      }
      const uint64_t host_t = now_ns(clock_);
      // RGB to match the realsense `rgb8` shared-memory format: policy_runner treats
      // every 3-channel frame as already-decoded RGB (no BGR2RGB on the consumer side).
      cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
      if (!rgb.isContinuous()) rgb = rgb.clone();
      ++frame_no;
      CapturedFrame f;
      f.camera_name = cfg_.name;
      f.serial = cfg_.serial;
      f.stream = "color";
      f.frame_number = frame_no;
      f.host_arrival_time_ns = host_t;
      f.sensor_timestamp_ns = host_t;
      f.realsense_timestamp_ms = static_cast<double>(host_t) / 1e6;
      f.width = static_cast<uint32_t>(rgb.cols);
      f.height = static_cast<uint32_t>(rgb.rows);
      f.stride_bytes = static_cast<uint32_t>(rgb.step);
      f.format = "rgb8";
      const size_t bytes = rgb.total() * rgb.elemSize();
      f.bytes.assign(rgb.data, rgb.data + bytes);
      f.data = f.bytes.data();
      f.size_bytes = static_cast<uint32_t>(bytes);
      cb_(std::move(f));
    }
  }

  CameraConfig cfg_;
  ClockKind clock_;
  FrameCallback cb_;
  std::atomic<bool> running_{false};
  std::thread thread_;
  cv::VideoCapture cap_;
};
#endif  // CAMERA_SERVER_HAVE_OPENCV

std::unique_ptr<ICameraDevice> make_uvc_device(const CameraConfig& cfg, ClockKind clock) {
#if CAMERA_SERVER_HAVE_OPENCV
  return std::make_unique<UvcDevice>(cfg, clock);
#else
  (void)cfg;
  (void)clock;
  throw std::runtime_error("uvc backend not compiled (OpenCV not found at build time)");
#endif
}

}  // namespace camera_server
