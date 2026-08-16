#include "camera_server/camera/realsense_device.hpp"

#include "camera_server/core/clock.hpp"
#include "camera_server/core/bounded_queue.hpp"

#include <chrono>
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>

#if CAMERA_SERVER_HAVE_REALSENSE
#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#endif

namespace camera_server {

namespace {

std::tuple<int, int, int, int> parse_version(const std::string& text) {
  std::tuple<int, int, int, int> out{0, 0, 0, 0};
  std::stringstream input(text);
  std::string part;
  int values[4]{0, 0, 0, 0};
  size_t index = 0;
  while (index < 4 && std::getline(input, part, '.')) {
    try {
      values[index] = std::stoi(part);
    } catch (...) {
      return {0, 0, 0, 0};
    }
    ++index;
  }
  return {values[0], values[1], values[2], values[3]};
}

}  // namespace

std::vector<RealSenseDeviceInfo> discover_realsense_devices() {
  std::vector<RealSenseDeviceInfo> devices;
#if CAMERA_SERVER_HAVE_REALSENSE
  rs2::context ctx;
  for (auto&& dev : ctx.query_devices()) {
    auto get = [&](rs2_camera_info field) -> std::string {
      return dev.supports(field) ? dev.get_info(field) : std::string();
    };
    RealSenseDeviceInfo info;
    info.name = get(RS2_CAMERA_INFO_NAME);
    info.serial = get(RS2_CAMERA_INFO_SERIAL_NUMBER);
    info.firmware_version = get(RS2_CAMERA_INFO_FIRMWARE_VERSION);
    info.recommended_firmware_version = get(RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION);
    info.physical_port = get(RS2_CAMERA_INFO_PHYSICAL_PORT);
    info.product_id = get(RS2_CAMERA_INFO_PRODUCT_ID);
    info.usb_type = get(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR);
    devices.push_back(std::move(info));
  }
#endif
  return devices;
}

std::vector<std::string> discover_realsense_serials() {
  std::vector<std::string> serials;
  for (const auto& device : discover_realsense_devices()) serials.push_back(device.serial);
  return serials;
}

std::string librealsense_sdk_version() {
#if CAMERA_SERVER_HAVE_REALSENSE
  rs2_error* error = nullptr;
  const int version = rs2_get_api_version(&error);
  if (error != nullptr) {
    const std::string message = rs2_get_error_message(error);
    rs2_free_error(error);
    throw std::runtime_error("failed to query librealsense SDK version: " + message);
  }
  return std::to_string(version / 10000) + "." + std::to_string((version / 100) % 100) + "." +
         std::to_string(version % 100);
#else
  return "unavailable";
#endif
}

std::string librealsense_backend() {
#if CAMERA_SERVER_HAVE_REALSENSE
  return CAMERA_SERVER_REALSENSE_BACKEND;
#else
  return "unavailable";
#endif
}

void validate_realsense_preflight(const AppConfig& cfg,
                                  const std::vector<RealSenseDeviceInfo>& devices,
                                  const std::string& sdk_version) {
  if (cfg.server.simulate_cameras) return;
  bool requires_realsense = false;
  for (const auto& camera : cfg.cameras) {
    if (camera.backend == "realsense" && camera.required) requires_realsense = true;
  }
  if (!requires_realsense) return;
  if (parse_version(sdk_version) < parse_version("2.58.1")) {
    throw std::runtime_error("librealsense SDK " + sdk_version +
                             " is older than required 2.58.1 for the deployed D400 firmware set");
  }

  std::map<std::string, RealSenseDeviceInfo> by_serial;
  for (const auto& device : devices) by_serial[device.serial] = device;
  std::string d405_firmware;
  for (const auto& camera : cfg.cameras) {
    if (camera.backend != "realsense" || !camera.required) continue;
    const auto found = by_serial.find(camera.serial);
    if (found == by_serial.end()) continue;  // Missing-device error retains the connected-serial detail.
    const auto& device = found->second;
    if (device.usb_type.empty() || device.usb_type.front() != '3') {
      throw std::runtime_error("required RealSense camera is not on USB3: " + camera.name +
                               " serial=" + camera.serial + " usb_type=" + device.usb_type);
    }
    if (device.product_id != "0B5B") continue;
    if (parse_version(device.firmware_version) < parse_version("5.17.0.10")) {
      throw std::runtime_error("D405 firmware is older than required 5.17.0.10: " + camera.name +
                               " serial=" + camera.serial + " firmware=" + device.firmware_version);
    }
    if (d405_firmware.empty()) {
      d405_firmware = device.firmware_version;
    } else if (device.firmware_version != d405_firmware) {
      throw std::runtime_error("configured D405 cameras must use identical firmware: expected=" +
                               d405_firmware + " got=" + device.firmware_version +
                               " serial=" + camera.serial);
    }
  }
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
      for (const auto& [name, scfg] : enabled_streams(cfg_)) emit_stream(name, scfg, frame_no, t);
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
namespace {

class AtomicTimingWindow {
 public:
  void record(uint64_t value_ns) {
    samples_[next_.fetch_add(1, std::memory_order_relaxed) % samples_.size()].store(
        value_ns, std::memory_order_relaxed);
  }

  std::pair<double, double> p95_max_us() const {
    std::vector<uint64_t> values;
    values.reserve(samples_.size());
    for (const auto& sample : samples_) {
      const auto value = sample.load(std::memory_order_relaxed);
      if (value != 0) values.push_back(value);
    }
    if (values.empty()) return {0.0, 0.0};
    std::sort(values.begin(), values.end());
    const size_t p95 = static_cast<size_t>(0.95 * static_cast<double>(values.size() - 1));
    return {static_cast<double>(values[p95]) / 1000.0, static_cast<double>(values.back()) / 1000.0};
  }

 private:
  std::array<std::atomic<uint64_t>, 256> samples_{};
  std::atomic<uint64_t> next_{0};
};

uint64_t steady_now_ns() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
}

}  // namespace

class RealSenseDevice final : public ICameraDevice {
 public:
  RealSenseDevice(CameraConfig cfg, ClockKind clock, SyncConfig sync)
      : cfg_(std::move(cfg)), clock_(clock), sync_(std::move(sync)) {}
  ~RealSenseDevice() override { stop(); }

  void start(FrameCallback cb) override {
    if (running_.load()) return;
    cb_ = std::move(cb);
    configure_hardware_sync_if_requested();
    maybe_load_advanced_json();
    rs2::config rs_cfg;
    rs_cfg.enable_device(cfg_.serial);
    if (cfg_.color.enabled) {
      rs_cfg.enable_stream(RS2_STREAM_COLOR, cfg_.color.width, cfg_.color.height,
                           cfg_.color.format == "bgr8" ? RS2_FORMAT_BGR8 : RS2_FORMAT_RGB8, cfg_.color.fps);
    }
    if (cfg_.depth.enabled) {
      rs_cfg.enable_stream(RS2_STREAM_DEPTH, cfg_.depth.width, cfg_.depth.height, RS2_FORMAT_Z16, cfg_.depth.fps);
    }
    if (cfg_.ir_left.enabled) {
      rs_cfg.enable_stream(RS2_STREAM_INFRARED, 1, cfg_.ir_left.width, cfg_.ir_left.height,
                           cfg_.ir_left.format == "y16" ? RS2_FORMAT_Y16 : RS2_FORMAT_Y8, cfg_.ir_left.fps);
    }
    if (cfg_.ir_right.enabled) {
      rs_cfg.enable_stream(RS2_STREAM_INFRARED, 2, cfg_.ir_right.width, cfg_.ir_right.height,
                           cfg_.ir_right.format == "y16" ? RS2_FORMAT_Y16 : RS2_FORMAT_Y8, cfg_.ir_right.fps);
    }
    running_ = true;
    frame_thread_ = std::thread(&RealSenseDevice::frame_loop, this);
    rs2::pipeline_profile profile;
    try {
      profile = pipe_.start(rs_cfg, [this](const rs2::frame& frame) { enqueue_frame(frame); });
    } catch (...) {
      running_ = false;
      frame_queue_.close();
      if (frame_thread_.joinable()) frame_thread_.join();
      throw;
    }
    apply_controls(profile);
  }

  void stop() override {
    if (!running_.exchange(false)) return;
    try { pipe_.stop(); } catch (...) {}
    frame_queue_.close();
    if (frame_thread_.joinable()) frame_thread_.join();
  }

  CameraCaptureStats capture_stats() const override {
    CameraCaptureStats out;
    out.queue_drop_count = queue_drop_count_.load(std::memory_order_relaxed);
    out.queue_depth = frame_queue_.size();
    const auto callback = callback_timing_.p95_max_us();
    const auto wait = queue_wait_timing_.p95_max_us();
    const auto process = process_timing_.p95_max_us();
    out.callback_enqueue_us_p95 = callback.first;
    out.callback_enqueue_us_max = callback.second;
    out.queue_wait_us_p95 = wait.first;
    out.queue_wait_us_max = wait.second;
    out.frame_process_us_p95 = process.first;
    out.frame_process_us_max = process.second;
    return out;
  }

 private:
  struct QueuedFrame {
    rs2::frame frame;
    uint64_t host_arrival_time_ns{0};
    uint64_t enqueue_steady_ns{0};
  };

  void enqueue_frame(const rs2::frame& frame) {
    if (!running_.load(std::memory_order_relaxed)) return;
    const uint64_t begin_ns = steady_now_ns();
    QueuedFrame queued{frame, now_ns(clock_), begin_ns};
    if (!frame_queue_.try_push_drop_oldest(std::move(queued))) {
      queue_drop_count_.fetch_add(1, std::memory_order_relaxed);
    }
    callback_timing_.record(steady_now_ns() - begin_ns);
  }

  void frame_loop() {
    while (true) {
      auto queued = frame_queue_.pop_wait();
      if (!queued) break;
      const uint64_t process_start_ns = steady_now_ns();
      queue_wait_timing_.record(process_start_ns - queued->enqueue_steady_ns);
      process_frame(queued->frame, queued->host_arrival_time_ns);
      process_timing_.record(steady_now_ns() - process_start_ns);
    }
  }

  // 수집(.40)과 동일한 rs400 advanced-mode JSON을 pipeline start 전에 적용한다.
  // 경로는 env CAMERA_SERVER_REALSENSE_JSON. 실패해도 캡처는 계속(드라이버 기본값).
  // JSON에 controls-autoexposure-auto=True가 있으면 노출은 그대로 auto 유지.
  void maybe_load_advanced_json() {
    const char* env = std::getenv("CAMERA_SERVER_REALSENSE_JSON");
    if (env == nullptr || env[0] == '\0') return;
    const std::string path(env);
    std::ifstream f(path);
    if (!f) {
      std::cerr << "[CAM] advanced json not found, skipping: " << path << '\n';
      return;
    }
    std::stringstream ss;
    ss << f.rdbuf();
    const std::string json_str = ss.str();
    if (json_str.empty()) {
      std::cerr << "[CAM] advanced json empty, skipping: " << path << '\n';
      return;
    }
    try {
      rs2::context ctx;
      rs2::device dev;
      bool found = false;
      for (auto&& d : ctx.query_devices()) {
        if (cfg_.serial == d.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)) {
          dev = d;
          found = true;
          break;
        }
      }
      if (!found) {
        std::cerr << "[CAM] advanced json: device serial=" << cfg_.serial << " not found, skipping\n";
        return;
      }
      auto adv = dev.as<rs400::advanced_mode>();
      if (!adv) {
        std::cerr << "[CAM] advanced json: " << cfg_.name << " does not support advanced mode, skipping\n";
        return;
      }
      int tries = 0;
      while (!adv.is_enabled() && tries < 5) {
        adv.toggle_advanced_mode(true);
        std::this_thread::sleep_for(std::chrono::seconds(5));  // advanced toggle -> USB 재열거
        rs2::context ctx2;
        for (auto&& d : ctx2.query_devices()) {
          if (cfg_.serial == d.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)) {
            dev = d;
            break;
          }
        }
        adv = dev.as<rs400::advanced_mode>();
        ++tries;
      }
      adv.load_json(json_str);
      std::cerr << "[CAM] advanced json applied for " << cfg_.name << " serial=" << cfg_.serial
                << ": " << path << '\n';
      enforce_depth_units(dev, json_str);
    } catch (const std::exception& e) {
      std::cerr << "[CAM] advanced json apply failed for " << cfg_.name << ": " << e.what() << '\n';
    }
  }

  // D405 FW에서 load_json 은 param-depthunits 를 무시하므로, .40 수집측과 동일하게
  // RS2_OPTION_DEPTH_UNITS 를 JSON 값으로 직접 강제한다(advanced param 은 µm).
  // 예) 100 -> 1e-4 m/LSB = 0.1mm -> max range ~6.55m. depth 비활성 시에도
  // 디바이스 옵션은 설정해두어 depth 사용 시 .40 과 동일 scale 을 보장한다.
  void enforce_depth_units(rs2::device& dev, const std::string& json_str) {
    const std::string key = "\"param-depthunits\"";
    auto p = json_str.find(key);
    if (p == std::string::npos) return;
    p = json_str.find(':', p + key.size());
    if (p == std::string::npos) return;
    size_t i = p + 1;
    while (i < json_str.size() &&
           (json_str[i] == ' ' || json_str[i] == '"' || json_str[i] == '\t')) ++i;
    size_t j = i;
    while (j < json_str.size() &&
           (std::isdigit(static_cast<unsigned char>(json_str[j])) || json_str[j] == '.')) ++j;
    if (j == i) return;
    double um = 0.0;
    try {
      um = std::stod(json_str.substr(i, j - i));
    } catch (...) {
      return;
    }
    const double depth_units_m = um * 1e-6;
    try {
      for (auto&& s : dev.query_sensors()) {
        if (!s.supports(RS2_OPTION_DEPTH_UNITS)) continue;
        auto rng = s.get_option_range(RS2_OPTION_DEPTH_UNITS);
        double v = std::min(std::max(depth_units_m, static_cast<double>(rng.min)),
                            static_cast<double>(rng.max));
        s.set_option(RS2_OPTION_DEPTH_UNITS, static_cast<float>(v));
        std::cerr << "[CAM] depth_units=" << v << " m (param-depthunits=" << um
                  << "um, max range~" << v * 65535.0 << " m) for " << cfg_.name << '\n';
        return;
      }
    } catch (const std::exception& e) {
      std::cerr << "[CAM] depth_units set failed for " << cfg_.name << ": " << e.what() << '\n';
    }
  }

  // pipeline start 직후: 센서 단위 제어(emitter/노출/게인) 적용.
  // emitter/노출은 스테레오 모듈 센서(=EMITTER_ENABLED 지원 센서)에만 적용해 color
  // 센서 노출과 분리한다. 미설정(-1) 필드는 건드리지 않는다.
  void apply_controls(const rs2::pipeline_profile& profile) {
    const auto& ctrl = cfg_.controls;
    try {
      auto dev = profile.get_device();
      for (auto&& s : dev.query_sensors()) {
        if (!s.supports(RS2_OPTION_EMITTER_ENABLED)) continue;  // stereo module only
        auto set = [&](rs2_option opt, float v, const char* nm) {
          if (!s.supports(opt)) {
            std::cerr << "[CAM] " << cfg_.name << ": option " << nm << " unsupported, skip\n";
            return;
          }
          try {
            s.set_option(opt, v);
            std::cerr << "[CAM] " << cfg_.name << ": " << nm << "=" << v << '\n';
          } catch (const std::exception& e) {
            std::cerr << "[CAM] " << cfg_.name << ": set " << nm << " failed: " << e.what() << '\n';
          }
        };
        // auto_exposure 먼저(수동 노출/게인 적용 전에 auto off 필요).
        if (ctrl.auto_exposure >= 0) set(RS2_OPTION_ENABLE_AUTO_EXPOSURE, static_cast<float>(ctrl.auto_exposure), "auto_exposure");
        if (ctrl.emitter_enabled >= 0) set(RS2_OPTION_EMITTER_ENABLED, static_cast<float>(ctrl.emitter_enabled), "emitter_enabled");
        if (ctrl.laser_power >= 0.0f) set(RS2_OPTION_LASER_POWER, ctrl.laser_power, "laser_power");
        if (ctrl.ir_exposure_us >= 0) set(RS2_OPTION_EXPOSURE, static_cast<float>(ctrl.ir_exposure_us), "ir_exposure_us");
        if (ctrl.ir_gain >= 0) set(RS2_OPTION_GAIN, static_cast<float>(ctrl.ir_gain), "ir_gain");
        break;
      }
    } catch (const std::exception& e) {
      std::cerr << "[CAM] controls apply failed for " << cfg_.name << ": " << e.what() << '\n';
    }
  }

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
    // Frame metadata is optional by device, stream, firmware, kernel/backend,
    // and auto-exposure mode. Follow librealsense's check-then-query contract;
    // never substitute a current sensor option for missing per-frame evidence.
    if (vf.supports_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE)) {
      f.actual_exposure_us =
          static_cast<double>(vf.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE));
    }
    if (vf.supports_frame_metadata(RS2_FRAME_METADATA_GAIN_LEVEL)) {
      f.gain_level =
          static_cast<double>(vf.get_frame_metadata(RS2_FRAME_METADATA_GAIN_LEVEL));
    }
    if (vf.supports_frame_metadata(RS2_FRAME_METADATA_AUTO_EXPOSURE)) {
      f.auto_exposure =
          vf.get_frame_metadata(RS2_FRAME_METADATA_AUTO_EXPOSURE) != 0;
    }
    f.width = static_cast<uint32_t>(vf.get_width());
    f.height = static_cast<uint32_t>(vf.get_height());
    f.stride_bytes = static_cast<uint32_t>(vf.get_stride_in_bytes());
    f.format = scfg.format;
    const size_t bytes = static_cast<size_t>(vf.get_height()) * static_cast<size_t>(vf.get_stride_in_bytes());
    f.data = static_cast<const uint8_t*>(vf.get_data());
    f.size_bytes = static_cast<uint32_t>(bytes);
    cb_(std::move(f));
  }

  void process_frame(const rs2::frame& frame, uint64_t host_t) {
    try {
      if (auto fs = frame.as<rs2::frameset>()) {
        if (cfg_.color.enabled) {
          auto color = fs.get_color_frame();
          if (color) on_video_frame(color, "color", cfg_.color, host_t);
        }
        if (cfg_.depth.enabled) {
          auto depth = fs.get_depth_frame();
          if (depth) on_video_frame(depth, "depth", cfg_.depth, host_t);
        }
        if (cfg_.ir_left.enabled) {
          auto ir = fs.get_infrared_frame(1);
          if (ir) on_video_frame(ir, "ir_left", cfg_.ir_left, host_t);
        }
        if (cfg_.ir_right.enabled) {
          auto ir = fs.get_infrared_frame(2);
          if (ir) on_video_frame(ir, "ir_right", cfg_.ir_right, host_t);
        }
      } else if (auto vf = frame.as<rs2::video_frame>()) {
        const auto profile = vf.get_profile().stream_type();
        if (profile == RS2_STREAM_COLOR && cfg_.color.enabled) on_video_frame(vf, "color", cfg_.color, host_t);
        if (profile == RS2_STREAM_DEPTH && cfg_.depth.enabled) on_video_frame(vf, "depth", cfg_.depth, host_t);
        if (profile == RS2_STREAM_INFRARED) {
          const int si = vf.get_profile().stream_index();
          if (si == 1 && cfg_.ir_left.enabled) on_video_frame(vf, "ir_left", cfg_.ir_left, host_t);
          else if (si == 2 && cfg_.ir_right.enabled) on_video_frame(vf, "ir_right", cfg_.ir_right, host_t);
        }
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
  BoundedQueue<QueuedFrame> frame_queue_{2};
  std::atomic<bool> running_{false};
  std::atomic<uint64_t> queue_drop_count_{0};
  std::thread frame_thread_;
  AtomicTimingWindow callback_timing_;
  AtomicTimingWindow queue_wait_timing_;
  AtomicTimingWindow process_timing_;
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
