#include "camera_server/camera/camera_manager.hpp"

#include "camera_server/core/clock.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <set>
#include <stdexcept>

namespace camera_server {

CameraManager::CameraManager(const AppConfig& cfg, SharedMemoryRingBuffer& shm, MetadataPublisher& publisher,
                             FrameSynchronizerSet& synchronizer, Recorder& recorder,
                             DeviceFactory device_factory)
    : cfg_(cfg), shm_(shm), publisher_(publisher), synchronizer_(synchronizer), recorder_(recorder),
      device_factory_(std::move(device_factory)),
      metadata_queue_(std::max<uint32_t>(32, cfg.recording.queue_capacity_frames)) {
  for (const auto& cam : cfg_.cameras) {
    connected_[cam.name] = false;
    reconnect_stats_[cam.name] = {};
    camera_runtimes_.push_back(std::make_unique<CameraRuntime>(cam));
    for (const auto& [name, scfg] : enabled_streams(cam)) {
      (void)scfg;
      stats_[stream_key(cam.name, name)] = {};
    }
  }
}
CameraManager::~CameraManager() { stop(); }

void CameraManager::ensure_required_cameras_present() {
  if (cfg_.server.simulate_cameras) return;
  // UVC fisheye cameras are matched by V4L2 device path, not serial: verify the
  // configured device node currently resolves.
  for (const auto& cam : cfg_.cameras) {
    if (cam.backend != "uvc") continue;
    if (cam.required && !uvc_device_present(cam.device)) {
      const std::string message = "required uvc camera missing: " + cam.name + " device=" + cam.device +
                                  " (not present on this host)";
      if (!cfg_.reconnect.enabled) throw std::runtime_error(message);
      std::cerr << "[CAM] " << message << "; per-camera reconnect will keep probing\n";
    }
  }
#if CAMERA_SERVER_HAVE_REALSENSE
  const auto devices = discover_realsense_devices();
  std::vector<std::string> serials;
  std::map<std::string, RealSenseDeviceInfo> discovered_info;
  for (const auto& device : devices) {
    serials.push_back(device.serial);
    discovered_info[device.serial] = device;
  }
  realsense_sdk_version_ = librealsense_sdk_version();
  realsense_backend_ = librealsense_backend();
  std::set<std::string> present(serials.begin(), serials.end());
  for (const auto& cam : cfg_.cameras) {
    if (cam.backend == "uvc") continue;
    if (cam.required && present.count(cam.serial) == 0) {
      std::string connected = "none";
      if (!serials.empty()) {
        connected.clear();
        for (const auto& serial : serials) {
          if (!connected.empty()) connected += ", ";
          connected += serial;
        }
      }
      const std::string message = "required RealSense camera missing: " + cam.name + " serial=" + cam.serial +
                                  " (connected serials: " + connected + ")";
      if (!cfg_.reconnect.enabled) throw std::runtime_error(message);
      std::cerr << "[CAM] " << message << "; per-camera reconnect will keep probing\n";
    }
  }
  validate_realsense_preflight(cfg_, devices, realsense_sdk_version_);
  std::cerr << "[CAM] librealsense sdk=" << realsense_sdk_version_
            << " backend=" << realsense_backend_ << '\n';
  for (const auto& cam : cfg_.cameras) {
    if (cam.backend != "realsense") continue;
    const auto found = discovered_info.find(cam.serial);
    if (found == discovered_info.end()) continue;
    const auto& device = found->second;
    std::cerr << "[CAM] device " << cam.name << " serial=" << device.serial
              << " product_id=" << device.product_id << " firmware=" << device.firmware_version
              << " recommended=" << device.recommended_firmware_version
              << " usb=" << device.usb_type << " port=" << device.physical_port << '\n';
  }
#else
  for (const auto& cam : cfg_.cameras) {
    if (cam.backend == "uvc") continue;  // checked by device-path probe above
    if (cam.required) throw std::runtime_error("RealSense backend not compiled; set server.simulate_cameras=true for mock runs");
  }
#endif
}

void CameraManager::start() {
  if (running_.exchange(true)) return;
  start_time_ns_ = now_ns(cfg_.server.clock);
  try {
    ensure_required_cameras_present();
    metadata_thread_ = std::thread(&CameraManager::metadata_loop, this);
    if (cfg_.reconnect.enabled) {
      std::cerr << "[CAM] per-camera reconnect enabled retry_interval_ms="
                << cfg_.reconnect.retry_interval_ms << " frame_timeout_ms="
                << cfg_.reconnect.frame_timeout_ms << " max_attempts="
                << cfg_.reconnect.max_attempts << " (0=unlimited)\n";
      for (auto& runtime : camera_runtimes_) {
        runtime->supervisor_thread = std::thread(&CameraManager::supervise_camera, this,
                                                 std::ref(*runtime));
        // Opening several librealsense pipelines concurrently can make devices
        // sharing a controller fail USB negotiation. Preserve the previously
        // stable sequential startup while keeping independent supervisors after
        // startup. A missing camera consumes only its configured timeout.
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::milliseconds(cfg_.reconnect.frame_timeout_ms);
        while (running_ && std::chrono::steady_clock::now() < deadline) {
          bool connected = false;
          {
            std::lock_guard<std::mutex> lk(mu_);
            connected = connected_[runtime->cfg.name];
          }
          if (connected) break;
          std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
      }
      return;
    }
    for (auto& runtime : camera_runtimes_) {
      if (!running_) break;
      std::optional<RealSenseDeviceInfo> device_info;
      {
        std::lock_guard<std::mutex> lifecycle_lk(device_lifecycle_mu_);
        auto dev = make_device(runtime->cfg);
        dev->start([this](CapturedFrame&& f) { handle_frame(std::move(f)); });
        device_info = validated_device_info(runtime->cfg, *dev);
        std::lock_guard<std::mutex> device_lk(runtime->device_mu);
        runtime->device = std::move(dev);
      }
      {
        std::lock_guard<std::mutex> lk(mu_);
        if (!running_) break;
        if (device_info) realsense_device_info_[runtime->cfg.serial] = *device_info;
        else realsense_device_info_.erase(runtime->cfg.serial);
        connected_[runtime->cfg.name] = true;
      }
      std::cerr << "[CAM] started " << runtime->cfg.name << " serial="
                << runtime->cfg.serial << '\n';
    }
  } catch (...) {
    running_ = false;
    {
      std::lock_guard<std::mutex> lk(mu_);
      for (auto& [_, connected] : connected_) connected = false;
      realsense_device_info_.clear();
    }
    for (auto& runtime : camera_runtimes_) {
      std::lock_guard<std::mutex> lifecycle_lk(device_lifecycle_mu_);
      std::lock_guard<std::mutex> lk(runtime->device_mu);
      if (runtime->device) runtime->device->stop();
      runtime->device.reset();
    }
    metadata_queue_.close();
    if (metadata_thread_.joinable()) metadata_thread_.join();
    throw;
  }
}

void CameraManager::stop() {
  if (!running_.exchange(false)) return;
  {
    std::lock_guard<std::mutex> lk(mu_);
    for (auto& [_, connected] : connected_) connected = false;
    realsense_device_info_.clear();
  }
  for (auto& runtime : camera_runtimes_) runtime->wake_cv.notify_all();
  for (auto& runtime : camera_runtimes_) {
    if (runtime->supervisor_thread.joinable()) runtime->supervisor_thread.join();
    std::lock_guard<std::mutex> lifecycle_lk(device_lifecycle_mu_);
    std::lock_guard<std::mutex> lk(runtime->device_mu);
    if (runtime->device) runtime->device->stop();
    runtime->device.reset();
  }
  metadata_queue_.close();
  if (metadata_thread_.joinable()) metadata_thread_.join();
  std::lock_guard<std::mutex> lk(mu_);
  for (auto& [_, connected] : connected_) connected = false;
  realsense_device_info_.clear();
}

std::unique_ptr<ICameraDevice> CameraManager::make_device(const CameraConfig& cam) const {
  if (device_factory_) return device_factory_(cam);
  if (cfg_.server.simulate_cameras) return make_mock_camera_device(cam, cfg_.server.clock);
  if (cam.backend == "uvc") return make_uvc_device(cam, cfg_.server.clock);
  return make_realsense_device(cam, cfg_.server.clock, cfg_.sync);
}

std::optional<RealSenseDeviceInfo> CameraManager::validated_device_info(
    const CameraConfig& cam, const ICameraDevice& device) const {
  const auto info = device.active_device_info();
  if (info && (info->serial.empty() || info->serial != cam.serial)) {
    throw std::runtime_error("active camera serial mismatch for " + cam.name +
                             ": expected=" + cam.serial + " actual=" + info->serial);
  }
  if (!info && cam.backend == "realsense" && !cfg_.server.simulate_cameras) {
    throw std::runtime_error("active RealSense device info unavailable for " + cam.name);
  }
  return info;
}

bool CameraManager::wait_or_stopping(CameraRuntime& runtime,
                                     std::chrono::milliseconds duration) {
  std::unique_lock<std::mutex> lk(runtime.wake_mu);
  return runtime.wake_cv.wait_for(lk, duration, [this] { return !running_.load(); });
}

void CameraManager::supervise_camera(CameraRuntime& runtime) {
  const auto retry_delay = std::chrono::milliseconds(cfg_.reconnect.retry_interval_ms);
  const uint64_t timeout_ns = static_cast<uint64_t>(cfg_.reconnect.frame_timeout_ms) * 1000000ull;
  uint32_t retries_used = 0;
  bool retry = false;
  bool recovery_pending = false;

  while (running_) {
    if (retry) {
      if (cfg_.reconnect.max_attempts != 0 && retries_used >= cfg_.reconnect.max_attempts) {
        std::lock_guard<std::mutex> lk(mu_);
        auto& stats = reconnect_stats_[runtime.cfg.name];
        stats.exhausted = true;
        stats.consecutive_failures = retries_used;
        std::cerr << "[CAM] reconnect exhausted " << runtime.cfg.name
                  << " attempts=" << retries_used << '\n';
        break;
      }
      if (wait_or_stopping(runtime, retry_delay)) break;
      ++retries_used;
      {
        std::lock_guard<std::mutex> lk(mu_);
        auto& stats = reconnect_stats_[runtime.cfg.name];
        ++stats.attempt_count;
        stats.consecutive_failures = retries_used;
        stats.exhausted = false;
      }
    }

    runtime.last_frame_time_ns = 0;
    {
      std::lock_guard<std::mutex> lk(mu_);
      connected_[runtime.cfg.name] = false;
      realsense_device_info_.erase(runtime.cfg.serial);
    }
    const uint64_t start_ns = now_ns(cfg_.server.clock);
    try {
      std::optional<RealSenseDeviceInfo> device_info;
      CameraConfig attempt_cfg = runtime.cfg;
      if (retry && attempt_cfg.backend == "realsense") {
        // Sensor controls and intrinsics are persistent for the device session
        // and were already applied on initial startup. Reissuing XU controls
        // while a USB device is recovering can block for many seconds and delay
        // the frame-timeout state machine, so reconnect only restores streams.
        attempt_cfg.controls = {};
      }
      {
        std::lock_guard<std::mutex> lifecycle_lk(device_lifecycle_mu_);
        auto dev = make_device(attempt_cfg);
        dev->start([this](CapturedFrame&& frame) { handle_frame(std::move(frame)); });
        device_info = validated_device_info(runtime.cfg, *dev);
        std::lock_guard<std::mutex> device_lk(runtime.device_mu);
        runtime.device = std::move(dev);
      }
      std::cerr << "[CAM] pipeline started " << runtime.cfg.name << " serial="
                << runtime.cfg.serial << (retry ? " (reconnect attempt)" : "") << '\n';

      bool reported_connected = false;
      bool timed_out = false;
      while (running_) {
        if (wait_or_stopping(runtime, std::chrono::milliseconds(50))) break;
        const uint64_t last_frame_ns = runtime.last_frame_time_ns.load(std::memory_order_relaxed);
        if (last_frame_ns != 0 && !reported_connected) {
          const bool reconnected = recovery_pending;
          size_t reset_bundle_groups = 0;
          if (reconnected) {
            // The first frame proves that a new device pipeline is producing
            // data. Reset affected synchronizers before declaring the camera
            // connected so a restarted master frame counter cannot leave a
            // per-camera or policy bundle permanently stalled.
            reset_bundle_groups = synchronizer_.reset_camera_generation(runtime.cfg.name);
          }
          reported_connected = true;
          {
            std::lock_guard<std::mutex> lk(mu_);
            // Publish identity from this opened pipeline atomically with its
            // first-frame readiness; discovery before start can be stale after
            // advanced-mode toggling or USB disconnect/re-enumeration.
            if (!running_) break;
            if (device_info) realsense_device_info_[runtime.cfg.serial] = *device_info;
            else realsense_device_info_.erase(runtime.cfg.serial);
            connected_[runtime.cfg.name] = true;
            auto& stats = reconnect_stats_[runtime.cfg.name];
            stats.consecutive_failures = 0;
            stats.exhausted = false;
            if (recovery_pending) {
              ++stats.success_count;
              stats.last_reconnect_time_ns = last_frame_ns;
            }
          }
          retries_used = 0;
          retry = false;
          recovery_pending = false;
          std::cerr << "[CAM] streaming " << runtime.cfg.name
                    << (reconnected ? " (reconnected)" : "");
          if (reconnected) std::cerr << " reset_bundle_groups=" << reset_bundle_groups;
          if (device_info) std::cerr << " port=" << device_info->physical_port;
          std::cerr << '\n';
        }
        const uint64_t reference_ns = last_frame_ns == 0 ? start_ns : last_frame_ns;
        const uint64_t current_ns = now_ns(cfg_.server.clock);
        if (current_ns > reference_ns && current_ns - reference_ns > timeout_ns) {
          timed_out = true;
          recovery_pending = true;
          {
            std::lock_guard<std::mutex> lk(mu_);
            connected_[runtime.cfg.name] = false;
            realsense_device_info_.erase(runtime.cfg.serial);
            auto& stats = reconnect_stats_[runtime.cfg.name];
            ++stats.disconnect_count;
            stats.last_disconnect_time_ns = current_ns;
            stats.last_error = "frame timeout after " +
                               std::to_string(cfg_.reconnect.frame_timeout_ms) + " ms";
          }
          std::cerr << "[CAM] disconnected " << runtime.cfg.name
                    << ": frame timeout; restarting only this camera\n";
          break;
        }
      }
      {
        std::lock_guard<std::mutex> lifecycle_lk(device_lifecycle_mu_);
        std::lock_guard<std::mutex> lk(runtime.device_mu);
        if (runtime.device) runtime.device->stop();
        runtime.device.reset();
      }
      if (!running_) break;
      retry = timed_out;
    } catch (const std::exception& e) {
      {
        std::lock_guard<std::mutex> lk(mu_);
        connected_[runtime.cfg.name] = false;
        realsense_device_info_.erase(runtime.cfg.serial);
        auto& stats = reconnect_stats_[runtime.cfg.name];
        stats.last_error = e.what();
        stats.last_disconnect_time_ns = now_ns(cfg_.server.clock);
      }
      {
        std::lock_guard<std::mutex> lifecycle_lk(device_lifecycle_mu_);
        std::lock_guard<std::mutex> lk(runtime.device_mu);
        if (runtime.device) runtime.device->stop();
        runtime.device.reset();
      }
      recovery_pending = true;
      retry = true;
      std::cerr << "[CAM] pipeline start failed " << runtime.cfg.name << ": "
                << e.what() << "; retrying only this camera\n";
    }
  }
}

void CameraManager::handle_frame(CapturedFrame&& frame) {
  for (const auto& runtime : camera_runtimes_) {
    if (runtime->cfg.name == frame.camera_name) {
      runtime->last_frame_time_ns.store(frame.host_arrival_time_ns, std::memory_order_relaxed);
      break;
    }
  }
  const auto key = stream_key(frame.camera_name, frame.stream);
  try {
    const uint8_t* data = frame.data ? frame.data : frame.bytes.data();
    const uint32_t size_bytes = frame.size_bytes != 0 ? frame.size_bytes : static_cast<uint32_t>(frame.bytes.size());
    if (!data || size_bytes == 0) throw std::runtime_error("captured frame has no image payload");
    auto meta = shm_.write_frame(frame.camera_name, frame.serial, frame.stream, frame.frame_number,
                                 frame.host_arrival_time_ns, frame.sensor_timestamp_ns,
                                 frame.realsense_timestamp_ms, frame.stride_bytes, data, size_bytes);
    meta.actual_exposure_us = frame.actual_exposure_us;
    meta.gain_level = frame.gain_level;
    meta.auto_exposure = frame.auto_exposure;
    if (recorder_.enabled()) {
      bool accepted = recorder_.enqueue_copy(meta, data, size_bytes);
      if (!accepted) {
        std::lock_guard<std::mutex> lk(mu_);
        stats_[key].recorder_drop_count++;
      }
    }
    if (!metadata_queue_.try_push_drop_oldest(ProcessedFrame{meta})) {
      std::lock_guard<std::mutex> lk(mu_);
      stats_[key].internal_queue_drop_count++;
    }
  } catch (const std::exception& e) {
    std::lock_guard<std::mutex> lk(mu_);
    stats_[key].shared_memory_write_errors++;
    std::cerr << "[CAM] frame handling error for " << key << ": " << e.what() << '\n';
  }
}

void CameraManager::metadata_loop() {
  while (true) {
    auto item = metadata_queue_.pop_wait();
    if (!item) break;
    process_frame_metadata(std::move(*item));
  }
}

void CameraManager::process_frame_metadata(ProcessedFrame&& frame) {
  const auto key = frame.meta.ring_name;
  std::map<std::string, uint64_t> drops;
  {
    std::lock_guard<std::mutex> lk(mu_);
    auto& st = stats_[key];
    if (st.last_frame_number != 0 && frame.meta.frame_number > st.last_frame_number + 1) {
      st.frame_number_gap_drop_count += frame.meta.frame_number - st.last_frame_number - 1;
    }
    st.last_frame_number = frame.meta.frame_number;
    st.last_frame_time_ns = frame.meta.host_arrival_time_ns;
    if (st.first_frame_time_ns == 0) st.first_frame_time_ns = frame.meta.host_arrival_time_ns;
    ++st.frame_count;
    ++st.shared_memory_write_count;
    const double elapsed = static_cast<double>(frame.meta.host_arrival_time_ns - st.first_frame_time_ns) / 1e9;
    if (elapsed > 0.0 && st.frame_count > 1) {
      st.fps_estimate = static_cast<double>(st.frame_count - 1) / elapsed;
    }
    auto& times = frame_time_windows_[key];
    times.push_back(frame.meta.host_arrival_time_ns);
    const uint64_t window_ns = static_cast<uint64_t>(cfg_.health.fps_window_sec * 1e9);
    while (!times.empty() && frame.meta.host_arrival_time_ns > times.front() &&
           frame.meta.host_arrival_time_ns - times.front() > window_ns) {
      times.pop_front();
    }
    if (times.size() >= 2) {
      const double window_elapsed = static_cast<double>(times.back() - times.front()) / 1e9;
      if (window_elapsed > 0.0) st.fps_window_hz = static_cast<double>(times.size() - 1) / window_elapsed;
    }
    for (const auto& [k, s] : stats_) {
      drops[k] = s.frame_number_gap_drop_count + s.internal_queue_drop_count + s.recorder_drop_count;
    }
  }
  for (auto& published : synchronizer_.push_frame(frame.meta, drops)) {
    publisher_.publish_bundle(published.topic, published.bundle);
  }
}

HealthSnapshot CameraManager::snapshot() const {
  HealthSnapshot h;
  // Capture diagnostics first and never wait for a device undergoing teardown.
  // Identity must be copied afterward: copying connected/routing and then
  // waiting for device_mu could return an old connected=true path after stop.
  // Do not nest device_mu and mu_: stop joins a frame thread that may need mu_.
  for (const auto& runtime : camera_runtimes_) {
    std::unique_lock<std::mutex> device_lk(runtime->device_mu, std::try_to_lock);
    if (device_lk.owns_lock() && runtime->device) {
      h.camera_capture_stats[runtime->cfg.name] = runtime->device->capture_stats();
    }
  }
  std::lock_guard<std::mutex> lk(mu_);
  h.host_time_ns = now_ns(cfg_.server.clock);
  h.uptime_sec = start_time_ns_ == 0 ? 0.0 : static_cast<double>(h.host_time_ns - start_time_ns_) / 1e9;
  h.mode = cfg_.server.mode;
  h.camera_connected = connected_;
  h.camera_reconnect_stats = reconnect_stats_;
  h.realsense_sdk_version = cfg_.server.simulate_cameras ? "mock" : realsense_sdk_version_;
  h.realsense_backend = cfg_.server.simulate_cameras ? "mock" : realsense_backend_;
  for (const auto& cam : cfg_.cameras) {
    h.camera_serial[cam.name] = cam.serial;
    const auto found = realsense_device_info_.find(cam.serial);
    if (found == realsense_device_info_.end()) continue;
    const auto& device = found->second;
    h.camera_firmware_version[cam.name] = device.firmware_version;
    h.camera_recommended_firmware_version[cam.name] = device.recommended_firmware_version;
    h.camera_physical_port[cam.name] = device.physical_port;
    h.camera_product_id[cam.name] = device.product_id;
    h.camera_usb_type[cam.name] = device.usb_type;
  }
  h.stream_stats = stats_;
  // fps_window_hz is only recomputed when a frame arrives, so a stalled stream
  // would report its last (healthy-looking) rate forever. Zero it once a full
  // fps window has elapsed with no frame; last_frame_time_ns and host_time_ns
  // share cfg_.server.clock (see apply_health_thresholds age check).
  const uint64_t fps_window_ns = static_cast<uint64_t>(cfg_.health.fps_window_sec * 1e9);
  for (auto& [stream_key, st] : h.stream_stats) {
    if (st.last_frame_time_ns != 0 && h.host_time_ns > st.last_frame_time_ns &&
        h.host_time_ns - st.last_frame_time_ns > fps_window_ns) {
      st.fps_window_hz = 0.0;
    }
  }
  h.bundle_groups = synchronizer_.stats();
  const auto compat = synchronizer_.compatibility_stats();
  h.bundle_seq = compat.bundle_seq;
  h.complete_bundle_count = compat.complete_bundle_count;
  h.incomplete_bundle_count = compat.dropped_master_count;
  h.max_time_diff_ms = compat.last_skew_ms;
  h.shm_name = shm_.name();
  h.shm_size_bytes = shm_.size_bytes();
  h.metadata_publish_count = publisher_.publish_count();
  h.metadata_publish_errors = publisher_.error_count();
  h.recorder_enabled = recorder_.enabled();
  h.recorder_queue_depth = recorder_.queue_depth();
  h.recorder_dropped_by_queue = recorder_.dropped_by_queue();
  return h;
}

}  // namespace camera_server
