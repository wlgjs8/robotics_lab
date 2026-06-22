#include "camera_server/camera/camera_manager.hpp"

#include "camera_server/core/clock.hpp"

#include <algorithm>
#include <iostream>
#include <set>
#include <stdexcept>

namespace camera_server {

CameraManager::CameraManager(const AppConfig& cfg, SharedMemoryRingBuffer& shm, MetadataPublisher& publisher,
                             FrameSynchronizer& synchronizer, Recorder& recorder)
    : cfg_(cfg), shm_(shm), publisher_(publisher), synchronizer_(synchronizer), recorder_(recorder),
      metadata_queue_(std::max<uint32_t>(32, cfg.recording.queue_capacity_frames)) {
  for (const auto& cam : cfg_.cameras) {
    connected_[cam.name] = false;
    if (cam.color.enabled) stats_[stream_key(cam.name, "color")] = {};
    if (cam.depth.enabled) stats_[stream_key(cam.name, "depth")] = {};
  }
}
CameraManager::~CameraManager() { stop(); }

void CameraManager::ensure_required_cameras_present() const {
  if (cfg_.server.simulate_cameras) return;
  // UVC fisheye cameras are matched by V4L2 device path, not serial: verify the
  // configured device node currently resolves.
  for (const auto& cam : cfg_.cameras) {
    if (cam.backend != "uvc") continue;
    if (cam.required && !uvc_device_present(cam.device)) {
      throw std::runtime_error("required uvc camera missing: " + cam.name + " device=" + cam.device +
                               " (not present on this host)");
    }
  }
#if CAMERA_SERVER_HAVE_REALSENSE
  const auto serials = discover_realsense_serials();
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
      throw std::runtime_error("required RealSense camera missing: " + cam.name + " serial=" + cam.serial +
                               " (connected serials: " + connected + ")");
    }
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
  ensure_required_cameras_present();
  if (cfg_.reconnect.enabled) {
    std::cerr << "[CAM] warning: reconnect.enabled is configured but reconnect is not implemented; "
                 "stale/disconnected cameras are reported via health only\n";
  }
  metadata_thread_ = std::thread(&CameraManager::metadata_loop, this);
  for (const auto& cam : cfg_.cameras) {
    std::unique_ptr<ICameraDevice> dev;
    const char* backend_label;
    if (cfg_.server.simulate_cameras) {
      dev = make_mock_camera_device(cam, cfg_.server.clock);
      backend_label = " (mock)";
    } else if (cam.backend == "uvc") {
      dev = make_uvc_device(cam, cfg_.server.clock);
      backend_label = " (uvc)";
    } else {
      dev = make_realsense_device(cam, cfg_.server.clock, cfg_.sync);
      backend_label = " (RealSense)";
    }
    {
      std::lock_guard<std::mutex> lk(mu_);
      connected_[cam.name] = true;
    }
    dev->start([this](CapturedFrame&& f) { handle_frame(std::move(f)); });
    devices_.push_back(std::move(dev));
    std::cerr << "[CAM] started " << cam.name << " serial=" << cam.serial << backend_label << '\n';
  }
}

void CameraManager::stop() {
  if (!running_.exchange(false)) return;
  for (auto& dev : devices_) dev->stop();
  devices_.clear();
  metadata_queue_.close();
  if (metadata_thread_.joinable()) metadata_thread_.join();
  std::lock_guard<std::mutex> lk(mu_);
  for (auto& [_, v] : connected_) v = false;
}

void CameraManager::handle_frame(CapturedFrame&& frame) {
  const auto key = stream_key(frame.camera_name, frame.stream);
  try {
    const uint8_t* data = frame.data ? frame.data : frame.bytes.data();
    const uint32_t size_bytes = frame.size_bytes != 0 ? frame.size_bytes : static_cast<uint32_t>(frame.bytes.size());
    if (!data || size_bytes == 0) throw std::runtime_error("captured frame has no image payload");
    auto meta = shm_.write_frame(frame.camera_name, frame.serial, frame.stream, frame.frame_number,
                                 frame.host_arrival_time_ns, frame.sensor_timestamp_ns,
                                 frame.realsense_timestamp_ms, frame.stride_bytes, data, size_bytes);
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
    ++st.frame_count;
    ++st.shared_memory_write_count;
    const double elapsed = static_cast<double>(frame.meta.host_arrival_time_ns - start_time_ns_) / 1e9;
    if (elapsed > 0.0) st.fps_estimate = static_cast<double>(st.frame_count) / elapsed;
    for (const auto& [k, s] : stats_) {
      drops[k] = s.frame_number_gap_drop_count + s.internal_queue_drop_count + s.recorder_drop_count;
    }
  }
  auto bundle = synchronizer_.push_frame(frame.meta, drops);
  if (bundle) publisher_.publish_bundle(*bundle);
}

HealthSnapshot CameraManager::snapshot() const {
  std::lock_guard<std::mutex> lk(mu_);
  HealthSnapshot h;
  h.host_time_ns = now_ns(cfg_.server.clock);
  h.uptime_sec = start_time_ns_ == 0 ? 0.0 : static_cast<double>(h.host_time_ns - start_time_ns_) / 1e9;
  h.mode = cfg_.server.mode;
  h.camera_connected = connected_;
  for (const auto& cam : cfg_.cameras) h.camera_serial[cam.name] = cam.serial;
  h.stream_stats = stats_;
  h.bundle_seq = synchronizer_.bundle_seq();
  h.complete_bundle_count = synchronizer_.complete_bundle_count();
  h.incomplete_bundle_count = synchronizer_.incomplete_bundle_count();
  h.max_time_diff_ms = synchronizer_.last_max_time_diff_ms();
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
