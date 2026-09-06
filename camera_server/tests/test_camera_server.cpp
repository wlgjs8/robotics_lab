#include "camera_server/camera/realsense_device.hpp"
#include "camera_server/camera/camera_manager.hpp"
#include "camera_server/config/config.hpp"
#include "camera_server/core/bounded_queue.hpp"
#include "camera_server/core/clock.hpp"
#include "camera_server/health/health_monitor.hpp"
#include "camera_server/publish/metadata_publisher.hpp"
#include "camera_server/recording/recorder.hpp"
#include "camera_server/shm/shared_memory_ring.hpp"
#include "camera_server/shm/shm_layout.hpp"
#include "camera_server/sync/frame_synchronizer.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cassert>
#include <atomic>
#include <chrono>
#include <cstring>
#include <exception>
#include <filesystem>
#include <functional>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>
#include <thread>

using namespace camera_server;

struct MappedShm {
  int fd{-1};
  void* data{nullptr};
  size_t size{0};

  MappedShm(const std::string& name, size_t bytes) : size(bytes) {
    fd = shm_open(name.c_str(), O_RDWR, 0666);
    assert(fd >= 0);
    data = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    assert(data != MAP_FAILED);
  }

  ~MappedShm() {
    if (data && data != MAP_FAILED) munmap(data, size);
    if (fd >= 0) ::close(fd);
  }
};

AppConfig make_test_config() {
  AppConfig cfg;
  cfg.server.name = "camera_server";
  cfg.server.mode = "diagnostic";
  cfg.server.simulate_cameras = true;
  cfg.shared_memory.name = "/camera_server_unit_test";
  cfg.shared_memory.size_mb = 16;
  cfg.shared_memory.ring_slots = 4;
  cfg.metadata.pub_bind = "tcp://127.0.0.1:5999";
  cfg.sync.master_camera = "head";
  cfg.sync.max_bundle_time_diff_ms = 20.0;
  cfg.cameras.clear();
  for (auto name : {"head", "left_wrist", "right_wrist"}) {
    CameraConfig cam;
    cam.name = name;
    cam.serial = std::string("MOCK_") + name;
    cam.required = true;
    cam.color.enabled = true;
    cam.color.width = 8;
    cam.color.height = 4;
    cam.color.fps = 30;
    cam.color.format = "rgb8";
    cam.depth.enabled = false;
    cfg.cameras.push_back(cam);
  }
  validate_config(cfg);
  return cfg;
}

void expect_config_failure(const std::function<void()>& fn, const std::string& expected) {
  bool threw = false;
  try {
    fn();
  } catch (const std::runtime_error& e) {
    threw = true;
    assert(std::string(e.what()).find(expected) != std::string::npos);
  }
  assert(threw);
}

void test_config() {
  std::string path = "config/mock_triple_realsense.yaml";
  if (!std::ifstream(path).good()) path = "../config/mock_triple_realsense.yaml";
  auto cfg = load_config(path);
  assert(cfg.server.name == "camera_server");
  assert(cfg.server.simulate_cameras);
  assert(cfg.cameras.size() == 3);
  assert(required_stream_keys(cfg).size() == 3);
  assert(required_shared_memory_bytes(cfg) < cfg.shared_memory.size_mb * 1024ull * 1024ull);
}

void test_priority_bundle_config() {
  std::string path = "config/d435_head_1280x720.yaml";
  if (!std::ifstream(path).good()) path = "../config/d435_head_1280x720.yaml";
  const auto cfg = load_config(path);
  const auto groups = effective_bundle_groups(cfg);
  assert(groups.size() == 5);
  assert(groups[0].topic == "camera.bundle");
  assert(groups[1].topic == "camera.bundle.policy");
  assert(groups[1].required_streams.size() == 4);
  assert(groups[2].topic == "camera.bundle.wrist_left");
  assert(groups[2].required_streams.size() == 2);
  assert(groups[3].topic == "camera.bundle.wrist_right");
  assert(groups[3].required_streams.size() == 2);
  assert(groups[4].topic == "camera.bundle.stereo");
  assert(groups[4].required_streams.size() == 3);
  assert(cfg.health.fps_window_sec == 5.0);
  assert(cfg.health.warn_if_fps_below == 29.0);
}

void test_config_validation_rejects_real_serial_placeholders() {
  auto cfg = make_test_config();
  cfg.server.simulate_cameras = false;
  cfg.cameras[0].serial = "1234567890";
  cfg.cameras[1].serial = "1234567891";
  cfg.cameras[2].serial = "1234567892";
  validate_config(cfg);

  auto empty_serial = cfg;
  empty_serial.cameras[0].serial = "";
  expect_config_failure([&] { validate_config(empty_serial); }, "empty serial");

  const std::vector<std::string> placeholders = {"REPLACE_HEAD_SERIAL", "TODO", "CHANGEME", "UNKNOWN"};
  for (const auto& placeholder : placeholders) {
    auto bad = cfg;
    bad.cameras[0].serial = placeholder;
    expect_config_failure([&] { validate_config(bad); }, "invalid serial placeholder");
  }

  auto mock_serial_in_real_mode = cfg;
  mock_serial_in_real_mode.cameras[0].serial = "MOCK_HEAD";
  expect_config_failure([&] { validate_config(mock_serial_in_real_mode); }, "MOCK_* serial");
}

void test_config_validation_rejects_invalid_sync_combinations() {
  auto cfg = make_test_config();
  cfg.server.simulate_cameras = false;
  cfg.cameras[0].serial = "1234567890";
  cfg.cameras[1].serial = "1234567891";
  cfg.cameras[2].serial = "1234567892";

  auto software_frame_number = cfg;
  software_frame_number.sync.mode = "software";
  software_frame_number.sync.bundle_policy = "frame_number";
  expect_config_failure([&] { validate_config(software_frame_number); }, "sync.mode=software");

  auto hardware_nearest_timestamp = cfg;
  hardware_nearest_timestamp.sync.mode = "hardware";
  hardware_nearest_timestamp.sync.bundle_policy = "nearest_timestamp";
  expect_config_failure([&] { validate_config(hardware_nearest_timestamp); }, "sync.mode=hardware");
}

void test_config_validation_accepts_bounded_reconnect() {
  auto cfg = make_test_config();
  cfg.reconnect.enabled = true;
  cfg.reconnect.max_attempts = 5;
  cfg.reconnect.retry_interval_ms = 20;
  cfg.reconnect.frame_timeout_ms = 100;
  validate_config(cfg);

  auto zero_interval = cfg;
  zero_interval.reconnect.retry_interval_ms = 0;
  expect_config_failure([&] { validate_config(zero_interval); }, "retry_interval_ms must be > 0");
  auto short_timeout = cfg;
  short_timeout.reconnect.frame_timeout_ms = 99;
  expect_config_failure([&] { validate_config(short_timeout); }, "frame_timeout_ms must be >= 100");
}

struct TestDeviceStopGate {
  std::atomic<bool> entered{false};
  std::atomic<bool> release{false};
};

class ReconnectTestDevice final : public ICameraDevice {
 public:
  ReconnectTestDevice(CameraConfig cfg, int frames_before_stall,
                      std::optional<RealSenseDeviceInfo> info = std::nullopt,
                      bool fail_start = false,
                      std::shared_ptr<std::atomic<bool>> frames_allowed = {},
                      std::shared_ptr<TestDeviceStopGate> stop_gate = {})
      : cfg_(std::move(cfg)), frames_before_stall_(frames_before_stall),
        info_(std::move(info)), fail_start_(fail_start),
        frames_allowed_(std::move(frames_allowed)), stop_gate_(std::move(stop_gate)) {}
  ~ReconnectTestDevice() override { stop(); }

  void start(FrameCallback cb) override {
    if (fail_start_) throw std::runtime_error("synthetic pipeline start failure");
    cb_ = std::move(cb);
    running_ = true;
    thread_ = std::thread([this] {
      uint64_t frame_number = 0;
      while (running_) {
        if (frames_allowed_ && !frames_allowed_->load()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
          continue;
        }
        if (frames_before_stall_ >= 0 &&
            frame_number >= static_cast<uint64_t>(frames_before_stall_)) {
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
          continue;
        }
        CapturedFrame frame;
        frame.camera_name = cfg_.name;
        frame.serial = cfg_.serial;
        frame.stream = "color";
        frame.frame_number = ++frame_number;
        frame.host_arrival_time_ns = now_ns();
        frame.sensor_timestamp_ns = frame.host_arrival_time_ns;
        frame.width = static_cast<uint32_t>(cfg_.color.width);
        frame.height = static_cast<uint32_t>(cfg_.color.height);
        frame.stride_bytes = frame.width * 3;
        frame.format = "rgb8";
        frame.bytes.assign(static_cast<size_t>(frame.stride_bytes) * frame.height, 7);
        frame.data = frame.bytes.data();
        frame.size_bytes = static_cast<uint32_t>(frame.bytes.size());
        cb_(std::move(frame));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    });
  }

  void stop() override {
    const bool was_running = running_.exchange(false);
    if (thread_.joinable()) thread_.join();
    if (was_running && stop_gate_) {
      stop_gate_->entered = true;
      while (!stop_gate_->release) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    }
  }

  std::optional<RealSenseDeviceInfo> active_device_info() const override {
    return running_.load() ? info_ : std::nullopt;
  }

 private:
  CameraConfig cfg_;
  int frames_before_stall_;
  std::optional<RealSenseDeviceInfo> info_;
  bool fail_start_;
  std::shared_ptr<std::atomic<bool>> frames_allowed_;
  std::shared_ptr<TestDeviceStopGate> stop_gate_;
  FrameCallback cb_;
  std::atomic<bool> running_{false};
  std::thread thread_;
};

void test_per_camera_reconnect_keeps_other_camera_streaming() {
  auto cfg = make_test_config();
  cfg.cameras.resize(2);
  cfg.shared_memory.name = "/camera_server_reconnect_test";
  cfg.metadata.pub_bind = "tcp://127.0.0.1:5998";
  cfg.sync.master_camera = cfg.cameras[0].name;
  cfg.reconnect.enabled = true;
  cfg.reconnect.max_attempts = 3;
  cfg.reconnect.retry_interval_ms = 20;
  cfg.reconnect.frame_timeout_ms = 120;
  validate_config(cfg);

  std::atomic<int> left_instances{0};
  CameraManager::DeviceFactory factory = [&](const CameraConfig& cam) {
    const bool is_left = cam.name == cfg.cameras[0].name;
    const int instance = is_left ? left_instances.fetch_add(1) : 0;
    return std::make_unique<ReconnectTestDevice>(cam, is_left && instance == 0 ? 3 : -1);
  };

  SharedMemoryRingBuffer shm;
  shm.create(cfg);
  MetadataPublisher publisher(cfg.metadata);
  FrameSynchronizerSet synchronizer(cfg);
  Recorder recorder(cfg.recording);
  CameraManager manager(cfg, shm, publisher, synchronizer, recorder, std::move(factory));
  manager.start();

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  bool recovered = false;
  uint64_t right_frames_before_recovery = 0;
  uint64_t bundles_before_recovery = 0;
  while (std::chrono::steady_clock::now() < deadline) {
    const auto snapshot = manager.snapshot();
    const auto& left_reconnect = snapshot.camera_reconnect_stats.at(cfg.cameras[0].name);
    if (left_reconnect.disconnect_count > 0 && right_frames_before_recovery == 0) {
      right_frames_before_recovery = snapshot.stream_stats.at(cfg.cameras[1].name + ".color").frame_count;
      bundles_before_recovery = snapshot.complete_bundle_count;
    }
    if (left_reconnect.success_count > 0 &&
        snapshot.camera_connected.at(cfg.cameras[0].name) &&
        snapshot.camera_connected.at(cfg.cameras[1].name) &&
        snapshot.complete_bundle_count > bundles_before_recovery) {
      recovered = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  const auto final_snapshot = manager.snapshot();
  manager.stop();

  assert(recovered);
  assert(left_instances.load() >= 2);
  assert(final_snapshot.camera_reconnect_stats.at(cfg.cameras[0].name).attempt_count >= 1);
  assert(final_snapshot.camera_reconnect_stats.at(cfg.cameras[0].name).success_count >= 1);
  assert(final_snapshot.stream_stats.at(cfg.cameras[1].name + ".color").frame_count >
         right_frames_before_recovery);
  assert(final_snapshot.complete_bundle_count > bundles_before_recovery);
}

AppConfig make_identity_test_config(const std::string& suffix, bool reconnect) {
  auto cfg = make_test_config();
  cfg.cameras.resize(1);
  cfg.shared_memory.name = "/camera_server_identity_" + suffix;
  cfg.metadata.pub_bind = "tcp://127.0.0.1:5997";
  cfg.reconnect.enabled = reconnect;
  cfg.reconnect.max_attempts = 3;
  cfg.reconnect.retry_interval_ms = 40;
  cfg.reconnect.frame_timeout_ms = 120;
  validate_config(cfg);
  return cfg;
}

struct IdentityManagerFixture {
  AppConfig cfg;
  SharedMemoryRingBuffer shm;
  MetadataPublisher publisher;
  FrameSynchronizerSet synchronizer;
  Recorder recorder;
  std::unique_ptr<CameraManager> manager;

  IdentityManagerFixture(AppConfig config, CameraManager::DeviceFactory factory)
      : cfg(std::move(config)), publisher(cfg.metadata), synchronizer(cfg),
        recorder(cfg.recording) {
    shm.create(cfg);
    manager = std::make_unique<CameraManager>(cfg, shm, publisher, synchronizer,
                                             recorder, std::move(factory));
  }

  ~IdentityManagerFixture() {
    manager.reset();
    shm.close();
    ::shm_unlink(cfg.shared_memory.name.c_str());
  }
};

RealSenseDeviceInfo test_session_info(const std::string& serial,
                                     const std::string& port, const std::string& tag) {
  RealSenseDeviceInfo info;
  info.name = "RealSense test device";
  info.serial = serial;
  info.physical_port = port;
  info.firmware_version = "firmware-" + tag;
  info.recommended_firmware_version = "recommended-" + tag;
  info.product_id = "product-" + tag;
  info.usb_type = "usb-" + tag;
  return info;
}

void assert_no_session_info(const HealthSnapshot& snapshot, const std::string& name) {
  // Configured camera_serial remains useful while disconnected; session metadata
  // must be absent, rather than a stale value or an empty routing placeholder.
  assert(snapshot.camera_physical_port.count(name) == 0);
  assert(snapshot.camera_firmware_version.count(name) == 0);
  assert(snapshot.camera_recommended_firmware_version.count(name) == 0);
  assert(snapshot.camera_product_id.count(name) == 0);
  assert(snapshot.camera_usb_type.count(name) == 0);
}

void assert_session_info(const HealthSnapshot& snapshot, const std::string& name,
                         const RealSenseDeviceInfo& info) {
  assert(snapshot.camera_connected.at(name));
  assert(snapshot.camera_serial.at(name) == info.serial);
  assert(snapshot.camera_physical_port.at(name) == info.physical_port);
  assert(snapshot.camera_firmware_version.at(name) == info.firmware_version);
  assert(snapshot.camera_recommended_firmware_version.at(name) == info.recommended_firmware_version);
  assert(snapshot.camera_product_id.at(name) == info.product_id);
  assert(snapshot.camera_usb_type.at(name) == info.usb_type);
}

bool wait_for_test_condition(const std::function<bool()>& predicate,
                             std::chrono::milliseconds timeout = std::chrono::seconds(3)) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return predicate();
}

void test_session_identity_refreshes_leaf_and_usb_root_after_reconnect() {
  auto cfg = make_identity_test_config("reconnect", true);
  const auto name = cfg.cameras[0].name;
  const auto serial = cfg.cameras[0].serial;
  const std::vector<RealSenseDeviceInfo> sessions{
      test_session_info(serial, "/sys/devices/usb4/4-2/4-2.2/4-2.2:1.0/video4linux/video2", "old"),
      test_session_info(serial, "/sys/devices/usb4/4-2/4-2.2/4-2.2:1.0/video4linux/video32", "leaf"),
      test_session_info(serial, "/sys/devices/usb8/8-1/8-1.2/8-1.2:1.0/video4linux/video12", "root")};
  std::atomic<int> instances{0};
  IdentityManagerFixture fixture(cfg, [&](const CameraConfig& cam) {
    const int instance = instances.fetch_add(1);
    const int session = instance < 2 ? 0 : (instance == 2 ? 1 : 2);
    return std::make_unique<ReconnectTestDevice>(cam, session < 2 ? 3 : -1,
                                                sessions[session], instance == 1);
  });
  fixture.manager->start();
  bool seen[3]{false, false, false};
  bool saw_disconnected = false;
  bool saw_start_failure = false;
  const bool recovered = wait_for_test_condition([&] {
    const auto h = fixture.manager->snapshot();
    if (!h.camera_connected.at(name)) {
      assert_no_session_info(h, name);
      saw_disconnected = true;
    } else {
      const auto port = h.camera_physical_port.at(name);
      bool recognized = false;
      for (size_t i = 0; i < sessions.size(); ++i) {
        if (port != sessions[i].physical_port) continue;
        assert_session_info(h, name, sessions[i]);
        recognized = true;
        seen[i] = true;
      }
      assert(recognized);
    }
    if (h.camera_reconnect_stats.at(name).last_error.find("synthetic pipeline start failure") !=
        std::string::npos) saw_start_failure = true;
    return seen[0] && seen[1] && seen[2] && saw_disconnected && saw_start_failure;
  });
  fixture.manager->stop();
  const auto stopped = fixture.manager->snapshot();
  assert(!stopped.camera_connected.at(name));
  assert_no_session_info(stopped, name);
  assert(recovered);
  assert(instances.load() >= 4);
}

void test_reconnect_identity_waits_for_first_frame() {
  auto cfg = make_identity_test_config("pending", true);
  cfg.reconnect.frame_timeout_ms = 500;
  const auto name = cfg.cameras[0].name;
  const auto info = test_session_info(cfg.cameras[0].serial, "/usb8/video32", "pending");
  auto frames_allowed = std::make_shared<std::atomic<bool>>(false);
  std::atomic<bool> created{false};
  IdentityManagerFixture fixture(cfg, [&](const CameraConfig& cam) {
    created = true;
    return std::make_unique<ReconnectTestDevice>(cam, -1, info, false, frames_allowed);
  });
  std::exception_ptr start_error;
  std::thread starter([&] {
    try { fixture.manager->start(); } catch (...) { start_error = std::current_exception(); }
  });
  const bool pipeline_created = wait_for_test_condition([&] { return created.load(); });
  // The opened session has valid identity, but none may be advertised before data.
  for (int i = 0; i < 10; ++i) {
    const auto h = fixture.manager->snapshot();
    assert(!h.camera_connected.at(name));
    assert_no_session_info(h, name);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  *frames_allowed = true;
  starter.join();
  if (start_error) std::rethrow_exception(start_error);
  assert(pipeline_created);
  assert(wait_for_test_condition([&] { return fixture.manager->snapshot().camera_connected.at(name); }));
  assert_session_info(fixture.manager->snapshot(), name, info);
  fixture.manager->stop();
  assert_no_session_info(fixture.manager->snapshot(), name);
}

void test_no_reconnect_identity_start_stop_missing_and_invalid_serial() {
  for (const bool has_identity : {true, false}) {
    auto cfg = make_identity_test_config(has_identity ? "direct" : "missing", false);
    const auto name = cfg.cameras[0].name;
    const auto info = test_session_info(cfg.cameras[0].serial, "/usb4/video2", "direct");
    IdentityManagerFixture fixture(cfg, [&](const CameraConfig& cam) {
      return std::make_unique<ReconnectTestDevice>(cam, 0,
          has_identity ? std::optional<RealSenseDeviceInfo>(info) : std::nullopt);
    });
    fixture.manager->start();
    const auto h = fixture.manager->snapshot();
    // Without reconnect, successful start is the established readiness boundary.
    assert(h.camera_connected.at(name));
    if (has_identity) assert_session_info(h, name, info);
    else assert_no_session_info(h, name);  // Mock devices may omit identity.
    fixture.manager->stop();
    const auto stopped = fixture.manager->snapshot();
    assert(!stopped.camera_connected.at(name));
    assert_no_session_info(stopped, name);
  }
  for (const std::string actual_serial : {std::string{}, std::string{"WRONG_SERIAL"}}) {
    auto cfg = make_identity_test_config(actual_serial.empty() ? "empty" : "wrong", false);
    const auto name = cfg.cameras[0].name;
    const auto info = test_session_info(actual_serial, "/usb8/video32", "invalid");
    IdentityManagerFixture fixture(cfg, [&](const CameraConfig& cam) {
      return std::make_unique<ReconnectTestDevice>(cam, 0, info);
    });
    expect_config_failure([&] { fixture.manager->start(); }, "active camera serial mismatch");
    const auto failed = fixture.manager->snapshot();
    assert(!failed.camera_connected.at(name));
    assert_no_session_info(failed, name);
  }
}

void test_partial_start_failure_clears_previous_camera_identity() {
  auto cfg = make_identity_test_config("partial_failure", false);
  cfg.cameras = make_test_config().cameras;
  cfg.cameras.resize(2);
  IdentityManagerFixture fixture(cfg, [&](const CameraConfig& cam) {
    const auto info = test_session_info(cam.serial, "/usb4/video2", cam.name);
    return std::make_unique<ReconnectTestDevice>(cam, 0, info, cam.name == cfg.cameras[1].name);
  });
  expect_config_failure([&] { fixture.manager->start(); }, "synthetic pipeline start failure");
  const auto failed = fixture.manager->snapshot();
  for (const auto& cam : cfg.cameras) {
    assert(!failed.camera_connected.at(cam.name));
    assert_no_session_info(failed, cam.name);
  }
}

void test_snapshot_does_not_wait_for_blocked_timeout_teardown() {
  auto cfg = make_identity_test_config("blocked_stop", true);
  const auto name = cfg.cameras[0].name;
  const auto info = test_session_info(cfg.cameras[0].serial, "/usb4/video2", "blocked");
  auto stop_gate = std::make_shared<TestDeviceStopGate>();
  IdentityManagerFixture fixture(cfg, [&](const CameraConfig& cam) {
    return std::make_unique<ReconnectTestDevice>(cam, 3, info, false, nullptr, stop_gate);
  });
  fixture.manager->start();
  assert_session_info(fixture.manager->snapshot(), name, info);
  const bool teardown_started = wait_for_test_condition([&] { return stop_gate->entered.load(); });
  HealthSnapshot during_stop;
  std::atomic<bool> snapshot_done{false};
  std::thread reader([&] {
    during_stop = fixture.manager->snapshot();
    snapshot_done = true;
  });
  const bool returned_before_release = wait_for_test_condition(
      [&] { return snapshot_done.load(); }, std::chrono::milliseconds(100));
  // Release before joining/asserting so an old blocking implementation fails
  // cleanly instead of hanging the test process indefinitely.
  stop_gate->release = true;
  reader.join();
  fixture.manager->stop();
  assert(teardown_started);
  assert(returned_before_release);
  assert(!during_stop.camera_connected.at(name));
  assert_no_session_info(during_stop, name);
  assert(during_stop.camera_capture_stats.count(name) == 0);
  assert_no_session_info(fixture.manager->snapshot(), name);
}

void test_realsense_preflight_contract() {
  auto cfg = make_test_config();
  cfg.server.simulate_cameras = false;
  cfg.cameras.clear();
  for (const auto& [name, serial] :
       std::vector<std::pair<std::string, std::string>>{{"left", "LEFT"}, {"right", "RIGHT"}}) {
    CameraConfig cam;
    cam.name = name;
    cam.backend = "realsense";
    cam.serial = serial;
    cam.required = true;
    cam.color.enabled = true;
    cam.depth.enabled = true;
    cfg.cameras.push_back(cam);
  }

  const RealSenseDeviceInfo left{"Intel RealSense D405", "LEFT", "5.17.0.10", "5.17.0.10",
                                 "/usb4/4-2.3.2", "0B5B", "3.2"};
  const RealSenseDeviceInfo right{"Intel RealSense D405", "RIGHT", "5.17.0.10", "5.17.0.10",
                                  "/usb4/4-2.4.2", "0B5B", "3.2"};
  validate_realsense_preflight(cfg, {left, right}, "2.58.1");

  expect_config_failure(
      [&] { validate_realsense_preflight(cfg, {left, right}, "2.55.1"); }, "older than required 2.58.1");
  auto usb2 = right;
  usb2.usb_type = "2.1";
  expect_config_failure(
      [&] { validate_realsense_preflight(cfg, {left, usb2}, "2.58.1"); }, "not on USB3");
  auto old_firmware = left;
  old_firmware.firmware_version = "5.12.14.100";
  expect_config_failure(
      [&] { validate_realsense_preflight(cfg, {old_firmware, right}, "2.58.1"); }, "older than required 5.17.0.10");
  auto mismatched = right;
  mismatched.firmware_version = "5.17.0.11";
  expect_config_failure(
      [&] { validate_realsense_preflight(cfg, {left, mismatched}, "2.58.1"); }, "must use identical firmware");
}

void test_real_placeholder_config_fails() {
  std::string path = "config/triple_realsense.yaml";
  if (!std::ifstream(path).good()) path = "../config/triple_realsense.yaml";
  expect_config_failure([&] { (void)load_config(path); }, "invalid serial placeholder");
}

AppConfig make_realsense_plus_fisheye_config() {
  AppConfig cfg;
  cfg.server.name = "camera_server";
  cfg.server.simulate_cameras = false;
  cfg.shared_memory.name = "/camera_server_unit_test";
  cfg.shared_memory.size_mb = 64;
  cfg.shared_memory.ring_slots = 4;
  cfg.metadata.pub_bind = "tcp://127.0.0.1:5999";
  cfg.sync.master_camera = "left_realsense";
  CameraConfig rs;
  rs.name = "left_realsense";
  rs.backend = "realsense";
  rs.serial = "260322278348";
  rs.required = true;
  rs.color.enabled = true;
  rs.color.format = "rgb8";
  CameraConfig fe;
  fe.name = "left_fisheye";
  fe.backend = "uvc";
  fe.device = "/dev/video12";
  fe.required = true;
  fe.color.enabled = true;
  fe.color.format = "rgb8";
  fe.depth.enabled = false;
  cfg.cameras = {rs, fe};
  return cfg;
}

void test_config_uvc_fisheye_backend() {
  auto cfg = make_realsense_plus_fisheye_config();
  validate_config(cfg);  // uvc selects by device path, no serial required
  assert(required_stream_keys(cfg).size() == 2);

  // uvc bypasses the realsense serial placeholder / MOCK rules (the DECXIN fisheye
  // shares a fixed serial, so serial-based discovery is impossible).
  auto shared_serial = cfg;
  shared_serial.cameras[1].serial = "UNKNOWN";  // would fail for a realsense camera
  validate_config(shared_serial);

  auto no_device = cfg;
  no_device.cameras[1].device = "";
  expect_config_failure([&] { validate_config(no_device); }, "empty device");

  auto with_depth = cfg;
  with_depth.cameras[1].depth.enabled = true;
  expect_config_failure([&] { validate_config(with_depth); }, "does not support a depth stream");

  auto bad_backend = cfg;
  bad_backend.cameras[1].backend = "zed";
  expect_config_failure([&] { validate_config(bad_backend); }, "unsupported backend");
}

void test_uvc_device_resolution() {
  assert(resolve_v4l2_index("12") == 12);
  assert(resolve_v4l2_index("/dev/video7") == 7);  // filename parse, even if absent
  assert(resolve_v4l2_index("garbage") == -1);
  assert(resolve_v4l2_index("") == -1);
  assert(!uvc_device_present(""));
  assert(!uvc_device_present("/dev/v4l/by-path/does-not-exist-video-index0"));
}

void test_quad_fisheye_config_parses() {
  std::string path = "config/quad_realsense_fisheye.yaml";
  if (!std::ifstream(path).good()) path = "../config/quad_realsense_fisheye.yaml";
  auto cfg = load_config(path);
  assert(cfg.cameras.size() == 4);
  int uvc = 0;
  int realsense = 0;
  for (const auto& cam : cfg.cameras) {
    if (cam.backend == "uvc") {
      ++uvc;
      assert(!cam.device.empty());
      assert(!cam.depth.enabled);
      assert(cam.color.enabled);
    } else {
      ++realsense;
    }
  }
  assert(uvc == 2 && realsense == 2);
  // 2 realsense color + 2 realsense depth + 2 fisheye color
  assert(required_stream_keys(cfg).size() == 6);
}

void test_shared_memory_roundtrip() {
  auto cfg = make_test_config();
  SharedMemoryRingBuffer shm;
  shm.create(cfg);
  std::vector<uint8_t> bytes(8 * 4 * 3, 42);
  auto meta = shm.write_frame("head", "MOCK_HEAD", "color", 7, now_ns(), now_ns(), 1.0, 8 * 3, bytes.data(), bytes.size());
  assert(meta.valid);
  assert(meta.ring_name == "head.color");
  auto read = shm.read_slot(meta.ring_name, meta.slot_index);
  assert(read.has_value());
  assert(read->meta.frame_number == 7);
  assert(read->bytes == bytes);
}

void test_shared_memory_rejects_oversized_slot_metadata() {
  auto cfg = make_test_config();
  SharedMemoryRingBuffer shm;
  shm.create(cfg);
  std::vector<uint8_t> bytes(8 * 4 * 3, 42);
  auto meta = shm.write_frame("head", "MOCK_HEAD", "color", 7, now_ns(), now_ns(), 1.0, 8 * 3, bytes.data(), bytes.size());

  MappedShm mapped(cfg.shared_memory.name, static_cast<size_t>(shm.size_bytes()));
  auto* slot = reinterpret_cast<shm_layout::FrameSlotHeader*>(
      static_cast<uint8_t*>(mapped.data) + meta.shm_offset - sizeof(shm_layout::FrameSlotHeader));
  slot->size_bytes = static_cast<uint32_t>(bytes.size() + 1024 * 1024);

  auto read = shm.read_slot(meta.ring_name, meta.slot_index);
  assert(!read.has_value());
}

void test_shared_memory_open_rejects_invalid_slot_offsets() {
  auto cfg = make_test_config();
  {
    SharedMemoryRingBuffer shm;
    shm.create(cfg);
    MappedShm mapped(cfg.shared_memory.name, static_cast<size_t>(shm.size_bytes()));
    auto* header = reinterpret_cast<shm_layout::ShmHeader*>(mapped.data);
    auto* desc = reinterpret_cast<shm_layout::StreamRingHeader*>(
        static_cast<uint8_t*>(mapped.data) + header->descriptors_offset);
    desc[0].slots_offset = header->total_size_bytes + 64;
  }

  SharedMemoryRingBuffer reader;
  bool threw = false;
  try {
    reader.open_read_only(cfg.shared_memory.name);
  } catch (const std::runtime_error&) {
    threw = true;
  }
  assert(threw);
}

void test_hardware_sync_uses_selected_master_frame_number() {
  auto cfg = make_test_config();
  cfg.sync.mode = "hardware";
  cfg.sync.bundle_policy = "frame_number";
  cfg.sync.max_bundle_time_diff_ms = 100.0;
  FrameSynchronizer sync(cfg);
  std::map<std::string, uint64_t> drops;
  const uint64_t t = now_ns();

  FrameMeta h;
  h.camera_name = "head";
  h.stream = "color";
  h.ring_name = "head.color";
  h.frame_number = 10;
  h.host_arrival_time_ns = t;
  h.valid = true;
  FrameMeta l = h;
  l.camera_name = "left_wrist";
  l.ring_name = "left_wrist.color";
  l.frame_number = 10;
  l.host_arrival_time_ns = t + 1000000;
  FrameMeta l11 = l;
  l11.frame_number = 11;
  l11.host_arrival_time_ns = t + 3000000;
  FrameMeta r11 = h;
  r11.camera_name = "right_wrist";
  r11.ring_name = "right_wrist.color";
  r11.frame_number = 11;
  r11.host_arrival_time_ns = t + 4000000;
  FrameMeta r10 = r11;
  r10.frame_number = 10;
  r10.host_arrival_time_ns = t + 2000000;

  assert(!sync.push_frame(l, drops).has_value());
  assert(!sync.push_frame(h, drops).has_value());
  assert(!sync.push_frame(l11, drops).has_value());
  assert(!sync.push_frame(r11, drops).has_value());
  auto bundle = sync.push_frame(r10, drops);
  assert(bundle.has_value());
  assert(bundle->complete);
  assert(bundle->frames.at("head.color").frame_number == 10);
  assert(bundle->frames.at("left_wrist.color").frame_number == 10);
  assert(bundle->frames.at("right_wrist.color").frame_number == 10);
}

void test_synchronizer_and_drop_schema() {
  auto cfg = make_test_config();
  FrameSynchronizer sync(cfg);
  std::map<std::string, uint64_t> drops{{"head.color", 1}, {"left_wrist.color", 0}, {"right_wrist.color", 0}};
  const uint64_t t = now_ns();
  FrameMeta h; h.camera_name = "head"; h.stream = "color"; h.ring_name = "head.color"; h.frame_number = 1; h.host_arrival_time_ns = t; h.valid = true;
  FrameMeta l = h; l.camera_name = "left_wrist"; l.ring_name = "left_wrist.color"; l.host_arrival_time_ns = t + 1000000;
  FrameMeta r = h; r.camera_name = "right_wrist"; r.ring_name = "right_wrist.color"; r.host_arrival_time_ns = t + 2000000;
  assert(!sync.push_frame(h, drops).has_value());
  assert(!sync.push_frame(l, drops).has_value());
  auto bundle = sync.push_frame(r, drops);
  assert(bundle.has_value());
  assert(bundle->complete);
  assert(bundle->bundle_seq == 1);
  h.frame_number = 2;
  h.host_arrival_time_ns = t + 33333333;
  l.frame_number = 2;
  l.host_arrival_time_ns = t + 34333333;
  r.frame_number = 2;
  r.host_arrival_time_ns = t + 35333333;
  assert(!sync.push_frame(l, drops).has_value());
  assert(!sync.push_frame(r, drops).has_value());
  bundle = sync.push_frame(h, drops);
  assert(bundle.has_value());
  assert(bundle->complete);
  assert(bundle->frames.size() == 3);
  assert(bundle->bundle_seq == 2);
  assert(bundle->drop_counters.at("head.color") == 1);
  const auto json = bundle_to_json(*bundle);
  assert(json.find("camera_server.bundle.v1") != std::string::npos);
  assert(json.find("\"group_name\":\"default\"") != std::string::npos);
  assert(json.find("\"complete\":true") != std::string::npos);
}

void test_optional_capture_metadata_json() {
  FrameMeta frame;
  frame.camera_name = "left_realsense";
  frame.serial = "412622272078";
  frame.stream = "color";
  frame.ring_name = "left_realsense.color";
  frame.frame_number = 42;
  frame.actual_exposure_us = 6951.0;
  frame.gain_level = 16.0;
  frame.auto_exposure = true;
  frame.valid = true;

  const auto frame_json = frame_to_json(frame);
  assert(frame_json.find("\"actual_exposure_us\":6951") != std::string::npos);
  assert(frame_json.find("\"gain_level\":16") != std::string::npos);
  assert(frame_json.find("\"auto_exposure\":true") != std::string::npos);

  FrameBundleMeta bundle;
  bundle.complete = true;
  bundle.frames[frame.ring_name] = frame;
  const auto bundle_json = bundle_to_json(bundle);
  assert(bundle_json.find("\"actual_exposure_us\":6951") != std::string::npos);

  frame.actual_exposure_us.reset();
  frame.gain_level.reset();
  frame.auto_exposure.reset();
  const auto absent_json = frame_to_json(frame);
  assert(absent_json.find("actual_exposure_us") == std::string::npos);
  assert(absent_json.find("gain_level") == std::string::npos);
  assert(absent_json.find("auto_exposure") == std::string::npos);
}

void test_bundle_groups_publish_independently() {
  auto cfg = make_test_config();
  cfg.bundle_groups = {
      BundleGroupConfig{"policy", "camera.bundle.policy", "left_wrist.color",
                        {"left_wrist.color", "right_wrist.color"}, 20.0, false},
      BundleGroupConfig{"stereo", "camera.bundle.stereo", "head.color",
                        {"head.color"}, 20.0, false},
  };
  validate_config(cfg);
  FrameSynchronizerSet sync(cfg);
  const uint64_t t = now_ns();
  std::map<std::string, uint64_t> drops;

  FrameMeta head;
  head.camera_name = "head";
  head.stream = "color";
  head.ring_name = "head.color";
  head.frame_number = 1;
  head.host_arrival_time_ns = t;
  head.valid = true;
  auto published = sync.push_frame(head, drops);
  assert(published.size() == 1);
  assert(published[0].topic == "camera.bundle.stereo");
  assert(published[0].bundle.group_name == "stereo");

  FrameMeta left = head;
  left.camera_name = "left_wrist";
  left.ring_name = "left_wrist.color";
  left.host_arrival_time_ns = t + 1000000;
  assert(sync.push_frame(left, drops).empty());
  FrameMeta right = left;
  right.camera_name = "right_wrist";
  right.ring_name = "right_wrist.color";
  right.host_arrival_time_ns = t + 2000000;
  published = sync.push_frame(right, drops);
  assert(published.size() == 1);
  assert(published[0].topic == "camera.bundle.policy");
  assert(published[0].bundle.frames.size() == 2);
}

void test_master_camera_generation_reset_resumes_lower_frame_numbers() {
  auto cfg = make_test_config();
  cfg.bundle_groups = {
      BundleGroupConfig{"wrist_right", "camera.bundle.wrist_right", "right_wrist.color",
                        {"right_wrist.color"}, 20.0, false},
  };
  validate_config(cfg);
  FrameSynchronizerSet sync(cfg);
  std::map<std::string, uint64_t> drops;
  const uint64_t t = now_ns();

  FrameMeta right;
  right.camera_name = "right_wrist";
  right.stream = "color";
  right.ring_name = "right_wrist.color";
  right.frame_number = 100;
  right.host_arrival_time_ns = t;
  right.valid = true;
  auto published = sync.push_frame(right, drops);
  assert(published.size() == 1);

  right.frame_number = 101;
  right.host_arrival_time_ns = t + 33333333;
  published = sync.push_frame(right, drops);
  assert(published.size() == 1);
  auto stats = sync.stats().at("wrist_right");
  assert(stats.bundle_seq == 2);
  assert(stats.complete_bundle_count == 2);
  assert(stats.publish_rate_hz > 0.0);

  assert(sync.reset_camera_generation("right_wrist") == 1);
  stats = sync.stats().at("wrist_right");
  assert(stats.bundle_seq == 2);
  assert(stats.complete_bundle_count == 2);
  assert(stats.publish_rate_hz == 0.0);
  assert(stats.last_skew_ms == 0.0);

  right.frame_number = 1;
  right.host_arrival_time_ns = t + 1000000000;
  published = sync.push_frame(right, drops);
  assert(published.size() == 1);
  assert(published[0].bundle.bundle_seq == 3);
  assert(published[0].bundle.frames.at("right_wrist.color").frame_number == 1);
}

void test_non_master_camera_generation_reset_discards_old_group_buffers() {
  auto cfg = make_test_config();
  cfg.cameras.resize(2);
  cfg.bundle_groups = {
      BundleGroupConfig{"policy", "camera.bundle.policy", "head.color",
                        {"head.color", "left_wrist.color"}, 20.0, false},
  };
  validate_config(cfg);
  FrameSynchronizerSet sync(cfg);
  std::map<std::string, uint64_t> drops;
  const uint64_t t = now_ns();

  FrameMeta head;
  head.camera_name = "head";
  head.stream = "color";
  head.ring_name = "head.color";
  head.frame_number = 100;
  head.host_arrival_time_ns = t;
  head.valid = true;
  FrameMeta left = head;
  left.camera_name = "left_wrist";
  left.ring_name = "left_wrist.color";
  left.host_arrival_time_ns = t + 1000000;
  assert(sync.push_frame(head, drops).empty());
  auto published = sync.push_frame(left, drops);
  assert(published.size() == 1);

  assert(sync.reset_camera_generation("left_wrist") == 1);
  left.frame_number = 1;
  left.host_arrival_time_ns = t + 1000000000;
  assert(sync.push_frame(left, drops).empty());

  // The old master generation remains gated, while the next master frame can
  // pair only with the post-reconnect non-master frame.
  head.host_arrival_time_ns = t + 1001000000;
  assert(sync.push_frame(head, drops).empty());
  head.frame_number = 101;
  head.host_arrival_time_ns = t + 1002000000;
  published = sync.push_frame(head, drops);
  assert(published.size() == 1);
  assert(published[0].bundle.bundle_seq == 2);
  assert(published[0].bundle.frames.at("head.color").frame_number == 101);
  assert(published[0].bundle.frames.at("left_wrist.color").frame_number == 1);
}

void test_bundle_retry_and_master_drop_are_distinct() {
  auto cfg = make_test_config();
  FrameSynchronizerSet sync(cfg);
  const uint64_t t = now_ns();
  std::map<std::string, uint64_t> drops;
  FrameMeta head;
  head.camera_name = "head";
  head.stream = "color";
  head.ring_name = "head.color";
  head.frame_number = 1;
  head.host_arrival_time_ns = t;
  head.valid = true;
  assert(sync.push_frame(head, drops).empty());
  auto stats = sync.stats().at("default");
  assert(stats.incomplete_retry_count == 1);
  assert(stats.dropped_master_count == 0);

  head.frame_number = 2;
  head.host_arrival_time_ns = t + 33333333;
  assert(sync.push_frame(head, drops).empty());
  stats = sync.stats().at("default");
  assert(stats.incomplete_retry_count == 2);
  assert(stats.dropped_master_count == 1);
}

void test_health_json() {
  HealthSnapshot h;
  h.host_time_ns = now_ns();
  h.mode = "diagnostic";
  h.status = "degraded";
  h.status_reasons.push_back("drops_observed:head.color");
  h.stream_status["head.color"] = "degraded";
  h.camera_connected["head"] = true;
  h.camera_serial["head"] = "MOCK_HEAD";
  h.realsense_sdk_version = "2.58.1";
  h.realsense_backend = "native";
  h.camera_firmware_version["head"] = "5.17.3.10";
  h.camera_physical_port["head"] = "/usb4/4-1.3.3";
  h.camera_product_id["head"] = "0B07";
  h.camera_usb_type["head"] = "3.2";
  h.camera_reconnect_stats["head"].attempt_count = 2;
  h.camera_reconnect_stats["head"].success_count = 1;
  h.stream_stats["head.color"].frame_count = 3;
  h.stream_stats["head.color"].internal_queue_drop_count = 1;
  h.stream_stats["head.color"].recorder_drop_count = 2;
  h.shm_name = "/camera_server_unit_test";
  auto json = health_to_json(h);
  assert(json.find("camera_server.health.v1") != std::string::npos);
  assert(json.find("\"status\":\"degraded\"") != std::string::npos);
  assert(json.find("\"status_reasons\"") != std::string::npos);
  assert(json.find("\"sdk_version\":\"2.58.1\"") != std::string::npos);
  assert(json.find("\"backend\":\"native\"") != std::string::npos);
  assert(json.find("\"firmware_version\":\"5.17.3.10\"") != std::string::npos);
  assert(json.find("\"usb_type\":\"3.2\"") != std::string::npos);
  assert(json.find("\"reconnect\":{\"attempt_count\":2,\"success_count\":1") != std::string::npos);
  assert(json.find("\"status_color\":\"degraded\"") != std::string::npos);
  assert(json.find("\"drops_color\":{\"frame_number_gap\":0,\"internal_queue\":1,\"recorder\":2,\"total\":3") != std::string::npos);
  assert(json.find("\"internal_queue_delta\":0") != std::string::npos);
  assert(json.find("head") != std::string::npos);
}

FrameMeta make_recording_meta(uint64_t frame_number) {
  FrameMeta meta;
  meta.camera_name = "head";
  meta.serial = "MOCK_HEAD";
  meta.stream = "color";
  meta.ring_name = "head.color";
  meta.frame_number = frame_number;
  meta.host_arrival_time_ns = now_ns();
  meta.width = 2;
  meta.height = 2;
  meta.stride_bytes = 6;
  meta.format = "rgb8";
  meta.size_bytes = 12;
  meta.valid = true;
  return meta;
}

void test_queue_drop_newest_factory_skips_payload_creation() {
  BoundedQueue<std::vector<uint8_t>> queue(1);
  int payload_creations = 0;
  assert(queue.try_push_drop_newest_with_factory([&] {
    ++payload_creations;
    return std::vector<uint8_t>(4, 1);
  }));
  assert(!queue.try_push_drop_newest_with_factory([&] {
    ++payload_creations;
    return std::vector<uint8_t>(1024 * 1024, 2);
  }));
  assert(payload_creations == 1);
  assert(queue.dropped_count() == 1);
  assert(queue.size() == 1);
}

void test_recorder_stop_drains_queue() {
  namespace fs = std::filesystem;
  const fs::path dir = fs::temp_directory_path() / ("camera_server_recorder_test_" + std::to_string(now_ns()));
  RecordingConfig cfg;
  cfg.enabled = true;
  cfg.output_dir = dir.string();
  cfg.queue_capacity_frames = 16;
  cfg.writer_threads = 1;
  cfg.drop_policy = "drop_newest";

  Recorder recorder(cfg);
  recorder.start();
  std::vector<uint8_t> payload(12, 7);
  for (uint64_t i = 1; i <= 5; ++i) {
    assert(recorder.enqueue_copy(make_recording_meta(i), payload.data(), payload.size()));
  }
  recorder.stop();

  size_t raw_files = 0;
  for (const auto& entry : fs::recursive_directory_iterator(dir)) {
    if (entry.is_regular_file() && entry.path().filename() != "camera_metadata.jsonl") ++raw_files;
  }
  assert(raw_files == 5);
  assert(recorder.dropped_by_queue() == 0);

  std::ifstream metadata(dir / "camera_metadata.jsonl");
  size_t metadata_lines = 0;
  std::string line;
  while (std::getline(metadata, line)) ++metadata_lines;
  assert(metadata_lines == 5);
  fs::remove_all(dir);
}

void test_health_threshold_status() {
  HealthConfig cfg;
  cfg.warn_if_frame_age_ms_gt = 10.0;
  cfg.warn_if_drop_count_increases = true;
  cfg.warn_if_bundle_skew_ms_gt = 5.0;

  HealthSnapshot h;
  h.host_time_ns = 1000000000ull;
  h.mode = "diagnostic";
  h.camera_connected["head"] = true;
  h.camera_serial["head"] = "MOCK_HEAD";
  auto& st = h.stream_stats["head.color"];
  st.frame_count = 10;
  st.last_frame_time_ns = h.host_time_ns - 20000000ull;
  st.recorder_drop_count = 1;
  h.max_time_diff_ms = 6.0;

  apply_health_thresholds(cfg, h);
  assert(h.status == "degraded");
  assert(h.stream_status.at("head.color") == "degraded");
  assert(!h.status_reasons.empty());

  auto json = health_to_json(h);
  assert(json.find("\"status\":\"degraded\"") != std::string::npos);
  assert(json.find("frame_age_ms_gt_threshold:head.color") != std::string::npos);
  assert(json.find("\"recorder\":1") != std::string::npos);
}

void test_health_drop_deltas_recover() {
  HealthConfig cfg;
  cfg.warn_if_drop_count_increases = true;
  cfg.warn_if_fps_below = 0.0;
  HealthSnapshot previous;
  previous.host_time_ns = 1000000000ull;
  previous.camera_connected["head"] = true;
  previous.stream_stats["head.color"].frame_number_gap_drop_count = 5;

  HealthSnapshot unchanged = previous;
  unchanged.host_time_ns += 100000000ull;
  unchanged.stream_stats["head.color"].last_frame_time_ns = unchanged.host_time_ns;
  populate_health_deltas(unchanged, &previous);
  apply_health_thresholds(cfg, unchanged);
  assert(unchanged.stream_stats.at("head.color").frame_number_gap_drop_delta == 0);
  assert(unchanged.status == "ok");

  HealthSnapshot increased = unchanged;
  increased.host_time_ns += 100000000ull;
  increased.stream_stats["head.color"].last_frame_time_ns = increased.host_time_ns;
  increased.stream_stats["head.color"].frame_number_gap_drop_count = 6;
  populate_health_deltas(increased, &unchanged);
  apply_health_thresholds(cfg, increased);
  assert(increased.stream_stats.at("head.color").frame_number_gap_drop_delta == 1);
  assert(increased.status == "degraded");
}

int main() {
  test_config();
  test_priority_bundle_config();
  test_config_validation_rejects_real_serial_placeholders();
  test_config_validation_rejects_invalid_sync_combinations();
  test_config_validation_accepts_bounded_reconnect();
  test_per_camera_reconnect_keeps_other_camera_streaming();
  test_session_identity_refreshes_leaf_and_usb_root_after_reconnect();
  test_reconnect_identity_waits_for_first_frame();
  test_no_reconnect_identity_start_stop_missing_and_invalid_serial();
  test_partial_start_failure_clears_previous_camera_identity();
  test_snapshot_does_not_wait_for_blocked_timeout_teardown();
  test_realsense_preflight_contract();
  test_real_placeholder_config_fails();
  test_config_uvc_fisheye_backend();
  test_uvc_device_resolution();
  test_quad_fisheye_config_parses();
  test_shared_memory_roundtrip();
  test_shared_memory_rejects_oversized_slot_metadata();
  test_shared_memory_open_rejects_invalid_slot_offsets();
  test_hardware_sync_uses_selected_master_frame_number();
  test_synchronizer_and_drop_schema();
  test_optional_capture_metadata_json();
  test_bundle_groups_publish_independently();
  test_master_camera_generation_reset_resumes_lower_frame_numbers();
  test_non_master_camera_generation_reset_discards_old_group_buffers();
  test_bundle_retry_and_master_drop_are_distinct();
  test_health_json();
  test_queue_drop_newest_factory_skips_payload_creation();
  test_recorder_stop_drains_queue();
  test_health_threshold_status();
  test_health_drop_deltas_recover();
  std::cout << "camera_server_tests passed\n";
  return 0;
}
