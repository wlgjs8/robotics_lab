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
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

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
  assert(json.find("\"complete\":true") != std::string::npos);
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
  h.stream_stats["head.color"].frame_count = 3;
  h.stream_stats["head.color"].internal_queue_drop_count = 1;
  h.stream_stats["head.color"].recorder_drop_count = 2;
  h.shm_name = "/camera_server_unit_test";
  auto json = health_to_json(h);
  assert(json.find("camera_server.health.v1") != std::string::npos);
  assert(json.find("\"status\":\"degraded\"") != std::string::npos);
  assert(json.find("\"status_reasons\"") != std::string::npos);
  assert(json.find("\"status_color\":\"degraded\"") != std::string::npos);
  assert(json.find("\"drops_color\":{\"frame_number_gap\":0,\"internal_queue\":1,\"recorder\":2,\"total\":3}") != std::string::npos);
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

int main() {
  test_config();
  test_shared_memory_roundtrip();
  test_shared_memory_rejects_oversized_slot_metadata();
  test_shared_memory_open_rejects_invalid_slot_offsets();
  test_hardware_sync_uses_selected_master_frame_number();
  test_synchronizer_and_drop_schema();
  test_health_json();
  test_queue_drop_newest_factory_skips_payload_creation();
  test_recorder_stop_drains_queue();
  test_health_threshold_status();
  std::cout << "camera_server_tests passed\n";
  return 0;
}
