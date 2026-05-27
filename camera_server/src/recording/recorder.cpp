#include "camera_server/recording/recorder.hpp"

#include "camera_server/publish/metadata_publisher.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace camera_server {

Recorder::Recorder(RecordingConfig cfg) : cfg_(std::move(cfg)), queue_(cfg_.queue_capacity_frames) {}
Recorder::~Recorder() { stop(); }

void Recorder::start() {
  if (!cfg_.enabled || running_.exchange(true)) return;
  std::filesystem::create_directories(cfg_.output_dir);
  const uint32_t threads = cfg_.writer_threads == 0 ? 1 : cfg_.writer_threads;
  for (uint32_t i = 0; i < threads; ++i) workers_.emplace_back(&Recorder::worker_loop, this, i);
}

void Recorder::stop() {
  if (!running_.exchange(false)) return;
  queue_.close();
  for (auto& t : workers_) {
    if (t.joinable()) t.join();
  }
  workers_.clear();
}

bool Recorder::enqueue(RecordingFrame frame) {
  if (!cfg_.enabled || !running_) return true;
  if (cfg_.drop_policy == "drop_newest") return queue_.try_push_drop_newest(std::move(frame));
  return queue_.try_push_drop_oldest(std::move(frame));
}

bool Recorder::enqueue_copy(FrameMeta meta, const uint8_t* data, size_t size_bytes) {
  if (!cfg_.enabled) return true;
  if (!running_) return false;
  if (data == nullptr && size_bytes != 0) return false;
  auto make_frame = [&] {
    RecordingFrame frame;
    frame.meta = std::move(meta);
    frame.bytes.assign(data, data + size_bytes);
    return frame;
  };
  if (cfg_.drop_policy == "drop_newest") {
    return queue_.try_push_drop_newest_with_factory(make_frame);
  }
  return queue_.try_push_drop_oldest_with_factory(make_frame);
}

void Recorder::worker_loop(size_t) {
  while (true) {
    auto item = queue_.pop_wait();
    if (!item) break;
    const auto seq = file_seq_.fetch_add(1);
    const std::filesystem::path dir = std::filesystem::path(cfg_.output_dir) / item->meta.camera_name / item->meta.stream;
    std::filesystem::create_directories(dir);
    std::ostringstream filename;
    filename << std::setw(12) << std::setfill('0') << seq << "_"
             << item->meta.host_arrival_time_ns << "_fn" << item->meta.frame_number << "." << item->meta.format;
    const std::filesystem::path raw = dir / filename.str();
    std::ofstream ofs(raw, std::ios::binary);
    if (ofs) ofs.write(reinterpret_cast<const char*>(item->bytes.data()), static_cast<std::streamsize>(item->bytes.size()));

    const std::filesystem::path metadata = std::filesystem::path(cfg_.output_dir) / "camera_metadata.jsonl";
    {
      std::lock_guard<std::mutex> lk(metadata_mutex_);
      std::ofstream meta(metadata, std::ios::app);
      if (!meta) continue;
      std::string line = frame_to_json(item->meta);
      if (!line.empty() && line.back() == '}') line.pop_back();
      meta << line << ",\"recording_file\":\"" << raw.string() << "\",\"recording_seq\":" << seq << "}\n";
    }
  }
}

}  // namespace camera_server
