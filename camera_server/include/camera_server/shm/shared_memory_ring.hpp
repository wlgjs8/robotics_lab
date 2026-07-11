#pragma once

#include "camera_server/config/config.hpp"
#include "camera_server/core/types.hpp"
#include "camera_server/shm/shm_layout.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace camera_server {

struct ShmReadResult {
  FrameMeta meta;
  std::vector<uint8_t> bytes;
};

class SharedMemoryRingBuffer {
 public:
  SharedMemoryRingBuffer() = default;
  ~SharedMemoryRingBuffer();
  SharedMemoryRingBuffer(const SharedMemoryRingBuffer&) = delete;
  SharedMemoryRingBuffer& operator=(const SharedMemoryRingBuffer&) = delete;

  void create(const AppConfig& cfg);
  void open_read_only(const std::string& name);
  void close();

  FrameMeta write_frame(const std::string& camera_name, const std::string& serial,
                        const std::string& stream_name, uint64_t frame_number,
                        uint64_t host_arrival_time_ns, uint64_t sensor_timestamp_ns,
                        double realsense_timestamp_ms, uint32_t stride_bytes, const uint8_t* bytes,
                        uint32_t size_bytes);

  std::optional<ShmReadResult> read_slot(const std::string& ring_name, uint32_t slot_index,
                                         uint32_t max_retries = 1000) const;

  const std::string& name() const { return name_; }
  uint64_t size_bytes() const { return size_bytes_; }
  bool valid() const { return data_ != nullptr; }

 private:
  struct RingInfo {
    shm_layout::StreamRingHeader* desc{nullptr};
    uint32_t next_slot{0};
    std::shared_ptr<std::mutex> write_mutex{std::make_shared<std::mutex>()};
  };

  void* ptr_at(uint64_t offset) const;
  void* ptr_range_at(uint64_t offset, uint64_t length) const;
  uint64_t slot_payload_capacity(const RingInfo& ring) const;
  shm_layout::FrameSlotHeader* slot_header(const RingInfo& ring, uint32_t slot_index) const;
  uint8_t* slot_payload(const RingInfo& ring, uint32_t slot_index) const;
  void load_descriptors();

  std::string name_;
  int fd_{-1};
  uint8_t* data_{nullptr};
  uint64_t size_bytes_{0};
  bool owner_{false};
  std::map<std::string, RingInfo> rings_;
};

}  // namespace camera_server
