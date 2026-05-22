#pragma once

#include <atomic>
#include <cstdint>

namespace camera_server::shm_layout {

constexpr uint32_t kMagic = 0x43534D52;  // CSMR
constexpr uint32_t kVersion = 1;
constexpr uint32_t kMaxStreams = 16;
constexpr uint32_t kNameBytes = 32;
constexpr uint32_t kStreamBytes = 16;
constexpr uint32_t kFormatBytes = 16;

struct ShmHeader {
  uint32_t magic;
  uint32_t version;
  uint32_t stream_count;
  uint32_t reserved0;
  uint64_t total_size_bytes;
  uint64_t created_time_ns;
  uint64_t descriptors_offset;
};

struct StreamRingHeader {
  char camera_name[kNameBytes];
  char stream_name[kStreamBytes];
  char format[kFormatBytes];
  uint32_t width;
  uint32_t height;
  uint32_t channels;
  uint32_t bytes_per_pixel;
  uint32_t slot_count;
  uint32_t reserved0;
  uint64_t slot_bytes;
  uint64_t slots_offset;
};

struct FrameSlotHeader {
  std::atomic<uint64_t> seq_begin;
  std::atomic<uint64_t> seq_end;
  uint64_t frame_number;
  uint64_t host_arrival_time_ns;
  uint64_t sensor_timestamp_ns;
  uint64_t write_time_ns;
  uint32_t width;
  uint32_t height;
  uint32_t stride_bytes;
  uint32_t size_bytes;
  uint32_t valid;
  uint32_t reserved;
};

}  // namespace camera_server::shm_layout
