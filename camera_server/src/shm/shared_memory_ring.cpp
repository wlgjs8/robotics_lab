#include "camera_server/shm/shared_memory_ring.hpp"

#include "camera_server/core/clock.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace camera_server {
namespace {

uint64_t align64(uint64_t v) { return (v + 63u) & ~63ull; }

bool checked_add(uint64_t a, uint64_t b, uint64_t& out) {
  if (a > std::numeric_limits<uint64_t>::max() - b) return false;
  out = a + b;
  return true;
}

bool checked_mul(uint64_t a, uint64_t b, uint64_t& out) {
  if (a != 0 && b > std::numeric_limits<uint64_t>::max() / a) return false;
  out = a * b;
  return true;
}

uint64_t payload_capacity_bytes(const CameraStreamConfig& scfg) {
  const uint64_t row_bytes = static_cast<uint64_t>(scfg.width) * bytes_per_pixel_for_format(scfg.format);
  return align64(row_bytes) * static_cast<uint64_t>(scfg.height);
}

void write_cstr(char* dst, size_t n, const std::string& value) {
  std::memset(dst, 0, n);
  std::strncpy(dst, value.c_str(), n - 1);
}

std::string read_cstr(const char* src, size_t n) {
  return std::string(src, strnlen(src, n));
}

}  // namespace

SharedMemoryRingBuffer::~SharedMemoryRingBuffer() { close(); }

void SharedMemoryRingBuffer::close() {
  if (data_) {
    munmap(data_, size_bytes_);
    data_ = nullptr;
  }
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  rings_.clear();
  size_bytes_ = 0;
  owner_ = false;
}

void* SharedMemoryRingBuffer::ptr_at(uint64_t offset) const {
  if (!data_ || offset >= size_bytes_) throw std::runtime_error("shared memory offset out of range");
  return data_ + offset;
}

void* SharedMemoryRingBuffer::ptr_range_at(uint64_t offset, uint64_t length) const {
  uint64_t end = 0;
  if (!data_ || !checked_add(offset, length, end) || end > size_bytes_) {
    throw std::runtime_error("shared memory range out of bounds");
  }
  return data_ + offset;
}

void SharedMemoryRingBuffer::create(const AppConfig& cfg) {
  close();
  name_ = cfg.shared_memory.name;
  size_bytes_ = cfg.shared_memory.size_mb * 1024ull * 1024ull;
  if (cfg.shared_memory.unlink_on_start) shm_unlink(name_.c_str());
  fd_ = shm_open(name_.c_str(), O_CREAT | O_RDWR, 0666);
  if (fd_ < 0) throw std::runtime_error("shm_open create failed for " + name_);
  if (ftruncate(fd_, static_cast<off_t>(size_bytes_)) != 0) throw std::runtime_error("ftruncate failed for " + name_);
  data_ = static_cast<uint8_t*>(mmap(nullptr, size_bytes_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0));
  if (data_ == MAP_FAILED) {
    data_ = nullptr;
    throw std::runtime_error("mmap failed for " + name_);
  }
  owner_ = true;
  std::memset(data_, 0, size_bytes_);

  auto* header = reinterpret_cast<shm_layout::ShmHeader*>(data_);
  header->magic = shm_layout::kMagic;
  header->version = shm_layout::kVersion;
  header->total_size_bytes = size_bytes_;
  header->created_time_ns = now_ns(cfg.server.clock);
  header->descriptors_offset = align64(sizeof(shm_layout::ShmHeader));

  uint32_t stream_count = 0;
  for (const auto& cam : cfg.cameras) {
    if (cam.color.enabled) ++stream_count;
    if (cam.depth.enabled) ++stream_count;
  }
  if (stream_count > shm_layout::kMaxStreams) throw std::runtime_error("too many streams for shm layout");
  header->stream_count = stream_count;

  auto* desc = reinterpret_cast<shm_layout::StreamRingHeader*>(data_ + header->descriptors_offset);
  uint64_t cursor = align64(header->descriptors_offset + stream_count * sizeof(shm_layout::StreamRingHeader));
  uint32_t idx = 0;
  auto add_stream = [&](const CameraConfig& cam, const std::string& stream_name, const CameraStreamConfig& scfg) {
    auto& d = desc[idx++];
    write_cstr(d.camera_name, sizeof(d.camera_name), cam.name);
    write_cstr(d.stream_name, sizeof(d.stream_name), stream_name);
    write_cstr(d.format, sizeof(d.format), scfg.format);
    d.width = static_cast<uint32_t>(scfg.width);
    d.height = static_cast<uint32_t>(scfg.height);
    d.channels = channels_for_format(scfg.format);
    d.bytes_per_pixel = bytes_per_pixel_for_format(scfg.format);
    d.slot_count = cfg.shared_memory.ring_slots;
    d.slot_bytes = align64(sizeof(shm_layout::FrameSlotHeader) + payload_capacity_bytes(scfg));
    d.slots_offset = cursor;
    cursor += d.slot_bytes * d.slot_count;
  };
  for (const auto& cam : cfg.cameras) {
    if (cam.color.enabled) add_stream(cam, "color", cam.color);
    if (cam.depth.enabled) add_stream(cam, "depth", cam.depth);
  }
  if (cursor > size_bytes_) throw std::runtime_error("computed shared memory layout exceeds configured size");
  load_descriptors();
}

void SharedMemoryRingBuffer::open_read_only(const std::string& name) {
  close();
  name_ = name;
  fd_ = shm_open(name_.c_str(), O_RDONLY, 0666);
  if (fd_ < 0) throw std::runtime_error("shm_open read-only failed for " + name_);
  struct stat st {};
  if (fstat(fd_, &st) != 0) throw std::runtime_error("fstat failed for " + name_);
  size_bytes_ = static_cast<uint64_t>(st.st_size);
  data_ = static_cast<uint8_t*>(mmap(nullptr, size_bytes_, PROT_READ, MAP_SHARED, fd_, 0));
  if (data_ == MAP_FAILED) {
    data_ = nullptr;
    throw std::runtime_error("mmap read-only failed for " + name_);
  }
  load_descriptors();
}

void SharedMemoryRingBuffer::load_descriptors() {
  rings_.clear();
  if (!data_ || size_bytes_ < sizeof(shm_layout::ShmHeader)) {
    throw std::runtime_error("shared memory segment is too small for header");
  }
  const auto* header = reinterpret_cast<const shm_layout::ShmHeader*>(data_);
  if (header->magic != shm_layout::kMagic || header->version != shm_layout::kVersion) {
    throw std::runtime_error("invalid shared memory magic/version");
  }
  if (header->stream_count > shm_layout::kMaxStreams) {
    throw std::runtime_error("shared memory stream count out of range");
  }
  if (header->total_size_bytes == 0 || header->total_size_bytes > size_bytes_) {
    throw std::runtime_error("shared memory total size out of range");
  }
  uint64_t descriptors_bytes = 0;
  if (!checked_mul(header->stream_count, sizeof(shm_layout::StreamRingHeader), descriptors_bytes)) {
    throw std::runtime_error("shared memory descriptors size overflows");
  }
  uint64_t descriptors_end = 0;
  if (!checked_add(header->descriptors_offset, descriptors_bytes, descriptors_end) ||
      descriptors_end > header->total_size_bytes) {
    throw std::runtime_error("shared memory descriptors out of bounds");
  }

  auto* desc = reinterpret_cast<shm_layout::StreamRingHeader*>(
      ptr_range_at(header->descriptors_offset, descriptors_bytes));
  for (uint32_t i = 0; i < header->stream_count; ++i) {
    if (desc[i].slot_count == 0 || desc[i].slot_bytes < sizeof(shm_layout::FrameSlotHeader)) {
      throw std::runtime_error("shared memory ring slot metadata is invalid");
    }
    uint64_t slots_bytes = 0;
    if (!checked_mul(desc[i].slot_count, desc[i].slot_bytes, slots_bytes)) {
      throw std::runtime_error("shared memory ring slot span overflows");
    }
    uint64_t slots_end = 0;
    if (!checked_add(desc[i].slots_offset, slots_bytes, slots_end) || slots_end > header->total_size_bytes) {
      throw std::runtime_error("shared memory ring slots out of bounds");
    }
    const std::string camera = read_cstr(desc[i].camera_name, sizeof(desc[i].camera_name));
    const std::string stream = read_cstr(desc[i].stream_name, sizeof(desc[i].stream_name));
    rings_[stream_key(camera, stream)] = RingInfo{&desc[i], 0};
  }
}

uint64_t SharedMemoryRingBuffer::slot_payload_capacity(const RingInfo& ring) const {
  if (ring.desc->slot_bytes < sizeof(shm_layout::FrameSlotHeader)) {
    throw std::runtime_error("shared memory slot size is invalid");
  }
  return ring.desc->slot_bytes - sizeof(shm_layout::FrameSlotHeader);
}

shm_layout::FrameSlotHeader* SharedMemoryRingBuffer::slot_header(const RingInfo& ring, uint32_t slot_index) const {
  if (slot_index >= ring.desc->slot_count) throw std::runtime_error("slot index out of range");
  uint64_t slot_delta = 0;
  uint64_t slot_offset = 0;
  if (!checked_mul(ring.desc->slot_bytes, slot_index, slot_delta) ||
      !checked_add(ring.desc->slots_offset, slot_delta, slot_offset)) {
    throw std::runtime_error("shared memory slot offset overflows");
  }
  return reinterpret_cast<shm_layout::FrameSlotHeader*>(ptr_range_at(slot_offset, sizeof(shm_layout::FrameSlotHeader)));
}

uint8_t* SharedMemoryRingBuffer::slot_payload(const RingInfo& ring, uint32_t slot_index) const {
  return reinterpret_cast<uint8_t*>(slot_header(ring, slot_index)) + sizeof(shm_layout::FrameSlotHeader);
}

FrameMeta SharedMemoryRingBuffer::write_frame(const std::string& camera_name, const std::string& serial,
                                              const std::string& stream_name, uint64_t frame_number,
                                              uint64_t host_arrival_time_ns, uint64_t sensor_timestamp_ns,
                                              double realsense_timestamp_ms, uint32_t stride_bytes, const uint8_t* bytes,
                                              uint32_t size_bytes) {
  std::lock_guard<std::mutex> lk(write_mutex_);
  auto it = rings_.find(stream_key(camera_name, stream_name));
  if (it == rings_.end()) throw std::runtime_error("unknown shm ring: " + stream_key(camera_name, stream_name));
  RingInfo& ring = it->second;
  const uint32_t slot_idx = ring.next_slot++ % ring.desc->slot_count;
  auto* slot = slot_header(ring, slot_idx);
  const uint64_t expected = slot_payload_capacity(ring);
  if (size_bytes > expected) throw std::runtime_error("frame too large for ring slot");

  uint64_t begin = slot->seq_begin.load(std::memory_order_relaxed);
  if ((begin % 2u) == 0u) ++begin;
  else begin += 2u;
  slot->seq_begin.store(begin, std::memory_order_release);
  std::memcpy(slot_payload(ring, slot_idx), bytes, size_bytes);
  slot->frame_number = frame_number;
  slot->host_arrival_time_ns = host_arrival_time_ns;
  slot->sensor_timestamp_ns = sensor_timestamp_ns;
  slot->write_time_ns = now_ns();
  slot->width = ring.desc->width;
  slot->height = ring.desc->height;
  slot->stride_bytes = stride_bytes != 0 ? stride_bytes : ring.desc->width * ring.desc->bytes_per_pixel;
  slot->size_bytes = size_bytes;
  slot->valid = 1;
  const uint64_t complete = begin + 1u;
  slot->seq_end.store(complete, std::memory_order_release);
  slot->seq_begin.store(complete, std::memory_order_release);

  FrameMeta meta;
  meta.camera_name = camera_name;
  meta.serial = serial;
  meta.stream = stream_name;
  meta.frame_number = frame_number;
  meta.host_arrival_time_ns = host_arrival_time_ns;
  meta.sensor_timestamp_ns = sensor_timestamp_ns;
  meta.realsense_timestamp_ms = realsense_timestamp_ms;
  meta.width = static_cast<int>(ring.desc->width);
  meta.height = static_cast<int>(ring.desc->height);
  meta.stride_bytes = static_cast<int>(slot->stride_bytes);
  meta.format = read_cstr(ring.desc->format, sizeof(ring.desc->format));
  meta.shm_name = name_;
  meta.ring_name = stream_key(camera_name, stream_name);
  meta.slot_index = slot_idx;
  meta.shm_offset = ring.desc->slots_offset + ring.desc->slot_bytes * slot_idx + sizeof(shm_layout::FrameSlotHeader);
  meta.size_bytes = size_bytes;
  meta.seq = complete;
  meta.valid = true;
  return meta;
}

std::optional<ShmReadResult> SharedMemoryRingBuffer::read_slot(const std::string& ring_name, uint32_t slot_index,
                                                               uint32_t max_retries) const {
  auto it = rings_.find(ring_name);
  if (it == rings_.end()) return std::nullopt;
  const RingInfo& ring = it->second;
  if (slot_index >= ring.desc->slot_count) return std::nullopt;
  auto* slot = slot_header(ring, slot_index);
  std::vector<uint8_t> bytes;
  for (uint32_t retry = 0; retry < max_retries; ++retry) {
    const uint64_t a = slot->seq_begin.load(std::memory_order_acquire);
    if ((a % 2u) == 1u || slot->valid == 0) continue;
    const uint32_t size = slot->size_bytes;
    if (size > slot_payload_capacity(ring)) return std::nullopt;
    bytes.resize(size);
    std::memcpy(bytes.data(), slot_payload(ring, slot_index), size);
    const uint64_t b = slot->seq_end.load(std::memory_order_acquire);
    const uint64_t c = slot->seq_begin.load(std::memory_order_acquire);
    if (a == b && b == c && (b % 2u) == 0u) {
      FrameMeta meta;
      const std::string camera = read_cstr(ring.desc->camera_name, sizeof(ring.desc->camera_name));
      const std::string stream = read_cstr(ring.desc->stream_name, sizeof(ring.desc->stream_name));
      meta.camera_name = camera;
      meta.stream = stream;
      meta.ring_name = ring_name;
      meta.shm_name = name_;
      meta.frame_number = slot->frame_number;
      meta.host_arrival_time_ns = slot->host_arrival_time_ns;
      meta.sensor_timestamp_ns = slot->sensor_timestamp_ns;
      meta.width = static_cast<int>(slot->width);
      meta.height = static_cast<int>(slot->height);
      meta.stride_bytes = static_cast<int>(slot->stride_bytes);
      meta.size_bytes = slot->size_bytes;
      meta.slot_index = slot_index;
      meta.shm_offset = ring.desc->slots_offset + ring.desc->slot_bytes * slot_index + sizeof(shm_layout::FrameSlotHeader);
      meta.format = read_cstr(ring.desc->format, sizeof(ring.desc->format));
      meta.seq = b;
      meta.valid = true;
      return ShmReadResult{meta, std::move(bytes)};
    }
  }
  return std::nullopt;
}

}  // namespace camera_server
