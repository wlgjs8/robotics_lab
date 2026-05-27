#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>

namespace camera_server {

template <typename T>
class BoundedQueue {
 public:
  explicit BoundedQueue(size_t capacity = 1) : capacity_(capacity == 0 ? 1 : capacity) {}

  bool try_push_drop_oldest(T value) {
    return try_push_drop_oldest_with_factory([&value] { return std::move(value); });
  }

  bool try_push_drop_newest(T value) {
    return try_push_drop_newest_with_factory([&value] { return std::move(value); });
  }

  template <typename Factory>
  bool try_push_drop_oldest_with_factory(Factory make_value) {
    std::lock_guard<std::mutex> lk(mu_);
    if (closed_) return false;
    bool dropped = false;
    if (queue_.size() >= capacity_) {
      ++dropped_count_;
      queue_.pop_front();
      dropped = true;
    }
    queue_.push_back(make_value());
    cv_.notify_one();
    return !dropped;
  }

  template <typename Factory>
  bool try_push_drop_newest_with_factory(Factory make_value) {
    std::lock_guard<std::mutex> lk(mu_);
    if (closed_) return false;
    if (queue_.size() >= capacity_) {
      ++dropped_count_;
      return false;
    }
    queue_.push_back(make_value());
    cv_.notify_one();
    return true;
  }

  std::optional<T> pop_wait() {
    std::unique_lock<std::mutex> lk(mu_);
    cv_.wait(lk, [&] { return closed_ || !queue_.empty(); });
    if (queue_.empty()) return std::nullopt;
    T value = std::move(queue_.front());
    queue_.pop_front();
    return value;
  }

  std::optional<T> try_pop() {
    std::lock_guard<std::mutex> lk(mu_);
    if (queue_.empty()) return std::nullopt;
    T value = std::move(queue_.front());
    queue_.pop_front();
    return value;
  }

  void close() {
    std::lock_guard<std::mutex> lk(mu_);
    closed_ = true;
    cv_.notify_all();
  }

  size_t size() const {
    std::lock_guard<std::mutex> lk(mu_);
    return queue_.size();
  }

  uint64_t dropped_count() const {
    std::lock_guard<std::mutex> lk(mu_);
    return dropped_count_;
  }

 private:
  size_t capacity_;
  mutable std::mutex mu_;
  std::condition_variable cv_;
  std::deque<T> queue_;
  bool closed_{false};
  uint64_t dropped_count_{0};
};

}  // namespace camera_server
