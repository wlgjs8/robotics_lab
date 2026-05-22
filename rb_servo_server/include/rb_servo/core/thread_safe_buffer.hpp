#pragma once

#include <mutex>
#include <optional>

namespace rb_servo {

template <typename T>
class ThreadSafeBuffer {
public:
    void set(const T& value) {
        std::lock_guard<std::mutex> lock(mutex_);
        value_ = value;
        has_value_ = true;
    }

    std::optional<T> get() const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!has_value_) {
            return std::nullopt;
        }
        return value_;
    }

    T getOr(const T& fallback) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!has_value_) {
            return fallback;
        }
        return value_;
    }

    bool hasValue() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return has_value_;
    }

private:
    mutable std::mutex mutex_;
    T value_{};
    bool has_value_ = false;
};

}  // namespace rb_servo
