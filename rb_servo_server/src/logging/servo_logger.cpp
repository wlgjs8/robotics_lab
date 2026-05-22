#include "rb_servo/logging/servo_logger.hpp"

#include <filesystem>
#include <iostream>
#include <string>

namespace rb_servo {

ServoLogger::ServoLogger(const LoggingConfig& config) : config_(config) {}

ServoLogger::~ServoLogger() {
    stop();
}

bool ServoLogger::start() {
    if (!config_.enable) return true;
    if (running_) return true;

    std::filesystem::create_directories(config_.directory);
    file_.open(config_.directory + "/servo_log.csv", std::ios::out | std::ios::trunc);
    if (!file_) {
        std::cerr << "[ERROR] failed to open servo log file\n";
        return false;
    }
    writeHeader();

    running_ = true;
    thread_ = std::thread(&ServoLogger::threadMain, this);
    return true;
}

void ServoLogger::stop() {
    if (!running_) {
        if (file_.is_open()) file_.close();
        return;
    }
    running_ = false;
    cv_.notify_all();
    if (thread_.joinable()) {
        thread_.join();
    }
    if (file_.is_open()) {
        file_.flush();
        file_.close();
    }
}

void ServoLogger::push(const ServoSample& sample) {
    if (!config_.enable || !running_) return;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (config_.queue_capacity == 0) {
            dropped_samples_.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        if (queue_.size() >= config_.queue_capacity) {
            queue_.pop_front();
            dropped_samples_.fetch_add(1, std::memory_order_relaxed);
        }
        queue_.push_back(sample);
    }
    cv_.notify_one();
}

uint64_t ServoLogger::droppedSamples() const {
    return dropped_samples_.load(std::memory_order_relaxed);
}

void ServoLogger::threadMain() {
    while (running_) {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait_for(lock, std::chrono::milliseconds(config_.flush_period_ms), [&] {
            return !queue_.empty() || !running_;
        });
        while (!queue_.empty()) {
            ServoSample sample = queue_.front();
            queue_.pop_front();
            lock.unlock();
            writeSample(sample);
            lock.lock();
        }
        if (file_) file_.flush();
    }
}

void ServoLogger::writeHeader() {
    file_ << "tick,loop_start_time_ns,loop_end_time_ns,period_ms,jitter_ms,filter_dt_ms,safety_verdict,motion_state,fault_latched,fault_reason,logger_dropped_samples,command_seq,left_mode,right_mode,left_send_ok,right_send_ok";
    file_ << ",left_send_start_ns,left_send_end_ns,right_send_start_ns,right_send_end_ns,send_skew_us,left_send_duration_us,right_send_duration_us";
    for (int i = 0; i < kDof; ++i) file_ << ",left_q_actual_" << i;
    for (int i = 0; i < kDof; ++i) file_ << ",right_q_actual_" << i;
    for (int i = 0; i < kDof; ++i) file_ << ",left_q_sent_" << i;
    for (int i = 0; i < kDof; ++i) file_ << ",right_q_sent_" << i;
    file_ << ",left_error_code,right_error_code\n";
}

namespace {
std::string csvEscape(const std::string& value) {
    bool quote = false;
    for (char c : value) {
        if (c == '"' || c == ',' || c == '\n' || c == '\r') {
            quote = true;
            break;
        }
    }
    if (!quote) return value;

    std::string out = "\"";
    for (char c : value) {
        if (c == '"') out += '"';
        out += c;
    }
    out += '"';
    return out;
}
}  // namespace

void ServoLogger::writeSample(const ServoSample& sample) {
    file_ << sample.tick << ','
          << sample.loop_start_time_ns << ','
          << sample.loop_end_time_ns << ','
          << sample.period_ms << ','
          << sample.jitter_ms << ','
          << sample.filter_dt_ms << ','
          << toString(sample.safety_verdict) << ','
          << toString(sample.motion_state) << ','
          << sample.fault_latched << ','
          << csvEscape(sample.fault_reason) << ','
          << droppedSamples() << ','
          << sample.command.seq << ','
          << toString(sample.command.left.mode) << ','
          << toString(sample.command.right.mode) << ','
          << sample.left_send_ok << ','
          << sample.right_send_ok << ','
          << sample.left_send_start_ns << ','
          << sample.left_send_end_ns << ','
          << sample.right_send_start_ns << ','
          << sample.right_send_end_ns << ','
          << sample.send_skew_us << ','
          << sample.left_send_duration_us << ','
          << sample.right_send_duration_us;
    for (double v : sample.left_state.q_actual_deg) file_ << ',' << v;
    for (double v : sample.right_state.q_actual_deg) file_ << ',' << v;
    for (double v : sample.left_sent_q_deg) file_ << ',' << v;
    for (double v : sample.right_sent_q_deg) file_ << ',' << v;
    file_ << ',' << sample.left_state.error_code << ',' << sample.right_state.error_code << '\n';
}

}  // namespace rb_servo
