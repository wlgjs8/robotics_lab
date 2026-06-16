#include "rb_servo/logging/servo_logger.hpp"

#include <ctime>
#include <filesystem>
#include <iostream>
#include <string>

namespace rb_servo {
namespace {

// Per-run local-time stamp, matching the policy_runner action-log convention
// (actions_%Y%m%d_%H%M%S.jsonl). Local time = wall-clock (Korea time when the
// host is set to KST), so runs sort and read naturally.
std::string runStamp() {
    std::time_t now = std::time(nullptr);
    std::tm tm_local{};
    localtime_r(&now, &tm_local);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm_local);
    return std::string(buf);
}

}  // namespace

ServoLogger::ServoLogger(const LoggingConfig& config) : config_(config) {}

ServoLogger::~ServoLogger() {
    stop();
}

bool ServoLogger::start() {
    if (!config_.enable) return true;
    if (running_) return true;

    std::filesystem::create_directories(config_.directory);
    // One file per run: servo_log_<YYYYMMDD_HHMMSS>.csv (no longer truncated/
    // overwritten each run). `servo_log.csv` is kept as a symlink to the latest
    // run so existing tooling/acceptance scripts that read the fixed name still
    // resolve to the current run.
    const std::string run_name = "servo_log_" + runStamp() + ".csv";
    file_.open(config_.directory + "/" + run_name, std::ios::out | std::ios::trunc);
    if (!file_) {
        std::cerr << "[ERROR] failed to open servo log file\n";
        return false;
    }
    const std::filesystem::path latest = std::filesystem::path(config_.directory) / "servo_log.csv";
    std::error_code ec;
    std::filesystem::remove(latest, ec);  // clear any prior file/symlink
    std::filesystem::create_symlink(run_name, latest, ec);  // relative target
    if (ec) {
        std::cerr << "[WARN] servo log: could not update servo_log.csv symlink: "
                  << ec.message() << "\n";
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
    file_ << ",fault_context_verdict,fault_context_domain,fault_context_arm,fault_context_backend_op,fault_context_backend_error_kind,fault_context_backend_error_name,fault_context_backend_error_code,fault_context_retryable,fault_context_recoverable,fault_context_robot_fault,fault_context_transport_fault,fault_context_state_after_source,fault_context_reason";
    file_ << ",left_send_start_ns,left_send_end_ns,right_send_start_ns,right_send_end_ns,send_skew_us,left_send_duration_us,right_send_duration_us";
    file_ << ",left_ack_policy,right_ack_policy,left_ack_observed,right_ack_observed,left_controller_acceptance_observed,right_controller_acceptance_observed,left_ack_wait_duration_us,right_ack_wait_duration_us,left_rbpodo_waiting_ack,right_rbpodo_waiting_ack,left_send_acceptance_semantics,right_send_acceptance_semantics";
    file_ << ",left_state_age_us,right_state_age_us,left_send_result_age_us,right_send_result_age_us";
    file_ << ",left_send_within_period,right_send_within_period,left_send_period_overrun,right_send_period_overrun,left_send_command_deadline_missed,right_send_command_deadline_missed";
    file_ << ",left_send_deadline_hit,right_send_deadline_hit,dispatch_skew_us,left_worker_loop_read_duration_us,right_worker_loop_read_duration_us";
    for (int i = 0; i < kDof; ++i) file_ << ",left_q_actual_" << i;
    for (int i = 0; i < kDof; ++i) file_ << ",right_q_actual_" << i;
    for (int i = 0; i < kDof; ++i) file_ << ",left_q_sent_" << i;
    for (int i = 0; i < kDof; ++i) file_ << ",right_q_sent_" << i;
    file_ << ",left_error_code,right_error_code\n";
}

namespace {
double ageUs(uint64_t newer_ns, uint64_t older_ns) {
    if (newer_ns == 0 || older_ns == 0 || newer_ns < older_ns) return 0.0;
    return static_cast<double>(newer_ns - older_ns) / 1000.0;
}

bool sendWithinPeriod(const ServoSample& sample, uint64_t send_end_ns) {
    if (sample.loop_start_time_ns == 0 || send_end_ns == 0 || sample.period_ms <= 0.0) return false;
    const auto period_ns = static_cast<uint64_t>(sample.period_ms * 1'000'000.0);
    return send_end_ns <= sample.loop_start_time_ns + period_ns;
}

bool sendPeriodOverrun(const ServoSample& sample, uint64_t send_end_ns) {
    if (sample.loop_start_time_ns == 0 || send_end_ns == 0 || sample.period_ms <= 0.0) return false;
    return !sendWithinPeriod(sample, send_end_ns);
}

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
          << sample.right_send_ok << ',';
    if (sample.latched_fault_context.has_value()) {
        const LatchedFaultContextSnapshot& context = *sample.latched_fault_context;
        file_ << context.verdict << ','
              << context.domain << ','
              << context.arm << ','
              << context.backend_op << ','
              << context.backend_error_kind << ','
              << csvEscape(context.backend_error_name) << ','
              << csvEscape(context.backend_error_code) << ','
              << context.retryable << ','
              << context.recoverable << ','
              << context.robot_fault << ','
              << context.transport_fault << ','
              << context.state_after_source << ','
              << csvEscape(context.reason) << ',';
    } else {
        file_ << ",,,,,,,,,,,,,";
    }
    file_ << sample.left_send_start_ns << ','
          << sample.left_send_end_ns << ','
          << sample.right_send_start_ns << ','
          << sample.right_send_end_ns << ','
          << sample.send_skew_us << ','
          << sample.left_send_duration_us << ','
          << sample.right_send_duration_us << ','
          << toString(sample.left_last_send.ack_policy) << ','
          << toString(sample.right_last_send.ack_policy) << ','
          << sample.left_last_send.ack_observed << ','
          << sample.right_last_send.ack_observed << ','
          << sample.left_last_send.controller_acceptance_observed << ','
          << sample.right_last_send.controller_acceptance_observed << ','
          << sample.left_last_send.ack_wait_duration_us << ','
          << sample.right_last_send.ack_wait_duration_us << ','
          << sample.left_last_send.rbpodo_waiting_ack << ','
          << sample.right_last_send.rbpodo_waiting_ack << ','
          << csvEscape(sample.left_last_send.acceptance_semantics) << ','
          << csvEscape(sample.right_last_send.acceptance_semantics) << ','
          << ageUs(sample.loop_end_time_ns, sample.left_state.host_time_ns) << ','
          << ageUs(sample.loop_end_time_ns, sample.right_state.host_time_ns) << ','
          << ageUs(sample.loop_end_time_ns, sample.left_send_end_ns) << ','
          << ageUs(sample.loop_end_time_ns, sample.right_send_end_ns) << ','
          << sendWithinPeriod(sample, sample.left_send_end_ns) << ','
          << sendWithinPeriod(sample, sample.right_send_end_ns) << ','
          << sendPeriodOverrun(sample, sample.left_send_end_ns) << ','
          << sendPeriodOverrun(sample, sample.right_send_end_ns) << ','
          << "" << ','
          << "" << ','
          << sendWithinPeriod(sample, sample.left_send_end_ns) << ','
          << sendWithinPeriod(sample, sample.right_send_end_ns) << ','
          << sample.send_skew_us << ','
          << sample.left_last_read.duration_us << ','
          << sample.right_last_read.duration_us;
    for (double v : sample.left_state.q_actual_deg) file_ << ',' << v;
    for (double v : sample.right_state.q_actual_deg) file_ << ',' << v;
    for (double v : sample.left_sent_q_deg) file_ << ',' << v;
    for (double v : sample.right_sent_q_deg) file_ << ',' << v;
    file_ << ',' << sample.left_state.error_code << ',' << sample.right_state.error_code << '\n';
}

}  // namespace rb_servo
