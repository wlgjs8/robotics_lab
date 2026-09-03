// Header/row column-count parity for the servo CSV logger.
//
// writeHeader() and writeSample() are maintained by hand in parallel; a column
// added to one and not the other silently SHIFTS every later column, which
// poisons offline analysis (scripts/analyze_smoothness.py,
// scripts/analyze_box_latency.py resolve columns by name). This test writes
// one real sample through the production logger and asserts the row has
// exactly as many CSV fields as the header, and that the columns added for
// wire/projection observability are present.

#include <unistd.h>

#include <cassert>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "rb_servo/config/config.hpp"
#include "rb_servo/core/types.hpp"
#include "rb_servo/logging/servo_logger.hpp"

namespace {

// Quote-aware CSV field splitter (the logger escapes strings with quotes).
std::vector<std::string> splitCsv(const std::string& line) {
    std::vector<std::string> out;
    std::string field;
    bool quoted = false;
    for (size_t i = 0; i < line.size(); ++i) {
        const char c = line[i];
        if (quoted) {
            if (c == '"') {
                if (i + 1 < line.size() && line[i + 1] == '"') {
                    field += '"';
                    ++i;
                } else {
                    quoted = false;
                }
            } else {
                field += c;
            }
        } else if (c == '"') {
            quoted = true;
        } else if (c == ',') {
            out.push_back(field);
            field.clear();
        } else {
            field += c;
        }
    }
    out.push_back(field);
    return out;
}

bool contains(const std::vector<std::string>& fields, const std::string& name) {
    for (const auto& f : fields) {
        if (f == name) return true;
    }
    return false;
}

}  // namespace

int main() {
    namespace fs = std::filesystem;
    const fs::path dir =
        fs::temp_directory_path() /
        ("rb-servo-logger-columns-test-" + std::to_string(::getpid()));
    fs::create_directories(dir);

    {
        rb_servo::LoggingConfig config;
        config.enable = true;
        config.directory = dir.string();
        config.flush_period_ms = 10;
        rb_servo::ServoLogger logger(config);
        if (!logger.start()) {
            std::cerr << "logger.start() failed\n";
            return 1;
        }
        rb_servo::ServoSample sample;
        sample.tick = 1;
        sample.loop_start_time_ns = 1'000'000'000;
        sample.loop_end_time_ns = 1'000'100'000;
        sample.left_worker_telemetry.worker_pending_overwrites_total = 3;
        sample.left_worker_telemetry.worker_repeated_sends_total = 2;
        sample.left_worker_telemetry.worker_wire_dispatches_total = 7;
        sample.left_worker_telemetry.worker_last_wire_send_start_ns = 42;
        sample.safety_projection.active = true;
        sample.safety_projection.constraint_count = 2;
        sample.safety_projection.ceiling_clamped = true;
        sample.safety_projection.min_margin_m = 0.004;
        // Push REPEATEDLY, not once. ServoLogger::push() takes the ring mutex with
        // try_to_lock and DROPS the sample if the writer thread holds it -- deliberate,
        // documented RT behaviour, since a servo tick must never stall to log itself.
        // A single push that loses that race leaves nothing to read back, which is what
        // made this test fail about 4 runs in 10 (measured 2026-09-03, under the load of
        // the full suite). The rows are identical, and only the first data row is read.
        for (int i = 0; i < 32; ++i) {
            logger.push(sample);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        logger.stop();
    }

    std::string header;
    std::string row;
    for (const auto& entry : fs::directory_iterator(dir)) {
        if (entry.path().extension() != ".csv") continue;
        if (fs::is_symlink(entry.path())) continue;
        std::ifstream in(entry.path());
        std::getline(in, header);
        std::getline(in, row);
        break;
    }
    fs::remove_all(dir);

    if (header.empty() || row.empty()) {
        std::cerr << "no csv content written\n";
        return 1;
    }
    const auto header_fields = splitCsv(header);
    const auto row_fields = splitCsv(row);
    if (header_fields.size() != row_fields.size()) {
        std::cerr << "column count mismatch: header=" << header_fields.size()
                  << " row=" << row_fields.size() << "\n";
        return 1;
    }
    const char* required[] = {
        "left_worker_pending_overwrites_total",
        "left_worker_repeated_sends_total",
        "left_worker_wire_dispatches_total",
        "left_worker_wire_send_start_ns",
        "right_worker_wire_send_end_ns",
        "projection_active",
        "projection_ceiling_clamped",
        "projection_min_margin_m",
        "selfcol_verdict_age_ms",
        // the force fold (force_control.fold_deviation)
        "left_fc_folded",
        "left_fc_fold_sink",
        "right_fc_fold_z_m",
        "right_fc_absorbed_norm_m",
        "left_fc_absorbed_norm_rad",
    };
    for (const char* name : required) {
        if (!contains(header_fields, name)) {
            std::cerr << "missing column: " << name << "\n";
            return 1;
        }
    }
    std::cout << "servo_logger columns OK (" << header_fields.size()
              << " columns, header/row parity)\n";
    return 0;
}
