// The servo CSV header and row are emitted by two hand-maintained streams of
// `<<` in servo_logger.cpp. Nothing structurally ties them together, so adding a
// column to one and forgetting the other silently shifts every field to its
// right -- and the log is the primary evidence for every hardware diagnosis we
// run (latency, lead budgets, fault forensics), so a silent shift corrupts
// conclusions rather than crashing anything. This test pins them to the same
// width.

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <unistd.h>

#include "rb_servo/logging/servo_logger.hpp"

namespace {

#define RB_CHECK(expr) \
    do { \
        if (!(expr)) { \
            std::cerr << "CHECK failed: " #expr << " at " << __FILE__ << ":" << __LINE__ << "\n"; \
            return false; \
        } \
    } while (0)

// Counts top-level commas, honouring the RFC4180 quoting csvEscape() emits, so a
// quoted field containing a comma is not miscounted as a separator.
std::size_t countFields(const std::string& line) {
    std::size_t fields = 1;
    bool in_quotes = false;
    for (std::size_t i = 0; i < line.size(); ++i) {
        const char c = line[i];
        if (c == '"') {
            if (in_quotes && i + 1 < line.size() && line[i + 1] == '"') {
                ++i;  // escaped quote inside a quoted field
            } else {
                in_quotes = !in_quotes;
            }
        } else if (c == ',' && !in_quotes) {
            ++fields;
        }
    }
    return fields;
}

bool testHeaderAndRowHaveTheSameWidth() {
    const std::string dir =
        "/tmp/rb-servo-logger-csv-" + std::to_string(::getpid());
    std::filesystem::remove_all(dir);

    rb_servo::LoggingConfig config;
    config.enable = true;
    config.directory = dir;
    config.flush_period_ms = 20;

    {
        rb_servo::ServoLogger logger(config);
        RB_CHECK(logger.start());
        // push() try-locks and DROPS on contention by design (it must never stall
        // the RT servo tick), so a single sample is not guaranteed to land. Push
        // a burst instead and assert on the rows that survive.
        //
        // A default-constructed sample exercises every column's "absent" branch,
        // which is where optional-field writers most often drop a separator.
        for (int i = 0; i < 200; ++i) {
            logger.push(rb_servo::ServoSample{});
            std::this_thread::sleep_for(std::chrono::microseconds(200));
        }
        logger.stop();
    }

    std::string csv_path;
    for (const auto& entry : std::filesystem::directory_iterator(dir)) {
        const std::string name = entry.path().filename().string();
        if (entry.is_regular_file() && name.rfind("servo_log_", 0) == 0) {
            csv_path = entry.path().string();
            break;
        }
    }
    RB_CHECK(!csv_path.empty());

    std::ifstream file(csv_path);
    std::string header;
    RB_CHECK(static_cast<bool>(std::getline(file, header)));
    const std::size_t header_fields = countFields(header);

    std::string row;
    std::size_t rows_checked = 0;
    while (std::getline(file, row)) {
        if (row.empty()) continue;
        const std::size_t row_fields = countFields(row);
        if (header_fields != row_fields) {
            std::cerr << "servo CSV width mismatch on row " << rows_checked + 1
                      << ": header has " << header_fields << " fields, row has "
                      << row_fields << "\n";
            return false;
        }
        ++rows_checked;
    }
    RB_CHECK(rows_checked > 0);

    // Guard the specific columns this test was added alongside, so a future
    // rename does not quietly drop the box-queue observer from telemetry.
    for (const char* column : {"left_box_queue_fill", "right_box_queue_fill",
                               "left_box_queue_fill_samples",
                               "right_box_queue_fill_unparsed"}) {
        if (header.find(column) == std::string::npos) {
            std::cerr << "servo CSV header is missing column: " << column << "\n";
            return false;
        }
    }

    std::filesystem::remove_all(dir);
    return true;
}

}  // namespace

int main() {
    if (!testHeaderAndRowHaveTheSameWidth()) return 1;
    return 0;
}
