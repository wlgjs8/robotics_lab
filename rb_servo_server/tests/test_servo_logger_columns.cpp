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
#include <cmath>
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
        // Retained reference displacement must remain visible when coverage is
        // off; it is distinct from the current tick's applied deviation.
        sample.left_force_control.covered = false;
        sample.left_force_control.reference_deviation_m = {0.011, -0.022, 0.033};
        sample.left_force_control.reference_deviation_rad = {-0.044, 0.055, -0.066};
        sample.left_force_control.reference_strip_enabled = false;
        sample.left_force_control.reference_reset_count = 9'007'199'254'740'993ULL;
        sample.right_force_control.covered = true;
        sample.right_force_control.reference_deviation_m = {-0.071, 0.082, -0.093};
        sample.right_force_control.reference_deviation_rad = {0.104, -0.115, 0.126};
        sample.right_force_control.reference_strip_enabled = true;
        sample.right_force_control.reference_reset_count = 7;
        rb_servo::Pose6D prefilter{0.11,-0.22,0.33,0,0,0};
        prefilter.quaternion_xyzw=std::array<double,4>{0.0,0.0,0.6,0.8};
        sample.left_cartesian_solve.follower_prefilter_stand=prefilter;
        sample.left_cartesian_solve.follower_sample_velocity=rb_servo::Vec6{.01,.02,.03,.04,.05,.06};
        sample.left_cartesian_solve.follower_sample_acceleration=rb_servo::Vec6{.1,.2,.3,.4,.5,.6};
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
        "init_motion_left_request_id",
        "init_motion_right_request_id",
        "left_follower_axis_duration_sec_2",
        "right_follower_target_velocity_5",
        "left_follower_prefilter_stand_qw",
        "right_follower_sample_velocity_5",
        "left_follower_sample_acceleration_0",
        "left_follower_target_acceleration_0",
        "left_follower_advance_dir_z",
        "right_follower_output_smd_reseeded",
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
        "right_fc_hold_engaged",
        "left_fc_hold_force_n",
        // the reach shell (safety.reach_constraint) — no column at all until 2026-09-04
        "left_reach_engaged",
        "left_reach_margin_m",
        "left_reach_r_far_m",
        "left_reach_shell",
        "right_reach_margin_m",
        "right_reach_r_far_m",
        "reach_clamp_count",
    };
    for (const char* name : required) {
        if (!contains(header_fields, name)) {
            std::cerr << "missing column: " << name << "\n";
            return 1;
        }
    }
    auto column = [&](const std::string& name) -> std::string {
        for (size_t i = 0; i < header_fields.size(); ++i) {
            if (header_fields[i] == name) return row_fields[i];
        }
        return {};
    };
    if (std::abs(std::stod(column("left_follower_prefilter_stand_qw"))-.8)>1e-9 ||
        std::abs(std::stod(column("left_follower_sample_velocity_5"))-.06)>1e-9 ||
        std::abs(std::stod(column("left_follower_sample_acceleration_0"))-.1)>1e-9 ||
        !column("right_follower_prefilter_stand_qw").empty() ||
        !column("right_follower_sample_velocity_5").empty()) {
        std::cerr << "sampled follower pose/derivative telemetry mismatch\n";return 1;
    }
    const char* reference_axes[] = {"x_m", "y_m", "z_m", "rx_rad", "ry_rad", "rz_rad"};
    const double reference_values[2][6] = {
        {0.011, -0.022, 0.033, -0.044, 0.055, -0.066},
        {-0.071, 0.082, -0.093, 0.104, -0.115, 0.126},
    };
    const char* sides[] = {"left", "right"};
    for (size_t side = 0; side < 2; ++side) {
        for (size_t axis = 0; axis < 6; ++axis) {
            const std::string name = std::string(sides[side]) + "_fc_reference_dev_" + reference_axes[axis];
            const std::string value = column(name);
            if (value.empty() || !std::isfinite(std::stod(value)) ||
                std::abs(std::stod(value) - reference_values[side][axis]) > 1e-9) {
                std::cerr << "incorrect reference deviation column: " << name << '\n';
                return 1;
            }
        }
    }
    if (column("left_fc_reference_strip_enabled") != "0" ||
        column("right_fc_reference_strip_enabled") != "1" ||
        column("left_fc_reference_reset_count") != "9007199254740993" ||
        column("right_fc_reference_reset_count") != "7" ||
        column("left_fc_covered") != "0" ||
        column("right_fc_covered") != "1") {
        std::cerr << "incorrect reference lifecycle telemetry\n";
        return 1;
    }
    std::cout << "servo_logger columns OK (" << header_fields.size()
              << " columns, header/row parity)\n";
    return 0;
}
