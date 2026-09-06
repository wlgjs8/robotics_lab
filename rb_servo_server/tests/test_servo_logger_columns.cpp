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
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
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

void populateDetailedPreviewFixture(rb_servo::PreviewExecutionTelemetry& p) {
    p.gate_revision = 9007199254740993ULL;
    p.gauge_revision = 9007199254740995ULL;
    p.parent_plan_id = 9007199254740997ULL;
    p.request_id = 9007199254740999ULL;
    p.result_valid = true;
    p.result_solve_attempted = true;
    p.last_worker_status = "fixture_last_worker_status,\"quoted\"";
    p.last_solve_status = "fixture_last_solve_status,\"quoted\"";
    p.last_admission_reason = "fixture_last_admission_reason,\"quoted\"";
    p.result_request_id = 9007199254741011ULL;
    p.result_epoch = 9007199254741013ULL;
    p.result_gate_revision = 9007199254741015ULL;
    p.result_gauge_revision = 9007199254741017ULL;
    p.result_source_wire_seq = 9007199254741019ULL;
    p.result_source_recv_seq = 9007199254741021ULL;
    p.result_parent_plan_id = 9007199254741023ULL;
    p.result_gauge_transported = 9007199254741025ULL;
    p.staged_gauge_transported = 9007199254741027ULL;
    p.gauge_transport_failed = 9007199254741029ULL;
    p.result_generated_at_sec = 0.002;
    p.result_splice_at_sec = 0.0021;
    p.result_valid_until_sec = 0.0022;
    p.result_completed_at_sec = 0.0023;
    p.result_observed_at_sec = 0.0024;
    p.solve_iterations = 19;
    p.solve_contact_constrained = true;
    p.solve_contact_decomposed = true;
    p.solve_contact_coupled_fallback = true;
    p.solve_max_constraint_violation = 0.0029;
    p.solve_max_contact_velocity_violation_m_s = 0.003;
    p.ready_not_staged = 9007199254741053ULL;
    p.staged_identity_rejected = 9007199254741055ULL;
    p.staged_expired = 9007199254741057ULL;
    p.staged_sample_rejected = 9007199254741059ULL;
    p.staged_contact_rejected = 9007199254741061ULL;
    p.last_staged_cancel_reason = "fixture_last_staged_cancel_reason,\"quoted\"";
    p.last_staged_cancel_time_sec = 0.0037;
    p.last_staged_cancel_request_id = 9007199254741067ULL;
    p.last_admission_time_sec = 0.0039;
    p.last_admission_gap_sec = 0.004;
    p.last_admitted_request_id = 9007199254741073ULL;
    p.last_admitted_parent_plan_id = 9007199254741075ULL;
    p.last_brake_reason = "fixture_last_brake_reason,\"quoted\"";
    p.last_brake_start_time_sec = 0.0044;
    p.last_brake_origin_sec = 0.0045;
    p.angular_continuations_started = 9007199254741083ULL;
    p.angular_brakes_started = 9007199254741085ULL;
    p.last_contact_reject_time_sec = 0.0048;
    p.last_contact_reject_gate = 0.0049;
    p.last_contact_reject_closing_m_s = 0.005;
    p.last_contact_reject_allowed_m_s = 0.0051;
    p.fold_count = 9007199254741095ULL;
    p.fold_force_count = 9007199254741097ULL;
    p.fold_roi_floor_count = 9007199254741099ULL;
    p.fold_geometry_hold_count = 9007199254741101ULL;
    p.fold_unknown_count = 9007199254741103ULL;
    p.fold_booked_time_ns = 9007199254741105ULL;
    p.fold_applied_time_ns = 9007199254741107ULL;
    p.fold_revision = 9007199254741109ULL;
    p.fold_geometry_cause_mask = 13;
    p.pending_geometry_fold_valid = true;
    p.pending_geometry_fold_time_ns = 9007199254749991ULL;
    p.pending_geometry_fold_cause_mask = 6;
    p.pending_geometry_fold_translation_m = {-0.051,0.052,-0.053};
    p.pending_geometry_fold_quaternion_xyzw = {0.6,0.0,0.0,0.8};
    p.request_invalid = 9007199254741113ULL;
    p.request_mailbox_full = 9007199254741115ULL;
    p.request_coalesced = 9007199254741117ULL;
    p.result_publish_dropped = 9007199254741119ULL;
    p.result_coalesced = 9007199254741121ULL;
    p.solve_angular_norm_coupled = true;
    p.solve_angular_norm_cuts = 9007199254748881ULL;
    p.solve_max_angular_chart_velocity_norm = 1.23;
    p.solve_max_angular_chart_acceleration_norm = 2.34;
    p.result_initial_linear_velocity_max_m_s = 0.345;
    p.result_initial_linear_acceleration_max_m_s2 = 4.56;
    p.result_initial_angular_velocity_norm_rad_s = 0.789;
    p.result_initial_angular_acceleration_norm_rad_s2 = 7.89;
    p.fold_cause = rb_servo::PreviewFoldCause::GeometryHold;
    for (std::size_t i=0; i<p.worker_status_counts.size(); ++i) p.worker_status_counts[i] = 9007199254741993ULL + 0*100 + i*2;
    for (std::size_t i=0; i<p.solve_status_counts.size(); ++i) p.solve_status_counts[i] = 9007199254741993ULL + 1*100 + i*2;
    for (std::size_t i=0; i<p.result_checks.size(); ++i) p.result_checks[i] = 9007199254741993ULL + 2*100 + i*2;
    for (std::size_t i=0; i<p.staged_cancel_counts.size(); ++i) p.staged_cancel_counts[i] = 9007199254741993ULL + 3*100 + i*2;
    for (std::size_t i=0; i<p.brake_counts.size(); ++i) p.brake_counts[i] = 9007199254741993ULL + 4*100 + i*2;
    p.last_contact_reject_normal = {0.011,0.012,0.013};
    p.fold_translation_m = {0.021,0.022,0.023};
    p.fold_quaternion_xyzw = {0,0,0.6,0.8};
    p.fold_booked_translation_m = {0.041,0.042,0.043};
    p.fold_booked_quaternion_xyzw = {0,0,0.6,0.8};
    p.gauge_translation_m = {0.101,-0.202,0.303};
    p.gauge_quaternion_xyzw = {0.0,0.8,0.0,0.6};
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
        sample.safety_projection.joint_stage_trace_valid = true;
        for (std::size_t side = 0; side < 2; ++side) {
            for (std::size_t j = 0; j < 6; ++j) {
                sample.safety_projection.requested_q_deg[side][j] = 100 * side + j + .1;
                sample.safety_projection.projected_q_deg[side][j] = 100 * side + j + .2;
                sample.safety_projection.released_q_deg[side][j] = 100 * side + j + .3;
            }
        }
        sample.left_force_control.smd_gate_sample_valid = true;
        sample.left_force_control.smd_gate_releasing = true;
        sample.left_force_control.smd_gate_translation = .75;
        sample.left_force_control.smd_gate_normal_stand = {0, 0, 1};
        sample.left_force_control.smd_gate_measured_force_stand_n = {.1, -.2, .3};
        sample.left_force_control.smd_gate_removed_velocity_m_s = .04;
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
        sample.left_cartesian_solve.follower_jerk_scale = .375;
        sample.left_cartesian_solve.follower_jerk_search_calculations = 6;
        auto& p = sample.left_cartesian_solve.preview_execution;
        p.enabled = true; p.active = true; p.status = "tracking";
        p.sample_time_ns = 9'007'199'254'740'993ULL;
        p.epoch = 7; p.plan_id = 11; p.source_wire_seq = 13; p.source_recv_seq = 17;
        p.backlog_sec = .012; p.rate = 1.03; p.plan_age_sec = .024;
        p.accepted_position_error_m = .00015; p.accepted_rotation_error_rad = .00025;
        p.solve_time_sec = .0004; p.submitted = 23; p.accepted = 19;
        p.rejected = 3; p.expired = 2; p.contact_guard_count = 5;
        populateDetailedPreviewFixture(p);
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
    for (const auto& name : header_fields) {
        if ((name.rfind("left_preview_execution_", 0) == 0 ||
             name.rfind("right_preview_execution_", 0) == 0) &&
            std::count(header_fields.begin(), header_fields.end(), name) != 1) {
            std::cerr << "duplicate preview diagnostic column: " << name << '\n';
            return 1;
        }
    }
    const char* required[] = {
        "init_motion_left_request_id",
        "init_motion_right_request_id",
        "left_follower_axis_duration_sec_2",
        "left_follower_jerk_scale",
        "right_follower_jerk_search_calculations",
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
    for (const char* side : {"left", "right"}) {
        for (const char* field : {"enabled","active","status","sample_time_ns","epoch","plan_id",
             "source_wire_seq","source_recv_seq","backlog_sec","rate","plan_age_sec",
             "accepted_position_error_m","accepted_rotation_error_rad","solve_time_sec",
             "submitted","accepted","rejected","expired","contact_guard_count"}) {
            const std::string name = std::string(side) + "_preview_execution_" + field;
            if (std::count(header_fields.begin(), header_fields.end(), name) != 1) {
                std::cerr << "missing or duplicate preview column: " << name << '\n'; return 1;
            }
        }
    }
    for (const auto& item : std::vector<std::pair<std::string, std::string>>{
             {"enabled","1"},{"active","1"},{"status","tracking"},{"sample_time_ns","9007199254740993"},
             {"epoch","7"},{"plan_id","11"},{"source_wire_seq","13"},{"source_recv_seq","17"},
             {"submitted","23"},{"accepted","19"},{"rejected","3"},{"expired","2"},{"contact_guard_count","5"}}) {
        if (column("left_preview_execution_" + item.first) != item.second) {
            std::cerr << "incorrect preview identity/counter column: " << item.first << '\n'; return 1;
        }
    }
    for (const auto& item : std::vector<std::pair<std::string, double>>{
             {"backlog_sec",.012},{"rate",1.03},{"plan_age_sec",.024},
             {"accepted_position_error_m",.00015},{"accepted_rotation_error_rad",.00025},{"solve_time_sec",.0004}}) {
        if (std::abs(std::stod(column("left_preview_execution_" + item.first)) - item.second) > 1e-9) {
            std::cerr << "incorrect preview timing/error column: " << item.first << '\n'; return 1;
        }
    }
    static_assert(std::is_trivially_copyable_v<rb_servo::PreviewExecutionTelemetry>,
                  "Preview telemetry must remain fixed-size and copyable without allocation");
    auto checkDetailedColumn = [&](const std::string& suffix, const std::string& expected) {
        const std::string name = "left_preview_execution_" + suffix;
        if (std::count(header_fields.begin(), header_fields.end(), name) != 1 || column(name) != expected) {
            std::cerr << "incorrect detailed preview field: " << name << '\n'; return false;
        }
        return true;
    };
    if (!checkDetailedColumn("gate_revision", "9007199254740993")) return 1;
    if (!checkDetailedColumn("gauge_revision", "9007199254740995")) return 1;
    if (!checkDetailedColumn("parent_plan_id", "9007199254740997")) return 1;
    if (!checkDetailedColumn("request_id", "9007199254740999")) return 1;
    if (!checkDetailedColumn("result_valid", "1")) return 1;
    if (!checkDetailedColumn("result_solve_attempted", "1")) return 1;
    if (!checkDetailedColumn("last_worker_status", "fixture_last_worker_status,\"quoted\"")) return 1;
    if (!checkDetailedColumn("last_solve_status", "fixture_last_solve_status,\"quoted\"")) return 1;
    if (!checkDetailedColumn("last_admission_reason", "fixture_last_admission_reason,\"quoted\"")) return 1;
    if (!checkDetailedColumn("result_request_id", "9007199254741011")) return 1;
    if (!checkDetailedColumn("result_epoch", "9007199254741013")) return 1;
    if (!checkDetailedColumn("result_gate_revision", "9007199254741015")) return 1;
    if (!checkDetailedColumn("result_gauge_revision", "9007199254741017")) return 1;
    if (!checkDetailedColumn("result_source_wire_seq", "9007199254741019")) return 1;
    if (!checkDetailedColumn("result_source_recv_seq", "9007199254741021")) return 1;
    if (!checkDetailedColumn("result_parent_plan_id", "9007199254741023")) return 1;
    if (!checkDetailedColumn("result_gauge_transported", "9007199254741025")) return 1;
    if (!checkDetailedColumn("staged_gauge_transported", "9007199254741027")) return 1;
    if (!checkDetailedColumn("gauge_transport_failed", "9007199254741029")) return 1;
    if (std::abs(std::stod(column("left_preview_execution_result_generated_at_sec")) - 0.002) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_result_splice_at_sec")) - 0.0021) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_result_valid_until_sec")) - 0.0022) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_result_completed_at_sec")) - 0.0023) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_result_observed_at_sec")) - 0.0024) > 1e-9) return 1;
    if (!checkDetailedColumn("solve_iterations", "19")) return 1;
    if (!checkDetailedColumn("solve_contact_constrained", "1")) return 1;
    if (!checkDetailedColumn("solve_contact_decomposed", "1")) return 1;
    if (!checkDetailedColumn("solve_contact_coupled_fallback", "1")) return 1;
    if (std::abs(std::stod(column("left_preview_execution_solve_max_constraint_violation")) - 0.0029) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_solve_max_contact_velocity_violation_m_s")) - 0.003) > 1e-9) return 1;
    if (!checkDetailedColumn("ready_not_staged", "9007199254741053")) return 1;
    if (!checkDetailedColumn("staged_identity_rejected", "9007199254741055")) return 1;
    if (!checkDetailedColumn("staged_expired", "9007199254741057")) return 1;
    if (!checkDetailedColumn("staged_sample_rejected", "9007199254741059")) return 1;
    if (!checkDetailedColumn("staged_contact_rejected", "9007199254741061")) return 1;
    if (!checkDetailedColumn("last_staged_cancel_reason", "fixture_last_staged_cancel_reason,\"quoted\"")) return 1;
    if (std::abs(std::stod(column("left_preview_execution_last_staged_cancel_time_sec")) - 0.0037) > 1e-9) return 1;
    if (!checkDetailedColumn("last_staged_cancel_request_id", "9007199254741067")) return 1;
    if (std::abs(std::stod(column("left_preview_execution_last_admission_time_sec")) - 0.0039) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_last_admission_gap_sec")) - 0.004) > 1e-9) return 1;
    if (!checkDetailedColumn("last_admitted_request_id", "9007199254741073")) return 1;
    if (!checkDetailedColumn("last_admitted_parent_plan_id", "9007199254741075")) return 1;
    if (!checkDetailedColumn("last_brake_reason", "fixture_last_brake_reason,\"quoted\"")) return 1;
    if (std::abs(std::stod(column("left_preview_execution_last_brake_start_time_sec")) - 0.0044) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_last_brake_origin_sec")) - 0.0045) > 1e-9) return 1;
    if (!checkDetailedColumn("angular_continuations_started", "9007199254741083")) return 1;
    if (!checkDetailedColumn("angular_brakes_started", "9007199254741085")) return 1;
    if (std::abs(std::stod(column("left_preview_execution_last_contact_reject_time_sec")) - 0.0048) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_last_contact_reject_gate")) - 0.0049) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_last_contact_reject_closing_m_s")) - 0.005) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_last_contact_reject_allowed_m_s")) - 0.0051) > 1e-9) return 1;
    if (!checkDetailedColumn("fold_count", "9007199254741095")) return 1;
    if (!checkDetailedColumn("fold_force_count", "9007199254741097")) return 1;
    if (!checkDetailedColumn("fold_roi_floor_count", "9007199254741099")) return 1;
    if (!checkDetailedColumn("fold_geometry_hold_count", "9007199254741101")) return 1;
    if (!checkDetailedColumn("fold_unknown_count", "9007199254741103")) return 1;
    if (!checkDetailedColumn("fold_booked_time_ns", "9007199254741105")) return 1;
    if (!checkDetailedColumn("fold_applied_time_ns", "9007199254741107")) return 1;
    if (!checkDetailedColumn("fold_revision", "9007199254741109")) return 1;
    if (!checkDetailedColumn("fold_geometry_cause_mask", "13")) return 1;
    if (!checkDetailedColumn("pending_geometry_fold_valid", "1")) return 1;
    if (!checkDetailedColumn("pending_geometry_fold_time_ns", "9007199254749991")) return 1;
    if (!checkDetailedColumn("pending_geometry_fold_cause_mask", "6")) return 1;
    if (std::abs(std::stod(column("left_preview_execution_pending_geometry_fold_translation_x_m")) - (-0.051)) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_pending_geometry_fold_translation_y_m")) - (0.052)) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_pending_geometry_fold_translation_z_m")) - (-0.053)) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_pending_geometry_fold_quaternion_qx")) - (0.6)) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_pending_geometry_fold_quaternion_qy")) - (0.0)) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_pending_geometry_fold_quaternion_qz")) - (0.0)) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_pending_geometry_fold_quaternion_qw")) - (0.8)) > 1e-9) return 1;

    if (!checkDetailedColumn("request_invalid", "9007199254741113")) return 1;
    if (!checkDetailedColumn("request_mailbox_full", "9007199254741115")) return 1;
    if (!checkDetailedColumn("request_coalesced", "9007199254741117")) return 1;
    if (!checkDetailedColumn("result_publish_dropped", "9007199254741119")) return 1;
    if (!checkDetailedColumn("result_coalesced", "9007199254741121")) return 1;
    if (!checkDetailedColumn("solve_angular_norm_coupled", "1")) return 1;
    if (!checkDetailedColumn("solve_angular_norm_cuts", "9007199254748881")) return 1;
    if (std::abs(std::stod(column("left_preview_execution_solve_max_angular_chart_velocity_norm")) - 1.23) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_solve_max_angular_chart_acceleration_norm")) - 2.34) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_result_initial_linear_velocity_max_m_s")) - 0.345) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_result_initial_linear_acceleration_max_m_s2")) - 4.56) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_result_initial_angular_velocity_norm_rad_s")) - 0.789) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_result_initial_angular_acceleration_norm_rad_s2")) - 7.89) > 1e-9) return 1;
    if (!checkDetailedColumn("fold_cause", "geometry_hold")) return 1;
    for (std::size_t i=0; i<rb_servo::kPreviewWorkerStatusNames.size(); ++i) {
        if (!checkDetailedColumn(std::string("worker_status_") + rb_servo::kPreviewWorkerStatusNames[i] + "_count",
                                 std::to_string(9007199254741993ULL + 0*100 + i*2))) return 1;
    }
    for (std::size_t i=0; i<rb_servo::kPreviewSolveStatusNames.size(); ++i) {
        if (!checkDetailedColumn(std::string("solve_status_") + rb_servo::kPreviewSolveStatusNames[i] + "_count",
                                 std::to_string(9007199254741993ULL + 1*100 + i*2))) return 1;
    }
    for (std::size_t i=0; i<rb_servo::kPreviewResultCheckNames.size(); ++i) {
        if (!checkDetailedColumn(std::string("result_check_") + rb_servo::kPreviewResultCheckNames[i] + "_count",
                                 std::to_string(9007199254741993ULL + 2*100 + i*2))) return 1;
    }
    for (std::size_t i=0; i<rb_servo::kPreviewStagedCancelNames.size(); ++i) {
        if (!checkDetailedColumn(std::string("staged_cancel_") + rb_servo::kPreviewStagedCancelNames[i] + "_count",
                                 std::to_string(9007199254741993ULL + 3*100 + i*2))) return 1;
    }
    for (std::size_t i=0; i<rb_servo::kPreviewBrakeCauseNames.size(); ++i) {
        if (!checkDetailedColumn(std::string("brake_") + rb_servo::kPreviewBrakeCauseNames[i] + "_count",
                                 std::to_string(9007199254741993ULL + 4*100 + i*2))) return 1;
    }
    if (std::abs(std::stod(column("left_preview_execution_last_contact_reject_normal_x")) - 0.011) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_last_contact_reject_normal_y")) - 0.012) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_last_contact_reject_normal_z")) - 0.013) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_fold_translation_x_m")) - 0.021) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_fold_translation_y_m")) - 0.022) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_fold_translation_z_m")) - 0.023) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_fold_quaternion_qx")) - 0) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_fold_quaternion_qy")) - 0) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_fold_quaternion_qz")) - 0.6) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_fold_quaternion_qw")) - 0.8) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_fold_booked_translation_x_m")) - 0.041) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_fold_booked_translation_y_m")) - 0.042) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_fold_booked_translation_z_m")) - 0.043) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_fold_booked_quaternion_qx")) - 0) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_fold_booked_quaternion_qy")) - 0) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_fold_booked_quaternion_qz")) - 0.6) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_fold_booked_quaternion_qw")) - 0.8) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_gauge_translation_x_m")) - 0.101) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_gauge_translation_y_m")) - -0.202) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_gauge_translation_z_m")) - 0.303) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_gauge_quaternion_qx")) - 0) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_gauge_quaternion_qy")) - 0.8) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_gauge_quaternion_qz")) - 0) > 1e-9) return 1;
    if (std::abs(std::stod(column("left_preview_execution_gauge_quaternion_qw")) - 0.6) > 1e-9) return 1;
    for (const char* name : {"gate_revision", "gauge_revision", "result_request_id", "result_gauge_transported",
                             "fold_count", "fold_applied_time_ns", "request_coalesced", "result_publish_dropped"}) {
        if (column(std::string("right_preview_execution_") + name) != "0") return 1;
    }
    if (column("right_preview_execution_fold_cause") != "unknown" ||
        column("right_preview_execution_last_worker_status") != "not_observed" ||
        std::stod(column("right_preview_execution_fold_quaternion_qw")) != 1.0) return 1;

    if (column("right_preview_execution_enabled") != "0" ||
        column("right_preview_execution_active") != "0" ||
        column("right_preview_execution_status") != "disabled" ||
        column("right_preview_execution_plan_id") != "0" ||
        column("right_preview_execution_sample_time_ns") != "0" ||
        std::stod(column("right_preview_execution_rate")) != 1.0) {
        std::cerr << "inactive preview columns carry unexpected state\n"; return 1;
    }
    if (std::abs(std::stod(column("left_follower_prefilter_stand_qw"))-.8)>1e-9 ||
        std::abs(std::stod(column("left_follower_jerk_scale"))-.375)>1e-9 ||
        column("left_follower_jerk_search_calculations") != "6" ||
        std::abs(std::stod(column("right_follower_jerk_scale"))-1.)>1e-9 ||
        std::abs(std::stod(column("left_follower_sample_velocity_5"))-.06)>1e-9 ||
        std::abs(std::stod(column("left_follower_sample_acceleration_0"))-.1)>1e-9 ||
        !column("right_follower_prefilter_stand_qw").empty() ||
        !column("right_follower_sample_velocity_5").empty()) {
        std::cerr << "sampled follower pose/derivative telemetry mismatch\n";return 1;
    }
    if (column("projection_joint_stage_trace_valid") != "1" ||
        column("left_smd_gate_sample_valid") != "1" ||
        column("left_smd_gate_releasing") != "1" ||
        column("left_smd_gate_armed") != "0" ||
        column("right_smd_gate_sample_valid") != "0") return 1;
    for (const auto& item : std::vector<std::pair<std::string, double>>{
             {"left_smd_gate_translation", .75}, {"left_smd_gate_normal_z", 1},
             {"left_smd_gate_measured_fx_n", .1}, {"left_smd_gate_measured_fy_n", -.2},
             {"left_smd_gate_measured_fz_n", .3}, {"left_smd_gate_removed_velocity_m_s", .04}}) {
        if (std::abs(std::stod(column(item.first)) - item.second) > 1e-9) return 1;
    }
    for (std::size_t side = 0; side < 2; ++side) {
        const std::string prefix = side == 0 ? "left_" : "right_";
        int stage_index = 1;
        for (const char* stage : {"projection_requested_q_deg_", "projection_solved_q_deg_", "projection_released_q_deg_"}) {
            for (std::size_t j = 0; j < 6; ++j) {
                const std::string name = prefix + stage + std::to_string(j);
                if (std::count(header_fields.begin(), header_fields.end(), name) != 1 ||
                    std::abs(std::stod(column(name)) - (100 * side + j + .1 * stage_index)) > 1e-9) return 1;
            }
            ++stage_index;
        }
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
