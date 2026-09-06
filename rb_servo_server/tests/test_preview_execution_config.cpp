#include "rb_servo/config/config.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <unistd.h>

namespace {
const std::filesystem::path stack = std::filesystem::path(__FILE__).parent_path().parent_path() /
                                    "config/stack_real.yaml";
YAML::Node follower(YAML::Node root) {
    return root["cartesian_control"]["tcp_pose_target_profiles"]["flow_infer_preview"]["ruckig_follower"];
}
bool rejects(const YAML::Node& root, const std::string& text) {
    const auto path = std::filesystem::temp_directory_path() /
        ("rb-preview-config-" + std::to_string(getpid()) + ".yaml");
    { std::ofstream file(path); file << root; }
    bool rejected = false;
    try { (void)rb_servo::loadConfigFromYaml(path.string()); }
    catch (const std::exception& e) {
        rejected = std::string(e.what()).find(text) != std::string::npos;
        if (!rejected) std::cerr << "Unexpected rejection: " << e.what() << '\n';
    }
    std::filesystem::remove(path);
    return rejected;
}
#define CHECK(condition) do { if (!(condition)) { std::cerr << "Failed: " #condition << " at " << __LINE__ << '\n'; return 1; } } while (0)
}
int main() {
    const auto cfg = rb_servo::loadConfigFromYaml(stack.string());
    const rb_servo::RuckigFollowerConfig* fresh = nullptr;
    const rb_servo::RuckigFollowerConfig* preview = nullptr;
    for (const auto& profile : cfg.cartesian_control.tcp_pose_target_profiles) {
        if (profile.name == "flow_infer_fresh") fresh = &profile.ruckig_follower;
        if (profile.name == "flow_infer_preview") preview = &profile.ruckig_follower;
    }
    CHECK(fresh && preview);
    CHECK(!fresh->preview_execution.enable && fresh->output_smd.enable);
    CHECK(preview->preview_execution.enable && !preview->output_smd.enable);
    CHECK(preview->preview_execution.cursor.enable);
    const auto& p = preview->preview_execution;
    CHECK(p.tracker.max_linear_velocity_m_s == preview->max_linear_velocity_m_s);
    CHECK(p.tracker.max_linear_acceleration_m_s2 == preview->max_linear_accel_m_s2);
    CHECK(p.tracker.max_linear_jerk_m_s3 == preview->max_linear_jerk_m_s3);
    CHECK(p.tracker.max_angular_velocity_rad_s == preview->max_angular_velocity_rad_s);
    CHECK(p.tracker.max_angular_acceleration_rad_s2 == preview->max_angular_accel_rad_s2);
    CHECK(p.tracker.max_angular_jerk_rad_s3 == preview->max_angular_jerk_rad_s3);
    CHECK(cfg.cartesian_control.tcp_pose_target_profile_default != "flow_infer_preview");
    for (const char* key : {"replan_period_sec", "splice_lead_sec", "max_result_age_sec",
                            "worker_poll_period_sec", "max_source_rows", "tracker", "cursor"}) {
        auto root = YAML::LoadFile(stack.string());
        follower(root)["preview_execution"].remove(key);
        CHECK(rejects(root, std::string(key) + " is required"));
    }
    const char* tracker_keys[] = {"planning_dt_sec", "horizon_steps", "linear_tracking_scale_m",
        "angular_tracking_scale_rad", "jerk_weight", "jerk_difference_weight",
        "linear_tracking_tolerance_m", "angular_tracking_tolerance_rad",
        "max_linear_tracking_slack_m", "max_angular_tracking_slack_rad",
        "max_reference_chart_angle_rad", "feasibility_tolerance",
        "max_working_set_recalculations", "max_solve_time_sec"};
    for (const char* key : tracker_keys) {
        auto root = YAML::LoadFile(stack.string());
        follower(root)["preview_execution"]["tracker"].remove(key);
        CHECK(rejects(root, std::string(key) + " is required"));
    }
    for (const char* key : {"enable", "max_backlog_sec", "catchup_time_sec", "max_rate",
                           "translation_velocity_floor", "angular_velocity_floor"}) {
        auto root = YAML::LoadFile(stack.string());
        follower(root)["preview_execution"]["cursor"].remove(key);
        CHECK(rejects(root, std::string(key) + " is required"));
    }
    for (const char* key : {"fresh_chunk_replan", "continuous_hold_resume", "plan_leash_enable"}) {
        auto root = YAML::LoadFile(stack.string()); follower(root)[key] = false;
        CHECK(rejects(root, "preview_execution"));
    }
    {
        auto root = YAML::LoadFile(stack.string());
        follower(root)["output_smd"]["enable"] = true;
        CHECK(rejects(root, "output_smd disabled"));
    }
    {
        auto root = YAML::LoadFile(stack.string());
        follower(root)["preview_execution"]["cursor"]["enable"] = false;
        CHECK(rejects(root, "cursor"));
    }
    {
        auto root = YAML::LoadFile(stack.string());
        follower(root)["preview_execution"]["max_result_age_sec"] = 0.01;
        CHECK(rejects(root, "inconsistent"));
    }
    for (int count : {-1, 0, 65}) {
        auto root = YAML::LoadFile(stack.string());
        follower(root)["preview_execution"]["max_source_rows"] = count;
        CHECK(rejects(root, "max_source_rows"));
    }
    for (const char* key : {"replan_period_sec", "splice_lead_sec", "max_result_age_sec", "worker_poll_period_sec"}) {
        auto root = YAML::LoadFile(stack.string());
        follower(root)["preview_execution"][key] = YAML::Load(".nan");
        CHECK(rejects(root, key));
    }
    {
        auto root = YAML::LoadFile(stack.string());
        follower(root)["preview_execution"]["tracker"]["max_linear_velocity_m_s"] = 1.0;
        CHECK(rejects(root, "max_linear_velocity_m_s"));
    }
    {
        auto root = YAML::LoadFile(stack.string());
        follower(root)["preview_execution"]["cursor"]["max_backlog_sec"] = 1.0;
        CHECK(rejects(root, "bounded history"));
    }
    std::cout << "preview execution config checks passed\n";
}
