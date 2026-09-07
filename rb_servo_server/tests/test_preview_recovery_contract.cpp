// Recovery's public YAML/wire/state/CSV contracts, without backends or sockets.
#include "rb_servo/config/config.hpp"
#include "rb_servo/core/types.hpp"
#include "rb_servo/logging/servo_logger.hpp"
#include "rb_servo/network/chunk_frame_receiver.hpp"
#include "rb_servo/network/state_publisher.hpp"

#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>
#include <unistd.h>

namespace {
using namespace rb_servo;
using json = nlohmann::json;
const auto stack = std::filesystem::path(__FILE__).parent_path().parent_path() / "config/stack_real.yaml";
constexpr uint64_t big = 9007199254740993ULL; // Must not pass through an IEEE754 double.
#define CHECK(x) do { if (!(x)) throw std::runtime_error(std::string(#x) + " at line " + std::to_string(__LINE__)); } while (0)

struct TempDirectory {
    std::filesystem::path path = std::filesystem::temp_directory_path() /
        ("rb-preview-recovery-contract-" + std::to_string(getpid()));
    TempDirectory() { std::filesystem::create_directories(path); }
    ~TempDirectory() { std::error_code e; std::filesystem::remove_all(path, e); }
};
YAML::Node preview(YAML::Node root) {
    return root["cartesian_control"]["tcp_pose_target_profiles"]["flow_infer_preview"]
               ["ruckig_follower"]["preview_execution"];
}
void requireRejected(const YAML::Node& node, const std::string& reason, const TempDirectory& dir) {
    const auto file = dir.path / "invalid.yaml";
    { std::ofstream out(file); out << node; }
    try { (void)loadConfigFromYaml(file.string()); }
    catch (const std::exception& error) {
        CHECK(std::string(error.what()).find(reason) != std::string::npos);
        return;
    }
    throw std::runtime_error("config accepted invalid recovery contract: " + reason);
}
void checkConfig(const TempDirectory& dir) {
    auto cfg = loadConfigFromYaml(stack.string());
    const auto it = std::find_if(cfg.cartesian_control.tcp_pose_target_profiles.begin(),
        cfg.cartesian_control.tcp_pose_target_profiles.end(),
        [](const auto& p) { return p.name == "flow_infer_preview"; });
    CHECK(it != cfg.cartesian_control.tcp_pose_target_profiles.end());
    CHECK(it->ruckig_follower.preview_execution.recovery.enable);
    CHECK(!it->ruckig_follower.output_smd.enable);
    for (const char* key : {"enable", "fresh_plan_timeout_sec", "max_attempts"}) {
        auto root = YAML::LoadFile(stack.string());
        preview(root)["recovery"].remove(key);
        requireRejected(root, std::string(key) + " is required", dir);
    }
    {
        auto root = YAML::LoadFile(stack.string());
        preview(root)["cursor"].remove("phase_lookahead_sec");
        requireRejected(root, "phase_lookahead_sec is required", dir);
    }
    for (const auto value : {"0", "-1", ".nan", ".inf", "0.05"}) {
        auto root = YAML::LoadFile(stack.string());
        preview(root)["recovery"]["fresh_plan_timeout_sec"] = YAML::Load(value);
        requireRejected(root, "recovery", dir);
    }
    for (int value : {-1, 0, 101}) {
        auto root = YAML::LoadFile(stack.string());
        preview(root)["recovery"]["max_attempts"] = value;
        requireRejected(root, "recovery", dir);
    }
    {
        auto root = YAML::LoadFile(stack.string());
        preview(root)["enable"] = false;
        requireRejected(root, "recovery requires preview_execution.enable=true", dir);
    }
    {
        auto root = YAML::LoadFile(stack.string());
        preview(root)["cursor"]["phase_lookahead_sec"] = 0.051;
        requireRejected(root, "phase_lookahead_sec", dir);
    }
}

json packet() {
    const json poses = json::array({json::array({0., 0., .3, 0., 0., 0., 1., 50.})});
    const json deltas = json::array({json::array({.001, 0., 0., 0., 0., 0., 50.})});
    return {{"schema_version", "robotics_lab.chunk_overlay.v3"}, {"seq", 1},
            {"policy_dt_sec", .0334}, {"horizon", 1}, {"left", poses}, {"right", poses},
            {"left_delta", deltas}, {"right_delta", deltas},
            {"chunk_metadata", {{"observation_step_seq", 0}, {"activation_step_seq", 0},
                 {"source_start_index", 0}, {"original_horizon", 1}, {"selected_horizon", 1},
                 {"proprio", {{"valid", true}}}, {"preview_recovery_epoch", big},
                 {"observation_time_ns", big + 2}}}};
}
ChunkFrameReceiver::Frame parse(const json& packet) {
    const auto text = packet.dump();
    ChunkFrameReceiver::Frame frame;
    CHECK(ChunkFrameReceiver::parsePacket(text.data(), text.size(), &frame));
    return frame;
}
void checkChunkWire() {
    const auto good = parse(packet());
    CHECK(good.schema_generation == 3 && good.chunk_metadata_present && good.proprio_valid);
    CHECK(good.recovery_metadata_present);
    CHECK(good.preview_recovery_epoch == big && good.observation_time_ns == big + 2);
    CHECK(good.has_left_delta && good.has_right_delta);
    for (const char* field : {"preview_recovery_epoch", "observation_time_ns"}) {
        auto missing = packet(); missing["chunk_metadata"].erase(field);
        CHECK(!parse(missing).recovery_metadata_present);
        for (const json& bad : std::vector<json>{nullptr, true, -1, 1.0, "1", json::array()}) {
            auto changed = packet(); changed["chunk_metadata"][field] = bad;
            CHECK(!parse(changed).recovery_metadata_present);
        }
    }
    // Zero remains an explicit unsigned wire value; stop-barrier/current-epoch
    // authority is checked by the native coordinator, not invented by parsing.
    auto zero = packet(); zero["chunk_metadata"]["preview_recovery_epoch"] = 0;
    zero["chunk_metadata"]["observation_time_ns"] = 0;
    const auto explicit_zero = parse(zero);
    CHECK(explicit_zero.recovery_metadata_present && explicit_zero.observation_time_ns == 0);
}

PreviewRecoveryTelemetry recoveryFixture() {
    PreviewRecoveryTelemetry r;
    r.enabled = true; r.state = PreviewRecoveryState::Paused; r.cause = PreviewRecoveryCause::PlanExpired;
    r.epoch = big; r.sample_time_ns = big + 2; r.min_observation_time_ns = big - 2;
    r.started_time_ns = big - 10; r.state_started_time_ns = big - 4;
    r.attempts = 3; r.completed = 2; r.rejected_frames = 7;
    r.abandoned_source_wire_seq = big + 4; r.abandoned_source_recv_seq = big + 6;
    r.candidate_source_wire_seq = big + 8; r.candidate_source_recv_seq = big + 10;
    r.abandoned_backlog_sec = .099;
    r.abandoned_position_error_m = {.011, .022};
    r.abandoned_rotation_error_rad = {.033, .044};
    return r;
}
void checkStateWire() {
    auto cfg = loadConfigFromYaml(stack.string()); cfg.gripper.enable = false;
    StatePublisher publisher(cfg); // serialize only: no sockets, no publisher thread.
    ServoSnapshot s; s.preview_recovery = recoveryFixture();
    for (const auto state : {PreviewRecoveryState::Tracking, PreviewRecoveryState::Braking,
                            PreviewRecoveryState::WaitingFresh, PreviewRecoveryState::Starting,
                            PreviewRecoveryState::Paused}) {
        s.preview_recovery.state = state;
        const auto message = json::parse(publisher.serializeSnapshot(s));
        const auto& r = message.at("preview_recovery");
        CHECK(r.at("enabled").is_boolean() && r.at("enabled").get<bool>());
        CHECK(r.at("state") == toString(state)); CHECK(r.at("reason") == "plan_expired");
        CHECK(r.at("epoch").is_number_unsigned() && r.at("epoch").get<uint64_t>() == big);
        CHECK(r.at("sample_time_ns").get<uint64_t>() == big + 2);
        CHECK(r.at("min_observation_time_ns").get<uint64_t>() == big - 2);
        CHECK(r.at("attempts") == 3 && r.at("completed") == 2 && r.at("rejected_frames") == 7);
        CHECK(r.at("candidate_source_wire_seq").get<uint64_t>() == big + 8);
        CHECK(!message.at("fault_latched").get<bool>()); // PolicyPaused is not a hardware latch.
        int supported = 0;
        for (const auto& p : message.at("chunk_execution_profiles")) if (p.at("name") == "flow_infer_preview") {
            CHECK(p.at("preview_recovery").is_boolean() && p.at("preview_recovery").get<bool>());
            CHECK(p.at("gripper_state_max_age_sec").get<double>() > 0.0); ++supported;
        }
        CHECK(supported == 1);
    }
}

std::vector<std::string> splitCsv(const std::string& text) {
    std::vector<std::string> fields; std::string field; bool quoted = false;
    for (std::size_t i = 0; i < text.size(); ++i) {
        const char c = text[i];
        if (c == '"') {
            if (quoted && i + 1 < text.size() && text[i + 1] == '"') { field += c; ++i; }
            else quoted = !quoted;
        } else if (c == ',' && !quoted) { fields.push_back(field); field.clear(); }
        else field += c;
    }
    fields.push_back(field); return fields;
}
void checkCsv(const TempDirectory& dir) {
    LoggingConfig config; config.directory = (dir.path / "csv").string(); config.queue_capacity = 8;
    ServoLogger logger(config); CHECK(logger.start());
    ServoSample sample; sample.tick = 11; sample.preview_recovery = recoveryFixture();
    sample.loop_start_time_ns = big + 2; sample.loop_end_time_ns = big + 3;
    bool queued = false;
    for (int i = 0; i < 1000 && !queued; ++i) {
        const auto dropped = logger.droppedSamples(); logger.push(sample);
        queued = logger.droppedSamples() == dropped;
    }
    CHECK(queued); logger.stop();
    std::ifstream file(std::filesystem::path(config.directory) / "servo_log.csv");
    std::string header, row; CHECK(static_cast<bool>(std::getline(file, header)));
    CHECK(static_cast<bool>(std::getline(file, row)));
    const auto names = splitCsv(header), values = splitCsv(row); CHECK(names.size() == values.size());
    std::map<std::string, std::string> cells;
    for (std::size_t i = 0; i < names.size(); ++i) cells[names[i]] = values[i];
    CHECK(cells.at("preview_recovery_enabled") == "1");
    CHECK(cells.at("preview_recovery_state") == "paused");
    CHECK(cells.at("preview_recovery_reason") == "plan_expired");
    CHECK(cells.at("preview_recovery_epoch") == std::to_string(big));
    CHECK(cells.at("preview_recovery_min_observation_time_ns") == std::to_string(big - 2));
    CHECK(cells.at("preview_recovery_candidate_source_recv_seq") == std::to_string(big + 10));
    CHECK(cells.at("preview_recovery_abandoned_source_recv_seq") == std::to_string(big + 6));
    CHECK(cells.at("preview_recovery_attempts") == "3");
    CHECK(cells.at("preview_recovery_rejected_frames") == "7");
    CHECK(std::abs(std::stod(cells.at("preview_recovery_abandoned_backlog_sec")) - .099) < 1e-12);
    CHECK(std::abs(std::stod(cells.at("preview_recovery_abandoned_right_position_error_m")) - .022) < 1e-12);
    CHECK(std::abs(std::stod(cells.at("preview_recovery_abandoned_left_rotation_error_rad")) - .033) < 1e-12);
}
}

int main() {
    try {
        TempDirectory dir;
        checkConfig(dir); checkChunkWire(); checkStateWire(); checkCsv(dir);
        std::cout << "preview recovery config, wire, state and CSV contracts passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "preview recovery contract failed: " << error.what() << '\n'; return 1;
    }
}
