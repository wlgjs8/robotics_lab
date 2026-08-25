#include <cmath>
#include <array>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <type_traits>
#include <unistd.h>

#include "rb_servo/config/config.hpp"
#include "rb_servo/control/command_buffer.hpp"
#include "rb_servo/control/dual_arm_servo_loop.hpp"
#include "rb_servo/kinematics/i_kinematics.hpp"
#include "rb_servo/kinematics/pinocchio_kinematics.hpp"
#include "rb_servo/network/state_publisher.hpp"
#include "rb_servo/robot/i_robot_backend.hpp"

#include <nlohmann/json.hpp>

namespace {

#define RB_CHECK(expr) \
    do { \
        if (!(expr)) { \
            std::cerr << "CHECK failed: " #expr << " at " << __FILE__ << ":" << __LINE__ << "\n"; \
            return false; \
        } \
    } while (0)

template <typename T, typename = void>
struct HasComputeIk : std::false_type {};

template <typename T>
struct HasComputeIk<T, std::void_t<decltype(&T::computeIk)>> : std::true_type {};

std::filesystem::path servoRoot() {
    return std::filesystem::path(__FILE__).parent_path().parent_path();
}

std::filesystem::path rb3Urdf() {
    return servoRoot() / "descriptions" / "urdf" / "rb3_730e.urdf";
}

std::string writeTempConfig(const std::string& name, const std::string& body) {
    const std::string path = "/tmp/rb-servo-kinematics-" + name + "-" + std::to_string(getpid()) + ".yaml";
    std::ofstream file(path);
    file << body;
    return path;
}

bool loadRejects(const std::string& path) {
    try {
        (void)rb_servo::loadConfigFromYaml(path);
    } catch (const std::exception&) {
        return true;
    }
    return false;
}

bool finitePose(const rb_servo::Pose6D& pose) {
    return std::isfinite(pose.x) &&
           std::isfinite(pose.y) &&
           std::isfinite(pose.z) &&
           std::isfinite(pose.rx) &&
           std::isfinite(pose.ry) &&
           std::isfinite(pose.rz);
}

bool differentPose(const rb_servo::Pose6D& a, const rb_servo::Pose6D& b) {
    return std::fabs(a.x - b.x) > 1e-9 ||
           std::fabs(a.y - b.y) > 1e-9 ||
           std::fabs(a.z - b.z) > 1e-9 ||
           std::fabs(a.rx - b.rx) > 1e-9 ||
           std::fabs(a.ry - b.ry) > 1e-9 ||
           std::fabs(a.rz - b.rz) > 1e-9;
}

bool normalizedQuaternion(const rb_servo::Pose6D& pose) {
    if (!pose.quaternion_xyzw.has_value()) return false;
    const auto& q = *pose.quaternion_xyzw;
    const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    return std::isfinite(norm) && std::fabs(norm - 1.0) < 1e-9;
}

std::array<double, 3> baseZAxisInStand(const rb_servo::Pose6D& pose) {
    const double cr = std::cos(pose.rx);
    const double sr = std::sin(pose.rx);
    const double cp = std::cos(pose.ry);
    const double sp = std::sin(pose.ry);
    const double cy = std::cos(pose.rz);
    const double sy = std::sin(pose.rz);
    return {
        cy * sp * cr + sy * sr,
        sy * sp * cr - cy * sr,
        cp * cr,
    };
}

std::array<double, 3> shoulderMountNormal(const rb_servo::Pose6D& pose, rb_servo::ArmId arm_id) {
    std::array<double, 3> z_axis = baseZAxisInStand(pose);
    if (arm_id == rb_servo::ArmId::Left) {
        z_axis[0] = -z_axis[0];
        z_axis[1] = -z_axis[1];
        z_axis[2] = -z_axis[2];
    }
    return z_axis;
}

bool closeVector(const std::array<double, 3>& actual, const std::array<double, 3>& expected, double tolerance) {
    for (std::size_t i = 0; i < actual.size(); ++i) {
        if (std::fabs(actual[i] - expected[i]) > tolerance) return false;
    }
    return true;
}

rb_servo::JointArray joints(double value) {
    rb_servo::JointArray out{};
    out.fill(value);
    return out;
}

rb_servo::DualArmConfig testConfig() {
    rb_servo::DualArmConfig cfg;
    cfg.left_robot.run_mode = rb_servo::RunMode::Mock;
    cfg.right_robot.run_mode = rb_servo::RunMode::Mock;
    cfg.left_mount.arm_id = rb_servo::ArmId::Left;
    cfg.left_mount.base_pose_in_stand = {0.1, 0.2, 0.3, 0.0, 0.0, 0.0};
    cfg.right_mount.arm_id = rb_servo::ArmId::Right;
    cfg.right_mount.base_pose_in_stand = {-0.1, 0.2, 0.3, 0.0, 0.0, 0.0};
    cfg.servo.rate_hz = 200;
    cfg.servo.enable_realtime_priority = false;
    cfg.servo.send_servo_commands = false;
    cfg.safety.q_min_deg = rb_servo::rbpodoDefaultSafetyJointMinDeg();
    cfg.safety.q_max_deg = rb_servo::rbpodoDefaultSafetyJointMaxDeg();
    cfg.safety.dq_max_deg_s = joints(10000.0);
    cfg.safety.ddq_max_deg_s2 = joints(100000.0);
    cfg.safety.max_tracking_error_deg = 1000.0;
    cfg.safety.tracking_error_policy = rb_servo::TrackingErrorPolicy::SnapToActual;
    cfg.safety.stop_both_arms_on_single_arm_error = false;
    cfg.safety.latch_fault_on_robot_state_error = true;
    return cfg;
}

class FakeKinematics final : public rb_servo::IKinematics {
public:
    rb_servo::Pose6D computeTcpBase(const rb_servo::JointArray& q_deg) const override {
        rb_servo::Pose6D pose{q_deg[0] * 0.001, q_deg[1] * 0.001, 0.7, 0.01, 0.02, 0.03};
        pose.quaternion_xyzw = std::array<double, 4>{0.0, 0.0, 0.0, 1.0};
        return pose;
    }

    rb_servo::Pose6D computeTcpStand(
        rb_servo::ArmId arm,
        const rb_servo::JointArray& q_deg,
        const rb_servo::ArmMountConfig& mount
    ) const override {
        const rb_servo::Pose6D tcp_base = computeTcpBase(q_deg);
        const double arm_offset = arm == rb_servo::ArmId::Left ? 1.0 : -1.0;
        rb_servo::Pose6D pose{
            mount.base_pose_in_stand.x + tcp_base.x + arm_offset,
            mount.base_pose_in_stand.y + tcp_base.y,
            mount.base_pose_in_stand.z + tcp_base.z,
            mount.base_pose_in_stand.rx + tcp_base.rx,
            mount.base_pose_in_stand.ry + tcp_base.ry,
            mount.base_pose_in_stand.rz + tcp_base.rz,
        };
        pose.quaternion_xyzw = std::array<double, 4>{0.0, 0.0, 0.0, 1.0};
        return pose;
    }

    rb_servo::IkResult solveIk(
        rb_servo::ArmId arm,
        const rb_servo::Pose6D& target_tcp_stand,
        const rb_servo::JointArray& seed_q_deg,
        const rb_servo::ArmMountConfig& mount
    ) const override {
        (void)arm;
        (void)target_tcp_stand;
        (void)mount;
        rb_servo::IkResult result;
        result.q_solution_deg = seed_q_deg;
        result.reason = "kinematics_unavailable";
        return result;
    }

};

class TestBackend final : public rb_servo::IRobotBackend {
public:
    TestBackend(
        rb_servo::ArmId arm_id,
        rb_servo::JointArray q_actual,
        std::optional<rb_servo::JointArray> q_target = std::nullopt
    ) : arm_id_(arm_id), q_actual_(q_actual), q_target_(q_target.value_or(q_actual)) {}

    rb_servo::BackendResult<rb_servo::RobotState> connect() override {
        connected_ = true;
        return result(rb_servo::BackendOp::Connect);
    }

    rb_servo::BackendResult<rb_servo::RobotState> initialize() override {
        initialized_ = true;
        return result(rb_servo::BackendOp::Initialize);
    }

    rb_servo::BackendResult<rb_servo::RobotState> readState() override {
        return result(rb_servo::BackendOp::ReadState);
    }

    rb_servo::SendServoJResult sendServoJ(const rb_servo::SendServoJRequest& request) override {
        q_target_ = request.q_target_deg;
        q_actual_ = request.q_target_deg;
        return rb_servo::acceptedSend(request, {}, currentState(), "cache");
    }

    rb_servo::BackendResult<rb_servo::RobotState> stop() override { return result(rb_servo::BackendOp::Stop); }
    rb_servo::BackendResult<rb_servo::RobotState> resetFault() override { return result(rb_servo::BackendOp::ResetFault); }
    bool isConnected() const override { return connected_; }
    rb_servo::ArmId armId() const override { return arm_id_; }
    std::string name() const override { return "test"; }

private:
    rb_servo::RobotState currentState() const {
        rb_servo::RobotState state;
        state.arm_id = arm_id_;
        state.q_actual_deg = q_actual_;
        state.q_target_deg = q_target_;
        state.has_valid_joint_state = true;
        state.connection_state = connected_
            ? rb_servo::RobotConnectionState::Connected
            : rb_servo::RobotConnectionState::Disconnected;
        state.servo_enabled = initialized_;
        return state;
    }

    rb_servo::BackendResult<rb_servo::RobotState> result(rb_servo::BackendOp op) const {
        rb_servo::BackendResult<rb_servo::RobotState> out;
        out.ok = true;
        out.op = op;
        out.value = currentState();
        out.error = rb_servo::noBackendError();
        return out;
    }

    rb_servo::ArmId arm_id_;
    rb_servo::JointArray q_actual_{};
    rb_servo::JointArray q_target_{};
    bool connected_ = false;
    bool initialized_ = false;
};

std::string validKinematicsYaml(const std::string& urdf_path) {
    return
        "schema: robotics_lab.rb_servo_server.v1\n"
        "kinematics:\n"
        "  enable: true\n"
        "  provider: pinocchio\n"
        "  urdf: \"" + urdf_path + "\"\n"
        "  base_link: \"world\"\n"
        "  tip_link: \"tcp\"\n"
        "  joint_names:\n"
        "    - base_joint\n"
        "    - shoulder_joint\n"
        "    - elbow_joint\n"
        "    - wrist1_joint\n"
        "    - wrist2_joint\n"
        "    - wrist3_joint\n"
        "  q_units: deg\n"
        "  publish_tcp: true\n";
}

bool testKinematicsConfigValidation() {
    const std::string valid_path = writeTempConfig("valid", validKinematicsYaml(rb3Urdf().string()));
    const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(valid_path);
    ::unlink(valid_path.c_str());

    RB_CHECK(cfg.kinematics.enable);
    RB_CHECK(cfg.kinematics.provider == "pinocchio");
    RB_CHECK(cfg.kinematics.base_link == "world");
    RB_CHECK(cfg.kinematics.tip_link == "tcp");
    RB_CHECK(cfg.kinematics.joint_names.size() == rb_servo::kDof);
    RB_CHECK(cfg.kinematics.q_units == "deg");
    RB_CHECK(cfg.kinematics.publish_tcp);
    RB_CHECK(std::filesystem::is_regular_file(cfg.kinematics.urdf));

    const std::string bad_urdf_path = writeTempConfig(
        "bad-urdf",
        validKinematicsYaml("/tmp/does-not-exist-rb3-730e.urdf")
    );
    const bool bad_urdf_rejected = loadRejects(bad_urdf_path);
    ::unlink(bad_urdf_path.c_str());
    RB_CHECK(bad_urdf_rejected);

    const std::string bad_units_path = writeTempConfig(
        "bad-units",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "kinematics:\n"
        "  enable: true\n"
        "  provider: pinocchio\n"
        "  urdf: \"" + rb3Urdf().string() + "\"\n"
        "  q_units: rad\n"
    );
    const bool bad_units_rejected = loadRejects(bad_units_path);
    ::unlink(bad_units_path.c_str());
    RB_CHECK(bad_units_rejected);

    return true;
}

bool testIkRemainsUnavailable() {
    static_assert(!HasComputeIk<rb_servo::IKinematics>::value, "P2-A must not add IK to IKinematics");
    static_assert(!HasComputeIk<rb_servo::PinocchioKinematics>::value, "P2-A must not add IK to PinocchioKinematics");
    return true;
}

bool testPinocchioFk() {
    rb_servo::KinematicsConfig cfg;
    cfg.enable = true;
    cfg.provider = "pinocchio";
    cfg.urdf = rb3Urdf().string();
    cfg.base_link = "world";
    cfg.tip_link = "tcp";
    cfg.joint_names = {
        "base_joint",
        "shoulder_joint",
        "elbow_joint",
        "wrist1_joint",
        "wrist2_joint",
        "wrist3_joint",
    };
    cfg.q_units = "deg";
    cfg.publish_tcp = true;

    rb_servo::PinocchioKinematics kin(cfg);

    rb_servo::JointArray zero{};
    const rb_servo::Pose6D tcp_zero = kin.computeTcpBase(zero);
    RB_CHECK(finitePose(tcp_zero));
    RB_CHECK(normalizedQuaternion(tcp_zero));

    rb_servo::JointArray base_90{};
    base_90[0] = 90.0;
    const rb_servo::Pose6D tcp_base_90 = kin.computeTcpBase(base_90);
    RB_CHECK(finitePose(tcp_base_90));
    RB_CHECK(normalizedQuaternion(tcp_base_90));
    RB_CHECK(differentPose(tcp_zero, tcp_base_90));

    rb_servo::ArmMountConfig mount;
    mount.arm_id = rb_servo::ArmId::Left;
    mount.base_pose_in_stand = {0.1601, -0.1725, 0.5825, 2.186649, 0.523831, 2.526296};
    const rb_servo::Pose6D tcp_stand = kin.computeTcpStand(rb_servo::ArmId::Left, zero, mount);
    RB_CHECK(finitePose(tcp_stand));
    RB_CHECK(normalizedQuaternion(tcp_stand));

    cfg.tip_link = "missing_tip";
    bool bad_tip_threw = false;
    try {
        rb_servo::PinocchioKinematics bad_tip(cfg);
    } catch (const std::exception&) {
        bad_tip_threw = true;
    }
    RB_CHECK(bad_tip_threw);

    cfg.tip_link = "tcp";
    cfg.joint_names[5] = "missing_joint";
    bool bad_joint_threw = false;
    try {
        rb_servo::PinocchioKinematics bad_joint(cfg);
    } catch (const std::exception&) {
        bad_joint_threw = true;
    }
    RB_CHECK(bad_joint_threw);
    return true;
}

// Per-arm control threads call FK/IK concurrently through the SAME shared
// IKinematics. pinocchio::Data is per-call scratch, so a single shared Data
// would silently return another thread's pose -- no crash, no log, just a wrong
// TCP. Prove concurrent results equal the serial ones exactly.
bool testKinematicsIsThreadSafe() {
    rb_servo::KinematicsConfig cfg;
    cfg.enable = true;
    cfg.provider = "pinocchio";
    cfg.urdf = rb3Urdf().string();
    cfg.base_link = "world";
    cfg.tip_link = "tcp";
    cfg.joint_names = {
        "base_joint", "shoulder_joint", "elbow_joint",
        "wrist1_joint", "wrist2_joint", "wrist3_joint",
    };
    cfg.q_units = "deg";
    cfg.publish_tcp = true;
    const auto kin = std::make_shared<rb_servo::PinocchioKinematics>(cfg);

    // Two well-separated configurations, so a leaked pose from the other thread
    // is far outside any tolerance rather than a rounding difference.
    const auto configuration = [](double base) {
        rb_servo::JointArray q{};
        for (int i = 0; i < rb_servo::kDof; ++i) q[i] = base + 7.0 * i;
        return q;
    };
    const rb_servo::JointArray qa = configuration(-40.0);
    const rb_servo::JointArray qb = configuration(35.0);
    const rb_servo::Pose6D expect_a = kin->computeTcpBase(qa);
    const rb_servo::Pose6D expect_b = kin->computeTcpBase(qb);
    RB_CHECK(finitePose(expect_a) && finitePose(expect_b));
    // The two references must actually differ, or the test proves nothing.
    RB_CHECK(std::abs(expect_a.x - expect_b.x) + std::abs(expect_a.y - expect_b.y) +
             std::abs(expect_a.z - expect_b.z) > 1e-3);

    constexpr int kIterations = 400;
    std::atomic<bool> mismatch{false};
    const auto hammer = [&](const rb_servo::JointArray& q, const rb_servo::Pose6D& expected) {
        for (int i = 0; i < kIterations && !mismatch.load(); ++i) {
            const rb_servo::Pose6D got = kin->computeTcpBase(q);
            if (std::abs(got.x - expected.x) > 1e-9 ||
                std::abs(got.y - expected.y) > 1e-9 ||
                std::abs(got.z - expected.z) > 1e-9) {
                mismatch.store(true);
            }
        }
    };
    std::thread ta(hammer, qa, expect_a);
    std::thread tb(hammer, qb, expect_b);
    ta.join();
    tb.join();
    RB_CHECK(!mismatch.load());
    return true;
}

bool testLinkCollisionPointsInStand() {
    rb_servo::KinematicsConfig cfg;
    cfg.enable = true;
    cfg.provider = "pinocchio";
    cfg.urdf = rb3Urdf().string();
    cfg.base_link = "world";
    cfg.tip_link = "tcp";
    cfg.joint_names = {
        "base_joint",
        "shoulder_joint",
        "elbow_joint",
        "wrist1_joint",
        "wrist2_joint",
        "wrist3_joint",
    };
    cfg.q_units = "deg";
    cfg.publish_tcp = true;
    rb_servo::PinocchioKinematics kin(cfg);

    rb_servo::ArmMountConfig mount;
    mount.arm_id = rb_servo::ArmId::Left;
    mount.base_pose_in_stand = {0.1601, -0.1725, 0.5825, 2.186649, 0.523831, 2.526296};

    rb_servo::JointArray q{};
    q[0] = 10.0;
    q[1] = -30.0;
    q[2] = 80.0;
    q[3] = 0.0;
    q[4] = 60.0;
    q[5] = 0.0;

    const std::vector<std::array<double, 3>> pts =
        kin.linkCollisionPointsInStand(rb_servo::ArmId::Left, q, mount);
    // base origin + 6 joint origins + tcp = 8 chain points.
    RB_CHECK(pts.size() == 8);
    for (const std::array<double, 3>& p : pts) {
        RB_CHECK(std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]));
    }
    // Base origin in stand equals the mount translation (base_link == world).
    RB_CHECK(std::fabs(pts.front()[0] - mount.base_pose_in_stand.x) < 1e-9);
    RB_CHECK(std::fabs(pts.front()[1] - mount.base_pose_in_stand.y) < 1e-9);
    RB_CHECK(std::fabs(pts.front()[2] - mount.base_pose_in_stand.z) < 1e-9);
    // Last chain point equals the TCP-in-stand translation (FK consistency).
    const rb_servo::Pose6D tcp = kin.computeTcpStand(rb_servo::ArmId::Left, q, mount);
    RB_CHECK(std::fabs(pts.back()[0] - tcp.x) < 1e-9);
    RB_CHECK(std::fabs(pts.back()[1] - tcp.y) < 1e-9);
    RB_CHECK(std::fabs(pts.back()[2] - tcp.z) < 1e-9);
    return true;
}

bool testConfiguredMountNormals() {
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.left_mount.base_pose_in_stand = {0.15707, -0.17036, 0.58036, 2.186649, 0.523831, 2.526296};
    cfg.right_mount.base_pose_in_stand = {-0.15707, -0.17036, 0.58036, 2.186649, -0.523831, -2.526296};
    const std::array<double, 3> left_normal = shoulderMountNormal(cfg.left_mount.base_pose_in_stand, rb_servo::ArmId::Left);
    const std::array<double, 3> right_normal = shoulderMountNormal(cfg.right_mount.base_pose_in_stand, rb_servo::ArmId::Right);

    RB_CHECK(closeVector(left_normal, {-0.709, -0.500, 0.498}, 0.01));
    RB_CHECK(closeVector(right_normal, {-0.708, 0.499, -0.500}, 0.01));
    return true;
}

bool testStatePublisherSerializesTcpPoseValidity() {
    rb_servo::DualArmConfig cfg = testConfig();
    rb_servo::ServoSnapshot snapshot;
    snapshot.left_state.arm_id = rb_servo::ArmId::Left;
    snapshot.left_state.has_valid_joint_state = true;
    snapshot.left_state.connection_state = rb_servo::RobotConnectionState::Connected;
    snapshot.left_state.tcp_base = rb_servo::Pose6D{0.1, 0.2, 0.3, 0.01, 0.02, 0.03};
    snapshot.left_state.tcp_stand = rb_servo::Pose6D{1.1, 1.2, 1.3, 0.11, 0.12, 0.13};
    snapshot.left_state.tcp_base->quaternion_xyzw = std::array<double, 4>{1.0, 2.0, 3.0, 4.0};
    snapshot.left_state.tcp_stand->quaternion_xyzw = std::array<double, 4>{0.0, 0.0, 0.0, 1.0};
    snapshot.left_state.has_valid_tcp_pose = true;
    snapshot.left_state.tcp_deferred = false;

    snapshot.right_state.arm_id = rb_servo::ArmId::Right;
    snapshot.right_state.has_valid_joint_state = false;
    snapshot.right_state.connection_state = rb_servo::RobotConnectionState::Connected;
    snapshot.right_state.has_valid_tcp_pose = false;
    snapshot.right_state.tcp_deferred = false;

    rb_servo::StatePublisher publisher(cfg);
    const nlohmann::json json = nlohmann::json::parse(publisher.serializeSnapshot(snapshot));

    RB_CHECK(!json.at("tcp_fields_deferred").get<bool>());
    RB_CHECK(!json.at("left").at("tcp_base").is_null());
    RB_CHECK(!json.at("left").at("tcp_stand").is_null());
    RB_CHECK(json.at("left").at("tcp_base").at("x").get<double>() == 0.1);
    RB_CHECK(json.at("left").at("tcp_base").at("rx").get<double>() == 0.01);
    RB_CHECK(json.at("left").at("tcp_base").at("ry").get<double>() == 0.02);
    RB_CHECK(json.at("left").at("tcp_base").at("rz").get<double>() == 0.03);
    RB_CHECK(json.at("left").at("tcp_actual_base").at("x").get<double>() == 0.1);
    RB_CHECK(json.at("left").at("tcp_actual_stand").at("x").get<double>() == 1.1);
    RB_CHECK(json.at("left").at("tcp_ref_base").is_null());
    RB_CHECK(json.at("left").at("tcp_ref_stand").is_null());
    RB_CHECK(json.at("left").at("tcp_base").at("quaternion_xyzw").is_array());
    const double q_norm = std::sqrt(1.0 + 4.0 + 9.0 + 16.0);
    RB_CHECK(std::fabs(json.at("left").at("tcp_base").at("quaternion_xyzw").at(0).get<double>() - 1.0 / q_norm) < 1e-12);
    RB_CHECK(std::fabs(json.at("left").at("tcp_base").at("quaternion_xyzw").at(1).get<double>() - 2.0 / q_norm) < 1e-12);
    RB_CHECK(std::fabs(json.at("left").at("tcp_base").at("quaternion_xyzw").at(2).get<double>() - 3.0 / q_norm) < 1e-12);
    RB_CHECK(std::fabs(json.at("left").at("tcp_base").at("quaternion_xyzw").at(3).get<double>() - 4.0 / q_norm) < 1e-12);
    RB_CHECK(std::fabs(json.at("left").at("tcp_base").at("qx").get<double>() - 1.0 / q_norm) < 1e-12);
    RB_CHECK(std::fabs(json.at("left").at("tcp_base").at("qy").get<double>() - 2.0 / q_norm) < 1e-12);
    RB_CHECK(std::fabs(json.at("left").at("tcp_base").at("qz").get<double>() - 3.0 / q_norm) < 1e-12);
    RB_CHECK(std::fabs(json.at("left").at("tcp_base").at("qw").get<double>() - 4.0 / q_norm) < 1e-12);
    RB_CHECK(json.at("left").at("has_valid_tcp_pose").get<bool>());
    RB_CHECK(json.at("left").at("tcp_actual_valid").get<bool>());
    RB_CHECK(!json.at("left").at("tcp_ref_valid").get<bool>());
    RB_CHECK(json.at("left").at("tcp_tracking_source").get<std::string>() == "tcp_actual_stand");
    RB_CHECK(json.at("left").at("tcp_tracking_source_recommendation").get<std::string>() == "actual");
    RB_CHECK(json.at("left").at("controller_simulation_mode").is_null());
    RB_CHECK(!json.at("left").at("tcp_deferred").get<bool>());

    RB_CHECK(json.at("right").at("tcp_base").is_null());
    RB_CHECK(json.at("right").at("tcp_stand").is_null());
    RB_CHECK(json.at("right").at("tcp_actual_base").is_null());
    RB_CHECK(json.at("right").at("tcp_actual_stand").is_null());
    RB_CHECK(json.at("right").at("tcp_ref_base").is_null());
    RB_CHECK(json.at("right").at("tcp_ref_stand").is_null());
    RB_CHECK(!json.at("right").at("has_valid_tcp_pose").get<bool>());
    RB_CHECK(!json.at("right").at("tcp_actual_valid").get<bool>());
    RB_CHECK(!json.at("right").at("tcp_ref_valid").get<bool>());
    RB_CHECK(json.at("right").at("tcp_tracking_source").get<std::string>() == "none");
    RB_CHECK(json.at("right").at("tcp_tracking_source_recommendation").get<std::string>() == "unavailable");
    RB_CHECK(!json.at("right").at("tcp_deferred").get<bool>());
    return true;
}

bool testStatePublisherKeepsTcpDeferredWhenFkDisabled() {
    rb_servo::DualArmConfig cfg = testConfig();
    rb_servo::ServoSnapshot snapshot;
    snapshot.left_state.arm_id = rb_servo::ArmId::Left;
    snapshot.right_state.arm_id = rb_servo::ArmId::Right;

    rb_servo::StatePublisher publisher(cfg);
    const nlohmann::json json = nlohmann::json::parse(publisher.serializeSnapshot(snapshot));

    RB_CHECK(json.at("tcp_fields_deferred").get<bool>());
    RB_CHECK(json.at("left").at("tcp_base").is_null());
    RB_CHECK(json.at("left").at("tcp_stand").is_null());
    RB_CHECK(json.at("left").at("tcp_actual_base").is_null());
    RB_CHECK(json.at("left").at("tcp_actual_stand").is_null());
    RB_CHECK(json.at("left").at("tcp_ref_base").is_null());
    RB_CHECK(json.at("left").at("tcp_ref_stand").is_null());
    RB_CHECK(!json.at("left").at("has_valid_tcp_pose").get<bool>());
    RB_CHECK(!json.at("left").at("tcp_actual_valid").get<bool>());
    RB_CHECK(!json.at("left").at("tcp_ref_valid").get<bool>());
    RB_CHECK(json.at("left").at("tcp_deferred").get<bool>());
    RB_CHECK(json.at("right").at("tcp_base").is_null());
    RB_CHECK(json.at("right").at("tcp_stand").is_null());
    RB_CHECK(json.at("right").at("tcp_actual_base").is_null());
    RB_CHECK(json.at("right").at("tcp_actual_stand").is_null());
    RB_CHECK(json.at("right").at("tcp_ref_base").is_null());
    RB_CHECK(json.at("right").at("tcp_ref_stand").is_null());
    RB_CHECK(!json.at("right").at("has_valid_tcp_pose").get<bool>());
    RB_CHECK(!json.at("right").at("tcp_actual_valid").get<bool>());
    RB_CHECK(!json.at("right").at("tcp_ref_valid").get<bool>());
    RB_CHECK(json.at("right").at("tcp_deferred").get<bool>());
    return true;
}

bool testServoLoopPublishesInjectedFkForValidJointState() {
    rb_servo::DualArmConfig cfg = testConfig();
    rb_servo::CommandBuffer buffer;
    auto kinematics = std::make_shared<FakeKinematics>();
    rb_servo::DualArmServoLoop loop(
        std::make_unique<TestBackend>(rb_servo::ArmId::Left, joints(10.0)),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, joints(20.0)),
        cfg,
        &buffer,
        nullptr,
        kinematics
    );

    RB_CHECK(loop.start());
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    loop.stop();

    RB_CHECK(snapshot.left_state.has_valid_tcp_pose);
    RB_CHECK(snapshot.right_state.has_valid_tcp_pose);
    RB_CHECK(snapshot.left_state.tcp_base.has_value());
    RB_CHECK(snapshot.left_state.tcp_stand.has_value());
    RB_CHECK(snapshot.right_state.tcp_base.has_value());
    RB_CHECK(snapshot.right_state.tcp_stand.has_value());
    RB_CHECK(normalizedQuaternion(*snapshot.left_state.tcp_base));
    RB_CHECK(normalizedQuaternion(*snapshot.left_state.tcp_stand));
    RB_CHECK(normalizedQuaternion(*snapshot.right_state.tcp_base));
    RB_CHECK(normalizedQuaternion(*snapshot.right_state.tcp_stand));
    RB_CHECK(!snapshot.left_state.tcp_deferred);
    RB_CHECK(!snapshot.right_state.tcp_deferred);

    rb_servo::StatePublisher publisher(cfg);
    const nlohmann::json json = nlohmann::json::parse(publisher.serializeSnapshot(snapshot));
    RB_CHECK(!json.at("left").at("tcp_base").is_null());
    RB_CHECK(!json.at("left").at("tcp_stand").is_null());
    RB_CHECK(json.at("left").at("tcp_base").contains("quaternion_xyzw"));
    RB_CHECK(json.at("left").at("tcp_stand").contains("quaternion_xyzw"));
    RB_CHECK(json.at("left").at("has_valid_tcp_pose").get<bool>());
    RB_CHECK(!json.at("right").at("tcp_base").is_null());
    RB_CHECK(!json.at("right").at("tcp_stand").is_null());
    RB_CHECK(json.at("right").at("tcp_base").contains("quaternion_xyzw"));
    RB_CHECK(json.at("right").at("tcp_stand").contains("quaternion_xyzw"));
    RB_CHECK(json.at("right").at("has_valid_tcp_pose").get<bool>());
    return true;
}

bool testServoLoopPublishesActualAndReferenceTcpForControllerSimulation() {
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.left_robot.backend_type = rb_servo::BackendType::Rbpodo;
    cfg.right_robot.backend_type = rb_servo::BackendType::Rbpodo;
    cfg.left_robot.operation_mode = "simulation";
    cfg.right_robot.operation_mode = "simulation";
    rb_servo::CommandBuffer buffer;
    auto kinematics = std::make_shared<FakeKinematics>();

    rb_servo::JointArray left_actual = joints(10.0);
    rb_servo::JointArray left_ref = joints(10.0);
    left_ref[0] = 20.0;
    left_ref[1] = 30.0;
    rb_servo::JointArray right_actual = joints(20.0);
    rb_servo::JointArray right_ref = joints(20.0);
    right_ref[0] = 25.0;
    right_ref[1] = 35.0;

    rb_servo::DualArmServoLoop loop(
        std::make_unique<TestBackend>(rb_servo::ArmId::Left, left_actual, left_ref),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, right_actual, right_ref),
        cfg,
        &buffer,
        nullptr,
        kinematics
    );

    RB_CHECK(loop.start());
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    loop.stop();

    RB_CHECK(snapshot.left_state.tcp_actual_valid);
    RB_CHECK(snapshot.left_state.tcp_ref_valid);
    RB_CHECK(snapshot.left_state.tcp_base.has_value());
    RB_CHECK(snapshot.left_state.tcp_stand.has_value());
    RB_CHECK(snapshot.left_state.tcp_actual_base.has_value());
    RB_CHECK(snapshot.left_state.tcp_actual_stand.has_value());
    RB_CHECK(snapshot.left_state.tcp_ref_base.has_value());
    RB_CHECK(snapshot.left_state.tcp_ref_stand.has_value());
    RB_CHECK(differentPose(*snapshot.left_state.tcp_actual_stand, *snapshot.left_state.tcp_ref_stand));
    RB_CHECK(std::fabs(snapshot.left_state.tcp_stand->x - snapshot.left_state.tcp_actual_stand->x) < 1e-12);
    RB_CHECK(std::fabs(snapshot.left_state.tcp_base->x - snapshot.left_state.tcp_actual_base->x) < 1e-12);
    RB_CHECK(normalizedQuaternion(*snapshot.left_state.tcp_actual_stand));
    RB_CHECK(normalizedQuaternion(*snapshot.left_state.tcp_ref_stand));

    rb_servo::StatePublisher publisher(cfg);
    const nlohmann::json json = nlohmann::json::parse(publisher.serializeSnapshot(snapshot));
    RB_CHECK(json.at("left").at("tcp_actual_valid").get<bool>());
    RB_CHECK(json.at("left").at("tcp_ref_valid").get<bool>());
    RB_CHECK(!json.at("left").at("tcp_actual_stand").is_null());
    RB_CHECK(!json.at("left").at("tcp_ref_stand").is_null());
    RB_CHECK(json.at("left").at("tcp_actual_stand").contains("quaternion_xyzw"));
    RB_CHECK(json.at("left").at("tcp_ref_stand").contains("quaternion_xyzw"));
    RB_CHECK(json.at("left").at("tcp_stand").at("x").get<double>() ==
             json.at("left").at("tcp_actual_stand").at("x").get<double>());
    RB_CHECK(json.at("left").at("tcp_ref_stand").at("x").get<double>() !=
             json.at("left").at("tcp_actual_stand").at("x").get<double>());
    RB_CHECK(json.at("left").at("tcp_tracking_source").get<std::string>() == "tcp_ref_stand");
    RB_CHECK(
        json.at("left").at("tcp_tracking_source_recommendation").get<std::string>() ==
        "reference_for_controller_simulation"
    );
    RB_CHECK(json.at("left").at("controller_simulation_mode").at("recommended_tracking_pose").get<std::string>() == "tcp_ref_stand");
    RB_CHECK(!json.at("left").at("controller_simulation_mode").at("physical_motion_expected").get<bool>());
    return true;
}

bool testServoLoopOmitsReferenceTcpForInvalidReferenceJoints() {
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.left_robot.backend_type = rb_servo::BackendType::Rbpodo;
    cfg.left_robot.operation_mode = "simulation";
    rb_servo::CommandBuffer buffer;
    auto kinematics = std::make_shared<FakeKinematics>();

    rb_servo::JointArray actual = joints(10.0);
    rb_servo::JointArray invalid_ref = joints(10.0);
    invalid_ref[2] = std::numeric_limits<double>::quiet_NaN();

    rb_servo::DualArmServoLoop loop(
        std::make_unique<TestBackend>(rb_servo::ArmId::Left, actual, invalid_ref),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, actual, actual),
        cfg,
        &buffer,
        nullptr,
        kinematics
    );

    RB_CHECK(loop.start());
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    loop.stop();

    RB_CHECK(snapshot.left_state.tcp_actual_valid);
    RB_CHECK(!snapshot.left_state.tcp_ref_valid);
    RB_CHECK(snapshot.left_state.tcp_actual_stand.has_value());
    RB_CHECK(!snapshot.left_state.tcp_ref_stand.has_value());

    rb_servo::StatePublisher publisher(cfg);
    const nlohmann::json json = nlohmann::json::parse(publisher.serializeSnapshot(snapshot));
    RB_CHECK(json.at("left").at("tcp_actual_valid").get<bool>());
    RB_CHECK(!json.at("left").at("tcp_ref_valid").get<bool>());
    RB_CHECK(!json.at("left").at("tcp_actual_stand").is_null());
    RB_CHECK(json.at("left").at("tcp_ref_base").is_null());
    RB_CHECK(json.at("left").at("tcp_ref_stand").is_null());
    RB_CHECK(json.at("left").at("tcp_tracking_source").get<std::string>() == "tcp_actual_stand");
    RB_CHECK(
        json.at("left").at("tcp_tracking_source_recommendation").get<std::string>() ==
        "actual_fallback_reference_unavailable"
    );
    RB_CHECK(json.at("left").at("controller_simulation_mode").at("recommended_tracking_pose").get<std::string>() == "tcp_actual_stand");
    RB_CHECK(!json.at("left").at("controller_simulation_mode").at("tcp_ref_valid").get<bool>());
    return true;
}

}  // namespace

int main() {
    if (!testKinematicsConfigValidation()) return 1;
    if (!testIkRemainsUnavailable()) return 1;
    if (!testConfiguredMountNormals()) return 1;
    if (!testPinocchioFk()) return 1;
    if (!testKinematicsIsThreadSafe()) return 1;
    if (!testLinkCollisionPointsInStand()) return 1;
    if (!testStatePublisherSerializesTcpPoseValidity()) return 1;
    if (!testStatePublisherKeepsTcpDeferredWhenFkDisabled()) return 1;
    if (!testServoLoopPublishesInjectedFkForValidJointState()) return 1;
    if (!testServoLoopPublishesActualAndReferenceTcpForControllerSimulation()) return 1;
    if (!testServoLoopOmitsReferenceTcpForInvalidReferenceJoints()) return 1;
    return 0;
}
