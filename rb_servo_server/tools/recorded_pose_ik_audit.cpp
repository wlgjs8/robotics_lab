// OFFLINE ONLY: recorded Cartesian targets -> actual Pinocchio IK -> existing
// joint position/rate/acceleration clamps. No backend, socket, device, worker or
// servo loop is constructed. This is NOT collision/contact/plant acceptance.
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

#include "rb_servo/config/config.hpp"
#include "rb_servo/control/safety_filter.hpp"
#include "rb_servo/kinematics/dh_calibration.hpp"
#include "rb_servo/kinematics/pinocchio_kinematics.hpp"
#include "rb_servo/math/se3.hpp"

namespace {
using namespace rb_servo;
using json = nlohmann::json;

void require(bool condition, const std::string& why) {
    if (!condition) throw std::runtime_error(why);
}

template <std::size_t N>
std::array<double, N> finiteArray(const json& value, const char* field) {
    require(value.is_array() && value.size() == N,
            std::string(field) + " must contain exactly " + std::to_string(N) + " numbers");
    const auto result = value.get<std::array<double, N>>();
    for (double x : result) require(std::isfinite(x), std::string(field) + " contains nonfinite value");
    return result;
}

Pose6D readPose(const json& value) {
    // Same quaternion layout as the production state/wire protocol: xyzw.
    const auto p = finiteArray<7>(value, "nominal_pose");
    Eigen::Quaterniond q(p[6], p[3], p[4], p[5]);
    require(std::abs(q.norm() - 1.0) <= 1e-5, "nominal_pose quaternion is not unit length");
    return math::poseFromSe3(pinocchio::SE3(q.normalized().toRotationMatrix(),
                                         math::Vector3(p[0], p[1], p[2])));
}

Pose6D composeRecordedDeviation(const Pose6D& nominal, const std::array<double, 6>& d) {
    // Same convention as AdmittanceOverlay::compose: stand translation, stand
    // rotation left-composed about the TCP itself (not an SE(3) origin rotation).
    auto pose = math::se3FromPose(nominal);
    pose.translation() += math::Vector3(d[0], d[1], d[2]);
    const math::Vector3 dr(d[3], d[4], d[5]);
    if (dr.norm() >= 1e-9) pose.rotation() = math::exp3(dr) * pose.rotation();
    return math::poseFromSe3(pose);
}

std::string fileText(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    require(in.good(), "cannot open calibrated URDF: " + path);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

std::string hashHex(const std::string& value) {
    std::ostringstream out;
    out << std::hex << std::setw(16) << std::setfill('0') << fnv1a64(value);
    return out.str();
}

void writePose(std::ostream& out, const Pose6D& pose) {
    const Eigen::Quaterniond q(math::rotationFromPose(pose));
    out << ',' << pose.x << ',' << pose.y << ',' << pose.z
        << ',' << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w();
}
}  // namespace

int main(int argc, char** argv) {
    try {
        require(argc == 4, "usage: recorded_pose_ik_audit CONFIG INPUT.jsonl OUTPUT.csv");
        const auto config = loadConfigFromYaml(argv[1]); // Parsing only; no calibration probe.
        require(config.kinematics.enable && config.kinematics.ik.enable &&
                    config.kinematics.provider == "pinocchio", "explicit Pinocchio IK config required");
        std::ifstream in(argv[2]);
        require(in.good(), "cannot open audit input");
        std::string line;
        require(static_cast<bool>(std::getline(in, line)), "missing audit header");
        const auto header = json::parse(line);
        require(header.at("schema") == "robotics_lab.recorded_pose_ik_audit.v1", "unsupported audit schema");
        const auto side = header.at("arm").get<std::string>();
        require(side == "left" || side == "right", "arm must be left or right");
        const ArmId arm = side == "left" ? ArmId::Left : ArmId::Right;
        const auto& mount = arm == ArmId::Left ? config.left_mount : config.right_mount;
        auto kinematics_config = config.kinematics;
        kinematics_config.urdf = header.at("calibrated_urdf").get<std::string>();
        const auto expected_hash = header.at("calibrated_urdf_fnv1a64").get<std::string>();
        const auto actual_hash = hashHex(fileText(kinematics_config.urdf));
        require(expected_hash == actual_hash, "calibrated URDF hash mismatch");
        // Per-arm process: use the recorded arm's immutable calibrated model.
        // This constructor only loads the supplied URDF; it never contacts a box.
        PinocchioKinematics kinematics(kinematics_config);
        SafetyFilter safety(config.safety);
        JointArray previous = finiteArray<6>(header.at("initial_q_deg"), "initial_q_deg");
        JointArray previous_previous = finiteArray<6>(header.at("initial_previous_q_deg"), "initial_previous_q_deg");
        for (int j = 0; j < kDof; ++j) {
            require(previous[j] >= config.safety.q_min_deg[j] && previous[j] <= config.safety.q_max_deg[j] &&
                    previous_previous[j] >= config.safety.q_min_deg[j] &&
                    previous_previous[j] <= config.safety.q_max_deg[j], "initial joints violate recorded safety bounds");
        }
        std::ofstream out(argv[3]);
        require(out.good(), "cannot open audit output");
        out << "t,dt,ik_success,ik_reason,ik_iterations,ik_position_error_m,ik_orientation_error_rad,"
               "ik_branch_rate_limited,ik_branch_clamped,joint_limit_clamped,velocity_clamped,accel_clamped,"
               "target_fk_error_m,target_fk_error_rad,recorded_q_max_error_deg";
        for (const auto* stem : {"q_ik", "q_sent", "dq_sent", "ddq_sent"})
            for (int j = 0; j < kDof; ++j) out << ',' << stem << '_' << j;
        for (const auto* stem : {"nominal", "target", "fk"})
            for (const auto* axis : {"x", "y", "z", "qx", "qy", "qz", "qw"}) out << ',' << stem << '_' << axis;
        out << '\n' << std::setprecision(16);
        double last_t = -std::numeric_limits<double>::infinity();
        std::size_t ticks = 0, failed = 0, limited = 0;
        double max_recorded_error = 0.0, max_fk_error_m = 0.0;
        while (std::getline(in, line)) {
            const auto row = json::parse(line);
            const double t = row.at("t"), dt = row.at("dt");
            require(std::isfinite(t) && t > last_t && std::isfinite(dt) && dt > 0,
                    "timestamps must increase and dt must be finite positive");
            last_t = t;
            const auto nominal = readPose(row.at("nominal_pose"));
            const auto deviation = finiteArray<6>(row.at("deviation_stand"), "deviation_stand");
            const auto target = row.at("compose_deviation").get<bool>()
                ? composeRecordedDeviation(nominal, deviation) : nominal;
            const auto ik = kinematics.solveIk(arm, target, previous, mount);
            // Production also holds a refused solve at the prior sent reference.
            // The clamp can continue a bounded deceleration; no refusal debounce
            // state machine is emulated here (the actual-loop fixture covers it).
            const JointArray raw = ik.success ? ik.q_solution_deg : previous;
            const auto clamp = safety.clampMotionDetailed(raw, previous, previous_previous, dt);
            const JointArray sent = clamp.q_after_accel_limit_deg;
            const auto fk = kinematics.computeTcpStand(arm, sent, mount);
            const double pos_error = math::positionDistance(target, fk);
            const double rot_error = math::orientationDistanceRad(target, fk);
            double recorded_error = std::numeric_limits<double>::quiet_NaN();
            if (row.contains("recorded_q_sent_deg")) {
                const auto q = finiteArray<6>(row.at("recorded_q_sent_deg"), "recorded_q_sent_deg");
                recorded_error = 0.0;
                for (int j = 0; j < kDof; ++j) recorded_error = std::max(recorded_error, std::abs(sent[j] - q[j]));
                max_recorded_error = std::max(max_recorded_error, recorded_error);
            }
            require(ik.reason.find(',') == std::string::npos, "unexpected comma in IK reason");
            out << t << ',' << dt << ',' << ik.success << ',' << ik.reason << ',' << ik.iterations
                << ',' << ik.position_error_m << ',' << ik.orientation_error_rad
                << ',' << ik.branch_jump_rate_limited << ',' << ik.branch_jump_clamped
                << ',' << clamp.joint_limit_clamped << ',' << clamp.velocity_clamped << ',' << clamp.accel_clamped
                << ',' << pos_error << ',' << rot_error << ',' << recorded_error;
            for (double value : raw) out << ',' << value;
            for (double value : sent) out << ',' << value;
            for (int j = 0; j < kDof; ++j) out << ',' << (sent[j] - previous[j]) / dt;
            for (int j = 0; j < kDof; ++j) out << ',' << (sent[j] - 2*previous[j] + previous_previous[j]) / (dt*dt);
            writePose(out, nominal); writePose(out, target); writePose(out, fk); out << '\n';
            ++ticks; failed += !ik.success;
            limited += clamp.joint_limit_clamped || clamp.velocity_clamped || clamp.accel_clamped;
            max_fk_error_m = std::max(max_fk_error_m, pos_error);
            previous_previous = previous; previous = sent;
        }
        require(ticks > 0 && out.good(), "empty audit or failed output write");
        std::cout << json{{"scope", "offline Pinocchio IK and joint-clamp audit; no plant/contact/collision acceptance"},
            {"arm", side}, {"ticks", ticks}, {"ik_failed_ticks", failed}, {"joint_clamped_ticks", limited},
            {"calibrated_urdf_fnv1a64", actual_hash}, {"max_target_fk_error_m", max_fk_error_m},
            {"max_recorded_q_error_deg_if_present", max_recorded_error}}.dump(2) << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "recorded_pose_ik_audit: " << error.what() << '\n';
        return 2;
    }
}
