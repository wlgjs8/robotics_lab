#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rb_servo/config/config.hpp"
#include "rb_servo/control/command_buffer.hpp"
#include "rb_servo/control/safety_filter.hpp"
#include "rb_servo/control/trajectory_filter.hpp"
#include "rb_servo/kinematics/i_kinematics.hpp"
#include "rb_servo/logging/servo_logger.hpp"
#include "rb_servo/robot/i_robot_backend.hpp"

namespace rb_servo {

class DualArmServoLoop {
public:
    DualArmServoLoop(
        std::unique_ptr<IRobotBackend> left_robot,
        std::unique_ptr<IRobotBackend> right_robot,
        const DualArmConfig& config,
        CommandBuffer* command_buffer,
        ServoLogger* logger,
        std::shared_ptr<IKinematics> kinematics = nullptr
    );

    ~DualArmServoLoop();

    bool start();
    void stop();

    bool isRunning() const;
    ServerMotionState motionState() const;
    bool faultLatched() const;
    SafetyVerdict latchedFaultReason() const;
    ServoTarget previousSentTarget() const;
    ServoSnapshot latestSnapshot() const;

private:
    void loopMain();
    bool configureRealtimeForLoop();

    bool initializeRobots();
    bool readRobotStates(RobotState& left, RobotState& right);
    void populateTcpPose(RobotState& state, const ArmMountConfig& mount) const;
    bool isValidRobotStateForStartup(const RobotState& state) const;
    bool isValidJointState(const RobotState& state) const;

    ServoTarget computeServoTarget(
        const RobotState& left_state,
        const RobotState& right_state,
        const DualArmCommand& command,
        double dt_sec,
        SafetyVerdict* command_verdict
    );

    ServoTarget applySafety(
        const ServoTarget& desired,
        const RobotState& left_state,
        const RobotState& right_state,
        double dt_sec,
        SafetyVerdict* verdict
    );

    void sendTargets(
        const ServoTarget& target,
        uint64_t command_seq,
        const std::string& send_policy,
        SendServoJResult* left_result,
        SendServoJResult* right_result,
        uint64_t* left_send_start_ns,
        uint64_t* left_send_end_ns,
        uint64_t* right_send_start_ns,
        uint64_t* right_send_end_ns
    );

    DualArmCommand makeHoldCommand(
        const RobotState& left_state,
        const RobotState& right_state,
        uint64_t now_ns
    ) const;

    bool commandRequestsResetFault(const DualArmCommand& command) const;
    bool commandRequestsEmergencyStop(const DualArmCommand& command) const;
    bool commandRequestsArmMotion(const DualArmCommand& command) const;
    bool commandRequestsDisarmMotion(const DualArmCommand& command) const;
    bool commandRequestsMotion(const DualArmCommand& command) const;
    bool commandBlockedByReadOnly(const DualArmCommand& command) const;
    bool readOnlyMode() const;
    bool motionAllowed() const;
    bool isRealMode() const;
    std::string currentSendPolicy() const;
    bool clearFaultLatch(RobotState& left_state, RobotState& right_state);
    void latchFault(
        SafetyVerdict verdict,
        const std::string& reason,
        const RobotState& left_state,
        const RobotState& right_state
    );
    void setMotionState(ServerMotionState state);
    ServoTarget currentFaultHoldTarget() const;
    JointArray chooseSafeHoldTarget(const RobotState& state, const JointArray& previous_sent) const;
    double computeFilterDtSec(uint64_t actual_period_ns, uint64_t nominal_period_ns) const;

private:
    std::unique_ptr<IRobotBackend> left_robot_;
    std::unique_ptr<IRobotBackend> right_robot_;

    DualArmConfig config_;

    CommandBuffer* command_buffer_ = nullptr;
    ServoLogger* logger_ = nullptr;
    std::shared_ptr<IKinematics> kinematics_;
    bool kinematics_injected_ = false;

    TrajectoryFilter left_traj_filter_;
    TrajectoryFilter right_traj_filter_;
    SafetyFilter safety_filter_;

    std::atomic<bool> running_{false};
    std::atomic<bool> startup_complete_{false};
    std::atomic<bool> startup_ok_{false};
    std::thread thread_;

    uint64_t tick_ = 0;
    uint64_t last_loop_start_ns_ = 0;

    JointArray left_prev_sent_q_deg_{};
    JointArray right_prev_sent_q_deg_{};

    JointArray left_prevprev_sent_q_deg_{};
    JointArray right_prevprev_sent_q_deg_{};

    std::atomic<ServerMotionState> motion_state_{ServerMotionState::Disconnected};
    mutable std::mutex state_mutex_;
    std::atomic<bool> fault_latched_{false};
    std::atomic<SafetyVerdict> fault_verdict_{SafetyVerdict::Ok};
    std::atomic<SafetyVerdict> latched_fault_reason_{SafetyVerdict::Ok};
    std::string fault_reason_;
    JointArray left_fault_hold_q_deg_{};
    JointArray right_fault_hold_q_deg_{};
    ServoSnapshot latest_snapshot_;
};

}  // namespace rb_servo
