#pragma once

#include <cstddef>
#include <memory>
#include <optional>
#include <string>

#include "rb_servo/config/config.hpp"
#include "rb_servo/robot/i_robot_backend.hpp"

namespace rb_servo {

struct RbpodoSystemStateSnapshot {
    JointArray q_actual_deg{};
    JointArray q_target_deg{};
    double robot_time_sec = 0.0;
    int real_vs_simulation_mode = 0;
    int init_state_info = 0;
    int init_error = 0;
    int op_stat_sos_flag = 0;
    int op_stat_ems_flag = 0;
    int op_stat_soft_estop_occur = 0;
    int op_stat_collision_occur = 0;
    int op_stat_self_collision = 0;
    // sdata.robot_state: 1 = Idle (no motion command), 3 = executing motion,
    // 5 = conveyor/force, 60+ = MovePB/ITPL waypoint. Used to gate freedrive
    // entry (direct teaching requires the controller to be idle).
    int robot_state = 0;
    // sdata.is_freedrive_mode: 1 = free-drive (gravity-compensation) on, 0 = off.
    // The controller's ground-truth direct-teaching state (set_freedrive_mode only
    // ACKs receipt, so this is the only reliable confirmation that teach engaged).
    int is_freedrive_mode = 0;
};

struct RbpodoStateDecodeOptions {
    bool controller_simulation_unreliable_status_fields_unavailable = false;
    // True only when the REAL physical-motion suspect-diagnostics gate opened the
    // unavailable-fields policy (operator-visible); distinct from the controller-sim
    // carve-out so real-motion telemetry is unambiguous.
    bool real_motion_suspect_diagnostics_accepted = false;
    // Controller-simulation (pgmode) ONLY: skip the hard op_stat_self_collision (1005)
    // fault, deferring to the server's URDF-mesh CollisionMonitor. Set only when the
    // controller-sim motion gate is open AND config opts in; never for real motion.
    bool demote_self_collision_fault = false;
};

RobotState mapRbpodoSystemStateSnapshot(
    ArmId arm_id,
    const RbpodoSystemStateSnapshot& snapshot
);

RobotState mapRbpodoSystemStateSnapshot(
    ArmId arm_id,
    const RbpodoSystemStateSnapshot& snapshot,
    const BackendConfig& config
);

RobotState mapRbpodoSystemStateSnapshot(
    ArmId arm_id,
    const RbpodoSystemStateSnapshot& snapshot,
    const RbpodoStateDecodeOptions& decode_options
);

std::optional<BackendError> rbpodoStateAcquisitionError(const RobotState& mapped);

// Scans a CobotData receive buffer for complete frames using the SDK wire
// framing ('$', size lo, size hi, type; total frame = size + 4 bytes — see
// rbpodo src/cobot_data.cpp request_data()). Consumes every complete frame
// (erasing them from buf), resyncs past garbage bytes, and returns the raw
// bytes of the NEWEST complete type-0x03 (SystemState) frame, or nullopt if
// none completed. A trailing partial frame is left in buf for the next drain.
// Used by the pipelined readState() path; exposed for hardware-free tests.
std::optional<std::string> extractNewestRbpodoStateFrame(std::string& buf);

// A controller mode switch or activation can leave the dedicated pipelined
// CobotData socket with a response requested before the transition. Re-prime
// that socket only when it is enabled and initialize() actually confirmed a
// state-changing controller transition. Exposed for hardware-free policy tests.
bool rbpodoPipelinedChannelNeedsReprime(
    bool state_read_pipelined,
    bool operation_mode_switch_confirmed,
    bool activation_confirmed
);

std::optional<BackendError> rbpodoMotionReadinessError(
    const BackendConfig& config,
    const RbpodoSystemStateSnapshot& snapshot,
    const RobotState& mapped
);

class RbpodoBackend final : public IRobotBackend {
public:
    RbpodoBackend(ArmId arm_id, const BackendConfig& config);
    ~RbpodoBackend() override;

    BackendResult<RobotState> connect() override;
    BackendResult<RobotState> initialize() override;

    BackendResult<RobotState> readState() override;
    SendServoJResult sendServoJ(const SendServoJRequest& request) override;

    BackendResult<RobotState> stop() override;
    BackendResult<RobotState> resetFault() override;
    BackendResult<RobotState> setFreedrive(bool on) override;

    bool isConnected() const override;
    ArmId armId() const override;
    std::string name() const override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;

    // Non-blocking pipelined readState() (config.state_read_pipelined): consume
    // the response that arrived since the previous tick, then fire this tick's
    // request. Defined only when RB_SERVO_ENABLE_RBPODO is on.
    BackendResult<RobotState> readStatePipelined(uint64_t start_ns);
};

}  // namespace rb_servo
