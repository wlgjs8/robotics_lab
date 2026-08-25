#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>
#include "rb_servo/core/types.hpp"

namespace rb_servo {

enum class ServoIoModel {
    Direct,
    Worker
};

struct BackendConfig {
    BackendType backend_type = BackendType::Rbpodo;
    RunMode run_mode = RunMode::Real;

    std::string name;
    std::string ip;
    std::string operation_mode = "real";

    double command_timeout_sec = 0.2;

    JointArray initial_q_deg{};

    double speed_bar = 0.1;

    double servo_t1_sec = 0.002;
    double servo_t2_sec = 0.05;
    double servo_gain = 1.0;
    double servo_alpha = 0.5;

    // Deprecated rbpodo aliases. Kept synchronized with canonical fields while
    // old configs migrate.
    double servo_time_sec = 0.002;
    double servo_lookahead_sec = 0.05;
    double servo_acc = 0.5;

    // Soft-entry gain ramp for move_servo_j RT-servo (re)engagement.
    // On every server (re)start the controller transitions from soft
    // position-hold into stiff real-time servo_j streaming. With servo_gain=1.0
    // the joints can stiffen in a single 2 ms tick and take up gearbox
    // backlash/compliance on the gravity-loaded joints (J0/J2) -> an audible
    // "clunk" + a few-mm settle on every bring-up (independent of
    // pgmode/activation). Ramping ONLY the proportional gain from
    // servo_gain*servo_soft_entry_gain_start_scale up to servo_gain over
    // servo_soft_entry_sec spreads that take-up out so the joints stiffen
    // gradually. This is a TRANSIENT shaping of how the gain is reached at
    // engagement; it never changes the steady-state servo_gain nor t1/t2/alpha.
    // The ramp re-arms whenever servo_j streaming resumes after a gap >
    // servo_soft_entry_rearm_gap_sec (the same RT-servo re-engagement clunk
    // happens after any stream interruption).
    bool servo_soft_entry_enable = true;
    double servo_soft_entry_sec = 0.08;
    double servo_soft_entry_gain_start_scale = 0.1;  // start gain = servo_gain * this
    double servo_soft_entry_rearm_gap_sec = 0.05;    // stream gap that re-arms the ramp

    // rbpodo-only. When true, Cobot::disable_waiting_ack() makes command calls
    // return after socket send instead of waiting for controller ACK.
    bool disable_waiting_ack = false;
    // rbpodo-only. When true, readState() runs a pipelined non-blocking state
    // exchange on a dedicated CobotData connection: each servo tick consumes
    // the response that arrived since the previous tick and fires the next
    // request, so the RT loop never blocks on the controller round trip. State
    // is therefore one tick (~2 ms at 500 Hz) old. Fail-closed: no frame newer
    // than the blocking path's response timeout (0.2 s) fails the read.
    bool state_read_pipelined = false;
    bool allow_controller_simulation_diagnostics_suspect = false;
    bool controller_simulation_treat_unreliable_status_fields_as_unavailable = false;
    // Controller-simulation (pgmode) ONLY: demote the rbpodo controller's CLEAN
    // op_stat_self_collision (code 1005) hard RobotFault to non-latching, deferring to the
    // server's trusted async URDF-mesh CollisionMonitor (which keeps enforcing). The vendor
    // self-collision flag false-positives at full-amplitude TcpPoseTarget replay in pgmode.
    // Default false (clean self_collision still latches); effective only when the
    // controller-sim motion gate is open, so it can NEVER affect real (operation_mode: real)
    // motion. Does NOT touch EMS/SOS/soft-estop/collision_occur/init faults.
    bool controller_simulation_demote_self_collision_fault = false;
    // Real (operation_mode: real) physical-motion opt-in mirror of the field above:
    // accept the same vendor-unreliable status fields (op_stat_self_collision shape,
    // robot_time) as UNAVAILABLE instead of latching diagnostics_suspect. Fail-closed,
    // gated by rbpodoSuspectDiagnosticsRealMotionGateOpen (needs operation_mode==real +
    // this config opt-in). Does NOT suppress EMS/SOS/soft-estop/collision_occur/
    // unknown-mode faults.
    bool allow_real_motion_with_suspect_diagnostics = false;
    bool allow_controller_simulation_init_error = false;

    // rbpodo controller-simulation only: tolerate up to N consecutive transient
    // readState misses (no CobotData frame within the read window) by holding the
    // last valid state and staying connected, instead of declaring the controller
    // disconnected on the first miss. A sustained outage still trips after N
    // consecutive misses. 0 = no tolerance (fail-closed, default). Ignored for
    // physical real operation (operation_mode != simulation).
    int max_consecutive_read_misses = 0;
};

struct ArmMountConfig {
    ArmId arm_id = ArmId::Left;
    Pose6D base_pose_in_stand;
};

struct IkSolverConfig {
    bool enable = true;
    int max_iterations = 50;
    double timeout_ms = 2.0;
    double damping = 0.001;
    double position_tolerance_m = 0.001;
    double orientation_tolerance_rad = 0.02;
    JointArray max_step_deg{2.0, 2.0, 2.0, 3.0, 3.0, 4.0};
    // Selective singularity-robust damping (Nakamura-Hanafusa). When the task
    // Jacobian's smallest singular value sigma_min drops below
    // singular_region_eps, damping on the (near-)singular direction is ramped
    // up to at most damping_max, trading Cartesian accuracy in the unachievable
    // direction for joint-space continuity. This stops the DLS step from
    // blowing up along a degenerate direction and flipping to a distant IK
    // branch (a ~5 deg single-tick joint jump for a ~0 mm Cartesian move near a
    // wrist singularity). Outside the singular region the base `damping` is used
    // unchanged, so well-conditioned tracking accuracy is preserved. Both <= 0
    // disables the ramp (pure constant `damping`).
    double singular_region_eps = 0.0;
    double damping_max = 0.0;
    // Observability guard: when > 0, flag the solve (telemetry
    // ik_branch_jump_suspected) if the returned solution differs from the seed
    // by more than this many degrees on any joint. On its own it does NOT alter
    // the solution; it surfaces branch-jump events for diagnosis. The clamp
    // fields below turn it into an actual correction.
    double max_solution_jump_deg = 0.0;
    // Branch-jump CLAMP (acts on the solution, not just observe). When
    // max_solution_jump_deg > 0 and a converged solution jumps more than that
    // from the seed:
    //   1) if branch_jump_damping_scale > 1 and branch_jump_max_retries > 0,
    //      re-solve the SAME tick with damping multiplied by the scale (escalating
    //      per retry) to pull the step back onto the local branch — the first
    //      attempt whose jump is within threshold wins;
    //   2) if it still exceeds the threshold, the OVERSHOOT policy decides:
    //      - branch_jump_rate_limit == true: scale the whole seed->solution joint
    //        delta so the largest per-joint step equals max_solution_jump_deg —
    //        the arm advances toward the solution along the same joint-space
    //        direction at a bounded joint speed (no freeze, no abrupt flip). The
    //        seed advances every tick, so there is no deadlock and the threshold
    //        doubles as the smoothness/lag knob (telemetry reason
    //        "branch_jump_rate_limited", ik_branch_jump_clamped stays false).
    //      - else if branch_jump_clamp_to_seed == true: return the SEED (zero
    //        motion this tick). WARNING: deadlocks under streaming targets — the
    //        frozen seed keeps re-exceeding the threshold, so the arm stays
    //        clamped until the target returns near the frozen pose. Prefer
    //        rate_limit. (telemetry ik_branch_jump_clamped).
    //      - else: return the flagged (most-damped) solution unchanged (allow;
    //        can be rough on high-gain motion).
    // branch_jump_rate_limit takes precedence over branch_jump_clamp_to_seed.
    // All default-off (scale <= 1 / retries <= 0 / rate_limit & clamp false) =>
    // pure observability, behavior unchanged.
    double branch_jump_damping_scale = 0.0;
    int branch_jump_max_retries = 0;
    bool branch_jump_clamp_to_seed = false;
    bool branch_jump_rate_limit = false;
    // JOINT-LIMIT BEST EFFORT. When the DLS iteration clamps a joint to its range
    // the solve returns kReasonJointLimit and the whole tick is refused, so the arm
    // holds — including the components of the requested motion that were perfectly
    // feasible. Measured 2026-08-25 (pi0.5 rollouts, J3 pinned at +/-150 deg): the
    // clamped iterate sat 34 um / 9e-5 rad from the target while position_tolerance_m
    // is 20 um, so the solve failed by 14 um and the arm froze for seconds.
    // When these are > 0 and the clamped iterate is within them, accept it as a
    // success (telemetry reason "joint_limit_best_effort") so the arm keeps tracking
    // every direction the limit does not block. Keep them small: the residual IS
    // unrealized command, and it is what the follower lead/divergence guards measure.
    // Both <= 0 disables (hard failure, previous behavior).
    double joint_limit_best_effort_position_tolerance_m = 0.0;
    double joint_limit_best_effort_orientation_tolerance_rad = 0.0;
};

struct KinematicsConfig {
    bool enable = false;
    std::string provider = "none";
    std::string urdf = "descriptions/urdf/rb3_730e.urdf";
    std::string base_link = "world";
    std::string tip_link = "tcp";
    std::vector<std::string> joint_names{
        "base_joint",
        "shoulder_joint",
        "elbow_joint",
        "wrist1_joint",
        "wrist2_joint",
        "wrist3_joint",
    };
    std::string q_units = "deg";
    bool publish_tcp = false;
    IkSolverConfig ik;
};

enum class ControllerSimulationTrackingErrorSource {
    Actual,
    Reference
};

enum class ControllerSimulationPhysicalMotionPolicy {
    WarnOnly,
    FaultLatch
};

enum class RbpodoAsyncStreamingMode {
    Disabled,
    SdkAckWorker,
    SocketSendSupervised
};

enum class RbpodoAsyncQueuePolicy {
    LatestWins
};

enum class RbpodoAsyncReferenceSupervisionPolicy {
    WarnOnly,
    FaultLatch
};

struct RbpodoAsyncAckSupervisionConfig {
    bool enable = true;
    double expected_ack_timeout_ms = 50.0;
    double missing_ack_fault_after_ms = 100.0;
    int max_consecutive_missing_ack = 10;
};

struct RbpodoAsyncReferenceSupervisionConfig {
    bool enable = true;
    double q_ref_update_timeout_ms = 50.0;
    double q_ref_target_tolerance_deg = 1.0;
    double q_ref_target_fault_after_ms = 100.0;
    double tcp_ref_update_timeout_ms = 50.0;
    double tcp_ref_target_tolerance_m = 0.02;
    double tcp_ref_target_fault_after_ms = 100.0;
    RbpodoAsyncReferenceSupervisionPolicy policy =
        RbpodoAsyncReferenceSupervisionPolicy::FaultLatch;
};

struct RbpodoAsyncDiagnosticsConfig {
    bool publish_per_command_jsonl = false;
};

struct RbpodoAsyncStreamingConfig {
    bool enable = false;
    RbpodoAsyncStreamingMode mode = RbpodoAsyncStreamingMode::Disabled;
    int rate_hz = 500;
    RbpodoAsyncQueuePolicy queue_policy = RbpodoAsyncQueuePolicy::LatestWins;
    double max_pending_age_ms = 10.0;
    RbpodoAsyncAckSupervisionConfig ack_supervision;
    RbpodoAsyncReferenceSupervisionConfig reference_supervision;
    RbpodoAsyncDiagnosticsConfig diagnostics;
};

enum class SelfCollisionFailPolicy {
    ClampToHold,
    FaultLatch,
};

// Extra collision primitive for the mesh self-collision monitor — geometry the
// URDF does not carry: wrist cameras, cable bundles, the work table/box. Attached
// to a named frame: an ARM frame (e.g. an attachment_site) makes it move with that
// arm (auto-classified left/right by ancestry); "stand"/"world" makes it a static
// obstacle paired against both arms. Shapes are coal primitives.
struct ExtraCollisionConfig {
    std::string name;
    std::string shape = "box";       // box | sphere | capsule | cylinder
    std::string parent_frame;        // URDF frame to attach to (arm frame or stand/world)
    std::array<double, 3> size_m{0.0, 0.0, 0.0};  // box: full extents (x,y,z)
    double radius_m = 0.0;           // sphere/capsule/cylinder
    double length_m = 0.0;           // capsule/cylinder (length between caps / height)
    std::array<double, 3> xyz_m{0.0, 0.0, 0.0};   // offset in the parent frame
    std::array<double, 3> rpy{0.0, 0.0, 0.0};      // orientation in the parent frame
};

// Server-side self-collision guard treating stand + left arm + right arm as one
// "self" (the rbpodo controller firmware does not populate op_stat_self_collision).
// The ONLY implementation is the async URDF-mesh CollisionMonitor (pinocchio +
// coal): every candidate joint target is checked against the dual-arm + stand
// collision geometry and refused (velocity barrier / fault) before it is sent.
// There is no capsule approximation path.
struct SelfCollisionConfig {
    // Master switch for the URDF-mesh self-collision guard. When true the servo
    // loop feeds every candidate target to the monitor and applies the shared
    // velocity barrier; a hard breach, or a stale/absent monitor, fails closed.
    bool enable = false;
    SelfCollisionFailPolicy fail_policy = SelfCollisionFailPolicy::ClampToHold;
    // Observe-only: still evaluate and publish clearance/violation telemetry, but
    // do NOT clamp or latch. For tuning d_hard/d_slow in simulation against a known
    // collision-free trajectory. Never use monitor_only as a real-motion safety
    // posture.
    bool monitor_only = false;

    // URDF mesh self-collision parameters (consumed only when enable is true). The
    // monitor runs on a separate thread (off the 2 ms servo_j path) + a velocity
    // barrier. Barrier params are a SINGLE shared set, common to every motion
    // primitive (JointTarget, TcpPoseTarget, TcpLinearMove) — speed adaptation comes from
    // the measured closing speed, so nothing is tuned per primitive.
    struct MeshConfig {
        std::string unified_urdf;        // stand+both-arms URDF (e.g. dual_rb3_730e_ver3.urdf)
        std::vector<std::string> package_dirs;  // resolve mesh "../../../meshes" paths
        std::string pika_gripper_mesh;   // optional; attached as a convex hull per arm
        // Optional ARTICULATED gripper collision: a static base hull + two movable
        // finger hulls (convex hulls of the visual STLs) attached at attachment_site,
        // the fingers tracking the live jaw open percent (mirrors the articulated URDF).
        // When all three are set they take precedence over the single pika_gripper_mesh.
        std::string pika_gripper_base_mesh;
        std::string pika_finger_left_mesh;
        std::string pika_finger_right_mesh;
        double gripper_finger_travel_m = 0.047;  // per-finger jaw travel open(0)->closed
        std::string left_prefix = "dual_rb3_730e_left_";
        std::string right_prefix = "dual_rb3_730e_right_";
        std::string stand_frame = "stand";
        // arm geometry whose name contains any of these is NOT paired vs the stand
        std::vector<std::string> stand_ignore_arm_substrings{"link0"};
        // Left/right classification by kinematic-tree ancestry (robust to mesh/link
        // renaming). Default arm root frame = "<prefix>world" if left empty.
        std::string left_arm_root_frame;
        std::string right_arm_root_frame;
        // Intra-arm self-collision (an arm folding onto itself); adjacent links
        // (chain separation < intra_arm_min_chain_separation) are skipped.
        bool check_intra_arm = true;
        int intra_arm_min_chain_separation = 2;
        // SRDF-style curated structural false-positive pairs. Matching is symmetric;
        // each side supports exact strings or '*' globs against geometry names.
        std::vector<CollisionPairPattern> disabled_collision_pairs;
        bool debug_pair_curation = false;
        // Swept-volume guard: samples between consecutive evaluations (1 = endpoint
        // only). >=2 prevents fast motion tunneling a thin obstacle between ticks.
        int swept_samples = 2;  // 1=endpoint, >=2 sweeps (cost ~x per sample)
        // shared velocity-barrier params
        double d_hard_m = 0.005;
        double d_slow_m = 0.025;
        double a_brake_m_s2 = 4.0;
        double hyst_m = 0.005;
        // Velocity-damper projection (Stage 2): Gauss-Seidel sweeps over active near
        // pairs, and optional active push-out speed below d_hard (0 = only block
        // deeper penetration, do not push the arm out).
        int projection_iterations = 3;
        double recover_speed_m_s = 0.0;
        double latency_s = 0.010;
        // Verdict older than this -> hold (recoverable, not a latch). Loose enough
        // to ride out normal OS scheduling jitter of the (non-RT) monitor thread;
        // the monitor normally refreshes every ~1.5 ms.
        double max_staleness_s = 0.050;
        int monitor_core = -1;
        int max_near_pairs = 8;
        // VISUALIZATION ONLY (does not affect the barrier): publish near-pair witness
        // segments to the GUI for any checked pair within this clearance. Decoupled
        // from d_slow so close-call markers stay visible even when the barrier band
        // (d_slow) is tuned tight. Effective threshold = max(this, d_slow).
        double viz_near_pairs_m = 0.06;
        // Extra collision primitives not in the URDF (wrist cameras, cables, table).
        std::vector<ExtraCollisionConfig> extra_collision;
        // Whole-arm floor: a large thin box injected as a static (world-frame)
        // collision obstacle so the SAME mesh barrier that guards arm<->arm /
        // arm<->stand also keeps EVERY arm link above the floor (the
        // floor_constraint plane only checks the TCP + its offset points, so an
        // elbow/wrist can dip below it). When enable is true a box of lateral
        // extent size_m=[Lx,Ly] and thickness_m is attached to parent_frame at
        // z = z_m (its top face), classified as Stand and paired against both
        // arms (minus stand_ignore_arm_substrings). Keep z_m <= the TCP floor
        // (floor_constraint.z_min_m) so the two floors do not fight.
        struct GroundPlaneConfig {
            bool enable = false;
            double z_m = 0.0;                  // top face height in parent_frame (m)
            std::array<double, 2> size_m{4.0, 4.0};  // lateral full extents [Lx, Ly] (m)
            double thickness_m = 0.10;         // box thickness below z_m (m)
            std::string parent_frame = "world";  // static frame (world/stand)
            // When true the whole-arm floor box is no longer a static plane at z_m:
            // the servo loop drives its pose at runtime to track the OPERATOR's active
            // viser floor — the user floor plane when userFloorActive(), else the stand
            // floor (horizontal at effectiveFloorZ()) when floorConstraintActive(), and
            // disabled (moved far below, inert) when NEITHER is active. Requires
            // enable=true (the box geometry must still be built at startup to be moved).
            // z_m is then only the startup/fallback height. NOTE: the InitMotion planner
            // keeps its own static copy (config plane), so it stays conservative.
            bool follow_safety_floors = false;
        };
        GroundPlaneConfig ground_plane;

        // EXTERNAL-collision barrier params, applied to arm<->external-obstacle pairs
        // (currently the ground_plane whole-arm floor) instead of the self-collision
        // set above. Lets the floor — a known surface the operator approaches on purpose
        // — be cleared by a smaller margin than the robot keeps from itself. Also used
        // by the InitMotion planner's external clearance gate. d_hard defaults tighter
        // (3 mm) than self (5 mm).
        struct ExternalConfig {
            double d_hard_m = 0.003;
            double d_slow_m = 0.025;
            double a_brake_m_s2 = 4.0;
            double hyst_m = 0.005;
            double recover_speed_m_s = 0.0;
            double latency_s = 0.010;
        };
        ExternalConfig external;

        // Intra-arm-only self-collision barrier params. These apply only to
        // same-arm non-adjacent link pairs, separately from arm<->arm and
        // arm<->stand self pairs. Negative values inherit the corresponding
        // self-collision value during runtime config conversion.
        struct IntraArmConfig {
            double d_hard_m = -1.0;
            double d_slow_m = -1.0;
            double a_brake_m_s2 = -1.0;
            double hyst_m = -1.0;
            double recover_speed_m_s = -1.0;
            double latency_s = -1.0;
        };
        IntraArmConfig intra_arm;

        // Preallocated external keep-out boxes updated at runtime by the leaseless
        // SetExternalBoxes command. Disabled by default; when enabled the monitor
        // builds exactly max_count box geometries at startup.
        struct ExternalBoxesConfig {
            bool enable = false;
            int max_count = 2;
            std::array<double, 3> size_m{0.380, 0.240, 0.105};
            std::array<double, 3> margin_m{0.025, 0.025, 0.025};  // per-axis [x,y,z] box-local inflation; index 2 = height
            bool monitor_only = true;
            double stale_timeout_s = 0.5;
            std::string stale_policy = "hold";
            // Box-only keep-out velocity-barrier params, SEPARATE from the floor's
            // `external` set: a box is a keep-out the operator drives toward at teleop
            // speed, so the slow zone must be wide enough to brake the fastest approach
            // (the floor's 5 mm zone stops only ~0.12 m/s and let teleop overshoot ~40 mm
            // in). Defaults stop ~0.9 m/s and eject on penetration.
            struct BarrierConfig {
                double d_hard_m = 0.010;
                double d_slow_m = 0.080;
                double a_brake_m_s2 = 6.0;
                double hyst_m = 0.010;
                double recover_speed_m_s = 0.030;
                double latency_s = 0.010;
            };
            BarrierConfig barrier;
        };
        ExternalBoxesConfig external_boxes;
    };
    MeshConfig mesh;
};

enum class FloorConstraintFailPolicy {
    ClampToHold,
    FaultLatch,
};

// One named extra floor-check point, expressed as an offset in the TCP frame.
// offset_m is the gripper-OPEN position (gripper percent = 100); offset_closed_m
// is the gripper-CLOSED position (percent = 0). At runtime the point is linearly
// interpolated by the live gripper open percent: offset = closed + t*(open-closed),
// t = clamp(percent,0,100)/100 (see interpolateOffsetPoints in floor_constraint.hpp).
// has_closed=false means no closed value was configured; the parser then mirrors
// offset_closed_m = offset_m so interpolation is the identity (static point, the
// legacy behavior) and the gripper percent has no effect on this point.
struct FloorCheckPointConfig {
    std::string name;
    std::array<double, 3> offset_m{};         // OPEN (gripper percent = 100)
    std::array<double, 3> offset_closed_m{};  // CLOSED (gripper percent = 0)
    bool has_closed = false;
};

// Stand-frame floor plane constraint: the TCP of either arm must never go below
// z = z_min_m (meters, stand frame), regardless of motion primitive or run mode.
// Tier 1 (hard backstop) FK-checks every candidate joint target at the final
// safety gate; Tier 2 clamps Cartesian targets / negative stand v_z so streaming
// commands slide along the plane. z_min_m is runtime-adjustable via the leaseless
// SetSafetyFloorZ command, bounded to [runtime_min_z_m, runtime_max_z_m].
struct FloorConstraintConfig {
    bool enable = false;
    double z_min_m = 0.010;
    // Operator-adjustable lower bound for SetSafetyFloorZ. Allowed below the stand
    // origin (down to -0.2 m) so the floor can be lowered under z=0; this only
    // widens what the operator MAY set — the applied floor stays z_min_m until a
    // SetSafetyFloorZ command moves it. Config validation still requires
    // runtime_min_z_m <= z_min_m <= runtime_max_z_m.
    double runtime_min_z_m = -0.2;
    double runtime_max_z_m = 0.5;
    FloorConstraintFailPolicy fail_policy = FloorConstraintFailPolicy::ClampToHold;
    // Observe-only: publish per-arm tcp z / violation telemetry without clamping
    // or latching. Never use monitor_only as a real-motion safety posture.
    bool monitor_only = false;
    // Additional floor-check points expressed in the TCP frame (meters). The
    // TCP point is always checked; each entry adds one more point at
    // tcp_position + R_tcp * offset_m — e.g. the two PIKA gripper fingertips,
    // which dip below the TCP point when the tool rotates. The published
    // per-arm tcp_z_m becomes the LOWEST checked point's z.
    std::vector<FloorCheckPointConfig> tcp_offset_points;
    // Velocity-damper projection (Stage 3): when within d_slow_m of the plane the
    // commanded downward (closing) speed of the lowest point is limited to
    // sqrt(2 a_brake (z - z_min)) so it brakes to zero AT the plane and slides
    // along it; lateral/upward motion is free. Replaces the binary Hold revert.
    double a_brake_m_s2 = 4.0;
    double d_slow_m = 0.05;  // engage band above the plane (0 => always active)
};

// Stand-frame axis-aligned ROI box (workspace limit): the TCP of either arm —
// and each configured TCP-frame offset point — must stay INSIDE the box
// min_m[k] <= p_k <= max_m[k] for stand-frame axes k in {x, y, z}, regardless of
// motion primitive or run mode. Independent of safety.floor_constraint: both
// apply when enabled (the box's z lower face and the floor plane may overlap;
// the stricter wins). Enforced with the SAME velocity-damper projection as the
// floor — each of the 6 box faces contributes one closing-velocity row to the
// shared Gauss-Seidel solve, so the commanded speed of the most-exposed point
// toward a face is limited to sqrt(2 a_brake (margin)) and brakes to zero AT the
// face (tangential/inward motion stays free). Tier-2 clamps Cartesian targets so
// streaming commands slide along a face. Bounds are runtime-adjustable via the
// leaseless SetSafetyRoiBounds command, each value bounded to
// [runtime_min_m[k], runtime_max_m[k]]. monitor_only publishes telemetry without
// clamping or latching (never a real-motion posture).
struct RoiBoxConfig {
    bool enable = false;
    std::array<double, 3> min_m{-0.5, -1.0, 0.0};  // stand frame, startup bounds
    std::array<double, 3> max_m{0.5, 0.0, 1.0};
    // SetSafetyRoiBounds envelope: a runtime request is rejected unless
    // runtime_min_m[k] <= requested_min_m[k] <= requested_max_m[k] <= runtime_max_m[k].
    std::array<double, 3> runtime_min_m{-1.0, -1.5, -0.2};
    std::array<double, 3> runtime_max_m{1.0, 0.5, 1.5};
    FloorConstraintFailPolicy fail_policy = FloorConstraintFailPolicy::ClampToHold;
    bool monitor_only = false;
    // Additional check points expressed in the TCP frame (meters), same as the
    // floor's tcp_offset_points (e.g. the two PIKA gripper fingertips). The TCP
    // point is always checked; each face binds on the most-exposed point in that
    // face's direction.
    std::vector<FloorCheckPointConfig> tcp_offset_points;
    double a_brake_m_s2 = 4.0;
    double d_slow_m = 0.05;  // engage band inside each face (0 => always active)
};

// Per-arm reachable-workspace shell limit. The TCP (and each configured
// TCP-frame offset point) must stay inside the spherical shell
// r_min_m <= ||p_stand - base_stand|| <= r_max_m, where base_stand is the arm's
// mount origin in the stand frame (left_mount/right_mount.base_pose_in_stand).
// This bounds the radial distance from the shoulder so a Cartesian command never
// drives the TCP past the arm's actual reach (where IK fails / the arm hits a
// full-extension singularity and the legacy behavior was to silently stop). It is
// enforced with the SAME velocity-damper projection as the floor and ROI box: the
// outer shell adds one closing-velocity row limiting d(r)/dt of the farthest
// point to <= +sqrt(2 a_brake (r_max - r)), the inner shell limits the nearest
// point's d(r)/dt to >= -sqrt(2 a_brake (r - r_min)); both brake to zero AT the
// shell and let tangential / returning motion stay free, so the TCP slides along
// the reach boundary instead of stalling. r_max_m/r_min_m MUST be measured with
// tools/reach_envelope.py (FK Monte-Carlo of THIS urdf's tip frame) minus a safety
// margin; the defaults below are the measured RB3-730E `tcp`-frame envelope
// (radius from the arm base/mount origin, which includes the tool offset, so the
// raw reach is ~1.09 m, not the 0.73 m nominal arm reach). monitor_only publishes
// telemetry without clamping or latching (never a real-motion posture). r_min_m
// <= 0 disables the inner shell.
struct ReachConstraintConfig {
    bool enable = false;
    double r_max_m = 1.050;  // outer reach shell (m), measured raw ~1.088 - margin
    double r_min_m = 0.130;  // inner shell (m); <= 0 disables the inner limit
    FloorConstraintFailPolicy fail_policy = FloorConstraintFailPolicy::ClampToHold;
    bool monitor_only = false;
    // Additional check points in the TCP frame (meters), same as the floor/ROI
    // tcp_offset_points (e.g. the two PIKA gripper fingertips). The TCP point is
    // always checked; the outer shell binds on the farthest point, the inner shell
    // on the nearest.
    std::vector<FloorCheckPointConfig> tcp_offset_points;
    double a_brake_m_s2 = 4.0;
    double d_slow_m = 0.05;  // engage band inside each shell (0 => always active)
};

// Stand-frame USER-defined tilted floor plane (half-space): the TCP of either arm
// — and each configured TCP-frame offset point — must satisfy
// n . (p_stand - point_m) >= margin_m, where n is a unit normal pointing into the
// allowed (upper) half-space. Unlike floor_constraint (a HORIZONTAL plane
// z >= z_min_m), this plane may be tilted, so it can be fit to a physical floor
// that is not parallel to the stand z=0 plane. The plane is fit in the GUI from
// >= 3 captured floor-contact points (both arms) and pushed at runtime via the
// leaseless SetUserSafetyFloorPlane command (bounded by validateUserFloorPlaneRequest:
// unit normal, max_tilt_deg from vertical with n.z > 0, point z within
// [runtime_min_point_z_m, runtime_max_point_z_m], margin within [0, max_margin_m]).
// Independent of and ADDITIVE to floor_constraint / roi_box / reach_constraint:
// every enabled constraint applies (the stricter wins), enforced with the SAME
// velocity-damper projection using n as the stand-frame direction for every point.
// monitor_only publishes telemetry without clamping/latching (never a real posture).
struct UserFloorConstraintConfig {
    bool enable = false;
    // Whether point_m/normal hold a usable plane at startup. false (default) keeps
    // the constraint inert (always Allow) until a SetUserSafetyFloorPlane arrives,
    // even when enable=true — so a site can opt in without baking in a stale plane.
    bool has_initial_plane = false;
    std::array<double, 3> point_m{0.0, 0.0, 0.0};   // p0, stand frame
    std::array<double, 3> normal{0.0, 0.0, 1.0};    // n, unit, into allowed half-space
    double margin_m = 0.0;                           // lift the plane up by this much
    // Runtime-request envelope (the tilted-plane analog of floor runtime_min/max_z):
    double max_tilt_deg = 35.0;            // max angle of n from vertical (+z); n.z must be > 0
    double runtime_min_point_z_m = -0.2;   // point_m z bounds
    double runtime_max_point_z_m = 0.5;
    double max_margin_m = 0.2;
    FloorConstraintFailPolicy fail_policy = FloorConstraintFailPolicy::ClampToHold;
    bool monitor_only = false;
    // Additional check points in the TCP frame (meters), same as floor/ROI/reach
    // tcp_offset_points (e.g. the PIKA gripper fingertips). The TCP point is always
    // checked; the most-exposed point (lowest signed distance) binds the plane.
    std::vector<FloorCheckPointConfig> tcp_offset_points;
    double a_brake_m_s2 = 4.0;
    double d_slow_m = 0.05;  // engage band above the plane (0 => always active)
};

// Joint-space SMD profile for the JointTarget primitive (the joint-space
// mirror of cartesian_control.pose_track_smd): the sent target follows the
// commanded goal as a second-order system (mass fixed at 1.0) stepped at the
// servo rate, per joint:
//   ddq = wn^2 * (goal - q) - 2 * zeta * wn * dq
// The per-joint velocity/accel clamps are the max-force saturation; inside the
// limits the zeta/fn dynamics are exact. These are PROFILE limits — the global
// safety.dq_max_deg_s / ddq_max_deg_s2 clamps still apply downstream as the
// outer hard safety bound. Disabled (default): JointTarget keeps the legacy
// full-speed rate-limited ramp at dq_max.
struct JointTargetSmdConfig {
    bool enable = false;
    double damping_ratio = 1.0;        // 1.0 = critical damping (no overshoot)
    double natural_frequency_hz = 0.4;
    JointArray max_velocity_deg_s{30.0, 30.0, 30.0, 45.0, 45.0, 60.0};
    JointArray max_accel_deg_s2{150.0, 150.0, 150.0, 250.0, 250.0, 350.0};
    // Arrival-decel taper (decoupled from natural_frequency). When a command carries an
    // explicit final stop (InitMotion pursuit), the per-step joint velocity is uniformly
    // capped at sqrt(2*arrival_decel_deg_s2*d) — d = max per-joint distance to the stop —
    // so the last stretch eases in gently while the snappy start/cruise (governed by
    // natural_frequency) is unchanged. Uniform scaling preserves the straight-line joint
    // path. Smaller arrival_decel = gentler/longer glide-in. The min-speed floor keeps the
    // tail from crawling so it settles into waypoint_tol quickly. Disabled => no taper.
    bool arrival_taper_enable = false;
    double arrival_decel_deg_s2 = 200.0;
    double arrival_min_speed_deg_s = 3.0;
};

// Collision-free JointTarget init_motion profile planner (server-side). A direct
// JointTarget PTP can pass through a self-colliding or floor-violating
// configuration; the reactive barrier then brakes the arms at the boundary and
// they never reach the init pose. When enabled, a JointTarget carrying
// joint_target_profile=init_motion triggers a 12-DOF (both arms) RRT-Connect plan
// for a collision-free + floor-safe joint path (oracle = a private CollisionMonitor
// incl. the ground plane), which the server streams as ordinary JointTarget setpoints
// through the full safety gate. Requires self_collision.enable (the mesh model is
// the planner's oracle). Fail-closed: planning failure holds.
struct InitMotionPlannerConfig {
    bool enable = false;
    double max_planning_time_sec = 2.0;   // wall-clock budget per plan (off the RT loop)
    int max_iterations = 20000;           // RRT-Connect node cap before giving up
    double step_size_rad = 0.20;          // RRT extension step (per-joint, combined space)
    double edge_resolution_rad = 0.02;    // local-planner collision sampling step
    double goal_bias = 0.10;              // probability of sampling the goal directly
    int shortcut_passes = 200;            // post-process straightening attempts
    double sample_margin_deg = 30.0;      // per-joint sampling band beyond [start,goal]
    // Optional per-joint sample margins (left/right share axis index). When all entries
    // are <=0, sample_margin_deg is used exactly as before; positive entries override
    // only their axis and zero entries fall back to sample_margin_deg.
    JointArray sample_margin_deg_per_joint{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double global_sample_fraction = 0.0;     // probability of sampling the wide band
    double global_sample_margin_deg = 150.0; // wide-band bbox(start,goal) margin
    double collision_margin_m = 0.005;    // oracle clearance threshold (extra over d_hard)
    unsigned int seed = 12345;            // RNG seed (reproducible plans/tests)
    double waypoint_tol_deg = 1.5;        // arrival tolerance at the FINAL waypoint
    // If the selected active arm(s) are already at the target within this tolerance,
    // InitMotion completes as a no-op. This prevents a start==goal pose that is merely
    // inside the planner's clearance-margin band from first executing an unnecessary
    // gradient-escape wiggle and then returning to the same pose.
    double noop_tol_deg = 0.75;
    double max_segment_deg = 5.0;         // densify so no segment exceeds this per joint
    double escape_max_time_sec = 0.75;     // escape-only budget before RRT budget starts
    int escape_max_steps = 40;             // gradient escape iteration cap
    int escape_restart_attempts = 4;       // random perturbation restarts after saddle
    double escape_perturb_deg = 5.0;       // per-joint restart perturbation half-width
    bool lazy_edges = true;                // validate RRT edges lazily, final path fail-closed
    // Execution pure-pursuit lookahead: each tick the streamed JointTarget aims at the
    // farthest planned waypoint within this joint-space chord of the current pose, so
    // the SMD always sees a large error and runs near max velocity (instead of settling
    // at every densified waypoint -> stop-and-go crawl). Larger = faster but cuts path
    // corners more (the reactive barrier nets any cut into a keep-out); smaller hugs the
    // planned path. ~25 deg saturates the default joint_target_smd profile.
    double execution_lookahead_deg = 25.0;
    // InitMotion is a one-shot command but the move can outlast the command's
    // freshness window (timeout_sec). Once a plan starts executing the server keeps
    // driving it to completion EVEN IF the one-shot command goes stale (deadman
    // synthesises a Hold) — a brief barrier pause is fine, the move still finishes
    // from a single click. An explicit operator command (Hold/Disarm/E-stop/new
    // motion) still cancels immediately, and the per-tick safety gate stays active.
    // This is the runaway bound: if the sequence has not finished within this many
    // seconds it gives up (Failed -> hold), so a permanently barrier-blocked corner
    // cannot hold motion authority forever.
    double execution_timeout_sec = 30.0;
    // false: single-arm InitMotion preserves the non-selected arm command so
    // flow-infer can keep controlling it; true: hold/rewrite the other arm too.
    bool single_arm_freeze_other_arm = false;
    // Brake-before-plan (2026-08-19). InitMotion used to snap the SENT target to the
    // MEASURED joints and Hold the instant a request arrived (reanchor_selected_to_measured
    // + hold_selected while planning). While the arm is still streaming under the policy
    // (or under an earlier init move — the a->c double press replans both arms), the sent
    // target leads the encoders by the tracking lag, so that snap is a 0.2-0.7 deg
    // BACKWARD step inside one 2 ms tick (100-350 deg/s sent-velocity spike, 8-26 N on the
    // wrist F/T, tool ringing at ~9.5 Hz; servo_log_20260819_085404 t=130.94 / 152.28).
    // With brake_before_plan the selected arm(s) instead decelerate along the
    // joint_target_smd profile FROM THE LAST SENT TARGET (monotone stop, goal = q + dq/wn,
    // no reversal) and only once every selected joint's sent velocity is below
    // brake_exit_deg_s (or brake_timeout_sec elapsed) does the sequencer re-anchor to
    // measured — now within the encoder noise of the sent target — and launch the
    // collision-free plan from rest. Arms already at rest take the old path unchanged.
    bool brake_before_plan = true;
    double brake_enter_deg_s = 1.0;     // any selected joint moving faster (sent) => brake first
    double brake_exit_deg_s = 0.5;      // braked when every selected joint is slower than this
    double brake_timeout_sec = 0.75;    // upper bound on the brake phase, then plan anyway
    double brake_max_travel_deg = 3.0;  // cap on the per-joint dq/wn stopping distance
};

struct SafetyConfig {
    JointArray q_min_deg{};
    JointArray q_max_deg{};
    JointArray dq_max_deg_s{};
    JointArray ddq_max_deg_s2{};
    JointArray joint_wrap_period_deg{};
    // Per-axis opt-out from JointTarget shortest-path +/-360 goal selection.
    // true keeps the commanded raw target exactly (used for cable-sensitive J6).
    JointBoolArray joint_target_literal_axes{};

    double command_timeout_sec = 0.2;
    double max_tracking_error_deg = 10.0;

    // mock can use SnapToActual for fast iteration. real should use FaultLatch.
    TrackingErrorPolicy tracking_error_policy = TrackingErrorPolicy::FaultLatch;

    bool stop_both_arms_on_single_arm_error = true;
    bool latch_fault_on_robot_state_error = true;
    bool joint_wrap_for_startup_validation = false;
    bool joint_wrap_for_motion_safety = false;
    ControllerSimulationTrackingErrorSource controller_simulation_tracking_error_source =
        ControllerSimulationTrackingErrorSource::Actual;
    ControllerSimulationPhysicalMotionPolicy controller_simulation_physical_motion_policy =
        ControllerSimulationPhysicalMotionPolicy::FaultLatch;
    double controller_simulation_physical_motion_threshold_deg = 0.05;
    // pgmode controller-sim only (opt-in, default false). When the controller-sim
    // motion gate is open, the reference/actual tracking-error divergence is treated
    // as ADVISORY (degraded telemetry + throttled WARN) instead of latching
    // SafetyVerdict::TrackingError. The diagnostics_suspect controller's reference
    // readback lags the commanded joints with no physical motion, so the latch is
    // spurious there. Inert in real mode (gate closed → keeps latching). Does NOT
    // affect the controller_simulation_physical_motion guard, which still latches.
    bool controller_simulation_tracking_error_nonlatching = false;
    SelfCollisionConfig self_collision;
    FloorConstraintConfig floor_constraint;
    RoiBoxConfig roi_box;
    ReachConstraintConfig reach_constraint;
    UserFloorConstraintConfig user_floor_constraint;
    JointTargetSmdConfig joint_target_smd;
    InitMotionPlannerConfig init_motion_planner;
};

inline constexpr JointArray rbpodoDefaultSafetyJointMinDeg() {
    return JointArray{-360.0, -360.0, -360.0, -360.0, -360.0, -360.0};
}

inline constexpr JointArray rbpodoDefaultSafetyJointMaxDeg() {
    return JointArray{360.0, 360.0, 360.0, 360.0, 360.0, 360.0};
}

struct ServoConfig {
    int rate_hz = 500;
    double command_timeout_sec = 0.2;
    ServoIoModel io_model = ServoIoModel::Direct;
    ControlMode startup_mode = ControlMode::Hold;
    bool send_servo_commands = true;
    // When true, the tick's servo_j dispatch happens FIRST, immediately after
    // wake-up, sending the target computed (and safety-filtered) by the
    // PREVIOUS tick. Wire timing then depends only on wake-up latency, not on
    // the tick's compute time. Adds one tick (~2 ms at 500 Hz) of
    // command-to-wire latency. The send policy (fault latch / lease /
    // freedrive / read-only) is re-evaluated at dispatch time, so a fault
    // latched after compute still suppresses the staged target.
    bool send_at_tick_start = false;
    bool allow_readonly_faulted_startup = false;
    bool allow_readonly_q_range_violation_startup = false;
    bool allow_readonly_wrong_mode_startup = false;
    bool allow_controller_simulation_motion = false;
    bool allow_controller_simulation_diagnostics_suspect = false;
    bool controller_simulation_treat_unreliable_status_fields_as_unavailable = false;
    // Controller-simulation (pgmode) ONLY: demote the rbpodo controller's clean
    // op_stat_self_collision (1005) hard fault to non-latching; the server's URDF-mesh
    // CollisionMonitor stays the trusted self-collision guard. See servo struct comment.
    bool controller_simulation_demote_self_collision_fault = false;
    // Real physical-motion (operation_mode: real) opt-in; propagated to both backends.
    bool allow_real_motion_with_suspect_diagnostics = false;
    bool controller_simulation_async_supervision_nonlatching = false;
    bool allow_controller_simulation_init_error = false;
    bool allow_controller_simulation_not_activated = false;
    // Per-arm direct-teaching (free-drive). Fail-closed opt-in: when false, the
    // server rejects every Freedrive command. Enable only on configs that accept
    // releasing servo_j authority for operator hand-guiding (see
    // docs/runbooks/freedrive_direct_teaching.md).
    bool allow_freedrive = false;

    bool enable_realtime_priority = true;
    int realtime_priority = 80;
    int cpu_core = -1;
    // Hybrid sleep-then-spin tail (microseconds). 0 = plain sleep_until (default,
    // unchanged). When > 0 the loop sleeps until `slack` before the tick, then
    // busy-spins the last `slack` so wake-up carries no C-state/scheduler jitter.
    // GUARDED at loop start: spin is silently disabled (falls back to sleep_until)
    // unless enable_realtime_priority is true AND kernel.sched_rt_runtime_us == -1
    // (RT throttling off) — otherwise a ~100% RT spin would be throttled ~50 ms/s.
    // Must be < the tick period. Only worthwhile on a dedicated isolated cpu_core.
    int spin_slack_us = 0;
    double worker_read_period_sec = 0.002;
    // Staleness budget for a worker-cached state read, in control periods.
    //
    // In direct I/O the loop reads the backend itself and the state is ~25-125 us
    // old (measured). In worker I/O it crosses a thread boundary, so its age is
    // roughly the phase offset between the two threads: measured MEDIAN 1567 us
    // at a 2 ms period. The original budget of 2 periods (4 ms) therefore left
    // under one period of margin, and a single tick exceeding it returns a
    // default-constructed RobotState -- no joints, no FK, no TCP -- which latched
    // an unrecoverable force-control fault on 2026-08-25.
    //
    // This is a real detection-vs-robustness trade: a larger budget notices a
    // genuinely dead link later. It is exposed rather than hardcoded so the
    // choice is visible, and a stale read is warned about either way.
    double worker_state_max_age_periods = 4.0;

    // Use actual period for logging, but cap filter dt so one late tick does not
    // create an unexpectedly large joint step.
    double filter_dt_min_ratio = 0.5;
    double filter_dt_max_ratio = 1.5;

    // Final-stage moving average over the last N sent joint targets (applied
    // after the safety filter; convex combination keeps limits intact).
    // 0/1 disables. 40 at 500 Hz = 80 ms boxcar, ~40 ms group delay.
    int output_moving_average_window = 0;

    double servo_t1_rate_match_tolerance_ratio = 0.2;
    bool allow_servo_t1_rate_mismatch = false;

    RbpodoAsyncStreamingConfig rbpodo_async_streaming;
};

struct NetworkConfig {
    std::string command_bind = "udp://127.0.0.1:50010";
    std::string state_pub_endpoint = "udp://127.0.0.1:50110";
    // Deprecated compatibility alias. Keep synchronized with the first endpoint.
    std::string state_pub_bind = "udp://127.0.0.1:50110";
    // Canonical UDP state destinations. The first entry mirrors state_pub_endpoint.
    std::vector<std::string> state_pub_endpoints{"udp://127.0.0.1:50110"};
    std::vector<std::string> scope_pub_endpoints{"udp://127.0.0.1:50357"};
    int state_pub_rate_hz = 20;
    std::vector<std::string> command_source_allowlist{"127.0.0.1/32"};
    double command_timeout_sec = 0.2;
    bool command_source_enforce_lease = false;
    double command_source_lease_timeout_sec = 1.0;
    // Ruckig chunk-follower ingest: dedicated UDP bind for whole action-chunk
    // frames (the producer's chunk-overlay wire format, fanned out here in
    // addition to the GUI). DELIBERATELY separate from the lease-gated
    // command_bind so a high-rate per-tick command stream can never starve the
    // ~1-2 Hz chunk feed, and lease admission never drops telemetry-shaped
    // frames. Empty = receiver disabled.
    std::string chunk_frame_bind;
};

struct CommandSourceConfig {
    bool enforce_lease = false;
    double lease_timeout_sec = 1.0;
};

struct LoggingConfig {
    bool enable = true;
    std::string directory = "./logs";
    int flush_period_ms = 100;
    size_t queue_capacity = 4096;
};

// Rainbow control-box command-queue regulation. See
// rb_servo/control/queue_sync_controller.hpp for the plant and the provenance
// of every gain. Fail-closed: disabled by default, because enabling it changes
// the send cadence on a real-motion path.
//
// Gains are controller-manager's (Arm::qsync_step) used AS-IS: this server now
// uses the same actuator (a per-arm send-period trim), so they transfer
// directly. Retuning one side without the other is a mistake.
struct QueueSyncConfig {
    bool enable = false;
    int target_fill = 5;                    // occupancy setpoint == box dead time in ticks
    int protect_fill = 1;                   // at/below this, wake early hard (underrun)
    double lpf_alpha = 0.02;                // fill low-pass; ~0.1 s at 500 Hz
    double kp_above_us = 6.0;               // us of period trim per fill above target
    double kp_below_us = 18.0;              // per fill below target (underrun costs more)
    double ki_us = 0.006;                   // us per fill per cycle; learns the clock drift
    double integral_clamp_us = 25.0;
    double adj_clamp_us = 120.0;            // Track cap
    double protect_adj_us = -80.0;          // fill <= protect_fill: wake early NOW
    double drain_adj_us = 200.0;            // Drain floor (+10% period)
    double drain_max_us = 2000.0;           // Drain cap (+100% period = half rate)
    double drain_per_fill_us = 4.0;         // Drain proportional term
    int redrain_fill_margin = 15;           // Track -> Drain on a queue rebase
    int highwater_fill = 500;               // absurd backlog -> event
    double warmup_min_sec = 0.4;            // let the box startup transient develop
    double warmup_max_sec = 1.2;
    double drain_timeout_sec = 8.0;
    int stall_cycles = 25;                  // no fresh RBACK for this many ticks -> event
    int no_consumption_rise_per_sec = 100;  // fill rising this fast -> box stopped consuming
};

struct ScopeConfig {
    bool enable = false;
    int publish_rate_hz = 100;
    size_t max_samples_per_batch = 64;
};

struct FtWrenchPipelineConfig {
    // Runtime activation remains explicit and fail-closed. The rbpodo EFT
    // adapter does not provide an independent sensor-health channel, so real
    // use is experimental and must be accepted through site-local config.
    bool enable = false;
    bool frame_configured = false;
    std::string sensor_identity;
    std::string calibration_id;
    std::string freshness_source = "sequence";
    double max_sample_age_sec = 0.02;
    double max_source_stall_sec = 0.02;
    double control_lpf_alpha = 0.2;
    bool inertial_compensation_enable = false;
    double inertial_effective_mass_kg = 0.0;
    double inertial_accel_lpf_alpha = 0.0;
    double max_tcp_speed_m_s = 0.0;
    double max_tcp_accel_m_s2 = 0.0;
    bool auto_tare_after_init_motion = false;
    // MAXIMUM settle wait after Init Motion completes before the tare window is
    // collected. With auto_tare_settle_detect_enable (2026-08-19) collection starts
    // earlier, as soon as the tool has stopped ringing: the rolling per-axis stddev of
    // the pre-tare wrench over the last auto_tare_settle_window_samples fresh samples is
    // inside the residual_tare stddev limits, the arm's sent joint speed is below
    // auto_tare_settle_max_joint_speed_deg_s and at least auto_tare_settle_min_sec have
    // elapsed. A spoiled/never-quiet window still starts at the maximum (old behaviour).
    double auto_tare_settle_sec = 0.5;
    bool auto_tare_settle_detect_enable = true;
    double auto_tare_settle_min_sec = 0.5;
    int auto_tare_settle_window_samples = 150;
    double auto_tare_settle_max_joint_speed_deg_s = 1.0;
    // Software-zero reuse (2026-08-19). Every Init Motion invalidates the zero, so the arm
    // stayed unarmed for settle+collect (~4 s) after each press — long enough that
    // rollouts started unarmed and the acceptance (with its motion_epoch bump, since
    // removed) landed mid-motion. When the new Init Motion ends within
    // auto_tare_reuse_pose_tol_deg (per joint, measured) of where the last accepted zero
    // was captured and that zero is younger than auto_tare_reuse_max_age_sec, the
    // previous zero is re-validated at Init Motion completion (armed immediately) and a
    // fresh window is collected in the background to refresh it: the delta is logged, a
    // spoiled window / arm leaving the pose / contact keeps the previous zero. Payload
    // identification and freedrive still discard the capture.
    bool auto_tare_reuse_enable = true;
    double auto_tare_reuse_pose_tol_deg = 2.0;
    double auto_tare_reuse_max_age_sec = 600.0;
    int residual_tare_min_samples = 50;
    double residual_tare_max_force_stddev_n = 0.1;
    double residual_tare_max_torque_stddev_nm = 0.01;

    // T_tcp_sensor: pose of the sensor frame expressed in the TCP frame.
    Pose6D t_tcp_sensor;
    Wrench6D sensor_bias;
    // Orientation-dependent gravity wrench removed before the pose-local
    // residual tare.  The legacy rigid_payload model uses payload_mass_kg and
    // payload_com_tcp_m.  controller_compensated_linear is a separately
    // identified residual model for controller feedback that is assumed to
    // have already undergone undocumented gravity/source processing.
    std::string gravity_compensation_model = "rigid_payload";
    std::string gravity_compensation_calibration_id;
    std::array<double, 9> gravity_force_matrix_n_per_m_s2{};
    std::array<double, 9> gravity_torque_matrix_nm_per_m_s2{};
    bool gravity_force_matrix_configured = false;
    bool gravity_torque_matrix_configured = false;
    double payload_mass_kg = 0.0;
    std::array<double, 3> payload_com_tcp_m{};
    Wrench6D residual_tare_tcp;
};

struct PayloadIdentificationConfig {
    // Disabled unless the tracked stack explicitly supplies and validates the
    // complete acquisition/fit profile. Zero values are deliberately invalid
    // when enable=true; callers must never invent motion or acceptance bounds.
    bool enable = false;
    // rigid_payload identifies physical mass/CoG.  The linear controller-
    // compensated model identifies only an orientation-dependent residual and
    // must never be presented as a physical payload or CoG.
    std::string observation_model;
    // Observation sign for the rigid pre-payload/pre-tare wrench model.
    // Required for rigid_payload and deliberately empty for the linear model.
    // payload_load: w = bias + [m*g, c x (m*g)]
    // sensor_reaction: w = bias - [m*g, c x (m*g)]
    std::string wrench_convention;
    int min_poses = 0;
    double arrival_tolerance_deg = 0.0;
    double settle_sec = 0.0;
    int samples_per_pose = 0;
    double max_force_stddev_n = 0.0;
    double max_torque_stddev_nm = 0.0;
    double max_force_fit_rms_n = 0.0;
    double max_torque_fit_rms_nm = 0.0;
    double max_design_condition_number = 0.0;
};

struct ForceTorqueConfig {
    std::string source = "null";
    PayloadIdentificationConfig payload_identification;
    FtWrenchPipelineConfig left;
    FtWrenchPipelineConfig right;
};

struct ForceControlArmConfig {
    bool enable = false;
    // V1 supports one stand-frame normal: a server-owned plane, or a direction
    // captured from measured force at the start of a debounced contact episode.
    std::string surface_source = "floor_constraint";
    // contact_force episode ENTRY gate: an episode may only start while the
    // commanded TCP speed is at or below this. Real surface contact happens in
    // a decelerating approach; grip/inertial residuals during fast transit
    // (2026-07-18 15:23 runs: 3.5-9.5 N with rotating direction) must never
    // freeze a bogus normal. Zero is invalid for a motion-affecting
    // contact_force profile — the gate must be an explicit reviewed choice.
    double contact_entry_max_speed_m_s = 0.0;
    // Frame used by the symmetric 6D Cartesian admittance controller.
    // surface: selected stand-fixed surface axes at the TCP origin.
    // sensor_origin: URDF/configured sensor axes and measurement origin.
    // tcp_origin: sensor axes translated to the TCP origin.
    std::string compliance_frame = "surface";
    double target_force_n = 0.0;
    double contact_enter_force_n = 0.0;
    double contact_release_force_n = 0.0;
    double force_deadband_n = 0.0;
    double hard_normal_force_n = 0.0;
    double hard_force_norm_n = 0.0;
    double hard_torque_norm_nm = 0.0;
    // Optional resultant-force rise-rate hard-limit trigger. The rate is
    // evaluated only across fresh F/T acquisitions using their host sample
    // timestamps. Zero disables the trigger; a positive rate requires a
    // positive arming floor so near-zero noise cannot trip it.
    double hard_limit_rate_n_per_ms = 0.0;
    double hard_limit_rate_floor_n = 10.0;
    int debounce_samples = 1;
    int hard_limit_debounce_samples = 1;
    double release_dwell_sec = 0.0;
    double release_velocity_threshold_m_s = 0.002;
    // Soft-contact hysteresis for the five non-normal Cartesian compliance
    // axes.  The known surface normal keeps using the scalar thresholds above.
    double transverse_contact_enter_force_n = 5.0;
    double transverse_contact_release_force_n = 4.0;
    double torque_contact_enter_nm = 0.9;
    double torque_contact_release_nm = 0.7;
    ForceControlAxis compliance_axes;
};

struct NormalAdmittanceConfig {
    double virtual_mass_kg = 5.0;
    double damping_n_s_m = 80.0;
    double stiffness_n_m = 0.0;
    double max_unload_offset_m = 0.01;
    double max_normal_velocity_m_s = 0.02;
    double max_normal_acceleration_m_s2 = 0.2;
    double max_normal_jerk_m_s3 = 2.0;
    double max_normal_step_m = 0.001;
    double max_energy_j = 2.0;
};

struct ForceControlConfig {
    std::string provider = "null";
    bool enable = false;
    std::string operating_mode = "monitor";
    bool allow_in_real = false;
    // rbpodo EFT does not expose an independent sensor presence/fault/
    // overrange channel. This explicit opt-in prevents a generic real-motion
    // allow flag from silently treating controller-frame freshness as a
    // safety-rated sensor contract.
    bool supervised_experimental_real = false;
    int update_rate_hz = 500;

    ForceControlArmConfig left;
    ForceControlArmConfig right;
    NormalAdmittanceConfig normal_admittance;

    // Layer-3 generic force limiter (task-agnostic back-off). Above
    // force_limit_n the TRANSLATION response envelope opens proportionally to
    // the excess force so the arm actively escapes along the measured force
    // direction — no pre-defined normal, no episode state: the per-tick
    // deadband-filtered wrench IS the direction. force_limit_n <= 0 disables
    // the layer (plain bounded compliance). When enabled, validation requires
    // a positive gain and backoff_max_velocity_m_s >= max_linear_velocity_m_s.
    double force_limit_n = 0.0;
    double backoff_gain_m_s_per_n = 0.0;
    double backoff_max_velocity_m_s = 0.0;

    // Hard-limit policy. "latch" (default): the debounced hard force/torque
    // limit faults and freezes motion. "retreat": instead of latching, run a
    // bounded retreat episode — a virtual wrench of retreat_virtual_force_n
    // along the measured press direction drives the admittance offset away
    // from the contact until the TCP has escaped retreat_distance_m and the
    // instantaneous force is back under the hard threshold; policy streaming
    // and inference continue throughout (no fault). The latch remains the
    // fail-closed backstop: it fires when the retreat cannot unload within
    // retreat_timeout_sec, when no press direction is measurable, when the
    // scalar contact_force episode owns the normal, or when episodes trigger
    // more than retreat_max_attempts times per retreat_attempt_window_sec
    // (0 attempts = unlimited retreats). Requires operating_mode
    // cartesian_admittance.
    std::string hard_limit_policy = "latch";
    // Retreat travel CAP, not a target: the escape brakes at whichever of
    // retreat_release_force_n / this distance / the offset-budget guards comes
    // first. Config validation enforces retreat_distance_m < max_pos_offset_m,
    // because when the two were equal (both 30 mm, 2026-07-24 -> 07-31) the
    // 0.8*max_pos_offset braking guard at 24 mm ALWAYS fired first and the
    // distance condition became dead code.
    double retreat_distance_m = 0.010;
    // Force-terminated retreat: brake once the measured external force falls to
    // this level. 0 disables it (distance/budget termination only).
    //
    // Distance alone is open-loop on the wrong quantity. Measured 2026-07-31
    // (left arm, servo_log_20260731_153934): when the compliance offset started
    // near zero the arm unloaded to 1.3-2.2 N after only 12-16 mm of escape, but
    // once repeated contacts had ratcheted the offset to 24-29 mm there was no
    // room left, the escape ended with 6.0-8.8 N still applied, and the next
    // press stacked on top -- a 24 N plateau across 19 retreat episodes. Braking
    // on the force instead of on a fixed distance ends the escape as soon as the
    // objective is met and keeps escaping while it is not.
    double retreat_release_force_n = 0.0;
    double retreat_virtual_force_n = 20.0;
    double retreat_timeout_sec = 1.0;
    int retreat_max_attempts = 0;
    double retreat_attempt_window_sec = 10.0;

    // Diagonal Cartesian admittance parameters ordered [x,y,z,rx,ry,rz].
    std::array<double, 6> virtual_mass{5.0, 5.0, 5.0, 0.5, 0.5, 0.5};
    std::array<double, 6> damping{80.0, 80.0, 80.0, 8.0, 8.0, 8.0};
    std::array<double, 6> stiffness{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // Released-state offset bleed for zero-stiffness axes. With stiffness 0 the
    // protective admittance offset never drains on its own; a policy that
    // hovers at the contact and re-approaches (instead of moving away) ratchets
    // the offset up to max_pos_offset_m across repeated contacts, after which
    // the unload authority is gone and the next press runs straight into the
    // hard force limit (2026-07-22 servo_log_20260722_172914: three floor
    // contacts, offset -7 -> -22 -> -30 mm(cap), peaks 6.6 -> 15.6 -> 31.6 N
    // latch). While an axis BLOCK is fully released (every enabled axis inside
    // the wrench deadband), a zero-stiffness axis uses this spring value so the
    // residual offset decays toward the equilibrium (tau ~= damping/bleed).
    // Any re-contact re-loads the block and immediately stops the bleed, so
    // the in-contact spring re-press that motivated stiffness 0 stays
    // impossible. 0 disables the bleed (offset rests where contact put it).
    std::array<double, 6> release_bleed_stiffness{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<double, 6> wrench_deadband{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // When enabled, translation and rotation each defer per-axis spring
    // recentering until every enabled axis in that block is released.  The
    // released block then shares one feasible jerk scale so its return
    // direction is preserved instead of being reshaped by six independent
    // jerk clamps.
    bool blockwise_release_recenter = false;

    double max_dt_sec = 0.02;

    double max_pos_offset_m = 0.01;
    double max_rot_offset_rad = 0.1;
    double max_linear_velocity_m_s = 0.02;
    double max_angular_velocity_rad_s = 0.2;
    double max_linear_acceleration_m_s2 = 0.2;
    double max_angular_acceleration_rad_s2 = 2.0;
    double max_linear_jerk_m_s3 = 2.0;
    double max_angular_jerk_rad_s3 = 20.0;
    double max_pos_step_m = 0.001;
    double max_rot_step_rad = 0.01;
    double max_energy_j = 2.0;

};

struct LinearMoveConfig {
    double min_duration_sec = 0.05;
    double max_duration_sec = 10.0;
    double default_linear_speed_m_s = 0.03;
    double default_angular_speed_rad_s = 0.2;
    double constant_orientation_tolerance_rad = 0.005;
    LinearMoveOrientationMode default_orientation_mode = LinearMoveOrientationMode::Constant;
    // Collision-free MoveL (requires safety.init_motion_planner.enable — reuses its
    // planner + private collision/floor oracle + a private IK). When true, a
    // TcpLinearMove first checks whether the straight Cartesian path is collision- and
    // floor-clear: if so it runs the exact straight MoveL (orientation constant/slerp
    // preserved); if the straight path would collide it falls back to a collision-free
    // joint-space detour (RRT-Connect) to the IK'd target and streams that, so the move
    // still reaches the target without self-collision or crossing a safety plane.
    // Default off (strict straight MoveL, guarded only by the reactive barrier).
    bool collision_free = false;
    int collision_check_samples = 40;  // dense samples along the straight path for the precheck
};

// Spring-Mass-Damper smoothing for the streaming TcpPoseTarget path: received
// command deltas integrate into a goal pose, and the published target follows
// that goal as a second-order system (mass fixed at 1.0) stepped at the servo
// rate. Translation and rotation are tunable independently.
struct PoseTrackSmdConfig {
    bool enable = false;
    double damping_ratio_linear = 1.0;
    double natural_frequency_linear_hz = 0.5;
    double damping_ratio_angular = 1.0;
    double natural_frequency_angular_hz = 0.5;
    // Saturation clamps applied as vector-norm limits each tick. Mass is fixed
    // at 1.0, so the accel clamp IS the max-force clamp. Inside the limits the
    // zeta/fn dynamics are exactly preserved. 0 = unlimited.
    double max_linear_velocity_m_s = 0.0;
    double max_linear_accel_m_s2 = 0.0;
    double max_angular_velocity_rad_s = 0.0;
    double max_angular_accel_rad_s2 = 0.0;
    // Re-engagement re-latch guard. If > 0, a single updateGoalFromCommand delta
    // larger than this (position m / rotation rad) is treated as a re-anchor after a
    // teleop disengage gap — the command buffer holds the last TcpPoseTarget across
    // the deadman-release gap, keeping the tracker active, then the source re-anchors
    // to the live pose, so the first post-gap delta is the whole accumulated lead.
    // Integrating it would lurch the arm on the next pedal/deadman press; instead we
    // RE-LATCH the reference (goal unchanged). Real per-tick teleop deltas are ~1 mm
    // at 500 Hz (source-capped well below this), so a healthy stream never trips it.
    // 0 = disabled (legacy: integrate any delta — required for model-rollout / test
    // profiles that step the goal by large synthetic deltas).
    double reengage_relatch_max_step_m = 0.0;
    double reengage_relatch_max_step_rad = 0.0;
    // Velocity feedforward: damp on the velocity ERROR (goal_dot - x_dot) rather
    // than the absolute x_dot, so the error dynamics become
    //   e_ddot + 2*zeta*wn*e_dot + wn^2*e = goal_ddot.
    // A constant-velocity (ramp) goal then has ZERO steady-state lag, while jitter
    // (the goal_ddot term) still rolls off through the 2nd-order response. This
    // decouples natural_frequency from tracking accuracy: fn becomes a pure
    // smoothing dial that can be lowered for more smoothing without adding lag.
    // The goal velocity is estimated internally from the per-tick goal delta
    // (auto stand/body frame; valid because every caller integrates the goal once
    // per step()). Off by default = exact legacy 2nd-order SMD.
    bool velocity_feedforward = false;
    // Singularity velocity scaling (manipulability guard). As the last IK solve's
    // task-Jacobian min singular value drops, scale the SMD's max tracking velocity
    // down so a near-singular pose is approached GENTLY (the arm physically cannot
    // track Cartesian fast there) instead of lurching, and the operator feels the
    // slow-down and backs out. scale = 1 at sigma >= full_sigma, ramps linearly to
    // scale_min at sigma <= floor_sigma (kept > 0 so motion never fully freezes and
    // the operator can always command back out). Touches ONLY the velocity caps —
    // NOT IK damping/iterations — so it cannot cause IK max_iterations. full_sigma
    // <= 0 disables (default off; preserves every non-UMI profile).
    double singularity_scale_full_sigma = 0.0;
    double singularity_scale_floor_sigma = 0.0;
    double singularity_scale_min = 1.0;
};

enum class RuckigFollowerFallbackPolicy {
    Smd,
    Fault
};

// What a sustained delta_preview projection-budget breach does. Fault latches
// (bring-up default); Warn logs and keeps following — the projection gate is a
// plan-fidelity alarm, while lead / the divergence latch / ROI / self-collision
// / FT hard limits own runaway safety.
enum class RuckigProjectionFaultPolicy {
    Fault,
    Warn
};

enum class RuckigFollowerController {
    RuckigWaypoint,
    DeltaTwist,
    DeltaPreview
};

// Continuous output conditioning for the Cartesian chunk follower. This is a
// pure post-follower stage: chunk chaining, projection, divergence, and actual-
// lead accounting continue to use the unfiltered follower state.
struct FollowerOutputSmdConfig {
    bool enable = false;
    double nf_linear_hz = 3.5;
    double nf_angular_hz = 2.5;
    double damping_ratio = 1.0;
    bool velocity_ff = true;
    // 0 follows the natural frequency of each domain independently.
    double velocity_ff_lpf_hz = 0.0;
};

// Per-profile chunk-follower stage that REPLACES the pose_track_smd step while
// active. The default controller consumes measured-anchored absolute waypoint
// rows through the Ruckig receding-horizon follower; delta_twist consumes
// conditioned local per-frame model action deltas directly.
struct RuckigFollowerConfig {
    bool enable = false;
    // Default preserves the absolute-waypoint follower. delta_twist consumes
    // conditioned local per-frame action deltas from ChunkFrame::delta.
    RuckigFollowerController controller = RuckigFollowerController::RuckigWaypoint;
    // Fallback policy for follower-regime interruptions:
    // - Smd: legacy behavior; cold start / feed timeout / divergence quietly
    //   fall back to pose_track_smd.
    // - Fault: strict behavior once the follower regime applies (enable &&
    //   TcpPoseTarget && has_tcp_target && kinematics && chunk receiver). Cold
    //   start holds at the live FK reference and latches if no first chunk
    //   engages within engage_timeout_sec; post-engage feed timeout or
    //   divergence latches ChunkFollowerFault instead of falling back to SMD.
    //   Mode changes (Hold / JointTarget / InitMotion / TcpLinearMove /
    //   deadman-stale) remain normal deactivations under both policies.
    RuckigFollowerFallbackPolicy fallback_policy = RuckigFollowerFallbackPolicy::Smd;
    double engage_timeout_sec = 3.0;
    // Fixed conservative Cartesian limits (per-axis). The joint-space safety
    // clamp downstream remains the sole hard guarantor; these caps only shape
    // the generated reference. At 33 ms segments the JERK cap is the binding
    // convergence budget (|dv| <= j*dt^2/4), not accel.
    double max_linear_velocity_m_s = 0.6;
    double max_linear_accel_m_s2 = 3.0;
    double max_linear_jerk_m_s3 = 20.0;
    double max_angular_velocity_rad_s = 1.5;
    double max_angular_accel_rad_s2 = 8.0;
    double max_angular_jerk_rad_s3 = 40.0;
    // Chunk windowing. The producer re-anchors each chunk to the measured TCP
    // at activation, so inference latency is already absorbed -> discard_head
    // defaults 0 (a nonzero value would double-compensate). consume_steps is a
    // preempt preference; the window clamps it to horizon - discard - reserve,
    // and the follower keeps consuming into the reserve tail (decelerating into
    // the final waypoint) when the next chunk is late, instead of stalling.
    int discard_head_steps = 0;
    int consume_steps = 16;
    int reserve_steps = 1;             // central-difference lookahead (>= 1)
    int smoothing_window = 3;          // odd; pre-difference chunk smoothing
    FollowerOutputSmdConfig output_smd;
    // Feedforward accel damping, (0, 1], split by axis class 2026-07-31 because the two classes
    // sit at 31% and 95% of their acceleration limits -- see control::GuardConfig for the
    // measurement and why the single scalar could not be tuned. The legacy scalar
    // `af_damping_beta` key is still accepted and seeds BOTH (existing configs keep their
    // behavior); the per-class keys override it.
    double af_damping_beta_lin = 0.85;
    double af_damping_beta_ang = 0.85;
    // DeltaTwistFollower params. The model delta is a per-policy-frame local
    // displacement, not m/s; these tune how that backlog is drained into an
    // internally jerk-limited body twist.
    double delta_twist_tau_sec = 0.015;
    int delta_twist_residual_drain_steps = 1;
    bool delta_twist_clear_residual_on_new_frame = true;
    double delta_twist_min_time_to_go_sec = 0.015;
    double delta_twist_max_residual_m = 0.030;
    double delta_twist_max_residual_rad = 0.35;
    double delta_twist_max_lead_m = 0.060;
    double delta_twist_max_lead_rad = 0.30;
    double delta_twist_stale_residual_timeout_sec = 0.15;
    // Corner (direction-reversal) ring-down guard of the chunk follower. A flanking
    // step below the deadband contributes sign 0 and cannot form a reversal pair, so
    // a LARGER deadband means the guard ignores small wobble. Size them against the
    // real per-step displacement: at 30 Hz the policy commands ~2 mm / ~0.3 deg per
    // step, and 0.3 deg split across three rotation axes leaves per-axis components
    // only a few times the 0.029 deg default -- rotational sign noise then trips the
    // guard on a large fraction of segments. Because ruckig time-synchronizes the
    // segment, one braked rotation axis stretches the whole 6-DoF duration and slows
    // translation too. Defaults reproduce the previously hard-coded values exactly.
    double corner_deadband_lin_m = 3e-4;
    double corner_deadband_ang_rad = 5e-4;
    // Target-velocity ring-down for a reversing axis (target acceleration is always
    // zeroed). 1.0 disables the velocity cut and keeps only the acceleration reset.
    double corner_velocity_scale = 0.25;
    // delta_preview safety contract. Zero means unspecified and is rejected
    // whenever controller=delta_preview; no motion-relevant fallback exists.
    double preview_max_projection_error_m = 0.0;
    double preview_max_projection_error_rad = 0.0;
    int preview_max_consecutive_projection_errors = 0;
    double preview_max_actual_lead_m = 0.0;
    double preview_max_actual_lead_rad = 0.0;
    int preview_max_consecutive_actual_lead_errors = 0;
    RuckigProjectionFaultPolicy preview_projection_fault_policy =
        RuckigProjectionFaultPolicy::Fault;
    // Quasi-static gate for the wrench-gated loading projection: the follower
    // only projects contact loading out of the plan while its own linear plan
    // acceleration is below this bound. Fast transit acceleration puts a real
    // m*a inertial component (up to ~10 N for the 3.5 kg tool) into the
    // measured wrench that the gravity map cannot remove, and a projection
    // firing on that inertial "contact" yanks the plan. Exceeding the bound
    // fails toward the baseline blind follower (assist off, motion untouched).
    double loading_projection_max_accel_m_s2 = 0.5;
    // Brief upstream Hold interleaves may preserve the active chunk and its
    // chained p/v/a state. Zero is deliberately invalid for an enabled
    // follower: every active profile must select the reviewed grace window.
    double hold_bounce_resume_sec = 0.0;
    // Feed-liveness watchdog: with no fresh chunk frame for this long the
    // follower deactivates (falls back to pose_track_smd / hold).
    double chunk_feed_timeout_sec = 1.5;
};

struct TcpPoseTargetProfileConfig {
    std::string name = "default";
    PoseTrackSmdConfig pose_track_smd;
    RuckigFollowerConfig ruckig_follower;
    // Server-side backlog cap for future enforcement/telemetry. A value <= 0
    // leaves clamping disabled; source-side lead clamps remain the primary
    // UMI control-quality limiter.
    double max_smd_goal_lead_m = 0.0;
    double max_smd_goal_lead_rad = 0.0;
};

enum class CartesianLimitPolicy {
    Clamp,
    Reject
};

enum class CartesianControllerSimulationStateSource {
    Actual,
    Reference
};

struct CartesianControlConfig {
    bool enable = true;
    bool allow_in_simulation = true;
    bool allow_in_real = false;
    bool allow_in_controller_simulation = false;
    double warn_ik_duration_us = 3000.0;
    double fail_ik_duration_us = 0.0;
    // Deprecated compatibility field. New control code uses path_kp_pos/path_kp_ori.
    double path_kp = 6.0;
    double path_kp_pos = 6.0;
    double path_kp_ori = 6.0;
    double velocity_damping = 0.01;
    double max_linear_move_speed_m_s = 0.05;
    double max_angular_move_speed_rad_s = 0.3;
    std::optional<double> max_cartesian_step_m;
    std::optional<double> max_cartesian_step_rad;
    CartesianLimitPolicy exceed_limit_policy = CartesianLimitPolicy::Clamp;
    CartesianControllerSimulationStateSource controller_simulation_servo_state_source =
        CartesianControllerSimulationStateSource::Actual;
    CartesianControllerSimulationStateSource controller_simulation_divergence_source =
        CartesianControllerSimulationStateSource::Actual;
    LinearMoveConfig linear_move;
    PoseTrackSmdConfig pose_track_smd;
    // Base ruckig_follower defaults copied into each profile (same pattern as
    // pose_track_smd): a profile without its own ruckig_follower block inherits
    // this one.
    RuckigFollowerConfig ruckig_follower;
    std::string tcp_pose_target_profile_default = "default";
    std::vector<TcpPoseTargetProfileConfig> tcp_pose_target_profiles;
};

// Bridge to the out-of-process gripper_server (docs/plans/gripper_server_design.md).
// When enabled, the server forwards the arbitrated per-arm gripper setpoint
// (left/right.gripper from the command packet) to gripper_server as gripper_cmd.v1
// and stamps the gripper_state.v1 feedback into the published state JSON. The
// gripper is NOT a motion-safety constraint; this only routes setpoints/feedback.
struct GripperConfig {
    bool enable = false;
    std::string command_endpoint = "udp://127.0.0.1:50410";  // -> gripper_server
    std::string feedback_bind = "udp://127.0.0.1:50420";     // <- gripper_server
    int forward_rate_hz = 50;
    double feedback_stale_timeout_ms = 500.0;
};

struct DualArmConfig {
    BackendConfig left_robot;
    BackendConfig right_robot;

    ArmMountConfig left_mount;
    ArmMountConfig right_mount;

    ServoConfig servo;
    SafetyConfig safety;
    NetworkConfig network;
    CommandSourceConfig command_source;
    LoggingConfig logging;
    ScopeConfig scope;
    QueueSyncConfig queue_sync;
    ForceTorqueConfig force_torque;
    ForceControlConfig force_control;
    CartesianControlConfig cartesian_control;
    KinematicsConfig kinematics;
    GripperConfig gripper;
};

DualArmConfig loadConfigFromYaml(const std::string& path);

}  // namespace rb_servo
