// Scratch bench #3: reproduce the BRANCH-JUMP RATE-LIMIT RELEASE transient.
//
// Measured 2026-08-28 (servo_log_20260828_135443.csv, left arm): the solver spends runs
// of ticks with iters==100 and branch_jump_rate_limited==1, its step scaled to ~14% of
// what IK asked for; then the solve converges in ONE iteration, the limiter releases in
// the SAME tick, and the joint reverses direction between two 2 ms samples.
//     |commanded accel| median  4,482 deg/s^2 at those transitions
//                       median      83 deg/s^2 everywhere else   (54x)
//                          max 184,224 deg/s^2  = 122x ddq_max
// The limiter is STATELESS: each tick independently decides on/off, so the release is a
// step in the transfer function, not a ramp.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include "rb_servo/config/config.hpp"
#include "rb_servo/kinematics/pinocchio_kinematics.hpp"
#include "rb_servo/control/safety_filter.hpp"

using namespace rb_servo;

static KinematicsConfig stackRealIk(const std::string& urdf) {
    KinematicsConfig c;
    c.enable = true; c.provider = "pinocchio"; c.urdf = urdf;
    c.base_link = "world"; c.tip_link = "tcp";
    c.joint_names = {"base_joint","shoulder_joint","elbow_joint","wrist1_joint","wrist2_joint","wrist3_joint"};
    c.q_units = "deg"; c.publish_tcp = true;
    c.ik.enable = true; c.ik.min_iterations = 1; c.ik.max_iterations = 100;
    c.ik.timeout_ms = 20.0; c.ik.damping = 0.02;
    c.ik.position_tolerance_m = 0.00002; c.ik.orientation_tolerance_rad = 0.0002;
    c.ik.max_step_deg = {2,2,2,3,3,4};
    c.ik.singular_region_eps = 0.10; c.ik.damping_max = 0.08;
    c.ik.max_solution_jump_deg = 1.0;
    c.ik.branch_jump_damping_scale = 10.0; c.ik.branch_jump_max_retries = 2;
    c.ik.branch_jump_rate_limit = true;
    c.ik.singular_step_scale_full_sigma = 0.17;
    c.ik.singular_step_scale_floor_sigma = 0.11;
    c.ik.singular_step_scale_min = 0.30;
    c.ik.joint_limit_track_feasible = true;
    c.ik.joint_limit_best_effort_position_tolerance_m = 0.0005;
    c.ik.joint_limit_best_effort_orientation_tolerance_rad = 0.002;
    c.ik.max_iterations_best_effort_position_tolerance_m = 0.0005;
    c.ik.max_iterations_best_effort_orientation_tolerance_rad = 0.002;
    return c;
}

static SafetyConfig stackRealSafety() {
    SafetyConfig s;
    s.q_min_deg = {-360,-360,-150,-360,-360,-360};
    s.q_max_deg = { 360, 360, 150, 360, 360, 360};
    s.dq_max_deg_s   = {170,170,170,240,240,320};
    s.ddq_max_deg_s2 = {1500,1500,1500,2300,2300,3000};
    s.ddq_max_decel_ratio = 4.0;
    s.decel_overshoot_budget_deg = 0.5;
    s.joint_limit_barrier.enable = true;
    s.joint_limit_barrier.q_min_deg = {-360,-360,-150,-360,-360,-360};
    s.joint_limit_barrier.q_max_deg = { 360, 360, 150, 360, 360, 360};
    s.joint_limit_barrier.d_slow_deg     = {12,12,12,16,16,22};
    s.joint_limit_barrier.a_brake_deg_s2 = {1500,1500,1500,2300,2300,3000};
    s.joint_limit_barrier.standoff_deg   = {0.10,0.10,0.10,0.10,0.10,0.10};
    return s;
}

int main(int argc, char** argv) {
    const std::string urdf = argv[1];
    const double SWEEP_HZ = argc > 2 ? std::atof(argv[2]) : 0.5;
    const double NOISE_MM = argc > 3 ? std::atof(argv[3]) : 0.5;
    const double SPEED_MM = argc > 4 ? std::atof(argv[4]) : 269.0;
    ArmMountConfig mount; mount.arm_id = ArmId::Left;
    mount.base_pose_in_stand = {0.15707,-0.17036,0.58036, 2.186649,0.523831,2.526296};
    // MEASURED conditions of the longest sustained rate-limit run on hardware
    // (servo_log_20260828_135443.csv, t=52.540..52.838, 150 ticks):
    //   seed posture below, sigma 0.1135..0.1203 (the singular_step_scale floor is 0.11),
    //   the IK wanting 4.233 deg/tick median while the ceiling is 0.350 deg, and the
    //   POLICY TARGET sweeping at 269 mm/s median / 661 mm/s peak.
    // The trigger is a fast Cartesian command through an ill-conditioned pose -- not
    // noise. Bench #1 and #2 both used noise and produced per-tick chatter instead of
    // the 53-tick engaged runs the hardware shows.
    const JointArray start{238.382418, 6.978305, 146.804746,
                           -51.180536, -81.037023, -102.756718};
    KinematicsConfig cfg = stackRealIk(urdf);
    PinocchioKinematics k(cfg);
    const Pose6D p0 = k.computeTcpStand(ArmId::Left, start, mount);
    const int N = 3000; const double dt = 0.002;
    std::vector<double> noise(3*N+3);
    { unsigned s=99991u; for (auto& v:noise){ s=s*1664525u+1013904223u; v=((s>>8)/8388608.0)-1.0; } }

    // THE CLAMP CHAIN THE HARDWARE RUNS. Without it the bench measures the raw IK
    // output (median accel ~60,000 deg/s^2) instead of the SENT command (median 83), and
    // the whole contrast the bench exists to measure disappears. The correlation study
    // on servo_log_20260828_135443.csv named the acceleration clamp as the STRONGEST
    // predictor of roughness (+0.534), so it is not optional scenery here.
    const SafetyFilter safety(stackRealSafety());
    JointArray seed = start, prev = start, prevprev = start;
    std::vector<std::array<double,6>> q;
    std::vector<int> rl, it;
    for (int i=0;i<N;++i) {
        // A slow continuous sweep, like the measured transit (the arm swept J1 31->42
        // deg over ~1 s while J2 went 137->125). A square wave makes EVERYTHING violent
        // and destroys the contrast the bench exists to measure -- the hardware sees
        // 0.39 releases/s, not 100.
        // Bursts of fast Cartesian motion (SPEED mm/s) separated by rest -- the arm
        // lags, the limiter engages for a run of ticks, then releases when the burst
        // ends. Period chosen so releases land near the measured 0.39/s.
        const double cycle = std::fmod(i*dt, 1.0/SWEEP_HZ) * SWEEP_HZ;   // 0..1
        const double moving = cycle < 0.30 ? 1.0 : 0.0;
        static double travelled = 0.0;
        travelled += moving * (SPEED_MM*1e-3) * dt;
        if (cycle >= 0.30 && cycle < 0.32) travelled = 0.0;
        Pose6D t = p0;
        t.x = p0.x + travelled + (NOISE_MM*1e-3)*noise[3*i+0];
        t.y = p0.y + 0.6*travelled + (NOISE_MM*1e-3)*noise[3*i+1];
        t.z = p0.z - 0.4*travelled + (NOISE_MM*1e-3)*noise[3*i+2];
        IkResult r = k.solveIk(ArmId::Left, t, seed, mount);
        const JointArray sent = safety.clampMotion(r.q_solution_deg, prev, prevprev, dt);
        std::array<double,6> out{};
        for (int j=0;j<6;++j) out[j]=sent[j];
        q.push_back(out); rl.push_back(r.branch_jump_rate_limited?1:0); it.push_back(r.iterations);
        prevprev = prev; prev = sent;
        seed = sent;      // the loop seeds IK from the PREVIOUS SENT joints

    }
    // per-tick |acceleration| of the command, and where the limiter released
    std::vector<double> acc(N,0.0);
    for (int i=2;i<N;++i) {
        double m=0;
        for (int j=0;j<6;++j) m=std::max(m,std::fabs((q[i][j]-2*q[i-1][j]+q[i-2][j])/(dt*dt)));
        acc[i]=m;
    }
    std::vector<double> at, other;
    int releases=0;
    for (int i=2;i<N;++i) {
        const bool rel = (rl[i-1]==1 && rl[i]==0) || (it[i-1]>=100 && it[i]<100);
        if (rel) { ++releases; at.push_back(acc[i]); } else other.push_back(acc[i]);
    }
    auto med=[](std::vector<double> v){ if(v.empty())return 0.0; std::sort(v.begin(),v.end()); return v[v.size()/2]; };
    auto mx =[](const std::vector<double>& v){ return v.empty()?0.0:*std::max_element(v.begin(),v.end()); };
    int limited=0; for(int x:rl) limited+=x;
    printf("burst %.2f Hz speed %.0f mm/s noise %.1f mm | ", SWEEP_HZ, SPEED_MM, NOISE_MM);
    printf("rate-limited %d/%d ticks (%.0f%%)   releases %d (%.2f/s)\n",
           limited,N,100.0*limited/N,releases,releases/(N*dt));
    printf("|commanded accel| deg/s^2:  AT release  median %8.0f  max %10.0f   (n=%zu)\n", med(at), mx(at), at.size());
    printf("                            elsewhere   median %8.0f  max %10.0f   (n=%zu)\n", med(other), mx(other), other.size());
    printf("ratio of medians: %.1fx   (hardware measured 54x, max 184224)\n",
           med(other)>0? med(at)/med(other) : 0.0);
    return 0;
}
