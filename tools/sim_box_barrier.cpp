// Faithful barrier simulation: drive the LEFT gripper straight DOWN into a solid
// external keep-out box through the REAL CollisionMonitor + velocity-damper
// (applyCollisionVelocityProjection), and report how deep it penetrates. Run twice —
// once with the OLD floor barrier set (d_slow=5mm) and once with the NEW box set
// (d_slow=80mm) — to show the fix stops the arm at the box surface instead of ~40mm in.
#include <cstdio>
#include <string>
#include <array>
#include <vector>
#include <algorithm>
#include <cmath>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include "rb_servo/control/collision_monitor.hpp"
using namespace rb_servo;
namespace pin = pinocchio;
static const std::string WS = "/home/plaif/workspace";

static CollisionMonitorConfig baseCfg() {
    CollisionMonitorConfig c;
    c.enable = true;
    const std::string ud = WS + "/mo_robot_descriptions/mo_robot_descriptions/robots/urdf/dual_rb3_730e";
    c.unified_urdf = ud + "/dual_rb3_730e_ver5.urdf";
    c.package_dirs = {ud};
    const std::string tool = WS + "/robotics_lab/rb_servo_server/descriptions/meshes/robots/rb5_850e/visual/tool/";
    c.pika_gripper_base_mesh = tool + "pika_gripper_base_hull.STL";
    c.pika_finger_left_mesh = tool + "pika_finger_left_hull.STL";
    c.pika_finger_right_mesh = tool + "pika_finger_right_hull.STL";
    c.stand_ignore_arm_substrings = {"link0"};
    c.swept_samples = 2;
    c.max_near_pairs = 8;
    const char* jn[kDof] = {"base","shoulder","elbow","wrist1","wrist2","wrist3"};
    for (int i=0;i<kDof;i++){ c.left_joints[i]=std::string("dual_rb3_730e_left_")+jn[i]+"_joint";
                             c.right_joints[i]=std::string("dual_rb3_730e_right_")+jn[i]+"_joint"; }
    c.external_boxes.enable = true;
    c.external_boxes.max_count = 2;
    c.external_boxes.size_m = {0.380,0.240,0.105};
    c.external_boxes.margin_m = {0.000, 0.000, 0.040};
    c.external_boxes.monitor_only = false;
    c.external_boxes.stale_timeout_s = 1e12;  // harness: never treat the box feed as stale
    return c;
}

// FK: left attachment_site (gripper) position in STAND frame + d(z_stand)/dq for the 6
// left joints (finite diff), to build a pure "lower the gripper" joint velocity.
struct Fk { pin::Model model; pin::Data data; std::array<int,6> lq; int fid, sid;
    Fk(const std::string& urdf){ pin::urdf::buildModel(urdf, model); data=pin::Data(model);
        const char* jn[6]={"base","shoulder","elbow","wrist1","wrist2","wrist3"};
        for(int i=0;i<6;i++) lq[i]=model.joints[model.getJointId(std::string("dual_rb3_730e_left_")+jn[i]+"_joint")].idx_q();
        fid=model.getFrameId("dual_rb3_730e_left_attachment_site"); sid=model.getFrameId("stand"); }
    Eigen::Vector3d attachInStand(const std::array<double,6>& ldeg, const std::array<double,6>& rdeg){
        Eigen::VectorXd q=pin::neutral(model); const double D=M_PI/180.0;
        const char* jr[6]={"base","shoulder","elbow","wrist1","wrist2","wrist3"};
        for(int i=0;i<6;i++){ q[lq[i]]=ldeg[i]*D;
            q[model.joints[model.getJointId(std::string("dual_rb3_730e_right_")+jr[i]+"_joint")].idx_q()]=rdeg[i]*D; }
        pin::forwardKinematics(model,data,q); pin::updateFramePlacements(model,data);
        return data.oMf[sid].actInv(data.oMf[fid].translation()); }
};

int main() {
    Fk fk(baseCfg().unified_urdf);
    std::array<double,6> L={315.053,77.7532,97.5803,37.4993,-122.826,-115.576};
    std::array<double,6> R={-253.709,-63.0735,-93.8389,79.992,90.8985,123.119};
    const Eigen::Vector3d g0 = fk.attachInStand(L,R);
    // Joint velocity that lowers the gripper (stand -z): -gradient of z_stand wrt left q.
    std::array<double,6> grad{};
    for(int i=0;i<6;i++){ auto Lp=L; Lp[i]+=0.5; double zp=fk.attachInStand(Lp,R).z();
                          auto Lm=L; Lm[i]-=0.5; double zm=fk.attachInStand(Lm,R).z();
                          grad[i]=(zp-zm)/(2*0.5); }  // dz/ddeg
    double gn=0; for(double v:grad) gn+=v*v; gn=std::sqrt(gn);
    std::array<double,6> dir; for(int i=0;i<6;i++) dir[i]=-grad[i]/gn;  // deg-direction, lowers z

    auto run=[&](bool use_box_params, double approach_mps, int lag, bool trace=false){
        CollisionMonitorConfig c = baseCfg();
        if(!use_box_params){ // OLD: boxes reuse the floor's external_* set
            c.external_box_d_hard_m=0.0025; c.external_box_d_slow_m=0.005;
            c.external_box_a_brake_m_s2=3.0; c.external_box_recover_speed_m_s=0.0; c.external_box_latency_s=0.003;
        } // else: keep the header keep-out defaults (d_slow=0.080, a_brake=6, recover=0.03)
        CollisionMonitor mon(c);
        // Auto-place the box directly under the gripper so the START clearance (gripper
        // LOWEST point to box top) is +60mm, regardless of gripper reach/orientation.
        // Binary-search the box center z (lower center = box further down = bigger clearance).
        ExternalBoxPose bp; bp.enable=true; bp.R=Eigen::Matrix3d::Identity();
        bp.t=Eigen::Vector3d(g0.x(), g0.y(), 0.0);
        JointArray l0,r0; for(int i=0;i<6;i++){ l0[i]=L[i]; r0[i]=R[i]; }
        auto clrAt=[&](double cz){ bp.t.z()=cz; mon.setExternalBoxes({bp,ExternalBoxPose{}},0.0);
            return mon.evalOnce(l0,r0).external_box_min_clearance_m; };
        double lo=g0.z()-0.80, hi=g0.z()+0.05;  // clearance DECREASES as box z rises
        for(int it=0;it<40;it++){ double mid=(lo+hi)/2; if(clrAt(mid) > 0.060) lo=mid; else hi=mid; }
        bp.t.z()=(lo+hi)/2;
        mon.setExternalBoxes({bp, ExternalBoxPose{}}, 0.0);
        // deg/s to hit approach_mps of gripper descent: |dz/ddeg|·(deg/s)=m/s -> deg/s=mps/|dz/ddeg along dir|
        double dzds=0; for(int i=0;i<6;i++) dzds+=grad[i]*dir[i]; // dz per deg along dir (negative)
        const double degps = approach_mps/std::fabs(dzds);
        const double dt=0.002;  // async monitor lag (ticks) passed in: eval>tick + load
        std::array<double,6> lprev=L, rprev=R;
        JointArray maxv={170,170,170,240,240,320};
        double minclear=1e9;
        std::vector<CollisionVerdict> vbuf;  // ring of recent verdicts (async lag)
        for(int t=0;t<1500;t++){ // 3s
            JointArray lp,rp,lt,rt;
            for(int i=0;i<6;i++){ lp[i]=lprev[i]; rp[i]=rprev[i];
                lt[i]=lprev[i]+dir[i]*degps*dt; rt[i]=rprev[i]; }
            mon.setExternalBoxes({bp, ExternalBoxPose{}}, 0.0);  // keep the box live each tick
            CollisionVerdict vnow = mon.evalOnce(lp, rp);
            vbuf.push_back(vnow);
            if(vnow.external_box_min_clearance_m<minclear) minclear=vnow.external_box_min_clearance_m;
            // Barrier acts on a LAGGED verdict (async monitor), age-extrapolated by verdict_age.
            const CollisionVerdict& v = vbuf[std::max(0,(int)vbuf.size()-1-lag)];
            const double vage = std::min((int)vbuf.size()-1, lag)*dt;
            auto proj = applyCollisionVelocityProjection(v, c, lp, rp, lt, rt, dt, vage, maxv);
            if(trace && t%40==0){ int nb=0; double fc=1e9; for(const auto&p:v.near){ if(p.external_box)nb++; if(p.d_m<fc)fc=p.d_m; }
                printf("    t=%4d box_clear=%+7.1fmm near=%zu(box=%d) nearest=%+.1fmm proj_corr=%.2fdeg/s\n",
                       t, v.external_box_min_clearance_m*1000, v.near.size(), nb, fc*1000, proj.max_correction_deg_s); }
            for(int i=0;i<6;i++){ lprev[i]=lt[i]; rprev[i]=rt[i]; }
        }
        const double real = minclear + c.external_boxes.margin_m[2];  // clearance to the PHYSICAL box
        printf("  [%s] approach=%.2f m/s lag=%dms -> inflated %+7.1f mm | REAL box %+7.1f mm  (%s)\n",
               use_box_params?"BOX  d_slow=80":"FLOOR d_slow=5", approach_mps, lag*2, minclear*1000,
               real*1000, real < -0.001 ? "PENETRATED REAL BOX" : "outside real box");
    };
    printf("gripper start z_stand=%.3f m; box top 60mm below (auto-placed).\n", g0.z());
    for(int lag : {4, 25}){  // nominal async lag vs degraded (GPU-loaded, near staleness limit)
        printf("--- monitor lag = %d ticks (%d ms) ---\n", lag, lag*2);
        for(double v: {0.30, 0.50, 0.78}){ run(false, v, lag); run(true, v, lag); }
    }
    return 0;
}
