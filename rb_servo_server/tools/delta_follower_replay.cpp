// Offline-only replay of recorded delta_preview inputs. No backend, socket,
// device, GUI or real-time loop is created. See docs/reports/griponly_replay_20260906.md.
#include "rb_servo/config/config.hpp"
#include "rb_servo/control/cartesian_chunk_follower.hpp"
#include "rb_servo/control/follower_output_smd.hpp"
#include "rb_servo/math/se3.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <set>

using namespace rb_servo;
using namespace rb_servo::control;
using json = nlohmann::json;

namespace {
Pose6D pose(const json& row) {
    const auto v = row.get<std::array<double, 7>>();
    for (double x : v) if (!std::isfinite(x)) throw std::runtime_error("nonfinite pose");
    Eigen::Quaterniond q(v[6], v[3], v[4], v[5]);
    if (q.norm() < 1e-9) throw std::runtime_error("zero quaternion");
    return math::poseFromSe3(pinocchio::SE3(q.normalized().toRotationMatrix(), Eigen::Vector3d(v[0],v[1],v[2])));
}
Vec6 delta(const json& row) {
    if (!row.is_array() || row.size() != 7) throw std::runtime_error("delta row must have 7 values");
    for (const auto& x : row) if (!x.is_number() || !std::isfinite(x.get<double>())) throw std::runtime_error("nonfinite delta");
    return Vec6{row[0],row[1],row[2],row[3],row[4],row[5]};
}
CartesianChunkFollowerConfig config(const RuckigFollowerConfig& r) {
    CartesianChunkFollowerConfig c;
    c.lin={r.max_linear_velocity_m_s,r.max_linear_accel_m_s2,r.max_linear_jerk_m_s3};
    c.ang={r.max_angular_velocity_rad_s,r.max_angular_accel_rad_s2,r.max_angular_jerk_rad_s3};
    c.window={r.discard_head_steps,r.consume_steps,r.reserve_steps,r.smoothing_window};
    c.guard.af_damping_beta_lin=r.af_damping_beta_lin;
    c.guard.af_damping_beta_ang=r.af_damping_beta_ang;
    c.guard.corner_deadband_lin_m=r.corner_deadband_lin_m;
    c.guard.corner_deadband_ang_rad=r.corner_deadband_ang_rad;
    c.guard.corner_velocity_scale=r.corner_velocity_scale;
    return c;
}
Vec6 blend(const Vec6& old, const Vec6& next, double w) {
    const Eigen::Quaterniond qa(math::exp3(Eigen::Vector3d(old.rx,old.ry,old.rz)));
    const Eigen::Quaterniond qb(math::exp3(Eigen::Vector3d(next.rx,next.ry,next.rz)));
    const auto r = math::log3(qa.slerp(w,qb).normalized().toRotationMatrix());
    return {(1-w)*old.x+w*next.x,(1-w)*old.y+w*next.y,(1-w)*old.z+w*next.z,r[0],r[1],r[2]};
}
}

int main(int argc,char** argv) {
  try {
    if (argc!=5 && argc!=6) throw std::runtime_error("usage: delta_follower_replay CONFIG EVENTS.jsonl OUTPUT.csv VARIANT [VARIANT_START_SEC]");
    const std::string variant=argv[4];
    const std::set<std::string> variants={"baseline","no_force_gate","linear_relaxed","angular_relaxed",
        "zero_af","zero_af_linear","zero_af_angular","no_corner_brake","translation_only","rotation_only",
        "overlap_linear2","overlap_linear4","overlap_quintic4"};
    if (!variants.count(variant)) throw std::runtime_error("unknown variant");
    const double variant_start=argc==6?std::stod(argv[5]):0.;
    if(!std::isfinite(variant_start) || variant_start<0) throw std::runtime_error("invalid variant start");
    if(argc==6 && variant!="baseline" && variant!="no_force_gate" && variant.rfind("overlap_",0)!=0)
        throw std::runtime_error("delayed activation supports only gate/overlap variants");
    const auto all=loadConfigFromYaml(argv[1]);
    const RuckigFollowerConfig* r=nullptr;
    for (const auto& p:all.cartesian_control.tcp_pose_target_profiles) if(p.name=="flow_infer_smooth") r=&p.ruckig_follower;
    if(!r || !r->enable || r->controller!=RuckigFollowerController::DeltaPreview)
        throw std::runtime_error("flow_infer_smooth must explicitly enable delta_preview");
    auto c=config(*r);
    // Counterfactual limits belong ONLY to this offline process. They are not
    // emitted as a configuration recommendation or applied to any robot.
    if(variant=="linear_relaxed") {c.lin.a_max*=4;c.lin.j_max*=4;}
    if(variant=="angular_relaxed") {c.ang.a_max*=4;c.ang.j_max*=4;}
    if(variant=="zero_af" || variant=="zero_af_linear") c.guard.af_damping_beta_lin=0;
    if(variant=="zero_af" || variant=="zero_af_angular") c.guard.af_damping_beta_ang=0;
    if(variant=="no_corner_brake") c.guard.corner_velocity_scale=1;
    CartesianChunkFollower f(c);
    FollowerOutputSmd smd(r->output_smd);
    std::ifstream in(argv[2]);std::ofstream out(argv[3]);
    if(!in || !out) throw std::runtime_error("cannot open replay input/output");
    out << "t,active,segment,wire_seq,step,policy_dt,duration,converged,corner,stall,projection_m,gate,plan_gate,blended_rows";
    for(const auto name:{"raw","smd"}) for(const auto axis:{"x","y","z"}) out<<','<<name<<'_'<<axis;
    for(const auto name:{"axis_duration","vf","af"}) for(int i=0;i<6;++i)out<<','<<name<<'_'<<i;
    out<<'\n'<<std::setprecision(12);
    double prev_t=-1,policy_dt=0;ChunkFrame previous;
    bool was_active=false;int blended_rows=0,ticks=0;std::string line;
    while(std::getline(in,line)) {
      const json j=json::parse(line);
      const double t=j.at("t");
      if(!std::isfinite(t) || (ticks && t<=prev_t)) throw std::runtime_error("timestamps must increase");
      prev_t=t;++ticks;
      if(!j.at("active").get<bool>()) {f.deactivate();smd.deactivate();previous={};was_active=false;continue;}
      const Pose6D ref=pose(j.at("reference"));
      if(j.contains("frame")) {
        const auto& b=j.at("frame");ChunkFrame frame;
        frame.policy_dt=b.at("policy_dt");frame.wire_seq=b.at("seq");frame.recv_seq=frame.wire_seq;frame.recv_time=t;
        if(!(frame.policy_dt>0)) throw std::runtime_error("invalid policy dt");
        policy_dt=frame.policy_dt;
        for(const auto& row:b.at("delta")) {frame.delta.push_back(delta(row));frame.grip.push_back(row[6]);}
        if(frame.delta.size()<2) throw std::runtime_error("delta frame too short");
        for(auto& d:frame.delta) {
          if(variant=="translation_only") d.rx=d.ry=d.rz=0;
          if(variant=="rotation_only") d.x=d.y=d.z=0;
        }
        blended_rows=0;
        const bool overlap=variant.rfind("overlap_",0)==0 && t>=variant_start;
        const int span=variant=="overlap_linear2"?2:4;
        const auto offset=f.windowIndex();
        if(overlap && was_active && !previous.delta.empty()) {
          // Frozen-prediction overlap experiment: blend only matching available
          // future body deltas. Keep the new gripper commands untouched.
          for(int i=0;i<span && i<static_cast<int>(frame.delta.size()) && offset+i<previous.delta.size();++i) {
            double w=double(i)/double(span-1);
            if(variant=="overlap_quintic4") w=w*w*w*(10+w*(-15+6*w));
            frame.delta[i]=blend(previous.delta[offset+i],frame.delta[i],w);++blended_rows;
          }
        }
        previous=frame;
        f.submitDeltaFrame(frame,ref);
      }
      if(!f.active()) throw std::runtime_error("active tick without a seed frame");
      const auto g=j.at("force").get<std::array<double,3>>();
      const Eigen::Vector3d force(g[0],g[1],g[2]);
      const double gate=variant=="no_force_gate" && t>=variant_start?1.0:j.at("gate").get<double>();
      const double pg=j.at("plan_gate");
      if(!std::isfinite(gate)||gate<0||gate>1||!std::isfinite(pg)||pg<0||pg>1||!force.allFinite())
          throw std::runtime_error("invalid gate input");
      f.setAdvanceGate(gate,force.norm()>1e-9?Eigen::Vector3d(force.normalized()):Eigen::Vector3d::Zero());
      f.setPlanRateGate(pg);
      const double tick_dt=j.at("dt");
      if(!std::isfinite(tick_dt)||tick_dt<=0) throw std::runtime_error("invalid servo dt");
      const Pose6D raw=f.tick(tick_dt);
      const Vec6 vel=f.currentVelocity().value_or(Vec6{});
      if(!smd.active()) smd.reset(ref,vel);
      const Pose6D filtered=r->output_smd.enable?smd.step(raw,vel,tick_dt):raw;
      const auto& d=f.diag();
      out<<t<<",1,"<<d.segments<<','<<d.seg_wire_seq<<','<<d.seg_step_index<<','<<policy_dt<<','<<d.last_solve.duration
         <<','<<d.last_solve.converged<<','<<d.last_solve.corner<<','<<d.stall<<','<<d.projection_error_m<<','<<gate<<','<<pg<<','<<blended_rows
         <<','<<raw.x<<','<<raw.y<<','<<raw.z<<','<<filtered.x<<','<<filtered.y<<','<<filtered.z;
      for(const auto* values:{&d.last_solve.axis_duration_sec,&d.last_solve.target_velocity,&d.last_solve.target_acceleration})
        for(double v:*values)out<<','<<v;
      out<<'\n';was_active=true;
    }
    if(!ticks)throw std::runtime_error("empty replay");
    if(!out.good())throw std::runtime_error("replay output write failed");
    std::cout<<"offline delta replay completed: "<<variant<<", "<<ticks<<" input ticks\n";
    return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 2;}
}
