// OFFLINE ONLY. Fixed recorded goal/force replay through the production
// ForceGate and SmdPoseTracker. No robot backend, servo loop or sockets.
// Legacy direction tracking is an analysis oracle, never a runtime fallback.
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <nlohmann/json.hpp>
#include "rb_servo/config/config.hpp"
#include "rb_servo/control/admittance_overlay.hpp"
#include "rb_servo/control/smd_pose_tracker.hpp"

namespace {
using namespace rb_servo;
using json = nlohmann::json;
using V = math::Vector3;
V vector(const json& j) {
    if (!j.is_array() || j.size() != 3) throw std::runtime_error("expected 3-vector");
    V v(j.at(0).get<double>(), j.at(1).get<double>(), j.at(2).get<double>());
    if (!v.allFinite()) throw std::runtime_error("non-finite vector");
    return v;
}
Pose6D pose(const V& p) { Pose6D out; out.x=p.x(); out.y=p.y(); out.z=p.z(); return out; }
V position(const Pose6D& p) { return {p.x,p.y,p.z}; }
}

int main(int argc, char** argv) {
    try {
        if (argc != 4) throw std::runtime_error("usage: umi_stream_gate_replay CONFIG INPUT.jsonl OUTPUT.csv");
        const auto cfg = loadConfigFromYaml(argv[1]); // parse only; never connect hardware
        std::ifstream input(argv[2]); std::ofstream out(argv[3]);
        if (!input || !out) throw std::runtime_error("cannot open replay input/output");
        std::string line;
        if (!std::getline(input,line)) throw std::runtime_error("missing metadata");
        const auto meta = json::parse(line);
        if (meta.at("schema") != "umi_stream_gate_replay.v1") throw std::runtime_error("unsupported schema");
        const PoseTrackSmdConfig* selected=nullptr;
        for (const auto& p:cfg.cartesian_control.tcp_pose_target_profiles)
            if (p.name == meta.at("profile").get<std::string>()) selected=&p.pose_track_smd;
        if (!selected || !selected->enable || selected->velocity_feedforward)
            throw std::runtime_error("replay requires an explicit SMD profile without feedforward");
        if (meta.at("nf_hz").get<double>() != selected->natural_frequency_linear_hz ||
            meta.at("vmax").get<double>() != selected->max_linear_velocity_m_s ||
            meta.at("amax").get<double>() != selected->max_linear_accel_m_s2)
            throw std::runtime_error("recorded SMD profile differs from config");
        const V seed=vector(meta.at("seed_position")), velocity=vector(meta.at("seed_velocity"));
        SmdPoseTracker legacy(*selected), revised(*selected);
        const Vec6 twist{velocity.x(),velocity.y(),velocity.z(),0,0,0};
        for (auto* tracker:{&legacy,&revised}) {
            tracker->reset(pose(seed),twist);
            tracker->updateGoalFromCommand(pose(seed));
        }
        control::ForceGate gate;
        gate.configure(cfg.force_control,1.0/cfg.servo.rate_hz);
        out<<"t,legacy_x,legacy_y,legacy_z,revised_x,revised_y,revised_z,recorded_x,recorded_y,recorded_z,legacy_cut_m,revised_cut_m,gate,releasing,slow_force_error_n\n"<<std::setprecision(17);
        double max_error=0, previous_t=-1; std::size_t rows=0;
        while(std::getline(input,line)) {
            const auto j=json::parse(line);const double t=j.at("t");
            if (!std::isfinite(t) || t<=previous_t) throw std::runtime_error("non-monotonic time");
            previous_t=t;
            if (!j.at("sample").get<bool>()) {gate.updateStream(vector(j.at("force")));continue;}
            const double dt=j.at("dt");
            if (!std::isfinite(dt) || dt<=0) throw std::runtime_error("invalid SMD dt");
            const V goal=vector(j.at("goal")), recorded=vector(j.at("recorded"));
            std::array<V,2> p; std::array<double,2> cut{};
            for (int i=0;i<2;++i) {
                auto& tracker=i==0?legacy:revised;
                tracker.updateGoalFromCommand(pose(goal));
                const V before=position(tracker.currentPose());
                V advance=position(tracker.step(dt))-before;
                V normal=gate.streamForceDirection();
                if(i==0) {
                    normal=gate.streamMeasuredForce();const double n=normal.norm();
                    if(n>1e-6) normal/=n;else normal.setZero();
                    const double into=advance.dot(normal);
                    if(gate.streamTranslation()<1 && normal.squaredNorm()>.5 && into<0) {
                        const V removed=(1-gate.streamTranslation())*into*normal;
                        advance-=removed;cut[i]=removed.norm();
                    }
                } else advance=gate.applyStreamTranslation(advance,&cut[i]);
                if(cut[i]>0) tracker.constrainTranslation(before+advance,normal,1-gate.streamTranslation());
                p[i]=position(tracker.currentPose());
            }
            const double g=gate.streamTranslation();const bool releasing=gate.streamReleasing();
            gate.updateStream(vector(j.at("force")));
            const double ferr=std::abs(gate.streamForceN()-j.at("slow_force_n").get<double>());
            max_error=std::max(max_error,(p[0]-recorded).norm());++rows;
            out<<t;
            for(const V& v:{p[0],p[1],recorded})for(int a=0;a<3;++a)out<<','<<v[a];
            out<<','<<cut[0]<<','<<cut[1]<<','<<g<<','<<releasing<<','<<ferr<<'\n';
        }
        if(!rows)throw std::runtime_error("no replay samples");
        std::cout<<"rows="<<rows<<" legacy_max_error_mm="<<max_error*1000<<'\n';
        return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
