// Deliberately links no servo core/backend. This is exogenous force playback,
// not a counterfactual robot rollout or an IK/queue/contact stability test.
#include "rb_servo/experimental/force_reference.hpp"
#include "rb_servo/experimental/velocity_preview.hpp"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>
using namespace rb_servo::experimental;
using V=Eigen::Vector3d;
using Json=nlohmann::json;
namespace {
void require(bool v,const char* reason){if(!v)throw std::runtime_error(reason);}
V vector(const Json& j){const auto a=j.get<std::array<double,3>>();V v(a[0],a[1],a[2]);require(v.allFinite(),"nonfinite vector");return v;}
struct Frame {std::size_t index;std::uint64_t sequence;double period;std::vector<V> velocities;};
}
int main(int argc,char** argv) {
try {
    require(argc==3,"usage: replay_velocity_force_reference INPUT.json OUTPUT.csv");
    std::ifstream file(argv[1]);require(file.good(),"input unavailable");const auto root=Json::parse(file);
    require(root.at("schema")=="robotics_lab.velocity_force_replay.v1","invalid schema");
    const double dt=root.at("period_sec");require(dt==.002,"fixture requires a 2 ms servo period");
    const auto& f=root.at("force_config");
    ForceReferenceConfig fc{f.at("sustained_force_n"),f.at("recovery_force_n"),f.at("noise_force_n"),f.at("mass_kg"),
        f.at("damping_ns_m"),f.at("contact_stiffness_n_m"),f.at("max_velocity_m_s"),f.at("max_acceleration_m_s2"),f.at("max_jerk_m_s3")};
    const auto& p=root.at("preview_config");
    VelocityPreviewConfig pc{p.at("step_sec"),p.at("horizon_steps"),p.at("max_velocity_m_s"),p.at("max_acceleration_m_s2"),p.at("max_jerk_m_s3"),
        p.at("tracking_scale_m_s"),p.at("acceleration_weight"),p.at("jerk_weight"),p.at("jerk_difference_weight"),
        p.at("feasibility_tolerance"),p.at("max_working_set_recalculations"),p.at("max_solve_time_sec")};
    const std::size_t cadence=root.at("preview_replan_ticks");require(cadence>=1&&cadence*dt<pc.horizon_steps*pc.step_sec,"invalid preview cadence");
    ForceReference force(fc,dt);force.reset({});VelocityPreview preview(pc);
    const auto& samples=root.at("samples");require(samples.is_array()&&!samples.empty(),"empty force samples");
    std::vector<Frame> frames;
    for(const auto& j:root.at("frames")) {
        Frame frame{j.at("sample_index"),j.at("source_sequence"),j.at("period_sec"),{}};
        require(frame.index<samples.size()&&std::isfinite(frame.period)&&frame.period>0,"invalid frame time");
        require(frame.sequence>0&&(frames.empty()||(frame.index>frames.back().index&&frame.sequence>frames.back().sequence)),"nonmonotonic frame arrival/sequence");
        for(const auto& v:j.at("velocities_m_s"))frame.velocities.push_back(vector(v));
        require(!frame.velocities.empty(),"empty frame");frames.push_back(frame);
    }
    const bool fixed=root.contains("constant_intent_m_s");
    require(fixed==frames.empty(),"use exactly one of constant intent and arrival frames");
    const V fixed_intent=fixed?vector(root.at("constant_intent_m_s")):V::Zero().eval();
    const bool zero_force=root.at("zero_force");
    std::ofstream out(argv[2]);require(out.good(),"output unavailable");out<<std::setprecision(17);
    out<<"tick,t,phase,source_sequence,force_x,force_y,force_z,intent_x,intent_y,intent_z,nominal_x,nominal_y,nominal_z,p_x,p_y,p_z,v_x,v_y,v_z,a_x,a_y,a_z,constraint_active,preview_solve_sec,subtick_max_acceleration_axis_m_s2,subtick_acceleration_difference_m_s3\n";
    std::size_t cursor=0,last_plan_tick=0;const Frame* active=nullptr;
    VelocityPreviewState nominal_seed;std::vector<double> solves;
    for(std::size_t i=0;i<samples.size();++i) {
        const bool fresh=cursor<frames.size()&&frames[cursor].index==i;
        if(fresh)active=&frames[cursor++];
        const bool stopped=samples[i].at("stop_intent");
        const auto intent=[&](double offset)->V {
            if(stopped)return V::Zero();
            if(fixed)return fixed_intent;
            if(!active)return V::Zero();
            const double row=std::floor(((i-active->index)*dt+offset)/active->period);
            return row<active->velocities.size()?active->velocities[static_cast<std::size_t>(row)]:V::Zero().eval();
        };
        // Only the already arrived frame contributes future velocity. The next
        // recorded frame is not consulted until its actual consumer tick.
        if(i==0||fresh||i-last_plan_tick>=cadence||stopped!=samples[i-1].at("stop_intent").get<bool>()) {
            if(i) {VelocityPreviewSample seed;require(preview.trajectory().sample((i-last_plan_tick)*dt,seed),"nominal predecessor expired");nominal_seed=seed;}
            VelocityPreviewReference ref;ref.count=pc.horizon_steps+1;
            for(std::size_t k=0;k<ref.count;++k)ref.knots[k]={k*pc.step_sec,intent(k*pc.step_sec)};
            const auto result=preview.plan(ref,nominal_seed);require(result.valid,result.reason);
            solves.push_back(result.solve_time_sec);last_plan_tick=i;
        }
        VelocityPreviewSample nominal;
        require(preview.trajectory().sample((i-last_plan_tick+1)*dt,nominal),"nominal sample expired");
        const V measured=force.state().position; // ideal one-step command application, no physical force feedback
        const V wrench=zero_force?V::Zero().eval():vector(samples[i].at("force_n"));
        const auto before=force.state();const auto result=force.stepVelocity(nominal.velocity,measured,wrench);
        require(result.valid,result.reason);
        require((result.state.acceleration-before.acceleration).cwiseAbs().maxCoeff()/dt<=fc.max_jerk_m_s3+1e-6,"jerk bound");
        double max_a=0,max_j=0;ForceReferenceState sub_previous;
        require(force.sampleLastStep(0,sub_previous),"subtick origin unavailable");
        require((sub_previous.position-before.position).norm()<1e-10&&
                (sub_previous.velocity-before.velocity).norm()<1e-9&&
                (sub_previous.acceleration-before.acceleration).norm()<1e-8,"generated-state splice mismatch");
        for(int k=1;k<=32;++k) {
            ForceReferenceState sub;require(force.sampleLastStep(dt*k/32,sub),"subtick sample unavailable");
            max_a=std::max(max_a,sub.acceleration.cwiseAbs().maxCoeff());
            max_j=std::max(max_j,(sub.acceleration-sub_previous.acceleration).cwiseAbs().maxCoeff()/(dt/32));
            require(sub.velocity.cwiseAbs().maxCoeff()<=fc.max_velocity_m_s+1e-8,"subtick velocity limit");
            sub_previous=sub;
        }
        require(max_a<=fc.max_acceleration_m_s2+1e-7&&max_j<=fc.max_jerk_m_s3+1e-5,"subtick derivative limit");
        out<<i<<','<<i*dt<<','<<std::quoted(samples[i].at("phase").get<std::string>(),'"','"')<<','<<(active?active->sequence:0);
        for(const V& v:{wrench,intent(0),nominal.velocity,result.state.position,result.state.velocity,result.state.acceleration})
            for(int a=0;a<3;++a)out<<','<<v[a];
        out<<','<<result.constraint_active<<','<<solves.back()<<','<<max_a<<','<<max_j<<'\n';
    }
    require(out.good(),"output write failed");
    std::sort(solves.begin(),solves.end());
    Json summary={{"schema","robotics_lab.velocity_force_replay_result.v1"},{"rows",samples.size()},{"frames_consumed",cursor},
        {"preview_solves",solves.size()},{"solve_p95_sec",solves[static_cast<std::size_t>(.95*(solves.size()-1))]},
        {"solve_max_sec",solves.back()},{"zero_force",zero_force},
        {"scope","translation-only fixed-force replay; explicit known velocity horizon, synchronous offline preview, ideal applied position, no IK, physical actuator, queue, contact feedback, torque or hardware qualification"}};
    std::ofstream metadata(std::string(argv[2])+".json");metadata<<summary.dump(2)<<'\n';require(metadata.good(),"metadata write failed");
    std::cout<<summary.dump()<<'\n';return 0;
}catch(const std::exception& e){std::cerr<<"velocity force replay: "<<e.what()<<'\n';return 2;}
}
