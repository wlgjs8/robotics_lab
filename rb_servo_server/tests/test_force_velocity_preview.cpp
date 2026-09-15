#include "rb_servo/experimental/force_reference.hpp"
#include "rb_servo/experimental/velocity_preview.hpp"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

using namespace rb_servo::experimental;
using V=Eigen::Vector3d;
namespace {
constexpr double dt=.002,pi=3.141592653589793;
void require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
// Sensitivity-test parameters, NOT an identified robot/environment or runtime
// defaults. The previous position-pursuit candidate used 4 kg; both are audited.
ForceReferenceConfig forceConfig(){return {10,8,3,10,500,44400,.6,5,2000};}
VelocityPreviewConfig previewConfig(){return {.01,24,.6,5,2000,.1,.1,.02,.01,1e-8,1000,.25};}
VelocityPreviewReference constant(const V& v) {
    VelocityPreviewReference r;r.count=2;r.knots[0]={0,v};r.knots[1]={.24,v};return r;
}
struct Plant {
    ForceReference force_controller;
    VelocityPreview preview{previewConfig()};
    VelocityPreviewState nominal_seed{};
    std::deque<V> positions,forces,sensor_positions;
    V measured{V::Zero()},velocity{V::Zero()},physical_force{V::Zero()},observed{V::Zero()};
    V normal{V(1,2,3).normalized()};
    double stiffness{4400},wall_position{.02},tool_mass{.814},noise_m{0},ripple_n{0};
    double peak_force{0},maximum_jerk{0},time{0},force_gain{1};
    int pose_period{1},tick_index{0};
    bool wall{true},use_preview{true},vector_noise{false},paired_sensor_delay{false};
    ForceReferenceResult last{};
    explicit Plant(int command_delay=9,int force_delay=0,const ForceReferenceConfig& c=forceConfig())
        :force_controller(c,dt),positions(command_delay,V::Zero()),forces(force_delay,V::Zero()),sensor_positions(force_delay,V::Zero()) {
        force_controller.reset({});
    }
    void tick(const V& intent) {
        const V old_velocity=velocity,previous=measured;
        positions.push_back(force_controller.state().position);
        measured=positions.front();positions.pop_front();velocity=(measured-previous)/dt;
        const V acceleration=(velocity-old_velocity)/dt;
        physical_force=wall?-stiffness*std::max(0.,normal.dot(measured)-wall_position)*normal:V::Zero().eval();
        const V ripple=vector_noise?V(std::sin(2*pi*41*time),std::sin(2*pi*73*time),std::sin(2*pi*113*time)):
                                          std::sin(2*pi*50*time)*normal;
        // Deliberately leave actual tool inertia in the measurement. No noisy
        // second difference of observed pose is used as a force estimate.
        forces.push_back(force_gain*physical_force-tool_mass*acceleration+ripple_n*ripple);
        const V sensed=forces.front();forces.pop_front();
        sensor_positions.push_back(measured);const V lagged_position=sensor_positions.front();sensor_positions.pop_front();
        const V perturbed=(paired_sensor_delay?lagged_position:measured)+noise_m*V(std::sin(2*pi*73*time),std::sin(2*pi*41*time),std::sin(2*pi*113*time));
        for(int axis=0;axis<3;++axis)if((tick_index+axis)%pose_period==0)observed[axis]=perturbed[axis];
        V requested=intent;
        if(use_preview) {
            if(tick_index%10==0) {
                if(tick_index) {VelocityPreviewSample s;require(preview.trajectory().sample(.02,s),"nominal seed unavailable");nominal_seed=s;}
                const auto result=preview.plan(constant(intent),nominal_seed);
                if(!result.valid)std::cerr<<"preview tick="<<tick_index<<" reason="<<result.reason<<" residual="<<result.max_constraint_violation<<'\n';
                require(result.valid,"velocity preview refused");
            }
            VelocityPreviewSample s;
            require(preview.trajectory().sample((tick_index%10+1)*dt,s),"nominal preview expired");requested=s.velocity;
        }
        const auto before=force_controller.state();
        last=force_controller.stepVelocity(requested,observed,sensed);
        require(last.valid,last.reason);
        const auto& c=force_controller.config();const auto& after=last.state;
        const double j=(after.acceleration-before.acceleration).cwiseAbs().maxCoeff()/dt;
        maximum_jerk=std::max(maximum_jerk,j);peak_force=std::max(peak_force,physical_force.norm());
        require(after.velocity.cwiseAbs().maxCoeff()<=c.max_velocity_m_s+1e-8,"velocity limit");
        require(after.acceleration.cwiseAbs().maxCoeff()<=c.max_acceleration_m_s2+1e-7,"acceleration limit");
        require(j<=c.max_jerk_m_s3+1e-6,"acceleration discontinuity");
        ++tick_index;time=tick_index*dt;
    }
};
void invalidAndFrames() {
    ForceReference a(forceConfig(),dt),b(forceConfig(),dt);
    require(!a.stepVelocity(V::Zero(),V::Zero(),V::Zero()).valid,"unseeded intent accepted");
    a.reset({});b.reset({});
    require(!a.stepVelocity(V::Constant(std::numeric_limits<double>::quiet_NaN()),V::Zero(),V::Zero()).valid,"NaN intent accepted");
    require(a.state().position.isZero(0),"refused intent mutated state");
    require(!a.stepVelocity(V::Constant(1e200),V::Zero(),V::Zero()).valid,"overflowing intent silently became zero speed");
    // Geometry/projection is frame-covariant. Ruckig's per-axis physical bounds
    // are explicitly not a rotation-invariant norm constraint.
    const Eigen::Matrix3d R=Eigen::AngleAxisd(.7,V(1,2,3).normalized()).toRotationMatrix();
    const V f(-12,2,3),intent(.08,.01,-.02),measured(-.001,.002,.003);
    const auto x=a.stepVelocity(intent,measured,f),y=b.stepVelocity(R*intent,R*measured,R*f);
    require(x.valid&&y.valid,"rotated input refused");
    require((R*x.compliant_velocity-y.compliant_velocity).norm()<1e-12,"projection depends on a press axis");
    const V tangent=f.unitOrthogonal();
    require(std::abs(tangent.dot(x.compliant_velocity-intent))<1e-12,"force erased tangent intent");
    // The same accepted p/v/a with translated coordinates has identical motion.
    const V translation(5,-2,3);ForceReferenceState seed;seed.position=translation;b.reset(seed);a.reset({});
    for(int i=0;i<1000;++i) {
        const auto u=a.stepVelocity(intent,a.state().position,V::Zero());
        const auto v=b.stepVelocity(intent,b.state().position,V::Zero());
        require(u.valid&&v.valid&&(v.state.position-u.state.position-translation).norm()<1e-10,"hidden position fence/origin dependence");
    }
}
void releaseAndFree() {
    std::vector<double> peaks;
    for(int hold_ticks:{1500,4500,10000}) {
        Plant p;p.normal=V::UnitX();p.ripple_n=1;p.noise_m=5e-6;p.pose_period=2;
        const V intent(.02,0,0);
        for(int i=0;i<hold_ticks;++i)p.tick(intent);
        require(p.physical_force.norm()<10,"held contact criterion failed");
        p.wall=false;double maximum=0;
        for(int i=0;i<1000;++i) {p.tick(intent);maximum=std::max(maximum,p.force_controller.state().velocity.x());}
        require(maximum<.022,"release accelerated above constant input");
        require(std::abs(p.force_controller.state().velocity.x()-.02)<1e-5,"release did not resume intent");
        peaks.push_back(maximum);
    }
    require(*std::max_element(peaks.begin(),peaks.end())-*std::min_element(peaks.begin(),peaks.end())<.001,
            "longer obstruction accumulated release motion");
    for(double speed:{.02,.15,.3}) {
        Plant p;p.wall=false;p.ripple_n=1;p.vector_noise=true;p.noise_m=5e-6;p.pose_period=2;
        const V intent=speed*p.normal;double low=1e100,high=-1e100;
        for(int i=0;i<4000;++i) {p.tick(intent);if(i>=3500) {const double v=p.normal.dot(p.force_controller.state().velocity);low=std::min(low,v);high=std::max(high,v);}}
        require(std::abs(p.normal.dot(p.force_controller.state().velocity)-speed)<1e-4,"free speed distorted");
        require(high-low<.001,"free-space self excitation");
        require(p.force_controller.state().position.norm()>.1,"travel radius restriction");
    }
    // Persistent absolute goals remain possible through a bounded pursuit task.
    // The task does not integrate the position error or the obstructed velocity.
    Plant p;p.use_preview=false;p.normal=V::UnitX();const V goal(.1,0,0);
    auto pursuit=[&]() {V v=2*(goal-p.force_controller.state().position);if(v.norm()>.03)v*=.03/v.norm();return v;};
    for(int i=0;i<3000;++i)p.tick(pursuit());
    require(p.physical_force.norm()<10,"absolute goal force failure");p.wall=false;
    for(int i=0;i<4000;++i)p.tick(pursuit());
    require((p.force_controller.state().position-goal).norm()<.001,"bounded goal not reached after release");
}
void subTickContinuity() {
    ForceReference controller(forceConfig(),dt);controller.reset({});ForceReferenceState previous;
    require(!controller.sampleLastStep(0,previous),"reset exposes a prior interval");
    double missed_acceleration=0;
    for(int i=0;i<1000;++i) {
        const auto before=controller.state();
        const V intent(.02*std::sin(i*.007),.01,-.015);
        const V force=i>300&&i<650?V(-12,2,1):V::Zero().eval();
        const auto result=controller.stepVelocity(intent,before.position,force);require(result.valid,result.reason);
        require(controller.sampleLastStep(0,previous),"interval origin unavailable");
        require((previous.position-before.position).norm()<1e-11&&
                (previous.velocity-before.velocity).norm()<1e-10&&
                (previous.acceleration-before.acceleration).norm()<1e-9,"p/v/a splice discontinuity");
        for(int k=1;k<=32;++k) {
            ForceReferenceState s;require(controller.sampleLastStep(dt*k/32,s),"subtick sample unavailable");
            require(s.velocity.cwiseAbs().maxCoeff()<=.6+1e-8,"subtick velocity bound");
            require(s.acceleration.cwiseAbs().maxCoeff()<=5+1e-7,"subtick acceleration bound");
            require((s.acceleration-previous.acceleration).cwiseAbs().maxCoeff()/(dt/32)<=2000+1e-5,"subtick jerk bound");
            missed_acceleration=std::max(missed_acceleration,s.acceleration.cwiseAbs().maxCoeff()-result.state.acceleration.cwiseAbs().maxCoeff());
            previous=s;
        }
        require((previous.position-result.state.position).norm()<1e-11,"interval endpoint mismatch");
    }
    require(missed_acceleration>.1,"fixture no longer detects endpoint-only derivative blindness");
    require(!controller.sampleLastStep(-dt,previous)&&!controller.sampleLastStep(2*dt,previous),"subtick sampler extrapolated");
    controller.clear();require(!controller.sampleLastStep(0,previous),"clear retained interval");
}
// Writing an audit is not a passing stability assertion. The cross-product
// includes unqualified sensor delay, force gain and vector-noise stresses.
void audit(const std::string& path,bool stress,bool damping_study=false,bool combined=false,bool paired=false) {
    std::ofstream csv(path);require(csv.good(),"audit file unavailable");csv<<std::setprecision(12);
    csv<<"mass_kg,damping_ns_m,predictor_n_m,wall,stiffness_n_m,command_fifo_sec,force_fifo_sec,paired_sensor_delay,speed_m_s,noise_m,force_gain,vector_noise,preview,valid,peak_force_n,tail_min_n,tail_max_n,tail_pp_n,longest_above_10_sec,tail_velocity_pp_m_s,final_speed_error_m_s,tail_below_10,tail_ripple_pass,reason\n";
    int passed=0,count=0;
    const std::vector<double> masses=damping_study?std::vector<double>{10,20,40,80}:
        (stress||combined?std::vector<double>{10}:std::vector<double>{4,10});
    const std::vector<double> predictors=stress&&!damping_study?std::vector<double>{15000,44400,90000}:std::vector<double>{44400};
    for(double mass:masses)for(double predictor:predictors)
    for(bool wall:{false,true})for(double stiffness:{1000.,4400.,15000.,44400.})
    for(int delay:{3,9,13})for(double speed:{.02,.15})
    for(double noise:damping_study?std::vector<double>{5e-6}:std::vector<double>{0.,1e-6,5e-6})
    for(int force_delay:stress?std::vector<int>{0,2,4}:std::vector<int>{0})
    for(double gain:stress?std::vector<double>{.8,1.,1.2}:std::vector<double>{1.}) {
        if(!wall&&stiffness!=4400)continue;
        auto c=forceConfig();c.mass_kg=mass;c.contact_stiffness_n_m=predictor;
        if(damping_study)c.damping_ns_m=mass/.02; // explicit equal M/D comparison
        Plant p(delay,force_delay,c);p.wall=wall;p.stiffness=stiffness;p.noise_m=noise;p.ripple_n=1;
        p.pose_period=2;p.vector_noise=stress;p.force_gain=gain;
        p.paired_sensor_delay=paired;
        // Direct intent isolates force robustness; regression separately tests
        // the complete preview+force path and continuous source replacement.
        p.use_preview=combined;
        double lo=1e100,hi=0,vlo=1e100,vhi=-1e100,streak=0,longest=0;
        bool valid=true;const char* reason="completed";
        try {for(int i=0;i<5000;++i) {
            p.tick(speed*p.normal);
            if(p.physical_force.norm()>=10) {streak+=dt;longest=std::max(longest,streak);}else streak=0;
            if(i>=4500) {lo=std::min(lo,p.physical_force.norm());hi=std::max(hi,p.physical_force.norm());
                const double v=p.normal.dot(p.force_controller.state().velocity);vlo=std::min(vlo,v);vhi=std::max(vhi,v);}
        }}catch(const std::exception&){valid=false;reason="controller_refused";}
        const double speed_error=(p.force_controller.state().velocity-speed*p.normal).norm();
        const bool sustained=valid&&hi<10,ripple=valid&&(wall?hi-lo<.2:vhi-vlo<.001&&speed_error<.001);
        ++count;if(sustained&&ripple)++passed;
        csv<<mass<<','<<c.damping_ns_m<<','<<predictor<<','<<wall<<','<<stiffness<<','<<delay*dt<<','<<force_delay*dt<<','<<paired<<','<<speed<<','<<noise<<','<<gain<<','<<stress<<','<<p.use_preview<<','<<valid<<','<<p.peak_force<<','<<lo<<','<<hi<<','<<hi-lo<<','<<longest<<','<<vhi-vlo<<','<<speed_error<<','<<sustained<<','<<ripple<<','<<reason<<'\n';
    }
    require(csv.good(),"audit write failed");std::cout<<"force velocity audit: "<<passed<<'/'<<count<<" meet final-second criteria; not hardware qualification\n";
}

// Two translational ideal position actuators sharing one normal spring. This
// tests interacting feedback, not grasping, friction, six-axis contact, torque,
// object transfer, robot IK or a dual-arm hardware handover.
void coupledAudit(const std::string& path) {
    std::ofstream out(path);require(out.good(),"coupled audit unavailable");out<<std::setprecision(12);
    out<<"mass_kg,damping_ns_m,stiffness_n_m,left_command_fifo_sec,right_command_fifo_sec,force_fifo_sec,each_speed_m_s,valid,peak_force_n,tail_min_n,tail_max_n,tail_pp_n,longest_above_10_sec,tail_below_10,tail_ripple_pass\n";
    const V n=V(1,2,3).normalized();int passed=0,count=0;
    for(double mass:{10.,40.})for(double stiffness:{1000.,4400.,15000.,44400.})
    for(int dl:{3,9,13})for(int dr:{3,9,13})for(int df:{0,4})for(double speed:{.02,.075}) {
        auto c=forceConfig();c.mass_kg=mass;c.damping_ns_m=mass/.02;
        ForceReference left(c,dt),right(c,dt);ForceReferenceState seed;left.reset(seed);seed.position=.04*n;right.reset(seed);
        std::array<std::deque<V>,2> positions{std::deque<V>(dl,V::Zero()),std::deque<V>(dr,seed.position)};
        std::array<std::deque<V>,2> forces{std::deque<V>(df,V::Zero()),std::deque<V>(df,V::Zero())};
        std::array<V,2> actual{V::Zero(),seed.position},v{V::Zero(),V::Zero()},observed=actual;
        double peak=0,lo=1e100,hi=0,streak=0,longest=0;bool valid=true;
        for(int i=0;i<5000&&valid;++i) {
            const std::array<V,2> command{left.state().position,right.state().position};
            std::array<V,2> acceleration;
            for(int arm=0;arm<2;++arm) {
                const V previous=actual[arm],oldv=v[arm];positions[arm].push_back(command[arm]);
                actual[arm]=positions[arm].front();positions[arm].pop_front();v[arm]=(actual[arm]-previous)/dt;
                acceleration[arm]=(v[arm]-oldv)/dt;
                const V noisy=actual[arm]+5e-6*V(std::sin(2*pi*73*i*dt),std::sin(2*pi*41*i*dt),std::sin(2*pi*113*i*dt));
                for(int axis=0;axis<3;++axis)if((i+axis+arm)%2==0)observed[arm][axis]=noisy[axis];
            }
            const double contact=stiffness*std::max(0.,n.dot(actual[0]-actual[1]));
            peak=std::max(peak,contact);if(contact>=10){streak+=dt;longest=std::max(longest,streak);}else streak=0;
            if(i>=4500){lo=std::min(lo,contact);hi=std::max(hi,contact);}
            for(int arm=0;arm<2;++arm) {
                const double sign=arm?1.:-1.;
                const V noise=V(std::sin(2*pi*41*i*dt+arm),std::sin(2*pi*73*i*dt),std::sin(2*pi*113*i*dt));
                forces[arm].push_back(sign*contact*n-.814*acceleration[arm]+noise);
                const V sensed=forces[arm].front();forces[arm].pop_front();
                const auto result=(arm?right:left).stepVelocity(-sign*speed*n,observed[arm],sensed);
                valid=valid&&result.valid;
            }
        }
        const bool sustained=valid&&hi<10,ripple=valid&&hi-lo<.2;++count;if(sustained&&ripple)++passed;
        out<<mass<<','<<c.damping_ns_m<<','<<stiffness<<','<<dl*dt<<','<<dr*dt<<','<<df*dt<<','<<speed<<','<<valid<<','<<peak<<','<<lo<<','<<hi<<','<<hi-lo<<','<<longest<<','<<sustained<<','<<ripple<<'\n';
    }
    require(out.good(),"coupled audit write failed");std::cout<<"coupled normal-spring audit: "<<passed<<'/'<<count<<" meet final-second criteria; not handover qualification\n";
}
}
int main(int argc,char** argv){try{
    if(argc==3&&std::string(argv[1])=="--audit-csv"){audit(argv[2],false);return 0;}
    if(argc==3&&std::string(argv[1])=="--stress-audit-csv"){audit(argv[2],true);return 0;}
    if(argc==3&&std::string(argv[1])=="--damping-audit-csv"){audit(argv[2],true,true);return 0;}
    if(argc==3&&std::string(argv[1])=="--combined-audit-csv"){audit(argv[2],false,false,true);return 0;}
    if(argc==3&&std::string(argv[1])=="--coupled-audit-csv"){coupledAudit(argv[2]);return 0;}
    if(argc==3&&std::string(argv[1])=="--paired-sensor-audit-csv"){audit(argv[2],true,false,false,true);return 0;}
    require(argc==1,"unknown arguments");invalidAndFrames();releaseAndFree();subTickContinuity();
    std::cout<<"force velocity preview: all regression checks passed\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
