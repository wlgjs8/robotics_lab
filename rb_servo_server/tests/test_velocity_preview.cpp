#include "rb_servo/experimental/velocity_preview.hpp"
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace rb_servo::experimental;
using V=Eigen::Vector3d;
namespace {
void require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
VelocityPreviewConfig config(){return {.01,24,.6,5,2000,.1,.1,.02,.01,1e-8,1000,.25};}
VelocityPreviewReference constant(const V& v) {
    VelocityPreviewReference r;r.count=2;r.knots[0]={0,v};r.knots[1]={.24,v};return r;
}
void certify(const VelocityPreviewTrajectory& p,const VelocityPreviewConfig& c) {
    VelocityPreviewSample s;
    for(int i=0;i<=2400;++i) {
        require(p.sample(p.durationSec()*i/2400.,s),"sample failed");
        require(s.velocity.cwiseAbs().maxCoeff()<=c.max_velocity_m_s+1e-7,"interior velocity cap");
        require(s.acceleration.cwiseAbs().maxCoeff()<=c.max_acceleration_m_s2+1e-7,"interior acceleration cap");
        require(s.jerk.cwiseAbs().maxCoeff()<=c.max_jerk_m_s3+1e-5,"jerk cap");
    }
    // Independent endpoint and interior extremum checks, not the solver's rows.
    for(std::size_t k=0;k<p.count;++k)for(int a=0;a<3;++a) {
        const double j=p.jerk[k][a];
        if(j!=0) {
            const double t=-p.acceleration[k][a]/j;
            if(t>0&&t<p.step_sec) {
                const double v=p.velocity[k][a]+t*p.acceleration[k][a]+.5*t*t*j;
                require(std::abs(v)<=c.max_velocity_m_s+1e-7,"quadratic velocity extremum");
            }
        }
        const double endv=p.velocity[k][a]+p.step_sec*p.acceleration[k][a]+.5*p.step_sec*p.step_sec*j;
        require(std::abs(endv-p.velocity[k+1][a])<1e-12,"velocity splice");
        require(std::abs(p.acceleration[k][a]+p.step_sec*j-p.acceleration[k+1][a])<1e-12,"acceleration splice");
    }
    require(!p.sample(-.001,s)&&!p.sample(p.durationSec()+1e-7,s),"expired trajectory extrapolated");
}
void invalidAndTransaction() {
    auto c=config();
    for(double bad:{0.,-1.,std::numeric_limits<double>::quiet_NaN()}) {
        auto x=c;x.max_jerk_m_s3=bad;bool refused=false;
        try{VelocityPreview p(x);}catch(const std::invalid_argument&){refused=true;}
        require(refused,"bad config accepted");
    }
    VelocityPreview p(c);auto r=constant(V(.1,.02,-.03));
    require(p.plan(r,{}).valid,"good initial plan failed");const auto before=p.trajectory();
    r.knots[1].time_sec=0;require(!p.plan(r,{}).valid,"duplicate times accepted");
    r=constant(V::Zero());r.knots[1].time_sec=.239;require(!p.plan(r,{}).valid,"short known horizon extrapolated");
    r=constant(V::Zero());r.count=33;require(!p.plan(r,{}).valid,"oversized reference accepted");
    r=constant(V::Zero());VelocityPreviewState seed;seed.velocity.x()=.601;
    require(!p.plan(r,seed).valid,"seed clipped into bounds");
    // At the velocity limit, outward acceleration requires future overshoot.
    seed.velocity.x()=.6;seed.acceleration.x()=1;
    require(!p.plan(r,seed).valid,"physically infeasible seed accepted");
    // This state has enough physical braking distance (a^2/(2J)=15.625 um
    // versus 1 mm headroom), but the first Bernstein middle coefficient is
    // 0.60025 m/s. Record the conservative refusal rather than hiding it.
    seed.velocity.x()=.599;seed.acceleration.x()=.25;
    require(!p.plan(r,seed).valid,"conservative-bound test fixture changed");
    require(p.trajectory().valid,"failed plan erased prior immutable trajectory");
    for(std::size_t k=0;k<before.count;++k)
        require(p.trajectory().jerk[k]==before.jerk[k],"failed plan partially committed");
    p.clear();require(!p.trajectory().valid,"clear kept trajectory");
}
void steadyAndReversal() {
    const auto c=config();VelocityPreview p(c);VelocityPreviewState state;
    const auto r=constant(V(.02,0,0));double maxv=0;
    for(int tick=0;tick<100;++tick) {
        require(p.plan(r,state).valid,"steady rolling preview failed");
        certify(p.trajectory(),c);VelocityPreviewSample s;require(p.trajectory().sample(.02,s),"sample");
        state=s;maxv=std::max(maxv,s.velocity.x());
    }
    require(std::abs(state.velocity.x()-.02)<1e-8,"constant velocity not reproduced");
    std::cout<<"constant 20 mm/s intent: peak="<<maxv*1000<<" mm/s\n";
    require(maxv<.021,"constant intent amplified");
    for(const V& goal:{V(.6,-.5,.4),V(-.6,.5,-.4),V::Zero().eval()}) {
        const auto result=p.plan(constant(goal),state);
        if(!result.valid)std::cerr<<"reversal: "<<result.reason<<" residual="<<result.max_constraint_violation<<" iterations="<<result.working_set_recalculations<<'\n';
        require(result.valid,"reversal refused");certify(p.trajectory(),c);
        VelocityPreviewSample s;require(p.trajectory().sample(.04,s),"reversal sample");state=s;
    }
}
void futureAndCoordinates() {
    const auto c=config();VelocityPreview a(c),b(c);
    auto r=constant(V(.03,-.01,.015));r.count=4;
    r.knots[1]={.08,V(.03,-.01,.015)};r.knots[2]={.16,V(-.03,.02,.01)};r.knots[3]={.24,V::Zero()};
    const Eigen::Matrix3d R=Eigen::AngleAxisd(.7,V(1,2,-3).normalized()).toRotationMatrix();
    auto rotated=r;for(std::size_t k=0;k<r.count;++k)rotated.knots[k].velocity=R*r.knots[k].velocity;
    VelocityPreviewState seed;seed.velocity=V(.01,.005,0);
    VelocityPreviewState rotated_seed;rotated_seed.velocity=R*seed.velocity;
    require(a.plan(r,seed).valid&&b.plan(rotated,rotated_seed).valid,"known future failed");
    certify(a.trajectory(),c);certify(b.trajectory(),c);
    for(int i=0;i<=120;++i) {
        VelocityPreviewSample x,y;require(a.trajectory().sample(.002*i,x)&&b.trajectory().sample(.002*i,y),"rotated sample");
        require((R*x.velocity-y.velocity).norm()<1e-10,"unconstrained objective depends on axis choice");
    }
    VelocityPreview zero(c);auto z=constant(V::Zero());
    require(zero.plan(z,{}).valid,"zero reference failed");
    VelocityPreviewSample s;require(zero.trajectory().sample(.12,s)&&s.velocity.isZero(0),"zero intent generated travel");
    // Different future samples must affect the plan: this is a preview solve,
    // not a current-value filter accidentally labeled as preview.
    VelocityPreview current_only(c);require(current_only.plan(constant(r.knots[0].velocity),seed).valid,"constant plan");
    VelocityPreviewSample future,flat;
    a.trajectory().sample(.08,future);current_only.trajectory().sample(.08,flat);
    require((future.velocity-flat.velocity).norm()>.001,"known reversal not previewed");
}
}
int main(){try{invalidAndTransaction();steadyAndReversal();futureAndCoordinates();
    std::cout<<"velocity preview: all checks passed\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
