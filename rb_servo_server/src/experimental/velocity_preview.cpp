#include "rb_servo/experimental/velocity_preview.hpp"
#include <Eigen/Cholesky>
#include <qpOASES.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace rb_servo::experimental {
namespace {
using Matrix=Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>;
using Vector=Eigen::VectorXd;
bool positive(double x) {return std::isfinite(x)&&x>0;}
bool nonnegative(double x) {return std::isfinite(x)&&x>=0;}
Eigen::Vector3d target(const VelocityPreviewReference& r,double t) {
    std::size_t hi=1;
    while(hi+1<r.count&&r.knots[hi].time_sec<t)++hi;
    const auto& a=r.knots[hi-1];const auto& b=r.knots[hi];
    const double u=std::clamp((t-a.time_sec)/(b.time_sec-a.time_sec),0.,1.);
    return (1-u)*a.velocity+u*b.velocity;
}
}

bool VelocityPreviewTrajectory::sample(double t,VelocityPreviewSample& out) const {
    if(!valid||count<1||count>kCapacity||!positive(step_sec)||!std::isfinite(t)||
       t<0||t>durationSec())return false;
    const std::size_t k=std::min(static_cast<std::size_t>(t/step_sec),count-1);
    const double s=t-k*step_sec;
    out.velocity=velocity[k]+s*acceleration[k]+.5*s*s*jerk[k];
    out.acceleration=acceleration[k]+s*jerk[k];out.jerk=jerk[k];
    return out.velocity.allFinite()&&out.acceleration.allFinite()&&out.jerk.allFinite();
}

struct VelocityPreview::Impl {
    VelocityPreviewConfig c;
    int n;
    Matrix B,A,C,H;
    Vector g,lower,upper,lb,ub,solution;
    Eigen::LLT<Matrix> factor;
    std::unique_ptr<qpOASES::QProblem> qp;
    VelocityPreviewTrajectory accepted;
    explicit Impl(const VelocityPreviewConfig& config):c(config),n(static_cast<int>(c.horizon_steps)) {
        if(c.horizon_steps<1||c.horizon_steps>VelocityPreviewTrajectory::kCapacity||
           !positive(c.step_sec)||!std::isfinite(n*c.step_sec)||
           !positive(c.max_velocity_m_s)||!positive(c.max_acceleration_m_s2)||
           !positive(c.max_jerk_m_s3)||!positive(c.tracking_scale_m_s)||
           !positive(c.acceleration_weight)||!positive(c.jerk_weight)||!nonnegative(c.jerk_difference_weight)||
           !positive(c.feasibility_tolerance)||c.feasibility_tolerance>1e-4||
           c.max_working_set_recalculations<1||c.max_working_set_recalculations>1000||
           !positive(c.max_solve_time_sec))throw std::invalid_argument("invalid velocity preview config");
        const double h=c.step_sec,j=c.max_jerk_m_s3;
        B=Matrix::Zero(n,n);A=Matrix::Zero(n,n);C=Matrix::Zero(3*n,n);
        // Normalized jerk decision, -1 <= u <= 1. The quadratic velocity
        // interval has Bernstein coefficients v_k, v_k+h*a_k/2, v_(k+1).
        // Bounding those coefficients certifies the entire interval; merely
        // constraining endpoints misses an interior velocity maximum.
        for(int k=0;k<n;++k)for(int l=0;l<=k;++l) {
            const double t1=(k+1-l)*h,t0=(k-l)*h;
            B(k,l)=.5*j*(t1*t1-t0*t0)/c.tracking_scale_m_s;
            C(3*k+1,l)=B(k,l)*c.tracking_scale_m_s/c.max_velocity_m_s;
            C(3*k+2,l)=j*h/c.max_acceleration_m_s2;
            A(k,l)=C(3*k+2,l);
            if(l<k)C(3*k,l)=(.5*j*(t0*t0-(t0-h)*(t0-h))+.5*j*h*h)/c.max_velocity_m_s;
        }
        H=2.*B.transpose()*B+2.*c.acceleration_weight*A.transpose()*A;
        H.diagonal().array()+=2.*c.jerk_weight;
        for(int k=1;k<n;++k) {
            H(k,k)+=2*c.jerk_difference_weight;H(k-1,k-1)+=2*c.jerk_difference_weight;
            H(k,k-1)-=2*c.jerk_difference_weight;H(k-1,k)-=2*c.jerk_difference_weight;
        }
        if(!H.allFinite()||!C.allFinite())throw std::invalid_argument("velocity preview scale overflow");
        factor.compute(H);
        if(factor.info()!=Eigen::Success)throw std::invalid_argument("velocity preview objective singular");
        g=Vector::Zero(n);lower=Vector::Constant(n,-1);upper=Vector::Constant(n,1);
        lb=Vector::Zero(3*n);ub=Vector::Zero(3*n);solution=Vector::Zero(n);
        qp=std::make_unique<qpOASES::QProblem>(n,3*n);
        qpOASES::Options options;options.setToReliable();options.printLevel=qpOASES::PL_NONE;
        qp->setOptions(options);
    }
};
VelocityPreview::VelocityPreview(const VelocityPreviewConfig& c):impl_(std::make_unique<Impl>(c)){}
VelocityPreview::~VelocityPreview()=default;
const VelocityPreviewTrajectory& VelocityPreview::trajectory() const{return impl_->accepted;}
void VelocityPreview::clear(){impl_->accepted={};}

VelocityPreviewResult VelocityPreview::plan(const VelocityPreviewReference& ref,const VelocityPreviewState& initial) {
    const auto start=std::chrono::steady_clock::now();
    VelocityPreviewResult out;
    auto finish=[&](const char* reason) {
        out.reason=reason;out.solve_time_sec=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
        return out;
    };
    auto& p=*impl_;const auto& c=p.c;const double h=c.step_sec;const int n=p.n;
    if(ref.count<2||ref.count>ref.kCapacity||ref.knots[0].time_sec!=0||
       ref.knots[ref.count-1].time_sec<n*h)return finish("invalid_reference");
    for(std::size_t k=0;k<ref.count;++k)if(!ref.knots[k].velocity.allFinite()||
       !std::isfinite(ref.knots[k].time_sec)||(k&&ref.knots[k].time_sec<=ref.knots[k-1].time_sec))return finish("invalid_reference");
    if(!initial.velocity.allFinite()||!initial.acceleration.allFinite()||
       initial.velocity.cwiseAbs().maxCoeff()>c.max_velocity_m_s*(1+c.feasibility_tolerance)||
       initial.acceleration.cwiseAbs().maxCoeff()>c.max_acceleration_m_s2*(1+c.feasibility_tolerance))return finish("invalid_initial_state");
    VelocityPreviewTrajectory trial;trial.step_sec=h;trial.count=n;
    trial.velocity[0]=initial.velocity;trial.acceleration[0]=initial.acceleration;
    for(int axis=0;axis<3;++axis) {
        Vector error(n);
        for(int k=0;k<n;++k) {
            const double t=(k+1)*h;
            error[k]=(initial.velocity[axis]+t*initial.acceleration[axis]-target(ref,t)[axis])/c.tracking_scale_m_s;
            const double base[]={initial.velocity[axis]+(k+.5)*h*initial.acceleration[axis],
                                 initial.velocity[axis]+t*initial.acceleration[axis],initial.acceleration[axis]};
            for(int r=0;r<3;++r) {
                const double scale=r==2?c.max_acceleration_m_s2:c.max_velocity_m_s;
                p.lb[3*k+r]=-1-base[r]/scale;p.ub[3*k+r]=1-base[r]/scale;
            }
        }
        p.g=2.*p.B.transpose()*error+2.*c.acceleration_weight*p.A.transpose()*
            Vector::Constant(n,initial.acceleration[axis]/c.max_acceleration_m_s2);
        p.solution=p.factor.solve(-p.g);
        if(!p.g.allFinite()||!p.solution.allFinite())return finish("numerical_failure");
        auto violation=[&]() {
            const Vector y=p.C*p.solution;
            return std::max({0.,(p.solution-p.upper).maxCoeff(),(p.lower-p.solution).maxCoeff(),
                             (y-p.ub).maxCoeff(),(p.lb-y).maxCoeff()});
        };
        if(violation()>c.feasibility_tolerance) {
            p.qp->reset();int iterations=c.max_working_set_recalculations;
            qpOASES::real_t seconds=c.max_solve_time_sec-
                std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
            if(seconds<=0)return finish("time_budget_exceeded");
            const auto code=p.qp->init(p.H.data(),p.g.data(),p.C.data(),p.lower.data(),p.upper.data(),
                                      p.lb.data(),p.ub.data(),iterations,&seconds);
            out.working_set_recalculations+=iterations;
            if(code!=qpOASES::SUCCESSFUL_RETURN)return finish("qp_refused");
            if(p.qp->getPrimalSolution(p.solution.data())!=qpOASES::SUCCESSFUL_RETURN)return finish("numerical_failure");
        }
        const double residual=violation();
        if(!p.solution.allFinite()||!std::isfinite(residual))return finish("numerical_failure");
        out.max_constraint_violation=std::max(out.max_constraint_violation,residual);
        if(residual>c.feasibility_tolerance)return finish("constraint_violation");
        for(int k=0;k<n;++k) {
            const double j=p.solution[k]*c.max_jerk_m_s3;
            trial.jerk[k][axis]=j;
            trial.velocity[k+1][axis]=trial.velocity[k][axis]+h*trial.acceleration[k][axis]+.5*h*h*j;
            trial.acceleration[k+1][axis]=trial.acceleration[k][axis]+h*j;
        }
    }
    finish("solved");
    if(out.solve_time_sec>c.max_solve_time_sec)return finish("time_budget_exceeded");
    trial.valid=true;p.accepted=trial;out.valid=true;
    return out;
}
} // namespace rb_servo::experimental
