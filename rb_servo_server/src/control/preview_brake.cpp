#include "rb_servo/control/preview_brake.hpp"
#include "rb_servo/math/se3.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rb_servo::control {
namespace {
bool positive(double value) { return std::isfinite(value) && value > 0.; }
bool finitePose(const Pose6D& p) {
  return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z)&&
      std::isfinite(p.rx)&&std::isfinite(p.ry)&&std::isfinite(p.rz);
}

// Exact extrema on each constant-jerk interval. Velocity-mode Ruckig does not
// enforce velocity caps itself; certify them here, including interior v extrema.
bool boundedInterval(double t, double v, double a, double j,
                     double vmax, double amax, double jmax, double eps) {
  if (!std::isfinite(t)||t<0.||!std::isfinite(v)||!std::isfinite(a)||!std::isfinite(j)) return false;
  const auto bounded = [eps](double value,double limit) {
    return std::isfinite(value)&&std::abs(value)<=limit+eps;
  };
  const double vend=v+a*t+.5*j*t*t, aend=a+j*t;
  if (!bounded(v,vmax)||!bounded(vend,vmax)||!bounded(a,amax)||
      !bounded(aend,amax)||!bounded(j,jmax)) return false;
  if (j!=0.) {
    const double extremum=-a/j;
    if (extremum>0.&&extremum<t&&!bounded(v+a*extremum+.5*j*extremum*extremum,vmax)) return false;
  }
  return true;
}

double jerkAt(const ruckig::Profile& p,double time) {
  if (p.brake.duration>0.) {
    if (time<p.brake.duration) return p.brake.j[time<p.brake.t[0]?0:1];
    time-=p.brake.duration;
  }
  if (time>=p.t_sum.back()) return 0.;
  const auto it=std::upper_bound(p.t_sum.begin(),p.t_sum.end(),time);
  return it==p.t_sum.end()?0.:p.j[static_cast<std::size_t>(it-p.t_sum.begin())];
}
} // namespace

const char* previewBrakeStatusName(PreviewBrakeStatus status) {
  switch(status) {
    case PreviewBrakeStatus::Inactive:return "inactive";
    case PreviewBrakeStatus::Ready:return "ready";
    case PreviewBrakeStatus::InvalidInitialState:return "invalid_initial_state";
    case PreviewBrakeStatus::InitialOutsideLimits:return "initial_outside_limits";
    case PreviewBrakeStatus::Infeasible:return "infeasible";
    case PreviewBrakeStatus::LimitViolation:return "limit_violation";
    case PreviewBrakeStatus::NumericalFailure:return "numerical_failure";
  }
  return "unknown";
}

ruckig::PositionExtrema previewBrakeIntegratedPositionExtrema(ruckig::Profile profile) {
  profile.pf=profile.p.back();
  return profile.get_position_extrema();
}

double PreviewBrakeTrajectory::durationSec() const {return valid?trajectory.get_duration():0.;}
bool PreviewBrakeTrajectory::sample(double time,PreviewMotionSample& output) const {
  if (!valid||!std::isfinite(time)||time<0.||!angular_basis.allFinite()) return false;
  const double duration=trajectory.get_duration();
  if (!std::isfinite(duration)||duration<0.) return false;
  const double t=std::min(time,duration);
  std::array<double,6> p{},v{},a{},j{};
  trajectory.at_time(t,p,v,a);
  const auto profiles=trajectory.get_profiles();
  for(std::size_t axis=0;axis<6;++axis) j[axis]=t>=duration?0.:jerkAt(profiles[0][axis],t);
  // start() separately certifies the raw Ruckig terminal residual. Its declared
  // target v=a=0 defines the exact stationary hold; retain no floating-point
  // residue that would otherwise create a spurious omega x alpha jerk.
  if(t>=duration){v.fill(0.);a.fill(0.);}
  const Eigen::Vector3d theta=angular_basis*Eigen::Vector3d{p[3],p[4],p[5]};
  const Eigen::Matrix3d rotation=rotation0*math::exp3(theta);
  output.pose=math::poseFromSe3(pinocchio::SE3(rotation,Eigen::Vector3d{p[0],p[1],p[2]}+translation_offset));
  output.linear_velocity={v[0],v[1],v[2]};
  output.linear_acceleration={a[0],a[1],a[2]};
  output.linear_jerk={j[0],j[1],j[2]};
  previewAngularKinematics(theta,angular_basis*Eigen::Vector3d{v[3],v[4],v[5]},
      angular_basis*Eigen::Vector3d{a[3],a[4],a[5]},angular_basis*Eigen::Vector3d{j[3],j[4],j[5]},rotation,
      output.angular_velocity_body,output.angular_acceleration_body,output.angular_jerk_stand);
  return finitePose(output.pose)&&output.linear_velocity.allFinite()&&
      output.linear_acceleration.allFinite()&&output.linear_jerk.allFinite()&&
      output.angular_velocity_body.allFinite()&&output.angular_acceleration_body.allFinite()&&
      output.angular_jerk_stand.allFinite();
}
void PreviewBrakeTrajectory::shiftCommonFrame(const Eigen::Vector3d& translation,
                                             const Eigen::Quaterniond& rotation) {
  if (!translation.allFinite()||!rotation.coeffs().allFinite()||rotation.norm()<1e-12) {
    valid=false;return;
  }
  translation_offset+=translation;
  rotation0=rotation.normalized().toRotationMatrix()*rotation0;
}

void PreviewAngularContinuation::clear() {
  kind_=Kind::None;polynomial_.valid=false;brake_.valid=false;
  origin_sec_=original_valid_until_sec_=0.;
}
bool PreviewAngularContinuation::retainPolynomial(const PreviewPolynomialTrajectory& polynomial,
    double origin,double expiry) {
  clear();
  if(!polynomial.valid||polynomial.count<1||polynomial.count>PreviewPolynomialTrajectory::kMaxHorizonSteps||
     !positive(polynomial.step_sec)||!std::isfinite(origin)||origin<0.||!std::isfinite(expiry)||
     !std::isfinite(polynomial.durationSec())||!std::isfinite(origin+polynomial.durationSec())||
     expiry<=origin||expiry>origin+polynomial.durationSec()||
     !polynomial.rotation0.allFinite()||
     !polynomial.p.topRows(polynomial.count+1).allFinite()||
     !polynomial.v.topRows(polynomial.count+1).allFinite()||
     !polynomial.a.topRows(polynomial.count+1).allFinite()||
     !polynomial.jerk.topRows(polynomial.count).allFinite())return false;
  PreviewMotionSample checked;
  if(!polynomial.sample(0.,checked)||!polynomial.sample(
      std::clamp(expiry-origin,0.,polynomial.durationSec()),checked))return false;
  polynomial_=polynomial;origin_sec_=origin;original_valid_until_sec_=expiry;kind_=Kind::Polynomial;
  return true;
}
bool PreviewAngularContinuation::startBrake(const PreviewBrakeTrajectory& brake,double origin) {
  clear();
  if(!brake.valid||!std::isfinite(origin)||origin<0.||!std::isfinite(brake.durationSec())||
     brake.durationSec()<0.||!std::isfinite(origin+brake.durationSec()))return false;
  PreviewMotionSample checked;
  if(!brake.sample(0.,checked)||!brake.sample(brake.durationSec(),checked))return false;
  brake_=brake;origin_sec_=origin;kind_=Kind::Brake;return true;
}
bool PreviewAngularContinuation::needsBrake(double time) const {
  return kind_==Kind::Polynomial&&std::isfinite(time)&&time>=original_valid_until_sec_;
}
bool PreviewAngularContinuation::sample(double time,PreviewMotionSample& inout) const {
  if(!std::isfinite(time)||time<0.)return false;
  if(kind_==Kind::None)return true;
  if(time<origin_sec_)return false;
  PreviewMotionSample angular;
  if(kind_==Kind::Polynomial) {
    if(time>=original_valid_until_sec_||!polynomial_.sample(
        std::clamp(time-origin_sec_,0.,polynomial_.durationSec()),angular))return false;
  } else {
    // Use the same absolute endpoint predicate as terminalHoldAvailableAt.
    // Subtraction at a large host timestamp can otherwise round one ulp below
    // duration while the corresponding absolute deadline is already reached.
    const double elapsed=time>=origin_sec_+brake_.durationSec()?brake_.durationSec():time-origin_sec_;
    if(!brake_.sample(elapsed,angular))return false;
  }
  inout.pose.rx=angular.pose.rx;inout.pose.ry=angular.pose.ry;inout.pose.rz=angular.pose.rz;
  // The optional quaternion is canonical; copying only display RPY leaves the
  // translation helper's old orientation authoritative in rotationFromPose().
  inout.pose.quaternion_xyzw=angular.pose.quaternion_xyzw;
  inout.angular_velocity_body=angular.angular_velocity_body;
  inout.angular_acceleration_body=angular.angular_acceleration_body;
  inout.angular_jerk_stand=angular.angular_jerk_stand;
  return true;
}
bool PreviewAngularContinuation::terminalHoldAvailableAt(double time) const {
  if(!std::isfinite(time)||time<0.)return false;
  if(kind_==Kind::None)return true;
  return kind_==Kind::Brake&&brake_.valid&&time>=origin_sec_+brake_.durationSec();
}
void PreviewAngularContinuation::shiftCommonFrame(const Eigen::Vector3d& translation,
    const Eigen::Quaterniond& rotation) {
  if(kind_==Kind::None)return;
  if(!translation.allFinite()||!rotation.coeffs().allFinite()||rotation.norm()<1e-12) {
    polynomial_.valid=false;brake_.valid=false;return;
  }
  if(kind_==Kind::Polynomial) {
    // Translation is deliberately ignored: this descriptor never supplies it.
    // Body angular derivatives are invariant under a common left rotation;
    // the shared sampler rotates physical stand jerk through rotation0.
    polynomial_.rotation0=rotation.normalized().toRotationMatrix()*polynomial_.rotation0;
  } else brake_.shiftCommonFrame(Eigen::Vector3d::Zero(),rotation);
}

PreviewBrake::PreviewBrake(const PreviewTrackerConfig& config,double servo_period_sec)
    :config_(config),calculator_(servo_period_sec) {
  if (!positive(servo_period_sec)||!positive(config.max_linear_velocity_m_s)||
      !positive(config.max_linear_acceleration_m_s2)||!positive(config.max_linear_jerk_m_s3)||
      !positive(config.max_angular_velocity_rad_s)||!positive(config.max_angular_acceleration_rad_s2)||
      !positive(config.max_angular_jerk_rad_s3)||!positive(config.feasibility_tolerance)||
      config.feasibility_tolerance>1e-4||!positive(config.max_reference_chart_angle_rad)||
      config.max_reference_chart_angle_rad>1.4) throw std::invalid_argument("Invalid preview brake limits");
  const double V=config.max_angular_velocity_rad_s;
  const double A=config.max_angular_acceleration_rad_s2-.5*V*V;
  const double J=config.max_angular_jerk_rad_s3-2.5*V*A-(5./6.)*V*V*V;
  if (!positive(A)||!positive(J)) throw std::invalid_argument("No conservative angular brake budget");
  input_.control_interface=ruckig::ControlInterface::Velocity;
  input_.current_position.fill(0.);
  input_.target_position.fill(0.);
  // Each axis stops as soon as feasible; completed axes explicitly hold while
  // the remaining axes finish. Synchronization must not stretch a contact stop.
  input_.synchronization=ruckig::Synchronization::None;
  for(std::size_t i=0;i<6;++i) {
    input_.max_velocity[i]=i<3?config.max_linear_velocity_m_s:V/std::sqrt(3.);
    input_.max_acceleration[i]=i<3?config.max_linear_acceleration_m_s2:A/std::sqrt(3.);
    input_.max_jerk[i]=i<3?config.max_linear_jerk_m_s3:J/std::sqrt(3.);
    input_.target_velocity[i]=0.;input_.target_acceleration[i]=0.;
  }
  // Warm the same fixed-size velocity calculation before any servo use.
  ruckig::Trajectory<6> warm;
  input_.current_velocity.fill(.001);
  if (calculator_.calculate(input_,warm)<ruckig::Result::Working)
    throw std::runtime_error("Preview brake prewarm failed");
  input_.current_velocity.fill(0.);
}

PreviewBrakeStatus PreviewBrake::start(const PreviewMotionState& initial) {
  accepted_.valid=false;
  const auto reject=[this](PreviewBrakeStatus s){status_=s;return s;};
  if (!finitePose(initial.pose)||!initial.linear_velocity.allFinite()||
      !initial.linear_acceleration.allFinite()||!initial.angular_velocity_body.allFinite()||
      !initial.angular_acceleration_body.allFinite()) return reject(PreviewBrakeStatus::InvalidInitialState);
  // Per-axis numerical tolerance must not multiply into a larger accepted
  // physical norm at the cube corners. Use the existing physical norm budget.
  if(initial.angular_velocity_body.norm()>config_.max_angular_velocity_rad_s+config_.feasibility_tolerance||
     initial.angular_acceleration_body.norm()>config_.max_angular_acceleration_rad_s2+config_.feasibility_tolerance)
    return reject(PreviewBrakeStatus::InitialOutsideLimits);
  input_.current_position={initial.pose.x,initial.pose.y,initial.pose.z,0.,0.,0.};
  for(std::size_t i=0;i<6;++i) {
    input_.current_velocity[i]=i<3?initial.linear_velocity[i]:initial.angular_velocity_body[i-3];
    input_.current_acceleration[i]=i<3?initial.linear_acceleration[i]:initial.angular_acceleration_body[i-3];
  }
  const auto inside_axis=[&](std::size_t i) {
    const double v=input_.current_velocity[i],a=input_.current_acceleration[i];
    const double eps=config_.feasibility_tolerance;
    // A nonzero initial acceleration can unavoidably raise speed before jerk
    // brings acceleration to zero; reject this case without changing the seed.
    const double unavoidable=v+std::copysign(a*a/(2.*input_.max_jerk[i]),a);
    return std::isfinite(v)&&std::isfinite(a)&&std::isfinite(unavoidable)&&
        std::abs(v)<=input_.max_velocity[i]+eps&&std::abs(a)<=input_.max_acceleration[i]+eps&&
        std::abs(unavoidable)<=input_.max_velocity[i]+eps;
  };
  for(std::size_t i=0;i<3;++i)if(!inside_axis(i))return reject(PreviewBrakeStatus::InitialOutsideLimits);
  Eigen::Matrix3d angular_basis=Eigen::Matrix3d::Identity();
  if(!inside_axis(3)||!inside_axis(4)||!inside_axis(5)) {
    // A trajectory certified in an earlier chart can have a body-omega
    // component just outside the recentered inscribed cube while its physical
    // norm is well inside the unchanged cap. Rotate the CHART, never the state
    // or a hardware limit: B^T*omega is balanced along the cube's diagonal.
    const double norm=initial.angular_velocity_body.norm();
    if(!std::isfinite(norm)||norm<=0.)return reject(PreviewBrakeStatus::InitialOutsideLimits);
    const Eigen::Quaterniond to_balanced=Eigen::Quaterniond::FromTwoVectors(
        initial.angular_velocity_body/norm,Eigen::Vector3d::Ones()/std::sqrt(3.));
    angular_basis=to_balanced.toRotationMatrix().transpose();
    if(!angular_basis.allFinite()||
       (angular_basis.transpose()*angular_basis-Eigen::Matrix3d::Identity()).norm()>config_.feasibility_tolerance||
       std::abs(angular_basis.determinant()-1.)>config_.feasibility_tolerance)
      return reject(PreviewBrakeStatus::NumericalFailure);
    const Eigen::Vector3d v=angular_basis.transpose()*initial.angular_velocity_body;
    const Eigen::Vector3d a=angular_basis.transpose()*initial.angular_acceleration_body;
    for(std::size_t i=0;i<3;++i){input_.current_velocity[i+3]=v[i];input_.current_acceleration[i+3]=a[i];}
    if(!inside_axis(3)||!inside_axis(4)||!inside_axis(5))
      return reject(PreviewBrakeStatus::InitialOutsideLimits);
  }
  PreviewBrakeTrajectory candidate;
  candidate.angular_basis=angular_basis;
  const auto result=calculator_.calculate(input_,candidate.trajectory);
  if (result<ruckig::Result::Working) return reject(PreviewBrakeStatus::Infeasible);
  const double duration=candidate.trajectory.get_duration();
  if (!std::isfinite(duration)||duration<0.) return reject(PreviewBrakeStatus::NumericalFailure);
  const auto profiles=candidate.trajectory.get_profiles();
  for(std::size_t i=0;i<6;++i) {
    const auto& p=profiles[0][i];
    if (p.brake.duration>0.) for(std::size_t k=0;k<2;++k) {
      if (p.brake.t[k]>0.&&!boundedInterval(p.brake.t[k],p.brake.v[k],p.brake.a[k],p.brake.j[k],
          input_.max_velocity[i],input_.max_acceleration[i],input_.max_jerk[i],config_.feasibility_tolerance))
        return reject(PreviewBrakeStatus::LimitViolation);
    }
    for(std::size_t k=0;k<7;++k) {
      if (!boundedInterval(p.t[k],p.v[k],p.a[k],p.j[k],input_.max_velocity[i],
          input_.max_acceleration[i],input_.max_jerk[i],config_.feasibility_tolerance))
        return reject(PreviewBrakeStatus::LimitViolation);
    }
  }
  Eigen::Vector3d angular_extent;
  for(std::size_t i=0;i<3;++i) {
    const auto extent=previewBrakeIntegratedPositionExtrema(profiles[0][i+3]);
    angular_extent[i]=std::max(std::abs(extent.min),std::abs(extent.max));
  }
  if (!angular_extent.allFinite()||angular_extent.norm()>config_.max_reference_chart_angle_rad)
    return reject(PreviewBrakeStatus::LimitViolation);
  std::array<double,6> terminal_position{},terminal_velocity{},terminal_acceleration{};
  candidate.trajectory.at_time(duration,terminal_position,terminal_velocity,terminal_acceleration);
  double velocity_residual2=0.,acceleration_residual2=0.;
  for(std::size_t i=0;i<6;++i){
    if(!std::isfinite(terminal_position[i])||!std::isfinite(terminal_velocity[i])||
        !std::isfinite(terminal_acceleration[i]))return reject(PreviewBrakeStatus::NumericalFailure);
    velocity_residual2+=terminal_velocity[i]*terminal_velocity[i];
    acceleration_residual2+=terminal_acceleration[i]*terminal_acceleration[i];
  }
  if(std::sqrt(velocity_residual2)>config_.feasibility_tolerance||
      std::sqrt(acceleration_residual2)>config_.feasibility_tolerance)
    return reject(PreviewBrakeStatus::NumericalFailure);
  candidate.rotation0=math::rotationFromPose(initial.pose);
  candidate.valid=true;
  PreviewMotionSample terminal;
  if (!candidate.sample(duration,terminal)||terminal.linear_velocity.norm()>config_.feasibility_tolerance||
      terminal.linear_acceleration.norm()>config_.feasibility_tolerance||
      terminal.angular_velocity_body.norm()>config_.feasibility_tolerance||
      terminal.angular_acceleration_body.norm()>config_.feasibility_tolerance)
    return reject(PreviewBrakeStatus::NumericalFailure);
  accepted_=candidate;status_=PreviewBrakeStatus::Ready;return status_;
}
void PreviewBrake::reset(){accepted_.valid=false;status_=PreviewBrakeStatus::Inactive;}
bool PreviewBrake::sample(double time,PreviewMotionSample& output)const{return accepted_.sample(time,output);}
bool PreviewBrake::exportTrajectory(PreviewBrakeTrajectory& output)const{output=accepted_;return output.valid;}
double PreviewBrake::durationSec()const{return accepted_.durationSec();}
} // namespace rb_servo::control
