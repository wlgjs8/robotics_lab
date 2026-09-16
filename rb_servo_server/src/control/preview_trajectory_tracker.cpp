#include "rb_servo/control/preview_trajectory_tracker.hpp"
#include "rb_servo/math/se3.hpp"

#include <unsupported/Eigen/AutoDiff>
#include <Eigen/Cholesky>
#include <pinocchio/spatial/explog.hpp>
#include <qpOASES.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace pinocchio {
template <>
struct TaylorSeriesExpansion<Eigen::AutoDiffScalar<Eigen::Matrix<double, 1, 1>>> {
  template <int degree>
  static auto precision() {
    return Eigen::AutoDiffScalar<Eigen::Matrix<double, 1, 1>>(
        TaylorSeriesExpansion<double>::precision<degree>());
  }
};
template <>
struct TaylorSeriesExpansion<Eigen::AutoDiffScalar<Eigen::Matrix<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, 1, 1>>, 1, 1>>> {
  template <int degree>
  static auto precision() {
    using Inner = Eigen::AutoDiffScalar<Eigen::Matrix<double, 1, 1>>;
    using Outer = Eigen::AutoDiffScalar<Eigen::Matrix<Inner, 1, 1>>;
    return Outer(Inner(TaylorSeriesExpansion<double>::precision<degree>()));
  }
};
} // namespace pinocchio

namespace rb_servo::control {
namespace {
using Clock = std::chrono::steady_clock;
using Matrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using Vector = Eigen::VectorXd;
using Axes = Eigen::Matrix<double, 6, 1>;
constexpr int kCapacity = static_cast<int>(PreviewTrajectoryTracker::kMaxHorizonSteps);
// Fixed worker-owned storage for angular norm support planes. Exhaustion
// rejects the candidate; it never discards a plane or enlarges a norm ball.
constexpr int kAngularCutsPerInterval = 32;
constexpr int kMaxContactRows = 3 * (static_cast<int>(PreviewContactConstraint::kCapacity) - 1 + kCapacity);

bool positive(double x) { return std::isfinite(x) && x > 0.0; }
bool nonnegative(double x) { return std::isfinite(x) && x >= 0.0; }
Eigen::Vector3d position(const Pose6D& p) { return {p.x, p.y, p.z}; }
bool validPose(const Pose6D& p) {
  if (!position(p).allFinite() || !std::isfinite(p.rx) ||
      !std::isfinite(p.ry) || !std::isfinite(p.rz)) return false;
  try { return math::rotationFromPose(p).allFinite(); }
  catch (const std::exception&) { return false; }
}

// Pinocchio owns exp/Jexp; nested fixed-size Eigen autodiff obtains the first
// and second time derivatives of its Jacobian, including small-angle branches.
// omega/alpha are body components. Physical stand jerk additionally contains
// R * (omega_body x alpha_body); alpha_body_dot alone is not stand jerk.
void angularKinematics(const Eigen::Vector3d& p, const Eigen::Vector3d& v,
                       const Eigen::Vector3d& a, const Eigen::Vector3d& j,
                       const Eigen::Matrix3d& rotation,
                       Eigen::Vector3d& omega, Eigen::Vector3d& alpha,
                       Eigen::Vector3d& jerk_stand) {
  using Inner = Eigen::AutoDiffScalar<Eigen::Matrix<double, 1, 1>>;
  using Outer = Eigen::AutoDiffScalar<Eigen::Matrix<Inner, 1, 1>>;
  Eigen::Matrix<Outer, 3, 1> argument;
  for (int i = 0; i < 3; ++i) {
    argument[i].value().value() = p[i];
    argument[i].value().derivatives()[0] = v[i];
    argument[i].derivatives()[0].value() = v[i];
    argument[i].derivatives()[0].derivatives()[0] = a[i];
  }
  Eigen::Matrix<Outer, 3, 3> result;
  pinocchio::Jexp3(argument, result);
  Eigen::Matrix3d J, Jdot, Jddot;
  for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) {
    J(r,c) = result(r,c).value().value();
    Jdot(r,c) = result(r,c).value().derivatives()[0];
    Jddot(r,c) = result(r,c).derivatives()[0].derivatives()[0];
  }
  omega = J * v;
  alpha = J * a + Jdot * v;
  const Eigen::Vector3d alpha_dot_body = J*j + 2.0*Jdot*a + Jddot*v;
  jerk_stand = rotation * (alpha_dot_body + omega.cross(alpha));
}

using Trajectory = PreviewPolynomialTrajectory;

bool evaluate(const Trajectory& trajectory, int count, double h, double time,
              PreviewMotionSample& out) {
  if (!trajectory.valid || count < 1 || count > kCapacity || !positive(h) ||
      !std::isfinite(time) || time < 0.0 || time > count*h)
    return false;
  const int k = std::min(static_cast<int>(time/h), count-1);
  const double s = time-k*h;
  const Axes j = trajectory.jerk.row(k).transpose();
  const Axes a = trajectory.a.row(k).transpose() + j*s;
  const Axes v = trajectory.v.row(k).transpose() + trajectory.a.row(k).transpose()*s + j*(.5*s*s);
  const Axes p = trajectory.p.row(k).transpose() + trajectory.v.row(k).transpose()*s +
      trajectory.a.row(k).transpose()*(.5*s*s) + j*(s*s*s/6.0);
  const Eigen::Matrix3d rotation = trajectory.rotation0 * math::exp3(p.tail<3>());
  out.pose = math::poseFromSe3(pinocchio::SE3(rotation, p.head<3>()));
  out.linear_velocity = v.head<3>(); out.linear_acceleration = a.head<3>();
  out.linear_jerk = j.head<3>();
  angularKinematics(p.tail<3>(), v.tail<3>(), a.tail<3>(), j.tail<3>(), rotation,
                    out.angular_velocity_body, out.angular_acceleration_body,
                    out.angular_jerk_stand);
  return position(out.pose).allFinite() && out.linear_velocity.allFinite() &&
      out.linear_acceleration.allFinite() && out.angular_velocity_body.allFinite() &&
      out.angular_acceleration_body.allFinite() && out.angular_jerk_stand.allFinite();
}

Pose6D referenceAt(const PreviewReference& ref, double time) {
  std::size_t hi = 1;
  while (hi + 1 < ref.count && ref.knots[hi].time_sec < time) ++hi;
  const auto& lo = ref.knots[hi-1]; const auto& up = ref.knots[hi];
  const double u = std::clamp((time-lo.time_sec)/(up.time_sec-lo.time_sec), 0.0, 1.0);
  return math::interpolateLinear(lo.pose, up.pose, true, u);
}
double contactBoundAt(const PreviewContactConstraint& contact, double time) {
  std::size_t hi = 1;
  while (hi + 1 < contact.count && contact.knots[hi].time_sec < time) ++hi;
  const auto& a = contact.knots[hi-1]; const auto& b = contact.knots[hi];
  const double u = std::clamp((time-a.time_sec)/(b.time_sec-a.time_sec), 0.0, 1.0);
  return (1.0-u)*a.upper_velocity_m_s + u*b.upper_velocity_m_s;
}
} // namespace

void previewAngularKinematics(const Eigen::Vector3d& p, const Eigen::Vector3d& v,
                              const Eigen::Vector3d& a, const Eigen::Vector3d& j,
                              const Eigen::Matrix3d& rotation,
                              Eigen::Vector3d& omega, Eigen::Vector3d& alpha,
                              Eigen::Vector3d& jerk_stand) {
  angularKinematics(p, v, a, j, rotation, omega, alpha, jerk_stand);
}

struct PreviewTrajectoryTracker::Impl {
  struct AxisQp {
    Matrix H, C, prediction;
    Vector g, lower, upper, lower_c, upper_c, solution;
    std::unique_ptr<qpOASES::QProblem> qp;
    bool warm{false};
  };
  struct ContactQp {
    Matrix H, C, previous_C, full_C;
    Vector g, lower, upper, lower_c, upper_c, solution, row_scale, full_upper, full_lower;
    std::array<int,kCapacity> endpoint_rows{};
    std::array<int,kMaxContactRows> row_interval{};
    std::unique_ptr<qpOASES::SQProblem> qp;
    bool warm{false};
  };
  PreviewTrackerConfig cfg;
  int n;
  std::array<AxisQp, 6> axes;
  ContactQp contact;
  ContactQp normal_contact;
  std::vector<ContactQp> angular_norm_pool;
  Matrix angular_norm_cuts;
  Vector angular_norm_upper;
  double angular_objective_scale{1.0};
  // e-folding time of the raw reference's share past the committed window (see
  // PreviewTrackerConfig). Objective-neutral: it moves the target, never the Hessian.
  double reference_trust_tau{0.0};
  Eigen::LLT<Matrix> translation_factor;
  Matrix unconstrained_translation;
  Axes velocity_limit, acceleration_limit, jerk_limit;
  Trajectory trajectory;

  explicit Impl(const PreviewTrackerConfig& input) : cfg(input), n(static_cast<int>(input.horizon_steps)) {
    if (!positive(cfg.planning_dt_sec) || input.horizon_steps < 1 ||
        input.horizon_steps > static_cast<std::size_t>(kCapacity) ||
        n*cfg.planning_dt_sec < .2-1e-10 || n*cfg.planning_dt_sec > .3+1e-10 ||
        !positive(cfg.max_linear_velocity_m_s) || !positive(cfg.max_linear_acceleration_m_s2) ||
        !positive(cfg.max_linear_jerk_m_s3) || !positive(cfg.max_angular_velocity_rad_s) ||
        !positive(cfg.max_angular_acceleration_rad_s2) || !positive(cfg.max_angular_jerk_rad_s3) ||
        !positive(cfg.linear_tracking_scale_m) || !positive(cfg.angular_tracking_scale_rad) ||
        !positive(cfg.jerk_weight) || !nonnegative(cfg.jerk_difference_weight) ||
        !nonnegative(cfg.linear_tracking_tolerance_m) || !nonnegative(cfg.angular_tracking_tolerance_rad) ||
        !nonnegative(cfg.max_linear_tracking_slack_m) || !nonnegative(cfg.max_angular_tracking_slack_rad) ||
        !positive(cfg.max_reference_chart_angle_rad) || cfg.max_reference_chart_angle_rad > 1.4 ||
        !positive(cfg.feasibility_tolerance) || cfg.feasibility_tolerance > 1e-4 ||
        cfg.max_working_set_recalculations < 1 || cfg.max_working_set_recalculations > 1000 ||
        !nonnegative(cfg.reference_trust_full_sec) ||
        !(cfg.reference_trust_tail_sec > cfg.reference_trust_full_sec) ||
        !std::isfinite(cfg.reference_trust_tail_sec) ||
        !positive(cfg.reference_trust_tail) || cfg.reference_trust_tail >= 1.0 ||
        !positive(cfg.max_solve_time_sec)) throw std::invalid_argument("Invalid preview tracker configuration");
    const double V = cfg.max_angular_velocity_rad_s;
    // Jr = integral_0^1 exp(-s*phi) ds gives ||Jr||<=1,
    // ||Jr_dot||<=V/2, ||Jr_ddot||<=A/2+V²/3. Reserve the additional
    // omega x alpha term when limiting physical stand angular jerk.
    const double A = cfg.max_angular_acceleration_rad_s2 - .5*V*V;
    const double J = cfg.max_angular_jerk_rad_s3 - 2.5*V*A - (5.0/6.0)*V*V*V;
    if (!positive(A) || !positive(J))
      throw std::invalid_argument("Angular physical norm limits leave no conservative tangent budget");
    velocity_limit.head<3>().setConstant(cfg.max_linear_velocity_m_s);
    acceleration_limit.head<3>().setConstant(cfg.max_linear_acceleration_m_s2);
    jerk_limit.head<3>().setConstant(cfg.max_linear_jerk_m_s3);
    // These are outer boxes only. The joint three-axis Bernstein norm
    // certificate below enforces V and A continuously. An inscribed cube
    // incorrectly limited a dominant-axis rotation to V/sqrt(3), and a chart
    // rebase could reject an already accepted physical angular velocity.
    velocity_limit.tail<3>().setConstant(V);
    acceleration_limit.tail<3>().setConstant(A);
    // Keep the original jerk normalization and regularization objective.
    // This inscribed jerk cube still certifies the same chart jerk norm J.
    jerk_limit.tail<3>().setConstant(J/std::sqrt(3.0));
    const double h = cfg.planning_dt_sec;
    // w(t) = 1 for t <= full, exp(-(t-full)/tau) after, with tau fixed by the
    // (tail_sec, tail) pair.
    reference_trust_tau=(cfg.reference_trust_tail_sec-cfg.reference_trust_full_sec)/
        std::log(1.0/cfg.reference_trust_tail);
    for (int axis = 0; axis < 6; ++axis) {
      auto& q = axes[axis];
      q.prediction = Matrix::Zero(n,n); q.C = Matrix::Zero(3*n,n);
      q.g = Vector::Zero(n); q.lower = Vector::Constant(n,-1.0); q.upper = Vector::Constant(n,1.0);
      q.lower_c = Vector::Zero(3*n); q.upper_c = Vector::Zero(3*n); q.solution = Vector::Zero(n);
      const double scale = axis < 3 ? cfg.linear_tracking_scale_m : cfg.angular_tracking_scale_rad;
      for (int k = 0; k < n; ++k) for (int l = 0; l <= k; ++l) {
        const double t1=(k+1-l)*h, t0=(k-l)*h, jm=jerk_limit[axis];
        q.prediction(k,l)=jm*(t1*t1*t1-t0*t0*t0)/(6.0*scale);
        q.C(3*k+1,l)=jm*.5*(t1*t1-t0*t0); // interval-end velocity
        q.C(3*k+2,l)=jm*h;                 // interval-end acceleration
        if (l < k) {
          const double b1=(k-l)*h, b0=(k-l-1)*h;
          q.C(3*k,l)=jm*(.5*(b1*b1-b0*b0)+.5*h*h);
        } // middle Bernstein velocity control = v_k + h*a_k/2
      }
      q.H = 2.0*q.prediction.transpose()*q.prediction;
      q.H.diagonal().array() += 2.0*cfg.jerk_weight;
      for (int k=1;k<n;++k) {
        const double w=2.0*cfg.jerk_difference_weight;
        q.H(k,k)+=w; q.H(k-1,k-1)+=w; q.H(k,k-1)-=w; q.H(k-1,k)-=w;
      }
      bounds(axis,0.0,0.0);
      q.qp=std::make_unique<qpOASES::QProblem>(n,3*n,qpOASES::HST_POSDEF);
      qpOASES::Options options; options.setToMPC(); options.printLevel=qpOASES::PL_NONE;
      options.terminationTolerance=1e-10; options.boundTolerance=1e-10;
      q.qp->setOptions(options);
      int iterations=1000; double startup_budget=1.0;
      const auto status=q.qp->init(q.H.data(),q.g.data(),q.C.data(),q.lower.data(),q.upper.data(),
                                   q.lower_c.data(),q.upper_c.data(),iterations,&startup_budget);
      if (status!=qpOASES::SUCCESSFUL_RETURN) throw std::runtime_error("Preview QP prewarm failed");
      q.warm=true;
    }
    // One coupled translation solve is needed for an arbitrary contact normal.
    // Off-contact plans still use the exact existing independent-axis lane.
    auto& q = contact;
    // A bounded cutting-plane set avoids a degenerate active set containing
    // hundreds of equivalent flat-contact rows. Every candidate is checked
    // against ALL original 500-Hz Bernstein rows before acceptance.
    const int variables = 3*n, constraints = 12*n;
    q.H = Matrix::Zero(variables,variables);
    q.C = Matrix::Zero(constraints,variables);
    q.full_C=Matrix::Zero(kMaxContactRows,variables);
    q.full_upper=Vector::Constant(kMaxContactRows,qpOASES::INFTY);
    q.full_lower=Vector::Constant(kMaxContactRows,-qpOASES::INFTY);
    q.g = Vector::Zero(variables); q.solution = Vector::Zero(variables);
    q.row_scale=Vector::Ones(kMaxContactRows);
    q.lower = Vector::Constant(variables,-1.0); q.upper = Vector::Constant(variables,1.0);
    q.lower_c = Vector::Constant(constraints,-qpOASES::INFTY);
    q.upper_c = Vector::Constant(constraints,qpOASES::INFTY);
    for (int axis=0; axis<3; ++axis) {
      q.H.block(axis*n,axis*n,n,n)=axes[axis].H;
      q.C.block(axis*3*n,axis*n,3*n,n)=axes[axis].C;
      q.lower_c.segment(axis*3*n,3*n)=axes[axis].lower_c;
      q.upper_c.segment(axis*3*n,3*n)=axes[axis].upper_c;
    }
    q.previous_C=q.C;
    q.qp=std::make_unique<qpOASES::SQProblem>(variables,constraints,qpOASES::HST_POSDEF);
    qpOASES::Options options; options.setToMPC(); options.printLevel=qpOASES::PL_NONE;
    options.terminationTolerance=1e-10; options.boundTolerance=1e-10;
    q.qp->setOptions(options);
    int iterations=1000; double startup_budget=1.0;
    const auto status=q.qp->init(q.H.data(),q.g.data(),q.C.data(),q.lower.data(),q.upper.data(),
                                q.lower_c.data(),q.upper_c.data(),iterations,&startup_budget);
    if(status!=qpOASES::SUCCESSFUL_RETURN)throw std::runtime_error("Preview contact QP prewarm failed");
    // The neutral constructor problem has absent contact rows. Initializing
    // the first real contact problem avoids a homotopy from infinite bounds
    // into hundreds of simultaneously active plane constraints.
    q.warm=false;
    // All three stand translation axes have the same SPD objective. The exact
    // contact-only relaxation preserves unconstrained tangents and solves one
    // normal coordinate; original stand-axis bounds are verified afterward.
    for(int axis=1;axis<3;++axis)
      if(jerk_limit[axis]!=jerk_limit[0] || (axes[axis].H.array()!=axes[0].H.array()).any())
        throw std::invalid_argument("Exact contact decomposition requires identical translation objectives and jerk normalization");
    translation_factor.compute(axes[0].H);
    if(translation_factor.info()!=Eigen::Success)throw std::runtime_error("Preview translation factorization failed");
    unconstrained_translation=Matrix::Zero(n,3);
    auto& normal=normal_contact;
    normal.H=axes[0].H;normal.C=Matrix::Zero(3*n,n);normal.previous_C=normal.C;
    normal.full_C=Matrix::Zero(kMaxContactRows,n);
    normal.full_upper=Vector::Constant(kMaxContactRows,qpOASES::INFTY);
    normal.full_lower=Vector::Constant(kMaxContactRows,-qpOASES::INFTY);
    normal.g=Vector::Zero(n);normal.solution=Vector::Zero(n);normal.row_scale=Vector::Ones(kMaxContactRows);
    // Rotating a stand-axis box does not produce independent normal/tangent
    // boxes. The relaxation imposes NONE of those artificial rotated limits.
    normal.lower=Vector::Constant(n,-qpOASES::INFTY);normal.upper=Vector::Constant(n,qpOASES::INFTY);
    normal.lower_c=Vector::Constant(3*n,-qpOASES::INFTY);normal.upper_c=Vector::Constant(3*n,qpOASES::INFTY);
    normal.qp=std::make_unique<qpOASES::SQProblem>(n,3*n,qpOASES::HST_POSDEF);
    normal.qp->setOptions(options);
    iterations=1000;startup_budget=1.0;
    if(normal.qp->init(normal.H.data(),normal.g.data(),normal.C.data(),normal.lower.data(),normal.upper.data(),
        normal.lower_c.data(),normal.upper_c.data(),iterations,&startup_budget)!=qpOASES::SUCCESSFUL_RETURN)
      throw std::runtime_error("Preview normal QP prewarm failed");
    normal.warm=false;
    // Fast independent angular optima are accepted only if their complete
    // Bernstein controls satisfy the norm balls. Otherwise a bounded QP
    // cutting-plane solve couples the three axes. All storage is worker-owned.
    const int angular_variables=3*n, maximum_cuts=kAngularCutsPerInterval*n;
    angular_norm_cuts=Matrix::Zero(maximum_cuts,angular_variables);
    angular_norm_upper=Vector::Constant(maximum_cuts,qpOASES::INFTY);
    // Retain the original component velocity/acceleration outer boxes: although
    // redundant after vector-norm certification, they tighten the intermediate
    // relaxation and preserve convergence at rebased nonzero accelerations.
    // A small pool avoids scanning hundreds of unused support-plane rows for
    // the ordinary case. Complex requests use the original full-sized QP from
    // their first solve; repeated capacity promotions would discard active-set
    // progress and waste the fixed iteration budget. One full fallback remains
    // if the initially small problem grows. No support plane is ever dropped.
    angular_objective_scale=1.0/axes[3].H.cwiseAbs().maxCoeff();
    for(int capacity=std::min(32,maximum_cuts);;capacity=maximum_cuts) {
      angular_norm_pool.emplace_back();
      auto& angular=angular_norm_pool.back();
      angular.H=Matrix::Zero(angular_variables,angular_variables);
      angular.C=Matrix::Zero(9*n+capacity,angular_variables);
      angular.g=Vector::Zero(angular_variables);angular.solution=Vector::Zero(angular_variables);
      angular.lower=Vector::Constant(angular_variables,-1.0);
      angular.upper=Vector::Constant(angular_variables,1.0);
      angular.lower_c=Vector::Constant(9*n+capacity,-qpOASES::INFTY);
      angular.upper_c=Vector::Constant(9*n+capacity,qpOASES::INFTY);
      for(int axis=0;axis<3;++axis) {
        angular.H.block(axis*n,axis*n,n,n)=angular_objective_scale*axes[axis+3].H;
        angular.C.block(axis*3*n,axis*n,3*n,n)=axes[axis+3].C;
        angular.lower_c.segment(axis*3*n,3*n)=axes[axis+3].lower_c;
        angular.upper_c.segment(axis*3*n,3*n)=axes[axis+3].upper_c;
      }
      // Common positive H/g scaling leaves tracking/jerk ratios unchanged.
      angular.previous_C=angular.C;
      angular.qp=std::make_unique<qpOASES::SQProblem>(angular_variables,9*n+capacity,qpOASES::HST_POSDEF);
      angular.qp->setOptions(options);iterations=1000;startup_budget=1.0;
      if(angular.qp->init(angular.H.data(),angular.g.data(),angular.C.data(),angular.lower.data(),angular.upper.data(),
          angular.lower_c.data(),angular.upper_c.data(),iterations,&startup_budget)!=qpOASES::SUCCESSFUL_RETURN)
        throw std::runtime_error("Preview angular norm QP prewarm failed");
      angular.warm=false;
      if(capacity==maximum_cuts)break;
    }
  }

  void bounds(int axis, double v0, double a0) {
    auto& q=axes[axis]; const double h=cfg.planning_dt_sec;
    for(int k=0;k<n;++k) {
      const double middle=v0+a0*h*(k+.5), end=v0+a0*h*(k+1);
      q.lower_c[3*k]=-velocity_limit[axis]-middle;
      q.upper_c[3*k]= velocity_limit[axis]-middle;
      q.lower_c[3*k+1]=-velocity_limit[axis]-end;
      q.upper_c[3*k+1]= velocity_limit[axis]-end;
      q.lower_c[3*k+2]=-acceleration_limit[axis]-a0;
      q.upper_c[3*k+2]= acceleration_limit[axis]-a0;
    }
  }

  int contactBounds(const PreviewContactConstraint& authority,
                    const Axes& v0,const Axes& a0) {
    auto& q=contact;
    q.full_C.setZero();q.full_upper.setConstant(qpOASES::INFTY);q.full_lower.setConstant(-qpOASES::INFTY);
    q.endpoint_rows.fill(-1);
    for(int axis=0;axis<3;++axis) {
      q.g.segment(axis*n,n)=axes[axis].g;
      q.lower_c.segment(axis*3*n,3*n)=axes[axis].lower_c;
      q.upper_c.segment(axis*3*n,3*n)=axes[axis].upper_c;
    }
    const double h=cfg.planning_dt_sec;
    int rows=0;
    bool impossible=false;
    // Bernstein controls formed at the interval START avoid subtracting two
    // almost equal jerk coefficients. The first interval's B0/B1 depend only
    // on fixed initial v/a. Normalize nonzero rows for qpOASES while retaining
    // their m/s scale for independent feasibility diagnostics.
    // Two-sided since 2026-09-16: the same Bernstein control row carries the ceiling
    // (upper_c) and, when the knot has a finite retreat floor, the floor (lower_c).
    // qpOASES takes both sides of one row, so the floor costs no extra rows.
    const auto add = [&](double t,double a_weight,double j_weight,
                         int active_jerk,double bound,double floor_bound,bool endpoint=false) {
      if(rows>=kMaxContactRows)return false;
      const int row=rows;
      const Eigen::Vector3d free_velocity=v0.head<3>()+(t+a_weight)*a0.head<3>();
      const double free_closing=authority.normal_stand.dot(free_velocity);
      q.full_upper[row]=bound-free_closing;
      const bool floored=std::isfinite(floor_bound);
      q.full_lower[row]=floored?floor_bound-free_closing:-qpOASES::INFTY;
      for(int axis=0;axis<3;++axis)for(int l=0;l<n;++l) {
        const double t1=std::max(0.0,t-l*h),t0=std::max(0.0,t-(l+1)*h);
        const double v=.5*(t1*t1-t0*t0);
        const double a=t1-t0;
        q.full_C(row,axis*n+l)=authority.normal_stand[axis]*jerk_limit[axis]*
            (v+a_weight*a+(l==active_jerk?j_weight:0.0));
      }
      const double norm=q.full_C.row(row).norm();
      if(norm==0.0) {
        if(q.full_upper[row]<-cfg.feasibility_tolerance ||
           (floored && q.full_lower[row]>cfg.feasibility_tolerance))impossible=true;
        q.full_upper[row]=qpOASES::INFTY;q.full_lower[row]=-qpOASES::INFTY;
        return true;
      }
      q.row_scale[rows]=norm;q.full_C.row(row)/=norm;q.full_upper[row]/=norm;
      if(floored)q.full_lower[row]/=norm;
      q.row_interval[rows]=active_jerk;if(endpoint)q.endpoint_rows[active_jerk]=rows;
      ++rows;
      return true;
    };
    for(std::size_t i=1;i<authority.count;++i) {
      const auto& a=authority.knots[i-1];const auto& b=authority.knots[i];
      if(a.time_sec>=n*h)break;
      const double end=std::min(b.time_sec,n*h);
      const double slope=(b.upper_velocity_m_s-a.upper_velocity_m_s)/(b.time_sec-a.time_sec);
      const bool floored=std::isfinite(a.lower_velocity_m_s)&&std::isfinite(b.lower_velocity_m_s);
      const double floor_slope=floored?(b.lower_velocity_m_s-a.lower_velocity_m_s)/(b.time_sec-a.time_sec):0.0;
      constexpr double kNoFloor=-std::numeric_limits<double>::infinity();
      double begin=a.time_sec;
      while(begin<end) {
        // Split at BOTH force-reference and jerk-grid boundaries. Tiny floating
        // point subintervals are legal; no tolerance expands contact authority.
        int next=static_cast<int>(std::floor(begin/h))+1;
        while(next*h<=begin)++next;
        const double finish=std::min(end,next*h);
        const double span=finish-begin;
        const double ceiling_begin=a.upper_velocity_m_s+slope*(begin-a.time_sec);
        const double ceiling_finish=a.upper_velocity_m_s+slope*(finish-a.time_sec);
        const double floor_begin=floored?a.lower_velocity_m_s+floor_slope*(begin-a.time_sec):kNoFloor;
        const double floor_finish=floored?a.lower_velocity_m_s+floor_slope*(finish-a.time_sec):kNoFloor;
        const int active_jerk=std::min(next-1,n-1);
        if(!add(begin,0,0,active_jerk,ceiling_begin,floor_begin)||
           !add(begin,.5*span,0,active_jerk,ceiling_begin+.5*slope*span,
                floored?floor_begin+.5*floor_slope*span:kNoFloor)||
           !add(begin,span,.5*span*span,active_jerk,ceiling_finish,floor_finish,finish==next*h))return -1;
        begin=finish;
      }
    }
    return impossible?-2:rows;
  }
};

PreviewTrajectoryTracker::PreviewTrajectoryTracker(const PreviewTrackerConfig& cfg)
  : impl_(std::make_unique<Impl>(cfg)) {}
PreviewTrajectoryTracker::~PreviewTrajectoryTracker() = default;
const PreviewTrackerConfig& PreviewTrajectoryTracker::config() const { return impl_->cfg; }
bool PreviewTrajectoryTracker::hasTrajectory() const { return impl_->trajectory.valid; }
double PreviewTrajectoryTracker::durationSec() const { return impl_->n*impl_->cfg.planning_dt_sec; }
void PreviewTrajectoryTracker::reset() { impl_->trajectory.valid=false; }

PreviewSolveResult PreviewTrajectoryTracker::plan(const PreviewReference& ref,
                                                const PreviewMotionState& initial) {
  return plan(ref,initial,PreviewContactConstraint{});
}

PreviewSolveResult PreviewTrajectoryTracker::plan(const PreviewReference& ref,
                                                const PreviewMotionState& initial,
                                                const PreviewContactConstraint& authority,
                                                PreviewContactSolveMode mode,
                                                double request_solve_budget_sec) {
  auto& x=*impl_; const auto& cfg=x.cfg; const auto begin=Clock::now();
  PreviewSolveResult result;
  auto elapsed=[&] { return std::chrono::duration<double>(Clock::now()-begin).count(); };
  auto finish=[&](PreviewSolveStatus status) {
    result.status=status; result.diagnostics.status=status;
    result.diagnostics.solve_time_sec=elapsed(); return result;
  };
  if(std::isnan(request_solve_budget_sec) || request_solve_budget_sec<0.0)
    return finish(PreviewSolveStatus::InvalidReference);
  const double solve_budget=std::min(cfg.max_solve_time_sec,request_solve_budget_sec);
  if(solve_budget<=0.0)return finish(PreviewSolveStatus::TimeBudgetExceeded);
  if (!validPose(initial.pose) || !initial.linear_velocity.allFinite() ||
      !initial.linear_acceleration.allFinite() || !initial.angular_velocity_body.allFinite() ||
      !initial.angular_acceleration_body.allFinite()) return finish(PreviewSolveStatus::InvalidInitialState);
  Axes start_p, start_v, start_a;
  start_p.head<3>()=position(initial.pose); start_p.tail<3>().setZero();
  start_v << initial.linear_velocity, initial.angular_velocity_body;
  start_a << initial.linear_acceleration, initial.angular_acceleration_body;
  // The accepted QP and physical envelope use this same explicit tolerance.
  // Preserve the initial state exactly: a round-off-scale overshoot of an
  // accepted endpoint is not a request to clip/reseed the next C2 splice.
  if ((start_v.cwiseAbs().array()>x.velocity_limit.array()+cfg.feasibility_tolerance).any() ||
      (start_a.cwiseAbs().array()>x.acceleration_limit.array()+cfg.feasibility_tolerance).any() ||
      start_v.tail<3>().norm()>x.velocity_limit[3]+cfg.feasibility_tolerance ||
      start_a.tail<3>().norm()>x.acceleration_limit[3]+cfg.feasibility_tolerance)
    return finish(PreviewSolveStatus::InvalidInitialState); // never clip the splice
  if (ref.count<2 || ref.count>ref.knots.size() || ref.knots[0].time_sec!=0.0 ||
      ref.knots[ref.count-1].time_sec+1e-12<durationSec()) return finish(PreviewSolveStatus::InvalidReference);
  const Eigen::Matrix3d R0=math::rotationFromPose(initial.pose);
  for(std::size_t k=0;k<ref.count;++k) {
    if (!std::isfinite(ref.knots[k].time_sec) || !validPose(ref.knots[k].pose) ||
        (k && ref.knots[k].time_sec<=ref.knots[k-1].time_sec)) return finish(PreviewSolveStatus::InvalidReference);
    if (math::log3(R0.transpose()*math::rotationFromPose(ref.knots[k].pose)).norm()>
        cfg.max_reference_chart_angle_rad) return finish(PreviewSolveStatus::InvalidReference);
  }
  if(authority.enabled) {
    result.diagnostics.contact_constrained=true;
    if(authority.count<2 || authority.count>authority.knots.size() ||
       !authority.normal_stand.allFinite() ||
       std::abs(authority.normal_stand.norm()-1.0)>cfg.feasibility_tolerance ||
       authority.knots[0].time_sec!=0.0 ||
       authority.knots[authority.count-1].time_sec<durationSec())
      return finish(PreviewSolveStatus::InvalidReference);
    for(std::size_t k=0;k<authority.count;++k) {
      const auto& knot=authority.knots[k];
      // The floor is -infinity (none) or finite and at most the ceiling; NaN and
      // +infinity are refused like any other malformed authority.
      const bool floor_ok=knot.lower_velocity_m_s==-std::numeric_limits<double>::infinity() ||
          (std::isfinite(knot.lower_velocity_m_s) &&
           knot.lower_velocity_m_s<=knot.upper_velocity_m_s+cfg.feasibility_tolerance);
      if(!std::isfinite(knot.time_sec) || !nonnegative(knot.upper_velocity_m_s) || !floor_ok ||
         (k && knot.time_sec<=authority.knots[k-1].time_sec))
        return finish(PreviewSolveStatus::InvalidReference);
    }
    // A new velocity authority cannot change the accepted C2 splice. A seed
    // moving too fast into contact must first use the separate finite brake; a
    // seed retreating faster than the floor must have been admitted by the slew.
    const double seed_closing=authority.normal_stand.dot(initial.linear_velocity);
    const double excess=seed_closing-authority.knots[0].upper_velocity_m_s;
    const double deficit=std::isfinite(authority.knots[0].lower_velocity_m_s)?
        authority.knots[0].lower_velocity_m_s-seed_closing:0.0;
    result.diagnostics.max_contact_velocity_violation_m_s=std::max({0.0,excess,deficit});
    if(std::max(excess,deficit)>cfg.feasibility_tolerance)return finish(PreviewSolveStatus::Infeasible);
  }
  const double h=cfg.planning_dt_sec;
  // REFERENCE TRUST (see PreviewTrackerConfig): past the committed window the target is
  // blended toward the constant-velocity continuation of that window, so an uncommitted
  // far demand no longer dictates today's acceleration. A steady reference continues
  // into itself, so this is exactly a no-op for it. ONE closure serves both consumers --
  // the QP's knots and the dense acceptance below -- so the plan is scored against
  // exactly the reference it was asked to follow (scoring it against the raw reference
  // instead spent tracking budget on the lag the blend deliberately introduces: 4
  // tracking_budget_exceeded rejections in one 1.5 s floor contact,
  // servo_log_20260910_172712 @28.4 s).
  const double trust_full=cfg.reference_trust_full_sec;
  const auto chart_of=[&](const Pose6D& p) {
    Axes v;
    v.head<3>()=position(p);
    v.tail<3>()=math::log3(R0.transpose()*math::rotationFromPose(p));
    return v;
  };
  Axes trust_anchor=start_p, trust_slope=start_v;
  double trust_anchor_time=0.0;
  if(trust_full>=h) {
    trust_anchor=chart_of(referenceAt(ref,trust_full));
    trust_anchor_time=trust_full;
    trust_slope=(trust_anchor-chart_of(referenceAt(ref,trust_full-h)))/h;
  }
  const auto target_at=[&](double t) {
    Axes raw=chart_of(referenceAt(ref,t));
    if(t<=trust_full)return raw;
    Axes continuation=trust_anchor+trust_slope*(t-trust_anchor_time);
    // The chart is only linearized out to max_reference_chart_angle_rad and every raw
    // knot was validated against it; hold the continuation to the same ball so the
    // blend (a convex combination) cannot leave it either.
    const double angle=continuation.tail<3>().norm();
    if(angle>cfg.max_reference_chart_angle_rad)
      continuation.tail<3>()*=cfg.max_reference_chart_angle_rad/angle;
    // A STEADY REFERENCE IS RETURNED UNTOUCHED, BIT FOR BIT. Its own continuation is
    // itself, so the blend is analytically a no-op -- but only analytically: log3 and
    // the knot interpolation leave a ~1e-16 residue, and the angular cutting-plane
    // solve for a demand sitting ON the 1.4 rad/s ball is sensitive to exactly that
    // (measured 2026-09-10 on the rebased-splice case: 348 -> 557 planes and the
    // iteration cap, from a 1e-16 change in the target and nothing else). The
    // threshold is 1 pm / 1 prad -- far below anything physical, far above the residue.
    const Axes delta=continuation-raw;
    if(delta.squaredNorm()<=1e-24)return raw;
    const double w=std::exp(-(t-trust_full)/x.reference_trust_tau);
    return Axes(raw+(1.0-w)*delta);
  };
  Eigen::Matrix<double,kCapacity,6,Eigen::RowMajor> targets;
  for(int k=0;k<x.n;++k) targets.row(k)=target_at((k+1)*h).transpose();
  Trajectory candidate;
  candidate.count = static_cast<std::size_t>(x.n);
  candidate.step_sec = h;
  candidate.p.setZero(); candidate.v.setZero(); candidate.a.setZero(); candidate.jerk.setZero();
  candidate.rotation0=R0; candidate.p.row(0)=start_p.transpose();
  candidate.v.row(0)=start_v.transpose(); candidate.a.row(0)=start_a.transpose();
  const auto prepare_axis = [&](int axis) {
    auto& q=x.axes[axis]; const double scale=axis<3?cfg.linear_tracking_scale_m:cfg.angular_tracking_scale_rad;
    // g=-2 P' (reference-free_motion)/tracking_scale, without temporary allocation.
    q.g.setZero();
    for(int k=0;k<x.n;++k) {
      const double t=(k+1)*h;
      const double desired=(targets(k,axis)-start_p[axis]-start_v[axis]*t-.5*start_a[axis]*t*t)/scale;
      for(int l=0;l<x.n;++l) q.g[l]-=2.0*q.prediction(k,l)*desired;
    }
    x.bounds(axis,start_v[axis],start_a[axis]);
  };
  const auto integrate_axis = [&](int axis) {
    const auto& solution=x.axes[axis].solution;
    for(int k=0;k<x.n;++k) {
      const double j=solution[k]*x.jerk_limit[axis];
      candidate.jerk(k,axis)=j;
      candidate.p(k+1,axis)=candidate.p(k,axis)+candidate.v(k,axis)*h+.5*candidate.a(k,axis)*h*h+j*h*h*h/6.;
      candidate.v(k+1,axis)=candidate.v(k,axis)+candidate.a(k,axis)*h+.5*j*h*h;
      candidate.a(k+1,axis)=candidate.a(k,axis)+j*h;
    }
  };
  if(authority.enabled) {
    for(int axis=0;axis<3;++axis)prepare_axis(axis);
    const int rows=x.contactBounds(authority,start_v,start_a);
    if(rows==-2)return finish(PreviewSolveStatus::Infeasible);
    if(rows<0)return finish(PreviewSolveStatus::InvalidReference);
    result.diagnostics.contact_constraint_rows=static_cast<std::size_t>(rows);
    auto& q=x.contact;
    const auto solve_contact = [&](Impl::ContactQp& q,int base_rows) -> PreviewSolveStatus {
      std::array<int,3*kCapacity> cuts{};
      std::array<bool,kMaxContactRows> selected{};
      int cut_count=0;
      for(int k=0;k<x.n;++k)if(q.endpoint_rows[k]>=0) {
        const int row=q.endpoint_rows[k];cuts[cut_count++]=row;selected[row]=true;
      }
      for(;;) {
        q.C.bottomRows(3*x.n).setZero();
        q.lower_c.tail(3*x.n).setConstant(-qpOASES::INFTY);
        q.upper_c.tail(3*x.n).setConstant(qpOASES::INFTY);
        for(int k=0;k<cut_count;++k) {
          q.C.row(base_rows+k)=q.full_C.row(cuts[k]);q.upper_c[base_rows+k]=q.full_upper[cuts[k]];
          q.lower_c[base_rows+k]=q.full_lower[cuts[k]];
        }
        double remaining=solve_budget-elapsed();
        if(remaining<=0.0)return PreviewSolveStatus::TimeBudgetExceeded;
        int iterations=cfg.max_working_set_recalculations-result.diagnostics.working_set_recalculations;
        if(iterations<=0)return PreviewSolveStatus::IterationLimit;
        qpOASES::returnValue status;
        if(q.warm)status=q.qp->hotstart(q.H.data(),q.g.data(),q.C.data(),q.lower.data(),q.upper.data(),
                                       q.lower_c.data(),q.upper_c.data(),iterations,&remaining);
        else {
          q.qp->reset();
          status=q.qp->init(q.H.data(),q.g.data(),q.C.data(),q.lower.data(),q.upper.data(),
                            q.lower_c.data(),q.upper_c.data(),iterations,&remaining);
        }
        // qpOASES keeps a shallow view of A. Its previous A must remain intact
        // while the next hotstart computes a new factorization. Rebuilding A in
        // that same backing buffer invalidates the active-set state silently.
        q.C.swap(q.previous_C);
        result.diagnostics.working_set_recalculations+=iterations;
        q.warm=status==qpOASES::SUCCESSFUL_RETURN;
        if(elapsed()>solve_budget)return PreviewSolveStatus::TimeBudgetExceeded;
        if(!q.warm) {
          if(status==qpOASES::RET_MAX_NWSR_REACHED)return PreviewSolveStatus::IterationLimit;
          if(status==qpOASES::RET_INIT_FAILED_INFEASIBILITY || status==qpOASES::RET_QP_INFEASIBLE ||
             status==qpOASES::RET_HOTSTART_STOPPED_INFEASIBILITY)return PreviewSolveStatus::Infeasible;
          return PreviewSolveStatus::NumericalFailure;
        }
        if(q.qp->getPrimalSolution(q.solution.data())!=qpOASES::SUCCESSFUL_RETURN || !q.solution.allFinite())
          return PreviewSolveStatus::NumericalFailure;
        double violation=base_rows?std::max(0.0,q.solution.cwiseAbs().maxCoeff()-1.0):0.0;
        for(int r=0;r<base_rows;++r) {
          const double value=q.previous_C.row(r).dot(q.solution);
          violation=std::max({violation,q.lower_c[r]-value,value-q.upper_c[r]});
        }
        if(violation>cfg.feasibility_tolerance)return PreviewSolveStatus::NumericalFailure;
        std::array<int,kCapacity> worst_index;worst_index.fill(-1);
        std::array<double,kCapacity> worst;worst.fill(cfg.feasibility_tolerance);
        double contact_violation=0;
        for(int r=0;r<rows;++r) {
          const double value=q.full_C.row(r).dot(q.solution);
          const double excess=q.row_scale[r]*std::max(value-q.full_upper[r],q.full_lower[r]-value);
          contact_violation=std::max(contact_violation,excess);
          if(selected[r] && excess>cfg.feasibility_tolerance)return PreviewSolveStatus::NumericalFailure;
          const int segment=q.row_interval[r];
          if(!selected[r] && excess>worst[segment]) {worst[segment]=excess;worst_index[segment]=r;}
        }
        result.diagnostics.max_contact_velocity_violation_m_s=contact_violation;
        result.diagnostics.max_constraint_violation=std::max(violation,contact_violation);
        if(contact_violation<=cfg.feasibility_tolerance)break;
        bool added=false;
        for(int k=0;k<x.n;++k)if(worst_index[k]>=0) {
          if(cut_count>=3*x.n)return PreviewSolveStatus::IterationLimit;
          const int row=worst_index[k];cuts[cut_count++]=row;selected[row]=true;added=true;
        }
        if(!added)return PreviewSolveStatus::NumericalFailure;
      }
      return PreviewSolveStatus::Solved;
    };
    bool relaxed_feasible=false;
    if(mode==PreviewContactSolveMode::Automatic) {
      const Eigen::Vector3d normal=authority.normal_stand.normalized();
      auto& reduced=x.normal_contact;
      reduced.g.setZero();
      for(int axis=0;axis<3;++axis) {
        x.unconstrained_translation.col(axis)=x.translation_factor.solve(-x.axes[axis].g);
        const auto residual=x.axes[axis].H*x.unconstrained_translation.col(axis)+x.axes[axis].g;
        if(!x.unconstrained_translation.col(axis).allFinite() ||
           residual.norm()>1e-10*std::max(1.0,x.axes[axis].g.norm()))
          return finish(PreviewSolveStatus::NumericalFailure);
        reduced.g+=normal[axis]*x.axes[axis].g;
      }
      reduced.full_C.setZero();
      for(int axis=0;axis<3;++axis)
        reduced.full_C.topRows(rows)+=normal[axis]*q.full_C.block(0,axis*x.n,rows,x.n);
      reduced.full_upper=q.full_upper;reduced.full_lower=q.full_lower;reduced.row_scale=q.row_scale;
      reduced.endpoint_rows=q.endpoint_rows;reduced.row_interval=q.row_interval;
      const auto status=solve_contact(reduced,0);
      if(status!=PreviewSolveStatus::Solved)return finish(status);
      const Vector correction=reduced.solution-x.unconstrained_translation*normal;
      for(int axis=0;axis<3;++axis)
        q.solution.segment(axis*x.n,x.n)=x.unconstrained_translation.col(axis)+normal[axis]*correction;
      // A feasible minimizer of this contact-only relaxation is also the unique
      // minimizer of the full strictly convex problem. Audit every original
      // stand-axis jerk box / continuous Bernstein v,a row, not rotated bounds.
      double violation=std::max(0.0,q.solution.cwiseAbs().maxCoeff()-1.0);
      for(int axis=0;axis<3;++axis)for(int r=0;r<3*x.n;++r) {
        const auto& axis_q=x.axes[axis];
        const double value=axis_q.C.row(r).dot(q.solution.segment(axis*x.n,x.n));
        violation=std::max({violation,axis_q.lower_c[r]-value,value-axis_q.upper_c[r]});
      }
      for(int r=0;r<rows;++r) {
        const double value=q.full_C.row(r).dot(q.solution);
        violation=std::max(violation,q.row_scale[r]*std::max(value-q.full_upper[r],q.full_lower[r]-value));
      }
      relaxed_feasible=violation<=cfg.feasibility_tolerance;
      result.diagnostics.contact_decomposed=relaxed_feasible;
      result.diagnostics.contact_coupled_fallback=!relaxed_feasible;
    }
    if(!relaxed_feasible) {
      const auto status=solve_contact(q,9*x.n);
      if(status!=PreviewSolveStatus::Solved)return finish(status);
    }
    for(int axis=0;axis<3;++axis) {
      x.axes[axis].solution=q.solution.segment(axis*x.n,x.n);
      integrate_axis(axis);
    }
  }
  for(int axis=authority.enabled?3:0;axis<6;++axis) {
    prepare_axis(axis);
    auto& q=x.axes[axis];
    double remaining=solve_budget-elapsed();
    if (remaining<=0.0) return finish(PreviewSolveStatus::TimeBudgetExceeded);
    int iterations=cfg.max_working_set_recalculations;
    qpOASES::returnValue status;
    if(q.warm) status=q.qp->hotstart(q.g.data(),q.lower.data(),q.upper.data(),
                                    q.lower_c.data(),q.upper_c.data(),iterations,&remaining);
    else {
      q.qp->reset();
      status=q.qp->init(q.H.data(),q.g.data(),q.C.data(),q.lower.data(),q.upper.data(),
                        q.lower_c.data(),q.upper_c.data(),iterations,&remaining);
    }
    result.diagnostics.working_set_recalculations+=iterations;
    q.warm=status==qpOASES::SUCCESSFUL_RETURN;
    if(elapsed()>solve_budget) return finish(PreviewSolveStatus::TimeBudgetExceeded);
    if(!q.warm) {
      if(status==qpOASES::RET_MAX_NWSR_REACHED) return finish(PreviewSolveStatus::IterationLimit);
      if(status==qpOASES::RET_INIT_FAILED_INFEASIBILITY || status==qpOASES::RET_QP_INFEASIBLE ||
         status==qpOASES::RET_HOTSTART_STOPPED_INFEASIBILITY) return finish(PreviewSolveStatus::Infeasible);
      return finish(PreviewSolveStatus::NumericalFailure);
    }
    if(q.qp->getPrimalSolution(q.solution.data())!=qpOASES::SUCCESSFUL_RETURN || !q.solution.allFinite())
      return finish(PreviewSolveStatus::NumericalFailure);
    double violation=std::max(0.0,q.solution.cwiseAbs().maxCoeff()-1.0);
    for(int r=0;r<3*x.n;++r) {
      const double value=q.C.row(r).dot(q.solution);
      violation=std::max({violation,q.lower_c[r]-value,value-q.upper_c[r]});
    }
    result.diagnostics.max_constraint_violation=std::max(result.diagnostics.max_constraint_violation,violation);
    if(violation>cfg.feasibility_tolerance) return finish(PreviewSolveStatus::NumericalFailure);
    integrate_axis(axis);
  }
  // A quadratic velocity curve lies in the convex hull of its three
  // Bernstein controls. Acceleration is linear, so its endpoints suffice.
  // Certify the vector norm, rather than an artificial per-axis V/sqrt(3)
  // ceiling. The initial endpoint was checked above and later endpoints
  // become the next interval's initial endpoint.
  const auto angular_free_control = [&](int row) -> Eigen::Vector3d {
    const int k=row/3;
    if(row%3==2)return start_a.tail<3>();
    return start_v.tail<3>()+start_a.tail<3>()*h*(k+(row%3==0?.5:1.0));
  };
  const auto angular_control = [&](int row) -> Eigen::Vector3d {
    Eigen::Vector3d value=angular_free_control(row);
    for(int axis=0;axis<3;++axis)value[axis]+=x.axes[axis+3].C.row(row).dot(x.axes[axis+3].solution);
    return value;
  };
  const auto angular_certificate = [&]() {
    double violation=0;
    auto& d=result.diagnostics;
    d.max_angular_chart_velocity_norm=start_v.tail<3>().norm();
    d.max_angular_chart_acceleration_norm=start_a.tail<3>().norm();
    for(int row=0;row<3*x.n;++row) {
      const double norm=angular_control(row).norm();
      const bool acceleration=row%3==2;
      auto& maximum=acceleration?d.max_angular_chart_acceleration_norm:d.max_angular_chart_velocity_norm;
      maximum=std::max(maximum,norm);
      violation=std::max(violation,norm-(acceleration?x.acceleration_limit[3]:x.velocity_limit[3]));
    }
    return violation;
  };
  if(angular_certificate()>cfg.feasibility_tolerance) {
    result.diagnostics.angular_norm_coupled=true;
    const int base_rows=9*x.n,cut_rows=kAngularCutsPerInterval*x.n;
    // Store cuts separately from qpOASES's previous A buffer. Each plane is
    // an outer approximation of a norm ball. No plan is accepted merely for
    // satisfying these planes: the complete norm certificate is rechecked.
    auto& cuts=x.angular_norm_cuts;auto& cut_upper=x.angular_norm_upper;
    cuts.setZero();cut_upper.setConstant(qpOASES::INFTY);
    // One cold init per nonlinear problem. Measured 2026-09-10 on the rebased
    // splice case: a warm start ACROSS problems ran into 445 cuts / the NWSR
    // limit (replaced cut rows make a misleading homotopy), and seeding the
    // previous problem's support planes was worse as well (348 -> 460 cuts,
    // NWSR 880 -> 1123). The live cost was never the plane count: the real
    // solves needed 1-9 cuts and 3-16 NWSR yet took 3-17 ms, because qpOASES's
    // BLAS calls were fanned over OpenBLAS's thread pool (pinBlasThreads).
    for(auto& pool:x.angular_norm_pool)pool.warm=false;
    int retained_cuts=0;
    std::size_t cuts_added=0;
    const auto add_plane=[&](int row,const Eigen::Vector3d& normal,double limit) -> bool {
      const int cut=retained_cuts++;
      for(int axis=0;axis<3;++axis)
        cuts.block(cut,axis*x.n,1,x.n)=normal[axis]*x.axes[axis+3].C.row(row);
      const double scale=cuts.row(cut).norm();
      // A fixed initial Bernstein control cannot be repaired by a future
      // jerk. Refuse it without clipping the accepted splice derivatives.
      if(scale==0)return false;
      cuts.row(cut)/=scale;
      cut_upper[cut]=(limit-normal.dot(angular_free_control(row)))/scale;
      ++cuts_added;
      return true;
    };
    for(;;) {
      for(int row=0;row<3*x.n;++row) {
        const Eigen::Vector3d value=angular_control(row);
        const double norm=value.norm();
        const double limit=row%3==2?x.acceleration_limit[3]:x.velocity_limit[3];
        if(norm<=limit+cfg.feasibility_tolerance)continue;
        // Keep every support plane for this nonlinear problem. Dropping even
        // inactive planes can revisit earlier directions, and replacing rows
        // creates an unnecessarily ill-conditioned SQProblem homotopy. Each
        // successful relaxation therefore retains the previous feasible set
        // and adds only valid outer planes. The fixed storage, total NWSR and
        // wall-time budgets all remain fail-closed limits on this iteration.
        if(retained_cuts>=cut_rows)return finish(PreviewSolveStatus::IterationLimit);
        if(!add_plane(row,value/norm,limit))return finish(PreviewSolveStatus::Infeasible);
      }
      result.diagnostics.angular_norm_cuts=cuts_added;
      auto pool=std::find_if(x.angular_norm_pool.begin(),x.angular_norm_pool.end(),
          [&](const Impl::ContactQp& candidate){return candidate.C.rows()>=base_rows+retained_cuts;});
      if(pool==x.angular_norm_pool.end())return finish(PreviewSolveStatus::IterationLimit);
      auto& q=*pool;
      q.C.setZero();q.lower_c.setConstant(-qpOASES::INFTY);q.upper_c.setConstant(qpOASES::INFTY);
      for(int axis=0;axis<3;++axis) {
        const auto& aq=x.axes[axis+3];
        q.g.segment(axis*x.n,x.n)=x.angular_objective_scale*aq.g;
        q.C.block(axis*3*x.n,axis*x.n,3*x.n,x.n)=aq.C;
        q.lower_c.segment(axis*3*x.n,3*x.n)=aq.lower_c;
        q.upper_c.segment(axis*3*x.n,3*x.n)=aq.upper_c;
      }
      q.C.middleRows(base_rows,retained_cuts)=cuts.topRows(retained_cuts);
      q.upper_c.segment(base_rows,retained_cuts)=cut_upper.head(retained_cuts);
      double remaining=solve_budget-elapsed();
      if(remaining<=0)return finish(PreviewSolveStatus::TimeBudgetExceeded);
      int iterations=std::min(3*cfg.max_working_set_recalculations,
          6*cfg.max_working_set_recalculations-result.diagnostics.working_set_recalculations);
      if(iterations<=0)return finish(PreviewSolveStatus::IterationLimit);
      qpOASES::returnValue status;
      const int requested_iterations=iterations;
      ++result.diagnostics.angular_norm_rounds;
      const auto qp_begin=Clock::now();
      if(q.warm) {
        status=q.qp->hotstart(q.H.data(),q.g.data(),q.C.data(),q.lower.data(),q.upper.data(),
                              q.lower_c.data(),q.upper_c.data(),iterations,&remaining);
        if(status!=qpOASES::SUCCESSFUL_RETURN && status!=qpOASES::RET_MAX_NWSR_REACHED) {
          // A warm start from the previous request's active set is an
          // optimisation, never an authority: retry cold inside the same budget.
          result.diagnostics.working_set_recalculations+=iterations;
          remaining=solve_budget-elapsed();
          if(remaining<=0)return finish(PreviewSolveStatus::TimeBudgetExceeded);
          iterations=requested_iterations;
          q.qp->reset();
          status=q.qp->init(q.H.data(),q.g.data(),q.C.data(),q.lower.data(),q.upper.data(),
                           q.lower_c.data(),q.upper_c.data(),iterations,&remaining);
        }
      } else {
        q.qp->reset();
        status=q.qp->init(q.H.data(),q.g.data(),q.C.data(),q.lower.data(),q.upper.data(),
                         q.lower_c.data(),q.upper_c.data(),iterations,&remaining);
      }
      q.C.swap(q.previous_C); // keep the old shallow A view intact during hotstart
      result.diagnostics.angular_norm_qp_time_sec+=std::chrono::duration<double>(Clock::now()-qp_begin).count();
      result.diagnostics.working_set_recalculations+=iterations;
      q.warm=status==qpOASES::SUCCESSFUL_RETURN;
      if(elapsed()>solve_budget)return finish(PreviewSolveStatus::TimeBudgetExceeded);
      if(!q.warm) {
        if(status==qpOASES::RET_MAX_NWSR_REACHED)return finish(PreviewSolveStatus::IterationLimit);
        if(status==qpOASES::RET_INIT_FAILED_INFEASIBILITY || status==qpOASES::RET_QP_INFEASIBLE ||
           status==qpOASES::RET_HOTSTART_STOPPED_INFEASIBILITY)return finish(PreviewSolveStatus::Infeasible);
        return finish(PreviewSolveStatus::NumericalFailure);
      }
      if(q.qp->getPrimalSolution(q.solution.data())!=qpOASES::SUCCESSFUL_RETURN || !q.solution.allFinite())
        return finish(PreviewSolveStatus::NumericalFailure);
      double violation=std::max(0.0,q.solution.cwiseAbs().maxCoeff()-1.0);
      for(int row=0;row<base_rows+retained_cuts;++row) {
        const double value=q.previous_C.row(row).dot(q.solution);
        violation=std::max({violation,q.lower_c[row]-value,value-q.upper_c[row]});
      }
      if(violation>cfg.feasibility_tolerance)return finish(PreviewSolveStatus::NumericalFailure);
      for(int axis=0;axis<3;++axis)x.axes[axis+3].solution=q.solution.segment(axis*x.n,x.n);
      if(angular_certificate()<=cfg.feasibility_tolerance)break;
    }
    for(int axis=3;axis<6;++axis)integrate_axis(axis);
  }
  candidate.valid=true;
  // Independent dense physical evaluation also checks the frame derivative
  // implementation. Continuous hard envelopes follow the Bernstein/analytic
  // bounds above; this is additional validation, not their only evidence.
  const int samples=static_cast<int>(std::ceil(durationSec()/.002));
  for(int k=0;k<=samples;++k) {
    const double t=durationSec()*k/samples;
    PreviewMotionSample state;
    if(!evaluate(candidate,x.n,h,t,state)) return finish(PreviewSolveStatus::NumericalFailure);
    auto& d=result.diagnostics;
    d.max_linear_velocity=std::max(d.max_linear_velocity,state.linear_velocity.cwiseAbs().maxCoeff());
    d.max_linear_acceleration=std::max(d.max_linear_acceleration,state.linear_acceleration.cwiseAbs().maxCoeff());
    d.max_linear_jerk=std::max(d.max_linear_jerk,state.linear_jerk.cwiseAbs().maxCoeff());
    d.max_angular_velocity_norm=std::max(d.max_angular_velocity_norm,state.angular_velocity_body.norm());
    d.max_angular_acceleration_norm=std::max(d.max_angular_acceleration_norm,state.angular_acceleration_body.norm());
    d.max_angular_jerk_norm=std::max(d.max_angular_jerk_norm,state.angular_jerk_stand.norm());
    const Axes blended=target_at(t);
    const Pose6D target=math::poseFromSe3(pinocchio::SE3(
        Eigen::Matrix3d(R0*math::exp3(blended.tail<3>())),Eigen::Vector3d(blended.head<3>())));
    d.max_position_tracking_error_m=std::max(d.max_position_tracking_error_m,math::positionDistance(state.pose,target));
    d.max_orientation_tracking_error_rad=std::max(d.max_orientation_tracking_error_rad,math::orientationDistanceRad(state.pose,target));
    if(authority.enabled) {
      d.max_contact_velocity_violation_m_s=std::max(d.max_contact_velocity_violation_m_s,
          authority.normal_stand.dot(state.linear_velocity)-contactBoundAt(authority,t));
      if(d.max_contact_velocity_violation_m_s>cfg.feasibility_tolerance)
        return finish(PreviewSolveStatus::NumericalFailure);
    }
    if(elapsed()>solve_budget) return finish(PreviewSolveStatus::TimeBudgetExceeded);
  }
  auto& d=result.diagnostics;
  const double e=cfg.feasibility_tolerance;
  if(d.max_linear_velocity>cfg.max_linear_velocity_m_s+e ||
     d.max_linear_acceleration>cfg.max_linear_acceleration_m_s2+e ||
     d.max_linear_jerk>cfg.max_linear_jerk_m_s3*(1.+e) ||
     d.max_angular_velocity_norm>cfg.max_angular_velocity_rad_s+e ||
     d.max_angular_acceleration_norm>cfg.max_angular_acceleration_rad_s2+e ||
     d.max_angular_jerk_norm>cfg.max_angular_jerk_rad_s3+e)
    return finish(PreviewSolveStatus::NumericalFailure);
  d.max_position_tracking_slack_m=std::max(0.0,d.max_position_tracking_error_m-cfg.linear_tracking_tolerance_m);
  d.max_orientation_tracking_slack_rad=std::max(0.0,d.max_orientation_tracking_error_rad-cfg.angular_tracking_tolerance_rad);
  if(d.max_position_tracking_slack_m>cfg.max_linear_tracking_slack_m ||
     d.max_orientation_tracking_slack_rad>cfg.max_angular_tracking_slack_rad)
    return finish(PreviewSolveStatus::TrackingBudgetExceeded);
  if(elapsed()>solve_budget) return finish(PreviewSolveStatus::TimeBudgetExceeded);
  x.trajectory=candidate;
  return finish(PreviewSolveStatus::Solved);
}

bool PreviewTrajectoryTracker::sample(double time, PreviewMotionSample& out) const {
  return evaluate(impl_->trajectory,impl_->n,impl_->cfg.planning_dt_sec,time,out);
}
bool PreviewPolynomialTrajectory::sample(double time, PreviewMotionSample& out) const {
  if (count > kMaxHorizonSteps) return false;
  return evaluate(*this, static_cast<int>(count), step_sec, time, out);
}
bool PreviewTrajectoryTracker::exportTrajectory(PreviewPolynomialTrajectory& out) const {
  if (!impl_->trajectory.valid) return false;
  out = impl_->trajectory;
  return true;
}
void PreviewTrajectoryTracker::shiftCommonFrame(const Eigen::Vector3d& translation,
                                               const Eigen::Quaterniond& rotation) {
  if(!translation.allFinite() || !rotation.coeffs().allFinite() || rotation.norm()<1e-12)
    throw std::invalid_argument("Invalid preview common frame shift");
  auto& t=impl_->trajectory;
  if(!t.valid) return;
  for(int k=0;k<=impl_->n;++k) t.p.row(k).head<3>()+=translation.transpose();
  t.rotation0=rotation.normalized().toRotationMatrix()*t.rotation0;
}

} // namespace rb_servo::control
