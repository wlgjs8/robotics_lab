#include "plaif/moforcecontroller/algorithm.h"

double safety_ori_amp = 1.0;
double imp_ori_amp = 5.0;
double adm_ori_amp = 2.0;

// double adm_ori_amp_pitch = 50.0;

namespace plaif::moforcecontroller {
CForceController::CForceController(Type controller_type)
    : Ts(0.002),
      reference_data{},
      feedback_data{},
      current_target{},
      controller_result{},
      controller_flag{},
      measurement_data{},
      delta_tcp{},
      delta_tcp_dot{},
      variable_b{},
      variable_k{},
      variable_tankE{},
      compensation_type(Type::kSafetedExternalForce) {
  compensation_type = controller_type;
  if (this->Initialize(compensation_type)) {
  } else {
  }
}

CForceController::~CForceController() {}

bool CForceController::Initialize(Type controller_type) {
  utils = new ControllerUtils();
  this->CreateStateModel(controller_type);
  this->InitController();

  return true;
}

void CForceController::CreateStateModel(Type controller_type) {
  if (controller_type == Type::kEnduringReferenceForce) {
    double k_gain_ = 500.0;
    double b_gain_ = 40.0;
    double k_gain_o_ = 10.0;
    double b_gain_o_ = 1.5;

    double tank_energy_ = 100.0;

    variable_k = {k_gain_, k_gain_, k_gain_, k_gain_o_, k_gain_o_, k_gain_o_};
    variable_b = {b_gain_, b_gain_, b_gain_, b_gain_o_, b_gain_o_, b_gain_o_};
    variable_tankE = {tank_energy_, tank_energy_, tank_energy_, tank_energy_, tank_energy_, tank_energy_};

    this->CreateStateSpace(1.5, k_gain_, b_gain_, x_A, x_B, x_C, x_E, x_D, x_L, x_K_lqr, x_k_int);
    this->CreateStateSpace(1.5, k_gain_, b_gain_, y_A, y_B, y_C, y_E, y_D, y_L, y_K_lqr, y_k_int);
    this->CreateStateSpace(1.5, k_gain_, b_gain_, z_A, z_B, z_C, z_E, z_D, z_L, z_K_lqr, z_k_int);
    this->CreateStateSpace(0.1, k_gain_o_, b_gain_o_, roll_A, roll_B, roll_C, roll_E, roll_D, roll_L, roll_K_lqr, roll_k_int);
    this->CreateStateSpace(0.1, k_gain_o_, b_gain_o_, pitch_A, pitch_B, pitch_C, pitch_E, pitch_D, pitch_L, pitch_K_lqr, pitch_k_int);
    this->CreateStateSpace(0.1, k_gain_o_, b_gain_o_, yaw_A, yaw_B, yaw_C, yaw_E, yaw_D, yaw_L, yaw_K_lqr, yaw_k_int);
  } else if (controller_type == Type::kSafetedExternalForce) {
    this->CreateStateSpaceInIntegrator(1.0, 800, 50.91, x_A, x_B, x_C, x_E, x_D, x_L, x_K_lqr, x_k_int);  // 56.57
    this->CreateStateSpaceInIntegrator(1.0, 800, 50.91, y_A, y_B, y_C, y_E, y_D, y_L, y_K_lqr, y_k_int);
    this->CreateStateSpaceInIntegrator(1.0, 800, 50.91, z_A, z_B, z_C, z_E, z_D, z_L, z_K_lqr, z_k_int);
    this->CreateStateSpaceInIntegrator(0.1, 10.0, 1.5, roll_A, roll_B, roll_C, roll_E, roll_D, roll_L, roll_K_lqr, roll_k_int);
    this->CreateStateSpaceInIntegrator(0.1, 10.0, 1.5, pitch_A, pitch_B, pitch_C, pitch_E, pitch_D, pitch_L, pitch_K_lqr, pitch_k_int);
    this->CreateStateSpaceInIntegrator(0.1, 10.0, 1.5, yaw_A, yaw_B, yaw_C, yaw_E, yaw_D, yaw_L, yaw_K_lqr, yaw_k_int);
  } else if (controller_type == Type::kPoseErrorBasedForce) {
    imp_params_ = {{
        {0.0, 2000.0, 0.0},
        {0.0, 2000.0, 0.0},
        {0.0, 2000.0, 0.0},
        {0.0, 50.0, 0.0},
        {0.0, 50.0, 0.0},
        {0.0, 50.0, 0.0},
    }};

    double k_gain_ = 500.0;
    double b_gain_ = 54.76;
    double k_gain_o_ = 10.0;
    double b_gain_o_ = 1.5;

    double tank_energy_ = 100.0;

    variable_k = {k_gain_, k_gain_, k_gain_, k_gain_o_, k_gain_o_, k_gain_o_};
    variable_b = {b_gain_, b_gain_, b_gain_, b_gain_o_, b_gain_o_, b_gain_o_};
    variable_tankE = {tank_energy_, tank_energy_, tank_energy_, tank_energy_, tank_energy_, tank_energy_};

    this->CreateStateSpace(1.5, k_gain_, b_gain_, x_A, x_B, x_C, x_E, x_D, x_L, x_K_lqr, x_k_int);
    this->CreateStateSpace(1.5, k_gain_, b_gain_, y_A, y_B, y_C, y_E, y_D, y_L, y_K_lqr, y_k_int);
    this->CreateStateSpace(1.5, k_gain_, b_gain_, z_A, z_B, z_C, z_E, z_D, z_L, z_K_lqr, z_k_int);
    this->CreateStateSpace(0.1, k_gain_o_, b_gain_o_, roll_A, roll_B, roll_C, roll_E, roll_D, roll_L, roll_K_lqr, roll_k_int);
    this->CreateStateSpace(0.1, k_gain_o_, b_gain_o_, pitch_A, pitch_B, pitch_C, pitch_E, pitch_D, pitch_L, pitch_K_lqr, pitch_k_int);
    this->CreateStateSpace(0.1, k_gain_o_, b_gain_o_, yaw_A, yaw_B, yaw_C, yaw_E, yaw_D, yaw_L, yaw_K_lqr, yaw_k_int);

  }
}

void CForceController::SetImpedanceParam(Axis axis, double m, double k, double b) { imp_params_[axis] = {m, k, b}; }
void CForceController::SetImpedanceParams(const std::array<ImpedanceParam, 6>& params) { imp_params_ = params; }

void CForceController::CreateStateSpace(double m, double k, double b, Eigen::MatrixXd& A_d, Eigen::MatrixXd& B_d, Eigen::MatrixXd& C_d, Eigen::MatrixXd& E_d, Eigen::MatrixXd& D_d,
                                        Eigen::VectorXd& L_observer, Eigen::RowVectorXd& K_lqr, Eigen::RowVectorXd& K_int) {
  STATE_DIM = 3;
  INPUT_DIM = 1;
  OUTPUT_DIM = 1;

  // Mass Spring Damper System ==================================================
  Eigen::MatrixXd A_ct(STATE_DIM, STATE_DIM);
  A_ct.setZero();
  A_ct(0, 1) = 1.0;
  A_ct(1, 0) = -k / m;
  A_ct(1, 1) = -b / m;
  A_ct(1, 2) = -1 / m;

  Eigen::MatrixXd B_ct(STATE_DIM, INPUT_DIM);
  B_ct.setZero();
  B_ct(1, 0) = 1.0 / m;

  Eigen::MatrixXd C_ct(OUTPUT_DIM, STATE_DIM);
  C_ct.setZero();
  C_ct(0, 0) = 1.0;

  Eigen::MatrixXd E_ct(STATE_DIM, OUTPUT_DIM);
  E_ct.setZero();
  E_ct(1, 0) = 1.0 / m;

  Eigen::MatrixXd D_ct = Eigen::MatrixXd::Zero(1, 1);

  Eigen::MatrixXd A_dt(STATE_DIM, STATE_DIM);
  Eigen::MatrixXd B_dt(STATE_DIM, INPUT_DIM);
  Eigen::MatrixXd C_dt(OUTPUT_DIM, STATE_DIM);
  Eigen::MatrixXd E_dt(STATE_DIM, INPUT_DIM);
  Eigen::MatrixXd D_dt(OUTPUT_DIM, INPUT_DIM);

  utils->c2d_multiInput(A_ct, B_ct, E_ct, Ts, A_dt, B_dt, E_dt);
  // utils->c2d(A_ct, B_ct, Ts, A_dt, B_dt);
  C_dt = C_ct;
  D_dt = D_ct;

  A_d = A_dt;
  B_d = B_dt;
  C_d = C_dt;
  D_d = D_dt;
  E_d = E_dt;

  // Integrator-------------------------------------------
  //  Eigen::MatrixXd A_int, B_int;
  Eigen::MatrixXd A_int(STATE_DIM + 1, STATE_DIM + 1);
  A_int.setZero();
  A_int.block(0, 0, 3, 3) = A_dt;
  A_int(3, 3) = 1.0;
  A_int(3, 0) = -1.0;

  Eigen::MatrixXd B_int(STATE_DIM + 1, 1);
  B_int.setZero();
  B_int.block(0, 0, 3, 1) = B_dt;

  Eigen::MatrixXd Q_i = Eigen::MatrixXd::Identity(STATE_DIM + 1, STATE_DIM + 1);
  Q_i(0, 0) = 1.5;  // x1 가중
  Q_i(1, 1) = 1.0;  // x2 가중
  Q_i(2, 2) = 1.0;  // 외란 x3 가중
  Q_i(3, 3) = 0.3;  // 적분 상태 가중
  Eigen::MatrixXd R_i = Eigen::MatrixXd::Identity(INPUT_DIM, INPUT_DIM);
  R_i(0, 0) = 1;

  Eigen::MatrixXd Ki = utils->ComputeDiscreteLQRSolution(A_int, B_int, Q_i, R_i);
  K_int = Ki;
  //---------------------------------------------------------

  // LQR 제어 게인 계산
  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
  Q(0, 0) = 1.5;  // 위치(x1) 가중
  Q(1, 1) = 1.0;  // 속도(x2) 가중
  Q(2, 2) = 1.0;  // 외란(x3) 가중
  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(INPUT_DIM, INPUT_DIM);
  R(0, 0) = 1;

  Eigen::MatrixXd K_ = utils->ComputeDiscreteLQRSolution(A_dt, B_dt, Q, R);
  K_lqr = K_;

  // 관측기 설계
  Eigen::MatrixXd A_dt_t = A_dt.transpose();
  Eigen::MatrixXd C_dt_t = C_dt.transpose();

  Eigen::MatrixXd Q_o = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
  Q_o(0, 0) = 1.5;  // 위치(x1) 관측 가중
  Q_o(1, 1) = 1.0;  // 속도(x2) 관측 가중
  Q_o(2, 2) = 1.0;  // 외란(x3) 관측 가중
  Eigen::MatrixXd R_o = Eigen::MatrixXd::Identity(INPUT_DIM, INPUT_DIM);
  R_o(0, 0) = 1;

  Eigen::MatrixXd L_temp = utils->ComputeDiscreteLQRSolution(A_dt_t, C_dt_t, Q_o, R_o);
  L_observer = L_temp.transpose();
}

double CForceController::UpdateStateSpace(const double ref_force, const double feedback_force, const double measured, Eigen::VectorXd& x_hat, Eigen::VectorXd& x_, const Eigen::MatrixXd A_d,
                                          const Eigen::MatrixXd B_d, const Eigen::MatrixXd C_d, Eigen::MatrixXd E_d, const Eigen::MatrixXd D_d, const Eigen::VectorXd L_observer,
                                          const Eigen::RowVectorXd K_lqr, const Eigen::RowVectorXd& K_int, Eigen::VectorXd& d_hat, double& integrator, double& error, double d_hat_amp,
                                          double& force_err_f, double forward) {  //
  double force_err = feedback_force - (ref_force);
  double k_p = 0.03;
  double K_i = K_int(3);

  // if (std::fabs(force_err) < 1.5) { force_err = 0.0; }

  integrator += (Ts * 0.25) * (force_err);

  double u_ = (k_p * force_err) - (K_i * integrator) - d_hat_amp * d_hat(2);
  double u = u_ + forward;

  x_ = A_d * x_ + B_d * u;
  double y = (C_d * x_)(0);

  double yhat = (C_d * x_hat)(0);

  x_hat = A_d * x_hat + B_d * u + L_observer * (measured - yhat);

  d_hat = x_hat;

  return y;
}

double CForceController::UpdateStateSpaceT(const double ref_force, const double feedback_force, const double measured, Eigen::VectorXd& x_hat, Eigen::VectorXd& x_, const Eigen::MatrixXd A_d,
                                           const Eigen::MatrixXd B_d, const Eigen::MatrixXd C_d, Eigen::MatrixXd E_d, const Eigen::MatrixXd D_d, const Eigen::VectorXd L_observer,
                                           const Eigen::RowVectorXd K_lqr, const Eigen::RowVectorXd& K_int, Eigen::VectorXd& d_hat, double& integrator, double& error, double d_hat_amp,
                                           double& force_err_f, double forward) {  //
  double force_err = feedback_force - (ref_force);
  double k_p = 0.01;
  double K_i = K_int(3);

  const double leak = 0.998;
  integrator = leak * integrator + (Ts * 0.25) * force_err;

  constexpr double integrator_limit = 50.0;
  integrator = std::clamp(integrator, -integrator_limit, integrator_limit);

  double u_ = (k_p * force_err) - (K_i * integrator) - d_hat_amp * d_hat(2);
  double u = u_ + forward;

  x_ = A_d * x_ + B_d * u;
  double y = (C_d * x_)(0);

  double yhat = (C_d * x_hat)(0);

  x_hat = A_d * x_hat + B_d * u + L_observer * (measured - yhat);

  d_hat = x_hat;

  return y;
}

double CForceController::UpdateStateSpaceO(const double ref_force, const double feedback_force, const double measured, Eigen::VectorXd& x_hat, Eigen::VectorXd& x_, const Eigen::MatrixXd A_d,
                                           const Eigen::MatrixXd B_d, const Eigen::MatrixXd C_d, Eigen::MatrixXd E_d, const Eigen::MatrixXd D_d, const Eigen::VectorXd L_observer,
                                           const Eigen::RowVectorXd K_lqr, const Eigen::RowVectorXd& K_int, Eigen::VectorXd& d_hat, double& integrator, double& error, double d_hat_amp,
                                           double& force_err_f, double forward) {  //
  double force_err = feedback_force - (ref_force);
  double k_p = 0.005;
  double K_i = K_int(3);

  const double leak = 0.995;
  integrator = leak * integrator + (Ts * 0.25) * force_err;

  constexpr double integrator_limit = 3.0;
  integrator = std::clamp(integrator, -integrator_limit, integrator_limit);

  double u_ = (k_p * force_err) - (K_i * integrator) - d_hat_amp * d_hat(2);
  double u = u_ + forward;

  x_ = A_d * x_ + B_d * u;
  double y = (C_d * x_)(0);

  double yhat = (C_d * x_hat)(0);

  x_hat = A_d * x_hat + B_d * u + L_observer * (measured - yhat);
  d_hat = x_hat;

  return y;
}

void CForceController::CreateStateSpaceInIntegrator(double m, double k, double b, Eigen::MatrixXd& A_d, Eigen::MatrixXd& B_d, Eigen::MatrixXd& C_d, Eigen::MatrixXd& E_d, Eigen::MatrixXd& D_d,
                                                    Eigen::VectorXd& L_observer, Eigen::RowVectorXd& K_lqr, Eigen::RowVectorXd& K_int) {
  STATE_DIM = 4;
  INPUT_DIM = 1;
  OUTPUT_DIM = 1;

  // Mass Spring Damper System ==================================================
  Eigen::MatrixXd A_ct(STATE_DIM, STATE_DIM);
  A_ct.setZero();
  A_ct(0, 1) = 1.0;
  A_ct(1, 0) = -k / m;
  A_ct(1, 1) = -b / m;
  A_ct(1, 2) = -1 / m;

  A_ct(3, 0) = 1.0;

  Eigen::MatrixXd B_ct(STATE_DIM, INPUT_DIM);
  B_ct.setZero();
  B_ct(1, 0) = 1.0 / m;

  Eigen::MatrixXd C_ct(OUTPUT_DIM, STATE_DIM);
  C_ct.setZero();
  C_ct(0, 0) = 1.0;

  Eigen::MatrixXd E_ct(STATE_DIM, OUTPUT_DIM);
  E_ct.setZero();
  E_ct(1, 0) = 1.0 / m;

  Eigen::MatrixXd D_ct = Eigen::MatrixXd::Zero(1, 1);

  Eigen::MatrixXd A_dt(STATE_DIM, STATE_DIM);
  Eigen::MatrixXd B_dt(STATE_DIM, INPUT_DIM);
  Eigen::MatrixXd C_dt(OUTPUT_DIM, STATE_DIM);
  Eigen::MatrixXd E_dt(STATE_DIM, INPUT_DIM);
  Eigen::MatrixXd D_dt(OUTPUT_DIM, INPUT_DIM);

  // utils->c2d(A_ct, B_ct, Ts, A_dt, B_dt);
  utils->c2d_multiInput(A_ct, B_ct, E_ct, Ts, A_dt, B_dt, E_dt);
  C_dt = C_ct;
  D_dt = D_ct;

  A_d = A_dt;
  B_d = B_dt;
  C_d = C_dt;
  D_d = D_dt;
  E_d = E_dt;

  // LQR 제어 게인 계산
  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
  Q(0, 0) = 10.0;  // 위치(x1) 가중
  Q(1, 1) = 10.0;  // 속도(x2) 가중
  Q(2, 2) = 1.0;   // 외란(x3) 가중
  Q(3, 3) = 1.0;   // 적분(x4) 가중 (적절히 조정 가능)
  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(INPUT_DIM, INPUT_DIM);
  R(0, 0) = 1.0;

  Eigen::MatrixXd K_ = utils->ComputeDiscreteLQRSolution(A_dt, B_dt, Q, R);
  K_lqr = K_;

  // 관측기 설계
  Eigen::MatrixXd A_dt_t = A_dt.transpose();
  Eigen::MatrixXd C_dt_t = C_dt.transpose();

  Eigen::MatrixXd Q_o = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
  // dist, xI 추정을 중요시할 수도 있음
  Q_o(0, 0) = 10.0;  // 위치(x1) 관측 가중
  Q_o(1, 1) = 10.0;  // 속도(x2) 관측 가중
  Q_o(2, 2) = 1.0;   // 외란(x3) 관측 가중
  Q_o(3, 3) = 1.0;   // 적분(x4) 관측 가중
  Eigen::MatrixXd R_o = Eigen::MatrixXd::Identity(INPUT_DIM, INPUT_DIM);
  R_o(0, 0) = 1.0;

  Eigen::MatrixXd L_temp = utils->ComputeDiscreteLQRSolution(A_dt_t, C_dt_t, Q_o, R_o);
  L_observer = L_temp.transpose();
}

double CForceController::UpdateStateSpaceInIntegrator(const double ref_force, const double feedback_force, const double measured, Eigen::VectorXd& x_hat, Eigen::VectorXd& x_,
                                                      const Eigen::MatrixXd A_d, const Eigen::MatrixXd B_d, const Eigen::MatrixXd C_d, Eigen::MatrixXd E_d, const Eigen::MatrixXd D_d,
                                                      const Eigen::VectorXd L_observer, const Eigen::RowVectorXd K_lqr, const Eigen::RowVectorXd& K_int, Eigen::VectorXd& d_hat, double& integrator,
                                                      double& error) {
  const double force_err = feedback_force - ref_force;
  const double u_fb = -(K_lqr * x_hat)(0, 0);
  const double u = force_err + u_fb;

  x_ = A_d * x_ + B_d * u;  // + E_d * force_err;
  const double y = (C_d * x_ + D_d * u)(0);

  const double yhat = (C_d * x_hat)(0);
  x_hat = A_d * x_hat + B_d * u + L_observer * (measured - yhat);  // + E_d * force_err

  d_hat = x_hat;

  return y;
}

double CForceController::FilteredBasicMassSpringDamperT(const ImpedanceParam& param, const double ref_pose, const double feedback_pose, const double feedback_force, double& error, double& error_d,
                                                        double& out_prev) {
  const double pose_err = ref_pose - feedback_pose;

  const double M = param.m;
  const double K = param.k;
  const double B = param.b;

  const double fc = 20.0;

  const double w = 2.0 * M_PI * fc * Ts;
  const double a = w / (1.0 + w);

  const double raw_vel = (pose_err - error) / Ts;
  const double vel_err = a * raw_vel + (1.0 - a) * error_d;

  const double acc_err = (vel_err - error_d) / Ts;

  error = pose_err;
  error_d = vel_err;

  const double F_cmd = M * acc_err + B * raw_vel + K * pose_err;

  return F_cmd;
}

double CForceController::FilteredBasicMassSpringDamperO(const ImpedanceParam& param, const double transf_pose_err, const double feedback_force, double& error, double& error_d, double& out_prev) {
  const double pose_err = transf_pose_err;

  const double M = param.m;
  const double K = param.k;
  const double B = param.b;

  const double raw_vel = (pose_err - error) / Ts;
  const double acc_err = (raw_vel - error_d) / Ts;

  error = pose_err;
  error_d = raw_vel;

  const double F_cmd = M * acc_err + B * raw_vel + K * pose_err;
  return F_cmd;
}

void CForceController::AdaptiveControl(const int type, const double F_err, const double K_min, const double K_max, const double B_min, const double B_max, const double M, const double K,
                                       const double B, const double x, const double x_dot, double& dK_des, double& dB_des) {
  if (type == MRAC) {
    const double a = 5.0;
    Eigen::MatrixXd Am(1, 1);
    Am(0, 0) = -a;
    Eigen::MatrixXd Qm(1, 1);
    Qm(0, 0) = 1.0;
    Eigen::MatrixXd In(1, 1);
    In(0, 0) = 1.0;
    Eigen::MatrixXd Pm(1, 1);
    double res = 0.0;
    utils->SolveLyapunovEquation(1, 1, Am, -Qm, In, Pm, res);
    const double P11 = Pm(0, 0);

    const double eps_F = 0.1;
    double eF_dz = 0.0;
    if (std::fabs(F_err) > eps_F) {
      eF_dz = (std::fabs(F_err) - eps_F) * ((F_err >= 0.0) ? 1.0 : -1.0);
    }

    if (eF_dz == 0.0) {
      dK_des = 0.0;
      dB_des = 0.0;
      return;
    }
    const double kappa = 1.0;
    const double sigma_e = kappa * P11 * eF_dz;

    const double c_norm = 5.0;
    const double eta = 1.0 / (1.0 + c_norm * (x * x + x_dot * x_dot));

    static double gK = 100.0;
    static double gB = 80.0;
    const double lambda_k = 0.05;
    const double lambda_b = 0.02;
    double dK_raw = -(gK * (x - lambda_k)) * sigma_e * Ts * eta;
    double dB_raw = -(gB * (x_dot - lambda_b)) * sigma_e * Ts * eta;

    // Projection operator
    auto project_component = [](double theta, double dtheta, double th_min, double th_max, double eps = 1e-9) {
      if ((theta <= th_min + eps && dtheta < 0.0) || (theta >= th_max - eps && dtheta > 0.0)) {
        return 0.0;
      }
      return dtheta;
    };
    dK_raw = project_component(K, dK_raw, K_min, K_max);
    dB_raw = project_component(B, dB_raw, B_min, B_max);

    dK_des = dK_raw;
    dB_des = dB_raw;
  } else if (type == SCHEDULING) {
    const double kK_slope = 1.0;  // dK_des = -kK_slope*|F_err| [N/m per N]
    double dK_des_raw = -kK_slope * std::fabs(F_err);

    // 목표 K (클램프 반영)
    double K_des = std::clamp(K + dK_des_raw, K_min, K_max);
    dK_des = K_des - K;  // clamp 반영된 희망 변화

    // 목표 B: ζ=1
    double K_for_B = std::max(K_des, 1e-9);
    double B_target = 2.0 * 1.0 * std::sqrt(std::max(0.0, M * K_for_B));  // ζ=1
    B_target = std::clamp(B_target, B_min, B_max);
    dB_des = B_target - B;
  }
}

double CForceController::EnergyTankStepKB(const double M, double& K, double& B,
                                          double x,      // MUST be pose error (deformation)
                                          double x_dot,  // MUST be error velocity
                                          double F_fb, double F_ref,
                                          double& tankE) {  // [W] 전력 상한
  // tankE = 100
  // ─────────────────────────────────────────────────────────────
  // 0) 1회 초기화: 명목값 저장(클램프 기준) + 파라미터
  // ─────────────────────────────────────────────────────────────
  static bool init = false;
  static double K_nom = 0.0;
  static double B_nom = 0.0;
  static double K_min = 0.0, K_max = 0.0;
  static double B_min = 0.0, B_max = 0.0;

  constexpr double CLAMP_K_PCT = 0.9;
  constexpr double CLAMP_B_PCT = 0.9;

  constexpr double DLIM_K_PCT = 0.02;
  constexpr double DLIM_B_PCT = 0.02;

  constexpr double cB_cost = 2.0;

  constexpr double T_min = 0.1;     // [J]
  constexpr double k_leak = 0.002;  // [W]
  constexpr double P_max = 180.0;   // [W]

  if (!init) {
    K_nom = K;
    B_nom = B;
    K_min = (1.0 - CLAMP_K_PCT) * K_nom;
    K_max = (1.0 + CLAMP_K_PCT) * K_nom;
    B_min = (1.0 - CLAMP_B_PCT) * B_nom;
    B_max = (1.0 + CLAMP_B_PCT) * B_nom;
    init = true;
  }

  const double eps = 1e-12;
  const double x2 = x * x;
  const double v2 = x_dot * x_dot;

  // ─────────────────────────────────────────────────────────────
  // 1) Adaptive Control for K, B
  // ─────────────────────────────────────────────────────────────
  double dB_des;
  double dK_des;
  const double F_err = F_ref - F_fb;

  // MRAC, SCHEDULING
  this->AdaptiveControl(MRAC, F_err, K_min, K_max, B_min, B_max, M, K, B, x, x_dot, dK_des, dB_des);

  // ─────────────────────────────────────────────────────────────
  // 2) rate-limit (step당 변화 제한)
  // ─────────────────────────────────────────────────────────────
  const double dK_step_max = DLIM_K_PCT * std::max(std::fabs(K_nom), eps);
  const double dB_step_max = DLIM_B_PCT * std::max(std::fabs(B_nom), eps);
  dK_des = std::clamp(dK_des, -dK_step_max, dK_step_max);
  dB_des = std::clamp(dB_des, -dB_step_max, dB_step_max);

  // ─────────────────────────────────────────────────────────────
  // 3) 에너지 장부(출금/환급 잠정 계산)
  // ─────────────────────────────────────────────────────────────
  double E_need = 0.0;          // 출금(양수)
  double E_refund_sched = 0.0;  // 환급(양수)
  if (dK_des > 0.0)
    E_need += 0.5 * dK_des * x2;
  else
    E_refund_sched += 0.5 * (-dK_des) * x2;
  if (dB_des < 0.0) E_need += cB_cost * (-dB_des) * v2 * Ts;

  double available = std::max(0.0, tankE - T_min);
  double scale = 1.0;
  if (E_need > available && E_need > 0.0) {
    scale = available / E_need;
  }
  scale = std::clamp(scale, 0.0, 1.0);

  double dK_sc = scale * dK_des;
  double dB_sc = scale * dB_des;

  // ─────────────────────────────────────────────────────────────
  // 4) 최종 적용(클램프 재반영) + 실제 적용량 기준으로 에너지 재계산
  // ─────────────────────────────────────────────────────────────
  double K_new = std::clamp(K + dK_sc, K_min, K_max);
  double B_new = std::clamp(B + dB_sc, B_min, B_max);

  const double dK_applied = K_new - K;
  const double dB_applied = B_new - B;

  double E_withdraw = 0.0;
  double E_refund = 0.0;
  if (dK_applied > 0.0)
    E_withdraw += 0.5 * dK_applied * x2;
  else
    E_refund += 0.5 * (-dK_applied) * x2;
  if (dB_applied < 0.0) E_withdraw += cB_cost * (-dB_applied) * v2 * Ts;

  K = K_new;
  B = B_new;

  // ─────────────────────────────────────────────────────────────
  // 5) 탱크 에너지 갱신
  // ─────────────────────────────────────────────────────────────
  const double P_diss = B * v2;
  const double P_leakW = (tankE > T_min) ? k_leak * (tankE - T_min) : 0.0;

  tankE += P_diss * Ts;
  tankE += E_refund;
  tankE -= E_withdraw;
  tankE -= P_leakW * Ts;
  tankE = std::max(tankE, T_min);

  // ─────────────────────────────────────────────────────────────
  // 6) FF 힘 계산 (★소프트 전력 리미터: β 추가★)
  // ─────────────────────────────────────────────────────────────
  static double Fff_prev = 0.0;
  static double Fcap_lp = 0.0;  // 전력기반 힘 상한의 저역 통과값

  // (a) 원시 FF
  const double Fff_raw = (K - K_nom) * x + (B - B_nom) * x_dot;

  // (b) 전력기반 힘 상한의 "연속" 계산
  const double v_eps = 0.01;                               // [m/s] 분모 보호
  double F_cap_inst = P_max / (std::fabs(x_dot) + v_eps);  // [N]
  const double TAU_CAP = 0.006;                            // [s] 네가 설정한 6 ms 유지
  const double a_cap = Ts / (TAU_CAP + Ts);
  if (Fcap_lp <= 0.0)
    Fcap_lp = F_cap_inst;
  else
    Fcap_lp += a_cap * (F_cap_inst - Fcap_lp);

  // (c) 소프트 리미터: tanh에 β 스케일 추가 (★여기만 변경★)
  const double BETA_SOFT = 1.5;  // ⇐ 튜닝 노브(1.3 ~ 2.0 권장)
  double F_soft = Fcap_lp * std::tanh(Fff_raw / (BETA_SOFT * Fcap_lp + eps));

  // (d) FF 자체 1차 저역 (원 설정)
  constexpr double TAU_FF = 0.001;  // [s]
  const double alpha = Ts / (TAU_FF + Ts);
  double F_lpf = Fff_prev + alpha * (F_soft - Fff_prev);

  // (e) (옵션) 출력 레이트리밋 — 필요시만 사용
  const double dFmax = 800.0;  // [N/s] 400~800 범위로 조정
  double dF = std::clamp(F_lpf - Fff_prev, -dFmax * Ts, dFmax * Ts);
  double F_ff = Fff_prev + dF;

  Fff_prev = F_ff;

  // 모니터링
  const double P_port = F_ff * x_dot;

  return F_ff;
}

void CForceController::Calculation() {
  if (compensation_type == Type::kEnduringReferenceForce) {
    this->EnduringReferenceForceControl();
  } else if (compensation_type == Type::kPoseErrorBasedForce) {
    this->PoseErrorBasedCreateForce();
  } else if (compensation_type == Type::kSafetedExternalForce) {
    this->SafetedExternalForceControl();
  }
}

void CForceController::EnduringReferenceForceControlImp() {
  Wrench F_add = {};
  controller_result.pose = {};

  if (controller_flag[0] == 1) {
    // F_add.force.x = this->EnergyTankStepKB(3.0, variable_k.force.x, variable_b.force.x, delta_tcp.pose.x, delta_tcp_dot.pose.x, feedback_data.wrench.force.x, reference_data.wrench.force.x,
    //                                        variable_tankE.force.x);
    controller_result.pose.x = this->UpdateStateSpaceT(reference_data.wrench.force.x, feedback_data.wrench.force.x, measurement_data.pose.x, x_x_old, x_x_new, x_A, x_B, x_C, x_E, x_D, x_L, x_K_lqr,
                                                       x_k_int, x_d_hat, x_integrator, x_error_d1, d_hat_amp_x, force_err_f_x, F_add.force.x);
  }
  if (controller_flag[1] == 1) {
    // F_add.force.y = this->EnergyTankStepKB(1.0, variable_k.force.y, variable_b.force.y, delta_tcp.pose.y, delta_tcp_dot.pose.y, feedback_data.wrench.force.y, reference_data.wrench.force.y,
    //                                        variable_tankE.force.y);
    controller_result.pose.y = this->UpdateStateSpaceT(reference_data.wrench.force.y, feedback_data.wrench.force.y, measurement_data.pose.y, y_x_old, y_x_new, y_A, y_B, y_C, y_E, y_D, y_L, y_K_lqr,
                                                       y_k_int, y_d_hat, y_integrator, y_error_d1, d_hat_amp_y, force_err_f_y, F_add.force.y);
  }
  if (controller_flag[2] == 1) {
    // F_add.force.z = this->EnergyTankStepKB(1.0, variable_k.force.z, variable_b.force.z, delta_tcp.pose.z, delta_tcp_dot.pose.z, feedback_data.wrench.force.z, reference_data.wrench.force.z,
    //                                        variable_tankE.force.z);
    controller_result.pose.z = this->UpdateStateSpaceT(reference_data.wrench.force.z, feedback_data.wrench.force.z, measurement_data.pose.z, z_x_old, z_x_new, z_A, z_B, z_C, z_E, z_D, z_L, z_K_lqr,
                                                       z_k_int, z_d_hat, z_integrator, z_error_d1, d_hat_amp_z, force_err_f_z, F_add.force.z);
  }
  if (controller_flag[3] == 1) {
    // F_add.torque.x = this->EnergyTankStepKB(0.01, variable_k.torque.x, variable_b.torque.x, delta_tcp.pose.roll, delta_tcp_dot.pose.roll, feedback_data.wrench.torque.x,
    // reference_data.wrench.torque.x, variable_tankE.torque.x);
    controller_result.pose.roll =
        this->UpdateStateSpaceO(reference_data.wrench.torque.x, feedback_data.wrench.torque.x, measurement_data.pose.roll, roll_x_old, roll_x_new, roll_A, roll_B, roll_C, roll_E, roll_D, roll_L,
                                roll_K_lqr, roll_k_int, roll_d_hat, roll_integrator, roll_error_d1, d_hat_amp_roll, force_err_f_roll, F_add.torque.x) *
        imp_ori_amp;
  }
  if (controller_flag[4] == 1) {
    // F_add.torque.y = this->EnergyTankStepKB(0.01, variable_k.torque.y, variable_b.torque.y, delta_tcp.pose.pitch, delta_tcp_dot.pose.pitch, feedback_data.wrench.torque.y,
    //                                         reference_data.wrench.torque.y, variable_tankE.torque.y);
    controller_result.pose.pitch =
        this->UpdateStateSpaceO(reference_data.wrench.torque.y, feedback_data.wrench.torque.y, measurement_data.pose.pitch, pitch_x_old, pitch_x_new, pitch_A, pitch_B, pitch_C, pitch_E, pitch_D,
                                pitch_L, pitch_K_lqr, pitch_k_int, pitch_d_hat, pitch_integrator, pitch_error_d1, d_hat_amp_pitch, force_err_f_pitch, F_add.torque.y) *
        imp_ori_amp;
  }
  if (controller_flag[5] == 1) {
    // F_add.torque.z = this->EnergyTankStepKB(0.01, variable_k.torque.z, variable_b.torque.z, delta_tcp.pose.yaw, delta_tcp_dot.pose.yaw, feedback_data.wrench.torque.z,
    // reference_data.wrench.torque.z, variable_tankE.torque.z);
    controller_result.pose.yaw = this->UpdateStateSpaceO(reference_data.wrench.torque.z, feedback_data.wrench.torque.z, measurement_data.pose.yaw, yaw_x_old, yaw_x_new, yaw_A, yaw_B, yaw_C, yaw_E,
                                                         yaw_D, yaw_L, yaw_K_lqr, yaw_k_int, yaw_d_hat, yaw_integrator, yaw_error_d1, d_hat_amp_yaw, force_err_f_yaw, F_add.torque.z) *
                                 imp_ori_amp;
  }
}

void CForceController::EnduringReferenceForceControl() {
  Wrench F_add = {};
  controller_result.pose = {};

  if (controller_flag[0] == 1) {
    controller_result.pose.x = this->UpdateStateSpace(reference_data.wrench.force.x, feedback_data.wrench.force.x, measurement_data.pose.x, x_x_old, x_x_new, x_A, x_B, x_C, x_E, x_D, x_L, x_K_lqr,
                                                      x_k_int, x_d_hat, x_integrator, x_error_d1, d_hat_amp_x, force_err_f_x, F_add.force.x);
  }
  if (controller_flag[1] == 1) {
    controller_result.pose.y = this->UpdateStateSpace(reference_data.wrench.force.y, feedback_data.wrench.force.y, measurement_data.pose.y, y_x_old, y_x_new, y_A, y_B, y_C, y_E, y_D, y_L, y_K_lqr,
                                                      y_k_int, y_d_hat, y_integrator, y_error_d1, d_hat_amp_y, force_err_f_y, F_add.force.y);
  }
  if (controller_flag[2] == 1) {
    controller_result.pose.z = this->UpdateStateSpace(reference_data.wrench.force.z, feedback_data.wrench.force.z, measurement_data.pose.z, z_x_old, z_x_new, z_A, z_B, z_C, z_E, z_D, z_L, z_K_lqr,
                                                      z_k_int, z_d_hat, z_integrator, z_error_d1, d_hat_amp_z, force_err_f_z, F_add.force.z);
  }
  if (controller_flag[3] == 1) {
    controller_result.pose.roll = this->UpdateStateSpace(reference_data.wrench.torque.x, feedback_data.wrench.torque.x, measurement_data.pose.roll, roll_x_old, roll_x_new, roll_A, roll_B, roll_C,
                                                         roll_E, roll_D, roll_L, roll_K_lqr, roll_k_int, roll_d_hat, roll_integrator, roll_error_d1, d_hat_amp_roll, force_err_f_roll, F_add.torque.x) *
                                  adm_ori_amp;
  }
  if (controller_flag[4] == 1) {
    controller_result.pose.pitch =
        this->UpdateStateSpace(reference_data.wrench.torque.y, feedback_data.wrench.torque.y, measurement_data.pose.pitch, pitch_x_old, pitch_x_new, pitch_A, pitch_B, pitch_C, pitch_E, pitch_D,
                               pitch_L, pitch_K_lqr, pitch_k_int, pitch_d_hat, pitch_integrator, pitch_error_d1, d_hat_amp_pitch, force_err_f_pitch, F_add.torque.y) *
        adm_ori_amp;
  }
  if (controller_flag[5] == 1) {
    controller_result.pose.yaw = this->UpdateStateSpace(reference_data.wrench.torque.z, feedback_data.wrench.torque.z, measurement_data.pose.yaw, yaw_x_old, yaw_x_new, yaw_A, yaw_B, yaw_C, yaw_E,
                                                        yaw_D, yaw_L, yaw_K_lqr, yaw_k_int, yaw_d_hat, yaw_integrator, yaw_error_d1, d_hat_amp_yaw, force_err_f_yaw, F_add.torque.z) *
                                 adm_ori_amp;
  }

}

void CForceController::PoseErrorBasedCreateForce() {
  Eigen::VectorXd result(6);

  const double ori_amp = 1.0;
  result(0) = this->FilteredBasicMassSpringDamperT(imp_params_[kX], reference_data.pose.x, feedback_data.pose.x, feedback_data.wrench.force.x, imp_x_error_d1, imp_x_error_dd, imp_F_prev_x);
  result(1) = this->FilteredBasicMassSpringDamperT(imp_params_[kY], reference_data.pose.y, feedback_data.pose.y, feedback_data.wrench.force.y, imp_y_error_d1, imp_y_error_dd, imp_F_prev_y);
  result(2) = this->FilteredBasicMassSpringDamperT(imp_params_[kZ], reference_data.pose.z, feedback_data.pose.z, feedback_data.wrench.force.z, imp_z_error_d1, imp_z_error_dd, imp_F_prev_z);
  result(3) = this->FilteredBasicMassSpringDamperO(imp_params_[kRoll], measurement_data.pose.roll, feedback_data.wrench.torque.x, imp_roll_error_d1, imp_roll_error_dd, imp_F_prev_roll) * ori_amp;
  result(4) = this->FilteredBasicMassSpringDamperO(imp_params_[kPitch], measurement_data.pose.pitch, feedback_data.wrench.torque.y, imp_pitch_error_d1, imp_pitch_error_dd, imp_F_prev_pitch) * ori_amp;
  result(5) = this->FilteredBasicMassSpringDamperO(imp_params_[kYaw], measurement_data.pose.yaw, feedback_data.wrench.torque.z, imp_yaw_error_d1, imp_yaw_error_dd, imp_F_prev_yaw) * ori_amp;

  Eigen::VectorXd current_target_(6);
  current_target_ << current_target.pose.x, current_target.pose.y, current_target.pose.z, current_target.pose.roll, current_target.pose.pitch, current_target.pose.yaw;

  Eigen::Quaterniond q_we = Eigen::AngleAxisd(current_target.pose.yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(current_target.pose.pitch, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(current_target.pose.roll, Eigen::Vector3d::UnitX());
  Eigen::Matrix3d R_we = q_we.toRotationMatrix();
  Eigen::Matrix3d R_ew = R_we.transpose();

  Eigen::Vector3d p_we(current_target.pose.x, current_target.pose.y, current_target.pose.z);

  Eigen::VectorXd imp_flag(6);
  imp_flag << controller_flag[0], controller_flag[1], controller_flag[2], controller_flag[3], controller_flag[4], controller_flag[5];

  Eigen::Matrix<double, 6, 1> weight = utils->BaseSelectionMatrix(imp_flag, current_target_);

  Eigen::Vector3d f_world(result(0) * weight(0), result(1) * weight(1), result(2) * weight(2));
  Eigen::Vector3d f_eef = R_ew * f_world;

  Eigen::Vector3d tau_body(result(3) * weight(3), result(4) * weight(4), result(5) * weight(5));


  controller_result.wrench.force.x = f_eef(0);
  controller_result.wrench.force.y = f_eef(1);
  controller_result.wrench.force.z = f_eef(2);
  controller_result.wrench.torque.x = tau_body(0);
  controller_result.wrench.torque.y = tau_body(1);
  controller_result.wrench.torque.z = tau_body(2);
}

void CForceController::SafetedExternalForceControl() {
  controller_result.pose = {};
  if (controller_flag[0] == 1)
    controller_result.pose.x = this->UpdateStateSpaceInIntegrator(reference_data.wrench.force.x, feedback_data.wrench.force.x, measurement_data.pose.x, x_x_old, x_x_new, x_A, x_B, x_C, x_E, x_D, x_L,
                                                                  x_K_lqr, x_k_int, x_d_hat, x_integrator, x_error_d1);
  if (controller_flag[1] == 1)
    controller_result.pose.y = this->UpdateStateSpaceInIntegrator(reference_data.wrench.force.y, feedback_data.wrench.force.y, measurement_data.pose.y, y_x_old, y_x_new, y_A, y_B, y_C, y_E, y_D, y_L,
                                                                  y_K_lqr, y_k_int, y_d_hat, y_integrator, y_error_d1);
  if (controller_flag[2] == 1)
    controller_result.pose.z = this->UpdateStateSpaceInIntegrator(reference_data.wrench.force.z, feedback_data.wrench.force.z, measurement_data.pose.z, z_x_old, z_x_new, z_A, z_B, z_C, z_E, z_D, z_L,
                                                                  z_K_lqr, z_k_int, z_d_hat, z_integrator, z_error_d1);
  if (controller_flag[3] == 1)
    controller_result.pose.roll = this->UpdateStateSpaceInIntegrator(reference_data.wrench.torque.x, feedback_data.wrench.torque.x, measurement_data.pose.roll, roll_x_old, roll_x_new, roll_A, roll_B,
                                                                     roll_C, roll_E, roll_D, roll_L, roll_K_lqr, roll_k_int, roll_d_hat, roll_integrator, roll_error_d1) *
                                  safety_ori_amp;
  if (controller_flag[4] == 1)
    controller_result.pose.pitch = this->UpdateStateSpaceInIntegrator(reference_data.wrench.torque.y, feedback_data.wrench.torque.y, measurement_data.pose.pitch, pitch_x_old, pitch_x_new, pitch_A,
                                                                      pitch_B, pitch_C, pitch_E, pitch_D, pitch_L, pitch_K_lqr, pitch_k_int, pitch_d_hat, pitch_integrator, pitch_error_d1) *
                                   safety_ori_amp;
  if (controller_flag[5] == 1)
    controller_result.pose.yaw = this->UpdateStateSpaceInIntegrator(reference_data.wrench.torque.z, feedback_data.wrench.torque.z, measurement_data.pose.yaw, yaw_x_old, yaw_x_new, yaw_A, yaw_B, yaw_C,
                                                                    yaw_E, yaw_D, yaw_L, yaw_K_lqr, yaw_k_int, yaw_d_hat, yaw_integrator, yaw_error_d1) *
                                 safety_ori_amp;
}

void CForceController::InitController() {
  utils->ResetEE6Cache();

  this->XAxisParamInit();
  this->YAxisParamInit();
  this->ZAxisParamInit();

  this->RollAxisParamInit();
  this->PitchAxisParamInit();
  this->YawAxisParamInit();
  controller_result = {};
}

void CForceController::ZAxisParamInit() {
  z_x_old = Eigen::VectorXd::Zero(STATE_DIM);
  z_x_new = Eigen::VectorXd::Zero(STATE_DIM);
  z_d_hat = Eigen::VectorXd::Zero(STATE_DIM);
  z_integrator = 0.0;
  z_error_d1 = 0.0;
  z_error_dd = 0.0;

  d_hat_amp_z = 1.0;
  imp_F_prev_z = 0.0;
  force_err_f_z = 0.0;

  imp_z_error_d1 = 0.0;
  imp_z_error_dd = 0.0;
}

void CForceController::XAxisParamInit() {
  x_x_old = Eigen::VectorXd::Zero(STATE_DIM);
  x_x_new = Eigen::VectorXd::Zero(STATE_DIM);
  x_d_hat = Eigen::VectorXd::Zero(STATE_DIM);
  // x_d_hat = 0.0;
  x_integrator = 0.0;
  x_error_d1 = 0.0;
  x_error_dd = 0.0;

  d_hat_amp_x = 1.0;
  imp_F_prev_x = 0.0;
  force_err_f_x = 0.0;

  imp_x_error_d1 = 0.0;
  imp_x_error_dd = 0.0;
}

void CForceController::YAxisParamInit() {
  y_x_old = Eigen::VectorXd::Zero(STATE_DIM);
  y_x_new = Eigen::VectorXd::Zero(STATE_DIM);
  y_d_hat = Eigen::VectorXd::Zero(STATE_DIM);
  // y_d_hat = 0.0;
  y_integrator = 0.0;
  y_error_d1 = 0.0;
  y_error_dd = 0.0;

  d_hat_amp_y = 1.0;
  imp_F_prev_y = 0.0;
  force_err_f_y = 0.0;

  imp_y_error_d1 = 0.0;
  imp_y_error_dd = 0.0;
}

void CForceController::RollAxisParamInit() {
  roll_x_old = Eigen::VectorXd::Zero(STATE_DIM);
  roll_x_new = Eigen::VectorXd::Zero(STATE_DIM);
  roll_d_hat = Eigen::VectorXd::Zero(STATE_DIM);
  // roll_d_hat = 0.0;
  roll_integrator = 0.0;
  roll_error_d1 = 0.0;
  roll_error_dd = 0.0;

  d_hat_amp_roll = 0.01;
  imp_F_prev_roll = 0.0;
  force_err_f_roll = 0.0;

  imp_roll_error_d1 = 0.0;
  imp_roll_error_dd = 0.0;
}

void CForceController::PitchAxisParamInit() {
  pitch_x_old = Eigen::VectorXd::Zero(STATE_DIM);
  pitch_x_new = Eigen::VectorXd::Zero(STATE_DIM);
  pitch_d_hat = Eigen::VectorXd::Zero(STATE_DIM);
  // pitch_d_hat = 0.0;
  pitch_integrator = 0.0;
  pitch_error_d1 = 0.0;
  pitch_error_dd = 0.0;

  d_hat_amp_pitch = 0.01;
  imp_F_prev_pitch = 0.0;
  force_err_f_pitch = 0.0;

  imp_pitch_error_d1 = 0.0;
  imp_pitch_error_dd = 0.0;
}

void CForceController::YawAxisParamInit() {
  yaw_x_old = Eigen::VectorXd::Zero(STATE_DIM);
  yaw_x_new = Eigen::VectorXd::Zero(STATE_DIM);
  yaw_d_hat = Eigen::VectorXd::Zero(STATE_DIM);
  // yaw_d_hat = 0.0;
  yaw_integrator = 0.0;
  yaw_error_d1 = 0.0;
  yaw_error_dd = 0.0;

  d_hat_amp_yaw = 0.01;
  imp_F_prev_yaw = 0.0;
  force_err_f_yaw = 0.0;

  imp_yaw_error_d1 = 0.0;
  imp_yaw_error_dd = 0.0;

}

void CForceController::SetFeedbackData(const Feedback& data) {
  std::unique_lock<std::mutex> guard(mutex);
  feedback_data = data;
}

void CForceController::SetCurrentTarget(const Feedback& data) {
  std::unique_lock<std::mutex> guard(mutex);
  current_target = data;
}

void CForceController::SetMeasurementData(const Feedback& data) {
  std::unique_lock<std::mutex> guard(mutex);
  measurement_data = data;
}

void CForceController::SetReferenceData(const Feedback& data) {
  std::unique_lock<std::mutex> guard(mutex);
  reference_data = data;
}

void CForceController::SetControllerFlag(const std::array<double, 6>& data) {
  std::unique_lock<std::mutex> guard(mutex);
  controller_flag = data;
}

void CForceController::SetCompensationData(const Feedback& data) {
  std::unique_lock<std::mutex> guard(mutex);
  controller_result = data;
}

void CForceController::GetCompensationData(Feedback& data) {
  std::unique_lock<std::mutex> guard(mutex);
  data = controller_result;
}

void CForceController::SetOrientationDhatAmp(double amp) {
  d_hat_amp_roll = amp;
  d_hat_amp_pitch = amp;
  d_hat_amp_yaw = amp;
}

void CForceController::CalculateTCPError(const Pose cur_pose, const Pose ref_pose, Pose& x, Pose& x_dot) {
  Eigen::VectorXd reference_data_vector(6);
  reference_data_vector << ref_pose;
  Eigen::VectorXd feedback_data_vector(6);
  feedback_data_vector << cur_pose;

  Eigen::VectorXd x_(6);
  Eigen::VectorXd x_dot_(6);

  utils->EE6AxisScalars2(feedback_data_vector, reference_data_vector, Ts, x_, x_dot_);

  x.x = x_(0);
  x.y = x_(1);
  x.z = x_(2);
  x.roll = x_(3);
  x.pitch = x_(4);
  x.yaw = x_(5);

  x_dot.x = x_dot_(0);
  x_dot.y = x_dot_(1);
  x_dot.z = x_dot_(2);
  x_dot.roll = x_dot_(3);
  x_dot.pitch = x_dot_(4);
  x_dot.yaw = x_dot_(5);

  Feedback delta_tcp_ = {};
  delta_tcp_.pose = x;
  this->SetDeltaTCP(delta_tcp_);

  Feedback delta_tcp_dot_ = {};
  delta_tcp_dot_.pose = x_dot;
  this->SetDeltaTCPDot(delta_tcp_dot_);
}

void CForceController::SetDeltaTCP(const Feedback& data) { delta_tcp = data; }

void CForceController::SetDeltaTCPDot(const Feedback& data) { delta_tcp_dot = data; }

}  // namespace plaif::moforcecontroller