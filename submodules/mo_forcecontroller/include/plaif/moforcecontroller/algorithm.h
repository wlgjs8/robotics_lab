#ifndef CADMITTANCECONTRLLER_H_
#define CADMITTANCECONTRLLER_H_

#include <plaif/robot-data/robot-data.h>

#include <Eigen/Dense>
#include <complex>
#include <iostream>
#include <mutex>
#include <unsupported/Eigen/KroneckerProduct>
#include <unsupported/Eigen/MatrixFunctions>

#include "controller_utils.h"
#include "data_analysis.h"

#define MRAC 0
#define SCHEDULING 1

namespace plaif::moforcecontroller {
struct ImpedanceParam {
  double m = 1.0;
  double k = 220.0;
  double b = 11.0;
};

class CForceController {
 public:
  enum class Type { kUndefined = 0, kPoseErrorBasedForce = 3, kEnduringReferenceForce = 4, kSafetedExternalForce = 5 };
  enum Axis { kX = 0, kY = 1, kZ = 2, kRoll = 3, kPitch = 4, kYaw = 5 };

 public:
  CForceController(CForceController::Type controller_type);
  ~CForceController();

  void SetFeedbackData(const Feedback& data);
  void SetCurrentTarget(const Feedback& data);
  void SetMeasurementData(const Feedback& data);
  void SetReferenceData(const Feedback& data);
  void SetControllerFlag(const std::array<double, 6>& data);
  void GetCompensationData(Feedback& data);

  void Calculation();
  void EnduringReferenceForceControl();
  void EnduringReferenceForceControlImp();
  void SafetedExternalForceControl();
  void PoseErrorBasedCreateForce();

  void InitController();

  void SetCompensationData(const Feedback& data);
  void CalculateTCPError(const Pose cur_pose, const Pose ref_pose, Pose& x, Pose& x_dot);

  void SetOrientationDhatAmp(double amp);

 private:
  // double dt_ = static_cast<double>(__CONTROL_FREQUENCY__) / static_cast<double>(__SECOND__);
  ControllerUtils* utils;

  CForceController::Type compensation_type = CForceController::Type::kUndefined;

  std::mutex mutex;
  double Ts;

  std::array<ImpedanceParam, 6> imp_params_{};

  Feedback reference_data;
  Feedback feedback_data;
  Feedback current_target;
  Feedback measurement_data;
  std::array<double, 6> controller_flag;
  Feedback controller_result;

  Feedback delta_tcp;
  Feedback delta_tcp_dot;
  // Feedback adm_result;

  Wrench variable_k;      // = 800.0;
  Wrench variable_b;      // = 20.0;
  Wrench variable_tankE;  // = 100.0;

  // Note. Please verify the types of following variables. They may need to be of type double. by gunjae (2025.05.22)
  int STATE_DIM;
  int INPUT_DIM;
  int OUTPUT_DIM;

  bool Initialize(CForceController::Type controller_type);
  void CreateStateModel(CForceController::Type controller_type);

  void XAxisParamInit();
  Eigen::MatrixXd x_A_dob, x_B_dob, x_C_dob, x_D_dob;
  Eigen::MatrixXd x_A, x_B, x_C, x_E, x_D, x_L2;
  Eigen::VectorXd x_x_ref, x_x_old, x_x_new, x_L, x_d_hat;
  double x_ki, x_integrator, x_error_d1, x_error_dd;
  double imp_x_error_d1, imp_x_error_dd;
  double d_hat_amp_x;
  double force_err_f_x;
  Eigen::RowVectorXd x_K_lqr, x_k_int;
  double imp_F_prev_x;

  void YAxisParamInit();
  Eigen::MatrixXd y_A_dob, y_B_dob, y_C_dob, y_D_dob;
  Eigen::MatrixXd y_A, y_B, y_C, y_E, y_D, y_L2;
  Eigen::VectorXd y_x_ref, y_x_old, y_x_new, y_L, y_d_hat;
  double y_ki, y_integrator, y_error_d1, y_error_dd;
  double imp_y_error_d1, imp_y_error_dd, x_virtual_prev;
  double d_hat_amp_y;
  double force_err_f_y;
  Eigen::RowVectorXd y_K_lqr, y_k_int;
  double imp_F_prev_y;

  void ZAxisParamInit();
  Eigen::MatrixXd z_A_dob, z_B_dob, z_C_dob, z_D_dob;
  Eigen::MatrixXd z_A, z_B, z_C, z_E, z_D, z_L2_;
  Eigen::VectorXd z_x_ref, z_x_old, z_x_new, z_L, z_d_hat;
  double z_ki, z_integrator, z_error_d1, z_error_dd;
  double imp_z_error_d1, imp_z_error_dd;
  double d_hat_amp_z;
  double force_err_f_z;
  Eigen::RowVectorXd z_K_lqr, z_k_int;

  double output_y;
  //--------------------------------------------------------------------------------
  Eigen::MatrixXd z_A_dob2, z_B_dob2, z_C_dob2, z_D_dob2;
  Eigen::MatrixXd z_A2, z_B2, z_C2, z_E2, z_D2;
  Eigen::VectorXd z_x_old2, z_x_old_2, z_x_new2, z_x_new_2, z_L2, z_d_hat2;
  double z_ki2, z_integrator2, z_error_d12;
  Eigen::RowVectorXd z_K_lqr2, z_k_int2;
  double imp_F_prev_z;
  //--------------------------------------------------------------------------------

  void RollAxisParamInit();
  Eigen::MatrixXd roll_A_dob, roll_B_dob, roll_C_dob, roll_D_dob;
  Eigen::MatrixXd roll_A, roll_B, roll_C, roll_E, roll_D, roll_L2;
  Eigen::VectorXd roll_x_ref, roll_x_old, roll_x_new, roll_L, roll_d_hat;
  double roll_ki, roll_integrator, roll_error_d1, roll_error_dd;
  double imp_roll_error_d1, imp_roll_error_dd;
  double d_hat_amp_roll;
  double force_err_f_roll;
  Eigen::RowVectorXd roll_K_lqr, roll_k_int;
  double imp_F_prev_roll;

  void PitchAxisParamInit();
  Eigen::MatrixXd pitch_A_dob, pitch_B_dob, pitch_C_dob, pitch_D_dob;
  Eigen::MatrixXd pitch_A, pitch_B, pitch_C, pitch_E, pitch_D, pitch_L2;
  Eigen::VectorXd pitch_x_ref, pitch_x_old, pitch_x_new, pitch_L, pitch_d_hat;
  double pitch_ki, pitch_integrator, pitch_error_d1, pitch_error_dd;
  double imp_pitch_error_d1, imp_pitch_error_dd;
  Eigen::RowVectorXd pitch_K_lqr, pitch_k_int;
  double d_hat_amp_pitch;
  double force_err_f_pitch;
  double imp_F_prev_pitch;

  void YawAxisParamInit();
  Eigen::MatrixXd yaw_A_dob, yaw_B_dob, yaw_C_dob, yaw_D_dob;
  Eigen::MatrixXd yaw_A, yaw_B, yaw_C, yaw_E, yaw_D, yaw_L2;
  Eigen::VectorXd yaw_x_ref, yaw_x_old, yaw_x_new, yaw_L, yaw_d_hat;
  double yaw_ki, yaw_integrator, yaw_error_d1, yaw_error_dd;
  double imp_yaw_error_d1, imp_yaw_error_dd;
  Eigen::RowVectorXd yaw_K_lqr, yaw_k_int;
  double d_hat_amp_yaw;
  double force_err_f_yaw;
  double imp_F_prev_yaw;

  void CreateStateSpace(double m, double k, double b, Eigen::MatrixXd& A_d, Eigen::MatrixXd& B_d, Eigen::MatrixXd& C_d, Eigen::MatrixXd& E_d, Eigen::MatrixXd& D_d, Eigen::VectorXd& L_observer,
                        Eigen::RowVectorXd& K_lqr, Eigen::RowVectorXd& K_int);

  double UpdateStateSpace(const double ref_force, const double feedback_force, const double measured, Eigen::VectorXd& x_hat, Eigen::VectorXd& x_, const Eigen::MatrixXd A_d, const Eigen::MatrixXd B_d,
                          const Eigen::MatrixXd C_d, Eigen::MatrixXd E_d, const Eigen::MatrixXd D_d, const Eigen::VectorXd L_observer, const Eigen::RowVectorXd K_lqr, const Eigen::RowVectorXd& K_int,
                          Eigen::VectorXd& d_hat, double& integrator, double& error, double d_hat_amp, double& force_err_f, double forward = 0.0);  //


  double UpdateStateSpaceT(const double ref_force, const double feedback_force, const double measured, Eigen::VectorXd& x_hat, Eigen::VectorXd& x_, const Eigen::MatrixXd A_d, const Eigen::MatrixXd B_d,
                          const Eigen::MatrixXd C_d, Eigen::MatrixXd E_d, const Eigen::MatrixXd D_d, const Eigen::VectorXd L_observer, const Eigen::RowVectorXd K_lqr, const Eigen::RowVectorXd& K_int,
                          Eigen::VectorXd& d_hat, double& integrator, double& error, double d_hat_amp, double& force_err_f, double forward = 0.0);  //

  double UpdateStateSpaceO(const double ref_force, const double feedback_force, const double measured, Eigen::VectorXd& x_hat, Eigen::VectorXd& x_, const Eigen::MatrixXd A_d, const Eigen::MatrixXd B_d,
                          const Eigen::MatrixXd C_d, Eigen::MatrixXd E_d, const Eigen::MatrixXd D_d, const Eigen::VectorXd L_observer, const Eigen::RowVectorXd K_lqr, const Eigen::RowVectorXd& K_int,
                          Eigen::VectorXd& d_hat, double& integrator, double& error, double d_hat_amp, double& force_err_f, double forward = 0.0);  //

  void CreateStateSpaceInIntegrator(double m, double k, double b, Eigen::MatrixXd& A_d, Eigen::MatrixXd& B_d, Eigen::MatrixXd& C_d, Eigen::MatrixXd& E_d, Eigen::MatrixXd& D_d,
                                    Eigen::VectorXd& L_observer, Eigen::RowVectorXd& K_lqr, Eigen::RowVectorXd& K_int);
  double UpdateStateSpaceInIntegrator(const double ref_force, const double feedback_force, const double measured, Eigen::VectorXd& x_hat, Eigen::VectorXd& x_, const Eigen::MatrixXd A_d,
                                      const Eigen::MatrixXd B_d, const Eigen::MatrixXd C_d, Eigen::MatrixXd E_d, const Eigen::MatrixXd D_d, const Eigen::VectorXd L_observer,
                                      const Eigen::RowVectorXd K_lqr, const Eigen::RowVectorXd& K_int, Eigen::VectorXd& d_hat, double& integrator, double& error);

  // void SetImpedanceParam(double m, double k, double b);
  void SetImpedanceParam(Axis axis, double m, double k, double b);
  void SetImpedanceParams(const std::array<ImpedanceParam, 6>& params);

  double FilteredBasicMassSpringDamperT(const ImpedanceParam& param, const double ref_pose, const double feedback_pose, const double feedback_force, double& error, double& error_d, double& out_prev);
  double FilteredBasicMassSpringDamperO(const ImpedanceParam& param, const double transf_pose_err, const double feedback_force, double& error, double& error_d, double& out_prev);

  void SetDeltaTCP(const Feedback& data);
  void SetDeltaTCPDot(const Feedback& data);

  void AdaptiveControl(const int type, const double F_err, const double K_min, const double K_max, const double B_min, const double B_max, const double M, const double K, const double B,
                       const double x, const double x_dot, double& dK_des, double& dB_des);
  double EnergyTankStepKB(const double M, double& K, double& B, double x, double x_dot, double F_fb, double F_ref, double& tankE);
};
}  // namespace plaif::moforcecontroller
#endif
