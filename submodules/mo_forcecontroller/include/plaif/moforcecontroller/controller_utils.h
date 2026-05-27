#ifndef CONTROLLERUTILS_H_
#define CONTROLLERUTILS_H_

#include <Eigen/Dense>
#include <complex>
#include <unsupported/Eigen/KroneckerProduct>
#include <unsupported/Eigen/MatrixFunctions>

namespace plaif::moforcecontroller {
class ControllerUtils {
 public:
  ControllerUtils();
  ~ControllerUtils();

  Eigen::MatrixXd ComputeLQRSolution(Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::MatrixXd& Q, Eigen::MatrixXd& R);
  Eigen::MatrixXd ComputeDARESolution(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R);
  Eigen::MatrixXd ComputeDARESolutionSchur(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R);
  Eigen::MatrixXd ComputeDiscreteLQRSolution(Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::MatrixXd& Q, Eigen::MatrixXd& R);
  void c2d(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, double Ts, Eigen::MatrixXd& Ad, Eigen::MatrixXd& Bd);
  void c2d_multiInput(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& E, double Ts, Eigen::MatrixXd& A_d, Eigen::MatrixXd& B_d, Eigen::MatrixXd& E_d);
  Eigen::MatrixXd ctrb(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B);

  bool isStabilizable(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B);
  bool isDetectable(const Eigen::MatrixXd& A, const Eigen::MatrixXd& C);
  Eigen::MatrixXd computeSqrt(const Eigen::MatrixXd& Q);

  double angleDiff(double angle1, double angle2);

  Eigen::Matrix3d skew(const Eigen::Vector3d& v);
  Eigen::Matrix<double, 6, 6> forceTransform_World_to_EEF(const Eigen::Vector3d& p_we, const Eigen::Matrix3d& R_we);
  Eigen::Matrix<double, 6, 1> wrench_World_to_EEF(const Eigen::Matrix<double, 6, 1>& W_w, const Eigen::Vector3d& p_we, const Eigen::Matrix3d& R_we);

  Eigen::Matrix<double, 6, 1> BaseSelectionMatrix(const Eigen::VectorXd& selection_matrix, const Eigen::VectorXd& current_pose);

  void EE6AxisScalars2(const Eigen::VectorXd cur_pose, const Eigen::VectorXd ref_pose, const double Ts, Eigen::VectorXd& x, Eigen::VectorXd& xdot);
  void SolveLyapunovEquation(int n, int m, const Eigen::MatrixXd& Am, const Eigen::MatrixXd& Rm, Eigen::MatrixXd& In, Eigen::MatrixXd& SolutionMatrix, double& residual);

  void ResetEE6Cache() { ee6_cache_ = EE6Cache{}; }

 private:
  struct EE6Cache {
    bool init = false;
    Eigen::VectorXd cur_prev = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd ref_prev = Eigen::VectorXd::Zero(6);
    Eigen::Vector3d v_cE_f = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_tE_f = Eigen::Vector3d::Zero();
    Eigen::Vector3d w_cE_f = Eigen::Vector3d::Zero();
    Eigen::Vector3d w_tE_f = Eigen::Vector3d::Zero();
  };

  void ComputeInitialGuess(int n, int m, Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::MatrixXd& Q, Eigen::MatrixXd& R, Eigen::MatrixXd& S, Eigen::MatrixXd& In, Eigen::MatrixXd& initialGuessMatrix);

  //------------------------------------------------------------------------------------------------------------------------
  Eigen::MatrixXd lqr(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R);
  Eigen::MatrixXd solveContinuousARE(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R);
  Eigen::MatrixXd solveRiccati(Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::MatrixXd& Q, Eigen::MatrixXd& R);
  Eigen::MatrixXd solveContinuousARE_(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R);
  Eigen::MatrixXd lqr_(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, const Eigen::MatrixXd& N = Eigen::MatrixXd());

  Eigen::Matrix3d RPYToRotationMatrix(double roll, double pitch, double yaw, const std::string& type = "");
  Eigen::Affine3d BuildTransformXYZRPY(const Eigen::VectorXd& pose, const std::string& type = "");

  Eigen::Matrix3d leftJacobianInvSO3(const Eigen::Vector3d& phi);
  Eigen::Vector3d logSO3(const Eigen::Matrix3d& R);

  EE6Cache ee6_cache_;
};
}  // namespace plaif::moforcecontroller

#endif
