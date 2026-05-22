#include "plaif/moforcecontroller/controller_utils.h"

namespace plaif::moforcecontroller {

const double TOL = 1e-6;

ControllerUtils::ControllerUtils() {}

ControllerUtils::~ControllerUtils() {}

Eigen::MatrixXd ControllerUtils::lqr_(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, const Eigen::MatrixXd& N) {
  const Eigen::MatrixXd P = solveContinuousARE_(A, B, Q, R);
  const Eigen::MatrixXd K = (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;

  return K;
}

Eigen::MatrixXd ControllerUtils::solveContinuousARE_(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R) {
  int n = A.rows();
  int m = B.cols();

  Eigen::MatrixXd P = Q;
  Eigen::MatrixXd P_next(n, n);

  double tol = 1e-9;
  int max_iter = 1000;
  int iter = 0;

  while (iter < max_iter) {
    P_next = A.transpose() * P * A - A.transpose() * P * B * (R + B.transpose() * P * B).inverse() * B.transpose() * P * A + Q;
    if ((P_next - P).norm() < tol) {
      break;
    }
    P = P_next;
    iter++;
  }

  return P_next;
}

void ControllerUtils::c2d_multiInput(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& E, double Ts, Eigen::MatrixXd& A_d, Eigen::MatrixXd& B_d, Eigen::MatrixXd& E_d) {
  int n = A.rows();
  int m1 = B.cols();
  int m2 = E.cols();

  Eigen::MatrixXd BE(n, m1 + m2);
  BE << B, E;

  Eigen::MatrixXd M(n + (m1 + m2), n + (m1 + m2));
  M.setZero();
  M.block(0, 0, n, n) = A;
  M.block(0, n, n, m1 + m2) = BE;
  M.block(n, n, m1 + m2, m1 + m2).setIdentity();

  // exponent
  Eigen::MatrixXd expM = (M * Ts).exp();

  // A_d
  A_d = expM.block(0, 0, n, n);
  Eigen::MatrixXd BE_d = expM.block(0, n, n, m1 + m2);

  // split
  B_d = BE_d.block(0, 0, n, m1);
  E_d = BE_d.block(0, m1, n, m2);
}

void ControllerUtils::c2d(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, double Ts, Eigen::MatrixXd& Ad, Eigen::MatrixXd& Bd) {
  int n = A.rows();
  int m = B.cols();
  Eigen::MatrixXd M(n + m, n + m);
  M << A, B, Eigen::MatrixXd::Zero(m, n), Eigen::MatrixXd::Identity(m, m);
  Eigen::MatrixXd expM = (M * Ts).exp();
  Ad = expM.block(0, 0, n, n);
  Bd = expM.block(0, n, n, m);
}

Eigen::MatrixXd ControllerUtils::ctrb(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B) {
  int n = A.rows();
  Eigen::MatrixXd C(B);
  Eigen::MatrixXd AB = B;
  for (int i = 1; i < n; ++i) {
    AB = A * AB;
    Eigen::MatrixXd C_new(n, C.cols() + B.cols());
    C_new << C, AB;
    C = C_new;
  }
  return C;
}

Eigen::MatrixXd ControllerUtils::solveContinuousARE(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R) {
  int n = A.rows();
  Eigen::MatrixXd Z = Eigen::MatrixXd::Zero(n, n);
  Eigen::MatrixXd H(2 * n, 2 * n);
  H << A, -B * R.inverse() * B.transpose(), -Q, -A.transpose();

  Eigen::EigenSolver<Eigen::MatrixXd> es(H);
  Eigen::MatrixXd eigVecs = es.eigenvectors().real();
  Eigen::MatrixXd eigVals = es.eigenvalues().real().asDiagonal();

  Eigen::MatrixXd V = eigVecs.block(0, 0, n, n);
  Eigen::MatrixXd U = eigVecs.block(n, 0, n, n);

  Eigen::MatrixXd P = U * V.inverse();

  return P;
}

Eigen::MatrixXd ControllerUtils::lqr(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R) {
  Eigen::MatrixXd P = solveContinuousARE(A, B, Q, R);

  // Compute the LQR gain
  Eigen::MatrixXd K = R.inverse() * B.transpose() * P;
  return K;
}

Eigen::MatrixXd ControllerUtils::solveRiccati(Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::MatrixXd& Q, Eigen::MatrixXd& R) {
  double tolerance = 0.00001;
  int max_iter = 100000;
  double dt = 0.002;

  int dim_x = A.rows();
  int dim_u = B.cols();

  Eigen::MatrixXd P = Q;
  Eigen::MatrixXd P_next;
  double diff = std::numeric_limits<double>::max();
  int ii = 0;

  while (diff > tolerance && ii < max_iter) {
    ii++;
    P_next = P + (A.transpose() * P + P * A + Q - P * B * R.inverse() * B.transpose() * P) * dt;
    diff = fabs((P_next - P).maxCoeff());
    P = P_next;
  }

  return R.inverse() * B.transpose() * P;
}

void ControllerUtils::ComputeInitialGuess(int n, int m, Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::MatrixXd& Q, Eigen::MatrixXd& R, Eigen::MatrixXd& S, Eigen::MatrixXd& In,
                                          Eigen::MatrixXd& initialGuessMatrix) {
  Eigen::MatrixXd Anew;
  Eigen::MatrixXd Bnew;

  Anew = A - B * (R.inverse()) * (S.transpose());
  Bnew = B * (R.inverse()) * (B.transpose());

  Eigen::EigenSolver<Eigen::MatrixXd> eigenValueSolver(Anew);
  std::complex<double> lambda;
  double bParam = 0;
  std::vector<double> realPartEigenValues;
  for (int i = 0; i < n; i++) {
    lambda = eigenValueSolver.eigenvalues()[i];
    realPartEigenValues.push_back(lambda.real());
  }
  bParam = (*min_element(realPartEigenValues.begin(), realPartEigenValues.end()));
  bParam = (-1) * bParam;
  bParam = std::max(bParam, 0.0) + 0.02;
  Eigen::MatrixXd Abar;
  Abar = (bParam * In + Anew).transpose();
  Eigen::MatrixXd RHS;
  RHS = 2 * Bnew * (Bnew.transpose());
  Eigen::MatrixXd solutionLyapunov;
  solutionLyapunov.resize(n, n);
  solutionLyapunov.setZero();
  double residualLyap = 10e10;
  SolveLyapunovEquation(n, m, Abar, RHS, In, solutionLyapunov, residualLyap);
  initialGuessMatrix = (Bnew.transpose()) * (solutionLyapunov.inverse());
  initialGuessMatrix = 0.5 * (initialGuessMatrix + initialGuessMatrix.transpose());
}

void ControllerUtils::SolveLyapunovEquation(int n, int m, const Eigen::MatrixXd& Am, const Eigen::MatrixXd& Rm, Eigen::MatrixXd& In, Eigen::MatrixXd& SolutionMatrix, double& residual) {
  Eigen::MatrixXd LHSMatrix = Eigen::kroneckerProduct(In, Am.transpose()) + Eigen::kroneckerProduct(Am.transpose(), In);

  Eigen::VectorXd RHSVector = Eigen::Map<const Eigen::VectorXd>(Rm.data(), Rm.size());

  Eigen::VectorXd solutionLyapunovVector = LHSMatrix.completeOrthogonalDecomposition().solve(RHSVector);

  SolutionMatrix = Eigen::Map<const Eigen::MatrixXd>(solutionLyapunovVector.data(), n, n);

  SolutionMatrix = 0.5 * (SolutionMatrix + SolutionMatrix.transpose());

  Eigen::MatrixXd residualMatrix = (Am.transpose()) * SolutionMatrix + SolutionMatrix * Am - Rm;

  residual = residualMatrix.squaredNorm();
}

Eigen::MatrixXd ControllerUtils::ComputeLQRSolution(Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::MatrixXd& Q, Eigen::MatrixXd& R) {
  int maxNumberIterations = 5000;
  double tolerance = 1e-8;
  int n = A.rows();
  int m = B.cols();

  Eigen::MatrixXd solutionRiccati;
  solutionRiccati.resize(n, n);
  solutionRiccati.setZero();

  Eigen::MatrixXd K;
  K.resize(m, n);
  K.setZero();

  Eigen::MatrixXd Acl;
  Acl.resize(n, n);
  Acl.setZero();

  Eigen::MatrixXd S;
  S.resize(n, m);
  S.setZero();

  Eigen::MatrixXd In;
  In = Eigen::MatrixXd::Identity(n, n);
  Eigen::MatrixXd initialGuess;
  initialGuess.resize(n, n);
  initialGuess.setZero();
  ComputeInitialGuess(n, m, A, B, Q, R, S, In, initialGuess);
  Eigen::MatrixXd initialK;
  Eigen::MatrixXd initialAcl;
  initialK = (R.inverse()) * ((B.transpose()) * initialGuess + S.transpose());
  initialAcl = A - B * initialK;
  Eigen::EigenSolver<Eigen::MatrixXd> eigenValueSolver(initialAcl);
  solutionRiccati = initialGuess;
  Eigen::MatrixXd solutionLyapunov;
  solutionLyapunov.resize(n, n);
  solutionLyapunov.setZero();
  Eigen::MatrixXd RHS;
  Eigen::MatrixXd tmpMatrix;
  Eigen::MatrixXd updateMatrix;
  double residualLyap = 10e10;
  double errorConvergence = 10e10;
  int currentIteration = 0;
  while (currentIteration <= maxNumberIterations && errorConvergence >= tolerance) {
    tmpMatrix = (B.transpose()) * solutionRiccati + S.transpose();
    K = (R.inverse()) * tmpMatrix;
    Acl = A - B * K;
    RHS = Q + (K.transpose()) * R * K - S * K - (K.transpose()) * (S.transpose());
    RHS = -RHS;
    SolveLyapunovEquation(n, m, Acl, RHS, In, solutionLyapunov, residualLyap);
    updateMatrix = solutionLyapunov - solutionRiccati;
    errorConvergence = (updateMatrix.cwiseAbs().colwise().sum().maxCoeff()) / (solutionRiccati.cwiseAbs().colwise().sum().maxCoeff());
    solutionRiccati = solutionLyapunov;
    currentIteration = currentIteration + 1;
  }
  if (currentIteration > maxNumberIterations) {
  }

  Eigen::EigenSolver<Eigen::MatrixXd> eigenValueSolver2(Acl);

  return K;
}

Eigen::MatrixXd ControllerUtils::ComputeDARESolution(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R) {
  const int n = A.rows();
  const int m = B.cols();

  Eigen::MatrixXd X = Q;
  int maxIter = 20000;  // 최대 반복 횟수 증가
  double tol = 1e-6;    // 허용 오차 완화
  int iter = 0;

  for (iter = 0; iter < maxIter; iter++) {
    Eigen::MatrixXd temp = R + B.transpose() * X * B;

    Eigen::MatrixXd K = temp.inverse() * (B.transpose() * X * A);

    Eigen::MatrixXd X_next = Q + A.transpose() * X * A - A.transpose() * X * B * K;

    double err = (X_next - X).norm();

    X = X_next;
    if (err < tol) {
      break;
    }
  }

  if (iter >= maxIter) {
    // std::cout << "DARE iteration did not converge! Current error: " << (X - Q).norm() << std::endl;
  }

  Eigen::MatrixXd temp = R + B.transpose() * X * B;
  Eigen::MatrixXd K_final = temp.inverse() * (B.transpose() * X * A);

  return K_final;
}

Eigen::MatrixXd ControllerUtils::ComputeDARESolutionSchur(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R) {
  const int n = A.rows();  // 여기서 n은 3이어야 함
  Eigen::MatrixXd Rinv = R.inverse();

  Eigen::MatrixXd H(2 * n, 2 * n);
  H.topLeftCorner(n, n) = A;
  H.topRightCorner(n, n) = -B * Rinv * B.transpose();
  H.bottomLeftCorner(n, n) = -Q;
  H.bottomRightCorner(n, n) = A.transpose();

  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(2 * n, 2 * n);
  J.topLeftCorner(n, n) = Eigen::MatrixXd::Identity(n, n);
  J.bottomRightCorner(n, n) = -Eigen::MatrixXd::Identity(n, n);

  Eigen::RealQZ<Eigen::MatrixXd> qz(H, J);
  Eigen::MatrixXd S = qz.matrixS();
  Eigen::MatrixXd T = qz.matrixT();
  Eigen::MatrixXd Z = qz.matrixZ();

  struct EigenInfo {
    int index;
    double margin;  // 1 - |lambda|
  };
  std::vector<EigenInfo> stableList;

  for (int i = 0; i < 2 * n; i++) {
    double alpha = S(i, i);
    double beta = T(i, i);
    double lambda = (std::abs(beta) > 1e-12) ? (alpha / beta) : 1e10;
    if (std::abs(lambda) < 1.0) {
      double margin = 1.0 - std::abs(lambda);
      stableList.push_back({i, margin});
    }
  }

  if (stableList.size() < static_cast<size_t>(n)) {
    return Eigen::MatrixXd::Zero(n, n);
  }

  std::sort(stableList.begin(), stableList.end(), [](const EigenInfo& a, const EigenInfo& b) { return a.margin > b.margin; });

  std::vector<int> selectedIndices;
  for (int i = 0; i < n; i++) {
    selectedIndices.push_back(stableList[i].index);
  }

  Eigen::MatrixXcd Zc = Z.cast<std::complex<double>>();
  Eigen::MatrixXcd U1(n, n), U2(n, n);
  for (int i = 0; i < n; i++) {
    U1.col(i) = Zc.block(0, selectedIndices[i], n, 1);
    U2.col(i) = Zc.block(n, selectedIndices[i], n, 1);
  }

  Eigen::MatrixXcd Xc = U2 * U1.inverse();
  Eigen::MatrixXd X = Xc.real();
  X = 0.5 * (X + X.transpose());

  Eigen::MatrixXd temp = R + B.transpose() * X * B;
  Eigen::MatrixXd K_final = temp.inverse() * (B.transpose() * X * A);

  return X;
}

Eigen::MatrixXd ControllerUtils::ComputeDiscreteLQRSolution(Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::MatrixXd& Q, Eigen::MatrixXd& R) {
  int maxIter = 5000;
  double tol = 1e-9;
  int n = A.rows();
  Eigen::MatrixXd P = Q;
  Eigen::MatrixXd P_next = Q;
  double error = 1e10;
  int iter = 0;
  while (iter < maxIter && error > tol) {
    Eigen::MatrixXd temp = R + B.transpose() * P * B;
    Eigen::MatrixXd K = temp.inverse() * (B.transpose() * P * A);
    P_next = Q + A.transpose() * P * A - A.transpose() * P * B * K;
    error = (P_next - P).norm();
    P = P_next;
    iter++;
  }
  // 최종 LQR 게인
  Eigen::MatrixXd tempFinal = R + B.transpose() * P * B;
  Eigen::MatrixXd K_final = tempFinal.inverse() * (B.transpose() * P * A);
  return K_final;
}

bool ControllerUtils::isStabilizable(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B) {
  int n = A.rows();
  Eigen::EigenSolver<Eigen::MatrixXd> es(A);
  Eigen::VectorXcd eigVals = es.eigenvalues();

  for (int i = 0; i < n; ++i) {
    if (std::abs(eigVals(i)) >= 1.0) {
      Eigen::MatrixXcd M(n, n + B.cols());
      M.block(0, 0, n, n) = eigVals(i) * Eigen::MatrixXcd::Identity(n, n) - A.cast<std::complex<double>>();
      M.block(0, n, n, B.cols()) = B.cast<std::complex<double>>();

      Eigen::BDCSVD<Eigen::MatrixXcd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
      int rank = (svd.singularValues().array() > TOL).count();

      if (rank < n) {
        // std::cout << "Eigenvalue " << eigVals(i) << " is not controllable (rank = " << rank << ")." << std::endl;
        return false;
      }
    }
  }
  return true;
}

// 검출 가능성 검사 함수: (A, C)
bool ControllerUtils::isDetectable(const Eigen::MatrixXd& A, const Eigen::MatrixXd& C) {
  int n = A.rows();
  Eigen::EigenSolver<Eigen::MatrixXd> es(A);
  Eigen::VectorXcd eigVals = es.eigenvalues();

  for (int i = 0; i < n; ++i) {
    if (std::abs(eigVals(i)) >= 1.0) {
      // (λI - A; C)를 구성 (세로로 연결)
      Eigen::MatrixXcd M(n + C.rows(), n);
      M.block(0, 0, n, n) = eigVals(i) * Eigen::MatrixXcd::Identity(n, n) - A.cast<std::complex<double>>();
      M.block(n, 0, C.rows(), n) = C.cast<std::complex<double>>();

      Eigen::BDCSVD<Eigen::MatrixXcd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
      int rank = (svd.singularValues().array() > TOL).count();

      if (rank < n) {
        // std::cout << "Eigenvalue " << eigVals(i) << " is not observable (rank = " << rank << ")." << std::endl;
        return false;
      }
    }
  }
  return true;
}

// Q^(1/2) 계산 함수 (Q가 양의정의 행렬이라고 가정)
Eigen::MatrixXd ControllerUtils::computeSqrt(const Eigen::MatrixXd& Q) {
  Eigen::LLT<Eigen::MatrixXd> llt(Q);
  if (llt.info() == Eigen::Success)
    return llt.matrixL();  // Lower triangular matrix such that L*L^T = Q
  else {
    // std::cerr << "Cholesky decomposition failed. Q may not be positive definite." << std::endl;
    return Eigen::MatrixXd::Zero(Q.rows(), Q.cols());
  }
}

double ControllerUtils::angleDiff(double angle1, double angle2) {
  double diff = angle2 - angle1;
  // -pi ~ pi 범위로 보정 (라디안 단위)
  while (diff > M_PI) diff -= 2.0 * M_PI;
  while (diff < -M_PI) diff += 2.0 * M_PI;
  return diff;
}

Eigen::Affine3d ControllerUtils::BuildTransformXYZRPY(const Eigen::VectorXd& pose, const std::string& type) {
  Eigen::Matrix3d R = RPYToRotationMatrix(pose(3), pose(4), pose(5), type);
  Eigen::Vector3d t(pose(0), pose(1), pose(2));
  Eigen::Affine3d T = Eigen::Affine3d::Identity();
  T.linear() = R;
  T.translation() = t;
  return T;
}

Eigen::Matrix3d ControllerUtils::RPYToRotationMatrix(double roll, double pitch, double yaw, const std::string& type) {
  double cr = std::cos(roll), sr = std::sin(roll);
  double cp = std::cos(pitch), sp = std::sin(pitch);
  double cy = std::cos(yaw), sy = std::sin(yaw);

  Eigen::Matrix3d Rx;
  Rx << 1, 0, 0, 0, cr, -sr, 0, sr, cr;

  Eigen::Matrix3d Ry;
  Ry << cp, 0, sp, 0, 1, 0, -sp, 0, cp;

  Eigen::Matrix3d Rz;
  Rz << cy, -sy, 0, sy, cy, 0, 0, 0, 1;

  if (type == "xyz") {
    return Rx * Ry * Rz;
  }

  return Rz * Ry * Rx;
}

Eigen::Matrix3d ControllerUtils::skew(const Eigen::Vector3d& v)
{
    Eigen::Matrix3d S;
    S <<     0, -v.z(),  v.y(),
          v.z(),     0, -v.x(),
         -v.y(),  v.x(),     0;
    return S;
}

Eigen::Matrix<double,6,6> ControllerUtils::forceTransform_World_to_EEF(const Eigen::Vector3d& p_we, const Eigen::Matrix3d& R_we)
{
    //World to EEF include torque
    const Eigen::Matrix3d R_ew = R_we.transpose();
    Eigen::Matrix3d px;
    px <<   0.0,     -p_we.z(),  p_we.y(),
            p_we.z(),  0.0,     -p_we.x(),
           -p_we.y(),  p_we.x(),  0.0;

    Eigen::Matrix<double,6,6> Xf;
    Xf.setZero();
    Xf.block<3,3>(0,0) = R_ew;
    Xf.block<3,3>(3,0) = -R_ew * px;
    Xf.block<3,3>(3,3) =  R_ew;
    return Xf;
}

Eigen::Matrix<double,6,1> ControllerUtils::wrench_World_to_EEF(const Eigen::Matrix<double,6,1>& W_w, const Eigen::Vector3d& p_we, const Eigen::Matrix3d& R_we)
{
  auto Xf = forceTransform_World_to_EEF(p_we, R_we);
  return Xf * W_w;
}

Eigen::Matrix<double,6,1> ControllerUtils::BaseSelectionMatrix(const Eigen::VectorXd& selection_matrix, const Eigen::VectorXd& current_pose)
{
  // 1) 현재 EE의 base 기준 위치/자세 추출
  Eigen::Vector3d p_be(current_pose(0), current_pose(1), current_pose(2));
  double roll  = current_pose(3);
  double pitch = current_pose(4);
  double yaw   = current_pose(5);

  // 2) R_be 구성 (yaw→pitch→roll 순서 그대로 유지)
  Eigen::Quaterniond q_be =
      Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX());
  Eigen::Matrix3d R_be = q_be.toRotationMatrix();

  // 3) Twist adjoint Ad_be
  Eigen::Matrix3d px = skew(p_be); // 질문에 주신 동일 'skew' 사용
  Eigen::Matrix<double,6,6> Ad_be = Eigen::Matrix<double,6,6>::Zero();
  Ad_be.topLeftCorner<3,3>()      = R_be;
  Ad_be.bottomRightCorner<3,3>()  = R_be;
  Ad_be.bottomLeftCorner<3,3>()   = px * R_be;

  // 4) EE 프레임 선택행렬 S_ee (6x6 대각)
  Eigen::Matrix<double,6,6> S_ee = Eigen::Matrix<double,6,6>::Zero();
  for (int i = 0; i < 6; ++i) S_ee(i,i) = selection_matrix(i);

  // 5) Wrench용(base측) 선택행렬: P_b^(F) = Ad^T * S_ee * Ad^{-T}
  Eigen::Matrix<double,6,6> P_b_F =
      Ad_be.transpose() * S_ee * Ad_be.inverse().transpose();

  // 6) 축별 가중치(0~1): e_i^T P_b^(F) e_i
  Eigen::Matrix<double,6,1> imp_flag = Eigen::Matrix<double,6,1>::Zero();
  for (int i = 0; i < 6; ++i) {
    Eigen::Matrix<double,6,1> e = Eigen::Matrix<double,6,1>::Zero();
    e(i) = 1.0;
    double val = (e.transpose() * P_b_F * e)(0,0);
    // 수치 잡음에 의한 음/1+epsilon 보정
    imp_flag(i) = std::min(1.0, std::max(0.0, val));
  }
  return imp_flag;
}

void ControllerUtils::EE6AxisScalars2(
  // const Pose cur_pose,            // 현재 EEF 포즈 (world)
  // const Pose ref_pose,            // 기준(명령) EEF 포즈 (world)
  // const double Ts,                // 샘플링 주기 [s]
  // Pose& x,                        // [out] EEF 기준 변형
  // Pose& xdot                      // [out] EEF 기준 변형 속도
  const Eigen::VectorXd cur_pose,            // 현재 EEF 포즈 (world)
  const Eigen::VectorXd ref_pose,            // 기준(명령) EEF 포즈 (world)
  const double Ts,                // 샘플링 주기 [s]
  Eigen::VectorXd& x,                        // [out] EEF 기준 변형
  Eigen::VectorXd& xdot                      // [out] EEF 기준 변형 속도
){
  // ---------- (0) 내부 상태 (직전 포즈/필터) ----------

  // ---------- (1) 회전행렬 ----------
  Eigen::Matrix3d R_c = this->RPYToRotationMatrix(cur_pose[3], cur_pose[4], cur_pose[5]);
  Eigen::Matrix3d R_t = this->RPYToRotationMatrix(ref_pose[3], ref_pose[4], ref_pose[5]);

  // ---------- (2) 변형 x (EEF 기준) ----------
  Eigen::Matrix3d R_e = R_t.transpose() * R_c;
  Eigen::Vector3d phi = logSO3(R_e);                      // 회전 변형
  Eigen::Vector3d e_p_E = R_t.transpose() *
                          (Eigen::Vector3d(cur_pose[0],cur_pose[1],cur_pose[2])
                         - Eigen::Vector3d(ref_pose[0],ref_pose[1],ref_pose[2]));
  Eigen::Vector3d dp_E  = leftJacobianInvSO3(phi) * e_p_E; // 병진 변형(작은각 안정)

  x << dp_E.x(), dp_E.y(), dp_E.z(), phi.x(), phi.y(), phi.z();

  // ---------- (3) 속도 xdot (포즈 차분으로 추정) ----------
  Eigen::Vector3d v_cE = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_tE = Eigen::Vector3d::Zero();
  Eigen::Vector3d w_cE = Eigen::Vector3d::Zero();
  Eigen::Vector3d w_tE = Eigen::Vector3d::Zero();

  if(!ee6_cache_.init){
      ee6_cache_.init = true;
      ee6_cache_.cur_prev = cur_pose;
      ee6_cache_.ref_prev = ref_pose;
      xdot = Eigen::VectorXd::Zero(6);
      return;
  }

  // --- 현재/기준 각각 바디 트위스트 산출 (k-1 -> k) ---
  auto bodyTwistFromDiff = [&](const Eigen::VectorXd& P_prev, const Eigen::VectorXd& P_now,
                               Eigen::Vector3d& v_b, Eigen::Vector3d& w_b)
  {
      Eigen::Matrix3d R_prev = this->RPYToRotationMatrix(P_prev[3],P_prev[4],P_prev[5]);
      Eigen::Matrix3d R_now  = this->RPYToRotationMatrix(P_now[3] ,P_now[4] ,P_now[5] );

      Eigen::Vector3d p_prev(P_prev[0],P_prev[1],P_prev[2]);
      Eigen::Vector3d p_now (P_now[0] ,P_now[1] ,P_now[2]);

      // 회전/병진 증분(이전 바디 프레임)
      Eigen::Matrix3d R_inc = R_prev.transpose() * R_now;
      Eigen::Vector3d phi_inc = logSO3(R_inc);
      Eigen::Vector3d ep_bprev = R_prev.transpose() * (p_now - p_prev);

      // 바디 각속/선속 (J_l^{-1}로 보정)
      w_b = phi_inc / Ts;
      v_b = leftJacobianInvSO3(phi_inc) * ep_bprev / Ts;
  };

  // cur: 바디 트위스트 → EEF 기준으로
  {
      Eigen::Vector3d v_cb, w_cb;
      bodyTwistFromDiff(ee6_cache_.cur_prev, cur_pose, v_cb, w_cb);
      Eigen::Matrix3d R_tTRc = R_t.transpose() * R_c;
      v_cE = R_tTRc * v_cb;
      w_cE = R_tTRc * w_cb;
  }
  // ref: 바디 트위스트 → EEF 기준(= ref 바디와 동일)
  {
      Eigen::Vector3d v_tb, w_tb;
      bodyTwistFromDiff(ee6_cache_.ref_prev, ref_pose, v_tb, w_tb);
      v_tE = v_tb;
      w_tE = w_tb;
  }

  // ---------- (4) 저역통과 필터(선택) ----------
  const double fc_lin = 30.0;  // 선속 LPF 차단[Hz] (원하는 값으로 튜닝)
  const double fc_ang = 40.0;  // 각속 LPF 차단[Hz]
  const double a_lin = (2.0*M_PI*fc_lin*Ts) / (1.0 + 2.0*M_PI*fc_lin*Ts);
  const double a_ang = (2.0*M_PI*fc_ang*Ts) / (1.0 + 2.0*M_PI*fc_ang*Ts);

  ee6_cache_.v_cE_f = (1.0 - a_lin)*ee6_cache_.v_cE_f + a_lin*v_cE;
  ee6_cache_.v_tE_f = (1.0 - a_lin)*ee6_cache_.v_tE_f + a_lin*v_tE;
  ee6_cache_.w_cE_f = (1.0 - a_ang)*ee6_cache_.w_cE_f + a_ang*w_cE;
  ee6_cache_.w_tE_f = (1.0 - a_ang)*ee6_cache_.w_tE_f + a_ang*w_tE;

  // ---------- (5) 상대 속도 ----------
  Eigen::Vector3d vE   = ee6_cache_.v_cE_f - ee6_cache_.v_tE_f;
  Eigen::Vector3d dome = ee6_cache_.w_cE_f - ee6_cache_.w_tE_f;
  Eigen::Vector3d dphi = leftJacobianInvSO3(phi) * dome;

  xdot << vE.x(), vE.y(), vE.z(), dphi.x(), dphi.y(), dphi.z();

  // ---------- (6) 캐시 갱신 ----------
  ee6_cache_.cur_prev = cur_pose;
  ee6_cache_.ref_prev = ref_pose;
}

Eigen::Matrix3d ControllerUtils::leftJacobianInvSO3(const Eigen::Vector3d& phi){
  double th = phi.norm();
  Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  if(th < 1e-6){
      // J^{-1} ~ I + 1/2 [phi]x + 1/12 [phi]x^2
      Eigen::Matrix3d S = skew(phi);
      return I + 0.5*S + (1.0/12.0)*(S*S);
  } else {
      Eigen::Matrix3d S = skew(phi);
      double half = 0.5*th;
      double cot_half = std::cos(half)/std::sin(half); // = 2/th * (1 - (th/2)cot(th/2))^-1 형태 유도 가능
      // 표준 형태: J^{-1} = I - 0.5 [phi]x + (1/th^2) * (1 - (th/2)cot(th/2)) [phi]x^2
      double a = 0.5;
      double b = (1.0/(th*th))*(1.0 - half*cot_half);
      return I - a*S + b*(S*S);
  }
}

Eigen::Vector3d ControllerUtils::logSO3(const Eigen::Matrix3d& R){
  double cos_th = (R.trace() - 1.0) * 0.5;
  cos_th = std::min(1.0, std::max(-1.0, cos_th));
  double th = std::acos(cos_th);

  if(th < 1e-8){
      // 매우 작은 각: log(R) ~ vee(R - R^T)/2
      Eigen::Matrix3d A = (R - R.transpose()) * 0.5;
      return Eigen::Vector3d(A(2,1), A(0,2), A(1,0));
  } else {
      Eigen::Matrix3d lnR = (th/(2.0*std::sin(th))) * (R - R.transpose());
      return Eigen::Vector3d(lnR(2,1), lnR(0,2), lnR(1,0));
  }
}



}  // namespace plaif::moforcecontroller