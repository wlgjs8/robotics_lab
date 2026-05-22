#ifndef DATACALCONTROLLER_H_
#define DATACALCONTROLLER_H_

#include <plaif/robot-data/robot-data.h>

#include <Eigen/Dense>

#include "algorithm.h"
#include "data_analysis.h"

namespace plaif::moforcecontroller {
class ForceController {
 private:  // attributes
  CForceController* n_pforcecontroller_adm = nullptr;
  CForceController* n_pforcecontroller_imp = nullptr;
  CForceController* n_pforcecontroller_safe = nullptr;
  CForceController* n_pforcecontroller_imp_adm = nullptr;
  Transformation* m_ptransformation = nullptr;

  Pose prev_adm_pose_ = {};

  Feedback cont_result;
  Feedback feedback_data;
  Feedback reference_data;
  Feedback controller_flag;
  Feedback measurement_data;
  Feedback prev_feedback;
  Feedback current_target;

  Wrench imp_output_wrench;
  Pose adm_output_pose;
  Pose safety_output_pose;

 public:  // methods
  ForceController(const double* robot_frame);
  ~ForceController();

  void Initialize(const double* robot_frame);
  void Calculation();
  void SetFeedbackData(const Feedback& data);
  void SetMeasurementData(const Feedback& data);
  void SetReferenceData(const Feedback& data);
  void SetControllerFlag(const Feedback& flag);
  void SetPrevFeedbackData(const Feedback& data);
  void SetCurrentTarget(const Feedback& data);

  Feedback GetControllerData();
  void InitController();

  Pose ImpedanceCalculator(const Pose& reference, const Pose& flag);
  Pose AdmittanceCalculator(const Wrench& reference, const Wrench& flag);
  Pose ExternalForceSafetyCalculator(const Wrench& reference, const Joint& flag);

  void ResetSafetyController();

  static constexpr int MA_WINDOW = 500;  // N = 10 샘플 평균
  Wrench buf[MA_WINDOW];                 // 순환 버퍼
  int head = 0;                          // 다음에 쓸 위치
  int filled = 0;                        // 현재 채워진 샘플 수

  Wrench movingAverage(const Wrench& new_sample);
  void UpdateDataAnalysis(const bool is_debug);
};
}  // namespace plaif::moforcecontroller

#endif
