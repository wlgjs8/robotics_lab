#include "plaif/moforcecontroller/force_control.h"

#include <math.h>
// #include <plaif/util/logger.h>
#include <iostream>
#include <sstream>

namespace plaif::moforcecontroller {
ForceController::ForceController(const double* robot_frame)
    : cont_result{}, feedback_data{}, reference_data{}, controller_flag{}, measurement_data{}, imp_output_wrench{}, adm_output_pose{}, safety_output_pose{}, prev_feedback{}, current_target{} {
  this->Initialize(robot_frame);
}

void ForceController::Initialize(const double* robot_frame) {
  m_ptransformation = new Transformation();
  if (m_ptransformation) {
    m_ptransformation->Reset(robot_frame, nullptr);
  }

  n_pforcecontroller_adm = new CForceController(CForceController::Type::kEnduringReferenceForce);
  n_pforcecontroller_imp = new CForceController(CForceController::Type::kPoseErrorBasedForce);
  n_pforcecontroller_safe = new CForceController(CForceController::Type::kSafetedExternalForce);
}

void ForceController::InitController() {
  n_pforcecontroller_adm->InitController();
  n_pforcecontroller_imp->InitController();
  n_pforcecontroller_safe->InitController();

  imp_output_wrench = {};
  adm_output_pose = {};
  safety_output_pose = {};
}

void ForceController::ResetSafetyController() {
  if (n_pforcecontroller_safe) {
    n_pforcecontroller_safe->InitController();
  }
}

Pose ForceController::ImpedanceCalculator(const Pose& reference, const Pose& flag) {
  Feedback imp_measured_data = measurement_data;

  Feedback delta_tcp = {};
  Feedback delta_tcp_dot = {};
  n_pforcecontroller_imp->CalculateTCPError(feedback_data.pose, reference_data.pose, delta_tcp.pose, delta_tcp_dot.pose);
  imp_measured_data.pose = delta_tcp.pose;

  Feedback imp_reference_data = {};
  imp_reference_data.pose = reference;

  Feedback imp_feedback_data = feedback_data;

  std::array<double, 6> imp_controller_flag = {flag.x, flag.y, flag.z, flag.roll, flag.pitch, flag.yaw};

  n_pforcecontroller_imp->SetCurrentTarget(current_target);
  n_pforcecontroller_imp->SetControllerFlag(imp_controller_flag);
  n_pforcecontroller_imp->SetMeasurementData(imp_measured_data);
  n_pforcecontroller_imp->SetFeedbackData(imp_feedback_data);
  n_pforcecontroller_imp->SetReferenceData(imp_reference_data);

  Feedback imp_output = {};
  n_pforcecontroller_imp->PoseErrorBasedCreateForce();
  n_pforcecontroller_imp->GetCompensationData(imp_output);
  imp_output_wrench = imp_output.wrench;

  n_pforcecontroller_imp->SetReferenceData(imp_output);
  Feedback adm_output = {};
  Pose imp_output_wrench_ = {};

  n_pforcecontroller_imp->EnduringReferenceForceControlImp();
  n_pforcecontroller_imp->GetCompensationData(adm_output);

  imp_output_wrench_ = adm_output.pose * 1.0;

  return imp_output_wrench_;
}


Wrench ForceController::movingAverage(const Wrench& new_sample) {
  // 1) 새 샘플을 버퍼에 넣기
  buf[head] = new_sample;
  head = (head + 1) % MA_WINDOW;
  if (filled < MA_WINDOW) ++filled;

  // 2) 합산
  Wrench sum{};
  for (int i = 0; i < filled; ++i) {
    sum.force.x += buf[i].force.x;
    sum.force.y += buf[i].force.y;
    sum.force.z += buf[i].force.z;
    sum.torque.x += buf[i].torque.x;
    sum.torque.y += buf[i].torque.y;
    sum.torque.z += buf[i].torque.z;
  }

  // 3) 평균
  double inv = 1.0 / static_cast<double>(filled);
  sum.force.x *= inv;
  sum.force.y *= inv;
  sum.force.z *= inv;
  sum.torque.x *= inv;
  sum.torque.y *= inv;
  sum.torque.z *= inv;
  return sum;
}

Pose ForceController::AdmittanceCalculator(const Wrench& reference, const Wrench& flag) {
  Feedback controller_output_ = {};
  Feedback adm_measured_data = {};

  Feedback delta_tcp = {};
  Feedback delta_tcp_dot = {};
  n_pforcecontroller_adm->CalculateTCPError(feedback_data.pose, reference_data.pose, delta_tcp.pose, delta_tcp_dot.pose);
  adm_measured_data.pose = delta_tcp.pose;

  std::array<double, 6> adm_controller_flag = {flag.force.x, flag.force.y, flag.force.z, flag.torque.x, flag.torque.y, flag.torque.z};

  Feedback adm_reference_data = {};
  adm_reference_data.wrench = reference;

  Feedback adm_feedback_data = feedback_data;

  n_pforcecontroller_adm->SetCurrentTarget(current_target);
  n_pforcecontroller_adm->SetControllerFlag(adm_controller_flag);
  n_pforcecontroller_adm->SetMeasurementData(adm_measured_data);
  n_pforcecontroller_adm->SetFeedbackData(adm_feedback_data);
  n_pforcecontroller_adm->SetReferenceData(adm_reference_data);
  n_pforcecontroller_adm->EnduringReferenceForceControl();
  n_pforcecontroller_adm->GetCompensationData(controller_output_);

  adm_output_pose = controller_output_.pose;
  return adm_output_pose;
}

Pose ForceController::ExternalForceSafetyCalculator(const Wrench& reference, const Joint& flag) {
  Feedback controller_output_ = {};
  Feedback adm_measured_data = {};

  Feedback delta_tcp = {};
  Feedback delta_tcp_dot = {};
  n_pforcecontroller_safe->CalculateTCPError(feedback_data.pose, reference_data.pose, delta_tcp.pose, delta_tcp_dot.pose);
  adm_measured_data.pose = delta_tcp.pose;

  std::array<double, 6> adm_controller_flag = {flag.position[0], flag.position[1], flag.position[2], flag.position[3], flag.position[4], flag.position[5]};

  Feedback adm_reference_data = {};
  adm_reference_data.wrench = reference;

  Feedback adm_feedback_data = feedback_data;

  n_pforcecontroller_safe->SetCurrentTarget(current_target);
  n_pforcecontroller_safe->SetControllerFlag(adm_controller_flag);
  n_pforcecontroller_safe->SetMeasurementData(adm_measured_data);
  n_pforcecontroller_safe->SetFeedbackData(adm_feedback_data);
  n_pforcecontroller_safe->SetReferenceData(adm_reference_data);
  n_pforcecontroller_safe->SafetedExternalForceControl();
  n_pforcecontroller_safe->GetCompensationData(controller_output_);

  safety_output_pose = controller_output_.pose;

  return safety_output_pose;
}

void ForceController::Calculation() {}

void ForceController::SetFeedbackData(const Feedback& data) {
  feedback_data = data;
  // wrench : eef based - bias
  // pose   : world based
}



void ForceController::SetMeasurementData(const Feedback& data) {
  measurement_data = data;
  measurement_data.wrench = m_ptransformation->ToWorld(feedback_data.wrench, feedback_data.pose);
  // wrench : world based feedback
  // pose   : prev adm output pose(EEF based)
}

void ForceController::SetReferenceData(const Feedback& data) { reference_data = data; }

void ForceController::SetControllerFlag(const Feedback& flag) { controller_flag = flag; }

void ForceController::SetPrevFeedbackData(const Feedback& data) { prev_feedback = data; }

void ForceController::SetCurrentTarget(const Feedback& data) { current_target = data; }

Feedback ForceController::GetControllerData() { return cont_result; }
}  // namespace plaif::moforcecontroller
