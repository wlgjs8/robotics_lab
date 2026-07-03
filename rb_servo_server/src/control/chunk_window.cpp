#include "rb_servo/control/chunk_window.hpp"

#include "rb_servo/math/se3.hpp"

#include <algorithm>
#include <cmath>

namespace rb_servo::control {

namespace {

// Centered orientation average over a window, done on the SO(3) tangent about
// the window-center rotation (small-angle log/exp average). Keeps the smoothed
// orientation on the manifold instead of averaging raw quaternion components.
Eigen::Quaterniond smoothOrientation(const std::vector<Pose6D>& src,
                                     std::size_t center, int half) {
  const int n = static_cast<int>(src.size());
  const Eigen::Matrix3d R_ref = math::rotationFromPose(src[center]);
  const Eigen::Quaterniond q_ref(R_ref);
  math::Vector3 acc = math::Vector3::Zero();
  int count = 0;
  for (int off = -half; off <= half; ++off) {
    const int idx = std::clamp(static_cast<int>(center) + off, 0, n - 1);
    const Eigen::Matrix3d R_i = math::rotationFromPose(src[idx]);
    acc += math::log3(R_ref.transpose() * R_i);
    ++count;
  }
  acc /= std::max(count, 1);
  const Eigen::Matrix3d R_out = R_ref * math::exp3(acc);
  return Eigen::Quaterniond(R_out).normalized();
}

}  // namespace

bool ChunkWindow::activate(const ChunkFrame& frame) {
  const std::size_t n = frame.pose.size();
  const int L = std::max(0, cfg_.discard_head_L);
  const int R = std::max(1, cfg_.reserve_R);  // central diff needs >=1

  // Hard invariant L + C + R <= horizon: clamp C, never over-read.
  const int c_room = static_cast<int>(n) - L - R;
  const int c_eff = std::min(cfg_.consume_C, c_room);
  if (c_eff < 1) {
    active_ = false;
    return false;  // frame too short for even one consumable step with reserve
  }

  // --- smooth (position componentwise + orientation on tangent) ---
  pose_.assign(frame.pose.begin(), frame.pose.end());
  const int half = std::max(0, (cfg_.smoothing_window - 1) / 2);
  if (half > 0 && n >= 3) {
    std::vector<Pose6D> out = pose_;
    for (std::size_t i = 0; i < n; ++i) {
      double sx = 0, sy = 0, sz = 0;
      int cnt = 0;
      for (int off = -half; off <= half; ++off) {
        const int idx =
            std::clamp(static_cast<int>(i) + off, 0, static_cast<int>(n) - 1);
        sx += frame.pose[idx].x;
        sy += frame.pose[idx].y;
        sz += frame.pose[idx].z;
        ++cnt;
      }
      out[i].x = sx / cnt;
      out[i].y = sy / cnt;
      out[i].z = sz / cnt;
      const Eigen::Quaterniond q = smoothOrientation(frame.pose, i, half);
      out[i].quaternion_xyzw = math::quaternionToXyzw(q);
    }
    pose_.swap(out);
  } else {
    // ensure canonical quaternion is populated even without smoothing
    for (Pose6D& p : pose_) {
      if (!p.quaternion_xyzw.has_value()) {
        p.quaternion_xyzw = math::quaternionToXyzw(
            Eigen::Quaterniond(math::rotationFromPose(p)));
      }
    }
  }

  grip_ = frame.grip;
  grip_.resize(n, grip_.empty() ? 0.0 : grip_.back());
  dt_ = frame.policy_dt > 1e-6 ? frame.policy_dt : (1.0 / 30.0);
  seq_ = frame.seq;
  recv_ = frame.recv_time;
  k_ = static_cast<std::size_t>(L);
  consumed_ = 0;
  consume_eff_ = c_eff;
  active_ = true;
  return true;
}

bool ChunkWindow::hasStep() const {
  if (!active_) return false;
  if (consumed_ >= consume_eff_) return false;
  return k_ + 1 < pose_.size();  // real forward neighbor for central diff
}

const Pose6D& ChunkWindow::poseAt(std::size_t k) const {
  const std::size_t idx =
      pose_.empty() ? 0 : std::min(k, pose_.size() - 1);
  return pose_[idx];
}

double ChunkWindow::gripAt(std::size_t k) const {
  if (grip_.empty()) return 0.0;
  return grip_[std::min(k, grip_.size() - 1)];
}

}  // namespace rb_servo::control
