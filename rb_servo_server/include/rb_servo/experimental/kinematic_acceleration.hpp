#pragma once

#include <Eigen/Core>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>

namespace rb_servo::experimental {

// Three received position records with their timestamps. Packet identity removes
// repeated worker-cache samples but does NOT prove synchronous/fresh encoders in
// the payload. This baseline fails the staggered-readback audit; do not deploy it
// as an inertia observer. The quadratic estimate is centered in the sample window,
// not a zero-delay acceleration measurement or a force-signal low-pass filter.
class KinematicAcceleration {
public:
    void clear() { count_=0; sequence_=0; acceleration_.reset(); }
    const std::optional<Eigen::Vector3d>& value() const { return acceleration_; }
    bool update(std::uint64_t sequence, std::uint64_t stamp_ns,
                const Eigen::Vector3d& position, double max_gap_sec) {
        if (!sequence || !stamp_ns || !position.allFinite() ||
            !std::isfinite(max_gap_sec) || !(max_gap_sec>0)) {
            clear(); return false;
        }
        if (sequence==sequence_) return acceleration_.has_value();
        if (count_ && (sequence<sequence_ || stamp_ns<=samples_[count_-1].stamp ||
            (stamp_ns-samples_[count_-1].stamp)*1e-9>max_gap_sec)) clear();
        sequence_=sequence;
        if (count_==samples_.size()) { samples_[0]=samples_[1];samples_[1]=samples_[2];--count_; }
        samples_[count_++]={stamp_ns,position};
        if (count_<3) return false;
        const double a=(samples_[1].stamp-samples_[0].stamp)*1e-9;
        const double b=(samples_[2].stamp-samples_[1].stamp)*1e-9;
        const Eigen::Vector3d result=2.0*((samples_[2].position-samples_[1].position)/b-
            (samples_[1].position-samples_[0].position)/a)/(a+b);
        if (!result.allFinite()) { clear();return false; }
        acceleration_=result;
        return true;
    }
private:
    struct Sample { std::uint64_t stamp{0}; Eigen::Vector3d position{Eigen::Vector3d::Zero()}; };
    std::array<Sample,3> samples_{};
    std::size_t count_{0};
    std::uint64_t sequence_{0};
    std::optional<Eigen::Vector3d> acceleration_;
};

} // namespace rb_servo::experimental
