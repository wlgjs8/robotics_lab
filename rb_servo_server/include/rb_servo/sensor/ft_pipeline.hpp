// ft_pipeline.hpp - the F/T compensation pipeline, ported from controller-manager's
// `Arm::process_ft` (submodules/controller-manager/src/arm/Arm.cpp).
//
// CM IS THE CALIBRATION AUTHORITY for this cell: the sensor axis triad and the tool
// mass/CoM in our config are values the operator measured through CM's own
// procedures on this hardware. This module exists so the SAME numbers produce the
// SAME wrench here, which is the only way two stacks driving one arm can be compared.
//
// THE ORDER OF THE STAGES IS THE DESIGN, not an implementation detail:
//
//   (1) AXIS MAP ONLY.  Fs = [fx|fy|fz] * f_raw
//       The sensor's axes are given as basis COLUMNS in the flange frame. There is
//       NO origin shift here, so the torque stays referenced at the SENSING
//       REFERENCE ORIGIN. `sensor_offset` only relates the SRO to the flange.
//
//   (2) TOOL GRAVITY.   Fg = mass * (R_stand_flange^T * g_stand)
//                       Mg = com x Fg                       (com is SENSOR-frame)
//       The box is told a ZERO payload, so it subtracts NOTHING and the whole term
//       belongs here. Get this backwards and the compensation runs twice.
//
//   (3) BIAS.           Fc = Fs - bias_F - Fg
//       The bias is a SENSOR OFFSET, subtracted in the sensor frame before anything
//       rotates. A tare therefore averages `raw - gravity`, never `raw` (see
//       FtPipeline::tareSample).
//
//   (4) REFERENCE-POINT SHIFT, THEN ROTATE, THEN DEADZONE.
//       M_tcp = R(tool.rpy)^T * (Mc - r_tcp x Fc),  F_tcp = R(tool.rpy)^T * Fc
//       The deadzone is PER-AXIS, so it must be applied on the axes the consumer
//       reasons about - which is why it comes after the rotation and not before.
//
// A wrench without a frame AND a reference point is not a measurement. Every output
// of this module names both.
//
// NOT RT-HOSTILE: no allocation, no locks, no I/O. Safe on the 500 Hz loop.
#pragma once

#include <array>
#include <cstdint>
#include <string>

#include "rb_servo/config/config.hpp"
#include "rb_servo/core/types.hpp"
#include "rb_servo/math/se3.hpp"

namespace rb_servo {
namespace sensor {

// What one tick needs from the rest of the loop.
struct FtPipelineInput {
    // The raw sensor reading in the SENSOR'S OWN AXES (RobotState::eft_wrench).
    Wrench6D raw_sensor_axes{};
    bool raw_valid = false;
    // Rotation stand -> flange at THE CONFIGURATION THE WRENCH BELONGS TO. CM pins
    // this to the arm's current COMMAND, so the wrench compensation and the
    // correction it drives are evaluated in ONE configuration per cycle.
    math::Matrix3 r_stand_flange = math::Matrix3::Identity();
    bool kinematics_valid = false;
};

class FtPipeline {
public:
    FtPipeline() = default;

    // COLD. `arm_config` must outlive this object.
    void configure(const FtArmConfig& arm_config, double control_period_sec);

    // RT: run the four stages. Returns false and pins every compensated channel to
    // EXACT ZERO when the sensor is absent, the reading is invalid, or the
    // kinematics are not available — a consumer must never read a bias- or
    // gravity-fabricated force where nothing was measured.
    bool step(const FtPipelineInput& in);

    // ---- the surfaces this tick produced ------------------------------------
    // (1) RAW, axis-mapped. SENSOR frame (flange-aligned), torque about the SRO.
    const Wrench6D& rawSensor() const { return raw_sensor_; }
    // The tool-gravity term subtracted this tick. SENSOR frame @SRO.
    const Wrench6D& gravitySensor() const { return gravity_sensor_; }
    // (2) COMPENSATED, SENSOR frame @SRO, pre-deadzone / post-deadzone.
    const Wrench6D& compSensorNoDeadzone() const { return comp_sensor_nodz_; }
    const Wrench6D& compSensor() const { return comp_sensor_; }
    // (3) COMPENSATED, TCP reference point, TOOL axes, deadzoned. THE FORCE LAW'S
    //     INPUT — and the surface the compose pivot must be paired with.
    const Wrench6D& compTcp() const { return comp_tcp_; }
    // (4) the same wrench rotated into STAND axes, torque still about the TCP.
    const Wrench6D& compStand() const { return comp_stand_; }
    // (4') the STAND-axes wrench BEFORE the deadzone, torque about the TCP. For a
    //      consumer that filters the vector itself (the force gate's stream channel):
    //      a deadzone clips exactly the small values a slow filter is meant to keep.
    const Wrench6D& compStandNoDeadzone() const { return comp_stand_nodz_; }

    // The heavily low-passed STAND-frame force, as a magnitude and as a mass. Exists
    // to escape the deadzone, which flattens ~204 g to exactly zero at the shipped
    // 2 N/axis. The VECTOR is filtered and the norm taken afterwards: the norm is
    // rectified, so filtering it would leave the noise's positive bias as a DC term.
    double loadForceN() const { return load_force_n_; }
    double loadMassKg() const { return load_mass_kg_; }
    bool loadSettled() const { return load_settled_; }

    // ---- the bias (tare) -----------------------------------------------------
    const Wrench6D& bias() const { return bias_; }
    bool biasValid() const { return bias_valid_; }
    const std::string& biasSource() const { return bias_source_; }
    std::uint64_t biasGeneration() const { return bias_generation_; }

    // Feed ONE tare sample. Averages `raw - gravity`, NEVER `raw`: the box subtracts
    // no payload, so raw still contains the tool's weight, and averaging it would
    // fold the tare pose's gravity into the bias and then subtract gravity twice.
    // Only valid immediately after a step() that returned true.
    void tareSample();
    int tareSampleCount() const { return tare_count_; }
    void tareReset();
    // Commit the averaged samples as the bias. Refuses (returns false, `reason`
    // filled) below `min_samples`.
    bool tareCommit(int min_samples, std::string* reason);
    // Drop the bias and say it is gone. Used when a new zero has been REQUESTED but
    // not yet taken (auto-tare on InitMotion): from here forceControlCovered refuses
    // this arm until a tare is accepted, which is the point - the alternative is a
    // law regulating against a zero whose provenance nobody can state. Zeroes the
    // bias value too, so the compensated surface is `raw - gravity` and not
    // `raw - gravity - (a bias we just declared invalid)`.
    void invalidateBias();

    // ---- the sensor-presence verdict ----------------------------------------
    // Feed one liveness sample during the COLD window. A live RFT always jitters
    // above its noise floor; a stream flat across the window is unplugged.
    void livenessSample(const Wrench6D& raw_sensor_axes);
    bool livenessDecide();   // -> connected
    // Only for an externally stepped sensor whose sample sequence/time has been
    // verified by its transport. A noiseless simulated sensor cannot use RFT's
    // electrical-noise presence test. Does not grant a tare or validate a wrench.
    void setExternallyVerifiedConnection(bool connected);
    bool connected() const { return connected_; }
    const std::string& connectReason() const { return connect_reason_; }
    double livenessForcePpN() const { return force_pp_; }
    double livenessTorquePpNm() const { return torque_pp_; }

    // The determinant of the configured axis triad. -1 on this cell's RFT64, which
    // is CORRECT: the unit reports in a left-handed axis set.
    double axesDeterminant() const { return axes_det_; }

    // Fill the telemetry block for this tick.
    void fillTelemetry(FtTelemetry* out) const;

private:
    const FtArmConfig* cfg_ = nullptr;
    double dt_ = 0.002;

    math::Matrix3 axis_map_ = math::Matrix3::Identity();   // columns = [fx|fy|fz]
    math::Matrix3 r_flange_tool_ = math::Matrix3::Identity();
    math::Vector3 sensor_offset_m_ = math::Vector3::Zero();
    math::Vector3 tool_com_m_ = math::Vector3::Zero();
    math::Vector3 tcp_from_sro_m_ = math::Vector3::Zero();
    double axes_det_ = 1.0;

    Wrench6D raw_sensor_{};
    Wrench6D gravity_sensor_{};
    Wrench6D comp_sensor_nodz_{};
    Wrench6D comp_sensor_{};
    Wrench6D comp_tcp_{};
    Wrench6D comp_stand_{};

    Wrench6D comp_stand_nodz_{};
    Wrench6D bias_{};
    bool bias_valid_ = false;
    std::string bias_source_ = "none";
    std::uint64_t bias_generation_ = 0;

    math::Vector3 tare_force_sum_ = math::Vector3::Zero();
    math::Vector3 tare_torque_sum_ = math::Vector3::Zero();
    int tare_count_ = 0;

    math::Vector3 load_f_ = math::Vector3::Zero();
    bool load_seeded_ = false;
    std::uint32_t load_ticks_ = 0;
    bool load_settled_ = false;
    double load_force_n_ = 0.0;
    double load_mass_kg_ = 0.0;

    bool live_have_ = false;
    std::array<double, 3> fmin_{}, fmax_{}, mmin_{}, mmax_{};
    int live_samples_ = 0;
    double force_pp_ = 0.0;
    double torque_pp_ = 0.0;
    bool connected_ = false;
    std::string connect_reason_ = "not checked";
};

}  // namespace sensor
}  // namespace rb_servo
