#pragma once
// THE BOX DH CALIBRATION STAGE (2026-09-05): runs ONCE at `make run`, before the
// backends, the kinematics and the collision monitor exist. Probes each controller
// box for its calibrated DH (get_link_parameter), adopts it (dh_calibration.hpp),
// writes the calibrated runtime URDFs (per-arm single chain for IK/FK, the unified
// dual for the collision monitor and the GUI), and rewrites the in-memory config so
// every consumer is built from those files. real: kinematics.calibration.source=box
// -> any failure is FATAL (the server exits; run_stack stops). sim: source=nominal
// -> a no-op. The backends re-read the box at connect and refuse to connect when
// the values differ from what was adopted here (BackendConfig::expected_link_parameter).
#include "rb_servo/config/config.hpp"
#include "rb_servo/kinematics/dh_calibration.hpp"

#include <array>
#include <string>
#include <vector>

namespace rb_servo {

struct BoxDhCalibrationResult {
    bool ok = false;
    bool applied = false;             // false when source == nominal
    std::string error;
    std::array<std::vector<double>, 2> raw{};
    std::array<DhTable, 2> table{};
    std::string urdf_left;
    std::string urdf_right;
    std::string urdf_dual;
    std::string manifest_path;
};

BoxDhCalibrationResult runBoxDhCalibration(DualArmConfig& config);

}  // namespace rb_servo
