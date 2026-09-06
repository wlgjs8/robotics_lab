#pragma once
// THE BOX DH CALIBRATION (2026-09-05). The Rainbow controller box keeps a calibrated
// Denavit-Hartenberg table for its own arm and answers `get_link_parameter()` with
// the calibrated cells. controller-manager adopts them at connect; this stack used
// the URDF nominal. Measured against the box's own TCP report (servo_log_20260904_
// 233101/235421.csv): URDF FK vs box FK differ by p50 0.6-1.2 mm and ~0.1 deg per
// arm. This header holds the table, the RB5-850E slot layout, the adoption rules
// and the URDF rewrite that carries the adopted table into every consumer (IK/FK,
// the collision monitor, the GUI) as ONE runtime URDF per arm.
#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace rb_servo {

// One row of the classic (distal) DH table in the box's units (deg / mm) - the same
// row controller-manager keeps (src/core/Config.h DhRow).
struct DhRow {
    double theta_offset_deg = 0.0;
    double d_mm = 0.0;
    double a_mm = 0.0;
    double alpha_deg = 0.0;
};
using DhTable = std::array<DhRow, 6>;

// The RB5-850E nominal table (controller-manager
// platforms/chimpanzee/params-presets/models/rb5-850e.yaml).
DhTable rb5850eNominalDh();

enum class DhField { ThetaOffset, D, A, Alpha };
struct DhSlot {
    int slot = 0;
    int joint = 1;      // 1-based
    DhField field = DhField::D;
};
struct DhSlotLayout {
    int count = 0;       // EXACT reply length; anything else is an unknown firmware layout
    std::vector<DhSlot> slots;
};
// get_link_parameter() on the RB5-850E: 8 values; slots 0-5 = J2.alpha J3.alpha
// J1.d J2.a J3.a J4.d (absolute cells, baseline 0); slots 6-7 zero.
const DhSlotLayout& rb5850eLinkParameterLayout();

// Numbers inside the first [...] of the reply (or after the key when there are no
// brackets). Empty when the reply carried none.
std::vector<double> parseLinkParameterPayload(const std::string& payload);

struct DhAdoption {
    DhTable table{};
    std::vector<double> raw;
    int changed_cells = 0;
    double max_abs_delta_mm = 0.0;
    double max_abs_delta_deg = 0.0;
    std::vector<std::string> skipped;   // "J1.d (box 163.944, kept nominal 169.2)"
};
// Cell names as "J<joint>.<field>" with field in {theta_offset, d, a, alpha}.
std::string dhCellName(int joint, DhField field);
bool parseDhCellName(const std::string& name, int* joint, DhField* field);
// Adopt a reply into a table. Returns an empty string on success, otherwise the
// refusal: wrong value count (an unknown layout - a prefix guess once wrote RB5 cells
// over an RB3 table, 600 mm of FK corruption), a nonzero UNMAPPED slot (an unknown
// calibration arrived), or a cell farther from nominal than the limits (a box that
// answered garbage). Nothing is adopted on refusal. Cells named in `skip_cells` are
// validated and logged but KEPT NOMINAL: the box may state a cell in a convention
// this URDF does not share (measured 2026-09-06: adopting J1.d moved our FK away
// from the box's own TCP report by exactly the J1.d delta, 5.3 mm).
std::string adoptLinkParameter(const std::vector<double>& raw, const DhTable& nominal,
                               const DhSlotLayout& layout, double max_abs_delta_mm,
                               double max_abs_delta_deg, DhAdoption* out,
                               const std::vector<std::string>& skip_cells = {});

// Rewrite the joint origins of ONE RB5-850E arm chain in a URDF text so the chain
// carries `calibrated` instead of `nominal`. This URDF keeps joint axes along +y and
// link offsets along +z, so the DH cells land as: J1.d -> base_joint z, J2.a ->
// elbow_joint z, J3.a -> wrist1_joint z, J4.d -> wrist2_joint y, J2.alpha ->
// elbow_joint yaw (rpy z), J3.alpha -> wrist1_joint yaw. Deltas only; a joint whose
// cells did not change is left byte-identical, so a nominal table returns the input
// unchanged. `joint_prefix` is "" for the single-arm file and the per-arm prefix of
// the unified dual URDF. Empty string = ok; `edits` = joints rewritten.
std::string patchUrdfWithDh(const std::string& urdf_text, const std::string& joint_prefix,
                            const DhTable& nominal, const DhTable& calibrated,
                            std::string* out_text, int* edits);

// Make every relative mesh `filename="..."` absolute against `urdf_dir`, so a copy
// written elsewhere still finds its meshes.
std::string absolutizeMeshPaths(const std::string& urdf_text, const std::string& urdf_dir);

std::uint64_t fnv1a64(const std::string& s);
std::string formatDhTable(const DhTable& t);
std::string formatDhDelta(const DhTable& nominal, const DhTable& calibrated);

}  // namespace rb_servo
